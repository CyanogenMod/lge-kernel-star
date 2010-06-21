/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nvcommon.h"
#include "nvrm_clocks.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_boot.h"
#include "nvbootargs.h"
#include "nvrm_memmgr.h"
#include "ap15/ap15rm_private.h"
#include "ap15/project_relocation_table.h"


#define NvRmPrivGetStepMV(hRmDevice, step) \
         (s_ChipFlavor.pSocShmoo->ShmooVoltages[(step)])

// Extended clock limits IDs
typedef enum
{
    // Last Module ID
    NvRmClkLimitsExtID_LastModuleID = NvRmPrivModuleID_Num,

    // Extended ID for display A pixel clock limits
    NvRmClkLimitsExtID_DisplayA,

    // Extended ID for display B pixel clock limits
    NvRmClkLimitsExtID_DisplayB,

    // Extended ID for CAR clock sources limits
    NvRmClkLimitsExtID_ClkSrc,

    NvRmClkLimitsExtID_Num,
    NvRmClkLimitsExtID_Force32 = 0x7FFFFFFF,
} NvRmClkLimitsExtID;

/*
 * Module clocks frequency limits table ordered by s/w module ids.
 * Display is a special case and has 3 entries associated:
 * - one entry that corresponds to display ID specifies pixel clock limit used
 * for CAR clock sources configuration; it is retrieved by RM clock manager
 * via private interface (same limit for both CAR display clock selectors);
 * - two entries appended at the end of the table specify pixel clock limits
 * for two display heads used for DDK clock configuration, these limits will
 * be retrieved by DDK via public interface
 * Also appended at the end of the table limits for clock sources (PLLs) forced
 * by CAR clock dividers
 */
static NvRmModuleClockLimits s_ClockRangeLimits[NvRmClkLimitsExtID_Num];

// Translation table for module clock limits scaled with voltage
static const NvRmFreqKHz* s_pClockScales[NvRmClkLimitsExtID_Num];

// Reference counts of clocks that require the respective core voltage to run
// (appended with pending voltage change reference count)
static NvU32 s_VoltageStepRefCounts[NVRM_VOLTAGE_STEPS + 1];
static NvU32 s_VoltagePendingMv = 0;
#define NVRM_VOLTAGE_PENDING_STEP (NVRM_VOLTAGE_STEPS)

// Chip shmoo data records
static NvRmChipFlavor s_ChipFlavor;
static NvRmSocShmoo s_SocShmoo;
static NvRmCpuShmoo s_CpuShmoo;
static void* s_pShmooData = NULL;

static NvError
NvRmBootArgChipShmooGet(
    NvRmDeviceHandle hRmDevice,
    NvRmChipFlavor* pChipFlavor);

static void NvRmPrivChipFlavorInit(NvRmDeviceHandle hRmDevice);
static void NvRmPrivChipFlavorInit(NvRmDeviceHandle hRmDevice)
{
    NvOsMemset((void*)&s_ChipFlavor, 0, sizeof(s_ChipFlavor));

    if (NvRmPrivChipShmooDataInit(hRmDevice, &s_ChipFlavor) == NvSuccess)
    {
        NvOsDebugPrintf("NVRM Initialized shmoo database\n");
        return;
    }
    if (NvRmBootArgChipShmooGet(hRmDevice, &s_ChipFlavor) == NvSuccess)
    {
        NvOsDebugPrintf("NVRM Got shmoo boot argument (at 0x%x)\n",
                        ((NvUPtr)s_pShmooData));
        return;
    }
    NV_ASSERT(!"Failed to set clock limits");
}

const NvRmModuleClockLimits*
NvRmPrivClockLimitsInit(NvRmDeviceHandle hRmDevice)
{
    NvU32 i;
    NvRmFreqKHz CpuMaxKHz, AvpMaxKHz, VdeMaxKHz, TDMaxKHz, DispMaxKHz;
    const NvRmSKUedLimits* pSKUedLimits;
    const NvRmScaledClkLimits* pHwLimits;
    const NvRmSocShmoo* pShmoo;

    NV_ASSERT(hRmDevice);
    NvRmPrivChipFlavorInit(hRmDevice);
    pShmoo = s_ChipFlavor.pSocShmoo;
    pHwLimits = &pShmoo->ScaledLimitsList[0];
    pSKUedLimits = pShmoo->pSKUedLimits;
    NvOsDebugPrintf("NVRM corner (%d, %d)\n",
        s_ChipFlavor.corner, s_ChipFlavor.CpuCorner);

    NvOsMemset((void*)s_pClockScales, 0, sizeof(s_pClockScales));
    NvOsMemset(s_ClockRangeLimits, 0, sizeof(s_ClockRangeLimits));
    NvOsMemset(s_VoltageStepRefCounts, 0, sizeof(s_VoltageStepRefCounts));
    s_VoltageStepRefCounts[0] = NvRmPrivModuleID_Num; // all at minimum step

    // Combine AVP/System clock absolute limit with scaling V/F ladder upper
    // boundary, and set default clock range for all present modules the same
    // as for AVP/System clock
    AvpMaxKHz = pSKUedLimits->AvpMaxKHz;
    for (i = 0; i < pShmoo->ScaledLimitsListSize; i++)
    {
        if (pHwLimits[i].HwDeviceId == NV_DEVID_AVP)
        {
            AvpMaxKHz = NV_MIN(
                AvpMaxKHz, pHwLimits[i].MaxKHzList[pShmoo->ShmooVmaxIndex]);
            break;
        }
    }

    for (i = 0; i < NvRmPrivModuleID_Num; i++)
    {
        NvRmModuleInstance *inst;
        if (NvRmPrivGetModuleInstance(hRmDevice, i, &inst) == NvSuccess)
        {
            s_ClockRangeLimits[i].MaxKHz = AvpMaxKHz;
            s_ClockRangeLimits[i].MinKHz = NVRM_BUS_MIN_KHZ;

        }
    }

    // Fill in limits for modules with slectable clock sources and/or dividers
    // as specified by the h/w table according to the h/w device ID
    // (CPU and AVP are not in relocation table - need translate id explicitly)
    // TODO: need separate subclock limits? (current implementation applies
    // main clock limits to all subclocks)
    for (i = 0; i < pShmoo->ScaledLimitsListSize; i++)
    {
        NvRmModuleID id;
        if (pHwLimits[i].HwDeviceId == NV_DEVID_CPU)
            id = NvRmModuleID_Cpu;
        else if (pHwLimits[i].HwDeviceId == NV_DEVID_AVP)
            id = NvRmModuleID_Avp;
        else if (pHwLimits[i].HwDeviceId == NVRM_DEVID_CLK_SRC)
            id = NvRmClkLimitsExtID_ClkSrc;
        else
            id = NvRmPrivDevToModuleID(pHwLimits[i].HwDeviceId);
        if ((id != NVRM_DEVICE_UNKNOWN) &&
            (pHwLimits[i].SubClockId == 0))
        {
            s_ClockRangeLimits[id].MinKHz = pHwLimits[i].MinKHz;
            s_ClockRangeLimits[id].MaxKHz =
                pHwLimits[i].MaxKHzList[pShmoo->ShmooVmaxIndex];
            s_pClockScales[id] = pHwLimits[i].MaxKHzList;
        }
    }
    // Fill in CPU scaling data if SoC has dedicated CPU rail, and CPU clock
    // characterization data is separated from other modules on common core rail
    if (s_ChipFlavor.pCpuShmoo)
    {
        const NvRmScaledClkLimits* pCpuLimits =
            s_ChipFlavor.pCpuShmoo->pScaledCpuLimits;
        NV_ASSERT(pCpuLimits && (pCpuLimits->HwDeviceId == NV_DEVID_CPU));

        s_ClockRangeLimits[NvRmModuleID_Cpu].MinKHz = pCpuLimits->MinKHz;
        s_ClockRangeLimits[NvRmModuleID_Cpu].MaxKHz =
            pCpuLimits->MaxKHzList[s_ChipFlavor.pCpuShmoo->ShmooVmaxIndex];
        s_pClockScales[NvRmModuleID_Cpu] = pCpuLimits->MaxKHzList;
    }

    // Set AVP upper clock boundary with combined Absolute/Scaled limit;
    // Sync System clock with AVP (System is not in relocation table)
    s_ClockRangeLimits[NvRmModuleID_Avp].MaxKHz = AvpMaxKHz;
    s_ClockRangeLimits[NvRmPrivModuleID_System].MaxKHz =
        s_ClockRangeLimits[NvRmModuleID_Avp].MaxKHz;
    s_ClockRangeLimits[NvRmPrivModuleID_System].MinKHz =
        s_ClockRangeLimits[NvRmModuleID_Avp].MinKHz;
    s_pClockScales[NvRmPrivModuleID_System] = s_pClockScales[NvRmModuleID_Avp];

    // Set VDE upper clock boundary with combined Absolute/Scaled limit (on
    // AP15/Ap16 VDE clock derived from the system bus, and VDE maximum limit
    // must be the same as AVP/System).
    VdeMaxKHz = pSKUedLimits->VdeMaxKHz;
    VdeMaxKHz = NV_MIN(
        VdeMaxKHz, s_ClockRangeLimits[NvRmModuleID_Vde].MaxKHz);
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
    {
        NV_ASSERT(VdeMaxKHz == AvpMaxKHz);
    }
    s_ClockRangeLimits[NvRmModuleID_Vde].MaxKHz = VdeMaxKHz;

    // Set upper clock boundaries for devices on CPU bus (CPU, Mselect,
    // CMC) with combined Absolute/Scaled limits
    CpuMaxKHz = pSKUedLimits->CpuMaxKHz;
    CpuMaxKHz = NV_MIN(
        CpuMaxKHz, s_ClockRangeLimits[NvRmModuleID_Cpu].MaxKHz);
    s_ClockRangeLimits[NvRmModuleID_Cpu].MaxKHz = CpuMaxKHz;
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
    {
        s_ClockRangeLimits[NvRmModuleID_CacheMemCtrl].MaxKHz = CpuMaxKHz;
        s_ClockRangeLimits[NvRmPrivModuleID_Mselect].MaxKHz = CpuMaxKHz;
        NV_ASSERT(s_ClockRangeLimits[NvRmClkLimitsExtID_ClkSrc].MaxKHz >=
                  CpuMaxKHz);
    }
    else if (hRmDevice->ChipId.Id == 0x20)
    {
        // No CMC; TODO: Mselect/CPU <= 1/4?
        s_ClockRangeLimits[NvRmPrivModuleID_Mselect].MaxKHz = CpuMaxKHz >> 2;
    }
    else
    {
        NV_ASSERT(!"Unsupported chip ID");
    }

    // Fill in memory controllers absolute range (scaled data is on ODM level)
    s_ClockRangeLimits[NvRmPrivModuleID_MemoryController].MaxKHz =
        pSKUedLimits->McMaxKHz;
    s_ClockRangeLimits[NvRmPrivModuleID_ExternalMemoryController].MaxKHz =
        pSKUedLimits->Emc2xMaxKHz;
    s_ClockRangeLimits[NvRmPrivModuleID_ExternalMemoryController].MinKHz =
        NVRM_SDRAM_MIN_KHZ * 2;
    s_ClockRangeLimits[NvRmPrivModuleID_ExternalMemory].MaxKHz =
        pSKUedLimits->Emc2xMaxKHz / 2;
    s_ClockRangeLimits[NvRmPrivModuleID_ExternalMemory].MinKHz =
        NVRM_SDRAM_MIN_KHZ;

    // Set 3D upper clock boundary with combined Absolute/Scaled limit.
    TDMaxKHz = pSKUedLimits->TDMaxKHz;
    TDMaxKHz = NV_MIN(
        TDMaxKHz, s_ClockRangeLimits[NvRmModuleID_3D].MaxKHz);
    s_ClockRangeLimits[NvRmModuleID_3D].MaxKHz = TDMaxKHz;

    // Set Display upper clock boundary with combined Absolute/Scaled limit.
    // (fill in clock limits for both display heads)
    DispMaxKHz = NV_MAX(pSKUedLimits->DisplayAPixelMaxKHz,
                        pSKUedLimits->DisplayBPixelMaxKHz);
    DispMaxKHz = NV_MIN(
        DispMaxKHz, s_ClockRangeLimits[NvRmModuleID_Display].MaxKHz);
    s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayA].MaxKHz =
        NV_MIN(DispMaxKHz, pSKUedLimits->DisplayAPixelMaxKHz);
    s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayA].MinKHz =
            s_ClockRangeLimits[NvRmModuleID_Display].MinKHz;
    s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayB].MaxKHz =
        NV_MIN(DispMaxKHz, pSKUedLimits->DisplayBPixelMaxKHz);
    s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayB].MinKHz =
            s_ClockRangeLimits[NvRmModuleID_Display].MinKHz;

    return s_ClockRangeLimits;
}

NvRmFreqKHz
NvRmPowerModuleGetMaxFrequency(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleId)
{
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE(ModuleId);
    NvRmModuleID Module = NVRM_MODULE_ID_MODULE(ModuleId);
    NV_ASSERT(Module < NvRmPrivModuleID_Num);
    NV_ASSERT(hRmDevice);

    // For all modules, except display, ignore instance, and return
    // max frequency for the clock generated from CAR dividers
    if (Module != NvRmModuleID_Display)
        return s_ClockRangeLimits[Module].MaxKHz;

    // For display return pixel clock for the respective head
    if (Instance == 0)
        return s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayA].MaxKHz;
    else if (Instance == 1)
        return s_ClockRangeLimits[NvRmClkLimitsExtID_DisplayB].MaxKHz;
    else
    {
        NV_ASSERT(!"Inavlid display instance");
        return 0;
    }
}

NvRmMilliVolts
NvRmPrivGetNominalMV(NvRmDeviceHandle hRmDevice)
{
    const NvRmSocShmoo* p = s_ChipFlavor.pSocShmoo;
    return p->ShmooVoltages[p->ShmooVmaxIndex];
}

void
NvRmPrivGetSvopParameters(
    NvRmDeviceHandle hRmDevice,
    NvRmMilliVolts* pSvopLowMv,
    NvU32* pSvopLvSetting,
    NvU32* pSvopHvSetting)
{
    const NvRmSocShmoo* p = s_ChipFlavor.pSocShmoo;

    NV_ASSERT(pSvopLowMv && pSvopLvSetting && pSvopHvSetting);
    *pSvopLowMv = p->SvopLowVoltage;
    *pSvopLvSetting = p->SvopLowSetting;
    *pSvopHvSetting = p->SvopHighSetting;
}

NvRmMilliVolts
NvRmPrivSourceVscaleGetMV(NvRmDeviceHandle hRmDevice, NvRmFreqKHz FreqKHz)
{
    NvU32 i;
    const NvU32* pScaleSrc = s_pClockScales[NvRmClkLimitsExtID_ClkSrc];

    for (i = 0; i < s_ChipFlavor.pSocShmoo->ShmooVmaxIndex; i++)
    {
        if (FreqKHz <= pScaleSrc[i])
            break;
    }
    return NvRmPrivGetStepMV(hRmDevice, i);
}

NvRmMilliVolts
NvRmPrivModulesGetOperationalMV(NvRmDeviceHandle hRmDevice)
{
    NvU32 i;
    NV_ASSERT(hRmDevice);

    for (i = s_ChipFlavor.pSocShmoo->ShmooVmaxIndex; i != 0; i--)
    {
        if (s_VoltageStepRefCounts[i])
            break;
    }
    return NV_MAX(NvRmPrivGetStepMV(hRmDevice, i), s_VoltagePendingMv);
}

NvRmMilliVolts
NvRmPrivModuleVscaleGetMV(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module,
    NvRmFreqKHz FreqKHz)
{
    NvU32 i;
    const NvRmFreqKHz* pScale;
    NV_ASSERT(hRmDevice);
    NV_ASSERT(Module < NvRmPrivModuleID_Num);

    // If no scaling for this module - exit
    pScale = s_pClockScales[Module];
    if(!pScale)
        return NvRmPrivGetStepMV(hRmDevice, 0);

    // Find voltage step for the requested frequency, and convert it to MV
    // Use CPU specific voltage ladder if SoC has dedicated CPU rail
    if (s_ChipFlavor.pCpuShmoo && (Module == NvRmModuleID_Cpu))
    {
        for (i = 0; i < s_ChipFlavor.pCpuShmoo->ShmooVmaxIndex; i++)
        {
            if (FreqKHz <= pScale[i])
                break;
        }
        return s_ChipFlavor.pCpuShmoo->ShmooVoltages[i];
    }
    // Use common ladder for all other modules or CPU on core rail
    for (i = 0; i < s_ChipFlavor.pSocShmoo->ShmooVmaxIndex; i++)
    {
        if (FreqKHz <= pScale[i])
            break;
    }
    return NvRmPrivGetStepMV(hRmDevice, i);
}

const NvRmFreqKHz*
NvRmPrivModuleVscaleGetMaxKHzList(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module,
    NvU32* pListSize)
{
    NV_ASSERT(hRmDevice);
    NV_ASSERT(pListSize && (Module < NvRmPrivModuleID_Num));

    // Use CPU specific voltage ladder if SoC has dedicated CPU rail
    if (s_ChipFlavor.pCpuShmoo && (Module == NvRmModuleID_Cpu))
        *pListSize = s_ChipFlavor.pCpuShmoo->ShmooVmaxIndex + 1;
    else
        *pListSize = s_ChipFlavor.pSocShmoo->ShmooVmaxIndex + 1;

    return s_pClockScales[Module];
}

NvRmMilliVolts
NvRmPrivModuleVscaleAttach(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* pCinfo,
    NvRmModuleClockState* pCstate,
    NvBool Enable,
    NvBool Preview)
{
    NvBool Enabled;
    NvU32 reg, vstep1, vstep2, VstepMax;
    NvRmMilliVolts VoltageRequirement = NvRmVoltsUnspecified;
    NvBool CheckSubclock = ((pCinfo->Module == NvRmModuleID_Spdif) ||
                            (pCinfo->Module == NvRmModuleID_Vi) ||
                            (pCinfo->Module == NvRmModuleID_Tvo));

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo && pCstate);

    // If no scaling for this module - exit
    if (!pCstate->Vscale)
        return VoltageRequirement;

    //Check changes in clock status - exit if none (if clock is already
    // enabled || if clock still enabled => if enabled)
    NV_ASSERT(pCinfo->ClkEnableOffset);
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
        pCinfo->ClkEnableOffset);
    Enabled = ((reg & pCinfo->ClkEnableField) == pCinfo->ClkEnableField);
    if (Enabled)
        return VoltageRequirement;

    // Find maximum operational voltage step for all already attached modules,
    // and voltage steps for this module including subclock if any (subclock
    // state is located immediately after main one)
    for (VstepMax = s_ChipFlavor.pSocShmoo->ShmooVmaxIndex;
          VstepMax != 0; VstepMax--)
    {
        if (s_VoltageStepRefCounts[VstepMax])
            break;
    }
    vstep1 = pCstate->Vstep;
    if (CheckSubclock)
        vstep2 = pCstate[1].Vstep;
    else
        vstep2 = 0;

    // Specify new required voltage if module clock is to be enabled and need
    // voltage increase, leave requirements unspecified if current operational
    // voltage is enough, return "Off" indicator if module is to be disabled.
    if (Enable)
    {
        if (VstepMax < NV_MAX(vstep1, vstep2))
        {
            VstepMax = NV_MAX(vstep1, vstep2);
            VoltageRequirement = NvRmPrivGetStepMV(hRmDevice, VstepMax);

            // If preview and voltage increase - return without count update
            if (Preview)
                return VoltageRequirement;
        }
    }
    else
    {
        VoltageRequirement = NvRmVoltsOff;
    }

    // Update ref counts for module clock and subclock if any
    if (Enable)
    {
        s_VoltageStepRefCounts[vstep1]++;
        if ((pCinfo->Module == NvRmModuleID_Usb2Otg) &&
            (hRmDevice->ChipId.Id == 0x16))
        {
            // Two AP16 USB modules share clock enable control
            s_VoltageStepRefCounts[vstep1]++;
        }
    }
    else
    {
        NV_ASSERT(s_VoltageStepRefCounts[vstep1]);
        s_VoltageStepRefCounts[vstep1]--;
        if ((pCinfo->Module == NvRmModuleID_Usb2Otg) &&
            (hRmDevice->ChipId.Id == 0x16))
        {
            // Two AP16 USB modules share clock enable control
            NV_ASSERT(s_VoltageStepRefCounts[vstep1]);
            s_VoltageStepRefCounts[vstep1]--;
        }
    }
    if (CheckSubclock)
    {
        if (Enable)
        {
            s_VoltageStepRefCounts[vstep2]++;
        }
        else
        {
            NV_ASSERT(s_VoltageStepRefCounts[vstep2]);
            s_VoltageStepRefCounts[vstep2]--;
        }
    }
    return VoltageRequirement;
}


NvRmMilliVolts
NvRmPrivModuleVscaleReAttach(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* pCinfo,
    NvRmModuleClockState* pCstate,
    NvRmFreqKHz TargetModuleKHz,
    NvRmFreqKHz TargetSrcKHz,
    NvBool Preview)
{
    NvU32 i, j, reg, VstepMax;
    const NvRmFreqKHz* pScale;
    NvRmFreqKHz FreqKHz;
    NvRmMilliVolts VoltageRequirement = NvRmVoltsUnspecified;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo && pCstate);

    // No scaling for this module - exit
    if (!pCstate->Vscale)
        return VoltageRequirement;

    // Find current maximum module voltage step
    for (VstepMax = s_ChipFlavor.pSocShmoo->ShmooVmaxIndex;
          VstepMax != 0; VstepMax--)
    {
        if (s_VoltageStepRefCounts[VstepMax])
            break;
    }

    // Clip target frequency to module clock limits and find voltage step for
    // running at target frequency
    FreqKHz = s_ClockRangeLimits[pCinfo->Module].MinKHz;
    FreqKHz = NV_MAX(FreqKHz, TargetModuleKHz);
    if (FreqKHz > s_ClockRangeLimits[pCinfo->Module].MaxKHz)
        FreqKHz = s_ClockRangeLimits[pCinfo->Module].MaxKHz;

    pScale = s_pClockScales[pCinfo->Module];
    NV_ASSERT(pScale);
    for (i = 0; i < s_ChipFlavor.pSocShmoo->ShmooVmaxIndex; i++)
    {
        if (FreqKHz <= pScale[i])
            break;
    }

    // Find voltage step for using the target source, and select maximum
    // step required for both module and its source to operate
    pScale = s_pClockScales[NvRmClkLimitsExtID_ClkSrc];
    NV_ASSERT(pScale);
    for (j = 0; j < s_ChipFlavor.pSocShmoo->ShmooVmaxIndex; j++)
    {
        if (TargetSrcKHz <= pScale[j])
            break;
    }
    i = NV_MAX(i, j);

    // If voltage step has changed, always update module state, and update
    // ref count provided module clock is enabled
    if (pCstate->Vstep != i)
    {
        NV_ASSERT(pCinfo->ClkEnableOffset);
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->ClkEnableOffset);
        if ((reg & pCinfo->ClkEnableField) == pCinfo->ClkEnableField)
        {
            // Specify new required voltage if module clock is enabled and need
            // voltage increase, return "Off" indicator if module operational
            // volatge is going down; otherwise leave requirements unspecified
            if (i > VstepMax)
            {
                VoltageRequirement = NvRmPrivGetStepMV(hRmDevice, i);

                // If preview and voltage increase - return without count update
                if (Preview)
                    return VoltageRequirement;
            }
            else if ((i < pCstate->Vstep) && (VstepMax == pCstate->Vstep))
            {
                VoltageRequirement = NvRmVoltsOff;
            }

            // Update ref counts
            NV_ASSERT(s_VoltageStepRefCounts[pCstate->Vstep]);
            s_VoltageStepRefCounts[pCstate->Vstep]--;
            s_VoltageStepRefCounts[i]++;
        }
        pCstate->Vstep = i;
    }
    return VoltageRequirement;
}

void NvRmPrivModuleVscaleSetPending(
    NvRmDeviceHandle hRmDevice,
    NvRmMilliVolts PendingMv)
{
    if (PendingMv != NvRmVoltsOff)
    {
        s_VoltageStepRefCounts[NVRM_VOLTAGE_PENDING_STEP]++;
        s_VoltagePendingMv = NV_MAX(s_VoltagePendingMv, PendingMv);

    }
    else
    {
        NV_ASSERT(s_VoltageStepRefCounts[NVRM_VOLTAGE_PENDING_STEP]);
        if (s_VoltageStepRefCounts[NVRM_VOLTAGE_PENDING_STEP])
            s_VoltageStepRefCounts[NVRM_VOLTAGE_PENDING_STEP]--;

        if (s_VoltageStepRefCounts[NVRM_VOLTAGE_PENDING_STEP] == 0)
            s_VoltagePendingMv = NvRmVoltsOff;
    }
}

void
NvRmPrivModuleSetScalingAttribute(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* pCinfo,
    NvRmModuleClockState* pCstate)
{
    const NvRmFreqKHz* pScale;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo && pCstate);

    // Voltage scaling for free running core clocks is done by DFS
    // independently from module clock control. Therefore modules
    // that have core clock as a source do not have their own v-scale
    // attribute set
    switch (pCinfo->Sources[0])
    {
        case NvRmClockSource_CpuBus:
        case NvRmClockSource_SystemBus:
        case NvRmClockSource_Ahb:
        case NvRmClockSource_Apb:
        case NvRmClockSource_Vbus:
            pCstate->Vscale = NV_FALSE;
            return;
        default:
            break;
    }

    // Memory controller scale is specified separately on ODM layer, as
    // it is board dependent; TVDAC scaling always follow CVE (TV) or
    // display (CRT); PMU transport must work at any volatge - no
    // v-scale attribute for these modules
    switch (pCinfo->Module)
    {
        case NvRmModuleID_Dvc:  // TOD0: check PMU transport with ODM DB
        case NvRmPrivModuleID_MemoryController:
        case NvRmPrivModuleID_ExternalMemoryController:
            pCstate->Vscale = NV_FALSE;
            return;
        case NvRmModuleID_Tvo:
            if (pCinfo->SubClockId == 2)
            {   // TVDAC is TVO subclock 2
                pCstate->Vscale = NV_FALSE;
                return;
            }
            break;
        default:
            break;
    }

    // Check if this module can run at maximum frequency at all
    // voltages - no v-scale for this module as well
    pScale = s_pClockScales[pCinfo->Module];
    if(!pScale)
    {
        NV_ASSERT(!"Need scaling information");
        pCstate->Vscale = NV_FALSE;
        return;
    }
    if (pScale[0] == pScale[s_ChipFlavor.pSocShmoo->ShmooVmaxIndex])
    {
        NvRmMilliVolts SrcMaxMv = NvRmPrivSourceVscaleGetMV(
            hRmDevice, NvRmPrivModuleGetMaxSrcKHz(hRmDevice, pCinfo));
        if (SrcMaxMv == NvRmPrivGetStepMV(hRmDevice, 0))
        {
            pCstate->Vscale = NV_FALSE;
            return;
        }
    }
    // Other modules have v-scale
    pCstate->Vscale = NV_TRUE;
}

NvU32
NvRmPrivGetEmcDqsibOffset(NvRmDeviceHandle hRmDevice)
{
    const NvRmSocShmoo* p = s_ChipFlavor.pSocShmoo;
    return p->DqsibOffset;
}

NvError
NvRmPrivGetOscDoublerTaps(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz OscKHz,
    NvU32* pTaps)
{
    NvU32 i;
    NvU32 size = s_ChipFlavor.pSocShmoo->OscDoublerCfgListSize;
    const NvRmOscDoublerConfig* p = s_ChipFlavor.pSocShmoo->OscDoublerCfgList;

    // Find doubler settings for the specified oscillator frequency, and
    // return the number of taps for the SoC corner
    for (i = 0; i < size; i++)
    {
        if (p[i].OscKHz == OscKHz)
        {
            *pTaps = p[i].Taps[s_ChipFlavor.corner];
            return NvSuccess;
        }
    }
    return NvError_NotSupported;     // Not supported oscillator frequency
}

NvBool NvRmPrivIsCpuRailDedicated(NvRmDeviceHandle hRmDevice)
{
    const NvRmCpuShmoo* p = s_ChipFlavor.pCpuShmoo;
    return (p != NULL);
}

/*****************************************************************************/

// TODO: clock limits deinit in NvRmClose() - free s_pShmooData
// TODO: remove after RM partition is completed
#define NVRM_BOOT_USE_BOOTARG_SHMOO (1)

static NvError NvRmBootArgChipShmooGet(
    NvRmDeviceHandle hRmDevice,
    NvRmChipFlavor* pChipFlavor)
{
#if NVRM_BOOT_USE_BOOTARG_SHMOO

    NvU32 offset, size, TotalSize = 0;
    NvBootArgsChipShmoo BootArgSh;
    NvBootArgsChipShmooPhys BootArgShPhys;
    void* pBootShmooData = NULL;
    NvRmMemHandle hMem = NULL;
    NvError err = NvSuccess;
    ExecPlatform env;

    // Retrieve shmoo data
    err = NvOsBootArgGet(NvBootArgKey_ChipShmoo, &BootArgSh, sizeof(BootArgSh));
    if (err != NvSuccess)
    {
        err = NvError_BadParameter;
        goto fail;
    }

    if (BootArgSh.MemHandleKey != 0)
    {
        err = NvRmMemHandleClaimPreservedHandle(
            hRmDevice, BootArgSh.MemHandleKey, &hMem);
        if (err != NvSuccess)
        {
            goto fail;
        }

        TotalSize = NvRmMemGetSize(hMem);
        NV_ASSERT(TotalSize);
        err = NvRmMemMap(hMem, 0, TotalSize, NVOS_MEM_READ, &pBootShmooData);
        if( err != NvSuccess )
        {
            goto fail;
        }

        // Use OS memory to keep shmoo data, and release carveout buffer
        s_pShmooData = NvOsAlloc(TotalSize);
        if (!s_pShmooData)
        {
            err = NvError_InsufficientMemory;
            goto fail;
        }
        NvOsMemcpy(s_pShmooData, pBootShmooData, TotalSize);
        NvRmMemUnmap(hMem, pBootShmooData, TotalSize);
        NvRmMemHandleFree(hMem);
    }
    else
    {
        env = NvRmPrivGetExecPlatform(hRmDevice);
        if (env != ExecPlatform_Fpga)
        {
            err = NvError_BadParameter;
            goto fail;
        }

        err = NvOsBootArgGet(NvBootArgKey_ChipShmooPhys, &BootArgShPhys, sizeof(BootArgShPhys));
        if ((err != NvSuccess) || (BootArgShPhys.PhysShmooPtr == 0))
        {
            err = NvError_BadParameter;
            goto fail;
        }

        TotalSize = BootArgShPhys.Size;
        NV_ASSERT(TotalSize);

        // Use OS memory to keep shmoo data
        s_pShmooData = NvOsAlloc(TotalSize);
        if (!s_pShmooData)
        {
            err = NvError_InsufficientMemory;
            goto fail;
        }

        // Map the physical shmoo address passed by the backdoor loader
        err = NvOsPhysicalMemMap(BootArgShPhys.PhysShmooPtr, TotalSize,
            NvOsMemAttribute_WriteBack, 0, &pBootShmooData);
        if (err != NvSuccess)
        {
            goto fail;
        }

        // Copy the shmoo data and unmap the backdoor shmoo pointer.
        NvOsMemcpy(s_pShmooData, pBootShmooData, TotalSize);
        NvOsPhysicalMemUnmap(pBootShmooData, TotalSize);
        pBootShmooData = NULL;
    }

    // Fill in shmoo data records
    pChipFlavor->sku = hRmDevice->ChipId.SKU;
    pChipFlavor->corner = BootArgSh.CoreCorner;
    pChipFlavor->CpuCorner = BootArgSh.CpuCorner;

    // Shmoo data for core domain
    pChipFlavor->pSocShmoo = &s_SocShmoo;

    offset = BootArgSh.CoreShmooVoltagesListOffset;
    size = BootArgSh.CoreShmooVoltagesListSize;
    NV_ASSERT (offset + size <= TotalSize);
    s_SocShmoo.ShmooVoltages = (const NvU32*)((NvUPtr)s_pShmooData + offset);
    size /= sizeof(*s_SocShmoo.ShmooVoltages);
    NV_ASSERT((size * sizeof(*s_SocShmoo.ShmooVoltages) ==
              BootArgSh.CoreShmooVoltagesListSize) && (size > 1));
    s_SocShmoo.ShmooVmaxIndex = size - 1;

    offset = BootArgSh.CoreScaledLimitsListOffset;
    size = BootArgSh.CoreScaledLimitsListSize;
    NV_ASSERT (offset + size <= TotalSize);
    s_SocShmoo.ScaledLimitsList =
        (const NvRmScaledClkLimits*) ((NvUPtr)s_pShmooData + offset);
    size /= sizeof(*s_SocShmoo.ScaledLimitsList);
    NV_ASSERT((size * sizeof(*s_SocShmoo.ScaledLimitsList) ==
              BootArgSh.CoreScaledLimitsListSize) && size);
    s_SocShmoo.ScaledLimitsListSize = size;

    offset = BootArgSh.OscDoublerListOffset;
    size = BootArgSh.OscDoublerListSize;
    NV_ASSERT (offset + size <= TotalSize);
    s_SocShmoo.OscDoublerCfgList =
        (const NvRmOscDoublerConfig*)((NvUPtr)s_pShmooData + offset);
    size /= sizeof(*s_SocShmoo.OscDoublerCfgList);
    NV_ASSERT((size * sizeof(*s_SocShmoo.OscDoublerCfgList) ==
              BootArgSh.OscDoublerListSize) && size);
    s_SocShmoo.OscDoublerCfgListSize = size;

    offset = BootArgSh.SKUedLimitsOffset;
    size = BootArgSh.SKUedLimitsSize;
    NV_ASSERT (offset + size <= TotalSize);
    s_SocShmoo.pSKUedLimits =
        (const NvRmSKUedLimits*)((NvUPtr)s_pShmooData + offset);
    NV_ASSERT(size == sizeof(*s_SocShmoo.pSKUedLimits));

    s_SocShmoo.DqsibOffset = BootArgSh.Dqsib;
    s_SocShmoo.SvopHighSetting = BootArgSh.SvopHighSetting;
    s_SocShmoo.SvopLowSetting = BootArgSh.SvopLowSetting;
    s_SocShmoo.SvopLowVoltage = BootArgSh.SvopLowVoltage;

    if (BootArgSh.CpuShmooVoltagesListSize && BootArgSh.CpuScaledLimitsSize)
    {
        // Shmoo data for dedicated CPU domain
        pChipFlavor->pCpuShmoo = &s_CpuShmoo;

        offset = BootArgSh.CpuShmooVoltagesListOffset;
        size = BootArgSh.CpuShmooVoltagesListSize;
        NV_ASSERT (offset + size <= TotalSize);
        s_CpuShmoo.ShmooVoltages =(const NvU32*)((NvUPtr)s_pShmooData + offset);
        size /= sizeof(*s_CpuShmoo.ShmooVoltages);
        NV_ASSERT((size * sizeof(*s_CpuShmoo.ShmooVoltages) ==
              BootArgSh.CpuShmooVoltagesListSize) && (size > 1));
        s_CpuShmoo.ShmooVmaxIndex = size - 1;

        offset = BootArgSh.CpuScaledLimitsOffset;
        size = BootArgSh.CpuScaledLimitsSize;
        NV_ASSERT (offset + size <= TotalSize);
        s_CpuShmoo.pScaledCpuLimits =
            (const NvRmScaledClkLimits*)((NvUPtr)s_pShmooData + offset);
        NV_ASSERT(size == sizeof(*s_CpuShmoo.pScaledCpuLimits));
    }
    else
    {
        pChipFlavor->pCpuShmoo = NULL;
    }
    return err;

fail:
    NvRmMemUnmap(hMem, pBootShmooData, TotalSize);
    NvRmMemHandleFree(hMem);
    NvOsFree(s_pShmooData);
    s_pShmooData = NULL;
    return err;
#else
    s_pShmooData = NULL;
    s_SocShmoo.ShmooVoltages = NULL;
    s_CpuShmoo.ShmooVoltages = NULL:
    return NvError_NotSupported;
#endif
}

NvError NvRmBootArgChipShmooSet(NvRmDeviceHandle hRmDevice)
{
#if NVRM_BOOT_USE_BOOTARG_SHMOO

// Alignment and size to get boot shmoo data into carveout memory
#define NVRM_BOOT_MEM_ALIGNMENT (0x1 << 12)
#define NVRM_BOOT_MEM_SIZE (0x1 << 13)

    static const NvRmHeap s_heaps[] =
    {
        NvRmHeap_ExternalCarveOut,
    };

    NvBootArgsChipShmoo BootArgSh;
    NvRmChipFlavor* pChipFlavor = &s_ChipFlavor;
    NvRmMemHandle hMem = NULL;
    void* p = NULL;
    NvError err = NvSuccess;
    NvU32 size = 0;

    NV_ASSERT(pChipFlavor->pSocShmoo);

    // Pack shmoo arrays and structures (all members are of NvU32 type).
    // Start with core domain.
    BootArgSh.CoreShmooVoltagesListOffset = size;
    BootArgSh.CoreShmooVoltagesListSize =
        (pChipFlavor->pSocShmoo->ShmooVmaxIndex + 1) *
        sizeof(*pChipFlavor->pSocShmoo->ShmooVoltages);
    size += BootArgSh.CoreShmooVoltagesListSize;

    BootArgSh.CoreScaledLimitsListOffset = size;
    BootArgSh.CoreScaledLimitsListSize =
        pChipFlavor->pSocShmoo->ScaledLimitsListSize *
        sizeof(*pChipFlavor->pSocShmoo->ScaledLimitsList);
    size += BootArgSh.CoreScaledLimitsListSize;

    BootArgSh.OscDoublerListOffset = size;
    BootArgSh.OscDoublerListSize =
        pChipFlavor->pSocShmoo->OscDoublerCfgListSize *
        sizeof(*pChipFlavor->pSocShmoo->OscDoublerCfgList);
    size += BootArgSh.OscDoublerListSize;

    BootArgSh.SKUedLimitsOffset = size;
    BootArgSh.SKUedLimitsSize =
        sizeof(*pChipFlavor->pSocShmoo->pSKUedLimits);
    size += BootArgSh.SKUedLimitsSize;

    if (pChipFlavor->pCpuShmoo)
    {
        // Add data for dedicated CPU domain
        BootArgSh.CpuShmooVoltagesListOffset = size;
        BootArgSh.CpuShmooVoltagesListSize =
            (pChipFlavor->pCpuShmoo->ShmooVmaxIndex + 1) *
            sizeof(*pChipFlavor->pCpuShmoo->ShmooVoltages);
        size += BootArgSh.CpuShmooVoltagesListSize;

        BootArgSh.CpuScaledLimitsOffset = size;
        BootArgSh.CpuScaledLimitsSize =
            sizeof(*pChipFlavor->pCpuShmoo->pScaledCpuLimits);
        size += BootArgSh.CpuScaledLimitsSize;
    }
    else
    {
        BootArgSh.CpuShmooVoltagesListOffset =
            BootArgSh.CpuScaledLimitsOffset = size;
        BootArgSh.CpuShmooVoltagesListSize = 0;
        BootArgSh.CpuScaledLimitsSize = 0;
    }

    // Align, allocate, and fill in shmoo packed data buffer
    size = NV_MAX(size, NVRM_BOOT_MEM_SIZE);

    err = NvRmMemHandleCreate(hRmDevice, &hMem, size);
    if( err!= NvSuccess )
    {
        goto fail;
    }
    err = NvRmMemAlloc(hMem, s_heaps, NV_ARRAY_SIZE(s_heaps),
                       NVRM_BOOT_MEM_ALIGNMENT, NvOsMemAttribute_Uncached);
    if( err != NvSuccess )
    {
        goto fail;
    }
    err = NvRmMemMap(hMem, 0, size, 0, &p);
    if( err != NvSuccess )
    {
        goto fail;
    }

    NvOsMemset(p, 0, size);
    NvRmMemWrite(hMem, BootArgSh.CoreShmooVoltagesListOffset,
        pChipFlavor->pSocShmoo->ShmooVoltages,
        BootArgSh.CoreShmooVoltagesListSize);
    NvRmMemWrite(hMem, BootArgSh.CoreScaledLimitsListOffset,
        pChipFlavor->pSocShmoo->ScaledLimitsList,
        BootArgSh.CoreScaledLimitsListSize);
    NvRmMemWrite(hMem, BootArgSh.OscDoublerListOffset,
        pChipFlavor->pSocShmoo->OscDoublerCfgList,
        BootArgSh.OscDoublerListSize);
    NvRmMemWrite(hMem, BootArgSh.SKUedLimitsOffset,
        pChipFlavor->pSocShmoo->pSKUedLimits, BootArgSh.SKUedLimitsSize);

    if (pChipFlavor->pCpuShmoo)
    {
        NvRmMemWrite(hMem, BootArgSh.CpuShmooVoltagesListOffset,
            pChipFlavor->pCpuShmoo->ShmooVoltages,
            BootArgSh.CpuShmooVoltagesListSize);
        NvRmMemWrite(hMem, BootArgSh.CpuScaledLimitsOffset,
            pChipFlavor->pCpuShmoo->pScaledCpuLimits,
            BootArgSh.CpuScaledLimitsSize);
    }

    // Preserve packed shmoo data buffer, and complete boot arg setting
    err = NvRmMemHandlePreserveHandle(hMem, &BootArgSh.MemHandleKey);
    if ( err != NvSuccess )
    {
        goto fail;
    }
    BootArgSh.Dqsib = pChipFlavor->pSocShmoo->DqsibOffset;
    BootArgSh.SvopHighSetting = pChipFlavor->pSocShmoo->SvopHighSetting;
    BootArgSh.SvopLowSetting = pChipFlavor->pSocShmoo->SvopLowSetting;
    BootArgSh.SvopLowVoltage = pChipFlavor->pSocShmoo->SvopLowVoltage;
    BootArgSh.CoreCorner = pChipFlavor->corner;
    BootArgSh.CpuCorner = pChipFlavor->CpuCorner;

    err = NvOsBootArgSet(NvBootArgKey_ChipShmoo, &BootArgSh, sizeof(BootArgSh));
    if ( err != NvSuccess )
    {
        goto fail;
    }
    return err;

fail:
    NvRmMemHandleFree(hMem);
    return err;
#else
    return NvSuccess;
#endif
}
