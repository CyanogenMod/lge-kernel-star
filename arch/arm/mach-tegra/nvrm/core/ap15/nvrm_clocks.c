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

/**
 * @file
 * @brief <b>nVIDIA Driver Development Kit:
 *           Clock Resource manager </b>
 *
 * @b Description: Implements Clock control API. All code in this file chip
 * independent. All chip dependent code should move to ap15rm_clocks.c file.
 */

#include "nvrm_clocks.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"
#include "ap15rm_private.h"
#include "ap15rm_clocks.h"
#include "ap20/ap20rm_clocks.h"
#include "nvrm_pmu_private.h"
#include "nvrm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"

// Module debug: 0=disable, 1=enable
#define NVRM_ENABLE_PRINTF (0)

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_POWER_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_POWER_PRINTF(x)
#endif

// TODO: Replace NvOsWaitUS() with NvRmPrivWaitUS()
// TODO: CAR access macro

// Actual FPGA clock frequency for all modules is 8.33MHz
// (display is exception)
#define FPGA_MODULE_KHZ_AP15 (8330)
#define FPGA_MODULE_KHZ_AP20 (13000)
#define FPGA_DISPLAY_KHZ (27000)

// QT clock frequency used as a special value (actual frequency is irrelevant)
#define QT_MODULE_KHZ (1)

// UART rate divider is part of the UART module and it is not discribed
// in central module clock information table. Hence, need this define.
#define NVRM_UART_TIMING_DIVISOR_MAX (0xFFFFUL)

/*****************************************************************************/

// Clock source descriptors and frequencies
static NvRmClockSourceInfo* s_ClockSourceTable = NULL;
static NvU32 s_ClockSourceFreq[NvRmClockSource_Num];
static NvRmSystemBusComplexInfo s_SystemBusComplex = {0};

// Module clocks frequency limits
static const NvRmModuleClockLimits* s_ModuleClockLimits;

// Module clocks descriptors and module clock state arrays of the same size
static const NvRmModuleClockInfo *s_moduleClockTable;
static NvU32 s_moduleClockTableSize;
static NvRmModuleClockState *s_moduleClockState = NULL;

// PLL references
static NvRmPllReference* s_PllReferencesTable;
static NvU32 s_PllReferencesTableSize;
static NvBool s_MipiPllVddOn = NV_FALSE;

// Mutex for thread-safe access to clock control records and h/w
static NvOsSpinMutexHandle s_hClockMutex = NULL;

// Mutex for thread-safe access to shared PLLs
static NvOsMutexHandle s_hPllMutex = NULL;

/*****************************************************************************/

NvError
NvRmPrivGetClockState(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvRmModuleClockInfo** CinfoOut,
    NvRmModuleClockState** StateOut)
{
    NvRmModuleInstance* inst;

    NV_ASSERT( hDevice );
    NV_ASSERT(s_moduleClockState);

    if (NvRmPrivGetModuleInstance(hDevice, ModuleId, &inst) != NvSuccess)
    {
        return NvError_ModuleNotPresent;
    }
    if (inst->ModuleData)
    {
        *CinfoOut = (NvRmModuleClockInfo*)inst->ModuleData;
        *StateOut = &s_moduleClockState[(*CinfoOut) - s_moduleClockTable];
        return NvSuccess;
    }
    else
    {
        // Starting with AP20 no dedicated HSMMC clock (mapped to SDMMC)
        if ((ModuleId == NvRmModuleID_Hsmmc) &&
            (hDevice->ChipId.Id != 0x15) && (hDevice->ChipId.Id != 0x16))
        {
            return NvError_ModuleNotPresent;
        }
        NV_ASSERT(!"module clock info missing --"
                   " fillup the [apxx]rm_clocks_info.c file");
        return NvError_NotSupported;
    }
}

/*****************************************************************************/

static void
NvRmPrivPllDPowerControl(
    NvRmDeviceHandle hDevice,
    NvBool ConfigEntry,
    NvBool* pMipiPllVddOn)
{
    NvRmPrivAp15PllDPowerControl(hDevice, ConfigEntry, pMipiPllVddOn);
}

static void
NvRmPrivDisablePLLs(
    NvRmDeviceHandle hDevice,
    const NvRmModuleClockInfo* cinfo,
    const NvRmModuleClockState* state)
{
    NvRmPrivAp15DisablePLLs(hDevice, cinfo, state);
}

static NvRmClockSource
NvRmPrivGetImplicitPllSource(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module)
{
    switch (Module)
    {
        // DSI, CSI, I2C and UART modules are implicitely attached to PLLP3
        // output derived from primary PLLP0.
        case NvRmModuleID_Dsi:
        case NvRmModuleID_Csi:
        case NvRmModuleID_I2c:
        case NvRmModuleID_Uart:
            return NvRmClockSource_PllP0;

        // MPE depends on PLLA for audio in AP15/16
        case NvRmModuleID_Mpe:
            if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
                return NvRmClockSource_PllA0;
        // fall through

        // No implicit dependencies for other modules
        default:
            return NvRmClockSource_Invalid;
    }
}

static void
NvRmPrivModuleClockAttach(
    NvRmDeviceHandle hDevice,
    const NvRmModuleClockInfo* cinfo,
    const NvRmModuleClockState* state,
    NvBool Enable)
{
    NvU32 i, reg;
    NvBool Enabled;
    NvRmClockSource SubSourceId = NvRmClockSource_Invalid;
    NvRmClockSource SourceId = cinfo->Sources[state->SourceClock];

    if ((cinfo->Module == NvRmModuleID_Spdif) ||
        (cinfo->Module == NvRmModuleID_Vi) ||
        (cinfo->Module == NvRmModuleID_Tvo))
    {
        // Find secondary source for modules with explicit subclocks; subclock
        // descriptor and state are located after main ones, respectively
        SubSourceId = (cinfo + 1)->Sources[(state + 1)->SourceClock];
    }
    else
    {
        // Find implicit secondary source (if any) for other modules
        SubSourceId = NvRmPrivGetImplicitPllSource(hDevice, cinfo->Module);

    }

    NV_ASSERT(cinfo->ClkEnableOffset);
    reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  cinfo->ClkEnableOffset);
    Enabled = ((reg & cinfo->ClkEnableField) == cinfo->ClkEnableField);
    if (Enabled == Enable)
        return; // Exit if no changes in clock status

    for (i = 0; i < s_PllReferencesTableSize; i++)
    {
        // If module clock is to be enabled - attach sources (inc ref count)
        // If module clock is to be disabled - detach sources (dec ref count)
        if (s_PllReferencesTable[i].SourceId == SourceId)
            NvRmPrivPllRefUpdate(hDevice, &s_PllReferencesTable[i], Enable);
        if (s_PllReferencesTable[i].SourceId == SubSourceId)
            NvRmPrivPllRefUpdate(hDevice, &s_PllReferencesTable[i], Enable);
    }
}

void
NvRmPrivModuleClockReAttach(
    NvRmDeviceHandle hDevice,
    const NvRmModuleClockInfo* cinfo,
    const NvRmModuleClockState* state)
{
    NvU32 i, reg;
    NvRmClockSource SourceId = cinfo->Sources[state->SourceClock];

    for (i = 0; i < s_PllReferencesTableSize; i++)
    {
        NvBool* pAttached =
         &s_PllReferencesTable[i].AttachedModules[cinfo - s_moduleClockTable];
        NvBool WasAttached = *pAttached;
        NvBool IsAttached = (s_PllReferencesTable[i].SourceId == SourceId);

        if (WasAttached != IsAttached)
        {
            // Changes in source reference always recorded but affect
            // ref count only when the module clock is enabled
            if(cinfo->ClkEnableOffset != 0)
            {
                reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                              cinfo->ClkEnableOffset);
                if ((reg & cinfo->ClkEnableField) == cinfo->ClkEnableField)
                {
                    NvRmPrivPllRefUpdate(
                        hDevice, &s_PllReferencesTable[i], IsAttached);
                }
            }
            *pAttached = IsAttached;
        }
    }
}

static void
NvRmPrivCoreClockReAttach(
    NvRmDeviceHandle hDevice,
    NvRmClockSource CoreId,
    NvRmClockSource SourceId)
{
    static NvU32 s_CpuModuleIndex = (NvU32)-1;
    static NvU32 s_AvpModuleIndex = (NvU32)-1;

    NvU32 i, ModuleIndex;

    // Map core bus clock to processor module. CPU, AVP are not in relocation
    // table, can not use module instance shortcut - search clock descriptors.
    if (CoreId == NvRmClockSource_CpuBus)
    {
        if (s_CpuModuleIndex == (NvU32)-1)
        {
            for (i = 0; i < s_moduleClockTableSize; i++)
            {
                if (s_moduleClockTable[i].Module == NvRmModuleID_Cpu)
                    break;
            }
            s_CpuModuleIndex = i;
        }
        NV_ASSERT(s_CpuModuleIndex < s_moduleClockTableSize);
        ModuleIndex = s_CpuModuleIndex;
    }
    else if (CoreId == NvRmClockSource_SystemBus)
    {
        if (s_AvpModuleIndex == (NvU32)-1)
        {
            for (i = 0; i < s_moduleClockTableSize; i++)
            {
                if (s_moduleClockTable[i].Module == NvRmModuleID_Avp)
                    break;
            }
            s_AvpModuleIndex = i;
        }
        NV_ASSERT(s_AvpModuleIndex < s_moduleClockTableSize);
        ModuleIndex = s_AvpModuleIndex;
    }
    else
    {
        NV_ASSERT(!"Invalid core id");
        return;
    }

    // Map secondary divided PLL outputs to primary PLLs
    switch (SourceId)
    {
        case NvRmClockSource_PllC1:
            SourceId = NvRmClockSource_PllC0;
            break;
        case NvRmClockSource_PllM1:
            SourceId = NvRmClockSource_PllM0;
            break;
        case NvRmClockSource_PllP1:
        case NvRmClockSource_PllP2:
        case NvRmClockSource_PllP3:
        case NvRmClockSource_PllP4:
            SourceId = NvRmClockSource_PllP0;
            break;
        default:
            break;
    }

    // Record changes in PLL references and update ref count
    for (i = 0; i < s_PllReferencesTableSize; i++)
    {
        NvBool* pAttached =
         &s_PllReferencesTable[i].AttachedModules[ModuleIndex];
        NvBool WasAttached = *pAttached;
        NvBool IsAttached = (s_PllReferencesTable[i].SourceId == SourceId);

        if (WasAttached != IsAttached)
        {
            *pAttached = IsAttached;
            NvRmPrivPllRefUpdate(hDevice, &s_PllReferencesTable[i], IsAttached);
        }
    }
}

void
NvRmPrivMemoryClockReAttach(
    NvRmDeviceHandle hDevice,
    const NvRmModuleClockInfo* cinfo,
    const NvRmModuleClockState* state)
{
    NvU32 i;
    NvRmClockSource SourceId = cinfo->Sources[state->SourceClock];

    // MC clock on AP20 and newer chips is always the same as EMC1x domain clock
    // So there is no need for source reference double-counting.
    if ((hDevice->ChipId.Id >= 0x20) &&
        (cinfo->Module == NvRmPrivModuleID_MemoryController))
        return;

    for (i = 0; i < s_PllReferencesTableSize; i++)
    {
        NvBool* pAttached =
         &s_PllReferencesTable[i].AttachedModules[cinfo - s_moduleClockTable];
        NvBool WasAttached = *pAttached;
        NvBool IsAttached = (s_PllReferencesTable[i].SourceId == SourceId);

        // Record changes in PLL references and update ref count.
        // TODO: secondary PLL outputs mapping (only primary PLLs are used now)
        if (WasAttached != IsAttached)
        {
            *pAttached = IsAttached;
            NvRmPrivPllRefUpdate(hDevice, &s_PllReferencesTable[i], IsAttached);
        }
    }
}

void
NvRmPrivExternalClockAttach(
    NvRmDeviceHandle hDevice,
    NvRmClockSource SourceId,
    NvBool Enable)
{
    NvU32 i;

    // Map secondary divided PLL outputs to primary PLLs
    switch (SourceId)
    {
        case NvRmClockSource_PllC1:
            SourceId = NvRmClockSource_PllC0;
            break;
        case NvRmClockSource_PllM1:
            SourceId = NvRmClockSource_PllM0;
            break;
        case NvRmClockSource_PllP1:
        case NvRmClockSource_PllP2:
        case NvRmClockSource_PllP3:
        case NvRmClockSource_PllP4:
            SourceId = NvRmClockSource_PllP0;
            break;
        default:
            break;
    }

    // Attach external clock
    for (i = 0; i < s_PllReferencesTableSize; i++)
    {
        if (s_PllReferencesTable[i].SourceId == SourceId)
        {
            // If ext clock is enabled - attach source (inc ref count)
            // If ext clock is disabled - detach source (dec ref count)
            NvOsSpinMutexLock(s_hClockMutex);
            s_PllReferencesTable[i].ExternalClockRefCnt += (Enable ? 1 : (-1));
            NvRmPrivPllRefUpdate(hDevice, &s_PllReferencesTable[i], Enable);

            // Configure clock source if necessary (required for PLLA)
            if (SourceId == NvRmClockSource_PllA0)
                NvRmPrivConfigureClockSource(hDevice, NvRmModuleID_I2s, Enable);
            NvOsSpinMutexUnlock(s_hClockMutex);
        }
    }
}



/*****************************************************************************/

void
NvRmPrivEnableModuleClock(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleId,
    ModuleClockState ClockState)
{
    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            Ap15EnableModuleClock(hRmDevice, ModuleId, ClockState);
            break;
        case 0x20:
            Ap20EnableModuleClock(hRmDevice, ModuleId, ClockState);
            break;
        default:
            NV_ASSERT(!"Unsupported chip ID");
    }
}

NvError NvRmPowerModuleClockControl(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvU32 ClientId,
    NvBool Enable)
{
    NvRmModuleClockInfo *cinfo;
    NvRmModuleClockState *state;
    NvRmMilliVolts v = NvRmVoltsUnspecified;
    NvError err;
    ModuleClockState ClockState =
        Enable ? ModuleClockState_Enable : ModuleClockState_Disable;
    NvRmModuleID ModuleName = NVRM_MODULE_ID_MODULE(ModuleId);
    // Clock control/configurations shared between drivers and DVFS
    NvBool SharedModule = ((ModuleName == NvRmModuleID_Display) ||
                           (ModuleName == NvRmModuleID_Dsi) ||
                           (ModuleName == NvRmModuleID_Vde));

    if (NvRmPrivIsDiagMode(ModuleId))
        return NvSuccess;

    // Get pointers to module clock info and current module clock state
    err = NvRmPrivGetClockState(hDevice, ModuleId, &cinfo, &state);
    if (err != NvSuccess)
        return err;

    // Protect access to clocks that shared control (directly, or via PLLs)
    // with DVFS. Made sure DSI power rail is up if DSI clock is enabled.
    if (SharedModule)
    {
        NvOsMutexLock(s_hPllMutex);
        if (Enable && (ModuleName == NvRmModuleID_Dsi))
            NvRmPrivPllDPowerControl(hDevice, NV_TRUE, &s_MipiPllVddOn);
    }
    NvOsSpinMutexLock(s_hClockMutex);

    // Check if voltage scaling is required before module clock is enabled.
    // Core voltage access is shared with DVFS. PMU access transport must
    // *not* be scalable.
    if (Enable)
    {
        // Preview, don't update scaling ref counts if voltage going up
        v = NvRmPrivModuleVscaleAttach(
            hDevice, cinfo, state, Enable, NV_TRUE);

        if ((v != NvRmVoltsUnspecified) &&
            (v != NvRmVoltsOff))
        {
            // Preview reported voltage increase - set target pending to
            // prevent DVFS scaling down, while lock is released
            NvRmPrivModuleVscaleSetPending(hDevice, v);

            NvOsSpinMutexUnlock(s_hClockMutex);
            if (!SharedModule)
                NvOsMutexLock(s_hPllMutex);
            NvRmPrivDvsRequest(v);
            if (!SharedModule)
                NvOsMutexUnlock(s_hPllMutex);
            NvOsSpinMutexLock(s_hClockMutex);

            // Now, after voltage is increased - update scaling ref counts
            // and cancel pending request
            v = NvRmPrivModuleVscaleAttach(
                hDevice, cinfo, state, Enable, NV_FALSE);
            NvRmPrivModuleVscaleSetPending(hDevice, NvRmVoltsOff);
        }
    }

    // Restart reference counting if it is the 1st clock control call
    if (!state->FirstReference)
    {
        state->FirstReference = NV_TRUE;
        state->refCount = 0;
    }

    // Update reference count, and exit if
    // - clock enable requested and module clock is already enabled
    // - clock disable requested, but not all enable requests have been matched
    if (Enable)
    {
        if (state->refCount != 0)
        {
            state->refCount++;
            goto leave; // err = NvSuccess already
        }
        state->refCount = 1;
    }
    else if (state->refCount != 0)
    {
        state->refCount --;
        if (state->refCount != 0)
        {
            goto leave; // err = NvSuccess already
        }
    }
    else
    {
        // TODO: assert on disable without enable
        NvOsDebugPrintf(
            "Clock control balance failed for module %d, instance %d\n",
            NVRM_MODULE_ID_MODULE(ModuleId), NVRM_MODULE_ID_INSTANCE(ModuleId));
        // NV_ASSERT(!"Clock control balance violation");
    }
    NvRmPrivModuleClockAttach(hDevice, cinfo, state, Enable);
    NvRmPrivEnableModuleClock(hDevice, ModuleId, ClockState);

    // Check if voltage can be lowered after module clock is disabled.
    if (!Enable)
    {
        v = NvRmPrivModuleVscaleAttach(
            hDevice, cinfo, state, Enable, NV_FALSE);
        if (v == NvRmVoltsOff)
            NvRmPrivDvsRequest(v);  // No transaction, just set update flag
    }

    // Common exit
leave:
    NvOsSpinMutexUnlock(s_hClockMutex);
    if (SharedModule)
    {
        if (!Enable && (ModuleName == NvRmModuleID_Dsi))
            NvRmPrivPllDPowerControl(hDevice, NV_FALSE, &s_MipiPllVddOn);
        NvOsMutexUnlock(s_hPllMutex);
    }
    return err;
}

/*****************************************************************************/

ExecPlatform NvRmPrivGetExecPlatform(NvRmDeviceHandle hRmDeviceHandle)
{
    if (hRmDeviceHandle->ChipId.Major != 0)
    {
        return ExecPlatform_Soc;
    }
    if (NvRmIsSimulation())
    {
        return ExecPlatform_Sim;
    }
    if (hRmDeviceHandle->ChipId.Minor != 0)
    {
        return ExecPlatform_Fpga;
    }
    return ExecPlatform_Qt;
}

/*****************************************************************************/

#define NVRM_DEBUG_MODULE_CLOCK_SET (1)

/* Sets module clock source/divider register */
void NvRmPrivModuleClockSet(
    NvRmDeviceHandle hDevice,
    const NvRmModuleClockInfo* cinfo,
    const NvRmModuleClockState* state)
{
    NvU32 reg, divisor;

#if NVRM_DEBUG_MODULE_CLOCK_SET
    if(cinfo->ClkEnableOffset != 0)
    {
        reg = NV_REGR(hDevice,
            NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkEnableOffset);
        if ((reg & cinfo->ClkEnableField) != cinfo->ClkEnableField)
            NvOsDebugPrintf("tegra: configuring disabled clock ( module %d, "
                "instance %d )\n", cinfo->Module, cinfo->Instance);
    }
#endif
    NV_ASSERT(cinfo->ClkSourceOffset);
    reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset);
    divisor = (reg >> cinfo->DivisorFieldShift) & cinfo->DivisorFieldMask;
    if ((cinfo->Divider  != NvRmClockDivider_None) &&
        (state->Divider > divisor))
    {
        // Switch divider 1st, source 2nd, if new divisor is bigger
        NV_ASSERT(state->Divider <= cinfo->DivisorFieldMask);
        reg &= ~(cinfo->DivisorFieldMask << cinfo->DivisorFieldShift);
        reg |= state->Divider << cinfo->DivisorFieldShift;
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }

    NV_ASSERT(state->SourceClock <= cinfo->SourceFieldMask);
    reg &= (~(cinfo->SourceFieldMask << cinfo->SourceFieldShift));
    reg |= ( state->SourceClock << cinfo->SourceFieldShift);
    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);

    if ((cinfo->Divider  != NvRmClockDivider_None) &&
        (state->Divider < divisor))
    {
        // Switch source 1st, divider 2nd, if new divisor is smaller
        reg &= ~(cinfo->DivisorFieldMask << cinfo->DivisorFieldShift);
        reg |= state->Divider << cinfo->DivisorFieldShift;
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
}

static NvRmFreqKHz
NvRmPrivGetEmcSyncFreq(
    NvRmDeviceHandle hDevice,
    NvRmModuleID Module)
{
    switch (hDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            return NvRmPrivAp15GetEmcSyncFreq(hDevice, Module);
        case 0x20:
            return NvRmPrivAp20GetEmcSyncFreq(hDevice, Module);
        default:
            NV_ASSERT(!"Unsupported chip ID");
            return NvRmFreqMaximum;
    }
}

static NvBool
NvRmPrivIsModuleClockException(
    NvRmDeviceHandle hDevice,
    NvRmModuleClockInfo *cinfo,
    NvU32 clockSourceCount,
    NvU32 MinFreq,
    NvU32 MaxFreq,
    const NvRmFreqKHz* PrefFreqList,
    NvU32 PrefCount,
    NvRmModuleClockState *state,
    NvU32 flags)
{
    return NvRmPrivAp15IsModuleClockException(
        hDevice, cinfo, clockSourceCount, MinFreq, MaxFreq,
        PrefFreqList, PrefCount, state, flags);
}

/* Returns the best source clock and the best divider */
static NvError NvRmFindBestClockSource(
    NvRmDeviceHandle hDevice,
    NvRmModuleClockInfo *cinfo,
    NvU32 clockSourceCount,
    NvU32 MinFreq,
    NvU32 MaxFreq,
    const NvRmFreqKHz* PrefFreqList,
    NvU32 PrefCount,
    NvRmModuleClockState *state,
    NvU32 flags)
{
    NvU32 bestdiff = 0x7FFFFFFF;
    NvU32 bestdiv = 0x0;
    NvU32 SourceClock = (NvU32)-1;
    NvU32 SourceClockFreq = 0;
    NvU32 i = 0,j = 0;
    NvRmFreqKHz freq = 0, ReachedFreq = 0;
    NvU32 temp = 0, div = 0, mantissa = 0;
    NvS32 diff = 0;

    NV_ASSERT((MinFreq != 0) && (MinFreq <= MaxFreq));

    // Check if exceptional handling is required this module clock, and exit
    // if it is completed
    if (NvRmPrivIsModuleClockException(hDevice, cinfo, clockSourceCount,
          MinFreq, MaxFreq, PrefFreqList, PrefCount, state, flags))
        return NvSuccess;

    for (j=0; j< PrefCount; j++)        // loop through target frequencies
    {
        freq = (PrefFreqList[j] == NvRmFreqMaximum) ? MaxFreq : PrefFreqList[j];
        if (flags & NvRmClockConfig_QuietOverClock)
            freq = (PrefFreqList[j] > MaxFreq) ? MaxFreq : PrefFreqList[j];
        if ((freq < MinFreq) || (freq > MaxFreq))
            continue;

        for (i=0; i< clockSourceCount; i++) // loop through avilable sources
        {
            NV_ASSERT(cinfo->Sources[i] < NvRmClockSource_Num);
            if (cinfo->Sources[i] == NvRmClockSource_Invalid)
                break;

            SourceClockFreq = s_ClockSourceFreq[(cinfo->Sources[i])];
            if (SourceClockFreq < MinFreq)
                continue;
            if (NvRmPrivIsSourceProtected(
                hDevice, cinfo->Module, cinfo->Sources[i]))
                continue;

            if ((cinfo->Divider == NvRmClockDivider_None) ||
                (SourceClockFreq <= freq))
            {
                div = 1;
                if (cinfo->Module == NvRmModuleID_Uart)
                {
                    // If target is not reachable from the source by integer
                    // division - reject the source
                    if (!NvRmIsFreqRangeReachable(SourceClockFreq,
                            MinFreq, MaxFreq, NVRM_UART_TIMING_DIVISOR_MAX))
                        continue;
                }
                else if (SourceClockFreq > MaxFreq)
                    continue;
            }
            else    // Divider, SourceClockFreq > freq
            {
                // Default integer divider: Freq = SourceClockFreq / div
                // where div = h/w divisor field
                NvU32 MaxDivisor = cinfo->DivisorFieldMask;
                NV_ASSERT(MaxDivisor);

                if (cinfo->Divider == NvRmClockDivider_Integer_1)
                {
                    // Integer divider: Freq = SourceClockFreq / div
                    // where div = h/w divisor field + 1
                    MaxDivisor += 1;
                }
                else if (cinfo->Divider == NvRmClockDivider_Fractional_2)
                {
                    // Fractional divider: Freq = (SourceClockFreq * 2) / div
                    // where div = h/w divisor field + 2
                    SourceClockFreq = (SourceClockFreq << 1);
                    MaxDivisor += 2;
                }

                // Find divisor floor / freq ceiling, and
                // the 1st bit of the fractional part
                temp = (SourceClockFreq << 1) / freq;
                div = temp >> 1;
                mantissa = temp & 0x01;

                // Check if divisor value fits divisor field
                if (div >= MaxDivisor)
                {
                    div = MaxDivisor;
                    if (SourceClockFreq > div * (NvU64)MaxFreq)
                        continue;   // max boundary violation at max divisor
                }
                else if (SourceClockFreq > div * (NvU64)MaxFreq)
                {
                    div += 1; // divisor ceiling / freq floor
                    if (SourceClockFreq < div * (NvU64)MinFreq)
                        continue;   // both max and min boundaries violation
                }
                else if (mantissa)
                {
                    div += 1; // divisor ceiling / freq floor
                    if (SourceClockFreq < div * (NvU64)MinFreq)
                        div -= 1; // fall back to divisor floor / freq ceiling
                }
            }
            // Check if new traget frequency approximation is the best, so far
            ReachedFreq = SourceClockFreq / div;
            diff = freq - ReachedFreq;
            if (diff < 0)
                diff *= -1;
            if ( ((NvU32) diff < bestdiff) ||
                 (((NvU32) diff == bestdiff) && (div < bestdiv)) )
            {
                SourceClock = i;
                bestdiv = div;
                bestdiff = (NvU32)diff;
            }
        }
        // stop searching if "perfect" match found
        if (!bestdiff)
            break;
    }

    if ((bestdiv == 0) || (SourceClock == (NvU32) -1))
    {
        NV_ASSERT(!"No clock source found for this panel");
        return NvError_NotSupported;
    }

    // Fill in clock state taking into account different types of dividers
    state->Divider = bestdiv;
    state->SourceClock = SourceClock;
    SourceClockFreq = s_ClockSourceFreq[cinfo->Sources[SourceClock]];

    if (cinfo->Divider == NvRmClockDivider_Integer_1)
    {
        state->Divider = bestdiv - 1;
    }
    else if (cinfo->Divider == NvRmClockDivider_Fractional_2)
    {
        if (bestdiv == 1)
            bestdiv = 2; // cast pass thru case into generic formula
        state->Divider = (bestdiv - 2);
        SourceClockFreq = (SourceClockFreq << 1);
    }

    state->actual_freq = SourceClockFreq / bestdiv;

    return NvSuccess;
}

/*****************************************************************************/

static void RmReset2D(NvRmDeviceHandle hRmDevice)
{
    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            NvRmPrivAp15Reset2D(hRmDevice);
            return;
        case 0x20:
            NvRmPrivAp20Reset2D(hRmDevice);
            return;
        default:
            NV_ASSERT(!"Unsupported chip ID");
            return;
    }
}

static void ScaledClockConfigInit(NvRmDeviceHandle hRmDevice)
{
    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
        return;     // Initialize scaled clock configuration only on SoC

    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            NvRmPrivAp15EmcConfigInit(hRmDevice);
            return;
        case 0x20:
            NvRmPrivAp20ScaledClockConfigInit(hRmDevice);
            return;
        default:
            NV_ASSERT(!"Unsupported chip ID");
            return;
    }
}

static void ModuleClockStateInit(NvRmDeviceHandle hRmDevice)
{
    NvError e;
    NvU32 i, j, flags, reg;
    NvRmModuleID ModuleId;
    NvRmClockSource ImplicitPll;
    const NvRmModuleClockInfo* cinfo;
    NvRmModuleClockState *state;

    for (i = 0; i < s_moduleClockTableSize; i++)
    {
        flags = 0;
        ImplicitPll = NvRmClockSource_Invalid;
        cinfo = &s_moduleClockTable[i];
        state = &s_moduleClockState[i];
        ModuleId = NVRM_MODULE_ID(cinfo->Module, cinfo->Instance);

        if (cinfo->SubClockId)
        {
            // Check module subclock configuration
            if ((cinfo->Module == NvRmModuleID_Spdif) ||
                (cinfo->Module == NvRmModuleID_Vi) ||
                (cinfo->Module == NvRmModuleID_Tvo))
                flags = NvRmClockConfig_SubConfig;
        }
        else
        {
            // Check implicit attachment to PLLs for main clocks only
            ImplicitPll =
                NvRmPrivGetImplicitPllSource(hRmDevice, cinfo->Module);
            NV_ASSERT((ImplicitPll == NvRmClockSource_Invalid) ||
                      (cinfo->ClkEnableOffset));
        }

        // Fill in module clock state, attach explicit PLL sources for clocks
        // and subclocks. Special cases: CPU and AVP are not in the relocation
        // table, and attached to PLL via CPU and System bus, respectively
        e = NvRmPowerModuleClockConfig(
            hRmDevice, ModuleId, 0, 0, 0, NULL, 0, NULL, flags);
        NV_ASSERT((e == NvSuccess) || (e == NvError_ModuleNotPresent));
        if (e == NvSuccess)
        {
            NvRmMilliVolts v;   // can be ignored as we always boot at max V
            NvRmFreqKHz SourceClockFreq =
                s_ClockSourceFreq[(cinfo->Sources[state->SourceClock])];
            NvRmPrivModuleSetScalingAttribute(hRmDevice, cinfo, state);
            v = NvRmPrivModuleVscaleReAttach(hRmDevice,
                cinfo, state, state->actual_freq, SourceClockFreq, NV_FALSE);
            (void)v;
        }
        else if ((cinfo->Module == NvRmModuleID_Cpu) ||
                 (cinfo->Module == NvRmModuleID_Avp))
        {
            const NvRmCoreClockInfo* pCore =
                NvRmPrivGetClockSourceHandle(cinfo->Sources[0])->pInfo.pCore;
            NvRmClockSource SourceId =
                NvRmPrivCoreClockSourceGet(hRmDevice, pCore);
            NvRmPrivCoreClockReAttach(hRmDevice, pCore->SourceId, SourceId);
        }

        // Attach implicit PLL sources and update reference count
        // for enabled main clocks
        if (flags == NvRmClockConfig_SubConfig)
            continue;

        if (cinfo->ClkEnableOffset)
        {
            reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                cinfo->ClkEnableOffset);
            if ((reg & cinfo->ClkEnableField) == cinfo->ClkEnableField)
            {
                for (j = 0; j < s_PllReferencesTableSize; j++)
                {
                    if (s_PllReferencesTable[j].SourceId == ImplicitPll)
                        NvRmPrivPllRefUpdate(
                            hRmDevice, &s_PllReferencesTable[j], NV_TRUE);
                }
                state->refCount = 1;
            }
        }
        else
            state->refCount = 1;    // free running clock
    }
}

NvError
NvRmPrivClocksInit(NvRmDeviceHandle hRmDevice)
{
    NvRmModuleID ModuleId;
    NvU32 i = 0;
    NvU32 fpgaModuleFreq = 0;
    ExecPlatform env;
    NvError e;

    NV_ASSERT(hRmDevice);
    env = NvRmPrivGetExecPlatform(hRmDevice);

    NV_CHECK_ERROR_CLEANUP(NvOsSpinMutexCreate(&s_hClockMutex));
    NV_CHECK_ERROR_CLEANUP(NvOsMutexCreate(&s_hPllMutex));

    /*
     * Clock tree descriptors and reference tables initialization
     */
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
    {
        s_moduleClockTable = g_Ap15ModuleClockTable;
        s_moduleClockTableSize = g_Ap15ModuleClockTableSize;
        NvRmPrivAp15PllReferenceTableInit(&s_PllReferencesTable,
                &s_PllReferencesTableSize);
        s_ClockSourceTable = NvRmPrivAp15ClockSourceTableInit();
        fpgaModuleFreq = FPGA_MODULE_KHZ_AP15;
    }
    else if (hRmDevice->ChipId.Id == 0x20)
    {
        s_moduleClockTable = g_Ap20ModuleClockTable;
        s_moduleClockTableSize = g_Ap20ModuleClockTableSize;
        NvRmPrivAp20PllReferenceTableInit(&s_PllReferencesTable,
                &s_PllReferencesTableSize);
        s_ClockSourceTable = NvRmPrivAp20ClockSourceTableInit();
        fpgaModuleFreq = FPGA_MODULE_KHZ_AP20;
    }
    else
        NV_ASSERT(!"Unsupported chip ID");

    /*
     * Allocate module clock state array, and map module clock descriptors
     * to module instances.
     */
    s_moduleClockState = (NvRmModuleClockState *)
        NvOsAlloc(sizeof (NvRmModuleClockState) * s_moduleClockTableSize);
    if (s_moduleClockState == NULL)
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }
    NvOsMemset(s_moduleClockState, 0,
            sizeof(NvRmModuleClockState) * s_moduleClockTableSize);

    for (i = 0; i < s_moduleClockTableSize; i++)
    {
        NvRmModuleInstance* inst;
        ModuleId = NVRM_MODULE_ID(
            s_moduleClockTable[i].Module, s_moduleClockTable[i].Instance);

        if (s_moduleClockTable[i].SubClockId)
            continue;   // skip subclocks

        if (NvRmPrivGetModuleInstance(hRmDevice, ModuleId, &inst) == NvSuccess)
        {
            inst->ModuleData = (void *)&s_moduleClockTable[i];
        }
        else
        {
            // NvOsDebugPrintf(
            // "No module found for clock descriptor with module ID %d\n", ModuleID);
        }
    }

    /*
     * Clock limits and sources initialization
     */
    s_ModuleClockLimits = NvRmPrivClockLimitsInit(hRmDevice);
    s_ClockSourceFreq[NvRmClockSource_Invalid] = 0;
    s_SystemBusComplex.BusRateOffset = 0;
    {
        if (env == ExecPlatform_Fpga)
        {
            s_ClockSourceFreq[NvRmClockSource_ClkS] = 32;
            s_ClockSourceFreq[NvRmClockSource_ClkM] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_ClkD] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_PllA0] = 12288;
            s_ClockSourceFreq[NvRmClockSource_PllP0] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_PllC0] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_PllM0] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_PllX0] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_CpuBus] = fpgaModuleFreq;
            s_ClockSourceFreq[NvRmClockSource_SystemBus] = fpgaModuleFreq;
            NvRmPrivBusClockInit(
                hRmDevice, s_ClockSourceFreq[NvRmClockSource_SystemBus]);
        }
        else if ((env == ExecPlatform_Qt) || (env == ExecPlatform_Sim))
        {
            s_ClockSourceFreq[NvRmClockSource_ClkS] = 32;
            if (env == ExecPlatform_Sim) // On sim set main frequency 13MHz
            {
                s_ClockSourceFreq[NvRmClockSource_ClkM] = 13000;
                s_ClockSourceFreq[NvRmClockSource_ClkD] = 26000;
            }
            else                        // On Qt keep 12MHz
            {
                s_ClockSourceFreq[NvRmClockSource_ClkM] = 12000;
                s_ClockSourceFreq[NvRmClockSource_ClkD] = 24000;
            }
            s_ClockSourceFreq[NvRmClockSource_PllA0] =  12288;
            s_ClockSourceFreq[NvRmClockSource_PllP0] = 432000;
            s_ClockSourceFreq[NvRmClockSource_PllP1] =  28800;
            s_ClockSourceFreq[NvRmClockSource_PllP2] =  48000;
            s_ClockSourceFreq[NvRmClockSource_PllP3] =  72000;
            s_ClockSourceFreq[NvRmClockSource_PllP4] = 108000;
            s_ClockSourceFreq[NvRmClockSource_PllC0] = 600000;
            s_ClockSourceFreq[NvRmClockSource_PllM0] = 333000;
            s_ClockSourceFreq[NvRmClockSource_SystemBus] = 150000;
            NvRmPrivAp15SimPllInit(hRmDevice); // Enable plls in simulation
            NvRmPrivBusClockInit(
                hRmDevice, s_ClockSourceFreq[NvRmClockSource_SystemBus]);
        }
        else if (env == ExecPlatform_Soc)
        {
            NvRmPrivClockSourceFreqInit(hRmDevice, s_ClockSourceFreq);
        }
        else
        {
            NV_ASSERT(!"Not supported execution platform");
        }
        RmReset2D(hRmDevice);
    }

    /*
     * Initialize current modules clock state
     */
    if (env == ExecPlatform_Fpga)
    {
        for (i = 0; i < s_moduleClockTableSize; i++)
        {
            s_moduleClockState[i].actual_freq = fpgaModuleFreq;
        }
    }
    else if (env == ExecPlatform_Qt)
    {
        for (i = 0; i < s_moduleClockTableSize; i++)
        {
            s_moduleClockState[i].actual_freq = QT_MODULE_KHZ;
        }
    }
    ModuleClockStateInit(hRmDevice);
    ScaledClockConfigInit(hRmDevice);

    /* debug info... print out some initial frequencies */
    {
        NvU32 freq;

        if (NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0))
        {
            NvOsDebugPrintf("NVRM CLOCKS: PLLX0:      %d Khz\n",
                            s_ClockSourceFreq[NvRmClockSource_PllX0]);
        }
        NvOsDebugPrintf("NVRM CLOCKS: PLLM0:      %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_PllM0]);
        NvOsDebugPrintf("NVRM CLOCKS: PLLC0:      %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_PllC0]);
        NvOsDebugPrintf("NVRM CLOCKS: PLLP0:      %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_PllP0]);
        NvOsDebugPrintf("NVRM CLOCKS: PLLA0:      %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_PllA0]);
        NvOsDebugPrintf("NVRM CLOCKS: CPU:        %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_CpuBus]);
        NvOsDebugPrintf("NVRM CLOCKS: AVP:        %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_SystemBus]);
        NvOsDebugPrintf("NVRM CLOCKS: System Bus: %d Khz\n",
                        s_ClockSourceFreq[NvRmClockSource_SystemBus]);

        NV_ASSERT_SUCCESS(NvRmPowerModuleClockConfig(
            hRmDevice, NvRmPrivModuleID_MemoryController,
            0, 0, 0, NULL, 0, &freq, 0));
        NvOsDebugPrintf("NVRM CLOCKS: Memory Controller: %d\n", freq);

        NV_ASSERT_SUCCESS(NvRmPowerModuleClockConfig(
            hRmDevice, NvRmPrivModuleID_ExternalMemoryController,
            0, 0, 0, NULL, 0, &freq, 0));
        NvOsDebugPrintf("NVRM CLOCKS: External Memory Controller: %d\n", freq);
    }

    return NvSuccess;

fail:
    NvOsFree(s_moduleClockState);
    s_moduleClockState = NULL;
    NvOsMutexDestroy(s_hPllMutex);
    s_hPllMutex = NULL;
    NvOsSpinMutexDestroy(s_hClockMutex);
    s_hClockMutex = NULL;
    return e;
}

void
NvRmPrivClocksDeinit(NvRmDeviceHandle hRmDevice)
{
    NV_ASSERT(hRmDevice);

    if (s_moduleClockState != NULL)
    {
        // TODO: check refrence counts for "clock leakage"
    }
    NvOsFree(s_moduleClockState);
    s_moduleClockState = NULL;
    NvOsMutexDestroy(s_hPllMutex);
    s_hPllMutex = NULL;
    NvOsSpinMutexDestroy(s_hClockMutex);
    s_hClockMutex = NULL;
}

void
NvRmPrivBoostClocks(NvRmDeviceHandle hRmDevice)
{
    NvRmFreqKHz FreqKHz;

    // Initialize core voltage control
    NvRmPrivDvsInit();

    // Configure fast memory and core clocks (nominal core, CPU and memory
    // voltages are already set by this time during PMU initialization)
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
    {
        NvRmPrivAp15FastClockConfig(hRmDevice);
    }
    else if (hRmDevice->ChipId.Id == 0x20)
    {
        NvRmPrivAp20FastClockConfig(hRmDevice);
    }

    // Print fast clocks
    NvOsDebugPrintf("ADJUSTED CLOCKS:\n");
    NV_ASSERT_SUCCESS(NvRmPowerModuleClockConfig(
        hRmDevice, NvRmPrivModuleID_MemoryController,
        0, 0, 0, NULL, 0, &FreqKHz, 0));
    NvOsDebugPrintf("MC clock is set to %6d KHz\n", FreqKHz);

    NV_ASSERT_SUCCESS(NvRmPowerModuleClockConfig(
        hRmDevice, NvRmPrivModuleID_ExternalMemoryController,
        0, 0, 0, NULL, 0, &FreqKHz, 0));
    NvOsDebugPrintf("EMC clock is set to %6d KHz (DDR clock is at %6d KHz)\n",
                    FreqKHz, FreqKHz/2);

    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0))
    {
        FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllX0);
        NvOsDebugPrintf("PLLX0 clock is set to %6d KHz\n", FreqKHz);
    }
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
    NvOsDebugPrintf("PLLC0 clock is set to %6d KHz\n", FreqKHz);
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_CpuBus);
    NvOsDebugPrintf("CPU clock is set to %6d KHz\n", FreqKHz);
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus);
    NvOsDebugPrintf("System and AVP clock is set to %6d KHz\n", FreqKHz);

    // Print GPU clocks
    #define DEBUG_PRINT_MODULE_CLOCK(Name) \
    do\
    {\
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockConfig( \
            hRmDevice, NvRmModuleID_##Name, 0, 0, 0, NULL, 0, &FreqKHz, 0)); \
        NvOsDebugPrintf(#Name " clock is set to %6d KHz\n", FreqKHz); \
    } while (0)

    DEBUG_PRINT_MODULE_CLOCK(GraphicsHost);
    DEBUG_PRINT_MODULE_CLOCK(3D);
    DEBUG_PRINT_MODULE_CLOCK(2D);
    DEBUG_PRINT_MODULE_CLOCK(Epp);
    DEBUG_PRINT_MODULE_CLOCK(Mpe);
    DEBUG_PRINT_MODULE_CLOCK(Vde);
    #undef DEBUG_PRINT_MODULE_CLOCK
}

typedef struct NvRmPllRailMapRec
{
    // PLL Clock Source Id
    NvRmClockSource PllId;

    // Power rail GUID
    NvU64 PllRailId;
} NvRmPllRailMap;

static const NvRmPllRailMap s_PllRailMap[] =
{
    { NvRmClockSource_ClkM, NV_VDD_OSC_ODM_ID},
    { NvRmClockSource_PllA1, NV_VDD_PLLA_ODM_ID},
    { NvRmClockSource_PllC0, NV_VDD_PLLC_ODM_ID},
    { NvRmClockSource_PllD0, NV_VDD_PLLD_ODM_ID},
    { NvRmClockSource_PllM0, NV_VDD_PLLM_ODM_ID},
    { NvRmClockSource_PllP0, NV_VDD_PLLP_ODM_ID},
    { NvRmClockSource_PllU0, NV_VDD_PLLU1_ODM_ID},
    { NvRmClockSource_PllX0, NV_VDD_PLLX_ODM_ID},
};

void
NvRmPrivPllRailsInit(NvRmDeviceHandle hRmDevice)
{
    NvU32 i;

    for (i = 0; i < NV_ARRAY_SIZE(s_PllRailMap); i++)
    {
        NvU64 PllRailId = s_PllRailMap[i].PllRailId;
        NvRmClockSource PllId = s_PllRailMap[i].PllId;
        switch (PllId)
        {
            // If present PLLX is treated as other boot PLLs
            case NvRmClockSource_PllX0:
                if (!NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0))
                    break;
                // fall through

            // Oscillator and boot PLLs are already running - turn the
            // respective rails On, anyway, to sync ref count
            case NvRmClockSource_ClkM:
            case NvRmClockSource_PllC0:
            case NvRmClockSource_PllM0:
            case NvRmClockSource_PllP0:
                NvRmPrivPmuRailControl(hRmDevice, PllRailId, NV_TRUE);
                break;

            // If PLLA rail is turned On by BL - update ref count, otherwise
            // turn rail On, but leave PLLA disabled
            case NvRmClockSource_PllA1:
                if (NvRmPrivPmuRailGetVoltage(hRmDevice, PllRailId) == 0)
                {
                    NvRmPrivAp15PllSet(hRmDevice,
                        NvRmPrivGetClockSourceHandle(PllId)->pInfo.pPll,
                        0, 0, 0, (NvU32)-1, 0, 0, NV_TRUE, 0);
                }
                NvRmPrivPmuRailControl(hRmDevice, PllRailId, NV_TRUE);
                break;

            // If PLLD rail is turned On by BL - update ref count, otherwise
            // keep it Off and disable PLLD; initialize PLLD rail status
            case NvRmClockSource_PllD0:
                if (NvRmPrivPmuRailGetVoltage(hRmDevice, PllRailId) != 0)
                {
                    s_MipiPllVddOn = NV_TRUE;
                    NvRmPrivPmuRailControl(hRmDevice, PllRailId, NV_TRUE);
                }
                else
                {
                    s_MipiPllVddOn = NV_FALSE;
                    NvRmPrivAp15PllSet(hRmDevice,
                        NvRmPrivGetClockSourceHandle(PllId)->pInfo.pPll,
                        0, 0, 0, (NvU32)-1, 0, 0, NV_TRUE, 0);
                }
                break;

            // PLLU rail is controlled by USB stack - don't touch it, unless
            // USB download transport is active. In the latter case update ref
            // counts for PLLU and USB power rails
            case NvRmClockSource_PllU0:
                if (NvRmPrivGetDownloadTransport(hRmDevice) ==
                    NvOdmDownloadTransport_Usb)
                {
                    NvRmPrivPmuRailControl(hRmDevice, PllRailId, NV_TRUE);
                    NvRmPrivPmuRailControl(hRmDevice, NV_VDD_USB_ODM_ID, NV_TRUE);
                }
                break;

            default:
                NV_ASSERT(!"Invalid Id");
        }
    }
}

void
NvRmPrivClocksResume(NvRmDeviceHandle hRmDevice)
{
    // Sync clock sources after LP0
    NvRmPrivClockSourceFreqInit(hRmDevice, s_ClockSourceFreq);
    ScaledClockConfigInit(hRmDevice);
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
        NvRmPrivAp15FastClockConfig(hRmDevice);
    else if (hRmDevice->ChipId.Id == 0x20)
        NvRmPrivAp20FastClockConfig(hRmDevice);

}

/*****************************************************************************/

NvRmFreqKHz
NvRmPrivGetInterfaceMaxClock(NvRmDeviceHandle hRmDevice, NvRmModuleID ModuleId)
{

    NvU32 OdmModules[4];
    NvU32 OdmInstances[4];
    NvU32* pMaxClockSpeed = NULL;
    NvU32 count = 0;
    NvU32 i = 0;
    NvU32 instance = 0;
    NvU32 NumOdmModules = 0;
    NvU32 MaxFreq = 0;

    MaxFreq = NvRmFreqMaximum;

    NumOdmModules = NvRmPrivRmModuleToOdmModule(hRmDevice->ChipId.Id,
                        ModuleId, (NvOdmIoModule *)OdmModules, OdmInstances);

    for(i = 0; i < NumOdmModules; i++)
    {
        instance = OdmInstances[i];
        NvOdmQueryClockLimits(OdmModules[i], (const NvU32 **)&pMaxClockSpeed, &count);
        if ((pMaxClockSpeed) && (instance < count))
        {
            MaxFreq = pMaxClockSpeed[instance];
        }
    }

    return MaxFreq;
}

NvRmFreqKHz
NvRmPrivModuleGetMaxSrcKHz(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* cinfo)
{
    NvU32 i;
    NvRmFreqKHz SourceClockFreq = 0;

    for (i=0; i < NvRmClockSource_Num; i++)
    {
        NV_ASSERT(cinfo->Sources[i] < NvRmClockSource_Num);
        if (cinfo->Sources[i] == NvRmClockSource_Invalid)
            break;
        if (NvRmPrivIsSourceProtected(
            hRmDevice, cinfo->Module, cinfo->Sources[i]))
            continue;
        SourceClockFreq =
            NV_MAX(SourceClockFreq, s_ClockSourceFreq[(cinfo->Sources[i])]);
    }
    return SourceClockFreq;
}

static NvRmMilliVolts ModuleVscaleConfig(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* cinfo,
    NvRmModuleClockState *state,
    NvRmFreqKHz MaxFreq,
    NvBool Preview)
{
    NvRmFreqKHz f, SourceClockFreq;
    NvRmMilliVolts v = NvRmVoltsUnspecified;
    NvRmModuleID ModuleName = cinfo->Module;

    if (!state->Vscale)
        return v;

    // Find voltage level for the actually configured frequency. For Display,
    // UART and USB use maximum requested frequency, instead (Display and UART
    // actual clock configuration is completed outside CAR but it will not
    // exceed maximum requested boundary level; actual USB frequency is always
    // set to fixed PLLU output, but maximum boundary is used by driver to
    // communicate scaled voltage requirements).
    SourceClockFreq =
        s_ClockSourceFreq[(cinfo->Sources[state->SourceClock])];

    if ((ModuleName == NvRmModuleID_Display) ||
        (ModuleName == NvRmModuleID_Uart) ||
        (ModuleName == NvRmModuleID_Usb2Otg))
        f = MaxFreq;
    else
        f = state->actual_freq;

    v = NvRmPrivModuleVscaleReAttach(
        hRmDevice, cinfo, state, f, SourceClockFreq, Preview);
    return v;
}

NvError
NvRmPowerModuleClockConfig (
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvU32 ClientId,
    NvRmFreqKHz MinFreq,
    NvRmFreqKHz MaxFreq,
    const NvRmFreqKHz* PrefFreqList,
    NvU32 PrefFreqListCount,
    NvRmFreqKHz* CurrentFreq,
    NvU32 flags)
{
    NvError err = NvSuccess;
    NvRmModuleClockInfo *cinfo = NULL;
    NvU32 divisor = 0x0;
    NvU32 reg = 0x0;
    NvRmFreqKHz f, SourceClockFreq;
    ExecPlatform env;
    NvRmModuleClockState *state;
    NvRmMilliVolts v = NvRmVoltsOff;
    NvRmModuleID ModuleName = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 MaxInterfaceClock = 0;

    NvBool DiagMode = NvRmPrivIsDiagMode(ModuleId);

    /* validate the Rm Handle */
    NV_ASSERT(hDevice);
    env = NvRmPrivGetExecPlatform(hDevice);

    // Get pointers to module clock info and current module clock state
    err = NvRmPrivGetClockState(hDevice, ModuleId, &cinfo, &state);
    if (err != NvSuccess)
        return err;

    if ((flags & NvRmClockConfig_SubConfig) &&
        ((ModuleName == NvRmModuleID_Spdif) ||
         (ModuleName == NvRmModuleID_Vi) ||
         (ModuleName == NvRmModuleID_Tvo)))
    {
        // Module subclock is to be configured. Use subclock descriptor
        // and subclock state (located immediately after main descriptor,
        // and state, respectively)
        state++;
        cinfo++;
        NV_ASSERT(cinfo->Module == ModuleName);
        NV_ASSERT(cinfo->SubClockId == 1);
    }
    else if (PrefFreqList && (PrefFreqList[0] == NvRmFreqMaximum) &&
             ((ModuleName == NvRmModuleID_2D)  ||
              (ModuleName == NvRmModuleID_Epp) ||
              (ModuleName == NvRmModuleID_GraphicsHost)))
    {
        // Maximum frequency for these modules is synchronized with EMC
        f = NvRmPrivGetEmcSyncFreq(hDevice, ModuleName);
        if (f == state->actual_freq)
        {
            if (CurrentFreq)
                *CurrentFreq = f;
            return err;             // already in sync
        }
        MaxFreq = f + 1;            // 1 kHz margin
    }
    else if (PrefFreqList &&
             ((ModuleName == NvRmModuleID_Vde) ||
              (ModuleName == NvRmPrivModuleID_MemoryController) ||
              (ModuleName == NvRmPrivModuleID_ExternalMemoryController)))
    {   // CPU, AVP are not allowed too, but failed get state if tried
        NV_ASSERT(!"MC/EMC, VDE clock configuration is not allowed here");
        return NvError_NotSupported;
    }

    // Clip frequency boundaries to h/w limitations
    if (PrefFreqList)
    {
        const NvRmModuleClockLimits* pClimits =
            NvRmPrivGetSocClockLimits(cinfo->Module);
        if ((MinFreq == NvRmFreqUnspecified) ||
            (MinFreq < pClimits->MinKHz))
        {
            MinFreq = pClimits->MinKHz;
        }
        MaxInterfaceClock = NV_MIN(pClimits->MaxKHz,
            NvRmPrivGetInterfaceMaxClock(hDevice, ModuleId));
        if ((MaxFreq == NvRmFreqUnspecified) ||
            (MaxFreq > MaxInterfaceClock))
        {
            MaxFreq = MaxInterfaceClock;
        }
    }

#if NVRM_DIAG_LOCK_SUPPORTED
    // Check/set individual diag lock for this clock only
    DiagMode |= state->DiagLock;
    if (flags & NvRmClockConfig_DiagLock)
        state->DiagLock = NV_TRUE;
#endif

    // Display/DSI clock configuration also affects PLLs shared with DVFS
    // and involves PLLD power control. Always perform at nominal voltage.
    // PMU access transport must *not* be scalable (PMU transport API must
    // be called outside clock mutex).
    if (PrefFreqList && (!DiagMode) &&
        ((ModuleName == NvRmModuleID_Display) ||
         (ModuleName == NvRmModuleID_Dsi)))
    {
        NvOsMutexLock(s_hPllMutex);
        NvRmPrivPllDPowerControl(hDevice, NV_TRUE, &s_MipiPllVddOn);
        NvRmPrivDvsRequest(NvRmVoltsMaximum);
    }

    NvOsSpinMutexLock(s_hClockMutex);
    {
        if (env == ExecPlatform_Fpga || env == ExecPlatform_Qt)
        {
            // Clock configuration only supported for the i2s, VI, i2c,
            // dvc and HSMMC on this environment
            if (!(ModuleName == NvRmModuleID_I2s ||
                    ModuleName == NvRmModuleID_Vi ||
                    ModuleName == NvRmModuleID_Dvc ||
                    ModuleName == NvRmModuleID_I2c ||
                    ModuleName == NvRmModuleID_Hsmmc ||
                    ModuleName == NvRmModuleID_OneWire
                    ))
            {
                // Return actual display clock only on FPGA
                if ((env == ExecPlatform_Fpga) &&
                    (ModuleName == NvRmModuleID_Display))
                {
                    state->actual_freq = FPGA_DISPLAY_KHZ;
                }

                goto end;
            }
        }
        if (PrefFreqList && (!DiagMode))
        {
            if ((ModuleName != NvRmModuleID_Dsi) &&
                (ModuleName != NvRmModuleID_Usb2Otg))
                NV_ASSERT(cinfo->SourceFieldMask || cinfo->DivisorFieldMask);

            // Get the best module source clock and divider
            err = NvRmFindBestClockSource(hDevice, cinfo, NvRmClockSource_Num,
                   MinFreq, MaxFreq, PrefFreqList, PrefFreqListCount, state, flags);
            if (err != NvSuccess)
            {
                goto leave;
            }
            NV_ASSERT(state->SourceClock <= cinfo->SourceFieldMask);

            // For "shared" clocks (Display/DSI) voltage is already at max;
            // just record new voltage requirements
            if ((ModuleName == NvRmModuleID_Display) ||
                (ModuleName == NvRmModuleID_Dsi))
            {
                NvRmPrivModuleVscaleSetPending(hDevice, NvRmVoltsMaximum);
                v = ModuleVscaleConfig(
                    hDevice, cinfo, state, MaxFreq, NV_FALSE);
                NvRmPrivModuleVscaleSetPending(hDevice, NvRmVoltsOff);
            }
            else
            {
                // Preview, don't update scaling ref counts if voltage going up
                v = ModuleVscaleConfig(
                    hDevice, cinfo, state, MaxFreq, NV_TRUE);

                if ((v != NvRmVoltsOff) &&
                    (v != NvRmVoltsUnspecified))
                {
                    // Preview reported voltage increase - set target
                    // pending to prevent DVFS scaling down
                    NvRmPrivModuleVscaleSetPending(hDevice, v);

                    NvOsSpinMutexUnlock(s_hClockMutex);
                    NvOsMutexLock(s_hPllMutex);
                    NvRmPrivDvsRequest(v);
                    NvOsMutexUnlock(s_hPllMutex);
                    NvOsSpinMutexLock(s_hClockMutex);

                    // Now, after voltage is increased - update scaling counts
                    // and cancel pending request
                    v = ModuleVscaleConfig(
                        hDevice, cinfo, state, MaxFreq, NV_FALSE);
                    NvRmPrivModuleVscaleSetPending(hDevice, NvRmVoltsOff);
                }
            }

            // Finally change clock configuration
            if ((ModuleName != NvRmModuleID_Dsi) &&
                (ModuleName != NvRmModuleID_Usb2Otg))
            {
                // Set new clock state
                NvRmPrivModuleClockSet(hDevice, cinfo, state);
                if ((ModuleName == NvRmModuleID_Tvo) &&
                    (cinfo->SubClockId == 1))   // if CVE - sync TVDAC
                {
                    NV_ASSERT(((cinfo + 1)->Module == NvRmModuleID_Tvo) &&
                              ((cinfo + 1)->SubClockId == 2));
                    *(state + 1) = *state;
                    NvRmPrivModuleClockSet(hDevice, (cinfo + 1), state);
                }
                NvRmPrivModuleClockReAttach(hDevice, cinfo, state);
                NvRmPrivDisablePLLs(hDevice, cinfo, state);
            }
            if (v == NvRmVoltsOff)
                NvRmPrivDvsRequest(v); // No transaction, just set update flag

            // FIXME is this a hack just for the AP15 FPGA
            // Special treatment to the i2s on the fpga to do the workaround
            // for the i2s recording, the clock source to i2s should be less than
            // the system clock frequency 8.33MHz for the fpga, so dividing by 2
            // if its more than
            if ((hDevice->ChipId.Id == 0x15 || hDevice->ChipId.Id == 0x16) &&
                    (env == ExecPlatform_Fpga) && (ModuleName == NvRmModuleID_I2s))
            {
                reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                              cinfo->ClkSourceOffset);
                if (!(reg & 0x7f))
                {
                    reg |= 1;
                    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            cinfo->ClkSourceOffset, reg);
                    state->actual_freq = state->actual_freq/2;
                }
            }
            // Hack: on FPGA OneWire divider is implemented as integer divider
            // (on SoC it is fractional divider)
            if ((env == ExecPlatform_Fpga) &&
                (ModuleName == NvRmModuleID_OneWire))
            {
                reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                              cinfo->ClkSourceOffset);
                reg &= ~(cinfo->DivisorFieldMask << cinfo->DivisorFieldShift);
                reg |= (state->Divider >> 1) << cinfo->DivisorFieldShift;
                NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                        cinfo->ClkSourceOffset, reg);
            }
        }
        else    // No target list just update state from h/w and return current frequency
        {
            if (cinfo->SourceFieldMask != 0)
            {
                NV_ASSERT(cinfo->ClkSourceOffset);
                state->SourceClock = NV_REGR(
                    hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset);
                state->SourceClock >>= cinfo->SourceFieldShift;
                state->SourceClock &= cinfo->SourceFieldMask;
                SourceClockFreq = s_ClockSourceFreq[(cinfo->Sources[state->SourceClock])];
            }
            else
            {
                // If source is Fixed source always at index 0
                SourceClockFreq = s_ClockSourceFreq[(cinfo->Sources[0])];
            }
            if ((ModuleName == NvRmPrivModuleID_MemoryController) ||
                (ModuleName == NvRmPrivModuleID_ExternalMemoryController))
                NvRmPrivMemoryClockReAttach(hDevice, cinfo, state);
            else
                NvRmPrivModuleClockReAttach(hDevice, cinfo, state);

            if ( cinfo->Divider  != NvRmClockDivider_None )
            {
                NV_ASSERT(cinfo->ClkSourceOffset);
                state->Divider = NV_REGR(
                    hDevice, NvRmPrivModuleID_ClockAndReset, 0, cinfo->ClkSourceOffset);
                state->Divider >>= cinfo->DivisorFieldShift;
                state->Divider &= cinfo->DivisorFieldMask;

                divisor = state->Divider;
                if (cinfo->Divider == NvRmClockDivider_Integer_1)
                {
                    divisor += 1;
                }
                else if (cinfo->Divider == NvRmClockDivider_Fractional_2)
                {
                    divisor += 2;
                    SourceClockFreq = (SourceClockFreq << 1);
                }
                else if (cinfo->Divider == NvRmClockDivider_Integer_2)
                {
                    divisor += 2;
                }
            }
            else
            {
                state->Divider = 1;
                divisor = 1;
            }
            NV_ASSERT(divisor);
            state->actual_freq = SourceClockFreq / divisor;
        }

        /*
         * VI and I2S has some special bits in the clock register
         */
        NvRmPrivAp15ClockConfigEx(
            hDevice, ModuleName, cinfo->ClkSourceOffset, flags);


        /*
         * SDMMC internal feedback tap delay adjustment
         * This is required for the ap20 based boards.
        */
        if ((PrefFreqListCount) && (hDevice->ChipId.Id == 0x20) &&
                (ModuleName == NvRmModuleID_Sdio))
        {
            NvRmPrivAp20SdioTapDelayConfigure(hDevice, ModuleId,
                    cinfo->ClkSourceOffset, state->actual_freq);
        }
    }

end:
    if (CurrentFreq)
    {
        *CurrentFreq = state->actual_freq;
    }
leave:
    NvOsSpinMutexUnlock(s_hClockMutex);
    if (PrefFreqList && (!DiagMode) &&
        ((ModuleName == NvRmModuleID_Display) ||
         (ModuleName == NvRmModuleID_Dsi)))
    {
        NvRmPrivPllDPowerControl(hDevice, NV_FALSE, &s_MipiPllVddOn);
        NvRmPrivDvsRequest(NvRmVoltsOff);
        NvOsMutexUnlock(s_hPllMutex);
    }
    return err;
}

/*****************************************************************************/

NvRmClockSource
NvRmPrivCoreClockSourceGet(
    NvRmDeviceHandle hRmDevice,
    const NvRmCoreClockInfo* pCinfo)
{
    NvU32 i, reg;
    NvU32 ModeField;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);

    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->SelectorOffset);
    ModeField = (reg >> pCinfo->ModeFieldShift) & pCinfo->ModeFieldMask;
    if (ModeField == 0)
    {
        // One fixed 32kHz clock source, if mode field is cleared
        return NvRmClockSource_ClkS;
    }
    // Selected Clock Mode = 1 + LOG2(mode field)
    for (i = 0; ModeField != 0; ModeField >>= 1, i++);
    NV_ASSERT(i < NvRmCoreClockMode_Num);

    // Source selection index = source field value for currently selected mode
    reg = (reg >> pCinfo->SourceFieldShifts[i]) & pCinfo->SourceFieldMasks[i];
    NV_ASSERT(reg < NvRmClockSource_Num);

    return pCinfo->Sources[reg];
}

NvRmFreqKHz
NvRmPrivCoreClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    const NvRmCoreClockInfo* pCinfo)
{
    NvU32 reg, n, m;
    NvRmFreqKHz ClkFreq;
    NvRmClockSource ClkSrcId;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);

    // Get source frequency
    ClkSrcId = NvRmPrivCoreClockSourceGet(hRmDevice, pCinfo);
    ClkFreq = s_ClockSourceFreq[ClkSrcId];
    NV_ASSERT(ClkFreq);

    // Get divider settings and calculate clock frequency
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->DividerOffset);
    m = (reg >> pCinfo->DividendFieldShift) & pCinfo->DividendFieldMask;
    n = (reg >> pCinfo->DivisorFieldShift) & pCinfo->DivisorFieldMask;
    if ((reg >> pCinfo->DividerEnableFiledShift) & pCinfo->DividerEnableFiledMask)
    {
        if (m < n)  // if enabled and dividend below divisor
        {
            if (n == pCinfo->DivisorFieldMask)
            {
                // special divisor DFS is using
                ClkFreq = (ClkFreq * (m + 1)) >> pCinfo->DivisorFieldSize;
            }
            else
            {
                // initially may be general divisor
                ClkFreq = (ClkFreq * (m + 1)) / (n + 1);
            }
        }
    }
    return ClkFreq;
}

static void
CoreClockSwitch(
    NvRmDeviceHandle hRmDevice,
    const NvRmCoreClockInfo* pCinfo,
    NvU32 SourceIndex,
    NvU32 Divider,
    NvBool SrcFirst,
    NvRmFreqKHz CoreFreq)
{
    NvU32 reg;

    // Construct core source control register settings.
    // Always use Idle clock mode; mode field = 2 ^ (Mode - 1)
    NV_ASSERT(pCinfo->SelectorOffset);
    NV_ASSERT(SourceIndex <= pCinfo->SourceFieldMasks[NvRmCoreClockMode_Idle]);

    reg = ( ((0x1 << (NvRmCoreClockMode_Idle - 1)) << pCinfo->ModeFieldShift) |
            (SourceIndex << pCinfo->SourceFieldShifts[NvRmCoreClockMode_Idle]) );

    if (reg != NV_REGR(
        hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->SelectorOffset))
    {
        // Update PLL reference
        NvRmPrivCoreClockReAttach(
            hRmDevice, pCinfo->SourceId, pCinfo->Sources[SourceIndex]);
    }

    // Switch source and divider according to specified order. This guarantees
    // that core frequency stays below maximum of "old" and "new" settings.
    // Configure EMC LL path before and after clock switch.
    if (pCinfo->SourceId == NvRmClockSource_CpuBus)
        if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
            NvRmPrivAp15SetEmcForCpuSrcSwitch(hRmDevice);
    if (SrcFirst)
    {
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                pCinfo->SelectorOffset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->DividerOffset, Divider);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    if (!SrcFirst)
    {
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                pCinfo->SelectorOffset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
    if (pCinfo->SourceId == NvRmClockSource_CpuBus)
        if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
            NvRmPrivAp15SetEmcForCpuDivSwitch(hRmDevice, CoreFreq, NV_FALSE);
}

void
NvRmPrivCoreClockSourceIndexFind(
    const NvRmCoreClockInfo* pCinfo,
    NvRmClockSource SourceId,
    NvU32* pSourceIndex)
{
    NvU32 i;
    NV_ASSERT(pSourceIndex && pCinfo);
    *pSourceIndex = NvRmClockSource_Num; // source index out of valid range

    // Find core descriptor index for the specified clock source
    for (i = 0; i < NvRmClockSource_Num; i++)
    {
        if (pCinfo->Sources[i] == SourceId)
        {
            *pSourceIndex = i;
            break;
        }
    }
}

void
NvRmPrivCoreClockBestSourceFind(
    const NvRmCoreClockInfo* pCinfo,
    NvRmFreqKHz MaxFreq,
    NvRmFreqKHz TargetFreq,
    NvRmFreqKHz* pSourceFreq,
    NvU32* pSourceIndex)
{
    NvU32 i;
    NvRmFreqKHz SrcFreq = 0;
    NvRmFreqKHz BestSrcFreq = 0;
    NvU32 SrcIndex = NvRmClockSource_Num; // source index out of valid range

    NV_ASSERT(pSourceFreq && pSourceIndex && pCinfo);

    /*
     * Find valid source with frequency closest to the requested one from
     * the above; if such source does not exist, find source with frequency
     * closest to the requested one from the below
     */
    for (i = 0; i < NvRmClockSource_Num; i++)
    {
        SrcFreq = s_ClockSourceFreq[pCinfo->Sources[i]];
        if (SrcFreq == 0)
            continue;
        if (SrcFreq <= MaxFreq)
        {
            if (((BestSrcFreq < SrcFreq) && (BestSrcFreq < TargetFreq)) ||
                ((BestSrcFreq >= SrcFreq) && (SrcFreq >= TargetFreq)))
            {
                SrcIndex = i;
                BestSrcFreq = SrcFreq;
            }
        }
    }
    *pSourceIndex = SrcIndex;
    *pSourceFreq = BestSrcFreq;
}

NvError
NvRmPrivCoreClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmCoreClockInfo* pCinfo,
    NvRmFreqKHz MaxFreq,
    NvRmFreqKHz* pFreq,
    NvRmClockSource* pSourceId)
{
    NvU32 m, n, reg;
    NvBool SrcFirst;
    NvRmFreqKHz ClkFreq;
    NvRmFreqKHz SrcFreq = 0;
    NvU32 SrcIndex = NvRmClockSource_Num; // source index out of valid range

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pFreq && pSourceId && pCinfo);
    NV_ASSERT(*pSourceId < NvRmClockSource_Num);

    // 0 kHz is not achievable, anyway; changing target to 1 kHz will result in
    // minimum configurable frequency
    ClkFreq = *pFreq;
    if (ClkFreq == 0)
        ClkFreq = 1;

    /*
     * If no valid source explicitly specified by the caller, determine the
     * best clock source for the requested frequency. Otherwise, just use the
     * requested source.
     */
    if (*pSourceId == NvRmClockSource_Invalid)
    {
        NvRmPrivCoreClockBestSourceFind(
            pCinfo, MaxFreq, ClkFreq, &SrcFreq, &SrcIndex);
    }
    else
    {
        SrcFreq = s_ClockSourceFreq[*pSourceId];
        if (SrcFreq <= MaxFreq)
        {
            NvRmPrivCoreClockSourceIndexFind(pCinfo, *pSourceId, &SrcIndex);
        }
    }
    if (SrcIndex >= NvRmClockSource_Num)
    {
        // Could not find source
        return NvError_NotSupported;
    }
    NV_ASSERT(SrcFreq);

    /*
     * Determine super divider settings and enable divider if necessary. Always
     * use maximum possible divisor n = divisor mask, so n+1 = 2^(divisor size).
     * Hence, Fout = Fin * (m+1) / (n+1) = (Fin * (m+1)) >> (divisor size), and
     * respectively, m = ((Fout << (divisor size)) / Fin) - do not subtract 1
     * as integer division would round down, anyway. Determine switching order:
     * switch source 1st if new divider quotient is bigger than the old one.
     */
    n = pCinfo->DivisorFieldMask;
    m = (ClkFreq << pCinfo->DivisorFieldSize) / (SrcFreq + 1);
    if ((m < n) && (m <= pCinfo->DividendFieldMask))
    {
        NvU32 m_old, n_old;
        SrcFirst = NV_FALSE;
        reg = NV_REGR(
            hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->DividerOffset);
        if ( ((reg >> pCinfo->DividerEnableFiledShift) &
              pCinfo->DividerEnableFiledMask) == pCinfo->DividerEnableFiledMask )
        {
            m_old = (reg >> pCinfo->DividendFieldShift) & pCinfo->DividendFieldMask;
            n_old = (reg >> pCinfo->DivisorFieldShift) & pCinfo->DivisorFieldMask;
            if ( ((m + 1) * (n_old + 1)) >  ((n + 1) * (m_old + 1)) )
                SrcFirst = NV_TRUE;
        }
        reg = (pCinfo->DividerEnableFiledMask << pCinfo->DividerEnableFiledShift) |
              (m << pCinfo->DividendFieldShift) | (n << pCinfo->DivisorFieldShift);
        // return actual clock frequency from the divider
        *pFreq = (SrcFreq * (m + 1)) >> pCinfo->DivisorFieldSize;
    }
    else
    {
        SrcFirst = NV_TRUE;
        reg = 0; // clear = disable divider
        // return actual clock frequency from the source directly
        *pFreq = SrcFreq;
    }
    // Finally set new core clock
    CoreClockSwitch(hRmDevice, pCinfo, SrcIndex, reg, SrcFirst, *pFreq);

    // return selected source id and update core bus frequency
    *pSourceId = pCinfo->Sources[SrcIndex];
    s_ClockSourceFreq[pCinfo->SourceId] = *pFreq;
    if ((pCinfo->SourceId == NvRmClockSource_CpuBus) &&
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge))
    {
        s_ClockSourceFreq[NvRmClockSource_CpuBridge] = NvRmPrivDividerFreqGet(
            hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge)->pInfo.pDivider);
    }
    return NvSuccess;
}

void
NvRmPrivCoreClockSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmCoreClockInfo* pCinfo,
    NvRmClockSource SourceId,
    NvU32 m,
    NvU32 n)
{
    NvU32 reg;
    NvBool SrcFirst;
    NvRmFreqKHz CoreFreq = 0;
    NvU32 SrcIndex = NvRmClockSource_Num; // source index out of valid range
    ExecPlatform env;


    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);

    env = NvRmPrivGetExecPlatform(hRmDevice);

    if (env == ExecPlatform_Fpga)
        return;

    NvRmPrivCoreClockSourceIndexFind(pCinfo, SourceId, &SrcIndex);
    NV_ASSERT(SrcIndex < NvRmClockSource_Num);

    /*
     * Set divide: just cut off MSbits out of dividend and divisor range, and
     * enable divider if m/n ration is below 1. Update new core frequency.
     * Determine switching order: switch source 1st if new divider quotient is
     * bigger than the old one.
     */
    m &= pCinfo->DividendFieldMask;
    n &= pCinfo->DivisorFieldMask;
    CoreFreq = s_ClockSourceFreq[SourceId];
    if (m < n)
    {
        NvU32 m_old, n_old;
        SrcFirst = NV_FALSE;
        reg = NV_REGR(
            hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->DividerOffset);
        if ( ((reg >> pCinfo->DividerEnableFiledShift) &
              pCinfo->DividerEnableFiledMask) == pCinfo->DividerEnableFiledMask )
        {
            m_old = (reg >> pCinfo->DividendFieldShift) & pCinfo->DividendFieldMask;
            n_old = (reg >> pCinfo->DivisorFieldShift) & pCinfo->DivisorFieldMask;
            if ( ((m + 1) * (n_old + 1)) >  ((n + 1) * (m_old + 1)) )
                SrcFirst = NV_TRUE;
        }
        reg = (pCinfo->DividerEnableFiledMask << pCinfo->DividerEnableFiledShift) |
              (m << pCinfo->DividendFieldShift) | (n << pCinfo->DivisorFieldShift);
        CoreFreq = (CoreFreq * (m + 1)) / (n + 1);
    }
    else
    {
        SrcFirst = NV_TRUE;
        reg = 0; // clear = disable divider
    }
    // Finally set new core clock
    CoreClockSwitch(hRmDevice, pCinfo, SrcIndex, reg, SrcFirst, CoreFreq);

    // update core bus frequency
    s_ClockSourceFreq[pCinfo->SourceId] = CoreFreq;
    if ((pCinfo->SourceId == NvRmClockSource_CpuBus) &&
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge))
    {
        s_ClockSourceFreq[NvRmClockSource_CpuBridge] = NvRmPrivDividerFreqGet(
            hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge)->pInfo.pDivider);
    }
}

/*****************************************************************************/

static NvRmSystemBusComplexInfo*
GetSystemBusComplexHandle(NvRmDeviceHandle hRmDevice)
{
    if (s_SystemBusComplex.BusRateOffset == 0)
    {
        NvU32 i, m;
        const NvRmDividerClockInfo* pAhb =
            NvRmPrivGetClockSourceHandle(NvRmClockSource_Ahb)->pInfo.pDivider;
        const NvRmDividerClockInfo* pApb =
            NvRmPrivGetClockSourceHandle(NvRmClockSource_Apb)->pInfo.pDivider;
        NvOsMemset(&s_SystemBusComplex, 0, sizeof(s_SystemBusComplex));

        // Confirm implied fixed AHB and APB dividers configuration and
        // fill in other AHB and APB dividers parameters
        NV_ASSERT(pAhb->Divider == NvRmClockDivider_Integer_1);
        NV_ASSERT(pAhb->ClkControlField == pAhb->ClkDisableSettings);
        NV_ASSERT(pApb->Divider == NvRmClockDivider_Integer_1);
        NV_ASSERT(pApb->ClkControlField == pApb->ClkDisableSettings);
        NV_ASSERT(pAhb->ClkControlOffset == pApb->ClkControlOffset);

        s_SystemBusComplex.BusRateOffset = pAhb->ClkControlOffset;
        s_SystemBusComplex.BusClockDisableFields =
            pAhb->ClkControlField | pApb->ClkControlField;

        s_SystemBusComplex.HclkDivisorFieldShift = pAhb->ClkRateFieldShift;
        s_SystemBusComplex.HclkDivisorFieldMask = pAhb->ClkRateFieldMask;
        for (i = 0, m = pAhb->ClkRateFieldMask; (m >> i) != 0; i++);
        s_SystemBusComplex.HclkDivisorFieldSize = i;

        s_SystemBusComplex.PclkDivisorFieldShift = pApb->ClkRateFieldShift;
        s_SystemBusComplex.PclkDivisorFieldMask = pApb->ClkRateFieldMask;
        for (i = 0, m = pApb->ClkRateFieldMask; (m >> i) != 0; i++);
        s_SystemBusComplex.PclkDivisorFieldSize = i;

        // Comfirm implied VDE divider configuration, and fill in VDE divider
        // parameters provided System bus complex includes VDE clock; otherwise
        //  leave all VDE parameters cleared.
        if (NvRmPrivGetClockSourceHandle(NvRmClockSource_Vbus))
        {
            const NvRmDividerClockInfo* pVbus =
                NvRmPrivGetClockSourceHandle(NvRmClockSource_Vbus)->pInfo.pDivider;

            NV_ASSERT(pVbus->Divider == NvRmClockDivider_Keeper16);
            NV_ASSERT(pAhb->ClkControlOffset == pVbus->ClkControlOffset);

            s_SystemBusComplex.VclkDividendFieldShift = pVbus->ClkRateFieldShift;
            s_SystemBusComplex.VclkDividendFieldMask = pVbus->ClkRateFieldMask;
            for (i = 0, m = pVbus->ClkRateFieldMask; (m >> i) != 0; i++);
            s_SystemBusComplex.VclkDividendFieldSize = i;
        }
    }
    return &s_SystemBusComplex;
}

void
NvRmPrivBusClockFreqSet(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz SystemFreq,
    NvRmFreqKHz* pVclkFreq,
    NvRmFreqKHz* pHclkFreq,
    NvRmFreqKHz* pPclkFreq,
    NvRmFreqKHz PclkMaxFreq)
{
    NvU32 VclkDividend, HclkDivisor, PclkDivisor, reg;
    NvRmFreqKHz ClkFreq;
    const NvRmSystemBusComplexInfo* pCinfo =
        GetSystemBusComplexHandle(hRmDevice);

    NV_ASSERT(hRmDevice);
    NV_ASSERT(SystemFreq);
    NV_ASSERT(pHclkFreq && pPclkFreq);

    /*
     * AHB clock divider: Fout = System Frequency / (n+1). Divider settings
     * n = System Frequency / Hclk Frequency - 1. Avoid division for extreme
     * cases of very small, or very large request via direct comparison.
     */
    ClkFreq = *pHclkFreq;
    if ((ClkFreq << pCinfo->HclkDivisorFieldSize) <= SystemFreq)
    {
        HclkDivisor = pCinfo->HclkDivisorFieldMask;
        *pHclkFreq = SystemFreq >> pCinfo->HclkDivisorFieldSize;
    }
    else if ((ClkFreq << 1) > SystemFreq)
    {
        HclkDivisor = 0;
        *pHclkFreq = SystemFreq;
    }
    else
    {
        HclkDivisor = (SystemFreq / ClkFreq) - 1;
        *pHclkFreq = SystemFreq / (HclkDivisor + 1);
    }
    s_ClockSourceFreq[NvRmClockSource_Ahb] = *pHclkFreq;

    /*
     * APB clock divider: Fout = AHB Frequency / (n+1). Divider settings
     * n = AHB Frequency / Pclk Frequency - 1. Avoid division for extreme
     * cases of very small, or very large request via direct comparison.
     * Check against clock frequency maximum - this the only one bus clock
     * that may have different (lower) maximum limit.
     */
    ClkFreq = *pPclkFreq;
    NV_ASSERT(ClkFreq <= PclkMaxFreq);
    if ((ClkFreq << pCinfo->PclkDivisorFieldSize) <= (*pHclkFreq))
    {
        PclkDivisor = pCinfo->PclkDivisorFieldMask;
        *pPclkFreq = (*pHclkFreq) >> pCinfo->PclkDivisorFieldSize;
        NV_ASSERT(*pPclkFreq <= PclkMaxFreq);
    }
    else if ((ClkFreq << 1) > (*pHclkFreq))
    {
        PclkDivisor = ((*pHclkFreq) <= PclkMaxFreq)? 0 : 1;
        *pPclkFreq = (*pHclkFreq) >> PclkDivisor;
    }
    else
    {
        PclkDivisor = ((*pHclkFreq) / ClkFreq);
        if ((*pHclkFreq) <= PclkMaxFreq * PclkDivisor)
            PclkDivisor--;
        *pPclkFreq = (*pHclkFreq) / (PclkDivisor + 1);
    }
    s_ClockSourceFreq[NvRmClockSource_Apb] = *pPclkFreq;

    /*
     * V-clock divider: Fout = System Frequency * (n + 1) / 2 ^ dividend size.
     * Divider settings n = (Vclk Frequency << dividend size) / System Frequency.
     * Do not subtract 1 as integer division would round down, anyway. If VDE
     * clock is decoupled from the System bus, clear dividend and return 0 kHz.
     */
    if (pCinfo->VclkDividendFieldMask)
    {
        NV_ASSERT(pVclkFreq);
        if ((*pVclkFreq) >= SystemFreq)
        {
            VclkDividend = pCinfo->VclkDividendFieldMask;
            *pVclkFreq = SystemFreq;
        }
        else
        {
            VclkDividend =
                ((*pVclkFreq) << pCinfo->VclkDividendFieldSize) / (SystemFreq + 1);
            *pVclkFreq =
                (SystemFreq * (VclkDividend + 1)) >> pCinfo->VclkDividendFieldSize;
        }
        s_ClockSourceFreq[NvRmClockSource_Vbus] = *pVclkFreq;
    }
    else
    {
        VclkDividend = 0;
        if (pVclkFreq)
            *pVclkFreq = 0;
        s_ClockSourceFreq[NvRmClockSource_Vbus] = 0;
    }

    /*
     * Set bus clocks dividers in bus rate control register.
     * Always enable all bus clocks.
     */
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->BusRateOffset);
    reg &= ((~pCinfo->BusClockDisableFields) &
            (~(pCinfo->HclkDivisorFieldMask << pCinfo->HclkDivisorFieldShift)) &
            (~(pCinfo->PclkDivisorFieldMask << pCinfo->PclkDivisorFieldShift)) &
            (~(pCinfo->VclkDividendFieldMask << pCinfo->VclkDividendFieldShift)));
    reg |= ((HclkDivisor << pCinfo->HclkDivisorFieldShift) |
            (PclkDivisor << pCinfo->PclkDivisorFieldShift) |
            (VclkDividend << pCinfo->VclkDividendFieldShift));
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->BusRateOffset, reg);
}

void
NvRmPrivBusClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz SystemFreq,
    NvRmFreqKHz* pVclkFreq,
    NvRmFreqKHz* pHclkFreq,
    NvRmFreqKHz* pPclkFreq)
{
    NvU32 VclkDividend, HclkDivisor, PclkDivisor, reg;
    const NvRmSystemBusComplexInfo* pCinfo =
        GetSystemBusComplexHandle(hRmDevice);

    NV_ASSERT(hRmDevice);
    NV_ASSERT(SystemFreq);
    NV_ASSERT(pHclkFreq && pPclkFreq);

    // Get current bus dividers settings
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->BusRateOffset);
    NV_ASSERT((reg & pCinfo->BusClockDisableFields) == 0);

    HclkDivisor = (reg >> pCinfo->HclkDivisorFieldShift) & pCinfo->HclkDivisorFieldMask;
    PclkDivisor = (reg >> pCinfo->PclkDivisorFieldShift) & pCinfo->PclkDivisorFieldMask;
    VclkDividend = (reg >> pCinfo->VclkDividendFieldShift) & pCinfo->VclkDividendFieldMask;

    /*
     * AHB clock divider: Fout = System Frequency / (n+1). Avoid division
     * for extreme cases of min/max divider values.
     */
    if (HclkDivisor == 0)
        *pHclkFreq = SystemFreq;
    else if (HclkDivisor == pCinfo->HclkDivisorFieldMask)
        *pHclkFreq = SystemFreq >> pCinfo->HclkDivisorFieldSize;
    else
        *pHclkFreq = SystemFreq / (HclkDivisor + 1);

    /*
     * APB clock divider: Fout = AHB Frequency / (n+1).  Avoid division
     * for extreme cases of min/max divider values.
     */
    if (PclkDivisor == 0)
        *pPclkFreq = *pHclkFreq;
    else if (PclkDivisor == pCinfo->PclkDivisorFieldMask)
        *pPclkFreq = (*pHclkFreq) >> pCinfo->PclkDivisorFieldSize;
    else
        *pPclkFreq = (*pHclkFreq) / (PclkDivisor + 1);

    /*
     * V-clock divider: Fout = System Frequency * (n + 1) / 2 ^ dividend size.
     * If VDE clock is decoupled from the System bus, return 0 kHz.
     */
    if (pCinfo->VclkDividendFieldMask)
    {
        NV_ASSERT(pVclkFreq);
        *pVclkFreq =
        (SystemFreq * (VclkDividend + 1)) >> pCinfo->VclkDividendFieldSize;
    }
    else if (pVclkFreq)
        *pVclkFreq = 0;
}

/*****************************************************************************/

void
NvRmPrivPllFreqUpdate(
    NvRmDeviceHandle hRmDevice,
    const NvRmPllClockInfo* pCinfo)
{
    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);

    s_ClockSourceFreq[pCinfo->SourceId] =
        NvRmPrivAp15PllFreqGet(hRmDevice, pCinfo);
}

void
NvRmPrivDividerFreqUpdate(
    NvRmDeviceHandle hRmDevice,
    const NvRmDividerClockInfo* pCinfo)
{
    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);

    s_ClockSourceFreq[pCinfo->SourceId] =
        NvRmPrivDividerFreqGet(hRmDevice, pCinfo);
}

void
NvRmPrivDividerSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmDividerClockInfo* pCinfo,
    NvU32 setting)
{
    NvU32 reg;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);
    NV_ASSERT(pCinfo->ClkControlOffset);

    reg = NV_REGR(
        hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkControlOffset);

    // Make sure divider is enabled. Update rate field for divider with
    // variable divisor
    reg &= (~(pCinfo->ClkControlField));
    reg |= pCinfo->ClkEnableSettings;
    if (pCinfo->FixedRateSetting ==  NVRM_VARIABLE_DIVIDER)
    {
        reg &= (~(pCinfo->ClkRateFieldMask << pCinfo->ClkRateFieldShift));
        reg |= ((setting & pCinfo->ClkRateFieldMask) << pCinfo->ClkRateFieldShift);
    }
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkControlOffset, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    s_ClockSourceFreq[pCinfo->SourceId] = NvRmPrivDividerFreqGet(hRmDevice, pCinfo);
}

NvRmFreqKHz
NvRmPrivDividerFreqGet(
    NvRmDeviceHandle hRmDevice,
    const NvRmDividerClockInfo* pCinfo)
{
    NvRmFreqKHz DividerKHz;
    NvU32 reg, n;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);
    NV_ASSERT(pCinfo->ClkControlOffset);

    reg = NV_REGR(
        hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkControlOffset);

    // Return 0 kHz if divider is disabled
    if ((pCinfo->ClkControlField != 0) &&
        ((reg & pCinfo->ClkControlField) == pCinfo->ClkDisableSettings))
    {
        return 0;
    }
    // Determine divider rate setting
    n = pCinfo->FixedRateSetting;
    if (n == NVRM_VARIABLE_DIVIDER)
    {
        n = ((reg >> pCinfo->ClkRateFieldShift) & pCinfo->ClkRateFieldMask);
    }

    // Calculate output frequency
    DividerKHz = s_ClockSourceFreq[pCinfo->InputId];
    switch (pCinfo->Divider)
    {
        case NvRmClockDivider_Keeper16:
            return ((DividerKHz * (n + 1)) >> 4);
        case NvRmClockDivider_Skipper16:
            return ((DividerKHz * (16 - n)) >> 4);
        case NvRmClockDivider_Fractional_2:
            n += 2;
            DividerKHz = DividerKHz << 1;
            break;
        case NvRmClockDivider_Integer_1:
            n += 1;
            break;
        case NvRmClockDivider_Integer:
            break;
        default:
            NV_ASSERT(!"Invalid divider type");
            return 0;
    }
    NV_ASSERT(n != 0);
    return (DividerKHz / n);
}


// Shortcut (this mask can be retrieved from module clock information table)
#define NVRM_FRACTIONAL_DIVISOR_FIELD_MASK (0xFF)

NvU32
NvRmPrivFindFreqMinAbove(
    NvRmClockDivider DividerType,
    NvRmFreqKHz SourceKHz,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pTargetKHz)
{
    NvU32 n;
    NV_ASSERT(pTargetKHz);
    NV_ASSERT( ((*pTargetKHz) != 0) && ((*pTargetKHz) <= MaxKHz) );
    NV_ASSERT(DividerType == NvRmClockDivider_Fractional_2); // only this type

    /*
     * Get fractional divider setting n for the best target approximation from
     * the above. Fractional divider: FoutKHz = (2 * FinKHz) / (n + 2)
     */
    if ((*pTargetKHz) < SourceKHz)
    {
        SourceKHz = SourceKHz << 1;
        n = SourceKHz / (*pTargetKHz);
        if (SourceKHz > n * MaxKHz)
            n++;
        *pTargetKHz = SourceKHz / n;
        n = n - 2;
    }
    else
    {
        n = 0;
        *pTargetKHz = SourceKHz;
    }
    NV_ASSERT(n <= NVRM_FRACTIONAL_DIVISOR_FIELD_MASK);
    return n;
}

NvU32
NvRmPrivFindFreqMaxBelow(
    NvRmClockDivider DividerType,
    NvRmFreqKHz SourceKHz,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pTargetKHz)
{
    NvU32 n;
    NV_ASSERT(pTargetKHz);
    NV_ASSERT( ((*pTargetKHz) != 0) && ((*pTargetKHz) <= MaxKHz) );
    NV_ASSERT(DividerType == NvRmClockDivider_Fractional_2); // only this type

    /*
     * Get fractional divider setting n for the best target approximation from
     * the below. Fractional divider: FoutKHz = (2 * FinKHz) / (n + 2)
     */
    if ((*pTargetKHz) < SourceKHz)
    {
        SourceKHz = SourceKHz << 1;
        n = (SourceKHz + (*pTargetKHz) - 1) / (*pTargetKHz);
        *pTargetKHz = SourceKHz / n;
        n = n - 2;
    }
    else
    {
        n = 0;
        *pTargetKHz = SourceKHz;
    }
    NV_ASSERT(n <= NVRM_FRACTIONAL_DIVISOR_FIELD_MASK);
    return n;
}

void
NvRmPrivSelectorClockSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmSelectorClockInfo* pCinfo,
    NvRmClockSource SourceId,
    NvBool Double)
{
    NvU32 i, reg;
    NvRmFreqKHz SourceFreq;
    NvU32 SrcIndex = NvRmClockSource_Num; // source index out of valid range

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);
    NV_ASSERT(pCinfo->SelectorOffset);

    // Find selector index for the specified input clock source
    for (i = 0; i < NvRmClockSource_Num; i++)
    {
        if (pCinfo->Sources[i] == SourceId)
        {
            SrcIndex = i;
            break;
        }
    }
    NV_ASSERT(SrcIndex < NvRmClockSource_Num);

    // Select specified clock source
    NV_ASSERT(SrcIndex <= pCinfo->SourceFieldMask);
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->SelectorOffset);
    reg &= (~(pCinfo->SourceFieldMask << pCinfo->SourceFieldShift));
    reg |= (SrcIndex << pCinfo->SourceFieldShift);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->SelectorOffset, reg);
    SourceFreq = s_ClockSourceFreq[SourceId];

    // Enable/Disable doubler
    if (pCinfo->DoublerEnableField != 0)
    {
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                      pCinfo->DoublerEnableOffset);
        if (Double)
        {
            reg |= pCinfo->DoublerEnableField;
            SourceFreq = SourceFreq << 1;
        }
        else
        {
            reg &= (~pCinfo->DoublerEnableField);
            SourceFreq = 0; // no clock out if doubler disabled
        }
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                pCinfo->DoublerEnableOffset, reg);
    }
    s_ClockSourceFreq[pCinfo->SourceId] = SourceFreq;
}

/*****************************************************************************/

void NvRmPrivParseClockSources(
    NvRmClockSourceInfo* pDst,
    NvU32 DestinationTableSize,
    NvRmClockSourceInfoPtr Src,
    NvU32 SourceTableSize,
    NvRmClockSourceType SourceType)
{
    NvU32 i;
    NvRmClockSource id = NvRmClockSource_Invalid;
    NV_ASSERT(pDst);

    for (i = 0; i < SourceTableSize; i++)
    {
        // Bsed on specified source type retrieve source id
        // from the source table
        switch (SourceType)
        {
            case NvRmClockSourceType_Fixed:
                id = Src.pFixed[i].SourceId;
                pDst[id].pInfo.pFixed = &Src.pFixed[i];
                break;
            case NvRmClockSourceType_Pll:
                id = Src.pPll[i].SourceId;
                pDst[id].pInfo.pPll = &Src.pPll[i];
                break;
            case NvRmClockSourceType_Divider:
                id = Src.pDivider[i].SourceId;
                pDst[id].pInfo.pDivider = &Src.pDivider[i];
                break;
            case NvRmClockSourceType_Core:
                id = Src.pCore[i].SourceId;
                pDst[id].pInfo.pCore = &Src.pCore[i];
                break;
            case NvRmClockSourceType_Selector:
                id = Src.pSelector[i].SourceId;
                pDst[id].pInfo.pSelector = &Src.pSelector[i];
                break;
            default:
                NV_ASSERT(!"Not defined source type");
        }
        // Fill in destination table
        NV_ASSERT((NvU32)id < DestinationTableSize);
        NV_ASSERT(pDst[id].SourceId == NvRmClockSource_Invalid);
        pDst[id].SourceId = id;
        pDst[id].SourceType = SourceType;
    }
}

NvRmClockSourceInfo* NvRmPrivGetClockSourceHandle(NvRmClockSource id)
{
    NvRmClockSourceInfo* pSource = NULL;

    NV_ASSERT((id != NvRmClockSource_Invalid) && (id < NvRmClockSource_Num));
    if (s_ClockSourceTable[id].SourceId == id)
    {
         pSource = &s_ClockSourceTable[id];
         NV_ASSERT(pSource->pInfo.pFixed);
    }
    return pSource;
}

NvRmFreqKHz
NvRmPrivGetClockSourceFreq(NvRmClockSource id)
{
    NV_ASSERT(id < NvRmClockSource_Num);
    return s_ClockSourceFreq[id];
}

NvRmFreqKHz
NvRmPowerGetPrimaryFrequency(
    NvRmDeviceHandle hRmDeviceHandle)
{
    return s_ClockSourceFreq[NvRmClockSource_ClkM];
}

NvBool
NvRmPrivIsSourceSelectedByModule(
    NvRmDeviceHandle hRmDevice,
    NvRmClockSource SourceId,
    NvRmModuleID ModuleId)
{
    NvError Error;
    NvU32 SourceIndex = 0;
    NvRmModuleClockInfo* pCinfo;
    NvRmModuleInstance* pInst = NULL;
    NV_ASSERT(hRmDevice);

    Error = NvRmPrivGetModuleInstance(hRmDevice, ModuleId, &pInst);
    if (Error != NvSuccess)
        return NV_FALSE;     // Module is not present - not using anything

    pCinfo = (NvRmModuleClockInfo*)pInst->ModuleData;
    if (pCinfo->ClkSourceOffset != 0)
    {
        SourceIndex = NV_REGR(
            hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkSourceOffset);
        SourceIndex =
            (SourceIndex >> pCinfo->SourceFieldShift) & pCinfo->SourceFieldMask;
    }
    return (pCinfo->Sources[SourceIndex] == SourceId);
}

NvBool
NvRmIsFreqRangeReachable(
    NvRmFreqKHz SourceFreq,
    NvRmFreqKHz MinFreq,
    NvRmFreqKHz MaxFreq,
    NvU32 MaxDivisor)
{
    NvU32 divisor;
    NV_ASSERT(SourceFreq && MaxFreq);
    NV_ASSERT(MinFreq <= MaxFreq);

    // Determine minimum divisor that satisfies maximum boundary
    divisor = SourceFreq / MaxFreq;
    if ((divisor * MaxFreq) < SourceFreq)
    {
        divisor += 1;
    }
    // The specified range is reachable if minimum divisor is
    // fits divisor field and satisfies minimum boundary
    if ((divisor <= MaxDivisor) &&
        ((divisor * MinFreq) <= SourceFreq))
    {
        return NV_TRUE;
    }
    return NV_FALSE;
}

const NvRmModuleClockLimits*
NvRmPrivGetSocClockLimits(NvRmModuleID Module)
{
    NV_ASSERT(Module < NvRmPrivModuleID_Num);
    return &s_ModuleClockLimits[Module];
}

void NvRmPrivLockSharedPll(void)
{
    NvOsMutexLock(s_hPllMutex);
}

void NvRmPrivUnlockSharedPll(void)
{
    NvOsMutexUnlock(s_hPllMutex);
}

void NvRmPrivLockModuleClockState(void)
{
    NvOsSpinMutexLock(s_hClockMutex);
}

void NvRmPrivUnlockModuleClockState(void)
{
    NvOsSpinMutexUnlock(s_hClockMutex);
}

/*****************************************************************************/
/*****************************************************************************/

// PLLC may be selected as a source only for Display, TVO, GPU, and VDE
// modules. (It is also used for CPU and System/Avp core clocks, controlled
// by DFS with its own configuration path - no need to specify here)
static const NvRmModuleID s_Ap15PllC0UsagePolicy[] =
{
    NvRmModuleID_Display,
    NvRmModuleID_3D,
    NvRmModuleID_2D,
    NvRmModuleID_Mpe,
    NvRmModuleID_Hdmi,
};

static const NvRmModuleID s_Ap20PllC0UsagePolicy[] =
{
    NvRmModuleID_Display,
    NvRmModuleID_Tvo,
    NvRmModuleID_3D,
    NvRmModuleID_2D,
    NvRmModuleID_Epp,
    NvRmModuleID_Mpe,
    NvRmModuleID_Hdmi,
    NvRmModuleID_Vde
};

// PLLM may be selected as a source for GPU, UART and VDE modules. (It is also
// used for EMC, CPU and System/Avp core clocks, controlled by DFS with its
// own configuration path - no need to specify here)
static const NvRmModuleID s_Ap15PllM0UsagePolicy[] =
{
    NvRmModuleID_GraphicsHost,
    NvRmModuleID_Vi,
    NvRmModuleID_3D,
    NvRmModuleID_2D,
    NvRmModuleID_Epp,
    NvRmModuleID_Mpe,
    NvRmModuleID_Vde,
    NvRmModuleID_Uart
};

// PLLD may be selected as a source only for Display, HDMI, and DSI modules.
static const NvRmModuleID s_Ap15PllD0UsagePolicy[] =
{
    NvRmModuleID_Display,
    NvRmModuleID_Hdmi,
    NvRmModuleID_Dsi
};

// PLLA may be selected as a source only for I2S and SPDIF modules.
static const NvRmModuleID s_Ap15PllA0UsagePolicy[] =
{
    NvRmModuleID_I2s,
    NvRmModuleID_Spdif,
};

static const NvRmModuleID*
GetPolicySourceToModuleList(
    NvRmDeviceHandle hRmDevice,
    NvRmClockSource SourceId,
    NvU32* pListSize)
{
    NV_ASSERT(hRmDevice && pListSize);

    // Unless explicitly overwritten, use AP15 policy as a base for all SoCs;
    // return list of modules that may use specified source
    switch (SourceId)
    {
        case NvRmClockSource_PllC0:
            if (hRmDevice->ChipId.Id == 0x20)
            {
                *pListSize = NV_ARRAY_SIZE(s_Ap20PllC0UsagePolicy);
                return s_Ap20PllC0UsagePolicy;
            }
            *pListSize = NV_ARRAY_SIZE(s_Ap15PllC0UsagePolicy);
            return s_Ap15PllC0UsagePolicy;

        case NvRmClockSource_PllM0:
            *pListSize = NV_ARRAY_SIZE(s_Ap15PllM0UsagePolicy);
            return s_Ap15PllM0UsagePolicy;

        case NvRmClockSource_PllD0:
            *pListSize = NV_ARRAY_SIZE(s_Ap15PllD0UsagePolicy);
            return s_Ap15PllD0UsagePolicy;

        case NvRmClockSource_PllA0:
        case NvRmClockSource_AudioSync:
            *pListSize = NV_ARRAY_SIZE(s_Ap15PllA0UsagePolicy);
            return s_Ap15PllA0UsagePolicy;

        default:
            *pListSize = 0;
            return NULL;    // No policy - any module may use the source
    }
}

NvBool
NvRmPrivIsSourceProtected(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module,
    NvRmClockSource SourceId)
{
    NvU32 i, ListSize;
    const NvRmModuleID* pModuleList = GetPolicySourceToModuleList(
        hRmDevice, SourceId, &ListSize);

    if (pModuleList)
    {
        // Policy in place - check the module against it
        NV_ASSERT(ListSize);
        for (i = 0; i < ListSize; i++)
        {
            if (Module == pModuleList[i])
                return NV_FALSE;
        }
        return NV_TRUE;
    }
    else
    {
        // No policy for this source - just make sure I2C module is
        // on main clock only
        if (SourceId != NvRmClockSource_ClkM)
        {
            if ((Module == NvRmModuleID_Dvc) ||
                (Module == NvRmModuleID_I2c))
                return NV_TRUE;
        }
        return NV_FALSE;
    }
}

/*****************************************************************************/

void
NvRmPrivReConfigurePllX(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz TargetFreq)
{
    NvRmClockSource SourceId;
    NvRmFreqKHz f = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllX0);
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
    NvRmFreqKHz MaxFreq = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;

    NV_ASSERT(NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0));
    NV_ASSERT(TargetFreq <= MaxFreq);

    // Do nothing if current PLLX frequency is below
    // and close enough to the target
    if (f <= TargetFreq)                            // if below - DVS-safe
    {
        f += (MaxFreq >> pCinfo->DivisorFieldSize); // CPU divider resolution
        if (f >= TargetFreq)
            return;
    }

    /*
     * If PLLX is in use by CPU switch CPU to back-up PLLP0 source during PLLX
     * reconfiguration. This is DVS safe as per DFS policy, PLLX is used for
     * high frequencies above PLLP0 output. In any case, configure PLLX target
     * frequency, and let the caller to complete CPU clock configuration (PLLX
     * is used for CPU only, so the caller is always CPU DVFS)
     */
    SourceId = NvRmPrivCoreClockSourceGet(hRmDevice, pCinfo);
    if (SourceId == NvRmClockSource_PllX0)
    {
        SourceId = NvRmClockSource_PllP0;
        f = NvRmPrivGetClockSourceFreq(SourceId);
        NV_ASSERT(f <= MaxFreq);
        NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
            hRmDevice, pCinfo, MaxFreq, &f, &SourceId));
    }
    SourceId = NvRmClockSource_PllX0;
    NvRmPrivAp15PllConfigureSimple(hRmDevice, SourceId, TargetFreq, &TargetFreq);
}

/*****************************************************************************/

static void BackupClockSource(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleClockInfo* pCinfo,
    NvRmClockSource BackupSource)
{
    NvBool Disabled;
    NvU32 reg, SourceIndex;
    NvRmModuleID ModuleId;

    NV_ASSERT(pCinfo);
    ModuleId = NVRM_MODULE_ID(pCinfo->Module, pCinfo->Instance);

    // Check if currently clock is disabled
    NV_ASSERT(pCinfo->ClkEnableOffset);
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  pCinfo->ClkEnableOffset);
    Disabled = ((reg & pCinfo->ClkEnableField) != pCinfo->ClkEnableField);

    // Find backup source index
    for (SourceIndex = 0; SourceIndex < NvRmClockSource_Num; SourceIndex++)
    {
        if (pCinfo->Sources[SourceIndex] == BackupSource)
            break;
    }
    NV_ASSERT(SourceIndex < NvRmClockSource_Num);

    // Switch module to backup source clock. If module clock is disabled,
    // temporarily enable it.
    if (Disabled)
    {
        NvRmPrivEnableModuleClock(hRmDevice, ModuleId, ModuleClockState_Enable);
    }
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  pCinfo->ClkSourceOffset);
    reg &= (~(pCinfo->SourceFieldMask << pCinfo->SourceFieldShift));
    reg |= (SourceIndex << pCinfo->SourceFieldShift);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->ClkSourceOffset, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    if (Disabled)
    {
        NvRmPrivEnableModuleClock(hRmDevice, ModuleId, ModuleClockState_Disable);
    }
}

static void RestoreClockSource(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleClockInfo* pCinfo,
    NvRmModuleClockState* pCstate,
    NvRmFreqKHz NewSourceFreq)
{
    NvU32 reg;
    NvBool Disabled;
    NvRmModuleID ModuleId;

    NV_ASSERT(pCinfo && pCstate);
    ModuleId = NVRM_MODULE_ID(pCinfo->Module, pCinfo->Instance);

    // Check if currently clock is disabled
    NV_ASSERT(pCinfo->ClkEnableOffset);
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  pCinfo->ClkEnableOffset);
    Disabled = ((reg & pCinfo->ClkEnableField) != pCinfo->ClkEnableField);

    // Restore module clock source  If module clock is disabled, temporarily
    // enable it. Update module v-scale requirements.
    if (Disabled)
    {
        NvRmPrivEnableModuleClock(hRmDevice, ModuleId, ModuleClockState_Enable);
    }
    NvRmPrivModuleClockSet(hRmDevice, pCinfo, pCstate);
    if (Disabled)
    {
        NvRmPrivEnableModuleClock(hRmDevice, ModuleId, ModuleClockState_Disable);
    }
    NvRmPrivModuleVscaleReAttach(hRmDevice,
        pCinfo, pCstate, pCstate->actual_freq, NewSourceFreq, NV_FALSE);
}

static void BackupModuleClocks(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module,
    NvRmClockSource UpdatedSource,
    NvRmClockSource BackupSource)
{
    NvU32 j;
    NvBool SubClock = NV_FALSE;
    NvRmModuleClockInfo* pCinfo = NULL;
    NvRmModuleClockState* pCstate = NULL;

    for (j = NvRmModuleGetNumInstances(hRmDevice, Module); j != 0; j--)
    {
        NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
            hRmDevice, NVRM_MODULE_ID(Module, j-1), &pCinfo, &pCstate));
        do
        {
            // If on updated source, switch module to backup source. Note
            // that module clock state records are preserved and will be used
            // to restore clock configuration after source update completed.
            NV_ASSERT(NvRmPrivGetClockSourceFreq(BackupSource) <=
                      NvRmPrivGetClockSourceFreq(UpdatedSource));
            if (pCinfo->Sources[pCstate->SourceClock] == UpdatedSource)
                BackupClockSource(hRmDevice, pCinfo, BackupSource);

            // Check if module subclock should be backed up as well
            // TODO: boundary check
            pCinfo++;
            pCstate++;
            SubClock = (pCinfo->Module == Module) &&
                       (pCinfo->Instance == (j - 1));
        } while (SubClock);
    }
}

static void
RestoreModuleClocks(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module,
    NvRmClockSource UpdatedSource,
    NvRmFreqKHz NewSourceFreq)
{
    NvU32 j;
    NvRmFreqKHz MaxFreq;
    NvBool SubClock = NV_FALSE;
    NvRmModuleClockInfo* pCinfo = NULL;
    NvRmModuleClockState* pCstate = NULL;

    MaxFreq = NvRmPrivGetSocClockLimits(Module)->MaxKHz;
    for (j = NvRmModuleGetNumInstances(hRmDevice, Module); j != 0; j--)
    {
        NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
            hRmDevice, NVRM_MODULE_ID(Module, j-1), &pCinfo, &pCstate));
        do
        {
            // Restore updated module clock source, and set divider to get as
            // close/above to previous frequency as new source output allows.
            if (pCinfo->Sources[pCstate->SourceClock] == UpdatedSource)
            {
                pCstate->Divider = NvRmPrivFindFreqMinAbove(
                    pCinfo->Divider, NewSourceFreq, MaxFreq, &pCstate->actual_freq);
                RestoreClockSource(hRmDevice, pCinfo, pCstate, NewSourceFreq);
            }

            // Check if module subclock should be backed up as well
            // TODO: boundary check
            pCinfo++;
            pCstate++;
            SubClock = (pCinfo->Module == Module) &&
                       (pCinfo->Instance == (j - 1));
        } while (SubClock);
    }
}

/*****************************************************************************/

static void PllCBackupModuleClocks(NvRmDeviceHandle hRmDevice)
{
    NvU32 i, ListSize;
    NvRmModuleID Module;
    const NvRmModuleID* pModuleList = GetPolicySourceToModuleList(
        hRmDevice, NvRmClockSource_PllC0, &ListSize);
    NV_ASSERT(pModuleList && ListSize);

    // Check all modules that can use PLLC0 as a source, and switch to PLLP0
    // as a backcup source
    for (i = 0; i < ListSize; i++)
    {
        Module = pModuleList[i];
        BackupModuleClocks(
            hRmDevice, Module, NvRmClockSource_PllC0, NvRmClockSource_PllP0);
    }
}

static void
PllCRestoreModuleClocks(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz NewPllCFreq)
{
    NvU32 i, ListSize;
    NvRmModuleID Module;
    const NvRmModuleID* pModuleList = GetPolicySourceToModuleList(
        hRmDevice, NvRmClockSource_PllC0, &ListSize);
    NV_ASSERT(pModuleList && ListSize);

    // Check all modules that can use PLLC0 as a source, and restore source
    // configuration
    for (i = 0; i < ListSize; i++)
    {
        // Skip display (PLLC is adjusted as part of display configuration)
        Module = pModuleList[i];
        if (Module == NvRmModuleID_Display)
            continue;

        RestoreModuleClocks(
            hRmDevice, Module, NvRmClockSource_PllC0, NewPllCFreq);
    }
}

static NvRmFreqKHz PllCBackupCpuClock(NvRmDeviceHandle hRmDevice)
{
    NvRmClockSource SourceId;
    NvRmFreqKHz OldCpuFreq = 0;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;

    // If PLLC0 is used as a source for CPU clock - switch CPU to PLLP0, and
    // return saved CPU clock frequency (to be restored). Note that DVFS uses
    // PLLC0 as a source only for frequencies above PLLP0
    SourceId = NvRmPrivCoreClockSourceGet(hRmDevice, pCinfo);
    if (SourceId == NvRmClockSource_PllC0)
    {
        OldCpuFreq = NvRmPrivGetClockSourceFreq(NvRmClockSource_CpuBus);
        NV_ASSERT(NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0) <=
                  OldCpuFreq);
        NV_ASSERT(NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0) <=
                  NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz);
        NvRmPrivCoreClockSet(hRmDevice, pCinfo, NvRmClockSource_PllP0, 0, 0);
    }
    return OldCpuFreq; // frequency for restoration, or 0 if no restoration
}

static void
PllCRestoreCpuClock(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz NewPllCFreq,
    NvRmFreqKHz OldCpuFreq)
{
    // Restore CPU clock as high as new PLLC0 output allows, provoded PLLC0
    // was used as a source for CPU
    if (OldCpuFreq != 0)
    {
        NvRmClockSource SourceId = NvRmClockSource_PllC0;
        NvRmFreqKHz CpuFreq = NV_MIN(NewPllCFreq, OldCpuFreq);
        const NvRmCoreClockInfo* pCinfo =
            NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
        NvRmFreqKHz MaxFreq =
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;

        NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
            hRmDevice, pCinfo, MaxFreq, &CpuFreq, &SourceId));
    }
}

static NvRmFreqKHz PllCBackupSystemClock(NvRmDeviceHandle hRmDevice)
{
    NvRmClockSource SourceId;
    NvRmFreqKHz OldSysFreq = 0;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore;

    // If PLLC1 divider output is used as a source for System clock - switch
    // System clock to to PLLP2, and return saved System clock frequency (to
    // be restored). Note that DVFS uses PLLC1 as a source starting with AP20
    SourceId = NvRmPrivCoreClockSourceGet(hRmDevice, pCinfo);
    if (SourceId == NvRmClockSource_PllC1)
    {
        OldSysFreq = NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus);
        NV_ASSERT(hRmDevice->ChipId.Id >= 0x20);
        NV_ASSERT(NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP2) <=
                  NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System)->MaxKHz);
        NvRmPrivCoreClockSet(hRmDevice, pCinfo, NvRmClockSource_PllP2, 0, 0);
    }
    return OldSysFreq; // frequency for restoration, or 0 if no restoration
}

static void
PllCRestoreSystemClock(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz NewPllCFreq,
    NvRmFreqKHz OldSysFreq)
{
    NvU32 divc1;
    NvRmFreqKHz SysFreq;
    const NvRmClockSourceInfo* pSrcCinfo;
    NvRmClockSource SourceId = NvRmClockSource_PllC1;
    NvRmFreqKHz MaxFreq =
        NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System)->MaxKHz;

    // Reconfigure PLLC1 divider at maximum possible frequency
    SysFreq = MaxFreq;
    divc1 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, NewPllCFreq, MaxFreq, &SysFreq);
    pSrcCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_PllC1);
    NvRmPrivDividerSet(hRmDevice, pSrcCinfo->pInfo.pDivider, divc1);

    // Restore System clock as high as new PLLC1 output allows, provoded PLLC1
    // was used as a source for System clock
    if (OldSysFreq != 0)
    {
        SysFreq = NV_MIN(SysFreq, OldSysFreq);
        pSrcCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus);
        NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
            hRmDevice, pSrcCinfo->pInfo.pCore, MaxFreq, &SysFreq, &SourceId));
        NvRmPrivBusClockInit(hRmDevice, SysFreq);
    }
}

NvRmFreqKHz NvRmPrivGetMaxFreqPllC(NvRmDeviceHandle hRmDevice)
{
    // PLLC maximum limit is fixed for SoC with dedicated CPU PLLX; otherwise
    // it is equal to CPU maximum frequency, as PLLC is a primary CPU source.
    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0))
        return NVRM_PLLC_DEFAULT_FREQ_KHZ;
    else
        return NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
}

/*
 * PLLC is reconfigured:
 * (a) when RM is setting fast clocks during boot/resume from deep sleep,
 *     provided PLLC is not already in use by any of the display heads
 * (b) when DDK/ODM is reconfiguring display clock (typically PLLC is required
 *     for CRT)
 *
 * In both cases core voltage is set at nominal - reconfiguration is DVS-save.
 * Core clocks that use PLLC: CPU and System bus (starting with AP20) - are
 * switched to PLLP during reconfiguration and restored afterwards. Module
 * clocks that use PLLC are backed up to PLLP and then restored as well, with
 * the exception of display, which does not need restoration in a process of
 * reconfiguration (case b).
 */
void
NvRmPrivReConfigurePllC(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz TargetFreq)
{
    NvRmFreqKHz CpuFreq, SysFreq, MaxFreq;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
    NvBool IsHdmi = NvRmIsFixedHdmiKHz(TargetFreq);

    // If maximum PLLC target is requested, and current PLLC output is close
    // enough - exit without adjusting PLLC (use CPU divider resolution as
    // "close enough" criteria). For specific PLLC target, find multiple of
    // target frequency as close as possible to PLLC maximum limit.
    MaxFreq = NvRmPrivGetMaxFreqPllC(hRmDevice);
    if (TargetFreq == NvRmFreqMaximum)
    {
        TargetFreq = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
        if (TargetFreq <= MaxFreq)
        {
             TargetFreq += (MaxFreq >> pCinfo->DivisorFieldSize);
             if (TargetFreq >= MaxFreq)
                 return;
        }
        TargetFreq = MaxFreq;
    }
    NV_ASSERT(TargetFreq <= MaxFreq);
    NV_ASSERT((TargetFreq * NVRM_DISPLAY_DIVIDER_MAX) >= MaxFreq);
    TargetFreq = (MaxFreq / TargetFreq) * TargetFreq;

    // Backup core and/or module clocks that are using PLLC as a clock source
    // at the moment. Reconfigure PLLC to the new target, and restore backuped
    // clocks as close as possible with the new PLLC output frequency
    CpuFreq = PllCBackupCpuClock(hRmDevice);
    SysFreq = PllCBackupSystemClock(hRmDevice);
    PllCBackupModuleClocks(hRmDevice);

    // For 720p or 1080i/1080p HDMI - use fixed PLLC configuration;
    // for other targets use simple variable configuration
    if (IsHdmi)
        NvRmPrivAp15PllConfigureHdmi(
            hRmDevice, NvRmClockSource_PllC0, &TargetFreq);
    else
        NvRmPrivAp15PllConfigureSimple(
            hRmDevice, NvRmClockSource_PllC0, TargetFreq, &TargetFreq);

    PllCRestoreCpuClock(hRmDevice, TargetFreq, CpuFreq);
    PllCRestoreSystemClock(hRmDevice, TargetFreq, SysFreq);
    PllCRestoreModuleClocks(hRmDevice, TargetFreq);

#if !NV_OAL
        // Resync DFS as PLLC may be reconfigured for display "behind DFS back"
        if (NvRmPrivGetExecPlatform(hRmDevice) == ExecPlatform_Soc)
            NvRmPrivDfsResync();
#endif
}

void NvRmPrivBoostPllC(NvRmDeviceHandle hRmDevice)
{
    // Boost PLLC to maximum output, if it is not used as pixel clock source
    if (!NvRmPrivIsSourceSelectedByModule(hRmDevice, NvRmClockSource_PllC0,
            NVRM_MODULE_ID(NvRmModuleID_Display, 0)) &&
        !NvRmPrivIsSourceSelectedByModule(hRmDevice, NvRmClockSource_PllC0,
            NVRM_MODULE_ID(NvRmModuleID_Display, 1))
        )
        NvRmPrivReConfigurePllC(hRmDevice, NvRmFreqMaximum);
}
