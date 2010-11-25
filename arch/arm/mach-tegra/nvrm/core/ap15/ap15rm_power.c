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
 *           Power Resource manager </b>
 *
 * @b Description: Implements the interface of the NvRM Power.
 *
 */

#include "nvrm_power_private.h"
#include "nvrm_pmu.h"
#include "nvrm_pmu_private.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvodm_query_discovery.h"
#include "ap15rm_private.h"
#include "ap15rm_clocks.h"
#include "ap15/arapbpm.h"
#include "ap15/project_relocation_table.h"

// TODO: Always Disable before check-in
// Module debug: 0=disable, 1=enable
#define NVRM_ENABLE_PRINTF (0)

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_POWER_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_POWER_PRINTF(x)
#endif


#if !NV_OAL
/*****************************************************************************/
/*****************************************************************************/

#define NV_POWER_GATE_TD    (0)
#define NV_POWER_GATE_PCIE  (1)
//  TODO: check VDE/BSEV/NSEA voltage control calls before enabling
#define NV_POWER_GATE_VDE   (0)
//  TODO: check MPE voltage control calls before enabling
#define NV_POWER_GATE_MPE   (0)

// Power Group -to- Power Gating Ids mapping
static const NvU32* s_PowerGroupIds = NULL;
static NvBool s_UngateOnResume[NV_POWERGROUP_MAX] = {0};

/*****************************************************************************/

static NvBool IsRunTimePowerGateSupported(NvU32 PowerGroup)
{
    // 1st check h/w support capabilities
    NV_ASSERT(s_PowerGroupIds);
    if (s_PowerGroupIds[PowerGroup] == NV_POWERGROUP_INVALID)
        return NV_FALSE;

    // now check s/w support
    switch (PowerGroup)
    {
        case NV_POWERGROUP_TD:
            return NV_POWER_GATE_TD;
        case NV_POWERGROUP_PCIE:
            return NV_POWER_GATE_PCIE;
        case NV_POWERGROUP_VDE:
            return NV_POWER_GATE_VDE;
        case NV_POWERGROUP_MPE:
            return NV_POWER_GATE_MPE;
        default:
            return NV_FALSE;
    }
}

static NvBool IsSuspendPowerGateForced(NvU32 PowerGroup)
{
    // 1st check h/w support capabilities
    NV_ASSERT(s_PowerGroupIds);
    if (s_PowerGroupIds[PowerGroup] == NV_POWERGROUP_INVALID)
        return NV_FALSE;

    // now check s/w support
    switch (PowerGroup)
    {
        case NV_POWERGROUP_PCIE:
        case NV_POWERGROUP_VDE:
        case NV_POWERGROUP_VE:
        case NV_POWERGROUP_MPE:
            return NV_TRUE;
        default:
            return NV_FALSE;
    }
}

static void PowerGroupResetControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup,
    NvBool Assert)
{
    switch (PowerGroup)
    {
        case NV_POWERGROUP_TD:
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_3D, Assert);
            break;
        case NV_POWERGROUP_PCIE:
            NvRmModuleResetWithHold(
                hRmDeviceHandle, NvRmPrivModuleID_Pcie, Assert);
            NvRmModuleResetWithHold(
                hRmDeviceHandle, NvRmPrivModuleID_Afi, Assert);
            if (Assert)     // Keep PCIEXCLK in reset - let driver to take it out
            {
                NvRmModuleResetWithHold(
                    hRmDeviceHandle, NvRmPrivModuleID_PcieXclk, Assert);
            }
            NvRmPrivAp20PowerPcieXclkControl(hRmDeviceHandle, !Assert);
            break;
        case NV_POWERGROUP_VDE:
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Vde, Assert);
            break;
        case NV_POWERGROUP_VE:
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Vi, Assert);
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Csi, Assert);
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Isp, Assert);
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Epp, Assert);
            break;
        case NV_POWERGROUP_MPE:
            NvRmModuleResetWithHold(hRmDeviceHandle, NvRmModuleID_Mpe, Assert);
            break;
        default:
            break;
    }
}

static void  PowerGroupClockControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup,
    NvBool Enable)
{
    ModuleClockState ClockState =
        Enable ? ModuleClockState_Enable : ModuleClockState_Disable;

    switch (PowerGroup)
    {
        case NV_POWERGROUP_TD:
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_3D, ClockState);
            break;
        case NV_POWERGROUP_PCIE:
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmPrivModuleID_Pcie, ClockState);
            break;
        case NV_POWERGROUP_VDE:
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Vde, ClockState);
            break;
        case NV_POWERGROUP_VE:
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Vi, ClockState);
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Csi, ClockState);
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Isp, ClockState);
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Epp, ClockState);
            break;
        case NV_POWERGROUP_MPE:
            NvRmPrivEnableModuleClock(
                hRmDeviceHandle, NvRmModuleID_Mpe, ClockState);
            break;
        default:
            break;
    }
}

static void
PowerGroupPowerControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup,
    NvBool Enable)
{
    NvU32 reg, Id, Mask, Status;

    // Do nothing if not SoC platform
    NV_ASSERT(hRmDeviceHandle);
    if (NvRmPrivGetExecPlatform(hRmDeviceHandle) != ExecPlatform_Soc)
        return;

    // Do nothing if power group is already in requested state
    NV_ASSERT(s_PowerGroupIds[PowerGroup] != NV_POWERGROUP_INVALID);
    Id = s_PowerGroupIds[PowerGroup];
    Mask = (0x1 << Id);
    Status = Mask & NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
                            APBDEV_PMC_PWRGATE_STATUS_0);
    if (Enable == (Status != 0x0))
        return;

    /*
     * Gating procedure:
     * - assert resets to all modules in power group
     * - toggle power gate
     *
     * Ungating procedure
     * - assert resets to all modules in power group (redundunt)
     * - toggle power gate
     * - enable clocks to all modules in power group
     * - reset propagation delay
     * - remove clamping
     * - disable clocks to all modules in power group
     * - de-assert reset to all modules in power group
     *
     * Special note on toggle timers( shared with OAL which does CPU power
     * gating): per convention with OAL default settings are never changed.
     */
    PowerGroupResetControl(hRmDeviceHandle, PowerGroup, NV_TRUE);

    reg = NV_DRF_DEF(APBDEV_PMC, PWRGATE_TOGGLE, START, ENABLE) | Id;
    NV_REGW(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
            APBDEV_PMC_PWRGATE_TOGGLE_0, reg);
    for (;;)
    {
        reg = NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
                      APBDEV_PMC_PWRGATE_STATUS_0);
        if (Status != (reg & Mask))
            break;
    }
    if (Enable)
    {
        PowerGroupClockControl(hRmDeviceHandle, PowerGroup, NV_TRUE);
        NvOsWaitUS(NVRM_RESET_DELAY);

        // PCIE and VDE clamping masks are swapped relatively to
        // partition Ids (bug 602975)
        if (PowerGroup == NV_POWERGROUP_PCIE)
            Mask = 0x1 << s_PowerGroupIds[NV_POWERGROUP_VDE];
        else if (PowerGroup == NV_POWERGROUP_VDE)
            Mask = 0x1 << s_PowerGroupIds[NV_POWERGROUP_PCIE];

        NV_REGW(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
                APBDEV_PMC_REMOVE_CLAMPING_CMD_0, Mask);
        for (;;)
        {
            reg = NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
                          APBDEV_PMC_REMOVE_CLAMPING_CMD_0);
            if (reg == 0)
                break;
        }
        PowerGroupClockControl(hRmDeviceHandle, PowerGroup, NV_FALSE);
        PowerGroupResetControl(hRmDeviceHandle, PowerGroup, NV_FALSE);
    }
}

void
NvRmPrivPowerGroupControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup,
    NvBool Enable)
{
    NVRM_POWER_PRINTF(("%s Power Group %d\n",
                       (Enable ? "Enable" : "Disable"), PowerGroup));

    // Do nothing if dynamic power gating is not supported for this group
    if (PowerGroup >= NV_POWERGROUP_MAX)
        return;             // "virtual" groups are always On
    if (!IsRunTimePowerGateSupported(PowerGroup))
        return;

    // Gate/ungate the group
    PowerGroupPowerControl(hRmDeviceHandle, PowerGroup, Enable);
}

NvRmMilliVolts
NvRmPrivPowerGroupGetVoltage(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup)
{
    NvRmMilliVolts Voltage = NvRmVoltsUnspecified;
    if (PowerGroup >= NV_POWERGROUP_MAX)
        return Voltage;     // "virtual" groups are always On

    // Do not check non-gated power group  - it is On by definition
    if (s_PowerGroupIds[PowerGroup] != NV_POWERGROUP_INVALID)
    {
        NvU32 reg = NV_REGR(
            hRmDeviceHandle, NvRmModuleID_Pmif, 0, APBDEV_PMC_PWRGATE_STATUS_0);
        if ((reg & (0x1 << s_PowerGroupIds[PowerGroup])) == 0x0)
        {
            // Specified power group is gated
            Voltage = NvRmVoltsOff;
        }
    }
    return Voltage;
}

void NvRmPrivPowerGroupSuspend(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;

    // On suspend entry power gate core group that is still On, but must be Off
    for (i = 0; i < NV_POWERGROUP_MAX; i++)
    {
        if (!IsSuspendPowerGateForced(i) ||
            (NvRmPrivPowerGroupGetVoltage(hRmDeviceHandle, i) == NvRmVoltsOff))
        {
            s_UngateOnResume[i] = NV_FALSE;
            continue;
        }

        s_UngateOnResume[i] = NV_TRUE;
        PowerGroupPowerControl(hRmDeviceHandle, i, NV_FALSE);
    }
}

void NvRmPrivPowerGroupResume(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;

    // On resume ungate core group that was forcefully gated on suspend entry
    for (i = 0; i < NV_POWERGROUP_MAX; i++)
    {
        if (s_UngateOnResume[i] == NV_TRUE)
            PowerGroupPowerControl(hRmDeviceHandle, i, NV_TRUE);
    }
}

void NvRmPrivPowerGroupControlInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i, Size;

    // Init chip specific power group map
    if ((hRmDeviceHandle->ChipId.Id == 0x15) ||
        (hRmDeviceHandle->ChipId.Id == 0x16))
    {
        NvRmPrivAp15PowerGroupTableInit(&s_PowerGroupIds, &Size);
    }
    else if (hRmDeviceHandle->ChipId.Id == 0x20)
    {
        NvRmPrivAp20PowerGroupTableInit(&s_PowerGroupIds, &Size);
    }
    else
    {
        NV_ASSERT(!"Unsupported chip ID");
    }
    NV_ASSERT(Size == NV_POWERGROUP_MAX);

    // Power gate supported partitions
    for (i = 0; i < NV_POWERGROUP_MAX; i++)
        NvRmPrivPowerGroupControl(hRmDeviceHandle, i, NV_FALSE);
}

#endif  // !NV_OAL
/*****************************************************************************/
/*****************************************************************************/

#define NV_NO_IOPOWER_CONTROL (1)
#define NV_RAIL_ADDR_INVALID ((NvU32)-1)

typedef struct IoPowerDetectInfoRec
{
    // SoC Power rail GUID
    NvU64 PowerRailId;

    // IO Power rail disable bit mask
    NvU32 DisableRailMask;

    // IO Power Detect cell enable bit mask
    NvU32 EnablePwrDetMask;

    // PMU Rail Address
    NvU32 PmuRailAddress;

} IoPowerDetectInfo;

static IoPowerDetectInfo s_IoPowerDetectMap[] =
{
    {NV_VDD_SYS_ODM_ID,  (0x1 << 0), (0x1 << 0), 0},
    {NV_VDD_NAND_ODM_ID, (0x1 << 1), (0x1 << 1), 0},
    {NV_VDD_UART_ODM_ID, (0x1 << 2), (0x1 << 2), 0},
    {NV_VDD_BB_ODM_ID,   (0x1 << 3), (0x1 << 3), 0},

    {NV_VDD_VI_ODM_ID,   (0x1 << 4), (0x1 << 4), 0},
    {NV_VDD_AUD_ODM_ID,  (0x1 << 5), (0x1 << 5), 0},
    {NV_VDD_LCD_ODM_ID,  (0x1 << 6), (0x1 << 6), 0},
    {NV_VDD_DDR_ODM_ID,  (0x1 << 7), (0x1 << 7), 0},

    {NV_VDD_SDIO_ODM_ID, (0x1 << 8), (0x1 << 8), 0},
    {NV_VDD_MIPI_ODM_ID, (0x1 << 9),      (0x0), 0} // No detect cell
};

static void IoPowerMapRail(
    NvU32 PmuRailAddress,
    NvU32* pIoPwrDetectMask,
    NvU32* pNoIoPwrMask)
{
    NvU32 i;
    *pIoPwrDetectMask = 0;
    *pNoIoPwrMask = 0;

    // Find all power detect cells and controls on this IO rail
    for (i = 0; i < NV_ARRAY_SIZE(s_IoPowerDetectMap); i++)
    {
        if (s_IoPowerDetectMap[i].PmuRailAddress == PmuRailAddress)
        {
            *pIoPwrDetectMask |= s_IoPowerDetectMap[i].EnablePwrDetMask;
            *pNoIoPwrMask |= s_IoPowerDetectMap[i].DisableRailMask;
        }
    }
}

void NvRmPrivIoPowerControlInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i, v;
    NvU32 NoIoPwrMask = 0;
    const NvOdmPeripheralConnectivity* pPmuRail = NULL;

    if (NvRmPrivGetExecPlatform(hRmDeviceHandle) != ExecPlatform_Soc)
    {
        // Invalidate IO Power detect map if not SoC platform
        for (i = 0; i < NV_ARRAY_SIZE(s_IoPowerDetectMap); i++)
            s_IoPowerDetectMap[i].PmuRailAddress = NV_RAIL_ADDR_INVALID;
        return;
    }

    for (i = 0; i < NV_ARRAY_SIZE(s_IoPowerDetectMap); i++)
    {
        // Fill in PMU rail addresses in IO Power detect map
        pPmuRail = NvOdmPeripheralGetGuid(s_IoPowerDetectMap[i].PowerRailId);
        NV_ASSERT(pPmuRail && pPmuRail->NumAddress);
        s_IoPowerDetectMap[i].PmuRailAddress = pPmuRail->AddressList[0].Address;

        // Find all unpowered rails
        v = NvRmPrivPmuRailGetVoltage(
            hRmDeviceHandle, s_IoPowerDetectMap[i].PowerRailId);
        if (v == ODM_VOLTAGE_OFF)
            NoIoPwrMask |= s_IoPowerDetectMap[i].DisableRailMask;
    }

    // Latch already powered IO rails
    NvRmPrivIoPowerDetectLatch(hRmDeviceHandle);

    // Disable IO pads for unpowered rails
    if (NoIoPwrMask)
        NvRmPrivIoPowerControl(hRmDeviceHandle, NoIoPwrMask, NV_FALSE);
}

void NvRmPrivIoPowerDetectStart(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PwrDetMask)
{
// (1) Enable specified power detect cell
    NV_REGW(hRmDeviceHandle,
            NvRmModuleID_Pmif, 0, APBDEV_PMC_PWR_DET_0, PwrDetMask);

// (2-3) Set power detect latches for enabled cells to safe "1" (high) value
    if ((hRmDeviceHandle->ChipId.Id == 0x15) ||
        (hRmDeviceHandle->ChipId.Id == 0x16))
    {
        // On AP15/AP16 set/clear reset bit in PMC scratch0
        NvRmPrivAp15IoPowerDetectReset(hRmDeviceHandle);

        // For AP15 A01 chip the above reset does nothing, therefore
        // need to set latch "pass-thru" before transition
        if ((hRmDeviceHandle->ChipId.Id == 0x15) &&
            (hRmDeviceHandle->ChipId.Major == 0x01) &&
            (hRmDeviceHandle->ChipId.Minor == 0x01))
        {
            NvOsWaitUS(NVRM_PWR_DET_DELAY_US);
            NV_REGW(hRmDeviceHandle,
                    NvRmModuleID_Pmif, 0, APBDEV_PMC_PWR_DET_LATCH_0, 1);
        }
    }
    else
    {
        // On AP20+ reset high values directly
        NvRmPrivAp20IoPowerDetectReset(hRmDeviceHandle);
    }
}
//
//  (4) Power rail OFF -> ON transition and stabilization
//
void NvRmPrivIoPowerDetectLatch(NvRmDeviceHandle hRmDeviceHandle)
{
// (5) Set latch "pass-thru"
// (6) Latch results
// (7) Disable all power detect cells
    NV_REGW(hRmDeviceHandle,
            NvRmModuleID_Pmif, 0, APBDEV_PMC_PWR_DET_LATCH_0, 1);
    NV_REGW(hRmDeviceHandle,
            NvRmModuleID_Pmif, 0, APBDEV_PMC_PWR_DET_LATCH_0, 0);
    NV_REGW(hRmDeviceHandle,
            NvRmModuleID_Pmif, 0, APBDEV_PMC_PWR_DET_0, 0);
}

void NvRmPrivIoPowerControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 NoIoPwrMask,
    NvBool Enable)
{
    NvU32 reg = NV_REGR(
        hRmDeviceHandle, NvRmModuleID_Pmif, 0, APBDEV_PMC_NO_IOPOWER_0);
    reg = Enable ? (reg & (~NoIoPwrMask)) : (reg | NoIoPwrMask);

#if NV_NO_IOPOWER_CONTROL
    NV_REGW(hRmDeviceHandle,
            NvRmModuleID_Pmif, 0, APBDEV_PMC_NO_IOPOWER_0, reg);
#endif
}

void
NvRmPrivSetSocRailPowerState(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PmuRailAddress,
    NvBool Enable,
    NvU32* pIoPwrDetectMask,
    NvU32* pNoIoPwrMask)
{
    IoPowerMapRail(PmuRailAddress, pIoPwrDetectMask, pNoIoPwrMask);
    if ((*pIoPwrDetectMask == 0) && (*pNoIoPwrMask == 0))
        return;     // Exit if not mapped rail

    if (Enable)
    {
        // On/Off transition: activate power detect cells and keep control
        // masks so that the results can be latched and IO pads enabled after
        // the transition is completed
        if (*pIoPwrDetectMask != 0)
            NvRmPrivIoPowerDetectStart(hRmDeviceHandle, *pIoPwrDetectMask);
    }
    else
    {
        // Off/On transition: disable IO pads, and clear control masks,
        // as no action is required after the transition is completed
        if (*pNoIoPwrMask != 0)
            NvRmPrivIoPowerControl(hRmDeviceHandle, *pNoIoPwrMask, NV_FALSE);
        *pIoPwrDetectMask = *pNoIoPwrMask = 0;
    }
}

/*****************************************************************************/

void NvRmPrivCoreVoltageInit(NvRmDeviceHandle hRmDevice)
{
    NvU32 CoreRailAddress, RtcRailAddress, CpuRailAddress;
    const NvOdmPeripheralConnectivity* pPmuRail;
    NvRmMilliVolts CurrentCoreMv = 0;
    NvRmMilliVolts CurrentRtcMv = 0;
    NvRmMilliVolts NominalCoreMv = NvRmPrivGetNominalMV(hRmDevice);

    NV_ASSERT(hRmDevice);

    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
    {
        return;
    }

    pPmuRail = NvOdmPeripheralGetGuid(NV_VDD_CORE_ODM_ID);
    NV_ASSERT(pPmuRail);
    NV_ASSERT(pPmuRail->NumAddress);
    CoreRailAddress = pPmuRail->AddressList[0].Address;

    pPmuRail = NvOdmPeripheralGetGuid(NV_VDD_RTC_ODM_ID);
    NV_ASSERT(pPmuRail);
    NV_ASSERT(pPmuRail->NumAddress);
    RtcRailAddress = pPmuRail->AddressList[0].Address;

    // This function is called during PMU initialization when current (= boot)
    // core voltage is expected to be within one safe step from nominal, and
    // RTC voltage must be within one safe step from the core. Set nominal
    // voltage (bump PMU ref count), if the above conditions are true.
    NvRmPmuGetVoltage(hRmDevice, CoreRailAddress, &CurrentCoreMv);
    NvRmPmuGetVoltage(hRmDevice, RtcRailAddress, &CurrentRtcMv);
    if((CurrentCoreMv > (NominalCoreMv + NVRM_SAFE_VOLTAGE_STEP_MV)) ||
       ((CurrentCoreMv + NVRM_SAFE_VOLTAGE_STEP_MV) < NominalCoreMv))
    {
        NV_ASSERT(!"Unexpected initial core voltage");
        return;
    }
    if((CurrentRtcMv > (CurrentCoreMv + NVRM_SAFE_VOLTAGE_STEP_MV)) ||
       ((CurrentRtcMv + NVRM_SAFE_VOLTAGE_STEP_MV) < CurrentCoreMv))
    {
        NV_ASSERT(!"Unexpected initial RTC voltage");
        return;
    }
    // If core voltage is going up, update it before CPU
    if (CurrentCoreMv <= NominalCoreMv)
    {
        NvRmPmuSetVoltage(hRmDevice, RtcRailAddress, NominalCoreMv, NULL);
        NvRmPmuSetVoltage(hRmDevice, CoreRailAddress, NominalCoreMv, NULL);
    }

    // If the platform has dedicated CPU voltage rail, make sure it is set to
    // nominal level as well (bump PMU ref count along the way).
    if (NvRmPrivIsCpuRailDedicated(hRmDevice))
    {
        NvRmPmuVddRailCapabilities cap;
        NvRmMilliVolts NominalCpuMv = NvRmPrivModuleVscaleGetMV(
            hRmDevice, NvRmModuleID_Cpu,
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz);

        pPmuRail = NvOdmPeripheralGetGuid(NV_VDD_CPU_ODM_ID);
        NV_ASSERT(pPmuRail);
        NV_ASSERT(pPmuRail->NumAddress);
        CpuRailAddress = pPmuRail->AddressList[0].Address;

        // Clip nominal CPU voltage to minimal PMU capabilities, and set it.
        // (note: PMU with CPU voltage range above nominal is temporary
        // accepted exception; for other limit violations: PMU maximum level
        // for CPU is not high enough, or PMU core range does not include
        // nominal core voltage, assert is fired inside NvRmPmuSetVoltage())
        NvRmPmuGetCapabilities(hRmDevice, CpuRailAddress, &cap);
        NominalCpuMv = NV_MAX(NominalCpuMv, cap.MinMilliVolts);
        NvRmPmuSetVoltage(hRmDevice, CpuRailAddress, NominalCpuMv, NULL);
        if (CurrentCoreMv > NominalCoreMv)
            NvOsWaitUS(NVRM_CPU_TO_CORE_DOWN_US); // delay if core to go down
    }

    // If core voltage is going down, update it after CPU voltage
    if (CurrentCoreMv > NominalCoreMv)
    {
        NvRmPmuSetVoltage(hRmDevice, RtcRailAddress, NominalCoreMv, NULL);
        NvRmPmuSetVoltage(hRmDevice, CoreRailAddress, NominalCoreMv, NULL);
    }

    // Always On System I/O, DDR IO and RX DDR (if exist) - set nominal,
    // bump ref count
    NvRmPrivPmuRailControl(hRmDevice, NV_VDD_SYS_ODM_ID, NV_TRUE);
    NvRmPrivPmuRailControl(hRmDevice, NV_VDD_DDR_ODM_ID, NV_TRUE);
    if (NvOdmPeripheralGetGuid(NV_VDD_DDR_RX_ODM_ID))
        NvRmPrivPmuRailControl(hRmDevice, NV_VDD_DDR_RX_ODM_ID, NV_TRUE);
}
