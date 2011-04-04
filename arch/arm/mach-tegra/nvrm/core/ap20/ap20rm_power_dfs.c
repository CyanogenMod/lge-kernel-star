/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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

#include "ap20rm_power_dfs.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_pmu.h"
#include "ap20/aremc.h"
#include "ap20/arclk_rst.h"
#include "ap20/arapb_misc.h"

/*****************************************************************************/

// Regsiter access macros for EMC module
#define NV_EMC_REGR(pEmcRegs, reg) \
            NV_READ32((((NvU32)(pEmcRegs)) + EMC_##reg##_0))
#define NV_EMC_REGW(pEmcRegs, reg, val) \
            NV_WRITE32((((NvU32)(pEmcRegs)) + EMC_##reg##_0), (val))

// Regsiter access macros for APB MISC module
#define NV_APB_REGR(pApbRegs, reg) \
         NV_READ32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0))
#define NV_APB_REGW(pApbRegs, reg, val) \
         NV_WRITE32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0), (val))

// TODO: Always Disable before check-in
#define NVRM_TEST_PMREQUEST_UP_MODE (0)

// Microsecond timer access
static void* s_pTimerUs = NULL;
extern void* NvRmPrivAp15GetTimerUsVirtAddr(NvRmDeviceHandle hRm);

/*****************************************************************************/
// EMC MODULE INTERFACES
/*****************************************************************************/

void NvRmPrivAp20EmcParametersAdjust(NvRmDfs* pDfs)
{
    NvRmDfsParam EmcParamDddr2 = { NVRM_DFS_PARAM_EMC_AP20_DDR2 };
    NvRmDfsParam EmcParamLpDddr2 = { NVRM_DFS_PARAM_EMC_AP20_LPDDR2 };

    NvU32 RegValue = NV_REGR(pDfs->hRm,
        NvRmPrivModuleID_ExternalMemoryController, 0, EMC_FBIO_CFG5_0);

    // Overwrite default EMC parameters and LP2 policy with SDRAM type specific
    // settings
    switch (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_TYPE, RegValue))
    {
        case EMC_FBIO_CFG5_0_DRAM_TYPE_LPDDR2:
            pDfs->DfsParameters[NvRmDfsClockId_Emc] = EmcParamLpDddr2;
            break;

        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR2:
            pDfs->DfsParameters[NvRmDfsClockId_Emc] = EmcParamDddr2;
            break;

        default:
            NV_ASSERT(!"Not supported DRAM type");
    }
}

NvError NvRmPrivAp20EmcMonitorsInit(NvRmDfs* pDfs)
{
    NvU32 RegValue;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;
    NV_ASSERT(pEmcRegs);

    /*
     * EMC power management monitor belongs to EMC module - just reset it,
     * and do not touch anything else in EMC.
     */ 
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, RST, RegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    /*
    * EMC active clock cycles = EMC monitor reading * 2^M, where M depends
    * on DRAM type and bus width. Power M is stored as EMC readouts scale
    */
    #define COUNT_SHIFT_DDR1_X32 (1)
    RegValue = NV_EMC_REGR(pEmcRegs, FBIO_CFG5);
    switch (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_TYPE, RegValue))
    {
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR1:
        case EMC_FBIO_CFG5_0_DRAM_TYPE_LPDDR2:
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR2:
            pDfs->Modules[NvRmDfsModuleId_Emc].Scale = COUNT_SHIFT_DDR1_X32;
            break;
        default:
            NV_ASSERT(!"Not supported DRAM type");
    }
    if (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_WIDTH, RegValue) ==
        EMC_FBIO_CFG5_0_DRAM_WIDTH_X16)
    {
        pDfs->Modules[NvRmDfsModuleId_Emc].Scale++;
    }
    return NvSuccess;
}

void NvRmPrivAp20EmcMonitorsDeinit(NvRmDfs* pDfs)
{
     // Stop monitor using initialization procedure
    (void)NvRmPrivAp20EmcMonitorsInit(pDfs);
}

void
NvRmPrivAp20EmcMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs)
{
    NvU32 RegValue, SavedRegValue;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;

    // EMC sample period is specified in EMC clock cycles, accuracy 0-16 cycles.
    #define MEAN_EMC_LIMIT_ERROR (8)
    NvU32 cycles = IntervalMs * pDfsKHz->Domains[NvRmDfsClockId_Emc] +
                    MEAN_EMC_LIMIT_ERROR;
    /*
     * Start EMC power monitor for the next sample period: clear EMC counters,
     * set sample interval limit in EMC cycles, enable monitoring. Monitor is
     * counting EMC 1x clock cycles while any memory access is detected. 
     */
    SavedRegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, CLEAR, SavedRegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    RegValue = NV_DRF_NUM(EMC, STAT_PWR_CLOCK_LIMIT, PWR_CLOCK_LIMIT, cycles);
    NV_EMC_REGW(pEmcRegs, STAT_PWR_CLOCK_LIMIT, RegValue);

    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, ENABLE, SavedRegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);
}

void
NvRmPrivAp20EmcMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData)
{
    NvU32 RegValue, TotalClocks;
    NvU32 CountShift = pDfs->Modules[NvRmDfsModuleId_Emc].Scale;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;

    /*
     * Read EMC monitor: disable it (=stop, the readings are preserved), and
     * determine idle count based on total and active clock counts. Monitor
     * readings are multiplied by 2^M factor to determine active count, where
     * power M depends on DRAM type and bus width. Store result in the idle
     * data packet.
     */
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, DISABLE, RegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    RegValue = NV_EMC_REGR(pEmcRegs, STAT_PWR_CLOCKS);
    TotalClocks = NV_DRF_VAL(EMC, STAT_PWR_CLOCKS, PWR_CLOCKS, RegValue);
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_PWR_COUNT);
    RegValue = NV_DRF_VAL(EMC, STAT_PWR_COUNT, PWR_COUNT, RegValue) << CountShift;

    pIdleData->Readings[NvRmDfsClockId_Emc] = 
        (TotalClocks > RegValue) ? (TotalClocks - RegValue) : 0;
}

/*****************************************************************************/

typedef enum
{
    NvRmDttAp20PolicyRange_Unknown = 0,

    // No throttling
    NvRmDttAp20PolicyRange_FreeRunning,

    // Keep CPU frequency below low voltage threshold
    NvRmDttAp20PolicyRange_LimitVoltage,

    // Decrease CPU frequency in steps over time
    NvRmDttAp20PolicyRange_ThrottleDown,

    NvRmDttAp20PolicyRange_Num,
    NvRmDttAp20PolicyRange_Force32 = 0x7FFFFFFFUL
} NvRmDttAp20PolicyRange;

static NvRmFreqKHz s_CpuThrottleMaxKHz = 0;
static NvRmFreqKHz s_CpuThrottleMinKHz = 0;
static NvRmFreqKHz s_CpuThrottleKHz = 0;

void
NvRmPrivAp20DttPolicyUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    NvRmDtt* pDtt)
{
    NvRmDttAp20PolicyRange Range;

    // CPU throttling limits are set at 50% of CPU frequency range (no
    // throttling below this value), and at CPU frequency boundary that
    // matches specified voltage throttling limit.
    if ((!s_CpuThrottleMaxKHz) || (!s_CpuThrottleMinKHz))
    {
        NvU32 steps;
        const NvRmFreqKHz* p = NvRmPrivModuleVscaleGetMaxKHzList(
            hRmDevice, NvRmModuleID_Cpu, &steps);
        NV_ASSERT(p && steps);
        for (; steps != 0 ; steps--)
        {
            if (NVRM_DTT_VOLTAGE_THROTTLE_MV >= NvRmPrivModuleVscaleGetMV(
                hRmDevice, NvRmModuleID_Cpu, p[steps-1]))
                break;
        }
        NV_ASSERT(steps);
        s_CpuThrottleMaxKHz = NV_MIN(
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz, p[steps-1]);
        s_CpuThrottleMinKHz =
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz /
            NVRM_DTT_RATIO_MAX;
        NV_ASSERT(s_CpuThrottleMaxKHz > s_CpuThrottleMinKHz); 
        NV_ASSERT(s_CpuThrottleMinKHz > NVRM_DTT_CPU_DELTA_KHZ); 

        s_CpuThrottleKHz = s_CpuThrottleMaxKHz;

        NV_ASSERT(pDtt->TcoreCaps.Tmin <
                  (NVRM_DTT_DEGREES_LOW - NVRM_DTT_DEGREES_HYSTERESIS));
        NV_ASSERT(pDtt->TcoreCaps.Tmax > NVRM_DTT_DEGREES_HIGH);
    }

    // Advanced policy range state machine (one step at a time)
    Range = (NvRmDttAp20PolicyRange)pDtt->TcorePolicy.PolicyRange;
    switch (Range)
    {
        case NvRmDttAp20PolicyRange_Unknown:
            Range = NvRmDttAp20PolicyRange_FreeRunning;
            // fall through
        case NvRmDttAp20PolicyRange_FreeRunning:
            if (TemperatureC >= NVRM_DTT_DEGREES_LOW)
                Range = NvRmDttAp20PolicyRange_LimitVoltage;
            break;

        case NvRmDttAp20PolicyRange_LimitVoltage:
            if (TemperatureC <=
                (NVRM_DTT_DEGREES_LOW - NVRM_DTT_DEGREES_HYSTERESIS))
                Range = NvRmDttAp20PolicyRange_FreeRunning;
            else if (TemperatureC >= NVRM_DTT_DEGREES_HIGH)
                Range = NvRmDttAp20PolicyRange_ThrottleDown;
            break;

        case NvRmDttAp20PolicyRange_ThrottleDown:
            if (TemperatureC <=
                (NVRM_DTT_DEGREES_HIGH - NVRM_DTT_DEGREES_HYSTERESIS))
                Range = NvRmDttAp20PolicyRange_LimitVoltage;
            break;

        default:
            break;
    }

    /*
     * Fill in new policy. Temperature limits are set around current
     * temperature for the next out-of-limit interrupt (possible exception
     * - temperature "jump" over two ranges would result in two interrupts
     * in a row before limits cover the temperature). Polling time is set
     * always in ThrottleDown range, and only for poll mode in other ranges.
     */
    pDtt->CoreTemperatureC = TemperatureC;
    switch (Range)
    {
        case NvRmDttAp20PolicyRange_FreeRunning:
            pDtt->TcorePolicy.LowLimit = pDtt->TcoreLowLimitCaps.MinValue;
            pDtt->TcorePolicy.HighLimit = NVRM_DTT_DEGREES_LOW;
            pDtt->TcorePolicy.UpdateIntervalUs = pDtt->UseIntr ?
                NV_WAIT_INFINITE : (NVRM_DTT_POLL_MS_SLOW * 1000);
            break;

        case NvRmDttAp20PolicyRange_LimitVoltage:
            pDtt->TcorePolicy.LowLimit =
                NVRM_DTT_DEGREES_LOW - NVRM_DTT_DEGREES_HYSTERESIS;
            pDtt->TcorePolicy.HighLimit = NVRM_DTT_DEGREES_HIGH;
            pDtt->TcorePolicy.UpdateIntervalUs = pDtt->UseIntr ?
                NV_WAIT_INFINITE : (NVRM_DTT_POLL_MS_FAST * 1000);
            break;

        case NvRmDttAp20PolicyRange_ThrottleDown:
            pDtt->TcorePolicy.LowLimit =
                NVRM_DTT_DEGREES_HIGH - NVRM_DTT_DEGREES_HYSTERESIS;
            pDtt->TcorePolicy.HighLimit = pDtt->TcoreHighLimitCaps.MaxValue;
            pDtt->TcorePolicy.UpdateIntervalUs = NVRM_DTT_POLL_MS_CRITICAL * 1000;
            break;

        default:
            NV_ASSERT(!"Invalid DTT policy range");
            NvOsDebugPrintf("DTT: Invalid Range = %d\n", Range);
            pDtt->TcorePolicy.HighLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
            pDtt->TcorePolicy.LowLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
            pDtt->TcorePolicy.PolicyRange = NvRmDttAp20PolicyRange_Unknown;
            return;
    }
    pDtt->TcorePolicy.PolicyRange = (NvU32)Range;
}

//20101121 cs77.ha@lge.com, HW power off in thermal limit [START]
#if defined(CONFIG_MACH_STAR)
static NvU32    ThermalLimitPwrOffEnalble = NV_FALSE;
extern NvRmDeviceHandle s_hRmGlobal;

void
NvRmPrivStarDttPolicyUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    NvRmDtt* pDtt)
{
    NvRmDttAp20PolicyRange Range;

    Range = (NvRmDttAp20PolicyRange)pDtt->TcorePolicy.PolicyRange;
    switch (Range)
    {
        case NvRmDttAp20PolicyRange_ThrottleDown:
            if(!ThermalLimitPwrOffEnalble){
                NvOsDebugPrintf(">>>>>> DTT: NvRmDttAp20PolicyRange_ThrottleDown!!!!!! <<<<<< \n");
                ThermalLimitPwrOffEnalble = NV_TRUE;
                NvOsDebugPrintf(">>>>>> DTT: HWPowerOffConfig ON!!!!!! <<<<<< \n");
                NvRmPmuSetHwPowerOffConfig(s_hRmGlobal, NV_TRUE);
            }
            break;
            
        case NvRmDttAp20PolicyRange_FreeRunning:
        case NvRmDttAp20PolicyRange_LimitVoltage:
        default:
            if(ThermalLimitPwrOffEnalble){
                if(Range==NvRmDttAp20PolicyRange_FreeRunning)
                    NvOsDebugPrintf(">>>>>> DTT: NvRmDttAp20PolicyRange_FreeRunning \n" );
                else if(Range==NvRmDttAp20PolicyRange_LimitVoltage)
                    NvOsDebugPrintf(">>>>>> DTT: NvRmDttAp20PolicyRange_LimitVoltage \n" );
                else
                    NvOsDebugPrintf(">>>>>> DTT: NvRmDttAp20PolicyRange_unknow \n" );
                ThermalLimitPwrOffEnalble = NV_FALSE;
                NvRmPmuSetHwPowerOffConfig(s_hRmGlobal, NV_FALSE);
                NvOsDebugPrintf(">>>>>> DTT: HWPowerOffConfig OFF!!!!!! <<<<<< \n");
            }
            break;
    }
}
#endif
//20101121 cs77.ha@lge.com, HW power off in thermal limit [END]


NvBool
NvRmPrivAp20DttClockUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    const NvRmTzonePolicy* pDttPolicy,
    const NvRmDfsFrequencies* pCurrentKHz,
    NvRmDfsFrequencies* pDfsKHz)
{
    switch ((NvRmDttAp20PolicyRange)pDttPolicy->PolicyRange)
    {
        case NvRmDttAp20PolicyRange_LimitVoltage:
            s_CpuThrottleKHz = s_CpuThrottleMaxKHz;
            break;

        case NvRmDttAp20PolicyRange_ThrottleDown:
            if (pDttPolicy->UpdateFlag)
                s_CpuThrottleKHz -= NVRM_DTT_CPU_DELTA_KHZ;
            s_CpuThrottleKHz = NV_MAX(s_CpuThrottleKHz, s_CpuThrottleMinKHz); 
            break;

        // No throttling by default (just reset throttling limit to max)
        default:
            s_CpuThrottleKHz = s_CpuThrottleMaxKHz;
            return NV_FALSE;
    }
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] =
        NV_MIN(pDfsKHz->Domains[NvRmDfsClockId_Cpu], s_CpuThrottleKHz);

    // Throttling step is completed - no need to force extra DVFS update
    return NV_FALSE;
}

/*****************************************************************************/

NvRmPmRequest 
NvRmPrivAp20GetPmRequest(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz* pCpuKHz)
{
    // Assume initial slave CPU1 On request
    static NvRmPmRequest s_LastPmRequest = (NvRmPmRequest_CpuOnFlag | 0x1);
    static NvRmFreqKHz s_Cpu1OnMinKHz = 0, s_Cpu1OffMaxKHz = 0;
    static NvU32 s_Cpu1OnPendingCnt = 0, s_Cpu1OffPendingCnt = 0;

    NvU32 t;
    NvRmPmRequest PmRequest = NvRmPmRequest_None;
    NvBool Cpu1Off =
        (0 != NV_DRF_VAL(CLK_RST_CONTROLLER, RST_CPU_CMPLX_SET, SET_CPURESET1,
                         NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                 CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0)));
    NvRmFreqKHz CpuLoadGaugeKHz = *pCpuKHz;

    // Slave CPU1 power management policy thresholds:
    // - use fixed values if they are defined explicitly, otherwise
    // - set CPU1 OffMax threshold at 2/3 of cpu frequency range,
    //   and half of that frequency as CPU1 OnMin threshold
    if ((s_Cpu1OffMaxKHz == 0) && (s_Cpu1OnMinKHz == 0))
    {
        NvRmFreqKHz MaxKHz =
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;

        s_Cpu1OnMinKHz = NVRM_CPU1_ON_MIN_KHZ ?
                         NVRM_CPU1_ON_MIN_KHZ : (MaxKHz / 3);
        s_Cpu1OffMaxKHz = NVRM_CPU1_OFF_MAX_KHZ ?
                          NVRM_CPU1_OFF_MAX_KHZ : (2 * MaxKHz / 3);
        NV_ASSERT(s_Cpu1OnMinKHz < s_Cpu1OffMaxKHz);
    }

    // Timestamp
    if (s_pTimerUs == NULL)
        s_pTimerUs = NvRmPrivAp15GetTimerUsVirtAddr(hRmDevice);
    t = NV_READ32(s_pTimerUs);

    /*
     * Request OS kernel to turn CPU1 Off if all of the following is true:
     * (a) CPU frequency is below OnMin threshold, 
     * (b) CPU1 is actually On
     *
     * Request OS kernel to turn CPU1 On if all of the following is true:
     * (a) CPU frequency is above OffMax threshold 
     * (b) CPU1 is actually Off
     */
    if (CpuLoadGaugeKHz < s_Cpu1OnMinKHz)
    {
        s_Cpu1OnPendingCnt = 0;
        if ((s_Cpu1OffPendingCnt & 0x1) == 0)
        {
            s_Cpu1OffPendingCnt = t | 0x1;  // Use LSb as a delay start flag
            return PmRequest;
        }
        if ((t - s_Cpu1OffPendingCnt) < (NVRM_CPU1_OFF_PENDING_MS * 1000))
            return PmRequest;

        if (!Cpu1Off)
        {
            s_LastPmRequest = PmRequest = (NvRmPmRequest_CpuOffFlag | 0x1);
            s_Cpu1OffPendingCnt = 0;   // re-start delay after request
        }
#if NVRM_TEST_PMREQUEST_UP_MODE
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0_SET_CPURESET1_FIELD);
#endif
    }
    else if (CpuLoadGaugeKHz > s_Cpu1OffMaxKHz)
    {
        s_Cpu1OffPendingCnt = 0;
        if ((s_Cpu1OnPendingCnt & 0x1) == 0)
        {
            s_Cpu1OnPendingCnt = t | 0x1;  // Use LSb as a delay start flag
            return PmRequest;
        }
        if ((t - s_Cpu1OnPendingCnt) < (NVRM_CPU1_ON_PENDING_MS * 1000))
            return PmRequest;

        if (Cpu1Off)
        {
            s_LastPmRequest = PmRequest = (NvRmPmRequest_CpuOnFlag | 0x1);
            *pCpuKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
            s_Cpu1OnPendingCnt = 0;  // re-start delay after request
        }
#if NVRM_TEST_PMREQUEST_UP_MODE
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0_CLR_CPURESET1_FIELD);
#endif
    }
    else
    {   // Re-start both delays inside hysteresis loop
        s_Cpu1OnPendingCnt = 0;
        s_Cpu1OffPendingCnt = 0;
    }
    return PmRequest;
}

/*****************************************************************************/
