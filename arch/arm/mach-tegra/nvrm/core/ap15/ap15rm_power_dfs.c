/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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
 *           Dynamic Frequency Scaling manager </b>
 *
 * @b Description: Implements NvRM Dynamic Frequency Scaling (DFS)
 *                  manager for SOC-wide clock domains.
 * 
 */

#include "nvrm_power_dfs.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_pmu.h"
#include "ap15rm_power_dfs.h"
#include "ap15/arstat_mon.h"
#include "ap15/arvde_mon.h"
#include "ap15/aremc.h"
#include "ap15/arclk_rst.h"
#include "ap15/arapb_misc.h"
#include "ap15/artimerus.h"

/*****************************************************************************/

// Regsiter access macros for System Statistic module
#define NV_SYSTAT_REGR(pSystatRegs, reg) \
            NV_READ32((((NvU32)(pSystatRegs)) + STAT_MON_##reg##_0))
#define NV_SYSTAT_REGW(pSystatRegs, reg, val) \
            NV_WRITE32((((NvU32)(pSystatRegs)) + STAT_MON_##reg##_0), (val))

// Regsiter access macros for VDE module
#define NV_VDE_REGR(pVdeRegs, reg) \
            NV_READ32((((NvU32)(pVdeRegs)) + ARVDE_PPB_##reg##_0))
#define NV_VDE_REGW(pVdeRegs, reg, val) \
            NV_WRITE32((((NvU32)(pVdeRegs)) + ARVDE_PPB_##reg##_0), (val))

// Regsiter access macros for EMC module
#define NV_EMC_REGR(pEmcRegs, reg) \
            NV_READ32((((NvU32)(pEmcRegs)) + EMC_##reg##_0))
#define NV_EMC_REGW(pEmcRegs, reg, val) \
            NV_WRITE32((((NvU32)(pEmcRegs)) + EMC_##reg##_0), (val))

// Regsiter access macros for CAR module
#define NV_CAR_REGR(pCarRegs, reg) \
         NV_READ32((((NvU32)(pCarRegs)) + CLK_RST_CONTROLLER_##reg##_0))
#define NV_CAR_REGW(pCarRegs, reg, val) \
         NV_WRITE32((((NvU32)(pCarRegs)) + CLK_RST_CONTROLLER_##reg##_0), (val))

// Regsiter access macros for APB MISC module
#define NV_APB_REGR(pApbRegs, reg) \
         NV_READ32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0))
#define NV_APB_REGW(pApbRegs, reg, val) \
         NV_WRITE32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0), (val))

/*****************************************************************************/
// SYSTEM STATISTIC MODULE INTERFACES
/*****************************************************************************/

NvError NvRmPrivAp15SystatMonitorsInit(NvRmDfs* pDfs)
{
    NvError error;
    NvU32 RegValue;
    void* pSystatRegs = pDfs->Modules[NvRmDfsModuleId_Systat].pBaseReg;
    NV_ASSERT(pSystatRegs);

    /*
     * System Statistic Monitor module belongs to DFS, therefore it is full
     * initialization: Enable Clock => Reset => clear all control registers
     * including interrupt status flags (cleared by writing "1"). Note that
     * all monitors - used, or not used by DFS - are initialized. (The VPIPE
     * monitor in this module does not provide neccessary data for DFS; the
     * VDE idle monitor is employed for video-pipe domain control)
     */
    error = NvRmPowerModuleClockControl(
        pDfs->hRm, NvRmModuleID_SysStatMonitor, pDfs->PowerClientId, NV_TRUE);
    if (error != NvSuccess)
    {
        return error;
    }
    NvRmModuleReset(pDfs->hRm, NvRmModuleID_SysStatMonitor);

    RegValue = NV_DRF_NUM(STAT_MON, CPU_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, CPU_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, COP_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, COP_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, CACHE2_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, CACHE2_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, AHB_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, AHB_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, APB_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, APB_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, VPIPE_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, VPIPE_MON_CTRL, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, SMP_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, SMP_MON_CTRL, RegValue);

    return NvSuccess;
}

void NvRmPrivAp15SystatMonitorsDeinit(NvRmDfs* pDfs)
{
    // First initialize monitor, and then just turn off the clock (twice to
    // balance the clock control)
    (void)NvRmPrivAp15SystatMonitorsInit(pDfs);
    (void)NvRmPowerModuleClockControl(
        pDfs->hRm, NvRmModuleID_SysStatMonitor, pDfs->PowerClientId, NV_FALSE);
    (void)NvRmPowerModuleClockControl(
        pDfs->hRm, NvRmModuleID_SysStatMonitor, pDfs->PowerClientId, NV_FALSE);

}

void
NvRmPrivAp15SystatMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs)
{
    NvU32 RegValue;
    NvU32 msec = IntervalMs - 1; // systat monitors use (n+1) ms counters
    void* pSystatRegs = pDfs->Modules[NvRmDfsModuleId_Systat].pBaseReg;

    /*
     * Start AVP (COP) monitor for the next sample period. Interrupt is
     * cleared (by writing "1") and left disabled. Monitor is counting
     * System clock cycles while AVP is halted by flow controller. Note
     * that AVP monitor is counting System (not AVP!) clock cycles
     */
    RegValue = NV_DRF_DEF(STAT_MON, COP_MON_CTRL, ENB, ENABLE) |
               NV_DRF_NUM(STAT_MON, COP_MON_CTRL, INT, 1) |
               NV_DRF_NUM(STAT_MON, COP_MON_CTRL, SAMPLE_PERIOD, msec);
    NV_SYSTAT_REGW(pSystatRegs, COP_MON_CTRL, RegValue);

    /*
     * Start AHB monitor for the next sample period. Interrupt is cleared
     * (by writing "1") and left disabled. Monitor is counting AHB clock
     * cycles while there is no data transfer on AHB initiated by any master
     */
    RegValue = NV_DRF_DEF(STAT_MON, AHB_MON_CTRL, ENB, ENABLE) |
               NV_DRF_NUM(STAT_MON, AHB_MON_CTRL, INT, 1) |
               NV_DRF_DEF(STAT_MON, AHB_MON_CTRL, MST_NUMBER, DEFAULT_MASK) |
               NV_DRF_NUM(STAT_MON, AHB_MON_CTRL, SAMPLE_PERIOD, msec);
    NV_SYSTAT_REGW(pSystatRegs, AHB_MON_CTRL, RegValue);

    /*
     * Start APB monitor for the next sample period. Interrupt is cleared
     * (by writing "1") and left disabled. Monitor is counting APB clock
     * cycles while there is no data transfer on APB targeted to any slave 
     */
    RegValue = NV_DRF_DEF(STAT_MON, APB_MON_CTRL, ENB, ENABLE) |
               NV_DRF_NUM(STAT_MON, APB_MON_CTRL, INT, 1) |
               NV_DRF_DEF(STAT_MON, APB_MON_CTRL, SLV_NUMBER, DEFAULT_MASK) |
               NV_DRF_NUM(STAT_MON, APB_MON_CTRL, SAMPLE_PERIOD, msec);
    NV_SYSTAT_REGW(pSystatRegs, APB_MON_CTRL, RegValue);

    /*
     * Start CPU monitor for the next sample period. Interrupt is cleared
     * (by writing "1") and enabled, since CPU monitor is used to generate
     * DFS interrupt. Monitor is counting System clock cycles while CPU is
     * halted by flow controller. Note: CPU monitor is counting System (not
     * CPU!) clock cycles
     */
    RegValue = NV_DRF_DEF(STAT_MON, CPU_MON_CTRL, ENB, ENABLE) |
               NV_DRF_DEF(STAT_MON, CPU_MON_CTRL, INT_EN, ENABLE) |
               NV_DRF_NUM(STAT_MON, CPU_MON_CTRL, INT, 1) |
               NV_DRF_NUM(STAT_MON, CPU_MON_CTRL, SAMPLE_PERIOD, msec);
    NV_SYSTAT_REGW(pSystatRegs, CPU_MON_CTRL, RegValue);

    // Initialize LP2 time storage (WAR for bug 429585)
    NvRmPrivSetLp2TimeUS(pDfs->hRm, 0);
}

void
NvRmPrivAp15SystatMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData)
{
    NvU32 RegValue;
    NvU64 temp;
    void* pSystatRegs = pDfs->Modules[NvRmDfsModuleId_Systat].pBaseReg;
    NvBool NoLp2Offset = pDfs->Modules[NvRmDfsModuleId_Systat].Offset !=
        NVRM_CPU_IDLE_LP2_OFFSET;

    /*
     * Read AVP (COP) monitor: disable it (=stop, the readings are preserved)
     * and clear interrupt status bit (by writing "1"). Then, read AVP idle
     * count. Since AVP monitor is counting System (not AVP!) clock cycles,
     * the monitor reading is converted to AVP clocks before storing it in
     * idle data packet.
     */
    RegValue = NV_DRF_NUM(STAT_MON, COP_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, COP_MON_CTRL, RegValue);

    RegValue = NV_SYSTAT_REGR(pSystatRegs, COP_MON_STATUS);
    RegValue = NV_DRF_VAL(STAT_MON, COP_MON_STATUS, COUNT, RegValue);

    temp = ((NvU64)RegValue * pDfsKHz->Domains[NvRmDfsClockId_Avp]);
    temp = NvDiv64(temp, pDfsKHz->Domains[NvRmDfsClockId_System]);

    pIdleData->Readings[NvRmDfsClockId_Avp] = (NvU32)temp;

    /*
     * Read AHB monitor: disable it (=stop, the readings are preserved) and
     * clear interrupt status bit (by writing "1"). Then, read AHB idle count
     * (in AHB clock cycles) and store it in idle data packet.
     */
    RegValue = NV_DRF_NUM(STAT_MON, AHB_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, AHB_MON_CTRL, RegValue);

    RegValue = NV_SYSTAT_REGR(pSystatRegs, AHB_MON_STATUS);
    pIdleData->Readings[NvRmDfsClockId_Ahb] = 
        NV_DRF_VAL(STAT_MON, AHB_MON_STATUS, COUNT, RegValue);

    /*
     * Read APB monitor: disable it (=stop, the readings are preserved) and
     * clear interrupt status bit (by writing "1"). Then, read APB idle count
     * (in APB clock cycles) and store it in idle data packet.
     */
    RegValue = NV_DRF_NUM(STAT_MON, APB_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, APB_MON_CTRL, RegValue);

    RegValue = NV_SYSTAT_REGR(pSystatRegs, APB_MON_STATUS);
    pIdleData->Readings[NvRmDfsClockId_Apb] = 
        NV_DRF_VAL(STAT_MON, APB_MON_STATUS, COUNT, RegValue);

    /*
     * Read CPU monitor: read current sampling period and store it in idle
     * data packet. Disable monitor (=stop, the readings are preserved) and
     * clear interrupt status bit (by writing "1"). Read CPU idle count.
     * Since CPU monitor is counting System (not CPU!) cycles, the monitor
     * readings are converted to CPU clocks before storing in idle data packet
     */
    RegValue = NV_SYSTAT_REGR(pSystatRegs, CPU_MON_CTRL);
    pIdleData->CurrentIntervalMs = 1 +  // systat monitors use (n+1) ms counters
        NV_DRF_VAL(STAT_MON, CPU_MON_CTRL, SAMPLE_PERIOD, RegValue);

    RegValue = NV_DRF_NUM(STAT_MON, CPU_MON_CTRL, INT, 1);
    NV_SYSTAT_REGW(pSystatRegs, CPU_MON_CTRL, RegValue);

    // Add LP2 time to idle measurements (WAR for bug 429585)
    // For logging only - use 2^10 ~ 1000, and round up
    RegValue = NvRmPrivGetLp2TimeUS(pDfs->hRm);
    pIdleData->Lp2TimeMs = (RegValue + (0x1 << 10) - 1) >> 10;
    if ((RegValue == 0) || NoLp2Offset)
    {
        pIdleData->Readings[NvRmDfsClockId_Cpu] = 0;
    }
    else if (RegValue < NVRM_DFS_MAX_SAMPLE_MS * 1000)
    {   //  (US * KHz) / 1000 ~ (US * 131 * KHz) / (128 * 1024)
        pIdleData->Readings[NvRmDfsClockId_Cpu] =
            (NvU32)(((NvU64)(RegValue * 131) *
                     pDfsKHz->Domains[NvRmDfsClockId_Cpu]) >> 17);
    }
    else
    {
        pIdleData->Readings[NvRmDfsClockId_Cpu] = 0xFFFFFFFFUL;
        return; // the entire sample is idle, anyway
    }
    RegValue = NV_SYSTAT_REGR(pSystatRegs, CPU_MON_STATUS);
    RegValue = NV_DRF_VAL(STAT_MON, CPU_MON_STATUS, COUNT, RegValue);

    temp = ((NvU64)RegValue * pDfsKHz->Domains[NvRmDfsClockId_Cpu]);
    temp = NvDiv64(temp, pDfsKHz->Domains[NvRmDfsClockId_System]);

    pIdleData->Readings[NvRmDfsClockId_Cpu] += (NvU32)temp;
}

/*****************************************************************************/
// VDE MODULE INTERFACES
/*****************************************************************************/

NvError NvRmPrivAp15VdeMonitorsInit(NvRmDfs* pDfs)
{
    NvU32 RegValue;
    void* pVdeRegs = pDfs->Modules[NvRmDfsModuleId_Vde].pBaseReg;
    NV_ASSERT(pVdeRegs);

    /*
     * Video pipe monitor belongs to VDE module - just clear monitor control
     * register including interrupt status bit, and do not touch anything
     * else in VDE 
     */ 
    RegValue = NV_DRF_NUM(ARVDE_PPB, IDLE_MON, INT_STATUS, 1);
    NV_VDE_REGW(pVdeRegs, IDLE_MON, RegValue);

    return NvSuccess;
}

void NvRmPrivAp15VdeMonitorsDeinit(NvRmDfs* pDfs)
{
    // Stop monitor using initialization procedure
    (void)NvRmPrivAp15VdeMonitorsInit(pDfs);
}

void
NvRmPrivAp15VdeMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs)
{
    NvU32 RegValue;
    NvU32 cycles = IntervalMs * pDfsKHz->Domains[NvRmDfsClockId_Vpipe];
    void* pVdeRegs = pDfs->Modules[NvRmDfsModuleId_Vde].pBaseReg;

    /*
     * Start VDE vpipe monitor for the next sample period. Interrupt status bit
     * is cleared by writing "1" (it is not connected to interrupt controller,
     * just "count end" status bit). Monitor is counting v-clock cycles while
     * all VDE submodules are idle. The sample period is specified in v-clock
     * cycles rather than in time units. 
     */
    RegValue = NV_DRF_NUM(ARVDE_PPB, IDLE_MON, ENB, 1) |
               NV_DRF_NUM(ARVDE_PPB, IDLE_MON, INT_STATUS, 1) |
               NV_DRF_NUM(ARVDE_PPB, IDLE_MON, SAMPLE_PERIOD, cycles);
    NV_VDE_REGW(pVdeRegs, IDLE_MON, RegValue);
}

void
NvRmPrivAp15VdeMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData)
{
    // CAR module virtual base address
    static void* s_pCarBaseReg = NULL;
    NvU32 RegValue;
    void* pVdeRegs = pDfs->Modules[NvRmDfsModuleId_Vde].pBaseReg;

    if (s_pCarBaseReg == NULL)
    {
        NvRmModuleTable *tbl = NvRmPrivGetModuleTable(pDfs->hRm);
        s_pCarBaseReg = (tbl->ModInst +
            tbl->Modules[NvRmPrivModuleID_ClockAndReset].Index)->VirtAddr;
    }
    RegValue = NV_CAR_REGR(s_pCarBaseReg, CLK_OUT_ENB_H);

    // If VDE clock is disabled set idle count to maximum
    if (!(RegValue & CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_VDE_FIELD))
    {
        pIdleData->Readings[NvRmDfsClockId_Vpipe] = (NvU32)-1;
        return;
    }

    /*
     * Read VDE vpipe monitor: disable it (=stop, the readings are preserved) and
     * clear count done status bit (by writing "1"). Then, read VDE idle count (in
     * v-clock cycles) and store it in idle data packet.
     */
    RegValue = NV_DRF_NUM(ARVDE_PPB, IDLE_MON, INT_STATUS, 1);
    NV_VDE_REGW(pVdeRegs, IDLE_MON, RegValue);

    RegValue = NV_VDE_REGR(pVdeRegs, IDLE_STATUS);
    pIdleData->Readings[NvRmDfsClockId_Vpipe] = 
        NV_DRF_VAL(ARVDE_PPB, IDLE_STATUS, COUNT, RegValue);
}

/*****************************************************************************/
// EMC MODULE INTERFACES
/*****************************************************************************/

NvError NvRmPrivAp15EmcMonitorsInit(NvRmDfs* pDfs)
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
    #define COUNT_SHIFT_SDRAM_X32 (2)
    #define COUNT_SHIFT_DDR1_X32 (1)
    RegValue = NV_EMC_REGR(pEmcRegs, FBIO_CFG5);
    switch (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_TYPE, RegValue))
    {
        case EMC_FBIO_CFG5_0_DRAM_TYPE_SDR:
            pDfs->Modules[NvRmDfsModuleId_Emc].Scale = COUNT_SHIFT_SDRAM_X32;
            break;
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR1:
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

void NvRmPrivAp15EmcMonitorsDeinit(NvRmDfs* pDfs)
{
     // Stop monitor using initialization procedure
    (void)NvRmPrivAp15EmcMonitorsInit(pDfs);
}

void
NvRmPrivAp15EmcMonitorsStart(
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
NvRmPrivAp15EmcMonitorsRead(
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

void
NvRmPrivAp15SetSvopControls(
    NvRmDeviceHandle hRm,
    NvU32 SvopSetting)
{
#define SVOP_MASK \
    (NV_DRF_NUM(APB_MISC, GP_ASDBGREG, CFG2TMC_RAM_SVOP_DP, 0xFFFFFFFFUL) |  \
     NV_DRF_NUM(APB_MISC, GP_ASDBGREG, CFG2TMC_RAM_SVOP_PDP, 0xFFFFFFFFUL) | \
     NV_DRF_NUM(APB_MISC, GP_ASDBGREG, CFG2TMC_RAM_SVOP_REG, 0xFFFFFFFFUL) | \
     NV_DRF_NUM(APB_MISC, GP_ASDBGREG, CFG2TMC_RAM_SVOP_SP, 0xFFFFFFFFUL))

    NvU32 reg;
    static void* s_pApbBaseReg = NULL;  // APB MISC module virtual base address

    if (s_pApbBaseReg == NULL)
    {
        NvRmModuleTable *tbl = NvRmPrivGetModuleTable(hRm);
        s_pApbBaseReg = (tbl->ModInst +
            tbl->Modules[NvRmModuleID_Misc].Index)->VirtAddr;
    }
    NV_ASSERT((SvopSetting & (~SVOP_MASK)) == 0);
    reg = NV_APB_REGR(s_pApbBaseReg, GP_ASDBGREG);  // RAM timing control
    reg = (reg & (~SVOP_MASK)) | SvopSetting;
    NV_APB_REGW(s_pApbBaseReg, GP_ASDBGREG, reg); 
}

/*****************************************************************************/

void* NvRmPrivAp15GetTimerUsVirtAddr(NvRmDeviceHandle hRm)
{
    NvRmModuleTable *tbl = NvRmPrivGetModuleTable(hRm);
    void* va = (void*)((NvUPtr)((tbl->ModInst + 
        tbl->Modules[NvRmModuleID_TimerUs].Index)->VirtAddr) +
        TIMERUS_CNTR_1US_0);
    return va;
}

/*****************************************************************************/
