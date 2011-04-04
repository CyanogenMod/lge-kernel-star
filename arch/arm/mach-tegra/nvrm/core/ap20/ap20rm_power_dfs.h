/*
 * Copyright (c) 2009 NVIDIA Corporation.
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
 * @b Description: NvRM DFS parameters. 
 * 
 */

#ifndef INCLUDED_AP20RM_POWER_DFS_H
#define INCLUDED_AP20RM_POWER_DFS_H

#include "nvrm_power_dfs.h"

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

// Min KHz for CPU and AVP with regards to JTAG support - 1MHz * 8  = 8MHz
// TODO: any other limitations on min KHz?
// TODO: adjust boost parameters based on testing

/**
 * Default DFS algorithm parameters for CPU domain
 */
#define NVRM_DFS_PARAM_CPU_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    216000, /* Minimum domain frequency 216 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        32000, /* Fixed frequency boost increase 32 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        10000, /* Fixed frequency boost increase 10 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    2,      /* Relative adjustement of average freqiency 1/2^2 ~ 25% */ \
    1,      /* Number of smaple intervals with NRT to trigger boost = 2 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 *  Default DFS algorithm parameters for AVP domain
 */
#define NVRM_DFS_PARAM_AVP_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        2000,  /* Fixed frequency NRT boost increase 2 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    3,      /* Relative adjustement of average freqiency 1/2^3 ~ 12% */ \
    3,      /* Number of smaple intervals with NRT to trigger boost = 4 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for System clock domain
 */
#define NVRM_DFS_PARAM_SYSTEM_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    5,      /* Relative adjustement of average freqiency 1/2^5 ~ 3% */ \
    2,      /* Number of smaple intervals with NRT to trigger boost = 3 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for AHB clock domain
 */
#define NVRM_DFS_PARAM_AHB_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for APB clock domain
 */
#define NVRM_DFS_PARAM_APB_AP20 \
    NVRM_AP20_APB_MAX_KHZ, /* AP20 APB limit is lower than other buses */ \
    36000,  /* Minimum domain frequency 36 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for Video-pipe clock domain
 */
#define NVRM_DFS_PARAM_VPIPE_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        16000, /* Fixed frequency RT boost increase 16 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    5,      /* Relative adjustement of average freqiency 1/2^5 ~ 3% */ \
    3,      /* Number of smaple intervals with NRT to trigger boost = 4 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for EMC clock domain
 */

// Defines minimum scaling limit for each supported SDRAM type
#define NVRM_AP20_DDR2_MIN_KHZ (50000)
#define NVRM_AP20_LPDDR2_MIN_KHZ (18000)

#define NVRM_DFS_PARAM_EMC_AP20_DDR2 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    NVRM_AP20_DDR2_MIN_KHZ,  /* Minimum domain frequency for DDR2 */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        16000, /* Fixed frequency RT boost increase 16 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    1,      /* Relative adjustement of average freqiency 1/2^1 ~ 50% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */

#define NVRM_DFS_PARAM_EMC_AP20_LPDDR2 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    NVRM_AP20_LPDDR2_MIN_KHZ,  /* Minimum domain frequency for LPDDR2 */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        16000, /* Fixed frequency RT boost increase 16 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */

#define NVRM_DFS_PARAM_EMC_AP20 NVRM_DFS_PARAM_EMC_AP20_LPDDR2

/**
 * Defines CPU frequency threshold for slave CPU1 power management:
 * - CPU1 is turned Off when cpu clock is below ON_MIN for
 *   ON_PENDING time in a row
 * - CPU1 is turned On when cpu clock is above OFF_MAX for
 *   OFF_PENDING time in a row
 * If thresholds are set to 0, the values are derived at run time from the
 * characterization data
 */
#define NVRM_CPU1_ON_MIN_KHZ (0)
#define NVRM_CPU1_OFF_MAX_KHZ (0)

#define NVRM_CPU1_ON_PENDING_MS (1500)
#define NVRM_CPU1_OFF_PENDING_MS (1000)

/**
 * Defines AP20 Thermal policy parameters.
 *
 * Low and high thresholds specify 3 temperature ranges.
 * If temperature is below low threshold:
 *   - no throttling,
 *   - slow polling (if interrupt mode is not supported by ODM)
 * If temperature is above low threshold, but below high threshold:
 *   - limit CPU voltage to THROTTLE_MV (no throttling if set to
 *     NvRmVoltsMaximum)
 *   - fast polling (if interrupt mode is not supported by ODM)
 * If temperature is above high threshold:
 *   - throttle CPU frequency with CPU_DELTA_KHZ/POLL_MS_CRITICAL
 *     gradient until maximum throttling ratio is reached
 *   - critical polling (if interrupt mode is not supported by ODM)
 *
 * ODM should also set a critical threshold to trigger h/w shutdown
 * mechanism.
 */
#define NVRM_DTT_DEGREES_HIGH           (90L)
#define NVRM_DTT_DEGREES_LOW            (60L)
#define NVRM_DTT_DEGREES_HYSTERESIS     (5L)

#define NVRM_DTT_VOLTAGE_THROTTLE_MV    (NvRmVoltsMaximum)
#define NVRM_DTT_CPU_DELTA_KHZ          (100000UL)
#define NVRM_DTT_RATIO_MAX              (2)

#define NVRM_DTT_POLL_MS_CRITICAL       (2000UL)
#define NVRM_DTT_POLL_MS_FAST           (4000UL)
#define NVRM_DTT_POLL_MS_SLOW           (8000UL)

/// Default low corners for core and dedicated CPU voltages
#define NVRM_AP20_LOW_CORE_MV (950)
//#define NVRM_AP20_LOW_CPU_MV (750)
//20100903 taewan.kim@lge.com min voltage fix
#define NVRM_AP20_LOW_CPU_MV (770)


/// Core voltage in suspend
#define NVRM_AP20_SUSPEND_CORE_MV (1000)

/*****************************************************************************/

/**
 * Adjust EMC scaling algorithm parameters based on the SDRAM type selected by
 * current EMC configuration.
 *
 * @param pDfs - A pointer to DFS structure.
 */
void NvRmPrivAp20EmcParametersAdjust(NvRmDfs* pDfs);

/**
 * Initializes activity monitors within the DFS module. Only activity
 * monitors are affected. The rest of module's h/w is preserved.
 * 
 * @param pDfs - A pointer to DFS structure.
 * 
 * @return NvSuccess if initialization completed successfully
 * or one of common error codes on failure.
 */
NvError NvRmPrivAp20EmcMonitorsInit(NvRmDfs* pDfs);

/**
 * Deinitializes activity monitors within the DFS module. Only activity
 * monitors are affected. The rest of module's h/w is preserved.
 * 
 * @param pDfs - A pointer to DFS structure.
 */
void NvRmPrivAp20EmcMonitorsDeinit(NvRmDfs* pDfs);

/**
 * Starts activity monitors in the DFS module for the next sample interval.
 * 
 * @param pDfs - A pointer to DFS structure.
 * @param pDfsKHz - A pointer to current DFS clock frequencies structure.
 * @param IntervalMs Next sampling interval in ms.
 */
void
NvRmPrivAp20EmcMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs);

/**
 * Reads idle count from activity monitors in the DFS module. The monitors are
 * stopped.
 * 
 * @param pDfs - A pointer to DFS structure.
 * @param pDfsKHz - A pointer to current DFS clock frequencies structure.
 * @param pIdleData - A pointer to idle cycles structure to be filled in with
 *  data read from the monitor.
 * 
 */
void
NvRmPrivAp20EmcMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData);

/**
 * Changes core and rtc voltages, keeping them in synch
 * 
 * @param hRm The RM device handle.
 * @param A pointer to DVS structure.
 * @param TargetMv Requested core/rtc voltage in mV.
 * 
 */
void
NvRmPrivAp20DvsChangeCoreVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv);

/**
 * Updates thermal policy according to current temperature.
 * 
 * @param hRmDevice The RM device handle.
 * @param TemperatureC Current core temperature in degrees C.
 * @param pDtt A pointer to dynamic thermal throttling structure.
 */

void
NvRmPrivAp20DttPolicyUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    NvRmDtt* pDt);

//20101121 cs77.ha@lge.com, HW power off in thermal limit [START]
#if defined(CONFIG_MACH_STAR)
void
NvRmPrivStarDttPolicyUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    NvRmDtt* pDt);

#endif
//20101121 cs77.ha@lge.com, HW power off in thermal limit [END]

/**
 * Throttles DFS target clocks.
 * 
 * @param hRmDevice The RM device handle.
 * @param TemperatureC Current core temperature in degrees C.
 * @param pPolicy A pointer to current throttling policy.
 * @param pCurrentKHz A pointer to current DFS clock frequencies structure.
 * @param pDfsKHz A pointer to DFS clock structure with target frequencies
 *  on entry, and throttled frequencies on exit.
 * 
 * @return NV_TRUE if throttling requires additional DVFS scaling steps,
 *  and NV_FALSE otherwise.
 */
NvBool
NvRmPrivAp20DttClockUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    const NvRmTzonePolicy* pDttPolicy,
    const NvRmDfsFrequencies* pCurrentKHz,
    NvRmDfsFrequencies* pDfsKHz);

/**
 * Determines PM request to change CPU(s) power state.
 * 
 * @param hRmDevice The RM device handle.
 * @param pCpuSampler Pointer to the DFS CPU clock sampling records
 * @param pCpuKHz Pointer to the CPU clock frequency target
 * 
 * @return New PM request to change CPU power state
 */
NvRmPmRequest 
NvRmPrivAp20GetPmRequest(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz* pCpuKHz);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif // INCLUDED_AP20RM_POWER_DFS_H
