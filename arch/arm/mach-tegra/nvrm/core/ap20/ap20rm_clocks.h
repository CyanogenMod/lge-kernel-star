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


#ifndef INCLUDED_AP20RM_CLOCKS_H
#define INCLUDED_AP20RM_CLOCKS_H

#include "nvrm_clocks.h"
#include "nvodm_query_memc.h"

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

extern const NvRmModuleClockInfo g_Ap20ModuleClockTable[];
extern const NvU32 g_Ap20ModuleClockTableSize;

// Minimum PLLX VCO frequency for reliable operation of DCC circuit
#define NVRM_PLLX_DCC_VCO_MIN (600000)

// Default PLLC output frequency
#define NVRM_PLLC_DEFAULT_FREQ_KHZ (600000)

// Defines number of EMC frequency steps for DFS 
#define NVRM_AP20_DFS_EMC_FREQ_STEPS (8)

// Defines maximum APB frequency (bug 559823)
#define NVRM_AP20_APB_MAX_KHZ (125000)

// Defines graphics Host frequency
#define NVRM_AP20_HOST_KHZ (108000)

// Defines main clock doubler support
#define NVRM_AP20_USE_OSC_DOUBLER (0)

/**
 * Defines frequency steps derived from PLLP0 fixed output to be used as System
 * clock source frequency. The frequency specified in kHz, and it will be rounded
 * up to the closest divider output. 
 */
#define NVRM_AP20_PLLP_POLICY_SYSTEM_CLOCK \
    PLLP_POLICY_ENTRY(24000)   /* PLLP divider 16, output frequency  24,000kHz */ \
    PLLP_POLICY_ENTRY(54000)   /* PLLP divider  6, output frequency  54,000kHz */ \
    PLLP_POLICY_ENTRY(72000)   /* PLLP divider  4, output frequency  72,000kHz */ \
    PLLP_POLICY_ENTRY(108000)  /* PLLP divider  2, output frequency 108,000kHz */ \
    PLLP_POLICY_ENTRY(216000)  /* PLLP divider  0, output frequency 216,000kHz */

/**
 * Defines frequency steps derived from PLLP0 fixed output to be used as CPU
 * clock source frequency. The frequency specified in kHz, and it will be rounded
 * up to the closest divider output. On AP20 we will use only main PLLP0 output,
 * and no divided down steps, so that PLLP_OUT4 divider output is available as
 * a source for external devices.
 */
#define NVRM_AP20_PLLP_POLICY_CPU_CLOCK \
    PLLP_POLICY_ENTRY(216000)  /* PLLP divider  0, output frequency 216,000kHz */

// On AP20 PLLP4 is used as 24MHz source for external devices. This setting will
// overwrite initial PLLP4 frequency after boot/resume from LP0.
#define NVRM_AP20_FIXED_PLLP4_SETTING (16)   /* 216 / (1 + 16/2) = 24 */

/**
 * Combines EMC 2x frequency and the respective set of EMC timing parameters for
 * pre-defined EMC configurations (DDR clock is running at EMC 1x frequency)
 */
typedef struct NvRmAp20EmcTimingConfigRec
{
    NvRmFreqKHz Emc2xKHz;
    const NvOdmSdramControllerConfigAdv* pOdmEmcConfig;
    NvU32 Emc2xClockSource;
    NvU32 Emc2xDivisor;
    NvU32 Emc2xUndividedIndex;
    NvRmFreqKHz CpuLimitKHz;
    NvU32 EmcDigDll;
} NvRmAp20EmcTimingConfig;

// Digital DLL override equation parameters within packed dqsib
// characterization data
#define EMC_DIG_DLL_OVERRIDE_0_LOW_KHZ_MULT_RANGE    7:0
#define EMC_DIG_DLL_OVERRIDE_0_LOW_KHZ_OFFS_RANGE   15:8
#define EMC_DIG_DLL_OVERRIDE_0_HIGH_KHZ_MULT_RANGE  23:16
#define EMC_DIG_DLL_OVERRIDE_0_HIGH_KHZ_OFFS_RANGE  31:24

/*****************************************************************************/

/**
 * Enables/disables module clock.
 * 
 * @param hDevice The RM device handle.
 * @param ModuleId Combined module ID and instance of the target module.
 * @param ClockState Target clock state.
 */
void
Ap20EnableModuleClock(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    ModuleClockState ClockState);

// Separate API to control TVDAC clock independently of TVO
// (when TVDAC is used for CRT)  
void
Ap20EnableTvDacClock(
    NvRmDeviceHandle hDevice,
    ModuleClockState ClockState);

/**
 * Resets module (assert/delay/deassert reset signal) if the hold paramter is
 * NV_FLASE. If the hols paramter is NV_TRUE, just assert the reset and return.
 * 
 * @param hDevice The RM device handle.
 * @param Module Combined module ID and instance of the target module.
 * @param hold      To hold or relese the reset.
 */
void 
AP20ModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId, NvBool hold);

/**
 * Resets 2D module.
 * 
 * @param hRmDevice The RM device handle. 
 */
void
NvRmPrivAp20Reset2D(NvRmDeviceHandle hRmDevice);

/**
 * Initializes clock source table.
 * 
 * @return Pointer to the clock sources descriptor table.
 */
NvRmClockSourceInfo* NvRmPrivAp20ClockSourceTableInit(void);

/**
 * Initializes PLL references table.
 * 
 * @param pPllReferencesTable A pointer to a pointer which this function sets
 *  to the PLL reference table base. 
 * @param pPllReferencesTableSize A pointer to a variable which this function
 *  sets to the PLL reference table size.
 */
void
NvRmPrivAp20PllReferenceTableInit(
    NvRmPllReference** pPllReferencesTable,
    NvU32* pPllReferencesTableSize);

/**
 * Controls PLLE.
 * 
 * @param hRmDevice The RM device handle.
 * @param Enable Specifies if PLLE should be enabled or disabled (PLLE power
 *  management is not supported, and it is never disabled as of now).
 */
void NvRmPrivAp20PllEControl(NvRmDeviceHandle hRmDevice, NvBool Enable);

/**
 * Initializes configuration structures and tables for DVFS controlled clocks.
 * 
 * @param hRmDevice The RM device handle. 
 */
void
NvRmPrivAp20ScaledClockConfigInit(NvRmDeviceHandle hRmDevice);

/**
 * Configures oscillator (main) clock doubler.
 * 
 * @param hRmDevice The RM device handle.
 * @param OscKHz Oscillator (main) clock frequency in kHz.
 * 
 * @return NvSuccess if the specified oscillator frequency is supported, and
 * NvError_NotSupported, otherwise.
 */
NvError
NvRmPrivAp20OscDoublerConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz OscKHz);

/**
 * Configures maximum core and memory clocks.
 * 
 * @param hRmDevice The RM device handle.
 */
void
NvRmPrivAp20FastClockConfig(NvRmDeviceHandle hRmDevice);

/**
 * Gets module frequency synchronized with EMC speed.
 * 
 * @param hRmDevice The RM device handle.
 * @param Module The target module ID.
 * 
 * @return Module frequency in kHz.
 */
NvRmFreqKHz NvRmPrivAp20GetEmcSyncFreq(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module);

/**
 * Clips EMC frequency high limit to one of the fixed DFS EMC configurations,
 * and if necessary adjust CPU high limit respectively.
 * 
 * @param hRmDevice The RM device handle.
 * @param pCpuHighKHz A pointer to the variable, which contains CPU frequency
 *  high limit in KHz (on entry - requested limit, on exit - clipped limit)
 * @param pEmcHighKHz A pointer to the variable, which contains EMC frequency
 *  high limit in KHz (on entry - requested limit, on exit - clipped limit)
 */
void
NvRmPrivAp20ClipCpuEmcHighLimits(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz* pCpuHighKHz,
    NvRmFreqKHz* pEmcHighKHz);

/**
 * Gets frequencies of DFS controlled clocks
 * 
 * @param hRmDevice The RM device handle.
 * @param pDfsKHz Output storage pointer for DFS clock frequencies structure
 *  (all frequencies returned in kHz).
 */
void
NvRmPrivAp20DfsClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsFrequencies* pDfsKHz);

/**
 * Configures DFS controlled clocks
 * 
 * @param hRmDevice The RM device handle.
 * @param pMaxKHz Pointer to the DFS clock frequencies upper limits
 * @param pDfsKHz Pointer to the target DFS frequencies structure on entry;
 *  updated with actual DFS clock frequencies on exit.
 * 
 * @return NV_TRUE if clock configuration is completed; NV_FALSE if this
 *  function has to be called again to complete configuration.
 */
NvBool
NvRmPrivAp20DfsClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsFrequencies* pMaxKHz,
    NvRmDfsFrequencies* pDfsKHz);

/**
 * Gets DFS domains frequencies to be set for suspend (LP1) entry/exit.
 *
 * @param hRmDevice The RM device handle.
 * @param TargetMv Targeted suspend core voltage in mV.
 * @param pDfsKHz Pointer to a structure filled in by this function with
 *  output clock frequencies.
 */
void
NvRmPrivAp20DfsSuspendFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmMilliVolts TargetMv,
    NvRmDfsFrequencies* pDfsKHz);

/**
 * Configures the sdio tap delay
 * 
 * @param hRmDevice The RM device handle.
 * @param Module The target module ID.
 * @param ClkSourceOffset Clock source offset.
 * @param ConfiguredFreqKHz The configured frequency in KHz.
 * 
 */
void
NvRmPrivAp20SdioTapDelayConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleId,
    NvU32 ClkSourceOffset,
    NvRmFreqKHz ConfiguredFreqKHz);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  // INCLUDED_AP20RM_CLOCKS_H 
