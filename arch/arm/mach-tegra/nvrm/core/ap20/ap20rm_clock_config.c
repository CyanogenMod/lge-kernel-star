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

#include "nvcommon.h"
#include "nvassert.h"
#include "nvrm_clocks.h"
#include "nvrm_hwintf.h"
#include "nvrm_drf.h"
#include "ap20rm_clocks.h"
#include "ap20/aremc.h"
#include "ap20/arclk_rst.h"
#include "ap20/arapbpm.h"
#include "ap15/ap15rm_private.h"
#include "nvodm_query.h"

// Enable CPU/EMC ratio policy
#define NVRM_LIMIT_CPU_EMC_RATIO (1)

// Use DRAM power down mode with EMC clock change
#define NVRM_EMC_CLKCHANGE_PD (1)

// Default CPU power good delay
#define NVRM_DEFAULT_CPU_PWRGOOD_US (2000) 

// Default PMU accuracy %
#define NVRM_DEFAULT_PMU_ACCURACY_PCT (3)

// Minimum core over CPU voltage margin (at SoC)
#define NV_AP20_CORE_OVER_CPU_MV (120)

/*****************************************************************************/

/*
 * TODO: Basic DFS clock control policy outline:
 */

// Limit frequencies ratio for AHB : System >= 1:2 and APB : System >= 1 : 4
#define LIMIT_SYS_TO_AHB_APB_RATIOS (1)

// PLLP2 must be used as a variable low frequency source for System clock.
#define PLLP_POLICY_ENTRY(KHz) \
 { NvRmClockSource_PllP2,\
   (NVRM_PLLP_FIXED_FREQ_KHZ * 2)/((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz),\
   ((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz - 2)\
 },
static const NvRmDfsSource s_Ap20PllPSystemClockPolicy[] =
{
    NVRM_AP20_PLLP_POLICY_SYSTEM_CLOCK
};
static const NvU32 s_Ap20PllPSystemClockPolicyEntries =
    NV_ARRAY_SIZE(s_Ap20PllPSystemClockPolicy);
#undef PLLP_POLICY_ENTRY


// PLLP4 must be used as a variable low frequency source for cpu clock.
#define PLLP_POLICY_ENTRY(KHz) \
 { NvRmClockSource_PllP4,\
   (NVRM_PLLP_FIXED_FREQ_KHZ * 2)/((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz),\
   ((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz - 2)\
 },
static const NvRmDfsSource s_Ap20PllPCpuClockPolicy[] =
{
    NVRM_AP20_PLLP_POLICY_CPU_CLOCK
};
static const NvU32 s_Ap20PllPCpuClockPolicyEntries =
    NV_ARRAY_SIZE(s_Ap20PllPCpuClockPolicy);
#undef PLLP_POLICY_ENTRY

// EMC timing registers
static const NvU32 s_EmcTimingRegAddrRev20[] =
{
    EMC_RC_0,                   /* RC */
    EMC_RFC_0,                  /* RFC */
    EMC_RAS_0,                  /* RAS */
    EMC_RP_0,                   /* RP */
    EMC_R2W_0,                  /* R2W */
    EMC_W2R_0,                  /* W2R */
    EMC_R2P_0,                  /* R2P */
    EMC_W2P_0,                  /* W2P */
    EMC_RD_RCD_0,               /* RD_RCD */
    EMC_WR_RCD_0,               /* WR_RCD */
    EMC_RRD_0,                  /* RRD */
    EMC_REXT_0,                 /* REXT */
    EMC_WDV_0,                  /* WDV */
    EMC_QUSE_0,                 /* QUSE */
    EMC_QRST_0,                 /* QRST */
    EMC_QSAFE_0,                /* QSAFE */
    EMC_RDV_0,                  /* RDV */
    EMC_REFRESH_0,              /* REFRESH */
    EMC_BURST_REFRESH_NUM_0,    /* BURST_REFRESH_NUM */
    EMC_PDEX2WR_0,              /* PDEX2WR */
    EMC_PDEX2RD_0,              /* PDEX2RD */
    EMC_PCHG2PDEN_0,            /* PCHG2PDEN */
    EMC_ACT2PDEN_0,             /* ACT2PDEN */
    EMC_AR2PDEN_0,              /* AR2PDEN */
    EMC_RW2PDEN_0,              /* RW2PDEN */
    EMC_TXSR_0,                 /* TXSR */
    EMC_TCKE_0,                 /* TCKE */
    EMC_TFAW_0,                 /* TFAW */
    EMC_TRPAB_0,                /* TRPAB */
    EMC_TCLKSTABLE_0,           /* TCLKSTABLE */
    EMC_TCLKSTOP_0,             /* TCLKSTOP */
    EMC_TREFBW_0,               /* TREFBW */
    EMC_QUSE_EXTRA_0,           /* QUSE_EXTRA */
    EMC_FBIO_CFG6_0,            /* FBIO_CFG6 */
    EMC_ODT_WRITE_0,            /* ODT_WRITE */
    EMC_ODT_READ_0,             /* ODT_READ */
    EMC_FBIO_CFG5_0,            /* FBIO_CFG5 */
    EMC_CFG_DIG_DLL_0,          /* CFG_DIG_DLL */
    EMC_DLL_XFORM_DQS_0,        /* DLL_XFORM_DQS */
    EMC_DLL_XFORM_QUSE_0,       /* DLL_XFORM_QUSE */
    EMC_ZCAL_REF_CNT_0,         /* ZCAL_REF_CNT */
    EMC_ZCAL_WAIT_CNT_0,        /* ZCAL_WAIT_CNT */
    EMC_AUTO_CAL_INTERVAL_0,    /* AUTO_CAL_INTERVAL */
    EMC_CFG_CLKTRIM_0_0,        /* CFG_CLKTRIM_0 */
    EMC_CFG_CLKTRIM_1_0,        /* CFG_CLKTRIM_1 */
    EMC_CFG_CLKTRIM_2_0,        /* CFG_CLKTRIM_2 */
};

#define EMC_CFG_DIG_DLL_INDEX (37)

// Sorted list of timing parameters for discrete set of EMC frequencies used
// by DFS; entry 0 specifies timing parameters for PLLM0 output frequency.
static NvRmAp20EmcTimingConfig
s_Ap20EmcConfigSortedTable[NVRM_AP20_DFS_EMC_FREQ_STEPS];

static struct Ap20EmcConfigRec
{
    // Index of selected EMC configuration entry
    NvU32 Index;

    // Status of undivided PLLM0 path
    NvBool UdPllM0;

    // Pointers to EMC clock state
    NvRmModuleClockState* pEmc2xState;

    // Pointers to EMC clock descriptors
    NvRmModuleClockInfo* pEmcInfo;

    // Array of EMC timing registers
    const NvU32* pEmcTimingReg;

    // Total number of EMC timing registers
    NvU32 EmcTimingRegNum;

} s_Ap20EmcConfig = {0};

static struct Ap20VdeConfigRec
{
    // Pointer to VDE clock descriptor
    NvRmModuleClockInfo* pVdeInfo;

    // Pointer to VDE clock state
    NvRmModuleClockState* pVdeState;

} s_Ap20VdeConfig = {0};

static struct Ap20CpuConfigRec
{
    // Number of PLLX frequency steps
    NvU32 PllXStepsNo;

    // PLLX frequency steps table pointer 
    const NvRmFreqKHz* pPllXStepsKHz;

    // Core over CPU voltage dependency parameters:
    // Vcore >= CoreOverCpuSlope * Vcpu + CoreOverCpuOffset
    NvU32 CoreOverCpuOffset;
    NvU32 CoreOverCpuSlope;

} s_Ap20CpuConfig = {0};

/*****************************************************************************/
/*****************************************************************************/

static void
Ap20Emc2xClockStateUpdate(
    NvRmDeviceHandle hRmDevice)
{
    NvU32 reg;
    NvRmFreqKHz SourceClockFreq;
    NvRmModuleClockInfo* pCinfo = s_Ap20EmcConfig.pEmcInfo;
    NvRmModuleClockState* pCstate = s_Ap20EmcConfig.pEmc2xState;

    NV_ASSERT(pCinfo && pCstate);

    // Determine EMC2x source and divider setting; update EMC2x clock state
    reg = NV_REGR(hRmDevice,
                  NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkSourceOffset);
    pCstate->Divider =
        ((reg >> pCinfo->DivisorFieldShift) & pCinfo->DivisorFieldMask);
    pCstate->SourceClock =
        ((reg >> pCinfo->SourceFieldShift) & pCinfo->SourceFieldMask);
    s_Ap20EmcConfig.UdPllM0 = NV_DRF_VAL(CLK_RST_CONTROLLER,
        CLK_SOURCE_EMC, USE_PLLM_UD, reg) ? NV_TRUE : NV_FALSE;
    if (s_Ap20EmcConfig.UdPllM0)
    {
        NV_ASSERT(  // policy: Src/Div settings must be synced with UD path
            (pCstate->Divider == 0) && (pCstate->SourceClock ==
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0));
    }
    SourceClockFreq =
        NvRmPrivGetClockSourceFreq(pCinfo->Sources[pCstate->SourceClock]);

    // Fractional divider output = (Source Frequency * 2) / (divider + 2)
    pCstate->actual_freq = ((SourceClockFreq << 1) / (pCstate->Divider + 2));
}

static NvBool
Ap20EmcClkChangeConfig(
    NvRmDeviceHandle hRmDevice)
{
// NO-DEVICE for dummy MRW/MRS commands
#define NULL_DEV_SELECTN (3)

    NvU32 cfg2, cfg5;

    cfg2 = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_CFG_2_0);
    cfg5 = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_FBIO_CFG5_0);

    switch (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_TYPE, cfg5))
    {
        case EMC_FBIO_CFG5_0_DRAM_TYPE_LPDDR2:
#if NVRM_EMC_CLKCHANGE_PD
            // Dummy mode control command to activate PD state machine
            NV_REGW(hRmDevice,
                    NvRmPrivModuleID_ExternalMemoryController, 0, EMC_MRW_0,
                    NV_DRF_NUM(EMC, MRW, MRW_DEV_SELECTN, NULL_DEV_SELECTN));
            NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);

            cfg2 = NV_FLD_SET_DRF_DEF(
                EMC, CFG_2, CLKCHANGE_PD_ENABLE, ENABLED, cfg2);
            cfg2 = NV_FLD_SET_DRF_DEF(
                EMC, CFG_2, CLKCHANGE_SR_ENABLE, DISABLED, cfg2);
            break;
#endif
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR2:
#if NVRM_EMC_CLKCHANGE_PD
            // Dummy mode control command to activate PD state machine
            NV_REGW(hRmDevice,
                    NvRmPrivModuleID_ExternalMemoryController, 0, EMC_MRS_0,
                    NV_DRF_NUM(EMC, MRS, MRS_DEV_SELECTN, NULL_DEV_SELECTN));
            NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
#endif
            cfg2 = NV_FLD_SET_DRF_DEF(
                EMC, CFG_2, CLKCHANGE_PD_ENABLE, DISABLED, cfg2);
            cfg2 = NV_FLD_SET_DRF_DEF(
                EMC, CFG_2, CLKCHANGE_SR_ENABLE, ENABLED, cfg2);
            break;
        default:
            NV_ASSERT(!"Not supported DRAM type");
            return NV_FALSE;
    }
    cfg2 = NV_FLD_SET_DRF_DEF(
        EMC, CFG_2, CLKCHANGE_REQ_ENABLE, ENABLED, cfg2);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_CFG_2_0, cfg2);
    return NV_TRUE;
}

static NvRmFreqKHz Ap20CpuToEmcRatio(NvRmFreqKHz Emc2xKHz)
{
#if NVRM_LIMIT_CPU_EMC_RATIO
    /*
     * CPU/EMC ratio is limited by the policy curve tabulated below: when cpu
     * frequency is reduced by 25%, emc frequency along the curve is reduced
     * by 50%.
     */
    static const NvU32 CpuToEmc[] = { 0,
        7,  10, 11, 13, 14, 15, 17, 18, 18, 19, 20, 21, 22, 22, 23, 24,
        24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32
    };
    #define CPU_TO_EMC_MAX_RATIO (12)
    
    NvRmFreqKHz CpuKHz;
    NvRmFreqKHz CpuMaxKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
    NvRmFreqKHz Emc2xMaxKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);

    NvU32 M = NV_ARRAY_SIZE(CpuToEmc) - 1;
    NvU32 x = (Emc2xKHz * M + Emc2xMaxKHz - 1) / Emc2xMaxKHz;
    NV_ASSERT((x >= 1) && (x <= M));
    CpuKHz = (CpuMaxKHz * CpuToEmc[x] + M - 1) / M;
    CpuKHz = NV_MIN(CpuKHz, (Emc2xKHz * (CPU_TO_EMC_MAX_RATIO / 2)));

    return CpuKHz;
#else
    return NvRmFreqMaximum;
#endif
}

static NvU32 Ap20EmcDigDllUpdate(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz PllM0KHz,
    NvRmFreqKHz PllP0KHz,
    NvRmFreqKHz Emc2xKHz,
    NvU32 OdmDigDll)
{
    NvU32 mult, offset, d;
    NvU32 dqsib = NvRmPrivGetEmcDqsibOffset(hRmDevice);
    NV_ASSERT(PllP0KHz < PllM0KHz);

    /* Equation to determine digital dll override value based on dqsib
     * characterization data:
     * d = b - (m * (Emc2xMHz - PllP0MHz) / 2 / 100)), if Emc2xMHz > PllP0MHz
     * d = b if Emc2xMHz <= PllP0Mhz
     * Dqsib value packs 2 pairs of slope m/offset b settings. One pair ("high
     * m, b") is applied only when undivided PLLM0 is used as a clock source.
     * The other ("low m, b") is applied to all other clock sources.
     */
    if (Emc2xKHz < PllM0KHz)
    {
        mult = NV_DRF_VAL(EMC, DIG_DLL_OVERRIDE, LOW_KHZ_MULT, dqsib);
        offset = NV_DRF_VAL(EMC, DIG_DLL_OVERRIDE, LOW_KHZ_OFFS, dqsib);
    }
    else
    {
        mult = NV_DRF_VAL(EMC, DIG_DLL_OVERRIDE, HIGH_KHZ_MULT, dqsib);
        offset = NV_DRF_VAL(EMC, DIG_DLL_OVERRIDE, HIGH_KHZ_OFFS, dqsib);
    }
    if (Emc2xKHz < PllP0KHz)
        Emc2xKHz = PllP0KHz;

    d = ((Emc2xKHz - PllP0KHz) + 500) / 1000;
    d = offset - (mult * d + 100) / 200;
    d = NV_FLD_SET_DRF_NUM(EMC, CFG_DIG_DLL, CFG_DLL_OVERRIDE_VAL,
                            d, OdmDigDll);
    return d;
}

static void Ap20EmcConfigInit(NvRmDeviceHandle hRmDevice)
{
    NvRmFreqKHz Emc2xKHz, SourceKHz;
    NvU32 i, j, k, Source, ConfigurationsCount, UndividedIndex, dll;
    NvU32 Revision = 0;
    
    NvRmFreqKHz PllM0KHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    NvRmFreqKHz PllP0KHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0);
    const NvRmModuleClockLimits* pEmcClockLimits =
         NvRmPrivGetSocClockLimits(NvRmPrivModuleID_ExternalMemoryController);
    const NvOdmSdramControllerConfigAdv* pEmcConfigurations =
        NvOdmQuerySdramControllerConfigGet(&ConfigurationsCount, &Revision);

    // Init memory configuration structure
    NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
        hRmDevice, NvRmPrivModuleID_ExternalMemoryController,
        &s_Ap20EmcConfig.pEmcInfo, &s_Ap20EmcConfig.pEmc2xState));

     s_Ap20EmcConfig.pEmcTimingReg = &s_EmcTimingRegAddrRev20[0];
     s_Ap20EmcConfig.EmcTimingRegNum = NV_ARRAY_SIZE(s_EmcTimingRegAddrRev20);
     s_Ap20EmcConfig.Index = NVRM_AP20_DFS_EMC_FREQ_STEPS; // invalid index

     // Clean table, which invalidates PLLM0 entry - no EMC DFS if exits
     // before sorting below
     NvOsMemset(s_Ap20EmcConfigSortedTable, 0,
                sizeof(s_Ap20EmcConfigSortedTable));

     // Get EMC2x clock state from h/w
     Ap20Emc2xClockStateUpdate(hRmDevice);

     // Configure EMC clock change mechanism - exit if not supported
     if (!Ap20EmcClkChangeConfig(hRmDevice))
         return;

     // Check if configuration table is provided by ODM
     if ((ConfigurationsCount == 0) || (pEmcConfigurations == NULL))
         return;

    // EMC DVFS is supported on AP20 starting with A02 chip
    if ((hRmDevice->ChipId.Major == 1) && (hRmDevice->ChipId.Minor <= 1))
        return;

    // Only 2.0 table revision is supported
    if (Revision != 0x20)
    {
        NV_ASSERT(!"Invalid configuration table revision");
        return;
    }

     // Check PLLs clock range
     if ((PllP0KHz < pEmcClockLimits->MinKHz) ||
         (PllP0KHz > pEmcClockLimits->MaxKHz))
     {
         NV_ASSERT(!"PLLP0 is outside supported EMC range");
         return;
     }
     if ((PllM0KHz < pEmcClockLimits->MinKHz) ||
         (PllM0KHz > pEmcClockLimits->MaxKHz))
     {
         NV_ASSERT(!"PLLM0 is outside supported EMC range");
         return;
     }

     // Check if PLLM0 is configured by boot loader as EMC clock source
     if (s_Ap20EmcConfig.pEmc2xState->SourceClock !=
         CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0)
     {
         NV_ASSERT(!"Other than PLLM0 clock source is used for EMC");
         return;
     }

     // Sort list of EMC timing parameters in descending order of frequencies
     // evenly divided down from the selected source. Supported sources: PLLM0
     // PLLP0, and Oscillator
     Source = CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0;
     for (i = 0; i < NVRM_AP20_DFS_EMC_FREQ_STEPS; )
     {
         SourceKHz = Emc2xKHz = NvRmPrivGetClockSourceFreq(
             s_Ap20EmcConfig.pEmcInfo->Sources[Source]);

         for (k = 0, UndividedIndex = i; i < NVRM_AP20_DFS_EMC_FREQ_STEPS; )
         {
             s_Ap20EmcConfigSortedTable[i].Emc2xKHz = 0; // mark entry invalid
             for (j = 0; j < ConfigurationsCount; j++)
             {
                 // Find match with 0.4% accuracy for ODM configuration
                 if (((pEmcConfigurations[j].SdramKHz * 2) <=
                      (Emc2xKHz + (Emc2xKHz >> 8))) &&
                     ((pEmcConfigurations[j].SdramKHz * 2) >=
                      (Emc2xKHz - (Emc2xKHz >> 8))))
                 {
                     NV_ASSERT(pEmcConfigurations[j].Revision == Revision);
                     NV_ASSERT(pEmcConfigurations[j].EmcTimingParamNum ==
                               s_Ap20EmcConfig.EmcTimingRegNum);

                     s_Ap20EmcConfigSortedTable[i].pOdmEmcConfig =
                         &pEmcConfigurations[j];

                     s_Ap20EmcConfigSortedTable[i].Emc2xClockSource = Source;
                     s_Ap20EmcConfigSortedTable[i].Emc2xUndividedIndex =
                         UndividedIndex;
                     s_Ap20EmcConfigSortedTable[i].Emc2xKHz = Emc2xKHz;

                    // Update digital dll settings
                    dll =  pEmcConfigurations[j].EmcTimingParameters[
                            EMC_CFG_DIG_DLL_INDEX];
                    s_Ap20EmcConfigSortedTable[i].EmcDigDll =
                        Ap20EmcDigDllUpdate(hRmDevice, PllM0KHz, PllP0KHz,
                                            Emc2xKHz, dll);

                     /*
                      * The undivided table entry specifies parameters for
                      * EMC2xKHz = SourceKHz; the EMC divisor field is set to
                      * "0". Next table entries specify parameters for EMC2xKHz
                      * = SourceKHz / (2 * k); the EMC divisor field should be
                      * set as 2 * (2 * k) - 2 = 4 * k - 2.
                      */
                     if (k == 0)
                         s_Ap20EmcConfigSortedTable[i].Emc2xDivisor = 0;
                     else
                         s_Ap20EmcConfigSortedTable[i].Emc2xDivisor =
                                                            (k << 2) - 2;
                     // Check boot configuration (to be recognized boot Src/Div
                     // settings must be synced with UD path)
                     if ((SourceKHz == PllM0KHz) &&
                         (s_Ap20EmcConfigSortedTable[i].Emc2xDivisor ==
                          s_Ap20EmcConfig.pEmc2xState->Divider))
                     {
                         if (s_Ap20EmcConfig.UdPllM0 == (i == 0))
                             s_Ap20EmcConfig.Index = i;
                     }
                     s_Ap20EmcConfigSortedTable[i].CpuLimitKHz =
                         Ap20CpuToEmcRatio(Emc2xKHz);
                     break;
                 }
             }
             if (s_Ap20EmcConfigSortedTable[i].Emc2xKHz != 0)
                 i++;       // Entry found - advance sorting index
             else if (k == 0)
                 break;     // Abort sorting if undiveded source not found

             Emc2xKHz = SourceKHz / ((++k) << 1);
             if (Emc2xKHz < pEmcClockLimits->MinKHz)
                 break;     // Abort sorting if min frequency reached
         }

         if (i == 0) 
             break;         // Finish sorting if PLLM0 entry not found

         // Next source selection
         if (Source ==
             CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0)
         {
             Source =       // After PLLM0 try PLLP0 as a source
             CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLP_OUT0;
         }
         else
             break;     // Finish sorting
     }
}

// EMC module virtual base address to speed up timing update
static void* s_pEmcBaseReg = NULL;

static void
Ap20EmcTimingSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmAp20EmcTimingConfig* pEmcConfig)
{
    NvU32 i, a, d;

    if (s_pEmcBaseReg == NULL)
    {
        NvRmModuleTable *tbl = NvRmPrivGetModuleTable(hRmDevice);
        s_pEmcBaseReg = (tbl->ModInst +
             tbl->Modules[NvRmPrivModuleID_ExternalMemoryController].Index)->VirtAddr;
    }
    a = (NvU32)s_pEmcBaseReg;

    for (i = 0; i < s_Ap20EmcConfig.EmcTimingRegNum; i++)
    {
	    //20110218, , DVFS patch [START]
	    #if 1
        if (s_Ap20EmcConfig.pEmcTimingReg[i] == EMC_DLL_XFORM_DQS_0)
        {
            NvU32 mask =
                NV_DRF_NUM(EMC, DLL_XFORM_DQS, XFORM_DQS_OFFS, 0xFFFFFFFFUL);
            NvU32 offs = NV_DRF_NUM(EMC, DLL_XFORM_DQS, XFORM_DQS_OFFS, 0xC0);
            d = pEmcConfig->pOdmEmcConfig->EmcTimingParameters[i];
            d = ((((d & mask) << 2) - offs) & mask) | (d & (~mask));
        }
        else if (s_Ap20EmcConfig.pEmcTimingReg[i] == EMC_CFG_DIG_DLL_0)
	    #else
        if (i == EMC_CFG_DIG_DLL_INDEX)
	    #endif
	    //20110218, , DVFS patch [END]
            d = pEmcConfig->EmcDigDll;
        else
            d = pEmcConfig->pOdmEmcConfig->EmcTimingParameters[i];
        a = (((NvU32)(s_pEmcBaseReg)) + s_Ap20EmcConfig.pEmcTimingReg[i]);
        NV_WRITE32(a, d);
    }

    // Clear clock change interrupt status (write 1 to clear)
    a = (((NvU32)(s_pEmcBaseReg)) + EMC_INTSTATUS_0);
    d = NV_DRF_NUM(EMC, INTSTATUS, CLKCHANGE_COMPLETE_INT, 1);
    NV_WRITE32(a, d);

    d = NV_READ32(a);   // make sure writes are completed
}

static void
Ap20EmcTimingSetFinish(
    NvRmDeviceHandle hRmDevice,
    const NvRmAp20EmcTimingConfig* pEmcConfig)
{
    NvU32 a, d;
    NV_ASSERT(s_pEmcBaseReg);

    // After EMC clock change is completed, digital DLL should be restarted
    // (provided it is enabled for current EMC frequency)
    if (!NV_DRF_VAL(EMC, CFG_DIG_DLL, CFG_DLL_EN, pEmcConfig->EmcDigDll))
        return;

    a = (((NvU32)(s_pEmcBaseReg)) + EMC_INTSTATUS_0);
//20110318, , nVidia recommandation for no LCD response [START]
#if 0
    for (;;)
    {
        d = NV_DRF_VAL(EMC, INTSTATUS, CLKCHANGE_COMPLETE_INT, NV_READ32(a));
        if (d)
            break;
    }
#endif
    NvOsWaitUS(10);
//20110318, , nVidia recommandation for no LCD response [END]

    a = (((NvU32)(s_pEmcBaseReg)) + EMC_CFG_DIG_DLL_0);
    NV_WRITE32(a, pEmcConfig->EmcDigDll);
    d = NV_READ32(a);   // make sure writes are completed

    a = (((NvU32)(s_pEmcBaseReg)) + EMC_TIMING_CONTROL_0);
    d = NV_DRF_NUM(EMC, TIMING_CONTROL, TIMING_UPDATE, 1);
    NV_WRITE32(a, d);
    d = NV_READ32(a);   // make sure writes are completed
}

static void
Ap20EmcDividerBackgroundSet( 
    NvRmDeviceHandle hRmDevice,
    NvU32 value)
{
    NvU32 cfg2, clk;

    // Check if divider is actually to be changed - exit if not
    clk = cfg2 = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
    clk = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                             EMC_2X_CLK_DIVISOR, value, clk);
    if (cfg2 == clk)
        return;

    // Disable EMC clock change request, so that following EMC clock divider
    // change will not trigger EMC timing switch ("background change")
    cfg2 = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_CFG_2_0);
    NV_ASSERT(NV_DRF_VAL(EMC, CFG_2, CLKCHANGE_REQ_ENABLE, cfg2) == 1);
    cfg2 = NV_FLD_SET_DRF_DEF(EMC, CFG_2, CLKCHANGE_REQ_ENABLE,
                              DISABLED, cfg2);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
            EMC_CFG_2_0, cfg2);
    cfg2 = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_CFG_2_0);    // make sure write is completed

    // Set EMC clock divider (UD bit must be set during "background change")
    NV_ASSERT(NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_EMC, USE_PLLM_UD, clk));
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, clk);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);

    // Restore EMC clock change request
    cfg2 = NV_FLD_SET_DRF_DEF(EMC, CFG_2, CLKCHANGE_REQ_ENABLE,
                              ENABLED, cfg2);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
            EMC_CFG_2_0, cfg2);
    cfg2 = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                   EMC_CFG_2_0);    // make sure write is completed
}

static void
Ap20EmcSwitchToUndividedPllM0(
        NvRmDeviceHandle hRmDevice,
        const NvRmAp20EmcTimingConfig* pEmcConfig)
{
    NvU32 reg;
    NV_ASSERT(pEmcConfig->Emc2xKHz);    // validate table entry

    // Update EMC shadow registers
    Ap20EmcTimingSet(hRmDevice, pEmcConfig);

    // Set EMC clock source as undivided PLLM0 (divider is "don't care" in this
    // case, so keep it as is to satisfy restriction: source and divider can not
    // be changed simultaneously)
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                  EMC_2X_CLK_SRC, pEmcConfig->Emc2xClockSource, reg);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                             USE_PLLM_UD, 1, reg);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    Ap20EmcTimingSetFinish(hRmDevice, pEmcConfig);

    // Now set new divider value. Note that PLLM_UD bit is already set, so
    // the actual EMC frequency is not changed. Hence, no need to update EMC
    // timing - the old settings already match the frequency.
    Ap20EmcDividerBackgroundSet(hRmDevice, pEmcConfig->Emc2xDivisor);

    // Update EMC state
    s_Ap20EmcConfig.UdPllM0 = NV_TRUE;
    s_Ap20EmcConfig.pEmc2xState->SourceClock = pEmcConfig->Emc2xClockSource;
    s_Ap20EmcConfig.pEmc2xState->Divider = pEmcConfig->Emc2xDivisor;
    s_Ap20EmcConfig.pEmc2xState->actual_freq = pEmcConfig->Emc2xKHz;
    NvRmPrivMemoryClockReAttach(
        hRmDevice, s_Ap20EmcConfig.pEmcInfo, s_Ap20EmcConfig.pEmc2xState);
}

static void
Ap20EmcSwitchFromUndividedPllM0(
        NvRmDeviceHandle hRmDevice,
        const NvRmAp20EmcTimingConfig* pEmcConfig)
{
    NvU32 reg;
    NV_ASSERT(pEmcConfig->Emc2xKHz);    // validate table entry

    // 1st set new divider value. Note that PLLM_UD bit is still set, so the
    // actual EMC frequency is not changed. Hence, no need to update EMC
    // timing - the old settings already match the frequency.
    Ap20EmcDividerBackgroundSet(hRmDevice, pEmcConfig->Emc2xDivisor);

    // Update EMC shadow registers
    Ap20EmcTimingSet(hRmDevice, pEmcConfig);

    // Now set new EMC clock source and disable undivided path (can be done
    // in one shot - cumulatively it is considered as source change only)
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                  EMC_2X_CLK_SRC, pEmcConfig->Emc2xClockSource, reg);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                             USE_PLLM_UD, 0, reg);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    Ap20EmcTimingSetFinish(hRmDevice, pEmcConfig);

    // Update EMC state
    s_Ap20EmcConfig.UdPllM0 = NV_FALSE;
    s_Ap20EmcConfig.pEmc2xState->SourceClock = pEmcConfig->Emc2xClockSource;
    s_Ap20EmcConfig.pEmc2xState->Divider = pEmcConfig->Emc2xDivisor;
    s_Ap20EmcConfig.pEmc2xState->actual_freq = pEmcConfig->Emc2xKHz;
    NvRmPrivMemoryClockReAttach(
        hRmDevice, s_Ap20EmcConfig.pEmcInfo, s_Ap20EmcConfig.pEmc2xState);
}

static void
Ap20EmcSwitchDividedSources(
        NvRmDeviceHandle hRmDevice,
        const NvRmAp20EmcTimingConfig* pEmcConfig)
{
    NvU32 reg, div, src;
    NV_ASSERT(pEmcConfig->Emc2xKHz);    // validate table entry

    // Update EMC shadow registers
    Ap20EmcTimingSet(hRmDevice, pEmcConfig);

    // This switch must be called only when original and target configurations
    // have either common source or common divider - switch in one shot.
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
    src = NV_DRF_VAL(
        CLK_RST_CONTROLLER, CLK_SOURCE_EMC, EMC_2X_CLK_SRC, reg);
    div = NV_DRF_VAL(
        CLK_RST_CONTROLLER, CLK_SOURCE_EMC, EMC_2X_CLK_DIVISOR, reg);
    NV_ASSERT((src == pEmcConfig->Emc2xClockSource) ||
              (div == pEmcConfig->Emc2xDivisor));

    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                  EMC_2X_CLK_DIVISOR, pEmcConfig->Emc2xDivisor, reg);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                  EMC_2X_CLK_SRC, pEmcConfig->Emc2xClockSource, reg);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, reg);
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    Ap20EmcTimingSetFinish(hRmDevice, pEmcConfig);

    // Update EMC state (undivided path status not changed)
    NV_ASSERT(!s_Ap20EmcConfig.UdPllM0);
    s_Ap20EmcConfig.pEmc2xState->SourceClock = pEmcConfig->Emc2xClockSource;
    s_Ap20EmcConfig.pEmc2xState->Divider = pEmcConfig->Emc2xDivisor;
    s_Ap20EmcConfig.pEmc2xState->actual_freq = pEmcConfig->Emc2xKHz;
    NvRmPrivMemoryClockReAttach(
        hRmDevice, s_Ap20EmcConfig.pEmcInfo, s_Ap20EmcConfig.pEmc2xState);
}

static NvBool
Ap20Emc2xClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmFreqKHz* pCpuTargetKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 i;
    NvBool FinalStep = NV_TRUE;
    NV_ASSERT(DomainKHz <= MaxKHz);
    NV_ASSERT(s_Ap20EmcConfigSortedTable[0].Emc2xKHz <= MaxKHz);

    // If PLLM0 entry in EMC frequeuncies table is invalid, EMC frequency
    // will not be scaled; just fill in current EMC frequency
    if (s_Ap20EmcConfigSortedTable[0].Emc2xKHz == 0)
    {
        pDfsSource->SourceId = NvRmClockSource_Invalid;  // invalidate source
        pDfsSource->DividerSetting = NVRM_AP20_DFS_EMC_FREQ_STEPS;
        pDfsSource->SourceKHz = s_Ap20EmcConfig.pEmc2xState->actual_freq;
        pDfsSource->MinMv = NvRmVoltsMaximum; // no v-scaling in this case
        return FinalStep;
    }

    // Search sorted pre-defind EMC frequencies for the entry above and closest
    // to the traget that also has CPU limit above the CPU target. Use PLLM0
    // entry if not found.
    for (i = NVRM_AP20_DFS_EMC_FREQ_STEPS; i > 0;)
    {
        i--;
        if ((DomainKHz <= s_Ap20EmcConfigSortedTable[i].Emc2xKHz) &&
            (*pCpuTargetKHz <= s_Ap20EmcConfigSortedTable[i].CpuLimitKHz))
            break;
    }

    // In case, when boot EMC configuration does not match any entry in
    // EMC sorted table switch to undivided PLLM0 first
    if ((i != 0) && (s_Ap20EmcConfig.Index >= NVRM_AP20_DFS_EMC_FREQ_STEPS))
    {
        i = 0;
        FinalStep = NV_FALSE;
    }

    // Target can be reached in one step, provided: 
    // - either current or target entry is PLLM0     OR
    // - current and target entries have same source OR
    // - current and target entries have same divider
    if ((i != 0) && (s_Ap20EmcConfig.Index != 0) &&
        (s_Ap20EmcConfigSortedTable[i].Emc2xDivisor !=
         s_Ap20EmcConfigSortedTable[s_Ap20EmcConfig.Index].Emc2xDivisor) &&
        (s_Ap20EmcConfigSortedTable[i].Emc2xClockSource !=
         s_Ap20EmcConfigSortedTable[s_Ap20EmcConfig.Index].Emc2xClockSource))
    {
        i = 0;  // one-step check failed - use PLLM0 as intermediate target
        FinalStep = NV_FALSE;
    }

    // Record found EMC target, and limit CPU target if necessary
    pDfsSource->DividerSetting = i;
    pDfsSource->SourceId = s_Ap20EmcConfig.pEmcInfo->Sources[
        s_Ap20EmcConfigSortedTable[i].Emc2xClockSource];
    pDfsSource->SourceKHz = s_Ap20EmcConfigSortedTable[i].Emc2xKHz;
    pDfsSource->MinMv =
        s_Ap20EmcConfigSortedTable[i].pOdmEmcConfig->EmcCoreVoltageMv;
    if (*pCpuTargetKHz > s_Ap20EmcConfigSortedTable[i].CpuLimitKHz)
        *pCpuTargetKHz = s_Ap20EmcConfigSortedTable[i].CpuLimitKHz;
    return FinalStep;
}

static void
Ap20Emc2xClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
    NvU32 Index;

    // Always return the requested source frequency
    *pDomainKHz = pDfsSource->SourceKHz;
    NV_ASSERT(*pDomainKHz);

    // If no valid source is found, EMC frequency is not scaled.
    if (pDfsSource->SourceId == NvRmClockSource_Invalid)
        return;

    // Divider settings in EMC source descriptor is an index into the table of
    // pre-defined EMC configurations in descending frequency order.
    Index = pDfsSource->DividerSetting;
    if (Index == s_Ap20EmcConfig.Index)
        return;     // do nothing if new index is the same as current

    // Switch EMC to the new target
    if (Index == 0)
    {
        Ap20EmcSwitchToUndividedPllM0(
            hRmDevice, &s_Ap20EmcConfigSortedTable[Index]);
    }
    else if (s_Ap20EmcConfig.Index == 0)
    {
        Ap20EmcSwitchFromUndividedPllM0(
            hRmDevice, &s_Ap20EmcConfigSortedTable[Index]);
    }
    else
    {
        Ap20EmcSwitchDividedSources(
            hRmDevice, &s_Ap20EmcConfigSortedTable[Index]);
    }
    s_Ap20EmcConfig.Index = Index;
}

/*****************************************************************************/

NvRmFreqKHz
NvRmPrivAp20GetEmcSyncFreq(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module)
{
    NvRmFreqKHz FreqKHz;

    switch (Module)
    {
        case NvRmModuleID_2D:
        case NvRmModuleID_Epp:
            // Scale down 2D/EPP whith EMC clock (set 2D/EPP frequency at
            // 50% of max when EMC clock is at or below 50% of max)
            FreqKHz = NvRmPrivGetSocClockLimits(Module)->MaxKHz;
            if ((0 < s_Ap20EmcConfig.Index) &&
                (s_Ap20EmcConfig.Index < NVRM_AP20_DFS_EMC_FREQ_STEPS))
                FreqKHz >>= 1;
            break;

        case NvRmModuleID_GraphicsHost:
            FreqKHz = NVRM_AP20_HOST_KHZ;
            break;

        default:
            NV_ASSERT(!"Invalid module for EMC synchronization");
            FreqKHz = NvRmPrivGetSocClockLimits(Module)->MaxKHz;
            break;
    }
    return FreqKHz;
}

void
NvRmPrivAp20ClipCpuEmcHighLimits(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz* pCpuHighKHz,
    NvRmFreqKHz* pEmcHighKHz)
{
#if !NV_OAL
    NvU32 i;
    NvRmFreqKHz EmcKHz;
    NvRmFreqKHz MinKHz = NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Emc);
    NV_ASSERT(pEmcHighKHz && pCpuHighKHz);

    // Nothing to do if no EMC scaling.
    if (s_Ap20EmcConfigSortedTable[0].Emc2xKHz == 0)
        return;

    // Clip strategy: "throttling" - find the floor for EMC high limit
    // (above domain minimum, of course)
    if ((*pEmcHighKHz) < MinKHz)
        *pEmcHighKHz = MinKHz;
    for (i = 0; i < NVRM_AP20_DFS_EMC_FREQ_STEPS; i++)
    {
        EmcKHz = s_Ap20EmcConfigSortedTable[i].Emc2xKHz >> 1;
        if (EmcKHz <= (*pEmcHighKHz))
            break;
    }
    if ((i == NVRM_AP20_DFS_EMC_FREQ_STEPS) || (EmcKHz < MinKHz))
    {
        i--;
        EmcKHz = s_Ap20EmcConfigSortedTable[i].Emc2xKHz >> 1;
    }
    *pEmcHighKHz = EmcKHz;

#if NVRM_LIMIT_CPU_EMC_RATIO
    // Clip strategy: "throttling" - restrict CPU high limit by EMC
    // configuration ((above domain minimum, of course)
    if ((*pCpuHighKHz) > s_Ap20EmcConfigSortedTable[i].CpuLimitKHz)
        (*pCpuHighKHz) = s_Ap20EmcConfigSortedTable[i].CpuLimitKHz;
    if ((*pCpuHighKHz) < NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Cpu))
        *pCpuHighKHz = NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Cpu);
#endif
#endif
}

static void
Ap20VdeClockStateUpdate(
    NvRmDeviceHandle hRmDevice)
{
    NvU32 reg;
    NvRmFreqKHz SourceClockFreq;
    NvRmModuleClockInfo* pCinfo = s_Ap20VdeConfig.pVdeInfo;
    NvRmModuleClockState* pCstate = s_Ap20VdeConfig.pVdeState;

    NV_ASSERT(pCinfo && pCstate);

    // Determine VDE source and divider setting; update VDE clock state
    reg = NV_REGR(hRmDevice,
                  NvRmPrivModuleID_ClockAndReset, 0, pCinfo->ClkSourceOffset);
    pCstate->Divider =
        ((reg >> pCinfo->DivisorFieldShift) & pCinfo->DivisorFieldMask);
    pCstate->SourceClock =
        ((reg >> pCinfo->SourceFieldShift) & pCinfo->SourceFieldMask);
    SourceClockFreq =
        NvRmPrivGetClockSourceFreq(pCinfo->Sources[pCstate->SourceClock]);

    // Fractional divider output = (Source Frequency * 2) / (divider + 2)
    pCstate->actual_freq = ((SourceClockFreq << 1) / (pCstate->Divider + 2));
}

static void Ap20VdeConfigInit(NvRmDeviceHandle hRmDevice)
{
    // Init VDE configuration shadow structure
    NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
        hRmDevice, NvRmModuleID_Vde,
        &s_Ap20VdeConfig.pVdeInfo, &s_Ap20VdeConfig.pVdeState));

     // Get VDE clock state from h/w
     Ap20VdeClockStateUpdate(hRmDevice);
}

static void
Ap20VdeClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 c, m, p;
    NvRmFreqKHz SourceKHz, ReachedKHzP, ReachedKHzC, ReachedKHzM, BestKHz;
    NV_ASSERT(DomainKHz <= MaxKHz);

    // VDE clock is disabled - can not change configuration at all,
    // and does not have any voltage requirements
    if (s_Ap20VdeConfig.pVdeState->refCount == 0)
    {
        pDfsSource->SourceId = NvRmClockSource_Invalid;
        pDfsSource->MinMv = NvRmVoltsOff;
        return;
    }

    // 1st try oscillator through VDE divider
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkM;
        pDfsSource->DividerSetting = NvRmPrivFindFreqMinAbove(
            s_Ap20VdeConfig.pVdeInfo->Divider, SourceKHz, MaxKHz, &DomainKHz);
        goto get_mv;
    }

    // 2nd option - PLLP0 through VDE divider selected unconditionally if
    // target is below half of PLLP0 output (divider granularity is "fair"
    ReachedKHzP = DomainKHz;
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0);
    p = NvRmPrivFindFreqMinAbove(
        s_Ap20VdeConfig.pVdeInfo->Divider, SourceKHz, MaxKHz, &ReachedKHzP);
    if (DomainKHz <= (SourceKHz >> 1))
    {
        pDfsSource->SourceId = NvRmClockSource_PllP0;
        pDfsSource->DividerSetting = p;
        DomainKHz = ReachedKHzP;
        goto get_mv;
    }

    /*
     * For high target frequencies add 3rd and 4th options - PLLC0, or PLLM0
     * through VDE divider, respectively. Select the option that provides
     * minimum output frequency equal or above the target, if all output
     * frequencies within domain maximum limit are below the target, select
     * the option with maximum output frequency.
     */
    ReachedKHzC = ReachedKHzM = DomainKHz;
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
    c = NvRmPrivFindFreqMinAbove(
        s_Ap20VdeConfig.pVdeInfo->Divider, SourceKHz, MaxKHz, &ReachedKHzC);
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    m = NvRmPrivFindFreqMinAbove(
        s_Ap20VdeConfig.pVdeInfo->Divider, SourceKHz, MaxKHz, &ReachedKHzM);

    BestKHz = NV_MAX(NV_MAX(ReachedKHzP, ReachedKHzC), ReachedKHzM);
    if ((DomainKHz <= ReachedKHzP) && (ReachedKHzP < BestKHz))
        BestKHz = ReachedKHzP;
    if ((DomainKHz <= ReachedKHzC) && (ReachedKHzC < BestKHz))
        BestKHz = ReachedKHzC;
    // PLLM0 may be selected as the last resort if two others are below target

    // Set souce clock parameters for selected option
    if (BestKHz == ReachedKHzP)
    {
        SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0);
        pDfsSource->SourceId = NvRmClockSource_PllP0;
        pDfsSource->DividerSetting = p;
        DomainKHz = ReachedKHzP;             // use PLLP0 as source
    }
    else if (BestKHz == ReachedKHzC)
    {
        SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
        pDfsSource->SourceId = NvRmClockSource_PllC0;
        pDfsSource->DividerSetting = c;
        DomainKHz = ReachedKHzC;             // use PLLC0 as source
    }
    else
    {
        SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
        pDfsSource->SourceId = NvRmClockSource_PllM0;
        pDfsSource->DividerSetting = m;
        DomainKHz = ReachedKHzM;            // use PLLM0 as source
    }

get_mv:
    // Finally get operational voltage for the found source/divider settings,
    // and store new domain frequency
    pDfsSource->MinMv =
        NvRmPrivModuleVscaleGetMV(hRmDevice, NvRmModuleID_Vde, DomainKHz);
    if (pDfsSource->MinMv < NvRmPrivSourceVscaleGetMV(hRmDevice, SourceKHz))
        pDfsSource->MinMv = NvRmPrivSourceVscaleGetMV(hRmDevice, SourceKHz);
    pDfsSource->SourceKHz = DomainKHz;
}

static void
Ap20VdeClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
// Shortcut - number of AP20 VDE sources (instead of checking descriptor)
#define AP20_VDE_SOURCES_NUMBER (4)

    NvU32 SourceIndex;
    NvRmModuleClockInfo* pCinfo = s_Ap20VdeConfig.pVdeInfo;
    NvRmModuleClockState* pCstate = s_Ap20VdeConfig.pVdeState;

    // Configuration can not be changed (VDE clock disabled) - exit
    if (pDfsSource->SourceId == NvRmClockSource_Invalid)
    {
        *pDomainKHz = pCstate->actual_freq;
        return;
    }

    // Convert Source ID into VDE source selector index
    for (SourceIndex = 0; SourceIndex < AP20_VDE_SOURCES_NUMBER; SourceIndex++)
    {
        NvRmClockSource SourceId = pCinfo->Sources[SourceIndex];
        if (SourceId == pDfsSource->SourceId)
            break;
    }
    NV_ASSERT(SourceIndex < AP20_VDE_SOURCES_NUMBER);

    // No changes in VDE clock configuration - exit
    if ((pCstate->SourceClock == SourceIndex) &&
        (pCstate->Divider == pDfsSource->DividerSetting))
    {
        *pDomainKHz = pCstate->actual_freq;
        return;
    }

    // Set new VDE clock state and update PLL, V-scale references
    NvRmPrivLockModuleClockState();
    pCstate->SourceClock = SourceIndex;
    pCstate->Divider = pDfsSource->DividerSetting;
    pCstate->actual_freq = pDfsSource->SourceKHz;
    NvRmPrivModuleClockSet(hRmDevice, pCinfo, pCstate);

    NvRmPrivModuleClockReAttach(hRmDevice, pCinfo, pCstate);
    NvRmPrivModuleVscaleReAttach(
        hRmDevice, pCinfo, pCstate, pCstate->actual_freq,
        NvRmPrivGetClockSourceFreq(pDfsSource->SourceId), NV_FALSE);
    NvRmPrivUnlockModuleClockState();

    *pDomainKHz = pCstate->actual_freq;
}

/*****************************************************************************/

static void
Ap20SystemClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 i, m, c;
    NvRmMilliVolts DivMv;
    NvRmFreqKHz SourceKHz, M1KHz, C1KHz, P2KHz;
    NV_ASSERT(DomainKHz <= MaxKHz);
    DivMv = pDfsSource->DividerSetting = 0; // no 2ndary divider by default

    // 1st try oscillator
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkM;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }
#if NVRM_AP20_USE_OSC_DOUBLER
    // 2nd choice - doubler
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkD);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkD;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }
#endif
    /*
     * 3rd option - PLLP divider per policy specification. Find
     * the policy entry with source frequency closest and above requested.
     * If requested frequency exceeds all policy options within domain
     * maximum limit, select the entry with the highest possible frequency.
     */
    for (i = 0; i < s_Ap20PllPSystemClockPolicyEntries; i++)
    {
        SourceKHz = s_Ap20PllPSystemClockPolicy[i].SourceKHz;
        if (SourceKHz > MaxKHz)
        {
            NV_ASSERT(i);
            i--;
            break;
        }
        if (DomainKHz <= SourceKHz)
        {
            break;
        }
    }
    if (i == s_Ap20PllPSystemClockPolicyEntries)
    {
        i--;    // last/highest source is the best we can do
    }
    SourceKHz = P2KHz = s_Ap20PllPSystemClockPolicy[i].SourceKHz;

    /*
     * 4th and 5th options - PLLM1 and PLLC1 secondary dividers. Use these
     * options only if target frequency is above half of PLLP0 output. Select
     * the option that provides minimum output frequency equal or above the
     * target, if all output frequencies within domain maximum are below the
     * target, select the maximum option.
     */
    if (DomainKHz > (NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0) >> 1))
    {
        C1KHz = M1KHz = DomainKHz;
        c = NvRmPrivFindFreqMinAbove(NvRmClockDivider_Fractional_2,
                NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0),
                MaxKHz, &C1KHz);
        m = NvRmPrivFindFreqMinAbove(NvRmClockDivider_Fractional_2,
                NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0),
                MaxKHz, &M1KHz);

        SourceKHz = NV_MAX(NV_MAX(C1KHz, M1KHz), P2KHz);
        if ((DomainKHz <= P2KHz) && (P2KHz < SourceKHz))
            SourceKHz = P2KHz;
        if ((DomainKHz <= C1KHz) && (C1KHz < SourceKHz))
            SourceKHz = C1KHz;
        // PLLM1 is selected as the last resort if two others are below target

        if (C1KHz == SourceKHz)
        {
            pDfsSource->SourceKHz = C1KHz;  // Selected PLLC 2ndary divider
            pDfsSource->SourceId = NvRmClockSource_PllC1;
            pDfsSource->DividerSetting = c;
            DivMv = NvRmPrivSourceVscaleGetMV(hRmDevice,
                NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0));
            goto get_mv;
        }
        if (M1KHz == SourceKHz)
        {
            pDfsSource->SourceKHz = M1KHz;  // Selected PLLM 2ndary divider
            pDfsSource->SourceId = NvRmClockSource_PllM1;
            pDfsSource->DividerSetting = m;
            DivMv = NvRmPrivSourceVscaleGetMV(hRmDevice,
                NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0));
            goto get_mv;
        }
    }
    // Selected PLLP 2ndary divider
    pDfsSource->SourceKHz = s_Ap20PllPSystemClockPolicy[i].SourceKHz;
    pDfsSource->SourceId = s_Ap20PllPSystemClockPolicy[i].SourceId;
    pDfsSource->DividerSetting = s_Ap20PllPSystemClockPolicy[i].DividerSetting;
    DivMv = NvRmPrivSourceVscaleGetMV(hRmDevice,
        NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0));

get_mv:
    // Finally get operational voltage for found source
    pDfsSource->MinMv = NvRmPrivModuleVscaleGetMV(
        hRmDevice, NvRmPrivModuleID_System, pDfsSource->SourceKHz);
    if (pDfsSource->MinMv < DivMv)
        pDfsSource->MinMv = DivMv;
}

static void
Ap20SystemBusClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
    NvRmClockSource SourceId = pDfsSource->SourceId;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore;

    switch(SourceId)
    {
        case NvRmClockSource_PllP2:
        case NvRmClockSource_PllC1:
        case NvRmClockSource_PllM1:
            // Reconfigure secondary divider if it is used as a source
            NvRmPrivDividerSet(hRmDevice,
                NvRmPrivGetClockSourceHandle(SourceId)->pInfo.pDivider,
                pDfsSource->DividerSetting);
            // fall through
        case NvRmClockSource_ClkD:
        case NvRmClockSource_ClkM:
            break;  // fixed sources - do nothing
        default:
            NV_ASSERT(!"Invalid source (per policy)");
    }
    NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
        hRmDevice, pCinfo, MaxKHz, pDomainKHz, &SourceId));
}

/*****************************************************************************/

// Fixed point calculation bits
#define FIXED_POINT_BITS (10)

static void Ap20CpuConfigInit(NvRmDeviceHandle hRmDevice)
{
    NvOdmPmuProperty PmuProperty;

    // Init PLLX frequency steps table based on chacterization data, so that
    // each entry corresponds to the v-scale level
    s_Ap20CpuConfig.pPllXStepsKHz = NvRmPrivModuleVscaleGetMaxKHzList(
        hRmDevice, NvRmModuleID_Cpu, &s_Ap20CpuConfig.PllXStepsNo);
    NV_ASSERT(s_Ap20CpuConfig.pPllXStepsKHz && s_Ap20CpuConfig.PllXStepsNo);
    NV_ASSERT(s_Ap20CpuConfig.pPllXStepsKHz[0] >= NVRM_PLLP_FIXED_FREQ_KHZ);

    // Init CPU power good delay and Core over CPU voltage dependency
    // parameters based on PMU property.
    if (!NvOdmQueryGetPmuProperty(&PmuProperty))
    {
        PmuProperty.AccuracyPercent = NVRM_DEFAULT_PMU_ACCURACY_PCT;
    }
    NV_ASSERT(PmuProperty.AccuracyPercent);
    NV_ASSERT(PmuProperty.AccuracyPercent < 5);  // 5% is a must for PMU

    s_Ap20CpuConfig.CoreOverCpuOffset = (NV_AP20_CORE_OVER_CPU_MV * 100) /
                                        (100 - PmuProperty.AccuracyPercent);
    s_Ap20CpuConfig.CoreOverCpuSlope =
        ((0x1 << FIXED_POINT_BITS) * (100 + PmuProperty.AccuracyPercent)) /
        (100 - PmuProperty.AccuracyPercent);
}

static void
Ap20CpuClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmDfsSource* pDfsSource,
    NvRmMilliVolts* pSystemMv)
{
    NvU32 i;
    NvRmMilliVolts DivMv = 0;
    NvRmMilliVolts CpuMv = 0;
    NvRmFreqKHz SourceKHz;

    NV_ASSERT(DomainKHz <= MaxKHz);
    NV_ASSERT(s_Ap20CpuConfig.pPllXStepsKHz);
    pDfsSource->DividerSetting = 0; // no 2ndary divider by default

    // 1st try oscillator
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkM;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }

    // 2nd choice - PLLP divider per policy specification
    SourceKHz =
        s_Ap20PllPCpuClockPolicy[s_Ap20PllPCpuClockPolicyEntries-1].SourceKHz;
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        // The requested frequency is within PLLP divider policy table, and all
        // policy entries are within domain maximum limit. Then, find the entry
        // with source frequency closest and above the requested.
        for (i = 0; i < s_Ap20PllPCpuClockPolicyEntries; i++)
        {
            SourceKHz = s_Ap20PllPCpuClockPolicy[i].SourceKHz;
            if (DomainKHz <= SourceKHz)
                break;
        }
        if (s_Ap20PllPCpuClockPolicy[i].DividerSetting == 0)
            pDfsSource->SourceId = NvRmClockSource_PllP0;   // Bypass 1:1 divider
        else
        {
            pDfsSource->SourceId = s_Ap20PllPCpuClockPolicy[i].SourceId;
            DivMv = NvRmPrivSourceVscaleGetMV(hRmDevice,
                NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0));
        }
        pDfsSource->SourceKHz = s_Ap20PllPCpuClockPolicy[i].SourceKHz;
        pDfsSource->DividerSetting = s_Ap20PllPCpuClockPolicy[i].DividerSetting;
        goto get_mv;
    }

    /*
     * 3rd and final choice - PLLX base output. Clip PllX policy entries to
     * domain maximum limit, and find the entry with source frequency closest
     * and above the requested. If not found, use the last entry with the
     * highest frequency. 
     */
    for (i = 0; i < s_Ap20CpuConfig.PllXStepsNo; i++)
    {
        SourceKHz = NV_MIN(s_Ap20CpuConfig.pPllXStepsKHz[i], MaxKHz);
        if (DomainKHz <= SourceKHz)
            break;
    }
    pDfsSource->SourceId = NvRmClockSource_PllX0;
    pDfsSource->SourceKHz = SourceKHz;

get_mv:
    // Finally get operational voltage for found source
    pDfsSource->MinMv = NvRmPrivModuleVscaleGetMV(
        hRmDevice, NvRmModuleID_Cpu, pDfsSource->SourceKHz);
#if !NV_OAL
    NvRmPrivGetLowVoltageThreshold(NvRmDfsVoltageRailId_Cpu, &CpuMv, NULL);
#endif
    CpuMv = NV_MAX(CpuMv, pDfsSource->MinMv);
    *pSystemMv = ((CpuMv * s_Ap20CpuConfig.CoreOverCpuSlope) >>
                  FIXED_POINT_BITS) + s_Ap20CpuConfig.CoreOverCpuOffset;
    *pSystemMv = NV_MAX(DivMv, (*pSystemMv));
}

static void
Ap20CpuBusClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
    NvRmFreqKHz SourceKHz = pDfsSource->SourceKHz;
    NvRmClockSource SourceId = pDfsSource->SourceId;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;

    switch(SourceId)
    {
        case NvRmClockSource_PllX0:
            // Reconfigure PLLX if it is used as a source
            NvRmPrivReConfigurePllX(hRmDevice, SourceKHz);
            break;
        case NvRmClockSource_PllP4:
            // Reconfigure PLLP variable divider if it is used as a source
            NvRmPrivDividerSet(hRmDevice,
                NvRmPrivGetClockSourceHandle(SourceId)->pInfo.pDivider,
                pDfsSource->DividerSetting);
            // fall through
        case NvRmClockSource_PllP0:
        case NvRmClockSource_ClkM:
            break;  // fixed sources - do nothing
        default:
            NV_ASSERT(!"Invalid source (per policy)");
    }
    NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
        hRmDevice, pCinfo, MaxKHz, pDomainKHz, &SourceId));
}

/*****************************************************************************/
/*****************************************************************************/

void
NvRmPrivAp20ScaledClockConfigInit(NvRmDeviceHandle hRmDevice)
{
    Ap20EmcConfigInit(hRmDevice);
    Ap20VdeConfigInit(hRmDevice);
    Ap20CpuConfigInit(hRmDevice);
}

NvBool NvRmPrivAp20DfsClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsFrequencies* pMaxKHz,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvBool Status;
    NvRmFreqKHz FreqKHz;
    NvRmMilliVolts SystemMv;
    NvRmDfsSource CpuClockSource, Emc2xClockSource;
    NvRmDfsSource SystemClockSource, VdeClockSource;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pMaxKHz && pDfsKHz);

    /*
     * Adjust System bus core clock. It should be sufficient to supply AVP,
     * and all bus clocks. Also make sure that AHB bus frequency is above
     * the one requested for APB clock.
     */
    pDfsKHz->Domains[NvRmDfsClockId_Ahb] = NV_MAX(
        pDfsKHz->Domains[NvRmDfsClockId_Ahb],
        pDfsKHz->Domains[NvRmDfsClockId_Apb]);
    FreqKHz = pDfsKHz->Domains[NvRmDfsClockId_System];
    FreqKHz = NV_MAX(FreqKHz, pDfsKHz->Domains[NvRmDfsClockId_Ahb]);
    FreqKHz = NV_MAX(FreqKHz, pDfsKHz->Domains[NvRmDfsClockId_Avp]);
    pDfsKHz->Domains[NvRmDfsClockId_System] = FreqKHz;

#if LIMIT_SYS_TO_AHB_APB_RATIOS
    if (pDfsKHz->Domains[NvRmDfsClockId_Apb] < (FreqKHz >> 2))
    {
        pDfsKHz->Domains[NvRmDfsClockId_Apb] = (FreqKHz >> 2);
    }
    if (pDfsKHz->Domains[NvRmDfsClockId_Ahb] < (FreqKHz >> 1))
    {
        pDfsKHz->Domains[NvRmDfsClockId_Ahb] = (FreqKHz >> 1);
    }
#endif

    // Find clock sources for CPU, System, VDE and Memory clocks.
    Ap20VdeClockSourceFind(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_Vpipe],
        pDfsKHz->Domains[NvRmDfsClockId_Vpipe],
        &VdeClockSource);
    Ap20SystemClockSourceFind(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_System],
        pDfsKHz->Domains[NvRmDfsClockId_System],
        &SystemClockSource);
    Status = Ap20Emc2xClockSourceFind(hRmDevice,
        (pMaxKHz->Domains[NvRmDfsClockId_Emc] << 1),
        (pDfsKHz->Domains[NvRmDfsClockId_Emc] << 1),
        &pDfsKHz->Domains[NvRmDfsClockId_Cpu], // Need for CPU/EMC ratio policy
        &Emc2xClockSource);
    Ap20CpuClockSourceFind(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_Cpu],
        pDfsKHz->Domains[NvRmDfsClockId_Cpu],
        &CpuClockSource, &SystemMv);
    // CPU and VDE clocks affect system core voltage as well
    SystemMv = NV_MAX(SystemMv, SystemClockSource.MinMv);
    SystemMv = NV_MAX(SystemMv, VdeClockSource.MinMv);

#if !NV_OAL
    // Adjust core and cpu voltage for the new clock sources before actual
    // change. Note that core voltage dependencies on CPU and VDE are already
    // included into system voltage requirement.
    NvRmPrivVoltageScale(NV_TRUE, CpuClockSource.MinMv,
                         SystemMv, Emc2xClockSource.MinMv);
#endif

    // Configure VDE, System bus and derived clocks (do not care about MIO on
    // AP20). Note that APB is the only clock in system complex that may have
    // different (lower) maximum limit - pass it explicitly to set function.
    Ap20SystemBusClockConfigure(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_System],
        &pDfsKHz->Domains[NvRmDfsClockId_System],
        &SystemClockSource);
    pDfsKHz->Domains[NvRmDfsClockId_Avp] = FreqKHz = // no AVP clock skipping
        pDfsKHz->Domains[NvRmDfsClockId_System];        
    NvRmPrivBusClockFreqSet(hRmDevice,
        pDfsKHz->Domains[NvRmDfsClockId_System],
        &FreqKHz,                                       // VDE decoupled
        &pDfsKHz->Domains[NvRmDfsClockId_Ahb],
        &pDfsKHz->Domains[NvRmDfsClockId_Apb],
        pMaxKHz->Domains[NvRmDfsClockId_Apb]);
    Ap20VdeClockConfigure(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_Vpipe],
        &pDfsKHz->Domains[NvRmDfsClockId_Vpipe],
        &VdeClockSource);

    // Configure Memory clocks and convert frequency to DFS EMC 1x domain
    FreqKHz = pDfsKHz->Domains[NvRmDfsClockId_Emc] << 1;
    Ap20Emc2xClockConfigure(hRmDevice,
        (pMaxKHz->Domains[NvRmDfsClockId_Emc] << 1),
        &FreqKHz, &Emc2xClockSource);
    pDfsKHz->Domains[NvRmDfsClockId_Emc] = FreqKHz >> 1;

    // Configure CPU core clock 
    Ap20CpuBusClockConfigure(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_Cpu],
        &pDfsKHz->Domains[NvRmDfsClockId_Cpu],
        &CpuClockSource);

#if !NV_OAL
    // Adjust core and cpu voltage after actual clock change.
    NvRmPrivVoltageScale(NV_FALSE, CpuClockSource.MinMv,
                         SystemMv, Emc2xClockSource.MinMv);
#endif
    return Status;
}

void
NvRmPrivAp20DfsClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvRmFreqKHz SystemFreq;
    const NvRmCoreClockInfo* pCinfo;
    NV_ASSERT(hRmDevice && pDfsKHz);

    // Get frequencies of the System core clock, AVP clock (the same as System
    // - no clock skipping), AHB, APB, and V-pipe bus clock. Note that on AP20
    // V-pipe is decoupled from the System bus, and has its own controls.
    pCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore;
    SystemFreq = NvRmPrivCoreClockFreqGet(hRmDevice, pCinfo);
    pDfsKHz->Domains[NvRmDfsClockId_System] = SystemFreq;
    pDfsKHz->Domains[NvRmDfsClockId_Avp] = SystemFreq;

    NvRmPrivBusClockFreqGet(
        hRmDevice, SystemFreq,
        &pDfsKHz->Domains[NvRmDfsClockId_Vpipe],
        &pDfsKHz->Domains[NvRmDfsClockId_Ahb],
        &pDfsKHz->Domains[NvRmDfsClockId_Apb]);
    Ap20VdeClockStateUpdate(hRmDevice);
    pDfsKHz->Domains[NvRmDfsClockId_Vpipe] =
        s_Ap20VdeConfig.pVdeState->actual_freq;

    // Get CPU core clock frequencies
    pCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] =
        NvRmPrivCoreClockFreqGet(hRmDevice, pCinfo);

    // Get EMC clock frequency (DFS monitors EMC 1x domain)
    Ap20Emc2xClockStateUpdate(hRmDevice);   // Get EMC2x clock state from h/w
    pDfsKHz->Domains[NvRmDfsClockId_Emc] =
        (s_Ap20EmcConfig.pEmc2xState->actual_freq >> 1);
}

void
NvRmPrivAp20DfsSuspendFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmMilliVolts TargetMv,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvU32 i;
    NvRmMilliVolts v;
    NvRmFreqKHz Fa, Fb, f;
    NvRmDfsSource DfsClockSource;
    NvRmFreqKHz CpuMaxKHz =
        NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
    NvRmFreqKHz SysMaxKHz =
        NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System)->MaxKHz;
    NV_ASSERT(hRmDevice && pDfsKHz);

    // Binary search for maximum System/Avp frequency, with source that
    // can be used at target voltage or below
    Fb = SysMaxKHz;
    Fa = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(Fa <= Fb);
    while ((Fb - Fa) > 1000)    // 1MHz resolution
    {
        f = (Fa + Fb) >> 1;
        Ap20SystemClockSourceFind(hRmDevice, SysMaxKHz, f, &DfsClockSource);
        v = DfsClockSource.MinMv;
        if (v <= TargetMv)
            Fa = f;
        else
            Fb = f;
    }
    pDfsKHz->Domains[NvRmDfsClockId_System] = Fa;
    pDfsKHz->Domains[NvRmDfsClockId_Avp] = Fa;
    pDfsKHz->Domains[NvRmDfsClockId_Ahb] = Fa;
    pDfsKHz->Domains[NvRmDfsClockId_Apb] = Fa;
    // On AP20 Vde clock has its own voltage scale; however, it is disabled
    // on suspend entry; hence, the setting below is "don't care"
    pDfsKHz->Domains[NvRmDfsClockId_Vpipe] = Fa;

    // If PLLM0 entry in EMC scaling table is valid, search the table for
    // the entry below and closest to the traget voltage. Otherwise, there
    // is no EMC scaling - just return current EMC frequency.
    pDfsKHz->Domains[NvRmDfsClockId_Emc] =
        (s_Ap20EmcConfig.pEmc2xState->actual_freq >> 1);
    if (s_Ap20EmcConfigSortedTable[0].Emc2xKHz != 0)
    {
        for (i = 0; i < (NVRM_AP20_DFS_EMC_FREQ_STEPS - 1); i++)
        {
            if ((s_Ap20EmcConfigSortedTable[i+1].Emc2xKHz == 0) ||
                (s_Ap20EmcConfigSortedTable[i].pOdmEmcConfig->EmcCoreVoltageMv
                 <= TargetMv))
                break;  // exit if found entry or next entry is invalid
        }
        pDfsKHz->Domains[NvRmDfsClockId_Emc] =
            (s_Ap20EmcConfigSortedTable[i].Emc2xKHz >> 1);
        f = s_Ap20EmcConfigSortedTable[i].CpuLimitKHz;
        CpuMaxKHz = NV_MIN(CpuMaxKHz, f);     // throttle CPU if necessary
    }

    // CPU voltage is turned Off in suspend. Use CPU frequency derived from
    // PLLP as LP1 resume start-up clock
    f = s_Ap20PllPCpuClockPolicy[s_Ap20PllPCpuClockPolicyEntries-1].SourceKHz;
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] = NV_MIN(CpuMaxKHz, f);

    NvOsDebugPrintf("LP1 entry/exit kHz: Cpu = %6d, Emc = %6d, Sys = %6d\n",
                    pDfsKHz->Domains[NvRmDfsClockId_Cpu],
                    pDfsKHz->Domains[NvRmDfsClockId_Emc],
                    pDfsKHz->Domains[NvRmDfsClockId_System]);
}

/*****************************************************************************/
/*****************************************************************************/

void
NvRmPrivAp20FastClockConfig(NvRmDeviceHandle hRmDevice)
{
#if !NV_OAL
    static NvBool resume = NV_FALSE;

    NvU32 divc1, divm1, divp2;
    NvRmFreqKHz SclkKHz, CpuKHz, PllP2KHz, PllM1KHz, PllC1KHz;
    NvRmDfsSource VdeSource;

    NvRmFreqKHz FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
        return; // fast clocks on SoC only

    // Set fastest EMC/MC configuration provided PLLM0 boot frequency matches
    // one of the pre-defined configurations, i.e, it is the first entry in the
    // sorted table. Preserve warm boot EMC configuration during resume.
    if ((s_Ap20EmcConfigSortedTable[0].Emc2xKHz == FreqKHz) &&
        (s_Ap20EmcConfig.Index != 0) && (!resume))
    {
        Ap20EmcSwitchToUndividedPllM0(hRmDevice, s_Ap20EmcConfigSortedTable);
        s_Ap20EmcConfig.Index = 0;
    }

    // Set AVP/System Bus clock to maximum during initialization (core voltage
    // is already nominal). During resume core voltage maybe below nominal,
    // hence, preserve system bus frequency set by the warm boot code, but
    // re-arrange source selection, and dividers settings per RM convention.
    if (!resume)
        SclkKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Avp)->MaxKHz;
    else
        SclkKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus);
    NV_ASSERT(SclkKHz);

    // First determine settings for PLLP/PLLM/PLLC secondary dividers to get
    // closest target approximation from below
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP0);
    PllP2KHz = SclkKHz;
    divp2 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, FreqKHz, PllP2KHz, &PllP2KHz);

    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    PllM1KHz = SclkKHz;
    divm1 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, FreqKHz, PllM1KHz, &PllM1KHz);

    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
    PllC1KHz = SclkKHz;
    divc1 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, FreqKHz, PllC1KHz, &PllC1KHz);

    // Now configure secondary dividers and select the output with highest
    // frequency // as a source for the system bus clock.
    SclkKHz = NV_MAX(PllC1KHz, NV_MAX(PllM1KHz, PllP2KHz)); 
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP2)->pInfo.pDivider,
        divp2);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllM1)->pInfo.pDivider,
        divm1);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllC1)->pInfo.pDivider,
        divc1);
    if (SclkKHz == PllP2KHz)
    {
        NvRmPrivCoreClockSet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore,
            NvRmClockSource_PllP2, 0, 0);
    }
    else if (SclkKHz == PllM1KHz)
    {
        NvRmPrivCoreClockSet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore,
            NvRmClockSource_PllM1, 0, 0);
    }
    else
    {
        NvRmPrivCoreClockSet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore,
            NvRmClockSource_PllC1, 0, 0);
    }
    NvRmPrivBusClockInit(hRmDevice, SclkKHz);

    // No need for VDE re-configurations during resume.
    if (!resume)
    {
        // Set VDE maximum clock during initialization (VDE is disabled
        // after basic reset - need to temporary enable it for configuration)
        FreqKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Vde)->MaxKHz;
        NvRmPowerModuleClockControl(hRmDevice, NvRmModuleID_Vde, 0, NV_TRUE);
        Ap20VdeClockSourceFind(hRmDevice, FreqKHz, FreqKHz, &VdeSource);
        Ap20VdeClockConfigure(hRmDevice, FreqKHz, &FreqKHz, &VdeSource);
        NvRmPowerModuleClockControl(hRmDevice, NvRmModuleID_Vde, 0, NV_FALSE);
    }

    // Set PLLX0 and CPU clock to SoC maximum (cpu voltage is guaranteed to
    // be nominal during initialization and resume as well)
    CpuKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllX0);
    if (CpuKHz != FreqKHz)
    {
        NvRmPrivReConfigurePllX(hRmDevice, CpuKHz);
    }
    NvRmPrivCoreClockSet(hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore,
        NvRmClockSource_PllX0, 0, 0);

    // Set PLLP4 fixed frequency to be used by external device(s)
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP4)->pInfo.pDivider,
        NVRM_AP20_FIXED_PLLP4_SETTING);

    resume = NV_TRUE;
#endif
}

/*****************************************************************************/

void
NvRmPrivAp20SdioTapDelayConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleId,
    NvU32 ClkSourceOffset,
    NvRmFreqKHz ConfiguredFreqKHz)
{
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );
    const NvOdmQuerySdioInterfaceProperty *pSdioInterfaceProps = NULL;
    NvU32 ClkSrcReg;
    
    if (Module != NvRmModuleID_Sdio)
        return;
    pSdioInterfaceProps = NvOdmQueryGetSdioInterfaceProperty(Instance);
    if (pSdioInterfaceProps == NULL)
        return;

    // Allow only less than 16 as tap delay.
    NV_ASSERT(pSdioInterfaceProps->TapDelay < 0x10);
    
    if (pSdioInterfaceProps->TapDelay > 0)
    {
        ClkSrcReg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, 
                        ClkSourceOffset);

        // CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1_0_SDMMC1_INT_FB_SEL_RANGE 
        ClkSrcReg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_SDMMC1,
                                SDMMC1_INT_FB_SEL, 1, ClkSrcReg);
        
        // CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1_0_SDMMC1_INT_FB_DLY_RANGE
        ClkSrcReg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_SDMMC1,
                            SDMMC1_INT_FB_DLY, pSdioInterfaceProps->TapDelay, ClkSrcReg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, 
                        ClkSourceOffset, ClkSrcReg);
    }
}

/*****************************************************************************/
