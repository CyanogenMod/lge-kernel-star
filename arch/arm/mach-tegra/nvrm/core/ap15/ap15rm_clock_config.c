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
#include "nvassert.h"
#include "nvrm_clocks.h"
#include "nvrm_hwintf.h"
#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "ap15/aremc.h"
#include "ap15/arclk_rst.h"
#include "ap15/arapb_misc.h"
#include "ap15rm_clocks.h"
#include "ap15rm_private.h"
#include "nvrm_pmu_private.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_memc.h"
#include "ap20/ap20rm_clocks.h"

// TODO: CAR and EMC access macros for time critical access

/*****************************************************************************/

static const NvU32 s_Ap15OscFreqKHz[] = { 13000, 19200, 12000, 26000 };

static void
Ap15PllPConfigure(NvRmDeviceHandle hRmDevice);

static void
Ap15MioReconfigure(NvRmDeviceHandle hRmDevice, NvRmFreqKHz MioKHz);

static void
Ap15AudioSyncInit(NvRmDeviceHandle hRmDevice, NvRmFreqKHz AudioSyncKHz);

static NvError
NvRmPrivOscDoublerConfigure(NvRmDeviceHandle hRmDevice, NvRmFreqKHz OscKHz)
{
    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            return NvRmPrivAp15OscDoublerConfigure(hRmDevice, OscKHz);
        case 0x20:
            return NvRmPrivAp20OscDoublerConfigure(hRmDevice, OscKHz);
        default:
            NV_ASSERT(!"Unsupported chip ID");
            return NvError_NotSupported;
    }
}

void
NvRmPrivClockSourceFreqInit(
    NvRmDeviceHandle hRmDevice,
    NvU32* pClockSourceFreq)
{
    NvU32 reg;
    const NvRmCoreClockInfo* pCore = NULL;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pClockSourceFreq);

    /*
     * Fixed clock sources: 32kHz, main oscillator and doubler
     * (OSC control should be already configured by the boot code)
     */
    pClockSourceFreq[NvRmClockSource_ClkS] = 32;

    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_OSC_CTRL_0);
    pClockSourceFreq[NvRmClockSource_ClkM] =
        s_Ap15OscFreqKHz[NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg)];

    if (NvSuccess == NvRmPrivOscDoublerConfigure(
        hRmDevice, pClockSourceFreq[NvRmClockSource_ClkM]))
    {
        pClockSourceFreq[NvRmClockSource_ClkD] =
            pClockSourceFreq[NvRmClockSource_ClkM] << 1;
    }
    else
        pClockSourceFreq[NvRmClockSource_ClkD] = 0;

    /*
     * PLLs and secondary PLL dividers
     */
    #define INIT_PLL_FREQ(PllId) \
    do\
    {\
        pClockSourceFreq[NvRmClockSource_##PllId] = NvRmPrivAp15PllFreqGet( \
         hRmDevice, NvRmPrivGetClockSourceHandle(NvRmClockSource_##PllId)->pInfo.pPll); \
    } while(0)

    // PLLX (check if present, keep boot settings
    // and just init frequency table)
    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_PllX0))
    {
        INIT_PLL_FREQ(PllX0);
    }
    // PLLC with output divider (if enabled keep boot settings and just init
    // frequency table, if disbled or bypassed - configure)
    INIT_PLL_FREQ(PllC0);
    if (pClockSourceFreq[NvRmClockSource_PllC0] <=
        pClockSourceFreq[NvRmClockSource_ClkM])
    {
        NvRmFreqKHz f = NVRM_PLLC_DEFAULT_FREQ_KHZ;
        NvRmPrivAp15PllConfigureSimple(hRmDevice, NvRmClockSource_PllC0, f, &f);
    }
    pClockSourceFreq[NvRmClockSource_PllC1] = NvRmPrivDividerFreqGet(hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllC1)->pInfo.pDivider);

    // PLLM with output divider (keep boot settings
    // and just init frequency)
    INIT_PLL_FREQ(PllM0);
    pClockSourceFreq[NvRmClockSource_PllM1] = NvRmPrivDividerFreqGet(hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllM1)->pInfo.pDivider);
#if !NV_OAL
    // PLLD and PLLU with no output dividers (keep boot settings
    // and just init frequency table)
    INIT_PLL_FREQ(PllD0);
    INIT_PLL_FREQ(PllU0);
#endif

    // PLLP and output dividers: set PLLP fixed frequency and enable dividers
    // with fixed settings in override mode, so they can be changed later, as
    // necessary. Switch system clock to oscillator during PLLP reconfiguration
    INIT_PLL_FREQ(PllP0);
    if (pClockSourceFreq[NvRmClockSource_PllP0] != NVRM_PLLP_FIXED_FREQ_KHZ)
    {
        pCore = NvRmPrivGetClockSourceHandle(
            NvRmClockSource_SystemBus)->pInfo.pCore;
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                      pCore->SelectorOffset);
        NvRmPrivCoreClockSet(hRmDevice, pCore, NvRmClockSource_ClkM, 0, 0);
        Ap15PllPConfigure(hRmDevice);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                pCore->SelectorOffset, reg);
    }
    NV_ASSERT(pClockSourceFreq[NvRmClockSource_PllP0] == NVRM_PLLP_FIXED_FREQ_KHZ);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP1)->pInfo.pDivider,
        NVRM_FIXED_PLLP1_SETTING);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP2)->pInfo.pDivider,
        NVRM_FIXED_PLLP2_SETTING);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP3)->pInfo.pDivider,
        NVRM_FIXED_PLLP3_SETTING);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP4)->pInfo.pDivider,
        NVRM_FIXED_PLLP4_SETTING);

    // PLLA and output divider must be init after PLLP1, used as a
    // reference (keep boot settings and just init frequency table)
    INIT_PLL_FREQ(PllA1);
    pClockSourceFreq[NvRmClockSource_PllA0] = NvRmPrivDividerFreqGet(hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllA0)->pInfo.pDivider);

    #undef INIT_PLL_FREQ

    /*
     * Core and bus clock sources
     * - Leave CPU bus as set by boot-loader
     * - Leave System bus as set by boot-loader, make sure all bus dividers are 1:1
     */
    pCore = NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
    pClockSourceFreq[NvRmClockSource_CpuBus] =
        NvRmPrivCoreClockFreqGet(hRmDevice, pCore);
    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge))
    {
        pClockSourceFreq[NvRmClockSource_CpuBridge] = NvRmPrivDividerFreqGet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBridge)->pInfo.pDivider);
    }
    pCore = NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore;
    pClockSourceFreq[NvRmClockSource_SystemBus] =
        NvRmPrivCoreClockFreqGet(hRmDevice, pCore);
    NvRmPrivBusClockInit(
        hRmDevice, pClockSourceFreq[NvRmClockSource_SystemBus]);

    /*
     * Initialize AudioSync clocks (PLLA will be re-configured if necessary)
     */
    Ap15AudioSyncInit(hRmDevice, NVRM_AUDIO_SYNC_KHZ);
}

void
NvRmPrivBusClockInit(NvRmDeviceHandle hRmDevice, NvRmFreqKHz SystemFreq)
{
    /*
     * Set all bus clock frequencies equal to the system clock frequency,
     * and clear AVP clock skipper i.e., set all bus clock dividers 1:1.
     * If APB clock is limited below system clock for a particular SoC,
     * set the APB divider to satisfy this limitation.
     */
    NvRmFreqKHz AhbFreq, ApbFreq;
    NvRmFreqKHz ApbMaxFreq = SystemFreq;
    if (hRmDevice->ChipId.Id == 0x20)
    {
        ApbMaxFreq = NVRM_AP20_APB_MAX_KHZ; // AP20 limitation
    }
    AhbFreq = SystemFreq;
    ApbFreq = NV_MIN(SystemFreq, ApbMaxFreq);

    NvRmPrivBusClockFreqSet(
        hRmDevice, SystemFreq, &SystemFreq, &AhbFreq, &ApbFreq, ApbMaxFreq);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_COP_CLK_SKIP_POLICY_0, 0x0);
}

/*****************************************************************************/
/*****************************************************************************/

static const NvRmFreqKHz s_PllLpCpconSelectionTable[] =
{
    NVRM_PLL_LP_CPCON_SELECT_STEPS_KHZ
};
static const NvU32 s_PllLpCpconSelectionTableSize =
NV_ARRAY_SIZE(s_PllLpCpconSelectionTable);

static const NvU32 s_PllMipiCpconSelectionTable[] =
{
    NVRM_PLL_MIPI_CPCON_SELECT_STEPS_N_DIVIDER
};
static const NvU32 s_PllMipiCpconSelectionTableSize =
NV_ARRAY_SIZE(s_PllMipiCpconSelectionTable);

static void
PllLpGetTypicalControls(
    NvRmFreqKHz InputKHz,
    NvU32 M,
    NvU32 N,
    NvU32* pCpcon)
{
    NvU32 i;
    if (N >= NVRM_PLL_LP_MIN_N_FOR_CPCON_SELECTION)
    {
        // CPCON depends on comparison frequency
        for (i = 0; i < s_PllLpCpconSelectionTableSize; i++)
        {
            if (InputKHz >= s_PllLpCpconSelectionTable[i] * M)
                break;
        }
        *pCpcon = i + 1;
    }
    else    // CPCON is 1, regardless of frequency
    {
        *pCpcon = 1;
    }
}

static void
PllMipiGetTypicalControls(
    NvU32 N,
    NvU32* pCpcon,
    NvU32* pLfCon)
{
    NvU32 i;

    // CPCON depends on feedback divider
    for (i = 0; i < s_PllMipiCpconSelectionTableSize; i++)
    {
        if (N <= s_PllMipiCpconSelectionTable[i])
            break;
    }
    *pCpcon = i + 1;
    *pLfCon = (N >= NVRM_PLL_MIPI_LFCON_SELECT_N_DIVIDER) ? 1 : 0;
}

void
NvRmPrivAp15PllSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmPllClockInfo* pCinfo,
    NvU32 M,
    NvU32 N,
    NvU32 P,
    NvU32 StableDelayUs,
    NvU32 cpcon,
    NvU32 lfcon,
    NvBool TypicalControls,
    NvU32 flags)
{
    NvU32 base, misc;
    NvU32 old_base, old_misc;
    NvU32 delay = 0;
    NvU32 override = 0;
    NvBool diff_clock = NV_FALSE;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);
    NV_ASSERT(pCinfo->PllBaseOffset);
    NV_ASSERT(pCinfo->PllMiscOffset);

    /*
     * PLL control fields used below have the same layout for all PLLs with
     * the following exceptions:
     *
     * a) PLLP base register OVERRIDE field has to be set in order to enable
     *  PLLP re-configuration in diagnostic mode. For other PLLs this field is
     *  "Don't care".
     * b) PLLU HS P divider field is one bit, inverse logic field. Other control
     *  bits, that are mapped to P divider in common layout should be set to 0.
     *
     * PLLP h/w field definitions will be used in DRF macros to construct base
     * values for all PLLs, with special care of a) and b). All base fields not
     * explicitly used below are set to 0 for all PLLs.
     *
     * c) PLLD/PLLU miscellaneous register has a unique fields determined based
     *  on the input flags. For other PLLs these fields have different meaning,
     *  and will be preserved.
     *
     *  PLLP h/w field definitions will be used in DRF macros to construct
     *  miscellaneous values with common layout. For unique fields PLLD h/w
     *  definitions will be used. All miscellaneous fields not explicitly used
     *  below are preserved for all PLLs.
     */
    base = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset);
    old_base = base;

    // Disable PLL if either input or feedback divider setting is zero
    if ((M == 0) || (N == 0))
    {
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, DISABLE, base);
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, DISABLE, base);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset, base);
        NvRmPrivPllFreqUpdate(hRmDevice, pCinfo);
        return;
    }

    // Determine type-specific controls, construct new misc settings
    misc = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllMiscOffset);
    old_misc = misc;
    if (pCinfo->PllType == NvRmPllType_MIPI)
    {
        if (flags & NvRmPllConfigFlags_SlowMode)
        {
            misc = NV_FLD_SET_DRF_NUM(  // "1" = slow (/8) MIPI clock output
                    CLK_RST_CONTROLLER, PLLD_MISC, PLLD_FO_MODE, 1, misc);
        }
        else if (flags & NvRmPllConfigFlags_FastMode)
        {
            misc = NV_FLD_SET_DRF_NUM(  // "0" = fast MIPI clock output
                    CLK_RST_CONTROLLER, PLLD_MISC, PLLD_FO_MODE, 0, misc);
        }
        if (flags & NvRmPllConfigFlags_DiffClkEnable)
        {
            misc = NV_FLD_SET_DRF_NUM(  // Enable differential clocks
                    CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, 1, misc);
        }
        else if (flags & NvRmPllConfigFlags_DiffClkDisable)
        {
            misc = NV_FLD_SET_DRF_NUM(  // Disable differential clocks
                    CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, 0, misc);
        }
        if (TypicalControls)
        {
            PllMipiGetTypicalControls(N, &cpcon, &lfcon);
        }
        delay = NVRM_PLL_MIPI_STABLE_DELAY_US;
    }
    else if (pCinfo->PllType == NvRmPllType_LP)
    {
        if (flags & NvRmPllConfigFlags_DccEnable)
        {
            misc = NV_FLD_SET_DRF_NUM(  // "1" = enable DCC
                    CLK_RST_CONTROLLER, PLLP_MISC, PLLP_DCCON, 1, misc);
        }
        else if (flags & NvRmPllConfigFlags_DccDisable)
        {
            misc = NV_FLD_SET_DRF_NUM(  // "0" = disable DCC
                    CLK_RST_CONTROLLER, PLLP_MISC, PLLP_DCCON, 0, misc);
        }
        if (TypicalControls)
        {
            NvRmFreqKHz InputKHz = NvRmPrivGetClockSourceFreq(pCinfo->InputId);
            PllLpGetTypicalControls(InputKHz, M, N, &cpcon);
        }
        lfcon = 0; // always for LP PLL
        delay = NVRM_PLL_LP_STABLE_DELAY_US;
    }
    else if (pCinfo->PllType == NvRmPllType_UHS)
    {
        if (TypicalControls)    // Same as MIPI typical controls
        {
            PllMipiGetTypicalControls(N, &cpcon, &lfcon);
        }
        delay = NVRM_PLL_MIPI_STABLE_DELAY_US;
        P = (P == 0) ? 1 : 0;   // P-divider is 1 bit, inverse logic
    }
    else
    {
        NV_ASSERT(!"Invalid PLL type");
    }
    misc = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_MISC, PLLP_CPCON, cpcon, misc);
    misc = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_MISC, PLLP_LFCON, lfcon, misc);

    // Construct new base setting
    // Override is PLLP specific, and it is just ignored by other PLLs;
    override = ((flags & NvRmPllConfigFlags_Override) != 0) ?
                CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BASE_OVRRIDE_ENABLE :
                CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BASE_OVRRIDE_DISABLE;
    {   // Compiler failed to generate correct code for the base fields
        // concatenation without the split below
        volatile NvU32 prebase =
            NV_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, DISABLE) |
            NV_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, ENABLE) |
            NV_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_REF_DIS, REF_ENABLE);
        base = prebase |
            NV_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BASE_OVRRIDE, override) |
            NV_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVP, P) |
            NV_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVN, N) |
            NV_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVM, M);
    }

    // If PLL is not bypassed, and new configurations is the same as the old
    // one - exit without overwriting h/w. Otherwise, bypass and disable PLL
    // outputs before changing configuration.
    if ((base == old_base) && (misc == old_misc))
    {
        NvRmPrivPllFreqUpdate(hRmDevice, pCinfo);
        return;
    }
    if (pCinfo->SourceId == NvRmClockSource_PllD0)
    {
        old_misc = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, 0, old_misc);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                pCinfo->PllMiscOffset, old_misc);
        if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, misc))
        {
            diff_clock = NV_TRUE;
            misc = NV_FLD_SET_DRF_NUM(
                CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, 0, misc);
        }
    }
    old_base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, ENABLE, old_base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllBaseOffset, old_base);
    old_base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, DISABLE, old_base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllBaseOffset, old_base);

    // Configure and enable PLL, keep it bypassed
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, ENABLE, base);
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, DISABLE, base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllBaseOffset, base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllMiscOffset, misc);
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, ENABLE, base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllBaseOffset, base);

    // Wait for PLL to stabilize and switch to PLL output
    NV_ASSERT(StableDelayUs);
    if (StableDelayUs > delay)
        StableDelayUs = delay;
    NvOsWaitUS(StableDelayUs);

    if (diff_clock)
    {
        misc = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, PLLD_MISC, PLLD_CLKENABLE, 1, misc);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllMiscOffset, misc);
    }
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, DISABLE, base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            pCinfo->PllBaseOffset, base);
    NvRmPrivPllFreqUpdate(hRmDevice, pCinfo);
}

NvRmFreqKHz
NvRmPrivAp15PllFreqGet(
    NvRmDeviceHandle hRmDevice,
    const NvRmPllClockInfo* pCinfo)
{
    NvU32 M, N, P;
    NvU32 base, misc;
    NvRmFreqKHz PllKHz;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo);
    NV_ASSERT(pCinfo->PllBaseOffset);
    NV_ASSERT(pCinfo->PllMiscOffset);

    /*
     * PLL control fields used below have the same layout for all PLLs with
     * the following exceptions:
     *
     * a) PLLP base register OVERRIDE field ("Don't care" for other PLLs).
     *  Respectively, PLLP h/w field definitions will be used in DRF macros
     *  to construct base values for all PLLs.
     *
     * b) PLLD/PLLU miscellaneous register fast/slow mode control (does not
     *  affect output frequency for other PLLs). Respectively, PLLD h/w field
     *  definitions will be used in DRF macros to construct miscellaneous values.
     */
    base = NV_REGR(
        hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset);
    PllKHz = NvRmPrivGetClockSourceFreq(pCinfo->InputId);
    NV_ASSERT(PllKHz);
    NV_ASSERT(NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_REF_DIS, base) ==
              CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_REF_DIS_REF_ENABLE);

    if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, base) ==
        CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BYPASS_DISABLE)
    {
        // Special cases: PLL is disabled, or in fixed mode (PLLP only)
        if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, base) ==
            CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_ENABLE_DISABLE)
            return 0;
        if ((pCinfo->SourceId == NvRmClockSource_PllP0) &&
            (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BASE_OVRRIDE, base) ==
             CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_BASE_OVRRIDE_DISABLE))
            return NV_BOOT_PLLP_FIXED_FREQ_KHZ;

        // PLL formula - Output F = (Reference F * N) / (M * 2^P)
        M = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVM, base);
        N = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVN, base);
        P = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_DIVP, base);
        NV_ASSERT((M != 0) && (N != 0));

        if (pCinfo->PllType == NvRmPllType_UHS)
        {
            // Adjust P divider field size and inverse logic for USB HS PLL
            P = (P & 0x1) ? 0 : 1;
        }
        PllKHz = ((PllKHz * N) / M) >> P;

        // Check slow/fast mode selection for MIPI PLLs
        if (pCinfo->PllType == NvRmPllType_MIPI)
        {
            misc = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                           pCinfo->PllMiscOffset);
            if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLD_MISC, PLLD_FO_MODE, misc) == 1)
            {
                PllKHz = PllKHz >> 3;   // In slow mode output is divided by 8
            }
        }
    }
    if (pCinfo->SourceId == NvRmClockSource_PllD0)
    {
        PllKHz = PllKHz >> 1;   // PLLD output always divided by 2
    }
    return PllKHz;
}

static void
Ap15PllControl(
    NvRmDeviceHandle hRmDevice,
    NvRmClockSource PllId,
    NvBool Enable)
{
    NvU32 base;
    NvU32 delay = 0;
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(PllId)->pInfo.pPll;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo->PllBaseOffset);

    /*
     * PLL control fields used below have the same layout for all PLLs.
     * PLLP h/w field definitions will be used in DRF macros to construct base
     * values for all PLLs.
     */
    base = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset);

    if (Enable)
    {
        // No need to enable already enabled PLL - do nothing
        if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, base) ==
            CLK_RST_CONTROLLER_PLLP_BASE_0_PLLP_ENABLE_ENABLE)
            return;

        // Get ready stabilization delay
        if ((pCinfo->PllType == NvRmPllType_MIPI) ||
            (pCinfo->PllType == NvRmPllType_UHS))
            delay = NVRM_PLL_MIPI_STABLE_DELAY_US;
        else if (pCinfo->PllType == NvRmPllType_LP)
            delay = NVRM_PLL_LP_STABLE_DELAY_US;
        else
            NV_ASSERT(!"Invalid PLL type");

        // Bypass PLL => Enable PLL => wait for PLL to stabilize
        // => switch to PLL output. All other settings preserved.
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, ENABLE, base);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset, base);
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, ENABLE, base);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset, base);

        NvOsWaitUS(delay);

        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, DISABLE, base);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset, base);
    }
    else
    {
        // Disable PLL, no bypass. All other settings preserved.
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, DISABLE, base);
        base = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, DISABLE, base);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, pCinfo->PllBaseOffset, base);
    }
    NvRmPrivPllFreqUpdate(hRmDevice, pCinfo);
}

void
NvRmPrivAp15PllConfigureSimple(
    NvRmDeviceHandle hRmDevice,
    NvRmClockSource PllId,
    NvRmFreqKHz MaxOutKHz,
    NvRmFreqKHz* pPllOutKHz)
{
#define NVRM_PLL_FCMP_1 (1000)
#define NVRM_PLL_VCO_RANGE_1 (1000000)
#define NVRM_PLL_FCMP_2 (2000)
#define NVRM_PLL_VCO_RANGE_2 (2000000)

    /*
     * Simple PLL configuration (assuming that target output frequency is
     * always in VCO range, and does not exceed 2GHz).
     * - output divider is set 1:1
     * - input divider is set to get comparison frequency equal or slightly
     *   above 1MHz if VCO is below 1GHz . Otherwise, input divider is set
     *   to get comparison frequency equal or slightly below 2MHz.
     * - feedback divider is calculated based on target output frequency
     * With simple configuration the absolute output frequency error does not
     * exceed half of comparison frequency. It has been verified that simple
     * configuration provides necessary accuracy for all display pixel clocks
     * use cases.
     */
    NvU32 M, N, P;
    NvRmFreqKHz RefKHz, VcoKHz;
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(PllId)->pInfo.pPll;
    NvU32 flags = 0;

    NV_ASSERT(hRmDevice);
    VcoKHz = *pPllOutKHz;
    P = 0;

    if (pCinfo->SourceId == NvRmClockSource_PllD0)
    {   // PLLD output is always divided by 2 (after P-divider)
        VcoKHz = VcoKHz << 1;
        MaxOutKHz = MaxOutKHz << 1;
        while (VcoKHz < pCinfo->PllVcoMin)
        {
            VcoKHz = VcoKHz << 1;
            MaxOutKHz = MaxOutKHz << 1;
            P++;
        }
        NV_ASSERT(P <= CLK_RST_CONTROLLER_PLLD_BASE_0_PLLD_DIVP_DEFAULT_MASK);
        flags = NvRmPllConfigFlags_DiffClkEnable;
    }
    if (pCinfo->SourceId == NvRmClockSource_PllX0)
    {
        flags = VcoKHz < NVRM_PLLX_DCC_VCO_MIN ?
            NvRmPllConfigFlags_DccDisable : NvRmPllConfigFlags_DccEnable;
    }
    NV_ASSERT((pCinfo->PllVcoMin <= VcoKHz) && (VcoKHz <= pCinfo->PllVcoMax));
    NV_ASSERT(VcoKHz <= NVRM_PLL_VCO_RANGE_2);
    NV_ASSERT(VcoKHz <= MaxOutKHz);

    RefKHz = NvRmPrivGetClockSourceFreq(pCinfo->InputId);
    NV_ASSERT(RefKHz);
    if (VcoKHz <= NVRM_PLL_VCO_RANGE_1)
        M = RefKHz / NVRM_PLL_FCMP_1;
    else
        M = (RefKHz + NVRM_PLL_FCMP_2 - 1) / NVRM_PLL_FCMP_2;
    N = (RefKHz + ((VcoKHz * M) << 1) ) / (RefKHz << 1);
    if ((RefKHz * N) > (MaxOutKHz * M))
        N--;    // use floor if rounding violates client's max limit

    NvRmPrivAp15PllSet(
        hRmDevice, pCinfo, M, N, P, (NvU32)-1, 0, 0, NV_TRUE, flags);
    *pPllOutKHz = NvRmPrivGetClockSourceFreq(pCinfo->SourceId);
}


// Fixed list of PLL HDMI configurations for different reference frequencies
// arranged according to CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_FIELD enum
static const NvRmPllFixedConfig s_Ap15HdmiPllD_Configurations[] =
{
    NVRM_PLLHD_AT_13MHZ,
    NVRM_PLLHD_AT_19MHZ,
    NVRM_PLLHD_AT_12MHZ,
    NVRM_PLLHD_AT_26MHZ
};

static const NvRmPllFixedConfig s_Ap15HdmiPllC_Configurations[] =
{
    NVRM_PLLHC_AT_13MHZ,
    NVRM_PLLHC_AT_19MHZ,
    NVRM_PLLHC_AT_12MHZ,
    NVRM_PLLHC_AT_26MHZ
};

#define NVRM_HDMI_CPCON (8)

void
NvRmPrivAp15PllConfigureHdmi(
    NvRmDeviceHandle hRmDevice,
    NvRmClockSource PllId,
    NvRmFreqKHz* pPllOutKHz)
{
    NvU32 reg;
    NvRmPllFixedConfig HdmiConfig = {0};
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(PllId)->pInfo.pPll;

    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_OSC_CTRL_0);

    if (PllId == NvRmClockSource_PllD0)
        HdmiConfig = s_Ap15HdmiPllD_Configurations[NV_DRF_VAL(
            CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg)];
    else if (PllId == NvRmClockSource_PllC0)
        HdmiConfig = s_Ap15HdmiPllC_Configurations[NV_DRF_VAL(
            CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg)];
    else
    {
        NV_ASSERT(!"Only PLLD or PLLC should be configured here");
        return;
    }
    NvRmPrivAp15PllSet(hRmDevice, pCinfo, HdmiConfig.M, HdmiConfig.N,
        HdmiConfig.P, (NvU32)-1, NVRM_HDMI_CPCON, 0, NV_FALSE, 0);
    *pPllOutKHz = NvRmPrivGetClockSourceFreq(pCinfo->SourceId);
}

/*****************************************************************************/

// Fixed list of PLLP configurations for different reference frequencies
// arranged according to CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_FIELD enum
static const NvRmPllFixedConfig s_Ap15PllPConfigurations[] =
{
    NVRM_PLLP_AT_13MHZ,
    NVRM_PLLP_AT_19MHZ,
    NVRM_PLLP_AT_12MHZ,
    NVRM_PLLP_AT_26MHZ
};

static void
Ap15PllPConfigure(NvRmDeviceHandle hRmDevice)
{
    NvU32 reg;
    NvRmFreqKHz PllKHz;
    NvRmPllFixedConfig PllPConfig = {0};

    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP0)->pInfo.pPll;
    NV_ASSERT(hRmDevice);

    // Configure and enable PllP at RM fixed frequency,
    // if it is not already enabled
    PllKHz = NvRmPrivGetClockSourceFreq(pCinfo->SourceId);
    if (PllKHz == NVRM_PLLP_FIXED_FREQ_KHZ)
        return;

    // Get fixed PLLP configuration for current oscillator  frequency.
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_OSC_CTRL_0);
    PllPConfig = s_Ap15PllPConfigurations[NV_DRF_VAL(
        CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg)];

    // Configure and enable PLLP
    NvRmPrivAp15PllSet(hRmDevice, pCinfo, PllPConfig.M, PllPConfig.N,
                       PllPConfig.P, (NvU32)-1, 0, 0, NV_TRUE,
                       NvRmPllConfigFlags_Override);
}

/*****************************************************************************/

// Fixed list of PLLU configurations for different reference frequencies
// arranged according to CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_FIELD enum
static const NvRmPllFixedConfig s_Ap15UsbPllConfigurations[] =
{
    NVRM_PLLU_AT_13MHZ,
    NVRM_PLLU_AT_19MHZ,
    NVRM_PLLU_AT_12MHZ,
    NVRM_PLLU_AT_26MHZ
};

static const NvRmPllFixedConfig s_Ap15UlpiPllConfigurations[] =
{
    NVRM_PLLU_ULPI_AT_13MHZ,
    NVRM_PLLU_ULPI_AT_19MHZ,
    NVRM_PLLU_ULPI_AT_12MHZ,
    NVRM_PLLU_ULPI_AT_26MHZ
};

static const NvRmPllFixedConfig s_Ap15UhsPllConfigurations[] =
{
    NVRM_PLLU_HS_AT_13MHZ,
    NVRM_PLLU_HS_AT_19MHZ,
    NVRM_PLLU_HS_AT_12MHZ,
    NVRM_PLLU_HS_AT_26MHZ
};

static void
PllUmipiConfigure(NvRmDeviceHandle hRmDevice, NvRmFreqKHz TargetFreq)
{
    NvU32 reg;
    NvRmPllFixedConfig UsbConfig = {0};
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllU0)->pInfo.pPll;
    NvRmFreqKHz CurrentFreq = NvRmPrivGetClockSourceFreq(pCinfo->SourceId);
    NV_ASSERT(hRmDevice);

    if (CurrentFreq == TargetFreq)
        return;     // PLLU is already configured at target frequency - exit

    // Index into fixed PLLU configuration tables based on oscillator frequency
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_OSC_CTRL_0);
    reg = NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg);

    if (TargetFreq == NvRmFreqUnspecified)
    {
        // By default set standard USB frequency, if PLLU is not configured
        if ((CurrentFreq == s_Ap15UsbPllConfigurations[reg].OutputKHz) ||
            (CurrentFreq == s_Ap15UlpiPllConfigurations[reg].OutputKHz))
        {
            return; // PLLU is already configured at supported frequency - exit
        }
        UsbConfig = s_Ap15UsbPllConfigurations[reg];
    }
    else if (TargetFreq == s_Ap15UsbPllConfigurations[reg].OutputKHz)
    {
        UsbConfig = s_Ap15UsbPllConfigurations[reg];
    }
    else if (TargetFreq == s_Ap15UlpiPllConfigurations[reg].OutputKHz)
    {
        UsbConfig = s_Ap15UlpiPllConfigurations[reg];
    }
    else
    {
        NV_ASSERT(!"Invalid target frequency");
        return;
    }
    // Configure and enable PLLU
    NvRmPrivAp15PllSet(hRmDevice, pCinfo, UsbConfig.M, UsbConfig.N,
                       UsbConfig.P, (NvU32)-1, 0, 0, NV_TRUE, 0);
}

static void
PllUhsConfigure(NvRmDeviceHandle hRmDevice, NvRmFreqKHz TargetFreq)
{
    NvU32 reg;
    NvRmPllFixedConfig UsbConfig = {0};
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllU0)->pInfo.pPll;
    NvRmFreqKHz CurrentFreq = NvRmPrivGetClockSourceFreq(pCinfo->SourceId);
    NV_ASSERT(hRmDevice);

    // Index into fixed PLLU configuration tables based on oscillator frequency
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  CLK_RST_CONTROLLER_OSC_CTRL_0);
    reg = NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, reg);

    // If PLLU is already configured - exit
    if (CurrentFreq == s_Ap15UhsPllConfigurations[reg].OutputKHz)
        return;

    /*
     * Target may be unspecified, or any of the standard USB, ULPI, or UHS
     * frequencies. In any case, main PLLU HS output is configured at UHS
     * frequency, with ULPI and USB frequencies are generated on secondary
     * outputs by fixed post dividers
     */
    if (!( (TargetFreq == NvRmFreqUnspecified) ||
           (TargetFreq == s_Ap15UsbPllConfigurations[reg].OutputKHz) ||
           (TargetFreq == s_Ap15UlpiPllConfigurations[reg].OutputKHz) ||
           (TargetFreq == s_Ap15UhsPllConfigurations[reg].OutputKHz) )
        )
    {
        NV_ASSERT(!"Invalid target frequency");
        return;
    }
    // Configure and enable PLLU
    UsbConfig = s_Ap15UhsPllConfigurations[reg];
    NvRmPrivAp15PllSet(hRmDevice, pCinfo, UsbConfig.M, UsbConfig.N,
                       UsbConfig.P, (NvU32)-1, 0, 0, NV_TRUE, 0);
}

static void
Ap15PllUConfigure(NvRmDeviceHandle hRmDevice, NvRmFreqKHz TargetFreq)
{
    const NvRmPllClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllU0)->pInfo.pPll;

    if (pCinfo->PllType == NvRmPllType_MIPI)
        PllUmipiConfigure(hRmDevice, TargetFreq);
    else if (pCinfo->PllType == NvRmPllType_UHS)
        PllUhsConfigure(hRmDevice, TargetFreq);
}

/*****************************************************************************/

// Fixed list of PLLA configurations for supported audio clocks
static const NvRmPllFixedConfig s_Ap15AudioPllConfigurations[] =
{
    NVRM_PLLA_CONFIGURATIONS
};

static void
Ap15PllAConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz* pAudioTargetKHz)
{
// The reminder bits used to check divisibility
#define REMINDER_BITS (6)

    NvU32 i, rem;
    NvRmFreqKHz OutputKHz;
    NvU32 BestRem = (0x1 << REMINDER_BITS);
    NvU32 BestIndex = NV_ARRAY_SIZE(s_Ap15AudioPllConfigurations) - 1;

    NvRmPllFixedConfig AudioConfig = {0};
    const NvRmPllClockInfo* pPllCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllA1)->pInfo.pPll;
    const NvRmDividerClockInfo* pDividerCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllA0)->pInfo.pDivider;
    NV_ASSERT(hRmDevice);
    NV_ASSERT(*pAudioTargetKHz);

    // Fixed PLLA FPGA configuration
    if (NvRmPrivGetExecPlatform(hRmDevice) == ExecPlatform_Fpga)
    {
        *pAudioTargetKHz = NvRmPrivGetClockSourceFreq(pDividerCinfo->SourceId);
        return;
    }
    // Find PLLA configuration with smallest output frequency that can be
    // divided by fractional divider into the closest one to the target.
    for (i = 0; i < NV_ARRAY_SIZE(s_Ap15AudioPllConfigurations); i++)
    {
        OutputKHz = s_Ap15AudioPllConfigurations[i].OutputKHz;
        if (*pAudioTargetKHz > OutputKHz)
            continue;
        rem = ((OutputKHz << (REMINDER_BITS + 1)) / (*pAudioTargetKHz)) &
              ((0x1 << REMINDER_BITS) - 1);
        if (rem < BestRem)
        {
            BestRem = rem;
            BestIndex = i;
            if (rem == 0)
                break;
        }
    }

    // Configure PLLA and output divider
    AudioConfig = s_Ap15AudioPllConfigurations[BestIndex];
    NvRmPrivAp15PllSet(hRmDevice, pPllCinfo, AudioConfig.M, AudioConfig.N,
                       AudioConfig.P, (NvU32)-1, 0, 0, NV_TRUE, 0);
    NvRmPrivDividerSet(
        hRmDevice, pDividerCinfo, AudioConfig.D);
    *pAudioTargetKHz = NvRmPrivGetClockSourceFreq(pDividerCinfo->SourceId);
}

static void
Ap15PllAControl(
    NvRmDeviceHandle hRmDevice,
    NvBool Enable)
{
    const NvRmDividerClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllA0)->pInfo.pDivider;
    if (NvRmPrivGetExecPlatform(hRmDevice) == ExecPlatform_Fpga)
        return; // No PLLA control on FPGA

    if (Enable)
    {
        Ap15PllControl(hRmDevice, NvRmClockSource_PllA1, NV_TRUE);
    }
    else
    {
        // Disable provided PLLA is not used as a source for any clock
        if (NvRmPrivGetDfsFlags(hRmDevice) & NvRmDfsStatusFlags_StopPllA0)
            Ap15PllControl(hRmDevice, NvRmClockSource_PllA1, NV_FALSE);
    }
    NvRmPrivDividerFreqUpdate(hRmDevice, pCinfo);
}

static void
Ap15AudioSyncInit(NvRmDeviceHandle hRmDevice, NvRmFreqKHz AudioSyncKHz)
{
    NvRmFreqKHz AudioTargetKHz;
    NvRmClockSource AudioSyncSource;
    const NvRmSelectorClockInfo* pCinfo;
    NV_ASSERT(hRmDevice);

    // Configure PLLA. Requested frequency must always exactly match one of the
    // fixed audio frequencies.
    AudioTargetKHz = AudioSyncKHz;
    Ap15PllAConfigure(hRmDevice, &AudioTargetKHz);
    NV_ASSERT(AudioTargetKHz == AudioSyncKHz);

    // Use PLLA as audio sync source, and disable doublers.
    // (verify if SoC supports audio sync selectors)
    AudioSyncSource = NvRmClockSource_PllA0;
    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_AudioSync))
    {
        pCinfo = NvRmPrivGetClockSourceHandle(
            NvRmClockSource_AudioSync)->pInfo.pSelector;
        NvRmPrivSelectorClockSet(hRmDevice, pCinfo, AudioSyncSource, NV_FALSE);
    }
    if (NvRmPrivGetClockSourceHandle(NvRmClockSource_MpeAudio))
    {
        pCinfo = NvRmPrivGetClockSourceHandle(
            NvRmClockSource_MpeAudio)->pInfo.pSelector;
        NvRmPrivSelectorClockSet(hRmDevice, pCinfo, AudioSyncSource, NV_FALSE);
    }
}

/*****************************************************************************/

static void
Ap15PllDControl(
    NvRmDeviceHandle hRmDevice,
    NvBool Enable)
{
    NvU32 reg;
    NvRmModuleClockInfo* pCinfo = NULL;
    NvRmModuleClockState* pCstate = NULL;
    NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
        hRmDevice, NvRmModuleID_Dsi, &pCinfo, &pCstate));

    if (Enable)
    {
        Ap15PllControl(hRmDevice, NvRmClockSource_PllD0, NV_TRUE);
        pCstate->actual_freq =
            NvRmPrivGetClockSourceFreq(NvRmClockSource_PllD0);
        return;
    }

    // Disable PLLD if it is not used by either display head or DSI
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                  pCinfo->ClkEnableOffset);
    if (NvRmPrivIsSourceSelectedByModule(hRmDevice, NvRmClockSource_PllD0,
                                   NVRM_MODULE_ID(NvRmModuleID_Display, 0)) ||
        NvRmPrivIsSourceSelectedByModule(hRmDevice, NvRmClockSource_PllD0,
                                   NVRM_MODULE_ID(NvRmModuleID_Display, 1)) ||
        ((reg & pCinfo->ClkEnableField) == pCinfo->ClkEnableField))
        return;

    Ap15PllControl(hRmDevice, NvRmClockSource_PllD0, NV_FALSE);
    pCstate->actual_freq =
        NvRmPrivGetClockSourceFreq(NvRmClockSource_PllD0);
}

static void
Ap15PllDConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz TargetFreq)
{
    NvRmFreqKHz MaxFreq = NvRmPrivGetSocClockLimits(NvRmModuleID_Dsi)->MaxKHz;
    NvRmModuleClockInfo* pCinfo = NULL;
    NvRmModuleClockState* pCstate = NULL;
    NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
        hRmDevice, NvRmModuleID_Dsi, &pCinfo, &pCstate));

    /*
     * PLLD is adjusted when DDK/ODM is initializing DSI or reconfiguring
     * display clock (for HDMI, DSI, or in some cases CRT).
     */
    if (NvRmIsFixedHdmiKHz(TargetFreq))
    {
        // 480p or 720p or 1080i/1080p HDMI - use fixed PLLD configuration
        NvRmPrivAp15PllConfigureHdmi(
            hRmDevice, NvRmClockSource_PllD0, &TargetFreq);
    }
    else
    {
        // for other targets use simple variable configuration
        NV_ASSERT(TargetFreq <= MaxFreq);
        NvRmPrivAp15PllConfigureSimple(
            hRmDevice, NvRmClockSource_PllD0, MaxFreq, &TargetFreq);
    }

    // Update DSI clock state (PLLD is a single source, no divider)
    pCstate->SourceClock = 0;
    pCstate->Divider = 1;
    pCstate->actual_freq =
        NvRmPrivGetClockSourceFreq(NvRmClockSource_PllD0);
    NvRmPrivModuleVscaleReAttach(hRmDevice,
        pCinfo, pCstate, pCstate->actual_freq, pCstate->actual_freq, NV_FALSE);
}

/*****************************************************************************/
/*****************************************************************************/

static void
Ap15DisplayClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleClockInfo *pCinfo,
    NvRmFreqKHz MinFreq,
    NvRmFreqKHz MaxFreq,
    NvRmFreqKHz TargetFreq,
    NvRmModuleClockState* pCstate,
    NvU32 flags)
{
    NvU32 i;
    NvRmClockSource SourceId;
    NvRmFreqKHz PixelFreq = TargetFreq;
    NvRmFreqKHz SourceClockFreq = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);

    // Clip target to maximum - we still may be able to configure frequency
    // within tolearnce range
    PixelFreq = TargetFreq = NV_MIN(TargetFreq, MaxFreq);

    /*
     * Display clock source selection policy:
     * - if MIPI flag is specified - use PLLD, and reconfigure it as necessary
     * - else if Oscillator output provides required accuracy - use Oscillator
     * - else if PLLP fixed output provides required accuracy - use fixed PLLP
     * - else if PPLC is used by other head - use PLLD, and reconfigure it as
     *   necessary
     * - else - use use PLLC, and reconfigure it as necessary
     */
    if (flags & NvRmClockConfig_MipiSync)
    {
        // PLLD requested - use it as a source, and reconfigure,
        //  unless it is also routed to the pads
        SourceId = NvRmClockSource_PllD0;
        if (!(flags & NvRmClockConfig_InternalClockForPads))
            Ap15PllDConfigure(hRmDevice, TargetFreq);
    }
    else if (NvRmIsFreqRangeReachable(
        SourceClockFreq, MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX))
    {
        // Target frequency is reachable from Oscillator - nothing to do
        SourceId = NvRmClockSource_ClkM;
    }
    else if (NvRmIsFreqRangeReachable(NVRM_PLLP_FIXED_FREQ_KHZ,
                MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX))
    {
        // Target frequency is reachable from PLLP0 - make sure it is enabled
        SourceId = NvRmClockSource_PllP0;
        Ap15PllPConfigure(hRmDevice);
    }
    else if (NvRmPrivIsSourceSelectedByModule(hRmDevice, NvRmClockSource_PllC0,
                NVRM_MODULE_ID(pCinfo->Module, (1 - pCinfo->Instance))))
    {
        // PLLC is used by the other head - only PLLD left
        SourceId = NvRmClockSource_PllD0;
        Ap15PllDConfigure(hRmDevice, TargetFreq);
    }
    else
    {
        // PLLC is available - use it
        SourceId = NvRmClockSource_PllC0;
        if (!NvRmIsFixedHdmiKHz(TargetFreq))    // don't touch HDMI targets
        {
            TargetFreq = NvRmPrivGetMaxFreqPllC(hRmDevice); // Target PLLC max
            if (!NvRmIsFreqRangeReachable(
                TargetFreq, MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX))
            {
                TargetFreq = MaxFreq;               // Target pixel range max
            }
        }
        NvRmPrivReConfigurePllC(hRmDevice, TargetFreq);
    }

    // Fill in clock state
    for (i = 0; i < NvRmClockSource_Num; i++)
    {
        if (pCinfo->Sources[i] == SourceId)
            break;
    }
    NV_ASSERT(i < NvRmClockSource_Num);
    pCstate->SourceClock = i;       // source index
    pCstate->Divider = 1;           // no divider (display driver has its own)
    pCstate->actual_freq = NvRmPrivGetClockSourceFreq(SourceId); // source KHz
    NV_ASSERT(NvRmIsFreqRangeReachable(
        pCstate->actual_freq, MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX));

    if (flags & NvRmClockConfig_SubConfig)
    {
        NvRmModuleClockInfo* pTvDacInfo = NULL;
        NvRmModuleClockState* pTvDacState = NULL;
        NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
            hRmDevice, NvRmModuleID_Tvo, &pTvDacInfo, &pTvDacState));

        // TVDAC is the 2nd TVO subclock (CVE is the 1st one)
        pTvDacInfo += 2;
        pTvDacState += 2;
        NV_ASSERT(pTvDacInfo->Module == NvRmModuleID_Tvo);
        NV_ASSERT(pTvDacInfo->SubClockId == 2);

        // enable the tvdac clock
        if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
            Ap15EnableTvDacClock(hRmDevice, ModuleClockState_Enable);
        else if (hRmDevice->ChipId.Id == 0x20)
            Ap20EnableTvDacClock(hRmDevice, ModuleClockState_Enable);

        // Set TVDAC = pixel clock (same source index and calculate divider
        // exactly as dc_hal.c does)
        pTvDacState->SourceClock = i;
        pTvDacState->Divider =
            (((pCstate->actual_freq * 2 ) + PixelFreq / 2) / PixelFreq) - 2;
        pTvDacState->actual_freq =
            (pCstate->actual_freq * 2 ) / (pTvDacState->Divider + 2);
        NvRmPrivModuleClockSet(hRmDevice, pTvDacInfo, pTvDacState);
    }
    if (flags & NvRmClockConfig_DisableTvDAC)
    {
        // disable the tvdac clock
        if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
            Ap15EnableTvDacClock(hRmDevice, ModuleClockState_Disable);
        else if (hRmDevice->ChipId.Id == 0x20)
            Ap20EnableTvDacClock(hRmDevice, ModuleClockState_Disable);
    }
}

NvBool
NvRmPrivAp15IsModuleClockException(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleClockInfo *pCinfo,
    NvU32 ClockSourceCount,
    NvRmFreqKHz MinFreq,
    NvRmFreqKHz MaxFreq,
    const NvRmFreqKHz* PrefFreqList,
    NvU32 PrefCount,
    NvRmModuleClockState* pCstate,
    NvU32 flags)
{
    NvU32 i;
    NvRmFreqKHz FreqKHz;
    NvRmClockSource SourceId;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCinfo && PrefFreqList && pCstate);

    switch (pCinfo->Module)
    {
        case NvRmModuleID_Display:
            /*
             * Special handling for display clocks. Must satisfy requirements
             * for the 1st requested frequency and complete configuration.
             * Note that AP15 display divider is within module itself, so the
             * input request is for pisxel clock, but output *pCstate specifies
             * source frequency. Display driver will configure divider.
             */
            Ap15DisplayClockConfigure(hRmDevice, pCinfo,
                MinFreq, MaxFreq, PrefFreqList[0], pCstate, flags);
            return NV_TRUE;

        case NvRmModuleID_Dsi:
            /*
             * Reconfigure PLLD to match requested frequency, and update DSI
             * clock state.
             */
            Ap15PllDConfigure(hRmDevice, PrefFreqList[0]);
            NV_ASSERT((MinFreq <= pCstate->actual_freq) &&
                      (pCstate->actual_freq <= MaxFreq));
            return NV_TRUE;

        case NvRmModuleID_Hdmi:
            /*
             * Complete HDMI configuration; choose among possible sources:
             * Osc, PLLP, PLLD, PLLC in the same order as for display (PLLD
             * or PLLC should be already configured properly for display)
             */
            if (flags & NvRmClockConfig_MipiSync)
                SourceId = NvRmClockSource_PllD0;
            else if (NvRmIsFreqRangeReachable(
                         NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM),
                         MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX))
                SourceId = NvRmClockSource_ClkM;
            else if (NvRmIsFreqRangeReachable(NVRM_PLLP_FIXED_FREQ_KHZ,
                         MinFreq, MaxFreq, NVRM_DISPLAY_DIVIDER_MAX))
                SourceId = NvRmClockSource_PllP0;
            else
                SourceId = NvRmClockSource_PllC0;

            // HDMI clock state with selected source
            for (i = 0; i < NvRmClockSource_Num; i++)
            {
                if (pCinfo->Sources[i] == SourceId)
                    break;
            }
            NV_ASSERT(i < NvRmClockSource_Num);
            pCstate->SourceClock = i;       // source index
            FreqKHz = NvRmPrivGetClockSourceFreq(SourceId);
            pCstate->Divider = ((FreqKHz << 2) + PrefFreqList[0]) /
                               (PrefFreqList[0] << 1) - 2;
            pCstate->actual_freq = (FreqKHz << 1) / (pCstate->Divider + 2);
            NV_ASSERT(pCstate->Divider <= pCinfo->DivisorFieldMask);
            NV_ASSERT((MinFreq <= pCstate->actual_freq) &&
                      (pCstate->actual_freq <= MaxFreq));
            return NV_TRUE;

        case NvRmModuleID_Spdif:
            if (flags & NvRmClockConfig_SubConfig)
                return NV_FALSE; // Nothing special for SPDIFIN
            // fall through for SPDIFOUT
        case NvRmModuleID_I2s:
            /*
             * If requested, reconfigure PLLA to match target frequency, and
             * complete clock configuration with PLLA as a source. Otherwise,
             * make sure PLLA is enabled (at current configuration), and
             * continue regular configuration for SPDIFOUT and I2S.
             */
            if (flags & NvRmClockConfig_AudioAdjust)
            {
                FreqKHz = PrefFreqList[0];
                Ap15PllAConfigure(hRmDevice, &FreqKHz);

                pCstate->SourceClock = 0;   // PLLA source index
                pCstate->Divider = ((FreqKHz << 2) + PrefFreqList[0]) /
                                    (PrefFreqList[0] << 1) - 2;
                pCstate->actual_freq = (FreqKHz << 1) / (pCstate->Divider + 2);
                if (NvRmPrivGetExecPlatform(hRmDevice) == ExecPlatform_Fpga)
                {   // Fake return on FPGA (PLLA is not configurable, anyway)
                    pCstate->actual_freq = PrefFreqList[0];
                }
                NV_ASSERT(pCinfo->Sources[pCstate->SourceClock] ==
                          NvRmClockSource_PllA0);
                NV_ASSERT(pCstate->Divider <= pCinfo->DivisorFieldMask);
                NV_ASSERT((MinFreq <= pCstate->actual_freq) &&
                          (pCstate->actual_freq <= MaxFreq));
                return NV_TRUE;
            }
            Ap15PllAControl(hRmDevice, NV_TRUE);
            return NV_FALSE;

        case NvRmModuleID_Usb2Otg:
            /*
             * Reconfigure PLLU to match requested frequency, and complete USB
             * clock configuration (PLLU is a single source, no divider)
             */
            Ap15PllUConfigure(hRmDevice, PrefFreqList[0]);
            pCstate->SourceClock = 0;
            pCstate->Divider = 1;
            pCstate->actual_freq =
                NvRmPrivGetClockSourceFreq(pCinfo->Sources[0]);
            return NV_TRUE;

        default:
            // No exception for other modules - continue regular configuration
            return NV_FALSE;
    }
}

/*****************************************************************************/

void
NvRmPrivAp15DisablePLLs(
    NvRmDeviceHandle hRmDevice,
    const NvRmModuleClockInfo* pCinfo,
    const NvRmModuleClockState* pCstate)
{
#if !NV_OAL
    switch (pCinfo->Module)
    {
        case NvRmModuleID_Display:
            NvRmPrivBoostPllC(hRmDevice);
            Ap15PllDControl(hRmDevice, NV_FALSE);
            break;

        case NvRmModuleID_Spdif:
        case NvRmModuleID_I2s:
            Ap15PllAControl(hRmDevice, NV_FALSE);
            break;

        default:
            break;
    }
#endif
}

void
NvRmPrivAp15PllDPowerControl(
    NvRmDeviceHandle hRmDevice,
    NvBool ConfigEntry,
    NvBool* pMipiPllVddOn)
{
#if !NV_OAL
    if (ConfigEntry)
    {
        // On entry to display clock configuration get PLLD power ready
        if (!(*pMipiPllVddOn))
        {
            NvRmPrivPmuRailControl(hRmDevice, NV_VDD_PLLD_ODM_ID, NV_TRUE);
            *pMipiPllVddOn = NV_TRUE;
        }
    }
    else
    {
        // On exit from display clock configuration turn off PLLD power
        // if it is disabled
        if ((*pMipiPllVddOn) &&
            (NvRmPrivGetClockSourceFreq(NvRmClockSource_PllD0) == 0))
        {
            NvRmPrivPmuRailControl(hRmDevice, NV_VDD_PLLD_ODM_ID, NV_FALSE);
            *pMipiPllVddOn = NV_FALSE;
        }
    }
#endif
}

void
NvRmPrivConfigureClockSource(
        NvRmDeviceHandle hRmDevice,
        NvRmModuleID ModuleId,
        NvBool enable)
{
    // Extract module and instance from composite module id.
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );

    switch (Module)
    {
        case NvRmModuleID_Usb2Otg:
            // Do not disable the PLLU clock once it is enabled
            // Set PLLU default configuration if it is not already configured
            if (enable)
                Ap15PllUConfigure(hRmDevice, NvRmFreqUnspecified);
            break;
#if !NV_OAL
        case NvRmModuleID_Spdif:
        case NvRmModuleID_I2s:
            if (enable)
            {
                // Do not enable if PLLA is not used as a source for any clock
                if (NvRmPrivGetDfsFlags(hRmDevice) & NvRmDfsStatusFlags_StopPllA0)
                    break;
            }
            // fall through
        case NvRmModuleID_Mpe:
            Ap15PllAControl(hRmDevice, enable);
            break;

        case NvRmModuleID_Dsi:
            Ap15PllDControl(hRmDevice, enable);
            break;

        case NvRmPrivModuleID_Pcie:
            NvRmPrivAp20PllEControl(hRmDevice, enable);
            break;
#endif
        default:
            break;
    }
    return;
}

/*****************************************************************************/
/*****************************************************************************/

/*
 * Basic DFS clock control policy outline:
 * - Oscillator ClkM, doubler ClkD, and memory PLLM0 - always available, fixed
 *   frequency sources.
 * - Peripheral PLLP0 may be dynamically enabled / disabled when DFS is stopped
 *   and CPU is power gated. Hence, when DFS is running it is always enabled
 *   and configured at fixed PLLP0 frequency.
 * - Cpu PLLC0 may be dynamically enabled / disabled when DFS is stopped and
 *   CPU is power gated. Hence, when DFS is running it is always enabled. PLLC0
 *   is commonly configured at maximum CPU domain frequency. If necessary, it
 *   may be adjusted to provide required display pixel clock frequency.
 * - System buses, and MC/EMC configuration, clock source multiplexes and
 *   dividers, as well as PLLP2, PLLP4 and PLLM1 dividers are under exclusive
 *   DFS control, and are not accessed by any other code except bootloader
 *   before RM is open.
 */

// Limit frequencies ratio for Vpipe : System >= 1 : 2^(value - 1)
#define LIMIT_SYS_TO_VDE_RATIO (2)

// Limit frequencies ratio for AHB : System >= 1:2 and APB : System >= 1 : 4
#define LIMIT_SYS_TO_AHB_APB_RATIOS (1)

// PLLP2 must be used as a variable source for System clock.
#define PLLP_POLICY_ENTRY(KHz) \
 { NvRmClockSource_PllP2,\
   (NVRM_PLLP_FIXED_FREQ_KHZ * 2)/((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz),\
   ((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz - 2)\
 },
static const NvRmDfsSource s_Ap15PllPSystemClockPolicy[] =
{
    NVRM_AP15_PLLP_POLICY_SYSTEM_CLOCK
};
static const NvU32 s_Ap15PllPSystemClockPolicyEntries =
    NV_ARRAY_SIZE(s_Ap15PllPSystemClockPolicy);
#undef PLLP_POLICY_ENTRY


// PLLP4 must be used as a variable source for cpu clock.
#define PLLP_POLICY_ENTRY(KHz) \
 { NvRmClockSource_PllP4,\
   (NVRM_PLLP_FIXED_FREQ_KHZ * 2)/((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz),\
   ((NVRM_PLLP_FIXED_FREQ_KHZ * 2)/KHz - 2)\
 },
static const NvRmDfsSource s_Ap15PllPCpuClockPolicy[] =
{
    NVRM_AP15_PLLP_POLICY_CPU_CLOCK
};
static const NvU32 s_Ap15PllPCpuClockPolicyEntries =
    NV_ARRAY_SIZE(s_Ap15PllPCpuClockPolicy);
#undef PLLP_POLICY_ENTRY

/*
 * Sorted list of timing parameters for discrete set of EMC frequencies used
 * by DFS: entry 0 specifies timing parameters for PLLM0 output frequency,
 * entry n (n = 1, 2, ... number of EMC steps-1) specifies timing parameters
 * for EMC frequency = PLLM0 frequency / (2 * n); thus only frequencies evenly
 * divided down from PLLM0 will be used by DFS
 */
static NvRmAp15EmcTimingConfig
s_Ap15EmcConfigSortedTable[NVRM_AP15_DFS_EMC_FREQ_STEPS];

static struct MemClocksRec
{
    // Index of selected EMC configuration entry
    NvU32 Index;

    // Pointers to EMC and MC clock descriptors
    NvRmModuleClockInfo* pEmcInfo;
    NvRmModuleClockInfo* pMcInfo;

    // Pointers to EMC and MC clock state records
    NvRmModuleClockState* pEmcState;
    NvRmModuleClockState* pMcState;

} s_MemClocks = {0};

static const NvU32 s_Cpu2EmcRatioPolicyTable[] =
{
    NVRM_AP15_CPU_EMC_RATIO_POLICY
};

/*****************************************************************************/

static void
Ap15Emc2xFreqGet(
    NvRmDeviceHandle hRmDevice)
{
    NvU32 reg;
    NvRmFreqKHz SourceClockFreq;
    NvRmModuleClockInfo* pCinfo = s_MemClocks.pEmcInfo;
    NvRmModuleClockState* pCstate = s_MemClocks.pEmcState;

    NV_ASSERT(pCinfo && pCstate);

    // Determine EMC2x source and divider setting; update EMC2x clock state
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

// Enable/Disable EMC low-latency return-fifo reservation scheme
// (enable requires confirmation polling)
#define NVRM_AP15_EMCLL_RETRSV_ENABLE \
do\
{\
    NvU32 reg; \
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, \
                  0, EMC_LL_ARB_CONFIG_0); \
    reg = NV_FLD_SET_DRF_DEF( \
        EMC, LL_ARB_CONFIG, LL_RETRSV_ENABLE, ENABLED, reg); \
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, \
            0, EMC_LL_ARB_CONFIG_0, reg); \
    while (reg != NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController,\
                          0, EMC_LL_ARB_CONFIG_0)) \
        ; \
} while(0)

#define NVRM_AP15_EMCLL_RETRSV_DISABLE \
do\
{\
    NvU32 reg; \
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, \
                  0, EMC_LL_ARB_CONFIG_0); \
    reg = NV_FLD_SET_DRF_DEF( \
        EMC, LL_ARB_CONFIG, LL_RETRSV_ENABLE, DISABLED, reg); \
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, \
            0, EMC_LL_ARB_CONFIG_0, reg); \
} while (0)

void
NvRmPrivAp15SetEmcForCpuSrcSwitch(NvRmDeviceHandle hRmDevice)
{
    NVRM_AP15_EMCLL_RETRSV_ENABLE;
}

void
NvRmPrivAp15SetEmcForCpuDivSwitch(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz CpuFreq,
    NvBool Before)
{
    NvRmFreqKHz EmcFreq = (s_MemClocks.pEmcState->actual_freq >> 1);
    if (Before && (CpuFreq < EmcFreq))
    {
         NVRM_AP15_EMCLL_RETRSV_ENABLE;
    }
    else if (!Before && (CpuFreq >= EmcFreq))
    {
        NVRM_AP15_EMCLL_RETRSV_DISABLE;
    }
}

static void
Ap15EmcTimingSet(
    NvRmDeviceHandle hRmDevice,
    NvBool FreqRising,
    NvBool BeforeDividerChange,
    const NvRmAp15EmcTimingConfig* pEmcConfig)
{
    // Write shadow timing registers
    if (FreqRising == BeforeDividerChange)  // "overlap down" parameters
    {
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING0_0, pEmcConfig->Timing0Reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING1_0, pEmcConfig->Timing1Reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING2_0, pEmcConfig->Timing2Reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING3_0, pEmcConfig->Timing3Reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING5_0, pEmcConfig->Timing5Reg);
    }
    else                                    // "overlap up" parameters
    {
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING4_0, pEmcConfig->Timing4Reg);

        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_FBIO_CFG6_0, pEmcConfig->FbioCfg6Reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_FBIO_DQSIB_DLY_0, pEmcConfig->FbioDqsibDly);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_FBIO_QUSE_DLY_0, pEmcConfig->FbioQuseDly);
    }
    // Trigger active register update from shadow
    NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
            EMC_TIMING_CONTROL_0, 0x1);

    // Make sure update from shadow is completed
    if (FreqRising == BeforeDividerChange)
    {
        while((NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                       EMC_TIMING0_0)) != pEmcConfig->Timing0Reg);
    }
    else
    {
        while((NV_REGR(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                       EMC_TIMING4_0)) != pEmcConfig->Timing4Reg);
        // Re-trigger active register update (need it for trimmers only)
        NV_REGW(hRmDevice, NvRmPrivModuleID_ExternalMemoryController, 0,
                EMC_TIMING_CONTROL_0, 0x1);
    }
}

static void
Ap15Emc2xClockSet(
    NvRmDeviceHandle hRmDevice,
    NvBool FreqRising,
    const NvRmAp15EmcTimingConfig* pEmcConfig)
{
    NvU32 reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                        CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC,
                         EMC_2X_CLK_DIVISOR, pEmcConfig->Emc2xDivisor, reg);
    NV_ASSERT(pEmcConfig->Emc2xKHz);    // validate table entry

    // Update EMC state
    s_MemClocks.pEmcState->actual_freq = pEmcConfig->Emc2xKHz;
    s_MemClocks.pEmcState->Divider = pEmcConfig->Emc2xDivisor;

    // Set EMC parameters and EMC divisor (the EMC clock source is always
    // PLLM0 starting from BL)
    Ap15EmcTimingSet(hRmDevice, FreqRising, NV_TRUE, pEmcConfig);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, reg);
    Ap15EmcTimingSet(hRmDevice, FreqRising, NV_FALSE, pEmcConfig);
}

static void
Ap15McClockSet(
    NvRmDeviceHandle hRmDevice,
    const NvRmAp15EmcTimingConfig* pEmcConfig)
{
    NvU32 src, div;
    NvU32 reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                        CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0);
    src = NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_MEM, MEM_CLK_SRC, reg);
    div = NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_MEM, MEM_CLK_DIVISOR, reg);

    // Update MC state
    s_MemClocks.pMcState->actual_freq = pEmcConfig->McKHz;
    s_MemClocks.pMcState->SourceClock = pEmcConfig->McClockSource;
    s_MemClocks.pMcState->Divider = pEmcConfig->McDivisor;

    // Set MC divisor before source, if new value is bigger than the old one
    if (pEmcConfig->McDivisor > div)
    {
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_MEM,
            MEM_CLK_DIVISOR, pEmcConfig->McDivisor, reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }

    // Modify MC source if it is to be changed
    if (pEmcConfig->McClockSource != src)
    {
        NvRmPrivMemoryClockReAttach(
            hRmDevice, s_MemClocks.pMcInfo, s_MemClocks.pMcState);
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_MEM,
            MEM_CLK_SRC, pEmcConfig->McClockSource, reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }

    // Set MC divisor after source, if new value is smaller than the old one
    if (pEmcConfig->McDivisor < div)
    {
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_MEM,
            MEM_CLK_DIVISOR, pEmcConfig->McDivisor, reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
}

void
NvRmPrivAp15EmcConfigInit(NvRmDeviceHandle hRmDevice)
{
    NvU32 i, j, k, reg=0;
    NvU32 ConfigurationsCount;
    NvRmFreqKHz Emc2xKHz, McKHz, McMax;
    NvRmFreqKHz PllM0KHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    const NvOdmSdramControllerConfig* pEmcConfigurations =
        NvOdmQuerySdramControllerConfigGet(&ConfigurationsCount, &reg);

    // Init memory configuration structure
    NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
        hRmDevice, NvRmPrivModuleID_ExternalMemoryController,
        &s_MemClocks.pEmcInfo, &s_MemClocks.pEmcState));
     NV_ASSERT_SUCCESS(NvRmPrivGetClockState(
         hRmDevice, NvRmPrivModuleID_MemoryController,
         &s_MemClocks.pMcInfo, &s_MemClocks.pMcState));
     s_MemClocks.Index = NVRM_AP15_DFS_EMC_FREQ_STEPS;  // invalid index
     NvOsMemset(s_Ap15EmcConfigSortedTable, 0,          // clean table
                sizeof(s_Ap15EmcConfigSortedTable));

     // Get EMC2x clock state from h/w
     Ap15Emc2xFreqGet(hRmDevice);

    // Check if configuration table is provided by ODM
    if ((ConfigurationsCount == 0) || (pEmcConfigurations == NULL))
    {
        s_Ap15EmcConfigSortedTable[0].Emc2xKHz = 0; // invalidate PLLM0 entry
        return;
    }
    if (reg != NV_EMC_BASIC_REV)
    {
        s_Ap15EmcConfigSortedTable[0].Emc2xKHz = 0; // invalidate PLLM0 entry
        NV_ASSERT(!"Invalid configuration table revision");
        return;
    }

    // Check PLLM0 range
    NV_ASSERT(PllM0KHz);
    if (PllM0KHz > (NvRmPrivGetSocClockLimits(
        NvRmPrivModuleID_ExternalMemoryController)->MaxKHz))
    {
        s_Ap15EmcConfigSortedTable[0].Emc2xKHz = 0; // invalidate PLLM0 entry
        NV_ASSERT(!"PLLM0 is outside supported EMC range");
        return;
    }

    // Check if PLLM0 is configured by boot loader as EMC clock source
    // (it can not and will not be changed by RM)
    if (s_MemClocks.pEmcState->SourceClock !=
        CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0)
    {
        s_Ap15EmcConfigSortedTable[0].Emc2xKHz = 0; // invalidate PLLM0 entry
        NV_ASSERT(!"Other than PLLM0 clock source is used for EMC");
        return;
    }

    // Sort list of EMC timing parameters in descending order of frequencies
    // evenly divided down from PLLM0; find matching entry for boot divisor
    for (i = 0, k = 0, Emc2xKHz = PllM0KHz; i < NVRM_AP15_DFS_EMC_FREQ_STEPS; )
    {
        s_Ap15EmcConfigSortedTable[i].Emc2xKHz = 0; // mark entry invalid
        for (j = 0; j < ConfigurationsCount; j++)
        {
            // Find match with 1MHz tolerance for allowed configuration
            if ((Emc2xKHz <= (pEmcConfigurations[j].SdramKHz * 2 + 1000)) &&
                (Emc2xKHz >= (pEmcConfigurations[j].SdramKHz * 2 - 1000)))
            {
                s_Ap15EmcConfigSortedTable[i].Timing0Reg = pEmcConfigurations[j].EmcTiming0;
                s_Ap15EmcConfigSortedTable[i].Timing1Reg = pEmcConfigurations[j].EmcTiming1;
                s_Ap15EmcConfigSortedTable[i].Timing2Reg = pEmcConfigurations[j].EmcTiming2;
                s_Ap15EmcConfigSortedTable[i].Timing3Reg = pEmcConfigurations[j].EmcTiming3;
                s_Ap15EmcConfigSortedTable[i].Timing4Reg = pEmcConfigurations[j].EmcTiming4;
                s_Ap15EmcConfigSortedTable[i].Timing5Reg = pEmcConfigurations[j].EmcTiming5;

                s_Ap15EmcConfigSortedTable[i].FbioCfg6Reg =
                    pEmcConfigurations[j].EmcFbioCfg6;
                s_Ap15EmcConfigSortedTable[i].FbioDqsibDly =
                    pEmcConfigurations[j].EmcFbioDqsibDly +
                    NvRmPrivGetEmcDqsibOffset(hRmDevice);
                s_Ap15EmcConfigSortedTable[i].FbioQuseDly =
                    pEmcConfigurations[j].EmcFbioQuseDly;
                s_Ap15EmcConfigSortedTable[i].CoreVoltageMv =
                    pEmcConfigurations[j].EmcCoreVoltageMv;

                // Determine EMC and MC clock divisors, MC clock source
                // (EMC always uses PLLM0 as a source), and CPU clock limit
                s_Ap15EmcConfigSortedTable[i].Emc2xKHz = Emc2xKHz; // accurate KHz
                if (i == 0)
                {
                    /*
                     * The first table entry specifies parameters for EMC2xFreq
                     * = PLLM0 frequency; the divisor field in EMC fractional
                     * divider register is set to "0". The divisor field in MC
                     * divider is set to "1", so that Emc1xFreq ~ 75% of McFreq
                     * using PLLM0 as MC clock source, if maximum MC frequency
                     * limit is not violated. Otherwise, find the highest MC
                     * frequency below the limit with PLLP0 as a source.
                     */
                    s_Ap15EmcConfigSortedTable[i].Emc2xDivisor = 0;
                    McKHz = (PllM0KHz * 2) / 3;
                    McMax = NvRmPrivGetSocClockLimits(
                        NvRmPrivModuleID_MemoryController)->MaxKHz;
                    NV_ASSERT(McMax);
                    if (McKHz <= McMax)
                    {
                        s_Ap15EmcConfigSortedTable[i].McDivisor = 1;
                        s_Ap15EmcConfigSortedTable[i].McClockSource =
                            CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0_MEM_CLK_SRC_PLLM_OUT0;
                    }
                    else if (NVRM_PLLP_FIXED_FREQ_KHZ <= McMax)
                    {
                        McKHz = NVRM_PLLP_FIXED_FREQ_KHZ;
                        s_Ap15EmcConfigSortedTable[i].McDivisor = 0;
                        s_Ap15EmcConfigSortedTable[i].McClockSource =
                            CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0_MEM_CLK_SRC_PLLP_OUT0;
                    }
                    else
                    {
                        reg = (2 * NVRM_PLLP_FIXED_FREQ_KHZ + McMax - 1) / McMax;
                        McKHz = (2 * NVRM_PLLP_FIXED_FREQ_KHZ) / reg;
                        s_Ap15EmcConfigSortedTable[i].McDivisor = reg - 2;
                        s_Ap15EmcConfigSortedTable[i].McClockSource =
                            CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0_MEM_CLK_SRC_PLLP_OUT0;
                    }
                }
                else
                {
                    /*
                     * If i = 1, 2, ... the table entry specifies parameters
                     * for EMC2xFreq = PLLM0 frequency/(2 * k); the divisor
                     * field in EMC fractional divider register should be set
                     * as 2 * (2 * k) - 2 = 4 * k - 2. The divisor field in MC
                     * divider is determined so that Emc1xFreq ~ 85% of McFreq
                     * using the same PLLM0 as MC clock source
                     */
                    s_Ap15EmcConfigSortedTable[i].Emc2xDivisor = (k << 2) - 2;
                    s_Ap15EmcConfigSortedTable[i].McDivisor = (19 +
                         17 * s_Ap15EmcConfigSortedTable[i].Emc2xDivisor) / 10;
                    s_Ap15EmcConfigSortedTable[i].McClockSource =
                        CLK_RST_CONTROLLER_CLK_SOURCE_MEM_0_MEM_CLK_SRC_PLLM_OUT0;
                    McKHz = 2 * PllM0KHz /
                        (s_Ap15EmcConfigSortedTable[i].McDivisor + 2);
                }
                if (s_Ap15EmcConfigSortedTable[i].Emc2xDivisor ==
                    s_MemClocks.pEmcState->Divider)
                {
                    s_MemClocks.Index = i;      // Boot configuration found
                }
                s_Ap15EmcConfigSortedTable[i].McKHz = McKHz;
                /*
                 * H/w CPU clock limit is determined from inequality:
                 * 1 mcclk period + 12 cpuclk periods >= 2 emcclck periods, or
                 * CpuKHz <= 11.9 * McKHz * Emc2xKHz / (4 * McKHz - Emc2xKHz)
                 * with 0.1/12 ~ 0.8% margin
                 * S/w CPU clock limit is determined per s/w policy:
                 * CpuKHz <= CpuMax * PolicyTabel[PLLM0/(2*EMC2xKHz)] / 256
                 * Final CPU clock limit is minimum of the above limits
                 */
                s_Ap15EmcConfigSortedTable[i].CpuLimitKHz =
                    (NvU32)NvDiv64(((NvU64)Emc2xKHz * McKHz * 119),
                                (((McKHz << 2) - Emc2xKHz) * 10));
                reg = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
                if (k != 0)
                {
                    NV_ASSERT(k < NV_ARRAY_SIZE(s_Cpu2EmcRatioPolicyTable));
                    reg = (reg * s_Cpu2EmcRatioPolicyTable[k]) >> 8;
                }
                if (s_Ap15EmcConfigSortedTable[i].CpuLimitKHz > reg)
                    s_Ap15EmcConfigSortedTable[i].CpuLimitKHz = reg;

                break;
            }
        }
        if (s_Ap15EmcConfigSortedTable[i].Emc2xKHz != 0)
            i++;                // Entry found - advance sorting index
        else if (i == 0)
            break;              // PLLM0 entry not found - abort sorting

        Emc2xKHz = PllM0KHz / ((++k) << 1);
        if (Emc2xKHz < NvRmPrivGetSocClockLimits(
            NvRmPrivModuleID_ExternalMemoryController)->MinKHz)
            break;              // Abort sorting at minimum EMC frequency
    }
    // Check if match for boot configuration found
    if (s_MemClocks.Index == NVRM_AP15_DFS_EMC_FREQ_STEPS)
        s_Ap15EmcConfigSortedTable[0].Emc2xKHz = 0; // invalidate PLLM0 entry
}

static NvBool
Ap15Emc2xClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmFreqKHz* pCpuTargetKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 i;
    NvBool FinalStep = NV_TRUE;
    NV_ASSERT(DomainKHz <= MaxKHz);
    pDfsSource->DividerSetting = 0; // no divider

    // If PLLM0 entry in EMC frequeuncies table is invalid, EMC frequency
    // will not be scaled; just fill in current EMC frequency
    if (s_Ap15EmcConfigSortedTable[0].Emc2xKHz == 0)
    {
        pDfsSource->SourceId = NvRmClockSource_Invalid;
        pDfsSource->SourceKHz = s_MemClocks.pEmcState->actual_freq;
        pDfsSource->MinMv = NvRmVoltsMaximum; // no v-scaling in this case
        return FinalStep;
    }

    // Only PLLM0 is used as EMC frequency source by DFS; its frequency is
    // always within h/w limits
    pDfsSource->SourceId = NvRmClockSource_PllM0;
    NV_ASSERT(s_Ap15EmcConfigSortedTable[0].Emc2xKHz <= MaxKHz);

    // Search sorted pre-defind EMC frequencies (divided down from PLLM0) for
    // the entry above and closest to the traget that also has CPU limit above
    // the CPU target. Use PLLM0 entry if not found.
    for (i = NVRM_AP15_DFS_EMC_FREQ_STEPS; i > 0;)
    {
        i--;
        if ((DomainKHz <= s_Ap15EmcConfigSortedTable[i].Emc2xKHz) &&
            (*pCpuTargetKHz <= s_Ap15EmcConfigSortedTable[i].CpuLimitKHz))
            break;
    }

    // Make sure the new entry is adjacent to the current (one step at a time)
    if (i > (s_MemClocks.Index + 1))
    {
        i = s_MemClocks.Index + 1;
        FinalStep = NV_FALSE;    // need more steps to reach target
    }
    else if ((i + 1) < s_MemClocks.Index)
    {
        i = s_MemClocks.Index - 1;
        FinalStep = NV_FALSE;    // need more steps to reach target
    }

    // Record found EMC entry, and limit CPU target if necessary
    pDfsSource->DividerSetting = i;
    pDfsSource->SourceKHz = s_Ap15EmcConfigSortedTable[i].Emc2xKHz;
    if (*pCpuTargetKHz > s_Ap15EmcConfigSortedTable[i].CpuLimitKHz)
        *pCpuTargetKHz = s_Ap15EmcConfigSortedTable[i].CpuLimitKHz;
    pDfsSource->MinMv = s_Ap15EmcConfigSortedTable[i].CoreVoltageMv;
    return FinalStep;
}

static void
Ap15Emc2xClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
    NvU32 Index;
    NvRmFreqKHz CpuFreq = NvRmPrivGetClockSourceFreq(NvRmClockSource_CpuBus);

    // Always return the requested source frequency
    *pDomainKHz = pDfsSource->SourceKHz;
    NV_ASSERT(*pDomainKHz);

    // If other than PLLM0 source is selected, EMC frequency is not scaled.
    if (pDfsSource->SourceId != NvRmClockSource_PllM0)
        return;

    // Divider settings in EMC source descriptor is an index into the table of
    // pre-defined EMC configurations in descending frequency order.
    Index = pDfsSource->DividerSetting;
    if (Index == s_MemClocks.Index)
        return;     // do nothing new index is the same as current

    // In case of EMC frequency increase:  check if EMC LL reservation should
    // be enabled, reconfigure EMC, then MC (make sure MC never exceeds EMC2x)
    // In case of EMC frequency decrease: reconfigure MC, then EMC (make sure
    // MC never exceeds EMC2x) and check if EMC LL reservation can be disabled
    if (Index < s_MemClocks.Index)
    {
        if (CpuFreq < (*pDomainKHz >> 1))
        {
            NVRM_AP15_EMCLL_RETRSV_ENABLE;
        }
        Ap15Emc2xClockSet(
            hRmDevice, NV_TRUE, &s_Ap15EmcConfigSortedTable[Index]);
        Ap15McClockSet(hRmDevice, &s_Ap15EmcConfigSortedTable[Index]);
    }
    else
    {
        Ap15McClockSet(hRmDevice, &s_Ap15EmcConfigSortedTable[Index]);
        Ap15Emc2xClockSet(
            hRmDevice, NV_FALSE, &s_Ap15EmcConfigSortedTable[Index]);

        if (CpuFreq >= (*pDomainKHz >> 1))
        {
            NVRM_AP15_EMCLL_RETRSV_DISABLE;
        }
    }
    s_MemClocks.Index = Index;
}

void
NvRmPrivAp15FastClockConfig(NvRmDeviceHandle hRmDevice)
{
#if !NV_OAL
    NvU32 divm1, divp2;
    NvRmFreqKHz SclkKHz, CpuKHz, PllP2KHz, PllM1KHz;
    NvRmFreqKHz FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);

    // Set fastest EMC/MC configuration provided PLLM0 boot frequency matches
    // one of the pre-defined configurations, i.e, it is the first entry in the
    // sorted table
    if (s_Ap15EmcConfigSortedTable[0].Emc2xKHz == FreqKHz)
    {
        for (;;)
        {
            Ap15Emc2xClockSet(
                hRmDevice, NV_TRUE, &s_Ap15EmcConfigSortedTable[s_MemClocks.Index]);
            Ap15McClockSet(hRmDevice, &s_Ap15EmcConfigSortedTable[s_MemClocks.Index]);
            if (s_MemClocks.Index == 0)
                break;
            s_MemClocks.Index--;
        }
    }

    // Set AVP/System Bus clock (now, with nominal core voltage it can be up
    // to SoC maximum). First determine settings for PLLP and PLLM dividers
    // to get maximum possible frequency on PLLP_OUT2 and PLLM_OUT1 outputs.
    SclkKHz = NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System)->MaxKHz;
    NV_ASSERT(SclkKHz);

    FreqKHz = NVRM_PLLP_FIXED_FREQ_KHZ;
    PllP2KHz = SclkKHz;
    divp2 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, FreqKHz, PllP2KHz, &PllP2KHz);

    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    PllM1KHz = SclkKHz;
    divm1 = NvRmPrivFindFreqMaxBelow(
        NvRmClockDivider_Fractional_2, FreqKHz, PllM1KHz, &PllM1KHz);

    // Now configure both dividers and select the output with highest frequency
    // as a source for the system bus clock; reconfigure MIO as necessary
    SclkKHz = NV_MAX(PllM1KHz, PllP2KHz);
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus);
    if (FreqKHz < SclkKHz)
    {
        Ap15MioReconfigure(hRmDevice, SclkKHz);
    }
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllP2)->pInfo.pDivider,
        divp2);
    NvRmPrivDividerSet(
        hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_PllM1)->pInfo.pDivider,
        divm1);
    if (SclkKHz == PllP2KHz)
    {
        NvRmPrivCoreClockSet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore,
            NvRmClockSource_PllP2, 0, 0);
    }
    else
    {
        NvRmPrivCoreClockSet(hRmDevice,
            NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore,
            NvRmClockSource_PllM1, 0, 0);
    }
    if (FreqKHz >= SclkKHz)
    {
        Ap15MioReconfigure(hRmDevice, SclkKHz);
    }
    NvRmPrivBusClockInit(hRmDevice, SclkKHz);

    // Set PLLC and CPU clock to SoC maximum - can be done now, when core
    // voltage is guaranteed to be nominal, provided none of the display
    // heads is already using PLLC as pixel clock source.
    CpuKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
    FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0);
    if (CpuKHz != FreqKHz)
    {
        NvRmPrivBoostPllC(hRmDevice);
    }
    NvRmPrivCoreClockSet(hRmDevice,
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore,
        NvRmClockSource_PllC0, 0, 0);
#endif
}

void
NvRmPrivAp15ClipCpuEmcHighLimits(
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
    if (s_Ap15EmcConfigSortedTable[0].Emc2xKHz == 0)
        return;

    // Clip strategy: "throttling" - find the floor for EMC high limit
    // (above domain minimum, of course)
    if ((*pEmcHighKHz) < MinKHz)
        *pEmcHighKHz = MinKHz;
    for (i = 0; i < NVRM_AP15_DFS_EMC_FREQ_STEPS; i++)
    {
        EmcKHz = s_Ap15EmcConfigSortedTable[i].Emc2xKHz >> 1;
        if (EmcKHz <= (*pEmcHighKHz))
            break;
    }
    if ((i == NVRM_AP15_DFS_EMC_FREQ_STEPS) || (EmcKHz < MinKHz))
    {
        i--;
        EmcKHz = s_Ap15EmcConfigSortedTable[i].Emc2xKHz >> 1;
    }
    *pEmcHighKHz = EmcKHz;

    // Clip strategy: "throttling" - restrict CPU high limit by EMC
    // configuration ((above domain minimum, of course)
    if ((*pCpuHighKHz) > s_Ap15EmcConfigSortedTable[i].CpuLimitKHz)
        (*pCpuHighKHz) = s_Ap15EmcConfigSortedTable[i].CpuLimitKHz;
    if ((*pCpuHighKHz) < NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Cpu))
        *pCpuHighKHz = NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Cpu);
#endif
}

NvRmFreqKHz
NvRmPrivAp15GetEmcSyncFreq(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID Module)
{
    NvRmFreqKHz FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);

    switch (Module)
    {
        case NvRmModuleID_2D:
        case NvRmModuleID_Epp:
            // 2D/EPP frequency is dynamically synchronized with current EMC speed
            // (high if EMC divisor 0, and low otherwise)
            if (s_MemClocks.pEmcState && (s_MemClocks.pEmcState->Divider != 0))
                FreqKHz = FreqKHz / NVRM_PLLM_2D_LOW_SPEED_RATIO;
            else
                FreqKHz = FreqKHz / NVRM_PLLM_2D_HIGH_SPEED_RATIO;
            break;

        case NvRmModuleID_GraphicsHost:
            // Host frequency is static, synchronized with EMC range set by BCT
            FreqKHz = FreqKHz / NVRM_PLLM_HOST_SPEED_RATIO;
            break;

        default:
            NV_ASSERT(!"Invalid module for EMC synchronization");
            FreqKHz = NvRmPrivGetSocClockLimits(Module)->MaxKHz;
            break;
    }
    return FreqKHz;
}

/*****************************************************************************/

static void
Ap15SystemClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 i;
    NvRmFreqKHz SourceKHz;
    NV_ASSERT(DomainKHz <= MaxKHz);
    pDfsSource->DividerSetting = 0; // no divider

    // 1st try oscillator
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkM;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }

    // 2nd choice - doubler
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkD);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkD;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }

    /*
     * 3rd option - PLLP divider per policy specification. Find
     * the policy entry with source frequency closest and above requested.
     * If requested frequency exceeds all policy options within domain
     * maximum limit, select the entry with the highest possible frequency.
     */
    for (i = 0; i < s_Ap15PllPSystemClockPolicyEntries; i++)
    {
        SourceKHz = s_Ap15PllPSystemClockPolicy[i].SourceKHz;
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
    if (i == s_Ap15PllPSystemClockPolicyEntries)
    {
        i--;    // last/highest source is the best we can do
    }
    pDfsSource->SourceId = s_Ap15PllPSystemClockPolicy[i].SourceId;
    pDfsSource->SourceKHz = s_Ap15PllPSystemClockPolicy[i].SourceKHz;
    pDfsSource->DividerSetting = s_Ap15PllPSystemClockPolicy[i].DividerSetting;

    /*
     * 4st and final option - PLLM divider fixed at maximum possible frequency
     * during initialization. Select PLLP/PLLM divider according to the
     * following rule: select the divider with smaller frequency if it is equal
     * or above the target frequency, otherwise select the divider with bigger
     * output frequency.
     */
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM1);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (SourceKHz > pDfsSource->SourceKHz)
    {
        if (pDfsSource->SourceKHz >= DomainKHz)
            goto get_mv;    // keep PLLP divider as a source
    }
    else // SourceKHz <= pDfsSource->SourceKHz
    {
        if (SourceKHz < DomainKHz)
            goto get_mv;    // keep PLLP divider as a source
    }
    // Select PLLM_OUT1 divider as a source (considered as a fixed source -
    // divider settings are ignored)
    pDfsSource->SourceId = NvRmClockSource_PllM1;
    pDfsSource->SourceKHz = SourceKHz;

get_mv:
    // Finally get operational voltage for found source
    pDfsSource->MinMv = NvRmPrivModuleVscaleGetMV(
        hRmDevice, NvRmPrivModuleID_System, pDfsSource->SourceKHz);
}

static void
Ap15CpuClockSourceFind(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz DomainKHz,
    NvRmDfsSource* pDfsSource)
{
    NvU32 i;
    NvRmFreqKHz SourceKHz;
    NV_ASSERT(DomainKHz <= MaxKHz);
    pDfsSource->DividerSetting = 0; // no divider

    // 1st try oscillator
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_ClkM;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }

    // 2nd choice - doubler - no longer supported
    // 3rd choice - PLLP divider per policy specification
    SourceKHz =
        s_Ap15PllPCpuClockPolicy[s_Ap15PllPCpuClockPolicyEntries-1].SourceKHz;
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        // The requested frequency is within PLLP divider policy table, and all
        // policy entries are within domain maximum limit. Then, find the entry
        // with source frequency closest and above the requested.
        for (i = 0; i < s_Ap15PllPCpuClockPolicyEntries; i++)
        {
            SourceKHz = s_Ap15PllPCpuClockPolicy[i].SourceKHz;
            if (DomainKHz <= SourceKHz)
                break;
        }
        if (s_Ap15PllPCpuClockPolicy[i].DividerSetting == 0)
            pDfsSource->SourceId = NvRmClockSource_PllP0;   // Bypass 1:1 divider
        else
            pDfsSource->SourceId = s_Ap15PllPCpuClockPolicy[i].SourceId;
        pDfsSource->SourceKHz = s_Ap15PllPCpuClockPolicy[i].SourceKHz;
        pDfsSource->DividerSetting = s_Ap15PllPCpuClockPolicy[i].DividerSetting;
        goto get_mv;
    }

    // 4th choice PLLM base output
    SourceKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0);
    NV_ASSERT(SourceKHz <= MaxKHz);
    if (DomainKHz <= SourceKHz)
    {
        pDfsSource->SourceId = NvRmClockSource_PllM0;
        pDfsSource->SourceKHz = SourceKHz;
        goto get_mv;
    }

    // 5th choice PLLP base output (not used - covered by 3rd choice, case 1:1)
    // 6th and final choice - PLLC base output at domain limit
    pDfsSource->SourceId = NvRmClockSource_PllC0;
    pDfsSource->SourceKHz = MaxKHz;

get_mv:
    // Finally get operational voltage for found source
    pDfsSource->MinMv = NvRmPrivModuleVscaleGetMV(
        hRmDevice, NvRmModuleID_Cpu, pDfsSource->SourceKHz);
}

static void
Ap15SystemBusClockConfigure(
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
            // Reconfigure PLLP variable divider if it is used as a source
            NvRmPrivDividerSet(hRmDevice,
                NvRmPrivGetClockSourceHandle(SourceId)->pInfo.pDivider,
                pDfsSource->DividerSetting);
            // fall through
        case NvRmClockSource_PllM1:
        case NvRmClockSource_ClkD:
        case NvRmClockSource_ClkM:
            break;  // fixed sources - do nothing
        default:
            NV_ASSERT(!"Invalid source (per policy)");
    }
    NV_ASSERT_SUCCESS(NvRmPrivCoreClockConfigure(
        hRmDevice, pCinfo, MaxKHz, pDomainKHz, &SourceId));
}

static void
Ap15CpuBusClockConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz MaxKHz,
    NvRmFreqKHz* pDomainKHz,
    const NvRmDfsSource* pDfsSource)
{
    NvRmClockSource SourceId = pDfsSource->SourceId;
    const NvRmCoreClockInfo* pCinfo =
        NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;

    switch(SourceId)
    {
        case NvRmClockSource_PllC0:
            // DFS PLLC policy - configure PLLC if disabled; otherwise keep
            // keep it as is (the latter means either DFS has already set it
            // to domain limit, or PLLC is used as display pixel clock source)
            if (NvRmPrivGetClockSourceFreq(NvRmClockSource_PllC0) <=
                NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM))
            {
                NvRmFreqKHz TargetKHz = pDfsSource->SourceKHz;
                NvRmPrivAp15PllConfigureSimple(
                    hRmDevice, SourceId, MaxKHz, &TargetKHz);
            }
            break;
        case NvRmClockSource_PllP4:
            // Reconfigure PLLP variable divider if it is used as a source;
            // If source frequency is going down, get EMC configuration is ready
            if (pDfsSource->SourceKHz < NvRmPrivGetClockSourceFreq(SourceId))
                NvRmPrivAp15SetEmcForCpuSrcSwitch(hRmDevice);
            NvRmPrivDividerSet(hRmDevice,
                NvRmPrivGetClockSourceHandle(SourceId)->pInfo.pDivider,
                pDfsSource->DividerSetting);
            // fall through
        case NvRmClockSource_PllP0:
        case NvRmClockSource_PllM0:
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
/* If time is specified in ns, and frequency in KHz, then cycles =
 * (ns * KHz / 10^6) = (ns * (KHz * 2^20 / 10^6) / 2^20) = (ns * KiHz / 2^20),
 * where KiHz = (KHz * 2^20 / 10^6) ~ (KHz * 4295 / 4096) with error < 0.001%.
 */
#define NVRM_TIME_TO_CYCLES(ns, KiHz) (((ns * KiHz) + (0x1 << 20)- 1) >> 20)

#define NV_DRF_MAX_NUM(d,r,f,n) \
    ((((n) <= NV_FIELD_MASK(d##_##r##_0_##f##_RANGE)) ? \
      (n) : NV_FIELD_MASK(d##_##r##_0_##f##_RANGE)) << \
     NV_FIELD_SHIFT(d##_##r##_0_##f##_RANGE))

static void
Ap15MioReconfigure(
        NvRmDeviceHandle hRmDevice,
        NvRmFreqKHz MioKHz)
{
    NvU32 reg, mask;
    NvU32 MioKiHz = ((MioKHz * 4295) >> 12);
    NvOdmAsynchMemConfig  MemConfig;
    /*
     * Reconfigure MIO timing when clock frequency changes. Check only Async
     * Memory devices connected to CS1/MIO_B and CS3/MIO_A (CS0 is dedicated
     * for NOR, we do not care after boot, and CS2 is dedicated to SDRAM with
     * its own clock)
     */
    if (NvOdmQueryAsynchMemConfig(1, &MemConfig) == NV_TRUE)
    {
        reg = NV_REGR(hRmDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_XMB_MIO_CFG_0);
        mask =
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_WR_DEAD_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_WR_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_RD_DEAD_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_RD_TIME, 0xFFFFFFFFUL);
        reg = (reg & (~mask)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_WR_DEAD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.WriteDeadTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_WR_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.WriteAccessTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_RD_DEAD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.ReadDeadTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_B_RD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.ReadAccessTime, MioKiHz));
        NV_REGW(hRmDevice, NvRmModuleID_Misc, 0,  APB_MISC_PP_XMB_MIO_CFG_0, reg);
    }
    if (NvOdmQueryAsynchMemConfig(3, &MemConfig) == NV_TRUE)
    {
        reg = NV_REGR(hRmDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_XMB_MIO_CFG_0);
        mask =
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_WR_DEAD_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_WR_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_RD_DEAD_TIME, 0xFFFFFFFFUL) |
            NV_DRF_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_RD_TIME, 0xFFFFFFFFUL);
        reg = (reg & (~mask)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_WR_DEAD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.WriteDeadTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_WR_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.WriteAccessTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_RD_DEAD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.ReadDeadTime, MioKiHz)) |
            NV_DRF_MAX_NUM(APB_MISC_PP, XMB_MIO_CFG, MIO_A_RD_TIME,
                NVRM_TIME_TO_CYCLES(MemConfig.ReadAccessTime, MioKiHz));
        NV_REGW(hRmDevice, NvRmModuleID_Misc, 0,  APB_MISC_PP_XMB_MIO_CFG_0, reg);
    }
}

NvBool NvRmPrivAp15DfsClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsFrequencies* pMaxKHz,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvU32 i;
    NvBool Status;
    NvRmFreqKHz FreqKHz;
    NvRmDfsSource CpuClockSource;
    NvRmDfsSource SystemClockSource;
    NvRmDfsSource Emc2xClockSource;
    NvBool CpuKHzUp = pDfsKHz->Domains[NvRmDfsClockId_Cpu] >
        NvRmPrivGetClockSourceFreq(NvRmClockSource_CpuBus);

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pMaxKHz && pDfsKHz);

    /*
     * Adjust System bus core clock. It should be sufficient to supply AVP,
     * and all bus clocks. Also make sure that AHB bus frequency is above
     * the one requested for APB clock.
     */
    for (FreqKHz = 0, i = 1; i < NvRmDfsClockId_Num; i++)
    {
        if ((i != NvRmDfsClockId_Cpu) &&
            (i != NvRmDfsClockId_Emc))
        {
            FreqKHz = (FreqKHz > pDfsKHz->Domains[i]) ?
                            FreqKHz : pDfsKHz->Domains[i];
        }
    }
    pDfsKHz->Domains[NvRmDfsClockId_System] = FreqKHz;

#if LIMIT_SYS_TO_VDE_RATIO
    if (pDfsKHz->Domains[NvRmDfsClockId_Vpipe] <
        (FreqKHz >> (LIMIT_SYS_TO_VDE_RATIO - 1)))
    {
        pDfsKHz->Domains[NvRmDfsClockId_Vpipe] =
            (FreqKHz >> (LIMIT_SYS_TO_VDE_RATIO - 1));
    }
#endif

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
    if (pDfsKHz->Domains[NvRmDfsClockId_Ahb] <
        pDfsKHz->Domains[NvRmDfsClockId_Apb])
    {
        pDfsKHz->Domains[NvRmDfsClockId_Ahb] =
            pDfsKHz->Domains[NvRmDfsClockId_Apb];
    }

    // Find clock sources for CPU, System and Memory clocks. H/w requirement
    // to increase memory clocks in steps, may limit CPU clock as well
    Ap15SystemClockSourceFind(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_System],
        pDfsKHz->Domains[NvRmDfsClockId_System],
        &SystemClockSource);
    Status = Ap15Emc2xClockSourceFind(hRmDevice,
        (pMaxKHz->Domains[NvRmDfsClockId_Emc] << 1),
        (pDfsKHz->Domains[NvRmDfsClockId_Emc] << 1),
        &pDfsKHz->Domains[NvRmDfsClockId_Cpu],
        &Emc2xClockSource);
    Ap15CpuClockSourceFind(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_Cpu],
        pDfsKHz->Domains[NvRmDfsClockId_Cpu],
        &CpuClockSource);

#if !NV_OAL
    // Adjust core voltage for the new clock sources before actual change
    NvRmPrivVoltageScale(NV_TRUE, CpuClockSource.MinMv,
                         SystemClockSource.MinMv, Emc2xClockSource.MinMv);
#endif

    // Configure System bus and derived clocks. Note that APB is the only
    // clock in system complex that may have different (lower) maximum
    // limit - pass it explicitly to set function.
    if (FreqKHz < NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus))
        FreqKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_SystemBus);
    Ap15MioReconfigure(hRmDevice, FreqKHz); // MIO timing for max frequency
    Ap15SystemBusClockConfigure(hRmDevice,
        pMaxKHz->Domains[NvRmDfsClockId_System],
        &pDfsKHz->Domains[NvRmDfsClockId_System],
        &SystemClockSource);
    pDfsKHz->Domains[NvRmDfsClockId_Avp] =
        pDfsKHz->Domains[NvRmDfsClockId_System];        // no AVP clock skipping
    NvRmPrivBusClockFreqSet(hRmDevice,
        pDfsKHz->Domains[NvRmDfsClockId_System],
        &pDfsKHz->Domains[NvRmDfsClockId_Vpipe],
        &pDfsKHz->Domains[NvRmDfsClockId_Ahb],
        &pDfsKHz->Domains[NvRmDfsClockId_Apb],
        pMaxKHz->Domains[NvRmDfsClockId_Apb]);
    Ap15MioReconfigure(hRmDevice, pDfsKHz->Domains[NvRmDfsClockId_Ahb]);

    // Configure CPU core clock before Memory if CPU frequency goes down
    if (!CpuKHzUp)
    {
        Ap15CpuBusClockConfigure(hRmDevice,
            pMaxKHz->Domains[NvRmDfsClockId_Cpu],
            &pDfsKHz->Domains[NvRmDfsClockId_Cpu],
            &CpuClockSource);
    }
    // Configure Memory clocks and convert frequency to DFS EMC 1x domain
    FreqKHz = pDfsKHz->Domains[NvRmDfsClockId_Emc] << 1;
    Ap15Emc2xClockConfigure(hRmDevice,
        (pMaxKHz->Domains[NvRmDfsClockId_Emc] << 1),
        &FreqKHz, &Emc2xClockSource);
    pDfsKHz->Domains[NvRmDfsClockId_Emc] = FreqKHz >> 1;
    // Configure CPU core clock after Memory if CPU frequency goes up
    if (CpuKHzUp)
    {
        Ap15CpuBusClockConfigure(hRmDevice,
            pMaxKHz->Domains[NvRmDfsClockId_Cpu],
            &pDfsKHz->Domains[NvRmDfsClockId_Cpu],
            &CpuClockSource);
    }

#if !NV_OAL
    // Adjust core voltage for the new clock sources after actual change
    NvRmPrivVoltageScale(NV_FALSE, CpuClockSource.MinMv,
                         SystemClockSource.MinMv, Emc2xClockSource.MinMv);
#endif
    return Status;
}

void
NvRmPrivAp15DfsClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvRmFreqKHz SystemFreq;
    const NvRmCoreClockInfo* pCinfo;
    NV_ASSERT(hRmDevice && pDfsKHz);

    // Get frequencies of the System core clock, AVP clock (the same as System
    // - no clock skipping), AHB, APB, and V-pipe bus clock frequencies
    pCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_SystemBus)->pInfo.pCore;
    SystemFreq = NvRmPrivCoreClockFreqGet(hRmDevice, pCinfo);
    pDfsKHz->Domains[NvRmDfsClockId_System] = SystemFreq;
    pDfsKHz->Domains[NvRmDfsClockId_Avp] = SystemFreq;

    NvRmPrivBusClockFreqGet(
        hRmDevice, SystemFreq,
        &pDfsKHz->Domains[NvRmDfsClockId_Vpipe],
        &pDfsKHz->Domains[NvRmDfsClockId_Ahb],
        &pDfsKHz->Domains[NvRmDfsClockId_Apb]);

    // Get CPU core clock frequencies
    pCinfo = NvRmPrivGetClockSourceHandle(NvRmClockSource_CpuBus)->pInfo.pCore;
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] =
        NvRmPrivCoreClockFreqGet(hRmDevice, pCinfo);

    // Get EMC clock frequency (DFS monitors EMC 1x domain)
    Ap15Emc2xFreqGet(hRmDevice);   // Get EMC2x clock state from h/w
    pDfsKHz->Domains[NvRmDfsClockId_Emc] =
        (s_MemClocks.pEmcState->actual_freq >> 1);
}

void
NvRmPrivAp15DfsVscaleFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmMilliVolts TargetMv,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvU32 i;
    NvRmMilliVolts v;
    NvRmFreqKHz Fa, Fb, f;
    NvRmDfsSource DfsClockSource;
    NvRmFreqKHz CpuMaxKHz = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz;
    NvRmFreqKHz SysMaxKHz =
        NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System)->MaxKHz;
    NV_ASSERT(hRmDevice && pDfsKHz);

    // If PLLM0 entry in EMC scaling table is valid, search the table for
    // the entry below and closest to the traget voltage. Otherwise, there
    // is no EMC scaling - just return current EMC frequency.
    pDfsKHz->Domains[NvRmDfsClockId_Emc] =
        (s_MemClocks.pEmcState->actual_freq >> 1);
    f = NvRmFreqMaximum;    // assume CPU is not throttled by EMC
    if (s_Ap15EmcConfigSortedTable[0].Emc2xKHz != 0)
    {
        for (i = 0; i < (NVRM_AP15_DFS_EMC_FREQ_STEPS - 1); i++)
        {
            if ((s_Ap15EmcConfigSortedTable[i+1].Emc2xKHz == 0) ||
                (s_Ap15EmcConfigSortedTable[i].CoreVoltageMv <= TargetMv))
                break;  // exit if found entry or next entry is invalid
        }
        pDfsKHz->Domains[NvRmDfsClockId_Emc] =
            (s_Ap15EmcConfigSortedTable[i].Emc2xKHz >> 1);
        f = s_Ap15EmcConfigSortedTable[i].CpuLimitKHz;  // throttle CPU
    }

    // Binary search for maximum CPU frequency, with source that can be used
    // at target voltage or below
    Fb = NV_MIN(CpuMaxKHz, f);
    Fa = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(Fa <= Fb);
    while ((Fb - Fa) > 1000)    // 1MHz resolution
    {
        f = (Fa + Fb) >> 1;
        Ap15CpuClockSourceFind(hRmDevice, CpuMaxKHz, f, &DfsClockSource);
        v = DfsClockSource.MinMv;
        if (v <= TargetMv)
            Fa = f;
        else
            Fb = f;
    }
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] = Fa;

    // Binary search for maximum System/Avp frequency, with source that can be used
    // at target voltage or below
    Fb = SysMaxKHz;
    Fa = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    NV_ASSERT(Fa <= Fb);
    while ((Fb - Fa) > 1000)    // 1MHz resolution
    {
        f = (Fa + Fb) >> 1;
        Ap15SystemClockSourceFind(hRmDevice, SysMaxKHz, f, &DfsClockSource);
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
    pDfsKHz->Domains[NvRmDfsClockId_Vpipe] = Fa;
}
