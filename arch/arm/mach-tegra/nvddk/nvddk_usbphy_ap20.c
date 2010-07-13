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
 * @brief <b>NVIDIA Driver Development Kit:
 *           NvDDK USB PHY functions</b>
 *
 * @b Description: Defines USB PHY private functions
 *
 */

#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "ap20/arusb.h"
#include "ap20/arapb_misc.h"
#include "nvrm_hardware_access.h"
#include "nvddk_usbphy_priv.h"
#include "nvodm_query.h"
#include "ap20/arahb_arbc.h"
#include "ap20/arclk_rst.h"

/* Defines for USB register read and writes */
#define USB_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB2_CONTROLLER_2_USB2D_##reg##_0)/4))

#define USB_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB2_CONTROLLER_2_USB2D_##reg##_0)/4), (data))

// Read perticular field value from reg mentioned
#define USB_DRF_VAL(reg, field, value) \
    NV_DRF_VAL(USB2_CONTROLLER_2_USB2D, reg, field, value)

#define USB_FLD_SET_DRF_DEF(reg, field, define, value) \
    NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_2_USB2D, reg, field, define, value)

#define USB_REG_SET_DRF_DEF(reg, field, define) \
    NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_2_USB2D, reg, field, define, USB_REG_RD(reg))

#define USB_REG_UPDATE_DEF(reg, field, define) \
    USB_REG_WR(reg, USB_REG_SET_DRF_DEF(reg, field, define))

#define USB_DRF_DEF(reg, field, define) \
    NV_DRF_DEF(USB2_CONTROLLER_2_USB2D, reg, field, define)

#define USB_DRF_VAL(reg, field, value) \
    NV_DRF_VAL(USB2_CONTROLLER_2_USB2D, reg, field, value)

#define USB_REG_READ_VAL(reg, field) \
    USB_DRF_VAL(reg, field, USB_REG_RD(reg))

#define USB_FLD_SET_DRF_NUM(reg, field, num, value) \
    NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_2_USB2D, reg, field, num, value);

/* Defines for USB IF register read and writes */

#define USB1_IF_FLD_SET_DRF_DEF(reg, field, define, value) \
    NV_FLD_SET_DRF_DEF(USB1_IF, reg, field, define, value)

#define USB1_IF_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB1_IF_##reg##_0)/4))

#define USB1_IF_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB1_IF_##reg##_0)/4), (data))

#define USB1_IF_REG_SET_DRF_DEF(reg, field, define) \
    NV_FLD_SET_DRF_DEF(USB1_IF, reg, field, define, USB1_IF_REG_RD(reg))

#define USB1_IF_REG_UPDATE_DEF(reg, field, define) \
    USB1_IF_REG_WR(reg, USB1_IF_REG_SET_DRF_DEF(reg, field, define))

#define USB1_IF_DRF_VAL(reg, field, value) \
    NV_DRF_VAL(USB1_IF, reg, field, value)

#define USB1_IF_REG_READ_VAL(reg, field) \
    USB1_IF_DRF_VAL(reg, field, USB1_IF_REG_RD(reg))

#define USB3_IF_FLD_SET_DRF_DEF(reg, field, define, value) \
    NV_FLD_SET_DRF_DEF(USB3_IF_USB, reg, field, define, value)

#define USB_IF_DRF_DEF(reg, field, define) \
    NV_DRF_DEF(USB2_IF_USB, reg, field, define)

#define USB_IF_DRF_NUM(reg, field, value) \
    NV_DRF_NUM(USB2_IF_USB, reg, field, value)

#define USB_IF_DRF_VAL(reg, field, value) \
    NV_DRF_VAL(USB2_IF_USB, reg, field, value)

#define USB_IF_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB2_IF_USB_##reg##_0)/4))

#define USB_IF_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB2_IF_USB_##reg##_0)/4), (data))

#define USB_IF_REG_READ_VAL(reg, field) \
    USB_IF_DRF_VAL(reg, field, USB_IF_REG_RD(reg))

#define USB_IF_REG_SET_DRF_NUM(reg, field, define) \
    NV_FLD_SET_DRF_NUM(USB2_IF_USB, reg, field, define, USB_IF_REG_RD(reg))

#define USB_IF_REG_SET_DRF_DEF(reg, field, define) \
    NV_FLD_SET_DRF_DEF(USB2_IF_USB, reg, field, define, USB_IF_REG_RD(reg))

#define USB_IF_REG_UPDATE_DEF(reg, field, define) \
    USB_IF_REG_WR(reg, USB_IF_REG_SET_DRF_DEF(reg, field, define))

#define USB_IF_REG_UPDATE_NUM(reg, field, define) \
    USB_IF_REG_WR(reg, USB_IF_REG_SET_DRF_NUM(reg, field, define))


// defines for ULPI register access
#define ULPI_IF_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB2_IF_ULPI_##reg##_0)/4))

#define ULPI_IF_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB2_IF_ULPI_##reg##_0)/4), (data))

#define ULPI_IF_DRF_DEF(reg, field, define) \
    NV_DRF_DEF(USB2_IF_ULPI, reg, field, define)

#define ULPI_IF_DRF_NUM(reg, field, define) \
    NV_DRF_NUM(USB2_IF_ULPI, reg, field, define)


/* Defines for USB IF register read and writes */

#define USB_UTMIP_FLD_SET_DRF_DEF(reg, field, define, value) \
    NV_FLD_SET_DRF_NUM(USB1_UTMIP, reg, field, define, value)

#define USB_UTMIP_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB1_UTMIP_##reg##_0)/4))

#define USB_UTMIP_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB1_UTMIP_##reg##_0)/4), (data))

#define USB_UTMIP_REG_SET_DRF_NUM(reg, field, num) \
    NV_FLD_SET_DRF_NUM(USB1_UTMIP, reg, field, num, USB_UTMIP_REG_RD(reg))

#define USB_UTMIP_REG_UPDATE_NUM(reg, field, num) \
    USB_UTMIP_REG_WR(reg, USB_UTMIP_REG_SET_DRF_NUM(reg, field, num))

/* Defines for MISC register access */
#define USB_MISC_REGR(pUsbPhy, offset) \
    NV_READ32(pUsbPhy->MiscVirAdr + offset/4)

#define USB_MISC_REGW(pUsbPhy, offset, value) \
    NV_WRITE32(pUsbPhy->MiscVirAdr + offset/4, value)

/**
 * Structure defining the fields for USB UTMI clocks delay Parameters.
 */
typedef struct UsbPllDelayParamsRec
{
    // Pll-U Enable Delay Count
    NvU8 EnableDelayCount;
    //PLL-U Stable count
    NvU8 StableCount;
    //Pll-U Active delay count
    NvU8 ActiveDelayCount;
    //PLL-U Xtal frequency count
    NvU8 XtalFreqCount;
} UsbPllDelayParams;

/*
 * Set of oscillator frequencies supported
 */
typedef enum
{
    NvRmClocksOscFreq_13_MHz = 0x0,
    NvRmClocksOscFreq_19_2_MHz,
    NvRmClocksOscFreq_12_MHz,
    NvRmClocksOscFreq_26_MHz,
    NvRmClocksOscFreq_Num,       // dummy to get number of frequencies
    NvRmClocksOscFreq_Force32 = 0x7fffffff
} NvRmClocksOscFreq;

// Possible Oscillator Frequecies in KHz for mapping the index
static NvRmFreqKHz s_RmOscFrequecy [NvRmClocksOscFreq_Num] =
{
    13000, // 13 Mega Hertz
    19200,// 19.2 Mega Hertz
    12000,// 12 Mega Hertz
    26000 // 26 Mega Hertz
};

///////////////////////////////////////////////////////////////////////////////
// USB PLL CONFIGURATION & PARAMETERS: refer to the arapb_misc_utmip.spec file.
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS for different clock generators:
//-----------------------------------------------------------------------------
// Reference frequency     13.0MHz         19.2MHz         12.0MHz     26.0MHz
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   02 (02h)        03 (03h)        02 (02h)    04 (04h)
// PLLU_STABLE_COUNT       51 (33h)        75 (4Bh)        47 (2Fh)   102 (66h)
// PLL_ACTIVE_DLY_COUNT    05 (05h)        06 (06h)        04 (04h)    09 (09h)
// XTAL_FREQ_COUNT        127 (7Fh)       187 (BBh)       118 (76h)   254 (FEh)
///////////////////////////////////////////////////////////////////////////////
static const UsbPllDelayParams s_UsbPllDelayParams[NvRmClocksOscFreq_Num] =
{
    //ENABLE_DLY,  STABLE_CNT,  ACTIVE_DLY,  XTAL_FREQ_CNT
    {0x02,         0x33,        0x05,        0x7F}, // For NvRmClocksOscFreq_13_MHz,
    {0x03,         0x4B,        0x06,        0xBB}, // For NvRmClocksOscFreq_19_2_MHz
    {0x02,         0x2F,        0x04,        0x76}, // For NvRmClocksOscFreq_12_MHz
    {0x04,         0x66,        0x09,        0xFE}  // For NvRmClocksOscFreq_26_MHz
};

///////////////////////////////////////////////////////////////////////////////
// USB Debounce values IdDig, Avalid, Bvalid, VbusValid, VbusWakeUp, and SessEnd.
// Each of these signals have their own debouncer and for each of those one out
// of 2 debouncing times can be chosen (BIAS_DEBOUNCE_A or BIAS_DEBOUNCE_B.)
//
// The values of DEBOUNCE_A and DEBOUNCE_B are calculated as follows:
// 0xffff -> No debouncing at all
// <n> ms = <n> *1000 / (1/19.2MHz) / 4
// So to program a 1 ms debounce for BIAS_DEBOUNCE_A, we have:
// BIAS_DEBOUNCE_A[15:0] = 1000 * 19.2 / 4  = 4800 = 0x12c0
// We need to use only DebounceA, We dont need the DebounceB
// values, so we can keep those to default.
///////////////////////////////////////////////////////////////////////////////
static const NvU32 s_UsbBiasDebounceATime[NvRmClocksOscFreq_Num] =
{
    /* Ten milli second delay for BIAS_DEBOUNCE_A */
    0x7EF4,  // For NvRmClocksOscFreq_13_MHz,
    0xBB80,  // For NvRmClocksOscFreq_19_2_MHz
    0x7530,  // For NvRmClocksOscFreq_12_MHz
    0xFDE8   // For NvRmClocksOscFreq_26_MHz
};

///////////////////////////////////////////////////////////////////////////////
// The following arapb_misc_utmip.spec fields need to be programmed to ensure
// correct operation of the UTMIP block:
// Production settings :
//        'HS_SYNC_START_DLY' : 9,
//        'IDLE_WAIT'         : 17,
//        'ELASTIC_LIMIT'     : 16,
// All other fields can use the default reset values.
// Setting the fields above, together with default values of the other fields,
// results in programming the registers below as follows:
//         UTMIP_HSRX_CFG0 = 0x9168c000
//         UTMIP_HSRX_CFG1 = 0x13
///////////////////////////////////////////////////////////////////////////////
//UTMIP Idle Wait Delay
static const NvU8 s_UtmipIdleWaitDelay    = 17;
//UTMIP Elastic limit
static const NvU8 s_UtmipElasticLimit     = 16;
//UTMIP High Speed Sync Start Delay
static const NvU8 s_UtmipHsSyncStartDelay = 0;

static void
Ap20UsbPhyConfigUsbClockOverOn(
    NvDdkUsbPhy *pUsbPhy,
    NvBool EnableOverOn)
{
    NvU32 RegVal = 0x0;

    RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                 CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0);
    switch (pUsbPhy->Instance)
    {
        case 0:
            RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRB,
                USB1_CLK_OVR_ON, EnableOverOn, RegVal);
        break;
        case 1:
            RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRB,
                USB2_CLK_OVR_ON, EnableOverOn, RegVal);
        break;
        case 2:
            RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRB,
                USB3_CLK_OVR_ON, EnableOverOn, RegVal);
        break;
        default:
            NV_ASSERT(!"Unable to enable/disable USB Clock Over On");
            return;
    }
    NV_REGW(pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                 CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0, RegVal);

}

static NvError
Ap20UsbPhyWaitForInValidPhyClock(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;

    // Wait for the phy clock to become in valid or hardware timeout
    do {
        if (!USB_IF_REG_READ_VAL(SUSP_CTRL, USB_PHY_CLK_VALID))
            return NvSuccess;
        NvOsWaitUS(1);
        TimeOut--;
    } while (TimeOut);

    NvOsDebugPrintf("Phy Clock is not stopped and timed out\n");

    return NvError_Timeout;
}


static void
Ap20UsbPhyUlpiViewPortProgramData(
    NvDdkUsbPhy *pUsbPhy,
    NvU32 Addr,
    NvU32 Data)
{
    NvU32 RegVal = 0x0;
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;

    RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_WAKEUP, CLEAR, RegVal);
    RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_RUN, SET, RegVal);
    RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_RD_WR, WRITE, RegVal);
    RegVal = USB_FLD_SET_DRF_NUM(ULPI_VIEWPORT, ULPI_PORT, 0, RegVal);
    RegVal = USB_FLD_SET_DRF_NUM(ULPI_VIEWPORT, ULPI_REG_ADDR, Addr, RegVal);
    RegVal = USB_FLD_SET_DRF_NUM(ULPI_VIEWPORT, ULPI_DATA_WR, Data, RegVal);
    USB_REG_WR(ULPI_VIEWPORT, RegVal);

    // wait for run bit to be cleared
    do
    {
        RegVal = USB_REG_RD(ULPI_VIEWPORT);
        if (!TimeOut)
        {
            NvOsDebugPrintf(" !!!Error in accessing ULPI !!! \n");
            return;
        }
        NvOsWaitUS(1);
        TimeOut--;
    } while (USB_DRF_VAL(ULPI_VIEWPORT, ULPI_RUN, RegVal));
}

static NvError
Ap20UsbPhySuspendPort(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0x0;
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;

    // Set the SUSP bit (7:7) in the PORTSC1 reg to suspend the port.
    //This bit is RW in host mode only.
    if (pUsbPhy->pProperty->UsbMode != NvOdmUsbModeType_Host)
    {
        return NvError_NotSupported;
    }

    // If port power(PP) is 0, SUSP bit is 0. We should not try to set it to 1.
    if (!USB_REG_READ_VAL(PORTSC1, PP))
    {
        return NvError_NotSupported;
    }

    // Do nothing if port is disabled
    if (!USB_REG_READ_VAL(PORTSC1, PE))
    {
        return NvError_NotSupported;
    }

    RegVal = USB_REG_RD(PORTSC1);
    RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, SUSP, SUSPEND, RegVal);
    USB_REG_WR(PORTSC1, RegVal);
    // Wait until port suspend completes
    do
    {
        if (USB_REG_READ_VAL(PORTSC1, SUSP))
        {
            return NvSuccess;
        }

        NvOsWaitUS(1);
        TimeOut--;
    } while (TimeOut);

    return NvError_Timeout;
}

static NvError
Ap20UsbPhyUtmiConfigure(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;
    NvRmFreqKHz OscFreqKz = 0;
    NvU32 FreqIndex;

    // Get the Oscillator Frequency
    OscFreqKz = NvRmPowerGetPrimaryFrequency(pUsbPhy->hRmDevice);

    // Get the Oscillator Frequency Index
    for (FreqIndex = 0; FreqIndex < NvRmClocksOscFreq_Num; FreqIndex++)
    {
        if (OscFreqKz == s_RmOscFrequecy[FreqIndex])
        {
            // Bail Out if frequecy matches with the supported frequency
            break;
        }
    }
    // If Index is equal to the maximum supported frequency count
    // There is a mismatch of the frequecy, so returning since the
    // frequency is not supported.
    if (FreqIndex >= NvRmClocksOscFreq_Num)
    {
        return NvError_NotSupported;
    }

    // Hold UTMIP in reset
    RegVal =USB_IF_REG_RD(SUSP_CTRL);
    RegVal = USB3_IF_FLD_SET_DRF_DEF(SUSP_CTRL, UTMIP_RESET, ENABLE, RegVal);
    USB_IF_REG_WR(SUSP_CTRL, RegVal);

    // Change the USB1 to non-legacy mode
    if (pUsbPhy->Instance == 0)
    {
        RegVal =USB1_IF_REG_RD(USB1_LEGACY_CTRL);
        RegVal = USB1_IF_FLD_SET_DRF_DEF(
                    USB1_LEGACY_CTRL, USB1_NO_LEGACY_MODE, NEW, RegVal);
        USB1_IF_REG_WR(USB1_LEGACY_CTRL, RegVal);
    }

    // Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
    // Setting these fields, together with default values of the other
    // fields, results in programming the registers below as follows:
    //         UTMIP_HSRX_CFG0 = 0x9168c000
    //         UTMIP_HSRX_CFG1 = 0x13
    RegVal = USB_UTMIP_REG_RD(HSRX_CFG0);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(HSRX_CFG0, 
                UTMIP_IDLE_WAIT,s_UtmipIdleWaitDelay, RegVal);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(HSRX_CFG0, 
                UTMIP_ELASTIC_LIMIT,s_UtmipElasticLimit, RegVal);
    USB_UTMIP_REG_WR(HSRX_CFG0, RegVal);

    // Configure the UTMIP_HS_SYNC_START_DLY
    USB_UTMIP_REG_UPDATE_NUM(
        HSRX_CFG1, UTMIP_HS_SYNC_START_DLY, s_UtmipHsSyncStartDelay);

    // Program 1ms Debounce time for VBUS to become valid.
    USB_UTMIP_REG_UPDATE_NUM(
        DEBOUNCE_CFG0, UTMIP_BIAS_DEBOUNCE_A, s_UsbBiasDebounceATime[FreqIndex]);

    // PLL Delay CONFIGURATION settings
    // The following parameters control the bring up of the plls:
    USB_UTMIP_REG_UPDATE_NUM(MISC_CFG0, UTMIP_SUSPEND_EXIT_ON_EDGE, 0);

    RegVal = USB_UTMIP_REG_RD(MISC_CFG1);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(MISC_CFG1, UTMIP_PLLU_STABLE_COUNT,
                s_UsbPllDelayParams[FreqIndex].StableCount, RegVal);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(MISC_CFG1, UTMIP_PLL_ACTIVE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].ActiveDelayCount, RegVal);
    USB_UTMIP_REG_WR(MISC_CFG1, RegVal);

    // Set PLL enable delay count and Crystal frequency count

    RegVal = USB_UTMIP_REG_RD(PLL_CFG1);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(PLL_CFG1, UTMIP_PLLU_ENABLE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].EnableDelayCount, RegVal);
    RegVal = USB_UTMIP_FLD_SET_DRF_DEF(PLL_CFG1, UTMIP_XTAL_FREQ_COUNT,
                s_UsbPllDelayParams[FreqIndex].XtalFreqCount, RegVal);
    USB_UTMIP_REG_WR(PLL_CFG1, RegVal);

    return NvSuccess;
}

static void
Ap20UsbPhyUtmiPowerControl(
    NvDdkUsbPhy *pUsbPhy,
    NvBool Enable)
{
    NvU32 RegVal = 0;

    /* UTMIP_XCVR_SETUP setting of 0x9, in conjunction with
       UTMIP_XCVR_TERM_RANGE_ADJ of 0x6, gives the maximum guard band around
       the USB electrical spec. This is true across fast and slow chips, high
       and low voltage, and hot and cold temperature. */
    NvU32 XcvrSetupValue = 0x9;
    NvU32 XcvrTermRangeAdj = 0x6;

    if (Enable)
    {
        if (pUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Device)
        {
            // Disable the automatic phy enable on wakeup event
            USB1_IF_REG_UPDATE_DEF(USB_SUSP_CTRL, USB_WAKE_ON_CNNT_EN_DEV, DISABLE);
        }

        // USB Power Up sequence
        if (!pUsbPhy->pUtmiPadConfig->PadOnRefCount)
        {
            /* UTMI PAD control logic is common to all UTMIP phys and 
               they are controlled from USB1 controller */
            // Power Up OTG and Bias config pad circuitry
            RegVal = NV_READ32(pUsbPhy->pUtmiPadConfig->pVirAdr + ((USB1_UTMIP_BIAS_CFG0_0)/4));
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(BIAS_CFG0, UTMIP_OTGPD, 0, RegVal);
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(BIAS_CFG0, UTMIP_BIASPD, 0, RegVal);
            NV_WRITE32(pUsbPhy->pUtmiPadConfig->pVirAdr + ((USB1_UTMIP_BIAS_CFG0_0)/4), 
                       RegVal);
        }
        /* Increment the reference count to turn off the UTMIP pads */
        pUsbPhy->pUtmiPadConfig->PadOnRefCount++;

        // Turn on power in the tranciver
        RegVal = USB_UTMIP_REG_RD(XCVR_CFG0);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PDZI_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PD2_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PD_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_XCVR_SETUP, XcvrSetupValue, RegVal);
        if (pUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host)
        {
            // To slow rise/ fall times in low-speed eye diagrams in host mode
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                        XCVR_CFG0, UTMIP_XCVR_LSFSLEW, 2, RegVal);
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                        XCVR_CFG0, UTMIP_XCVR_LSRSLEW, 2, RegVal);
        }
        else
        {
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                        XCVR_CFG0, UTMIP_XCVR_HSSLEW_MSB, 0, RegVal);
        }

        USB_UTMIP_REG_WR(XCVR_CFG0, RegVal);

        RegVal = USB_UTMIP_REG_RD(XCVR_CFG1);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDDISC_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDCHRP_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDDR_POWERDOWN, 0, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_XCVR_TERM_RANGE_ADJ, XcvrTermRangeAdj, RegVal);
        USB_UTMIP_REG_WR(XCVR_CFG1, RegVal);

        // Enable Batery charge enabling bit, set to '0' for enable
        USB_UTMIP_REG_UPDATE_NUM(BAT_CHRG_CFG0, UTMIP_PD_CHRG, 0);

        if (pUsbPhy->Instance == 2)
        {
            // Enable UTMIP PHY for USB3 controller
            RegVal = USB_IF_REG_RD(SUSP_CTRL);
            RegVal = USB3_IF_FLD_SET_DRF_DEF(
                        SUSP_CTRL, UTMIP_PHY_ENB, ENABLE, RegVal);
            USB_IF_REG_WR(SUSP_CTRL, RegVal);
        }

        // Release reset to UTMIP
        RegVal =USB_IF_REG_RD(SUSP_CTRL);
        RegVal = USB3_IF_FLD_SET_DRF_DEF(SUSP_CTRL, UTMIP_RESET, DISABLE, RegVal);
        USB_IF_REG_WR(SUSP_CTRL, RegVal);

        if (pUsbPhy->Instance == 0)
        {
            RegVal = USB1_IF_REG_RD(USB1_LEGACY_CTRL);
            RegVal = USB1_IF_FLD_SET_DRF_DEF(
                        USB1_LEGACY_CTRL, USB1_VBUS_SENSE_CTL, A_SESS_VLD, RegVal);
            USB1_IF_REG_WR(USB1_LEGACY_CTRL, RegVal);
        }
        if (pUsbPhy->Instance == 0)
        {
            USB1_IF_REG_UPDATE_DEF(USB_SUSP_CTRL, USB_SUSP_SET, UNSET);
        }
        USB_IF_REG_UPDATE_DEF(SUSP_CTRL, USB_SUSP_CLR, SET);
        NvOsWaitUS(10);
        USB_IF_REG_UPDATE_DEF(SUSP_CTRL, USB_SUSP_CLR, UNSET);
    }
    else
    {
        // Put the Phy in the suspend mode
        if (pUsbPhy->Instance == 0)
        {
            USB1_IF_REG_UPDATE_DEF(USB_SUSP_CTRL, USB_SUSP_SET, SET);
            NvOsWaitUS(10);
            USB1_IF_REG_UPDATE_DEF(USB_SUSP_CTRL, USB_SUSP_SET, UNSET);
        }

        // Put the Phy in the suspend mode
        if (pUsbPhy->Instance == 2)
        {
            // Suspend port before setting PHCD bit
            Ap20UsbPhySuspendPort(pUsbPhy);
            RegVal = USB_REG_RD(PORTSC1);
            RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, PHCD, ENABLE, RegVal); 
            USB_REG_WR(PORTSC1, RegVal);
        }

        // wait untill phy in suspend..
        Ap20UsbPhyWaitForInValidPhyClock(pUsbPhy);

        if (pUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Device)
        {
            // Setup debounce time for wakeup event 5 HCLK cycles
            USB_IF_REG_UPDATE_NUM(SUSP_CTRL, USB_WAKEUP_DEBOUNCE_COUNT, 5);
            // ENABLE the automatic phy enable on wakeup event
            USB1_IF_REG_UPDATE_DEF(USB_SUSP_CTRL, USB_WAKE_ON_CNNT_EN_DEV, ENABLE);
        }
        // Hold UTMIP in reset
        RegVal =USB_IF_REG_RD(SUSP_CTRL);
        RegVal = USB3_IF_FLD_SET_DRF_DEF(SUSP_CTRL, UTMIP_RESET, ENABLE, RegVal);
        USB_IF_REG_WR(SUSP_CTRL, RegVal);

        // Disable Batery charge enabling bit set to '1' for disable
        USB_UTMIP_REG_UPDATE_NUM(BAT_CHRG_CFG0, UTMIP_PD_CHRG, 1);

        // Turn off power in the tranciver
        RegVal = USB_UTMIP_REG_RD(XCVR_CFG0);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PDZI_POWERDOWN, 1, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PD2_POWERDOWN, 1, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG0, UTMIP_FORCE_PD_POWERDOWN, 1, RegVal);
        USB_UTMIP_REG_WR(XCVR_CFG0, RegVal);

        RegVal = USB_UTMIP_REG_RD(XCVR_CFG1);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDDISC_POWERDOWN, 1, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDCHRP_POWERDOWN, 1, RegVal);
        RegVal = USB_UTMIP_FLD_SET_DRF_DEF(
                    XCVR_CFG1, UTMIP_FORCE_PDDR_POWERDOWN, 1, RegVal);
        USB_UTMIP_REG_WR(XCVR_CFG1, RegVal);


        // USB Power down sequence
        /* decrement the pad control refernce count */
        pUsbPhy->pUtmiPadConfig->PadOnRefCount--;
        if (!pUsbPhy->pUtmiPadConfig->PadOnRefCount)
        {
            /* since there is no reference to the pads turn off */
            RegVal = NV_READ32(pUsbPhy->pUtmiPadConfig->pVirAdr + ((USB1_UTMIP_BIAS_CFG0_0)/4));
            // Power down OTG and Bias circuitry
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(BIAS_CFG0, UTMIP_OTGPD, 1, RegVal);
            RegVal = USB_UTMIP_FLD_SET_DRF_DEF(BIAS_CFG0, UTMIP_BIASPD, 1, RegVal);
            NV_WRITE32(pUsbPhy->pUtmiPadConfig->pVirAdr + ((USB1_UTMIP_BIAS_CFG0_0)/4),
                       RegVal);
        }
    }
}

static void 
Ap20UsbPhySelectUsbMode(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;

    if ((pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi) &&
        (pUsbPhy->Instance == 2))
    {
        // Disable ICUSB interface 
        USB_REG_UPDATE_DEF(ICUSB_CTRL, IC_ENB1, DISABLE);

        // set UTMIP PHY and parallel UTMI interface
        RegVal = USB_REG_RD(PORTSC1);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, PTS, UTMI, RegVal); 
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, STS, PARALLEL_IF, RegVal); 
        USB_REG_WR(PORTSC1, RegVal);
    }
}


static void 
Ap20UsbPhyUlpiNullModeConfigure(
    NvDdkUsbPhy *pUsbPhy)
{
    // default trimmer values for ap20
    NvOdmUsbTrimmerCtrl trimmerCtrl = {0, 0, 4, 4};

    if (pUsbPhy->pProperty->TrimmerCtrl.UlpiShadowClkDelay ||
        pUsbPhy->pProperty->TrimmerCtrl.UlpiClockOutDelay ||
        pUsbPhy->pProperty->TrimmerCtrl.UlpiDataTrimmerSel ||
        pUsbPhy->pProperty->TrimmerCtrl.UlpiStpDirNxtTrimmerSel)
    {
        // update the trimmer values if they are specified in nvodm_query
        NvOsMemcpy(&trimmerCtrl, &pUsbPhy->pProperty->TrimmerCtrl, sizeof(NvOdmUsbTrimmerCtrl));
    }

    // Put the UHSIC in the reset
    USB_IF_REG_UPDATE_DEF(SUSP_CTRL, UHSIC_RESET, ENABLE);

    // Bypass the Pin Mux on the ULPI outputs and ULPI clock output enable
    ULPI_IF_REG_WR(TIMING_CTRL_0, 
                ULPI_IF_REG_RD(TIMING_CTRL_0) |
                ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_OUTPUT_PINMUX_BYP, ENABLE) |
                ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_CLKOUT_PINMUX_BYP, ENABLE));

    // Enable ULPI clock from clk_gen
    USB_IF_REG_UPDATE_DEF(SUSP_CTRL, ULPI_PHY_ENB, ENABLE);

    // Set the timming perameters
    ULPI_IF_REG_WR(TIMING_CTRL_0, 
                    ULPI_IF_REG_RD(TIMING_CTRL_0) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_SHADOW_CLK_LOOPBACK_EN, ENABLE) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_SHADOW_CLK_SEL, POST_PAD) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_OUTPUT_PINMUX_BYP, ENABLE) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_CLKOUT_PINMUX_BYP, ENABLE) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_LBK_PAD_EN, OUTPUT) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_0, ULPI_SHADOW_CLK_DELAY, trimmerCtrl.UlpiShadowClkDelay) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_0, ULPI_CLOCK_OUT_DELAY, trimmerCtrl.UlpiClockOutDelay) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_0, ULPI_LBK_PAD_E_INPUT_OR, 0));

    // Set all the trimmers to 0 at the start
    ULPI_IF_REG_WR(TIMING_CTRL_1, 0);

    // wait for 10 micro seconds
    NvOsWaitUS(10);

    // Set USB2 controller to null ulpi mode (ULPIS2S_ENA = 1)
    // Enable PLLU (ULPIS2S_PLLU_MASTER_BLASTER60 = 1)
    //  ULPIS2S_SPARE[0] = VBUS active
    USB_IF_REG_WR(ULPIS2S_CTRL, 
                    USB_IF_DRF_DEF(ULPIS2S_CTRL, ULPIS2S_ENA, ENABLE) |
                    USB_IF_DRF_DEF(ULPIS2S_CTRL, ULPIS2S_PLLU_MASTER_BLASTER60, ENABLE) |
                    USB_IF_DRF_NUM(ULPIS2S_CTRL, ULPIS2S_SPARE,
                    (pUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host)?3:1));

    // Select ULPI_CORE_CLK_SEL to SHADOW_CLK
    ULPI_IF_REG_WR(TIMING_CTRL_0, 
                    ULPI_IF_REG_RD(TIMING_CTRL_0) |
                    ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_CORE_CLK_SEL, SHADOW_CLK));

    // wait for 10 micro seconds
    NvOsWaitUS(10);

    // Set ULPI_CLK_OUT_ENA to 1 to enable ULPI null clocks
    // Can't set the trimmers before this
    ULPI_IF_REG_WR(TIMING_CTRL_0, 
                    ULPI_IF_REG_RD(TIMING_CTRL_0) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_0, ULPI_CLK_OUT_ENA, 1));

    // wait for 10 micro seconds
    NvOsWaitUS(10);

    // Set the trimmer values
    ULPI_IF_REG_WR(TIMING_CTRL_1, 
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_LOAD, 0) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_SEL, trimmerCtrl.UlpiDataTrimmerSel) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_LOAD, 0) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_SEL, trimmerCtrl.UlpiStpDirNxtTrimmerSel) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_LOAD, 0) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_SEL, 4));

    // wait for 10 micro seconds
    NvOsWaitUS(10);

    //Load the trimmers by toggling the load bits
    ULPI_IF_REG_WR(TIMING_CTRL_1, 
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_LOAD, 1) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_SEL, trimmerCtrl.UlpiDataTrimmerSel) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_LOAD, 1) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_SEL, trimmerCtrl.UlpiStpDirNxtTrimmerSel) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_LOAD, 1) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_SEL, 4));

    // wait for 10 micro seconds
    NvOsWaitUS(10);

    // Set ULPI_CLK_PADOUT_ENA  to 1 to enable ULPI null clock going out to
    // ulpi clk pad (ulpi_clk)
    ULPI_IF_REG_WR(TIMING_CTRL_0, 
                    ULPI_IF_REG_RD(TIMING_CTRL_0) |
                    ULPI_IF_DRF_NUM(TIMING_CTRL_0, ULPI_CLK_PADOUT_ENA, 1));
}


static void 
Ap20UsbPhyUlpiLinkModeConfigure(
    NvDdkUsbPhy *pUsbPhy, NvBool Enable)
{
    NvU32 RegVal;
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;

    if (Enable)
    {
        // Put the UHSIC in the reset
        USB_IF_REG_UPDATE_DEF(SUSP_CTRL, UHSIC_RESET, ENABLE);

        // Bypass the Pin Mux on the ULPI outputs and ULPI clock output enable
        ULPI_IF_REG_WR(TIMING_CTRL_0, 
            ULPI_IF_REG_RD(TIMING_CTRL_0) |
            ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_OUTPUT_PINMUX_BYP, ENABLE) |
            ULPI_IF_DRF_DEF(TIMING_CTRL_0, ULPI_CLKOUT_PINMUX_BYP, ENABLE));

        // Enable ULPI clock from clk_gen
        USB_IF_REG_UPDATE_DEF(SUSP_CTRL, ULPI_PHY_ENB, ENABLE);

        // Set all the trimmers to 0 at the start
        ULPI_IF_REG_WR(TIMING_CTRL_1, 0);

        // Set the trimmer values
        ULPI_IF_REG_WR(TIMING_CTRL_1, 
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_LOAD, 0) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_SEL, 4) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_LOAD, 0) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_SEL, 4) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_LOAD, 0) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_SEL, 4));

        // wait for 10 micro seconds
        NvOsWaitUS(10);

        //Load the trimmers by toggling the load bits
        ULPI_IF_REG_WR(TIMING_CTRL_1, 
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_LOAD, 1) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DATA_TRIMMER_SEL, 4) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_LOAD, 1) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_STPDIRNXT_TRIMMER_SEL, 4) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_LOAD, 1) |
            ULPI_IF_DRF_NUM(TIMING_CTRL_1, ULPI_DIR_TRIMMER_SEL, 4));

        RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_WAKEUP, SET, 0xFFFFFFFF);
        RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_RUN, CLEAR, RegVal);
        RegVal = USB_FLD_SET_DRF_DEF(ULPI_VIEWPORT, ULPI_RD_WR, WRITE, RegVal);
        RegVal = USB_FLD_SET_DRF_NUM(ULPI_VIEWPORT, ULPI_PORT, 0, RegVal);
        USB_REG_WR(ULPI_VIEWPORT, RegVal);

        // wait for ULPI wake up is cleared
        do {
            RegVal = USB_REG_RD(ULPI_VIEWPORT);
            if (!TimeOut)
            {
                NvOsDebugPrintf("Error accessing ULPI view port\n");
                break;
            }
            NvOsWaitUS(1);
            TimeOut--;
        } while (USB_DRF_VAL(ULPI_VIEWPORT, ULPI_WAKEUP, RegVal));

        // fix VbusValid for Harmony due to floating VBUS
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x08, 0x40);

        // set UseExternalVbusIndicator to 1 
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x0B, 0x80);

        // enabling WKCN/WKDS/WKOC bit
        RegVal = USB_REG_RD(PORTSC1);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKCN, ENABLE, RegVal);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKDS, ENABLE, RegVal);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKOC, ENABLE, RegVal);
        USB_REG_WR(PORTSC1, RegVal);
    }
    else
    {
        // Programming  the ULPI register function control
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x04, 0x4D);

        // Resetting  the ULPI register IndicatorPassThru
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x09, 0x40);

        // USB Interrupt Rising - making sure vbus comparator and id are off
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x0D, 0x00);

        // USB Interrupt Falling
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x10, 0x00);

        //Carkit Control
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x19, 0x00);

        // Disabling ID float Rise/Fall (Carkit Enable)
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x1D, 0x00);

        // USB I/O and power
        Ap20UsbPhyUlpiViewPortProgramData(pUsbPhy, 0x39, 0x00);

        // clear WKCN/WKDS/WKOC wake-on events that can cause the USB
        // Controller to immediately bring the ULPI PHY out of low power mode
        RegVal = USB_REG_RD(PORTSC1);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKCN, DISBLE, RegVal);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKDS, DISBLE, RegVal);
        RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, WKOC, DISBLE, RegVal);
        USB_REG_WR(PORTSC1, RegVal);
    }
}

static void
Ap20UsbPhyUlpiPowerControl(
    NvDdkUsbPhy *pUsbPhy,
    NvBool Enable)
{
    NvU32 RegVal = 0;

    if (pUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host)
    {
        if (Enable)
        {
            // Bring the Phy out of suspend mode
            USB_IF_REG_UPDATE_DEF(SUSP_CTRL, USB_SUSP_CLR, SET);
            NvOsWaitUS(100);
            USB_IF_REG_UPDATE_DEF(SUSP_CTRL, USB_SUSP_CLR, UNSET);
        }
        else
        {
            // Put the Phy in the suspend mode
            // Suspend port before setting PHCD bit
            Ap20UsbPhySuspendPort(pUsbPhy);
            RegVal = USB_REG_RD(PORTSC1);
            RegVal = USB_FLD_SET_DRF_DEF(PORTSC1, PHCD, ENABLE, RegVal); 
            USB_REG_WR(PORTSC1, RegVal);
            // Wait for phy clock to go down
            Ap20UsbPhyWaitForInValidPhyClock(pUsbPhy);
        }
    }
}


static NvError 
Ap20UsbPhyIoctlVbusInterrupt(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    NvDdkUsbPhyIoctl_VBusInterruptInputArgs *pVbusIntr = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pVbusIntr = (NvDdkUsbPhyIoctl_VBusInterruptInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);

        if (pVbusIntr->EnableVBusInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, 
                                        VBUS_WAKEUP_WAKEUP_EN, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, 
                                        VBUS_WAKEUP_INT_EN, ENABLE, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, 
                                        VBUS_WAKEUP_WAKEUP_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        VBUS_WAKEUP_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        VBUS_WAKEUP_CHG_DET, SET, RegVal);
        }

        USB1_IF_REG_WR(USB_PHY_VBUS_WAKEUP_ID, RegVal);
    }
    else if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_UlpiExternalPhy)
    {
        if (pVbusIntr->EnableVBusInterrupt)
        {
            USB_REG_WR(USBINTR, USB_DRF_DEF(USBINTR, UE, ENABLE));
            RegVal = USB_REG_SET_DRF_DEF(OTGSC, BSVIE, ENABLE);
            USB_REG_WR(OTGSC, RegVal);
        }
        else
        {
            USB_REG_WR(USBSTS, USB_REG_RD(USBSTS));
            RegVal = USB_REG_SET_DRF_DEF(OTGSC, BSVIE, DISABLE);
            USB_REG_WR(OTGSC, RegVal);
        }
    }
    else
    {
        // In NULL phy mode this is not required as VBUS is always present
    }


    return NvSuccess;
}

static NvError 
Ap20UsbPhyIoctlVBusStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_VBusStatusOutputArgs *pVBusStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pVBusStatus = (NvDdkUsbPhyIoctl_VBusStatusOutputArgs *)pOutputArgs;

    pVBusStatus = (NvDdkUsbPhyIoctl_VBusStatusOutputArgs *)pOutputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);
        if (NV_DRF_VAL(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, VBUS_WAKEUP_STS, RegVal))
        {
            pVBusStatus->VBusDetected = NV_TRUE;
        }
        else
        {
            pVBusStatus->VBusDetected = NV_FALSE;
        }
    }
    else if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_UlpiExternalPhy)
    {
        if (USB_REG_READ_VAL(OTGSC, BSV))
        {
            pVBusStatus->VBusDetected = NV_TRUE;
        }
        else
        {
            pVBusStatus->VBusDetected = NV_FALSE;
        }
    }
    else
    {
        // In NULL phy mode VBUS is always present
        pVBusStatus->VBusDetected = NV_TRUE;
    }

    return NvSuccess;
}

static NvError 
Ap20UsbPhyIoctlIdPinInterrupt(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    NvDdkUsbPhyIoctl_IdPinInterruptInputArgs *pIdPinIntr = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pIdPinIntr = (NvDdkUsbPhyIoctl_IdPinInterruptInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);

        if (pIdPinIntr->EnableIdPinInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, 
                                        ID_PU, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, 
                                        ID_INT_EN, ENABLE, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        ID_PU, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        ID_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        ID_CHG_DET, SET, RegVal);
        }
        USB1_IF_REG_WR(USB_PHY_VBUS_WAKEUP_ID, RegVal);
    }
    else
    {
        //TODO:ENable the ID pin interrupt for the ULPI
    }

    return NvSuccess;
}

static NvError 
Ap20UsbPhyIoctlIdPinStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_IdPinStatusOutputArgs *pIdPinStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pIdPinStatus = (NvDdkUsbPhyIoctl_IdPinStatusOutputArgs *)pOutputArgs;

    pIdPinStatus = (NvDdkUsbPhyIoctl_IdPinStatusOutputArgs *)pOutputArgs;

    RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);
    if (!NV_DRF_VAL(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, ID_STS, RegVal))
    {
        pIdPinStatus->IdPinSetToLow = NV_TRUE;
    }
    else
    {
        pIdPinStatus->IdPinSetToLow = NV_FALSE;
    }

    return NvSuccess;
}

static NvError 
Ap20UsbPhyIoctlDedicatedChargerDetection(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    // These values (in milli second) are taken from the battery charging spec.
    #define TDP_SRC_ON_MS    100
    #define TDPSRC_CON_MS    40
    NvDdkUsbPhyIoctl_DedicatedChargerDetectionInputArgs *pChargerDetection = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pChargerDetection = (NvDdkUsbPhyIoctl_DedicatedChargerDetectionInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType != NvOdmUsbInterfaceType_Utmi)
    {
        // Charger detection is not there for ULPI
        return NvSuccess;
    }

    if (pChargerDetection->EnableChargerDetection)
    {
        // Enable charger detection logic
        RegVal = USB_UTMIP_REG_RD(BAT_CHRG_CFG0);
        RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, BAT_CHRG_CFG0,
                                    UTMIP_OP_SRC_EN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, BAT_CHRG_CFG0,
                                   UTMIP_ON_SINK_EN, 1, RegVal);
        USB_UTMIP_REG_WR(BAT_CHRG_CFG0, RegVal);

        // Source should be on for 100 ms as per USB charging spec
        NvOsSleepMS(TDP_SRC_ON_MS);

        RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);
        if (pChargerDetection->EnableChargerInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                       VDAT_DET_INT_EN, ENABLE, RegVal);
        }
        else
        {
            // If charger is not connected disable the interrupt
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                       VDAT_DET_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                        VDAT_DET_CHG_DET, SET, RegVal);
        }
        USB1_IF_REG_WR(USB_PHY_VBUS_WAKEUP_ID, RegVal);
    }
    else
    {
        // disable the interrupt
        RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);
        RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                    VDAT_DET_INT_EN, DISABLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_PHY_VBUS_WAKEUP_ID,
                                    VDAT_DET_CHG_DET, SET, RegVal);
        USB1_IF_REG_WR(USB_PHY_VBUS_WAKEUP_ID, RegVal);

        // Disable charger detection logic
        RegVal = USB_UTMIP_REG_RD(BAT_CHRG_CFG0);
        RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, BAT_CHRG_CFG0,
                                    UTMIP_OP_SRC_EN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, BAT_CHRG_CFG0,
                                    UTMIP_ON_SINK_EN, 0, RegVal);
        USB_UTMIP_REG_WR(BAT_CHRG_CFG0, RegVal);

        // Delay of 40 ms before we pull the D+ as per battery charger spec.
        NvOsSleepMS(TDPSRC_CON_MS);
    }

    return NvSuccess;
}

static NvError 
Ap20UsbPhyIoctlDedicatedChargerStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_DedicatedChargerStatusOutputArgs *pChargerStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pChargerStatus = (NvDdkUsbPhyIoctl_DedicatedChargerStatusOutputArgs *)pOutputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType != NvOdmUsbInterfaceType_Utmi)
    {
        // Charger detection is not there for ULPI
        // return Charger not available
        pChargerStatus->ChargerDetected = NV_FALSE;
        return NvSuccess;
    }

    RegVal = USB1_IF_REG_RD(USB_PHY_VBUS_WAKEUP_ID);
    if (NV_DRF_VAL(USB1_IF, USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_STS, RegVal))
    {
        pChargerStatus->ChargerDetected = NV_TRUE;
    }
    else
    {
        pChargerStatus->ChargerDetected = NV_FALSE;
    }

    return NvSuccess;
}


static NvError 
Ap20UsbPhyIoctl(
    NvDdkUsbPhy *pUsbPhy,
    NvDdkUsbPhyIoctlType IoctlType,
    const void *pInputArgs,
    void *pOutputArgs)
{
    NvError ErrStatus = NvSuccess;

    switch (IoctlType)
    {
        case NvDdkUsbPhyIoctlType_VBusStatus:
            ErrStatus = Ap20UsbPhyIoctlVBusStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_VBusInterrupt:
            ErrStatus = Ap20UsbPhyIoctlVbusInterrupt(pUsbPhy, pInputArgs);
            break;
        case NvDdkUsbPhyIoctlType_IdPinStatus:
            ErrStatus = Ap20UsbPhyIoctlIdPinStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_IdPinInterrupt:
            ErrStatus = Ap20UsbPhyIoctlIdPinInterrupt(pUsbPhy, pInputArgs);
            break;
        case NvDdkUsbPhyIoctlType_DedicatedChargerStatus:
            ErrStatus = Ap20UsbPhyIoctlDedicatedChargerStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_DedicatedChargerDetection:
            ErrStatus = Ap20UsbPhyIoctlDedicatedChargerDetection(pUsbPhy, pInputArgs);
            break;
        default:
            return NvError_NotSupported;
    }

    return ErrStatus;
}

static NvError 
Ap20UsbPhyWaitForStableClock(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;
    NvU32 PhyClkValid = 0;

    // Wait for the phy clock to become valid or hardware timeout
    do {
        PhyClkValid = USB_IF_REG_READ_VAL(SUSP_CTRL, USB_PHY_CLK_VALID);
        if (!TimeOut)
        {
            return NvError_Timeout;
        }
        NvOsWaitUS(1);
        TimeOut--;
    } while (PhyClkValid != USB3_IF_USB_SUSP_CTRL_0_USB_PHY_CLK_VALID_SET);

    return NvSuccess;
}

static void Ap20UsbPhyMemoryPrefetch(NvDdkUsbPhy* pUsbPhy, NvBool Enable)
{
    NvU32 regVal = 0;

    if (Enable)
    {
        // Setup the AHB MEM configuration for USB performance.
        // Enabling the AHB prefetch bits for USB1.
        // 64kiloByte boundaries.
        // 4096 cycles before prefetched data is invalidated due to inactivity.
        regVal = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, ENABLE, 1) |
              NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG1, AHB_MST_ID, AHBDMA)|
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, ADDR_BNDRY, 0xC) |
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, SPEC_THROTTLE, 0x0) |
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, INACTIVITY_TIMEOUT, 0x1000);
        NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
             AHB_AHB_MEM_PREFETCH_CFG1_0, regVal);

        regVal = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, ENABLE, 1) |
              NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG2, AHB_MST_ID, USB)|
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, ADDR_BNDRY, 0xC) |
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, SPEC_THROTTLE, 0x0) |
              NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, INACTIVITY_TIMEOUT, 0x1000);
        NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
                 AHB_AHB_MEM_PREFETCH_CFG2_0, regVal);
    }
    else
    {
        regVal = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, ENABLE, 0);
        NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
                 AHB_AHB_MEM_PREFETCH_CFG1_0, regVal);

        regVal = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, ENABLE, 0);
        NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
                 AHB_AHB_MEM_PREFETCH_CFG2_0, regVal);
    }
}

static NvError 
Ap20UsbPhyPowerUp(
    NvDdkUsbPhy *pUsbPhy)
{
    NvError ErrVal = NvSuccess;

    Ap20UsbPhyConfigUsbClockOverOn(pUsbPhy, NV_TRUE);

    switch (pUsbPhy->pProperty->UsbInterfaceType)
    {
        case NvOdmUsbInterfaceType_UlpiNullPhy:
            Ap20UsbPhyUlpiNullModeConfigure(pUsbPhy);
            Ap20UsbPhyUlpiPowerControl(pUsbPhy, NV_TRUE);
            break;
        case NvOdmUsbInterfaceType_UlpiExternalPhy:
            pUsbPhy->hOdmUlpi = NvOdmUsbUlpiOpen(pUsbPhy->Instance);
            Ap20UsbPhyUlpiLinkModeConfigure(pUsbPhy, NV_TRUE);
            Ap20UsbPhyUlpiPowerControl(pUsbPhy, NV_TRUE);
            break;
        case NvOdmUsbInterfaceType_Utmi:
            ErrVal = Ap20UsbPhyUtmiConfigure(pUsbPhy);
            if (ErrVal != NvSuccess)
                return ErrVal;
            Ap20UsbPhyUtmiPowerControl(pUsbPhy, NV_TRUE);
            default:
            break;
    }

    ErrVal = Ap20UsbPhyWaitForStableClock(pUsbPhy);

    if (ErrVal == NvSuccess)
    {
        Ap20UsbPhySelectUsbMode(pUsbPhy);

        if (pUsbPhy->pProperty->UsbInterfaceType ==
            NvOdmUsbInterfaceType_UlpiNullPhy)
            pUsbPhy->hOdmUlpi = NvOdmUsbUlpiOpen(pUsbPhy->Instance);
    }

    return ErrVal;
}

static NvError 
Ap20UsbPhyPowerDown(
    NvDdkUsbPhy *pUsbPhy)
{
    switch (pUsbPhy->pProperty->UsbInterfaceType)
    {
        case NvOdmUsbInterfaceType_UlpiNullPhy:
            break;
        case NvOdmUsbInterfaceType_UlpiExternalPhy:
            Ap20UsbPhyUlpiLinkModeConfigure(pUsbPhy, NV_FALSE);
            Ap20UsbPhyUlpiPowerControl(pUsbPhy, NV_FALSE);
            if (pUsbPhy->hOdmUlpi)
            {
                NvOdmUsbUlpiClose(pUsbPhy->hOdmUlpi);
                pUsbPhy->hOdmUlpi = NULL;
            }
            break;
        case NvOdmUsbInterfaceType_Utmi:
            Ap20UsbPhyUtmiPowerControl(pUsbPhy, NV_FALSE);
            default:
            break;
    }

    Ap20UsbPhyConfigUsbClockOverOn(pUsbPhy, NV_FALSE);

    return NvSuccess;
}

static void 
Ap20UsbPhyClose(
    NvDdkUsbPhy *pUsbPhy)
{
    // Power down the USB controller 
    NV_ASSERT_SUCCESS(Ap20UsbPhyPowerDown(pUsbPhy));
}

/* static void
PrintRegs(
    NvDdkUsbPhy *pUsbPhy)
{
    NvOsDebugPrintf("********** USB REG DUMP - Start\n");
    NvOsDebugPrintf("USBMODE:          0x%x\n",USB_REG_RD(USBMODE));
    NvOsDebugPrintf("OTGSC:            0x%x\n",USB_REG_RD(OTGSC));
    NvOsDebugPrintf("TXFILLTUNING:     0x%x\n",USB_REG_RD(TXFILLTUNING));
    NvOsDebugPrintf("ASYNCLISTADDR:    0x%x\n",USB_REG_RD(ASYNCLISTADDR));
    NvOsDebugPrintf("PERIODICLISTBASE: 0x%x\n",USB_REG_RD(PERIODICLISTBASE));
    NvOsDebugPrintf("USBCMD:           0x%x\n",USB_REG_RD(USBCMD));
    NvOsDebugPrintf("USBINTR:          0x%x\n",USB_REG_RD(USBINTR));
    NvOsDebugPrintf("USBSTS:           0x%x\n",USB_REG_RD(USBSTS));
    NvOsDebugPrintf("PORTSC1:          0x%x\n",USB_REG_RD(PORTSC1));
    NvOsDebugPrintf("********** USB REG DUMP - End\n");
} */

static void
Ap20PhyRemoveTristate(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;
    // Remove ULPI pads from tristate.  This step will bring the PHY out of suspend, since
    // the controller will start driving the ULPI pads.
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_TRISTATE_REG_B_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, TRISTATE_REG_B, Z_UAA, NORMAL, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, TRISTATE_REG_B, Z_UAB, NORMAL, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, TRISTATE_REG_B, Z_UAC, NORMAL, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_PP_TRISTATE_REG_B_0, RegVal);

    Ap20UsbPhyWaitForStableClock(pUsbPhy);
}

static void
Ap20PhyRestoreRegs(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU16 RegCount = 0;

    RegCount = pUsbPhy->Context.UsbRegCount;
    RegCount--;
    USB_REG_WR(USBMODE, pUsbPhy->Context.UsbRegs[RegCount--]);
    USB_REG_WR(OTGSC, pUsbPhy->Context.UsbRegs[RegCount--]);
    USB_REG_WR(TXFILLTUNING, pUsbPhy->Context.UsbRegs[RegCount--]);
    USB_REG_WR(ASYNCLISTADDR, pUsbPhy->Context.UsbRegs[RegCount--]);
    USB_REG_WR(PERIODICLISTBASE, pUsbPhy->Context.UsbRegs[RegCount--]);
    // Restore interrupt context later
    RegCount--;
    USB_REG_WR(USBCMD, pUsbPhy->Context.UsbRegs[RegCount--]);

}

static void
Ap20PhySaveContext(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU16 RegCount = 0;

    pUsbPhy->Context.UsbPortSpeed = (NvDdkUsbPhyPortSpeedType)USB_REG_READ_VAL(PORTSC1, PSPD);

    // If no device connection or invalid speeds just return
    if( pUsbPhy->Context.UsbPortSpeed > NvDdkUsbPhyPortSpeedType_High)
    {
        pUsbPhy->Context.IsValid = NV_FALSE;
        return;
    }
    // PrintRegs(pUsbPhy);

    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(USBCMD);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(USBINTR);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(PERIODICLISTBASE);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(ASYNCLISTADDR);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(TXFILLTUNING);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(OTGSC);
    pUsbPhy->Context.UsbRegs[RegCount++] = USB_REG_RD(USBMODE);
    pUsbPhy->Context.UsbRegCount = RegCount;
    pUsbPhy->Context.IsValid = NV_TRUE;
}

static void
Ap20PhyRestoreContext(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;
    /* If any saved context is present, restore it */
    // In case of no valid context just return
    if (!pUsbPhy->Context.IsValid)
        return;

    // Restore register context
    Ap20PhyRestoreRegs(pUsbPhy);

    // Enable Port Power
    USB_REG_UPDATE_DEF(PORTSC1, PP, POWERED);
    NvOsWaitUS(10);

    // Program the field PTC in PORTSC based on the saved speed mode
    RegVal = USB_REG_RD(PORTSC1);
    if (pUsbPhy->Context.UsbPortSpeed == NvDdkUsbPhyPortSpeedType_High)
    {
        RegVal = USB_FLD_SET_DRF_NUM(PORTSC1, PTC, 0x05, RegVal);
    }
    else if (pUsbPhy->Context.UsbPortSpeed == NvDdkUsbPhyPortSpeedType_Full)
    {
        RegVal = USB_FLD_SET_DRF_NUM(PORTSC1, PTC, 0x06, RegVal);
    }
    else if (pUsbPhy->Context.UsbPortSpeed == NvDdkUsbPhyPortSpeedType_Low)
    {
        RegVal = USB_FLD_SET_DRF_NUM(PORTSC1, PTC, 0x07, RegVal);
    }
    else
    {
        NV_ASSERT(!"\n speed is not configureed properly");
    }
    USB_REG_WR(PORTSC1, RegVal);
    NvOsWaitUS(10);

    // Disable test mode by setting PTC field to NORMAL_OP
    USB_REG_UPDATE_DEF(PORTSC1, PTC, NORMAL_OP);
    // Poll until CCS is enabled
    do
    {
        RegVal = USB_REG_RD(PORTSC1);
        if (USB_DRF_VAL(PORTSC1, CCS, RegVal))
            break;
        NvOsWaitUS(1);
        TimeOut--;
    } while (TimeOut);
    // Poll until PE is enabled
    TimeOut = USB_PHY_HW_TIMEOUT_US;
    do
    {
        RegVal = USB_REG_RD(PORTSC1);
        if (USB_DRF_VAL(PORTSC1, PE, RegVal))
            break;
        NvOsWaitUS(1);
        TimeOut--;
    } while (TimeOut);
    // Clear the PCI status, to avoid an interrupt taken upon resume
    USB_REG_UPDATE_DEF(USBSTS, PCI, PORT_CHANGE);
    TimeOut = USB_PHY_HW_TIMEOUT_US;
    do
    {
        RegVal = USB_REG_RD(USBSTS);
        if (!(USB_DRF_VAL(USBSTS, PCI, RegVal)))
            break;
        NvOsWaitUS(1);
        TimeOut--;
    } while (TimeOut);

    // Put controller in suspend mode by writing 1 to SUSP bit of PORTSC
    Ap20UsbPhySuspendPort(pUsbPhy);

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_UlpiExternalPhy)
    {
        Ap20PhyRemoveTristate(pUsbPhy);
    }

    // Restore interrupt register
    USB_REG_WR(USBINTR, pUsbPhy->Context.UsbRegs[1]);
    NvOsWaitUS(10);

    // PrintRegs(pUsbPhy);
}

void
Ap20UsbPhyOpenHwInterface(
    NvDdkUsbPhy *pUsbPhy)
{
    pUsbPhy->PowerUp = Ap20UsbPhyPowerUp;
    pUsbPhy->PowerDown = Ap20UsbPhyPowerDown;
    pUsbPhy->Ioctl = Ap20UsbPhyIoctl;
    pUsbPhy->WaitForStableClock = Ap20UsbPhyWaitForStableClock;
    pUsbPhy->MemoryPrefetch = Ap20UsbPhyMemoryPrefetch;
    pUsbPhy->CloseHwInterface = Ap20UsbPhyClose;
    pUsbPhy->SaveContext = Ap20PhySaveContext;
    pUsbPhy->RestoreContext = Ap20PhyRestoreContext;
}

