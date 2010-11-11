/*
 * arch/arm/mach-tegra/odm_kit/query/harmony/nvodm_query.c
 *
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvodm_query.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_memc.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_pins.h"
#include "nvodm_query_pins_ap20.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"

#if !defined(NV_OAL)
#define NV_OAL (0)
#endif

#define BOARD_ID_HARMONY 0x0B3E
#define HARMONY_HYS5C1GB_SKU 0x3829

#define NVODM_ENABLE_EMC_DVFS (1)

// Although AP16 Concorde2 and AP16 Vail boards can support PMU
// interrupt, keep it disabled for now because of PMU VBUS input
// latch problem
#define NVODM_PMU_INT_ENABLED (0)

static const NvU8
s_NvOdmQueryDeviceNamePrefixValue[] = {'T','e','g','r','a',0};

static const NvU8
s_NvOdmQueryManufacturerSetting[] = {'N','V','I','D','I','A',0};

static const NvU8
s_NvOdmQueryModelSetting[] = {'A','P','2','0',0};

static const NvU8
s_NvOdmQueryPlatformSetting[] = {'H','a','r','m','o','n','y',0};

static const NvU8
s_NvOdmQueryProjectNameSetting[] = {'O','D','M',' ','K','i','t',0};

static const NvOdmDownloadTransport
s_NvOdmQueryDownloadTransportSetting = NvOdmDownloadTransport_None;

static const NvOdmQuerySdioInterfaceProperty s_NvOdmQuerySdioInterfaceProperty[4] =
{
    { NV_FALSE, 10,  NV_TRUE, 0x8, NvOdmQuerySdioSlotUsage_wlan   },
    { NV_TRUE,   0, NV_FALSE, 0x5, NvOdmQuerySdioSlotUsage_Media  },
    { NV_TRUE,   0, NV_FALSE, 0x6, NvOdmQuerySdioSlotUsage_unused },
    { NV_TRUE,   0, NV_FALSE, 0x4, NvOdmQuerySdioSlotUsage_Media  }
};

static const NvOdmQuerySpiDeviceInfo s_NvOdmQuerySpiDeviceInfoTable [] =
{
    {NvOdmQuerySpiSignalMode_0, NV_TRUE}    // Spi1_Devices_0 (chip sel 0)
};

// Spi idle signal state
static const NvOdmQuerySpiIdleSignalState s_NvOdmQuerySpiIdleSignalStateLevel[] =
{
    {NV_FALSE, NvOdmQuerySpiSignalMode_0, NV_FALSE}    // Spi 1
};
// We can have two I2s Instances
static const NvOdmQueryI2sInterfaceProperty s_NvOdmQueryI2sInterfacePropertySetting[] =
{
    {
        NvOdmQueryI2sMode_Master,               // Mode
        NvOdmQueryI2sLRLineControl_LeftOnLow,   // I2sLRLineControl
        NvOdmQueryI2sDataCommFormat_I2S,        // I2sDataCommunicationFormat
        NV_FALSE,                               // IsFixedMCLK  
        0                                       // FixedMCLKFrequency 
    },
    {
        NvOdmQueryI2sMode_Master,               // Mode
        NvOdmQueryI2sLRLineControl_LeftOnLow,   // I2sLRLineControl
        NvOdmQueryI2sDataCommFormat_I2S,        // I2sDataCommunicationFormat
        NV_FALSE,                               // IsFixedMCLK  
        0                                       // FixedMCLKFrequency 
    }
};


static const NvOdmQuerySpdifInterfaceProperty s_NvOdmQuerySpdifInterfacePropertySetting =
{
    NvOdmQuerySpdifDataCaptureControl_FromLeft
};

static const NvOdmQueryAc97InterfaceProperty s_NvOdmQueryAc97InterfacePropertySetting =
{
    NV_FALSE,
    NV_FALSE,
    NV_FALSE,
    NV_FALSE,
    NV_TRUE
};

// Add support for the Codec Formats
// It must the order (dapIndex) how the codec is connected to the Dap port
static const NvOdmQueryI2sACodecInterfaceProp s_NvOdmQueryI2sACodecInterfacePropSetting[] =
{
    {
        NV_FALSE,                               // IsCodecMaster
        0,                                      // DapPortIndex
        0x36,                                   // DevAddress
        NV_FALSE,                               // IsUsbmode
        NvOdmQueryI2sLRLineControl_LeftOnLow,   // I2sCodecLRLineControl
        NvOdmQueryI2sDataCommFormat_I2S         // I2sCodecDataCommFormat
    }
};

static const NvOdmQueryDapPortConnection s_NvOdmQueryDapPortConnectionTable[] =
{
    // the Default Music Path
    { NvOdmDapConnectionIndex_Music_Path, 2,
    { {NvOdmDapPort_I2s1, NvOdmDapPort_Dap1, NV_TRUE},
      {NvOdmDapPort_Dap1, NvOdmDapPort_I2s1, NV_FALSE}
    }},

    // Bluetooth to Codec
    { NvOdmDapConnectionIndex_BlueTooth_Codec, 3,
    { {NvOdmDapPort_Dap4, NvOdmDapPort_I2s1, NV_TRUE},
      {NvOdmDapPort_I2s1, NvOdmDapPort_Dap4, NV_FALSE},
      {NvOdmDapPort_I2s2, NvOdmDapPort_Dap1, NV_FALSE}
    }}
};


// Ap20 support 5 dap ports
// For port is connected to DAC(I2s) then PortMode is not valid- as Dac would be driving it
static const NvOdmQueryDapPortProperty s_NvOdmQueryDapPortInfoTable[] =
{
    {NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Reserved
    // I2S1 (DAC1) <-> DAP1 <-> HIFICODEC
    {NvOdmDapPort_I2s1, NvOdmDapPort_HifiCodecType,
        {2, 16, 44100, NvOdmQueryI2sDataCommFormat_I2S}},   // Dap1
    {NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Dap2
    {NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Dap3
    // I2S2 (DAC2) <-> DAP4 <-> BLUETOOTH
    {NvOdmDapPort_I2s2, NvOdmDapPort_BlueTooth,
        {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S}}     // Dap4
};

static const NvOdmSdramControllerConfigAdv s_NvOdmHyS5c1GbEmcConfigTable[] =
{
    {
                  0x20,   /* Rev 2.0 */
                166500,   /* SDRAM frquency */
                   950,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x0000000A,   /* RC */
            0x00000016,   /* RFC */
            0x00000008,   /* RAS */
            0x00000003,   /* RP */
            0x00000004,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000C,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000001,   /* REXT */
            0x00000004,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000D,   /* RDV */
            0x000004DF,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000003,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000A,   /* RW2PDEN */
            0x000000C8,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000006,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000000,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000083,   /* FBIO_CFG5 */
            0xE03B0323,   /* CFG_DIG_DLL */
            0x007FC010,   /* DLL_XFORM_DQS */
            0x00008010,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000000,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                333000,   /* SDRAM frquency */
                  1200,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000014,   /* RC */
            0x0000002B,   /* RFC */
            0x0000000F,   /* RAS */
            0x00000005,   /* RP */
            0x00000004,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x0000000C,   /* W2P */
            0x00000005,   /* RD_RCD */
            0x00000005,   /* WR_RCD */
            0x00000003,   /* RRD */
            0x00000001,   /* REXT */
            0x00000004,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000D,   /* RDV */
            0x000009FF,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000005,   /* PCHG2PDEN */
            0x00000005,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000F,   /* RW2PDEN */
            0x000000C8,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000C,   /* TFAW */
            0x00000006,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000000,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000083,   /* FBIO_CFG5 */
            0xF0320303,   /* CFG_DIG_DLL */
            0x007FC010,   /* DLL_XFORM_DQS */
            0x00008010,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000000,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    }
};

// Wake Events
static NvOdmWakeupPadInfo s_NvOdmWakeupPadInfo[] =
{
    {NV_FALSE,  0, NvOdmWakeupPadPolarity_Low},     // Wake Event  0 - ulpi_data4 (UART_RI)
    {NV_FALSE,  1, NvOdmWakeupPadPolarity_High},    // Wake Event  1 - gp3_pv[3] (BB_MOD, MODEM_RESET_OUT)
    {NV_FALSE,  2, NvOdmWakeupPadPolarity_High},    // Wake Event  2 - dvi_d3
    {NV_FALSE,  3, NvOdmWakeupPadPolarity_Low},     // Wake Event  3 - sdio3_dat1
    {NV_FALSE,  4, NvOdmWakeupPadPolarity_High},    // Wake Event  4 - hdmi_int (HDMI_HPD)
    {NV_TRUE,   5, NvOdmWakeupPadPolarity_Low},     // Wake Event  5 - vgp[6] (VI_GP6, Flash_EN2)
    {NV_FALSE,  6, NvOdmWakeupPadPolarity_High},    // Wake Event  6 - gp3_pu[5] (GPS_ON_OFF, GPS_IRQ)
#ifdef CONFIG_BT_BLUESLEEP
    {NV_TRUE,  7, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  7 - gp3_pu[6] (GPS_INT, BT_IRQ)
#else
    {NV_FALSE,  7, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  7 - gp3_pu[6] (GPS_INT, BT_IRQ)
#endif
    {NV_FALSE,  8, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  8 - gmi_wp_n (MICRO SD_CD)
    {NV_FALSE,  9, NvOdmWakeupPadPolarity_High},    // Wake Event  9 - gp3_ps[2] (KB_COL10)
    {NV_FALSE, 10, NvOdmWakeupPadPolarity_High},    // Wake Event 10 - gmi_ad21 (Accelerometer_TH/TAP)
    {NV_TRUE,  11, NvOdmWakeupPadPolarity_Low},     // Wake Event 11 - spi2_cs2 (PEN_INT, AUDIO-IRQ, LOW_BAT#)
    {NV_FALSE, 12, NvOdmWakeupPadPolarity_Low},     // Wake Event 12 - spi2_cs1 (HEADSET_DET, not used)
    {NV_TRUE,  13, NvOdmWakeupPadPolarity_Low},     // Wake Event 13 - sdio1_dat1 (WLAN_WAKE)
    {NV_FALSE, 14, NvOdmWakeupPadPolarity_High},    // Wake Event 14 - gp3_pv[6] (WLAN_INT)
    {NV_FALSE, 15, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 15 - gmi_ad16  (SPI3_DOUT, DTV_SPI4_CS1)
    {NV_TRUE,  16, NvOdmWakeupPadPolarity_High},    // Wake Event 16 - rtc_irq
#ifdef CONFIG_KEYBOARD_TEGRA
    {NV_TRUE,  17, NvOdmWakeupPadPolarity_High},    // Wake Event 17 - kbc_interrupt
#else
    {NV_FALSE, 17, NvOdmWakeupPadPolarity_High},
#endif
    {NV_TRUE,  18, NvOdmWakeupPadPolarity_Low},     // Wake Event 18 - pwr_int (PMIC_INT)
    {NV_FALSE, 19, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 19 - usb_vbus_wakeup[0]
    {NV_FALSE, 20, NvOdmWakeupPadPolarity_High},    // Wake Event 20 - usb_vbus_wakeup[1]
    {NV_FALSE, 21, NvOdmWakeupPadPolarity_Low},     // Wake Event 21 - usb_iddig[0]
    {NV_FALSE, 22, NvOdmWakeupPadPolarity_Low},     // Wake Event 22 - usb_iddig[1]
    {NV_TRUE,  23, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 23 - gmi_iordy (HSMMC_CLK)
    {NV_TRUE,  24, NvOdmWakeupPadPolarity_Low},     // Wake Event 24 - gp3_pv[2]
    {NV_FALSE, 25, NvOdmWakeupPadPolarity_High},    // Wake Event 25 - gp3_ps[4] (KB_COL12)
    {NV_FALSE, 26, NvOdmWakeupPadPolarity_High},    // Wake Event 26 - gp3_ps[5] (KB_COL10)
    {NV_FALSE, 27, NvOdmWakeupPadPolarity_High},    // Wake Event 27 - gp3_ps[0] (KB_COL8)
    {NV_FALSE, 28, NvOdmWakeupPadPolarity_Low},     // Wake Event 28 - gp3_pq[6] (KB_ROW6)
    {NV_FALSE, 29, NvOdmWakeupPadPolarity_Low},     // Wake Event 29 - gp3_pq[7] (KB_ROW6)
    {NV_FALSE, 30, NvOdmWakeupPadPolarity_High}     // Wake Event 30 - dap1_dout (DAP1_DOUT)
};

/* --- Function Implementations ---*/
static NvU32
GetBctKeyValue(void)
{
    NvOdmServicesKeyListHandle hKeyList = NULL;
    NvU32 BctCustOpt = 0;

    hKeyList = NvOdmServicesKeyListOpen();
    if (hKeyList)
    {
        BctCustOpt =
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
    }

    return BctCustOpt;
}

NvOdmDebugConsole
NvOdmQueryDebugConsole(void)
{
    NvU32 CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DEFAULT:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DCC:
        return NvOdmDebugConsole_Dcc;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_NONE:
        return NvOdmDebugConsole_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_UART:
        return NvOdmDebugConsole_UartA +
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE_OPTION, CustOpt);
    default:
        return NvOdmDebugConsole_None;
    }
}

NvOdmDownloadTransport
NvOdmQueryDownloadTransport(void)
{
    NvU32 CustOpt = GetBctKeyValue();

    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, TRANSPORT, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_NONE:
        return NvOdmDownloadTransport_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_USB:
        return NvOdmDownloadTransport_Usb;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_ETHERNET:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, ETHERNET_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_SPI:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_DEFAULT:
        default:
            return NvOdmDownloadTransport_SpiEthernet;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_UART:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, UART_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_B:
            return NvOdmDownloadTransport_UartB;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_C:
            return NvOdmDownloadTransport_UartC;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_DEFAULT:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_A:
        default:
            return NvOdmDownloadTransport_UartA;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_DEFAULT:
    default:
        return s_NvOdmQueryDownloadTransportSetting;
    }
}

const NvU8*
NvOdmQueryDeviceNamePrefix(void)
{
    return s_NvOdmQueryDeviceNamePrefixValue;
}

const NvOdmQuerySpiDeviceInfo *
NvOdmQuerySpiGetDeviceInfo(
    NvOdmIoModule OdmIoModule,
    NvU32 ControllerId,
    NvU32 ChipSelect)
{
    if (OdmIoModule == NvOdmIoModule_Spi)
    {
        switch (ControllerId)
        {
            case 0:
                if (ChipSelect == 0)
                    return &s_NvOdmQuerySpiDeviceInfoTable[0];
                break;

            default:
                break;
        }
        return NULL;
    }
    return NULL;
}

const NvOdmQuerySpiIdleSignalState *
NvOdmQuerySpiGetIdleSignalState(
    NvOdmIoModule OdmIoModule,
    NvU32 ControllerId)
{
    if (OdmIoModule == NvOdmIoModule_Spi)
    {
        if (ControllerId == 0)
            return &s_NvOdmQuerySpiIdleSignalStateLevel[0];
    }
    return NULL;
}

const NvOdmQueryI2sInterfaceProperty *
NvOdmQueryI2sGetInterfaceProperty(
    NvU32 I2sInstanceId)
{
    if ((I2sInstanceId == 0) || (I2sInstanceId == 1))
        return &s_NvOdmQueryI2sInterfacePropertySetting[I2sInstanceId];

    return NULL;
}

const NvOdmQueryDapPortProperty *
NvOdmQueryDapPortGetProperty(
    NvU32 DapPortId)
{
    if (DapPortId > 0 && DapPortId < NV_ARRAY_SIZE(s_NvOdmQueryDapPortInfoTable) )
        return &s_NvOdmQueryDapPortInfoTable[DapPortId];

    return NULL;
}

const NvOdmQueryDapPortConnection*
NvOdmQueryDapPortGetConnectionTable(
    NvU32 ConnectionIndex)
{
    NvU32 TableIndex   = 0;
    for( TableIndex = 0; 
         TableIndex < NV_ARRAY_SIZE(s_NvOdmQueryDapPortConnectionTable);
         TableIndex++)
    {
        if (s_NvOdmQueryDapPortConnectionTable[TableIndex].UseIndex 
                                                    == ConnectionIndex)
            return &s_NvOdmQueryDapPortConnectionTable[TableIndex];
    }
    return NULL;
}

const NvOdmQuerySpdifInterfaceProperty *
NvOdmQuerySpdifGetInterfaceProperty(
    NvU32 SpdifInstanceId)
{
    if (SpdifInstanceId == 0)
        return &s_NvOdmQuerySpdifInterfacePropertySetting;

    return NULL;
}

const NvOdmQueryAc97InterfaceProperty *
NvOdmQueryAc97GetInterfaceProperty(
    NvU32 Ac97InstanceId)
{
    if (Ac97InstanceId == 0)
        return &s_NvOdmQueryAc97InterfacePropertySetting;

    return NULL;
}

const NvOdmQueryI2sACodecInterfaceProp *
NvOdmQueryGetI2sACodecInterfaceProperty(
    NvU32 AudioCodecId)
{
    NvU32 NumInstance = sizeof(s_NvOdmQueryI2sACodecInterfacePropSetting)/
                            sizeof(s_NvOdmQueryI2sACodecInterfacePropSetting[0]);
    if (AudioCodecId < NumInstance)
        return &s_NvOdmQueryI2sACodecInterfacePropSetting[AudioCodecId];

    return NULL;
}

/**
 * This function is called from early boot process.
 * Therefore, it cannot use global variables.
 */
NvBool NvOdmQueryAsynchMemConfig(
    NvU32 ChipSelect,
    NvOdmAsynchMemConfig *pMemConfig)
{
    return NV_FALSE;
}

const void*
NvOdmQuerySdramControllerConfigGet(NvU32 *pEntries, NvU32 *pRevision)
{
#if NVODM_ENABLE_EMC_DVFS
    NvOdmBoardInfo BoardInfo;

    if (NvOdmPeripheralGetBoardInfo(BOARD_ID_HARMONY, &BoardInfo))
    {
        if (BoardInfo.SKU == HARMONY_HYS5C1GB_SKU)
        {
            if (pRevision)
                *pRevision = s_NvOdmHyS5c1GbEmcConfigTable[0].Revision;
            if (pEntries)
                *pEntries = NV_ARRAY_SIZE(s_NvOdmHyS5c1GbEmcConfigTable);
            return (const void*)s_NvOdmHyS5c1GbEmcConfigTable;
        }
    }
#endif
    if (pEntries)
        *pEntries = 0;
    return NULL;
}

NvOdmQueryOscillator NvOdmQueryGetOscillatorSource(void)
{
    return NvOdmQueryOscillator_Xtal;
}

NvU32 NvOdmQueryGetOscillatorDriveStrength(void)
{
    /// Oscillator drive strength range is 0 to 0x3F
    return 0x04;
}

const NvOdmWakeupPadInfo *NvOdmQueryGetWakeupPadTable(NvU32 *pSize)
{
    if (pSize)
        *pSize = NV_ARRAY_SIZE(s_NvOdmWakeupPadInfo);

    return (const NvOdmWakeupPadInfo *) s_NvOdmWakeupPadInfo;
}

const NvU8* NvOdmQueryManufacturer(void)
{
    return s_NvOdmQueryManufacturerSetting;
}

const NvU8* NvOdmQueryModel(void)
{
    return s_NvOdmQueryModelSetting;
}

const NvU8* NvOdmQueryPlatform(void)
{
    return s_NvOdmQueryPlatformSetting;
}

const NvU8* NvOdmQueryProjectName(void)
{
    return s_NvOdmQueryProjectNameSetting;
}

#define EXT 0     // external pull-up/down resistor
#define INT_PU 1  // internal pull-up
#define INT_PD 2  // internal pull-down

#define HIGHSPEED 1
#define SCHMITT 1
#define VREF    1
#define OHM_50 3
#define OHM_100 2
#define OHM_200 1
#define OHM_400 0

 // Pin attributes
static const NvOdmPinAttrib pin_config[] = {
    // Pull ups for the kbc pins
    { NvOdmPinRegister_Ap20_PullUpDown_B,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_B(0x0, 0x0, 0x0, 0x0, 0x2, 0x2, 0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0) },

    // Pull ups for the kbc pins
    { NvOdmPinRegister_Ap20_PullUpDown_E,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_E(0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x2, 0x2) },

    // Set pad control for the sdio2 - - AOCFG1 and AOCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio3 - SDIO2 and SDIO3 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO2CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_SDIO3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for I2C1 pins
    { NvOdmPinRegister_Ap20_PadCtrl_DBGCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for I2C2 (DDC) pins
    { NvOdmPinRegister_Ap20_PadCtrl_DDCCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio1 - SDIO1 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO1CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

     // WiFi Pins (DTA, DTD) need to be pulled up
    { NvOdmPinRegister_Ap20_PullUpDown_A,
      NVODM_QUERY_PIN_AP20_PULLUPDOWN_A(0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0) },

    // Set pad control for the sdio4- ATCCFG1 and ATCCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) }
};

NvU32
NvOdmQueryPinAttributes(const NvOdmPinAttrib** pPinAttributes)
{
    if (pPinAttributes)
    {
        *pPinAttributes = &pin_config[0];
        return NV_ARRAY_SIZE(pin_config);
    }
    return 0;
}

NvBool NvOdmQueryGetPmuProperty(NvOdmPmuProperty* pPmuProperty)
{
    pPmuProperty->IrqConnected = NV_TRUE;
    pPmuProperty->PowerGoodCount = 0x7E7E;
    pPmuProperty->IrqPolarity = NvOdmInterruptPolarity_Low;
    pPmuProperty->CorePowerReqPolarity = NvOdmCorePowerReqPolarity_Low;
    pPmuProperty->SysClockReqPolarity = NvOdmSysClockReqPolarity_High;
    pPmuProperty->CombinedPowerReq = NV_FALSE;
    pPmuProperty->CpuPowerGoodUs = 2000;
    pPmuProperty->AccuracyPercent = 3;
    pPmuProperty->VCpuOTPOnWakeup = NV_FALSE;
    pPmuProperty->PowerOffCount = 0;
    pPmuProperty->CpuPowerOffUs = 0;
    return NV_TRUE;
}

/**
 * Gets the lowest soc power state supported by the hardware
 *
 * @returns information about the SocPowerState
 */
const NvOdmSocPowerStateInfo* NvOdmQueryLowestSocPowerState(void)
{

    static                      NvOdmSocPowerStateInfo  PowerStateInfo;
    const static                NvOdmSocPowerStateInfo* pPowerStateInfo = NULL;
    NvOdmServicesKeyListHandle  hKeyList;
    NvU32                       LPStateSelection = 0;
    if (pPowerStateInfo == NULL)
    {

        hKeyList = NvOdmServicesKeyListOpen();
        if (hKeyList)
        {
            LPStateSelection = NvOdmServicesGetKeyValue(hKeyList,
                                                NvOdmKeyListId_ReservedBctCustomerOption);
            NvOdmServicesKeyListClose(hKeyList);
            LPStateSelection = NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, LPSTATE, LPStateSelection);
        }
        // Lowest power state controlled by the flashed custom option.
        PowerStateInfo.LowestPowerState =  ((LPStateSelection != TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP1)?
                                            NvOdmSocPowerState_Suspend : NvOdmSocPowerState_DeepSleep);
        pPowerStateInfo = (const NvOdmSocPowerStateInfo*) &PowerStateInfo;
    }
    return (pPowerStateInfo);
}

const NvOdmUsbProperty*
NvOdmQueryGetUsbProperty(NvOdmIoModule OdmIoModule,
                         NvU32 Instance)
{
    static const NvOdmUsbProperty Usb1Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        (NvOdmUsbChargerType_SE0 | NvOdmUsbChargerType_SE1 | NvOdmUsbChargerType_SK),
        20,
        NV_TRUE,
        NvOdmUsbModeType_Device,
        NvOdmUsbIdPinType_CableId,
        NvOdmUsbConnectorsMuxType_None,
        NV_TRUE
    };

     static const NvOdmUsbProperty Usb2Property =
     {
        NvOdmUsbInterfaceType_UlpiExternalPhy,
        NvOdmUsbChargerType_UsbHost,
        20,
        NV_TRUE,
        NvOdmUsbModeType_Host,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_TRUE
    };

    static const NvOdmUsbProperty Usb3Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        NvOdmUsbChargerType_UsbHost,
        20,
        NV_TRUE,
        NvOdmUsbModeType_Host,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_TRUE
    };

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 0)
        return &(Usb1Property);

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 1)
        return &(Usb2Property);

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 2)
        return &(Usb3Property);

    return (const NvOdmUsbProperty *)NULL;
}

const NvOdmQuerySdioInterfaceProperty* NvOdmQueryGetSdioInterfaceProperty(NvU32 Instance)
{
    return &s_NvOdmQuerySdioInterfaceProperty[Instance];
}

const NvOdmQueryHsmmcInterfaceProperty* NvOdmQueryGetHsmmcInterfaceProperty(NvU32 Instance)
{
    return NULL;
}

NvU32
NvOdmQueryGetBlockDeviceSectorSize(NvOdmIoModule OdmIoModule)
{
    return 0;
}

const NvOdmQueryOwrDeviceInfo* NvOdmQueryGetOwrDeviceInfo(NvU32 Instance)
{
    return NULL;
}

const NvOdmGpioWakeupSource *NvOdmQueryGetWakeupSources(NvU32 *pCount)
{
    *pCount = 0;
    return NULL;
}

/**
 * This function is called from early boot process.
 * Therefore, it cannot use global variables.
 */
NvU32 NvOdmQueryMemSize(NvOdmMemoryType MemType)
{
    NvOdmOsOsInfo Info;
    NvU32 MemBctCustOpt = GetBctKeyValue();

    switch (MemType)
    {
        // NOTE:
        // For Windows CE/WM operating systems the total size of SDRAM may
        // need to be reduced due to limitations in the virtual address map.
        // Under the legacy physical memory manager, Windows OSs have a
        // maximum 512MB statically mapped virtual address space. Under the
        // new physical memory manager, Windows OSs have a maximum 1GB
        // statically mapped virtual address space. Out of that virtual
        // address space, the upper 32 or 36 MB (depending upon the SOC)
        // of the virtual address space is reserved for SOC register
        // apertures.
        //
        // Refer to virtual_tables_apxx.arm for the reserved aperture list.
        // If the cumulative size of the reserved apertures changes, the
        // maximum size of SDRAM will also change.
        case NvOdmMemoryType_Sdram:
            switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_SYSTEM, MEMORY, MemBctCustOpt))
            {
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_256:
                    if ( NvOdmOsGetOsInformation(&Info) &&
                         ((Info.OsType!=NvOdmOsOs_Windows) ||
                          (Info.OsType==NvOdmOsOs_Windows && Info.MajorVersion>=7)) )
                        return 0x10000000;
                    else
                        return 0x0DD00000;  // Legacy Physical Memory Manager: 256 MB - 35 MB
    
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_1024:
                    if ( NvOdmOsGetOsInformation(&Info) &&
                         ((Info.OsType!=NvOdmOsOs_Windows) ||
                          (Info.OsType==NvOdmOsOs_Windows && Info.MajorVersion>=7)) )
                        return 0x40000000;
                    else
                        // Earlier versions of WinCE only support 512MB max memory size
                        return 0x1E000000;  // Legacy Physical Memory Manager: 512 MB - 32 MB

                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_512:
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_DEFAULT:
                default:
                    if ( NvOdmOsGetOsInformation(&Info) &&
                         ((Info.OsType!=NvOdmOsOs_Windows) ||
                          (Info.OsType==NvOdmOsOs_Windows && Info.MajorVersion>=7)) )
                        return 0x20000000;
                    else
                        return 0x1E000000;  // Legacy Physical Memory Manager: 512 MB - 32 MB
            }

        case NvOdmMemoryType_Nor:
            return 0x00400000;  // 4 MB

        case NvOdmMemoryType_Nand:
        case NvOdmMemoryType_I2CEeprom:
        case NvOdmMemoryType_Hsmmc:
        case NvOdmMemoryType_Mio:
        default:
            return 0;
    }
}

NvU32 NvOdmQueryCarveoutSize(void)
{
    return 0x04000000;  // 64 MB
}

NvU32 NvOdmQuerySecureRegionSize(void)
{
    return 0x00800000;// 8 MB
}
