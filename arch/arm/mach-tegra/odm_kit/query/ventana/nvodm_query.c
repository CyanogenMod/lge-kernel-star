/*
 * arch/arm/mach-tegra/odm_kit/query/ventana/nvodm_query.c
 *
 * Copyright (c) 2007-2010 NVIDIA Corporation.
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
#include "nvodm_query_nand.h"
#include "nvodm_query_kbc.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_pins.h"
#include "nvodm_query_pins_ap20.h"
#include "nvrm_drf.h"
#include "nvodm_keylist_reserved.h"


#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_RANGE 31:31
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP0 0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP1 0x1UL

#define NVODM_ENABLE_EMC_DVFS (1)

#define BOARD_ID_VENTANA 0x024B
// Wake Events
static NvOdmWakeupPadInfo s_NvOdmWakeupPadInfo[] =
{
    {NV_FALSE,  0, NvOdmWakeupPadPolarity_Low},     // Wake Event  0 - ulpi_data4 (UART_RI)
    {NV_FALSE,  1, NvOdmWakeupPadPolarity_High},    // Wake Event  1 - gp3_pv[3] (BB_MOD, MODEM_RESET_OUT)
    {NV_FALSE,  2, NvOdmWakeupPadPolarity_High},    // Wake Event  2 - dvi_d3
    {NV_FALSE,  3, NvOdmWakeupPadPolarity_Low},     // Wake Event  3 - sdio3_dat1
    {NV_FALSE,  4, NvOdmWakeupPadPolarity_High},    // Wake Event  4 - hdmi_int (HDMI_HPD)
    {NV_FALSE,   5, NvOdmWakeupPadPolarity_Low},      // Wake Event  5 - vgp[6] (VI_GP6, Flash_EN2)
    {NV_FALSE,  6, NvOdmWakeupPadPolarity_High},    // Wake Event  6 - gp3_pu[5] (Lid On/Off)
    {NV_TRUE,  7, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  7 - gp3_pu[6] (GPS_INT, BT_IRQ)
    {NV_TRUE,  8, NvOdmWakeupPadPolarity_Low}, // Wake Event  8 - gmi_wp_n (MICRO SD_CD)
    {NV_FALSE,  9, NvOdmWakeupPadPolarity_High},    // Wake Event  9 - gp3_ps[2] (KB_COL10)
    {NV_FALSE, 10, NvOdmWakeupPadPolarity_High},    // Wake Event 10 - gmi_ad21 (Accelerometer_TH/TAP)
    {NV_FALSE,  11, NvOdmWakeupPadPolarity_Low},     // Wake Event 11 - spi2_cs2 (PEN_INT, AUDIO-IRQ, LOW_BAT#)
    {NV_FALSE, 12, NvOdmWakeupPadPolarity_Low},     // Wake Event 12 - spi2_cs1 (HEADSET_DET, not used)
    {NV_TRUE,  13, NvOdmWakeupPadPolarity_Low},     // Wake Event 13 - sdio1_dat1 (WLAN_WAKE)
    {NV_FALSE, 14, NvOdmWakeupPadPolarity_High},    // Wake Event 14 - gp3_pv[6] (WLAN_INT)
    {NV_FALSE, 15, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 15 - gmi_ad16  (SPI3_DOUT, DTV_SPI4_CS1)
    {NV_TRUE, 16, NvOdmWakeupPadPolarity_High},    // Wake Event 16 - rtc_irq
    {NV_FALSE,  17, NvOdmWakeupPadPolarity_High},    // Wake Event 17 - kbc_interrupt
    {NV_FALSE, 18, NvOdmWakeupPadPolarity_Low},     // Wake Event 18 - pwr_int (PMIC_INT)
    {NV_FALSE, 19, NvOdmWakeupPadPolarity_High},    // Wake Event 19 - usb_vbus_wakeup[0]
    {NV_FALSE, 20, NvOdmWakeupPadPolarity_High},    // Wake Event 20 - usb_vbus_wakeup[1]
    {NV_FALSE, 21, NvOdmWakeupPadPolarity_Low},     // Wake Event 21 - usb_iddig[0]
    {NV_FALSE, 22, NvOdmWakeupPadPolarity_Low},     // Wake Event 22 - usb_iddig[1]
    {NV_FALSE, 23, NvOdmWakeupPadPolarity_Low},     // Wake Event 23 - gmi_iordy (HSMMC_CLK)
    {NV_TRUE, 24, NvOdmWakeupPadPolarity_Low},    // Wake Event 24 - gp3_pv[2] (power key on gpio  port v2)
    {NV_FALSE, 25, NvOdmWakeupPadPolarity_High},    // Wake Event 25 - gp3_ps[4] (KB_COL12)
    {NV_FALSE, 26, NvOdmWakeupPadPolarity_High},    // Wake Event 26 - gp3_ps[5] (KB_COL10)
    {NV_FALSE, 27, NvOdmWakeupPadPolarity_High},    // Wake Event 27 - gp3_ps[0] (KB_COL8)
    {NV_FALSE, 28, NvOdmWakeupPadPolarity_Low},     // Wake Event 28 - gp3_pq[6] (KB_ROW6)
    {NV_FALSE, 29, NvOdmWakeupPadPolarity_Low},     // Wake Event 29 - gp3_pq[7] (KB_ROW6)
    {NV_FALSE, 30, NvOdmWakeupPadPolarity_High}     // Wake Event 30 - dap1_dout (DAP1_DOUT)
};

static const NvOdmSdramControllerConfigAdv s_NvOdmPM275ElpidaEmcConfigTable[] =
{
    {
                  0x20,   /* Rev 2.0 */
                 18000,   /* SDRAM frquency */
                   900,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000002,   /* RC */
            0x00000006,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x00000009,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000002,   /* WDV */
            0x00000004,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000b,   /* RDV */
            0x00000038,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000a,   /* RW2PDEN */
            0x00000003,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000006,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x0000004b,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xA06A04AE,   /* CFG_DIG_DLL */
            0x00004410,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000002,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 27000,   /* SDRAM frquency */
                   950,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000002,   /* RC */
            0x00000006,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x00000009,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000002,   /* WDV */
            0x00000004,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000b,   /* RDV */
            0x00000054,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000a,   /* RW2PDEN */
            0x00000004,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000006,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000071,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xA06A04AE,   /* CFG_DIG_DLL */
            0x00004410,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000003,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 54000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000004,   /* RC */
            0x00000008,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x00000009,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000002,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000b,   /* RDV */
            0x000000a8,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000a,   /* RW2PDEN */
            0x00000008,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000006,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000000e1,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xA06A04AE,   /* CFG_DIG_DLL */
            0x00004410,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000005,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                108000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000007,   /* RC */
            0x0000000f,   /* RFC */
            0x00000005,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x00000009,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000002,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000b,   /* RDV */
            0x0000017f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000a,   /* RW2PDEN */
            0x00000010,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000006,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000001c2,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xA06A04AE,   /* CFG_DIG_DLL */
            0x007FC010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000a,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                150000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000009,   /* RC */
            0x00000014,   /* RFC */
            0x00000007,   /* RAS */
            0x00000004,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x00000009,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000002,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000b,   /* RDV */
            0x0000021f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000004,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000a,   /* RW2PDEN */
            0x00000015,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000006,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000270,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xA04C04AE,   /* CFG_DIG_DLL */
            0x007FC010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000e,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                300000,   /* SDRAM frquency */
                  1200,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000012,   /* RC */
            0x00000027,   /* RFC */
            0x0000000D,   /* RAS */
            0x00000007,   /* RP */
            0x00000007,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x00000009,   /* W2P */
            0x00000006,   /* RD_RCD */
            0x00000006,   /* WR_RCD */
            0x00000003,   /* RRD */
            0x00000003,   /* REXT */
            0x00000002,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000003,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x0000045f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000004,   /* PDEX2WR */
            0x00000004,   /* PDEX2RD */
            0x00000007,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000e,   /* RW2PDEN */
            0x0000002A,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000F,   /* TFAW */
            0x00000008,   /* TRPAB */
            0x00000005,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000004E1,   /* TREFBW */
            0x00000005,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xE03C048B,   /* CFG_DIG_DLL */
            0x007FC010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000001B,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    }
};


/* --- Function Implementations ---*/
NvOdmDebugConsole NvOdmQueryDebugConsole(void)
{
    return NvOdmDebugConsole_UartD;
}

NvOdmDownloadTransport NvOdmQueryDownloadTransport(void)
{
    return NvOdmDownloadTransport_None;
}

const NvU8* NvOdmQueryDeviceNamePrefix(void)
{
    return (const NvU8 *) "Tegra";
}

const NvOdmQuerySpiDeviceInfo *NvOdmQuerySpiGetDeviceInfo(
    NvOdmIoModule Module,
    NvU32 Instance,
    NvU32 ChipSelect)
{
    static const NvOdmQuerySpiDeviceInfo spi0_cs0 =
        { NvOdmQuerySpiSignalMode_0, NV_TRUE, NV_FALSE, NV_FALSE, 0, 0 };

    if (Module==NvOdmIoModule_Spi && Instance==0 && ChipSelect==0)
        return &spi0_cs0;

    return NULL;
}

const NvOdmQuerySpiIdleSignalState *NvOdmQuerySpiGetIdleSignalState(
    NvOdmIoModule Module,
    NvU32 Instance)
{
    static const NvOdmQuerySpiIdleSignalState spi[] =
    {
        { NV_FALSE, NvOdmQuerySpiSignalMode_0, NV_FALSE },
    };

    if (Module != NvOdmIoModule_Spi)
        return NULL;

    if (Instance >= NV_ARRAY_SIZE(spi))
        return NULL;

    return &spi[Instance];
}

const NvOdmQueryI2sInterfaceProperty *NvOdmQueryI2sGetInterfaceProperty(
    NvU32 Instance)
{
    static const NvOdmQueryI2sInterfaceProperty i2s[] =
    {
        {
            NvOdmQueryI2sMode_Master,
            NvOdmQueryI2sLRLineControl_LeftOnLow,
            NvOdmQueryI2sDataCommFormat_I2S,
            NV_FALSE, 0
        },
        {
            NvOdmQueryI2sMode_Master,
            NvOdmQueryI2sLRLineControl_LeftOnLow,
            NvOdmQueryI2sDataCommFormat_I2S,
            NV_FALSE, 0
        }
    };

    if (Instance >= NV_ARRAY_SIZE(i2s))
        return NULL;

    return &i2s[Instance];
}

const NvOdmQueryDapPortProperty *NvOdmQueryDapPortGetProperty(NvU32 Instance)
{
    static const NvOdmQueryDapPortProperty dap[] =
    {
        { NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Reserved

        // I2S1 (DAC1) <-> DAP1 <-> HIFICODEC
        {
            NvOdmDapPort_I2s1, NvOdmDapPort_HifiCodecType,
            { 2, 16, 44100, NvOdmQueryI2sDataCommFormat_I2S }
        },

        { NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Dap2
        { NvOdmDapPort_None, NvOdmDapPort_None , {0, 0, 0, 0} }, // Dap3

        // I2S2 (DAC2) <-> DAP4 <-> BLUETOOTH
        {
            NvOdmDapPort_I2s2, NvOdmDapPort_BlueTooth,
            { 2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S }
        }
    };

    if (Instance && Instance<NV_ARRAY_SIZE(dap))
        return &dap[Instance];

    return NULL;
}

const NvOdmQueryDapPortConnection *NvOdmQueryDapPortGetConnectionTable(
    NvU32 Index)
{
    static const NvOdmQueryDapPortConnection connect[] =
    {
        // the Default Music Path
        { 
            NvOdmDapConnectionIndex_Music_Path, 2,
            {
                { NvOdmDapPort_I2s1, NvOdmDapPort_Dap1, NV_TRUE },
                { NvOdmDapPort_Dap1, NvOdmDapPort_I2s1, NV_FALSE }
            }
        },

        // Bluetooth to Codec
        {
            NvOdmDapConnectionIndex_BlueTooth_Codec, 3,
            {
                { NvOdmDapPort_Dap4, NvOdmDapPort_I2s1, NV_TRUE },
                { NvOdmDapPort_I2s1, NvOdmDapPort_Dap4, NV_FALSE },
                { NvOdmDapPort_I2s2, NvOdmDapPort_Dap1, NV_FALSE }
            }
        }
    };

    NvU32 i = 0;
    for(i = 0; i < NV_ARRAY_SIZE(connect); i++)
    {
        if (connect[i].UseIndex == Index)
            return &connect[i];
    }
    return NULL;
}

const NvOdmQuerySpdifInterfaceProperty *NvOdmQuerySpdifGetInterfaceProperty(
    NvU32 Instance)
{
    static const NvOdmQuerySpdifInterfaceProperty spdif =
        { NvOdmQuerySpdifDataCaptureControl_FromLeft };

    return (Instance == 0) ? &spdif : NULL;
}

const NvOdmQueryAc97InterfaceProperty *NvOdmQueryAc97GetInterfaceProperty(
    NvU32 Instance)
{
    static const NvOdmQueryAc97InterfaceProperty ac97 =
        { NV_FALSE, NV_FALSE, NV_FALSE, NV_FALSE, NV_TRUE };

    return (Instance == 0) ? &ac97 : NULL;
}

const NvOdmQueryI2sACodecInterfaceProp *NvOdmQueryGetI2sACodecInterfaceProperty(
    NvU32 Instance)
{
    static const NvOdmQueryI2sACodecInterfaceProp i2s_codec[] =
    {
        {
            NV_FALSE, 0, 0x36, NV_FALSE,
            NvOdmQueryI2sLRLineControl_LeftOnLow,
            NvOdmQueryI2sDataCommFormat_I2S
        },
    };

    if (Instance < NV_ARRAY_SIZE(i2s_codec))
        return &i2s_codec[Instance];

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

const void* NvOdmQuerySdramControllerConfigGet(
    NvU32 *pEntries,
    NvU32 *pRevision)
{
#if NVODM_ENABLE_EMC_DVFS
    NvOdmBoardInfo BoardInfo;

    if (NvOdmPeripheralGetBoardInfo(BOARD_ID_VENTANA, &BoardInfo))
    {
        if (pRevision)
            *pRevision = s_NvOdmPM275ElpidaEmcConfigTable[0].Revision;
        if (pEntries)
            *pEntries = NV_ARRAY_SIZE(s_NvOdmPM275ElpidaEmcConfigTable);
        return (const void*)s_NvOdmPM275ElpidaEmcConfigTable;
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
    return (const NvU8 *) "NVIDIA";
}

const NvU8* NvOdmQueryModel(void)
{
    return (const NvU8 *) "AP20";
}

const NvU8* NvOdmQueryPlatform(void)
{
    return (const NvU8 *) "Ventana";
}

const NvU8* NvOdmQueryProjectName(void)
{
    return (const NvU8 *) "ODM Kit";
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

    // Pull ups for the kbc and sdio1 pins
    { NvOdmPinRegister_Ap20_PullUpDown_E,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_E(0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x2, 0x0, 0x0, 0x2, 0x0, 0x2, 0x2) },

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
    pPmuProperty->IrqConnected = NV_FALSE;
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

const NvOdmUsbProperty *NvOdmQueryGetUsbProperty(
    NvOdmIoModule Module,
    NvU32 Instance)
{
    static const NvOdmUsbProperty usb[] =
    {
        {
            NvOdmUsbInterfaceType_Utmi,
            (NvOdmUsbChargerType_SE0 | NvOdmUsbChargerType_SE1 |
             NvOdmUsbChargerType_SK),
            20, NV_TRUE, NvOdmUsbModeType_Device, NvOdmUsbIdPinType_CableId,
            NvOdmUsbConnectorsMuxType_None, NV_TRUE, { 0, 0, 0, 0 }
        },
        {
            NvOdmUsbInterfaceType_UlpiExternalPhy,
            NvOdmUsbChargerType_UsbHost,
            20, NV_TRUE, NvOdmUsbModeType_Host, NvOdmUsbIdPinType_None,
            NvOdmUsbConnectorsMuxType_None, NV_TRUE, { 0, 0, 0, 0 }
        },
        {
            NvOdmUsbInterfaceType_Utmi,
            NvOdmUsbChargerType_UsbHost,
            20, NV_TRUE, NvOdmUsbModeType_Host, NvOdmUsbIdPinType_None,
            NvOdmUsbConnectorsMuxType_None, NV_TRUE, { 0, 0, 0, 0 }
        },
    };

    if (Module != NvOdmIoModule_Usb || Instance >= NV_ARRAY_SIZE(usb))
        return NULL;

    return &usb[Instance];
}

const NvOdmQuerySdioInterfaceProperty* NvOdmQueryGetSdioInterfaceProperty(
    NvU32 Instance)
{
    static const NvOdmQuerySdioInterfaceProperty sdio[] =
    {
        { NV_FALSE, 10, NV_TRUE, 0x8, NvOdmQuerySdioSlotUsage_wlan   },
        { NV_FALSE,  0, NV_FALSE, 0, NvOdmQuerySdioSlotUsage_unused  },
        { NV_TRUE,   0, NV_FALSE, 0x6, NvOdmQuerySdioSlotUsage_Media },
        { NV_FALSE,  0, NV_FALSE, 0x4, NvOdmQuerySdioSlotUsage_Boot  },
    };

    if (Instance < NV_ARRAY_SIZE(sdio))
        return &sdio[Instance];

    return NULL;
}

const NvOdmQueryHsmmcInterfaceProperty* NvOdmQueryGetHsmmcInterfaceProperty(
    NvU32 Instance)
{
    return NULL;
}

NvU32 NvOdmQueryGetBlockDeviceSectorSize(NvOdmIoModule OdmIoModule)
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

NvU32 NvOdmQueryMemSize(NvOdmMemoryType MemType)
{
    switch (MemType)
    {
    case NvOdmMemoryType_Sdram:
        return 0x40000000;
    case NvOdmMemoryType_Nor:
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
    return 0x08000000; // 128 MB
}

NvU32 NvOdmQuerySecureRegionSize(void)
{
    return 0;
}
NvOdmNandFlashParams *NvOdmNandGetFlashInfo (NvU32 ReadID)
{
    return NULL;
}

void NvOdmKbcGetParameter(
    NvOdmKbcParameter Param,
    NvU32 SizeOfValue,
    void * pValue)
{
}

NvU32 NvOdmKbcGetKeyCode(
    NvU32 Row,
    NvU32 Column,
    NvU32 RowCount,
    NvU32 ColumnCount)
{
    return 0;
}

NvBool NvOdmKbcIsSelectKeysWkUpEnabled(
    NvU32 **pRowNumber,
    NvU32 **pColNumber,
    NvU32 *NumOfKeys)
{
    return NV_FALSE;
}
