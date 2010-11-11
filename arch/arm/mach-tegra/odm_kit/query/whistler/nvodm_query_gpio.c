/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/nvodm_query_gpio.c
 *
 * Copyright (c) 2009 NVIDIA Corporation.
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


#include "nvodm_query_gpio.h"
#include "nvodm_services.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"

#define NVODM_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define NVODM_PORT(x) ((x) - 'a')

static const NvOdmGpioPinInfo s_vi[] = {
    {NVODM_PORT('t'), 3, NvOdmGpioPinActiveState_High},
};

static const NvOdmGpioPinInfo s_display[] = {
    /* Panel 0 -- sony vga */
    { NVODM_PORT('m'), 3, NvOdmGpioPinActiveState_Low },
    { NVODM_PORT('b'), 2, NvOdmGpioPinActiveState_Low },
    { NVODM_PORT('n'), 4, NvOdmGpioPinActiveState_Low },
    { NVODM_PORT('j'), 3, NvOdmGpioPinActiveState_Low },
    { NVODM_PORT('j'), 4, NvOdmGpioPinActiveState_Low },
    // this pin is not needed for ap15
    {NVODM_GPIO_INVALID_PORT, NVODM_GPIO_INVALID_PIN,
        NvOdmGpioPinActiveState_Low},

    /* Panel 1 -- samtek */
    {NVODM_GPIO_INVALID_PORT, NVODM_GPIO_INVALID_PIN,
        NvOdmGpioPinActiveState_Low},
    {NVODM_GPIO_INVALID_PORT, NVODM_GPIO_INVALID_PIN,
        NvOdmGpioPinActiveState_Low},
    {NVODM_GPIO_INVALID_PORT, NVODM_GPIO_INVALID_PIN,
        NvOdmGpioPinActiveState_Low},
    {NVODM_GPIO_INVALID_PORT, NVODM_GPIO_INVALID_PIN,
        NvOdmGpioPinActiveState_Low},

    /* Panel 2 -- sharp wvga */
    { NVODM_PORT('v'), 7, NvOdmGpioPinActiveState_Low },

    /* Panel 3 -- sharp qvga */
    { NVODM_PORT('n'), 6, NvOdmGpioPinActiveState_High },   // LCD_DC0
    { NVODM_PORT('n'), 4, NvOdmGpioPinActiveState_Low },    // LCD_CS0
    { NVODM_PORT('b'), 3, NvOdmGpioPinActiveState_Low },    // LCD_PCLK
    { NVODM_PORT('b'), 2, NvOdmGpioPinActiveState_Low },    // LCD_PWR0
    { NVODM_PORT('e'), 0, NvOdmGpioPinActiveState_High },   // LCD_D0
    { NVODM_PORT('e'), 1, NvOdmGpioPinActiveState_High },   // LCD_D1
    { NVODM_PORT('e'), 2, NvOdmGpioPinActiveState_High },   // LCD_D2
    { NVODM_PORT('e'), 3, NvOdmGpioPinActiveState_High },   // LCD_D3
    { NVODM_PORT('e'), 4, NvOdmGpioPinActiveState_High },   // LCD_D4
    { NVODM_PORT('e'), 5, NvOdmGpioPinActiveState_High },   // LCD_D5
    { NVODM_PORT('e'), 6, NvOdmGpioPinActiveState_High },   // LCD_D6
    { NVODM_PORT('e'), 7, NvOdmGpioPinActiveState_High },   // LCD_D7
    { NVODM_PORT('f'), 0, NvOdmGpioPinActiveState_High },   // LCD_D8
    { NVODM_PORT('f'), 1, NvOdmGpioPinActiveState_High },   // LCD_D9
    { NVODM_PORT('f'), 2, NvOdmGpioPinActiveState_High },   // LCD_D10
    { NVODM_PORT('f'), 3, NvOdmGpioPinActiveState_High },   // LCD_D11
    { NVODM_PORT('f'), 4, NvOdmGpioPinActiveState_High },   // LCD_D12
    { NVODM_PORT('f'), 5, NvOdmGpioPinActiveState_High },   // LCD_D13
    { NVODM_PORT('f'), 6, NvOdmGpioPinActiveState_High },   // LCD_D14
    { NVODM_PORT('f'), 7, NvOdmGpioPinActiveState_High },   // LCD_D15
    { NVODM_PORT('m'), 3, NvOdmGpioPinActiveState_High },   // LCD_D19

     /* Panel 4 -- auo */
     { NVODM_PORT('v'), 7, NvOdmGpioPinActiveState_Low },
};

static const NvOdmGpioPinInfo s_Sdio2[] = {
    {NVODM_PORT('i'), 5, NvOdmGpioPinActiveState_Low},    // Card Detect for SDIO instance 2
    /* High for WP and low for read/write */
    {NVODM_PORT('v'), 5, NvOdmGpioPinActiveState_High},    // Write Protect for SDIO instance 2 
};

static const NvOdmGpioPinInfo s_NandFlash[] = {
    {NVODM_PORT('c'), 7, NvOdmGpioPinActiveState_High},     // MICRO SD_CD#
};

static const NvOdmGpioPinInfo s_spi_ethernet[] = {
    {NVODM_PORT('c'), 1, NvOdmGpioPinActiveState_Low}       // SPI_ENET_IRQ (EMC_INT)
};

static const NvOdmGpioPinInfo s_ScrollWheel[] = {
    {NVODM_PORT('q'), 4, NvOdmGpioPinActiveState_Low},  // Scroll wheel -QP1 (terminal  1)
    {NVODM_PORT('r'), 3, NvOdmGpioPinActiveState_Low},  // Scroll wheel_ONOFF
    {NVODM_PORT('q'), 5, NvOdmGpioPinActiveState_Low},  // Scroll wheel -SELECT (terminal 3)
    {NVODM_PORT('q'), 3, NvOdmGpioPinActiveState_Low},  // Scroll wheel -QP2 (terminal  4)
};

static const NvOdmGpioPinInfo s_ScrollWheel_TraceMode[] = {
    {NVODM_PORT('r'), 3, NvOdmGpioPinActiveState_Low},  // Scroll wheel_ONOFF
};

static const NvOdmGpioPinInfo s_Bluetooth[] = {
    {NVODM_PORT('u'), 0, NvOdmGpioPinActiveState_Low},  // Bluetooth Controls: BT_RST
};

static const NvOdmGpioPinInfo s_Wlan[] = {
    {NVODM_PORT('k'), 5, NvOdmGpioPinActiveState_Low},  // WLAN-OFF
    {NVODM_PORT('k'), 6, NvOdmGpioPinActiveState_Low},  // WLAN-RESET
};

static const NvOdmGpioPinInfo s_hdmi[] =
{
    /* hdmi hot-plug interrupt pin */
    { NVODM_PORT('n'), 7, NvOdmGpioPinActiveState_High},
};

const NvOdmGpioPinInfo *NvOdmQueryGpioPinMap(NvOdmGpioPinGroup Group,
    NvU32 Instance, NvU32 *pCount)
{
    NvU32 CustomerOption = 0;
    NvU32 Personality = 0;
    NvOdmServicesKeyListHandle hKeyList;

    hKeyList = NvOdmServicesKeyListOpen();
    if (hKeyList)
    {
        CustomerOption = 
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
        Personality =
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, PERSONALITY, CustomerOption);
    }

    if (!Personality)
        Personality = TEGRA_DEVKIT_DEFAULT_PERSONALITY;

    switch (Group)
    {
        case NvOdmGpioPinGroup_Display:
            *pCount = NVODM_ARRAY_SIZE(s_display);
            return s_display;

        case NvOdmGpioPinGroup_Sdio:
            if (Instance == 2)
            {
                *pCount = NVODM_ARRAY_SIZE(s_Sdio2);
                return s_Sdio2;
            }
            else
            {
                *pCount = 0;
                return NULL;
            }

        case NvOdmGpioPinGroup_ScrollWheel:
            if ((Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_11) ||
                (Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_15) ||
                (Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_C1))
            {
                *pCount = NVODM_ARRAY_SIZE(s_ScrollWheel_TraceMode);
                return s_ScrollWheel_TraceMode;
            }
            else
            {
                *pCount = NVODM_ARRAY_SIZE(s_ScrollWheel);
                return s_ScrollWheel;
            }

        case NvOdmGpioPinGroup_NandFlash:
            *pCount = NVODM_ARRAY_SIZE(s_NandFlash);
            return s_NandFlash;

        case NvOdmGpioPinGroup_Bluetooth:
            *pCount = NVODM_ARRAY_SIZE(s_Bluetooth);
            return s_Bluetooth;

        case NvOdmGpioPinGroup_Wlan:
            *pCount = NVODM_ARRAY_SIZE(s_Wlan);
            return s_Wlan;

        case NvOdmGpioPinGroup_SpiEthernet:
            *pCount = NVODM_ARRAY_SIZE(s_spi_ethernet);
            return s_spi_ethernet;
        case NvOdmGpioPinGroup_Vi:
            *pCount = NVODM_ARRAY_SIZE(s_vi);
            return s_vi;
        case NvOdmGpioPinGroup_Hdmi:
            *pCount = NVODM_ARRAY_SIZE(s_hdmi);
            return s_hdmi;

        default:
            *pCount = 0;
            return NULL;
    }
}
