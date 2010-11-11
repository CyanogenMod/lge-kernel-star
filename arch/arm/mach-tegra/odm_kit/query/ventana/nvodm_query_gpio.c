/*
 * arch/arm/mach-tegra/odm_kit/query/ventana/nvodm_query_gpio.c
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
#include "nvrm_drf.h"
#include <linux/input.h>

#define NVODM_PORT(x) ((x) - 'a')

static const NvOdmGpioPinInfo s_vi[] = {
    {NVODM_PORT('i'), 6, NvOdmGpioPinActiveState_High}, // EN_VDDIO_SD
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

    /* Panel 5 -- Harmony E1162 LVDS interface */
    { NVODM_PORT('b'), 2, NvOdmGpioPinActiveState_High },   // Enable (LVDS_SHTDN_N) (LO:OFF, HI:ON)
    { NVODM_PORT('w'), 0, NvOdmGpioPinActiveState_High },   // EN_VDD_BL
    { NVODM_PORT('d'), 4, NvOdmGpioPinActiveState_High },   // LCD_BL_EN
    { NVODM_PORT('c'), 6, NvOdmGpioPinActiveState_High },   // EN_VDD_PNL
    { NVODM_PORT('u'), 5, NvOdmGpioPinActiveState_High },   // LCD_BL_PWM
};

static const NvOdmGpioPinInfo s_hdmi[] =
{
    /* hdmi hot-plug interrupt pin */
    { NVODM_PORT('n'), 7, NvOdmGpioPinActiveState_High },    // HDMI HPD
};

static const NvOdmGpioPinInfo s_sdio[] = {
    {NVODM_PORT('i'), 5, NvOdmGpioPinActiveState_Low},    // Card Detect for SDIO instance 2
    /* High for WP and low for read/write */
    {NVODM_PORT('h'), 1, NvOdmGpioPinActiveState_High},    // Write Protect for SDIO instance 2
};

static const NvOdmGpioPinInfo s_Bluetooth[] = {
    {NVODM_PORT('u'), 0, NvOdmGpioPinActiveState_Low}   // BT_RST#
};

static const NvOdmGpioPinInfo s_Wlan[] = {
    {NVODM_PORT('k'), 5, NvOdmGpioPinActiveState_Low},  // WF_PWDN#
    {NVODM_PORT('k'), 6, NvOdmGpioPinActiveState_Low}   // WF_RST#
};

static const NvOdmGpioPinInfo s_Power[] = {
    // lid open/close, High = Lid Closed
    {NVODM_PORT('u'), 5, NvOdmGpioPinActiveState_High},
    // power button
    {NVODM_PORT('v'), 2, NvOdmGpioPinActiveState_Low}
};

// Gpio based keypad
static const NvOdmGpioPinKeyInfo s_GpioPinKeyInfo[] = {
    {KEY_MENU, 10, NV_TRUE},
    {KEY_HOME, 10, NV_TRUE},
    {KEY_BACK, 10, NV_TRUE},
    {KEY_VOLUMEDOWN, 10, NV_TRUE},
    {KEY_VOLUMEUP, 10, NV_TRUE},
};


// Gpio based keypad
static const NvOdmGpioPinInfo s_GpioKeyBoard[] = {
    {NVODM_PORT('c'), 7, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[0]},
    {NVODM_PORT('q'), 1, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[1]},
    {NVODM_PORT('q'), 2, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[2]},
    {NVODM_PORT('q'), 4, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[3]},
    {NVODM_PORT('q'), 5, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[4]},
    {NVODM_PORT('v'), 2, NvOdmGpioPinActiveState_Low, (void *)&s_GpioPinKeyInfo[0]},
};

static const NvOdmGpioPinInfo s_Battery[] = {
    // Low Battery
    {NVODM_PORT('w'), 3, NvOdmGpioPinActiveState_Low},
    {NVODM_PORT('r'), 6, NvOdmGpioPinActiveState_Low},
};
const NvOdmGpioPinInfo *NvOdmQueryGpioPinMap(NvOdmGpioPinGroup Group,
    NvU32 Instance, NvU32 *pCount)
{
    switch (Group)
    {
        case NvOdmGpioPinGroup_Display:
            *pCount = NV_ARRAY_SIZE(s_display);
            return s_display;

        case NvOdmGpioPinGroup_Hdmi:
            *pCount = NV_ARRAY_SIZE(s_hdmi);
            return s_hdmi;

        case NvOdmGpioPinGroup_Sdio:
            if (Instance == 2)
            {
                *pCount = NV_ARRAY_SIZE(s_sdio);
                return s_sdio;
            }
            else
            {
                *pCount = 0;
                return NULL;
            }

        case NvOdmGpioPinGroup_Bluetooth:
            *pCount = NV_ARRAY_SIZE(s_Bluetooth);
            return s_Bluetooth;

        case NvOdmGpioPinGroup_Wlan:
            *pCount = NV_ARRAY_SIZE(s_Wlan);
            return s_Wlan;

        case NvOdmGpioPinGroup_Vi:
            *pCount = NV_ARRAY_SIZE(s_vi);
            return s_vi;

        case NvOdmGpioPinGroup_keypadMisc:
                *pCount = NV_ARRAY_SIZE(s_GpioKeyBoard);
                return s_GpioKeyBoard;

        case NvOdmGpioPinGroup_Battery:
            *pCount = NV_ARRAY_SIZE(s_Battery);
            return s_Battery;

        default:
            *pCount = 0;
            return NULL;
    }
}
