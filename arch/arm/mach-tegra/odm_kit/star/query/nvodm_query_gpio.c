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

#include "nvodm_query_gpio.h"
#include "nvodm_services.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"
#include <linux/input.h>

#define NVODM_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define NVODM_PORT(x) ((x) - 'a')

/*
static const NvOdmGpioPinInfo s_vi[] = {
    {NVODM_PORT('t'), 3, NvOdmGpioPinActiveState_High},
};
*/

static const NvOdmGpioPinInfo s_display[] = {
    { NVODM_PORT('v'), 7, NvOdmGpioPinActiveState_High },   // LCD_RESET_N
    { NVODM_PORT('r'), 3, NvOdmGpioPinActiveState_High },   // BL_DCDC_RST_N
};

static const NvOdmGpioPinInfo s_Sdio2[] = {
    {NVODM_PORT('i'), 5, NvOdmGpioPinActiveState_Low},    // Card Detect for SDIO instance 2
};

static const NvOdmGpioPinInfo s_Bluetooth[] = {
    {NVODM_PORT('q'), 4, NvOdmGpioPinActiveState_Low},  // Bluetooth Controls: BT_EN
    {NVODM_PORT('c'), 7, NvOdmGpioPinActiveState_High},  // Bluetooth Controls: BT_HOST_WAKEUP
    {NVODM_PORT('x'), 4, NvOdmGpioPinActiveState_High},  // Bluetooth Controls: BT_WAKEUP
};

static const NvOdmGpioPinInfo s_Wlan[] = {
    {NVODM_PORT('q'), 3, NvOdmGpioPinActiveState_Low},  // Wlan Controls: EN
    {NVODM_PORT('s'), 0, NvOdmGpioPinActiveState_High},  // Wlan Controls: WLAN_HOST_WAKEUP
    {NVODM_PORT('g'), 2, NvOdmGpioPinActiveState_High},  // Wlan Controls: WAKEUP
};

static const NvOdmGpioPinInfo s_hdmi[] =
{
    /* hdmi hot-plug interrupt pin */
    { NVODM_PORT('n'), 7, NvOdmGpioPinActiveState_Low },
};

//20100802  gpio key mapping [START]
#if defined(CONFIG_KEYBOARD_GPIO)
static const NvOdmGpioPinKeyInfo s_key_gpio_map[] = {
    {KEY_VOLUMEDOWN, 10, NV_TRUE},
    {KEY_VOLUMEUP, 10, NV_TRUE},
};

static const NvOdmGpioPinInfo s_nvgpio_key_info[] = {
    {NVODM_PORT('g'), 0, NvOdmGpioPinActiveState_Low, &s_key_gpio_map[0]},
    {NVODM_PORT('g'), 1, NvOdmGpioPinActiveState_Low, &s_key_gpio_map[1]},
};
#endif
//20100802  gpio key mapping [END]

const NvOdmGpioPinInfo *NvOdmQueryGpioPinMap(NvOdmGpioPinGroup Group,
    NvU32 Instance, NvU32 *pCount)
{
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
            *pCount = 0;
            return NULL;

        case NvOdmGpioPinGroup_NandFlash:
            *pCount = 0;
            return NULL;

        case NvOdmGpioPinGroup_Bluetooth:
            *pCount = NVODM_ARRAY_SIZE(s_Bluetooth);
            return s_Bluetooth;

        case NvOdmGpioPinGroup_Wlan:
            *pCount = NVODM_ARRAY_SIZE(s_Wlan);
            return s_Wlan;

        case NvOdmGpioPinGroup_SpiEthernet:
            *pCount = 0;
            return NULL;
#if 0
        case NvOdmGpioPinGroup_Vi:
            *pCount = NVODM_ARRAY_SIZE(s_vi);
            return s_vi;
#endif
        case NvOdmGpioPinGroup_Hdmi:
            *pCount = NVODM_ARRAY_SIZE(s_hdmi);
            return s_hdmi;

        //20100802  gpio key mapping [START]
#if defined(CONFIG_KEYBOARD_GPIO) 
        case NvOdmGpioPinGroup_keypadMisc:
            *pCount = NVODM_ARRAY_SIZE(s_nvgpio_key_info);
            return s_nvgpio_key_info;
#endif
        //20100802  gpio key mapping [END]

        default:
            *pCount = 0;
            return NULL;
    }
}
