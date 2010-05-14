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
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: Specifies the peripheral connectivity 
 *                 database NvOdmIoAddress entries for the E906
 *                 LCD Module.
 */

#include "pmu/max8907b/max8907b_supply_info_table.h"

// Main LCD
static const NvOdmIoAddress s_ffaMainDisplayAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0 },
    { NvOdmIoModule_Spi, 0x2, 0x2 },                        // TBD (this is a guess)
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 },   /* VDDIO_LCD -> V3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5 },    /* AVDD_LCD_1 -> VOUT5 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19  },  /* AVDD_LCD_2 -> VOUT19 */
};

// DSI LCD
// WARNING: Whistler's board personality needs to be set to 077 for the
// reset gpio pin to work
static const NvOdmIoAddress s_DsiAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0 },

    { NvOdmIoModule_Gpio, (NvU32)('c' - 'a'), 1 },

    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 },   /* VDDIO_LCD -> V3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5 },    /* AVDD_LCD_1 -> VOUT5 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19  },  /* AVDD_LCD_2 -> VOUT19 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO17  },  /* MIPI DSI 1.2V */
};

// TouchPanel
static const NvOdmIoAddress s_ffaTouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0x20 },/* I2C device address is 0x20 */
    { NvOdmIoModule_Gpio, 'c' - 'a', 6}, /* GPIO Port V and Pin 3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19 }
};
