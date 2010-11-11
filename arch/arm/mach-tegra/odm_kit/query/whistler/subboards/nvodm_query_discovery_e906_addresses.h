/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e906_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E906 LCD module
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

#include "pmu/max8907b/max8907b_supply_info_table.h"

// Main LCD
static const NvOdmIoAddress s_ffaMainDisplayAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0, 0 },
    { NvOdmIoModule_Spi, 0x2, 0x2, 0 },                        // TBD (this is a guess)
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 },   /* VDDIO_LCD -> V3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5, 0 },    /* AVDD_LCD_1 -> VOUT5 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19, 0  },  /* AVDD_LCD_2 -> VOUT19 */
};

// DSI LCD
// WARNING: Whistler's board personality needs to be set to 077 for the
// reset gpio pin to work
static const NvOdmIoAddress s_DsiAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0, 0 },

    { NvOdmIoModule_Gpio, (NvU32)('c' - 'a'), 1, 0 },

    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 },   /* VDDIO_LCD -> V3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5, 0 },    /* AVDD_LCD_1 -> VOUT5 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19, 0  },  /* AVDD_LCD_2 -> VOUT19 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO17, 0  },  /* MIPI DSI 1.2V */
};

// TouchPanel
static const NvOdmIoAddress s_ffaTouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0x20, 0 },/* I2C device address is 0x20 */
    { NvOdmIoModule_Gpio, 'c' - 'a', 6, 0}, /* GPIO Port V and Pin 3 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO19, 0 }
};
