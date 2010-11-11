/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1109_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1109 module
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
#include "tmon/adt7461/nvodm_tmon_adt7461_channel.h"
#include "nvodm_tmon.h"

static const NvOdmIoAddress s_ffaHdmiAddresses[] =
{
    { NvOdmIoModule_Hdmi, 0, 0, 0 },

    /* Display Data Channel (DDC) for Extended Display Identification
     * Data (EDID)
     */
    { NvOdmIoModule_I2c, 0x01, 0xA0, 0 },

    /* HDCP downstream */
    { NvOdmIoModule_I2c, 0x01, 0x74, 0 },

    /* AVDD_HDMI -> D1REG */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11, 0 },

    /* MIPI PLL */
    { NvOdmIoModule_Vdd, 0, Max8907bPmuSupply_LDO6, 0 },

    /* lcd i/o rail (for hot plug pin) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 },
};

static const NvOdmIoAddress s_ffaCrtAddresses[] =
{
    { NvOdmIoModule_Crt, 0, 0, 0 },

    /* Display Data Channel (DDC) for Extended Display Identification
     * Data (EDID)
     */
    { NvOdmIoModule_I2c, 0x01, 0xA0, 0 },

    /* tvdac rail (required) */
    { NvOdmIoModule_Vdd, 0x00,  Max8907bPmuSupply_LDO14, 0 },

    /* lcd i/o rail (for hot plug pin) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 },
};

static const NvOdmIoAddress s_ffaVideoDacAddresses[] =
{
    { NvOdmIoModule_Tvo, 0x00, 0x00, 0 },
    /* tvdac rail */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO14, 0 },
};

static const NvOdmIoAddress s_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98, 0 }, /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO15, 0 }, /* TMON pwer rail -> D4REG */
    { NvOdmIoModule_Gpio, 0x08, 0x02, 0 },                   /* GPIO Port I and Pin 2 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote, 0 },   /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local, 0 }, /* TSENSOR */
};

