/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e936_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E936 ISDB-T module
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

// VIP raw bitstream addresses
static const NvOdmIoAddress s_ffaVIPBitstreamAddresses[] =
{
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LX_V3, 0  },
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO9, 0 },
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO18, 0 },
    // Reset. vgp0 and vgp5:
    { NvOdmIoModule_Gpio, 27, 1, 0 },      // vgp[0] - Port BB(27), Pin 1
    { NvOdmIoModule_Gpio, 'd'-'a', 2, 0 }, // vgp[5] - Port D, Pin 2
};

