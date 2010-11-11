/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e888_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E888 audio module
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

// Audio Codec
static const NvOdmIoAddress s_AudioCodecAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO7, 0},  /* AUDIO_PLL etc -> DCD2 */
    { NvOdmIoModule_ExternalClock, 0, 0, 0 },                  // connected to CDEV1
#if 1
    { NvOdmIoModule_Spi, 2,     1, 0 },                      /* FFA Audio codec on SP3- CS1*/
#else
    { NvOdmIoModule_I2c_Pmu, 0, 0x34, 0},          /* FFA Audio codec on DVC*/

#endif
    { NvOdmIoModule_Dap, 0, 0, 0 },                            /* Dap port Index 0 is used for codec*/
};
