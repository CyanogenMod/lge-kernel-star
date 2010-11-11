/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1120_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1120
 * development system motherboard
 *
 * Copyright (c) 2010 NVIDIA Corporation.
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

static const NvOdmIoAddress s_enc28j60EthernetAddresses[] =
{
    { NvOdmIoModule_Spi, 1, 1, 0 },
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 1, 0 }
};

static const NvOdmIoAddress s_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0, 0 },
    { NvOdmIoModule_Sdio, 0x2, 0x0, 0 },
    { NvOdmIoModule_Sdio, 0x3, 0x0, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12, 0 }, /* VDDIO_SDIO -> VOUT12 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5, 0 } /* VCORE_MMC -> VOUT05 */
};

static const NvOdmIoAddress s_VibAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LDO16, 0 },
};

static const NvOdmIoAddress s_AcceleroAddresses[] =
{
    { NvOdmIoModule_I2c, 0x0, 0x3A, 0 }, /* I2C address (7-bit) 0x1D < 1 = 0x3A (8-bit) */
    { NvOdmIoModule_Gpio, 0x1A, 0x1, 0 }, /* Gpio port AA[1] = (A=0, Z=25) thus AA = 26 = 0x1A */
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LX_V3, 0 }, /* VDDIO_UART = V3 */
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LDO1, 0 }, /* VCORE_ACC = VOUT1 = 2.8v */
};

