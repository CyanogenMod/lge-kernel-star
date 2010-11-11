/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e951_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E951 COMMS module
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
// Bluetooth
static const NvOdmIoAddress ffaBluetoothAddresses[] =
{
    { NvOdmIoModule_Uart, 0x2,  0x0, 0 },
    { NvOdmIoModule_Gpio, 0x14, 0x0, 0 },     /* GPIO Port U and Pin 0 */
    { NvOdmIoModule_Vdd, 0x00, MIC2826PmuSupply_LDO3, 0 }  /* VDD -> MIC2826 LDO3 */
};


// Wlan
static const NvOdmIoAddress s_ffaWlanAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x01, 0x0, 0 },    /* WLAN is on SD Bus */
    { NvOdmIoModule_Gpio, 0x0a, 0x5, 0 },    /* GPIO Port K and Pin 5 */
    { NvOdmIoModule_Gpio, 0x0a, 0x6, 0 },    /* GPIO Port K and Pin 6 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO8, 0 },  /* VDD -> LDO8 (VOUT8) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11, 0 },  /* VDD -> LDO11 (VOUT11) */
    { NvOdmIoModule_Vdd, 0x00, MIC2826PmuSupply_LDO3, 0 }  /* VDD -> MIC2826 LDO3 */    
};

// EMP Rainbow module
static const NvOdmIoAddress s_ffaEmpAddresses[] =
{
    { NvOdmIoModule_Uart, 0x0,  0x0, 0 },                      /* UART 0 */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 },                      /* GPIO Port V and Pin 1 Power */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 AWR */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 CWR */
    { NvOdmIoModule_Gpio, 0x0e, 0x6, 0 },                      /* GPIO Port O and Pin 6 SpiInt */
    { NvOdmIoModule_Gpio, 0x15, 0x2, 0 },                      /* GPIO Port V and Pin 2 SpiSlaveSelect */
    { NvOdmIoModule_Slink, 0x0,  0x0, 0 }                      /* Slink 0 */
};

// EMP M570 module
static const NvOdmIoAddress s_ffaEmpM570Addresses[] =
{
    { NvOdmIoModule_Uart, 0x0,  0x0, 0 },                      /* UART 0 */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 },                      /* GPIO Port V and Pin 1 Power */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 AWR */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 CWR */
};

// IFX Modem module
static const NvOdmIoAddress s_ffaInfnAddresses[] =
{
    { NvOdmIoModule_Spi, 0x0,  0x0, 0 },                      /* Spi Controller 0 and Chip Select 0 */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 SRDY */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 MRDY */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 }                      /* GPIO Port V and Pin 1 Power */
};

