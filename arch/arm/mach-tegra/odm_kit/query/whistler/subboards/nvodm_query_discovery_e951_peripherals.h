/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e951_peripherals.h
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

// Bluetooth on COMMs Module
{
     NV_ODM_GUID('l','b','e','e','9','q','m','b'),
     ffaBluetoothAddresses,
     NV_ARRAY_SIZE(ffaBluetoothAddresses),
     NvOdmPeripheralClass_Other
},

// Sdio wlan  on COMMs Module
{
    NV_ODM_GUID('s','d','i','o','w','l','a','n'),
    s_ffaWlanAddresses,
    NV_ARRAY_SIZE(s_ffaWlanAddresses),
    NvOdmPeripheralClass_Other
},

// EMP Modem on COMMs Module
{
    NV_ODM_GUID('e','m','p',' ','_','m','d','m'),
    s_ffaEmpAddresses,
    NV_ARRAY_SIZE(s_ffaEmpAddresses),
    NvOdmPeripheralClass_Other
},

// EMP M570 Modem on COMMs Module
{
    NV_ODM_GUID('e','m','p',' ','M','5','7','0'),
    s_ffaEmpM570Addresses,
    NV_ARRAY_SIZE(s_ffaEmpM570Addresses),
    NvOdmPeripheralClass_Other
},

// IFX Modem on COMMs Module
{
    NV_ODM_GUID('s','p','i',' ','_','i','p','c'),
    s_ffaInfnAddresses,
    NV_ARRAY_SIZE(s_ffaInfnAddresses),
    NvOdmPeripheralClass_Other
},

// NOTE: This list *must* end with a trailing comma.
