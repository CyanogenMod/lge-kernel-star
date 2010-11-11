/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1120_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1120 module
 * development system motherboard
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

//  Ethernet module
{
    NV_ODM_GUID('e','n','c','2','8','j','6','0'),
    s_enc28j60EthernetAddresses,
    NV_ARRAY_SIZE(s_enc28j60EthernetAddresses),
    NvOdmPeripheralClass_Other
},

//  Sdio module
{
    NV_ODM_GUID('s','d','i','o','_','m','e','m'),
    s_SdioAddresses,
    NV_ARRAY_SIZE(s_SdioAddresses),
    NvOdmPeripheralClass_Other,
},

//..Vibrate Module
{
    NV_ODM_GUID('v','i','b','r','a','t','o','r'),
    s_VibAddresses,
    NV_ARRAY_SIZE(s_VibAddresses),
    NvOdmPeripheralClass_Other,
},

// Accelerometer Module
{
    NV_ODM_GUID('a','c','c','e','l','e','r','o'),
    s_AcceleroAddresses,
    NV_ARRAY_SIZE(s_AcceleroAddresses),
    NvOdmPeripheralClass_Other,
},

// NOTE: This list *must* end with a trailing comma.
