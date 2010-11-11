/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1109_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1116 Processor module
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


// HDMI
{
    NV_ODM_GUID('f','f','a','_','h','d','m','i'),
    s_ffaHdmiAddresses,
    NV_ARRAY_SIZE(s_ffaHdmiAddresses),
    NvOdmPeripheralClass_Display
},

// CRT
{
    NV_ODM_GUID('f','f','a','_','-','c','r','t'),
    s_ffaCrtAddresses,
    NV_ARRAY_SIZE(s_ffaCrtAddresses),
    NvOdmPeripheralClass_Display
},

// TV Out Video Dac
{
    NV_ODM_GUID('f','f','a','t','v','o','u','t'),
    s_ffaVideoDacAddresses,
    NV_ARRAY_SIZE(s_ffaVideoDacAddresses),
    NvOdmPeripheralClass_Display
},

// Temperature Monitor (TMON)
{
    NV_ODM_GUID('a','d','t','7','4','6','1',' '),
    s_Tmon0Addresses,
    NV_ARRAY_SIZE(s_Tmon0Addresses),
    NvOdmPeripheralClass_Other
},
 
// NOTE: This list *must* end with a trailing comma.
