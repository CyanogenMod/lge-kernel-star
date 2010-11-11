/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e906_peripheral.h
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

// LCD module
{
    NV_ODM_GUID('S','H','P','_','A','P','2','0'),   // Sharp WVGA panel with AP20 backlight control
    s_ffaMainDisplayAddresses,
    NV_ARRAY_SIZE(s_ffaMainDisplayAddresses),
    NvOdmPeripheralClass_Display,
},

// DSI module
{
    NV_ODM_GUID('s','h','a','r','p','d','s','i'),
    s_DsiAddresses,
    NV_ARRAY_SIZE(s_DsiAddresses),
    NvOdmPeripheralClass_Display,
},

//  Touch Panel
{
    NV_ODM_GUID('t','p','k','t','o','u','c','h'),
    s_ffaTouchPanelAddresses,
    NV_ARRAY_SIZE(s_ffaTouchPanelAddresses),
    NvOdmPeripheralClass_HCI
},

// NOTE: This list *must* end with a trailing comma.
