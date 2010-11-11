/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1129_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1129 keypad module
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

// Key Pad
{
    NV_ODM_GUID('k','e','y','b','o','a','r','d'),
    s_KeyPadAddresses,
    NV_ARRAY_SIZE(s_KeyPadAddresses),
    NvOdmPeripheralClass_HCI
},

// Scroll Wheel
{
    NV_ODM_GUID('s','c','r','o','l','w','h','l'),
    s_ffaScrollWheelAddresses,
    NV_ARRAY_SIZE(s_ffaScrollWheelAddresses),
    NvOdmPeripheralClass_HCI
},

// NOTE: This list *must* end with a trailing comma.
