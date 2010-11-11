/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1129_addresses.h
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

#include "../nvodm_query_kbc_gpio_def.h"

// Key Pad
static const NvOdmIoAddress s_KeyPadAddresses[] =
{
    // instance = 1 indicates Column info.
    // instance = 0 indicates Row info.
    // address holds KBC pin number used for row/column.

    // All Row info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd,0x00, NvOdmKbcGpioPin_KBRow0, 0}, // Row 0
    { NvOdmIoModule_Kbd,0x00, NvOdmKbcGpioPin_KBRow1, 0}, // Row 1
    { NvOdmIoModule_Kbd,0x00 ,NvOdmKbcGpioPin_KBRow2, 0}, // Row 2

    // All Column info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd,0x01, NvOdmKbcGpioPin_KBCol0, 0}, // Column 0
    { NvOdmIoModule_Kbd,0x01, NvOdmKbcGpioPin_KBCol1, 0}, // Column 1
};

// s_ffa ScrollWheel...  only supported for personality 1
static const NvOdmIoAddress s_ffaScrollWheelAddresses[] =
{
    { NvOdmIoModule_Gpio, 0x10, 0x3, 0 }, // GPIO Port q - Pin3
    { NvOdmIoModule_Gpio, 0x11, 0x3, 0 }, // GpIO Port r - Pin 3
    { NvOdmIoModule_Gpio, 0x10, 0x5, 0 }, // GPIO Port q - Pin 5
    { NvOdmIoModule_Gpio, 0x10, 0x4, 0 }, // GPIO Port q - Pin 4
};

