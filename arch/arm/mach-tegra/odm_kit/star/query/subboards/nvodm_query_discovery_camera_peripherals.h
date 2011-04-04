/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e911_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E911 camera module
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

#include "../include/nvodm_imager_guids.h"

#if 1
{
    SN046F_GUID,
    s_ffaImagerSN046FAddresses,
    NV_ARRAY_SIZE(s_ffaImagerSN046FAddresses),
    NvOdmPeripheralClass_Imager
},
// focuser for Sharp 5MP module
{
    // Piezo motor driver IC 
    FOCUSER_GUID,
    s_ffaImagerDW9712Addresses,
    NV_ARRAY_SIZE(s_ffaImagerDW9712Addresses),
    NvOdmPeripheralClass_Other
},
#endif
{
    COMMONIMAGER_GUID,
    s_CommonImagerAddresses,
    NV_ARRAY_SIZE(s_CommonImagerAddresses),
    NvOdmPeripheralClass_Other
},
// ===> camera
#if 0
// !!! DON'T MOVE THINGS AROUND !!!
// Position 0 is used as default primary for E912
// Position 1 is used as default secondary for E912 and E911
// Position 2 is used as default primary for E911

// Imager - Primary  
// E912 A01 and Whistler Imager
{
    OV5630_GUID,
    s_ffaImagerOV5630Addresses,
    NV_ARRAY_SIZE(s_ffaImagerOV5630Addresses),
    NvOdmPeripheralClass_Imager
},
// Imager - Secondary
// sensor for SEMCO VGA
{
    // Aptina (Micron) SOC380
    SEMCOVGA_GUID,
    s_ffaImagerSOC380Addresses,
    NV_ARRAY_SIZE(s_ffaImagerSOC380Addresses),
    NvOdmPeripheralClass_Imager
},

// Dummy Entry for Whistler
{
    MI5130_GUID,
    s_ffaImagerOV5630Addresses,
    NV_ARRAY_SIZE(s_ffaImagerOV5630Addresses),
    NvOdmPeripheralClass_Imager
},

// focuser for OV5630 module
{
    // VCM driver IC AD5820 Analog Devices
    AD5820_GUID,
    s_ffaImagerAD5820Addresses,
    NV_ARRAY_SIZE(s_ffaImagerAD5820Addresses),
    NvOdmPeripheralClass_Other
},

// flash device 
{
    LTC3216_GUID,
    s_ffaFlashLTC3216Addresses,
    NV_ARRAY_SIZE(s_ffaFlashLTC3216Addresses),
    NvOdmPeripheralClass_Other
},

{
    COMMONIMAGER_GUID,
    s_CommonImagerAddresses,
    NV_ARRAY_SIZE(s_CommonImagerAddresses),
    NvOdmPeripheralClass_Other
},
#endif
