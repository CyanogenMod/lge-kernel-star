/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: Specifies the peripheral connectivity database Peripheral entries
 *                 for the E911 5MP + 1.3MP + VGA VI Camera module.
 */
#include "../include/nvodm_imager_guids.h"

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
