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
 * @b Description: Specifies the peripheral connectivity
 *                 database NvOdmIoAddress entries for the E1129
 *                 keypad module.
 */
#include "../nvodm_query_kbc_gpio_def.h"

// Key Pad
static const NvOdmIoAddress s_KeyPadAddresses[] =
{
    // instance = 1 indicates Column info.
    // instance = 0 indicates Row info.
    // address holds KBC pin number used for row/column.

    // All Row info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd,0x00, NvOdmKbcGpioPin_KBRow0}, // Row 0
    { NvOdmIoModule_Kbd,0x00, NvOdmKbcGpioPin_KBRow1}, // Row 1
    { NvOdmIoModule_Kbd,0x00 ,NvOdmKbcGpioPin_KBRow2}, // Row 2

    // All Column info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd,0x01, NvOdmKbcGpioPin_KBCol0}, // Column 0
    { NvOdmIoModule_Kbd,0x01, NvOdmKbcGpioPin_KBCol1}, // Column 1
};

// s_ffa ScrollWheel...  only supported for personality 1
static const NvOdmIoAddress s_ffaScrollWheelAddresses[] =
{
    { NvOdmIoModule_Gpio, 0x10, 0x3 }, // GPIO Port q - Pin3
    { NvOdmIoModule_Gpio, 0x11, 0x3 }, // GpIO Port r - Pin 3
    { NvOdmIoModule_Gpio, 0x10, 0x5 }, // GPIO Port q - Pin 5
    { NvOdmIoModule_Gpio, 0x10, 0x4 }, // GPIO Port q - Pin 4
};

