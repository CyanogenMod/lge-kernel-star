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

#include "max8907b.h"
#include "max8907b_adc.h"
#include "max8907b_i2c.h"
#include "max8907b_reg.h"

NvBool 
Max8907bAdcVBatSenseRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *volt)
{
#if 0
    NvU32 timeout = 0;
    NvU8  dataS1  = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);

    // Configure ADC (conversion cycle, resolution, etc...)

    // Start conversion

    // Wait for conversion

    // Make sure conversion is complete (or timeout or error)

    // Get result
    *volt = << compute >>;

#endif
    *volt = 0;
    return NV_TRUE;
}

NvBool 
Max8907bAdcVBatTempRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *volt)
{
#if 0
    NvU32 timeout = 0;
    NvU8  dataS1  = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);

    // Setup ADC (resolution, etc..)

    // Start conversion

    // Wait for conversion

    // Make sure conversion is complete (or timeout or error)

    // Get result
    *volt = << compute >>;

#endif
    *volt = 0;
    return NV_TRUE;
}

NvU32
Max8907bBatteryTemperature(
    NvU32 VBatSense,
    NvU32 VBatTemp)
{
    return 0;
}

