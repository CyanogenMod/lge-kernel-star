/*
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

