/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

/**
 * @file          NvAccelerometer.c
 * @brief         <b>Device Driver for Accelerometer</b>
 *
 * @Description : Implementation of the WinCE Accelerometer driver
 */

#include "nvodm_accelerometer.h"


NvBool
NvOdmAccelOpen(NvOdmAccelHandle* hDevice)
{
    *hDevice = NULL;
    return NV_FALSE;
}


void
NvOdmAccelClose(NvOdmAccelHandle hDevice)
{
}


/**
 * After setting the force threshold, we should remove all of interrupt flag
 * Which may be left from last threshold
 */
NvBool
NvOdmAccelSetIntForceThreshold(NvOdmAccelHandle  hDevice,
                               NvOdmAccelIntType IntType,
                               NvU32             IntNum,
                               NvU32             Threshold)
{
    return NV_FALSE;
}

/**
 * After setting the time threshold, we should remove all of interrupt flag
 * Which may be left from last threshold
 */
NvBool
NvOdmAccelSetIntTimeThreshold(NvOdmAccelHandle  hDevice,
                              NvOdmAccelIntType IntType,
                              NvU32             IntNum,
                              NvU32             Threshold)
{
    return NV_FALSE;
}


/**
 * After enable/disable threshold, we should remove all of interrupt flag
 * Which may be left from last threshold
 */
NvBool
NvOdmAccelSetIntEnable(NvOdmAccelHandle   hDevice,
                       NvOdmAccelIntType  IntType,
                       NvOdmAccelAxisType IntAxis,
                       NvU32              IntNum,
                       NvBool             Toggle)
{
    return NV_FALSE;
}


void
NvOdmAccelWaitInt(NvOdmAccelHandle    hDevice,
                  NvOdmAccelIntType  *IntType,
                  NvOdmAccelAxisType *IntMotionAxis,
                  NvOdmAccelAxisType *IntTapAxis)
{
}


void NvOdmAccelSignal(NvOdmAccelHandle hDevice)
{
}

NvBool
NvOdmAccelGetAcceleration(NvOdmAccelHandle hDevice,
                          NvS32           *AccelX,
                          NvS32           *AccelY,
                          NvS32           *AccelZ)
{
    return NV_FALSE;
}


NvOdmAccelerometerCaps
NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
    NvOdmAccelerometerCaps caps;
    NvOdmOsMemset(&caps, 0, sizeof(NvOdmAccelerometerCaps));

    return caps;
}


NvBool
NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    return NV_FALSE;
}


NvBool
NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    return NV_FALSE;
}

NvBool
NvOdmAccelSetPowerState(NvOdmAccelHandle hDevice, NvOdmAccelPowerType PowerState)
{
    return NV_FALSE;
}

