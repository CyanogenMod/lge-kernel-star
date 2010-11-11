/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#include "nvodm_priv_accelerometer.h"

NvBool NvOdmAccelOpen(NvOdmAccelHandle* hDevice)
{
    if(!bma150_init(hDevice))
    {
        NVODMACCELEROMETER_PRINTF(("\n BMA150 accel Open failed"));
    }
    else
    {
        NvOdmOsPrintf("\n Bosch accelerometer found");
        return NV_TRUE;
    }

    if(!kxtf9_init((hDevice)))
    {
        NVODMACCELEROMETER_PRINTF(("\n KXTF9 accel Open failed"));
    }
    else
    {
        NvOdmOsPrintf("\n Kionix accelerometer found");
        return NV_TRUE;
    }

    NVODMACCELEROMETER_PRINTF(("\n E1206: Error during NvOdmAccelOpen\n"));
    return NV_FALSE;
}

void NvOdmAccelClose(NvOdmAccelHandle hDevice)
{
    if (hDevice && hDevice->AccelClose)
        hDevice->AccelClose(hDevice);
}

NvBool NvOdmAccelSetIntForceThreshold(NvOdmAccelHandle  hDevice,
                               NvOdmAccelIntType IntType,
                               NvU32             IntNum,
                               NvU32             Threshold)
{
    if (hDevice && hDevice->AccelSetIntForceThreshold)
        return hDevice->AccelSetIntForceThreshold(hDevice, IntType, IntNum, Threshold);
    else
        return NV_FALSE;
}

NvBool
NvOdmAccelSetIntTimeThreshold(NvOdmAccelHandle  hDevice,
                              NvOdmAccelIntType IntType,
                              NvU32             IntNum,
                              NvU32             Threshold)
{
    if (hDevice && hDevice->AccelSetIntTimeThreshold)
        return hDevice->AccelSetIntTimeThreshold(hDevice, IntType, IntNum, Threshold);
    else
        return NV_FALSE;
}

NvBool
NvOdmAccelSetIntEnable(NvOdmAccelHandle  hDevice,
                           NvOdmAccelIntType  IntType,
                           NvOdmAccelAxisType IntAxis,
                           NvU32              IntNum,
                           NvBool             Toggle)
{
    if (hDevice && hDevice->AccelSetIntEnable)
        return hDevice->AccelSetIntEnable(hDevice, IntType, IntAxis, IntNum, Toggle);
    else
        return NV_FALSE;
}

void
NvOdmAccelWaitInt(NvOdmAccelHandle    hDevice,
                  NvOdmAccelIntType  *IntType,
                  NvOdmAccelAxisType *IntMotionAxis,
                  NvOdmAccelAxisType *IntTapAxis)
{
    if (hDevice && hDevice->AccelWaitInt)
        hDevice->AccelWaitInt(hDevice, IntType, IntMotionAxis, IntTapAxis);
}

void NvOdmAccelSignal(NvOdmAccelHandle hDevice)
{
    if (hDevice && hDevice->AccelSignal)
        hDevice->AccelSignal(hDevice);
}

NvBool NvOdmAccelGetAcceleration(NvOdmAccelHandle hDevice,
                          NvS32           *AccelX,
                          NvS32           *AccelY,
                          NvS32           *AccelZ)
{
    if (hDevice && hDevice->AccelGetAcceleration)
        return hDevice->AccelGetAcceleration(hDevice, AccelX, AccelY, AccelZ);
    else
        return NV_FALSE;
}

NvOdmAccelerometerCaps NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
    if (hDevice && hDevice->AccelGetCaps)
        return hDevice->AccelGetCaps(hDevice);
    else
    {
        NV_ASSERT(NULL != hDevice);
        return hDevice->Caption;
    }
}

NvBool NvOdmAccelSetPowerState(NvOdmAccelHandle hDevice,
                NvOdmAccelPowerType PowerState)
{
    if (hDevice && hDevice->AccelSetPowerState)
        return hDevice->AccelSetPowerState(hDevice, PowerState);
    else
        return NV_FALSE;
}

NvBool
NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice,
                             NvU32 SampleRate)
{
    if (hDevice && hDevice->AccelSetSampleRate)
        return hDevice->AccelSetSampleRate(hDevice, SampleRate);
    else
        return NV_FALSE;
}

NvBool
NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32* pSampleRate)
{
    if (hDevice && hDevice->AccelGetSampleRate)
        return hDevice->AccelGetSampleRate(hDevice, pSampleRate);
    else
        return NV_FALSE;
}


