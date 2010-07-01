/*
 * Copyright (c) 2010 NVIDIA Corporation.
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


