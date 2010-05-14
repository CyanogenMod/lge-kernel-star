/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

