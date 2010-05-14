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

#ifndef INCLUDED_NVODM_TOUCH_INT_H
#define INCLUDED_NVODM_TOUCH_INT_H

#include "nvodm_services.h"
#include "nvodm_touch.h"


// Module debug: 0=disable, 1=enable
#define NVODMTOUCH_ENABLE_PRINTF (0)

#if (NV_DEBUG && NVODMTOUCH_ENABLE_PRINTF)
#define NVODMTOUCH_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMTOUCH_PRINTF(x)
#endif

#if defined(__cplusplus)
extern "C"
{
#endif
 
typedef struct NvOdmTouchDeviceRec{
    NvBool (*ReadCoordinate)    (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord);
    NvBool (*EnableInterrupt)   (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);
    NvBool (*HandleInterrupt)   (NvOdmTouchDeviceHandle hDevice);
    NvBool (*GetSampleRate)     (NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate);
    NvBool (*SetSampleRate)     (NvOdmTouchDeviceHandle hDevice, NvU32 rate);
    NvBool (*PowerControl)      (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode);
    NvBool (*PowerOnOff)        (NvOdmTouchDeviceHandle hDevice, NvBool OnOff);
    void   (*GetCapabilities)   (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities);
    NvBool (*GetCalibrationData)(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer);
    void   (*Close)             (NvOdmTouchDeviceHandle hDevice);
    NvU16                       CurrentSampleRate;
    NvBool                      OutputDebugMessage;
} NvOdmTouchDevice;



#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_TOUCH_INT_H

