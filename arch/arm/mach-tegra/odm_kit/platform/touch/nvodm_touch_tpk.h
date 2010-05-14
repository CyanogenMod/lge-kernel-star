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

#ifndef INCLUDED_NVODM_TOUCH_TPK_H
#define INCLUDED_NVODM_TOUCH_TPK_H

#include "nvodm_touch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif

NvBool TPK_Open( NvOdmTouchDeviceHandle *hDevice);

void TPK_GetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities);

NvBool TPK_ReadCoordinate( NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord);

NvBool TPK_EnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

NvBool TPK_HandleInterrupt(NvOdmTouchDeviceHandle hDevice);

NvBool TPK_GetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate);

NvBool TPK_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate);

NvBool TPK_PowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode);

NvBool TPK_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer);

NvBool TPK_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff);

void TPK_Close( NvOdmTouchDeviceHandle hDevice);


#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_NVODM_TOUCH_TPK_H
