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

#ifndef INCLUDED_NVODM_TOUCH_PANJIT_H
#define INCLUDED_NVODM_TOUCH_PANJIT_H

#include "nvodm_touch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct PANJIT_TouchDevice_Rec
{
    NvOdmTouchDevice OdmTouch;
    NvOdmTouchCapabilities Caps;
    NvOdmServicesI2cHandle hOdmI2c;
    NvOdmServicesGpioHandle hGpio;
    NvOdmServicesPmuHandle hPmu;
    NvOdmGpioPinHandle hPin;
    NvOdmGpioPinHandle hResetPin;
    NvOdmServicesGpioIntrHandle hGpioIntr;
    NvOdmOsSemaphoreHandle hIntSema;
    NvU32 PrevFingers;
    NvU32 DeviceAddr;
    NvU32 SampleRate;
    NvU32 SleepMode;
    NvBool PowerOn;
    NvU32 I2cClockSpeedKHz;
} PANJIT_TouchDevice;

/**
 * Gets a handle to the touch pad in the system.
 *
 * @param hDevice A pointer to the handle of the touch pad.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool PANJIT_Open(NvOdmTouchDeviceHandle *hDevice);

/**
 *  Releases the touch pad handle.
 *
 * @param hDevice The touch pad handle to be released. If
 *     NULL, this API has no effect.
 */
void PANJIT_Close(NvOdmTouchDeviceHandle hDevice);

/**
 * Gets capabilities for the specified touch device.
 *
 * @param hDevice The handle of the touch pad.
 * @param pCapabilities A pointer to the targeted
 *  capabilities returned by the ODM.
 */
void
PANJIT_GetCapabilities(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchCapabilities* pCapabilities);

/**
 * Gets coordinate info from the touch device.
 *
 * @param hDevice The handle to the touch pad.
 * @param coord  A pointer to the structure holding coordinate info.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
PANJIT_ReadCoordinate(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchCoordinateInfo *pCoord);

/**
 * Hooks up the interrupt handle to the GPIO interrupt and enables the interrupt.
 *
 * @param hDevice The handle to the touch pad.
 * @param hInterruptSemaphore A handle to hook up the interrupt.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
PANJIT_EnableInterrupt(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmOsSemaphoreHandle hInterruptSemaphore);

/**
 * Prepares the next interrupt to get notified from the touch device.
 *
 * @param hDevice A handle to the touch pad.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool PANJIT_HandleInterrupt(NvOdmTouchDeviceHandle hDevice);

/**
 * Gets the touch ADC sample rate.
 *
 * @param hDevice A handle to the touch ADC.
 * @param pTouchSampleRate A pointer to the NvOdmTouchSampleRate stucture.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
*/
NvBool
PANJIT_GetSampleRate(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchSampleRate* pTouchSampleRate);

/**
 * Sets the touch ADC sample rate.
 *
 * @param hDevice A handle to the touch ADC.
 * @param SampleRate 1 indicates high frequency, 0 indicates low frequency.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
*/
NvBool PANJIT_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate);

/**
 * Sets the touch panel power mode.
 *
 * @param hDevice A handle to the touch ADC.
 * @param mode The mode, ranging from full power to power off.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
*/
NvBool
PANJIT_PowerControl(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchPowerModeType mode);

/**
 * Gets the touch panel calibration data.
 * This is optional as calibration may perform after the OS is up.
 * This is not required to bring up the touch panel.
 *
 * @param hDevice A handle to the touch panel.
 * @param NumOfCalibrationData Indicates the number of calibration points.
 * @param pRawCoordBuffer The collection of X/Y coordinate data.
 *
 * @return NV_TRUE if preset calibration data is required, or NV_FALSE otherwise.
 */
NvBool
PANJIT_GetCalibrationData(
    NvOdmTouchDeviceHandle hDevice,
    NvU32 NumOfCalibrationData,
    NvS32* pRawCoordBuffer);

/**
 * Powers the touch device on or off.
 *
 * @param hDevice A handle to the touch ADC.
 * @param OnOff  Specify 1 to power ON, 0 to power OFF.
*/
NvBool PANJIT_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVODM_TOUCH_PANJIT_H

