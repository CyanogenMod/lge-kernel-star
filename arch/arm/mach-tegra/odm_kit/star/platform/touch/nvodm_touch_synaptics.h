/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  SYNAPTICS T1320 Touch Device ODM Driver

EXTERNALIZED FUNCTIONS



INITIALIZATION AND SEQUENCING REQUIREMENTS

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================
                        EDIT HISTORY FOR MODULE

Date			Author				Descriptions
----------		-----------			------------------------------------
2010/03/12		taewan.kim			code create
2010/05/03		joseph.jung			review

===========================================================================*/

/*===========================================================================

                     INCLUDE FILES FOR ODM DRIVER

===========================================================================*/

#ifndef INCLUDED_NVODM_TOUCH_SYNAPTICS_H
#define INCLUDED_NVODM_TOUCH_SYNAPTICS_H

#include "nvodm_touch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif



NvBool Synaptics_Open(NvOdmTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema);

void Synaptics_Close(NvOdmTouchDeviceHandle hDevice);

NvBool Synaptics_EnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

NvBool Synaptics_HandleInterrupt(NvOdmTouchDeviceHandle hDevice);

NvBool Synaptics_ReadCoordinate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord);

void Synaptics_GetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities);

NvBool Synaptics_GetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate);

NvBool Synaptics_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate);

NvBool Synaptics_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff);

NvBool Synaptics_PowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode);

NvBool Synaptics_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer);

// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [START]
void Synaptics_InterruptMask(NvOdmTouchDeviceHandle hDevice, NvBool mask);
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [END]


#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_NVODM_TOUCH_SYNAPTICS_H
