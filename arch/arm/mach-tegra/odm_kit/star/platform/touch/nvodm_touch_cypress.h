/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  CYPRESS TMA300 Touch Device ODM Driver

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

#ifndef INCLUDED_NVODM_TOUCH_CYPRESS_H
#define INCLUDED_NVODM_TOUCH_CYPRESS_H

#include "nvodm_touch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif



NvBool Cypress_Open(NvOdmTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema);

void Cypress_Close(NvOdmTouchDeviceHandle hDevice);

NvBool Cypress_EnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

NvBool Cypress_HandleInterrupt(NvOdmTouchDeviceHandle hDevice);

NvBool Cypress_ReadCoordinate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord);

void Cypress_GetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities);

NvBool Cypress_GetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate);

NvBool Cypress_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate);

NvBool Cypress_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff);

NvBool Cypress_PowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode);

NvBool Cypress_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer);

// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [START]
void Cypress_InterruptMask(NvOdmTouchDeviceHandle hDevice, NvBool mask);
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [END]

#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_NVODM_TOUCH_CYPRESS_H
