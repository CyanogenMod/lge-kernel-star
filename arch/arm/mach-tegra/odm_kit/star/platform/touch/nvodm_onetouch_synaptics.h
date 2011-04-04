/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  SYNAPTICS SO340010-16QFN Touch Device ODM Driver

EXTERNALIZED FUNCTIONS



INITIALIZATION AND SEQUENCING REQUIREMENTS

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================
                        EDIT HISTORY FOR MODULE

Date			Author				Descriptions
----------		-----------			------------------------------------
2010/09/19		hyeongwon.oh			code create

===========================================================================*/

/*===========================================================================

                     INCLUDE FILES FOR ODM DRIVER

===========================================================================*/

#ifndef INCLUDED_NVODM_ONETOUCH_SYNAPTICS_H
#define INCLUDED_NVODM_ONETOUCH_SYNAPTICS_H

#include "nvodm_onetouch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif

NvBool Synaptics_OneTouch_Open(NvOdmOneTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema);

void Synaptics_OneTouch_Close(NvOdmOneTouchDeviceHandle hDevice);

NvBool Synaptics_OneTouch_EnableInterrupt(NvOdmOneTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

NvBool Synaptics_OneTouch_HandleInterrupt(NvOdmOneTouchDeviceHandle hDevice);

NvBool Synaptics_OneTouch_ReadButton(NvOdmOneTouchDeviceHandle hDevice, NvOdmOneTouchButtonInfo *button);

NvBool Synaptics_OneTouch_SleepMode(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);

NvBool Synaptics_OneTouch_PowerOnOff(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);

void Synaptics_OneTouch_InterruptMask(NvOdmOneTouchDeviceHandle hDevice, NvBool mask);

#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_NVODM_ONETOUCH_SYNAPTICS_H
