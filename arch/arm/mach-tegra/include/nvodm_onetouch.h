/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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
 * @file
 * <b>NVIDIA Tegra ODM Kit:
 *         Touch Pad Sensor Interface</b>
 *
 * @b Description: Defines the ODM adaptation interface for touch pad sensor devices.
 *
 */

#ifndef INCLUDED_NVODM_ONETOUCH_H
#define INCLUDED_NVODM_ONETOUCH_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"

/**
 * @defgroup nvodm_touch Touch Pad Adaptation Interface
 *
 * This is the touch pad ODM adaptation interface.
 *
 * @ingroup nvodm_adaptation
 * @{
 */


/**
 * Defines an opaque handle that exists for each touch device in the
 * system, each of which is defined by the customer implementation.
 */
typedef struct NvOdmOneTouchDeviceRec *NvOdmOneTouchDeviceHandle;

#ifdef CONFIG_MACH_STAR
#define FEATURE_LGE_TOUCH_CUSTOMIZE
#endif

#define 	OT_CONFIG_REG_START_ADDR		0x0000


#define		OT_DATA_REG_START_ADDR			0x0109
#define		OT_NUM_DATA_REG_BYTES			4		// (0x109 - 0x10A)*2

#define		OT_DATA_REG_START_ADDR_HIGH		0x01
#define		OT_DATA_REG_START_ADDR_LOW		0x09

#define		OT_DATA_REG_SLEEP_ADDR_HIGH		0x00
#define		OT_DATA_REG_SLEEP_ADDR_LOW		0x01

#define		OT_DATA_REG_SLEEP_DATA_HIGH		0x00
#define		OT_DATA_REG_SLEEP_DATA_LOW		0x80

#define		OT_DATA_REG_ACTIVE_DATA_HIGH		0x00
#define		OT_DATA_REG_ACTIVE_DATA_LOW		0x00

#define		OT_BUTTON_OFFSET				0

/**
 * Defines the ODM touch pad button information.
 */
typedef struct
{
		/// menu button.
		NvU8 menu;
		/// back button.
		NvU8 back;
} NvOdmOneTouchButtonInfo;

/**
 * Gets a handle to the touch pad in the system.
 *
 * @param hDevice A pointer to the handle of the touch pad.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
NvBool
NvOdmOneTouchDeviceOpen( NvOdmOneTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema);
#else
NvBool
NvOdmOneTouchDeviceOpen( NvOdmOneTouchDeviceHandle *hDevice);
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */

/**
 * Gets coordinate info from the touch device.
 *
 * @param hDevice The handle to the touch pad.
 * @param coord  A pointer to the structure holding coordinate info.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmOneTouchReadButton( NvOdmOneTouchDeviceHandle hDevice, NvOdmOneTouchButtonInfo *button);


/**
 * Hooks up the interrupt handle to the GPIO interrupt and enables the interrupt.
 *
 * @param hDevice The handle to the touch pad.
 * @param hInterruptSemaphore A handle to hook up the interrupt.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmOneTouchEnableInterrupt(NvOdmOneTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

/**
 * Prepares the next interrupt to get notified from the touch device.
 *
 * @param hDevice A handle to the touch pad.
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmOneTouchHandleInterrupt(NvOdmOneTouchDeviceHandle hDevice);

/**
 * Powers the touch device on or off.
 *
 * @param hDevice A handle to the touch ADC.
 * @param OnOff  Specify 1 to power on; 0 to power off.
*/
NvBool NvOdmOneTouchSleepMode(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);

/**
 * Powers the touch device on or off.
 *
 * @param hDevice A handle to the touch ADC.
 * @param OnOff  Specify 1 to power on; 0 to power off.
*/
void
NvOdmOneTouchPowerOnOff(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);

/**
 *  Releases the touch pad handle.
 *
 * @param hDevice The touch pad handle to be released. If
 *     NULL, this API has no effect.
 */
void NvOdmOneTouchDeviceClose(NvOdmOneTouchDeviceHandle hDevice);

/**
 * Gets the touch ODM debug configuration.
 *
 * @param hDevice A handle to the touch pad.
 *
 * @return NV_TRUE if debug message is enabled, or NV_FALSE if not.
 */
NvBool
NvOdmOneTouchOutputDebugMessage(NvOdmOneTouchDeviceHandle hDevice);

void
NvOdmOneTouchInterruptMask(NvOdmOneTouchDeviceHandle hDevice, NvBool mask);

#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_ONETOUCH_H
