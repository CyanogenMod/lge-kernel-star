/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#ifndef INCLUDED_NVODM_ONETOUCH_INT_H
#define INCLUDED_NVODM_ONETOUCH_INT_H

#include "nvodm_services.h"
#include "nvodm_onetouch.h"
#include <linux/kernel.h>


// Module debug: 0=disable, 1=enable
#define NVODMTOUCH_ENABLE_PRINTF (0)

// 20100420 joseph.jung@lge.com Debugging Message Print Setting [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
#if (NVODMTOUCH_ENABLE_PRINTF)
#define NVODMTOUCH_PRINTF(x)   NvOdmOsPrintf x
#else
#define NVODMTOUCH_PRINTF(x)
#endif
#else
#if (NV_DEBUG && NVODMTOUCH_ENABLE_PRINTF)
#define NVODMTOUCH_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMTOUCH_PRINTF(x)
#endif
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100420 joseph.jung@lge.com Debugging Message Print Setting [END]

#if defined(__cplusplus)
extern "C"
{
#endif
 
typedef struct NvOdmOneTouchDeviceRec{
    NvBool (*ReadButton)	    (NvOdmOneTouchDeviceHandle hDevice, NvOdmOneTouchButtonInfo *button);
    NvBool (*EnableInterrupt)   (NvOdmOneTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);
    NvBool (*HandleInterrupt)   (NvOdmOneTouchDeviceHandle hDevice);
    NvBool (*SleepMode)        (NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);
    NvBool (*PowerOnOff)        (NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff);
    void   (*Close)             (NvOdmOneTouchDeviceHandle hDevice);
    NvU16                       CurrentSampleRate;
    NvBool                      OutputDebugMessage;
	void	(*InterruptMask)	(NvOdmOneTouchDeviceHandle hDevice, NvBool mask);
} NvOdmOneTouchDevice;



#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_ONETOUCH_INT_H

