/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#ifndef INCLUDED_PCF50626_RTC_HEADER
#define INCLUDED_PCF50626_RTC_HEADER

#include "pmu_hal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Read RTC count register */

NvBool 
Pcf50626RtcCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Read RTC alarm count register */

NvBool 
Pcf50626RtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Write RTC count register */

NvBool 
Pcf50626RtcCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Write RTC alarm count register */

NvBool 
Pcf50626RtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Reads RTC alarm interrupt mask status */

NvBool 
Pcf50626RtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice);

/* Enables / Disables the RTC alarm interrupt */

NvBool 
Pcf50626RtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool Enable);

/* Checks if boot was from nopower / powered state */

NvBool 
Pcf50626RtcWasStartUpFromNoPower(NvOdmPmuDeviceHandle hDevice);

NvBool
Pcf50626IsRtcInitialized(NvOdmPmuDeviceHandle hDevice);


#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PCF50626_RTC_HEADER

