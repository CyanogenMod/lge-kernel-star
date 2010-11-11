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

#ifndef INCLUDED_MAX8907B_RTC_HEADER
#define INCLUDED_MAX8907B_RTC_HEADER

#include "pmu_hal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Read RTC count register */

NvBool 
Max8907bRtcCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Read RTC alarm count register */

NvBool 
Max8907bRtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Write RTC count register */

NvBool 
Max8907bRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Write RTC alarm count register */

NvBool 
Max8907bRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Reads RTC alarm interrupt mask status */

NvBool 
Max8907bRtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice);

/* Enables / Disables the RTC alarm interrupt */

NvBool 
Max8907bRtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool Enable);

/* Checks if boot was from nopower / powered state */

NvBool 
Max8907bRtcWasStartUpFromNoPower(NvOdmPmuDeviceHandle hDevice);

NvBool
Max8907bIsRtcInitialized(NvOdmPmuDeviceHandle hDevice);


#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_MAX8907B_RTC_HEADER

