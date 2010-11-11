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

#ifndef INCLUDED_TPS6586X_RTC_HEADER
#define INCLUDED_TPS6586X_RTC_HEADER

#include "nvodm_pmu_tps6586x.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Read RTC count register */

NvBool 
Tps6586xRtcCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Read RTC alarm count register */

NvBool 
Tps6586xRtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count);

/* Write RTC count register */

NvBool 
Tps6586xRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Write RTC alarm count register */

NvBool 
Tps6586xRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count);

/* Reads RTC alarm interrupt mask status */

NvBool 
Tps6586xRtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice);

/* Enables / Disables the RTC alarm interrupt */

NvBool 
Tps6586xRtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool Enable);

/* Checks if boot was from nopower / powered state */

NvBool 
Tps6586xRtcWasStartUpFromNoPower(NvOdmPmuDeviceHandle hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_RTC_HEADER

