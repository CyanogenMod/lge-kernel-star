/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

