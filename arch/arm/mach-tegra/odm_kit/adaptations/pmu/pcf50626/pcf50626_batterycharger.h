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

#ifndef INCLUDED_PCF50626_BATTERYCHARGER_HEADER
#define INCLUDED_PCF50626_BATTERYCHARGER_HEADER



#include "pcf50626.h"

/* the battery charger functions */
#if defined(__cplusplus)
extern "C"
{
#endif

/* Initliase all registers that related to battery charger */
NvBool 
Pcf50626BatteryChargerSetup(NvOdmPmuDeviceHandle hDevice);

/* Get battery Voltage */
NvBool 
Pcf50626BatteryChargerGetVoltage(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *res);


/* check OnKey level */
NvBool
Pcf50626BatteryChargerOnKeyStatus(
     NvOdmPmuDeviceHandle hDevice, 
     NvBool *status);

/* check rec1 level */
NvBool
Pcf50626BatteryChargerRec1Status(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check battery status */
NvBool
Pcf50626BatteryChargerBattStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check main charger status */
NvBool
Pcf50626BatteryChargerMainChgPresent(
     NvOdmPmuDeviceHandle hDevice, 
     NvBool *status);

/* check USB charger status */
NvBool
Pcf50626BatteryChargerUsbChgPresent(
     NvOdmPmuDeviceHandle hDevice, 
     NvBool *status);

/* check temparature status */
NvBool
Pcf50626BatteryChargerTempStatus(
     NvOdmPmuDeviceHandle hDevice, 
     NvBool *status);


/* check CBC batt_ful status */
NvBool
Pcf50626BatteryChargerCBCBattFul(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);



/* check CBC thermal limit activation status */
NvBool
Pcf50626BatteryChargerCBCTlimStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check CBC batt_ful status */
NvBool
Pcf50626BatteryChargerCBCWdExpired(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);


/* check CBC charger current status */
NvBool
Pcf50626BatteryChargerCBCChgCurStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check CBC charger voltage status */
NvBool
Pcf50626BatteryChargerCBCChgVoltStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check CBC charger resume status */
NvBool
Pcf50626BatteryChargerCBCChgResStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check CBC main batt presence */
NvBool
Pcf50626BatteryChargerCBCMainBatt(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check USB suspend status */
NvBool
Pcf50626BatteryChargerCBCUsbSuspStat(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);

/* check charger over-voltage protection status */
NvBool
Pcf50626BatteryChargerCBCChgOvpStat(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool *status);



#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PCF50626_BATTERYCHARGER_HEADER

