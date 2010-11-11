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

