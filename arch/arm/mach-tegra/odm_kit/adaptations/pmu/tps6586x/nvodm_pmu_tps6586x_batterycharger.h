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

#ifndef INCLUDED_TPS6586X_BATTERYCHARGER_HEADER
#define INCLUDED_TPS6586X_BATTERYCHARGER_HEADER

#include "nvodm_pmu_tps6586x.h"

/* the battery charger functions */
#if defined(__cplusplus)
extern "C"
{
#endif

/* Initliase all registers that related to battery charger */
NvBool 
Tps6586xBatteryChargerSetup(NvOdmPmuDeviceHandle hDevice);

/* check CBC main batt presence */
NvBool
Tps6586xBatteryChargerCBCMainBatt(NvOdmPmuDeviceHandle hDevice, NvBool *status);

/* check batt_ful status */
NvBool
Tps6586xBatteryChargerCBCBattFul(NvOdmPmuDeviceHandle hDevice, NvBool *status);

/* check main charger status */
NvBool
Tps6586xBatteryChargerMainChgPresent(NvOdmPmuDeviceHandle hDevice, NvBool *status);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_BATTERYCHARGER_HEADER

