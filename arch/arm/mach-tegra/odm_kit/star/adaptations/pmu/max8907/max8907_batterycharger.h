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

#ifndef INCLUDED_MAX8907_BATTERYCHARGER_HEADER
#define INCLUDED_MAX8907_BATTERYCHARGER_HEADER

/* the battery charger functions */
#if defined(__cplusplus)
extern "C"
{
#endif

/* check main battery presence */
NvBool
Max8907BatteryChargerMainBatt(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status);

/* check charger input voltage */
NvBool
Max8907BatteryChargerOK(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status);

/* check charger enable status */
NvBool
Max8907BatteryChargerEnabled(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status);

/* check main battery voltage status */
NvBool
Max8907BatteryChargerMainBattFull(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_MAX8907_BATTERYCHARGER_HEADER

