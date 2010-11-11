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

#include "max8907b.h"
#include "max8907b_batterycharger.h"
#include "max8907b_reg.h"
#include "max8907b_i2c.h"

NvBool
Max8907bBatteryChargerMainBatt(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;
    
    if(!Max8907bI2cRead8(hDevice, MAX8907B_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907B_CHG_STAT_MBDET_SHIFT) & MAX8907B_CHG_STAT_MBDET_MASK;
    *status = (data == 0 ? NV_TRUE : NV_FALSE );    // MBDET low (0) = present
    return NV_TRUE;
}

NvBool
Max8907bBatteryChargerOK(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;

    if(!Max8907bI2cRead8(hDevice, MAX8907B_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907B_CHG_STAT_VCHG_OK_SHIFT) & MAX8907B_CHG_STAT_VCHG_OK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

NvBool
Max8907bBatteryChargerEnabled(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;

    if(!Max8907bI2cRead8(hDevice, MAX8907B_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907B_CHG_STAT_CHG_EN_STAT_SHIFT) & MAX8907B_CHG_STAT_CHG_EN_STAT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

NvBool
Max8907bBatteryChargerMainBattFull(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;

    if(!Max8907bI2cRead8(hDevice, MAX8907B_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907B_CHG_STAT_MBATTLOW_SHIFT) & MAX8907B_CHG_STAT_MBATTLOW_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

