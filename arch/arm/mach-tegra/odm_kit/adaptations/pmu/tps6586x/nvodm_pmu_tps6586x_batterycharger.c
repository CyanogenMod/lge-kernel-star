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

#include "nvodm_pmu_tps6586x_i2c.h"
#include "nvodm_pmu_tps6586x_batterycharger.h"

NvBool Tps6586xBatteryChargerSetup(NvOdmPmuDeviceHandle hDevice)
{
    NvU32 data = 0;
    // Configure CHARGER RAM registers
    // CHG1: Charge safety timer value is 4 Hrs; Charge current scaling facotr: 1.0; 
    data = 0x0c;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R49_CHG1, data))
        return NV_FALSE;
        
    // CHG2: CHARGE SAFETY TIMER: ON; CHARGE VOLTAGE: 4.2V; CHARGER: ON;
    data = 0x1a;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R4A_CHG2, data))
        return NV_FALSE;

    // CHG3:
    data = 0x0;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R4B_CHG3, data))
        return NV_FALSE;
        
    // RAM Control BITS: CHARGE VOLTAGE RANGE: 3.95 - 4.2; USB Input current limit: 500mA;
    // Auto mode enabled; AC input current limit: 2A
    data = 0x05;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R4C_PPATH2, data))
        return NV_FALSE;

    return NV_TRUE;
}

/* check CBC main batt presence */
NvBool
Tps6586xBatteryChargerCBCMainBatt(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU32 data = 0;
    
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB9_STAT1, &data))
        return NV_FALSE;

    // bit 0 show if battery exists or not
    data = data & 0x01;
    
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}

/* check batt_ful status */
NvBool
Tps6586xBatteryChargerCBCBattFul(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU32 data = 0;
    
    if(! Tps6586xI2cRead8(hDevice, TPS6586x_RBA_STAT2, &data))
        return NV_FALSE;

    data = data & 0x2;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}

/* check main charger status */
NvBool
Tps6586xBatteryChargerMainChgPresent(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU32 data = 0;
    
    if(! Tps6586xI2cRead8(hDevice, TPS6586x_RBB_STAT3, &data))
        return NV_FALSE;

    data = data & 0xc;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}
