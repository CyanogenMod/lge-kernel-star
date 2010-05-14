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
