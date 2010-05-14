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

