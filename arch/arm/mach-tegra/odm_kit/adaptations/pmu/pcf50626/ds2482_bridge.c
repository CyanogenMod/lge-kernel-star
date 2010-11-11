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

#include "ds2482_bridge.h"
#include "pcf50626_i2c.h"
#include "ds2482_i2c.h"
#include "pcf50626_reg.h"
#include "ds2482_reg.h"

NvBool 
Ds2482Setup(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data = 0;
    
    // One wire I2C bridge
    //Device Reset Status
    if (!Ds2482OWI2cRead8(hDevice, DS2482_DEVICE_RESET, &data))
        return NV_FALSE;
    //NVODMPMU_PRINTF(("Device Reset reg 0x%02x = 0x%02x\n", DS2482_DEVICE_RESET, data));

    //1-Wire Reset Status
    if (!Ds2482OWI2cRead8(hDevice, DS2482_1WIRE_RESET, &data))
        return NV_FALSE;
    //NVODMPMU_PRINTF(("1-Wire Reset reg 0x%02x = 0x%02x\n", DS2482_1WIRE_RESET, data));

    while(1)
    {
        if (!Ds2482OWI2cWrite8(hDevice, DS2482_READ_DATA_REG_ADDR, DS2482_DEVICE_RESET))
            return NV_FALSE;

        if (!Ds2482OWI2cRead8(hDevice, DS2482_READ_DATA_REG_ADDR, &data))
            return NV_FALSE;

        if (!(data & 0x01))
            break;
    }


    return NV_TRUE;
}

