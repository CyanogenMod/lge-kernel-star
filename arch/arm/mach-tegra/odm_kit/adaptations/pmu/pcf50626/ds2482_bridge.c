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

