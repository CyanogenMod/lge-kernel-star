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


#include "pcf50626_interrupt.h"
#include "pcf50626_batterycharger.h"
#include "pcf50626_i2c.h"
#include "pcf50626_reg.h"
#include "nvodm_services.h"

NvBool Pcf50626SetupInterrupt(NvOdmPmuDeviceHandle  hDevice,
                                  Pcf50626Status *pmuStatus)
{
    NvBool status = NV_FALSE;
    NvU8   data   = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    /* Init Pmu Status */
    pmuStatus->lowBatt       = NV_FALSE;
    pmuStatus->highTemp      = NV_FALSE;
    pmuStatus->chgCcToCv     = NV_FALSE;

    if (!Pcf50626BatteryChargerMainChgPresent(hDevice,&status))
        return NV_FALSE;
    pmuStatus->mChgPresent = status;


    /* Set up Interrupt Mask */
    data = (NvU8) ~(PCF50626_INT1_LOWBATT | PCF50626_INT1_HIGHTEMP);
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT1M_ADDR, data ))
        return NV_FALSE;

    data = 0;
    data = (NvU8) ~PCF50626_INT2_VMAX;
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT2M_ADDR, data ))
        return NV_FALSE;

    data = 0;
    data = (NvU8) ~(PCF50626_INT3_MCHGINS | PCF50626_INT3_MCHGRM);
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT3M_ADDR, data ))
        return NV_FALSE;

    data = 0;
    data = (NvU8) ~(PCF50626_INT4_BATFUL | PCF50626_INT4_CHGRES);
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT4M_ADDR, data ))
        return NV_FALSE;

    //if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT5M_ADDR, 0xff))
    //    return NV_FALSE;

    //if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT6M_ADDR, 0xff))
    //    return NV_FALSE;

    //if(! Pcf50626I2cWrite8(hDevice, PCF50626_INT7M_ADDR, 0xff))
    //    return NV_FALSE;

    return NV_TRUE;
}

void Pcf50626InterruptHandler_int(NvOdmPmuDeviceHandle  hDevice,
                                  Pcf50626Status *pmuStatus)
{

    NvU8 data = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    // INT1
    if (!Pcf50626I2cRead8(hDevice, PCF50626_INT1_ADDR, &data))
    {
        NVODMPMU_PRINTF(("Error reading INT1"));
        return;
    }
    if (data != 0)
    {
        if (data & PCF50626_INT1_HIGHTEMP)
            pmuStatus->highTemp = NV_TRUE;
        else
            pmuStatus->highTemp = NV_FALSE;
        if (data & PCF50626_INT1_LOWBATT)
            pmuStatus->lowBatt = NV_TRUE;
        else
            pmuStatus->lowBatt = NV_FALSE;
    }

    // INT2
    if (!Pcf50626I2cRead8(hDevice, PCF50626_INT2_ADDR, &data))
    {
        NVODMPMU_PRINTF(("Error reading INT2"));
        return;
    }
    if (data != 0)
    {
        if (data & PCF50626_INT2_VMAX)
            pmuStatus->chgCcToCv = NV_TRUE;
        else
            pmuStatus->chgCcToCv = NV_FALSE;
    }

    // INT3
    if (!Pcf50626I2cRead8(hDevice, PCF50626_INT3_ADDR, &data))
    {
        NVODMPMU_PRINTF(("Error reading INT3"));
        return;
    }
    if (data != 0)
    {
        if (data & PCF50626_INT3_MCHGRM)
            pmuStatus->mChgPresent = NV_FALSE;

        if (data & PCF50626_INT3_MCHGINS)
        {
            pmuStatus->mChgPresent = NV_TRUE;
            NvOdmEnableOtgCircuitry(NV_TRUE);
        }
    }

    // INT4
    if (!Pcf50626I2cRead8(hDevice, PCF50626_INT4_ADDR, &data))
    {
        NVODMPMU_PRINTF(("Error reading INT4"));
        return;
    }
    if (data != 0)
    {
        if (data & PCF50626_INT4_CHGRES)
            pmuStatus->batFull = NV_FALSE;

        if (data & PCF50626_INT4_BATFUL)
            pmuStatus->batFull = NV_TRUE;
    }

}


