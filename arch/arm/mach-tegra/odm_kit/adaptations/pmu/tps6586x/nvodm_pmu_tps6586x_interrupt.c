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


#include "nvodm_pmu_tps6586x_interrupt.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "nvodm_pmu_tps6586x_supply_info_table.h"
#include "nvodm_services.h"
#include "nvodm_pmu_tps6586x_batterycharger.h"

#define TPS6586X_INT_BATT_INST 0x01
#define TPS6586X_INT_PACK_COLD_DET 0x02
#define TPS6586X_INT_PACK_HOT_DET 0x04

#define TPS6586X_INT_USB_DETECTION 0x04
#define TPS6586X_INT_AC_DETECTION  0x08
#define TPS6586X_INT_LOWSYS_DETECTION 0x40

NvBool Tps6586xSetupInterrupt(NvOdmPmuDeviceHandle  hDevice,
                              TPS6586xStatus *pmuStatus)
{
    NvBool status = NV_FALSE;
    NvU32   data = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    /* Init Pmu Status */
    pmuStatus->lowBatt    = NV_FALSE;
    pmuStatus->highTemp   = NV_FALSE;

    if (!Tps6586xBatteryChargerMainChgPresent(hDevice,&status))
        return NV_FALSE;
    pmuStatus->mChgPresent = status;

    /* Set up Interrupt Mask */
    /* Mask1 */
    data = 0xFF;
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_RB0_INT_MASK1, data ))
        return NV_FALSE;

    /* Mask2 */
    data = 0xFF;
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_RB1_INT_MASK2, data ))
        return NV_FALSE;

    /* Mask3: Battery detction, etc */
    data = 0;
    data = (NvU32)~(TPS6586X_INT_BATT_INST|TPS6586X_INT_PACK_COLD_DET|TPS6586X_INT_PACK_HOT_DET);
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_RB2_INT_MASK3, data ))
        return NV_FALSE;

    /* Mask4 */
    data = 0xFF;
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_RB3_INT_MASK4, data ))
        return NV_FALSE;

    /* Mask5: USB Detection; AC Detection; Low System detection;  */
    data = 0;
    data = (NvU32) ~(TPS6586X_INT_USB_DETECTION|TPS6586X_INT_AC_DETECTION|TPS6586X_INT_LOWSYS_DETECTION);
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_RB4_INT_MASK5, data ))
        return NV_FALSE;

    return NV_TRUE;
}

void Tps6586xInterruptHandler_int(NvOdmPmuDeviceHandle  hDevice,
                                  TPS6586xStatus *pmuStatus)
{
    NvU32   data = 0;

    /* INT_ACK1 */
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB5_INT_ACK1, &data))
    {
        return;
    }
    pmuStatus->powerGood = (data & 0xFF);


    /* INT_ACK2 */
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB6_INT_ACK2, &data))
    {
        return;
    }
    pmuStatus->powerGood |= ((data & 0xFF)<<8);

    /* INT_ACK3 */
    /* LOW SYS */
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB7_INT_ACK3, &data))
    {
        return;
    }
    if (data != 0)
    {
        if (data&0x40)
        {
            pmuStatus->highTemp = NV_TRUE;
        }
        if (data&0xc0)
        {
            pmuStatus->mChgPresent = NV_TRUE;
#if !defined(CONFIG_TEGRA_ODM_HARMONY)
            NvOdmEnableOtgCircuitry(NV_TRUE);
#endif
        }
    }

    /* INT_ACK4 */
    /* CHG TEMP */
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB8_INT_ACK4, &data))
    {
        return;
    }
    if (data != 0)
    {
        if (data&0x02)
        {
            pmuStatus->lowBatt = NV_TRUE;
        }
    }
}
