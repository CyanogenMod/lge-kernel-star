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

#include "nvodm_pmu_tps6586x_interrupt.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "nvodm_pmu_tps6586x_supply_info_table.h"
#include "nvodm_services.h"
#include "nvodm_pmu_tps6586x_batterycharger.h"

/* INT_MASK3 */
#define TPS6586X_INT_BATT_INST 0x01
#define TPS6586X_INT_PACK_COLD_DET 0x02
#define TPS6586X_INT_PACK_HOT_DET 0x04
/* INT_MASK4 */
#define TPS6586X_INT_ALM2_INST          0x02
/* INT_MASK5 */
#define TPS6586X_INT_USB_DETECTION      0x04
#define TPS6586X_INT_AC_DETECTION       0x08
#define TPS6586X_INT_ALM1_DETECTION     0x10
#define TPS6586X_INT_LOWSYS_DETECTION   0x40

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
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB7_INT_ACK3, &data))
    {
        return;
    }
    if (data != 0)
    {
        /* ACK_RTCALM1 */
        if (data&0x01)
        {
        /* printk("%s ACK_RTC_ALM_1 detect!!!\n", __func__); */
        }

        /* ACK_CHGTEMP */
        if (data&0x40)
        {
            pmuStatus->highTemp = NV_TRUE;
        }
        /* ACK_ACDET & ACK_USBDET */
        if (data&0x0C)
        {
            pmuStatus->mChgPresent = NV_TRUE;
        }
    }

    /* INT_ACK4 */
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RB8_INT_ACK4, &data))
    {
        return;
    }
    if (data != 0)
    {
        /* ACK_RTCALM2 */
        if (data&0x04)
        {
        /* printk("%s ACK_RTC_ALM_2 detect!!!\n", __func__); */
        }
        /* LOW SYS */
        if (data&0x02)
        {
            pmuStatus->lowBatt = NV_TRUE;
        }
    }
}
