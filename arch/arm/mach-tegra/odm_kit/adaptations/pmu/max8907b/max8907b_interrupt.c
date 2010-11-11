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
#include "max8907b_interrupt.h"
#include "max8907b_i2c.h"
#include "max8907b_reg.h"
#include "max8907b_rtc.h"
#include "max8907b_batterycharger.h"
#include "nvodm_services.h"

NvBool
Max8907bSetupInterrupt(
    NvOdmPmuDeviceHandle hDevice,
    Max8907bStatus *pmuStatus)
{
    NvBool status = NV_FALSE;
    NvU8   data   = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    /* Init Pmu Status */
    pmuStatus->lowBatt       = NV_FALSE;
    pmuStatus->highTemp      = NV_FALSE;

    if (!Max8907bBatteryChargerMainBatt(hDevice, &status))
        return NV_FALSE;
    pmuStatus->mChgPresent = status;

    /* Set up Interrupt Mask */

    // CHG_IRQ1
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_CHG_IRQ1, data))
        return NV_FALSE;

    // CHG_IRQ2
    data = MAX8907B_CHG_IRQ2_CHG_DONE_MASK      |
           MAX8907B_CHG_IRQ2_MBATTLOW_F_SHIFT   |
           MAX8907B_CHG_IRQ2_THM_OK_F_MASK      |
           MAX8907B_CHG_IRQ2_THM_OK_R_MASK      ;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_CHG_IRQ2, data))
        return NV_FALSE;

    // ON_OFF_IRQ1
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_ON_OFF_IRQ1, data))
        return NV_FALSE;

    // ON_OFF_IRQ2
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_ON_OFF_IRQ2, data))
        return NV_FALSE;

    // disable ALARM0
    data = 0;
    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_ALARM0_CNTL, data))
        return NV_FALSE;

    // disable ALARM1
    data = 0;
    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_ALARM1_CNTL, data))
        return NV_FALSE;

    // clear RTC_IRQ
    if (!Max8907bRtcI2cRead8(hDevice, MAX8907B_RTC_IRQ, &data))
        return NV_FALSE;

    // RTC_IRQ_MASK - disable ALARM0 and ALARM1
    data =
        ( MAX8907B_RTC_IRQ_ALARM0_R_MASK << MAX8907B_RTC_IRQ_ALARM0_R_SHIFT ) |
        ( MAX8907B_RTC_IRQ_ALARM1_R_MASK << MAX8907B_RTC_IRQ_ALARM1_R_SHIFT );
    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_RTC_IRQ_MASK, data))
        return NV_FALSE;

    return NV_TRUE;
}

void
Max8907bInterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice,
    Max8907bStatus *pmuStatus)
{
    NvU8 data = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    /* Check which interrupt... */

    // CHG_IRQ1
    if (!Max8907bI2cRead8(hDevice, MAX8907B_CHG_IRQ1, &data))
    {
        return;
    }

    if (data)
    {
        // VBUS connect interrupt
        if (data &
           (MAX8907B_CHG_IRQ1_VCHG_R_MASK << MAX8907B_CHG_IRQ1_VCHG_R_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_TRUE);
        }
        // VBUS dis-connect interrupt
        else if (data &
                (MAX8907B_CHG_IRQ1_VCHG_F_MASK << MAX8907B_CHG_IRQ1_VCHG_F_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_FALSE);
        }
    }

    // CHG_IRQ2
    if (!Max8907bI2cRead8(hDevice, MAX8907B_CHG_IRQ2, &data))
    {
        return;
    }
    if (data)
    {
        if (data & MAX8907B_CHG_IRQ2_CHG_DONE_MASK)
        {
            pmuStatus->mChgPresent = NV_TRUE;
            pmuStatus->batFull = NV_TRUE;
        }
        if (data & MAX8907B_CHG_IRQ2_MBATTLOW_F_SHIFT)
        {
            pmuStatus->batFull = NV_FALSE;
        }
        if (data & MAX8907B_CHG_IRQ2_THM_OK_F_MASK)
        {
            pmuStatus->highTemp = NV_TRUE;
        }
        if (data & MAX8907B_CHG_IRQ2_THM_OK_R_MASK)
        {
            pmuStatus->highTemp = NV_FALSE;
        }
    }

    // ON_OFF_IRQ1
    if (!Max8907bI2cRead8(hDevice, MAX8907B_ON_OFF_IRQ1, &data))
    {
        return;
    }

    // ON_OFF_IRQ2
    if (!Max8907bI2cRead8(hDevice, MAX8907B_ON_OFF_IRQ2, &data))
    {
        return;
    }


    // RTC_STATUS
    if (!Max8907bRtcI2cRead8(hDevice, MAX8907B_RTC_STATUS, &data))
    {
        return;
    }

    // RTC_IRQ
    if (!Max8907bRtcI2cRead8(hDevice, MAX8907B_RTC_IRQ, &data))
    {
        return;
    }
    if (data)
    {
        if (data &
           (MAX8907B_RTC_IRQ_ALARM1_R_MASK << MAX8907B_RTC_IRQ_ALARM1_R_SHIFT))
        {
	    /* disable further alarms from this source. */
	    Max8907bRtcAlarmIntEnable(hDevice, 0);
	    if(hDevice->pfnAlarmInterrupt)
		hDevice->pfnAlarmInterrupt(hDevice);
	}
    }

    return;
}
