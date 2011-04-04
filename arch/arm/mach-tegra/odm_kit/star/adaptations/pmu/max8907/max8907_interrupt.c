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

#include "max8907.h"
#include "max8907_interrupt.h"
#include "max8907_i2c.h"
#include "max8907_reg.h"
#include "max8907_batterycharger.h"
#include "nvodm_services.h"

NvBool 
Max8907SetupInterrupt(
    NvOdmPmuDeviceHandle hDevice)
{
    Max8907Status *pmuStatus;
    NvBool status = NV_FALSE;
    NvU8   data   = 0;

    NV_ASSERT(hDevice);
    pmuStatus = &((Max8907PrivData*)hDevice->pPrivate)->pmuStatus;
    
    /* Init Pmu Status */
    pmuStatus->lowBatt       = NV_FALSE;
    pmuStatus->highTemp      = NV_FALSE;

    if (!Max8907BatteryChargerMainBatt(hDevice, &status))
        return NV_FALSE;
    pmuStatus->mChgPresent = status;

    //20100413, cs77.ha@lge.com, temporary power key - Clear Interrupt
    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ1, &data)){        
        return NV_FALSE;
    }
    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ2, &data)){        
        return NV_FALSE;
    } 
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ1, &data)){        
        return NV_FALSE;
    }
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ2, &data)){        
        return NV_FALSE;
    } 
    if (!Max8907I2cRead8(hDevice, MAX8907_RTC_IRQ, &data)){        
        return NV_FALSE;
    } 

    /* Set up Interrupt Mask */
    // CHG_IRQ1
    //20100517, cs77.ha@lge.com, VCHG detect [START]
    #ifndef CONFIG_MACH_STAR
    data = ~(((NvU8)MAX8907_CHG_IRQ1_VCHG_R_MASK<<MAX8907_CHG_IRQ1_VCHG_R_SHIFT) |
            ((NvU8)MAX8907_CHG_IRQ1_VCHG_F_MASK<<MAX8907_CHG_IRQ1_VCHG_F_SHIFT));
    #else
    data = 0xff;
    #endif
    //20100517, cs77.ha@lge.com, VCHG detect [START]
    if (!Max8907I2cWrite8(hDevice, MAX8907_CHG_IRQ1_MASK, data))
        return NV_FALSE;

    // CHG_IRQ2
    //20100517, cs77.ha@lge.com, Batt Low detect [START]
    #ifndef CONFIG_MACH_STAR
    data = ~(((NvU8)MAX8907_CHG_IRQ2_MBATTLOW_R_MASK<<MAX8907_CHG_IRQ2_MBATTLOW_R_SHIFT) |
            ((NvU8)MAX8907_CHG_IRQ2_MBATTLOW_F_MASK<<MAX8907_CHG_IRQ2_MBATTLOW_F_SHIFT));
    #else
    data = 0xff;
    #endif
    //20100517, cs77.ha@lge.com, Batt Low detect [START]
    if (!Max8907I2cWrite8(hDevice, MAX8907_CHG_IRQ2_MASK, data))
        return NV_FALSE;

    // ON_OFF_IRQ1    
    data = 0xff;
    if (!Max8907I2cWrite8(hDevice, MAX8907_ON_OFF_IRQ1_MASK, data))
        return NV_FALSE;
    
    // ON_OFF_IRQ2
    data = 0xff;
    if (!Max8907I2cWrite8(hDevice, MAX8907_ON_OFF_IRQ2_MASK, data))
        return NV_FALSE;

    // RTC_IRQ
    data = 0xff;
    if (!Max8907I2cWrite8(hDevice, MAX8907_RTC_IRQ_MASK, data))
        return NV_FALSE;

    return NV_TRUE;
}

void 
Max8907InterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice)
{
    Max8907Status *pmuStatus;
    NvU8 data = 0;

    NV_ASSERT(hDevice);
    
    pmuStatus = &((Max8907PrivData*)hDevice->pPrivate)->pmuStatus;

    /* Check which interrupt... */

    // CHG_IRQ1
    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ1, &data)){
        return;
    }

    //20100518, cs77.ha@lge.com, unused [START]
    #ifndef CONFIG_MACH_STAR
    if (data)
    {
        // VBUS connect interrupt
        if (data & (MAX8907_CHG_IRQ1_VCHG_R_MASK << MAX8907_CHG_IRQ1_VCHG_R_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_TRUE);
        }
        // VBUS dis-connect interrupt
        else if (data & (MAX8907_CHG_IRQ1_VCHG_F_MASK << MAX8907_CHG_IRQ1_VCHG_F_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_FALSE);
        }
    }
    #endif
    //20100518, cs77.ha@lge.com, unused [END]

    // CHG_IRQ2
    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ2, &data))
    {
        return;
    }
    //20100518, cs77.ha@lge.com, unused [START]
    #ifndef CONFIG_MACH_STAR
    if (data){
        if (data & MAX8907_CHG_IRQ2_CHG_DONE_MASK)
        {
            pmuStatus->mChgPresent = NV_TRUE;
            pmuStatus->batFull = NV_TRUE;
        }
        if (data & MAX8907_CHG_IRQ2_MBATTLOW_F_SHIFT)
        {
            pmuStatus->batFull = NV_FALSE;
        }
        if (data & MAX8907_CHG_IRQ2_THM_OK_F_MASK)
        {
            pmuStatus->highTemp = NV_TRUE;
        }
        if (data & MAX8907_CHG_IRQ2_THM_OK_R_MASK)
        {
            pmuStatus->highTemp = NV_FALSE;
        }
    }
    #endif
    //20100518, cs77.ha@lge.com, unused [END]

    // ON_OFF_IRQ1
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ1, &data))
    {
        return;
    }

    // ON_OFF_IRQ2
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ2, &data))
    {
        return;
    }

//20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
#ifdef CONFIG_MACH_STAR
    // RTC_IRQ
    if (!Max8907RtcI2cRead8(hDevice, MAX8907_RTC_IRQ, &data))
    {
        return;
    }
	
    if (data){
        if (data & MAX8907_RTC_IRQ_ALARM0_R)
        {
            NvOdmOsDebugPrintf("\n MAX8907_RTC_IRQ_ALARM0_R is detected ");
            //clear rtc interrupt by reading
            if (!Max8907RtcI2cRead8(hDevice, MAX8907_ALARM0_CNTL, &data))
            {
                return;
            }            
        }
    }
#else
    // RTC_IRQ
    if (!Max8907I2cRead8(hDevice, MAX8907_RTC_IRQ, &data))
    {
        return;
    }
#endif
//20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]

    return;
}

