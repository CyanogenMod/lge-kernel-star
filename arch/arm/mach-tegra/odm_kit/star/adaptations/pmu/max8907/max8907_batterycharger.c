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
#include "max8907_batterycharger.h"
#include "max8907_reg.h"
#include "max8907_i2c.h"
#include "max8907_adc.h"

//20100529, jh.ahn@lge.com, For getting of Charger Information [START]
#include <linux/kernel.h>
#include <mach/lprintk.h>

//#define LG_DEBUG_PMUCHG
#undef LG_DEBUG_PMUCHG  // Define for Debug Serial

#ifdef LG_DEBUG_PMUCHG
#define LDPC(fmt, arg...) lprintk(D_CHARGER, "%s : " fmt "\n", __func__, ## arg)
#else
#define LDPC(fmt, arg...) do {} while (0)
#endif

#if defined (CONFIG_MACH_STAR)  && defined (CONFIG_STAR_BATTERY_CHARGER)
typedef enum {
  CHG_IC_DEFAULT_MODE=0,    		/* 0  */
  CHG_IC_TA_MODE,
  CHG_IC_USB_LO_MODE,
  CHG_IC_FACTORY_MODE,

  CHG_IC_DEACTIVE_MODE,			/* 4  */
  CHG_IC_INIT_MODE,
} max8922_status;

extern max8922_status get_charging_ic_status(void);
#endif // CONFIG_STAR_BATTERY_CHARGER
//20100529, jh.ahn@lge.com, For getting of Charger Information [END]

NvBool
Max8907BatteryChargerMainBatt(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    //20100529, jh.ahn@lge.com, Battery detection by battery Temp( -40`C ~ 90`C) [START]
#if defined (CONFIG_MACH_STAR)
    //read battery temp 
    NvU32 VBatTemp = 0;

    // Get VBatTemp
    if (!Max8907AdcVBatTempRead(hDevice, &VBatTemp))
    {
        LDPC("[Critical] Error reading VBatTempRead.");
        return NV_FALSE;
    }
    LDPC("[jh.ahn] BatTemp[Rthm]  = %d", VBatTemp);

    if ((309 > VBatTemp) || (4016 < VBatTemp)) // Temperatur -40'C ~ 90'C, if not, determine "No Battery Presence"
    {
        LDPC("[jh.ahn] No Battery Presence..");
        *status = NV_FALSE;
    }
    else 
    {
        LDPC("[jh.ahn] Battery Presence..");
        *status = NV_TRUE;
    }
#else // Original Code
    NvU8 data = 0;
    
    if(!Max8907I2cRead8(hDevice, MAX8907_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907_CHG_STAT_MBDET_SHIFT) & MAX8907_CHG_STAT_MBDET_MASK;
    *status = (data == 0 ? NV_TRUE : NV_FALSE );    // MBDET low (0) = present
#endif // CONFIG_MACH_STAR
    //20100529, jh.ahn@lge.com, Battery detection by battery Temp( -40`C ~ 90`C) [END]
    return NV_TRUE;
}

NvBool
Max8907BatteryChargerOK(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;

    if(!Max8907I2cRead8(hDevice, MAX8907_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907_CHG_STAT_VCHG_OK_SHIFT) & MAX8907_CHG_STAT_VCHG_OK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

NvBool
Max8907BatteryChargerEnabled(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
//20100529, jh.ahn@lge.com, Use charger status : not PMU but dedicated Charger IC [START]
#if defined (CONFIG_MACH_STAR) && defined (CONFIG_STAR_BATTERY_CHARGER)
    switch (get_charging_ic_status())
    	{
       case CHG_IC_DEFAULT_MODE:
	case CHG_IC_TA_MODE:
	case CHG_IC_USB_LO_MODE:
	case CHG_IC_FACTORY_MODE:
		*status = NV_TRUE;
		break;

	case CHG_IC_DEACTIVE_MODE:
		*status = NV_FALSE;
		break;

	default:
		return NV_FALSE;
    	}
#else // Original Code
    NvU8 data = 0;

    if(!Max8907I2cRead8(hDevice, MAX8907_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907_CHG_STAT_CHG_EN_STAT_SHIFT) & MAX8907_CHG_STAT_CHG_EN_STAT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
#endif // CONFIG_STAR_BATTERY_CHARGER
//20100529, jh.ahn@lge.com, Use charger status : not PMU but dedicated Charger IC [END]
    return NV_TRUE;
}

NvBool
Max8907BatteryChargerMainBattFull(
    NvOdmPmuDeviceHandle hDevice,
    NvBool *status)
{
    NvU8 data = 0;

    if(!Max8907I2cRead8(hDevice, MAX8907_CHG_STAT, &data))
        return NV_FALSE;

    data = (data >> MAX8907_CHG_STAT_MBATTLOW_SHIFT) & MAX8907_CHG_STAT_MBATTLOW_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

