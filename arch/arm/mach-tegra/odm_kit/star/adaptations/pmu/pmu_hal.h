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

#ifndef INCLUDED_NVODM_PMU_ADAPTATION_HAL_H
#define INCLUDED_NVODM_PMU_ADAPTATION_HAL_H

#include "nvcommon.h"
#include "nvodm_pmu.h"
#include "nvodm_services.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef NvBool (*pfnPmuSetup)(NvOdmPmuDeviceHandle);
typedef void   (*pfnPmuRelease)(NvOdmPmuDeviceHandle);
typedef void   (*pfnPmuGetCaps)(NvU32, NvOdmPmuVddRailCapabilities*);
typedef NvBool (*pfnPmuGetVoltage)(NvOdmPmuDeviceHandle, NvU32, NvU32*);
typedef NvBool (*pfnPmuSetVoltage)(NvOdmPmuDeviceHandle, NvU32, NvU32, NvU32*);
typedef NvBool (*pfnPmuGetAcLineStatus)(NvOdmPmuDeviceHandle, NvOdmPmuAcLineStatus*);
typedef NvBool (*pfnPmuGetBatteryStatus)(NvOdmPmuDeviceHandle, NvOdmPmuBatteryInstance, NvU8*);
typedef NvBool (*pfnPmuGetBatteryData)(NvOdmPmuDeviceHandle, NvOdmPmuBatteryInstance, NvOdmPmuBatteryData*);
typedef void   (*pfnPmuGetBatteryFullLifeTime)(NvOdmPmuDeviceHandle, NvOdmPmuBatteryInstance, NvU32*);
typedef void   (*pfnPmuGetBatteryChemistry)(NvOdmPmuDeviceHandle, NvOdmPmuBatteryInstance, NvOdmPmuBatteryChemistry*);
typedef NvBool (*pfnPmuSetChargingCurrent)(NvOdmPmuDeviceHandle, NvOdmPmuChargingPath, NvU32, NvOdmUsbChargerType);
typedef void   (*pfnPmuInterruptHandler)(NvOdmPmuDeviceHandle);
typedef NvBool (*pfnPmuReadRtc)(NvOdmPmuDeviceHandle, NvU32*);
typedef NvBool (*pfnPmuWriteRtc)(NvOdmPmuDeviceHandle, NvU32);
typedef NvBool (*pfnPmuIsRtcInitialized)(NvOdmPmuDeviceHandle);
typedef NvBool (*pfnPmuReadAlarm)(NvOdmPmuDeviceHandle, NvU32*);
typedef NvBool (*pfnPmuWriteAlarm)(NvOdmPmuDeviceHandle, NvU32);
typedef NvU32  (*pfnPmuGetHookAdc)(NvOdmPmuDeviceHandle);	//20100704 bergkamp.cho@lge.com jongik's headset porting [LGE]
typedef NvU32  (*pfnPmuSetHwPowerOffConfig)(NvOdmPmuDeviceHandle, NvBool);  //20101121 cs77.ha@lge.com, HW power off in thermal limit
typedef NvBool (*pfnPmuUpdateBatteryInfo)(NvOdmPmuDeviceHandle, NvOdmPmuAcLineStatus*, NvU8*, NvOdmPmuBatteryData*); // 20100924 jh.ahn@lge.com, For updating battery information totally
typedef NvBool (*pfnPmuAlarmInterrupt)(NvOdmPmuDeviceHandle);
typedef NvBool (*pfnPmuEnableRtcInt)(NvOdmPmuDeviceHandle, NvBool);
typedef NvBool (*pfnPmuSuspendRtc)(NvOdmPmuDeviceHandle);
typedef NvBool (*pfnPmuResumeRtc)(NvOdmPmuDeviceHandle);

typedef struct NvOdmPmuDeviceRec
{
    pfnPmuSetup                  pfnSetup;
    pfnPmuRelease                pfnRelease;
    pfnPmuGetCaps                pfnGetCaps;
    pfnPmuGetVoltage             pfnGetVoltage;
    pfnPmuSetVoltage             pfnSetVoltage;
    pfnPmuGetAcLineStatus        pfnGetAcLineStatus;
    pfnPmuGetBatteryStatus       pfnGetBatteryStatus;
    pfnPmuGetBatteryData         pfnGetBatteryData;
    pfnPmuGetBatteryFullLifeTime pfnGetBatteryFullLifeTime;
    pfnPmuGetBatteryChemistry    pfnGetBatteryChemistry;
    pfnPmuSetChargingCurrent     pfnSetChargingCurrent;
    pfnPmuInterruptHandler       pfnInterruptHandler;
    pfnPmuReadRtc                pfnReadRtc;
    pfnPmuWriteRtc               pfnWriteRtc;
    pfnPmuReadAlarm              pfnReadAlarm;
    pfnPmuWriteAlarm             pfnWriteAlarm;
    pfnPmuAlarmInterrupt         pfnAlarmInterrupt;
    pfnPmuEnableRtcInt           pfnEnableRtcInt;
    pfnPmuSuspendRtc             pfnSuspendRtc;
    pfnPmuResumeRtc              pfnResumeRtc;
    pfnPmuIsRtcInitialized       pfnIsRtcInitialized;
    pfnPmuGetHookAdc             pfnGetHookAdc;		//20100704 bergkamp.cho@lge.com jongik's headset porting [LGE]
    pfnPmuSetHwPowerOffConfig    pfnSetHwPowerOffConfig;    //20101121 cs77.ha@lge.com, HW power off in thermal limit
    pfnPmuUpdateBatteryInfo		 pfnUpdateBatteryInfo;	// 20100924 jh.ahn@lge.com, For updating battery information totally
    void                        *pPrivate;
    NvBool                       Hal;
    NvBool                       Init;
} NvOdmPmuDevice;

#ifdef __cplusplus
}
#endif

#endif
