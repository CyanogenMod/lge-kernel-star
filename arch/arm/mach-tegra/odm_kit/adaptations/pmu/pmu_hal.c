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

#include "nvcommon.h"
#include "nvodm_pmu.h"
#include "nvodm_query_discovery.h"
#include "pmu_hal.h"
#include "nvodm_services.h"
#include "tps6586x/nvodm_pmu_tps6586x.h"
#include "max8907b/max8907b.h"
#include "max8907b/max8907b_rtc.h"
#include "pcf50626/pcf50626.h"
#include "pcf50626/pcf50626_rtc.h"

static NvOdmPmuDevice*
GetPmuInstance(NvOdmPmuDeviceHandle hDevice)
{
    static NvOdmPmuDevice Pmu;
    static NvBool         first = NV_TRUE;

    if (first)
    {
        NvOdmOsMemset(&Pmu, 0, sizeof(Pmu));
        first = NV_FALSE;

        if (NvOdmPeripheralGetGuid(NV_ODM_GUID('t','p','s','6','5','8','6','x')))
        {
            //  fill in HAL functions here.
            Pmu.Hal = NV_TRUE;
            Pmu.pfnSetup = Tps6586xSetup;
            Pmu.pfnRelease = Tps6586xRelease;
            Pmu.pfnGetCaps = Tps6586xGetCapabilities;
            Pmu.pfnGetVoltage = Tps6586xGetVoltage;
            Pmu.pfnSetVoltage = Tps6586xSetVoltage;
            Pmu.pfnGetAcLineStatus = Tps6586xGetAcLineStatus;
            Pmu.pfnGetBatteryStatus = Tps6586xGetBatteryStatus;
            Pmu.pfnGetBatteryData = Tps6586xGetBatteryData;
            Pmu.pfnGetBatteryFullLifeTime = Tps6586xGetBatteryFullLifeTime;
            Pmu.pfnGetBatteryChemistry = Tps6586xGetBatteryChemistry;
            Pmu.pfnSetChargingCurrent = Tps6586xSetChargingCurrent;
            Pmu.pfnInterruptHandler = Tps6586xInterruptHandler;
            Pmu.pfnReadRtc = Tps6586xReadRtc;
            Pmu.pfnWriteRtc = Tps6586xWriteRtc;
            Pmu.pfnReadAlarm = Tps6586xReadAlarm;
            Pmu.pfnWriteAlarm = Tps6586xWriteAlarm;
            Pmu.pfnAlarmInterrupt = NULL;
            Pmu.pfnEnableRtcInt = NULL;
            Pmu.pfnSuspendRtc = NULL;
            Pmu.pfnResumeRtc = NULL;
            Pmu.pfnIsRtcInitialized = Tps6586xIsRtcInitialized;
        }
        else if (NvOdmPeripheralGetGuid(NV_ODM_GUID('p','c','f','_','p','m','u','0')))
        {

            Pmu.pfnSetup                  = Pcf50626Setup;
            Pmu.pfnRelease                = Pcf50626Release;
            Pmu.pfnGetCaps                = Pcf50626GetCapabilities;
            Pmu.pfnGetVoltage             = Pcf50626GetVoltage;
            Pmu.pfnSetVoltage             = Pcf50626SetVoltage;
            Pmu.pfnGetAcLineStatus        = Pcf50626GetAcLineStatus;
            Pmu.pfnGetBatteryStatus       = Pcf50626GetBatteryStatus;
            Pmu.pfnGetBatteryData         = Pcf50626GetBatteryData;
            Pmu.pfnGetBatteryFullLifeTime = Pcf50626GetBatteryFullLifeTime;
            Pmu.pfnGetBatteryChemistry    = Pcf50626GetBatteryChemistry;
            Pmu.pfnSetChargingCurrent     = Pcf50626SetChargingCurrent;
            Pmu.pfnInterruptHandler       = Pcf50626InterruptHandler;
            Pmu.pfnReadRtc                = Pcf50626RtcCountRead;
            Pmu.pfnWriteRtc               = Pcf50626RtcCountWrite;
            Pmu.pfnReadAlarm              = NULL;
            Pmu.pfnWriteAlarm             = NULL;
            Pmu.pfnAlarmInterrupt         = NULL;
            Pmu.pfnEnableRtcInt           = NULL;
            Pmu.pfnSuspendRtc             = NULL;
            Pmu.pfnResumeRtc              = NULL;
            Pmu.pfnIsRtcInitialized       = Pcf50626IsRtcInitialized;
            Pmu.pPrivate                  = NULL;
            Pmu.Hal                       = NV_TRUE;
            Pmu.Init                      = NV_FALSE;
        }
        else if (NvOdmPeripheralGetGuid(NV_ODM_GUID('m','a','x','8','9','0','7','b')))
        {

            Pmu.pfnSetup                  = Max8907bSetup;
            Pmu.pfnRelease                = Max8907bRelease;
            Pmu.pfnGetCaps                = Max8907bGetCapabilities;
            Pmu.pfnGetVoltage             = Max8907bGetVoltage;
            Pmu.pfnSetVoltage             = Max8907bSetVoltage;
            Pmu.pfnGetAcLineStatus        = Max8907bGetAcLineStatus;
            Pmu.pfnGetBatteryStatus       = Max8907bGetBatteryStatus;
            Pmu.pfnGetBatteryData         = Max8907bGetBatteryData;
            Pmu.pfnGetBatteryFullLifeTime = Max8907bGetBatteryFullLifeTime;
            Pmu.pfnGetBatteryChemistry    = Max8907bGetBatteryChemistry;
            Pmu.pfnSetChargingCurrent     = Max8907bSetChargingCurrent;
            Pmu.pfnInterruptHandler       = Max8907bInterruptHandler;
            Pmu.pfnReadRtc                = Max8907bRtcCountRead;
            Pmu.pfnWriteRtc               = Max8907bRtcCountWrite;
            Pmu.pfnReadAlarm              = Max8907bRtcAlarmCountRead;
            Pmu.pfnWriteAlarm             = Max8907bRtcAlarmCountWrite;
            Pmu.pfnAlarmInterrupt         = NULL;
            Pmu.pfnEnableRtcInt           = Max8907bRtcAlarmIntEnable;
            Pmu.pfnSuspendRtc             = Max8907bRtcSuspend;
            Pmu.pfnResumeRtc              = Max8907bRtcResume;
            Pmu.pfnIsRtcInitialized       = Max8907bIsRtcInitialized;
            Pmu.pPrivate                  = NULL;
            Pmu.Hal                       = NV_TRUE;
            Pmu.Init                      = NV_FALSE;
        }
    }

    if (hDevice && Pmu.Hal)
        return &Pmu;

    return NULL;
}

NvBool
NvOdmPmuDeviceOpen(NvOdmPmuDeviceHandle *hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance((NvOdmPmuDeviceHandle)1);

    *hDevice = (NvOdmPmuDeviceHandle)0;

    if (!pmu || !pmu->pfnSetup)
        return NV_FALSE;

    if (pmu->Init)
    {
        *hDevice = (NvOdmPmuDeviceHandle)1;
        return NV_TRUE;
    }

    if (pmu->pfnSetup(pmu))
    {
        *hDevice = (NvOdmPmuDeviceHandle)1;
        pmu->Init = NV_TRUE;
        return NV_TRUE;
    }

    return NV_FALSE;
}

void NvOdmPmuDeviceClose(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);    

    if (!pmu)
        return;

    if (pmu->pfnRelease)
        pmu->pfnRelease(pmu);

    pmu->Init = NV_FALSE;
}

void
NvOdmPmuGetCapabilities(NvU32 vddId,
                        NvOdmPmuVddRailCapabilities* pCapabilities)
{
    //  use a manual handle, since this function doesn't takea  handle
    NvOdmPmuDevice* pmu = GetPmuInstance((NvOdmPmuDeviceHandle)1);

    if (pmu && pmu->pfnGetCaps)
        pmu->pfnGetCaps(vddId, pCapabilities);
    else if (pCapabilities)
    {
        NvOdmOsMemset(pCapabilities, 0, sizeof(NvOdmPmuVddRailCapabilities));
        pCapabilities->OdmProtected = NV_TRUE;
    }
}


NvBool
NvOdmPmuGetVoltage(NvOdmPmuDeviceHandle hDevice,
                   NvU32 vddId,
                   NvU32* pMilliVolts)
{
    NvOdmPmuDevice* pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetVoltage)
        return pmu->pfnGetVoltage(pmu, vddId, pMilliVolts);

    return NV_TRUE;
}

NvBool
NvOdmPmuSetVoltage(NvOdmPmuDeviceHandle hDevice,
                   NvU32 VddId,
                   NvU32 MilliVolts,
                   NvU32* pSettleMicroSeconds)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnSetVoltage)
    {
        return pmu->pfnSetVoltage(pmu, VddId, MilliVolts, pSettleMicroSeconds);
    }

    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = 0;
    return NV_TRUE;
}


NvBool 
NvOdmPmuGetAcLineStatus(NvOdmPmuDeviceHandle hDevice, 
                        NvOdmPmuAcLineStatus *pStatus)
{
    NvOdmPmuDevice* pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetAcLineStatus)
        return pmu->pfnGetAcLineStatus(pmu, pStatus);

    return NV_TRUE;
}


NvBool 
NvOdmPmuGetBatteryStatus(NvOdmPmuDeviceHandle hDevice, 
                         NvOdmPmuBatteryInstance BatteryInst,
                         NvU8 *pStatus)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetBatteryStatus)
        return pmu->pfnGetBatteryStatus(pmu, BatteryInst, pStatus);

    return NV_TRUE;
}

NvBool
NvOdmPmuGetBatteryData(NvOdmPmuDeviceHandle hDevice, 
                       NvOdmPmuBatteryInstance BatteryInst,
                       NvOdmPmuBatteryData *pData)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetBatteryData)
        return pmu->pfnGetBatteryData(pmu, BatteryInst, pData);

    pData->batteryLifePercent = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryVoltage = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryCurrent = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryAverageCurrent = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryMahConsumed = NVODM_BATTERY_DATA_UNKNOWN;
    pData->batteryTemperature = NVODM_BATTERY_DATA_UNKNOWN;

    return NV_TRUE;
}


void
NvOdmPmuGetBatteryFullLifeTime(NvOdmPmuDeviceHandle hDevice, 
                               NvOdmPmuBatteryInstance BatteryInst,
                               NvU32 *pLifeTime)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetBatteryFullLifeTime)
        pmu->pfnGetBatteryFullLifeTime(pmu, BatteryInst, pLifeTime);

    else
    {
        if (pLifeTime)
            *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
    }
}


void
NvOdmPmuGetBatteryChemistry(NvOdmPmuDeviceHandle hDevice, 
                            NvOdmPmuBatteryInstance BatteryInst,
                            NvOdmPmuBatteryChemistry *pChemistry)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnGetBatteryChemistry)
        pmu->pfnGetBatteryChemistry(pmu, BatteryInst, pChemistry);
    else
    {
        if (pChemistry)
            *pChemistry = NVODM_BATTERY_DATA_UNKNOWN;
    }
}

NvBool 
NvOdmPmuSetChargingCurrent(NvOdmPmuDeviceHandle hDevice, 
                           NvOdmPmuChargingPath ChargingPath, 
                           NvU32 ChargingCurrentLimitMa,
                           NvOdmUsbChargerType ChargerType)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnSetChargingCurrent)
        return pmu->pfnSetChargingCurrent(pmu, ChargingPath, ChargingCurrentLimitMa, ChargerType);

    return NV_TRUE;
}


void NvOdmPmuInterruptHandler(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnInterruptHandler)
        pmu->pfnInterruptHandler(pmu);
}

NvBool NvOdmPmuReadRtc(
    NvOdmPmuDeviceHandle  hDevice,
    NvU32 *Count)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnReadRtc)
        return pmu->pfnReadRtc(pmu, Count);
    return NV_FALSE;
}


NvBool NvOdmPmuWriteRtc(
    NvOdmPmuDeviceHandle  hDevice,
    NvU32 Count)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnWriteRtc)
        return pmu->pfnWriteRtc(pmu, Count);
    return NV_FALSE;
}

NvBool NvOdmPmuReadAlarm(
    NvOdmPmuDeviceHandle  hDevice,
    NvU32 *Count)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnReadAlarm)
        return pmu->pfnReadAlarm(pmu, Count);
    return NV_FALSE;
}

NvBool NvOdmPmuWriteAlarm(
    NvOdmPmuDeviceHandle  hDevice,
    NvU32 Count)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnWriteAlarm)
        return pmu->pfnWriteAlarm(pmu, Count);
    return NV_FALSE;
}

NvBool
NvOdmPmuIsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);

    if (pmu && pmu->pfnIsRtcInitialized)
        return pmu->pfnIsRtcInitialized(pmu);

    return NV_FALSE;
}

NvBool
NvOdmPmuAlarmHandlerSet(NvOdmPmuDeviceHandle hDevice, pfnPmuAlarmInterrupt func)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);
    if (!pmu)
	return NV_FALSE;
    pmu->pfnAlarmInterrupt = func;
    return NV_TRUE;
}

NvBool
NvOdmPmuSuspendRtc(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);
    if (pmu && pmu->pfnSuspendRtc)
        return pmu->pfnSuspendRtc(pmu);
    return NV_FALSE;
}

NvBool
NvOdmPmuResumeRtc(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmPmuDevice *pmu = GetPmuInstance(hDevice);
    if (pmu && pmu->pfnResumeRtc)
        return pmu->pfnResumeRtc(pmu);
    return NV_FALSE;
}
