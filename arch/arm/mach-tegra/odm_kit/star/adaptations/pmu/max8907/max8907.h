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

#ifndef INCLUDED_PMU_MAX8907_H
#define INCLUDED_PMU_MAX8907_H

#include "nvodm_pmu.h"
#include "pmu_hal.h"

#if defined(CONFIG_MACH_STAR)
#define FEATURE_MAX8907C_MAX8952_COMBINATION
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

#define NV_DEBUG 0

#if (NV_DEBUG)
#define NVODMPMU_PRINTF(x)   NvOdmOsPrintf x
#else
#define NVODMPMU_PRINTF(x)
#endif

typedef struct Max8907StatusRec
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* Main charger Present */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

} Max8907Status;

typedef struct
{
    /* The handle to the I2C */
    NvOdmServicesI2cHandle hOdmI2C;

    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;
    
    /* the PMU I2C device Address */
    NvU32 DeviceAddr;

    /* the PMU status */
    Max8907Status pmuStatus;

    /* battery presence */
    NvBool battPresence;

    /* PMU interrupt support enabled */
    NvBool pmuInterruptSupported;

    /* The ref cnt table of the power supplies */
    NvU32 *supplyRefCntTable;

    /* The pointer to supply voltages shadow */
    NvU32 *pVoltages;

} Max8907PrivData;

NvBool 
Max8907Setup(NvOdmPmuDeviceHandle hDevice);

void 
Max8907Release(NvOdmPmuDeviceHandle hDevice);

NvBool
Max8907GetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts);

NvBool
Max8907SetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds);

#if defined(CONFIG_MACH_STAR) 
//20100704 bergkamp.cho@lge.com jongik's headset porting [LGE]
NvU32
Max8907GetHookAdc(
    NvOdmPmuDeviceHandle hDevice);

//20101121 cs77.ha@lge.com, HW power off in thermal limit [START]
NvU32
Max8907SetHwPowerOffConfig(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable);
//20101121 cs77.ha@lge.com, HW power off in thermal limit [END]
#endif

void
Max8907GetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities);

NvBool
Max8907GetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus);

NvBool
Max8907GetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus);

NvBool
Max8907GetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData);

//20100924, jh.ahn@lge.com, For updating battery information totally [START]
#if defined(CONFIG_MACH_STAR)
NvBool
Max8907UpdateBatteryInfo(
	NvOdmPmuDeviceHandle hDevice,
	NvOdmPmuAcLineStatus *pAcStatus,
	NvU8 *pBatStatus,
	NvOdmPmuBatteryData *pBatData);
#endif // CONFIG_MACH_STAR
//20100924, jh.ahn@lge.com, For updating battery information totally [END]

void
Max8907GetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime);

void
Max8907GetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry);

NvBool
Max8907SetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType);

NvBool
Max8907bRtcSuspend(
    NvOdmPmuDeviceHandle hDevice);

NvBool
Max8907bRtcResume(
    NvOdmPmuDeviceHandle hDevice);

void Max8907InterruptHandler( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PMU_MAX8907_H
