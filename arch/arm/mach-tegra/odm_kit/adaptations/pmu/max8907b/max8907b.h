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

#ifndef INCLUDED_PMU_MAX8907B_H
#define INCLUDED_PMU_MAX8907B_H

#include "nvodm_pmu.h"
#include"pmu_hal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#if (NV_DEBUG)
#define NVODMPMU_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMPMU_PRINTF(x)
#endif

typedef struct Max8907bStatusRec
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* Main charger Present */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

} Max8907bStatus;

typedef struct
{
    /* The handle to the I2C */
    NvOdmServicesI2cHandle hOdmI2C;

    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;
    
    /* the PMU I2C device Address */
    NvU32 DeviceAddr;

    /* the PMU status */
    Max8907bStatus pmuStatus;

    /* battery presence */
    NvBool battPresence;

    /* PMU interrupt support enabled */
    NvBool pmuInterruptSupported;

    /* The ref cnt table of the power supplies */
    NvU32 *supplyRefCntTable;

    /* The pointer to supply voltages shadow */
    NvU32 *pVoltages;

} Max8907bPrivData;

NvBool 
Max8907bSetup(NvOdmPmuDeviceHandle hDevice);

void 
Max8907bRelease(NvOdmPmuDeviceHandle hDevice);

NvBool
Max8907bGetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts);

NvBool
Max8907bSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds);

void
Max8907bGetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities);

NvBool
Max8907bGetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus);

NvBool
Max8907bGetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus);

NvBool
Max8907bGetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData);

void
Max8907bGetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime);

void
Max8907bGetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry);

NvBool
Max8907bSetChargingCurrent(
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

void Max8907bInterruptHandler( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PMU_MAX8907B_H
