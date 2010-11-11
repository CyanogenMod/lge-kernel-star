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

#ifndef INCLUDED_PMU_PCF50626_H
#define INCLUDED_PMU_PCF50626_H

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


typedef struct Pcf50626StatusRec
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* charger switch from CC mode to CV mode */
    NvBool chgCcToCv;

    /* Main charger Presents */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

} Pcf50626Status;



typedef struct
{
    /* The handle to the I2C */
    NvOdmServicesI2cHandle hOdmI2C;

    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;
    
    /* the PMU I2C device Address */
    NvU32 DeviceAddr;

    /* the PMU status */
    Pcf50626Status pmuStatus;

    /* battery presence */
    NvBool battPresence;

    /* PMU interrupt support enabled */
    NvBool pmuInterruptSupported;

    
    /* The ref cnt table of the power supplies */
    NvU32 *supplyRefCntTable;

}Pcf50626PrivData;


NvBool 
Pcf50626Setup(NvOdmPmuDeviceHandle hDevice);

void 
Pcf50626Release(NvOdmPmuDeviceHandle hDevice);

NvBool
Pcf50626GetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts);

NvBool
Pcf50626SetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds);

void
Pcf50626GetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities);


NvBool 
Pcf50626GetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuAcLineStatus *pStatus);


NvBool 
Pcf50626GetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus);

NvBool
Pcf50626GetBatteryData(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData);

void
Pcf50626GetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime);

void
Pcf50626GetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry);

NvBool 
Pcf50626SetChargingCurrent( 
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuChargingPath chargingPath, 
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType); 

void Pcf50626InterruptHandler( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_PMU_PCF50626_H
