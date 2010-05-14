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
