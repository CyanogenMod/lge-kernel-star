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

void Max8907bInterruptHandler( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PMU_MAX8907B_H
