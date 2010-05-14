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

#ifndef NVODM_PMU_TPS6586X_H_HH
#define NVODM_PMU_TPS6586X_H_HH

#include "nvodm_pmu.h"
#include "nvodm_services.h"
#include "nvodm_pmu_tps6586x_supply_info_table.h"
#include "tps6586x_reg.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#if (NV_DEBUG)
#define NVODMPMU_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMPMU_PRINTF(x)
#endif

typedef struct NvOdmPmuDeviceTPSRec
{
    /* The handle to the I2C */
    NvOdmServicesI2cHandle hOdmI2C;

    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;
    /* the PMU I2C device Address */
    NvU32 DeviceAddr;

    /* Device's private data */
    void *priv;
    
#if defined(CONFIG_TEGRA_ODM_HARMONY)
     /* Gpio Handles (for external supplies) */
     NvOdmServicesGpioHandle hGpio;
     NvOdmGpioPinHandle hPin[TPS6586x_EXTERNAL_SUPPLY_AP_GPIO_NUM];
#else
    /* The current voltage */
    NvU32 curVoltageTable[VRAILCOUNT];
#endif

    /* The ref cnt table of the power supplies */
#if defined(CONFIG_TEGRA_ODM_HARMONY)
    NvU32 supplyRefCntTable[TPS6586xPmuSupply_Num];
#else
    NvU32 supplyRefCntTable[VRAILCOUNT];
#endif

} NvOdmPmuDeviceTPS;

void Tps6586xGetCapabilities( NvU32 vddRail, NvOdmPmuVddRailCapabilities* pCapabilities);
NvBool Tps6586xGetVoltage( NvOdmPmuDeviceHandle hDevice, NvU32 vddRail, NvU32* pMilliVolts);
NvBool Tps6586xSetVoltage( NvOdmPmuDeviceHandle hDevice, NvU32 vddRail, NvU32 MilliVolts, NvU32* pSettleMicroSeconds);
NvBool Tps6586xSetup(NvOdmPmuDeviceHandle hDevice);
void Tps6586xRelease(NvOdmPmuDeviceHandle hDevice);
NvBool Tps6586xGetAcLineStatus( NvOdmPmuDeviceHandle hDevice, NvOdmPmuAcLineStatus *pStatus);
NvBool Tps6586xGetBatteryStatus( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvU8 *pStatus);
NvBool Tps6586xGetBatteryData( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvOdmPmuBatteryData *pData); 
void Tps6586xGetBatteryFullLifeTime( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvU32 *pLifeTime);
void Tps6586xGetBatteryChemistry( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvOdmPmuBatteryChemistry *pChemistry);
NvBool Tps6586xSetChargingCurrent( NvOdmPmuDeviceHandle hDevice, NvOdmPmuChargingPath chargingPath, NvU32 chargingCurrentLimitMa, NvOdmUsbChargerType ChargerType); 
void Tps6586xInterruptHandler( NvOdmPmuDeviceHandle  hDevice);
NvBool Tps6586xReadRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 *Count);
NvBool Tps6586xWriteRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 Count);
NvBool Tps6586xIsRtcInitialized( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif

#endif /* NVODM_PMU_TPS6586X_H_HH */
