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

/** 
 * @file
 * <b>NVIDIA Tegra ODM Kit:
 *         Abstraction layer stub for audio codec adaptations</b>
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
    pfnPmuIsRtcInitialized       pfnIsRtcInitialized;
    void                        *pPrivate;
    NvBool                       Hal;
    NvBool                       Init;
} NvOdmPmuDevice;

#ifdef __cplusplus
}
#endif

#endif
