/*
 * arch/arm/mach-tegra/odm_kit/adaptations/tmon/tmon_hal.h
 *
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

#ifndef INCLUDED_NVODM_TMON_ADAPTATION_HAL_H
#define INCLUDED_NVODM_TMON_ADAPTATION_HAL_H

#include "nvcommon.h"
#include "nvodm_tmon.h"
#include "nvodm_query_discovery.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef NvBool (*pfnTmonInit)(NvOdmTmonDeviceHandle);
typedef void   (*pfnTmonDeinit)(NvOdmTmonDeviceHandle);
typedef NvBool (*pfnTmonTemperatureGet)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvS32*);
typedef void   (*pfnTmonCapabilitiesGet)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvOdmTmonCapabilities*);
typedef void   (*pfnTmonParameterCapsGet)
               (NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvOdmTmonConfigParam, NvOdmTmonParameterCaps*);
typedef NvBool (*pfnTmonParameterConfig)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvOdmTmonConfigParam, NvS32*);
typedef NvBool (*pfnTmonRun)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID);
typedef NvBool (*pfnTmonStop)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID);
typedef NvOdmTmonIntrHandle
               (*pfnTmonIntrRegister)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvOdmInterruptHandler, void*);
typedef void   (*pfnTmonIntrUnregister)(NvOdmTmonDeviceHandle, NvOdmTmonZoneID, NvOdmTmonIntrHandle);

typedef struct NvOdmTmonDeviceRec
{ 
    pfnTmonInit                 pfnInit;
    pfnTmonDeinit               pfnDeinit;
    pfnTmonTemperatureGet       pfnTemperatureGet;
    pfnTmonCapabilitiesGet      pfnCapabilitiesGet;
    pfnTmonParameterCapsGet     pfnParameterCapsGet;
    pfnTmonParameterConfig      pfnParameterConfig;
    pfnTmonRun                  pfnRun;
    pfnTmonStop                 pfnStop;
    pfnTmonIntrRegister         pfnIntrRegister;
    pfnTmonIntrUnregister       pfnIntrUnregister;

    const NvOdmPeripheralConnectivity*  pConn;
    NvU32                       RefCount;
    void                        *pPrivate;
} NvOdmTmonDevice;

#ifdef __cplusplus
}
#endif

#endif  //INCLUDED_NVODM_TMON_ADAPTATION_HAL_H
