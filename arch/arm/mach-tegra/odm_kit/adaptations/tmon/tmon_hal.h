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
 *         Abstraction layer stub for Temperature Monitor adaptations</b>
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
