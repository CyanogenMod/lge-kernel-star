/*
 * arch/arm/mach-tegra/odm_kit/adaptations/tmon/tmon_hal.c
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

#include "tmon_hal.h"
#include "adt7461/nvodm_tmon_adt7461.h"

/*
 * TMON adaptation is a singleton linked directly with NVRM only.
 * Thread safety for TMON APIs is provided by NVRM as well.  
 */


// Temperature Monitors suported under hal
#define TMON_ADT7461_ID (NV_ODM_GUID('a','d','t','7','4','6','1',' '))

#define TMON_ZONE_PSEUDOHANDLE(h, z) \
    ( (NvOdmTmonDeviceHandle)((((NvU32)(h)) << 16) | (z)) )
#define TMON_PSEUDOHANDLE_ZONE(h) ( ((NvU32)(h)) & 0xFFFF )

/*****************************************************************************/

static NvOdmTmonDevice*
TmonGetInstance(NvOdmTmonZoneID ZoneId)
{
    static NvOdmTmonDevice s_TmonArray[NvOdmTmonZoneID_Num];
    static NvOdmTmonDevice* s_TmonMap[NvOdmTmonZoneID_Num];
    static NvBool s_Initialized = NV_FALSE;

    NvU32 i, j;
    NvOdmTmonDevice* pTmon = NULL;
    const NvOdmPeripheralConnectivity* pConn = NULL;

    // Check for invalid zone
    if (ZoneId == 0)
        return NULL;

    if (!s_Initialized)
    {
        NvOdmOsMemset(s_TmonArray, 0, sizeof(s_TmonArray));
        NvOdmOsMemset(s_TmonMap, 0, sizeof(s_TmonMap));
        s_Initialized = NV_TRUE;
        i = 0;  // allocation index

        pConn = NvOdmPeripheralGetGuid(TMON_ADT7461_ID);
        if (pConn)
        {
            pTmon = &s_TmonArray[i++];
            pTmon->pfnInit              = Adt7461Init;
            pTmon->pfnDeinit            = Adt7461Deinit;
            pTmon->pfnTemperatureGet    = Adt7461TemperatureGet;
            pTmon->pfnCapabilitiesGet   = Adt7461CapabilitiesGet;
            pTmon->pfnParameterCapsGet  = Adt7461ParameterCapsGet;
            pTmon->pfnParameterConfig   = Adt7461ParameterConfig;
            pTmon->pfnRun               = Adt7461Run;
            pTmon->pfnStop              = Adt7461Stop;         
            pTmon->pfnIntrRegister      = Adt7461IntrRegister;
            pTmon->pfnIntrUnregister    = Adt7461IntrUnregister;
            pTmon->pConn = pConn;
            pTmon->RefCount = 0;
            pTmon->pPrivate = NULL;

            // Fill in Zones => TMON devices map
            NV_ASSERT(pConn->AddressList);
            for (j = 0; j < pConn->NumAddress; j++)
            {
                if (pConn->AddressList[j].Interface == NvOdmIoModule_Tsense)
                    s_TmonMap[pConn->AddressList[j].Instance] = pTmon;
            }
        }
    }
    // Find TMON instance for the given zone
    if(ZoneId < NvOdmTmonZoneID_Num)
    {
        pTmon = s_TmonMap[ZoneId];
        if (pTmon && pTmon->pConn)
            return pTmon;
    }
    return NULL;
}

/*****************************************************************************/

NvOdmTmonDeviceHandle
NvOdmTmonDeviceOpen(NvOdmTmonZoneID ZoneId)
{
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon)
    {
        NV_ASSERT(pTmon->pfnInit && pTmon->pfnRun);
        // Init TMON device on the 1st open
        if (pTmon->RefCount == 0)
        {
            if (!pTmon->pfnInit(pTmon))
                return NULL;
        }
        // Make sure targeted zone is monitored
        if (pTmon->pfnRun(pTmon, ZoneId))
        {
            pTmon->RefCount++;
            return TMON_ZONE_PSEUDOHANDLE(pTmon, ZoneId);
        }
    }
    return NULL;
}

void NvOdmTmonDeviceClose(NvOdmTmonDeviceHandle hTmon)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon)
    {
        NV_ASSERT(pTmon->pfnDeinit && pTmon->pfnStop);
        (void)pTmon->pfnStop(pTmon, ZoneId);
        if (pTmon->RefCount == 1)
            pTmon->pfnDeinit(pTmon);

        if (pTmon->RefCount)
        {
            pTmon->RefCount--;
            return;
        }
        NV_ASSERT(!"RefCount balance failed");
    }
}

NvBool NvOdmTmonSuspend(NvOdmTmonDeviceHandle hTmon)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnStop);
        if (pTmon->pfnStop(pTmon, ZoneId))
            return NV_TRUE;
    }
    return NV_FALSE;
}

NvBool NvOdmTmonResume(NvOdmTmonDeviceHandle hTmon)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnRun);
        if (pTmon->pfnRun(pTmon, ZoneId))
            return NV_TRUE;
    }
    return NV_FALSE;
}

/*****************************************************************************/

NvBool
NvOdmTmonTemperatureGet(
    NvOdmTmonDeviceHandle hTmon,
    NvS32* pDegreesC)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnTemperatureGet);
        if (pTmon->pfnTemperatureGet(pTmon, ZoneId, pDegreesC))
            return NV_TRUE;
    }
    return NV_FALSE;
}

/*****************************************************************************/

void
NvOdmTmonCapabilitiesGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonCapabilities* pCaps)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnCapabilitiesGet);
        pTmon->pfnCapabilitiesGet(pTmon, ZoneId, pCaps);
    }
    else if (pCaps)
    {
        NvOdmOsMemset(pCaps, 0, sizeof(NvOdmTmonCapabilities));
        pCaps->Tmax = ODM_TMON_PARAMETER_UNSPECIFIED;
        pCaps->Tmin = ODM_TMON_PARAMETER_UNSPECIFIED;
    }
}

void
NvOdmTmonParameterCapsGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonConfigParam ParamId,
    NvOdmTmonParameterCaps* pCaps)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnParameterCapsGet);
        pTmon->pfnParameterCapsGet(pTmon, ZoneId, ParamId, pCaps);
    }
    else if (pCaps)
    {
        NvOdmOsMemset(pCaps, 0, sizeof(NvOdmTmonParameterCaps));
        pCaps->MaxValue = ODM_TMON_PARAMETER_UNSPECIFIED;
        pCaps->MinValue = ODM_TMON_PARAMETER_UNSPECIFIED;
        pCaps->OdmProtected = NV_TRUE;
    }
}

NvBool
NvOdmTmonParameterConfig(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonConfigParam ParamId,
    NvS32* pSetting)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnParameterConfig);
        if (pTmon->pfnParameterConfig(pTmon, ZoneId, ParamId, pSetting))
            return NV_TRUE;
    }
    return NV_FALSE;
}

/*****************************************************************************/

NvOdmTmonIntrHandle
NvOdmTmonIntrRegister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmInterruptHandler Callback,
    void* CallbackArg)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    NvOdmTmonIntrHandle hIntr = NULL;
    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnIntrRegister);
        hIntr = pTmon->pfnIntrRegister(
            pTmon, ZoneId, Callback, CallbackArg);
    }
    return hIntr;
}

void
NvOdmTmonIntrUnregister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonIntrHandle hIntr)
{
    NvOdmTmonZoneID ZoneId = TMON_PSEUDOHANDLE_ZONE(hTmon);
    NvOdmTmonDevice* pTmon = TmonGetInstance(ZoneId);

    if (pTmon && pTmon->RefCount)
    {
        NV_ASSERT(pTmon->pfnIntrUnregister);
        pTmon->pfnIntrUnregister(pTmon, ZoneId, hIntr);
    }
}

/*****************************************************************************/
