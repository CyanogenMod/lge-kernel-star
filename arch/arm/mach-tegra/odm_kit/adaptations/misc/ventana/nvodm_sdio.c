/*
 * Copyright (c) 2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvodm_sdio.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvodm_pmu.h"
#include "nvos.h"

typedef struct NvOdmSdioRec
{
    // NvODM PMU device handle
    NvOdmServicesPmuHandle hPmu;
    // Gpio Handle
    NvOdmServicesGpioHandle hGpio;
    // Pin handle to Wlan Reset Gpio pin
    NvOdmGpioPinHandle hResetPin;
    // Pin handle to Wlan PWR GPIO Pin
    NvOdmGpioPinHandle hPwrPin;
    const NvOdmPeripheralConnectivity *pConnectivity;
    // Power state
    NvBool PoweredOn;
    // Instance
    NvU32 Instance;
} NvOdmSdio;

static void NvOdmSetPowerOnSdio(NvOdmSdioHandle pDevice, NvBool enable)
{
    const NvOdmPeripheralConnectivity *pConn;
    NvU32 i;

    pConn = pDevice->pConnectivity;

    for (i=0; i<pConn->NumAddress; i++)
    {
        const NvOdmIoAddress *addr = &pConn->AddressList[i];
        NvU32 settle;
        NvU32 voltage;

        if (addr->Interface != NvOdmIoModule_Vdd)
            continue;

        if (enable)
        {
            NvOdmServicesPmuVddRailCapabilities caps;
            NvOdmServicesPmuGetCapabilities(pDevice->hPmu, addr->Address, &caps);
            voltage = caps.requestMilliVolts;
        }
        else
        {
            voltage = ODM_VOLTAGE_OFF;
        }

        NvOdmServicesPmuSetVoltage(pDevice->hPmu, addr->Address,
                                   voltage, &settle);

        if (settle)
            NvOdmOsWaitUS(settle);
    }
}

NvOdmSdioHandle NvOdmSdioOpen(NvU32 Instance)
{
    static NvOdmSdio *pDevice = NULL;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvU32 NumOfGuids = 1;
    NvU64 guid;
    NvU32 searchVals[2];
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;
    const NvOdmPeripheralSearch searchAttrs[] =
    {
        NvOdmPeripheralSearch_IoModule,
        NvOdmPeripheralSearch_Instance,
    };

    searchVals[0] =  NvOdmIoModule_Sdio;
    searchVals[1] =  Instance;

    NvOdmQueryPinMux(NvOdmIoModule_Sdio, &pOdmConfigs, &NumOdmConfigs);
    if (Instance >= NumOdmConfigs )
        return NULL;
    if( pOdmConfigs[Instance] == 0 )
        return NULL;

    NumOfGuids = NvOdmPeripheralEnumerate(searchAttrs, searchVals,
                                          2, &guid, NumOfGuids);


    // Get the peripheral connectivity information
    pConnectivity = NvOdmPeripheralGetGuid(guid);
    if (pConnectivity == NULL)
        return NULL;

    pDevice = NvOdmOsAlloc(sizeof(NvOdmSdio));
    pDevice->hPmu = NULL;
    if(pDevice == NULL)
        return (pDevice);

    if (pDevice->hPmu == NULL)
    {
        pDevice->hPmu = NvOdmServicesPmuOpen();
        if(pDevice->hPmu == NULL)
        {
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (NULL);
        }
    }

    pDevice->pConnectivity = pConnectivity;
    NvOdmSetPowerOnSdio(pDevice, NV_TRUE);

    pDevice->PoweredOn = NV_TRUE;
    pDevice->Instance = Instance;
    return pDevice;
}

void NvOdmSdioClose(NvOdmSdioHandle hOdmSdio)
{
    NvOdmSetPowerOnSdio(hOdmSdio, NV_FALSE);
    if (hOdmSdio->hPmu != NULL)
    {
         NvOdmServicesPmuClose(hOdmSdio->hPmu);
    }
    NvOdmOsFree(hOdmSdio);
    hOdmSdio = NULL;
}


NvBool NvOdmSdioSuspend(NvOdmSdioHandle hOdmSdio)
{
    NvBool Status = NV_TRUE;

    if (!hOdmSdio->PoweredOn)
        return NV_TRUE;

    NvOdmSetPowerOnSdio(hOdmSdio, NV_FALSE);

    hOdmSdio->PoweredOn = NV_FALSE;
    return Status;

}

NvBool NvOdmSdioResume(NvOdmSdioHandle hOdmSdio)
{
    NvBool Status = NV_TRUE;

    if (hOdmSdio->PoweredOn)
        return NV_TRUE;

    NvOdmSetPowerOnSdio(hOdmSdio, NV_TRUE);

    hOdmSdio->PoweredOn = NV_TRUE;
    return Status;
}
