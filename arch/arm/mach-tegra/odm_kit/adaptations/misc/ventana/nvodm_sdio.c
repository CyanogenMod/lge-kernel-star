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

#define WLAN_GUID   NV_ODM_GUID('s','d','i','o','w','l','a','n')

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

static NvBool SdioOdmWlanPower(NvOdmSdioHandle hOdmSdio, NvBool IsEnable)
{
    if (IsEnable)
    {
        // Wlan Power On Reset Sequence
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x0);
        NvOdmOsWaitUS(2000);
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x1);
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x1);
     }
     else
     {
         // Power Off sequence
         NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);
     }

    return NV_TRUE;
}

NvOdmSdioHandle NvOdmSdioOpen(NvU32 Instance)
{
    static NvOdmSdio *pDevice = NULL;
    NvOdmServicesGpioHandle hGpioTemp = NULL;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvU32 NumOfGuids = 1;
    NvU64 guid;
    NvU32 searchVals[2];
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;
    NvBool Status = NV_TRUE;
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

    if (pConnectivity->Guid == WLAN_GUID)
    {
        // Getting the OdmGpio Handle
        hGpioTemp = NvOdmGpioOpen();
        if (hGpioTemp == NULL)
        {
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }

        // Search for the Vdd rail and set the proper volage to the rail.
        if (pConnectivity->AddressList[1].Interface == NvOdmIoModule_Gpio)
        {
             // Acquiring Pin Handles for Power Pin
             pDevice->hPwrPin= NvOdmGpioAcquirePinHandle(hGpioTemp,
                   pConnectivity->AddressList[1].Instance,
                   pConnectivity->AddressList[1].Address);
        }

        if (pConnectivity->AddressList[2].Interface == NvOdmIoModule_Gpio)
        {
             // Acquiring Pin Handles for Reset Pin
             pDevice->hResetPin= NvOdmGpioAcquirePinHandle(hGpioTemp,
                   pConnectivity->AddressList[2].Instance,
                   pConnectivity->AddressList[2].Address);
        }

        // Setting the ON/OFF pin to output mode.
        NvOdmGpioConfig(hGpioTemp, pDevice->hPwrPin, NvOdmGpioPinMode_Output);
        NvOdmGpioConfig(hGpioTemp, pDevice->hResetPin, NvOdmGpioPinMode_Output);

        // Setting the Output Pin to Low
        NvOdmGpioSetState(hGpioTemp, pDevice->hPwrPin, 0x0);
        NvOdmGpioSetState(hGpioTemp, pDevice->hResetPin, 0x0);

        pDevice->hGpio = hGpioTemp;

        Status = SdioOdmWlanPower(pDevice, NV_TRUE);
        if (Status != NV_TRUE)
        {
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }
    }
    pDevice->PoweredOn = NV_TRUE;
    pDevice->Instance = Instance;
    return pDevice;
}

void NvOdmSdioClose(NvOdmSdioHandle hOdmSdio)
{
    if (hOdmSdio->pConnectivity->Guid == WLAN_GUID)
    {
        // Call Turn off power when close is Called
        (void)SdioOdmWlanPower(hOdmSdio, NV_FALSE);

        NvOdmGpioReleasePinHandle(hOdmSdio->hGpio, hOdmSdio->hPwrPin);
        NvOdmGpioReleasePinHandle(hOdmSdio->hGpio, hOdmSdio->hResetPin);
        NvOdmGpioClose(hOdmSdio->hGpio);
    }
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

    if (hOdmSdio->pConnectivity->Guid == WLAN_GUID)
        Status = SdioOdmWlanPower(hOdmSdio, NV_FALSE);

    hOdmSdio->PoweredOn = NV_FALSE;
    return Status;

}

NvBool NvOdmSdioResume(NvOdmSdioHandle hOdmSdio)
{
    NvBool Status = NV_TRUE;

    if (hOdmSdio->PoweredOn)
        return NV_TRUE;

    NvOdmSetPowerOnSdio(hOdmSdio, NV_TRUE);

    if (hOdmSdio->pConnectivity->Guid == WLAN_GUID)
        Status = SdioOdmWlanPower(hOdmSdio, NV_TRUE);

    hOdmSdio->PoweredOn = NV_TRUE;
    return Status;
}
