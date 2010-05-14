/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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
 * @file          Nvodm_Sdio.c
 * @brief         <b>Sdio odm implementation</b>
 *
 * @Description : Implementation of the odm sdio API
 */
#include "nvodm_sdio.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvodm_pmu.h"

#ifdef NV_DRIVER_DEBUG
    #define NV_DRIVER_TRACE(x) NvOdmOsDebugPrintf x
#else
    #define NV_DRIVER_TRACE(x)
#endif

#define WLAN_GUID            NV_ODM_GUID('s','d','i','o','w','l','a','n')
// Device Board definitions
#define BOARD_ID_E951        (0x0933) /* Decimal  951. => ((9<<8) | 51)*/


typedef enum
{
    NvOdmSdioDiscoveryAddress_0 = 0,
    NvOdmSdioDiscoveryAddress_1,
    
    NvOdmSdioDiscoveryAddress_Force32 = 0x7FFFFFFF,
        
} NvOdmSdioDiscoveryAddress;

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
    NvOdmPeripheralConnectivity *pConnectivity;
    // Power state
    NvBool PoweredOn;
    // Instance
    NvU32 Instance;
} NvOdmSdio;

static void NvOdmSetPowerOnSdio(NvOdmSdioHandle pDevice, NvBool IsEnable);
static NvBool SdioOdmWlanSetPowerOn(NvOdmSdioHandle hOdmSdio, NvBool IsEnable);


static NvBool SdioOdmWlanSetPowerOn(NvOdmSdioHandle hOdmSdio, NvBool IsEnable)
{
    if (IsEnable) 
    {
        // Wlan Power On Reset Sequence
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);      //PWD -> Low
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x0);    //RST -> Low
        NvOdmOsWaitUS(2000);
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x1);      //PWD -> High
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x1);    //RST -> High
     }
     else 
     {
         // Power Off sequence
         NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);     //PWD -> Low
     }
     return NV_TRUE;
}

NvOdmSdioHandle NvOdmSdioOpen(NvU32 Instance)
{
    static NvOdmSdio *pDevice = NULL;
    NvOdmServicesGpioHandle hGpioTemp = NULL;
    NvOdmPeripheralConnectivity *pConnectivity;
    NvU32 NumOfGuids = 1;
    NvU64 guid;
    NvU32 searchVals[4];
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;
    NvBool Status = NV_TRUE;
    const NvOdmPeripheralSearch searchAttrs[] =
    {
        NvOdmPeripheralSearch_PeripheralClass,
        NvOdmPeripheralSearch_IoModule,
        NvOdmPeripheralSearch_Instance,
        NvOdmPeripheralSearch_Address,
    };
    NvOdmBoardInfo BoardInfo;
    NvBool status = NV_FALSE;
    
    searchVals[0] =  NvOdmPeripheralClass_Other;
    searchVals[1] =  NvOdmIoModule_Sdio;
    searchVals[2] =  Instance;

    NvOdmQueryPinMux(NvOdmIoModule_Sdio, &pOdmConfigs, &NumOdmConfigs); 
    if ((Instance == 0) && (pOdmConfigs[0] == NvOdmSdioPinMap_Config1))
    {
        // sdio is connected to sdio2 slot.  
        searchVals[3] =  NvOdmSdioDiscoveryAddress_1;
    }
    else
    {
        // sdio is connected to wifi module.
        searchVals[3] =  NvOdmSdioDiscoveryAddress_0;
    }

    NumOfGuids = NvOdmPeripheralEnumerate(searchAttrs,
                                          searchVals,
                                          4,
                                          &guid,
                                          NumOfGuids);

    // Get the peripheral connectivity information
    pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(guid);
    if (pConnectivity == NULL)
        return NULL;

    pDevice = NvOdmOsAlloc(sizeof(NvOdmSdio));
    if(pDevice == NULL)
        return (pDevice);

    pDevice->hPmu = NvOdmServicesPmuOpen();
    if(pDevice->hPmu == NULL)
    {
        NvOdmOsFree(pDevice);
        pDevice = NULL;
        return (NULL);
    }

    if (pConnectivity->Guid == WLAN_GUID)
    {
        // WARNING: This function *cannot* be called before RmOpen().
        status = NvOdmPeripheralGetBoardInfo((BOARD_ID_E951), &BoardInfo);
        if (NV_TRUE != status)
        {
            // whistler should have E951 Module, if it is not presnt return NULL Handle.
            NvOdmServicesPmuClose(pDevice->hPmu);
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            NvOdmOsDebugPrintf(("No E951 Detected"));
            return (pDevice);
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
            NvOdmServicesPmuClose(pDevice->hPmu);
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

        Status = SdioOdmWlanSetPowerOn(pDevice, NV_TRUE);
        if (Status != NV_TRUE)
        {
            NvOdmServicesPmuClose(pDevice->hPmu);
            NvOdmGpioReleasePinHandle(pDevice->hGpio, pDevice->hPwrPin);
            NvOdmGpioReleasePinHandle(pDevice->hGpio, pDevice->hResetPin);    
            NvOdmGpioClose(pDevice->hGpio);   
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }
    }
    pDevice->PoweredOn = NV_TRUE;
    pDevice->Instance = Instance;
    NV_DRIVER_TRACE(("Open SDIO%d", Instance));
    return pDevice;
}

void NvOdmSdioClose(NvOdmSdioHandle hOdmSdio)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    NV_DRIVER_TRACE(("Close SDIO%d", hOdmSdio->Instance));

    pConnectivity = hOdmSdio->pConnectivity;
    if (pConnectivity->Guid == WLAN_GUID)
    {
        // Call Turn off power when close is Called
        (void)SdioOdmWlanSetPowerOn(hOdmSdio, NV_FALSE);

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

static void NvOdmSetPowerOnSdio(NvOdmSdioHandle pDevice, NvBool IsEnable)
{
    NvU32 Index = 0;
    NvOdmServicesPmuVddRailCapabilities RailCaps;
    NvU32 SettlingTime = 0;
    const NvOdmPeripheralConnectivity *pConnectivity;

    pConnectivity = pDevice->pConnectivity;
    if (IsEnable) // Turn on Power
    {
        // Search for the Vdd rail and set the proper volage to the rail.
        for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
        {
            if (pConnectivity->AddressList[Index].Interface == NvOdmIoModule_Vdd)
            {
                NvOdmServicesPmuGetCapabilities(pDevice->hPmu, pConnectivity->AddressList[Index].Address, &RailCaps);
                NvOdmServicesPmuSetVoltage(pDevice->hPmu, pConnectivity->AddressList[Index].Address,
                                RailCaps.requestMilliVolts, &SettlingTime);
                if (SettlingTime)
                {
                    NvOdmOsWaitUS(SettlingTime);
                }
            }
        }
    }
    else // Shutdown Power
    {
        // Search for the Vdd rail and power Off the module
        for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
        {
            if (pConnectivity->AddressList[Index].Interface == NvOdmIoModule_Vdd)
            {
                NvOdmServicesPmuGetCapabilities(pDevice->hPmu, pConnectivity->AddressList[Index].Address, &RailCaps);
                NvOdmServicesPmuSetVoltage(pDevice->hPmu, pConnectivity->AddressList[Index].Address,
                                ODM_VOLTAGE_OFF, &SettlingTime);
                if (SettlingTime)
                {
                    NvOdmOsWaitUS(SettlingTime);
                }
            }
        }
    }
}

NvBool NvOdmSdioSuspend(NvOdmSdioHandle hOdmSdio)
{

    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvBool Status = NV_TRUE;

    if (!hOdmSdio->PoweredOn)
    {
        NV_DRIVER_TRACE(("SDIO%d already suspended", hOdmSdio->Instance));
        return NV_TRUE;
    }

    NV_DRIVER_TRACE(("Suspend SDIO%d", hOdmSdio->Instance));
    NvOdmSetPowerOnSdio(hOdmSdio, NV_FALSE);

    pConnectivity = hOdmSdio->pConnectivity;
    if (pConnectivity->Guid == WLAN_GUID)
    {
        // Turn off power
        Status = SdioOdmWlanSetPowerOn(hOdmSdio, NV_FALSE);

    }
    hOdmSdio->PoweredOn = NV_FALSE;
    return Status;

}

NvBool NvOdmSdioResume(NvOdmSdioHandle hOdmSdio)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvBool Status = NV_TRUE;

    if (hOdmSdio->PoweredOn)
    {
        NV_DRIVER_TRACE(("SDIO%d already resumed", hOdmSdio->Instance));
        return NV_TRUE;
    }

    NvOdmSetPowerOnSdio(hOdmSdio, NV_TRUE);

    pConnectivity = hOdmSdio->pConnectivity;
    if (pConnectivity->Guid == WLAN_GUID)
    {
        // Turn on power
        Status = SdioOdmWlanSetPowerOn(hOdmSdio, NV_TRUE);
    }
    NV_DRIVER_TRACE(("Resume SDIO%d", hOdmSdio->Instance));
    hOdmSdio->PoweredOn = NV_TRUE;
    return Status;
}
