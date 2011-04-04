/*
 * arch/arm/mach-tegra/odm_kit/adaptions/misc/whistler/nvodm_sdio.c
 *
 * Implementation of the odm sdio API
 *
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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

/* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [START] */
static int nBlink32KClockOnWlan = 0;
static int nBlink32KClockOnBt = 0;
NvOdmServicesPwmHandle hBcmPwm;
/* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [END] */

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
    NvOdmServicesPwmHandle hPwm;
} NvOdmSdio;

static void NvOdmSetPowerOnSdio(NvOdmSdioHandle pDevice, NvBool IsEnable);
static NvBool SdioOdmWlanSetPowerOn(NvOdmSdioHandle hOdmSdio, NvBool IsEnable);


static NvBool SdioOdmWlanSetPowerOn(NvOdmSdioHandle hOdmSdio, NvBool IsEnable)
{
    NvU32 RequestedPeriod, ReturnedPeriod;
    RequestedPeriod = 0;

    /* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [START] */
    if (IsEnable) 
    {
        if (nBlink32KClockOnWlan == 0 && nBlink32KClockOnBt == 0)
        {
            NvOdmPwmConfig(hOdmSdio->hPwm, NvOdmPwmOutputId_Blink, NvOdmPwmMode_Blink_32KHzClockOutput, 0, &RequestedPeriod, &ReturnedPeriod);
            NvOdmOsDebugPrintf("ODM SDIO : turn on 32KHz clock\n");
        }
        nBlink32KClockOnWlan = 1;

        // Wlan Power On Reset Sequence
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);      //PWD -> Low
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x0);    //RST -> Low
        NvOdmOsWaitUS(10000);
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x1);      //PWD -> High
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x1);    //RST -> High
        NvOdmOsWaitUS(10000);
    }
    else 
    {
        // Power Off sequence
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hPwrPin, 0x0);     //PWD -> Low
        NvOdmGpioSetState(hOdmSdio->hGpio, hOdmSdio->hResetPin, 0x0);    //RST -> Low

        if(nBlink32KClockOnWlan == 1 && nBlink32KClockOnBt == 0)
        {
            NvOdmPwmConfig(hOdmSdio->hPwm, NvOdmPwmOutputId_Blink, NvOdmPwmMode_Blink_Disable, 0, &RequestedPeriod, &ReturnedPeriod);
            NvOdmOsDebugPrintf("ODM SDIO : turn off 32KHz clock\n");
        }
        nBlink32KClockOnWlan = 0;

     }
     /* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [END] */
     return NV_TRUE;
}

// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [START]
#include <linux/module.h>
NvOdmSdio *g_pDeviceWlan = NULL;
NvBool NvOdmWlanEnable( NvBool IsEnable )
{
	if( g_pDeviceWlan != NULL ){
		if(IsEnable)
		{
        	NvOdmOsDebugPrintf("WLAN Power ON ~~~\n");
			SdioOdmWlanSetPowerOn(g_pDeviceWlan, NV_TRUE);
		}
		else
		{
        	NvOdmOsDebugPrintf("WLAN Power OFF~~~\n");
			SdioOdmWlanSetPowerOn(g_pDeviceWlan, NV_FALSE);
		}
	}
	else
	{
        NvOdmOsDebugPrintf("g_pDeviceWlan is NULL\n");
		return NV_FALSE;
	}
	return NV_TRUE;
}
EXPORT_SYMBOL(NvOdmWlanEnable);
// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [END]

/* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [START] */
NvBool NvOdmBtEnable(NvBool IsEnable)
{
    NvU32 RequestedPeriod, ReturnedPeriod;
    NvOdmServicesPwmHandle hOdmPwm = NULL;

    RequestedPeriod = 0;
    hOdmPwm = NvOdmPwmOpen();
    if (!hOdmPwm) {
        pr_err("%s: failed to open NvOdmPwmOpen\n", __func__);
        return NV_FALSE;
    }

    if (IsEnable == NV_TRUE)
    {
        if (nBlink32KClockOnWlan == 0 && nBlink32KClockOnBt == 0)
        {
            NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_Blink, NvOdmPwmMode_Blink_32KHzClockOutput, 0, &RequestedPeriod, &ReturnedPeriod);
        }
        nBlink32KClockOnBt = 1;
    }
    else
    {
        if (nBlink32KClockOnWlan == 0 && nBlink32KClockOnBt == 1)
        {
            NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_Blink, NvOdmPwmMode_Blink_Disable, 0, &RequestedPeriod, &ReturnedPeriod);
        }

        nBlink32KClockOnBt = 0;
    }

    NvOdmPwmClose(hOdmPwm);

    return NV_TRUE;
}
EXPORT_SYMBOL(NvOdmBtEnable);
/* 20101005 taewan.kim@lge.com for debugging of resetting when getting a dump [END] */

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

    searchVals[0] =  NvOdmPeripheralClass_Other;
    searchVals[1] =  NvOdmIoModule_Sdio;
    searchVals[2] =  Instance;

    NvOdmQueryPinMux(NvOdmIoModule_Sdio, &pOdmConfigs, &NumOdmConfigs); 

    // sdio is connected to wifi module.
    searchVals[3] =  NvOdmSdioDiscoveryAddress_0;


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


    pDevice->pConnectivity = pConnectivity;
    NvOdmSetPowerOnSdio(pDevice, NV_TRUE);

    if (pConnectivity->Guid == WLAN_GUID)
    {
        NvOdmServicesPwmHandle hPwmTemp = NULL;
        
        // Getting the OdmGpio Handle
        hGpioTemp = NvOdmGpioOpen();
        if (hGpioTemp == NULL)
        {
            NvOdmServicesPmuClose(pDevice->hPmu);
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }

        hPwmTemp = NvOdmPwmOpen();
        NvOdmOsDebugPrintf("ODM SDIO : get pwm handle\n");
        if (hPwmTemp == NULL)
        {
            NvOdmGpioClose(hGpioTemp);
            NvOdmServicesPmuClose(pDevice->hPmu);
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }

        pDevice->hPwm = hPwmTemp;
/* 20100818 jaewoo56.lee@lge.com for debugging of resetting when getting a dump [START] */
        hBcmPwm = hPwmTemp;
/* 20100818 jaewoo56.lee@lge.com for debugging of resetting when getting a dump [END] */
    
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

// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [START]

		g_pDeviceWlan = pDevice;

/*
        Status = SdioOdmWlanSetPowerOn(pDevice, NV_TRUE);
        if (Status != NV_TRUE)
        {
            NvOdmServicesPmuClose(pDevice->hPmu);
            NvOdmGpioReleasePinHandle(pDevice->hGpio, pDevice->hPwrPin);
            NvOdmGpioReleasePinHandle(pDevice->hGpio, pDevice->hResetPin);    
            NvOdmGpioClose(pDevice->hGpio); 
            NvOdmPwmClose(pDevice->hPwm);
            NvOdmOsFree(pDevice);
            pDevice = NULL;
            return (pDevice);
        }
*/
// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [END]

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
        NvOdmOsDebugPrintf("ODM SDIO : close pwm handle\n");
        NvOdmPwmClose(hOdmSdio->hPwm);

// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [START]
		g_pDeviceWlan = NULL;
// 20100717 mingi.sung@lge.com WLAN power control code for nVidia [END]
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
// 20100827 mingi.sung@lge.com [WLAN] To prevent turning on/off WLAN when SDIO host controller suspends/resumes [START]
        //Status = SdioOdmWlanSetPowerOn(hOdmSdio, NV_FALSE);
// 20100827 mingi.sung@lge.com [WLAN] To prevent turning on/off WLAN when SDIO host controller suspends/resumes [END]

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
// 20100827 mingi.sung@lge.com [WLAN] To prevent turning on/off WLAN when SDIO host controller suspends/resumes [START]
        //Status = SdioOdmWlanSetPowerOn(hOdmSdio, NV_TRUE);
// 20100827 mingi.sung@lge.com [WLAN] To prevent turning on/off WLAN when SDIO host controller suspends/resumes [END]
    }
    NV_DRIVER_TRACE(("Resume SDIO%d", hOdmSdio->Instance));
    hOdmSdio->PoweredOn = NV_TRUE;
    return Status;
}
