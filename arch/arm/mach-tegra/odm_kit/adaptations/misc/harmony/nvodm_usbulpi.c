/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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
 
/**
 * @file          nvodm_usbulpi.c
 * @brief         <b>Adaptation for USB ULPI </b>
 *
 * @Description : Implementation of the USB ULPI adaptation.
 */
#include "nvodm_usbulpi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvos.h"

#define SMSC3317GUID NV_ODM_GUID('s','m','s','c','3','3','1','7')

#define MAX_CLOCKS 3

#define NVODM_PORT(x) ((x) - 'a')
#define ULPI_RESET_PORT NVODM_PORT('v')
#define ULPI_RESET_PIN 1


#ifdef NV_DRIVER_DEBUG
    #define NV_DRIVER_TRACE NvOsDebugPrintf
#else
    #define NV_DRIVER_TRACE (void)
#endif

typedef struct NvOdmUsbUlpiRec
{
     NvU64    CurrentGUID;
} NvOdmUsbUlpi;

static NvOdmServicesGpioHandle s_hGpio = NULL;
static NvOdmGpioPinHandle s_hResetPin = NULL;

NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
    NvOdmUsbUlpi*pDevice = NULL;
    NvU32 ClockInstances[MAX_CLOCKS];
    NvU32 ClockFrequencies[MAX_CLOCKS];
    NvU32 NumClocks;

    pDevice = NvOdmOsAlloc(sizeof(NvOdmUsbUlpi));
    if(pDevice == NULL)
		return NULL;
    
    if(!NvOdmExternalClockConfig(SMSC3317GUID, NV_FALSE, ClockInstances,
					ClockFrequencies, &NumClocks))
    {
        NV_DRIVER_TRACE (("ERROR NvOdmUsbUlpiOpen: "
				"NvOdmExternalClockConfig fail\n"));
        goto ExitUlpiOdm;
    }
    NvOdmOsSleepMS(10);

    if (!s_hGpio)
        s_hGpio = NvOdmGpioOpen();
    if (!s_hGpio)
    {
        NV_DRIVER_TRACE (("ERROR NvOdmUsbUlpiOpen: "
				"Not able to open gpio handle\n"));
        goto ExitUlpiOdm;
    }

    if (!s_hResetPin)
        s_hResetPin = NvOdmGpioAcquirePinHandle(s_hGpio, ULPI_RESET_PORT,
							ULPI_RESET_PIN);
    if (!s_hResetPin)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
        NV_DRIVER_TRACE (("ERROR NvOdmGpioAcquirePinHandle: "
					"Not able to Acq pinhandle\n"));
        goto ExitUlpiOdm;
    }

    // Pull high on RESETB ( 22nd pin of smsc3315) 
    // config as out put pin
    NvOdmGpioConfig(s_hGpio,s_hResetPin, NvOdmGpioPinMode_Output);
    // Set low to write high on ULPI_RESETB pin
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x01);
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x0);
    NvOdmOsSleepMS(5);
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x01);

    pDevice->CurrentGUID = SMSC3317GUID;
    return pDevice;

ExitUlpiOdm:
    NvOdmOsFree(pDevice);
    return NULL;
}

void NvOdmUsbUlpiClose(NvOdmUsbUlpiHandle hOdmUlpi)
{
    if (hOdmUlpi)
    {
        NvOdmOsFree(hOdmUlpi);
    }
    if (s_hResetPin)
    {
        NvOdmGpioReleasePinHandle(s_hGpio, s_hResetPin);
        s_hResetPin = NULL;
    }
    if (s_hGpio)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
    }

}

