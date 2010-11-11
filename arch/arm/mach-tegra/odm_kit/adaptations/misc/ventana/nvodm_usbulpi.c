/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
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


typedef struct NvOdmUsbUlpiRec
{
     NvU64    CurrentGUID;
     NvOdmServicesGpioHandle hGpio;
     NvOdmGpioPinHandle hResetPin;
} NvOdmUsbUlpi;

NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
    NvOdmUsbUlpi *pDevice = NULL;
    NvU32 ClockInstances[MAX_CLOCKS];
    NvU32 ClockFrequencies[MAX_CLOCKS];
    NvU32 NumClocks;
    NvU32 Port = NVODM_PORT('v');
    NvU32 Pin = 1;

    pDevice = NvOdmOsAlloc(sizeof(NvOdmUsbUlpi));
    if (pDevice == NULL)
        return NULL;

    if(!NvOdmExternalClockConfig(SMSC3317GUID, NV_FALSE,
                                 ClockInstances, ClockFrequencies, &NumClocks))
    {
        NvOdmOsDebugPrintf("NvOdmUsbUlpiOpen: NvOdmExternalClockConfig fail\n");
        goto ExitUlpiOdm;
    }
    NvOdmOsSleepMS(10);
    // Pull high on RESETB ( 22nd pin of smsc3315)
    pDevice->hGpio = NvOdmGpioOpen();
    pDevice->hResetPin = NvOdmGpioAcquirePinHandle(pDevice->hGpio, Port, Pin);
    // config as out put pin
    NvOdmGpioConfig(pDevice->hGpio, pDevice->hResetPin, NvOdmGpioPinMode_Output);
    // Set low to write high on ULPI_RESETB pin
    NvOdmGpioSetState(pDevice->hGpio, pDevice->hResetPin, 0x01);
    NvOdmGpioSetState(pDevice->hGpio, pDevice->hResetPin, 0x0);
    NvOdmOsSleepMS(5);
    NvOdmGpioSetState(pDevice->hGpio, pDevice->hResetPin, 0x01);

    pDevice->CurrentGUID = SMSC3317GUID;
    return pDevice;

ExitUlpiOdm:
    NvOdmOsFree(pDevice);
    return NULL;
}

void NvOdmUsbUlpiClose(NvOdmUsbUlpiHandle hOdmUlpi)
{
    if (hOdmUlpi->hResetPin)
    {
        NvOdmGpioSetState(hOdmUlpi->hGpio, hOdmUlpi->hResetPin, 0x0);
        NvOdmGpioReleasePinHandle(hOdmUlpi->hGpio, hOdmUlpi->hResetPin);
        hOdmUlpi->hResetPin = NULL;
    }
    if (hOdmUlpi->hGpio)
    {
        NvOdmGpioClose(hOdmUlpi->hGpio);
        hOdmUlpi->hGpio = NULL;
    }
    if (hOdmUlpi)
    {
        NvOdmOsFree(hOdmUlpi);
    }
}

