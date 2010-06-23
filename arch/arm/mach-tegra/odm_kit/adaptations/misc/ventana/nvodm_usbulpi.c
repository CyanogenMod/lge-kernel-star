/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
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
} NvOdmUsbUlpi;

NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
    NvOdmUsbUlpi *pDevice = NULL;
    NvU32 ClockInstances[MAX_CLOCKS];
    NvU32 ClockFrequencies[MAX_CLOCKS];
    NvU32 NumClocks;
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle hResetPin;
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
    hGpio = NvOdmGpioOpen();
    hResetPin = NvOdmGpioAcquirePinHandle(hGpio, Port, Pin);
    // config as out put pin
    NvOdmGpioConfig(hGpio,hResetPin, NvOdmGpioPinMode_Output);
    // Set low to write high on ULPI_RESETB pin
    NvOdmGpioSetState(hGpio, hResetPin, 0x01);
    NvOdmGpioSetState(hGpio, hResetPin, 0x0);
    NvOdmOsSleepMS(5);
    NvOdmGpioSetState(hGpio, hResetPin, 0x01);

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
        hOdmUlpi = NULL;
    }
}

