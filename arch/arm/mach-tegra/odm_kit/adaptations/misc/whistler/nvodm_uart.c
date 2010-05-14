/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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
 * @file          nvodm_uart.c
 * @brief         <b>Adaptation for uart </b>
 *
 * @Description : Implementation of the uart adaptation.
 */
#include "nvodm_uart.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvos.h"
#include "nvodm_pmu.h"

#ifdef NV_DRIVER_DEBUG
    #define NV_DRIVER_TRACE NvOsDebugPrintf
#else
    #define NV_DRIVER_TRACE (void)
#endif

typedef struct NvOdmUartRec
{
    // NvODM PMU device handle
    NvOdmServicesPmuHandle hPmu;
    // Gpio Handle
    NvOdmServicesGpioHandle hGpio;    
    // Pin handle to Bluetooth Reset Gpio pin
    NvOdmGpioPinHandle hResetPin;
    NvOdmPeripheralConnectivity *pConnectivity;
} NvOdmUart;

NvOdmUartHandle NvOdmUartOpen(NvU32 Instance)
{
    NvOdmUart *pDevice = NULL;
    NvOdmPeripheralConnectivity *pConnectivity;
    NvU32 NumOfGuids = 1;
    NvU64 guid;
    NvU32 searchVals[2];
    const NvOdmPeripheralSearch searchAttrs[] =
    {
        NvOdmPeripheralSearch_IoModule,
        NvOdmPeripheralSearch_Instance,
    };
    
    searchVals[0] =  NvOdmIoModule_Uart;
    searchVals[1] =  Instance;

    NumOfGuids = NvOdmPeripheralEnumerate(
                                                    searchAttrs,
                                                    searchVals,
                                                    2,
                                                    &guid,
                                                    NumOfGuids); 

    pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(guid);
    if (pConnectivity == NULL)
        goto ExitUartOdm;
    
    pDevice = NvOdmOsAlloc(sizeof(NvOdmUart));
    if(pDevice == NULL)
        goto ExitUartOdm;
    
    pDevice->hPmu = NvOdmServicesPmuOpen();
    if(pDevice->hPmu == NULL)
    {
        goto ExitUartOdm;
    }

    // Switch On UART Interface

    pDevice->pConnectivity = pConnectivity;
   
    return pDevice;

ExitUartOdm:
    NvOdmOsFree(pDevice);
    pDevice = NULL;
    
    return NULL;
}

void NvOdmUartClose(NvOdmUartHandle hOdmUart)
{

    if (hOdmUart)
    {
        // Switch OFF UART Interface

        if (hOdmUart->hPmu != NULL)
        {
            NvOdmServicesPmuClose(hOdmUart->hPmu);
        }
        NvOdmOsFree(hOdmUart);
        hOdmUart = NULL;
    }
}

NvBool NvOdmUartSuspend(NvOdmUartHandle hOdmUart)
{
    return NV_FALSE;
}

NvBool NvOdmUartResume(NvOdmUartHandle hOdmUart)
{
    return NV_FALSE;
}

