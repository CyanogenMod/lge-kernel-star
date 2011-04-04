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

#include "nvodm_services.h"
#include "nvrm_gpio.h"
#include "nvrm_spi.h"
#include "nvrm_i2c.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pmu.h"
#include "nvrm_keylist.h"
#include "nvrm_pwm.h"
#include "nvrm_power.h"
#include "nvrm_analog.h"
#include "nvrm_pinmux.h"
#include "nvrm_init.h"


typedef struct NvOdmServicesGpioRec
{
    NvRmDeviceHandle hRmDev;
    NvRmGpioHandle hGpio;
} NvOdmServicesGpio;

typedef struct NvOdmServicesSpiRec
{
    NvRmDeviceHandle hRmDev;
    NvRmSpiHandle hSpi;
    NvOdmSpiPinMap SpiPinMap;
} NvOdmServicesSpi;

typedef struct NvOdmServicesI2cRec
{
    NvRmDeviceHandle hRmDev;
    NvRmI2cHandle hI2c;
    NvOdmI2cPinMap I2cPinMap;
} NvOdmServicesI2c;

typedef struct NvOdmServicesPwmRec
{
    NvRmDeviceHandle hRmDev;
    NvRmPwmHandle hPwm;
} NvOdmServicesPwm;
// ----------------------- GPIO IMPLEMENTATION ------------

NvOdmServicesGpioHandle NvOdmGpioOpen(void)
{
    NvError e;
    NvOdmServicesGpio *pOdmServices = NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesGpio));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesGpio));

    // Open RM device and RM GPIO handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmGpioOpen(
        pOdmServices->hRmDev, &pOdmServices->hGpio));

    return pOdmServices;

fail:
    NvOdmGpioClose(pOdmServices);
    return NULL;
}

void NvOdmGpioClose(NvOdmServicesGpioHandle hOdmGpio)
{
    if (!hOdmGpio)
        return;
    NvRmGpioClose(hOdmGpio->hGpio);
    NvRmClose(hOdmGpio->hRmDev);
    NvOsFree(hOdmGpio);
}

NvOdmGpioPinHandle 
NvOdmGpioAcquirePinHandle(NvOdmServicesGpioHandle hOdmGpio, 
        NvU32 Port, NvU32 Pin)
{
    NvError e;
    NvRmGpioPinHandle hGpioPin;

    if (!hOdmGpio || Port == NVODM_GPIO_INVALID_PORT || Pin == NVODM_GPIO_INVALID_PIN)
        return NULL;

    NV_CHECK_ERROR_CLEANUP(NvRmGpioAcquirePinHandle(
            hOdmGpio->hGpio, Port, Pin, &hGpioPin)); 

    return (NvOdmGpioPinHandle)hGpioPin;

fail:
    return NULL;
}

void
NvOdmGpioReleasePinHandle(NvOdmServicesGpioHandle hOdmGpio, 
        NvOdmGpioPinHandle hPin)
{
    NvRmGpioPinHandle hRmPin = (NvRmGpioPinHandle)hPin;
    if (hOdmGpio && hPin)
        NvRmGpioReleasePinHandles(hOdmGpio->hGpio, &hRmPin, 1);
}

//cs77.ha@lge.com LCD IF fast init
//NvBug 716430
// hGpioPin == pointer to array of GPIO pin handles, arraySize ==  numberOfPins
// PinValue == pointer to array of corresponding GPIN pin values
// numberOfPins = number of pins in the bus
void
NvOdmGpioSetBusState(
    NvOdmServicesGpioHandle hOdmGpio,
    NvOdmGpioPinHandle *hGpioPin,
    NvU32 *PinValue,
	NvU32 numberOfPins)
{
	NvU32 i;
    NvRmGpioPinState *val = (NvRmGpioPinState *)PinValue;
    NvRmGpioPinHandle *hRmPin = (NvRmGpioPinHandle *)hGpioPin;
    if (hOdmGpio == NULL || hGpioPin == NULL)
        return;
/*
	NvOsDebugPrintf( "NvOdmGpioSetBusState: ================\n");
    NvOsDebugPrintf( "  BusSize = %d\n", numberOfPins );
	for( i = 0; i< numberOfPins; i++)
	{
		NvOsDebugPrintf( "  Bus[%d], hndl = 0x%x, value = %d\n", i, hRmPin[i], val[i]);
	}
	NvOsDebugPrintf( "NvOdmGpioSetBusState: ================\n");
*/
    NvRmGpioWritePins( hOdmGpio->hGpio, hRmPin, val, numberOfPins);
}

void
NvOdmGpioSetState(
    NvOdmServicesGpioHandle hOdmGpio,
    NvOdmGpioPinHandle hGpioPin,
    NvU32 PinValue)
{
    NvRmGpioPinState val = (NvRmGpioPinState)PinValue;
    NvRmGpioPinHandle hRmPin = (NvRmGpioPinHandle)hGpioPin;
    if (hOdmGpio == NULL || hGpioPin == NULL)
        return;

    NvRmGpioWritePins( hOdmGpio->hGpio, &hRmPin, &val, 1);
}

void
NvOdmGpioGetState(
    NvOdmServicesGpioHandle hOdmGpio,
    NvOdmGpioPinHandle hGpioPin,
    NvU32 *PinValue)
{
    NvRmGpioPinState val;
    NvRmGpioPinHandle hRmPin = (NvRmGpioPinHandle)hGpioPin;

    if (hOdmGpio == NULL || hGpioPin == NULL)
        return;

    NvRmGpioReadPins(hOdmGpio->hGpio, &hRmPin, &val, 1);
    *PinValue = val;
}

void
NvOdmGpioConfig( 
    NvOdmServicesGpioHandle hOdmGpio, 
    NvOdmGpioPinHandle hGpioPin,
    NvOdmGpioPinMode mode)
{
    NvRmGpioPinHandle hRmPin = (NvRmGpioPinHandle)hGpioPin;
    if (hOdmGpio == NULL || hGpioPin == NULL)
        return;

    NV_ASSERT_SUCCESS(
        NvRmGpioConfigPins(hOdmGpio->hGpio, &hRmPin, 1, (NvRmGpioPinMode)mode)
    );
}

NvBool
NvOdmGpioInterruptRegister(NvOdmServicesGpioHandle hOdmGpio,
    NvOdmServicesGpioIntrHandle *hGpioIntr,
    NvOdmGpioPinHandle hGpioPin,
    NvOdmGpioPinMode Mode, 
    NvOdmInterruptHandler Callback,
    void *arg,
    NvU32 DebounceTime)
{
    NvRmGpioInterruptHandle handle;
    NvError err;

    err = NvRmGpioInterruptRegister(
        hOdmGpio->hGpio,
        hOdmGpio->hRmDev,
        (NvRmGpioPinHandle)hGpioPin,
        (NvOsInterruptHandler)Callback,
        (NvRmGpioPinMode)Mode,
        arg,
        &handle,
        DebounceTime);

    if (err == NvSuccess)
    {
        *hGpioIntr = (NvOdmServicesGpioIntrHandle)handle;
        err = NvRmGpioInterruptEnable(handle);
        if (err != NvSuccess)
        {
            NvRmGpioInterruptUnregister(hOdmGpio->hGpio,
                hOdmGpio->hRmDev,
                (NvRmGpioInterruptHandle)handle);
            *hGpioIntr = NULL;
            return NV_FALSE;
        }
        return NV_TRUE;
    }
    else
    {
        *hGpioIntr = NULL;
        return NV_FALSE;
    }
}

void
NvOdmGpioInterruptMask(NvOdmServicesGpioIntrHandle handle, NvBool mask)
{
    NvRmGpioInterruptMask( (NvRmGpioInterruptHandle)handle, mask);
}

void
NvOdmGpioInterruptUnregister(NvOdmServicesGpioHandle hOdmGpio,
    NvOdmGpioPinHandle hGpioPin,
    NvOdmServicesGpioIntrHandle handle)
{
    NvRmGpioInterruptUnregister(hOdmGpio->hGpio,
            hOdmGpio->hRmDev,
            (NvRmGpioInterruptHandle)handle);
}

void NvOdmGpioInterruptDone( NvOdmServicesGpioIntrHandle handle )
{
    NvRmGpioInterruptDone((NvRmGpioInterruptHandle)handle);
}

NvBool
NvOdmExternalClockConfig(
    NvU64 Guid,
    NvBool EnableTristate,
    NvU32 *pInstances,
    NvU32 *pFrequencies,
    NvU32 *pNum)
{
    const NvOdmPeripheralConnectivity *pConn = NULL;
    const NvOdmIoAddress *pIo = NULL;
    const NvU32 *pOdmConfigs = NULL;
    NvU32 NumOdmConfigs;
    NvRmDeviceHandle hRmDev = NULL;
    NvBool result = NV_TRUE;
    NvU32 i;
    NvU32 ClockListEntries = 0;

    NV_ASSERT(pInstances);
    NV_ASSERT(pFrequencies);
    NV_ASSERT(pNum);

    pConn = NvOdmPeripheralGetGuid(Guid);
    NvOdmQueryPinMux(NvOdmIoModule_ExternalClock, &pOdmConfigs, &NumOdmConfigs);
    
    if (NvRmOpen(&hRmDev,0)!=NvSuccess)
        return NV_FALSE;

    if (pConn && pConn->AddressList && pConn->NumAddress)
    {
        NvBool found = NV_FALSE;
        pIo = pConn->AddressList;
        for (i=0; i<pConn->NumAddress; pIo++, i++)
        {
            if (pIo->Interface == NvOdmIoModule_ExternalClock)
            {
                found = NV_TRUE;

                if (pIo->Instance >= NumOdmConfigs)
                    result = NV_FALSE;
                else 
                {
                    pInstances[ClockListEntries] = pIo->Instance;
                    pFrequencies[ClockListEntries] = NvRmExternalClockConfig(
                        hRmDev, NvOdmIoModule_ExternalClock, pIo->Instance, 
                        pOdmConfigs[pIo->Instance], EnableTristate);
                    ClockListEntries++;
                }
            }
        }
        result = result && found;
    }
    else
        result = NV_FALSE;

    *pNum = ClockListEntries;
    NvRmClose(hRmDev);
    return result;
}

NvBool NvOdmGetStraps(NvOdmStrapGroup StrapGroup, NvU32* pStrapValue)
{
/*
    NvRmDeviceHandle hRmDevice = NULL;
    NV_ASSERT(NvOdmStrapGroup_Num == NvRmStrapGroup_Num);

    if (NvRmOpen(&hRmDevice, 0) == NvSuccess)
    {
        if (NvRmGetStraps(
            hRmDevice, (NvRmStrapGroup)StrapGroup, pStrapValue) == NvSuccess)
            return NV_TRUE;
    }
*/    
    return NV_FALSE;
}

// ----------------------- I2C IMPLEMENTATION ------------

// Maximum number of bytes that can be sent between the i2c start and stop conditions
#define NVODM_I2C_PACKETSIZE   8

// Maximum number of bytes that can be sent between the i2c start and repeat start condition.
#define NVODM_I2C_REPEAT_START_PACKETSIZE   4

NvOdmServicesI2cHandle
NvOdmI2cOpen(
    NvOdmIoModule OdmIoModuleId,
    NvU32 instance)
{
    NvError e;
    NvOdmServicesI2c *pOdmServices = NULL;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;

    if (OdmIoModuleId != NvOdmIoModule_I2c &&
        OdmIoModuleId != NvOdmIoModule_I2c_Pmu)
        return NULL;
    
    NvOdmQueryPinMux(OdmIoModuleId, &pOdmConfigs, &NumOdmConfigs);
    if (instance>=NumOdmConfigs || !pOdmConfigs[instance] ||
        (pOdmConfigs[instance]==NvOdmI2cPinMap_Multiplexed))
        return NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesI2c));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesI2c));

    // Open RM device and RM I2C handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmI2cOpen(
        pOdmServices->hRmDev, OdmIoModuleId, instance, &pOdmServices->hI2c));

    pOdmServices->I2cPinMap = 0;
    return pOdmServices;

fail:
    NvOdmI2cClose(pOdmServices);
    return NULL;
}

NvOdmServicesI2cHandle
NvOdmI2cPinMuxOpen(
    NvOdmIoModule OdmIoModuleId,
    NvU32 instance,
    NvOdmI2cPinMap PinMap)
{
    NvError e;
    NvOdmServicesI2c *pOdmServices = NULL;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;

    if (OdmIoModuleId != NvOdmIoModule_I2c &&
        OdmIoModuleId != NvOdmIoModule_I2c_Pmu)
        return NULL;
    
    NvOdmQueryPinMux(OdmIoModuleId, &pOdmConfigs, &NumOdmConfigs);
    if (instance>=NumOdmConfigs ||
        (pOdmConfigs[instance]!=NvOdmI2cPinMap_Multiplexed))
        return NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesI2c));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesI2c));

    // Open RM device and RM I2C handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmI2cOpen(
        pOdmServices->hRmDev, OdmIoModuleId, instance, &pOdmServices->hI2c));

    pOdmServices->I2cPinMap = PinMap;
    return pOdmServices;

fail:
    NvOdmI2cClose(pOdmServices);
    return NULL;
}

void NvOdmI2cClose(NvOdmServicesI2cHandle hOdmI2c)
{
    if (!hOdmI2c)
        return;
    NvRmI2cClose(hOdmI2c->hI2c);
    NvRmClose(hOdmI2c->hRmDev);
    NvOsFree(hOdmI2c);
}


NvOdmI2cStatus
NvOdmI2cTransaction(
    NvOdmServicesI2cHandle hOdmI2c,
    NvOdmI2cTransactionInfo *TransactionInfo,
    NvU32 NumberOfTransactions,
    NvU32 ClockSpeedKHz,
    NvU32 WaitTimeoutInMilliSeconds)
{
    NvU32 len = 0;
    NvU8 *buffer = NULL;
    NvU8 stack_buffer[64];
    NvRmI2cTransactionInfo stack_t[8];
    NvRmI2cTransactionInfo *t = NULL;
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;
    NvError err;
    NvU32 i;
    NvU8 *tempBuffer;

    NV_ASSERT(hOdmI2c);
    NV_ASSERT(TransactionInfo);
    NV_ASSERT(NumberOfTransactions);
    NV_ASSERT(WaitTimeoutInMilliSeconds);

    for (i=0; i < NumberOfTransactions; i++)
    {
        len += TransactionInfo[i].NumBytes;
    }

    if (len > 64)
    {
        buffer = NvOsAlloc(len);
        if (!buffer)
        {
            status = NvOdmI2cStatus_InternalError;
            goto fail;
        }
    } else
    {
        buffer = stack_buffer;
    }

    if (NumberOfTransactions > 8)
    {
        t = NvOsAlloc(sizeof(NvRmI2cTransactionInfo) * NumberOfTransactions);
        if (!t)
        {
            status = NvOdmI2cStatus_InternalError;
            goto fail;
        }
    } else
    {
        t = stack_t;
    }

    NvOsMemset(buffer, 0, len);
    NvOsMemset(t, 0, sizeof(NvRmI2cTransactionInfo) * NumberOfTransactions);

    tempBuffer = buffer;
    for (i=0; i < NumberOfTransactions; i++)
    {
        if ( TransactionInfo[i].Flags & NVODM_I2C_SOFTWARE_CONTROLLER )
        {
            t[i].Flags |= NVRM_I2C_SOFTWARE_CONTROLLER;
        } 

        if ( TransactionInfo[i].Flags & NVODM_I2C_USE_REPEATED_START )
        {
            t[i].Flags |= NVRM_I2C_NOSTOP;
        } 

        if ( TransactionInfo[i].Flags & NVODM_I2C_NO_ACK )
        {
            t[i].Flags |= NVRM_I2C_NOACK;
        } 

        if ( TransactionInfo[i].Flags & NVODM_I2C_IS_WRITE )
        {
            t[i].Flags |= NVRM_I2C_WRITE;
            /* Copy the data */
            NvOsMemcpy(tempBuffer, TransactionInfo[i].Buf, TransactionInfo[i].NumBytes);
        } else
        {
            t[i].Flags |= NVRM_I2C_READ;
        }

        tempBuffer += TransactionInfo[i].NumBytes;
        t[i].NumBytes = TransactionInfo[i].NumBytes;
        t[i].Is10BitAddress = (NvBool)(TransactionInfo[i].Flags & NVODM_I2C_IS_10_BIT_ADDRESS);
        t[i].Address = (TransactionInfo[i].Address) & ~0x1;
    }

    err = NvRmI2cTransaction(hOdmI2c->hI2c, 
            hOdmI2c->I2cPinMap,
            WaitTimeoutInMilliSeconds,
            ClockSpeedKHz,
            buffer,
            len,
            t,
            NumberOfTransactions);
    if (err != NvSuccess)
    {    
        switch ( err )
        {
            case NvError_I2cDeviceNotFound:
                status =  NvOdmI2cStatus_SlaveNotFound;
                break;
            case NvError_I2cReadFailed:
                NvOsDebugPrintf("[Kernel : I2C ERROR] NvError_I2cDeviceNotFound \n");
                status = NvOdmI2cStatus_ReadFailed;
                break;
            case NvError_I2cWriteFailed:
                NvOsDebugPrintf("[Kernel : I2C ERROR] NvError_I2cWriteFailed \n");
                status = NvOdmI2cStatus_WriteFailed;
                break;
            case NvError_I2cArbitrationFailed:
                NvOsDebugPrintf("[Kernel : I2C ERROR] NvError_I2cArbitrationFailed \n");
                status = NvOdmI2cStatus_ArbitrationFailed;
                break;
            case NvError_I2cInternalError:
                NvOsDebugPrintf("[Kernel : I2C ERROR] NvError_I2cInternalError \n");
                status = NvOdmI2cStatus_InternalError;
                break;
            case NvError_Timeout:
                NvOsDebugPrintf("[Kernel : I2C ERROR] NvError_Timeout \n");
            default:
                status = NvOdmI2cStatus_Timeout;
                break;
        }
        goto fail;
    }

    tempBuffer = buffer;
    for (i=0; i < NumberOfTransactions; i++)
    {
        if (t[i].Flags & NVRM_I2C_READ)
        {
            NvOsMemcpy(TransactionInfo[i].Buf, tempBuffer, TransactionInfo[i].NumBytes);
        }
        tempBuffer += TransactionInfo[i].NumBytes;
    }

fail:

    if (t != NULL && t != stack_t)
    {
        NvOsFree(t);
    }

    if (buffer != NULL && buffer != stack_buffer)
    {
        NvOsFree(buffer);
    }

    return status;
}

NvOdmServicesSpiHandle NvOdmSpiOpen(NvOdmIoModule OdmIoModule, NvU32 ControllerId)
{
    NvError e;
    NvOdmServicesSpi *pOdmServices;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;

    // Supporting the odm_sflash and odm_spi only.
    if (OdmIoModule != NvOdmIoModule_Sflash &&
        OdmIoModule != NvOdmIoModule_Spi)
        return NULL;

    NvOdmQueryPinMux(OdmIoModule, &pOdmConfigs, &NumOdmConfigs);
    if (ControllerId>=NumOdmConfigs || !pOdmConfigs[ControllerId] ||
        (pOdmConfigs[ControllerId]==NvOdmSpiPinMap_Multiplexed))
        return NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesSpi));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesSpi));

    // Open RM device and RM SPI handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmSpiOpen(
        pOdmServices->hRmDev, OdmIoModule, ControllerId, NV_TRUE, &pOdmServices->hSpi));

    pOdmServices->SpiPinMap = 0;
    return pOdmServices;

fail:
    NvOdmSpiClose(pOdmServices);
    return NULL;
}

NvOdmServicesSpiHandle
NvOdmSpiPinMuxOpen(NvOdmIoModule OdmIoModule, 
                   NvU32 ControllerId, 
                   NvOdmSpiPinMap PinMap)
{
    NvError e;
    NvOdmServicesSpi *pOdmServices;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;

    if (OdmIoModule != NvOdmIoModule_Sflash &&
        OdmIoModule != NvOdmIoModule_Spi)
        return NULL;

    NvOdmQueryPinMux(OdmIoModule, &pOdmConfigs, &NumOdmConfigs);
    if ((ControllerId >= NumOdmConfigs) ||
        (pOdmConfigs[ControllerId]!=NvOdmSpiPinMap_Multiplexed))
        return NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesSpi));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesSpi));

    // Open RM device and RM SPI handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmSpiOpen(
        pOdmServices->hRmDev, OdmIoModule, ControllerId, NV_TRUE, &pOdmServices->hSpi));

    pOdmServices->SpiPinMap = PinMap;

    return pOdmServices;

fail:
    NvOdmSpiClose(pOdmServices);
    return NULL;
}

NvOdmServicesSpiHandle 
NvOdmSpiSlaveOpen(
    NvOdmIoModule OdmIoModule, 
    NvU32 ControllerId)
{
    NvError e;
    NvOdmServicesSpi *pOdmServices;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;

    // Supporting the odm_sflash and odm_spi only.
    if (OdmIoModule != NvOdmIoModule_Sflash &&
        OdmIoModule != NvOdmIoModule_Spi)
        return NULL;

    NvOdmQueryPinMux(OdmIoModule, &pOdmConfigs, &NumOdmConfigs);
    if (ControllerId>=NumOdmConfigs || !pOdmConfigs[ControllerId] ||
        (pOdmConfigs[ControllerId]==NvOdmSpiPinMap_Multiplexed))
        return NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesSpi));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesSpi));

    // Open RM device and RM SPI handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmSpiOpen(
        pOdmServices->hRmDev, OdmIoModule, ControllerId, NV_FALSE, &pOdmServices->hSpi));

    pOdmServices->SpiPinMap = 0;
    return pOdmServices;

fail:
    NvOdmSpiClose(pOdmServices);
    return NULL;

}



void NvOdmSpiClose(NvOdmServicesSpiHandle hOdmSpi)
{
    if (!hOdmSpi)
        return;

    // clean up
    NvRmSpiClose(hOdmSpi->hSpi);
    NvRmClose(hOdmSpi->hRmDev);
    NvOsFree(hOdmSpi);
}

void
NvOdmSpiTransaction(
    NvOdmServicesSpiHandle hOdmSpi,
    NvU32 ChipSelect,
    NvU32 ClockSpeedInKHz,
    NvU8 *ReadBuf,
    const NvU8 *WriteBuf,
    NvU32 Size,
    NvU32 PacketSize)
{
    NvRmSpiTransaction(hOdmSpi->hSpi, hOdmSpi->SpiPinMap, ChipSelect,
        ClockSpeedInKHz, ReadBuf, (NvU8 *)WriteBuf, Size, PacketSize);
}

NvBool 
NvOdmSpiSlaveStartTransaction( 
    NvOdmServicesSpiHandle hOdmSpi,
    NvU32 ChipSelectId,
    NvU32 ClockSpeedInKHz,
    NvBool IsReadTransfer,
    NvU8 * pWriteBuffer,
    NvU32 BytesRequested,
    NvU32 PacketSizeInBits )
{
    NvError Error;
    Error = NvRmSpiStartTransaction(hOdmSpi->hSpi, ChipSelectId,
        ClockSpeedInKHz, IsReadTransfer, (NvU8 *)pWriteBuffer, 
        BytesRequested, PacketSizeInBits);
    if (Error)
        return NV_FALSE;
    return NV_TRUE;        
    
}

NvBool 
NvOdmSpiSlaveGetTransactionData( 
    NvOdmServicesSpiHandle hOdmSpi,
    NvU8 * pReadBuffer,
    NvU32 BytesRequested,
    NvU32 * pBytesTransfererd,
    NvU32 WaitTimeout )
{
    NvError Error;
    Error = NvRmSpiGetTransactionData(hOdmSpi->hSpi, pReadBuffer, 
                BytesRequested, pBytesTransfererd, WaitTimeout);

    if (Error)
        return NV_FALSE;
    return NV_TRUE;
}

void
NvOdmSpiSetSignalMode(
    NvOdmServicesSpiHandle hOdmSpi,
    NvU32 ChipSelectId,
    NvOdmQuerySpiSignalMode SpiSignalMode)
{
    NvRmSpiSetSignalMode(hOdmSpi->hSpi, ChipSelectId, SpiSignalMode);
}




NvOdmServicesPmuHandle NvOdmServicesPmuOpen(void)
{
    NvRmDeviceHandle hRmDev;

    if (NvRmOpen(&hRmDev, 0) != NvError_Success)
    {
        return (NvOdmServicesPmuHandle)0;
    }

    return (NvOdmServicesPmuHandle)hRmDev;
}

void NvOdmServicesPmuClose(NvOdmServicesPmuHandle handle)
{
    NvRmClose((NvRmDeviceHandle)handle);
    return;
}

void NvOdmServicesPmuGetCapabilities( 
        NvOdmServicesPmuHandle handle,
        NvU32 vddId, 
        NvOdmServicesPmuVddRailCapabilities *pCapabilities )
{
    NvRmDeviceHandle hRmDev = (NvRmDeviceHandle)handle;

    NV_ASSERT(sizeof(NvRmPmuVddRailCapabilities) == 
            sizeof(NvOdmServicesPmuVddRailCapabilities));

    NvRmPmuGetCapabilities(hRmDev, vddId, 
        (NvRmPmuVddRailCapabilities *)pCapabilities);
    return;
}

#if defined(CONFIG_MACH_STAR) //20100704 bergkamp.cho@lge.com jongik's headset porting [LGE]
NvU32 NvOdmServicesPmuGetHookValue( 
        NvOdmServicesPmuHandle handle)
{
    NvU32 value =0;
    NvRmDeviceHandle hRmDev = (NvRmDeviceHandle)handle;
    value = NvRmPmuGetHookAdc(hRmDev);
}
#endif

void NvOdmServicesPmuGetVoltage( 
        NvOdmServicesPmuHandle handle,
        NvU32 vddId, 
        NvU32 * pMilliVolts )
{
    NvRmDeviceHandle hRmDev = (NvRmDeviceHandle)handle;
    NvRmPmuGetVoltage(hRmDev, vddId, pMilliVolts);
}

void NvOdmServicesPmuSetVoltage( 
        NvOdmServicesPmuHandle handle,
        NvU32 vddId, 
        NvU32 MilliVolts, 
        NvU32 * pSettleMicroSeconds )
{
    NvRmDeviceHandle hRmDev = (NvRmDeviceHandle)handle;
    NvRmPmuSetVoltage(hRmDev, vddId, MilliVolts, pSettleMicroSeconds);
}

void NvOdmServicesPmuSetSocRailPowerState(
        NvOdmServicesPmuHandle handle,
        NvU32 vddId, 
        NvBool Enable )
{
    NvRmDeviceHandle hRmDev = (NvRmDeviceHandle)handle;
    NvRmPmuSetSocRailPowerState(hRmDev, vddId, Enable);
}

//20100909, jh.ahn@lge.com, for detecting power source in battery checker [START]
NvBool
NvOdmServicesPmuGetAcLineStatus(
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuAcLineStatus* pStatus)
{
    return NvRmPmuGetAcLineStatus(
        (NvRmDeviceHandle)handle,
        (NvRmPmuAcLineStatus *)pStatus);
}
//20100909, jh.ahn@lge.com, for detecting power source in battery checker [END]

NvBool 
NvOdmServicesPmuGetBatteryStatus(
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuBatteryInstance batteryInst,
    NvU8 * pStatus)
{
    return NvRmPmuGetBatteryStatus(
        (NvRmDeviceHandle)handle,
        (NvRmPmuBatteryInstance)batteryInst,
        pStatus);
}

NvBool
NvOdmServicesPmuGetBatteryData(
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuBatteryInstance batteryInst,
    NvOdmServicesPmuBatteryData * pData)
{
    return NvRmPmuGetBatteryData(
        (NvRmDeviceHandle)handle,
        (NvRmPmuBatteryInstance)batteryInst,
        (NvRmPmuBatteryData *)pData);
}

//20100924, jh.ahn@lge.com, For updating battery information totally [START]
#if defined(CONFIG_MACH_STAR)
NvBool
NvOdmServicesPmuUpdateBatteryInfo(
	NvOdmServicesPmuHandle handle,
	NvOdmServicesPmuAcLineStatus * pAcStatus,
	NvU8 * pBatStatus,
	NvOdmServicesPmuBatteryData * pBatData)
{
	return NvRmPmuUpdateBatteryInfo(
			(NvRmDeviceHandle)handle,
			(NvRmPmuAcLineStatus *)pAcStatus,
			pBatStatus,
			(NvRmPmuBatteryData *)pBatData);
}
#endif
//20100924, jh.ahn@lge.com, For updating battery information totally  [END]

void
NvOdmServicesPmuGetBatteryFullLifeTime(
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuBatteryInstance batteryInst,
    NvU32 * pLifeTime)
{
    NvRmPmuGetBatteryFullLifeTime(
        (NvRmDeviceHandle)handle,
        (NvRmPmuBatteryInstance)batteryInst,
        pLifeTime);
}

void
NvOdmServicesPmuGetBatteryChemistry(
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuBatteryInstance batteryInst,
    NvOdmServicesPmuBatteryChemistry * pChemistry)
{
    NvRmPmuGetBatteryChemistry(
        (NvRmDeviceHandle)handle,
        (NvRmPmuBatteryInstance)batteryInst,
        (NvRmPmuBatteryChemistry *)pChemistry);
}

void 
NvOdmServicesPmuSetChargingCurrentLimit( 
    NvOdmServicesPmuHandle handle,
    NvOdmServicesPmuChargingPath ChargingPath,
    NvU32 ChargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType)
{
    NvRmPmuSetChargingCurrentLimit(
        (NvRmDeviceHandle)handle,
        (NvRmPmuChargingPath)ChargingPath,
        ChargingCurrentLimitMa,
        ChargerType);
}

// ----------------------- PWM IMPLEMENTATION ------------

NvOdmServicesPwmHandle NvOdmPwmOpen(void)
{
    NvError e;
    NvOdmServicesPwm *pOdmServices = NULL;

    // Allocate memory for the handle.
    pOdmServices = NvOsAlloc(sizeof(NvOdmServicesPwm));
    if (!pOdmServices)
        return NULL;
    NvOsMemset(pOdmServices, 0, sizeof(NvOdmServicesPwm));

    // Open RM device and RM PWM handles
    NV_CHECK_ERROR_CLEANUP(NvRmOpen(&pOdmServices->hRmDev, 0));
    NV_CHECK_ERROR_CLEANUP(NvRmPwmOpen(
        pOdmServices->hRmDev, &pOdmServices->hPwm));

    return pOdmServices;

fail:
    NvOdmPwmClose(pOdmServices);
    return NULL;
}

void NvOdmPwmClose(NvOdmServicesPwmHandle hOdmPwm)
{
    if (!hOdmPwm)
        return;
    NvRmPwmClose(hOdmPwm->hPwm);
    NvRmClose(hOdmPwm->hRmDev);
    NvOsFree(hOdmPwm);
}

void
NvOdmPwmConfig(NvOdmServicesPwmHandle hOdmPwm,
    NvOdmPwmOutputId OutputId,
    NvOdmPwmMode Mode,            
    NvU32 DutyCycle,
    NvU32 *pRequestedFreqHzOrPeriod,
    NvU32 *pCurrentFreqHzOrPeriod)
{
    NvU32 RequestedFreqHzOrPeriod = 0;

    if (pRequestedFreqHzOrPeriod == NULL)
        RequestedFreqHzOrPeriod = NvRmFreqMaximum;
    else
        RequestedFreqHzOrPeriod = *pRequestedFreqHzOrPeriod;

    NvRmPwmConfig(hOdmPwm->hPwm,
        (NvRmPwmOutputId)OutputId, 
        (NvRmPwmMode)Mode,
        DutyCycle,
        RequestedFreqHzOrPeriod,
        pCurrentFreqHzOrPeriod);
}

void
NvOdmEnableUsbPhyPowerRail(
    NvBool Enable)
{
    NvU32 i;
    NvU32 settle_time_us;
    NvU64 guid = NV_VDD_USB_ODM_ID;
    NvOdmPeripheralConnectivity const *pConnectivity;
    NvRmDeviceHandle hRmDevice;
    /* get the connectivity info */
    pConnectivity = NvOdmPeripheralGetGuid( guid );
    if( !pConnectivity )
    {
        // Do nothing if no power rail info is discovered
        return;
    }

    if (NvRmOpen(&hRmDevice, 0) != NvSuccess)
    {
        return;
    }

    /* enable the power rail */
    if (Enable)
    {
        for( i = 0; i < pConnectivity->NumAddress; i++ )
        {
            if( pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd )
            {
                NvRmPmuVddRailCapabilities cap;

                /* address is the vdd rail id */
                NvRmPmuGetCapabilities(
                    hRmDevice,
                    pConnectivity->AddressList[i].Address, &cap );

                /* set the rail volatage to the recommended */
                NvRmPmuSetVoltage(
                    hRmDevice, pConnectivity->AddressList[i].Address,
                    cap.requestMilliVolts, &settle_time_us );

                /* wait for the rail to settle */
                NvOsWaitUS( settle_time_us );
            }
        }
    }
    else
    {
        for( i = 0; i < pConnectivity->NumAddress; i++ )
        {
            if( pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd )
            {
                /* set the rail volatage to the recommended */
                NvRmPmuSetVoltage(
                    hRmDevice, pConnectivity->AddressList[i].Address,
                    ODM_VOLTAGE_OFF, 0 );
            }
        }
    }


    NvRmClose(hRmDevice);
}


void NvOdmEnableOtgCircuitry(NvBool Enable)
{
    const NvOdmUsbProperty *pProperty = NULL;
    static NvBool s_PowerEnabled = NV_FALSE;

    if ((s_PowerEnabled && Enable) || (!s_PowerEnabled && !Enable))
        return;

    pProperty = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, 0);

    if (pProperty && (!pProperty->UseInternalPhyWakeup) &&
        ((pProperty->UsbMode == NvOdmUsbModeType_Device) ||
         (pProperty->UsbMode == NvOdmUsbModeType_OTG)))
    {
        NvOdmEnableUsbPhyPowerRail(s_PowerEnabled = Enable);
    }
}


NvBool NvOdmUsbIsConnected(void)
{
    NV_ASSERT("Not Supported ");
    return NV_FALSE;
}

NvOdmUsbChargerType NvOdmUsbChargingType(NvU32 Instance)
{
    NV_ASSERT("Not Supported ");
    return NvOdmUsbChargerType_Dummy;
}

NvU32 NvOdmServicesGetKeyValue(
    NvOdmServicesKeyListHandle handle,
    NvU32 Key)
{
    return NvRmGetKeyValue((NvRmDeviceHandle)handle, Key);
}

NvBool NvOdmServicesSetKeyValuePair(
    NvOdmServicesKeyListHandle handle,
    NvU32 Key,
    NvU32 Value)
{
    if (NvRmSetKeyValuePair((NvRmDeviceHandle)handle, Key, Value) != NvSuccess)
    {
        return NV_FALSE;
    }
    return NV_TRUE;
}

NvOdmServicesKeyListHandle
NvOdmServicesKeyListOpen(void)
{
    NvRmDeviceHandle hRm;
    NvError Error;

    Error = NvRmOpen(&hRm, 0);
    if (Error != NvSuccess)
    {
        return NULL;
    }
    return (NvOdmServicesKeyListHandle)hRm;
}

void NvOdmServicesKeyListClose(NvOdmServicesKeyListHandle handle)
{
    NvRmClose((NvRmDeviceHandle)handle);
}

