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

#include "nvassert.h"
#include "nvos.h"
#include "nvrm_gpio.h"
#include "nvrm_interrupt.h"
#include "nvrm_moduleids.h"

struct NvRmGpioInterruptRec 
{
    NvRmDeviceHandle hRm;
    NvRmGpioHandle hGpio;
    NvRmGpioPinHandle hPin;
    NvRmGpioPinMode Mode;
    NvOsInterruptHandler Callback;
    void *arg;
    NvU32 IrqNumber;
    NvOsInterruptHandle NvOsIntHandle;
    NvU32 DebounceTime;
};

static 
void NvRmPrivGpioIsr(void *arg);

NvError 
NvRmGpioInterruptRegister(
    NvRmGpioHandle hGpio,
    NvRmDeviceHandle hRm,
    NvRmGpioPinHandle hPin, 
    NvOsInterruptHandler Callback,
    NvRmGpioPinMode Mode,
    void *CallbackArg,
    NvRmGpioInterruptHandle *hGpioInterrupt,
    NvU32 DebounceTime)
{
    /* Get all these from the handle and/or gpio caps API */
    NvError err;
    struct NvRmGpioInterruptRec *h = NULL;
    NvOsInterruptHandler GpioIntHandler = NvRmPrivGpioIsr;

    NV_ASSERT(Mode == NvRmGpioPinMode_InputInterruptLow || 
                Mode == NvRmGpioPinMode_InputInterruptRisingEdge ||
                Mode == NvRmGpioPinMode_InputInterruptFallingEdge || 
                Mode == NvRmGpioPinMode_InputInterruptHigh || 
                Mode == NvRmGpioPinMode_InputInterruptAny);

    /* Allocate memory for the  NvRmGpioInterruptHandle */
    h = (NvRmGpioInterruptHandle)NvOsAlloc(sizeof(struct NvRmGpioInterruptRec));
    if (h == NULL) 
    {
        err = NvError_InsufficientMemory;
        goto fail;
    }

    NvOsMemset(h, 0, sizeof(struct NvRmGpioInterruptRec));

    h->hPin = hPin;
    h->Mode = Mode;
    h->Callback = Callback;
    h->hRm = hRm;
    h->hGpio = hGpio;
    h->arg = CallbackArg;
    h->DebounceTime = DebounceTime;

    err = NvRmGpioConfigPins(hGpio, &hPin, 1, Mode);
    if (err != NvSuccess)
    {
        goto fail;
    }

    if (!h->NvOsIntHandle)
    {
        NvRmGpioGetIrqs(hRm, &hPin, &(h->IrqNumber), 1);

        err = NvRmInterruptRegister(hRm, 1, &h->IrqNumber, &GpioIntHandler, 
                h, &h->NvOsIntHandle, NV_FALSE);

        if (err != NvSuccess)
        {
            NvError e;
            e = NvRmGpioConfigPins(hGpio, &hPin, 1, NvRmGpioPinMode_Inactive);
            NV_ASSERT(!e);
            (void)e;
            goto fail;
        }
    }

    NV_ASSERT(h->NvOsIntHandle);

    *hGpioInterrupt = h;
    return NvSuccess;

fail:
    NvOsFree(h);
    *hGpioInterrupt = 0;
    return err;
}

NvError
NvRmGpioInterruptEnable(NvRmGpioInterruptHandle hGpioInterrupt)
{
    NV_ASSERT(hGpioInterrupt);

    if (!hGpioInterrupt)
    {
        return NvError_BadParameter;
    }

    return NvRmInterruptEnable(hGpioInterrupt->hRm, hGpioInterrupt->NvOsIntHandle);
}

void
NvRmGpioInterruptMask(NvRmGpioInterruptHandle hGpioInterrupt, NvBool mask)
{
    NvOsInterruptMask(hGpioInterrupt->NvOsIntHandle, mask);
    return;
}

static 
void NvRmPrivGpioIsr(void *arg)
{
    NvU32 i = 0;
    NvRmGpioInterruptHandle info = (NvRmGpioInterruptHandle)arg;

    if (info->DebounceTime)
    {
        NvOsSleepMS(info->DebounceTime);
        for (i = 0; i < 100; i++)
            ;
    }
    /* Call the clients callback function */
    (*info->Callback)(info->arg);

    return;
}

void 
NvRmGpioInterruptUnregister(
    NvRmGpioHandle hGpio,
    NvRmDeviceHandle hRm,
    NvRmGpioInterruptHandle handle)
{
    if (handle == NULL)
        return;

    NV_ASSERT(hGpio);
    NV_ASSERT(hRm);

    NV_ASSERT(NvRmGpioConfigPins(hGpio, &handle->hPin, 1, NvRmGpioPinMode_Inactive) 
            == NvSuccess);
    NvRmInterruptUnregister(hRm, handle->NvOsIntHandle);
    handle->NvOsIntHandle = NULL;

    NvOsFree(handle);
    return;
}

void
NvRmGpioInterruptDone( NvRmGpioInterruptHandle handle )
{
    if (!(handle->NvOsIntHandle)) 
    {
        NV_ASSERT(!"Make sure that interrupt source is enabled AFTER the interrupt is succesfully registered.");
    }
    NvRmInterruptDone(handle->NvOsIntHandle);
}

