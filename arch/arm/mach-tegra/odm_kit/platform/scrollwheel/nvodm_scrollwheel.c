/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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

#include "nvodm_scrollwheel.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query.h"

#define SCROLL_WHEEL_GUID NV_ODM_GUID('s', 'c', 'r', 'o', 'l', 'w', 'h', 'l')
#define DEBOUNCE_TIME_MS 0

typedef struct NvOdmScrollWheelRec
{
    // Gpio Handle
    NvOdmServicesGpioHandle hGpio;
    // Pin handles to all the 4 Gpio pins
    NvOdmGpioPinHandle hInputPin1;
    NvOdmGpioPinHandle hInputPin2;
    NvOdmGpioPinHandle hSelectPin;
    NvOdmGpioPinHandle hOnOffPin;
    // Stores the key events the client had registered for.
    NvOdmScrollWheelEvent RegisterEvents;
    NvOdmOsSemaphoreHandle EventSema;
    NvOdmServicesGpioIntrHandle IntrHandle[2];
    NvOdmScrollWheelEvent Event;
    NvU32 LastPin1Val;
    NvOdmOsMutexHandle hKeyEventMutex;
    NvOdmOsThreadHandle hDebounceRotThread;
    NvOdmOsSemaphoreHandle hDebounceRotSema;
    NvOdmOsSemaphoreHandle hDummySema;
    volatile NvU32 shutdown;
    volatile NvBool Debouncing;
} NvOdmScrollWheel;

static NvU32
GetReliablePinValue(NvOdmServicesGpioHandle hGpio,
                    NvOdmGpioPinHandle hPin)
{
    NvU32 data = (NvU32)-1;
    const int sampleCount = 10;
    int i = 0;

    while (i < sampleCount)
    {
        NvU32 currData;
        NvOdmGpioGetState(hGpio, hPin, &currData);
        if (currData == data)
        {
            i++;
        }
        else
        {
            data = currData;
            i = 0;
        }
    }

    return data;
}

static void
ScrollWheelDebounceRotThread(void *arg)
{
    NvOdmScrollWheelHandle hOdmScrollWheel = (NvOdmScrollWheelHandle)arg;
    const NvU32 debounceTimeMS = 2;

    while (!hOdmScrollWheel->shutdown)
    {
        //  If a scroll wheel event is detected, wait <debounceTime> milliseconds
        //  and then read the Terminal 1 pin to determine the current level
        NvOdmOsSemaphoreWait(hOdmScrollWheel->hDebounceRotSema);
        // The dummy semaphore never gets signalled so it will always timeout
        NvOdmOsSemaphoreWaitTimeout(hOdmScrollWheel->hDummySema, debounceTimeMS);
        //NvOdmGpioGetState(hOdmScrollWheel->hGpio, hOdmScrollWheel->hInputPin1, &hOdmScrollWheel->LastPin1Val);
        hOdmScrollWheel->LastPin1Val = GetReliablePinValue(hOdmScrollWheel->hGpio, hOdmScrollWheel->hInputPin1);
        NvOdmGpioConfig(hOdmScrollWheel->hGpio,
                        hOdmScrollWheel->hInputPin1,
                        hOdmScrollWheel->LastPin1Val ?
                        NvOdmGpioPinMode_InputInterruptFallingEdge :
                        NvOdmGpioPinMode_InputInterruptRisingEdge);
        hOdmScrollWheel->Debouncing = NV_FALSE;
    }
}

static void RotGpioInterruptHandler(void *arg)
{
    NvOdmScrollWheelHandle hOdmScrollWheel = (NvOdmScrollWheelHandle)arg;
    NvU32 InPinValue2;
    NvOdmScrollWheelEvent Event = NvOdmScrollWheelEvent_None;

    /* if still debouncing, ignore interrupt */
    if (hOdmScrollWheel->Debouncing) 
    {
        NvOdmGpioInterruptDone(hOdmScrollWheel->IntrHandle[1]);
        return;
    }
    NvOdmGpioGetState(hOdmScrollWheel->hGpio, hOdmScrollWheel->hInputPin2, &InPinValue2);
    
    if (InPinValue2 == hOdmScrollWheel->LastPin1Val)
    {
        Event = NvOdmScrollWheelEvent_RotateAntiClockWise;
    }
    else
    {
        Event = NvOdmScrollWheelEvent_RotateClockWise;
    }

    Event &= hOdmScrollWheel->RegisterEvents;
    if (Event)
    {
        NvOdmOsMutexLock(hOdmScrollWheel->hKeyEventMutex);
        hOdmScrollWheel->Event &= ~(NvOdmScrollWheelEvent_RotateClockWise |
                                    NvOdmScrollWheelEvent_RotateAntiClockWise);
        hOdmScrollWheel->Event |= Event;
        NvOdmOsMutexUnlock(hOdmScrollWheel->hKeyEventMutex);
        NvOdmOsSemaphoreSignal(hOdmScrollWheel->EventSema);
    }
    /* start debounce */
    hOdmScrollWheel->Debouncing = NV_TRUE;
    NvOdmOsSemaphoreSignal(hOdmScrollWheel->hDebounceRotSema);

    NvOdmGpioInterruptDone(hOdmScrollWheel->IntrHandle[1]);
}

static void SelectGpioInterruptHandler(void *arg)
{
    NvOdmScrollWheelHandle hOdmScrollWheel = (NvOdmScrollWheelHandle)arg;
    NvU32 CurrSelectPinState;
    NvOdmScrollWheelEvent Event = NvOdmScrollWheelEvent_None;

    NvOdmGpioGetState(hOdmScrollWheel->hGpio, hOdmScrollWheel->hSelectPin, &CurrSelectPinState);
    Event = (CurrSelectPinState) ? NvOdmScrollWheelEvent_Release : NvOdmScrollWheelEvent_Press;
    Event &= hOdmScrollWheel->RegisterEvents;

    if (Event)
    {
        NvOdmOsMutexLock(hOdmScrollWheel->hKeyEventMutex);
        hOdmScrollWheel->Event &= ~(NvOdmScrollWheelEvent_Press | NvOdmScrollWheelEvent_Release);
        hOdmScrollWheel->Event |= Event;
        NvOdmOsMutexUnlock(hOdmScrollWheel->hKeyEventMutex);
        NvOdmOsSemaphoreSignal(hOdmScrollWheel->EventSema);
    }

    NvOdmGpioInterruptDone(hOdmScrollWheel->IntrHandle[0]);
}

NvOdmScrollWheelHandle 
NvOdmScrollWheelOpen( 
    NvOdmOsSemaphoreHandle hNotifySema,
    NvOdmScrollWheelEvent RegisterEvents)
{
    NvOdmScrollWheelHandle hOdmScroll = NULL;
    NvOdmPeripheralConnectivity *pConnectivity;
    NvU32 i;
    NvOdmInterruptHandler RotIntrHandler = (NvOdmInterruptHandler)RotGpioInterruptHandler;
    NvOdmInterruptHandler SelectIntrHandler = (NvOdmInterruptHandler)SelectGpioInterruptHandler;
    NvU32 GpioInstance[4];
    NvU32 GpioPin[4];
    NvU32 GpioIndex;
    
    pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(SCROLL_WHEEL_GUID);

    if (pConnectivity == NULL)
        return NULL;

    // Should be IO class device
    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
        return NULL;

    // Minimum 4 entry for the 4 line of GPIO
    if (pConnectivity->NumAddress < 4)
        return NULL;

    GpioIndex = 0;
    for (i=0; i<pConnectivity->NumAddress; i++)
    {
        if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_Gpio)
        {
            GpioInstance[GpioIndex] = pConnectivity->AddressList[i].Instance;
            GpioPin[GpioIndex++] = pConnectivity->AddressList[i].Address;
        }
    }

    // 4 GPIO entry for the scroll wheel
    if (GpioIndex != 4)
        return NULL;
        
    hOdmScroll  = NvOdmOsAlloc(sizeof(NvOdmScrollWheel));
    
    if(!hOdmScroll)
    {
        return NULL;
    }
     
    NvOdmOsMemset(hOdmScroll, 0, sizeof(NvOdmScrollWheel));

    hOdmScroll->EventSema = hNotifySema;
    hOdmScroll->RegisterEvents = RegisterEvents;
    hOdmScroll->Event = NvOdmScrollWheelEvent_None;

    hOdmScroll->hKeyEventMutex = NvOdmOsMutexCreate();
    hOdmScroll->hDebounceRotSema = NvOdmOsSemaphoreCreate(0);
    hOdmScroll->hDummySema = NvOdmOsSemaphoreCreate(0);
    
    if (!hOdmScroll->hKeyEventMutex ||
        !hOdmScroll->hDebounceRotSema ||
        !hOdmScroll->hDummySema)
    {
        goto ErrorExit;
    }

    // Getting the OdmGpio Handle
    hOdmScroll->hGpio = NvOdmGpioOpen();
    if (!hOdmScroll->hGpio)
    {
        goto ErrorExit;
    }

    hOdmScroll->hDebounceRotThread =
        NvOdmOsThreadCreate((NvOdmOsThreadFunction)ScrollWheelDebounceRotThread,
                            (void*)hOdmScroll);
    if (!hOdmScroll->hDebounceRotThread)
    {
        goto ErrorExit;
    }

    // Acquiring Pin Handles for all the four Gpio Pins
    // First entry is Input GPIO1
    // Second entry should be Input GPIO2
    // Third entry should be Select
    // 4 th entry should be OnOff pin
    hOdmScroll->hInputPin1= NvOdmGpioAcquirePinHandle(hOdmScroll ->hGpio, 
                            GpioInstance[3], GpioPin[3]);

    hOdmScroll->hInputPin2 = NvOdmGpioAcquirePinHandle(hOdmScroll ->hGpio, 
                            GpioInstance[0], GpioPin[0]);

    hOdmScroll->hSelectPin= NvOdmGpioAcquirePinHandle(hOdmScroll ->hGpio, 
                            GpioInstance[2], GpioPin[2]);

    hOdmScroll->hOnOffPin= NvOdmGpioAcquirePinHandle(hOdmScroll ->hGpio, 
                            GpioInstance[1], GpioPin[1]);

    if (!hOdmScroll->hInputPin1 || !hOdmScroll->hInputPin2 ||
        !hOdmScroll->hSelectPin || !hOdmScroll->hOnOffPin)
    {
        goto ErrorExit;
    }

    // Setting the ON/OFF pin to output mode and setting to Low.
    NvOdmGpioConfig(hOdmScroll->hGpio, hOdmScroll->hOnOffPin, NvOdmGpioPinMode_Output);
    NvOdmGpioSetState(hOdmScroll->hGpio, hOdmScroll->hOnOffPin, 0);

    // Configuring the other pins as input
    //    NvOdmGpioConfig(hOdmScroll->hGpio, hOdmScroll->hSelectPin, NvOdmGpioPinMode_InputData);
    NvOdmGpioConfig(hOdmScroll->hGpio, hOdmScroll->hInputPin1, NvOdmGpioPinMode_InputData);
    NvOdmGpioConfig(hOdmScroll->hGpio, hOdmScroll->hInputPin2, NvOdmGpioPinMode_InputData);

    if (NvOdmGpioInterruptRegister(hOdmScroll->hGpio, &hOdmScroll->IntrHandle[0],
        hOdmScroll->hSelectPin, NvOdmGpioPinMode_InputInterruptAny,
        SelectIntrHandler, (void *)(hOdmScroll), DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        goto ErrorExit;
    }

    hOdmScroll->LastPin1Val = GetReliablePinValue(hOdmScroll->hGpio, hOdmScroll->hInputPin1);
    
    if (NvOdmGpioInterruptRegister(hOdmScroll->hGpio, &hOdmScroll->IntrHandle[1],
        hOdmScroll->hInputPin1, hOdmScroll->LastPin1Val ?
            NvOdmGpioPinMode_InputInterruptFallingEdge :
            NvOdmGpioPinMode_InputInterruptRisingEdge,
        RotIntrHandler, (void *)(hOdmScroll), DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        goto ErrorExit;
    }

    if (!hOdmScroll->IntrHandle[0] || !hOdmScroll->IntrHandle[1])
    {
        goto ErrorExit;
    }

    hOdmScroll->Debouncing = NV_FALSE;
    
    return hOdmScroll;

   ErrorExit:
    NvOdmScrollWheelClose(hOdmScroll);
    return NULL;
}

void NvOdmScrollWheelClose(NvOdmScrollWheelHandle hOdmScrollWheel)
{
    if (hOdmScrollWheel)
    {
        hOdmScrollWheel->shutdown = 1;

        if (hOdmScrollWheel->hDebounceRotThread)
        {
            if (hOdmScrollWheel->hDebounceRotSema)                
                NvOdmOsSemaphoreSignal(hOdmScrollWheel->hDebounceRotSema);
            NvOdmOsThreadJoin(hOdmScrollWheel->hDebounceRotThread);
        }
 
        if (hOdmScrollWheel->hGpio)
        {
            if (hOdmScrollWheel->IntrHandle[0])
            {
                NvOdmGpioInterruptUnregister(hOdmScrollWheel->hGpio,
                                            hOdmScrollWheel->hSelectPin,
                                            hOdmScrollWheel->IntrHandle[0]);
            }
            if (hOdmScrollWheel->IntrHandle[1])
            {
                NvOdmGpioInterruptUnregister(hOdmScrollWheel->hGpio,
                                            hOdmScrollWheel->hInputPin1,
                                            hOdmScrollWheel->IntrHandle[1]);
            }
        
            if (hOdmScrollWheel->hOnOffPin)
            {
                NvOdmGpioReleasePinHandle(hOdmScrollWheel->hGpio, hOdmScrollWheel->hOnOffPin);
            }
        
            if (hOdmScrollWheel->hInputPin1)
            {
                NvOdmGpioReleasePinHandle(hOdmScrollWheel->hGpio, hOdmScrollWheel->hInputPin1);
            }
            if (hOdmScrollWheel->hInputPin2)
            {
                NvOdmGpioReleasePinHandle(hOdmScrollWheel->hGpio, hOdmScrollWheel->hInputPin2);
            }
        
            if (hOdmScrollWheel->hSelectPin)
            {
                NvOdmGpioReleasePinHandle(hOdmScrollWheel->hGpio, hOdmScrollWheel->hSelectPin);
            }

            NvOdmGpioClose(hOdmScrollWheel->hGpio);
        }

        if (hOdmScrollWheel->hDummySema)
        {
            NvOdmOsSemaphoreDestroy(hOdmScrollWheel->hDummySema);
        }
        if (hOdmScrollWheel->hDebounceRotSema)
        {
            NvOdmOsSemaphoreDestroy(hOdmScrollWheel->hDebounceRotSema);
        }
        if (hOdmScrollWheel->hKeyEventMutex)
        {
            NvOdmOsMutexDestroy(hOdmScrollWheel->hKeyEventMutex);
        }

        NvOdmOsFree(hOdmScrollWheel);
    }
}

NvOdmScrollWheelEvent NvOdmScrollWheelGetEvent(NvOdmScrollWheelHandle hOdmScrollWheel)
{
    NvOdmScrollWheelEvent Event;
    NvOdmOsMutexLock(hOdmScrollWheel->hKeyEventMutex);
    Event = hOdmScrollWheel->Event;
    hOdmScrollWheel->Event = NvOdmScrollWheelEvent_None;
    NvOdmOsMutexUnlock(hOdmScrollWheel->hKeyEventMutex);
    return Event;
}

