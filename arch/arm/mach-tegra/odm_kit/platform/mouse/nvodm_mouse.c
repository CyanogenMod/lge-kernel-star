/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#include "nvodm_mouse.h"
#include "nvodm_mouse_int.h"
#include "nvrm_drf.h"

// Module debug: 0=disable, 1=enable
#define NVODM_ENABLE_PRINTF      0

#if NVODM_ENABLE_PRINTF
#define NVODM_PRINTF(x) NvOdmOsDebugPrintf x
#else
#define NVODM_PRINTF(x)
#endif

// wake from mouse disabled for now
#define WAKE_FROM_MOUSE 1

/**
 * streaming data from mouse can be compressed if (uncompressed) packet size is
 * 3 bytes (as it is for all legacy ps2 mice).  Compression works by not sending
 * the first byte of the packet when it hasn't change.
 */

#define ENABLE_COMPRESSION 1

#define ECI_MOUSE_DISABLE_SUPPORTED 1

#define MAX_PAYLOAD_BYTES 32

/**
 * specify the ps2 port where the mouse is connected
 */
#define MOUSE_PS2_PORT_ID_0 NVEC_SUBTYPE_0_AUX_PORT_ID_0
#define MOUSE_PS2_PORT_ID_1 NVEC_SUBTYPE_0_AUX_PORT_ID_1

/** Implementation for the NvOdm Mouse */

NvBool
NvOdmMouseDeviceOpen(
    NvOdmMouseDeviceHandle *hDevice)
{
    NvOdmMouseDevice *hMouseDev = NULL;
    NvBool ret = NV_FALSE;
    NvU32 InstanceId = 0, count = 0, MousePort = 0, i = 0;
#if WAKE_FROM_MOUSE
    NvError err = NvError_Success;
    NvEcRequest Request = {0};
    NvEcResponse Response = {0};
#endif

    // Allocate memory for request type structure
    hMouseDev = (NvOdmMouseDevice *)NvOdmOsAlloc(sizeof(NvOdmMouseDevice));
    if (!hMouseDev)
    {
        ret = NV_FALSE;
        NVODMMOUSE_PRINTF(("NvOdmOsAlloc failed to allocate hMouseDev!!"));
        goto fail_safe;
    }
    NvOdmOsMemset(hMouseDev, 0, sizeof(NvOdmMouseDevice));

    // open channel to the EC
    if ((NvEcOpen(&hMouseDev->hEc, InstanceId)) != NvSuccess)
    {
        ret = NV_FALSE;
        NVODMMOUSE_PRINTF(("NvEcOpen failed !!"));
        goto fail_safe;
    }

    hMouseDev->pRequest = NULL;
    hMouseDev->pResponse = NULL;
    hMouseDev->pEvent = NULL;
    hMouseDev->CompressionEnabled = NV_FALSE;
    hMouseDev->CompressionState = 0x0;

    do
    {
        hMouseDev->ValidMousePorts[count] = INVALID_MOUSE_PORT_ID;
        count++;
    } while (count <= MAX_NUM_MOUSE_PORTS);

    // Allocate memory for request type structure
    hMouseDev->pRequest = NvOdmOsAlloc(sizeof(NvEcRequest));
    if (!hMouseDev->pRequest)
    {
        ret = NV_FALSE;
        NVODMMOUSE_PRINTF(("NvOdmOsAlloc failed to allocate pRequest!!"));
        goto fail_safe;
    }
    NvOdmOsMemset(hMouseDev->pRequest, 0, sizeof(NvEcRequest));

    // Allocate memory for response type structure
    hMouseDev->pResponse = NvOdmOsAlloc(sizeof(NvEcResponse));
    if (!hMouseDev->pResponse)
    {
        ret = NV_FALSE;
        NVODMMOUSE_PRINTF(("NvOdmOsAlloc failed to allocate pResponse!!"));
        goto fail_safe;
    }
    NvOdmOsMemset(hMouseDev->pResponse, 0, sizeof(NvEcResponse));

    // Allocate memory for event type structure
    hMouseDev->pEvent = NvOdmOsAlloc(sizeof(NvEcEvent));
    if (!hMouseDev->pEvent)
    {
        ret = NV_FALSE;
        NVODMMOUSE_PRINTF(("NvOdmOsAlloc failed to allocate pEvent!!"));
        goto fail_safe;
    }
    NvOdmOsMemset(hMouseDev->pEvent, 0, sizeof(NvEcEvent));

    MousePort = MOUSE_PS2_PORT_ID_0;
    count = CMD_MAX_RETRIES + 1; i = 0;
    while (count--)
    {
        // fill up request structure
        Request.PacketType = NvEcPacketType_Request;
        Request.RequestType = NvEcRequestResponseType_AuxDevice;
        Request.RequestSubtype = 
            ((NvEcRequestResponseSubtype) 
             (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,MousePort))) |
            ((NvEcRequestResponseSubtype) 
             NvEcAuxDeviceSubtype_SendCommand);
        Request.NumPayloadBytes = 2;
        Request.Payload[0] = 0xFF; // set the reset command
        Request.Payload[1] = 3;

        // Request to EC
        err = NvEcSendRequest(hMouseDev->hEc, &Request, &Response, sizeof(Request),
                        sizeof(Response));

        if (NvSuccess != err)
        {
            NVODMMOUSE_PRINTF(("NvEcSendRequest failed !!"));
            break;
        }

        // mouse not found
        if (NvEcStatus_Success != Response.Status)
        {
            NVODMMOUSE_PRINTF(("EC response failed !!"));
            if (MousePort != MOUSE_PS2_PORT_ID_1)
            {
                count = CMD_MAX_RETRIES + 1;
                MousePort = MOUSE_PS2_PORT_ID_1;
                continue;
            }
            break;
        }

        if (Response.NumPayloadBytes != 3)
            continue;

        // success
        if (Response.Payload[0] == 0xFA)
        {
            hMouseDev->ValidMousePorts[i] = MousePort;
            if (MousePort != MOUSE_PS2_PORT_ID_1)
            {
                count = CMD_MAX_RETRIES + 1;
                MousePort = MOUSE_PS2_PORT_ID_1;
                i++;
                continue;
            }
            break;
        }
    }

#if WAKE_FROM_MOUSE
    i = 0;
    do
    {
        /* enable mouse as wake up source */
        Request.PacketType = NvEcPacketType_Request;
        Request.RequestType = NvEcRequestResponseType_AuxDevice;
        Request.RequestSubtype = ((NvEcRequestResponseSubtype) 
             (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,hMouseDev->ValidMousePorts[i]))) |
             (NvEcRequestResponseSubtype)
             NvEcAuxDeviceSubtype_ConfigureWake;
        Request.NumPayloadBytes = 2;
        Request.Payload[0] = NVEC_AUX_DEVICE_WAKE_ENABLE_0_ACTION_ENABLE;
        Request.Payload[1] = NVEC_AUX_DEVICE_EVENT_TYPE_0_ANY_EVENT_ENABLE;

        err = NvEcSendRequest(
                    hMouseDev->hEc,
                    &Request,
                    &Response,
                    sizeof(Request),
                    sizeof(Response));
        if (err != NvError_Success)
        {
            ret = NV_FALSE;
            goto fail_safe;
        }

        if (Response.Status != NvEcStatus_Success)
        {
            ret = NV_FALSE;
            goto fail_safe;
        }
    } while (hMouseDev->ValidMousePorts[++i] != INVALID_MOUSE_PORT_ID);
#endif

    *hDevice = (NvOdmMouseDeviceHandle)hMouseDev;
    ret = NV_TRUE;
    return ret;

fail_safe:
    NvOdmMouseDeviceClose((NvOdmMouseDeviceHandle)hMouseDev);
    hMouseDev = NULL;
    return ret;
}

void
NvOdmMouseDeviceClose(
    NvOdmMouseDeviceHandle hDevice)
{
    if (hDevice)
    {
        // close channel to the EC
        NvEcClose(hDevice->hEc);
        hDevice->hEc = NULL;
        // Free the request/response structure objects
        NvOdmOsFree(hDevice->pRequest);
        hDevice->pRequest = NULL;
        NvOdmOsFree(hDevice->pResponse);
        hDevice->pResponse = NULL;
        NvOdmOsFree(hDevice->pEvent);
        hDevice->pEvent = NULL;
        NvOdmOsFree(hDevice);
        hDevice = NULL;
    }
}

NvBool NvOdmMouseEnableInterrupt(
    NvOdmMouseDeviceHandle hDevice, 
    NvOdmOsSemaphoreHandle hInterruptSemaphore)
{
    NvError Status = NvSuccess;
    NvEcEventType EventTypes[] = {
        (NvEcEventType) (NvEcEventType_AuxDevice0 + MOUSE_PS2_PORT_ID_0),
        (NvEcEventType) (NvEcEventType_AuxDevice0 + MOUSE_PS2_PORT_ID_1)
    };

    Status = NvEcRegisterForEvents(
        hDevice->hEc,
        &hDevice->hEcEventRegister,
        (NvOsSemaphoreHandle)hInterruptSemaphore,
        NV_ARRAY_SIZE(EventTypes), // number of EventType's
        EventTypes,                // Auxillary 0 event
        1,                         // One event packet is expected
        // event packet size = packet overhead + size of the mouse sample;
        // max sample size is 4 bytes (for an Intellimouse 5-button mouse)
        NVEC_MIN_EVENT_SIZE+4); 

    if (Status != NvSuccess)
        return NV_FALSE;

    return NV_TRUE;
}

NvBool
NvOdmMouseDisableInterrupt(
    NvOdmMouseDeviceHandle hDevice)
{
    NvError Status = NvSuccess;

    // Un-register the events
    Status = NvEcUnregisterForEvents(hDevice->hEcEventRegister);
    if (Status != NvSuccess)
        return NV_FALSE;

    return NV_TRUE;
}

NvBool NvOdmMouseGetEventInfo(
    NvOdmMouseDeviceHandle hDevice,
    NvU32 *NumPayLoad,
    NvU8 *PayLoadBuf)
{
    NvError Status = NvSuccess;

    // Retrive the event info
    Status = NvEcGetEvent(hDevice->hEcEventRegister,
                          hDevice->pEvent,
                          sizeof(NvEcEvent));

    if (Status != NvSuccess)
        return NV_FALSE;

    /**
     * if compression is enabled, latch the first data byte whenever a full-size
     * packet is received; then insert the latched data whenever a compressed
     * packet is seen.
     */
    if (hDevice->CompressionEnabled && hDevice->pEvent->NumPayloadBytes == 3)
    {
        hDevice->CompressionState = hDevice->pEvent->Payload[0];
    }
    
    /**
     * fill in the payload and number of bytes
     */
    if (hDevice->CompressionEnabled && hDevice->pEvent->NumPayloadBytes == 2)
    {
        // compressed packet, so insert latched data at beginning
        *NumPayLoad = 3;
        PayLoadBuf[0] = hDevice->CompressionState;
        PayLoadBuf[1] = hDevice->pEvent->Payload[0];
        PayLoadBuf[2] = hDevice->pEvent->Payload[1];
    }
    else
    {
        *NumPayLoad = hDevice->pEvent->NumPayloadBytes;
        NvOdmOsMemcpy(PayLoadBuf, hDevice->pEvent->Payload, *NumPayLoad);
    }

    return NV_TRUE;
}

NvBool
NvOdmMouseSendRequest(
    NvOdmMouseDeviceHandle hDevice, 
    NvU32 cmd, 
    NvU32 ExpectedResponseSize,
    NvU32 *NumPayLoad, 
    NvU8 *PayLoadBuf)
{
    NvError e;
    NvEcRequest *pRequest = hDevice->pRequest;
    NvEcResponse *pResponse = hDevice->pResponse;
    NvU32 Index = 0;

    do
    {
        // fill up request structure
        pRequest->PacketType = NvEcPacketType_Request;
        pRequest->RequestType = NvEcRequestResponseType_AuxDevice;
        pRequest->RequestSubtype = 
            ((NvEcRequestResponseSubtype) 
             (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,hDevice->ValidMousePorts[Index]))) |
            ((NvEcRequestResponseSubtype) 
             NvEcAuxDeviceSubtype_SendCommand);
        pRequest->NumPayloadBytes = 2;
        pRequest->Payload[0] = cmd; // set the command
        pRequest->Payload[1] = ExpectedResponseSize;

        // Request to EC
        e = NvEcSendRequest(hDevice->hEc, pRequest, pResponse, sizeof(*pRequest),
                        sizeof(*pResponse));
        
        if (NvSuccess != e)
        {
            NVODMMOUSE_PRINTF(("NvEcSendRequest failed !!"));
            return NV_FALSE;
        }

        if (NvEcStatus_Success != pResponse->Status)
        {
            NVODMMOUSE_PRINTF(("EC response failed !!"));
            return NV_FALSE;
        }

        // store/process the Mouse response and return to the client driver
        *NumPayLoad = pResponse->NumPayloadBytes;
        NvOdmOsMemcpy(PayLoadBuf, &pResponse->Payload, *NumPayLoad);
    } while (hDevice->ValidMousePorts[++Index] != INVALID_MOUSE_PORT_ID);

    return NV_TRUE;
}

NvBool
NvOdmMouseStartStreaming(
    NvOdmMouseDeviceHandle hDevice,
    NvU32 NumBytesPerSample)
{
    NvError e;
    NvEcRequest *pRequest = hDevice->pRequest;
    NvEcResponse *pResponse = hDevice->pResponse;
    NvU32 Index = 0;

    if (!hDevice)
        return NV_FALSE;

    hDevice->NumBytesPerSample = NumBytesPerSample;

#if ENABLE_COMPRESSION
    /**
     * automatically enable compression if sample size is 3 bytes
     *
     * compression is supported only for 3-byte data packets (which is the
     * common case for ps/2 mice and mouses
     *
     * compression reduces communication bandwidth by eliminating the first data
     * byte of the packet when it hasn't changed relative to the previous
     * packet.  Whenever a full-sized packet is sent, the first payload byte is
     * latched so that it can be inserted into an n y following compressed packets.
     */

    if (NumBytesPerSample == 3)
    {
        do
        {
            // prepare Aux Device request for Set Compression
            pRequest->PacketType = NvEcPacketType_Request;
            pRequest->RequestType = NvEcRequestResponseType_AuxDevice;
            pRequest->RequestSubtype = 
                ((NvEcRequestResponseSubtype) 
                 (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,hDevice->ValidMousePorts[Index]))) |
                ((NvEcRequestResponseSubtype) 
                 NvEcAuxDeviceSubtype_SetCompression);
            pRequest->NumPayloadBytes = 1; 
            pRequest->Payload[0] = 1; // enable compression

            // send request to EC
            e = NvEcSendRequest(hDevice->hEc, pRequest, pResponse, sizeof(*pRequest),
                                sizeof(*pResponse));

            if (NvSuccess != e)
            {
                NVODMMOUSE_PRINTF(("NvEcSendRequest (compression) failed !!"));
                return NV_FALSE;
            }

            // check status reported by EC
            if (NvEcStatus_Success != pResponse->Status)
            {
                NVODMMOUSE_PRINTF(("EC response (compression) failed !!"));
                return NV_FALSE;
            }
        } while(hDevice->ValidMousePorts[++Index] != INVALID_MOUSE_PORT_ID);

        hDevice->CompressionEnabled = NV_TRUE;
        hDevice->CompressionState = 0x0;
    }
    else
    {
        // compression not supported due to packet size (!= 3 bytes)
        hDevice->CompressionEnabled = NV_FALSE;
    }
#else // ENABLE_COMPRESSION
    // disable compression
    hDevice->CompressionEnabled = NV_FALSE;
#endif // ENABLE_COMPRESSION

    // prepare Aux Device request for Auto-Receive N Bytes
    Index = 0;
    do
    {
        pRequest->PacketType = NvEcPacketType_Request;
        pRequest->RequestType = NvEcRequestResponseType_AuxDevice;
        pRequest->RequestSubtype = 
            ((NvEcRequestResponseSubtype) 
             (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,hDevice->ValidMousePorts[Index]))) |
            ((NvEcRequestResponseSubtype) 
             NvEcAuxDeviceSubtype_AutoReceiveBytes);
        pRequest->NumPayloadBytes = 1; 
        pRequest->Payload[0] = NumBytesPerSample;

        // send request to EC
        e = NvEcSendRequest(hDevice->hEc, pRequest, pResponse, sizeof(*pRequest),
                        sizeof(*pResponse));

        if (NvSuccess != e)
        {
            NVODMMOUSE_PRINTF(("NvEcSendRequest (auto-receive) failed !!"));
            return NV_FALSE;
        }

        // check status reported by EC
        if (NvEcStatus_Success != pResponse->Status)
        {
            NVODMMOUSE_PRINTF(("EC response (auto-receive) failed !!"));
            return NV_FALSE;
        }
    } while(hDevice->ValidMousePorts[++Index] != INVALID_MOUSE_PORT_ID);

    return NV_TRUE;
}

/**
 *  Power suspend for mouse.
 *
 */
NvBool NvOdmMousePowerSuspend(NvOdmMouseDeviceHandle hDevice)
{
#if ECI_MOUSE_DISABLE_SUPPORTED
    NvError e;
    NvEcRequest *pRequest = hDevice->pRequest;
    NvEcResponse *pResponse = hDevice->pResponse;
    NvU32 Index = 0;

    if (!hDevice || !pRequest || !pResponse)
        return NV_FALSE;

    NV_ASSERT(hDevice->hEc);
    NV_ASSERT(hDevice->pRequest);
    NV_ASSERT(hDevice->pResponse);

    // cancel auto-receive (disables event reporting)

    NVODM_PRINTF(("NvOdmMousePowerSuspend: Cancel Auto Receive\n"));

    do
    {
        // fill up request structure
        pRequest->PacketType = NvEcPacketType_Request;
        pRequest->RequestType = NvEcRequestResponseType_AuxDevice;
        pRequest->RequestSubtype = 
            ((NvEcRequestResponseSubtype) 
             (NV_DRF_NUM(NVEC,SUBTYPE,AUX_PORT_ID,hDevice->ValidMousePorts[Index]))) |
            ((NvEcRequestResponseSubtype) 
             NvEcAuxDeviceSubtype_CancelAutoReceive);
        pRequest->NumPayloadBytes = 0;

        // Request to EC
        e = NvEcSendRequest(hDevice->hEc, pRequest, pResponse, sizeof(*pRequest),
                        sizeof(*pResponse));
        
        if (NvSuccess != e)
        {
            NVODMMOUSE_PRINTF(("NvOdmMousePowerSuspend: NvEcSendRequest failed !!"));
            return NV_FALSE;
        }

        if (NvEcStatus_Success != pResponse->Status)
        {
            NVODMMOUSE_PRINTF(("NvOdmMousePowerSuspend: EC response failed !!"));
            return NV_FALSE;
        }
     } while(hDevice->ValidMousePorts[++Index] != INVALID_MOUSE_PORT_ID);
#endif
    NVODM_PRINTF(("NvOdmMousePowerSuspend: Exit success\n"));

    return NV_TRUE;
}

/**
 *  Power resume for mouse.
 *
 */
NvBool NvOdmMousePowerResume(NvOdmMouseDeviceHandle hDevice)
{
#if ECI_MOUSE_DISABLE_SUPPORTED
    if (!hDevice)
        return NV_FALSE;

    NVODM_PRINTF(("NvOdmMousePowerResume: Start Streaming\n"));

    if (!NvOdmMouseStartStreaming(hDevice, hDevice->NumBytesPerSample))
        return NV_FALSE;
#endif
    NVODM_PRINTF(("NvOdmMousePowerResume: Exit success\n"));

    return NV_TRUE;
}

