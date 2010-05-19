/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#include "nvec_smbus_priv.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvos.h"

NvError
NvEcTransportSmbusPacketToBuffer(
    NvEcRequest *pRequest,
    NvU32 RequestSize,
    NvU8 *pBuffer,
    NvU32 BufferSize,
    NvU32 *pBufferSizeUsed)
{
    NV_ASSERT(pRequest);
    NV_ASSERT(pBuffer);
    NV_ASSERT(pBufferSizeUsed);
    
    // request packet buffer is too small?
    if (RequestSize < NVEC_MIN_REQUEST_SIZE)
        return NvError_InvalidSize;
    
    // bytestream buffer is too small?
    if (BufferSize < NVEC_SMBUS_MIN_REQUEST_SIZE)
        return NvError_InvalidSize;

    // packet type is request?
    if (pRequest->PacketType != NvEcPacketType_Request)
        return NvError_BadParameter;
    
    // request packet buffer is too small to hold entire packet?
    if (NVEC_MIN_REQUEST_SIZE + pRequest->NumPayloadBytes > RequestSize)
        return NvError_InvalidSize;
    
    // SMBus Block Read fields
    // * Byte Count
    pBuffer[0] = NVEC_SMBUS_MIN_REQUEST_BYTE_COUNT + pRequest->NumPayloadBytes;

    // bytestream exceeds SMBus max length?
    if (pBuffer[0] > NVEC_SMBUS_MAX_TRANSFER_SIZE)
        return NvError_InvalidSize;
    
    // bytestream exceeds buffer size?
    if (pBuffer[0] > BufferSize)
        return NvError_InvalidSize;
    
    // * Data Byte 1
    pBuffer[1] = (NvU8)(NV_DRF_DEF(NVEC,COMMAND,PACKET_TYPE,REQUEST_RESPONSE)
        | NV_DRF_NUM(NVEC,COMMAND,REQUESTOR_TAG,pRequest->RequestorTag)
        | NV_DRF_NUM(NVEC,COMMAND,REQUEST_RESPONSE_TYPE,pRequest->RequestType));

    // * Data Byte 2
    pBuffer[2] = pRequest->RequestSubtype;
    
    // * Data Byte 3 - N
    NvOsMemcpy(&pBuffer[NVEC_SMBUS_MIN_REQUEST_SIZE], pRequest->Payload, 
               pRequest->NumPayloadBytes);
    
    // +1 is for the Byte Count field
    *pBufferSizeUsed = pBuffer[0] + 1;
    
    return NvSuccess;
}

    
NvError
NvEcTransportSmbusBufferToPacket(
    NvU8 *pBuffer,
    NvU32 BufferSize,
    NvEcResponse *pResponse,
    NvU32 ResponseSize,
    NvU32 *pResponseSizeUsed)
{
    NvU32 iPayload; // offset in bytestream what Packet Payload data begins
    NvU32 PayloadSize; // effective payload size, in bytes
    NvBool IsEventError; // nonzero if ERROR_FLAG bit is set
    NvEcEvent *pEvent = (NvEcEvent *)pResponse;
    
    NV_ASSERT(pBuffer);
    NV_ASSERT(pResponse);
    NV_ASSERT(pResponseSizeUsed);
    
    // ok to decode Command Code?
    if (BufferSize < 1)
        return NvError_InvalidSize;

    // determine packet type

    if (NVEC_IS_EVENT(pBuffer[0]))
    {
        // decode bytestream into an Event Packet
        pEvent->PacketType = NvEcPacketType_Event;

        // bytestream buffer too small for Event?
        if (BufferSize < NVEC_SMBUS_MIN_EVENT_SIZE)
            return NvError_InvalidSize;
        
        // packet buffer too small for Event?
        if (ResponseSize < NVEC_MIN_EVENT_SIZE)
            return NvError_InvalidSize;
        
        // SMBus Write operation (all have a common first byte)
        // * Command Code
        pEvent->EventType =
            (NvEcEventType)NV_DRF_VAL(NVEC,COMMAND,EVENT_TYPE,pBuffer[0]);
        IsEventError = NVEC_IS_EVENT_ERROR(pBuffer[0]);
        
        // determine SMBus operation
        switch (NV_DRF_VAL(NVEC,COMMAND,EVENT_LENGTH,pBuffer[0]))
        {
        case NVEC_COMMAND_0_EVENT_LENGTH_FIXED_2BYTE:
            // SMBus Write Byte operation
            iPayload = 1;
            pEvent->NumPayloadBytes = 1;
            break;
                
        case NVEC_COMMAND_0_EVENT_LENGTH_FIXED_3BYTE:
            // SMBus Write Word operation
            iPayload = 1;
            pEvent->NumPayloadBytes = 2;
            break;
            
        case NVEC_COMMAND_0_EVENT_LENGTH_VARIABLE:
            // SMBus Block Write operation
            iPayload = 2;
            pEvent->NumPayloadBytes = pBuffer[1];
            break;

        default:
            return NvError_BadParameter;
        }
        
        pEvent->Status = NvEcStatus_Success;
        if (IsEventError)
        {
            // bytestream too small to hold Status field (even though Command
            // Field says that Status is present)?
            if (iPayload < BufferSize)
                return NvError_BadParameter;
 
            pEvent->NumPayloadBytes--;
            pEvent->Status = (NvEcStatus)pBuffer[iPayload];
            iPayload++;
        }
        
        // calculate amount of Payload that can be copied to Event Packet, which is
        // equal to the minimum of the following quantities --
        // * NumPayloadBytes
        // * payload space available in pBuffer
        // * payload space available in pEvent

        PayloadSize = pEvent->NumPayloadBytes;
        
        if (PayloadSize > BufferSize - iPayload)
            PayloadSize = BufferSize - iPayload;
        
        if (PayloadSize > ResponseSize - NVEC_MIN_EVENT_SIZE)
            PayloadSize = ResponseSize - NVEC_MIN_EVENT_SIZE;

        NvOsMemcpy(pEvent->Payload, &pBuffer[iPayload], PayloadSize);

        *pResponseSizeUsed = NVEC_MIN_EVENT_SIZE + PayloadSize;
        
    } 
    else if (NVEC_IS_RESPONSE(pBuffer[0])) 
    {
        // decode bytestream into a Response Packet

        // bytestream buffer too small for Response?
        if (BufferSize < NVEC_SMBUS_MIN_RESPONSE_SIZE)
            return NvError_InvalidSize;

        // packet buffer too small for Response?
        if (ResponseSize < NVEC_MIN_RESPONSE_SIZE)
            return NvError_InvalidSize;
       
        // SMBus Block Write fields
        // * Command Code
        pResponse->PacketType = NvEcPacketType_Response;
        pResponse->ResponseType = (NvEcRequestResponseType)
            NV_DRF_VAL(NVEC,COMMAND,REQUEST_RESPONSE_TYPE,pBuffer[0]);
        pResponse->RequestorTag = 
            NV_DRF_VAL(NVEC,COMMAND,REQUESTOR_TAG,pBuffer[0]);

        // * Byte Count

        // byte count is too small for legal Response bytestream?
        if (pBuffer[1] < NVEC_SMBUS_MIN_RESPONSE_BYTE_COUNT)
            return NvError_InvalidSize;
    
        pResponse->NumPayloadBytes = pBuffer[1] -
            NVEC_SMBUS_MIN_RESPONSE_BYTE_COUNT;
        
        // * Data Byte 1
        pResponse->ResponseSubtype = (NvEcRequestResponseSubtype)pBuffer[2];
        
        // * Data Byte 2
        pResponse->Status = (NvEcStatus)pBuffer[3];
            
        // * Data Byte 3 - N
        
        // calculate amount of Payload that can be copied to Event Packet, which
        // is equal to the minimum of the following quantities --
        // * NumPayloadBytes (derived from pBuffer[1])
        // * payload space available in pBuffer
        // * payload space available in pResponse

        PayloadSize = pResponse->NumPayloadBytes;
        
        if (PayloadSize > BufferSize - NVEC_SMBUS_MIN_RESPONSE_SIZE)
            PayloadSize = BufferSize - NVEC_SMBUS_MIN_RESPONSE_SIZE;
        
        if (PayloadSize > ResponseSize - NVEC_MIN_RESPONSE_SIZE)
            PayloadSize = ResponseSize - NVEC_MIN_RESPONSE_SIZE;

        NvOsMemcpy(pResponse->Payload, &pBuffer[NVEC_SMBUS_MIN_RESPONSE_SIZE],
                   PayloadSize);

        *pResponseSizeUsed = NVEC_MIN_RESPONSE_SIZE + PayloadSize;
     }
    else
    {
        return NvError_BadParameter;
    }
    
    return NvSuccess;
}


