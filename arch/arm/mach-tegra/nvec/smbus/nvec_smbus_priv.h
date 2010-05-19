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

/**
 * @file nvec_transport_smbus_priv.h
 *
 * @brief <b> Private implementation-specific definitions for SMBus
 * implementation of Nv Embedded Controller (EC) Transport Interface.</b>
 *
 * @b Description: This file defines private data structures and functions for
 * the SMBus implementation of the Embedded Controller (EC) Transport Interface.
 */

#ifndef INCLUDED_NVEC_TRANSPORT_SMBUS_PRIV_H
#define INCLUDED_NVEC_TRANSPORT_SMBUS_PRIV_H

#include "nvec_transport.h"
#include "nverror.h"
#include "nvcommon.h"
#include "nvos.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Command field definition
 *
 * PACKET_TYPE identifies the packet as either a request/response or
 * as an event.  Requests and responses are distinguished by context.
 *
 * If the PACKET_TYPE is EVENT, then the type of event can be determined
 * from the EVENT_TYPE field.
 *
 * If the PACKET_TYPE is REQUEST_RESPONSE, then the request/response
 * type can be determined from the REQUEST_RESPONSE_TYPE field.
 */
#define NVEC_COMMAND_0_PACKET_TYPE_RANGE              7:7
#define NVEC_COMMAND_0_PACKET_TYPE_REQUEST_RESPONSE   0x0
#define NVEC_COMMAND_0_PACKET_TYPE_EVENT              0x1

#define NVEC_COMMAND_0_EVENT_LENGTH_RANGE             6:5

#define NVEC_COMMAND_0_EVENT_LENGTH_FIXED_2BYTE       0x0
#define NVEC_COMMAND_0_EVENT_LENGTH_FIXED_3BYTE       0x1
#define NVEC_COMMAND_0_EVENT_LENGTH_VARIABLE          0x2
#define NVEC_COMMAND_0_EVENT_LENGTH_RESERVED          0x3

#define NVEC_COMMAND_0_ERROR_FLAG_RANGE               4:4

#define NVEC_COMMAND_0_EVENT_TYPE_RANGE               3:0

#define NVEC_COMMAND_0_REQUESTOR_TAG_RANGE            6:4
#define NVEC_COMMAND_0_REQUEST_RESPONSE_TYPE_RANGE    3:0

/**
 * Command to be sent by EC to AP in order to process the Request operation.
 */
#define NVEC_READ_REQUEST_COMMAND 0x01

/**
 * Command field macros
 */

#define NVEC_IS_EVENT(Command) \
    ( NV_DRF_VAL(NVEC,COMMAND,PACKET_TYPE,(NvU8)(Command)) == \
      NVEC_COMMAND_0_PACKET_TYPE_EVENT )
      
#define NVEC_IS_REQUEST_RESPONSE(Command) \
    ( NV_DRF_VAL(NVEC,COMMAND,PACKET_TYPE,(NvU8)(Command)) == \
      NVEC_COMMAND_0_PACKET_TYPE_REQUEST_RESPONSE )

#define NVEC_IS_REQUEST NVEC_IS_REQUEST_RESPONSE
#define NVEC_IS_RESPONSE NVEC_IS_REQUEST_RESPONSE

#define NVEC_IS_EVENT_FIXED_LENGTH(Command) \
    ( (NV_DRF_VAL(NVEC,COMMAND,EVENT_LENGTH,(NvU8)(Command)) == \
       NVEC_COMMAND_0_EVENT_LENGTH_FIXED_2BYTE) || \
      (NV_DRF_VAL(NVEC,COMMAND,EVENT_LENGTH,(NvU8)(Command)) == \
       NVEC_COMMAND_0_EVENT_LENGTH_FIXED_3BYTE) )
      
#define NVEC_IS_EVENT_VARIABLE_LENGTH(Command) \
    ( (NV_DRF_VAL(NVEC,COMMAND,EVENT_LENGTH,(NvU8)(Command)) == \
       NVEC_COMMAND_0_EVENT_LENGTH_VARIABLE) || \

#define NVEC_IS_EVENT_ERROR(Command) \
    ( NV_DRF_VAL(NVEC,COMMAND,ERROR_FLAG,(NvU8)(Command)) )

/**
 * SMBus transaction format for Request Packet
 *
 * Request Packets are always sent using the SMBus Block Read operation
 *
 * SMBus byte field   Packet Content
 * ----------------   ----------------------------------------------------------
 * Command Code       Must be 0x1.  This value indicates to that the Block Read
 *                    is directed to the EC interface.
 *
 *                    Note: the Command Code is checked and discarded by the
 *                          lowest-level SMBus transport code (for Block Reads)
 *
 * Byte Count         number of remaining bytes in transfer = NumPayloadBytes+3;
 *                    SMBus spec requires Byte Count be nonzero, which is
 *                    guaranteed by the required presence of the Command and
 *                    SubType data in Data Byte 1 - 2
 *
 * Data Byte 1        PacketType, RequestType, and RequestorTag; see details in
 *                    Command Field Definition section above
 *
 * Data Byte 2        SubType
 *
 * Data Byte 3 - N    Payload
 */

#define NVEC_SMBUS_MIN_REQUEST_SIZE 3
#define NVEC_SMBUS_MIN_REQUEST_BYTE_COUNT 2

/**
 * SMBus transaction format for Response Packet
 *
 * Response Packets are always sent using the SMBus Block Write operation
 *
 * SMBus byte field   Packet Content
 * ----------------   ----------------------------------------------------------
 * Command Code       PacketType, RequestType, and RequestorTag; see details in
 *                    Command Field Definition section above
 *
 * Byte Count         number of remaining bytes in transfer = NumPayloadBytes+1;
 *                    SMBus spec requires Byte Count be nonzero, which is
 *                    guaranteed by the required presence of the SubType amd
 *                    Status data in Data Byte 1 - 2
 *
 * Data Byte 1        SubType
 *
 * Data Byte 2        Status
 *
 * Data Byte 3 - N    Payload
 */

#define NVEC_SMBUS_MIN_RESPONSE_SIZE 4
#define NVEC_SMBUS_MIN_RESPONSE_BYTE_COUNT 2

/**
 * SMBus transaction format for Event Packet
 *
 * Event Packets can be sent using the SMBus Block Write, SMBus Write Byte or
 * SMBus Write Word operation
 *
 * Event Packet sent using the SMBus Block Write operation --
 *
 * SMBus byte field   Packet Content
 * ----------------   ----------------------------------------------------------
 * Command Code       PacketType and EventType; see details in Command Field
 *                    Definition section above
 *
 * Byte Count         number of remaining bytes in transfer = NumPayloadBytes+1;
 *                    SMBus spec requires Byte Count be nonzero
 *
 * Data Byte 1 - N    Payload; Note that if the ERROR_FLAG is set in the Command
 *                    Field, then the first byte of the Payload is interpreted
 *                    as a Status value
 *
 *
 * Event Packet sent using the SMBus Write Byte operation
 *
 * SMBus byte field   Packet Content
 * ----------------   ----------------------------------------------------------
 * Command Code       PacketType, NumPayloadBytes, and EventType; see details in
 *                    Command Field Definition section above
 *
 * Data Byte 1        Payload; Note that if the ERROR_FLAG is set in the Command
 *                    Field, then the Payload is interpreted as a Status value
 *
 *
 * Event Packet sent using the SMBus Write Word operation --
 *
 * SMBus byte field   Packet Content
 * ----------------   ----------------------------------------------------------
 * Command Code       PacketType, NumPayloadBytes, and EventType; see details in
 *                    Command Field Definition section above
 *
 * Data Byte 1 - 2    Payload; Note that if the ERROR_FLAG is set in the Command
 *                    Field, then the first byte of the Payload is interpreted
 *                    as a Status value
 */

#define NVEC_SMBUS_MIN_EVENT_SIZE 2

/**
 * Maximum SMBus transfer size
 */
#define NVEC_SMBUS_MAX_TRANSFER_SIZE 32

/**
 * convert a Request Packet into the equivalent bytestream used by the SMBus
 * transport layer
 *
 * @param pRequest pointer to buffer containing EC request
 * @param RequestSize length of EC request buffer, in bytes
 * @param pBuffer pointer to buffer where SMBus bytestream will be stored
 * @param BufferSize size of pBuffer buffer, in bytes
 * @param pBufferSizeUsed address of NvU32 where bytestream size (in bytes)
 *        is to be stored
 * 
 * @retval NvSuccess Request Packet was converted to a bytestream successfully
 * @retval NvError_BadParameter Invalid Request Packet
 * @retval NvError_InvalidSize Buffer is too small to hold bytestream or
 *         RequestSize is too small to hold complete Request Packet
 * @retval NvError_InvalidAddress Null memory pointer
 */

NvError
NvEcTransportSmbusPacketToBuffer(
    NvEcRequest *pRequest,
    NvU32 RequestSize,
    NvU8 *pBuffer,
    NvU32 BufferSize,
    NvU32 *pBufferSizeUsed);
    
/**
 * convert a SMBus bytestream into a Response Packet or Event Packet
 *
 * Note that the received packet can be either a Response Packet or an Event
 * Packet.  In both cases, the packet will be cast to NvEcResponse for return to
 * the caller.  Thereafter, the caller will need to inspect the PacketType field
 * in order to determine whether the received packet is actually a Response or
 * an Event.  Finally, the caller can re-cast to NvEcResponse or NvEcEvent, as
 * appropriate.  See "Packet definitions" section of nvec.h for details on why
 * this type of casting is possible.
 *
 * @param pBuffer pointer to buffer containing SMBus bytestream
 * @param BufferSize size of pBuffer buffer, in bytes
 * @param pResponse pointer to buffer where Request Packet or Event Packet is to
 *        be stored
 * @param ResponseSize length of pResponse buffer, in bytes
 * @param pBufferSizeUsed address of NvU32 where Request Packet or Event Packet
 *        size (in bytes) is to be stored
 * 
 * @retval NvSuccess Request Packet was converted to a bytestream successfully
 * @retval NvError_BadParameter Invalid Request Packet
 * @retval NvError_InvalidSize Buffer is too small to hold bytestream or
 *         RequestSize is too small to hold complete Request Packet
 * @retval NvError_InvalidAddress Null memory pointer
 */

NvError
NvEcTransportSmbusBufferToPacket(
    NvU8 *pBuffer,
    NvU32 BufferSize,
    NvEcResponse *pResponse,  // FIXME -- could be event, too
    NvU32 ResponseSize,
    NvU32 *pResponseSizeUsed);

#if defined(__cplusplus)
}
#endif  /* __cplusplus */

#endif // INCLUDED_NVEC_TRANSPORT_SMBUS_PRIV_H
