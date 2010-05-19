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
 * @file nvec_transport.h
 * @brief <b> Nv Embedded Controller (EC) Transport Interface.</b>
 *
 * @b Description: This file declares the interface for the low-level transport
 *    layer that handles communications between the AP and an Embedded
 *    Controller (EC).
 *
 * The EC Interface consist of a two layers, a high layer (NvEc) that manages
 * requests from multiple clients and a lower layer (NvEcTransport) that manages
 * the actual communication between AP and EC.  Multiple implementations of the
 * lower layer are possible, depending on the number of transport mechanisms
 * supported.
 *
 * Usage model is that the high layer allocates a semaphone, which is passed to
 * the transport layer.  The high layer typically invokes nonblocking API's to
 * request the Transport layer perform some operation, then blocks on the
 * semaphore until the operation has been completed.  The transport layer signals
 * the semaphone when the requested operation has been completed.
 *
 * All transport-layer operations share a single semaphore, so there's a
 * nonblocking query call per operation to allow the high layer to determine
 * which transport layer operation has completed.
 *
 * The high layer owns the transmit buffers, whereas the low (transport) layer
 * owns the receive buffers.  Thus, receive operations can begin as soon as the
 * NvEcTransportOpen() routine has finished; but send operations must wait until
 * a buffer is provided by the higher level.  Once a receive operation is 
 * completed, the higher level will query the transport layer for a pointer
 * to the received packet; the transport layer cannot reuse the received packet
 * buffer until the buffer is explicitly released by the higher level.
 *
 * Usage model from the perspective of the higher layer is as follows --
 * 1. initialize transport via NvEcTransportOpen()
 * 2. optionally, register buffer for sending data to the EC via
 *    NvEcTransportAsyncSendPacket()
 * 3. block on hEcNotifySema semaphone until lower level signals that some
 *    operation has been completed
 * 4. invoke NvEcTransportQueryStatus() to determine which operations have
 *    completed (by lower level)
 * 5. if send completed, finish high-level send tasks
 * 6. if receive completed, finish high-level receive tasks as follows --
 *    a. get received packet buffer via NvEcTransportGetReceivePacket()
 *    b. finish high-level processing of received packet
 *    c. release received packet buffer for re-use by lower level, via
 *       NvEcTransportReleaseReceivePacket()
 * 7. go to (2) while there are other operations to perform
 * 8. shut down transport layer via NvEcTransportClose()
 *
 */

#ifndef INCLUDED_NVEC_TRANSPORT_H
#define INCLUDED_NVEC_TRANSPORT_H

#include "nvec.h"
#include "nverror.h"
#include "nvcommon.h"
#include "nvos.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/// forward declarations
typedef struct NvEcTransportRec *NvEcTransportHandle;

/**
 * Transport Status word
 *
 * bit fields for the status word are defined below
 *
 * all bit fields are cleared on read
 */

/// the AP has sent maximum number of NACKs for a packet
#define NVEC_TRANSPORT_STATUS_EVENT_PACKET_MAX_NACK     0x40

/// an error occurred while receiving a event packet (if nonzero)
/// this bit is only meaningful if the EVENT_RECEIVE_COMPLETE bit is set
#define NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_ERROR      0x20

/// an error occurred while receiving a response packet (if nonzero)
/// this bit is only meaningful if the RESPONSERECEIVE_COMPLETE bit is set
#define NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_ERROR      0x10
    
/// an error occurred while sending a packet (if nonzero)
/// this bit is only meaningful if the SEND_COMPLETE bit is set
#define NVEC_TRANSPORT_STATUS_SEND_ERROR         0x8

/// a event packet has been received (if nonzero)
/// see RECEIVE_ERROR bit to determine if an error occurred during receive
#define NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_COMPLETE 0x4

/// a response packet has been received (if nonzero)
/// see RECEIVE_ERROR bit to determine if an error occurred during receive
#define NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_COMPLETE 0x2

/// a packet has been sent (if nonzero)
/// see SEND_ERROR bit to determine if an error occurred during send
#define NVEC_TRANSPORT_STATUS_SEND_COMPLETE      0x1

/**
 * Initialize and open a transport channel to the Embedded Controller (EC). This
 * routine allocates the handle for the EC transport channel and returns it to
 * the caller.
 *
 * @param phEcTrans pointer to location where EC transport channel handles is to
 *        be stored
 * @param hEcNotifySema handle for semaphone to signal when transport-layer
 *        operations have been completed
 * @param InstanceId instance of EC transport channel to be opened
 * @param Flags Extension flags; reserved, must be zero
 *
 * @retval NvSuccess transport channel has been successfully opened.
 * @retval NvError_InsufficientMemory routine was unable to allocate memory.
 * @retval NvError_AlreadyAllocated maximum number of transport channels have 
 *         already been opened
 * @retval NvError_NotSupported InstanceId is invalid
 */

NvError
NvEcTransportOpen(NvEcTransportHandle *phEcTrans,
                  NvU32 InstanceId,
                  NvOsSemaphoreHandle hEcNotifySema, 
                  NvU32 Flags);
    
/**
 * Closes and de-initializes a transport channel to the Embedded Controller
 * (EC).  Also, frees memory allocated for the handle.
 *
 * @param hEc handle for EC transport channel
 *
 * @retval none
 */

void
NvEcTransportClose(NvEcTransportHandle hEcTrans);

/**
 * Query status of transport operations.  This is a nonblocking call.
 *
 * Definition of bit fields within Transport Status word is given above.
 *
 * Note that Send Packet Status will be cleared on read. Receive Packet status
 * will not be clered on read. It will be cleared only after the Receive 
 * Packet data is recevied by client using NvEcTransportGetReceivePacket() API.
 * 
 * @param hEcTrans handle for EC transport channel
 *
 * @retval Transport Status word (defined above)
 */

NvU32
NvEcTransportQueryStatus(
    NvEcTransportHandle hEcTrans);
    
/**
 * Send a request to the EC via the transport channel.  This is a nonblocking
 * call.
 *
 * Once the request has been registered by the transport layer, this routine
 * returns.  Status of the send request can then be queried via the
 * NvEcTransportQueryStatus() routine.  The EC Transport layer will signal the
 * hEcNotifySema when the send has completed.
 *
 * @param hEc handle for EC transport channel
 * @param pRequest pointer to buffer containing EC request
 * @param RequestSize length of EC request buffer, in bytes
 *
 * @retval NvSuccess Request was successfully registered to be sent to EC
 * @retval NvError_Busy Another send operation is already in progress
 * @retval NvError_BadParameter Malformed request packet
 * @retval NvError_InvalidSize Request size is incorrect
 */

NvError
NvEcTransportAsyncSendPacket(
    NvEcTransportHandle hEcTrans,
    NvEcRequest *pRequest,
    NvU32 RequestSize);

/**
 * Aborts the Send Packet operation, which is initiated using.
 * NvEcTransportAsyncSendPacket() API.
 * 
 * @param hEc handle for EC transport channel
 * @param pRequest pointer to buffer containing EC request
 * @param RequestSize length of EC request buffer, in bytes
 *
 */

void
NvEcTransportAbortSendPacket(
    NvEcTransportHandle hEcTrans,
    NvEcRequest *pRequest,
    NvU32 RequestSize);


/**
 * Receive a response or event from the EC via the transport channel.  This is a
 * nonblocking call.
 *
 * When NvEcTransportQueryStatus() reports that a packet has been received,
 * invoke this routine to obtain the received packet.  Once the received packet
 * has been processed, be sure to release the buffer for reuse by the transport
 * layer via NvEcTransportReleaseReceivePacket().
 *
 * The transport layer owns the buffer containing the received packet and just
 * returns a pointer to the buffer to the caller.  Likewise, the transport layer
 * returns a pointer to the buffer size.
 *
 * Note that the received packet can be either a Response Packet or an Event
 * Packet.  In both cases, the packet will be cast to NvEcResponse for return to
 * the caller.  Thereafter, the caller will need to inspect the PacketType field
 * in order to determine whether the received packet is actually a Response or
 * an Event.  Finally, the caller can re-cast to NvEcResponse or NvEcEvent, as
 * appropriate.  See "Packet definitions" section of nvec.h for details on why
 * this type of casting is possible.
 *
 * @param hEcTrans handle for EC transport channel
 * @param pResponse pointer to buffer containing received packet
 * @param ResponseSize length of response buffer, in bytes
 *
 * @retval NvSuccess Response buffer was returned successfully
 * @retval NvError_InvalidState No response is available
 */

NvError
NvEcTransportGetReceivePacket(
    NvEcTransportHandle hEcTrans,
    NvEcResponse *pResponse,
    NvU32 ResponseSize);

/**
 * Suspend the power to EC transport channel.
 *
 * @param hEcTrans handle for EC transport channel
 *
 * @retval NvSuccess.
 */

NvError NvEcTransportPowerSuspend(NvEcTransportHandle hEcTrans);

/**
 * Resume the power to EC transport channel.
 *
 * @param hEcTrans handle for EC transport channel
 *
 * @retval NvSuccess.
 */

NvError NvEcTransportPowerResume(NvEcTransportHandle hEcTrans);

#if defined(__cplusplus)
}
#endif  /* __cplusplus */

#endif // INCLUDED_NVEC_TRANSPORT_H
