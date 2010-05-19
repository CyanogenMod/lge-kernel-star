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

#ifndef INCLUDED_EC_PRIVATE_H
#define INCLUDED_EC_PRIVATE_H

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Requestor tags
 *
 * The requestor tag is used to pair responses to a corresponding request.  Each
 * request packet contains a requestor tag when sent to the EC.  The EC echoes
 * back the requestor tag in the corresponding response packet.  The EC also
 * echoes back the request type and sub-type.
 *
 * In general, a request and response can be paired if their request type,
 * request sub-type, and requestor tag all match.  If there are multiple such
 * matches, then a response can generally be paired with the oldest matching
 * request.
 *
 * In a simple implementation, only one request may be allowed by open instance
 * of the NvEc interface.  In this case, it's possible to statically allocate
 * unique requestor tag values at NvEcOpen() time, provided the needed number of
 * open instances is less that the maximum allowable requestor tag value.  In
 * this case, pairing responses to request simplifies to matching the requestor
 * tag values.
 *
 * Legal requestor tag values are [0, NVEC_MAX_REQUESTOR_TAG)
 */

enum {NVEC_MAX_REQUESTOR_TAG = 8};
enum {NVEC_REQUESTOR_TAG_INVALID = NVEC_MAX_REQUESTOR_TAG + 1};

/* Timeout values */
enum {NVEC_RESPONSE_TIMEOUT_DEFAULT = 600};     // in msec
enum {NVEC_REQUEST_TIMEOUT_DEFAULT = 600};      // in msec
enum {NVEC_EVENT_TIMEOUT_DEFAULT = 100};        // in msec
enum {NVEC_PING_TIMEOUT = 2000};                // in msec

/* Internal inter-dependent request and response queue structures. */
struct NvEcResponseNodeRec      *NvEcResponseNodePtr;

typedef struct NvEcRequestNodeRec
{
    NvEcRequest                 request;
    NvU32                       size;
    NvOsSemaphoreHandle         sema;
    NvError                     status;
    struct NvEcResponseNodeRec  *responseNode;
    NvU32                       timeout;
    NvU32                       tag;
    NvBool                      completed;
    struct NvEcRequestNodeRec   *next;
} NvEcRequestNode;

typedef struct NvEcResponseNodeRec
{
    NvEcResponse                response;
    NvU32                       size;
    NvOsSemaphoreHandle         sema;
    NvError                     status;
    NvU32                       timeout;
    NvU32                       tag;
    NvEcRequestNode             *requestNode;
    struct NvEcResponseNodeRec  *next;
} NvEcResponseNode;


/* Internal event registration structures */
enum{NVEC_NUM_EVENT_PACKETS_DEFAULT = 8};

typedef struct NvEcEventRegistrationRec
{
    NvEcHandle                  hEc;
    NvOsSemaphoreHandle         sema;
    NvU32                       tag;
    NvU32                       eventBitmap;    // assume eventypes <= 32
    NvU32                       numEventTypes;

    // ignore these hints for now
    NvU32                       numEventPacketsHint;
    NvU32                       eventPacketSizeHint;
    struct NvEcEventRegistrationRec  *next;
} NvEcEventRegistration;

typedef struct NvEcEventNodeRec
{
    NvEcEvent                   event;      // for event pointer from transport
    NvU32                       tagBitmap;
    //NvU32                       size;

    // FIXME: should handle packet sitting too long before client call get?
    NvU32                       timeout;
    struct NvEcEventNodeRec     *next;
} NvEcEventNode; 

typedef struct NvEcPrivEventRegRec
{
    NvEcEventRegistration   *regBegin;
    NvEcEventRegistration   *regEnd;
} NvEcPrivEventReg;

/* Per NvEcOpen structure */
typedef struct NvEcRec
{
    NvU32                   tag;
    struct NvEcPrivStateRec *ec;
} NvEc;

/* Private internal states */
#define NVEC_IDX( type )    ((type) - NvEcPacketType_Request)
#define NVEC_IDX_REQUEST    NVEC_IDX(NvEcPacketType_Request)
#define NVEC_IDX_RESPONSE   NVEC_IDX(NvEcPacketType_Response)
#define NVEC_IDX_EVENT      NVEC_IDX(NvEcPacketType_Event)
#define NVEC_NUM            NvEcPacketType_Num

#define NVEC_EVENTYPE_NUM   NvEcEventType_Num

typedef struct NvEcPrivStateRec
{
    NvOsMutexHandle     mutex;
    NvOsMutexHandle     requestMutex;
    NvOsMutexHandle     responseMutex;
    NvOsMutexHandle     eventMutex;

    NvEcRequestNode     *requestBegin;
    NvEcRequestNode     *requestEnd;
    NvEcResponseNode    *responseBegin;
    NvEcResponseNode    *responseEnd;

    NvEcEventNode       *eventNodes;    // allocated node list

    /* Use fixed array of registration handles per client/tag */
    NvEcPrivEventReg    eventReg[NVEC_MAX_REQUESTOR_TAG];

    /* Use bit-map for indicating which event is registered per tag */
    NvEcEventRegistration *eventMap[NVEC_MAX_REQUESTOR_TAG][NVEC_EVENTYPE_NUM];

    /* event nodes */
    NvEcEventNode       *eventReadyBegin;
    NvEcEventNode       *eventReadyEnd;
    NvEcEventNode       *eventFreeBegin;
    NvEcEventNode       *eventFreeEnd;

    /* keeping track of clients registered for that eventtype */
    NvU32               eventTagBitmap[NVEC_EVENTYPE_NUM];

    NvOsSemaphoreHandle sema;

    NvOsThreadHandle    thread;
    NvBool              exitThread;

    NvU32               lastTime;               // in msec
    NvU32               timeDiff;
    NvU32               timeout[NVEC_NUM];
    NvU32               timeoutBase[NVEC_NUM];

    NvU32               receiveBuf[NVEC_MAX_PAYLOAD_BYTES];

    NvEcTransportHandle transport;
    NvEc                *hEc;
    
    NvBool              tagAllocated[NVEC_MAX_REQUESTOR_TAG];
    /* Indicates EC is enabled or disabled */
    NvBool              powerState;
    NvBool              EnterLowPowerState;
    NvOsSemaphoreHandle LowPowerEntrySema;
    NvOsSemaphoreHandle LowPowerExitSema;

    /* thread handle which sends "pings" to the EC */
    NvOsThreadHandle    hPingThread;
    NvOsSemaphoreHandle hPingSema;
    NvBool              exitPingThread;
    NvBool              IsEcActive;
} NvEcPrivState;


// doing a diff and ignoring the overflow gets you the correct value 
// even at wrararound, e.g. 
// timeoutBase   = 0xFFFFFFFF
// lastTime = 0x00000000
// lastTime - timeoutBase = 1 + overflow flag (ignored in C)
// this would *NOT* work if the counter was not 32 bits.

// timeout macros.
#define NVEC_TIME_BASE( p, idx ) \
            ((p)->lastTime - (p)->timeoutBase[(idx)])

#define NVEC_TIMEDIFF_WITH_BASE( p, idx ) \
            ((p)->timeDiff + NVEC_TIME_BASE((p), (idx)))

#if defined(__cplusplus)
}
#endif

#endif


