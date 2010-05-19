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

#include "linux/freezer.h"

#include "nvcommon.h"
#include "nvassert.h"
#include "nvec.h"
#include "nvec_transport.h"
#include "nvec_private.h"


#define DEBUG_NVEC 0
#define DISP_MESSAGE(x) do { if (DEBUG_NVEC) { NvOsDebugPrintf x ; } } while (0)

#define ENABLE_TIMEOUT              1
#define ENABLE_FAKE_TIMEOUT_TEST    0
#define ENABLE_POWER_MODES          1

static  NvU32           s_refcount = 0;
static  NvEcPrivState   g_ec = {0};     // init mutex to NULL

/* Wakes up EC */
static DECLARE_WAIT_QUEUE_HEAD(wq_ec);

// forward declarations
static void
NvEcPrivProcessSendRequest( NvEcPrivState *ec );


/*
 * Send commands to EC immediately after first open
 */
static NvError
NvEcPrivInitHook( NvEcHandle hEc )
{
#if ENABLE_POWER_MODES
    NvEcRequest req;
    NvEcResponse resp;
    NvError e;

    DISP_MESSAGE(("NvEcPrivInitHook: Enabling Event Reporting\n"));

    // enable global event reporting

    req.PacketType = NvEcPacketType_Request;
    req.RequestType = NvEcRequestResponseType_Sleep;
    req.RequestSubtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_GlobalConfigureEventReporting);
    req.NumPayloadBytes = 1;
    req.Payload[0] = NVEC_SLEEP_GLOBAL_REPORT_ENABLE_0_ACTION_ENABLE;

    NV_CHECK_ERROR( NvEcSendRequest(hEc, &req, &resp, sizeof(req), sizeof(resp)) );
    
    if ( resp.Status != NvEcStatus_Success )
        return NvError_InvalidState;
#endif // ENABLE_POWER_MODES
    
    DISP_MESSAGE(("NvEcPrivInitHook: Exit success\n"));

    return NvSuccess;
}

/*
 * Send commands to EC just before last close
 */
static NvError
NvEcPrivDeinitHook( NvEcHandle hEc )
{
#if ENABLE_POWER_MODES
    NvEcRequest req;
    NvEcResponse resp;
    NvError e;

    // disable global event reporting

    DISP_MESSAGE(("NvEcPrivDeinitHook: Disabling Event Reporting\n"));

    req.PacketType = NvEcPacketType_Request;
    req.RequestType = NvEcRequestResponseType_Sleep;
    req.RequestSubtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_GlobalConfigureEventReporting);
    req.NumPayloadBytes = 1;
    req.Payload[0] = NVEC_SLEEP_GLOBAL_REPORT_ENABLE_0_ACTION_DISABLE;

    NV_CHECK_ERROR( NvEcSendRequest(hEc, &req, &resp, sizeof(req), sizeof(resp)) );
    
    if ( resp.Status != NvEcStatus_Success )
        return NvError_InvalidState;
#endif // ENABLE_POWER_MODES

    DISP_MESSAGE(("NvEcPrivDeinitHook: Exit success\n"));

    return NvSuccess;
}

/*
 * Send commands to EC just before suspend
 */
static NvError
NvEcPrivPowerSuspendHook(
    NvEcHandle hEc,
    NvEcPowerState PowerState)
{
#if ENABLE_POWER_MODES
    NvEcRequest req;
    NvEcResponse resp;
    NvError e;
    NvEcRequestResponseSubtype Subtype;
    
    DISP_MESSAGE(("NvEcPrivPowerSuspendHook: PowerState = %d\n", (NvU32)PowerState));

    // determine operation type
    switch (PowerState)
    {
        case NvEcPowerState_PowerDown:
            Subtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_ApPowerDown);
            break;
        case NvEcPowerState_Suspend:
            Subtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_ApSuspend);
            break;
        case NvEcPowerState_Restart:
            Subtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_ApRestart);
            break;
        default:
            NV_ASSERT(!"NvEcPrivPowerSuspendHook: unknown power state\n");
            NV_CHECK_ERROR(NvError_BadValue);
            break;
    }
    
    // disable global event reporting

    DISP_MESSAGE(("NvEcPrivPowerSuspendHook: Disable Event Reporting\n"));

    req.PacketType = NvEcPacketType_Request;
    req.RequestType = NvEcRequestResponseType_Sleep;
    req.RequestSubtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_GlobalConfigureEventReporting);
    req.NumPayloadBytes = 1;
    req.Payload[0] = NVEC_SLEEP_GLOBAL_REPORT_ENABLE_0_ACTION_DISABLE;

    NV_CHECK_ERROR( NvEcSendRequest(hEc, &req, &resp, sizeof(req), sizeof(resp)) );
    
    if ( resp.Status != NvEcStatus_Success )
        return NvError_InvalidState;
    
    // instruct EC to go to sleep

    DISP_MESSAGE(("NvEcPrivPowerSuspendHook: Go to sleep\n"));

    req.PacketType = NvEcPacketType_Request;
    req.RequestType = NvEcRequestResponseType_Sleep;
    req.RequestSubtype = Subtype;
    req.NumPayloadBytes = 0;

    NV_CHECK_ERROR( NvEcSendRequest(hEc, &req, &resp, sizeof(req), sizeof(resp)) );
    
    if ( resp.Status != NvEcStatus_Success )
        return NvError_InvalidState;
#endif // ENABLE_POWER_MODES
    
    DISP_MESSAGE(("NvEcPrivPowerSuspendHook: Exit success\n"));

    return NvSuccess;

}

/*
 * Send commands to EC immediately after resume
 */
static NvError
NvEcPrivPowerResumeHook( NvEcHandle hEc )
{
#if ENABLE_POWER_MODES
    NvEcRequest req;
    NvEcResponse resp;
    NvError e;

    // enable global event reporting

    DISP_MESSAGE(("NvEcPrivPowerResumeHook: Enabling Event Reporting\n"));

    req.PacketType = NvEcPacketType_Request;
    req.RequestType = NvEcRequestResponseType_Sleep;
    req.RequestSubtype = ((NvEcRequestResponseSubtype) 
                          NvEcSleepSubtype_GlobalConfigureEventReporting);
    req.NumPayloadBytes = 1;
    req.Payload[0] = NVEC_SLEEP_GLOBAL_REPORT_ENABLE_0_ACTION_ENABLE;

    NV_CHECK_ERROR( NvEcSendRequest(hEc, &req, &resp, sizeof(req), sizeof(resp)) );
    
    if ( resp.Status != NvEcStatus_Success )
        return NvError_InvalidState;
#endif // ENABLE_POWER_MODES
    
    DISP_MESSAGE(("NvEcPrivPowerResumeHook: Exit success\n"));

    return NvSuccess;
}

/*
 * Thread to send no-op commands to EC
 */
static void
NvEcPrivPingThread(void *args)
{
	NvError NvStatus = NvError_Success;
	NvEcRequest req;
	NvEcResponse resp;
	NvEcPrivState *ec = (NvEcPrivState *)args;
	int timeout = 0;

	set_freezable_with_signal();

	for (;;) {
	NvOsSemaphoreWait(ec->hPingSema);
	if (ec->exitPingThread)
		break;

	// send no-op commands
	DISP_MESSAGE(("NvEcPrivPingThread: Sending no-op command\n"));
	req.PacketType = NvEcPacketType_Request;
	req.RequestType = NvEcRequestResponseType_Control;
	req.RequestSubtype = (NvEcRequestResponseSubtype)
		NvEcControlSubtype_NoOperation;
	req.NumPayloadBytes = 0;

	NvStatus = NvEcSendRequest(
			ec->hEc,
			&req,
			&resp,
			sizeof(req),
			sizeof(resp));
	if (NvStatus != NvError_Success)
	DISP_MESSAGE(("NvEcPrivPingThread: no-op command send fail\n"));

	if (resp.Status != NvEcStatus_Success)
	DISP_MESSAGE(("NvEcPrivPingThread: no-op command fail\n"));

	DISP_MESSAGE(("NvEcPrivPingThread: no-op command sent\n"));
	ec->IsEcActive = NV_FALSE;
	}
}

NvError
NvEcOpen(NvEcHandle *phEc,
         NvU32 InstanceId)
{
    NvEc            *hEc = NULL;
    NvU32           i;
    NvEcPrivState   *ec = &g_ec;
    NvOsMutexHandle mutex = NULL;
    NvError         e = NvSuccess;

    NV_ASSERT( phEc );

    if ( NULL == ec->mutex )
    {
        e = NvOsMutexCreate(&mutex);
        if (NvSuccess != e)
            return e;
        if (0 != NvOsAtomicCompareExchange32((NvS32*)&ec->mutex, 0,
                                                        (NvS32)mutex) )
            NvOsMutexDestroy( mutex );
    }

    NvOsMutexLock(ec->mutex);

    if ( !s_refcount )
    {
        mutex = ec->mutex;
        NvOsMemset( ec, 0, sizeof(NvEcPrivState) );
        ec->mutex = mutex;
        
        NV_CHECK_ERROR_CLEANUP( NvOsMutexCreate( &ec->requestMutex ));
        NV_CHECK_ERROR_CLEANUP( NvOsMutexCreate( &ec->responseMutex ));
        NV_CHECK_ERROR_CLEANUP( NvOsMutexCreate( &ec->eventMutex ));
        
        NV_CHECK_ERROR_CLEANUP( NvOsSemaphoreCreate( &ec->sema, 0));
        NV_CHECK_ERROR_CLEANUP( NvOsSemaphoreCreate( &ec->LowPowerEntrySema, 0));
        NV_CHECK_ERROR_CLEANUP( NvOsSemaphoreCreate( &ec->LowPowerExitSema, 0));
        
        NV_CHECK_ERROR_CLEANUP( NvEcTransportOpen( &ec->transport, InstanceId,
            ec->sema, 0 ) );
    }

    // Set this flag as TRUE to indicate power is enabled
    ec->powerState = NV_TRUE;

    // create private handle for internal communications between NvEc driver
    // and EC
    if ( !s_refcount )
    {
        ec->hEc = NvOsAlloc( sizeof(NvEc) );
        if ( NULL == ec->hEc )
            goto clean;
        
        // reserve the zero tag for internal use by the nvec driver; this ensures
        // that the driver always has a requestor tag available and can therefore
        // always talk to the EC
        ec->tagAllocated[0] = NV_TRUE;
        ec->hEc->ec = ec;
        ec->hEc->tag = 0;

        NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&ec->hPingSema, 0));

        // perform startup operations before mutex is unlocked
        NV_CHECK_ERROR_CLEANUP( NvEcPrivInitHook(ec->hEc) );

        // start thread to send "pings" - no-op commands to keep EC "alive"
        NV_CHECK_ERROR_CLEANUP(NvOsThreadCreate(
            (NvOsThreadFunction)NvEcPrivPingThread, ec, &ec->hPingThread));
    }

    hEc = NvOsAlloc( sizeof(NvEc) );
    if ( NULL == hEc )
        goto clean;

    NvOsMemset(hEc, 0x00, sizeof(NvEc));

    hEc->ec = ec;

    hEc->tag = NVEC_REQUESTOR_TAG_INVALID;
    for ( i = 0; i < NVEC_MAX_REQUESTOR_TAG; i++ )
    {
        if ( !ec->tagAllocated[i] )
        {
            ec->tagAllocated[i] = NV_TRUE;
            hEc->tag = i;
            break;
        }
    }
    if ( NVEC_REQUESTOR_TAG_INVALID == hEc->tag )
        goto clean;      // run out of tag, clean it up!

    *phEc = hEc;
    s_refcount++;

    NvOsMutexUnlock( ec->mutex );

    ec->IsEcActive = NV_FALSE;

    return NvSuccess;

clean:
    NvOsFree( hEc );
    NvOsMutexUnlock( ec->mutex );

    return NvError_InsufficientMemory;

fail:
    if (!s_refcount)
    {
        ec->exitPingThread = NV_TRUE;
        if (ec->hPingSema)
            NvOsSemaphoreSignal( ec->hPingSema );
        NvOsThreadJoin( ec->hPingThread );
        NvOsSemaphoreDestroy(ec->hPingSema);
        ec->exitThread = NV_TRUE;
        if (ec->sema)
            NvOsSemaphoreSignal( ec->sema );
        NvOsThreadJoin( ec->thread );
        NvOsFree( ec->hEc );
        if ( ec->transport )
            NvEcTransportClose( ec->transport );
        NvOsMutexDestroy( ec->requestMutex );
        NvOsMutexDestroy( ec->responseMutex );
        NvOsMutexDestroy( ec->eventMutex );
        NvOsSemaphoreDestroy( ec->sema );
        NvOsSemaphoreDestroy( ec->LowPowerEntrySema );
        NvOsSemaphoreDestroy( ec->LowPowerExitSema );
        if ( ec->mutex )
        {
            NvOsMutexUnlock( ec->mutex );
            // Destroying of this mutex here is not safe, if another thread is
            // waiting on this mutex, it can cause issues.  We shold have
            // serialized Init/DeInit calls for creating and destroying this mutex.
            NvOsMutexDestroy( ec->mutex );
            NvOsMemset( ec, 0, sizeof(NvEcPrivState) );
            ec->mutex = NULL;
        }
    }
    return NvError_NotInitialized;
}

void
NvEcClose(NvEcHandle hEc)
{
    NvEcPrivState   *ec;
    NvBool          destroy = NV_FALSE;

    if ( NULL == hEc )
        return;

    NV_ASSERT( s_refcount );

    ec = hEc->ec;
    NvOsMutexLock( ec->mutex );

    // FIXME: handle client still with outstanding event types
    if ( !--s_refcount )
    {
        NvEcPrivDeinitHook(ec->hEc);

        NV_ASSERT( NULL == ec->eventReg[hEc->tag].regBegin &&
                    NULL == ec->eventReg[hEc->tag].regEnd );
        NV_ASSERT( NULL == ec->requestBegin && NULL == ec->requestEnd );
        NV_ASSERT( NULL == ec->responseBegin && NULL == ec->responseEnd );

        ec->exitPingThread = NV_TRUE;
        NvOsSemaphoreSignal( ec->hPingSema );
        NvOsThreadJoin( ec->hPingThread );
        ec->exitThread = NV_TRUE;
        NvOsSemaphoreSignal( ec->sema );
        NvOsThreadJoin( ec->thread );

        NvEcTransportClose( ec->transport );
        NvOsMutexDestroy( ec->requestMutex );
        NvOsMutexDestroy( ec->responseMutex );
        NvOsMutexDestroy( ec->eventMutex );
        NvOsSemaphoreDestroy( ec->sema );
        NvOsSemaphoreDestroy( ec->hPingSema );
        NvOsSemaphoreDestroy( ec->LowPowerEntrySema );
        NvOsSemaphoreDestroy( ec->LowPowerExitSema );
        destroy = NV_TRUE;

        NvOsFree( ec->eventNodes );
        NvOsFree( ec->hEc );
    }

    // Set this flag as FALSE to indicate power is disabled
    ec->powerState = NV_FALSE;

    NV_ASSERT( hEc->tag < NVEC_MAX_REQUESTOR_TAG );
    ec->tagAllocated[hEc->tag] = NV_FALSE;      // to be recycled

    NvOsFree( hEc );
    NvOsMutexUnlock( ec->mutex );

    if ( destroy )
    {
        NvOsMutexDestroy( ec->mutex );
        NvOsMemset( ec, 0, sizeof(NvEcPrivState) );
        ec->mutex = NULL;
    }
}

#if ENABLE_TIMEOUT
static NvU32
NvEcPrivComputeTimeout( NvEcPrivState *ec, int idx )
{
    NvU32   t, to = ec->timeout[idx];

    if ( NV_WAIT_INFINITE == to ) 
        return NV_WAIT_INFINITE;

    t = NVEC_TIME_BASE( ec, idx );
    if ( t > to )
        return 0;
    return (to - t);
}
#endif

/* n = target element, p = previous temp var */
#define NVEC_UNLINK(q, n, p) \
    do \
    { \
        if ( (p) ) \
            (p)->next = (n)->next; \
        else \
            q##Begin = (n)->next; \
        if ( (n) == q##End ) \
            q##End = (p); \
    } while (0)

/* t = temp var, p = previous temp var */
#define NVEC_REMOVE_FROM_Q(q, n, t, p) \
    do \
    { \
        (p) = NULL; \
        (t) = q##Begin; \
        while ( (t) ) \
        { \
            if ( (t) == (n) ) \
            { \
                NVEC_UNLINK(q, (t), (p)); \
                break; \
            } \
            (p) = (t); \
            (t) = (t)->next; \
        } \
    } while (0)


#define NVEC_DEQ(q, n) \
    do \
    { \
        (n) = q##Begin; \
        if ( (n) ) \
        { \
            q##Begin = (n)->next; \
            if ( NULL == q##Begin ) \
                q##End = NULL; \
        } \
    } while (0)

#define NVEC_ENQ(q, n) \
    do \
    { \
        if ( q##End ) \
        { \
            q##End->next = (n); \
            q##End = (n); \
        } \
        else \
        { \
            q##Begin = q##End = (n); \
        } \
    } while(0)



/*
 * Since send request is blocking, only need to check requestBegin:
 *  - returns the requestBegin if asked
 *  - check if requestBegin timeout (will signal back too)
 *  - Update requestBegin's timeout by rebasing to
 *    EcPrivThread-global time (hEc->lastTime).
 *  - reset queue timeout to NV_WAIT_INFINITE
 */
static void
NvEcPrivFindAndDequeueRequest( NvEcPrivState    *ec,
                               NvEcRequestNode  **pRequestNode,
                               NvError          transportStatus,
                               NvBool           skipTimeout )
{
    NvEcRequestNode *t = NULL;
    NvBool          remove = skipTimeout;
                            // always remove node with success if skipTimeout

    NvOsMutexLock( ec->requestMutex );
    NV_ASSERT(ec->requestBegin);
    t = ec->requestBegin;
    DISP_MESSAGE(("\r\nFindDQReq requestBegin=0x%x", ec->requestBegin));
    if ( t )
    {
        if ( pRequestNode )
            *pRequestNode = t;

        if ( skipTimeout )
            t->status = transportStatus;
        else if ( t->timeout <= NVEC_TIMEDIFF_WITH_BASE(ec, NVEC_IDX_REQUEST) )
        {
            NvEcTransportAbortSendPacket( ec->transport, &t->request, t->size );
            t->status = NvError_Timeout;
            remove = NV_TRUE;
            DISP_MESSAGE(("RQ time out Reqnode=0x%x", t));
        }
 
        if ( remove )
        {
            ec->requestBegin = t->next;
            if ( NULL == ec->requestBegin )
                ec->requestEnd = NULL;
            DISP_MESSAGE(("\r\nFindDQReq removed=0x%x, removed->next=0x%x, "
                "ec->requestBegin=0x%x, t->status=0x%x", 
                t, t->next, ec->requestBegin, t->status));
            if ( pRequestNode == NULL )
                NvOsSemaphoreSignal( t->sema );
        }
        else
        {
            t->timeout -= NVEC_TIMEDIFF_WITH_BASE(ec, NVEC_IDX_REQUEST);
                        // update this request timeout with lastTime base
        }

        // update with per-queue timeout and timeoutBase
        ec->timeout[NVEC_IDX_REQUEST] = NV_WAIT_INFINITE;
        ec->timeoutBase[NVEC_IDX_REQUEST] = ec->lastTime;
        DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_REQUEST] is set to=%d",
            ec->timeout[NVEC_IDX_REQUEST]));
    }
    NvOsMutexUnlock( ec->requestMutex );
}

/*
 * Traverse response nodes with these:
 *  - one response matching the tag param (returns the node).  If bypassing
 *    tag checking, use INVALID tag as parameter.
 *  - all responses timeout (will signal back too)
 *  - Update individual responseNode's timeout by rebasing to
 *    EcPrivThread-global time (hEc->lastTime).
 *  - Update shortest timeout value for response queue.
 */
static void
NvEcPrivFindAndDequeueResponse( NvEcPrivState    *ec,
                                NvEcResponse     *response,
                                NvEcResponseNode **pResponseNode )
{
    NvEcResponseNode    *t = NULL, *p = NULL, *temp;
    NvU32               timeout = NV_WAIT_INFINITE;
    NvBool              remove = NV_FALSE, found = NV_FALSE;
    NvBool              SignalSema;
    
    NvOsMutexLock( ec->responseMutex );
    NV_ASSERT(ec->responseBegin);
    DISP_MESSAGE(("\r\nFindDQRes responseBegin=0x%x", ec->responseBegin));
    if ( ec->responseBegin )
    {
        t = ec->responseBegin;
        while( t )
        {
            SignalSema = NV_FALSE;
            /* FIXME: just match tag?  more to match?
             * There may be the cases where spurious response is received from EC.
             * Response should not be removed from the queue until req is complete.
             */
            DISP_MESSAGE(("t->tag=0x%x\n", t->tag));

            if (response)
                DISP_MESSAGE(("response->RequestorTag=0x%x\n", response->RequestorTag));

            if ( response && !found && (t->tag == response->RequestorTag) && 
                 t->requestNode->completed )
            {
                if ( pResponseNode )
                    *pResponseNode = t;
                found = NV_TRUE;
                remove = NV_TRUE;
            }
            else
            {
#if ENABLE_TIMEOUT
                if ( t->timeout <=
                            NVEC_TIMEDIFF_WITH_BASE(ec, NVEC_IDX_RESPONSE) )
                {
                    t->status = NvError_Timeout;
                    SignalSema = NV_TRUE;
                    remove = NV_TRUE;
                    DISP_MESSAGE(("Resp Timeout Respnode=0x%x", t));
                }
                else
                {
                    // This check is needed for spurious response case handling.
                    if (t->timeout != NV_WAIT_INFINITE)
                        t->timeout -=
                                NVEC_TIMEDIFF_WITH_BASE(ec, NVEC_IDX_RESPONSE);
                            // update this response timeout w/ lastTime as base
                }
#endif
            }
            
            if ( remove )
            {
                temp = t;
                NVEC_UNLINK( ec->response, t, p );
                DISP_MESSAGE(("\r\nFindDQRes removed=0x%x, removed->next=0x%x, "
                    "prev=0x%x  ec->responseBegin=0x%x", t, t->next, p, 
                    ec->responseBegin));
                remove = NV_FALSE;
                if (p)
                    t = p->next;
                else
                    t = ec->responseBegin;
                if (SignalSema == NV_TRUE)
                    NvOsSemaphoreSignal( temp->sema );
            }
            else
            {
                if ( timeout > t->timeout )
                    timeout = t->timeout;
                p = t;
                t = t->next;
            }
        }
        
        // update with per-queue timeout and timeoutBase
        ec->timeout[NVEC_IDX_RESPONSE] = timeout;
        ec->timeoutBase[NVEC_IDX_RESPONSE] = ec->lastTime;
        DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_RESPONSE] is set to=%d",
            ec->timeout[NVEC_IDX_RESPONSE]));
    }

    if (found == NV_FALSE)
        NvOsDebugPrintf("\r\n***NVEC:Received Spurious Response from EC.");
    NvOsMutexUnlock( ec->responseMutex );
}

/*
 * Go thru timeout handling for each packet type.
 */
static void
NvEcPrivProcessTimeout( NvEcPrivState *ec )
{
    int     i;

    for ( i = 0; i < NVEC_NUM; i++ )
    {
        if ( NV_WAIT_INFINITE == ec->timeout[i] )
            continue;

        if ( ec->timeout[i] > NVEC_TIMEDIFF_WITH_BASE(ec, i) )
        {
            // Update per-queue timeout value to save from traversing queue
            ec->timeout[i] -= ec->timeDiff;
            DISP_MESSAGE(("\r\nec->timeout[%d] is set to=%d", i, 
                ec->timeout[i]));
            continue;
        }
        else
        {
            // found timeout packet type
            switch( i )
            {
                case NVEC_IDX_REQUEST:
                    NvEcPrivFindAndDequeueRequest( ec, NULL, NvSuccess,
                        NV_FALSE );
                    break;
                case NVEC_IDX_RESPONSE:
                    NvEcPrivFindAndDequeueResponse( ec, NULL, NULL );
                    break;
                case NVEC_IDX_EVENT:
                    // nothing to do for now!
                    break;
            }
        }
    }
}


/* Compute most imminent timeout based on current hEc->lastTime */
#if ENABLE_TIMEOUT
static NvU32
NvEcPrivUpdateActualTimeout( NvEcPrivState *ec )
{
    NvU32       timeout, t;
    int         i;

    timeout = NvEcPrivComputeTimeout( ec, 0 );
    for ( i = 1; i < NVEC_NUM; i++ )
    {
        if ( NV_WAIT_INFINITE != ec->timeout[i] )
        {
            t = NvEcPrivComputeTimeout( ec, i );
            if ( timeout > t )
                timeout = t;
        }
    }

    return timeout;
}
#endif

/* Process receive response and update individual timeout */
static NvError
NvEcPrivProcessReceiveResponse( NvEcPrivState *ec,
                                NvError       transportStatus )
{
    NvError             e = NvSuccess;
    NvEcResponse        packet;
    NvEcResponseNode    *responseNode = NULL;
    NvU32               size;

    // FIXME: remove this extra copying and buffering for optimization
    NV_CHECK_ERROR( NvEcTransportGetReceivePacket( ec->transport,
                                &packet, sizeof(NvEcResponse) ) );
                                // nothing we can do here if error!

    NvEcPrivFindAndDequeueResponse( ec, &packet, &responseNode);
    if ( responseNode )
    {
        size = responseNode->size;
        if ( size > sizeof(NvEcResponse) )
            size = sizeof(NvEcResponse);
        NvOsMemcpy( &responseNode->response, &packet, size );
        responseNode->status = transportStatus;
        NvOsSemaphoreSignal(responseNode->sema);
            // all response stuff should be completed before signalling
    }
    else
        e = NvError_BadParameter;
            // will return Bad Param if can't find waiting response

    return e;
}


/*
 * Process receive (response & event) and update individual timeout.
 *
 * Return NvError_InsufficientMemory due to 2 conditions:
 * - internal event queue (ec->eventNodes) too small.
 * - client did NvEcRegisterForEvents.
 * Skip TransportGetReceivePacket and transport will keep NACK'ing EC in this
 * error case.
 */
static NvError
NvEcPrivProcessReceiveEvent( NvEcPrivState *ec,
                             NvError       transportStatus )
{
    NvError                 e = NvSuccess;
    NvEcEventNode           *eventNode = NULL;
    NvEcEventRegistration   *reg = NULL;
    NvEcEvent               *packet = NULL;
    NvEcEventType           eventType;
    NvU32                   i, tagBitmap;

    NvOsMutexLock( ec->eventMutex );
    if ( ec->eventFreeBegin )
    {
        eventNode = ec->eventFreeBegin;
        packet = &eventNode->event;
    }
    else
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }
    NV_CHECK_ERROR( NvEcTransportGetReceivePacket( ec->transport,
                    (NvEcResponse *)packet, sizeof(NvEcEvent) ) );
                    // nothing we can do here if error!

    eventType = packet->EventType;
    NV_ASSERT( eventType < NvEcEventType_Num );

    e = NvError_InvalidState;  // init to event type never registered
    i = 0;
    tagBitmap = ec->eventTagBitmap[eventType];
    while( tagBitmap )
    {
        NV_ASSERT( i < NvEcEventType_Num );
        if ( tagBitmap & 1 )
        {
            reg = ec->eventMap[i][eventType];
            NV_ASSERT( reg );

            if ( NvSuccess != e )
            {
                // dequeue from free and enqueue into ready if not done yet
                ec->eventFreeBegin = eventNode->next;
                if ( ec->eventFreeBegin == NULL )
                    ec->eventFreeEnd = NULL;
                eventNode->timeout = NVEC_EVENT_TIMEOUT_DEFAULT;    // ???
                eventNode->tagBitmap = ec->eventTagBitmap[eventType];
                eventNode->next = NULL;
                NVEC_ENQ( ec->eventReady, eventNode );
                e = NvSuccess;
            }
            NvOsSemaphoreSignal( reg->sema );
        }
        i++;
        tagBitmap = tagBitmap >> 1;
    }

fail:
    NvOsMutexUnlock( ec->eventMutex );
    return e;
}

static void
NvEcPrivProcessPostSendRequest( NvEcPrivState *ec,
                                NvError transportStatus )
{
    NvEcRequestNode     *requestNode = NULL;
    NvEcResponseNode    *responseNode;

    NvEcPrivFindAndDequeueRequest(ec, &requestNode, transportStatus, NV_TRUE);

    // update corresponding responseNode timeout
    if ( requestNode )
    {
        requestNode->completed = NV_TRUE;
        responseNode = requestNode->responseNode;
        if ( responseNode )
        {
            NvOsMutexLock( ec->responseMutex );
            NV_ASSERT(ec->responseBegin);
            if ( NV_WAIT_INFINITE == ec->timeout[NVEC_IDX_RESPONSE] )
            {
                // no current pending response on timeout watch.
                // Update response queue timeout.
                responseNode->timeout = NVEC_RESPONSE_TIMEOUT_DEFAULT;
                ec->timeout[NVEC_IDX_RESPONSE] = responseNode->timeout;
                ec->timeoutBase[NVEC_IDX_RESPONSE] = ec->lastTime;
                DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_RESPONSE] is set to=%d", 
                    ec->timeout[NVEC_IDX_RESPONSE]));
            }
            else
            {
                // Update this response timeout with current lastTime as base
                responseNode->timeout = NVEC_RESPONSE_TIMEOUT_DEFAULT +
                                        NVEC_TIME_BASE(ec, NVEC_IDX_RESPONSE);
                                    // wraparound time difference will work too.
            }
            NvOsMutexUnlock( ec->responseMutex );
        }
        NvOsSemaphoreSignal( requestNode->sema );
            // all request stuff should be done before signal
    }
}

void
NvEcPrivProcessSendRequest( NvEcPrivState *ec )
{
    NvEcRequestNode *requestNode;
    
    NvOsMutexLock( ec->requestMutex );
    if ( ec->requestBegin )
    {
        DISP_MESSAGE(("\r\nProcessSendReq requestBegin=0x%x", ec->requestBegin));
        requestNode = ec->requestBegin;
        // Update timeouts
        requestNode->timeout = NVEC_REQUEST_TIMEOUT_DEFAULT;
        ec->timeout[NVEC_IDX_REQUEST] = requestNode->timeout;
        ec->timeoutBase[NVEC_IDX_REQUEST] = ec->lastTime;
        DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_REQUEST] is set to=%d", 
            ec->timeout[NVEC_IDX_REQUEST]));
        if ( NvSuccess != NvEcTransportAsyncSendPacket( ec->transport,
                    &requestNode->request, requestNode->size ) )
        {
            // transport send fail: de-queue, set status and signal
            ec->requestBegin = requestNode->next;
            if ( NULL == ec->requestBegin )
                ec->requestEnd = NULL;
            requestNode->status = NvError_I2cWriteFailed;
            DISP_MESSAGE(("\r\nProcessSendReq Async Send failed"));
            DISP_MESSAGE(("\r\nProcessSendReq requestBegin=0x%x", ec->requestBegin));
            NvOsSemaphoreSignal( requestNode->sema );
        }
    }
    NvOsMutexUnlock( ec->requestMutex );
}

#if ENABLE_TIMEOUT
static void NvEcPrivResetActiveRequestResponseTimeouts(NvEcPrivState    *ec)
{
    NvU8 i;
    NvEcRequestNode     *q = NULL;
    NvEcResponseNode    *r = NULL;
    
    NvOsMutexLock( ec->requestMutex);
    q = ec->requestBegin;
    while ( q )
    {
        // Check if there are active requests.
        if (q->timeout != NV_WAIT_INFINITE)
        {
            q->timeout = NVEC_REQUEST_TIMEOUT_DEFAULT;
            ec->timeout[NVEC_IDX_REQUEST] = q->timeout;
            DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_REQUEST] is reset to=%d", 
                NVEC_REQUEST_TIMEOUT_DEFAULT));
        }
        q = q->next;
    }
    NvOsMutexUnlock( ec->requestMutex );
    NvOsMutexLock( ec->responseMutex );    
    r = ec->responseBegin;
    while ( r )
    {
        // Check if there are active responses.
        if (r->timeout != NV_WAIT_INFINITE)
        {
            r->timeout = NVEC_RESPONSE_TIMEOUT_DEFAULT;
            ec->timeout[NVEC_IDX_RESPONSE] = r->timeout;
            DISP_MESSAGE(("\r\nec->timeout[NVEC_IDX_RESPONSE] is reset to=%d",
                NVEC_RESPONSE_TIMEOUT_DEFAULT));
        }
        r = r->next;
    }
    NvOsMutexUnlock( ec->responseMutex );
    ec->timeDiff = 0;
    ec->lastTime = NvOsGetTimeMS();
    for ( i = 0; i < NVEC_NUM; i++ )
    {
        ec->timeoutBase[i] = ec->lastTime;
    }
}
#endif //ENABLE_TIMEOUT

/*
 * Always use one EcPrivThread-global clock/time for timeout calculations
 */
static void
NvEcPrivThread( void * args )
{
    NvEcPrivState   *ec = (NvEcPrivState *)args;
    NvU32           t, timeout = NV_WAIT_INFINITE;
    NvU32           tStatus = 0;
    NvError         wait = NvSuccess;
    NvError         e;

    while( !ec->exitThread )
    {
    #if ENABLE_TIMEOUT
        if ( timeout )
            wait = NvOsSemaphoreWaitTimeout( ec->sema, timeout );
    #else
        NvOsSemaphoreWait( ec->sema );
        wait = NvSuccess;
    #endif
        
    #if ENABLE_FAKE_TIMEOUT_TEST
        t = ec->lastTime + 0x200;
    #else
        t = NvOsGetTimeMS();
    #endif
        ec->timeDiff = t - ec->lastTime;
        ec->lastTime = t;      // update last timer value
        
        if ( !timeout || (wait == NvError_Timeout) )
        {
            // timeout case
            NvEcPrivProcessTimeout( ec );
        }
        
        // look for any pending packets
        tStatus = NvEcTransportQueryStatus( ec->transport );
        
        e = NvSuccess;

        /* 
         * SEND_COMPLETE event must be processed before RESPONSE_RECEIVE_COMPLETE 
         * event as SEND_COMPLETE event schedules timeout for RESPONSE_RECEIVE event.
         */
        if ( tStatus & (NVEC_TRANSPORT_STATUS_SEND_COMPLETE |
                         NVEC_TRANSPORT_STATUS_SEND_ERROR) )
        {
            NvEcPrivProcessPostSendRequest( ec,
                (tStatus & NVEC_TRANSPORT_STATUS_SEND_COMPLETE) ?
                    NvSuccess : NvError_I2cWriteFailed );
        }

        if ( tStatus & (NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_ERROR |
                    NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_COMPLETE) )
        {
            e = (tStatus & NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_COMPLETE) ?
                    NvSuccess : NvError_I2cReadFailed;
            e = NvEcPrivProcessReceiveResponse( ec, e );
            // return ignored.  Could be spurious response.
        }
        if ( tStatus & (NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_ERROR |
                    NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_COMPLETE) )
        {
            e = (tStatus & NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_COMPLETE) ?
                    NvSuccess : NvError_I2cReadFailed;
            e = NvEcPrivProcessReceiveEvent( ec, e );
                // return ignored.  Could be spurious event.
        }

	if ( tStatus & NVEC_TRANSPORT_STATUS_EVENT_PACKET_MAX_NACK ) {
		// signal the ping thread to send a ping command since max
		// number of nacks have been sent to the EC
		if (ec->hPingSema) {
			NvOsSemaphoreSignal(ec->hPingSema);
		}
	}

        // send request whenever possible 
        if ( (ec->timeout[NVEC_IDX_REQUEST] == NV_WAIT_INFINITE) && 
             (ec->EnterLowPowerState == NV_FALSE) )
            NvEcPrivProcessSendRequest( ec );
    #if ENABLE_TIMEOUT
        timeout = NvEcPrivUpdateActualTimeout( ec ); 
    #endif
        
        if (ec->EnterLowPowerState)
        {
            // This code assumes that EC is already in kept in sleep mode by
            // either shim or top level code. And there will not be any activity
            // going on SM bus.
            if (ec->timeout[NVEC_IDX_REQUEST] != NV_WAIT_INFINITE)
            {
                NvOsDebugPrintf("\r\nNvEc has active requests during suspend. "
                    "It shouldn't have. check it.");
            }
            
            // No active request is pending. Enter into low power state.
            // Signal power suspend API to enter into suspend mode.
            NvOsSemaphoreSignal(ec->LowPowerEntrySema);
            // Wait till power resume API signals resume operation.
            NvOsSemaphoreWait(ec->LowPowerExitSema);
            // Update the timeouts for the active responses, which are scheduled 
            // to receive before system entering into suspend.
#if ENABLE_TIMEOUT
            NvEcPrivResetActiveRequestResponseTimeouts( ec );
            timeout = NvEcPrivUpdateActualTimeout( ec );
#endif // ENABLE_TIMEOUT
        }
    }
}

static void
NvEcPrivThreadCreate(NvEcPrivState *ec)
{
    int     i;

#if ENABLE_FAKE_TIMEOUT_TEST
    ec->lastTime = 0xFFFFFFF0;
#else
    ec->lastTime = NvOsGetTimeMS();
#endif
    ec->timeDiff = NV_WAIT_INFINITE;
    for ( i = 0; i < NVEC_NUM; i++ )
    {
        ec->timeout[i] = NV_WAIT_INFINITE;
        ec->timeoutBase[i] = ec->lastTime;
        DISP_MESSAGE(("\r\nec->timeout[%d] is set to=%d", i, 
            ec->timeout[i]));
    }
    ec->exitThread = NV_FALSE;

    NV_ASSERT_SUCCESS( NvOsThreadCreate( NvEcPrivThread, ec, &ec->thread ) );
    // FIXME: handle thread create failure
}


NvError
NvEcSendRequest(
    NvEcHandle hEc,
    NvEcRequest *pRequest,
    NvEcResponse *pResponse,
    NvU32 RequestSize,
    NvU32 ResponseSize)
{
    NvEcPrivState       *ec;
    NvError             e = NvSuccess;
    NvEcRequestNode     *requestNode = NULL;
    NvEcResponseNode    *responseNode = NULL;
    NvOsSemaphoreHandle requestSema = NULL;
    NvOsSemaphoreHandle responseSema = NULL;
    
    NV_ASSERT( pRequest );
    NV_ASSERT( hEc );
    if ( (RequestSize > sizeof(NvEcRequest)) || 
         (ResponseSize > sizeof(NvEcResponse)) )
        return NvError_InvalidSize;
    
    ec = hEc->ec;
    requestNode = NvOsAlloc(sizeof(NvEcRequestNode));
    if ( NULL == requestNode )
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }
    NV_CHECK_ERROR_CLEANUP( NvOsSemaphoreCreate( &requestSema, 0 ) );
    
    if ( pResponse )
    {
        responseNode = NvOsAlloc(sizeof(NvEcResponseNode));
        if ( NULL == responseNode )
        {
            e = NvError_InsufficientMemory;
            goto fail;
        }
        NV_CHECK_ERROR_CLEANUP( NvOsSemaphoreCreate( &responseSema, 0 ) );
    }

    ec->IsEcActive = NV_TRUE;

    // request end-queue.  Timeout set to infinite until request sent.
    NvOsMemset( requestNode, 0, sizeof(NvEcRequestNode) );
    pRequest->RequestorTag = hEc->tag;      // assigned tag here
    DISP_MESSAGE(("NvEcSendRequest:pRequest->RequestorTag=0x%x\n", pRequest->RequestorTag));
    NvOsMemcpy(&requestNode->request, pRequest, RequestSize);
    requestNode->tag = hEc->tag;
    DISP_MESSAGE(("NvEcSendRequest:requestNode->tag=0x%x\n", requestNode->tag));
    requestNode->sema = requestSema;
    requestNode->timeout = NV_WAIT_INFINITE;
    requestNode->completed = NV_FALSE;
    requestNode->size = RequestSize;
    
    NvOsMutexLock( ec->requestMutex );
    NVEC_ENQ( ec->request, requestNode );
    DISP_MESSAGE(("\r\nSendReq ec->requestBegin=0x%x", ec->requestBegin));
    NvOsMutexUnlock( ec->requestMutex );
    
    // response en-queue.  Timeout set to infinite until request completes.
    if ( pResponse )
    {
        NvOsMemset( responseNode, 0, sizeof(NvEcResponseNode) );
        requestNode->responseNode = responseNode;   // association between
        responseNode->requestNode = requestNode;    //   request & response
        responseNode->sema = responseSema;
        responseNode->timeout = NV_WAIT_INFINITE;
        responseNode->tag = hEc->tag;
        DISP_MESSAGE(("NvEcSendRequest:responseNode->tag=0x%x\n", responseNode->tag));
        responseNode->size = ResponseSize;
        NvOsMutexLock( ec->responseMutex );
        NVEC_ENQ( ec->response, responseNode );
        DISP_MESSAGE(("\r\nSendReq ec->responseBegin=0x%x", ec->responseBegin));
        NvOsMutexUnlock( ec->responseMutex );
    }

    NvOsMutexLock( ec->mutex );
    if ( !ec->thread )
        NvEcPrivThreadCreate( ec );
    NvOsMutexUnlock( ec->mutex );

    // Trigger EcPrivThread
    NvOsSemaphoreSignal( ec->sema );
    DISP_MESSAGE(("\r\nSendReq requestNode=0x%x, requestNode->responseNode=0x%x",
        requestNode, requestNode->responseNode));
    // Wait on Request returns
    NvOsSemaphoreWait( requestSema );
    DISP_MESSAGE(("\r\nSendReq Out of req sema"));

    e = requestNode->status;
    if ( NvSuccess != e )
    {
        NvEcResponseNode    *t = NULL, *p = NULL;

        // de-queue responseNode too !!!!
        NvOsMutexLock( ec->responseMutex );
        NVEC_REMOVE_FROM_Q( ec->response, responseNode, t, p );
        DISP_MESSAGE(("\r\nSendReq responseBegin=0x%x", ec->responseBegin));
        NvOsMutexUnlock( ec->responseMutex );
        goto fail;
    }

    if ( pResponse )
    {
        // Wait on Response returns
        NvOsSemaphoreWait( responseSema );
        DISP_MESSAGE(("\r\nSendReq Out of resp sema"));
        NV_CHECK_ERROR_CLEANUP( responseNode->status );
        NvOsMemcpy(pResponse, &responseNode->response, ResponseSize);
    }
    // if successful, nodes should be de-queue already but not freed yet

fail:
    NvOsSemaphoreDestroy( requestSema );
    NvOsSemaphoreDestroy( responseSema );
    DISP_MESSAGE(("\r\nSendReq Freeing requestNode=0x%x, responseNode=0x%x", 
        requestNode, responseNode));
    NvOsFree( requestNode );
    NvOsFree( responseNode );
    return e;
}


NvError
NvEcRegisterForEvents(
    NvEcHandle hEc,
    NvEcEventRegistrationHandle *phEcEventRegistration,
    NvOsSemaphoreHandle hSema,
    NvU32 NumEventTypes,
    NvEcEventType *pEventTypes,
    NvU32 NumEventPackets,
    NvU32 EventPacketSize)
{
    NvEcPrivState   *ec = hEc->ec;
    NvEcEventRegistration *h = NULL;
    NvOsSemaphoreHandle hSemaClone = NULL;
    NvError e = NvSuccess;
    NvU32   val, i, tag = hEc->tag;
    NvU32   tagMask = (1UL << tag);

    if ( !hSema || !pEventTypes )
        return NvError_BadParameter;

    if ( !NumEventTypes || (NumEventTypes > NvEcEventType_Num) )
        return NvError_InvalidSize;     // FIXME: is this sufficient?

    NV_ASSERT( phEcEventRegistration );

    NvOsMutexLock( ec->mutex );
    if ( !ec->thread )
        NvEcPrivThreadCreate( ec );

    // Allocate common pool of internal event nodes bufferring if not already
    if ( !ec->eventNodes )
    {
        val = NVEC_NUM_EVENT_PACKETS_DEFAULT;
        if ( NumEventPackets > val )
            val = NumEventPackets;
        ec->eventNodes = NvOsAlloc(val * sizeof(NvEcEventNode));
        if ( NULL == ec->eventNodes )
        {
            NvOsMutexUnlock( ec->mutex );
            return NvError_InsufficientMemory;
        }
        NvOsMemset( ec->eventNodes, 0, (val * sizeof(NvEcEventNode)) );
        for( i = 0; i < val - 1; i++ )
            ec->eventNodes[i].next = &ec->eventNodes[i+1];
        ec->eventFreeBegin = ec->eventNodes;
        ec->eventFreeEnd = ec->eventNodes + val - 1;
    }
    NvOsMutexUnlock( ec->mutex );

    NV_CHECK_ERROR( NvOsSemaphoreClone( hSema, &hSemaClone ) );

    NvOsMutexLock( ec->eventMutex );
    // Quick pre-check for for AlreadyAllocated case
    for ( i = 0; i < NumEventTypes; i++ )
    {
        val = pEventTypes[i];
        if ( val >= NvEcEventType_Num )
            e = NvError_BadParameter;
        else if ( ec->eventMap[tag][val] )
            e = NvError_AlreadyAllocated;
        if ( NvSuccess != e )
            goto fail;
    }
    h = NvOsAlloc( sizeof(NvEcEventRegistration));
    if ( NULL == h )
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }

    NvOsMemset( h, 0, sizeof(NvEcEventRegistration) );
    NVEC_ENQ( ec->eventReg[tag].reg, h );

    // Fill up new registration handle
    NV_ASSERT( NvEcEventType_Num <= 32 );   // eventBitmap only works if <= 32
    for ( i = 0; i < NumEventTypes; i++ )
    {
        val = pEventTypes[i];
        h->eventBitmap |= (1 << val);
        ec->eventMap[tag][val] = h;
        ec->eventTagBitmap[val] |= tagMask;
    }
    h->numEventTypes = NumEventTypes;
    h->sema = hSemaClone;
    h->hEc = hEc;

    h->numEventPacketsHint = NumEventPackets;
    h->eventPacketSizeHint = EventPacketSize;       // ignored hints for now

    NvOsMutexUnlock( ec->eventMutex );
    *phEcEventRegistration = h;
    return e;

fail:
    NvOsSemaphoreDestroy( hSemaClone );
    NvOsMutexUnlock( ec->eventMutex );
    NvOsFree( h );
    return e;
}

static void
NvEcPrivRemoveEventFromReady( NvEcPrivState *ec,
                              NvEcEventNode *eventNode )
{
    // dequeue if all clients got this registered event. 
    // FIXME: Should use a bitmap to check off each client.
    if ( ec->eventReadyBegin == eventNode )
        ec->eventReadyBegin = eventNode->next;
    if ( ec->eventReadyEnd == eventNode )
        ec->eventReadyEnd = NULL;
    
    eventNode->next = NULL;
    NVEC_ENQ( ec->eventFree, eventNode );
}

/* We could potentially re-adjust the internal eventNode queue when there
   is no ready queue to base on accumulated NumEventPacket hint. */
NvError
NvEcGetEvent(
    NvEcEventRegistrationHandle hEcEventRegistration,
    NvEcEvent *pEvent,
    NvU32 EventSize)
{
    NvEcHandle      hEc = hEcEventRegistration->hEc;
    NvEcPrivState   *ec = hEc->ec;
    NvError         e = NvError_InvalidState;
    NvEcEventNode   *eventNode, *t;
    NvU32           tagMask = (1UL << hEc->tag);

    // FIXME: Should change these to assert ???
    if ( !pEvent )
        return NvError_InvalidAddress;

    if ( !hEcEventRegistration )
        return NvError_BadParameter;

    if ( EventSize > sizeof(NvEcEvent) )
        EventSize = sizeof(NvEcEvent);
    else if ( EventSize < NVEC_MIN_EVENT_SIZE )
        return NvError_InvalidSize;

    NvOsMutexLock( ec->eventMutex );
 
    eventNode = ec->eventReadyBegin;
    while ( eventNode )
    {
        // pre advance eventNode since current one could be removed
        t = eventNode;
        eventNode = eventNode->next;
        if ( (hEcEventRegistration->eventBitmap &
                            (1UL << t->event.EventType)) &&
                (t->tagBitmap & tagMask) )
        {
            // only return event with matching client tag set
            NvOsMemcpy( pEvent, &(t->event), EventSize );
            e = NvSuccess;
            t->tagBitmap &= ~tagMask;
            // removed if all registered client has GetEvent(t)
            if ( !t->tagBitmap )
            {
                // In case transport has been nack'ing due to eventFree
                // queue is all used up, signal thread to retrieve
                // queued up event in transport.  eventMutex should
                // synchronize NvEcPrivProcessReceiveEvent properly.
                if ( NULL == ec->eventFreeBegin )
                    NvOsSemaphoreSignal( ec->sema );
                NvEcPrivRemoveEventFromReady( ec, t );
            }
            break;
        }
    }
    
    NvOsMutexUnlock( ec->eventMutex );
    return e;
}

NvError
NvEcUnregisterForEvents(
    NvEcEventRegistrationHandle hEcEventRegistration)
{
    NvEcPrivState   *ec;
    NvU32           tag;
    NvU32           tagMask;
    NvError         e = NvSuccess;
    NvEcEventRegistration *p = NULL, *reg = NULL;
    NvEcEventNode   *eventNode, *t;
    NvU32           i;

    if( NULL == hEcEventRegistration )
        return NvError_BadParameter;
    
    ec = hEcEventRegistration->hEc->ec;
    tag = hEcEventRegistration->hEc->tag;
    tagMask = (1UL << tag);

    NvOsMutexLock( ec->eventMutex );
    
    NVEC_REMOVE_FROM_Q( ec->eventReg[tag].reg, hEcEventRegistration, reg, p );
    if ( !reg )
    {
        e = NvError_BadParameter;      // can't find the handle
        goto fail;
    }

    eventNode = ec->eventReadyBegin;
    while ( eventNode )
    {
        // pre advance eventNode since current one could be removed
        t = eventNode;
        eventNode = eventNode->next;
        if ( (reg->eventBitmap & (1UL << t->event.EventType)) &&
                (t->tagBitmap & tagMask) )
        {
            t->tagBitmap &= ~tagMask;
            if ( !t->tagBitmap )
            {
                NvEcPrivRemoveEventFromReady( ec, t );
            }
        }
    }
 
    // remove global references to this registration
    i = 0;
    while( reg->eventBitmap )
    {
        NV_ASSERT(i < NvEcEventType_Num);
        if ( reg->eventBitmap & 1 )
        {
            ec->eventTagBitmap[i] &= ~tagMask;
            ec->eventMap[tag][i] = NULL;
        }
        reg->eventBitmap = reg->eventBitmap >> 1;
        i++;
    }

    NvOsSemaphoreDestroy( reg->sema );
    NvOsFree( hEcEventRegistration );
fail:
    NvOsMutexUnlock( ec->eventMutex );

    return e;
}

NvError NvEcPowerSuspend(
    NvEcPowerState PowerState)
{
    NvError e = NvSuccess;
    NvEcPrivState   *ec = &g_ec;

    NvOsMutexLock(ec->mutex);
    
    // Call transport's power off only if it's in ON state
    if (ec->powerState == NV_TRUE)
    {
        // Perform pre-suspend EC operations
        NV_CHECK_ERROR_CLEANUP( NvEcPrivPowerSuspendHook(ec->hEc, PowerState) );
        // Enter low power state
        ec->EnterLowPowerState = NV_TRUE;
        // Signal priv thread to get ready for power suspend.
        NvOsSemaphoreSignal(ec->sema);
        // Wait till priv thread is ready for power suspend.
        NvOsSemaphoreWait(ec->LowPowerEntrySema);
        e = NvEcTransportPowerSuspend(ec->transport);
        ec->powerState = NV_FALSE;
    }

fail:
    NvOsMutexUnlock(ec->mutex);
    return e;
}

NvError NvEcPowerResume(void)
{
    NvError e = NvSuccess;
    NvEcPrivState   *ec = &g_ec;
    
    NvOsMutexLock(ec->mutex);

    // Call transport's power on if it's OFF state
    if (ec->powerState == NV_FALSE)
    {
        NV_CHECK_ERROR_CLEANUP( NvEcTransportPowerResume(ec->transport) );

        ec->powerState = NV_TRUE;
        ec->EnterLowPowerState = NV_FALSE;
        // Signal priv thread to get out of power suspend.
        NvOsSemaphoreSignal(ec->LowPowerExitSema);
        // Perform post-resume EC operations
        NvEcPrivPowerResumeHook(ec->hEc);
    }

fail:
    NvOsMutexUnlock(ec->mutex);
    return e;
}

