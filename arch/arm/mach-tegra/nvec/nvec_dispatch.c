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

#define NV_IDL_IS_DISPATCH

#include "nvcommon.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvreftrack.h"
#include "nvidlcmd.h"
#include "nvec.h"

#define OFFSET( s, e ) (NvU32)(void *)(&(((s*)0)->e))


typedef struct NvEcPowerResume_in_t
{
    NvU32 package_;
    NvU32 function_;
} NV_ALIGN(4) NvEcPowerResume_in;

typedef struct NvEcPowerResume_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcPowerResume_inout;

typedef struct NvEcPowerResume_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvEcPowerResume_out;

typedef struct NvEcPowerResume_params_t
{
    NvEcPowerResume_in in;
    NvEcPowerResume_inout inout;
    NvEcPowerResume_out out;
} NvEcPowerResume_params;

typedef struct NvEcPowerSuspend_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcPowerState PowerState;
} NV_ALIGN(4) NvEcPowerSuspend_in;

typedef struct NvEcPowerSuspend_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcPowerSuspend_inout;

typedef struct NvEcPowerSuspend_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvEcPowerSuspend_out;

typedef struct NvEcPowerSuspend_params_t
{
    NvEcPowerSuspend_in in;
    NvEcPowerSuspend_inout inout;
    NvEcPowerSuspend_out out;
} NvEcPowerSuspend_params;

typedef struct NvEcUnregisterForEvents_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcEventRegistrationHandle hEcEventRegistration;
} NV_ALIGN(4) NvEcUnregisterForEvents_in;

typedef struct NvEcUnregisterForEvents_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcUnregisterForEvents_inout;

typedef struct NvEcUnregisterForEvents_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvEcUnregisterForEvents_out;

typedef struct NvEcUnregisterForEvents_params_t
{
    NvEcUnregisterForEvents_in in;
    NvEcUnregisterForEvents_inout inout;
    NvEcUnregisterForEvents_out out;
} NvEcUnregisterForEvents_params;

typedef struct NvEcGetEvent_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcEventRegistrationHandle hEcEventRegistration;
    NvU32 EventSize;
} NV_ALIGN(4) NvEcGetEvent_in;

typedef struct NvEcGetEvent_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcGetEvent_inout;

typedef struct NvEcGetEvent_out_t
{
    NvError ret_;
    NvEcEvent pEvent;
} NV_ALIGN(4) NvEcGetEvent_out;

typedef struct NvEcGetEvent_params_t
{
    NvEcGetEvent_in in;
    NvEcGetEvent_inout inout;
    NvEcGetEvent_out out;
} NvEcGetEvent_params;

typedef struct NvEcRegisterForEvents_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcHandle hEc;
    NvOsSemaphoreHandle hSema;
    NvU32 NumEventTypes;
    NvEcEventType  * pEventTypes;
    NvU32 NumEventPackets;
    NvU32 EventPacketSize;
} NV_ALIGN(4) NvEcRegisterForEvents_in;

typedef struct NvEcRegisterForEvents_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcRegisterForEvents_inout;

typedef struct NvEcRegisterForEvents_out_t
{
    NvError ret_;
    NvEcEventRegistrationHandle phEcEventRegistration;
} NV_ALIGN(4) NvEcRegisterForEvents_out;

typedef struct NvEcRegisterForEvents_params_t
{
    NvEcRegisterForEvents_in in;
    NvEcRegisterForEvents_inout inout;
    NvEcRegisterForEvents_out out;
} NvEcRegisterForEvents_params;

typedef struct NvEcSendRequest_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcHandle hEc;
    NvEcRequest pRequest;
    NvU32 RequestSize;
    NvU32 ResponseSize;
} NV_ALIGN(4) NvEcSendRequest_in;

typedef struct NvEcSendRequest_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcSendRequest_inout;

typedef struct NvEcSendRequest_out_t
{
    NvError ret_;
    NvEcResponse pResponse;
} NV_ALIGN(4) NvEcSendRequest_out;

typedef struct NvEcSendRequest_params_t
{
    NvEcSendRequest_in in;
    NvEcSendRequest_inout inout;
    NvEcSendRequest_out out;
} NvEcSendRequest_params;

typedef struct NvEcClose_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvEcHandle hEc;
} NV_ALIGN(4) NvEcClose_in;

typedef struct NvEcClose_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcClose_inout;

typedef struct NvEcClose_out_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcClose_out;

typedef struct NvEcClose_params_t
{
    NvEcClose_in in;
    NvEcClose_inout inout;
    NvEcClose_out out;
} NvEcClose_params;

typedef struct NvEcOpen_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvU32 InstanceId;
} NV_ALIGN(4) NvEcOpen_in;

typedef struct NvEcOpen_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvEcOpen_inout;

typedef struct NvEcOpen_out_t
{
    NvError ret_;
    NvEcHandle phEc;
} NV_ALIGN(4) NvEcOpen_out;

typedef struct NvEcOpen_params_t
{
    NvEcOpen_in in;
    NvEcOpen_inout inout;
    NvEcOpen_out out;
} NvEcOpen_params;

static NvError NvEcPowerResume_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcPowerResume_out *p_out;
    p_out = (NvEcPowerResume_out *)((NvU8 *)OutBuffer + OFFSET(NvEcPowerResume_params, out) - OFFSET(NvEcPowerResume_params, inout));


    p_out->ret_ = NvEcPowerResume(  );

    return err_;
}

static NvError NvEcPowerSuspend_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcPowerSuspend_in *p_in;
    NvEcPowerSuspend_out *p_out;

    p_in = (NvEcPowerSuspend_in *)InBuffer;
    p_out = (NvEcPowerSuspend_out *)((NvU8 *)OutBuffer + OFFSET(NvEcPowerSuspend_params, out) - OFFSET(NvEcPowerSuspend_params, inout));


    p_out->ret_ = NvEcPowerSuspend( p_in->PowerState );

    return err_;
}

static NvError NvEcUnregisterForEvents_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcUnregisterForEvents_in *p_in;
    NvEcUnregisterForEvents_out *p_out;

    p_in = (NvEcUnregisterForEvents_in *)InBuffer;
    p_out = (NvEcUnregisterForEvents_out *)((NvU8 *)OutBuffer + OFFSET(NvEcUnregisterForEvents_params, out) - OFFSET(NvEcUnregisterForEvents_params, inout));

    if (p_in->hEcEventRegistration != NULL) NvRtFreeObjRef(Ctx, NvRtObjType_NvECPackage_NvEcEventRegistrationHandle, p_in->hEcEventRegistration);

    p_out->ret_ = NvEcUnregisterForEvents( p_in->hEcEventRegistration );

    return err_;
}

static NvError NvEcGetEvent_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcGetEvent_in *p_in;
    NvEcGetEvent_out *p_out;

    p_in = (NvEcGetEvent_in *)InBuffer;
    p_out = (NvEcGetEvent_out *)((NvU8 *)OutBuffer + OFFSET(NvEcGetEvent_params, out) - OFFSET(NvEcGetEvent_params, inout));


    p_out->ret_ = NvEcGetEvent( p_in->hEcEventRegistration, &p_out->pEvent, p_in->EventSize );

    return err_;
}

static NvError NvEcRegisterForEvents_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcRegisterForEvents_in *p_in;
    NvEcRegisterForEvents_out *p_out;
    NvRtObjRefHandle ref_phEcEventRegistration = 0;
    NvOsSemaphoreHandle hSema = NULL;
    NvEcEventType *pEventTypes = NULL;

    p_in = (NvEcRegisterForEvents_in *)InBuffer;
    p_out = (NvEcRegisterForEvents_out *)((NvU8 *)OutBuffer + OFFSET(NvEcRegisterForEvents_params, out) - OFFSET(NvEcRegisterForEvents_params, inout));

    if( p_in->hSema )
    {
        err_ = NvOsSemaphoreUnmarshal( p_in->hSema, &hSema );
        if( err_ != NvSuccess )
        {
            err_ = NvError_BadParameter;
            goto clean;
        }
    }
    if( p_in->NumEventTypes && p_in->pEventTypes )
    {
        pEventTypes = (NvEcEventType  *)NvOsAlloc( p_in->NumEventTypes * sizeof( NvEcEventType  ) );
        if( !pEventTypes )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pEventTypes )
        {
            err_ = NvOsCopyIn( pEventTypes, p_in->pEventTypes, p_in->NumEventTypes * sizeof( NvEcEventType  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }
    err_ = NvRtAllocObjRef(Ctx, &ref_phEcEventRegistration);
    if (err_ != NvSuccess)
    {
        goto clean;
    }

    p_out->ret_ = NvEcRegisterForEvents( p_in->hEc, &p_out->phEcEventRegistration, hSema, p_in->NumEventTypes, pEventTypes, p_in->NumEventPackets, p_in->EventPacketSize );

    if ( p_out->ret_ == NvSuccess )
    {
        NvRtStoreObjRef(Ctx, ref_phEcEventRegistration, NvRtObjType_NvECPackage_NvEcEventRegistrationHandle, p_out->phEcEventRegistration);
        ref_phEcEventRegistration = 0;
    }
clean:
    if (ref_phEcEventRegistration) NvRtDiscardObjRef(Ctx, ref_phEcEventRegistration);
    NvOsSemaphoreDestroy( hSema );
    NvOsFree( pEventTypes );
    return err_;
}

static NvError NvEcSendRequest_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcSendRequest_in *p_in;
    NvEcSendRequest_out *p_out;

    p_in = (NvEcSendRequest_in *)InBuffer;
    p_out = (NvEcSendRequest_out *)((NvU8 *)OutBuffer + OFFSET(NvEcSendRequest_params, out) - OFFSET(NvEcSendRequest_params, inout));


    p_out->ret_ = NvEcSendRequest( p_in->hEc, &p_in->pRequest, &p_out->pResponse, p_in->RequestSize, p_in->ResponseSize );

    return err_;
}

static NvError NvEcClose_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcClose_in *p_in;

    p_in = (NvEcClose_in *)InBuffer;

    if (p_in->hEc != NULL) NvRtFreeObjRef(Ctx, NvRtObjType_NvECPackage_NvEcHandle, p_in->hEc);

    NvEcClose( p_in->hEc );

    return err_;
}

static NvError NvEcOpen_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvEcOpen_in *p_in;
    NvEcOpen_out *p_out;
    NvRtObjRefHandle ref_phEc = 0;

    p_in = (NvEcOpen_in *)InBuffer;
    p_out = (NvEcOpen_out *)((NvU8 *)OutBuffer + OFFSET(NvEcOpen_params, out) - OFFSET(NvEcOpen_params, inout));

    err_ = NvRtAllocObjRef(Ctx, &ref_phEc);
    if (err_ != NvSuccess)
    {
        goto clean;
    }

    p_out->ret_ = NvEcOpen( &p_out->phEc, p_in->InstanceId );

    if ( p_out->ret_ == NvSuccess )
    {
        NvRtStoreObjRef(Ctx, ref_phEc, NvRtObjType_NvECPackage_NvEcHandle, p_out->phEc);
        ref_phEc = 0;
    }
clean:
    if (ref_phEc) NvRtDiscardObjRef(Ctx, ref_phEc);
    return err_;
}

NvError nvec_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvec_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;

    switch( function ) {
    case 7:
        err_ = NvEcPowerResume_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 6:
        err_ = NvEcPowerSuspend_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 5:
        err_ = NvEcUnregisterForEvents_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 4:
        err_ = NvEcGetEvent_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 3:
        err_ = NvEcRegisterForEvents_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 2:
        err_ = NvEcSendRequest_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 1:
        err_ = NvEcClose_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 0:
        err_ = NvEcOpen_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    default:
        err_ = NvError_BadParameter;
        break;
    }

    return err_;
}
