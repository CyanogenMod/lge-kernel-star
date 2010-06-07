
#define NV_IDL_IS_DISPATCH

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

#include "nvcommon.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvreftrack.h"
#include "nvidlcmd.h"
#include "nvddk_aes.h"

#define OFFSET( s, e ) (NvU32)(void *)(&(((s*)0)->e))


typedef struct NvDdkAesDisableCrypto_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
} NV_ALIGN(4) NvDdkAesDisableCrypto_in;

typedef struct NvDdkAesDisableCrypto_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesDisableCrypto_inout;

typedef struct NvDdkAesDisableCrypto_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesDisableCrypto_out;

typedef struct NvDdkAesDisableCrypto_params_t
{
    NvDdkAesDisableCrypto_in in;
    NvDdkAesDisableCrypto_inout inout;
    NvDdkAesDisableCrypto_out out;
} NvDdkAesDisableCrypto_params;

typedef struct NvDdkAesSetAndLockSecureStorageKey_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvDdkAesKeySize KeyLength;
    NvU8  * pSecureStorageKey;
} NV_ALIGN(4) NvDdkAesSetAndLockSecureStorageKey_in;

typedef struct NvDdkAesSetAndLockSecureStorageKey_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesSetAndLockSecureStorageKey_inout;

typedef struct NvDdkAesSetAndLockSecureStorageKey_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesSetAndLockSecureStorageKey_out;

typedef struct NvDdkAesSetAndLockSecureStorageKey_params_t
{
    NvDdkAesSetAndLockSecureStorageKey_in in;
    NvDdkAesSetAndLockSecureStorageKey_inout inout;
    NvDdkAesSetAndLockSecureStorageKey_out out;
} NvDdkAesSetAndLockSecureStorageKey_params;

typedef struct NvDdkAesLockSecureStorageKey_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
} NV_ALIGN(4) NvDdkAesLockSecureStorageKey_in;

typedef struct NvDdkAesLockSecureStorageKey_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesLockSecureStorageKey_inout;

typedef struct NvDdkAesLockSecureStorageKey_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesLockSecureStorageKey_out;

typedef struct NvDdkAesLockSecureStorageKey_params_t
{
    NvDdkAesLockSecureStorageKey_in in;
    NvDdkAesLockSecureStorageKey_inout inout;
    NvDdkAesLockSecureStorageKey_out out;
} NvDdkAesLockSecureStorageKey_params;

typedef struct NvDdkAesClearSecureBootKey_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
} NV_ALIGN(4) NvDdkAesClearSecureBootKey_in;

typedef struct NvDdkAesClearSecureBootKey_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesClearSecureBootKey_inout;

typedef struct NvDdkAesClearSecureBootKey_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesClearSecureBootKey_out;

typedef struct NvDdkAesClearSecureBootKey_params_t
{
    NvDdkAesClearSecureBootKey_in in;
    NvDdkAesClearSecureBootKey_inout inout;
    NvDdkAesClearSecureBootKey_out out;
} NvDdkAesClearSecureBootKey_params;

typedef struct NvDdkAesGetCapabilities_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
} NV_ALIGN(4) NvDdkAesGetCapabilities_in;

typedef struct NvDdkAesGetCapabilities_inout_t
{
    NvDdkAesCapabilities pCapabilities;
} NV_ALIGN(4) NvDdkAesGetCapabilities_inout;

typedef struct NvDdkAesGetCapabilities_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesGetCapabilities_out;

typedef struct NvDdkAesGetCapabilities_params_t
{
    NvDdkAesGetCapabilities_in in;
    NvDdkAesGetCapabilities_inout inout;
    NvDdkAesGetCapabilities_out out;
} NvDdkAesGetCapabilities_params;

typedef struct NvDdkAesProcessBuffer_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvU32 SrcBufferSize;
    NvU32 DestBufferSize;
    NvU8  * pSrcBuffer;
    NvU8  * pDestBuffer;
} NV_ALIGN(4) NvDdkAesProcessBuffer_in;

typedef struct NvDdkAesProcessBuffer_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesProcessBuffer_inout;

typedef struct NvDdkAesProcessBuffer_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesProcessBuffer_out;

typedef struct NvDdkAesProcessBuffer_params_t
{
    NvDdkAesProcessBuffer_in in;
    NvDdkAesProcessBuffer_inout inout;
    NvDdkAesProcessBuffer_out out;
} NvDdkAesProcessBuffer_params;

typedef struct NvDdkAesGetInitialVector_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvU32 VectorSize;
    NvU8  * pInitialVector;
} NV_ALIGN(4) NvDdkAesGetInitialVector_in;

typedef struct NvDdkAesGetInitialVector_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesGetInitialVector_inout;

typedef struct NvDdkAesGetInitialVector_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesGetInitialVector_out;

typedef struct NvDdkAesGetInitialVector_params_t
{
    NvDdkAesGetInitialVector_in in;
    NvDdkAesGetInitialVector_inout inout;
    NvDdkAesGetInitialVector_out out;
} NvDdkAesGetInitialVector_params;

typedef struct NvDdkAesSetInitialVector_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvU8  * pInitialVector;
    NvU32 VectorSize;
} NV_ALIGN(4) NvDdkAesSetInitialVector_in;

typedef struct NvDdkAesSetInitialVector_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesSetInitialVector_inout;

typedef struct NvDdkAesSetInitialVector_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesSetInitialVector_out;

typedef struct NvDdkAesSetInitialVector_params_t
{
    NvDdkAesSetInitialVector_in in;
    NvDdkAesSetInitialVector_inout inout;
    NvDdkAesSetInitialVector_out out;
} NvDdkAesSetInitialVector_params;

typedef struct NvDdkAesSelectOperation_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvDdkAesOperation pOperation;
} NV_ALIGN(4) NvDdkAesSelectOperation_in;

typedef struct NvDdkAesSelectOperation_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesSelectOperation_inout;

typedef struct NvDdkAesSelectOperation_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesSelectOperation_out;

typedef struct NvDdkAesSelectOperation_params_t
{
    NvDdkAesSelectOperation_in in;
    NvDdkAesSelectOperation_inout inout;
    NvDdkAesSelectOperation_out out;
} NvDdkAesSelectOperation_params;

typedef struct NvDdkAesSelectKey_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
    NvDdkAesKeyInfo pKeyInfo;
} NV_ALIGN(4) NvDdkAesSelectKey_in;

typedef struct NvDdkAesSelectKey_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesSelectKey_inout;

typedef struct NvDdkAesSelectKey_out_t
{
    NvError ret_;
} NV_ALIGN(4) NvDdkAesSelectKey_out;

typedef struct NvDdkAesSelectKey_params_t
{
    NvDdkAesSelectKey_in in;
    NvDdkAesSelectKey_inout inout;
    NvDdkAesSelectKey_out out;
} NvDdkAesSelectKey_params;

typedef struct NvDdkAesClose_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvDdkAesHandle hAes;
} NV_ALIGN(4) NvDdkAesClose_in;

typedef struct NvDdkAesClose_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesClose_inout;

typedef struct NvDdkAesClose_out_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesClose_out;

typedef struct NvDdkAesClose_params_t
{
    NvDdkAesClose_in in;
    NvDdkAesClose_inout inout;
    NvDdkAesClose_out out;
} NvDdkAesClose_params;

typedef struct NvDdkAesOpen_in_t
{
    NvU32 package_;
    NvU32 function_;
    NvU32 InstanceId;
} NV_ALIGN(4) NvDdkAesOpen_in;

typedef struct NvDdkAesOpen_inout_t
{
    NvU32 dummy_;
} NV_ALIGN(4) NvDdkAesOpen_inout;

typedef struct NvDdkAesOpen_out_t
{
    NvError ret_;
    NvDdkAesHandle phAes;
} NV_ALIGN(4) NvDdkAesOpen_out;

typedef struct NvDdkAesOpen_params_t
{
    NvDdkAesOpen_in in;
    NvDdkAesOpen_inout inout;
    NvDdkAesOpen_out out;
} NvDdkAesOpen_params;

static NvError NvDdkAesDisableCrypto_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesDisableCrypto_in *p_in;
    NvDdkAesDisableCrypto_out *p_out;

    p_in = (NvDdkAesDisableCrypto_in *)InBuffer;
    p_out = (NvDdkAesDisableCrypto_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesDisableCrypto_params, out) - OFFSET(NvDdkAesDisableCrypto_params, inout));


    p_out->ret_ = NvDdkAesDisableCrypto( p_in->hAes );

    return err_;
}

static NvError NvDdkAesSetAndLockSecureStorageKey_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesSetAndLockSecureStorageKey_in *p_in;
    NvDdkAesSetAndLockSecureStorageKey_out *p_out;
    NvU8  *pSecureStorageKey = NULL;

    p_in = (NvDdkAesSetAndLockSecureStorageKey_in *)InBuffer;
    p_out = (NvDdkAesSetAndLockSecureStorageKey_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesSetAndLockSecureStorageKey_params, out) - OFFSET(NvDdkAesSetAndLockSecureStorageKey_params, inout));

    if( p_in->KeyLength && p_in->pSecureStorageKey )
    {
        pSecureStorageKey = (NvU8  *)NvOsAlloc( p_in->KeyLength * sizeof( NvU8  ) );
        if( !pSecureStorageKey )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pSecureStorageKey )
        {
            err_ = NvOsCopyIn( pSecureStorageKey, p_in->pSecureStorageKey, p_in->KeyLength * sizeof( NvU8  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }

    p_out->ret_ = NvDdkAesSetAndLockSecureStorageKey( p_in->hAes, p_in->KeyLength, pSecureStorageKey );

clean:
    NvOsFree( pSecureStorageKey );
    return err_;
}

static NvError NvDdkAesLockSecureStorageKey_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesLockSecureStorageKey_in *p_in;
    NvDdkAesLockSecureStorageKey_out *p_out;

    p_in = (NvDdkAesLockSecureStorageKey_in *)InBuffer;
    p_out = (NvDdkAesLockSecureStorageKey_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesLockSecureStorageKey_params, out) - OFFSET(NvDdkAesLockSecureStorageKey_params, inout));


    p_out->ret_ = NvDdkAesLockSecureStorageKey( p_in->hAes );

    return err_;
}

static NvError NvDdkAesClearSecureBootKey_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesClearSecureBootKey_in *p_in;
    NvDdkAesClearSecureBootKey_out *p_out;

    p_in = (NvDdkAesClearSecureBootKey_in *)InBuffer;
    p_out = (NvDdkAesClearSecureBootKey_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesClearSecureBootKey_params, out) - OFFSET(NvDdkAesClearSecureBootKey_params, inout));


    p_out->ret_ = NvDdkAesClearSecureBootKey( p_in->hAes );

    return err_;
}

static NvError NvDdkAesGetCapabilities_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesGetCapabilities_in *p_in;
    NvDdkAesGetCapabilities_inout *p_inout;
    NvDdkAesGetCapabilities_out *p_out;
    NvDdkAesGetCapabilities_inout inout;

    p_in = (NvDdkAesGetCapabilities_in *)InBuffer;
    p_inout = (NvDdkAesGetCapabilities_inout *)((NvU8 *)InBuffer + OFFSET(NvDdkAesGetCapabilities_params, inout));
    p_out = (NvDdkAesGetCapabilities_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesGetCapabilities_params, out) - OFFSET(NvDdkAesGetCapabilities_params, inout));

    (void)inout;
    inout.pCapabilities = p_inout->pCapabilities;

    p_out->ret_ = NvDdkAesGetCapabilities( p_in->hAes, &inout.pCapabilities );


    p_inout = (NvDdkAesGetCapabilities_inout *)OutBuffer;
    p_inout->pCapabilities = inout.pCapabilities;
    return err_;
}

static NvError NvDdkAesProcessBuffer_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesProcessBuffer_in *p_in;
    NvDdkAesProcessBuffer_out *p_out;
    NvU8  *pSrcBuffer = NULL;
    NvU8  *pDestBuffer = NULL;

    p_in = (NvDdkAesProcessBuffer_in *)InBuffer;
    p_out = (NvDdkAesProcessBuffer_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesProcessBuffer_params, out) - OFFSET(NvDdkAesProcessBuffer_params, inout));

    if( p_in->SrcBufferSize && p_in->pSrcBuffer )
    {
        pSrcBuffer = (NvU8  *)NvOsAlloc( p_in->SrcBufferSize * sizeof( NvU8  ) );
        if( !pSrcBuffer )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pSrcBuffer )
        {
            err_ = NvOsCopyIn( pSrcBuffer, p_in->pSrcBuffer, p_in->SrcBufferSize * sizeof( NvU8  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }
    if( p_in->DestBufferSize && p_in->pDestBuffer )
    {
        pDestBuffer = (NvU8  *)NvOsAlloc( p_in->DestBufferSize * sizeof( NvU8  ) );
        if( !pDestBuffer )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pDestBuffer )
        {
            err_ = NvOsCopyIn( pDestBuffer, p_in->pDestBuffer, p_in->DestBufferSize * sizeof( NvU8  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }

    p_out->ret_ = NvDdkAesProcessBuffer( p_in->hAes, p_in->SrcBufferSize, p_in->DestBufferSize, pSrcBuffer, pDestBuffer );

    if(p_in->pDestBuffer && pDestBuffer)
    {
        err_ = NvOsCopyOut( p_in->pDestBuffer, pDestBuffer, p_in->DestBufferSize * sizeof( NvU8  ) );
        if( err_ != NvSuccess )
        {
            err_ = NvError_BadParameter;
        }
    }
clean:
    NvOsFree( pSrcBuffer );
    NvOsFree( pDestBuffer );
    return err_;
}

static NvError NvDdkAesGetInitialVector_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesGetInitialVector_in *p_in;
    NvDdkAesGetInitialVector_out *p_out;
    NvU8  *pInitialVector = NULL;

    p_in = (NvDdkAesGetInitialVector_in *)InBuffer;
    p_out = (NvDdkAesGetInitialVector_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesGetInitialVector_params, out) - OFFSET(NvDdkAesGetInitialVector_params, inout));

    if( p_in->VectorSize && p_in->pInitialVector )
    {
        pInitialVector = (NvU8  *)NvOsAlloc( p_in->VectorSize * sizeof( NvU8  ) );
        if( !pInitialVector )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pInitialVector )
        {
            err_ = NvOsCopyIn( pInitialVector, p_in->pInitialVector, p_in->VectorSize * sizeof( NvU8  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }

    p_out->ret_ = NvDdkAesGetInitialVector( p_in->hAes, p_in->VectorSize, pInitialVector );

    if(p_in->pInitialVector && pInitialVector)
    {
        err_ = NvOsCopyOut( p_in->pInitialVector, pInitialVector, p_in->VectorSize * sizeof( NvU8  ) );
        if( err_ != NvSuccess )
        {
            err_ = NvError_BadParameter;
        }
    }
clean:
    NvOsFree( pInitialVector );
    return err_;
}

static NvError NvDdkAesSetInitialVector_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesSetInitialVector_in *p_in;
    NvDdkAesSetInitialVector_out *p_out;
    NvU8  *pInitialVector = NULL;

    p_in = (NvDdkAesSetInitialVector_in *)InBuffer;
    p_out = (NvDdkAesSetInitialVector_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesSetInitialVector_params, out) - OFFSET(NvDdkAesSetInitialVector_params, inout));

    if( p_in->VectorSize && p_in->pInitialVector )
    {
        pInitialVector = (NvU8  *)NvOsAlloc( p_in->VectorSize * sizeof( NvU8  ) );
        if( !pInitialVector )
        {
            err_ = NvError_InsufficientMemory;
            goto clean;
        }
        if( p_in->pInitialVector )
        {
            err_ = NvOsCopyIn( pInitialVector, p_in->pInitialVector, p_in->VectorSize * sizeof( NvU8  ) );
            if( err_ != NvSuccess )
            {
                err_ = NvError_BadParameter;
                goto clean;
            }
        }
    }

    p_out->ret_ = NvDdkAesSetInitialVector( p_in->hAes, pInitialVector, p_in->VectorSize );

clean:
    NvOsFree( pInitialVector );
    return err_;
}

static NvError NvDdkAesSelectOperation_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesSelectOperation_in *p_in;
    NvDdkAesSelectOperation_out *p_out;

    p_in = (NvDdkAesSelectOperation_in *)InBuffer;
    p_out = (NvDdkAesSelectOperation_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesSelectOperation_params, out) - OFFSET(NvDdkAesSelectOperation_params, inout));


    p_out->ret_ = NvDdkAesSelectOperation( p_in->hAes, &p_in->pOperation );

    return err_;
}

static NvError NvDdkAesSelectKey_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesSelectKey_in *p_in;
    NvDdkAesSelectKey_out *p_out;

    p_in = (NvDdkAesSelectKey_in *)InBuffer;
    p_out = (NvDdkAesSelectKey_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesSelectKey_params, out) - OFFSET(NvDdkAesSelectKey_params, inout));


    p_out->ret_ = NvDdkAesSelectKey( p_in->hAes, &p_in->pKeyInfo );

    return err_;
}

static NvError NvDdkAesClose_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesClose_in *p_in;

    p_in = (NvDdkAesClose_in *)InBuffer;

    if (p_in->hAes != NULL) NvRtFreeObjRef(Ctx, NvRtObjType_NvDdkAes_NvDdkAesHandle, p_in->hAes);

    NvDdkAesClose( p_in->hAes );

    return err_;
}

static NvError NvDdkAesOpen_dispatch_( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;
    NvDdkAesOpen_in *p_in;
    NvDdkAesOpen_out *p_out;
    NvRtObjRefHandle ref_phAes = 0;

    p_in = (NvDdkAesOpen_in *)InBuffer;
    p_out = (NvDdkAesOpen_out *)((NvU8 *)OutBuffer + OFFSET(NvDdkAesOpen_params, out) - OFFSET(NvDdkAesOpen_params, inout));

    err_ = NvRtAllocObjRef(Ctx, &ref_phAes);
    if (err_ != NvSuccess)
    {
        goto clean;
    }

    p_out->ret_ = NvDdkAesOpen( p_in->InstanceId, &p_out->phAes );

    if ( p_out->ret_ == NvSuccess )
    {
        NvRtStoreObjRef(Ctx, ref_phAes, NvRtObjType_NvDdkAes_NvDdkAesHandle, p_out->phAes);
        ref_phAes = 0;
    }
clean:
    if (ref_phAes) NvRtDiscardObjRef(Ctx, ref_phAes);
    return err_;
}

NvError nvddk_aes_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvddk_aes_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvError err_ = NvSuccess;

    switch( function ) {
    case 11:
        err_ = NvDdkAesDisableCrypto_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 10:
        err_ = NvDdkAesSetAndLockSecureStorageKey_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 9:
        err_ = NvDdkAesLockSecureStorageKey_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 8:
        err_ = NvDdkAesClearSecureBootKey_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 7:
        err_ = NvDdkAesGetCapabilities_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 6:
        err_ = NvDdkAesProcessBuffer_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 5:
        err_ = NvDdkAesGetInitialVector_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 4:
        err_ = NvDdkAesSetInitialVector_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 3:
        err_ = NvDdkAesSelectOperation_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 2:
        err_ = NvDdkAesSelectKey_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 1:
        err_ = NvDdkAesClose_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    case 0:
        err_ = NvDdkAesOpen_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
        break;
    default:
        err_ = NvError_BadParameter;
        break;
    }

    return err_;
}
