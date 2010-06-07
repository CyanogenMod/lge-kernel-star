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
#include "nvidlcmd.h"
#include "nvreftrack.h"
#include "nvddk_aes.h"
NvError nvddk_aes_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );

// NvDdkAesCommon Package
typedef enum
{
    NvDdkAesCommon_Invalid = 0,
    NvDdkAesCommon_nvddk_aes_common,
    NvDdkAesCommon_Num,
    NvDdkAesCommon_Force32 = 0x7FFFFFFF,
} NvDdkAesCommon;

// NvDdkAes Package
typedef enum
{
    NvDdkAes_Invalid = 0,
    NvDdkAes_nvddk_aes,
    NvDdkAes_Num,
    NvDdkAes_Force32 = 0x7FFFFFFF,
} NvDdkAes;

typedef NvError (* NvIdlDispatchFunc)( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );

typedef struct NvIdlDispatchTableRec
{
    NvU32 PackageId;
    NvIdlDispatchFunc DispFunc;
} NvIdlDispatchTable;

static NvIdlDispatchTable gs_DispatchTable[] =
{
    { NvDdkAes_nvddk_aes, nvddk_aes_Dispatch },
    { 0, 0 },
};

NvError NvDdkAes_Dispatch( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvU32 packid_;
    NvU32 funcid_;
    NvIdlDispatchTable *table_;

    NV_ASSERT( InBuffer );
    NV_ASSERT( OutBuffer );

    packid_ = ((NvU32 *)InBuffer)[0];
    funcid_ = ((NvU32 *)InBuffer)[1];
    table_ = gs_DispatchTable;

    if ( packid_-1 >= NV_ARRAY_SIZE(gs_DispatchTable) ||
         !table_[packid_ - 1].DispFunc )
        return NvError_IoctlFailed;

    return table_[packid_ - 1].DispFunc( funcid_, InBuffer, InSize,
        OutBuffer, OutSize, Ctx );
}
