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
#include "nvodm_services.h"
#include "nvos.h"
#include "nvassert.h"

void NvOdmOsPrintf( const char *format, ...)
{

    va_list ap;

    va_start(ap, format);
    NvOsDebugVprintf(format, ap);
    va_end(ap);

}

void NvOdmOsDebugPrintf( const char *format, ... )
{
#if NV_DEBUG
    va_list ap;
    va_start( ap, format );
    NvOsDebugVprintf( format, ap );
    va_end( ap );
#endif
}

void *NvOdmOsAlloc(size_t size)
{
    return NvOsAlloc( size );
}

void NvOdmOsFree(void *ptr)
{
    NvOsFree( ptr );
}

void NvOdmOsMemcpy(void *dest, const void *src, size_t size)
{
    NvOsMemcpy(dest, src, size);
}

void NvOdmOsMemset(void *s, NvU8 c, size_t size)
{
    NvOsMemset(s, c, size);
}

NvOdmOsMutexHandle NvOdmOsMutexCreate(void)
{
    NvError err;
    NvOsMutexHandle m;

    err = NvOsMutexCreate(&m);
    if( err == NvSuccess )
    {
        return (NvOdmOsMutexHandle)m;
    }

    return NULL;
}

void NvOdmOsMutexLock(NvOdmOsMutexHandle mutex)
{
    NvOsMutexLock((NvOsMutexHandle)mutex);
}

void NvOdmOsMutexUnlock(NvOdmOsMutexHandle mutex)
{
    NvOsMutexUnlock((NvOsMutexHandle)mutex);
}

void NvOdmOsMutexDestroy(NvOdmOsMutexHandle mutex)
{
    NvOsMutexDestroy((NvOsMutexHandle)mutex);
}

NvOdmOsSemaphoreHandle NvOdmOsSemaphoreCreate(NvU32 value)
{
    NvError err;
    NvOsSemaphoreHandle s;

    err = NvOsSemaphoreCreate(&s, value);
    if( err == NvSuccess )
    {
        return (NvOdmOsSemaphoreHandle)s;
    }

    return NULL;
}

void NvOdmOsSemaphoreWait(NvOdmOsSemaphoreHandle semaphore)
{
    NvOsSemaphoreWait((NvOsSemaphoreHandle)semaphore);
}

NvBool NvOdmOsSemaphoreWaitTimeout(NvOdmOsSemaphoreHandle semaphore,
    NvU32 msec)
{
    NvError err;

    err = NvOsSemaphoreWaitTimeout((NvOsSemaphoreHandle)semaphore, msec);
    if (err == NvError_Timeout)
    {
        return NV_FALSE;
    }

    return NV_TRUE;
}

void NvOdmOsSemaphoreSignal(NvOdmOsSemaphoreHandle semaphore)
{
    NvOsSemaphoreSignal( (NvOsSemaphoreHandle)semaphore );
}

void NvOdmOsSemaphoreDestroy(NvOdmOsSemaphoreHandle semaphore)
{
    NvOsSemaphoreDestroy((NvOsSemaphoreHandle)semaphore);
}

NvOdmOsThreadHandle NvOdmOsThreadCreate(NvOdmOsThreadFunction function,
    void *args)
{
    NvError err;
    NvOsThreadHandle t;

    err = NvOsThreadCreate((NvOsThreadFunction)function, args, &t);
    if (err == NvSuccess)
    {
        return (NvOdmOsThreadHandle)t;
    }

    return NULL;
}

void NvOdmOsThreadJoin(NvOdmOsThreadHandle thread)
{
    NvOsThreadJoin((NvOsThreadHandle)thread);
}

void NvOdmOsWaitUS(NvU32 usec)
{
    NvOsWaitUS(usec);
}

void NvOdmOsSleepMS(NvU32 msec)
{
    NvOsSleepMS(msec);
}

NvU32 NvOdmOsGetTimeMS(void)
{
    return NvOsGetTimeMS();
}

// Assert that the types defined in nvodm_services.h map correctly to their
// corresponding nvos types.
NV_CT_ASSERT(NVOS_OPEN_READ == NVODMOS_OPEN_READ);
NV_CT_ASSERT(NVOS_OPEN_WRITE == NVODMOS_OPEN_WRITE);
NV_CT_ASSERT(NVOS_OPEN_CREATE == NVODMOS_OPEN_CREATE);

NV_CT_ASSERT(NvOsFileType_File == NvOdmOsFileType_File);
NV_CT_ASSERT(NvOsFileType_Directory == NvOdmOsFileType_Directory);
NV_CT_ASSERT(sizeof(NvOsStatType) == sizeof(NvOdmOsStatType));

NvBool NvOdmOsFopen(const char *path, NvU32 flags, NvOdmOsFileHandle *file)
{
    return (NvOsFopen(path, flags, (NvOsFileHandle*)file) == NvSuccess);
}

void NvOdmOsFclose(NvOdmOsFileHandle stream)
{
    NvOsFclose((NvOsFileHandle)stream);
}

NvBool NvOdmOsFwrite(NvOdmOsFileHandle stream, const void *ptr, size_t size)
{
    return (NvOsFwrite((NvOsFileHandle)stream, ptr, size) == NvSuccess);
}

NvBool NvOdmOsFread(NvOdmOsFileHandle stream, void *ptr, size_t size,
    size_t *bytes)
{
    return (NvOsFread((NvOsFileHandle)stream, ptr, size, bytes) == NvSuccess);
}

NvBool NvOdmOsStat(const char *filename, NvOdmOsStatType *stat)
{
    return (NvOsStat(filename, (NvOsStatType*)stat) == NvSuccess);
}

NvBool NvOdmOsGetOsInformation(NvOdmOsOsInfo *pOsInfo)
{
    NvOsOsInfo info;
    NvError e;

    if (!pOsInfo)
    {
        return NV_FALSE;
    }

    e = NvOsGetOsInformation(&info);
    if (e != NvSuccess)
    {
        return NV_FALSE;
    }

    pOsInfo->OsType = NvOdmOsOs_Linux;
    pOsInfo->Sku = NvOdmOsSku_Unknown;
    pOsInfo->MajorVersion = info.MajorVersion;
    pOsInfo->MinorVersion = info.MinorVersion;
    pOsInfo->SubVersion = info.SubVersion;
    pOsInfo->Caps = info.Caps;

    return NV_TRUE;
}

