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
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvos.h"
#include "chiplib_interface.h"
#include "nvrm_chiplib.h"
#include "nvrm_init.h"

/**
 * NOTE: newer versions of chiplib (t30) use GCC 4, which changes the virtual
 * table layout. To get around this, C wrapper functions were added, so this
 * needs to check for 'QueryIface_C', which will return a struct full of
 * C function wrappers rather than the old method, which is to overlay a C
 * structure manually.
 */

/* table for virtual to physical lookups */
typedef struct RmAddrMap_t
{
    NvRmPhysAddr phys;
    void *virt;
    size_t size;
} RmAddrMap;

#define RM_ADDR_MAP_SIZE 256

static RmAddrMap s_AddrMap[ RM_ADDR_MAP_SIZE ];
static NvBool s_Shutdown;

static NvOsLibraryHandle s_chiplib = 0;
static IChip *s_IChip = 0;
static IBusMem *s_IBusMem = 0;
static NvOsMutexHandle s_ChiplibMutex = 0;

static NvOsThreadHandle s_clockThreadId = NULL;
static volatile NvBool s_bShutdownClockThread = NV_FALSE;

static NvOsMutexHandle s_simIstMutex;

/**
 * IInterrupt support
 */
static IInterrupt s_Interrupt;
static void AddRef_IInterrupt(struct IInterruptRec *pThis) { }
static void Release_IInterrupt(struct IInterruptRec *pThis) { }

static void
NvRmPrivChiplibInterruptHandler( void );

static IIfaceObject *
QueryIface_IInterrupt(struct IInterruptRec *pThis,
    IID_TYPE id)
{
    IIfaceObject *ret;

    switch (id)  {
    case IID_INTERRUPT_IFACE:
        ret = (IIfaceObject *)&s_Interrupt;
        break;
    case IID_CHIP_IFACE:
        // fall through
    case IID_BUSMEM_IFACE:
        // fall through
    default:
        ret = 0;
    }

    return ret;
}

/**
 * Note about deadlock: the interrupt handler must not be called with
 * s_ChiplibMutex locked, otherwise there are lock ordering issues. The
 * best solution is to force cooperative threading.  The second best is to
 * use an indirection thread to actually execute the handler.
 */

static ChiplibHandleInterrupt s_HandleInterrupt;
static NvOsSemaphoreHandle s_IsrSemaphore;
static NvOsThreadHandle s_IsrThread;

static void
NvRmPrivChiplibInterruptThread( void *args )
{
    for( ;; )
    {
        NvOsSemaphoreWait( s_IsrSemaphore );
        if( s_Shutdown )
        {
            break;
        }

        if( !s_IChip )
        {
            break;
        }

        if( s_HandleInterrupt )
        {
            s_HandleInterrupt();
        }
    }
}

static void
HandleInterrupt_IInterrupt(struct IInterruptRec *pThis)
{
    if( s_IsrThread && s_IsrSemaphore )
    {
        NvOsSemaphoreSignal( s_IsrSemaphore );
    }
}

static void
NvRmPrivChiplibClockthread( void *args )
{
    NvError err;
    NvBool bSleep = NV_FALSE;

    err = NvOsThreadSetLowPriority();
    if( err != NvSuccess )
    {
        bSleep = NV_TRUE;
    }

    while( s_bShutdownClockThread == NV_FALSE )
    {
        if( s_IChip )
        {
            NvS32 clocks;
            if( bSleep )
            {
                clocks = 500;
            } else
            {
                clocks = 32;
            }

            NV_ASSERT(s_ChiplibMutex);
            NvOsMutexLock(s_ChiplibMutex);
            s_IChip->pVtable->ClockSimulator(s_IChip, clocks);
            NvOsMutexUnlock(s_ChiplibMutex);
        }

        /* thread package might not support low priority threads, emulate it.
         */
        if( bSleep )
        {
            NvOsSleepMS( 50 );
        }
        else
        {
            NvOsThreadYield();
        }
    }
}

static NvBool
NvRmPrivParseCommandline(const char *cmdline, int *argc, char ***argv,
    char ***argvbuf, char **pCopy);

#if NV_DEF_ENVIRONMENT_SUPPORTS_SIM
NvBool
NvRmIsSimulation(void)
{
    return (NvBool)(s_chiplib != NULL);
}
#endif

typedef void *(*QueryIfaceCFn)( IID_TYPE id );

NvError
NvRmPrivChiplibStartup(const char *lib, const char *cmdline,
    ChiplibHandleInterrupt handler)
{
    NvError err;
    void *sym;
    QueryIfaceFn query;
    QueryIfaceCFn c_wrap;
    char *copy = 0;
    char **argvbuf = 0;
    char **argv = 0;
    int argc = 0;
    int e;

    NV_ASSERT(lib);

    if( lib[0] == 0 )
    {
        /* no chiplib defined */
        return NvSuccess;
    }

    /* all chiplib accesses must be synchronized - do not use a multi-process
     * mutex since that prevents other simulation instances on the same
     * machine.
     */
    err = NvOsMutexCreate( &s_ChiplibMutex );
    if (err != NvSuccess)
    {
        goto fail;
    }

    err = NvOsSemaphoreCreate( &s_IsrSemaphore, 0 );
    if( err != NvSuccess )
    {
        goto fail;
    }

    s_Shutdown = NV_FALSE;
    err = NvOsThreadCreate( NvRmPrivChiplibInterruptThread, 0, &s_IsrThread );
    if( err != NvSuccess )
    {
        goto fail;
    }

    /* open the chiplib .so */
    err = NvOsLibraryLoad( lib, &s_chiplib );
    if( err != NvSuccess )
    {
        goto fail;
    }

    /* try to find the C wrapper struct, if not, fallback to the old way of
     * doing thigs.
     */
    c_wrap = NvOsLibraryGetSymbol( s_chiplib, "QueryIface_C" );
    if( c_wrap )
    {
        s_IChip = (IChip *)c_wrap( IID_CHIP_IFACE );
    }
    else
    {
        /* get a chiplib instance - QUERY_PROC_NAME, etc., are from chiplib
         * headers.
         */
        sym = NvOsLibraryGetSymbol( s_chiplib, QUERY_PROC_NAME );
        if( sym == NULL )
        {
            goto fail;
        }

        query = (QueryIfaceFn)sym;
        s_IChip = (IChip *)query( IID_CHIP_IFACE );
    }
    if( !s_IChip )
    {
        goto fail;
    }

    // FIXME: should probably check for errors
    (void)NvRmPrivParseCommandline(cmdline, &argc, &argv, &argvbuf, &copy);

    /* setup the interrupt handler */
    s_Interrupt.pVtable = NvOsAlloc(sizeof(IInterruptVtable));
    if( !s_Interrupt.pVtable )
    {
        goto fail;
    }
    s_Interrupt.pVtable->AddRef = AddRef_IInterrupt;
    s_Interrupt.pVtable->Release = Release_IInterrupt;
    s_Interrupt.pVtable->QueryIface = QueryIface_IInterrupt;
    s_Interrupt.pVtable->HandleInterrupt = HandleInterrupt_IInterrupt;

    /* Use the default handler if the passed handler is NULL */
    if( handler == NULL )
    {
        s_HandleInterrupt = NvRmPrivChiplibInterruptHandler;
    }

    /* start chiplib */
    e = s_IChip->pVtable->Startup(s_IChip, (IIfaceObject *)&s_Interrupt,
        argv, argc );
    if( e )
    {
        goto fail;
    }

    /* get the bus interface */
    s_IBusMem = (IBusMem *)s_IChip->pVtable->QueryIface( s_IChip,
        IID_BUSMEM_IFACE );
    if( !s_IBusMem )
    {
        goto fail;
    }

    if( NvRmIsSimulation() )
    {
        s_bShutdownClockThread = NV_FALSE;

        err = NvOsMutexCreate( &s_simIstMutex );
        if( err != NvSuccess )
        {
            goto fail;
        }

        err = NvOsThreadCreate(NvRmPrivChiplibClockthread, NULL,
            &s_clockThreadId);
        if (err != NvSuccess)
        {
            goto fail;
        }
    }

    NvOsFree( copy );
    NvOsFree( argvbuf );
    return NvSuccess;

fail:
    NvOsFree( copy );
    NvOsFree( argvbuf );
    NvOsMutexDestroy(s_ChiplibMutex);
    NvOsLibraryUnload(s_chiplib);
    s_chiplib = 0;
    s_IChip = 0;
    s_IBusMem = 0;

    if( s_IsrSemaphore && s_IsrThread )
    {
        NvOsSemaphoreSignal( s_IsrSemaphore );
        NvOsThreadJoin( s_IsrThread );
    }
    NvOsSemaphoreDestroy( s_IsrSemaphore );
    s_IsrSemaphore = 0;

    return NvError_RmInitFailed;
}

void
NvRmPrivChiplibShutdown(void)
{
    /* First shtdown the interrupt thread */
    if( s_IsrSemaphore && s_IsrThread )
    {
        s_Shutdown = NV_TRUE;
        NvOsSemaphoreSignal( s_IsrSemaphore );
        NvOsThreadJoin( s_IsrThread );
        s_Shutdown = NV_FALSE;
    }
    NvOsSemaphoreDestroy( s_IsrSemaphore );
    s_IsrSemaphore = 0;

    /* next shutdown the clocking thread */
    if (NvRmIsSimulation())
    {
        s_bShutdownClockThread = NV_TRUE;
        NvOsThreadJoin( s_clockThreadId );
        NvOsMutexDestroy(s_simIstMutex);
        s_simIstMutex = 0;
        s_clockThreadId = 0;
    }

    /* Finally shutdown the chiplib */
    NvOsMutexLock(s_ChiplibMutex);
    if (s_IChip)
    {
        s_IChip->pVtable->Shutdown(s_IChip);
        s_IChip->pVtable->Release(s_IChip);
        s_IChip = NULL;
    }
    if (s_IBusMem)
    {
        s_IBusMem->pVtable->Release(s_IBusMem);
        s_IBusMem = NULL;
    }

    if (s_Interrupt.pVtable)
    {
        s_Interrupt.pVtable->Release(&s_Interrupt);
        NvOsFree(s_Interrupt.pVtable);
    }
    NvOsMutexDestroy(s_ChiplibMutex);
}

static NvBool
NvRmPrivParseCommandline( const char *cmdline, int *argc, char ***argv,
    char ***pArgv, char **pCopy )
{
    /* keep some amount of stack space to prevent dynamic allocation in the
     * average case.
     */
    #define TOKEN_SIZE_GUESS 16

    static char *s_argv[ TOKEN_SIZE_GUESS ];
    char *env = 0;
    char *start = 0;
    char *end = 0;
    char *copy = 0;
    NvU32 size;
    NvU32 len;
    NvU32 index;

    /* get the command line */
    env = (char *)cmdline;

    /*
     * this needs to do two passes over the environment string. can't think
     * of a way to do it with one pass without using realloc.  performace
     * should be ok either way and doesn't really matter anyway.
     *
     * just allocate one copy of the env string, then replace the spaces
     * with nulls, assign the tokens into argv - this avoids strcpy and
     * an allocation per token.
     */

    /* count the number of tokens and env string length */
    size = 1; /* (should) always be at least one token */
    len = 0;
    start = env;
    while (*start)
    {
        if (*start == ' ')
        {
            size++;
        }

        start++;
        len++;
    }

    if (len == 0)
    {
        return NV_FALSE;
    }

    /* allocate argv */
    size++; /* executable name */
    size++; /* null terminate array */
    if (size >= TOKEN_SIZE_GUESS)
    {
        *argv = NvOsAlloc(size * sizeof(char *));
        if (!(*argv))
        {
            return NV_FALSE;
        }
        *pArgv = *argv;
    }
    else
    {
        /* guess that most argv arrays are TOKEN_SIZE_GUESS or less long */
        *argv = s_argv;
    }

    /* assign argc */
    *argc = size - 1; /* don't include null termination */

    (*argv)[ size ] = 0;
    // FIXME: should get the execuable name
    (*argv)[ 0 ] = "bogus"; /* executable name */

    /* allocate and copy the string */
    len++;
    copy = NvOsAlloc(len);
    if (copy == 0)
    {
        goto fail;
    }

    *pCopy = copy;

    NvOsStrncpy(copy, env, len - 1);
    copy[ len - 1 ] = 0;

    /* fill argv - find each token - assign to argv */
    index = 1;
    start = copy;
    while (*start)
    {
        /* find a token */
        end = start;
        while (*end && *end != ' ')
        {
            end++;
        }

        /* assign to argv */
        (*argv)[ index ] = start;
        index++;

        start = end;
        if (*end == ' ')
        {
            /* replace space with null and move to next token */
            *end = 0;
            start++;
        }
    }

    return NV_TRUE;

fail:
    NvOsFree(*pArgv);
    *pArgv = 0;
    NvOsFree(copy);
    *pCopy = 0;

    #undef TOKEN_SIZE_GUESS

    return NV_FALSE;
}

void *
NvRmPrivChiplibMap(NvRmPhysAddr addr, size_t size)
{
    NvError err;
    void *virt;
    NvU32 i;
    RmAddrMap *map = 0;

    /* map some bogus memory with guard page */
    err = NvOsPhysicalMemMap(addr, size + 4096, NvOsMemAttribute_WriteBack,
        NVOS_MEM_NONE, &virt);
    if (err != NvSuccess)
    {
        return 0;
    }

    /* find a free entry */
    for (i = 0; i < RM_ADDR_MAP_SIZE; i++)
    {
        if (s_AddrMap[i].phys == 0 &&
            s_AddrMap[i].virt == 0)
        {
            map = &s_AddrMap[i];
            break;
        }
    }

    if (!map)
    {
        NvOsPhysicalMemUnmap(virt, size + 4096);
        return 0;
    }

    /* setup entry */
    map->phys = addr;
    map->virt = virt;
    map->size = size;

    return virt;
}

void
NvRmPrivChiplibUnmap(void *addr)
{
    NvU32 i;

    if( !addr )
    {
        return;
    }

    for( i = 0; i < RM_ADDR_MAP_SIZE; i++ )
    {
        if( s_AddrMap[i].virt == addr )
        {
            /* unmap (don't forget the guard page) */
            NvOsPhysicalMemUnmap(addr, s_AddrMap[i].size + 4096);
            NvOsMemset(&s_AddrMap[i], 0, sizeof(s_AddrMap[i]));
            break;
        }
    }
}

static NvBool
NvRmPrivVirtToPhys(const void *virt, NvRmPhysAddr *phys)
{
    NvU32 i;
    RmAddrMap *map;
    NvRmPhysAddr addr;
    NvRmPhysAddr base;

    addr = (NvRmPhysAddr)virt;

    /* find the address range and convert to a physical address, use
     * physical address type just to be safe.
     */
    for( i = 0; i < RM_ADDR_MAP_SIZE; i++ )
    {
        map = &s_AddrMap[i];
        base = (NvRmPhysAddr)map->virt;
        if( addr >= base && addr < (base + map->size) )
        {
            *phys = addr - base + map->phys;
            return NV_TRUE;
        }
    }

    return NV_FALSE;
}

void NvWrite08(void *addr, NvU8 data)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        *(NvU8 *)addr = data;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemWrBlk(s_IBusMem, phys, &data,
            sizeof(data));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

void NvWrite16(void *addr, NvU16 data)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        *(NvU16 *)addr = data;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemWrBlk(s_IBusMem, phys, &data,
            sizeof(data));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

void NvWrite32(void *addr, NvU32 data)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        *(NvU32 *)addr = data;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemWrBlk(s_IBusMem, phys, &data,
            sizeof(data));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

void NvWrite64(void *addr, NvU64 data)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        *(NvU64 *)addr = data;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemWrBlk(s_IBusMem, phys, &data,
            sizeof(data));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

NvU8 NvRead08(void *addr)
{
    BusMemRet err;
    NvRmPhysAddr phys;
    NvU8 ret;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        ret = *(NvU8 *)addr;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemRdBlk(s_IBusMem, phys, &ret,
            sizeof(ret));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }

    return ret;
}

NvU16 NvRead16(void *addr)
{
    BusMemRet err;
    NvRmPhysAddr phys;
    NvU16 ret;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        ret = *(NvU16 *)addr;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemRdBlk(s_IBusMem, phys, &ret,
            sizeof(ret));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }

    return ret;
}

NvU32 NvRead32(void *addr)
{
    BusMemRet err;
    NvRmPhysAddr phys;
    NvU32 ret;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        ret = *(NvU32 *)addr;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemRdBlk(s_IBusMem, phys, &ret,
            sizeof(ret));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }

    return ret;
}

NvU64 NvRead64(void *addr)
{
    BusMemRet err;
    NvRmPhysAddr phys;
    NvU64 ret;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(addr, &phys))
    {
        ret = *(NvU64 *)addr;
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemRdBlk(s_IBusMem, phys, &ret,
            sizeof(ret));
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }

    return ret;
}

void NvWriteBlk(void *dst, const void *src, NvU32 length)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(dst, &phys))
    {
        NvOsMemcpy(dst, src, length);
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemWrBlk(s_IBusMem, phys, src, length);
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

void NvReadBlk(void *dst, const void *src, NvU32 length)
{
    BusMemRet err;
    NvRmPhysAddr phys;

    if (!s_IBusMem || !NvRmPrivVirtToPhys(src, &phys))
    {
        NvOsMemcpy(dst, src, length);
    }
    else
    {
        NvOsMutexLock(s_ChiplibMutex);
        err = s_IBusMem->pVtable->BusMemRdBlk(s_IBusMem, phys, dst, length);
        NV_ASSERT(err == BUSMEM_HANDLED);
        NvOsMutexUnlock(s_ChiplibMutex);
    }
}

extern void
NvRmPrivHandleOsInterrupt( void *arg );

static void
NvRmPrivChiplibInterruptHandler( void )
{
    if (NvRmIsSimulation())
    {
        NvOsMutexLock(s_simIstMutex);
    }

    /* Chiplib and AOS share the interrpt handling code. 
     * No chiplib interrupt support for wince and linux ARM port 
     */
#if !NVCPU_IS_ARM
    NvRmPrivHandleOsInterrupt(NULL);
#endif

    if (NvRmIsSimulation())
    {
        NvOsMutexUnlock(s_simIstMutex);
    }
}
