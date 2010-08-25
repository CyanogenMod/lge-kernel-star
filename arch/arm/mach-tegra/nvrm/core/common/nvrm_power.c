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

/**
 * @file
 * @brief <b>nVIDIA Driver Development Kit:
 *           Power Resource manager </b>
 *
 * @b Description: Implements the interface of the NvRM Power.
 *
 */

#include "nvrm_power_private.h"
#include "nvrm_pmu_private.h"
#include "ap15/ap15rm_private.h"
#include "ap15/project_relocation_table.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap15/arapbpm.h"
#include "nvrm_clocks.h"
#include "nvodm_query.h"

// TODO: Always Disable before check-in
// Module debug: 0=disable, 1=enable
#define NVRM_POWER_ENABLE_PRINTF (0)

// TODO: Always Disable before check-in
// Report every change in RM clients power state: 0=disable, 1=enable
#define NVRM_POWER_VERBOSE_PRINTF (0)

#if NVRM_POWER_ENABLE_PRINTF || NVRM_POWER_VERBOSE_PRINTF
#define NVRM_POWER_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_POWER_PRINTF(x)
#endif

// Active modules report on suspend entry : 0=disable, 1=enable
#define NVRM_POWER_DEBUG_SUSPEND_ENTRY (1)

/*****************************************************************************/

// Specifies initial registry size as well as delta for dynamic size change
#define NVRM_POWER_REGISTRY_DELTA (NvRmPrivModuleID_Num)

/*
 * Convert registry index to client ID and vice versa: just use
 * provided mask as high bits combined with index in low bits
 * (index is expected to not exceed 16 bits ever)
 */
#define NVRM_POWER_INDEX2ID(index, mask) (((mask) << 16) | (index))
#define NVRM_POWER_ID2INDEX(id) ((id) & 0xFFFF)


/*
 * Holds power client voltage request information for a
 * particular module
 */
typedef struct ModuleVoltageReqRec
{
    // Target module (combined ID and instance)
    NvRmModuleID ModuleId;

    // Power group number module belongs to
    NvU32 PowerGroup;

    // Module power cycle indicator
    NvBool PowerCycled;

    // Requested voltage range
    NvRmMilliVolts MinVolts;
    NvRmMilliVolts MaxVolts;

    // Pointer to the next module info node
    struct ModuleVoltageReqRec* pNext;
} ModuleVoltageReq;

/*
 * Holds power client clock request information for a
 * particular module
 */
typedef struct ModuleClockReqRec
{
    // TODO: Define clock request information members

    // Pointer to the next module info node
    struct ModuleClockReqRec* pNext;
} ModuleClockReq;

/*
 * Holds power client busy hint information for a
 * particular clock domain
 */
typedef struct BusyHintReqRec
{
    // Requested busy pulse mode
    NvBool BusyPulseMode;

    // Requested frequency boost in KHz
    NvRmFreqKHz BoostKHz;

    // Requested boost interval in ms
    NvU32 IntervalMs;

    // Boost start time in ms
    NvU32 StartTimeMs;

    // Id of the requester
    NvU32 ClientId;

    // Pointer to the next busy hint node
    struct BusyHintReqRec* pNext;
} BusyHintReq;

/*
 * Combines voltage and clock requets, starvation and busy hints,
 * as well as recorded power events for a particular client
 */
typedef struct NvRmPowerClientRec
{
    // Client registration ID
    NvU32 id;

    // Client semaphore for power management event signaling
    NvOsSemaphoreHandle hEventSemaphore;

    // Last detected power management event
    NvRmPowerEvent Event;

    // Pointer to the array of starvation hints
    NvBool* pStarvationHints;

    // Head pointer to client volatge request list
    ModuleVoltageReq* pVoltageReqHead;

    // Head pointer to client clock request list
    ModuleClockReq* pClockReqHead;

    // Client 4-character tag
    NvU32 tag;
} NvRmPowerClient;

/*
 * Combines information on power clients registred
 * with RM
 */
typedef struct NvRmPowerRegistryRec
{
    // Array of pointers to power client records
    NvRmPowerClient** pPowerClients;

    // Used index range (max used entry index + 1)
    NvU32 UsedIndexRange;

    // Total number of available entries (array size)
    NvU32 AvailableEntries;
} NvRmPowerRegistry;

// RM power clients registry
static NvRmPowerRegistry s_PowerRegistry;

// Mutex for thread-safe access to RM power clients records
static NvOsMutexHandle s_hPowerClientMutex = NULL;

// "Power On" request reference count for each SoC Power Group. Appended
// at the end is a duplicate entry for NPG group that represents power
// requirements for autonomous h/w operations with no s/w activity
static NvU32 s_PowerOnRefCounts[NV_POWERGROUP_MAX + 1];
#define NVRM_POWERGROUP_NPG_AUTO (NV_POWERGROUP_MAX)

// Active starvation hints reference count for each DFS clock domain
static NvU32 s_StarveOnRefCounts[NvRmDfsClockId_Num];

// Heads of busy hint lists for DFS clock domain
static BusyHintReq s_BusyReqHeads[NvRmDfsClockId_Num];

// Busy requests pool
#define NVRM_BUSYREQ_POOL_SIZE (24)
static BusyHintReq s_BusyReqPool[NVRM_BUSYREQ_POOL_SIZE];
static BusyHintReq* s_pFreeBusyReqPool[NVRM_BUSYREQ_POOL_SIZE];
static NvU32 s_FreeBusyReqPoolSize = 0;

/*****************************************************************************/

/*
 * Release memory and system resources allocated for the specified power client
 */
static void FreePowerClient(NvRmPowerClient* pPowerClient);

/*
 * Cancel all requests issued by the specified power client
 */
static void CancelPowerRequests(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerClient* pPowerClient);

/*
 * Notifies RM Clients about power management event
 */
static void
PowerEventNotify(NvRmDeviceHandle hRmDeviceHandle, NvRmPowerEvent Event);

/*
 * Records power cycle for all RM clients in the specified group
 */
static void
RecordPowerCycle(NvRmDeviceHandle hRmDeviceHandle, NvU32 PowerGroup);

/*
 * Reports combined RM clients power state to OS adaptation layer
 * (chip-aware implementation)
 */
static void
ReportRmPowerState(NvRmDeviceHandle hRmDeviceHandle);

/*
 * Manages busy request pool
 */
static BusyHintReq* BusyReqAlloc(void);
static void BusyReqFree(BusyHintReq* pBusyHintReq);

/*
 * Cancels busy hints reported by the specified client for
 * specified domain
 */
static void
CancelBusyHints(NvRmDfsClockId ClockId, NvU32 ClientId);

/*
 * Records starvation hints reported against DFS domains by
 * the specified client
 */
static NvError
RecordStarvationHints(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerClient* pPowerClient,
    const NvRmDfsStarvationHint* pMultiHint,
    NvU32 NumHints);

/*
 * Records busy hints reported against DFS domains by
 * the specified client
 */
static NvError
RecordBusyHints(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId,
    const NvRmDfsBusyHint* pMultiHint,
    NvU32 NumHints,
    NvBool* pSignalDfs);

/* Send a simple message to the AVP indicating that it needs
 * to save state in preparation for LP0 (explicit case)
 */
NvError
NvRmPrivSendAVPIdleMessage( NvRmDeviceHandle hRmDeviceHandle );

/*****************************************************************************/
static void FreePowerClient(NvRmPowerClient* pPowerClient)
{
    ModuleVoltageReq* pVoltageReq = NULL;
    ModuleClockReq* pClockReq = NULL;

    // Just return if null-pointer
    if (pPowerClient == NULL)
        return;

    // Free memory occupied by voltage requests
    while (pPowerClient->pVoltageReqHead != NULL)
    {
        pVoltageReq = pPowerClient->pVoltageReqHead;
        pPowerClient->pVoltageReqHead = pVoltageReq->pNext;
        NvOsFree(pVoltageReq);
    }

    // Free memory occupied by clock requests
    while (pPowerClient->pClockReqHead != NULL)
    {
        pClockReq = pPowerClient->pClockReqHead;
        pPowerClient->pClockReqHead = pClockReq->pNext;
        NvOsFree(pClockReq);
    }

    // Free memory occupied by starvation hints array
    NvOsFree(pPowerClient->pStarvationHints);

    // Free power management event semaphore handle
    NvOsSemaphoreDestroy(pPowerClient->hEventSemaphore);

    // Free memory occupied by the client record
    NvOsFree(pPowerClient);
}

static void CancelPowerRequests(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerClient* pPowerClient)
{
    NvU32 i;
    ModuleVoltageReq* pVoltageReq = NULL;

    // Cancel power On requests and update power planes as well as
    // combined RM clients power state accordingly
    pVoltageReq = pPowerClient->pVoltageReqHead;
    while (pVoltageReq != NULL)
    {
        if (pVoltageReq->MaxVolts != NvRmVoltsOff)
        {
            NvU32 PowerGroup = pVoltageReq->PowerGroup;
            pVoltageReq->MaxVolts = NvRmVoltsOff;

            NV_ASSERT(s_PowerOnRefCounts[PowerGroup] != 0);
            s_PowerOnRefCounts[PowerGroup]--;
            if (s_PowerOnRefCounts[PowerGroup] == 0)
            {
                NvRmPrivPowerGroupControl(hRmDeviceHandle, PowerGroup,
                    NV_FALSE);
                ReportRmPowerState(hRmDeviceHandle);
            }
        }
        pVoltageReq = pVoltageReq->pNext;
    }
    // Cancel starvation hints
    if (pPowerClient->pStarvationHints != NULL)
    {
        for (i = 0; i < NvRmDfsClockId_Num; i++)
        {
            if (pPowerClient->pStarvationHints[i])
            {
                pPowerClient->pStarvationHints[i] = NV_FALSE;
                if ((i == NvRmDfsClockId_Cpu) ||
                    (i == NvRmDfsClockId_Avp) ||
                    (i == NvRmDfsClockId_Vpipe))
                {
                    NV_ASSERT(s_StarveOnRefCounts[NvRmDfsClockId_Emc] != 0);
                    s_StarveOnRefCounts[NvRmDfsClockId_Emc]--;
                }
                NV_ASSERT(s_StarveOnRefCounts[i] != 0);
                s_StarveOnRefCounts[i]--;
            }
        }
    }

    // Cancle busy hints
    for (i = 0; i < NvRmDfsClockId_Num; i++)
    {
        CancelBusyHints(i, pPowerClient->id);
    }

    // TODO: Cancel clock requests issued by the client
}

/*****************************************************************************/
NvError NvRmPrivPowerInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;
    NvError e;

    NV_ASSERT(hRmDeviceHandle);

    // Initialize registry
    s_PowerRegistry.pPowerClients = NULL;
    s_PowerRegistry.AvailableEntries = 0;
    s_PowerRegistry.UsedIndexRange = 0;

    // Clear busy head pointers as well as starvation and power plane
    // reference counts. Aalthough power plane references are cleared
    // here, the combined power state is not updated - it will kept as
    // set by the boot code, until the 1st client requests power.
    NvOsMemset(s_BusyReqHeads, 0, sizeof(s_BusyReqHeads));
    NvOsMemset(s_StarveOnRefCounts, 0, sizeof(s_StarveOnRefCounts));
    NvOsMemset(s_PowerOnRefCounts, 0, sizeof(s_PowerOnRefCounts));

    // Initialize busy requests pool
    NvOsMemset(s_BusyReqPool, 0, sizeof(s_BusyReqPool));
    for (i = 0; i < NVRM_BUSYREQ_POOL_SIZE; i++)
        s_pFreeBusyReqPool[i] = &s_BusyReqPool[i];
    s_FreeBusyReqPoolSize = NVRM_BUSYREQ_POOL_SIZE;

    // Create the RM registry mutex and initialize RM/OAL interface
    s_hPowerClientMutex = NULL;
    NV_CHECK_ERROR_CLEANUP(NvOsMutexCreate(&s_hPowerClientMutex));
    NV_CHECK_ERROR_CLEANUP(NvRmPrivOalIntfInit(hRmDeviceHandle));

    // Initialize power group control, and power gate SoC partitions
    NvRmPrivPowerGroupControlInit(hRmDeviceHandle);
    return NvSuccess;

fail:
    NvRmPrivOalIntfDeinit(hRmDeviceHandle);
    NvOsMutexDestroy(s_hPowerClientMutex);
    s_hPowerClientMutex = NULL;
    return e;
}


void NvRmPrivPowerDeinit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;

    NV_ASSERT(hRmDeviceHandle);

    // TODO: expand after clock API is completed

    // Free busy hint lists for DFS clock domains
    for (i = 0; i < NvRmDfsClockId_Num; i++)
    {
        while (s_BusyReqHeads[i].pNext != NULL)
        {
            BusyHintReq* pBusyHintReq = s_BusyReqHeads[i].pNext;
            s_BusyReqHeads[i].pNext = pBusyHintReq->pNext;
            BusyReqFree(pBusyHintReq);
        }
    }
    // Free RM power registry memory
    for (i = 0; i < pRegistry->UsedIndexRange; i++)
    {
        FreePowerClient(pRegistry->pPowerClients[i]);
    }
    NvOsFree(pRegistry->pPowerClients);
    pRegistry->pPowerClients = NULL;
    pRegistry->AvailableEntries = 0;
    pRegistry->UsedIndexRange = 0;

    // Destroy RM registry mutex and free RM/OAL interface resources
    NvRmPrivOalIntfDeinit(hRmDeviceHandle);
    NvOsMutexDestroy(s_hPowerClientMutex);
    s_hPowerClientMutex = NULL;
}

/*****************************************************************************/

NvError
NvRmPowerRegister(
    NvRmDeviceHandle hRmDeviceHandle,
    NvOsSemaphoreHandle hEventSemaphore,
    NvU32* pClientId)
{
    NvU32 FreeIndex;
    NvError error;
    NvOsSemaphoreHandle hSema = NULL;
    NvRmPowerClient* pNewClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pClientId);

    // If non-zero semaphore handle is passed, duplicate it to be avialable
    // after the call. Abort registration if non-zero handle is invalid
    if (hEventSemaphore != NULL)
    {
        error = NvOsSemaphoreClone(hEventSemaphore, &hSema);
        if (error != NvSuccess)
        {
            NV_ASSERT(!" Power Register Semaphore Clone error. ");
        }
    }

    NvOsMutexLock(s_hPowerClientMutex);

    // Find free registry entry for the new client
    for (FreeIndex = 0; FreeIndex < pRegistry->UsedIndexRange; FreeIndex++)
    {
        if (pRegistry->pPowerClients[FreeIndex] == NULL)
            break;
    }
    if (FreeIndex == pRegistry->AvailableEntries)
    {
        // If all avilable entries are used, re-size registry array
        NvU32 entries = pRegistry->AvailableEntries +
            NVRM_POWER_REGISTRY_DELTA;
        size_t s = sizeof(*pRegistry->pPowerClients) * (size_t)entries;
        NvRmPowerClient** p = NvOsRealloc(pRegistry->pPowerClients, s);
        if (p == NULL)
        {
            NvU32 old_size;

            /* fall back to NvOsAlloc */
            p = NvOsAlloc( s );
            if( p == NULL )
            {
                goto failed;
            }

            /* copy the old data, free, etc, */
            old_size = sizeof(*pRegistry->pPowerClients) *
                pRegistry->AvailableEntries;

            NvOsMemcpy( p, pRegistry->pPowerClients, old_size );
            NvOsFree( pRegistry->pPowerClients );
        }
        pRegistry->pPowerClients = p;
        pRegistry->AvailableEntries = entries;
    }
    if (FreeIndex == pRegistry->UsedIndexRange)
    {
        // If reached used index range boundary, advance it
        pRegistry->UsedIndexRange++;
    }

    // Allocate and store new client record pointer in registry (null-pointer
    // marks registry entry as free, so it's OK to store it before error check)
    pNewClient = NvOsAlloc(sizeof(*pNewClient));
    pRegistry->pPowerClients[FreeIndex] = pNewClient;
    if (pNewClient == NULL)
    {
        goto failed;
    }

    // Fill in new client entry
    pNewClient->hEventSemaphore = hSema;
    pNewClient->Event = NvRmPowerEvent_NoEvent;
    pNewClient->pVoltageReqHead = NULL;
    pNewClient->pClockReqHead = NULL;
    pNewClient->pStarvationHints = NULL;
    pNewClient->tag = *pClientId;

    /*
     * Combine index with client pointer into registration ID returned to the
     * client. This will make it a little bit more difficult for not-registered
     * clients to guess/re-use IDs
     */
    pNewClient->id = NVRM_POWER_INDEX2ID(FreeIndex, (NvU32)pClientId);
    *pClientId = pNewClient->id;

    NvOsMutexUnlock(s_hPowerClientMutex);
    return NvSuccess;

failed:
    NvOsFree(pNewClient);
    NvOsSemaphoreDestroy(hSema);
    NvOsMutexUnlock(s_hPowerClientMutex);
    return NvError_InsufficientMemory;
}

/*****************************************************************************/

void NvRmPowerUnRegister(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId)
{
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
    NvU32 ClientIndex = NVRM_POWER_ID2INDEX(ClientId);

    NV_ASSERT(hRmDeviceHandle);

    NvOsMutexLock(s_hPowerClientMutex);

    // Check if this ID was registered
    if (ClientIndex < pRegistry->UsedIndexRange)
    {
        pPowerClient = pRegistry->pPowerClients[ClientIndex];
    }
    if ((pPowerClient == NULL) || (pPowerClient->id != ClientId))
    {
        NvOsMutexUnlock(s_hPowerClientMutex);
        return;
    }

    // Cancel power requets issued by the power client to be unregistered
    CancelPowerRequests(hRmDeviceHandle, pPowerClient);

    // Free power client memory and mark the respectve registry entry as free
    FreePowerClient(pPowerClient);
    pRegistry->pPowerClients[ClientIndex] = NULL;

    // Decrement used index range as much as possible
    while ((pRegistry->UsedIndexRange > 0) &&
           (pRegistry->pPowerClients[pRegistry->UsedIndexRange - 1] == NULL))
    {
        pRegistry->UsedIndexRange--;
    }

    // Shrink registry if too much free space (keep one delta margin)
    if ((pRegistry->UsedIndexRange + 2 * NVRM_POWER_REGISTRY_DELTA) <=
         pRegistry->AvailableEntries)
    {
        NvU32 entries = pRegistry->UsedIndexRange + NVRM_POWER_REGISTRY_DELTA;
        size_t s = sizeof(*pRegistry->pPowerClients) * (size_t)entries;
        NvRmPowerClient** p = NvOsRealloc(pRegistry->pPowerClients, s);
        if (p != NULL)
        {
            pRegistry->pPowerClients = p;
            pRegistry->AvailableEntries = entries;
        }

        // FIXME: handle NvOsRealloc failure -- try NvOsAlloc instead
    }
    NvOsMutexUnlock(s_hPowerClientMutex);
}

/*****************************************************************************/

NvError NvRmPowerGetEvent(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId,
    NvRmPowerEvent* pEvent)
{
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
    NvU32 ClientIndex = NVRM_POWER_ID2INDEX(ClientId);

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pEvent);

    NvOsMutexLock(s_hPowerClientMutex);

    // Check if this ID was registered; return error otherwise
    if (ClientIndex < pRegistry->UsedIndexRange)
    {
        pPowerClient = pRegistry->pPowerClients[ClientIndex];
    }
    if ((pPowerClient == NULL) || (pPowerClient->id != ClientId))
    {
        NvOsMutexUnlock(s_hPowerClientMutex);
        return NvError_BadValue;
    }

    // Return last recorded power event and set no outstanding events
    *pEvent = pPowerClient->Event;
    pPowerClient->Event = NvRmPowerEvent_NoEvent;

    NvOsMutexUnlock(s_hPowerClientMutex);
    return NvSuccess;
}

void NvRmPowerEventNotify(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerEvent Event)
{
    NV_ASSERT(hRmDeviceHandle);

    // Just in case
    if (Event == NvRmPowerEvent_NoEvent)
         return;

    NvOsMutexLock(s_hPowerClientMutex);
    PowerEventNotify(hRmDeviceHandle, Event);
    NvOsMutexUnlock(s_hPowerClientMutex);
}

static void
PowerEventNotify(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerEvent Event)
{
    NvU32 i;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;

    NVRM_POWER_PRINTF(("%s is reported to RM clients\n",
        (Event == NvRmPowerEvent_WakeLP0)? "Wake from LP0" : "Wake from LP1"));

    // Restore clocks after LP0
    if (Event == NvRmPowerEvent_WakeLP0)
        NvRmPrivClocksResume(hRmDeviceHandle);

    // Store event for all registered clients, and signal only those, that
    // have provided valid semaphore handle; on wake from low power states
    // set power cycled indicators
    for (i = 0; i < pRegistry->UsedIndexRange; i++)
    {
        pPowerClient = pRegistry->pPowerClients[i];
        if (pPowerClient != NULL)
        {
            ModuleVoltageReq* pVoltageReq = pPowerClient->pVoltageReqHead;
            while (pVoltageReq != NULL)
            {
                if (Event == NvRmPowerEvent_WakeLP0)
                {
                    //LP0: all power groups, except AO group, are powered down
                    // when core power is down
                    if (pVoltageReq->PowerGroup != NV_POWERGROUP_AO)
                        pVoltageReq->PowerCycled = NV_TRUE;
                }
                else if (Event == NvRmPowerEvent_WakeLP1)
                {
                    // LP1: core power is preserved; but all  power groups
                    // except AO and NPG group are power gated
                    if ((pVoltageReq->PowerGroup != NV_POWERGROUP_AO) &&
                        (pVoltageReq->PowerGroup != NV_POWERGROUP_NPG))
                        pVoltageReq->PowerCycled = NV_TRUE;
                }
                pVoltageReq = pVoltageReq->pNext;
            }
            pPowerClient->Event = Event;
            if (pPowerClient->hEventSemaphore != NULL)
            {
                NvOsSemaphoreSignal(pPowerClient->hEventSemaphore);
            }
        }
    }
}

static void
RecordPowerCycle(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 PowerGroup)
{
    NvU32 i;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;

    NVRM_POWER_PRINTF(("Power Cycled partition: %d\n", PowerGroup));

    // Traverse registered clients, and mark all modules in the specified
    // power group as power cycled
    for (i = 0; i < pRegistry->UsedIndexRange; i++)
    {
        pPowerClient = pRegistry->pPowerClients[i];
        if (pPowerClient != NULL)
        {
            ModuleVoltageReq* pVoltageReq = pPowerClient->pVoltageReqHead;
            while (pVoltageReq != NULL)
            {
                if (pVoltageReq->PowerGroup == PowerGroup)
                {
                    pVoltageReq->PowerCycled = NV_TRUE;
                }
                pVoltageReq = pVoltageReq->pNext;
            }
        }
    }
}

/*****************************************************************************/

static void
ReportRmPowerState(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;
    NvRmPowerState OldRmState = NvRmPrivPowerGetState(hRmDeviceHandle);
    NvRmPowerState NewRmState = NvRmPowerState_Idle;

    // RM clients are in h/w autonomous (bypass) state if there are Power On
    // references for NPG_AUTO group only; RM clients are in active state if
    // there are Power On references for any other group
    if (s_PowerOnRefCounts[NVRM_POWERGROUP_NPG_AUTO] != 0)
        NewRmState = NvRmPowerState_AutoHw;

    for (i = 0; i < NV_POWERGROUP_MAX; i++)
    {
        if (s_PowerOnRefCounts[i] != 0)
        {
            NewRmState = NvRmPowerState_Active;
            break;
        }
    }
    if (NewRmState == OldRmState)
        return;

#if NVRM_POWER_VERBOSE_PRINTF
    NVRM_POWER_PRINTF(("RM Clients Power State: %s\n",
        ((NewRmState == NvRmPowerState_Active) ? "Active" :
         ((NewRmState == NvRmPowerState_AutoHw) ? "AutoHw" : "Idle"))));
#endif
    /*
     * Set new combined RM clients power state in the storage shared with the
     * OS adaptation layer. Check the previous state; if it was any of the low
     * power states (i.e., this is the 1st RM power state report after suspend)
     * notify all clients about wake up event.
     */
    NvRmPrivPowerSetState(hRmDeviceHandle, NewRmState);
    switch (OldRmState)
    {
        case NvRmPowerState_LP0:
            NvOsDebugPrintf("*** Wakeup from LP0 *** wake-source: 0x%x\n",
                    NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0, 0x14));
            PowerEventNotify(hRmDeviceHandle, NvRmPowerEvent_WakeLP0);
            break;
        case NvRmPowerState_LP1:
            NvOsDebugPrintf("*** Wakeup from LP1 ***\n");
            PowerEventNotify(hRmDeviceHandle, NvRmPowerEvent_WakeLP1);
            break;
        case NvRmPowerState_SkippedLP0:
            NvOsDebugPrintf("*** Wakeup after Skipped LP0 ***\n");
            // resume procedure after Skipped LP0 is the same as after LP1
            PowerEventNotify(hRmDeviceHandle, NvRmPowerEvent_WakeLP1);
            break;
        default:
            break;
    }
}

NvError
NvRmPowerGetState(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerState* pState)
{
    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pState);

    NvOsMutexLock(s_hPowerClientMutex);
    *pState = NvRmPrivPowerGetState(hRmDeviceHandle);
    NvOsMutexUnlock(s_hPowerClientMutex);
    return NvSuccess;
}

NvError
NvRmPowerVoltageControl(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmModuleID ModuleId,
    NvU32 ClientId,
    NvRmMilliVolts MinVolts,
    NvRmMilliVolts MaxVolts,
    const NvRmMilliVolts* PrefVoltageList,
    NvU32 PrefVoltageListCount,
    NvRmMilliVolts* pCurrentVolts)
{
    NvError error;
    NvU32 PowerGroup = 0;
    NvBool PowerChanged = NV_FALSE;
    NvRmModuleInstance *pInstance = NULL;
    ModuleVoltageReq* pVoltageReq = NULL;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
    NvU32 ClientIndex = NVRM_POWER_ID2INDEX(ClientId);

    /* validate the Rm Handle */
    NV_ASSERT(hRmDeviceHandle);

    // Validate module ID and get associated Power Group
    if (ModuleId == NvRmPrivModuleID_System)
    {
        PowerGroup = NVRM_POWERGROUP_NPG_AUTO;
    }
    else
    {
        error = NvRmPrivGetModuleInstance(hRmDeviceHandle, ModuleId, &pInstance);
        if (error != NvSuccess)
        {
            NV_ASSERT(!" Voltage control: Invalid module ID. ");
            return NvError_ModuleNotPresent;
        }
        PowerGroup = pInstance->DevPowerGroup;
        NV_ASSERT(PowerGroup < NV_POWERGROUP_MAX);
    }

    NvOsMutexLock(s_hPowerClientMutex);

    // Check if this ID was registered; return error otherwise
    if (ClientIndex < pRegistry->UsedIndexRange)
    {
        pPowerClient = pRegistry->pPowerClients[ClientIndex];
    }
    if ((pPowerClient == NULL) || (pPowerClient->id != ClientId))
    {
        NvOsMutexUnlock(s_hPowerClientMutex);
        return NvError_BadValue;
    }

    // Search for the previously recorded voltage request for this module
    pVoltageReq = pPowerClient->pVoltageReqHead;
    while ((pVoltageReq != NULL) && (pVoltageReq->ModuleId != ModuleId))
    {
        pVoltageReq = pVoltageReq->pNext;
    }

    // If it is a new voltage request record, allocate and fill it in,
    // otherwise just update power status. In both cases determine if
    // power requirements for the module have changed.
    if (pVoltageReq == NULL)
    {
        pVoltageReq = NvOsAlloc(sizeof(*pVoltageReq));
        if (pVoltageReq == NULL)
        {
            NvOsMutexUnlock(s_hPowerClientMutex);
            return NvError_InsufficientMemory;
        }
        // Link at head
        pVoltageReq->pNext = pPowerClient->pVoltageReqHead;
        pPowerClient->pVoltageReqHead = pVoltageReq;
        pVoltageReq->ModuleId = ModuleId;
        pVoltageReq->PowerGroup = PowerGroup;
        pVoltageReq->PowerCycled = NV_FALSE;

        // Only new power On request counts as change
        PowerChanged = (MaxVolts != NvRmVoltsOff);
    }
    else
    {
        // Only changes from On to Off or vice versa counts
        PowerChanged = (pVoltageReq->MaxVolts != MaxVolts) &&
                       ((pVoltageReq->MaxVolts == NvRmVoltsOff) ||
                        (MaxVolts == NvRmVoltsOff));
    }
    // Record new power request voltages
    pVoltageReq->MinVolts = MinVolts;
    pVoltageReq->MaxVolts = MaxVolts;

    // If module power requirements have changed, update power group reference
    // count, and execute the respective h/w power control procedure
    if (PowerChanged)
    {
        if (MaxVolts != NvRmVoltsOff)
        {
            s_PowerOnRefCounts[PowerGroup]++;
            if (s_PowerOnRefCounts[PowerGroup] == 1)
            {
                NvRmMilliVolts v =
                    NvRmPrivPowerGroupGetVoltage(hRmDeviceHandle, PowerGroup);
                if (v == NvRmVoltsOff)
                {
                    RecordPowerCycle(hRmDeviceHandle, PowerGroup);
                    NvRmPrivPowerGroupControl(hRmDeviceHandle, PowerGroup, NV_TRUE);
                }
            }
        }
        else
        {
            NV_ASSERT(s_PowerOnRefCounts[PowerGroup] != 0);
            if (s_PowerOnRefCounts[PowerGroup] == 0)
            {
                NVRM_POWER_PRINTF(("Power balance failed: module %d\n", ModuleId));
            }
            s_PowerOnRefCounts[PowerGroup]--;
            if (s_PowerOnRefCounts[PowerGroup] == 0)
            {
                NvRmPrivPowerGroupControl(hRmDeviceHandle, PowerGroup, NV_FALSE);
            }
        }
    }
    ReportRmPowerState(hRmDeviceHandle);

    // Return current voltage, unless this is the first request after module
    // was power cycled by RM; in the latter case return NvRmVoltsCycled value
    if (pCurrentVolts != NULL)
    {
        *pCurrentVolts = NvRmPrivPowerGroupGetVoltage(hRmDeviceHandle, PowerGroup);
        if (pVoltageReq->PowerCycled && (*pCurrentVolts != NvRmVoltsOff))
        {
            *pCurrentVolts = NvRmVoltsCycled;
        }
    }
    // In any case clear power cycled indicator
    pVoltageReq->PowerCycled = NV_FALSE;

    NvOsMutexUnlock(s_hPowerClientMutex);
    return NvSuccess;
}

void
NvRmListPowerAwareModules(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32* pListSize,
    NvRmModuleID* pIdList,
    NvBool* pActiveList)
{
    NvBool active;
    NvU32 i, ModulesNum, ActiveNum;
    ModuleVoltageReq* pVoltageReq = NULL;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;


    /* validate the Rm Handle */
    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pListSize);
    NV_ASSERT(((*pListSize) == 0) || (pIdList && pActiveList));

    NvOsMutexLock(s_hPowerClientMutex);

    // Count power aware modules, fill in the list
    for (i = ModulesNum = ActiveNum = 0; i < pRegistry->UsedIndexRange; i++)
    {
        pPowerClient = pRegistry->pPowerClients[i];
        if (pPowerClient)
        {
            pVoltageReq = pPowerClient->pVoltageReqHead;
            while (pVoltageReq != NULL)
            {
                ModulesNum++;
                active = (pVoltageReq->MaxVolts != NvRmVoltsOff);
                ActiveNum += active ? 1 : 0;
                if (*pListSize >= ModulesNum)
                {
                    *(pIdList++) = pVoltageReq->ModuleId;
                    *(pActiveList++) = active;
                }
                pVoltageReq = pVoltageReq->pNext;
            }
        }
    }
    // Report number of found modules
    if ((*pListSize == 0) || (*pListSize > ModulesNum))
    {
        *pListSize = ModulesNum;
    }
    // Total refcounts must be = number of active modules
    for (i = 0; i <= NV_POWERGROUP_MAX; i++)
        ActiveNum -= s_PowerOnRefCounts[i];
    NV_ASSERT(ActiveNum == 0);

    NvOsMutexUnlock(s_hPowerClientMutex);
}

/*****************************************************************************/

static NvError
RecordStarvationHints(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPowerClient* pPowerClient,
    const NvRmDfsStarvationHint* pMultiHint,
    NvU32 NumHints)
{
    NvU32 i;
    NvBool HintChanged = NV_FALSE;

    for (i = 0; i < NumHints; i++)
    {
        NvRmDfsClockId ClockId = pMultiHint[i].ClockId;
        NvBool Starving = pMultiHint[i].Starving;
        NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));

        /*
         * If this is the first starvation hint, allocate hints array and fill
         * it in. Otherwise, just update starvation hint status. In both cases
         * determine if starvation hint for clock domain has changed.
         */
        if (pPowerClient->pStarvationHints == NULL)
        {
            size_t s = sizeof(NvBool) * (size_t)NvRmDfsClockId_Num;
            NvBool* p = NvOsAlloc(s);
            if (p == NULL)
            {
                return NvError_InsufficientMemory;
            }
            NvOsMemset(p, 0, s);
            pPowerClient->pStarvationHints = p;

            // Only new Satrvation On hint counts as change
            HintChanged = Starving;
        }
        else
        {
            // Only changes from On to Off or vice versa counts
            HintChanged = (pPowerClient->pStarvationHints[ClockId] != Starving);
        }
        pPowerClient->pStarvationHints[ClockId] = Starving;

        // If hint has changed, update clock domain starvation reference count
        // (hint against CPU, or AVP, or VDE is automatically applied to EMC)
        if (HintChanged)
        {
            if (Starving)
            {
                if ((ClockId == NvRmDfsClockId_Cpu) ||
                    (ClockId == NvRmDfsClockId_Avp) ||
                    (ClockId == NvRmDfsClockId_Vpipe))
                {
                    s_StarveOnRefCounts[NvRmDfsClockId_Emc]++;
                }
                s_StarveOnRefCounts[ClockId]++;
            }
            else
            {
                if ((ClockId == NvRmDfsClockId_Cpu) ||
                    (ClockId == NvRmDfsClockId_Avp) ||
                    (ClockId == NvRmDfsClockId_Vpipe))
                {
                    NV_ASSERT(s_StarveOnRefCounts[NvRmDfsClockId_Emc] != 0);
                    s_StarveOnRefCounts[NvRmDfsClockId_Emc]--;
                }
                NV_ASSERT(s_StarveOnRefCounts[ClockId] != 0);
                s_StarveOnRefCounts[ClockId]--;
            }
        }
    }
    return NvSuccess;
}

NvBool NvRmPrivDfsIsStarving(NvRmDfsClockId ClockId)
{
    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));
    // Boolean read - no need for lock
    return (s_StarveOnRefCounts[ClockId] != 0);
}

NvError
NvRmPowerStarvationHintMulti(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId,
    const NvRmDfsStarvationHint* pMultiHint,
    NvU32 NumHints)
{
    NvError error;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
    NvU32 ClientIndex = NVRM_POWER_ID2INDEX(ClientId);

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pMultiHint && NumHints);

    /* Do nothing on platforms where there is no freq scaling like QT and FPGA */
    if (NvRmPrivGetExecPlatform(hRmDeviceHandle) != ExecPlatform_Soc)
    {
        return NvSuccess;
    }
    // Do nothing if DFS is disabled, and therefore all clocks are maxed anyway
    if (NvRmDfsGetState(hRmDeviceHandle) <= NvRmDfsRunState_Disabled)
    {
        return NvSuccess;
    }

    NvOsMutexLock(s_hPowerClientMutex);

    // Check if this client ID was registered; return error otherwise
    if (ClientIndex < pRegistry->UsedIndexRange)
    {
        pPowerClient = pRegistry->pPowerClients[ClientIndex];
    }
    if ((pPowerClient == NULL) || (pPowerClient->id != ClientId))
    {
        NvOsMutexUnlock(s_hPowerClientMutex);
        return NvError_BadValue;
    }
    // Add new stravtion hint
    error = RecordStarvationHints(
        hRmDeviceHandle, pPowerClient, pMultiHint, NumHints);

    NvOsMutexUnlock(s_hPowerClientMutex);

    if (error == NvSuccess)
        NvRmPrivStarvationHintPrintf(
            ClientIndex, pPowerClient->tag, pMultiHint, NumHints);
    return error;
}

NvError
NvRmPowerStarvationHint (
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsClockId ClockId,
    NvU32 ClientId,
    NvBool Starving)
{
    NvRmDfsStarvationHint StarvationHint;

    // Pack hit record
    StarvationHint.ClockId = ClockId;
    StarvationHint.Starving = Starving;

    return NvRmPowerStarvationHintMulti(
        hRmDeviceHandle, ClientId, &StarvationHint, 1);
}

/*****************************************************************************/

static BusyHintReq* BusyReqAlloc(void)
{
    if (s_FreeBusyReqPoolSize != 0)
        return s_pFreeBusyReqPool[--s_FreeBusyReqPoolSize];
    else
    {
        NV_ASSERT(!"Busy pool size is too small");
        return NvOsAlloc(sizeof(BusyHintReq));
    }
}

static void BusyReqFree(BusyHintReq* pBusyHintReq)
{
    if ((pBusyHintReq >= &s_BusyReqPool[0]) &&
        (pBusyHintReq < &s_BusyReqPool[NVRM_BUSYREQ_POOL_SIZE]))
    {
        NV_ASSERT(s_FreeBusyReqPoolSize < NVRM_BUSYREQ_POOL_SIZE);
        s_pFreeBusyReqPool[s_FreeBusyReqPoolSize++] = pBusyHintReq;
        return;
    }
    NvOsFree(pBusyHintReq);
}

static void CancelBusyHints(NvRmDfsClockId ClockId, NvU32 ClientId)
{
    BusyHintReq* pBusyHintReq = NULL;
    BusyHintReq* pBusyHintNext = NULL;

    /*
     * Traverse busy hints list, starting from the head and looking for hints
     * reported by the specified client. Remove found hint nodes on the way.
     */
    pBusyHintReq = &s_BusyReqHeads[ClockId];
    if (pBusyHintReq->ClientId == ClientId)
    {
        pBusyHintReq->IntervalMs = 0;   // Keep head for just one more sample
    }
    while (pBusyHintReq != NULL)
    {
        pBusyHintNext = pBusyHintReq->pNext;
        if ((pBusyHintNext != NULL) && (pBusyHintNext->ClientId == ClientId))
        {
            pBusyHintReq->pNext = pBusyHintNext->pNext;
            BusyReqFree(pBusyHintNext);
            continue;
        }
        pBusyHintReq = pBusyHintNext;
    }
}

static void PurgeBusyHints(NvRmDfsClockId ClockId, NvU32 msec)
{
    static NvU32 s_LastPurgeMs = 0;
    BusyHintReq* pBusyHintReq = NULL;
    BusyHintReq* pBusyHintNext = NULL;

    if ((msec - s_LastPurgeMs) <= NVRM_DFS_BUSY_PURGE_MS)
        return;

    /*
     * If time to purge the busy hints list, traverse it starting from the
     * head and looking for the expired hints. Remove found nodes on the way.
     */
    pBusyHintReq = &s_BusyReqHeads[ClockId];
    while (pBusyHintReq != NULL)
    {
        pBusyHintNext = pBusyHintReq->pNext;
        if ( (pBusyHintNext != NULL) &&
             (pBusyHintNext->IntervalMs < (msec - pBusyHintNext->StartTimeMs)) )
        {
            pBusyHintReq->pNext = pBusyHintNext->pNext;
            BusyReqFree(pBusyHintNext);
            continue;
        }
        pBusyHintReq = pBusyHintNext;
    }
    s_LastPurgeMs = msec;
}

static NvError
RecordBusyHints(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId,
    const NvRmDfsBusyHint* pMultiHint,
    NvU32 NumHints,
    NvBool* pSignalDfs)
{
    NvU32 i;
    NvRmFreqKHz MaxKHz;
    BusyHintReq* pInsert = NULL;
    BusyHintReq* pBusyHintReq = NULL;
    NvU32 msec = NvOsGetTimeMS();

    *pSignalDfs = NV_FALSE;

    for (i = 0; i < NumHints; i++)
    {
        NvRmDfsClockId ClockId = pMultiHint[i].ClockId;
        NvRmFreqKHz BoostKHz = pMultiHint[i].BoostKHz;
        NvU32 BoostDurationMs = pMultiHint[i].BoostDurationMs;
        NvBool BusyPulseMode = pMultiHint[i].BusyAttribute;
        NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));

        // Clip requested boost frequency to domain maximum
        MaxKHz = NvRmPrivDfsGetMaxKHz(ClockId);
        if (BoostKHz > MaxKHz)
        {
            BoostKHz = MaxKHz;
        }

        // Cancel all hints sent by this client if it is no longer busy;
        // signal DFS boost removed
        if (BoostKHz == 0)
        {
            CancelBusyHints(ClockId, ClientId);
            *pSignalDfs = NV_TRUE;
            continue;
        }

        // Update maximum boost frequency stored in the head entry; signal DFS
        // boost increase
        if (s_BusyReqHeads[ClockId].BoostKHz < BoostKHz)
        {
            s_BusyReqHeads[ClockId].BoostKHz = BoostKHz;
            s_BusyReqHeads[ClockId].IntervalMs = BoostDurationMs;
            s_BusyReqHeads[ClockId].BusyPulseMode = BusyPulseMode;
            s_BusyReqHeads[ClockId].StartTimeMs = msec;
            s_BusyReqHeads[ClockId].ClientId = ClientId;
            *pSignalDfs = NV_TRUE;
        }

        /*
         * If it is a short spike no need to store the record, as maximum boost
         * has been already updated. Otherwise, insert new busy record into the
         * busy hints list in descending order of requested boost frequencies
         */
        if (BoostDurationMs > NVRM_DFS_BUSY_MIN_MS)
        {
            for (pInsert = &s_BusyReqHeads[ClockId] ;;)
            {
                if ((pInsert->pNext == NULL) ||
                    (pInsert->pNext->BoostKHz < BoostKHz))
                {
                    // Allocate and initialize new boost hint record
                    pBusyHintReq = BusyReqAlloc();
                    if (pBusyHintReq == NULL)
                    {
                        return NvError_InsufficientMemory;
                    }
                    pBusyHintReq->BoostKHz = BoostKHz;
                    pBusyHintReq->IntervalMs = BoostDurationMs;
                    pBusyHintReq->BusyPulseMode = BusyPulseMode;
                    pBusyHintReq->StartTimeMs = msec;
                    pBusyHintReq->ClientId = ClientId;
                    pBusyHintReq->pNext = pInsert->pNext;
                    pInsert->pNext = pBusyHintReq;
                    break;
                }
                else if (pInsert->pNext->BoostKHz == BoostKHz)
                {
                    // Combine hints from the same client with the same
                    // boost level and pulse mode
                    if ((pInsert->pNext->ClientId == ClientId) &&
                        (pInsert->pNext->BusyPulseMode == BusyPulseMode))
                    {
                        NvU32 t = msec - pInsert->pNext->StartTimeMs;
                        if ((BoostDurationMs > pInsert->pNext->IntervalMs) ||
                            (t > (pInsert->pNext->IntervalMs - BoostDurationMs)))
                        {
                            pInsert->pNext->StartTimeMs = msec;
                            pInsert->pNext->IntervalMs = BoostDurationMs;
                        }
                        break;
                    }
                }
                pInsert = pInsert->pNext;
            }
            PurgeBusyHints(ClockId, msec);  // Purge the list once in a while
        }
    }
    return NvSuccess;
}

void NvRmPrivDfsGetBusyHint(
    NvRmDfsClockId ClockId,
    NvRmFreqKHz* pBusyKHz,
    NvBool* pBusyPulseMode,
    NvU32* pBusyExpireMs)
{
    NvU32 msec;
    BusyHintReq* pBusyHintReq;

    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));

    // Boolean read - no need for lock - fast path for most common case
    // when no busy hints are recoeded
    if (s_BusyReqHeads[ClockId].BoostKHz == 0)
    {
        *pBusyKHz = 0;
        *pBusyPulseMode = NV_FALSE;
        *pBusyExpireMs = 0;
        return;
    }
    msec = NvOsGetTimeMS();

    NvOsMutexLock(s_hPowerClientMutex);
    /*
     * Get boost frequency from the head. Then, traverse busy hints list,
     * starting from the head looking for max non-expired frequency boost.
     * Remove expired nodes on the way. Update head boost frequency.
     */
    pBusyHintReq = &s_BusyReqHeads[ClockId];
    *pBusyKHz = pBusyHintReq->BoostKHz;
    *pBusyPulseMode = pBusyHintReq->BusyPulseMode;
    *pBusyExpireMs = 0;     // assume head hint has already expired
    if (pBusyHintReq->IntervalMs == NV_WAIT_INFINITE)
        *pBusyExpireMs = NV_WAIT_INFINITE;  // head hint until canceled
    else if (pBusyHintReq->IntervalMs >= (msec - pBusyHintReq->StartTimeMs))
        *pBusyExpireMs =                    // non-expired head hint
        pBusyHintReq->IntervalMs - (msec - pBusyHintReq->StartTimeMs);

    pBusyHintReq = pBusyHintReq->pNext;
    while (pBusyHintReq != NULL)
    {
        BusyHintReq* p;
        if (pBusyHintReq->IntervalMs >= (msec - pBusyHintReq->StartTimeMs))
        {
            break;
        }
        p = pBusyHintReq;
        pBusyHintReq = pBusyHintReq->pNext;
        BusyReqFree(p);
    }
    if (pBusyHintReq)
    {
        s_BusyReqHeads[ClockId] = *pBusyHintReq;
        s_BusyReqHeads[ClockId].pNext = pBusyHintReq;
    }
    else
        NvOsMemset(&s_BusyReqHeads[ClockId], 0, sizeof(s_BusyReqHeads[ClockId]));
    NvOsMutexUnlock(s_hPowerClientMutex);
}

NvError
NvRmPowerBusyHintMulti(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 ClientId,
    const NvRmDfsBusyHint* pMultiHint,
    NvU32 NumHints,
    NvRmDfsBusyHintSyncMode Mode)
{
    NvError error;
    NvRmDfsRunState DfsState;
    NvBool SignalDfs = NV_FALSE;
    NvRmPowerClient* pPowerClient = NULL;
    NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
    NvU32 ClientIndex = NVRM_POWER_ID2INDEX(ClientId);

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pMultiHint && NumHints);
    DfsState = NvRmDfsGetState(hRmDeviceHandle);

    /* Do nothing on platforms where there is no freq scaling like QT and FPGA */
    if (NvRmPrivGetExecPlatform(hRmDeviceHandle) != ExecPlatform_Soc)
    {
        return NvSuccess;
    }
    // Do nothing if DFS is disabled, and therefore all clocks are maxed anyway
    if (DfsState <= NvRmDfsRunState_Disabled)
    {
        return NvSuccess;    // error if disabled
    }

    NvOsMutexLock(s_hPowerClientMutex);

    // Check if this client ID was registered; return error otherwise
    if (ClientIndex < pRegistry->UsedIndexRange)
    {
        pPowerClient = pRegistry->pPowerClients[ClientIndex];
    }
    if ((pPowerClient == NULL) || (pPowerClient->id != ClientId))
    {
        NvOsMutexUnlock(s_hPowerClientMutex);
        return NvError_BadValue;
    }
    // Add new busy hint record to the list
    error = RecordBusyHints(
        hRmDeviceHandle, ClientId, pMultiHint, NumHints, &SignalDfs);

    NvOsMutexUnlock(s_hPowerClientMutex);

    if (error == NvSuccess)
    {
        NvRmPrivBusyHintPrintf(
            ClientIndex, pPowerClient->tag, pMultiHint, NumHints);
        if (SignalDfs && (DfsState > NvRmDfsRunState_Stopped))
        {
            // Signal DFS clock control provided DFS is running
            NvRmPrivDfsSignal(Mode);
        }
    }
    return error;
}

NvError
NvRmPowerBusyHint (
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsClockId ClockId,
    NvU32 ClientId,
    NvU32 BoostDurationMs,
    NvRmFreqKHz BoostKHz)
{
    NvRmDfsBusyHint BusyHint;

    // Pack hint record
    BusyHint.ClockId = ClockId;
    BusyHint.BoostKHz = BoostKHz;
    BusyHint.BoostDurationMs = BoostDurationMs;
    BusyHint.BusyAttribute = NV_FALSE;

    return NvRmPowerBusyHintMulti(hRmDeviceHandle, ClientId, &BusyHint, 1,
                                  NvRmDfsBusyHintSyncMode_Async);
}

/*****************************************************************************/

NvError
NvRmPowerActivityHint (
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmModuleID ModuleId,
    NvU32 ClientId,
    NvU32 ActivityDurationMs)
{
    /* validate the Rm Handle */
    NV_ASSERT( hRmDeviceHandle );

    return NvError_NotImplemented;
}

NvError
NvRmKernelPowerSuspend( NvRmDeviceHandle hRmDeviceHandle )
{
    NvOdmSocPowerState state = NvRmPowerLowestStateGet();

    if (state ==  NvOdmSocPowerState_Suspend)
        NvRmPrivPowerGroupSuspend(hRmDeviceHandle);

#if NVRM_POWER_DEBUG_SUSPEND_ENTRY
    NvOsMutexLock(s_hPowerClientMutex);
    {
        NvU32 i;
        ModuleVoltageReq* pVoltageReq = NULL;
        NvRmPowerClient* pPowerClient = NULL;
        NvRmPowerRegistry* pRegistry = &s_PowerRegistry;
        NvRmPowerState s = NvRmPrivPowerGetState(hRmDeviceHandle);

        // Report combined RM power stste and active modules
        NvOsDebugPrintf("RM power state before suspend: %s (%d)\n",
           ((s == NvRmPowerState_Active) ? "Active" :
            ((s == NvRmPowerState_AutoHw) ? "AutoHw" : "Idle")), s);
        if (s == NvRmPowerState_Active)
        {
            for (i = 0; i < pRegistry->UsedIndexRange; i++)
            {
                pPowerClient = pRegistry->pPowerClients[i];
                if (pPowerClient)
                {
                    pVoltageReq = pPowerClient->pVoltageReqHead;
                    while (pVoltageReq != NULL)
                    {
                        if (pVoltageReq->MaxVolts != NvRmVoltsOff)
                        {
                            // could also set some bad e = NvError_Bad???
                            NvOsDebugPrintf("Active Module: 0x%x\n",
                                                pVoltageReq->ModuleId);
                        }
                        pVoltageReq = pVoltageReq->pNext;
                    }
                }
            }
        }
    }
    NvOsMutexUnlock(s_hPowerClientMutex);
#endif

    return NvSuccess;
}

NvError
NvRmKernelPowerResume( NvRmDeviceHandle hRmDeviceHandle )
{
    NvOdmSocPowerState state = NvRmPowerLowestStateGet();

    NvOsMutexLock(s_hPowerClientMutex);
    ReportRmPowerState(hRmDeviceHandle);
    NvOsMutexUnlock(s_hPowerClientMutex);

    if (state ==  NvOdmSocPowerState_Suspend)
        NvRmPrivPowerGroupResume(hRmDeviceHandle);
    return NvSuccess;
}



