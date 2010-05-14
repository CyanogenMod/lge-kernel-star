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
 *           Power Resource manager API shared with OS adaptation layer</b>
 *
 * @b Description: Implements private HW interface shared by the NvRM Power
 *  manager and OS adaptation layer (OAL). 
 * 
 */

#include "nvrm_power.h"
#include "nvrm_clocks.h"
#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "ap15rm_private.h"
#include "nvrm_structure.h"
#include "ap15/arapb_misc.h"
#include "ap15rm_pmc_scratch_map.h" 
#include "common/nvrm_chiplib.h"
#include "nvassert.h"

/*****************************************************************************/

/*
 * Macros for power state register field access.
 * AP15+: a dedicated bits in PMC scratch register 0 are allocated for RM power
 * state fields.
 */
#define SET_POWER_FLD_AP15(rm, FieldName, FieldValue) \
   do \
   { \
       if (!NvRmIsSimulation())\
       {\
           NvU32 RegValue; \
           NvU32 RegOffset = APBDEV_PMC_SCRATCH0_0; \
           NvOsSpinMutexLock(s_hPmcScratchMutex); \
           RegValue = NV_REGR(rm, NvRmModuleID_Pmif, 0, RegOffset); \
           RegValue = NV_FLD_SET_DRF_NUM(\
               APBDEV_PMC, SCRATCH0, FieldName, FieldValue, RegValue); \
           NV_REGW(rm, NvRmModuleID_Pmif, 0, RegOffset, RegValue); \
           NvOsSpinMutexUnlock(s_hPmcScratchMutex); \
       }\
   } while (0)

#define GET_POWER_FLD_AP15(rm, FieldName) \
           NV_DRF_VAL(APBDEV_PMC, SCRATCH0, FieldName, \
                     (NV_REGR(rm, NvRmModuleID_Pmif, 0, APBDEV_PMC_SCRATCH0_0)));

/*****************************************************************************/

// Mutex for thread-safe access to PMC scratch fields
static NvOsSpinMutexHandle s_hPmcScratchMutex = NULL;

// Pointer to LP2 Time storage
static NvUPtr s_pLp2Time = 0;

NvError NvRmPrivOalIntfInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvError e;
    NV_ASSERT(hRmDeviceHandle);

    // Create PMC scratch register access mutex
    s_pLp2Time = 0;
    s_hPmcScratchMutex = NULL;
    NV_CHECK_ERROR_CLEANUP(NvOsSpinMutexCreate(&s_hPmcScratchMutex));

    // Clear DFS flags; other fields initialized by OAL and preserved by RM
    SET_POWER_FLD_AP15(hRmDeviceHandle, RM_DFS_FLAG, 0);
    return NvSuccess;

fail:
    NvRmPrivOalIntfDeinit(hRmDeviceHandle);
    return e;
}

void NvRmPrivOalIntfDeinit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvOsSpinMutexDestroy(s_hPmcScratchMutex);
    s_hPmcScratchMutex = NULL;
}

/*****************************************************************************/

/*
 * Write synchronization with the OAL is responsibility of the OAL, i.e., OAL
 * calls set state function only on entry to LPx state in single-thread
 * environment
 */
void
NvRmPrivPowerSetState(NvRmDeviceHandle hRmDeviceHandle, NvRmPowerState RmState)
{
    SET_POWER_FLD_AP15(hRmDeviceHandle, RM_PWR_STATE, RmState);
}

NvRmPowerState
NvRmPrivPowerGetState(NvRmDeviceHandle hRmDeviceHandle)
{
    NvRmPowerState state = 0;

    if (!NvRmIsSimulation())
    {
        state = GET_POWER_FLD_AP15(hRmDeviceHandle, RM_PWR_STATE);
    }
    return state;
}

/*****************************************************************************/

/*
 * Read synchronization with the OAL is responsibility of the OAL, i.e., OAL
 * calls get flags function only on entry to LPx state in single-thread
 * environment
 */
NvU32
NvRmPrivGetDfsFlags(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 Flags = 0;
    if (!NvRmIsSimulation())
    {
        Flags = GET_POWER_FLD_AP15(hRmDeviceHandle, RM_DFS_FLAG);
        if (!(Flags & NvRmDfsStatusFlags_StopPllA0))
            Flags &= (~NvRmDfsStatusFlags_StopPllP0); // PLLA input from PLLP
    }
    return Flags;
}

void
NvRmPrivUpdateDfsPauseFlag(
    NvRmDeviceHandle hRmDeviceHandle,
    NvBool Pause)
{
    if (!NvRmIsSimulation())
    {
        NvU32 RegValue;
        NvU32 RegOffset = APBDEV_PMC_SCRATCH0_0;
        NvU32 mask = (NvRmDfsStatusFlags_Pause <<
                      NV_FIELD_SHIFT(APBDEV_PMC_SCRATCH0_0_RM_DFS_FLAG_RANGE));

        NvOsSpinMutexLock(s_hPmcScratchMutex);

        RegValue = NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset);
        if (Pause)
            RegValue |= mask;
        else
            RegValue &= ~mask;
        NV_REGW(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset, RegValue);

        NvOsSpinMutexUnlock(s_hPmcScratchMutex);
    }
}

void
NvRmPrivPllRefUpdate(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmPllReference* pPllRef,
    NvBool Increment)
{
#if !NV_OAL
    NvU32 RegValue, mask;
    NvU32 RegOffset = APBDEV_PMC_SCRATCH0_0;

    // Do nothing for platforms other, than SoC
    if (NvRmPrivGetExecPlatform(hRmDeviceHandle) != ExecPlatform_Soc)
        return;

    NV_ASSERT(pPllRef);
    NV_ASSERT(pPllRef->StopFlag <=
              NV_FIELD_MASK(APBDEV_PMC_SCRATCH0_0_RM_DFS_FLAG_RANGE));
    mask = (pPllRef->StopFlag <<
            NV_FIELD_SHIFT(APBDEV_PMC_SCRATCH0_0_RM_DFS_FLAG_RANGE));

    NvOsSpinMutexLock(s_hPmcScratchMutex);

    if (Increment)
    {
        pPllRef->ReferenceCnt++;
        if (pPllRef->ReferenceCnt == 1)
        {
            RegValue = (~mask) &
                (NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset));
            NV_REGW(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset, RegValue);
        }
    }
    else
    {
        NV_ASSERT(pPllRef->ReferenceCnt);
        if (pPllRef->ReferenceCnt)
        {
            pPllRef->ReferenceCnt--;
            if (pPllRef->ReferenceCnt == 0)
            {
                RegValue = mask |
                    (NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset));
                NV_REGW(
                    hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset, RegValue);
            }
        }
    }
    NvOsSpinMutexUnlock(s_hPmcScratchMutex);
#endif
}

/*****************************************************************************/

/*
 * Write synchronization with the OAL is responsibility of the OAL, i.e., OAL
 * calls set state function only in OEMInit() single-thread environment
 */
void
NvRmPrivSetDownloadTransport(
    NvRmDeviceHandle hRmDeviceHandle,
    NvOdmDownloadTransport Transport)
{
    NV_ASSERT(Transport <=
              NV_FIELD_MASK(APBDEV_PMC_SCRATCH0_0_RM_LOAD_TRANSPORT_RANGE));
    SET_POWER_FLD_AP15(hRmDeviceHandle, RM_LOAD_TRANSPORT, Transport);
}

NvOdmDownloadTransport
NvRmPrivGetDownloadTransport(NvRmDeviceHandle hRmDeviceHandle)
{
    NvOdmDownloadTransport Transport = NvOdmDownloadTransport_None;
    if (!NvRmIsSimulation())
    {
        Transport = GET_POWER_FLD_AP15(hRmDeviceHandle, RM_LOAD_TRANSPORT);
    }
    return Transport;
}

/*****************************************************************************/

void NvRmPrivAp15IoPowerDetectReset(NvRmDeviceHandle hRmDeviceHandle)
{
    if (!NvRmIsSimulation())
    {
        NvU32 RegValue;
        NvU32 RegOffset = APBDEV_PMC_SCRATCH0_0;
        NvOsSpinMutexLock(s_hPmcScratchMutex);

        RegValue =
            NV_REGR(hRmDeviceHandle, NvRmModuleID_Pmif, 0, RegOffset);
        RegValue = NV_FLD_SET_DRF_NUM(
            APBDEV_PMC, SCRATCH0, RST_PWR_DET, 1, RegValue);
        NV_REGW(hRmDeviceHandle,
                NvRmModuleID_Pmif, 0, RegOffset, RegValue);
        RegValue = NV_FLD_SET_DRF_NUM(
            APBDEV_PMC, SCRATCH0, RST_PWR_DET, 0, RegValue);
        NV_REGW(hRmDeviceHandle,
                NvRmModuleID_Pmif, 0, RegOffset, RegValue);

        NvOsSpinMutexUnlock(s_hPmcScratchMutex);
    }
}

/*****************************************************************************/

/*
 * PMC scratch register 21 is dedicated as LP2 time storage.
 * Write synchronization with the OAL is responsibility of the OAL, i.e., OAL
 * calls access this register only in single-thread environment.
 */
void
NvRmPrivSetLp2TimeUS(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 TimeUS)
{
    if (NvRmIsSimulation())
        return;

    {
        if (s_pLp2Time == 0)
        {
            NvRmModuleTable* tbl = NvRmPrivGetModuleTable(hRmDeviceHandle);
            s_pLp2Time = ((NvUPtr)(tbl->ModInst + 
                tbl->Modules[NvRmModuleID_Pmif].Index)->VirtAddr) +
                APBDEV_PMC_SCRATCH21_0;
        }
        NV_WRITE32(s_pLp2Time, TimeUS);
    }
}

NvU32
NvRmPrivGetLp2TimeUS(NvRmDeviceHandle hRmDeviceHandle)
{
    if (NvRmIsSimulation())
        return 0;

    {
        if (s_pLp2Time == 0)
        {
            NvRmModuleTable* tbl = NvRmPrivGetModuleTable(hRmDeviceHandle);
            s_pLp2Time = ((NvUPtr)(tbl->ModInst + 
                tbl->Modules[NvRmModuleID_Pmif].Index)->VirtAddr) +
                APBDEV_PMC_SCRATCH21_0;
        }
        return NV_READ32(s_pLp2Time);
    }
}

/*****************************************************************************/

