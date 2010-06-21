/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
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

#include "nvrm_init.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "ap15/aremc.h"
#include "ap15/armc.h"
#include "ap15/arapb_misc.h"
#include "ap15rm_private.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_memctrl.h"
#include "nvrm_clocks.h"
#include "nvrm_structure.h"
#include "nvrm_arm_cp.h"
#include "nvrm_processor.h"

//define obs_struct
typedef struct ObsInfoRec
{
  NvRmModuleID modSelect;
  NvU32 partSelect;
} ObsInfo;

#define OBS_INFO_FIELD(modID, partition) \
    { \
    NvRmModuleID_##modID, \
    APB_MISC_GP_OBSCTRL_0_OBS_PART_SEL_##partition \
    }

// static table correspond to enum NvRmModuleID in \include\nvrm_module.idl
// Expand this table to add more moduleID - partition map entries.
static const ObsInfo ObsInfoTable[] =
{
  OBS_INFO_FIELD(Cpu, CPU),
  OBS_INFO_FIELD(Display, DIS),
  OBS_INFO_FIELD(Csi, DIS),
  OBS_INFO_FIELD(Hdmi, DIS),
  OBS_INFO_FIELD(Tvo, DIS),
  OBS_INFO_FIELD(Dsi, DIS),
  OBS_INFO_FIELD(2D, GR),
  OBS_INFO_FIELD(Fuse, GR),
  OBS_INFO_FIELD(Vde, VDE),
  OBS_INFO_FIELD(Isp, VE)
};

static const NvU32 ObsInfoTableSize =
    NV_ARRAY_SIZE(ObsInfoTable);


static void
McStatAp1x_Start(
        NvRmDeviceHandle rm,
        NvU32 client_id_0,
        NvU32 client_id_1,
        NvU32 llc_client_id)
{
    NvU32 emc_ctrl =
      (AREMC_STAT_CONTROL_MODE_BANDWIDTH << AREMC_STAT_CONTROL_MODE_SHIFT) |
      (AREMC_STAT_CONTROL_EVENT_QUALIFIED << AREMC_STAT_CONTROL_EVENT_SHIFT) |
      (AREMC_STAT_CONTROL_CLIENT_TYPE_CMCR <<
         AREMC_STAT_CONTROL_CLIENT_TYPE_SHIFT) |  // default is CMC Read client
      (AREMC_STAT_CONTROL_FILTER_CLIENT_ENABLE <<
         AREMC_STAT_CONTROL_FILTER_CLIENT_SHIFT) |
      (AREMC_STAT_CONTROL_FILTER_ADDR_DISABLE <<
         AREMC_STAT_CONTROL_FILTER_ADDR_SHIFT);

    NvU32 mc_filter_client_0 = (ARMC_STAT_CONTROL_FILTER_CLIENT_ENABLE <<
                   ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);

    NvU32 mc_filter_client_1 = (ARMC_STAT_CONTROL_FILTER_CLIENT_ENABLE <<
                   ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);

    if (client_id_0 == 0xffffffff)
    {
        mc_filter_client_0 = (ARMC_STAT_CONTROL_FILTER_CLIENT_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);
        client_id_0 = 0;
    }

    if (client_id_1 == 0xffffffff)
    {
        mc_filter_client_1 = (ARMC_STAT_CONTROL_FILTER_CLIENT_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);
        client_id_1 = 0;
    }

    if(llc_client_id == 1)
        emc_ctrl |= AREMC_STAT_CONTROL_CLIENT_TYPE_MPCORER <<
                    AREMC_STAT_CONTROL_CLIENT_TYPE_SHIFT;
                    // overwrite with MPCore read
        NV_REGW(rm, NvRmPrivModuleID_ExternalMemoryController,
                0, EMC_STAT_CONTROL_0,
                NV_DRF_DEF(EMC, STAT_CONTROL, LLMC_GATHER,DISABLE));
        NV_REGW(rm, NvRmPrivModuleID_ExternalMemoryController,
                0, EMC_STAT_LLMC_CLOCK_LIMIT_0, 0xffffffff);
        NV_REGW(rm, NvRmPrivModuleID_ExternalMemoryController,
                0, EMC_STAT_LLMC_CONTROL_0_0, emc_ctrl);
        NV_REGW(rm, NvRmPrivModuleID_ExternalMemoryController,
                0, EMC_STAT_CONTROL_0,
                NV_DRF_DEF(EMC, STAT_CONTROL, LLMC_GATHER, CLEAR));
        NV_REGW(rm, NvRmPrivModuleID_ExternalMemoryController,
                0, EMC_STAT_CONTROL_0,
                NV_DRF_DEF(EMC, STAT_CONTROL, LLMC_GATHER, ENABLE));
        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_CONTROL_0,
                NV_DRF_DEF(MC, STAT_CONTROL, EMC_GATHER, DISABLE));
        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_EMC_CLOCK_LIMIT_0, 0xffffffff);
        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_EMC_CONTROL_0_0,
                (ARMC_STAT_CONTROL_MODE_BANDWIDTH <<
                   ARMC_STAT_CONTROL_MODE_SHIFT) |
                (client_id_0 << ARMC_STAT_CONTROL_CLIENT_ID_SHIFT) |
                (ARMC_STAT_CONTROL_EVENT_QUALIFIED <<
                   ARMC_STAT_CONTROL_EVENT_SHIFT) |
                mc_filter_client_0 |
                (ARMC_STAT_CONTROL_FILTER_ADDR_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_ADDR_SHIFT) |
                (ARMC_STAT_CONTROL_FILTER_PRI_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_PRI_SHIFT) |
                (ARMC_STAT_CONTROL_FILTER_COALESCED_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_COALESCED_SHIFT));
        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_EMC_CONTROL_1_0,
                (ARMC_STAT_CONTROL_MODE_BANDWIDTH <<
                   ARMC_STAT_CONTROL_MODE_SHIFT) |
                (client_id_1 << ARMC_STAT_CONTROL_CLIENT_ID_SHIFT) |
                (ARMC_STAT_CONTROL_EVENT_QUALIFIED <<
                   ARMC_STAT_CONTROL_EVENT_SHIFT) |
                mc_filter_client_1 |
                (ARMC_STAT_CONTROL_FILTER_ADDR_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_ADDR_SHIFT) |
                (ARMC_STAT_CONTROL_FILTER_PRI_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_PRI_SHIFT) |
                (ARMC_STAT_CONTROL_FILTER_COALESCED_DISABLE <<
                   ARMC_STAT_CONTROL_FILTER_COALESCED_SHIFT));

        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_CONTROL_0,
                NV_DRF_DEF(MC, STAT_CONTROL, EMC_GATHER, CLEAR));
        NV_REGW(rm, NvRmPrivModuleID_MemoryController,
                0, MC_STAT_CONTROL_0,
                NV_DRF_DEF(MC, STAT_CONTROL, EMC_GATHER, ENABLE));
}

void
McStat_Start(
        NvRmDeviceHandle rm,
        NvU32 client_id_0,
        NvU32 client_id_1,
        NvU32 llc_client_id)
{
    switch (rm->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            McStatAp1x_Start(rm, client_id_0, client_id_1, llc_client_id);
            break;
        case 0x20:
            McStatAp20_Start(rm, client_id_0, client_id_1, llc_client_id);
            break;
        default:
            NV_ASSERT(!"Unsupported chip ID");
            break;
    }
}

static void
McStatAp1x_Stop(
        NvRmDeviceHandle rm,
        NvU32 *client_0_cycles,
        NvU32 *client_1_cycles,
        NvU32 *llc_client_cycles,
        NvU32 *llc_client_clocks,
        NvU32 *mc_clocks)
{
    *llc_client_cycles = NV_REGR(rm, NvRmPrivModuleID_ExternalMemoryController,
                           0, EMC_STAT_LLMC_COUNT_0_0);
    *llc_client_clocks = NV_REGR(rm, NvRmPrivModuleID_ExternalMemoryController,
                           0, EMC_STAT_LLMC_CLOCKS_0);
    *client_0_cycles = NV_REGR(rm, NvRmPrivModuleID_MemoryController,
                         0, MC_STAT_EMC_COUNT_0_0);
    *client_1_cycles = NV_REGR(rm, NvRmPrivModuleID_MemoryController,
                         0, MC_STAT_EMC_COUNT_1_0);
    *mc_clocks = NV_REGR(rm, NvRmPrivModuleID_MemoryController,
                   0, MC_STAT_EMC_CLOCKS_0);
}

void
McStat_Stop(
        NvRmDeviceHandle rm,
        NvU32 *client_0_cycles,
        NvU32 *client_1_cycles,
        NvU32 *llc_client_cycles,
        NvU32 *llc_client_clocks,
        NvU32 *mc_clocks)
{
    switch (rm->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            McStatAp1x_Stop(rm, client_0_cycles, client_1_cycles, llc_client_cycles, llc_client_clocks, mc_clocks );
            break;
        case 0x20:
            McStatAp20_Stop(rm, client_0_cycles, client_1_cycles, llc_client_cycles, llc_client_clocks, mc_clocks );
            break;
        default:
            NV_ASSERT(!"Unsupported chip ID");
            break;
    }
}


void
McStat_Report(
        NvU32 client_id_0,
        NvU32 client_0_cycles,
        NvU32 client_id_1,
        NvU32 client_1_cycles,
        NvU32 llc_client_id,
        NvU32 llc_client_clocks,
        NvU32 llc_client_cycles,
        NvU32 mc_clocks)
{
    NvOsDebugPrintf("LLC Client %d Count:  0x%.8X, %u\n",
      llc_client_id, llc_client_cycles, llc_client_cycles);
    NvOsDebugPrintf("LLC Client %d Clocks: 0x%.8X, %u\n",
      llc_client_id, llc_client_clocks, llc_client_clocks);
    NvOsDebugPrintf("Client %.3d Count: 0x%.8X, %u\n",
      client_id_0, client_0_cycles, client_0_cycles);
    NvOsDebugPrintf("Client %.3d Count: 0x%.8X, %u\n",
      client_id_1, client_1_cycles, client_1_cycles);
    NvOsDebugPrintf("Total MC Clocks:   0x%.8X, %u\n", mc_clocks, mc_clocks);
}

//API to read data from OBS bus
// The OBS_PART_SEL is mapped to the specified modID by obsInfoTable which is public in this file.

NvError
ReadObsData(
        NvRmDeviceHandle rm,
        NvRmModuleID modID,
        NvU32 start_index,
        NvU32 length,
        NvU32 *value)
{
    NvU32 i = 0, offset = 0, value1, value2;
    NvU32 timeout;
    NvU32 partID = 0xffffffff;
    NvU32 index, temp;

    for (i = 0; i < ObsInfoTableSize; i++)
    {
        if (modID == ObsInfoTable[i].modSelect)
        {
        partID = ObsInfoTable[i].partSelect;
        break;
        }
    }
    if (i == ObsInfoTableSize)
    {
        return NvError_BadParameter;
    }

    for(offset = 0; offset < length; offset++)
    {
        index = start_index + offset;
        temp = NV_DRF_DEF(APB_MISC_GP, OBSCTRL, OBS_EN, ENABLE) |
            NV_DRF_NUM(APB_MISC_GP, OBSCTRL, OBS_MOD_SEL, modID) |
            NV_DRF_NUM(APB_MISC_GP, OBSCTRL, OBS_PART_SEL, partID) |
            NV_DRF_NUM(APB_MISC_GP, OBSCTRL, OBS_SIG_SEL, index) ;
        NV_REGW(rm, NvRmModuleID_Misc, 0, APB_MISC_GP_OBSCTRL_0, temp);
        value1 = NV_REGR(rm, NvRmModuleID_Misc, 0, APB_MISC_GP_OBSCTRL_0);
        timeout = 100;
        do {
            value2 = value1;
            value1 = NV_REGR(rm, NvRmModuleID_Misc, 0, APB_MISC_GP_OBSDATA_0);
            timeout --;
        } while (value1 != value2 && timeout);
        NvOsDebugPrintf("OBS bus modID 0x%x index 0x%x = value 0x%x",
                modID, index, value1);
        value[offset] = value1;
    }
    return NvSuccess;
}

/******************************************************************************/

#define NVRM_AP15_MONITORED_EVENTS_MAX (2)

// AP15 CP15 performance monitor control register layout
#define AP15_CP15_PMNC_0_ENABLE_RANGE           0:0
#define AP15_CP15_PMNC_0_EVENT_CNTS_RESET_RANGE 1:1
#define AP15_CP15_PMNC_0_CYCLE_CNT_RESET_RANGE  2:2
#define AP15_CP15_PMNC_0_EVENT0_CNT_OV_RANGE    8:8
#define AP15_CP15_PMNC_0_EVENT1_CNT_OV_RANGE    9:9
#define AP15_CP15_PMNC_0_CYCLE_CNT_OV_RANGE     10:10
#define AP15_CP15_PMNC_0_EVENT0_RANGE           19:12
#define AP15_CP15_PMNC_0_EVENT1_RANGE           27:20

static void Ap15CorePerfMonDisable(void)
{
    // Disable all performance counters
    NvU32 RegValue = NV_DRF_NUM(AP15_CP15, PMNC, ENABLE, 0);
    MCR(p15, 0, RegValue, c15, c12, 0);
}

static NvError Ap15CorePerfMonCheckStatus(void)
{
    // Check if performance counters are enabled and no overflow has occurred
    NvU32 RegValue;
    MRC(p15, 0, RegValue, c15, c12, 0);
    if ((NV_DRF_VAL(AP15_CP15, PMNC, ENABLE, RegValue) == 0) ||
        (NV_DRF_VAL(AP15_CP15, PMNC, CYCLE_CNT_OV, RegValue) == 1) ||
        (NV_DRF_VAL(AP15_CP15, PMNC, EVENT0_CNT_OV, RegValue) == 1) ||
        (NV_DRF_VAL(AP15_CP15, PMNC, EVENT1_CNT_OV, RegValue) == 1))
        return NvError_InvalidState;
    else
        return NvSuccess;
}

static void Ap15CorePerfMonStart(NvU32* pEventList, NvU32* pEventListSize)
{
    NvU32 RegValue, Event0, Event1;

    // Just return maximum monitored events if no input list, otherwise
    // get both events ready (set the same if only one specified)
    if (*pEventListSize == 0)
    {
        *pEventListSize = NVRM_AP15_MONITORED_EVENTS_MAX;
        return;
    }
    Event0 = Event1 = pEventList[0];
    if (*pEventListSize >= NVRM_AP15_MONITORED_EVENTS_MAX)
    {
        Event1 = pEventList[1];
        *pEventListSize = NVRM_AP15_MONITORED_EVENTS_MAX;
    }

    // Reset, clear overflow flags and enable 3 performance counters:
    // total cycle counter and 2 event counters
    RegValue =
        NV_DRF_NUM(AP15_CP15, PMNC, ENABLE, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, EVENT_CNTS_RESET, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, CYCLE_CNT_RESET, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, CYCLE_CNT_OV, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, EVENT0_CNT_OV, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, EVENT1_CNT_OV, 1) |
        NV_DRF_NUM(AP15_CP15, PMNC, EVENT0, Event0) |
        NV_DRF_NUM(AP15_CP15, PMNC, EVENT1, Event1);
    MCR(p15, 0, RegValue, c15, c12, 0);
}

static NvError Ap15CorePerfMonStop(
    NvU32* pCountListSize,
    NvU32* pCountList,
    NvU32* pTotalCycleCount)
{
    NvU32 ccnt, pmn0, pmn1;

    // Disable monotors and check status
    NvError err = Ap15CorePerfMonCheckStatus();
    Ap15CorePerfMonDisable();
    if (err != NvSuccess)
        return err;

    // Read back cycle and event counters
    MRC(p15, 0, ccnt, c15, c12, 1);
    MRC(p15, 0, pmn0, c15, c12, 2);
    MRC(p15, 0, pmn1, c15, c12, 3);

    // Return total cycle count always, and event counts depending on
    // the room provided by the caller
    *pTotalCycleCount = ccnt;
    if (*pCountListSize == 0)
        return NvSuccess;

    pCountList[0] = pmn1;       // ARM spec Event0 <=> Counter 1 (not a typo)
    if (*pCountListSize >= NVRM_AP15_MONITORED_EVENTS_MAX)
    {
        pCountList[1] = pmn0;   // ARM spec Event1 <=> Counter 0 (not a typo)
        *pCountListSize = NVRM_AP15_MONITORED_EVENTS_MAX;
    }
    return NvSuccess;
}

NvError
NvRmCorePerfMonStart(
    NvRmDeviceHandle hRmDevice,
    NvU32* pEventListSize,
    NvU32* pEventList)
{
    NvU32 cpst;
    NV_ASSERT(hRmDevice);
    NV_ASSERT(pEventListSize);
    NV_ASSERT ((*pEventListSize == 0) || pEventList);

    // Monitoring is supported only for SoC environment in one
    // of the privileged modes
    GET_CPSR(cpst);
    if(IS_USER_MODE(cpst))
        return NvError_NotSupported;
    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
        return NvError_NotSupported;

    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            Ap15CorePerfMonStart(pEventList, pEventListSize);
            return NvSuccess;
        case 0x20:
            return NvError_NotSupported;
        default:
            NV_ASSERT(!"Invalid chip ID");
            return NvError_NotSupported;
    }
}

NvError
NvRmCorePerfMonStop(
    NvRmDeviceHandle hRmDevice,
    NvU32* pCountListSize,
    NvU32* pCountList,
    NvU32* pTotalCycleCount)
{
    NvU32 cpst;
    NV_ASSERT(hRmDevice);
    NV_ASSERT(pTotalCycleCount);
    NV_ASSERT(pCountListSize);
    NV_ASSERT ((*pCountListSize == 0) || pCountList);

    // Monitoring is supported only for SoC environment in one
    // of the privileged modes
    GET_CPSR(cpst);
    if(IS_USER_MODE(cpst))
        return NvError_NotSupported;
    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
        return NvError_NotSupported;

    switch (hRmDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            return Ap15CorePerfMonStop(
                pCountListSize, pCountList, pTotalCycleCount);
        case 0x20:
            return NvError_NotSupported;
        default:
            NV_ASSERT(!"Invalid chip ID");
            return NvError_NotSupported;
    }
}

static NvOsInterruptHandle s_McInterruptHandle = NULL;
static void McErrorIntHandler(void* args)
{
    NvU32 RegVal;
    NvU32 IntStatus;
    NvU32 IntClear = 0;
    NvRmDeviceHandle hRm = (NvRmDeviceHandle)args;

    IntStatus = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTSTATUS_0);
    if ( NV_DRF_VAL(MC, INTSTATUS, DECERR_AXI_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, DECERR_AXI_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_DECERR_AXI_ADR_0);
        NvOsDebugPrintf("AXI DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_DECERR_AXI_STATUS_0);
        NvOsDebugPrintf("AXI DecErrStatus=0x%x ", RegVal);
    }
    if ( NV_DRF_VAL(MC, INTSTATUS, DECERR_EMEM_OTHERS_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, DECERR_EMEM_OTHERS_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_DECERR_EMEM_OTHERS_ADR_0);
        NvOsDebugPrintf("EMEM DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_DECERR_EMEM_OTHERS_STATUS_0);
        NvOsDebugPrintf("EMEM DecErrStatus=0x%x ", RegVal);
    }
    if ( NV_DRF_VAL(MC, INTSTATUS, INVALID_GART_PAGE_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, INVALID_GART_PAGE_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_GART_ERROR_ADDR_0);
        NvOsDebugPrintf("GART DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
                     MC_GART_ERROR_REQ_0);
        NvOsDebugPrintf("GART DecErrStatus=0x%x ", RegVal);
    }

    NV_ASSERT(!"MC Decode Error ");
    // Clear the interrupt.
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTSTATUS_0, IntClear);
    NvRmInterruptDone(s_McInterruptHandle);
}

NvError NvRmPrivAp15McErrorMonitorStart(NvRmDeviceHandle hRm)
{
    NvU32 val;
    NvU32 IrqList;
    NvError e = NvSuccess;
    NvOsInterruptHandler handler;

    if (s_McInterruptHandle == NULL)
    {
        // Install an interrupt handler.
        handler = McErrorIntHandler;
        IrqList = NvRmGetIrqForLogicalInterrupt(hRm,
                      NvRmPrivModuleID_MemoryController, 0);
        NV_CHECK_ERROR( NvRmInterruptRegister(hRm, 1, &IrqList,  &handler,
            hRm, &s_McInterruptHandle, NV_TRUE) );
        // Enable Dec Err interrupts in memory Controller.
        val = NV_DRF_DEF(MC, INTMASK, DECERR_AXI_INTMASK, UNMASKED) |
              NV_DRF_DEF(MC, INTMASK, DECERR_EMEM_OTHERS_INTMASK, UNMASKED) |
              NV_DRF_DEF(MC, INTMASK, INVALID_GART_PAGE_INTMASK, UNMASKED);
        NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTMASK_0, val);
    }
    return e;
}

void NvRmPrivAp15McErrorMonitorStop(NvRmDeviceHandle hRm)
{
    NvRmInterruptUnregister(hRm, s_McInterruptHandle);
    s_McInterruptHandle = NULL;
}

/* This function sets some performance timings for Mc & Emc.  Numbers are from
 * the Arch team.
 *
 */
void NvRmPrivAp15SetupMc(NvRmDeviceHandle hRm)
{
    NvU32   reg, mask;
    reg = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
              MC_LOWLATENCY_CONFIG_0);
    mask = NV_DRF_DEF(MC, LOWLATENCY_CONFIG, CMCR_LL_CTRL, ENABLE) |
           NV_DRF_DEF(MC, LOWLATENCY_CONFIG, CMCR_LL_SEND_BOTH, ENABLE) |
           NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_CTRL, ENABLE) |
           NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_SEND_BOTH, ENABLE);
    if ( mask != (reg & mask) )
        NV_ASSERT(!"MC LL Path not enabled!");

    /* 1) TIMEOUT value for VDE is 256 cycles, 3D, 2D timeouts are disabled, all others 512 cycles. */
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_CTRL_0,    0x00000028);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_CMC_0,     0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_DC_0,      0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_DCB_0,     0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_EPP_0,     0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_G2_0,      0x0);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_HC_0,      0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_ISP_0,     0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_MPCORE_0,  0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_MPEA_0,    0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_MPEB_0,    0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_MPEC_0,    0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_NV_0,      0x0);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_PPCS_0,    0x88888888);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_VDE_0,     0x44444444);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT1_VDE_0,    0x44444444);
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_TIMEOUT_VI_0,      0x88888888);

    /* 2) Command Queue values should be 2,2,6 for better performance. */
    NV_REGW(hRm, NvRmPrivModuleID_ExternalMemoryController, 0, EMC_CMDQ_0,   0x00002206);

    /* 3) MC_EMEM_ARB_CFG0_0 Should have optimal values for 166Mhz DRAM.
     *    27:22 EMEM_BANKCNT_NSP_TH (0xC seems to be better for 166Mhz)
     *    21:16 EMEM_BANKCNT_TH     (0x8 seems to be better for 166Mhz)
     *
     *    MC_EMEM_ARB_CFG0_0 <= 0x0308_1010
     */

    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_EMEM_ARB_CFG0_0,    0x03081010);
}
