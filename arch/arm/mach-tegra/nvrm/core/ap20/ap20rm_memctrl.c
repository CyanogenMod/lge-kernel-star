/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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
#include "ap20/armc.h"
#include "ap20/aremc.h"
#include "ap20/arahb_arbc.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_structure.h"

NvError NvRmPrivAp20McErrorMonitorStart(NvRmDeviceHandle hRm);
void NvRmPrivAp20McErrorMonitorStop(NvRmDeviceHandle hRm);
void NvRmPrivAp20SetupMc(NvRmDeviceHandle hRm);

void
McStatAp20_Start(
        NvRmDeviceHandle rm,
        NvU32 client_id_0,
        NvU32 client_id_1,
        NvU32 llc_client_id);
void
McStatAp20_Stop(
        NvRmDeviceHandle rm,
        NvU32 *client_0_cycles,
        NvU32 *client_1_cycles,
        NvU32 *llc_client_cycles,
        NvU32 *llc_client_clocks,
        NvU32 *mc_clocks);

NvError NvRmPrivAp20McErrorMonitorStart(NvRmDeviceHandle hRm)
{
    return NvSuccess;
}

void NvRmPrivAp20McErrorMonitorStop(NvRmDeviceHandle hRm)
{
}

/* This function sets some performance timings for Mc & Emc.  Numbers are from
 * the Arch team.
 *
 */
void NvRmPrivAp20SetupMc(NvRmDeviceHandle hRm)
{
    NvU32   reg, mask;
    reg = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0,
              MC_LOWLATENCY_CONFIG_0);
    mask = NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_CTRL, ENABLE) |
           NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_SEND_BOTH, ENABLE);
    if ( mask != (reg & mask) )
        NV_ASSERT(!"MC LL Path not enabled!");
    // For AP20, no need to program any MC timeout registers here. Default
    // values should be good enough.
}



void
McStatAp20_Start(
        NvRmDeviceHandle rm,
        NvU32 client_id_0,
        NvU32 client_id_1,
        NvU32 llc_client_id)
{
    NvU32 emc_ctrl =
      (AREMC_STAT_CONTROL_MODE_BANDWIDTH << AREMC_STAT_CONTROL_MODE_SHIFT) |
      (AREMC_STAT_CONTROL_EVENT_QUALIFIED << AREMC_STAT_CONTROL_EVENT_SHIFT) |
      (AREMC_STAT_CONTROL_CLIENT_TYPE_MPCORER <<
         AREMC_STAT_CONTROL_CLIENT_TYPE_SHIFT) |  // default is MPCORE Read client
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
McStatAp20_Stop(
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
