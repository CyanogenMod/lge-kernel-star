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

#include "nvrm_diag.h"
#include "nvrm_clocks.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "nvrm_pmu.h"
#include "nvrm_pmu_private.h"
#include "nvodm_query_discovery.h"
#include "ap15rm_private.h"
#include "ap15/ap15rm_clocks.h"
#include "ap20/ap20rm_clocks.h"
#include "ap15/project_relocation_table.h"

#if (NV_DEBUG)
#define NVRM_DIAG_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_DIAG_PRINTF(x)
#endif

// TODO: remove this define when it is added to re-location table header
#if !defined(NV_POWERGROUP_INVALID)
#define NV_POWERGROUP_INVALID (0xFFFF)
#endif

/*
 * Holds mapping information between diagnostic module Ids and pointers to
 * clock information structures
 */
typedef struct DiagModuleMappingRec
{
    // Index mapping diagnostic module Id into the base pointer to the
    // respective module clock information structure
    NvU32 BaseIndex;

    // Total number of the module instances
    NvU32 InstancesNum;
} DiagModuleMapping;

/*
 * Combines modules diagnostic information
 */
typedef struct NvRmDiagModulesRec
{
    // Size of module information table
    NvU32 ModuleClockTableSize;

    // Module clock and reset information table
    const NvRmModuleClockInfo* ModuleClockTable;

    // Table of module instnace pointers into the information table
    const NvRmModuleClockInfo** pInstancePtrs;

    // Mapping indexes of module insatances
    DiagModuleMapping InstancesMap[NvRmDiagModuleID_Num];
} NvRmDiagModules;

/*
 * Combines clock sources diagnostic information
 */
typedef struct NvRmDiagSourcesRec
{
    // Total number of available clock sources
    NvU32 ClockSourcesNum;

    // Map between clock source IDs and handles
    NvRmDiagClockSourceHandle hSources[NvRmClockSource_Num];
} NvRmDiagSources;

// RM handle for diagnostic mode
NvRmDeviceHandle s_hDiagRm = NULL;

/*
 * Holds mapping information between power rails and module power
 * groups
 */
typedef struct NvRmDiagPowerRailRec
{
    // Power rail GUID
    NvU64 PowerRailId;

    // List of power group IDs mapped to this rail, terminated
    // by invalid power group ID
    NvU32 PowerRailGroups[NV_POWERGROUP_MAX + 1];
} NvRmDiagPowerRail;

/*
 * Combines power rails diagnostic information
 */
typedef struct NvRmDiagRailsRec
{
    // Total number of available module rails
    NvU32 PowerRailsNum;

    // Power Rails information table
    const NvRmDiagPowerRail* PowerRailsTable;

    // Combined Module ID and instance of the PMU communication
    // interface controller
    NvRmDiagModuleID PmuBusHostDiagId;
    NvRmModuleID PmuBusHostRmId;
} NvRmDiagRails;

/*****************************************************************************/

static const NvRmDiagPowerRail s_Ap15PowerRailsTable[] =
{
    {
        NV_VDD_RTC_ODM_ID,
        {
            NV_POWERGROUP_AO,
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_CORE_ODM_ID,
        {
            NV_POWERGROUP_NPG,
            NV_POWERGROUP_CPU,
            NV_POWERGROUP_TD,
            NV_POWERGROUP_VE,
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_PLLA_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLM_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLP_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLU_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLU1_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLHDMI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_OSC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_SYS_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_USB_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_HDMI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_MIPI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_LCD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_AUD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_DDR_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_NAND_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_UART_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_SDIO_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_VDAC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_VI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_BB_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    }
};
static const NvU32 s_Ap15PowerRailsTableSize = NV_ARRAY_SIZE(s_Ap15PowerRailsTable);

static const NvRmDiagPowerRail s_Ap20PowerRailsTable[] =
{
    {
        NV_VDD_RTC_ODM_ID,
        {
            NV_POWERGROUP_AO,
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_CORE_ODM_ID,
        {
            NV_POWERGROUP_NPG,
            NV_POWERGROUP_TD,
            NV_POWERGROUP_VE,
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_CPU_ODM_ID,
        {
            NV_POWERGROUP_CPU,
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_PLLA_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLM_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLP_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLU_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLU1_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLHDMI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_PLLX_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_OSC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },

    {
        NV_VDD_SYS_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_USB_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_HDMI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_MIPI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_LCD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_AUD_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_DDR_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_NAND_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_UART_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_SDIO_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_VDAC_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_VI_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    },
    {
        NV_VDD_BB_ODM_ID,
        {
            NV_POWERGROUP_INVALID
        }
    }
};
static const NvU32 s_Ap20PowerRailsTableSize = NV_ARRAY_SIZE(s_Ap20PowerRailsTable);

static const NvU64 s_ApClockSourceNames[] =
{
    0x0,
#define NVRM_CLOCK_SOURCE(A, B, C, D, E, F, G, H, x) \
((NvU64)((((A)&0xFFULL)<<56) | \
         (((B)&0xFFULL)<<48) | \
         (((C)&0xFFULL)<<40) | \
         (((D)&0xFFULL)<<32) | \
         (((E)&0xFFULL)<<24) | \
         (((F)&0xFFULL)<<16) | \
         (((G)&0xFFULL)<<8)  | \
         (((H)&0xFFULL))) ),
    #include "nvrm_clockids.h"
#undef NVRM_CLOCK_SOURCE
};

// Power rails diagnostic information
NvRmDiagRails s_Rails = {0};

// Modules diagnostic information
NvRmDiagModules s_Modules = {0};

// Clock sources diagnostic information
NvRmDiagSources s_Sources = {0};

/*****************************************************************************/

static NvRmModuleID MapDiagIdToRmId(NvRmDiagModuleID DiagId);

/*****************************************************************************/

NvError
NvRmDiagEnable(NvRmDeviceHandle hRmDevice)
{
    NvU32 i, index;
    size_t s;
    NvError error;
    void* p = NULL;

    /*
     * Initialize RM handle, which indicates enabled diagnastic mode
     */
    NV_ASSERT(hRmDevice);
    if (s_hDiagRm != NULL)
        return NvSuccess;   // Already enabled and initialized
    s_hDiagRm = hRmDevice;

    /*
     * Fill in modules information clear instance map, and allocate
     * module instance pointers table
     */
    if (hRmDevice->ChipId.Id == 0x20)
    {
        s_Modules.ModuleClockTableSize = g_Ap20ModuleClockTableSize;
        s_Modules.ModuleClockTable = g_Ap20ModuleClockTable;
        s_Rails.PowerRailsNum = s_Ap20PowerRailsTableSize;
        s_Rails.PowerRailsTable = s_Ap20PowerRailsTable;
    } else
    {
        s_Modules.ModuleClockTableSize = g_Ap15ModuleClockTableSize;
        s_Modules.ModuleClockTable = g_Ap15ModuleClockTable;
        s_Rails.PowerRailsNum = s_Ap15PowerRailsTableSize;
        s_Rails.PowerRailsTable = s_Ap15PowerRailsTable;
    }

    s_Rails.PmuBusHostDiagId = NvRmDiagModuleID_Dvc;  // Default for AP15


    NV_ASSERT(s_Modules.ModuleClockTableSize);

    NvOsMemset(s_Modules.InstancesMap, 0, sizeof(s_Modules.InstancesMap));
    s = sizeof(NvRmModuleClockInfo*) * s_Modules.ModuleClockTableSize;
    p = NvOsAlloc(s);
    if (!p)
    {
        error = NvError_InsufficientMemory;
        goto failed;
    }
    NvOsMemset(p, 0, s);
    s_Modules.pInstancePtrs = p;

    /*
     * Parse module clock/reset information table and fill in mapping arrays.
     * The table lists all valid (present) modules and only valid modules.
     */
    // 1st pass - count module instances
    for (i = 0; i < s_Modules.ModuleClockTableSize; i++)
    {
        NvRmDiagModuleID id = s_Modules.ModuleClockTable[i].DiagModuleID;
        NV_ASSERT((0 < id) && (id < NvRmDiagModuleID_Num));
        s_Modules.InstancesMap[id].InstancesNum++;
    }

    // 2nd pass - fill in mapping indexes
    for (index = 0, i = 0; i < NvRmDiagModuleID_Num; i++)
    {
        DiagModuleMapping* pMapping = &s_Modules.InstancesMap[i];
        if (pMapping->InstancesNum != 0)
        {
            pMapping->BaseIndex = index;
            index += pMapping->InstancesNum;
            NV_ASSERT(index <= s_Modules.ModuleClockTableSize);
        }
    }

    // 3rd pass - fill in instance pointers
    for (i = 0; i < s_Modules.ModuleClockTableSize; i++)
    {
        DiagModuleMapping* pMapping =
            &s_Modules.InstancesMap[s_Modules.ModuleClockTable[i].DiagModuleID];
        NvU32 instance = s_Modules.ModuleClockTable[i].Instance;
        index = pMapping->BaseIndex + instance;

        NV_ASSERT(instance < pMapping->InstancesNum);
        NV_ASSERT(s_Modules.pInstancePtrs[index] == NULL);

        s_Modules.pInstancePtrs[index] = &s_Modules.ModuleClockTable[i];
    }

    // Convert PMU Host diagnostic ID to common RM ID
    s_Rails.PmuBusHostRmId = MapDiagIdToRmId(s_Rails.PmuBusHostDiagId);

    /*
     * Parse clock sources information table and map clock source IDs
     * to handles. Count total available sources.
     */
    NvOsMemset(s_Sources.hSources, 0, sizeof(s_Sources.hSources));
    for (s_Sources.ClockSourcesNum = 0, i = 1; i < NvRmClockSource_Num; i++)
    {
        s_Sources.hSources[i] = NvRmPrivGetClockSourceHandle(i);
        if (s_Sources.hSources[i] != NULL)
            s_Sources.ClockSourcesNum++;
    }

    // Make sure DFS is not running
    NvRmDfsSetState(s_hDiagRm, NvRmDfsRunState_Stopped);
    return NvSuccess;

failed:
    NvOsFree(p);
    NvOsMemset(&s_Modules, 0, sizeof(s_Modules));
    NvOsMemset(&s_Sources, 0, sizeof(s_Sources));
    s_hDiagRm = NULL;
    return error;
}

/*****************************************************************************/

NvError
NvRmDiagListModules(
    NvU32* pListSize,
    NvRmDiagModuleID* pIdList)
{
    NvU32 ModulesNum, i;

    NV_ASSERT(pListSize);
    NV_ASSERT(pIdList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }
    ModulesNum = s_Modules.ModuleClockTableSize;

    // Return total number of modules if no room for the output list
    if ((*pListSize) == 0)
    {
        *pListSize = ModulesNum;
        return NvSuccess;
    }

    // Return modules list (min of requested and total size)
    if ((*pListSize) > ModulesNum)
    {
        (*pListSize) = ModulesNum;
    }
    for (i = 0; i < (*pListSize); i++, pIdList++)
    {
        const NvRmModuleClockInfo* pCinfo = &s_Modules.ModuleClockTable[i];
        *pIdList = NVRM_DIAG_MODULE(pCinfo->DiagModuleID, pCinfo->Instance);
    }
    return NvSuccess;
}

NvError
NvRmDiagListClockSources(
    NvU32* pListSize,
    NvRmDiagClockSourceHandle* phSourceList)
{
    NvU32 SourcesNum, i;
    NV_ASSERT(pListSize);
    NV_ASSERT(phSourceList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Return total number of sources if no room for the output list
    if ((*pListSize) == 0)
    {
        *pListSize = s_Sources.ClockSourcesNum;
        return NvSuccess;
    }

    // Return sources list (min of requested and total size)
    for (SourcesNum = 0, i = 0; i < NvRmClockSource_Num; i++)
    {
        NvRmDiagClockSourceHandle hSource = s_Sources.hSources[i];
        if (hSource != NULL)
        {
            SourcesNum++;
            *(phSourceList++) = hSource;
            if (SourcesNum >= (*pListSize))
                    break;
        }
    }
    *pListSize = SourcesNum;
    return NvSuccess;
}

/*****************************************************************************/

NvError
NvRmDiagModuleListClockSources(
    NvRmDiagModuleID id,
    NvU32 * pListSize,
    NvRmDiagClockSourceHandle* phSourceList)
{
    NvU32 SourcesNum, i;
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(id);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(id);

    NV_ASSERT(pListSize);
    NV_ASSERT(phSourceList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify module id and get module info
    NV_ASSERT((Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum));
    pCinfo =
        s_Modules.pInstancePtrs[s_Modules.InstancesMap[Module].BaseIndex + Instance];

    /*
     * Return total number of module sources if no room for the output list,
     * otherwise return module sources list (min of requested and total size)
     */
    for (SourcesNum = 0, i = 0; i < NvRmClockSource_Num; i++)
    {
        NvRmClockSource source = pCinfo->Sources[i];
        NV_ASSERT(source < NvRmClockSource_Num);
        if (source != NvRmClockSource_Invalid)
        {
            SourcesNum++;
            if ((*pListSize) != 0)
            {
                *phSourceList = s_Sources.hSources[source];
                NV_ASSERT(*phSourceList);
                phSourceList++;
                if (SourcesNum >= (*pListSize))
                    break;
            }
        }
    }
    *pListSize = SourcesNum;
    return NvSuccess;
}

NvError
NvRmDiagModuleClockEnable(
    NvRmDiagModuleID id,
    NvBool enable)
{
    NvU32 reg, offset;
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(id);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(id);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify module id and get module info
    NV_ASSERT((Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum));
    pCinfo =
        s_Modules.pInstancePtrs[s_Modules.InstancesMap[Module].BaseIndex + Instance];

    // Set/Clear clock control bit(s), if any
    if (pCinfo->ClkEnableField != 0)
    {
        offset = pCinfo->ClkEnableOffset;
        NV_ASSERT(offset);
        reg = NV_REGR(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset);
        reg = enable ?
            (reg | pCinfo->ClkEnableField) : (reg & (~ pCinfo->ClkEnableField));
        NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
    }
    return NvSuccess;
}

NvError
NvRmDiagModuleClockConfigure(
    NvRmDiagModuleID id,
    NvRmDiagClockSourceHandle hSource,
    NvU32 divider,
    NvBool Source1st)
{
    NvU32 reg, offset, SrcIndex;
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(id);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(id);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify source handle, module id, and get module info
    NV_ASSERT((hSource != NULL) &&
        (Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum));
    pCinfo =
        s_Modules.pInstancePtrs[s_Modules.InstancesMap[Module].BaseIndex + Instance];

    /*
     * Find source index for the specified module and source handle. If not
     * found report invalid handle. If module has fixed clock source and no
     * divider, return success.
     */
    for (SrcIndex = 0; SrcIndex < NvRmClockSource_Num; SrcIndex++)
    {
        if (hSource->SourceId == pCinfo->Sources[SrcIndex])
            break;
    }
    NV_ASSERT(SrcIndex != NvRmClockSource_Num);
    if ((pCinfo->SourceFieldMask == 0) && (pCinfo->DivisorFieldMask == 0))
    {
        return NvSuccess;
    }
    NV_ASSERT(SrcIndex <= pCinfo->SourceFieldMask);

    /*
    * Adjust divider valuse: if module divider is not fractional, shift out
    * half step bit. In any case truncate high divider bits to fit module
    * divider field.
    */
    if (pCinfo->Divider != NvRmClockDivider_Fractional_2)
    {
        divider >>= 1;
    }
    divider &= pCinfo->DivisorFieldMask;

    /*
     * Update clock control register. The order of source and divider fields
     * update is specified by the caller. Insert delay between the updates.
     */
    offset = pCinfo->ClkSourceOffset;
    NV_ASSERT(offset);
    reg = NV_REGR(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset);
    if (Source1st)
    {
        reg &= (~(pCinfo->SourceFieldMask << pCinfo->SourceFieldShift));
        reg |= (SrcIndex << pCinfo->SourceFieldShift);
        NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
    if (pCinfo->Divider != NvRmClockDivider_None)
    {
        reg &= (~(pCinfo->DivisorFieldMask << pCinfo->DivisorFieldShift));
        reg |= (divider << pCinfo->DivisorFieldShift);
        NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
    if (!Source1st)
    {
        reg &= (~(pCinfo->SourceFieldMask << pCinfo->SourceFieldShift));
        reg |= (SrcIndex << pCinfo->SourceFieldShift);
        NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);
    }
    return NvSuccess;
}

NvError
NvRmDiagModuleReset(
    NvRmDiagModuleID id,
    NvBool KeepAsserted)
{
    NvU32 reg, offset;
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(id);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(id);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify module id and get module info
    NV_ASSERT((Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum));
    pCinfo =
        s_Modules.pInstancePtrs[s_Modules.InstancesMap[Module].BaseIndex + Instance];

    /*
     * Assert reset bit and keep it asserted if requested by the caller.
     * Otherwise de-assert reset after the delay.
     */
    offset = pCinfo->ClkResetOffset;
    NV_ASSERT(offset);
    reg = NV_REGR(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset);
    reg |= pCinfo->ClkResetField;
    NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
    if (!KeepAsserted)
    {
        NvOsWaitUS(NVRM_RESET_DELAY);
        reg &= (~(pCinfo->ClkResetField));
        NV_REGW(s_hDiagRm, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
    }
    return NvSuccess;
}

/*****************************************************************************/

NvU64 NvRmDiagClockSourceGetName(
    NvRmDiagClockSourceHandle hSource)
{
    if ((s_hDiagRm == NULL) ||
        (hSource == NULL) ||
        (hSource->SourceId == NvRmClockSource_Invalid) ||
        (hSource->SourceId >= NvRmClockSource_Num))
    {
        return 0;
    }
    return s_ApClockSourceNames[hSource->SourceId];
}

NvRmDiagClockSourceType
NvRmDiagClockSourceGetType(NvRmDiagClockSourceHandle hSource)
{
    if ((s_hDiagRm == NULL) || (hSource == NULL))
    {
        return 0;
    }
    // Map RM source types to diagnostic source types
    switch (hSource->SourceType)
    {
        case NvRmClockSourceType_Fixed:
            return NvRmDiagClockSourceType_Oscillator;
        case NvRmClockSourceType_Pll:
            return NvRmDiagClockSourceType_Pll;
        case NvRmClockSourceType_Divider:
        case NvRmClockSourceType_Core:
        case NvRmClockSourceType_Selector:
            return NvRmDiagClockSourceType_Scaler;
        default:
            NV_ASSERT(!"Invalid source type");
            return 0;
    }
}

// TODO: does diagnostic scripts really need these details on scaler types?
NvRmDiagClockScalerType
NvRmDiagClockSourceGetScaler(NvRmDiagClockSourceHandle hSource)
{
    if ((s_hDiagRm == NULL) || (hSource == NULL))
    {
        return 0;
    }
    // Map RM divider types to diagnostic scaler types
    switch (hSource->SourceType)
    {
        case NvRmClockSourceType_Fixed:
        case NvRmClockSourceType_Pll:
            return NvRmDiagClockScalerType_NoScaler;
        case NvRmClockSourceType_Divider:
            switch (hSource->pInfo.pDivider->Divider)
            {
                case NvRmClockDivider_Keeper16:
                case NvRmClockDivider_Skipper16:
                    return NvRmDiagClockScalerType_Divider_M_16;
                case NvRmClockDivider_Fractional_2:
                case NvRmClockDivider_Integer_1:
                case NvRmClockDivider_Integer:
                    return NvRmDiagClockScalerType_Divider_1_N;
                default:
                    NV_ASSERT(!"Invalid divider type");
                    return 0;
            }
        case NvRmClockSourceType_Core:
            return NvRmDiagClockScalerType_Divider_M_N;
        case NvRmClockSourceType_Selector:
            return NvRmDiagClockScalerType_Doubler;
        default:
            NV_ASSERT(!"Invalid source type");
            return 0;
    }
}

NvError
NvRmDiagClockSourceListSources(
    NvRmDiagClockSourceHandle hSource,
    NvU32* pListSize,
    NvRmDiagClockSourceHandle * phSourceList)
{
    NvRmClockSource source = NvRmClockSource_Invalid;
    NvRmClockSource* Sources = NULL;

    NV_ASSERT(pListSize);
    NV_ASSERT(phSourceList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }
    NV_ASSERT((hSource != NULL) &&
        (hSource->SourceId != NvRmClockSource_Invalid) &&
        (hSource->SourceId < NvRmClockSource_Num));

    switch (hSource->SourceType)
    {
        // Get input clock ID for single-input clock sources;
        // (may be invalid for primary sources)
        case NvRmClockSourceType_Fixed:
            source = hSource->pInfo.pFixed->InputId;
            break;
        case NvRmClockSourceType_Pll:
            source = hSource->pInfo.pPll->InputId;
            break;
        case NvRmClockSourceType_Divider:
            source = hSource->pInfo.pDivider->InputId;
            break;
        // Get pointer to the source array for core and selector sources
        // (must be valid)
        case NvRmClockSourceType_Core:
            Sources = hSource->pInfo.pCore->Sources;
            NV_ASSERT(Sources);
            break;
        case NvRmClockSourceType_Selector:
            Sources = hSource->pInfo.pSelector->Sources;
            NV_ASSERT(Sources);
            break;
        default:
            NV_ASSERT(!"Invalid source type");
    }
    if (Sources != NULL)
    {
        // Return total number of input sources if no room for the output list,
        // otherwise return sources list (min of requested and total size)
        NvU32 SourcesNum, i;
        for (SourcesNum = 0, i = 0; i < NvRmClockSource_Num; i++)
        {
            NvRmClockSource source = Sources[i];
            NV_ASSERT(source < NvRmClockSource_Num);
            if (source != NvRmClockSource_Invalid)
            {
                SourcesNum++;
                if ((*pListSize) != 0)
                {
                    *phSourceList = s_Sources.hSources[source];
                    NV_ASSERT(*phSourceList);
                    phSourceList++;
                    if (SourcesNum >= (*pListSize))
                        break;
                }
            }
        }
        *pListSize = SourcesNum;
    }
    else if (source != NvRmClockSource_Invalid)
    {
        //Only one input source is available. Return the resepctive handle
        // if requested.
        NV_ASSERT(source < NvRmClockSource_Num);
        if ((*pListSize) != 0)
            *phSourceList = s_Sources.hSources[source];
        *pListSize = 1;
    }
    else
    {
        // Primary source (e.g., oscillator). No (= zero) input sources.
        *pListSize = 0;
    }
    return NvSuccess;
}

/*****************************************************************************/

NvU32 NvRmDiagOscillatorGetFreq(NvRmDiagClockSourceHandle hOscillator)
{

    if ((s_hDiagRm == NULL) || (hOscillator == NULL) ||
        (hOscillator->SourceId == NvRmClockSource_Invalid) ||
        (hOscillator->SourceType != NvRmClockSourceType_Fixed))
    {
        return 0;
    }
    return NvRmPrivGetClockSourceFreq(hOscillator->SourceId);
}

NvError
NvRmDiagPllConfigure(
    NvRmDiagClockSourceHandle hPll,
    NvU32 M,
    NvU32 N,
    NvU32 P)
{
    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }
    NV_ASSERT((hPll != NULL) &&
        (hPll->SourceId != NvRmClockSource_Invalid) &&
        (hPll->SourceType == NvRmClockSourceType_Pll));

    NvRmPrivAp15PllSet(s_hDiagRm, hPll->pInfo.pPll, M, N, P, (NvU32)-1,
                       0, 0, NV_TRUE, NvRmPllConfigFlags_Override);
    return NvSuccess;
}

NvError
NvRmDiagClockScalerConfigure(
    NvRmDiagClockSourceHandle hScaler,
    NvRmDiagClockSourceHandle hInput,
    NvU32 M,
    NvU32 N)
{
    NvU32 setting = 0;

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }
    NV_ASSERT(hScaler != NULL);

    switch (hScaler->SourceType)
    {
        case NvRmClockSourceType_Divider:
            switch (hScaler->pInfo.pDivider->Divider)
            {
                case NvRmClockDivider_Keeper16:
                    setting = M >> 1;
                    break;
                case NvRmClockDivider_Skipper16:
                    setting = (~(M >> 1));
                    break;
                case NvRmClockDivider_Fractional_2:
                    setting = N;
                    break;
                case NvRmClockDivider_Integer_1:
                case NvRmClockDivider_Integer:
                    setting = N >> 1;
                    break;
                default:
                    NV_ASSERT(!"Invalid divider type");
            }
            NvRmPrivDividerSet(s_hDiagRm, hScaler->pInfo.pDivider, setting);
            return NvSuccess;

        case NvRmClockSourceType_Core:
            NvRmPrivCoreClockSet(s_hDiagRm, hScaler->pInfo.pCore,
                                        hInput->SourceId, (M >> 1), (N >> 1));
            break;
        case NvRmClockSourceType_Selector:
            NvRmPrivSelectorClockSet(s_hDiagRm, hScaler->pInfo.pSelector,
                                         hInput->SourceId, (M != 0));
            break;
        case NvRmClockSourceType_Pll:
        case NvRmClockSourceType_Fixed:
            NV_ASSERT(!" Diag Clock Scaler Config: illegal clock source. ");
            break;
        default:
            NV_ASSERT(!"Invalid source type");
            break;
    }
    return NvSuccess;
}

/*****************************************************************************/

/*
 * Gets power group for the specified module if it is one of system modules, not
 * present in the relocation table. Otherwise, returns NV_POWERGROUP_INVALID.
 */
static NvU32
DiagGetSystemModulePowerGroup(const NvRmModuleClockInfo* pCinfo);

static NvU32
DiagGetSystemModulePowerGroup(const NvRmModuleClockInfo* pCinfo)
{
    NvU32 PowerGroup = NV_POWERGROUP_INVALID;
    switch (pCinfo->Module)
    {
        case NvRmModuleID_CacheMemCtrl:
            if (pCinfo->Instance == 0)
                break; // CPU cache controller is present
        // fall through if AVP cache controller
        case NvRmModuleID_Invalid:
        case NvRmPrivModuleID_System:
        case NvRmModuleID_Avp:
            PowerGroup = NV_POWERGROUP_NPG;
            break;
        case NvRmModuleID_Cpu:
            PowerGroup = NV_POWERGROUP_CPU;
            break;
        default:
            break;
    }
    return PowerGroup;
}

NvError
NvRmDiagListPowerRails(
    NvU32* pListSize,
    NvRmDiagPowerRailHandle* phRailList)
{
    NvU32 RailsNum, i;

    NV_ASSERT(pListSize);
    NV_ASSERT(phRailList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }
    RailsNum = s_Rails.PowerRailsNum;

    // Return total number of rails if no room for the output list
    if ((*pListSize) == 0)
    {
        *pListSize = RailsNum;
        return NvSuccess;
    }

    // Return rails list (min of requested and total size)
    if ((*pListSize) > RailsNum)
    {
        (*pListSize) = RailsNum;
    }
    for (i = 0; i < (*pListSize); i++, phRailList++)
    {
        *phRailList = (NvRmDiagPowerRailHandle)&s_Rails.PowerRailsTable[i];
    }
    return NvSuccess;
}

NvU64
NvRmDiagPowerRailGetName(NvRmDiagPowerRailHandle hRail)
{
    if ((s_hDiagRm == NULL) || (hRail == NULL))
    {
        return 0;
    }
    return hRail->PowerRailId;
}

NvError
NvRmDiagModuleListPowerRails(
    NvRmDiagModuleID id,
    NvU32* pListSize,
    NvRmDiagPowerRailHandle* phRailList)
{

    NvU32 ModulePowerGroup, i;
    const NvRmDiagPowerRail* pRail = NULL;
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(id);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(id);

    NV_ASSERT(pListSize);
    NV_ASSERT(phRailList);

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify module id
    NV_ASSERT((Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum));

    // One rail per module; just return if no room to return handle
    if ((*pListSize) == 0)
    {
        *pListSize = 1;
        return NvSuccess;
    }

    // Get module power group
    pCinfo =
        s_Modules.pInstancePtrs[s_Modules.InstancesMap[Module].BaseIndex + Instance];
    ModulePowerGroup = DiagGetSystemModulePowerGroup(pCinfo);
    if (ModulePowerGroup == NV_POWERGROUP_INVALID)
    {
        NvRmModuleInstance* pInst = NULL;
        NV_ASSERT_SUCCESS(NvRmPrivGetModuleInstance(
            s_hDiagRm, NVRM_MODULE_ID(pCinfo->Module, pCinfo->Instance), &pInst));
        ModulePowerGroup = pInst->DevPowerGroup;
    }
    NV_ASSERT(ModulePowerGroup != NV_POWERGROUP_INVALID);

    // Find the power rail for the group
    for (i = 0; i < s_Rails.PowerRailsNum; i++)
    {
        const NvU32* pPowerGroup = s_Rails.PowerRailsTable[i].PowerRailGroups;
        while ((*pPowerGroup) != NV_POWERGROUP_INVALID)
        {
            if ((*pPowerGroup) == ModulePowerGroup)
            {
                pRail = &s_Rails.PowerRailsTable[i];
                break;
            }
            pPowerGroup++;
            NV_ASSERT(pPowerGroup < (s_Rails.PowerRailsTable[i].PowerRailGroups +
                      NV_ARRAY_SIZE(s_Rails.PowerRailsTable[i].PowerRailGroups)));
        }
        if (pRail)
            break;
    }

    // Return power rail found
    NV_ASSERT(pRail);
    *phRailList = (NvRmDiagPowerRailHandle)pRail;
    *pListSize = 1;
    return NvSuccess;
}

NvError
NvRmDiagConfigurePowerRail(
    NvRmDiagPowerRailHandle hRail,
    NvU32 VoltageMV)
{
    NvU32 TimeUs = 0;
    NvU32 RailAddress = 0;
    const NvOdmPeripheralConnectivity* pPmuRail = NULL;

    if (s_hDiagRm == NULL)
    {
        return NvError_NotInitialized;
    }

    // Verify that targeted rail can be found on the board, and
    // it is connected to PMU
    if (hRail != NULL)
    {
        pPmuRail = NvOdmPeripheralGetGuid(hRail->PowerRailId);
    }
    if((pPmuRail == NULL) || (pPmuRail->NumAddress == 0))
    {
        NV_ASSERT(!"Invalid power rail");
        return NvError_NotSupported;
    }

    // Change voltage, and wait for settling time.
    RailAddress = pPmuRail->AddressList[0].Address;
    NVRM_DIAG_PRINTF(("Setting PMU rail %2d to %5dmV\n", RailAddress, VoltageMV));
    if (NvRmPrivDiagPmuSetVoltage(s_hDiagRm, RailAddress, VoltageMV, &TimeUs))
    {
        NvOsWaitUS(TimeUs);
        return NvSuccess;
    }
    return NvError_Busy;
}

/*****************************************************************************/

static NvRmModuleID
MapDiagIdToRmId(NvRmDiagModuleID DiagId)
{
    const NvRmModuleClockInfo* pCinfo = NULL;
    NvU32 Instance = NVRM_DIAG_MODULE_INSTANCE(DiagId);
    NvRmDiagModuleID Module = NVRM_DIAG_MODULE_ID(DiagId);

    NvRmModuleID RmId = NvRmModuleID_Invalid;
    if ((Module < NvRmDiagModuleID_Num) &&
        (Instance < s_Modules.InstancesMap[Module].InstancesNum))
    {
        pCinfo = s_Modules.pInstancePtrs[
            s_Modules.InstancesMap[Module].BaseIndex + Instance];
        RmId = NVRM_MODULE_ID(pCinfo->Module, pCinfo->Instance);
    }
    return RmId;
}

NvBool NvRmPrivIsDiagMode(NvRmModuleID ModuleId)
{
    if (s_hDiagRm == NULL)
        return NV_FALSE;    // Report no diagnostic in progress

    if (ModuleId == NvRmModuleID_Invalid)
        return NV_TRUE;     // Report diagnostic is in progress

    // Report diagnostic is in progress for any module except PMU bus host
    return (ModuleId != s_Rails.PmuBusHostRmId);
}

NvBool NvRmDiagIsLockSupported(void)
{
#if NVRM_DIAG_LOCK_SUPPORTED
    return NV_TRUE;
#else
    return NV_FALSE;
#endif
}
