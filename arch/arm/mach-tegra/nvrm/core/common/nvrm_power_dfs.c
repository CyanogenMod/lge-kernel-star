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
 * @b Description: Implements NvRM Dynamic Voltage and Frequency Scaling for
 *                  for SOC-wide clock domains.
 * 
 */

#include "nvrm_power_dfs.h"
#include "nvrm_pmu.h"
#include "nvrm_pmu_private.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "nvodm_query_discovery.h"
#include "ap15/ap15rm_private.h"
#include "ap15/ap15rm_power_dfs.h"
#include "ap15/ap15rm_clocks.h"
#include "ap20/ap20rm_power_dfs.h"
#include "ap20/ap20rm_clocks.h"

/*****************************************************************************/

// Initial DFS configuration
#define AP15_FPGA_FREQ (8330)
#define AP20_FPGA_FREQ (13000)
#define NVRM_FPGA_INITIAL_DFS_STATE (NvRmDfsRunState_Disabled)
#define NVRM_AP15_SOC_INITIAL_DFS_STATE (NvRmDfsRunState_Stopped)
#define NVRM_AP20_SOC_INITIAL_DFS_STATE (NvRmDfsRunState_Stopped)
#define NVRM_EMC_DFS_DEFAULT_DISABLED (0)

// Low boundaries for DFS clock frequencies imposed by download transports.
// For Ethernet this limitation is related to MIO WAR, and it is applied only
// to AP15 A01.
#define NVRM_USB_AHB_MIN_KHZ (100000)
#define NVRM_ETHERNET_AHB_MIN_KHZ (30000)
#define NVRM_ETHERNET_EMC_MIN_KHZ (24000)
#define NVRM_SPI_CPU_MIN_KHZ (40000)
#define NVRM_SPI_APB_MIN_KHZ (30000)

// An option to stall average accumulation during busy pulse
#define NVRM_DFS_STALL_AVERAGE_IN_BUSY_PULSE (0)

// Options for temperature monitoring
#define NVRM_DTT_DISABLED (0)
#define NVRM_DTT_USE_INTERRUPT (1)
#define NVRM_DTT_RANGE_CHANGE_PRINTF (1)

// Allow PMUs with CPU voltage range above chip minimum
#define NVRM_DVS_ACCEPT_PMU_HIGH_CPU_MIN (1)

/*****************************************************************************/

// TODO: Always Disable before check-in
// Module debug: 0=disable, 1=enable
#define NVRM_DFS_ENABLE_PRINTF (0)
#if NVRM_DFS_ENABLE_PRINTF
#define NVRM_DFS_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_DFS_PRINTF(x)
#endif

// TODO: Always Disable before check-in
// DFS profiling: 0=disable, 1=enable (prints from clock control thread)
#define DFS_PROFILING (0)

// TODO: Always Disable before check-in
// DFS clients busy and starvation hints report: 0=disable, 1=enable
// (prints from client API thread)
#define DFS_HINTS_PRINTF (0)

// TODO: Always Disable before check-in
// DFS detailed logging: 0=disable, non zero = enable (saves dfs log in memory
//  for at least specified number of seconds)
#define DFS_LOGGING_SECONDS (0)

// TODO: Always Disable before check-in
// DFS sync busy int timeout: 0=disable, non zero = enable (set sync busy hint
//  timeout for specified number of milliseconds)
#define DFS_SYNC_BUSY_TIMEOUT_MS (0)

/*****************************************************************************/

// Microsecond timer read macro
#define NvRmPrivGetUs() NV_READ32(s_pTimerUs)

/*****************************************************************************/

#if DFS_PROFILING

typedef struct DfsProfileRec
{
    NvU32 SamplesNo[NvRmDfsProfileId_Num];
    NvU32 StartUs[NvRmDfsProfileId_Num];
    NvU32 AccumulatedUs[NvRmDfsProfileId_Num];
} DfsProfile;


#define DfsProfileInit(pDfs) \
do\
{\
    NvU32 i; \
    NvOsMemset(&s_Profile, 0, sizeof(DfsProfile)); \
    for (i = 1; i < NvRmDfsProfileId_Num; i++) \
    {\
        s_Profile.StartUs[i] = NvRmPrivGetUs(); \
    }\
} while(0)

#define DfsProfileStart(pDfs, ProfileId) \
do\
{\
    if ((pDfs)->DfsRunState == NvRmDfsRunState_ProfiledLoop) \
    {\
        s_Profile.StartUs[(ProfileId)] = NvRmPrivGetUs(); \
    }\
} while(0)

#define DfsProfileSample(pDfs, ProfileId) \
do\
{\
    if ((pDfs)->DfsRunState == NvRmDfsRunState_ProfiledLoop) \
    {\
        s_Profile.SamplesNo[(ProfileId)]++; \
        s_Profile.AccumulatedUs[(ProfileId)] += \
            (NvRmPrivGetUs() - s_Profile.StartUs[(ProfileId)]); \
    }\
} while(0)

static DfsProfile s_Profile = {{0}};

#else

#define DfsProfileInit(pDfs) 
#define DfsProfileStart(pDfs, ProfileId) 
#define DfsProfileSample(pDfs, ProfileId) 
#endif

/*****************************************************************************/

#if DFS_HINTS_PRINTF

#define DfsHintsPrintInit() \
do\
{\
    s_DfsDomainNames[NvRmDfsClockId_Cpu] = "Cpu"; \
    s_DfsDomainNames[NvRmDfsClockId_Avp] = "Avp"; \
    s_DfsDomainNames[NvRmDfsClockId_System] = "Sys"; \
    s_DfsDomainNames[NvRmDfsClockId_Ahb] = "Ahb"; \
    s_DfsDomainNames[NvRmDfsClockId_Apb] = "Apb"; \
    s_DfsDomainNames[NvRmDfsClockId_Vpipe] = "Vde"; \
    s_DfsDomainNames[NvRmDfsClockId_Emc] = "Emc"; \
} while (0);

static char* s_DfsDomainNames[NvRmDfsClockId_Num];

static void ClientTagToString(NvU32 ClientTag, char ClientName[])
{
    NvU32 i;

    // Unpack in reverse order 4 caharacters from 32-bit DFS client tag
    for (i = 0; i < sizeof(ClientTag); i++)
    {
        NvU8 c = (NvU8)(ClientTag & 0xFF);
        ClientTag >>= 8;
        if ((c < ' ') || (c > 0x7F))
            c = '*'; // non-ASCII codes in non-initialized tags
        ClientName[sizeof(ClientTag) - 1 - i] = c;
    }
    ClientName[sizeof(ClientTag)] = 0x00;
}

#else
#define DfsHintsPrintInit()
#endif

/*****************************************************************************/

#if DFS_LOGGING_SECONDS

// Log size, assuming ~100 samples / sec
#define DFS_LOG_SIZE (100 * DFS_LOGGING_SECONDS)

typedef struct DfsLogEntryRec
{
    NvU32 SampleIntervalMs;
    NvU32 Lp2TimeMs;
    NvU32 ActiveCycles[NvRmDfsClockId_Num];
    NvRmDfsFrequencies CurrentKHz;
    NvRmDfsFrequencies AverageKHz;
} DfsLogEntry;

typedef struct DfsLogStarvationHintRec
{
    NvU32 LogSampleIndex;
    NvU32 ClientId;
    NvU32 ClientTag;
    NvRmDfsStarvationHint StarvationHint;
} DfsLogStarvationHint;

typedef struct DfsLogBusyHintRec
{
    NvU32 LogSampleIndex;
    NvU32 ClientId;
    NvU32 ClientTag;
    NvRmDfsBusyHint BusyHint;
} DfsLogBusyHint;

static NvU32 s_DfsLogWrIndex = 0;
static DfsLogEntry s_DfsLog[DFS_LOG_SIZE];

static NvU32 s_DfsLogStarvationWrIndex = 0;
static DfsLogStarvationHint s_DfsLogStarvation[DFS_LOG_SIZE];

static NvU32 s_DfsLogBusyWrIndex = 0;
static DfsLogBusyHint s_DfsLogBusy[DFS_LOG_SIZE];

#define DfsLogEnter(pDfs, Lp2Ms) \
do\
{\
    if (s_DfsLogOn && (s_DfsLogWrIndex < DFS_LOG_SIZE)) \
    {\
        NvU32 i; \
        DfsLogEntry* pEntry = &s_DfsLog[s_DfsLogWrIndex++]; \
        pEntry->SampleIntervalMs = *(pDfs)->SamplingWindow.pLastInterval; \
        pEntry->Lp2TimeMs = (Lp2Ms); \
        for (i = 1; i < NvRmDfsClockId_Num; i++) \
        { \
            pEntry->ActiveCycles[i] = *(pDfs)->Samplers[i].pLastSample; \
            pEntry->AverageKHz.Domains[i] = (pDfs)->Samplers[i].AverageKHz; \
        } \
        pEntry->CurrentKHz = (pDfs)->CurrentKHz; \
    }\
} while(0)

#else
#define DfsLogEnter(pDfs, Lp2TimeMs)
#endif

/*****************************************************************************/

#if NVRM_DTT_RANGE_CHANGE_PRINTF

#define DttRangeReport(T, pDtt) \
do\
{\
    NvOsDebugPrintf("DTT: T = %d, Range = %d (%d : %d)\n", \
        (T), (pDtt)->TcorePolicy.PolicyRange, \
        (pDtt)->TcorePolicy.LowLimit, (pDtt)->TcorePolicy.HighLimit); \
} while(0)

#else
#define DttRangeReport(T, pDtt)
#endif

/*****************************************************************************/

// DFS object
static NvRmDfs s_Dfs;

// Execution Platform
static ExecPlatform s_Platform;

// Microsecond timer virtual address
static void* s_pTimerUs;

// NV DFS logging enabled indicator
static NvBool s_DfsLogOn = NV_FALSE;


/*****************************************************************************/
/*****************************************************************************/

/*
 * Gets monitoring capabilities of the DFS module
 */
static NvError SystatMonitorsGetCapabilities(NvRmDfs* pDfs);
static NvError VdeMonitorsGetCapabilities(NvRmDfs* pDfs);
static NvError EmcMonitorsGetCapabilities(NvRmDfs* pDfs);

/*                         
 *  Gets monitoring capabilities of all DFS modules
 */
static NvError DfsGetModulesCapabilities(NvRmDfs* pDfs);

/*
 * Initializes all DFS HW monitors
 */
static NvError DfsHwInit(NvRmDfs* pDfs);

/*
 * Deinitializes all DFS HW monitors
 */
static void DfsHwDeinit(NvRmDfs* pDfs);

/*
 * Starts activity monitors in all DFS modules for the next sample interval
 * and enables DFS interrupt
 */
static void
DfsStartMonitors(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs);

/*
 * Reads idle count from activity monitors in all DFS modules. The monitors are
 * stopped.
 */
static void
DfsReadMonitors(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData);

/*****************************************************************************/

/*
 * Initializes DFS algorithm parameters
 */
static void DfsParametersInit(NvRmDfs* pDfs);

/*
 * Initializes DFS samplers for specified frequencies
 */
static void DfsSamplersInit(
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfs* pDfs);

/*****************************************************************************/

/*
 * DFS ISR (executes DFS algorithm)
 */
static void DfsIsr(void* args);

/*
 * Determines target frequencies for DFS domains
 */
static NvBool
DfsGetTargetFrequencies(
    const NvRmDfsIdleData* pIdleData,
    NvRmDfs* pDfs,
    NvRmDfsFrequencies* pDfsKHz);

/*
 * Adds new sample interval to the sample window
 */
static NvBool
AddSampleInterval(
    NvRmDfsSampleWindow* pSampleWindow,
    NvU32 IntervalMs);

/*
 * Adds new activity sample to the domain buffer
 */
static void
AddActivitySample(
    NvRmDfsSampler* pDomainSampler,
    NvU32 ActiveCount);

// Determine PM thread request for CPU state control
static NvRmPmRequest 
DfsGetPmRequest(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz* pCpuKHz);

/*****************************************************************************/

/*
 * DFS clock control thread entry point and termination function
 */ 
static NvRmPmRequest DfsThread(NvRmDfs* pDfs);
static void DfsThreadTerminate(NvRmDfs* pDfs);

/*
 * Returns current frequencies of DFS clocks
 */
static void
DfsClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsFrequencies* pDfsKHz);

/*
 * Configures DFS clocks according to target frequencies,
 * and returns actual frequencies
 */
static NvBool
DfsClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsFrequencies* pMaxKHz,
    NvRmDfsFrequencies* pDfsKHz);

/*
 * Clips EMC frequency high limit to one of the fixed DFS EMC configurations,
 * and if necessary adjust CPU high limit respectively.
 */
static void
DfsClipCpuEmcHighLimits(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz* pCpuHighKHz,
    NvRmFreqKHz* pEmcHighKHz);

/*
 * Emulate sampling results to achieve specified average frequency
 * provided it is bigger than the current one 
 */
static void
DfsSetAverageUp(
    NvRmDfsClockId ClockId,
    NvRmFreqKHz AverageKHz,
    NvRmDfs* pDfs);

/*
 * Changes core and rtc voltages, keeping them in synch
 */
static void
DvsChangeCoreVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv);

/*
 * Changes dedicated cpu rail voltage
 */
static void
DvsChangeCpuVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv);

/* 
 * Enable/Disable voltage scaling
 */
static void NvRmPrivDvsRun(void);
static void NvRmPrivDvsStopAtNominal(void);

/*
 * Updates thermal state and temperature monitoring policies according
 * to the new sampled temperature.
 */
static void
DttPolicyUpdate(
    NvRmDeviceHandle hRm,
    NvS32 TemperatureC,
    NvRmDtt* pDtt);

/*
 * Updates (throttles) target DFS frequencies based on SoC temperature.
 */
static NvBool
DttClockUpdate(
    const NvRmDfs* pDfs,
    NvRmDtt* pDtt,
    NvRmDfsFrequencies* pDfsKHz);

/*
 * DTT interrupt handler
 */
static void DttIntrCallback(void* args);

/*****************************************************************************/
// MONITORING CAPABILITIES
/*****************************************************************************/

static NvError SystatMonitorsGetCapabilities(NvRmDfs* pDfs)
{
    NvError error;

    NvRmModuleID ModuleId;
    NvRmDfsModule* pCaps;
    NvRmDfsModule SystatCaps[1] = {{{0}}};
    NvRmModuleCapability ModuleCaps[1];
    NvRmModuleTable *tbl;

    NvRmDeviceHandle hRm = pDfs->hRm;

    tbl = NvRmPrivGetModuleTable( hRm );

    /*
     * System Statistic module includes activity monitors for CPU, AVP, AHB,
     * and APB domains. Its presence is required for DFS to work.   
     */
    SystatCaps[0].DomainMap[NvRmDfsClockId_Cpu] = NV_TRUE;
    SystatCaps[0].DomainMap[NvRmDfsClockId_Avp] = NV_TRUE;
    SystatCaps[0].DomainMap[NvRmDfsClockId_Ahb] = NV_TRUE;
    SystatCaps[0].DomainMap[NvRmDfsClockId_Apb] = NV_TRUE;
    SystatCaps[0].Init = NvRmPrivAp15SystatMonitorsInit;
    SystatCaps[0].Deinit = NvRmPrivAp15SystatMonitorsDeinit;
    SystatCaps[0].Start = NvRmPrivAp15SystatMonitorsStart;
    SystatCaps[0].Read = NvRmPrivAp15SystatMonitorsRead;

    ModuleCaps[0].MajorVersion = 1;
    ModuleCaps[0].MinorVersion = 0;
    ModuleCaps[0].EcoLevel = 0;
    ModuleCaps[0].Capability = (void*)SystatCaps;

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_SysStatMonitor, 0);
    error = NvRmModuleGetCapabilities(hRm, ModuleId, ModuleCaps, 1, (void **)&pCaps);
    if (error != NvSuccess)
    {
        // Get capabilities failed - module is not present, DFS can not start
        return error;
    }
    pCaps->pBaseReg = (tbl->ModInst +
         tbl->Modules[NvRmModuleID_SysStatMonitor].Index)->VirtAddr;

    // AP15/AP16 h/w bug 429585 - time spent by CPU in LP2 is not counted
    // as idle - need explicitly offset monitor readings
    if ((pDfs->hRm->ChipId.Id == 0x15) || (pDfs->hRm->ChipId.Id == 0x16))
    {
        pCaps->Offset = NVRM_CPU_IDLE_LP2_OFFSET;
    }
    pDfs->Modules[NvRmDfsModuleId_Systat] = *pCaps;
    return NvSuccess;
}

static NvError VdeMonitorsGetCapabilities(NvRmDfs* pDfs)
{
    NvError error;

    NvRmModuleID ModuleId;
    NvRmDfsModule* pCaps;
    NvRmDfsModule VdeCaps[2] = {{{0}}};
    NvRmModuleCapability ModuleCaps[3];
    NvRmModuleTable *tbl;

    NvRmDeviceHandle hRm = pDfs->hRm;

    tbl = NvRmPrivGetModuleTable( hRm );

    /*
     * VDE module includes activity monitor for video-pipe domain. This
     * monitor may or may not be present on different versions of VDE
     */
    VdeCaps[0].DomainMap[NvRmDfsClockId_Vpipe] = NV_FALSE;

    VdeCaps[1].DomainMap[NvRmDfsClockId_Vpipe] = NV_TRUE;
    VdeCaps[1].Init = NvRmPrivAp15VdeMonitorsInit;
    VdeCaps[1].Deinit = NvRmPrivAp15VdeMonitorsDeinit;
    VdeCaps[1].Start = NvRmPrivAp15VdeMonitorsStart;
    VdeCaps[1].Read = NvRmPrivAp15VdeMonitorsRead;

    ModuleCaps[0].MajorVersion = 1; // AP15 A01
    ModuleCaps[0].MinorVersion = 0;
    ModuleCaps[0].EcoLevel = 0;
    ModuleCaps[0].Capability = (void*)&VdeCaps[1];

    ModuleCaps[1].MajorVersion = 1; // AP15 A02 (same caps as AP15 A01)
    ModuleCaps[1].MinorVersion = 1;
    ModuleCaps[1].EcoLevel = 0;
    ModuleCaps[1].Capability = (void*)&VdeCaps[1];

    ModuleCaps[2].MajorVersion = 1; // AP20 (same caps as AP15 A01)
    ModuleCaps[2].MinorVersion = 2;
    ModuleCaps[2].EcoLevel = 0;
    ModuleCaps[2].Capability = (void*)&VdeCaps[1];

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Vde, 0);
    error = NvRmModuleGetCapabilities(hRm, ModuleId, ModuleCaps, 3, (void **)&pCaps);

    if (error == NvSuccess)
    {
        if (pCaps->DomainMap[NvRmDfsClockId_Vpipe])
        {
            pCaps->pBaseReg =
                (tbl->ModInst + tbl->Modules[NvRmModuleID_Vde].Index)->VirtAddr;
        }
    }
    else
    {
        // If get capabilities failed, set "not present" cpabilities 
        pCaps = &VdeCaps[0];
    }
    pDfs->Modules[NvRmDfsModuleId_Vde] = *pCaps;
    return NvSuccess;
}

static NvError EmcMonitorsGetCapabilities(NvRmDfs* pDfs)
{
    NvError error;

    NvRmModuleID ModuleId;
    NvRmDfsModule* pCaps;
    NvRmDfsModule EmcCaps[3] = {{{0}}};
    NvRmModuleCapability ModuleCaps[3];
    NvRmModuleTable *tbl;

    NvRmDeviceHandle hRm = pDfs->hRm;

    tbl = NvRmPrivGetModuleTable( hRm );

    /*
     * EMC module includes activity monitor for EMC clock domain. This
     * monitor may, or may not be present on different versions of EMC 
     */
    EmcCaps[0].DomainMap[NvRmDfsClockId_Emc] = NV_FALSE;

    EmcCaps[1].DomainMap[NvRmDfsClockId_Emc] = NV_TRUE;
    EmcCaps[1].Init = NvRmPrivAp15EmcMonitorsInit;
    EmcCaps[1].Deinit = NvRmPrivAp15EmcMonitorsDeinit;
    EmcCaps[1].Start = NvRmPrivAp15EmcMonitorsStart;
    EmcCaps[1].Read = NvRmPrivAp15EmcMonitorsRead;

    EmcCaps[2].DomainMap[NvRmDfsClockId_Emc] = NV_TRUE;
    EmcCaps[2].Init = NvRmPrivAp20EmcMonitorsInit;
    EmcCaps[2].Deinit = NvRmPrivAp20EmcMonitorsDeinit;
    EmcCaps[2].Start = NvRmPrivAp20EmcMonitorsStart;
    EmcCaps[2].Read = NvRmPrivAp20EmcMonitorsRead;

    ModuleCaps[0].MajorVersion = 1; // AP15 A01       
    ModuleCaps[0].MinorVersion = 0;
    ModuleCaps[0].EcoLevel = 0;
    ModuleCaps[0].Capability = (void*)&EmcCaps[1];

    ModuleCaps[1].MajorVersion = 1; // AP15 A02 (same caps as AP15 A01)
    ModuleCaps[1].MinorVersion = 1;
    ModuleCaps[1].EcoLevel = 0;
    ModuleCaps[1].Capability = (void*)&EmcCaps[1];

    ModuleCaps[2].MajorVersion = 1; // AP20 EMC
    ModuleCaps[2].MinorVersion = 2;
    ModuleCaps[2].EcoLevel = 0;
    ModuleCaps[2].Capability = (void*)&EmcCaps[2];

    ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_ExternalMemoryController, 0);
    error = NvRmModuleGetCapabilities(hRm, ModuleId, ModuleCaps, 3, (void **)&pCaps);

    if (error == NvSuccess)
    {
        if (pCaps->DomainMap[NvRmDfsClockId_Emc])
        {       
            pCaps->pBaseReg = (tbl->ModInst +
                tbl->Modules[NvRmPrivModuleID_ExternalMemoryController].Index)->VirtAddr;
        }
    }
    else
    {
        // If get capabilities failed, set "not present" cpabilities 
        pCaps = &EmcCaps[0];
    }
    pDfs->Modules[NvRmDfsModuleId_Emc] = *pCaps;
    return NvSuccess;
}

static NvError DfsGetModulesCapabilities(NvRmDfs* pDfs)
{
    NvError error = SystatMonitorsGetCapabilities(pDfs);
    if (error == NvSuccess)
    {
        error = VdeMonitorsGetCapabilities(pDfs);
    }
    if (error == NvSuccess)
    {
        error = EmcMonitorsGetCapabilities(pDfs);
    }
    return error;
}

/*****************************************************************************/
// DFS INITIALIZATION PROCEDURES
/*****************************************************************************/

static void DfsParametersInit(NvRmDfs* pDfs)
{
    NvU32 i;
    NvRmModuleClockLimits HwLimitsKHz[NvRmDfsClockId_Num];
    const NvRmModuleClockLimits* pClimits;

    // TODO: ODM query for parameters initialization?

    // Macro to initialize scaling algorithm parameters
    #define INIT_PARAM(Domain, DOMAIN) \
    do \
    {  \
        if ((pDfs->hRm->ChipId.Id == 0x15) || (pDfs->hRm->ChipId.Id == 0x16)) \
        { \
            NvRmDfsParam dp = { NVRM_DFS_PARAM_##DOMAIN##_AP15 }; \
            pDfs->DfsParameters[NvRmDfsClockId_##Domain] = dp; \
        } \
        else if (pDfs->hRm->ChipId.Id == 0x20) \
        { \
            NvRmDfsParam dp = { NVRM_DFS_PARAM_##DOMAIN##_AP20 }; \
            pDfs->DfsParameters[NvRmDfsClockId_##Domain] = dp; \
        } \
        else \
            NV_ASSERT(!"Unsupported chip ID"); \
    } while(0)

    // Initialize scaling algorithm parameters for DFS domains
    INIT_PARAM(Cpu, CPU);
    INIT_PARAM(Avp, AVP);
    INIT_PARAM(System, SYSTEM);
    INIT_PARAM(Ahb, AHB);
    INIT_PARAM(Apb, APB);
    INIT_PARAM(Vpipe, VPIPE);
    INIT_PARAM(Emc, EMC);

    #undef INIT_PARAM

    // Adjust EMC parameters as required for particular SDRAM type
    if (pDfs->hRm->ChipId.Id == 0x20)
        NvRmPrivAp20EmcParametersAdjust(pDfs);

    // Update minimum frequency boundary for DFS clocks as required for
    // download transport support
    switch (NvRmPrivGetDownloadTransport(pDfs->hRm))
    {
        case NvOdmDownloadTransport_Ethernet:
            if ((pDfs->hRm->ChipId.Id == 0x15) &&
                (pDfs->hRm->ChipId.Major == 0x01) &&
                (pDfs->hRm->ChipId.Minor == 0x01))
            {
                pDfs->DfsParameters[NvRmDfsClockId_Apb].MinKHz =
                pDfs->DfsParameters[NvRmDfsClockId_Ahb].MinKHz =
                    NVRM_ETHERNET_AHB_MIN_KHZ;
                pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz =
                    NVRM_ETHERNET_EMC_MIN_KHZ;
            }
            break;
        case NvOdmDownloadTransport_Usb:
            pDfs->DfsParameters[NvRmDfsClockId_Apb].MinKHz =
            pDfs->DfsParameters[NvRmDfsClockId_Ahb].MinKHz =
            pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz =
                NVRM_USB_AHB_MIN_KHZ;
            break;
        case NvOdmDownloadTransport_Spi:
            pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz = NV_MAX(
                pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz,
                NVRM_SPI_CPU_MIN_KHZ);
            if (pDfs->hRm->ChipId.Id == 0x20)
                pDfs->DfsParameters[NvRmDfsClockId_Apb].MinKHz =
                    NVRM_SPI_APB_MIN_KHZ;
            break;
        default:
            break;
    }

    // CPU clock H/w limits
    pClimits = NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu);
    HwLimitsKHz[NvRmDfsClockId_Cpu] = *pClimits;

    // System clock H/w limits are applied to AVP, AHB, and APB
    pClimits = NvRmPrivGetSocClockLimits(NvRmPrivModuleID_System);
    HwLimitsKHz[NvRmDfsClockId_System] = *pClimits;
    HwLimitsKHz[NvRmDfsClockId_Avp] = *pClimits;
    HwLimitsKHz[NvRmDfsClockId_Ahb] = *pClimits;
    HwLimitsKHz[NvRmDfsClockId_Apb] = *pClimits;

    // V-pipe clock H/w limits
    pClimits = NvRmPrivGetSocClockLimits(NvRmModuleID_Vde);
    HwLimitsKHz[NvRmDfsClockId_Vpipe] = *pClimits;

    // EMC clock H/w limits (the limit table specifies EMC2x limits); on SoC
    // PLLM0 is used as a high limit for DFS
    pClimits =
        NvRmPrivGetSocClockLimits(NvRmPrivModuleID_ExternalMemoryController);
    HwLimitsKHz[NvRmDfsClockId_Emc].MaxKHz = pClimits->MaxKHz / 2;
    HwLimitsKHz[NvRmDfsClockId_Emc].MinKHz = pClimits->MinKHz / 2;
    if (s_Platform == ExecPlatform_Soc)
    {
        HwLimitsKHz[NvRmDfsClockId_Emc].MaxKHz =
            NvRmPrivGetClockSourceFreq(NvRmClockSource_PllM0) / 2;
    }

    // Clip requested clock boundaries to h/w limits, and initialize
    // low/high corner with minimum/maximum domain frequencies
    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        if (pDfs->DfsParameters[i].MaxKHz > HwLimitsKHz[i].MaxKHz)
            pDfs->DfsParameters[i].MaxKHz = HwLimitsKHz[i].MaxKHz;
        if (pDfs->DfsParameters[i].MinKHz < HwLimitsKHz[i].MinKHz)
            pDfs->DfsParameters[i].MinKHz = HwLimitsKHz[i].MinKHz;
        pDfs->LowCornerKHz.Domains[i] = pDfs->DfsParameters[i].MinKHz;
        pDfs->HighCornerKHz.Domains[i] = pDfs->DfsParameters[i].MaxKHz;
    }
    pDfs->CpuCornersShadow.MinKHz =
        pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu];
    pDfs->CpuCornersShadow.MaxKHz =
        pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu];

#if NVRM_EMC_DFS_DEFAULT_DISABLED
    pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Emc] =
        pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz;
#endif
    pDfs->CpuEnvelopeSet = NV_FALSE;
    pDfs->EmcEnvelopeSet = NV_FALSE;

    // Set initial boundaries for sampling interval
    pDfs->SamplingWindow.MinIntervalMs = NVRM_DFS_MIN_SAMPLE_MS;
    pDfs->SamplingWindow.MaxIntervalMs = NVRM_DFS_MAX_SAMPLE_MS;
    if (pDfs->hRm->ChipId.Id == 0x20) // constant for AP20 (TODO: revisit)
        pDfs->SamplingWindow.MaxIntervalMs = NVRM_DFS_MIN_SAMPLE_MS;

    // Fill in maximum DFS domains frequencies shortcut
    for (i = 1; i < NvRmDfsClockId_Num; i++)
        pDfs->MaxKHz.Domains[i] = pDfs->DfsParameters[i].MaxKHz;
}

static void DfsSamplersInit(
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfs* pDfs)
{
    NvU32 i, j, msec;
    NvRmDfsSampleWindow* pSampleWindow;

    /*
     * Clear Low Power Corner indicators, initilize current
     * and target frequencies
     */
    pDfs->LowCornerHit = NV_FALSE;
    pDfs->LowCornerReport = NV_FALSE;
    NvRmPrivUpdateDfsPauseFlag(pDfs->hRm, NV_FALSE);

    pDfs->CurrentKHz = *pDfsKHz;
    pDfs->TargetKHz = pDfs->CurrentKHz;

    /*
     * Initialize one full sampling window before DFS start. Use minimum
     * sampling interval.
     */
    pSampleWindow = &pDfs->SamplingWindow;
    msec = pSampleWindow->MinIntervalMs;
    pSampleWindow->NextIntervalMs = msec;
    for (j = 0; j < NVRM_DFS_MAX_SAMPLES; j++)
    {
        pSampleWindow->IntervalsMs[j] = msec;
    }
    pSampleWindow->pLastInterval = pSampleWindow->IntervalsMs;
    pSampleWindow->SampleWindowMs = (msec << NVRM_DFS_MAX_SAMPLES_LOG2);
    pSampleWindow->BusyCheckLastUs = 0;
    pSampleWindow->BusyCheckDelayUs = 0;

    /*
     * Initialize domain samplers
     */
    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        NvRmFreqKHz khz = pDfs->CurrentKHz.Domains[i];
        NvRmDfsSampler* pSampler = &pDfs->Samplers[i];
        NvU32 cycles = khz * msec;

        // Clear busy boost
        pDfs->BusyKHz.Domains[i] = 0;

        // Store DFS Clock Id
        pSampler->ClockId = i;

        // Use modules capabilities to determine if domain monitor is present
        for (j = 1; j < NvRmDfsModuleId_Num; j++)
        {
            pSampler->MonitorPresent |=
                pDfs->Modules[j].DomainMap[i];
        }

        // Initialize sampler data assuming constant current frequency
        // for one sampling window before the DFS start
        for (j = 0; j < NVRM_DFS_MAX_SAMPLES; j++)
        {
            pSampler->Cycles[j] = cycles;
        }
        pSampler->pLastSample = pSampler->Cycles;
        pSampler->TotalActiveCycles = (cycles << NVRM_DFS_MAX_SAMPLES_LOG2);
        if (pSampler->MonitorPresent)
        {
            pSampler->AverageKHz = khz;
            pSampler->BumpedAverageKHz = khz;
        }
        else
        {
            // For domain without monitor, average frequency is unspecified
            // and low corner is used as a base for target clalculation
            pSampler->AverageKHz = NvRmFreqUnspecified;
            pSampler->BumpedAverageKHz = pDfs->LowCornerKHz.Domains[i];
        }
        pSampler->NrtSampleCounter = 0;
        pSampler->NrtStarveBoostKHz = 0;
        pSampler->RtStarveBoostKHz = 0;
        pSampler->BusyPulseMode = NV_FALSE;
    }
}

static NvError DfsHwInit(NvRmDfs* pDfs)
{
    NvU32 i;
    NvError error = NvSuccess;

    s_pTimerUs = NvRmPrivAp15GetTimerUsVirtAddr(pDfs->hRm);

    for (i = 1; i < NvRmDfsModuleId_Num; i++)
    {
        if (pDfs->Modules[i].Init)
        {
            error = pDfs->Modules[i].Init(pDfs);
            if (error != NvSuccess)
            {
                break;
            }
        }
    }
    return error;
}

static void DfsHwDeinit(NvRmDfs* pDfs)
{
    NvU32 i;

    if (pDfs && pDfs->hRm)
    {
        for (i = 1; i < NvRmDfsModuleId_Num; i++)
        {
            if (pDfs->Modules[i].Deinit)
            {
                pDfs->Modules[i].Deinit(pDfs);
            }
        }
    }
}

/*****************************************************************************/
// DFS ALGORITHM IMPLEMENTATION
/*****************************************************************************/

static void
DfsStartMonitors(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs)
{
    NvU32 i;

    for (i = 1; i < NvRmDfsModuleId_Num; i++)
    {
        FuncPtrModuleMonitorsStart start = pDfs->Modules[i].Start;
        if (start)
        {
            start(pDfs, pDfsKHz, IntervalMs);
        }
    }
}

static void
DfsReadMonitors(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData)
{
    NvU32 i;

    for (i = 1; i < NvRmDfsModuleId_Num; i++)
    {
        FuncPtrModuleMonitorsRead read = pDfs->Modules[i].Read;
        if (read)
        {
            read(pDfs, pDfsKHz, pIdleData);
        }
    }
}

static NvRmPmRequest 
DfsGetPmRequest(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz* pCpuKHz)
{
    if (hRmDevice->ChipId.Id == 0x20)
    {
        return NvRmPrivAp20GetPmRequest(hRmDevice, pCpuSampler, pCpuKHz);
    }
    return NvRmPmRequest_None;
}

static NvBool
DfsGetTargetFrequencies(
    const NvRmDfsIdleData* pIdleData,
    NvRmDfs* pDfs,
    NvRmDfsFrequencies* pDfsKHz)
{
    static NvRmDfsFrequencies LastKHz = {{0}};

    NvU32 i;
    NvBool BusyCheckTime;
    NvBool ReturnValue = NV_FALSE;
    NvBool LowCornerHit = NV_TRUE;
    NvU32 msec = pIdleData->CurrentIntervalMs;
    NvU32 usec = NvRmPrivGetUs();

    // Add current sample interval to sampling window; always signal to clock
    // control thread if window wraparound; check busy hints expirtaion time
    ReturnValue = AddSampleInterval(&pDfs->SamplingWindow, msec);
    pDfs->SamplingWindow.SampleCnt++; 
    BusyCheckTime = pDfs->SamplingWindow.BusyCheckDelayUs <
        (usec - pDfs->SamplingWindow.BusyCheckLastUs);

    // Update thermal throttling polling control
    if (!NVRM_DTT_DISABLED && pDfs->ThermalThrottler.hOdmTcore)
    {
        if (pDfs->ThermalThrottler.TcorePolicy.UpdateIntervalUs <
            (usec - pDfs->ThermalThrottler.TcorePolicy.TimeUs))
        {
            pDfs->ThermalThrottler.TcorePolicy.TimeUs = usec;
            pDfs->ThermalThrottler.TcorePolicy.UpdateFlag = NV_TRUE;
        }
    }

    // Update cumulative log time (including LP2 time)
    if (s_DfsLogOn)
    {
        pDfs->SamplingWindow.CumulativeLogMs +=
            (pIdleData->CurrentIntervalMs + pIdleData->Lp2TimeMs);
        if (pIdleData->Lp2TimeMs)
        {
            pDfs->SamplingWindow.CumulativeLp2TimeMs += pIdleData->Lp2TimeMs;
            pDfs->SamplingWindow.CumulativeLp2Entries++;
        }
    }
    // Update LP2 indicator to synchronize DVFS state with dedicated CPU
    // rail after LP2 exit (required if CPU rail returns to default level
    // by PMU underneath DVFS on every LP2 exit)
    if (pIdleData->Lp2TimeMs && pDfs->VoltageScaler.VCpuOTPOnWakeup &&
        NvRmPrivIsCpuRailDedicated(pDfs->hRm))
    {
        pDfs->VoltageScaler.Lp2SyncOTPFlag = NV_TRUE;
        pDfs->VoltageScaler.UpdateFlag = NV_TRUE;
    }

    // Determine target frequency for each DFS domain
    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        NvRmDfsSampler* pDomainSampler = &pDfs->Samplers[i];
        NvRmDfsParam* pDomainParam = &pDfs->DfsParameters[i];
        NvRmFreqKHz* pDomainKHz = &pDfsKHz->Domains[i];
        NvRmFreqKHz CurrentDomainKHz = *pDomainKHz;
        NvRmFreqKHz LowCornerDomainKHz = pDfs->LowCornerKHz.Domains[i];
        NvRmFreqKHz HighCornerDomainKHz = pDfs->HighCornerKHz.Domains[i];
        NvRmFreqKHz DomainBusyKHz = pDfs->BusyKHz.Domains[i]; // from dfs thread

        /*
         * Find and adjust average activity frequency over the sampling
         * window
         */
        if (pDomainSampler->MonitorPresent)
        {
            NvU32 IdleCount = pIdleData->Readings[i];
            NvU32 ActiveCount = msec * CurrentDomainKHz; // max if never idle

            // Update cumulative number of cycles
            if (s_DfsLogOn)
                pDomainSampler->CumulativeLogCycles +=
                    (ActiveCount + pIdleData->Lp2TimeMs * CurrentDomainKHz);

            // Raw average = Sum(Activity Counts within sampling window)
            // divided by Sum(Sampling Intervals within sampling window)
            ActiveCount =
                (ActiveCount > IdleCount) ? (ActiveCount - IdleCount) : (0);
#if NVRM_DFS_STALL_AVERAGE_IN_BUSY_PULSE
            if (!pDomainSampler->BusyPulseMode) 
#endif
            {
                AddActivitySample(pDomainSampler, ActiveCount);
            }

            pDomainSampler->AverageKHz = (NvU32)NvDiv64(pDomainSampler->TotalActiveCycles,
                    pDfs->SamplingWindow.SampleWindowMs);

            // Check non real-time starvation
            if ((IdleCount >= (1 + (ActiveCount >> pDomainParam->RelAdjustBits))) &&
                (pDomainSampler->BumpedAverageKHz >= pDomainSampler->AverageKHz))
            {
                pDomainSampler->NrtSampleCounter = 0;
                if (pDomainSampler->NrtStarveBoostKHz != 0)
                {
                    // Domain is not starving, previously added boost has not been
                    // removed, yet - decrease starvation boost proportionally
                    pDomainSampler->NrtStarveBoostKHz = (pDomainSampler->NrtStarveBoostKHz *
                     ((0x1 << BOOST_FRACTION_BITS) - pDomainParam->NrtStarveParam.BoostDecKoef))
                      >> BOOST_FRACTION_BITS;

                    if (pDomainSampler->NrtStarveBoostKHz <
                        pDomainParam->NrtStarveParam.BoostStepKHz)
                        pDomainSampler->NrtStarveBoostKHz = 0;  // cut tail
                }
            }
            else if (pDomainSampler->NrtSampleCounter < pDomainParam->MinNrtSamples)
            {
                pDomainSampler->NrtSampleCounter++;
            }
            else
            {
                // Domain is starving - increase starvation boost
                // (proportionally plus a fixed step)
                pDomainSampler->NrtStarveBoostKHz = ((pDomainSampler->NrtStarveBoostKHz *
                 ((0x1 << BOOST_FRACTION_BITS) + pDomainParam->NrtStarveParam.BoostIncKoef))
                  >> BOOST_FRACTION_BITS) + pDomainParam->NrtStarveParam.BoostStepKHz;

                // Make sure the boost value is within domain limits
                if (pDomainSampler->NrtStarveBoostKHz > pDomainParam->MaxKHz)
                    pDomainSampler->NrtStarveBoostKHz = pDomainParam->MaxKHz;
            }

            // Average frequency change is recognized by DFS only if it exceeds
            // tolerance band. 
            if ((pDomainSampler->AverageKHz + pDomainParam->LowerBandKHz) <
                pDomainSampler->BumpedAverageKHz)
            {
                pDomainSampler->BumpedAverageKHz =
                    pDomainSampler->AverageKHz + pDomainParam->LowerBandKHz;
            }
            else if (pDomainSampler->AverageKHz > 
                (pDomainSampler->BumpedAverageKHz + pDomainParam->UpperBandKHz))
            {
                pDomainSampler->BumpedAverageKHz =
                    pDomainSampler->AverageKHz - pDomainParam->UpperBandKHz;
            }

            // Adjust average frequency up, to probe non real-time starvation
            pDomainSampler->BumpedAverageKHz +=
                (pDomainSampler->BumpedAverageKHz >> pDomainParam->RelAdjustBits);
        }
        else
        {
            // For domain without monitor average frequency is unspecified
            // and low corner is used as a base for target clalculation
            pDomainSampler->AverageKHz = NvRmFreqUnspecified;
            pDomainSampler->BumpedAverageKHz = LowCornerDomainKHz;
        }

        /*
         * Check real time starvation
         */
        if(NvRmPrivDfsIsStarving(i))
        {
            // Domain is starving - increase starvation boost (proportionally
            // plus a fixed step)
            pDomainSampler->RtStarveBoostKHz = ((pDomainSampler->RtStarveBoostKHz *
             ((0x1 << BOOST_FRACTION_BITS) + pDomainParam->RtStarveParam.BoostIncKoef))
              >> BOOST_FRACTION_BITS) + pDomainParam->RtStarveParam.BoostStepKHz;

            // Make sure the boost value is within domain limits
            if (pDomainSampler->RtStarveBoostKHz > pDomainParam->MaxKHz)
                pDomainSampler->RtStarveBoostKHz = pDomainParam->MaxKHz;
        }
        else if (pDomainSampler->RtStarveBoostKHz != 0)
        {
            // Domain is not starving, previously added boost has not been
            // removed, yet - decrease starvation boost proportionally
            pDomainSampler->RtStarveBoostKHz = (pDomainSampler->RtStarveBoostKHz *
             ((0x1 << BOOST_FRACTION_BITS) - pDomainParam->RtStarveParam.BoostDecKoef))
              >> BOOST_FRACTION_BITS;
        }

        /*
         * Combine average, starvation and busy demands into target frequency,
         * and clip it to the domain limits. Check low power corner hit. Set
         * return value if clock update is necessary.
         */
        *pDomainKHz = NV_MAX(pDomainSampler->BumpedAverageKHz,
                             LowCornerDomainKHz);
        if (pDomainSampler->RtStarveBoostKHz >= pDomainSampler->NrtStarveBoostKHz)
        {
            *pDomainKHz += pDomainSampler->RtStarveBoostKHz;
        }
        else
        {
            *pDomainKHz += pDomainSampler->NrtStarveBoostKHz;
        }

        if ((*pDomainKHz) < DomainBusyKHz)
        {
            (*pDomainKHz) = DomainBusyKHz;
        }
        if ((*pDomainKHz) > HighCornerDomainKHz)
        {
            *pDomainKHz = HighCornerDomainKHz;
        }

        /*
         * Determine if low corner is hit in this domain - clear hit indicator
         * if new target domain frequency is above low limit (with hysteresis)
         * For platform with dedicated CPU partition do not include activity
         * margin when there is no busy or starvation requirements
         */
        if (NvRmPrivIsCpuRailDedicated(pDfs->hRm) &&
            (DomainBusyKHz <= LowCornerDomainKHz) &&
            ((*pDomainKHz) == pDomainSampler->BumpedAverageKHz))
        {
            // Multiplying threshold has the same effect as dividing target
            // to reduce margin
            LowCornerDomainKHz +=
                (LowCornerDomainKHz >> pDomainParam->RelAdjustBits);
        }
        if ( ((*pDomainKHz) > 
              (LowCornerDomainKHz + pDomainParam->NrtStarveParam.BoostStepKHz))
             || (((*pDomainKHz) > LowCornerDomainKHz) && (!pDfs->LowCornerHit))
            )
        {
            LowCornerHit = NV_FALSE;
        }

        /*
         * Update PM request. Set return value if CPU power state change
         * is requested. 
         */
        if (i == NvRmDfsClockId_Cpu)
        {
            NvRmPmRequest r =
                DfsGetPmRequest(pDfs->hRm, pDomainSampler, pDomainKHz);
            if (r != NvRmPmRequest_None)
            {
                pDfs->PmRequest = r;
                ReturnValue = NV_TRUE;
            }
        }

        // Set return value, if the new target is outside the tolerance band
        // around the last recorded target, or if domain is busy
        ReturnValue = ReturnValue || (DomainBusyKHz && BusyCheckTime) ||
            (((*pDomainKHz) + pDomainParam->LowerBandKHz) <= LastKHz.Domains[i]) ||
            ((*pDomainKHz) >= (LastKHz.Domains[i] + pDomainParam->UpperBandKHz));
    }
    // Update low corner hit status if necessary
    if (pDfs->LowCornerHit != LowCornerHit)
    {
        pDfs->LowCornerHit = LowCornerHit;
        pDfs->LowCornerReport = NV_TRUE;
        ReturnValue = NV_TRUE;
    }
    // Update last recorded target if clock thread is to be signaled
    if (ReturnValue)
    {
        LastKHz = *pDfsKHz;
    }
    return ReturnValue;
}

static NvBool
AddSampleInterval(
    NvRmDfsSampleWindow* pSampleWindow,
    NvU32 IntervalMs)
{
    /*
     * Add current sampling interval to the sampling window (i.e., replace the
     * first/"oldest" interval with the new one and update window size). 
     */ 
    NvBool WrapAround = NV_FALSE;

    NvU32* pFirst = pSampleWindow->pLastInterval + 1; 
    if (pFirst >= &pSampleWindow->IntervalsMs[
        NV_ARRAY_SIZE(pSampleWindow->IntervalsMs)])
    {
        pFirst = pSampleWindow->IntervalsMs;
        WrapAround = NV_TRUE;
    }
    pSampleWindow->pLastInterval = pFirst;

    pSampleWindow->SampleWindowMs += IntervalMs;
    pSampleWindow->SampleWindowMs -= (*pFirst);
    *pFirst = IntervalMs;

    return WrapAround;
}

static void
AddActivitySample(
    NvRmDfsSampler* pDomainSampler,
    NvU32 ActiveCount)
{
    /*
     * Add new activity sample to the cicular buffer(i.e., replace the
     * first/"oldest" sample with the new one) and update total cycle count 
     */ 
    NvU32* pFirst = pDomainSampler->pLastSample + 1;
    if (pFirst >= &pDomainSampler->Cycles[
        NV_ARRAY_SIZE(pDomainSampler->Cycles)])
    {
        pFirst = pDomainSampler->Cycles;
    }
    pDomainSampler->pLastSample = pFirst;

    pDomainSampler->TotalActiveCycles += ActiveCount;
    pDomainSampler->TotalActiveCycles -= (*pFirst);
    *pFirst = ActiveCount;
}

static void DfsIsr(void* args)
{
    NvRmDfs* pDfs = (NvRmDfs*)args;
    NvBool ClockChange = NV_FALSE;
    NvRmDfsFrequencies DfsKHz;
    NvRmDfsIdleData IdleData;
    NvU32 msec;

    DfsProfileStart(pDfs, NvRmDfsProfileId_Isr);
    NvOsIntrMutexLock(pDfs->hIntrMutex);
    DfsProfileStart(pDfs, NvRmDfsProfileId_Algorithm);

    // Input to DFS algorithm from clock control thread: current frequencies
    DfsKHz = pDfs->CurrentKHz;

    // Adjust next sampling interval based on CPU domain frequency; keep it
    // minimum if NRT threshold was crossed during the last sample
    msec = pDfs->SamplingWindow.MinIntervalMs;
    if (pDfs->Samplers[NvRmDfsClockId_Cpu].NrtSampleCounter == 0)
    {
        if (DfsKHz.Domains[NvRmDfsClockId_Cpu] <
            (pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz +
             pDfs->DfsParameters[NvRmDfsClockId_Cpu].UpperBandKHz))
            msec = pDfs->SamplingWindow.MaxIntervalMs;
    }
    pDfs->SamplingWindow.NextIntervalMs = msec;

    // Read idle counts from  monitors, which clears DFS interrupt
    DfsReadMonitors(pDfs, &DfsKHz, &IdleData);

    if (pDfs->DfsRunState > NvRmDfsRunState_Stopped)
    {
        // If DFS is running re-start monitors, execute DFS algorithm, and 
        // determine new target frequencies for the clock control thread
        DfsStartMonitors(pDfs, &DfsKHz, msec);
        ClockChange = DfsGetTargetFrequencies(&IdleData, pDfs, &DfsKHz);
        pDfs->TargetKHz = DfsKHz;
        ClockChange = ClockChange || pDfs->VoltageScaler.UpdateFlag ||
            pDfs->ThermalThrottler.TcorePolicy.UpdateFlag;
    }
    DfsProfileSample(pDfs, NvRmDfsProfileId_Algorithm);
    DfsLogEnter(pDfs, IdleData.Lp2TimeMs);
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);

    // Signal clock control thread if clocks should be changed
    if (ClockChange)
    {
        NvOsSemaphoreSignal(pDfs->hSemaphore);
    }
    DfsProfileSample(pDfs, NvRmDfsProfileId_Isr);

    NvRmInterruptDone(pDfs->DfsInterruptHandle);
}

/*****************************************************************************/
// DFS CLOCK CONTROL THREAD
/*****************************************************************************/

static NvRmPmRequest DfsThread(NvRmDfs* pDfs)
{
    static NvRmDfsFrequencies LastKHz = {{0}};

    NvRmPowerEvent PowerEvent;
    NvRmDfsRunState DfsRunState;
    NvRmDfsFrequencies DfsKHz, HighKHz;
    NvBool LowCornerHit, LowCornerReport, NeedClockUpdate;
    NvU32 i, BusyCheckDelayMs;

    NvRmPmRequest PmRequest = NvRmPmRequest_None;

    // Thread has been initialized
    pDfs->InitializedThread = NV_TRUE;

    // CLOCK CONTROL EXECUTION LOOP //
    /********************************/
    {
        NvOsSemaphoreWait(pDfs->hSemaphore);
        if (pDfs->AbortThread)
        {
            pDfs->AbortThread = NV_FALSE;
            return NvRmPmRequest_ExitFlag;
        }
        DfsProfileStart(pDfs, NvRmDfsProfileId_Control);

        // Save traget frequency and DFS state variables, updated by ISR
        NvOsIntrMutexLock(pDfs->hIntrMutex);
        DfsKHz = pDfs->TargetKHz;
        HighKHz = pDfs->HighCornerKHz;
        DfsRunState = pDfs->DfsRunState;
        LowCornerHit = pDfs->LowCornerHit;
        LowCornerReport = pDfs->LowCornerReport;
        pDfs->LowCornerReport = NV_FALSE;
        PmRequest = pDfs->PmRequest;
        pDfs->PmRequest = NvRmPmRequest_None;
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);

        /*
         * On exit from low power state re-initialize DFS h/w, samplers, and
         * start monitors provided DFS is running. If DFS is stopped just get
         * DFS h/w ready. 
         */
        NV_ASSERT_SUCCESS(NvRmPowerGetEvent(
            pDfs->hRm, pDfs->PowerClientId, &PowerEvent));
        if (PowerEvent != NvRmPowerEvent_NoEvent)
        {
            // Full h/w re-initialization after LP0
            if (PowerEvent == NvRmPowerEvent_WakeLP0)
            {
                DfsHwDeinit(pDfs);
                NV_ASSERT_SUCCESS(DfsHwInit(pDfs));
            }
            // Re-initialize samplers if DVFS was running, but stopped on
            // entry to LPx; keep sampling history, if DVFS was not stopped;
            // restart monitors in either case
            NvRmPrivLockSharedPll();
            if (pDfs->DfsLPxSavedState > NvRmDfsRunState_Stopped)
            {
                DfsClockFreqGet(pDfs->hRm, &DfsKHz);

                NvOsIntrMutexLock(pDfs->hIntrMutex);
                if (pDfs->DfsRunState <= NvRmDfsRunState_Stopped)
                {
                    pDfs->DfsRunState = pDfs->DfsLPxSavedState;
                    DfsSamplersInit(&DfsKHz, pDfs);
                }
                NV_ASSERT(pDfs->DfsRunState == pDfs->DfsLPxSavedState);
                pDfs->CurrentKHz = DfsKHz;
                DfsStartMonitors(
                    pDfs, &DfsKHz, pDfs->SamplingWindow.MinIntervalMs);
                NvOsIntrMutexUnlock(pDfs->hIntrMutex);
            }
            NvRmPrivDvsRun();   // enable v-scaling even if DFS is stopped
            NvRmPrivUnlockSharedPll();
            return PmRequest;
        }

        /*
         * Advance busy hint state machine if DFS thread has been signaled by
         * synchronous busy hint.
         */
        if (pDfs->BusySyncState == NvRmDfsBusySyncState_Signal)
        {
            pDfs->BusySyncState = NvRmDfsBusySyncState_Execute;
            pDfs->VoltageScaler.UpdateFlag = NV_TRUE;
        }

        /*
         * When DFS is running evaluate busy boost and low corner status;
         * check if new target frequencies are significantly different from
         * the previously targeted.
         */
        if (DfsRunState > NvRmDfsRunState_Stopped)
        {
            NeedClockUpdate = NV_FALSE;
            BusyCheckDelayMs = NVRM_DFS_BUSY_PURGE_MS;
            
            for (i = 1; i < NvRmDfsClockId_Num; i++)
            {
                NvRmFreqKHz NewBusyKHz;
                NvBool NewPulseMode;
                NvU32 delay;
                NvRmFreqKHz OldBusyKHz = pDfs->BusyKHz.Domains[i];
                NvBool OldBusyPulseMode = pDfs->Samplers[i].BusyPulseMode;
                NvRmPrivDfsGetBusyHint(i, &NewBusyKHz, &NewPulseMode, &delay);

                if ((NewBusyKHz != 0) || (OldBusyKHz != 0))
                {
                    // When busy boost decreasing re-init average to the
                    // boosted level
                    if (NewBusyKHz < OldBusyKHz)
                    {
                        if (!OldBusyPulseMode)
                        {
                            NvU32 AverageKHz = OldBusyKHz - (OldBusyKHz / (1 +
                                (0x1 << pDfs->DfsParameters[i].RelAdjustBits)));
                            NvOsIntrMutexLock(pDfs->hIntrMutex);
                            DfsSetAverageUp(i, AverageKHz, pDfs);
                            NvOsIntrMutexUnlock(pDfs->hIntrMutex);
                        }
                        // Make sure new frequency to be set is above max busy
                        // and update DFS object
                        if (DfsKHz.Domains[i] < OldBusyKHz)
                        {
                            DfsKHz.Domains[i] = OldBusyKHz;
                        }
                    }
                    else
                    {
                        // Make sure new frequency to be set is above max busy
                        // and update DFS object
                        if (DfsKHz.Domains[i] < NewBusyKHz)
                        {
                            DfsKHz.Domains[i] = NewBusyKHz;
                        }
                    }
                    // Clip new dfs target to high domain corner
                    if (DfsKHz.Domains[i] > HighKHz.Domains[i])
                    {
                        DfsKHz.Domains[i] = HighKHz.Domains[i];
                    }
                    pDfs->BusyKHz.Domains[i] = NewBusyKHz;
                    pDfs->Samplers[i].BusyPulseMode = NewPulseMode;
                    if (BusyCheckDelayMs > delay)
                        BusyCheckDelayMs = delay;   // Min delay to next check
                }
                // Compare new domain target with the previous one - need clock
                // update if they differ significantly
                NeedClockUpdate = NeedClockUpdate ||
                    ((DfsKHz.Domains[i] + pDfs->DfsParameters[i].LowerBandKHz) <= LastKHz.Domains[i]) ||
                    (DfsKHz.Domains[i] >= (LastKHz.Domains[i] + pDfs->DfsParameters[i].UpperBandKHz));
            }
            // Make sure busy hints will be checked in time
            pDfs->SamplingWindow.BusyCheckLastUs = NvRmPrivGetUs();
            pDfs->SamplingWindow.BusyCheckDelayUs = BusyCheckDelayMs * 1000;

            // Low corner report
            if (LowCornerReport)
            {
                NVRM_DFS_PRINTF(("DFS got %s low corner\n",
                                 (LowCornerHit ? "into" : "out of")));
                NvRmPrivUpdateDfsPauseFlag(pDfs->hRm, LowCornerHit);
            }
        }
        else
        {
            // DFS is stopped - thread is signaled by API, always update clock 
            NeedClockUpdate = NV_TRUE;
        }

        // Configure DFS clocks and update current frequencies if necessary
        // (do not touch clocks and voltage if DVS is stopped)
        if (NeedClockUpdate || pDfs->VoltageScaler.UpdateFlag ||
            pDfs->ThermalThrottler.TcorePolicy.UpdateFlag)
        {
            NvRmPrivLockSharedPll();
            if (!pDfs->VoltageScaler.StopFlag)
            {
                // Check temperature and throttle DFS clocks if necessry. Make
                // sure V/F scaling is running while throttling is in progress.
                pDfs->VoltageScaler.UpdateFlag =
                    DttClockUpdate(pDfs, &pDfs->ThermalThrottler, &DfsKHz);
                LastKHz = DfsKHz;
                for (;;)
                {
                    if (DfsClockConfigure(pDfs->hRm, &pDfs->MaxKHz, &DfsKHz))
                        break;
                    DfsKHz = LastKHz;
                }

                NvOsIntrMutexLock(pDfs->hIntrMutex);
                pDfs->CurrentKHz = DfsKHz;
                NvOsIntrMutexUnlock(pDfs->hIntrMutex);
            }
            NvRmPrivUnlockSharedPll();

            // Complete synchronous busy hint processing. 
            if (pDfs->BusySyncState == NvRmDfsBusySyncState_Execute)
            {
                pDfs->BusySyncState = NvRmDfsBusySyncState_Idle;
                NvOsSemaphoreSignal(pDfs->hSyncBusySemaphore);
            }
        }
        DfsProfileSample(pDfs, NvRmDfsProfileId_Control);
    }
    if (PmRequest != NvRmPmRequest_None)
    {
        NVRM_DFS_PRINTF(("PM request: 0x%x\n", PmRequest));
    }
    return PmRequest;
}

static void DfsThreadTerminate(NvRmDfs* pDfs)
{
    /*
     * Request thread abort, signal semaphore to make sure the thread is
     * awaken and wait for its self-termination. Do nothing if invalid DFS
     * structure
     */
    if (pDfs)
    {
        if (pDfs->hSemaphore && pDfs->InitializedThread)
        {
            pDfs->AbortThread = NV_TRUE;
            NvOsSemaphoreSignal(pDfs->hSemaphore);
            for (;;)
            {
                if (!pDfs->AbortThread)
                {
                    break;
                }
                NvOsSleepMS(10);
            }
        }
    }
}

static void
DfsSetAverageUp(
    NvRmDfsClockId ClockId,
    NvRmFreqKHz AverageKHz,
    NvRmDfs* pDfs)
{
    NvRmDfsSampler* pDomainSampler = &pDfs->Samplers[ClockId];

    // Update monitored domain average frequency up
    if ((pDomainSampler->MonitorPresent) &&
        (pDomainSampler->AverageKHz < AverageKHz))
    {
        NvU32 cycles, j;
        NvU64 NewTotalCycles =
            (NvU64)AverageKHz * pDfs->SamplingWindow.SampleWindowMs;
        cycles = (NvU32)(NewTotalCycles >> NVRM_DFS_MAX_SAMPLES_LOG2);
        for (j = 0; j < NV_ARRAY_SIZE(pDomainSampler->Cycles); j++)
        {
            pDomainSampler->Cycles[j] = cycles;
        }
        pDomainSampler->TotalActiveCycles = NewTotalCycles;
        pDomainSampler->AverageKHz = AverageKHz;
    }
}

static void
DfsClockFreqGet(
    NvRmDeviceHandle hRmDevice,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvU32 i;
 
    switch (s_Platform)
    {
        case ExecPlatform_Soc:
            if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
                NvRmPrivAp15DfsClockFreqGet(hRmDevice, pDfsKHz);
            else if (hRmDevice->ChipId.Id == 0x20)
                NvRmPrivAp20DfsClockFreqGet(hRmDevice, pDfsKHz);
            else
                NV_ASSERT(!"Unsupported chip ID");
            break;

        case ExecPlatform_Fpga:
            for (i = 1; i < NvRmDfsClockId_Num; i++)
            {
                // Set fixed FPGA frequency (default: AP15 FPGA)
                if (hRmDevice->ChipId.Id == 0x20)
                    pDfsKHz->Domains[i] = AP20_FPGA_FREQ;
                else
                    pDfsKHz->Domains[i] = AP15_FPGA_FREQ;
            }
            break;

        default:
            NV_ASSERT(!"Not supported execution platform for DFS");
    }
}

static NvBool
DfsClockConfigure(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsFrequencies* pMaxKHz,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvU32 i;

    switch (s_Platform)
    {
        case ExecPlatform_Soc:
            if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
                return NvRmPrivAp15DfsClockConfigure(
                    hRmDevice, pMaxKHz, pDfsKHz);
            else if (hRmDevice->ChipId.Id == 0x20)
                return NvRmPrivAp20DfsClockConfigure(
                    hRmDevice, pMaxKHz, pDfsKHz);
            else
                NV_ASSERT(!"Unsupported chip ID");
            break;

        case ExecPlatform_Fpga:
            for (i = 1; i < NvRmDfsClockId_Num; i++)
            {
                // Set fixed FPGA frequency (default: AP15 FPGA)
                if (hRmDevice->ChipId.Id == 0x20)
                    pDfsKHz->Domains[i] = AP20_FPGA_FREQ;
                else
                    pDfsKHz->Domains[i] = AP15_FPGA_FREQ;
            }
            break;
        default:
            NV_ASSERT(!"Not supported execution platform for DFS");
    }
    return NV_TRUE; // configuration completed
}

static void
DfsClipCpuEmcHighLimits(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz* pCpuHighKHz,
    NvRmFreqKHz* pEmcHighKHz)
{
    if ((hRmDevice->ChipId.Id == 0x15) || (hRmDevice->ChipId.Id == 0x16))
        NvRmPrivAp15ClipCpuEmcHighLimits(hRmDevice, pCpuHighKHz, pEmcHighKHz);
    else if (hRmDevice->ChipId.Id == 0x20)
        NvRmPrivAp20ClipCpuEmcHighLimits(hRmDevice, pCpuHighKHz, pEmcHighKHz);
    else
        NV_ASSERT(!"Unsupported chip ID");
}

/*****************************************************************************/

static void
DttPolicyUpdate(
    NvRmDeviceHandle hRm,
    NvS32 TemperatureC,
    NvRmDtt* pDtt)
{
    if (hRm->ChipId.Id == 0x20)
    {
        NvRmPrivAp20DttPolicyUpdate(hRm, TemperatureC, pDtt);
        NV_ASSERT(pDtt->TcorePolicy.LowLimit !=
                  ODM_TMON_PARAMETER_UNSPECIFIED);
        NV_ASSERT(pDtt->TcorePolicy.HighLimit !=
                  ODM_TMON_PARAMETER_UNSPECIFIED);
    }
    else
    {
        // No thermal policy (= do nothing) for this SoC
        pDtt->TcorePolicy.LowLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
        pDtt->TcorePolicy.HighLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
        pDtt->TcorePolicy.UpdateIntervalUs = NV_WAIT_INFINITE;
        pDtt->TcorePolicy.PolicyRange = 0;
    }
}

static NvBool
DttClockUpdate(
    const NvRmDfs* pDfs,
    NvRmDtt* pDtt,
    NvRmDfsFrequencies* pDfsKHz)
{
    NvS32 TemperatureC;
    NvS32 LowLimit, HighLimit;
    NvU32 OldRange;
    NvRmTzonePolicy Policy;

    // Check if thermal throttling is supported
    if (NVRM_DTT_DISABLED || (!pDtt->hOdmTcore))
        return NV_FALSE; 

    if (pDtt->TcorePolicy.UpdateFlag)
    {
        // Register TMON interrupt, if it is supported by device, and chip
        // policy, but has not been registered yet. Set initial temperature
        // limits according to chip specific policy.
        if (pDtt->UseIntr && !pDtt->hOdmTcoreIntr &&
            NvOdmTmonTemperatureGet(pDtt->hOdmTcore, &TemperatureC))
        {
            DttPolicyUpdate(pDfs->hRm, TemperatureC, pDtt);
            DttRangeReport(TemperatureC, pDtt);
            LowLimit = pDtt->TcorePolicy.LowLimit;
            HighLimit = pDtt->TcorePolicy.HighLimit;

            if ((LowLimit != ODM_TMON_PARAMETER_UNSPECIFIED) &&
                (HighLimit != ODM_TMON_PARAMETER_UNSPECIFIED))
            {
                if(NvOdmTmonParameterConfig(pDtt->hOdmTcore,
                    NvOdmTmonConfigParam_IntrLimitLow, &LowLimit) &&
                   NvOdmTmonParameterConfig(pDtt->hOdmTcore,
                    NvOdmTmonConfigParam_IntrLimitHigh, &HighLimit))
                {
                    pDtt->hOdmTcoreIntr = NvOdmTmonIntrRegister(
                        pDtt->hOdmTcore, DttIntrCallback, (void*)pDfs);
                }
            }
            if (!pDtt->hOdmTcoreIntr)
                pDtt->UseIntr = NV_FALSE; // registration failed - use polling
        }

        // Update temperature monitoring policy
        OldRange = pDtt->TcorePolicy.PolicyRange;
        if (!pDtt->UseIntr &&
            NvOdmTmonTemperatureGet(pDtt->hOdmTcore, &TemperatureC))
        {
            NvOsIntrMutexLock(pDfs->hIntrMutex);
            DttPolicyUpdate(pDfs->hRm, TemperatureC, pDtt);
            Policy = pDtt->TcorePolicy;
        }
        else
        {
            NvOsIntrMutexLock(pDfs->hIntrMutex);
            Policy = pDtt->TcorePolicy;
            TemperatureC = pDtt->CoreTemperatureC;
        }
        if (pDfs->DfsRunState > NvRmDfsRunState_Stopped)
        {
            pDtt->TcorePolicy.UpdateFlag = NV_FALSE;
        }
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);

        // Report range change
        if (!pDtt->UseIntr && (OldRange != pDtt->TcorePolicy.PolicyRange))
        {
            DttRangeReport(TemperatureC, pDtt);
        }
    }
    else
    {
        NvOsIntrMutexLock(pDfs->hIntrMutex);
        Policy = pDtt->TcorePolicy;
        TemperatureC = pDtt->CoreTemperatureC;
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    }

    // Throttle clock frequencies, if necessary
    if (pDfs->hRm->ChipId.Id == 0x20)
        return NvRmPrivAp20DttClockUpdate(
            pDfs->hRm, TemperatureC, &Policy, &pDfs->CurrentKHz, pDfsKHz);
    else
        return NV_FALSE;    // No throttling policy for this chip ID
}

static void DttIntrCallback(void* args)
{
    NvS32 TemperatureC = 0;
    NvS32 LowLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
    NvS32 HighLimit = ODM_TMON_PARAMETER_UNSPECIFIED;
    NvRmDfs* pDfs = (NvRmDfs*)args;
    NvRmDtt* pDtt = &pDfs->ThermalThrottler;

    if (NvOdmTmonTemperatureGet(pDtt->hOdmTcore, &TemperatureC))
    {
        NvOsIntrMutexLock(pDfs->hIntrMutex);
        DttPolicyUpdate(pDfs->hRm, TemperatureC, pDtt);
        LowLimit = pDtt->TcorePolicy.LowLimit;
        HighLimit = pDtt->TcorePolicy.HighLimit;
        pDtt->TcorePolicy.UpdateFlag = NV_TRUE;
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);

        // Clear interrupt condition by setting new limits "around" temperature
        NV_ASSERT(LowLimit != ODM_TMON_PARAMETER_UNSPECIFIED);
        NV_ASSERT(HighLimit != ODM_TMON_PARAMETER_UNSPECIFIED);
        (void)NvOdmTmonParameterConfig(pDtt->hOdmTcore,
            NvOdmTmonConfigParam_IntrLimitLow, &LowLimit);
        (void)NvOdmTmonParameterConfig(pDtt->hOdmTcore,
            NvOdmTmonConfigParam_IntrLimitHigh, &HighLimit);
        DttRangeReport(TemperatureC, pDtt);
    }
}

/*****************************************************************************/
// DFS PRIVATE INTERFACES
/*****************************************************************************/

NvError NvRmPrivDfsInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvError error;
    NvRmDfsFrequencies DfsKHz;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    DfsHintsPrintInit();

    NvOsMemset(pDfs, 0, sizeof(NvRmDfs));
    pDfs->hRm = hRmDeviceHandle;
    s_Platform = NvRmPrivGetExecPlatform(hRmDeviceHandle);
    s_DfsLogOn = NV_FALSE;

    /*
     * Set DFS IRQ invalid to avoid accidental deregeistration of somebody's
     * else IRQ in case of DFS initialization error. Clear DFS clock control
     * execution thread state variables
     */
    pDfs->IrqNumber = NVRM_IRQ_INVALID;
    pDfs->InitializedThread = NV_FALSE;
    pDfs->AbortThread = NV_FALSE;
    pDfs->BusySyncState = NvRmDfsBusySyncState_Idle;
    pDfs->PmRequest = NvRmPmRequest_None;

    // DFS interrupt handler mutex
    error = NvOsIntrMutexCreate(&pDfs->hIntrMutex);
    if (error != NvSuccess)
    {
        goto failed;
    }

    // DFS algorithm parameters and clock limits
    DfsParametersInit(pDfs);

    /*
     * DFS is always disabled in QT and Sim execution environments,
     * when DFS testing is disabled. The initial DFS state for AP15 SoC and
     * FPGA is specified by the respective macros.
     */
    pDfs->DfsRunState = NvRmDfsRunState_Disabled;
    switch (s_Platform)
    {
        case ExecPlatform_Soc:
            if ((pDfs->hRm->ChipId.Id == 0x15) || (pDfs->hRm->ChipId.Id == 0x16))
                pDfs->DfsRunState = NVRM_AP15_SOC_INITIAL_DFS_STATE;
            else if (pDfs->hRm->ChipId.Id == 0x20)
                pDfs->DfsRunState = NVRM_AP20_SOC_INITIAL_DFS_STATE;
            break;
        case ExecPlatform_Fpga:
            pDfs->DfsRunState = NVRM_FPGA_INITIAL_DFS_STATE;
            break;
        default:
            break;
    }
    pDfs->DfsLPxSavedState = pDfs->DfsRunState;
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        // If DFS disabled abort initialization and exit
        return NvSuccess;
    }

    // DFS signaling semaphore
    error = NvOsSemaphoreCreate(&pDfs->hSemaphore, 0);
    if (error != NvSuccess)
    {
        goto failed;
    }
    // Register DFS as power client and obtain client id
    error = NvRmPowerRegister(hRmDeviceHandle, pDfs->hSemaphore, &pDfs->PowerClientId);
    if (error != NvSuccess)
    {
        goto failed;
    }

    // DFS busy hints synchronization objects
    error = NvOsMutexCreate(&pDfs->hSyncBusyMutex);
    if (error != NvSuccess)
    {
        goto failed;
    }
    error = NvOsSemaphoreCreate(&pDfs->hSyncBusySemaphore, 0);
    if (error != NvSuccess)
    {
        goto failed;
    }

    /* 
     * Get DFS modules capbilities, check which activity monitors are
     * supported, and initialize monitor access function pointers. Then
     * initialize DFS samples and H/w monitors
     */
    error = DfsGetModulesCapabilities(pDfs);
    if (error != NvSuccess)
    {
        goto failed;
    }
    DfsClockFreqGet(hRmDeviceHandle, &DfsKHz);
    DfsSamplersInit(&DfsKHz, pDfs);
    error = DfsHwInit(pDfs);
    if (error != NvSuccess)
    {
        goto failed;
    }

    /*
     * Configure System Statistic module interrupt, which will be used to
     * trigger DFS algorithm execution
     */
    {
        pDfs->IrqNumber = NvRmGetIrqForLogicalInterrupt(hRmDeviceHandle, 
                NVRM_MODULE_ID(NvRmModuleID_SysStatMonitor, 0), 
                0);
    }
    if (!pDfs->DfsInterruptHandle)
    {
        NvU32 IrqList = (NvU32)pDfs->IrqNumber;
        NvOsInterruptHandler hDfsIsr = DfsIsr;
        error = NvRmInterruptRegister(hRmDeviceHandle, 1, 
                &IrqList, &hDfsIsr, pDfs, &pDfs->DfsInterruptHandle, NV_TRUE);
        if (error != NvSuccess)
        {
            // Set IRQ invalid to avoid deregistration of other module interrupt
            pDfs->IrqNumber = NVRM_IRQ_INVALID;  
            goto failed;
        }
    }

    /*
     * Provided DFS is initialized in running state, start sampling for the
     * next sampling interval based on current DFS domain frtequencies and
     * enable DFS interrupt
     */
    if (pDfs->DfsRunState > NvRmDfsRunState_Stopped)
    {
        DfsStartMonitors(
            pDfs, &pDfs->CurrentKHz, pDfs->SamplingWindow.NextIntervalMs);
    }
    return NvSuccess;

failed:
    NvRmPrivDfsDeinit(hRmDeviceHandle);
    return error;
}

void NvRmPrivDfsDeinit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT(hRmDeviceHandle);

    // Release all DFS resources
    NvRmInterruptUnregister(hRmDeviceHandle, pDfs->DfsInterruptHandle);
    pDfs->DfsInterruptHandle = NULL;
    DfsThreadTerminate(pDfs);
    DfsHwDeinit(pDfs);
    NvOsSemaphoreDestroy(pDfs->hSyncBusySemaphore);
    NvOsMutexDestroy(pDfs->hSyncBusyMutex);
    NvRmPowerUnRegister(hRmDeviceHandle, pDfs->PowerClientId);
    NvOsSemaphoreDestroy(pDfs->hSemaphore);
    NvOsIntrMutexDestroy(pDfs->hIntrMutex); 
    NvOsMemset(pDfs, 0, sizeof(NvRmDfs));
}

NvRmFreqKHz NvRmPrivDfsGetMaxKHz(NvRmDfsClockId ClockId)
{
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));
    return pDfs->DfsParameters[ClockId].MaxKHz;
}

NvRmFreqKHz NvRmPrivDfsGetMinKHz(NvRmDfsClockId ClockId)
{
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));
    return pDfs->DfsParameters[ClockId].MinKHz;
}

NvRmFreqKHz NvRmPrivDfsGetCurrentKHz(NvRmDfsClockId ClockId)
{
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));
    return pDfs->CurrentKHz.Domains[ClockId];
}

void NvRmPrivDfsSignal(NvRmDfsBusyHintSyncMode Mode)
{
    NvRmDfs* pDfs = &s_Dfs;

    // Just signal clock control thread for asynchronous busy hint or if the
    // thread has not been created (no DFS execution at all)
    if (!((Mode == NvRmDfsBusyHintSyncMode_Sync) && pDfs->InitializedThread))
    {
        NvOsSemaphoreSignal(pDfs->hSemaphore);
        return;
    }

    // Signal clock control thread and wait for clock update before return
    // to caller for synchronous busy hint
    NvOsMutexLock(pDfs->hSyncBusyMutex);

    pDfs->BusySyncState = NvRmDfsBusySyncState_Signal;
    NvOsSemaphoreSignal(pDfs->hSemaphore);

#if !DFS_SYNC_BUSY_TIMEOUT_MS
    NvOsSemaphoreWait(pDfs->hSyncBusySemaphore);
#else
    if(NvError_Timeout == NvOsSemaphoreWaitTimeout(
        pDfs->hSyncBusySemaphore, DFS_SYNC_BUSY_TIMEOUT_MS))
    {
        NvOsDebugPrintf("Syncronous busy hint timeout detected");
        NV_ASSERT(0);
    }
#endif
    NvOsMutexUnlock(pDfs->hSyncBusyMutex);
}

void NvRmPrivDfsResync(void)
{
    NvRmDfsFrequencies DfsKHz;
    NvRmDfs* pDfs = &s_Dfs;

    DfsClockFreqGet(pDfs->hRm, &DfsKHz);

    NvOsIntrMutexLock(pDfs->hIntrMutex);
    pDfs->CurrentKHz = DfsKHz;
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
}

NvRmPmRequest NvRmPrivPmThread(void)
{
    return DfsThread(&s_Dfs);
}

void NvRmPrivStarvationHintPrintf(
    NvU32 ClientId,
    NvU32 ClientTag,
    const NvRmDfsStarvationHint* pMultiHint,
    NvU32 NumHints)
{
#if DFS_HINTS_PRINTF
    {
        NvU32 i;
        char ClientName[sizeof(ClientTag)+ 1];
        ClientTagToString(ClientTag, ClientName); 

        for (i = 0; i < NumHints; i++)
        {
            const NvRmDfsStarvationHint* pHint = &pMultiHint[i];
            NvOsDebugPrintf("%s starvation hint: %s from client %3d (%s)\n",
                            s_DfsDomainNames[pHint->ClockId], 
                            (pHint->Starving ? "TRUE " : "FALSE"),
                            ClientId, ClientName);
        }
    }
#endif
#if DFS_LOGGING_SECONDS
    {
        NvU32 i, SampleIndex;
        NvRmDfs* pDfs = &s_Dfs;

        NvOsIntrMutexLock(pDfs->hIntrMutex);
        if (s_DfsLogOn &&
            ((s_DfsLogStarvationWrIndex  + NumHints) < DFS_LOG_SIZE))
        {
            SampleIndex = s_DfsLogWrIndex;
            for (i = 0; i < NumHints; i++)
            {
                DfsLogStarvationHint* pEntry =
                    &s_DfsLogStarvation[s_DfsLogStarvationWrIndex++];
                pEntry->LogSampleIndex = SampleIndex;
                pEntry->ClientId = ClientId;
                pEntry->ClientTag = ClientTag;
                pEntry->StarvationHint = pMultiHint[i];
            }
        }
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    }
#endif
}

void NvRmPrivBusyHintPrintf(
    NvU32 ClientId,
    NvU32 ClientTag,
    const NvRmDfsBusyHint* pMultiHint,
    NvU32 NumHints)
{
#if DFS_HINTS_PRINTF
    {
        NvU32 i;
        char ClientName[sizeof(ClientTag)+ 1];
        ClientTagToString(ClientTag, ClientName); 

        for (i = 0; i < NumHints; i++)
        {
            const NvRmDfsBusyHint* pHint = &pMultiHint[i];
            NvRmFreqKHz BoostKHz = (pHint->BoostKHz == NvRmFreqMaximum) ?
                NvRmPrivDfsGetMaxKHz(pHint->ClockId) : pHint->BoostKHz;
            NvOsDebugPrintf("%s busy hint: %6dkHz %4dms from client %3d (%s)\n",
                            s_DfsDomainNames[pHint->ClockId], BoostKHz,
                            pHint->BoostDurationMs, ClientId, ClientName);
        }
    }
#endif
#if DFS_LOGGING_SECONDS
    {
        NvU32 i, SampleIndex;
        NvRmDfs* pDfs = &s_Dfs;

        NvOsIntrMutexLock(pDfs->hIntrMutex);
        if (s_DfsLogOn && ((s_DfsLogBusyWrIndex + NumHints) < DFS_LOG_SIZE))
        {
            SampleIndex = s_DfsLogWrIndex;
            for (i = 0; i < NumHints; i++)
            {
                DfsLogBusyHint* pEntry = &s_DfsLogBusy[s_DfsLogBusyWrIndex++];
                pEntry->LogSampleIndex = SampleIndex;
                pEntry->ClientId = ClientId;
                pEntry->ClientTag = ClientTag;
                pEntry->BusyHint = pMultiHint[i];
                if (pEntry->BusyHint.BoostKHz == NvRmFreqMaximum)
                    pEntry->BusyHint.BoostKHz =
                    NvRmPrivDfsGetMaxKHz(pEntry->BusyHint.ClockId);
            }
        }
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    }
#endif
}

/*****************************************************************************/
// DVS PRIVATE INTERFACES
/*****************************************************************************/

static void
DvsChangeCoreVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv)
{
    NvBool WasLow;
    NvRmMilliVolts CurrentMv = pDvs->CurrentCoreMv;

    NV_ASSERT(TargetMv >= pDvs->MinCoreMv);
    NV_ASSERT(TargetMv <= pDvs->NominalCoreMv);

    // Go from current to target voltage in safe steps keeping core and
    // rtc volatges in synch (core voltage above rtc during transition)
    while (CurrentMv != TargetMv)
    {
        WasLow = (CurrentMv < pDvs->LowSvopThresholdMv);

        if (CurrentMv < TargetMv)
        {
            CurrentMv += NVRM_SAFE_VOLTAGE_STEP_MV;
            if (CurrentMv > TargetMv)
                CurrentMv = TargetMv;
            NvRmPmuSetVoltage(hRm, pDvs->CoreRailAddress, CurrentMv, NULL);
            if (pDvs->CoreRailAddress != pDvs->RtcRailAddress)
                NvRmPmuSetVoltage(hRm, pDvs->RtcRailAddress, CurrentMv, NULL);
            if (WasLow && (CurrentMv >= pDvs->LowSvopThresholdMv))
            {
                // Clear SVOP bits after crossing SVOP threshold up
                NvRmPrivAp15SetSvopControls(hRm, pDvs->HighSvopSettings);
            }
        }
        else
        {
            CurrentMv -= NVRM_SAFE_VOLTAGE_STEP_MV;
            if (CurrentMv < TargetMv)
                CurrentMv = TargetMv;
            if (!WasLow && (CurrentMv < pDvs->LowSvopThresholdMv))
            {   // Set SVOP bits before crossing SVOP threshold down
                NvRmPrivAp15SetSvopControls(hRm, pDvs->LowSvopSettings);
            }
            NvRmPmuSetVoltage(hRm, pDvs->RtcRailAddress, CurrentMv, NULL);
            if (pDvs->CoreRailAddress != pDvs->RtcRailAddress)
                NvRmPmuSetVoltage(hRm, pDvs->CoreRailAddress, CurrentMv, NULL);
        }
    }
    pDvs->CurrentCoreMv = TargetMv;
}

static void
DvsChangeCpuVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv)
{
    NV_ASSERT(TargetMv >= pDvs->MinCpuMv);
    NV_ASSERT(TargetMv <= pDvs->NominalCpuMv);

    if (pDvs->CurrentCpuMv != TargetMv)
    {
        NvRmPmuSetVoltage(hRm, pDvs->CpuRailAddress, TargetMv, NULL);
        pDvs->CurrentCpuMv = TargetMv;
    }
}

void NvRmPrivDvsInit(void)
{
    NvRmPmuVddRailCapabilities cap;
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;
    NvOdmPmuProperty PmuProperty = {0};

    const NvOdmPeripheralConnectivity* pRtcRail =
        NvOdmPeripheralGetGuid(NV_VDD_RTC_ODM_ID);
    const NvOdmPeripheralConnectivity* pCoreRail =
        NvOdmPeripheralGetGuid(NV_VDD_CORE_ODM_ID);

    /* Some systems(ex. FPGA) does have power rail control. */
    if (!pRtcRail || !pCoreRail)
        return;

    pDvs->NominalCoreMv = NvRmPrivGetNominalMV(pDfs->hRm);
    pDvs->MinCoreMv = NvRmPrivSourceVscaleGetMV(pDfs->hRm, 0);
    pDvs->LowCornerCoreMv = pDvs->MinCoreMv;
    NvRmPrivGetSvopParameters(pDfs->hRm, &pDvs->LowSvopThresholdMv,
                              &pDvs->LowSvopSettings, &pDvs->HighSvopSettings);
    pDvs->UpdateFlag = NV_FALSE;
    pDvs->StopFlag = NV_FALSE;
    pDvs->Lp2SyncOTPFlag = NV_FALSE;

    // Get RTC rail address, check range and resolution
    NV_ASSERT(pRtcRail && pRtcRail->NumAddress);
    pDvs->RtcRailAddress = pRtcRail->AddressList[0].Address;
    NvRmPmuGetCapabilities(pDfs->hRm, pDvs->RtcRailAddress, &cap);
    NV_ASSERT((cap.StepMilliVolts) &&
              (cap.StepMilliVolts <= NVRM_SAFE_VOLTAGE_STEP_MV));
    NV_ASSERT(cap.MinMilliVolts <= pDvs->MinCoreMv);
    NV_ASSERT(cap.MaxMilliVolts >= pDvs->NominalCoreMv);

    // Get Core rail address, check range and resolution
    NV_ASSERT(pCoreRail && pCoreRail->NumAddress);
    pDvs->CoreRailAddress = pCoreRail->AddressList[0].Address;
    NvRmPmuGetCapabilities(pDfs->hRm, pDvs->CoreRailAddress, &cap);
    NV_ASSERT((cap.StepMilliVolts) &&
              (cap.StepMilliVolts <= NVRM_SAFE_VOLTAGE_STEP_MV));
    NV_ASSERT((cap.StepMilliVolts) &&
              (cap.StepMilliVolts <= NVRM_CORE_RESOLUTION_MV));
    NV_ASSERT(cap.MinMilliVolts <= pDvs->MinCoreMv);
    NV_ASSERT(cap.MaxMilliVolts >= pDvs->NominalCoreMv);

    if (NvRmPrivIsCpuRailDedicated(pDfs->hRm))
    {
        // Get dedicated CPU rail address, check range and resolution
        const NvOdmPeripheralConnectivity* pCpuRail =
            NvOdmPeripheralGetGuid(NV_VDD_CPU_ODM_ID);

        pDvs->NominalCpuMv = NvRmPrivModuleVscaleGetMV(
            pDfs->hRm, NvRmModuleID_Cpu,
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MaxKHz);
        pDvs->MinCpuMv = NvRmPrivModuleVscaleGetMV(
            pDfs->hRm, NvRmModuleID_Cpu,
            NvRmPrivGetSocClockLimits(NvRmModuleID_Cpu)->MinKHz);

        NV_ASSERT(pCpuRail && pCpuRail->NumAddress);
        pDvs->CpuRailAddress = pCpuRail->AddressList[0].Address;
        NvRmPmuGetCapabilities(pDfs->hRm, pDvs->CpuRailAddress, &cap);
        NV_ASSERT((cap.StepMilliVolts) &&
                  (cap.StepMilliVolts <= NVRM_SAFE_VOLTAGE_STEP_MV));
        NV_ASSERT((cap.StepMilliVolts) &&
                  (cap.StepMilliVolts <= NVRM_CORE_RESOLUTION_MV));
#if NVRM_DVS_ACCEPT_PMU_HIGH_CPU_MIN
        pDvs->MinCpuMv = NV_MAX(pDvs->MinCpuMv, cap.MinMilliVolts);
        pDvs->NominalCpuMv = NV_MAX(pDvs->NominalCpuMv, pDvs->MinCpuMv);
#else
        NV_ASSERT(pDvs->MinCpuMv <= pDvs->NominalCpuMv);
        NV_ASSERT(cap.MinMilliVolts <= pDvs->MinCpuMv);
#endif
        NV_ASSERT(cap.MaxMilliVolts >= pDvs->NominalCpuMv);
        pDvs->CpuOTPMv = cap.requestMilliVolts;
        pDvs->LowCornerCpuMv = pDvs->MinCpuMv;

        // CPU rail behaviour after CPU request signal On-Off-On transition
        if (NvOdmQueryGetPmuProperty(&PmuProperty))
            pDvs->VCpuOTPOnWakeup = PmuProperty.VCpuOTPOnWakeup;

        // Get dedicated CPU rail boot voltage
        NvRmPmuGetVoltage(pDfs->hRm, pDvs->CpuRailAddress, &pDvs->CurrentCpuMv);
    }

    // Get boot core voltage.  Check if DFS is disabled - no voltage scaling
    // in this case. Otherwise, set nominal core and dedicated cpu voltages.
    // Initialize DVS corner variables.
    NvRmPmuGetVoltage(pDfs->hRm, pDvs->CoreRailAddress, &pDvs->CurrentCoreMv);
    if ((pDfs->DfsRunState <= NvRmDfsRunState_Disabled))
    {
        pDvs->RtcRailAddress = pDvs->CoreRailAddress = 0;
        return;
    }

    if (NvRmPrivIsCpuRailDedicated(pDfs->hRm))
    {
        // If core voltage is going up, update it before CPU and vice versa
        if (pDvs->CurrentCoreMv <= pDvs->NominalCoreMv)
        {
            DvsChangeCoreVoltage(pDfs->hRm, pDvs, pDvs->NominalCoreMv);
        }
        DvsChangeCpuVoltage(pDfs->hRm, pDvs, pDvs->NominalCpuMv);
        pDvs->DvsCorner.CpuMv = pDvs->NominalCpuMv;

        if (pDvs->CurrentCoreMv > pDvs->NominalCoreMv)
        {
            NvOsWaitUS(NVRM_CPU_TO_CORE_DOWN_US); // delay if core to go down
            DvsChangeCoreVoltage(pDfs->hRm, pDvs, pDvs->NominalCoreMv);
        }
        // No core scaling if CPU voltage is not preserved across LPx
        if (pDvs->VCpuOTPOnWakeup)
            pDvs->MinCoreMv = pDvs->NominalCoreMv;
    }
    else
    {
        DvsChangeCoreVoltage(pDfs->hRm, pDvs, pDvs->NominalCoreMv);
        pDvs->DvsCorner.CpuMv = pDvs->NominalCoreMv;
    }
    pDvs->DvsCorner.SystemMv = pDvs->NominalCoreMv;
    pDvs->DvsCorner.EmcMv = pDvs->NominalCoreMv;
    pDvs->DvsCorner.ModulesMv = pDvs->NominalCoreMv;

    if ((pDfs->hRm->ChipId.Id == 0x15) || (pDfs->hRm->ChipId.Id == 0x16))
    {
        pDvs->LowCornerCoreMv = NV_MAX(NVRM_AP15_LOW_CORE_MV, pDvs->MinCoreMv);
        pDvs->LowCornerCoreMv =
            NV_MIN(pDvs->LowCornerCoreMv, pDvs->NominalCoreMv);
    }
    else if (pDfs->hRm->ChipId.Id == 0x20)
    {
        pDvs->LowCornerCoreMv = NV_MAX(NVRM_AP20_LOW_CORE_MV, pDvs->MinCoreMv);
        pDvs->LowCornerCoreMv =
            NV_MIN(pDvs->LowCornerCoreMv, pDvs->NominalCoreMv);

        pDvs->LowCornerCpuMv = NV_MAX(NVRM_AP20_LOW_CPU_MV, pDvs->MinCpuMv);
        pDvs->LowCornerCpuMv =
            NV_MIN(pDvs->LowCornerCpuMv, pDvs->NominalCpuMv);
    }
}

void NvRmPrivVoltageScale(
    NvBool BeforeFreqChange,
    NvRmMilliVolts CpuMv,
    NvRmMilliVolts SystemMv,
    NvRmMilliVolts EmcMv)
{
    NvRmMilliVolts TargetMv;
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;
    NvBool DedicatedCpuRail = NvRmPrivIsCpuRailDedicated(pDfs->hRm);
    
    /* Some systems(ex. FPGA) does have power rail control. */
    if (!pDvs->RtcRailAddress || !pDvs->CoreRailAddress)
        return;

    // Record new DVS threshold and determine new target voltage as maximunm of
    // all thresholds
    pDvs->DvsCorner.CpuMv = CpuMv;
    pDvs->DvsCorner.SystemMv = SystemMv;
    pDvs->DvsCorner.EmcMv = EmcMv;

    NvRmPrivLockModuleClockState();
    TargetMv = NvRmPrivModulesGetOperationalMV(pDfs->hRm);
    NvRmPrivUnlockModuleClockState();
    pDvs->DvsCorner.ModulesMv = TargetMv;

    if (!DedicatedCpuRail && (TargetMv < CpuMv))
        TargetMv = CpuMv;
    if (TargetMv < SystemMv)
        TargetMv = SystemMv;
    if (TargetMv < EmcMv)
        TargetMv = EmcMv;

    // Clip new target voltage to core voltage limits
    if (TargetMv > pDvs->NominalCoreMv)
        TargetMv = pDvs->NominalCoreMv;
    else if (TargetMv < pDvs->LowCornerCoreMv)
        TargetMv = pDvs->LowCornerCoreMv;

    if (DedicatedCpuRail)
    {
        // Clip new CPU voltage to CPU voltage limits
        if (CpuMv > pDvs->NominalCpuMv)
            CpuMv = pDvs->NominalCpuMv;
        else if (CpuMv < pDvs->LowCornerCpuMv)
            CpuMv = pDvs->LowCornerCpuMv;

        // Increase voltage before changing frequency, and vice versa;
        // Change core 1st before changing frequency, and vice versa
        // (to guarantee required margin of core voltage over CPU voltage) 
        if (BeforeFreqChange)
        {
            if (pDvs->Lp2SyncOTPFlag)
            {
                // If required, synchronize DVFS state with CPU rail default
                // level after LP2 exit
                pDvs->Lp2SyncOTPFlag = NV_FALSE;
                pDvs->CurrentCpuMv = pDvs->CpuOTPMv;
            }
            if (pDvs->CurrentCoreMv < TargetMv)
                DvsChangeCoreVoltage(pDfs->hRm, pDvs, TargetMv);
            if (pDvs->CurrentCpuMv < CpuMv)
                DvsChangeCpuVoltage(pDfs->hRm, pDvs, CpuMv);
        }
        else
        {
            if (pDvs->CurrentCpuMv > CpuMv)
            {
                DvsChangeCpuVoltage(pDfs->hRm, pDvs, CpuMv);
                // Defer core voltage change to the next DVFS tick to account
                // for CPU capacitors discharge
                if (pDvs->CurrentCoreMv > TargetMv)
                    pDvs->UpdateFlag = NV_TRUE;
            }
            else if (pDvs->CurrentCoreMv > TargetMv)
                DvsChangeCoreVoltage(pDfs->hRm, pDvs, TargetMv);
        }
    }
    else
    {
        //  Increase voltage before changing frequency, and vice versa
        if ((BeforeFreqChange && (pDvs->CurrentCoreMv < TargetMv)) ||
            (!BeforeFreqChange && (pDvs->CurrentCoreMv > TargetMv)))
        {
            DvsChangeCoreVoltage(pDfs->hRm, pDvs, TargetMv);
        }
    }
}

void NvRmPrivDvsRequest(NvRmMilliVolts TargetMv)
{
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;

    // Do nothing for unspecified target.
    if (TargetMv == NvRmVoltsUnspecified)
        return;

    /* Some systems(ex. FPGA) does have power rail control. */
    if (!pDvs->RtcRailAddress || !pDvs->CoreRailAddress)
        return;

    // Clip new target voltage to core voltage limits
    if (TargetMv > pDvs->NominalCoreMv)
        TargetMv = pDvs->NominalCoreMv;
    else if (TargetMv < pDvs->LowCornerCoreMv)
        TargetMv = pDvs->LowCornerCoreMv;

    // If new target voltage is above current - update immediately. If target
    // is below current voltage - just set update flag, so that next DFS ISR
    // signals DFS thread, which checks operational voltage for all modules.
    if (TargetMv > pDvs->CurrentCoreMv)
    {
        DvsChangeCoreVoltage(pDfs->hRm, pDvs, TargetMv);
    }
    else if (TargetMv < pDvs->CurrentCoreMv)
    {
        pDvs->UpdateFlag = NV_TRUE;
    }
}

void
NvRmPrivGetLowVoltageThreshold(
    NvRmDfsVoltageRailId RailId,
    NvRmMilliVolts* pLowMv,
    NvRmMilliVolts* pPresentMv)
{
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;
    NV_ASSERT(pLowMv);

    switch (RailId)
    {
        case NvRmDfsVoltageRailId_Core:
            *pLowMv = pDvs->LowCornerCoreMv;
            if(pPresentMv)
                *pPresentMv = pDvs->CurrentCoreMv;
            break;

        case NvRmDfsVoltageRailId_Cpu:
            if (NvRmPrivIsCpuRailDedicated(pDfs->hRm))
            {
                *pLowMv = pDvs->LowCornerCpuMv;
                if(pPresentMv)
                    *pPresentMv = pDvs->CurrentCpuMv;
                break;
            }
            // fall through

        default:
            *pLowMv = NvRmVoltsUnspecified;
            if(pPresentMv)
                *pPresentMv = NvRmVoltsUnspecified;
            break;
    }
}

static void NvRmPrivDvsStopAtNominal(void)
{
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;

    /* Some systems(ex. FPGA) does have power rail control. */
    if (!pDvs->RtcRailAddress || !pDvs->CoreRailAddress)
        return;

    // Set nominal voltage
    DvsChangeCoreVoltage(pDfs->hRm, pDvs, pDvs->NominalCoreMv);
    if(NvRmPrivIsCpuRailDedicated(pDfs->hRm))
        DvsChangeCpuVoltage(pDfs->hRm, pDvs, pDvs->NominalCpuMv);
}

static void NvRmPrivDvsRun(void)
{
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;
    pDvs->UpdateFlag = NV_TRUE;
    pDvs->StopFlag = NV_FALSE;
}

void NvRmPrivDfsSuspend(NvOdmSocPowerState state)
{
    NvRmDfs* pDfs = &s_Dfs;
    NvBool UpdateClocks = NV_FALSE;
    NvRmDfsFrequencies DfsKHz;

    // Fill in target frequencies for suspend state on the 1st entry
    // (use invalid domain frequency as 1st flag)
    if (pDfs->SuspendKHz.Domains[0] == 0)
    {
        if ((pDfs->hRm->ChipId.Id == 0x15) || (pDfs->hRm->ChipId.Id == 0x16))
            NvRmPrivAp15DfsVscaleFreqGet(
                pDfs->hRm, NVRM_AP15_SUSPEND_CORE_MV, &pDfs->SuspendKHz);
        else if (pDfs->hRm->ChipId.Id == 0x20)
            NvRmPrivAp20DfsSuspendFreqGet(
                pDfs->hRm, NVRM_AP20_SUSPEND_CORE_MV, &pDfs->SuspendKHz);
        else
            pDfs->SuspendKHz = pDfs->LowCornerKHz; // Low corner by default
        pDfs->SuspendKHz.Domains[0] = NvRmFreqMaximum;   
    }
    
    NvRmPrivLockSharedPll();
    if (state == NvOdmSocPowerState_DeepSleep)
    {
        // On entry to deep sleeep (LP0): set nominal voltage level and
        // stop DVFS at nominal voltage until resume.
        NvOsIntrMutexLock(pDfs->hIntrMutex);
        pDfs->DfsLPxSavedState = pDfs->DfsRunState;
        if (pDfs->DfsLPxSavedState > NvRmDfsRunState_Stopped)
            pDfs->DfsRunState = NvRmDfsRunState_Stopped;
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);

        NvRmPrivDvsStopAtNominal();
        pDfs->VoltageScaler.StopFlag = NV_TRUE;
    }
    else if (state == NvOdmSocPowerState_Suspend)
    {
        // On entry to suspend (LP1): set target frequencies for all DFS
        // clock domains, stop DFS monitors, and then configure clocks and
        // core voltage. Stop DVFS in suspend corner until resume.
        DfsKHz = pDfs->SuspendKHz;

        NvOsIntrMutexLock(pDfs->hIntrMutex);
        pDfs->DfsLPxSavedState = pDfs->DfsRunState;
        if (pDfs->DfsLPxSavedState > NvRmDfsRunState_Stopped)
        {
            pDfs->DfsRunState = NvRmDfsRunState_Stopped;
            pDfs->TargetKHz = DfsKHz;
            UpdateClocks = NV_TRUE;
        }
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);

        for (; UpdateClocks;)
        {
            if (DfsClockConfigure(pDfs->hRm, &pDfs->MaxKHz, &DfsKHz))
            {
                pDfs->CurrentKHz = DfsKHz;  // DFS is already stopped - no mutex
                break;
            }
            DfsKHz = pDfs->SuspendKHz;
        }

        if (NvRmPrivIsCpuRailDedicated(pDfs->hRm))
        {
            NvRmDvs* pDvs = &s_Dfs.VoltageScaler;
            NvRmMilliVolts v = NV_MAX(pDvs->DvsCorner.SystemMv,
                                      NV_MAX(pDvs->DvsCorner.EmcMv,
                                             pDvs->DvsCorner.ModulesMv));

            // If CPU rail returns to default level by PMU underneath DVFS
            // need to synchronize voltage after LP1 same way as after LP2
            if (pDvs->VCpuOTPOnWakeup)
                pDfs->VoltageScaler.Lp2SyncOTPFlag = NV_TRUE;

            // If core voltage change was deferred until CPU voltage is
            // settled - do it now
            if (v < pDvs->CurrentCoreMv)
            {
                NvOsWaitUS(NVRM_CPU_TO_CORE_DOWN_US);
                DvsChangeCoreVoltage(pDfs->hRm, pDvs, v);
            }
            NvOsDebugPrintf("DVFS set core at %dmV\n", pDvs->CurrentCoreMv);
        }

        pDfs->VoltageScaler.StopFlag = NV_TRUE;
    }
    NvRmPrivUnlockSharedPll();
}

/*****************************************************************************/
// DTT PRIVATE INTERFACES
/*****************************************************************************/

void NvRmPrivDttInit(NvRmDeviceHandle hRmDeviceHandle)
{
    NvRmDfs* pDfs = &s_Dfs;
    NvRmDtt* pDtt = &pDfs->ThermalThrottler;

    // Make sure TMON h/w is initialized
    pDtt->hOdmTcore = NvOdmTmonDeviceOpen(NvOdmTmonZoneID_Core);

    // No thermal throttling if DFS is disabled, otherwise start DTTS
    if (pDfs->DfsRunState < NvRmDfsRunState_Stopped)
    {
        NvOdmTmonDeviceClose(pDtt->hOdmTcore);
        pDtt->hOdmTcore = NULL;
        return;
    }

    if (!pDtt->hOdmTcore)
    {
        if (pDfs->hRm->ChipId.Id == 0x20)
        {
            // TODO: assert?
            NvOsDebugPrintf("DTT: TMON initialization failed\n");
        }
        return;
    }
    NvOdmTmonCapabilitiesGet(pDtt->hOdmTcore, &pDtt->TcoreCaps);
    NvOdmTmonParameterCapsGet(pDtt->hOdmTcore,
        NvOdmTmonConfigParam_IntrLimitLow, &pDtt->TcoreLowLimitCaps);
    NvOdmTmonParameterCapsGet(pDtt->hOdmTcore,
        NvOdmTmonConfigParam_IntrLimitHigh, &pDtt->TcoreHighLimitCaps);

#if !NVRM_DTT_DISABLED
    // Default policy for room temperature
    DttPolicyUpdate(hRmDeviceHandle, 25, pDtt);
    pDtt->TcorePolicy.TimeUs = NvRmPrivGetUs();
#endif

    if (pDtt->TcoreCaps.IntrSupported &&
        !pDtt->TcoreLowLimitCaps.OdmProtected &&
        !pDtt->TcoreHighLimitCaps.OdmProtected)
    {
        // Sanity checks to make sure out-of-limit interrupt is available in
        // the entire temperature range 
        NV_ASSERT(pDtt->TcoreLowLimitCaps.MinValue <= pDtt->TcoreCaps.Tmin);
        NV_ASSERT(pDtt->TcoreHighLimitCaps.MinValue <= pDtt->TcoreCaps.Tmin);
        NV_ASSERT(pDtt->TcoreLowLimitCaps.MaxValue >= pDtt->TcoreCaps.Tmax);
        NV_ASSERT(pDtt->TcoreHighLimitCaps.MaxValue >= pDtt->TcoreCaps.Tmax);
#if NVRM_DTT_USE_INTERRUPT
        pDtt->UseIntr = NV_TRUE;
#endif
    }
}

void NvRmPrivDttDeinit()
{
    NvRmDfs* pDfs = &s_Dfs;
    NvOdmTmonDeviceHandle hOdmTcore = pDfs->ThermalThrottler.hOdmTcore;
    NvOdmTmonIntrHandle hOdmTcoreIntr = pDfs->ThermalThrottler.hOdmTcoreIntr;

    NvOdmTmonIntrUnregister(hOdmTcore, hOdmTcoreIntr);
    pDfs->ThermalThrottler.hOdmTcoreIntr = NULL;

    NvOdmTmonDeviceClose(hOdmTcore);
    pDfs->ThermalThrottler.hOdmTcore = NULL;
}

/*****************************************************************************/
// DFS PUBLIC INTERFACES
/*****************************************************************************/

NvRmDfsRunState
NvRmDfsGetState(
    NvRmDeviceHandle hRmDeviceHandle)
{
    NvRmDfsRunState state;
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT(hRmDeviceHandle);

    if(!pDfs->hIntrMutex)
        return NvRmDfsRunState_Invalid;

    NvOsIntrMutexLock(pDfs->hIntrMutex);
    state = pDfs->DfsRunState;
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return state;
}

NvError
NvRmDfsSetState(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsRunState NewDfsRunState)
{
    NvRmDfsRunState OldDfsRunState;
    NvRmDfsFrequencies DfsKHz;
    NvError error = NvSuccess;
    NvRmDfs* pDfs = &s_Dfs;
    
    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT((0 < NewDfsRunState) && (NewDfsRunState < NvRmDfsRunState_Num)); 

    NvRmPrivLockSharedPll();
    DfsClockFreqGet(hRmDeviceHandle, &DfsKHz);

    NvOsIntrMutexLock(pDfs->hIntrMutex);
    OldDfsRunState = pDfs->DfsRunState;

    // No transition from disabled state is supported
    if (OldDfsRunState == NvRmDfsRunState_Disabled)
        NewDfsRunState = NvRmDfsRunState_Invalid;

    /*
     * State transition procedures
     */ 
    switch (NewDfsRunState)
    {
        // On transition to running states from stopped state samplers are
        // initialized and restarted; if profiled loop is supported and it
        // is specified as a new state, profile is initialized as well
#if DFS_PROFILING
        case NvRmDfsRunState_ProfiledLoop:
            DfsProfileInit(pDfs);
            // fall through
#endif
        case NvRmDfsRunState_ClosedLoop:
            pDfs->DfsRunState = NewDfsRunState;
            if (OldDfsRunState == NvRmDfsRunState_Stopped)
            {
                DfsSamplersInit(&DfsKHz, pDfs);
                DfsStartMonitors(
                    pDfs, &pDfs->CurrentKHz, pDfs->SamplingWindow.NextIntervalMs);
            }
            break;

        // On transition to stopped state just stop DFS targets at whatever
        // frequency they are now
        case NvRmDfsRunState_Stopped:
            pDfs->DfsRunState = NewDfsRunState;
            break;

        // Not supported transition
        default:
            error = NvError_NotSupported;
            break;
    }
    pDfs->DfsLPxSavedState = pDfs->DfsRunState;
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    NvRmPrivUnlockSharedPll();
    return error;
}

NvError
NvRmDfsSetLowCorner(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 DfsFreqListCount,
    const NvRmFreqKHz* pDfsLowFreqList)
{
    NvU32 i;
    NvRmDfs* pDfs = &s_Dfs;
    
    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(DfsFreqListCount == NvRmDfsClockId_Num);
    NV_ASSERT(pDfsLowFreqList);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to set if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Clip requested low corner frequencies to domain limits and update
    // DFS low corner (keep corner unchanged if new value is "unspecified")
    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        NvRmFreqKHz DomainKHz = pDfsLowFreqList[i];
        // Preserve CPU or EMC low boundary when the respective envelope is set
        if ((pDfs->CpuEnvelopeSet && (i == NvRmDfsClockId_Cpu)) ||
            (pDfs->EmcEnvelopeSet && (i == NvRmDfsClockId_Emc)))
        {
            continue;
        }
        if (DomainKHz != NvRmFreqUnspecified)
        {
            if (DomainKHz < pDfs->DfsParameters[i].MinKHz)
            {
                DomainKHz = pDfs->DfsParameters[i].MinKHz;
            }
            else if (DomainKHz > pDfs->HighCornerKHz.Domains[i])
            {
                DomainKHz = pDfs->HighCornerKHz.Domains[i];
            }
            pDfs->LowCornerKHz.Domains[i] = DomainKHz;
            if (i == NvRmDfsClockId_Cpu)
                pDfs->CpuCornersShadow.MinKHz = DomainKHz;
        }
    }
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsSetAvHighCorner(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmFreqKHz DfsAvSystemHighKHz,
    NvRmFreqKHz DfsAvpHighKHz,
    NvRmFreqKHz DfsVpipeHighKHz)
{
    NvU32 i;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to set if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Clip requested VDE high corner frequency to domain limits
    // (keep corner unchanged if new value is "unspecified")
    if (DfsVpipeHighKHz == NvRmFreqUnspecified)
        DfsVpipeHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Vpipe];
    else if (DfsVpipeHighKHz > pDfs->DfsParameters[NvRmDfsClockId_Vpipe].MaxKHz)
        DfsVpipeHighKHz = pDfs->DfsParameters[NvRmDfsClockId_Vpipe].MaxKHz;
    else if (DfsVpipeHighKHz < pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Vpipe])
        DfsVpipeHighKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Vpipe];

    // Clip requested AVP high corner frequency to domain limits
    // (keep corner unchanged if new value is "unspecified")
    if (DfsAvpHighKHz == NvRmFreqUnspecified)
        DfsAvpHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Avp];
    else if (DfsAvpHighKHz > pDfs->DfsParameters[NvRmDfsClockId_Avp].MaxKHz)
        DfsAvpHighKHz = pDfs->DfsParameters[NvRmDfsClockId_Avp].MaxKHz;
    else if (DfsAvpHighKHz < pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Avp])
        DfsAvpHighKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Avp];


    // Clip requested AVP/System high corner frequency to domain limits
    // (keep corner unchanged if new value is "unspecified")
    if (DfsAvSystemHighKHz == NvRmFreqUnspecified)
        DfsAvSystemHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_System];
    else if (DfsAvSystemHighKHz > pDfs->DfsParameters[NvRmDfsClockId_System].MaxKHz)
        DfsAvSystemHighKHz = pDfs->DfsParameters[NvRmDfsClockId_System].MaxKHz;
    else    
    {   // System high boundary must be above all AV low boundaries
        for (i = 1; i < NvRmDfsClockId_Num; i++)
        {
            if ((i != NvRmDfsClockId_Cpu) &&
                (i != NvRmDfsClockId_Emc))
            {
                if ((i == NvRmDfsClockId_Vpipe) &&
                    (!NvRmPrivGetClockSourceHandle(NvRmClockSource_Vbus)))
                    continue; // Skip v-pipe if VDE clock is decoupled from AV

                if (DfsAvSystemHighKHz < pDfs->LowCornerKHz.Domains[i])
                    DfsAvSystemHighKHz = pDfs->LowCornerKHz.Domains[i];
            }
        }
    }

    // Make sure new System and AVP, VDE high boundaries are consistent
    if ((DfsAvSystemHighKHz < DfsVpipeHighKHz) &&
        NvRmPrivGetClockSourceHandle(NvRmClockSource_Vbus))
    {
        DfsAvSystemHighKHz = DfsVpipeHighKHz;
    }
    if (DfsAvSystemHighKHz < DfsAvpHighKHz)
    {
        DfsAvSystemHighKHz = DfsAvpHighKHz;
    }

    // Finally update high corner
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_System] = DfsAvSystemHighKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Ahb] = NV_MIN(
        DfsAvSystemHighKHz, pDfs->DfsParameters[NvRmDfsClockId_Ahb].MaxKHz);
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Apb] = NV_MIN(
        DfsAvSystemHighKHz, pDfs->DfsParameters[NvRmDfsClockId_Apb].MaxKHz);

    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Avp] = DfsAvpHighKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Vpipe] = DfsVpipeHighKHz;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsSetCpuEmcHighCorner(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmFreqKHz DfsCpuHighKHz,
    NvRmFreqKHz DfsEmcHighKHz)
{
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to set if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Preserve CPU and EMC high corners if either CPU or EMC envelope is set
    if (pDfs->CpuEnvelopeSet || pDfs->EmcEnvelopeSet)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvSuccess;
    }

    // Keep corner unchanged if new requested value is "unspecified"
    if (DfsCpuHighKHz == NvRmFreqUnspecified)
        DfsCpuHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu];
    if (DfsEmcHighKHz == NvRmFreqUnspecified)
        DfsEmcHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc];

    // Clip requested CPU and EMC high corner frequencies to domain maximum
    if (DfsCpuHighKHz > pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz)
        DfsCpuHighKHz = pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz;
    if (DfsEmcHighKHz > pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz)
        DfsEmcHighKHz = pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz;

    // Clip requested CPU and EMC high corner to supported EMC configuration
    DfsClipCpuEmcHighLimits(
        hRmDeviceHandle, &DfsCpuHighKHz, &DfsEmcHighKHz);

    // Clip requested CPU and EMC frequencies to domain low limits
    if (DfsCpuHighKHz < pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu])
        DfsCpuHighKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu];
    if (DfsEmcHighKHz < pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Emc])
        DfsEmcHighKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Emc];

    // Finally update high corner
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu] = DfsCpuHighKHz;
    pDfs->CpuCornersShadow.MaxKHz = DfsCpuHighKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc] = DfsEmcHighKHz;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsSetCpuEnvelope(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmFreqKHz DfsCpuLowCornerKHz,
    NvRmFreqKHz DfsCpuHighCornerKHz)
{
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to set if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Preserve unspecified boundary, unless it violates new setting for
    // the other one; set both boundaries equal in the latter case
    if (DfsCpuLowCornerKHz == NvRmFreqUnspecified)
    {
        DfsCpuLowCornerKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu];
        if (DfsCpuLowCornerKHz > DfsCpuHighCornerKHz)
            DfsCpuLowCornerKHz = DfsCpuHighCornerKHz;
    }
    if (DfsCpuHighCornerKHz == NvRmFreqUnspecified)
    {
        DfsCpuHighCornerKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu];
        if (DfsCpuLowCornerKHz > DfsCpuHighCornerKHz)
            DfsCpuHighCornerKHz = DfsCpuLowCornerKHz;
    }

    // Can not set envelope with reversed boundaries
    if (DfsCpuLowCornerKHz > DfsCpuHighCornerKHz)
    {
        NV_ASSERT(!"CPU envelope boundaries are reversed");
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_BadValue;
    }

    // Clip requested boundaries to CPU domain limits; mark envelope "set" if
    // any requested boundary is inside the limits
    pDfs->CpuEnvelopeSet = NV_FALSE;    // assume envelope is open

    if (DfsCpuLowCornerKHz <= pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz)
        DfsCpuLowCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz;
    else
    {
        pDfs->CpuEnvelopeSet = NV_TRUE; // envelope sealed
        if (DfsCpuLowCornerKHz >= pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz)
            DfsCpuLowCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz;
    }

    if (DfsCpuHighCornerKHz >= pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz)
        DfsCpuHighCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Cpu].MaxKHz;
    else
    {
        pDfs->CpuEnvelopeSet = NV_TRUE; // envelope sealed
        if (DfsCpuHighCornerKHz <= pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz)
            DfsCpuHighCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Cpu].MinKHz;
    }
    // Shadow new limits before they may be throttled by EMC
    pDfs->CpuCornersShadow.MinKHz = DfsCpuLowCornerKHz;
    pDfs->CpuCornersShadow.MaxKHz = DfsCpuHighCornerKHz;

    // If EMC envelope is set, move (throttle) CPU envelope as necessary
    if (pDfs->EmcEnvelopeSet)
    {
        NvRmFreqKHz EmcHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc];
        DfsClipCpuEmcHighLimits(
            hRmDeviceHandle, &DfsCpuHighCornerKHz, &EmcHighKHz);
        NV_ASSERT(EmcHighKHz == pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc]);
        if (DfsCpuLowCornerKHz > DfsCpuHighCornerKHz)
            DfsCpuLowCornerKHz = DfsCpuHighCornerKHz;
    }

    // Finally update CPU limits
    pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu] = DfsCpuLowCornerKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu] = DfsCpuHighCornerKHz;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsSetEmcEnvelope(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmFreqKHz DfsEmcLowCornerKHz,
    NvRmFreqKHz DfsEmcHighCornerKHz)
{
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to set if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Preserve unspecified boundary, unless it violates new setting for
    // the other one; set both boundaries equal in the latter case
    if (DfsEmcLowCornerKHz == NvRmFreqUnspecified)
    {
        DfsEmcLowCornerKHz = pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Emc];
        if (DfsEmcLowCornerKHz > DfsEmcHighCornerKHz)
            DfsEmcLowCornerKHz = DfsEmcHighCornerKHz;
    }
    if (DfsEmcHighCornerKHz == NvRmFreqUnspecified)
    {
        DfsEmcHighCornerKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc];
        if (DfsEmcLowCornerKHz > DfsEmcHighCornerKHz)
            DfsEmcHighCornerKHz = DfsEmcLowCornerKHz;
    }

    // Can not set envelope with reversed boundaries
    if (DfsEmcLowCornerKHz > DfsEmcHighCornerKHz)
    {
        NV_ASSERT(!"EMC envelope boundaries are reversed");
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_BadValue;
    }

    // Clip requested boundaries to EMC domain limits; mark envelope "set" if
    // any requested boundary is inside the limits
    pDfs->EmcEnvelopeSet = NV_FALSE;    // assume envelope is open

    if (DfsEmcLowCornerKHz <= pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz)
        DfsEmcLowCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz;
    else
    {
        pDfs->EmcEnvelopeSet = NV_TRUE; // envelope sealed
        if (DfsEmcLowCornerKHz >= pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz)
            DfsEmcLowCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz;
    }

    if (DfsEmcHighCornerKHz >= pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz)
        DfsEmcHighCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Emc].MaxKHz;
    else
    {
        pDfs->EmcEnvelopeSet = NV_TRUE; // envelope sealed
        if (DfsEmcHighCornerKHz <= pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz)
            DfsEmcHighCornerKHz = pDfs->DfsParameters[NvRmDfsClockId_Emc].MinKHz;
    }

    // Restore CPU corners from shadow. If set, clip EMC envelope to the supported
    // EMC configuration, and throttle CPU corners as necessary
    pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu] = pDfs->CpuCornersShadow.MinKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu] = pDfs->CpuCornersShadow.MaxKHz;
    if (pDfs->EmcEnvelopeSet)
    {
        NvRmFreqKHz CpuHighKHz = pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu];
        DfsClipCpuEmcHighLimits(
            hRmDeviceHandle, &CpuHighKHz, &DfsEmcHighCornerKHz);
        if (DfsEmcLowCornerKHz > DfsEmcHighCornerKHz)
            DfsEmcLowCornerKHz = DfsEmcHighCornerKHz;

        if (pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu] > CpuHighKHz)
        {
            pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Cpu] = CpuHighKHz;
            if (pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu] > CpuHighKHz)
                pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Cpu] = CpuHighKHz;
        }
    }
    // Finally update EMC limits
    pDfs->LowCornerKHz.Domains[NvRmDfsClockId_Emc] = DfsEmcLowCornerKHz;
    pDfs->HighCornerKHz.Domains[NvRmDfsClockId_Emc] = DfsEmcHighCornerKHz;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsSetTarget( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 DfsFreqListCount,
    const NvRmFreqKHz* pDfsTargetFreqList)
{
    NvU32 i;
    NvRmDfsFrequencies DfsKHz;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(DfsFreqListCount == NvRmDfsClockId_Num);
    NV_ASSERT(pDfsTargetFreqList);

    NvRmPrivLockSharedPll();
    DfsClockFreqGet(hRmDeviceHandle, &DfsKHz);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Do nothing if DFS is not stopped (disabled or running)
    if (pDfs->DfsRunState != NvRmDfsRunState_Stopped)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        NvRmPrivUnlockSharedPll();
        return NvError_NotSupported;
    }

    // Clip requested target frequencies to domain limits
    // (keep current frequency as a target if new value is "unspecified")
    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        NvRmFreqKHz DomainKHz = pDfsTargetFreqList[i];
        if (DomainKHz != NvRmFreqUnspecified)
        {
            if (DomainKHz < pDfs->LowCornerKHz.Domains[i])
            {
                DomainKHz = pDfs->LowCornerKHz.Domains[i];
            }
            else if (DomainKHz > pDfs->HighCornerKHz.Domains[i])
            {
                DomainKHz = pDfs->HighCornerKHz.Domains[i];
            }
            DfsKHz.Domains[i] = DomainKHz;
        }
    }

    // Set target and signal clock control thread ("manual clock control")
    pDfs->TargetKHz = DfsKHz;
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    NvRmPrivUnlockSharedPll();
    NvOsSemaphoreSignal(pDfs->hSemaphore);
    return NvSuccess;
}

NvError
NvRmDfsGetClockUtilization(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsClockId ClockId,
    NvRmDfsClockUsage* pClockUsage)
{
    NvRmDfsFrequencies DfsKHz;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(pClockUsage);
    NV_ASSERT((0 < ClockId) && (ClockId < NvRmDfsClockId_Num));

    NvRmPrivLockSharedPll();
    DfsClockFreqGet(hRmDeviceHandle, &DfsKHz);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // If DFS is not running - update current frequencies directly from h/w
    if (pDfs->DfsRunState <= NvRmDfsRunState_Stopped)
    {
        pDfs->CurrentKHz = DfsKHz;
        if (pDfs->Samplers[ClockId].MonitorPresent)
            pDfs->Samplers[ClockId].AverageKHz = DfsKHz.Domains[ClockId];
    }
    // Update clock info
    pClockUsage->MinKHz = pDfs->DfsParameters[ClockId].MinKHz;
    pClockUsage->MaxKHz = pDfs->DfsParameters[ClockId].MaxKHz;
    pClockUsage->LowCornerKHz = pDfs->LowCornerKHz.Domains[ClockId];
    pClockUsage->HighCornerKHz = pDfs->HighCornerKHz.Domains[ClockId];
    pClockUsage->CurrentKHz = pDfs->CurrentKHz.Domains[ClockId];
    pClockUsage->AverageKHz = pDfs->Samplers[ClockId].AverageKHz; 

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    NvRmPrivUnlockSharedPll();
    return NvSuccess;
}

NvError
NvRmDfsGetProfileData( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 DfsProfileCount,
    NvU32* pSamplesNoList,
    NvU32* pProfileTimeUsList,
    NvU32* pDfsPeriodUs)
{
#if DFS_PROFILING
    NvU32 i;
    NvRmDfs* pDfs = &s_Dfs;
    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(pProfileTimeUsList && pSamplesNoList && pDfsPeriodUs);
    NV_ASSERT(DfsProfileCount == NvRmDfsProfileId_Num);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // Nothing to return if DFS is not in profiled loop
    if (pDfs->DfsRunState != NvRmDfsRunState_ProfiledLoop)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }
    // Return profile data
    for (i = 1; i < DfsProfileCount; i++)
    {
        pSamplesNoList[i] = s_Profile.SamplesNo[i];
        pProfileTimeUsList[i] = s_Profile.AccumulatedUs[i];
    }
    *pDfsPeriodUs = pDfs->SamplingWindow.SampleWindowMs * 1000 /
        NV_ARRAY_SIZE(pDfs->SamplingWindow.IntervalsMs);

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;

#else

    return NvError_NotSupported;
#endif
}

void
NvRmDfsLogStart(NvRmDeviceHandle hRmDeviceHandle)
{
    NvU32 i;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);

    NvOsIntrMutexLock(pDfs->hIntrMutex);
    s_DfsLogOn = NV_TRUE;

    for (i = 1; i < NvRmDfsClockId_Num; i++)
    {
        pDfs->Samplers[i].CumulativeLogCycles = 0;
    }
    pDfs->SamplingWindow.CumulativeLogMs = 0;
    pDfs->SamplingWindow.CumulativeLp2TimeMs = 0;
    pDfs->SamplingWindow.CumulativeLp2Entries = 0;
    
#if DFS_LOGGING_SECONDS
    s_DfsLogWrIndex = 0;
    s_DfsLogStarvationWrIndex = 0;
    s_DfsLogBusyWrIndex = 0;
#endif
    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
}

NvError
NvRmDfsLogGetMeanFrequencies(
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 LogMeanFreqListCount,
    NvRmFreqKHz* pLogMeanFreqList,
    NvU32* pLogLp2TimeMs,
    NvU32* pLogLp2Entries)
{
    NvU32 i, msec;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(LogMeanFreqListCount == NvRmDfsClockId_Num);
    NV_ASSERT(pLogMeanFreqList);

    NvOsIntrMutexLock(pDfs->hIntrMutex);
    s_DfsLogOn = NV_FALSE;

    // No logging if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Return cumulative mean frequencies: (Kcycles/ms) * 1000 = kHz;
    // (if log never started or running more than 49 days return 0)
    msec = pDfs->SamplingWindow.CumulativeLogMs;
    for (i = 1; i < LogMeanFreqListCount; i++)
    {
        pLogMeanFreqList[i] = 
            (NvU32)NvDiv64(pDfs->Samplers[i].CumulativeLogCycles, msec);
    }
    // TODO: update if condition SystemKHz = AvpKHz changes
    pLogMeanFreqList[NvRmDfsClockId_System] =
        pLogMeanFreqList[NvRmDfsClockId_Avp];

    // Return cumulative LP2 statistic
    *pLogLp2TimeMs = pDfs->SamplingWindow.CumulativeLp2TimeMs;
    *pLogLp2Entries = pDfs->SamplingWindow.CumulativeLp2Entries;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;
}

NvError
NvRmDfsLogActivityGetEntry( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 EntryIndex,
    NvU32 LogDomainsCount,
    NvU32* pIntervalMs,
    NvU32* pLp2TimeMs,
    NvU32* pActiveCyclesList,
    NvRmFreqKHz* pAveragesList,
    NvRmFreqKHz* pFrequenciesList)
{
#if DFS_LOGGING_SECONDS
    NvU32 i;
    DfsLogEntry* pEntry;
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(pFrequenciesList && pActiveCyclesList && pIntervalMs);
    NV_ASSERT(LogDomainsCount == NvRmDfsClockId_Num);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // No logging if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Nothing to return if log is empty
    if (EntryIndex >= s_DfsLogWrIndex)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_InvalidAddress;
    }

    // Return log data
    NV_ASSERT(EntryIndex < DFS_LOG_SIZE);
    pEntry = &s_DfsLog[EntryIndex];
    for (i = 1; i < LogDomainsCount; i++)
    {
        pFrequenciesList[i] = pEntry->CurrentKHz.Domains[i];
        pAveragesList[i] = pEntry->AverageKHz.Domains[i];
        pActiveCyclesList[i] = pEntry->ActiveCycles[i];
    }
    *pIntervalMs = pEntry->SampleIntervalMs;
    *pLp2TimeMs = pEntry->Lp2TimeMs;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;

#else

    return NvError_NotSupported;
#endif
}

NvError
NvRmDfsLogStarvationGetEntry( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 EntryIndex,
    NvU32* pSampleIndex,
    NvU32* pClientId,
    NvU32* pClientTag,
    NvRmDfsStarvationHint* pStarvationHint)
{
#if DFS_LOGGING_SECONDS
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(pSampleIndex && pStarvationHint);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // No logging if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Nothing to return if requested entry index is empty
    if (EntryIndex >= s_DfsLogStarvationWrIndex)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_InvalidAddress;
    }

    // Return log data
    NV_ASSERT(EntryIndex < DFS_LOG_SIZE);
    *pSampleIndex = s_DfsLogStarvation[EntryIndex].LogSampleIndex;
    *pClientId = s_DfsLogStarvation[EntryIndex].ClientId;
    *pClientTag = s_DfsLogStarvation[EntryIndex].ClientTag;
    *pStarvationHint = s_DfsLogStarvation[EntryIndex].StarvationHint;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;

#else

    return NvError_NotSupported;
#endif
}

NvError
NvRmDfsLogBusyGetEntry( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvU32 EntryIndex,
    NvU32* pSampleIndex,
    NvU32* pClientId,
    NvU32* pClientTag,
    NvRmDfsBusyHint* pBusyHint)
{
#if DFS_LOGGING_SECONDS
    NvRmDfs* pDfs = &s_Dfs;

    NV_ASSERT(hRmDeviceHandle);
    NV_ASSERT(pDfs->hIntrMutex);
    NV_ASSERT(pSampleIndex && pBusyHint);

    NvOsIntrMutexLock(pDfs->hIntrMutex);

    // No logging if DFS is disabled
    if (pDfs->DfsRunState == NvRmDfsRunState_Disabled)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_NotSupported;
    }

    // Nothing to return if requested entry index is empty
    if (EntryIndex >= s_DfsLogBusyWrIndex)
    {
        NvOsIntrMutexUnlock(pDfs->hIntrMutex);
        return NvError_InvalidAddress;
    }

    // Return log data
    NV_ASSERT(EntryIndex < DFS_LOG_SIZE);
    *pSampleIndex = s_DfsLogBusy[EntryIndex].LogSampleIndex;
    *pClientId = s_DfsLogBusy[EntryIndex].ClientId;
    *pClientTag = s_DfsLogBusy[EntryIndex].ClientTag;
    *pBusyHint = s_DfsLogBusy[EntryIndex].BusyHint;

    NvOsIntrMutexUnlock(pDfs->hIntrMutex);
    return NvSuccess;

#else

    return NvError_NotSupported;
#endif
}

/*****************************************************************************/
// DVS PUBLIC INTERFACES
/*****************************************************************************/

void
NvRmDfsGetLowVoltageThreshold(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsVoltageRailId RailId,
    NvRmMilliVolts* pLowMv,
    NvRmMilliVolts* pPresentMv)
{
    NV_ASSERT(hRmDeviceHandle);

    NvRmPrivLockSharedPll();
    NvRmPrivGetLowVoltageThreshold(RailId, pLowMv, pPresentMv);
    NvRmPrivUnlockSharedPll();
}

void
NvRmDfsSetLowVoltageThreshold(
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmDfsVoltageRailId RailId,
    NvRmMilliVolts LowMv)
{
    NvRmDvs* pDvs = &s_Dfs.VoltageScaler;

    NV_ASSERT(hRmDeviceHandle);

    // Low threshold is not specified - exit
    if (LowMv == NvRmVoltsUnspecified)
        return;

    NvRmPrivLockSharedPll();

    switch (RailId)
    {
        case NvRmDfsVoltageRailId_Core:
            // Clip specified voltage level to core voltage range,
            // and update low voltage settings
            if (LowMv > pDvs->NominalCoreMv)
                LowMv = pDvs->NominalCoreMv;
            else if (LowMv < pDvs->MinCoreMv)
                LowMv = pDvs->MinCoreMv;
            pDvs->LowCornerCoreMv = LowMv;
            pDvs->UpdateFlag = NV_TRUE;
            break;

        case NvRmDfsVoltageRailId_Cpu:
            if (NvRmPrivIsCpuRailDedicated(hRmDeviceHandle))
            {
                // Clip specified voltage level to CPU voltage range,
                // and update low voltage settings
                if (LowMv > pDvs->NominalCpuMv)
                    LowMv = pDvs->NominalCpuMv;
                else if (LowMv < pDvs->MinCpuMv)
                    LowMv = pDvs->MinCpuMv;
                pDvs->LowCornerCpuMv = LowMv;
                pDvs->UpdateFlag = NV_TRUE;
            }
            break;

        default:
            break;
    }
    NvRmPrivUnlockSharedPll();
}

/*****************************************************************************/
// DTT PUBLIC INTERFACES
/*****************************************************************************/

NvError
NvRmDiagGetTemperature( 
    NvRmDeviceHandle hRmDeviceHandle,
    NvRmTmonZoneId ZoneId,
    NvS32* pTemperatureC)
{
    NvRmDtt* pDtt = &s_Dfs.ThermalThrottler;

    NV_ASSERT(hRmDeviceHandle);

    switch (ZoneId)
    {
        case NvRmTmonZoneId_Core:
            if (pDtt->hOdmTcore)
            {
                if (NvOdmTmonTemperatureGet(pDtt->hOdmTcore, pTemperatureC))
                    return NvSuccess;
                return NvError_Busy;
            }
            // fall through
        default:
            return NvError_NotSupported;
    }
}

/*****************************************************************************/
