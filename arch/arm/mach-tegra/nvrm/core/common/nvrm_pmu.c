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

#include "nvrm_pmu.h"
#include "nvodm_pmu.h"
#include "nvassert.h"
#include "nvrm_interrupt.h"
#include "nvrm_moduleids.h"
#include "nvrm_pmu_private.h"
#include "nvrm_power_private.h"
#include "nvrm_clocks.h"
#include "nvrm_structure.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query.h"


// TODO: after testing is completed, remove this macro
// "1" - keep PMU rails always On, "0" - allows to power Off PMU rails
#define PMU_RAILS_NEVER_OFF (0)

// Retry count for voltage control API
#define VOLTAGE_CONTROL_RETRY_CNT (2)

/*
 * Combines RM PMU object data
 */
typedef struct NvRmPmuRec
{
    // RM PMU access mutex
    NvOsMutexHandle hMutex;

    // ODM PMU device handle
    NvOdmPmuDeviceHandle hOdmPmu;

    // PMU interrupt handle
    NvOsInterruptHandle hInterrupt;

    // PMU ISR semaphore
    NvOsSemaphoreHandle hSemaphore;

    // PMU interrupt thread
    NvOsThreadHandle hThread; 

    //  PMU interrupt mask state
    volatile NvBool IntrMasked;

    // PMU interrupt thread abort indicator
    volatile NvBool AbortThread;

    // IO power rails level detect mask
    NvU32 IoPwrDetectMask;

    // Unpowered IO rails mask
    NvU32 NoIoPwrMask;
} NvRmPmu;

// RM PMU object
static NvRmPmu s_Pmu;

// PMU supported execution environment flag
static NvBool s_PmuSupportedEnv = NV_FALSE;

/*****************************************************************************/

static void PmuIsr(void* args)
{
    NvRmPmu* pPmu = (NvRmPmu*)args;
    NvOsSemaphoreSignal(pPmu->hSemaphore);
}

static void PmuThread(void* args)
{
    NvRmPmu* pPmu = (NvRmPmu*)args;

    for (;;)
    {
        NvOsSemaphoreWait(pPmu->hSemaphore);
        if (pPmu->AbortThread)
        {
            break;
        }

        NvOsMutexLock(pPmu->hMutex);
        if (pPmu->IntrMasked)
        {
            NvOsMutexUnlock(pPmu->hMutex);
            continue;
        }
        NvOdmPmuInterruptHandler(pPmu->hOdmPmu);
        NvOsMutexUnlock(pPmu->hMutex);

        if (pPmu->hInterrupt)
            NvRmInterruptDone(pPmu->hInterrupt);
    }
}

static void PmuThreadTerminate(NvRmPmu* pPmu)
{
    /*
     * Request thread abort, signal semaphore to make sure the thread is
     * awaken and wait for its self-termination. Do nothing if invalid PMU
     * structure
     */
    if (pPmu)
    {
        if (pPmu->hSemaphore && pPmu->hThread)
        {
            pPmu->AbortThread = NV_TRUE;
            NvOsSemaphoreSignal(pPmu->hSemaphore);
            NvOsThreadJoin(pPmu->hThread);
        }
        pPmu->AbortThread = NV_FALSE;
    }
}

/*****************************************************************************/

NvError NvRmPrivPmuInit(NvRmDeviceHandle hRmDevice)
{
    NvError e;
    ExecPlatform env;
    NvOdmPmuProperty PmuProperty;

    NV_ASSERT(hRmDevice);
    env = NvRmPrivGetExecPlatform(hRmDevice);

    NvOsMemset(&s_Pmu, 0, sizeof(NvRmPmu));
    s_PmuSupportedEnv = NV_FALSE;

    if (env == ExecPlatform_Soc)
    {
        // Set supported environment flag
        s_PmuSupportedEnv = NV_TRUE;

        // Create the PMU mutex, semaphore, interrupt handler thread,
        // register PMU interrupt, and get ODM PMU handle
        NV_CHECK_ERROR_CLEANUP(NvOsMutexCreate(&s_Pmu.hMutex));
        NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&s_Pmu.hSemaphore, 0));

        if (NvOdmQueryGetPmuProperty(&PmuProperty) && PmuProperty.IrqConnected)
        {
            if (hRmDevice->ChipId.Id >= 0x20)
                NvRmPrivAp20SetPmuIrqPolarity(
                    hRmDevice, PmuProperty.IrqPolarity);
            else
                NV_ASSERT(PmuProperty.IrqPolarity ==
                          NvOdmInterruptPolarity_Low);
            {
                NvOsInterruptHandler hPmuIsr = PmuIsr;
                NvU32 PmuExtIrq = NvRmGetIrqForLogicalInterrupt(
                    hRmDevice, NVRM_MODULE_ID(NvRmPrivModuleID_PmuExt, 0), 0);
                NV_CHECK_ERROR_CLEANUP(NvRmInterruptRegister(hRmDevice, 1,
                    &PmuExtIrq, &hPmuIsr, &s_Pmu, &s_Pmu.hInterrupt, NV_FALSE));
            }
        }

        if(!NvOdmPmuDeviceOpen(&s_Pmu.hOdmPmu))
        {
            e = NvError_NotInitialized;
            goto fail;
        }
        NV_CHECK_ERROR_CLEANUP(NvOsThreadCreate(PmuThread, &s_Pmu, &s_Pmu.hThread));
        NvRmPrivIoPowerControlInit(hRmDevice);
        NvRmPrivCoreVoltageInit(hRmDevice);
    }
    return NvSuccess;

fail:
    NvRmPrivPmuDeinit(hRmDevice);
    return e;
}

void NvRmPrivPmuInterruptEnable(NvRmDeviceHandle hRmDevice)
{
    if (s_Pmu.hInterrupt)
        NvRmInterruptEnable(hRmDevice, s_Pmu.hInterrupt);
}

void NvRmPrivPmuInterruptMask(NvRmDeviceHandle hRmDevice, NvBool mask)
{
    if (s_Pmu.hInterrupt)
    {

        NvOsMutexLock(s_Pmu.hMutex);
        s_Pmu.IntrMasked = mask;
        NvOsMutexUnlock(s_Pmu.hMutex);

        NvOsInterruptMask(s_Pmu.hInterrupt, mask);
    }
}

void NvRmPrivPmuDeinit(NvRmDeviceHandle hRmDevice)
{
    if (s_PmuSupportedEnv == NV_FALSE)
        return;

    PmuThreadTerminate(&s_Pmu);
    NvOdmPmuDeviceClose(s_Pmu.hOdmPmu);
    NvRmInterruptUnregister(hRmDevice, s_Pmu.hInterrupt);
    NvOsSemaphoreDestroy(s_Pmu.hSemaphore);
    NvOsMutexDestroy(s_Pmu.hMutex);

    NvOsMemset(&s_Pmu, 0, sizeof(NvRmPmu));
    s_PmuSupportedEnv = NV_FALSE;
}

void NvRmPrivPmuLPxStateConfig(
    NvRmDeviceHandle hRmDevice,
    NvOdmSocPowerState state,
    NvBool enter)
{
    NvOdmPmuProperty PmuProperty = {0};
    NvBool HasPmuProperty = NvOdmQueryGetPmuProperty(&PmuProperty);
    const NvOdmPeripheralConnectivity* pCoreRail =
        NvOdmPeripheralGetGuid(NV_VDD_CORE_ODM_ID);

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pCoreRail);
    NV_ASSERT(pCoreRail->NumAddress);

    // On platforms with combined cpu/core power request core power rail
    // should be controlled by the combined request only during deep sleep
    // - enable the On/Off control on entry, and disable on exit
    if (state == NvOdmSocPowerState_DeepSleep)
    {
        if (HasPmuProperty && PmuProperty.CombinedPowerReq)
        {
            NvU32 level = enter ?
                ODM_VOLTAGE_ENABLE_EXT_ONOFF : ODM_VOLTAGE_DISABLE_EXT_ONOFF;
            NvRmPmuSetVoltage(hRmDevice, pCoreRail->AddressList[0].Address,
                              level, NULL);
        }
    }
    // Mask/Unmask PMU interrupt on entry/exit to/from suspend or deep sleep
    if ((state == NvOdmSocPowerState_Suspend) ||
        (state == NvOdmSocPowerState_DeepSleep))
        NvRmPrivPmuInterruptMask(hRmDevice, enter);
}

/*****************************************************************************/

void NvRmPmuGetCapabilities( 
    NvRmDeviceHandle hDevice,
    NvU32 vddId,
    NvRmPmuVddRailCapabilities * pCapabilities )
{
    NvOdmPmuVddRailCapabilities RailCap;

    if (!s_PmuSupportedEnv)
        return;

    NvOdmPmuGetCapabilities(vddId, &RailCap);
    pCapabilities->MaxMilliVolts = RailCap.MaxMilliVolts;
    pCapabilities->MinMilliVolts = RailCap.MinMilliVolts;
    pCapabilities->requestMilliVolts = RailCap.requestMilliVolts;
    pCapabilities->RmProtected = RailCap.OdmProtected;
    pCapabilities->StepMilliVolts = RailCap.StepMilliVolts;
}

void NvRmPmuGetVoltage(
    NvRmDeviceHandle hDevice,
    NvU32 vddId,
    NvU32 * pMilliVolts)
{
    NvU32 i;
    if (!s_PmuSupportedEnv)
        return;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    for (i = 0; i < VOLTAGE_CONTROL_RETRY_CNT; i++)
    {
        if (NvOdmPmuGetVoltage(s_Pmu.hOdmPmu, vddId, pMilliVolts))
            break;
    }
    NV_ASSERT(i < VOLTAGE_CONTROL_RETRY_CNT);
    NvOsMutexUnlock(s_Pmu.hMutex);
}

void NvRmPmuSetVoltage( 
    NvRmDeviceHandle hDevice,
    NvU32 vddId,
    NvU32 MilliVolts,
    NvU32 * pSettleMicroSeconds)
{
    NvU32 i;
    NvU32 t = NVRM_PWR_DET_DELAY_US;
    NV_ASSERT(hDevice);
    
    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = 0;

    if (!s_PmuSupportedEnv)
        return;

    // This API is blocked if diagnostic is in progress for any module 
    if (NvRmPrivIsDiagMode(NvRmModuleID_Invalid))
        return;

#if PMU_RAILS_NEVER_OFF
    if (MilliVolts == ODM_VOLTAGE_OFF)
        return;
#endif

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);

    // Set voltage and latch IO level sampling results
    for (i = 0; i < VOLTAGE_CONTROL_RETRY_CNT; i++)
    {
        if (NvOdmPmuSetVoltage(s_Pmu.hOdmPmu, vddId, MilliVolts, pSettleMicroSeconds))
            break;
    }
    NV_ASSERT(i < VOLTAGE_CONTROL_RETRY_CNT);

    if (s_Pmu.IoPwrDetectMask || s_Pmu.NoIoPwrMask)
    {
        NV_ASSERT(MilliVolts != ODM_VOLTAGE_OFF);
        if (pSettleMicroSeconds)
        {
            t += (*pSettleMicroSeconds);
            *pSettleMicroSeconds = 0;           // Don't wait twice
        }
        NvOsWaitUS(t);

        if (s_Pmu.IoPwrDetectMask)              // Latch just powered IO rails
            NvRmPrivIoPowerDetectLatch(hDevice);
        if (s_Pmu.NoIoPwrMask)                  // Enable just powered IO rails
            NvRmPrivIoPowerControl(hDevice, s_Pmu.NoIoPwrMask, NV_TRUE);
        s_Pmu.IoPwrDetectMask = s_Pmu.NoIoPwrMask = 0;
    }
    NvOsMutexUnlock(s_Pmu.hMutex);
}

void
NvRmPmuSetSocRailPowerState(
    NvRmDeviceHandle hDevice,
    NvU32 vddId,
    NvBool Enable)
{
    NV_ASSERT(hDevice);
    NvRmPrivSetSocRailPowerState(
        hDevice, vddId, Enable, &s_Pmu.IoPwrDetectMask, &s_Pmu.NoIoPwrMask);
}

/*****************************************************************************/

void NvRmPmuSetChargingCurrentLimit(
    NvRmDeviceHandle hRmDevice,
    NvRmPmuChargingPath ChargingPath,
    NvU32 ChargingCurrentLimitMa,
    NvU32 ChargerType)
{
    NvU32 i;

    if (!s_PmuSupportedEnv)
        return;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    for (i = 0; i < VOLTAGE_CONTROL_RETRY_CNT; i++)
    {
        if (NvOdmPmuSetChargingCurrent(
            s_Pmu.hOdmPmu, (NvOdmPmuChargingPath)ChargingPath, 
            ChargingCurrentLimitMa, ChargerType))
            break;
    }
    NV_ASSERT(i < VOLTAGE_CONTROL_RETRY_CNT);
    NvOsMutexUnlock(s_Pmu.hMutex);
}

/*****************************************************************************/

NvBool NvRmPmuGetAcLineStatus( 
    NvRmDeviceHandle hRmDevice,
    NvRmPmuAcLineStatus * pStatus )
{
    NvBool ReturnStatus = NV_FALSE;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    ReturnStatus =
        NvOdmPmuGetAcLineStatus(s_Pmu.hOdmPmu, (NvOdmPmuAcLineStatus*)pStatus);
    NvOsMutexUnlock(s_Pmu.hMutex);
    return ReturnStatus;
}

 NvBool NvRmPmuGetBatteryStatus( 
    NvRmDeviceHandle hRmDevice,
    NvRmPmuBatteryInstance batteryInst,
    NvU8 * pStatus )
{
    NvBool ReturnStatus = NV_FALSE;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    ReturnStatus = NvOdmPmuGetBatteryStatus(
        s_Pmu.hOdmPmu, (NvOdmPmuBatteryInstance)batteryInst, pStatus);
    NvOsMutexUnlock(s_Pmu.hMutex);
    return ReturnStatus;
}

/*****************************************************************************/

NvBool NvRmPmuGetBatteryData( 
    NvRmDeviceHandle hRmDevice,
    NvRmPmuBatteryInstance batteryInst,
    NvRmPmuBatteryData * pData )
{
    NvOdmPmuBatteryData BatteryData;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    if (NvOdmPmuGetBatteryData(
        s_Pmu.hOdmPmu, (NvOdmPmuBatteryInstance)batteryInst, &BatteryData))
    {
        pData->batteryAverageCurrent = BatteryData.batteryAverageCurrent;
        pData->batteryAverageInterval = BatteryData.batteryAverageInterval;
        pData->batteryCurrent = BatteryData.batteryCurrent;
        pData->batteryLifePercent = BatteryData.batteryLifePercent;
        pData->batteryLifeTime = BatteryData.batteryLifeTime;
        pData->batteryMahConsumed = BatteryData.batteryMahConsumed;
        pData->batteryTemperature = BatteryData.batteryTemperature;
        pData->batteryVoltage = BatteryData.batteryVoltage;
        NvOsMutexUnlock(s_Pmu.hMutex);
        return NV_TRUE;
    }
    NvOsMutexUnlock(s_Pmu.hMutex);
    return NV_FALSE;
}

void NvRmPmuGetBatteryFullLifeTime( 
    NvRmDeviceHandle hRmDevice,
    NvRmPmuBatteryInstance batteryInst,
    NvU32 * pLifeTime )
{
    if (!s_PmuSupportedEnv)
        return;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    NvOdmPmuGetBatteryFullLifeTime(
        s_Pmu.hOdmPmu,(NvOdmPmuBatteryInstance)batteryInst, pLifeTime);
    NvOsMutexUnlock(s_Pmu.hMutex);
}

void NvRmPmuGetBatteryChemistry( 
    NvRmDeviceHandle hRmDevice,
    NvRmPmuBatteryInstance batteryInst,
    NvRmPmuBatteryChemistry * pChemistry )
{
    if (!s_PmuSupportedEnv)
        return;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    NvOdmPmuGetBatteryChemistry(s_Pmu.hOdmPmu,
                                (NvOdmPmuBatteryInstance)batteryInst,
                                (NvOdmPmuBatteryChemistry*)pChemistry);
    NvOsMutexUnlock(s_Pmu.hMutex);
}

/*****************************************************************************/

NvBool
NvRmPmuReadRtc(
    NvRmDeviceHandle hRmDevice,
    NvU32* pCount)
{
    NvBool ReturnStatus = NV_FALSE;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    ReturnStatus = NvOdmPmuReadRtc(s_Pmu.hOdmPmu, pCount);
    NvOsMutexUnlock(s_Pmu.hMutex);
    return ReturnStatus;
}

NvBool
NvRmPmuWriteRtc(
    NvRmDeviceHandle hRmDevice,
    NvU32 Count)
{
    NvBool ReturnStatus = NV_FALSE;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    ReturnStatus = NvOdmPmuWriteRtc(s_Pmu.hOdmPmu, Count);
    NvOsMutexUnlock(s_Pmu.hMutex);
    return ReturnStatus;
}

NvBool
NvRmPmuIsRtcInitialized(
    NvRmDeviceHandle hRmDevice)
{
    NvBool ReturnStatus = NV_FALSE;

    if (!s_PmuSupportedEnv)
        return NV_FALSE;

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    ReturnStatus = NvOdmPmuIsRtcInitialized(s_Pmu.hOdmPmu);
    NvOsMutexUnlock(s_Pmu.hMutex);
    return ReturnStatus;
}

/*****************************************************************************/

NvBool
NvRmPrivDiagPmuSetVoltage( 
    NvRmDeviceHandle hDevice,
    NvU32 vddId,
    NvU32 MilliVolts,
    NvU32 * pSettleMicroSeconds)
{
    NvU32 i;

    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = 0;

    NV_ASSERT(s_PmuSupportedEnv);
    NV_ASSERT(NvRmPrivIsDiagMode(NvRmModuleID_Invalid));

    NV_ASSERT(s_Pmu.hMutex);
    NvOsMutexLock(s_Pmu.hMutex);
    for (i = 0; i < VOLTAGE_CONTROL_RETRY_CNT; i++)
    {
        if (NvOdmPmuSetVoltage(s_Pmu.hOdmPmu, vddId, MilliVolts, pSettleMicroSeconds))
            break;
    }
    NvOsMutexUnlock(s_Pmu.hMutex);

    return (i < VOLTAGE_CONTROL_RETRY_CNT);
}

/*****************************************************************************/

void
NvRmPrivPmuRailControl(
    NvRmDeviceHandle hRmDevice,
    NvU64 NvRailId,
    NvBool TurnOn)
{
    NvU32 RailAddress, TimeUs;
    const NvOdmPeripheralConnectivity* pPmuRail;
    NvRmPmuVddRailCapabilities RailCapabilities = {0};

    if (!s_PmuSupportedEnv)
        return;

    pPmuRail = NvOdmPeripheralGetGuid(NvRailId);

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pPmuRail);
    NV_ASSERT(pPmuRail->NumAddress);

    RailAddress = pPmuRail->AddressList[0].Address;
    if (TurnOn)
    {
        NvRmPmuGetCapabilities(hRmDevice, RailAddress, &RailCapabilities);
        NvRmPmuSetVoltage(hRmDevice, RailAddress,
                          RailCapabilities.requestMilliVolts, &TimeUs);
    }
    else
    {
        NvRmPmuSetVoltage(
            hRmDevice, RailAddress, ODM_VOLTAGE_OFF, &TimeUs);
    }
    NvOsWaitUS(TimeUs);
}

NvU32
NvRmPrivPmuRailGetVoltage(
    NvRmDeviceHandle hRmDevice,
    NvU64 NvRailId)
{
    NvU32 RailAddress;
    const NvOdmPeripheralConnectivity* pPmuRail;
    NvU32 MilliVolts = NVRM_NO_PMU_DEFAULT_VOLTAGE;

    if (s_PmuSupportedEnv)
    {
        pPmuRail = NvOdmPeripheralGetGuid(NvRailId);

        NV_ASSERT(hRmDevice);
        NV_ASSERT(pPmuRail);
        NV_ASSERT(pPmuRail->NumAddress);

        RailAddress = pPmuRail->AddressList[0].Address;
        NvRmPmuGetVoltage(hRmDevice, RailAddress, &MilliVolts);
    }
    return MilliVolts;
}

/*****************************************************************************/
