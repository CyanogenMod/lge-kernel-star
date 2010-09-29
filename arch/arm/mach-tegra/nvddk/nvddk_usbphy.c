/*
 * Copyright (c) 2009 NVIDIA Corporation.
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
 * @brief <b>NVIDIA Driver Development Kit:
 *           NvDDK USB PHY functions</b>
 *
 * @b Description: Defines USB PHY private functions
 *
 */

#include "nvrm_pinmux.h"
#include "nvrm_power_private.h"
#include "nvrm_pmu.h"
#include "nvrm_hardware_access.h"
#include "nvddk_usbphy_priv.h"
#include "nvodm_services.h"

#define MAX_USB_INSTANCES 5

// On platforms that never disable USB controller clock, use 1KHz as an
// indicator that USB controller is idle, and core voltage can be scaled down
#define USBC_IDLE_KHZ (1)

static NvDdkUsbPhy *s_pUsbPhy = NULL;
static NvDdkUsbPhyUtmiPadConfig *s_pUtmiPadConfig = NULL;
static NvOsMutexHandle   s_UsbPhyMutex = NULL;


static void UsbPrivEnableVbus(NvDdkUsbPhy *pUsbPhy, NvBool Enable)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvRmPmuVddRailCapabilities RailCaps;
    NvU32 i;

    switch (pUsbPhy->Instance)
    {
        case 0:
            pConnectivity = NvOdmPeripheralGetGuid(NV_VDD_VBUS_ODM_ID);
            break;
        case 1:
            pConnectivity = NvOdmPeripheralGetGuid(NV_VDD_USB2_VBUS_ODM_ID);
            break;
        case 2:
            pConnectivity = NvOdmPeripheralGetGuid(NV_VDD_USB3_VBUS_ODM_ID);
            break;
        default:
            break;
    }

    if (pConnectivity != NULL)
    {
        for (i = 0; i < pConnectivity->NumAddress; i++)
        {
            // Search for the vdd rail entry
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd)
            {
                NvRmPmuGetCapabilities(pUsbPhy->hRmDevice,
                            pConnectivity->AddressList[i].Address, &RailCaps);

                if (Enable)
                {
                    NvRmPmuSetVoltage(pUsbPhy->hRmDevice,
                            pConnectivity->AddressList[i].Address, RailCaps.requestMilliVolts, NULL);
                }
                else
                {
                    NvRmPmuSetVoltage(pUsbPhy->hRmDevice,
                            pConnectivity->AddressList[i].Address, ODM_VOLTAGE_OFF, NULL);
                }
            }
            else if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_Gpio)
            {
                if ((!pUsbPhy->hGpio))
                {
                    pUsbPhy->hGpio = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
                    if (!pUsbPhy->hGpio)
                    {
                        return;
                    }
                }
                if (!pUsbPhy->hPin)
                {
                    NvU32 GpioPort = 0, GpioPin = 0;
                    GpioPort = pConnectivity->AddressList[0].Instance;
                    GpioPin = pConnectivity->AddressList[0].Address;
                    pUsbPhy->hPin = NvOdmGpioAcquirePinHandle(pUsbPhy->hGpio, GpioPort, GpioPin);
                    if (!pUsbPhy->hPin)
                    {
                        NvOdmGpioClose(pUsbPhy->hGpio);
                        return;
                    }
                }
                NvOdmGpioConfig(pUsbPhy->hGpio, pUsbPhy->hPin, NvOdmGpioPinMode_Output);
                if (!Enable)
                    NvOdmGpioSetState(pUsbPhy->hGpio, pUsbPhy->hPin, 0);
                else
                    NvOdmGpioSetState(pUsbPhy->hGpio, pUsbPhy->hPin, 1);
            }
        }
    }
}

static NvBool UsbPhyTurnOffPowerRail(NvU32 MaxInstances)
{
    NvBool TurnOff = NV_FALSE;
    NvU32 instance = 0;
    const NvOdmUsbProperty *pProperty = NULL;

    for (instance = 0; instance < MaxInstances; instance++)
    {
        pProperty = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, instance);

        if (pProperty)
        {
            if (pProperty->UsbMode == NvOdmUsbModeType_None)
            {
                continue;
            }
            else if (((pProperty->UsbMode == NvOdmUsbModeType_Device) ||
                (pProperty->UsbMode == NvOdmUsbModeType_OTG)) &&
                (!pProperty->UseInternalPhyWakeup))
            {
                TurnOff = NV_TRUE;
            }
            else if (((pProperty->UsbMode == NvOdmUsbModeType_Host) &&
                (pProperty->IdPinDetectionType == NvOdmUsbIdPinType_CableId)) &&
                (!pProperty->UseInternalPhyWakeup))
            {
                TurnOff = NV_TRUE;
            }
            else
            {
                TurnOff = NV_FALSE;
            }
        }
    }

    return TurnOff;
}


static void
UsbPhyOpenHwInterface(
    NvDdkUsbPhyHandle hUsbPhy)
{
    static NvDdkUsbPhyCapabilities s_UsbPhyCap[] =
    {
        //  AP15
        { NV_FALSE, NV_FALSE },
        //  AP16
        { NV_FALSE, NV_TRUE },
        //  AP20
        { NV_TRUE, NV_FALSE},
    };
    NvDdkUsbPhyCapabilities *pUsbfCap = NULL;
    NvRmModuleCapability s_UsbPhyCaps[] =
    {
        {1, 0, 0, &s_UsbPhyCap[0]},  // AP15 A01
        {1, 1, 0, &s_UsbPhyCap[0]},  // AP15 A02
        {1, 2, 0, &s_UsbPhyCap[1]},  // AP16, USB1
        {1, 3, 0, &s_UsbPhyCap[1]},  // AP16, USB2
        {1, 5, 0, &s_UsbPhyCap[2]}, // AP20, USB1
        {1, 6, 0, &s_UsbPhyCap[2]}, // AP20, USB2
        {1, 7, 0, &s_UsbPhyCap[2]}, // AP20, USB3
    };

    NV_ASSERT_SUCCESS(
        NvRmModuleGetCapabilities(hUsbPhy->hRmDevice,
            NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
            s_UsbPhyCaps, NV_ARRAY_SIZE(s_UsbPhyCaps),
            (void**)&pUsbfCap));

    // Fill the client capabilities structure.
    NvOsMemcpy(&hUsbPhy->Caps, pUsbfCap, sizeof(NvDdkUsbPhyCapabilities));

    //NvOsDebugPrintf("NvDdkUsbPhyCapabilities::\n");
    //NvOsDebugPrintf("PhyRegInController::[%d] 0-FALSE 1-TRUE\n", hUsbPhy->Caps.PhyRegInController);
    //NvOsDebugPrintf("CommonClockAndReset::[%d] 0-FALSE 1-TRUE\n", hUsbPhy->Caps.CommonClockAndReset);

    if (hUsbPhy->Caps.PhyRegInController)
    {
        //NvOsDebugPrintf("AP20 USB Controllers\n");
        Ap20UsbPhyOpenHwInterface(hUsbPhy);
    }
}


static NvError
UsbPhyDfsBusyHint(
    NvDdkUsbPhyHandle hUsbPhy,
    NvBool DfsOn,
    NvU32 BoostDurationMs)
{
    NvRmDfsBusyHint pUsbHintOn[] =
    {
        { NvRmDfsClockId_Emc, NV_WAIT_INFINITE, USB_HW_MIN_SYSTEM_FREQ_KH, NV_TRUE },
        { NvRmDfsClockId_Ahb, NV_WAIT_INFINITE, USB_HW_MIN_SYSTEM_FREQ_KH, NV_TRUE },
        { NvRmDfsClockId_Cpu, NV_WAIT_INFINITE, USB_HW_MIN_CPU_FREQ_KH, NV_TRUE }
    };
    NvRmDfsBusyHint pUsbHintOff[] =
    {
        { NvRmDfsClockId_Emc, 0, 0, NV_TRUE },
        { NvRmDfsClockId_Ahb, 0, 0, NV_TRUE },
        { NvRmDfsClockId_Cpu, 0, 0, NV_TRUE }
    };
    NvError e = NvSuccess;
    NvU32 NumHints;

    if (hUsbPhy->IsHostMode)
    {
        // Do not enable busy hints for cpu clock in host mode
        NumHints = NV_ARRAY_SIZE(pUsbHintOn) - 1;
    }
    else
    {
        NumHints = NV_ARRAY_SIZE(pUsbHintOn);
    }

    pUsbHintOn[0].BoostDurationMs = BoostDurationMs;
    pUsbHintOn[1].BoostDurationMs = BoostDurationMs;
    pUsbHintOn[2].BoostDurationMs = BoostDurationMs;

    if (DfsOn)
    {
        if (hUsbPhy->Caps.PhyRegInController)
        {
            // Indicate USB controller is active
            NvRmFreqKHz PrefFreq = NvRmPowerModuleGetMaxFrequency(
                hUsbPhy->hRmDevice,
                NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance));

            NV_CHECK_ERROR_CLEANUP(
                NvRmPowerModuleClockConfig(hUsbPhy->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                    hUsbPhy->RmPowerClientId, PrefFreq, PrefFreq, &PrefFreq,
                    1, NULL, 0));
        }
        return NvRmPowerBusyHintMulti(hUsbPhy->hRmDevice,
                                      hUsbPhy->RmPowerClientId,
                                      pUsbHintOn,
                                      NumHints,
                                      NvRmDfsBusyHintSyncMode_Async);
    }
    else
    {
        if (hUsbPhy->Caps.PhyRegInController)
        {
            // Indicate USB controller is idle
            NvRmFreqKHz PrefFreq = USBC_IDLE_KHZ;

            NV_CHECK_ERROR_CLEANUP(
                NvRmPowerModuleClockConfig(hUsbPhy->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                    hUsbPhy->RmPowerClientId, PrefFreq, PrefFreq, &PrefFreq,
                    1, NULL, 0));
        }
        return NvRmPowerBusyHintMulti(hUsbPhy->hRmDevice,
                                      hUsbPhy->RmPowerClientId,
                                      pUsbHintOff,
                                      NumHints,
                                      NvRmDfsBusyHintSyncMode_Async);
    }

fail:
    return e;
}


static NvError
UsbPhyInitialize(
    NvDdkUsbPhyHandle hUsbPhy)
{
    NvError e = NvSuccess;
    NvRmFreqKHz CurrentFreq = 0;
    NvRmFreqKHz PrefFreqList[3] = {12000, 60000, NvRmFreqUnspecified};

    // NvOsDebugPrintf("UsbPhyInitialize::VOLTAGE ON, instance %d\n",
    //                hUsbPhy->Instance);
    // request power
    NV_CHECK_ERROR_CLEANUP(
        NvRmPowerVoltageControl(hUsbPhy->hRmDevice,
            NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
            hUsbPhy->RmPowerClientId, NvRmVoltsUnspecified,
            NvRmVoltsUnspecified, NULL, 0, NULL));

    // Enable clock to the USB controller and Phy
    NV_CHECK_ERROR_CLEANUP(
        NvRmPowerModuleClockControl(hUsbPhy->hRmDevice,
            NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                hUsbPhy->RmPowerClientId, NV_TRUE));

    if (!hUsbPhy->Caps.PhyRegInController)
    {
        if (hUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_UlpiNullPhy)
        {
            /* Request for 60MHz clk */
            NV_CHECK_ERROR_CLEANUP(
                NvRmPowerModuleClockConfig(hUsbPhy->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                    hUsbPhy->RmPowerClientId, PrefFreqList[1],
                    PrefFreqList[1], &PrefFreqList[1], 1, &CurrentFreq, 0));
        }
        else
        {
            /* Request for 12 MHz clk */
            NV_CHECK_ERROR_CLEANUP(
                NvRmPowerModuleClockConfig(hUsbPhy->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                    hUsbPhy->RmPowerClientId, PrefFreqList[0],
                    PrefFreqList[0], &PrefFreqList[0], 1, &CurrentFreq, 0));
        }
    }
    // else
    {
        /* No need for actual clock configuration - all USB PLL frequencies
         are available concurrently in this case. */
    }

    // Reset controller
    NvRmModuleReset(hUsbPhy->hRmDevice,
        NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance));


    // On AP20 H-CLK should not be turned off
    // This is required to detect the sensor interrupts.
    // However, phy can be programmed to put in the low power mode
    if (!hUsbPhy->Caps.PhyRegInController)
    {
        // Disable the clock
        NV_CHECK_ERROR_CLEANUP(
            NvRmPowerModuleClockControl(hUsbPhy->hRmDevice,
              NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
              hUsbPhy->RmPowerClientId, NV_FALSE));
    }

    // Disable power
    NV_CHECK_ERROR_CLEANUP(
        NvRmPowerVoltageControl(hUsbPhy->hRmDevice,
          NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
          hUsbPhy->RmPowerClientId, NvRmVoltsOff, NvRmVoltsOff,
          NULL, 0, NULL));

    // if we are not turning off the power rail during power up and down
    // then turn on only once during the initalization.
    if (!hUsbPhy->TurnOffPowerRail)
    {
        NvOdmEnableUsbPhyPowerRail(NV_TRUE);
    }

fail:

    return e;
}

static NvError
UsbPhyIoctlUsbBisuHintsOnOff(
    NvDdkUsbPhy *pUsbPhy,
    const void *pInputArgs)
{
   NvDdkUsbPhyIoctl_UsbBusyHintsOnOffInputArgs *pOnOff = NULL;

    if (!pInputArgs)
        return NvError_BadParameter;

    pOnOff = (NvDdkUsbPhyIoctl_UsbBusyHintsOnOffInputArgs *)pInputArgs;

    return UsbPhyDfsBusyHint(pUsbPhy, pOnOff->OnOff, pOnOff->BoostDurationMs);
}

NvError
NvDdkUsbPhyOpen(
    NvRmDeviceHandle hRm,
    NvU32 Instance,
    NvDdkUsbPhyHandle *hUsbPhy)
{
    NvError e;
    NvU32 MaxInstances = 0;
    NvDdkUsbPhy *pUsbPhy = NULL;
    NvOsMutexHandle UsbPhyMutex = NULL;
    NvRmModuleInfo info[MAX_USB_INSTANCES];
    NvU32 j;

    NV_ASSERT(hRm);
    NV_ASSERT(hUsbPhy);
    NV_ASSERT(Instance < MAX_USB_INSTANCES);

    NV_CHECK_ERROR(NvRmModuleGetModuleInfo( hRm, NvRmModuleID_Usb2Otg, &MaxInstances, NULL ));
    if (MaxInstances > MAX_USB_INSTANCES)
    {
       // Ceil "instances" to MAX_USB_INSTANCES
       MaxInstances = MAX_USB_INSTANCES;
    }
    NV_CHECK_ERROR(NvRmModuleGetModuleInfo( hRm, NvRmModuleID_Usb2Otg, &MaxInstances, info ));
    for (j = 0; j < MaxInstances; j++)
    {
    // Check whether the requested instance is present
        if(info[j].Instance == Instance)
            break;
    }
    // No match found return
    if (j == MaxInstances)
    {
        return NvError_ModuleNotPresent;
    }

    if (!s_UsbPhyMutex)
    {
        e = NvOsMutexCreate(&UsbPhyMutex);
        if (e!=NvSuccess)
            return e;

        if (NvOsAtomicCompareExchange32(
                (NvS32*)&s_UsbPhyMutex, 0, (NvS32)UsbPhyMutex)!=0)
        {
            NvOsMutexDestroy(UsbPhyMutex);
        }
    }

    NvOsMutexLock(s_UsbPhyMutex);
    if (!s_pUsbPhy)
    {
        s_pUsbPhy = NvOsAlloc(MaxInstances * sizeof(NvDdkUsbPhy));
        if (s_pUsbPhy)
            NvOsMemset(s_pUsbPhy, 0, MaxInstances * sizeof(NvDdkUsbPhy));
    }
    NvOsMutexUnlock(s_UsbPhyMutex);

    if (!s_pUsbPhy)
        return NvError_InsufficientMemory;

    NvOsMutexLock(s_UsbPhyMutex);
    if (!s_pUtmiPadConfig)
    {
        s_pUtmiPadConfig = NvOsAlloc(sizeof(NvDdkUsbPhyUtmiPadConfig));
        if (s_pUtmiPadConfig)
        {
            NvRmPhysAddr PhyAddr;

            NvOsMemset(s_pUtmiPadConfig, 0, sizeof(NvDdkUsbPhyUtmiPadConfig));
            NvRmModuleGetBaseAddress(
                hRm, 
                NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, 0),
                &PhyAddr, &s_pUtmiPadConfig->BankSize);

            NV_CHECK_ERROR_CLEANUP(
                NvRmPhysicalMemMap(
                    PhyAddr, s_pUtmiPadConfig->BankSize, NVOS_MEM_READ_WRITE,
                    NvOsMemAttribute_Uncached, (void **)&s_pUtmiPadConfig->pVirAdr));
        }
    }
    NvOsMutexUnlock(s_UsbPhyMutex);

    if (!s_pUtmiPadConfig)
        return NvError_InsufficientMemory;

    pUsbPhy = &s_pUsbPhy[Instance];

    NvOsMutexLock(s_UsbPhyMutex);
    if (!pUsbPhy->RefCount)
    {
        NvRmPhysAddr PhysAddr;
        NvOsMutexHandle ThreadSafetyMutex = NULL;

        NvOsMemset(pUsbPhy, 0, sizeof(NvDdkUsbPhy));
        pUsbPhy->Instance = Instance;
        pUsbPhy->hRmDevice = hRm;
        pUsbPhy->RefCount = 1;
        pUsbPhy->IsPhyPoweredUp = NV_FALSE;
        pUsbPhy->pUtmiPadConfig = s_pUtmiPadConfig;
        pUsbPhy->pProperty = NvOdmQueryGetUsbProperty(
                                    NvOdmIoModule_Usb, pUsbPhy->Instance);
        pUsbPhy->TurnOffPowerRail = UsbPhyTurnOffPowerRail(MaxInstances);

        NV_CHECK_ERROR_CLEANUP(NvOsMutexCreate(&ThreadSafetyMutex));
        if (NvOsAtomicCompareExchange32(
                (NvS32*)&pUsbPhy->ThreadSafetyMutex, 0,
                (NvS32)ThreadSafetyMutex)!=0)
        {
            NvOsMutexDestroy(ThreadSafetyMutex);
        }

        NvRmModuleGetBaseAddress(
            pUsbPhy->hRmDevice,
            NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, pUsbPhy->Instance),
            &PhysAddr, &pUsbPhy->UsbBankSize);

        NV_CHECK_ERROR_CLEANUP(
            NvRmPhysicalMemMap(
                PhysAddr, pUsbPhy->UsbBankSize, NVOS_MEM_READ_WRITE,
                NvOsMemAttribute_Uncached, (void **)&pUsbPhy->UsbVirAdr));

        NvRmModuleGetBaseAddress(
            pUsbPhy->hRmDevice,
            NVRM_MODULE_ID(NvRmModuleID_Misc, 0),
            &PhysAddr, &pUsbPhy->MiscBankSize);

        NV_CHECK_ERROR_CLEANUP(
            NvRmPhysicalMemMap(
                PhysAddr, pUsbPhy->MiscBankSize, NVOS_MEM_READ_WRITE,
                NvOsMemAttribute_Uncached, (void **)&pUsbPhy->MiscVirAdr));

        if ( ( pUsbPhy->pProperty->UsbInterfaceType ==
               NvOdmUsbInterfaceType_UlpiNullPhy) ||
             ( pUsbPhy->pProperty->UsbInterfaceType ==
               NvOdmUsbInterfaceType_UlpiExternalPhy))
        {
            if (NvRmSetModuleTristate(
                    pUsbPhy->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, pUsbPhy->Instance),
                    NV_FALSE) != NvSuccess )
               return NvError_NotSupported;
        }

        // Register with Power Manager
        NV_CHECK_ERROR_CLEANUP(
            NvOsSemaphoreCreate(&pUsbPhy->hPwrEventSem, 0));

        pUsbPhy->RmPowerClientId = NVRM_POWER_CLIENT_TAG('U','S','B','p');
        NV_CHECK_ERROR_CLEANUP(
            NvRmPowerRegister(pUsbPhy->hRmDevice,
            pUsbPhy->hPwrEventSem, &pUsbPhy->RmPowerClientId));

        // Open the H/W interface
        UsbPhyOpenHwInterface(pUsbPhy);

        // Initialize the USB Phy
        NV_CHECK_ERROR_CLEANUP(UsbPhyInitialize(pUsbPhy));
    }
    else
    {
        pUsbPhy->RefCount++;
    }

    *hUsbPhy = pUsbPhy;
    NvOsMutexUnlock(s_UsbPhyMutex);

    return NvSuccess;

fail:

    NvDdkUsbPhyClose(pUsbPhy);
    NvOsMutexUnlock(s_UsbPhyMutex);
    return e;
}


void
NvDdkUsbPhyClose(
    NvDdkUsbPhyHandle hUsbPhy)
{
    if (!hUsbPhy)
        return;

    NvOsMutexLock(s_UsbPhyMutex);

    if (!hUsbPhy->RefCount)
    {
        NvOsMutexUnlock(s_UsbPhyMutex);
        return;
    }

    --hUsbPhy->RefCount;

    if (hUsbPhy->RefCount)
    {
        NvOsMutexUnlock(s_UsbPhyMutex);
        return;
    }

    NvRmSetModuleTristate(
        hUsbPhy->hRmDevice,
        NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
        NV_TRUE);

    NvOsMutexLock(hUsbPhy->ThreadSafetyMutex);
    if (hUsbPhy->RmPowerClientId)
    {
        if (hUsbPhy->IsPhyPoweredUp)
        {
            NV_ASSERT_SUCCESS(
                NvRmPowerModuleClockControl(hUsbPhy->hRmDevice,
                  NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                  hUsbPhy->RmPowerClientId,
                  NV_FALSE));

            //NvOsDebugPrintf("NvDdkUsbPhyClose::VOLTAGE OFF\n");
            NV_ASSERT_SUCCESS(
                NvRmPowerVoltageControl(hUsbPhy->hRmDevice,
                  NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
                  hUsbPhy->RmPowerClientId,
                  NvRmVoltsOff, NvRmVoltsOff,
                  NULL, 0, NULL));
            hUsbPhy->IsPhyPoweredUp = NV_FALSE;
        }
        // Unregister driver from Power Manager
        NvRmPowerUnRegister(hUsbPhy->hRmDevice, hUsbPhy->RmPowerClientId);
        NvOsSemaphoreDestroy(hUsbPhy->hPwrEventSem);
    }
    NvOsMutexUnlock(hUsbPhy->ThreadSafetyMutex);

    NvOsMutexDestroy(hUsbPhy->ThreadSafetyMutex);

    if (hUsbPhy->CloseHwInterface)
    {
        hUsbPhy->CloseHwInterface(hUsbPhy);
    }

    if ((hUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host) ||
        (hUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_OTG))
    {
        UsbPrivEnableVbus(hUsbPhy, NV_FALSE);
    }

    NvOdmEnableUsbPhyPowerRail(NV_FALSE);

    if (hUsbPhy->hGpio)
    {
        if (hUsbPhy->hPin)
        {
            NvOdmGpioReleasePinHandle(hUsbPhy->hGpio, hUsbPhy->hPin);
            hUsbPhy->hPin = NULL;
        }
        NvOdmGpioClose(hUsbPhy->hGpio);
        hUsbPhy->hGpio = NULL;
    }

    NvRmPhysicalMemUnmap(
        (void*)hUsbPhy->UsbVirAdr, hUsbPhy->UsbBankSize);

    NvRmPhysicalMemUnmap(
        (void*)hUsbPhy->MiscVirAdr, hUsbPhy->MiscBankSize);

    NvOsMemset(hUsbPhy, 0, sizeof(NvDdkUsbPhy));
    NvOsMutexUnlock(s_UsbPhyMutex);
}


NvError
NvDdkUsbPhyPowerUp(
    NvDdkUsbPhyHandle hUsbPhy,
    NvBool IsHostMode,
    NvBool IsDpd)
{
    NvError e = NvSuccess;

    NV_ASSERT(hUsbPhy);

    NvOsMutexLock(hUsbPhy->ThreadSafetyMutex);
    if (hUsbPhy->IsPhyPoweredUp)
    {
        NvOsMutexUnlock(hUsbPhy->ThreadSafetyMutex);
        return e;
    }

    if (hUsbPhy->TurnOffPowerRail)
    {
        NvOdmEnableUsbPhyPowerRail(NV_TRUE);
    }

    // Enable power for USB module
    NV_CHECK_ERROR_CLEANUP(
        NvRmPowerVoltageControl(hUsbPhy->hRmDevice,
          NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
          hUsbPhy->RmPowerClientId, NvRmVoltsUnspecified,
          NvRmVoltsUnspecified, NULL, 0, NULL));

    // On Ap20 We will not turn off the H-Clk so not required to turn on
    if (!hUsbPhy->Caps.PhyRegInController)
    {
        NV_CHECK_ERROR_CLEANUP(
            NvRmPowerModuleClockControl(hUsbPhy->hRmDevice,
              NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
              hUsbPhy->RmPowerClientId, NV_TRUE));
    }

    // Power up the Phy
    NV_CHECK_ERROR_CLEANUP(hUsbPhy->PowerUp(hUsbPhy));

    /* Allow restoring register context for the USB host if it is a ULPI
       interface or if the lowest power state is LP1 */
    if (hUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host)
    {
        hUsbPhy->RestoreContext(hUsbPhy);
    }

    hUsbPhy->IsHostMode = IsHostMode;
    if (IsHostMode)
    {
        UsbPrivEnableVbus(hUsbPhy, NV_TRUE);
    }
    else
    {
        /* Turn on the USB busy hints */
        UsbPhyDfsBusyHint(hUsbPhy, NV_TRUE, NV_WAIT_INFINITE);
    }
    hUsbPhy->IsPhyPoweredUp = NV_TRUE;


fail:

    NvOsMutexUnlock(hUsbPhy->ThreadSafetyMutex);
    return e;
}


NvError
NvDdkUsbPhyPowerDown(
    NvDdkUsbPhyHandle hUsbPhy,
    NvBool IsHostMode,
    NvBool IsDpd)
{
    NvError e = NvSuccess;
    NvDdkUsbPhyIoctl_VBusStatusOutputArgs VBusStatus;
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;

    NV_ASSERT(hUsbPhy);

    NvOsMutexLock(hUsbPhy->ThreadSafetyMutex);
    if (!hUsbPhy->IsPhyPoweredUp)
    {
        NvOsMutexUnlock(hUsbPhy->ThreadSafetyMutex);
        return e;
    }

    /* Allow saving register context for the USB host if it is a ULPI
       interface or if the lowest power state is LP1 */
    if (hUsbPhy->pProperty->UsbMode == NvOdmUsbModeType_Host)
    {
        hUsbPhy->SaveContext(hUsbPhy);
    }

    /* Turn on/off the vbus for host mode */
    hUsbPhy->IsHostMode = IsHostMode;
    if (IsHostMode)
    {
        UsbPrivEnableVbus(hUsbPhy, NV_FALSE);
        /* Wait till Vbus is turned off */
        do
        {
            NvOsWaitUS(1000);
            TimeOut -= 1000;
            e = hUsbPhy->Ioctl(hUsbPhy,
                    NvDdkUsbPhyIoctlType_VBusStatus,
                    NULL,
                    &VBusStatus);
        } while (VBusStatus.VBusDetected && TimeOut);
    }
    // Power down the USB Phy
    NV_CHECK_ERROR_CLEANUP(hUsbPhy->PowerDown(hUsbPhy));

    // On AP20 H-CLK should not be turned off
    // This is required to detect the sensor interrupts.
    // However, phy can be programmed to put in the low power mode
    if (!hUsbPhy->Caps.PhyRegInController)
    {
        // Disable the clock
        NV_CHECK_ERROR_CLEANUP(
            NvRmPowerModuleClockControl(hUsbPhy->hRmDevice,
              NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
              hUsbPhy->RmPowerClientId, NV_FALSE));
    }

    // Disable power
    NV_CHECK_ERROR_CLEANUP(
        NvRmPowerVoltageControl(hUsbPhy->hRmDevice,
          NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, hUsbPhy->Instance),
          hUsbPhy->RmPowerClientId, NvRmVoltsOff, NvRmVoltsOff,
          NULL, 0, NULL));

    /* Turn off the USB busy hints */
    UsbPhyDfsBusyHint(hUsbPhy, NV_FALSE, NV_WAIT_INFINITE);

    if (hUsbPhy->TurnOffPowerRail)
    {
         NvOdmEnableUsbPhyPowerRail(NV_FALSE);
         NvOdmEnableOtgCircuitry(NV_FALSE);
    }

    hUsbPhy->IsPhyPoweredUp = NV_FALSE;

fail:

    NvOsMutexUnlock(hUsbPhy->ThreadSafetyMutex);
    return e;
}


NvError
NvDdkUsbPhyWaitForStableClock(
    NvDdkUsbPhyHandle hUsbPhy)
{
    NV_ASSERT(hUsbPhy);

    return hUsbPhy->WaitForStableClock(hUsbPhy);
}

void NvDdkUsbPhyMemoryPrefetch(NvDdkUsbPhyHandle hUsbPhy, NvBool Enable)
{
    NV_ASSERT(hUsbPhy);

    hUsbPhy->MemoryPrefetch(hUsbPhy, Enable);
}

NvError
NvDdkUsbPhyIoctl(
    NvDdkUsbPhyHandle hUsbPhy,
    NvDdkUsbPhyIoctlType IoctlType,
    const void *InputArgs,
    void *OutputArgs)
{
    NvError ErrStatus = NvSuccess;

    NV_ASSERT(hUsbPhy);

    switch(IoctlType)
    {
        case NvDdkUsbPhyIoctlType_UsbBusyHintsOnOff:
            ErrStatus= UsbPhyIoctlUsbBisuHintsOnOff(hUsbPhy,InputArgs);
            break;

        default:
            ErrStatus =  hUsbPhy->Ioctl(hUsbPhy, IoctlType, InputArgs, OutputArgs);
            break;
    }
    return ErrStatus;
}
