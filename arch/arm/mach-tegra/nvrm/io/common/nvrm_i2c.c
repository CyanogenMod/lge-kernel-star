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


/** @file
 * @brief <b>NVIDIA Driver Development Kit: I2C API</b>
 *
 * @b Description: Contains the NvRM I2C implementation.
 */

#include "nvrm_i2c.h"
#include "nvrm_i2c_private.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "nvrm_hardware_access.h"
#include "nvrm_power.h"
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pinmux.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"
#include "nvodm_modules.h"
#include "nvrm_structure.h"
#include "nvrm_pinmux_utils.h"

/* Array of controllers */
static NvRmI2cController gs_I2cControllers[MAX_I2C_INSTANCES];
static NvRmI2cController *gs_Cont = NULL;

// Maximum I2C instances present in this SOC
static NvU32 MaxI2cControllers;
static NvU32 MaxDvcControllers;
static NvU32 MaxI2cInstances;

static NvError PrivI2cSetSpeed(NvRmI2cController *c);
static NvError PrivI2cConfigurePower(NvRmI2cController *c, NvBool IsEnablePower);

/**
 * Get the I2C  SOC capability.
 *
 */
static void
I2cGetSocCapabilities(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvU32 Instance,
    SocI2cCapability *pI2cSocCaps)
{
    static SocI2cCapability s_SocI2cCapsList[2];
    NvRmModuleCapability I2cCapsList[3];
    SocI2cCapability *pI2cCaps = NULL;

    if (ModuleId == NvRmModuleID_I2c)
    {
        I2cCapsList[0].MajorVersion = 1;
        I2cCapsList[0].MinorVersion = 0;
        I2cCapsList[0].EcoLevel = 0;
        I2cCapsList[0].Capability = &s_SocI2cCapsList[0];

        I2cCapsList[1].MajorVersion = 1;
        I2cCapsList[1].MinorVersion = 1;
        I2cCapsList[1].EcoLevel = 0;
        I2cCapsList[1].Capability = &s_SocI2cCapsList[0];

        //AP15 A01P and A02 does not support packet interface
        s_SocI2cCapsList[0].IsNewMasterAvailable = NV_FALSE;

        I2cCapsList[2].MajorVersion = 1;
        I2cCapsList[2].MinorVersion = 2;
        I2cCapsList[2].EcoLevel = 0;
        I2cCapsList[2].Capability = &s_SocI2cCapsList[1];

        // AP20 supports Packet based interface with new master enable
        s_SocI2cCapsList[1].IsNewMasterAvailable= NV_TRUE;

        // Get the capability from modules files.
        NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hDevice,
                    NVRM_MODULE_ID(ModuleId, Instance), I2cCapsList,
                    NV_ARRAY_SIZE(I2cCapsList), (void **)&pI2cCaps));
    }
    else if (ModuleId == NvRmModuleID_Dvc)
    {
        I2cCapsList[0].MajorVersion = 1;
        I2cCapsList[0].MinorVersion = 0;
        I2cCapsList[0].EcoLevel = 0;
        I2cCapsList[0].Capability = &s_SocI2cCapsList[0];

        // AP15 does not support new master interface
        s_SocI2cCapsList[0].IsNewMasterAvailable= NV_FALSE;

        I2cCapsList[1].MajorVersion = 1;
        I2cCapsList[1].MinorVersion = 1;
        I2cCapsList[1].EcoLevel = 0;
        I2cCapsList[1].Capability = &s_SocI2cCapsList[1];

        // AP20 supports new master interface ans capable of doing the
        // packed mode transfer.
        s_SocI2cCapsList[1].IsNewMasterAvailable= NV_TRUE;

        // Get the capability from modules files.
        NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hDevice,
                    NVRM_MODULE_ID(ModuleId, Instance), I2cCapsList,
                    NV_ARRAY_SIZE(I2cCapsList), (void **)&pI2cCaps));
    }
    if (pI2cCaps)
        pI2cSocCaps->IsNewMasterAvailable= pI2cCaps->IsNewMasterAvailable;
    else
        NV_ASSERT(!"Invalid ModuleId is passed to I2cGetSocCapabilities() ");
}

NvError
NvRmI2cOpen(
    NvRmDeviceHandle hRmDevice,
    NvU32 IoModule,
    NvU32 instance,
    NvRmI2cHandle *phI2c)
{
    NvError status = NvSuccess;
    NvU32 PrefClockFreq = MAX_I2C_CLOCK_SPEED_KHZ;
    NvU32 Index = instance;
    NvRmModuleID ModuleID = NvRmModuleID_I2c;
    NvRmI2cController *c;
    NvOsMutexHandle hThreadSaftyMutex = NULL;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;


    NV_ASSERT(hRmDevice);
    NV_ASSERT(phI2c);
    NV_ASSERT((IoModule == NvOdmIoModule_I2c) || (IoModule == NvOdmIoModule_I2c_Pmu));

    *phI2c = 0;
    /* If none of the controller is opened, allocate memory for all controllers
    * in the system */
    if (gs_Cont == NULL)
    {
        gs_Cont = gs_I2cControllers;
        MaxI2cControllers = NvRmModuleGetNumInstances(hRmDevice, NvRmModuleID_I2c);
        MaxDvcControllers = NvRmModuleGetNumInstances(hRmDevice, NvRmModuleID_Dvc);
        MaxI2cInstances = MaxI2cControllers + MaxDvcControllers;
    }
    /* Validate the instance number passed and return the Index of the
     * controller to the caller.
     *
     */
    if (IoModule == NvOdmIoModule_I2c)
    {
        NV_ASSERT(instance < MaxI2cControllers);
        ModuleID = NvRmModuleID_I2c;
        Index = instance;
    }
    else  if (IoModule == NvOdmIoModule_I2c_Pmu)
    {
        NV_ASSERT(instance < MaxDvcControllers);
        ModuleID = NvRmModuleID_Dvc;
        Index = MaxI2cControllers + instance;
    }
    else
    {
        NV_ASSERT(!"Invalid IO module");
        return NvError_NotSupported;
    }

    c = &(gs_Cont[Index]);

    // Create the mutex for providing the thread safety for i2c API
    if ((c->NumberOfClientsOpened == 0) && (c->I2cThreadSafetyMutex == NULL))
    {
        status = NvOsMutexCreate(&hThreadSaftyMutex);
        if (status)
            return status;

        if (NvOsAtomicCompareExchange32((NvS32*)&c->I2cThreadSafetyMutex, 0,
                                                    (NvS32)hThreadSaftyMutex)!=0)
        {
            NvOsMutexDestroy(hThreadSaftyMutex);
            hThreadSaftyMutex = NULL;
        }
    }

    NvOsMutexLock(c->I2cThreadSafetyMutex);
    // If no clients are opened yet, initialize the i2c controller
    if (c->NumberOfClientsOpened == 0)
    {
        NvU32 len;
        /* Polulate the controller structure */
        c->hRmDevice = hRmDevice;
        c->OdmIoModule = IoModule;
        c->ModuleId = ModuleID;
        c->Instance = instance;

        c->I2cPowerClientId = 0;
        c->receive  = NULL;
        c->send = NULL;
        c->close = NULL;
        c->GetGpioPins = NULL;
        c->hGpio = NULL;
        c->hSclPin = 0;
        c->hSdaPin = 0;

        NvRmModuleGetBaseAddress(hRmDevice, NVRM_MODULE_ID(ModuleID, instance),
            &c->ControllerAdd, &len);

        I2cGetSocCapabilities(hRmDevice, ModuleID, instance, &(c->SocI2cCaps));
        c->EnableNewMaster = c->SocI2cCaps.IsNewMasterAvailable;

        NvOdmQueryPinMux(IoModule, &pOdmConfigs, &NumOdmConfigs);
        NV_ASSERT((instance < NumOdmConfigs) && (pOdmConfigs[instance]));
        if ((instance >= NumOdmConfigs) || (!pOdmConfigs[instance]))
        {
            status = NvError_NotSupported;
            goto fail_1;
        }
        c->PinMapConfig = pOdmConfigs[instance];

        /* Call appropriate open function according to the controller
         * supports packet mode or not. If packet mode is supported
         * call AP20RmI2cOpen for packet mode funcitons. Other wise
         * use normal mode */
        if (c->SocI2cCaps.IsNewMasterAvailable)
            status = AP20RmI2cOpen(c);
        else
            status = AP15RmI2cOpen(c);

        if (status)
            goto fail_1;
        /* Make sure that all the functions are polulated by the HAL driver */
        NV_ASSERT(c->receive && c->send && c->close);

        status = NvRmSetModuleTristate(c->hRmDevice,
                        NVRM_MODULE_ID(c->ModuleId, c->Instance), NV_FALSE);
        if (status != NvSuccess)
        {
            goto fail_1;
        }

        /* Initalize the GPIO handles only */
        if (c->GetGpioPins)
        {
            status = NvRmGpioOpen(c->hRmDevice, &c->hGpio);
            if(status)
                goto fail_1;
        }

        c->I2cPowerClientId = NVRM_POWER_CLIENT_TAG('I','2','C',' ');
        status = NvRmPowerRegister(hRmDevice, NULL, &c->I2cPowerClientId);
        if (status != NvSuccess)
        {
            goto fail_2;
        }

        /* Enable power rail, enable clock, configure clock to right freq,
         * reset, disable clock, notify to disable power rail.
         *
         *  All of this is done to just reset the controller.
         * */
        PrivI2cConfigurePower(c, NV_TRUE);
        status = NvRmPowerModuleClockConfig(hRmDevice,
                NVRM_MODULE_ID(ModuleID, instance), c->I2cPowerClientId,
                PrefClockFreq, NvRmFreqUnspecified, &PrefClockFreq, 1, NULL, 0);
        if (status != NvSuccess)
        {
            goto fail_3;
        }
        NvRmModuleReset(hRmDevice, NVRM_MODULE_ID(ModuleID, instance));

        PrivI2cConfigurePower(c, NV_FALSE);
    }
    c->NumberOfClientsOpened++;
    NvOsMutexUnlock(c->I2cThreadSafetyMutex);

    /*
     * We cannot return handle with a value of 0, as some clients check the
     * handle to ne non-zero. So, to get around that we set MSB bit to 1.
     */
    *phI2c = (NvRmI2cHandle)(Index | 0x80000000);
    return NvSuccess;

fail_3:
    PrivI2cConfigurePower(c, NV_FALSE);

fail_2:
    NvRmPowerUnRegister(hRmDevice, c->I2cPowerClientId);
    c->I2cPowerClientId = 0;

fail_1:
    if (c->close)
        (c->close)(c);
    *phI2c = 0;
    NvRmGpioReleasePinHandles(c->hGpio, &c->hSclPin, 1);
    NvRmGpioReleasePinHandles(c->hGpio, &c->hSdaPin, 1);
    NvRmGpioClose(c->hGpio);
    NvOsMutexUnlock(c->I2cThreadSafetyMutex);
    NvOsMutexDestroy(c->I2cThreadSafetyMutex);
    NvOsMemset(c, 0, sizeof(*c));

    return status;
}

void NvRmI2cClose(NvRmI2cHandle hI2c)
{
    NvU32 Index;
    NvRmI2cController *c;

    if (hI2c == NULL)
        return;

    Index = ((NvU32) hI2c) & 0xFF;
    if (Index < MaxI2cInstances)
    {
        c = &(gs_Cont[Index]);

        NvOsMutexLock(c->I2cThreadSafetyMutex);
        c->NumberOfClientsOpened--;
        if (c->NumberOfClientsOpened == 0)
        {

            if(c->GetGpioPins)
            {
                if (c->hSclPin)
                    NvRmGpioReleasePinHandles(c->hGpio, &c->hSclPin, 1);
                if (c->hSdaPin)
                    NvRmGpioReleasePinHandles(c->hGpio, &c->hSdaPin, 1);
                c->hSdaPin = 0;
                c->hSclPin = 0;
            }
            NvRmGpioClose(c->hGpio);

            /* Unregister the power client ID */
            NvRmPowerUnRegister(c->hRmDevice, c->I2cPowerClientId);
            c->I2cPowerClientId = 0;

            NV_ASSERT_SUCCESS( NvRmSetModuleTristate(c->hRmDevice,
            NVRM_MODULE_ID(c->ModuleId, c->Instance), NV_TRUE ));

            NV_ASSERT(c->close);
            (c->close)(c);

            /* FIXME: There is a race here. After the Mutex is unlocked someone can
            * call NvRmI2cOpen and create the mutex, which will then destropyed
            * here?
            * */
            NvOsMutexUnlock(c->I2cThreadSafetyMutex);
            NvOsMutexDestroy(c->I2cThreadSafetyMutex);
            c->I2cThreadSafetyMutex = NULL;
        }
        else
        {
            NvOsMutexUnlock(c->I2cThreadSafetyMutex);
        }
    }
}

static NvError PrivI2cSetSpeed(NvRmI2cController *c)
{
    NvError status;
    NvRmModuleID ModuleId = NVRM_MODULE_ID(c->ModuleId, c->Instance);

    // It seems like the I2C Controller has an hidden clock divider whose value
    // is 8. So, request for clock value multipled by 8.
    NvU32 PrefClockFreq = c->clockfreq * 8;

    status = NvRmPowerModuleClockConfig(
                c->hRmDevice,
                ModuleId,
                c->I2cPowerClientId,
                NvRmFreqUnspecified,
                PrefClockFreq,
                &PrefClockFreq,
                1,
                NULL,
                0);
    return status;
}

static NvError PrivI2cConfigurePower(NvRmI2cController *c, NvBool IsEnablePower)
{
    NvError status = NvSuccess;
    NvRmModuleID ModuleId = NVRM_MODULE_ID(c->ModuleId, c->Instance);

    if (IsEnablePower == NV_TRUE)
    {
#if !NV_OAL
        status = NvRmPowerVoltageControl(
                c->hRmDevice,
                ModuleId,
                c->I2cPowerClientId,
                NvRmVoltsUnspecified,
                NvRmVoltsUnspecified,
                NULL,
                0,
                NULL);
#endif
        if(status == NvSuccess)
        {
            // Enable the clock to the i2c controller
            NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(c->hRmDevice,
                        ModuleId,
                        c->I2cPowerClientId,
                        NV_TRUE));
        }
    }
    else
    {
       // Disable the clock to the i2c controller
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(c->hRmDevice,
                    ModuleId,
                    c->I2cPowerClientId,
                    NV_FALSE));

#if !NV_OAL
        //disable power
        status = NvRmPowerVoltageControl(c->hRmDevice,
                ModuleId,
                c->I2cPowerClientId,
                NvRmVoltsOff,
                NvRmVoltsOff,
                NULL,
                0,
                NULL);
#endif
    }
    return status;
}

NvError NvRmI2cTransaction(
    NvRmI2cHandle hI2c,
    NvU32 I2cPinMap,
    NvU32 WaitTimeoutInMilliSeconds,
    NvU32 ClockSpeedKHz,
    NvU8 *Data,
    NvU32 DataLength,
    NvRmI2cTransactionInfo * Transaction,
    NvU32 NumOfTransactions)
{
    NvU32 len = 0;
    NvError status;
    NvU32 i;
    NvU32 BytesTransferred = 0;
    NvBool useGpioI2c = NV_FALSE;
    NvRmI2cController* c;
    NvU32 Index;
    NvU32 RSCount  = 0; // repeat start count
    NvS32 StartTransIndex = -1;
    NvU32 scl, sda;

    Index = ((NvU32)hI2c) & 0x7FFFFFFF;

    NV_ASSERT(((NvU32)hI2c) & 0x80000000);
    NV_ASSERT(Index < MaxI2cInstances);
    NV_ASSERT(Transaction);
    NV_ASSERT(Data);
    NV_ASSERT(ClockSpeedKHz <= MAX_I2C_CLOCK_SPEED_KHZ);

    c = &(gs_Cont[Index]);
    if (c->SocI2cCaps.IsNewMasterAvailable == NV_FALSE)
    {
        c->timeout = WaitTimeoutInMilliSeconds;
    }
    else
    {
        c->timeout = 1000;
    }
    c->clockfreq = ClockSpeedKHz;

    NV_ASSERT(((c->PinMapConfig == NvOdmI2cPinMap_Multiplexed) && (I2cPinMap)) ||
              ((c->PinMapConfig != NvOdmI2cPinMap_Multiplexed) && (!I2cPinMap)));

    if (NvRmIsSimulation())
        return NvError_NotSupported;

    NvOsMutexLock(c->I2cThreadSafetyMutex);

    // If I2C does not support pkt format use narmal mode to transfer the data
    if (c->SocI2cCaps.IsNewMasterAvailable == NV_FALSE)
    {
        /* Do all the transactions using software GPIO, if one of the transactions
        * failed to satisfy the hardware requirements. */
        for (i=0; i< NumOfTransactions; i++)
        {
            if (Transaction[i].Flags & NVRM_I2C_NOSTOP)
            {
                if ((i+1) >= NumOfTransactions)
                {
                    useGpioI2c = NV_TRUE;
                    break;
                }
                else
                {
                    if ((Transaction[i].NumBytes > NVRM_I2C_PACKETSIZE_WITH_NOSTOP) ||
                            (Transaction[i].NumBytes != Transaction[i+1].NumBytes))
                    {
                        useGpioI2c = NV_TRUE;
                        break;
                    }
                }
            }
            else
            {
                if (Transaction[i].NumBytes > NVRM_I2C_PACKETSIZE)
                {
                    useGpioI2c = NV_TRUE;
                    break;
                }
            }
        }
    }
    if ((Transaction[0].Flags & NVRM_I2C_SOFTWARE_CONTROLLER) ||
                            (useGpioI2c == NV_TRUE))
    {
        if (c->hGpio == NULL)
        {
            status = NvRmGpioOpen(c->hRmDevice, &c->hGpio);
            if(status)
            {
                 NvOsMutexUnlock(c->I2cThreadSafetyMutex);
                 return status;
            }
        }
        /* Initalize the GPIO pin handles if it is not done */
        if (c->GetGpioPins)
        {
            if (c->PinMapConfig != NvOdmI2cPinMap_Multiplexed)
	    {
	         if ((c->hSclPin == 0) || (c->hSdaPin == 0))
                 {
                      if ((c->GetGpioPins)(c, c->PinMapConfig, &scl, &sda))
                      {
                           status = NvRmGpioAcquirePinHandle(c->hGpio,
                                          (scl >> 16), (scl & 0xFFFF),
                                           &c->hSclPin);
                            if(!status)
                            {
                                 status = NvRmGpioAcquirePinHandle(c->hGpio,
                                           (sda >> 16), (sda & 0xFFFF),
                                           &c->hSdaPin);
                                 if(status)
                                 {
                                      NvRmGpioReleasePinHandles(c->hGpio,
                                           &c->hSclPin, 1);
                                      c->hSclPin = 0;
                                 }
                            }
                       }
                  }
            }
        }
        else
        {
            status = NvError_NotSupported;
        }

        if (status == NvSuccess)
            status = NvRmGpioI2cTransaction(c, I2cPinMap, Data, DataLength,
                                        Transaction, NumOfTransactions);
        NvOsMutexUnlock(c->I2cThreadSafetyMutex);
        return status;
    }

    if (I2cPinMap)
    {
        NvRmPinMuxConfigSelect(c->hRmDevice, c->OdmIoModule,
            c->Instance, I2cPinMap);

        NvRmPinMuxConfigSetTristate(c->hRmDevice, c->OdmIoModule,
            c->Instance, I2cPinMap, NV_FALSE);
    }


    status = PrivI2cConfigurePower(c, NV_TRUE);
    if (status != NvSuccess)
        goto TransactionExit;

    status = PrivI2cSetSpeed(c);
    if (status != NvSuccess)
        goto TransactionExit;

    len = 0;
    StartTransIndex = -1;
    for (i = 0; i < NumOfTransactions; i++)
    {
        c->Is10BitAddress = Transaction[i].Is10BitAddress;
        c->NoACK = NV_FALSE;
        if (Transaction[i].Flags & NVRM_I2C_NOACK)
        {
            c->NoACK = NV_TRUE;
        }
        // Check  wheather this transation is repeat start or not
        if (!(Transaction[i].Flags & NVRM_I2C_NOSTOP) && (!RSCount))
        {
            if (Transaction[i].Flags & NVRM_I2C_WRITE)
            {
                // i2c send transaction
                status = (c->send)(
                        c,
                        Data,
                        &Transaction[i],
                        &BytesTransferred);
            }
            else
            {
                // i2c receive transaction
                status = (c->receive)(
                        c,
                        Data,
                        &Transaction[i],
                        &BytesTransferred);
            }
            Data += Transaction[i].NumBytes;
        }
        else
        {
            RSCount++;
            // If transation is repeat start,
            if (Transaction[i].Flags & NVRM_I2C_NOSTOP)
            {
                len += Transaction[i].NumBytes;
                if (StartTransIndex == -1)
                    StartTransIndex = i;
            }
            else
            {
                // i2c transaction with repeat-start
                status = (c->repeatStart)(c, Data, &(Transaction[StartTransIndex]), RSCount);
                Data += len + Transaction[i].NumBytes;
                RSCount = 0;
                len = 0;
                StartTransIndex = -1;
            }
        }
        if (status != NvSuccess)
        {
            break;
        }
    }
TransactionExit:
    PrivI2cConfigurePower(c, NV_FALSE);

    //  Re-tristate multi-plexed controllers, and re-multiplex the controller.
    if (I2cPinMap)
    {
        NvRmPinMuxConfigSetTristate(c->hRmDevice, c->OdmIoModule,
            c->Instance, I2cPinMap, NV_TRUE);

        NvRmPinMuxConfigSelect(c->hRmDevice, c->OdmIoModule,
            c->Instance, c->PinMapConfig);
    }

    NvOsMutexUnlock(c->I2cThreadSafetyMutex);
    return status;
}
