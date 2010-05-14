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

/** @file
 * @brief <b>NVIDIA Driver Development Kit: OWR API</b>
 *
 * @b Description: Contains the NvRM OWR implementation.
 */

#include "nvrm_owr.h"
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
#include "nvrm_owr_private.h"

// Mask to get the instance from the OWR handle
// LSB byte of the OWR handle stores the OWR instance.
#define OWR_HANDLE_INSTANCE_MASK   0xFF

// MSB bit of the OWR handle. MSB bit of the OWR 
// handle is always set to 1 to make sure OWR handle is never NULL.
#define OWR_HANDLE_MSB_BIT  0x80000000

/* Array of controllers */
static NvRmOwrController gs_OwrControllers[MAX_OWR_INSTANCES];
static NvRmOwrController *gs_OwrCont = NULL;

// Maximum OWR Instances present in this SOC
static NvU32 MaxOwrInstances;


static void
PrivOwrConfigurePower(
    NvRmOwrController *pOwrInfo,
    NvBool IsEnablePower);

static NvError
PrivOwrGetCaps(
    NvRmDeviceHandle hDevice,
    NvU32 Instance,
    NvRmOwrCapability** pOwrSocCaps);


static void
PrivOwrConfigurePower(
    NvRmOwrController *pOwrInfo,
    NvBool IsEnablePower)
{
    NvRmModuleID ModuleId =
            NVRM_MODULE_ID(pOwrInfo->ModuleId, pOwrInfo->Instance);

    if (IsEnablePower == NV_TRUE)
    {
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(
                pOwrInfo->hRmDevice, 
                ModuleId, 
                pOwrInfo->OwrPowerClientId, 
                NvRmVoltsUnspecified, 
                NvRmVoltsUnspecified, 
                NULL, 
                0, 
                NULL));

            // Enable the clock to the OWR controller
            NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(pOwrInfo->hRmDevice,
                        ModuleId,
                        pOwrInfo->OwrPowerClientId, 
                        NV_TRUE));
    }
    else
    {
        // Disable the clock to the OWR controller
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(pOwrInfo->hRmDevice, 
                    ModuleId, 
                    pOwrInfo->OwrPowerClientId, 
                    NV_FALSE));
        
        //disable power
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(pOwrInfo->hRmDevice,
                ModuleId,
                pOwrInfo->OwrPowerClientId,
                NvRmVoltsOff,
                NvRmVoltsOff,
                NULL,
                0,
                NULL));
    }
}

static NvError
PrivOwrGetCaps(
    NvRmDeviceHandle hDevice,
    NvU32 Instance,
    NvRmOwrCapability** pOwrSocCaps)
{
    static NvRmOwrCapability s_OwrCap0;
    NvError status = NvSuccess;
    NvRmOwrCapability *pOwrCaps = NULL;
    static NvRmModuleCapability s_OwrCaps[] = 
        { {1, 0, 0, &s_OwrCap0 }
        };
    
    s_OwrCap0.NoOfInstances = 1;
    // Get the capability from modules files.
    status = NvRmModuleGetCapabilities(hDevice, 
                    NVRM_MODULE_ID(NvRmModuleID_OneWire, Instance),
                    s_OwrCaps, 
                    NV_ARRAY_SIZE(s_OwrCaps), (void **)&pOwrCaps);
    if (status == NvSuccess)
    {
        *pOwrSocCaps = pOwrCaps;
    }
    return status;
}

NvError
NvRmOwrOpen(
    NvRmDeviceHandle hRmDevice,
    NvU32 Instance,
    NvRmOwrHandle *phOwr)
{
    NvError status = NvSuccess;
    NvRmOwrController *pOwrInfo;
    NvU32 PrefClockFreq = MAX_OWR_CLOCK_SPEED_KHZ;
    NvRmModuleID ModuleId;
    NvOdmIoModule IoModule = NvOdmIoModule_OneWire;
    NvRmOwrCapability *OwrSocCaps = NULL;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(phOwr);

    /** If none of the controller is opened, allocate memory for all controllers
     * in the system 
     */
    if (gs_OwrCont == NULL)
    {
        gs_OwrCont = gs_OwrControllers;
    }
    
    /* Validate the Instance number passed */
    if (IoModule == NvOdmIoModule_OneWire)
    {
        MaxOwrInstances =
                NvRmModuleGetNumInstances(hRmDevice, NvRmModuleID_OneWire);
        NV_ASSERT(Instance < MaxOwrInstances);
        ModuleId = NvRmModuleID_OneWire;
    }
    else
    {
        NV_ASSERT(!"Invalid IO module");
    }

    status = PrivOwrGetCaps(hRmDevice, Instance, &OwrSocCaps);
    if (status != NvSuccess)
    {
        return NvError_NotInitialized;
    }
    
    pOwrInfo = &(gs_OwrCont[Instance]);

    // Create the mutex for providing the thread safety for OWR API
    if (pOwrInfo->NumberOfClientsOpened == 0)
    {
        status = NvOsMutexCreate(&(pOwrInfo->OwrThreadSafetyMutex));
        if (status != NvSuccess)
        {
            pOwrInfo->OwrThreadSafetyMutex = NULL;
            return status;
        }
    }

    NvOsMutexLock(pOwrInfo->OwrThreadSafetyMutex);
    // If no clients are opened yet, initialize the OWR controller
    if (pOwrInfo->NumberOfClientsOpened == 0)
    {
        /* Polulate the controller structure */
        pOwrInfo->hRmDevice = hRmDevice;
        pOwrInfo->ModuleId = ModuleId;
        pOwrInfo->Instance = Instance;
        pOwrInfo->OwrPowerClientId = 0;

        NV_ASSERT_SUCCESS(AP20RmOwrOpen(pOwrInfo));

        /* Make sure that all the functions are populated by the HAL driver */
        NV_ASSERT(pOwrInfo->read && pOwrInfo->write && pOwrInfo->close);

        status = NvRmSetModuleTristate(pOwrInfo->hRmDevice, 
            NVRM_MODULE_ID(pOwrInfo->ModuleId, pOwrInfo->Instance), NV_FALSE);
        if (status != NvSuccess)
        {
            goto fail;
        }

        pOwrInfo->OwrPowerClientId = NVRM_POWER_CLIENT_TAG('O','W','R',' ');
        status = 
            NvRmPowerRegister(hRmDevice, NULL, &pOwrInfo->OwrPowerClientId);
        if (status != NvSuccess)
        {
            goto fail;
        }

        /** Enable power rail, enable clock, configure clock to right freq,
         * reset, disable clock, notify to disable power rail. 
         *
         *  All of this is done to just reset the controller.
         */
        PrivOwrConfigurePower(pOwrInfo, NV_TRUE);
        status = NvRmPowerModuleClockConfig(hRmDevice, 
                    NVRM_MODULE_ID(ModuleId, Instance), 
                    pOwrInfo->OwrPowerClientId,
                    PrefClockFreq, 
                    NvRmFreqUnspecified, 
                    &PrefClockFreq, 
                    1, 
                    NULL, 
                    0);
        if (status != NvSuccess)
        {
            PrivOwrConfigurePower(pOwrInfo, NV_FALSE);
            goto fail;
        }
        NvRmModuleReset(hRmDevice, NVRM_MODULE_ID(ModuleId, Instance));
        PrivOwrConfigurePower(pOwrInfo, NV_FALSE);
    }
    pOwrInfo->NumberOfClientsOpened++;
    NvOsMutexUnlock(pOwrInfo->OwrThreadSafetyMutex);

    /** We cannot return handle with a value of 0, as some clients check the
     * handle to ne non-zero. So, to get around that we set MSB bit to 1.
     */
    *phOwr = (NvRmOwrHandle)(Instance | OWR_HANDLE_MSB_BIT);
    return NvSuccess;

fail:
    if (pOwrInfo->OwrPowerClientId)
    {
        NvRmPowerUnRegister(hRmDevice, pOwrInfo->OwrPowerClientId);
        pOwrInfo->OwrPowerClientId = 0;
    }

    (pOwrInfo->close)(pOwrInfo);
    *phOwr = 0;

    NvOsMutexUnlock(pOwrInfo->OwrThreadSafetyMutex);
    NvOsMutexDestroy(pOwrInfo->OwrThreadSafetyMutex);

    return status;
}

void NvRmOwrClose(NvRmOwrHandle hOwr)
{
    NvU32 Index;
    NvRmOwrController *pOwrInfo;

    if (hOwr == NULL)
        return;

    Index = ((NvU32) hOwr) & OWR_HANDLE_INSTANCE_MASK;
    if (Index < MaxOwrInstances)
    {
        pOwrInfo = &(gs_OwrCont[Index]);

        NvOsMutexLock(pOwrInfo->OwrThreadSafetyMutex);
        pOwrInfo->NumberOfClientsOpened--;
        if (pOwrInfo->NumberOfClientsOpened == 0)
        {
            /* Unregister the power client ID */
            NvRmPowerUnRegister(pOwrInfo->hRmDevice,
                                    pOwrInfo->OwrPowerClientId);
            pOwrInfo->OwrPowerClientId = 0;

            NV_ASSERT_SUCCESS( NvRmSetModuleTristate(pOwrInfo->hRmDevice,
            NVRM_MODULE_ID(pOwrInfo->ModuleId, pOwrInfo->Instance), NV_TRUE ));

            NV_ASSERT(pOwrInfo->close);
            (pOwrInfo->close)(pOwrInfo);

           /** FIXME: There is a race here. After the Mutex is unlocked someone
            * can call NvRmOwrOpen and create the mutex, which will then be
            * destroyed here.
            */
            NvOsMutexUnlock(pOwrInfo->OwrThreadSafetyMutex);
            NvOsMutexDestroy(pOwrInfo->OwrThreadSafetyMutex);
            pOwrInfo->OwrThreadSafetyMutex = NULL;
        }
        else 
        {
            NvOsMutexUnlock(pOwrInfo->OwrThreadSafetyMutex);
        }
    }
}

NvError NvRmOwrTransaction( 
    NvRmOwrHandle hOwr,
    NvU32 OwrPinMap,
    NvU8 *Data,
    NvU32 DataLength,
    NvRmOwrTransactionInfo * Transaction,
    NvU32 NumOfTransactions)
{
    NvU32 i;
    NvRmOwrController* pOwrInfo;
    NvU32 Index;
    NvError status = NvSuccess;

    Index = ((NvU32)hOwr) & OWR_HANDLE_INSTANCE_MASK;

    NV_ASSERT(Index < MaxOwrInstances);
    NV_ASSERT(Transaction);
    NV_ASSERT(Data);

    pOwrInfo = &(gs_OwrCont[Index]);

    NvOsMutexLock(pOwrInfo->OwrThreadSafetyMutex);

    PrivOwrConfigurePower(pOwrInfo, NV_TRUE);

    for (i = 0; i < NumOfTransactions; i++)
    {
        if ((Transaction[i].Flags == NvRmOwr_MemWrite) ||
            (Transaction[i].Flags == NvRmOwr_WriteByte) ||
            (Transaction[i].Flags == NvRmOwr_WriteBit))
        {
            // OWR write transaction
            status = (pOwrInfo->write)(
                    pOwrInfo,
                    Data,
                    Transaction[i]);
        }
        else
        {
            // OWR read transaction
            status = (pOwrInfo->read)(
                    pOwrInfo,
                    Data,
                    Transaction[i]);
        }
        Data += Transaction[i].NumBytes;
        if (status != NvSuccess)
        {
            break;
        }
    }

    PrivOwrConfigurePower(pOwrInfo, NV_FALSE);

    NvOsMutexUnlock(pOwrInfo->OwrThreadSafetyMutex);
    return status;
}

