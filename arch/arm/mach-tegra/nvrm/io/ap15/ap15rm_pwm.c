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
 * @brief <b>NVIDIA Driver Development Kit: PWM API</b>
 *
 * @b Description: Contains the NvRM PWM implementation.
 */

#include "ap15rm_pwm_private.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "nvrm_hardware_access.h"
#include "nvrm_power.h" 
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_modules.h"
#include "nvrm_pinmux.h"
#include "nvrm_hwintf.h"
#include "ap15/arpwfm.h"
#include "ap15/arapbpm.h"

#define PWM_REGR( VirtualAddress, offset )  \
        NV_READ32(VirtualAddress + offset)  

#define PWM_REGW( VirtualAddress, offset, value ) \
        NV_WRITE32(VirtualAddress + offset, value)

#define PMC_REGR( VirtualAddress, offset )  \
        NV_READ32(VirtualAddress + offset)  

#define PMC_REGW( VirtualAddress, offset, value ) \
        NV_WRITE32(VirtualAddress + offset, value)
    
static NvU32 s_PwmPowerID = 0;
static NvOsMutexHandle s_hPwmMutex = NULL;
static NvRmPwmHandle s_hPwm = NULL;
static NvBool s_IsPwmFirstConfig = NV_FALSE;
static NvBool s_IsFreqDividerSupported = NV_FALSE;

// Checks whether all the PWM channels are disabled or not.
// Returns NV_FALSE if any of the channels are enabled
// else returns NV_TRUE
static NvBool IsPwmDisabled(NvRmPwmHandle hPwm);

static NvBool IsPwmDisabled(NvRmPwmHandle hPwm)
{
    NvU32 RegValue = 0;
    NvU32 i = 0;
    NvBool PwmDisabled = NV_TRUE;

    for (i = 0; i < NvRmPwmOutputId_Num-2; i++)
    {
        RegValue = PWM_REGR( hPwm->VirtualAddress[i], 0 );
        if (PWM_GET(CSR_0, ENB, RegValue))
        {
            PwmDisabled = NV_FALSE;
            break;
        }
    }
    return PwmDisabled;
}

static NvError PwmPowerConfigure(NvRmPwmHandle hPwm, NvBool IsEnablePower)
{
    NvError status = NvSuccess;

    if (IsEnablePower == NV_TRUE)
    {
        if (!hPwm->PowerEnabled)
        {
            // Enable power
            status = NvRmPowerVoltageControl(hPwm->RmDeviceHandle,
                        NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                                            s_PwmPowerID,
                                            NvRmVoltsUnspecified,
                                            NvRmVoltsUnspecified,
                                            NULL,
                                            0,
                                            NULL);
            if (status == NvSuccess)
            {
                // Enable the clock to the pwm controller
                status = NvRmPowerModuleClockControl(hPwm->RmDeviceHandle,
                        NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                        s_PwmPowerID,
                        NV_TRUE);
                hPwm->PowerEnabled = NV_TRUE;
            }
        }
    }
    else
    {
        if (hPwm->PowerEnabled)
        {
            // Disable the clock to the pwm controller
            status = NvRmPowerModuleClockControl(hPwm->RmDeviceHandle,
                        NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                        s_PwmPowerID,
                        NV_FALSE);

            if(status == NvSuccess)
            {
                // Disable power
                status = NvRmPowerVoltageControl(hPwm->RmDeviceHandle,
                            NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                            s_PwmPowerID,
                            NvRmVoltsOff,
                            NvRmVoltsOff,
                            NULL,
                            0,
                            NULL);
                hPwm->PowerEnabled = NV_FALSE;
            }
        }
    }

    return status;
}


static NvError PwmCheckValidConfig(NvRmPwmHandle hPwm, 
        NvRmPwmOutputId OutputId,
        NvRmPwmMode Mode)
{
    NvError status = NvSuccess;
    NvRmModulePwmInterfaceCaps PwmCaps;

    if ((Mode != NvRmPwmMode_Disable) &&
        (Mode != NvRmPwmMode_Enable))
    return NvError_NotSupported;

    status = NvRmGetModuleInterfaceCapabilities(hPwm->RmDeviceHandle,
                                                 NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                                                 sizeof(NvRmModulePwmInterfaceCaps),
                                                 &PwmCaps);
    if (status != NvSuccess)
        return status;
    
    if (PwmCaps.PwmOutputIdSupported & (1 << (OutputId-1)))
        return status;
    else
        return NvError_NotSupported; 
}

NvError NvRmPrivPwmInit(NvRmDeviceHandle hRm);
NvError NvRmPrivPwmInit(NvRmDeviceHandle hRm)
{
    NvError status = NvSuccess;

    // Creating the Mutex
    status = NvOsMutexCreate(&s_hPwmMutex);
    return status;
}

void NvRmPrivPwmDeInit(NvRmDeviceHandle hRm);
void NvRmPrivPwmDeInit(NvRmDeviceHandle hRm)
{
    NvOsMutexDestroy(s_hPwmMutex);
}

NvError 
NvRmPwmOpen(
    NvRmDeviceHandle hDevice,
    NvRmPwmHandle *phPwm)
{
    NvError status = NvSuccess;
    NvU32 PwmPhysAddr = 0, i = 0, PmcPhysAddr = 0;
    NvRmModuleCapability caps[4];  
    NvRmModuleCapability *pCap = NULL;

    NV_ASSERT(hDevice);
    NV_ASSERT(phPwm);
    
    NvOsMutexLock(s_hPwmMutex);
    
    if (s_hPwm)
    {
        s_hPwm->RefCount++;
        goto exit;
    }

    // Allcoate the memory for the pwm handle
    s_hPwm = NvOsAlloc(sizeof(NvRmPwm));
    if (!s_hPwm)
    {
        status = NvError_InsufficientMemory;
        goto fail;
    }
    NvOsMemset(s_hPwm, 0, sizeof(NvRmPwm));

    // Set the pwm handle parameters
    s_hPwm->RmDeviceHandle = hDevice;
    
    // Get the pwm physical and virtual base address
    NvRmModuleGetBaseAddress(hDevice,
            NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
            &PwmPhysAddr, &(s_hPwm->PwmBankSize));
    s_hPwm->PwmBankSize = PWM_BANK_SIZE;
    for (i = 0; i < NvRmPwmOutputId_Num-2; i++)
    {
        status = NvRmPhysicalMemMap(
            PwmPhysAddr + i*s_hPwm->PwmBankSize,
            s_hPwm->PwmBankSize, 
            NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&s_hPwm->VirtualAddress[i]);
        if (status != NvSuccess)
        {
            NvOsFree(s_hPwm);
            goto fail;
        }
    }

    // Get the pmc physical and virtual base address
    NvRmModuleGetBaseAddress(hDevice,
            NVRM_MODULE_ID(NvRmModuleID_Pmif, 0),
            &PmcPhysAddr, &(s_hPwm->PmcBankSize));
    s_hPwm->PmcBankSize = PMC_BANK_SIZE;

    status = NvRmPhysicalMemMap(
            PmcPhysAddr,
            s_hPwm->PmcBankSize, 
            NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&s_hPwm->VirtualAddress[NvRmPwmOutputId_Num-2]);
    if (status != NvSuccess)
    {
        NvOsFree(s_hPwm);
        goto fail;
    }

    caps[0].MajorVersion = 1;
    caps[0].MinorVersion = 0;
    caps[0].EcoLevel = 0;
    caps[0].Capability = &caps[0];

    caps[1].MajorVersion = 1;
    caps[1].MinorVersion = 1;
    caps[1].EcoLevel = 0;
    caps[1].Capability = &caps[1];

    caps[2].MajorVersion = 1;
    caps[2].MinorVersion = 2;
    caps[2].EcoLevel = 0;
    caps[2].Capability = &caps[2];

    caps[3].MajorVersion = 2;
    caps[3].MinorVersion = 0;
    caps[3].EcoLevel = 0;
    caps[3].Capability = &caps[3];

    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(
        hDevice,
        NvRmModuleID_Pwm,
        caps,
        sizeof(caps)/sizeof(caps[0]),
        (void**)&pCap));

    if ((pCap->MajorVersion > 1) ||
        ((pCap->MajorVersion == 1) && (pCap->MinorVersion > 0)))
            s_IsFreqDividerSupported = NV_TRUE;

    s_hPwm->RefCount++;
exit:
    *phPwm = s_hPwm;
    NvOsMutexUnlock(s_hPwmMutex);
    return NvSuccess;

fail:
    NvOsMutexUnlock(s_hPwmMutex);
    return status;
}

void NvRmPwmClose(NvRmPwmHandle hPwm)
{ 
    NvU32 i;
    if (!hPwm)
        return;

    NV_ASSERT(hPwm->RefCount);    

    NvOsMutexLock(s_hPwmMutex);
    hPwm->RefCount--;
    if (hPwm->RefCount == 0)
    {
        // Unmap the pwm register virtual address space
        for (i = 0; i < NvRmPwmOutputId_Num-2; i++)
        {
            NvRmPhysicalMemUnmap((void*)s_hPwm->VirtualAddress[i],
                             s_hPwm->PwmBankSize);
        }
        
        // Unmap the pmc register virtual address space
        NvRmPhysicalMemUnmap(
                (void*)s_hPwm->VirtualAddress[NvRmPwmOutputId_Num-2],
                s_hPwm->PmcBankSize);

        if (s_IsPwmFirstConfig)
        {
            // Disable power
            PwmPowerConfigure(hPwm, NV_FALSE);

            // Unregister with RM power
            NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);

            // Tri-state the pin-mux pins
            NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hPwm->RmDeviceHandle,
                NVRM_MODULE_ID(NvRmModuleID_Pwm, 0), NV_TRUE));
            s_IsPwmFirstConfig = NV_FALSE;
        }
        NvOsFree(s_hPwm);
        s_hPwm = NULL;
    }
    NvOsMutexUnlock(s_hPwmMutex);
}

#define MAX_DUTY_CYCLE 255
#define PWM_FREQ_FACTOR 256

NvError NvRmPwmConfig(
    NvRmPwmHandle hPwm,
    NvRmPwmOutputId OutputId,  
    NvRmPwmMode Mode, 
    NvU32 DutyCycle,
    NvU32 RequestedFreqHzOrPeriod,
    NvU32 *pCurrentFreqHzOrPeriod)
{
    NvError status = NvSuccess;
    NvU32 RegValue = 0, ResultFreqKHz = 0;
    NvU8 PwmMode = 0;
    NvU32 ClockFreqKHz = 0, DCycle = 0;
    NvU32 PmcCtrlReg = 0, PmcDpdPadsReg = 0, PmcBlinkTimerReg = 0;
    NvU32 RequestPeriod = 0, ResultPeriod = 0;
    NvU32 DataOnRegVal = 0, DataOffRegVal = 0;
    NvU32 *pPinMuxConfigTable = NULL;
    NvU32 Count = 0, divider = 1;
    
    NvOsMutexLock(s_hPwmMutex);
    
    if (OutputId != NvRmPwmOutputId_Blink)
    {
        if (!s_IsPwmFirstConfig)
        {
            hPwm->PowerEnabled = NV_FALSE;
            // Register with RM power
            s_PwmPowerID = NVRM_POWER_CLIENT_TAG('P','W','M',' ');
            status = NvRmPowerRegister(hPwm->RmDeviceHandle, NULL, &s_PwmPowerID);
            if (status != NvSuccess)
                goto fail;

            // Enable power
            status = PwmPowerConfigure(hPwm, NV_TRUE);
            if (status != NvSuccess)
            {
                NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);
                goto fail;
            }

            // Reset pwm module
            NvRmModuleReset(hPwm->RmDeviceHandle, NVRM_MODULE_ID(NvRmModuleID_Pwm, 0));

            // Config pwm pinmux
            NvOdmQueryPinMux(NvOdmIoModule_Pwm, (const NvU32 **)&pPinMuxConfigTable,
                        &Count);
            if (Count != 1)
            {
                status = NvError_NotSupported;
                PwmPowerConfigure(hPwm, NV_FALSE);
                NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);
                goto fail;
            }
            hPwm->PinMap = pPinMuxConfigTable[0];
            status = NvRmSetModuleTristate(hPwm->RmDeviceHandle,
                NVRM_MODULE_ID(NvRmModuleID_Pwm, 0), NV_FALSE);

            if (status != NvSuccess)
            {
                PwmPowerConfigure(hPwm, NV_FALSE);
                NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);
                goto fail;
            }
            s_IsPwmFirstConfig = NV_TRUE;
        }

        // Enable power
        status = PwmPowerConfigure(hPwm, NV_TRUE);
        if (status != NvSuccess)
        {
            NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);
            goto fail;
        }


        // Validate PWM output and pin map config
        status = PwmCheckValidConfig(hPwm, OutputId, Mode);
        if (status != NvSuccess)
            goto fail;

        ClockFreqKHz = (RequestedFreqHzOrPeriod * PWM_FREQ_FACTOR) / 1000;
        if (ClockFreqKHz == 0)
            ClockFreqKHz = 1;

        if (RequestedFreqHzOrPeriod == NvRmFreqMaximum)
            ClockFreqKHz = NvRmFreqMaximum;

        status = NvRmPowerModuleClockConfig(hPwm->RmDeviceHandle, 
                NVRM_MODULE_ID(NvRmModuleID_Pwm, 0),
                s_PwmPowerID,
                NvRmFreqUnspecified,
                NvRmFreqUnspecified,
                &ClockFreqKHz,
                1,
                &ResultFreqKHz,
                0);
        if (status != NvSuccess)
            goto fail;

        *pCurrentFreqHzOrPeriod = (ResultFreqKHz * 1000) / PWM_FREQ_FACTOR;

        if (Mode == NvRmPwmMode_Disable)
            PwmMode = 0;
        else
            PwmMode = 1;

        /*
         * Convert from percentage unsigned 15.16 fixed point
         * format to actual register value
         */
        DCycle = (DutyCycle * MAX_DUTY_CYCLE/100)>>16;
        if (DCycle > MAX_DUTY_CYCLE)
            DCycle = MAX_DUTY_CYCLE;

        RegValue = PWM_SETNUM(CSR_0, ENB, PwmMode) |
           PWM_SETNUM(CSR_0, PWM_0, DCycle);

        if (s_IsFreqDividerSupported)
        {
            if ((*pCurrentFreqHzOrPeriod > RequestedFreqHzOrPeriod) &&
                (RequestedFreqHzOrPeriod != 0))
            {
                divider = *pCurrentFreqHzOrPeriod/RequestedFreqHzOrPeriod;
                if ((*pCurrentFreqHzOrPeriod%RequestedFreqHzOrPeriod)*2>RequestedFreqHzOrPeriod)
                    divider +=1;
                *pCurrentFreqHzOrPeriod = *pCurrentFreqHzOrPeriod / divider;
                RegValue |= PWM_SETNUM(CSR_0, PFM_0, divider);
            }
        }

        PWM_REGW(hPwm->VirtualAddress[OutputId-1], 0, RegValue);

        // If PWM mode is disabled and all pwd channels are disabled then
        // disable power to PWM
        if (!PwmMode)
        {
            if (IsPwmDisabled(hPwm))
            {
                // Disable power
                status = PwmPowerConfigure(hPwm, NV_FALSE);
                if (status != NvSuccess)
                {
                    NvRmPowerUnRegister(hPwm->RmDeviceHandle, s_PwmPowerID);
                    goto fail;
                }
            }
        }

    }
    else
    {
        ResultPeriod = RequestedFreqHzOrPeriod;
        if (ResultPeriod > MAX_SUPPORTED_PERIOD)
            ResultPeriod = MAX_SUPPORTED_PERIOD;
        RequestPeriod = ResultPeriod * DATA_FACTOR;

        DCycle = DutyCycle>>16;
        if(DCycle > 100)
            DCycle = 100;

        DataOnRegVal = (RequestPeriod * DCycle)/100;
        if(DataOnRegVal > MAX_DATA_ON)
            DataOnRegVal = MAX_DATA_ON;

        DataOffRegVal = RequestPeriod - DataOnRegVal;
        if(DataOffRegVal > MAX_DATA_OFF)
            DataOffRegVal = MAX_DATA_OFF;

        PmcCtrlReg = PMC_REGR(hPwm->VirtualAddress[OutputId-1],
                                APBDEV_PMC_CNTRL_0);
        PmcDpdPadsReg = PMC_REGR(hPwm->VirtualAddress[OutputId-1],
                                APBDEV_PMC_DPD_PADS_ORIDE_0);
        PmcBlinkTimerReg = PMC_REGR(hPwm->VirtualAddress[OutputId-1],
                                APBDEV_PMC_BLINK_TIMER_0);
        PmcBlinkTimerReg &=~PMC_SETNUM(BLINK_TIMER, DATA_OFF, 0xFFFF);
        PmcBlinkTimerReg &=~PMC_SETNUM(BLINK_TIMER, DATA_ON, 0xFFFF);
        PmcBlinkTimerReg |=PMC_SETNUM(BLINK_TIMER, DATA_OFF, DataOffRegVal);
        PmcBlinkTimerReg |=PMC_SETNUM(BLINK_TIMER, DATA_ON, DataOnRegVal);
        PmcCtrlReg |= PMC_SETDEF(CNTRL, BLINK_EN, ENABLE);
        PmcDpdPadsReg |= PMC_SETDEF(DPD_PADS_ORIDE, BLINK, ENABLE);
        if (Mode == NvRmPwmMode_Blink_LED) 
        {
            PmcBlinkTimerReg |= (1 << 15);
        }
        
        if (Mode == NvRmPwmMode_Blink_32KHzClockOutput)
        {
            PmcBlinkTimerReg &= ~(1 << 15);
        }

        if (Mode == NvRmPwmMode_Blink_Disable)
        {
            PmcCtrlReg &= ~PMC_SETDEF(CNTRL, BLINK_EN, ENABLE);
            PmcDpdPadsReg &= ~PMC_SETDEF(DPD_PADS_ORIDE, BLINK, ENABLE);
        }
        PmcBlinkTimerReg |=PMC_SETNUM(BLINK_TIMER, DATA_OFF, DataOffRegVal)
                         | PMC_SETNUM(BLINK_TIMER, DATA_ON, DataOnRegVal);
        PMC_REGW(hPwm->VirtualAddress[OutputId-1], 
            APBDEV_PMC_CNTRL_0, PmcCtrlReg); 
        PMC_REGW(hPwm->VirtualAddress[OutputId-1], 
            APBDEV_PMC_DPD_PADS_ORIDE_0, PmcDpdPadsReg); 
        PMC_REGW(hPwm->VirtualAddress[OutputId-1], 
            APBDEV_PMC_BLINK_TIMER_0, PmcBlinkTimerReg);     
        *pCurrentFreqHzOrPeriod = ResultPeriod;
    }
fail:
    NvOsMutexUnlock(s_hPwmMutex);
    return status;
}

