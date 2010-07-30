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
 * @brief <b>NVIDIA Driver Development Kit: Timer API</b>
 *
 * @b Description: Contains the pwm declarations.
 */

#ifndef INCLUDED_PWM_PRIVATE_H
#define INCLUDED_PWM_PRIVATE_H

#include "nvrm_module.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pwm.h"

#define PWM_BANK_SIZE           16
#define PMC_BANK_SIZE           192
#define MAX_SUPPORTED_PERIOD    16
#define MAX_DATA_ON             0x7FFF
#define MAX_DATA_OFF            0xFFFF
#define DATA_FACTOR             2048 // 1s / (16 * 30.51us)

typedef struct NvRmPwmRec
{
    // RM device handle
    NvRmDeviceHandle RmDeviceHandle;

    // Pwm configuration pin-map.
    NvOdmPwmPinMap PinMap;

    // Pwm open reference count
    NvU32 RefCount;

    // Pwm virtual base address
    NvU32   VirtualAddress[NvRmPwmOutputId_Num-1];

    // Pwm bank size
    NvU32 PwmBankSize;

    // Pmc bank size
    NvU32 PmcBankSize;

    // pmu powerEnabled flag
    NvBool PowerEnabled;
} NvRmPwm;

#define PWM_RESET(r)            NV_RESETVAL(PWM_CONTROLLER_PWM,r)
#define PWM_SETDEF(r,f,c)       NV_DRF_DEF(PWM_CONTROLLER_PWM,r,f,c)
#define PWM_SETNUM(r,f,n)       NV_DRF_NUM(PWM_CONTROLLER_PWM,r,f,n)
#define PWM_GET(r,f,v)          NV_DRF_VAL(PWM_CONTROLLER_PWM,r,f,v)
#define PWM_CLRSETDEF(v,r,f,c)  NV_FLD_SET_DRF_DEF(PWM_CONTROLLER,r,f,c,v)
#define PWM_CLRSETNUM(v,r,f,n)  NV_FLD_SET_DRF_NUM(PWM_CONTROLLER,r,f,n,v)
#define PWM_MASK(x,y)           (1 << (PWM_CONTROLLER_##x##_0 - PWMCONTROLLER_##y##_0))

#define PMC_RESET(r)            NV_RESETVAL(APBDEV_PMC,r)
#define PMC_SETDEF(r,f,c)       NV_DRF_DEF(APBDEV_PMC,r,f,c)
#define PMC_SETNUM(r,f,n)       NV_DRF_NUM(APBDEV_PMC,r,f,n)
#define PMC_GET(r,f,v)          NV_DRF_VAL(APBDEV_PMC,r,f,v)
#define PMC_CLRSETDEF(v,r,f,c)  NV_FLD_SET_DRF_DEF(APBDEV_PMC,r,f,c,v)
#define PMC_CLRSETNUM(v,r,f,n)  NV_FLD_SET_DRF_NUM(APBDEV_PMC,r,f,n,v)

#endif  // INCLUDED_PWM_PRIVATE_H


