/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
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

#include "nvcommon.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvidlcmd.h"
#include "nvreftrack.h"
#include "nvrm_xpc.h"
#include "nvrm_transport.h"
#include "nvrm_memctrl.h"
#include "nvrm_pwm.h"
#include "nvrm_keylist.h"
#include "nvrm_pmu.h"
#include "nvrm_diag.h"
#include "nvrm_pinmux.h"
#include "nvrm_analog.h"
#include "nvrm_owr.h"
#include "nvrm_i2c.h"
#include "nvrm_spi.h"
#include "nvrm_interrupt.h"
#include "nvrm_dma.h"
#include "nvrm_power.h"
#include "nvrm_gpio.h"
#include "nvrm_module.h"
#include "nvrm_memmgr.h"
#include "nvrm_init.h"
#include <linux/kernel.h>
#include <linux/syscalls.h>

NvError nvrm_xpc_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_transport_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_memctrl_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_pwm_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_keylist_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_pmu_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_diag_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_pinmux_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_analog_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_owr_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_i2c_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_spi_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_interrupt_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_dma_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_power_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_gpio_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_module_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_memmgr_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );
NvError nvrm_init_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );

NvError nvrm_pcie_Dispatch( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    return NvError_AccessDenied;
}

// NvRm Package
typedef enum
{
    NvRm_Invalid = 0,
    NvRm_nvrm_xpc,
    NvRm_nvrm_transport,
    NvRm_nvrm_memctrl,
    NvRm_nvrm_pcie,
    NvRm_nvrm_pwm,
    NvRm_nvrm_keylist,
    NvRm_nvrm_pmu,
    NvRm_nvrm_diag,
    NvRm_nvrm_pinmux,
    NvRm_nvrm_analog,
    NvRm_nvrm_owr,
    NvRm_nvrm_i2c,
    NvRm_nvrm_spi,
    NvRm_nvrm_interrupt,
    NvRm_nvrm_dma,
    NvRm_nvrm_power,
    NvRm_nvrm_gpio,
    NvRm_nvrm_module,
    NvRm_nvrm_memmgr,
    NvRm_nvrm_init,
    NvRm_Num,
    NvRm_Force32 = 0x7FFFFFFF,
} NvRm;

typedef NvError (* NvIdlDispatchFunc)( NvU32 function, void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );

typedef struct NvIdlDispatchTableRec
{
    NvU32 PackageId;
    NvIdlDispatchFunc DispFunc;
} NvIdlDispatchTable;

static NvIdlDispatchTable gs_DispatchTable[] =
{
    { NvRm_nvrm_xpc, nvrm_xpc_Dispatch },
    { NvRm_nvrm_transport, nvrm_transport_Dispatch },
    { NvRm_nvrm_memctrl, nvrm_memctrl_Dispatch },
    { NvRm_nvrm_pcie, nvrm_pcie_Dispatch },
    { NvRm_nvrm_pwm, nvrm_pwm_Dispatch },
    { NvRm_nvrm_keylist, nvrm_keylist_Dispatch },
    { NvRm_nvrm_pmu, nvrm_pmu_Dispatch },
    { NvRm_nvrm_diag, nvrm_diag_Dispatch },
    { NvRm_nvrm_pinmux, nvrm_pinmux_Dispatch },
    { NvRm_nvrm_analog, nvrm_analog_Dispatch },
    { NvRm_nvrm_owr, nvrm_owr_Dispatch },
    { NvRm_nvrm_i2c, nvrm_i2c_Dispatch },
    { NvRm_nvrm_spi, nvrm_spi_Dispatch },
    { NvRm_nvrm_interrupt, nvrm_interrupt_Dispatch },
    { NvRm_nvrm_dma, nvrm_dma_Dispatch },
    { NvRm_nvrm_power, nvrm_power_Dispatch },
    { NvRm_nvrm_gpio, nvrm_gpio_Dispatch },
    { NvRm_nvrm_module, nvrm_module_Dispatch },
    { NvRm_nvrm_memmgr, nvrm_memmgr_Dispatch },
    { NvRm_nvrm_init, nvrm_init_Dispatch },
    { 0 },
};

NvError NvRm_Dispatch( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvU32 packid_;
    NvU32 funcid_;
    NvIdlDispatchTable *table_;

    NV_ASSERT( InBuffer );
    NV_ASSERT( OutBuffer );

    packid_ = ((NvU32 *)InBuffer)[0];
    funcid_ = ((NvU32 *)InBuffer)[1];
    table_ = gs_DispatchTable;

    if ( packid_-1 >= NV_ARRAY_SIZE(gs_DispatchTable) ||
         !table_[packid_ - 1].DispFunc )
        return NvError_IoctlFailed;

    return table_[packid_ - 1].DispFunc( funcid_, InBuffer, InSize,
        OutBuffer, OutSize, Ctx );
}

static NvU32 s_FuncAllowedInNvrmModulePackage =
    (0 << 15) | //err_ = NvRegw08_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 14) | //err_ = NvRegr08_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 13) | //err_ = NvRegrb_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 12) | //err_ = NvRegwb_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 11) | //err_ = NvRegwm_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 10) | //err_ = NvRegrm_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 9)  | //err_ = NvRegw_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 8)  | //err_ = NvRegr_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 7)  | //err_ = NvRmGetRandomBytes_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 6)  | //err_ = NvRmQueryChipUniqueId_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 5)  | //err_ = NvRmModuleGetCapabilities_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 4)  | //err_ = NvRmModuleResetWithHold_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 3)  | //err_ = NvRmModuleReset_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 2)  | //err_ = NvRmModuleGetNumInstances_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 1)  | //err_ = NvRmModuleGetBaseAddress_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 0);   //err_ = NvRmModuleGetModuleInfo_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );

static NvU32 s_FuncAllowedInNvrmInitPackage =
    (1 << 3)  | //err_ = NvRmClose_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 2)  | //err_ = NvRmOpenNew_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 1)  | //err_ = NvRmInit_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 0);  //err_ = NvRmOpen_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );

static NvU32 s_FuncAllowedInNvrmPowerPackage_1 =
    (1 << 31) | //err_ = NvRmDfsGetLowVoltageThreshold_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 30) | //err_ = NvRmDfsLogBusyGetEntry_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 29) | //err_ = NvRmDfsLogStarvationGetEntry_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 28) | //err_ = NvRmDfsLogActivityGetEntry_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 27) | //err_ = NvRmDfsLogGetMeanFrequencies_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 26) | //err_ = NvRmDfsLogStart_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 25) | //err_ = NvRmDfsGetProfileData_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 24) | //err_ = NvRmDfsSetAvHighCorner_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 23) | //err_ = NvRmDfsSetCpuEmcHighCorner_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 22) | //err_ = NvRmDfsSetEmcEnvelope_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 21) | //err_ = NvRmDfsSetCpuEnvelope_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 20) | //err_ = NvRmDfsSetTarget_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 19) | //err_ = NvRmDfsSetLowCorner_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 18) | //err_ = NvRmDfsSetState_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 17) | //err_ = NvRmDfsGetClockUtilization_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 16) | //err_ = NvRmDfsGetState_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 15) | //err_ = NvRmPowerActivityHint_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 14) | //err_ = NvRmPowerStarvationHintMulti_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 13) | //err_ = NvRmPowerStarvationHint_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 12) | //err_ = NvRmPowerBusyHintMulti_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 11) | //err_ = NvRmPowerBusyHint_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 10) | //err_ = NvRmListPowerAwareModules_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 9)  | //err_ = NvRmPowerVoltageControl_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 8)  | //err_ = NvRmPowerModuleClockControl_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 7)  | //err_ = NvRmPowerModuleClockConfig_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 6)  | //err_ = NvRmPowerModuleGetMaxFrequency_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 5)  | //err_ = NvRmPowerGetPrimaryFrequency_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 4)  | //err_ = NvRmPowerGetState_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (0 << 3)  | //err_ = NvRmPowerEventNotify_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 2)  | //err_ = NvRmPowerGetEvent_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 1)  | //err_ = NvRmPowerUnRegister_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );
    (1 << 0);  //err_ = NvRmPowerRegister_dispatch_( InBuffer, InSize, OutBuffer, OutSize, Ctx );

NvError NvRm_Dispatch_Others( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx )
{
    NvU32 packid_;
    NvU32 funcid_;
    NvIdlDispatchTable *table_;

    NV_ASSERT( InBuffer );
    NV_ASSERT( OutBuffer );

    packid_ = ((NvU32 *)InBuffer)[0];
    funcid_ = ((NvU32 *)InBuffer)[1];
    table_ = gs_DispatchTable;

    switch (packid_)
    {
        case NvRm_nvrm_init:
            if ( !(s_FuncAllowedInNvrmInitPackage & (1 << funcid_)) )
                goto fail;
            break;
        case NvRm_nvrm_module:
            if ( !(s_FuncAllowedInNvrmModulePackage & (1 << funcid_)) )
                goto fail;
            break;
        case NvRm_nvrm_power:
            if ( !(s_FuncAllowedInNvrmPowerPackage_1 & (1 << funcid_)) )
                goto fail;
            break;
        default:
            goto fail;
    }

    if ( packid_-1 >= NV_ARRAY_SIZE(gs_DispatchTable) ||
         !table_[packid_ - 1].DispFunc )
        return NvError_IoctlFailed;

    return table_[packid_ - 1].DispFunc( funcid_, InBuffer, InSize,
        OutBuffer, OutSize, Ctx );
fail:
    pr_debug("\n\n\n\n*****nvrm dispatch permission error,"
        "packid_=%d, funcid_=%d,uid=%d,gid=%d****\n\n\n\n",
        packid_, funcid_, (int)sys_getuid(), (int)sys_getgid());
    return NvError_IoctlFailed;
}

