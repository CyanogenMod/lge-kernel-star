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
 * @brief <b>NVIDIA Driver Development Kit: Fuse API</b>
 *
 * @b Description: Contains the NvRM Chip unique id implementation.
 */
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "nvrm_hardware_access.h"
#include "nvrm_hwintf.h"
#include "ap15/arclk_rst.h"
#include "ap15/arfuse.h"
#include "ap15/ap15rm_private.h"
#include "ap15rm_clocks.h"

NvError NvRmPrivAp15ChipUniqueId(NvRmDeviceHandle hDevHandle,void* pId)
{
    NvU32   OldRegData;            // Old register contents
    NvU32   NewRegData;            // New register contents
    NvU64   Temp;                  // Temp buffer to read the contents of fuses
    NV_ASSERT(hDevHandle);
    NV_ASSERT(pId);

#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    Ap15EnableModuleClock(hDevHandle, NvRmModuleID_Fuse, NV_TRUE);
#endif

    // Access to unique id is protected, so make sure all registers visible
    // first.
    OldRegData = NV_REGR(hDevHandle, 
                        NvRmPrivModuleID_ClockAndReset,
                         0, 
                        CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    NewRegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                     MISC_CLK_ENB,
                                     CFG_ALL_VISIBLE,
                                     1,
                                     OldRegData);
    NV_REGW(hDevHandle,
             NvRmPrivModuleID_ClockAndReset,
             0,
             CLK_RST_CONTROLLER_MISC_CLK_ENB_0,
             NewRegData);

    // Read the secure id from the fuse registers in to a local buffer
    Temp = ((NvU64)NV_REGR(hDevHandle,
                            (NvRmPrivModuleID)NvRmModuleID_Fuse,
                             0,
                             FUSE_JTAG_SECUREID_0_0)) |
            (((NvU64)NV_REGR(hDevHandle,
                             (NvRmPrivModuleID)NvRmModuleID_Fuse,
                              0,
                             FUSE_JTAG_SECUREID_1_0)) << 32);
    // Copy the read data to output buffer
    NvOsMemcpy(pId,&Temp,sizeof(NvU64));

    // Restore the protected registers enable to the way we found it.
    NV_REGW(hDevHandle,
         NvRmPrivModuleID_ClockAndReset,
         0,
         CLK_RST_CONTROLLER_MISC_CLK_ENB_0,
         OldRegData);

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    Ap15EnableModuleClock(hDevHandle, NvRmModuleID_Fuse, NV_FALSE);
#endif

    return NvError_Success;
}
