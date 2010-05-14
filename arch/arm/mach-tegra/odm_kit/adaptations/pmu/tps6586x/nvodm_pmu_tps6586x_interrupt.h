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

#ifndef INCLUDED_TPS6586X_INTERRUPT_HEADER
#define INCLUDED_TPS6586X_INTERRUPT_HEADER

#include "nvodm_pmu_tps6586x.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct TPS6586xStatusRef
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* Main charger Presents */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

    /* Porwer In type*/
    NvU32   powerType;      /* Bit meanings:
                                0: AC_DET, 
                                1: USB_DET,
                                2: BAT_DET */                           
    NvU32   powerGood;      /* Bit meanings:
                                0-7: LDO0 to LDO7 
                                10-11: LDO8 and LDO9,
                                12-15: SMO0 to SM11 */
}TPS6586xStatus;

#if 0
typedef struct {
    NvBool pmuInterruptSupported;
    NvBool pmuPresented; 
    NvBool battPresence;
    TPS6586xStatus pmuStatus;
} TPS6586xDevice, *TPS6586xHandle;
#endif

NvBool 
Tps6586xSetupInterrupt(
    NvOdmPmuDeviceHandle hDevice,
    TPS6586xStatus *pmuStatus);



void 
Tps6586xInterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice,
    TPS6586xStatus *pmuStatus);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_INTERRUPT_HEADER
