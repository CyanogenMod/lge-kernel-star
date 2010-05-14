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

#ifndef INCLUDED_AD5258_DPM_I2C_H
#define INCLUDED_AD5258_DPM_I2C_H

#include "nvodm_pmu.h"
#include "max8907b.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#define AD5258_SLAVE_ADDR       (0x9C)    // (7'h4E)
#define AD5258_I2C_SPEED_KHZ    (400)
#define AD5258_I2C_RETRY_CNT    (2)
#define AD5258_I2C_TIMEOUT_MS   (1000)

#define AD5258_RDAC_ADDR        (0x0)
#define AD5258_RDAC_MASK        (0x3F)

/*
 * Linear approximation of digital potentiometer (DPM) scaling ladder:
 * D(Vout) = (Vout - V0) * M1 / 2^b
 * Vout(D) = V0 + (D * M2) / 2^b
 * D - DPM setting, Vout - output voltage in mV, b - fixed point calculation
 * precision, approximation parameters V0, M1, M2 are determined for the
 * particular schematic combining constant resistors, DPM, and DCDC supply.
 */
// On Whistler:
#define AD5258_V0   (815)
#define AD5258_M1   (229)        
#define AD5258_M2   (4571)
#define AD5258_b    (10)

#define AD5258_VMAX (AD5258_V0 + ((AD5258_RDAC_MASK * AD5258_M2 + \
                     (0x1 << (AD5258_b - 1))) >> AD5258_b))

// Minimum voltage step is determined by DPM resolution, maximum voltage step
// is limited to keep dynamic over/under shoot within +/- 50mV
#define AD5258_MIN_STEP_MV ((AD5258_M2 + (0x1 << AD5258_b) - 1) >> AD5258_b)
#define AD5258_MAX_STEP_MV (50)
#define AD5258_MAX_STEP_SETTLE_TIME_US (20)
#define AD5258_TURN_ON_TIME_US (2000)

NvBool 
Ad5258I2cSetVoltage(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 MilliVolts);

NvBool 
Ad5258I2cGetVoltage(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* pMilliVolts);

#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_AD5258_DPM_I2C_H
