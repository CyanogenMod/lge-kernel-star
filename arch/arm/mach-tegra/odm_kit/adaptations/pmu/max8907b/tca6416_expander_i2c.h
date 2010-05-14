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

#ifndef INCLUDED_TCA6416_EXPANDER_I2C_H
#define INCLUDED_TCA6416_EXPANDER_I2C_H

#include "nvodm_pmu.h"
#include "max8907b.h"

#if defined(__cplusplus)
extern "C"
{
#endif



/**
 * @brief Defines the possible modes.
 */

typedef enum
{

    /**
     * Specifies the gpio pin as not in use. 
     */
    GpioPinMode_Inactive = 0,

    /// Specifies the gpio pin mode as input.
    GpioPinMode_InputData,

    /// Specifies the gpio pin mode as output.
    GpioPinMode_Output,

    GpioPinMode_Num,
    GpioPinMode_Force32 = 0x7FFFFFFF
} GpioPinMode;


/** 
 * @brief Defines the pin state
 */

typedef enum
{
   // Pin state high 
    GpioPinState_Low = 0,
    // Pin is high
    GpioPinState_High,
    GpioPinState_Num,
    GpioPinState_Force32 = 0x7FFFFFFF
} GpioPinState;


NvBool Tca6416ConfigPortPin(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 portNo,
    NvU32 pinNo,
    GpioPinMode Mode);

NvBool Tca6416WritePortPin(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 portNo,
    NvU32 pinNo,
    GpioPinState data);

NvBool Tca6416ReadPortPin(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 portNo,
    NvU32 pinNo, 
    GpioPinState*State);


#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_TCA6416_EXPANDER_I2C_H
