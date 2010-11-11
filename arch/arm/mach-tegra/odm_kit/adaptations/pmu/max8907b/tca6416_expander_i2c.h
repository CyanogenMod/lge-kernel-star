/*
 * Copyright (c) 2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
