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
