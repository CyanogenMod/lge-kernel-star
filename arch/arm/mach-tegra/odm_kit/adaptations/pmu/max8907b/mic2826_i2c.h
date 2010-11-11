/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#ifndef INCLUDED_NVODM_PMU_MIC2826_I2C_H
#define INCLUDED_NVODM_PMU_MIC2826_I2C_H

#include "nvodm_pmu.h"
#include "max8907b.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Constant definition
#define MIC2826_SLAVE_ADDR      0xB4
#define MIC2826_I2C_SPEED_KHZ   400

// Function declaration
NvBool MIC2826I2cWrite8(
   NvOdmPmuDeviceHandle hPmu,
   NvU8 Addr,
   NvU8 Data);

NvBool MIC2826I2cRead8(
   NvOdmPmuDeviceHandle hPmu,
   NvU8 Addr,
   NvU8 *Data);

#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_PMU_MIC2826_I2C_H
