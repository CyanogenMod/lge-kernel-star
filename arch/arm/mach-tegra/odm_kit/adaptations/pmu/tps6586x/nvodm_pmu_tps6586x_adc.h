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

#ifndef INCLUDED_TPS6586X_ADC_HEADER
#define INCLUDED_TPS6586X_ADC_HEADER

/* the ADC is used for battery voltage conversion */
#include "nvodm_pmu_tps6586x.h"


#if defined(__cplusplus)
extern "C"
{
#endif

/* read voltage from adc */
NvBool 
Tps6586xAdcVBatSenseRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *volt);

/* read bat temperature voltage from ADC */
NvBool 
Tps6586xAdcVBatTempRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *volt);

/* Calculate the battery temperature */
NvU32
Tps6586xBatteryTemperature(
    NvU32 VBatSense,
    NvU32 VBatTemp);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_ADC_HEADER
