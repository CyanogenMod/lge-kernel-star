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

#ifndef INCLUDED_NVODM_PMU_MAX8907_I2C_H
#define INCLUDED_NVODM_PMU_MAX8907_I2C_H

#if defined(__cplusplus)
extern "C"
{
#endif

// Constant definition
// #define PMU_MAX8907_DEVADDR     TBD
#define PMU_MAX8907_I2C_SPEED_KHZ   400

// Function declaration
NvBool Max8907I2cWrite8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 Data);

NvBool Max8907I2cRead8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 *Data);

//20100526, cs77.ha@lge.com, PMIC I2C [START]
#if defined(CONFIG_MACH_STAR)
NvBool Max8907RtcI2cWrite8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 Data);

NvBool Max8907RtcI2cRead8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 *Data);
#endif
//20100526, cs77.ha@lge.com, PMIC I2C [END]

NvBool Max8907I2cWrite32(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 Data);

NvBool Max8907I2cRead32(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 *Data);

NvBool Max8907RtcI2cWriteTime(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 Data);

NvBool Max8907RtcI2cReadTime(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 *Data);

NvBool Max8907AdcI2cWrite8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 Data);

NvBool Max8907AdcI2cRead8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 *Data);

NvBool Max8907AdcI2cWriteAddrOnly(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr);

#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_PMU_MAX8907_I2C_H
