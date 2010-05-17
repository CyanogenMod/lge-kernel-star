/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#ifndef INCLUDED_NVDDK_BOOTDEVICES_H
#define INCLUDED_NVDDK_BOOTDEVICES_H

/**
 * @defgroup nvddk_bootdevices_group Boot Device Types
 *
 * Provides types of boot devices for NVIDIA Tegra functions.
 *
 * @ingroup nvddk_group
 * @{
 */

/**
 * Defines the supported secondary boot device types.
 */
typedef enum
{
    // Specifies NAND flash (SLC and MLC).
    NvDdkSecBootDeviceType_Nand = 1,

    // Specifies NOR flash.
    NvDdkSecBootDeviceType_Nor,

    // Specifies SPI flash.
    NvDdkSecBootDeviceType_Spi,

    // Specifies eMMC flash
    // @note eSD is only supported on Tegra 2 devices.
    NvDdkSecBootDeviceType_eMMC,

    // Specifies 16-bit NAND flash.
    NvDdkSecBootDeviceType_Nand_x16,

    // Specifies mobileLBA NAND flash (Tegra 2 only).
    NvDdkSecBootDeviceType_MobileLbaNand,

    // Specifies SD flash.
    // @note eSD is only supported on Tegra 2 devices.
    NvDdkSecBootDeviceType_Sdmmc,

    // Specifies MuxOneNAND flash (Tegra 2 only).
    NvDdkSecBootDeviceType_MuxOneNand,

    // Specifies the maximum number of flash device types
    // -- Should appear after the last legal item.
    NvDdkSecBootDeviceType_Max,

    // Undefined.
    NvDdkSecBootDeviceType_Undefined,

    // Ignore -- Forces compilers to make 32-bit enums.
    NvDdkSecBootDeviceType_Force32 = 0x7FFFFFFF
} NvDdkSecBootDeviceType;

/** @} */

#endif // INCLUDED_NVDDK_BOOTDEVICES_H

