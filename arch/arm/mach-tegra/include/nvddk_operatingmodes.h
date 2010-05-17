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

#ifndef INCLUDED_NVDDK_OPERATINGMODES_H
#define INCLUDED_NVDDK_OPERATINGMODES_H

/**
 * @defgroup nvddk_operatingmodes_group  Operating Modes
 *
 * Provides operating mode information for NVIDIA Boot Loader functions.
 *
 * @ingroup nvddk_group
 * @{
 */

/**
 * Defines the supported operating modes.
 */
typedef enum
{
    // Specifies development mode; boot loader is validated using a fixed key of zeroes.
    NvDdkOperatingMode_NvProduction = 3,

    // Specifies production mode; boot loader is decrypted using the Secure Boot Key,
    // then validated using the Secure Boot Key.
    NvDdkOperatingMode_OdmProductionSecure,

    // Specifies ODM production mode; boot loader is validated using the Secure Boot Key.
    NvDdkOperatingMode_OdmProductionOpen,

    // Undefined.
    NvDdkOperatingMode_Undefined,

    // Ignore -- Forces compilers to make 32-bit enums.
    NvDdkOperatingMode_Force32 = 0x7FFFFFFF
} NvDdkOperatingMode;

/** @} */

#endif // INCLUDED_NVDDK_OPERATINGMODES_H

