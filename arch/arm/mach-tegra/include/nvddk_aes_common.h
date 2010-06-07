/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#ifndef INCLUDED_nvddk_aes_common_H
#define INCLUDED_nvddk_aes_common_H

#if defined(__cplusplus)
extern "C"
{
#endif


/** @file
 * @brief <b>NVIDIA Driver Development Kit: NvDdkAes APIs</b>
 *
 * @b Description: This file defines the common definitions for the AES DDK.
 */

/**
 * @defgroup nvddk_aes_common Advanced Encryption Standard API
 *
 * @ingroup nvddk_modules
 * @{
 */

/**
 * AES-related constants.
 */

typedef enum
{

    /**
     * Max supported AES key length, in bytes.
     */
    NvDdkAesConst_MaxKeyLengthBytes = 32,

    /**
     * AES initial vector length, in bytes.
     */
    NvDdkAesConst_IVLengthBytes = 16,

    /**
     * AES block size, in bytes.
     */
    NvDdkAesConst_BlockLengthBytes = 16,
    NvDdkAesConst_Num,
    NvDdkAesConst_Force32 = 0x7FFFFFFF
} NvDdkAesConst;

/**
 * AES key lengths.
 */

typedef enum
{
    NvDdkAesKeySize_Invalid,

    /**
     * Define 128 bit key length size in bytes.
     */
    NvDdkAesKeySize_128Bit = 16,

    /**
     * Define 192 bit key length size in bytes.
     */
    NvDdkAesKeySize_192Bit = 24,

    /**
     * Define 256 bit key length size in bytes.
     */
    NvDdkAesKeySize_256Bit = 32,
    NvDdkAesKeySize_Num,
    NvDdkAesKeySize_Force32 = 0x7FFFFFFF
} NvDdkAesKeySize;

/**
 * AES key types.
 */

typedef enum
{
    NvDdkAesKeyType_Invalid,

    /**
     * Secure Boot Key (SBK) is used to protect boot-related data when the chip
     * is in ODM Production Mode.
     */
    NvDdkAesKeyType_SecureBootKey,

    /**
     * Secure Storage Key (SSK) is unique per chip, unless explicitly over-ridden; it is
     * intended for data that needs to be tied to a specific instance of the chip.
     */
    NvDdkAesKeyType_SecureStorageKey,

    /**
     * User-Specified Key is an arbitrary key supplied explicitly by the caller.
     */
    NvDdkAesKeyType_UserSpecified,
    NvDdkAesKeyType_Num,
    NvDdkAesKeyType_Force32 = 0x7FFFFFFF
} NvDdkAesKeyType;

/**
 * Supported cipher algorithms.
 */

typedef enum
{
    NvDdkAesOperationalMode_Invalid,

    /**
     * AES with Cipher Block Chaining (CBC)
     * for AES spec, see FIPS Publication 197
     * for CBC spec, see NIST Special Publication 800-38A
     */
    NvDdkAesOperationalMode_Cbc,

    /**
     * AES ANSIX931 RNG.
     * Algorithm is specified in the document "NIST-Recommended Random Number
     * Generator Based on ANSI X9.31 Appendix A.2.4 Using the 3-Key Triple DES and
     * AES Algorithms," Sharon S. Keller, January 11, 2005.
     * Input vector is specified by user.
     */
    NvDdkAesOperationalMode_AnsiX931,

    /**
     * AES with Electronic Codebook (ECB)
     * for AES spec, see FIPS Publication 197
     * for ECB spec, see NIST Special Publication 800-38A
     */
    NvDdkAesOperationalMode_Ecb,
    NvDdkAesOperationalMode_Num,
    NvDdkAesOperationalMode_Force32 = 0x7FFFFFFF
} NvDdkAesOperationalMode;

/**
 * Key argument.
 */

typedef struct NvDdkAesKeyInfoRec
{

    /**
     * Select key type -- SBK, SSK, user-specified.
     */
    NvDdkAesKeyType KeyType;

    /**
     * Length of key. Ignored unless key type is user-specified.
     */
    NvDdkAesKeySize KeyLength;

    /**
     * Key value. Ignored unless key type is user-specified.
     */
    NvU8 Key[32];

    /**
     * NV_TRUE if key must be assigned a dedicated key slot, else NV_FALSE.
     * Assigned key slot may be time-multiplexed if IsDedicateKeySlot is not
     * set to NV_TRUE.
     */
    NvBool IsDedicatedKeySlot;
} NvDdkAesKeyInfo;

/**
 * NvDdkAesOperation argument.
 */

typedef struct NvDdkAesOperationRec
{

    /**
     * Block cipher mode of operation.
     */
    NvDdkAesOperationalMode OpMode;

    /**
     * Select NV_TRUE to perform an encryption operation, NV_FALSE to perform a
     * decryption operation.
     *
     * @note IsEncrypt will be ignored when AES operation mode is
     * NvDdkAesOperationalMode_AnsiX931.
     */
    NvBool IsEncrypt;
} NvDdkAesOperation;

/**
 * NvDdkAesCapabilities argument.
 */

typedef struct NvDdkAesCapabilitiesRec
{

    /**
     * Buffer alignment. For optimal performance, ensure that source and
     * destination buffers for cipher operations are aligned such that
     * (pBuffer % OptimalBufferAlignment) == 0 where pBuffer is the start
     * address of the buffer.
     */
    NvU32 OptimalBufferAlignment;
} NvDdkAesCapabilities;

#if defined(__cplusplus)
}
#endif

#endif
