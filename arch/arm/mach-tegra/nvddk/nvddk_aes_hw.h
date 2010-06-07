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

#ifndef INCLUDED_NVDDK_AES_HW_H
#define INCLUDED_NVDDK_AES_HW_H

#include "nverror.h"
#include "nvcommon.h"

#ifndef NV_OAL
#define NV_OAL 0
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * The key table length is 64 bytes.
 * (This includes first upto 32  bytes key + 16 bytes original initial
 * vector and 16 bytes updated initial vector)
 */
enum {AES_HW_KEY_TABLE_LENGTH_BYTES = 64};

/**
 * Keytable array, this must be in the IRAM.
 */
enum {AES_HW_KEY_TABLE_ADDR_ALIGNMENT = 256};

/**
 * The key Table length is 256 bytes = 64 words, (32 bit each word).
 * (This includes first 16 bytes key + 224 bytes
 * Key expantion data + 16 bytes Initial vector)
 */
enum {AES_HW_KEY_SCHEDULE_LENGTH = 64};

/**
 * The key Table length is 64 bytes = 16 words, (32 bit each word).
 */
enum {AES_HW_KEY_TABLE_LENGTH = 16};

#if NV_OAL
/*
 * Key table address must be in the IRAM.
 */
enum {NVBL_AES_KEY_TABLE_ADDR = 0x40001d00};

/**
 * DMA Buffer size for processing the encryption and decryption with H/W.
 * Each engine is allocated 4 Kilo Bytes buffer for data processing.
 * This buffer is shared for both input data and out put data.
 */
enum {AES_HW_DMA_BUFFER_SIZE_BYTES = 0x1000};

#else

/*
 * Key table address must be in the IRAM.
 */
enum {NVBL_AES_KEY_TABLE_OFFSET = 0x1d00};

/**
 * DMA Buffer size for processing the encryption and decryption with H/W.
 * Each engine is allocated 32 Kilo Bytes buffer for data processing.
 * This buffer is shared for both input data and out put data.
 */
enum {AES_HW_DMA_BUFFER_SIZE_BYTES = 0x8000};

/**
 * Define AES engine Max process bytes size in one go, which takes 1 msec.
 * AES engine spends about 176 cycles/16-bytes or 11 cycles/byte
 * The duration CPU can use the BSE to 1 msec, then the number of available
 * cycles of AVP/BSE is 216K. In this duration, AES can process 216/11 ~= 19 KBytes
 * Based on this AES_HW_MAX_PROCESS_SIZE_BYTES is configured to 16KB.
 */
enum {AES_HW_MAX_PROCESS_SIZE_BYTES = 0x4000};

#endif // NV_OAL

/**
 * DMA data buffer address alignment.
 */
enum {AES_HW_DMA_ADDR_ALIGNMENT = 16};

/**
 * The Initial Vector length in the 32 bit words. (128 bits = 4 words)
 */
enum {AES_HW_IV_LENGTH = 4};

/**
 * The Initial Vector length in the 32 bit words. (128 bits = 4 words)
 */
enum {AES_HW_KEY_LENGTH = 4};

/**
 * Define AES block length in the 32 bit words (128-bits = 4 words)
 */
enum {AES_HW_BLOCK_LENGTH = 4};

/**
 * Define AES block length in log2 bytes = 2^4 = 16 bytes.
 */
enum {AES_HW_BLOCK_LENGTH_LOG2 = 4};

/**
 * AES Key (128 bits).
 */
typedef struct AesHwKeyRec
{
    NvU32 key[AES_HW_KEY_LENGTH];
} AesHwKey;

/**
 * AES Initial Vector (128 bits).
 */
typedef struct AesHwIvRec
{
    NvU32 iv[AES_HW_IV_LENGTH];
} AesHwIv;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_AES_HW_H
