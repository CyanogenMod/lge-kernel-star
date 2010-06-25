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

#ifndef INCLUDED_nvddk_aes_H
#define INCLUDED_nvddk_aes_H


#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvddk_aes_common.h"

/** @file
 * @brief <b>NVIDIA Driver Development Kit:
 *           NvDdkAes APIs</b>
 *
 * @b Description: Declares Interface for NvDdkAes APIs.
 */

/**
 * @defgroup nvddk_aes Advanced Encryption Standard API
 *
 *
 * @ingroup nvddk_modules
 * @{
 */

/**
 *  NvDdkAesHandle is an opaque handle to the AES engine.
 */

typedef struct NvDdkAesRec *NvDdkAesHandle;

/**
 * @brief Initialize and open the AES engine.
 *
 * This function allocates the handle for the AES engine and provides it to the
 * client.
 *
 * @param InstanceId Instance of AES to be opened.
 * @param phAes Pointer to the location where the AES handle shall be stored.
 *
 * @retval NvSuccess Indicates that the AES has successfully opened.
 * @retval NvError_InsufficientMemory Indicates that function fails to allocate
 *         the memory.
 * @retval NvError_NotInitialized Indicates the AES initialization failed.
 */

 NvError NvDdkAesOpen(
    NvU32 InstanceId,
    NvDdkAesHandle * phAes );

/**
 * @brief Close the AES engine.
 *
 * This function frees the memory allocated for the AES handle and de-initializes the AES.
 * This API never fails.
 *
 * @param hAes A handle from NvDdkAesOpen(). If hAes is NULL, this API does nothing.
 */

 void NvDdkAesClose(
    NvDdkAesHandle hAes );

/**
 * @brief Select key for cipher operations.
 *
 * Select key value for use in subsequent AES cipher operations. Caller can
 * select one of the pre-defined keys (SBK, SSK, zeroes) or provide an explicit
 * key value.
 *
 * @note NvDdkAesSelectOperation must be called before calling NvDdkAesSelectKey.
 *
 * @param hAes Handle to the AES.
 * @param pKeyInfo Pointer to key attribute.
 *
 * @retval NvError_Success Key Operation is successful.
 * @retval NvError_BadParameter Key type or length is invalid.
 * @retval NvError_InvalidState if AES engine is disabled / Operating mode not set yet.
 */

 NvError NvDdkAesSelectKey(
    NvDdkAesHandle hAes,
    const NvDdkAesKeyInfo * pKeyInfo );

/**
 * @brief Select cipher operation to perform.
 *
 * @param hAes Handle to the AES.
 * @param pOperation Pointer to operation attribute.
 *
 * @retval NvError_Success Cipher Operation is successful.
 * @retval NvError_BadParameter Invalid operand value.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesSelectOperation(
    NvDdkAesHandle hAes,
    const NvDdkAesOperation * pOperation );

/**
 * @brief Set initial vector (IV) for cipher operation.
 *
 * @param hAes Handle to the AES.
 * @param pInitialVector Pointer to initial vector attribute.
 * @param VectorSize Size of vector in bytes.
 *
 * @retval NvError_Success Initial Operation is successful.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesSetInitialVector(
    NvDdkAesHandle hAes,
    const NvU8 * pInitialVector,
    NvU32 VectorSize );

/**
 * @brief Get initial vector (IV) for cipher operation.
 *
 * @param hAes Handle to the AES.
 * @param VectorSize Size of buffer pointed by pInitialVector (in bytes).
 * @param pInitialVector Pointer to initial vector attribute.
 *
 * @retval NvError_Success Initial Operation is successful.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesGetInitialVector(
    NvDdkAesHandle hAes,
    NvU32 VectorSize,
    NvU8 * pInitialVector );

/**
 * @brief Process specified source data buffer using selected operation, cipher key,
 * and initial vector.
 *
 * Store processed data in destination buffer.
 *
 * @note NvDdkAesSelectOperation must be called before calling NvDdkAesProcessBuffer.
 * @note pSrcBuffer and pDestBuffer must be of same size.
 * @note SrcBufferSize and DestBufferSize must be multiple of NvDdkAesConst_BlockLengthBytes.
 *
 * @param hAes Handle to the AES.
 * @param SrcBufferSize Size of src buffer in bytes.
 * @param DestBufferSize Size of dest buffer in bytes.
 * @param pSrcBuffer Pointer to src buffer.
 * @param pDestBuffer Pointer to dest buffer.
 *
 * @retval NvError_Success Crypto Operation is successful.
 * @retval NvError_InvalidSize BufferSize is not multiple of NvDdkAesConst_BlockLengthBytes.
 * @retval NvError_InvalidAddress Illegal buffer pointer (NULL).
 * @retval NvError_BadValue Other illegal parameter value.
 * @retval NvError_InvalidState Key and/or operating mode not set yet.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesProcessBuffer(
    NvDdkAesHandle hAes,
    NvU32 SrcBufferSize,
    NvU32 DestBufferSize,
    const NvU8 * pSrcBuffer,
    NvU8 * pDestBuffer );

/**
 * @brief Get hardware capabilities for cipher operations.
 *
 * @param hAes Handle to the AES.
 * @param pCapabilities Pointer to capabilities attribute.
 *
 * @retval NvError_Success Capabilities query is successful.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesGetCapabilities(
    NvDdkAesHandle hAes,
    NvDdkAesCapabilities * pCapabilities );

/**
 * @brief Overwrite Secure Boot Key (SBK) in AES key slot with zeroes.
 *
 * After this operation has been completed, the SBK value will no longer be
 * accessible for any operations until after the system is rebooted. Read
 * access to the AES key slot containing the SBK is always disabled.
 *
 * @param hAes Handle to the AES.
 *
 * @retval NvError_Success Secure Boot Key cleared successfully.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesClearSecureBootKey(
    NvDdkAesHandle hAes );

/**
 * @brief Disable write access to AES key slot containing the Secure Storage Key (SSK).
 *
 * Prior to this operation, write access is enabled to the AES key slot
 * containing the SSK. This allows the OEM to override the default SSK
 * value.
 *
 * After this operation has been completed, the SSK value will still remain
 * available for encrypt and decrypt operations, but read access to the key
 * will be disabled (until the system is rebooted).
 *
 * @note Write access to the key slot containing the SSK is always disabled.
 *
 * @param hAes Handle to the AES.
 *
 * @retval NvError_Success Secure Storage Key locked successfully.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesLockSecureStorageKey(
    NvDdkAesHandle hAes );

/**
 * @brief Override Secure Storage Key (SSK) value in AES key slot, then disable
 * write access to the key slot.
 *
 * After this operation has been completed, the SSK value will still remain
 * available for encrypt and decrypt operations, but read access to the key
 * will be disabled (until the system is rebooted).
 *
 * @note Write access to the key slot containing the SSK is always disabled.
 *
 * @param hAes Handle to the AES.
 * @param hAes Length of key in bytes.
 * @param pSecureStorageKey Pointer to key argument.
 *
 * @retval NvError_Success Secure Storage Key overridden and locked successfully.
 * @retval NvError_InvalidState if AES engine is disabled.
 */

 NvError NvDdkAesSetAndLockSecureStorageKey(
    NvDdkAesHandle hAes,
    NvDdkAesKeySize KeyLength,
    const NvU8 * pSecureStorageKey );

/**
 * @brief Disable all AES operations until the system is rebooted.
 *
 * @param hAes Handle to the AES.
 *
 * @retval NvError_Success AES operations disabled successfully.
 * @retval NvError_InvalidState If AES engine is already disabled.
 */

 NvError NvDdkAesDisableCrypto(
    NvDdkAesHandle hAes );

#ifdef CONFIG_PM
/**
 * @brief This function waits till AES engine completes its operation if any thing is already running.
 *
 * @param none.
 *
 * @retval none
 */

 void NvDdkAesSuspend(void);

/**
 * @brief This function restores dedicated key slot information back in the hardware by unwrapping
 * the key using RFC-3394 key unwrapping.
 *
 * @param none.
 *
 * @retval none
 */

 void NvDdkAesResume(void);
#endif

#if defined(__cplusplus)
}
#endif

#endif
