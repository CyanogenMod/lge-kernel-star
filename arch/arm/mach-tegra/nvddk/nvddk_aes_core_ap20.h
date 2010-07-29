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

#ifndef INCLUDED_NVDDK_AES_CORE_AP20_H
#define INCLUDED_NVDDK_AES_CORE_AP20_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Disable the selected AES engine.  No further operations can be
 * performed using the AES engine until the entire chip is reset.
 *
 * @param Engine AES engine to disable.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval NvSuccess if engine successfully disabled else NvError_AesDisableCryptoFailed.
 */
NvError NvAesCoreAp20DisableEngine(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr);

/**
 * Read the AES engine disable status.
 *
 * @param Engine AES engine to disable.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval NV_TRUE if engine is disabled else NV_FALSE.
 */
NvBool NvAesCoreAp20IsEngineDisabled(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr);

/**
 * Wait till the engine is idle.
 *
 * @param Engine The engine which needs to be in idle state.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval None.
 */
void NvAesCoreAp20WaitTillEngineIdle(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr);

/**
 * Set up the key table.
 *
 * @param Engine The AES engine to setup the Key table.
 * @param pEngineVirAddr AES engine virtual address.
 * @param KeyTablePhyAddr The physical address of the keytable.
 * @param Slot AES Key slot to use for setting up the key table.
 *
 * @retval None.
 */
void
NvAesCoreAp20SetupTable(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 KeyTablePhyAddr,
    const NvU32 Slot);

/**
 * Select the Key slot for updating the IV vectors.
 *
 * @param Engine The AES engine to be used.
 * @param pEngineVirAddr AES engine virtual address.
 * @param Slot The slot to be selected.
 *
 * @retval None.
 */
void NvAesCoreAp20SelectKeyIvSlot(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr, const NvU32 Slot);

/**
 * Set the IV in key table.
 *
 * @param Engine The AES engine to be used.
 * @param pEngineVirAddr AES engine virtual address.
 * @param Slot The slot of the engine to be used.
 * @param Start The start location within the keytable.
 * @param End The end location of keytable.
 * @param pKeyTable The physical address of the engine keytable.
 * @param KeyTableSize Size of key table in bytes.
 * @param pIvAddress The IV to set, if NULL then the IV is cleared.
 *
 * @retval None.
 */
void
NvAesCoreAp20SetIv(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 Slot,
    const NvU32 Start,
    const NvU32 End,
    const NvU32 *const pKeyTable,
    const NvU32 KeyTableSize,
    const NvU32 *const pIvAddress);

/**
 * Get the IV.
 *
 * @param Engine The AES engine to be used.
 * @param pEngineVirAddr AES engine virtual address.
 * @param Slot The slot of the engine to be used.
 * @param Start The start location within the keytable.
 * @param End The end location of keytable.
 * @param pIvAddress The pointer to the location where the IV will be stored.
 *
 * @retval None.
 */
void
NvAesCoreAp20GetIv(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 Slot,
    const NvU32 Start,
    const NvU32 End,
    const NvU32 *const pIvAddress);

/**
 * Enable/Disable the key schedule generation in hardware.
 *
 * @param Engine The engine for which the key generation needs to be disabled.
 * @param pEngineVirAddr AES engine virtual address.
 * @param IsEnabled NV_TRUE to enable the key schedule generation, NV_FALSE otherwise.
 *
 * @retval None.
 */
void
NvAesCoreAp20ControlKeyScheduleGeneration(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvBool IsEnabled);

/**
 * Lock down the permissions for SSK.
 *
 * @param Engine The engine which needs to be locked.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval None.
 */
void NvAesCoreAp20LockSskReadWrites(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr);

/**
 * Encrypt/Decrypt blocks of data.
 *
 * @param Engine The engine to be used.
 * @param pEngineVirAddr AES engine virtual address.
 * @param SrcPhyAddress The physical address of source buffer.
 * @param DestPhyAddress The physical address of destination buffer.
 * @param NumBlocks Number of blocks in Source buffer
 * @param IsEncryption NV_TRUE if encryption else NV_FALSE.
 * @param OpMode Specifies the AES operational mode.
 *
 * @retval None.
 */
void
NvAesCoreAp20ProcessBuffer(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 SrcPhyAddress,
    const NvU32 DestPhyAddress,
    const NvU32 NumBlocks,
    const NvBool IsEncryption,
    const NvDdkAesOperationalMode OpMode);

/**
 * Load the SSK key into secure scratch resgister and disables the write permissions.
 *
 * @param pPmicBaseAddr Pointer to the PMIC base address.
 * @param pKey Pointer to the key.
 * @param Size Length of the aperture in bytes.
 *
 * @retval None.
 */
void
NvAesCoreAp20LoadSskToSecureScratchAndLock(
    const NvU32 PmicBaseAddr,
    const NvU32 *const pKey,
    const size_t Size);

/**
 * Disables read access to the given key slot
 *
 * @param Engine AES engine for which read access needs to be disabled
 *               for the given key slot
 * @param Slot Key slot number for which read access needs to be disabled.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval None
 */
void
NvAesCoreAp20KeyReadDisable(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const NvU32 *const pEngineVirAddr);

/**
 * Queries whether SSK update is allowed or not
 *
 * @retval NV_TRUE if SSK update is allowed
 * @retval NV_FALSE if SSK update is not allowed
 */
NvBool NvAesCoreAp20IsSskUpdateAllowed(void);

#ifdef __cplusplus
};
#endif

#endif // #define INCLUDED_NVDDK_AES_CORE_AP20_H
