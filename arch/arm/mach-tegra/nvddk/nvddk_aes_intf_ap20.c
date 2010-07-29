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

#include "nvos.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvrm_init.h"
#include "ap20/arvde_bsev_aes.h"
#include "ap20/aravp_bsea_aes.h"
#include "nvddk_aes_common.h"
#include "nvddk_aes_priv.h"
#include "nvddk_aes_core_ap20.h"

/**
 * SBK and SSK settings.
 */
enum {AES_SBK_ENGINE_A = AesHwEngine_A};
enum {AES_SBK_ENGINE_B = AesHwEngine_B};
enum {AES_SBK_ENCRYPT_SLOT = AesHwKeySlot_0};
enum {AES_SBK_DECRYPT_SLOT = AES_SBK_ENCRYPT_SLOT};

enum {AES_SSK_ENGINE_A = AesHwEngine_A};
enum {AES_SSK_ENGINE_B = AesHwEngine_B};
enum {AES_SSK_ENCRYPT_SLOT = AesHwKeySlot_4};
enum {AES_SSK_DECRYPT_SLOT = AES_SSK_ENCRYPT_SLOT};

static void
Ap20AesSetupTable(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt);
static void
Ap20AesHwSelectKeyIvSlot(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt);
static void
Ap20AesHwClearKeyAndIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt);
static void
Ap20AesHwClearIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt);
static void
Ap20AesHwGetIv(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwIv *const pIv);
static void
Ap20AesHwLockSskReadWrites(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine SskEngine);
static void
Ap20AesHwLoadSskToSecureScratchAndLock(
    const NvRmPhysAddr PmicBaseAddr,
    const AesHwKey *const pKey,
    const size_t Size);
static void Ap20AesHwGetUsedSlots(AesCoreEngine *const pAesCoreEngine);
static void
Ap20AesHwSetKeyAndIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const AesHwKey *const pKey,
    const AesHwIv *const pIv,
    const NvBool IsEncryption,
    AesHwContext *const pAesHwCtxt);

static NvBool Ap20AesHwIsEngineDisabled(const AesHwContext *const pAesHwCtxt, const AesHwEngine Engine);

static NvError Ap20AesHwDisableEngine(const AesHwContext *const pAesHwCtxt, const AesHwEngine Engine);
static NvError
Ap20AesHwStartEngine(
    const AesHwEngine Engine,
    const NvU32 DataSize,
    const NvU8 *const pSrc,
    const NvBool IsEncryption,
    const NvDdkAesOperationalMode OpMode,
    NvU8 *const pDest,
    AesHwContext *const pAesHwCtxt);
static NvError
Ap20AesHwSetIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const AesHwIv *const pIv,
    AesHwContext *const pAesHwCtxt);
static void
Ap20AesHwDisableAllKeyRead(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine Engine,
    const AesHwKeySlot NumSlotsSupported);
static NvBool Ap20AesIsSskUpdateAllowed(void);

/**
 * Set the Setup Table command required for the AES engine.
 *
 * @param Engine AES engine to setup the Key table.
 * @param Slot AES Key slot to use for setting up the key table.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval None.
 */
void
Ap20AesSetupTable(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);

    NvAesCoreAp20SetupTable(Engine, pAesHwCtxt->pVirAdr[Engine], pAesHwCtxt->KeyTablePhyAddr[Engine], Slot);

    NvOsMemcpy(&pAesHwCtxt->IvContext[Engine].CurIv[Slot],
        (void *)(&pAesHwCtxt->pKeyTableVirAddr[Engine][AES_HW_KEY_TABLE_LENGTH - AES_HW_IV_LENGTH]),
        NvDdkAesConst_BlockLengthBytes);

    // Clear key table in the memory after updating the H/W
    NvOsMemset(pAesHwCtxt->pKeyTableVirAddr[Engine], 0, pAesHwCtxt->KeyTableSize[Engine]);
}

/**
 * Select the key and iv from the internal key table for a specified key slot.
 *
 * @param Engine AES engine.
 * @param Slot Key slot for which key and IV needs to be selected.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval None.
 */
void
Ap20AesHwSelectKeyIvSlot(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    // Wait till engine becomes IDLE
    NvAesCoreAp20WaitTillEngineIdle(Engine, pAesHwCtxt->pVirAdr[Engine]);

    // Select the KEY slot for updating the IV vectors
    NvAesCoreAp20SelectKeyIvSlot(Engine, pAesHwCtxt->pVirAdr[Engine], Slot);

    pAesHwCtxt->IvContext[Engine].CurKeySlot = Slot;

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);
}

/**
 * Disable the selected AES engine.  No further operations can be
 * performed using the AES engine until the entire chip is reset.
 *
 * @param pAesHwCtxt Pointer to the AES H/W context.
 * @param Engine AES engine to disable.
 *
 * @retval NvSuccess if engine successfully disabled else NvError_AesDisableCryptoFailed.
 */
NvError Ap20AesHwDisableEngine(const AesHwContext *const pAesHwCtxt, const AesHwEngine Engine)
{
    NvError e;

    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    // Wait till engine becomes IDLE
    NvAesCoreAp20WaitTillEngineIdle(Engine, pAesHwCtxt->pVirAdr[Engine]);

    e = NvAesCoreAp20DisableEngine(Engine, pAesHwCtxt->pVirAdr[Engine]);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);

    return e;
}

/**
 * Over-write the key schedule and Initial Vector in the in the specified
 * key slot with zeroes. Convenient for preventing subsequent callers from
 * gaining access to a previously-used key.
 *
 * @param Engine AES engine.
 * @param Slot key slot to clear.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval None.
 */
void
Ap20AesHwClearKeyAndIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    // Wait till engine becomes IDLE
    NvAesCoreAp20WaitTillEngineIdle(Engine, pAesHwCtxt->pVirAdr[Engine]);

    // Clear Key table this clears both Key Schedule and IV in key table
    NvOsMemset(pAesHwCtxt->pKeyTableVirAddr[Engine], 0, pAesHwCtxt->KeyTableSize[Engine]);

    // Setup the key table with Zero key and Zero Iv
    Ap20AesHwSelectKeyIvSlot(Engine, Slot, pAesHwCtxt);
    Ap20AesSetupTable(Engine, Slot, pAesHwCtxt);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);
}

/**
 * Over-write the initial vector in the specified AES engine with zeroes.
 * Convenient to prevent subsequent callers from gaining access to a
 * previously-used initial vector.
 *
 * @param Engine AES engine.
 * @param Slot Key slot for which Iv needs to be cleared.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval None.
 */
void
Ap20AesHwClearIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);
    NV_ASSERT(pAesHwCtxt->pKeyTableVirAddr[Engine]);

    NvOsMemset((void *)pAesHwCtxt->pKeyTableVirAddr[Engine], 0, NvDdkAesConst_IVLengthBytes);

    Ap20AesHwStartEngine(
        Engine,
        NvDdkAesConst_IVLengthBytes,
        pAesHwCtxt->pKeyTableVirAddr[Engine],
        NV_FALSE,
        NvDdkAesOperationalMode_Cbc,
        pAesHwCtxt->pKeyTableVirAddr[Engine],
        pAesHwCtxt);
}

/**
 * Compute key schedule for the given key, then load key schedule and
 * initial vector into the specified key slot.
 *
 * @param Engine AES engine.
 * @param Slot Key slot to load.
 * @param pKey Pointer to the key.
 * @param pIv Pointer to the iv.
 * @param IsEncryption If set to NV_TRUE indicates key schedule computation
 *        is for encryption else for decryption.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval None.
 */
void
Ap20AesHwSetKeyAndIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const AesHwKey *const pKey,
    const AesHwIv *const pIv,
    const NvBool IsEncryption,
    AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);
    NV_ASSERT(pKey);
    NV_ASSERT(pIv);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    // Wait till engine becomes IDLE
    NvAesCoreAp20WaitTillEngineIdle(Engine, pAesHwCtxt->pVirAdr[Engine]);

    // Disable read access to the key slot
    NvAesCoreAp20KeyReadDisable(Engine, Slot, pAesHwCtxt->pVirAdr[Engine]);

    NvAesCoreAp20ControlKeyScheduleGeneration(Engine, pAesHwCtxt->pVirAdr[Engine], NV_TRUE);

    Ap20AesHwSelectKeyIvSlot(Engine, Slot, pAesHwCtxt);
    // Clear key table first before expanding the Key
    NvOsMemset(pAesHwCtxt->pKeyTableVirAddr[Engine], 0, AES_HW_KEY_TABLE_LENGTH_BYTES);
    NvOsMemcpy(&pAesHwCtxt->pKeyTableVirAddr[Engine][0], &pKey->key[0], sizeof(AesHwKey));

    NvOsMemcpy(
        &pAesHwCtxt->pKeyTableVirAddr[Engine][NvDdkAesConst_MaxKeyLengthBytes + NvDdkAesConst_IVLengthBytes],
        &pIv->iv[0],
        sizeof(AesHwIv));

    Ap20AesSetupTable(Engine, Slot, pAesHwCtxt);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);
}

/**
 * Load an initial vector into the specified AES engine for using it
 * during encryption or decryption.
 *
 * @param Engine AES engine.
 * @param Slot Key slot for which Iv needs to be set.
 * @param pIv Pointer to the initial vector.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval NvSuccess if Iv was set correctly.
 *         NvError_InvalidState if operation is not allowed.
 */
NvError
Ap20AesHwSetIv(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const AesHwIv *const pIv,
    AesHwContext *const pAesHwCtxt)
{
    NvError e;

    NV_ASSERT(pAesHwCtxt);
    NV_ASSERT(pIv);

    e = Ap20AesHwStartEngine(
        Engine,
        NvDdkAesConst_IVLengthBytes,
        (NvU8 *)(&pIv->iv[0]),
        NV_FALSE,
        NvDdkAesOperationalMode_Cbc,
        pAesHwCtxt->pKeyTableVirAddr[Engine],
        pAesHwCtxt);

    NV_ASSERT(pAesHwCtxt->pKeyTableVirAddr[Engine]);

    NvOsMemset((void *)pAesHwCtxt->pKeyTableVirAddr[Engine], 0, AES_HW_KEY_TABLE_LENGTH);

    return e;
}

/**
 * Retrieve the initial vector for the specified AES engine.
 *
 * @param pAesHwCtxt Pointer to the AES H/W context.
 * @param Engine AES engine.
 * @param Slot Key slot for which Iv is to be retrieved.
 * @param pIv Pointer to the initial vector.
 *
 * @retval None.
 */
void
Ap20AesHwGetIv(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    AesHwIv *const pIv)
{
    NV_ASSERT(pAesHwCtxt);
    NV_ASSERT(pIv);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    NvOsMemcpy(&pIv->iv[0], &pAesHwCtxt->IvContext[Engine].CurIv[Slot], NvDdkAesConst_BlockLengthBytes);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);
}

/**
 * Lock the Secure Session Key (SSK) slots.
 * This API disables the read/write permissions to the secure key slots.
 *
 * @param pAesHwCtxt Pointer to the AES H/W context.
 * @param SskEngine SSK engine number.
 *
 * @retval None.
 */
void
Ap20AesHwLockSskReadWrites(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine SskEngine)
{
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[SskEngine]);

    NvAesCoreAp20LockSskReadWrites(SskEngine, pAesHwCtxt->pVirAdr[SskEngine]);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[SskEngine]);
}

/**
 * Encrypt/Decrypt a specified number of blocks of data. A block is 16 bytes.
 * This is non-blocking API and need to call AesHwEngineIdle()
 * to check the engine status to confirm the AES engine operation is
 * done and comes out of the BUSY state.
 * Also make sure before calling this API engine must be IDLE.
 *
 * @param Engine AES engine.
 * @param DataSize Number of blocks of ciphertext to process.
 *        One block is 16 bytes. Max number of blocks possible = 0xFFFFF.
 * @param pSrc Pointer to nblock blocks of ciphertext/plaintext depending on the
 *        IsEncryption status; ciphertext/plaintext is not modified (input).
 * @param IsEncryption If set to NV_TRUE indicates AES engine to start
 *        encryption on the source data to give cipher text else starts
 *        decryption on the source cipher data to give plain text.
 * @param OpMode Specifies the AES operational mode.
 * @param pDest Pointer to nblock blocks of cleartext/ciphertext (output)
 *        depending on the IsEncryption.
 * @param pAesHwCtxt Pointer to the AES H/W context.
 *
 * @retval NvSuccess if AES operation is successful.
 * @retval NvError_InvalidState if operation mode is not supported.
 */
NvError
Ap20AesHwStartEngine(
    const AesHwEngine Engine,
    const NvU32 DataSize,
    const NvU8 *const pSrc,
    const NvBool IsEncryption,
    const NvDdkAesOperationalMode OpMode,
    NvU8 *const pDest,
    AesHwContext *const pAesHwCtxt)
{
    NvU32 TotalBytes = DataSize;
    NvU32 NumBlocks = 0;
    NvU32 BytesToProcess = 0;
    NvU8 *pSourceBuffer = (NvU8 *)pSrc;
    NvU8 *pDestBuffer = pDest;

    NV_ASSERT(pAesHwCtxt);
    NV_ASSERT(pSrc);
    NV_ASSERT(pDest);

    switch (OpMode)
    {
        case NvDdkAesOperationalMode_AnsiX931:
        case NvDdkAesOperationalMode_Cbc:
        case NvDdkAesOperationalMode_Ecb:
            break;
        default:
            return NvError_InvalidState;
    }

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    if (DataSize && (!IsEncryption) && (OpMode == NvDdkAesOperationalMode_Cbc))
    {
        NvOsMemcpy(&pAesHwCtxt->IvContext[Engine].CurIv[pAesHwCtxt->IvContext[Engine].CurKeySlot],
            (pSrc + DataSize - NvDdkAesConst_BlockLengthBytes),
            NvDdkAesConst_BlockLengthBytes);
    }

    while (TotalBytes)
    {
        if (TotalBytes > AES_HW_DMA_BUFFER_SIZE_BYTES)
        {
            BytesToProcess = AES_HW_DMA_BUFFER_SIZE_BYTES;
        }
        else
        {
            BytesToProcess = TotalBytes;
        }

        // Copy data to DMA buffer from the client buffer
        NvOsMemcpy(pAesHwCtxt->pDmaVirAddr[Engine], (void *)pSourceBuffer, BytesToProcess);
        NvOsFlushWriteCombineBuffer();

        NumBlocks = BytesToProcess / NvDdkAesConst_BlockLengthBytes;

        NvAesCoreAp20ProcessBuffer(
            Engine,
            pAesHwCtxt->pVirAdr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            NumBlocks,
            IsEncryption,
            OpMode);
        NvOsFlushWriteCombineBuffer();

        // Copy data from DMA buffer to the client buffer
        NvOsMemcpy(pDestBuffer, pAesHwCtxt->pDmaVirAddr[Engine], BytesToProcess);

        // Increment the buffer pointer
        pSourceBuffer += BytesToProcess;
        pDestBuffer += BytesToProcess;
        TotalBytes -= BytesToProcess;
    }

    /**
     * If DataSize is zero, Iv would remain unchanged.
     * For an encryption operation, the current Iv will be the last block of
     * ciphertext.
     */
    if (DataSize && IsEncryption && (OpMode == NvDdkAesOperationalMode_Cbc))
    {
        NvOsMemcpy(&pAesHwCtxt->IvContext[Engine].CurIv[pAesHwCtxt->IvContext[Engine].CurKeySlot],
            (pDest + DataSize - NvDdkAesConst_BlockLengthBytes),
             NvDdkAesConst_BlockLengthBytes);
    }
    else if (DataSize && (OpMode == NvDdkAesOperationalMode_AnsiX931))
    {
        // For X931 operation, get the updated IV by following steps:
        // 1. Perform CBC encryption on zero data to get A=CBC(encrypt, plaintext=zeroes)
        // 2. Perform ECB decryption on A. This will result in Updated IV. UpdatedIV = ECB(decrypt, A)
        NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, NvDdkAesKeySize_128Bit);
        NvOsFlushWriteCombineBuffer();
        NvAesCoreAp20ProcessBuffer(
            Engine,
            pAesHwCtxt->pVirAdr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            1,
            NV_TRUE,
            NvDdkAesOperationalMode_Cbc);
        NvOsFlushWriteCombineBuffer();

        NvAesCoreAp20ProcessBuffer(
            Engine,
            pAesHwCtxt->pVirAdr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            pAesHwCtxt->DmaPhyAddr[Engine],
            1,
            NV_FALSE,
            NvDdkAesOperationalMode_Ecb);
        NvOsFlushWriteCombineBuffer();
        NvOsMemcpy(&pAesHwCtxt->IvContext[Engine].CurIv[pAesHwCtxt->IvContext[Engine].CurKeySlot],
            pAesHwCtxt->pDmaVirAddr[Engine],
            NvDdkAesConst_BlockLengthBytes);
    }

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);

    return NvSuccess;
}

/**
 * Load the SSK key into secure scratch resgister and disables the write permissions.
 * Note: If Key is not specified then this API locks the Secure Scratch registers.
 *
 * @param PmicBaseAddr PMIC base address.
 * @param pKey Pointer to the key. If pKey=NULL then key will not be set to the
 *             secure scratch registers, but locks the Secure scratch register.
 * @param Size Length of the aperture in bytes.
 *
 * @retval None.
 */
void
Ap20AesHwLoadSskToSecureScratchAndLock(
    const NvRmPhysAddr PmicBaseAddr,
    const AesHwKey *const pKey,
    const size_t Size)
{
    NV_ASSERT(pKey);
    NvAesCoreAp20LoadSskToSecureScratchAndLock(PmicBaseAddr, (pKey ? pKey->key : 0), Size);
}

/**
 * Mark all dedicated slots as used.
 *
 * @param pAesCoreEngine Pointer to AES Core Engine.
 *
 * @retval None.
 */
void Ap20AesHwGetUsedSlots(AesCoreEngine *const pAesCoreEngine)
{
    NV_ASSERT(pAesCoreEngine);

    // For ap20, SBK and SSK reside on both engines
    pAesCoreEngine->SbkEngine[0] = AES_SBK_ENGINE_A;
    pAesCoreEngine->SbkEncryptSlot = AES_SBK_ENCRYPT_SLOT;
    pAesCoreEngine->SbkDecryptSlot = AES_SBK_DECRYPT_SLOT;
    pAesCoreEngine->IsKeySlotUsed[AES_SBK_ENGINE_A][AES_SBK_ENCRYPT_SLOT] = NV_TRUE;

    pAesCoreEngine->SbkEngine[1] = AES_SBK_ENGINE_B;
    pAesCoreEngine->IsKeySlotUsed[AES_SBK_ENGINE_B][AES_SBK_DECRYPT_SLOT] = NV_TRUE;

    pAesCoreEngine->SskEngine[0] = AES_SSK_ENGINE_A;
    pAesCoreEngine->SskEncryptSlot = AES_SSK_ENCRYPT_SLOT;
    pAesCoreEngine->SskDecryptSlot = AES_SSK_DECRYPT_SLOT;
    pAesCoreEngine->IsKeySlotUsed[AES_SSK_ENGINE_A][AES_SSK_ENCRYPT_SLOT] = NV_TRUE;

    pAesCoreEngine->SskEngine[1] = AES_SSK_ENGINE_B;
    pAesCoreEngine->IsKeySlotUsed[AES_SSK_ENGINE_B][AES_SSK_DECRYPT_SLOT] = NV_TRUE;
}

/**
 * Read the AES engine disable status.
 *
 * @param pAesHwCtxt Pointer to the AES H/W context.
 * @param Engine AES engine to disable.
 *
 * @return NV_TRUE if engine is disabled else NV_FALSE.
 *
 */
NvBool Ap20AesHwIsEngineDisabled(const AesHwContext *const pAesHwCtxt, const AesHwEngine Engine)
{
    NvBool IsEngineDisabled = NV_FALSE;

    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);

    IsEngineDisabled = NvAesCoreAp20IsEngineDisabled(Engine, pAesHwCtxt->pVirAdr[Engine]);

    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);

    return IsEngineDisabled;
}

/**
 * Disables read access to all key slots for the given engine.
 *
 * @param pAesHwCtxt Pointer to the AES H/W context
 * @param Engine AES engine for which key reads needs to be disabled
 * @param NumSlotsSupported Number of key slots supported in the engine
 *
 * @retval None
 */
void
Ap20AesHwDisableAllKeyRead(
    const AesHwContext *const pAesHwCtxt,
    const AesHwEngine Engine,
    const AesHwKeySlot NumSlotsSupported)
{
    AesHwKeySlot Slot;
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(pAesHwCtxt->Mutex[Engine]);
    NvAesCoreAp20WaitTillEngineIdle(Engine, pAesHwCtxt->pVirAdr[Engine]);

    // Disable read access to key slots
    for(Slot = AesHwKeySlot_0; Slot < NumSlotsSupported; Slot++)
    {
        NvAesCoreAp20KeyReadDisable(Engine, Slot, pAesHwCtxt->pVirAdr[Engine]);
    }
    NvOsMutexUnlock(pAesHwCtxt->Mutex[Engine]);
}

/**
 * Queries whether SSK update is allowed or not
 *
 * @retval NV_TRUE if SSK update is allowed
 * @retval NV_FALSE if SSK update is not allowed
 */
NvBool Ap20AesIsSskUpdateAllowed(void)
{
    return NvAesCoreAp20IsSskUpdateAllowed();
}

void NvAesIntfAp20GetHwInterface(AesHwInterface *const pAp20AesHw)
{
    NV_ASSERT(pAp20AesHw);

    pAp20AesHw->AesHwDisableEngine = Ap20AesHwDisableEngine;
    pAp20AesHw->AesHwClearKeyAndIv = Ap20AesHwClearKeyAndIv;
    pAp20AesHw->AesHwClearIv = Ap20AesHwClearIv;
    pAp20AesHw->AesHwSetKeyAndIv = Ap20AesHwSetKeyAndIv;
    pAp20AesHw->AesHwSetIv = Ap20AesHwSetIv;
    pAp20AesHw->AesHwGetIv = Ap20AesHwGetIv;
    pAp20AesHw->AesHwLockSskReadWrites = Ap20AesHwLockSskReadWrites;
    pAp20AesHw->AesHwSelectKeyIvSlot = Ap20AesHwSelectKeyIvSlot;
    pAp20AesHw->AesHwStartEngine = Ap20AesHwStartEngine;
    pAp20AesHw->AesHwLoadSskToSecureScratchAndLock = Ap20AesHwLoadSskToSecureScratchAndLock;
    pAp20AesHw->AesHwGetUsedSlots = Ap20AesHwGetUsedSlots;
    pAp20AesHw->AesHwIsEngineDisabled = Ap20AesHwIsEngineDisabled;
    pAp20AesHw->AesHwDisableAllKeyRead = Ap20AesHwDisableAllKeyRead;
    pAp20AesHw->AesHwIsSskUpdateAllowed = Ap20AesIsSskUpdateAllowed;
}
