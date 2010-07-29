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
#include "nvrm_power.h"
#include "nvrm_hardware_access.h"
#include "nvrm_module.h"
#include "nvrm_xpc.h"
#if NV_OAL
#include "nvbl_virtual.h"
#endif
#include "nvddk_aes.h"
#include "nvddk_aes_priv.h"

#include <linux/interrupt.h>
#include <linux/proc_fs.h>

// RFC3394 key wrap block size is 64 bites which is equal to 8 bytes
#define AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES 8

// Number of RFC3394 key wrap blocks for 128 bit key
#define AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY 2

static void AesCoreRequestHwAccess(void);
static void AesCoreReleaseHwAccess(void);
static void AesCoreDeAllocateRmMemory(AesHwContext *const pAesHwCtxt);
static void AesCoreFreeUpEngine(AesCoreEngine *const pAesCoreEngine);
static void AesCorePowerUp(const AesCoreEngine *const pAesCoreEngine, const NvBool SetDfsBusyHints);
static void AesCorePowerDown(const AesCoreEngine *const pAesCoreEngine, const NvBool SetDfsBusyHints);
static void AesCoreDeInitializeEngineSpace(const AesHwContext *const pAesHwCtxt);

static NvError AesCoreAllocateRmMemory(AesHwContext *const pAesHwCtxt);
static NvError AesCoreLoadSskToSecureScratchAndLock(const AesHwContext *const pAesHwCtxt, const NvU8 *const pKey);
static NvError AesCoreDfsBusyHint(const NvRmDeviceHandle hRmDevice, const NvU32 PowerClientId, const NvBool IsDfsOn);
static NvError AesCoreInitializeEngineSpace(const NvRmDeviceHandle hRmDevice, AesHwContext *const pAesHwCtxt);
static NvError AesCoreInitEngine(const NvRmDeviceHandle hRmDevice);
static NvError AesCoreGetCapabilities(const NvRmDeviceHandle hRmDevice, AesHwCapabilities **const ppCaps);
static NvError AesCoreClearUserKey(const NvDdkAes *const pAesClient);
static NvError
AesCoreWrapKey(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pOrgKey,
    const NvU8 *const pOrgIv,
    NvU8 *const pWrappedKey,
    NvU8 *const pWrappedIv);
static NvError
AesCoreUnWrapKey(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pWrappedKey,
    const NvU8 *const pWrappedIv,
    NvU8 *const pOrgKey,
    NvU8 *const pOrgIv);
static NvError
AesCoreGetFreeSlot(
    const AesCoreEngine *const pAesCoreEngine,
    AesHwEngine *const pEngine,
    AesHwKeySlot *const pKeySlot);
static NvError
AesCoreSetKey(
    const NvDdkAesKeyType KeyType,
    const NvBool IsDedicatedSlot,
    const NvU8 *const pKeyData,
    NvDdkAes *const pAesClient);
static NvError
AesCoreProcessBuffer(
    const NvU32 SkipOffset,
    const NvU32 SrcBufferSize,
    const NvU32 DestBufferSize,
    NvDdkAes *const pAesClient,
    const NvU8 *pSrcBuffer,
    NvU8 *pDestBuffer);
static NvError
AesCoreEcbProcessBuffer(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pInputBuffer,
    const NvU32 BufSize,
    const NvBool IsEncrypt,
    NvU8 *const pOutputBuffer);

static NvBool
AesCoreIsUserKeyCleared(
    const AesHwEngine Engine,
    const AesHwKeySlot KeySlot,
    AesHwContext *const pAesHwCtxt);
static NvBool
AesCoreIsSbkCleared(
    const AesHwEngine Engine,
    const AesHwKeySlot EncryptSlot,
    const AesHwKeySlot DecryptSlot,
    AesHwContext *const pAesHwCtxt);
static NvBool
AesCoreIsSskLocked(
    const AesHwEngine Engine,
    const AesHwKeySlot EncryptSlot,
    const AesHwKeySlot DecryptSlot,
    AesHwContext *const pAesHwCtxt);

static AesCoreEngine *gs_pAesCoreEngine = NULL;
static NvOsMutexHandle gs_hAesCoreEngineMutex = {0};

// Original IV of size 8 byte which is used in RFC 3394 key wrap algorithm
static NvU8 gs_OriginalIV[AES_RFC_IV_LENGTH_BYTES] =
{
    0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6
};

extern NvRmDeviceHandle s_hRmGlobal;

#define NVDDK_AES_CHECK_INPUT_PARAMS(parm) \
    do \
    { \
        if ((!pAesClient) || (!parm)) \
            return NvError_BadParameter; \
    } while (0)

#define NVDDK_AES_CHECK_ROOT_PERMISSION \
    do \
    { \
        if ((0 != pAesClient->uid) || (0 != pAesClient->gid)) \
            return NvError_AesPermissionDenied; \
    } while (0)

#define NVDDK_AES_CHECK_USER_IDENTITY \
    do \
    { \
        if ((pAesClient->uid != current->cred->uid) || (pAesClient->gid != current->cred->gid)) \
            return NvError_AesPermissionDenied; \
    } while (0)

#define NVDDK_AES_CHECK_INTERFACE(ctxt, engine) \
    do \
    { \
        NV_ASSERT(ctxt); \
        NV_ASSERT(ctxt->ppEngineCaps); \
        NV_ASSERT(ctxt->ppEngineCaps[engine]); \
        NV_ASSERT(ctxt->ppEngineCaps[engine]->pAesInterf); \
    } while (0)

#define NVDDK_AES_CHECK_INTERFACE_FUNC(ctxt, engine, func) \
    do \
    { \
        NV_ASSERT(ctxt->ppEngineCaps[engine]->pAesInterf->func); \
    } while (0)

#ifdef CONFIG_PM
void NvDdkAesSuspend(void)
{
    if (gs_hAesCoreEngineMutex)
    {
        NvOsMutexLock(gs_hAesCoreEngineMutex);
        if (gs_pAesCoreEngine)
            AesCorePowerDown(gs_pAesCoreEngine, NV_FALSE);
        NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    }
}

void NvDdkAesResume(void)
{
    NvU8 UnWrappedRFCIv[AES_RFC_IV_LENGTH_BYTES];
    NvU8 Iv[NvDdkAesConst_IVLengthBytes];
    AesHwEngine Engine;
    AesHwKeySlot KeySlot;
    AesHwContext *pAesHwCtxt;
    NvError e;

    if ((!gs_hAesCoreEngineMutex) ||(!gs_pAesCoreEngine))
        return;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_FALSE);

    // Get the AES H/W context
    pAesHwCtxt = &gs_pAesCoreEngine->AesHwCtxt;
    NvOsMemset(Iv, 0, sizeof(Iv));

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwDisableAllKeyRead);
        pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwDisableAllKeyRead(
            pAesHwCtxt,
            Engine,
            pAesHwCtxt->ppEngineCaps[Engine]->NumSlotsSupported);
    }

    // Get the dedicated slot
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        for (KeySlot = AesHwKeySlot_0;
            KeySlot < (pAesHwCtxt->ppEngineCaps[Engine]->NumSlotsSupported);
            KeySlot++)
        {
            if ((gs_pAesCoreEngine->IsKeySlotUsed[Engine][KeySlot]) &&
                (gs_pAesCoreEngine->SbkEncryptSlot != KeySlot) &&
                (gs_pAesCoreEngine->SbkDecryptSlot != KeySlot) &&
                (gs_pAesCoreEngine->SskEncryptSlot != KeySlot) &&
                (gs_pAesCoreEngine->SskDecryptSlot != KeySlot))
            {
                AesCoreRequestHwAccess();

                // This is dedicated slot. Load key in slot
                e = (AesCoreUnWrapKey(
                    gs_pAesCoreEngine,
                    gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].WrappedKey,
                    gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].WrappedIv,
                    pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine],
                    UnWrappedRFCIv));
                if (e != NvSuccess)
                    goto fail;

                // Check whether the key unwrap is success or not by comparing
                // the unwrapped RFC IV with original RFC IV
                if (NvOsMemcmp(UnWrappedRFCIv, gs_OriginalIV, sizeof(gs_OriginalIV)))
                {
                    // Unwrap key failed
                    NV_ASSERT(!"fail to unwrap key");
                    goto fail;
                }

                // Setup Key table
                NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetKeyAndIv);
                pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetKeyAndIv(
                    Engine,
                    KeySlot,
                    (AesHwKey *)(pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine]),
                    (AesHwIv *)Iv,
                    gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].IsEncryption,
                    pAesHwCtxt);

                // Memset the local variable to zeros where the key is stored
                NvOsMemset(
                    (pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine]),
                    0,
                    NvDdkAesKeySize_128Bit);
                AesCoreReleaseHwAccess();
            }
        }
    }
    AesCorePowerDown(gs_pAesCoreEngine, NV_FALSE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return;
fail:
    AesCorePowerDown(gs_pAesCoreEngine, NV_FALSE);
    AesCoreReleaseHwAccess();
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
}
#endif

NvError NvDdkAesOpen(NvU32 InstanceId, NvDdkAesHandle *phAes)
{
    NvError e = NvSuccess;
    NvDdkAes *pAes = NULL;
    NvOsMutexHandle hMutex = NULL;

    if (!phAes)
       return NvError_BadParameter;

    // Create mutex (if not already done)
    if (NULL == gs_hAesCoreEngineMutex)
    {
        e = NvOsMutexCreate(&hMutex);
        if (NvSuccess != e)
            return e;
        if (0 != NvOsAtomicCompareExchange32((NvS32*)&gs_hAesCoreEngineMutex, 0, (NvS32)hMutex))
            NvOsMutexDestroy(hMutex);
    }

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Create client record
    pAes = NvOsAlloc(sizeof(NvDdkAes));
    if (!pAes)
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }
    NvOsMemset(pAes, 0, sizeof(NvDdkAes));

    if (NULL == gs_pAesCoreEngine)
    {
        // Init engine
        NV_CHECK_ERROR_CLEANUP(AesCoreInitEngine(s_hRmGlobal));
    }

    // Add client
    gs_pAesCoreEngine->OpenCount++;

    pAes->pAesCoreEngine = gs_pAesCoreEngine;

    // Store uid & gid
    pAes->uid = current->cred->uid;
    pAes->gid = current->cred->gid;

    *phAes = pAes;
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvSuccess;

fail:
    NvDdkAesClose(pAes);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

void NvDdkAesClose(NvDdkAesHandle hAes)
{
    NvDdkAes *pAesClient = (NvDdkAes *)hAes;

    if (!hAes)
        return;

    if ((pAesClient->uid != current->cred->uid) || (pAesClient->gid != current->cred->gid))
        return;

    if (pAesClient->pAesCoreEngine)
    {
        NvOsMutexLock(gs_hAesCoreEngineMutex);

        // Check if client is using USER key with dedicated slot then free the associated client slot
        if ((NV_TRUE == pAesClient->IsDedicatedSlot) &&
            (NvDdkAesKeyType_UserSpecified == pAesClient->KeyType))
        {
            AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);
            // Client is using USER key with dedicated slot. So
            // clear the key in hardware slot and free the associated client slot
            if (!AesCoreClearUserKey(pAesClient))
            {
                NV_ASSERT(!"Failed to clear User Key");
            }
            AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
            pAesClient->pAesCoreEngine->IsKeySlotUsed[pAesClient->Engine][pAesClient->KeySlot] = NV_FALSE;
        }

        // Free up engine if no other clients
        if (pAesClient->pAesCoreEngine->OpenCount)
        {
            // Decrement the Open count
            pAesClient->pAesCoreEngine->OpenCount--;

            if (0 == pAesClient->pAesCoreEngine->OpenCount)
            {
                // Free up resources
                AesCoreFreeUpEngine(pAesClient->pAesCoreEngine);

                NvOsFree(gs_pAesCoreEngine);
                gs_pAesCoreEngine = NULL;
            }
        }
        NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    }

    NvOsMemset(pAesClient, 0, sizeof(NvDdkAes));
    NvOsFree(pAesClient);
}

NvError NvDdkAesSelectKey(NvDdkAesHandle hAes, const NvDdkAesKeyInfo *pKeyInfo)
{
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;

    NVDDK_AES_CHECK_INPUT_PARAMS(pKeyInfo);

    NVDDK_AES_CHECK_USER_IDENTITY;

    switch (pKeyInfo->KeyLength)
    {
        case NvDdkAesKeySize_128Bit:
            return AesCoreSetKey(
                pKeyInfo->KeyType,
                pKeyInfo->IsDedicatedKeySlot,
                pKeyInfo->Key,
                pAesClient);
            break;
        case NvDdkAesKeySize_192Bit:
        case NvDdkAesKeySize_256Bit:
        default:
            return NvError_NotSupported;
    }
}

NvError NvDdkAesSelectOperation(NvDdkAesHandle hAes, const NvDdkAesOperation *pOperation)
{
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;

    NVDDK_AES_CHECK_INPUT_PARAMS(pOperation);

    NVDDK_AES_CHECK_USER_IDENTITY;

    switch (pOperation->OpMode)
    {
        case NvDdkAesOperationalMode_Cbc:
        case NvDdkAesOperationalMode_Ecb:
            pAesClient->IsEncryption = pOperation->IsEncrypt;
            break;
        case NvDdkAesOperationalMode_AnsiX931:
            pAesClient->IsEncryption = NV_TRUE;
            break;
        default:
            return NvError_NotSupported;
    }

    pAesClient->OpMode = pOperation->OpMode;
    return NvSuccess;
}

NvError NvDdkAesSetInitialVector(NvDdkAesHandle hAes, const NvU8 *pInitialVector, NvU32 VectorSize)
{
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;

    NVDDK_AES_CHECK_INPUT_PARAMS(pInitialVector);

    NVDDK_AES_CHECK_USER_IDENTITY;

    if (NvDdkAesOperationalMode_Ecb == pAesClient->OpMode)
        return NvError_NotSupported;

    if (VectorSize < NvDdkAesConst_IVLengthBytes)
        return NvError_BadParameter;

    // Set the IV and store it with client
    NvOsMemcpy(pAesClient->Iv, pInitialVector, NvDdkAesConst_IVLengthBytes);

    return NvSuccess;
}

NvError NvDdkAesGetInitialVector(NvDdkAesHandle hAes, NvU32 VectorSize, NvU8 *pInitialVector)
{
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;

    NVDDK_AES_CHECK_INPUT_PARAMS(pInitialVector);

    NVDDK_AES_CHECK_USER_IDENTITY;

    if (VectorSize < NvDdkAesConst_IVLengthBytes)
        return NvError_BadParameter;

    NvOsMemcpy(pInitialVector, pAesClient->Iv, NvDdkAesConst_IVLengthBytes);

    return NvSuccess;
}

NvError
NvDdkAesProcessBuffer(
    NvDdkAesHandle hAes,
    NvU32 SrcBufferSize,
    NvU32 DestBufferSize,
    const NvU8 *pSrcBuffer,
    NvU8 *pDestBuffer)
{
    if ((!hAes) || (!pSrcBuffer) || (!pDestBuffer))
        return NvError_BadParameter;

    return AesCoreProcessBuffer(0, SrcBufferSize, DestBufferSize, (NvDdkAes*)hAes, pSrcBuffer, pDestBuffer);
}

NvError NvDdkAesClearSecureBootKey(NvDdkAesHandle hAes)
{
    NvError e = NvSuccess;
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;

    if (!hAes)
        return NvError_BadParameter;

    NVDDK_AES_CHECK_USER_IDENTITY;

    NVDDK_AES_CHECK_ROOT_PERMISSION;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        AesHwEngine SbkEngine = pAesClient->pAesCoreEngine->SbkEngine[Engine];
        AesHwKeySlot DecryptSlot = pAesClient->pAesCoreEngine->SbkDecryptSlot;
        AesHwKeySlot EncryptSlot = pAesClient->pAesCoreEngine->SbkEncryptSlot;

        if (SbkEngine < AesHwEngine_Num)
        {
            NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, SbkEngine);

            AesCoreRequestHwAccess();

            // Clear the SBK encrypt key
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SbkEngine, AesHwClearKeyAndIv);
            pAesHwCtxt->ppEngineCaps[SbkEngine]->pAesInterf->AesHwClearKeyAndIv(SbkEngine, EncryptSlot, pAesHwCtxt);

            // Clear the SBK decrypt key
            pAesHwCtxt->ppEngineCaps[SbkEngine]->pAesInterf->AesHwClearKeyAndIv(SbkEngine, DecryptSlot, pAesHwCtxt);

            AesCoreReleaseHwAccess();

            if (!AesCoreIsSbkCleared(SbkEngine, EncryptSlot, DecryptSlot, pAesHwCtxt))
            {
                // Return error if SB clear check fails
                e = NvError_AesClearSbkFailed;
            }
        }
    }
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

NvError NvDdkAesLockSecureStorageKey(NvDdkAesHandle hAes)
{
    NvError e = NvSuccess;
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;

    if (!hAes)
        return NvError_BadParameter;

    NVDDK_AES_CHECK_USER_IDENTITY;

    NVDDK_AES_CHECK_ROOT_PERMISSION;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        AesHwEngine SskEngine = pAesClient->pAesCoreEngine->SskEngine[Engine];
        AesHwKeySlot DecryptSlot = pAesClient->pAesCoreEngine->SskDecryptSlot;
        AesHwKeySlot EncryptSlot = pAesClient->pAesCoreEngine->SskEncryptSlot;

        if (SskEngine < AesHwEngine_Num)
        {
            NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, SskEngine);

            AesCoreRequestHwAccess();

            // Disable permissions to the SSK key slot in the AES engine
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SskEngine, AesHwLockSskReadWrites);
            pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwLockSskReadWrites(pAesHwCtxt, SskEngine);

            AesCoreReleaseHwAccess();

            if (!AesCoreIsSskLocked(SskEngine, EncryptSlot, DecryptSlot, pAesHwCtxt))
            {
                e = NvError_AesLockSskFailed;
                goto fail;
            }

            // Also, lock the Secure scratch registers
            NV_CHECK_ERROR_CLEANUP(AesCoreLoadSskToSecureScratchAndLock(pAesHwCtxt, NULL));
        }
    }

fail:
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

NvError
NvDdkAesSetAndLockSecureStorageKey(
    NvDdkAesHandle hAes,
    NvDdkAesKeySize KeyLength,
    const NvU8 *pSecureStorageKey)
{
    NvError e = NvSuccess;
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwIv Iv;
    AesHwEngine Engine;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    if (!gs_pAesCoreEngine->SskUpdateAllowed)
    {
        NvOsMutexUnlock(gs_hAesCoreEngineMutex);
        return NvError_NotSupported;
    }
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);

    NVDDK_AES_CHECK_INPUT_PARAMS(pSecureStorageKey);

    NVDDK_AES_CHECK_USER_IDENTITY;

    NVDDK_AES_CHECK_ROOT_PERMISSION;

    switch (KeyLength)
    {
        case NvDdkAesKeySize_128Bit:
            break;
        case NvDdkAesKeySize_192Bit:
        case NvDdkAesKeySize_256Bit:
        default:
            return NvError_NotSupported;
    }

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        AesHwEngine SskEngine = pAesClient->pAesCoreEngine->SskEngine[Engine];
        AesHwKeySlot DecryptSlot = pAesClient->pAesCoreEngine->SskDecryptSlot;
        AesHwKeySlot EncryptSlot = pAesClient->pAesCoreEngine->SskEncryptSlot;

        if (SskEngine < AesHwEngine_Num)
        {
            // Setup the SSK with Zero IV
            NvOsMemset(&Iv, 0, NvDdkAesConst_IVLengthBytes);

            NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, SskEngine);

            AesCoreRequestHwAccess();

            // Setup SSK Key table for encryption
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SskEngine, AesHwSetKeyAndIv);
            pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwSetKeyAndIv(
                SskEngine,
                EncryptSlot,
                (AesHwKey*)pSecureStorageKey,
                &Iv,
                NV_TRUE,
                pAesHwCtxt);

            // Setup SSK Key table for decryption
            pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwSetKeyAndIv(
                SskEngine,
                DecryptSlot,
                (AesHwKey*)pSecureStorageKey,
                &Iv,
                NV_FALSE,
                pAesHwCtxt);

            // Disable the read / write access
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SskEngine, AesHwLockSskReadWrites);
            pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwLockSskReadWrites(pAesHwCtxt, SskEngine);

            AesCoreReleaseHwAccess();

            if (!AesCoreIsSskLocked(SskEngine, EncryptSlot, DecryptSlot, pAesHwCtxt))
            {
                e = NvError_AesLockSskFailed;
                goto fail;
            }

            // Store the SSK in the Secure scratch and lock
            NV_CHECK_ERROR_CLEANUP(AesCoreLoadSskToSecureScratchAndLock(pAesHwCtxt, pSecureStorageKey));
        }
    }

fail:
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

NvError NvDdkAesDisableCrypto(NvDdkAesHandle hAes)
{
    NvError e = NvSuccess;
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;

    if (!hAes)
        return NvError_BadParameter;

    NVDDK_AES_CHECK_USER_IDENTITY;

    NVDDK_AES_CHECK_ROOT_PERMISSION;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    AesCoreRequestHwAccess();

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwDisableEngine);

        if (NvSuccess != pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwDisableEngine(pAesHwCtxt, Engine))
            e = NvError_AesDisableCryptoFailed;
    }

    AesCoreReleaseHwAccess();

    if (NvSuccess == e)
    {
        // Mark engine as disabled
        pAesClient->pAesCoreEngine->IsEngineDisabled = NV_TRUE;
    }

    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

NvError NvDdkAesGetCapabilities(NvDdkAesHandle hAes, NvDdkAesCapabilities *pCapabilities)
{
    NvDdkAes *pAesClient = (NvDdkAes*)hAes;

    NVDDK_AES_CHECK_INPUT_PARAMS(pCapabilities);

    NVDDK_AES_CHECK_USER_IDENTITY;

    pCapabilities->OptimalBufferAlignment = 1;

    return NvSuccess;
}

/**
 * Request access to HW.
 */
void AesCoreRequestHwAccess(void)
{
#if !NV_OAL
    NvRmXpcModuleAcquire(NvRmModuleID_Vde);
    NvRmXpcModuleAcquire(NvRmModuleID_BseA);
#endif
}

/**
 * Release access to HW.
 */
void AesCoreReleaseHwAccess(void)
{
#if !NV_OAL
    NvRmXpcModuleRelease(NvRmModuleID_BseA);
    NvRmXpcModuleRelease(NvRmModuleID_Vde);
#endif
}

#if NV_OAL

/**
 * Allocate the RM memory for key table and dma buffers.
 *
 * @param pAesHwCtxt Pointer to the AES H/W engine context.
 *
 * @retval NvError_Success if memory is allocated successfully.
 * @retval NvError_InsufficientMemory if memory is not allocated successfully.
 */
NvError AesCoreAllocateRmMemory(AesHwContext *const pAesHwCtxt)
{
    NvError e;
    NvU8 *pKeyTabVirtAddr = NULL;
    NvRmPhysAddr KeyTabPhyAddr = NVBL_AES_KEY_TABLE_ADDR;
    NvU8 *pDmaVirtAddr = NULL;
    NvRmPhysAddr DmaPhyAddr = 0;
    AesHwEngine Engine;
    NvU32 size = 0;

    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Calculate total size needed
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
        size += (pAesHwCtxt->KeyTableSize[Engine] + NvDdkAesKeySize_128Bit);

    // Get virtual address
    NV_ASSERT_SUCCESS(NvOsPhysicalMemMap(
        KeyTabPhyAddr,
        size,
        NvOsMemAttribute_Uncached,
        NVOS_MEM_READ_WRITE,
        (void*)&pKeyTabVirtAddr));

    size = 0;
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        pAesHwCtxt->KeyTablePhyAddr[Engine] = KeyTabPhyAddr + size;
        pAesHwCtxt->pKeyTableVirAddr[Engine] = pKeyTabVirtAddr + size;
        size = pAesHwCtxt->KeyTableSize[Engine] + NvDdkAesKeySize_128Bit;
    }

    // Allocate DMA buffer for both the engines
    size = AES_HW_DMA_BUFFER_SIZE_BYTES * AesHwEngine_Num;
    NV_CHECK_ERROR_CLEANUP(NvRmMemHandleCreate(pAesHwCtxt->hRmDevice, &pAesHwCtxt->hDmaMemBuf, size));

    NV_CHECK_ERROR_CLEANUP(NvRmMemAlloc(
        pAesHwCtxt->hDmaMemBuf,
        NULL,
        0,
        AES_HW_DMA_ADDR_ALIGNMENT,
        NvOsMemAttribute_Uncached));

    // Get the virtual address
    pDmaVirtAddr = (NvU8 *)NvRmMemPin((NvRmMemHandle)pAesHwCtxt->hDmaMemBuf);

    // Get the physical address
    DmaPhyAddr = NvBlVaToPa(pDmaVirtAddr);
    if (NVBL_INVALID_PA == DmaPhyAddr)
    {
        NV_ASSERT(0);
        e = NvError_InvalidAddress;
        goto fail;
    }

    // Get a uncached alias to the buffer
    if (NvOsPhysicalMemMap(DmaPhyAddr, size, NvOsMemAttribute_Uncached, NVOS_MEM_READ_WRITE, (void **)&pDmaVirtAddr) !=
        NvError_Success)
    {
        NV_ASSERT(0);
        e = NvError_InvalidAddress;
        goto fail;
    }

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        pAesHwCtxt->DmaPhyAddr[Engine] = DmaPhyAddr + (Engine * AES_HW_DMA_BUFFER_SIZE_BYTES);
        pAesHwCtxt->pDmaVirAddr[Engine] = pDmaVirtAddr + (Engine * AES_HW_DMA_BUFFER_SIZE_BYTES);
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvError_Success;

fail:
    AesCoreDeAllocateRmMemory(pAesHwCtxt);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

/**
 * De-allocate the RM memory allocated with .AesAllocateRmMemory().
 *
 * @param pAesHwCtxt Pointer to the AES Hw engine context.
 *
 * @retval None.
 */
void AesCoreDeAllocateRmMemory(AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    if (pAesHwCtxt->hKeyTableMemBuf)
    {
        NvRmMemUnpin(pAesHwCtxt->hKeyTableMemBuf);
        NvRmMemHandleFree(pAesHwCtxt->hKeyTableMemBuf);
        pAesHwCtxt->hKeyTableMemBuf = NULL;
    }

    if (pAesHwCtxt->hDmaMemBuf)
    {
        NvRmMemUnpin(pAesHwCtxt->hDmaMemBuf);
        NvRmMemHandleFree(pAesHwCtxt->hDmaMemBuf);
        pAesHwCtxt->hDmaMemBuf = NULL;
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
}

#else

/**
 * Allocate the RM memory for key table and dma buffers.
 *
 * @param pAesHwCtxt Pointer to the AES H/W engine context.
 *
 * @retval NvError_Success if memory is allocated successfully.
 * @retval NvError_InsufficientMemory if memory is not allocated successfully.
 */
NvError AesCoreAllocateRmMemory(AesHwContext *const pAesHwCtxt)
{
    NvError e;
    static const NvRmHeap s_Heaps[] = {NvRmHeap_IRam};
    NvU8 *pKeyTabVirtAddr = NULL;
    NvRmPhysAddr KeyTabPhyAddr = 0;
    NvU8 * pDmaVirtAddr = NULL;
    NvRmPhysAddr DmaPhyAddr = 0;
    // Allocate key table memory for both the engines
    NvU32 size = NVBL_AES_KEY_TABLE_OFFSET;
    AesHwEngine Engine;

    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Calculate total size needed
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
        size += (pAesHwCtxt->KeyTableSize[Engine] + NvDdkAesKeySize_128Bit);

    NV_CHECK_ERROR(NvRmMemHandleCreate(pAesHwCtxt->hRmDevice, &pAesHwCtxt->hKeyTableMemBuf, size));

    if (!pAesHwCtxt->hKeyTableMemBuf)
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }

    NV_CHECK_ERROR_CLEANUP(NvRmMemAlloc(
        pAesHwCtxt->hKeyTableMemBuf,
        s_Heaps,
        NV_ARRAY_SIZE(s_Heaps),
        AES_HW_KEY_TABLE_ADDR_ALIGNMENT,
        NvOsMemAttribute_Uncached));

    KeyTabPhyAddr = NvRmMemPin(pAesHwCtxt->hKeyTableMemBuf) + NVBL_AES_KEY_TABLE_OFFSET;

    NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(
        KeyTabPhyAddr,
        (size - NVBL_AES_KEY_TABLE_OFFSET),
        NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void **)&pKeyTabVirtAddr));

    size = 0;
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        pAesHwCtxt->KeyTablePhyAddr[Engine] = KeyTabPhyAddr + size;
        pAesHwCtxt->pKeyTableVirAddr[Engine] = pKeyTabVirtAddr + size;
        size = pAesHwCtxt->KeyTableSize[Engine] + NvDdkAesKeySize_128Bit;
    }

    // Allocate DMA buffer for both the engines
    size = AES_HW_DMA_BUFFER_SIZE_BYTES * AesHwEngine_Num;

    NV_CHECK_ERROR_CLEANUP(NvRmMemHandleCreate(pAesHwCtxt->hRmDevice, &pAesHwCtxt->hDmaMemBuf, size));

    NV_CHECK_ERROR_CLEANUP(NvRmMemAlloc(
        pAesHwCtxt->hDmaMemBuf,
        NULL,
        0,
        AES_HW_DMA_ADDR_ALIGNMENT,
        NvOsMemAttribute_Uncached));

    DmaPhyAddr = NvRmMemPin(pAesHwCtxt->hDmaMemBuf);

    NV_CHECK_ERROR_CLEANUP(NvRmMemMap(pAesHwCtxt->hDmaMemBuf, 0, size, NVOS_MEM_READ_WRITE, (void **)&pDmaVirtAddr));

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        pAesHwCtxt->DmaPhyAddr[Engine] = DmaPhyAddr + (Engine * AES_HW_DMA_BUFFER_SIZE_BYTES);
        pAesHwCtxt->pDmaVirAddr[Engine] = pDmaVirtAddr + (Engine * AES_HW_DMA_BUFFER_SIZE_BYTES);
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvSuccess;

fail:
    AesCoreDeAllocateRmMemory(pAesHwCtxt);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

/**
 * De-allocate the RM memory allocated with .AesAllocateRmMemory().
 *
 * @param pAesHwCtxt Pointer to the AES Hw engine context.
 *
 * @retval None.
 */
void AesCoreDeAllocateRmMemory(AesHwContext *const pAesHwCtxt)
{
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    if (pAesHwCtxt->hKeyTableMemBuf)
    {
        AesHwEngine Engine;
        NvU32 size = 0;

        for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
            size += (pAesHwCtxt->KeyTableSize[Engine] + NvDdkAesKeySize_128Bit);

        NvRmPhysicalMemUnmap(pAesHwCtxt->pKeyTableVirAddr[0], size);
        NvRmMemUnpin(pAesHwCtxt->hKeyTableMemBuf);
        NvRmMemHandleFree(pAesHwCtxt->hKeyTableMemBuf);
        pAesHwCtxt->hKeyTableMemBuf = NULL;
    }

    if (pAesHwCtxt->hDmaMemBuf)
    {
        NvRmMemUnmap(
            pAesHwCtxt->hDmaMemBuf,
            pAesHwCtxt->pDmaVirAddr[0],
            AES_HW_DMA_BUFFER_SIZE_BYTES * AesHwEngine_Num);
        NvRmMemUnpin(pAesHwCtxt->hDmaMemBuf);
        NvRmMemHandleFree(pAesHwCtxt->hDmaMemBuf);
        pAesHwCtxt->hDmaMemBuf = NULL;
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
}

#endif // NV_OAL

/**
 * Select the key specified by the client in the AES engine.
 *

 * @param KeyType Key type.
 * @param IsDedicatedSlot NV_TRUE if slot is dedicated, NV_FALSE if not.
 * @param pKeyData Pointer to the key data.
 * @param pAesClient Pointer to AES client.
 *
 * @retval NvSuccess if successfully completed.
 * @retval NvError_NotSupported if operation is not supported.
 */
NvError
AesCoreSetKey(
    const NvDdkAesKeyType KeyType,
    const NvBool IsDedicatedSlot,
    const NvU8 *const pKeyData,
    NvDdkAes *const pAesClient)
{
    NvError e = NvSuccess;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;
    AesHwKeySlot KeySlot;

    NV_ASSERT(pAesClient);

    if ((NvDdkAesKeyType_SecureBootKey == KeyType) || (NvDdkAesKeyType_SecureStorageKey == KeyType))
    {
        NVDDK_AES_CHECK_ROOT_PERMISSION;
    }

    pAesClient->IsDedicatedSlot = NV_FALSE;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    switch (KeyType)
    {
        case NvDdkAesKeyType_SecureBootKey:
            pAesClient->Engine = pAesClient->pAesCoreEngine->SbkEngine[0];
            pAesClient->KeySlot = pAesClient->IsEncryption ?
                pAesClient->pAesCoreEngine->SbkEncryptSlot : pAesClient->pAesCoreEngine->SbkDecryptSlot;
            break;
        case NvDdkAesKeyType_SecureStorageKey:
            pAesClient->Engine = pAesClient->pAesCoreEngine->SskEngine[0];
            pAesClient->KeySlot = pAesClient->IsEncryption ?
                pAesClient->pAesCoreEngine->SskEncryptSlot : pAesClient->pAesCoreEngine->SskDecryptSlot;
            break;
        case NvDdkAesKeyType_UserSpecified:
            pAesClient->IsDedicatedSlot = IsDedicatedSlot;
            // Wrap the key using RFC3394 key wrapping algorithm
            // The wrapped key and RFCwrapped IV will be stored in client handle
            AesCoreRequestHwAccess();
            e = AesCoreWrapKey(
                pAesClient->pAesCoreEngine,
                pKeyData,
                gs_OriginalIV,
                pAesClient->Key,
                pAesClient->WrappedIv);
            AesCoreReleaseHwAccess();
            if (NvSuccess != e)
                goto fail;
            if (!IsDedicatedSlot)
               break;
            // It is dedicated slot
            NV_CHECK_ERROR_CLEANUP(AesCoreGetFreeSlot(pAesClient->pAesCoreEngine, &Engine, &KeySlot));
            pAesClient->pAesCoreEngine->IsKeySlotUsed[Engine][KeySlot] = NV_TRUE;
            pAesClient->Engine = Engine;
            pAesClient->KeySlot = KeySlot;
            // Store the dedicated slot key info in wrapped form to use in LP0 resume
            gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].IsEncryption = pAesClient->IsEncryption;
            NvOsMemcpy(gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].WrappedKey,
                pAesClient->Key,
                NvDdkAesConst_MaxKeyLengthBytes);
            NvOsMemcpy(gs_pAesCoreEngine->DedicatedSlotKeyInfo[Engine][KeySlot].WrappedIv,
                pAesClient->WrappedIv,
                AES_RFC_IV_LENGTH_BYTES);
            // Initialize IV to zeros
            NvOsMemset(pAesClient->Iv, 0, NvDdkAesConst_IVLengthBytes);
            NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);

            AesCoreRequestHwAccess();

            // Setup Key table
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetKeyAndIv);
            pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwSetKeyAndIv(
                pAesClient->Engine,
                pAesClient->KeySlot,
                (AesHwKey *)pKeyData,
                (AesHwIv *)pAesClient->Iv,
                pAesClient->IsEncryption,
                pAesHwCtxt);

            AesCoreReleaseHwAccess();
            break;
        default:
            goto fail;
    }

    if ((KeyType == NvDdkAesKeyType_UserSpecified) && (!IsDedicatedSlot))
        goto done;
    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, pAesClient->Engine);

    AesCoreRequestHwAccess();

    // Select Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwSelectKeyIvSlot(
        pAesClient->Engine,
        pAesClient->KeySlot,
        pAesHwCtxt);

    // Get the IV and store it with client
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwGetIv);
    pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwGetIv(
        pAesHwCtxt,
        pAesClient->Engine,
        pAesClient->KeySlot,
        (AesHwIv *)pAesClient->Iv);

    AesCoreReleaseHwAccess();

done:
    // Store the Key Type for this client
    pAesClient->KeyType = KeyType;

fail:
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

/**
 * Load the SSK into the secure scratch and disables the write permissions.
 * Note: If Key is not specified then this API locks the Secure Scratch registers.
 *
 * @param pAesHwCtxt Pointer to AES H/W context.
 * @param pKey Pointer to the key. If pKey == NULL then key will not be set to the
 *             secure scratch registers, but locks the Secure scratch register.
 *
 * @retval NvSuccess if successfully completed.
 */
NvError AesCoreLoadSskToSecureScratchAndLock(const AesHwContext *const pAesHwCtxt, const NvU8 *const pKey)
{
    NvError e = NvSuccess;
    NvRmPhysAddr PhysAdr;
    NvU32 BankSize;
    NvU32 RmPwrClientId = 0;

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, 0);

    // Get the secure scratch base address
    NvRmModuleGetBaseAddress(pAesHwCtxt->hRmDevice, NVRM_MODULE_ID(NvRmModuleID_Pmif, 0), &PhysAdr, &BankSize);

    // Register with Power Manager
    RmPwrClientId = NVRM_POWER_CLIENT_TAG('A','E','S','2');

    NV_CHECK_ERROR_CLEANUP(NvRmPowerRegister(pAesHwCtxt->hRmDevice, NULL, &RmPwrClientId));

    // Enable the Voltage
    NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(
        pAesHwCtxt->hRmDevice,
        NvRmModuleID_Pmif,
        RmPwrClientId,
        NvRmVoltsUnspecified,
        NvRmVoltsUnspecified,
        NULL,
        0,
        NULL));

    // Emable the clock to the PMIC
    NV_CHECK_ERROR_CLEANUP(NvRmPowerModuleClockControl(pAesHwCtxt->hRmDevice, NvRmModuleID_Pmif, RmPwrClientId, NV_TRUE));

    // Store SSK and lock the secure scratch -- engine doesn't matter here
    // since the key is being provided and not really read from the key table of an engine .here
    // If pKey == NULL this call will disable the write permissions to the scratch registers.
    AesCoreRequestHwAccess();

    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, 0, AesHwLoadSskToSecureScratchAndLock);
    pAesHwCtxt->ppEngineCaps[0]->pAesInterf->AesHwLoadSskToSecureScratchAndLock(PhysAdr, (AesHwKey *)pKey, BankSize);

    AesCoreReleaseHwAccess();

fail:
    // Disable the clock to the PMIC
    NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(pAesHwCtxt->hRmDevice, NvRmModuleID_Pmif, RmPwrClientId, NV_FALSE));

    // Disable the Voltage
    NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(
        pAesHwCtxt->hRmDevice,
        NvRmModuleID_Pmif,
        RmPwrClientId,
        NvRmVoltsOff,
        NvRmVoltsOff,
        NULL,
        0,
        NULL));

    // Unregister driver from Power Manager
    NvRmPowerUnRegister(pAesHwCtxt->hRmDevice, RmPwrClientId);
    return e;
}

/**
 * Check the SBK clear by encrypting / decrypting the known data.
 *
 * @param Engine Engine on which encryption and decryption need to be performed
 * @param EncryptSlot Key slot where encrypt key is located.
 * @param DecryptSlot Key slot where decrypt key is located.
 * @param pAesHwCtxt Pointer to AES H/W context.
 *
 * @retval NV_TRUE if successfully encryption and decryption is done else NV_FALSE.
 */
NvBool AesCoreIsSbkCleared(
    const AesHwEngine Engine,
    const AesHwKeySlot EncryptSlot,
    const AesHwKeySlot DecryptSlot,
    AesHwContext *const pAesHwCtxt)
{
    NvError e = NvSuccess;
    AesHwIv ZeroIv;
    NvU32 i;

    // Known Good data
    static NvU8 s_GoldData[NvDdkAesConst_BlockLengthBytes] =
    {
        0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F
    };

    // Encrypted data for above known data with Zero key and Zero IV
    static NvU8 s_EncryptDataWithZeroKeyTable[NvDdkAesConst_BlockLengthBytes] =
    {
        0x7A, 0xCA, 0x0F, 0xD9,
        0xBC, 0xD6, 0xEC, 0x7C,
        0x9F, 0x97, 0x46, 0x66,
        0x16, 0xE6, 0xA2, 0x82
    };

    // Encrypted data for above known data with Zero key and Zero IV
    static NvU8 s_EncryptDataWithZeroKeySchedule[NvDdkAesConst_BlockLengthBytes] =
    {
        0x18, 0x9D, 0x19, 0xEA,
        0xDB, 0xA7, 0xE3, 0x0E,
        0xD9, 0x72, 0x80, 0x8F,
        0x3F, 0x2B, 0xA0, 0x30
    };

    NvU8 *pEncryptData = NULL;
    NvU8 EncryptBuffer[NvDdkAesConst_BlockLengthBytes];
    NvU8 DecryptBuffer[NvDdkAesConst_BlockLengthBytes];

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    if (pAesHwCtxt->ppEngineCaps[Engine]->IsHwKeySchedGenSupported)
        pEncryptData = s_EncryptDataWithZeroKeyTable;
    else
        pEncryptData = s_EncryptDataWithZeroKeySchedule;

    NvOsMemset(EncryptBuffer, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(DecryptBuffer, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(&ZeroIv, 0, sizeof(AesHwIv));

    AesCoreRequestHwAccess();

    // Select Encrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, EncryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        EncryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_TRUE,
        NvDdkAesOperationalMode_Cbc,
        EncryptBuffer,
        pAesHwCtxt));

    // Select Decrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, DecryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        DecryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        EncryptBuffer,
        NV_FALSE,
        NvDdkAesOperationalMode_Cbc,
        DecryptBuffer,
        pAesHwCtxt));

    // Clear the IV in the engine before we leave
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwClearIv);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwClearIv(Engine, EncryptSlot, pAesHwCtxt);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwClearIv(Engine, DecryptSlot, pAesHwCtxt);

    // Clear the DMA buffer before we leave from this operation.
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);

    for (i = 0; i < NvDdkAesConst_BlockLengthBytes; i++)
    {
        if ((pEncryptData[i] != EncryptBuffer[i]) || (s_GoldData[i] != DecryptBuffer[i]))
            goto fail;
    }

    AesCoreReleaseHwAccess();
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_TRUE;

fail:
    // Clear the DMA buffer before we leave from this operation
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);
    AesCoreReleaseHwAccess();
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_FALSE;
}

/**
 * Check the SSK lock is successfully done or not by writing zero key into SSK.
 *
 * @param Engine Engine where SSK is set.
 * @param EncryptSlot Key slot where encrypt key is located.
 * @param DecryptSlot Key slot where decrypt key is located.
 * @param pAesHwCtxt Pointer to AES H/W context.
 *
 * @retval NV_TRUE if successfully SSK is locked else NV_FALSE.
 */
NvBool
AesCoreIsSskLocked(
    const AesHwEngine Engine,
    const AesHwKeySlot EncryptSlot,
    const AesHwKeySlot DecryptSlot,
    AesHwContext *const pAesHwCtxt)
{
    NvError e = NvSuccess;
    AesHwIv ZeroIv;
    AesHwKey ZeroKey;
    NvU32 i;

    static NvU8 s_GoldData[NvDdkAesConst_BlockLengthBytes] =
    {
        0x00, 0x11, 0x22, 0x33,
        0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xAA, 0xBB,
        0xCC, 0xDD, 0xEE, 0xFF
    };

    NvU8 EncryptBuffer1[NvDdkAesConst_BlockLengthBytes];
    NvU8 DecryptBuffer1[NvDdkAesConst_BlockLengthBytes];
    NvU8 EncryptBuffer2[NvDdkAesConst_BlockLengthBytes];
    NvU8 DecryptBuffer2[NvDdkAesConst_BlockLengthBytes];

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);

    NvOsMemset(EncryptBuffer1, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(DecryptBuffer1, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(EncryptBuffer2, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(DecryptBuffer2, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(&ZeroIv, 0, sizeof(AesHwIv));
    NvOsMemset(&ZeroKey, 0, sizeof(AesHwKey));

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    AesCoreRequestHwAccess();

    // Select Encrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, EncryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        EncryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_TRUE,
        NvDdkAesOperationalMode_Cbc,
        EncryptBuffer1,
        pAesHwCtxt));

    // Select Decrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, DecryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        DecryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_FALSE,
        NvDdkAesOperationalMode_Cbc,
        DecryptBuffer1,
        pAesHwCtxt));

    // Set Zero key to the SSK slot and try encryption / decryption with
    // known data and check data after encryption and decryption are same with SSK

    // Setup SSK Key table for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetKeyAndIv);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetKeyAndIv(
        Engine,
        EncryptSlot,
        &ZeroKey,
        &ZeroIv,
        NV_TRUE,
        pAesHwCtxt);

    // Setup SSK Key table for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetKeyAndIv);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetKeyAndIv(
        Engine,
        DecryptSlot,
        &ZeroKey,
        &ZeroIv,
        NV_FALSE,
        pAesHwCtxt);

    // Select Encrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, EncryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        EncryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_TRUE,
        NvDdkAesOperationalMode_Cbc,
        EncryptBuffer2,
        pAesHwCtxt));

    // Select Decrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, DecryptSlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        DecryptSlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_FALSE,
        NvDdkAesOperationalMode_Cbc,
        DecryptBuffer2,
        pAesHwCtxt));

    // Clear the IV in the engine before we leave
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwClearIv);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwClearIv(Engine, EncryptSlot, pAesHwCtxt);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwClearIv(Engine, DecryptSlot, pAesHwCtxt);

    // Clear the DMA buffer before we leave from this operation.
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);

    // Check encrypt and decrypt output is same before and after zero key set to SSK
    // If both encrypt and decrypt data match then SSK lock is OK
    for (i = 0; i < NvDdkAesConst_BlockLengthBytes; i++)
    {
        if ((EncryptBuffer1[i] != EncryptBuffer2[i]) || (DecryptBuffer1[i] != DecryptBuffer2[i]))
            goto fail;
    }

    AesCoreReleaseHwAccess();
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_TRUE;

fail:
    // Clear the DMA buffer before we leave from this operation.
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);
    AesCoreReleaseHwAccess();
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_FALSE;
}

/**
 * Add the Busy hints to boost or reduce the CPU, System and EMC frequencies.
 *
 * @param hRmDevice RM device handle.
 * @param PowerClientId The client ID obtained during Power registration.
 * @param IsDfsOn Indicator to boost the frequency, if set to NV_TRUE. Cancel the
 *        DFS busy hint if set to NV_FALSE.
 *
 * @retval NvSuccess if busy hint request completed successfully.
 * @retval NvError_NotSupported if DFS is disabled.
 * @retval NvError_BadValue if specified client ID is not registered.
 * @retval NvError_InsufficientMemory if failed to allocate memory for busy hints.
 */
NvError AesCoreDfsBusyHint(const NvRmDeviceHandle hRmDevice, const NvU32 PowerClientId, const NvBool IsDfsOn)
{
    #define AES_EMC_BOOST_FREQ_KHZ 100000
    #define AES_SYS_BOOST_FREQ_KHZ 100000

    NvRmDfsBusyHint AesBusyHintOn[] =
    {
        {NvRmDfsClockId_Emc, NV_WAIT_INFINITE, AES_EMC_BOOST_FREQ_KHZ, NV_TRUE},
        {NvRmDfsClockId_System, NV_WAIT_INFINITE, AES_SYS_BOOST_FREQ_KHZ, NV_TRUE}
    };

    NvRmDfsBusyHint AesBusyHintOff[] =
    {
        {NvRmDfsClockId_Emc, 0, 0, NV_TRUE},
        {NvRmDfsClockId_System, 0, 0, NV_TRUE}
    };

    NV_ASSERT(hRmDevice);

    if (IsDfsOn)
    {
        return NvRmPowerBusyHintMulti(
            hRmDevice,
            PowerClientId,
            AesBusyHintOn,
            NV_ARRAY_SIZE(AesBusyHintOn),
            NvRmDfsBusyHintSyncMode_Async);
    }
    else
    {
        return NvRmPowerBusyHintMulti(
            hRmDevice,
            PowerClientId,
            AesBusyHintOff,
            NV_ARRAY_SIZE(AesBusyHintOff),
            NvRmDfsBusyHintSyncMode_Async);
    }
}

/**
 * Populate the structure for AES context with the engine base address.
 *
 * @param hRmDevice Rm device handle.
 * @param pAesHwCtxt Pointer to AES H/W context.
 *
 * @retval NvSuccess if successfully completed.
 *
 */
NvError AesCoreInitializeEngineSpace(const NvRmDeviceHandle hRmDevice, AesHwContext *const pAesHwCtxt)
{
    NvError e = NvSuccess;
    AesHwEngine Engine;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        switch (Engine)
        {
            case AesHwEngine_A:
                // Get the controller base address
                NvRmModuleGetBaseAddress(
                    hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Vde, 0),
                    &pAesHwCtxt->PhysAdr[Engine],
                    &pAesHwCtxt->BankSize[Engine]);
                break;
            case AesHwEngine_B:
                // Get the controller base address
                NvRmModuleGetBaseAddress(
                    hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_BseA, 0),
                    &pAesHwCtxt->PhysAdr[Engine],
                    &pAesHwCtxt->BankSize[Engine]);
                break;
            default:
                break;
        }

        // Map the physical memory to virtual memory
        NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(
            pAesHwCtxt->PhysAdr[Engine],
            pAesHwCtxt->BankSize[Engine],
            NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void **)&pAesHwCtxt->pVirAdr[Engine]));
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvSuccess;

fail:
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

/**
 * Unmap all engine space.
 *
 * @param pAesHwCtxt Pointer to AES H/W context.
 *
 * @retval None.
 *
 */
void AesCoreDeInitializeEngineSpace(const AesHwContext *const pAesHwCtxt)
{
    AesHwEngine Engine;

    NV_ASSERT(pAesHwCtxt);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Clean up resources
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        // UnMap the virtual Address
        NvRmPhysicalMemUnmap(pAesHwCtxt->pVirAdr[Engine], pAesHwCtxt->BankSize[Engine]);
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
}

/**
 * Find the unused key slot.
 *
 * @param pAesCoreEngine Pointer to AES core engine.
 * @param pEngine Pointer to the engine.
 * @param pKeySlot Pointer to the key slot.
 *
 * @retval NvSuccess if successfully completed.
 * @retval NvError_AlreadyAllocated if all slots are allocated.
 */
NvError
AesCoreGetFreeSlot(
    const AesCoreEngine *const pAesCoreEngine,
    AesHwEngine *const pEngine,
    AesHwKeySlot *const pKeySlot)
{
    AesHwEngine Engine;
    AesHwKeySlot KeySlot;
    const AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);
    NV_ASSERT(pEngine);
    NV_ASSERT(pKeySlot);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Get the AES H/W context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    // Get the free slot
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NV_ASSERT(pAesHwCtxt->ppEngineCaps);
        NV_ASSERT(pAesHwCtxt->ppEngineCaps[Engine]);
        for (KeySlot = AesHwKeySlot_0;
            KeySlot < (NvU32)(pAesHwCtxt->ppEngineCaps[Engine]->NumSlotsSupported);
            KeySlot++)
        {
            if (!pAesCoreEngine->IsKeySlotUsed[Engine][KeySlot])
            {
                *pEngine = Engine;
                *pKeySlot = KeySlot;

                NvOsMutexUnlock(gs_hAesCoreEngineMutex);
                return NvSuccess;
            }
        }
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvError_AlreadyAllocated;
}

/**
 * Init AES engine.
 *
 * @param hRmDevice Resource Manager Handle.
 *
 * @retval NvSuccess if successful.
 */
NvError AesCoreInitEngine(const NvRmDeviceHandle hRmDevice)
{
    NvError e = NvSuccess;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;

    NV_ASSERT(hRmDevice);

    gs_pAesCoreEngine = NvOsAlloc(sizeof(AesCoreEngine));
    if (NULL == gs_pAesCoreEngine)
        return NvError_InsufficientMemory;

    // Clear the memory initially
    NvOsMemset(gs_pAesCoreEngine, 0, sizeof(AesCoreEngine));

    // Get the AES H/W context
    pAesHwCtxt = &gs_pAesCoreEngine->AesHwCtxt;

    // Store the RM handle for future use
    pAesHwCtxt->hRmDevice = hRmDevice;

    pAesHwCtxt->ppEngineCaps = (AesHwCapabilities **)NvOsAlloc(sizeof(AesHwCapabilities) * AesHwEngine_Num);
    if (NULL == pAesHwCtxt->ppEngineCaps)
    {
        NvOsFree(gs_pAesCoreEngine);
        gs_pAesCoreEngine = NULL;
        return NvError_InsufficientMemory;
    }
    AesCoreGetCapabilities(hRmDevice, pAesHwCtxt->ppEngineCaps);

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NV_ASSERT(pAesHwCtxt->ppEngineCaps[Engine]);
        pAesHwCtxt->KeyTableSize[Engine] = pAesHwCtxt->ppEngineCaps[Engine]->HwKeySchedLengthBytes;
    }

    NV_CHECK_ERROR(AesCoreInitializeEngineSpace(hRmDevice, pAesHwCtxt));

    // Allocate memories required for H/W operation
    NV_CHECK_ERROR(AesCoreAllocateRmMemory(pAesHwCtxt));

    // Create mutex to guard the H/W engine
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NV_CHECK_ERROR(NvOsMutexCreate(&pAesHwCtxt->Mutex[Engine]));

        // Register with Power Manager
        gs_pAesCoreEngine->RmPwrClientId[Engine] = NVRM_POWER_CLIENT_TAG('A','E','S','1');

        NV_CHECK_ERROR(NvRmPowerRegister(hRmDevice, NULL, &gs_pAesCoreEngine->RmPwrClientId[Engine]));

        // Enable the voltage
        NV_CHECK_ERROR(NvRmPowerVoltageControl(
            hRmDevice,
            (AesHwEngine_A == Engine) ?
            NvRmModuleID_Vde : NvRmModuleID_BseA,
            gs_pAesCoreEngine->RmPwrClientId[Engine],
            NvRmVoltsUnspecified,
            NvRmVoltsUnspecified,
            NULL,
            0,
            NULL));

        // Enable clock
        NV_CHECK_ERROR(NvRmPowerModuleClockControl(
            hRmDevice,
            (AesHwEngine_A == Engine) ?
            NvRmModuleID_Vde : NvRmModuleID_BseA,
            gs_pAesCoreEngine->RmPwrClientId[Engine],
            NV_TRUE));
    }

    // Request the H/W semaphore before accessing the AES H/W
    AesCoreRequestHwAccess();

    // Reset the BSEV and BSEA engines
    NvRmModuleReset(hRmDevice, NvRmModuleID_Vde);
    NvRmModuleReset(hRmDevice, NvRmModuleID_BseA);

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwIsEngineDisabled);

        gs_pAesCoreEngine->IsEngineDisabled =
            pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwIsEngineDisabled(pAesHwCtxt, Engine);
    }

    // If engine is not disabled then set the SBK & SSK
    if (!gs_pAesCoreEngine->IsEngineDisabled)
    {
        // The slots already dedicated don't depend on which engine is being used but
        // on the capabilities the engines can provide. Basic assumption: both engines have
        // same capabilities.
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, 0, AesHwGetUsedSlots);
        pAesHwCtxt->ppEngineCaps[0]->pAesInterf->AesHwGetUsedSlots(gs_pAesCoreEngine);
    }

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwDisableAllKeyRead);
        pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwDisableAllKeyRead(
            pAesHwCtxt,
            Engine,
            pAesHwCtxt->ppEngineCaps[Engine]->NumSlotsSupported);
    }

    // Release the H/W semaphore
    AesCoreReleaseHwAccess();

    // Disable clocks after AES init
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        // Disable clock
        NV_CHECK_ERROR(NvRmPowerModuleClockControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ?
            NvRmModuleID_Vde : NvRmModuleID_BseA,
            gs_pAesCoreEngine->RmPwrClientId[Engine],
            NV_FALSE));

        // Disable the voltage
        NV_CHECK_ERROR(NvRmPowerVoltageControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ?
            NvRmModuleID_Vde : NvRmModuleID_BseA,
            gs_pAesCoreEngine->RmPwrClientId[Engine],
            NvRmVoltsOff,
            NvRmVoltsOff,
            NULL,
            0,
            NULL));
    }
    gs_pAesCoreEngine->SskUpdateAllowed =
            pAesHwCtxt->ppEngineCaps[AesHwEngine_A]->pAesInterf->AesHwIsSskUpdateAllowed();
    return e;
}

/**
  * Free up resources.
  *
  * @retval None.
  */
void AesCoreFreeUpEngine(AesCoreEngine *const pAesCoreEngine)
{
    AesHwEngine Engine;
    AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);

    // Get the AES H/W context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    AesCoreDeInitializeEngineSpace(pAesHwCtxt);

    // Deallocate the memory
    AesCoreDeAllocateRmMemory(pAesHwCtxt);

    // Destroy mutex
    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        NvOsMutexDestroy(pAesHwCtxt->Mutex[Engine]);
        // Unregister driver from Power Manager
        NvRmPowerUnRegister(pAesHwCtxt->hRmDevice, pAesCoreEngine->RmPwrClientId[Engine]);
        if (pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf)
        {
            NvOsFree(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf);
            pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf = NULL;
        }
    }
    if (pAesHwCtxt->ppEngineCaps)
    {
        NvOsFree(pAesHwCtxt->ppEngineCaps);
        pAesHwCtxt->ppEngineCaps = NULL;
    }
}

/**
 * Power up the AES core engine.
 *
 * @param pAesCoreEngine Pointer to the AesCoreEngine argument.
 * @param SetDfsBusyHints If set to NV_TRUE, DFS busy hints will be set to ON
 *
 * @retval None.
 */
void AesCorePowerUp(const AesCoreEngine *const pAesCoreEngine, const NvBool SetDfsBusyHints)
{
    AesHwEngine Engine;
    const AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);

    // Get the Aes Hw context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        if (SetDfsBusyHints)
        {
            // DFS busy hints On
            NV_ASSERT_SUCCESS(AesCoreDfsBusyHint(pAesHwCtxt->hRmDevice, pAesCoreEngine->RmPwrClientId[Engine], NV_TRUE));
        }

        // Enable the voltage
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ? NvRmModuleID_Vde : NvRmModuleID_BseA,
            pAesCoreEngine->RmPwrClientId[Engine],
            NvRmVoltsUnspecified,
            NvRmVoltsUnspecified,
            NULL,
            0,
            NULL));

        // Enable clock
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ? NvRmModuleID_Vde : NvRmModuleID_BseA,
            pAesCoreEngine->RmPwrClientId[Engine],
            NV_TRUE));
    }
}

/**
 * Power down the AES core engine.
 *
 * @param pAesCoreEngine Pointer to the AesCoreEngine argument.
 * @param SetDfsBusyHints If set to NV_TRUE, DFS busy hints will be set to OFF
 *
 * @retval None.
 */
void AesCorePowerDown(const AesCoreEngine *const pAesCoreEngine, const NvBool SetDfsBusyHints)
{
    AesHwEngine Engine;
    const AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);

    // Get the AES H/W context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    for (Engine = AesHwEngine_A; Engine < AesHwEngine_Num; Engine++)
    {
        // Disable clock
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ? NvRmModuleID_Vde : NvRmModuleID_BseA,
            pAesCoreEngine->RmPwrClientId[Engine],
            NV_FALSE));

        // Disable the voltage
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(
            pAesHwCtxt->hRmDevice,
            (AesHwEngine_A == Engine) ? NvRmModuleID_Vde : NvRmModuleID_BseA,
            pAesCoreEngine->RmPwrClientId[Engine],
            NvRmVoltsOff,
            NvRmVoltsOff,
            NULL,
            0,
            NULL));
        if (SetDfsBusyHints)
        {
            // DFS busy hints Off
            NV_ASSERT_SUCCESS(AesCoreDfsBusyHint(pAesHwCtxt->hRmDevice, pAesCoreEngine->RmPwrClientId[Engine], NV_FALSE));
        }
    }
}

/**
 * Process the buffers for encryption or decryption.
 *
 * @param SkipOffset Skip initial SkipOffset bytes of SrcBuffer before beginning cipher.
 * @param SrcBufferSize Size of src buffer in bytes.
 * @param DestBufferSize Size of dest buffer in bytes.
 * @param pAesClient Pointer to AES client.
 * @param pSrcBuffer Pointer to src buffer.
 * @param pDestBuffer Pointer to dest buffer.
 *
 * @retval NvSuccess if successfully completed.
 */
NvError
AesCoreProcessBuffer(
    const NvU32 SkipOffset,
    const NvU32 SrcBufferSize,
    const NvU32 DestBufferSize,
    NvDdkAes *const pAesClient,
    const NvU8 *pSrcBuffer,
    NvU8 *pDestBuffer)
{
    NvError e = NvSuccess;
    AesHwContext *pAesHwCtxt = NULL;
    AesHwEngine Engine;
    AesHwKeySlot KeySlot;
    NvU32 TotalBytesToProcess = 0;
    NvU32 BytesToProcess = 0;

    NV_ASSERT(pAesClient);
    NV_ASSERT(pSrcBuffer);
    NV_ASSERT(pDestBuffer);

    NVDDK_AES_CHECK_USER_IDENTITY;

    // Check type of operation supported for the process buffer
    switch (pAesClient->OpMode)
    {
        case NvDdkAesOperationalMode_Cbc:
        case NvDdkAesOperationalMode_Ecb:
        case NvDdkAesOperationalMode_AnsiX931:
            break;
        default:
            return NvError_InvalidState;
    }

    if (DestBufferSize % NvDdkAesConst_BlockLengthBytes)
        return NvError_InvalidSize;

    // Check if client has already assigned key for this process if not return
    if ((NvDdkAesKeyType_Invalid == pAesClient->KeyType) || (pAesClient->KeyType >= NvDdkAesKeyType_Num))
        return NvError_InvalidState;

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    TotalBytesToProcess = DestBufferSize;

    NvOsMutexLock(gs_hAesCoreEngineMutex);
    AesCorePowerUp(gs_pAesCoreEngine, NV_TRUE);

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, pAesClient->Engine);

    while (TotalBytesToProcess)
    {
        // Request the H/W semaphore before accesing the AES H/W
        AesCoreRequestHwAccess();

        // In the bootloader version entire buffer is processed for AES operation
#if NV_OAL
        BytesToProcess = TotalBytesToProcess;
        TotalBytesToProcess = 0;
#else
        // At OS level only AES_HW_MAX_PROCESS_SIZE_BYTES will be processed.
        // Once the AES H/W lock is acquired again then remaining bytes or maximum of
        // AES_HW_MAX_PROCESS_SIZE_BYTES will be processed.
        if (TotalBytesToProcess > AES_HW_MAX_PROCESS_SIZE_BYTES)
            BytesToProcess = AES_HW_MAX_PROCESS_SIZE_BYTES;
        else
            BytesToProcess = TotalBytesToProcess;
#endif

        if ((!pAesClient->IsDedicatedSlot) && (NvDdkAesKeyType_UserSpecified == pAesClient->KeyType))
        {
            // If it is not dedicated slot, unwrap the key
            NvU8 UnWrappedRFCIv[AES_RFC_IV_LENGTH_BYTES];

            NV_CHECK_ERROR_CLEANUP(AesCoreGetFreeSlot(pAesClient->pAesCoreEngine, &Engine, &KeySlot));
            pAesClient->Engine = Engine;
            pAesClient->KeySlot = KeySlot;

            // Unwrap the key
            NV_CHECK_ERROR_CLEANUP(AesCoreUnWrapKey(
                pAesClient->pAesCoreEngine,
                pAesClient->Key,
                pAesClient->WrappedIv,
                pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine],
                UnWrappedRFCIv));
            // Check whether the key unwrap is success or not by comparing the unwrapped RFC IV with original RFC IV
            if (NvOsMemcmp(UnWrappedRFCIv, gs_OriginalIV, sizeof(gs_OriginalIV)))
            {
                // Unwrap key failed
                e = NvError_AesKeyUnWrapFailed;
                goto fail;
            }

            // Setup Key table
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwSetKeyAndIv);
            pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwSetKeyAndIv(
                pAesClient->Engine,
                pAesClient->KeySlot,
                (AesHwKey *)(pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine]),
                (AesHwIv *)pAesClient->Iv,
                pAesClient->IsEncryption,
                pAesHwCtxt);

            // Memset the local variable to zeros where the key is stored
            NvOsMemset(
                (pAesHwCtxt->pKeyTableVirAddr[Engine] + pAesHwCtxt->KeyTableSize[Engine]),
                0,
                NvDdkAesKeySize_128Bit);
        }
        else
        {
            // Select Key slot
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwSelectKeyIvSlot);
            pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwSelectKeyIvSlot(
                pAesClient->Engine,
                pAesClient->KeySlot,
                pAesHwCtxt);

            // Set the last IV operated with this client
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwSetIv);
            NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwSetIv(
                pAesClient->Engine,
                pAesClient->KeySlot,
                (AesHwIv *)pAesClient->Iv,
                pAesHwCtxt));
        }

        // Process the buffer for encryption/decryption
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwStartEngine);
        e = pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwStartEngine(
            pAesClient->Engine,
            BytesToProcess,
            pSrcBuffer,
            pAesClient->IsEncryption,
            pAesClient->OpMode,
            pDestBuffer,
            pAesHwCtxt);
        if (NvSuccess != e)
        {
            // If the key is user specified and not in dedicated slot, clear it
            if ((!pAesClient->IsDedicatedSlot) && (NvDdkAesKeyType_UserSpecified == pAesClient->KeyType))
            {
                // Clear key
                NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwClearKeyAndIv);
                pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwClearKeyAndIv(
                    pAesClient->Engine,
                    pAesClient->KeySlot,
                    pAesHwCtxt);
            }
            goto fail;
        }

        // Store the last IV operated with this client
        NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwGetIv);
        pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwGetIv(
            pAesHwCtxt,
            pAesClient->Engine,
            pAesClient->KeySlot,
            (AesHwIv *)pAesClient->Iv);

        // If the key is user specified and not in dedicated slot, clear it
        if ((!pAesClient->IsDedicatedSlot) && (NvDdkAesKeyType_UserSpecified == pAesClient->KeyType))
        {
            // Clear key
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwClearKeyAndIv);
            pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwClearKeyAndIv(
                pAesClient->Engine,
                pAesClient->KeySlot,
                pAesHwCtxt);
        }
        else
        {
            // Clear the IV in the engine
            NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwClearIv);
            pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwClearIv(
                pAesClient->Engine,
                pAesClient->KeySlot,
                pAesHwCtxt);
        }

#if !NV_OAL
        pSrcBuffer += BytesToProcess;
        pDestBuffer += BytesToProcess;
        TotalBytesToProcess -= BytesToProcess;
#endif

        // Release the H/W semaphore
        AesCoreReleaseHwAccess();
    }

    // Clear the DMA buffer before we leave from this operation
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[pAesClient->Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[pAesClient->Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NvSuccess;

fail:
    // Release the H/W semaphore
    AesCoreReleaseHwAccess();
    // Clear the DMA buffer before we leave from this operation
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[pAesClient->Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[pAesClient->Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);
    AesCorePowerDown(gs_pAesCoreEngine, NV_TRUE);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return e;
}

/**
 * Clear the User Key.
 *
 * @param pAesClient Pointer to AES client.
 *
 * @retval NvSuccess if User key is cleared, NV_FALSE otherwise.
 */
NvError AesCoreClearUserKey(const NvDdkAes *const pAesClient)
{
    AesHwContext *pAesHwCtxt = NULL;
    NvBool IsSuccess = NV_TRUE;

    NV_ASSERT(pAesClient);

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    // Get the AES H/W context
    NV_ASSERT(pAesClient->pAesCoreEngine);
    pAesHwCtxt = &pAesClient->pAesCoreEngine->AesHwCtxt;

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, pAesClient->Engine);

    AesCoreRequestHwAccess();

    // Clear the key and IV
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, pAesClient->Engine, AesHwClearKeyAndIv);
    pAesHwCtxt->ppEngineCaps[pAesClient->Engine]->pAesInterf->AesHwClearKeyAndIv(
        pAesClient->Engine,
        pAesClient->KeySlot,
        pAesHwCtxt);

    AesCoreReleaseHwAccess();

    IsSuccess = AesCoreIsUserKeyCleared(pAesClient->Engine, pAesClient->KeySlot, pAesHwCtxt);
    // Clear dedicated slot wrapped key info in local buffer
    NvOsMemset(gs_pAesCoreEngine->DedicatedSlotKeyInfo[pAesClient->Engine][pAesClient->KeySlot].WrappedKey,
        0,
        NvDdkAesConst_MaxKeyLengthBytes);
    NvOsMemset(gs_pAesCoreEngine->DedicatedSlotKeyInfo[pAesClient->Engine][pAesClient->KeySlot].WrappedIv,
        0,
        AES_RFC_IV_LENGTH_BYTES);

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return IsSuccess;
}

/**
 * Check the user key clear by encrypting the known data.
 *
 * @param Engine Engine on which encryption need to be performed.
 * @param KeySlot Key slot where encrypt key is located.
 * @param pAesHw Pointer to AES H/W context.
 *
 * @retval NV_TRUE if encryption and decryption is successfully done else NV_FALSE
 */
NvBool
AesCoreIsUserKeyCleared(
    const AesHwEngine Engine,
    const AesHwKeySlot KeySlot,
    AesHwContext *const pAesHwCtxt)
{
    NvError e = NvSuccess;
    AesHwIv ZeroIv;
    NvU32 i;

    // Known Good data
    static NvU8 s_GoldData[NvDdkAesConst_BlockLengthBytes] =
    {
        0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F
    };

    // Encrypted data for above known data with Zero key and Zero IV
    static NvU8 s_EncryptDataWithZeroKeyTable[NvDdkAesConst_BlockLengthBytes] =
    {
        0x7A, 0xCA, 0x0F, 0xD9,
        0xBC, 0xD6, 0xEC, 0x7C,
        0x9F, 0x97, 0x46, 0x66,
        0x16, 0xE6, 0xA2, 0x82
    };

    // Encrypted data for above known data with Zero key and Zero IV
    static NvU8 s_EncryptDataWithZeroKeySchedule[NvDdkAesConst_BlockLengthBytes] =
    {
        0x18, 0x9D, 0x19, 0xEA,
        0xDB, 0xA7, 0xE3, 0x0E,
        0xD9, 0x72, 0x80, 0x8F,
        0x3F, 0x2B, 0xA0, 0x30
    };

    NvU8 *pEncryptData;
    NvU8 EncryptBuffer[NvDdkAesConst_BlockLengthBytes];

    NvOsMutexLock(gs_hAesCoreEngineMutex);

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, Engine);

    if (pAesHwCtxt->ppEngineCaps[Engine]->IsHwKeySchedGenSupported)
        pEncryptData = s_EncryptDataWithZeroKeyTable;
    else
        pEncryptData = s_EncryptDataWithZeroKeySchedule;

    NvOsMemset(EncryptBuffer, 0, NvDdkAesConst_BlockLengthBytes);
    NvOsMemset(&ZeroIv, 0, sizeof(AesHwIv));

    AesCoreRequestHwAccess();

    // Select Encrypt Key slot
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSelectKeyIvSlot(Engine, KeySlot, pAesHwCtxt);

    // Set the Zero IV for test data
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwSetIv);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwSetIv(
        Engine,
        KeySlot,
        &ZeroIv,
        pAesHwCtxt));

    // Process the buffer for encryption
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwStartEngine);
    NV_CHECK_ERROR_CLEANUP(pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwStartEngine(
        Engine,
        NvDdkAesConst_BlockLengthBytes,
        s_GoldData,
        NV_TRUE,
        NvDdkAesOperationalMode_Cbc,
        EncryptBuffer,
        pAesHwCtxt));

    // Clear the IV in the engine before we leave
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, Engine, AesHwClearIv);
    pAesHwCtxt->ppEngineCaps[Engine]->pAesInterf->AesHwClearIv(Engine, KeySlot, pAesHwCtxt);

    AesCoreReleaseHwAccess();

    // Clear the DMA buffer before we leave from this operation
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);

    for (i = 0; i < NvDdkAesConst_BlockLengthBytes; i++)
    {
        if (pEncryptData[i] != EncryptBuffer[i])
            goto fail;
    }

    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_TRUE;

fail:
    // Clear the DMA buffer before we leave from this operation
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[Engine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[Engine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);
    NvOsMutexUnlock(gs_hAesCoreEngineMutex);
    return NV_FALSE;
}

/**
 * Return the AES module capabilities.
 *
 * @param hRmDevice Rm Device handle.
 * @param ppCaps Pointer to pointer to capbilities structure.
 *
 * @retval NvSuccess if capabilities could be returned successfully.
 *
 */
NvError AesCoreGetCapabilities(const NvRmDeviceHandle hRmDevice, AesHwCapabilities **const ppCaps)
{
    NvError e = NvSuccess;

    static AesHwCapabilities s_Caps1;
    static AesHwCapabilities s_Caps2;

    static NvRmModuleCapability s_EngineA_Caps[] =
    {
        {1, 0, 0, &s_Caps1},
        {1, 1, 0, &s_Caps1},
        {1, 2, 0, &s_Caps2},
        {1, 3, 0, &s_Caps2}
    };

    static NvRmModuleCapability s_engineB_caps[] =
    {
        {1, 0, 0, &s_Caps1},
        {1, 1, 0, &s_Caps2},
        {1, 2, 0, &s_Caps2}
    };

    NV_ASSERT(hRmDevice);
    NV_ASSERT(ppCaps);

    NvOsMemset(&s_Caps1, 0, sizeof(s_Caps1));
    NvOsMemset(&s_Caps2, 0, sizeof(s_Caps2));

    s_Caps1.IsHashSupported = NV_FALSE;
    s_Caps1.IsHwKeySchedGenSupported = NV_FALSE;
    s_Caps1.MinBufferAlignment = 16;
    s_Caps1.MinKeyTableAlignment = 256;
    s_Caps1.NumSlotsSupported = AesHwKeySlot_Num;
    s_Caps1.pAesInterf = NULL;

    s_Caps2.IsHashSupported = NV_FALSE;
    s_Caps2.IsHwKeySchedGenSupported = NV_TRUE;
    s_Caps2.HwKeySchedLengthBytes = 80;
    s_Caps2.MinBufferAlignment = 16;
    s_Caps2.MinKeyTableAlignment = 4;
    s_Caps2.NumSlotsSupported = AesHwKeySlot_NumExt;
    s_Caps2.pAesInterf = NULL;

    NV_CHECK_ERROR_CLEANUP(NvRmModuleGetCapabilities(
        hRmDevice,
        NvRmModuleID_Vde,
        s_EngineA_Caps,
        NV_ARRAY_SIZE(s_EngineA_Caps),
        (void **)&(ppCaps[AesHwEngine_A])));

    if (ppCaps[AesHwEngine_A] == &s_Caps2)
    {
        if (NULL == s_Caps2.pAesInterf)
        {
            s_Caps2.pAesInterf = NvOsAlloc(sizeof(AesHwInterface));
            if (NULL == s_Caps2.pAesInterf)
                goto fail;
            NvAesIntfAp20GetHwInterface(s_Caps2.pAesInterf);
        }
    }

    NV_CHECK_ERROR_CLEANUP(NvRmModuleGetCapabilities(
        hRmDevice,
        NvRmModuleID_BseA,
        s_engineB_caps,
        NV_ARRAY_SIZE(s_engineB_caps),
        (void **)&(ppCaps[AesHwEngine_B])));

    if (ppCaps[AesHwEngine_B] == &s_Caps2)
    {
        if (NULL == s_Caps2.pAesInterf)
        {
            s_Caps2.pAesInterf = NvOsAlloc(sizeof(AesHwInterface));
            if (NULL == s_Caps2.pAesInterf)
                goto fail;
            NvAesIntfAp20GetHwInterface(s_Caps2.pAesInterf);
        }
    }

fail:
    return e;
}

/**
 * Encrypt/Decrypt the given input buffer using Electronic CodeBook (ECB) mode.
 * Prerequisite:It is caller responsibility to acquire hardware lock before using this function.
 *
 * @param pAesCoreEngine Pointer to AES core engine.
 * @param pInputBuffer Pointer to input bufffer.
 * @param BufSize Buffer size.
 * @param IsEncrypt If set to NV_TRUE, encrypt the input buffer else decrypt it.
 * @param pOutputBuffer Pointer to output buffer.
 *
 * @retval NvSuccess if successfully completed.
 */
NvError
AesCoreEcbProcessBuffer(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pInputBuffer,
    const NvU32 BufSize,
    const NvBool IsEncrypt,
    NvU8 *const pOutputBuffer)
{
    NvError e = NvSuccess;
    AesHwEngine SskEngine;
    AesHwKeySlot SskKeySlot;
    AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);
    NV_ASSERT(pInputBuffer);
    NV_ASSERT(pOutputBuffer);

    // Get the AES H/W context
    pAesHwCtxt = (AesHwContext *)&pAesCoreEngine->AesHwCtxt;

    SskEngine = pAesCoreEngine->SskEngine[0];
    SskKeySlot = (IsEncrypt ? pAesCoreEngine->SskEncryptSlot : pAesCoreEngine->SskDecryptSlot);

    NVDDK_AES_CHECK_INTERFACE(pAesHwCtxt, SskEngine);

    // Select SSK key for processing
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SskEngine, AesHwSelectKeyIvSlot);
    pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwSelectKeyIvSlot(SskEngine, SskKeySlot, pAesHwCtxt);

    // Process the buffer
    NVDDK_AES_CHECK_INTERFACE_FUNC(pAesHwCtxt, SskEngine, AesHwStartEngine);
    e = pAesHwCtxt->ppEngineCaps[SskEngine]->pAesInterf->AesHwStartEngine(
        SskEngine,
        BufSize,
        pInputBuffer,
        IsEncrypt,
        NvDdkAesOperationalMode_Ecb,
        pOutputBuffer,
        pAesHwCtxt);

    // Clear the DMA buffer
    NV_ASSERT(pAesHwCtxt->pDmaVirAddr[SskEngine]);
    NvOsMemset(pAesHwCtxt->pDmaVirAddr[SskEngine], 0, AES_HW_DMA_BUFFER_SIZE_BYTES);

    return e;
}

/**
 * Wrap the given key data using RFC 3394 algoritham.
 *
 * Follwing is RFC3394 key wrap algoritham.
 *     Inputs:  Plaintext, n 64-bit values {P1, P2, ..., Pn}, and
 *             Key, K (the KEK).
 *    Outputs: Ciphertext, (n+1) 64-bit values {C0, C1, ..., Cn}.

 *    1) Initialize variables.
 *        Set A = IV, an initial value (see 2.2.3)
 *        For i = 1 to n
 *            R[i] = P[i]
 *    2) Calculate intermediate values.
 *        For j = 0 to 5
 *            For i=1 to n
 *                B = AES(K, A | R[i])
 *                A = MSB(64, B) ^ t where t = (n*j)+i
 *                R[i] = LSB(64, B)
 *    3) Output the results.
 *        Set C[0] = A
 *        For i = 1 to n
 *            C[i] = R[i]
 *
 * @param pAesCoreEngine Pointer to AES core engine.
 * @param pOrgKey Pointer to Original Key.
 * @param pOrgIv Pointer to Original Iv which is used in RFC3394 algorithm.
 * @param pWrappedKey Pointer to wrapped key.
 * @param pWrappedIv Pointer to wrapped Iv.
 *
 */
NvError
AesCoreWrapKey(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pOrgKey,
    const NvU8 *const pOrgIv,
    NvU8 *const pWrappedKey,
    NvU8 *const pWrappedIv)
{
    NvError e = NvSuccess;
    NvU8 n = AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY, i, j, k, t;
    NvU8 *A, *B;
    NvU8 **R;
    const AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);
    NV_ASSERT(pOrgKey);
    NV_ASSERT(pOrgIv);
    NV_ASSERT(pWrappedKey);
    NV_ASSERT(pWrappedIv);

    // Get the AES H/W context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    // Local buffers which are used for processing should be in IRAM.
    // Use KeyTable buffer which is in IRAM.
    // The local variables should be of following format and sizes.
    // NvU8 A[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES];
    // NvU8 B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES * AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY];
    // NvU8 R[AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY][AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES];

    A = pAesHwCtxt->pKeyTableVirAddr[0];
    B = A + AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES;

    R = (NvU8 **)(B + (AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES));
    for(i = 0; i < AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY; i++)
    {
        R[i] = ((B + (AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES)) +
            (AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES * sizeof(NvU8 *)) +
            (i * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES));
    }

    // Set A = IV
    for (i = 0; i < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; i++)
        A[i] =  pOrgIv[i];

    //For i = 1 to n 	R[i] = P[i]
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; j++)
            R[i][j] = pOrgKey[j + (i *AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES)];
    }

    // Calculate intermediate values.
    //  For j = 0 to 5
    //      For i=1 to n
    //          B = AES(K, A | R[i])
    //          A = MSB(64, B) ^ t where t = (n*j)+i
    //          R[i] = LSB(64, B)
    for (j = 0; j <= 5; j++)
    {
        for (i = 0; i < n; i++)
        {
            for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
            {
                B[k] = A[k];
                B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES+k] = R[i][k];
            }
            NV_CHECK_ERROR(AesCoreEcbProcessBuffer(pAesCoreEngine, B, NvDdkAesKeySize_128Bit, NV_TRUE, B));
            for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
            {
                A[k] =  B[k];
                R[i][k] = B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES + k];
            }
            t = (n * j) + (i+1);
            A[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES-1] ^=  t;
        }
    }

    // Output the results.
    // Set C[0] = A
    // For i = 1 to n
    //    C[i] = R[i]
    for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++) {
        pWrappedIv[k] = A[k];
    }
    for (i = 0; i < n; i++)
    {
        for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
        {
            pWrappedKey[(AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES*i) + k] = R[i][k];
        }
    }

    // Clear the buffers.
    NvOsMemset(pAesHwCtxt->pKeyTableVirAddr[0], 0, pAesHwCtxt->KeyTableSize[0]);
    return e;
}

/**
 * UnWrapKey the given key data using RFC 3394 algorithm.
 *
 * Follwing is RFC3394 key unwrap algoritham.
 *  Inputs:  Ciphertext, (n+1) 64-bit values {C0, C1, ..., Cn}, and
 *             Key, K (the KEK).
 *    Outputs: Plaintext, n 64-bit values {P0, P1, K, Pn}.
 *
 *    1) Initialize variables.
 *        Set A = C[0]
 *        For i = 1 to n
 *            R[i] = C[i]
 *    2) Compute intermediate values.
 *        For j = 5 to 0
 *            For i = n to 1
 *                B = AES-1(K, (A ^ t) | R[i]) where t = n*j+i
 *                A = MSB(64, B)
 *                R[i] = LSB(64, B)
 *    3) Output results.
 *    If A is an appropriate initial value (see 2.2.3),
 *    Then
 *        For i = 1 to n
 *            P[i] = R[i]
 *    Else
 *        Return an error
 *
 * @param pAesCoreEngine Pointer to AES core engine.
 * @param pWrappedKey Pointer to wrapped key
 * @param pWrappedIv Pointer to wrapped Iv
 * @param pOrgKey Pointer to Original Key.
 * @param pOrgIv Pointer to Original Iv which is used in RFC3394 algorithm.
 *
 */
NvError
AesCoreUnWrapKey(
    const AesCoreEngine *const pAesCoreEngine,
    const NvU8 *const pWrappedKey,
    const NvU8 *const pWrappedIv,
    NvU8 *const pOrgKey,
    NvU8 *const pOrgIv)
{
    NvError e = NvSuccess;
    NvS32 n = AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY, i, j, k, t;
    NvU8 *A, *B;
    NvU8 **R;
    const AesHwContext *pAesHwCtxt;

    NV_ASSERT(pAesCoreEngine);
    NV_ASSERT(pWrappedKey);
    NV_ASSERT(pWrappedIv);
    NV_ASSERT(pOrgKey);
    NV_ASSERT(pOrgIv);

    // Get the AES H/W context
    pAesHwCtxt = &pAesCoreEngine->AesHwCtxt;

    // Local buffers which are used for processing should in IRAM.
    // Use KeyTable buffer which is in IRAM.
    // The local variables should be of following format and sizes.
    // NvU8 A[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES];
    // NvU8 B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES * AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY];
    // NvU8 R[AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY][AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES];

    A = pAesHwCtxt->pKeyTableVirAddr[0];
    B = A + AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES;
    R = (NvU8 **)(B + (AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES));
    for(i = 0; i < AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY; i++)
    {
        R[i] = ((B + (AES_RFC_3394_NUM_OF_BLOCKS_FOR_128BIT_KEY * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES)) +
            (AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES * sizeof(NvU8 *)) +
            (i * AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES));
    }

    // Set A = C[0]
    for (i = 0; i < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; i++)
        A[i] = pWrappedIv[i];

    // For i = 1 to n R[i] = C[i]
    for (i = 0; i < n; i++) {
        for (j = 0; j < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; j++)
            R[i][j] = pWrappedKey[j + (i *AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES)];
    }

    // Compute intermediate values.
    //   For j = 5 to 0
    //       For i = n to 1
    //           B = AES-1(K, (A ^ t) | R[i]) where t = n*j+i
    //           A = MSB(64, B)
    //           R[i] = LSB(64, B)
    for (j = 5; j >= 0; j--)
    {
        for (i = n; i > 0; i--)
        {
            t = (n * j) + (i);
            A[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES-1] ^=  t;
            for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
            {
                B[k] = A[k];
                B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES+k] = R[i-1][k];
            }
            NV_CHECK_ERROR(AesCoreEcbProcessBuffer(pAesCoreEngine, B, NvDdkAesKeySize_128Bit, NV_FALSE, B));
            for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
            {
                A[k] =  B[k];
                R[i-1][k] = B[AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES + k];
            }
        }
    }

    // Output results.
    // For i = 1 to n
    // P[i] = R[i]
    for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
    {
        pOrgIv[k] = A[k];
    }

    for (i = 0; i < n; i++)
    {
        for (k = 0; k < AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES; k++)
        {
            pOrgKey[(AES_RFC_3394_KEY_WRAP_BLOCK_SIZE_BYTES*i) + k] = R[i][k];
        }
    }

    // Clear the buffers.
    NvOsMemset(pAesHwCtxt->pKeyTableVirAddr[0], 0, pAesHwCtxt->KeyTableSize[0]);
    return e;
}
