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

#ifndef INCLUDED_NVDDK_AES_PRIV_H
#define INCLUDED_NVDDK_AES_PRIV_H

#include "nverror.h"
#include "nvcommon.h"
#include "nvrm_memmgr.h"
#include "nvddk_aes_hw.h"

#if defined(__cplusplus)
extern "C" {
#endif

// As of now only 5 commands are used for AES encryption/decryption
#define AES_HW_MAX_ICQ_LENGTH 5

// AES RFC 3394 initial vector length, in bytes
#define AES_RFC_IV_LENGTH_BYTES 8

/**
 * AES Engine instances.
 */
typedef enum
{
    // AES Engine "A" BSEV
    AesHwEngine_A,
    // AES Engine "B" BSEA
    AesHwEngine_B,
    AesHwEngine_Num,
    AesHwEngine_Force32 = 0x7FFFFFFF
} AesHwEngine;

/**
 * AES Key Slot instances (per AES engine).
 */
typedef enum
{
    AesHwKeySlot_0,
    AesHwKeySlot_1,
    AesHwKeySlot_2,
    AesHwKeySlot_3,
    AesHwKeySlot_Num,
    AesHwKeySlot_4 = AesHwKeySlot_Num,
    AesHwKeySlot_5,
    AesHwKeySlot_6,
    AesHwKeySlot_7,
    AesHwKeySlot_NumExt,
    AesHwKeySlot_Force32 = 0x7FFFFFFF
} AesHwKeySlot;

// Iv Context
typedef struct AesIvContextRec
{
    // Updated/current Iv for each key slot
    NvU32 CurIv[AesHwKeySlot_NumExt][AES_HW_IV_LENGTH];
    // The current key slot in use
    AesHwKeySlot CurKeySlot;
} AesIvContext;

// Wrapped key slot information context
typedef struct AesWrappedKeyContextRec
{
    // Wrapped key for dedicated slot
    NvU8 WrappedKey[NvDdkAesConst_MaxKeyLengthBytes];
    // Wrapped Iv for dedicated slot
    NvU8 WrappedIv[AES_RFC_IV_LENGTH_BYTES];
    // KeySlot used for encrypiton or decryption
    NvBool IsEncryption;
} AesWrappedKeyContext;

typedef struct AesHwInterfaceRec AesHwInterface;

// AES engine capabilities
typedef struct AesHwCapabilitiesRec
{
    // Number of slots supported in each AES instance
    AesHwKeySlot NumSlotsSupported;
    // Key schedule generation in hardware to be supported
    NvBool IsHwKeySchedGenSupported;
    // Size needed for key scheduling
    NvU32 HwKeySchedLengthBytes;
    // Hashing to be supported within the engine
    NvBool IsHashSupported;
    // Minimum Alignment for the key table/schedule buffer in IRAM or VRAM
    NvU32 MinKeyTableAlignment;
    // Min Input/Output buffer alignment for AES
    NvU32 MinBufferAlignment;
    // Pointer to the AES engine low level interface
    AesHwInterface *pAesInterf;
} AesHwCapabilities;

// H/W Context
typedef struct AesHwContextRec
{
    // Capabilities
    AesHwCapabilities **ppEngineCaps;
    // Mutex to support concurrent operation on the AES engine
    NvOsMutexHandle Mutex[AesHwEngine_Num];
    // RM device handle
    NvRmDeviceHandle hRmDevice;
    // Controller registers physical base address
    NvRmPhysAddr PhysAdr[AesHwEngine_Num];
    // Holds the virtual address for accessing registers
    NvU32 *pVirAdr[AesHwEngine_Num];
    // Holds the register map size
    NvU32 BankSize[AesHwEngine_Num];
    // Memory handle to the key table
    NvRmMemHandle hKeyTableMemBuf;
    // Pointer to the key table virtual buffer
    NvU8 *pKeyTableVirAddr[AesHwEngine_Num];
    // Key table physical addr
    NvRmPhysAddr KeyTablePhyAddr[AesHwEngine_Num];
    // Key table size in bytes
    NvU32 KeyTableSize[AesHwEngine_Num];
    // Memory handle to the AES H/W Buffer
    NvRmMemHandle hDmaMemBuf;
    // Contains the physical Address of the H/W buffer
    NvRmPhysAddr DmaPhyAddr[AesHwEngine_Num];
    // Virtual pointer to the AES buffer
    NvU8 *pDmaVirAddr[AesHwEngine_Num];
    // Icq commands length
    NvU32 CommandQueueLength[AesHwEngine_Num];
    // Holds the Icq commands for the AES operation
    NvU32 CommandQueueData[AesHwEngine_Num][AES_HW_MAX_ICQ_LENGTH];
    // Iv Context for each AES engine
    AesIvContext IvContext[AesHwEngine_Num];
} AesHwContext;

// AES Core Engine record
typedef struct AesCoreEngineRec
{
    // Dedicated slot Key Information
    AesWrappedKeyContext DedicatedSlotKeyInfo[AesHwEngine_Num][AesHwKeySlot_NumExt];
    // Keeps the count of open handles
    NvU32 OpenCount;
    // Id returned from driver's registration with Power Manager
    NvU32 RmPwrClientId[AesHwEngine_Num];
    // Indicates whether key slot is used or not
    NvBool IsKeySlotUsed[AesHwEngine_Num][AesHwKeySlot_NumExt];
    // SSK engine where SSK is stored
    AesHwEngine SskEngine[AesHwEngine_Num];
    // SSK encrypt key slot
    AesHwKeySlot SskEncryptSlot;
    // SSK Decrypt key slot
    AesHwKeySlot SskDecryptSlot;
    // SBK engine where SBK is stored
    AesHwEngine SbkEngine[AesHwEngine_Num];
    // SBK encrypt key slot
    AesHwKeySlot SbkEncryptSlot;
    // SSK Decrypt key slot
    AesHwKeySlot SbkDecryptSlot;
    // Aes H/W Engine context
    AesHwContext AesHwCtxt;
    // Indicates whether engine is disabled or not
    NvBool IsEngineDisabled;
    // Indicates whether ssk update is allowed or not
    NvBool SskUpdateAllowed;
} AesCoreEngine;

// Set of function pointers to be used to access the hardware interface for
// the AES engines.
struct AesHwInterfaceRec
{
    /**
     * Disable the selected AES engine. No further operations can be
     * performed using the AES engine until the entire chip is reset.
     *
     * @param pAesHwCtxt Pointer to the AES H/W context.
     * @param Engine AES engine to disable.
     *
     * @retval NvSuccess if engine successfully disabled else NvError_AesDisableCryptoFailed.
     */
    NvError (*AesHwDisableEngine)(
        const AesHwContext *const pAesHwCtxt,
        const AesHwEngine Engine);

    /**
     * Over-write the key schedule and Initial Vector in the in the specified
     * key slot with zeroes.Convenient for preventing subsequent callers from
     * gaining access to a previously-used key.
     *
     * @param Engine AES engine.
     * @param Slot key slot to clear.
     * @param pAesHwCtxt Pointer to the AES H/W context.
     *
     * @retval None.
     */
    void (*AesHwClearKeyAndIv)(
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        AesHwContext *const pAesHwCtxt);

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
    void (*AesHwClearIv)(
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        AesHwContext *const pAesHwCtxt);

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
    void (*AesHwSetKeyAndIv)(
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        const AesHwKey *const pKey,
        const AesHwIv *const pIv,
        const NvBool IsEncryption,
        AesHwContext *const pAesHwCtxt);

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
    NvError (*AesHwSetIv)(
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        const AesHwIv *const pIv,
        AesHwContext *const pAesHwCtxt);

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
    void (*AesHwGetIv)(
        const AesHwContext *const pAesHwCtxt,
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        AesHwIv *const pIv);

    /**
     * Lock the Secure Session Key (SSK) slots.
     * This API disables the read/write permissions to the secure key slots.
     *
     * @param pAesHwCtxt Pointer to the AES H/W context.
     * @param SskEngine SSK engine number.
     *
     * @retval None.
     */
    void (*AesHwLockSskReadWrites)(
        const AesHwContext *const pAesHwCtxt,
        const AesHwEngine SskEngine);

    /**
     * Select the key and iv from the internal key table for a specified key slot.
     *
     * @param Engine AES engine.
     * @param Slot Key slot for which key and IV needs to be selected.
     * @param pAesHwCtxt Pointer to the AES H/W context.
     *
     * @retval None.
     */
    void (*AesHwSelectKeyIvSlot)(
        const AesHwEngine Engine,
        const AesHwKeySlot Slot,
        AesHwContext *const pAesHwCtxt);

    /**
     * Encrypt/Decrypt a specified number of blocks of cyphertext using
     * Cipher Block Chaining (CBC) mode.  A block is 16 bytes.
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
    NvError (*AesHwStartEngine)(
        const AesHwEngine Engine,
        const NvU32 DataSize,
        const NvU8 *const pSrc,
        const NvBool IsEncryption,
        const NvDdkAesOperationalMode OpMode,
        NvU8 *const pDest,
        AesHwContext *const pAesHwCtxt);

    /**
     * Load the SSK key into secure scratch resgister and disable the write permissions.
     * Note: If Key is not specified then this API locks the Secure Scratch registers.
     *
     * @param PmicBaseAddr PMIC base address.
     * @param pKey Pointer to the key. If pKey=NULL then key will not be set to the
     *             secure scratch registers, but locks the Secure scratch register.
     * @param Size Length of the aperture in bytes.
     *
     * @retval None.
     */
    void (*AesHwLoadSskToSecureScratchAndLock)(
        const NvRmPhysAddr PmicBaseAddr,
        const AesHwKey *const pKey,
        const size_t Size);

    /**
     * Mark all dedicated slots as used.
     *
     * @param pAesCoreEngine Pointer to AES Core Engine.
     *
     * @retval None.
     */
    void (*AesHwGetUsedSlots)(AesCoreEngine *const pAesCoreEngine);

    /**
     * Read the AES engine disable status.
     *
     * @param pAesHwCtxt Pointer to the AES H/W context.
     * @param Engine AES engine to disable.
     *
     * @return NV_TRUE if engine is disabled else NV_FALSE.
     */
    NvBool (*AesHwIsEngineDisabled)(const AesHwContext *const pAesHwCtxt, const AesHwEngine Engine);

    /**
     * Disables read access to all key slots for the given engine.
     *
     * @param pAesHwCtxt Pointer to the AES H/W context
     * @param Engine AES engine for which key reads needs to be disabled
     * @param NumSlotsSupported Number of key slots supported in the engine
     *
     * @retval None
     */
    void (*AesHwDisableAllKeyRead)(
        const AesHwContext *const pAesHwCtxt,
        const AesHwEngine Engine,
        const AesHwKeySlot NumSlotsSupported);

    /**
     * Queries whether SSK update is allowed or not
     *
     * @retval NV_TRUE if SSK update is allowed
     * @retval NV_FALSE if SSK update is not allowed
     */
    NvBool (*AesHwIsSskUpdateAllowed)(void);
};

// AES client state: this structure is common to all clients
typedef struct NvDdkAesRec
{
    // Algorithm type set for this client
    NvDdkAesOperationalMode OpMode;
    // Select key type -- SBK, SSK or user-specified
    NvDdkAesKeyType KeyType;
    // AES key length; must be 128Bit for SBK or SSK
    NvDdkAesKeySize KeySize;
    // Specified AES key value if KeyType is user-specified; else ignored
    NvU8 Key[NvDdkAesConst_MaxKeyLengthBytes];
    // Initial vector to use when encrypting/decrypting last IV will be stored
    NvU8 Iv[NvDdkAesConst_IVLengthBytes];
    // Initial vector to use in RFC3394 key unwrapping
    NvU8 WrappedIv[AES_RFC_IV_LENGTH_BYTES];
    // Client selected AES engine, depends on the key slot selected
    AesHwEngine Engine;
    // Client selected Key slot
    AesHwKeySlot KeySlot;
    // Operation type selected
    NvBool IsEncryption;
    // If user key is specified client can tell to use the dedicated slot
    // If set to NV_TRUE then key slot is dedicated else shared with other keys
    NvBool IsDedicatedSlot;
    // Pointer to the AES core engine
    AesCoreEngine *pAesCoreEngine;
    // Client's user id
    uid_t uid;
    // Client's group id
    gid_t gid;
} NvDdkAes;

/**
 * Return the interface to be used for AP20 AES engine.
 *
 * @param pAp20AesHw Pointer to the interface.
 *
 * @retval None.
 */
void NvAesIntfAp20GetHwInterface(AesHwInterface *const pAp20AesHw);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVDDK_AES_PRIV_H
