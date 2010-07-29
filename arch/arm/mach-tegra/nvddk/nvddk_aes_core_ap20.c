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
#include "ap20/arvde_bsev_aes.h"
#include "ap20/aravp_bsea_aes.h"
#include "ap20/arapbpm.h"
#include "nvddk_aes_common.h"
#include "nvddk_aes_priv.h"
#include "nvddk_aes_hw.h"
#include "nvddk_aes_core_ap20.h"
#include <linux/interrupt.h>
#include "../board.h"

#define SECURE_HW_REGR(engine, viraddr, reg, value) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        (value) = NV_READ32((NvU32)(viraddr) + (ARVDE_BSEV_##reg##_0)); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        (value) = NV_READ32((NvU32)(viraddr) + (AVPBSEA_##reg##_0)); \
    } \
}

#define SECURE_HW_REGW(engine, viraddr, reg, data) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        NV_WRITE32((NvU32)(viraddr) + (ARVDE_BSEV_##reg##_0), (data)); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        NV_WRITE32((NvU32)(viraddr) + (AVPBSEA_##reg##_0), (data)); \
    } \
}

#define SECURE_DRF_SET_VAL(engine, reg, field, newData, value) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        value = NV_FLD_SET_DRF_NUM(ARVDE_BSEV, reg, field, newData, value); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        value = NV_FLD_SET_DRF_NUM(AVPBSEA, reg, field, newData, value); \
    } \
}

#define SECURE_DRF_READ_VAL(engine, reg, field, regData, value) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        value = NV_DRF_VAL(ARVDE_BSEV, reg, field, regData); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        value = NV_DRF_VAL(AVPBSEA, reg, field, regData); \
    } \
}

#define SECURE_DRF_NUM(engine, reg, field, num) \
    NV_DRF_NUM(ARVDE_BSEV, reg, field, num) \

#define SECURE_INDEXED_REGR(engine, viraddr, index, value) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        (value) = NV_READ32((NvU32)(viraddr) + ARVDE_BSEV_SECURE_SEC_SEL0_0 + ((index) * 4)); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        (value) = NV_READ32((NvU32)(viraddr) + AVPBSEA_SECURE_SEC_SEL0_0 + ((index) * 4)); \
    } \
}

#define SECURE_INDEXED_REGW(engine, viraddr, index, value) \
{ \
    if (AesHwEngine_A == engine) \
    { \
        NV_WRITE32((NvU32)(viraddr) + (ARVDE_BSEV_SECURE_SEC_SEL0_0 + ((index) * 4)), (value)); \
    } \
    else if (AesHwEngine_B == engine) \
    { \
        NV_WRITE32((NvU32)(viraddr) + (AVPBSEA_SECURE_SEC_SEL0_0 + ((index) * 4)), (value)); \
    } \
}

#define NV_ADDRESS_MAP_IRAM_A_BASE 0x40000000

static NvBool NvAesCoreAp20IsEngineBusy(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr);

/**
 *  Define Ucq opcodes required for AES operation.
 */
typedef enum
{
    // Opcode for Block Start Engine command
    AesUcqOpcode_BlockStartEngine = 0x0E,
    // Opcode for Dma setup command
    AesUcqOpcode_DmaSetup = 0x10,
    // Opcode for Dma Finish Command
    AesUcqOpcode_DmaFinish = 0x11,
    // Opcode for Table setup command
    AesUcqOpcode_SetupTable = 0x15,
    AesUcqOpcode_Force32 = 0x7FFFFFFF
} AesUcqOpcode;

/**
 *  Define Aes command values.
 */
typedef enum
{
    // Command value for AES Table select
    AesUcqCommand_TableSelect = 0x3,
    // Command value for Keytable selection
    AesUcqCommand_KeyTableSelect = 0x8,
    // Command value for KeySchedule selection
    AesUcqCommand_KeySchedTableSelect = 0x4,
    // Command mask for ketable address mask
    AesUcqCommand_KeyTableAddressMask = 0x1FFFF,
    AesUcqCommand_Force32 = 0x7FFFFFFF
} AesUcqCommand;

/**
 * @brief Define AES Interactive command Queue commands Bit positions.
 */
typedef enum
{
    // Define bit position for command Queue Opcode
    AesIcqBitShift_Opcode = 26,
    // Define bit position for AES Table select
    AesIcqBitShift_TableSelect = 24,
    // Define bit position for AES Key Table select
    AesIcqBitShift_KeyTableId = 17,
    // Define bit position for AES Key Table Address
    AesIcqBitShift_KeyTableAddr = 0,
    // Define bit position for 128 bit blocks count
    AesIcqBitShift_BlockCount = 0,
    AesIcqBitShift_Force32 = 0x7FFFFFFF
} AesIcqBitShift;

NvError NvAesCoreAp20DisableEngine(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr)
{
    NvU32 RegValue = 0;

    // Disable the AES engine
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_SECURITY, RegValue);
    SECURE_DRF_SET_VAL(Engine, SECURE_SECURITY, SECURE_ENG_DIS, 1, RegValue);
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_SECURITY, RegValue);

    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_SECURITY, 0);
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_SECURITY, RegValue);

    if (RegValue == SECURE_DRF_NUM(Engine, SECURE_SECURITY, SECURE_ENG_DIS, 1))
        return  NvSuccess;
    return NvError_AesDisableCryptoFailed;
}

NvBool NvAesCoreAp20IsEngineDisabled(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr)
{
    NvU32 RegValue = 0;
    NvU32 EngineStatus = 0;

    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_SECURITY, RegValue);
    SECURE_DRF_READ_VAL(Engine, SECURE_SECURITY, SECURE_ENG_DIS, RegValue, EngineStatus);

    return (NvBool)EngineStatus;
}

/**
 * Query the status of the engine.
 *
 * @param Engine The engine for which status is required.
 * @param pEngineVirAddr AES engine virtual address.
 *
 * @retval NV_TRUE if the engine is busy.
 * @retval NV_FALSE if the engine is not busy.
 */
NvBool NvAesCoreAp20IsEngineBusy(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr)
{
    NvU32 RegValue = 0;
    NvU32 EngBusy = 0;
    NvU32 IcqEmpty = 0;
    NvU32 DmaBusy = 0;

    SECURE_HW_REGR(Engine, pEngineVirAddr, INTR_STATUS, RegValue);

    // Extract the EngBusy, IcqEmpty and DmaBusy status
    SECURE_DRF_READ_VAL(Engine, INTR_STATUS, ENGINE_BUSY, RegValue, EngBusy);
    SECURE_DRF_READ_VAL(Engine, INTR_STATUS, ICQ_EMPTY, RegValue, IcqEmpty);
    SECURE_DRF_READ_VAL(Engine, INTR_STATUS, DMA_BUSY, RegValue, DmaBusy);

    // Check for engine busy, ICQ not empty and DMA busy
    if ((EngBusy) || (!IcqEmpty) || (DmaBusy))
    {
        // Return TRUE if any of the condition is true
        return NV_TRUE;
    }

    // Return FALSE if engine is not doing anything
    return NV_FALSE;
}

void NvAesCoreAp20WaitTillEngineIdle(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr)
{
    while (NvAesCoreAp20IsEngineBusy(Engine, pEngineVirAddr));
}

void
NvAesCoreAp20SetupTable(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 KeyTablePhyAddr,
    const NvU32 Slot)
{
    NvU32 SetupTableCmd = ((AesUcqOpcode_SetupTable << AesIcqBitShift_Opcode) |
        (AesUcqCommand_TableSelect << AesIcqBitShift_TableSelect) |
        ((AesUcqCommand_KeyTableSelect | Slot) << AesIcqBitShift_KeyTableId) |
        ((KeyTablePhyAddr & AesUcqCommand_KeyTableAddressMask) <<
        AesIcqBitShift_KeyTableAddr));

    NvAesCoreAp20WaitTillEngineIdle(Engine, pEngineVirAddr);

    // Issue the ICQ command to update the table to H/W registers
    SECURE_HW_REGW(Engine, pEngineVirAddr, ICMDQUE_WR, SetupTableCmd);

    NvAesCoreAp20WaitTillEngineIdle(Engine, pEngineVirAddr);
}

void NvAesCoreAp20SelectKeyIvSlot(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr, const NvU32 Slot)
{
    NvU32 RegValue = 0;

    // Select the KEY slot for updating the IV vectors
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_CONFIG, RegValue);
    // 2-bit index to select between the 4- keys
    SECURE_DRF_SET_VAL(Engine, SECURE_CONFIG, SECURE_KEY_INDEX, Slot, RegValue);
    // Update the AES config register
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_CONFIG, RegValue);
}

void
NvAesCoreAp20SetIv(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 Slot,
    const NvU32 Start,
    const NvU32 End,
    const NvU32 *const pKeyTable,
    const NvU32 KeyTableSize,
    const NvU32 *const pIvAddress)
{
    // Setting the iv will be done through decryption of Iv
    // because Iv can't be written without overwriting the key
    // especially in cases when key is not readable.
}

void
NvAesCoreAp20GetIv(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 Slot,
    const NvU32 Start,
    const NvU32 End,
    const NvU32 *const pIvAddress)
{
    // The Iv is preserved within the driver because read permission is locked
    // down for dedicated key slots
}

void
NvAesCoreAp20ControlKeyScheduleGeneration(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvBool IsEnabled)
{
    NvU32 RegValue = 0;

    // Disable key schedule generation in hardware
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_CONFIG_EXT, RegValue);
    SECURE_DRF_SET_VAL(Engine, SECURE_CONFIG_EXT, SECURE_KEY_SCH_DIS, (IsEnabled ? 0 : 1), RegValue);
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_CONFIG_EXT, RegValue);
}

void NvAesCoreAp20LockSskReadWrites(const AesHwEngine Engine, const NvU32 *const pEngineVirAddr)
{
    NvU32 RegValue = 0;

    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_SEC_SEL4, RegValue);
    SECURE_DRF_SET_VAL(Engine, SECURE_SEC_SEL4, KEYREAD_ENB4, 0, RegValue);
    SECURE_DRF_SET_VAL(Engine, SECURE_SEC_SEL4, KEYUPDATE_ENB4, 0, RegValue);
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_SEC_SEL4, RegValue);
}

void
NvAesCoreAp20ProcessBuffer(
    const AesHwEngine Engine,
    const NvU32 *const pEngineVirAddr,
    const NvU32 SrcPhyAddress,
    const NvU32 DestPhyAddress,
    const NvU32 NumBlocks,
    const NvBool IsEncryption,
    const NvDdkAesOperationalMode OpMode)
{
    NvU32 CommandQueueData[AES_HW_MAX_ICQ_LENGTH];
    NvU32 CommandQueueLength = 0;
    NvU32 RegValue = 0;
    NvU32 EngBusy = 0;
    NvU32 IcqEmpty = 0;
    NvU32 i;

    // Setup DMA command
    CommandQueueData[CommandQueueLength++] = (AesUcqOpcode_DmaSetup << AesIcqBitShift_Opcode);
    CommandQueueData[CommandQueueLength++] = SrcPhyAddress;

    // Setup Block Start Engine Command
    CommandQueueData[CommandQueueLength++] =
        ((AesUcqOpcode_BlockStartEngine << AesIcqBitShift_Opcode) |
        ((NumBlocks - 1) << AesIcqBitShift_BlockCount));

    // Setup DMA finish command
    CommandQueueData[CommandQueueLength++] = (AesUcqOpcode_DmaFinish << AesIcqBitShift_Opcode);

    // Wait for engine to become idle
    NvAesCoreAp20WaitTillEngineIdle(Engine, pEngineVirAddr);

    // Configure command Queue control register
    SECURE_HW_REGR(Engine, pEngineVirAddr, CMDQUE_CONTROL, RegValue);

    if (AesHwEngine_A == Engine)
    {
        RegValue |=
            // Source Stream interface select,
            // (SRC_STM_SEL = 0: through CIF (SDRAM)),
            // and (SRC_STM_SEL = 1: through AHB (SDRAM/IRAM)).
            SECURE_DRF_NUM(Engine, CMDQUE_CONTROL, SRC_STM_SEL, (SrcPhyAddress & NV_ADDRESS_MAP_IRAM_A_BASE) ? 1 : 0) |
            // Destination Stream interface select,
            // (DST_STM_SEL = 0: through CIF (SDRAM)),
            // and (DST_STM_SEL = 1: through AHB (SDRAM/IRAM)).
            SECURE_DRF_NUM(Engine, CMDQUE_CONTROL, DST_STM_SEL, (DestPhyAddress& NV_ADDRESS_MAP_IRAM_A_BASE) ? 1 : 0);
    }
    else
    {
        RegValue |=
            // Source Stream interface select,
            // (SRC_STM_SEL = 1: through AHB (SDRAM/IRAM)).
            SECURE_DRF_NUM(Engine, CMDQUE_CONTROL, SRC_STM_SEL, 1) |
            // Destination Stream interface select,
            // (DST_STM_SEL = 1: through AHB (SDRAM/IRAM)).
            SECURE_DRF_NUM(Engine, CMDQUE_CONTROL, DST_STM_SEL, 1);
    }

    // Update the Command Queue control register
    SECURE_HW_REGW(Engine, pEngineVirAddr, CMDQUE_CONTROL, RegValue);

    RegValue = SECURE_DRF_NUM(Engine, BSE_CONFIG, BSE_MODE_SEL, 0) |
        // Endian conversion enable for the bit stream
        // (ENDIAN_ENB = 1: little Endian stream)
        SECURE_DRF_NUM(Engine, BSE_CONFIG, ENDIAN_ENB, 1);

    // Update the BSE config register
    SECURE_HW_REGW(Engine, pEngineVirAddr, BSE_CONFIG, RegValue);

    // Read the AES config register
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_CONFIG_EXT, RegValue);

    // Set counter offset to '0'
    SECURE_DRF_SET_VAL(Engine, SECURE_CONFIG_EXT, SECURE_OFFSET_CNT, 0, RegValue);
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_CONFIG_EXT, RegValue);

    // Configure the AES extension register for KEY and IV select
    SECURE_HW_REGR(Engine, pEngineVirAddr, SECURE_INPUT_SELECT, RegValue);
    SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_IV_SELECT, 1, RegValue);

    // Select AES operation mode for configuring the AES config register
    //////////////////////////////////////////////////////////////////////////
    switch (OpMode)
    {
        case NvDdkAesOperationalMode_Cbc:
            // Configuration for CBC mode of operation:
            // ----------------------------------------------------------------------
            //                  |   Encryption              |       Decryption
            //-----------------------------------------------------------------------
            // SECURE_XOR_POS      |   10b (top)               |   11b (bottom)
            // SECURE_INPUT_SEL    |   00b (AHB)               |   00b (AHB)
            // SECURE_VCTRAM_SEL   |   10b (IV + AES Output)   |   11b (IV_PreAHB)
            // SECURE_CORE_SEL     |    1b (AES)               |    0b (inverse AES)
            //////////////////////////////////////////////////////////////////////////
            /* AES configuration for Encryption/Decryption */
            // AES XOR position 10b = '2': top, before AES
            // AES XOR position 11b = '3': bottom, after AES
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_XOR_POS, (IsEncryption ? 2 : 3), RegValue);
            // AES input select 0?b = '00' or '01': From AHB input vector
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_INPUT_SEL, 0, RegValue);
            // Vector RAM select 10b = '2': Init Vector for first round and AES
            // output for the rest rounds.
            // Vector RAM select 11b = '3': Init Vector for the first round
            // and previous AHB input for the rest rounds
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_VCTRAM_SEL, (IsEncryption ? 2 : 3), RegValue);
            // AES core selection 1b = '1': Encryption
            // Inverse AES core selection 0b = '0': Decryption
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_CORE_SEL, (IsEncryption ? 1 : 0), RegValue);
            // Disable the random number generator
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_RNG_ENB, 0, RegValue);
            break;
        case NvDdkAesOperationalMode_Ecb:
            // Configuration for ECB mode of operation:
            // ----------------------------------------------------------------------
            //                  |   Encryption              |       Decryption
            //-----------------------------------------------------------------------
            // SECURE_XOR_POS      |   0X (bypass)               |   0X (bypass)
            // SECURE_INPUT_SEL    |   00b (AHB)               |   00b (AHB)
            // SECURE_VCTRAM_SEL   |   XX(don’t care)   |   XX(don’t care)
            // SECURE_CORE_SEL     |    1b (AES)               |    0b (inverse AES)
            //////////////////////////////////////////////////////////////////////////
            /* AES configuration for Encryption/Decryption */
            // For ECB mode, XOR position shoud be bypassed
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_XOR_POS, 0, RegValue);
            // AES input select is zero
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_INPUT_SEL, 0, RegValue);
            // AES core selection 1b = '1': Encryption
            // Inverse AES core selection 0b = '0': Decryption
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_CORE_SEL, (IsEncryption ? 1 : 0), RegValue);
            // Disable the random number generator
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_RNG_ENB, 0, RegValue);
            break;
        case NvDdkAesOperationalMode_AnsiX931:
            // For ECB mode, XOR position shoud be bypassed
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_XOR_POS, 0, RegValue);
            // AES input select is zero
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_INPUT_SEL, 0, RegValue);
            // AES core selection 1b = '1': Encryption
            // Inverse AES core selection 0b = '0': Decryption
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_CORE_SEL, (IsEncryption ? 1 : 0), RegValue);
            // To generate a random number, enable the random number generator
            SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_RNG_ENB, 1, RegValue);
            break;
        default:
            return;
    }

    SECURE_DRF_SET_VAL(Engine, SECURE_INPUT_SELECT, SECURE_HASH_ENB, 0, RegValue);
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_INPUT_SELECT, RegValue);

    // Update the AES destination physical address
    SECURE_HW_REGW(Engine, pEngineVirAddr, SECURE_DEST_ADDR, DestPhyAddress);

    // Issue the AES commands to the ICQ
    for (i = 0; i < CommandQueueLength; i++)
    {
        // Wait till engine becomes IDLE before issuing any command
        do
        {
            SECURE_HW_REGR(Engine, pEngineVirAddr, INTR_STATUS, RegValue);
            SECURE_DRF_READ_VAL(Engine, INTR_STATUS, ENGINE_BUSY, RegValue, EngBusy);
            SECURE_DRF_READ_VAL(Engine, INTR_STATUS, ICQ_EMPTY, RegValue, IcqEmpty);
        } while ((EngBusy) && ~(IcqEmpty));
        // Write the command to the ICQ register
        SECURE_HW_REGW(Engine, pEngineVirAddr, ICMDQUE_WR, CommandQueueData[i]);
    }

    // Wait for engine to become idle
    NvAesCoreAp20WaitTillEngineIdle(Engine, pEngineVirAddr);
}

void
NvAesCoreAp20LoadSskToSecureScratchAndLock(
    const NvU32 PmicBaseAddr,
    const NvU32 *const pKey,
    const size_t Size)
{
    NvU32 *pSecureScratch = NULL;
    NvU32 *pPmicBaseAddr = NULL;

    NvRmPhysicalMemMap(
        PmicBaseAddr,
        Size,
        NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void **)&pPmicBaseAddr);

    // Get the secure scratch base address
    pSecureScratch = pPmicBaseAddr + (APBDEV_PMC_SECURE_SCRATCH0_0 / 4);

    // If key is supplied then load it into the scratch registers
    if (pKey)
    {
        NvU32 i;

        // Load the SSK into the secure scratch registers
        for (i = 0; i < AES_HW_KEY_LENGTH; i++)
        {
            NV_WRITE32(pSecureScratch, pKey[i]);
            pSecureScratch++;
        }
    }

    // Disable write access to the secure scratch register
    NV_WRITE32((pPmicBaseAddr) + (APBDEV_PMC_SEC_DISABLE_0 / 4), (NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE, ON)));

    // UnMap the virtual address
    NvRmPhysicalMemUnmap(pPmicBaseAddr, Size);
}

void NvAesCoreAp20KeyReadDisable(
    const AesHwEngine Engine,
    const AesHwKeySlot Slot,
    const NvU32 *const pEngineVirAddr)
{
    NvU32 RegValue = 0;

    SECURE_INDEXED_REGR(Engine, pEngineVirAddr, Slot, RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(ARVDE_BSEV, SECURE_SEC_SEL0, KEYREAD_ENB0, 0, RegValue);
    SECURE_INDEXED_REGW(Engine, pEngineVirAddr, Slot, RegValue);
}

NvBool NvAesCoreAp20IsSskUpdateAllowed(void)
{
    if (tegra_is_ap20_a03())
    {
        // It is AO3 chip
        // Check whether it is AO3P or not
        // SSK update is not supported on AO3 board
        if (!tegra_is_ap20_a03p())
            return NV_FALSE;
    }
    // Except AO3, all other chips support SSK update
    return NV_TRUE;
}
