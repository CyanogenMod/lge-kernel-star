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

/** 
 * @file
 * @brief <b>nVIDIA Driver Development Kit: 
 *           DMA Resource manager private API for Hw access </b>
 *
 * @b Description: Implements the private interface of the nvrm dma to access
 * the hw apb dma register.
 *
 * This files implements the API for accessing the register of the Dma 
 * controller and configure the dma transfers.
 */

#include "nvrm_dma.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "rm_dma_hw_private.h"
#include "ap15/arapbdma.h"
#include "ap15/arapbdmachan.h"
#include "nvrm_drf.h"
#include "nvassert.h"

#define APBDMACHAN_READ32(pVirtBaseAdd, reg) \
        NV_READ32((pVirtBaseAdd) + ((APBDMACHAN_CHANNEL_0_##reg##_0)/4))
#define APBDMACHAN_WRITE32(pVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32(((pVirtBaseAdd) + ((APBDMACHAN_CHANNEL_0_##reg##_0)/4)), (val)); \
    } while(0)


/**
 * Global Enable/disable the apb dma controller.
 */
static void GlobalSetApbDma(NvU32 *pGenVirtBaseAddress, NvBool IsEnable)
{
    NvU32 CommandRegs = 0;

    // Read the apb dma command register.
    CommandRegs = NV_READ32((pGenVirtBaseAddress + (APBDMA_COMMAND_0/4)));

    // Enable/disable the global enable bit of this register.
    if(IsEnable)
        CommandRegs = NV_FLD_SET_DRF_DEF(APBDMA, COMMAND, GEN, ENABLE, CommandRegs);
    else
        CommandRegs = NV_FLD_SET_DRF_DEF(APBDMA, COMMAND, GEN, DISABLE, CommandRegs);

    // Write into the register.
    NV_WRITE32( (pGenVirtBaseAddress + ( APBDMA_COMMAND_0/4)),CommandRegs);
}


/**
 * Configure the address registers of the apb dma for data transfer.
 */
static void
ConfigureApbDmaAddress(
    DmaChanRegisters *pDmaChRegs,
    NvRmPhysAddr SourceAdd,
    NvRmPhysAddr DestAdd,
    NvBool IsSourceAddPeripheralType)
{
    pDmaChRegs->ApbAddressReg = (IsSourceAddPeripheralType)? SourceAdd: DestAdd;
    pDmaChRegs->AhbAddressReg = (IsSourceAddPeripheralType)? DestAdd: SourceAdd;
}


/**
 * Set the data transfer size for the apb dma.
 */
static void
SetApbDmaTransferSize(
    DmaChanRegisters *pDmaChRegs,
    NvU32 TransferSize,
    NvBool IsDoubleBuffMode)
{
    // If double buff mode the programmed word count will be half of the data 
    // request.
    NvU32 WordCount = (IsDoubleBuffMode)? (TransferSize >> 3): (TransferSize >> 2);

    // Configure the word count in the control register.
    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
                                        (WordCount-1), pDmaChRegs->ControlReg);
}


/**
 * Add the transferred count for apb dma.
 */
static void AddApbDmaTransferredCount(DmaChanRegisters *pDmaChRegs)
{
    NvU32 ProgrammedWordCount;

    // Get the programmed transfer count.
    ProgrammedWordCount = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
                                            pDmaChRegs->ControlReg);
    ProgrammedWordCount = (ProgrammedWordCount +1) << 2;
    pDmaChRegs->TransferedCount += ProgrammedWordCount;

    // Limiting the transfer count to not be more than 2 times
    if (pDmaChRegs->TransferedCount > (ProgrammedWordCount << 1))
        pDmaChRegs->TransferedCount = ProgrammedWordCount << 1;
}

static void AckNClearApbDmaInterrupt(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaStatusReg;
    // Get the status of the dma channel.
    DmaStatusReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, STA);

    // Write 1 on clear
    if (DmaStatusReg & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, ISE_EOC, INTR))
        APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, STA, DmaStatusReg);
}
/**
 * Check whether the dma transfer is completed or not for the given channel.
 */
static NvBool IsApbDmaTransferCompleted(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaStatusReg;
    
    // Get the status of the dma channel.
    DmaStatusReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, STA);
    if (DmaStatusReg & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, ISE_EOC, INTR))
    {
        // Write the status to clear it
        APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, STA, DmaStatusReg);
        return NV_TRUE;
    }    
    else
        return NV_FALSE;
}

/**
 * Get the transferred count for apb dma.
 */
static NvU32 GetApbDmaTransferredCount(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaStatusReg;
    NvU32 ProgrammedWordCount;
    NvU32 RemainingWordCount;
    NvU32 TransferedSize;
    NvU32 RetTransferSize;

    // Get the status of the dma channel.
    DmaStatusReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, STA);
    ProgrammedWordCount = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
                                            pDmaChRegs->ControlReg);
    RemainingWordCount = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, STA, COUNT, DmaStatusReg);
    if (IsApbDmaTransferCompleted(pDmaChRegs))
        AddApbDmaTransferredCount(pDmaChRegs);
        
    if (DmaStatusReg & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, BSY, ACTIVE))
    {
        if (RemainingWordCount)
            TransferedSize = (ProgrammedWordCount - RemainingWordCount);
        else
            TransferedSize = (ProgrammedWordCount);
    }
    else
    {
        TransferedSize = (ProgrammedWordCount +1 );
    }
    RetTransferSize = (TransferedSize << 2) + pDmaChRegs->TransferedCount;
    pDmaChRegs->TransferedCount = 0;
    return (RetTransferSize);
}


/**
 * Get the transferred count for apb dma.
 */
static NvU32 GetApbDmaTransferredCountWithStop(
    DmaChanRegisters *pDmaChRegs,
    NvBool IsTransferStop)
{
    NvU32 DmaStatusReg;
    NvU32 FlowCtrlReg;
    NvU32 ProgrammedWordCount;
    NvU32 RemainingWordCount;
    NvU32 TransferedSize;
    NvU32 RetTransferSize;

    if (IsApbDmaTransferCompleted(pDmaChRegs))
        AddApbDmaTransferredCount(pDmaChRegs);

    if (IsTransferStop)
    {
        FlowCtrlReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, CSR);
        FlowCtrlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    CSR, REQ_SEL, NA31,
                                    FlowCtrlReg);
        APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, FlowCtrlReg);
    }

    // Get the status of the dma channel.
    DmaStatusReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, STA);
    ProgrammedWordCount = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
                                            pDmaChRegs->ControlReg);
    RemainingWordCount = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, STA, COUNT, DmaStatusReg);
    if (DmaStatusReg & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, BSY, ACTIVE))
    {
        if (RemainingWordCount)
            TransferedSize = (ProgrammedWordCount - RemainingWordCount);
        else
            TransferedSize = (ProgrammedWordCount);
    }
    else
    {
        TransferedSize = (ProgrammedWordCount+1);
    }
    RetTransferSize = (TransferedSize << 2) + pDmaChRegs->TransferedCount;
    pDmaChRegs->TransferedCount = 0;
    return (RetTransferSize);
}

/**
 * Set the dma burst size in the dma registers.
 */
static void
SetDmaBurstSize(
    DmaChanRegisters *pDmaChRegs,
    NvRmDmaModuleID DmaReqModuleId,
    NvU32 TransferSize)
{
    // Check for the dma requestor Id and based on the requestor and module Id
    // Select the burst size.
    switch (DmaReqModuleId)
    {
        case NvRmDmaModuleID_Uart:
        case NvRmDmaModuleID_I2c:
        case NvRmDmaModuleID_Dvc:
        
            // Dma requestor is the uart/I2c/DvcI2c.
            // Set the dma burst size to 1 words.
            pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        AHB_SEQ, AHB_BURST, DMA_BURST_1WORDS,
                                        pDmaChRegs->AhbSequenceReg);
            break;

        case NvRmDmaModuleID_I2s:
        case NvRmDmaModuleID_Spdif:
            // Dma requestor is the i2s.
            pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        AHB_SEQ, AHB_BURST, DMA_BURST_4WORDS,
                                        pDmaChRegs->AhbSequenceReg);
            break;

        case NvRmDmaModuleID_Slink:
        case NvRmDmaModuleID_Spi:
            // Dma requestor is the spi/slink.
            if ((TransferSize & 0xF) == 0)
            {
                // Multiple of 4 words
                pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        AHB_SEQ, AHB_BURST, DMA_BURST_4WORDS,
                                        pDmaChRegs->AhbSequenceReg);
            }
            else
            {
                // Non multiple of 4 words
                pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        AHB_SEQ, AHB_BURST, DMA_BURST_1WORDS,
                                        pDmaChRegs->AhbSequenceReg);
            }
            break;

        case NvRmDmaModuleID_Vfir:
        case NvRmDmaModuleID_Mipi:
            if ((TransferSize & 0x1F)) 
            {
                // Non multiple of 8 words
                if (TransferSize & 0xF)
                {
                    // Non multiple of 4 words
                    pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(
                                        APBDMACHAN_CHANNEL_0, AHB_SEQ, AHB_BURST, 
                                        DMA_BURST_1WORDS, pDmaChRegs->AhbSequenceReg);
                }
                else
                {
                    pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(
                                        APBDMACHAN_CHANNEL_0, AHB_SEQ, AHB_BURST, 
                                        DMA_BURST_4WORDS, pDmaChRegs->AhbSequenceReg);
                }
            }
            else
            {
                pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(
                                        APBDMACHAN_CHANNEL_0, AHB_SEQ, AHB_BURST, 
                                        DMA_BURST_8WORDS, pDmaChRegs->AhbSequenceReg);
            }
            break;

        default:
            NV_ASSERT(!"Invalid module");
    }
}


/**
 * Enable the bit swap for the destionation address for apb dma.
 */
static void
EnableApbDmaDestBitSwap(
    DmaChanRegisters *pDmaChRegs,
    NvBool IsDestAddPeripheralType)
{
    // Source to destination address.
    if (IsDestAddPeripheralType)
    {
        // Enable the bit swap to the Peripheral address.
        pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    APB_SEQ, APB_DATA_SWAP, ENABLE,
                                    pDmaChRegs->ApbSequenceReg);
    }
    else
    {
        // Enable the bit swap to the memory address.
        pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, 
                                    AHB_SEQ, AHB_DATA_SWAP, 1,
                                    pDmaChRegs->AhbSequenceReg);
    }
}

/**
 * Set the address wrapping information for apb dma.
 * The different address wrapping is supported by APB dma.
 */
static void
SetApbDmaAddressWrapping(
    DmaChanRegisters *pDmaChRegs,
    NvRmPhysAddr SourceAddWrap,
    NvRmPhysAddr DestAddWrap,
    NvU32 TransferSize,
    NvBool IsSourceAddPeripheralType)
{
    NvU32 ApbWrapSizeInWords;
    NvU32 AhbWrapSizeInWords;

    // Supported address wrap on ahb side. These are in word (4 bytes) count.
    NvU32 SupportedAhbSideAddWrapSize[8] = {0, 32, 64, 128, 256, 512,1024, 2048};

    // Supported address wrap on apb side. These are in words (4 bytes) count.
    NvU32 SupportedApbSideAddWrapSize[8] = {0, 1, 2, 4, 8, 16, 32, 64};

    int MaxSupportedTable = 8;
    int ApbWrapIndex = 0;
    int AhbWrapIndex = 0;

    // Converting the address wrapping size in words and storing in the
    // variable as per source and destination module type.
    ApbWrapSizeInWords = (IsSourceAddPeripheralType)? SourceAddWrap: DestAddWrap;
    AhbWrapSizeInWords = (IsSourceAddPeripheralType)? DestAddWrap : SourceAddWrap;

    ApbWrapSizeInWords = ApbWrapSizeInWords >> 2;
    AhbWrapSizeInWords = AhbWrapSizeInWords >> 2;
    
    // Check for the supported address wrap for APB Side
    for (ApbWrapIndex = 0; ApbWrapIndex < MaxSupportedTable; ++ApbWrapIndex)
    {
        if (ApbWrapSizeInWords == SupportedApbSideAddWrapSize[ApbWrapIndex])
            break;
    }
    NV_ASSERT(ApbWrapIndex < MaxSupportedTable);

    // Check for the supported address wrap for AHB Side
    for (AhbWrapIndex = 0; AhbWrapIndex < MaxSupportedTable; ++AhbWrapIndex)
    {
        if (AhbWrapSizeInWords == SupportedAhbSideAddWrapSize[AhbWrapIndex])
            break;
    }
    NV_ASSERT(AhbWrapIndex < MaxSupportedTable);

    // Configure the registers.
    pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0,
                                    APB_SEQ, APB_ADDR_WRAP, ApbWrapIndex, 
                                    pDmaChRegs->ApbSequenceReg);

    pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0,
                                    AHB_SEQ, WRAP, AhbWrapIndex, 
                                    pDmaChRegs->AhbSequenceReg);
}

/**
 * Start the apb dma transfer from the current request. This will read the
 * dma register information  from the dma configuration register and program the
 * dma register and start the transfer.
 */
static void StartApbDmaTransfer(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaStartCommand;

    pDmaChRegs->TransferedCount = 0;

    // Write configured data into the hw register of dma.
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, pDmaChRegs->ControlReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, AHB_SEQ, pDmaChRegs->AhbSequenceReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, AHB_PTR, pDmaChRegs->AhbAddressReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, APB_SEQ, pDmaChRegs->ApbSequenceReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, APB_PTR, pDmaChRegs->ApbAddressReg);

    // Start the dma transfer.
    DmaStartCommand = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, ENABLE,
                                                pDmaChRegs->ControlReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, DmaStartCommand);
}

/**
 * Continue the apb dma transfer special for the continuous mode.
 */
static void ContinueApbDmaTransfer(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaStartCommand;
    NvU32 CurrControlReg;
    NvU32 NewControlReg;

    NewControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, ENABLE, 
                                        pDmaChRegs->ControlReg);
    CurrControlReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, CSR);

    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, AHB_PTR, pDmaChRegs->AhbAddressReg);

    // Write the control register only when there is difference between the 
    // current setting and new setting.
    if (NewControlReg != CurrControlReg)
    {
        // Start the dma transfer.
        DmaStartCommand = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, 
                                ENABLE, pDmaChRegs->ControlReg);
        APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, DmaStartCommand);
    }
}

/**
 * Start the Apb dma transfer from the current request. This will read the
 * current configured address from the register and increment them as per
 * passed parameter and start the dma transfer.
 */
static void
StartApbDmaWithAddInc(
    DmaChanRegisters *pDmaChRegs,
    NvU32 PeriAddIncSize,
    NvU32 MemoryAddIncSize,
    NvU32 IsContMode)
{
    NvU32 NewControlReg;
    NvU32 CurrControlReg;
    NvU32 AhbAddress;

    NewControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, ENABLE,
                                    pDmaChRegs->ControlReg);

    // Read the addresses programmed in the dma hw registers.
    AhbAddress = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, AHB_PTR);

    // Increment the address and write back the new address.
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, AHB_PTR, 
                                            (AhbAddress + MemoryAddIncSize));

    // If it is continuous mode and old control information is same as the new 
    // one then need nt to rewrite the control resgiter.
    if (IsContMode)
    {
        CurrControlReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, CSR);
        if (CurrControlReg == NewControlReg)
            return;
    }
    // Start the dma transfer.
    NewControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, ENABLE,
                            NewControlReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, NewControlReg);
}



/**
 * Stop the data transfer in the given APB/AHB dma channel number.
 */
static void StopApbDmaTransfer(DmaChanRegisters *pDmaChRegs)
{
    NvU32 DmaCommandReg;

    // Stop the dma transfer.
    // First disable the interrupt and then diasable the dma enable bit.
    DmaCommandReg = APBDMACHAN_READ32(pDmaChRegs->pHwDmaChanReg, CSR);
    DmaCommandReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, IE_EOC, DISABLE, 
                            DmaCommandReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, DmaCommandReg);

    DmaCommandReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, DISABLE, 
                            DmaCommandReg);
    APBDMACHAN_WRITE32(pDmaChRegs->pHwDmaChanReg, CSR, DmaCommandReg);
    AckNClearApbDmaInterrupt(pDmaChRegs);
}


/**
 * Tells whether the given address is valid peripheral device address or not.
 */
NvBool NvRmPrivDmaHwIsValidPeripheralAddress(NvRmPhysAddr PhysAddress)
{
    NvU32 Address32Bit;
    NvU32 MostSignificantNibble;

    // Get the most significant nibble
    Address32Bit = (NvU32)PhysAddress;
    MostSignificantNibble = Address32Bit >> 28;

    // Only address start at 7XXX:XXXX address are the valid device address.
    if (MostSignificantNibble == 7)
        return NV_TRUE;
    return NV_FALSE;    
}

void NvRmPrivDmaInitDmaHwInterfaces(DmaHwInterface *pApbDmaInterface)
{
    pApbDmaInterface->DmaHwGlobalSetFxn = GlobalSetApbDma;
    pApbDmaInterface->DmaHwConfigureAddressFxn = ConfigureApbDmaAddress;
    pApbDmaInterface->DmaHwSetTransferSizeFxn =  SetApbDmaTransferSize;
    pApbDmaInterface->DmaHwGetTransferredCountFxn = GetApbDmaTransferredCount;
    pApbDmaInterface->DmaHwGetTransferredCountWithStopFxn = GetApbDmaTransferredCountWithStop;
    pApbDmaInterface->DmaHwAddTransferCountFxn = AddApbDmaTransferredCount;
    pApbDmaInterface->DmaHwSetBurstSizeFxn =  SetDmaBurstSize;
    pApbDmaInterface->DmaHwEnableDestBitSwapFxn = EnableApbDmaDestBitSwap;
    pApbDmaInterface->DmaHwSetAddressWrappingFxn = SetApbDmaAddressWrapping;
    pApbDmaInterface->DmaHwStartTransferFxn = StartApbDmaTransfer;
    pApbDmaInterface->DmaHwContinueTransferFxn = ContinueApbDmaTransfer;
    pApbDmaInterface->DmaHwStartTransferWithAddIncFxn = StartApbDmaWithAddInc;
    pApbDmaInterface->DmaHwStopTransferFxn = StopApbDmaTransfer;
    pApbDmaInterface->DmaHwIsTransferCompletedFxn = IsApbDmaTransferCompleted;
    pApbDmaInterface->DmaHwAckNClearInterruptFxn = AckNClearApbDmaInterrupt;

    NvRmPrivDmaInitAp15DmaHwInterfaces(pApbDmaInterface);
}

