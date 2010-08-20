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
 *           Private functions implementation for the spi Ddk driver</b>
 *
 * @b Description:  Implements the private functions for the spi hw interface.
 * 
 */
 
#include "rm_spi_slink_hw_private.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"

// hardware includes
#include "ap15/arspi.h"

#define SPI_REG_READ32(pSpiHwRegsVirtBaseAdd, reg) \
        NV_READ32((pSpiHwRegsVirtBaseAdd) + ((SPI_##reg##_0)/4))
#define SPI_REG_WRITE32(pSpiHwRegsVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32((((pSpiHwRegsVirtBaseAdd) + ((SPI_##reg##_0)/4))), (val)); \
    } while (0)

#define MAX_SPI_FIFO_DEPTH 4

#define RESET_ALL_CS \
        (NV_DRF_DEF(SPI, COMMAND,  CS0_EN, ENABLE) | \
         NV_DRF_DEF(SPI, COMMAND,  CS1_EN, ENABLE) | \
         NV_DRF_DEF(SPI, COMMAND,  CS2_EN, ENABLE) | \
         NV_DRF_DEF(SPI, COMMAND,  CS3_EN, ENABLE))

#define ALL_SPI_STATUS_CLEAR \
        (NV_DRF_NUM(SPI, STATUS,  RDY, 1) | \
         NV_DRF_NUM(SPI, STATUS,  RXF_UNR, 1) | \
         NV_DRF_NUM(SPI, STATUS,  TXF_OVF, 1))

static void
SpiHwSetSignalMode(
    SerialHwRegisters *pSpiHwRegs, 
    NvOdmQuerySpiSignalMode SignalMode);

/**
 * Initialize the spi register.
 */
static void
SpiHwRegisterInitialize(
    NvU32 SerialInstanceId, 
    SerialHwRegisters *pSpiHwRegs)
{
    NvU32 CommandReg;
    pSpiHwRegs->InstanceId = SerialInstanceId;
    pSpiHwRegs->pRegsBaseAdd = NULL;
    pSpiHwRegs->RegBankSize = 0;
    pSpiHwRegs->HwTxFifoAdd = SPI_TX_FIFO_0;
    pSpiHwRegs->HwRxFifoAdd = SPI_RX_FIFO_0;
    pSpiHwRegs->IsPackedMode = NV_FALSE;
    pSpiHwRegs->PacketLength = 1;
    pSpiHwRegs->CurrSignalMode = NvOdmQuerySpiSignalMode_Invalid;
    pSpiHwRegs->MaxWordTransfer = MAX_SPI_FIFO_DEPTH;
    pSpiHwRegs->IsLsbFirst = NV_FALSE;
    pSpiHwRegs->IsMasterMode = NV_TRUE;
    pSpiHwRegs->IsNonWordAlignedPackModeSupported = NV_TRUE;
    pSpiHwRegs->IsHwChipSelectSupported = NV_FALSE;

    CommandReg = NV_RESETVAL(SPI, COMMAND);
    // Initialize the chip select bits to select the s/w only
    CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_SOFT, 1, CommandReg);
    CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_VAL, 1, CommandReg);

    if (pSpiHwRegs->IsIdleDataOutHigh)
        CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SDA, DRIVE_HIGH, CommandReg);
    else
        CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SDA, DRIVE_LOW, CommandReg);
    
    pSpiHwRegs->HwRegs.SpiRegs.Command = CommandReg;
    pSpiHwRegs->HwRegs.SpiRegs.Status = NV_RESETVAL(SPI, STATUS);
    pSpiHwRegs->HwRegs.SpiRegs.DmaControl = NV_RESETVAL(SPI, DMA_CTL);
}

static void SpiHwControllerInitialize(SerialHwRegisters *pSpiHwRegs)
{
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, COMMAND, 
                pSpiHwRegs->HwRegs.SpiRegs.Command);
}

/**
 * Set the functional mode whether this is the master or slave mode.
 */
static void
SpiHwSetFunctionalMode(
    SerialHwRegisters *pSpiHwRegs,
    NvBool IsMasterMode)
{
    // Slave mode is not supported.
    if (!IsMasterMode)
            NV_ASSERT(!"Not Supported");
}

    
/**
 * Initialize the spi register.
 */
static void
SpiHwResetFifo(
    SerialHwRegisters *pSpiHwRegs, 
    SerialHwFifo FifoType)
{
    NvU32 ResetBits = 0;

    NvU32 StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    if (FifoType & SerialHwFifo_Rx)
        ResetBits = NV_DRF_NUM(SPI, STATUS, RXF_FLUSH, 1);
    if (FifoType & SerialHwFifo_Tx)
        ResetBits |= NV_DRF_NUM(SPI, STATUS, RXF_FLUSH, 1);

    StatusReg |= ResetBits;
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, STATUS, StatusReg);

    // Now wait till the flush bits become 0
    StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    while (StatusReg & ResetBits)
    {
        StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    }
}

/**
 * Findout whether transmit fio is full or not
 */
static NvBool SpiHwIsTransmitFifoFull(SerialHwRegisters *pSpiHwRegs)
{
    NvU32 StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    if (StatusReg & NV_DRF_DEF(SPI, STATUS, TXF_FULL, FULL))
        return NV_TRUE;
    return NV_FALSE;    
}


/**
 * Set the signal mode of communication whether this is the mode  0, 1, 2 or 3.
 */
static void
SpiHwSetSignalMode(
    SerialHwRegisters *pSpiHwRegs, 
    NvOdmQuerySpiSignalMode SignalMode)
{
    NvU32 CommandReg;
    CommandReg = pSpiHwRegs->HwRegs.SpiRegs.Command;
    switch (SignalMode)
    {
        case NvOdmQuerySpiSignalMode_0:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SCLK,
                                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CK_SDA, 0,
                                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_1:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SCLK,
                                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CK_SDA, 1,
                                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_2:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SCLK,
                                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CK_SDA, 0,
                                CommandReg);
            break;
        case NvOdmQuerySpiSignalMode_3:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  ACTIVE_SCLK,
                                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CK_SDA, 1,
                                CommandReg);
            break;
        default:
            NV_ASSERT(!"Invalid SignalMode");
    }
    pSpiHwRegs->HwRegs.SpiRegs.Command = CommandReg;
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, COMMAND, CommandReg);
    pSpiHwRegs->CurrSignalMode = SignalMode;
}

/**
 * Set the transfer order whether the bit will start from the lsb or from
 * msb.
 */
static void
SpiHwSetTransferBitOrder(
    SerialHwRegisters *pSpiHwRegs, 
    NvBool IsLsbFirst)
{
    // This feature is not supported on the spi controller.
    if (IsLsbFirst)
        NV_ASSERT(!"Not Supported");
}

/**
 * Start the transfer of the communication.
 */
static void SpiHwStartTransfer(SerialHwRegisters *pSpiHwRegs, NvBool IsReconfigure)
{
    NvU32 DmaControlReg = pSpiHwRegs->HwRegs.SpiRegs.DmaControl;

    // Enable the dma bit in the register variable only
    DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE, DmaControlReg);

    // Now write the command and dma control values into the controller register
    
    // Need to write on the command register only if the reconfiguration is done.
    // Other wise it is not required.
    if (IsReconfigure)
        SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, COMMAND, 
                    pSpiHwRegs->HwRegs.SpiRegs.Command);

    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
}

/**
 * Enable/disable the data transfer flow.
 */
static void 
SpiHwSetDataFlow(
SerialHwRegisters *pSerialHwRegs, 
    SerialHwDataFlow DataFlow, 
    NvBool IsEnable)
{
    NvU32 CommandReg = pSerialHwRegs->HwRegs.SpiRegs.Command;
    if (DataFlow & SerialHwDataFlow_Rx)
    {
        if (IsEnable)
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  RXEN, 
                            ENABLE, CommandReg);
        else
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  RXEN, 
                            DISABLE, CommandReg);
    }

    if (DataFlow & SerialHwDataFlow_Tx)
    {
        if (IsEnable)
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  TXEN, 
                            ENABLE, CommandReg);
        else
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  TXEN, 
                            DISABLE, CommandReg);
    }
    pSerialHwRegs->HwRegs.SpiRegs.Command = CommandReg; 
    SPI_REG_WRITE32(pSerialHwRegs->pRegsBaseAdd, COMMAND, 
                            pSerialHwRegs->HwRegs.SpiRegs.Command);
}

/**
 * Set the chip select signal level to be default based on device during the
 * initialization.
 */
static void
SpiHwSetChipSelectDefaultLevelFxn(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh)
{
    // No control over the individual cs lines.
}

/**
 * Set the chip select signal level.
 */
static void
SpiHwSetChipSelectLevel(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh)
{
    NvU32 CommandReg = pSpiHwRegs->HwRegs.SpiRegs.Command;

    // Clear all chipselect
    CommandReg &= ~(RESET_ALL_CS);

    // Set the chip select level.
    if (IsHigh)
        CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_VAL, 0, CommandReg);
    else
        CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_VAL, 1, CommandReg);

    switch (ChipSelectId)
    {
        case 0:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  CS0_EN, ENABLE,
                CommandReg);
            break;
        case 1:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  CS1_EN, ENABLE,
                CommandReg);
            break;
        case 2:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  CS2_EN, ENABLE,
                CommandReg);
            break;
        case 3:
            CommandReg = NV_FLD_SET_DRF_DEF(SPI, COMMAND,  CS3_EN, ENABLE,
                CommandReg);
            break;
        default:
            NV_ASSERT(!"Invalid ChipSelectId");
    }     
    pSpiHwRegs->HwRegs.SpiRegs.Command = CommandReg;
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, COMMAND, CommandReg);
}

/**
 * Set the chip select signal level based on the transfer size.
 * it can use the hw based CS or SW based CS based on transfer size and
 * cpu/apb dma based transfer.
 * Return NV_TRUE if the SW based chipselection is used otherwise return
 * NV_FALSE;
 */
static NvBool
SpiHwSetChipSelectLevelBasedOnPacket(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh,
    NvU32 PacketRequested,
    NvU32 PacketPerWord,
    NvBool IsApbDmaBasedTransfer,
    NvBool IsOnlyUseSWCS)
{
    SpiHwSetChipSelectLevel(pSpiHwRegs, ChipSelectId, IsHigh);
    return NV_TRUE;
}

static void
SpiHwSetCsSetupHoldTime(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 CsSetupTimeInClocks,
    NvU32 CsHoldTimeInClocks)
{
    NV_ASSERT(0);
}

static void
SpiHwSetSlaveCsId(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 CsId,
    NvBool IsHigh)
{
    NV_ASSERT(0);
}

/**
 * Set the packet length and packed mode.
 */
static void
SpiHwSetPacketLength(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 PacketLength, 
    NvBool IsPackedMode)
{
    NvU32 CommandReg = pSpiHwRegs->HwRegs.SpiRegs.Command;
    NvU32 DmaControlReg = pSpiHwRegs->HwRegs.SpiRegs.DmaControl;

    CommandReg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  BIT_LENGTH, 
                    (PacketLength -1), CommandReg);
    if (IsPackedMode)
        DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL,  PACKED, ENABLE,
            DmaControlReg);
    else
        DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL,  PACKED, DISABLE,
            DmaControlReg);

    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, COMMAND, CommandReg);
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
            
    pSpiHwRegs->HwRegs.SpiRegs.Command = CommandReg;
    pSpiHwRegs->HwRegs.SpiRegs.DmaControl = DmaControlReg;
    pSpiHwRegs->PacketLength = PacketLength;
    pSpiHwRegs->IsPackedMode = IsPackedMode;
}

/**
 * Set the Dma transfer size.
 */
static void
SpiHwSetDmaTransferSize(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 DmaBlockSize)
{
    pSpiHwRegs->HwRegs.SpiRegs.DmaControl = 
            NV_FLD_SET_DRF_NUM(SPI, DMA_CTL, DMA_BLOCK_SIZE, (DmaBlockSize-1), 
                    pSpiHwRegs->HwRegs.SpiRegs.DmaControl);
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, DMA_CTL, pSpiHwRegs->HwRegs.SpiRegs.DmaControl);
}

static NvU32 SpiHwGetTransferdCount(SerialHwRegisters *pSpiHwRegs)
{
    NvU32 DmaBlockSize;
    NvU32 DmaControlReg = pSpiHwRegs->HwRegs.SpiRegs.DmaControl;
    DmaBlockSize = NV_DRF_VAL(SPI, DMA_CTL, DMA_BLOCK_SIZE, DmaControlReg); 
    return (DmaBlockSize +1);
}

/**
 * Set the trigger level.
 */
static void 
SpiHwSetTriggerLevel(
    SerialHwRegisters *pSpiHwRegs, 
    SerialHwFifo FifoType,
    NvU32 TriggerLevel)
{
    NvU32 DmaControlReg = pSpiHwRegs->HwRegs.SpiRegs.DmaControl;
    switch(TriggerLevel)
    {
        case 4:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG1, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG1, 
                                                DmaControlReg);
            break;

        case 16:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG4, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG4, 
                                                DmaControlReg);
            break;
        default:
            NV_ASSERT(!"Invalid Triggerlevel");
    }
    pSpiHwRegs->HwRegs.SpiRegs.DmaControl = DmaControlReg;
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
}

/**
 * Write into the transmit fifo register.
 * returns the number of words written.
 */
static NvU32
SpiHwWriteInTransmitFifo(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 *pTxBuff,  
    NvU32 WordRequested)
{
    NvU32 WordWritten = 0;
    NvU32 WordsRemaining = NV_MIN(WordRequested, MAX_SPI_FIFO_DEPTH);
    while (WordsRemaining)
    {
        SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, TX_FIFO, *pTxBuff);
        pTxBuff++;
        WordsRemaining--;
        WordWritten++;
    }
    return WordWritten;
}

/**
 * Read the data from the receive fifo.
 * Returns the number of words it read.   
 */
static NvU32
SpiHwReadFromReceiveFifo(
    SerialHwRegisters *pSpiHwRegs, 
    NvU32 *pRxBuff,  
    NvU32 WordRequested)
{
    NvU32 WordsRemaining = WordRequested;
    while (WordsRemaining)
    {
        *pRxBuff = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, RX_FIFO);
        pRxBuff++;
        WordsRemaining--;
    }
    return WordRequested;
}

/**
 * Enable/disable the interrupt source. 
 */
static void
SpiHwSetInterruptSource(
    SerialHwRegisters *pSpiHwRegs, 
    SerialHwDataFlow DataDirection,
    NvBool IsEnable)
{
    NvU32 DmaControlReg = pSpiHwRegs->HwRegs.SpiRegs.DmaControl;
    if (DataDirection & SerialHwDataFlow_Rx)
    {
        if (IsEnable)
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, IE_RXC,
                ENABLE, DmaControlReg);
        }
        else
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, IE_RXC,
                DISABLE, DmaControlReg);
        }
    }
    
    if (DataDirection & SerialHwDataFlow_Tx)
    {
        if (IsEnable)
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, IE_TXC,
                ENABLE, DmaControlReg);
        }
        else
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, IE_TXC,
                DISABLE, DmaControlReg);
        }
    }

    pSpiHwRegs->HwRegs.SpiRegs.DmaControl = DmaControlReg;
}

/**
 * Get the transfer status. 
 */
static NvError SpiHwGetTransferStatus(SerialHwRegisters *pSpiHwRegs,
            SerialHwDataFlow DataFlow)
{
    NvU32 StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    
    pSpiHwRegs->HwRegs.SlinkRegs.Status = StatusReg;
    // Check for the receive error 
    if (DataFlow & SerialHwDataFlow_Rx)
    {
        if (StatusReg & NV_DRF_NUM(SPI, STATUS, RXF_UNR, 1))
             return NvError_SpiReceiveError;
    }

    // Check for the transmit error 
    if (DataFlow & SerialHwDataFlow_Tx)
    {
        if (StatusReg & NV_DRF_NUM(SPI, STATUS, TXF_OVF, 1))
            return NvError_SpiTransmitError;
    }
    return NvSuccess;
}

static void SpiHwClearTransferStatus(SerialHwRegisters *pSpiHwRegs,
            SerialHwDataFlow DataFlow)
{
    NvU32 StatusReg = pSpiHwRegs->HwRegs.SpiRegs.Status ;

    // Clear all the write 1 on clear status.
    StatusReg &= (~ALL_SPI_STATUS_CLEAR);

    // Make ready clear to 1.
    StatusReg = NV_FLD_SET_DRF_NUM(SPI, STATUS, RDY, 1, StatusReg);
    
    // Check for the receive error 
    if (DataFlow & SerialHwDataFlow_Rx)
         StatusReg |= NV_DRF_NUM(SPI, STATUS, RXF_UNR, 1);

    // Check for the transmit error 
    if (DataFlow & SerialHwDataFlow_Tx)
         StatusReg |= NV_DRF_NUM(SPI, STATUS, TXF_OVF, 1);

    // Write on slink status register.     
    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, STATUS, StatusReg);
}

/**
 * Check whether transfer is completed or not. 
 */
static NvBool SpiHwIsTransferCompleted(SerialHwRegisters *pSpiHwRegs)
{
    // Read the Status register 
    NvU32 StatusReg = SPI_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);

    if (StatusReg & NV_DRF_DEF(SPI, STATUS, BSY, BUSY))
        return NV_FALSE;

    // Transfer is completed so clear the ready bit by write 1 to clear
    // Clear all the write 1 on clear status.
    StatusReg &= (~ALL_SPI_STATUS_CLEAR);

    // Make ready clear to 1.
    StatusReg = NV_FLD_SET_DRF_NUM(SPI, STATUS, RDY, 1, StatusReg);    

    SPI_REG_WRITE32(pSpiHwRegs->pRegsBaseAdd, STATUS, StatusReg);

    return NV_TRUE;
}

static NvBool
SpiHwClearFifosForNewTransfer(
    SerialHwRegisters *pSpiHwRegs,
    SerialHwDataFlow DataDirection)
{
    return NV_FALSE;
}

/**
 * Initialize the spi intterface for the hw access.
 */
void NvRmPrivSpiSlinkInitSpiInterface(HwInterface *pSpiInterface)
{
    pSpiInterface->HwRegisterInitializeFxn = SpiHwRegisterInitialize;
    pSpiInterface->HwControllerInitializeFxn = SpiHwControllerInitialize; 
    pSpiInterface->HwSetFunctionalModeFxn = SpiHwSetFunctionalMode;
    pSpiInterface->HwResetFifoFxn = SpiHwResetFifo;
    pSpiInterface->HwIsTransmitFifoFull = SpiHwIsTransmitFifoFull;
    pSpiInterface->HwSetSignalModeFxn = SpiHwSetSignalMode;
    pSpiInterface->HwSetTransferBitOrderFxn = SpiHwSetTransferBitOrder;
    pSpiInterface->HwStartTransferFxn = SpiHwStartTransfer;
    pSpiInterface->HwSetDataFlowFxn = SpiHwSetDataFlow;
    pSpiInterface->HwSetChipSelectDefaultLevelFxn = SpiHwSetChipSelectDefaultLevelFxn;
    pSpiInterface->HwSetChipSelectLevelFxn = SpiHwSetChipSelectLevel;
    pSpiInterface->HwSetChipSelectLevelBasedOnPacketFxn = SpiHwSetChipSelectLevelBasedOnPacket;
    pSpiInterface->HwSetCsSetupHoldTime    = SpiHwSetCsSetupHoldTime;
    pSpiInterface->HwSetSlaveCsIdFxn    = SpiHwSetSlaveCsId;
    pSpiInterface->HwSetPacketLengthFxn = SpiHwSetPacketLength;
    pSpiInterface->HwSetDmaTransferSizeFxn = SpiHwSetDmaTransferSize;
    pSpiInterface->HwGetTransferdCountFxn = SpiHwGetTransferdCount;
    pSpiInterface->HwSetTriggerLevelFxn = SpiHwSetTriggerLevel;
    pSpiInterface->HwWriteInTransmitFifoFxn = SpiHwWriteInTransmitFifo;
    pSpiInterface->HwReadFromReceiveFifoFxn =  SpiHwReadFromReceiveFifo;
    pSpiInterface->HwSetInterruptSourceFxn = SpiHwSetInterruptSource;
    pSpiInterface->HwClearTransferStatusFxn = SpiHwClearTransferStatus;
    pSpiInterface->HwGetTransferStatusFxn = SpiHwGetTransferStatus;
    pSpiInterface->HwIsTransferCompletedFxn = SpiHwIsTransferCompleted;
    pSpiInterface->HwClearFifosForNewTransferFxn = SpiHwClearFifosForNewTransfer;
}
