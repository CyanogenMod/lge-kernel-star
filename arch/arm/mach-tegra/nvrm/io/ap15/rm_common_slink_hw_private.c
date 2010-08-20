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
 * @brief <b>nVIDIA driver Development Kit:
 *           Private functions implementation for the slink Rm driver</b>
 *
 * @b Description:  Implements the private functions for the slink hw interface.
 *
 */

// hardware includes
#include "ap15/arslink.h"
#include "rm_spi_slink_hw_private.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvos.h"
#include "linux/kernel.h"

#define SLINK_REG_READ32(pSlinkHwRegsVirtBaseAdd, reg) \
        NV_READ32((pSlinkHwRegsVirtBaseAdd) + ((SLINK_##reg##_0)/4))
#define SLINK_REG_WRITE32(pSlinkHwRegsVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32((((pSlinkHwRegsVirtBaseAdd) + ((SLINK_##reg##_0)/4))), (val)); \
    } while(0)


#define MAX_SLINK_FIFO_DEPTH 32

#define ALL_SLINK_STATUS_CLEAR \
        (NV_DRF_NUM(SLINK, STATUS,  RDY, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  RX_UNF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  TX_UNF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  TX_OVF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  RX_OVF, 1))

#define RX_ERROR_STATUS (NV_DRF_NUM(SLINK, STATUS, RX_UNF, 1) | \
                            NV_DRF_NUM(SLINK, STATUS, RX_OVF, 1))
#define TX_ERROR_STATUS (NV_DRF_NUM(SLINK, STATUS, TX_OVF, 1) | \
                            NV_DRF_NUM(SLINK, STATUS, TX_UNF, 1))

static void SlinkHwControllerInitialize(SerialHwRegisters *pSlinkHwRegs)
{
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
}


/**
 * Set the functional mode whether this is the master or slave mode.
 */
static void
SlinkHwSetFunctionalMode(
    SerialHwRegisters *pSlinkHwRegs,
    NvBool IsMasterMode)
{
    NvU32 CommandReg = pSlinkHwRegs->HwRegs.SlinkRegs.Command1;
    if (IsMasterMode)
        CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, M_S, MASTER, CommandReg);
    else
        CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, M_S, SLAVE, CommandReg);

    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, CommandReg);
    pSlinkHwRegs->IsMasterMode = IsMasterMode;
}

/**
 * Initialize the slink register.
 */
static void
SlinkHwResetFifo(
    SerialHwRegisters *pSlinkHwRegs, 
    SerialHwFifo FifoType)
{
    NvU32 ResetBits = 0;
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    if (FifoType & SerialHwFifo_Rx)
        ResetBits = NV_DRF_NUM(SLINK, STATUS, RX_FLUSH, 1);
    if (FifoType & SerialHwFifo_Tx)
        ResetBits |= NV_DRF_NUM(SLINK, STATUS, TX_FLUSH, 1);

    StatusReg |= ResetBits;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, STATUS, StatusReg);

    // Now wait till the flush bits become 0
    do 
    {
        StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    } while (StatusReg & ResetBits);
}

/**
 * Findout whether transmit fio is full or not
 */
static NvBool SlinkHwIsTransmitFifoFull(SerialHwRegisters *pSpiHwRegs)
{
    NvU32 StatusReg = SLINK_REG_READ32(pSpiHwRegs->pRegsBaseAdd, STATUS);
    if (StatusReg & NV_DRF_DEF(SLINK, STATUS, TX_FULL, FULL))
        return NV_TRUE;
    return NV_FALSE;    
}

    
/**
 * Set the transfer order whether the bit will start from the lsb or from
 * msb.
 */
static void
SlinkHwSetTransferBitOrder(
    SerialHwRegisters *pSlinkHwRegs, 
    NvBool IsLsbFirst)
{
    NvU32 Command2Reg = pSlinkHwRegs->HwRegs.SlinkRegs.Command2;
    if (IsLsbFirst)
        Command2Reg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, LSBFE, LAST, Command2Reg);
    else
        Command2Reg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, LSBFE, FIRST, Command2Reg);

    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = Command2Reg;
}

/**
 * Start the transfer of the communication.
 */
static void SlinkHwStartTransfer(SerialHwRegisters *pSlinkHwRegs, NvBool IsReconfigure)
{
    NvU32 DmaControlReg = pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl;

    // Program the packed mode
    if (pSlinkHwRegs->IsPackedMode)
    {
        DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL,  PACKED, ENABLE,
            DmaControlReg);
        SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);

        // Hw bug: Need to give some delay after setting the packed mode.
        NvOsWaitUS(1);
    }

    // Enable the dma bit in the register variable only
    DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, DMA_EN, ENABLE, DmaControlReg);

    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
}

/**
 * Enable/disable the data transfer flow.
 */
static void 
SlinkHwSetDataFlow(
    SerialHwRegisters *pSlinkHwRegs, 
    SerialHwDataFlow DataFlow, 
    NvBool IsEnable)
{
    NvU32 CommandReg2 = pSlinkHwRegs->HwRegs.SlinkRegs.Command2;
    if (DataFlow & SerialHwDataFlow_Rx)
    {
        if (IsEnable)
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2,  RXEN, 
                            ENABLE, CommandReg2);
        else
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2,  RXEN, 
                            DISABLE, CommandReg2);
    }

    if (DataFlow & SerialHwDataFlow_Tx)
    {
        if (IsEnable)
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2,  TXEN, 
                            ENABLE, CommandReg2);
        else
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2,  TXEN, 
                            DISABLE, CommandReg2);
    }
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2; 
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
}

/**
 * Set CS for slave communication.
 */
static void
SlinkHwSetSlaveCsId(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 CsId,
    NvBool IsHigh)
{
    NvU32 CommandReg1 = pSlinkHwRegs->HwRegs.SlinkRegs.Command1;
    NvU32 CommandReg2 = pSlinkHwRegs->HwRegs.SlinkRegs.Command2;

    // Set the chip select level.
    if (IsHigh)
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND,  CS_VALUE, LOW, CommandReg1);
    else
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND,  CS_VALUE, HIGH, CommandReg1);

    switch (CsId)
    {
        case 0:
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, SS_EN, CS0, CommandReg2);
            break;

        case 1:
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, SS_EN, CS1, CommandReg2);
            break;

        case 2:
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, SS_EN, CS2, CommandReg2);
            break;

        case 3:
            CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, SS_EN, CS3, CommandReg2);
            break;

        default:
            NV_ASSERT(!"Invalid ChipSelectId");
    }
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;

    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2,
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND,
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
}



/**
 * Set the packet length and packed mode.
 */
static void
SlinkHwSetPacketLength(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 PacketLength, 
    NvBool IsPackedMode)
{
    NvU32 CommandReg1 = pSlinkHwRegs->HwRegs.SlinkRegs.Command1;
    NvU32 DmaControlReg = pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl;

    CommandReg1 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND,  BIT_LENGTH, 
                                            (PacketLength -1), CommandReg1);
                                            
    // Unset the packed bit if it is there
    DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACKED, DISABLE, DmaControlReg);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);

    if (IsPackedMode)
    {
        if (PacketLength == 4)
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACK_SIZE, PACK4, 
                            DmaControlReg);
        else if (PacketLength == 8)
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACK_SIZE, PACK8, 
                            DmaControlReg);
        else if (PacketLength == 16)
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACK_SIZE, PACK16, 
                            DmaControlReg);
        else if (PacketLength == 32)
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACK_SIZE, PACK32, 
                            DmaControlReg);
    }
    else
    {
        DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, PACK_SIZE, PACK4, 
                        DmaControlReg);
    }

    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, CommandReg1);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
                    
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl = DmaControlReg;

    pSlinkHwRegs->PacketLength = PacketLength;
    pSlinkHwRegs->IsPackedMode = IsPackedMode;
}

/**
 * Set the Dma transfer size.
 */
static void
SlinkHwSetDmaTransferSize(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 DmaBlockSize)
{
    pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl = 
            NV_FLD_SET_DRF_NUM(SLINK, DMA_CTL, DMA_BLOCK_SIZE, (DmaBlockSize-1), 
                                pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, 
                                pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl);
}

static NvU32 SlinkHwGetTransferdCount(SerialHwRegisters *pSlinkHwRegs)
{
    NvU32 DmaBlockSize;
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    DmaBlockSize = NV_DRF_VAL(SLINK, STATUS, BLK_CNT, StatusReg); 
    return (DmaBlockSize);
}

/**
 * Set the trigger level.
 */
static void 
SlinkHwSetTriggerLevel(
    SerialHwRegisters *pSlinkHwRegs, 
    SerialHwFifo FifoType,
    NvU32 TriggerLevel)
{
    NvU32 DmaControlReg = pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl;
    switch(TriggerLevel)
    {
        case 4:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, RX_TRIG, TRIG1, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, TX_TRIG, TRIG1, 
                                                DmaControlReg);
            break;

        case 16:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, RX_TRIG, TRIG4, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, TX_TRIG, TRIG4, 
                                                DmaControlReg);
            break;


        case 32:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, RX_TRIG, TRIG8, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, TX_TRIG, TRIG8, 
                                                DmaControlReg);
            break;

        case 64:
            if (FifoType & SerialHwFifo_Rx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, RX_TRIG, TRIG16, 
                                                DmaControlReg);
            if (FifoType & SerialHwFifo_Tx)
                DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, TX_TRIG, TRIG16, 
                                                DmaControlReg);
            break;

        default:
            NV_ASSERT(!"Invalid Triggerlevel");
    }
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
    pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl = DmaControlReg;
}

/**
 * Enable/disable the interrupt source. 
 */
static void
SlinkHwSetInterruptSource(
    SerialHwRegisters *pSlinkHwRegs, 
    SerialHwDataFlow DataDirection,
    NvBool IsEnable)
{
#if !NV_OAL
    NvU32 DmaControlReg = pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl;
    if (DataDirection & SerialHwDataFlow_Rx)
    {
        if (IsEnable)
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, IE_RXC,
                ENABLE, DmaControlReg);
        }
        else
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, IE_RXC,
                DISABLE, DmaControlReg);
        }
    }
    
    if (DataDirection & SerialHwDataFlow_Tx)
    {
        if (IsEnable)
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, IE_TXC,
                ENABLE, DmaControlReg);
        }
        else
        {
            DmaControlReg = NV_FLD_SET_DRF_DEF(SLINK, DMA_CTL, IE_TXC,
                DISABLE, DmaControlReg);
        }
    }

    pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl = DmaControlReg;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, DMA_CTL, DmaControlReg);
#endif
}

/**
 * Get the transfer status. 
 */
static NvError SlinkHwGetTransferStatus(SerialHwRegisters *pSlinkHwRegs,
            SerialHwDataFlow DataFlow)
{
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    pSlinkHwRegs->HwRegs.SlinkRegs.Status = StatusReg;
    // Check for the receive error 
    if (DataFlow & SerialHwDataFlow_Rx)
    {
        if (StatusReg & RX_ERROR_STATUS) {
		pr_err("SPI RX ERROR Status 0x%x\n",StatusReg);
             return NvError_SpiReceiveError;
	}
    }

    // Check for the transmit error 
    if (DataFlow & SerialHwDataFlow_Tx)
    {
        if (StatusReg & TX_ERROR_STATUS) {
		pr_err("SPI TX ERROR Status 0x%x\n",StatusReg);
            return NvError_SpiTransmitError;
	}
    }
    return NvSuccess;
}

static void SlinkHwClearTransferStatus(SerialHwRegisters *pSlinkHwRegs,
            SerialHwDataFlow DataFlow)
{
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);

    // Clear all the write 1 on clear status.
    StatusReg &= (~ALL_SLINK_STATUS_CLEAR);

    // Make ready clear to 1.
    StatusReg = NV_FLD_SET_DRF_NUM(SLINK, STATUS, RDY, 1, StatusReg);
    
    // Check for the receive error 
    if (DataFlow & SerialHwDataFlow_Rx)
         StatusReg |= RX_ERROR_STATUS;

    // Check for the transmit error 
    if (DataFlow & SerialHwDataFlow_Tx)
         StatusReg |= TX_ERROR_STATUS;

    // Write on slink status register.     
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, STATUS, StatusReg);
}

/**
 *  Check whether transfer is completed or not. 
 */
static NvBool SlinkHwIsTransferCompleted( SerialHwRegisters *pSlinkHwRegs)
{
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    if (StatusReg & NV_DRF_DEF(SLINK, STATUS, BSY, BUSY))
        return NV_FALSE;

    return NV_TRUE;        
}

/**
 * Initialize the slink intterface for the hw access.
 */
void NvRmPrivSpiSlinkInitSlinkInterface(HwInterface *pSlinkInterface)
{
    pSlinkInterface->HwControllerInitializeFxn = SlinkHwControllerInitialize; 
    pSlinkInterface->HwSetFunctionalModeFxn = SlinkHwSetFunctionalMode;
    pSlinkInterface->HwResetFifoFxn = SlinkHwResetFifo;
    pSlinkInterface->HwIsTransmitFifoFull = SlinkHwIsTransmitFifoFull;
    pSlinkInterface->HwSetTransferBitOrderFxn = SlinkHwSetTransferBitOrder;
    pSlinkInterface->HwStartTransferFxn = SlinkHwStartTransfer;
    pSlinkInterface->HwSetSlaveCsIdFxn = SlinkHwSetSlaveCsId;
    pSlinkInterface->HwSetDataFlowFxn = SlinkHwSetDataFlow;
    pSlinkInterface->HwSetPacketLengthFxn = SlinkHwSetPacketLength;
    pSlinkInterface->HwSetDmaTransferSizeFxn = SlinkHwSetDmaTransferSize;
    pSlinkInterface->HwGetTransferdCountFxn = SlinkHwGetTransferdCount;
    pSlinkInterface->HwSetTriggerLevelFxn = SlinkHwSetTriggerLevel;
    pSlinkInterface->HwSetInterruptSourceFxn = SlinkHwSetInterruptSource;
    pSlinkInterface->HwGetTransferStatusFxn = SlinkHwGetTransferStatus;
    pSlinkInterface->HwClearTransferStatusFxn = SlinkHwClearTransferStatus;
    pSlinkInterface->HwIsTransferCompletedFxn = SlinkHwIsTransferCompleted;
}

