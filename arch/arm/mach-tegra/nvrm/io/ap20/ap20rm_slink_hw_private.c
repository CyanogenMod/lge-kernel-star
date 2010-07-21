/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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
#include "ap20/arslink.h"
#include "../ap15/rm_spi_slink_hw_private.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvos.h"

// Enable the hw based chipselect
#define ENABLE_HW_BASED_CS 1
#define SLINK_REG_READ32(pSlinkHwRegsVirtBaseAdd, reg) \
        NV_READ32((pSlinkHwRegsVirtBaseAdd) + ((SLINK_##reg##_0)/4))
#define SLINK_REG_WRITE32(pSlinkHwRegsVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32((((pSlinkHwRegsVirtBaseAdd) + ((SLINK_##reg##_0)/4))), (val)); \
    } while(0)


#define MAX_SLINK_FIFO_DEPTH 32
#define MAX_SLINK_WORD_FOR_HW_CS 128
#define MAX_SLINK_PACKET_FOR_HW_CS 64*1024

#define ALL_SLINK_STATUS_CLEAR \
        (NV_DRF_NUM(SLINK, STATUS,  RDY, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  RX_UNF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  TX_UNF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  TX_OVF, 1) | \
        NV_DRF_NUM(SLINK, STATUS,  RX_OVF, 1))

static void
SlinkHwSetSignalMode(
    SerialHwRegisters *pSlinkHwRegs, 
    NvOdmQuerySpiSignalMode SignalMode);

/**
 * Initialize the slink register.
 */
static void
SlinkHwRegisterInitialize(
    NvU32 SlinkInstanceId, 
    SerialHwRegisters *pSlinkHwRegs)
{
    NvU32 CommandReg1;
    NvU32 CommandReg2;
    pSlinkHwRegs->InstanceId = SlinkInstanceId;
    pSlinkHwRegs->pRegsBaseAdd = NULL;
    pSlinkHwRegs->RegBankSize = 0;
    pSlinkHwRegs->HwTxFifoAdd = SLINK_TX_FIFO_0;
    pSlinkHwRegs->HwRxFifoAdd = SLINK_RX_FIFO_0;
    pSlinkHwRegs->IsPackedMode = NV_FALSE;
    pSlinkHwRegs->PacketLength = 1;
    pSlinkHwRegs->CurrSignalMode = NvOdmQuerySpiSignalMode_Invalid;
    pSlinkHwRegs->MaxWordTransfer = MAX_SLINK_FIFO_DEPTH;
    pSlinkHwRegs->IsLsbFirst = NV_FALSE;
    pSlinkHwRegs->IsMasterMode = NV_TRUE;
    pSlinkHwRegs->IsNonWordAlignedPackModeSupported = NV_TRUE;
    pSlinkHwRegs->IsHwChipSelectSupported = NV_TRUE;

    CommandReg1 = NV_RESETVAL(SLINK, COMMAND);
    CommandReg2 = NV_RESETVAL(SLINK, COMMAND2);
    
    // Do not toggle the CS between each packet.
    CommandReg2 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND2, CS_ACTIVE_BETWEEN, HIGH,
                    CommandReg2);

    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, M_S, MASTER, CommandReg1);

    if (pSlinkHwRegs->IsIdleDataOutHigh)
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SDA, DRIVE_HIGH, CommandReg1);
    else
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SDA, DRIVE_LOW, CommandReg1);
    
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;
    pSlinkHwRegs->HwRegs.SlinkRegs.Status = NV_RESETVAL(SLINK, STATUS);
    pSlinkHwRegs->HwRegs.SlinkRegs.DmaControl = NV_RESETVAL(SLINK, DMA_CTL);
}

/**
 * Set the signal mode of communication whether this is the mode  0, 1, 2 or 3.
 */
static void
SlinkHwSetSignalMode(
    SerialHwRegisters *pSlinkHwRegs, 
    NvOdmQuerySpiSignalMode SignalMode)
{
    NvU32 CommandReg = pSlinkHwRegs->HwRegs.SlinkRegs.Command1;
    switch (SignalMode)
    {
        case NvOdmQuerySpiSignalMode_0:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, FIRST_CLK_EDGE,
                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_1:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, SECOND_CLK_EDGE,
                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_2:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, FIRST_CLK_EDGE,
                CommandReg);
            break;
        case NvOdmQuerySpiSignalMode_3:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, SECOND_CLK_EDGE,
                CommandReg);
            break;
        default:
            NV_ASSERT(!"Invalid SignalMode");

    }
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, CommandReg);
    pSlinkHwRegs->CurrSignalMode = SignalMode;
}

/**
 * Set the chip select polarity bit in the command register.
 */
static NvU32
SetPolarityBits(
    NvU32 ChipSelectId,
    NvBool IsHigh,
    NvU32 Command1)
{
    NvU32 CSPolVal = (IsHigh)?0:1;
    NvU32 CommandReg1 = Command1;
    switch (ChipSelectId)
    {
        case 0:
            CommandReg1 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND, CS_POLARITY0, 
                                                       CSPolVal, CommandReg1);
            break;
        
        case 1:
            CommandReg1 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND, CS_POLARITY1, 
                                                       CSPolVal, CommandReg1);
            break;
        
        case 2:
            CommandReg1 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND, CS_POLARITY2, 
                                                       CSPolVal, CommandReg1);
            break;
        
        case 3:
            CommandReg1 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND, CS_POLARITY3, 
                                                       CSPolVal, CommandReg1);
            break;
        
        default:
            NV_ASSERT(!"Invalid ChipSelectId");
    }     
    return CommandReg1;
}

/**
 * Set the chip select numbers in the command register.
 */
static NvU32
SetCSNumber(
    NvU32 ChipSelectId,
    NvU32 Command2)
{
    NvU32 CommandReg2 = Command2;
    switch (ChipSelectId)
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
    return CommandReg2;
}

/**
 * Set the chip select signal level to be default based on device during the
 * initialization.
 */
static void
SlinkHwSetChipSelectDefaultLevel(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh)
{
    NvU32 CommandReg1;
    CommandReg1 = SetPolarityBits(ChipSelectId, IsHigh, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, CommandReg1);
}

/**
 * Set the chip select signal level.
 */
static void
SlinkHwSetChipSelectLevel(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh)
{
    NvU32 CommandReg1;
    NvU32 CommandReg2;

    // Select SW based CS
    CommandReg1 = SetPolarityBits(ChipSelectId, IsHigh, 
                                pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CS_SW, SOFT, CommandReg1);

    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, CommandReg1);


    CommandReg2 = SetCSNumber(ChipSelectId, pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
}
/**
 * Set the chip select signal level based on the transfer size.
 * it can use the hw based CS or SW based CS based on transfer size and
 * cpu/apb dma based transfer.
 * Return NV_TRUE if the SW based chipselection is used otherwise return
 * NV_FALSE;
 */
static NvBool
SlinkHwSetChipSelectLevelBasedOnPacket(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 ChipSelectId,
    NvBool IsHigh,
    NvU32 PacketRequested,
    NvU32 PacketPerWord,
    NvBool IsApbDmaBasedTransfer,
    NvBool IsOnlyUseSWCS)
{
    NvU32 MaxWordReq;
    NvU32 CommandReg1;
    NvU32 CommandReg2;
    NvU32 RefillCount = 0;
#if ENABLE_HW_BASED_CS 
    NvBool UseSWBaseCS = IsOnlyUseSWCS;
#else    
    NvBool UseSWBaseCS = NV_TRUE;
#endif    
    
    if (!UseSWBaseCS)
    {
        if (IsApbDmaBasedTransfer)
        {
            UseSWBaseCS = (PacketRequested <= MAX_SLINK_PACKET_FOR_HW_CS)? 
                                NV_FALSE: NV_TRUE;
        }    
        else
        {
            MaxWordReq = (PacketRequested + PacketPerWord -1)/PacketPerWord;
            NV_ASSERT(MaxWordReq);
            if (MaxWordReq <= MAX_SLINK_WORD_FOR_HW_CS)
            {
                RefillCount = (MaxWordReq)/MAX_SLINK_FIFO_DEPTH;
                UseSWBaseCS = NV_FALSE;    
            }
            else
            {
                UseSWBaseCS = NV_TRUE;
            }
        }
    }
    if (UseSWBaseCS)
    {
        SlinkHwSetChipSelectLevel(pSlinkHwRegs, ChipSelectId, IsHigh);
        return NV_TRUE;
    }
    
    // Select HW based chipselect.
    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CS_SW, HARD, 
                                    pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
    
    CommandReg2 = SetCSNumber(ChipSelectId, 
                                    pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
    if (!IsApbDmaBasedTransfer)
        CommandReg2 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND2, FIFO_REFILLS, 
                                        RefillCount, CommandReg2);
                                        
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
    
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2, 
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
                            
    return NV_FALSE;
}

static void
SlinkHwSetCsSetupHoldTime(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 CsSetupTimeInClocks,
    NvU32 CsHoldTimeInClocks)
{
    NvU32 CommandReg2 = pSlinkHwRegs->HwRegs.SlinkRegs.Command2;
    NvU32 SetupTime;

    SetupTime = (CsSetupTimeInClocks +1)/2;
    SetupTime = (SetupTime > 3)?3: SetupTime;
    CommandReg2 = NV_FLD_SET_DRF_NUM(SLINK, COMMAND2, SS_SETUP,
                                        SetupTime, CommandReg2);
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2,
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
}

/**
 * Write into the transmit fifo register.
 * returns the number of words written.
 */
static NvU32
SlinkHwWriteInTransmitFifo(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 *pTxBuff,  
    NvU32 WordRequested)
{
    NvU32 WordWritten = 0;
    NvU32 WordsRemaining;
    NvU32 SlinkFifoEmptyCountReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS2);
    SlinkFifoEmptyCountReg = NV_DRF_VAL(SLINK, STATUS2, TX_FIFO_EMPTY_COUNT, SlinkFifoEmptyCountReg);
    WordsRemaining = NV_MIN(WordRequested, SlinkFifoEmptyCountReg);
    WordWritten = WordsRemaining;
    while (WordsRemaining)
    {
        SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, TX_FIFO, *pTxBuff);
        pTxBuff++;
        WordsRemaining--;
    }
    return WordWritten;
}

/**
 * Read the data from the receive fifo.
 * Returns the number of words it read.   
 */
static NvU32
SlinkHwReadFromReceiveFifo(
    SerialHwRegisters *pSlinkHwRegs, 
    NvU32 *pRxBuff,  
    NvU32 WordRequested)
{
    NvU32 WordsRemaining;
    NvU32 SlinkFifoFullCountReg =  SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS2);
    NvU32 WordsRead;
   
    SlinkFifoFullCountReg = NV_DRF_VAL(SLINK, STATUS2, RX_FIFO_FULL_COUNT, SlinkFifoFullCountReg);
    WordsRemaining = NV_MIN(WordRequested, SlinkFifoFullCountReg);
    WordsRead = WordsRemaining;
    while (WordsRemaining)
    {
        *pRxBuff = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, RX_FIFO);
        pRxBuff++;
        WordsRemaining--;
    }
    return WordsRead;
}

static NvBool
SlinkHwClearFifosForNewTransfer(
    SerialHwRegisters *pSlinkHwRegs,
    SerialHwDataFlow DataDirection)
{
    NvU32 ResetBits = 0;
    NvU32 StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);

    if (!(StatusReg & NV_DRF_DEF(SLINK, STATUS, TX_EMPTY, EMPTY)))
        ResetBits |= NV_DRF_NUM(SLINK, STATUS, TX_FLUSH, 1);

    if (!(StatusReg & NV_DRF_DEF(SLINK, STATUS, RX_EMPTY, EMPTY)))
        ResetBits |= NV_DRF_NUM(SLINK, STATUS, RX_FLUSH, 1);

    if (!ResetBits)
        return NV_FALSE;

    StatusReg |= ResetBits;
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, STATUS, StatusReg);

    // Now wait till the flush bits become 0
    do
    {
        StatusReg = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, STATUS);
    } while (StatusReg & ResetBits);
    return NV_TRUE;
}

/**
 * Initialize the slink intterface for the hw access.
 */
void NvRmPrivSpiSlinkInitSlinkInterface_v1_1(HwInterface *pSlinkInterface)
{
    pSlinkInterface->HwRegisterInitializeFxn = SlinkHwRegisterInitialize;
    pSlinkInterface->HwSetSignalModeFxn = SlinkHwSetSignalMode;
    pSlinkInterface->HwSetChipSelectDefaultLevelFxn = SlinkHwSetChipSelectDefaultLevel;
    pSlinkInterface->HwSetChipSelectLevelFxn = SlinkHwSetChipSelectLevel;
    pSlinkInterface->HwSetChipSelectLevelBasedOnPacketFxn = SlinkHwSetChipSelectLevelBasedOnPacket;
    pSlinkInterface->HwSetCsSetupHoldTime    = SlinkHwSetCsSetupHoldTime;
    pSlinkInterface->HwWriteInTransmitFifoFxn = SlinkHwWriteInTransmitFifo;
    pSlinkInterface->HwReadFromReceiveFifoFxn =  SlinkHwReadFromReceiveFifo;
    pSlinkInterface->HwClearFifosForNewTransferFxn =  SlinkHwClearFifosForNewTransfer;
}
