/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
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
    pSlinkHwRegs->IsNonWordAlignedPackModeSupported = NV_FALSE;
    pSlinkHwRegs->IsHwChipSelectSupported = NV_FALSE;

    CommandReg1 = NV_RESETVAL(SLINK, COMMAND);

    // Initialize the chip select bits to select the s/w only
    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CS_SW, SOFT, CommandReg1);

    // Set chip select to normal high level. (inverted polarity).
    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND,  CS_VALUE, HIGH, CommandReg1);

    CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, M_S, MASTER, CommandReg1);

    if (pSlinkHwRegs->IsIdleDataOutHigh)
    {
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SDA, DRIVE_HIGH, CommandReg1);
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SDA, DRIVE_HIGH, CommandReg1);
    }
    else
    {
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SDA, DRIVE_LOW, CommandReg1);
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SDA, DRIVE_LOW, CommandReg1);
    }
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = NV_RESETVAL(SLINK, COMMAND2);
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
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, FIRST_CLK_EDGE,
                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_1:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_LOW, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, SECOND_CLK_EDGE,
                CommandReg);
            break;

        case NvOdmQuerySpiSignalMode_2:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SCLK,
                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, IDLE_SCLK,
                DRIVE_HIGH, CommandReg);
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, CK_SDA, FIRST_CLK_EDGE,
                CommandReg);
            break;
        case NvOdmQuerySpiSignalMode_3:
            CommandReg = NV_FLD_SET_DRF_DEF(SLINK, COMMAND, ACTIVE_SCLK,
                DRIVE_HIGH, CommandReg);
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
 * Set the chip select signal level to be default based on device during the
 * initialization.
 */
static void
SlinkHwSetChipSelectDefaultLevelFxn(
    SerialHwRegisters *pHwRegs,
    NvU32 ChipSelectId,
    NvBool IsHigh)
{
    // No control over the individual cs lines.
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
    NvU32 CommandReg1 = pSlinkHwRegs->HwRegs.SlinkRegs.Command1;
    NvU32 CommandReg2 = pSlinkHwRegs->HwRegs.SlinkRegs.Command2;

    // Set the chip select level.
    if (IsHigh)
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND,  CS_VALUE, LOW, CommandReg1);
    else
        CommandReg1 = NV_FLD_SET_DRF_DEF(SLINK, COMMAND,  CS_VALUE, HIGH, CommandReg1);

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
    pSlinkHwRegs->HwRegs.SlinkRegs.Command1 = CommandReg1;
    pSlinkHwRegs->HwRegs.SlinkRegs.Command2 = CommandReg2;

    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND2,
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command2);
    SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, COMMAND,
                            pSlinkHwRegs->HwRegs.SlinkRegs.Command1);
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
    SlinkHwSetChipSelectLevel(pSlinkHwRegs, ChipSelectId, IsHigh);
    return NV_TRUE;
}

static void
SlinkHwSetCsSetupHoldTime(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 CsSetupTimeInClocks,
    NvU32 CsHoldTimeInClocks)
{
    NV_ASSERT(0);
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
    NvU32 WordsRemaining = NV_MIN(WordRequested, MAX_SLINK_FIFO_DEPTH);

    while (WordsRemaining)
    {
        SLINK_REG_WRITE32(pSlinkHwRegs->pRegsBaseAdd, TX_FIFO, *pTxBuff);
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
SlinkHwReadFromReceiveFifo(
    SerialHwRegisters *pSlinkHwRegs,
    NvU32 *pRxBuff,
    NvU32 WordRequested)
{
    NvU32 WordsRemaining = WordRequested;
    while (WordsRemaining)
    {
        *pRxBuff = SLINK_REG_READ32(pSlinkHwRegs->pRegsBaseAdd, RX_FIFO);
        pRxBuff++;
        WordsRemaining--;
    }
    return WordRequested;
}

static NvBool
SlinkHwClearFifosForNewTransfer(
    SerialHwRegisters *pSlinkHwRegs,
    SerialHwDataFlow DataDirection)
{
    return NV_FALSE;
}

/**
 * Initialize the slink intterface for the hw access.
 */
void NvRmPrivSpiSlinkInitSlinkInterface_v1_0(HwInterface *pSlinkInterface)
{
    pSlinkInterface->HwRegisterInitializeFxn = SlinkHwRegisterInitialize;
    pSlinkInterface->HwSetSignalModeFxn = SlinkHwSetSignalMode;
    pSlinkInterface->HwSetChipSelectDefaultLevelFxn = SlinkHwSetChipSelectDefaultLevelFxn;
    pSlinkInterface->HwSetChipSelectLevelFxn = SlinkHwSetChipSelectLevel;
    pSlinkInterface->HwSetChipSelectLevelBasedOnPacketFxn = SlinkHwSetChipSelectLevelBasedOnPacket;
    pSlinkInterface->HwSetCsSetupHoldTime    = SlinkHwSetCsSetupHoldTime;
    pSlinkInterface->HwWriteInTransmitFifoFxn = SlinkHwWriteInTransmitFifo;
    pSlinkInterface->HwReadFromReceiveFifoFxn =  SlinkHwReadFromReceiveFifo;
    pSlinkInterface->HwClearFifosForNewTransferFxn =  SlinkHwClearFifosForNewTransfer;
}
