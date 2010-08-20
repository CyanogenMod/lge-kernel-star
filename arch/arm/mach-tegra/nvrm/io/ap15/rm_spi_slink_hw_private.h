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
 *           Private functions for the spi Rm driver</b>
 *
 * @b Description:  Defines the private interfacing functions for the spi 
 * hw interface.
 * 
 */

#ifndef INCLUDED_RMSPI_HW_PRIVATE_H
#define INCLUDED_RMSPI_HW_PRIVATE_H

/**
 * @defgroup nvrm_spi Synchrnous Peripheral Interface(SPI) Controller hw 
 * interface API
 * 
 * This is the synchrnous peripheral interface (SPI) hw interface controller api
 * which communicate to the device/other processor using the spi protocols.
 * 
 * @ingroup nvddk_modules
 * @{
 *
 */

#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvodm_query.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Combines the spi hw register states.
 */
typedef struct 
{
    NvU32 Command;    
    NvU32 Status;
    NvU32 DmaControl;
} SpiHwRegisters;

/**
 * Combines the slink hw register states.
 */
typedef struct 
{
    NvU32 Command1;    
    NvU32 Command2;    
    NvU32 Status;
    NvU32 DmaControl;
} SlinkHwRegisters;

/**
 * Making the union of the spi/slink hw register states.
 */
typedef union
{
    SpiHwRegisters SpiRegs;
    SlinkHwRegisters SlinkRegs;
} SerialHwRegistersState;


/**
 * Combines the definition of the spi register and modem signals.
 */
typedef struct 
{
    // Serial channel Id.
    NvU32 InstanceId;

    // Virtual base address of the spi hw register.
    NvU32 *pRegsBaseAdd;

    NvRmPhysAddr HwTxFifoAdd;

    NvRmPhysAddr HwRxFifoAdd;

    NvU32 RegBankSize;

    NvBool IsPackedMode;

    NvU32 PacketLength;

    NvOdmQuerySpiSignalMode CurrSignalMode;

    NvOdmQuerySpiSignalMode IdleSignalMode;

    NvBool IsIdleDataOutHigh;
    
    NvBool IsLsbFirst;

    SerialHwRegistersState HwRegs;

    NvU32 MaxWordTransfer;
    NvBool IsMasterMode;

    // Tells whether the non word aligned packet size is supported or not.
    // If it is supported then we need not to do any sw workaround otherwise
    // Transfer the nearest word aligned packet using the packed mode and 
    // remaining as non-packed format.
    NvBool IsNonWordAlignedPackModeSupported;

    /// Flag to tell whether the Hw based chipselect is supported or not.
    NvBool IsHwChipSelectSupported;
} SerialHwRegisters;

/**
 * Combines the spi  hw data flow direction where it is the receive side 
 * or transmit side.
 */
typedef enum 
{
    // No data transfer.
    SerialHwDataFlow_None      = 0x0,

    // Receive data flow.
    SerialHwDataFlow_Rx        = 0x1,
    
    // Transmit data flow.
    SerialHwDataFlow_Tx       = 0x2,
    
    SerialHwDataFlow_Force32 = 0x7FFFFFFF
} SerialHwDataFlow;

/**
 * Combines the spi interrupt reasons.
 */
typedef enum 
{
    // No Serial interrupt reason.
    SerialHwIntReason_None           = 0x0,
    
    // Receive error Serial interrupt reason.
    SerialHwIntReason_RxError   = 0x1,

    // Transmit Error spi interrupt reason.
    SerialHwIntReason_TxError  = 0x2,

    // Transfer complete interrupt reason.
    SerialHwIntReason_TransferComplete = 0x4,
    
    SerialHwIntReason_Force32 = 0x7FFFFFFF
} SerialHwIntReason;

/**
 * Combines the spi hw fifo type.
 */
typedef enum 
{
    // Receive fifo type.
    SerialHwFifo_Rx = 0x1,

    // Transmit fifo type.
    SerialHwFifo_Tx = 0x2,

    // Both Rx and Tx fifo
    SerialHwFifo_Both = 0x3,
    
    SerialHwFifo_Force32 = 0x7FFFFFFF
    
} SerialHwFifo;


// The structure of the function pointers to provide the interface to access
// the spi/slink hw registers and their property.
typedef struct
{
    /**
     * Initialize the spi register.
     */
    void (* HwRegisterInitializeFxn)(NvU32 SerialChannelId, SerialHwRegisters *pHwRegs);

    /**
     * Initialize the spi controller.
     */
    void (* HwControllerInitializeFxn)(SerialHwRegisters *pHwRegs);

    /**
     * Set the functional mode whether this is the master or slave mode.
     */
    void (* HwSetFunctionalModeFxn)(SerialHwRegisters *pHwRegs, NvBool IsMasterMode);

    /**
     * Set the signal mode of communication whether this is the mode  0, 1, 2 or 3.
     */
    void (* HwSetSignalModeFxn)(SerialHwRegisters *pHwRegs, NvOdmQuerySpiSignalMode SignalMode);

    /**
     * Reset the fifo
     */
    void (* HwResetFifoFxn)(SerialHwRegisters *pHwRegs, SerialHwFifo FifoType);

    /**
     * Find out whether transmit fifo is full or not.
     */
    NvBool (* HwIsTransmitFifoFull)(SerialHwRegisters *pHwRegs);

    /**
     * Set the transfer order whether the bit will start from the lsb or from
     * msb.
     */
    void (* HwSetTransferBitOrderFxn)(SerialHwRegisters *pHwRegs, NvBool IsLsbFirst);

    /**
     * Start the transfer of the communication.
     */
    void (* HwStartTransferFxn)(SerialHwRegisters *pHwRegs, NvBool IsReconfigure);

    /**
     * Enable/disable the data transfer flow.
     */
    void 
    (* HwSetDataFlowFxn)(
        SerialHwRegisters *pHwRegs, 
        SerialHwDataFlow DataFlow, 
        NvBool IsEnable);

    /**
     * Set the chip select signal level to be default based on device during the
     * initialization.
     */
    void
    (* HwSetChipSelectDefaultLevelFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 ChipSelectId,
        NvBool IsHigh);

   /**
     * Set the chip select signal level.
     */
    void
    (* HwSetChipSelectLevelFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 ChipSelectId,
        NvBool IsHigh);
        
    /**
     * Set the chip select signal level based on the transfer size.
     * it can use the hw based CS or SW based CS based on transfer size and
     * cpu/apb dma based transfer.
     * Return NV_TRUE if the SW based chipselection is used otherwise return
     * NV_FALSE;
     */
    NvBool
    (* HwSetChipSelectLevelBasedOnPacketFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 ChipSelectId,
        NvBool IsHigh,
        NvU32 PacketRequested,
        NvU32 PacketPerWord,
        NvBool IsApbDmaBasedTransfer,
        NvBool IsOnlyUseSWCS);

    /**
     * Set the CS setup and hold time.
     */
    void
    (* HwSetCsSetupHoldTime)(
        SerialHwRegisters *pHwRegs,
        NvU32 CsSetupTimeInClocks,
        NvU32 CsHoldTimeInClocks);

    /**
     * Set active CS id for slave.
     */
    void
    (* HwSetSlaveCsIdFxn)(
        SerialHwRegisters *pHwRegs,
        NvU32 CsId,
        NvBool IsHigh);

    /**
     * Set the packet length.
     */
    void
    (* HwSetPacketLengthFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 PacketLength, 
        NvBool IsPackedMode);

    /**
     * Set the Dma transfer size.
     */
    void
    (* HwSetDmaTransferSizeFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 DmaBlockSize);

    /**
     * Get the transferred packet count.
     */
    NvU32 (* HwGetTransferdCountFxn)(SerialHwRegisters *pHwRegs);


    /**
     * Set the trigger level.
     */
    void 
    (* HwSetTriggerLevelFxn)(
        SerialHwRegisters *pHwRegs, 
        SerialHwFifo FifoType,
        NvU32 TriggerLevel);

    /**
     * Write into the transmit fifo register.
     * returns the number of words written.
     */
    NvU32
    (* HwWriteInTransmitFifoFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 *pTxBuff,  
        NvU32 WordRequested);
        
    /**
     * Read the data from the receive fifo.
     * Returns the number of words it read.   
     */
    NvU32
    (* HwReadFromReceiveFifoFxn)(
        SerialHwRegisters *pHwRegs, 
        NvU32 *pRxBuff,  
        NvU32 WordRequested);

    /**
     * Enable/disable the interrupt source. 
     */
    void
    (* HwSetInterruptSourceFxn)(
        SerialHwRegisters *pHwRegs, 
        SerialHwDataFlow DataDirection,
        NvBool IsEnable);

    /**
     * Get the transfer status. 
     */
    NvError 
    (* HwGetTransferStatusFxn)(
        SerialHwRegisters *pHwRegs,
        SerialHwDataFlow DataDirection);

    /**
     * Clear the transfer status. 
     */
    void 
    (* HwClearTransferStatusFxn)(
        SerialHwRegisters *pHwRegs,
        SerialHwDataFlow DataDirection);


    /**
     * Clear the fifos before starting new transfer to make sure there is no
     * residuals from past.
     */
    NvBool
    (* HwClearFifosForNewTransferFxn)(
        SerialHwRegisters *pHwRegs,
        SerialHwDataFlow DataDirection);

    /**
     * Check whether transfer is completed or not. 
     */
    NvBool (* HwIsTransferCompletedFxn)( SerialHwRegisters *pHwRegs);
} HwInterface, *HwInterfaceHandle;


/**
 * Initialize the spi intterface for the hw access.
 */
void NvRmPrivSpiSlinkInitSpiInterface(HwInterface *pSpiInterface);

/**
 * Initialize the slink intterface for the hw access which are common across 
 * the version.
 */
void NvRmPrivSpiSlinkInitSlinkInterface(HwInterface *pSpiInterface);

/**
 * Initialize the slink interface of version 1.0 for the hw access.
 */
void NvRmPrivSpiSlinkInitSlinkInterface_v1_0(HwInterface *pSlinkInterface);


/**
 * Initialize the ap20 slink interface of version 1.1 for the hw access.
 */
void NvRmPrivSpiSlinkInitSlinkInterface_v1_1(HwInterface *pSlinkInterface);

/** @} */

#if defined(__cplusplus)
}
#endif

#endif  // INCLUDED_RMSPI_HW_PRIVATE_H
