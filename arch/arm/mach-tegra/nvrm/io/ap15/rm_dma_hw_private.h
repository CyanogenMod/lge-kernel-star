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
 *           Private functions for the dma resource manager</b>
 *
 * @b Description:  Defines the HW access of the apb dma register.
 * 
 */

#ifndef INCLUDED_NVRM_DMA_HW_PRIVATE_H
#define INCLUDED_NVRM_DMA_HW_PRIVATE_H

/**
 * @defgroup nvrm_dma Direct Memory Access (DMA) Hw controller interface API.
 * 
 * This is the Hw Dma controller interface.  These API provides the register 
 * access of the dma controller register. This configures the hw related dma 
 * information in the passed parameters.
 *
 * @ingroup nvddk_rm
 * 
 * @{
 */

#include "nvcommon.h"
#include "nvrm_dma.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Combines the apb Dma regsiters physical base address, channel address, 
 * bank size of the channel address and  general controller address.
 */
typedef struct 
{   
    NvU32 GenAddBankSize;
    NvU32 *pGenVirtBaseAdd;
} DmaGenRegisters;


/**
 * Combines the Dma register which contains the APB register sets.
 */
typedef struct
{
    // Virtual address Pointer to the dma channel base register. 
    NvU32 *pHwDmaChanReg; 

    NvU32 ControlReg;
    NvU32 StatusReg;
    NvU32 AhbAddressReg;
    NvU32 ApbAddressReg;
    NvU32 XmbAddressReg;
    NvU32 AhbSequenceReg;
    NvU32 XmbSequenceReg;
    NvU32 ApbSequenceReg;

    NvU32 TransferedCount;
} DmaChanRegisters;


typedef struct DmaHwInterfaceRec
{
    /**
     * Configure the Apb dma register as per clients information.
     * This function do the register setting based on device Id and will  be stored 
     * in the dma handle. This information will be used when there is dma transfer 
     * request and want to configure the dma controller registers. 
     */
    void
    (*DmaHwInitRegistersFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvRmDmaModuleID DmaReqModuleId,
        NvU32 DmaReqInstId);

    /**
     * Global Enable/disable the dma controller.
     */
    void (*DmaHwGlobalSetFxn)(NvU32 *pGenVirtBaseAddress, NvBool IsEnable);

    /**
     * Continue the remaining transfer.
     */
    void (*DmaContinueRemainingTransferFxn)(void *pDmaChannel);

    NvError  (*LogDmaTransferRequestFxn)(NvRmDmaHandle hDma, void *pCurrRequest);

    /**
     * Configure the address registers of the dma from the client buffer
     * source and destination address.
     */
    void
    (*DmaHwConfigureAddressFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvRmPhysAddr SourceAdd,
        NvRmPhysAddr DestAdd,
        NvBool IsSourceAddPeripheralXmbType);

    /**
     * Set the data transfer size for the apb dma.
     */
    void
    (*DmaHwSetTransferSizeFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvU32 TransferSize,
        NvBool IsDoubleBuffMode);

    /**
     * Get the transferred count for apb dma.
     */
    NvU32 (*DmaHwGetTransferredCountFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Get the transferred count for apb dma.
     */
    NvU32 (*DmaHwGetTransferredCountWithStopFxn)(
        DmaChanRegisters *pDmaChRegs,
        NvBool IsTransferStop);

    /**
     * Add the transfer count in the dma transferred size.
     */
    void (*DmaHwAddTransferCountFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Set the transferred mode for apb dma.
     */
    void
    (*DmaHwSetTransferModeFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvBool IsContinuousMode,
        NvBool IsDoubleBuffMode);

    /**
     * Set the dma direction of data transfer.
     */
    void 
    (*DmaHwSetDirectionFxn)(
        DmaChanRegisters *pDmaChRegs, 
        NvBool IsSourceAddPerXmbType);
        
    /**
     * Set the dma burst size in the dma registers copy.
     */
    void
    (*DmaHwSetBurstSizeFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvRmDmaModuleID DmaReqModuleId,
        NvU32 TransferSize);

    /**
     * Enable the bit swap for the destionation address for apb dma.
     */
    void
    (*DmaHwEnableDestBitSwapFxn)(    
        DmaChanRegisters *pDmaChRegs,
        NvBool IsDestAddPeripheralXmbType);

    /**
     * Set the address wrapping information for dma.
     * The different address wrapping is supported by dma.
     */
    void
    (*DmaHwSetAddressWrappingFxn)(
        DmaChanRegisters *pDmaChRegs,
        NvRmPhysAddr SourceAddWrap,
        NvRmPhysAddr DestAddWrap,
        NvU32 TransferSize,
        NvBool IsSourceAddPeripheralXmbType);

    /**
     * Start the dma transfer from the current request. 
     * This will start the dma transfer.
     */
    void (*DmaHwStartTransferFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Continue the dma transfer special for the continuous mode. 
     */
    void (*DmaHwContinueTransferFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Start the dma transfer from the current request. This will read the 
     * current configured address from the register and increment them as per  
     * passed parameter and start the dma transfer.
     */
    void 
    (*DmaHwStartTransferWithAddIncFxn)(
        DmaChanRegisters *pDmaChRegs,
        NvU32 XmbPeriAddIncSize,
        NvU32 MemoryAddIncSize,
        NvU32 IsContMode);

    /**
     * Stop the data transfer in the given dma channel number.
     */
    void (*DmaHwStopTransferFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Check whether the dma transfer is completed or not for the given channel.
     */
    NvBool (*DmaHwIsTransferCompletedFxn)(DmaChanRegisters *pDmaChRegs);

    /**
     * Ack and clear the interrupt of dma channel.
     */
    void (*DmaHwAckNClearInterruptFxn)(DmaChanRegisters *pDmaChRegs);
} DmaHwInterface;



/**
 * Tells whether the given address is on Xmb or not.
 */
NvBool NvRmPrivDmaHwIsValidXmbAddress(NvRmPhysAddr PhysAddress);

/**
 * Tells whether the given address is valid peripheral device address or not.
 */
NvBool NvRmPrivDmaHwIsValidPeripheralAddress(NvRmPhysAddr PhysAddress) ;


void NvRmPrivDmaInitAp15DmaHwInterfaces(DmaHwInterface *pApbDmaInterface);

void NvRmPrivDmaInitDmaHwInterfaces(DmaHwInterface *pApbDmaInterface);

NvU32 NvRmPrivDmaInterruptDecode(NvRmDeviceHandle hRmDevice );

void NvRmPrivDmaInterruptEnable(NvRmDeviceHandle hRmDevice, NvU32 Channel, NvBool Enable );

#if defined(__cplusplus)
}
#endif

/** @} */

#endif  // INCLUDED_NVRM_DMA_HW_PRIVATE_H
