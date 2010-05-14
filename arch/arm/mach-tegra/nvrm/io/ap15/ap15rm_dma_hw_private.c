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
 * @b Description: Implements the private interface of the nnvrm dma to access
 * the hw apb/ahb dma register.
 *
 * This files implements the API for accessing the register of the Dma 
 * controller and configure the dma transfers for Ap15.
 */

#include "nvrm_dma.h"
#include "rm_dma_hw_private.h"
#include "ap20/arapbdma.h"
#include "ap20/arapbdmachan.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hardware_access.h"

#define APBDMACHAN_READ32(pVirtBaseAdd, reg) \
        NV_READ32((pVirtBaseAdd) + ((APBDMACHAN_CHANNEL_0_##reg##_0)/4))
#define APBDMACHAN_WRITE32(pVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32(((pVirtBaseAdd) + ((APBDMACHAN_CHANNEL_0_##reg##_0)/4)), (val)); \
    } while(0)


static void
ConfigureDmaRequestor(
    DmaChanRegisters *pDmaChRegs,
    NvRmDmaModuleID DmaReqModuleId,
    NvU32 DmaReqInstId)
{
    // Check for the dma module Id and based on the dma module Id, decide
    // the trigger requestor source.
    switch (DmaReqModuleId)
    {
        /// Specifies the dma module Id for memory
        case NvRmDmaModuleID_Memory:
            // Dma transfer will be from memory to memory.
            // Use the reset value only for the ahb data transfer.
            break;


        case NvRmDmaModuleID_I2s:
            // Dma requestor is the I2s controller.
            NV_ASSERT(DmaReqInstId < 2);
            if (DmaReqInstId == 0)
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, I2S_1, pDmaChRegs->ControlReg);
            else 
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, I2S2_1, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32, 
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Uart:
            // Dma requestor is the uart.
            NV_ASSERT(DmaReqInstId < 5);
            switch (DmaReqInstId)
            {
                default:
                case 0:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                            CSR, REQ_SEL, UART_A, pDmaChRegs->ControlReg);
                    break;
                case 1:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                            CSR, REQ_SEL, UART_B, pDmaChRegs->ControlReg);
                    break;
                case 2:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                            CSR, REQ_SEL, UART_C, pDmaChRegs->ControlReg);
                    break;
                case 3:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                            CSR, REQ_SEL, UART_D, pDmaChRegs->ControlReg);
                    break;
                case 4:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                            CSR, REQ_SEL, UART_E, pDmaChRegs->ControlReg);
                    break;
            }
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                        APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_8,
                                        pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Vfir:
            // Dma requestor is the vfir.
            NV_ASSERT(DmaReqInstId < 1);
            if (DmaReqInstId == 1)
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, UART_B, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Mipi:
            // Dma requestor is the Mipi controller.
            NV_ASSERT(DmaReqInstId < 1);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, MIPI, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Spi:
            // Dma requestor is the Spi controller.
            NV_ASSERT(DmaReqInstId < 1);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, SPI, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, 
                                            CSR, TRIG_SEL, 0, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Slink:
            // Dma requestor is the Slink controller.
            NV_ASSERT(DmaReqInstId < 3);
            if (DmaReqInstId == 0)
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, SL2B1, pDmaChRegs->ControlReg);
            else if (DmaReqInstId == 1)
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, SL2B2, pDmaChRegs->ControlReg);
            else
                pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, SL2B3, pDmaChRegs->ControlReg);
            
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, 
                                            CSR, TRIG_SEL, 0, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_Spdif:
            // Dma requestor is the Spdif controller.
            NV_ASSERT(DmaReqInstId < 1);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, SPD_I, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;

        case NvRmDmaModuleID_I2c:
            // Dma requestor is the I2c controller.
            NV_ASSERT(DmaReqInstId < 3);
            switch (DmaReqInstId)
            {
                default:
                case 0:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                                CSR, REQ_SEL, I2C, 
                                                    pDmaChRegs->ControlReg);
                    break;
                case 1:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                                CSR, REQ_SEL, I2C2, 
                                                    pDmaChRegs->ControlReg);
                    break;
                case 2:
                    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                                CSR, REQ_SEL, I2C3, 
                                                    pDmaChRegs->ControlReg);
                    break;
            }
        
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;
        
        case NvRmDmaModuleID_Dvc:
            // Dma requestor is the I2c controller.
            NV_ASSERT(DmaReqInstId < 1);
        
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, REQ_SEL, DVC_I2C, pDmaChRegs->ControlReg);
            pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            CSR, FLOW, ENABLE, pDmaChRegs->ControlReg);
            pDmaChRegs->ApbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                            APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32,
                                            pDmaChRegs->ApbSequenceReg);
            break;


        default:
            NV_ASSERT(!"Invalid module");
    }
}

/**
 * Configure the Apb dma register as per clients information.
 * This function do the register setting based on device Id and will  be stored
 * in the dma handle. This information will be used when there is dma transfer
 * request and want to configure the dma controller registers.
 */
static void 
InitApbDmaRegisters(
    DmaChanRegisters *pDmaChRegs,
    NvRmDmaModuleID DmaReqModuleId,
    NvU32 DmaReqInstId)
{
    pDmaChRegs->pHwDmaChanReg = NULL;

    //  Set the dma register of dma handle to their power on reset values.
    pDmaChRegs->ControlReg = NV_RESETVAL(APBDMACHAN_CHANNEL_0, CSR);
    pDmaChRegs->AhbSequenceReg = NV_RESETVAL(APBDMACHAN_CHANNEL_0, AHB_SEQ);
    pDmaChRegs->ApbSequenceReg = NV_RESETVAL(APBDMACHAN_CHANNEL_0,APB_SEQ);

    // Configure the dma register for the OnceMode
    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, 
                                ONCE, SINGLE_BLOCK, pDmaChRegs->ControlReg);

    // Configure the dma register for  enabling the interrupt so that it will generate the interrupt
    // after transfer completes.
    pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, 
                                IE_EOC, ENABLE, pDmaChRegs->ControlReg);

    // Configure the dma register for  interrupting the cpu only.
    pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                AHB_SEQ, INTR_ENB, CPU, pDmaChRegs->AhbSequenceReg);

    // Configure the dma registers as per requestor information.
    ConfigureDmaRequestor(pDmaChRegs, DmaReqModuleId, DmaReqInstId);
}

/**
 * Set the data transfer mode for the dma transfer.
 */
static void 
SetApbDmaTransferMode(
    DmaChanRegisters *pDmaChRegs,
    NvBool IsContinuousMode,
    NvBool IsDoubleBuffMode)
{
    // Configure the dma register for the Continuous Mode
    if (IsContinuousMode)
        pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    CSR, ONCE, MULTIPLE_BLOCK, pDmaChRegs->ControlReg);
    else
        pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    CSR, ONCE, SINGLE_BLOCK, pDmaChRegs->ControlReg);

    // Configure the dma register for the double buffering Mode
    if (IsDoubleBuffMode)
        pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    AHB_SEQ, DBL_BUF, RELOAD_FOR_2X_BLOCKS, 
                                    pDmaChRegs->AhbSequenceReg);
    else
        pDmaChRegs->AhbSequenceReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, 
                                    AHB_SEQ, DBL_BUF, RELOAD_FOR_1X_BLOCKS, 
                                    pDmaChRegs->AhbSequenceReg);
}

/**
 * Set the Apb dma direction of data transfer.
 */
static void 
SetApbDmaDirection(
    DmaChanRegisters *pDmaChRegs,
    NvBool IsSourceAddPerType)
{
    if (IsSourceAddPerType)
        pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
                                    CSR, DIR, AHB_WRITE, pDmaChRegs->ControlReg);
    else
        pDmaChRegs->ControlReg = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
                                    CSR, DIR, AHB_READ, pDmaChRegs->ControlReg);
}

void NvRmPrivDmaInitAp15DmaHwInterfaces(DmaHwInterface *pApbDmaInterface)
{

    pApbDmaInterface->DmaHwInitRegistersFxn = InitApbDmaRegisters;
    pApbDmaInterface->DmaHwSetTransferModeFxn = SetApbDmaTransferMode;
    pApbDmaInterface->DmaHwSetDirectionFxn = SetApbDmaDirection;
}
