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

/** @file
 * @brief <b>NVIDIA Driver Development Kit: I2C API</b>
 *
 * @b Description: Contains the NvRM I2C implementation. for Ap20 
 */

#include "nvrm_i2c.h"
#include "nvrm_i2c_private.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "ap20/ari2c.h"
#include "nvrm_hardware_access.h"
#include "nvrm_power.h" 
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "ap20/ardvc.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pinmux.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"

/* Register access Macros */
#define I2C_REGR(c, reg) NV_REGR((c)->hRmDevice, (c)->ModuleId, (c)->Instance,  \
        (c)->I2cRegisterOffset + (((c)->ModuleId == NvRmModuleID_Dvc) ? DVC_##reg##_0 : \
                                                I2C_##reg##_0)) 

#define I2C_REGW(c, reg, val) \
        do { \
            NV_REGW((c)->hRmDevice, (c)->ModuleId, (c)->Instance, \
                ((c)->I2cRegisterOffset + (((c)->ModuleId == NvRmModuleID_Dvc) ? DVC_##reg##_0 : \
                                            I2C_##reg##_0)), (val)); \
        } while(0)

#define DVC_REGR(c, reg)    NV_REGR((c)->hRmDevice, NvRmModuleID_Dvc, (c)->Instance, \
                    DVC_##reg##_0)
#define DVC_REGW(c, reg, val) \
        do {    \
            NV_REGW((c)->hRmDevice, NvRmModuleID_Dvc, (c)->Instance, \
                    DVC_##reg##_0, val); \
        } while(0)


/* Register access Macros */
#define I2C2_REGR(c, reg) NV_REGR((c)->hRmDevice, (c)->ModuleId, (c)->Instance,  \
                    (c)->I2cRegisterOffset + I2C_##reg##_0 )

#define I2C2_REGW(c, reg, val) \
        do {    \
                NV_REGW((c)->hRmDevice, (c)->ModuleId, (c)->Instance, \
                    ((c)->I2cRegisterOffset + I2C_##reg##_0), (val) ); \
        } while(0);
                    

#define DEBUG_SEND_PROCESS 0
#define DEBUG_READ_PROCESS 0
#define DEBUG_TRACE_PROCESS 0

#if DEBUG_SEND_PROCESS
#define DEBUG_I2C_SEND(Expr, Format) \
                    do { \
                        if (Expr) \
                        {   \
                            NvOsDebugPrintf Format; \
                        }   \
                    } while(0)
#else
#define DEBUG_I2C_SEND(Expr, Format)
#endif

#if DEBUG_READ_PROCESS
#define DEBUG_I2C_READ(Expr, Format) \
                    do { \
                        if (Expr) \
                        {   \
                            NvOsDebugPrintf Format; \
                        }   \
                    } while(0)
#else
#define DEBUG_I2C_READ(Expr, Format)
#endif

#if DEBUG_TRACE_PROCESS
#define DEBUG_I2C_TRACE(Expr, Format) \
                    do { \
                        if (Expr) \
                        {   \
                            NvOsDebugPrintf Format; \
                        } \
                    } while(0)
#else
#define DEBUG_I2C_TRACE(Expr, Format)
#endif

// The maximum transfer size by one transaction.
enum {MAX_I2C_ONE_TRANSACTION_SIZE = 0x1000}; // 4KB

// The maximum request size for one transaction using the dma.
// + 64 bytes for the packet header.
enum {DEFAULT_I2C_DMA_BUFFER_SIZE = (MAX_I2C_ONE_TRANSACTION_SIZE + 0x40)}; // 4KB

// The default request size for one transaction using the nondma mode.
enum {DEFAULT_I2C_CPU_BUFFER_SIZE = MAX_I2C_ONE_TRANSACTION_SIZE};

// Wait time to poll the status for completion.
enum { I2C_POLLING_TIMEOUT_STEP_USEC = 1000};

// I2C fifo depth.
enum { I2C_FIFO_DEPTH = 8};

// I2C Dma/CPU based seletion thresold.
enum { I2C_MAX_WORD_TO_USE_CPU = 16};

// Holding the apb dma for the continuous non dma transaction count
enum {HOLDING_DMA_TRANSACTION_COUNT = 15};
#define I2C_TRANSACTION_STATUS_ERRORS \
    (NV_DRF_DEF(I2C, INTERRUPT_STATUS_REGISTER, TFIFO_OVF, SET) | \
     NV_DRF_DEF(I2C, INTERRUPT_STATUS_REGISTER, RFIFO_UNF, SET) | \
     NV_DRF_DEF(I2C, INTERRUPT_STATUS_REGISTER, ARB_LOST, SET))

#define I2C_ARBITRATION_LOST_ERRORS \
         (NV_DRF_DEF(I2C, INTERRUPT_STATUS_REGISTER, ARB_LOST, SET))

#define I2C_ERRORS_INTERRUPT_MASK \
    (NV_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, TFIFO_OVF_INT_EN, ENABLE) | \
     NV_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, RFIFO_UNF_INT_EN, ENABLE) | \
     NV_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, ARB_LOST_INT_EN, ENABLE))

#if NV_OAL
#define USE_POLLING_METHOD 1
#else
#define USE_POLLING_METHOD 0
#endif

#if USE_POLLING_METHOD
#define RESET_SEMA_COUNT(hSema) 
#else
#define RESET_SEMA_COUNT(hSema) \
    while(NvOsSemaphoreWaitTimeout(hSema, 0) != NvError_Timeout)
#endif

// Convert the number of bytes to word.
#define BYTES_TO_WORD(ReqSize) (((ReqSize) + 3) >> 2)

/**
 * Create the dma buffer memory handle.
 */
static NvError
CreateDmaBufferMemoryHandle(
    NvRmDeviceHandle hDevice,
    NvRmMemHandle *phNewMemHandle,
    NvRmPhysAddr *pNewMemAddr,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hNewMemHandle = NULL;
    static const NvRmHeap HeapProperty[] =
    {
        NvRmHeap_ExternalCarveOut,
        NvRmHeap_External,
        NvRmHeap_GART,
    };

    // Initialize the memory handle with NULL
    *phNewMemHandle = NULL;

    /// Create memory handle
    Error = NvRmMemHandleCreate(hDevice, &hNewMemHandle, BufferSize);

    // Allocates the memory from the sdram
    if (!Error)
        Error = NvRmMemAlloc(hNewMemHandle, HeapProperty,
                        NV_ARRAY_SIZE(HeapProperty), 4, NvOsMemAttribute_Uncached);

    // Pin the memory allocation so that it should not move by memory manager.
    if (!Error)
        *pNewMemAddr = NvRmMemPin(hNewMemHandle);

    // If error then free the memory allocation and memory handle.
    if (Error)
    {
        NvRmMemHandleFree(hNewMemHandle);
        hNewMemHandle = NULL;
    }

    *phNewMemHandle = hNewMemHandle;
    return Error;
}

 /**
  * Destroy the dma buffer memory handle.
  */
static void DestroyDmaBufferMemoryHandle(NvRmMemHandle hMemHandle)
{
    // Can accept the null parameter. If it is not null then only destroy.
    if (hMemHandle)
    {
        // Unpin the memory allocation.
        NvRmMemUnpin(hMemHandle);

        // Free the memory handle.
        NvRmMemHandleFree(hMemHandle);
    }
}

/**
 * Create the dma transfer buffer for the given handles.
 */
static NvError
CreateDmaTransferBuffer(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle *phRmMemory,
    NvRmPhysAddr *pBuffPhysAddr,
    void **pBuffPtr,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hRmMemory = NULL;
    NvRmPhysAddr BuffPhysAddr;

    // Reset all the members realted to the dma buffer.
    BuffPhysAddr = 0;

    *phRmMemory = NULL;
    *pBuffPtr = (void *)NULL;
    *pBuffPhysAddr = 0;

    // Create the dma buffer memory for receive and transmit.
    // It will be double of the OneBufferSize
    Error = CreateDmaBufferMemoryHandle(hRmDevice, &hRmMemory, 
                                                &BuffPhysAddr, BufferSize);
    if (!Error)
    {
        // 0 to OneBufferSize-1 is buffer 1 and OneBufferSize to 2*OneBufferSize
        // is second buffer.
        Error = NvRmMemMap(hRmMemory, 0, BufferSize, 
                                                NVOS_MEM_READ_WRITE, pBuffPtr);
        // If error then free the allocation and reset all changed value.
        if (Error)
        {
            DestroyDmaBufferMemoryHandle(hRmMemory);
            hRmMemory = NULL;
            *pBuffPtr = (void *)NULL;
            return Error;
        }
        *phRmMemory = hRmMemory;
        *pBuffPhysAddr = BuffPhysAddr;
    }
    return Error;
}

/**
 * Destroy the dma transfer buffer.
 */
static void
DestroyDmaTransferBuffer(
    NvRmMemHandle hRmMemory,
    void *pBuffPtr,
    NvU32 BufferSize)
{
    if (hRmMemory)
    {
        if (pBuffPtr)
            NvRmMemUnmap(hRmMemory, pBuffPtr, BufferSize);
        DestroyDmaBufferMemoryHandle(hRmMemory);
    }
}

static void SetTxFifoTriggerLevel(NvRmI2cControllerHandle hRmI2cCont, NvU32 TrigLevel)
{
    NvU32 FifoControlReg;
    NvU32 ActualTriggerLevel = NV_MIN(I2C_FIFO_DEPTH, TrigLevel);

    if (!ActualTriggerLevel)
        return;
    
    FifoControlReg = I2C_REGR (hRmI2cCont, FIFO_CONTROL);
    FifoControlReg = NV_FLD_SET_DRF_NUM(I2C, FIFO_CONTROL, TX_FIFO_TRIG, 
                                                ActualTriggerLevel - 1, FifoControlReg);
    DEBUG_I2C_SEND(1, ("Tx Fifo Control  0x%08x\n", FifoControlReg));
    I2C_REGW (hRmI2cCont, FIFO_CONTROL, FifoControlReg);
}

static void SetRxFifoTriggerLevel(NvRmI2cControllerHandle hRmI2cCont, NvU32 TrigLevel)
{
    NvU32 FifoControlReg;
    NvU32 ActualTriggerLevel = NV_MIN(I2C_FIFO_DEPTH, TrigLevel);

    if (!ActualTriggerLevel)
        return;
        
    FifoControlReg = I2C_REGR (hRmI2cCont, FIFO_CONTROL);
    FifoControlReg = NV_FLD_SET_DRF_NUM(I2C, FIFO_CONTROL, RX_FIFO_TRIG, 
                                                ActualTriggerLevel - 1, FifoControlReg);
    DEBUG_I2C_READ(1, ("Rx Fifo Control  0x%08x\n", FifoControlReg));
    I2C_REGW (hRmI2cCont, FIFO_CONTROL, FifoControlReg);
}


static void ResetTxFifo(NvRmI2cControllerHandle hRmI2cCont)
{
    NvU32 FifoControlReg;
    
    FifoControlReg = I2C_REGR (hRmI2cCont, FIFO_CONTROL);
    FifoControlReg = NV_FLD_SET_DRF_DEF(I2C, FIFO_CONTROL, TX_FIFO_FLUSH, 
                                                SET, FifoControlReg);
    I2C_REGW (hRmI2cCont, FIFO_CONTROL, FifoControlReg);
    do
    {
        NvOsWaitUS(10);
        FifoControlReg = I2C_REGR (hRmI2cCont, FIFO_CONTROL);
    }while(FifoControlReg & NV_DRF_DEF(I2C, FIFO_CONTROL, TX_FIFO_FLUSH, SET));
}

static void DoDvcI2cControlInitialization(NvRmI2cControllerHandle hRmI2cCont)
{
    NvU32 RegVal = 0;
    
    RegVal = DVC_REGR(hRmI2cCont, CTRL_REG3);
    RegVal = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_HW_SW_PROG, SW, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_DONE_INTR_EN, ENABLE, RegVal);
    DVC_REGW(hRmI2cCont, CTRL_REG3, RegVal);
    
    RegVal = DVC_REGR(hRmI2cCont, CTRL_REG1);
    RegVal = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG1, INTR_EN, ENABLE, RegVal);
    DVC_REGW(hRmI2cCont, CTRL_REG1, RegVal);
}

static void UseDvcI2cNewSlave(NvRmI2cControllerHandle hRmI2cCont)
{
    NvU32 RegVal = 0;
    RegVal = NV_DRF_DEF(I2C, I2C_SL_CNFG, NEWSL, ENABLE);
    I2C2_REGW(hRmI2cCont, I2C_SL_CNFG, RegVal);
    
    RegVal = NV_DRF_NUM(I2C, I2C_SL_ADDR1, SL_ADDR0, 0xF);
    I2C2_REGW(hRmI2cCont, I2C_SL_ADDR1, RegVal);
}

static void 
GetPacketHeaders(
    NvRmI2cControllerHandle hRmI2cCont,
    NvRmI2cTransactionInfo *pTransaction,
    NvU32 PacketId,
    NvU32 *pPacketHeader1,
    NvU32 *pPacketHeader2,
    NvU32 *pPacketHeader3)
{
    NvU32 PacketHeader1;
    NvU32 PacketHeader2;
    NvU32 PacketHeader3;

    // prepare Generic header1
    // Header size = 0 Protocol  = I2C,pktType = 0
    PacketHeader1 = NV_DRF_DEF(I2C, IO_PACKET_HEADER, HDRSZ, ONE) | 
                              NV_DRF_DEF(I2C, IO_PACKET_HEADER, PROTOCOL, I2C);
    
    // Set pkt id as 1
    PacketHeader1 = NV_FLD_SET_DRF_NUM(I2C, IO_PACKET_HEADER, PKTID, PacketId, PacketHeader1);
    
    // Controller id is according to the instance of the i2c/dvc
    PacketHeader1 = NV_FLD_SET_DRF_NUM(I2C, IO_PACKET_HEADER, 
                               CONTROLLER_ID, hRmI2cCont->ControllerId, PacketHeader1);
    
    PacketHeader2 = NV_FLD_SET_DRF_NUM(I2C, IO_PACKET_HEADER, 
                                    PAYLOADSIZE, (pTransaction->NumBytes - 1), 0);
    
    // prepare IO specific header
    // Configure the slave address
    PacketHeader3 = pTransaction->Address;
    
    // 10 bit address mode: Set address mode to 10 bit
    if (hRmI2cCont->Is10BitAddress)
        PacketHeader3 = NV_FLD_SET_DRF_DEF(I2C, IO_PACKET_HEADER, ADDR_MODE, 
                                                        TEN_BIT, PacketHeader3);
    
    hRmI2cCont->IsCurrentTransferNoAck = NV_FALSE;
    // Enable mode to handle devices that do not generate ACK 
    if (pTransaction->Flags & NVRM_I2C_NOACK)
    {
        PacketHeader3 = NV_FLD_SET_DRF_DEF(I2C, IO_PACKET_HEADER, CONTUNE_ON_NACK, 
                                                        ENABLE, PacketHeader3);
        hRmI2cCont->IsCurrentTransferNoAck = NV_TRUE;
    }
    
    hRmI2cCont->IsCurrentTransferNoStop = NV_FALSE;

    // Enable mode to repeat start if it is configured
    if (pTransaction->Flags & NVRM_I2C_NOSTOP)
    {
        PacketHeader3 = NV_FLD_SET_DRF_DEF(I2C, IO_PACKET_HEADER,REPEAT_START, 
                                                REPEAT_START, PacketHeader3);
        hRmI2cCont->IsCurrentTransferNoStop = NV_TRUE;
    }                                           

    hRmI2cCont->IsCurrentTransferRead = NV_FALSE;
    // Enable Read if it is required
    if (!(pTransaction->Flags & NVRM_I2C_WRITE))
    {
        PacketHeader3 = NV_FLD_SET_DRF_DEF(I2C, IO_PACKET_HEADER, READ, READ, 
                                                PacketHeader3);
        hRmI2cCont->IsCurrentTransferRead = NV_TRUE;
    }

    *pPacketHeader1 = PacketHeader1;
    *pPacketHeader2 = PacketHeader2;
    *pPacketHeader3 = PacketHeader3;
}

static void StartI2cPacketMode(NvRmI2cControllerHandle hRmI2cCont)
{
    NvU32 I2cConfig;
    // PACKET_MODE_TRANSFER_EN field of I2C Controller configuration Register
    I2cConfig = NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
    I2cConfig = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, PACKET_MODE_EN, GO, I2cConfig);
    I2C_REGW(hRmI2cCont, I2C_CNFG, I2cConfig);
}

static void 
DoTxFifoEmpty(
    NvRmI2cControllerHandle hRmI2cCont, 
    NvU32 *pFifoEmptyCount)
{
    NvU32 TFifoEmptyCount = 0;
    NvU32 FifoStatus;
    
    // Tx Fifo should be empty. If not force to make it empty
    FifoStatus = I2C_REGR(hRmI2cCont, FIFO_STATUS);
    TFifoEmptyCount = NV_DRF_VAL(I2C, FIFO_STATUS, TX_FIFO_EMPTY_CNT, FifoStatus);
    if (TFifoEmptyCount < I2C_FIFO_DEPTH)
        ResetTxFifo(hRmI2cCont);

    *pFifoEmptyCount = TFifoEmptyCount;
}

static void WriteIntMaksReg(NvRmI2cControllerHandle hRmI2cCont)
{
#if !USE_POLLING_METHOD
    I2C_REGW (hRmI2cCont, INTERRUPT_MASK_REGISTER, hRmI2cCont->IntMaskReg);
#endif
}
static void I2cIsr(void* args)
{
    NvRmI2cControllerHandle hRmI2cCont = (NvRmI2cControllerHandle)args;
    NvU32 FifoStatus;
    NvU32 WordCount;
    NvU32 FilledSlots;
    NvU32 MaxWordToRead;
    NvBool IsFinalIntGot = NV_FALSE;
    NvU32 FreeSlots;
    NvU32 MaxWordToWrite;
    // Read the Interrupt status register & PKT_STATUS
    hRmI2cCont->ControllerStatus = I2C_REGR(hRmI2cCont, INTERRUPT_STATUS_REGISTER);

    // Write one to clear in the interrupt status register
    I2C_REGW(hRmI2cCont, INTERRUPT_STATUS_REGISTER, hRmI2cCont->ControllerStatus);
    FifoStatus = I2C_REGR(hRmI2cCont, FIFO_STATUS);

    DEBUG_I2C_READ(1, ("ISR ContStatus 0x%08x FifoStatus 0x%08x\n", 
                                    hRmI2cCont->ControllerStatus, FifoStatus));

    if (hRmI2cCont->ControllerStatus & hRmI2cCont->FinalInterrupt)
        IsFinalIntGot = NV_TRUE;

    // If any error then stop transfer.
    if (hRmI2cCont->ControllerStatus & I2C_TRANSACTION_STATUS_ERRORS)
    {
        NvOsDebugPrintf("Err in I2c transfer: Controller Status 0x%08x \n",
                                hRmI2cCont->ControllerStatus);
        IsFinalIntGot = NV_TRUE;
        goto FinalIntDone;
    }

    if (hRmI2cCont->IsCurrentTransferRead)
    {
        // If there is remianing word to read then read here from fifo.
        if ((hRmI2cCont->WordRemaining) && (!hRmI2cCont->IsUsingApbDma))
        {
            DEBUG_I2C_READ(1, ("Reading RxFifo From Int\n"));

            // Get RFifo full count
            FilledSlots = NV_DRF_VAL(I2C, FIFO_STATUS, RX_FIFO_FULL_CNT, FifoStatus);
        
            MaxWordToRead = NV_MIN(hRmI2cCont->WordRemaining, FilledSlots);
            for (WordCount = 0; WordCount < MaxWordToRead; ++WordCount)
            {
                // Read data from the I2C RX pkt FIFO Register
                hRmI2cCont->pDataBuffer[hRmI2cCont->WordTransferred] = 
                                        I2C_REGR(hRmI2cCont, I2C_RX_FIFO);
                hRmI2cCont->WordTransferred++;
            }
            hRmI2cCont->WordRemaining -= MaxWordToRead;
        
            if ((IsFinalIntGot) || (hRmI2cCont->WordRemaining == 0))
                goto FinalIntDone;

            // If still want to receive more than the fifo depth then continue 
            // the int   
            if (hRmI2cCont->WordRemaining > I2C_FIFO_DEPTH)
                goto IntDone;

            if(hRmI2cCont->IsCurrentTransferNoStop)
            {
                // If remaining required read is less than fifo depth then enable the
                // all tranfer interrupt only and disable the fifo trigger level interrupt
                
                if (hRmI2cCont->WordRemaining < I2C_FIFO_DEPTH)
                    SetRxFifoTriggerLevel(hRmI2cCont, hRmI2cCont->WordRemaining);
                    
                hRmI2cCont->FinalInterrupt = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                RFIFO_DATA_REQ_INT_EN, ENABLE, hRmI2cCont->FinalInterrupt);
            }
            else
            {
                hRmI2cCont->IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                          RFIFO_DATA_REQ_INT_EN, DISABLE, hRmI2cCont->IntMaskReg);

                hRmI2cCont->IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, hRmI2cCont->IntMaskReg);
                                
                
                hRmI2cCont->FinalInterrupt = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, hRmI2cCont->FinalInterrupt);
                WriteIntMaksReg(hRmI2cCont);
            }
            goto IntDone;
        }
    }
    else
    {
        if (IsFinalIntGot)
            goto FinalIntDone;

        // If there is remaining word to write then keep writing it.
        if (hRmI2cCont->WordRemaining)
        {
            DEBUG_I2C_SEND(1, ("Writing Tx from int\n"));
            
            // Get TFifo empty count
            FreeSlots = NV_DRF_VAL(I2C, FIFO_STATUS, TX_FIFO_EMPTY_CNT, FifoStatus);
            MaxWordToWrite = NV_MIN(hRmI2cCont->WordRemaining, FreeSlots);
            for (WordCount = 0; WordCount < MaxWordToWrite; ++WordCount)
            {
                // Write data into the I2C TX pkt FIFO Register
                I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, 
                        hRmI2cCont->pDataBuffer[hRmI2cCont->WordTransferred]);
                hRmI2cCont->WordTransferred++;
            }
            hRmI2cCont->WordRemaining -= MaxWordToWrite;

            if (hRmI2cCont->WordRemaining == 0)
            {
                if(hRmI2cCont->IsCurrentTransferNoStop)
                {
                    hRmI2cCont->FinalInterrupt = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    TFIFO_DATA_REQ_INT_EN, ENABLE, hRmI2cCont->FinalInterrupt);
                }
                else
                {
                    hRmI2cCont->IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                              TFIFO_DATA_REQ_INT_EN, DISABLE, hRmI2cCont->IntMaskReg);
                
                    hRmI2cCont->IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, hRmI2cCont->IntMaskReg);
                                    
                    
                    hRmI2cCont->FinalInterrupt = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, hRmI2cCont->FinalInterrupt);
                    WriteIntMaksReg(hRmI2cCont);
                }                
            }
        }
        goto IntDone;
    }
FinalIntDone:    
    if(IsFinalIntGot)
    {
        // Clear interrupt mask register, and DVC interrupt status for DVC I2C.
        // Note that h/w clean up and use of hRmI2cCont (shared with transaction
        // API thread) must be completed in ISR before semaphore is signaled.
        I2C_REGW (hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
        hRmI2cCont->IsTransferCompleted = NV_TRUE;
        if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
            DVC_REGW(hRmI2cCont, STATUS_REG,
                     NV_DRF_NUM(DVC, STATUS_REG, I2C_DONE_INTR, 1));
        NvOsSemaphoreSignal(hRmI2cCont->I2cSyncSemaphore);
        goto Done;
    }

    NvOsDebugPrintf("AP20 I2c Isr got unwanted interrupt IntStatus 0x%08x\n", 
                                                    hRmI2cCont->ControllerStatus);
    NV_ASSERT(0);
    
IntDone:    
    if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
        DVC_REGW(hRmI2cCont, STATUS_REG, NV_DRF_NUM(DVC, STATUS_REG, I2C_DONE_INTR, 1));
Done:        
    NvRmInterruptDone(hRmI2cCont->I2CInterruptHandle);
}

#if USE_POLLING_METHOD
static NvError WaitForTransactionCompletesPolling(
    NvRmI2cControllerHandle hRmI2cCont, 
    NvU32 Timeout)
{
    NvU32 RemainingTime;
    RemainingTime = (Timeout == NV_WAIT_INFINITE)?Timeout:Timeout*1000;
    do {
        NvOsWaitUS(I2C_POLLING_TIMEOUT_STEP_USEC);

        // Read the Interrupt status register & PKT_STATUS
        hRmI2cCont->ControllerStatus = I2C_REGR(hRmI2cCont, INTERRUPT_STATUS_REGISTER);
        if (hRmI2cCont->ControllerStatus & hRmI2cCont->IntMaskReg)
        {
            I2cIsr(hRmI2cCont);
            if (hRmI2cCont->IsTransferCompleted)
                break;
        }

        if (Timeout != NV_WAIT_INFINITE)
            RemainingTime = (RemainingTime > I2C_POLLING_TIMEOUT_STEP_USEC)? 
                                    (RemainingTime - I2C_POLLING_TIMEOUT_STEP_USEC): 0;
    } while(RemainingTime);

    if (!RemainingTime)
        return NvError_Timeout;

    return NvSuccess;
}
#endif

static NvError WaitForTransactionCompletes(
    NvRmI2cControllerHandle hRmI2cCont, 
    NvU32 Timeout)
{
    NvError Error;

#if USE_POLLING_METHOD
    hRmI2cCont->IsTransferCompleted = NV_FALSE;
    Error = WaitForTransactionCompletesPolling(hRmI2cCont, hRmI2cCont->timeout);
#else
    // Wait for the  Transfer completes
    Error = NvOsSemaphoreWaitTimeout(hRmI2cCont->I2cSyncSemaphore, Timeout);
#endif
    return Error;
}


static NvError 
DoOneReceiveTransaction(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU32 PacketId,
    NvU8*  pBuffer,
    NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvU32 WordsToRead = 0;
    NvU32 TFifoEmptyCount = 0;
    NvError Error = NvSuccess;
    NvU32 PacketHeader1;
    NvU32 PacketHeader2;
    NvU32 PacketHeader3;
    NvU32 IntMaskReg;
    NvRmDmaModuleID DmaModuleId = NvRmDmaModuleID_I2c;
    NvU32 BytesRead;


    hRmI2cCont->WordTransferred = 0;
    hRmI2cCont->WordRemaining = 0;
    hRmI2cCont->TransCountFromLastDmaUsage++;

    GetPacketHeaders(hRmI2cCont, pTransaction, PacketId, &PacketHeader1, 
                                            &PacketHeader2, &PacketHeader3);
    
    DoTxFifoEmpty(hRmI2cCont, &TFifoEmptyCount);
    
    // Enable all possible i2c controller error.
    IntMaskReg  = I2C_ERRORS_INTERRUPT_MASK;

    if (!hRmI2cCont->IsCurrentTransferNoAck)
        IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    NOACK_INT_EN, ENABLE, IntMaskReg);

    hRmI2cCont->FinalInterrupt = IntMaskReg;

    // Words to read 
    WordsToRead = BYTES_TO_WORD(pTransaction->NumBytes);
    hRmI2cCont->WordTransferred = 0;
    hRmI2cCont->WordRemaining = WordsToRead;

    hRmI2cCont->IsTransferCompleted = NV_FALSE;

    // If requested size is more than cpu transaction thresold then use dma.
    if ((hRmI2cCont->DmaBufferSize) && 
            (hRmI2cCont->WordRemaining > I2C_MAX_WORD_TO_USE_CPU))
    {
        if (!hRmI2cCont->IsApbDmaAllocated)
        {
            if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
                DmaModuleId =NvRmDmaModuleID_Dvc;
                
            Error = NvRmDmaAllocate(hRmI2cCont->hRmDevice, &hRmI2cCont->hRmDma,
                             NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                             hRmI2cCont->Instance);
            if (!Error)
                hRmI2cCont->IsApbDmaAllocated = NV_TRUE;
            Error = NvSuccess;
        }
        if (!hRmI2cCont->IsApbDmaAllocated)
            goto CpuBasedReading;
        
        hRmI2cCont->IsUsingApbDma = NV_TRUE;
        hRmI2cCont->TransCountFromLastDmaUsage = 0;
        hRmI2cCont->RxDmaReq.TransferSize = hRmI2cCont->WordRemaining << 2;
        SetRxFifoTriggerLevel(hRmI2cCont, 1);
        if (hRmI2cCont->IsCurrentTransferNoStop)
        {
#if USE_POLLING_METHOD
            goto CpuBasedReading;
#else
            Error = NvRmDmaStartDmaTransfer(hRmI2cCont->hRmDma, &hRmI2cCont->RxDmaReq,
                    NvRmDmaDirection_Forward, 0, hRmI2cCont->I2cSyncSemaphore);
#endif                    
        }
        else
        {
            Error = NvRmDmaStartDmaTransfer(hRmI2cCont->hRmDma, &hRmI2cCont->RxDmaReq,
                    NvRmDmaDirection_Forward, 0, NULL);
        }
        if (!Error)
        {
            hRmI2cCont->ControllerStatus = 0;
            I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader1);
            I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader2);
            
            // Write I2C specific header
            I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader3);
        
            if (!hRmI2cCont->IsCurrentTransferNoStop)
            {
                IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, IntMaskReg);
                hRmI2cCont->FinalInterrupt = IntMaskReg;
            }
            hRmI2cCont->IntMaskReg = IntMaskReg;
            WriteIntMaksReg(hRmI2cCont);
            goto WaitForCompletion;
        }
        Error = NvSuccess;
    }

CpuBasedReading:    
    hRmI2cCont->IsUsingApbDma = NV_FALSE;

    // Enable the Rx trigger level interrupt if the word to read is more than
    // fifo depth or no stop transfer is selected
    if ((hRmI2cCont->WordRemaining > I2C_FIFO_DEPTH) ||
                                hRmI2cCont->IsCurrentTransferNoStop)
    {
        SetRxFifoTriggerLevel(hRmI2cCont, hRmI2cCont->WordRemaining);
        IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    RFIFO_DATA_REQ_INT_EN, ENABLE, IntMaskReg);
    }
    else
    {
        IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, IntMaskReg);
        hRmI2cCont->FinalInterrupt = IntMaskReg;
    }
    
    hRmI2cCont->IntMaskReg = IntMaskReg;

    WriteIntMaksReg(hRmI2cCont);

    //Write Generic Header1 & 2
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader1);
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader2);
    
    // Write I2C specific header
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader3);

WaitForCompletion:
    Error = WaitForTransactionCompletes(hRmI2cCont, hRmI2cCont->timeout);
    if (Error == NvSuccess)
    {
        hRmI2cCont->I2cTransferStatus = NvError_I2cReadFailed;
        if (hRmI2cCont->ControllerStatus & I2C_TRANSACTION_STATUS_ERRORS)
        {
            if (hRmI2cCont->ControllerStatus & I2C_ARBITRATION_LOST_ERRORS)
                hRmI2cCont->I2cTransferStatus = NvError_I2cArbitrationFailed;
            else
                hRmI2cCont->I2cTransferStatus = NvError_I2cInternalError;

            goto ReadExitWithReset;
        }
        else
        {
            if (!hRmI2cCont->IsCurrentTransferNoAck)
            {
                if(hRmI2cCont->ControllerStatus & 
                    NV_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, NOACK_INT_EN, ENABLE))
                {    
                    hRmI2cCont->I2cTransferStatus = NvError_I2cDeviceNotFound;
                    goto ReadExitWithReset;
                }
            }
            // Memcopy fifo back to actual buffer given by client
            if (hRmI2cCont->IsUsingApbDma)
            {
                BytesRead = NV_MIN(pTransaction->NumBytes, 
                                        hRmI2cCont->RxDmaReq.TransferSize);
                NvOsMemcpy(pBuffer, (NvU8* )hRmI2cCont->pDmaBuffer, BytesRead);
            }
            else
            {
                BytesRead = NV_MIN(pTransaction->NumBytes, 
                                        (4*hRmI2cCont->WordTransferred));
                if (BytesRead != pTransaction->NumBytes)
                {
                    hRmI2cCont->I2cTransferStatus = NvError_I2cReadFailed;
                    goto ReadExitWithReset;
                }
                NvOsMemcpy(pBuffer, (NvU8* )hRmI2cCont->pDataBuffer, BytesRead);
            }

            if (pBytesTransferred != NULL)
                *pBytesTransferred = BytesRead;

            
            hRmI2cCont->I2cTransferStatus = NvSuccess;
            goto ReadExit;
        }
    }
    else if (Error == NvError_Timeout)
    {
        DEBUG_I2C_READ(1, ("Read Timeout Error \n"));
        hRmI2cCont->I2cTransferStatus = NvError_Timeout;
    }

ReadExitWithReset:
    // If we reach here then there is something wrong in transfer, reset the module.
    if (hRmI2cCont->IsUsingApbDma)
        NvRmDmaAbort(hRmI2cCont->hRmDma);

    // If there is NACK error, then there is possibilty that i2c controller is 
    // still busy to send the stop signal.
    // Wait for 2x of i2c clock period is recommended, waiting for 1 ms to use 
    // the NvOsMsSleep api.
    NvOsSleepMS(1);
        
    NvRmModuleReset(hRmI2cCont->hRmDevice, 
                NVRM_MODULE_ID(hRmI2cCont->ModuleId, hRmI2cCont->Instance));
    RESET_SEMA_COUNT(hRmI2cCont->I2cSyncSemaphore);
ReadExit:    
    DEBUG_I2C_READ(1, ("Read Transfer Status 0x%08x \n", hRmI2cCont->I2cTransferStatus));
    
    // Time to free dma??
    if ((hRmI2cCont->IsApbDmaAllocated) && 
        (hRmI2cCont->TransCountFromLastDmaUsage > HOLDING_DMA_TRANSACTION_COUNT))
    {
        NvRmDmaFree(hRmI2cCont->hRmDma);
        hRmI2cCont->hRmDma = NULL;
        hRmI2cCont->IsApbDmaAllocated = NV_FALSE;
    }
    return hRmI2cCont->I2cTransferStatus;
}


static NvError 
DoOneSendTransaction(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU32 PacketId,
    NvU8*  pBuffer,
    NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvU32 WordsToSend = 0;
    NvU32 TFifoEmptyCount = 0;
    NvError Error = NvSuccess;
    NvU32 PacketHeader1;
    NvU32 PacketHeader2;
    NvU32 PacketHeader3;
    NvU32 IntMaskReg;
    NvU32 WordCount;
    NvRmDmaModuleID DmaModuleId = NvRmDmaModuleID_I2c;

    hRmI2cCont->WordTransferred = 0;
    hRmI2cCont->WordRemaining = 0;
    hRmI2cCont->TransCountFromLastDmaUsage++;

    GetPacketHeaders(hRmI2cCont, pTransaction, PacketId, &PacketHeader1, 
                                            &PacketHeader2, &PacketHeader3);
    DoTxFifoEmpty(hRmI2cCont, &TFifoEmptyCount);
    
    // Enable all possible i2c controller error.
    IntMaskReg  = I2C_ERRORS_INTERRUPT_MASK;

    if (!hRmI2cCont->IsCurrentTransferNoAck)
        IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    NOACK_INT_EN, ENABLE, IntMaskReg);

    hRmI2cCont->FinalInterrupt = IntMaskReg;

    // Words to write 
    WordsToSend = BYTES_TO_WORD(pTransaction->NumBytes);
    hRmI2cCont->WordTransferred = 0;
    hRmI2cCont->WordRemaining = WordsToSend;
    hRmI2cCont->IsTransferCompleted = NV_FALSE;

    if ((hRmI2cCont->DmaBufferSize) && 
            (hRmI2cCont->WordRemaining > I2C_MAX_WORD_TO_USE_CPU))
    {
        if (!hRmI2cCont->IsApbDmaAllocated)
        {
            if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
                DmaModuleId =NvRmDmaModuleID_Dvc;
                
            Error = NvRmDmaAllocate(hRmI2cCont->hRmDevice, &hRmI2cCont->hRmDma,
                             NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                             hRmI2cCont->Instance);
            if (!Error)
                hRmI2cCont->IsApbDmaAllocated = NV_TRUE;
            Error = NvSuccess;
        }
        if (!hRmI2cCont->IsApbDmaAllocated)
            goto CpuBasedWriting;
    
        hRmI2cCont->IsUsingApbDma = NV_TRUE;
        hRmI2cCont->TransCountFromLastDmaUsage = 0;
        hRmI2cCont->pDmaBuffer[0] = PacketHeader1;
        hRmI2cCont->pDmaBuffer[1] = PacketHeader2;
        hRmI2cCont->pDmaBuffer[2] = PacketHeader3;
        hRmI2cCont->TxDmaReq.TransferSize = (hRmI2cCont->WordRemaining + 3) << 2;
        NvOsMemcpy(hRmI2cCont->pDmaBuffer + 3, (void *)pBuffer, pTransaction->NumBytes);
        SetTxFifoTriggerLevel(hRmI2cCont, 8);
        Error = NvRmDmaStartDmaTransfer(hRmI2cCont->hRmDma, &hRmI2cCont->TxDmaReq,
                NvRmDmaDirection_Forward, 0, NULL);
        if (!Error)
        {
            hRmI2cCont->WordRemaining = 0;
            if (hRmI2cCont->IsCurrentTransferNoStop)
            {
                IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                        TFIFO_DATA_REQ_INT_EN, ENABLE, IntMaskReg);
                hRmI2cCont->FinalInterrupt = IntMaskReg;                        
            }
            else
            {
                IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, IntMaskReg);
                hRmI2cCont->FinalInterrupt = IntMaskReg;
            }
            goto WaitForCompletion;
        }
        
//        NvOsDebugPrintf("Send Using Dma\n");
        Error = NvSuccess;
    }

CpuBasedWriting:
    hRmI2cCont->IsUsingApbDma = NV_FALSE;

    //Write Generic Header1 & 2
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader1);
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader2);
    
    // Write I2C specific header
    I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, PacketHeader3);
    
    TFifoEmptyCount -= 3;
    
    if (hRmI2cCont->WordRemaining)
    {
        NvOsMemcpy(hRmI2cCont->pDataBuffer, (void *)pBuffer, pTransaction->NumBytes);
        
        WordsToSend = NV_MIN(hRmI2cCont->WordRemaining, TFifoEmptyCount);
        for (WordCount = 0; WordCount < WordsToSend; WordCount++)
        {
            // Write data into the I2C TX pkt FIFO Register
            I2C_REGW(hRmI2cCont, I2C_TX_PACKET_FIFO, 
                        hRmI2cCont->pDataBuffer[hRmI2cCont->WordTransferred]);
            hRmI2cCont->WordTransferred++;
        }
        hRmI2cCont->WordRemaining -= WordsToSend;

        if (hRmI2cCont->WordRemaining == 0)
        {
            if (hRmI2cCont->IsCurrentTransferNoStop)
            {
                SetTxFifoTriggerLevel(hRmI2cCont, 8);
                IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                        TFIFO_DATA_REQ_INT_EN, ENABLE, IntMaskReg);
                hRmI2cCont->FinalInterrupt = IntMaskReg;                        
            }
            else
            {
                IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                ALL_PACKETS_XFER_COMPLETE_INT_EN, ENABLE, IntMaskReg);
                hRmI2cCont->FinalInterrupt = IntMaskReg;
            }
        }
        else
        {
            SetTxFifoTriggerLevel(hRmI2cCont, 8);
            IntMaskReg = NV_FLD_SET_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, 
                                    TFIFO_DATA_REQ_INT_EN, ENABLE, IntMaskReg);
        }
    }

WaitForCompletion:
    hRmI2cCont->IntMaskReg = IntMaskReg;
    WriteIntMaksReg(hRmI2cCont);
    Error = WaitForTransactionCompletes(hRmI2cCont, hRmI2cCont->timeout);
    if (Error == NvSuccess)
    {
        hRmI2cCont->I2cTransferStatus = NvError_I2cWriteFailed;
        if (hRmI2cCont->ControllerStatus & I2C_TRANSACTION_STATUS_ERRORS)
        {
            if (hRmI2cCont->ControllerStatus & I2C_ARBITRATION_LOST_ERRORS)
                hRmI2cCont->I2cTransferStatus = NvError_I2cArbitrationFailed;
            else
                hRmI2cCont->I2cTransferStatus = NvError_I2cInternalError;
            goto WriteExitWithReset;
        }   
        else
        {
            if (!hRmI2cCont->IsCurrentTransferNoAck)
            {
                if(hRmI2cCont->ControllerStatus & 
                    NV_DRF_DEF(I2C, INTERRUPT_MASK_REGISTER, NOACK_INT_EN, ENABLE))
                {    
                    hRmI2cCont->I2cTransferStatus = NvError_I2cDeviceNotFound;
                    goto WriteExitWithReset;
                }
            }   
            if (pBytesTransferred != NULL)
                *pBytesTransferred = pTransaction->NumBytes;
            hRmI2cCont->I2cTransferStatus = NvSuccess;
            goto WriteExit;
        }
    }
    else if (Error == NvError_Timeout)
    {
        DEBUG_I2C_SEND(1, ("SEND Timeout Error \n"));
        hRmI2cCont->I2cTransferStatus = NvError_Timeout;
    }
    
WriteExitWithReset:
    if (hRmI2cCont->IsUsingApbDma)
        NvRmDmaAbort(hRmI2cCont->hRmDma);

    // If there is NACK error, then there is possibilty that i2c controller is 
    // still busy to send the stop signal.
    // Wait for 2x of i2c clock period is recommended, waiting for 1 ms to use 
    // the NvOsMsSleep api.
    NvOsSleepMS(1);
    
    // If we reach here then there is something wrong in transfer, reset the module.
    NvRmModuleReset(hRmI2cCont->hRmDevice, 
                NVRM_MODULE_ID(hRmI2cCont->ModuleId, hRmI2cCont->Instance));
    RESET_SEMA_COUNT(hRmI2cCont->I2cSyncSemaphore);
WriteExit:    
    DEBUG_I2C_SEND(1, ("Send Transfer Status 0x%08x \n", hRmI2cCont->I2cTransferStatus));

    // Time to free dma??
    if ((hRmI2cCont->IsApbDmaAllocated) && 
        (hRmI2cCont->TransCountFromLastDmaUsage > HOLDING_DMA_TRANSACTION_COUNT))
    {
        NvRmDmaFree(hRmI2cCont->hRmDma);
        hRmI2cCont->hRmDma = NULL;
        hRmI2cCont->IsApbDmaAllocated = NV_FALSE;
    }
    
    return hRmI2cCont->I2cTransferStatus;
}


static NvError 
DoMultiReceiveTransaction(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU32 *pPacketId,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvRmI2cTransactionInfo Transaction;
    NvU32 BytesTransferredYet = 0;
    NvU32 TotalBytesRequested;
    NvU8 *pReadBuffer = pBuffer;
    NvError Error = NvSuccess;
    NvU32 CurrBytesRequested;
    NvU32 PacketId;
    NvU32 BytesTransferred = 0;

    Transaction.Is10BitAddress = pTransaction->Is10BitAddress;
    Transaction.Address = pTransaction->Address;
    TotalBytesRequested = pTransaction->NumBytes;
    PacketId = *pPacketId;
    while (TotalBytesRequested)
    {
        Transaction.Flags = pTransaction->Flags;
        CurrBytesRequested = TotalBytesRequested;
        if (TotalBytesRequested > MAX_I2C_ONE_TRANSACTION_SIZE)
        {
            Transaction.Flags |= NVRM_I2C_NOSTOP;
            CurrBytesRequested = MAX_I2C_ONE_TRANSACTION_SIZE;
        }
        Transaction.NumBytes = CurrBytesRequested;
        Error = DoOneReceiveTransaction(hRmI2cCont, PacketId, pReadBuffer, 
                                &Transaction, &BytesTransferred);
        if (Error)
            break;
        BytesTransferredYet += CurrBytesRequested;  
        pReadBuffer += CurrBytesRequested;
        TotalBytesRequested -= CurrBytesRequested;
        PacketId++;
        I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
    }
    *pPacketId = PacketId;
    *pBytesTransferred = BytesTransferredYet;
    return Error;
}

static NvError
DoMultiSendTransaction(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU32 *pPacketId,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)

{
    NvRmI2cTransactionInfo Transaction;
    NvU32 BytesTransferredYet = 0;
    NvU32 TotalBytesRequested;
    NvU8 *pReadBuffer = pBuffer;
    NvError Error = NvSuccess;
    NvU32 CurrBytesRequested;
    NvU32 PacketId;
    NvU32 BytesTransferred = 0;
    
    Transaction.Is10BitAddress = pTransaction->Is10BitAddress;
    Transaction.Address = pTransaction->Address;
    TotalBytesRequested = pTransaction->NumBytes;
    PacketId = *pPacketId;
    while (TotalBytesRequested)
    {
        Transaction.Flags = pTransaction->Flags;
        CurrBytesRequested = TotalBytesRequested;
        if (TotalBytesRequested > MAX_I2C_ONE_TRANSACTION_SIZE)
        {
            Transaction.Flags |= NVRM_I2C_NOSTOP;
            CurrBytesRequested = MAX_I2C_ONE_TRANSACTION_SIZE;
        }
        Transaction.NumBytes = CurrBytesRequested;
        Error = DoOneSendTransaction(hRmI2cCont, PacketId, pReadBuffer, 
                                &Transaction, &BytesTransferred);
        if (Error)
            break;
        BytesTransferredYet += CurrBytesRequested;  
        pReadBuffer += CurrBytesRequested;
        TotalBytesRequested -= CurrBytesRequested;
        PacketId++;
        I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
    }
    *pPacketId = PacketId;
    *pBytesTransferred = BytesTransferred;
    return Error;
}


static NvError 
AP20RmI2cReceive(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvError Error = NvSuccess;
    NvU32 PacketId = 1;
    
    NV_ASSERT(pBuffer);
    NV_ASSERT(pTransaction->NumBytes > 0);

    DEBUG_I2C_TRACE(1, ("AP20RmI2cReceive()++ 0x%08x and add 0x%02x\n", pTransaction->NumBytes, Transaction.Address));

    if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
        DoDvcI2cControlInitialization(hRmI2cCont);
        
    // Clear interrupt mask register to avoid any false interrupts.
    I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
    
    // Start the packet mode
    StartI2cPacketMode(hRmI2cCont);

    Error = DoMultiReceiveTransaction(hRmI2cCont, &PacketId, pBuffer, 
                                      pTransaction, pBytesTransferred);
    DEBUG_I2C_TRACE(1, ("AP20RmI2cReceive()-- 0x%08x\n", Error));
    return Error;
}


static NvError
AP20RmI2cSend(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvError Error = NvSuccess;
    NvU32 PacketId = 1;

    NV_ASSERT(pBuffer);
    NV_ASSERT(pTransaction->NumBytes > 0);

    DEBUG_I2C_TRACE(1, ("AP20RmI2cSend()++ 0x%08x and 0x%02x\n", pTransaction->NumBytes, Transaction.Address));

    if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
        DoDvcI2cControlInitialization(hRmI2cCont);

    // Clear interrupt mask register to avoid any false interrupts.
    I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
    
    // Start the packet mode
    StartI2cPacketMode(hRmI2cCont);

    Error = DoMultiSendTransaction(hRmI2cCont, &PacketId, pBuffer, 
                                   pTransaction, pBytesTransferred);
    DEBUG_I2C_TRACE(1, ("AP20RmI2cSend()-- 0x%08x\n", Error));
    return Error;
}

static NvError
AP20RmI2cRepeatStartTransaction(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU8* pBuffer,
    NvRmI2cTransactionInfo *pTransactions,
    NvU32 NoOfTransations)
{
    NvError Error = NvSuccess;
    NvU8 *pReqBuffer = pBuffer;
    NvU32 BytesSend;
    NvU32 BytesRecvd;
    NvU32 PacketId;
    NvU32 TransCount;
    
    NV_ASSERT(pBuffer);
    NV_ASSERT(pTransactions);
    NV_ASSERT(pBuffer);
    
    DEBUG_I2C_TRACE(1, ("AP20RmI2cRepeatStartTransaction()++ 0x%08x and 0x%02x\n", NoOfTransations, pTransactions[0].Address));

    if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
    {    
        DoDvcI2cControlInitialization(hRmI2cCont);
        UseDvcI2cNewSlave(hRmI2cCont);
    }

    // Clear interrupt mask register to avoid any false interrupts.
    I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
    
    // Start the packet mode
    StartI2cPacketMode(hRmI2cCont);

    PacketId = 1;
    for (TransCount = 0; TransCount < NoOfTransations; TransCount++)
    {
        if (pTransactions[TransCount].Flags & NVRM_I2C_WRITE) 
        {
            Error = DoMultiSendTransaction(hRmI2cCont, &PacketId, 
                        pReqBuffer, &pTransactions[TransCount], &BytesSend);
        }
        else
        {
            Error = DoMultiReceiveTransaction(hRmI2cCont, &PacketId, 
                        pReqBuffer, &pTransactions[TransCount], &BytesRecvd);
        }
        if (Error)
        {
            DEBUG_I2C_TRACE(1, ("AP20RmI2cRepeatStartTransaction()-- 0x%08x at Transaction %d \n", Error, TransCount));
            break;
        }    
        pReqBuffer += pTransactions[TransCount].NumBytes;
        
        I2C_REGW(hRmI2cCont, INTERRUPT_MASK_REGISTER, 0);
        PacketId++;
    }
    DEBUG_I2C_TRACE(1, ("AP20RmI2cRepeatStartTransaction()-- 0x%08x at Transaction %d \n", Error, TransCount));
    return Error;
}

static NvBool AP20RmI2cGetGpioPins(
    NvRmI2cControllerHandle hRmI2cCont,
    NvU32 I2cPinMap,
    NvU32 *pScl,
    NvU32 *pSda)
{
    NvU32 SclPin = 0;
    NvU32 SdaPin = 0;
    NvU32 SclPort = 0;
    NvU32 SdaPort = 0;
    NvBool Result = NV_TRUE;

    NV_ASSERT((pScl!=NULL) && (pSda!=NULL));

    if (hRmI2cCont->ModuleId == NvRmModuleID_I2c)
    {
        switch ((hRmI2cCont->Instance<<4) | I2cPinMap)
        {
            case ((0<<4) | 1):
                SclPort = 'c' - 'a';
                SdaPort = 'c' - 'a';
                SclPin = 4;
                SdaPin = 5;
                break;
            case ((0<<4) | 2):
                SclPort = 'k' - 'a';
                SdaPort = 'k' - 'a';
                SclPin = 5;
                SdaPin = 6;
                break;
            case ((0<<4) | 3):
                SclPort = 'w' - 'a';
                SdaPort = 'w' - 'a';
                SclPin = 2;
                SdaPin = 3;
                break;
                /*  NOTE:  The pins used by pin map 1 for instance 1 are not
                 *  connected to a GPIO controller (DDC pins), so the software
                 *  fallback is not supported for them.  */
            case ((1<<4) | 2):
                SclPort = 't' - 'a';
                SdaPort = 't' - 'a';
                SclPin = 5;
                SdaPin = 6;
                break;
            case ((2<<4) | 1):
                //  Port 'BB'
                SclPort = 'z' - 'a' + 2;
                SdaPort = 'z' - 'a' + 2;
                SclPin = 2;
                SdaPin = 3;
                break;
            default:
                Result = NV_FALSE;
        }
    }
    else if ((hRmI2cCont->ModuleId == NvRmModuleID_Dvc) &&
                 (hRmI2cCont->Instance == 0) &&
                     (I2cPinMap == NvOdmI2cPmuPinMap_Config1))
    {
        SclPin = 6;
        SdaPin = 7;
        SclPort = 'z' - 'a';
        SdaPort = 'z' - 'a';
    }
    else
        Result = NV_FALSE;

    *pScl = (SclPin | (SclPort<<16));
    *pSda = (SdaPin | (SdaPort<<16));
    return Result;
}

static void AP20RmI2cClose(NvRmI2cControllerHandle hRmI2cCont)
{

    if (hRmI2cCont->I2cSyncSemaphore)
    {
#if !USE_POLLING_METHOD
        NvRmInterruptUnregister(hRmI2cCont->hRmDevice, hRmI2cCont->I2CInterruptHandle);
#endif
        NvOsSemaphoreDestroy(hRmI2cCont->I2cSyncSemaphore);
        hRmI2cCont->I2cSyncSemaphore = NULL;
        hRmI2cCont->I2CInterruptHandle = NULL;
    }

    if (hRmI2cCont->pCpuBuffer)
    {
        NvOsFree(hRmI2cCont->pCpuBuffer);
        hRmI2cCont->pCpuBuffer = NULL;
        hRmI2cCont->pDataBuffer = NULL;
    }

    if (hRmI2cCont->hRmDma)
    {
        NvRmDmaAbort(hRmI2cCont->hRmDma);
        NvRmDmaFree(hRmI2cCont->hRmDma);
    }
    
    DestroyDmaTransferBuffer(hRmI2cCont->hRmMemory, hRmI2cCont->pDmaBuffer,
                hRmI2cCont->DmaBufferSize);

    hRmI2cCont->hRmDma = NULL;
    hRmI2cCont->hRmMemory = NULL;
    hRmI2cCont->DmaBuffPhysAdd = 0;
    hRmI2cCont->pDmaBuffer = NULL;

    hRmI2cCont->receive = 0;
    hRmI2cCont->send = 0;
    hRmI2cCont->repeatStart = 0;
    hRmI2cCont->close = 0;
    hRmI2cCont->GetGpioPins = 0;
}

NvError AP20RmI2cOpen(NvRmI2cControllerHandle hRmI2cCont)
{
    NvError Error = NvSuccess;
#if !USE_POLLING_METHOD
    NvU32 IrqList;
    NvOsInterruptHandler IntHandlers = I2cIsr;
#endif
    NvU32 RxFifoPhyAddress;
    NvU32 TxFifoPhyAddress;
    
    NV_ASSERT(hRmI2cCont);
    DEBUG_I2C_TRACE(1, ("AP20RmI2cOpen\n"));

    // Polulate the structures 
    hRmI2cCont->receive = AP20RmI2cReceive;
    hRmI2cCont->send = AP20RmI2cSend;
    hRmI2cCont->repeatStart = AP20RmI2cRepeatStartTransaction;
    hRmI2cCont->close = AP20RmI2cClose;
    hRmI2cCont->GetGpioPins =  AP20RmI2cGetGpioPins;
    hRmI2cCont->I2cRegisterOffset = I2C_I2C_CNFG_0;
    hRmI2cCont->ControllerId = hRmI2cCont->Instance;

    hRmI2cCont->hRmDma = NULL;
    hRmI2cCont->hRmMemory = NULL;
    hRmI2cCont->DmaBuffPhysAdd = 0;
    hRmI2cCont->pDmaBuffer = NULL;

    hRmI2cCont->pCpuBuffer = NULL;
    hRmI2cCont->pDataBuffer = NULL;
    hRmI2cCont->I2cSyncSemaphore = NULL;
    hRmI2cCont->I2CInterruptHandle = NULL;
    hRmI2cCont->TransCountFromLastDmaUsage = 0;

    TxFifoPhyAddress = hRmI2cCont->I2cRegisterOffset + I2C_I2C_TX_PACKET_FIFO_0;
    RxFifoPhyAddress = hRmI2cCont->I2cRegisterOffset + I2C_I2C_RX_FIFO_0;
    
    if (hRmI2cCont->ModuleId == NvRmModuleID_Dvc)
    {
        hRmI2cCont->I2cRegisterOffset = 0;
        hRmI2cCont->ControllerId = 3;
        RxFifoPhyAddress = DVC_I2C_RX_FIFO_0;
        TxFifoPhyAddress = DVC_I2C_TX_PACKET_FIFO_0;
    }    

    hRmI2cCont->IsApbDmaAllocated = NV_FALSE;
    hRmI2cCont->hRmDma = NULL;

    // Allocate the dma buffer 
    hRmI2cCont->DmaBufferSize = 0;
    Error = CreateDmaTransferBuffer(hRmI2cCont->hRmDevice, &hRmI2cCont->hRmMemory, 
            &hRmI2cCont->DmaBuffPhysAdd, (void **)&hRmI2cCont->pDmaBuffer,
            DEFAULT_I2C_DMA_BUFFER_SIZE);
    if (Error)
    {
        hRmI2cCont->hRmMemory = NULL;
        hRmI2cCont->DmaBuffPhysAdd = 0;
        hRmI2cCont->pDmaBuffer = NULL;
        Error = NvSuccess;
    }
    else
    {
        hRmI2cCont->DmaBufferSize = DEFAULT_I2C_DMA_BUFFER_SIZE;
        
        hRmI2cCont->RxDmaReq.SourceBufferPhyAddress= RxFifoPhyAddress;
        hRmI2cCont->RxDmaReq.DestinationBufferPhyAddress = hRmI2cCont->DmaBuffPhysAdd;
        hRmI2cCont->RxDmaReq.SourceAddressWrapSize = 4;
        hRmI2cCont->RxDmaReq.DestinationAddressWrapSize = 0;
        
        hRmI2cCont->TxDmaReq.SourceBufferPhyAddress= hRmI2cCont->DmaBuffPhysAdd;
        hRmI2cCont->TxDmaReq.DestinationBufferPhyAddress = TxFifoPhyAddress;
        hRmI2cCont->TxDmaReq.SourceAddressWrapSize = 0;
        hRmI2cCont->TxDmaReq.DestinationAddressWrapSize = 4;
    }

    if (!Error)
    {
        hRmI2cCont->pCpuBuffer = NvOsAlloc(DEFAULT_I2C_CPU_BUFFER_SIZE);
        if (!hRmI2cCont->pCpuBuffer)
            Error = NvError_InsufficientMemory;
    }            
    
    if (!Error)
        hRmI2cCont->pDataBuffer = hRmI2cCont->pCpuBuffer;
    
    // Create the sync semaphore for the interrupt synchrnoisation
    if (!Error)
        Error = NvOsSemaphoreCreate( &hRmI2cCont->I2cSyncSemaphore, 0);

#if !USE_POLLING_METHOD
    if (!Error)
    {
        IrqList = NvRmGetIrqForLogicalInterrupt(
                hRmI2cCont->hRmDevice, NVRM_MODULE_ID(hRmI2cCont->ModuleId, hRmI2cCont->Instance), 0);

        Error = NvRmInterruptRegister(hRmI2cCont->hRmDevice, 1, &IrqList, &IntHandlers, 
                hRmI2cCont, &hRmI2cCont->I2CInterruptHandle, NV_TRUE);
    }
#endif
    // Packet mode initialization
    hRmI2cCont->RsTransfer =  NV_FALSE;

    // If error then destroy all the allocation done here.
    if (Error)
        AP20RmI2cClose(hRmI2cCont);
    
    return Error;
}
