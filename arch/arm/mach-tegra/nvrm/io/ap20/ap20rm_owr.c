/*
 * Copyright (c) 2009 NVIDIA Corporation.
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
 * @brief <b>NVIDIA Driver Development Kit: OWR API</b>
 *
 * @b Description: Contains the NvRM OWR implementation. for Ap20 
 */

#include "nvrm_owr.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "ap20/arowr.h"
#include "nvrm_hardware_access.h"
#include "nvrm_power.h"
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pinmux.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"
#include "nvrm_owr_private.h"
#include "nvodm_query.h"
#include "nvrm_module.h"

// Enable the following flag for debug messages
#define ENABLE_OWR_DEBUG   0

#if ENABLE_OWR_DEBUG
#define OWR_PRINT(X)   NvOsDebugPrintf X
#else
#define OWR_PRINT(X)
#endif

// Enabling the following flag for enabling the polling in bit transfer mode
#define OWR_BIT_TRANSFER_POLLING_MODE   0

/* Timeout for transferring a bit in micro seconds */
#define BIT_TRASNFER_DONE_TIMEOUT_USEC  1000

/* Polling timeout steps for transferring a bit in micro seconds */
#define BIT_TRASNFER_DONE_STEP_TIMEOUT_USEC  10

/* Semaphore timeout for bit/byte transfers */
#define OWR_TRANSFER_TIMEOUT_MILLI_SEC  5000

/* OWR controller errors in byte transfer mode */
#define OWR_BYTE_TRANSFER_ERRORS  0x70F

/* OWR controller errors in bit transfer mode */
#define OWR_BIT_TRANSFER_ERRORS 0x1

/* OWR CRC size in bytes */
#define OWR_CRC_SIZE_BYTES  1

/* OWR ROM command size */
#define OWR_ROM_CMD_SIZE_BYTES  1

/* OWR MEM command size */
#define OWR_MEM_CMD_SIZE_BYTES  1

/* OWR fifo depth */
#define OWR_FIFO_DEPTH          32
/* OWR fifo word size */
#define OWR_FIFO_WORD_SIZE  4


/* default read data clock value */
#define OWR_DEFAULT_READ_DTA_CLK_VALUE    0x7
/* default read presence  clock value */
#define OWR_DEFAULT_PRESENCE_CLK_VALUE    0x50
/* Default OWR device memory offset size */
#define OWR_DEFAULT_OFFSET_SIZE_BYTES   2
/* Default OWR memory size */
#define OWR_DEFAULT_MEMORY_SIZE     0x80

/* Register access Macros */
#define OWR_REGR(OwrVirtualAddress, reg) \
        NV_READ32((OwrVirtualAddress) + ((OWR_##reg##_0)/4))

#define OWR_REGW(OwrVirtualAddress, reg, val) \
    do\
    {\
        NV_WRITE32((((OwrVirtualAddress) + ((OWR_##reg##_0)/4))), (val));\
    }while (0)

void PrivOwrEnableInterrupts(NvRmOwrController *pOwrInfo, NvU32 OwrIntStatus);
NvError PrivOwrSendCommand(NvRmOwrController *pOwrInfo, NvU32 Command);
NvError PrivOwrSendBit(NvRmOwrController *pOwrInfo, NvU32 Bit);

NvError
PrivOwrCheckBitTransferDone(
    NvRmOwrController* pOwrInfo,
    OwrIntrStatus status);

NvError
PrivOwrReadData(
    NvRmOwrController *pOwrInfo,
    NvU8* Buffer,
    NvU32 NoOfBytes);

NvError
PrivOwrReadDataBit(
    NvRmOwrController *pOwrInfo,
    NvU8* Buffer);

static NvError
PrivOwrCheckPresence(
        NvRmOwrController* pOwrInfo,
        NvU32 ReadDataClk,
        NvU32 PresenceClk);

static NvError
PrivOwrReadFifo(
    NvRmOwrController* pOwrInfo,
    NvU8*  pBuffer,
    NvRmOwrTransactionInfo Transaction,
    const NvOdmQueryOwrDeviceInfo* pOdmInfo,
    NvU32 NumBytes);


void PrivOwrEnableInterrupts(NvRmOwrController *pOwrInfo, NvU32 OwrIntStatus)
{
    // Write to the interrupt status register
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, INTR_MASK, OwrIntStatus);
}

NvError
PrivOwrCheckBitTransferDone(
    NvRmOwrController* pOwrInfo,
    OwrIntrStatus status)
{

#if OWR_BIT_TRANSFER_POLLING_MODE

    NvU32 timeout = 0;
    NvU32 val = 0;

    // Check for presence
    while(timeout < BIT_TRASNFER_DONE_TIMEOUT_USEC)
    {
        val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, INTR_STATUS);
        if (val & status)
        {
             // clear the bit transfer done status
            OWR_REGW(pOwrInfo->pOwrVirtualAddress, INTR_STATUS, val);
            break;
        }
        NvOsWaitUS(BIT_TRASNFER_DONE_STEP_TIMEOUT_USEC);
        timeout += BIT_TRASNFER_DONE_STEP_TIMEOUT_USEC;
    }

    if (timeout >= BIT_TRASNFER_DONE_TIMEOUT_USEC)
    {
        return NvError_Timeout;
    }

   return NvSuccess;
#else
        // wait for the read to complete
        status = NvOsSemaphoreWaitTimeout(pOwrInfo->OwrSyncSemaphore,
                    OWR_TRANSFER_TIMEOUT_MILLI_SEC);
        if (status == NvSuccess)
        {
            if (pOwrInfo->OwrTransferStatus & OWR_BIT_TRANSFER_ERRORS)
            {
                status = NvError_OwrBitTransferFailed;
                NvRmModuleReset(pOwrInfo->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
                OWR_PRINT(("RM_OWR Bit mode error[0x%x]\n",
                                            pOwrInfo->OwrTransferStatus));
            }
            else if(!pOwrInfo->OwrTransferStatus)
            {
                status = NvError_OwrBitTransferFailed;
                NvRmModuleReset(pOwrInfo->hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
                OWR_PRINT(("RM_OWR bit mode spurious interrupt [0x%x]\n",
                                    pOwrInfo->OwrTransferStatus));
                NV_ASSERT(!"RM_OWR spurious interrupt in Bit transfer mode\n");
            }
        }
         pOwrInfo->OwrTransferStatus = 0;
        return status;
#endif
}

NvError PrivOwrSendCommand(NvRmOwrController *pOwrInfo, NvU32 Command)
{
    NvU32 val = 0;
    NvU32 data = Command;
    NvError status = NvError_Timeout;
    NvU32 i =0;
    NvU32 ControlReg = 0;

    val =
        (NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, 0x7) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, 0x50) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BIT_TRANSFER_MODE));

    for (i = 0; i < OWR_NO_OF_BITS_PER_BYTE; i++)
    {

        if (data & 0x1)
        {
            ControlReg =
                val | (NV_DRF_DEF(OWR, CONTROL, WR1_BIT, TRANSFER_ONE));
        }
        else
        {
           ControlReg =
            val | (NV_DRF_DEF(OWR, CONTROL, WR0_BIT, TRANSFER_ZERO));
        }
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, ControlReg);

        status = PrivOwrCheckBitTransferDone(pOwrInfo,
                                    OwrIntrStatus_BitTransferDoneIntEnable);
        if (status != NvSuccess)
        {
            return status;
        }

        data = (data >> 1);
    }

    return NvSuccess;
}

NvError PrivOwrSendBit(NvRmOwrController *pOwrInfo, NvU32 Bit)
{
    NvU32 val = 0;
    NvU32 data = Bit;
    NvError status = NvError_Timeout;
    NvU32 ControlReg = 0;

    val =
        (NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, 0x7) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, 0x50) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BIT_TRANSFER_MODE));

    if (data & 0x1)
    {
        ControlReg =
            val | (NV_DRF_DEF(OWR, CONTROL, WR1_BIT, TRANSFER_ONE));
    }
    else
    {
        ControlReg =
            val | (NV_DRF_DEF(OWR, CONTROL, WR0_BIT, TRANSFER_ZERO));
    }
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, ControlReg);

    status = PrivOwrCheckBitTransferDone(pOwrInfo,
                                    OwrIntrStatus_BitTransferDoneIntEnable);
    if (status != NvSuccess)
    {
        return status;
    }

    return NvSuccess;
}

NvError
PrivOwrReadData(
    NvRmOwrController *pOwrInfo,
    NvU8* Buffer,
    NvU32 NoOfBytes)
{
    NvU32 ControlReg = 0;
    NvError status = NvError_Timeout;
    NvU8* pBuf = Buffer;
    NvU32 val = 0;
    NvU32 i =0;
    NvU32 j =0;

    NvOsMemset(pBuf, 0, NoOfBytes);

    ControlReg =
        NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, 0x7) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, 0x50) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BIT_TRANSFER_MODE) |
        NV_DRF_DEF(OWR, CONTROL, RD_BIT, TRANSFER_READ_SLOT);

    for (i = 0; i < NoOfBytes; i++)
    {
        for (j = 0; j < OWR_NO_OF_BITS_PER_BYTE; j++)
        {
            OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, ControlReg);
            status = PrivOwrCheckBitTransferDone(pOwrInfo,
                                        OwrIntrStatus_BitTransferDoneIntEnable);
            if (status != NvSuccess)
            {
                return status;
            }
            val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, STATUS);
            val = NV_DRF_VAL(OWR, STATUS, READ_SAMPLED_BIT, val);
            *pBuf |= (val << j);
        }
        pBuf++;
    }
    return NvSuccess;
}

NvError
PrivOwrReadDataBit(
    NvRmOwrController *pOwrInfo,
    NvU8* Buffer)
{
    NvU32 ControlReg = 0;
    NvError status = NvError_Timeout;
    NvU32 val = 0;

    ControlReg =
        NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, 0x7) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, 0x50) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BIT_TRANSFER_MODE) |
        NV_DRF_DEF(OWR, CONTROL, RD_BIT, TRANSFER_READ_SLOT);

    OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, ControlReg);
    status = PrivOwrCheckBitTransferDone(pOwrInfo,
        OwrIntrStatus_BitTransferDoneIntEnable);
    if (status != NvSuccess)
    {
        return status;
    }
    val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, STATUS);
    val = NV_DRF_VAL(OWR, STATUS, READ_SAMPLED_BIT, val);
    *Buffer = val;
    return NvSuccess;
}

static NvError
PrivOwrReadFifo(
    NvRmOwrController* pOwrInfo,
    NvU8*  pBuffer,
    NvRmOwrTransactionInfo Transaction,
    const NvOdmQueryOwrDeviceInfo* pOdmInfo,
    NvU32 NumBytes)
{
    NvU32 val = 0;
    NvError status = NvError_OwrReadFailed;
    NvU32 BytesToRead = 0;
    NvU32 WordsToRead = 0;
    NvU32 ReadDataClk = OWR_DEFAULT_READ_DTA_CLK_VALUE;
    NvU32 PresenceClk = OWR_DEFAULT_PRESENCE_CLK_VALUE;
    NvU32 i = 0;
    NvU32 size = OWR_DEFAULT_MEMORY_SIZE;
    NvU32 value = 0;

    if (pOdmInfo)
    {
        ReadDataClk = pOdmInfo->ReadDataSampleClk;
        PresenceClk = pOdmInfo->PresenceSampleClk;
        size = pOdmInfo->MemorySize;
    }

    if ( Transaction.Offset >= size)
    {
        status = NvError_OwrInvalidOffset;
        return status;
    }
    // Configure the number of bytes to read
    value = size - Transaction.Offset - 1;
    OWR_REGW(pOwrInfo->pOwrVirtualAddress,
                    EPROM,
                    value);

    // Configure the read, presence sample clock and
    // configure for byte transfer mode
    val = 
        NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, ReadDataClk) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, PresenceClk) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BYTE_TRANSFER_MODE) |
        NV_DRF_DEF(OWR, CONTROL, RD_MEM_CRC_REQ, CRC_READ) |
        NV_DRF_DEF(OWR, CONTROL, GO, START_PRESENCE_PULSE);
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, val);

    // wait for the read to complete
    status = NvOsSemaphoreWaitTimeout(pOwrInfo->OwrSyncSemaphore,
                                            OWR_TRANSFER_TIMEOUT_MILLI_SEC);
    if (status == NvSuccess)
    {
        if (pOwrInfo->OwrTransferStatus & OWR_BYTE_TRANSFER_ERRORS)
        {
            NvRmModuleReset(pOwrInfo->hRmDevice,
                NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
            OWR_PRINT(("RM_OWR Byte mode error interrupt[0x%x]\n",
                                        pOwrInfo->OwrTransferStatus));
            return NvError_OwrReadFailed;
        }
        else if (pOwrInfo->OwrTransferStatus & OwrIntrStatus_MemCmdDoneIntEnable)
        {
            // Read the data
            if (Transaction.Flags == NvRmOwr_ReadAddress)
            {
                // Read and copy and the ROM ID
                val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, READ_ROM0);
                NvOsMemcpy(pBuffer, &val, 4);
                pBuffer += 4;

                val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, READ_ROM1);
                NvOsMemcpy(pBuffer, &val, 4);
            }
            else if (Transaction.Flags == NvRmOwr_MemRead)
            {
                val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, BYTE_CNT);
                val = NV_DRF_VAL(OWR, BYTE_CNT, RECEIVED, val);
                /** Decrement the number of bytes to read count as it includes
                 * one byte CRC.
                 */
                val--;

                BytesToRead = (val > NumBytes) ? NumBytes : val;
                WordsToRead =  BytesToRead / OWR_FIFO_WORD_SIZE;
                for (i = 0; i < WordsToRead; i++)
                {
                    val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, RX_FIFO);
                    NvOsMemcpy(pBuffer, &val, OWR_FIFO_WORD_SIZE);
                    pBuffer += OWR_FIFO_WORD_SIZE;
                }

                BytesToRead = (BytesToRead % OWR_FIFO_WORD_SIZE);
                if (BytesToRead)
                {
                      val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, RX_FIFO);
                       NvOsMemcpy(pBuffer, &val, BytesToRead);
                }
            }
        }
        else
        {
            OWR_PRINT(("RM_OWR Byte mode spurious interrupt[0x%x]\n",
                            pOwrInfo->OwrTransferStatus));
            NV_ASSERT(!"RM_OWR spurious interrupt\n");
            NvRmModuleReset(pOwrInfo->hRmDevice,
                NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
            return NvError_OwrReadFailed;
        }
    }
    return status;
}

static NvError
PrivOwrWriteFifo(
    NvRmOwrController* pOwrInfo,
    NvU8*  pBuffer,
    NvRmOwrTransactionInfo Transaction,
    const NvOdmQueryOwrDeviceInfo* pOdmInfo,
    NvU32 NumBytes)
{
    NvU32 val = 0;
    NvError status = NvError_OwrWriteFailed;
    NvU32 ReadDataClk = OWR_DEFAULT_READ_DTA_CLK_VALUE;
    NvU32 PresenceClk = OWR_DEFAULT_PRESENCE_CLK_VALUE;
    NvU32 i = 0;

    if (pOdmInfo)
    {
        ReadDataClk = pOdmInfo->ReadDataSampleClk;
        PresenceClk = pOdmInfo->PresenceSampleClk;
    }
    // Configure the number of bytes to write
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, EPROM, (NumBytes - 1));

    // Write data into the FIFO
    for (i = NumBytes; i > 0; )
    {
        NvU32 BytesToWrite = NV_MIN(sizeof(NvU32),i);
        val = 0;
        switch (BytesToWrite)
        {
        case 4: val |= pBuffer[3]; i--;           // fallthrough
        case 3: val <<=8; val |= pBuffer[2]; i--; // fallthrough
        case 2: val <<=8; val |= pBuffer[1]; i--; // fallthrough
        case 1: val <<=8; val |= pBuffer[0]; i--;
            OWR_REGW(pOwrInfo->pOwrVirtualAddress, TX_FIFO, val);
            pBuffer += BytesToWrite;
            break;
        }
    }

    // Configure the read, presence sample clock and
    // configure for byte transfer mode
    val =
        NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, ReadDataClk) |
        NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, PresenceClk) |
        NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BYTE_TRANSFER_MODE) |
        NV_DRF_DEF(OWR, CONTROL, GO, START_PRESENCE_PULSE);
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, val);

    // wait for the write to complete
    status = NvOsSemaphoreWaitTimeout(pOwrInfo->OwrSyncSemaphore,
                                            OWR_TRANSFER_TIMEOUT_MILLI_SEC);
    if (status == NvSuccess)
    {
        if (pOwrInfo->OwrTransferStatus & OWR_BYTE_TRANSFER_ERRORS)
        {
            NvRmModuleReset(pOwrInfo->hRmDevice,
                NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
            OWR_PRINT(("RM_OWR Byte mode error interrupt[0x%x]\n",
                                        pOwrInfo->OwrTransferStatus));
            return NvError_OwrWriteFailed;
        }
        else if (pOwrInfo->OwrTransferStatus & OwrIntrStatus_MemCmdDoneIntEnable)
        {
            val = OWR_REGR(pOwrInfo->pOwrVirtualAddress, BYTE_CNT);
            val = NV_DRF_VAL(OWR, BYTE_CNT, RECEIVED, val);

            /** byte count includes ROM, Mem command size and Memory
             * address size. So, subtract ROM, Mem Command size and
             * memory address size from byte count.
             */
            val -= OWR_MEM_CMD_SIZE_BYTES;
            val -= OWR_MEM_CMD_SIZE_BYTES;
            val -= OWR_DEFAULT_OFFSET_SIZE_BYTES;

            /** Assert if the actual bytes written is
             * not equal to the bytes written
             */
            NV_ASSERT(val == NumBytes);
        }
        else
        {
            OWR_PRINT(("RM_OWR Byte mode spurious interrupt[0x%x]\n",
                            pOwrInfo->OwrTransferStatus));
            NV_ASSERT(!"RM_OWR spurious interrupt\n");
            NvRmModuleReset(pOwrInfo->hRmDevice,
                NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance));
            return NvError_OwrWriteFailed;
        }
    }
    return status;
}

NvError
PrivOwrCheckPresence(
        NvRmOwrController* pOwrInfo,
        NvU32 ReadDataClk,
        NvU32 PresenceClk)
{
        NvError status = NvSuccess;
        NvU32 val = 0;

        // Enable the bit transfer done interrupt
        PrivOwrEnableInterrupts(pOwrInfo, OwrIntrStatus_PresenceDoneIntEnable);
        pOwrInfo->OwrTransferStatus = 0;

        // Configure for presence
        val =
            NV_DRF_NUM(OWR, CONTROL, RD_DATA_SAMPLE_CLK, ReadDataClk) |
            NV_DRF_NUM(OWR, CONTROL, PRESENCE_SAMPLE_CLK, PresenceClk) |
            NV_DRF_DEF(OWR, CONTROL, DATA_TRANSFER_MODE, BIT_TRANSFER_MODE) |
            NV_DRF_DEF(OWR, CONTROL, GO, START_PRESENCE_PULSE);

        OWR_REGW(pOwrInfo->pOwrVirtualAddress, CONTROL, val);

        // Check for presence
        status = PrivOwrCheckBitTransferDone(pOwrInfo,
                                    OwrIntrStatus_PresenceDoneIntEnable);
        return status;
}

/****************************************************************************/

static void OwrIsr(void* args)
{
    NvRmOwrController* pOwrInfo = args;
    NvU32 IntStatus;

    // Read the interrupt status register
    IntStatus = OWR_REGR(pOwrInfo->pOwrVirtualAddress, INTR_STATUS);

    // Save the status
    pOwrInfo->OwrTransferStatus = IntStatus;

    // Clear the interrupt status register
    OWR_REGW(pOwrInfo->pOwrVirtualAddress, INTR_STATUS, IntStatus);

    // Signal the sema
    NvOsSemaphoreSignal(pOwrInfo->OwrSyncSemaphore);
    NvRmInterruptDone(pOwrInfo->OwrInterruptHandle);
}

static void AP20RmOwrClose(NvRmOwrController *pOwrInfo)
{
    if (pOwrInfo->OwrSyncSemaphore)
    {
#if !OWR_BIT_TRANSFER_POLLING_MODE
        NvRmInterruptUnregister(pOwrInfo->hRmDevice,
                        pOwrInfo->OwrInterruptHandle);
#endif
        NvOsSemaphoreDestroy(pOwrInfo->OwrSyncSemaphore);
        pOwrInfo->OwrSyncSemaphore = NULL;
        pOwrInfo->OwrInterruptHandle = NULL;
    }
    pOwrInfo->read = 0;
    pOwrInfo->write = 0;
    pOwrInfo->close = 0;
    NvRmPhysicalMemUnmap(pOwrInfo->pOwrVirtualAddress, pOwrInfo->OwrBankSize);
}

static NvError
AP20RmOwrRead(
    NvRmOwrController* pOwrInfo,
    NvU8*  pBuffer,
    NvRmOwrTransactionInfo Transaction)
{
    NvU32 val = 0;
    NvError status = NvError_BadParameter;
    NvBool IsByteModeSupported = NV_FALSE;
    const NvOdmQueryOwrDeviceInfo* pOwrOdmInfo = NULL;
    NvU32 ReadDataClk = OWR_DEFAULT_READ_DTA_CLK_VALUE;
    NvU32 PresenceClk = OWR_DEFAULT_PRESENCE_CLK_VALUE;
    NvU32 DeviceOffsetSize = OWR_DEFAULT_OFFSET_SIZE_BYTES;
    NvU32 TotalBytesToRead = 0;
    NvU32 BytesRead = 0;
    NvU32 FifoSize = 0;
    NvU32 i = 0;
    NvU8* pReadPtr = pBuffer;

    if ((Transaction.Flags == NvRmOwr_MemRead) && (!Transaction.NumBytes))
    {
        return NvError_BadParameter;
    }

    pOwrOdmInfo = NvOdmQueryGetOwrDeviceInfo(pOwrInfo->Instance);
    if (!pOwrOdmInfo)
    {
        IsByteModeSupported = NV_FALSE;

         // program the default timing registers
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, WR_RD_TCTL, 0x13fde0f7);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, RST_PRESENCE_TCTL, 0x787bbfdf);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, PROG_PULSE_TCTL, 0x01e05555);
    }
    else
    {
        IsByteModeSupported = pOwrOdmInfo->IsByteModeSupported;
        ReadDataClk = pOwrOdmInfo->ReadDataSampleClk;
        PresenceClk = pOwrOdmInfo->PresenceSampleClk;
        DeviceOffsetSize = pOwrOdmInfo->AddressSize;

         // program the timing registers
        val =
            NV_DRF_NUM(OWR, WR_RD_TCTL, TSLOT, pOwrOdmInfo->TSlot) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TLOW1, pOwrOdmInfo->TLow1) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TLOW0, pOwrOdmInfo->TLow0) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TRDV, pOwrOdmInfo->TRdv) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TRELEASE, pOwrOdmInfo->TRelease) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TSU, pOwrOdmInfo->Tsu);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, WR_RD_TCTL, val);

        val =
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TRSTH, pOwrOdmInfo->TRsth) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TRSTL, pOwrOdmInfo->TRstl) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TPDH, pOwrOdmInfo->Tpdh) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TPDL, pOwrOdmInfo->Tpdl);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, RST_PRESENCE_TCTL, val);

        val =
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TPD, pOwrOdmInfo->Tpd) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TDV, pOwrOdmInfo->Tdv) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TRP, pOwrOdmInfo->Trp) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TFP, pOwrOdmInfo->Tfp) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TPP, pOwrOdmInfo->Tpp);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, PROG_PULSE_TCTL, val);
    }

    if (Transaction.Flags == NvRmOwr_CheckPresence)
    {
        NV_ASSERT(!IsByteModeSupported);
        status = PrivOwrCheckPresence(pOwrInfo, ReadDataClk, PresenceClk);
    }
    else if (Transaction.Flags == NvRmOwr_ReadByte)
    {
        NV_ASSERT(!IsByteModeSupported);

        pOwrInfo->OwrTransferStatus = 0;
        // Enable the bit transfer done interrupt
        PrivOwrEnableInterrupts(pOwrInfo,
                                    OwrIntrStatus_BitTransferDoneIntEnable);
        status =
            PrivOwrReadData(pOwrInfo, pReadPtr, 1);
    }
    else if (Transaction.Flags == NvRmOwr_ReadBit)
    {
        NV_ASSERT(!IsByteModeSupported);
        pOwrInfo->OwrTransferStatus = 0;
        // Enable the bit transfer done interrupt
        PrivOwrEnableInterrupts(pOwrInfo,
                                    OwrIntrStatus_BitTransferDoneIntEnable);
        status = PrivOwrReadDataBit(pOwrInfo, pReadPtr);
    }
    else if ((Transaction.Flags == NvRmOwr_MemRead) ||
                 (Transaction.Flags == NvRmOwr_ReadAddress))
    {
        if (!IsByteModeSupported)
        {
            // Bit transfer mode
            status = PrivOwrCheckPresence(pOwrInfo, ReadDataClk, PresenceClk);
            if (status != NvSuccess)
                return status;

            if (Transaction.Flags == NvRmOwr_ReadAddress)
            {

                // Send the ROM Read Command
                NV_ASSERT_SUCCESS(PrivOwrSendCommand(pOwrInfo,
                            OWR_ROM_READ_COMMAND));

                // Read byte
                status = PrivOwrReadData(pOwrInfo, pReadPtr, OWR_ROM_ID_SIZE_BYTES);
            }
            else
            {
                // Skip the ROM Read Command
                NV_ASSERT_SUCCESS(
                PrivOwrSendCommand(pOwrInfo, OWR_ROM_SKIP_COMMAND));

                // Send the Mem Read Command
                NV_ASSERT_SUCCESS(
                PrivOwrSendCommand(pOwrInfo, OWR_MEM_READ_COMMAND));

                // Send offset in memory
                for (i = 0; i < DeviceOffsetSize; i++)
                {
                val = (Transaction.Offset >> i) & 0xFF;
                NV_ASSERT_SUCCESS(PrivOwrSendCommand(pOwrInfo, val));
                }

                // Read the CRC
                NV_ASSERT_SUCCESS(
                PrivOwrReadData(pOwrInfo, pReadPtr, OWR_CRC_SIZE_BYTES));

                // TODO: Need to compute the CRC and compare with the CRC read

                // Read Mem data
                status = PrivOwrReadData(pOwrInfo, pReadPtr, Transaction.NumBytes);
            }
        }
        else
        {
            // Byte transfer Mode
            // Enable the interrupts
            PrivOwrEnableInterrupts(pOwrInfo,
                        (OwrIntrStatus_PresenceErrIntEnable |
                         OwrIntrStatus_CrcErrIntEnable |
                         OwrIntrStatus_MemWriteErrIntEnable |
                         OwrIntrStatus_ErrCommandIntEnable |
                         OwrIntrStatus_MemCmdDoneIntEnable|
                         OwrIntrStatus_TxfOvfIntEnable |
                         OwrIntrStatus_RxfUnrIntEnable));

            // Configure the Rom command and the eeprom starting address
            val = (
                NV_DRF_NUM(OWR, COMMAND, ROM_CMD, OWR_ROM_READ_COMMAND) |
                NV_DRF_NUM(OWR, COMMAND, MEM_CMD, OWR_MEM_READ_COMMAND) |
                NV_DRF_NUM(OWR, COMMAND, MEM_ADDR, Transaction.Offset));
            OWR_REGW(pOwrInfo->pOwrVirtualAddress, COMMAND, val);

            /** We can't porgam ROM ID read alone, memory read should also be given
            * along with ROM ID read. So, preogramming memory read of 1byte even
            * for ROM ID read.
            */
            TotalBytesToRead = (Transaction.NumBytes) ? Transaction.NumBytes : 1;
            FifoSize = (OWR_FIFO_DEPTH * OWR_FIFO_WORD_SIZE);
            while(TotalBytesToRead)
            {
                BytesRead =
                    (TotalBytesToRead > FifoSize) ? FifoSize : TotalBytesToRead;
                pOwrInfo->OwrTransferStatus = 0;
                status =
                    PrivOwrReadFifo(pOwrInfo, pReadPtr, Transaction,
                                        pOwrOdmInfo, BytesRead);
                if (status != NvSuccess)
                {
                    break;
                }
                TotalBytesToRead -= BytesRead;
                pReadPtr += BytesRead;
            }
        }
    }
    return status;
}

static NvError
AP20RmOwrWrite(
    NvRmOwrController *pOwrInfo,
    NvU8*  pBuffer,
    NvRmOwrTransactionInfo Transaction)
{
    NvU32 val = 0;
    NvError status = NvError_BadParameter;
    NvBool IsByteModeSupported = NV_FALSE;
    const NvOdmQueryOwrDeviceInfo* pOwrOdmInfo = NULL;
    NvU32 TotalBytesToWrite = 0;
    NvU32 BytesWritten = 0;
    NvU32 FifoSize = 0;
    NvU8* pWritePtr = pBuffer;

    if ((Transaction.Flags == NvRmOwr_MemWrite) && (!Transaction.NumBytes))
    {
        return NvError_BadParameter;
    }

    pOwrOdmInfo = NvOdmQueryGetOwrDeviceInfo(pOwrInfo->Instance);
    if (!pOwrOdmInfo)
    {
        IsByteModeSupported = NV_FALSE;

         // program the default timing registers
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, WR_RD_TCTL, 0x13fde0f7);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, RST_PRESENCE_TCTL, 0x787bbfdf);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, PROG_PULSE_TCTL, 0x01e05555);
    }
    else
    {
        IsByteModeSupported = pOwrOdmInfo->IsByteModeSupported;

         // program the timing registers
        val =
            NV_DRF_NUM(OWR, WR_RD_TCTL, TSLOT, pOwrOdmInfo->TSlot) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TLOW1, pOwrOdmInfo->TLow1) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TLOW0, pOwrOdmInfo->TLow0) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TRDV, pOwrOdmInfo->TRdv) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TRELEASE, pOwrOdmInfo->TRelease) |
            NV_DRF_NUM(OWR, WR_RD_TCTL, TSU, pOwrOdmInfo->Tsu);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, WR_RD_TCTL, val);

        val =
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TRSTH, pOwrOdmInfo->TRsth) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TRSTL, pOwrOdmInfo->TRstl) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TPDH, pOwrOdmInfo->Tpdh) |
            NV_DRF_NUM(OWR, RST_PRESENCE_TCTL, TPDL, pOwrOdmInfo->Tpdl);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, RST_PRESENCE_TCTL, val);

        val =
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TPD, pOwrOdmInfo->Tpd) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TDV, pOwrOdmInfo->Tdv) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TRP, pOwrOdmInfo->Trp) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TFP, pOwrOdmInfo->Tfp) |
            NV_DRF_NUM(OWR, PROG_PULSE_TCTL, TPP, pOwrOdmInfo->Tpp);
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, PROG_PULSE_TCTL, val);
    }

    if (Transaction.Flags == NvRmOwr_MemWrite)
    {
        // Only Byte transfer Mode is supported for writes
        NV_ASSERT(IsByteModeSupported == NV_TRUE);

        // Enable the interrupts
        PrivOwrEnableInterrupts(pOwrInfo,
                        (OwrIntrStatus_PresenceErrIntEnable |
                         OwrIntrStatus_CrcErrIntEnable |
                         OwrIntrStatus_MemWriteErrIntEnable |
                         OwrIntrStatus_ErrCommandIntEnable |
                         OwrIntrStatus_MemCmdDoneIntEnable|
                         OwrIntrStatus_TxfOvfIntEnable |
                         OwrIntrStatus_RxfUnrIntEnable));

        // Configure the Rom command and the eeprom starting address
        val = (
                NV_DRF_NUM(OWR, COMMAND, ROM_CMD, OWR_ROM_READ_COMMAND) |
                NV_DRF_NUM(OWR, COMMAND, MEM_CMD, OWR_MEM_WRITE_COMMAND) |
                NV_DRF_NUM(OWR, COMMAND, MEM_ADDR, Transaction.Offset));
        OWR_REGW(pOwrInfo->pOwrVirtualAddress, COMMAND, val);

        TotalBytesToWrite = Transaction.NumBytes;
        FifoSize = (OWR_FIFO_DEPTH * OWR_FIFO_WORD_SIZE);
        while(TotalBytesToWrite)
        {
            BytesWritten =
                (TotalBytesToWrite > FifoSize) ? FifoSize : TotalBytesToWrite;
            pOwrInfo->OwrTransferStatus = 0;
            status =
                PrivOwrWriteFifo(pOwrInfo, pWritePtr, Transaction,
                                    pOwrOdmInfo, BytesWritten);
            if (status != NvSuccess)
            {
                break;
            }
            TotalBytesToWrite -= BytesWritten;
            pWritePtr += BytesWritten;
        }
    }
    else if(Transaction.Flags == NvRmOwr_WriteByte)
    {
        // Enable the bit transfer done interrupt
        PrivOwrEnableInterrupts(pOwrInfo, OwrIntrStatus_BitTransferDoneIntEnable);
        pOwrInfo->OwrTransferStatus = 0;
        status = PrivOwrSendCommand(pOwrInfo, (NvU32)(*pWritePtr));
    }
    else if(Transaction.Flags == NvRmOwr_WriteBit)
    {
        // Enable the bit transfer done interrupt
        PrivOwrEnableInterrupts(pOwrInfo, OwrIntrStatus_BitTransferDoneIntEnable);
        pOwrInfo->OwrTransferStatus = 0;
        status = PrivOwrSendBit(pOwrInfo, (NvU32)(*pWritePtr));
    }

    return status;
}

NvError AP20RmOwrOpen(NvRmOwrController *pOwrInfo)
{
    NvError status = NvSuccess;

    NV_ASSERT(pOwrInfo != NULL);

    /* Polulate the structures */
    pOwrInfo->read = AP20RmOwrRead;
    pOwrInfo->write = AP20RmOwrWrite;
    pOwrInfo->close = AP20RmOwrClose;

    NvRmModuleGetBaseAddress(
        pOwrInfo->hRmDevice,
        NVRM_MODULE_ID(NvRmModuleID_OneWire, pOwrInfo->Instance),
        &pOwrInfo->OwrPhysicalAddress,
        &pOwrInfo->OwrBankSize);

    NV_ASSERT_SUCCESS(NvRmPhysicalMemMap(
        pOwrInfo->OwrPhysicalAddress,
        pOwrInfo->OwrBankSize, NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void **)&pOwrInfo->pOwrVirtualAddress));

        // Create the sync semaphore
    status = NvOsSemaphoreCreate( &pOwrInfo->OwrSyncSemaphore, 0);

    if (pOwrInfo->OwrSyncSemaphore)
    {
        NvU32 IrqList;
        NvOsInterruptHandler IntHandlers;

        IntHandlers = OwrIsr;
        IrqList = NvRmGetIrqForLogicalInterrupt(
                pOwrInfo->hRmDevice,
                NVRM_MODULE_ID(pOwrInfo->ModuleId, pOwrInfo->Instance), 0);

#if !OWR_BIT_TRANSFER_POLLING_MODE
        status = NvRmInterruptRegister(pOwrInfo->hRmDevice, 1, &IrqList,
                    &IntHandlers, pOwrInfo,
                    &pOwrInfo->OwrInterruptHandle, NV_TRUE);
#endif

        if (status != NvSuccess)
        {
            NV_ASSERT(!"OWR module interrupt register failed!");
            NvOsSemaphoreDestroy(pOwrInfo->OwrSyncSemaphore);
            pOwrInfo->OwrSyncSemaphore = 0;
        }
    }

    return status;
}

