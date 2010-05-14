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
 * @brief <b>NVIDIA Driver Development Kit:
 *                  Spi Driver implementation</b>
 *
 * @b Description: Implementation of the NvRm SPI API of the OAL and non-OAL
 *                 version.
 *
 */

#include "nvrm_spi.h"
#include "nvrm_power.h"
#include "nvrm_interrupt.h"
#include "nvrm_memmgr.h"
#include "nvrm_dma.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "rm_spi_slink_hw_private.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pinmux.h"
#include "nvrm_pinmux_utils.h"
#include "nvodm_modules.h"
#include "rm_spi_slink.h"
#include "nvrm_priv_ap_general.h"
#include "ap15/ap15rm_private.h"


// Combined maximum spi/slink controllers
#define MAX_SPI_SLINK_INSTANCE (MAX_SLINK_CONTROLLERS + MAX_SPI_CONTROLLERS)

// Constants used to size arrays for the maximum chipselect available for the
// per spi/slink channel.
#define MAX_CHIPSELECT_PER_INSTANCE 4

// The maximum slave size request in words. Maximum 64KB/64K packet
#define MAXIMUM_SLAVE_TRANSFER_WORD (1 << (16-2))

// Maximum number which  is return by the NvOsGetTimeMS().
// For NV_OAL, NvOsGetTimeMS() returns the MicroSecond Timer count divided by 1000.
// and microsecond timer have the maximum count of 0xFFFFFFFF.
// For Non-NV_OAL, it returned maximum of 0xFFFFFFFF
#if NV_OAL
#define MAX_TIME_IN_MS (0xFFFFFFFF/1000)
#else
#define MAX_TIME_IN_MS 0xFFFFFFFF
#endif

// The maximum request size for one transaction using the dma
enum {DEFAULT_DMA_BUFFER_SIZE = (0x4000)}; // 16KB

// Maximum buffer size when transferring the data using the cpu.
enum {MAX_CPU_TRANSACTION_SIZE_WORD  = 0x80};  // 256 bytes

// Maximum non dma transfer count for apb dma got hold from allocation
enum {MAX_DMA_HOLD_TIME = 16};  // Maximum 16 non dma transaction


// The maximum number of word on which it can select the polling method when
// cpu based transaction is selected.
enum {SLINK_POLLING_HIGH_THRESOLD = 64};

// The dma buffer alignment requirement.
enum {DMA_BUFFER_ALIGNMENT = 0x10};

// Combined the Details of the current transfer information.
typedef struct
{
    NvU32 *pTxBuff;
    NvU32 *pRxBuff;

    NvU32 BytesPerPacket;
    NvU32 PacketBitLength;
    NvBool IsPackedMode;

    NvU32 PacketsPerWord;
    NvU32 PacketRequested;
    NvU32 PacketTransferred;
    NvU32 TotalPacketsRemaining;

    NvU32 RxPacketsRemaining;
    NvU32 TxPacketsRemaining;

    NvU32 CurrPacketCount;
} TransferBufferInfo;

/**
 * Combines the spi/slink channel information.
 */
typedef struct NvRmSpiRec
{
    // Nv Rm device handles.
    NvRmDeviceHandle hDevice;

    // Instance Id
    NvU32 InstanceId;

    // Is opened in master mode or slave mode.
    NvBool IsMasterMode;

    // Rm module Id for the reference.
    NvRmModuleID RmModuleId;

    // Rm IO module Id for the reference.
    NvOdmIoModule RmIoModuleId;

    // Tells whether this is the spi channel or not.
    NvBool IsSpiChannel;

    // The channel open count.
    NvU32 OpenCount;

    // Spi hw register information.
    SerialHwRegisters HwRegs;

    // Current chipselect id on which data transfer is going on.
    NvU32 CurrTransferChipSelId;

    // Synchronous sempahore Id which need to be signalled on transfer
    // completion.
    NvOsSemaphoreHandle hSynchSema;

    // Mutex to access this channel to provide the mutual exclusion.
    NvOsMutexHandle hChannelAccessMutex;

    // Tells whether the dma mode is supported or not.
    NvBool IsApbDmaAllocated;

    NvU32 TransCountFromLastDmaUsage;

    // Read dma handle.
    NvRmDmaHandle hRmRxDma;

    // Write dma handle.
    NvRmDmaHandle hRmTxDma;

    // Memory handle to create the uncached memory.
    NvRmMemHandle hRmMemory;

    // Rx Dma buffer physical address.
    NvRmPhysAddr DmaRxBuffPhysAdd;

    // Tx Dma buffer physical address.
    NvRmPhysAddr DmaTxBuffPhysAdd;

    // Virtual pointer to the Rx dma buffer.
    NvU32 *pRxDmaBuffer;

    // Virtual pointer to the Tx dma buffer.
    NvU32 *pTxDmaBuffer;

    // Current Dma transfer size for the Rx and tx
    NvU32 DmaBufferSize;

    // Dma request for Tx
    NvRmDmaClientBuffer TxDmaReq;

    // Dma request for rx
    NvRmDmaClientBuffer RxDmaReq;

    // Tell whether it is using the apb dma for the transfer or not.
    NvBool IsUsingApbDma;

    // Buffer which will be used when cpu does the data receving.
    NvU32 *pRxCpuBuffer;

    // Buffer which will be used when cpu does the data transmitting.
    NvU32 *pTxCpuBuffer;

    NvU32 CpuBufferSizeInWords;

    // Details of the current transfer information.
    TransferBufferInfo CurrTransInfo;

    // The data transfer dirction.
    SerialHwDataFlow CurrentDirection;

    // The transfer status for the receive and transmit
    NvError RxTransferStatus;
    NvError TxTransferStatus;

    // Currently configured clock frequency
    NvU32 ClockFreqInKHz;

    NvOdmQuerySpiDeviceInfo DeviceInfo[MAX_CHIPSELECT_PER_INSTANCE];

    NvBool IsCurrentChipSelStateHigh[MAX_CHIPSELECT_PER_INSTANCE];

    NvBool IsChipSelSupported[MAX_CHIPSELECT_PER_INSTANCE];

    NvBool IsChipSelConfigured;

    NvBool IsCurrentlySwBasedChipSel;
    
    HwInterfaceHandle hHwInterface;

    NvU32 RmPowerClientId;

    NvOsInterruptHandle SpiInterruptHandle;

    // Configured pin mux
    NvU32 SpiPinMap;

    // Idle signal state for the spi channel.
    NvBool IsIdleSignalTristate;

    // Frequency requiremets
    NvRmDfsBusyHint BusyHints[4];

    // Is this interface used for the pmu programmings
    NvBool IsPmuInterface;

    // If pmu interface then the CS Id for the interfacing.
    NvU32 PmuChipSelectId;

    // Tells whether frequency is boosted or not.
    NvBool IsFreqBoosted;
} NvRmSpi;

/**
 * Combines the spi/slink structure information.
 */
typedef struct
{
    // Nv Rm device handles.
    NvRmDeviceHandle hDevice;

    // Pointer to the list of the handles of the spi/slink channels.
    NvRmSpiHandle hSpiSlinkChannelList[MAX_SPI_SLINK_INSTANCE];

    // Mutex for spi/slink channel information.
    NvOsMutexHandle hChannelAccessMutex;
} NvRmPrivSpiSlinkInfo;

typedef struct
{
    NvU32 MajorVersion;
    NvU32 MinorVersion;
} SlinkCapabilities;

static NvRmPrivSpiSlinkInfo s_SpiSlinkInfo;
static HwInterface s_SpiHwInterface;
static HwInterface s_SlinkHwInterface;

/**
 * Get the interfacing property for the device connected to given chip select Id.
 * Returns whether this is supported or not.
 */
static NvBool
SpiSlinkGetDeviceInfo(
    NvBool IsSpiChannel,
    NvU32 InstanceId,
    NvU32 ChipSelect,
    NvOdmQuerySpiDeviceInfo *pDeviceInfo)
{
    const NvOdmQuerySpiDeviceInfo *pSpiDevInfo = NULL;
    NvOdmIoModule OdmModuleName;

    OdmModuleName = (IsSpiChannel)?  NvOdmIoModule_Sflash: NvOdmIoModule_Spi;
    pSpiDevInfo = NvOdmQuerySpiGetDeviceInfo(OdmModuleName, InstanceId, ChipSelect);
    if (!pSpiDevInfo)
    {
        // No device info in odm, so set it on default state.
        pDeviceInfo->SignalMode = NvOdmQuerySpiSignalMode_0;
        pDeviceInfo->ChipSelectActiveLow = NV_TRUE;
        return NV_FALSE;
    }
    pDeviceInfo->SignalMode = pSpiDevInfo->SignalMode;
    pDeviceInfo->ChipSelectActiveLow = pSpiDevInfo->ChipSelectActiveLow;
    return NV_TRUE;
}

/**
 * Find whether this interface is the pmu interface or not.
 * Returns TRUE if the given spi channel is the pmu interface else return 
 * FALSE.
 */
static NvBool
SpiSlinkIsPmuInterface(
    NvBool IsSpiChannel,
    NvU32 InstanceId,
    NvU32 *pChipSelectId)
{
    NvOdmIoModule OdmModuleName;
    NvU64 Guid = NV_PMU_TRANSPORT_ODM_ID;
    NvOdmPeripheralConnectivity const *pConnectivity;
    NvU32 Index;

    OdmModuleName = (IsSpiChannel)?  NvOdmIoModule_Sflash: NvOdmIoModule_Spi;
    *pChipSelectId = 0xFF;

     /* get the connectivity info */
    pConnectivity = NvOdmPeripheralGetGuid(Guid);
    if (!pConnectivity)
         return NV_FALSE;

    // Search for the Vdd rail and set the proper volage to the rail.
    for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
    {
        if ((pConnectivity->AddressList[Index].Interface == OdmModuleName) &&
             (pConnectivity->AddressList[Index].Instance == InstanceId))
        {
            *pChipSelectId = pConnectivity->AddressList[Index].Address;
            return NV_TRUE;
        }
    }
    return NV_FALSE;
}


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

    // Initialize the memory handle with NULL
    *phNewMemHandle = NULL;

    /// Create memory handle
    Error = NvRmMemHandleCreate(hDevice, &hNewMemHandle, BufferSize);

    // Allocates the memory from the sdram
    if (!Error)
        Error = NvRmMemAlloc(hNewMemHandle, NULL,
                        0, DMA_BUFFER_ALIGNMENT,
                        NvOsMemAttribute_Uncached);

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
  * Thread safety: Caller responsibity.
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
 * Thread safety: Caller responsibity.
 */
static NvError
CreateDmaTransferBuffer(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle *phRmMemory,
    NvRmPhysAddr *pBuffPhysAddr1,
    void **pBuffPtr1,
    NvRmPhysAddr *pBuffPhysAddr2,
    void **pBuffPtr2,
    NvU32 OneBufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hRmMemory = NULL;
    NvRmPhysAddr BuffPhysAddr;

    // Reset all the members realted to the dma buffer.
    BuffPhysAddr = 0;

    *phRmMemory = NULL;
    *pBuffPtr1 = (void *)NULL;
    *pBuffPhysAddr1 = 0;
    *pBuffPtr2 = (void *)NULL;
    *pBuffPhysAddr2 = 0;

    // Create the dma buffer memory for receive and transmit.
    // It will be double of the OneBufferSize
    Error = CreateDmaBufferMemoryHandle(hRmDevice, &hRmMemory, &BuffPhysAddr,
                                                            (OneBufferSize <<1));
    if (!Error)
    {
        // 0 to OneBufferSize-1 is buffer 1 and OneBufferSize to 2*OneBufferSize
        // is second buffer.
        Error = NvRmMemMap(hRmMemory, 0, OneBufferSize,
                                        NVOS_MEM_READ_WRITE, pBuffPtr1);
        if (!Error)
        {
            Error = NvRmMemMap(hRmMemory, OneBufferSize, OneBufferSize,
                                        NVOS_MEM_READ_WRITE, pBuffPtr2);
            if (Error)
                NvRmMemUnmap(hRmMemory, pBuffPtr1, OneBufferSize);
        }
        // If error then free the allocation and reset all changed value.
        if (Error)
        {
            DestroyDmaBufferMemoryHandle(hRmMemory);
            hRmMemory = NULL;
            *pBuffPtr1 = (void *)NULL;
            *pBuffPtr2 = (void *)NULL;
            return Error;
        }
        *phRmMemory = hRmMemory;
        *pBuffPhysAddr1 = BuffPhysAddr;
        *pBuffPhysAddr2 = BuffPhysAddr + OneBufferSize;
    }
    return Error;
}

/**
 * Destroy the dma transfer buffer.
 * Thread safety: Caller responsibity.
 */
static void
DestroyDmaTransferBuffer(
    NvRmMemHandle hRmMemory,
    void *pBuffPtr1,
    void *pBuffPtr2,
    NvU32 OneBufferSize)
{
    if (hRmMemory)
    {
        if (pBuffPtr1)
            NvRmMemUnmap(hRmMemory, pBuffPtr1, OneBufferSize);
        if (pBuffPtr2)
            NvRmMemUnmap(hRmMemory, pBuffPtr2, OneBufferSize);
        DestroyDmaBufferMemoryHandle(hRmMemory);
    }
}

static NvBool HandleTransferCompletion(NvRmSpiHandle hRmSpiSlink)
{
    NvU32 WordsReq;
    NvU32 WordsRead;
    NvU32 CurrPacketSize;
    NvU32 WordsWritten;
    HwInterfaceHandle hHwInt = hRmSpiSlink->hHwInterface;

    if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Tx)
        hRmSpiSlink->TxTransferStatus =
            hHwInt->HwGetTransferStatusFxn(&hRmSpiSlink->HwRegs, SerialHwDataFlow_Tx);

    if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx)
        hRmSpiSlink->RxTransferStatus =
            hHwInt->HwGetTransferStatusFxn(&hRmSpiSlink->HwRegs, SerialHwDataFlow_Rx);

    hHwInt->HwClearTransferStatusFxn(&hRmSpiSlink->HwRegs, hRmSpiSlink->CurrentDirection);

    // Any error then stop the transfer and return.
    if (hRmSpiSlink->RxTransferStatus || hRmSpiSlink->TxTransferStatus)
    {
        hHwInt->HwSetDataFlowFxn(&hRmSpiSlink->HwRegs, hRmSpiSlink->CurrentDirection, NV_FALSE);
        hHwInt->HwResetFifoFxn(&hRmSpiSlink->HwRegs, SerialHwFifo_Both);
        hRmSpiSlink->CurrTransInfo.PacketTransferred +=
                            hHwInt->HwGetTransferdCountFxn(&hRmSpiSlink->HwRegs);
        hRmSpiSlink->CurrentDirection = SerialHwDataFlow_None;
        return NV_TRUE;
    }

    // If dma transfer complete then return transfer completion.
    if (hRmSpiSlink->IsUsingApbDma)
        return NV_TRUE;

    if ((hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx) &&
                                (hRmSpiSlink->CurrTransInfo.RxPacketsRemaining))
    {
        WordsReq = ((hRmSpiSlink->CurrTransInfo.CurrPacketCount) +
                        ((hRmSpiSlink->CurrTransInfo.PacketsPerWord) -1))/
                        (hRmSpiSlink->CurrTransInfo.PacketsPerWord);

        WordsRead = hHwInt->HwReadFromReceiveFifoFxn(&hRmSpiSlink->HwRegs,
                            hRmSpiSlink->CurrTransInfo.pRxBuff, WordsReq);
        hRmSpiSlink->CurrTransInfo.RxPacketsRemaining -=
                                hRmSpiSlink->CurrTransInfo.CurrPacketCount;
        hRmSpiSlink->CurrTransInfo.PacketTransferred +=
                                hRmSpiSlink->CurrTransInfo.CurrPacketCount;
        hRmSpiSlink->CurrTransInfo.pRxBuff += WordsRead;
     }

    if ((hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Tx) &&
                            (hRmSpiSlink->CurrTransInfo.TxPacketsRemaining))
    {
        WordsReq = (hRmSpiSlink->CurrTransInfo.TxPacketsRemaining +
                        hRmSpiSlink->CurrTransInfo.PacketsPerWord -1)/
                        hRmSpiSlink->CurrTransInfo.PacketsPerWord;

        WordsWritten = hHwInt->HwWriteInTransmitFifoFxn(
                            &hRmSpiSlink->HwRegs,
                            hRmSpiSlink->CurrTransInfo.pTxBuff, WordsReq);
        CurrPacketSize = NV_MIN(hRmSpiSlink->CurrTransInfo.PacketsPerWord * WordsWritten,
                            hRmSpiSlink->CurrTransInfo.TxPacketsRemaining);
        hHwInt->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs, CurrPacketSize);
        hHwInt->HwStartTransferFxn(&hRmSpiSlink->HwRegs,
                                                NV_FALSE);
        hRmSpiSlink->CurrTransInfo.CurrPacketCount = CurrPacketSize;
        hRmSpiSlink->CurrTransInfo.TxPacketsRemaining -= CurrPacketSize;
        hRmSpiSlink->CurrTransInfo.PacketTransferred += CurrPacketSize;
        hRmSpiSlink->CurrTransInfo.pTxBuff += WordsWritten;
        return NV_FALSE;
    }

    // If still need to do the transfer for receiving the data then start now.
    if ((hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx) &&
                            (hRmSpiSlink->CurrTransInfo.RxPacketsRemaining))
    {
        CurrPacketSize = NV_MIN(hRmSpiSlink->CurrTransInfo.RxPacketsRemaining,
                                (hRmSpiSlink->HwRegs.MaxWordTransfer*
                                    hRmSpiSlink->CurrTransInfo.PacketsPerWord));
        hRmSpiSlink->CurrTransInfo.CurrPacketCount = CurrPacketSize;
        hHwInt->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs, CurrPacketSize);
        hHwInt->HwStartTransferFxn(&hRmSpiSlink->HwRegs, NV_FALSE);
        return NV_FALSE;
    }

    // All requested transfer is completed.
    return NV_TRUE;
}

static void SpiSlinkIsr(void *args)
{
    NvRmSpiHandle hRmSpiSlink = args;
    NvBool IsTransferCompleted;

    IsTransferCompleted = HandleTransferCompletion(hRmSpiSlink);
    if (IsTransferCompleted)
        NvOsSemaphoreSignal(hRmSpiSlink->hSynchSema);
    NvRmInterruptDone(hRmSpiSlink->SpiInterruptHandle);
}


static NvError
WaitForTransferCompletion(
    NvRmSpiHandle hRmSpiSlink,
    NvU32 WaitTimeOutMS,
    NvBool IsPoll)
{
    NvBool IsReady;
    NvBool IsTransferComplete= NV_FALSE;
    NvU32 StartTime;
    NvU32 CurrentTime;
    NvU32 TimeElapsed;
    NvBool IsWait = NV_TRUE;
    NvError Error = NvSuccess;
    NvU32 DmaRxTransferCountBytes = 0;
    NvU32 PacketTransferedFromFifoYet = 0;
    NvU32 CurrentSlinkPacketTransfer;
    NvU32 PacketsInRxFifo;
    NvU32 WordsAvailbleInFifo;
    NvU32 WordsRead;
    NvU32 *pUpdatedRxBuffer = NULL;
#if NV_OAL
    // For oal version, we only use the polling method.
    IsPoll = NV_TRUE;
#endif

    if (IsPoll)
    {
        StartTime = NvOsGetTimeMS();
        while (IsWait)
        {
            IsReady = hRmSpiSlink->hHwInterface->HwIsTransferCompletedFxn(&hRmSpiSlink->HwRegs);
            if (IsReady)
            {
                IsTransferComplete = HandleTransferCompletion(hRmSpiSlink);
                if(IsTransferComplete)
                    break;
            }
            if (WaitTimeOutMS != NV_WAIT_INFINITE)
            {
                CurrentTime = NvOsGetTimeMS();
                TimeElapsed = (CurrentTime >= StartTime)? (CurrentTime - StartTime):
                                    MAX_TIME_IN_MS - StartTime + CurrentTime;
                IsWait = (TimeElapsed > WaitTimeOutMS)? NV_FALSE: NV_TRUE;
            }
        }

        Error = (IsTransferComplete)? NvError_Success: NvError_Timeout;
#if NV_OAL
        // If no error and apb dma based transfer then stop the dma transfer to
        // make the state dma state machine as non busy.
        if ((!Error) && (hRmSpiSlink->IsUsingApbDma))
        {
            if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx)
                NvRmDmaAbort(hRmSpiSlink->hRmRxDma);
            if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Tx)
                NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
        }
#endif
    }
    else
    {
        Error = NvOsSemaphoreWaitTimeout(hRmSpiSlink->hSynchSema, WaitTimeOutMS);
    }

    // If timeout happen then stop all transfer and exit.
    if (Error == NvError_Timeout)
    {
        // Disable the data flow first.
        hRmSpiSlink->hHwInterface->HwSetDataFlowFxn(&hRmSpiSlink->HwRegs,
                                    hRmSpiSlink->CurrentDirection, NV_FALSE);

        // Get the transfer count now.
        if (hRmSpiSlink->IsUsingApbDma)
        {
            if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx)
            {
                // Get the Rx transfer count transferred by Dma.
                Error = NvRmDmaGetTransferredCount(hRmSpiSlink->hRmRxDma,
                            &DmaRxTransferCountBytes, NV_TRUE);
                NV_ASSERT(Error == NvSuccess);
                if (Error != NvSuccess)
                    DmaRxTransferCountBytes = 0;
                PacketTransferedFromFifoYet = (DmaRxTransferCountBytes >> 2) *
                                                hRmSpiSlink->CurrTransInfo.PacketsPerWord;
                pUpdatedRxBuffer = hRmSpiSlink->pRxDmaBuffer + (DmaRxTransferCountBytes >> 2);
                NvRmDmaAbort(hRmSpiSlink->hRmRxDma);
            }

            if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Tx)
                NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
        }
        else
        {
            PacketTransferedFromFifoYet = hRmSpiSlink->CurrTransInfo.PacketTransferred;
            pUpdatedRxBuffer = hRmSpiSlink->CurrTransInfo.pRxBuff;
        }

        // Check again whether the transfer is completed or not.
        // It may be possible that transfer is completed when we reach here.
        // If transfer is completed then we may read 0 from the status
        // register
        IsReady = hRmSpiSlink->hHwInterface->HwIsTransferCompletedFxn(&hRmSpiSlink->HwRegs);
        if (IsReady)
        {
            // All requested transfer has been done.
            CurrentSlinkPacketTransfer = hRmSpiSlink->CurrTransInfo.CurrPacketCount;
            Error = NvSuccess;
        }
        else
        {
            // Get the transfer count from status register.
            CurrentSlinkPacketTransfer =
                hRmSpiSlink->hHwInterface->HwGetTransferdCountFxn(&hRmSpiSlink->HwRegs);

            // If it is in packed mode and number of received packet is non word
            // aligned then ignore the packet which does not able to make the word.
            // This is because we can not read such packet from fifo as this is not
            // avaiable in the fifo. -- Hw issue
            if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx)
            {
                if (hRmSpiSlink->CurrTransInfo.PacketsPerWord > 1)
                    CurrentSlinkPacketTransfer -=
                                CurrentSlinkPacketTransfer%
                                            hRmSpiSlink->CurrTransInfo.PacketsPerWord;
            }

        }
        hRmSpiSlink->CurrTransInfo.PacketTransferred += CurrentSlinkPacketTransfer;

        // Disable the interrupt.
        if (!IsPoll)
            hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
                                        hRmSpiSlink->CurrentDirection, NV_FALSE);

        // For Rx: Dma will always transfer equal to or less than slink has
        // transferred. If slink has transferred more data and dma have
        // not transferrd from the fifo to memory then there may be some more
        // data available into the fifo. Reading those from cpu.
        // For Tx: The dma will transfer more than slink has and non transferred
        // data wil be in foopf which will get reset after slink reset. No need
        // to do any more for tx case.
        if (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx)
        {
            // If slink transfrred word is more than the dma transfer count
            // then some more data is available into the fifo. Read then
            // through CPU.
            if (PacketTransferedFromFifoYet < CurrentSlinkPacketTransfer)
            {
                PacketsInRxFifo = CurrentSlinkPacketTransfer - PacketTransferedFromFifoYet;
                WordsAvailbleInFifo =
                    (PacketsInRxFifo + hRmSpiSlink->CurrTransInfo.PacketsPerWord -1)/
                                            hRmSpiSlink->CurrTransInfo.PacketsPerWord;
                WordsRead = hRmSpiSlink->hHwInterface->HwReadFromReceiveFifoFxn(
                                &hRmSpiSlink->HwRegs, pUpdatedRxBuffer, WordsAvailbleInFifo);

                // Expecting the WordsRead should be equal to WordsAvailbleInFifo
                if (WordsRead != WordsAvailbleInFifo)
                {
                    NV_ASSERT(WordsRead == WordsAvailbleInFifo);
                }
            }
        }


        // The busy bit will still show the busy status so need to reset the
        // controller.  .. Hw Bug
        NvRmModuleReset(hRmSpiSlink->hDevice,
                    NVRM_MODULE_ID(hRmSpiSlink->RmModuleId, hRmSpiSlink->InstanceId));
        hRmSpiSlink->CurrentDirection = SerialHwDataFlow_None;
    }
    return Error;
}

#if NV_OAL
static void OalMasterSpiSlinkPoll(NvRmSpiHandle hRmSpiSlink)
{
    NvBool IsReady;
    NvBool TransferComplete = NV_FALSE;
    //Check for the transfer complete in infinite loop
    while (1)
    {
        IsReady =  hRmSpiSlink->hHwInterface->HwIsTransferCompletedFxn(&hRmSpiSlink->HwRegs);
        if (IsReady)
        {
            TransferComplete = HandleTransferCompletion(hRmSpiSlink);
            if(TransferComplete)
                break;
        }
    }
}
#endif

/**
 * Register the spi interrupt.
 * Thread safety: Caller responsibity.
 */
static NvError
RegisterSpiSlinkInterrupt(
    NvRmDeviceHandle hDevice,
    NvRmSpiHandle hRmSpiSlink,
    NvU32 InstanceId)
{
    NvU32 IrqList;
    NvOsInterruptHandler hIntHandlers;
    if (hRmSpiSlink->SpiInterruptHandle)
        return NvSuccess;

    IrqList = NvRmGetIrqForLogicalInterrupt(
            hDevice, NVRM_MODULE_ID(hRmSpiSlink->RmModuleId, InstanceId), 0);
    hIntHandlers = SpiSlinkIsr;
    return(NvRmInterruptRegister(hDevice, 1, &IrqList,
            &hIntHandlers, hRmSpiSlink, &hRmSpiSlink->SpiInterruptHandle, NV_TRUE));
}
// Boosting the Emc/Ahb/Apb/Cpu frequency
static void BoostFrequency(NvRmSpiHandle hRmSpiSlink, NvBool IsBoost, NvU32 TransactionSize)
{
    if (IsBoost)
    {
        if (TransactionSize > hRmSpiSlink->HwRegs.MaxWordTransfer)
        {
            if (!((hRmSpiSlink->IsPmuInterface) &&
                   (hRmSpiSlink->PmuChipSelectId == hRmSpiSlink->CurrTransferChipSelId)))
            {
                hRmSpiSlink->BusyHints[0].BoostKHz = 80000; // Emc
                hRmSpiSlink->BusyHints[1].BoostKHz = 80000; // Ahb
                hRmSpiSlink->BusyHints[2].BoostKHz = 80000; // Apb
                hRmSpiSlink->BusyHints[3].BoostKHz = 240000; // Cpu
                NvRmPowerBusyHintMulti(hRmSpiSlink->hDevice, hRmSpiSlink->RmPowerClientId,
                                       hRmSpiSlink->BusyHints, 4,
                                       NvRmDfsBusyHintSyncMode_Async);
                hRmSpiSlink->IsFreqBoosted = NV_TRUE;
            }
        }
    }
    else
    {
        if (hRmSpiSlink->IsFreqBoosted)
        {
            if (!((hRmSpiSlink->IsPmuInterface) &&
                   (hRmSpiSlink->PmuChipSelectId == hRmSpiSlink->CurrTransferChipSelId)))
            {
                hRmSpiSlink->BusyHints[0].BoostKHz = 0; // Emc
                hRmSpiSlink->BusyHints[1].BoostKHz = 0; // Ahb
                hRmSpiSlink->BusyHints[2].BoostKHz = 0; // Apb
                hRmSpiSlink->BusyHints[3].BoostKHz = 0; // Cpu
                NvRmPowerBusyHintMulti(hRmSpiSlink->hDevice, hRmSpiSlink->RmPowerClientId,
                                       hRmSpiSlink->BusyHints, 4,
                                       NvRmDfsBusyHintSyncMode_Async);
                hRmSpiSlink->IsFreqBoosted = NV_FALSE;
            }
        }
    }
}

static NvError SetPowerControl(NvRmSpiHandle hRmSpiSlink, NvBool IsEnable)
{
    NvError Error = NvSuccess;
    NvRmModuleID ModuleId;

    ModuleId = NVRM_MODULE_ID(hRmSpiSlink->RmModuleId, hRmSpiSlink->InstanceId);
    if (IsEnable)
    {
        // Enable power for spi/slink module
        Error = NvRmPowerVoltageControl(hRmSpiSlink->hDevice, ModuleId,
                                    hRmSpiSlink->RmPowerClientId,
                                    NvRmVoltsUnspecified, NvRmVoltsUnspecified,
                                    NULL, 0, NULL);
        // Enable the clock.
        if (!Error)
            Error = NvRmPowerModuleClockControl(hRmSpiSlink->hDevice, ModuleId,
                                    hRmSpiSlink->RmPowerClientId, NV_TRUE);
    }
    else
    {
        // Disable the clocks.
        (void)NvRmPowerModuleClockControl(hRmSpiSlink->hDevice, ModuleId,
                        hRmSpiSlink->RmPowerClientId, NV_FALSE);


        // Disable the power to the controller.
        (void)NvRmPowerVoltageControl(hRmSpiSlink->hDevice, ModuleId,
                                        hRmSpiSlink->RmPowerClientId,
                                        NvRmVoltsOff, NvRmVoltsOff,
                                        NULL, 0, NULL);
    }
    return Error;
}

/**
 * Destroy the handle of spi channel and free all the allocation done for it.
 * Thread safety: Caller responsibity.
 */
static void DestroySpiSlinkChannelHandle(NvRmSpiHandle hRmSpiSlink)
{
    NvU32 HandleStartIndex;
#if !NV_OAL
    NvRmInterruptUnregister(hRmSpiSlink->hDevice, hRmSpiSlink->SpiInterruptHandle);
    hRmSpiSlink->SpiInterruptHandle = NULL;
#endif


    // Unmap the virtual mapping of the spi hw register.
    NvRmPhysicalMemUnmap(hRmSpiSlink->HwRegs.pRegsBaseAdd, hRmSpiSlink->HwRegs.RegBankSize);

    // the clocks should already be disabled for Non-oal. don't disable it here for non-oal
    // For oal disable here.
#if NV_OAL

    // Resetting the Emc/Ahb/Apb/Cpu frequency
    BoostFrequency(hRmSpiSlink, NV_FALSE, 0);
    (void)SetPowerControl(hRmSpiSlink, NV_FALSE);
#endif

#if !NV_OAL
    // Unregister for the power manager.
    NvRmPowerUnRegister(hRmSpiSlink->hDevice, hRmSpiSlink->RmPowerClientId);
#endif

    // Tri-State the pin-mux pins
    NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hRmSpiSlink->hDevice,
        NVRM_MODULE_ID(hRmSpiSlink->RmModuleId,hRmSpiSlink->InstanceId), NV_TRUE));

    NvOsFree(hRmSpiSlink->pTxCpuBuffer);
    NvOsFree(hRmSpiSlink->pRxCpuBuffer);

    if (hRmSpiSlink->hRmRxDma)
    {
        NvRmDmaAbort(hRmSpiSlink->hRmRxDma);
        NvRmDmaFree(hRmSpiSlink->hRmRxDma);
    }

    if (hRmSpiSlink->hRmTxDma)
    {
        NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
        NvRmDmaFree(hRmSpiSlink->hRmTxDma);
    }

    DestroyDmaTransferBuffer(hRmSpiSlink->hRmMemory, hRmSpiSlink->pRxDmaBuffer,
                hRmSpiSlink->pTxDmaBuffer, hRmSpiSlink->DmaBufferSize);

#if !NV_OAL
    // Destroy the mutex allocated for the channel accss.
    NvOsMutexDestroy(hRmSpiSlink->hChannelAccessMutex);

    // Destroy the sync sempahores.
    NvOsSemaphoreDestroy(hRmSpiSlink->hSynchSema);
#endif

    HandleStartIndex = (hRmSpiSlink->IsSpiChannel)? 0: MAX_SPI_CONTROLLERS;
    s_SpiSlinkInfo.hSpiSlinkChannelList[HandleStartIndex + hRmSpiSlink->InstanceId] = NULL;

    // Free the memory of the spi handles.
    NvOsFree(hRmSpiSlink);
}


/**
 * Create the handle for the spi channel.
 * Thread safety: Caller responsibity.
 */
static NvError CreateSpiSlinkChannelHandle(
    NvRmDeviceHandle hDevice,
    NvBool IsSpiChannel,
    NvU32 InstanceId,
    NvBool IsMasterMode,
    NvRmSpiHandle *phSpiSlinkChannel)
{
    NvError Error = NvSuccess;
    NvRmSpiHandle hRmSpiSlink = NULL;
    NvRmModuleID ModuleId;
    NvU32 ChipSelIndex;
    NvU32 InstIndexOffset = (IsSpiChannel)? 0: MAX_SPI_CONTROLLERS;
    const NvU32 *pOdmConfigs;
    NvU32 NumOdmConfigs;
    NvU32 CpuBufferSize;
    NvRmDmaModuleID DmaModuleId;
    const NvOdmQuerySpiIdleSignalState *pSignalState = NULL;

    *phSpiSlinkChannel = NULL;

    // Allcoate the memory for the spi handle.
    hRmSpiSlink = NvOsAlloc(sizeof(NvRmSpi));
    if (!hRmSpiSlink)
        return NvError_InsufficientMemory;

    NvOsMemset(hRmSpiSlink, 0, sizeof(NvRmSpi));

    // Set the spi handle parameters.
    hRmSpiSlink->hDevice = hDevice;
    hRmSpiSlink->InstanceId = InstanceId;
    hRmSpiSlink->IsSpiChannel = IsSpiChannel;
    hRmSpiSlink->IsMasterMode = IsMasterMode;
    hRmSpiSlink->RmModuleId = (IsSpiChannel)?NvRmModuleID_Spi: NvRmModuleID_Slink;
    hRmSpiSlink->RmIoModuleId = (IsSpiChannel)?NvOdmIoModule_Sflash: NvOdmIoModule_Spi;
    hRmSpiSlink->OpenCount = 1;
    hRmSpiSlink->IsApbDmaAllocated = NV_FALSE;
    hRmSpiSlink->TransCountFromLastDmaUsage = 0;
    hRmSpiSlink->hRmRxDma = NULL;
    hRmSpiSlink->hRmMemory = NULL;
    hRmSpiSlink->hRmTxDma = NULL;
    hRmSpiSlink->DmaRxBuffPhysAdd = 0;
    hRmSpiSlink->DmaTxBuffPhysAdd = 0;
    hRmSpiSlink->pRxDmaBuffer = NULL;
    hRmSpiSlink->pTxDmaBuffer = NULL;
    hRmSpiSlink->pTxCpuBuffer = NULL;
    hRmSpiSlink->pRxCpuBuffer = NULL;
    hRmSpiSlink->CpuBufferSizeInWords = 0;
    hRmSpiSlink->hHwInterface = NULL;
    hRmSpiSlink->RmPowerClientId = 0;
    hRmSpiSlink->SpiPinMap = 0;
    hRmSpiSlink->CurrTransferChipSelId = 0;
    hRmSpiSlink->IsChipSelConfigured = NV_FALSE;
    hRmSpiSlink->IsCurrentlySwBasedChipSel = NV_TRUE;

    // Initialize the frequncy requirements array
    hRmSpiSlink->BusyHints[0].ClockId = NvRmDfsClockId_Emc;
    hRmSpiSlink->BusyHints[0].BoostDurationMs = NV_WAIT_INFINITE;
    hRmSpiSlink->BusyHints[0].BusyAttribute = NV_TRUE;

    hRmSpiSlink->BusyHints[1].ClockId = NvRmDfsClockId_Ahb;
    hRmSpiSlink->BusyHints[1].BoostDurationMs = NV_WAIT_INFINITE;
    hRmSpiSlink->BusyHints[1].BusyAttribute = NV_TRUE;

    hRmSpiSlink->BusyHints[2].ClockId = NvRmDfsClockId_Apb;
    hRmSpiSlink->BusyHints[2].BoostDurationMs = NV_WAIT_INFINITE;
    hRmSpiSlink->BusyHints[2].BusyAttribute = NV_TRUE;

    hRmSpiSlink->BusyHints[3].ClockId = NvRmDfsClockId_Cpu;
    hRmSpiSlink->BusyHints[3].BoostDurationMs = NV_WAIT_INFINITE;
    hRmSpiSlink->BusyHints[3].BusyAttribute = NV_TRUE;

    hRmSpiSlink->IsFreqBoosted = NV_FALSE;
    hRmSpiSlink->IsPmuInterface = NV_FALSE;
    hRmSpiSlink->PmuChipSelectId = 0xFF;

    ModuleId = NVRM_MODULE_ID(hRmSpiSlink->RmModuleId, InstanceId);

    if (IsSpiChannel)
        hRmSpiSlink->hHwInterface = &s_SpiHwInterface;
    else
        hRmSpiSlink->hHwInterface = &s_SlinkHwInterface;

    for (ChipSelIndex = 0; ChipSelIndex < MAX_CHIPSELECT_PER_INSTANCE; ++ChipSelIndex)
        hRmSpiSlink->IsChipSelSupported[ChipSelIndex] =
                SpiSlinkGetDeviceInfo(IsSpiChannel, InstanceId, ChipSelIndex,
                                    &hRmSpiSlink->DeviceInfo[ChipSelIndex]);

    // Findout whether this spi instance is used for the pmu interface or not.
    hRmSpiSlink->IsPmuInterface = SpiSlinkIsPmuInterface(IsSpiChannel, 
                                                InstanceId,
                                                &hRmSpiSlink->PmuChipSelectId);

    // Get the odm pin map
    NvOdmQueryPinMux(hRmSpiSlink->RmIoModuleId, &pOdmConfigs, &NumOdmConfigs);
    NV_ASSERT((InstanceId < NumOdmConfigs) && (pOdmConfigs[InstanceId]));
    hRmSpiSlink->SpiPinMap = pOdmConfigs[InstanceId];

    pSignalState = NvOdmQuerySpiGetIdleSignalState(hRmSpiSlink->RmIoModuleId, InstanceId);
    if (pSignalState)
    {
        hRmSpiSlink->IsIdleSignalTristate = pSignalState->IsTristate;
        hRmSpiSlink->HwRegs.IdleSignalMode = pSignalState->SignalMode;
        hRmSpiSlink->HwRegs.IsIdleDataOutHigh = pSignalState->IsIdleDataOutHigh;
    }
    else
    {
        hRmSpiSlink->IsIdleSignalTristate = NV_FALSE;
        hRmSpiSlink->HwRegs.IdleSignalMode = NvOdmQuerySpiSignalMode_0;
        hRmSpiSlink->HwRegs.IsIdleDataOutHigh = NV_FALSE;
    }
    Error = NvRmSetModuleTristate(hRmSpiSlink->hDevice, ModuleId,
                                        hRmSpiSlink->IsIdleSignalTristate);
    if (Error)
    {
        // If error then return from here.
        NvOsFree(hRmSpiSlink);
        return Error;
    }

    hRmSpiSlink->RxTransferStatus = NvSuccess;
    hRmSpiSlink->TxTransferStatus = NvSuccess;

    hRmSpiSlink->hHwInterface->HwRegisterInitializeFxn(InstanceId, &hRmSpiSlink->HwRegs);

#if !NV_OAL
    // Create the mutex for channel access.
    if (!Error)
        Error = NvOsMutexCreate(&hRmSpiSlink->hChannelAccessMutex);

    // Create the synchronous semaphores.
    if (!Error)
        Error = NvOsSemaphoreCreate(&hRmSpiSlink->hSynchSema, 0);
#endif

    // Get the spi hw physical base address and map in virtual memory space.
    if (!Error)
    {
        NvRmPhysAddr SpiSlinkPhysAddr;
        NvRmModuleGetBaseAddress(hDevice, ModuleId,
                        &SpiSlinkPhysAddr, &hRmSpiSlink->HwRegs.RegBankSize);

        hRmSpiSlink->HwRegs.HwRxFifoAdd += SpiSlinkPhysAddr;
        hRmSpiSlink->HwRegs.HwTxFifoAdd += SpiSlinkPhysAddr;
        Error = NvRmPhysicalMemMap(SpiSlinkPhysAddr,
                            hRmSpiSlink->HwRegs.RegBankSize, NVOS_MEM_READ_WRITE,
                            NvOsMemAttribute_Uncached,
                            (void **)&hRmSpiSlink->HwRegs.pRegsBaseAdd);
    }

    // Allocate the dma buffer and the dma channel
    if (!Error)
    {
        hRmSpiSlink->IsApbDmaAllocated = NV_TRUE;

    // Don't go to the dma allocation if the oal and master mode.
    // It creates the download issue using the spi kitl if dma mode is used.
#if NV_OAL
        if (IsMasterMode)
        {
            Error = NvError_NotSupported;
        }
        else
        {
            Error = CreateDmaTransferBuffer(hRmSpiSlink->hDevice, &hRmSpiSlink->hRmMemory,
                    &hRmSpiSlink->DmaRxBuffPhysAdd, (void **)&hRmSpiSlink->pRxDmaBuffer,
                    &hRmSpiSlink->DmaTxBuffPhysAdd, (void **)&hRmSpiSlink->pTxDmaBuffer,
                    DEFAULT_DMA_BUFFER_SIZE);
        }
#else
        Error = CreateDmaTransferBuffer(hRmSpiSlink->hDevice, &hRmSpiSlink->hRmMemory,
                &hRmSpiSlink->DmaRxBuffPhysAdd, (void **)&hRmSpiSlink->pRxDmaBuffer,
                &hRmSpiSlink->DmaTxBuffPhysAdd, (void **)&hRmSpiSlink->pTxDmaBuffer,
                DEFAULT_DMA_BUFFER_SIZE);
#endif
        if (!Error)
        {
            hRmSpiSlink->DmaBufferSize = DEFAULT_DMA_BUFFER_SIZE;
            DmaModuleId = (IsSpiChannel)?NvRmDmaModuleID_Spi: NvRmDmaModuleID_Slink;

            // Allocate the dma (for Rx and for Tx) with high priority
            // Allocate dma now only for the slave mode handle.
            // For master mode, it will be allaocated based on the transaction size
            // to make it adaptive.
            if (!IsMasterMode)
            {
                Error = NvRmDmaAllocate(hRmSpiSlink->hDevice, &hRmSpiSlink->hRmRxDma,
                                 NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                                 hRmSpiSlink->InstanceId);
                if (!Error)
                {
                    Error = NvRmDmaAllocate(hRmSpiSlink->hDevice, &hRmSpiSlink->hRmTxDma,
                                     NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                                     hRmSpiSlink->InstanceId);
                    if (Error)
                        NvRmDmaFree(hRmSpiSlink->hRmRxDma);
                }
                if (Error)
                {
                    DestroyDmaTransferBuffer(hRmSpiSlink->hRmMemory,
                            hRmSpiSlink->pRxDmaBuffer, hRmSpiSlink->pTxDmaBuffer,
                            hRmSpiSlink->DmaBufferSize);
                }
            }
            else
            {
                hRmSpiSlink->IsApbDmaAllocated = NV_FALSE;
                hRmSpiSlink->hRmRxDma = NULL;
                hRmSpiSlink->hRmTxDma = NULL;
            }
        }
        if (Error)
        {
            hRmSpiSlink->IsApbDmaAllocated = NV_FALSE;
            hRmSpiSlink->hRmRxDma = NULL;
            hRmSpiSlink->hRmMemory = NULL;
            hRmSpiSlink->hRmTxDma = NULL;
            hRmSpiSlink->DmaRxBuffPhysAdd = 0;
            hRmSpiSlink->DmaTxBuffPhysAdd = 0;
            hRmSpiSlink->pRxDmaBuffer = NULL;
            hRmSpiSlink->pTxDmaBuffer = NULL;
            Error = NvSuccess;
        }
        else
        {
            hRmSpiSlink->RxDmaReq.SourceBufferPhyAddress= hRmSpiSlink->HwRegs.HwRxFifoAdd;
            hRmSpiSlink->RxDmaReq.DestinationBufferPhyAddress = hRmSpiSlink->DmaRxBuffPhysAdd;
            hRmSpiSlink->RxDmaReq.SourceAddressWrapSize = 4;
            hRmSpiSlink->RxDmaReq.DestinationAddressWrapSize = 0;

            hRmSpiSlink->TxDmaReq.SourceBufferPhyAddress= hRmSpiSlink->DmaTxBuffPhysAdd;
            hRmSpiSlink->TxDmaReq.DestinationBufferPhyAddress = hRmSpiSlink->HwRegs.HwTxFifoAdd;
            hRmSpiSlink->TxDmaReq.SourceAddressWrapSize = 0;
            hRmSpiSlink->TxDmaReq.DestinationAddressWrapSize = 4;
        }
    }

    if (!Error)
    {
        // If dma is allocated then allocate the less size of the cpu buffer
        // otherwise allocate bigger size to get the optimized timing execution.
        CpuBufferSize = (hRmSpiSlink->IsApbDmaAllocated)?
                            (MAX_CPU_TRANSACTION_SIZE_WORD << 2): DEFAULT_DMA_BUFFER_SIZE;

        hRmSpiSlink->pRxCpuBuffer = NvOsAlloc(CpuBufferSize);
        if (!hRmSpiSlink->pRxCpuBuffer)
            Error = NvError_InsufficientMemory;

        if (!Error)
        {
            hRmSpiSlink->pTxCpuBuffer = NvOsAlloc(CpuBufferSize);
            if (!hRmSpiSlink->pTxCpuBuffer)
                Error = NvError_InsufficientMemory;
        }
        if (!Error)
            hRmSpiSlink->CpuBufferSizeInWords = CpuBufferSize >> 2;
    }

#if !NV_OAL
    // Register slink/spi for Rm power client
    if (!Error)
    {
       hRmSpiSlink->RmPowerClientId = NVRM_POWER_CLIENT_TAG('S','P','I',' ');
       Error = NvRmPowerRegister(hRmSpiSlink->hDevice, NULL, &hRmSpiSlink->RmPowerClientId);
    }
#endif

    // Enable Power/Clock.
    if (!Error)
        Error = SetPowerControl(hRmSpiSlink, NV_TRUE);

    // Reset the module.
    if (!Error)
        NvRmModuleReset(hDevice, ModuleId);

#if !NV_OAL
    // Register the interrupt.
    if (!Error)
        Error = RegisterSpiSlinkInterrupt(hDevice, hRmSpiSlink, InstanceId);
#endif

    // Initialize the controller register.
    if (!Error)
    {
        // Set the default signal mode of the spi channel.
        hRmSpiSlink->hHwInterface->HwSetSignalModeFxn(&hRmSpiSlink->HwRegs, hRmSpiSlink->HwRegs.IdleSignalMode);

        // Set chip select to non active state.
        hRmSpiSlink->hHwInterface->HwControllerInitializeFxn(&hRmSpiSlink->HwRegs);
        for (ChipSelIndex = 0; ChipSelIndex < MAX_CHIPSELECT_PER_INSTANCE; ++ChipSelIndex)
        {
            hRmSpiSlink->IsCurrentChipSelStateHigh[ChipSelIndex] = NV_TRUE;
            if (hRmSpiSlink->IsChipSelSupported[ChipSelIndex])
            {
                hRmSpiSlink->IsCurrentChipSelStateHigh[ChipSelIndex] =
                    hRmSpiSlink->DeviceInfo[ChipSelIndex].ChipSelectActiveLow;
                hRmSpiSlink->hHwInterface->HwSetChipSelectDefaultLevelFxn(
                            &hRmSpiSlink->HwRegs, ChipSelIndex,
                            hRmSpiSlink->IsCurrentChipSelStateHigh[ChipSelIndex]);
            }
        }
        // Let chipselect to be stable for 1 ms before doing any transaction.
        NvOsWaitUS(1000);
#if !NV_OAL
        // switch off clock and power to the slink module by default.
        Error = SetPowerControl(hRmSpiSlink, NV_FALSE);
#endif
    }

    // If error then destroy all the allocation done here.
    if (Error)
    {
        DestroySpiSlinkChannelHandle(hRmSpiSlink);
        hRmSpiSlink = NULL;
    }

    *phSpiSlinkChannel = hRmSpiSlink;
    s_SpiSlinkInfo.hSpiSlinkChannelList[InstanceId + InstIndexOffset] = hRmSpiSlink;
    return Error;
}

/**
 * Set the chip select signal level to be active or inactive.
 */
static NvError
SetChipSelectSignalLevel(
    NvRmSpiHandle hRmSpiSlink,
    NvU32 ChipSelectId,
    NvU32 ClockSpeedInKHz,
    NvBool IsActive,
    NvBool IsOnlyUseSWCS)
{
    NvError Error = NvSuccess;
    NvBool IsHigh;
    NvRmModuleID ModuleId;
    NvU32 PrefClockFreqInKHz;
    NvU32 ConfiguredClockFreqInKHz = 0;
    NvOdmQuerySpiDeviceInfo *pDevInfo = &hRmSpiSlink->DeviceInfo[ChipSelectId];
    HwInterfaceHandle hHwIntf = hRmSpiSlink->hHwInterface;
    if (IsActive)
    {
        if (ClockSpeedInKHz != hRmSpiSlink->ClockFreqInKHz)
        {
            ModuleId = NVRM_MODULE_ID(hRmSpiSlink->RmModuleId, hRmSpiSlink->InstanceId);

            // The slink clock source should be 4 times of the interface clock speed
            PrefClockFreqInKHz = (hRmSpiSlink->RmModuleId == NvRmModuleID_Slink)?
                                (ClockSpeedInKHz << 2): (ClockSpeedInKHz);
            Error = NvRmPowerModuleClockConfig(hRmSpiSlink->hDevice,
                                          ModuleId, 0, PrefClockFreqInKHz,
                                          NvRmFreqUnspecified, &PrefClockFreqInKHz,
                                          1, &ConfiguredClockFreqInKHz, 0);
            if (Error)
                return Error;

            hRmSpiSlink->ClockFreqInKHz = ClockSpeedInKHz;
        }

        if (pDevInfo->SignalMode != hRmSpiSlink->HwRegs.CurrSignalMode)
            hHwIntf->HwSetSignalModeFxn(&hRmSpiSlink->HwRegs, pDevInfo->SignalMode);

        if (hRmSpiSlink->IsMasterMode != hRmSpiSlink->HwRegs.IsMasterMode)
            hHwIntf->HwSetFunctionalModeFxn(&hRmSpiSlink->HwRegs, hRmSpiSlink->IsMasterMode);

        if (IsOnlyUseSWCS || (!hRmSpiSlink->HwRegs.IsHwChipSelectSupported))
        {
            IsHigh = (pDevInfo->ChipSelectActiveLow)? NV_FALSE: NV_TRUE;
            hHwIntf->HwSetChipSelectLevelFxn(&hRmSpiSlink->HwRegs, ChipSelectId, IsHigh);
            hRmSpiSlink->IsChipSelConfigured = NV_TRUE;
            hRmSpiSlink->IsCurrentlySwBasedChipSel = NV_TRUE;
            hRmSpiSlink->IsCurrentChipSelStateHigh[ChipSelectId] = IsHigh;
        }
        else
        {
            hRmSpiSlink->IsChipSelConfigured = NV_FALSE;
        }
    }
    else
    {
        if (IsOnlyUseSWCS || hRmSpiSlink->IsCurrentlySwBasedChipSel)
        {
            IsHigh = (pDevInfo->ChipSelectActiveLow)? NV_TRUE: NV_FALSE;
            hHwIntf->HwSetChipSelectLevelFxn(&hRmSpiSlink->HwRegs, ChipSelectId, IsHigh);
            if (hRmSpiSlink->HwRegs.IdleSignalMode != hRmSpiSlink->HwRegs.CurrSignalMode)
                hHwIntf->HwSetSignalModeFxn(&hRmSpiSlink->HwRegs, hRmSpiSlink->HwRegs.IdleSignalMode);

            hRmSpiSlink->IsCurrentChipSelStateHigh[ChipSelectId] = IsHigh;
        }
        hRmSpiSlink->IsChipSelConfigured = NV_FALSE;

    }
    return NvSuccess;
}


static void
MakeMasterSpiBufferFromClientBuffer(
    NvU8 *pTxBuffer,
    NvU32 *pSpiBuffer,
    NvU32 BytesRequested,
    NvU32 PacketBitLength,
    NvU32 IsPackedMode)
{
    NvU32 Shift0;
    NvU32 MSBMaskData = 0xFF;
    NvU32 BytesPerPackets;
    NvU32 Index;
    NvU32 PacketRequest;

    if (IsPackedMode)
    {
        if (PacketBitLength == 8)
        {
            NvOsMemcpy(pSpiBuffer, pTxBuffer, BytesRequested);
            return;
        }

        BytesPerPackets = (PacketBitLength + 7)/8;
        PacketRequest = BytesRequested / BytesPerPackets;
        if (PacketBitLength == 16)
        {
            NvU16 *pOutBuffer = (NvU16 *)pSpiBuffer;
             for (Index =0; Index < PacketRequest; ++Index)
             {
                 *pOutBuffer++ = (NvU16)(((*(pTxBuffer  )) << 8) |
                                         ((*(pTxBuffer+1))& 0xFF));
                 pTxBuffer += 2;
             }
            return;
        }
    }

    BytesPerPackets = (PacketBitLength + 7)/8;
    PacketRequest = BytesRequested / BytesPerPackets;

    Shift0 = (PacketBitLength & 7);
    if (Shift0)
        MSBMaskData = (0xFF >> (8-Shift0));

    if (BytesPerPackets == 1)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((*(pTxBuffer))& MSBMaskData);
            pTxBuffer++;
        }
        return;
    }

    if (BytesPerPackets == 2)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer))& MSBMaskData) << 8) |
                                    ((*(pTxBuffer+1))));
            pTxBuffer += 2;
        }
        return;
    }

    if (BytesPerPackets == 3)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer)) & MSBMaskData) << 16) |
                                    ((*(pTxBuffer+1)) << 8) |
                                    ((*(pTxBuffer+2))));
            pTxBuffer += 3;
        }
        return;
    }

    if (BytesPerPackets == 4)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer))& MSBMaskData) << 24) |
                                    ((*(pTxBuffer+1)) << 16) |
                                    ((*(pTxBuffer+2)) << 8) |
                                    ((*(pTxBuffer+3))));
            pTxBuffer += 4;
        }
        return;
    }
}

// Similar to MakeMasterSpiBufferFromClientBuffer() except that SPI slave byte order
// is reversed compared to SPI master
static void
MakeSlaveSpiBufferFromClientBuffer(
    NvU8 *pTxBuffer,
    NvU32 *pSpiBuffer,
    NvU32 BytesRequested,
    NvU32 PacketBitLength,
    NvU32 IsPackedMode)
{
    NvU32 Shift0;
    NvU32 MSBMaskData = 0xFF;
    NvU32 BytesPerPackets;
    NvU32 Index;
    NvU32 PacketRequest;

    if (IsPackedMode)
    {
        /* SPI slave byte order matches processor endianness, so memcpy can be used */
        if ((PacketBitLength == 8) || (PacketBitLength == 16))
        {
            NvOsMemcpy(pSpiBuffer, pTxBuffer, BytesRequested);
            return;
        }
    }

    BytesPerPackets = (PacketBitLength + 7)/8;
    PacketRequest = BytesRequested / BytesPerPackets;

    Shift0 = (PacketBitLength & 7);
    if (Shift0)
        MSBMaskData = (0xFF >> (8-Shift0));

    if (BytesPerPackets == 1)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((*(pTxBuffer))& MSBMaskData);
            pTxBuffer++;
        }
        return;
    }

    if (BytesPerPackets == 2)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer+1))& MSBMaskData) << 8) |
                                    ((*(pTxBuffer))));
            pTxBuffer += 2;
        }
        return;
    }

    if (BytesPerPackets == 3)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer+2)) & MSBMaskData) << 16) |
                                    ((*(pTxBuffer+1)) << 8) |
                                    ((*(pTxBuffer))));
            pTxBuffer += 3;
        }
        return;
    }

    if (BytesPerPackets == 4)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            *pSpiBuffer++ = (NvU32)((((*(pTxBuffer+3))& MSBMaskData) << 24) |
                                    ((*(pTxBuffer+2)) << 16) |
                                    ((*(pTxBuffer+1)) << 8) |
                                    ((*(pTxBuffer))));
            pTxBuffer += 4;
        }
        return;
    }
}

static void
MakeMasterClientBufferFromSpiBuffer(
    NvU8 *pRxBuffer,
    NvU32 *pSpiBuffer,
    NvU32 BytesRequested,
    NvU32 PacketBitLength,
    NvU32 IsPackedMode)
{
    NvU32 Shift0;
    NvU32 MSBMaskData = 0xFF;
    NvU32 BytesPerPackets;
    NvU32 Index;
    NvU32 RxData;
    NvU32 PacketRequest;

    NvU8 *pOutBuffer = NULL;

    if (IsPackedMode)
    {
        if (PacketBitLength == 8)
        {
            NvOsMemcpy(pRxBuffer, pSpiBuffer, BytesRequested);
            return;
        }

        BytesPerPackets = (PacketBitLength + 7)/8;
        PacketRequest = BytesRequested / BytesPerPackets;
        if (PacketBitLength == 16)
        {
            pOutBuffer = (NvU8 *)pSpiBuffer;
            for (Index =0; Index < PacketRequest; ++Index)
            {
                *pRxBuffer++  = (NvU8) (*(pOutBuffer+1));
                *pRxBuffer++  = (NvU8) (*(pOutBuffer));
                pOutBuffer += 2;
            }
            return;
        }
    }

    BytesPerPackets = (PacketBitLength + 7)/8;
    PacketRequest = BytesRequested / BytesPerPackets;
    Shift0 = (PacketBitLength & 7);
    if (Shift0)
        MSBMaskData = (0xFF >> (8-Shift0));

    if (BytesPerPackets == 1)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
            *pRxBuffer++ = (NvU8)((*pSpiBuffer++) & MSBMaskData);
        return;
    }

    if (BytesPerPackets == 2)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData >> 8) & MSBMaskData);
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
        }
        return;
    }

    if (BytesPerPackets == 3)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData >> 16)& MSBMaskData);
            *pRxBuffer++ = (NvU8)((RxData >> 8)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
        }
        return;
    }

    if (BytesPerPackets == 4)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData >> 24)& MSBMaskData);
            *pRxBuffer++ = (NvU8)((RxData >> 16)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 8)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
        }
        return;
    }
}

// Similar to MakeMasterClientBufferFromSpiBuffer() except that SPI slave byte order
// is reversed compared to SPI master
static void
MakeSlaveClientBufferFromSpiBuffer(
    NvU8 *pRxBuffer,
    NvU32 *pSpiBuffer,
    NvU32 BytesRequested,
    NvU32 PacketBitLength,
    NvU32 IsPackedMode)
{
    NvU32 Shift0;
    NvU32 MSBMaskData = 0xFF;
    NvU32 BytesPerPackets;
    NvU32 Index;
    NvU32 RxData;
    NvU32 PacketRequest;

    if (IsPackedMode)
    {
        /* SPI slave byte order matches processor endianness, so memcpy can be used */
        if ((PacketBitLength == 8) || (PacketBitLength == 16))
        {
            NvOsMemcpy(pRxBuffer, pSpiBuffer, BytesRequested);
            return;
        }
    }

    BytesPerPackets = (PacketBitLength + 7)/8;
    PacketRequest = BytesRequested / BytesPerPackets;
    Shift0 = (PacketBitLength & 7);
    if (Shift0)
        MSBMaskData = (0xFF >> (8-Shift0));

    if (BytesPerPackets == 1)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
            *pRxBuffer++ = (NvU8)((*pSpiBuffer++) & MSBMaskData);
        return;
    }

    if (BytesPerPackets == 2)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 8) & MSBMaskData);
        }
        return;
    }

    if (BytesPerPackets == 3)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 8)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 16)& MSBMaskData);
        }
        return;
    }

    if (BytesPerPackets == 4)
    {
        for (Index = 0; Index < PacketRequest; ++Index)
        {
            RxData = *pSpiBuffer++;
            *pRxBuffer++ = (NvU8)((RxData) & 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 8)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 16)& 0xFF);
            *pRxBuffer++ = (NvU8)((RxData >> 24)& MSBMaskData);
        }
        return;
    }
}

static NvError
MasterModeReadWriteCpu(
    NvRmSpiHandle hRmSpiSlink,
    NvU8 *pClientRxBuffer,
    NvU8 *pClientTxBuffer,
    NvU32 PacketsRequested,
    NvU32 *pPacketsTransferred,
    NvU32 IsPackedMode,
    NvU32 PacketBitLength)
{
    NvError Error = NvSuccess;
    NvU32 CurrentTransWord;
    NvU32 BufferOffset = 0;
    NvU32 WordsWritten;
    NvU32 MaxPacketPerTrans;
    NvU32 CurrentTransPacket;
    NvU32 PacketsPerWord;
    NvU32 MaxPacketTrans;
    NvBool IsPolling;

    hRmSpiSlink->CurrTransInfo.BytesPerPacket = (PacketBitLength + 7)/8;
    PacketsPerWord = (IsPackedMode)? 4/hRmSpiSlink->CurrTransInfo.BytesPerPacket:1;

    hRmSpiSlink->IsUsingApbDma = NV_FALSE;
    hRmSpiSlink->CurrTransInfo.PacketsPerWord =  PacketsPerWord;

    MaxPacketPerTrans = hRmSpiSlink->CpuBufferSizeInWords*PacketsPerWord;
    hRmSpiSlink->CurrTransInfo.TotalPacketsRemaining = PacketsRequested;

    while (hRmSpiSlink->CurrTransInfo.TotalPacketsRemaining)
    {
        MaxPacketTrans = NV_MIN(hRmSpiSlink->CurrTransInfo.TotalPacketsRemaining, MaxPacketPerTrans);


        // If hw does not support the nonword alined packed mode then
        // Transfer the nearest word alligned packet first with packed mode
        // and then the remaining packet in non packed mode.
        if (hRmSpiSlink->HwRegs.IsNonWordAlignedPackModeSupported)
            CurrentTransWord = (MaxPacketTrans + PacketsPerWord -1)/PacketsPerWord;
        else
            CurrentTransWord = (MaxPacketTrans)/PacketsPerWord;

        if (!CurrentTransWord)
        {
            PacketsPerWord = 1;
            CurrentTransWord = MaxPacketTrans;
            hRmSpiSlink->hHwInterface->HwSetPacketLengthFxn(&hRmSpiSlink->HwRegs,
                PacketBitLength, NV_FALSE);
            hRmSpiSlink->CurrTransInfo.PacketsPerWord = PacketsPerWord;
            IsPackedMode = NV_FALSE;
        }

        CurrentTransPacket = NV_MIN(MaxPacketTrans, CurrentTransWord*PacketsPerWord) ;

        // Select polling if less number of transfer is required.
        if (CurrentTransWord < SLINK_POLLING_HIGH_THRESOLD)
        {
            IsPolling = NV_TRUE;
            hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
                    hRmSpiSlink->CurrentDirection, NV_FALSE);
        }
        else
        {
            IsPolling = NV_FALSE;
            hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
                    hRmSpiSlink->CurrentDirection, NV_TRUE);
        }
        hRmSpiSlink->TxTransferStatus = NvSuccess;
        hRmSpiSlink->RxTransferStatus = NvSuccess;
        hRmSpiSlink->CurrTransInfo.PacketTransferred  = 0;

        if (pClientRxBuffer)
        {
            hRmSpiSlink->CurrTransInfo.pRxBuff = hRmSpiSlink->pRxCpuBuffer;
            hRmSpiSlink->CurrTransInfo.RxPacketsRemaining = CurrentTransPacket;
        }

        if (pClientTxBuffer)
        {
            MakeMasterSpiBufferFromClientBuffer(pClientTxBuffer + BufferOffset,
                hRmSpiSlink->pTxCpuBuffer, CurrentTransPacket*hRmSpiSlink->CurrTransInfo.BytesPerPacket,
                PacketBitLength, IsPackedMode);
            WordsWritten = hRmSpiSlink->hHwInterface->HwWriteInTransmitFifoFxn(
                &hRmSpiSlink->HwRegs, hRmSpiSlink->pTxCpuBuffer, CurrentTransWord);

            hRmSpiSlink->CurrTransInfo.CurrPacketCount =
                    NV_MIN(WordsWritten*PacketsPerWord, CurrentTransPacket);

            hRmSpiSlink->CurrTransInfo.pTxBuff =
                                    hRmSpiSlink->pTxCpuBuffer + WordsWritten;
            hRmSpiSlink->CurrTransInfo.TxPacketsRemaining = CurrentTransPacket -
                                    hRmSpiSlink->CurrTransInfo.CurrPacketCount;
        }
        else
        {
            hRmSpiSlink->CurrTransInfo.CurrPacketCount =
                    NV_MIN(hRmSpiSlink->HwRegs.MaxWordTransfer*PacketsPerWord,
                                        CurrentTransPacket);
        }
        if (!hRmSpiSlink->IsChipSelConfigured)
        {
            hRmSpiSlink->IsCurrentlySwBasedChipSel = 
                hRmSpiSlink->hHwInterface->HwSetChipSelectLevelBasedOnPacketFxn(
                    &hRmSpiSlink->HwRegs, hRmSpiSlink->CurrTransferChipSelId,
                    !hRmSpiSlink->DeviceInfo[hRmSpiSlink->CurrTransferChipSelId].ChipSelectActiveLow,
                    PacketsRequested, PacketsPerWord, NV_FALSE, NV_FALSE);
            hRmSpiSlink->IsChipSelConfigured = NV_TRUE;
        }
        
        hRmSpiSlink->hHwInterface->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs,
                                    hRmSpiSlink->CurrTransInfo.CurrPacketCount);
        hRmSpiSlink->hHwInterface->HwStartTransferFxn(&hRmSpiSlink->HwRegs, NV_TRUE);
#if NV_OAL
        OalMasterSpiSlinkPoll(hRmSpiSlink);
#else
        WaitForTransferCompletion(hRmSpiSlink, NV_WAIT_INFINITE, IsPolling);
#endif
        Error = (hRmSpiSlink->RxTransferStatus)? hRmSpiSlink->RxTransferStatus:
                                    hRmSpiSlink->TxTransferStatus;
        if (Error)
            break;

        if (pClientRxBuffer)
        {
            MakeMasterClientBufferFromSpiBuffer(pClientRxBuffer + BufferOffset,
                hRmSpiSlink->pRxCpuBuffer, CurrentTransPacket*hRmSpiSlink->CurrTransInfo.BytesPerPacket,
                PacketBitLength, IsPackedMode);
        }

        BufferOffset += CurrentTransPacket*hRmSpiSlink->CurrTransInfo.BytesPerPacket;
        hRmSpiSlink->CurrTransInfo.TotalPacketsRemaining -= CurrentTransPacket;
    }

    *pPacketsTransferred = PacketsRequested - hRmSpiSlink->CurrTransInfo.TotalPacketsRemaining;
    return Error;
}

static NvError MasterModeReadWriteDma(
    NvRmSpiHandle hRmSpiSlink,
    NvU8 *pClientRxBuffer,
    NvU8 *pClientTxBuffer,
    NvU32 PacketsRequested,
    NvU32 *pPacketsTransferred,
    NvU32 IsPackedMode,
    NvU32 PacketBitLength)
{
    NvError Error = NvSuccess;
    NvU32 CurrentTransWord;
    NvU32 BufferOffset = 0;
    NvU32 BytesPerPacket = (PacketBitLength +7)/8;
    NvU32 MaxPacketPerTrans;
    NvU32 CurrentTransPacket;
    NvU32 PacketsRemaining;
    NvU32 PacketsPerWord = (IsPackedMode)?4/BytesPerPacket:1;
    NvU32 TriggerLevel;
    NvU32 MaxPacketTransPossible;
    NvU32 PackSend = 0;
    NvU8 *pReadReqCpuBuffer = NULL;
    NvU8 *pWriteReqCpuBuffer = NULL;
    NvU32 WrittenWord;
    NvBool IsOnlyUseSWCS;

    hRmSpiSlink->IsUsingApbDma = NV_TRUE;
    hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
            hRmSpiSlink->CurrentDirection, NV_TRUE);

    hRmSpiSlink->CurrTransInfo.PacketsPerWord =  PacketsPerWord;


    MaxPacketPerTrans = (hRmSpiSlink->DmaBufferSize >> 2)*PacketsPerWord;
    PacketsRemaining = PacketsRequested;
    while (PacketsRemaining)
    {
        MaxPacketTransPossible = NV_MIN(PacketsRemaining, MaxPacketPerTrans);

        // If hw does not support the nonword alined packed mode then
        // Transfer the nearest word alligned packet first with packed mode
        // and then the remaining packet in non packed mode.
        if (hRmSpiSlink->HwRegs.IsNonWordAlignedPackModeSupported)
            CurrentTransWord = (MaxPacketTransPossible + PacketsPerWord -1)/PacketsPerWord;
        else
            CurrentTransWord = (MaxPacketTransPossible)/PacketsPerWord;

        // For the non multiple of the 4 bytes, it can do the transfer using the
        // cpu for the remaining transfer.
        if (!CurrentTransWord)
        {
            if (pClientRxBuffer)
                pReadReqCpuBuffer = (pClientRxBuffer + BufferOffset);
            if (pClientTxBuffer)
                pWriteReqCpuBuffer = (pClientTxBuffer + BufferOffset);

            hRmSpiSlink->hHwInterface->HwSetPacketLengthFxn(&hRmSpiSlink->HwRegs,
                                PacketBitLength, NV_FALSE);
            Error = MasterModeReadWriteCpu(hRmSpiSlink,  pReadReqCpuBuffer,
                        pWriteReqCpuBuffer, MaxPacketTransPossible,
                        &PackSend, NV_FALSE, PacketBitLength);
            PacketsRemaining -= PackSend;
            break;
        }
        if (hRmSpiSlink->HwRegs.IsNonWordAlignedPackModeSupported)
            CurrentTransPacket = MaxPacketTransPossible;
        else
            CurrentTransPacket = CurrentTransWord*PacketsPerWord;

        hRmSpiSlink->TxTransferStatus = NvSuccess;
        hRmSpiSlink->RxTransferStatus = NvSuccess;
        hRmSpiSlink->CurrTransInfo.PacketTransferred  = 0;
        hRmSpiSlink->CurrTransInfo.CurrPacketCount = CurrentTransPacket;

        if (pClientRxBuffer)
            hRmSpiSlink->CurrTransInfo.RxPacketsRemaining = CurrentTransPacket;

        if (!hRmSpiSlink->IsChipSelConfigured)
        {
            IsOnlyUseSWCS = (CurrentTransPacket == PacketsRequested)? NV_FALSE: NV_TRUE;
            hRmSpiSlink->IsCurrentlySwBasedChipSel = 
                hRmSpiSlink->hHwInterface->HwSetChipSelectLevelBasedOnPacketFxn(
                    &hRmSpiSlink->HwRegs, hRmSpiSlink->CurrTransferChipSelId,
                    !hRmSpiSlink->DeviceInfo[hRmSpiSlink->CurrTransferChipSelId].ChipSelectActiveLow,
                    PacketsRequested, PacketsPerWord, NV_TRUE, IsOnlyUseSWCS);
            hRmSpiSlink->IsChipSelConfigured = NV_TRUE;
        }

        hRmSpiSlink->hHwInterface->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs,
                                            CurrentTransPacket);

        TriggerLevel = (CurrentTransWord  & 0x3)? 4: 16;
        hRmSpiSlink->hHwInterface->HwSetTriggerLevelFxn(&hRmSpiSlink->HwRegs,
                                            SerialHwFifo_Both , TriggerLevel);

        if (pClientTxBuffer)
        {
            MakeMasterSpiBufferFromClientBuffer(pClientTxBuffer + BufferOffset,
                        hRmSpiSlink->pTxDmaBuffer, CurrentTransPacket*BytesPerPacket,
                        PacketBitLength, IsPackedMode);
            hRmSpiSlink->CurrTransInfo.pTxBuff = hRmSpiSlink->pTxDmaBuffer;
            hRmSpiSlink->TxDmaReq.TransferSize = CurrentTransWord *4;

            // If transfer word is more than fifo size the use the dma
            // otherwise direct write into the fifo.
            if (CurrentTransWord >= hRmSpiSlink->HwRegs.MaxWordTransfer)
            {
                Error = NvRmDmaStartDmaTransfer(hRmSpiSlink->hRmTxDma,
                                &hRmSpiSlink->TxDmaReq, NvRmDmaDirection_Forward, 0, NULL);
                // Wait till fifo full if the transfer size is more than fifo size
                if (!Error)
                {
                    do
                    {
                        if (hRmSpiSlink->hHwInterface->HwIsTransmitFifoFull(&hRmSpiSlink->HwRegs))
                            break;
                    } while(1);
                }
            }
            else
            {
                WrittenWord = hRmSpiSlink->hHwInterface->HwWriteInTransmitFifoFxn(
                                            &hRmSpiSlink->HwRegs,
                                                hRmSpiSlink->CurrTransInfo.pTxBuff,
                                                CurrentTransWord);
                if (WrittenWord !=  CurrentTransWord)
                {
                    NV_ASSERT(WrittenWord == CurrentTransWord);
                    Error = NvError_Timeout;
                }
            }
        }

        if ((!Error) && (pClientRxBuffer))
        {
            hRmSpiSlink->RxDmaReq.TransferSize = CurrentTransWord *4;
            Error = NvRmDmaStartDmaTransfer(hRmSpiSlink->hRmRxDma, &hRmSpiSlink->RxDmaReq,
                                        NvRmDmaDirection_Forward, 0, NULL);
            if ((Error) && (pClientTxBuffer))
                NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
        }

        if (!Error)
            hRmSpiSlink->hHwInterface->HwStartTransferFxn(&hRmSpiSlink->HwRegs, NV_TRUE);

        if (!Error)
            WaitForTransferCompletion(hRmSpiSlink, NV_WAIT_INFINITE, NV_FALSE);

        Error = (hRmSpiSlink->RxTransferStatus)? hRmSpiSlink->RxTransferStatus:
                                    hRmSpiSlink->TxTransferStatus;
        if (Error)
        {
            if (pClientRxBuffer)
                NvRmDmaAbort(hRmSpiSlink->hRmRxDma);
            if (pClientTxBuffer)
                NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
            break;
        }
        if (pClientRxBuffer)
        {
            MakeMasterClientBufferFromSpiBuffer(pClientRxBuffer + BufferOffset,
                    hRmSpiSlink->pRxDmaBuffer, CurrentTransPacket*BytesPerPacket,
                    PacketBitLength, IsPackedMode);
        }

        BufferOffset += CurrentTransPacket*BytesPerPacket;
        PacketsRemaining -= CurrentTransPacket;
    }

    hRmSpiSlink->hHwInterface->HwSetDataFlowFxn(&hRmSpiSlink->HwRegs,
                                    hRmSpiSlink->CurrentDirection, NV_FALSE);
    hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
                                    hRmSpiSlink->CurrentDirection, NV_FALSE);

    *pPacketsTransferred = PacketsRequested - PacketsRemaining;
    return Error;
}
static NvError SlaveModeSpiStartReadWriteCpu(
    NvRmSpiHandle hRmSpiSlink,
    NvBool IsReadTransfer,
    NvU8 *pClientTxBuffer,
    NvU32 PacketsRequested,
    NvU32 IsPackedMode,
    NvU32 PacketBitLength)
{
    NvError Error = NvSuccess;
    NvU32 BytesPerPacket;
    NvU32 WordsWritten;
    NvU32 PacketsPerWord;
    NvU32 TotalWordsRequested;

    BytesPerPacket = (PacketBitLength + 7)/8;
    PacketsPerWord = (IsPackedMode)? 4/BytesPerPacket: 1;
    TotalWordsRequested = (PacketsRequested + PacketsPerWord -1)/PacketsPerWord;

    hRmSpiSlink->IsUsingApbDma = NV_FALSE;

    hRmSpiSlink->hHwInterface->HwSetPacketLengthFxn(&hRmSpiSlink->HwRegs,
                        PacketBitLength, IsPackedMode);

    hRmSpiSlink->CurrTransInfo.PacketsPerWord =  PacketsPerWord;
    hRmSpiSlink->CurrTransInfo.BytesPerPacket =  BytesPerPacket;
    hRmSpiSlink->CurrTransInfo.PacketBitLength = PacketBitLength;
    hRmSpiSlink->CurrTransInfo.IsPackedMode =  IsPackedMode;

    hRmSpiSlink->TxTransferStatus = NvSuccess;
    hRmSpiSlink->RxTransferStatus = NvSuccess;

    hRmSpiSlink->CurrTransInfo.PacketTransferred  = 0;

    hRmSpiSlink->CurrTransInfo.pRxBuff =
                            (IsReadTransfer)? hRmSpiSlink->pRxCpuBuffer: NULL;
    hRmSpiSlink->CurrTransInfo.RxPacketsRemaining =
                            (IsReadTransfer)? PacketsRequested: 0;

    hRmSpiSlink->CurrTransInfo.PacketRequested = PacketsRequested;

    hRmSpiSlink->CurrTransInfo.pTxBuff = NULL;
    hRmSpiSlink->CurrTransInfo.TxPacketsRemaining = 0;

    WordsWritten = hRmSpiSlink->HwRegs.MaxWordTransfer;

    if (pClientTxBuffer)
    {
        MakeSlaveSpiBufferFromClientBuffer(pClientTxBuffer, hRmSpiSlink->pTxCpuBuffer,
                    PacketsRequested*BytesPerPacket, PacketBitLength,
                    IsPackedMode);
        WordsWritten = hRmSpiSlink->hHwInterface->HwWriteInTransmitFifoFxn(
                        &hRmSpiSlink->HwRegs, hRmSpiSlink->pTxCpuBuffer,
                        TotalWordsRequested);

        hRmSpiSlink->CurrTransInfo.CurrPacketCount =
                NV_MIN(WordsWritten*PacketsPerWord, PacketsRequested);
        hRmSpiSlink->CurrTransInfo.pTxBuff =
                                    hRmSpiSlink->pTxCpuBuffer + WordsWritten;
        hRmSpiSlink->CurrTransInfo.TxPacketsRemaining = PacketsRequested -
                                hRmSpiSlink->CurrTransInfo.CurrPacketCount;
    }
    else
    {
        hRmSpiSlink->CurrTransInfo.CurrPacketCount =
                    NV_MIN(WordsWritten*PacketsPerWord, PacketsRequested);
    }

    hRmSpiSlink->hHwInterface->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs,
                    hRmSpiSlink->CurrTransInfo.CurrPacketCount);

    hRmSpiSlink->hHwInterface->HwStartTransferFxn(&hRmSpiSlink->HwRegs, NV_TRUE);

    return Error;
}

static NvError SlaveModeSpiStartReadWriteDma(
    NvRmSpiHandle hRmSpiSlink,
    NvBool IsReadTransfer,
    NvU8 *pClientTxBuffer,
    NvU32 PacketsRequested,
    NvU32 IsPackedMode,
    NvU32 PacketBitLength)
{
    NvError Error = NvSuccess;
    NvU32 CurrentTransWord;
    NvU32 BytesPerPacket;
    NvU32 CurrentTransPacket;
    NvU32 PacketsPerWord;
    NvU32 TriggerLevel;
    NvU32 TotalWordsRequested;
    NvU32 NewBufferSize;

    BytesPerPacket = (PacketBitLength + 7)/8;
    PacketsPerWord = (IsPackedMode)? 4/BytesPerPacket: 1;
    TotalWordsRequested = (PacketsRequested + PacketsPerWord -1)/PacketsPerWord;

    hRmSpiSlink->IsUsingApbDma = NV_TRUE;

    // Create the buffer if the required size of the buffer is not available.
    if (hRmSpiSlink->DmaBufferSize < (TotalWordsRequested*4))
    {
        DestroyDmaTransferBuffer(hRmSpiSlink->hRmMemory,
                hRmSpiSlink->pRxDmaBuffer, hRmSpiSlink->pTxDmaBuffer,
                hRmSpiSlink->DmaBufferSize);
        hRmSpiSlink->hRmMemory = NULL;
        hRmSpiSlink->pRxDmaBuffer = NULL;
        hRmSpiSlink->DmaRxBuffPhysAdd = 0;

        // Better to findout the neearest 2powern
        NewBufferSize = NV_MAX(hRmSpiSlink->DmaBufferSize, (TotalWordsRequested*4));
        Error = CreateDmaTransferBuffer(hRmSpiSlink->hDevice, &hRmSpiSlink->hRmMemory,
                &hRmSpiSlink->DmaRxBuffPhysAdd, (void **)&hRmSpiSlink->pRxDmaBuffer,
                &hRmSpiSlink->DmaTxBuffPhysAdd, (void **)&hRmSpiSlink->pTxDmaBuffer,
                NewBufferSize);

        if (Error)
        {
            hRmSpiSlink->DmaBufferSize = 0;
            return Error;
        }
        hRmSpiSlink->RxDmaReq.DestinationBufferPhyAddress = hRmSpiSlink->DmaRxBuffPhysAdd;
        hRmSpiSlink->TxDmaReq.SourceBufferPhyAddress = hRmSpiSlink->DmaTxBuffPhysAdd;
        hRmSpiSlink->DmaBufferSize = NewBufferSize;
    }

    hRmSpiSlink->CurrTransInfo.PacketsPerWord =  PacketsPerWord;
    hRmSpiSlink->CurrTransInfo.BytesPerPacket =  BytesPerPacket;
    hRmSpiSlink->CurrTransInfo.PacketBitLength = PacketBitLength;
    hRmSpiSlink->CurrTransInfo.IsPackedMode =  IsPackedMode;

    CurrentTransPacket = NV_MIN((TotalWordsRequested*PacketsPerWord), PacketsRequested);

    hRmSpiSlink->CurrTransInfo.PacketTransferred  = 0;
    hRmSpiSlink->CurrTransInfo.RxPacketsRemaining = 0;
    hRmSpiSlink->CurrTransInfo.pRxBuff = NULL;
    hRmSpiSlink->CurrTransInfo.CurrPacketCount = CurrentTransPacket;
    hRmSpiSlink->CurrTransInfo.PacketRequested = CurrentTransPacket;
    hRmSpiSlink->TxTransferStatus = NvSuccess;
    hRmSpiSlink->RxTransferStatus = NvSuccess;

    hRmSpiSlink->CurrTransInfo.pTxBuff = NULL;

    CurrentTransWord = (CurrentTransPacket + PacketsPerWord -1)/PacketsPerWord;

    TriggerLevel = (CurrentTransWord  & 0x3)? 4: 16;
    hRmSpiSlink->hHwInterface->HwSetTriggerLevelFxn(&hRmSpiSlink->HwRegs,
                            SerialHwFifo_Both , TriggerLevel);

    hRmSpiSlink->hHwInterface->HwSetDmaTransferSizeFxn(&hRmSpiSlink->HwRegs,
                                    CurrentTransPacket);
    if (pClientTxBuffer)
    {
        MakeSlaveSpiBufferFromClientBuffer(pClientTxBuffer, hRmSpiSlink->pTxDmaBuffer,
                    CurrentTransPacket*BytesPerPacket,
                    PacketBitLength, IsPackedMode);
        hRmSpiSlink->CurrTransInfo.pTxBuff = hRmSpiSlink->pTxDmaBuffer;
        hRmSpiSlink->TxDmaReq.TransferSize = CurrentTransWord *4;
        Error = NvRmDmaStartDmaTransfer(hRmSpiSlink->hRmTxDma, &hRmSpiSlink->TxDmaReq,
                                        NvRmDmaDirection_Forward, 0, NULL);
        do
        {
            if (hRmSpiSlink->hHwInterface->HwIsTransmitFifoFull(&hRmSpiSlink->HwRegs))
                break;
        } while(1);
    }

    if ((!Error) && (IsReadTransfer))
    {
        hRmSpiSlink->RxDmaReq.TransferSize = CurrentTransWord *4;
        hRmSpiSlink->CurrTransInfo.RxPacketsRemaining = CurrentTransPacket;
        hRmSpiSlink->CurrTransInfo.pRxBuff = hRmSpiSlink->pRxDmaBuffer;

        Error = NvRmDmaStartDmaTransfer(hRmSpiSlink->hRmRxDma, &hRmSpiSlink->RxDmaReq,
                        NvRmDmaDirection_Forward, 0, NULL);
        if ((Error) && (pClientTxBuffer))
            NvRmDmaAbort(hRmSpiSlink->hRmTxDma);
    }

    if (!Error)
        hRmSpiSlink->hHwInterface->HwStartTransferFxn(&hRmSpiSlink->HwRegs, NV_TRUE);
    return Error;
}

static NvError SlaveModeSpiCompleteReadWrite(
    NvRmSpiHandle hRmSpiSlink,
    NvU8 *pClientRxBuffer,
    NvU32 *pBytesTransferred,
    NvU32 TimeoutMs)
{
    NvError Error = NvSuccess;
    NvU32 TransferdPacket;
    NvU32 ReqSizeInBytes;
    NvU32 *pRxBuffer = NULL;

    Error = WaitForTransferCompletion(hRmSpiSlink, TimeoutMs, NV_FALSE);
    if (Error == NvError_Timeout)
    {
        TransferdPacket = hRmSpiSlink->CurrTransInfo.PacketTransferred;
    }
    else
    {
        Error = (hRmSpiSlink->RxTransferStatus)? hRmSpiSlink->RxTransferStatus:
                                    hRmSpiSlink->TxTransferStatus;
        if (Error)
            TransferdPacket = hRmSpiSlink->CurrTransInfo.PacketTransferred;
        else
            TransferdPacket = hRmSpiSlink->CurrTransInfo.PacketRequested;
    }
    ReqSizeInBytes = NV_MIN(TransferdPacket, hRmSpiSlink->CurrTransInfo.PacketRequested)
                            *hRmSpiSlink->CurrTransInfo.BytesPerPacket;

    if (pClientRxBuffer)
    {
        pRxBuffer = (hRmSpiSlink->IsUsingApbDma)?hRmSpiSlink->pRxDmaBuffer:
                            hRmSpiSlink->pRxCpuBuffer;
        MakeSlaveClientBufferFromSpiBuffer(pClientRxBuffer,
                        pRxBuffer, ReqSizeInBytes,
                        hRmSpiSlink->CurrTransInfo.PacketBitLength,
                        hRmSpiSlink->CurrTransInfo.IsPackedMode);
    }

    *pBytesTransferred = ReqSizeInBytes;
    hRmSpiSlink->hHwInterface->HwSetInterruptSourceFxn(&hRmSpiSlink->HwRegs,
                            hRmSpiSlink->CurrentDirection, NV_FALSE);
    hRmSpiSlink->hHwInterface->HwSetDataFlowFxn(&hRmSpiSlink->HwRegs,
                            hRmSpiSlink->CurrentDirection, NV_FALSE);
    hRmSpiSlink->CurrentDirection = SerialHwDataFlow_None;
    return Error;
}


static void
InitSlinkInterface(
    NvRmDeviceHandle hDevice,
    HwInterface *pSlinkInterface)
{
    static SlinkCapabilities s_SpiCap[2];
    SlinkCapabilities *pSpiCap = NULL;
    static NvRmModuleCapability s_SpiCapList[] =
        {
            {1, 0, 0, &s_SpiCap[0]}, // AP15 version 1.0
            {1, 1, 0, &s_SpiCap[1]}, // AP20 version 1.1
        };

    // (AP15) version 1.0
    s_SpiCap[0].MajorVersion = 1;
    s_SpiCap[0].MinorVersion = 0;

    // (AP20) version 1.1
    s_SpiCap[1].MajorVersion = 1;
    s_SpiCap[1].MinorVersion = 1;

    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hDevice, NVRM_MODULE_ID(NvRmModuleID_Slink, 0),
                        s_SpiCapList, NV_ARRAY_SIZE(s_SpiCapList), (void**)&(pSpiCap)));

    NvRmPrivSpiSlinkInitSlinkInterface(&s_SlinkHwInterface);
    if ((pSpiCap->MajorVersion == 1) && (pSpiCap->MinorVersion == 0))
    {
        NvRmPrivSpiSlinkInitSlinkInterface_v1_0(&s_SlinkHwInterface);
    }
    else // 1.1
    {
        NvRmPrivSpiSlinkInitSlinkInterface_v1_1(&s_SlinkHwInterface);
    }
}

/**
 * Initialize the spi info structure.
 * Thread safety: Caller responsibity.
 */
NvError NvRmPrivSpiSlinkInit(NvRmDeviceHandle hDevice)
{
    NvError e;
    NvU32 Index;

    NV_ASSERT(NvRmModuleGetNumInstances(hDevice, NvRmModuleID_Spi) <= MAX_SPI_CONTROLLERS);
    NV_ASSERT(NvRmModuleGetNumInstances(hDevice, NvRmModuleID_Slink) <= MAX_SLINK_CONTROLLERS);

    NvRmPrivSpiSlinkInitSpiInterface(&s_SpiHwInterface);
    InitSlinkInterface(hDevice, &s_SlinkHwInterface);

    // Initialize all the parameters.
    s_SpiSlinkInfo.hDevice = hDevice;

    for (Index = 0; Index < MAX_SPI_SLINK_INSTANCE; ++Index)
        s_SpiSlinkInfo.hSpiSlinkChannelList[Index] = NULL;

    // Create the mutex to access the spi information.
    NV_CHECK_ERROR(NvOsMutexCreate(&s_SpiSlinkInfo.hChannelAccessMutex));
    return NvSuccess;
}

/**
 * Destroy all the spi struture information. It frees all the allocated resource.
 * Thread safety: Caller responsibity.
 */
void NvRmPrivSpiSlinkDeInit(void)
{
    NvU32 Index;

    // Free all allocations.
    NvOsMutexLock(s_SpiSlinkInfo.hChannelAccessMutex);
    for (Index = 0; Index < MAX_SPI_SLINK_INSTANCE; ++Index)
    {
        if (s_SpiSlinkInfo.hSpiSlinkChannelList[Index] != NULL)
        {
            DestroySpiSlinkChannelHandle(s_SpiSlinkInfo.hSpiSlinkChannelList[Index]);
            s_SpiSlinkInfo.hSpiSlinkChannelList[Index] = NULL;
        }
    }
    NvOsMutexUnlock(s_SpiSlinkInfo.hChannelAccessMutex);

    NvOsMutexDestroy(s_SpiSlinkInfo.hChannelAccessMutex);
    s_SpiSlinkInfo.hChannelAccessMutex = NULL;
    s_SpiSlinkInfo.hDevice = NULL;
}

/**
 * Open the handle for the spi.
 */
NvError
NvRmSpiOpen(
    NvRmDeviceHandle hRmDevice,
    NvU32 IoModule,
    NvU32 InstanceId,
    NvBool IsMasterMode,
    NvRmSpiHandle * phRmSpi)
{
    NvError Error = NvSuccess;
    NvRmSpiHandle hRmSpiSlink = NULL;
    NvU32 ContInstanceId;
    NvBool IsSpiChannel;

    NV_ASSERT(phRmSpi);
    NV_ASSERT(hRmDevice);

    *phRmSpi = NULL;

    IsSpiChannel = (IoModule == NvOdmIoModule_Sflash)? NV_TRUE: NV_FALSE;

    // SPI controller does not support the slave mode
    if ((IsSpiChannel) && (!IsMasterMode))
        return NvError_NotSupported;

    // 0 to (MAX_SPI_CONTROLLERS-1) will be the spi handles and then
    // slink handles.
    ContInstanceId = (IsSpiChannel)? InstanceId: (MAX_SPI_CONTROLLERS + InstanceId);

    // Lock the spi info mutex access.
    NvOsMutexLock(s_SpiSlinkInfo.hChannelAccessMutex);

    if (s_SpiSlinkInfo.hSpiSlinkChannelList[ContInstanceId] == NULL)
    {
        Error = CreateSpiSlinkChannelHandle(hRmDevice, IsSpiChannel,
                    InstanceId, IsMasterMode, &hRmSpiSlink);
        if (Error)
            goto FuncExit;
    }
    else
    {
        // If the handle is not in master mode then not sharing across the
        // client.
        if (IsMasterMode)
        {
            hRmSpiSlink = s_SpiSlinkInfo.hSpiSlinkChannelList[ContInstanceId];
            if (hRmSpiSlink->IsMasterMode)
            {
                hRmSpiSlink->OpenCount++;
            }
            else
            {
                Error = NvError_AlreadyAllocated;
                goto FuncExit;
            }
        }
        else
        {
            Error = NvError_AlreadyAllocated;
            goto FuncExit;
        }
    }
    *phRmSpi = hRmSpiSlink;

FuncExit:
    NvOsMutexUnlock(s_SpiSlinkInfo.hChannelAccessMutex);
    return Error;
}

/**
 * Close the spi handle.
 */
void NvRmSpiClose(NvRmSpiHandle hRmSpi)
{
    if (hRmSpi)
    {
        NvOsMutexLock(s_SpiSlinkInfo.hChannelAccessMutex);
        hRmSpi->OpenCount--;
        if (!hRmSpi->OpenCount)
            DestroySpiSlinkChannelHandle(hRmSpi);

        NvOsMutexUnlock(s_SpiSlinkInfo.hChannelAccessMutex);
    }
}

void NvRmSpiMultipleTransactions(
    NvRmSpiHandle hRmSpi,
    NvU32 SpiPinMap,
    NvU32 ChipSelectId,
    NvU32 ClockSpeedInKHz,
    NvU32 PacketSizeInBits,
    NvRmSpiTransactionInfo *t,
    NvU32 NumOfTransactions)
{
    NvError Error = NvSuccess;
    NvBool IsPackedMode;
    NvU32 BytesPerPacket;
    NvU32 PacketsTransferred;
    NvU32 PacketsPerWord;
    NvU32 TotalPacketsRequsted;
    NvU32 TotalWordsRequested;
    NvU32 i;
    NvRmDmaModuleID DmaModuleId;
    NvRmSpiTransactionInfo *pTrans = t;
    NvU32 TotalTransByte = 0;

    NV_ASSERT(hRmSpi);
    NV_ASSERT((PacketSizeInBits > 0) && (PacketSizeInBits <= 32));
    NV_ASSERT(hRmSpi->IsMasterMode);

    // Chip select should be supported by the odm.
    NV_ASSERT(hRmSpi->IsChipSelSupported[ChipSelectId]);

    // Proper spi pin map if it is multiplexed otherwise 0.
    NV_ASSERT(((SpiPinMap) && (hRmSpi->SpiPinMap == NvOdmSpiPinMap_Multiplexed)) ||
               ((!SpiPinMap) && (hRmSpi->SpiPinMap != NvOdmSpiPinMap_Multiplexed)));

    // Select Packed mode for the 8/16 bit length.
    BytesPerPacket = (PacketSizeInBits + 7)/8;
    IsPackedMode = ((PacketSizeInBits == 8) || ((PacketSizeInBits == 16)));
    PacketsPerWord =  (IsPackedMode)? 4/BytesPerPacket: 1;

    // Lock the channel access by other client till this client finishes the ops
    NvOsMutexLock(hRmSpi->hChannelAccessMutex);

    hRmSpi->CurrTransferChipSelId = ChipSelectId;
    
    // Enable Power/Clock.
    Error = SetPowerControl(hRmSpi, NV_TRUE);
    if (Error != NvSuccess)
        goto cleanup;

    // Boost frequency if the total bytes requetsed is more than thresold.
    for (i=0; i< NumOfTransactions; i++, pTrans++)
    {
        if (!((pTrans->rxBuffer || pTrans->txBuffer) && pTrans->len))
            continue;
        TotalTransByte += pTrans->len;
    }

    BoostFrequency(hRmSpi, NV_TRUE, TotalTransByte);

    hRmSpi->CurrTransInfo.PacketsPerWord = PacketsPerWord;
    if (SpiPinMap)
    {
        NvRmPinMuxConfigSelect(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                                                hRmSpi->InstanceId, SpiPinMap);
        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                                        hRmSpi->InstanceId, SpiPinMap, NV_FALSE);
    }
    else
    {
        if (hRmSpi->IsIdleSignalTristate)
            NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_FALSE);
    }

    if (!Error)
        Error = SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz,
                       NV_TRUE, NV_TRUE);
    if (Error)
        goto cleanup;

    hRmSpi->hHwInterface->HwSetPacketLengthFxn(&hRmSpi->HwRegs,
        PacketSizeInBits, IsPackedMode);

    for (i=0; i< NumOfTransactions; i++, t++)
    {
        if (!((t->rxBuffer || t->txBuffer) && t->len))
            continue;

        hRmSpi->CurrTransInfo.pRxBuff = NULL;
        hRmSpi->CurrTransInfo.RxPacketsRemaining = 0;
        hRmSpi->CurrTransInfo.pTxBuff = NULL;
        hRmSpi->CurrTransInfo.TxPacketsRemaining = 0;

        /* If not packed mode, packet == word */
        TotalPacketsRequsted = t->len/BytesPerPacket;
        TotalWordsRequested = (TotalPacketsRequsted + PacketsPerWord -1)/PacketsPerWord;
        NV_ASSERT((t->len % BytesPerPacket) == 0);
        NV_ASSERT(TotalPacketsRequsted);

        // Allocate the dma here if transaction size is more than cpu based
        // transaction thresold.
        if ((TotalWordsRequested > hRmSpi->HwRegs.MaxWordTransfer) &&
                (hRmSpi->DmaBufferSize) &&
                (!hRmSpi->IsApbDmaAllocated))
        {
            hRmSpi->TransCountFromLastDmaUsage = 0;
            hRmSpi->IsApbDmaAllocated = NV_TRUE;
            DmaModuleId = (hRmSpi->IsSpiChannel)?NvRmDmaModuleID_Spi: NvRmDmaModuleID_Slink;
            Error = NvRmDmaAllocate(hRmSpi->hDevice, &hRmSpi->hRmRxDma,
                             NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                             hRmSpi->InstanceId);
            if (!Error)
            {
                Error = NvRmDmaAllocate(hRmSpi->hDevice, &hRmSpi->hRmTxDma,
                                 NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                                 hRmSpi->InstanceId);
                if (Error)
                    NvRmDmaFree(hRmSpi->hRmRxDma);
            }
            if (Error)
            {
                hRmSpi->hRmRxDma = NULL;
                hRmSpi->hRmTxDma = NULL;
                hRmSpi->IsApbDmaAllocated = NV_FALSE;
                Error = NvSuccess;
            }
        }

        hRmSpi->CurrentDirection = SerialHwDataFlow_None;
        if (t->txBuffer)
            hRmSpi->CurrentDirection |= SerialHwDataFlow_Tx;
        if (t->rxBuffer)
            hRmSpi->CurrentDirection |= SerialHwDataFlow_Rx;
        hRmSpi->hHwInterface->HwSetDataFlowFxn(&hRmSpi->HwRegs,
            hRmSpi->CurrentDirection, NV_TRUE);

        if ((!hRmSpi->IsApbDmaAllocated) ||
            (TotalWordsRequested <= hRmSpi->HwRegs.MaxWordTransfer))
        {
            hRmSpi->TransCountFromLastDmaUsage++;
            Error = MasterModeReadWriteCpu(hRmSpi, t->rxBuffer, t->txBuffer,
                TotalPacketsRequsted, &PacketsTransferred,
                IsPackedMode, PacketSizeInBits);
        }
        else
        {
            hRmSpi->TransCountFromLastDmaUsage = 0;
            Error = MasterModeReadWriteDma(hRmSpi, t->rxBuffer, t->txBuffer,
                TotalPacketsRequsted, &PacketsTransferred,
                IsPackedMode, PacketSizeInBits);
        }
        hRmSpi->hHwInterface->HwSetDataFlowFxn(&hRmSpi->HwRegs,
            hRmSpi->CurrentDirection, NV_FALSE);
    }
    hRmSpi->CurrentDirection = SerialHwDataFlow_None;
    (void)SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz,
                NV_FALSE, NV_TRUE);

cleanup:

    //  Re-tristate multi-plexed controllers, and re-multiplex the controller.
    if (SpiPinMap)
    {
        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, SpiPinMap, NV_TRUE);

        NvRmPinMuxConfigSelect(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, hRmSpi->SpiPinMap);
    }
    else
    {
        if (hRmSpi->IsIdleSignalTristate)
            NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_TRUE);
    }
    if ((hRmSpi->IsApbDmaAllocated) &&
            (hRmSpi->TransCountFromLastDmaUsage > MAX_DMA_HOLD_TIME))
    {
        NvRmDmaFree(hRmSpi->hRmRxDma);
        NvRmDmaFree(hRmSpi->hRmTxDma);
        hRmSpi->hRmRxDma = NULL;
        hRmSpi->hRmTxDma = NULL;
        hRmSpi->IsApbDmaAllocated = NV_FALSE;
    }

    BoostFrequency(hRmSpi, NV_FALSE, 0);
    SetPowerControl(hRmSpi, NV_FALSE);
    NvOsMutexUnlock(hRmSpi->hChannelAccessMutex);
    NV_ASSERT(Error == NvSuccess);
}

/**
 * Perform the data transfer.
 */
void NvRmSpiTransaction(
    NvRmSpiHandle hRmSpi,
    NvU32 SpiPinMap,
    NvU32 ChipSelectId,
    NvU32 ClockSpeedInKHz,
    NvU8 *pReadBuffer,
    NvU8 *pWriteBuffer,
    NvU32 BytesRequested,
    NvU32 PacketSizeInBits)
{
    NvError Error = NvSuccess;
    NvBool IsPackedMode;
    NvU32 BytesPerPackets;
    NvU32 PacketsTransferred;
    NvU32 PacketsPerWord;
    NvU32 TotalPacketsRequsted;
    NvU32 TotalWordsRequested;
    NvRmDmaModuleID DmaModuleId;

    NV_ASSERT(hRmSpi);
    NV_ASSERT(pReadBuffer || pWriteBuffer);

    // Packet size should be 1 to 32..
    NV_ASSERT((PacketSizeInBits > 0) && (PacketSizeInBits <= 32));

    NV_ASSERT(hRmSpi->IsMasterMode);

    // Bytes requested should be  multiple of of bytes per packets.
    BytesPerPackets = (PacketSizeInBits + 7)/8;
    TotalPacketsRequsted = BytesRequested/BytesPerPackets;
    NV_ASSERT((BytesRequested % BytesPerPackets) == 0);
    NV_ASSERT(TotalPacketsRequsted);

    // Chip select should be supported by the odm.
    NV_ASSERT(hRmSpi->IsChipSelSupported[ChipSelectId]);

    // Proper spi pin map if it is multiplexed otherwise 0.
    NV_ASSERT(((SpiPinMap) && (hRmSpi->SpiPinMap == NvOdmSpiPinMap_Multiplexed)) ||
               ((!SpiPinMap) && (hRmSpi->SpiPinMap != NvOdmSpiPinMap_Multiplexed)));

    // Select Packed mode for the 8/16 bit length.
    IsPackedMode = ((PacketSizeInBits == 8) || ((PacketSizeInBits == 16)));
    PacketsPerWord =  (IsPackedMode)? 4/BytesPerPackets: 1;
    TotalWordsRequested = (TotalPacketsRequsted + PacketsPerWord -1)/PacketsPerWord;

#if !NV_OAL
    // Lock the channel access by other client till this client finishes the ops
    NvOsMutexLock(hRmSpi->hChannelAccessMutex);

    hRmSpi->CurrTransferChipSelId = ChipSelectId;
    // Enable Power/Clock.
    Error = SetPowerControl(hRmSpi, NV_TRUE);
    if (Error != NvSuccess)
        goto cleanup;
    BoostFrequency(hRmSpi, NV_TRUE, BytesRequested);

#else
    hRmSpi->CurrTransferChipSelId = ChipSelectId;
#endif
    hRmSpi->CurrTransInfo.PacketsPerWord = PacketsPerWord;

    // Enable the transmit if the Tx buffer is supplied.
    hRmSpi->CurrentDirection = (pWriteBuffer)?SerialHwDataFlow_Tx: SerialHwDataFlow_None;

    // Enable the receive if the Rx buffer is supplied.
    if (pReadBuffer)
        hRmSpi->CurrentDirection |= SerialHwDataFlow_Rx;

    if (SpiPinMap)
    {

        NvRmPinMuxConfigSelect(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                                                hRmSpi->InstanceId, SpiPinMap);

        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                                        hRmSpi->InstanceId, SpiPinMap, NV_FALSE);
    }
    else
    {
        if (hRmSpi->IsIdleSignalTristate)
            NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_FALSE);
    }

    hRmSpi->CurrTransInfo.pRxBuff = NULL;
    hRmSpi->CurrTransInfo.RxPacketsRemaining = 0;
    hRmSpi->CurrTransInfo.pTxBuff = NULL;
    hRmSpi->CurrTransInfo.TxPacketsRemaining = 0;

    TotalWordsRequested = (TotalPacketsRequsted + PacketsPerWord -1)/PacketsPerWord;

    // Allocate the dma here if transaction size is more than cpu based
    // transaction thresold.
    if ((TotalWordsRequested > hRmSpi->HwRegs.MaxWordTransfer) &&
            (hRmSpi->DmaBufferSize) &&
            (!hRmSpi->IsApbDmaAllocated))
    {
        hRmSpi->TransCountFromLastDmaUsage = 0;
        hRmSpi->IsApbDmaAllocated = NV_TRUE;
        DmaModuleId = (hRmSpi->IsSpiChannel)?NvRmDmaModuleID_Spi: NvRmDmaModuleID_Slink;
        Error = NvRmDmaAllocate(hRmSpi->hDevice, &hRmSpi->hRmRxDma,
                         NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                         hRmSpi->InstanceId);
        if (!Error)
        {
            Error = NvRmDmaAllocate(hRmSpi->hDevice, &hRmSpi->hRmTxDma,
                             NV_FALSE, NvRmDmaPriority_High, DmaModuleId,
                             hRmSpi->InstanceId);
            if (Error)
                NvRmDmaFree(hRmSpi->hRmRxDma);
        }
        if (Error)
        {
            hRmSpi->hRmRxDma = NULL;
            hRmSpi->hRmTxDma = NULL;
            hRmSpi->IsApbDmaAllocated = NV_FALSE;
            Error = NvSuccess;
        }
    }
    Error = SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz,
                                        NV_TRUE, NV_FALSE);
    if (Error)
        goto cleanup;

    hRmSpi->hHwInterface->HwSetDataFlowFxn(&hRmSpi->HwRegs,
                                                hRmSpi->CurrentDirection, NV_TRUE);

    hRmSpi->hHwInterface->HwSetPacketLengthFxn(&hRmSpi->HwRegs,
                                                PacketSizeInBits, IsPackedMode);

    // Use cpu for less number of the data transfer.
    if ((!hRmSpi->IsApbDmaAllocated) ||
        (TotalWordsRequested <= hRmSpi->HwRegs.MaxWordTransfer))
    {
        hRmSpi->TransCountFromLastDmaUsage++;
        Error = MasterModeReadWriteCpu(hRmSpi, pReadBuffer, pWriteBuffer,
            TotalPacketsRequsted, &PacketsTransferred,
            IsPackedMode, PacketSizeInBits);
    }
    else
    {
        hRmSpi->TransCountFromLastDmaUsage = 0;
        Error = MasterModeReadWriteDma(hRmSpi, pReadBuffer, pWriteBuffer,
            TotalPacketsRequsted, &PacketsTransferred,
            IsPackedMode, PacketSizeInBits);
    }

    hRmSpi->hHwInterface->HwSetDataFlowFxn(&hRmSpi->HwRegs,
                                            hRmSpi->CurrentDirection, NV_FALSE);
    hRmSpi->CurrentDirection = SerialHwDataFlow_None;
    (void)SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz,
                                   NV_FALSE, NV_FALSE);

cleanup:

    //  Re-tristate multi-plexed controllers, and re-multiplex the controller.
    if (SpiPinMap)
    {
        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, SpiPinMap, NV_TRUE);

        NvRmPinMuxConfigSelect(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, hRmSpi->SpiPinMap);
    }
    else
    {
        if (hRmSpi->IsIdleSignalTristate)
            NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
                hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_TRUE);
    }

    if ((hRmSpi->IsApbDmaAllocated) &&
            (hRmSpi->TransCountFromLastDmaUsage > MAX_DMA_HOLD_TIME))
    {
        NvRmDmaFree(hRmSpi->hRmRxDma);
        NvRmDmaFree(hRmSpi->hRmTxDma);
        hRmSpi->hRmRxDma = NULL;
        hRmSpi->hRmTxDma = NULL;
        hRmSpi->IsApbDmaAllocated = NV_FALSE;
    }

#if !NV_OAL
    BoostFrequency(hRmSpi, NV_FALSE, 0);
    SetPowerControl(hRmSpi, NV_FALSE);
    NvOsMutexUnlock(hRmSpi->hChannelAccessMutex);
#endif

    NV_ASSERT(Error == NvSuccess);

}

/**
 * Start the data trasfer in slave mode.
 */
NvError NvRmSpiStartTransaction(
    NvRmSpiHandle hRmSpi,
    NvU32 ChipSelectId,
    NvU32 ClockSpeedInKHz,
    NvBool IsReadTransfer,
    NvU8 *pWriteBuffer,
    NvU32 BytesRequested,
    NvU32 PacketSizeInBits)
{
    NvError Error = NvSuccess;
    NvBool IsPackedMode;
    NvU32 BytesPerPackets;
    NvU32 TotalWordsRequested;
    NvU32 PacketsPerWord;
    NvU32 TotalPacketsRequsted;

    NV_ASSERT(hRmSpi);
    NV_ASSERT((IsReadTransfer) || (pWriteBuffer));

    // Packet size should be 1 to 32..
    NV_ASSERT((PacketSizeInBits > 0) && (PacketSizeInBits <= 32));

    // Transfer is allowed for the slave mode only from this API.
    NV_ASSERT(!hRmSpi->IsMasterMode);

    BytesPerPackets = (PacketSizeInBits + 7)/8;

    // Packets should be byte alligned.
    NV_ASSERT((BytesRequested % BytesPerPackets) == 0);

    // Slave mode will take the configuration from the Chip select 0.
    NV_ASSERT(hRmSpi->IsChipSelSupported[ChipSelectId]);

    TotalPacketsRequsted = BytesRequested/BytesPerPackets;

    // Select Packed mode for the 8/16 bit length.
    // nonwordaligned packed mode is not supported then check for the wordaligend
    // packets also.
    if (hRmSpi->HwRegs.IsNonWordAlignedPackModeSupported)
    {
        IsPackedMode = ((PacketSizeInBits == 8) ||(PacketSizeInBits == 16));
    }
    else
    {
        IsPackedMode = (((PacketSizeInBits == 8) && (!(TotalPacketsRequsted & 0x3))) ||
                        ((PacketSizeInBits == 16) && (!(TotalPacketsRequsted & 0x1))));
    }
    PacketsPerWord =  (IsPackedMode)? 4/BytesPerPackets: 1;
    hRmSpi->CurrTransInfo.PacketsPerWord = PacketsPerWord;

    TotalWordsRequested = (TotalPacketsRequsted + PacketsPerWord -1)/PacketsPerWord;

    // Total word trasfer should be maximum of 16KW (64KB): Hw Dma constraints
    NV_ASSERT(TotalWordsRequested <= MAXIMUM_SLAVE_TRANSFER_WORD);

    // Packet requested should not be more than 64KB: Slink controller constraints
    NV_ASSERT(TotalPacketsRequsted <= (1 << 16));

    // If total transfer word is more than 64KB (dma max transfer) or
    // number of packet requested is more than 64K (slink max packet transfer)
    // then return the error as NotSupported.
    if ((TotalWordsRequested > MAXIMUM_SLAVE_TRANSFER_WORD) ||
            (TotalPacketsRequsted > (1 << 16)))
        return NvError_NotSupported;


    // Lock the channel access.
    NvOsMutexLock(hRmSpi->hChannelAccessMutex);

    hRmSpi->CurrTransferChipSelId = ChipSelectId;

    if (hRmSpi->IsIdleSignalTristate)
        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_FALSE);

    // Enable Power/Clock.
    Error = SetPowerControl(hRmSpi, NV_TRUE);
    if (!Error)
        BoostFrequency(hRmSpi, NV_TRUE, BytesRequested);

    if (!Error)
        Error = SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz,
                                                    NV_TRUE, NV_FALSE);

    if (Error)
        goto cleanup;

    hRmSpi->CurrentDirection = (IsReadTransfer)?SerialHwDataFlow_Rx : SerialHwDataFlow_None;
    if (pWriteBuffer)
        hRmSpi->CurrentDirection |= SerialHwDataFlow_Tx;

    // Set the data direction
    hRmSpi->hHwInterface->HwSetDataFlowFxn(&hRmSpi->HwRegs,
                                            hRmSpi->CurrentDirection, NV_TRUE);

    // Use only interrupt mode for transfer
    hRmSpi->hHwInterface->HwSetInterruptSourceFxn(&hRmSpi->HwRegs,
                                            hRmSpi->CurrentDirection, NV_TRUE);

    hRmSpi->hHwInterface->HwSetPacketLengthFxn(&hRmSpi->HwRegs,
                                        PacketSizeInBits, IsPackedMode);

    // Use cpu if the dma is not allocated or the transfer size is less than
    // one fifo depth
    if ((!hRmSpi->IsApbDmaAllocated) ||
                (TotalWordsRequested <= hRmSpi->HwRegs.MaxWordTransfer))
    {
        // Non dma mode: The maximum word transfer is the fifo depth.
        // The word requested should be less than the maximum one transaction.
        // We can not split the  slave transaction in multiple small transactions
        NV_ASSERT(TotalWordsRequested <= hRmSpi->HwRegs.MaxWordTransfer);

        Error = SlaveModeSpiStartReadWriteCpu(hRmSpi, IsReadTransfer, pWriteBuffer,
                    TotalPacketsRequsted, IsPackedMode, PacketSizeInBits);
    }
    else
    {
        Error = SlaveModeSpiStartReadWriteDma(hRmSpi, IsReadTransfer, pWriteBuffer,
                    TotalPacketsRequsted, IsPackedMode, PacketSizeInBits);
    }

    if (!Error)
        return Error;
cleanup:

    (void)SetChipSelectSignalLevel(hRmSpi, ChipSelectId, ClockSpeedInKHz, NV_FALSE, NV_TRUE);

    if (hRmSpi->IsIdleSignalTristate)
        NvRmPinMuxConfigSetTristate(hRmSpi->hDevice,hRmSpi->RmIoModuleId,
            hRmSpi->InstanceId, hRmSpi->SpiPinMap, NV_TRUE);
    BoostFrequency(hRmSpi, NV_FALSE, 0);
    SetPowerControl(hRmSpi, NV_FALSE);
    NvOsMutexUnlock(hRmSpi->hChannelAccessMutex);
    return Error;
}

NvError
NvRmSpiGetTransactionData(
    NvRmSpiHandle hRmSpiSlink,
    NvU8 *pReadBuffer,
    NvU32 BytesRequested,
    NvU32 *pBytesTransfererd,
    NvU32 WaitTimeout)
{
    NvError Error = NvSuccess;

    NV_ASSERT(pBytesTransfererd);
    NV_ASSERT((pReadBuffer && (hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx )) ||
              ((!pReadBuffer) && (!(hRmSpiSlink->CurrentDirection & SerialHwDataFlow_Rx))));

    if (hRmSpiSlink->CurrentDirection == SerialHwDataFlow_None)
        return NvError_InvalidState;

    Error = SlaveModeSpiCompleteReadWrite(hRmSpiSlink, pReadBuffer,
                                            pBytesTransfererd, WaitTimeout);

    (void)SetChipSelectSignalLevel(hRmSpiSlink, hRmSpiSlink->CurrTransferChipSelId,
                                                0, NV_FALSE, NV_TRUE);

    if (hRmSpiSlink->IsIdleSignalTristate)
        NvRmPinMuxConfigSetTristate(hRmSpiSlink->hDevice,hRmSpiSlink->RmIoModuleId,
            hRmSpiSlink->InstanceId, hRmSpiSlink->SpiPinMap, NV_TRUE);

    BoostFrequency(hRmSpiSlink, NV_FALSE, 0);

    // Disable Power/Clock.
    SetPowerControl(hRmSpiSlink, NV_FALSE);
    NvOsMutexUnlock(hRmSpiSlink->hChannelAccessMutex);
    return Error;
}

void
NvRmSpiSetSignalMode(
    NvRmSpiHandle hRmSpi,
    NvU32 ChipSelectId,
    NvU32 SpiSignalMode)
{
    NV_ASSERT(hRmSpi);
    if (hRmSpi->IsChipSelSupported[ChipSelectId])
    {
        NvOsMutexLock(hRmSpi->hChannelAccessMutex);
        hRmSpi->DeviceInfo[ChipSelectId].SignalMode = (NvOdmQuerySpiSignalMode)SpiSignalMode;
        NvOsMutexUnlock(hRmSpi->hChannelAccessMutex);
    }
}

