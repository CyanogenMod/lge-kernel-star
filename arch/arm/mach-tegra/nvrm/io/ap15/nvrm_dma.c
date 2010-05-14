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
 *           DMA Resource manager </b>
 *
 * @b Description: Implements the interface of the NvRM DMA. This files
 *  implements the API for the dma for the AP15 Dma controller.
 *
 * This file contains the common code for ap15 dma controller to manage
 * the different operation of the dma.
 */

/**
 *                Dma Design details
 *                ------------------
 * 1. There is two type of dma allocation i.e low priority and high priority
 * dma. The low prioirty allocation shares the same dma channel between different
 * client. The high prioirty allocation does not share the dma channel and the
 * dma channel is used by the requestd clients only. Hence, the high priority
 * dma allocation may fail if there is no channel for the allocation but the low
 * priority channel allocation will not fail till we have the sufficient memory
 * for the dma handle creation.
 *
 * 2. The dma allocation is done based on the requestor module Id. It only
 * support the dma transfer from teh memory to the apb peripheral or vice versa.
 *
 * 3. The DmaTransfer transfers the data from source to dest and dest to source
 * based on the direction passed. It may be possible to do the dma transfer
 * from destination to source address by passing the dma direction as reverse.
 *
 * 4. The destination and source address may be any type like peripheral or
 * memory or xmb memory. There is no restriction on passing the source/destn
 * address by the client. The implementation will take care of proper
 * configuration of the dma register address.
 *
 * 5. It may be possible to free the dma when transfer is going on.
 * In this case, the dma will be free for the another allocation once the
 * transfer completes. The dma handle will be destroyed immediately for the
 * client.
 *
 * 6. It is possible to abort the dma transfer for both type of dma, high
 * priority and low priority. In this case, the dma transfer will be immediatly
 * stops if the transfer is going on for the requestor client and all  dma
 * request will be aborted.
 *
 * 7. The client can request for any ammount of the data transfer. If dma is not
 * capable of transferring the data in one transaction, it will do the multiple
 * transaction internally and will notify the client after last transaction.
 *
 *
 *                Implementation details
 *                ----------------------
 *  1. The implementation should support any number of the apb dma
 * channel on run time. There should not be any static allocation till it
 * very necessarily. It does not support the ahb dma.
 *
 * 2. 1 dma channel allocated for the  low priority dma channel allocation to
 * allocate the low priority dma handle. These channes are shared between the
 * low priority reqestor clients.
 *
 * 3. The client will abort the dma request done by him only. It can not cancel
 * the request done by other clients.
 *
 * 4. Dma Request can be queued and there is not any limitation to queue the
 * request till we have the sufficient memory from the os.
 *
 * 5. It supports the synchrnous and asynchrnous, both type of the operation.
 *
 * 6. For each dma channel, it allocates the memory for keeping the client
 * request.
 * if the number of request is more than the allocated number of list then it
 * again reallocate the memory for the new request and free the already allocated
 * list. The old request transferered to the new allocated list. the benifit
 * of this type of method is that we need not to do the allocation to queue the
 * request for each transfer request. In this way we can avoid the memory
 * allocation and freeing of the memory for the each time.
 * We start the allocation of memory from n and if the number of request is more
 * than this (n) then reallocation is done for the (n +n) request and if it is
 * full then again reallocation is done for the (2n + 2n). In this way the order
 * of allocation is Log(n).
 *
 * 7. All apb dma channel inetrrupt is handle in single isr.
 * The detection of the interrupted dma channel is done by scanning all the dma
 * channels one by one.
 *
 * 8. The apb dma hw control api is called using the function pointer. So
 * whenever there is difefrence in the handling of the dma request for dma
 * channel, it uses the dma hw interface.
 *
 * 9. I2s channels related request will use the continuous double buffering.
 * Uart receive (from fifo to memory) will use the continuous double buffering
 * on same buffer.
 *
 */

#include "nvrm_dma.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_moduleids.h"
#include "nvrm_hardware_access.h"
#include "rm_dma_hw_private.h"
#include "nvassert.h"
#include "nvrm_priv_ap_general.h"
#include "mach/nvrm_linux.h"

/* FIXME move these to some header file */
NvError NvRmPrivDmaInit(NvRmDeviceHandle hDevice);
void NvRmPrivDmaDeInit(void);
NvError NvRmPrivDmaSuspend(void);
NvError NvRmPrivDmaResume(void);

#define MAX_AVP_DMA_CHANNELS 3

// DMA capabilities -- these currently do not vary between chips

// Maximum dma transfer size for one transfer.
#define DMA_MAX_TRANSFER_SIZE       0x10000

// Address allignment  reequirement for the dma buffer address
#define DMA_ADDRESS_ALIGNMENT       4

// Transfer size allignment  for the dma transfer.
#define DMA_TRANSFER_SIZE_ALIGNMENT 4

// Dma transfer request depth for initial req depth
#define DMA_TRANSFER_REQ_DEPTH 16

// The end index of the list
#define DMA_NULL_INDEX 0xFFFF

// Defines the dma request states.
typedef enum
{
    // The request has not been started.
    RmDmaRequestState_NotStarted = 0x1,

    // The request is running state.
    RmDmaRequestState_Running ,

    // The request is completed state.
    RmDmaRequestState_Completed ,

    // The request is stopped state.
    RmDmaRequestState_Stopped,

    // The request is unused state.
    RmDmaRequestState_Unused,

    RmDmaRequestState_Force32 = 0x7FFFFFFF
} RmDmaRequestState;

// Defines the dma channel allocation state.
typedef enum
{
    // Dma channel is free and available for the allocation.
    RmDmaChannelState_Free = 0x1,

    // The dma channel is free from the client but still it has the request
    // for the data transfer.
    RmDmaChannelState_MarkedFree ,

    // Dma channel is used by the client.
    RmDmaChannelState_Used,

    RmDmaChannelState_Force32 = 0x7FFFFFFF
} RmDmaChannelState;

// Defines the dma channel transfer mode and property.
typedef enum
{
    // initial value of the states.
    RmDmaTransferMode_Init = 0x0,

    // Dma channel transfer mode is continuous.
    RmDmaTransferMode_Continuous = 0x1,

    // Dma channel transfer mode is Double buffering.
    RmDmaTransferMode_DoubleBuff = 0x2,

    // Dma channel transfer mode is to transfer the same buffer afain and again.
    RmDmaTransferMode_SameBuff = 0x4,

    // Dma channel transfer where source address is the Xmb address.
    RmDmaTransferMode_SourceXmb = 0x8,

    // Dma channel transfer where source address is the Peripheral address.
    RmDmaTransferMode_SourcePeripheral = 0x10,

    // Dma channel transfer request is asynchrnous.
    RmDmaTransferMode_Asynch = 0x20,

    // Dma channel transfer is for the pin interrupt now.
    RmDmaTransferMode_PingIntMode = 0x40,

    RmDmaTransferMode_Force32 = 0x7FFFFFFF
} RmDmaTransferMode;

/**
 * Combines the Dma transfer request information which will be queued and
 * require to start the transfer and for notification after transfer completes.
 */
typedef struct DmaTransReqRec
{
    // Unique Id
    NvU32 UniqueId;

    // Current state of the channel.
    RmDmaRequestState State;

    // The dema request transfer mode and details of the request.
    RmDmaTransferMode TransferMode;

    // The Source address for the data transfer.
    NvRmPhysAddr SourceAdd;

    // The destiniation address for the data transfer.
    NvRmPhysAddr DestAdd;

    // The source address wrapping.
    NvU32 SourceAddWrap;

    // The destination address wrapping.
    NvU32 DestAddWrap;

    // Number of bytes requested.
    NvU32 BytesRequested;

    // Number of bytes programmed for current data transfer.
    NvU32 BytesCurrProgram;

    // Number of bytes remaining to transfer.
    NvU32 BytesRemaining;

    // The configuartion of dma in terms of register content and channel
    // register info.
    DmaChanRegisters DmaChanRegs;

    // Semaphore Id which need to be signalled after completion.
    NvOsSemaphoreHandle hOnDmaCompleteSema;

    // Semaphore Id which need to be signalled after half of the transfer
    // completion.
    NvOsSemaphoreHandle hOnHalfDmaCompleteSema;

    // Semaphore Id which need to be destoyed when new request will be placed
    // by this list memory.
    NvOsSemaphoreHandle hLastReqSema;

    // Array based the double link list.
    NvU16 NextIndex;

    NvU16 PrevIndex;

} DmaTransReq;

/**
 * Combines the channel information, status, requestor information for the
 * channel dma, type of dma etc.
 */
typedef struct RmDmaChannelRec
{
    // State of the channel.
    RmDmaChannelState ChannelState;

    // Dma priority whether this is low priority channel or high prority
    // channel.
    NvRmDmaPriority Priority;

    // Pointer to the list of the transfer request.
    struct DmaTransReqRec *pTransReqList;

    // Currently maximum request possible.
    NvU16 MaxReqList;

    // Head index to the request
    NvU16 HeadReqIndex;

    // Tail Index to the request
    NvU16 TailReqIndex;

    // Head index to the free list.
    NvU16 HeadFreeIndex;

    // Mutex to provide the thread/interrupt safety for the channel specific
    // data.
    NvOsIntrMutexHandle hIntrMutex;

    // The virtual base address of the channel registers.
    NvU32 *pVirtChannelAdd;

    // Channel address bank size.
    NvU32 ChannelAddBankSize;

    // Pointer to the dma hw interface apis strcuture.
    DmaHwInterface *pHwInterface;

    // Log the last requested size
    NvU32 LastReqSize;

#if NVOS_IS_LINUX
    // Channel interrupt handle 
    NvOsInterruptHandle hIntrHandle;
#endif

} RmDmaChannel, *RmDmaChannelHandle;

/**
 * Combines the dma information
 */
typedef struct
{
    // Device handle.
    NvRmDeviceHandle hDevice;

    // Actual numbers of Apb dma channels available on the soc.
    NvU32 NumApbDmaChannels;

    RmDmaChannel *pListApbDmaChannel;

    // Apb Dma General registers
    DmaGenRegisters ApbDmaGenReg;

    // OS mutex for channel allocation and deallocation:  provide thread safety
    NvOsMutexHandle hDmaAllocMutex;
} NvRmPrivDmaInfo;

/**
 * Combines the Dma requestor and related information which is required for
 * other dma operation request.
 */
typedef struct NvRmDmaRec
{
    // Store the Rm device handle
    NvRmDeviceHandle  hRmDevice;

    // Corresponding dma channel pointer to APB dma for this handle.
    RmDmaChannel *pDmaChannel;

    // Flag to tells whether 32 bit swap is enabled or not.
    NvBool IsBitSwapEnable;

    // Unique Id
    NvU32 UniqueId;

    // Dma requestor module Id.
    NvRmDmaModuleID DmaReqModuleId;

    // dma requestor instance Id.
    NvU32 DmaReqInstId;

    // Dma register information which contain the configuration for dma when it
    // was allocated
    DmaChanRegisters DmaChRegs;

    // NvOs semaphore which will be used when synchrnous operation is requested.
    NvOsSemaphoreHandle hSyncSema;
} NvRmDma;

static NvRmPrivDmaInfo s_DmaInfo;
static DmaHwInterface s_ApbDmaInterface;
#if !NVOS_IS_LINUX
static NvOsInterruptHandle s_ApbDmaInterruptHandle = NULL;
#endif

NvU32 NvRmDmaUnreservedChannels(void)
{
    return s_DmaInfo.NumApbDmaChannels - MAX_AVP_DMA_CHANNELS -
        TEGRA_SYSTEM_DMA_CH_NUM;
}


/**
 * Deinitialize the apb dma physical/virtual addresses. This function will
 * unmap the virtual mapping.
 *
 * Thread Safety: Caller responsibility.
 */
static void DeInitDmaGeneralHwRegsAddress(void)
{
    // Unmap the virtual mapping for apb general register.
    NvRmPhysicalMemUnmap(s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd,
                         s_DmaInfo.ApbDmaGenReg.GenAddBankSize);
    s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd = NULL;
}

/**
 * Initialize the apb dma physical/virtual addresses. This function will get
 * the physical address of Apb dma channel from Nvrm module APIs, get the
 * virtual address.
 *
 * Thread Safety: Caller responsibility.
 */
static NvError InitDmaGeneralHwRegsAddress(void)
{
    NvError Error = NvSuccess;
    NvRmDeviceHandle hDevice = NULL;
    NvRmModuleID ModuleId;
    NvRmPhysAddr ApbPhysAddr;

    // Required the valid device handles.
    hDevice = s_DmaInfo.hDevice;

    // Get the physical base address of the apb dma controller general register.
    ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0);
    NvRmModuleGetBaseAddress(hDevice, ModuleId,
        &ApbPhysAddr, &s_DmaInfo.ApbDmaGenReg.GenAddBankSize);

    // Initialize the apb dma register virtual address.
    s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd = NULL;

    // Get the virtual address of apb dma general base address.
    Error = NvRmPhysicalMemMap(ApbPhysAddr,
        s_DmaInfo.ApbDmaGenReg.GenAddBankSize, NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void **)&s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd);

    return Error;
}

static NvError AllocateReqList(RmDmaChannel *pDmaChannel, NvU16 MoreListSize)
{
    NvU16 Index;
    DmaTransReq *pTransReqList = NULL;
    DmaTransReq *pExistTransReqList = pDmaChannel->pTransReqList;
    NvU32 TotalReqSize = (pDmaChannel->MaxReqList + MoreListSize);

    // Allocate the memory for logging the client requests.
    pTransReqList = NvOsAlloc(TotalReqSize * sizeof(DmaTransReq));
    if (!pTransReqList)
        return NvError_InsufficientMemory;

    NvOsMemset(pTransReqList, 0, TotalReqSize * sizeof(DmaTransReq));

    // Copy the existing request if it exist to the new allocated request list.
    if (pExistTransReqList)
    {
        NvOsMemcpy(pTransReqList, pExistTransReqList,
                    pDmaChannel->MaxReqList * sizeof(DmaTransReq));
        NvOsFree(pExistTransReqList);
    }

    for (Index = pDmaChannel->MaxReqList; Index < TotalReqSize; ++Index)
    {
        if (Index == pDmaChannel->MaxReqList)
            pTransReqList[pDmaChannel->MaxReqList].PrevIndex = DMA_NULL_INDEX;
        else
            pTransReqList[Index].PrevIndex = Index-1;

        pTransReqList[Index].NextIndex = Index + 1;
    }
    pTransReqList[Index-1].NextIndex = DMA_NULL_INDEX;
    pDmaChannel->pTransReqList = pTransReqList;
    pDmaChannel->HeadFreeIndex = pDmaChannel->MaxReqList;
    pDmaChannel->MaxReqList += MoreListSize;
    return NvSuccess;
}

/**
 * Deinitialize the Apb dma channels. It will free all the memory and resource
 * allocated for the dma channels.
 *
 * Thread Safety: Caller responsibility.
 */
static void DeInitDmaChannels(RmDmaChannel *pDmaList, NvU32 TotalChannel)
{
    NvU32 i;
    if (!pDmaList)
        return;

    for (i = 0; i < TotalChannel; i++)
    {
        RmDmaChannel *pDmaChannel = &pDmaList[i];
        if (pDmaChannel)
        {
            NvOsFree(pDmaChannel->pTransReqList);
            pDmaChannel->MaxReqList = 0;

            // Free the dma virtual maping
            NvRmPhysicalMemUnmap(pDmaChannel->pVirtChannelAdd,
                pDmaChannel->ChannelAddBankSize);
            NvOsIntrMutexDestroy(pDmaChannel->hIntrMutex);
        }
    }
    NvOsFree(pDmaList);
}

/**
 * Init Apb dma channels.It makes the list of all available dma channesl and
 * keep in the free channel list so that it will be available for the
 * allocation.
 * Once client ask for dma channel, it will look in the free list and remove the
 * channel from the free list and attach with the dma handle and keep in the
 * used list. The client data trasfer request is queued for the dma channels.
 *
 * Thread Safety: Caller responsibility.
 */
static NvError
InitDmaChannels(
    NvRmDeviceHandle hDevice,
    RmDmaChannel **pDmaChannelList,
    NvU32 TotalChannel,

    NvRmModuleID DmaModuleId)
{
    NvU32 ChanIndex;
    NvError Error = NvSuccess;
    RmDmaChannel *pDmaChannel = NULL;
    NvRmModuleID ModuleId = 0;
    NvRmPhysAddr ChannelPhysAddr;
    RmDmaChannel *pDmaList = NULL;

    // Allocate the memory to store the all dma channel information.
    pDmaList = NvOsAlloc(TotalChannel * sizeof(RmDmaChannel));
    if (!pDmaList)
        return NvError_InsufficientMemory;

    // Initialize all dma channel structure with default values.
    for (ChanIndex = 0; ChanIndex < TotalChannel; ++ChanIndex)
    {
        pDmaChannel = &pDmaList[ChanIndex];

        // Initialize all channel member to the initial known states.
        pDmaChannel->ChannelState = RmDmaChannelState_Free;
        pDmaChannel->Priority = NvRmDmaPriority_High;
        pDmaChannel->pTransReqList = NULL;
        pDmaChannel->MaxReqList = 0;
        pDmaChannel->HeadReqIndex = DMA_NULL_INDEX;
        pDmaChannel->TailReqIndex = DMA_NULL_INDEX;
        pDmaChannel->HeadFreeIndex = DMA_NULL_INDEX;
        pDmaChannel->hIntrMutex = NULL;
        pDmaChannel->pVirtChannelAdd = NULL;
        pDmaChannel->ChannelAddBankSize = 0;
        pDmaChannel->pHwInterface = &s_ApbDmaInterface;
    }

    // Allocate the resource and register address for each channels.
    for (ChanIndex = 0; ChanIndex < TotalChannel; ++ChanIndex)
    {
        pDmaChannel = &pDmaList[ChanIndex];

        // Allocate the memory for logging the client request.
        Error = AllocateReqList(pDmaChannel, DMA_TRANSFER_REQ_DEPTH);

        // Create mutex for the channel access.
        if (!Error)
            Error = NvOsIntrMutexCreate(&pDmaChannel->hIntrMutex);

        // Initialize the base address of the channel.
        if (!Error)
        {
            ModuleId = NVRM_MODULE_ID(DmaModuleId, ChanIndex);
            NvRmModuleGetBaseAddress(hDevice, ModuleId, &ChannelPhysAddr,
                                &pDmaChannel->ChannelAddBankSize);
            Error = NvRmPhysicalMemMap(ChannelPhysAddr,
                pDmaChannel->ChannelAddBankSize, NVOS_MEM_READ_WRITE,
                NvOsMemAttribute_Uncached,
                (void **)&pDmaChannel->pVirtChannelAdd);
        }
        if (Error)
            break;
    }

    if (!Error)
    {
        // Allocate last channel as a low priority request, others are
        // high priority channel
        *pDmaChannelList = (RmDmaChannel *)pDmaList;
    }
    else
    {
        DeInitDmaChannels(pDmaList, TotalChannel);
        *pDmaChannelList = (RmDmaChannel *)NULL;
    }
    return Error;
}

/**
 * Initialize the Apb dma channels.
 * Thread Safety: Caller responsibility.
 */
static NvError InitAllDmaChannels(void)
{
    NvError Error = NvSuccess;

    // Initialize the apb dma channel list.
    Error = InitDmaChannels(s_DmaInfo.hDevice, &s_DmaInfo.pListApbDmaChannel,
                s_DmaInfo.NumApbDmaChannels, NvRmPrivModuleID_ApbDmaChannel);
    return Error;
}

/**
 * Deinitialize the Apb dma channels.
 * Thread Safety: Caller responsibility.
 */
static void DeInitAllDmaChannels(void)
{
    // Deinitialize the apb dma channels.
    DeInitDmaChannels(s_DmaInfo.pListApbDmaChannel, s_DmaInfo.NumApbDmaChannels);
    s_DmaInfo.pListApbDmaChannel = NULL;
}

/**
 * DeInitialize the Dmas. It include the deinitializaton of Apb dma channels.
 * It unmap the dma register address, disable clock of dma, reset the dma,
 * destroy the dma interrupt threads and destroy the list of all channels.
 *
 * Thread Safety: Caller responsibility.
 */
static void DeInitDmas(void)
{
    // Global disable the dma channels.
    s_ApbDmaInterface.DmaHwGlobalSetFxn(s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd,
                                                        NV_FALSE);

    // Disable the dma clocks.
    // Disable clock for the apb dma channels.
    (void)NvRmPowerModuleClockControl(s_DmaInfo.hDevice, NvRmPrivModuleID_ApbDma,
                                                        0, NV_FALSE);

    // De-Initialize of the dma channel lists.
    DeInitAllDmaChannels();
}

/**
 * Initialize the Dma. It include the initializaton of Apb dma channels.
 * It initalize the dma register address, clock of dma, do the reset of dma,
 * create the dma interrupt threads and make the list of all channels
 * for allocation.
 *
 * Thread Safety: Caller responsibility.
 */
static NvError InitDmas(NvRmDeviceHandle hRmDevice)
{
    NvError Error = NvSuccess;

    // Initialize of the dma channel lists.
    Error = InitAllDmaChannels();

    // Enable the clocks of dma channels.
    if (!Error)
        Error = NvRmPowerModuleClockControl(hRmDevice, NvRmPrivModuleID_ApbDma,
                                                        0, NV_TRUE);
    // Reset the dma channels.
    if (!Error)
        NvRmModuleReset(hRmDevice, NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0));

    // Global enable the dma channels.
    if (!Error)
        s_ApbDmaInterface.DmaHwGlobalSetFxn(s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd,
                                                            NV_TRUE);

    // If error exist then disable the dma clocks.
    if (Error)
        DeInitDmas();

    return Error;
}


/**
 * Continue the current transfer by sending the next chunk of the data from the
 * current dma transfer request. This may be called when requested size is
 * larger than the supported dma transfer size in single go by hw.
 *
 */
static void ApbDmaContinueRemainingTransfer(void *pDmaChan)
{
    NvU32 CurrProgSize;
    NvU32 LastTransferSize;
    DmaTransReq  *pCurrReq = NULL;
    NvBool IsDoubleBuff;
    NvBool IsContMode;
    RmDmaChannel *pDmaChannel = (RmDmaChannel *)pDmaChan;

    pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];

    // Get the last transfer size in bytes from the start of the source and
    // destination address
    LastTransferSize  = pCurrReq->BytesCurrProgram;

    // Calculate the possible transfer size based on remaining bytes and
    // maximum transfer size. Updates the remaining size, transfer size and
    // programmed size accordingly.
    CurrProgSize = NV_MIN(pCurrReq->BytesRemaining, DMA_MAX_TRANSFER_SIZE);

    IsDoubleBuff = (pCurrReq->TransferMode & RmDmaTransferMode_DoubleBuff)? NV_TRUE: NV_FALSE;
    IsContMode = (pCurrReq->TransferMode & RmDmaTransferMode_Continuous)? NV_TRUE: NV_FALSE;

    // Program the transfer size.
    pDmaChannel->pHwInterface->DmaHwSetTransferSizeFxn(&pCurrReq->DmaChanRegs,
                                                CurrProgSize, IsDoubleBuff);
    pDmaChannel->pHwInterface->DmaHwStartTransferWithAddIncFxn(
                    &pCurrReq->DmaChanRegs, 0, LastTransferSize, IsContMode);

    // Update the parameter which will be used in future.
    pCurrReq->BytesRemaining  -= CurrProgSize;
    pCurrReq->BytesCurrProgram = CurrProgSize;
}


/**
 * Handle the dma complete interrupt in once mode.
 *
 * Thread Safety: Caller responsibility.
 */
static void
OnDmaCompleteInOnceMode(
    RmDmaChannel *pDmaChannel,
    DmaTransReq  *pCurrReq)
{
    NvOsSemaphoreHandle hSignalSema = NULL;
    NvU16 CurrHeadIndex;

    pDmaChannel->pHwInterface->DmaHwAckNClearInterruptFxn(&pCurrReq->DmaChanRegs);

    // The transfer was in running state.
    // Check if there is data remaining to transfer or not from the
    // current request. If there is bytes remaining for data transfer
    // then continue the transfer.
    if (pCurrReq->BytesRemaining)
    {
        pDmaChannel->pHwInterface->DmaContinueRemainingTransferFxn(pDmaChannel);
        return;
    }

    pCurrReq->State = RmDmaRequestState_Completed;

    // Store the sempahore whihc need to be signal.
    hSignalSema = pCurrReq->hOnDmaCompleteSema;
    pDmaChannel->LastReqSize = pCurrReq->BytesRequested;

    // Free this index.
    CurrHeadIndex = pDmaChannel->HeadReqIndex;
    pDmaChannel->HeadReqIndex = pDmaChannel->pTransReqList[CurrHeadIndex].NextIndex;
    pDmaChannel->pTransReqList[CurrHeadIndex].NextIndex = pDmaChannel->HeadFreeIndex;
    pDmaChannel->HeadFreeIndex = CurrHeadIndex;
    if (pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
    {
        pDmaChannel->TailReqIndex = DMA_NULL_INDEX;

        // If channel is marked as free by client then make this channel
        // for next allocation.
        if (pDmaChannel->ChannelState == RmDmaChannelState_MarkedFree)
            pDmaChannel->ChannelState = RmDmaChannelState_Free;

        // Notify the client for the data transfers completes.
        if (hSignalSema)
            NvOsSemaphoreSignal(hSignalSema);
        return;
    }
    pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];
    pCurrReq->State = RmDmaRequestState_Running;
    pDmaChannel->pHwInterface->DmaHwStartTransferFxn(&pCurrReq->DmaChanRegs);

    // Generate the notification for the current transfer completes.
    if (hSignalSema)
        NvOsSemaphoreSignal(hSignalSema);
}

static void
OnDmaCompleteInContinuousMode(
    RmDmaChannel *pDmaChannel,
    DmaTransReq  *pCurrReq)
{
    NvOsSemaphoreHandle hSignalSema = NULL;
    NvU16 NextHeadIndex;
    DmaTransReq  *pNextReq = NULL;

    pDmaChannel->pHwInterface->DmaHwAckNClearInterruptFxn(&pCurrReq->DmaChanRegs);

    // The transfer was in running state.
    // Check if there is data remaining to transfer or not from the
    // current request. If there is bytes remaining for data transfer
    // then continue the transfer.
    if (pCurrReq->BytesRemaining)
    {
        if (pCurrReq->TransferMode & RmDmaTransferMode_PingIntMode)
        {
            pCurrReq->TransferMode &= ~RmDmaTransferMode_PingIntMode;
            pDmaChannel->pHwInterface->DmaContinueRemainingTransferFxn(pDmaChannel);
        }
        else
        {
            pCurrReq->TransferMode |= RmDmaTransferMode_PingIntMode;
        }
        return;
    }

    NextHeadIndex = pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].NextIndex;
    if (NextHeadIndex != DMA_NULL_INDEX)
        pNextReq = &pDmaChannel->pTransReqList[NextHeadIndex];

    if (pCurrReq->TransferMode & RmDmaTransferMode_PingIntMode)
    {
        if (NextHeadIndex != DMA_NULL_INDEX)
        {
            pDmaChannel->pHwInterface->DmaHwContinueTransferFxn(&pNextReq->DmaChanRegs);
            pNextReq->State = RmDmaRequestState_Running;
            pNextReq->TransferMode |= RmDmaTransferMode_PingIntMode;
        }
        pDmaChannel->pHwInterface->DmaHwAddTransferCountFxn(&pCurrReq->DmaChanRegs);

        if (pCurrReq->hOnHalfDmaCompleteSema)
            NvOsSemaphoreSignal(pCurrReq->hOnHalfDmaCompleteSema);


        pCurrReq->TransferMode &= ~RmDmaTransferMode_PingIntMode;
        return;
    }

    pCurrReq->State = RmDmaRequestState_Completed;

    // Store the sempahore which need to be signal.
    hSignalSema = pCurrReq->hOnDmaCompleteSema;

    if (!pNextReq)
    {
        if (pCurrReq->TransferMode & RmDmaTransferMode_SameBuff)
        {
            pCurrReq->TransferMode |= RmDmaTransferMode_PingIntMode;
            pCurrReq->State = RmDmaRequestState_Running;
            if (hSignalSema)
                NvOsSemaphoreSignal(pCurrReq->hOnDmaCompleteSema);
            pDmaChannel->pHwInterface->DmaHwAddTransferCountFxn(&pCurrReq->DmaChanRegs);
            return;
        }
        else
        {
            pDmaChannel->pHwInterface->DmaHwStopTransferFxn(&pCurrReq->DmaChanRegs);

            pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].NextIndex = pDmaChannel->HeadFreeIndex;
            pDmaChannel->HeadFreeIndex = pDmaChannel->HeadReqIndex;
            pDmaChannel->HeadReqIndex = DMA_NULL_INDEX;
            pDmaChannel->TailReqIndex = DMA_NULL_INDEX;

            // If channel is marked as free then make this channel available
            // for next allocation.
            if (pDmaChannel->ChannelState == RmDmaChannelState_MarkedFree)
                pDmaChannel->ChannelState = RmDmaChannelState_Free;
        }
    }
    else
    {
        pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].NextIndex = pDmaChannel->HeadFreeIndex;
        pDmaChannel->HeadFreeIndex = pDmaChannel->HeadReqIndex;
        pDmaChannel->HeadReqIndex = NextHeadIndex;

        // May be we got this request after ping buffer completion.
        if (pNextReq->State != RmDmaRequestState_Running)
        {
            // Start the next request transfer.
            pDmaChannel->pHwInterface->DmaHwContinueTransferFxn(&pNextReq->DmaChanRegs);
            pNextReq->State = RmDmaRequestState_Running;
            pCurrReq->TransferMode |= RmDmaTransferMode_PingIntMode;
        }
    }

    // Generate the notification for the current transfer completes.
    if (hSignalSema)
        NvOsSemaphoreSignal(hSignalSema);
}



#if NVOS_IS_LINUX
/**
 * Handle the Apb dma interrupt.
 */
static void ApbDmaIsr(void *args)
{
    RmDmaChannel *pDmaChannel = (RmDmaChannel *)args;
    DmaTransReq *pCurrReq;
    NvBool IsTransferComplete;

    NvOsIntrMutexLock(pDmaChannel->hIntrMutex);
    if (pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
        goto exit;

    pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];
    if (pCurrReq->State != RmDmaRequestState_Running)
        goto exit;

    IsTransferComplete = pDmaChannel->pHwInterface->DmaHwIsTransferCompletedFxn(
                                            &pCurrReq->DmaChanRegs);
    if (IsTransferComplete) {
        if (pCurrReq->TransferMode & RmDmaTransferMode_Continuous)
            OnDmaCompleteInContinuousMode(pDmaChannel, pCurrReq);
        else
            OnDmaCompleteInOnceMode(pDmaChannel, pCurrReq);
    }

exit:
    NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    NvRmInterruptDone(pDmaChannel->hIntrHandle);
}
#else
static void ApbDmaIsr(void *args)
{
    RmDmaChannel *pDmaChannel;
    DmaTransReq *pCurrReq;
    NvU32 ChanIndex;
    NvBool IsTransferComplete;

    for (ChanIndex = 0; ChanIndex < s_DmaInfo.NumApbDmaChannels; ++ChanIndex)
    {
        pDmaChannel = &s_DmaInfo.pListApbDmaChannel[ChanIndex];
        if (pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
            continue;

        NvOsIntrMutexLock(pDmaChannel->hIntrMutex);
        if (pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
            goto NextLoop;

        pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];
        if (pCurrReq->State != RmDmaRequestState_Running)
            goto NextLoop;

        IsTransferComplete = pDmaChannel->pHwInterface->DmaHwIsTransferCompletedFxn(
                                                &pCurrReq->DmaChanRegs);
        if (!IsTransferComplete)
            goto NextLoop;

        if (pCurrReq->TransferMode & RmDmaTransferMode_Continuous)
            OnDmaCompleteInContinuousMode(pDmaChannel, pCurrReq);
        else
            OnDmaCompleteInOnceMode(pDmaChannel, pCurrReq);

    NextLoop:
        NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    }

    NvRmInterruptDone(s_ApbDmaInterruptHandle);
}
#endif


/**
 * Register apb Dma interrupt.
 */
static NvError RegisterAllDmaInterrupt(NvRmDeviceHandle hDevice)
{
    NvRmModuleID ModuleId = NvRmPrivModuleID_ApbDma;
    NvError Error = NvSuccess;
    NvOsInterruptHandler DmaIntHandler = ApbDmaIsr;
    NvU32 Irq = 0;
    NvU32 i;

    /* Disable interrupts for all channels */
    for (i=0; i < s_DmaInfo.NumApbDmaChannels; i++)
    {
        NvRmPrivDmaInterruptEnable(hDevice, i, NV_FALSE);
    }

#if NVOS_IS_LINUX
    /* Register same interrupt hanlder for all APB DMA channels. */
    for (i=0; i < NvRmDmaUnreservedChannels(); i++)
    {
        Irq = NvRmGetIrqForLogicalInterrupt(hDevice, ModuleId, i);
        Error = NvRmInterruptRegister(hDevice, 1, &Irq,
            &DmaIntHandler, &s_DmaInfo.pListApbDmaChannel[i], 
            &(s_DmaInfo.pListApbDmaChannel[i].hIntrHandle), NV_TRUE);
    }
#else
    /* Register one interrupt handler for all APB DMA channels
     * Pass index 0xFF to get the main IRQ of the ADB DMA sub-interrupt 
     * controller. */
    Irq = NvRmGetIrqForLogicalInterrupt(hDevice, ModuleId, 0xFF);
    Error = NvRmInterruptRegister(hDevice, 1, &Irq,
            &DmaIntHandler, hDevice, &s_ApbDmaInterruptHandle, NV_TRUE);

#endif

    if (Error != NvSuccess) return Error;

    /* Enable interrupts for all channels */
    for (i=0; i < s_DmaInfo.NumApbDmaChannels; i++)
    {
        NvRmPrivDmaInterruptEnable(hDevice, i, NV_TRUE);
    }
    return Error;
}

/**
 * Unregister apb Dma interrupts.
 */
static void UnregisterAllDmaInterrupt(NvRmDeviceHandle hDevice)
{
#if NVOS_IS_LINUX
    NvU32 i;

    for (i=0; i < NvRmDmaUnreservedChannels(); i++)
    {
        NvRmInterruptUnregister(hDevice,
            s_DmaInfo.pListApbDmaChannel[i].hIntrHandle);
    }
#else
    NvRmInterruptUnregister(hDevice, s_ApbDmaInterruptHandle);
    s_ApbDmaInterruptHandle = NULL;
#endif
}

/**
 * Destroy the dma informations. It releases all the memory and os resources
 * which was allocated to create the dma infomation.
 * PENDING: What happen if there is a request for data transfer and it is ask
 * for the DeInit().
 *
 */
static void DestroyDmaInfo(void)
{
    // Unregister for the dma interrupts.
    UnregisterAllDmaInterrupt(s_DmaInfo.hDevice);

    // Deinitialize the dmas.
    DeInitDmas();

    // Destroy the list of dma channels and release memory for all dma channels.
    NvOsMutexDestroy(s_DmaInfo.hDmaAllocMutex);
    s_DmaInfo.hDmaAllocMutex = NULL;

    //Deinitialize the dma hw register address.
    DeInitDmaGeneralHwRegsAddress();
}

/**
 * Create the dma information and setup the dma channesl to their initial state.
 * It enables all dma channels, make list of dma channels, initailize the
 * registes address, create reosurces for the channel allocation and bring the
 * dma driver in know states.
 *
 * It creates all the mutex which are used for dma channel, register the
 * interrupt, enable the clock and reset the dma channel.
 *
 * Verification of al the steps is done and if it fails then it relases the
 * resource which were created and it will return error.
 *
 */
static NvError CreateDmaInfo(NvRmDeviceHandle hDevice)
{
    NvError Error = NvSuccess;

    s_DmaInfo.hDevice = hDevice;
    s_DmaInfo.NumApbDmaChannels =
            NvRmModuleGetNumInstances(hDevice, NvRmPrivModuleID_ApbDmaChannel);

    NV_ASSERT(s_DmaInfo.NumApbDmaChannels > 0);
    NV_ASSERT(s_DmaInfo.NumApbDmaChannels <= MAX_APB_DMA_CHANNELS);

    // Initialize the dma hw register addresses.
    Error = InitDmaGeneralHwRegsAddress();

    // Initialze the channel alllocation mutex.
    if (!Error)
        Error = NvOsMutexCreate(&s_DmaInfo.hDmaAllocMutex);

    // Initialze the dma channels.
    if (!Error)
        Error = InitDmas(hDevice);

    // Register for the dma interrupts.
    if (!Error)
        Error = RegisterAllDmaInterrupt(hDevice);

    if (Error)
        DestroyDmaInfo();
    return Error;
}

/**
 * Start the dma transfer from the head request of the dma channels.
 * Thread Safety: Caller responsibilty.
 */
static void StartDmaTransfer(RmDmaChannel *pDmaChannel)
{
    DmaTransReq *pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];

    // The state of the transfer will be running state.
    pCurrReq->State = RmDmaRequestState_Running;

    // Start the dma transfer.
    pDmaChannel->pHwInterface->DmaHwStartTransferFxn(&pCurrReq->DmaChanRegs);
}

/**
 * Stop the current transfer on dma channel immediately.
 *
 * Thread Safety: It is caller responsibility.
 */
static void StopDmaTransfer(RmDmaChannel *pDmaChannel)
{
    // Get the curent request of the dma channel.
    DmaTransReq *pCurrReq = NULL;
    if (pDmaChannel->HeadReqIndex != DMA_NULL_INDEX)
    {
        pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex];
        if (pCurrReq->State == RmDmaRequestState_Running)
        {
            pDmaChannel->pHwInterface->DmaHwStopTransferFxn(&pCurrReq->DmaChanRegs);
            pCurrReq->State = RmDmaRequestState_Stopped;
        }
    }
}

/**
 * Set the mode of the data transfer whether this is once mode or continuous mode
 * or single buffering or double buffering mode.
 */
static void
SetApbDmaSpecialTransferMode(
    NvRmDmaHandle hDma,
    NvBool IsSourceAddPerip,
    DmaTransReq *pCurrReq)
{
    // Special mode of dma transfer is not supported for the low priority channel.
    if (hDma->pDmaChannel->Priority == NvRmDmaPriority_Low)
        return;

    // For I2s the continuous double buffering is selected.
    if (hDma->DmaReqModuleId == NvRmDmaModuleID_I2s ||
        hDma->DmaReqModuleId == NvRmDmaModuleID_Spdif)
    {
        pCurrReq->TransferMode |=  (RmDmaTransferMode_Continuous |
                                   RmDmaTransferMode_DoubleBuff);
        hDma->pDmaChannel->pHwInterface->DmaHwSetTransferModeFxn(
                            &pCurrReq->DmaChanRegs, NV_TRUE, NV_TRUE);
        pCurrReq->hOnHalfDmaCompleteSema = NULL;
        return;
    }

    // For Uart only receive mode is supported in the continuous double transfer
    if ((hDma->DmaReqModuleId == NvRmDmaModuleID_Uart) && (IsSourceAddPerip))
    {
        pCurrReq->TransferMode |=  (RmDmaTransferMode_Continuous |
                                   RmDmaTransferMode_DoubleBuff |
                                   RmDmaTransferMode_SameBuff);
        hDma->pDmaChannel->pHwInterface->DmaHwSetTransferModeFxn(
                            &pCurrReq->DmaChanRegs, NV_TRUE, NV_TRUE);
        pCurrReq->hOnHalfDmaCompleteSema = pCurrReq->hOnDmaCompleteSema;
        return;
    }
}

/**
 * Configure the current request of the apb dma transfer into the request
 * struture.
 *
 * It validates the source and destination address for the dma transfers.
 * It validates the address wrap and get the address wrapping value.
 * It sets the ahp/apb address as per dma request.
 * It set the direction of transfer and destination bit swap.
 *
 * It break the dma transfer size in multiple transfer if the request transfer
 * size is more than supported transfer size of one dma transfer.
 * Thread Safety: Not required as it will not access any shared informations.
 *
 */
static NvError LogApbDmaTransferRequest(NvRmDmaHandle hDma, void *pCurrRequest)
{
    NvBool IsSourceAddPerip;
    NvBool IsDestAddPerip;
    NvBool IsDoubleBuff;
    DmaTransReq *pCurrReq = (DmaTransReq *)pCurrRequest;

    // Find which address is the Perip address.
    IsSourceAddPerip = NvRmPrivDmaHwIsValidPeripheralAddress(pCurrReq->SourceAdd);
    IsDestAddPerip = NvRmPrivDmaHwIsValidPeripheralAddress(pCurrReq->DestAdd);

    // Only one of the address should be Peripheral address to use the apb dma.
    if (((IsSourceAddPerip == NV_TRUE) && (IsDestAddPerip == NV_TRUE)) ||
        ((IsSourceAddPerip == NV_FALSE) && (IsDestAddPerip == NV_FALSE)))
    {
        return NvError_NotSupported;
    }

    if (IsSourceAddPerip)
        pCurrReq->TransferMode |= RmDmaTransferMode_SourcePeripheral;

    // Configure for address wrapping of the dma register as per source and
    // destination address wrapping of this transfer request.
    hDma->pDmaChannel->pHwInterface->DmaHwSetAddressWrappingFxn(
                                &pCurrReq->DmaChanRegs, pCurrReq->SourceAddWrap,
                                pCurrReq->DestAddWrap, pCurrReq->BytesRequested,
                                IsSourceAddPerip);

    // Configure for source and destination address for data transfer.
    hDma->pDmaChannel->pHwInterface->DmaHwConfigureAddressFxn(
                                &pCurrReq->DmaChanRegs, pCurrReq->SourceAdd,
                                pCurrReq->DestAdd, IsSourceAddPerip);

    // Configure the dma register for direction of transfer as per
    // source/destination address of this transfer request and dma direction
    hDma->pDmaChannel->pHwInterface->DmaHwSetDirectionFxn(&pCurrReq->DmaChanRegs,
                        IsSourceAddPerip);

    if (pCurrReq->TransferMode & RmDmaTransferMode_Asynch)
        SetApbDmaSpecialTransferMode(hDma, IsSourceAddPerip, pCurrReq);

    // Configure the dma register as per the clients byte swap infrmation
    // It will swap for destination only
    if (hDma->IsBitSwapEnable)
        hDma->pDmaChannel->pHwInterface->DmaHwEnableDestBitSwapFxn(
                                &pCurrReq->DmaChanRegs, IsDestAddPerip);

    // Configure the dma register for the burst size. This is calculated based
    // on the requested transfer size.
    hDma->pDmaChannel->pHwInterface->DmaHwSetBurstSizeFxn(&pCurrReq->DmaChanRegs,
                        hDma->DmaReqModuleId, pCurrReq->BytesRequested);

    // Configure the dma register for the transfer bytes count. The requested
    // transfer size can go on many dma transfer cycles.
    pCurrReq->BytesCurrProgram = NV_MIN(pCurrReq->BytesRequested, DMA_MAX_TRANSFER_SIZE);
    pCurrReq->BytesRemaining = pCurrReq->BytesRequested - pCurrReq->BytesCurrProgram;

    IsDoubleBuff = (pCurrReq->TransferMode & RmDmaTransferMode_DoubleBuff)? NV_TRUE: NV_FALSE;
    hDma->pDmaChannel->pHwInterface->DmaHwSetTransferSizeFxn(&pCurrReq->DmaChanRegs,
                pCurrReq->BytesCurrProgram, IsDoubleBuff);
    return NvSuccess;
}


/**
 * Initialize the NvRm dma informations and allocates all resources.
 */
NvError NvRmPrivDmaInit(NvRmDeviceHandle hDevice)
{

    s_ApbDmaInterface.DmaContinueRemainingTransferFxn = ApbDmaContinueRemainingTransfer;
    s_ApbDmaInterface.LogDmaTransferRequestFxn = LogApbDmaTransferRequest;

    NvRmPrivDmaInitDmaHwInterfaces(&s_ApbDmaInterface);

    // Create the dma information.
    return CreateDmaInfo(hDevice);
}

/**
 * Deinitialize the NvRm dma informations and frees all resources.
 */
void NvRmPrivDmaDeInit(void)
{
    DestroyDmaInfo();
}


/**
 * Get the RmDma capabilities.
 */
NvError
NvRmDmaGetCapabilities(
    NvRmDeviceHandle hDevice,
    NvRmDmaCapabilities *pRmDmaCaps)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(pRmDmaCaps);
    pRmDmaCaps->DmaAddressAlignmentSize = DMA_ADDRESS_ALIGNMENT;
    pRmDmaCaps->DmaGranularitySize = DMA_TRANSFER_SIZE_ALIGNMENT;
    return NvSuccess;
}

/**
 * Allocate the dma handles.
 *
 * Implementation Details:
 * For high priority dma handle, it allocated from the available free channel.
 * If there is not the free channel then it reutrns error. The high priority dma
 * requestor client owns the dma channel. Such channel will not be shared by
 * other clients.
 *
 * For low priority dma handle, it allocates the handle from the low priotity
 * channel. The allocation of hande only fails if there is unsufficient memory
 * to allocate the  handle. The low priority dma requestor client share the
 * channel with other clients which is requested for the lower priority dma and
 * so it can suffer the delayed response.
 *
 * Validation of the parameter:
 * It allocates the memory for the dma handle and if memory allocation fails then
 * it return error.
 *
 * Thread safety: Thread safety is provided by locking the mutex for the dma
 * data. This will avoid to access the dma data by the other threads. This is
 * require because it allocate the channel for high priority.
 *
 */
NvError
NvRmDmaAllocate(
    NvRmDeviceHandle  hRmDevice,
    NvRmDmaHandle    *phDma,
    NvBool            Enable32bitSwap,
    NvRmDmaPriority   Priority,
    NvRmDmaModuleID   DmaRequestorModuleId,
    NvU32             DmaRequestorInstanceId)
{
    NvError Error = NvSuccess;

    NvU32 UniqueId;
    RmDmaChannel *pDmaChannel = NULL;
    NvRmDmaHandle hNewDma = NULL;
    RmDmaChannel *pChannelList = NULL;
    NvU32 ChanIndex;

    NV_ASSERT(hRmDevice);
    NV_ASSERT(phDma);

    // Do not allow mem->mem DMAs, which use AHB DMA;
    NV_ASSERT(DmaRequestorModuleId != NvRmDmaModuleID_Memory);

    *phDma = NULL;

    if ((DmaRequestorModuleId == NvRmDmaModuleID_Invalid) ||
        (DmaRequestorModuleId >= NvRmDmaModuleID_Max))
    {
        return NvError_InvalidSourceId;
    }

    // Create the unique Id for each allocation based on requestors
    UniqueId = ((DmaRequestorModuleId << 24) | (DmaRequestorInstanceId << 16) |
                    (NvRmDmaModuleID_Memory << 8));

    // Allocate the memory for the new dma handle.
    hNewDma = NvOsAlloc(sizeof(*hNewDma));

    // If memory allocation fails then it will return error
    if (!hNewDma)
        return NvError_InsufficientMemory;

    // Initialize the allocated memory area with 0
    NvOsMemset(hNewDma, 0, sizeof(*hNewDma));

    // Log all requestor information in the dma handle for future reference.
    hNewDma->DmaReqModuleId = DmaRequestorModuleId;
    hNewDma->DmaReqInstId = DmaRequestorInstanceId;
    hNewDma->IsBitSwapEnable = Enable32bitSwap;
    hNewDma->hRmDevice = hRmDevice;
    hNewDma->UniqueId = UniqueId;
    hNewDma->pDmaChannel = NULL;
    hNewDma->hSyncSema = NULL;

    // Create the semaphore for synchronous semaphore allocation.
    Error = NvOsSemaphoreCreate(&hNewDma->hSyncSema, 0);

    // If error the free the allocation and return error.
    if (Error)
        goto ErrorExit;

    // Configure the dma channel configuration registers as per requestor.
    s_ApbDmaInterface.DmaHwInitRegistersFxn(&hNewDma->DmaChRegs,
                            DmaRequestorModuleId, DmaRequestorInstanceId);

    // If it is the high priority dma request then allocate the channel from
    // free available channel.Otherwise it will return the handle and will
    // share the channel across the clients. All clients with low priority dma
    // requestor will use the low priority channel.

    // For high priority dma channel request, use the free channel. And for low
    // priority channel use the used channel low priority channels.
    pChannelList = s_DmaInfo.pListApbDmaChannel;

    // Going to access the data which is shared across the different thread.
    NvOsMutexLock(s_DmaInfo.hDmaAllocMutex);

    for (ChanIndex = 0; ChanIndex < NvRmDmaUnreservedChannels(); ++ChanIndex)
    {
        pDmaChannel = &pChannelList[ChanIndex];
        if ((Priority == pDmaChannel->Priority) && (pDmaChannel->ChannelState == RmDmaChannelState_Free))
            break;
        pDmaChannel = NULL;
    }

    // If the dma channel is null then it is error.
    if (!pDmaChannel)
    {
        NvOsMutexUnlock(s_DmaInfo.hDmaAllocMutex);
        Error = NvError_DmaChannelNotAvailable;
        goto ErrorExit;
    }

    // If got the free channel for the high priority then mark at used.
    if (NvRmDmaPriority_High == Priority)
        pDmaChannel->ChannelState = RmDmaChannelState_Used;

    NvOsMutexUnlock(s_DmaInfo.hDmaAllocMutex);

    // Attach the dma channel in the dma handle.
    hNewDma->pDmaChannel = pDmaChannel;
    hNewDma->DmaChRegs.pHwDmaChanReg = pDmaChannel->pVirtChannelAdd;

    *phDma = hNewDma;
    return Error;

ErrorExit:
    NvOsSemaphoreDestroy(hNewDma->hSyncSema);
    NvOsFree(hNewDma);
    return Error;
}


/**
 * Free the dma handle which is allocated to the user.
 * Implementation Details:
 * For high priority dma handle, mark the channel free if it has pending
 * transfer request. If the there is no pending request then release the channel
 * and add in the free list so that it will be allocated to the other clients.
 *
 * For Low priority dma handle, it deletes the handle only. The low priority dma
 * requestor does not own the channel so the channel will not be added in the
 * free list.
 *
 * Thread safety: Done inside the functions.
 *
 */
void NvRmDmaFree(NvRmDmaHandle hDma)
{
    RmDmaChannel *pDmaChannel = NULL;

    // If it is null handle then return.
    if (!hDma)
        return;

    // Get the dma channels.
    pDmaChannel = hDma->pDmaChannel;

    // For high priority dma handle, mark the channel is free.
    // For Low priority dma handle, it deletes the handle only. The low priority
    // dma requestor does not own the channel.

    if (NvRmDmaPriority_High == pDmaChannel->Priority)
    {
        // Thread safety: Avoid any request for this channel
        NvOsIntrMutexLock(pDmaChannel->hIntrMutex);

        // If there is a transfer request then mark channel as free but does not
        // free the channel now. This channel will be free after last transfer
        // is done.
        // If there is no pending transfer request then free this channel
        // immediately so that it will be available for the next allocation.
        if (pDmaChannel->HeadReqIndex != DMA_NULL_INDEX)
            pDmaChannel->ChannelState = RmDmaChannelState_MarkedFree;
        else
        {
            // Thread Safety: Lock the channel allocation data base to avoid the
            // access by other threads
            pDmaChannel->ChannelState = RmDmaChannelState_Free;
        }
        NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    }

    // Release the semaphore created for the synchronous operation.
    NvOsSemaphoreDestroy(hDma->hSyncSema);

    // Free the dma channels.
    NvOsFree(hDma);
}


/**
 * Start the dma transfer. It queued the rwueste if there is already reueet on
 * the dma channel.
 * It supports the synchrnous and asynchrnous request both.
 *
 * For sync opeartion, it will wait till timeout or till data transfer completes,
 * whichever happens first.
 *
 * For asynch operation it queued the request, start if no data transfer is
 * going on the channel and return to the caller. This is the caller
 * resposibility to synchrnoise the request. On completion, it will signal the
 * semaphore which was passed alongwith request.
 * If no sempahor is passed then also it queued the request but after
 * completion it will not signal the semaphore.
 *
 * Thread safety: The thread safety is provided inside the function.
 *
 */

NvError
NvRmDmaStartDmaTransfer(
    NvRmDmaHandle       hDma,
    NvRmDmaClientBuffer *pClientBuffer,
    NvRmDmaDirection    DmaDirection,
    NvU32               WaitTimeoutInMS,
    NvOsSemaphoreHandle       AsynchSemaphoreId)
{
    DmaTransReq  *pCurrReq = NULL;
    RmDmaChannel *pDmaChannel = NULL;
    NvOsSemaphoreHandle hOnCompleteSema = NULL;
    NvOsSemaphoreHandle hClonedSemaphore = NULL;
    NvError Error = NvSuccess;
    NvU16 FreeIndex;
    NvU16 PrevIndex;
    NvU16 NextIndex;

    NV_ASSERT(hDma);
    NV_ASSERT(pClientBuffer);

    // Get the dma info and the dma channel and validate that it shoudl not be
    // null
    pDmaChannel = hDma->pDmaChannel;

    // Validate for the source and destination address alignment.
    NV_ASSERT(!(pClientBuffer->SourceBufferPhyAddress & (DMA_ADDRESS_ALIGNMENT-1)));
    NV_ASSERT(!(pClientBuffer->DestinationBufferPhyAddress & (DMA_ADDRESS_ALIGNMENT-1)));

    // Validate for the transfer size granularity level.
    NV_ASSERT(!(pClientBuffer->TransferSize & (DMA_TRANSFER_SIZE_ALIGNMENT-1)));

    //Log the notification parameters after completion.
    if (WaitTimeoutInMS)
    {
         hOnCompleteSema = hDma->hSyncSema;
    }
    else
    {
        if (AsynchSemaphoreId)
        {
            Error = NvOsSemaphoreClone(AsynchSemaphoreId, &hClonedSemaphore);
            if (Error)
                return Error;
            hOnCompleteSema = hClonedSemaphore;
        }
    }

    NvOsIntrMutexLock(pDmaChannel->hIntrMutex);
    if (pDmaChannel->HeadFreeIndex == DMA_NULL_INDEX)
    {
        Error = AllocateReqList(pDmaChannel, pDmaChannel->MaxReqList);
        if (Error)
            goto Exit;
    }

    pCurrReq = &pDmaChannel->pTransReqList[pDmaChannel->HeadFreeIndex];

    // Delete the semaphore which was cloned during the last req by this list.
    NvOsSemaphoreDestroy(pCurrReq->hLastReqSema);
    pCurrReq->hLastReqSema = NULL;


    // Configure the request infromation.
    pCurrReq->UniqueId = hDma->UniqueId;
    pCurrReq->TransferMode = RmDmaTransferMode_PingIntMode;
    pCurrReq->State = RmDmaRequestState_NotStarted;
    pCurrReq->hOnDmaCompleteSema = hOnCompleteSema;
    pCurrReq->hOnHalfDmaCompleteSema = NULL;

    if (!WaitTimeoutInMS)
        pCurrReq->TransferMode |= RmDmaTransferMode_Asynch;

    if (DmaDirection ==  NvRmDmaDirection_Forward)
    {
       pCurrReq->SourceAdd = pClientBuffer->SourceBufferPhyAddress;
       pCurrReq->DestAdd = pClientBuffer->DestinationBufferPhyAddress;
       pCurrReq->SourceAddWrap = pClientBuffer->SourceAddressWrapSize;
       pCurrReq->DestAddWrap = pClientBuffer->DestinationAddressWrapSize;
    }
    else
    {
       pCurrReq->SourceAdd = pClientBuffer->DestinationBufferPhyAddress;
       pCurrReq->DestAdd = pClientBuffer->SourceBufferPhyAddress;;
       pCurrReq->SourceAddWrap = pClientBuffer->DestinationAddressWrapSize;
       pCurrReq->DestAddWrap = pClientBuffer->SourceAddressWrapSize;
    }

    pCurrReq->BytesRequested = pClientBuffer->TransferSize;
    pCurrReq->BytesCurrProgram = 0;
    pCurrReq->BytesRemaining = 0;

    // Copy the Client related information from register to the current request.
    pCurrReq->DmaChanRegs.ControlReg = hDma->DmaChRegs.ControlReg;
    pCurrReq->DmaChanRegs.AhbSequenceReg = hDma->DmaChRegs.AhbSequenceReg;
    pCurrReq->DmaChanRegs.ApbSequenceReg = hDma->DmaChRegs.ApbSequenceReg;
    pCurrReq->DmaChanRegs.XmbSequenceReg = hDma->DmaChRegs.XmbSequenceReg;
    pCurrReq->DmaChanRegs.pHwDmaChanReg = hDma->pDmaChannel->pVirtChannelAdd;


    // Configure registers as per current data request.
    Error = hDma->pDmaChannel->pHwInterface->LogDmaTransferRequestFxn(hDma, pCurrReq);
    if (Error)
        goto Exit;

    // Adding the request on the list
    FreeIndex = pDmaChannel->HeadFreeIndex;
    pDmaChannel->HeadFreeIndex = pDmaChannel->pTransReqList[pDmaChannel->HeadFreeIndex].NextIndex;

    PrevIndex = pDmaChannel->TailReqIndex;
    if (pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
    {
        pDmaChannel->HeadReqIndex = FreeIndex;
        pDmaChannel->TailReqIndex = FreeIndex;
        pDmaChannel->pTransReqList[FreeIndex].NextIndex = DMA_NULL_INDEX;
        StartDmaTransfer(pDmaChannel);
    }
    else
    {
        pDmaChannel->pTransReqList[pDmaChannel->TailReqIndex].NextIndex = FreeIndex;
        pDmaChannel->pTransReqList[FreeIndex].NextIndex = DMA_NULL_INDEX;
        pDmaChannel->pTransReqList[FreeIndex].PrevIndex = pDmaChannel->TailReqIndex;
        pDmaChannel->TailReqIndex = FreeIndex;
    }

    // If asynchronous operation then return.
    if (!WaitTimeoutInMS)
    {
        pCurrReq->hLastReqSema = hClonedSemaphore;
        goto Exit;
    }
    NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);

    // Not worrying about the wait error as the state of the request will decide
    // the status of the transfer.
    (void)NvOsSemaphoreWaitTimeout(hOnCompleteSema, WaitTimeoutInMS);

    // Lock the channel to access the request.
    NvOsIntrMutexLock(pDmaChannel->hIntrMutex);

    // Check for the state of the current transfer.
    switch (pCurrReq->State)
    {
        case RmDmaRequestState_NotStarted :
            // Free the req list.
            NextIndex = pDmaChannel->pTransReqList[FreeIndex].NextIndex;
            pDmaChannel->pTransReqList[FreeIndex].NextIndex = pDmaChannel->HeadFreeIndex;
            pDmaChannel->HeadFreeIndex = FreeIndex;
            if (PrevIndex == DMA_NULL_INDEX)
            {
                pDmaChannel->HeadReqIndex = NextIndex;
                if (NextIndex == DMA_NULL_INDEX)
                    pDmaChannel->TailReqIndex = DMA_NULL_INDEX;
            }
            else
            {
                pDmaChannel->pTransReqList[PrevIndex].NextIndex = NextIndex;
                if (NextIndex != DMA_NULL_INDEX)
                    pDmaChannel->pTransReqList[NextIndex].PrevIndex = PrevIndex;
            }
            Error =  NvError_Timeout;
            break;

        case RmDmaRequestState_Running:
            // Current transfer is running so stop it now.
            StopDmaTransfer(pDmaChannel);
            if (pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].NextIndex
                            == DMA_NULL_INDEX)
            {
                pDmaChannel->HeadReqIndex = DMA_NULL_INDEX;
                pDmaChannel->TailReqIndex = DMA_NULL_INDEX;
            }
            else
            {
                pDmaChannel->HeadReqIndex = pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].NextIndex;
            }
            pDmaChannel->pTransReqList[FreeIndex].NextIndex = pDmaChannel->HeadFreeIndex;
            pDmaChannel->HeadFreeIndex = FreeIndex;

            // if there is more request then Start the transfer now.
            if (pDmaChannel->HeadReqIndex != DMA_NULL_INDEX)
                StartDmaTransfer(pDmaChannel);
            Error =  NvError_Timeout;
            break;


        case RmDmaRequestState_Completed:
            // If transfer is completed then transfer state will be NvSuccess;
            Error =  NvSuccess;
            break;

        default:
            NV_ASSERT(!"Client Request is in the invalid state");
            break;
    }

Exit:
    NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    if (Error)
        NvOsSemaphoreDestroy(hClonedSemaphore);

    return Error;
}

/**
 * It Immediately stop the dma transfer in the channel, delete all the request
 * from the queue,
 * Free all the memory of requests.
 *
 * Thread safety: During killing of all request, the channel specific data
 * access is locked to avoid the access of these data by the other thread.
 * This provide the thread safety.
 *
 * For async queued request, the semaphore Id which was passed with start
 * transfer request are not destroyed. This is the caller responsibility to
 * destroy all the semaphore which was passed.
 *
 */
void NvRmDmaAbort(NvRmDmaHandle hDma)
{
    NvU16 ReqIndex;
    NvU16 NextIndex;
    NvU16 PrevIndex;
    RmDmaChannel *pDmaChannel = NULL;
    NvBool IsRequireToStart = NV_FALSE;

    // If null dma handle then return.
    if (!hDma)
        return;

    // Get the dma channel pointer and if its null pointer then return.
    pDmaChannel = hDma->pDmaChannel;

    // The process of killing all the request is depends on the priority of the
    // dma.
    if (NvRmDmaPriority_High == pDmaChannel->Priority)
    {
        // Stop the dma transfer.
        StopDmaTransfer(pDmaChannel);

        // Kill all request
        // Lock the channel related data base to avoid the access by other
        // client.
        NvOsIntrMutexLock(pDmaChannel->hIntrMutex);

        ReqIndex = pDmaChannel->HeadReqIndex;
        while (ReqIndex != DMA_NULL_INDEX)
        {
            NextIndex = pDmaChannel->pTransReqList[ReqIndex].NextIndex;
            if (pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema)
            {
                NvOsSemaphoreDestroy(pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema);
                pDmaChannel->pTransReqList[ReqIndex].hLastReqSema = NULL;
                pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema = NULL;
            }

            if (pDmaChannel->HeadFreeIndex != DMA_NULL_INDEX)
                pDmaChannel->pTransReqList[ReqIndex].NextIndex = pDmaChannel->HeadFreeIndex;
            pDmaChannel->HeadFreeIndex = ReqIndex;
            ReqIndex = NextIndex;
        }
        pDmaChannel->HeadReqIndex = DMA_NULL_INDEX;
        pDmaChannel->TailReqIndex = DMA_NULL_INDEX;

        // Unlock the channel related data base so that it can be access by
        // other client
        NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    }
    else
    {
        // Lock the channel access mutex.
        NvOsIntrMutexLock(pDmaChannel->hIntrMutex);

        // Check whether the abort request is for current running transfer
        // or not. The identification is done based on unique Id.
        IsRequireToStart = NV_FALSE;
        if (pDmaChannel->pTransReqList[pDmaChannel->HeadReqIndex].UniqueId ==
                                                            hDma->UniqueId)
        {
            // The request need to be abort so stop the dma channel.
            StopDmaTransfer(pDmaChannel);
            IsRequireToStart = NV_TRUE;
        }

        ReqIndex = pDmaChannel->HeadReqIndex;
        PrevIndex = DMA_NULL_INDEX;
        while (ReqIndex != DMA_NULL_INDEX)
        {
            NextIndex = pDmaChannel->pTransReqList[ReqIndex].NextIndex;
            if (pDmaChannel->pTransReqList[ReqIndex].UniqueId == hDma->UniqueId)
            {
                if (pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema)
                {
                    NvOsSemaphoreDestroy(pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema);
                    pDmaChannel->pTransReqList[ReqIndex].hLastReqSema = NULL;
                    pDmaChannel->pTransReqList[ReqIndex].hOnDmaCompleteSema = NULL;
                }
                if (PrevIndex != DMA_NULL_INDEX)
                    pDmaChannel->pTransReqList[PrevIndex].NextIndex = NextIndex;

                if (NextIndex == DMA_NULL_INDEX)
                    pDmaChannel->TailReqIndex = PrevIndex;
                else
                    pDmaChannel->pTransReqList[NextIndex].PrevIndex = PrevIndex;
                pDmaChannel->pTransReqList[ReqIndex].NextIndex = pDmaChannel->HeadFreeIndex;
                pDmaChannel->HeadFreeIndex = ReqIndex;
            }
            PrevIndex = ReqIndex;
            if (pDmaChannel->HeadReqIndex == ReqIndex)
                    pDmaChannel->HeadReqIndex = NextIndex;
            ReqIndex = NextIndex;
        }
        if (pDmaChannel->HeadReqIndex != DMA_NULL_INDEX)
        {
            if (IsRequireToStart)
                StartDmaTransfer(pDmaChannel);
        }
        // Unlock the channel access mutex.
        NvOsIntrMutexUnlock(pDmaChannel->hIntrMutex);
    }
}

#define DEBUG_GET_COUNT 0
NvError NvRmDmaGetTransferredCount(
    NvRmDmaHandle hDma,
    NvU32 *pTransferCount,
    NvBool IsTransferStop )
{
    DmaTransReq  *pCurrReq = NULL;
    NvError Error = NvSuccess;
#if DEBUG_GET_COUNT
    NvBool IsPrint = NV_TRUE;
#endif

    NV_ASSERT(hDma);
    NV_ASSERT(pTransferCount);

    NvOsIntrMutexLock(hDma->pDmaChannel->hIntrMutex);

    if (hDma->pDmaChannel->HeadReqIndex == DMA_NULL_INDEX)
    {
        *pTransferCount = hDma->pDmaChannel->LastReqSize;
#if DEBUG_GET_COUNT
        NvOsDebugPrintf("RmDmaGetTransCount ERROR1\n");
#endif
        goto ErrorExit;
    }

    pCurrReq = &hDma->pDmaChannel->pTransReqList[hDma->pDmaChannel->HeadReqIndex];
    if ((pCurrReq->State != RmDmaRequestState_Running) &&
            (pCurrReq->State != RmDmaRequestState_Stopped))
    {
        Error = NvError_InvalidState;
#if DEBUG_GET_COUNT
        NvOsDebugPrintf("RmDmaGetTransCount ERROR\n");
#endif
        goto ErrorExit;
    }

    if (IsTransferStop)
    {
        if (pCurrReq->State == RmDmaRequestState_Running)
        {
            *pTransferCount = hDma->pDmaChannel->pHwInterface->DmaHwGetTransferredCountWithStopFxn(
                                &pCurrReq->DmaChanRegs, NV_TRUE);
            pCurrReq->State = RmDmaRequestState_Stopped;
            hDma->pDmaChannel->pHwInterface->DmaHwStopTransferFxn(&pCurrReq->DmaChanRegs);
        }
        else
        {
            *pTransferCount = hDma->pDmaChannel->pHwInterface->DmaHwGetTransferredCountFxn(
                                        &pCurrReq->DmaChanRegs);
        }
    }
    else
    {
        if (pCurrReq->State == RmDmaRequestState_Stopped)
        {
            pCurrReq->State = RmDmaRequestState_Running;
            hDma->pDmaChannel->pHwInterface->DmaHwStartTransferFxn(&pCurrReq->DmaChanRegs);
            *pTransferCount = 0;
#if DEBUG_GET_COUNT
                IsPrint = NV_FALSE;
#endif
        }
        else
        {
            *pTransferCount = hDma->pDmaChannel->pHwInterface->DmaHwGetTransferredCountFxn(
                                        &pCurrReq->DmaChanRegs);
        }
    }

#if DEBUG_GET_COUNT
          NvOsDebugPrintf("RmDmaGetTransCount() TransferCount 0x%08x \n", *pTransferCount);
#endif

ErrorExit:
    NvOsIntrMutexUnlock(hDma->pDmaChannel->hIntrMutex);
    return Error;
}

NvBool NvRmDmaIsDmaTransferCompletes(
    NvRmDmaHandle hDma,
    NvBool IsFirstHalfBuffer)
{
    // This API is not supported in the os level driver.
    NV_ASSERT(0);
    return NV_FALSE;
}


NvError NvRmPrivDmaSuspend()
{
    // Global disable the dma channels.
    s_ApbDmaInterface.DmaHwGlobalSetFxn(s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd,
                                                        NV_FALSE);
    // Disables clocks
    (void)NvRmPowerModuleClockControl(s_DmaInfo.hDevice, NvRmPrivModuleID_ApbDma,
                                                        0, NV_FALSE);
    return NvSuccess;
}

NvError NvRmPrivDmaResume()
{
    // Global enable the dma channels.
    s_ApbDmaInterface.DmaHwGlobalSetFxn(s_DmaInfo.ApbDmaGenReg.pGenVirtBaseAdd,
                                                        NV_TRUE);
    // Enables clocks
    (void)NvRmPowerModuleClockControl(s_DmaInfo.hDevice, NvRmPrivModuleID_ApbDma,
                                                        0, NV_TRUE);
    return NvSuccess;
}
