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
 *           NvDDK NAND APIs</b>
 *
 * @b Description: Declares Interface for NvDDK NAND module.
 *
 */

#include "nvddk_nand.h"
#include "nvodm_query_nand.h"
#include "ap20/arnandflash.h"
#include "nvrm_hardware_access.h"
#include "nvrm_power.h"
#include "nvrm_module.h"
#include "nvrm_gpio.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_memmgr.h"
#include "nvrm_interrupt.h"
#include "nvrm_pinmux.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_nand.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pmu.h"

#if NV_OAL
    #define IS_CQ_ENABLED 0
#else
    #define IS_CQ_ENABLED 1
#endif


// Enable following define for verifying write operations on to Nand
#define WRITE_VERIFY 0
#define NAND_RANDOM_FAILURES 0
#define NUMBER_OF_ITERATIONS_BEFORE_ERROR 100
#define ENABLE_INTERNAL_COPYBACK 0
#define NAND_MAX_BYTES_PER_PAGE 4096

// For internal Debug purpose and to display messages selectively.
#define DEBUG_NAND 0
NvU8 DebugPrintEnable = 0;
#if DEBUG_NAND
#define NAND_TIME_STAMP 0
#define NAND_DISP_INTS 0
#define NAND_DISP_ERROR 1
#define NAND_DISP_REG 1
#define NAND_DISPLAY_ALL 0
#define NAND_ASSRT_ERRORS 0
#else
#define NAND_TIME_STAMP 0
#define NAND_DISP_INTS 0
#define NAND_DISP_ERROR 1
#define NAND_DISP_REG 1
#define NAND_DISPLAY_ALL 0
#define NAND_ASSRT_ERRORS 0
#endif
#if NAND_DISPLAY_ALL
    #define PRINT_ALL(X) NvOsDebugPrintf X
#else
    #define PRINT_ALL(X)
#endif
#if NAND_DISP_ERROR
    #define PRINT_ERROR(X) NvOsDebugPrintf X
#else
    #define PRINT_ERROR(X)
#endif
#if NAND_DISP_INTS
    #define PRINT_INTS(X) NvOsDebugPrintf X
#else
    #define PRINT_INTS(X)
#endif
#if NAND_DISP_REG
    #define PRINT_REG(X) NvOsDebugPrintf X
#else
    #define PRINT_REG(X)
#endif
#if NAND_ASSRT_ERRORS
    #define NAND_ASSERT(e) \
        do \
        { \
            if (e != NvSuccess) \
            { \
                NV_ASSERT(NV_FALSE); \
                NvOsDebugPrintf("\r\n Nand Assert Err = 0x%x", e); \
            }\
        }while (0)
#else
    #define NAND_ASSERT(e)
#endif

#if NV_OAL
    #define NAND_DDK_ENABLE_DMA_POLLING_MODE 1
#else
    #define NAND_DDK_ENABLE_DMA_POLLING_MODE 0
#endif
#if NAND_DDK_ENABLE_DMA_POLLING_MODE
    #define NAND_DDK_ENABLE_COMMAND_POLLING_MODE 1
#else
    #define NAND_DDK_ENABLE_COMMAND_POLLING_MODE 0
#endif

// Wait time out in mili seconds
#define NAND_COMMAND_TIMEOUT_IN_MS 1000
/*
    These constants are used to get the Bus Width, Page Size, Block Size and
    the Redundant Area Size from ReadID
*/
#define DDK_NAND_ID_DECODE_0_BUS_WIDTH_RANGE            30:30
#define DDK_NAND_ID_DECODE_0_BLOCK_SIZE_RANGE           29:28
#define DDK_NAND_ID_DECODE_0_REDUNDANT_AREA_SIZE_RANGE  26:26
#define DDK_NAND_ID_DECODE_0_PAGE_SIZE_RANGE            25:24

#define DDK_42NM_NAND_ID_DECODE_0_BLOCK_SIZE_MSB_RANGE           31:31
#define DDK_42NM_NAND_ID_DECODE_0_REDUNDANT_AREA_SIZE_RANGE      27:26
#define DDK_42NM_NAND_ID_DECODE_0_REDUNDANT_AREA_SIZE_MSB_RANGE  30:30
// Some of the golden constant values
enum GoldenValues
{
    // Timing register value that works with most of the Nand flashes.
    // This value is used for identifying nand flash.
    TIMING_VALUE = 0x3F0BD214, // 0x3F2BD214 can be used for Ap15. not for Ap10,
    TIMING2_VALUE = 0xB,
    // Physical Buffers need to be Word Aligned
    NAND_BUFFER_ALIGNMENT = 32,
    // Nand Flash operation success status. valid for most of the flashes
    SUCCESS_STATUS = 0x40,
    // Max round trip delay for Non EDO modes
    MAX_ROUND_TRIP_DELAY = 13
};

enum 
{
    // maximum erase attempts before returning failure
    DDK_NAND_MAX_ERASE_RETRY = 3
};

typedef enum
{
    NandOperation_Reset = 1,
    NandOperation_ReadId,
    NandOperation_Read,
    NandOperation_Write,
    NandOperation_Erase,
    NandOperation_GetStatus,
    NandOperation_CopybackRead,
    NandOperation_CopybackProgram,
    NandOperation_ReadParamPage,
    NandOperation_DataCyclesAlone,
    NandOperation_Force32 = 0x7FFFFFFF
}NandOperation;

// Structure to hold parameters required for Nand thread to perform
// Read/Write/Copy-back operations.
typedef struct NandParamsRec
{
    // Flash chip number on to which the requested operation is to take place.
    NvU8 DeviceNumber;
    // Destination device number for copyback.
    NvU8 DstnDeviceNumber;
    NvU32 StartPageNumber;
    // The starting page number of the Nand flash for the requested opeartion.
    // this will be considered as source page in case of copy bacck operations.
    NvU32* pStartPageNumbers;
    // The destination page number of the Nand flash for copy back opeartion.
    // for Read/ Write operations this param is don't care.
    NvU32* pDstnPageNumbers;
    // Holds the column number.
    NvU32 ColumnNumber;
    // Number of spare area bytes to read/write.
    NvU32 NumSpareAreaBytes;
    // Client buffer for receiving or transmitting the page data.
    NvU8* pDataBuffer;
    // Client buffer for receiving or transmitting the spare data.
    NvU8* pTagBuffer;
    // Holds the number of pages to read/write.
    NvU32 NumberOfPages;
    // Returns the number of pages completed.
    NvU32 NumberOfPagesCompleted;
    // The type of nand operation requested - Read/Write/Copy back.
    NandOperation OperationName;
    // time out for the requested operation in milli seconds.
    NvU32 WaitTimeoutInMilliSeconds;
    // Semaphore Id that needs to signalled once the operation is complete.
    NvOsSemaphoreHandle SemaphoreId;
}NandParams;

// params to be passed for allocating physical buffer from SDRAM
typedef struct SdramBufferParamsRec
{
    // Virtual Buffer Pointer used to hold the pointer to the buffer
    NvU8* pVirtualBuffer;
    // Physical Buffer Pointer used to hold the pointer to the buffer
    NvRmPhysAddr PhysBuffer;
    // memory handle
    NvRmMemHandle hMem;
    // holds buffer size
    NvU32 BufferSize;
}SdramBufferParams;

// Defines the type of algorithm for error-correcting code (ECC).
typedef enum
{
    // Specifies Hamming ECC.
    ECCAlgorithm_Hamming = 0,
    // Specifies Reed-Solomon ECC.
    ECCAlgorithm_ReedSolomon,
    // Specifies BCH ECC.
    ECCAlgorithm_BCH,
    // Ecc disabled
    ECCAlgorithm_None,
    ECCAlgorithm_Force32 = 0x7FFFFFFF
}ECCAlgorithm;

// structure for Nand handle.
typedef struct NvDdkNandRec
{
    // Holds a flag indicating whether Nand controller is open and initialized.
    NvBool IsNandOpen;
    // Nand Controller registers physical base address
    NvRmPhysAddr pBaseAddress;
    // Holds the virtual address for accessing registers.
    NvU32 *pVirtualAddress;
    // Holds the register map size.
    NvU32 BankSize;
    // RM device handle
    NvRmDeviceHandle RmDevHandle;
    // copy of Gpio handle
    NvRmGpioHandle hGpio;
    NvRmGpioPinHandle hWriteProtectPin;
    // Variable used to hold information related ECC algorithm to be used:
    // Hamming or Reed-solomon
    ECCAlgorithm EccAlgorithm;
    // Variable used to hold flash device information.
    NvDdkNandDeviceInfo DevInfo;
    // Variable used to hold the number of active devices on board.
    NvU8 NumOfActiveDevices;
    // variable to hold the RS-T value (number of correctable errors per sector
    NvU8 TValue;
    // Variable used to hold the number of Pages requested for current Nand operation
    NvU32 NumOfPagesRequested;
    // Number of Chips to be interleaved for the current read/ Write oepration.
    NvU32 NumberOfChipsToBeInterleaved;
    // Variable to hold the number of Pages successfully read/ written in
    // current Nand operation
    NvU32 NumOfPagesTransferred;
    // Variable to hold the number of Tag (Spare) area Pages successfully
    // read/ written in current Nand operation.
    NvU32 NumOfTagPagesTransferred;
    // Variable used to hold the information related to the current operation
    // status (from thread).
    NvError OperationStatus;
    // Semaphore used for Synchronizing internal activities of to the ddk.
    NvOsSemaphoreHandle CommandDoneSema;
    // Semaphore to be used for DMA Interrupt signaling.
    NvOsSemaphoreHandle DmaDoneSema;
    // Semaphore to be used for Power Management signaling.purposes
    NvOsSemaphoreHandle PowerMgmtSema;
    // Nand configuration pin-map.
    NvOdmNandPinMap PinMap;
    // Variable to hold various Flash parameter values.
    NvOdmNandFlashParams FlashParams;
    // a flag to indicate the current nand flash operation usage of command
    // queue. If True-use command Q mode else normal mode
    NvBool IsCommandQueueOperation;
    // Gives information about the Nand flash capability to support copy back 
    // operation.
    NvBool IsCopybackSupported;
    // a flag to indicate if the combined rdy/bsy mode is being used.
    NvBool IsCombRbsyMode;
    // A flag to check if command queue error has occured
    NvBool IsCqError;
    // PhysicalDeviceNumber[0] will give chipId / deviceNumber of the first
    // physical nand chip avail on the board.
    NvU8 PhysicalDeviceNumber[NDFLASH_CS_MAX];
    // pointer to nand driver capability structure
    NvDdkNandDriverCapabilities NandCapability;
    // Command queue buffer size
    NvU32 CqBufferSize;
    // Variable to hold the information of Cache write command support
    NvBool IsCacheWriteSupproted;
    // Ecc buffer size
    NvU32 EccBufferSize;
    // structure to hold parameters related to Command queue buffer in SDRAM
    SdramBufferParams CqBuffer;
    // structure to hold parameters related to Ecc buffer in SDRAM
    SdramBufferParams EccBuffer;
    // structure to hold parameters related to Physical data buffer in SDRAM
    SdramBufferParams DataBuffer;
    // structure to hold parameters related to Physical Tag buffer in SDRAM
    SdramBufferParams TagBuffer;
    // Number of lock apertures used for nand flash.
    NvU8 NumberOfAperturesUsed;
    // For each lock aperture, current Aperture Start, Aperture End, and Chip ID
    // register values 
    NvU32 LockAperStart[NDFLASH_CS_MAX];
    NvU32 LockAperEnd[NDFLASH_CS_MAX];
    NvU32 LockAperChipId[NDFLASH_CS_MAX];
    // To store Rm power client Id
    NvU32 RmPowerClientId;
    // To store optimum timing register value.
    NvU32 OptimumTiming;
    NvU32 OptimumTiming2;
    // Maximum number of pages that can bre read/written in one stretch through DMA.
    NvU32 MaxNumOfPagesPerDMARequest;
    // Interrupt handle
    NvOsInterruptHandle InterruptHandle;
    // Frequency set to Nand.
    NvRmFreqKHz FreqInKhz;
    // Indicates whether to signal command done sema.
    volatile NvBool SignalCommandDoneSema;
    // Holds the Ecc error page info.
    volatile NvU32* pEccErrorData;
    volatile NvU32 EccErrorCount;
    NvU32 EccFailedPage;
    NvU8 NandBusWidth;
    NandParams Params;
    NvBool IsNandClkEnabled;
    /* flag to ensure suspend is not executed twice */
    NvBool IsNandSuspended;
    // Flag to check if lock status is read from the device
    NvBool IsLockStatusAvailable;
    // To Hold the error threshold value.
    NvU8 ErrThreshold;
    NvBool IsBCHEccSupported;
    // This flag is set if ONFI Nand is used on CS0. Based on this info other 
    // CS will be checked for ONFI initialization.
    NvBool IsONFINandOnCs0;
    // Peripheral DataBase
    const NvOdmPeripheralConnectivity *pConnectivity;
    // Mutex for thread safety
    NvOsMutexHandle hMutex;
    // Reference count to keep track of number of open calls made.
    NvU32 RefCount;
    // Profiling command issue count
    NvU32 StartCommandCount;
    // To hold the spare area size per each page of the flash
    NvU16 SpareAreaSize;
}NvDdkNand;

// This enum enumerates various bit positions for framing Nand Flash
// Controller Command Queue command word
typedef enum
{
    // presence of Config reg data in the command queue data field
    CqCommand_NandConfig = 0,
    // presence of DMA Config_A reg data in the command queue data field
    CqCommand_NandDmaConfig_A = 1,
    // presence of DMA Config_B reg data in the command queue data field
    CqCommand_NandDmaConfig_B = 2,
    // presence of Data Block Pointer reg data in the command queue data field
    CqCommand_NandDataBlockPtr = 3,
    // presence of Tag Pointer reg data in the command queue data field
    CqCommand_NandTagPtr = 4,
    // presence of ECC Pointer reg data in the command queue data field
    CqCommand_NandEccPtr = 5,
    // presence of DMA Master control reg data in the command queue data field
    CqCommand_NandDmaMstCtrl = 6,
    // presence of Addr Reg1 reg data in the command queue data field
    CqCommand_NandAddrReg1 = 7,
    // presence of Addr Reg2 reg data in the command queue data field
    CqCommand_NandAddrReg2 = 8,
    // presence of Command Reg1 reg data in the command queue data field
    CqCommand_NandCmdReg1 = 9,
    // presence of Command Reg2 reg data in the command queue data field
    CqCommand_NandCmdReg2 = 10,
    // presence of Hardware Status Command reg data in the command queue data field
    CqCommand_NandHwStatusCmd = 11,
    // presence of Hardware Status Mask reg data in the command queue data field
    CqCommand_NandHwStatusMask = 12,
    // presence of Command reg data in the command queue data field
    CqCommand_NandCmd = 13,
    // Packet ID of the command in the queue
    CqCommand_PacketId = 24,
    CqCommand_Force32 = 0x7FFFFFFF
}CqCommand;

// This is stucture definition for Nand Flash Controller Command Queue packet
typedef struct NandCqPacket1Rec
{
    // Command queue command word, gives information about what registers
    // to be filled with the following data.
    NvU32 CqCommand;
    // Data to be filled into Nand DMA control register.
    NvU32 NandDmaMstCtrl;
    // Data to be filled into Nand Command register2.
    NvU32 NandCmdReg2;
    // Data to be filled into Nand Command register.
    NvU32 NandCmd;
}NandCqPacket1;

typedef struct NandCqPacket2Rec
{
    // Command queue command word, gives information about what registers
    // to be filled with the following data.
    NvU32 CqCommand;
    // Data to be filled into Nand Address register1.
    NvU32 NandAddrReg1;
    // Data to be filled into Nand Address register2.
    NvU32 NandAddrReg2;
    // Data to be filled into Nand Command register2.
    NvU32 NandCmdReg2;
    // Data to be filled into Nand Command register.
    NvU32 NandCmd;
}NandCqPacket2;

typedef struct NandCqPacket3Rec
{
    // Command queue command word, gives information about what registers
    // to be filled with the following data.
    NvU32 CqCommand;
    // Data to be filled into Nand Command register.
    NvU32 NandCmd;
}NandCqPacket3;

// Defining constants to identify Ddk operations used to print error messages
typedef enum {
    NAND_OP_READ,
    NAND_OP_WRITE,
    NAND_OP_ERASE,
    NAND_OP_CPYBK,
    NAND_OP_NUM,
    NAND_OP_Force32 = 0x7FFFFFFF
} NAND_OP;

/** Static variables */
// Nand ddk structure pointer.
static NvDdkNand* s_pNandRec = NULL;

#if WRITE_VERIFY
static NvU8 s_WriteVerifyBuffer[65536];
#endif

// static functions prototype.//
static NvError
MemAllocBuffer(
    NvDdkNandHandle hNand,
    SdramBufferParams *SdramParams,
    NvU32 Alignment);
static void DestroyMemHandle(SdramBufferParams *SdramParams);
static void DumpRegData(NvDdkNandHandle hNand);
static void SetTimingRegVal(NvDdkNandHandle hNand, NvBool IsCalcRequired);
static NvU32 GetNumOfErrorVectorBytes(NvDdkNandHandle hNand);
static NvU8 GetNumOfParityBytesForMainArea(NvDdkNandHandle hNand);
static void FillPageSize(NvDdkNandHandle hNand, NvU32* ConfigReg);
static void ChipSelectEnable(NvU8 DeviceNumber, NvU32 *CommandReg);
static void StartCqOperation(NvDdkNandHandle hNand);
static void StartNandOperation(NvDdkNandHandle hNand);
static void SetCombRbsyAndEdoModes(NvDdkNandHandle hNand);
static void SetupInterrupt(NvDdkNandHandle hNand, NandOperation Op);
static void CleanInterruptRegisters(NvDdkNandHandle hNand);
static void
SetupAddressAndDeviceReg(
    NvDdkNandHandle hNand,
    NvU32 DevNum,
    NvU32 StartPageNumber);
static void SetupRegisters(NvDdkNandHandle hNand, NandOperation Op);
static NvError NandRead(NvDdkNandHandle hNand, NvBool IgnoreEccError);
static void
SkipUnusedDevices(NvDdkNandHandle hNand,
    NvU32* pPageNumbers,
    NvU8* pStartDeviceNum,
    NvU32* pOffset);
static NvError NandCopyback(NvDdkNandHandle hNand, NvBool IgnoreEccError);
#if ENABLE_INTERNAL_COPYBACK
static NvError NandInternalCopyback(NvDdkNandHandle hNand);
#endif
static NvError EnableNandClock(NvDdkNandHandle hNand);
static NvError NvDdkNandPrivSetPinMux(NvDdkNandHandle hNand);
static NvError InitNandController(NvDdkNandHandle hNand);
static void SetupDMA(NvDdkNandHandle hNand);
static void
SetCommandQueueOperationState(
    NvDdkNandHandle hNand,
    NvU32 NumberOfPages,
    NandParams *p);
static NvU32 GetColumnNumber(NvDdkNandHandle hNand);
static NvError WaitForCqDone(NvDdkNandHandle hNand);
static void GetNumOfCsInterleaved(NvDdkNandHandle hNand, NvU32 *pPageNumbers);
static NvError WaitForDmaDone(NvDdkNandHandle hNand);
static NvError WaitForCommandDone(NvDdkNandHandle hNand);
static NvError NandCheckForEccError(NvDdkNandHandle hNand, NvU32* Offset);
static NvError NandWrite(NvDdkNandHandle hNand);
static NvError 
NandReadID(
    NvDdkNandHandle hNand,
    NvU8 DevNumber,
    NvU32* ReadID,
    NvBool IsOnfiNand);
#if NAND_DDK_ENABLE_DMA_POLLING_MODE
static void NandWaitUS(NvU32 usec);
static NvError NandWaitDmaDone(NvDdkNandHandle hNand);
static NvError NandWaitCqDone(NvDdkNandHandle hNand);
#endif
static NvError NandWaitCommandDone(NvDdkNandHandle hNand);
static NvError
RegisterNandInterrupt(
    NvRmDeviceHandle hDevice,
    NvDdkNandHandle hNand);
static void ClearNandFifos(NvDdkNandHandle hNand);
static void NandIsr(void* args);
static NvError NandDisableWriteProtect(NvDdkNandHandle hNand);
static void NandLoadLockCfg(NvDdkNandHandle hNand);
static void NandRestoreLocks(NvDdkNandHandle hNand);
static void
NandPrivLockInterruptService(
    NvDdkNandHandle hNand,
    NvU32 InterruptStatusRegister);
static void SetupCqPkt(NvDdkNandHandle hNand);
static NvError GetOperationStatus(NvDdkNandHandle hNand, NvU8 DeviceNumber);
static NvU8 GetBitPosition(NvU32 Number);
static void NandPowerRailEnable(NvDdkNandHandle hNand, NvBool IsEnable);
static void NandWaitUS(NvU32 usec);
static NvError ResetNandFlash(NvDdkNandHandle hNand, const NvU8 DeviceNumber);

// Macro definitions //
// Nand register read macro
#define Nand_REGR(hNand, reg) \
    NV_READ32(hNand->pVirtualAddress + ((NAND_##reg##_0) / 4))
// Nand register write macro
#define Nand_REGW(hNand, reg, data) \
do \
{ \
    NV_WRITE32(hNand->pVirtualAddress + ((NAND_##reg##_0) / 4), (data)); \
}while (0)

// Nand register read macro with offset.
#define Nand_REGR_OFFSET(hNand, reg, OffsetInBytes) \
    NV_READ32(hNand->pVirtualAddress + (OffsetInBytes / 4) + \
        ((NAND_##reg##_0) / 4))

// Nand register write macro with offset.
#define Nand_REGW_OFFSET(hNand, reg, OffsetInBytes, data) \
do \
{ \
    NV_WRITE32(hNand->pVirtualAddress + (OffsetInBytes / 4) + \
        ((NAND_##reg##_0) / 4), (data)); \
}while (0)

// Macro to get the difference between two numbers
#define DIFF(T1, T2) \
    (((T1) > (T2)) ? ((T1) - (T2)) : ((T2) - (T1)))

// Macro to get the bigger of the two
#define BIGGEROF(A, B) ((A > B) ? A : B)

// Macro to get the biggest of the three numbers
#define BIGGESTOF(A, B, C) BIGGEROF(BIGGEROF(A, B), C)

// static functions definition.//
static NvError
MemAllocBuffer(
    NvDdkNandHandle hNand,
    SdramBufferParams *SdramParams,
    NvU32 Alignment)
{
    NvError e;
    NvRmPhysAddr* PhyBuffer = &(SdramParams->PhysBuffer);
    NvU8** VirBuffer = &(SdramParams->pVirtualBuffer);
    NvU32 Size = SdramParams->BufferSize;

    // The first step to allocate a memory buffer is, allocating a handle for it.
    NV_CHECK_ERROR(NvRmMemHandleCreate(hNand->RmDevHandle,
        &(SdramParams->hMem), Size));
    // After specifying the properties of the memory buffer,
    // it can be allocated.
    NV_CHECK_ERROR_CLEANUP(NvRmMemAlloc(SdramParams->hMem, NULL,
        0, Alignment, NvOsMemAttribute_Uncached));
    // Before the memory buffer is used, it must be pinned.
    *PhyBuffer = NvRmMemPin(SdramParams->hMem);
    // For virtual address, the memory buffer is mapped into the process's
    // address space.
    NV_CHECK_ERROR_CLEANUP(NvRmMemMap(SdramParams->hMem, 0, Size,
        NVOS_MEM_READ_WRITE, (void **)VirBuffer));
    NvOsMemset(*VirBuffer, 0, Size);
    return e;
fail:
    DestroyMemHandle(SdramParams);
    return e;
}

// This function deallocates physical buffer that was allocated in MemAllocBuffer.
static void DestroyMemHandle(SdramBufferParams *SdramParams)
{
    if (SdramParams->hMem)
    {
        // UnMap the handle
        if (SdramParams->pVirtualBuffer)
            NvRmMemUnmap(SdramParams->hMem, SdramParams->pVirtualBuffer,
                SdramParams->BufferSize);
        SdramParams->pVirtualBuffer = NULL;
        // Unpin the memory allocation.
        NvRmMemUnpin(SdramParams->hMem);
        // Free the memory handle.
        NvRmMemHandleFree(SdramParams->hMem);
        SdramParams->hMem = NULL;
    }
}

// Function to compare byte by byte and print details if mismatched
NvU32 NandUtilMemcmp(const void *pSrc, const void *pDst, NvU32 Size)
{
    NvU32 i;
    NvU32 MismatchCount = 0;
    for (i = 0; i< Size; i++)
    {
        if ((*((NvU8 *)pSrc + i)) != (*((NvU8 *)pDst + i)))
        {
            // mismatch between compared byte sequences
            if (!MismatchCount)
            {
                PRINT_ALL(("\n[index: Wr,Rd] "));
            }
            PRINT_ALL((" [%d: 0x%x,0x%x] ", i,
                (*((NvU8 *)pSrc + i)), (*((NvU8 *)pDst + i))));
            MismatchCount++;
        }
    }
    if (MismatchCount)
    {
        PRINT_ALL(("\nMismatch: %d/%d ", MismatchCount, Size));
    }
    return MismatchCount;
}

// function to check power of 2
static NvBool
UtilCheckPowerOf2(NvU32 Num)
{
    // A power of 2 satisfies condition (N & (N - 1)) == (2 * N - 1)
    if ((Num & (Num - 1)) == 0)
        return NV_TRUE;
    else
        return NV_FALSE;
}

// Simple function to get log2, assumed value power of 2, else return 
// returns log2 of immediately smaller number
NvU8
NandUtilGetLog2(NvU32 Val)
{
    NvU8 Log2Val = 0;
    NvU32 i;
    // Value should be non-zero
    NV_ASSERT(Val > 0);
    if (UtilCheckPowerOf2(Val) == NV_FALSE)
    {
        NvOsDebugPrintf("\nCalling simple log2 with value which is "
            "not power of 2 ");
        // In case of values that are not power of 2 we return the 
        // integer part of the result of log2
    }
    // Value is power of 2
    if (Val > 0)
    {
        // Assumed that Val is NvU32
        for (i = 0; i < 32; i++)
        {
            // divide by 2
            Val = MACRO_DIV_POW2_LOG2NUM(Val, 1);
            if (Val == 0)
            {
                // Return 0 when Val is 1
                break;
            }
            Log2Val++;
        }
    }
    return Log2Val;
}

static void DumpRegData(NvDdkNandHandle hNand)
{
    PRINT_REG(("====== Register Dump Start =========\n"));
    PRINT_REG((" Start command count=0x%x\n", hNand->StartCommandCount));
#if NV_OAL
    // bootloader version of print does not support %8.8x
    PRINT_REG((" NAND_COMMAND = 0x%x\n", Nand_REGR(hNand, COMMAND)));
    PRINT_REG((" NAND_STATUS = 0x%x\n", Nand_REGR(hNand, STATUS)));
    PRINT_REG((" NAND_ISR = 0x%x\n", Nand_REGR(hNand, ISR)));
    PRINT_REG((" NAND_IER = 0x%x\n", Nand_REGR(hNand, IER)));
    PRINT_REG((" NAND_CONFIG = 0x%x\n", Nand_REGR(hNand, CONFIG)));
    PRINT_REG((" NAND_TIMING = 0x%x\n", Nand_REGR(hNand, TIMING)));
    PRINT_REG((" NAND_RESP = 0x%x\n", Nand_REGR(hNand, RESP)));
    PRINT_REG((" NAND_TIMING2 = 0x%x\n", Nand_REGR(hNand, TIMING2)));
    PRINT_REG((" NAND_CMD_REG1 = 0x%x\n", Nand_REGR(hNand, CMD_REG1)));
    PRINT_REG((" NAND_CMD_REG2 = 0x%x\n", Nand_REGR(hNand, CMD_REG2)));
    PRINT_REG((" NAND_ADDR_REG1 = 0x%x\n", Nand_REGR(hNand, ADDR_REG1)));
    PRINT_REG((" NAND_ADDR_REG2 = 0x%x\n", Nand_REGR(hNand, ADDR_REG2)));
    PRINT_REG((" NAND_DMA_MST_CTRL = 0x%x\n", Nand_REGR(hNand, DMA_MST_CTRL)));
    PRINT_REG((" NAND_DMA_CFG.A = 0x%x\n", Nand_REGR(hNand, DMA_CFG_A)));
    PRINT_REG((" NAND_DMA_CFG.B = 0x%x\n", Nand_REGR(hNand, DMA_CFG_B)));
    PRINT_REG((" NAND_FIFO_CTRL = 0x%x\n", Nand_REGR(hNand, FIFO_CTRL)));
    PRINT_REG((" NAND_DATA_BLOCK_PTR = 0x%x\n", Nand_REGR(hNand, DATA_BLOCK_PTR)));
    PRINT_REG((" NAND_TAG_PTR = 0x%x\n", Nand_REGR(hNand, TAG_PTR)));
    PRINT_REG((" NAND_ECC_PTR = 0x%x\n", Nand_REGR(hNand, ECC_PTR)));
    PRINT_REG((" NAND_DEC_STATUS = 0x%x\n", Nand_REGR(hNand, DEC_STATUS)));
    PRINT_REG((" NAND_HWSTATUS_CMD = 0x%x\n", Nand_REGR(hNand, HWSTATUS_CMD)));
    PRINT_REG((" NAND_HWSTATUS_MASK = 0x%x\n", Nand_REGR(hNand, HWSTATUS_MASK)));
    PRINT_REG((" NAND_LL_CONFIG = 0x%x\n", Nand_REGR(hNand, LL_CONFIG)));
    PRINT_REG((" NAND_LL_PTR = 0x%x\n", Nand_REGR(hNand, LL_PTR)));
    PRINT_REG((" NAND_LL_STATUS = 0x%x\n", Nand_REGR(hNand, LL_STATUS)));
#else
    PRINT_REG((" NAND_COMMAND = 0x%8.8x\n", Nand_REGR(hNand, COMMAND)));
    PRINT_REG((" NAND_STATUS = 0x%8.8x\n", Nand_REGR(hNand, STATUS)));
    PRINT_REG((" NAND_ISR = 0x%8.8x\n", Nand_REGR(hNand, ISR)));
    PRINT_REG((" NAND_IER = 0x%8.8x\n", Nand_REGR(hNand, IER)));
    PRINT_REG((" NAND_CONFIG = 0x%8.8x\n", Nand_REGR(hNand, CONFIG)));
    PRINT_REG((" NAND_TIMING = 0x%8.8x\n", Nand_REGR(hNand, TIMING)));
    PRINT_REG((" NAND_RESP = 0x%8.8x\n", Nand_REGR(hNand, RESP)));
    PRINT_REG((" NAND_TIMING2 = 0x%8.8x\n", Nand_REGR(hNand, TIMING2)));
    PRINT_REG((" NAND_CMD_REG1 = 0x%8.8x\n", Nand_REGR(hNand, CMD_REG1)));
    PRINT_REG((" NAND_CMD_REG2 = 0x%8.8x\n", Nand_REGR(hNand, CMD_REG2)));
    PRINT_REG((" NAND_ADDR_REG1 = 0x%8.8x\n", Nand_REGR(hNand, ADDR_REG1)));
    PRINT_REG((" NAND_ADDR_REG2 = 0x%8.8x\n", Nand_REGR(hNand, ADDR_REG2)));
    PRINT_REG((" NAND_DMA_MST_CTRL = 0x%8.8x\n", Nand_REGR(hNand, DMA_MST_CTRL)));
    PRINT_REG((" NAND_DMA_CFG.A = 0x%8.8x\n", Nand_REGR(hNand, DMA_CFG_A)));
    PRINT_REG((" NAND_DMA_CFG.B = 0x%8.8x\n", Nand_REGR(hNand, DMA_CFG_B)));
    PRINT_REG((" NAND_FIFO_CTRL = 0x%8.8x\n", Nand_REGR(hNand, FIFO_CTRL)));
    PRINT_REG((" NAND_DATA_BLOCK_PTR = 0x%8.8x\n", Nand_REGR(hNand, DATA_BLOCK_PTR)));
    PRINT_REG((" NAND_TAG_PTR = 0x%8.8x\n", Nand_REGR(hNand, TAG_PTR)));
    PRINT_REG((" NAND_ECC_PTR = 0x%8.8x\n", Nand_REGR(hNand, ECC_PTR)));
    PRINT_REG((" NAND_DEC_STATUS = 0x%8.8x\n", Nand_REGR(hNand, DEC_STATUS)));
    PRINT_REG((" NAND_HWSTATUS_CMD = 0x%8.8x\n", Nand_REGR(hNand, HWSTATUS_CMD)));
    PRINT_REG((" NAND_HWSTATUS_MASK = 0x%8.8x\n", Nand_REGR(hNand, HWSTATUS_MASK)));
    PRINT_REG((" NAND_LL_CONFIG = 0x%8.8x\n", Nand_REGR(hNand, LL_CONFIG)));
    PRINT_REG((" NAND_LL_PTR = 0x%8.8x\n", Nand_REGR(hNand, LL_PTR)));
    PRINT_REG((" NAND_LL_STATUS = 0x%8.8x\n", Nand_REGR(hNand, LL_STATUS)));
#endif
    PRINT_REG(("====== Register Dump End ===========\n"));
}

static NvU32 CalcTcsVal(NvDdkNandHandle hNand, NvU32 Twp, NvU32 Twh)
{
    NvU32 TcsCntMax = 3;  // TCS_CNT has 2 bits assigned in the TRM register spec.
    NvBool IsSetupTimeSet;
    NvOdmNandFlashParams* FP = &(hNand->FlashParams);
    NvU16 MaxSetupTime = BIGGESTOF(FP->TCLS, FP->TALS, FP->TCS);
    NvU16 MaxHoldTime = BIGGESTOF(FP->TCLH, FP->TALH, FP->TCH);
    NvU32 TcsCnt = 0;
    NvU32 Tcs = 0;

    for(TcsCnt = 0; TcsCnt <= TcsCntMax; TcsCnt ++)
    {
        IsSetupTimeSet = NV_FALSE;

        if((((TcsCnt + Twp + 2) * 1000000) / hNand->FreqInKhz) >=MaxSetupTime)
        {
            IsSetupTimeSet = NV_TRUE;
        }
        else
        {
            continue;
        }

        if ((((TcsCnt + Twh + 3) * 1000000) / hNand->FreqInKhz) >=MaxHoldTime)
       {
           if (IsSetupTimeSet == NV_TRUE)
           {
               Tcs = TcsCnt;
               break;
           }
        }
    }
    return Tcs;
}

static void SetTimingRegVal(NvDdkNandHandle hNand, NvBool IsCalcRequired)
{
    NvU32 TimingRegVal = 0;
    NvU32 Tcs = 0;
    NvU32 Twp = 0;
    NvU32 Twh = 0;
    NvU32 MaxOfTrrTarTcr = 0;
    NvU32 Trp = 0;
    NvU32 Trp_resp = 0;
    NvOdmNandFlashParams* FParams = &(hNand->FlashParams);
    // This Macro Converts time in nano seconds to cycle count bitfield value.
    // The 't' comes in nano seconds.
    // Bit field Value '0' means '1' cycle. So, subtract '1'.
    if (IsCalcRequired)
    {
        #define CNT(t) \
            (((((t) * hNand->FreqInKhz) + 1000000 - 1) / 1000000) - 1)
        // calculating Max of TRR_TAR_TCR
        MaxOfTrrTarTcr = BIGGESTOF(FParams->TCR, FParams->TAR, FParams->TRR);
        // Calculate Trp and Trp_resp values
        // Trp:: EDO mode: tRP timing from flash datasheet 
        // Trp:: Non-EDO mode: Max(tRP, tREA) timing + 13ns (round trip delay) 
        if (hNand->NandCapability.IsEdoModeSupported)
        {
            Trp = FParams->TRP;
            Trp_resp = FParams->TREA;
        }
        else
        {
            Trp = BIGGEROF(FParams->TRP, FParams->TREA) + MAX_ROUND_TRIP_DELAY;
            Trp_resp = Trp;
        }
        // Adjust Trp for TRC <= (TRP + TRH)
        if (FParams->TRC > (Trp + FParams->TRH))
        {
            Trp = FParams->TRC - FParams->TRH;
        }
        // Adjust Twp for TWC <= (TWP + TWH)
        if (FParams->TWC > (FParams->TWP + FParams->TWH))
        {
            FParams->TWP = FParams->TWC - FParams->TWH;
        }
        // Calculating Tcs
        Twp = CNT(FParams->TWP);
        Twh = CNT(FParams->TWH);
        Tcs = CalcTcsVal(hNand, Twp, Twh);

        TimingRegVal = NV_DRF_NUM(NAND, TIMING, TRP_RESP_CNT, CNT(Trp_resp)) |
                       NV_DRF_NUM(NAND, TIMING, TWB_CNT, CNT(FParams->TWB)) |
                       NV_DRF_NUM(NAND, TIMING, TCR_TAR_TRR_CNT,
                            (CNT(MaxOfTrrTarTcr) - 2))|
                       NV_DRF_NUM(NAND, TIMING, TWHR_CNT, CNT(FParams->TWHR)) |
                       NV_DRF_NUM(NAND, TIMING, TCS_CNT, Tcs) |
                       NV_DRF_NUM(NAND, TIMING, TWH_CNT, Twh) |
                       NV_DRF_NUM(NAND, TIMING, TWP_CNT, Twp) |
                       NV_DRF_NUM(NAND, TIMING, TRH_CNT, CNT(FParams->TRH)) |
                       NV_DRF_NUM(NAND, TIMING, TRP_CNT, CNT(Trp));
        hNand->OptimumTiming = TimingRegVal;
        hNand->OptimumTiming2 = NV_DRF_NUM(NAND, TIMING2, TADL_CNT,
                                    (CNT(FParams->TADL) - 2));
        #undef CNT
    }
    // Set Nand timing reg values.
    Nand_REGW(hNand, TIMING, hNand->OptimumTiming);
    Nand_REGW(hNand, TIMING2, hNand->OptimumTiming2);
}

static NvU32 GetNumOfErrorVectorBytes(NvDdkNandHandle hNand)
{
    /*
     * Following are the arrays that contain the number of error vectors for
     * various algorithms selected with different pages sizes (in KB) as Index.
    */
    const NvU32 HammingErrorVectorBytes[] = {4, 8, 16, 0, 32};
    const NvU32 Rs4ErrorVectorBytes[] = {32, 64, 128, 0, 256};
    const NvU32 Rs6ErrorVectorBytes[] = {48, 96, 192, 0, 384};
    const NvU32 Rs8ErrorVectorBytes[] = {64, 128, 256, 0, 512};

    NvU32 ErrorVectorIndex = 0;
    NvU32 NumOfErrorVectorBytes = 0;
    NvU32 MaxNumberOfCorrectableErrors = 0;

    ErrorVectorIndex = hNand->DevInfo.PageSize / 1024;
    if ((hNand->EccAlgorithm == ECCAlgorithm_BCH) ||
        (hNand->EccAlgorithm == ECCAlgorithm_None))
    {
        // for BCH Ecc algorithm vector allocation space not required
        return 0;
    }
    else if (hNand->EccAlgorithm == ECCAlgorithm_Hamming)
    {
        return HammingErrorVectorBytes[ErrorVectorIndex];
    }
    else if (hNand->EccAlgorithm == ECCAlgorithm_ReedSolomon)
    {
        if (hNand->TValue == 0)
        {
            MaxNumberOfCorrectableErrors = 4;
            NumOfErrorVectorBytes = Rs4ErrorVectorBytes[ErrorVectorIndex];
        }
        else if (hNand->TValue == 1)
        {
            MaxNumberOfCorrectableErrors = 6;
            NumOfErrorVectorBytes = Rs6ErrorVectorBytes[ErrorVectorIndex];
        }
        else if (hNand->TValue == 2)
        {
            MaxNumberOfCorrectableErrors = 8;
            NumOfErrorVectorBytes = Rs8ErrorVectorBytes[ErrorVectorIndex];
        }
    }
    // Set error threshold to (MaxCorrectableErrors - 1) per every 512 
    // bytes so that to know if a page is developing uncorrectable 
    // number of errors for the selected T-value.
    hNand->ErrThreshold = MaxNumberOfCorrectableErrors - 1;

    if (NumOfErrorVectorBytes)
        return NumOfErrorVectorBytes;
    else
    {
        // If we reached here means either page size or ECC algorithm selected are
        // not supported.
        NV_ASSERT(NV_FALSE);
        return 512;
    }
}

static NvU8 GetNumOfParityBytesForMainArea(NvDdkNandHandle hNand)
{
    const NvU8 HammingParityBytes[] = {0,
                                       NDFLASH_PARITY_SZ_HAMMING_1024,
                                       NDFLASH_PARITY_SZ_HAMMING_2048,
                                       0,
                                       NDFLASH_PARITY_SZ_HAMMING_4096};
    const NvU8 Rs4ParityBytes[] = {0,
                                   NDFLASH_PARITY_SZ_RS_T4_1024,
                                   NDFLASH_PARITY_SZ_RS_T4_2048,
                                   0,
                                   NDFLASH_PARITY_SZ_RS_T4_4096};
    const NvU8 Rs6ParityBytes[] = {0,
                                   NDFLASH_PARITY_SZ_RS_T6_1024,
                                   NDFLASH_PARITY_SZ_RS_T6_2048,
                                   0,
                                   NDFLASH_PARITY_SZ_RS_T6_4096};
    const NvU8 Rs8ParityBytes[] = {0,
                                   NDFLASH_PARITY_SZ_RS_T8_1024,
                                   NDFLASH_PARITY_SZ_RS_T8_2048,
                                   0,
                                   NDFLASH_PARITY_SZ_RS_T8_4096};
    const NvU8 Bch4ParityBytes[] = {NDFLASH_PARITY_SZ_BCH_T4_512,
                                   2 * NDFLASH_PARITY_SZ_BCH_T4_512,
                                   4 * NDFLASH_PARITY_SZ_BCH_T4_512,
                                   0,
                                   8 * NDFLASH_PARITY_SZ_BCH_T4_512};
    const NvU8 Bch8ParityBytes[] = {NDFLASH_PARITY_SZ_BCH_T8_512,
                                   2 * NDFLASH_PARITY_SZ_BCH_T8_512,
                                   4 * NDFLASH_PARITY_SZ_BCH_T8_512,
                                   0,
                                   8 * NDFLASH_PARITY_SZ_BCH_T8_512};
    const NvU8 Bch14ParityBytes[] = {NDFLASH_PARITY_SZ_BCH_T14_512,
                                   2 * NDFLASH_PARITY_SZ_BCH_T14_512,
                                   4 * NDFLASH_PARITY_SZ_BCH_T14_512,
                                   0,
                                   8 * NDFLASH_PARITY_SZ_BCH_T14_512};
    const NvU8 Bch16ParityBytes[] = {NDFLASH_PARITY_SZ_BCH_T16_512,
                                   2 * NDFLASH_PARITY_SZ_BCH_T16_512,
                                   4 * NDFLASH_PARITY_SZ_BCH_T16_512,
                                   0,
                                   8 * NDFLASH_PARITY_SZ_BCH_T16_512};
    NvU32 ParityIndex;

    NV_ASSERT(hNand->DevInfo.PageSize);

    if (hNand->EccAlgorithm == ECCAlgorithm_None)
        return 0;
    if (hNand->NandCapability.IsEccSupported)
    {
        ParityIndex = hNand->DevInfo.PageSize / 1024;
        if (hNand->EccAlgorithm == ECCAlgorithm_BCH)
        {
            if (hNand->TValue == 0)
                return Bch4ParityBytes[ParityIndex];
            else if (hNand->TValue == 1)
                return Bch8ParityBytes[ParityIndex];
            else if (hNand->TValue == 2)
                return Bch14ParityBytes[ParityIndex];
            else if (hNand->TValue == 3)
                return Bch16ParityBytes[ParityIndex];
        }
        else if (hNand->EccAlgorithm == ECCAlgorithm_Hamming)
        {
            return HammingParityBytes[ParityIndex];
        }
        else if (hNand->EccAlgorithm == ECCAlgorithm_ReedSolomon)
        {
            if (hNand->TValue == 0)
                return Rs4ParityBytes[ParityIndex];
            else if (hNand->TValue == 1)
                return Rs6ParityBytes[ParityIndex];
            else if (hNand->TValue == 2)
                return Rs8ParityBytes[ParityIndex];
        }
    }
    // If we reached here means either page size or ECC algorithm selected are
    // not supported.
    NV_ASSERT(NV_FALSE);
    return 0;
}

static void FillPageSize(NvDdkNandHandle hNand, NvU32* ConfigReg)
{
    const NvU8 PageSizeSelectArray[] = {0, 0, 1, 0, 2, 0, 0, 0, 3,
                                        0, 0, 0, 0, 0, 0, 0, 4};
    // Get the index by dividing by 256.
    NvU8 Index = hNand->DevInfo.PageSize >> 8;
    NV_ASSERT((hNand->DevInfo.PageSize > 0) &&
        (hNand->DevInfo.PageSize <= 4096));

    *ConfigReg |= NV_DRF_NUM(NAND, CONFIG, PAGE_SIZE_SEL,
                      PageSizeSelectArray[Index]);
}

static void ChipSelectEnable(NvU8 DeviceNumber, NvU32 *CommandReg)
{
    NvU32 CommandRegisterValue = *CommandReg;
    NV_ASSERT(DeviceNumber < NDFLASH_CS_MAX);
    // To clear Chip select field value.
    CommandRegisterValue &= (~(0xFF << NAND_COMMAND_0_CE0_SHIFT));
    // To fill required chip select value
    CommandRegisterValue |= (1 << (NAND_COMMAND_0_CE0_SHIFT + DeviceNumber));
    *CommandReg = CommandRegisterValue;
}

static void SetCombRbsyAndEdoModes(NvDdkNandHandle hNand)
{
    NvU32 RegVal = 0;
    RegVal = Nand_REGR(hNand, CONFIG);
    if (hNand->IsCombRbsyMode)
        RegVal |= NV_DRF_DEF(NAND, CONFIG, COM_BSY, ENABLE);
    if (hNand->NandCapability.IsEdoModeSupported)
        RegVal |= NV_DRF_NUM(NAND, CONFIG, EDO_MODE, 1);
    Nand_REGW(hNand, CONFIG, RegVal);
}

static void StartCqOperation(NvDdkNandHandle hNand)
{
    NvU32 RegVal = 0;
    SetCombRbsyAndEdoModes(hNand);
    SetupCqPkt(hNand);
    // Start Command queue operation.
    RegVal = Nand_REGR(hNand, LL_CONFIG);
    RegVal |= NV_DRF_NUM(NAND, LL_CONFIG, LL_START, 1);
    Nand_REGW(hNand, LL_CONFIG, RegVal);
}

static void StartNandOperation(NvDdkNandHandle hNand)
{
    NvU32 RegVal = 0;
    // increment command issue count
    hNand->StartCommandCount++;
    SetCombRbsyAndEdoModes(hNand);
    // Signal nand controller start operation.
    RegVal = Nand_REGR(hNand, COMMAND);
    if (RegVal & NV_DRF_DEF(NAND, COMMAND, GO, ENABLE))
    {
        DumpRegData(hNand);
        NV_ASSERT(NV_FALSE);
    }
    RegVal |= NV_DRF_DEF(NAND, COMMAND, GO, ENABLE);
    Nand_REGW(hNand, COMMAND, RegVal);
}

// Set ISR & IER registers to 0.
static void CleanInterruptRegisters(NvDdkNandHandle hNand)
{
    NvU32 RegVal = 0;

    RegVal = Nand_REGR(hNand, ISR);
    Nand_REGW(hNand, ISR, RegVal);
    Nand_REGW(hNand, IER, 0);
}

#if NAND_DDK_ENABLE_DMA_POLLING_MODE
static void SetupInterrupt(NvDdkNandHandle hNand, NandOperation Op)
{
// For Polling nothing to be done
}
#else
static void SetupInterrupt(NvDdkNandHandle hNand, NandOperation Op)
{
    NvU32 RegVal = 0;
    NvU32 IerReg = 0;

    CleanInterruptRegisters(hNand);
    // Enalbe Global interrupt enable bit.
    IerReg = NV_DRF_NUM(NAND, IER, GIE, 1) |
             NV_DRF_NUM(NAND, IER, IE_UND, 1) |
             NV_DRF_NUM(NAND, IER, IE_OVR, 1);
    if (hNand->EccAlgorithm == ECCAlgorithm_ReedSolomon)
        IerReg |= NV_DRF_NUM(NAND, IER, ERR_TRIG_VAL, hNand->ErrThreshold);

    if (hNand->IsCommandQueueOperation)
    {
        IerReg |= NV_DRF_NUM(NAND, IER, IE_LL_DONE, 1) |
                  NV_DRF_NUM(NAND, IER, IE_LL_ERR, 1) |
                  NV_DRF_NUM(NAND, IER, IE_ECC_ERR, 1);
    }
    else
    {
        if (Op == NandOperation_Read)
            IerReg |= NV_DRF_NUM(NAND, IER, IE_ECC_ERR, 1);
        IerReg |= NV_DRF_NUM(NAND, IER, IE_CMD_DONE, 1);
    }
    if (hNand->NumberOfAperturesUsed)
        RegVal |= NV_DRF_NUM(NAND, LOCK_CONTROL, IE_LOCK_ERR, 1);
    Nand_REGW(hNand, IER, IerReg);
}
#endif

static NvError EnableNandPower(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;
    /* Enable power for Nand module */
    NV_CHECK_ERROR(NvRmPowerVoltageControl(hNand->RmDevHandle,
        NvRmModuleID_Nand, hNand->RmPowerClientId, NvRmVoltsUnspecified,
        NvRmVoltsUnspecified, NULL, 0, NULL));
    NandPowerRailEnable(hNand, NV_TRUE);
    return e;
}

static NvError EnableNandClock(NvDdkNandHandle hNand)
{
    NvError e;
    // Add any more frequencies required here.
    const NvRmFreqKHz PrefFreqList[] = {130000, 108000, 80000, 72000, 48000,
        24000, 12000, 8300, NvRmFreqUnspecified};
    NvU32 ListCount = NV_ARRAY_SIZE(PrefFreqList);
    NvRmFreqKHz CurrentFreq = 0;

    // Enable clock to Nand controller
    NV_CHECK_ERROR(NvRmPowerModuleClockControl(hNand->RmDevHandle,
        NvRmModuleID_Nand, hNand->RmPowerClientId, NV_TRUE));

    // Request for clk
    NV_CHECK_ERROR(NvRmPowerModuleClockConfig(hNand->RmDevHandle,
        NvRmModuleID_Nand, hNand->RmPowerClientId, PrefFreqList[ListCount - 2],
        PrefFreqList[0], PrefFreqList, ListCount, &CurrentFreq, 0));
    hNand->FreqInKhz = CurrentFreq;
    return e;
}

static NvError NvDdkNandPrivSetPinMux(NvDdkNandHandle hNand)
{
    NvError e = NvError_BadValue;
    NvU32 *pPinMuxConfigTable = NULL;
    NvU32 Count = 0;
    NvRmModuleNandInterfaceCaps InterfaceCaps;

    NV_ASSERT(hNand);
    NvOdmQueryPinMux(NvOdmIoModule_Nand, (const NvU32 **)&pPinMuxConfigTable,
        &Count);
    NV_ASSERT(pPinMuxConfigTable);

    if (Count != 0)
    {
        NV_ASSERT_SUCCESS(NvRmGetModuleInterfaceCapabilities(
            hNand->RmDevHandle,
            NVRM_MODULE_ID(NvRmModuleID_Nand,0), 
            sizeof(NvRmModuleNandInterfaceCaps), &InterfaceCaps));
        hNand->IsCombRbsyMode = InterfaceCaps.IsCombRbsyMode;
        hNand->NandBusWidth = InterfaceCaps.NandInterfaceWidth;

        e= NvRmSetModuleTristate(hNand->RmDevHandle,
                            NVRM_MODULE_ID(NvRmModuleID_Nand,0),
                            NV_FALSE);
    }
    return e;
}

static NvError InitNandController(NvDdkNandHandle hNand)
{
    NvError e;

    e = NvDdkNandPrivSetPinMux(hNand);
    if ((e != NvSuccess) && (e != NvError_NotSupported))
    {
        return e;
    }
    // Disable write protect.
    e = NandDisableWriteProtect(hNand);
    // Addresses for write protect disable are finalised.in case of AP15.
    if ((e != NvSuccess) && (e != NvError_NotSupported))
    {
        return e;
    }
    return e;
}

static void SetupDMA(NvDdkNandHandle hNand)
{
    NandParams* p = &hNand->Params;
    NvU32 NumOfBytesData = 0;
    NvU32 ConfigReg = 0;
    NvU32 DmaMasterCtlReg = 0;
    NvU32 DmaConfigAReg = 0;
    NvU32 DmaConfigBReg = 0;
    NvU32 DataBlockPtrReg = 0;
    NvU32 TagBlockPtrReg = 0;
    NvU32 TagBytes;
    NvU32 BchConfigReg = 0;

    DmaMasterCtlReg = NV_DRF_DEF(NAND, DMA_MST_CTRL, BURST_SIZE, BURST_8WORDS) |
                      NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_PERF_EN, 1);
    if (p->OperationName == NandOperation_Write)
        DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DIR, 1);
    else
        DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, REUSE_BUFFER, 1);
    // Setup Configuration register

    if (hNand->EccAlgorithm == ECCAlgorithm_BCH)
    {
        NumOfBytesData = (hNand->DevInfo.PageSize) * p->NumberOfPages;
        // setting DMA data transfer size
        DmaConfigAReg |= NV_DRF_NUM(NAND, DMA_CFG_A, DMA_BLOCK_SIZE_A,
                            (NumOfBytesData - 1));
        // Setting DMA data buffer ptr.
        DataBlockPtrReg = (NvU32)hNand->DataBuffer.PhysBuffer;
        // To enable Main Page Data transfer by DMA.
        DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_EN_A, 1);

        // Enable skip bit
        if (hNand->FlashParams.SkippedSpareBytes !=
            NvOdmNandSkipSpareBytes_0)
        {
            ConfigReg |= NV_DRF_NUM(NAND, CONFIG, SKIP_SPARE, 1) |
                         NV_DRF_NUM(NAND, CONFIG, SKIP_SPARE_SEL,
                            (hNand->FlashParams.SkippedSpareBytes - 1));
        }

        TagBytes = hNand->DevInfo.TagSize - 1;
        ConfigReg |= NV_DRF_NUM(NAND, CONFIG, TAG_BYTE_SIZE, TagBytes);
        if (p->OperationName == NandOperation_Read)
        {
            TagBytes = ((p->NumberOfPages * (hNand->DevInfo.TagSize + 
                (hNand->FlashParams.SkippedSpareBytes << 2))) - 1);
        }
        else
            TagBytes = ((p->NumberOfPages * hNand->DevInfo.TagSize) - 1);
        DmaConfigBReg |= NV_DRF_NUM(NAND, DMA_CFG_B, DMA_BLOCK_SIZE_B, 
            TagBytes);
        // Set TAG pointer.
        TagBlockPtrReg = (NvU32)hNand->TagBuffer.PhysBuffer;
        // To enable Spare Data transfer by DMA.
        DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_EN_B, 1);
    }
    else // RS or Hamming
    {
        if (p->pDataBuffer)
        {
            NumOfBytesData = (hNand->DevInfo.PageSize) * p->NumberOfPages;
            // setting DMA data transfer size
            DmaConfigAReg |= NV_DRF_NUM(NAND, DMA_CFG_A, DMA_BLOCK_SIZE_A,
                                (NumOfBytesData - 1));
            // Setting DMA data buffer ptr.
            DataBlockPtrReg = (NvU32)hNand->DataBuffer.PhysBuffer;
            // To enable Main Page Data transfer by DMA.
            DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_EN_A, 1);

            // Enable skip bit
            if (hNand->FlashParams.SkippedSpareBytes !=
                NvOdmNandSkipSpareBytes_0)
            {
                ConfigReg |= NV_DRF_NUM(NAND, CONFIG, SKIP_SPARE, 1) |
                             NV_DRF_NUM(NAND, CONFIG, SKIP_SPARE_SEL,
                                (hNand->FlashParams.SkippedSpareBytes - 1));
            }
        }

        if (p->pTagBuffer)
        {
            if (hNand->NandCapability.IsEccSupported)
                TagBytes = hNand->DevInfo.TagSize + 
                    NDFLASH_PARITY_SZ_HAMMING_SPARE - 1;
            else
                TagBytes = hNand->DevInfo.TagSize - 1;
            if (p->NumSpareAreaBytes)
                // We are trying to read/write spare area.
                TagBytes = p->NumSpareAreaBytes - 1;
            ConfigReg |= NV_DRF_NUM(NAND, CONFIG, TAG_BYTE_SIZE, TagBytes);
            
            if (p->OperationName == NandOperation_Read)
            {
                if (hNand->NandCapability.IsEccSupported)
                {
                    TagBytes = ((p->NumberOfPages * (hNand->DevInfo.TagSize + 
                        NDFLASH_PARITY_SZ_HAMMING_SPARE)) - 1);
                }
                else
                    TagBytes = ((p->NumberOfPages * hNand->DevInfo.TagSize) - 1);

                // Add Skip Spare bytes.when page data also needs to be read.
                if (p->pDataBuffer)
                {
                    TagBytes += ((hNand->FlashParams.SkippedSpareBytes << 2) * 
                                 p->NumberOfPages);
                }
            }
            else
            {
                TagBytes = (p->NumberOfPages * hNand->DevInfo.TagSize) - 1;
            }
            if (p->NumSpareAreaBytes)
                // We are trying to read/write spare area.
                TagBytes = p->NumSpareAreaBytes - 1;
            
            DmaConfigBReg |= NV_DRF_NUM(NAND, DMA_CFG_B, DMA_BLOCK_SIZE_B,
                TagBytes);
            // Set TAG pointer.
            TagBlockPtrReg = (NvU32)hNand->TagBuffer.PhysBuffer;
            // To enable Spare Data transfer by DMA.
            DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_EN_B, 1);
        }
    }
    // Set config reg params here.
    if ((hNand->NandBusWidth == 16) && (hNand->DevInfo.BusWidth == 16))
        ConfigReg |= NV_DRF_DEF(NAND, CONFIG, BUS_WIDTH, BUS_WIDTH_16);
    else
        ConfigReg |= NV_DRF_DEF(NAND, CONFIG, BUS_WIDTH, BUS_WIDTH_8);

    if (hNand->NandCapability.IsEccSupported == NV_TRUE)
    {
        if (hNand->EccAlgorithm != ECCAlgorithm_BCH)
        {
            if (p->OperationName == NandOperation_Read)
                ConfigReg |= NV_DRF_NUM(NAND, CONFIG, HW_ERR_CORRECTION, 1);
            ConfigReg |= NV_DRF_NUM(NAND, CONFIG, TVALUE, hNand->TValue);
            if (hNand->TValue == 0)
                ConfigReg |= NV_DRF_DEF(NAND, CONFIG, TVALUE, TVAL4);
            else if (hNand->TValue == 1)
                ConfigReg |= NV_DRF_DEF(NAND, CONFIG, TVALUE, TVAL6);
            if (hNand->TValue == 2)
                ConfigReg |= NV_DRF_DEF(NAND, CONFIG, TVALUE, TVAL8);

            if (p->pDataBuffer)
            {
                ConfigReg |= NV_DRF_NUM(NAND, CONFIG, HW_ECC, 1);
            }
            if (p->pTagBuffer)
            {
                if (p->NumSpareAreaBytes == 0)
                    ConfigReg |= NV_DRF_NUM(NAND, CONFIG, ECC_EN_TAG, 1);
            }
            // Enable reed_solomon algorithm by default
            ConfigReg |= NV_DRF_NUM(NAND, CONFIG, ECC_SEL, hNand->EccAlgorithm);
        }
        ConfigReg |= NV_DRF_NUM(NAND, CONFIG, ECC_SEL, hNand->EccAlgorithm);
    }

    FillPageSize(hNand, &ConfigReg);
    ConfigReg |= NV_DRF_NUM(NAND, CONFIG, PIPELINE_EN, 1);
    // Configure DMA MST Cntrl register.
    DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, IS_DMA_DONE, 1) |
                       NV_DRF_NUM(NAND, DMA_MST_CTRL, IE_DMA_DONE, 1);
    if (!hNand->IsCommandQueueOperation)
        DmaMasterCtlReg |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_GO, 1);
    // Finally, Write all required registers.
    // Set ECC buffer pointer.
    Nand_REGW(hNand, ECC_PTR, (NvU32)hNand->EccBuffer.PhysBuffer);
    Nand_REGW(hNand, DATA_BLOCK_PTR, DataBlockPtrReg);
    Nand_REGW(hNand, TAG_PTR, TagBlockPtrReg);
    Nand_REGW(hNand, DMA_CFG_A, DmaConfigAReg);
    Nand_REGW(hNand, DMA_CFG_B, DmaConfigBReg);
    Nand_REGW(hNand, BCH_CONFIG, BchConfigReg);
    Nand_REGW(hNand, CONFIG, ConfigReg);
    Nand_REGW(hNand, DMA_MST_CTRL, DmaMasterCtlReg);
}

static void SetupAddressAndDeviceReg(
    NvDdkNandHandle hNand,
    NvU32 DevNum,
    NvU32 StartPageNumber)
{
    NandParams *p = &hNand->Params;
    NvU32 CommandReg = 0;

    // Setup Address Registers.
    if (p->OperationName != NandOperation_Erase)
    {
        Nand_REGW(hNand, ADDR_REG1, (p->ColumnNumber |
            (StartPageNumber << NAND_ADDR_REG1_0_ADDR_BYTE2_SHIFT)));
        Nand_REGW(hNand, ADDR_REG2,
            (StartPageNumber >> NAND_ADDR_REG1_0_ADDR_BYTE2_SHIFT));
    }
    CommandReg = Nand_REGR(hNand, COMMAND);
    ChipSelectEnable(DevNum, &CommandReg);
    Nand_REGW(hNand, COMMAND, CommandReg);
}

static void
SetupRegisters(
    NvDdkNandHandle hNand,
    NandOperation Op)
{
    NvU32 CommandReg = 0;
    NvU32 StatusMaskReg = 0;
    NvU32 CommandReg1 = 0;
    NvU32 CommandReg2 = 0;
    NvU32 AddressReg1 = 0;
    NvU32 AddressReg2 = 0;
    NvU32 ConfigReg = 0;
    NvU32 StatusCommandReg = NvOdmNandCommandList_Status;
    NandParams *p = &hNand->Params;

    StatusMaskReg = NV_DRF_NUM(NAND, HWSTATUS_MASK, RDSTATUS_MASK, 1)|
                    NV_DRF_NUM(NAND, HWSTATUS_MASK, RDSTATUS_EXP_VAL, 0)|
                    NV_DRF_NUM(NAND, HWSTATUS_MASK, RBSY_MASK,
                        hNand->FlashParams.OperationSuccessStatus)|
                    NV_DRF_NUM(NAND, HWSTATUS_MASK, RBSY_EXP_VAL,
                        hNand->FlashParams.OperationSuccessStatus);

    if (Op == NandOperation_Read)
    {
        CommandReg1 = NvOdmNandCommandList_Read;
        CommandReg2 = NvOdmNandCommandList_Read_Start;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, RX, 1)|
                      NV_DRF_NUM(NAND, COMMAND, SEC_CMD, 1)|
                      NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES5)|
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
        if (hNand->EccAlgorithm == ECCAlgorithm_BCH)
        {
            CommandReg |= NV_DRF_NUM(NAND, COMMAND, B_VALID, 1);
            CommandReg |= NV_DRF_NUM(NAND, COMMAND, A_VALID, 1)|
                NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
        }
        else
        {
            if (p->pTagBuffer)
            {
                CommandReg |= NV_DRF_NUM(NAND, COMMAND, B_VALID, 1);
            }
            if (p->pDataBuffer)
            {
                CommandReg |= NV_DRF_NUM(NAND, COMMAND, A_VALID, 1)|
                    NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
            }
        }
    }
    else if (Op == NandOperation_ReadParamPage)
    {
        CommandReg1 = NvOdmNandCommandList_ONFIReadId;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
            NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
            NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES1);
    }
    else if(Op == NandOperation_DataCyclesAlone)
    {
        CommandReg = NV_DRF_NUM(NAND, COMMAND, A_VALID, 1)|
            NV_DRF_NUM(NAND, COMMAND, RX, 1)|
            NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1)|
            NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
    }
    else if (Op == NandOperation_Write)
    {
        CommandReg1 = NvOdmNandCommandList_Page_Program;
        CommandReg2 = NvOdmNandCommandList_Page_Program_Start;
        // Setup required bits
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, TX, 1)|
                      NV_DRF_NUM(NAND, COMMAND, SEC_CMD, 1)|
                      NV_DRF_NUM(NAND, COMMAND, AFT_DAT, 1)|
                      NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES5)|
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
        if (hNand->EccAlgorithm == ECCAlgorithm_BCH)
        {
            CommandReg |= NV_DRF_NUM(NAND, COMMAND, B_VALID, 1);
            CommandReg |= NV_DRF_NUM(NAND, COMMAND, A_VALID, 1)|
                NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
        }
        else
        {
            if (p->pTagBuffer)
            {
                CommandReg |= NV_DRF_NUM(NAND, COMMAND, B_VALID, 1);
            }
            if (p->pDataBuffer)
            {
                CommandReg |= NV_DRF_NUM(NAND, COMMAND, A_VALID, 1)|
                    NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
            }
        }
    }
    else if (Op == NandOperation_Erase)
    {
        CommandReg1 = NvOdmNandCommandList_Block_Erase;
        CommandReg2 = NvOdmNandCommandList_Block_Erase_Start;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1) |
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1) |
                      NV_DRF_NUM(NAND, COMMAND, SEC_CMD, 1) |
                      // Erase needs only 3 address cycles.to access the block.
                      NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES3) |
                      // Wait for chip to be ready.
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
        // Setup Address Registers.
        AddressReg1 = p->StartPageNumber;
    }
    else if (Op == NandOperation_GetStatus)
    {
        CommandReg1 = NvOdmNandCommandList_Status;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, RX, 1)|
                      NV_DRF_NUM(NAND, COMMAND, PIO, 1)|
                      // Wait for chip to be ready
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
    }
    else if (Op == NandOperation_Reset)
    {
        CommandReg1 = NvOdmNandCommandList_Reset;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, CLE, 1);
    }
    else if (Op == NandOperation_ReadId)
    {
        CommandReg1 = NvOdmNandCommandList_Read_Id;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, RX, 1)|
                      NV_DRF_NUM(NAND, COMMAND, PIO, 1)|
                      // Only 4 bytes are read as part of read ID
                      NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES4);
    }
    else if (Op == NandOperation_CopybackRead)
    {
        CommandReg1 = NvOdmNandCommandList_Read_Cpy_Bck;
        CommandReg2 = NvOdmNandCommandList_Read_Cpy_Bck_Start;
        // Setup required bits.
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, SEC_CMD, 1)|
                      NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES5)|
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
        FillPageSize(hNand, &ConfigReg);
    }
    else if (Op == NandOperation_CopybackProgram)
    {
        CommandReg1 = NvOdmNandCommandList_Copy_Back;
        CommandReg2 = NvOdmNandCommandList_Page_Program_Start;
        // Setup required bits
        CommandReg |= NV_DRF_NUM(NAND, COMMAND, ALE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, CLE, 1)|
                      NV_DRF_NUM(NAND, COMMAND, SEC_CMD, 1)|
                      NV_DRF_DEF(NAND, COMMAND, ALE_BYTE_SIZE, ALE_BYTES5)|
                      NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
        FillPageSize(hNand, &ConfigReg);
    }

    if ((Op != NandOperation_Read) && (Op != NandOperation_Write) &&
         (Op != NandOperation_GetStatus))
    {
        Nand_REGW(hNand, CONFIG, ConfigReg);
    }
    Nand_REGW(hNand, ADDR_REG1, AddressReg1);
    Nand_REGW(hNand, ADDR_REG2, AddressReg2);
    Nand_REGW(hNand, CMD_REG1, CommandReg1);
    Nand_REGW(hNand, CMD_REG2, CommandReg2);
    Nand_REGW(hNand, HWSTATUS_CMD, StatusCommandReg);
    Nand_REGW(hNand, HWSTATUS_MASK, StatusMaskReg);
    Nand_REGW(hNand, COMMAND, CommandReg);
}

static void
SetCommandQueueOperationState(
    NvDdkNandHandle hNand,
    NvU32 NumberOfPages,
    NandParams *p)
{
    hNand->IsCommandQueueOperation = NV_FALSE;
    if (p)
    {
        // For reading Tag info alone, we don't do cq mode.
        if (hNand->NandCapability.IsCommandQueueModeSupported &&
             (p->pTagBuffer == NULL) && (p->NumberOfPages > 1))
            hNand->IsCommandQueueOperation = NV_TRUE;
    }
}

static NvU32 GetColumnNumber(NvDdkNandHandle hNand)
{
    NvU32 ColumnNumber = 0;
    NvU32 ParityBytes = 0;
    NandParams *p = &hNand->Params;
    if (hNand->EccAlgorithm != ECCAlgorithm_BCH)
    {
        ParityBytes = GetNumOfParityBytesForMainArea(hNand);
        if ((!p->pDataBuffer) && p->pTagBuffer)
        {
            // Column Number = Page Size + Parity bytes (in case RS/ Hamming are 
            // selected).+ Skipped Bytes;
            ColumnNumber = hNand->DevInfo.PageSize + ParityBytes + 
                                    (hNand->FlashParams.SkippedSpareBytes << 2);
        }
        // If Flash is of 16 bit bus width, divide column Number by 2.
        if (hNand->DevInfo.BusWidth == 16)
            ColumnNumber = ColumnNumber >> 1;
    }
    return ColumnNumber;
}

static void GetNumOfCsInterleaved(NvDdkNandHandle hNand, NvU32 *pPageNumbers)
{
    NvU32 i = 0;
    hNand->NumberOfChipsToBeInterleaved = 0;
    for (i = 0; i < NDFLASH_CS_MAX; i++)
    {
        if (pPageNumbers[i] != 0xFFFFFFFF)
        {
            hNand->NumberOfChipsToBeInterleaved++;
        }
    }
    NV_ASSERT(hNand->NumberOfChipsToBeInterleaved <= hNand->NumOfActiveDevices);
}

static NvError WaitForCqDone(NvDdkNandHandle hNand)
{
    NvError Error;
#if !NAND_DDK_ENABLE_DMA_POLLING_MODE
    NandParams *p = &hNand->Params;
    NvU32 CqConfig;
    NvU32 CqGo;
    NvU32 CqMaxWaitTime = p->WaitTimeoutInMilliSeconds * 
        hNand->MaxNumOfPagesPerDMARequest;
#endif
#if NAND_DDK_ENABLE_COMMAND_POLLING_MODE
    Error = NandWaitCqDone(hNand);
#else
    Error = NvOsSemaphoreWaitTimeout(hNand->CommandDoneSema, CqMaxWaitTime);
    if (Error == NvError_Timeout)
    {
        CqConfig = Nand_REGR(hNand, LL_CONFIG);
        CqGo = NV_DRF_VAL(NAND, LL_CONFIG, LL_START, CqConfig);
        if (CqGo == 0)
            Error = NvSuccess;
    }
#endif
    NAND_ASSERT(Error);
    if (Error != NvSuccess)
    {
        DumpRegData(hNand);
        return Error;
    }
    if (hNand->IsCqError)
    {
        Error = NvError_NandCommandQueueError;
        hNand->IsCqError = NV_FALSE;
        DumpRegData(hNand);
        NV_ASSERT("Command Queue Error in Nand HW");
    }
    return Error;
}

static NvError WaitForDmaDone(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;
#if !NAND_DDK_ENABLE_DMA_POLLING_MODE
    NandParams *p = &hNand->Params;
    NvU32 DmaMasterControl;
    NvU32 DmaGo;
#endif
    if (!hNand->IsCommandQueueOperation)
    {
        #if NAND_DDK_ENABLE_DMA_POLLING_MODE
        e = NandWaitDmaDone(hNand);
        #else
        e = NvOsSemaphoreWaitTimeout(hNand->DmaDoneSema,
                 p->WaitTimeoutInMilliSeconds);
        if (e == NvError_Timeout)
        {
            DmaMasterControl = Nand_REGR(hNand, DMA_MST_CTRL);
            DmaGo = NV_DRF_VAL(NAND, DMA_MST_CTRL, DMA_GO, DmaMasterControl);
            if (DmaGo == 0)
                e = NvSuccess;
        }
        #endif
        NAND_ASSERT(e);
        if (e != NvSuccess)
        {
            DumpRegData(hNand);
        }
    }
    return e;
}

static NvError WaitForCommandDone(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;
    #if !NAND_DDK_ENABLE_COMMAND_POLLING_MODE
    NandParams *p = &hNand->Params;
    NvU32 CommandReg;
    NvU32 CommandGo;
    #endif

    if (!hNand->IsCommandQueueOperation)
    {
        #if NAND_DDK_ENABLE_COMMAND_POLLING_MODE
        e = NandWaitCommandDone(hNand);
        #else
        e = NvOsSemaphoreWaitTimeout(hNand->CommandDoneSema,
                p->WaitTimeoutInMilliSeconds);
        if (e == NvError_Timeout)
        {
            CommandReg = Nand_REGR(hNand, COMMAND);
            CommandGo = NV_DRF_VAL(NAND, COMMAND, GO, CommandReg);
            if (CommandGo == 0)
                e = NvSuccess;
        }
        #endif
        NAND_ASSERT(e);
        if (e != NvSuccess)
        {
            DumpRegData(hNand);
        }
    }
    return e;
}

static void
SkipUnusedDevices(NvDdkNandHandle hNand,
    NvU32* pPageNumbers,
    NvU8* pStartDeviceNum,
    NvU32* pOffset)
{
    NvU8 StartDeviceNum = *pStartDeviceNum;
    NvU8 Count = 0;
    while (pPageNumbers[StartDeviceNum] == 0xFFFFFFFF)
    {
        StartDeviceNum++;
        if (StartDeviceNum >= NDFLASH_CS_MAX)
        {
            StartDeviceNum = 0;
            if (pOffset)
                *pOffset = *pOffset + 1;
        }
        Count++;
        // If follwing ASSERT hits, then none of the devices are filled with
        // valid page numbers.
        if (Count >= NDFLASH_CS_MAX)
            NV_ASSERT(NV_FALSE);
    }
    *pStartDeviceNum = StartDeviceNum;
}

// command queue read operation
static NvError CommandQueueRead(NvDdkNandHandle hNand, NvU32* pPageNumOffset)
{
    NandParams *p = &hNand->Params;
    NvError e = NvSuccess;
    NvU8 DeviceNumber = 0;
    NvU32 PageNumber = 0;

    SkipUnusedDevices(hNand, p->pStartPageNumbers,
    &(p->DeviceNumber),  pPageNumOffset);
    DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
    PageNumber = p->pStartPageNumbers[p->DeviceNumber];
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nRd DevNum = %d, PageNum = %d",
            DeviceNumber, (PageNumber + (*pPageNumOffset))));
    }
    #endif
    // Setup all registers required for main area read here.
    SetupAddressAndDeviceReg(hNand, DeviceNumber,
        (PageNumber + (*pPageNumOffset)));
    // Set Command queue buffer and start the data transfers
    StartCqOperation(hNand);

    e = WaitForCqDone(hNand);
    if (e != NvSuccess)
    {
        return e;
    }
    (*pPageNumOffset) +=
        (p->NumberOfPages / hNand->NumberOfChipsToBeInterleaved);
    p->DeviceNumber +=
        (p->NumberOfPages % hNand->NumberOfChipsToBeInterleaved);
    do
    {
        p->DeviceNumber++;
        if (p->DeviceNumber >= NDFLASH_CS_MAX)
        {
            p->DeviceNumber = 0;
        }
    }
    while(p->pStartPageNumbers[p->DeviceNumber] == 0xFFFFFFFF);
    p->NumberOfPagesCompleted += p->NumberOfPages;

    // Wait till DMA done. This is only for non-command queue mode.
    // For Command queue mode, this function just returns back.
    NV_CHECK_ERROR_CLEANUP(WaitForDmaDone(hNand));
    // Shift the data buffer pointer by p->NumberOfPages.
    if ((p->pDataBuffer != NULL) &&
        (p->pDataBuffer !=hNand->DataBuffer.pVirtualBuffer))
    {
        NvOsMemcpy(p->pDataBuffer, hNand->DataBuffer.pVirtualBuffer,
           (hNand->DevInfo.PageSize) * (p->NumberOfPages));
        p->pDataBuffer += (hNand->DevInfo.PageSize) *
                          (p->NumberOfPages);
    }

    return NvSuccess;
fail:
    return e;
}

// Normal non-command queue read operation
static NvError NonCQRead(NvDdkNandHandle hNand, NvU32* pPageNumOffset)
{
    NandParams *p = &hNand->Params;
    NvError e = NvSuccess;
    NvU32 i = 0;
    NvU32 j = 0;
    NvU8 DeviceNumber = 0;
    NvU8 NumberOfPages = 0;
    NvU32 PageNumber = 0;
    NvU8 *pTagBuf = NULL;
    NvU8* pBuf = hNand->DataBuffer.pVirtualBuffer;
    #if NAND_DDK_ENABLE_COMMAND_POLLING_MODE
        NvU32 IsrReg = 0;
        NvU32 IsEccError =0;
    #endif

    for (i = 0; i < p->NumberOfPages; i += NumberOfPages)
    {
        NumberOfPages = 0;
        for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
        {
            SkipUnusedDevices(hNand, p->pStartPageNumbers,
            &(p->DeviceNumber), pPageNumOffset);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            PageNumber = p->pStartPageNumbers[p->DeviceNumber];
            #if NAND_DISPLAY_ALL
            if (DebugPrintEnable)
            {
                PRINT_ALL(("\nRd DevNum = %d, PageNum = %d",
                    DeviceNumber, (PageNumber + (*pPageNumOffset))));
            }
            #endif
            // Setup all registers required for main area read here.
            SetupAddressAndDeviceReg(hNand, DeviceNumber,
                (PageNumber + (*pPageNumOffset)));
            // Start Nand operation.
            StartNandOperation(hNand);
            // Copy previous page data from local buffer to OS buffer
            if ((i > 0) && (p->pDataBuffer != NULL) &&
                (p->pDataBuffer !=hNand->DataBuffer.pVirtualBuffer))
            {
                NvOsMemcpy(p->pDataBuffer, pBuf, hNand->DevInfo.PageSize);
                p->pDataBuffer += (hNand->DevInfo.PageSize);
                pBuf += (hNand->DevInfo.PageSize);
            }
            // Wait till command done.
            e = WaitForCommandDone(hNand);
            if (e != NvSuccess)
            {
                return e;
            }
            #if NAND_DDK_ENABLE_COMMAND_POLLING_MODE
            IsrReg = Nand_REGR(hNand, ISR);
            IsEccError = NV_DRF_VAL(NAND, ISR, IS_ECC_ERR, IsrReg);
            if (IsEccError)
            {
                hNand->pEccErrorData[hNand->EccErrorCount] =
                                                       Nand_REGR(hNand, DEC_STATUS);
                hNand->EccErrorCount++;
                IsrReg = NV_DRF_NUM(NAND, ISR, IS_ECC_ERR, 1);
                Nand_REGW(hNand, ISR, IsrReg);
            }
            #endif
            p->NumberOfPagesCompleted++;
            p->DeviceNumber++;
            if (p->DeviceNumber >= NDFLASH_CS_MAX)
            {
                p->DeviceNumber = 0;
                (*pPageNumOffset)++;
            }
            NumberOfPages++;
            if ((i + NumberOfPages) == p->NumberOfPages)
                break;
        }
    }

    // Wait till DMA done. This is only for non-command queue mode.
    // For Command queue mode, this function just returns back.
    NV_CHECK_ERROR_CLEANUP(WaitForDmaDone(hNand));
    // Shift the data buffer pointer by p->NumberOfPages.
    if ((p->pDataBuffer != NULL) &&
        (p->pDataBuffer !=hNand->DataBuffer.pVirtualBuffer))
    {
        NvU32 PageCount;
        if (hNand->NumberOfChipsToBeInterleaved > p->NumberOfPages)
        {
            NvOsDebugPrintf("\nNandRead Error: Number of Pages=%d < "
                "interleave count=%d ", p->NumberOfPages,
                hNand->NumberOfChipsToBeInterleaved);
        }
        PageCount = (p->NumberOfPages <
            hNand->NumberOfChipsToBeInterleaved)? p->NumberOfPages :
            hNand->NumberOfChipsToBeInterleaved;
        NvOsMemcpy(p->pDataBuffer, pBuf,
            hNand->DevInfo.PageSize * PageCount);
        p->pDataBuffer += (hNand->DevInfo.PageSize * PageCount);
    }
    if (p->pTagBuffer)
    {
        pTagBuf = hNand->TagBuffer.pVirtualBuffer;
        for (i = 0; i < p->NumberOfPages; i++)
        {
            if (p->pDataBuffer != NULL)
                pTagBuf += (hNand->FlashParams.SkippedSpareBytes << 2);
            // user the NumSpareBytes if specified, else use the TagSize
            if (p->NumSpareAreaBytes)
            {
                NvOsMemcpy(p->pTagBuffer, pTagBuf, p->NumSpareAreaBytes);
                p->pTagBuffer += (p->NumSpareAreaBytes);
                pTagBuf += (p->NumSpareAreaBytes);
            }
            else
            {
                NvOsMemcpy(p->pTagBuffer, pTagBuf, hNand->DevInfo.TagSize);
                p->pTagBuffer += (hNand->DevInfo.TagSize);
                pTagBuf += (hNand->DevInfo.TagSize);
            }
        }
    }
    return NvSuccess;
fail:
    return e;
}

static NvError NandRead(NvDdkNandHandle hNand, NvBool IgnoreEccError)
{
    NandParams *p = &hNand->Params;
    NvU32 NumOfPagesRemaining = 0;
    NvU32 PageNumberOffset = 0;
    NvError e = NvSuccess;
    NvU32 EccErrorPageOffset = 0;
    NvU32 i = 0;
    #if NAND_TIME_STAMP
    NvU64 Time;
    #endif

    NumOfPagesRemaining = p->NumberOfPages;
    // Calculate number of CS interleaved in the current transaction
    GetNumOfCsInterleaved(hNand, p->pStartPageNumbers);

    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nRd DevNum = %d, PageNum = %d, No. pages = %d",
            p->DeviceNumber, p->pStartPageNumbers[p->DeviceNumber], 
            p->NumberOfPages));
    }
    #endif
    SetupRegisters(hNand, NandOperation_Read);
    #if NAND_TIME_STAMP
    Time = NvOsGetTimeUS();
    #endif
    while (NumOfPagesRemaining)
    {
        // Calculate number of pages to be read in one DMA set up.
        p->NumberOfPages =
            (NumOfPagesRemaining > hNand->MaxNumOfPagesPerDMARequest) ?
            hNand->MaxNumOfPagesPerDMARequest : NumOfPagesRemaining;
        NumOfPagesRemaining -= p->NumberOfPages;
        // In command queue mode If ECC error occurs, then command queue may
        // return LL_ERROR. Hence to avoid this, in cases when IgnoreEccError is
        // NV_TRUE, then use Non-command queue mode only for read operations.
        if (IgnoreEccError)
            hNand->IsCommandQueueOperation = NV_FALSE;
        else
        {
            #if IS_CQ_ENABLED
            SetCommandQueueOperationState(hNand, p->NumberOfPages, p);
            #else
            hNand->IsCommandQueueOperation = NV_FALSE;
            #endif
        }
        SetupInterrupt(hNand, NandOperation_Read);
        hNand->EccErrorCount = 0;
        // Setup DMA upto maximum data size that can be transferrable.
        SetupDMA(hNand);
        if (hNand->IsCommandQueueOperation)
            e = CommandQueueRead(hNand, &PageNumberOffset);
        else
            e = NonCQRead(hNand, &PageNumberOffset);
        if (!IgnoreEccError)
            NV_CHECK_ERROR_CLEANUP(
                NandCheckForEccError(hNand, &EccErrorPageOffset));
    }
    #if NAND_TIME_STAMP
    Time = NvOsGetTimeUS() - Time;
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\r\ntms Read time = %dus, pc = %d", Time, p->NumberOfPages));
    }
    #endif
fail:
    if (e != NvSuccess)
    {
        for (i = 0; i < 8; i++)
        {
            if (p->pStartPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n Chip: %d, Page = %d\n", i, p->pStartPageNumbers[i]);
        }
    }
    // Don't assert for ECC failure
    if (e != NvError_NandReadEccFailed)
    {
        NAND_ASSERT(e);
    }
    else
        DumpRegData(hNand);
    if (e == NvError_Timeout)
    {
        DumpRegData(hNand);
        NvRmModuleReset(hNand->RmDevHandle, NvRmModuleID_Nand);
        SetTimingRegVal(hNand, NV_FALSE);
        // Restore the NAND lock cfg that was stored during previous Suspend, 
        // as locks should have got released due to reset operation.
        NandRestoreLocks(hNand);
    }
    else if (e == NvError_NandReadEccFailed)
    {
        p->NumberOfPagesCompleted =
        ((p->NumberOfPagesCompleted / hNand->MaxNumOfPagesPerDMARequest) *
        hNand->MaxNumOfPagesPerDMARequest) + EccErrorPageOffset;
    }
    if (hNand->IsCommandQueueOperation)
    {
        if (e == NvError_NandCommandQueueError)
            p->NumberOfPagesCompleted = hNand->NumOfPagesTransferred;
        hNand->IsCommandQueueOperation = NV_FALSE;
    }
    hNand->OperationStatus = e;
    return e;
}

static NvError NandCheckForEccError(NvDdkNandHandle hNand, NvU32* ErrorPage)
{
    NandParams *p = &hNand->Params;
    NvU32 DecStatusReg;
    NvU32 BufferStart;
    NvU32 i;
    NvU32 j;
    NvBool IsDefaultData = NV_FALSE;
    for (i = 0; i < hNand->EccErrorCount; i++)
    {
        IsDefaultData = NV_FALSE;
        // Check Nand decode status register to check decode failures.
        DecStatusReg = hNand->pEccErrorData[i];
        *ErrorPage = NV_DRF_VAL(NAND, DEC_STATUS, ERR_PAGE_NUMBER, DecStatusReg);
        if (NV_DRF_VAL(NAND, DEC_STATUS, A_ECC_FAIL, DecStatusReg))
        {
            // Don't raise Ecc error if all data is 0xFF.
            if (p->pDataBuffer)
            {
                BufferStart = ((*ErrorPage) * hNand->DevInfo.PageSize);
                for (j = 0; j < hNand->DevInfo.PageSize; j++)
                {
                    if (hNand->DataBuffer.pVirtualBuffer[j + BufferStart] != 0xFF)
                    {
                        DumpRegData(hNand);
                        PRINT_ERROR(("\r\n Ecc.Err pgoffset: %d, status: 0x%x", *ErrorPage, DecStatusReg));
                        return NvError_NandReadEccFailed;
                    }
                }
                // We reached here because Flash has default data i.e. 0xFF
                IsDefaultData = NV_TRUE;
            }
        }
        if (NV_DRF_VAL(NAND, DEC_STATUS, B_ECC_FAIL, DecStatusReg))
        {
            if (p->pTagBuffer)
            {
                BufferStart = ((*ErrorPage) * hNand->DevInfo.TagSize);
                for (j = 0; j < hNand->DevInfo.TagSize; j++)
                {
                    if (hNand->TagBuffer.pVirtualBuffer[j + BufferStart] != 0xFF)
                    {
                        DumpRegData(hNand);
                        PRINT_ERROR(("\r\n Ecc.Err in Tag pgoffset: %d, status: 0x%x", *ErrorPage, DecStatusReg));
                        return NvError_NandReadEccFailed;
                    }
                }
                // We reached here because Flash has default data i.e. 0xFF
                IsDefaultData = NV_TRUE;
           }
        }
        // If we have ECC error with no decode failure observed and also Flash
        // contains Non-0xFF data means, the error threshold has reached.
        if (!IsDefaultData)
            return NvError_NandErrorThresholdReached;
    }
    return NvSuccess;
}

static NvError NonCQWrite(NvDdkNandHandle hNand, NvU32 MaxPageNumberOffset, NvU32* pPageNumberOffset)
{
    NvU32 i = 0;
    NvU32 j = 0;
    NandParams *p = &hNand->Params;
    NvError e = NvSuccess;
    #if NAND_TIME_STAMP
    NvU64 Time;
    #endif
    NvU32 NumberOfPages = 0;
    NvU8 StartDeviceNumber = 0;
    NvU8 DeviceNumber = 0;
    NvU32 PageNumber = 0;

    for (i = 0; i < p->NumberOfPages; i += NumberOfPages)
    {
        NumberOfPages = 0;
        StartDeviceNumber = p->DeviceNumber;
        for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
        {
            SkipUnusedDevices(hNand, p->pStartPageNumbers,
            &(p->DeviceNumber), pPageNumberOffset);

            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            if (i)
            {
                // Check status of previous operation here
                e = GetOperationStatus(hNand, DeviceNumber);
                if (e != NvError_Success)
                    return e;
            }

            SetupRegisters(hNand, NandOperation_Write);
            SetupInterrupt(hNand, NandOperation_Write);
            if (((*pPageNumberOffset) < (MaxPageNumberOffset - 1)) &&
                    hNand->IsCacheWriteSupproted)
                Nand_REGW(hNand, CMD_REG2, NvOdmNandCommandList_Cache_Program_Start);
            else
                Nand_REGW(hNand, CMD_REG2, NvOdmNandCommandList_Page_Program_Start);

            PageNumber = p->pStartPageNumbers[p->DeviceNumber];
            #if NAND_DISPLAY_ALL
            if (DebugPrintEnable)
            {
                PRINT_ALL(("\nWr DevNum = %d, PageNum = %d",
                    DeviceNumber, (PageNumber + (*pPageNumberOffset))));
            }
            #endif
            // Setup all registers required for main area read here.
            SetupAddressAndDeviceReg(hNand, DeviceNumber,
                (PageNumber + (*pPageNumberOffset)));
            // Start Nand operation.
            StartNandOperation(hNand);

            // Wait till command done.
            e = WaitForCommandDone(hNand);
            if (e != NvError_Success)
                return e;
            p->DeviceNumber++;
            if (p->DeviceNumber >= NDFLASH_CS_MAX)
            {
                p->DeviceNumber = 0;
                (*pPageNumberOffset)++;
            }
            NumberOfPages++;
            p->NumberOfPagesCompleted++;
            if ((i + NumberOfPages) == p->NumberOfPages)
                break; 
        }
    }
    p->DeviceNumber = StartDeviceNumber;
    for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
    {
        SkipUnusedDevices(hNand, p->pStartPageNumbers,
        &(p->DeviceNumber), NULL);
        DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
        // Check status of previous operation here
        e = GetOperationStatus(hNand, DeviceNumber);
        if (e != NvError_Success)
            return NvError_NandProgramFailed;
        p->DeviceNumber++;
        if (p->DeviceNumber == hNand->NumberOfChipsToBeInterleaved)
        {
            p->DeviceNumber = 0;
        }
    }
    (*pPageNumberOffset)++;
    return e;
}

static NvError CommandQueueWrite(NvDdkNandHandle hNand, NvU32 MaxPageNumberOffset, NvU32* pPageNumberOffset)
{
    NandParams *p = &hNand->Params;
    NvError e = NvSuccess;
    #if NAND_TIME_STAMP
    NvU64 Time;
    #endif
    NvU8 DeviceNumber = 0;
    NvU32 PageNumber = 0;

    SkipUnusedDevices(hNand, p->pStartPageNumbers,
    &(p->DeviceNumber), pPageNumberOffset);

    DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
    SetupRegisters(hNand, NandOperation_Write);
    SetupInterrupt(hNand, NandOperation_Write);
    if (((*pPageNumberOffset) < (MaxPageNumberOffset - 1)) &&
            hNand->IsCacheWriteSupproted)
        Nand_REGW(hNand, CMD_REG2, NvOdmNandCommandList_Cache_Program_Start);
    else
        Nand_REGW(hNand, CMD_REG2, NvOdmNandCommandList_Page_Program_Start);

    PageNumber = p->pStartPageNumbers[p->DeviceNumber];
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nWr DevNum = %d, PageNum = %d",
            DeviceNumber, (PageNumber + (*pPageNumberOffset))));
    }
    #endif
    // Setup all registers required for main area read here.
    SetupAddressAndDeviceReg(hNand, DeviceNumber,
        (PageNumber + (*pPageNumberOffset)));
    // Set Command queue buffer and start the data transfers
    StartCqOperation(hNand);

    e = WaitForCqDone(hNand);
    if (e != NvSuccess)
        return e;
    (*pPageNumberOffset) +=
        (p->NumberOfPages / hNand->NumberOfChipsToBeInterleaved);
    p->DeviceNumber += (p->NumberOfPages % hNand->NumberOfChipsToBeInterleaved);
    do
    {
        p->DeviceNumber++;
        if (p->DeviceNumber >= NDFLASH_CS_MAX)
        {
            p->DeviceNumber = 0;
        }
    }
    while(p->pStartPageNumbers[p->DeviceNumber] == 0xFFFFFFFF);
    p->NumberOfPagesCompleted += p->NumberOfPages;

    return e;
}

static NvError NandWrite(NvDdkNandHandle hNand)
{
    NvU32 NumOfPagesRemaining = 0;
    NvU32 PageNumberOffset = 0;
    NvError e = NvSuccess;
    NvU32 i = 0;
    NandParams *p = &hNand->Params;
    NvU32 MaxPageNumberOffset = 0;
    #if NAND_TIME_STAMP
    NvU64 Time;
    #endif

    NumOfPagesRemaining = p->NumberOfPages;
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nWr DevNum = %d, PageNum = %d, No. pages = %d",
            p->DeviceNumber, p->pStartPageNumbers[p->DeviceNumber],
            p->NumberOfPages));
    }
    #endif
    // Calculate number of CS interleaved in the current transaction
    GetNumOfCsInterleaved(hNand, p->pStartPageNumbers);

    #if NAND_TIME_STAMP
    Time = NvOsGetTimeUS();
    #endif
    while (NumOfPagesRemaining)
    {
        // Calculate number of pages to be written in one DMA set up.
        p->NumberOfPages =
            (NumOfPagesRemaining > hNand->MaxNumOfPagesPerDMARequest) ?
            hNand->MaxNumOfPagesPerDMARequest : NumOfPagesRemaining;
        NumOfPagesRemaining -= p->NumberOfPages;
        // Shift the data buffer pointer by p->NumberOfPages .
        if ((p->pDataBuffer != NULL) && (p->pDataBuffer !=hNand->DataBuffer.pVirtualBuffer))
        {
            NvOsMemcpy(hNand->DataBuffer.pVirtualBuffer,
                        p->pDataBuffer,
                       (hNand->DevInfo.PageSize) * (p->NumberOfPages));
        }
        if (p->pTagBuffer)
        {
            NvOsMemcpy( hNand->TagBuffer.pVirtualBuffer,
                        p->pTagBuffer,
                        p->NumSpareAreaBytes ? p->NumSpareAreaBytes : 
                        ((hNand->DevInfo.TagSize) * (p->NumberOfPages)) );
        }
    #if IS_CQ_ENABLED
        SetCommandQueueOperationState(hNand, NumOfPagesRemaining, p);
    #else
        hNand->IsCommandQueueOperation = NV_FALSE;
    #endif
        // Setup DMA upto maximum data size that can be transferrable.
        SetupDMA(hNand);
        MaxPageNumberOffset += (p->NumberOfPages / hNand->NumberOfChipsToBeInterleaved);

        if (hNand->IsCommandQueueOperation)
            e = CommandQueueWrite(hNand, MaxPageNumberOffset, &PageNumberOffset);
        else
            e = NonCQWrite(hNand, MaxPageNumberOffset, &PageNumberOffset);
        if (p->pDataBuffer)
        {
            // Shift the data buffer pointer by p->NumberOfPages .
            p->pDataBuffer += (hNand->DevInfo.PageSize) *
                              (p->NumberOfPages);
        }
        if (p->pTagBuffer)
        {
            p->pTagBuffer += (hNand->DevInfo.TagSize) *
                             (p->NumberOfPages);
        }

        // Wait till DMA done. This is only for non-command queue mode.
        // For Command queue mode, this function just returns back.
        NV_CHECK_ERROR_CLEANUP(WaitForDmaDone(hNand));
    }
    #if NAND_TIME_STAMP
    Time = NvOsGetTimeUS() - Time;
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\r\ntms Write time = %dus, pc = %d", Time, p->NumberOfPages));
    }
    #endif
fail:
    NAND_ASSERT(e);
    if (e != NvSuccess)
    {
            for (i = 0;i < 8; i++)
            {
                if (p->pStartPageNumbers[i] != 0xFFFFFFFF)
                    NvOsDebugPrintf("\n Chip: %d, Page = %d\n", i, p->pStartPageNumbers[i]);
            }
    }
    if (e == NvError_Timeout)
    {
        DumpRegData(hNand);
        NvRmModuleReset(hNand->RmDevHandle, NvRmModuleID_Nand);
        SetTimingRegVal(hNand, NV_FALSE);
        // Restore the NAND lock cfg that was stored during previous Suspend, 
        // as locks should have got released due to reset operation.
        NandRestoreLocks(hNand);
    }
    if (hNand->IsCommandQueueOperation)
    {
        if (e == NvError_NandCommandQueueError)
            p->NumberOfPagesCompleted = hNand->NumOfPagesTransferred;
        hNand->IsCommandQueueOperation = NV_FALSE;
    }
    hNand->OperationStatus = e;
    return e;
}

///////////////// TO SUPPORT ONFI NANDS ///////////////////////

#define ONFI_INITIAL_PAGE_SIZE 2048  // FixMe: Does it need to be set to the Max Page size?
#define ONFI_SPEC2_INITIAL_TR_TIME 200 // 200 microseconds as per section 4.2.1 of ONFI spec2
#define READ_ID_ADDR_FOR_ONFI_NAND 0x20
#define ONFI_ID_SIZE_IN_BYTES 4

static NvError ReadONFIParamPage(NvDdkNandHandle hNand, NvU8 DeviceNumber, NvRmPhysAddr pDataBuffer)
{
    NvError e = NvSuccess;
    NvU32 DmaMasterControl = 0;
    SetupRegisters(hNand, NandOperation_ReadParamPage);
    SetupInterrupt(hNand, NandOperation_Read);

    hNand->Params.ColumnNumber = 0;
    SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
    // Start Nand operation.
    StartNandOperation(hNand);
    NV_CHECK_ERROR_CLEANUP(WaitForCommandDone(hNand));

    // Wait for initial tR time here as per ONFI spec.
    NandWaitUS(ONFI_SPEC2_INITIAL_TR_TIME);

    //issue data cycles 
    SetupRegisters(hNand, NandOperation_DataCyclesAlone);
    SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
    Nand_REGW(hNand, DATA_BLOCK_PTR, (NvU32)pDataBuffer);
    Nand_REGW(hNand, DMA_CFG_A, (ONFI_INITIAL_PAGE_SIZE - 1));
    Nand_REGW(hNand, CONFIG, 0x30000);

    // Set the DMA burst size to 8 words.
    // Enable DMA performance.
    // Enable DMA for main area.
    // Finally, Enable the DMA
    DmaMasterControl = NV_DRF_DEF(NAND, DMA_MST_CTRL, BURST_SIZE, BURST_8WORDS)|
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_PERF_EN, ENABLE) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_EN_A, DEFAULT_MASK) |
                       NV_DRF_NUM(NAND, DMA_MST_CTRL, IS_DMA_DONE, 1) |
                       NV_DRF_NUM(NAND, DMA_MST_CTRL, IE_DMA_DONE, 1) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_GO, ENABLE);

    Nand_REGW(hNand, DMA_MST_CTRL, DmaMasterControl);

    // Start Nand operation.
    StartNandOperation(hNand);
    NV_CHECK_ERROR_CLEANUP(WaitForCommandDone(hNand));
    NV_CHECK_ERROR_CLEANUP(WaitForDmaDone(hNand));
fail:
    return e;
}

static NvError InitONFINands(NvDdkNandHandle hNand, const NvU8 DeviceNumber)
{
    NvU8 ReadID[ONFI_ID_SIZE_IN_BYTES] ;
    NvError e = NvSuccess;
    SdramBufferParams BufParams;
    NvU32 CompareResult = 0;
    NvU8 OnfiSignature[ONFI_ID_SIZE_IN_BYTES] = {0x4F, 0x4E, 0x46, 0x49}; // ASCII values of O, N, F, I

    BufParams.BufferSize = ONFI_INITIAL_PAGE_SIZE;
    NV_CHECK_ERROR_CLEANUP(MemAllocBuffer(hNand, &BufParams, NAND_BUFFER_ALIGNMENT));
    hNand->Params.WaitTimeoutInMilliSeconds = NAND_COMMAND_TIMEOUT_IN_MS;
    ResetNandFlash(hNand, DeviceNumber);
    // Check for the ONFI signature
    NV_CHECK_ERROR_CLEANUP(NandReadID(hNand, DeviceNumber, (NvU32 *)ReadID, NV_TRUE));
    CompareResult = NandUtilMemcmp(ReadID, OnfiSignature, ONFI_ID_SIZE_IN_BYTES);
    if ((DeviceNumber == 0) && CompareResult)
        hNand->IsONFINandOnCs0 = NV_FALSE;
    // Read Params Page
    NV_CHECK_ERROR_CLEANUP(ReadONFIParamPage(hNand, DeviceNumber, BufParams.PhysBuffer));
    CompareResult = NandUtilMemcmp(BufParams.pVirtualBuffer, OnfiSignature, ONFI_ID_SIZE_IN_BYTES);
    if ((DeviceNumber == 0) && CompareResult)
        hNand->IsONFINandOnCs0 = NV_FALSE;
fail:
    DestroyMemHandle(&BufParams);
    return e;
}

///////////////// TO SUPPORT ONFI NANDS ///////////////////////

static NvError NandReadID(
    NvDdkNandHandle hNand,
    NvU8 DeviceNumber,
    NvU32* ReadID,
    NvBool IsOnfiNand)
{
    NvError Error = NvSuccess;

    SetupRegisters(hNand, NandOperation_ReadId);

    if (IsOnfiNand)
        hNand->Params.ColumnNumber = READ_ID_ADDR_FOR_ONFI_NAND;
    else
        hNand->Params.ColumnNumber = 0;
    SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
    CleanInterruptRegisters(hNand);
    // Start Nand operation.
    StartNandOperation(hNand);
    Error = NandWaitCommandDone(hNand);
    *ReadID = Nand_REGR(hNand, RESP);
    return Error;
}

static void NandWaitUS(NvU32 usec)
{
    NvU64 t0;
    NvU64 t1;

    t0 = NvOsGetTimeUS();
    t1 = t0;
    // Use the difference for the comparison to be wraparound safe
    while (DIFF(t1, t0) < usec)
    {
        t1 = NvOsGetTimeUS();
    }
}

#if NAND_DDK_ENABLE_DMA_POLLING_MODE
static NvError NandWaitDmaDone(NvDdkNandHandle hNand)
{
    NvU32 DmaMasterControl;
    NvU32 DmaGo;
    NvU32 StatusReg;
    NvU32 IsIdle;
    NvU32 IsrReg;
    NvU32 IsDmaDone;
    NvU32 TimeOutCounter = 10000000;
    // Wait for DMA done with timeout check.
    while (TimeOutCounter)
    {
        // Check whether the TX is completed.
        DmaMasterControl = Nand_REGR(hNand, DMA_MST_CTRL);
        DmaGo = NV_DRF_VAL(NAND, DMA_MST_CTRL, DMA_GO, DmaMasterControl);
        // Check the status register.
        StatusReg = Nand_REGR(hNand, STATUS);
        IsIdle = NV_DRF_VAL(NAND, STATUS, ISEMPTY, StatusReg);
        IsDmaDone = NV_DRF_VAL(NAND, DMA_MST_CTRL, IS_DMA_DONE, DmaMasterControl);
        if ((DmaGo == 0) && IsIdle && IsDmaDone)
        {
            IsrReg = Nand_REGR(hNand, ISR);
            // Clear the status.
            Nand_REGW(hNand, ISR, IsrReg);
            Nand_REGW(hNand, DMA_MST_CTRL, DmaMasterControl);
            break;
        }
        NvOsWaitUS(1);
        TimeOutCounter--;
    }
    if (!TimeOutCounter)
    {
        DmaGo = NV_DRF_VAL(NAND, DMA_MST_CTRL, DMA_GO, DmaMasterControl);
        if (DmaGo != 0) 
        {
            NAND_ASSERT(NvError_Timeout);
            return NvError_Timeout;
        }
    }
    return NvSuccess;
}

static NvError NandWaitCqDone(NvDdkNandHandle hNand)
{
    NvU32 CommandReg;
    NvU32 CommandGo;
    NvU32 TimeOutCounter = 1000 * NAND_COMMAND_TIMEOUT_IN_MS;
    NvU32 IsrReg;
    NvU32 IsCommandDone;
    // Wait for Command to be sent out with time out.
    while (TimeOutCounter)
    {
        CommandReg = Nand_REGR(hNand, COMMAND);
        CommandGo = NV_DRF_VAL(NAND, LL_CONFIG, LL_START, CommandReg);
        // CommandGo = 1 indicates that command send cycle is not yet complete.
        // CommandGo = 0 indicates that command is sent.
        IsrReg = Nand_REGR(hNand, ISR);
        IsCommandDone = NV_DRF_VAL(NAND, ISR, IS_LL_DONE, IsrReg);
        if ((CommandGo == 0) && IsCommandDone)
        {
            IsCommandDone = NV_DRF_NUM(NAND, ISR, IS_LL_DONE, 1);
            Nand_REGW(hNand, ISR, IsCommandDone);
            break;
        }
        NvOsWaitUS(1);
        TimeOutCounter--;
    }
    if (!TimeOutCounter)
    {
        NAND_ASSERT(NvError_Timeout);
        return NvError_Timeout;
    }
    return NvSuccess;
}

#endif

static NvError NandWaitCommandDone(NvDdkNandHandle hNand)
{
    NvU32 CommandReg;
    NvU32 CommandGo;
    NvU32 TimeOutCounter = 1000 * NAND_COMMAND_TIMEOUT_IN_MS;
    NvU32 IsrReg;
    NvU32 IsCommandDone;
    // Wait for Command to be sent out with time out.
    while (TimeOutCounter)
    {
        CommandReg = Nand_REGR(hNand, COMMAND);
        CommandGo = NV_DRF_VAL(NAND, COMMAND, GO, CommandReg);
        // CommandGo = 1 indicates that command send cycle is not yet complete.
        // CommandGo = 0 indicates that command is sent.
        IsrReg = Nand_REGR(hNand, ISR);
        IsCommandDone = NV_DRF_VAL(NAND, ISR, IS_CMD_DONE, IsrReg);
        if ((CommandGo == 0) && IsCommandDone)
        {
            IsCommandDone = NV_DRF_NUM(NAND, ISR, IS_CMD_DONE, 1);
            Nand_REGW(hNand, ISR, IsCommandDone);
            break;
        }
        NvOsWaitUS(1);
        TimeOutCounter--;
    }
    if (!TimeOutCounter)
    {
        CommandGo = NV_DRF_VAL(NAND, COMMAND, GO, CommandReg);
        if (CommandGo != 0) 
        {
            NAND_ASSERT(NvError_Timeout);
            return NvError_Timeout;
        }
    }
    return NvSuccess;
}

static NvError
RegisterNandInterrupt(
    NvRmDeviceHandle hDevice,
    NvDdkNandHandle hNand)
{
    NvU32 IrqList;
    NvOsInterruptHandler IntHandlers;
    if (hNand->InterruptHandle)
    {
        return NvSuccess;
    }
    IrqList = NvRmGetIrqForLogicalInterrupt(hDevice, NvRmModuleID_Nand, 0);
    IntHandlers = NandIsr;
    return NvRmInterruptRegister(hDevice, 1, &IrqList, &IntHandlers,
        hNand, &hNand->InterruptHandle, NV_TRUE);
}

static void ClearNandFifos(NvDdkNandHandle hNand)
{
    NvU32 FifoControlRegister = 0;
    FifoControlRegister = NV_DRF_NUM(NAND, FIFO_CTRL, LL_BUF_CLR, 1)|
        NV_DRF_NUM(NAND, FIFO_CTRL, FIFO_A_CLR, 1)|
        NV_DRF_NUM(NAND, FIFO_CTRL, FIFO_B_CLR, 1)|
        NV_DRF_NUM(NAND, FIFO_CTRL, FIFO_C_CLR, 1);
    Nand_REGW(hNand, FIFO_CTRL, FifoControlRegister);
}

static void NandIsr(void* args)
{
    NvU32 InterruptStatusReg = 0;
    NvU32 InterruptEnableReg = 0;
    NvU32 Interrupts2Clear = 0;
    NvU32 EccErr2Clear = 0;
    NvU32 DmaStatusReg = 0;
    NvU32 DecodeStatusReg = 0;
    NvU32 SignalCommandDoneSema = 0;
    NvU32 SignalDmaDoneSema = 0;
    NvDdkNandHandle hNand = args;

    InterruptStatusReg = Nand_REGR(hNand, ISR);
    InterruptEnableReg = Nand_REGR(hNand, IER);
    DmaStatusReg = Nand_REGR(hNand, DMA_MST_CTRL);

    // If the interrupt is enabled, then only clear that particular interrupt.
    if (NV_DRF_VAL(NAND, ISR, IS_LL_ERR, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_LL_ERR, InterruptEnableReg))
    {
        // Clear LL_ERR field of ISR.
        Interrupts2Clear |= NV_DRF_NUM(NAND, ISR, IS_LL_ERR, 1);
        ClearNandFifos(hNand);
        DecodeStatusReg = Nand_REGR(hNand, DEC_STATUS);
        hNand->NumOfPagesTransferred = NV_DRF_VAL(NAND, DEC_STATUS,
                                            ERR_PAGE_NUMBER, DecodeStatusReg);
        hNand->IsCqError = NV_TRUE;
        PRINT_INTS(("\r\n NINT- IS_LL_ERR"));
        SignalCommandDoneSema++;
    }
    if (NV_DRF_VAL(NAND, ISR, IS_LL_DONE, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_LL_DONE, InterruptEnableReg))
    {
        // Clear LL_DONE field of ISR.
        Interrupts2Clear |= NV_DRF_NUM(NAND, ISR, IS_LL_DONE, 1);
        // Signal the Command done sema.
        PRINT_INTS(("\r\n NINT- IS_LL_DONE"));
        if (!hNand->IsCqError)
            SignalCommandDoneSema++;
    }
    if (NV_DRF_VAL(NAND, ISR, IS_UND, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_UND, InterruptEnableReg))
    {
        NV_ASSERT(NV_FALSE);
        // Clear UND field of ISR.
        Interrupts2Clear |= NV_DRF_NUM(NAND, ISR, IS_UND, 1);
        PRINT_INTS(("\r\n NINT- IS_UND"));
    }
    if (NV_DRF_VAL(NAND, ISR, IS_OVR, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_OVR, InterruptEnableReg))
    {
        NV_ASSERT(NV_FALSE);
        // Clear OVR field of ISR.
        Interrupts2Clear |= NV_DRF_NUM(NAND, ISR, IS_OVR, 1);
        PRINT_INTS(("\r\n NINT- IS_OVR"));
    }
    if (NV_DRF_VAL(NAND, ISR, IS_ECC_ERR, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_ECC_ERR, InterruptEnableReg))
    {
        // Clear ECC_ERR field of ISR.
        hNand->pEccErrorData[hNand->EccErrorCount] = Nand_REGR(hNand, DEC_STATUS);
        hNand->EccErrorCount++;
        NV_ASSERT(hNand->EccErrorCount <= hNand->MaxNumOfPagesPerDMARequest);
        NV_ASSERT(hNand->Params.OperationName == NandOperation_Read);
        EccErr2Clear |= NV_DRF_NUM(NAND, ISR, IS_ECC_ERR, 1);
        Nand_REGW(hNand, ISR, EccErr2Clear);
        PRINT_INTS(("\r\n NINT- IS_ECC_ERR"));
    }
    if (NV_DRF_VAL(NAND, ISR, IS_CMD_DONE, InterruptStatusReg) &&
        NV_DRF_VAL(NAND, IER, IE_CMD_DONE, InterruptEnableReg))
    {
        // Clear CMD_DONE field of ISR.
        Interrupts2Clear |= NV_DRF_NUM(NAND, ISR, IS_CMD_DONE, 1);
        // Signal the Command done sema.
        PRINT_INTS(("\r\n NINT- IS_CMD_DONE"));
        SignalCommandDoneSema++;
    }
    if (NV_DRF_VAL(NAND, DMA_MST_CTRL, IS_DMA_DONE, DmaStatusReg) &&
        NV_DRF_VAL(NAND, DMA_MST_CTRL, IE_DMA_DONE, DmaStatusReg))
    {
        // Clear DMA_DONE field of DMA_MST_CNTL reg.
        Nand_REGW(hNand, DMA_MST_CTRL, DmaStatusReg);
        // Signal the Dma done sema.
        PRINT_INTS(("\r\n NINT- IS_DMA_DONE"));
        SignalDmaDoneSema++;
    }
    // Write registers at the end.
    if (Interrupts2Clear & InterruptStatusReg)
    {
        Nand_REGW(hNand, ISR, Interrupts2Clear);
        InterruptStatusReg = Nand_REGR(hNand, ISR);
        PRINT_INTS(("\r\n NINT- Isr 0x%x 0x%x ", Interrupts2Clear,
            InterruptStatusReg));
    }

    InterruptStatusReg = Nand_REGR(hNand, LOCK_STATUS);
    if (NV_DRF_VAL(NAND, LOCK_STATUS, IS_LOCK_ERR, InterruptStatusReg))
    {
        PRINT_INTS(("\r\n NINT- IS_LOCK_ERR"));
        NandPrivLockInterruptService(hNand, InterruptStatusReg);
    }

    NV_ASSERT(SignalCommandDoneSema <= 1);
    NV_ASSERT(SignalDmaDoneSema <= 1);
    while (SignalCommandDoneSema--)
        NvOsSemaphoreSignal(hNand->CommandDoneSema);
    while (SignalDmaDoneSema--)
        NvOsSemaphoreSignal(hNand->DmaDoneSema);
    NvRmInterruptDone(hNand->InterruptHandle);
}

static NvError NandDisableWriteProtect(NvDdkNandHandle hNand)
{
    NvError ErrStatus = NvError_Success;
    NvRmGpioPinState PinState;
    const NvOdmGpioPinInfo *pPinInfoTable = NULL;
    NvU32 PinCount = 1;

     ErrStatus = NvRmGpioOpen(hNand->RmDevHandle, &hNand->hGpio);
    if (ErrStatus != NvSuccess)
        return ErrStatus;

    pPinInfoTable = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_NandFlash, 0, &PinCount);
    if (!pPinInfoTable)    // write protect gpio is optional.
        return NvSuccess;

    ErrStatus = NvRmGpioAcquirePinHandle(hNand->hGpio, pPinInfoTable->Port,
                    pPinInfoTable->Pin, &hNand->hWriteProtectPin);
    if (ErrStatus != NvSuccess)
        return ErrStatus;

    NvRmGpioConfigPins(hNand->hGpio, &hNand->hWriteProtectPin, 1, NvRmGpioPinMode_Output);
    PinState = NvRmGpioPinState_High;
    NvRmGpioWritePins(hNand->hGpio, &hNand->hWriteProtectPin, &PinState, 1);

    return ErrStatus;
}

static void NandLoadLockCfg(NvDdkNandHandle hNand)
{
    NvU32 i;
    NvU32 LockApertureMask;
    NvU32 LockCtrlReg;
    NvU32 LockRegOffset;

    LockCtrlReg = Nand_REGR(hNand, LOCK_CONTROL);
    LockApertureMask = 1;
    hNand->NumberOfAperturesUsed = 0;
    for (i = 0; i < NDFLASH_CS_MAX; i++)
    {
        if(LockCtrlReg & LockApertureMask)
        {
            // Lock aperture enabled...read the aperture cfg
            LockRegOffset = (NAND_LOCK_APER_CHIPID1_0 - NAND_LOCK_APER_CHIPID0_0) * i;
            hNand->LockAperStart[i] = Nand_REGR_OFFSET(hNand, LOCK_APER_START0, LockRegOffset);
            hNand->LockAperEnd[i] = Nand_REGR_OFFSET(hNand, LOCK_APER_END0, LockRegOffset);
            hNand->LockAperChipId[i] = Nand_REGR_OFFSET(hNand, LOCK_APER_CHIPID0, LockRegOffset);
            hNand->NumberOfAperturesUsed++;
        }
        LockApertureMask <<= 1;
    }

    return;
}

static void NandRestoreLocks(NvDdkNandHandle hNand) 
{
    NvU32 i;
    NvU32 LockCtrlReg;
    NvU32 LockRegOffset;
    LockCtrlReg = 0;

    // Restore the lock aperture definitions
    if(hNand->NumberOfAperturesUsed)
    {
        for (i = 0; i < hNand->NumberOfAperturesUsed; i++)
        {
            LockRegOffset = (NAND_LOCK_APER_CHIPID1_0 - NAND_LOCK_APER_CHIPID0_0) * i;
            Nand_REGW_OFFSET(hNand, LOCK_APER_START0, LockRegOffset, hNand->LockAperStart[i]);
            Nand_REGW_OFFSET(hNand, LOCK_APER_END0, LockRegOffset, hNand->LockAperEnd[i]);
            Nand_REGW_OFFSET(hNand, LOCK_APER_CHIPID0, LockRegOffset, hNand->LockAperChipId[i]);
            LockCtrlReg |= (1 << i);
        }
        // re-enable the apertures
        Nand_REGW(hNand, LOCK_CONTROL, LockCtrlReg);
    }
    return;
}

static void
NandPrivLockInterruptService(
    NvDdkNandHandle hNand,
    NvU32 InterruptStatusRegister)
{
    NvU32 LockStatusRegister = 0;

    InterruptStatusRegister |= NV_DRF_NUM(NAND, LOCK_STATUS, IS_LOCK_ERR, 1);
    Nand_REGW(hNand, LOCK_STATUS, InterruptStatusRegister);
    LockStatusRegister = Nand_REGR(hNand, LOCK_STATUS);
    Nand_REGW(hNand, LOCK_STATUS, LockStatusRegister);
    ClearNandFifos(hNand);
}

static void SetupCqPkt(NvDdkNandHandle hNand)
{
    NandParams* p = &hNand->Params;
    NvU32 RegVal;
    NvU32 Offset = 0;
    NvU32 PageNumber = 0;
    NvU32 NumOfPages = p->NumberOfPages;
    NvU32 CqLength = 0;
    NandCqPacket1 *pCqPkt1;
    NandCqPacket2 *pCqPkt2;
    NandCqPacket3 *pCqPkt3;
    NvU32 Command2 = 0;
    NvU32 PageNumberOffset = p->NumberOfPagesCompleted / hNand->NumberOfChipsToBeInterleaved;
    NvU8 DeviceNumber = 0;
    NvU32 MaxPageNumberOffset = 0;
    NvU8 i = 0;

    while (NumOfPages)
    {
        MaxPageNumberOffset = p->NumberOfPages / hNand->NumberOfChipsToBeInterleaved;
        if (p->OperationName == NandOperation_Write)
        {
            if ((PageNumberOffset < (MaxPageNumberOffset - 1)) &&
                hNand->IsCacheWriteSupproted)
                Command2 = NvOdmNandCommandList_Cache_Program_Start;
            else
                Command2 = NvOdmNandCommandList_Page_Program_Start;
        }
        else
        {
            Command2 = Nand_REGR(hNand, CMD_REG2);
        }

        // Fill dma control reg + address reg's + command reg's.
        if (Offset == 0)
        {
            pCqPkt1 = (NandCqPacket1*)(hNand->CqBuffer.pVirtualBuffer);
            // DMA Master Control register.
            RegVal = Nand_REGR(hNand, DMA_MST_CTRL);
            if (p->OperationName != NandOperation_Erase)
                RegVal |= NV_DRF_NUM(NAND, DMA_MST_CTRL, DMA_GO, 1);
            pCqPkt1->NandDmaMstCtrl = RegVal;
            pCqPkt1->CqCommand = (1 << CqCommand_NandDmaMstCtrl);
            // Command value register2.
            pCqPkt1->NandCmdReg2 = Command2;
            pCqPkt1->CqCommand |= (1 << CqCommand_NandCmdReg2);
            // Command register.
            RegVal = Nand_REGR(hNand, COMMAND);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            ChipSelectEnable(DeviceNumber, &RegVal);
            RegVal |= NV_DRF_DEF(NAND, COMMAND, GO, ENABLE);
            pCqPkt1->NandCmd = RegVal;
            pCqPkt1->CqCommand |= (1 << CqCommand_NandCmd);
            // Packet Id
            pCqPkt1->CqCommand |= (Offset << CqCommand_PacketId);
            CqLength += sizeof(NandCqPacket1);
        }
        // Fill address reg's + command reg's.
        else
        {
            do
            {
                if (p->DeviceNumber < (NDFLASH_CS_MAX - 1))
                {
                    p->DeviceNumber++;
                }
                else
                {
                    p->DeviceNumber = 0;
                    PageNumberOffset++;
                }
            }
            while(p->pStartPageNumbers[p->DeviceNumber] == 0xFFFFFFFF);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            pCqPkt2 = (NandCqPacket2*)(hNand->CqBuffer.pVirtualBuffer +
                                            CqLength);
            // Address register1.
            PageNumber = p->pStartPageNumbers[p->DeviceNumber];
            RegVal = p->ColumnNumber |
                ((PageNumber + PageNumberOffset) << NAND_ADDR_REG1_0_ADDR_BYTE2_SHIFT);
            pCqPkt2->NandAddrReg1 = RegVal;
            pCqPkt2->CqCommand = (1 << CqCommand_NandAddrReg1);
            // Address register2.
            RegVal = ((PageNumber + PageNumberOffset) >> NAND_ADDR_REG1_0_ADDR_BYTE2_SHIFT);
            pCqPkt2->NandAddrReg2 = RegVal;
            pCqPkt2->CqCommand |= (1 << CqCommand_NandAddrReg2);
            // Command value register2.
            pCqPkt2->NandCmdReg2 = Command2;
            pCqPkt2->CqCommand |= (1 << CqCommand_NandCmdReg2);
            // Command register.
            RegVal = Nand_REGR(hNand, COMMAND);
            ChipSelectEnable(DeviceNumber, &RegVal);
            RegVal |= NV_DRF_DEF(NAND, COMMAND, GO, ENABLE);
            if (p->OperationName != NandOperation_Read)
                RegVal |= NV_DRF_NUM(NAND, COMMAND, RD_STATUS_CHK, 1);
            pCqPkt2->NandCmd = RegVal;
            pCqPkt2->CqCommand |= (1 << CqCommand_NandCmd);
            // Packet Id
            pCqPkt2->CqCommand |= (Offset << CqCommand_PacketId);
            CqLength += sizeof(NandCqPacket2);
        }
        Offset++;
        NumOfPages--;
    }
    // Fill commad reg for issuing status read for last write page operation.
    if (p->OperationName != NandOperation_Read)
    {
        for (i = 0; i < hNand->NumberOfChipsToBeInterleaved; i++)
        {
            pCqPkt3 = (NandCqPacket3*)(hNand->CqBuffer.pVirtualBuffer + CqLength);
            // Command register.
            RegVal = NV_DRF_DEF(NAND, COMMAND, GO, ENABLE) |
                     NV_DRF_NUM(NAND, COMMAND, RD_STATUS_CHK, 1) |
                     NV_DRF_NUM(NAND, COMMAND, RBSY_CHK, 1);
            do
            {
                p->DeviceNumber++;
                if (p->DeviceNumber >= NDFLASH_CS_MAX)
                {
                    p->DeviceNumber = 0;
                }
            }
            while(p->pStartPageNumbers[p->DeviceNumber] == 0xFFFFFFFF);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            ChipSelectEnable(DeviceNumber, &RegVal);
            pCqPkt3->NandCmd = RegVal;
            pCqPkt3->CqCommand = (1 << CqCommand_NandCmd);
            // Packed Id
            pCqPkt3->CqCommand |= (Offset << CqCommand_PacketId);
            CqLength += sizeof(NandCqPacket3);
            Offset++;
        }
    }
    CqLength = (CqLength / sizeof(NvU32));
    RegVal = NV_DRF_NUM(NAND, LL_CONFIG, LL_LENGTH, CqLength) |
             NV_DRF_DEF(NAND, LL_CONFIG, BURST_SIZE, BURST_8WORDS) |
             NV_DRF_NUM(NAND, LL_CONFIG, WORD_CNT_STATUS_EN, 1);
    Nand_REGW(hNand, LL_CONFIG, RegVal);
    // Set LL_pointer register
    Nand_REGW(hNand, LL_PTR, hNand->CqBuffer.PhysBuffer);
}

static NvError GetOperationStatus(NvDdkNandHandle hNand, NvU8 DeviceNumber)
{
    NvError e = NvSuccess;
    NvU32 Response = 0;

    SetCommandQueueOperationState(hNand, 0, NULL);
    SetupRegisters(hNand, NandOperation_GetStatus);
    SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
    CleanInterruptRegisters(hNand);

    // Start Nand operation.
    StartNandOperation(hNand);
    NV_CHECK_ERROR_CLEANUP(NandWaitCommandDone(hNand));
    Response = Nand_REGR(hNand, RESP);

    if (((Response & hNand->FlashParams.OperationSuccessStatus) !=
         hNand->FlashParams.OperationSuccessStatus) ||
         (Response & 1))
    {
        // Either chip is not ready or operation failed.
        PRINT_ALL(("\r\n Status failed, Response = %x", Response));
        NV_CHECK_ERROR_CLEANUP(NvError_NandOperationFailed);
    }
fail:
    return e;
}

static void NandPowerRailEnable(NvDdkNandHandle hNand, NvBool IsEnable)
{
    NvU32 i;
    NvRmPmuVddRailCapabilities RailCaps;
    NvU32 SettlingTime;

    // As per Bug 525355
    if (hNand->pConnectivity == NULL)
        return;
    for (i = 0; i < (hNand->pConnectivity->NumAddress); i++)
    {
        // Search for the vdd rail entry
        if (hNand->pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd)
        {
            NvRmPmuGetCapabilities(hNand->RmDevHandle,
                    hNand->pConnectivity->AddressList[i].Address, &RailCaps);
            if (IsEnable)
            {
                NvRmPmuSetVoltage(hNand->RmDevHandle,
                        hNand->pConnectivity->AddressList[i].Address,
                            RailCaps.requestMilliVolts, &SettlingTime);
            }
            else
            {
                NvRmPmuSetVoltage(hNand->RmDevHandle,
                        hNand->pConnectivity->AddressList[i].Address,
                            ODM_VOLTAGE_OFF, &SettlingTime);
            }
            if (SettlingTime)
                NvOsWaitUS(SettlingTime);
        }
    }
}

// Following function includes all register operations required for resetting a Nand
// flash referred by Device number.
static NvError ResetNandFlash(NvDdkNandHandle hNand, const NvU8 DeviceNumber)
{
    NvError Error = NvSuccess;
    NvU32 RegVal = 0;

    SetupRegisters(hNand, NandOperation_Reset);
    hNand->Params.ColumnNumber = 0;
    SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
    CleanInterruptRegisters(hNand);

    // Start Nand operation.
    StartNandOperation(hNand);
    Error = WaitForCommandDone(hNand);
    if (Error != NvSuccess)
    {
        return Error;
    }
    // Check if GO did not become '0' yet. if so, send error.
    RegVal = Nand_REGR(hNand, COMMAND);
    RegVal = NV_DRF_VAL(NAND, COMMAND, GO, RegVal);
    if (RegVal)
    {
        return NvError_NandOperationFailed;
    }
    Error = GetOperationStatus(hNand, DeviceNumber);
    return Error;
}

// Nand DDK APIs.
NvError NvDdkNandOpen(NvRmDeviceHandle hRmDevice, NvDdkNandHandle *phNand)
{
    NvU32 DeviceNumber = 0;
    NvError e = NvError_Success;
    NvU32 Count = 0;
    NvDdkNandDeviceInfo DevInfo;
    NvU32 BchConfigReg = 0;
    // code for using NvOdmPeripheralEnumerate
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvU64 NandGuid = 0;
    NvU32 NumGuid = 0;
    NvOsMutexHandle hMutex;
    NvOdmPeripheralSearch SearchAttrs[] =
    {
        NvOdmPeripheralSearch_IoModule,
    };
    NvU32 SearchVals[] = 
    {
        NvOdmIoModule_Nand,
    };

    NV_ASSERT(phNand);
    NV_ASSERT(hRmDevice);

    if (s_pNandRec)
    {
        NvOsMutexLock(s_pNandRec->hMutex);
        *phNand = s_pNandRec;
        s_pNandRec->RefCount++;
        NvOsMutexUnlock(s_pNandRec->hMutex);
        return NvSuccess;
    }
    e = NvOsMutexCreate(&hMutex);
    if (e)
    {
        return e;
    }
    NvOsMutexLock(hMutex);
    
    NumGuid = NvOdmPeripheralEnumerate(SearchAttrs, SearchVals,
       NV_ARRAY_SIZE(SearchAttrs), &NandGuid, 1);
    if (NumGuid)
    {
        pConnectivity = NvOdmPeripheralGetGuid(NandGuid);
        if (!pConnectivity)
        {
            e = NvError_ModuleNotPresent;
            goto fail;
        }
    }

    // Allocate memory for handle.
    s_pNandRec = NvOsAlloc(sizeof(NvDdkNand));
    if (s_pNandRec == NULL)
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }
    NvOsMemset(s_pNandRec, 0, sizeof(NvDdkNand));
    s_pNandRec->RmDevHandle = hRmDevice;
    s_pNandRec->FlashParams.OperationSuccessStatus = SUCCESS_STATUS;
    s_pNandRec->IsNandOpen = NV_TRUE;
    s_pNandRec->pConnectivity = pConnectivity;
    s_pNandRec->hMutex = hMutex;
    // Get the base address of the Nand registers
    NvRmModuleGetBaseAddress(hRmDevice,
        NVRM_MODULE_ID(NvRmModuleID_Nand, 0),
        &(s_pNandRec->pBaseAddress), &(s_pNandRec->BankSize));
    NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(s_pNandRec->pBaseAddress,
        s_pNandRec->BankSize, NVOS_MEM_READ_WRITE, NvOsMemAttribute_Uncached,
        (void **)&(s_pNandRec->pVirtualAddress)));
    NvDdkNandGetCapabilities(s_pNandRec, &(s_pNandRec->NandCapability));
    NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&s_pNandRec->DmaDoneSema, 0));
    NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&s_pNandRec->CommandDoneSema, 0));
    // Event semaphore to register with the rm_power module
    NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&s_pNandRec->PowerMgmtSema, 0));
    // Register with the rm_power manager
    s_pNandRec->RmPowerClientId = NVRM_POWER_CLIENT_TAG('N','A','N','D');
    NV_CHECK_ERROR_CLEANUP(NvRmPowerRegister(s_pNandRec->RmDevHandle,
        s_pNandRec->PowerMgmtSema, &(s_pNandRec->RmPowerClientId)));
    // Read the NAND lock cfg before resetting the NAND controller (NAND controller
    // reset unlocks all flash).
    s_pNandRec->IsLockStatusAvailable = NV_FALSE;
    NvDdkNandResume(s_pNandRec);
    s_pNandRec->IsLockStatusAvailable = NV_TRUE;
    // As part of Suspend, Lock status is read from the controller.
    NvDdkNandSuspend(s_pNandRec);
    NvRmModuleReset(s_pNandRec->RmDevHandle, NvRmModuleID_Nand);
    NV_CHECK_ERROR_CLEANUP(InitNandController(s_pNandRec));
    // Enable interrupt
    NV_CHECK_ERROR_CLEANUP(RegisterNandInterrupt(hRmDevice, s_pNandRec));
    // Set Nand timing reg to a known Timing Value before identifying the flash.
    s_pNandRec->OptimumTiming = TIMING_VALUE;
    s_pNandRec->OptimumTiming2 = TIMING2_VALUE;
    NvDdkNandResume(s_pNandRec);
    // Disabling ONFI support by default, as it is increasing fastboot time. 
    // For enabling ONFI support "s_pNandRec->IsONFINandOnCs0" should be set to NV_TRUE 
    s_pNandRec->IsONFINandOnCs0 = NV_FALSE;
    // To identify how many flashes are present on the board.
    for (DeviceNumber = 0; DeviceNumber <
         s_pNandRec->NandCapability.NumberOfDevicesSupported; DeviceNumber++)
    {
        s_pNandRec->PhysicalDeviceNumber[DeviceNumber] = 0xFF;
        if (s_pNandRec->IsONFINandOnCs0)
        {
            e = InitONFINands(s_pNandRec, DeviceNumber);
            if (e == NvSuccess)
            {
                PRINT_ALL(("\n\r ONFI Nand is connected to CS# %d", DeviceNumber));
            }
        }
        e = NvDdkNandGetDeviceInfo(s_pNandRec, DeviceNumber, &DevInfo);
        if (e == NvError_Success)
        {
            // As Nand driver expects same flash part to be connected at 
            // all the chip selects, storing flash details in Nand handle once 
            if (!Count)
            {
                NvOsMemcpy(&(s_pNandRec->DevInfo), &DevInfo, 
                    sizeof(NvDdkNandDeviceInfo));
            }
            s_pNandRec->PhysicalDeviceNumber[Count] = DeviceNumber;
            Count++;
        }
    }
    // Return error either if the none of the flashes connected to the 
    // controller are identified or the flash is not supported. Add flash 
    // details to the ODM query table, if the flash used is not supported.
    if (!Count)
        NV_CHECK_ERROR_CLEANUP(NvError_NandFlashNotSupported);
    s_pNandRec->NumOfActiveDevices = Count;
    NvRmModuleReset(s_pNandRec->RmDevHandle, NvRmModuleID_Nand);
    s_pNandRec->IsCacheWriteSupproted = s_pNandRec->FlashParams.IsCacheWriteSupported;
    SetTimingRegVal(s_pNandRec, NV_TRUE);
    // We must restore the lock state after resets
    NandRestoreLocks(s_pNandRec);
    s_pNandRec->MaxNumOfPagesPerDMARequest =
        (s_pNandRec->NandCapability.MaxDataTransferSize) /
        (s_pNandRec->DevInfo.PageSize);
    s_pNandRec->pEccErrorData = NvOsAlloc(sizeof(NvU32) *
                              s_pNandRec->MaxNumOfPagesPerDMARequest);
    if (s_pNandRec->pEccErrorData == NULL)
        NV_CHECK_ERROR_CLEANUP(NvError_InsufficientMemory);
    // Do Command queue related initialization.
    if (s_pNandRec->NandCapability.IsCommandQueueModeSupported)
    {
        // As 4 bytes per each command queue word
        s_pNandRec->CqBuffer.BufferSize = NDFLASH_CMDQ_MAX_PKT_LENGTH * 8 *
                                        s_pNandRec->MaxNumOfPagesPerDMARequest;
        // Need a 4-byte aligned memory buffer
        NV_CHECK_ERROR_CLEANUP(MemAllocBuffer(s_pNandRec, &(s_pNandRec->CqBuffer),
            NAND_BUFFER_ALIGNMENT));
    }
    // Allocate memory required for storing error vectors.
    if (s_pNandRec->NandCapability.IsEccSupported)
    {
        if (s_pNandRec->EccAlgorithm == ECCAlgorithm_BCH)
        {
            if (s_pNandRec->TValue == 0)
                BchConfigReg |= NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL4);
            else if (s_pNandRec->TValue == 1)
                BchConfigReg |= NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL8);
            else if (s_pNandRec->TValue == 2)
                BchConfigReg |= NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL14);
            else if (s_pNandRec->TValue == 3)
                BchConfigReg |= NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL16);
            else
                NV_ASSERT(NV_FALSE);
            // Enable BCH algorithm
            BchConfigReg |= NV_DRF_DEF(NAND, BCH_CONFIG, BCH_ECC, ENABLE);
            Nand_REGW(s_pNandRec, BCH_CONFIG, BchConfigReg);
        }
        else
        {
            s_pNandRec->EccBuffer.BufferSize = GetNumOfErrorVectorBytes(s_pNandRec);
            // Need a 4-byte aligned memory buffer
            NV_CHECK_ERROR_CLEANUP(MemAllocBuffer(s_pNandRec, &(s_pNandRec->EccBuffer),
                NAND_BUFFER_ALIGNMENT));
        }
    }
    s_pNandRec->TValue = 0;
    s_pNandRec->RmDevHandle = hRmDevice;
    // Allocate physical buffer for Main data transfer
    s_pNandRec->DataBuffer.BufferSize =
                            s_pNandRec->NandCapability.MaxDataTransferSize;
    NV_CHECK_ERROR_CLEANUP(MemAllocBuffer(s_pNandRec, &(s_pNandRec->DataBuffer),
        NAND_BUFFER_ALIGNMENT));
    // Allocate physical buffer for Tag data transfer
    s_pNandRec->TagBuffer.BufferSize = s_pNandRec->DevInfo.NumSpareAreaBytes *
                                       s_pNandRec->MaxNumOfPagesPerDMARequest;
    NV_CHECK_ERROR_CLEANUP(MemAllocBuffer(s_pNandRec, &(s_pNandRec->TagBuffer),
        NAND_BUFFER_ALIGNMENT));
    s_pNandRec->Params.WaitTimeoutInMilliSeconds = NAND_COMMAND_TIMEOUT_IN_MS;
    s_pNandRec->RefCount++;
    *phNand = s_pNandRec;
    NvOsMutexUnlock(hMutex);
    return NvSuccess;
fail:
    // Disable interrupts.
    PRINT_ALL(("\nNand ddk open err:0x%x\n", e));
    if (s_pNandRec)
    {
        NvDdkNandSuspend(s_pNandRec);
        NvRmInterruptUnregister(s_pNandRec->RmDevHandle,
            s_pNandRec->InterruptHandle);
        s_pNandRec->InterruptHandle = NULL;
        NvOsSemaphoreDestroy(s_pNandRec->DmaDoneSema);
        NvOsSemaphoreDestroy(s_pNandRec->CommandDoneSema);
        DestroyMemHandle(&(s_pNandRec->EccBuffer));
        DestroyMemHandle(&(s_pNandRec->CqBuffer));
        DestroyMemHandle(&(s_pNandRec->DataBuffer));
        DestroyMemHandle(&(s_pNandRec->TagBuffer));
        NvRmPhysicalMemUnmap((void *)s_pNandRec->pVirtualAddress,
            s_pNandRec->BankSize);
        if (s_pNandRec->PowerMgmtSema)
        {
            NvRmPowerUnRegister(s_pNandRec->RmDevHandle,
                s_pNandRec->RmPowerClientId);
            NvOsSemaphoreDestroy(s_pNandRec->PowerMgmtSema);
        }
        NvOsFree((void*)s_pNandRec->pEccErrorData);
        NvOsFree(s_pNandRec);
        *phNand = NULL;
    }
    NvOsMutexUnlock(hMutex);
    NvOsMutexDestroy(hMutex);
    return e;
}

void NvDdkNandClose(NvDdkNandHandle hNand)
{
    if ((hNand == NULL) || (!hNand->IsNandOpen))
        return;

    NvOsMutexLock(hNand->hMutex);
    NV_ASSERT(hNand->RefCount);
    hNand->RefCount--;
    if (hNand->RefCount)
    {
        NvOsMutexUnlock(hNand->hMutex);
        return;
    }
    NvDdkNandSuspend(hNand);
    // Disable interrupts
    NvRmInterruptUnregister(hNand->RmDevHandle, hNand->InterruptHandle);
    hNand->InterruptHandle = NULL;
    // Destroy all memory handles.
    DestroyMemHandle(&(hNand->EccBuffer));
    DestroyMemHandle(&(hNand->CqBuffer));
    DestroyMemHandle(&(hNand->DataBuffer));
    DestroyMemHandle(&(hNand->TagBuffer));
    // Delete all semaphores.
    NvOsSemaphoreDestroy(hNand->DmaDoneSema);
    NvOsSemaphoreDestroy(hNand->CommandDoneSema);
    if (hNand->RmPowerClientId)
    {
        // Unregister with RM Power
        NvRmPowerUnRegister(hNand->RmDevHandle, hNand->RmPowerClientId);
    }
    NvOsSemaphoreDestroy(hNand->PowerMgmtSema);
    // Call RM Unmap here
    NvRmPhysicalMemUnmap((void *)(hNand->pVirtualAddress), hNand->BankSize);
    NvOsMutexUnlock(hNand->hMutex);
    NvOsMutexDestroy(hNand->hMutex);
    if (hNand->hGpio)
    {
        if (hNand->hWriteProtectPin)
        {
            NvRmGpioReleasePinHandles(hNand->hGpio,
                &hNand->hWriteProtectPin, 1);
        }
        NvRmGpioClose(hNand->hGpio);
    }
    NvOsFree(hNand);
    s_pNandRec = NULL;
}

NvError
NvDdkNandReadSpare(
    NvDdkNandHandle hNand,
    NvU8 StartDeviceNum,
    NvU32* pPageNumbers,
    NvU8* const pSpareBuffer,
    NvU8 OffsetInSpareAreaInBytes,
    NvU8 NumSpareAreaBytes)
{
    NvError e;
    NV_ASSERT(hNand);
    NV_ASSERT(pSpareBuffer != NULL);
    
    NvOsMutexLock(hNand->hMutex);
    if (!hNand->IsNandOpen)
    {    
        e = NvError_NandNotOpened;
        goto fail;
    }
    hNand->Params.DeviceNumber = StartDeviceNum;
    hNand->Params.pStartPageNumbers = pPageNumbers;
    hNand->Params.pDstnPageNumbers = NULL;
    hNand->Params.NumberOfPagesCompleted = 0;
    hNand->Params.pDataBuffer = NULL;
    hNand->Params.pTagBuffer = pSpareBuffer;
    hNand->Params.NumberOfPages = 1;
    hNand->Params.NumSpareAreaBytes = NumSpareAreaBytes;

    if (hNand->EccAlgorithm != NvOdmNandECCAlgorithm_BCH)
    {
        if (hNand->DevInfo.BusWidth == 16)
            hNand->Params.ColumnNumber = 
                (hNand->DevInfo.PageSize + OffsetInSpareAreaInBytes) >> 1;
        else
            hNand->Params.ColumnNumber = 
                hNand->DevInfo.PageSize + OffsetInSpareAreaInBytes;
    }
    else
        hNand->Params.ColumnNumber = 0;

    hNand->Params.OperationName = NandOperation_Read;
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nDDK_SpareRead:dev = %d, page_num = %d",
           StartDeviceNum, pPageNumbers[StartDeviceNum]));
    }
    #endif
    e = NandRead(hNand, NV_TRUE);
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
  }

NvError
NvDdkNandWriteSpare(
    NvDdkNandHandle hNand,
    NvU8 StartDeviceNum,
    NvU32* pPageNumbers,
    NvU8* const pSpareBuffer,
    NvU8 OffsetInSpareAreaInBytes,
    NvU8 NumSpareAreaBytes)
{
    NvError e;
    NV_ASSERT(hNand);
    NV_ASSERT(pSpareBuffer != NULL);
    NvOsMutexLock(hNand->hMutex);
    if (!hNand->IsNandOpen)
    {
        e = NvError_NandNotOpened;
        goto fail;
    }

    /* Don't allow writes to spare area containing factory bad block info */
    if (OffsetInSpareAreaInBytes == 0)
    {
        e = NvError_BadParameter;
        goto fail;
    }

    hNand->Params.DeviceNumber = StartDeviceNum;
    hNand->Params.pStartPageNumbers = pPageNumbers;
    hNand->Params.pDstnPageNumbers = NULL;
    hNand->Params.NumberOfPagesCompleted = 0;
    hNand->Params.pDataBuffer = NULL;
    hNand->Params.pTagBuffer = pSpareBuffer;
    hNand->Params.NumberOfPages = 1;
    hNand->Params.NumSpareAreaBytes = NumSpareAreaBytes;

    if (hNand->EccAlgorithm != NvOdmNandECCAlgorithm_BCH)
    {
        if (hNand->DevInfo.BusWidth == 16)
            hNand->Params.ColumnNumber = 
                (hNand->DevInfo.PageSize + OffsetInSpareAreaInBytes) >> 1;
        else
            hNand->Params.ColumnNumber = 
                hNand->DevInfo.PageSize + OffsetInSpareAreaInBytes;
    }
    else
        hNand->Params.ColumnNumber = 0;

    hNand->Params.OperationName = NandOperation_Write;
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nDDK_SpareWrite:dev = %d, page_num = %d",
           StartDeviceNum, pPageNumbers[StartDeviceNum]));
    }
    #endif
    e = NandWrite(hNand);
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError
NvDdkNandGetBlockInfo(
    NvDdkNandHandle hNand,
    NvU32 DeviceNumber,
    NvU32 BlockNumber,
    NandBlockInfo* pBlockInfo,
    NvBool SkippedBytesReadEnable)
{
    NvU32 PageNumbers[NDFLASH_CS_MAX];
    NvU32 Numpages = 1;
    NvU32 devCount;
    NvU32 i = 0;
    NvError Status;
    NV_ASSERT(pBlockInfo->pTagBuffer != NULL);

    NvOsMutexLock(hNand->hMutex);

    NV_ASSERT(pBlockInfo->pTagBuffer != NULL);
    for(devCount = 0;devCount < NDFLASH_CS_MAX;devCount++) 
        PageNumbers[devCount] = 0xFFFFFFFF;
    PageNumbers[DeviceNumber] = BlockNumber * hNand->DevInfo.PagesPerBlock;

    // To read Factory Bad Block information
    if (hNand->DevInfo.NandType == NvOdmNandFlashType_Mlc)
        PageNumbers[DeviceNumber] += (hNand->DevInfo.PagesPerBlock - 1);
    Status = NvDdkNandReadSpare(hNand, DeviceNumber, PageNumbers, pBlockInfo->pTagBuffer, 0, 4);
    if (Status != NvSuccess)
        return Status;
    if (pBlockInfo->pTagBuffer[0] == 0xFF)
        pBlockInfo->IsFactoryGoodBlock = NV_TRUE;
    else
        pBlockInfo->IsFactoryGoodBlock = NV_FALSE;

    // To check if the block is locked
    pBlockInfo->IsBlockLocked = NV_FALSE;
    while (i < NDFLASH_CS_MAX)
    {
        if ((DeviceNumber == hNand->LockAperChipId[i]) &&
            (BlockNumber > hNand->LockAperStart[i]) &&
            (BlockNumber < hNand->LockAperEnd[i]))
            {
                pBlockInfo->IsBlockLocked = NV_TRUE;
                break;
            }
        i++;
    }

    if (pBlockInfo->IsFactoryGoodBlock == NV_TRUE)
    {
        // To read Tag data
        PageNumbers[DeviceNumber] = BlockNumber * hNand->DevInfo.PagesPerBlock;
        if (SkippedBytesReadEnable)
        {
            // When skipped bytes are requested complete spare area returned
            Status = NvDdkNandReadSpare(hNand, DeviceNumber, PageNumbers,
                pBlockInfo->pTagBuffer, 0, hNand->DevInfo.NumSpareAreaBytes);
        }
        else
        {
            Status = NvDdkNandRead(hNand, DeviceNumber, PageNumbers, NULL,
                pBlockInfo->pTagBuffer,&Numpages,NV_TRUE);
        }
        if (Status != NvSuccess)
            return Status;
    }
    NvOsMutexUnlock(hNand->hMutex);
    return NvSuccess;
}

static void
PrintBadBlockIfError(
    NvDdkNandHandle hNand, 
    NvU32* pPageNumbers,
    NvError Err,
    NAND_OP OpType)
{
    NvU32 i;
    NvU32 Log2PagesPerBlock;
    NvU32 BlkNum;
    if (Err != NvSuccess)
    {
        switch(OpType)
        {
            case NAND_OP_READ:
                NvOsDebugPrintf("\n Failed Ddk Rd. Bad block ");
                break;
            case NAND_OP_WRITE:
                NvOsDebugPrintf("\n Failed Ddk Wr. Bad block");
                    break;
            case NAND_OP_ERASE:
                NvOsDebugPrintf("\n Failed Ddk Erase. Bad block");
                break;
            case NAND_OP_CPYBK:
                NvOsDebugPrintf("\n Failed Ddk Cpybk. Bad block");
                break;
            default:
                NvOsDebugPrintf("\n Failed Ddk unknown Operation. Bad block");
                break;
        }
        NvOsDebugPrintf(" Error code=0x%x ", Err);
        Log2PagesPerBlock = NandUtilGetLog2(hNand->DevInfo.PagesPerBlock);
        for (i = 0; i < hNand->DevInfo.NumberOfDevices; i++)
        {
            if (((NvS32)pPageNumbers[i]) == -1)
                continue;
            BlkNum = pPageNumbers[i] >> Log2PagesPerBlock;
            NvOsDebugPrintf(" at chip=%d,block=%d ", i, BlkNum);
        }
    }
}

NvU32 DDK_Time =0;

NvError
NvDdkNandRead(
    NvDdkNandHandle hNand,
    NvU8 StartDeviceNum,
    NvU32* pPageNumbers,
    NvU8* const pDataBuffer,
    NvU8* const pTagBuffer,
    NvU32 *pNoOfPages,
    NvBool IgnoreEccError)
{
    NvError e;
    NvU8 i;
 
    #if NAND_RANDOM_FAILURES 
    static NvU32 FakeErrCnt = 0;
    #endif
    NV_ASSERT(hNand);
    NV_ASSERT(StartDeviceNum < hNand->DevInfo.NumberOfDevices);
    NV_ASSERT((pDataBuffer != NULL) || (pTagBuffer != NULL));
    NvOsMutexLock(hNand->hMutex);
    
    if (DebugPrintEnable)
    {
        NvOsDebugPrintf("\nDDK_Rd:dev = %d, %s + %s,"
            " number_of_pages = %d", StartDeviceNum,
             (pDataBuffer ? "MAIN":"-"),
            (pTagBuffer ? "TAG":"-"), *pNoOfPages);
        for (i = 0;i < 8; i++)
        {
            if (pPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n Chip: %d, Page = %d, blk = %d\n",
                    i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
        }
    }
    if (!hNand->IsNandOpen)
    {
        e = NvError_NandNotOpened;
        goto fail;
    }

    hNand->Params.pDataBuffer = (NvU8 *)pDataBuffer;
    if ((hNand->EccAlgorithm == NvOdmNandECCAlgorithm_BCH) &&
        (pDataBuffer == NULL) && (pTagBuffer != NULL))
    {
        hNand->Params.pDataBuffer = hNand->DataBuffer.pVirtualBuffer;
    }
    hNand->Params.DeviceNumber = StartDeviceNum;
    hNand->Params.pStartPageNumbers = pPageNumbers;
    hNand->Params.pDstnPageNumbers = NULL;
    hNand->Params.NumberOfPagesCompleted = 0;
    hNand->Params.pTagBuffer = pTagBuffer;
    hNand->Params.NumberOfPages = *pNoOfPages;
    hNand->Params.ColumnNumber = GetColumnNumber(hNand);
    hNand->Params.NumSpareAreaBytes = 0;
    hNand->Params.OperationName = NandOperation_Read;
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nDDK_Read:dev = %d, page_num = %d, %s + %s,"
            "number_of_pages = %d", StartDeviceNum,
            pPageNumbers[StartDeviceNum], (pDataBuffer ? "MAIN":"-"),
           (pTagBuffer ? "TAG":"-"), hNand->Params.NumberOfPages));
    }
    #endif
    #if NAND_RANDOM_FAILURES 
   if (DebugPrintEnable)
        NvOsDebugPrintf("\r\n  Read FakeErrCnt %d", FakeErrCnt);
    FakeErrCnt++;
    if (!(FakeErrCnt % NUMBER_OF_ITERATIONS_BEFORE_ERROR) && DebugPrintEnable)
    {
        if(!IgnoreEccError)
        {
            NvOsDebugPrintf("\n\n\n\n $$$$$$ Returning Read Failure for FAKE BAD BLOCK test");
            for (i = 0;i < 8; i++)
            {
                if (pPageNumbers[i] != 0xFFFFFFFF)
                    NvOsDebugPrintf("\n FAKE BB Chip: %d, Page = %d, blk = %d\n",
                        i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
            }
        }
    }
    #endif
    e = NandRead(hNand, IgnoreEccError);
    *pNoOfPages = hNand->Params.NumberOfPagesCompleted;
    // Print Bad block for failure
    PrintBadBlockIfError(hNand, pPageNumbers, e, NAND_OP_READ);
    #if NAND_RANDOM_FAILURES 
        if (!(FakeErrCnt % NUMBER_OF_ITERATIONS_BEFORE_ERROR) && DebugPrintEnable)
        {
            if(!IgnoreEccError)
                return NvError_NandReadEccFailed;
        }
    #endif
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError
NvDdkNandWrite(
    NvDdkNandHandle hNand,
    NvU8 StartDeviceNum,
    NvU32* pPageNumbers,
    const NvU8* pDataBuffer,
    const NvU8* pTagBuffer,
    NvU32 *pNoOfPages)
{
    NvError e;
    static NvU8 flag = 0, i;
    NvU32 NumOfPages = 0;
    #if NAND_RANDOM_FAILURES
        static NvU32 FakeErrCnt = 0;
        NvBool RetFail = NV_FALSE;
    #endif
#if WRITE_VERIFY
    NvU8 * pTempDataBuffer = NULL;
    NvU8 * pRefDataBuffer = NULL;
    NvU8 * pTempTagBuffer = NULL;
    NvU8 * pRefTagBuffer = NULL;
    NvU32 TotPageSize = 0;
    NvU32 TotTagSize = 0;
    NvU32 ErrCnt = 0;
    NvError RdError;
#endif

#if WRITE_VERIFY
    if(!DDK_Time) DDK_Time = NvOsGetTimeMS();
    if ((!DebugPrintEnable) && ((NvOsGetTimeMS() - DDK_Time) > 120000))
    {
        DebugPrintEnable = 1;
        DDK_Time = 0xDEADC0DE;
    }
#endif

    NV_ASSERT(pNoOfPages); 
    NV_ASSERT(hNand);
    NV_ASSERT(StartDeviceNum < hNand->DevInfo.NumberOfDevices);
    NV_ASSERT((pDataBuffer != NULL) || (pTagBuffer != NULL));
    
    NvOsMutexLock(hNand->hMutex);
    NumOfPages = *pNoOfPages;
    if (!hNand->IsNandOpen)
    {
        e = NvError_NandNotOpened;
        goto fail;
    }
    if (DebugPrintEnable)
    {
        NvOsDebugPrintf("\nDDK_Write:device = %d, %s + %s,"
            " number_of_pages = %d", StartDeviceNum,
             (pDataBuffer ? "MAIN":"-"),
            (pTagBuffer ? "TAG":"-"), NumOfPages);
        for (i = 0;i < 8; i++)
        {
            if (pPageNumbers[i] != 0xFFFFFFFF)
            NvOsDebugPrintf("\n Chip: %d, Page = %d\n", i, pPageNumbers[i]);
        }
    }
    #if WRITE_VERIFY
    TotPageSize = NumOfPages * hNand->DevInfo.PageSize;
    if (pDataBuffer)
    {
        pTempDataBuffer = NvOsAlloc(TotPageSize);
        NV_ASSERT(pTempDataBuffer);
        pRefDataBuffer = NvOsAlloc(TotPageSize);
        NV_ASSERT(pRefDataBuffer);
        NvOsMemset(pRefDataBuffer, 0xFF, TotPageSize);
    }

    TotTagSize = NumOfPages * hNand->DevInfo.TagSize;
    if (pTagBuffer)
    {
        pTempTagBuffer = NvOsAlloc(TotTagSize);
        NV_ASSERT(pTempTagBuffer);
        pRefTagBuffer = NvOsAlloc(TotTagSize);
        NV_ASSERT(pRefTagBuffer);
        NvOsMemset(pRefTagBuffer, 0xFF, TotTagSize);
    }
#if WRITE_VERIFY
    if (DDK_Time == 0xDEADC0DE)
        DebugPrintEnable = 0;
#endif
        RdError = NvDdkNandRead(hNand, StartDeviceNum, pPageNumbers, 
        pTempDataBuffer, pTempTagBuffer, &NumOfPages, NV_TRUE);
#if WRITE_VERIFY
    if (DDK_Time == 0xDEADC0DE)
        DebugPrintEnable = 1;
#endif

    if (RdError != NvSuccess)
    {
        NvOsDebugPrintf("rd in wr fail. errcode: %d", RdError);
    }
    if (pTempDataBuffer != NULL)
    {
        i = 0;
        while (TotPageSize--)
        {
            if (*(pTempDataBuffer + i) != *(pRefDataBuffer + i))
            {
                ErrCnt++;
                i++;
            }
        }
        if (ErrCnt)
        {
            NvOsDebugPrintf("\nWrong Write, Numof Pages: %d, Cnt: %d", *pNoOfPages, ErrCnt);

            for (i = 0;i < 8; i++)
            {
                if (pPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n Chip: %d, Page = %d, blk = %d\n",
                    i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
            }
        }
    }
    if (pTempTagBuffer != NULL)
    {
        ErrCnt = 0;
        i = 0;
        while (TotTagSize--)
        {
            if (*(pTempTagBuffer + i) != *(pRefTagBuffer + i))
            {
                ErrCnt++;
                i++;
            }
        }
        if (ErrCnt)
        {
            NvOsDebugPrintf("\nWrong Tag, Numof Pages: %d, Cnt : %d", *pNoOfPages, ErrCnt);
        }
    }

    #endif
    NumOfPages = *pNoOfPages;

    hNand->Params.pDataBuffer = (NvU8 *)pDataBuffer;
    if ((hNand->EccAlgorithm == NvOdmNandECCAlgorithm_BCH) &&
        (pDataBuffer == NULL) && (pTagBuffer != NULL))
    {
        hNand->Params.pDataBuffer = hNand->DataBuffer.pVirtualBuffer;
    }

    if ((hNand->EccAlgorithm == NvOdmNandECCAlgorithm_BCH) &&
        (pDataBuffer == NULL) && (pTagBuffer != NULL))
            NV_ASSERT(NV_FALSE);

    hNand->Params.DeviceNumber = StartDeviceNum;
    hNand->Params.pStartPageNumbers = pPageNumbers;
    hNand->Params.pDstnPageNumbers = NULL;
    hNand->Params.NumberOfPagesCompleted = 0;
    hNand->Params.pTagBuffer = (NvU8 *)pTagBuffer;
    hNand->Params.NumberOfPages = *pNoOfPages;
    hNand->Params.ColumnNumber = GetColumnNumber(hNand);
    hNand->Params.NumSpareAreaBytes = 0;
    hNand->Params.OperationName = NandOperation_Write;
    if (flag)
    {
        for (i = 0;i < 8; i++)
        {
            if (pPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("In DDK write %d: PhysBlkNum = %d PageOffset = %d, Page = %d\n",
                i, pPageNumbers[i] / hNand->DevInfo.PagesPerBlock, 
                pPageNumbers[i] % hNand->DevInfo.PagesPerBlock,
                pPageNumbers[i]);
        }
    }
    #if NAND_RANDOM_FAILURES 
    if (DebugPrintEnable)
        NvOsDebugPrintf("\r\n  Write FakeErrCnt %d", FakeErrCnt);
        FakeErrCnt++;
        if (!(FakeErrCnt % NUMBER_OF_ITERATIONS_BEFORE_ERROR) && DebugPrintEnable)
            if (hNand->Params.NumberOfPages > 1)
            {
                hNand->Params.NumberOfPages -= 1;
                NvOsDebugPrintf("\n\n\n\n $$$$$$ Returning Write Failure for FAKE BAD BLOCK test");
                for (i = 0;i < 8; i++)
                {
                    if (pPageNumbers[i] != 0xFFFFFFFF)
                        NvOsDebugPrintf("\n FAKE BB Chip: %d, Page = %d, blk = %d\n",
                            i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
                }
                RetFail = NV_TRUE;
            }
   
    #endif
    e = NandWrite(hNand);
    *pNoOfPages = hNand->Params.NumberOfPagesCompleted;
    if (flag)
    {
        NvOsDebugPrintf("In DDK write: PagesReq = %d, pages trans = %d\n", 
            hNand->Params.NumberOfPages, *pNoOfPages);
    }
#if WRITE_VERIFY
    NumOfPages = *pNoOfPages;
    if (DDK_Time == 0xDEADC0DE)
        DebugPrintEnable = 0;
    RdError = NvDdkNandRead(hNand, StartDeviceNum, pPageNumbers, 
        pTempDataBuffer, pTempTagBuffer, &NumOfPages, NV_TRUE);
    if (DDK_Time == 0xDEADC0DE)
        DebugPrintEnable = 1;
    TotPageSize = NumOfPages * hNand->DevInfo.PageSize;
    TotTagSize = NumOfPages * hNand->DevInfo.TagSize;

    if (RdError != NvSuccess)
    {
        NvOsDebugPrintf("rd after wr fail. errcode: %d", RdError);
    }
    if (pTempDataBuffer != NULL)
    {
        i = 0;
        while (TotPageSize--)
        {
            if (*(pTempDataBuffer + i) != *(pDataBuffer + i))
            {
                ErrCnt++;
                i++;
            }
        }
        if (ErrCnt)
        {
            NvOsDebugPrintf("\nrd after wr Wrong Write, Numof Pages: %d, Cnt: %d",
                *pNoOfPages, ErrCnt);

            for (i = 0;i < 8; i++)
            {
                if (pPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n rd after wr Chip: %d, Page = %d\n", i, pPageNumbers[i]);
            }
        }
    }
    if (pTempTagBuffer != NULL)
    {
        ErrCnt = 0;
        i = 0;
        while (TotTagSize--)
        {
            if (*(pTempTagBuffer + i) != *(pTagBuffer + i))
            {
                ErrCnt++;
                i++;
            }
        }
        if (ErrCnt)
        {
            NvOsDebugPrintf("\nrd after wr Wrong Tag, Numof Pages: %d, Cnt : %d",
                *pNoOfPages, ErrCnt);
        }
    }

    if (pTempDataBuffer)
    {
        NvOsFree(pTempDataBuffer);
        pTempDataBuffer = NULL;
    }
    if (pTempTagBuffer)
    {
        NvOsFree(pTempTagBuffer);
        pTempTagBuffer = NULL;
    }
    if (pRefDataBuffer)
    {
        NvOsFree(pRefDataBuffer);
        pRefDataBuffer = NULL;
    }
    if (pRefTagBuffer)
    {
        NvOsFree(pRefTagBuffer);
        pRefTagBuffer = NULL;
    }
#endif
    #if NAND_RANDOM_FAILURES 
        if (RetFail && DebugPrintEnable)
            return NvError_NandWriteFailed;
    #endif
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError
NvDdkNandErase(
    NvDdkNandHandle hNand,
    NvU8 StartDeviceNum,
    NvU32* pPageNumbers,
    NvU32* pNumberOfBlocks)
{
    NvError e = NvError_Success;
    NvU32 i;
    NvU32 DeviceNumber;
    NvU32 NumberOfBlocks;
    NvU32 NumberOfBlocksToBeErased;
    NvU32 InterleaveBlockNumber;
    NvU32 FirstDeviceNum;
    NvU32 RetryCount;
    #if NAND_RANDOM_FAILURES
        static NvU32 FakeErrCnt = 0;
        FakeErrCnt++;

        if (!(FakeErrCnt % NUMBER_OF_ITERATIONS_BEFORE_ERROR))
        {
            if (DebugPrintEnable)
            {
                NvOsDebugPrintf("\n\n\n\n $$$$$$ Returning Erase Failure for FAKE BAD BLOCK test");
                for (i = 0;i < 8; i++)
                {
                    if (pPageNumbers[i] != 0xFFFFFFFF)
                        NvOsDebugPrintf("\n FAKE BB Chip: %d, Page = %d, blk = %d\n",
                            i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
                }
            }
            return NvError_NandEraseFailed;
        }
    #endif

    NV_ASSERT(hNand);
    NV_ASSERT(StartDeviceNum < hNand->DevInfo.NumberOfDevices);
    NvOsMutexLock(hNand->hMutex);
    if (!hNand->IsNandOpen)
    {
        e = NvError_NandNotOpened;
        goto fail;
    }

    if (DebugPrintEnable)
    {
        NvOsDebugPrintf("\nDDK_Ers:dev = %d,"
            " number of blks = %d", StartDeviceNum,
            *pNumberOfBlocks);
        for (i = 0;i < 8; i++)
        {
            if (pPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n Chip: %d, Page = %d, blk = %d\n",
                    i, pPageNumbers[i], (pPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
        }
    }

    for (RetryCount = 0; RetryCount < DDK_NAND_MAX_ERASE_RETRY; RetryCount++)
    {
        InterleaveBlockNumber = 0;
        NumberOfBlocks = *pNumberOfBlocks;

        // Calculate number of CS interleaved in the current transaction
        GetNumOfCsInterleaved(hNand, pPageNumbers);

        hNand->Params.NumberOfPagesCompleted = 0;
        hNand->Params.OperationName = NandOperation_Erase;
        SetCommandQueueOperationState(hNand, 0, NULL);
        do
        {
            NumberOfBlocksToBeErased =
                (NumberOfBlocks > hNand->NumberOfChipsToBeInterleaved) ?
                hNand->NumberOfChipsToBeInterleaved : NumberOfBlocks;
            FirstDeviceNum = StartDeviceNum;
            // Issue erase command for all blocks present in interleave way
            for (i = 0; i < NumberOfBlocksToBeErased; i++)
            {
                hNand->Params.StartPageNumber = pPageNumbers[StartDeviceNum] +
                            (InterleaveBlockNumber * hNand->DevInfo.PagesPerBlock);
                DeviceNumber = hNand->PhysicalDeviceNumber[StartDeviceNum];

                if (hNand->Params.NumberOfPagesCompleted > 
                        hNand->NumberOfChipsToBeInterleaved)
                {
                    e = GetOperationStatus(hNand, DeviceNumber);
                    if (e != NvError_Success)
                        NV_CHECK_ERROR_CLEANUP(NvError_NandEraseFailed);
                }
                // Set up registers.
                SetupRegisters(hNand, NandOperation_Erase);
                SetupAddressAndDeviceReg(hNand, DeviceNumber, 0);
                // Set interrupts.
                SetupInterrupt(hNand, NandOperation_Erase);
                #if NAND_DISPLAY_ALL
                if (DebugPrintEnable)
                {
                    PRINT_ALL(("\nErs DevNum = %d, PageNum = %d",
                        DeviceNumber, hNand->Params.StartPageNumber));
                }
                #endif
                // Start Nand operation.
                StartNandOperation(hNand);
                e = WaitForCommandDone(hNand);
                // Error can be returned only in last attempt
                if (RetryCount == (DDK_NAND_MAX_ERASE_RETRY - 1))
                    NV_CHECK_ERROR_CLEANUP(e);
                else
                {
                    // Retry in case of error till maximum attempts
                    if (e != NvError_Success)
                        goto LblNextTry;
                }
                do
                {
                    StartDeviceNum++;
                    if (StartDeviceNum >= NDFLASH_CS_MAX)
                    {
                        StartDeviceNum = 0;
                        InterleaveBlockNumber++;
                    }
                }
                while (pPageNumbers[StartDeviceNum] == 0xFFFFFFFF);
                hNand->Params.NumberOfPagesCompleted++;
            }
            NumberOfBlocks -= NumberOfBlocksToBeErased;
        }while (NumberOfBlocks);
        // Check status of erase command for all blocks present in interleave way
        StartDeviceNum = FirstDeviceNum;
        for (i = 0; i < hNand->NumberOfChipsToBeInterleaved; i++)
        {
            DeviceNumber = hNand->PhysicalDeviceNumber[StartDeviceNum];
            e = GetOperationStatus(hNand, DeviceNumber);
            if (e != NvSuccess)
                NV_CHECK_ERROR_CLEANUP(NvError_NandEraseFailed);
            do
            {
                StartDeviceNum++;
                if (StartDeviceNum >= NDFLASH_CS_MAX)
                {
                    StartDeviceNum = 0;
                }
            }
            while (pPageNumbers[StartDeviceNum] == 0xFFFFFFFF);
        }
LblNextTry: 
        if (e == NvError_Success)
        {
            // Return if success
            break;
        }
    }

fail:
    if (e == NvError_Timeout)
    {
        NAND_ASSERT(NvError_Timeout);
        DumpRegData(hNand);
        NvRmModuleReset(hNand->RmDevHandle, NvRmModuleID_Nand);
        SetTimingRegVal(hNand, NV_FALSE);
        // Restore the NAND lock cfg that was stored during previous Suspend, 
        // as locks should have got released due to above reset operation.
        NandRestoreLocks(hNand);
    }
    // Return the number of blocks erased
    *pNumberOfBlocks = hNand->Params.NumberOfPagesCompleted;
    hNand->OperationStatus = e;
    // Print Bad block for failure
    PrintBadBlockIfError(hNand, pPageNumbers, e, NAND_OP_ERASE);
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

static NvError NandCopyback(NvDdkNandHandle hNand, NvBool IgnoreEccError)
{
    NvError Error = NvSuccess;
    NandParams *p = &hNand->Params;
    NvU32* pSourcePageNumbers = p->pStartPageNumbers;
    NvU32* pDstnPageNumbers = p->pDstnPageNumbers;
    NvU32 NoOfPages;
    NvU32 i;
    NvU32 NoOfFullPg2Copyback = p->NumberOfPages;
    NvU8 DeviceNum = p->DeviceNumber;
    NvU8 DstDeviceNum = p->DstnDeviceNumber;
    NvU32 SrcPgNums[MAX_NAND_SUPPORTED];
    NvU32 DstPgNums[MAX_NAND_SUPPORTED] ;
    NvU32 NumOfPagesTransferred = 0;
    #if ENABLE_INTERNAL_COPYBACK
    NvU32 SrcBlock[MAX_NAND_SUPPORTED];
    NvU32 DstnBlock[MAX_NAND_SUPPORTED];
    NvU32 SrcPlane[MAX_NAND_SUPPORTED];
    NvU32 DstnPlane[MAX_NAND_SUPPORTED];
    NvU32 BlocksPerZone = hNand->DevInfo.NoOfBlocks / hNand->DevInfo.ZonesPerDevice ;
    NvBool DoInternalCopyBk = NV_FALSE;
    #endif
    NvU32 PagesPerInterleaveColumn;
    NvBool IsTagRequired = NV_FALSE;
    #if WRITE_VERIFY
    NvU32 TotPageSize = 0;
    NvU32 ErrCnt = 0;
    NvError RdErr;
    #endif
    if (DebugPrintEnable)
    {
        NvOsDebugPrintf("\nDDK_Cpbk:Srcdev = %d, Dstdev = %d,"
            " number_of_pages = %d", DeviceNum, DstDeviceNum,
            hNand->Params.NumberOfPages);
        for (i = 0;i < 8; i++)
        {
            if (pSourcePageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n SrcChip: %d, Page = %d, blk = %d\n",
                    i, pSourcePageNumbers[i], (pSourcePageNumbers[i]/hNand->DevInfo.PagesPerBlock));
            if (pDstnPageNumbers[i] != 0xFFFFFFFF)
                NvOsDebugPrintf("\n DstChip: %d, Page = %d, blk = %d\n",
                    i, pDstnPageNumbers[i], (pDstnPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
        }
    }

    // Calculate number of CS interleaved in the current transaction
    GetNumOfCsInterleaved(hNand, pSourcePageNumbers);

    for (i = 0; i < NDFLASH_CS_MAX; i++)
    {
        SrcPgNums[i] = pSourcePageNumbers[i];
        DstPgNums[i] = pDstnPageNumbers[i];
        #if ENABLE_INTERNAL_COPYBACK
        SrcBlock[i] = (SrcPgNums[i] / hNand->DevInfo.PagesPerBlock);
        DstnBlock[i] = (DstPgNums[i] / hNand->DevInfo.PagesPerBlock);
        SrcPlane[i] = SrcBlock[i] / BlocksPerZone;
        DstnPlane[i] = DstnBlock[i] / BlocksPerZone;
        #endif
    }

    if ((!(pSourcePageNumbers[DeviceNum] % hNand->DevInfo.PagesPerBlock)) &&
        (NoOfFullPg2Copyback == 1))
        IsTagRequired = NV_TRUE;

    while (NoOfFullPg2Copyback > 0)
    {
        #if ENABLE_INTERNAL_COPYBACK
        for (i = 0; i < hNand->NumberOfChipsToBeInterleaved; i++)
        {
            if ((SrcPgNums[i] != -1) &&
                (DstPgNums[i] != -1) &&
                (SrcPlane[i] == DstnPlane[i]) &&
                ((SrcBlock[i] & 0x1) == (DstnBlock[i] & 0x1)))
                DoInternalCopyBk = NV_TRUE;
        }
        if (DoInternalCopyBk)
        {
            NoOfPages = NoOfFullPg2Copyback;
            Error = NandInternalCopyback(hNand);
            if (Error != NvSuccess)
            {
                Error = NvError_NandCopyBackFailed;
                NumOfPagesTransferred = p->NumberOfPagesCompleted;
                goto fail;
            }
            NumOfPagesTransferred = NoOfPages;
        }
        else
        #endif
        {
            NoOfPages = hNand->NandCapability.MaxDataTransferSize /
                hNand->DevInfo.PageSize;
            if (NoOfFullPg2Copyback < NoOfPages)
            {
                NoOfPages = NoOfFullPg2Copyback;
            }

            p->DeviceNumber = DeviceNum;
            p->NumberOfPagesCompleted = 0;
            p->ColumnNumber = GetColumnNumber(hNand);
            p->OperationName = NandOperation_Read;
            p->NumberOfPages = NoOfPages;
            p->pStartPageNumbers = SrcPgNums;
            p->pDataBuffer = hNand->DataBuffer.pVirtualBuffer;
            if (IsTagRequired)
            {
                p->pTagBuffer = hNand->TagBuffer.pVirtualBuffer;
            }
            else
            {
                p->pTagBuffer = NULL;
            }

            Error = NandRead(hNand, IgnoreEccError);
            if (Error != NvSuccess)
            {
                Error = NvError_NandReadEccFailed;
                PrintBadBlockIfError(hNand, SrcPgNums, Error, NAND_OP_READ);
                goto fail;
            }
            p->DeviceNumber = DstDeviceNum;
            p->NumberOfPagesCompleted = 0;
            p->OperationName = NandOperation_Write;
            p->pStartPageNumbers = DstPgNums;
            p->pDataBuffer = hNand->DataBuffer.pVirtualBuffer;
            if (IsTagRequired)
            {
                p->pTagBuffer = hNand->TagBuffer.pVirtualBuffer;
            }
            else
            {
                p->pTagBuffer = NULL;
            }
            Error = NandWrite(hNand);
            if (Error != NvSuccess)
            {
                Error = NvError_NandWriteFailed;
                PrintBadBlockIfError(hNand, DstPgNums, Error, NAND_OP_WRITE);
                goto fail;
            }

#if WRITE_VERIFY
            p->DeviceNumber = DstDeviceNum;
            p->NumberOfPagesCompleted = 0;
            p->ColumnNumber = 0;
            p->OperationName = NandOperation_Read;
            p->NumberOfPages = NoOfPages;
            p->pStartPageNumbers = DstPgNums;
            p->pDataBuffer = s_WriteVerifyBuffer;
            TotPageSize = NoOfPages * hNand->DevInfo.PageSize;
            RdErr = NandRead(hNand, IgnoreEccError);
            if (RdErr != NvSuccess)
            {
                PrintBadBlockIfError(hNand, SrcPgNums, Error, NAND_OP_READ);
                NvOsDebugPrintf("rd in cpbk validation fail.post write errcode: %d", Error);
            }
            for (i = 0; i < TotPageSize; i++)
            {
                if (s_WriteVerifyBuffer[i] != hNand->DataBuffer.pVirtualBuffer[i])
                {
                    ErrCnt++;
                }
            }
            if (ErrCnt)
            {
                NvOsDebugPrintf("\nWrong Cpbk post write, Numof Pages: %d, Cnt: %d",
                    NoOfPages, ErrCnt);
            }
#endif
            NumOfPagesTransferred += NoOfPages;
            NoOfFullPg2Copyback -= NoOfPages;
            PagesPerInterleaveColumn = NoOfPages / hNand->NumberOfChipsToBeInterleaved;
            for (i = 0; i < NDFLASH_CS_MAX; i++)
            {
                if (SrcPgNums[i] != 0xFFFFFFFF)
                {
                    SrcPgNums[i] += PagesPerInterleaveColumn;
                }
                if (DstPgNums[i] != 0xFFFFFFFF)
                {
                    DstPgNums[i] += PagesPerInterleaveColumn;
                }
            }
        }
    }
fail:
    p->NumberOfPagesCompleted = NumOfPagesTransferred;
    return Error;
}

#if ENABLE_INTERNAL_COPYBACK
static NvError NandInternalCopyback(NvDdkNandHandle hNand)
{
    NandParams *p = &hNand->Params;
    NvError e = NvSuccess;
    NvU32 i = 0;
    NvU32 j = 0;
    NvU8 DeviceNumber = 0;
    NvU8 StartDeviceNumber = 0;
    NvU32 PageNumber = 0;
    NvU32 ReadPageNumberOffset = 0;
    NvU32 WritePageNumberOffset = 0;
    NvU32 NumberOfPages = 0;

    SetCommandQueueOperationState(hNand, p->NumberOfPages, NULL);

    // Calculate number of CS interleaved in the current transaction
    GetNumOfCsInterleaved(hNand, p->pSourcePageNumbers);

    for (i = 0; i < p->NumberOfPages; i += NumberOfPages)
    {
        // Setup for copyback read.
        SetupRegisters(hNand, NandOperation_CopybackRead);
        StartDeviceNumber = p->DeviceNumber;
        NumberOfPages = 0;
        for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
        {
            SkipUnusedDevices(hNand, p->pStartPageNumbers, &(p->DeviceNumber),
                &ReadPageNumberOffset);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            PageNumber = p->pStartPageNumbers[p->DeviceNumber];
            SetupAddressAndDeviceReg(hNand, DeviceNumber,
                    (PageNumber + ReadPageNumberOffset));
            SetupInterrupt(hNand, NandOperation_Read);
            // Start Nand operation.
            StartNandOperation(hNand);
            NV_CHECK_ERROR_CLEANUP(WaitForCommandDone(hNand));
            p->DeviceNumber++;
            if (p->DeviceNumber == hNand->NumOfActiveDevices)
            {
                p->DeviceNumber = 0;
                ReadPageNumberOffset++;
            }
            NumberOfPages++;
            if ((i + NumberOfPages) == p->NumberOfPages)
                break;
        }
        // Setup for copyback program.
        p->DeviceNumber = StartDeviceNumber;
        SetupRegisters(hNand, NandOperation_CopybackProgram);
        NumberOfPages = 0;
        p->DeviceNumber = StartDeviceNumber;
        for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
        {
            SkipUnusedDevices(hNand, p->pDstnPageNumbers, &(p->DeviceNumber),
                &WritePageNumberOffset);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            PageNumber = p->pDstnPageNumbers[p->DeviceNumber];
            SetupAddressAndDeviceReg(hNand, DeviceNumber,
                    (PageNumber + WritePageNumberOffset));
            SetupInterrupt(hNand, NandOperation_Write);
            // Start Nand operation.
            StartNandOperation(hNand);
            NV_CHECK_ERROR_CLEANUP(WaitForCommandDone(hNand));
            p->DeviceNumber++;
            if (p->DeviceNumber == hNand->NumOfActiveDevices)
            {
                p->DeviceNumber = 0;
                WritePageNumberOffset++;
            }
            NumberOfPages++;
            if ((i + NumberOfPages) == p->NumberOfPages)
                break;
        }
        // Check the status.
        p->DeviceNumber = StartDeviceNumber;
        SetupRegisters(hNand, NandOperation_CopybackProgram);
        NumberOfPages = 0;
        p->DeviceNumber = StartDeviceNumber;
        for (j = 0; j < hNand->NumberOfChipsToBeInterleaved; j++)
        {
            SkipUnusedDevices(hNand, p->pDstnPageNumbers, &(p->DeviceNumber),
                NULL);
            DeviceNumber = hNand->PhysicalDeviceNumber[p->DeviceNumber];
            NV_CHECK_ERROR_CLEANUP(GetOperationStatus(hNand, DeviceNumber));
            p->DeviceNumber++;
            if (p->DeviceNumber == hNand->NumOfActiveDevices)
            {
                p->DeviceNumber = 0;
            }
            p->NumberOfPagesCompleted++;
            NumberOfPages++;
            if ((i + NumberOfPages) == p->NumberOfPages)
                break;
        }
    }
fail:
    NAND_ASSERT(e);
    if (hNand->IsCommandQueueOperation)
    {
    // Number of pages transferred to be divided by 2 for copy back as each
    // copy back command requires two sets of command queue commands.i.e. one
    // for copy-back-read & another for copy-back-write
        if (e == NvError_NandCommandQueueError)
            p->NumberOfPagesCompleted = (hNand->NumOfPagesTransferred / 2);
        hNand->IsCommandQueueOperation = NV_FALSE;
    }
    if (e == NvError_Timeout)
    {
        DumpRegData(hNand);
        NvRmModuleReset(hNand->RmDevHandle, NvRmModuleID_Nand);
        SetTimingRegVal(hNand, NV_FALSE);
        // We must restore the lock state after resets
        NandRestoreLocks(hNand);
    }
    hNand->OperationStatus = e;
    return e;
}
#endif

NvError
NvDdkNandCopybackPages(
    NvDdkNandHandle hNand,
    NvU8 SrcStartDeviceNum,
    NvU8 DstStartDeviceNum,
    NvU32* pSrcPageNumbers,
    NvU32* pDestPageNumbers,
    NvU32 *pNoOfPages,
    NvBool IgnoreEccError)
{
    NvError Error;
    #if NAND_RANDOM_FAILURES
        static NvU32 FakeErrCnt = 0;
        NvBool RetFail = NV_FALSE;
        NvU32 i = 0;
    #endif
    NV_ASSERT(hNand);
    NV_ASSERT(*pNoOfPages);
    NV_ASSERT(SrcStartDeviceNum < NDFLASH_CS_MAX);
    NV_ASSERT(DstStartDeviceNum < NDFLASH_CS_MAX);
    NV_ASSERT(hNand->IsNandOpen);
    NvOsMutexLock(hNand->hMutex);

    hNand->Params.DeviceNumber = SrcStartDeviceNum;
    hNand->Params.DstnDeviceNumber = DstStartDeviceNum;
    hNand->Params.pStartPageNumbers = pSrcPageNumbers;
    hNand->Params.pDstnPageNumbers = pDestPageNumbers;
    hNand->Params.pDataBuffer = NULL;
    hNand->Params.pTagBuffer = NULL;
    hNand->Params.NumberOfPages = *pNoOfPages;
    hNand->Params.NumSpareAreaBytes = 0;
    hNand->Params.OperationName = NandOperation_CopybackRead;
    #if NAND_DISPLAY_ALL
    if (DebugPrintEnable)
    {
        PRINT_ALL(("\nDDk_CpyBk: Dev1 = 0x%x, Dev2 = 0x%x, SrcPage = 0x%x, DstnPage = 0x%x, "
                    "NoOfPages = 0x%x", SrcStartDeviceNum, DstStartDeviceNum,
                    pSrcPageNumbers[SrcStartDeviceNum],
                    pDestPageNumbers[DstStartDeviceNum],
                    *pNoOfPages));
    }
    #endif
    #if NAND_RANDOM_FAILURES 
    if (DebugPrintEnable)
        NvOsDebugPrintf("\r\n  copyBack FakeErrCnt %d", FakeErrCnt);
        FakeErrCnt++;
        if (!(FakeErrCnt % NUMBER_OF_ITERATIONS_BEFORE_ERROR) && DebugPrintEnable)
            if (hNand->Params.NumberOfPages > 1)
            {
                hNand->Params.NumberOfPages -= 1;

                NvOsDebugPrintf("\n\n\n\n $$$$$$ Returning copyback Failure for FAKE BAD BLOCK test");
                for (i = 0;i < 8; i++)
                {
                    if (pSrcPageNumbers[i] != 0xFFFFFFFF)
                        NvOsDebugPrintf("\n FAKE BB Chip: %d, Page = %d, blk = %d\n",
                            i, pDestPageNumbers[i], (pDestPageNumbers[i]/hNand->DevInfo.PagesPerBlock));
                }
                RetFail = NV_TRUE;
            }
    #endif
    Error = NandCopyback(hNand, IgnoreEccError);
    *pNoOfPages = hNand->Params.NumberOfPagesCompleted;
    #if NAND_RANDOM_FAILURES 
        if (RetFail && DebugPrintEnable)
            return NvError_NandWriteFailed;
    #endif
    NvOsMutexUnlock(hNand->hMutex);
    return Error;
}

NvError
NvDdkNandGetDeviceInfo(
    NvDdkNandHandle hNand,
    NvU8 DeviceNumber,
    NvDdkNandDeviceInfo* pDeviceInfo)
{
    NvError e;
    NvU32  ReadID = 0;
    NvU32 BlockSize = 0;
    NvU32 TempValue = 0;
    NvU32 SpareSizePer512Bytes = 0;
    NvU32 SpareAreaSize = 0;
    const NvU32 TValueArray[] = {0, 1, 2, 0};
    NvOdmNandFlashParams *pFlashParams;

    /* validate input params */
    NV_ASSERT(hNand);
    NV_ASSERT(pDeviceInfo);
    NV_ASSERT(DeviceNumber < NDFLASH_CS_MAX);
    NV_ASSERT(hNand->IsNandOpen);
    
    NvOsMutexLock(hNand->hMutex);
    
    SetCommandQueueOperationState(hNand, 0, NULL);
    NV_CHECK_ERROR_CLEANUP(NandReadID(hNand, DeviceNumber, &ReadID, NV_FALSE));
    pFlashParams = NvOdmNandGetFlashInfo(ReadID);
    if (!pFlashParams)
    {
        NV_CHECK_ERROR_CLEANUP(NvError_NandFlashNotSupported);
    }
    else
        NvOsMemcpy(&(hNand->FlashParams), pFlashParams,
            sizeof(NvOdmNandFlashParams));
    hNand->DevInfo.VendorId = hNand->FlashParams.VendorId;
    hNand->DevInfo.DeviceId = hNand->FlashParams.DeviceId;
    hNand->DevInfo.NandType = hNand->FlashParams.NandType;
    // Currently internal copy back is disabled, till error handling in FTL is 
    //fixed.
//    hNand->IsCopybackSupported = pFlashParams->IsCopyBackCommandSupported;
    hNand->IsCopybackSupported = NV_FALSE;

    if (hNand->FlashParams.NandDeviceType == NvOdmNandDeviceType_Type2)
    {
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, PAGE_SIZE, ReadID);
        NV_ASSERT(TempValue <= 3);
        hNand->DevInfo.PageSize = (1 << (11 + TempValue));

        // If MSB bit is Reserved it is unknown flash.
        // This should be 0. This can be updated if any flash supports.
        TempValue = NV_DRF_VAL(DDK_42NM_NAND, ID_DECODE, BLOCK_SIZE_MSB, ReadID);
        if (TempValue)
            NV_ASSERT(0);

        // Extracting the block size
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, BLOCK_SIZE, ReadID);
        NV_ASSERT(TempValue <= 3 );
        BlockSize = (1 << (17 + TempValue));

        // If MSB bit is Reserved it is unknown flash.
        // This should be 0. This can be updated if any flash supports.
        TempValue = 
            NV_DRF_VAL(DDK_42NM_NAND, ID_DECODE, REDUNDANT_AREA_SIZE_MSB, ReadID);
        if (TempValue)
            NV_ASSERT(0);

        // Extracting the redundant data size.
        TempValue = 
            NV_DRF_VAL(DDK_42NM_NAND, ID_DECODE, REDUNDANT_AREA_SIZE, ReadID);
        if (TempValue == 1)
            SpareAreaSize = 128;
        else if (TempValue == 2)
            SpareAreaSize = 218;
        else
            NV_ASSERT(0);

        // Bus width cannot be decoded from ReadId like other Nand Flashes.
        // So bus width is 8 for 42nm Falsh.
        hNand->DevInfo.BusWidth = 8;
    }
    else
    {
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, PAGE_SIZE, ReadID);
        NV_ASSERT(TempValue <= 3);
        hNand->DevInfo.PageSize = (1 << (10 + TempValue));
        // Extracting the block size
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, BLOCK_SIZE, ReadID);
        NV_ASSERT(TempValue <= 3 );
        BlockSize = (1 << (16 + TempValue));

        // Extracting redundant Area size per 512 bytes
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, REDUNDANT_AREA_SIZE, ReadID);
        // If TempValue calculated above is 0 then redundant area per 512 bytes is
        // 8 bytes and if it is 1 then redundant area per 512 bytes is 16 bytes.
        SpareSizePer512Bytes = TempValue? 16: 8;
        // Calculate spare area size per flash page, from the
        // spare area per 512 bytes read from the flash.
        SpareAreaSize = ((hNand->DevInfo.PageSize / 512) *
                        SpareSizePer512Bytes);

        // Extracting flash organization 8bit or 16 bit
        TempValue = NV_DRF_VAL(DDK_NAND, ID_DECODE, BUS_WIDTH, ReadID);
        hNand->DevInfo.BusWidth = TempValue? 16: 8;
    }

    hNand->SpareAreaSize = SpareAreaSize;
    hNand->DevInfo.PagesPerBlock = BlockSize / hNand->DevInfo.PageSize;
    if (hNand->NandCapability.IsEccSupported)
    {
        if (pFlashParams->EccAlgorithm == NvOdmNandECCAlgorithm_BCH)
        {
            if (hNand->IsBCHEccSupported)
            {
                hNand->EccAlgorithm = ECCAlgorithm_BCH;
                hNand->TValue = TValueArray[pFlashParams->ErrorsCorrectable];
            }
            else // fall back to next best Ecc algorithm available, for ap15
            {
                hNand->EccAlgorithm = ECCAlgorithm_ReedSolomon;
                hNand->TValue = TValueArray[pFlashParams->ErrorsCorrectable];
            }
        }
        else if (pFlashParams->EccAlgorithm == NvOdmNandECCAlgorithm_ReedSolomon)
        {
            hNand->EccAlgorithm = ECCAlgorithm_ReedSolomon;
            hNand->TValue = TValueArray[pFlashParams->ErrorsCorrectable];
        }
        else if (pFlashParams->EccAlgorithm == NvOdmNandECCAlgorithm_Hamming)
            hNand->EccAlgorithm = ECCAlgorithm_Hamming;
        else if (pFlashParams->EccAlgorithm == NvOdmNandECCAlgorithm_NoEcc)
        {
            hNand->EccAlgorithm = ECCAlgorithm_None;
            hNand->NandCapability.IsEccSupported = NV_FALSE;
        }
        else
            NV_ASSERT(NV_FALSE);
    }
    hNand->DevInfo.TagOffset = GetNumOfParityBytesForMainArea(hNand) +
                            (hNand->FlashParams.SkippedSpareBytes << 2);

    // Redundant area size per page to write any Tag information. This will be
    // calculated as TagSize = spareAreaSize - mainAreaEcc - SpareAreaEcc.
    if (hNand->EccAlgorithm == ECCAlgorithm_None)
        hNand->DevInfo.TagSize = SpareAreaSize;
    else
        hNand->DevInfo.TagSize = SpareAreaSize -
                        GetNumOfParityBytesForMainArea(hNand) -
                        NDFLASH_PARITY_SZ_HAMMING_SPARE;
    hNand->DevInfo.TagSize -= (hNand->FlashParams.SkippedSpareBytes << 2);
    hNand->DevInfo.NumSpareAreaBytes = SpareAreaSize;
    NV_ASSERT(SpareAreaSize > hNand->DevInfo.TagOffset);
    if (hNand->DevInfo.TagSize % 4)
    {
        hNand->DevInfo.TagSize -= (hNand->DevInfo.TagSize % 4);
    }
    // Assert here if Tag size is less than 12 bytes, as 12 bytes are required
    // for storing bad block management etc.
    NV_ASSERT(hNand->DevInfo.TagSize >= 8);
#if 1
    hNand->DevInfo.NoOfBlocks = (hNand->FlashParams.BlocksPerZone) *
                              (hNand->FlashParams.ZonesPerDevice);
#else
    hNand->DevInfo.NoOfBlocks = 2048;
#endif
    hNand->DevInfo.DeviceCapacityInKBytes = hNand->DevInfo.NoOfBlocks *
                                                            (BlockSize / 1024);
    hNand->DevInfo.InterleaveCapability = hNand->FlashParams.InterleaveCapability;
    hNand->DevInfo.ZonesPerDevice = hNand->FlashParams.ZonesPerDevice;
    hNand->DevInfo.NumberOfDevices = hNand->NumOfActiveDevices;
    NvOsMemcpy(pDeviceInfo, &(hNand->DevInfo), sizeof(NvDdkNandDeviceInfo));
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

void NvDdkNandGetCapabilities(
    NvDdkNandHandle hNand,
    NvDdkNandDriverCapabilities* pNandDriverCapabilities)
{
    static NvDdkNandDriverCapabilities s_NandCap[2];
    static NvRmModuleCapability s_NandCaps[] =
    {
        {1, 1, 0, &s_NandCap[0]}, // (Major version, Minor version) = (1,1) for AP15
        {1, 2, 0, &s_NandCap[1]} // (Major version, Minor version) = (1,2) for AP20
    };

    NvDdkNandDriverCapabilities *pNandCapability = NULL;
    NvU32 i = 0;

    NV_ASSERT(hNand);
    NV_ASSERT(pNandDriverCapabilities);
    NvOsMutexLock(hNand->hMutex);
    for (i = 0; i < 2; i++)
    {
            #if IS_CQ_ENABLED
            s_NandCap[i].IsCommandQueueModeSupported = NV_TRUE;
            #else
            s_NandCap[i].IsCommandQueueModeSupported = NV_FALSE;
            #endif
        s_NandCap[i].IsEccSupported = NV_TRUE;
        s_NandCap[i].IsEdoModeSupported = NV_TRUE;
        s_NandCap[i].IsInterleavingSupported = NV_FALSE;
        s_NandCap[i].NumberOfDevicesSupported = NDFLASH_CS_MAX;
#if NV_OAL
        s_NandCap[i].MaxDataTransferSize = NAND_MAX_BYTES_PER_PAGE;
#else
        s_NandCap[i].MaxDataTransferSize = NDFLASH_DMA_MAX_BYTES;
#endif
        s_NandCap[i].TagEccParitySize = NDFLASH_PARITY_SZ_HAMMING_SPARE;
        s_NandCap[i].ControllerDefaultTiming = TIMING_VALUE;
    }
    // (Major version, Minor version) = (1,1) for AP15
    s_NandCap[0].IsBCHEccSupported = NV_FALSE;
    // (Major version, Minor version) = (1,2) for AP20
    s_NandCap[1].IsBCHEccSupported = NV_TRUE;
    s_NandCap[1].NumberOfDevicesSupported = 0x2;
    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hNand->RmDevHandle,
        NVRM_MODULE_ID(NvRmModuleID_Nand, 0),
                            s_NandCaps, 2, (void **)&(pNandCapability)));
    NvOsMemcpy(pNandDriverCapabilities, pNandCapability,
        sizeof(NvDdkNandDriverCapabilities));
    hNand->IsBCHEccSupported = pNandCapability->IsBCHEccSupported;
    
    NvOsMutexUnlock(hNand->hMutex);
}

NvU8 GetBitPosition(NvU32 Number)
{
    NvU8 BitPosition = 0;
    while(Number)
    {
        Number >>= 1;
        BitPosition++;
    }
    return BitPosition-1;
}

void NvDdkNandGetLockedRegions(
    NvDdkNandHandle hNand,
    LockParams* pFlashLockParams)
{
    NvU32 i;
    NvU32 LockApertureMask;
    NvU32 LockCtrlReg;
    NvU32 LockRegOffset;
    
    NvOsMutexLock(hNand->hMutex);
    LockCtrlReg = Nand_REGR(hNand, LOCK_CONTROL);
    LockApertureMask = 1;
    hNand->NumberOfAperturesUsed = 0;
    for (i = 0; i < NDFLASH_CS_MAX; i++)
    {
        pFlashLockParams[i].DeviceNumber = 0xFF;
        if(LockCtrlReg & LockApertureMask)
        {
            // Lock aperture enabled...read the aperture cfg
            LockRegOffset = 
                (NAND_LOCK_APER_CHIPID1_0 - NAND_LOCK_APER_CHIPID0_0) * i;
            pFlashLockParams[hNand->NumberOfAperturesUsed].StartPageNumber =
                Nand_REGR_OFFSET(hNand, LOCK_APER_START0, LockRegOffset);
            pFlashLockParams[hNand->NumberOfAperturesUsed].EndPageNumber =
                Nand_REGR_OFFSET(hNand, LOCK_APER_END0, LockRegOffset);
            pFlashLockParams[hNand->NumberOfAperturesUsed].DeviceNumber =
                GetBitPosition(Nand_REGR_OFFSET(hNand, LOCK_APER_CHIPID0, 
                LockRegOffset));
            hNand->NumberOfAperturesUsed++;
        }
        LockApertureMask <<= 1;
    }
    NvOsMutexUnlock(hNand->hMutex);
}


void NvDdkNandSetFlashLock(NvDdkNandHandle hNand, LockParams* pFlashLockParams)
{
    NvU32 Offset;

    NV_ASSERT(hNand);
    NV_ASSERT(hNand->NumberOfAperturesUsed < NDFLASH_CS_MAX);
    
    NvOsMutexLock(hNand->hMutex);
    if (hNand->NumberOfAperturesUsed < NDFLASH_CS_MAX)
    {
        Offset = (NAND_LOCK_APER_CHIPID1_0 - NAND_LOCK_APER_CHIPID0_0) *
                 hNand->NumberOfAperturesUsed;
        // Program the startpage/endpage NAND row address into the start/end Lock aperture
        // registers
        Nand_REGW_OFFSET(hNand, LOCK_APER_START0, Offset, pFlashLockParams->StartPageNumber);
        Nand_REGW_OFFSET(hNand, LOCK_APER_END0, Offset, pFlashLockParams->EndPageNumber);
        Nand_REGW_OFFSET(hNand, LOCK_APER_CHIPID0, Offset,
            (1 << pFlashLockParams->DeviceNumber));
        Nand_REGW(hNand, LOCK_CONTROL, (1 << hNand->NumberOfAperturesUsed));

        // Store away a copy of the new lock settings so they can be restored should a NAND
        // controller reset be necessary (NAND controller reset unlocks all flash).
        hNand->LockAperStart[hNand->NumberOfAperturesUsed] = pFlashLockParams->StartPageNumber;
        hNand->LockAperEnd[hNand->NumberOfAperturesUsed] = pFlashLockParams->EndPageNumber;
        hNand->LockAperChipId[hNand->NumberOfAperturesUsed] = (1 << pFlashLockParams->DeviceNumber);

        hNand->NumberOfAperturesUsed++;
    }
    NvOsMutexUnlock(hNand->hMutex);
}

void NvDdkNandReleaseFlashLock(NvDdkNandHandle hNand)
{
    NvU32 i;
    NV_ASSERT(hNand);
    NvOsMutexLock(hNand->hMutex);
    // Issue a H/W reset to the NAND controller to release all Locked regions
    NvRmModuleReset(hNand->RmDevHandle, NvRmModuleID_Nand);
    // Re-initialize controller timing registers.
    SetTimingRegVal(hNand, NV_FALSE);
    // Reset NAND lock apertures
    hNand->NumberOfAperturesUsed = 0;
    for(i = 0; i < NDFLASH_CS_MAX; i++)
    {
        hNand->LockAperStart[i] = 0;
        hNand->LockAperEnd[i] = 0;
        hNand->LockAperChipId[i] = 0;
    }
    NvOsMutexUnlock(hNand->hMutex);
}

NvError NvDdkNandSuspendClocks(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;

    NvOsMutexLock(hNand->hMutex);
    if (!hNand->IsNandClkEnabled)
    {
        e = NvSuccess;
        goto fail;
    }

    /* Disable the clock */
    NV_CHECK_ERROR_CLEANUP(NvRmPowerModuleClockControl(hNand->RmDevHandle,
        NvRmModuleID_Nand, hNand->RmPowerClientId, NV_FALSE));
    hNand->IsNandClkEnabled = NV_FALSE;
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError NvDdkNandResumeClocks(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;
    NvRmDfsBusyHint BusyHints[3];
    NvBool BusyAttribute = NV_TRUE;

    NvOsMutexLock(hNand->hMutex);
    if (hNand->IsNandClkEnabled)
    {
        e = NvSuccess;
        goto fail;
    }
    BusyHints[0].ClockId = NvRmDfsClockId_Emc;
    BusyHints[0].BoostDurationMs = 50;
    BusyHints[0].BoostKHz = 100000;
    BusyHints[0].BusyAttribute = BusyAttribute;

    BusyHints[1].ClockId = NvRmDfsClockId_Ahb;
    BusyHints[1].BoostDurationMs = 50;
    BusyHints[1].BoostKHz = 100000;
    BusyHints[1].BusyAttribute = BusyAttribute;

    BusyHints[2].ClockId = NvRmDfsClockId_Cpu;
    BusyHints[2].BoostDurationMs = 50;
    BusyHints[2].BoostKHz = 350000;
    BusyHints[2].BusyAttribute = BusyAttribute;

    NvRmPowerBusyHintMulti(hNand->RmDevHandle,
                           hNand->RmPowerClientId,
                           BusyHints,
                           3,
                           NvRmDfsBusyHintSyncMode_Async);
    /* Enable clk to Nand controller */
    NV_CHECK_ERROR_CLEANUP(EnableNandClock(hNand));
    SetTimingRegVal(hNand, NV_FALSE);
    hNand->IsNandClkEnabled = NV_TRUE;
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError NvDdkNandSuspend(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;

    if (hNand->IsNandSuspended)
    {
        /* already in suspend state */
        return e;
    }
    /* disable clock */
    NvDdkNandSuspendClocks(hNand);
    NvOsMutexLock(hNand->hMutex);
    /* save lock data */
    if (hNand->IsLockStatusAvailable)
        NandLoadLockCfg(hNand);
    /* disable power */
    NandPowerRailEnable(hNand, NV_FALSE);
    NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(hNand->RmDevHandle,
        NvRmModuleID_Nand, hNand->RmPowerClientId, NvRmVoltsOff, NvRmVoltsOff,
        NULL, 0, NULL));
    /* enter suspend state */
    hNand->IsNandSuspended = NV_TRUE;
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

NvError NvDdkNandResume(NvDdkNandHandle hNand)
{
    NvError e = NvSuccess;

    if (!hNand->IsNandSuspended) {
        /* already in resume state */
        return e;
    }
    NvOsMutexLock(hNand->hMutex);
    /* Enable power to the Nand controller */
    NV_CHECK_ERROR_CLEANUP(EnableNandPower(hNand));
    /* Restore lock data into Nand registers */
    if (hNand->IsLockStatusAvailable) {
        NandRestoreLocks(hNand);
    }
    NvOsMutexUnlock(hNand->hMutex);
    /* enable clock outside mutex lock */
    e = NvDdkNandResumeClocks(hNand);
    if (e != NvSuccess) {
        /* failed clock enable */
        return e;
    }
    /* enter resume state */
    hNand->IsNandSuspended = NV_FALSE;
    return e;
fail:
    NvOsMutexUnlock(hNand->hMutex);
    return e;
}

