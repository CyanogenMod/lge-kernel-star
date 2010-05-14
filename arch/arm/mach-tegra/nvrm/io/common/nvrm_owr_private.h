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
 *
 * @b Description: Contains the OWR declarations.
 */

#ifndef INCLUDED_NVRM_OWR_PRIVATE_H
#define INCLUDED_NVRM_OWR_PRIVATE_H

#include "nvrm_module.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_gpio.h"
#include "nvrm_owr.h"

/* Maximum number of OWR controller instances supported */
#define MAX_OWR_INSTANCES     1

#define MAX_OWR_CLOCK_SPEED_KHZ 1000

#define OWR_NO_OF_BITS_PER_BYTE 8

// ROM id size
#define OWR_ROM_ID_SIZE_BYTES     8

/* OWR ROM commands */
#define OWR_ROM_READ_COMMAND    0x33
#define OWR_ROM_SKIP_COMMAND     0xCC

/* OWR MEM commands */
#define OWR_MEM_READ_COMMAND    0xF0
#define OWR_MEM_WRITE_COMMAND   0x0F

/**
 * @brief OWR interrupt status bits
 */
typedef enum
{
    // Presence Error Interrupt enable
    OwrIntrStatus_PresenceErrIntEnable  = 0x1,

    // CRC Error Interrupt enable
    OwrIntrStatus_CrcErrIntEnable  = 0x2,

    // Mem write Error Interrupt enable
    OwrIntrStatus_MemWriteErrIntEnable  = 0x4,

    // Error Command Interrupt enable
    OwrIntrStatus_ErrCommandIntEnable  = 0x8,

    // Reset done Interrupt enable
    OwrIntrStatus_RstDoneIntEnable  = 0x10,

    // Presence done Interrupt enable
    OwrIntrStatus_PresenceDoneIntEnable  = 0x20,

    // ROM Command done Interrupt enable
    OwrIntrStatus_RomCmdDoneIntEnable  = 0x40,

    // MEM Command done Interrupt enable
    OwrIntrStatus_MemCmdDoneIntEnable  = 0x80,

    // TXF overflow Interrupt enable
    OwrIntrStatus_TxfOvfIntEnable  = 0x100,

    // RXF underrun Interrupt enable
    OwrIntrStatus_RxfUnrIntEnable  = 0x200,

    // Dglitch Interrupt enable
    OwrIntrStatus_DglitchIntEnable  = 0x400,

    // TX FIFO Data Request Interrupt enable
    OwrIntrStatus_TxFifoDataReqIntEnable  = 0x800,

    // RX FIFO Data Request Interrupt enable
    OwrIntrStatus_RxFifoDataReqIntEnable  = 0x1000,

    // Bit transfer done Interrupt enable
    OwrIntrStatus_BitTransferDoneIntEnable  = 0x2000,
    
    /** Force to 32 bit */
    OwrIntrStatus_Force32 = 0x7FFFFFFF
} OwrIntrStatus;

/**
 * @brief OWR interrupt status bits
 */
typedef enum
{
    // Ready bit
    OwrStatus_Rdy = 0x0,

    // Tx FIFO Full
    OwrStatus_TxfFull = 0,
        
    // Tx FIFO Empty
    OwrStatus_TxfEmpty = 0,

    // RTx FIFO Full
    OwrStatus_RxfFull = 0,

    // Rx FIFO Empty
    OwrStatus_RxfEmpty = 0,

     // Tx Flush
    OwrStatus_TxfFlush = 0,

     // Rx Flush
    OwrStatus_RxfFlush = 0,

    // Rx Fifo Full Count
    OwrStatus_RxFifoFullCnt = 0,

    // Tx Fifo empty Count
    OwrStatus_TxFifoEmptyCnt = 0,

    // Reset
    OwrStatus_Rpp = 0,

    // Write bit 0
    OwrStatus_Write0 = 0,

    // Write bit 1
    OwrStatus_Write1 = 0,

    // Read bit
    OwrStatus_Read = 0,

    // Read Sampled bit
    OwrStatus_ReadSampledBit = 0,

    /** Force to 32 bit */
    OwrStatus_Force32 = 0x7FFFFFFF
} OwrStatus;

typedef enum
{
    // Specifies a read transaction.
    OWR_READ,
    // Specifies a write transaction.
    OWR_WRITE,
} OwrTransactionType;

struct NvRmOwrControllerRec;

/* OWR controller state. There are will one instance of this structure for each
 * OWR controller instance */
typedef struct NvRmOwrControllerRec
{
    /* Controller static Information */

    /* Rm device handle */
    NvRmDeviceHandle hRmDevice;
    
    /* Contains the owrtransfer status */
    NvU32 OwrTransferStatus;
    
    /* Contains the number of opened clients */
    NvU32 NumberOfClientsOpened;
    
    /* Contains the semaphore id to block the synchronous owr client calls */
    NvOsSemaphoreHandle OwrSyncSemaphore;
    
    /* Contains the mutex for providing the thread safety */
    NvOsMutexHandle OwrThreadSafetyMutex;
    
    /* Power clinet ID */
    NvU32 OwrPowerClientId;
    
    /* Contoller module ID. */
    NvRmModuleID ModuleId;
    
    /* Instance of the above specified module */
    NvU32 Instance;
    
    // OWR interrupt handle for this controller instance
    NvOsInterruptHandle OwrInterruptHandle;
 
    /* Read data */
    NvError (*read)(struct NvRmOwrControllerRec *c, NvU8 * pBuffer, 
               NvRmOwrTransactionInfo Transaction);
    
    /* Send data */
    NvError (*write)(struct NvRmOwrControllerRec *c, NvU8 * pBuffer, 
                NvRmOwrTransactionInfo Transaction);

    /* Shutdown the controller */
    void (*close)(struct NvRmOwrControllerRec *c);

    //  OWR capabiity for this SOC only.
    NvU32* pOwrVirtualAddress;
    NvU32 OwrBankSize;
    NvRmPhysAddr OwrPhysicalAddress;
} NvRmOwrController;

/** OWR SOC capability structure. */
typedef struct SocOwrCapabilityRec
{
    NvU32 NoOfInstances;
} NvRmOwrCapability;


/**
 * brief Initialze the controller to start the data transfer 
 *
 * This APIs should always return NvSuccess
 *
 * @param c    OWR controller structure.
 * */
NvError AP20RmOwrOpen(NvRmOwrController *c);

#endif  // INCLUDED_NVRM_OWR_PRIVATE_H


