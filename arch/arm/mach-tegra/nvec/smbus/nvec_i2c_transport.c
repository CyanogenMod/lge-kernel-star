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
 * @brief <b> I2C Slave mode Implementation</b>
 *
 * @b Description: Fill Here.
 */

#include "nvos.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_gpio.h"
#include "nvrm_power.h"
#include "ap20/ari2c.h"
#include "nvrm_pinmux.h"
#include "nvrm_module.h"
#include "nvodm_modules.h"
#include "nvec_transport.h"
#include "nvrm_interrupt.h"
#include "nvec_smbus_priv.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_hardware_access.h"
#include "nvodm_query_discovery.h"

#define ENABLE_NEW_SLAVE 1
#define ADD_ACK_DELAY 1
#define DELAY_COUNT 0x1E
#define MAX_NACKS   1

#ifndef __KERNEL__
#define __KERNEL__ 0
#endif

#if (__KERNEL__) && (NVOS_IS_LINUX)
#include <linux/irqflags.h>
#endif 

// Enable this to print debug messages.
#define DEBUG_I2C_SLAVE 0
// Enable this to log data transmitted/received on SM bus.
#define CAPTURE_TX_RX_DATA 0

#if DEBUG_I2C_SLAVE
#define PRINT_I2C_MESSAGES(X)    NvOsDebugPrintf X;
#else
#define PRINT_I2C_MESSAGES(X)    
#endif

#define I2C_SMBUS_TRASNSPORT_GUID NV_ODM_GUID('I','2','c','S','m','B','u','s')

// Register access Macros.
#define I2C_REGR(t, reg) NV_REGR((t)->hRmDevice, (t)->ModuleId, (t)->Instance, \
            (t)->I2cRegisterOffset + I2C_##reg##_0);
#define I2C_REGW(t, reg, val) NV_REGW((t)->hRmDevice, (t)->ModuleId, \
            (t)->Instance, ((t)->I2cRegisterOffset + I2C_##reg##_0), (val));

// Maximum size of SMBus packet (write block protocol):
// 1Command byte + 1Block Count byte + 32max data bytes + 1PEC byte + 1rsvd = 36
#define ECT_MAX_PACKET_SIZE (1 + 1 + NVEC_SMBUS_MAX_TRANSFER_SIZE + 1 + 1)
#define ECT_MAX_RECEIVE_BUFFERS 4
#define ENABLE_I2C_SLAVE(t) \
    NV_DRF_NUM(I2C, I2C_SL_CNFG, NEWSL, t->I2cSocCaps.IsNewSlaveAvailable) | \
    NV_DRF_DEF(I2C, I2C_SL_CNFG, NACK, DISABLE) | \
    NV_DRF_DEF(I2C, I2C_SL_CNFG, RESP, DISABLE)

#define DISABLE_I2C_SLAVE(t) \
    NV_DRF_NUM(I2C, I2C_SL_CNFG, NEWSL, t->I2cSocCaps.IsNewSlaveAvailable) | \
    NV_DRF_DEF(I2C, I2C_SL_CNFG, NACK, ENABLE) | \
    NV_DRF_DEF(I2C, I2C_SL_CNFG, RESP, DISABLE)

#if CAPTURE_TX_RX_DATA
#define CAPTURE_DATA_BUFFER_SIZE 2048

typedef struct
{
    NvU8 Buffer[CAPTURE_DATA_BUFFER_SIZE];
    NvU32 sts[CAPTURE_DATA_BUFFER_SIZE];
    NvBool PktStatus[CAPTURE_DATA_BUFFER_SIZE];
    NvU32 Index;
} I2cSlaveData;
volatile I2cSlaveData g_I2cSlaveRxData;
volatile I2cSlaveData g_I2cSlaveTxData;
#endif 

typedef enum
{
    EctPacketType_Response_Request = 0,
    EctPacketType_Event,
    EctPacketType_Num,
    EctPacketType_Force32 = 0x7FFFFFFF
} EctPacketType;

typedef enum
{
    EctEventLength_2Byte = 0,
    EctEventLength_3Byte,
    EctEventLength_Variable,
    EctEventLength_Num,
    EctEventLength_Force32 = 0x7FFFFFFF
} EctEventLength;

typedef struct
{
    NvU8 Buffer[ECT_MAX_PACKET_SIZE];
    NvBool BufferInUse;
    NvBool DataValid;
} EctRxBuffer;

/**
 * SOC I2C capability structure.
 */
typedef struct SocI2cCapabilityRec
{
    // Tells whether new slave is available or not.
    NvBool IsNewSlaveAvailable;
} SocI2cCapability;

typedef struct NvEcTransportRec
{
    // Indicares whether controller is initialized.
    NvBool IsInitialized;
    // Rm device handle.
    NvRmDeviceHandle hRmDevice;
    // Gpio handle.
    NvRmGpioHandle hGpio;
    // EC send request pin handle.
    NvRmGpioPinHandle hEcRequestPin;
    // Ec send request pin info.
    const NvOdmGpioPinInfo* EcRequestPinInfo;
    // Power clinet ID.
    NvU32 I2cPowerClientId;
    // Contoller module ID.
    NvRmModuleID ModuleId;
    // Instance of the above specified module.
    NvU32 Instance;
    // Slave address in byte format.
    NvU32 SlaveAddress;
    // I2C interrupt handle for this controller instance.
    NvOsInterruptHandle I2CInterruptHandle;
    // Data send to Master is pending.
    NvBool SendPending;
    // Flag to indicate max number of NACKS sent
    NvBool MaxNacksSent;
    // To track number of bytes sent out.
    NvU8 SendCounter;
    // Buffer to send the data to master from.
    NvU8 SendBuffer[ECT_MAX_PACKET_SIZE];
    // Number of bytes to send to master.
    NvU32 NumBytesToSend;
    // Allocated Receive buffer.
    EctRxBuffer* ReceiveBufferForPacket;
    // Number of packets Received from Ec 
    NvU8 NumPacketsRcvd;
    // Buffer for receiving data from Master.
    NvU8* PacketBuffer;
    // Counter to keep track of received data.
    NvU8 PacketCounter;
    // Counter to track the number of NACKs
    NvU8 NackCounter;
    // Packet type
    EctPacketType PktType;
    // Trnasfer type
    EctEventLength EventLength;
    // Array of Receive Buffers.
    EctRxBuffer RxBuffers[ECT_MAX_RECEIVE_BUFFERS];
    // This gives Free buffer index in the array of RxBuffers.
    NvU8 BufferFreeIndex;
    //This gives Valid buffer index in the array of RxBuffers.
    NvU8 BufferValidIndex;
    // Status of Rx and Tx.
    NvU32 Status;
    // Notification sema on send/receive complete.
    NvOsSemaphoreHandle hEcNotifySema;
    // Contains the mutex for providing the thread safety.
    NvOsIntrMutexHandle I2cThreadSafetyMutex;
    /* Controller run time state. These members will be polulated before the HAL
     * functions are called. HAL functions should only read these members and
     * should not clobber these registers. */
    /* I2c clock freq */
    NvU32 ClockSpeedInKHz;
    /* timeout for the transfer */
    NvU32 timeout;
    /* Though all the controllers have same register spec, their start address
     * doesn't match. DVC contoller I2C register start address differs from the I2C
     * controller. */
    NvU32 I2cRegisterOffset;
    // I2C capabilities.
    SocI2cCapability I2cSocCaps;
} NvRmI2cSlaveController;

static NvRmI2cSlaveController g_I2cTransport;
static NvBool g_IsInitialized = NV_FALSE;

static EctRxBuffer* AllocReceiveBuffer(NvEcTransportHandle t)
{
    NvU8 Index = t->BufferFreeIndex;
    
    if (t->RxBuffers[Index].BufferInUse == NV_TRUE)
        // No Buffer available.
        return NULL;
    t->RxBuffers[Index].BufferInUse = NV_TRUE;
    t->RxBuffers[Index].DataValid = NV_FALSE;
    t->BufferFreeIndex++;
    if (t->BufferFreeIndex == ECT_MAX_RECEIVE_BUFFERS)
        t->BufferFreeIndex = 0;
    return &t->RxBuffers[Index];
}

static void FreeReceiveBuffer(EctRxBuffer* pRxBuffer)
{
    pRxBuffer->BufferInUse = NV_FALSE;
}

static void GetBufferForRxData(NvEcTransportHandle t)
{
    if (t->ReceiveBufferForPacket != NULL)
        t->ReceiveBufferForPacket->DataValid = NV_TRUE;
    t->ReceiveBufferForPacket = AllocReceiveBuffer(t);
    if (t->ReceiveBufferForPacket != NULL)
        t->PacketBuffer = t->ReceiveBufferForPacket->Buffer;
    else
        t->PacketBuffer = NULL;
}

static NvBool 
HwI2cHandleVariableWrite(
    NvEcTransportHandle t, 
    NvU8 Data, 
    NvBool IsPECSupported)
{
    NvBool PktRcvd = NV_FALSE;
    NvU8 PECBytes = IsPECSupported ? 1 : 0;

    switch (t->PacketCounter)
    {
        /*
         * This case may happen only when protocol is broken.
         * Reset the counter.
         */
        case 1:
            t->PacketCounter = 0;
            break;
        /*
         * The 2nd byte is SMBus Block Count, if not between 1 and 32, 
         * Reset the counter.
         */
        case 2:
            if ( (Data == 0) || (Data > NVEC_SMBUS_MAX_TRANSFER_SIZE) )
                t->PacketCounter = 0;
            break;
        /*
         * For all other bytes just check if it is the last byte of the 
         * packet: 
         * 1st Command Byte + 2nd Block Count byte + stored Block Count 
         * + PEC byte 
         * Byte # = 1 + 1 + (packetBuffer[1]) + 1
         */
        default:
            if (t->PacketCounter == (2 + t->PacketBuffer[1] + PECBytes))
            {
                PktRcvd = NV_TRUE;
            }
            break;
    }
    return PktRcvd;
}

static void 
HwI2cHandleVariableRead(
    NvEcTransportHandle t, 
    NvU32 SlaveStatus, 
    NvBool IsPECSupported,
    NvU64 IsrStartTime)
{
    NvU8 DataToSend = 0xFD;
    NvBool ValidProtocol = NV_FALSE;
    NvU8 PECBytes = IsPECSupported ? 1 : 0;
    NvRmGpioPinState PinState = NvRmGpioPinState_High;

    // No request is pending. Just send 0xFF's to release bus and clear intr.
    if (t->SendPending == NV_FALSE)
    {
        I2C_REGW(t, I2C_SL_RCVD, DataToSend);
        PRINT_I2C_MESSAGES(("DW Invalid=0x%x", DataToSend));
        goto exit;
    }

    if (NV_DRF_VAL(I2C, I2C_SL_STATUS, RCVD, SlaveStatus))
    {
        #if ADD_ACK_DELAY
        // Work around for AP20 New Slave Hw Bug. Give 1us extra.
        while( (NvOsGetTimeUS() - IsrStartTime) < (((1000 / t->ClockSpeedInKHz) / 2) + 1) );
        #endif

        /* 
         * Write the 1st data byte to the master provided the slave read 
         * transaction has re-started immediately after the master wrote 
         * the request command to the slave.
         */
        if ( (t->PacketCounter == 1) && 
             (t->PacketBuffer[0] == NVEC_READ_REQUEST_COMMAND) )
        {
            DataToSend = t->SendBuffer[t->SendCounter++];
            PRINT_I2C_MESSAGES(("\r\nNew DW =0x%x ", DataToSend));
            I2C_REGW(t, I2C_SL_RCVD, DataToSend);
            ValidProtocol = NV_TRUE;
            // De-assert the Gpio Line here.
            if (t->hEcRequestPin)
                NvRmGpioWritePins(t->hGpio, &t->hEcRequestPin, &PinState, 1);
        }
    }
    else
    {
        /*
         * Write the "next" (1+) data byte to the master provided its 
         * number is within requested length; if enabled write PEC byte 
         * after all requested data bytes.
         */
        if ( (t->SendCounter >= 1) && 
             (t->SendCounter < (t->NumBytesToSend + PECBytes)) )
        {
            DataToSend = t->SendBuffer[t->SendCounter++];
            PRINT_I2C_MESSAGES(("DW=0x%x ", DataToSend));
            I2C_REGW(t, I2C_SL_RCVD, DataToSend);
            ValidProtocol = NV_TRUE;
        }
        else if ( (t->PacketCounter == t->NumBytesToSend) && 
                  IsPECSupported )
        {
            // Send PEC byte if PEC enabled - PEC is not supported, yet.
            NV_ASSERT(NV_FALSE);
        }
    }

    /*
     * Invalid protocol - keep data line OD (send all 1's) and 
     * reset packet counter.
     */
    if (ValidProtocol == NV_FALSE)
    {
        I2C_REGW(t, I2C_SL_RCVD, 0xFF);
        PRINT_I2C_MESSAGES(("DW Invalid=0xFF "));
    }
    // Process Master read completion.
    else if (t->SendCounter == (t->NumBytesToSend + PECBytes))
    {
        NvOsIntrMutexLock(t->I2cThreadSafetyMutex);
        if (t->SendPending)
        {
            PRINT_I2C_MESSAGES(("DW End\r\n"));
            t->NumBytesToSend = 0;
            t->SendPending = NV_FALSE;
            t->Status |= NVEC_TRANSPORT_STATUS_SEND_COMPLETE;
            // Set Status to Send Complete.
            // Reset Packet buffer counter.
            // Trigger Notification Sema.
            NvOsSemaphoreSignal(t->hEcNotifySema);
        }
        NvOsIntrMutexUnlock(t->I2cThreadSafetyMutex);
    }
exit:
    t->PacketCounter = 0;
    #if CAPTURE_TX_RX_DATA
    if (g_I2cSlaveTxData.Index >= CAPTURE_DATA_BUFFER_SIZE)
        g_I2cSlaveTxData.Index = 0;
    g_I2cSlaveTxData.sts[g_I2cSlaveTxData.Index] = SlaveStatus;
    g_I2cSlaveTxData.PktStatus[g_I2cSlaveTxData.Index] = t->SendPending;
    g_I2cSlaveTxData.Buffer[g_I2cSlaveTxData.Index++] = DataToSend;
    #endif
}

static NvU32 ReadSlaveRcvdRegister(NvRmI2cSlaveController* t, NvU32 SlaveStatus)
{
    NvU32 Data;
    NvU32 ClearReg;

    ClearReg = NV_DRF_VAL(I2C, I2C_SL_STATUS, RCVD, SlaveStatus);
    if (t->I2cSocCaps.IsNewSlaveAvailable && ClearReg)
    {
        #if (__KERNEL__) && (NVOS_IS_LINUX)
        unsigned long flags;
        local_irq_save(flags);
        #endif
        // Read data byte to release the bus.
        Data = (NvU8)I2C_REGR(t, I2C_SL_RCVD);
        // Workaround for AP20 New I2C Slave Controller bug #626607.
        I2C_REGW(t, I2C_SL_RCVD, 0);
        #if (__KERNEL__) && (NVOS_IS_LINUX)
        local_irq_restore(flags);
        #endif
    }
    else
    {
        // Read data byte to release the bus.
        Data = (NvU8)I2C_REGR(t, I2C_SL_RCVD);
    }
    return Data;
}

/*
 * Slave always acks master, if it is ready to receive data.
 * If Slave is not ready to receive/send data, it nacks.
 * This driver supports
 *      1.Write Block(To recive Response/Event from Master).
 *      2.Write Byte-Read Block(To Send Request to Master).
 *      3.Write Byte(To recive 1 byte Event from Master).
 *      4.Write Word(To recive word Event from Master).
 */
static void Isr(void* args)
{
    NvU8 Data = 0xCC;
    NvU32 SlaveConfig;
    NvBool PktRcvd = NV_FALSE;
    volatile NvU32 SlaveStatus;
    NvBool CaptureData = NV_TRUE;
    // FIXME: Get it from ODM?
    NvBool IsPECSupported = NV_FALSE;
    NvRmI2cSlaveController* t = args;
    NvU64 IsrStartTime = NvOsGetTimeUS();
    NvU8 PECBytes = IsPECSupported ? 1 : 0;
    NvU32 ExpectedInterruptMask = 
        NV_DRF_DEF(I2C, I2C_SL_STATUS, END_TRANS, DEFAULT_MASK) | 
        NV_DRF_DEF(I2C, I2C_SL_STATUS, SL_IRQ, DEFAULT_MASK) | 
        NV_DRF_DEF(I2C, I2C_SL_STATUS, RCVD, DEFAULT_MASK) |
        NV_DRF_DEF(I2C, I2C_SL_STATUS, RNW, DEFAULT_MASK);

    // Read the slave status register.
    SlaveStatus = I2C_REGR(t, I2C_SL_STATUS);
    // validate interrupts.
    if (ExpectedInterruptMask)
    {
        NV_ASSERT(SlaveStatus & ExpectedInterruptMask);
        NV_ASSERT((SlaveStatus & (~ExpectedInterruptMask)) == 0);
    }

    // Filter out if it is only END_TRANS Event.
    if (NV_DRF_VAL(I2C, I2C_SL_STATUS, SL_IRQ, SlaveStatus) == 0)
    {
        NvOsDebugPrintf("\n********* Spurious I2C Slave Interrupt ******\n");
        return;
    }
    if ( NV_DRF_VAL(I2C, I2C_SL_STATUS, END_TRANS, SlaveStatus) && 
         (!NV_DRF_VAL(I2C, I2C_SL_STATUS, RCVD, SlaveStatus)) )
    {
        //NvOsDebugPrintf("\n************Just END_TRANS********\n");
        goto exit;
    }
    // Check if Master is requesting data from Slave. i.e Read from Slave.
    if (NV_DRF_VAL(I2C, I2C_SL_STATUS, RNW, SlaveStatus))
    {
        HwI2cHandleVariableRead(t, SlaveStatus, IsPECSupported, IsrStartTime);
    }
    // Data is sent to Slave by Master. i.e Write to Slave.
    else if (t->PacketBuffer != NULL)
    {
        // Read data byte to release the bus.
        Data = ReadSlaveRcvdRegister(t, SlaveStatus);
        /*
         * If the 1st Command byte of the new transaction is received, reset the
         * packet counter regardless of the previous count, and store the 
         * received byte.
         */
        if (NV_DRF_VAL(I2C, I2C_SL_STATUS, RCVD, SlaveStatus))
        {
            PRINT_I2C_MESSAGES(("\r\nNew DR =0x%x, sts=0x%x ", Data, SlaveStatus));
            t->PacketCounter = t->SendCounter = 0;
            if (t->I2cSocCaps.IsNewSlaveAvailable)
            {
                CaptureData = NV_FALSE;
                NV_ASSERT(Data == t->SlaveAddress);
            }
        }

        if ( (t->PacketCounter == 0) && CaptureData )
        {
            PRINT_I2C_MESSAGES(("\r\nNew DR =0x%x, sts=0x%x ", Data, SlaveStatus));
            t->PacketBuffer[t->PacketCounter++] = Data;
            t->PktType = (EctPacketType)NV_DRF_VAL(NVEC, COMMAND, PACKET_TYPE, 
                                            Data);
            t->EventLength = (EctEventLength)NV_DRF_VAL(NVEC, COMMAND, 
                                                EVENT_LENGTH, Data);
            NV_ASSERT(t->PktType < EctPacketType_Num);
            NV_ASSERT(t->EventLength < EctEventLength_Num);
        }
        else if ( (t->PacketCounter == 0) && !CaptureData)
        {
            // New slave shows up here during new transaction. Nothing to do.
        }
        /*
         * If the "next" (2nd+) transaction byte is received, store data 
         * and advance slave state machine packet counter.
         */
        else
        {
            PRINT_I2C_MESSAGES(("DR =0x%x, sts=0x%x ", Data, SlaveStatus));
            if (t->PacketCounter < ECT_MAX_PACKET_SIZE) 
                t->PacketBuffer[t->PacketCounter++] = Data;

            if ( (t->EventLength == EctEventLength_Variable) || 
                 (t->PktType == EctPacketType_Response_Request) )
                PktRcvd = HwI2cHandleVariableWrite(t, Data, IsPECSupported);
            // Events fall through.
            else if (t->EventLength == EctEventLength_2Byte)
                PktRcvd = (t->PacketCounter == (2 + PECBytes)) ? 1 : 0;
            else if (t->EventLength == EctEventLength_3Byte)
                PktRcvd = (t->PacketCounter == (3 + PECBytes)) ? 1 : 0;
            else
                NV_ASSERT(NV_FALSE);
        }
        #if CAPTURE_TX_RX_DATA
        if (g_I2cSlaveRxData.Index >= CAPTURE_DATA_BUFFER_SIZE)
            g_I2cSlaveRxData.Index = 0;
        g_I2cSlaveRxData.sts[g_I2cSlaveRxData.Index] = SlaveStatus;
        g_I2cSlaveRxData.PktStatus[g_I2cSlaveRxData.Index] = PktRcvd;
        g_I2cSlaveRxData.Buffer[g_I2cSlaveRxData.Index++] = Data;
        #endif
    }
    else if (t->PacketBuffer == NULL)
    {
        // Enable NACK here.
        SlaveConfig = DISABLE_I2C_SLAVE(t);
        I2C_REGW(t, I2C_SL_CNFG, SlaveConfig);
        // Read data byte to release bus.
        Data = ReadSlaveRcvdRegister(t, SlaveStatus);
        PRINT_I2C_MESSAGES(("DR=0x%x ", Data));
        #if CAPTURE_TX_RX_DATA
        if (g_I2cSlaveRxData.Index >= CAPTURE_DATA_BUFFER_SIZE)
            g_I2cSlaveRxData.Index = 0;
        g_I2cSlaveRxData.sts[g_I2cSlaveRxData.Index] = SlaveStatus;
        g_I2cSlaveRxData.Buffer[g_I2cSlaveRxData.Index++] = Data;
        #endif

	// increment counter which tracks number of nacks
	t->NackCounter++;

	/*
	 * if the count ever exceeds max number of nacks permitted
	 * (as called out in the spec), reset the count and set a flag
	 * indicating max nacks have been sent. This allows the upper layer to
	 * send a ping to get the EC back in sync.
	*/
	if (t->NackCounter >= MAX_NACKS) {
	    t->NackCounter = 0; // reset count
	    t->MaxNacksSent = NV_TRUE;
	    NvOsSemaphoreSignal(t->hEcNotifySema);
	}
    }
    if (PktRcvd)
    {
        // Copy Data to receive buffer.
        // increment Buffers Received.
        // Reset Packet buffer counter.
        // Trigger Notification Sema.
        GetBufferForRxData(t);
        PRINT_I2C_MESSAGES(("DR End\r\n"));
        NvOsIntrMutexLock(t->I2cThreadSafetyMutex);
        t->NumPacketsRcvd++;
        NvOsIntrMutexUnlock(t->I2cThreadSafetyMutex);
        t->PacketCounter = 0;
	t->NackCounter = 0;
        NvOsSemaphoreSignal(t->hEcNotifySema);
    }
exit:
    NvRmInterruptDone(t->I2CInterruptHandle);
}

static NvError HwI2cConfigPower(NvEcTransportHandle t, NvBool IsEnablePower)
{
    NvError e;
    NvRmModuleID ModuleId = NVRM_MODULE_ID(t->ModuleId, t->Instance);
    
    if (IsEnablePower == NV_TRUE)
    {
        // Enable Power.
        NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(t->hRmDevice, ModuleId,
            t->I2cPowerClientId, NvRmVoltsUnspecified, 
            NvRmVoltsUnspecified, NULL, 0, NULL));
        // Enable the clock to the i2c controller
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(t->hRmDevice, ModuleId, 
            t->I2cPowerClientId, NV_TRUE));
    }
    else
    {
        // Disable the clock to the i2c controller
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(t->hRmDevice, ModuleId, 
            t->I2cPowerClientId, NV_FALSE));
        // Disable power
        NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(t->hRmDevice, ModuleId,
            t->I2cPowerClientId, NvRmVoltsOff, NvRmVoltsOff, NULL, 0, NULL));
    }
fail:
    return e;
}

static NvError HwI2cInitController(NvRmI2cSlaveController* t)
{
    NvError e;
    NvU32 IrqList;
    NvU32 NumOdmConfigs;
    const NvU32 *pOdmConfigs;
    NvU32 SlaveConfig;
    NvOsInterruptHandler IntHandlers;
    NvOdmIoModule OdmModule = NvOdmIoModule_I2c;
    // It seems like the I2C Controller has an hidden clock divider,
    // whose value is 8. So, request for clock value multipled by 8.
    NvU32 PrefClockFreq = t->ClockSpeedInKHz * 8;
    
    NV_CHECK_ERROR_CLEANUP(NvOsIntrMutexCreate(&t->I2cThreadSafetyMutex));
    // Bring the pins out of tristate.
    NV_CHECK_ERROR_CLEANUP(NvRmSetModuleTristate(t->hRmDevice,
        NVRM_MODULE_ID(t->ModuleId, t->Instance), NV_FALSE));
    // FIXME: Do Pinmux here.
    NvOdmQueryPinMux(OdmModule, &pOdmConfigs, &NumOdmConfigs);
    
    // Enable Clock.
    // Configure Clock.
    t->I2cPowerClientId = NVRM_POWER_CLIENT_TAG('N','V','E','C');
    NV_CHECK_ERROR_CLEANUP(NvRmPowerRegister(t->hRmDevice, NULL, 
        &t->I2cPowerClientId));
    /* 
     * Enable power rail, enable clock, configure clock to right freq,
     * reset, disable clock, notify to disable power rail.
     */
    HwI2cConfigPower(t, NV_TRUE);
    NV_CHECK_ERROR_CLEANUP(NvRmPowerModuleClockConfig(t->hRmDevice,
            NVRM_MODULE_ID(t->ModuleId, t->Instance), t->I2cPowerClientId,
            PrefClockFreq, NvRmFreqUnspecified, &PrefClockFreq, 1, NULL, 0));
    // Reset Controller.
    NvRmModuleReset(t->hRmDevice, NVRM_MODULE_ID(t->ModuleId, t->Instance));
    
    /// Install interrupt handler.
    IntHandlers = Isr;
    IrqList = NvRmGetIrqForLogicalInterrupt(t->hRmDevice, 
                NVRM_MODULE_ID(t->ModuleId, t->Instance), 0);
    NV_CHECK_ERROR_CLEANUP(NvRmInterruptRegister(t->hRmDevice, 1, &IrqList, 
        &IntHandlers, t, &t->I2CInterruptHandle, NV_TRUE));
    
    // Set the slave address and 7-bit address mode.
    I2C_REGW(t, I2C_SL_ADDR1, (t->SlaveAddress >> 1));
    I2C_REGW(t, I2C_SL_ADDR2, 0);
    // Set Delay count register
    I2C_REGW(t, I2C_SL_DELAY_COUNT, DELAY_COUNT);
    // Enable NEW_MASTER_FSM in slave for T20
    // It is found that in some corner case, it appears that the slave is
    // driving '0' on the bus and the HW team suggested to enable new master
    // even if it is not used as old master is known to go into
    // bad state
    SlaveConfig = NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
    I2C_REGW(t, I2C_CNFG, SlaveConfig);
    // Enable Ack and disable response to general call.
    SlaveConfig = ENABLE_I2C_SLAVE(t);
    //NvOsDebugPrintf("\n***SlaveConfig=0x%x", SlaveConfig);
    I2C_REGW(t, I2C_SL_CNFG, SlaveConfig);
    
    return e;
fail:
    NV_ASSERT(!"I2C slave module Init failed!");
    return e;
}

/**
 * Get the I2C  SOC capability.
 *
 */
static void
I2cGetSocCapabilities(
    NvRmDeviceHandle hDevice,
    NvU32 Instance,
    SocI2cCapability *pI2cSocCaps)
{
    SocI2cCapability s_SocI2cCapsList[2];
    NvRmModuleCapability I2cCapsList[3];
    SocI2cCapability *pI2cCaps = NULL;

    I2cCapsList[0].MajorVersion = 1;
    I2cCapsList[0].MinorVersion = 0;
    I2cCapsList[0].EcoLevel = 0;
    I2cCapsList[0].Capability = &s_SocI2cCapsList[0];

    I2cCapsList[1].MajorVersion = 1;
    I2cCapsList[1].MinorVersion = 1;
    I2cCapsList[1].EcoLevel = 0;
    I2cCapsList[1].Capability = &s_SocI2cCapsList[0];
    // AP15 does not support New Slave.
    s_SocI2cCapsList[0].IsNewSlaveAvailable = 0;

    I2cCapsList[2].MajorVersion = 1;
    I2cCapsList[2].MinorVersion = 2;
    I2cCapsList[2].EcoLevel = 0;
    I2cCapsList[2].Capability = &s_SocI2cCapsList[1];
    // AP20 supports New Slave.
    s_SocI2cCapsList[1].IsNewSlaveAvailable= ENABLE_NEW_SLAVE;

    // Get the capability from modules files.
    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hDevice,
        NVRM_MODULE_ID(NvRmModuleID_I2c, Instance), 
        I2cCapsList, 3, (void **)&pI2cCaps));
    if (pI2cCaps)
        pI2cSocCaps->IsNewSlaveAvailable = pI2cCaps->IsNewSlaveAvailable;
    else
        NV_ASSERT(!"Capabilities not found in I2cGetSocCapabilities()");
    NvOsDebugPrintf("\nI2C Slave is %s", 
        (pI2cSocCaps->IsNewSlaveAvailable ? "*New*" : "*Old*"));
}

NvError
NvEcTransportOpen(NvEcTransportHandle *phEcTrans,
    NvU32 InstanceId,
    NvOsSemaphoreHandle hEcNotifySema, 
    NvU32 Flags)
{
    NvU32 i;
    NvU32 Found = 0;
    NvU32 GpioPin = 0;
    NvU32 GpioPort = 0;
    NvError e = NvSuccess;
    NvEcTransportHandle t = &g_I2cTransport;
    NvRmGpioPinState PinState = NvRmGpioPinState_High;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    
    if (g_IsInitialized == NV_FALSE)
    {
        #if CAPTURE_TX_RX_DATA
        NvOsMemset((void*)&g_I2cSlaveRxData, 0xCD, sizeof(g_I2cSlaveRxData));
        NvOsMemset((void*)&g_I2cSlaveTxData, 0xCD, sizeof(g_I2cSlaveTxData));
        #endif
        NvOsMemset(t, 0, sizeof(NvRmI2cSlaveController));
        pConnectivity = NvOdmPeripheralGetGuid(I2C_SMBUS_TRASNSPORT_GUID);
        if (!pConnectivity)
            goto fail;
        NV_ASSERT(pConnectivity->NumAddress == 2);
        for (i = 0; i < pConnectivity->NumAddress; i++)
        {
            switch (pConnectivity->AddressList[i].Interface)
            {
                case NvOdmIoModule_I2c:
                    t->SlaveAddress = pConnectivity->AddressList[i].Address;
                    t->Instance = pConnectivity->AddressList[i].Instance;
                    Found |= 1;
                    break;
                case NvOdmIoModule_Gpio:
                    GpioPort = pConnectivity->AddressList[i].Instance;
                    GpioPin = pConnectivity->AddressList[i].Address;
                    Found |= 2;
                    break;
                default:
                    NV_ASSERT(NV_FALSE);
            }
        }
        // Check if we got everything.
        if (Found != 3)
            goto fail;
        t->hEcNotifySema = hEcNotifySema;
        NV_ASSERT_SUCCESS(NvRmOpen(&t->hRmDevice, 0));
        t->ModuleId = NvRmModuleID_I2c;
        // FIXME: Get clock speed from ODM?
        t->ClockSpeedInKHz = 100;
        I2cGetSocCapabilities(t->hRmDevice, t->Instance, &t->I2cSocCaps);
        NV_ASSERT_SUCCESS(NvRmGpioOpen(t->hRmDevice, &t->hGpio));
        NvRmGpioAcquirePinHandle(t->hGpio, GpioPort, GpioPin, &t->hEcRequestPin);
        // De-assert the Gpio Line here.
        if (t->hEcRequestPin) {
            NvRmGpioWritePins(t->hGpio, &t->hEcRequestPin, &PinState, 1);
            NV_CHECK_ERROR_CLEANUP(NvRmGpioConfigPins(t->hGpio,
                &t->hEcRequestPin, 1, NvRmGpioPinMode_Output));
        }

        NV_CHECK_ERROR(HwI2cInitController(t));
        GetBufferForRxData(t);
        g_IsInitialized = NV_TRUE;
        PRINT_I2C_MESSAGES(("\r\nNvEcTransportOpen Successful "));
    }
    else
    {
        t->hEcNotifySema = hEcNotifySema;
        //NV_ASSERT(!"Transport doesn't support multiple clients.");
    }
    *phEcTrans = t;
fail:
    
    return e;
}

void NvEcTransportClose(NvEcTransportHandle t)
{
    NvU32 SlaveConfig;
    
    // Disable Ack and disable response to general call.
    SlaveConfig = DISABLE_I2C_SLAVE(t);
    I2C_REGW(t, I2C_SL_CNFG, SlaveConfig);
    NvRmInterruptUnregister(t->hRmDevice, t->I2CInterruptHandle);
    (void)HwI2cConfigPower(t, NV_FALSE);
    NvRmPowerUnRegister(t->hRmDevice, t->I2cPowerClientId);
    NvOsIntrMutexDestroy(t->I2cThreadSafetyMutex);
    NvRmGpioReleasePinHandles(t->hGpio, &t->hEcRequestPin,1);
    NvRmGpioClose(t->hGpio);
    NvRmClose(t->hRmDevice);
    g_IsInitialized = NV_FALSE;
}

NvU32 NvEcTransportQueryStatus(NvEcTransportHandle t)
{
	NvU8 Data;
	EctPacketType PktType;
	NvU32 Status = t->Status;
	NvU8 Index = t->BufferValidIndex;

	if (t->MaxNacksSent)
	{
		Status |= NVEC_TRANSPORT_STATUS_EVENT_PACKET_MAX_NACK;
		t->MaxNacksSent = NV_FALSE;
	}

	if (t->NumPacketsRcvd) {
		NV_ASSERT(t->RxBuffers[Index].BufferInUse &&
			t->RxBuffers[Index].DataValid);
		Data = t->RxBuffers[Index].Buffer[0];
		PktType = (EctPacketType) NV_DRF_VAL(NVEC, COMMAND, PACKET_TYPE, Data);
	if (PktType == EctPacketType_Event)
		Status |= NVEC_TRANSPORT_STATUS_EVENT_RECEIVE_COMPLETE;
	else
		Status |= NVEC_TRANSPORT_STATUS_RESPONSE_RECEIVE_COMPLETE;
	}

	// Clear Send Complete Event.
	t->Status = 0;
	return Status;
}

NvError
NvEcTransportAsyncSendPacket(
    NvEcTransportHandle t,
    NvEcRequest *pRequest,
    NvU32 RequestSize)
{
    NvError e;
    NvRmGpioPinState PinState = NvRmGpioPinState_Low;
    
    NV_ASSERT(t->SendPending == NV_FALSE);
    NV_CHECK_ERROR(NvEcTransportSmbusPacketToBuffer(pRequest, RequestSize, 
        t->SendBuffer, ECT_MAX_PACKET_SIZE, &t->NumBytesToSend));
    t->SendCounter = 0;
    t->SendPending = NV_TRUE;
    PRINT_I2C_MESSAGES(("\r\nBytes2Send=%d\r\n", t->NumBytesToSend));
    // Assert the Gpio Line here.
    if (t->hEcRequestPin)
        NvRmGpioWritePins(t->hGpio, &t->hEcRequestPin, &PinState, 1);
    return e;
}

void
NvEcTransportAbortSendPacket(
    NvEcTransportHandle t,
    NvEcRequest *pRequest,
    NvU32 RequestSize)
{
    NvRmGpioPinState PinState = NvRmGpioPinState_High;
    
    NvOsIntrMutexLock(t->I2cThreadSafetyMutex);
    t->SendPending = NV_FALSE;
    // Clear Send Complete Event.
    t->Status = 0;
    // De-assert the Gpio Line here.
    if (t->hEcRequestPin)
        NvRmGpioWritePins(t->hGpio, &t->hEcRequestPin, &PinState, 1);
    NvOsIntrMutexUnlock(t->I2cThreadSafetyMutex);
}

NvError
NvEcTransportGetReceivePacket(
    NvEcTransportHandle t,
    NvEcResponse *pResponse,
    NvU32 ResponseSize)
{
    NvError e = NvSuccess;
    NvU8 Index = t->BufferValidIndex;
    NvU32 SlaveConfig;
    NvU32 ResponseSizeUsed;
    
    NV_ASSERT(t->NumPacketsRcvd);
    if (t->RxBuffers[Index].BufferInUse && t->RxBuffers[Index].DataValid)
    {
        // Transfer the data from buffer to Packet.
        NV_CHECK_ERROR_CLEANUP(NvEcTransportSmbusBufferToPacket(
            t->RxBuffers[Index].Buffer, ECT_MAX_PACKET_SIZE, pResponse, 
            ResponseSize, &ResponseSizeUsed));
    }
    else
    {
        // No Valid buffer exists. Something is fishy.
        NV_ASSERT(NV_FALSE);
    }
fail:
    NvOsIntrMutexLock(t->I2cThreadSafetyMutex);
    t->NumPacketsRcvd--;
    NvOsIntrMutexUnlock(t->I2cThreadSafetyMutex);
    // Release the buffer.
    FreeReceiveBuffer(&t->RxBuffers[Index]);
    t->BufferValidIndex++;
    if (t->BufferValidIndex == ECT_MAX_RECEIVE_BUFFERS)
        t->BufferValidIndex = 0;
    if (t->PacketBuffer == NULL)
    {
        GetBufferForRxData(t);
        // Enable ACK.
        SlaveConfig = ENABLE_I2C_SLAVE(t);
        I2C_REGW(t, I2C_SL_CNFG, SlaveConfig);
    }
    return e;
}

NvError NvEcTransportPowerSuspend(NvEcTransportHandle t)
{
    NvError err = NvSuccess;
    
    if (g_IsInitialized)
    {
        err = HwI2cConfigPower(t, NV_FALSE);
    }
    return err;
}

NvError NvEcTransportPowerResume(NvEcTransportHandle t)
{
    NvU32 SlaveConfig;
    NvError err = NvSuccess;
    
    if (g_IsInitialized)
    {
        err = HwI2cConfigPower(t, NV_TRUE);
        if (err == NvSuccess)
        {
            // Set the slave address and 7-bit address mode.
            I2C_REGW(t, I2C_SL_ADDR1, (t->SlaveAddress >> 1));
            I2C_REGW(t, I2C_SL_ADDR2, 0);
            // Enable NEW_MASTER_FSM in slave for T20
            // It is found that in some corner case, it appears that the slave is
            // driving '0' on the bus and the HW team suggested to enable new master
            // even if it is not used as old master is known to go into
            // bad state
            SlaveConfig = NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
            I2C_REGW(t, I2C_CNFG, SlaveConfig);
            // Enable Ack and disable response to general call.
            SlaveConfig = ENABLE_I2C_SLAVE(t);
            I2C_REGW(t, I2C_SL_CNFG, SlaveConfig);
        }
    }
    return err;
}

