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

/** @file
 * @brief <b>NVIDIA Driver Development Kit: I2C API</b>
 *
 * @b Description: Contains the NvRM I2C implementation.
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


#define I2C_PACKET_SIZE 8

/* Register access Macros */
#define I2C_REGR(c, reg) NV_REGR((c)->hRmDevice, (c)->ModuleId, (c)->Instance,  \
                    (c)->I2cRegisterOffset + I2C_##reg##_0 ); \

#define I2C_REGW(c, reg, val) NV_REGW((c)->hRmDevice, (c)->ModuleId, (c)->Instance, \
        ((c)->I2cRegisterOffset + I2C_##reg##_0), (val) );

#define DVC_REGR(c, reg)    NV_REGR((c)->hRmDevice, NvRmModuleID_Dvc, (c)->Instance, DVC_##reg##_0)
#define DVC_REGW(c, reg, val)   NV_REGW((c)->hRmDevice, NvRmModuleID_Dvc, (c)->Instance, DVC_##reg##_0, val )

static void I2cIsr(void* args)
{
    NvRmI2cController* c = args;
    NvU32 status_register;
    NvU32 FailedByte;

    // Read the status register
    status_register = I2C_REGR(c, I2C_STATUS);

    if (status_register)
    {    
        FailedByte = (NV_DRF_VAL(I2C, I2C_STATUS, CMD1_STAT, status_register) + 
                NV_DRF_VAL(I2C, I2C_STATUS, CMD2_STAT, status_register));
        if (FailedByte == 0)
        {
            NV_ASSERT(!"Something wrong with the controller, got interrupt when the controller is busy");
        }
        if (FailedByte == 1)
        {
            /* If the first byte is failed then, it means there is no ACK on the 
             * address phase.i.e there is no device with that address */
            c->I2cTransferStatus = NvError_I2cDeviceNotFound;
        }
        else 
        {
            /* It failed on some subsequent bytes, just report the transcation 
             * as failed */
            if (c->TransactionType == I2C_READ)
            {
                c->I2cTransferStatus = NvError_I2cReadFailed;
            }
            else
            {
                c->I2cTransferStatus = NvError_I2cWriteFailed;
            }
        }
        NvOsSemaphoreSignal(c->I2cSyncSemaphore);
        NvRmInterruptDone(c->I2CInterruptHandle);
        return;
    }
    
    c->I2cTransferStatus = NvSuccess;
    NvOsSemaphoreSignal(c->I2cSyncSemaphore);

    NvRmInterruptDone(c->I2CInterruptHandle);
}

static void DvcIsr(void* args)
{
    NvRmI2cController* c = args;

    //  The DVC module interrupt is not cleared until the DVC_STATUS_REG0 register
    //  is written
    DVC_REGW(c, STATUS_REG, NV_DRF_NUM(DVC, STATUS_REG, I2C_DONE_INTR, 1));
    I2cIsr(args);
}

static void 
NvRmPrivI2cOalPoll(
    NvRmI2cController *c)
{
    NvU32 busy = 1;
    NvU32 status_register = 0;
    NvU32 FailedByte;
    NvU32 count;
    NvU32 timeout = c->timeout;

    /* Assume success as a default condition */
    c->I2cTransferStatus = NvSuccess;

    do
    {
        count = 20;
        while (count)
        {
            /* Assume a best case transfer of 400KHz I2C clock and 2 byte transfer: 
             *  (i.e 1 address byte and 1 data byte )
             * It should complete in around 50 micro sec */
            NvOsWaitUS(50);
            status_register = I2C_REGR(c, I2C_STATUS);
            busy = NV_DRF_VAL(I2C, I2C_STATUS, BUSY, status_register);
            if (busy == 0)
            {
                goto done_polling;
            }
            count -= 1;
        }
        /* Above loop takes around 1 msec */
    } while (timeout-- );
    
done_polling:

    if (busy)
    {
        /* Something bad happened, controller cannot complete the transaction in
         * the time specified. */
        c->I2cTransferStatus = NvError_Timeout;
    } else 
    {
        if (c->ModuleId == NvRmModuleID_Dvc)
        {
            DVC_REGW(c, STATUS_REG, NV_DRF_NUM(DVC, STATUS_REG, I2C_DONE_INTR, 1));
        }

        /* Transfer completed, check the status */
        FailedByte = NV_DRF_VAL(I2C, I2C_STATUS, CMD1_STAT, status_register) + 
            NV_DRF_VAL(I2C, I2C_STATUS, CMD2_STAT, status_register);

        if (FailedByte != 0)
        {
            if (FailedByte == 1)
            {
                /* If the first byte is failed then, it means there is no ACK on the
                 * address phase.i.e there is no device with that address */
                c->I2cTransferStatus = NvError_I2cDeviceNotFound;
            }
            else
            {
                /* It failed on some subsequent bytes, just report the transcation 
                 * as failed */
                if (c->TransactionType == I2C_READ)
                {
                    c->I2cTransferStatus = NvError_I2cReadFailed;
                } else
                {
                    c->I2cTransferStatus = NvError_I2cWriteFailed;
                }
            }
        }    
    }
    return;
}

static NvBool AP15RmI2cGetGpioPins(
    NvRmI2cController *c,
    NvU32 I2cPinMap, 
    NvU32 *Scl,
    NvU32 *Sda)
{
    NvU32 SclPin = 0;
    NvU32 SdaPin = 0;
    NvU32 SclPort = 0;
    NvU32 SdaPort = 0;
    NvBool Result = NV_TRUE;

    NV_ASSERT((Scl != NULL) && (Sda != NULL));

    //  FIXME:  All of this should be moved over to the pin mux module,
    //  rather than the I2C module.
    if (c->ModuleId == NvRmModuleID_I2c)
    {
        switch ((c->Instance<<4) | I2cPinMap)
        {
        case ((0<<4) | 1):
            SclPin = 4;
            SdaPin = 5;
            SclPort = 'c' - 'a';
            SdaPort = 'c' - 'a';
            break;
        case ((0<<4) | 2):
            SclPin = 5;
            SdaPin = 6;
            SclPort = 'k' - 'a';
            SdaPort = 'k' - 'a';
            break;
        case ((0<<4) | 3):
            SclPin = 2;
            SdaPin = 3;
            SclPort = 'w' - 'a';
            SdaPort = 'w' - 'a';
            break;
            /*  NOTE:  The pins used in Pin Map 1 do not have a GPIO controller
             *  connected to them (VGP pins), so the software I2C implementation
             *  is not supported for this pin mux configuration */
        case ((1<<4) | 2):
            SclPin = 5;
            SdaPin = 6;
            SclPort = 't' - 'a';
            SdaPort = 't' - 'a';
            break;
        case ((1<<4) | 3):
            SclPin = 7;
            SdaPin = 1;
            SclPort = 'v' - 'a';
            SdaPort = 'w' - 'a';
            break;
        case ((1<<4) | 4):
            SclPin = 5;
            SdaPin = 4;
            SclPort = 'm' - 'a';
            SdaPort = 'm' - 'a';
            break;
        default:
            Result = NV_FALSE;
            break;
        }
    }
    else if (c->ModuleId == NvRmModuleID_Dvc && 
             c->Instance == 0 &&
             I2cPinMap == NvOdmI2cPmuPinMap_Config1)
    {
        SclPin = 6;
        SdaPin = 7;
        SclPort = 'q' - 'a';
        SdaPort = 'q' - 'a';
    }
    else
        Result = NV_FALSE;

    *Scl = SclPin | (SclPort << 16);
    *Sda = SdaPin | (SdaPort << 16);
    
    return Result;
}


static void AP15RmI2cClose(NvRmI2cController *c)
{
    if (c->I2cSyncSemaphore)
    {
        NvRmInterruptUnregister(c->hRmDevice, c->I2CInterruptHandle);
        NvOsSemaphoreDestroy(c->I2cSyncSemaphore);
        c->I2cSyncSemaphore = NULL;
        c->I2CInterruptHandle = NULL;
    }
    c->receive = 0;
    c->send = 0;
    c->repeatStart = 0;
    c->close = 0;
    c->GetGpioPins = 0;
}

static NvError 
AP15RmI2cReceive(
    NvRmI2cController* c,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvU32 val = 0;
    NvU32 ByteCount;
    NvU32 fifo[2];

    NV_ASSERT(pBuffer);
    NV_ASSERT(pTransaction->NumBytes > 0);


    // If requested i2c is dvc i2c, then disable dvc hardware from using the dvc i2c bus.
    if (c->ModuleId == NvRmModuleID_Dvc)
    {
        val = DVC_REGR(c, CTRL_REG3);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_HW_SW_PROG, SW, val);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_DONE_INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG3, val);

        val = DVC_REGR(c, CTRL_REG1);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG1, INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG1, val);
    }

    val = 0;

    if (c->EnableNewMaster)
    {
        // Enable new master if it is available
        val |= NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
    }

    /* 7 bit address */
    if (c->Is10BitAddress == NV_FALSE)
    {
        /* write the slave address */
        I2C_REGW(c, I2C_CMD_ADDR0, (pTransaction->Address | 1));

        // Configure for read trasaction
        val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, ENABLE);
        // Configure the slave address type as 7bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD,
                        SEVEN_BIT_DEVICE_ADDRESS);
    }
     /* 10 bit address */
    else
    {
        /* write the slave address */
        I2C_REGW(c, I2C_CMD_ADDR0, pTransaction->Address);

        // Configure for read trasaction
        val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, ENABLE);
        // Configure the slave address type as 10bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD,
                        TEN_BIT_DEVICE_ADDRESS);
    }

    if (c->NoACK)
    {
        val |= NV_DRF_DEF(I2C, I2C_CNFG, NOACK, ENABLE);
    }

    // Calculate the number of bytes that can be read
    ByteCount = (pTransaction->NumBytes > I2C_PACKET_SIZE) ?
                    I2C_PACKET_SIZE : pTransaction->NumBytes;

    // Initialize the I2C param structure
    c->TransactionType = I2C_READ;
    c->I2cTransferStatus = NvError_Timeout;
        
    // Configure the number of bytes to be read
    val |= NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, ByteCount - 1);
    // disable repeated start
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SLV2, DISABLE);
    I2C_REGW(c, I2C_CNFG, val);
        
    // Start the transaction
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SEND, GO);
    I2C_REGW(c, I2C_CNFG, val);

    if (!c->I2cSyncSemaphore)
    {
        NvRmPrivI2cOalPoll(c);
    } else
    {
        NvOsSemaphoreWaitTimeout(c->I2cSyncSemaphore, c->timeout);
    }

    /* Controller should return some sort of error. If not, then there is
     * something gross happened. */
    if (c->I2cTransferStatus != NvError_Timeout)
    {
        if (c->I2cTransferStatus == NvSuccess)
        {
            /* Read the FIFO */
            fifo[0] = I2C_REGR(c, I2C_CMD_DATA1);
            fifo[1] = I2C_REGR(c, I2C_CMD_DATA2);

            NvOsMemcpy(pBuffer, (NvU8* )fifo, ByteCount);
        }
        if (pBytesTransferred != NULL)
        {
            *pBytesTransferred = ByteCount;
        }
    }
    else
    {           
        if (pBytesTransferred != NULL)
        {
            *pBytesTransferred = ByteCount;
        }
        // In case of timeout, reset  the I2C controller
        NvRmModuleReset(c->hRmDevice, NVRM_MODULE_ID(c->ModuleId, c->Instance));
    }

    return c->I2cTransferStatus;
}

static NvError
AP15RmI2cRepeatStartTransaction(
    NvRmI2cController *c,
    NvU8* pBuffer,
    NvRmI2cTransactionInfo * Transactions,
    NvU32 NoOfTransations)
{
    NvU32 val = 0;
    NvU32 data = 0;
    NvU8 *pBuffer1, *pBuffer2;

    NV_ASSERT(pBuffer);
    NV_ASSERT(Transactions);
    NV_ASSERT(Transactions[0].NumBytes <= 4);
    NV_ASSERT(Transactions[1].NumBytes <= 4);

    // If requested i2c is dvc i2c, then disable dvc hardware from using the dvc i2c bus.
    if (c->ModuleId == NvRmModuleID_Dvc)
    {
        val = DVC_REGR(c, CTRL_REG3);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_HW_SW_PROG, SW, val);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_DONE_INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG3, val);

        val = DVC_REGR(c, CTRL_REG1);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG1, INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG1, val);
    }

    // There will be always only 2 transations  in normal mode 
    pBuffer1 = pBuffer;
    pBuffer2 = (NvU8 *)(( NvU32)pBuffer + Transactions[0].NumBytes);

    if (c->EnableNewMaster)
    {
        // Enable new master if it is available
        val = NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
    }


    if (Transactions[0].Flags & NVRM_I2C_WRITE)
    {
        // Configure for CMD 1 as write trasaction
        val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, DISABLE);

        // Prepare the data to be written
        NvOsMemcpy((NvU8* )&data, (void *)pBuffer1, Transactions[0].NumBytes);
        // Write the data to the controller data registers
        I2C_REGW(c, I2C_CMD_DATA1, data);
        // configure slave1 device address
        I2C_REGW(c, I2C_CMD_ADDR0, Transactions[0].Address);
    }
    else
    {
        // Configure for CMD 1 as read trasaction
        val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, ENABLE);
         // configure slave1 device address
        I2C_REGW(c, I2C_CMD_ADDR0, (Transactions[0].Address | 1));
    }

    if (Transactions[1].Flags & NVRM_I2C_WRITE)
    {
         // Configure for CMD 2 as write trasaction
         val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD2, DISABLE);

         //Prepare the data to be written
        NvOsMemcpy((NvU8* )&data, (void *)pBuffer2, Transactions[1].NumBytes);

         // Write the data to the controller data registers
        I2C_REGW(c, I2C_CMD_DATA2, data);

        /* write the slave 2 address */
        I2C_REGW(c, I2C_CMD_ADDR1, Transactions[1].Address);
    }
    else 
    {
         // Configure for CMD 2 as read trasaction
         val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD2, ENABLE);
         /* write the slave 2 address */
        I2C_REGW(c, I2C_CMD_ADDR1, (Transactions[1].Address | 1));
    }

    /* 7 bit address */
    if (Transactions[0].Is10BitAddress == NV_FALSE)
    {       
        // Configure the slave address type as 7bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD,
                        SEVEN_BIT_DEVICE_ADDRESS);
    }
     /* 10 bit address */
    else
    {
        // Configure the slave address type as 10bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD,
                        TEN_BIT_DEVICE_ADDRESS);
    }

    if (Transactions[0].Flags & NVRM_I2C_NOACK)
    {
        val |= NV_DRF_DEF(I2C, I2C_CNFG, NOACK, ENABLE);
    }


    // Initialize the I2C param structure
    c->TransactionType = I2C_REPEAT_START_TRANSACTION;
    c->I2cTransferStatus = NvError_Timeout;
        
    // Configure the number of bytes to read/write
    val |= NV_DRF_NUM(I2C, I2C_CNFG, LENGTH,
                Transactions[0].NumBytes - 1);
    // Configure the slave 2 as present
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SLV2, ENABLE);

    I2C_REGW(c, I2C_CNFG, val);
    
    // Start the transaction
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SEND, GO);
    I2C_REGW(c, I2C_CNFG, val);
    
    if (!c->I2cSyncSemaphore)
    {
        NvRmPrivI2cOalPoll(c);
    } else
    {
        NvOsSemaphoreWaitTimeout(c->I2cSyncSemaphore, c->timeout);
    }
    if (c->I2cTransferStatus != NvError_Timeout)
    {
        if (c->I2cTransferStatus == NvSuccess)
        {
            if (!(Transactions[0].Flags & NVRM_I2C_WRITE))
            {
                // read the data for the first transaction
                data = I2C_REGR(c, I2C_CMD_DATA1);

                NvOsMemcpy(pBuffer1, (NvU8* )&data, Transactions[0].NumBytes);
            }

            if (!(Transactions[1].Flags & NVRM_I2C_WRITE))
            {
                 // read the data for the second transaction
                data = I2C_REGR(c, I2C_CMD_DATA2);

                NvOsMemcpy(pBuffer2, (NvU8* )&data, Transactions[1].NumBytes);
            }
        }
    }
    else
    {        
        // In case of timeout, reset  the I2C controller
        NvRmModuleReset(c->hRmDevice, NVRM_MODULE_ID(c->ModuleId, c->Instance));
    }
    return c->I2cTransferStatus;
}

static NvError
AP15RmI2cSend(
    NvRmI2cController *c,
    NvU8*  pBuffer,
    const NvRmI2cTransactionInfo *pTransaction,
    NvU32* pBytesTransferred)
{
    NvU32 val = 0;
    NvU32 fifo[2];
    NvU32 ByteCount;

    NV_ASSERT(pBuffer);
    NV_ASSERT(pTransaction->NumBytes > 0);


    // If requested i2c is dvc i2c, then disable dvc hardware from using the dvc i2c bus.
    if (c->ModuleId == NvRmModuleID_Dvc)
    {
        val = DVC_REGR(c, CTRL_REG3);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_HW_SW_PROG, SW, val);    
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG3, I2C_DONE_INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG3, val);

        val = DVC_REGR(c, CTRL_REG1);
        val = NV_FLD_SET_DRF_DEF(DVC, CTRL_REG1, INTR_EN, ENABLE, val);
        DVC_REGW(c, CTRL_REG1, val);
    }

    val = 0;

    if (c->EnableNewMaster)
    {
        // Enable new master if it is available
        val |= NV_DRF_DEF(I2C, I2C_CNFG, NEW_MASTER_FSM, ENABLE);
    }

    // Configure the slave address
    if (c->Is10BitAddress == NV_FALSE)
    {
        /* 7 bit address */
        /* write the slave address */
        I2C_REGW(c, I2C_CMD_ADDR0, pTransaction->Address);
        // Configure for write trasaction
        val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, DISABLE);
        // Configure the slave address type as 7bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD, SEVEN_BIT_DEVICE_ADDRESS);
    }
    else
    {
        /* 10 bit address */
        
        /* write the slave address */
        I2C_REGW(c, I2C_CMD_ADDR0, pTransaction->Address);
        // Configure for write trasaction
            val |= NV_DRF_DEF(I2C, I2C_CNFG, CMD1, DISABLE);
       // Configure the slave address type as 10bit address
        val |= NV_DRF_DEF(I2C, I2C_CNFG, A_MOD,
                        TEN_BIT_DEVICE_ADDRESS);
    }

    if (c->NoACK)
    {
        val |= NV_DRF_DEF(I2C, I2C_CNFG, NOACK, ENABLE);
    }

    // Calculate the number of bytes that can be written 
    ByteCount = (pTransaction->NumBytes > I2C_PACKET_SIZE) ?
                    I2C_PACKET_SIZE : pTransaction->NumBytes;

    NvOsMemcpy((NvU8 *)fifo,(void *)pBuffer, ByteCount);

    // Initialize the I2C param structure
    c->TransactionType = I2C_WRITE;
    c->I2cTransferStatus = NvError_Timeout;

     // Write the data to the controller data registers
    I2C_REGW(c, I2C_CMD_DATA1, fifo[0]);
    I2C_REGW(c, I2C_CMD_DATA2, fifo[1]);

    // Configure the number of bytes to be written
    val |= NV_DRF_NUM(I2C, I2C_CNFG, LENGTH,
                                    ByteCount - 1);
    // disable repeated start
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SLV2, DISABLE);
    I2C_REGW(c, I2C_CNFG, val);
    
     // Start the transaction
    val |= NV_DRF_DEF(I2C, I2C_CNFG, SEND, GO);
    I2C_REGW(c, I2C_CNFG, val);

    if (!c->I2cSyncSemaphore)
    {
        // Wait for the transaction to be completed till there is timeout/max retries
        NvRmPrivI2cOalPoll(c);
    } else
    {
        // Wait for the transaction to be completed till there is timeout
        NvOsSemaphoreWaitTimeout(c->I2cSyncSemaphore, c->timeout);
    }

    if (c->I2cTransferStatus == NvSuccess 
            && pBytesTransferred != NULL)
    {
        *pBytesTransferred = ByteCount;
    }
    if (c->I2cTransferStatus == NvError_Timeout)
    {
         // In case of timeout, reset  the I2C controller
        NvRmModuleReset(c->hRmDevice, NVRM_MODULE_ID(c->ModuleId, c->Instance));
    }
    return c->I2cTransferStatus;
}


NvError AP15RmI2cOpen(NvRmI2cController *c)
{
    NvError status = NvSuccess;

    NV_ASSERT(c!= NULL);

    /* Populate the structures */
    c->receive = AP15RmI2cReceive;
    c->send = AP15RmI2cSend;
    c->repeatStart = AP15RmI2cRepeatStartTransaction;
    c->close = AP15RmI2cClose;
    c->GetGpioPins = AP15RmI2cGetGpioPins;

    c->I2cRegisterOffset = I2C_I2C_CNFG_0;
    if (c->ModuleId == NvRmModuleID_Dvc)
    {
        c->I2cRegisterOffset = DVC_I2C_CNFG_0;
    }

    // Create the sync semaphore
    status = NvOsSemaphoreCreate( &c->I2cSyncSemaphore, 0);
    
    if (status == NvSuccess)
    {
        NvU32 IrqList;
        NvOsInterruptHandler IntHandlers;

        /* Install interrupt handler */
        if (c->ModuleId == NvRmModuleID_Dvc)
        {
            IntHandlers = DvcIsr;
        } else
        {
            IntHandlers = I2cIsr;
        }
        IrqList = NvRmGetIrqForLogicalInterrupt(
                c->hRmDevice, NVRM_MODULE_ID(c->ModuleId, c->Instance), 0);
        
        status = NvRmInterruptRegister(c->hRmDevice, 1, &IrqList, &IntHandlers, 
                c, &c->I2CInterruptHandle, NV_TRUE);
        if (status != NvSuccess)
        {
            /* Fall back to Polling mode, but assert in debug build */
            NV_ASSERT(!"I2C module interrupt register failed!");
            NvOsSemaphoreDestroy(c->I2cSyncSemaphore);
            c->I2cSyncSemaphore = 0;
        }
    } 
    
    return NvSuccess;
}

