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

#include "nvodm_pmu_tps6586x_i2c.h"
#include "nvodm_pmu_tps6586x.h"
#include "pmu_hal.h"

NvBool 
Tps6586xI2cWrite8(
    NvOdmPmuDeviceHandle hPmu,
    NvU32 Addr,
    NvU32 Data)
{  
    NvU8 WriteBuffer[2];
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    NvOdmI2cTransactionInfo TransactionInfo = {0};

    WriteBuffer[0] = Addr & 0xFF;   // PMU offset
    WriteBuffer[1] = Data & 0xFF;   // written data

    TransactionInfo.Address = ((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr;
    TransactionInfo.Buf = &WriteBuffer[0];
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    status = NvOdmI2cTransaction(((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->hOdmI2C, &TransactionInfo, 1, 
                        TPS6586x_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status == NvOdmI2cStatus_Success)
    {
        return NV_TRUE;
    }
    else
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }    
}

NvBool
Tps6586xI2cRead8(
    NvOdmPmuDeviceHandle hPmu,
    NvU32 Addr,
    NvU32 *Data)
{
    NvU8 ReadBuffer=0;
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    NvOdmI2cTransactionInfo TransactionInfo[2];

    // Write the PMU offset
    ReadBuffer = Addr & 0xFF;

    TransactionInfo[0].Address = ((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr;
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;    
    TransactionInfo[1].Address = (((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

    // Read data from PMU at the specified offset
    status = NvOdmI2cTransaction(((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->hOdmI2C, &TransactionInfo[0], 2, 
                                    TPS6586x_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status != NvOdmI2cStatus_Success)
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }  

    *Data = ReadBuffer;
    return NV_TRUE;
}

NvBool Tps6586xI2cWrite32(
    NvOdmPmuDeviceHandle hPmu,
    NvU32 Addr,
    NvU32 Data)
{
    NvU8 WriteBuffer[5];
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;
    NvOdmI2cTransactionInfo TransactionInfo = {0};
    
    WriteBuffer[0] = (NvU8)(Addr & 0xFF);  
    WriteBuffer[1] = (NvU8)((Data >> 24) & 0xFF);
    WriteBuffer[2] = (NvU8)((Data >> 16) & 0xFF);
    WriteBuffer[3] = (NvU8)((Data >>  8) & 0xFF);
    WriteBuffer[4] = (NvU8)(Data & 0xFF);

    TransactionInfo.Address = ((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr;
    TransactionInfo.Buf = &WriteBuffer[0];
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 5;

    status = NvOdmI2cTransaction(((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->hOdmI2C, &TransactionInfo, 1, 
                        TPS6586x_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status == NvOdmI2cStatus_Success)
    {
        return NV_TRUE;
    }
    else
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }
}    
    
NvBool Tps6586xI2cRead32(
    NvOdmPmuDeviceHandle hPmu,
    NvU32 Addr,
    NvU32 *Data)
{
    NvU8 ReadBuffer[5];
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    NvOdmI2cTransactionInfo TransactionInfo[2];

    // Write the PMU offset
    ReadBuffer[0] = Addr & 0xFF;

    TransactionInfo[0].Address = ((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr;
    TransactionInfo[0].Buf = &ReadBuffer[0];
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;
  
    TransactionInfo[1].Address = (((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->DeviceAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer[0];
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 4;

    // Read data from PMU at the specified offset
    status = NvOdmI2cTransaction(((NvOdmPmuDeviceTPS *)(hPmu->pPrivate))->hOdmI2C, &TransactionInfo[0], 2, 
                                    TPS6586x_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status != NvOdmI2cStatus_Success)
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }  

    *Data = (ReadBuffer[0] << 24) | (ReadBuffer[1] << 16) |
            (ReadBuffer[2] <<  8) |  ReadBuffer[3];
    return NV_TRUE;
}

