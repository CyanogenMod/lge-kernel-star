/*
 * Copyright (c) 2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "pcf50626_i2c.h"
#include "nvodm_pmu.h"
#include "nvodm_services.h"


NvBool Pcf50626I2cWrite8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 Data)
{  
    NvU8 WriteBuffer[2];
    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus status  = NvOdmI2cStatus_Success;    
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
        
    WriteBuffer[0] = Addr & 0xFF;   // PMU offset
    WriteBuffer[1] = Data & 0xFF;   // written data

    TransactionInfo.Address = hPmu->DeviceAddr;
    TransactionInfo.Buf = &WriteBuffer[0];
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1, 
                        PCF506226_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
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

NvBool Pcf50626I2cRead8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 *Data)
{
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;    
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    // Write the PMU offset
    ReadBuffer = Addr & 0xFF;

    TransactionInfo[0].Address = hPmu->DeviceAddr;
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;    
    TransactionInfo[1].Address = (hPmu->DeviceAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

    // Read data from PMU at the specified offset
    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2, 
                                    PCF506226_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
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

NvBool Pcf50626I2cWrite32(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU32 Data)
{
    NvU8 WriteBuffer[5];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;    
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo;

    WriteBuffer[0] = (NvU8)(Addr & 0xFF);  
    WriteBuffer[1] = (NvU8)((Data >> 24) & 0xFF);
    WriteBuffer[2] = (NvU8)((Data >> 16) & 0xFF);
    WriteBuffer[3] = (NvU8)((Data >>  8) & 0xFF);
    WriteBuffer[4] = (NvU8)(Data & 0xFF);

    TransactionInfo.Address = hPmu->DeviceAddr;
    TransactionInfo.Buf = &WriteBuffer[0];
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 5;

    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
                                        PCF506226_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status != NvOdmI2cStatus_Success)
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite32 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cWrite32 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }

    return NV_TRUE;
}

NvBool Pcf50626I2cRead32(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 *Data)
{
    NvU8 ReadBuffer[5];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;    
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];


    ReadBuffer[0] = Addr & 0xFF;

    TransactionInfo[0].Address = hPmu->DeviceAddr;
    TransactionInfo[0].Buf = &ReadBuffer[0];
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;

    TransactionInfo[1].Address = (hPmu->DeviceAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer[0];
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 4;

    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2,
                                        PCF506226_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status != NvOdmI2cStatus_Success)
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead32 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuI2cRead32 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }

    *Data = (ReadBuffer[0] << 24) | (ReadBuffer[1] << 16) | 
            (ReadBuffer[2] << 8) | ReadBuffer[3];

    return NV_TRUE;
}


