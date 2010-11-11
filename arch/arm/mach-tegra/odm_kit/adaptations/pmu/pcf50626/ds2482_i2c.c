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

#include "ds2482_i2c.h"
#include "nvodm_pmu.h"
#include "nvodm_services.h"
#include "ds2482_reg.h"

NvBool Ds2482OWI2cWrite8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 Data)
{  
    NvU8 WriteBuffer[2];
    NvOdmI2cStatus status  = NvOdmI2cStatus_Success;
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo;

    WriteBuffer[0] = Addr & 0xFF;   
    WriteBuffer[1] = Data & 0xFF;   // written data

    TransactionInfo.Address = DS2482_SLAVE_ADDR;
    TransactionInfo.Buf = &WriteBuffer[0];
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1, 
                        DS2482_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status == NvOdmI2cStatus_Success)
    {
        return NV_TRUE;
    }
    else
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuOWI2cWrite8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuOWI2cWrite8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }    
}

NvBool Ds2482OWI2cRead8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 *Data)
{
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Pcf50626PrivData *hPmu = (Pcf50626PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    ReadBuffer = Addr & 0xFF;

    TransactionInfo[0].Address = DS2482_SLAVE_ADDR;
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;    
    TransactionInfo[1].Address = (DS2482_SLAVE_ADDR | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

    status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2, 
                                    DS2482_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status != NvOdmI2cStatus_Success)
    {
        switch (status)
        {
            case NvOdmI2cStatus_Timeout:
                NVODMPMU_PRINTF(("NvOdmPmuOWI2cRead8 Failed: Timeout\n")); 
                break;
             case NvOdmI2cStatus_SlaveNotFound:
             default:
                NVODMPMU_PRINTF(("NvOdmPmuOWI2cRead8 Failed: SlaveNotFound\n"));
                break;             
        }
        return NV_FALSE;
    }  

    *Data = ReadBuffer;
    return NV_TRUE;
}

