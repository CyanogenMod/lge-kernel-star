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

