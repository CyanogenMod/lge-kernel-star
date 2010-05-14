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

#include "nvodm_pmu.h"
#include "nvodm_services.h"
#include "ad5258_dpm.h"


static NvBool
Ad5258I2cWrite8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 Data)
{
    NvU32 i;
    NvU8 WriteBuffer[2];
    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;

    for (i = 0; i < AD5258_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = Addr & 0xFF;   // AD5258 address
        WriteBuffer[1] = Data & 0xFF;   // written data

        TransactionInfo.Address = AD5258_SLAVE_ADDR;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
            AD5258_I2C_SPEED_KHZ, AD5258_I2C_TIMEOUT_MS);
        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmDpmI2cWrite8 Failed: Timeout\n")); 
            break;
         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODMPMU_PRINTF(("NvOdmDpmI2cWrite8 Failed: SlaveNotFound\n"));
            break;             
    }
    return NV_FALSE;
}

static NvBool
Ad5258I2cRead8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 *Data)
{
    NvU32 i;
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;    
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    for (i = 0; i < AD5258_I2C_RETRY_CNT; i++)
    {
        // The AD5258 register address
        ReadBuffer = Addr & 0xFF;

        TransactionInfo[0].Address = AD5258_SLAVE_ADDR;
        TransactionInfo[0].Buf = &ReadBuffer;
        TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo[0].NumBytes = 1;

        TransactionInfo[1].Address = (AD5258_SLAVE_ADDR | 0x1);;
        TransactionInfo[1].Buf = &ReadBuffer;
        TransactionInfo[1].Flags = 0;
        TransactionInfo[1].NumBytes = 1;

        // Read data from PMU at the specified offset
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2, 
                                     AD5258_I2C_SPEED_KHZ, AD5258_I2C_TIMEOUT_MS);
        if (status == NvOdmI2cStatus_Success)
        {
            *Data = ReadBuffer;
            return NV_TRUE;
        }
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmDpmI2cRead8 Failed: Timeout\n")); 
            break;
         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODMPMU_PRINTF(("NvOdmDpmI2cRead8 Failed: SlaveNotFound\n"));
            break;             
    }
    return NV_FALSE;
}

NvBool 
Ad5258I2cSetVoltage(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 MilliVolts)
{
    static NvU32 s_LastMilliVolts = 0;

    NvU8 Data = 0;
    NvU8 Addr = AD5258_RDAC_ADDR;

    if (s_LastMilliVolts == 0)
    {
        if (!Ad5258I2cGetVoltage(hDevice, &s_LastMilliVolts))
            return NV_FALSE;
        NV_ASSERT((s_LastMilliVolts >= AD5258_V0) &&
                  (s_LastMilliVolts <= AD5258_VMAX));
    }

    // Change voltage level one maximum allowed step at a time
    while (s_LastMilliVolts != MilliVolts)
    {
        if (MilliVolts > s_LastMilliVolts + AD5258_MAX_STEP_MV)
            s_LastMilliVolts += AD5258_MAX_STEP_MV;
        else if (MilliVolts + AD5258_MAX_STEP_MV < s_LastMilliVolts)
            s_LastMilliVolts -= AD5258_MAX_STEP_MV;
        else
            s_LastMilliVolts = MilliVolts;

        // D(Vout) = (Vout - V0) * M1 / 2^b
        Data = 0;
        if (s_LastMilliVolts > AD5258_V0)
        {
            Data = (NvU8)(((s_LastMilliVolts - AD5258_V0) * AD5258_M1 +
                           (0x1 << (AD5258_b - 1))) >> AD5258_b);
            Data++; // account for load
        }
        NV_ASSERT(Data <= AD5258_RDAC_MASK);
        if (!Ad5258I2cWrite8(hDevice, Addr, Data))
            return NV_FALSE;
        NvOdmOsWaitUS(AD5258_MAX_STEP_SETTLE_TIME_US);
    }
    return NV_TRUE;
}

NvBool 
Ad5258I2cGetVoltage(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* pMilliVolts)
{
    NvU8 Data = 0;
    NvU8 Addr = AD5258_RDAC_ADDR;

    if (!Ad5258I2cRead8(hDevice, Addr, &Data))
        return NV_FALSE;

    // Vout(D) = V0 + (D * M2) / 2^b
    Data &= AD5258_RDAC_MASK;
    *pMilliVolts = AD5258_V0 +
        (((NvU32)Data * AD5258_M2 + (0x1 << (AD5258_b - 1))) >> AD5258_b);

    return NV_TRUE;
}

