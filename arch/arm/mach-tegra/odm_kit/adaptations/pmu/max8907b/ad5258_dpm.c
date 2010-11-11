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

