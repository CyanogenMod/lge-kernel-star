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

#include "nvodm_services.h"
#include "nvassert.h"
#include "nvodm_gpio_ext.h"
#include "gpio_pcf50626.h"

#if NV_DEBUG
#define ASSERT_SUCCESS( expr ) \
    do { \
        NvBool b = (expr); \
        NV_ASSERT( b == NV_TRUE ); \
    } while( 0 )
#else
#define ASSERT_SUCCESS( expr ) \
    do { \
        (void)(expr); \
    } while( 0 )
#endif

static NvOdmServicesI2cHandle s_hOdmI2c = NULL;

#define PCF50626_I2C_SPEED_KHZ  400
#define PCF50626_DEVICE_ADDR    0xE0
#define PCF50626_GPO2C1_ADDR    0x55
#define PCF50626_PWM1S_ADDR     0x2D
#define PCF50626_PWM1D_ADDR     0x2E

static NvBool GPIO_PCF50626_I2cWrite8(NvU8 Addr, NvU8 Data);

void
GPIO_PCF50626_NvOdmExternalGpioWritePins(
    NvU32 Port,
    NvU32 Pin,
    NvU32 PinValue)
{
    NvU8 val;
    NvBool RetVal = NV_TRUE;

    switch (Port)
    {
    case NVODM_GPIO_EXT_PORT_2:
        if (Pin != 1) // Only Pin 1 is implemented at this time
            break;

        if (PinValue) // Enable
        {
            val = (1UL << 6) // invert polarity
                | 0x3;       // pwm1 output
            RetVal = GPIO_PCF50626_I2cWrite8(PCF50626_GPO2C1_ADDR, val);
        }
        else // Disable
        {
            RetVal = GPIO_PCF50626_I2cWrite8(PCF50626_GPO2C1_ADDR, 0x0);
        }
        break;
    }

    if (RetVal == NV_FALSE)
    {
        NvOdmOsDebugPrintf("ERROR: GPIO_PCF50626_I2cWrite8() failed.\n");
    }

    return;
}

NvU32
GPIO_PCF50626_NvOdmExternalGpioReadPins(
    NvU32 Port,
    NvU32 Pin)
{
    // Implement external GPIO port read routine here.
    return 0;
}

static NvBool GPIO_PCF50626_I2cWrite8(
    NvU8 Addr,
    NvU8 Data)
{
    NvBool RetVal = NV_TRUE;
    NvU8 WriteBuffer[2];
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU32 DeviceAddr = (NvU32)PCF50626_DEVICE_ADDR;

    s_hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    if (!s_hOdmI2c)
    {
        RetVal = NV_FALSE;
        goto GPIO_PCF50626_I2cWrite8_exit;
    }

    WriteBuffer[0] = Addr & 0xFF;   // PMU offset
    WriteBuffer[1] = Data & 0xFF;   // written data

    TransactionInfo.Address = DeviceAddr;
    TransactionInfo.Buf = WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    status = NvOdmI2cTransaction(s_hOdmI2c, &TransactionInfo, 1, 
                        PCF50626_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
    if (status == NvOdmI2cStatus_Success)
        RetVal = NV_TRUE;
    else
        RetVal = NV_FALSE;

GPIO_PCF50626_I2cWrite8_exit:
    NvOdmI2cClose(s_hOdmI2c);
    s_hOdmI2c = NULL;
    return RetVal;
}
