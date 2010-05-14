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
