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

#include "nvrm_i2c_private.h"
#include "nvassert.h"

#define NVRM_SOFT_I2C_ENABLE_PRINTF (0)

#if (NV_DEBUG && NVRM_SOFT_I2C_ENABLE_PRINTF)
#define  I2C_DUMP1(x)   NvOsDebugPrintf x
#define  I2C_DUMP(x)   NvOsDebugPrintf x
#else
#define  I2C_DUMP1(x)
#define  I2C_DUMP(x)
#endif

#define WAIT_USEC(x)   NvOsWaitUS(x)

static void I2CSetHigh( NvRmI2cController *c );
static void I2CStart( NvRmI2cController *c );
static void I2CStop( NvRmI2cController *c );

static NvU8 I2CReadByte( NvRmI2cController *c );
static NvError I2CWriteByte( NvRmI2cController *c, NvU8 data);

static NvU8 I2CReadBit( NvRmI2cController *c );
static NvError I2CWriteBit( NvRmI2cController *c, NvU8 bit);

NV_INLINE static void I2CClockHigh(NvRmI2cController *c);
NV_INLINE static void I2CClockLow(NvRmI2cController *c);
NV_INLINE static void I2CDataHigh( NvRmI2cController *c );
NV_INLINE static void I2CDataLow(NvRmI2cController *c);
NV_INLINE static void I2CWaitDataHigh(NvRmI2cController *c);
NV_INLINE static NvU8 I2CDataRead( NvRmI2cController *c );

NvError
NvRmGpioI2cRead( NvRmI2cController *c,
         NvU32          slaveAddr,
         NvU8           *pDataBytes,
         NvU32          len,
         NvU32          flags);

NvError
NvRmGpioI2cWrite( NvRmI2cController *c,
          NvU32         slaveAddr,
          NvU8          *pDataBytes,
          NvU32         len,
          NvU32         flags);

NvError NvRmGpioI2cTransaction(
        NvRmI2cController *c, 
        NvU32 I2cPinMap,
        NvU8 *Data, 
        NvU32 DataLength, 
        NvRmI2cTransactionInfo * Transaction, 
        NvU32 NumOfTransactions)
{
    NvU32 i;
    NvError status = NvSuccess;
    NvU32 clockPeriod;
    NvRmGpioPinState val = 0;

    NV_ASSERT(Transaction);
    NV_ASSERT(Data);
    NV_ASSERT((c->hSdaPin && !I2cPinMap) ||
              (!c->hSdaPin && I2cPinMap));

    /* Convert frequency to period */
    clockPeriod =  (NvU32)(1000 / c->clockfreq);
    if (clockPeriod * c->clockfreq  < 1000) 
    {
        /* This is a ciel operation */
        clockPeriod++;
    }
    c->I2cClockPeriod = clockPeriod;

    if (I2cPinMap)
    {
        NvU32 scl, sda;
        if ((c->GetGpioPins)(c, I2cPinMap, &scl, &sda))
        {
            status = NvRmGpioAcquirePinHandle(c->hGpio, (scl>>16), (scl&0xffff),
                                     &c->hSclPin);
            if(!status)
                status = NvRmGpioAcquirePinHandle(c->hGpio, (sda>>16), (sda&0xffff),
                                         &c->hSdaPin);
            if(status)
            {
                NvRmGpioReleasePinHandles(c->hGpio, &c->hSclPin, 1);
                NvRmGpioReleasePinHandles(c->hGpio, &c->hSdaPin, 1);
                c->hSclPin = 0;
                c->hSdaPin = 0;
                return status;
            }
        }
        else
            return NvError_NotSupported;
    }

    NV_ASSERT(c->hSclPin && c->hSdaPin);

    I2C_DUMP1(("Clock period = %d", clockPeriod));

    /* Load the outputs register to 0, as we always drive the pin low, if at all
     * we are driving the pin. Otherwise, we make put the pin input mode,
     * causing the pin to be tristated. */
    NvRmGpioWritePins(c->hGpio, &c->hSdaPin, &val, 1);
    NvRmGpioWritePins(c->hGpio, &c->hSclPin, &val, 1);

    NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_InputData);
    NvRmGpioConfigPins(c->hGpio, &c->hSclPin, 1, NvRmGpioPinMode_InputData);

    /* No support yet for repeat start */
    i = 0;
    while ( i < NumOfTransactions )
    {
        if ( Transaction[i].Flags & NVRM_I2C_WRITE )
        {
            status = NvRmGpioI2cWrite(c, Transaction[i].Address, 
                    Data, Transaction[i].NumBytes, Transaction[i].Flags);
        }            
        else if ( Transaction[i].Flags & NVRM_I2C_READ )
        {
            status = NvRmGpioI2cRead(c, Transaction[i].Address, Data, 
                    Transaction[i].NumBytes, Transaction[i].Flags);
        }
        Data += Transaction[i].NumBytes;
        i++;

        if (status != NvSuccess)
            break;
    }

    /* Put back the pins in function mode */
    NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_Function);
    NvRmGpioConfigPins(c->hGpio, &c->hSclPin, 1, NvRmGpioPinMode_Function);

    if (I2cPinMap)
    {
        NvRmGpioReleasePinHandles(c->hGpio, &c->hSclPin, 1);
        NvRmGpioReleasePinHandles(c->hGpio, &c->hSdaPin, 1);
        c->hSclPin = 0;
        c->hSdaPin = 0;
    }

    return status;
}

NvError
NvRmGpioI2cRead( NvRmI2cController *c,
         NvU32          slaveAddr,
         NvU8           *pDataBytes,
         NvU32          len,
         NvU32          flags)
{
    NV_ASSERT(c->hGpio);

    /* LSB is always 1 for reads */
    slaveAddr = slaveAddr | 0x1;
    I2CStart( c );

    if (I2CWriteByte( c, (NvU8)slaveAddr)  != NvSuccess)
    {
        I2C_DUMP1(("I2CReadPacket : no ACK for the slave address %x", (slaveAddr >> 1)));
        I2CStop( c );
        return NvError_I2cDeviceNotFound;
    }

    while ( len-- )
    {
        *pDataBytes++ = I2CReadByte( c );

        /* For all reads execpt the last byte, master should send the ACK. For
         * the last byte, it should send the NAK */
        if (!len)
        {
            I2CDataHigh( c );
        } else
        {
            I2CDataLow( c );
        }

        /* Pulse the clock line */
        I2CClockHigh( c );
        WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );
        I2CClockLow( c );
        WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );

        /* Release the data line */
        I2CDataHigh( c );
    }

    if (flags & NVRM_I2C_NOSTOP)
    {
        I2CSetHigh( c );
    } else
    {
        I2CStop( c );
    }

    return NvSuccess;
}

NvError
NvRmGpioI2cWrite( NvRmI2cController *c,
          NvU32         slaveAddr,
          NvU8          *pDataBytes,
          NvU32         len,
          NvU32         flags)
{
    NvError err = NvSuccess;

    NV_ASSERT(c);

    slaveAddr = slaveAddr & ~0x1;

    I2CStart( c );

    if (I2CWriteByte( c, (NvU8)slaveAddr ) != NvSuccess)
    {
        I2C_DUMP1(("I2CWrite : no ACK for the slave address %x", slaveAddr));
        err = NvError_I2cDeviceNotFound;
        goto fail;
    }

    while ( len-- )
    {
        if (I2CWriteByte( c, *pDataBytes++ ) != NvSuccess)
        {
            I2C_DUMP(("I2CWrite: no ACK for the data\r\n"));
            err = NvError_I2cDeviceNotFound;
            goto fail;
        }
    }

    if (flags & NVRM_I2C_NOSTOP)
    {
        I2CSetHigh(c);
    } else
fail: 
    {
        I2CStop( c );
    }
    return err;
}

static void
I2CSetHigh( NvRmI2cController *c )
{
    I2CWaitDataHigh( c );
    I2CClockHigh( c );
}

static void
I2CStart( NvRmI2cController *c )
{

    I2CDataLow( c );
    I2CClockLow( c );

}

static void
I2CStop( NvRmI2cController *c )
{

    I2CDataLow( c );
    I2CClockHigh( c );
    I2CDataHigh( c );

}

static NvU8
I2CReadByte( NvRmI2cController *c )
{
    int     ctr;
    NvU8   data;


    data = 0;
    for ( ctr = 0; ctr < 8; ctr++ )
    {
       data = (data << 1) | I2CReadBit( c );
    }

    return data;
}

static NvError
I2CWriteByte( NvRmI2cController *c,
              NvU8     data )
{
    NvU32 err = NvSuccess;
    NvU32       SDA = 0;
    NvU8        ctr, bit;

    for ( ctr = 0; ctr < 8; ctr++ )
    {
       bit = (data >> (7 - ctr)) & 0x01;
       (void)I2CWriteBit( c, bit );
    }
   
    /* Wait for ACK from slave i.e tristate the Data and pulse the clock and
     * check if the data line is driven low during the clock high stage.
     */
    I2CDataHigh( c );
    I2CClockHigh( c );

    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );
    
    SDA = I2CDataRead( c );
    if (SDA)
    {
        err = NvError_I2cDeviceNotFound;
    }

    I2CClockLow( c );

    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );

    return err;
}

static NvU8
I2CReadBit( NvRmI2cController *c )
{
    NvU8   SDA = 0;

    I2CDataHigh( c );    // DATA set to high first
    I2CClockHigh( c );
    
    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );

    SDA = I2CDataRead( c );

    I2CClockLow( c );

    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );

    return SDA;
}

static NvError
I2CWriteBit( NvRmI2cController *c,
             NvU8      bit )
{
    if ( bit & 0x1 )
        I2CDataHigh( c );
    else
        I2CDataLow( c );

    I2CClockHigh( c );
    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );
    I2CClockLow( c );
    WAIT_USEC( (c->I2cClockPeriod + 1) / 2 );

    return NvSuccess;
}

static void
I2CClockHigh( NvRmI2cController *c )
{
    // The scheme is to make SCL pin in tri-state, thus depends on
    // outside pull-up to generate High condition.  To be in this
    // tri-state, enable SCL pin with IN direction.  Then, always
    // clear the latched SDA and SCL bits in the register in preparation
    // for any next switching to Data Low condition (pin direction changed
    // to OUT).
    NvU32 timeout = c->timeout * 1000 / c->I2cClockPeriod ;
    NvU32   inout;

    NvRmGpioConfigPins(c->hGpio, &c->hSclPin, 1, NvRmGpioPinMode_InputData);

    // check whether slave doesn't hold SCL low
    // if so, wait for certain timeout for release by slave    
    do
    {
        WAIT_USEC( c->I2cClockPeriod );
        NvRmGpioReadPins(c->hGpio, &c->hSclPin, (NvRmGpioPinState *)&inout, 1);
        if ( inout )
        {
            return;
        }
    } while ( timeout-- );
}

NV_INLINE static void
I2CClockLow( NvRmI2cController *c )
{
    NvRmGpioConfigPins(c->hGpio, &c->hSclPin, 1, NvRmGpioPinMode_Output);
}

NV_INLINE static void
I2CDataHigh( NvRmI2cController *c )
{
    NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_InputData);
}

NV_INLINE static void
I2CDataLow( NvRmI2cController *c )
{
    NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_Output);
}

NV_INLINE static void
I2CWaitDataHigh( NvRmI2cController *c )
{
    NvU32 timeout = c->timeout * 1000 / c->I2cClockPeriod ;
    NvU32   inout;

    do
    {
        WAIT_USEC( c->I2cClockPeriod );

        NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_InputData);
        NvRmGpioReadPins(c->hGpio, &c->hSdaPin, (NvRmGpioPinState *)&inout, 1);
        if ( inout )
        {
            return;
        }
    } while ( timeout-- );
}

NV_INLINE static NvU8
I2CDataRead( NvRmI2cController *c )
{
    NvU32   data;

    NvRmGpioConfigPins(c->hGpio, &c->hSdaPin, 1, NvRmGpioPinMode_InputData);
    NvRmGpioReadPins(c->hGpio, &c->hSdaPin, (NvRmGpioPinState *)&data, 1);

    return (NvU8)(data);
}

