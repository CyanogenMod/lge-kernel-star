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

#include "pcf50626_rtc.h"
#include "pcf50626_i2c.h"
#include "pcf50626_reg.h"

/* Read RTC count register */

static NvBool bRtcNotInitialized = NV_TRUE;

NvBool 
Pcf50626RtcCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count)
{
    if (Pcf50626RtcWasStartUpFromNoPower(hDevice) && bRtcNotInitialized)
    {
        if (!Pcf50626I2cWrite32 (hDevice, PCF50626_RTC1_ADDR, 0))
        {
            return NV_FALSE;
        }
        bRtcNotInitialized = NV_FALSE;
        *Count = 0;
        return NV_TRUE;
    } else
    {   
        return ( Pcf50626I2cRead32 (hDevice, PCF50626_RTC1_ADDR, Count) );
    }   
}

/* Write RTC count register */

NvBool 
Pcf50626RtcCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count)
{
    NvBool ret;

    ret = Pcf50626I2cWrite32 (hDevice, PCF50626_RTC1_ADDR, Count);

    if (ret && bRtcNotInitialized)
        bRtcNotInitialized = NV_FALSE;
    
    return ret;
}

/* Read RTC alarm count register */

NvBool 
Pcf50626RtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32* Count)
{
    return ( Pcf50626I2cRead32 (hDevice, PCF50626_RTC1A_ADDR, Count) );
}

/* Write RTC alarm count register */

NvBool 
Pcf50626RtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 Count)
{
    return ( Pcf50626I2cWrite32 (hDevice, PCF50626_RTC1A_ADDR, Count) );
}

/* Reads RTC alarm interrupt mask status */

NvBool 
Pcf50626RtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice)
{
    NvU8    Mask;

    if(Pcf50626I2cRead8 (hDevice, PCF50626_INT1M_ADDR, &Mask))
    {
        return ((Mask & 0x8)? NV_FALSE:NV_TRUE);
    }

    return NV_FALSE;
}

/* Enables / Disables the RTC alarm interrupt */

NvBool 
Pcf50626RtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice, 
    NvBool Enable)
{
    NvBool  Status = NV_FALSE;
    NvU8    Mask;

    if ((Status = Pcf50626I2cRead8(hDevice, PCF50626_INT1M_ADDR, &Mask)) == NV_TRUE)
    {
        (Mask = Enable? (Mask & ~0x8):(Mask|0x8));
        Status = Pcf50626I2cWrite8 (hDevice, PCF50626_INT1M_ADDR, Mask);
    }

    return Status;
}

/* Checks if boot was from nopower / powered state */

NvBool 
Pcf50626RtcWasStartUpFromNoPower(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 Data;

    // Check "nopower" bit of the ERROR status register.
    // Make sure the backup battery charger is enabled (bbce and vsaveen bits). This is done by
    // the bootloader. If this is not done, the "nopower" bit will remain stuck at 0x1.
    if ((Pcf50626I2cRead8(hDevice, PCF50626_ERROR_ADDR, &Data)) == NV_TRUE)
    {
        return ((Data & 0x20)? NV_TRUE : NV_FALSE);
    }

    return NV_FALSE;
}

NvBool
Pcf50626IsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    return ((Pcf50626RtcWasStartUpFromNoPower(hDevice))? NV_FALSE : NV_TRUE);
}

