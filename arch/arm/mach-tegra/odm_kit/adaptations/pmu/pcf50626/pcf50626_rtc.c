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

