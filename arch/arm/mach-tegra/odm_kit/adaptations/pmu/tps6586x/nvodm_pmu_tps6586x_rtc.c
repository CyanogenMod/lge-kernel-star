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

#include <linux/time.h>
#include "nvodm_pmu_tps6586x_rtc.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "tps6586x_reg.h"

// macro OFFSET_BASE_YEAR if 1, uses epoch as reference year instead of 1970
// This is because RTC in PMU TPS6586x can store duration of 34 years,
// else we cannot retain date beyond 2004
#define OFFSET_BASE_YEAR 1
#if OFFSET_BASE_YEAR
static unsigned long epoch = 2009;
static unsigned long epoch_sec = 0;
#endif

static NvBool bRtcNotInitialized = NV_TRUE;

/* Read RTC count register */
NvBool
Tps6586xRtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    NvU32 ReadBuffer[2];

    // 1) The I2C address pointer must not be left pointing in the range 0xC6 to 0xCA
    // 2) The maximum time for the address pointer to be in this range is 1ms
    // 3) Always read RTC_ALARM2 in the following order to prevent the address pointer
    // from stopping at 0xC6: RTC_ALARM2_LO, then RTC_ALARM2_HI

    if (Tps6586xRtcWasStartUpFromNoPower(hDevice) && bRtcNotInitialized)
    {
        Tps6586xRtcCountWrite(hDevice, 0);
        *Count = 0;
    }
    else
    {
        // The unit of the RTC count is second!!! 1024 tick = 1s.
        // Read all 40 bit and right move 10 = Read the hightest 32bit and right move 2
        Tps6586xI2cRead32(hDevice, TPS6586x_RC6_RTC_COUNT4, &ReadBuffer[0]);

        Tps6586xI2cRead8(hDevice, TPS6586x_RCA_RTC_COUNT0, &ReadBuffer[1]);

        Tps6586xI2cRead8(hDevice, TPS6586x_RC0_RTC_CTRL, &ReadBuffer[1]);

        // return second
        *Count = ReadBuffer[0]>>2;
    }
#if OFFSET_BASE_YEAR
    // calculate epoch_sec once
    if (!epoch_sec)
        epoch_sec = mktime(epoch,1,1,0,0,0);
    *Count += epoch_sec;
#endif

    return NV_TRUE;
}

/* Write RTC count register */

NvBool
Tps6586xRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    NvU32 ReadBuffer = 0;
#if OFFSET_BASE_YEAR
    // calculate epoch_sec once
    if (!epoch_sec)
        epoch_sec = mktime(epoch,1,1,0,0,0);
    if (Count < (NvU32)epoch_sec)
    {
        // prevent setting date earlier than 'epoch'
        pr_warning("\n Date being set cannot be earlier than least "
            "year=%d. Setting as least year. ", (int)epoch);
        // base year seconds count is 0
        Count = 0;
    }
    else
        Count -= (NvU32)epoch_sec;
#endif

    // Switch to 32KHz crystal oscillator
    // POR_SRC_SEL=1 and OSC_SRC_SEL=1
    Tps6586xI2cRead8(hDevice, TPS6586x_RC0_RTC_CTRL, &ReadBuffer);
    ReadBuffer = ReadBuffer | 0xC0;
    Tps6586xI2cWrite8(hDevice, TPS6586x_RC0_RTC_CTRL, ReadBuffer);

    // To enable incrementing of the RTC_COUNT[39:0] from an initial value set by the host,
    // the RTC_ENABLE bit should be written to 1 only after the RTC_OUT voltage reaches
    // the operating range

    // Clear RTC_ENABLE before writing RTC_COUNT
    Tps6586xI2cRead8(hDevice, TPS6586x_RC0_RTC_CTRL, &ReadBuffer);
    ReadBuffer = ReadBuffer & 0xDF;
    Tps6586xI2cWrite8(hDevice, TPS6586x_RC0_RTC_CTRL, ReadBuffer);

    Tps6586xI2cWrite32(hDevice, TPS6586x_RC6_RTC_COUNT4, (Count<<2));
    Tps6586xI2cWrite8(hDevice,  TPS6586x_RCA_RTC_COUNT0, 0);

    // Set RTC_ENABLE after writing RTC_COUNT
    Tps6586xI2cRead8(hDevice, TPS6586x_RC0_RTC_CTRL, &ReadBuffer);
    ReadBuffer = ReadBuffer | 0x20;
    Tps6586xI2cWrite8(hDevice, TPS6586x_RC0_RTC_CTRL, ReadBuffer);

    if (bRtcNotInitialized)
        bRtcNotInitialized = NV_FALSE;

    return NV_TRUE;
}

/* Read RTC alarm count register */

NvBool
Tps6586xRtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    return NV_FALSE;
}

/* Write RTC alarm count register */

NvBool
Tps6586xRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    return NV_FALSE;
}

/* Reads RTC alarm interrupt mask status */

NvBool
Tps6586xRtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice)
{
    return NV_FALSE;
}

/* Enables / Disables the RTC alarm interrupt */

NvBool
Tps6586xRtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable)
{
    return NV_FALSE;
}

/* Checks if boot was from nopower / powered state */

NvBool
Tps6586xRtcWasStartUpFromNoPower(NvOdmPmuDeviceHandle hDevice)
{
    NvU32 Data = 0;

    if ((Tps6586xI2cRead8(hDevice, TPS6586x_RC0_RTC_CTRL, &Data)) == NV_TRUE)
    {
        return ((Data & 0x20)? NV_FALSE : NV_TRUE);
    }
    return NV_FALSE;
}
