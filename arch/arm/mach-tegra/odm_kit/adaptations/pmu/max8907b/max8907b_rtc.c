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
#include <linux/rtc.h>
#include "max8907b.h"
#include "max8907b_rtc.h"
#include "max8907b_i2c.h"
#include "max8907b_reg.h"

/**
* The Maxim 8907B does not have an RTC that simply counts
* seconds from some time t0 (as defined by the OS API).
* Instead, this RTC contains several BCD (Binary Coded Decimal)
* registers, including: seconds, minutes, hours, days, day of
* week, date, etc...  These registers account for leap year and
* the various days of the month as well.
*
* Since the OS interpretation of seconds to a particular
* date/time from some OS-defined t0 is unknown at this level of
* the implementation, it is not possible to translate the given
* seconds into these registers (at least, not without a
* dependency on some OS-specific information).
*
*/

#define MAX8907B_SECONDS_PER_DAY    (60*60*24)
#define MAX8907B_SECONDS_PER_HOUR   (60*60)
#define MAX8907B_SECONDS_PER_MINUTE (60)

#define LINUX_RTC_BASE_YEAR 1900

/* Macro for conversion of BCD number to decimal format */
#define BCD_TO_DECIMAL(BCD) \
            ((((BCD) & 0xF0) >> 4) * 10 + ((BCD) & 0xF))
/* Macro for conversion of decimal number to BCD format */
#define DECIMAL_TO_BCD(DEC) \
            ((((DEC) / 10) << 4) | ((DEC) % 10))

static NvBool bRtcNotInitialized = NV_TRUE;

NvBool
Max8907bRtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    NvU32 data = 0;
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 Hours, Minutes, Seconds;
    NvU32 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU32 DD, MM, YY1, YY2, YYYY;
#if NV_DEBUG
    struct rtc_time tm;
#endif

    *Count = 0;
    // Read seconds, minute, hour and weekday data from RTC registers
    if (Max8907bRtcI2cReadTime(hDevice, MAX8907B_RTC_SEC, &data))
    {
        NVODMPMU_PRINTF(("\n Read time data-sec=0x%x ", data));
        // Extract seconds, minute and hour data from RTC registers read
        BcdHours   = (data >>  8) & 0xFF;
        BcdMinutes = (data >> 16) & 0xFF;
        BcdSeconds = (data >> 24) & 0xFF;

        // Convert BCD time into decimal values
        Hours   = BCD_TO_DECIMAL(BcdHours);
        Minutes = BCD_TO_DECIMAL(BcdMinutes);
        Seconds = BCD_TO_DECIMAL(BcdSeconds);

        // Read day, month, yy1 and yy2 data from RTC registers
        if (Max8907bRtcI2cReadTime(hDevice, MAX8907B_RTC_DATE, &data))
        {
            NVODMPMU_PRINTF(("\n Read time data-year=0x%x ", data));
            // Extract day, month, yy1 and yy2 data from RTC registers read
            BcdYY2   = (data & 0xFF);
            BcdYY1   = (data >>  8) & 0xFF;
            BcdMM = (data >> 16) & 0xFF;
            BcdDD = (data >> 24) & 0xFF;
            // convert bcd day/month/year data to decimal values
            YY2 = BCD_TO_DECIMAL(BcdYY2);
            YY1 = BCD_TO_DECIMAL(BcdYY1);
            YYYY = (YY2 * 100 + YY1) & 0xFFFF;
            MM = BCD_TO_DECIMAL(BcdMM);
            DD = BCD_TO_DECIMAL(BcdDD);
            // get seconds since reference time value given
            // year, month, day, hour, minutes and seconds
            // NOTE: Using linux specific API mktime for conversion
            *Count = mktime(YYYY, (MM + 1), DD, Hours, Minutes, Seconds);
            NVODMPMU_PRINTF(("\n Rtc read count=0x%x ", *Count));
            NVODMPMU_PRINTF(("\n mktime: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
                "Sec=%d, *Count=0x%x ", YYYY, (MM + 1), DD, Hours, Minutes,
                Seconds, *Count));
#if NV_DEBUG
            // Call to verify that reverse conversion of seconds matches date
            rtc_time_to_tm(*Count, &tm);
            // Check if Local_rtc_time_to_tm can return values sent to mktime
            NVODMPMU_PRINTF(("\n rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d "
                "Min=%d Sec=%d, *Count=0x%x ", (tm.tm_year +
                LINUX_RTC_BASE_YEAR), tm.tm_mon, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, *Count));
#endif
        }
        else
        {
            NVODMPMU_PRINTF(("\n Max8907bRtcCountRead() error. "));
            return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcCountRead() error. "));
        return NV_FALSE;
    }
    NVODMPMU_PRINTF(("\n *Count=0x%x ", *Count));
    return NV_TRUE;
}

NvBool
Max8907bRtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    return NV_FALSE;
}

NvBool
Max8907bRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 data = 0;
    NvU8 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU16 YYYY;
    struct rtc_time tm;
#if NV_DEBUG
    NvU32 data1;
#endif

    NVODMPMU_PRINTF(("\n Rtc write count=0x%x ", Count));
    // convert seconds since reference time into date
    // NOTE: using linux specific convert function rtc_time_to_tm
    rtc_time_to_tm(Count, &tm);
    NVODMPMU_PRINTF(("\n rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
        "Sec=%d, *Count=0x%x ", (tm.tm_year + LINUX_RTC_BASE_YEAR),
        (tm.tm_mon + 1), tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec, Count));

    // Convert time to bcd format
    BcdHours   = DECIMAL_TO_BCD(tm.tm_hour);
    BcdMinutes = DECIMAL_TO_BCD(tm.tm_min);
    BcdSeconds = DECIMAL_TO_BCD(tm.tm_sec);

    data = (BcdSeconds << 24) | (BcdMinutes << 16) | (BcdHours << 8);
    // write time - seconds, minutes and hours in a day to RTC registers
    if (Max8907bRtcI2cWriteTime(hDevice, MAX8907B_RTC_SEC, data))
    {
        // set the day, month, year
        // Assuming we get the days since 1 Jan 1970

        // convert date to bcd format
        BcdDD = DECIMAL_TO_BCD((NvU8)tm.tm_mday);
        BcdMM = DECIMAL_TO_BCD((NvU8)tm.tm_mon);
        YYYY = (NvU16)tm.tm_year + LINUX_RTC_BASE_YEAR;
        BcdYY1 = DECIMAL_TO_BCD((NvU8)(YYYY % 100));
        BcdYY2 = DECIMAL_TO_BCD((NvU8)(YYYY / 100));
        data = (NvU32)((BcdDD << 24) | (BcdMM << 16) | (BcdYY1 << 8) | BcdYY2);
        // write date - day, month, and year to RTC registers
        if (!(Max8907bRtcI2cWriteTime(hDevice, MAX8907B_RTC_DATE, data)))
        {
            NVODMPMU_PRINTF(("\n Max8907bRtcCountWrite() error. "));
            return NV_FALSE;
        }
#if NV_DEBUG
        // verify that read back values from RTC matches written values
        if (!(Max8907bRtcI2cReadTime(hDevice, MAX8907B_RTC_DATE, &data1)))
        {
            NVODMPMU_PRINTF(("\n Max8907bRtcCountRead() error. "));
            return NV_FALSE;
        }
        if (data1 == data)
        {
            NVODMPMU_PRINTF(("\n Write read Success. "));
            return NV_TRUE;
        }
        else
        {
            // return error when read data does not match written data
            NVODMPMU_PRINTF(("\n Error: write data=0x%x, rd data=0x%x. ", data, data1));
            return NV_FALSE;
        }
#endif
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcCountWrite() error. "));
        return NV_FALSE;
    }

    return NV_TRUE;
}

NvBool
Max8907bRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    return NV_FALSE;
}

NvBool
Max8907bRtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice)
{
    return NV_FALSE;
}

NvBool
Max8907bRtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable)
{
    return NV_FALSE;
}

NvBool
Max8907bIsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    return (!bRtcNotInitialized);
}

