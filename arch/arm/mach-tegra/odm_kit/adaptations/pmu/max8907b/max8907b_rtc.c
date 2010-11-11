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

static NvBool
Max8907bRtcTimeRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU32 *Count)
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
    if (Max8907bRtcI2cReadTime(hDevice, Addr, &data))
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
        if (Max8907bRtcI2cReadTime(hDevice, Addr + MAX8907B_RTC_DATE, &data))
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
            NVODMPMU_PRINTF(("\n Max8907bRtcTimeRead() error. "));
            return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcTimeRead() error. "));
        return NV_FALSE;
    }
    NVODMPMU_PRINTF(("\n *Count=0x%x ", *Count));
    return NV_TRUE;
}

NvBool
Max8907bRtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    return Max8907bRtcTimeRead(hDevice, MAX8907B_RTC_SEC, Count);
}

NvBool
Max8907bRtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    if (!Max8907bRtcTimeRead(hDevice, MAX8907B_ALARM1_SEC, Count))
	return NV_FALSE;
    return NV_TRUE;
}

/* write time and date in a BCD format */
static NvBool
Max8907bRtcTimeWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
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

    NVODMPMU_PRINTF(("\n Rtc write count=0x%x to addr=0x%x", Count, Addr));
    // convert seconds since reference time into date
    // NOTE: using linux specific convert function rtc_time_to_tm
    rtc_time_to_tm(Count, &tm);
    NVODMPMU_PRINTF(("\n rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
        "Sec=%d, *Count=0x%x ", (tm.tm_year + LINUX_RTC_BASE_YEAR),
        (tm.tm_mon + 1), tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec, Count));

    // set the day, month, year

    // convert date to bcd format
    BcdDD = DECIMAL_TO_BCD((NvU8)tm.tm_mday);
    BcdMM = DECIMAL_TO_BCD((NvU8)tm.tm_mon);
    YYYY = (NvU16)tm.tm_year + LINUX_RTC_BASE_YEAR;
    BcdYY1 = DECIMAL_TO_BCD((NvU8)(YYYY % 100));
    BcdYY2 = DECIMAL_TO_BCD((NvU8)(YYYY / 100));
    data = (NvU32)((BcdDD << 24) | (BcdMM << 16) | (BcdYY1 << 8) | BcdYY2);
    // write date - day, month, and year to RTC registers
    if (!(Max8907bRtcI2cWriteTime(hDevice, Addr + MAX8907B_RTC_DATE, data)))
    {
	NVODMPMU_PRINTF(("\n Max8907bRtcTimeWrite() error. "));
	return NV_FALSE;
    }

    // Convert time to bcd format
    BcdHours   = DECIMAL_TO_BCD(tm.tm_hour);
    BcdMinutes = DECIMAL_TO_BCD(tm.tm_min);
    BcdSeconds = DECIMAL_TO_BCD(tm.tm_sec);

    data = (BcdSeconds << 24) | (BcdMinutes << 16) | (BcdHours << 8);
    // write time - seconds, minutes and hours in a day to RTC registers
    if (!Max8907bRtcI2cWriteTime(hDevice, Addr, data))
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcTimeWrite() error. "));
        return NV_FALSE;
    }

    return NV_TRUE;
}


NvBool
Max8907bRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    return Max8907bRtcTimeWrite(hDevice, MAX8907B_RTC_SEC, Count);
}

/**
 * Set the RTC alarm.
 * @param hDevice handle to the PMU.
 * @param Count seconds in the future, treat 0 as disable.
 * @return NV_TRUE if successful, or NV_FALSE on failure.
 */
NvBool
Max8907bRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    NvBool alarm_int;

    /* disable alarms while setting */
    if (!Max8907bRtcAlarmIntEnable(hDevice, 0))
        return NV_FALSE;

    if (!Max8907bRtcTimeWrite(hDevice, MAX8907B_ALARM1_SEC, Count))
	return NV_FALSE;

    /* enable alarms if Count is non-zero. */
    if (!Max8907bRtcAlarmIntEnable(hDevice, Count != 0))
        return NV_FALSE;

#if NV_DEBUG
    NVODMPMU_PRINTF(("\n Max8907bRtcAlarmCountWrite() wrote count=0x%x. ", Count));
    if (!Max8907bRtcTimeRead(hDevice, MAX8907B_ALARM1_SEC, &Count))
	return NV_FALSE;
    NVODMPMU_PRINTF(("\n Max8907bRtcAlarmCountWrite() read back count=0x%x. ", Count));
#endif
    return NV_TRUE;
}

NvBool
Max8907bRtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data;
    if (Max8907bRtcI2cRead8(hDevice, MAX8907B_RTC_IRQ_MASK, &data))
        if ((data >> MAX8907B_RTC_IRQ_ALARM1_R_SHIFT)
	& MAX8907B_RTC_IRQ_ALARM1_R_MASK)
        {
            return NV_TRUE;
        }
    return NV_FALSE;
}

NvBool
Max8907bRtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable)
{
    NvU8 cntl_data, mask_data;
    NvU8 tmp;

    if (Enable)
    {
	/* Alarm check of HOUR, MIN, SEC, YEAR, MONTH, DATE */
	cntl_data = 0x77;
	/* mask everything except ALARM1. */
        mask_data = ~(MAX8907B_RTC_IRQ_ALARM1_R_MASK << MAX8907B_RTC_IRQ_ALARM1_R_SHIFT);
    }
    else
    {
	/* disable alarm comparisons. */
	cntl_data = 0;
	/* mask everything, including ALARM1 */
        mask_data = ~0;
    }

    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_RTC_IRQ_MASK, mask_data))
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcAlarmIntEnable() error. "));
        return NV_FALSE;
    }

    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_ALARM1_CNTL, cntl_data))
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcAlarmIntEnable() error. "));
        return NV_FALSE;
    }

    /* always force ALARM0 off */
    if (!Max8907bRtcI2cWrite8(hDevice, MAX8907B_ALARM0_CNTL, 0))
    {
        NVODMPMU_PRINTF(("\n Max8907bRtcAlarmIntEnable() error. "));
        return NV_FALSE;
    }

    return NV_TRUE;
}

NvBool
Max8907bIsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    return (!bRtcNotInitialized);
}

