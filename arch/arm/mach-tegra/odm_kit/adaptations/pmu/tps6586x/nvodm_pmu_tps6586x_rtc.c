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
#include "nvodm_pmu_tps6586x_rtc.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "tps6586x_reg.h"

// macro OFFSET_BASE_YEAR if 1, uses epoch as reference year instead of 1970
// This is because RTC in PMU TPS6586x can store duration of 34 years,
// else we cannot retain date beyond 2004
#define OFFSET_BASE_YEAR 1
#if OFFSET_BASE_YEAR
static unsigned long epoch = 2009;
#define epoch_end            2037L
static unsigned long epoch_sec = 0;
#endif

#define RTC_LIMITED                     1

static NvBool rtc_alarm_active = NV_FALSE;      /* RTC_ALARM_ACTIVE */
static NvBool ALARM1_used = NV_FALSE;

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
#if RTC_LIMITED
         Tps6586xRtcCountWrite(hDevice, (NvU32)mktime(epoch,1,1,0,0,0));
        *Count = 0;
#else   /* RTC_LIMITED */
        Tps6586xRtcCountWrite(hDevice, 0);
        *Count = 0;
#endif  /* RTC_LIMITED */
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
    *Count += (NvU32)epoch_sec;
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
#if RTC_LIMITED
        Count = (NvU32)epoch_sec; //reset RTC count to 2010/01/01 00:00:00
#else   /* RTC_LIMITED */
        Count = 0;
#endif  /* RTC_LIMITED */
    }
    else
        Count -= (NvU32)epoch_sec;
#if RTC_LIMITED
    if (Count > (NvU32)mktime(epoch_end,12,31,23,59,59))
        Count = (NvU32)epoch_sec; //reset RTC count to 2010/01/01 00:00:00
#endif  /* RTC_LIMITED */

#endif  /*OFFSET_BASE_YEAR */

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
    NvU32 Counter;
    NvU32 ReadBuffer[3];

    if ( rtc_alarm_active ) {

        if ( ALARM1_used ) {
            Tps6586xI2cRead8(hDevice, TPS6586x_RC3_RTC_ALARM1_LO,       &ReadBuffer[2]);
            Tps6586xI2cRead8(hDevice, TPS6586x_RC2_RTC_ALARM1_MID,      &ReadBuffer[1]);
            Tps6586xI2cRead8(hDevice, TPS6586x_RC1_RTC_ALARM1_HI,       &ReadBuffer[0]);
            Counter = (ReadBuffer[0] << 16) + (ReadBuffer[1] << 8) + ReadBuffer[2];
            *Count = Counter >> 10;
        } else {        //( ALARM1_used )
            Tps6586xI2cRead8(hDevice, TPS6586x_RC5_RTC_ALARM2_LO,       &ReadBuffer[1]);
            Tps6586xI2cRead8(hDevice, TPS6586x_RC4_RTC_ALARM2_HI,       &ReadBuffer[0]);
            Counter = (ReadBuffer[0]<<8) + ReadBuffer[1];
            *Count = Counter << 2;
        }               //( ALARM1_used )

        return NV_TRUE;

    } else {    // ( rtc_alarm_active )
        return NV_FALSE;
    }           // ( rtc_alarm_active )

}

/* Write RTC alarm count register */

NvBool
Tps6586xRtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    NvU32 ReadBuffer32;
    NvU32 Counter;
    NvU32 temp;

    NvU32 alarm1_start_sec;
    NvU32 alarm1_end_sec;
    NvU32 alarm2_start_sec;
    NvU32 alarm2_end_sec;

    alarm1_start_sec = 0;       //2^(10-10)-1 = 2^0-1 = 0
    alarm1_end_sec = 16384;     //2^(24-10) = 2^14 =16384 sec = 273 min 4 sec = 4 hr 33min 4 sec

    alarm2_start_sec = 3;       //2^(12-10)-1 = 2^2-1 = 3
    alarm2_end_sec = 262144;    //2^(28-10)= 2^18 = 262144sec = 4369min 4sec = 72hr 49min 4sec = 3day 0hr 49min 4sec

    if ( Count < alarm2_end_sec && Count > alarm1_start_sec ) {
        rtc_alarm_active = NV_TRUE;
        if ( Count < alarm1_end_sec )
            ALARM1_used = NV_TRUE;
        else
            ALARM1_used = NV_FALSE;
    }

    if ( rtc_alarm_active ) {

        if ( ALARM1_used ) {

            if (Count >= alarm1_end_sec || Count <= alarm1_start_sec)
                return NV_FALSE;

                Tps6586xI2cRead32(hDevice, TPS6586x_RC6_RTC_COUNT4, &ReadBuffer32);
                Tps6586xI2cRead8(hDevice,  TPS6586x_RCA_RTC_COUNT0, &temp);
                ReadBuffer32 &= 0x0000ffff;     //bit[23:08]
                Counter = (ReadBuffer32<<8)+ temp;
                Counter += Count << 10;

                Tps6586xI2cWrite8(hDevice, TPS6586x_RC3_RTC_ALARM1_LO,  ((Counter >> 0) & 0xFF));
                Tps6586xI2cWrite8(hDevice, TPS6586x_RC2_RTC_ALARM1_MID, ((Counter >> 8) & 0xFF));
                Tps6586xI2cWrite8(hDevice, TPS6586x_RC1_RTC_ALARM1_HI,  ((Counter >> 16) & 0xFF));
        //FIXME:
            Tps6586xI2cRead8(hDevice, TPS6586x_RB4_INT_MASK5, &temp);
            temp = temp & 0xEF;
            Tps6586xI2cWrite8(hDevice, TPS6586x_RB4_INT_MASK5, temp);

        } else {        //( ALARM1_used )

            if (Count >= alarm2_end_sec || Count <= alarm2_start_sec)
                return NV_FALSE;

                Tps6586xI2cRead32(hDevice, TPS6586x_RC6_RTC_COUNT4, &ReadBuffer32);
                Tps6586xI2cRead8(hDevice,  TPS6586x_RCA_RTC_COUNT0, &temp);
                ReadBuffer32 &= 0x000ffff0;     //bit[27:12]
                Counter = ReadBuffer32 >> 4;
                Counter += Count >>2; //(Count*4sec)

                Tps6586xI2cWrite8(hDevice, TPS6586x_RC5_RTC_ALARM2_LO,  ((Counter >> 0) & 0xFF));
                Tps6586xI2cWrite8(hDevice, TPS6586x_RC4_RTC_ALARM2_HI,  ((Counter >> 8) & 0xFF));
        //FIXME:
            Tps6586xI2cRead8(hDevice, TPS6586x_RB3_INT_MASK4, &temp);
            temp = temp & 0xFD;
            Tps6586xI2cWrite8(hDevice, TPS6586x_RB3_INT_MASK4, temp);

        }               //( ALARM1_used )
        return NV_TRUE;
    } else {    // ( rtc_alarm_active )
        return NV_FALSE;
    }           // ( rtc_alarm_active )

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
