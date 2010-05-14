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
 
#ifndef INCLUDED_NVODM_TMON_ADT7461_REG_H
#define INCLUDED_NVODM_TMON_ADT7461_REG_H

#include "nvodm_tmon_adt7461.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// ODM policy: use ADT7461 extended=1 (standard=0) range
#define ADT7461_ODM_EXTENDED_RANGE (1)

// ODM policy: enable=1 (disable=0) ADT7461 standby mode
#define ADT7461_ODM_STANDBY_ENABLED (0)

// ODM policy: protect=1 (not=0) thermal limits from being overwritten by API
#define ADT7461_ODM_LOCAL_INTR_LIMITS_PROTECTED         (1)
#define ADT7461_ODM_LOCAL_COMPARATOR_LIMIT_PROTECTED    (1)

#define ADT7461_ODM_REMOTE_INTR_LIMITS_PROTECTED        (0)
#define ADT7461_ODM_REMOTE_COMPARATOR_LIMIT_PROTECTED   (1)

// ODM policy: protect=1 (not=0) sample rate from being overwritten by API
#define ADT7461_ODM_LOCAL_RATE_PROTECTED                (1)
#define ADT7461_ODM_REMOTE_RATE_PROTECTED               (0)

// ODM policy: comparator limit values for critical shutdown (in degrees C)
#define ADT7461_ODM_LOCAL_COMPARATOR_LIMIT_VALUE        (120L)
#define ADT7461_ODM_REMOTE_COMPARATOR_LIMIT_VALUE       (115L)

// ODM ADT7461 remote channel measurement offset
#define ADT7461_ODM_REMOTE_OFFSET_VALUE                 (6L)

// ODM ADT7461 interrupt polarity
#define ADT7461_ODM_INTR_POLARITY (NvOdmGpioPinMode_InputInterruptLow)

// ADT7461 Register POR settings
#define ADT7461_LOCAL_TDATA_POR                         (0x00)
#define ADT7461_REMOTE_TDATA_POR                        (0x00)
// #define ADT7461_STATUS_POR  unknown
#define ADT7461_CONFIG_POR                              (0x00)
#define ADT7461_RATE_POR                                (0x08)
#define ADT7461_LOCAL_INTR_LIMIT_HIGH_POR               (0x55)
#define ADT7461_LOCAL_INTR_LIMIT_LOW_POR                (0x00)
#define ADT7461_REMOTE_INTR_LIMIT_HIGH_POR              (0x55)
#define ADT7461_REMOTE_INTR_LIMIT_LOW_POR               (0x00)
// #define ADT7461_ONE_SHOT_POR unknown
#define ADT7461_REMOTE_TDATA_FRACTION_POR               (0x00)
#define ADT7461_REMOTE_TOFFSET_POR                      (0x00)
#define ADT7461_REMOTE_TOFFSET_FRACTION_POR             (0x00)
#define ADT7461_REMOTE_INTR_LIMIT_HIGH_FRACTION_POR     (0x00)
#define ADT7461_REMOTE_INTR_LIMIT_LOW_FRACTION_POR      (0x00)
#define ADT7461_REMOTE_COMPARATOR_LIMIT_POR             (0x55)
#define ADT7461_LOCAL_COMPARATOR_LIMIT_POR              (0x55)
#define ADT7461_COMPARATOR_HYSTERESIS_POR               (0x0A)
#define ADT7461_INTR_CNT_DELAY_POR                      (0x01)
#define ADT7461_CHIP_ID_POR                             (0x41)
#define ADT7461_CHIP_REV_POR                            (0x51)


// ADT7461 Register Addresses
#define ADT7461_LOCAL_TDATA_RD_ADDR                     (0x00)
#define ADT7461_REMOTE_TDATA_RD_ADDR                    (0x01)
                                                              
#define ADT7461_STATUS_RD_ADDR                          (0x02)
#define ADT7461_CONFIG_RD_ADDR                          (0x03)
#define ADT7461_CONFIG_WR_ADDR                          (0x09)
#define ADT7461_RATE_RD_ADDR                            (0x04)
#define ADT7461_RATE_WR_ADDR                            (0x0A)
                                                              
#define ADT7461_LOCAL_INTR_LIMIT_HIGH_RD_ADDR           (0x05)
#define ADT7461_LOCAL_INTR_LIMIT_HIGH_WR_ADDR           (0x0B)
#define ADT7461_LOCAL_INTR_LIMIT_LOW_RD_ADDR            (0x06)
#define ADT7461_LOCAL_INTR_LIMIT_LOW_WR_ADDR            (0x0C)
                                                              
#define ADT7461_REMOTE_INTR_LIMIT_HIGH_RD_ADDR          (0x07)
#define ADT7461_REMOTE_INTR_LIMIT_HIGH_WR_ADDR          (0x0D)
#define ADT7461_REMOTE_INTR_LIMIT_LOW_RD_ADDR           (0x08)
#define ADT7461_REMOTE_INTR_LIMIT_LOW_WR_ADDR           (0x0E)
                                                              
#define ADT7461_ONE_SHOT_WR_ADDR                        (0x0F)
                                                              
#define ADT7461_REMOTE_TDATA_FRACTION_RD_ADDR           (0x10)
#define ADT7461_REMOTE_TOFFSET_ADDR                     (0x11)
#define ADT7461_REMOTE_TOFFSET_FRACTION_ADDR            (0x12)
#define ADT7461_REMOTE_INTR_LIMIT_HIGH_FRACTION_ADDR    (0x13)
#define ADT7461_REMOTE_INTR_LIMIT_LOW_FRACTION_ADDR     (0x14)

#define ADT7461_REMOTE_COMPARATOR_LIMIT_ADDR            (0x19)
#define ADT7461_LOCAL_COMPARATOR_LIMIT_ADDR             (0x20)
#define ADT7461_COMPARATOR_HYSTERESIS_ADDR              (0x21)

#define ADT7461_INTR_CNT_DELAY_ADDR                     (0x22)
#define ADT7461_CHIP_ID_RD_ADDR                         (0xFE)
#define ADT7461_CHIP_REV_RD_ADDR                        (0xFF)

#define ADT7461_INVALID_ADDR                            (0xF0)


// ADT7461 conversion range (signed values)
#define ADT7461_RANGE_STANDARD_LIMIT_HIGH               (127L)
#define ADT7461_RANGE_STANDARD_LIMIT_LOW                (0L)
#define ADT7461_RANGE_EXTENDED_LIMIT_HIGH               (150L)
#define ADT7461_RANGE_EXTENDED_LIMIT_LOW                (-64L)

// ADT7461 data reading offsets (unsigned data)
#define ADT7461_RANGE_STANDARD_DATA_OFFSET              (0UL)
#define ADT7461_RANGE_EXTENDED_DATA_OFFSET              (64UL)


// ADT7461 Configuration Register bitfields
typedef enum
{
    // If set - extended temperature range (-55C to 150C); data offset 64C
    // If cleared - stnadard temperature range (0C to 127C); data offset 0 
    ADT7461ConfigBits_ExtendedRange =   (0x1 << 2),

    // If set - interrupt output works as second auto cleared comparator
    // If cleared - interrupt output works as level out of limit interrupt,
    // cleared by (a) reading status and (b) alert response protocol over I2C
    ADT7461ConfigBits_IntrAutoClear =   (0x1 << 5),

    // If set - put device in stanby mode
    // If cleared - put device in running mode
    ADT7461ConfigBits_Standby =         (0x1 << 6),

    // If set - interrupt from device is disabled
    // If cleared - interrupt from device is enabled
    ADT7461ConfigBits_IntrDisabled =    (0x1 << 7),
} ADT7461ConfigBits;

// ADT7461 initial configuration set by adaptation:
// ADT7461 THERM1 output is dedicated for critical h/w shutdown, and ADT7461
// ALERT/THERM2 output is always configured as out of limit ALERT interrupt.
// Monitor is in running mode, in the range selected per ODM policy.
#define ADT7461_INITIAL_CONFIG \
        ((ADT7461ConfigBits_IntrDisabled) | \
         (ADT7461_ODM_EXTENDED_RANGE  ? ADT7461ConfigBits_ExtendedRange : 0))


// ADT7461 sample intervals and conversion time limits rounded to the nearest
// milliseconds, in descending order indexed by rate register DATA settings

// RATE: 1/16   1/8   1/4   1/2     1     2     4     8    16    32    64 (1/s)
// DATA: 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A 
#define ADT7461_SAMPLE_INTERVALS_MS \
        16000, 8000, 4000, 2000, 1000,  500,  250,  125,   63,   31,   16
#define ADT7461_CONVERSION_TIME_MS \
          115,  115,  115,  115,  115,  115,  115,  115,   13,   13,   13

#define ADT7461_INITIAL_RATE_SETTING        (0x0A)


// ADT7461 I2C (SMBus) clock speed, bus timeout, retries, and fixed
// Alert Response Address (ARA).
#define ADT7461_I2C_SPEED_KHZ   (400)
#define ADT7461_I2C_TIMEOUT_MS  (500)
#define ADT7461_I2C_RETRY_CNT   (2)
#define ADT7461_ARA_RETRY_CNT   (4)
#define ADT7461_ARA             (0x18)

// ADT7461 power up delay (TODO: get spec for delay from vendor)
#define ADT7461_POWERUP_DELAY_MS    (5)

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_NVODM_TMON_ADT7461_REG_H

