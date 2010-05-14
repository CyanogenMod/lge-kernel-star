/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
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

#include "nvodm_query_discovery.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "max8907b.h"
#include "max8907b_reg.h"
#include "max8907b_adc.h"
#include "max8907b_i2c.h"
#include "max8907b_interrupt.h"
#include "max8907b_batterycharger.h"
#include "max8907b_supply_info_table.h"
#include "fan5355_buck_reg.h"
#include "fan5355_buck_i2c.h"
#include "tca6416_expander_reg.h"
#include "tca6416_expander_i2c.h"
#include "mic2826_reg.h"
#include "mic2826_i2c.h"
#include "ad5258_dpm.h"

// Private PMU context info
Max8907bPrivData *hMax8907bPmu;

#define PMUGUID NV_ODM_GUID('m','a','x','8','9','0','7','b')

#define MAX_CHARGER_LIMIT_MA    1000

#define ALWAYS_ONLINE (1)

/**
 * MAX8907B regulators can be enabled/disabled via s/w I2C commands only
 * when MAX8907B_SEQSEL_I2CEN_LXX (7) is selected as regulator sequencer.
 * Otherwise, regulator is controlled by h/w sequencers: SEQ1 (SYSEN),
 * which is always On when PMU is On, or SEQ2 (PWREN) which is always On,
 * when system is running (it is Off in LPx mode only).
 */
#define MAX8907B_OUT_VOLTAGE_CONTROL_MASK \
    ((MAX8907B_CTL_SEQ_MASK << MAX8907B_CTL_SEQ_SHIFT) | \
     MAX8907B_OUT_VOLTAGE_ENABLE_BIT)

#define MAX8907B_OUT_VOLTAGE_CONTROL_DISABLE \
    (MAX8907B_SEQSEL_I2CEN_LXX << MAX8907B_CTL_SEQ_SHIFT)

// MAX8907B revision that requires s/w WAR to connect PWREN input to
// sequencer 2 because of the bug in the silicon.
#define MAX8907B_II2RR_PWREN_WAR (0x12)

/**
*   The FAN5355 is used to scale the voltage of an external
*   DC/DC voltage rail (for PCIE).  However, voltage scaling is
*   not required for this source, since the 1.05V default
*   voltage when enabled is OK.  On some boards, the FAN5355 may
*   not function properly, as an I2C re-work may be required
*   (otherwise, the slave address may not be found).  Therefore,
*   this feature is disabled by default.
*/
#undef MAX8907B_USE_FAN5355_VOLTAGE_SCALING

/*-- Output Voltage tables --*/

// V1, V2 (millivolts x 10)
static const NvU32 VoltageTable_SD_A[] = {
     6375,  6500,  6625,  6750,  6875,  7000,  7125,  7250,
     7375,  7500,  7625,  7750,  7875,  8000,  8125,  8250,
     8375,  8500,  8625,  8750,  8875,  9000,  9125,  9250,
     9375,  9500,  9625,  9750,  9875, 10000, 10125, 10250,
    10375, 10500, 10625, 10750, 10875, 11000, 11125, 11250,
    11375, 11500, 11625, 11750, 11875, 12000, 12125, 12250,
    12375, 12500, 12625, 12750, 12875, 13000, 13125, 13250,
    13375, 13500, 13625, 13750, 13875, 14000, 14125, 14250
};

// V3, LDO1, LDO4-LDO16, LDO19-20 (millivolts)
static const NvU32 VoltageTable_SD_B_LDO_B[] = {
     750,  800,  850,  900,  950, 1000, 1050, 1100,
    1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500,
    1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900,
    1950, 2000, 2050, 2100, 2150, 2200, 2250, 2300,
    2350, 2400, 2450, 2500, 2550, 2600, 2650, 2700,
    2750, 2800, 2850, 2900, 2950, 3000, 3050, 3100,
    3150, 3200, 3250, 3300, 3350, 3400, 3450, 3500,
    3550, 3600, 3650, 3700, 3750, 3800, 3850, 3900
};

// LDO2, LDO3, LDO17, LDO18 (millivolts)
static const NvU32 VoltageTable_LDO_A[] = {
     650,  675,  700,  725,  750,  775,  800,  825,
     850,  875,  900,  925,  950,  975, 1000, 1025,
    1050, 1075, 1100, 1125, 1150, 1175, 1200, 1225,
    1250, 1275, 1300, 1325, 1350, 1375, 1400, 1425,
    1450, 1475, 1500, 1525, 1550, 1575, 1600, 1625,
    1650, 1675, 1700, 1725, 1750, 1775, 1800, 1825,
    1850, 1875, 1900, 1925, 1950, 1975, 2000, 2025,
    2050, 2075, 2100, 2125, 2150, 2175, 2200, 2225
};

// FAN5355 VOUT_02 (millivolts x 10)
static const NvU32 VoltageTable_VOUT_02[] = {
     7500,  7625,  7750,  7875,  8000,  8125,  8250,  8375,
     8500,  8625,  8750,  8875,  9000,  9125,  9250,  9375,
     9500,  9625,  9750,  9875, 10000, 10125, 10250, 10375,
    10500, 10625, 10750, 10875, 11000, 11125, 11250, 11375,
    11500, 11625, 11750, 11875, 12000, 12125, 12250, 12375,
    12500, 12625, 12750, 12875, 13000, 13125, 13250, 13375,
    13500, 13625, 13750, 13875, 14000, 14125, 14250, 14375,
};

/*-- Sequencer table --*/

// Timer period, microseconds (us).
// Specifies the time between each sequencer event.

// Disable temporarily to keep the compiler happy.
//static const NvU32 SequencerPeriod[] = { 20, 40, 80, 160, 320, 640, 1280, 2560 };

/*-- Voltage translation functions --*/

// OutVoltageIndex is the lower six bits of the output voltage registers, VO[5:0]
static NvU32 Max8907bPmuVoltageGet_SD_A(const NvU32 OutVoltageIndex);
static NvU32 Max8907bPmuVoltageGet_SD_B_LDO_B(const NvU32 OutVoltageIndex);
static NvU32 Max8907bPmuVoltageGet_LDO_A(const NvU32 OutVoltageIndex);

static NvU32 Max8907bPmuVoltageSet_SD_A(const NvU32 OutMilliVolts);
static NvU32 Max8907bPmuVoltageSet_SD_B_LDO_B(const NvU32 OutMilliVolts);
static NvU32 Max8907bPmuVoltageSet_LDO_A(const NvU32 OutMilliVolts);

#define MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX   0x3F
#define FAN5355_MAX_OUTPUT_VOLTAGE_INDEX    0x37

static NvU32 Max8907bPmuVoltageGet_SD_A(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_SD_A[OutVoltageIndex]/10;
}

static NvU32 Max8907bPmuVoltageGet_SD_B_LDO_B(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_SD_B_LDO_B[OutVoltageIndex];
}

static NvU32 Max8907bPmuVoltageGet_LDO_A(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_LDO_A[OutVoltageIndex];
}

// Secondary PMU MIC2826 API
static NvBool MIC2826ReadVoltageReg(NvOdmPmuDeviceHandle hDevice,
                 NvU32 vddRail, NvU32* pMilliVolts);

static NvBool MIC2826WriteVoltageReg( NvOdmPmuDeviceHandle hDevice,
                 NvU32 vddRail, NvU32  MilliVolts, NvU32* pSettleMicroSeconds);

const NvU8 MIC2826_BUCK_Votage_Table[] =
{
   MIC2826_BUCK_OUT_VOLTAGE_0800,
   MIC2826_BUCK_OUT_VOLTAGE_0825,
   MIC2826_BUCK_OUT_VOLTAGE_0850,
   MIC2826_BUCK_OUT_VOLTAGE_0875,
   MIC2826_BUCK_OUT_VOLTAGE_0900,
   MIC2826_BUCK_OUT_VOLTAGE_0925,
   MIC2826_BUCK_OUT_VOLTAGE_0950,
   MIC2826_BUCK_OUT_VOLTAGE_0975,
   MIC2826_BUCK_OUT_VOLTAGE_1000,
   MIC2826_BUCK_OUT_VOLTAGE_1025,
   MIC2826_BUCK_OUT_VOLTAGE_1050,
   MIC2826_BUCK_OUT_VOLTAGE_1075,
   MIC2826_BUCK_OUT_VOLTAGE_1100,
   MIC2826_BUCK_OUT_VOLTAGE_1125,
   MIC2826_BUCK_OUT_VOLTAGE_1150,
   MIC2826_BUCK_OUT_VOLTAGE_1175,
   MIC2826_BUCK_OUT_VOLTAGE_1200,
   MIC2826_BUCK_OUT_VOLTAGE_1250,
   MIC2826_BUCK_OUT_VOLTAGE_1300,
   MIC2826_BUCK_OUT_VOLTAGE_1350,
   MIC2826_BUCK_OUT_VOLTAGE_1400,
   MIC2826_BUCK_OUT_VOLTAGE_1450,
   MIC2826_BUCK_OUT_VOLTAGE_1500,
   MIC2826_BUCK_OUT_VOLTAGE_1550,
   MIC2826_BUCK_OUT_VOLTAGE_1600,
   MIC2826_BUCK_OUT_VOLTAGE_1650,
   MIC2826_BUCK_OUT_VOLTAGE_1700,
   MIC2826_BUCK_OUT_VOLTAGE_1750,
   MIC2826_BUCK_OUT_VOLTAGE_1800
};

const NvU8 MIC2826_LDO_Votage_Table[] =
{
   MIC2826_LDO_OUT_VOLTAGE_0800,
   MIC2826_LDO_OUT_VOLTAGE_0850,
   MIC2826_LDO_OUT_VOLTAGE_0900,
   MIC2826_LDO_OUT_VOLTAGE_0950,
   MIC2826_LDO_OUT_VOLTAGE_1000,
   MIC2826_LDO_OUT_VOLTAGE_1050,
   MIC2826_LDO_OUT_VOLTAGE_1100,
   MIC2826_LDO_OUT_VOLTAGE_1150,
   MIC2826_LDO_OUT_VOLTAGE_1200,
   MIC2826_LDO_OUT_VOLTAGE_1250,
   MIC2826_LDO_OUT_VOLTAGE_1300,
   MIC2826_LDO_OUT_VOLTAGE_1350,
   MIC2826_LDO_OUT_VOLTAGE_1400,
   MIC2826_LDO_OUT_VOLTAGE_1450,
   MIC2826_LDO_OUT_VOLTAGE_1500,
   MIC2826_LDO_OUT_VOLTAGE_1550,
   MIC2826_LDO_OUT_VOLTAGE_1600,
   MIC2826_LDO_OUT_VOLTAGE_1650,
   MIC2826_LDO_OUT_VOLTAGE_1700,
   MIC2826_LDO_OUT_VOLTAGE_1750,
   MIC2826_LDO_OUT_VOLTAGE_1800,
   MIC2826_LDO_OUT_VOLTAGE_1850,
   MIC2826_LDO_OUT_VOLTAGE_1900,
   MIC2826_LDO_OUT_VOLTAGE_1950,
   MIC2826_LDO_OUT_VOLTAGE_2000,
   MIC2826_LDO_OUT_VOLTAGE_2050,
   MIC2826_LDO_OUT_VOLTAGE_2100,
   MIC2826_LDO_OUT_VOLTAGE_2150,
   MIC2826_LDO_OUT_VOLTAGE_2200,
   MIC2826_LDO_OUT_VOLTAGE_2250,
   MIC2826_LDO_OUT_VOLTAGE_2300,
   MIC2826_LDO_OUT_VOLTAGE_2350,
   MIC2826_LDO_OUT_VOLTAGE_2400,
   MIC2826_LDO_OUT_VOLTAGE_2450,
   MIC2826_LDO_OUT_VOLTAGE_2500,
   MIC2826_LDO_OUT_VOLTAGE_2550,
   MIC2826_LDO_OUT_VOLTAGE_2600,
   MIC2826_LDO_OUT_VOLTAGE_2650,
   MIC2826_LDO_OUT_VOLTAGE_2700,
   MIC2826_LDO_OUT_VOLTAGE_2750,
   MIC2826_LDO_OUT_VOLTAGE_2800,
   MIC2826_LDO_OUT_VOLTAGE_2850,
   MIC2826_LDO_OUT_VOLTAGE_2900,
   MIC2826_LDO_OUT_VOLTAGE_2950,
   MIC2826_LDO_OUT_VOLTAGE_3000,
   MIC2826_LDO_OUT_VOLTAGE_3050,
   MIC2826_LDO_OUT_VOLTAGE_3100,
   MIC2826_LDO_OUT_VOLTAGE_3150,
   MIC2826_LDO_OUT_VOLTAGE_3200,
   MIC2826_LDO_OUT_VOLTAGE_3250,
   MIC2826_LDO_OUT_VOLTAGE_3300
};

#define MIC2826_BUCK_Votage_Table_Size NV_ARRAY_SIZE(MIC2826_BUCK_Votage_Table)
#define MIC2826_LDO_Votage_Table_Size  NV_ARRAY_SIZE(MIC2826_LDO_Votage_Table)

#ifndef MIN
#define MIN(a, b)   (a) <= (b) ? (a) : (b)
#endif

#define MAX8907B_MIN_OUTPUT_VOLTAGE_SD_A_x10          6375  // 637.5 mV
#define MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B        750   // 750 mV
#define MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A             650   // 650 mV
#define FAN5355_MIN_OUTPUT_VOLTAGE_x10                7500  // 750.0 mV

#define MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10     125  // 12.5 mV
#define MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B   50   // 50 mV
#define MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A        25   // 25 mV
#define FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10           125  // 12.5 mV

#define MAX8907B_MAX_OUTPUT_VOLTAGE_SD_A_x10         14250  // 1,425.0 mV
#define MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B       3900   // 3,900 mV
#define MAX8907B_MAX_OUTPUT_VOLTAGE_LDO_A            2225   // 2,225 mV
#define FAN5355_MAX_OUTPUT_VOLTAGE_x10               14375  // 1,437.5 mV

#define MAX8907B_MIN_OUTPUT_VOLTAGE_RTC                 0   // 0 mV
#define MAX8907B_OUTPUT_VOLTAGE_INCREMENT_RTC           1   // Protected; use dummy, non-zero value
//#define MAX8907B_MAX_OUTPUT_VOLTAGE_RTC              3300   // 3,300 mV
// WHISTLER/AP16 - Make this 1.2V for now, since ap15rm_power.c expects it that way.
#define MAX8907B_MAX_OUTPUT_VOLTAGE_RTC              1200

static NvU32 Max8907bPmuVoltageSet_SD_A(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907B_MIN_OUTPUT_VOLTAGE_SD_A_x10/10)
        return 0;
    else
        return MIN(                                                 \
            (OutMilliVolts*10 - MAX8907B_MIN_OUTPUT_VOLTAGE_SD_A_x10) /    \
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10,                 \
            MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
}

static NvU32 Max8907bPmuVoltageSet_SD_B_LDO_B(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B)
        return 0;
    else
        return MIN(                                                     \
            (OutMilliVolts - MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B) /  \
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,               \
            MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
}

static NvU32 Max8907bPmuVoltageSet_LDO_A(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A)
        return 0;
    else
        return MIN(                                                 \
            (OutMilliVolts - MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A) /   \
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A,                \
            MAX8907B_MAX_OUTPUT_VOLTAGE_INDEX);
}

static NvU32 Fan5355PmuVoltageGet_VOUT_02(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= FAN5355_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_VOUT_02[OutVoltageIndex]/10;
}

static NvU32 Fan5355PmuVoltageSet_VOUT_02(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < FAN5355_MIN_OUTPUT_VOLTAGE_x10/10)
        return 0;
    else
        return MIN(                                                 \
            (OutMilliVolts*10 - FAN5355_MIN_OUTPUT_VOLTAGE_x10) /   \
            FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10,                   \
            FAN5355_MAX_OUTPUT_VOLTAGE_x10);
}

// This board-specific table is indexed by Max8907bPmuSupply
const Max8907bPmuSupplyInfo Max8907bSupplyInfoTable[] =
{
    {
        Max8907bPmuSupply_Invalid,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // LX_V1 (V1)
    {
        Max8907bPmuSupply_LX_V1,
        MAX8907B_SDCTL1,
        MAX8907B_SDSEQCNT1,
        MAX8907B_SDV1,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_A,
        Max8907bPmuVoltageSet_SD_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10/10,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907B_REQUESTVOLTAGE_LX_V1
        },
    },

    // LX_V2 (V2)
    {
        Max8907bPmuSupply_LX_V2,
        MAX8907B_SDCTL2,
        MAX8907B_SDSEQCNT2,
        MAX8907B_SDV2,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_A,
        Max8907bPmuVoltageSet_SD_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10/10,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907B_REQUESTVOLTAGE_LX_V2
        },
    },

    // LX_V3 (V3)
    {
        Max8907bPmuSupply_LX_V3,
        MAX8907B_SDCTL3,
        MAX8907B_SDSEQCNT3,
        MAX8907B_SDV3,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LX_V3
        },
    },

    // VRTC (RTC)
    {
        Max8907bPmuSupply_VRTC,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_TRUE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_RTC,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_RTC,
            MAX8907B_MAX_OUTPUT_VOLTAGE_RTC,
            MAX8907B_MAX_OUTPUT_VOLTAGE_RTC
        },
    },

    // LDO1 (VOUT1)
    {
        Max8907bPmuSupply_LDO1,
        MAX8907B_LDOCTL1,
        MAX8907B_LDOSEQCNT1,
        MAX8907B_LDO1VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO1
        },
    },

    // LDO2 (VOUT2)
    {
        Max8907bPmuSupply_LDO2,
        MAX8907B_LDOCTL2,
        MAX8907B_LDOSEQCNT2,
        MAX8907B_LDO2VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_LDO_A,
        Max8907bPmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907B_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_REQUESTVOLTAGE_LDO2
        },
    },

    // LDO3 (VOUT3)
    {
        Max8907bPmuSupply_LDO3,
        MAX8907B_LDOCTL3,
        MAX8907B_LDOSEQCNT3,
        MAX8907B_LDO3VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_LDO_A,
        Max8907bPmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907B_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_REQUESTVOLTAGE_LDO3
        },
    },

    // LDO4 (VOUT4)
    {
        Max8907bPmuSupply_LDO4,
        MAX8907B_LDOCTL4,
        MAX8907B_LDOSEQCNT4,
        MAX8907B_LDO4VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO4
        },
    },

    // LDO5 (VOUT5)
    {
        Max8907bPmuSupply_LDO5,
        MAX8907B_LDOCTL5,
        MAX8907B_LDOSEQCNT5,
        MAX8907B_LDO5VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO5
        },
    },

    // LDO6 (VOUT6)
    {
        Max8907bPmuSupply_LDO6,
        MAX8907B_LDOCTL6,
        MAX8907B_LDOSEQCNT6,
        MAX8907B_LDO6VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO6
        },
    },

    // LDO7 (VOUT7)
    {
        Max8907bPmuSupply_LDO7,
        MAX8907B_LDOCTL7,
        MAX8907B_LDOSEQCNT7,
        MAX8907B_LDO7VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO7
        },
    },

    // LDO8 (VOUT8)
    {
        Max8907bPmuSupply_LDO8,
        MAX8907B_LDOCTL8,
        MAX8907B_LDOSEQCNT8,
        MAX8907B_LDO8VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO8
        },
    },

    // LDO9 (VOUT9)
    {
        Max8907bPmuSupply_LDO9,
        MAX8907B_LDOCTL9,
        MAX8907B_LDOSEQCNT9,
        MAX8907B_LDO9VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO9
        },
    },

    // LDO10 (VOUT10)
    {
        Max8907bPmuSupply_LDO10,
        MAX8907B_LDOCTL10,
        MAX8907B_LDOSEQCNT10,
        MAX8907B_LDO10VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO10
        },
    },

    // LDO11 (VOUT11)
    {
        Max8907bPmuSupply_LDO11,
        MAX8907B_LDOCTL11,
        MAX8907B_LDOSEQCNT11,
        MAX8907B_LDO11VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO11
        },
    },

    // LDO12 (VOUT12)
    {
        Max8907bPmuSupply_LDO12,
        MAX8907B_LDOCTL12,
        MAX8907B_LDOSEQCNT12,
        MAX8907B_LDO12VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO12
        },
    },

    // LDO13 (VOUT13)
    {
        Max8907bPmuSupply_LDO13,
        MAX8907B_LDOCTL13,
        MAX8907B_LDOSEQCNT13,
        MAX8907B_LDO13VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO13
        },
    },

    // LDO14 (VOUT14)
    {
        Max8907bPmuSupply_LDO14,
        MAX8907B_LDOCTL14,
        MAX8907B_LDOSEQCNT14,
        MAX8907B_LDO14VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO14
        },
    },

    // LDO15 (VOUT15)
    {
        Max8907bPmuSupply_LDO15,
        MAX8907B_LDOCTL15,
        MAX8907B_LDOSEQCNT15,
        MAX8907B_LDO15VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO15
        },
    },

    // LDO16 (VOUT16)
    {
        Max8907bPmuSupply_LDO16,
        MAX8907B_LDOCTL16,
        MAX8907B_LDOSEQCNT16,
        MAX8907B_LDO16VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO16
        },
    },

    // LDO17 (VOUT17)
    {
        Max8907bPmuSupply_LDO17,
        MAX8907B_LDOCTL17,
        MAX8907B_LDOSEQCNT17,
        MAX8907B_LDO17VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_LDO_A,
        Max8907bPmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907B_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_REQUESTVOLTAGE_LDO17
        },
    },

    // LDO18 (VOUT18)
    {
        Max8907bPmuSupply_LDO18,
        MAX8907B_LDOCTL18,
        MAX8907B_LDOSEQCNT18,
        MAX8907B_LDO18VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_LDO_A,
        Max8907bPmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907B_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907B_REQUESTVOLTAGE_LDO18
        },
    },

    // LDO19 (VOUT19)
    {
        Max8907bPmuSupply_LDO19,
        MAX8907B_LDOCTL19,
        MAX8907B_LDOSEQCNT19,
        MAX8907B_LDO19VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO19
        },
    },

    // LDO20 (VOUT20)
    {
        Max8907bPmuSupply_LDO20,
        MAX8907B_LDOCTL20,
        MAX8907B_LDOSEQCNT20,
        MAX8907B_LDO20VOUT,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        Max8907bPmuVoltageGet_SD_B_LDO_B,
        Max8907bPmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907B_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907B_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907B_REQUESTVOLTAGE_LDO20
        },
    },

    // WHITE_LED
    {
        Max8907bPmuSupply_WHITE_LED,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC1 (for HDMI, VGA, USB)
    // By default, this is hard-wired as "always on" (see schematics)
    {
        Max8907bPmuSupply_EXT_DCDC_1,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_TRUE,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_1,
            0,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_1,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_1
        },
    },

    // EXT_DC/DC2 (not connected / reserved)
    {
        Max8907bPmuSupply_EXT_DCDC_2,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC3 (PCI Express)
    {
        Max8907bPmuSupply_EXT_DCDC_3,
        TCA6416_CONFIG_PORT_0,
        MAX8907B_REG_INVALID,
        FAN5335_VSEL0,
        TCA6416_PORT_0,
        TCA6416_PIN_6,
        Fan5355PmuVoltageGet_VOUT_02,
        Fan5355PmuVoltageSet_VOUT_02,
        {
            NV_FALSE,
            FAN5355_MIN_OUTPUT_VOLTAGE_x10/10,
            FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10/10,
            FAN5355_MAX_OUTPUT_VOLTAGE_x10/10,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },

    // EXT_DC/DC4 (Backlight-1 Intensity Enable)
    // By default, this is hard-wired as "always enabled" (see schematics)
    {
        Max8907bPmuSupply_EXT_DCDC_4,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC5 (Backlight-2 Intensity Enable)
    // By default, this is hard-wired as "always enabled" (see schematics)
    {
        Max8907bPmuSupply_EXT_DCDC_5,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC6 (not connected / reserved)
    {
        Max8907bPmuSupply_EXT_DCDC_6,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // USB1   VBUS is wired from with DCDC_3.
    {
        Max8907bPmuSupply_EXT_DCDC_3_USB1,
        TCA6416_CONFIG_PORT_0,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        TCA6416_PORT_0,
        TCA6416_PIN_0,
        NULL,
        NULL,
        {
            NV_FALSE,
            FAN5355_MIN_OUTPUT_VOLTAGE_x10/10,
            FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10/10,
            FAN5355_MAX_OUTPUT_VOLTAGE_x10/10,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },

    // USB3   VBUS is wired from with DCDC_3.
    {
        Max8907bPmuSupply_EXT_DCDC_3_USB3,
        TCA6416_CONFIG_PORT_0,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        TCA6416_PORT_0,
        TCA6416_PIN_1,
        NULL,
        NULL,
        {
            NV_FALSE,
            FAN5355_MIN_OUTPUT_VOLTAGE_x10/10,
            FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10/10,
            FAN5355_MAX_OUTPUT_VOLTAGE_x10/10,
            MAX8907B_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },

    // MIC2826 BUCK Regulator(BUCK)
    {
        MIC2826PmuSupply_BUCK,
        MIC2826_REG_ADDR_BUCK,
        MIC2826_REG_INVALID,
        MIC2826_REG_INVALID,
        MIC2826_INVALID_PORT,
        MIC2826_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            MIC2826_BUCK_VOLTAGE_MIN_MV,
            MIC2826_BUCK_VOLTAGE_STEP_MV,
            MIC2826_BUCK_VOLTAGE_MAX_MV,
            MIC2826_BUCK_REQUESTVOLTAGE_MV
        },
    },
    // MIC2826 LDO1
    {
        MIC2826PmuSupply_LDO1,
        MIC2826_REG_ADDR_LD01,
        MIC2826_REG_INVALID,
        MIC2826_REG_INVALID,
        MIC2826_INVALID_PORT,
        MIC2826_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            MIC2826_LDO_VOLTAGE_MIN_MV,
            MIC2826_LDO_VOLTAGE_STEP_MV,
            MIC2826_LDO_VOLTAGE_MAX_MV,
            MIC2826_LDO1_REQUESTVOLTAGE_MV
        },
    },

    // MIC2826 LDO2
    {
        MIC2826PmuSupply_LDO2,
        MIC2826_REG_ADDR_LD02,
        MIC2826_REG_INVALID,
        MIC2826_REG_INVALID,
        MIC2826_INVALID_PORT,
        MIC2826_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            MIC2826_LDO_VOLTAGE_MIN_MV,
            MIC2826_LDO_VOLTAGE_STEP_MV,
            MIC2826_LDO_VOLTAGE_MAX_MV,
            MIC2826_LDO2_REQUESTVOLTAGE_MV
        },
    },

    // LDO3
    {
        MIC2826PmuSupply_LDO3,
        MIC2826_REG_ADDR_LD03,
        MIC2826_REG_INVALID,
        MIC2826_REG_INVALID,
        MIC2826_INVALID_PORT,
        MIC2826_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            MIC2826_LDO_VOLTAGE_MIN_MV,
            MIC2826_LDO_VOLTAGE_STEP_MV,
            MIC2826_LDO_VOLTAGE_MAX_MV,
            MIC2826_LDO3_REQUESTVOLTAGE_MV
        },
    },

    // EXT_DC/DC7 (controlled by LX_V1, scaled by AD5258 DPM)
    {
        Max8907bLxV1_Ad5258_DPM_EXT_DCDC_7,
        AD5258_RDAC_ADDR,
        MAX8907B_REG_INVALID,
        MAX8907B_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            AD5258_V0,
            AD5258_MIN_STEP_MV,
            AD5258_VMAX,
            MAX8907B_REQUESTVOLTAGE_LX_V1
        },
    }
};

static NvBool
Max8907bReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 milliVolts = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if (pSupplyInfo->ControlRegAddr != MAX8907B_REG_INVALID)
    {
        if (!Max8907bI2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data))
            return NV_FALSE;

        if ((data & MAX8907B_OUT_VOLTAGE_CONTROL_MASK) ==
            MAX8907B_OUT_VOLTAGE_CONTROL_DISABLE)
        {
            ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail] =
                ODM_VOLTAGE_OFF;
            *pMilliVolts = ODM_VOLTAGE_OFF;
            return NV_TRUE;
        }
    }

    if (pSupplyInfo->OutputVoltageRegAddr == MAX8907B_REG_INVALID)
        return NV_FALSE;

    if (!Max8907bI2cRead8(hDevice, pSupplyInfo->OutputVoltageRegAddr, &data))
        return NV_FALSE;

    data &= MAX8907B_OUT_VOLTAGE_MASK;
    if (!data) //OFF
        milliVolts = ODM_VOLTAGE_OFF;
    else
        milliVolts = pSupplyInfo->GetVoltage(data);

    ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail] = milliVolts;
    *pMilliVolts = milliVolts;
    return NV_TRUE;
}

static NvBool
Max8907bWriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32  MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 SettleUS = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1)
        {
            // turn off the supply
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

            // Disable the output (read-modify-write the control register)
            Max8907bI2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data);
            data &= (~MAX8907B_OUT_VOLTAGE_CONTROL_MASK);
            data |= MAX8907B_OUT_VOLTAGE_CONTROL_DISABLE;
            if (!Max8907bI2cWrite8(hDevice, pSupplyInfo->ControlRegAddr, data))
                return NV_FALSE;

            ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail] =
                ODM_VOLTAGE_OFF;
            SettleUS = MAX8907B_TURN_OFF_TIME_US;
        }

        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] != 0)
            ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] --;

        if (pSettleMicroSeconds)
            *pSettleMicroSeconds = SettleUS;
        else
            NvOdmOsWaitUS(SettleUS);

        return NV_TRUE;
    }

    // Set voltage level
    data = pSupplyInfo->SetVoltage(MilliVolts);
    if (!Max8907bI2cWrite8(hDevice, pSupplyInfo->OutputVoltageRegAddr, data))
        return NV_FALSE;
    if (MilliVolts >
        ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail])
    {
        NvU32 LastMV =
            ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail];
        SettleUS = (MilliVolts - LastMV) * 1000 / MAX8907B_SCALE_UP_UV_PER_US;
    }
    ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[vddRail] = MilliVolts;

    // turn on supply
    if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)
    {
        // Enable the output (read-modify-write the control register)
        Max8907bI2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data);

        if ((data & MAX8907B_OUT_VOLTAGE_CONTROL_MASK) ==
            MAX8907B_OUT_VOLTAGE_CONTROL_DISABLE)
        {
            // Voltage on/change (supply was off, so it must be turned on)
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907bPrivData*)(hDevice->pPrivate))->hOdmPmuSevice,
                pSupplyInfo->supply, NV_TRUE);
            data |= MAX8907B_OUT_VOLTAGE_ENABLE_BIT;
            if (!Max8907bI2cWrite8(hDevice, pSupplyInfo->ControlRegAddr, data))
                return NV_FALSE;

            SettleUS = MAX8907B_TURN_ON_TIME_US;
        }
    }

    ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] ++;

    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = SettleUS;
    else
        NvOdmOsWaitUS(SettleUS);

    return NV_TRUE;
}

static NvBool
Max8907bOnOffConfigure(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data = 0;

    if (!Max8907bI2cRead8(hDevice, MAX8907B_SYSENSEL, &data))
        return NV_FALSE;

    // Enable hard reset - power-off after ONKEY press for 5 seconds
    // (must be enabled for thermal auto-shutdown)
    data |= (MAX8907B_SYSENSEL_HRDSTEN_MASK <<
             MAX8907B_SYSENSEL_HRDSTEN_SHIFT);

    return Max8907bI2cWrite8(hDevice, MAX8907B_SYSENSEL, data);
}

static NvBool
Max8907bPwrEnConfigure(NvOdmPmuDeviceHandle hDevice, NvBool Enable)
{
    NvU8 data = 0;

    if (!Max8907bI2cRead8(hDevice, MAX8907B_RESET_CNFG, &data))
        return NV_FALSE;

    // Enable/disable PWREN h/w control mechanism (PWREN signal must be
    // inactive = high at this time)
    if (Enable)
        data |= (MAX8907B_RESET_CNFG_PWREN_EN_MASK <<
                 MAX8907B_RESET_CNFG_PWREN_EN_SHIFT);
    else
        data &= (~(MAX8907B_RESET_CNFG_PWREN_EN_MASK <<
                   MAX8907B_RESET_CNFG_PWREN_EN_SHIFT));
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_RESET_CNFG, data))
        return NV_FALSE;

    // When enabled, connect PWREN to SEQ2 by clearing SEQ2 configuration
    // settings for silicon revision that requires s/w WAR. On other MAX8907B
    // revisions PWREN is always connected to SEQ2.
    if (Enable)
    {
        if (!Max8907bI2cRead8(hDevice, MAX8907B_II2RR, &data))
            return NV_FALSE;

        if (data == MAX8907B_II2RR_PWREN_WAR)
        {
            data = 0x00;
            if (!Max8907bI2cWrite8(hDevice, MAX8907B_SEQ2CNFG, data))
                return NV_FALSE;
        }
    }
    return NV_TRUE;
}

static NvBool
Max8907bPwrEnAttach(
    NvOdmPmuDeviceHandle hDevice,
    Max8907bPmuSupply Supply,
    NvBool Attach)
{
    NvU8 CtlAddr, CtlData, CntAddr, CntData, SeqSel;

    switch (Supply)
    {
        case Max8907bPmuSupply_LX_V1:   // CPU
            // No sequencer delay for CPU rail when it is attached
            CntData = Attach ?  0x00 : MAX8907B_SEQCNT_DEFAULT_LX_V1;
            SeqSel = Attach ? MAX8907B_SEQSEL_PWREN_LXX :
                              MAX8907B_SEQSEL_DEFAULT_LX_V1;
            break;

        case Max8907bPmuSupply_LX_V2:   // Core
            // Change CPU sequencer delay when core is attached to assure
            // order of Core/CPU rails control; clear CPU delay when core
            // is detached
            CntAddr = Max8907bSupplyInfoTable[
                Max8907bPmuSupply_LX_V1].SequencerCountRegAddr;
            CntData = Attach ?  MAX8907B_SEQCNT_PWREN_LX_V1 : 0x00;
            if (!Max8907bI2cWrite8(hDevice, CntAddr, CntData))
                return NV_FALSE;

            CntData = Attach ?  MAX8907B_SEQCNT_PWREN_LX_V2 :
                                MAX8907B_SEQCNT_DEFAULT_LX_V2;
            SeqSel = Attach ? MAX8907B_SEQSEL_PWREN_LXX :
                              MAX8907B_SEQSEL_DEFAULT_LX_V2;
            break;

        default:
            NV_ASSERT(!"This supply must not be attached to PWREN");
            return NV_FALSE;
    }
    CtlAddr = Max8907bSupplyInfoTable[Supply].ControlRegAddr;
    CntAddr = Max8907bSupplyInfoTable[Supply].SequencerCountRegAddr;

    // Read control refgister, and select target sequencer
    if (!Max8907bI2cRead8(hDevice, CtlAddr, &CtlData))
        return NV_FALSE;
    CtlData &= (~(MAX8907B_CTL_SEQ_MASK << MAX8907B_CTL_SEQ_SHIFT ));
    CtlData |= ((SeqSel & MAX8907B_CTL_SEQ_MASK) << MAX8907B_CTL_SEQ_SHIFT );

    // Attach: set count => set control
    // Dettach: reset control => reset count
    if (Attach)
    {
        if (!Max8907bI2cWrite8(hDevice, CntAddr, CntData))
            return NV_FALSE;
    }

    if (!Max8907bI2cWrite8(hDevice, CtlAddr, CtlData))
        return NV_FALSE;

    if (!Attach)
    {
        if (!Max8907bI2cWrite8(hDevice, CntAddr, CntData))
            return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool
Tca6416ConfigPort(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvBool Enable)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];
    NvU32 PortNo;
    NvU32 PinNo;

    // Get port number and pin number
    PortNo = pSupplyInfo->OutputPort;
    PinNo = pSupplyInfo->PmuGpio;

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if (Enable)
    {
        // Configure GPIO as output
        if (!Tca6416ConfigPortPin(hDevice, PortNo, PinNo, GpioPinMode_Output))
            return NV_FALSE;

        // Set the output port
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_High))
            return NV_FALSE;
    }
    else
    // check if the supply can be turned off
    {
        // turn off the supply
        NvOdmServicesPmuSetSocRailPowerState(
            ((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

        // Configure port pin as output
        if (!Tca6416ConfigPortPin(hDevice, PortNo, PinNo, GpioPinMode_Output))
            return NV_FALSE;

        // Set the output port (for disable, data = 0)
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_Low))
            return NV_FALSE;
    }
    return NV_TRUE;
}


#if defined(MAX8907B_USE_FAN5355_VOLTAGE_SCALING)
static NvBool
Fan5355ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 milliVolts = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if (pSupplyInfo->OutputVoltageRegAddr == FAN5335_REG_INVALID)
        return NV_FALSE;

    if (!Fan5355I2cRead8(hDevice, pSupplyInfo->OutputVoltageRegAddr, &data))
        return NV_FALSE;

    if (!data) //OFF
        milliVolts = 0;
    else
        milliVolts = pSupplyInfo->GetVoltage(data);

    *pMilliVolts = milliVolts;
    return NV_TRUE;
}
#endif

static NvBool
Fan5355WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32  MilliVolts)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = NULL;
    NvU8 data = 0;

    if ((vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB1) ||
            (vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB3))
    {
        vddRail = Max8907bPmuSupply_EXT_DCDC_3;
    }

    pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    // TO DO: Account for reference counting
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        {
            // turn off the supply
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

            // Disable the output
            if (!Tca6416ConfigPort(hDevice, vddRail, NV_FALSE))
                return NV_FALSE;
        }
    }
    else
    {
        // Voltage on/change
        NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907bPrivData*)(hDevice->pPrivate))->hOdmPmuSevice, pSupplyInfo->supply, NV_TRUE);

        // Set voltage level
        data = pSupplyInfo->SetVoltage(MilliVolts);
#if defined(MAX8907B_USE_FAN5355_VOLTAGE_SCALING)
        if (!Fan5355I2cWrite8(hDevice, pSupplyInfo->OutputVoltageRegAddr, data))
            return NV_FALSE;
#endif

        // Enable the output
        if (!Tca6416ConfigPort(hDevice, vddRail, NV_TRUE))
            return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool
Max8907bLxV1Ad5258ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907bPmuSupplyInfo* pSupplyInfo =
        &Max8907bSupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    // Check if DC/DC has been turned Off (controlled by LxV1 main PMU output)
    if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] == 0)
    {
        if (!Max8907bReadVoltageReg(
            hDevice, Max8907bPmuSupply_LX_V1, pMilliVolts))
            return NV_FALSE;
        if (*pMilliVolts == ODM_VOLTAGE_OFF)
            return NV_TRUE;
    }

    // DC/DC is On - now get DPM-scaled voltage
    return Ad5258I2cGetVoltage(hDevice, pMilliVolts);
}

static NvBool
Max8907bLxV1Ad5258WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo =
        &Max8907bSupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // Check if the supply can be turned off, and if yes - turn off
        // LxV1 main PMU output, which controls DC/DC
        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] == 1)
        {
            if (!Max8907bWriteVoltageReg(hDevice, Max8907bPmuSupply_LX_V1,
                                         MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] != 0)
        {
            ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] --;
        }
        return NV_TRUE;
    }

    // Set DPM voltage level (includes DPM and DCDC change level settle time)
    if (!Ad5258I2cSetVoltage(hDevice, MilliVolts))
        return NV_FALSE;

    // Turn on control LxV1 supply on main PMU if necessary
    if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] == 0)
    {
        if (!Max8907bWriteVoltageReg(hDevice, Max8907bPmuSupply_LX_V1,
                MAX8907B_REQUESTVOLTAGE_LX_V1, pSettleMicroSeconds))
            return NV_FALSE;

        // Add external DCDC turning On settling time
        if (pSettleMicroSeconds)
            *pSettleMicroSeconds += AD5258_TURN_ON_TIME_US;
        else
            NvOdmOsWaitUS(AD5258_TURN_ON_TIME_US);
    }
    ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] ++;

    return NV_TRUE;
}

NvBool
Max8907bSetup(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmIoModule I2cModule = NvOdmIoModule_I2c;
    NvU32  I2cInstance = 0;
    NvU32  I2cAddress  = 0;
    NvU32  i           = 0;
    const NvOdmPeripheralConnectivity *pConnectivity =
                           NvOdmPeripheralGetGuid(PMUGUID);

    NV_ASSERT(hDevice);

    hMax8907bPmu = (Max8907bPrivData*) NvOdmOsAlloc(sizeof(Max8907bPrivData));
    if (hMax8907bPmu == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating Max8907bPrivData.\n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hMax8907bPmu, 0, sizeof(Max8907bPrivData));
    hDevice->pPrivate = hMax8907bPmu;

    ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable =
        NvOdmOsAlloc(sizeof(NvU32) * Max8907bPmuSupply_Num);
    if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating RefCntTable. \n"));
        goto fail;
    }
    ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages =
        NvOdmOsAlloc(sizeof(NvU32) * Max8907bPmuSupply_Num);
    if (((Max8907bPrivData*)hDevice->pPrivate)->pVoltages == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating shadow voltages table. \n"));
        goto fail;
    }

    // memset
    for (i = 0; i < Max8907bPmuSupply_Num; i++)
    {
        ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[i] = 0;
        // Setting shadow to 0 would cause spare delay on the 1st scaling of
        // always On rail; however the alternative reading of initial settings
        // over I2C is even worse.
        ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages[i] = 0;
    }

    if (pConnectivity != NULL) // PMU is in database
    {
        NvU32 i = 0;

        for (i = 0; i < pConnectivity->NumAddress; i ++)
        {
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_I2c_Pmu)
            {
                I2cModule   = NvOdmIoModule_I2c_Pmu;
                I2cInstance = pConnectivity->AddressList[i].Instance;
                I2cAddress  = pConnectivity->AddressList[i].Address;
                break;
            }
        }

        NV_ASSERT(I2cModule  == NvOdmIoModule_I2c_Pmu);
        NV_ASSERT(I2cAddress != 0);

        ((Max8907bPrivData*)hDevice->pPrivate)->hOdmI2C = NvOdmI2cOpen(I2cModule, I2cInstance);
        if (!((Max8907bPrivData*)hDevice->pPrivate)->hOdmI2C)
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: Error Opening I2C device. \n"));
            NVODMPMU_PRINTF(("[NVODM PMU]Please check PMU device I2C settings. \n"));
            return NV_FALSE;
        }
        ((Max8907bPrivData*)hDevice->pPrivate)->DeviceAddr = I2cAddress;
        ((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice = NvOdmServicesPmuOpen();
    }
    else
    {
        // if PMU is not present in the database, then the platform is PMU-less.
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: The system did not doscover PMU from the data base.\n"));
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: If this is not intended, please check the peripheral database for PMU settings.\n"));
        return NV_FALSE;
    }

    // Configure OnOff options
    if (!Max8907bOnOffConfigure(hDevice))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: Max8907bOnOffConfigure() failed. \n"));
        return NV_FALSE;
    }

    // Configure PWREN, and attach CPU V1 rail to it.
    // TODO: h/w events (power cycle, reset, battery low) auto-disables PWREN.
    // Only soft reset (not supported) requires s/w to disable PWREN explicitly
    if (!Max8907bPwrEnConfigure(hDevice, NV_TRUE))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: Max8907bPwrEnConfigure() failed. \n"));
        return NV_FALSE;
    }
    if (!Max8907bPwrEnAttach(hDevice, Max8907bPmuSupply_LX_V1, NV_TRUE))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: Max8907bPwrEnAttach() failed. \n"));
        return NV_FALSE;
    }

    //Check battery presence
    if (!Max8907bBatteryChargerMainBatt(hDevice,&((Max8907bPrivData*)hDevice->pPrivate)->battPresence))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup: Max8907bBatteryChargerMainBatt() failed. \n"));
        return NV_FALSE;
    }

    // Power up Whistler thermal monitor (it is reported "not connected"
    // if board info ROMs cannot be read when the thermal rail is enabled).
    // This has to be done as early as possible because of 100ms+ power up
    // delay before interface level shifters are operational.
    pConnectivity =
        NvOdmPeripheralGetGuid(NV_ODM_GUID('a','d','t','7','4','6','1',' '));
    if (pConnectivity)
    {
        for (i = 0; i < pConnectivity->NumAddress; i++)
        {
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd)
            {
                NvU32 vddRail = pConnectivity->AddressList[i].Address;
                NvU32 mv =
                    Max8907bSupplyInfoTable[vddRail].cap.requestMilliVolts;
                if(!Max8907bSetVoltage(hDevice, vddRail, mv, NULL))
                {
                    NVODMPMU_PRINTF(("[NVODM PMU]Max8907bSetup:"
                                     " Thermal rail setup failed. \n"));
                }
            }
        }
    }

    // The interrupt assumes not supported until Max8907bInterruptHandler() is called.
    ((Max8907bPrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_FALSE;

    return NV_TRUE;

fail:
    Max8907bRelease(hDevice);
    return NV_FALSE;
}

void
Max8907bRelease(NvOdmPmuDeviceHandle hDevice)
{
    if (hDevice->pPrivate != NULL)
    {
        if (((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice != NULL)
        {
            NvOdmServicesPmuClose(((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice);
            ((Max8907bPrivData*)hDevice->pPrivate)->hOdmPmuSevice = NULL;
        }

        if (((Max8907bPrivData*)hDevice->pPrivate)->hOdmI2C != NULL)
        {
            NvOdmI2cClose(((Max8907bPrivData*)hDevice->pPrivate)->hOdmI2C);
            ((Max8907bPrivData*)hDevice->pPrivate)->hOdmI2C = NULL;
        }

        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable != NULL)
        {
            NvOdmOsFree(((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable);
            ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable = NULL;
        }

        if (((Max8907bPrivData*)hDevice->pPrivate)->pVoltages != NULL)
        {
            NvOdmOsFree(((Max8907bPrivData*)hDevice->pPrivate)->pVoltages);
            ((Max8907bPrivData*)hDevice->pPrivate)->pVoltages = NULL;
        }


        NvOdmOsFree(hDevice->pPrivate);
        hDevice->pPrivate = NULL;
    }
}

NvBool
Max8907bGetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(pMilliVolts);
    NV_ASSERT(vddRail < Max8907bPmuSupply_Num);

    // RTC is a special case, since it's "always on"
    if (vddRail == Max8907bPmuSupply_VRTC)
    {
        // Fixed voltage
        *pMilliVolts = MAX8907B_MAX_OUTPUT_VOLTAGE_RTC;
    }
    else if (vddRail == Max8907bPmuSupply_EXT_DCDC_1)
    {
        // Fixed voltage
        *pMilliVolts = MAX8907B_REQUESTVOLTAGE_EXT_DCDC_1;
    }
    else if (vddRail == Max8907bPmuSupply_EXT_DCDC_3)
    {
#if defined(MAX8907B_USE_FAN5355_VOLTAGE_SCALING)
        if (!Fan5355ReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
#else
        // Fixed voltage
        *pMilliVolts = MAX8907B_REQUESTVOLTAGE_EXT_DCDC_3;
#endif
    }
    else if((vddRail == MIC2826PmuSupply_BUCK) ||
            (vddRail == MIC2826PmuSupply_LDO1) ||
            (vddRail == MIC2826PmuSupply_LDO2) ||
            (vddRail == MIC2826PmuSupply_LDO3))
    {
        // Secondary PMU Case
        if(! MIC2826ReadVoltageReg(hDevice, (vddRail), pMilliVolts))
            return NV_FALSE;
    }
    else if (vddRail == Max8907bLxV1_Ad5258_DPM_EXT_DCDC_7)
    {
        // External DCDC controlled by LX_V1, and scaled by DPM
        if (!Max8907bLxV1Ad5258ReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
    }
    else
    {
        if (!Max8907bReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
    }

    return NV_TRUE;
}


static  NvBool
Tca6416UsbVbusControl(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts)
{
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];
    NvU32 PortNo;
    NvU32 PinNo;

    // Get port number and pin number
    PortNo = pSupplyInfo->OutputPort;
    PinNo = pSupplyInfo->PmuGpio;

    // Configure port pin as output
    if (!Tca6416ConfigPortPin(hDevice, PortNo, PinNo, GpioPinMode_Output))
        return NV_FALSE;

    if (MilliVolts == ODM_VOLTAGE_OFF)  // to disable VBUS
    {
        // Set Low  on pin
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_Low))
            return NV_FALSE;
    }
    else  // to Enable VBUS
    {
        // Set high  on pin
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_High))
            return NV_FALSE;
    }
    return NV_TRUE;
}

NvBool
Max8907bSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(vddRail < Max8907bPmuSupply_Num);

    if ( Max8907bSupplyInfoTable[vddRail].cap.OdmProtected == NV_TRUE)
    {
        NVODMPMU_PRINTF(("The voltage is protected and cannot be set.\n"));
        return NV_TRUE;
    }

    if ((MilliVolts == ODM_VOLTAGE_ENABLE_EXT_ONOFF) ||
        (MilliVolts == ODM_VOLTAGE_DISABLE_EXT_ONOFF))
    {
        return Max8907bPwrEnAttach(hDevice, (Max8907bPmuSupply)vddRail,
            (MilliVolts == ODM_VOLTAGE_ENABLE_EXT_ONOFF));
    }

    if ((MilliVolts == ODM_VOLTAGE_OFF) ||
        ((MilliVolts <=  Max8907bSupplyInfoTable[vddRail].cap.MaxMilliVolts) &&
         (MilliVolts >=  Max8907bSupplyInfoTable[vddRail].cap.MinMilliVolts)))
    {
        if ((vddRail == Max8907bPmuSupply_EXT_DCDC_1) ||
            (vddRail == Max8907bPmuSupply_EXT_DCDC_3) ||
            (vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB1) ||
            (vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB3))
        {
            // Use External DC/DC switcher
            if (!Fan5355WriteVoltageReg(hDevice, vddRail, MilliVolts))
                return NV_FALSE;
        }
        else if((vddRail == MIC2826PmuSupply_BUCK) ||
                (vddRail == MIC2826PmuSupply_LDO1) ||
                (vddRail == MIC2826PmuSupply_LDO2) ||
                (vddRail == MIC2826PmuSupply_LDO3))
        {
            // Secondary PMU Case
            if (!MIC2826WriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
        else if (vddRail == Max8907bLxV1_Ad5258_DPM_EXT_DCDC_7)
        {
            // External DCDC controlled by LX_V1, and scaled by DPM
            if (!Max8907bLxV1Ad5258WriteVoltageReg(
                hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
        else
        {
            if (!Max8907bWriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("The requested voltage is not supported.\n"));
        return NV_FALSE;
    }

    // Check whether need to enable VBUS for any of the USB Instance
    if ((vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB1) ||
        (vddRail == Max8907bPmuSupply_EXT_DCDC_3_USB3))
    {
        // Enable  VBUS for USB1 or USB3
       if (!Tca6416UsbVbusControl(hDevice, vddRail, MilliVolts))
            return NV_FALSE;
    }
    return NV_TRUE;
}

void
Max8907bGetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities)
{
    NV_ASSERT(pCapabilities);
    NV_ASSERT(vddRail < Max8907bPmuSupply_Num);

    *pCapabilities = Max8907bSupplyInfoTable[vddRail].cap;
}

NvBool
Max8907bGetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus)
{
    NvBool acLineStatus = NV_FALSE;

    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);

    // check if battery is present
    if (((Max8907bPrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
    {
        *pStatus = NvOdmPmuAcLine_Online;
        return NV_TRUE;
    }

    if ( ((Max8907bPrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE )
    {
        if ( ((Max8907bPrivData*)hDevice->pPrivate)->pmuStatus.mChgPresent == NV_TRUE )
        {
            *pStatus = NvOdmPmuAcLine_Online;
            acLineStatus = NV_TRUE;
        }
        else
        {
            *pStatus = NvOdmPmuAcLine_Offline;
            acLineStatus = NV_FALSE;
        }
    }
    else
    {
        // battery is present, now check if charger present
        if (!Max8907bBatteryChargerOK(hDevice, &acLineStatus))
        {
            NVODMPMU_PRINTF(("[NVODM PMU] Max8907bGetAcLineStatus: Error in checking main charger presence.\n"));
            return NV_FALSE;
        }

        if (acLineStatus == NV_TRUE)
            *pStatus = NvOdmPmuAcLine_Online;
        else
            *pStatus = NvOdmPmuAcLine_Offline;
    }

#if ALWAYS_ONLINE
    // Currently on Whistler battery is not used, and AC is connected to PMU
    // battery input, causing false battery presence detection. Voyager battery
    // management is don't care at the moment. Hence, force OnLine status.
    *pStatus = NvOdmPmuAcLine_Online;
#endif

    return NV_TRUE;
}

NvBool
Max8907bGetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus)
{
    NvU8 status = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    if (batteryInst == NvOdmPmuBatteryInst_Main)
    {
        if ( ((Max8907bPrivData*)hDevice->pPrivate)->battPresence == NV_TRUE )
        {
            NvOdmPmuAcLineStatus stat = NvOdmPmuAcLine_Offline;
            NvU32 VBatSense = 0;
            if (!Max8907bGetAcLineStatus(hDevice, &stat))
                return NV_FALSE;

            if (stat == NvOdmPmuAcLine_Online)
            {
                if ( ((Max8907bPrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE )
                {
                    if ( ((Max8907bPrivData*)hDevice->pPrivate)->pmuStatus.batFull == NV_FALSE )
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
                else
                {
                    NvBool batFull = NV_FALSE;
                    if (!Max8907bBatteryChargerMainBattFull(hDevice, &batFull))
                        return NV_FALSE;
                    if (batFull == NV_FALSE)
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
            }

            // Get VBatSense
            if (!Max8907bAdcVBatSenseRead(hDevice, &VBatSense))
                return NV_FALSE;

            // TO DO: Update status based on VBatSense
        }
        else
        {
            /* Battery is actually not present */
            status = NVODM_BATTERY_STATUS_NO_BATTERY;
        }
        *pStatus = status;
    }
    else
    {
        *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
    }

    return NV_TRUE;
}

NvBool
Max8907bGetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData)
{
    NvOdmPmuBatteryData batteryData;

    batteryData.batteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    NV_ASSERT(hDevice);
    NV_ASSERT(pData);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    if (batteryInst == NvOdmPmuBatteryInst_Main)
    {
        NvU32 VBatSense = 0;
        NvU32 VBatTemp  = 0;

        if (((Max8907bPrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
        {
            /* retrieve Battery voltage and temperature */

            // Get VBatSense
            if (!Max8907bAdcVBatSenseRead(hDevice, &VBatSense))
            {
                NVODMPMU_PRINTF(("Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            // Get VBatTemp
            if (!Max8907bAdcVBatTempRead(hDevice, &VBatTemp))
            {
                NVODMPMU_PRINTF(("Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            batteryData.batteryVoltage = VBatSense;
            batteryData.batteryTemperature = Max8907bBatteryTemperature(VBatSense, VBatTemp);
        }

        *pData = batteryData;
    }
    else
    {
        *pData = batteryData;
    }

    return NV_TRUE;
}

void
Max8907bGetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}

void
Max8907bGetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry)
{
    *pChemistry = NvOdmPmuBatteryChemistry_LION;
}

NvBool
Max8907bSetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType)
{
    NvU8 data = 0;
    NvU8 fchg = 0;
    NV_ASSERT(hDevice);

    // If no battery is connected, then do nothing.
    if (((Max8907bPrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
        return NV_TRUE;

    // If requested current is more than supported maximum then limit to supported.
    if ( chargingCurrentLimitMa > MAX_CHARGER_LIMIT_MA )
        chargingCurrentLimitMa = MAX_CHARGER_LIMIT_MA;

    // If dedicated charger is connected, request maximum current.
    if (chargingPath == NvOdmPmuChargingPath_UsbBus)
    {
        switch (ChargerType)
        {
            case NvOdmUsbChargerType_SJ:
            case NvOdmUsbChargerType_SK:
            case NvOdmUsbChargerType_SE1:
            case NvOdmUsbChargerType_SE0:
                chargingCurrentLimitMa = MAX_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_UsbHost:
            default:
                break;
        }
    }

    // Read the current charger setup.
    if ( !Max8907bI2cRead8(hDevice, MAX8907B_CHG_CNTL1, &data) )
        return NV_FALSE;

    // Set charging current to the value no larger than requested.
    // If less than 85mA is requested, set to 85mA.
    // If larger than 1000mA is requested, set to 1000mA.
    if (chargingCurrentLimitMa >= 1000)
        fchg = MAX8907B_CHG_CNTL1_FCHG_1000MA;
    else if (chargingCurrentLimitMa >= 900)
        fchg = MAX8907B_CHG_CNTL1_FCHG_900MA;
    else if (chargingCurrentLimitMa >= 800)
        fchg = MAX8907B_CHG_CNTL1_FCHG_800MA;
    else if (chargingCurrentLimitMa >= 700)
        fchg = MAX8907B_CHG_CNTL1_FCHG_700MA;
    else if (chargingCurrentLimitMa >= 600)
        fchg = MAX8907B_CHG_CNTL1_FCHG_600MA;
    else if (chargingCurrentLimitMa >= 460)
        fchg = MAX8907B_CHG_CNTL1_FCHG_460MA;
    else if (chargingCurrentLimitMa >= 300)
        fchg = MAX8907B_CHG_CNTL1_FCHG_300MA;
    else
        fchg = MAX8907B_CHG_CNTL1_FCHG_85MA;

    data &= ~(MAX8907B_CHG_CNTL1_FCHG_MASK <<
              MAX8907B_CHG_CNTL1_FCHG_SHIFT);
    data |= fchg << MAX8907B_CHG_CNTL1_FCHG_SHIFT;

    // Turn off the charger path if the requested current limit is 0mA.
    // Turn on the path otherwise.
    if ( chargingCurrentLimitMa == 0 )
    {
        // off
        data |= (MAX8907B_CHG_CNTL1_NOT_CHGEN_MASK <<
                 MAX8907B_CHG_CNTL1_NOT_CHGEN_SHIFT);
    }
    else
    {
        // on
        data &= ~(MAX8907B_CHG_CNTL1_NOT_CHGEN_MASK <<
                  MAX8907B_CHG_CNTL1_NOT_CHGEN_SHIFT);
    }

    // Update the current charger setup.
    if ( !Max8907bI2cWrite8(hDevice, MAX8907B_CHG_CNTL1, data) )
        return NV_FALSE;

    return NV_TRUE;
}

void Max8907bInterruptHandler( NvOdmPmuDeviceHandle  hDevice)
{
    // If the interrupt handle is called, the interrupt is supported.
    ((Max8907bPrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_TRUE;

    Max8907bInterruptHandler_int(hDevice, &((Max8907bPrivData*)hDevice->pPrivate)->pmuStatus);
}

/****************   Secondary PMU MIC2826 Programming  */
static NvBool
MIC2826ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    NvU32 milliVolts = 0;
    NvU32 index = 0;
    NvU8 data = 0;
    const Max8907bPmuSupplyInfo *pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    if(! MIC2826I2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data))
        return NV_FALSE;

    // convert Data to MilliVolts
    if (!data) //OFF
        milliVolts = 0;
    else
    {
        // set voltage
        if(vddRail == MIC2826PmuSupply_BUCK)
        {
            for(index=0;index<MIC2826_BUCK_Votage_Table_Size;index++)
            {
                if(data == MIC2826_BUCK_Votage_Table[index])
                    break;
            }
            if(index  < 0x10)
            {
                milliVolts = index * MIC2826_BUCK_VOLTAGE_STEP_25MV + MIC2826_BUCK_VOLTAGE_OFFSET ;
            }else
            {
                milliVolts = 1200 + ((index - 0x10) * MIC2826_BUCK_VOLTAGE_STEP_50MV) ;
            }
        }
        else if ( (vddRail == MIC2826PmuSupply_LDO1) ||
                  (vddRail == MIC2826PmuSupply_LDO2) ||
                  (vddRail == MIC2826PmuSupply_LDO3))
        {
            for(index=0;index<MIC2826_LDO_Votage_Table_Size;index++)
            {
                if(data == MIC2826_BUCK_Votage_Table[index])
                   break;
            }
            milliVolts = (index * pSupplyInfo->cap.StepMilliVolts) + MIC2826_LDO_VOLTAGE_OFFSET;
        }
    }

    *pMilliVolts = milliVolts;

    return NV_TRUE;
}


static NvBool
MIC2826WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32  MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NvU8 data = 0;
    NvU32 index = 0;
    NvU32 settleTime = 0;
    const Max8907bPmuSupplyInfo* pSupplyInfo = &Max8907bSupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907bPmuSupply)vddRail);

    // Require to turn off the supply
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1)
        {

            // Read the current supply info
            if(! MIC2826I2cRead8(hDevice, MIC2826_REG_ADDR_ENABLE, &data))
                return NV_FALSE;

            // turn off the supply of particular rail
            if(vddRail == MIC2826PmuSupply_BUCK)
                data &= MIC2826_REG_DISABLE_BK;
            else if(vddRail == MIC2826PmuSupply_LDO1)
                data &= MIC2826_REG_DISABLE_LDO1;
            else if(vddRail == MIC2826PmuSupply_LDO2)
                data &= MIC2826_REG_DISABLE_LDO2;
            else if(vddRail == MIC2826PmuSupply_LDO3)
                data &= MIC2826_REG_DISABLE_LDO3;

            if (!MIC2826I2cWrite8(hDevice, MIC2826_REG_ADDR_ENABLE, data))
                return NV_FALSE;
        }

        //TODO: check if the supply input can be turned off
        if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] != 0)
            ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] --;

        if (pSettleMicroSeconds != NULL)
            *pSettleMicroSeconds = settleTime;
        else
            NvOdmOsWaitUS(settleTime);

        return NV_TRUE;
    }

    // turn on supply
    if (((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)
    {
        {
            // Read the current supply info
            if(! MIC2826I2cRead8(hDevice, MIC2826_REG_ADDR_ENABLE, &data))
                return NV_FALSE;

            // turn on the supply of particular rail
           if(vddRail == MIC2826PmuSupply_BUCK)
                data |= MIC2826_REG_ENABLE_BK;
           else if(vddRail == MIC2826PmuSupply_LDO1)
                data |= MIC2826_REG_ENABLE_LDO1;
           else if(vddRail == MIC2826PmuSupply_LDO2)
                data |= MIC2826_REG_ENABLE_LDO2;
           else if(vddRail == MIC2826PmuSupply_LDO3)
                data |= MIC2826_REG_ENABLE_LDO3;

           if (!MIC2826I2cWrite8(hDevice, MIC2826_REG_ADDR_ENABLE, data))
                return NV_FALSE;
        }
    }

    // set voltage
    if(vddRail == MIC2826PmuSupply_BUCK)
    {
       if(MilliVolts < 1200)
       {
           index = (MilliVolts - MIC2826_BUCK_VOLTAGE_OFFSET) / MIC2826_BUCK_VOLTAGE_STEP_25MV ;
       }else
       {
           index = 0x10 + ((MilliVolts - 1200) / MIC2826_BUCK_VOLTAGE_STEP_50MV) ;
       }
       data = MIC2826_BUCK_Votage_Table[index];
    }
    else if ( (vddRail == MIC2826PmuSupply_LDO1) ||
              (vddRail == MIC2826PmuSupply_LDO2) ||
              (vddRail == MIC2826PmuSupply_LDO3))
    {
        index = (MilliVolts - MIC2826_LDO_VOLTAGE_OFFSET) / pSupplyInfo->cap.StepMilliVolts;
        data = MIC2826_LDO_Votage_Table[index];
    }

    // Program the Rail Voltage
    if(! MIC2826I2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data))
         return NV_FALSE;

    ((Max8907bPrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] ++;

    // TODO: turn on supply input if necessary
    if (pSettleMicroSeconds != NULL)
        *pSettleMicroSeconds = settleTime;
    else
        NvOdmOsWaitUS(settleTime);

    return NV_TRUE;
}

