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
     
#include <linux/kernel.h>

#include "nvodm_query_discovery.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "max8907.h"
#include "max8907_reg.h"
#include "max8907_adc.h"
#include "max8907_i2c.h"
#include "max8907_interrupt.h"
#include "max8907_batterycharger.h"
#include "max8907_supply_info_table.h"
//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
#include "fan5355_buck_reg.h"
#include "fan5355_buck_i2c.h"
#include "tca6416_expander_reg.h"
#include "tca6416_expander_i2c.h"
#include "mic2826_reg.h"
#include "mic2826_i2c.h"
#include "ad5258_dpm.h"
#endif
//20100413, , unused [END]

#include "max8952_buck_reg.h"
#include "max8952_buck_i2c.h"

//20100528, , Write the description here in detail [START]
#if defined (CONFIG_MACH_STAR)
#include <linux/err.h>
#endif // CONFIG_MACH_STAR
//20100528, , Write the description here in detail [END]

//20100428, , This define for Debug Message function [START]
#include <mach/lprintk.h>

//#define LG_DEBUG_PMU
#undef LG_DEBUG_PMU  // Define for Debug Serial

#ifdef LG_DEBUG_PMU
#define LDP(fmt, arg...) lprintk(D_BATT, "%s : " fmt "\n", __func__, ## arg)
#else
#define LDP(fmt, arg...) do {} while (0)
#endif // LG_DEBUG_BATT
//20100428, , This define for Debug Message function [END]

//20100528, , Write the description here in detail [START]
#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
typedef enum {
  CHG_IC_DEFAULT_MODE=0,    		/* 0  */
  CHG_IC_TA_MODE,
  CHG_IC_USB_LO_MODE,
  CHG_IC_FACTORY_MODE,

  CHG_IC_DEACTIVE_MODE,			/* 4  */
  CHG_IC_INIT_MODE,
} max8922_status;

extern max8922_status get_charging_ic_status(void);
extern NvBool ARRAY_TP_BOOT(void);
#endif // CONFIG_STAR_BATTERY_CHARGER
//20100528, , Write the description here in detail [END]

//20110131, , Stop i2c comm during reset 
NvBool stop_i2c_flag = NV_FALSE;

// Private PMU context info
Max8907PrivData *hMax8907Pmu;

#define PMUGUID NV_ODM_GUID('m','a','x','8','9','0','7','_')

#define MAX_CHARGER_LIMIT_MA    1000

//20100528, , This definition is for Whistler which has no Battery! [START]
#if defined (CONFIG_MACH_STAR)  && defined (CONFIG_STAR_BATTERY_CHARGER)
#define ALWAYS_ONLINE (0)
#else
#define ALWAYS_ONLINE (1)
#endif
//20100528, , This definition is for Whistler which has no Battery! [END]

/**
 * MAX8907 regulators can be enabled/disabled via s/w I2C commands only
 * when MAX8907_SEQSEL_I2CEN_LXX (7) is selected as regulator sequencer.
 * Otherwise, regulator is controlled by h/w sequencers: SEQ1 (SYSEN),
 * which is always On when PMU is On, or SEQ2 (PWREN) which is always On,
 * when system is running (it is Off in LPx mode only).
 */
#define MAX8907_OUT_VOLTAGE_CONTROL_MASK \
    ((MAX8907_CTL_SEQ_MASK << MAX8907_CTL_SEQ_SHIFT) | \
     MAX8907_OUT_VOLTAGE_ENABLE_BIT)

#define MAX8907_OUT_VOLTAGE_CONTROL_DISABLE \
    (MAX8907_SEQSEL_I2CEN_LXX << MAX8907_CTL_SEQ_SHIFT)

// MAX8907 revision that requires s/w WAR to connect PWREN input to
// sequencer 2 because of the bug in the silicon.
#define MAX8907_II2RR_PWREN_WAR (0x12)

// Power up AVDD_USB on exit from deep sleep mode together with core rail,
// before boot code starts, and keep it On during resume until USB driver
// takes over
#define WAR_AVDD_USB_EARLY_PWR_UP 1
//#define WAR_AVDD_USB_RESUME_KEEP_ON 1

/**
*   The FAN5355 is used to scale the voltage of an external
*   DC/DC voltage rail (for PCIE).  However, voltage scaling is
*   not required for this source, since the 1.05V default
*   voltage when enabled is OK.  On some boards, the FAN5355 may
*   not function properly, as an I2C re-work may be required
*   (otherwise, the slave address may not be found).  Therefore,
*   this feature is disabled by default.
*/
#undef MAX8907_USE_FAN5355_VOLTAGE_SCALING

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
//static const NvU32 SequencerPeriod[] = {20, 40, 80, 160, 320, 640, 1280, 2560};

/*-- Voltage translation functions --*/

// OutVoltageIndex is the lower six bits of the output voltage registers, VO[5:0]
static NvU32 Max8907PmuVoltageGet_SD_A(const NvU32 OutVoltageIndex);
static NvU32 Max8907PmuVoltageGet_SD_B_LDO_B(const NvU32 OutVoltageIndex);
static NvU32 Max8907PmuVoltageGet_LDO_A(const NvU32 OutVoltageIndex);

static NvU32 Max8907PmuVoltageSet_SD_A(const NvU32 OutMilliVolts);
static NvU32 Max8907PmuVoltageSet_SD_B_LDO_B(const NvU32 OutMilliVolts);
static NvU32 Max8907PmuVoltageSet_LDO_A(const NvU32 OutMilliVolts);

#define MAX8907_MAX_OUTPUT_VOLTAGE_INDEX   0x3F
//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
#define FAN5355_MAX_OUTPUT_VOLTAGE_INDEX    0x37
#endif
//20100413, , unused [END]
#define MAX8952_MAX_OUTPUT_VOLTAGE_INDEX	0x3F

static NvU32 Max8907PmuVoltageGet_SD_A(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_SD_A[OutVoltageIndex]/10;
}

static NvU32 Max8907PmuVoltageGet_SD_B_LDO_B(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_SD_B_LDO_B[OutVoltageIndex];
}

static NvU32 Max8907PmuVoltageGet_LDO_A(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
    return VoltageTable_LDO_A[OutVoltageIndex];
}

//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
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
#endif
//20100413, , unused [END]

#ifndef MIN
#define MIN(a, b)   (a) <= (b) ? (a) : (b)
#endif

#define MAX8907_MIN_OUTPUT_VOLTAGE_SD_A_x10          6375  // 637.5 mV
#define MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B        750   // 750 mV
#define MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A             650   // 650 mV
//20100413, , unused
#ifndef CONFIG_MACH_STAR
#define FAN5355_MIN_OUTPUT_VOLTAGE_x10                7500  // 750.0 mV
#endif
//20100819, , Voltage bug fix
#define MAX8952_MIN_OUTPUT_VOLTAGE_x10				7700 // 770.0mV <-- 750.0mV

#define MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10     125  // 12.5 mV
#define MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B   50   // 50 mV
#define MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A        25   // 25 mV
//20100413, , unused
#ifndef CONFIG_MACH_STAR
#define FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10           125  // 12.5 mV
#endif
#define MAX8952_OUTPUT_VOLTAGE_INCREMENT_x10		100 // 10 mV

#define MAX8907_MAX_OUTPUT_VOLTAGE_SD_A_x10         14250  // 1,425.0 mV
#define MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B       3900   // 3,900 mV
#define MAX8907_MAX_OUTPUT_VOLTAGE_LDO_A            2225   // 2,225 mV
//20100413, , unused
#ifndef CONFIG_MACH_STAR
#define FAN5355_MAX_OUTPUT_VOLTAGE_x10               14375  // 1,437.5 mV
#endif
#define MAX8952_MAX_OUTPUT_VOLTAGE_x10				13800 

#define MAX8907_MIN_OUTPUT_VOLTAGE_RTC                 0   // 0 mV
#define MAX8907_OUTPUT_VOLTAGE_INCREMENT_RTC           1   // Protected; use dummy, non-zero value
//#define MAX8907_MAX_OUTPUT_VOLTAGE_RTC              3300   // 3,300 mV
// WHISTLER/AP16 - Make this 1.2V for now, since ap15rm_power.c expects it that way.
#define MAX8907_MAX_OUTPUT_VOLTAGE_RTC              1200

static NvU32 Max8907PmuVoltageSet_SD_A(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907_MIN_OUTPUT_VOLTAGE_SD_A_x10/10)
        return 0;
    else
        return MIN(                                                 \
            (OutMilliVolts*10 - MAX8907_MIN_OUTPUT_VOLTAGE_SD_A_x10) /    \
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10,                 \
            MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
}

static NvU32 Max8907PmuVoltageSet_SD_B_LDO_B(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B)
        return 0;
    else
        return MIN(                                                     \
            (OutMilliVolts - MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B) /  \
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,               \
            MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
}

static NvU32 Max8907PmuVoltageSet_LDO_A(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A)
        return 0;
    else
        return MIN(                                                 \
            (OutMilliVolts - MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A) /   \
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A,                \
            MAX8907_MAX_OUTPUT_VOLTAGE_INDEX);
}

//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
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
#endif
//20100413, , unused [END]

static NvU32 Max8952PmuVoltageGet_VOUT(const NvU32 OutVoltageIndex)
{
    NV_ASSERT(OutVoltageIndex <= MAX8952_MAX_OUTPUT_VOLTAGE_INDEX);
    return (MAX8952_MIN_OUTPUT_VOLTAGE_x10/10 + OutVoltageIndex * MAX8952_OUTPUT_VOLTAGE_INCREMENT_x10/10);
}

static NvU32 Max8952PmuVoltageSet_VOUT(const NvU32 OutMilliVolts)
{
    if (OutMilliVolts < MAX8952_MIN_OUTPUT_VOLTAGE_x10/10){
		//printk(("ERROR Max8952PmuVoltageSet_VOUT: invalid voltage range. \n"));
        return 0;
	}
    else
        return MIN( (OutMilliVolts - MAX8952_MIN_OUTPUT_VOLTAGE_x10/10)/10 ,  MAX8952_MAX_OUTPUT_VOLTAGE_INDEX);
}


// This board-specific table is indexed by Max8907PmuSupply
const Max8907PmuSupplyInfo Max8907SupplyInfoTable[] =
{
    {
        Max8907PmuSupply_Invalid,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // LX_V1 (V1)
    {
        Max8907PmuSupply_LX_V1,
        MAX8907_SDCTL1,
        MAX8907_SDSEQCNT1,
        MAX8907_SDV1,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_A,
        Max8907PmuVoltageSet_SD_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10/10,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907_REQUESTVOLTAGE_LX_V1
        },
    },

    // LX_V2 (V2)
    {
        Max8907PmuSupply_LX_V2,
        MAX8907_SDCTL2,
        MAX8907_SDSEQCNT2,
        MAX8907_SDV2,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_A,
        Max8907PmuVoltageSet_SD_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_A_x10/10,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_A_x10/10,
            MAX8907_REQUESTVOLTAGE_LX_V2
        },
    },

    // LX_V3 (V3)
    {
        Max8907PmuSupply_LX_V3,
        MAX8907_SDCTL3,
        MAX8907_SDSEQCNT3,
        MAX8907_SDV3,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LX_V3
        },
    },

    // VRTC (RTC)
    {
        Max8907PmuSupply_VRTC,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_TRUE,
            MAX8907_MIN_OUTPUT_VOLTAGE_RTC,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_RTC,
            MAX8907_MAX_OUTPUT_VOLTAGE_RTC,
            MAX8907_MAX_OUTPUT_VOLTAGE_RTC
        },
    },

    // LDO1 (VOUT1)
    {
        Max8907PmuSupply_LDO1,
        MAX8907_LDOCTL1,
        MAX8907_LDOSEQCNT1,
        MAX8907_LDO1VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO1
        },
    },

    // LDO2 (VOUT2)
    {
        Max8907PmuSupply_LDO2,
        MAX8907_LDOCTL2,
        MAX8907_LDOSEQCNT2,
        MAX8907_LDO2VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_LDO_A,
        Max8907PmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_REQUESTVOLTAGE_LDO2
        },
    },

    // LDO3 (VOUT3)
    {
        Max8907PmuSupply_LDO3,
        MAX8907_LDOCTL3,
        MAX8907_LDOSEQCNT3,
        MAX8907_LDO3VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_LDO_A,
        Max8907PmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_REQUESTVOLTAGE_LDO3
        },
    },

    // LDO4 (VOUT4)
    {
        Max8907PmuSupply_LDO4,
        MAX8907_LDOCTL4,
        MAX8907_LDOSEQCNT4,
        MAX8907_LDO4VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO4
        },
    },

    // LDO5 (VOUT5)
    {
        Max8907PmuSupply_LDO5,
        MAX8907_LDOCTL5,
        MAX8907_LDOSEQCNT5,
        MAX8907_LDO5VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO5
        },
    },

    // LDO6 (VOUT6)
    {
        Max8907PmuSupply_LDO6,
        MAX8907_LDOCTL6,
        MAX8907_LDOSEQCNT6,
        MAX8907_LDO6VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO6
        },
    },

    // LDO7 (VOUT7)
    {
        Max8907PmuSupply_LDO7,
        MAX8907_LDOCTL7,
        MAX8907_LDOSEQCNT7,
        MAX8907_LDO7VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO7
        },
    },

    // LDO8 (VOUT8)
    {
        Max8907PmuSupply_LDO8,
        MAX8907_LDOCTL8,
        MAX8907_LDOSEQCNT8,
        MAX8907_LDO8VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO8
        },
    },

    // LDO9 (VOUT9)
    {
        Max8907PmuSupply_LDO9,
        MAX8907_LDOCTL9,
        MAX8907_LDOSEQCNT9,
        MAX8907_LDO9VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO9
        },
    },

    // LDO10 (VOUT10)
    {
        Max8907PmuSupply_LDO10,
        MAX8907_LDOCTL10,
        MAX8907_LDOSEQCNT10,
        MAX8907_LDO10VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO10
        },
    },

    // LDO11 (VOUT11)
    {
        Max8907PmuSupply_LDO11,
        MAX8907_LDOCTL11,
        MAX8907_LDOSEQCNT11,
        MAX8907_LDO11VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO11
        },
    },

    // LDO12 (VOUT12)
    {
        Max8907PmuSupply_LDO12,
        MAX8907_LDOCTL12,
        MAX8907_LDOSEQCNT12,
        MAX8907_LDO12VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO12
        },
    },

    // LDO13 (VOUT13)
    {
        Max8907PmuSupply_LDO13,
        MAX8907_LDOCTL13,
        MAX8907_LDOSEQCNT13,
        MAX8907_LDO13VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO13
        },
    },

    // LDO14 (VOUT14)
    {
        Max8907PmuSupply_LDO14,
        MAX8907_LDOCTL14,
        MAX8907_LDOSEQCNT14,
        MAX8907_LDO14VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO14
        },
    },

    // LDO15 (VOUT15)
    {
        Max8907PmuSupply_LDO15,
        MAX8907_LDOCTL15,
        MAX8907_LDOSEQCNT15,
        MAX8907_LDO15VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO15
        },
    },

    // LDO16 (VOUT16)
    {
        Max8907PmuSupply_LDO16,
        MAX8907_LDOCTL16,
        MAX8907_LDOSEQCNT16,
        MAX8907_LDO16VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO16
        },
    },

    // LDO17 (VOUT17)
    {
        Max8907PmuSupply_LDO17,
        MAX8907_LDOCTL17,
        MAX8907_LDOSEQCNT17,
        MAX8907_LDO17VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_LDO_A,
        Max8907PmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_REQUESTVOLTAGE_LDO17
        },
    },

    // LDO18 (VOUT18)
    {
        Max8907PmuSupply_LDO18,
        MAX8907_LDOCTL18,
        MAX8907_LDOSEQCNT18,
        MAX8907_LDO18VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_LDO_A,
        Max8907PmuVoltageSet_LDO_A,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_LDO_A,
            MAX8907_MAX_OUTPUT_VOLTAGE_LDO_A,
            MAX8907_REQUESTVOLTAGE_LDO18
        },
    },

    // LDO19 (VOUT19)
    {
        Max8907PmuSupply_LDO19,
        MAX8907_LDOCTL19,
        MAX8907_LDOSEQCNT19,
        MAX8907_LDO19VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO19
        },
    },

    // LDO20 (VOUT20)
    {
        Max8907PmuSupply_LDO20,
        MAX8907_LDOCTL20,
        MAX8907_LDOSEQCNT20,
        MAX8907_LDO20VOUT,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        Max8907PmuVoltageGet_SD_B_LDO_B,
        Max8907PmuVoltageSet_SD_B_LDO_B,
        {
            NV_FALSE,
            MAX8907_MIN_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_OUTPUT_VOLTAGE_INCREMENT_SD_B_LDO_B,
            MAX8907_MAX_OUTPUT_VOLTAGE_SD_B_LDO_B,
            MAX8907_REQUESTVOLTAGE_LDO20
        },
    },

    #if defined(CONFIG_MACH_STAR)
    // WHITE_LED
    {
        Max8907PmuSupply_WHITE_LED,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE, 0, 0, 0, 
            MAX8907_REQUESTVOLTAGE_WLED
        },
    },

    // Power off
    {
        Max8907PmuSupply_SOC,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        NULL,
        NULL,
        {NV_FALSE, 0, 0, 0, 0},
    },

    // Power reset
    {
        Max8907PmuSupply_reset,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_INVALID_PORT,
        MAX8907_INVALID_PORT,
        NULL,
        NULL,
        {NV_FALSE, 0, 0, 0, 0},
    },
    #endif

    //20100413, , unused [START]
    #ifndef CONFIG_MACH_STAR
    // EXT_DC/DC1 (for HDMI, VGA, USB)
    // By default, this is hard-wired as "always on" (see schematics)
    {
        Max8907PmuSupply_EXT_DCDC_1,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_TRUE,
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_1,
            0,
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_1,
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_1
        },
    },

    // EXT_DC/DC2 (not connected / reserved)
    {
        Max8907PmuSupply_EXT_DCDC_2,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC3 (PCI Express)
    {
        Max8907PmuSupply_EXT_DCDC_3,
        TCA6416_CONFIG_PORT_0,
        MAX8907_REG_INVALID,
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
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },

    // EXT_DC/DC4 (Backlight-1 Intensity Enable)
    // By default, this is hard-wired as "always enabled" (see schematics)
    {
        Max8907PmuSupply_EXT_DCDC_4,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC5 (Backlight-2 Intensity Enable)
    // By default, this is hard-wired as "always enabled" (see schematics)
    {
        Max8907PmuSupply_EXT_DCDC_5,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // EXT_DC/DC6 (not connected / reserved)
    {
        Max8907PmuSupply_EXT_DCDC_6,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // USB1   VBUS is wired from with DCDC_3.
    {
        Max8907PmuSupply_EXT_DCDC_3_USB1,
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
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },

    // USB3   VBUS is wired from with DCDC_3.
    {
        Max8907PmuSupply_EXT_DCDC_3_USB3,
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
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_3
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
        Max8907LxV1_Ad5258_DPM_EXT_DCDC_7,
        AD5258_RDAC_ADDR,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        NULL,
        NULL,
        {
            NV_FALSE,
            AD5258_V0,
            AD5258_MIN_STEP_MV,
            AD5258_VMAX,
            MAX8907_REQUESTVOLTAGE_LX_V1
        },
    },
    #endif
    //20100413, , unused [END]

    #if 0
    // FUSE Vcc is wired from VBAT.
    {
        Max8907PmuSupply_VBAT_FUSE,
        TCA6416_CONFIG_PORT_0,
        TCA6416_INVALID_PORT,
        TCA6416_INVALID_PORT,
        TCA6416_PORT_0,
        TCA6416_PIN_2,
        NULL,
        NULL,
        {
            NV_FALSE,
            FAN5355_MIN_OUTPUT_VOLTAGE_x10/10,
            FAN5355_OUTPUT_VOLTAGE_INCREMENT_x10/10,
            FAN5355_MAX_OUTPUT_VOLTAGE_x10/10,
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_3
        },
    },
    #endif

	// MAX8952
	{
		Max8907PmuSupply_EXT_DCDC_8_CPU,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        MAX8952_MODE1,
        MAX8907_REG_INVALID,
        MAX8907_REG_INVALID,
        Max8952PmuVoltageGet_VOUT,
        Max8952PmuVoltageSet_VOUT,
        {
            NV_FALSE,
            MAX8952_MIN_OUTPUT_VOLTAGE_x10/10,
            MAX8952_OUTPUT_VOLTAGE_INCREMENT_x10/10,
            MAX8952_MAX_OUTPUT_VOLTAGE_x10/10,
            MAX8907_REQUESTVOLTAGE_EXT_DCDC_3
        },		
	}
};

static NvBool
Max8907ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 milliVolts = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    if (pSupplyInfo->ControlRegAddr != MAX8907_REG_INVALID)
    {
        if (!Max8907I2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data))
            return NV_FALSE;

        if ((data & MAX8907_OUT_VOLTAGE_CONTROL_MASK) ==
            MAX8907_OUT_VOLTAGE_CONTROL_DISABLE)
        {
            ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail] =
                ODM_VOLTAGE_OFF;
            *pMilliVolts = ODM_VOLTAGE_OFF;
            return NV_TRUE;
        }
    }

    if (pSupplyInfo->OutputVoltageRegAddr == MAX8907_REG_INVALID)
        return NV_FALSE;

    if (!Max8907I2cRead8(hDevice, pSupplyInfo->OutputVoltageRegAddr, &data))
        return NV_FALSE;

    data &= MAX8907_OUT_VOLTAGE_MASK;
    if (!data) //OFF
        milliVolts = ODM_VOLTAGE_OFF;
    else
        milliVolts = pSupplyInfo->GetVoltage(data);

    ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail] = milliVolts;
    *pMilliVolts = milliVolts;
    return NV_TRUE;
}

//20100413, , trace info [START]
static NvU32 currentPowerState = 0;
static void PMUStateDump(NvOdmPmuDeviceHandle hDevice)
{
    NvU32 MilliVolts[30], i;
    return;

    Max8907ReadVoltageReg(hDevice, 1,&MilliVolts[1]);
    Max8907ReadVoltageReg(hDevice, 2,&MilliVolts[2]);
    Max8907ReadVoltageReg(hDevice, 3,&MilliVolts[3]);
    printk("V1(%4dmA)V2(%4dmA)V3(%4dmA)\n",
                MilliVolts[1],MilliVolts[2],MilliVolts[3]);
    for(i=Max8907PmuSupply_LDO1;i<=Max8907PmuSupply_LDO20;i++){
        Max8907ReadVoltageReg(hDevice, i,&MilliVolts[i]);
    }
    printk("L1(%dmA)L2(%dmA)L3(%dmA)L4(%dmA)L5(%dmA)\n",
                MilliVolts[Max8907PmuSupply_LDO1],
                MilliVolts[Max8907PmuSupply_LDO2],
                MilliVolts[Max8907PmuSupply_LDO3],
                MilliVolts[Max8907PmuSupply_LDO4],
                MilliVolts[Max8907PmuSupply_LDO5]);
    printk("L6(%dmA)L7(%dmA)L8(%dmA)L9(%dmA)L10(%dmA)\n",
                MilliVolts[Max8907PmuSupply_LDO6],
                MilliVolts[Max8907PmuSupply_LDO7],
                MilliVolts[Max8907PmuSupply_LDO8],
                MilliVolts[Max8907PmuSupply_LDO9],
                MilliVolts[Max8907PmuSupply_LDO10]);
    printk("L11(%dmA)L12(%dmA)L13(%dmA)L14(%dmA)L15(%dmA)\n",
                MilliVolts[Max8907PmuSupply_LDO11],
                MilliVolts[Max8907PmuSupply_LDO12],
                MilliVolts[Max8907PmuSupply_LDO13],
                MilliVolts[Max8907PmuSupply_LDO14],
                MilliVolts[Max8907PmuSupply_LDO15]);
    printk("L16(%dmA)L17(%dmA)L18(%dmA)L19(%dmA)L20(%dmA)\n",
                MilliVolts[Max8907PmuSupply_LDO16],
                MilliVolts[Max8907PmuSupply_LDO17],
                MilliVolts[Max8907PmuSupply_LDO18],
                MilliVolts[Max8907PmuSupply_LDO19],
                MilliVolts[Max8907PmuSupply_LDO20]);
}
//20100413, , trace info [END]

extern NvU32 Accel_PRail; 

static NvBool
Max8907WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32  MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 SettleUS = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        //if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1)
        if (((((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1))
			|| (vddRail == Max8907PmuSupply_LDO8) || (vddRail == Max8907PmuSupply_LDO7) 
			|| (Accel_PRail == vddRail))  
        {
            // turn off the supply
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

            // Disable the output (read-modify-write the control register)
            Max8907I2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data);
            data &= (~MAX8907_OUT_VOLTAGE_CONTROL_MASK);
            data |= MAX8907_OUT_VOLTAGE_CONTROL_DISABLE;

// 20110125  fix sensor LDO off [START] 				
            if ( (vddRail == Max8907PmuSupply_LDO8) || (vddRail == Max8907PmuSupply_LDO7) )			
	     	data |= 0x2;	// enable OUTPUT SHUTDOWN DISCHARGE REGISTER
// 20110125  fix sensor LDO off [END] 	

            if (!Max8907I2cWrite8(hDevice, pSupplyInfo->ControlRegAddr, data))
                return NV_FALSE;

            ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail] =
                ODM_VOLTAGE_OFF;
            SettleUS = MAX8907_TURN_OFF_TIME_US;
            //20100603, , power log [START]
            #if 0 //def CONFIG_LPRINTK
            currentPowerState &= ~(0x1 << vddRail);
            PMUStateDump(hDevice);
            if(vddRail>=Max8907PmuSupply_LDO1)
                lprintk(D_POWER, "OFF ===== LDO%d(%dmA)====\n",
                        (int)(vddRail-Max8907PmuSupply_LDO1+1),
                        (int)ODM_VOLTAGE_OFF);
            else
                lprintk(D_POWER, "OFF ===== SDV%d(%dmA)====\n",
                        (int)vddRail, (int)ODM_VOLTAGE_OFF);
            #endif
            //20100603, , power log [END]
        }

        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] != 0)
            ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] --;

        if (pSettleMicroSeconds)
            *pSettleMicroSeconds = SettleUS;
        else
            NvOdmOsWaitUS(SettleUS);

        return NV_TRUE;
    }

    //20100819 skip same voltage setting [START]
    if (MilliVolts == ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail])
        goto setDone;
    //20100819 skip same voltage setting [END]

    // Set voltage level
    data = pSupplyInfo->SetVoltage(MilliVolts);
    if (!Max8907I2cWrite8(hDevice, pSupplyInfo->OutputVoltageRegAddr, data))
        return NV_FALSE;
    if (MilliVolts >
        ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail])
    {
        NvU32 LastMV =
            ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail];
        SettleUS = 1 + (MilliVolts - LastMV) * 1000 / MAX8907_SCALE_UP_UV_PER_US;
    }
    ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail] = MilliVolts;

    // turn on supply
    //if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)
    if (((((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)) 
		|| (vddRail == Max8907PmuSupply_LDO8) || (vddRail == Max8907PmuSupply_LDO7) 
		|| (Accel_PRail == vddRail))
    {
        // Enable the output (read-modify-write the control register)
        Max8907I2cRead8(hDevice, pSupplyInfo->ControlRegAddr, &data);

        if ((data & MAX8907_OUT_VOLTAGE_CONTROL_MASK) ==
            MAX8907_OUT_VOLTAGE_CONTROL_DISABLE)
        {
            // Voltage on/change (supply was off, so it must be turned on)
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)(hDevice->pPrivate))->hOdmPmuSevice,
                pSupplyInfo->supply, NV_TRUE);
            data |= MAX8907_OUT_VOLTAGE_ENABLE_BIT;
            if (!Max8907I2cWrite8(hDevice, pSupplyInfo->ControlRegAddr, data))
                return NV_FALSE;

            SettleUS = MAX8907_TURN_ON_TIME_US;
        }
        //20100603, , power log [START]
        #if 0 //def CONFIG_LPRINTK 
        currentPowerState |= (0x1 << vddRail);
        PMUStateDump(hDevice);
        if(vddRail>=Max8907PmuSupply_LDO1)
            lprintk(D_POWER, "ON ===== LDO%d(%dmA)====\n",
                    vddRail-Max8907PmuSupply_LDO1+1,
                    (int)data);
        else
            lprintk(D_POWER, "ON ===== SDV%d(%dmA)====\n",
                    vddRail, (int)data);
        #endif
        //20100603, , power log [END]
    }

//20100819 skip same voltage setting [START]
setDone:
//20100819 skip same voltage setting [END]

    if(((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] < 0x1fffffff)
    ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] ++;

    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = SettleUS;
    else
        NvOdmOsWaitUS(SettleUS);

    return NV_TRUE;
}

static NvBool
Max8907OnOffConfigure(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data = 0;
    
#ifdef CONFIG_MACH_STAR
    // verison 
    //data |= (MAX8907_SYSENSEL_RSTINEN_MASK  << MAX8907_SYSENSEL_RSTINEN_SHIFT);
    data |= (MAX8907_SYSENSEL_WKCHG_MASK    << MAX8907_SYSENSEL_WKCHG_SHIFT);
    data |= (MAX8907_SYSENSEL_WKSW_MASK     << MAX8907_SYSENSEL_WKSW_SHIFT);
#else
    if (!Max8907I2cRead8(hDevice, MAX8907_SYSENSEL, &data))
        return NV_FALSE;

    // Enable hard reset - power-off after ONKEY press for 5 seconds
    // (must be enabled for thermal auto-shutdown)
    data |= (MAX8907_SYSENSEL_HRDSTEN_MASK <<
             MAX8907_SYSENSEL_HRDSTEN_SHIFT);
#endif

    return Max8907I2cWrite8(hDevice, MAX8907_SYSENSEL, data);
}
//20100413, , configuration for I2C mode of LDOs [START]
#if defined(CONFIG_MACH_STAR)

//20100427, , SMPL [START]
#define SMPL_TIME           MAX8907_MPL_TIME_0_5_SEC

static NvBool
Max8907SetSMPL(
    NvOdmPmuDeviceHandle hDevice)
{
    NvU8 CtlData;

    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ1, &CtlData)){        
        return NV_FALSE;
    }
    if (!Max8907I2cRead8(hDevice, MAX8907_CHG_IRQ2, &CtlData)){        
        return NV_FALSE;
    } 
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ1, &CtlData)){        
        return NV_FALSE;
    }
    if (!Max8907I2cRead8(hDevice, MAX8907_ON_OFF_IRQ2, &CtlData)){        
        return NV_FALSE;
    } 
    if (!Max8907I2cRead8(hDevice, MAX8907_RTC_IRQ, &CtlData)){        
        return NV_FALSE;
    } 

    //BBATT_CNFG : 3V
    if (!Max8907I2cRead8(hDevice, MAX8907_BBATT_CNFG, &CtlData))
        return NV_FALSE;
    CtlData |= 0x3;
    if (!Max8907I2cWrite8(hDevice, MAX8907_BBATT_CNFG, CtlData))
        return NV_FALSE;

    // MAXIM recommend : write 0x00 before setting value.
    CtlData = 0x00;
    if (!Max8907RtcI2cWrite8(hDevice, MAX8907_MPL_CNTL, CtlData))
        return NV_FALSE;
    
    CtlData |= (SMPL_TIME << MAX8907_MPL_TIME_SHIFT );
    CtlData |= (MAX8907_MPL_EN_MASK << MAX8907_MPL_EN_SHIFT );
    if (!Max8907RtcI2cWrite8(hDevice, MAX8907_MPL_CNTL, CtlData))
        return NV_FALSE;
        
    return NV_TRUE;
}
//20100427, , SMPL [END]

//20100518, , OUT5V VBUS enable / OUT3.3V disable [START]
static NvBool
Max8907SetVchgLDO(
    NvOdmPmuDeviceHandle hDevice)
{

    //OUT5.0
    //by VCHG detection
    Max8907I2cWrite8(hDevice, MAX8907_OUT5VEN, 0x10); 
    //OUT3.3
    Max8907I2cWrite8(hDevice, MAX8907_OUT_3_3VEN, 0x0E);
    
    return NV_TRUE;
}
//20100518, , ... [END]


static NvBool
Max8907SetI2CControlMode(
    NvOdmPmuDeviceHandle hDevice,
    Max8907PmuSupply Supply)
{
    NvU8 CtlAddr, CtlData, CntAddr, SeqSel;

    
    //CntData = MAX8907_SEQCNT_DEFAULT_LDO;
    SeqSel = MAX8907_SEQSEL_I2CEN_LXX;
    
    CtlAddr = Max8907SupplyInfoTable[Supply].ControlRegAddr;
    CntAddr = Max8907SupplyInfoTable[Supply].SequencerCountRegAddr;

    // Read control refgister, and select target sequencer
    if (!Max8907I2cRead8(hDevice, CtlAddr, &CtlData))
        return NV_FALSE;
    CtlData &= (~(MAX8907_CTL_SEQ_MASK << MAX8907_CTL_SEQ_SHIFT ));
    CtlData |= ((SeqSel & MAX8907_CTL_SEQ_MASK) << MAX8907_CTL_SEQ_SHIFT );

    // reset control => reset count
    if (!Max8907I2cWrite8(hDevice, CtlAddr, CtlData))
        return NV_FALSE;
    //if (!Max8907I2cWrite8(hDevice, CntAddr, CntData))
    //    return NV_FALSE;
    
    return NV_TRUE;
}

static NvBool
Max8907WhiteLEDSwitch(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 data)
{
    NvU8 CtlData;

    if(data){
        //CtlData = 0x00;
        //if (!Max8907I2cWrite8(hDevice, MAX8907_WLED_MODE_CNTL, CtlData))
        //    return NV_FALSE;

        //07MHz
        CtlData = 0x01; 
        if (!Max8907I2cWrite8(hDevice, MAX8907_WLED_MODE_CNTL, CtlData))
            return NV_FALSE;

        if(data > 100 )     /*10.0mA*/
            CtlData = 100;  /*10.0mA*/
        else
            CtlData = data;
        if (!Max8907I2cWrite8(hDevice, MAX8907_ILED_CNTL, CtlData))
            return NV_FALSE;
    }else{
        CtlData = 0x00;
        if (!Max8907I2cWrite8(hDevice, MAX8907_ILED_CNTL, CtlData))
            return NV_FALSE;
        
        CtlData = 0x00;
        if (!Max8907I2cWrite8(hDevice, MAX8907_WLED_MODE_CNTL, CtlData))
            return NV_FALSE;
    }        
    return NV_TRUE;
}

#endif
//20100413, , configuration for I2C mode of LDOs [END]

static NvBool
Max8907PwrEnConfigure(NvOdmPmuDeviceHandle hDevice, NvBool Enable)
{
    NvU8 data = 0;

    if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
        return NV_FALSE;

    // Enable/disable PWREN h/w control mechanism (PWREN signal must be
    // inactive = high at this time)
    if (Enable)
        data |= (MAX8907_RESET_CNFG_PWREN_EN_MASK <<
                 MAX8907_RESET_CNFG_PWREN_EN_SHIFT);
    else
        data &= (~(MAX8907_RESET_CNFG_PWREN_EN_MASK <<
                   MAX8907_RESET_CNFG_PWREN_EN_SHIFT));
    if (!Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data))
        return NV_FALSE;

    // When enabled, connect PWREN to SEQ2 by clearing SEQ2 configuration
    // settings for silicon revision that requires s/w WAR. On other MAX8907
    // revisions PWREN is always connected to SEQ2.
    if (Enable)
    {
        if (!Max8907I2cRead8(hDevice, MAX8907_II2RR, &data))
            return NV_FALSE;

        if (data == MAX8907_II2RR_PWREN_WAR)
        {
            data = 0x00;
            if (!Max8907I2cWrite8(hDevice, MAX8907_SEQ2CNFG, data))
                return NV_FALSE;
        }
    }
    return NV_TRUE;
}

static NvBool
Max8907PwrEnAttach(
    NvOdmPmuDeviceHandle hDevice,
    Max8907PmuSupply Supply,
    NvBool Attach)
{
    static NvU8 s_ResumeSeqSelLdo4 = MAX8907_SEQSEL_DEFAULT_LDO4;

    NvU8 CtlAddr, CtlData, CntAddr, CntData, SeqSel;

    switch (Supply)
    {
        #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION

        #else
        case Max8907PmuSupply_LX_V1:   // CPU
            // No sequencer delay for CPU rail when it is attached
            CntData = Attach ?  0x00 : MAX8907_SEQCNT_DEFAULT_LX_V1;
            SeqSel = Attach ? MAX8907_SEQSEL_PWREN_LXX :
                              MAX8907_SEQSEL_DEFAULT_LX_V1;
            break;
        #endif
        case Max8907PmuSupply_LX_V2:   // Core
            // Change CPU sequencer delay when core is attached to assure
            // order of Core/CPU rails control; clear CPU delay when core
            // is detached
            CntAddr = Max8907SupplyInfoTable[
                Max8907PmuSupply_LX_V1].SequencerCountRegAddr;
            CntData = Attach ?  MAX8907_SEQCNT_PWREN_LX_V1 : 0x00;
            if (!Max8907I2cWrite8(hDevice, CntAddr, CntData))
                return NV_FALSE;
			
#if WAR_AVDD_USB_EARLY_PWR_UP
            // Attach USB rail to sequencer 2 as well, when core is attached
            // (= entry to LP0), and dettach it respectively.
            if (!Max8907PwrEnAttach(hDevice, Max8907PmuSupply_LDO4, Attach))
                return NV_FALSE;
#endif
            CntData = Attach ?  MAX8907_SEQCNT_PWREN_LX_V2 :
                                MAX8907_SEQCNT_DEFAULT_LX_V2;
            SeqSel = Attach ? MAX8907_SEQSEL_PWREN_LXX :
                              MAX8907_SEQSEL_DEFAULT_LX_V2;
            break;

        case Max8907PmuSupply_LDO4:    // USB
            CntData = Attach ?  MAX8907_SEQCNT_PWREN_LD04 :
                                MAX8907_SEQCNT_DEFAULT_LDO4;
            SeqSel = Attach ? MAX8907_SEQSEL_PWREN_LXX :
                              s_ResumeSeqSelLdo4;
            break;

        default:
            NV_ASSERT(!"This supply must not be attached to PWREN");
            return NV_FALSE;
    }
    CtlAddr = Max8907SupplyInfoTable[Supply].ControlRegAddr;
    CntAddr = Max8907SupplyInfoTable[Supply].SequencerCountRegAddr;

    // Read control refgister, and select target sequencer
    if (!Max8907I2cRead8(hDevice, CtlAddr, &CtlData))
        return NV_FALSE;

    if (Supply == Max8907PmuSupply_LDO4)
    {
        NvU8 seq = (CtlData >> MAX8907_CTL_SEQ_SHIFT) & MAX8907_CTL_SEQ_MASK;
        //NvOdmOsPrintf("[NVODM PMU] AVDD_USB switching sequencer from %d to %d,"
        //                " (control was = 0x%x)\n", seq, SeqSel, CtlData);
#if !WAR_AVDD_USB_RESUME_KEEP_ON
        if (Attach)
            s_ResumeSeqSelLdo4 = seq;  // save to restore on LP0 resume
#endif
    }

    CtlData &= (~(MAX8907_CTL_SEQ_MASK << MAX8907_CTL_SEQ_SHIFT ));
    CtlData |= ((SeqSel & MAX8907_CTL_SEQ_MASK) << MAX8907_CTL_SEQ_SHIFT );

    // Attach: set count => set control
    // Dettach: reset control => reset count
    if (Attach)
    {
        if (!Max8907I2cWrite8(hDevice, CntAddr, CntData))
            return NV_FALSE;
    }

    if (!Max8907I2cWrite8(hDevice, CtlAddr, CtlData))
        return NV_FALSE;

    if (!Attach)
    {
        if (!Max8907I2cWrite8(hDevice, CntAddr, CntData))
            return NV_FALSE;
    }
    return NV_TRUE;
}

//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
static NvBool
Tca6416ConfigPort(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvBool Enable)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU32 PortNo;
    NvU32 PinNo;

    // Get port number and pin number
    PortNo = pSupplyInfo->OutputPort;
    PinNo = pSupplyInfo->PmuGpio;

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

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
            ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

        // Configure port pin as output
        if (!Tca6416ConfigPortPin(hDevice, PortNo, PinNo, GpioPinMode_Output))
            return NV_FALSE;

        // Set the output port (for disable, data = 0)
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_Low))
            return NV_FALSE;
    }
    return NV_TRUE;
}


#if defined(MAX8907_USE_FAN5355_VOLTAGE_SCALING)
static NvBool
Fan5355ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 milliVolts = 0;

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

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
    const Max8907PmuSupplyInfo *pSupplyInfo = NULL;
    NvU8 data = 0;

    if ((vddRail == Max8907PmuSupply_EXT_DCDC_3_USB1) ||
            (vddRail == Max8907PmuSupply_EXT_DCDC_3_USB3))
    {
        vddRail = Max8907PmuSupply_EXT_DCDC_3;
    }

    pSupplyInfo = &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    // TO DO: Account for reference counting
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        {
            // turn off the supply
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

            // Disable the output
            if (!Tca6416ConfigPort(hDevice, vddRail, NV_FALSE))
                return NV_FALSE;
        }
    }
    else
    {
        // Voltage on/change
        NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)(hDevice->pPrivate))->hOdmPmuSevice, pSupplyInfo->supply, NV_TRUE);

        // Set voltage level
        data = pSupplyInfo->SetVoltage(MilliVolts);
#if defined(MAX8907_USE_FAN5355_VOLTAGE_SCALING)
        if (!Fan5355I2cWrite8(hDevice, pSupplyInfo->OutputVoltageRegAddr, data))
            return NV_FALSE;
#endif

        // Enable the output
        if (!Tca6416ConfigPort(hDevice, vddRail, NV_TRUE))
            return NV_FALSE;
    }
    return NV_TRUE;
}
#endif

#ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
static NvBool
Max8952ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU8 data = 0;
    NvU32 milliVolts = 0;

	
    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);
	//NVODMPMU_PRINTF(("Max8952ReadVoltageReg: vddRail = %d \n", vddRail));
	
    if (pSupplyInfo->OutputVoltageRegAddr == MAX8952_REG_INVALID)
        return NV_FALSE;

    if (!Max8952I2cRead8(hDevice, pSupplyInfo->OutputVoltageRegAddr, &data))
        return NV_FALSE;

    if (!data) //OFF
        milliVolts = 0;
    else
        milliVolts = pSupplyInfo->GetVoltage(data);

    *pMilliVolts = milliVolts;
    return NV_TRUE;
}

static NvBool
Max8952WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32  MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = NULL;
    NvU8 data = 0;
    NvU32 SettleUS = 0;
    
    NvU32 LastMV;
    
    Max8952ReadVoltageReg(hDevice, vddRail, &LastMV);

    pSupplyInfo = &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    // TO DO: Account for reference counting
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
        //printk(("ERROR Max8952WriteVoltageReg: MAX8952 cannot be turned off by I2C. \n"));
		return NV_FALSE;
        // MAX8952 is directly connected to BUCK1
        #else
        // check if the supply can be turned off
        {
            // turn off the supply
            NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE);

            // Disable the output
            if (!Tca6416ConfigPort(hDevice, vddRail, NV_FALSE))
                return NV_FALSE;
        }
        #endif
    }
    else
    {
        // Voltage on/change
        NvOdmServicesPmuSetSocRailPowerState(
                ((Max8907PrivData*)(hDevice->pPrivate))->hOdmPmuSevice, pSupplyInfo->supply, NV_TRUE);

        //20100819 skip same voltage setting [START]
        if (MilliVolts == LastMV)
            goto setDone;
        //20100819 skip same voltage setting [END]

        // Set voltage level
        #if defined(TMUS_B)   //TMUS revB
        //20100819, , Voltage bug fix
        data = pSupplyInfo->SetVoltage(MilliVolts) | MAX8952_FPWM_EN0;
        #else 
        data = pSupplyInfo->SetVoltage(MilliVolts);
        #endif

        if (!Max8952I2cWrite8(hDevice, pSupplyInfo->OutputVoltageRegAddr, data))
            return NV_FALSE;

        //20100811 CPU power patch [START]
        if (MilliVolts > LastMV)
        {
            SettleUS = 1 + (MilliVolts - LastMV) * 1000 / MAX8952_SCALE_UP_UV_PER_US;
        }
		
        ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[vddRail] = MilliVolts;
        //20100811 CPU power patch [END]
		
        #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
        // MAX8952 is directly connected to BUCK1
        #else
        // Enable the output
        if (!Tca6416ConfigPort(hDevice, vddRail, NV_TRUE))
            return NV_FALSE;
        #endif
    }

//20100819 skip same voltage setting [START]
setDone:
//20100819 skip same voltage setting [END]

    //20100811 CPU power patch [START]
    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = SettleUS;
    else
        NvOdmOsWaitUS(SettleUS);
    //20100811 CPU power patch [END]
	
    return NV_TRUE;
}
#endif

#ifndef CONFIG_MACH_STAR
static NvBool
Max8907LxV1Ad5258ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    const Max8907PmuSupplyInfo* pSupplyInfo =
        &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    // Check if DC/DC has been turned Off (controlled by LxV1 main PMU output)
    if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] == 0)
    {
        if (!Max8907ReadVoltageReg(
            hDevice, Max8907PmuSupply_LX_V1, pMilliVolts))
            return NV_FALSE;
        if (*pMilliVolts == ODM_VOLTAGE_OFF)
            return NV_TRUE;
    }

    // DC/DC is On - now get DPM-scaled voltage
    return Ad5258I2cGetVoltage(hDevice, pMilliVolts);
}

static NvBool
Max8907LxV1Ad5258WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    const Max8907PmuSupplyInfo *pSupplyInfo =
        &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // Check if the supply can be turned off, and if yes - turn off
        // LxV1 main PMU output, which controls DC/DC
        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] == 1)
        {
            if (!Max8907WriteVoltageReg(hDevice, Max8907PmuSupply_LX_V1,
                                         MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] != 0)
        {
            ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
            pSupplyInfo->supply] --;
        }
        return NV_TRUE;
    }

    // Set DPM voltage level (includes DPM and DCDC change level settle time)
    if (!Ad5258I2cSetVoltage(hDevice, MilliVolts))
        return NV_FALSE;

    // Turn on control LxV1 supply on main PMU if necessary
    if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] == 0)
    {
        if (!Max8907WriteVoltageReg(hDevice, Max8907PmuSupply_LX_V1,
                MAX8907_REQUESTVOLTAGE_LX_V1, pSettleMicroSeconds))
            return NV_FALSE;

        // Add external DCDC turning On settling time
        if (pSettleMicroSeconds)
            *pSettleMicroSeconds += AD5258_TURN_ON_TIME_US;
        else
            NvOdmOsWaitUS(AD5258_TURN_ON_TIME_US);
    }
    ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[
        pSupplyInfo->supply] ++;

    return NV_TRUE;
}
#endif
//20100413, , unused [END]

NvBool
Max8907Setup(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmIoModule I2cModule = NvOdmIoModule_I2c;
    NvU32  I2cInstance = 0;
    NvU32  I2cAddress  = 0;
    NvU32  i           = 0;
    const NvOdmPeripheralConnectivity *pConnectivity =
                           NvOdmPeripheralGetGuid(PMUGUID);

    NV_ASSERT(hDevice);


    hMax8907Pmu = (Max8907PrivData*) NvOdmOsAlloc(sizeof(Max8907PrivData));
    if (hMax8907Pmu == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating Max8907PrivData.\n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hMax8907Pmu, 0, sizeof(Max8907PrivData));
    hDevice->pPrivate = hMax8907Pmu;

    ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable =
        NvOdmOsAlloc(sizeof(NvU32) * Max8907PmuSupply_Num);
    if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating RefCntTable. \n"));
        goto fail;
    }
    ((Max8907PrivData*)hDevice->pPrivate)->pVoltages =
        NvOdmOsAlloc(sizeof(NvU32) * Max8907PmuSupply_Num);
    if (((Max8907PrivData*)hDevice->pPrivate)->pVoltages == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating shadow voltages table. \n"));
        goto fail;
    }

    // memset
    for (i = 0; i < Max8907PmuSupply_Num; i++)
    {
        ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[i] = 0;
        // Setting shadow to 0 would cause spare delay on the 1st scaling of
        // always On rail; however the alternative reading of initial settings
        // over I2C is even worse.
        ((Max8907PrivData*)hDevice->pPrivate)->pVoltages[i] = 0;
    }

    //20110131, , Stop i2c comm during reset 
    stop_i2c_flag = NV_FALSE;

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

        ((Max8907PrivData*)hDevice->pPrivate)->hOdmI2C = NvOdmI2cOpen(I2cModule, I2cInstance);
        if (!((Max8907PrivData*)hDevice->pPrivate)->hOdmI2C)
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: Error Opening I2C device. \n"));
            NVODMPMU_PRINTF(("[NVODM PMU]Please check PMU device I2C settings. \n"));
            return NV_FALSE;
        }
        ((Max8907PrivData*)hDevice->pPrivate)->DeviceAddr = I2cAddress;
        ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice = NvOdmServicesPmuOpen();
    }
    else
    {
        // if PMU is not present in the database, then the platform is PMU-less.
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: The system did not doscover PMU from the data base.\n"));
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: If this is not intended, please check the peripheral database for PMU settings.\n"));
        return NV_FALSE;
    }

    // Configure OnOff options
    if (!Max8907OnOffConfigure(hDevice))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: Max8907OnOffConfigure() failed. \n"));
        return NV_FALSE;
    }

    //20100413, , configuration [START]
    #if defined(CONFIG_MACH_STAR)    
    //20100427, , SMPL
    Max8907SetSMPL(hDevice);

    //20100518, , OUT5V VCHG detect / OUT3.3V diable
    Max8907SetVchgLDO(hDevice);    
    #endif
    //20100413, , configuration [END]

    // Configure PWREN, and attach CPU V1 rail to it.
    // TODO: h/w events (power cycle, reset, battery low) auto-disables PWREN.
    // Only soft reset (not supported) requires s/w to disable PWREN explicitly
    if (!Max8907PwrEnConfigure(hDevice, NV_TRUE))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: Max8907PwrEnConfigure() failed. \n"));
        return NV_FALSE;
    }
    #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
	// BUCK1 reserved to DDR-SDRAM power.
    #else
    if (!Max8907PwrEnAttach(hDevice, Max8907PmuSupply_LX_V1, NV_TRUE))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: Max8907PwrEnAttach() failed. \n"));
        return NV_FALSE;
    }
    #endif

    //20100622, , ADC Setting for BAT voltage, temperature read [START]
    #if defined(CONFIG_MACH_STAR)
    if (!Max8907AdcSetup(hDevice))
    {
        LDP("[eidola] ADC Setup fail!!");
        return NV_FALSE;
    }
    LDP("[eidola] ADC Setup Done!");
    #endif
    //20100622, , ADC Setting for BAT voltage, temperature read [END]

    //Check battery presence
    if (!Max8907BatteryChargerMainBatt(hDevice,&((Max8907PrivData*)hDevice->pPrivate)->battPresence))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: Max8907BatteryChargerMainBatt() failed. \n"));
        return NV_FALSE;
    }
    else {LDP("BatteryChargerMainBatt(TRUE:%d)", ((Max8907PrivData*)hDevice->pPrivate)->battPresence);}

#if 0 
    if(!Max8907SetVoltage(hDevice, Max8907PmuSupply_LDO7, 3000, NULL))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: set voltage error\n"));
    }
    if(!Max8907SetVoltage(hDevice, Max8907PmuSupply_LDO8, 1800, NULL))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup: set voltage error\n"));
    }
#endif

    //emmc : for voltage margin
    Max8907SetVoltage(hDevice, Max8907PmuSupply_LDO5, 3000, NULL);

    //20100928, , enable PMU interrupt for RTC wakeup [START]
    Max8907SetupInterrupt(hDevice);
    //20100928, , enable PMU interrupt for RTC wakeup [END]
    
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
                    Max8907SupplyInfoTable[vddRail].cap.requestMilliVolts;
                if(!Max8907SetVoltage(hDevice, vddRail, mv, NULL))
                {
                    NVODMPMU_PRINTF(("[NVODM PMU]Max8907Setup:"
                                     " Thermal rail setup failed. \n"));
                }
            }
        }
    }

    // The interrupt assumes not supported until Max8907InterruptHandler() is called.
    ((Max8907PrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_FALSE;

    return NV_TRUE;

fail:
    Max8907Release(hDevice);
    return NV_FALSE;
}

void
Max8907Release(NvOdmPmuDeviceHandle hDevice)
{
    if (hDevice->pPrivate != NULL)
    {
        if (((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice != NULL)
        {
            NvOdmServicesPmuClose(((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice);
            ((Max8907PrivData*)hDevice->pPrivate)->hOdmPmuSevice = NULL;
        }

        if (((Max8907PrivData*)hDevice->pPrivate)->hOdmI2C != NULL)
        {
            NvOdmI2cClose(((Max8907PrivData*)hDevice->pPrivate)->hOdmI2C);
            ((Max8907PrivData*)hDevice->pPrivate)->hOdmI2C = NULL;
        }

        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable != NULL)
        {
            NvOdmOsFree(((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable);
            ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable = NULL;
        }

        if (((Max8907PrivData*)hDevice->pPrivate)->pVoltages != NULL)
        {
            NvOdmOsFree(((Max8907PrivData*)hDevice->pPrivate)->pVoltages);
            ((Max8907PrivData*)hDevice->pPrivate)->pVoltages = NULL;
        }


        NvOdmOsFree(hDevice->pPrivate);
        hDevice->pPrivate = NULL;
    }
}

NvBool
Max8907GetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(pMilliVolts);
    NV_ASSERT(vddRail < Max8907PmuSupply_Num);

#ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
    if (vddRail == Max8907PmuSupply_EXT_DCDC_8_CPU)
    {
        if (!Max8952ReadVoltageReg(hDevice, vddRail, pMilliVolts)){
			//printk("ERROR Max8907GetVoltage: I2C read failed \n");
            return NV_FALSE;
        }
	}
    // RTC is a special case, since it's "always on"
    else 
#endif
    if (vddRail == Max8907PmuSupply_VRTC)
    {
        // Fixed voltage
        *pMilliVolts = MAX8907_MAX_OUTPUT_VOLTAGE_RTC;
    }
    //20100413, , unused [START]
    #ifndef CONFIG_MACH_STAR
    else if (vddRail == Max8907PmuSupply_EXT_DCDC_1)
    {
        // Fixed voltage
        *pMilliVolts = MAX8907_REQUESTVOLTAGE_EXT_DCDC_1;
    }
    else if (vddRail == Max8907PmuSupply_EXT_DCDC_3)
    {
#if defined(MAX8907_USE_FAN5355_VOLTAGE_SCALING)
        if (!Fan5355ReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
#else
        // Fixed voltage
        *pMilliVolts = MAX8907_REQUESTVOLTAGE_EXT_DCDC_3;
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
    else if (vddRail == Max8907LxV1_Ad5258_DPM_EXT_DCDC_7)
    {
#ifdef FEATURE_MAX8907

#else
        // External DCDC controlled by LX_V1, and scaled by DPM
        if (!Max8907LxV1Ad5258ReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
#endif
		*pMilliVolts = 0;
    }
    #endif
    //20100413, , unused [END]
    else
    {
        if (!Max8907ReadVoltageReg(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
    }
	//NVODMPMU_PRINTF(("Max8907GetVoltage: vddRail = %d, Volts = %d\n", vddRail, *pMilliVolts ));
	
    return NV_TRUE;
}

//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
static  NvBool
Tca6416UsbVbusControl(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
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

static NvBool
Tca6416FuseControl(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts)
{
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];
    NvU32 PortNo;
    NvU32 PinNo;

    // Get port number and pin number
    PortNo = pSupplyInfo->OutputPort;
    PinNo = pSupplyInfo->PmuGpio;

    // Configure port pin as output
    if (!Tca6416ConfigPortPin(hDevice, PortNo, PinNo, GpioPinMode_Output))
        return NV_FALSE;

    if (MilliVolts == ODM_VOLTAGE_OFF)  // to disable FUSE voltage
    {
        // Set Low  on pin
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_Low))
            return NV_FALSE;
    }
    else  // to Enable FUSE voltage
    {
        // Set high  on pin
        if (!Tca6416WritePortPin(hDevice, PortNo, PinNo, GpioPinState_High))
            return NV_FALSE;
    }
    return NV_TRUE;
}
#endif
//20100413, , unused [END]

//20100727, , change SEQ of LDO5 before power-off [LGE_START]
void
Max8907ChangeSEQ( NvOdmPmuDeviceHandle hDevice, NvU32 reg )
{
	NvU8 data;

	if (!Max8907I2cRead8(hDevice, reg, &data))
	{
		printk("[PowerOff] Max8907I2cRead8() error !!!" );
		return;
	}

	printk("[PowerOff] LDOCTL%d=0x%x (before) \n", reg/4-4, data);
	data &= ~(MAX8907_LDOCTL_SEQ_MASK <<
			 MAX8907_LDOCTL_SEQ_SHIFT);
	Max8907I2cWrite8(hDevice, reg, data);
	if (!Max8907I2cRead8(hDevice, reg, &data))
	{
		printk("[PowerOff] Max8907I2cRead8() error !!!" );
		return;
	}
	printk("[PowerOff] LDOCTL%d=0x%x (after) \n", reg/4-4, data);
}
//20100727, , change SEQ of LDO5 before power-off [LGE_END]

NvBool
Max8907SetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
	NvU32 VoltData = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(vddRail < Max8907PmuSupply_Num);

	//NVODMPMU_PRINTF(("Max8907SetVoltage(): vddRail = %d, Volts = %d \n", vddRail, MilliVolts));
    if ( Max8907SupplyInfoTable[vddRail].cap.OdmProtected == NV_TRUE)
    {
        NVODMPMU_PRINTF(("The voltage is protected and cannot be set.\n"));
        return NV_TRUE;
    }

    //20110131, , Stop i2c comm during reset [START]
    if ( vddRail == Max8907PmuSupply_Stop_i2c_Flag )    
    {
        printk("[I2C STOP] setting_value=%d, previous_value=%d \n", MilliVolts,  stop_i2c_flag);
        if( MilliVolts == 0 )
            stop_i2c_flag = NV_FALSE;
        else
            stop_i2c_flag = NV_TRUE;

        printk("[I2C STOP] set stop_i2c_flag = %d \n", stop_i2c_flag );
        return NV_TRUE;
    }
    //20110131, , Stop i2c comm during reset [END]


    #if defined(CONFIG_MACH_STAR)
    //20100413, , power off [START]
    if ( vddRail == Max8907PmuSupply_SOC && MilliVolts == ODM_VOLTAGE_OFF)
    {
        NvU8 data;

        //20100626, , Touch LED off
        Max8907WhiteLEDSwitch(hDevice, 0);

		VoltData = 1000;
        Max8952WriteVoltageReg(hDevice, Max8907PmuSupply_EXT_DCDC_8_CPU, VoltData, NULL);
        Max8907WriteVoltageReg(hDevice, Max8907PmuSupply_LX_V1, 1000, NULL);
        Max8907WriteVoltageReg(hDevice, Max8907PmuSupply_LX_V2, 1200, NULL);
        Max8907WriteVoltageReg(hDevice, Max8907PmuSupply_LX_V3, 1800, NULL);

        Max8907GetVoltage(hDevice, Max8907PmuSupply_EXT_DCDC_8_CPU, &VoltData);
        printk("[PowerOff] Voltage of Max8907PmuSupply_EXT_DCDC_8_CPU = %d  \n", VoltData);

        Max8907GetVoltage(hDevice, Max8907PmuSupply_LX_V1, &VoltData);
        printk("[PowerOff] Voltage of Max8907PmuSupply_LX_V1 = %d  \n", VoltData);
        Max8907GetVoltage(hDevice, Max8907PmuSupply_LX_V2, &VoltData);
        printk("[PowerOff] Voltage of Max8907PmuSupply_LX_V2 = %d  \n", VoltData);
        Max8907GetVoltage(hDevice, Max8907PmuSupply_LX_V3, &VoltData);
        printk("[PowerOff] Voltage of Max8907PmuSupply_LX_V3 = %d  \n", VoltData);

        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL20, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL19, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL18, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL17, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL16, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL15, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL14, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL13, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL12, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL11, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL10, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL9, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL8, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL7, 0x1C);
        Max8907I2cWrite8(hDevice, MAX8907_LDOCTL6, 0x1C);

        //20100727, , change SEQ of LDO5 before power-off
        Max8907ChangeSEQ( hDevice, MAX8907_LDOCTL5);
        Max8907ChangeSEQ( hDevice, MAX8907_LDOCTL4);
        Max8907ChangeSEQ( hDevice, MAX8907_LDOCTL3);
        Max8907ChangeSEQ( hDevice, MAX8907_LDOCTL2);
        Max8907ChangeSEQ( hDevice, MAX8907_LDOCTL1);
        //20100426, , PMIC off

        Max8907I2cWrite8(hDevice, MAX8907_SDCTL2, 0x02);
        Max8907I2cWrite8(hDevice, MAX8907_SDSEQCNT2, 0x2E);        
        
        if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
            return NV_FALSE;

        // sw reset 
        data |= (MAX8907_RESET_CNFG_SFT_RST_MASK <<
                 MAX8907_RESET_CNFG_SFT_RST_SHIFT);

        // Enable sw power off
        data |= (MAX8907_SYSENSEL_POWER_OFF_MASK <<
                 MAX8907_SYSENSEL_POWER_OFF_SHIFT);

        // disable pwren_en
        data &= ~(MAX8907_RESET_CNFG_PWREN_EN_SHIFT <<
                 MAX8907_RESET_CNFG_PWREN_EN_MASK);

        return Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data);
    }
    //20100413, , power off [END]

    //20100611, , Touch LED [START]
    if ( vddRail == Max8907PmuSupply_WHITE_LED )
    {
        NvU8 data = (NvU8)MilliVolts;
        
        return Max8907WhiteLEDSwitch(hDevice, data);
    }
    //20100611, , Touch LED [END]    

    //20100703, , PMIC reset [START]
    if ( vddRail == Max8907PmuSupply_reset && MilliVolts == ODM_VOLTAGE_OFF)
    {
        NvU8 data;

        printk("PMIC reset\n");

        //20100626, , Touch LED off
        Max8907WhiteLEDSwitch(hDevice, 0);
        
        //20100426, , PMIC reset
        if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
            return NV_FALSE;

        // Enable sw power off
        data |= (MAX8907_RESET_CNFG_SFT_RST_MASK<<
                 MAX8907_RESET_CNFG_SFT_RST_SHIFT);

        return Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data);
    }
    //20100703, , PMIC reset [END]
    #endif

#if 0
    if (vddRail == Max8907PmuSupply_VBAT_FUSE)
    {
        // Enable  fuse voltage
       if (!Tca6416FuseControl(hDevice, vddRail, MilliVolts))
            return NV_FALSE;

       return NV_TRUE;
    }
#endif

    if ((MilliVolts == ODM_VOLTAGE_ENABLE_EXT_ONOFF) ||
        (MilliVolts == ODM_VOLTAGE_DISABLE_EXT_ONOFF))
    {
        return Max8907PwrEnAttach(hDevice, (Max8907PmuSupply)vddRail,
            (MilliVolts == ODM_VOLTAGE_ENABLE_EXT_ONOFF));
    }

    if ((MilliVolts == ODM_VOLTAGE_OFF) ||
        ((MilliVolts <=  Max8907SupplyInfoTable[vddRail].cap.MaxMilliVolts) &&
         (MilliVolts >=  Max8907SupplyInfoTable[vddRail].cap.MinMilliVolts)))
    {
        #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
		if( vddRail == Max8907PmuSupply_EXT_DCDC_8_CPU )
		{
//20100811 CPU power patch [START]
            //if (!Max8952WriteVoltageReg(hDevice, vddRail, MilliVolts))
            if (!Max8952WriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
//20100811 CPU power patch [END]
                return NV_FALSE;
		}
        #endif
        //20100413, , unused [START]
        #ifndef CONFIG_MACH_STAR
		else if ((vddRail == Max8907PmuSupply_EXT_DCDC_1) ||
            (vddRail == Max8907PmuSupply_EXT_DCDC_3) ||
            (vddRail == Max8907PmuSupply_EXT_DCDC_3_USB1) ||
            (vddRail == Max8907PmuSupply_EXT_DCDC_3_USB3))
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
        else if (vddRail == Max8907LxV1_Ad5258_DPM_EXT_DCDC_7)
        {
#ifdef FEATURE_MAX8907

#else

            // External DCDC controlled by LX_V1, and scaled by DPM
            if (!Max8907LxV1Ad5258WriteVoltageReg(
                hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
#endif
        }
        #endif
        //20100413, , unused [END]
        #ifdef FEATURE_MAX8907C_MAX8952_COMBINATION
        else
        #endif
        {
            if (!Max8907WriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
                return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("The requested voltage is not supported. vddRail=%d, MilliVolts=%d\n"
                    , vddRail, MilliVolts));
        return NV_FALSE;
    }

    //20100413, , unused [START]
    #ifndef CONFIG_MACH_STAR   
    // Check whether need to enable VBUS for any of the USB Instance
    if ((vddRail == Max8907PmuSupply_EXT_DCDC_3_USB1) ||
        (vddRail == Max8907PmuSupply_EXT_DCDC_3_USB3))
    {
        // Enable  VBUS for USB1 or USB3
       if (!Tca6416UsbVbusControl(hDevice, vddRail, MilliVolts))
            return NV_FALSE;
    }
    #endif  
    //20100413, , unused [END]
    return NV_TRUE;
}

#if defined(CONFIG_MACH_STAR) 
//20100704  jongik's headset porting [LGE]
NvU32
Max8907GetHookAdc(
    NvOdmPmuDeviceHandle hDevice)
{
    NvU32 value;

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return 0;
    //20110131, , Stop i2c comm during reset [END]
	
    value = Max8907AdcHookAdcRead(hDevice);
    return value;
}

//20101121 , HW power off in thermal limit [START]
NvU32
Max8907SetHwPowerOffConfig(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable)
{   
    NvU8 data = 0;
    
    if (!Max8907I2cRead8(hDevice, MAX8907_SYSENSEL, &data))
        return NV_FALSE;

    if(Enable){
        // Enable hard reset - power-off after ONKEY press for 5 seconds
        // (must be enabled for thermal auto-shutdown)
        data |= (MAX8907_SYSENSEL_HRDSTEN_MASK <<
                 MAX8907_SYSENSEL_HRDSTEN_SHIFT);

        return Max8907I2cWrite8(hDevice, MAX8907_SYSENSEL, data);
    }else{
        data &= ~(MAX8907_SYSENSEL_HRDSTEN_MASK <<
                 MAX8907_SYSENSEL_HRDSTEN_SHIFT);

        return Max8907I2cWrite8(hDevice, MAX8907_SYSENSEL, data);
    }
}
//20101121 , HW power off in thermal limit [END]

#endif
void
Max8907GetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities)
{
    NV_ASSERT(pCapabilities);
    NV_ASSERT(vddRail < Max8907PmuSupply_Num);

    *pCapabilities = Max8907SupplyInfoTable[vddRail].cap;
}

NvBool
Max8907GetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus)
{
#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
	NvBool acLineStatus = NV_FALSE;
#endif // CONFIG_STAR_BATTERY_CHARGER

	NV_ASSERT(hDevice);
	NV_ASSERT(pStatus);

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return NV_FALSE;
    //20110131, , Stop i2c comm during reset [END]

#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
//20100907, , AcLine status function for bootloader Battery checker [START]
	if (!Max8907BatteryChargerOK(hDevice, &acLineStatus))
	{
		NVODMPMU_PRINTF(("[NVODM PMU] Max8907GetAcLineStatus: Error in checking main charger presence.\n"));
		return NV_FALSE;
	}

	if (acLineStatus == NV_TRUE)
		*pStatus = NvOdmPmuAcLine_Online;
	else
		*pStatus = NvOdmPmuAcLine_Offline;
	return NV_TRUE;
//20100907, , AcLine status function for bootloader Battery checker [END]
#else // Original Code
    // check if battery is present
    if (((Max8907PrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
    {
        *pStatus = NvOdmPmuAcLine_Online;
        return NV_TRUE;
    }

    if ( ((Max8907PrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE )
    {
        if ( ((Max8907PrivData*)hDevice->pPrivate)->pmuStatus.mChgPresent == NV_TRUE )
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
        if (!Max8907BatteryChargerOK(hDevice, &acLineStatus))
        {
            NVODMPMU_PRINTF(("[NVODM PMU] Max8907GetAcLineStatus: Error in checking main charger presence.\n"));
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
#endif // CONFIG_STAR_BATTERY_CHARGER
    return NV_TRUE;
}

NvBool
Max8907GetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus)
{
//20100529, , Code Change for STAR [START]
	NvOdmPmuAcLineStatus stat = NvOdmPmuAcLine_Offline;
	//NvBool batFull = NV_FALSE;
//20100529, , Code Change for STAR [END]

	NV_ASSERT(hDevice);
	NV_ASSERT(pStatus);
	NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return 0;
    //20110131, , Stop i2c comm during reset [END]

//20100529, , Code Change for STAR [START]
	//Check battery presence
	if (!Max8907BatteryChargerMainBatt(hDevice,&((Max8907PrivData*)hDevice->pPrivate)->battPresence))
	{
		LDP("Max8907BatteryChargerMainBatt() failed. \n");
		return NV_FALSE;
	}
	else {LDP("Max8907BatteryChargerMainBatt(TRUE:%d)", ((Max8907PrivData*)hDevice->pPrivate)->battPresence);}

	if (batteryInst == NvOdmPmuBatteryInst_Main)
	{
		if ( ((Max8907PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE )
		{
			if (!Max8907GetAcLineStatus(hDevice, &stat))
			    return NV_FALSE;

			if (stat == NvOdmPmuAcLine_Online)
			{
#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
				switch (get_charging_ic_status())
				{
					case CHG_IC_DEFAULT_MODE:
					case CHG_IC_USB_LO_MODE:
					case CHG_IC_TA_MODE:
					case CHG_IC_FACTORY_MODE:
						*pStatus = NVODM_BATTERY_STATUS_CHARGING;
						break;

					case CHG_IC_DEACTIVE_MODE:
						*pStatus = NVODM_BATTERY_STATUS_DISCHARGING;
						break;

					default:
						*pStatus = NVODM_BATTERY_STATUS_DISCHARGING;
						break;
				}
#else // Original Code
				if ( ((Max8907PrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE )
				{
					if ( ((Max8907PrivData*)hDevice->pPrivate)->pmuStatus.batFull == NV_FALSE )
						*pStatus = NVODM_BATTERY_STATUS_CHARGING;
				}
				else
				{
					NvBool batFull = NV_FALSE;
					if (!Max8907BatteryChargerMainBattFull(hDevice, &batFull))
						return NV_FALSE;
					if (batFull == NV_FALSE)
						*pStatus = NVODM_BATTERY_STATUS_CHARGING;
				}
#endif // CONFIG_STAR_BATTERY_CHARGER
			}
//20100529, , Battery presence && AC offline [START]
			else  // Battery presence && AC offline
			{
				// Not implemented yet by Nvidia
				// Get VBatSense <- Battery Voltage
				//if (!Max8907AdcVBatSenseRead(hDevice, &VBatSense))
				    //return NV_FALSE;

				// TO DO: Update status based on VBatSense
				//LDP("[jh.ahn] VMBATT  = %d", VBatSense);
				*pStatus = NVODM_BATTERY_STATUS_DISCHARGING;
			}
//20100529, , Battery presence && AC offline [END]
		}
		else // Battery is not present
		{
			/* Battery is actually not present */
			*pStatus = NVODM_BATTERY_STATUS_NO_BATTERY;
		}
	}
	else // (batteryInst != NvOdmPmuBatteryInst_Main)
	{
		*pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
	}
//20100529, , Code Change for STAR [END]
    return NV_TRUE;
}

NvBool
Max8907GetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData)
{
	NvOdmPmuBatteryData batteryData;
#if defined (CONFIG_MACH_STAR)
	NvU32 VBatSense = 0;
	NvU32 VBatTemp = 0;
#endif

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

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return NV_FALSE;
    //20110131, , Stop i2c comm during reset [END]

//20100529, , Write the description here in detail [START]
#if defined (CONFIG_MACH_STAR)
	batteryData.batteryCurrent = 10;
	batteryData.batteryMahConsumed = 500;

	//if (batteryInst == NvOdmPmuBatteryInst_Main)
	//{
		//if (((Max8907PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
		//{
			/* retrieve Battery voltage and temperature */

			// Get VBatSense
			if (!Max8907AdcVBatSenseRead(hDevice, &VBatSense))
			{
				LDP("Error reading VBATSense.");
				return NV_FALSE;
			}
			LDP("[jh.ahn] VMBATT  = %d", VBatSense);

			// Get VBatTemp
			if (!Max8907AdcVBatTempRead(hDevice, &VBatTemp))
			{
				LDP("Error reading VBatTempRead.");
				return NV_FALSE;
			}
			LDP("[jh.ahn] BatTemp[Rthm]  = %d", VBatTemp);

			batteryData.batteryVoltage = VBatSense;
			batteryData.batteryTemperature = Max8907BatteryTemperature(VBatSense, VBatTemp);
		//}

		*pData = batteryData;
	//}
#else // Original Code
	if (batteryInst == NvOdmPmuBatteryInst_Main)
	{
		NvU32 VBatSense = 0;
		NvU32 VBatTemp  = 0;

		if (((Max8907PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
		{
			/* retrieve Battery voltage and temperature */

			// Get VBatSense
			if (!Max8907AdcVBatSenseRead(hDevice, &VBatSense))
			{
				NVODMPMU_PRINTF(("Error reading VBATSense. \n"));
				return NV_FALSE;
			}

			// Get VBatTemp
			if (!Max8907AdcVBatTempRead(hDevice, &VBatTemp))
			{
				NVODMPMU_PRINTF(("Error reading VBatTempRead. \n"));
				return NV_FALSE;
			}

			batteryData.batteryVoltage = VBatSense;
			batteryData.batteryTemperature = Max8907BatteryTemperature(VBatSense, VBatTemp);
		}

		*pData = batteryData;
	}
#endif // CONFIG_MACH_STAR
//20100529, , Write the description here in detail [END]
/*	else
	{
		*pData = batteryData;
	}
*/
//20100529, , For Factory Power Test [START]
// No_battery & AC online state.... also need battery temperature for Battery detect
	//if ( 1 == ARRAY_TP_BOOT() ) // For Factory Power Test
	//{
		/* retrieve Battery voltage and temperature */
/*
		// Get VBatSense
		if (!Max8907AdcVBatSenseRead(hDevice, &VBatSense))
		{
			LDP("Error reading VBATSense.");
			return NV_FALSE;
		}
		LDP("[jh.ahn] VMBATT  = %d", VBatSense);

		// Get VBatTemp
		if (!Max8907AdcVBatTempRead(hDevice, &VBatTemp))
		{
			LDP("Error reading VBatTempRead.");
			return NV_FALSE;
		}
		LDP("[jh.ahn] BatTemp[Rthm]  = %d", VBatTemp);

		batteryData.batteryVoltage = VBatSense;
		batteryData.batteryTemperature = Max8907BatteryTemperature(VBatSense, VBatTemp);

		*pData = batteryData;
	//}
//20100915, , For Factory Power Test [END]
*/
    return NV_TRUE;
}

//20100924, , For updating battery information totally [START]
#if defined(CONFIG_MACH_STAR)
NvBool
Max8907UpdateBatteryInfo(
	NvOdmPmuDeviceHandle hDevice,
	NvOdmPmuAcLineStatus *pAcStatus,
	NvU8 *pBatStatus,
	NvOdmPmuBatteryData *pBatData)
{
	NvBool acLineStatus = NV_FALSE;
	NvOdmPmuBatteryData batteryData;
	NvU32 VBatSense = 0;
	NvU32 VBatTemp = 0;

	NV_ASSERT(hDevice);
	NV_ASSERT(pAcStatus);
	NV_ASSERT(pBatData);

	batteryData.batteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
	batteryData.batteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return NV_FALSE;
    //20110131, , Stop i2c comm during reset [END]
    
	// Check ACLine Status [Start]
	if (!Max8907BatteryChargerOK(hDevice, &acLineStatus))
	{
		LDP(("[NVODM PMU] Max8907GetAcLineStatus: Error in checking main charger presence.\n"));
		return NV_FALSE;
	}
	// Check ACLine Status [End]

	// Check Battery Data & Status [Start]
	// Get VBatSense
	if (!Max8907AdcVBatSenseRead(hDevice, &VBatSense))
	{
		LDP("Error reading VBATSense.");
		return NV_FALSE;
	}
	LDP("[jh.ahn] VMBATT  = %d", VBatSense);

	// Get VBatTemp
	if (!Max8907AdcVBatTempRead(hDevice, &VBatTemp))
	{
		LDP("Error reading VBatTempRead.");
		return NV_FALSE;
	}
	LDP("[jh.ahn] BatTemp[Rthm]  = %d", VBatTemp);

	// Check Battery Present
	if ( ((309 > VBatTemp) || (4016 < VBatTemp)) && (0 == ARRAY_TP_BOOT()) )// Temperatur -40'C ~ 90'C, if not, determine "No Battery Presence"
	{
		/* Battery is actually not present */
		LDP("[jh.ahn] No Battery Presence..");
		*pBatStatus = NVODM_BATTERY_STATUS_NO_BATTERY;
		if (acLineStatus == NV_TRUE) // This information is required for id_check_polling
			*pAcStatus = NvOdmPmuAcLine_Online;
		else
			*pAcStatus = NvOdmPmuAcLine_Offline;
	}
	else
	{
		LDP("[jh.ahn] Battery Presence..");
		if (acLineStatus == NV_TRUE)
		{
			*pAcStatus = NvOdmPmuAcLine_Online;
			switch (get_charging_ic_status())
			{
				case CHG_IC_DEFAULT_MODE:
				case CHG_IC_USB_LO_MODE:
				case CHG_IC_TA_MODE:
				case CHG_IC_FACTORY_MODE:
					*pBatStatus = NVODM_BATTERY_STATUS_CHARGING;
					break;

				case CHG_IC_DEACTIVE_MODE:
				case CHG_IC_INIT_MODE:
					*pBatStatus = NVODM_BATTERY_STATUS_DISCHARGING;
					break;

				default:
					*pBatStatus = NVODM_BATTERY_STATUS_DISCHARGING;
					break;
			}
		}
		else
		{
			*pAcStatus = NvOdmPmuAcLine_Offline;
			*pBatStatus = NVODM_BATTERY_STATUS_DISCHARGING;
		}
	}

	batteryData.batteryVoltage = VBatSense;
	batteryData.batteryTemperature = Max8907BatteryTemperature(VBatSense, VBatTemp);

	*pBatData = batteryData;
	// Check Battery Data & Status [End]

	return NV_TRUE;
}
#endif // CONFIG_MACH_STAR
//20100924, , For updating battery information totally  [END]

void
Max8907GetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}

void
Max8907GetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry)
{
    *pChemistry = NvOdmPmuBatteryChemistry_LION;
}

NvBool
Max8907SetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType)
{
#ifndef CONFIG_MACH_STAR
    NvU8 data = 0;
    NvU8 fchg = 0;
#endif // CONFIG_MACH_STAR
    NV_ASSERT(hDevice);

    //20110131, , Stop i2c comm during reset [START]
    if ( stop_i2c_flag == NV_TRUE )    
	return NV_FALSE;
    //20110131, , Stop i2c comm during reset [END]

    //20100518, , remove PMIC battery routine [START]
    #if defined(CONFIG_MACH_STAR)
    return NV_TRUE;
    #else    
    // If no battery is connected, then do nothing.
    if (((Max8907PrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
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
    if ( !Max8907I2cRead8(hDevice, MAX8907_CHG_CNTL1, &data) )
        return NV_FALSE;

    // Set charging current to the value no larger than requested.
    // If less than 85mA is requested, set to 85mA.
    // If larger than 1000mA is requested, set to 1000mA.
    if (chargingCurrentLimitMa >= 1000)
        fchg = MAX8907_CHG_CNTL1_FCHG_1000MA;
    else if (chargingCurrentLimitMa >= 900)
        fchg = MAX8907_CHG_CNTL1_FCHG_900MA;
    else if (chargingCurrentLimitMa >= 800)
        fchg = MAX8907_CHG_CNTL1_FCHG_800MA;
    else if (chargingCurrentLimitMa >= 700)
        fchg = MAX8907_CHG_CNTL1_FCHG_700MA;
    else if (chargingCurrentLimitMa >= 600)
        fchg = MAX8907_CHG_CNTL1_FCHG_600MA;
    else if (chargingCurrentLimitMa >= 460)
        fchg = MAX8907_CHG_CNTL1_FCHG_460MA;
    else if (chargingCurrentLimitMa >= 300)
        fchg = MAX8907_CHG_CNTL1_FCHG_300MA;
    else
        fchg = MAX8907_CHG_CNTL1_FCHG_85MA;

    data &= ~(MAX8907_CHG_CNTL1_FCHG_MASK <<
              MAX8907_CHG_CNTL1_FCHG_SHIFT);
    data |= fchg << MAX8907_CHG_CNTL1_FCHG_SHIFT;

    // Turn off the charger path if the requested current limit is 0mA.
    // Turn on the path otherwise.
    if ( chargingCurrentLimitMa == 0 )
    {
        // off
        data |= (MAX8907_CHG_CNTL1_NOT_CHGEN_MASK <<
                 MAX8907_CHG_CNTL1_NOT_CHGEN_SHIFT);
    }
    else
    {
        // on
        data &= ~(MAX8907_CHG_CNTL1_NOT_CHGEN_MASK <<
                  MAX8907_CHG_CNTL1_NOT_CHGEN_SHIFT);
    }

    // Update the current charger setup.
    if ( !Max8907I2cWrite8(hDevice, MAX8907_CHG_CNTL1, data) )
        return NV_FALSE;

    return NV_TRUE;
    #endif
    //20100518, , remove PMIC battery routine [END]
}

void Max8907InterruptHandler( NvOdmPmuDeviceHandle  hDevice)
{
    // If the interrupt handle is called, the interrupt is supported.
    ((Max8907PrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_TRUE;

//20100413, , change [START]
#if defined(CONFIG_MACH_STAR)
    Max8907InterruptHandler_int(hDevice);
#else
    Max8907InterruptHandler_int(hDevice, &((Max8907PrivData*)hDevice->pPrivate)->pmuStatus);
#endif
//20100413, , change [END]
}

//20100413, , unused [START]
#ifndef CONFIG_MACH_STAR
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
    const Max8907PmuSupplyInfo *pSupplyInfo = &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

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
    const Max8907PmuSupplyInfo* pSupplyInfo = &Max8907SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (Max8907PmuSupply)vddRail);

    // Require to turn off the supply
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1)
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
        if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] != 0)
            ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] --;

        if (pSettleMicroSeconds != NULL)
            *pSettleMicroSeconds = settleTime;
        else
            NvOdmOsWaitUS(settleTime);

        return NV_TRUE;
    }

    // turn on supply
    if (((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)
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

    ((Max8907PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] ++;

    // TODO: turn on supply input if necessary
    if (pSettleMicroSeconds != NULL)
        *pSettleMicroSeconds = settleTime;
    else
        NvOdmOsWaitUS(settleTime);

    return NV_TRUE;
}
#endif  
//20100413, , unused [END]

