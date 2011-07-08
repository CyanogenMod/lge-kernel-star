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

#ifndef INCLUDED_MAX8907_SUPPLY_INFO_HEADER
#define INCLUDED_MAX8907_SUPPLY_INFO_HEADER

#include <star_hw_definition.h>
#include "nvodm_pmu.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Defines for the requested voltage for each supply (mV).
// This is board specific and ODM should change this based on device.
#define MAX8907_REQUESTVOLTAGE_LX_V1       1200
#define MAX8907_REQUESTVOLTAGE_LX_V2       1200
#define MAX8907_REQUESTVOLTAGE_LX_V3       1800

//20100413, , add [START]
#if defined(CONFIG_MACH_STAR)
#define MAX8907_REQUESTVOLTAGE_LDO1        3300     //DDRx 3.3V
#define MAX8907_REQUESTVOLTAGE_LDO2        1100     //PLL 1.1V
#define MAX8907_REQUESTVOLTAGE_LDO3        1800     //LCD 1.8V
#define MAX8907_REQUESTVOLTAGE_LDO4        3300     //USB
//emmc : for voltage margin
//#define MAX8907_REQUESTVOLTAGE_LDO5        2800
#define MAX8907_REQUESTVOLTAGE_LDO5        3000    //2800->3000
#define MAX8907_REQUESTVOLTAGE_LDO6        1800     //HDMI_PLL
#define MAX8907_REQUESTVOLTAGE_LDO7        3000     //SENSOR_3V
#define MAX8907_REQUESTVOLTAGE_LDO8        1800     //SENSOR_1.8V
#define MAX8907_REQUESTVOLTAGE_LDO9        3300     //DNI - FUSE_3.3V
#define MAX8907_REQUESTVOLTAGE_LDO10       3100     //TOUCH 3.1V
#define MAX8907_REQUESTVOLTAGE_LDO11       3300     //HDMI 3.3V
#define MAX8907_REQUESTVOLTAGE_LDO12       2800     //SDIO 2.8V
#define MAX8907_REQUESTVOLTAGE_LDO13       3000     //MOTOR 3V
#define MAX8907_REQUESTVOLTAGE_LDO14       2800     //LCD 2.8V
#define MAX8907_REQUESTVOLTAGE_LDO15       3300     //APTEMP 3.3V
#define MAX8907_REQUESTVOLTAGE_LDO16       2800     //DNI - emmc 2.8V
#define MAX8907_REQUESTVOLTAGE_LDO17       1200     //MIPI 1.2V
#define MAX8907_REQUESTVOLTAGE_LDO18       1800     //VI 1.8V
#define MAX8907_REQUESTVOLTAGE_LDO19       1800     //TOUCH 1.8V
#define MAX8907_REQUESTVOLTAGE_LDO20       0
#define MAX8907_REQUESTVOLTAGE_WLED        0x14     //0x14(2mA)  <- 0x46(7mA) <- 0x64(10mA) : 0x00(0.1mA)~0xff(25.5mA)
#else
#error "NOT SURPORTED!!!"
#endif
//20100413, , add [END]

#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_1  5000    // Fixed
#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_2     0    // Reserved
#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_3  1050    // Fixed (unless FAN5355 is enabled)
#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_4     0    // VBL1, controlled by display adaptation
#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_5     0    // VBL2, controlled by display adaptation
#define MAX8907_REQUESTVOLTAGE_EXT_DCDC_6     0    // Reserved

#define MAX8907_REQUESTVOLTAGE_SDBY        1100

// Defines default sequencer selection
#define MAX8907_SEQSEL_DEFAULT_LX_V1       0   /* SEQ1 (SYSEN) */
#define MAX8907_SEQSEL_DEFAULT_LX_V2       0   /* SEQ1 (SYSEN) */
#define MAX8907_SEQSEL_DEFAULT_LDO4        0   /* SEQ1 (SYSEN) */

// Defines common for all supplies PWREN  sequencer selection
#define MAX8907_SEQSEL_PWREN_LXX           1   /* SEQ2 (PWREN) */

// Defines common for all supplies I2C (s/w) sequencer selection
#define MAX8907_SEQSEL_I2CEN_LXX           7   /* I2CEN (s/w) */

// Defines sequencer count default values
#define MAX8907_SEQCNT_DEFAULT_LX_V1       0x1C
#define MAX8907_SEQCNT_DEFAULT_LX_V2       0x1C
#define MAX8907_SEQCNT_DEFAULT_LDO4        0x72

// Defines sequencer count PWREN control values (these settings applied
// togeteher, when both CPU/V1 and CORE/V2 rails are attached to PWREN;
// in case when only CPU/V1 rail is attached no delay is specified)
// B[7:4] - power up delay in 20us taps
// B[3:0] - power down delay in 20us taps
#define MAX8907_SEQCNT_PWREN_LX_V1         0xC0    /* 240us up delay */
#define MAX8907_SEQCNT_PWREN_LX_V2         0x00    /* no delay */
#define MAX8907_SEQCNT_PWREN_LD04          0x00    /* no delay */

// Defines PMU output timing parameters. Scaling up time is dynamically
// calculated based on the slew rate maintained by MAX8907. Turn On delay
// is fixed at max. Turn Off time is "just in case" placeholder -  no need
// for s/w to track when output capacitors are discharged.

//20100811 CPU power patch [START]
#define MAX8952_SCALE_UP_UV_PER_US         (16000)   // 16mv per 1us
//20100811 CPU power patch [END]
#define MAX8907_SCALE_UP_UV_PER_US         (2500)
#define MAX8907_TURN_ON_TIME_US            (3000)
#define MAX8907_TURN_OFF_TIME_US           (20)

//20100413, , add
#define MAX8907_INVALID_PORT               0xff

// Output voltages supplied by PMU
typedef enum
{
    Max8907PmuSupply_Invalid = 0x0,

    /*-- Step-Down DC Regulators --*/
    Max8907PmuSupply_LX_V1,    // LX_V1 (V1), step-down DC regulator
    Max8907PmuSupply_LX_V2,    // LX_V2 (V2), step-down DC regulator
    Max8907PmuSupply_LX_V3,    // LX_V3 (V3), step-down DC regulator

    /*-- Standby LDO --*/
    Max8907PmuSupply_VRTC,     // VRTC (RTC), always-on supply for RTC

    /*-- Linear Regulator Outputs --*/
    Max8907PmuSupply_LDO1,     // LDO1 (VOUT1), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO2,     // LDO2 (VOUT2), linear regulator output (default = 1.2V)
    Max8907PmuSupply_LDO3,     // LDO3 (VOUT3), linear regulator output (default = 1.2V)
    Max8907PmuSupply_LDO4,     // LDO4 (VOUT4), linear regulator output (default = 3.3V)
    Max8907PmuSupply_LDO5,     // LDO5 (VOUT5), linear regulator output (default = 1.8V)
    Max8907PmuSupply_LDO6,     // LDO6 (VOUT6), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO7,     // LDO7 (VOUT7), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO8,     // LDO8 (VOUT8), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO9,     // LDO9 (VOUT9), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO10,    // LDO10 (VOUT10), linear regulator output (default = 1.8V)
    Max8907PmuSupply_LDO11,    // LDO11 (VOUT11), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO12,    // LDO12 (VOUT12), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO13,    // LDO13 (VOUT13), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO14,    // LDO14 (VOUT14), linear regulator output (default = 3.3V)
    Max8907PmuSupply_LDO15,    // LDO15 (VOUT15), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO16,    // LDO16 (VOUT16), linear regulator output (default = 2.8V)
    Max8907PmuSupply_LDO17,    // LDO17 (VOUT17), linear regulator output (default = 1.2V)
    Max8907PmuSupply_LDO18,    // LDO18 (VOUT18), linear regulator output (default = 1.2V)
    Max8907PmuSupply_LDO19,    // LDO19 (VOUT19), linear regulator output (default = 3.3V)
    Max8907PmuSupply_LDO20,    // LDO20 (VOUT20), linear regulator output (default = 1.2V)
        
    #if defined(CONFIG_MACH_STAR)
    /*-- White LED --*/
    Max8907PmuSupply_WHITE_LED,    // (Boost WLED)
    //20100413, , add [START]
    Max8907PmuSupply_SOC,
    Max8907PmuSupply_reset,
    //20100413, , add [END]
    #endif

    //20100413, , add [START]
    #ifndef CONFIG_MACH_STAR
    /*-- External DC/DC switcher --*/
    Max8907PmuSupply_EXT_DCDC_1,   // EXT_DC/DC1
    Max8907PmuSupply_EXT_DCDC_2,   // EXT_DC/DC2
    Max8907PmuSupply_EXT_DCDC_3,   // EXT_DC/DC3
    Max8907PmuSupply_EXT_DCDC_4,   // EXT_DC/DC4
    Max8907PmuSupply_EXT_DCDC_5,   // EXT_DC/DC5
    Max8907PmuSupply_EXT_DCDC_6,   // EXT_DC/DC6

    /** USB1 & USB3 VBus's are getting 5V from DCDC_3  **/
    Max8907PmuSupply_EXT_DCDC_3_USB1, //USB1 VBUS
    Max8907PmuSupply_EXT_DCDC_3_USB3, // USB3 VBUS

    /** Secondary PMU MIC2826 Rails  **/
    MIC2826PmuSupply_BUCK,
    MIC2826PmuSupply_LDO1,
    MIC2826PmuSupply_LDO2,
    MIC2826PmuSupply_LDO3,

    // External DCDC controlled by LX_V1, and scaled by digital
    // potentiometer (DPM) AD5258
    Max8907LxV1_Ad5258_DPM_EXT_DCDC_7,
    #endif
    //20100413, , add [END],

    #if 0
    //Temp for enabling fuse using p2 of i0 expander
    Max8907PmuSupply_VBAT_FUSE,
    #endif

	// MAX8952 for VDD CPU
	Max8907PmuSupply_EXT_DCDC_8_CPU,

    //20110131, , Stop i2c comm during reset [START]
    #if defined(CONFIG_MACH_STAR)
    Max8907PmuSupply_Stop_i2c_Flag,    
    #endif
    //20110131, , Stop i2c comm during reset [END]

    Max8907PmuSupply_Num,
    Max8907PmuSupply_Force32 = 0x7FFFFFFF
} Max8907PmuSupply;

typedef NvU32  (*Max8907PmuVoltageFunc)(const NvU32 data);

typedef struct Max8907PmuSupplyInfoRec
{
    Max8907PmuSupply supply;

    // I2C Registers
    NvU8 ControlRegAddr;
    NvU8 SequencerCountRegAddr;
    NvU8 OutputVoltageRegAddr;
    NvU8 OutputPort;
    NvU8 PmuGpio;

    Max8907PmuVoltageFunc GetVoltage;  // Function to convert register bits to real voltage
    Max8907PmuVoltageFunc SetVoltage;  // Function to convert real voltage to register bits

    NvOdmPmuVddRailCapabilities cap;
} Max8907PmuSupplyInfo;

#if defined(__cplusplus)
}
#endif
#endif //INCLUDED_MAX8907_SUPPLY_INFO_HEADER

