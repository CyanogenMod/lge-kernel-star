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

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: Specifies the peripheral connectivity 
 *                 database NvOdmIoAddress entries for the E1109
 *                 Processor Module.
 */

#include "pmu/max8907/max8907_supply_info_table.h"
#include "tmon/adt7461/nvodm_tmon_adt7461.h"

//20100426 sk.hwang@lge.com For global definition
//#include <star_global_definition.h>
#include <star_hw_definition.h>
#include <star_pinmux_definition.h>

#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
static const NvOdmIoAddress s_lge_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98, 0 },               /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO15, 0 }, /* TMON pwer rail -> D4REG */
    { NvOdmIoModule_Gpio, 'k' - 'a', 2, 0 },                /* GPIO Port K and Pin 2 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote, 0 }, /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local, 0 }, /* TSENSOR */
};
#elif defined(CONFIG_MACH_STAR_REV_F)	// modified in LGP990 revF
static const NvOdmIoAddress s_lge_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98, 0 },               /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO15, 0 }, /* TMON pwer rail -> D4REG */
    { NvOdmIoModule_Gpio, 'k' - 'a', 2, 0 },                /* GPIO Port K and Pin 2 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote, 0 }, /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local, 0 }, /* TSENSOR */
};
#endif

#if defined(CONFIG_MACH_STAR)
//20100413, cs77.ha@lge.com, power off [START]
static const NvOdmIoAddress s_lge_SocOdmAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_SOC, 0 },
};
//20100413, cs77.ha@lge.com, power off [END]

//20100703, cs77.ha@lge.com, power reset [START]
static const NvOdmIoAddress s_lge_PowerResetAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_reset, 0 },
};
//20100703, cs77.ha@lge.com, power reset [END]
#endif

//20100413, cs77.ha@lge.com, powerkey [START]
#if defined(CONFIG_MACH_STAR)
static const NvOdmIoAddress s_lge_PowerKeyAddresses[] =
{
    { NvOdmIoModule_Gpio, 'v' - 'a' , 2, 0 },
};
#endif
//20100413, cs77.ha@lge.com, powerkey [END]

//20101129, hyeongwon.oh@lge.com, SU660 homekey [START]
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F) 
static const NvOdmIoAddress s_lge_HomeKeyAddresses[] =
{
    { NvOdmIoModule_Gpio, 'v' - 'a' , 6, 0 },
};
#endif
#endif
//20101129, hyeongwon.oh@lge.com, SU660 homekey [END]

static const NvOdmIoAddress s_lge_HdmiAddresses[] =
{
    { NvOdmIoModule_Hdmi, 0, 0, 0 },

    /* Display Data Channel (DDC) for Extended Display Identification
     * Data (EDID)
     */
    { NvOdmIoModule_I2c, 0x01, 0xA0, 0 },

    /* HDCP downstream */
    { NvOdmIoModule_I2c, 0x01, 0x74, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO6, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO11, 0 },
};

static const NvOdmIoAddress s_lge_VideoDacAddresses[] =
{
    { NvOdmIoModule_Tvo, 0x00, 0x00, 0 },
};

#if 1

// RTC voltage rail address
static const NvOdmIoAddress s_lge_RtcAddresses[] =
{
    // On Maxim 8907B, the standby rail automatically follows V2
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V2, 0 },  /* VDD_RTC -> RTC */
};


// Core voltage rail address
static const NvOdmIoAddress s_lge_CoreAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V2, 0 },    
};

// CPU voltage rail
static const NvOdmIoAddress s_lge_CpuAddresses[] = 
{
//20100725 taewan.kim@lge.com add MAX8907C FEATURE [START]
#if defined(CONFIG_MACH_STAR)
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_EXT_DCDC_8_CPU, 0 },  /* VDD_CPU_PMU -> V1 */
#else
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V1, 0 },  /* VDD_CPU_PMU -> V1 */
#endif
//20100725 taewan.kim@lge.com add MAX8907C FEATURE [END]
};
#endif

// PLLA voltage rail
static const NvOdmIoAddress s_lge_PllAAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_lge_PllMAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLM -> VOUT2 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_lge_PllPAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLC voltage rail
static const NvOdmIoAddress s_lge_PllCAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLA_P_C_S -> VOUT2 */
};


// PLLU1 voltage rail
static const NvOdmIoAddress s_lge_PllDAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLU -> VOUT2 */
};

// PLLE voltage rail
static const NvOdmIoAddress s_lge_PllEAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLL_E -> VOUT2 */
};

// PLLU1 voltage rail
static const NvOdmIoAddress s_lge_PllU1Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLU -> VOUT2 */
};

// PLLS voltage rail
static const NvOdmIoAddress s_lge_PllSAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_lge_PllHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO6, 0 }, /* AVDD_HDMI_PLL -> VOUT6 */
};

// PLLX voltage rail
static const NvOdmIoAddress s_lge_PllXAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 }, /* AVDD_PLLX -> VOUT2 */
};

// OSC voltage rail
static const NvOdmIoAddress s_lge_VddOscAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* AVDD_OSC -> V3 */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_lge_PllUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO4, 0 }, /* AVDD_USB_PLL -> VOUT4 */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_lge_VddSysAddresses[] = 
{
    //System IO voltage rail is 1.8V
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_SYS -> V3 */
};

// USB voltage rail
static const NvOdmIoAddress s_lge_VddUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO4, 0 }, /* AVDD_USB -> VOUT4 */
};

// HDMI voltage rail
static const NvOdmIoAddress s_lge_VddHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO11, 0 }, /* AVDD_HDMI -> VOUT11 */
};

// MIPI voltage rail
static const NvOdmIoAddress s_lge_VddMipiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO17, 0 }, /* VDDIO_MIPI -> VOUT17 */
};

// LCD voltage rail
static const NvOdmIoAddress s_lge_VddLcdAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_LCD_PMU -> V3 */
};

// Audio voltage rail
static const NvOdmIoAddress s_lge_VddAudAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_AUDIO -> V3 */
};

// DDR voltage rail
static const NvOdmIoAddress s_lge_VddDdrAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, 
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_lge_VddDdrRxAddresses[] = 
{
    //In a schematics, VADC_3.3V.
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO1, 0 },  /* VDDIO_RX_DDR(2.7-3.3) -> VOUT1 */
};

// NAND voltage rail
static const NvOdmIoAddress s_lge_VddNandAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_NAND_PMU -> V3 */
};

// UART voltage rail
static const NvOdmIoAddress s_lge_VddUartAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_UART -> V3 */
};

// SDIO voltage rail
static const NvOdmIoAddress s_lge_VddSdioAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO12, 0 },
};

// VDAC voltage rail
static const NvOdmIoAddress s_lge_VddVdacAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, 0x00, 0 }, /* AVDD_VDAC -> None (No TVout) */
};

// VI voltage rail
static const NvOdmIoAddress s_lge_VddViAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO18, 0 }, /* VDDIO_VI -> VOUT18 */
};

// BB voltage rail
static const NvOdmIoAddress s_lge_VddBbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, /* VDDIO_BB -> V3 */
};

// HSIC voltage rail
static const NvOdmIoAddress s_lge_VddHsicAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, 0x00, 0 }, 
};

// USB_IC voltage rail
static const NvOdmIoAddress s_lge_VddUsbIcAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, 0x00, 0 },  
};

// PMU0
static const NvOdmIoAddress s_lge_Pmu0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x78, 0 },
};

// USB1 VBus voltage rail
static const NvOdmIoAddress s_lge_VddUsb1VBusAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO4, 0 },
};

// USB3 VBus voltage rail
static const NvOdmIoAddress s_lge_VddUsb3VBusAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, 0x00, 0 },	// USB3 is not used in Star1
};


// FUSE voltage enablel
static const NvOdmIoAddress s_lge_VddFuseAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO9, 0 },
};

// SDIO memory such as SD card and eMMC
static const NvOdmIoAddress s_lge_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x2, 0x0, 0 }, // microSD
    { NvOdmIoModule_Sdio, 0x3, 0x0, 0 }, // eMMC
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO12, 0 }, // 2.8V microSD(VDDIO_SDIO) 
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, // 1.8V eMMC VCCQ & microSD detect(VDDIO_NAND)
    //20100727 cs77.ha@lge.com it should be always on for (a03 deepsleep isseu) work around [START]
    #if defined(CONFIG_MACH_STAR_REV_B)   
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO5, 0 }, // eMMC 2.8V VCC
    #endif
    //20100727 cs77.ha@lge.com it should be always on for (a03 deepsleep isseu) work around [END]
};

//Vibrator sk.hwang@lge.com for vibrator 
static const NvOdmIoAddress s_lge_VibAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x0, Max8907PmuSupply_LDO13, 0 }, // VCC_MOTOR_3V0
#if defined (CONFIG_MACH_STAR_REV_A)
    { NvOdmIoModule_Gpio, 'u' - 'a', 5, 0 }, //VIBE_EN
#else
    { NvOdmIoModule_Gpio, 'u' - 'a', 4, 0 }, //VIBE_EN
#endif
};

// Accelerometer
static const NvOdmIoAddress s_lge_AccelerometerAddresses[] =
{
    { NvOdmIoModule_I2c,  0x01, 0x0f, 0 }, /* KXTF9 I2C device address is 0x0f */
    { NvOdmIoModule_Gpio, 'i' - 'a', 0, 0 }, //MOTION_INT
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO7, 0 },   /* LDO7 3.0v VCC_SENSOR_3V0 */
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO8 , 0 },  /* LDO8 1.8V VCC_SENSOR_1V8*/
};

//Compass sk.hwang@lge.com
static const NvOdmIoAddress s_lge_CompassAddresses[] =
{
    { NvOdmIoModule_I2c,  0x01, 0x0e, 0 }, /* I2C device address is 0x0F */
    { NvOdmIoModule_Gpio, 'r' - 'a', 4, 0 }, 
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO7 , 0 },  /* LDO8 1.8V VCC_SENSOR_1V8*/
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO8, 0 },   /* LDO7 3.0v VCC_SENSOR_3V0 */
};

static const NvOdmIoAddress s_lge_GyroAddresses[] =
{
    { NvOdmIoModule_I2c,  0x01, 0x68, 0 }, /* MPU3050 I2C device address is 0x0f */
    { NvOdmIoModule_Gpio, 'q' - 'a', 5, 0 }, //GYRO_INT
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO7, 0 },   /* LDO7 3.0v VCC_SENSOR_3V0 */
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO8 , 0 },  /* LDO8 1.8V VCC_SENSOR_1V8*/
};

//Proximity sk.hwang@lge.com
static const NvOdmIoAddress s_lge_ProximityAddresses[] =
{
	{ NvOdmIoModule_I2c,  0x01, 0x44, 0 }, /* I2C device address is 0x44 */
#if defined (CONFIG_MACH_STAR_REV_E) || defined (CONFIG_MACH_STAR_REV_F)
	{ NvOdmIoModule_Gpio, 'w' - 'a', 2, 0 }, 
#elif defined (CONFIG_MACH_STAR_REV_D)
	{ NvOdmIoModule_Gpio, 'r' - 'a', 2, 0 }, 
#else
	{ NvOdmIoModule_Gpio, 'a' - 'a', 0, 0 }, 
#endif
	{ NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO7 , 0 },  /* LDO8 1.8V VCC_SENSOR_1V8*/
	{ NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO8, 0 },   /* LDO7 3.0v VCC_SENSOR_3V0 */
};


// Audio Codec
static const NvOdmIoAddress s_lge_AudioCodecAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 }, //VCC_IO_1V8
    { NvOdmIoModule_ExternalClock, 0, 0, 0 },                  // connected to CDEV1
    { NvOdmIoModule_I2c, 1, 0x34, 0 },           
    { NvOdmIoModule_Dap, 0, 0, 0 },                            /* Dap port Index 0 is used for codec*/
};

// Main LCD
static const NvOdmIoAddress s_lge_MainDisplayAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LX_V3, 0 },   /* VDDIO_LCD -> V3 */
//20100725 taewan.kim@lge.com add CPU Interface [START]
#if defined(CONFIG_MACH_STAR)
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO3 , 0 },  // NVVDD_LDO3_1V8
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO14 , 0 },  // NVVDD_LDO14_2V8
#else
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO17 , 0 },  /* MIPI DSI 1.2V */
    { NvOdmIoModule_Gpio, (NvU32)('v' - 'a'), 7, 0 },
#endif
//20100725 taewan.kim@lge.com add CPU Interface [END]
};

#if 0
// PMU resistive TouchPanel is not used in Star1
static const NvOdmIoAddress s_lge_TouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x8E, 0 }, 
    { NvOdmIoModule_Gpio, 'b' - 'a', 2, 0 }, /* GPIO Port X and Pin 4 */
    //Using Touch controller inside in PMU for now, so do not need voltage for Touch controller.
};

// I2C TouchPanel for LGE MP
static const NvOdmIoAddress s_lge_TouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0x02, 0 }, // 0x02 should be changed with the actual Touch I2C address (currently unknown).
    { NvOdmIoModule_Gpio, 'd' - 'a', 0, 0 }, /* GPIO Port D and Pin 0 */
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO3, 0 },	// NVVDD_LDO3_1V8
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO13, 0 },	// 3.1V_TOUCH
};
#endif

#if defined(CONFIG_MACH_STAR)
// 20100927  hyeongwon.oh@lge.com Synaptics OneTouch support [START]
static const NvOdmIoAddress s_lge_SynapticsOneTouchAddresses[] = 
{
	{ NvOdmIoModule_I2c, 0x00, 0x2C, 0 },						/* GEN1_I2C instance = 0x00, OneTouch IC I2C Address = 0x2C */
	{ NvOdmIoModule_Gpio, 'j' - 'a', 6, 0 },					/* GPIO Port J and Pin 6, Int */
	{ NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO16, 0 },	// VCC_TOUCH_1V8
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F) // 20101120 hyeongwon.oh@lge.com power off when Onetouch close
	{ NvOdmIoModule_I2c_Pmu, 0x00, Max8907PmuSupply_LDO19, 0 },	// TOUCH_I2C_1V8
#endif 
};
// 20100927  hyeongwon.oh@lge.com Synaptics OneTouch support [END]

// 20100527 joseph.jung@lge.com Synaptics/Cypress Touch support [START]
static const NvOdmIoAddress s_lge_SynapticsTouchAddresses[] = 
{
	{ NvOdmIoModule_I2c, 0x00, 0x20, 0 },						/* GEN1_I2C instance = 0x00, Touch IC I2C Address = 0x20 */
	{ NvOdmIoModule_Gpio, 'x' - 'a', 6, 0 },					/* GPIO Port X and Pin 6, Int */
	{ NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO10, 0 },		// TOUCH_VCC_3V1
	{ NvOdmIoModule_I2c_Pmu, 0x00, Max8907PmuSupply_LDO19, 0 },	// TOUCH_I2C_1V8
};

static const NvOdmIoAddress s_lge_CypressTouchAddresses[] = 
{
	{ NvOdmIoModule_I2c, 0x00, 0x24, 0 },						/* GEN1_I2C instance = 0x00, Touch IC I2C Address = 0x35 */
	{ NvOdmIoModule_Gpio, 'x' - 'a', 6, 0 },					/* GPIO Port X and Pin 6, Int */
	{ NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO10, 0 },		// TOUCH_VCC_3V1
	{ NvOdmIoModule_I2c_Pmu, 0x00, Max8907PmuSupply_LDO19, 0 },	// TOUCH_I2C_1V8
};
// 20100527 joseph.jung@lge.com Synaptics/Cypress Touch support [END]
#endif

// 20100401 taewan.kim@lge.com MUIC driver [START]
#if defined (CONFIG_MACH_STAR)
static const NvOdmIoAddress s_lge_MuicAddresses[] = 
{
#if defined (CONFIG_MACH_STAR_REV_A)
    { NvOdmIoModule_I2c, 0x00, 0x44, 0 },            /* GEN1_I2C instance = 0x00, Muic IC I2C Address = 0x44 */
#endif
    { NvOdmIoModule_Gpio, 'u' - 'a', 0, 0 }, /* INT_N_MUIC */
    { NvOdmIoModule_Gpio, 'u' - 'a', 1, 0 }, /* AP20_UART_SW */
    { NvOdmIoModule_Gpio, 'u' - 'a', 2, 0 }, /* IFX_UART_SW   */
    { NvOdmIoModule_Gpio, 'u' - 'a', 3, 0 }, /* USIF1_SW  */
#if defined(CONFIG_MACH_STAR_REV_D) || defined(CONFIG_MACH_STAR_REV_E) || defined(CONFIG_MACH_STAR_REV_F)
//20100917 julius.moon@lge.com for IFX_USB_VBUS_EN GPIO Port # for SKT  [START]
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
    { NvOdmIoModule_Gpio, 'r' - 'a', 5, 0 }, /* IFX_USB_VBUS_EN  */
#else
//for P990 Below...
    { NvOdmIoModule_Gpio, 'r' - 'a', 7, 0 }, /* IFX_USB_VBUS_EN  */
#endif
//20100917 julius.moon@lge.com for IFX_USB_VBUS_EN GPIO Port # for SKT  [END]
#endif

};
#endif
// 20100401 taewan.kim@lge.com MUIC driver [END]

//20100609, jh.ahn@lge.com, Charger IC Driver [START]
#if defined (CONFIG_MACH_STAR)
static const NvOdmIoAddress s_lge_ChargerAddresses[] =
{
    { NvOdmIoModule_Gpio, 's' - 'a', 1, 0 }, /* CHG_EN_SET_N_AP20 */
    { NvOdmIoModule_Gpio, 's' - 'a', 2, 0 }, /* CHG_STATUS_N_AP20 */
	{ NvOdmIoModule_Gpio, 'q' - 'a', 2, 0 }, /* CHG_PGB_N */
};
#endif
//20100609, jh.ahn@lge.com, Charger IC Driver [END]

static const NvOdmIoAddress s_lge_BackLightAddresses[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0xC0, 0 },            /* GEN1_I2C instance = 0x00, AAT2870 I2C Address = 0xC0 */
    { NvOdmIoModule_Gpio, 'r' - 'a', 3, 0 }, /* GPIO Port R and Pin 3 */
};

//20100424, bojung.ko@lge.com, RIL code from firenze [START]
#if defined (CONFIG_MACH_STAR)
static const NvOdmIoAddress s_lge_SpiAddresses[] = 
{
    { NvOdmIoModule_Gpio, 'o' - 'a', 5, 0 }, 
    { NvOdmIoModule_Gpio, 'o' - 'a', 0, 0 }, 
//20100711-1, syblue.lee@lge.com, Add spi controller 0 and chip select 0 [START]
    { NvOdmIoModule_Spi, AP20_SPI1,  0x0, 0 }, /* Spi Controller 0 and Chip Select 0 */
//20100711, syblue.lee@lge.com, Add spi controller 0 and chip select 0 [END]
};
#endif
//20100424, bojung.ko@lge.com, RIL code from firenze [END]

// Bluetooth
static const NvOdmIoAddress s_lge_BluetoothAddresses[] =
{
    { NvOdmIoModule_Uart, 0x2,  0x0, 0 }, //Instance 2 means UART3.
#if defined(CONFIG_MACH_STAR_SKT_REV_D) || defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_REV_F)
    { NvOdmIoModule_Gpio, 'z' - 'a', 2, 0 }, // bt_en
#else
    { NvOdmIoModule_Gpio, 'q' - 'a', 4, 0 }, // bt_en
#endif    
    { NvOdmIoModule_Gpio, 'c' - 'a', 7, 0 }, // bt_host_wakeup
    { NvOdmIoModule_Gpio, 'x' - 'a', 4, 0 }, // bt_wakeup
};

// Wlan
static const NvOdmIoAddress s_lge_WlanAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0, 0 },    // WLAN is on SD Bus
#if defined(CONFIG_MACH_STAR_SKT_REV_D) || defined(CONFIG_MACH_STAR_REV_F)
    { NvOdmIoModule_Gpio, 'w' - 'a', 1, 0 }, // wlan_en
#else
    { NvOdmIoModule_Gpio, 'q' - 'a', 3, 0 }, // wlan_en
#endif    
    { NvOdmIoModule_Gpio, 's' - 'a', 0, 0 }, // wlan_host_wakeup
    { NvOdmIoModule_Gpio, 'g' - 'a', 2, 0 }, // wlan_wakeup
};


//20100421 bergkamp.cho@lge.com [LGE_START]
#if defined (CONFIG_MACH_STAR)
#if !defined(STAR_COUNTRY_KR)//Global 						//20101005 seki.par@lge.com Gpio MicBias[START_LGE_LAB1]
static const NvOdmIoAddress s_lge_HeadsetAddresses[] = 
{
    { NvOdmIoModule_Gpio, 'g' - 'a', 3, 0 }, //Headset Detection
    { NvOdmIoModule_Gpio, 'd' - 'a', 3, 0 }, //hook Detection //jongik2.kim 20100803 HOOK_DETECTION    
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO12, 0 }, //hook detection power rail
};
#else //KR
static const NvOdmIoAddress s_lge_HeadsetAddresses[] = 
{
    { NvOdmIoModule_Gpio, 'g' - 'a', 3, 0 }, //Headset Detection
#if defined(CONFIG_MACH_STAR_SKT_REV_A)
    { NvOdmIoModule_Gpio, 'd' - 'a', 3, 0 }, //hook Detection //jongik2.kim 20100803 HOOK_DETECTION     
#elif defined(CONFIG_MACH_STAR_SKT_REV_B)||defined(CONFIG_MACH_STAR_SKT_REV_C)
    { NvOdmIoModule_Gpio, 'h' - 'a', 3, 0 }, //TEST
#else
    { NvOdmIoModule_Gpio, 'n' - 'a', 5, 0 }, //CONFIG_MACH_STAR_SKT_REV_D //20101101 seki.par@lge.com Gpio MicBias[START_LGE_LAB1]
#endif
	{ NvOdmIoModule_Gpio, 'h' - 'a', 1, 0 },  //Headset MicBias
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO12, 0 }, //hook detection power rail
};
#endif /*STAR_COUNTRY_KR*/ //20101005 seki.par@lge.com Gpio MicBias[END_LGE_LAB1]
#endif /* CONFIG_MACH_STAR */
//20100421 bergkamp.cho@lge.com [LGE_END]

//20100730 kyungsik.lee@lge.com [LGE_START]
#if defined (CONFIG_MACH_STAR)
static const NvOdmIoAddress s_lge_CPdeviceAddresses[] = 
{
#if defined (CONFIG_SPI_MDM6600)
		    { NvOdmIoModule_Gpio, 'v' - 'a', 3, 0 }, //CP device V3 Pin
#else
		    { NvOdmIoModule_Gpio, 'r' - 'a', 1, 0 }, //CP device V3 Pin
#endif /* CONFIG_SPI_MDM6600 */
};
#endif /* CONFIG_MACH_STAR */
//20100730 kyungsik.lee@lge.com [LGE_END]


typedef enum
{
    NvOdmKbcGpioPin_KBRow0=0,
    NvOdmKbcGpioPin_KBRow1,
    NvOdmKbcGpioPin_KBRow2,
    NvOdmKbcGpioPin_KBRow3,
    NvOdmKbcGpioPin_KBRow4,
    NvOdmKbcGpioPin_KBRow5,
    NvOdmKbcGpioPin_KBRow6,
    NvOdmKbcGpioPin_KBRow7,
    NvOdmKbcGpioPin_KBRow8,
    NvOdmKbcGpioPin_KBRow9,
    NvOdmKbcGpioPin_KBRow10,
    NvOdmKbcGpioPin_KBRow11,
    NvOdmKbcGpioPin_KBRow12,
    NvOdmKbcGpioPin_KBRow13,
    NvOdmKbcGpioPin_KBRow14,
    NvOdmKbcGpioPin_KBRow15,
    NvOdmKbcGpioPin_KBCol0,
    NvOdmKbcGpioPin_KBCol1,
    NvOdmKbcGpioPin_KBCol2,
    NvOdmKbcGpioPin_KBCol3,
    NvOdmKbcGpioPin_KBCol4,
    NvOdmKbcGpioPin_KBCol5,
    NvOdmKbcGpioPin_KBCol6,
    NvOdmKbcGpioPin_KBCol7,
    NvOdmKbcGpioPin_Num,
    NvOdmKbcGpioPin_Force32 = 0x7FFFFFFF
}NvOdmKbcGpioPin;


// No key matrix is used in Star1
static const NvOdmIoAddress s_lge_KeyPadAddresses[] =
{
    // instance = 1 indicates Column info.
    // instance = 0 indicates Row info.
    // address holds KBC pin number used for row/column.

    // All Row info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow0, 0 },    // Row 0
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow1, 0 },    // Row 1
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow2, 0 },    // Row 2

    // All Column info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol0, 0 },    // Column 0
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol1, 0 },    // Column 1
};

//LGE_UPDATE_S neo.shin@lge.com 2010-05-024 GPS UART & GPIO Setting
static const NvOdmIoAddress s_lge_GPSAddresses[] =
{
    { NvOdmIoModule_Uart, 0x3,  0x0, 0 }, 		//Instance 3 means UART4.
    { NvOdmIoModule_Gpio, 'j' - 'a', 0, 0 }, 	// GPIO_PJ0- reset
    { NvOdmIoModule_Gpio, 'j' - 'a', 2, 0 }, 	// GPIO_PJ2 -poweron
    { NvOdmIoModule_Gpio, 'd' - 'a', 0, 0 }, 	// GPIO_PD0 -External LNA
};
//LGE_UPDATE_E neo.shin@lge.com 2010-05-024 GPS UART & GPIO Setting


//20100611, cs77.ha@lge.com, Touch LED [START]
static const NvOdmIoAddress s_lge_TouchLEDAddresses[] = 
{
	{ NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_WHITE_LED, 0 },   //Touch LED
};
//20100611, cs77.ha@lge.com, Touch LED [END]


//20100603, cs77.ha@lge.com, star pmic [START]
#ifdef CONFIG_STAR_PMIC
static const NvOdmIoAddress s_lge_AllRailAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO1, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO2, 0 },      // always on
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO3, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO4, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO5, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO6, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO7, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO8, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO9, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO10, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO11, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO12, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO13, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO14, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO15, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO16, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO17, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO18, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO19, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907PmuSupply_LDO20, 0 },
};
#endif
//20100603, cs77.ha@lge.com, star pmic [END]

#ifdef CONFIG_SPI_TDMB	
//20100918 suyong.han@lge.com TDMB Base [START_LGE_LAB1]
//20100912, suyong.han@lge.com [START]	
static const NvOdmIoAddress s_tdmbSIC2102Addresses[] =
{
    { NvOdmIoModule_Spi, AP20_SPI2,  0x0, 0 },                      /* Spi Controller 0 and Chip Select 0 */
    { NvOdmIoModule_Gpio, 'r' - 'a', 0x7, 0 },                      /* GPIO Port R and Pin 7 DMB_EN */
    { NvOdmIoModule_Gpio, 'o' - 'a', 0x7, 0 },                      /* GPIO Port O and Pin 7 DMB_RESET */
    { NvOdmIoModule_Gpio, 'o' - 'a', 0x6, 0 },                      /* GPIO Port O and Pin 6 DMB_INT */
    { NvOdmIoModule_Gpio, 'x' - 'a', 0x7, 0 }                      /* GPIO Port X and Pin 7 DMB_EAR_ANT */
};
//20100912, syblue.lee@lge.com [END]
//20100918 suyong.han@lge.com TDMB Base [END_LGE_LAB1]
#endif


