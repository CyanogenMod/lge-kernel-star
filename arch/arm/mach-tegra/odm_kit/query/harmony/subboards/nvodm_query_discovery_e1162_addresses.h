/*
 * arch/arm/mach-tegra/odm_kit/query/harmony/subboards/nvodm_query_discovery_e1162-addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1162 module
 *
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

#include "pmu/tps6586x/nvodm_pmu_tps6586x_supply_info_table.h"
#include "tmon/adt7461/nvodm_tmon_adt7461_channel.h"
#include "nvodm_tmon.h"
#include "../nvodm_query_kbc_gpio_def.h"


// RTC voltage rail
static const NvOdmIoAddress s_RtcAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO2, 0 }  /* VDD_RTC -> LD02 */
};

// Core voltage rail
static const NvOdmIoAddress s_CoreAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_DCD0, 0 }  /* VDD_CORE -> SM0 */
};

// CPU voltage rail
static const NvOdmIoAddress s_ffaCpuAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_DCD1, 0 }  /* VDD_CPU -> SM1 */
};

// PLLA voltage rail
static const NvOdmIoAddress s_PllAAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_PllMAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_PllPAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLC voltage rail
static const NvOdmIoAddress s_PllCAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLE voltage rail
static const NvOdmIoAddress s_PllEAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS62290PmuSupply_BUCK, 0 } /* AVDD_PLLE -> VDD_1V05 */
};

// PLLU voltage rail
static const NvOdmIoAddress s_PllUAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDD_PLLU -> LDO1 */
};

// PLLU1 voltage rail
static const NvOdmIoAddress s_ffaPllU1Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDD_PLLU -> LDO1 */
};

// PLLS voltage rail
static const NvOdmIoAddress s_PllSAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* PLL_S -> LDO1 */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_PllHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO8, 0 } /* AVDD_HDMI_PLL -> LDO8 */
};

// OSC voltage rail
static const NvOdmIoAddress s_VddOscAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* AVDD_OSC -> LDO4 */
};

// PLLX voltage rail
static const NvOdmIoAddress s_PllXAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1, 0 } /* AVDDPLLX -> LDO1 */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_PllUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 } /* AVDD_USB_PLL -> derived from LDO3 (VDD_3V3) */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_VddSysAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDIO_SYS -> LDO4 */
};

// USB voltage rail
static const NvOdmIoAddress s_VddUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 } /* AVDD_USB -> derived from LDO3 (VDD_3V3) */
};

// HDMI voltage rail
static const NvOdmIoAddress s_VddHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO7, 0 } /* AVDD_HDMI -> LDO7 */
};

// MIPI voltage rail (DSI_CSI)
static const NvOdmIoAddress s_VddMipiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS72012PmuSupply_LDO, 0  } /* AVDD_DSI_CSI -> VDD_1V2 */
};

// LCD voltage rail
static const NvOdmIoAddress s_VddLcdAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDIO_LCD -> (LDO4PG) */
};

// Audio voltage rail
static const NvOdmIoAddress s_VddAudAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDIO_AUDIO -> (LDO4PG) */
};

// DDR voltage rail
static const NvOdmIoAddress s_VddDdrAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 }  /* VDDIO_DDR -> (LDO4PG) */
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_VddDdrRxAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO9, 0 }  /* VDDIO_RX_DDR(2.7-3.3) -> LDO9 */
};

// NAND voltage rail
static const NvOdmIoAddress s_VddNandAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 }  /* VDDIO_NAND_3V3 -> derived from LDO3 (VDD_3V3) */
};

// UART voltage rail
static const NvOdmIoAddress s_VddUartAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDIO_UART -> (LDO4PG) */
};

// SDIO voltage rail
static const NvOdmIoAddress s_VddSdioAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 } /* VDDIO_SDIO -> derived from LDO3 (VDD_3V3) */
};

// VDAC voltage rail
static const NvOdmIoAddress s_VddVdacAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO6, 0 } /* AVDD_VDAC -> LDO6 */
};

// VI voltage rail
static const NvOdmIoAddress s_VddViAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 } /* VDDIO_VI -> derived from LDO3 (VDD_3V3) */
};

// BB voltage rail
static const NvOdmIoAddress s_VddBbAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDIO_BB -> (LDO4PG) */
};

// Super power voltage rail for the SOC
static const NvOdmIoAddress s_VddSocAddresses[]=
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_SoC, 0 } /* VDD SOC */
};

// PEX_CLK voltage rail
static const NvOdmIoAddress s_VddPexClkAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO0, 0 }, /* VDDIO_PEX_CLK -> LDO0 */
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS62290PmuSupply_BUCK, 0 }, /* AVDD_PLLE -> VDD_1V05 */
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS74201PmuSupply_LDO, 0 }, /* PMU_GPIO-1 -> VDD_1V5 */
};

// PMU0
static const NvOdmIoAddress s_Pmu0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x68, 0 },
};

static const NvOdmIoAddress s_Vddio_Vid_En[] = {
    { NvOdmIoModule_Gpio, 't'-'a', 2, 0 },
};

static const NvOdmIoAddress s_Vddio_Sd_En[] = {
    { NvOdmIoModule_Gpio, 't'-'a', 3, 0 },
};

static const NvOdmIoAddress s_Vddio_Sdmmc_En[] = {
    { NvOdmIoModule_Gpio, 'i'-'a', 6, 0 },
};

static const NvOdmIoAddress s_Vddio_Bl_En[] = {
    { NvOdmIoModule_Gpio, 'w'-'a', 0, 0 },
};

static const NvOdmIoAddress s_Vddio_Pnl_En[] = {
    { NvOdmIoModule_Gpio, 'c'-'a', 6, 0 },
};

// SPI1 for Spi Ethernet Kitl only
static const NvOdmIoAddress s_SpiEthernetAddresses[] =
{
    { NvOdmIoModule_Spi, 0, 0, 0 },
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 1, 0 },  // DBQ_IRQ, Port C, Pin 1
};

// P1160 ULPI USB
static const NvOdmIoAddress s_UlpiUsbAddresses[] = 
{
    { NvOdmIoModule_ExternalClock, 1, 0, 0 }, /* ULPI PHY Clock -> DAP_MCLK2 */
};

//  LVDS LCD Display
static const NvOdmIoAddress s_LvdsDisplayAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0, 0 },
    { NvOdmIoModule_I2c, 0x00, 0xA0, 0 },
    { NvOdmIoModule_Pwm, 0x00, 0, 0 },
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 },    /* VDDIO_LCD (AON:VDD_1V8) */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 },    /* VDD_LVDS (VDD_3V3) */
};

//  HDMI addresses based on Concorde 2 design
static const NvOdmIoAddress s_HdmiAddresses[] =
{
    { NvOdmIoModule_Hdmi, 0, 0, 0 },

    // Display Data Channel (DDC) for Extended Display Identification
    // Data (EDID)
    { NvOdmIoModule_I2c, 0x01, 0xA0, 0 },

    // HDCP ROM
    { NvOdmIoModule_I2c, 0x01, 0x74, 0 },

    // AVDD_HDMI
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO7, 0 },
    // AVDD_HDMI_PLL
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO8, 0 },

    // Power for DDC (VDDIO_LCD (VDD_1V8))
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 },
    // HDMI +5V for the pull-up for DDC (VDDIO_VID)
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS2051BPmuSupply_VDDIO_VID, 0 },
};

//  Power for HDMI Hotplug
static const NvOdmIoAddress s_HdmiHotplug[] =
{
    // Power for Hotplug GPIO
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 },
    // HDMI +5V for hotplug
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS2051BPmuSupply_VDDIO_VID, 0 },
};

// CRT address based on Concorde 2 design
static const NvOdmIoAddress s_CrtAddresses[] =
{
    { NvOdmIoModule_Crt, 0, 0, 0 },

    // Display Data Channel (DDC) for Extended Display Identification
    // Data (EDID)
    // FIXME: Disable this for now since it causes some TV not display.
    { NvOdmIoModule_I2c, 0x01, 0xA0, 0 },

    /* tvdac rail */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO6, 0 },    // VDDIO_VDAC

    /* lcd rail (required for crt out) */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 },    // VDDIO_LCD (VDD_1V8)
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS2051BPmuSupply_VDDIO_VID, 0 },   // VDDIO_VGA
};

static const NvOdmIoAddress s_ffaVideoDacAddresses[] =
{
    { NvOdmIoModule_Tvo, 0x00, 0x00, 0 },
    /* tvdac rail */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO6, 0 },    // AVDD_VDAC
};

// Sdio
static const NvOdmIoAddress s_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x1,  0x0, 0 },                      /* SD Memory on SD Bus */
    { NvOdmIoModule_Sdio, 0x3,  0x0, 0 },                      /* SD Memory on SD Bus */
    { NvOdmIoModule_Vdd,  0x00, Ext_SWITCHPmuSupply_VDDIO_SD, 0 },   /* EN_VDDIO_SD */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 } /* VDDIO_SDIO -> derived from LDO3 (VDD_3V3) */
};

static const NvOdmIoAddress s_I2cSmbusAddresses[] = 
{
    { NvOdmIoModule_I2c, 2, 0x8A, 0 },
    { NvOdmIoModule_Gpio, 27, 1, 0} //Port BB:01 is used on harmony. 
};

static const NvOdmIoAddress s_UsbMuxAddress[] = 
{
    {NvOdmIoModule_Usb, 1, 0, 0}
};

static const NvOdmIoAddress s_QwertyKeyPad16x8Addresses[] =
{
    // instance = 1 indicates Column info.
    // instance = 0 indicates Row info.
    // address holds KBC pin number used for row/column.

    // All Row info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow0, 0}, // Row 0
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow1, 0}, // Row 1
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow2, 0}, // Row 2
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow3, 0}, // Row 3
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow4, 0}, // Row 4
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow5, 0}, // Row 5
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow6, 0}, // Row 6
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow7, 0}, // Row 7
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow8, 0}, // Row 8
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow9, 0}, // Row 9
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow10, 0}, // Row 10
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow11, 0}, // Row 11
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow12, 0}, // Row 12
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow13, 0}, // Row 13
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow14, 0}, // Row 14
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow15, 0}, // Row 15
    
    // All Column info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol0, 0}, // Column 0
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol1, 0}, // Column 1
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol2, 0}, // Column 2
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol3, 0}, // Column 3
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol4, 0}, // Column 4
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol5, 0}, // Column 5
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol6, 0}, // Column 6
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol7, 0}, // Column 7
};


static const NvOdmIoAddress s_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98, 0 },                  /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3, 0 },    /* TMON pwer rail -> LDO3 (VDD_3V3) */
    { NvOdmIoModule_Gpio, (NvU32)'n'-'a', 6, 0 },              /* GPIO Port N and Pin 6 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote, 0 },   /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local, 0 }, /* TSENSOR */
};

// Bluetooth
static const NvOdmIoAddress s_p1162BluetoothAddresses[] =
{
    { NvOdmIoModule_Uart, 0x2,  0x0, 0 },                  // FIXME: Is this used?
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0, 0 },          /* BT_RST#: GPIO Port U and Pin 0 */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4, 0 } /* VDDHOSTIF_BT -> LDO4 (AON:VDD_1V8) */
};

// Wlan
static const NvOdmIoAddress s_WlanAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0, 0 },                      /* WLAN is on SD Bus */
    { NvOdmIoModule_Gpio, 0xa, 0x5, 0 },                      /* GPIO Port K and Pin 5 - WIFI_PWR*/
    { NvOdmIoModule_Gpio, 0xa, 0x6, 0 },                      /* GPIO Port K and Pin 6 - WIFI_RST */
    { NvOdmIoModule_Vdd,  0x00, TPS6586xPmuSupply_LDO4, 0 },  /* VDDIO_WLAN (AON:VDD_1V8) */
    { NvOdmIoModule_Vdd,  0x00, Ext_TPS72012PmuSupply_LDO, 0 } /* VCORE_WIFI (VDD_1V2) */
};

// Audio Codec
static const NvOdmIoAddress s_AudioCodecAddresses[] = 
{
    { NvOdmIoModule_ExternalClock, 0, 0, 0 },       /* Codec MCLK -> APxx DAP_MCLK1 */
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x34, 0 },       /* Codec I2C ->  APxx PMU I2C, segment 0 */
                                                 /* Codec I2C address is 0x34 */
    { NvOdmIoModule_Gpio, (NvU32)'w'-'a', 0x02, 0 }, /* GPIO Port W and Pin 2 for HP_DET */
};

// Audio Codec on GEN1_I2C (I2C_1)
static const NvOdmIoAddress s_AudioCodecAddressesI2C_1[] = 
{
    { NvOdmIoModule_ExternalClock, 0, 0, 0 },       /* Codec MCLK -> APxx DAP_MCLK1 */
    { NvOdmIoModule_I2c, 0x00, 0x34, 0 },           /* Codec I2C ->  APxx PMU I2C, segment 0 */
                                                 /* Codec I2C address is 0x34 */
};

// TouchPanel
static const NvOdmIoAddress s_TouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x06, 0 }, /* I2C address (7-bit) 0x03<<1=0x06(8-bit)  */
    { NvOdmIoModule_Gpio, (NvU32)'d'-'a', 0x02, 0 }, /* GPIO Port D and Pin 2 */
};

static const NvOdmIoAddress s_AcceleroAddresses[] =
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x70, 0 }, /* I2C address (7-bit) 0x38<<1 = 0x70(8-bit) */
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 0x03, 0 }, /* Gpio port C and Pin 3 */
};

static const NvOdmIoAddress s_KXTFAcceleroAddresses[] =
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x1E, 0 }, /* I2C address (7-bit) 0xF<<1 = 0x1E(8-bit) */
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 0x03, 0 }, /* Gpio port C and Pin 3 */
};

