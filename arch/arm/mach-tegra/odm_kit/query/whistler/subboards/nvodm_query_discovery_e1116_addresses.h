/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1116_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1116 module
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

#include "pmu/max8907b/max8907b_supply_info_table.h"

// Persistent voltage rail (ie, for RTC, Standby, etc...)
static const NvOdmIoAddress s_ffaRtcAddresses[] =
{
    // On Maxim 8907B, the standby rail automatically follows V2
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V2, 0 }  /* VDD_RTC -> RTC */
};

// Core voltage rail
static const NvOdmIoAddress s_ffaCoreAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V2, 0 }  /* VDD_CORE -> V2 */
};

// PMU CPU voltage rail
static const NvOdmIoAddress s_ffaCpuAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V1, 0 }  /* VDD_CPU_PMU -> V1 */
};

// External CPU DCDC voltage rail
static const NvOdmIoAddress s_ffaCpuExtSupplyAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bLxV1_Ad5258_DPM_EXT_DCDC_7, 0 }  /* VDD_CPU_PMU -> DCDC7 */
};

// PLLA voltage rail
static const NvOdmIoAddress s_ffaPllAAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_ffaPllMAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLM -> VOUT2 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_ffaPllPAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLC voltage rail
static const NvOdmIoAddress s_ffaPllCAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLE voltage rail
static const NvOdmIoAddress s_ffaPllEAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLL_E -> VOUT2 */
};

// PLLU1 voltage rail
static const NvOdmIoAddress s_ffaPllU1Addresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLU -> VOUT2 */
};

// PLLS voltage rail
static const NvOdmIoAddress s_ffaPllSAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_ffaPllHdmiAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO6, 0 } /* AVDD_HDMI_PLL -> VOUT6 */
};

// OSC voltage rail
static const NvOdmIoAddress s_ffaVddOscAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* AVDD_OSC -> V3 */
};

// PLLX voltage rail
static const NvOdmIoAddress s_ffaPllXAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2, 0 } /* AVDD_PLLX -> VOUT2 */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_ffaPllUsbAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO4, 0 } /* AVDD_USB_PLL -> VOUT4 */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_ffaVddSysAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_SYS -> V3 */
};

// USB voltage rail
static const NvOdmIoAddress s_ffaVddUsbAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO4, 0 } /* AVDD_USB -> VOUT4 */
};

// HDMI voltage rail
static const NvOdmIoAddress s_ffaVddHdmiAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11, 0 } /* AVDD_HDMI -> VOUT11 */
};

// MIPI voltage rail
static const NvOdmIoAddress s_ffaVddMipiAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO17, 0 } /* VDDIO_MIPI -> VOUT17 */
};

// LCD voltage rail
static const NvOdmIoAddress s_ffaVddLcdAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_LCD_PMU -> V3 */
};

// Audio voltage rail
static const NvOdmIoAddress s_ffaVddAudAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_AUDIO -> V3 */
};

// LPDDR2 voltage rail (default)
static const NvOdmIoAddress s_ffaVddDdrAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO20, 0 }  /* VDDIO_DDR_1V2 -> VOUT20 */
};

// DDR2 voltage rail (on E1109 board ext 1.8V DCDC is controlled by LDO5)
static const NvOdmIoAddress s_ffaVddDdr2Addresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5, 0 }  /* VDDIO_DDR_1V8 -> VOUT05 */
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_ffaVddDdrRxAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO1, 0 }  /* VDDIO_RX_DDR(2.7-3.3) -> VOUT1 */
};

// NAND voltage rail
static const NvOdmIoAddress s_ffaVddNandAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_NAND_PMU -> V3 */
};

// UART voltage rail
static const NvOdmIoAddress s_ffaVddUartAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_UART -> V3 */
};

// SDIO voltage rail
static const NvOdmIoAddress s_ffaVddSdioAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12, 0 } /* VDDIO_SDIO -> VOUT12 */
};

// VDAC voltage rail
static const NvOdmIoAddress s_ffaVddVdacAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO14, 0 } /* AVDD_VDAC -> VOUT14 */
};

// VI voltage rail
static const NvOdmIoAddress s_ffaVddViAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO18, 0 } /* VDDIO_VI -> VOUT18 */
};

// BB voltage rail
static const NvOdmIoAddress s_ffaVddBbAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* VDDIO_BB -> V3 */
};

// HSIC voltage rail
static const NvOdmIoAddress s_ffaVddHsicAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO20, 0 } /* VDDIO_HSIC -> VOUT20 */
};

// USB_IC voltage rail
static const NvOdmIoAddress s_ffaVddUsbIcAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3, 0 } /* AVDD_USB_IC -> V3 */
};

// PEX_CLK voltage rail
static const NvOdmIoAddress s_ffaVddPexClkAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11, 0 }, /* VDDIO_PEX_CLK -> VOUT11 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12, 0 }, /* VDDIO_PEX_CLK -> VOUT12 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3, 0 }
};

// PMU0
static const NvOdmIoAddress s_Pmu0Addresses[] =
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x78, 0 },
};

// I2C IO Expander
static const NvOdmIoAddress s_I2cioexpanderAddress[] =
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x40, 0 },
};

// USB1 VBus voltage rail
static const NvOdmIoAddress s_ffaVddUsb1VBusAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3_USB1, 0 },
};

// USB3 VBus voltage rail
static const NvOdmIoAddress s_ffaVddUsb3VBusAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3_USB3, 0 },
};

// FUSE voltage enablel
static const NvOdmIoAddress s_ffaVddFuseAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_VBAT_FUSE, 0 },
};

