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
 *                 database NvOdmIoAddress entries for the E1116
 *                 Power module.
 */

#include "pmu/max8907b/max8907b_supply_info_table.h"

// Persistent voltage rail (ie, for RTC, Standby, etc...)
static const NvOdmIoAddress s_ffaRtcAddresses[] = 
{
    // On Maxim 8907B, the standby rail automatically follows V2
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V2 }  /* VDD_RTC -> RTC */
};

// Core voltage rail
static const NvOdmIoAddress s_ffaCoreAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V2 }  /* VDD_CORE -> V2 */
};

// PMU CPU voltage rail
static const NvOdmIoAddress s_ffaCpuAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V1 }  /* VDD_CPU_PMU -> V1 */
};

// External CPU DCDC voltage rail
static const NvOdmIoAddress s_ffaCpuExtSupplyAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bLxV1_Ad5258_DPM_EXT_DCDC_7 }  /* VDD_CPU_PMU -> DCDC7 */
};

// PLLA voltage rail
static const NvOdmIoAddress s_ffaPllAAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_ffaPllMAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLM -> VOUT2 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_ffaPllPAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLC voltage rail
static const NvOdmIoAddress s_ffaPllCAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLE voltage rail
static const NvOdmIoAddress s_ffaPllEAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLL_E -> VOUT2 */
};

// PLLU1 voltage rail
static const NvOdmIoAddress s_ffaPllU1Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLU -> VOUT2 */
};

// PLLS voltage rail
static const NvOdmIoAddress s_ffaPllSAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_ffaPllHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO6 } /* AVDD_HDMI_PLL -> VOUT6 */
};

// OSC voltage rail
static const NvOdmIoAddress s_ffaVddOscAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* AVDD_OSC -> V3 */
};

// PLLX voltage rail
static const NvOdmIoAddress s_ffaPllXAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO2 } /* AVDD_PLLX -> VOUT2 */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_ffaPllUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO4 } /* AVDD_USB_PLL -> VOUT4 */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_ffaVddSysAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_SYS -> V3 */
};

// USB voltage rail
static const NvOdmIoAddress s_ffaVddUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO4 } /* AVDD_USB -> VOUT4 */
};

// HDMI voltage rail
static const NvOdmIoAddress s_ffaVddHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11 } /* AVDD_HDMI -> VOUT11 */
};

// MIPI voltage rail
static const NvOdmIoAddress s_ffaVddMipiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO17 } /* VDDIO_MIPI -> VOUT17 */
};

// LCD voltage rail
static const NvOdmIoAddress s_ffaVddLcdAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_LCD_PMU -> V3 */
};

// Audio voltage rail
static const NvOdmIoAddress s_ffaVddAudAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_AUDIO -> V3 */
};

// LPDDR2 voltage rail (default)
static const NvOdmIoAddress s_ffaVddDdrAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO20 }  /* VDDIO_DDR_1V2 -> VOUT20 */
};

// DDR2 voltage rail (on E1109 board ext 1.8V DCDC is controlled by LDO5)
static const NvOdmIoAddress s_ffaVddDdr2Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5 }  /* VDDIO_DDR_1V8 -> VOUT05 */
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_ffaVddDdrRxAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO1 }  /* VDDIO_RX_DDR(2.7-3.3) -> VOUT1 */
};

// NAND voltage rail
static const NvOdmIoAddress s_ffaVddNandAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_NAND_PMU -> V3 */
};

// UART voltage rail
static const NvOdmIoAddress s_ffaVddUartAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_UART -> V3 */
};

// SDIO voltage rail
static const NvOdmIoAddress s_ffaVddSdioAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12 } /* VDDIO_SDIO -> VOUT12 */
};

// VDAC voltage rail
static const NvOdmIoAddress s_ffaVddVdacAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO14 } /* AVDD_VDAC -> VOUT14 */
};

// VI voltage rail
static const NvOdmIoAddress s_ffaVddViAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO18 } /* VDDIO_VI -> VOUT18 */
};

// BB voltage rail
static const NvOdmIoAddress s_ffaVddBbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* VDDIO_BB -> V3 */
};

// HSIC voltage rail
static const NvOdmIoAddress s_ffaVddHsicAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO20 } /* VDDIO_HSIC -> VOUT20 */
};

// USB_IC voltage rail
static const NvOdmIoAddress s_ffaVddUsbIcAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 } /* AVDD_USB_IC -> V3 */
};

// PEX_CLK voltage rail
static const NvOdmIoAddress s_ffaVddPexClkAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11 }, /* VDDIO_PEX_CLK -> VOUT11 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12 }, /* VDDIO_PEX_CLK -> VOUT12 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3 } 
};

// PMU0
static const NvOdmIoAddress s_Pmu0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x78 },
};

// I2C IO Expander
static const NvOdmIoAddress s_I2cioexpanderAddress[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x40 },
};

// USB1 VBus voltage rail
static const NvOdmIoAddress s_ffaVddUsb1VBusAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3_USB1 },
};

// USB3 VBus voltage rail
static const NvOdmIoAddress s_ffaVddUsb3VBusAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_EXT_DCDC_3_USB3 },
};

