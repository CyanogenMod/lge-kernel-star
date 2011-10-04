/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/nvodm_query_pinmux.c
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

/*
 * This file implements the pin-mux configuration tables for each I/O module.
 */

// THESE SETTINGS ARE PLATFORM-SPECIFIC (not SOC-specific).
// PLATFORM = AP20 Whistler/Voyager

#include "nvodm_query_pinmux.h"
#include "nvassert.h"
#include "nvodm_services.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"

#define NVODM_PINMUX_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const NvU32 s_NvOdmPinMuxConfig_Uart_Hsi_Ulpi[] = {
    NvOdmUartPinMap_Config7,    // Instance 0: UART-A is mapped to SDIO1 pins when ULPI is used
    NvOdmUartPinMap_Config1,    // Instance 1: UART-B
    NvOdmUartPinMap_Config1,    // Instance 2: UART-C
    0, // UART-D function disabled: pins used by BB (SPI1)
    0, // UART-E function disabled: pins used by WiFi (SDIO1)
};

static const NvU32 s_NvOdmPinMuxConfig_Uart_Ril_Emp[] = {
    NvOdmUartPinMap_Config6,    // Instance 0: UART-A is mapped to UAA pin group.
    NvOdmUartPinMap_Config1,    // Instance 1: UART-B
    NvOdmUartPinMap_Config1,    // Instance 2: UART-C
    0, // UART-D function disabled: pins used by BB (SPI1)
    0, // UART-E function disabled: pins used by WiFi (SDIO1)
};

static const NvU32 s_NvOdmPinMuxConfig_Uart[] = {
    NvOdmUartPinMap_Config6,
    NvOdmUartPinMap_Config2, // Debug message
    NvOdmUartPinMap_Config1, // BT
#if defined (CONFIG_MACH_STAR) && !defined(CONFIG_MACH_STAR_REV_F)
// mdm does not use ap side - gps
    0,
#else
    NvOdmUartPinMap_Config2, // GPS
#endif
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Spi[] = {
    NvOdmSpiPinMap_Config1, // Modem
//20100809-1, , Add SPI2 for AP-CP IPC [START]
#ifdef CONFIG_DUAL_SPI
	NvOdmSpiPinMap_Config4, // Modem
#else    
		0,
#endif
//20100809, , Add SPI2 for AP-CP IPC [END]
    0,
    0,
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Twc[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_I2c[] = {
    NvOdmI2cPinMap_Config1,	 // MUIC, Touch, LCD backlight IC
    NvOdmI2cPinMap_Multiplexed,  // For HDMI(5V tolerant), Gyro, Accelerometer, Compass, Proximity, CODEC
    NvOdmI2cPinMap_Config1,	 // 8MP camera, Piezo Actuator,  1,3M Camera, Camera PMU
};

static const NvU32 s_NvOdmPinMuxConfig_I2c_Pmu[] = {
    NvOdmI2cPmuPinMap_Config1,
};

static const NvU32 s_NvOdmPinMuxConfig_Ulpi[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Sdio[] = {
    NvOdmSdioPinMap_Config1,	// WiFi
    0,    
    NvOdmSdioPinMap_Config2,	// microSD
    NvOdmSdioPinMap_Config2,    // eMMC primary 8bit
};

static const NvU32 s_NvOdmPinMuxConfig_Spdif[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Hsi[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Hdmi[] = {
    NvOdmHdmiPinMap_Config1,
};

static const NvU32 s_NvOdmPinMuxConfig_Pwm[] = {
    NvOdmPwmPinMap_Config1,	// GPIO_PU6 for LCD backlight, GPIO_PU[5:3] must be set as GPIO to overwirte PWM pinmux
};

static const NvU32 s_NvOdmPinMuxConfig_Ata[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Nand[] = {
    0,  
};

static const NvU32 s_NvOdmPinMuxConfig_Dap[] = {
    NvOdmDapPinMap_Config1,
    NvOdmDapPinMap_Config1,
    NvOdmDapPinMap_Config1,
    NvOdmDapPinMap_Config1,
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Kbd[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Hdcp[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_SyncNor[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Mio[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_ExternalClock[] = {
    NvOdmExternalClockPinMap_Config1, //CDEV1 MCLK1. Config1 =  PLLA_OUT
    NvOdmExternalClockPinMap_Config1,  //CDEV2 MCLK2, Config1 = AHB_CLK
    NvOdmExternalClockPinMap_Config1, // CSUS VI_MCLK
};

static const NvU32 s_NvOdmPinMuxConfig_VideoInput[] = {
    NvOdmVideoInputPinMap_Config2,
};

static const NvU32 s_NvOdmPinMuxConfig_Display[] = {
//20100725  add CPU Panel [START]
#if defined(CONFIG_MACH_STAR)
    NvOdmDisplayPinMap_Config1,
#else
    0, // RGB panel is not used
#endif
//20100725  add CPU Panel [END]
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_BacklightPwm[] = {
    0, //NvOdmBacklightPwmPinMap_Config3,
    0,
    0,  
    0,                                  
};

static const NvU32 s_NvOdmPinMuxConfig_Crt[] = {
    0,
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_Tvo[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_OneWire[] = {
    0,
};

static const NvU32 s_NvOdmPinMuxConfig_PciExpress[] = {
    0, // To enable Pcie, set pinmux config for SDIO3 to 0
};

static const NvU32 s_NvOdmClockLimit_Sdio[] = {
    50000,  // WiFi , BCM4329 is limited with max 25MHz.
    0,      // not used
    50000,  // Micro SD;  reduced due to a few SDHC vendor compatibility issue for now
    50000,  // eMMC
};

static const NvU32 s_NvOdmPinMuxConfig_Ptm[] = {
    0,
};

///TODO: keep it here for now and might remove it.
static const NvU32 s_NvOdmPinMuxConfig_Dsi[] = {
//20100725  add CPU Panel [START]
#if defined(CONFIG_MACH_STAR)
    0,
#else
    NvOdmDapPinMap_Config1, // fake one, otherwise, ddk display will assert.
#endif
//20100725  add CPU Panel [END]
};


void
NvOdmQueryPinMux(
    NvOdmIoModule IoModule,
    const NvU32 **pPinMuxConfigTable,
    NvU32 *pCount)
{
	//20101023  add tegra-10.9.3[start] 
	NvU32 CustomerOption = 0;
	NvU32 Personality = 0;
	NvU32 Ril = 0;
	NvOdmServicesKeyListHandle hKeyList;
	hKeyList = NvOdmServicesKeyListOpen();
	if (hKeyList)
    {   
        CustomerOption =
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
        Personality =
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, PERSONALITY, CustomerOption);
    Ril =
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, RIL, CustomerOption);
    }   

    if (!Personality)
        Personality = TEGRA_DEVKIT_DEFAULT_PERSONALITY;

    if (!Ril)
        Ril = TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_DEFAULT;	
	//20101023  add tegra-10.9.3[end] 

    switch (IoModule)
    {
    case NvOdmIoModule_Display:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Display;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Display);
        break;

    case NvOdmIoModule_Dap:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Dap;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Dap);
        break;

    case NvOdmIoModule_Hdcp:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Hdcp;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Hdcp);
        break;

    case NvOdmIoModule_Hdmi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Hdmi;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Hdmi);
        break;

    case NvOdmIoModule_I2c:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_I2c;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_I2c);
        break;

    case NvOdmIoModule_I2c_Pmu:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_I2c_Pmu;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_I2c_Pmu);
        break;

    case NvOdmIoModule_Kbd:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Kbd;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Kbd);
        break;

    case NvOdmIoModule_Mio:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Mio;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Mio);
        break;

    case NvOdmIoModule_Nand:
            *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Nand;
            *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Nand);
        break;

    case NvOdmIoModule_Sdio:
            *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Sdio;
            *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Sdio);
        break;

    case NvOdmIoModule_Spdif:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Spdif;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Spdif);
        break;

    case NvOdmIoModule_Spi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Spi;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Spi);
        break;

    case NvOdmIoModule_Uart:
        if (Ril == TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW_ULPI)
        {
            *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Uart_Hsi_Ulpi;
            *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Uart_Hsi_Ulpi);
        }
        else if (Ril == TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW)
        {
            *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Uart_Ril_Emp;
            *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Uart_Ril_Emp);
        }
        else
        {
            *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Uart;
            *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Uart);
		}
        break;

    case NvOdmIoModule_ExternalClock:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_ExternalClock;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_ExternalClock);
        break;

    case NvOdmIoModule_VideoInput:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_VideoInput;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_VideoInput);
        break;

    case NvOdmIoModule_Crt:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Crt;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Crt);
        break;

    case NvOdmIoModule_Tvo:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Tvo;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Tvo);
        break;

    case NvOdmIoModule_Ata:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Ata;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Ata);
        break;

    case NvOdmIoModule_Pwm:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Pwm;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Pwm);
        break;

    case NvOdmIoModule_Hsi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Hsi;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Hsi);
        break;

    case NvOdmIoModule_Twc:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Twc;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Twc);
        break;

    case NvOdmIoModule_Ulpi:
        *pPinMuxConfigTable = NULL;
        *pCount = 0;
        break;

    case NvOdmIoModule_OneWire:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_OneWire;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_OneWire);
        break;

    case NvOdmIoModule_SyncNor:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_SyncNor;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_SyncNor);
        break;

    case NvOdmIoModule_PciExpress:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_PciExpress;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_PciExpress);
        break;

    case NvOdmIoModule_Trace:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Ptm;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Ptm);
        break;

    case NvOdmIoModule_BacklightPwm:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_BacklightPwm;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_BacklightPwm);
        break;

//20100725  add CPU Panel [START]
#if !defined(CONFIG_MACH_STAR)
    case NvOdmIoModule_Dsi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Dsi;
        *pCount = NV_ARRAY_SIZE(s_NvOdmPinMuxConfig_Dsi);
        break;
#endif
//20100725  add CPU Panel [END]

    case NvOdmIoModule_Hsmmc:
    case NvOdmIoModule_Csi:
//20100725  add CPU Panel [START]
#if defined(CONFIG_MACH_STAR)
    case NvOdmIoModule_Dsi:
#endif
//20100725  add CPU Panel [END]
    case NvOdmIoModule_Sflash:
    case NvOdmIoModule_Slink:
    case NvOdmIoModule_Gpio:
    case NvOdmIoModule_I2s:
    case NvOdmIoModule_Usb:
    case NvOdmIoModule_Vdd:
    case NvOdmIoModule_Xio:
    case NvOdmIoModule_Tsense:
        *pCount = 0;
        break;

    default:
        NV_ASSERT(!"Bad Parameter!");
        *pCount = 0;
        break;
    }
}

void
NvOdmQueryClockLimits(
    NvOdmIoModule IoModule,
    const NvU32 **pClockSpeedLimits,
    NvU32 *pCount)
{
    switch (IoModule)
    {
    case NvOdmIoModule_Hsmmc:
        *pCount = 0;
        break;

    case NvOdmIoModule_Sdio:
        *pClockSpeedLimits = s_NvOdmClockLimit_Sdio;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmClockLimit_Sdio);
        break;


    default:
        *pClockSpeedLimits = NULL;
        *pCount = 0;
        break;
    }
}

