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

/*
 * This file implements the pin-mux configuration tables for each I/O module.
 */

// THESE SETTINGS ARE PLATFORM-SPECIFIC (not SOC-specific).
// PLATFORM = Harmony

#include "nvodm_query_pinmux.h"
#include "nvassert.h"
#include "nvodm_query.h"
#include "nvodm_services.h"

#define NVODM_PINMUX_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const NvU32 s_NvOdmPinMuxConfig_Uart[] = {
    NvOdmUartPinMap_Config4,    // UART1, 2 lines
    NvOdmUartPinMap_Config2,    // UART2, 2 lines
    NvOdmUartPinMap_Config1,    // UART3, 4 lines
    NvOdmUartPinMap_Config2,    // UART4, 4 lines
    0                           // UART5
};

static const NvU32 s_NvOdmPinMuxConfig_Spi[] = {
    NvOdmSpiPinMap_Config4,
    0,
    0,
    0,
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Twc[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_I2c[] = {
    NvOdmI2cPinMap_Config1,
    NvOdmI2cPinMap_Config1,
    NvOdmI2cPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_I2cPmu[] = {
    NvOdmI2cPmuPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_Ulpi[] = {
    NvOdmUlpiPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_Sdio[] = {
    NvOdmSdioPinMap_Config1, 
    NvOdmSdioPinMap_Config5,
    0,
    NvOdmSdioPinMap_Config2
};

static const NvU32 s_NvOdmPinMuxConfig_Spdif[] = {
    NvOdmSpdifPinMap_Config2
};

static const NvU32 s_NvOdmPinMuxConfig_Hsi[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Hdcp[] = {
    NvOdmHdcpPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_Hdmi[] = {
    NvOdmHdmiPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_Pwm[] = {
    NvOdmPwmPinMap_Config6
};

static const NvU32 s_NvOdmPinMuxConfig_Ata[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Nand[] = {
    NvOdmNandPinMap_Config4, 
};

static const NvU32 s_NvOdmPinMuxConfig_Dap[] = {
    NvOdmDapPinMap_Config1,
    NvOdmDapPinMap_Config1, 
    0, 
    NvOdmDapPinMap_Config1,
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Kbd[] = {
    NvOdmKbdPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_SyncNor[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Mio[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_ExternalClock[] = {
    NvOdmExternalClockPinMap_Config2,  
    NvOdmExternalClockPinMap_Config3,
    NvOdmExternalClockPinMap_Config1
};

static const NvU32 s_NvOdmPinMuxConfig_VideoInput[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Display[] = {
    NvOdmDisplayPinMap_Config1,
    0  // Only 1 display is connected to the LCD pins
};

static const NvU32 s_NvOdmPinMuxConfig_BacklightPwm[] = {
    0,
    0
};

static const NvU32 s_NvOdmPinMuxConfig_Crt[] = {
    NvOdmCrtPinMap_Config1,
};

static const NvU32 s_NvOdmPinMuxConfig_Tvo[] = {
    NvOdmTvoPinMap_Config1,
};

static const NvU32 s_NvOdmPinMuxConfig_OneWire[] = {
    0
};

static const NvU32 s_NvOdmPinMuxConfig_PciExpress[] = {
    NvOdmPciExpressPinMap_Config1,
};

void
NvOdmQueryPinMux(
    NvOdmIoModule IoModule,
    const NvU32 **pPinMuxConfigTable,
    NvU32 *pCount)
{
    switch (IoModule)
    {
    case NvOdmIoModule_Display:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Display;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Display);
        break;

    case NvOdmIoModule_Dap:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Dap;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Dap);
        break;

    case NvOdmIoModule_Hdcp:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Hdcp;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Hdcp);
        break;

    case NvOdmIoModule_Hdmi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Hdmi;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Hdmi);
        break;

    case NvOdmIoModule_I2c:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_I2c;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_I2c);
        break;

    case NvOdmIoModule_I2c_Pmu:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_I2cPmu;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_I2cPmu);
        break;

    case NvOdmIoModule_Nand:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Nand;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Nand);
        break;

    case NvOdmIoModule_Sdio:
       *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Sdio;
       *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Sdio);
        break;

    case NvOdmIoModule_Spi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Spi;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Spi);
        break;

    case NvOdmIoModule_Uart:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Uart;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Uart);
        break;

    case NvOdmIoModule_ExternalClock:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_ExternalClock;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_ExternalClock);
        break;
    
    case NvOdmIoModule_Crt:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Crt;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Crt);
        break;

    case NvOdmIoModule_PciExpress:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_PciExpress;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_PciExpress); 
        break;
        
    case NvOdmIoModule_Tvo:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Tvo;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Tvo);
        break;
    case NvOdmIoModule_BacklightPwm:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_BacklightPwm;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_BacklightPwm);
        break;

    case NvOdmIoModule_Pwm:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Pwm;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Pwm);
        break;

    case NvOdmIoModule_Ulpi:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Ulpi;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Ulpi);
        break;

    case NvOdmIoModule_Spdif:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Spdif;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Spdif);
        break;

    case NvOdmIoModule_Kbd:
        *pPinMuxConfigTable = s_NvOdmPinMuxConfig_Kbd;
        *pCount = NVODM_PINMUX_ARRAY_SIZE(s_NvOdmPinMuxConfig_Kbd);
        break;

    case NvOdmIoModule_Twc:
    case NvOdmIoModule_Hsi:
    case NvOdmIoModule_Ata:
    case NvOdmIoModule_SyncNor:
    case NvOdmIoModule_Mio:
    case NvOdmIoModule_VideoInput:
    case NvOdmIoModule_OneWire:
        *pPinMuxConfigTable = NULL;
        *pCount = 0;
        break;

    default:
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
        default:
            *pClockSpeedLimits = NULL;
            *pCount = 0;
            break;
    }
}

