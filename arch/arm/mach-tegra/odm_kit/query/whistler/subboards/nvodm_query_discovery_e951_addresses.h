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
 *                 database NvOdmIoAddress entries for the E951
 *                 COMMS module.
 */

#include "pmu/max8907b/max8907b_supply_info_table.h"
// Bluetooth
static const NvOdmIoAddress ffaBluetoothAddresses[] =
{
    { NvOdmIoModule_Uart, 0x2,  0x0, 0 },
    { NvOdmIoModule_Gpio, 0x14, 0x0, 0 },     /* GPIO Port U and Pin 0 */
    { NvOdmIoModule_Vdd, 0x00, MIC2826PmuSupply_LDO3, 0 }  /* VDD -> MIC2826 LDO3 */
};


// Wlan
static const NvOdmIoAddress s_ffaWlanAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x01, 0x0, 0 },    /* WLAN is on SD Bus */
    { NvOdmIoModule_Gpio, 0x0a, 0x5, 0 },    /* GPIO Port K and Pin 5 */
    { NvOdmIoModule_Gpio, 0x0a, 0x6, 0 },    /* GPIO Port K and Pin 6 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO8, 0 },  /* VDD -> LDO8 (VOUT8) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11, 0 },  /* VDD -> LDO11 (VOUT11) */
    { NvOdmIoModule_Vdd, 0x00, MIC2826PmuSupply_LDO3, 0 }  /* VDD -> MIC2826 LDO3 */    
};

// EMP Rainbow module
static const NvOdmIoAddress s_ffaEmpAddresses[] =
{
    { NvOdmIoModule_Uart, 0x0,  0x0, 0 },                      /* UART 0 */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 },                      /* GPIO Port V and Pin 1 Power */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 AWR */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 CWR */
    { NvOdmIoModule_Gpio, 0x0e, 0x6, 0 },                      /* GPIO Port O and Pin 6 SpiInt */
    { NvOdmIoModule_Gpio, 0x15, 0x2, 0 },                      /* GPIO Port V and Pin 2 SpiSlaveSelect */
    { NvOdmIoModule_Slink, 0x0,  0x0, 0 }                      /* Slink 0 */
};

// EMP M570 module
static const NvOdmIoAddress s_ffaEmpM570Addresses[] =
{
    { NvOdmIoModule_Uart, 0x0,  0x0, 0 },                      /* UART 0 */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 },                      /* GPIO Port V and Pin 1 Power */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 AWR */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 CWR */
};

// IFX Modem module
static const NvOdmIoAddress s_ffaInfnAddresses[] =
{
    { NvOdmIoModule_Spi, 0x0,  0x0, 0 },                      /* Spi Controller 0 and Chip Select 0 */
    { NvOdmIoModule_Gpio, 0x18, 0x6, 0 },                      /* GPIO Port Y and Pin 6 SRDY */
    { NvOdmIoModule_Gpio, 0x19, 0x0, 0 },                      /* GPIO Port Z and Pin 0 MRDY */
    { NvOdmIoModule_Gpio, 0x15, 0x0, 0 },                      /* GPIO Port V and Pin 0 Reset */
    { NvOdmIoModule_Gpio, 0x15, 0x1, 0 }                      /* GPIO Port V and Pin 1 Power */
};

