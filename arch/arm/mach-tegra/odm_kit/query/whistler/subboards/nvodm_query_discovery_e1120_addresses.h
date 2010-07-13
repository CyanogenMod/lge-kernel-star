

/*
 * Copyright (c) 2010 NVIDIA Corporation.
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
 *                 database NvOdmIoAddress entries for the E1120
 *                 AP20 Development System Motherboard.
 */

#include "pmu/max8907b/max8907b_supply_info_table.h"

static const NvOdmIoAddress s_enc28j60EthernetAddresses[] =
{
    { NvOdmIoModule_Spi, 1, 1, 0 },
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 1, 0 }
};

static const NvOdmIoAddress s_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0, 0 },
    { NvOdmIoModule_Sdio, 0x2, 0x0, 0 },
    { NvOdmIoModule_Sdio, 0x3, 0x0, 0 },
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12, 0 }, /* VDDIO_SDIO -> VOUT12 */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5, 0 } /* VCORE_MMC -> VOUT05 */
};

static const NvOdmIoAddress s_VibAddresses[] =
{
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LDO16, 0 },
};

static const NvOdmIoAddress s_AcceleroAddresses[] =
{
    { NvOdmIoModule_I2c, 0x0, 0x3A, 0 }, /* I2C address (7-bit) 0x1D < 1 = 0x3A (8-bit) */
    { NvOdmIoModule_Gpio, 0x1A, 0x1, 0 }, /* Gpio port AA[1] = (A=0, Z=25) thus AA = 26 = 0x1A */
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LX_V3, 0 }, /* VDDIO_UART = V3 */
    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LDO1, 0 }, /* VCORE_ACC = VOUT1 = 2.8v */
};

