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

#include "pmu/max8907b/max8907b_supply_info_table.h"
#include "tmon/adt7461/nvodm_tmon_adt7461_channel.h"
#include "nvodm_tmon.h"

static const NvOdmIoAddress s_ffaHdmiAddresses[] =
{
    { NvOdmIoModule_Hdmi, 0, 0 },

    /* Display Data Channel (DDC) for Extended Display Identification
     * Data (EDID)
     */
    { NvOdmIoModule_I2c, 0x01, 0xA0 },

    /* HDCP downstream */
    { NvOdmIoModule_I2c, 0x01, 0x74 },

    /* AVDD_HDMI -> D1REG */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO11 },

    /* MIPI PLL */
    { NvOdmIoModule_Vdd, 0, Max8907bPmuSupply_LDO6 },

    /* lcd i/o rail (for hot plug pin) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 },
};

static const NvOdmIoAddress s_ffaCrtAddresses[] =
{
    { NvOdmIoModule_Crt, 0, 0 },

    /* Display Data Channel (DDC) for Extended Display Identification
     * Data (EDID)
     */
    { NvOdmIoModule_I2c, 0x01, 0xA0 },

    /* tvdac rail (required) */
    { NvOdmIoModule_Vdd, 0x00,  Max8907bPmuSupply_LDO14 },

    /* lcd i/o rail (for hot plug pin) */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LX_V3 },
};

static const NvOdmIoAddress s_ffaVideoDacAddresses[] =
{
    { NvOdmIoModule_Tvo, 0x00, 0x00 },
    /* tvdac rail */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO14 },
};

static const NvOdmIoAddress s_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98 }, /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO15 }, /* TMON pwer rail -> D4REG */
    { NvOdmIoModule_Gpio, 0x08, 0x02 },                   /* GPIO Port I and Pin 2 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote },   /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local }, /* TSENSOR */
};

