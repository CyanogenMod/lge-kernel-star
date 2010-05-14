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
 * @b Description: Specifies the peripheral connectivity database NvOdmIoAddress entries
 *                 for the E911 5MP + 1.3MP + VGA VI Camera module.
 */
#include "nvodm_query_gpio.h"
#include "../include/nvodm_imager_guids.h"
#include "pmu/max8907b/max8907b_supply_info_table.h"

#define NVODM_PORT(x) ((x) - 'a')
/* VGP5 is apparently inverted on some boards.
 * For E912- A01, you may need to change the VGP5_RESET_AL line to:
 * NVODM_CAMERA_VGP5_RESET
 * If you find other boards for which it needs to be inverted, please
 * add your information to this comment.
 */
#define OV5630_PINS (NVODM_CAMERA_SERIAL_CSI_D1A | \
                     NVODM_CAMERA_DEVICE_IS_DEFAULT)
static const NvOdmIoAddress s_ffaImagerOV5630Addresses[] =
{
    { NvOdmIoModule_I2c,  0x02, 0x6C }, 
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 5 | NVODM_IMAGER_RESET_AL },
    { NvOdmIoModule_Gpio, NVODM_PORT('t'), 3 | NVODM_IMAGER_POWERDOWN },
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO18 }, //VDDIO_VI
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO9 }, //AVDD_CAM1
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO13 }, //VDDIO_AF
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO17 }, //VDDIO_MIPI
    { NvOdmIoModule_VideoInput, 0x00, OV5630_PINS },
    { NvOdmIoModule_ExternalClock, 2, 0 } // CSUS
};

// OV5630 focuser
static const NvOdmIoAddress s_ffaImagerAD5820Addresses[] =
{
    { NvOdmIoModule_I2c,  0x02, 0x18 },  // focuser i2c
};

// OV5630 flash
static const NvOdmIoAddress s_ffaFlashLTC3216Addresses[] =
{
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 3 | NVODM_IMAGER_FLASH0 },  // Flash 200mA
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 6 | NVODM_IMAGER_FLASH1 }   // Flash 600mA
};

// For SEMCO VGA
#define SOC380_PINS (NVODM_CAMERA_PARALLEL_VD0_TO_VD7)
static const NvOdmIoAddress s_ffaImagerSOC380Addresses[] =
{
    { NvOdmIoModule_I2c,  0x02, 0x78 },
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 4 | NVODM_IMAGER_POWERDOWN_AL },
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 0 | NVODM_IMAGER_RESET_AL },
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO18}, //VDDIO_VI
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO9 }, //AVDD_CAM2
    { NvOdmIoModule_Vdd,  0x00, Max8907bPmuSupply_LDO13}, //VDDIO_AF
    { NvOdmIoModule_VideoInput, 0x00, SOC380_PINS },
    { NvOdmIoModule_ExternalClock, 2, 0 } // CSUS
};

static const NvOdmIoAddress s_CommonImagerAddresses[] =
{
    { NvOdmIoModule_ExternalClock, 2, 0 } // CSUS
};


