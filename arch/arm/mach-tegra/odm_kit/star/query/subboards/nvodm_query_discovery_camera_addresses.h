/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
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

#include "pmu/max8907/max8907_supply_info_table.h"

#define NVODM_PORT(x) ((x) - 'a')
#define SN046F_PINS (NVODM_CAMERA_VGP5_RESET_AL | \
                     NVODM_CAMERA_PARALLEL_VD0_TO_VD9 | \
                     NVODM_CAMERA_DEVICE_IS_DEFAULT)
static const NvOdmIoAddress s_ffaImagerSN046FAddresses[] =
{
    { NvOdmIoModule_I2c,  0x02, 0x34, 0 }, 
    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 5, 0 },  // Reset
//    { NvOdmIoModule_Vdd,  0x00, PCF50626PmuSupply_D4REG, 0 }, // 1.2V
//    { NvOdmIoModule_Vdd,  0x00, PCF50626PmuSupply_RF3REG, 0 }, // 1.8V for I/O voltage(I2C & reset voltage)

    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO18, 0 }, //VDDIO_VI
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO9, 0 }, //AVDD_CAM1
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO13, 0 }, //VDDIO_AF
    { NvOdmIoModule_Vdd,  0x00, Max8907PmuSupply_LDO17, 0 }, //VDDIO_MIPI7
    { NvOdmIoModule_VideoInput, 0x00, SN046F_PINS, 0 },
    { NvOdmIoModule_ExternalClock, 2, 0, 0 }, // CSUS
};

static const NvOdmIoAddress s_ffaImagerDW9712Addresses[] =
{
    { NvOdmIoModule_I2c,  0x02, 0x18, 0 },  // focuser i2c
//    { NvOdmIoModule_Gpio, NVODM_GPIO_CAMERA_PORT, 6, 0 },  // Need to Focuser enabled
};
static const NvOdmIoAddress s_CommonImagerAddresses[] =
{
    { NvOdmIoModule_ExternalClock, 2, 0, 0 }, // CSUS
};

// ===> camera

