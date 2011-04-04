/*
 * Copyright (c) 2007 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVODM_QUERY_DISCOVERY_IMAGER_H
#define NVODM_QUERY_DISCOVERY_IMAGER_H


#include "nvodm_query_discovery.h"

#if defined(__cplusplus)
extern "C"
{
#endif


// SONY IU046F2-Z
#define SN046F_GUID NV_ODM_GUID('s','_','S','N','0','4','6','F')
#define FOCUSER_GUID NV_ODM_GUID('f','_','D','W','9','7','1','2')
// E912-A02 and Concorde2 Sensors
#define OV5630_GUID NV_ODM_GUID('s','_','O','V','5','6','3','0')
/// focuser?

// GUIDS
#define SENSOR_BAYER_GUID   NV_ODM_GUID('s','_','S','M','P','B','A','Y')
#define SENSOR_YUV_GUID     NV_ODM_GUID('s','_','S','M','P','Y','U','V')
//#define FOCUSER_GUID        NV_ODM_GUID('f','_','S','M','P','F','O','C')
#define FLASH_GUID          NV_ODM_GUID('f','_','S','M','P','F','L','A')

// for specific sensors
#define SENSOR_BAYER_OV5630_GUID NV_ODM_GUID('s','_','O','V','5','6','3','0')
#define SENSOR_BAYER_OV5650_GUID NV_ODM_GUID('s','_','O','V','5','6','5','0')

// Pin Use Codes:
// VI/CSI Parallel and Serial Pins and GPIO Pins

// More than one device may be retrieved thru the query
#define NVODM_CAMERA_DEVICE_IS_DEFAULT      (1)

// The imager devices can connect to the parallel bus or the serial bus
// Parallel connections use pins VD0 thru VD9.
// Serial connections use the mipi pins (ex: CSI_D1AN/CSI_D1AP)
#define NVODM_CAMERA_DATA_PIN_SHIFT         (1)
//#define NVODM_CAMERA_DATA_PIN_MASK          (0x0F)
#define NVODM_CAMERA_PARALLEL_VD0_TO_VD9    (1 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_PARALLEL_VD0_TO_VD7    (2 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1A         (4 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D2A         (5 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1A_D2A     (6 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1B         (7 << NVODM_CAMERA_DATA_PIN_SHIFT)

// Switching the encoding from the VideoInput module address to use with
// each GPIO module address.
// NVODM_IMAGER_GPIO will tell the nvodm imager how to use each gpio
// A gpio can be used for powerdown (lo, hi) or !powerdown (hi, lo)
// used for reset (hi, lo, hi) or for !reset (lo, hi, lo)
// Or, for mclk or pwm (unimplemented yet)
// We have moved the flash to its own, so it is not needed here
#define NVODM_IMAGER_GPIO_PIN_SHIFT    (24)
#define NVODM_IMAGER_UNUSED            (0x0)
#define NVODM_IMAGER_RESET             (0x1 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_RESET_AL          (0x2 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_POWERDOWN         (0x3 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_POWERDOWN_AL      (0x4 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// only on VGP0
#define NVODM_IMAGER_MCLK              (0x8 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// only on VGP6
#define NVODM_IMAGER_PWM               (0x9 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// If flash code wants the gpio's labelled
// use for any purpose, or not at all
#define NVODM_IMAGER_FLASH0           (0x5 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_FLASH1           (0x6 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_FLASH2           (0x7 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// Shutter control
#define NVODM_IMAGER_SHUTTER          (0xA << NVODM_IMAGER_GPIO_PIN_SHIFT)
// 

#define NVODM_IMAGER_MASK              (0xF << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_CLEAR(_s)         ((_s) & ~(NVODM_IMAGER_MASK))
#define NVODM_IMAGER_IS_SET(_s)        (((_s) & (NVODM_IMAGER_MASK)) != 0)
#define NVODM_IMAGER_FIELD(_s)         ((_s) >> NVODM_IMAGER_GPIO_PIN_SHIFT)


#if defined(__cplusplus)
}
#endif

#endif //NVODM_QUERY_DISCOVERY_IMAGER_H
