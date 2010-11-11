/*
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

#ifndef INCLUDED_NVODM_GPIO_EXT_ADAPTATION_HAL_H
#define INCLUDED_NVODM_GPIO_EXT_ADAPTATION_HAL_H

#include "nvodm_gpio_ext.h"

#ifdef __cplusplus
extern "C"
{
#endif

//  A simple HAL for the External GPIO adaptations.
typedef void (*pfnExternalGpioWritePins)(NvU32, NvU32, NvU32);
typedef NvU32 (*pfnExternalGpioReadPins)(NvU32, NvU32);

typedef struct NvOdmGpioExtDeviceRec
{
    pfnExternalGpioWritePins    pfnWritePins;
    pfnExternalGpioReadPins     pfnReadPins;

} NvOdmGpioExtDevice;


#ifdef __cplusplus
}
#endif

#endif
