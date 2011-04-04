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

#ifndef INCLUDED_GPIO_PCF50626_H
#define INCLUDED_GPIO_PCF50626_H

#include "gpio_ext_hal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

void
GPIO_PCF50626_NvOdmExternalGpioWritePins(
    NvU32 Port,
    NvU32 Pin,
    NvU32 PinValue);

NvU32
GPIO_PCF50626_NvOdmExternalGpioReadPins(
    NvU32 Port,
    NvU32 Pin);

#if defined(__cplusplus)
}
#endif

#endif
