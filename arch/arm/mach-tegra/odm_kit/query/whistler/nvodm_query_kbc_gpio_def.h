/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/nvodm_query_kbc_gpio_def.h
 *
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#ifndef NVODM_QUERY_KBC_GPIO_DEF_H
#define NVODM_QUERY_KBC_GPIO_DEF_H

typedef enum
{
    NvOdmKbcGpioPin_KBRow0 = 0,
    NvOdmKbcGpioPin_KBRow1,
    NvOdmKbcGpioPin_KBRow2,
    NvOdmKbcGpioPin_KBRow3,
    NvOdmKbcGpioPin_KBRow4,
    NvOdmKbcGpioPin_KBRow5,
    NvOdmKbcGpioPin_KBRow6,
    NvOdmKbcGpioPin_KBRow7,
    NvOdmKbcGpioPin_KBRow8,
    NvOdmKbcGpioPin_KBRow9,
    NvOdmKbcGpioPin_KBRow10,
    NvOdmKbcGpioPin_KBRow11,
    NvOdmKbcGpioPin_KBRow12,
    NvOdmKbcGpioPin_KBRow13,
    NvOdmKbcGpioPin_KBRow14,
    NvOdmKbcGpioPin_KBRow15,
    NvOdmKbcGpioPin_KBCol0,
    NvOdmKbcGpioPin_KBCol1,
    NvOdmKbcGpioPin_KBCol2,
    NvOdmKbcGpioPin_KBCol3,
    NvOdmKbcGpioPin_KBCol4,
    NvOdmKbcGpioPin_KBCol5,
    NvOdmKbcGpioPin_KBCol6,
    NvOdmKbcGpioPin_KBCol7,
    NvOdmKbcGpioPin_Num,
    NvOdmKbcGpioPin_Force32 = 0x7FFFFFFF
}NvOdmKbcGpioPin;

#endif // NVODM_QUERY_KBC_GPIO_DEF_H
