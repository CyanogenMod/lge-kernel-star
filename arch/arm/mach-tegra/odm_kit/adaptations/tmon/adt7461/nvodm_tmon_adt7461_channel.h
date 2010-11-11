/*
 * arch/arm/mach-tegra/odm_kit/adaptations/tmon/adt7461/nvodm_tmon_adt7461_channel.h
 *
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
 
#ifndef INCLUDED_NVODM_TMON_ADT7461_CHANNEL_H
#define INCLUDED_NVODM_TMON_ADT7461_CHANNEL_H

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
    // Local sensor
    ADT7461ChannelID_Local = 1,

    // Remote sensor
    ADT7461ChannelID_Remote,

    ADT7461ChannelID_Num,
    ADT7461ChannelID_Force32 = 0x7FFFFFFFUL
} ADT7461ChannelID;

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_NVODM_TMON_ADT7461_CHANNEL_H

