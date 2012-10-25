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

#ifndef INCLUDED_MAX8907_RTC_HEADER
#define INCLUDED_MAX8907_RTC_HEADER

/* Read RTC count register */
int max8907c_rtc_count_read(unsigned int *count);

/* Read RTC alarm count register */
int max8907c_rtc_alarm_count_read(unsigned int *count);

/* Write RTC count register */
int max8907c_rtc_count_write(unsigned int count);

/* Write RTC alarm count register */
int max8907c_rtc_alarm_write(unsigned int count);

#endif // INCLUDED_MAX8907_RTC_HEADER
