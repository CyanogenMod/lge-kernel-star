/*
 * arch/arm/mach-tegra/include/mach/hardware.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_HARDWARE_H
#define __MACH_TEGRA_HARDWARE_H

#define pcibios_assign_all_busses()	0
#define PCIBIOS_MIN_IO			0x83000000ul
#define PCIBIOS_MIN_MEM			0x90000000ul

#endif
