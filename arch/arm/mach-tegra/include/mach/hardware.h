/*
 * arch/arm/mach-tegra/include/mach/hardware.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corp.
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

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define PCIBIOS_MIN_IO				0x1000
#define PCIBIOS_MIN_MEM				0
#define pcibios_assign_all_busses()		1

#else

#define PCIBIOS_MIN_IO				0x03000000ul
#define PCIBIOS_MIN_MEM				0x10000000ul
#define pcibios_assign_all_busses()		0
#endif

enum tegra_chipid {
	TEGRA_CHIPID_UNKNOWN = 0,
	TEGRA_CHIPID_TEGRA2 = 0x20,
	TEGRA_CHIPID_TEGRA3 = 0x30,
};

enum tegra_revision {
	TEGRA_REVISION_UNKNOWN = 0,
	TEGRA_REVISION_A01,
	TEGRA_REVISION_A02,
	TEGRA_REVISION_A03,
	TEGRA_REVISION_A03p,
	TEGRA_REVISION_MAX,
};

enum tegra_chipid tegra_get_chipid(void);
enum tegra_revision tegra_get_revision(void);
#endif
