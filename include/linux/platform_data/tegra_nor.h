/*
 * include/linux/platform_data/tegra_nor.h
 *
 * Copyright (C) 2010 - 2011 NVIDIA Corporation.
 *
 * Author:
 *	Raghavendra V K <rvk@nvidia.com>
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

#ifndef __MACH_TEGRA_NOR_PDATA_H
#define __MACH_TEGRA_NOR_PDATA_H

#include <asm/mach/flash.h>

struct tegra_nor_chip_parms {
	struct {
		uint32_t timing0;
		uint32_t timing1;
	} timing_default, timing_read;
};

struct tegra_nor_platform_data {
	struct tegra_nor_chip_parms chip_parms;
	struct flash_platform_data flash;
};

#endif /* __MACH_TEGRA_NOR_PDATA_H */
