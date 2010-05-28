/*
 * arch/arm/mach-tegra/include/mach/i2c.h
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_I2C_H
#define __MACH_TEGRA_I2C_H

#include <mach/pinmux.h>

#define TEGRA_I2C_MAX_BUS 2

struct tegra_i2c_plat_parms {
	int adapter_nr;
	int bus_count;
	unsigned int bus_mux[TEGRA_I2C_MAX_BUS];
	unsigned long bus_clk[TEGRA_I2C_MAX_BUS];
	int bus_mux_len[TEGRA_I2C_MAX_BUS];
	bool is_dvc;
};

#endif
