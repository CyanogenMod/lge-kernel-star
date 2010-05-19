/*
 * arch/arm/mach-tegra/include/mach/serial.h
 *
 * Definitions for serial platform data
 *
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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
#ifndef __MACH_TEGRA_SERIAL_H
#define __MACH_TEGRA_SERIAL_H

#include <linux/types.h>
#include <linux/serial_8250.h>

#include <mach/pinmux.h>

struct tegra_serial_platform_data {
	struct plat_serial8250_port p;
	const struct tegra_pingroup_config *pinmux;
	int nr_pins;
};

#endif
