/*
 * arch/arm/mach-tegra/include/mach/usb-otg.h
 *
 * Definitions for platform devices and related flags NVIDIA Tegra SoCs
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

#ifndef __MACH_TEGRA_REGULATOR_H
#define __MACH_TEGRA_REGULATOR_H

struct tegra_regulator_entry {
	unsigned int charging_path;
	unsigned long long guid;
	const char *name;
	struct regulator_consumer_supply *consumers;
	int nr_consumers;
	int id;
	bool is_charger;
};

struct tegra_regulator_platform_data {
	struct tegra_regulator_entry *regs;
	int nr_regs;
};

#endif
