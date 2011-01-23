/*
 * arch/arm/mach-tegra/tegra3_speedo.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>

#include <mach/iomap.h>

#include "fuse.h"

static int cpu_process_id;
static int core_process_id;
static int soc_speedo_id;

void tegra_init_speedo_data(void)
{
	cpu_process_id = 0;
	core_process_id = 0;
	soc_speedo_id = 0;
}

int tegra_cpu_process_id(void)
{
	return cpu_process_id;
}

int tegra_core_process_id(void)
{
	return core_process_id;
}

int tegra_soc_speedo_id(void)
{
	return soc_speedo_id;
}
