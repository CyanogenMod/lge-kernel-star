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

#define PROCESS_CORNERS_NUM	4

#define FUSE_SPEEDO_CALIB_0	0x114

/* Maximum speedo levels for each core process corner */
static const u32 core_process_speedos[][PROCESS_CORNERS_NUM] = {
// proc_id 0    1
	{180, 240}, // soc_speedo_id 0
	{180, 240}, // soc_speedo_id 1
	{200, 240}, // soc_speedo_id 2
};

/* Maximum speedo levels for each CPU process corner */
static const u32 cpu_process_speedos[][PROCESS_CORNERS_NUM] = {
// proc_id 0    1    2    3
	{306, 338, 360, 376}, // soc_speedo_id 0
	{306, 338, 360, 376}, // soc_speedo_id 1
	{338, 338, 360, 376}, // soc_speedo_id 2
};

static int cpu_process_id;
static int core_process_id;
static int soc_speedo_id;

static void fuse_speedo_calib(u32 *speedo_g, u32 *speedo_lp)
{
	u32 reg;

	BUG_ON(!speedo_g || !speedo_lp);
	reg = tegra_fuse_readl(FUSE_SPEEDO_CALIB_0);

	// Speedo LP = Lower 16-bits Multiplied by 4
	*speedo_lp = (reg & 0xFFFF) * 4;

	// Speedo G = Upper 16-bits Multiplied by 4
	*speedo_g = ((reg >> 16) & 0xFFFF) * 4;
}

static int rev_sku_to_soc_speedo(int rev, int sku)
{
	int soc_speedo;

	switch (rev) {
	case TEGRA_REVISION_A01:
		soc_speedo = 0;
		break;
	case TEGRA_REVISION_A02:
		switch (sku) {
		case 0: // AP30
			soc_speedo = 1;
			break;
		case 1: // T30
			soc_speedo = 2;
			break;
		default:
			BUG();
			break;
		}
		break;
	default:
		BUG();
		break;
	}

	pr_debug("Tegra3 SKU: %d Rev: %s Speedo: %d ",
		 sku, tegra_get_revision_name(), soc_speedo);
	return soc_speedo;
}

void tegra_init_speedo_data(void)
{
	u32 cpu_speedo_val, core_speedo_val;
	int iv;

	soc_speedo_id = rev_sku_to_soc_speedo(tegra_get_revision(),
					      tegra_sku_id());
	BUG_ON(soc_speedo_id >= ARRAY_SIZE(cpu_process_speedos));

	fuse_speedo_calib(&cpu_speedo_val, &core_speedo_val);
	pr_debug("%s CPU speedo value %u\n", __func__, cpu_speedo_val);
	pr_debug("%s Core speedo value %u\n", __func__, core_speedo_val);

	cpu_process_id = -1; // out of range for valid cpu-speedo
	for (iv = 0; iv < PROCESS_CORNERS_NUM; iv++) {
		if (cpu_speedo_val < cpu_process_speedos[soc_speedo_id][iv]) {
			cpu_process_id = iv -1;
			break;
		}
	}
	if (cpu_process_id == -1) {
		pr_err("****************************************************");
		pr_err("****************************************************");
		pr_err("* tegra3_speedo: CPU speedo value %3d out of range *",
		       cpu_speedo_val);
		pr_err("****************************************************");
		pr_err("****************************************************");

		cpu_process_id = INVALID_PROCESS_ID;
	}

	core_process_id = -1; // out of range for valid core-speedo
	for (iv = 0; iv < PROCESS_CORNERS_NUM; iv++) {
		if (core_speedo_val < core_process_speedos[soc_speedo_id][iv]) {
			core_process_id = iv -1;
			break;
		}
	}
	if (core_process_id == -1) {
		pr_err("*****************************************************");
		pr_err("*****************************************************");
		pr_err("* tegra3_speedo: CORE speedo value %3d out of range *",
		       core_speedo_val);
		pr_err("*****************************************************");
		pr_err("*****************************************************");

		core_process_id = INVALID_PROCESS_ID;
	}

	pr_info("Tegra3 SKU: %d Rev: %s CPU Process: %d CORE Process: %d "
		"Speedo ID: %d",
		tegra_sku_id(), tegra_get_revision_name(),
		cpu_process_id, core_process_id, soc_speedo_id);
}

int tegra_cpu_process_id(void)
{
	// FIXME: remove this when ready to deprecate invalid process-id boards
	if (cpu_process_id == INVALID_PROCESS_ID)
		return 0;
	else
		return cpu_process_id;
}

int tegra_core_process_id(void)
{
	// FIXME: remove this when ready to deprecate invalid process-id boards
	if (core_process_id == INVALID_PROCESS_ID)
		return 0;
	else
		return core_process_id;
}

int tegra_soc_speedo_id(void)
{
	return soc_speedo_id;
}
