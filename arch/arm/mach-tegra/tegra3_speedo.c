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

#define CORE_PROCESS_CORNERS_NUM	1
#define CPU_PROCESS_CORNERS_NUM		4

#define FUSE_SPEEDO_CALIB_0	0x114
#define FUSE_PACKAGE_INFO	0X1FC

/* Maximum speedo levels for each core process corner */
static const u32 core_process_speedos[][CORE_PROCESS_CORNERS_NUM] = {
/* proc_id 0 */
	{180}, /* threshold_index 0: soc_speedo_id 0: any A01 */

/* T30 family */
	{183}, /* threshold_index 1: soc_speedo_id 1: AP30 */
	{197}, /* threshold_index 2: soc_speedo_id 2: T30  */
	{197}, /* threshold_index 3: soc_speedo_id 2: T30S */

/* Characterization SKUs */
	{170}, /* threshold_index 4: soc_speedo_id 1: AP30 char */
	{190}, /* threshold_index 5: soc_speedo_id 2: T30  char */
	{190}, /* threshold_index 6: soc_speedo_id 2: T30S char */

/* T33 family: Numbers cloned from T30 family; FIXME: adjust these later */
	{183}, /* threshold_index 7: soc_speedo_id = 1 - AP33 */
	{197}, /* threshold_index 8: soc_speedo_id = 2 - T33  */
	{197}, /* threshold_index 9: soc_speedo_id = 2 - T33S */
};

/* Maximum speedo levels for each CPU process corner */
static const u32 cpu_process_speedos[][CPU_PROCESS_CORNERS_NUM] = {
/* proc_id 0    1    2    3 */
	{306, 338, 360, 376}, /* threshold_index 0: cpu_speedo_id 0: any A01 */

/* T30 family */
	{305, 336, 358, 375}, /* threshold_index 1: cpu_speedo_id 1: AP30 */
	{336, 336, 358, 375}, /* threshold_index 2: cpu_speedo_id 2: T30  */
	{336, 336, 358, 375}, /* threshold_index 3: cpu_speedo_id 3: T30S */

/* Characterization SKUs */
	{295, 326, 348, 364}, /* threshold_index 4: cpu_speedo_id 1: AP30char */
	{326, 326, 348, 364}, /* threshold_index 5: cpu_speedo_id 2: T30char  */
	{326, 326, 348, 364}, /* threshold_index 6: cpu_speedo_id 3: T30Schar */

/* T33 family: Numbers cloned from T30 family; FIXME: adjust these later */
	{376, 376, 376, 376}, /* threshold_FIXME 7: cpu_speedo_id = 1 - AP33 */
	{376, 376, 376, 376}, /* threshold_index 8: cpu_speedo_id = 2 - T33  */
	{376, 376, 376, 376}, /* threshold_index 9: cpu_speedo_id = 3 - T33S */
};

/*
 * Common speedo_value array threshold index for both core_process_speedos and
 * cpu_process_speedos arrays. Make sure these two arrays are always in synch.
 */
static int threshold_index;

static int cpu_process_id;
static int core_process_id;
static int cpu_speedo_id;
static int soc_speedo_id;
static int package_id;

static void fuse_speedo_calib(u32 *speedo_g, u32 *speedo_lp)
{
	u32 reg;

	BUG_ON(!speedo_g || !speedo_lp);
	reg = tegra_fuse_readl(FUSE_SPEEDO_CALIB_0);

	/* Speedo LP = Lower 16-bits Multiplied by 4 */
	*speedo_lp = (reg & 0xFFFF) * 4;

	/* Speedo G = Upper 16-bits Multiplied by 4 */
	*speedo_g = ((reg >> 16) & 0xFFFF) * 4;
}

static void rev_sku_to_speedo_ids(int rev, int sku)
{
	switch (rev) {
	case TEGRA_REVISION_A01: /* any A01 */
		cpu_speedo_id = 0;
		soc_speedo_id = 0;
		threshold_index = 0;
		break;

	case TEGRA_REVISION_A02:
	case TEGRA_REVISION_A03:
		switch (sku) {
		case 0x87: /* AP30 */
			cpu_speedo_id = 1;
			soc_speedo_id = 1;
			threshold_index = 1;
			break;

		case 0x81: /* T30 */
			switch (package_id) {
			case 1: /* MID => T30 */
				cpu_speedo_id = 2;
				soc_speedo_id = 2;
				threshold_index = 2;
				break;
			case 2: /* DSC => AP33 */
				cpu_speedo_id = 1;
				soc_speedo_id = 1;
				threshold_index = 7;
				break;
			default:
				pr_err("Tegra3 Rev-A02: Reserved pkg: %d\n",
				       package_id);
				BUG();
				break;
			}
			break;

		case 0x80: /* T33 or T33S */
			switch (package_id) {
			case 1: /* MID => T33 */
				cpu_speedo_id = 2;
				soc_speedo_id = 2;
				threshold_index = 8;
				break;
			case 2: /* DSC => T33S */
				cpu_speedo_id = 3;
				soc_speedo_id = 2;
				threshold_index = 9;
				break;
			default:
				pr_err("Tegra3 Rev-A02: Reserved pkg: %d\n",
				       package_id);
				BUG();
				break;
			}
			break;

		case 0x83: /* T30S */
			cpu_speedo_id = 3;
			soc_speedo_id = 2;
			threshold_index = 3;
			break;

/* Characterization SKUs */
		case 0x08: /* AP30 char */
			cpu_speedo_id = 1;
			soc_speedo_id = 1;
			threshold_index = 4;
			break;
		case 0x02: /* T30 char */
			cpu_speedo_id = 2;
			soc_speedo_id = 2;
			threshold_index = 5;
			break;
		case 0x04: /* T30S char */
			cpu_speedo_id = 3;
			soc_speedo_id = 2;
			threshold_index = 6;
			break;

		case 0:    /* ENG - check package_id */
			pr_info("Tegra3 ENG SKU: Checking package_id\n");
			switch (package_id) {
			case 1: /* MID => assume T30 */
				cpu_speedo_id = 2;
				soc_speedo_id = 2;
				threshold_index = 2;
				break;
			case 2: /* DSC => assume T30S */
				cpu_speedo_id = 3;
				soc_speedo_id = 2;
				threshold_index = 3;
				break;
			default:
				pr_err("Tegra3 Rev-A02: Reserved pkg: %d\n",
				       package_id);
				BUG();
				break;
			}
			break;

		default:
			/* FIXME: replace with BUG() when all SKU's valid */
			pr_err("Tegra3 Rev-A02: Unknown SKU %d\n", sku);
			cpu_speedo_id = 0;
			soc_speedo_id = 0;
			threshold_index = 0;
			break;
		}
		break;
	default:
		BUG();
		break;
	}
}

void tegra_init_speedo_data(void)
{
	u32 cpu_speedo_val, core_speedo_val;
	int iv;

	/* Package info: 4 bits - 0,3:reserved 1:MID 2:DSC */
	package_id = tegra_fuse_readl(FUSE_PACKAGE_INFO) & 0x0F;

	/* Arrays must be of equal size - each index corresponds to a SKU */
	BUG_ON(ARRAY_SIZE(cpu_process_speedos) !=
	       ARRAY_SIZE(core_process_speedos));

	rev_sku_to_speedo_ids(tegra_get_revision(), tegra_sku_id());
	BUG_ON(threshold_index >= ARRAY_SIZE(cpu_process_speedos));

	fuse_speedo_calib(&cpu_speedo_val, &core_speedo_val);
	pr_debug("%s CPU speedo value %u\n", __func__, cpu_speedo_val);
	pr_debug("%s Core speedo value %u\n", __func__, core_speedo_val);

	for (iv = 0; iv < CPU_PROCESS_CORNERS_NUM; iv++) {
		if (cpu_speedo_val <
		    cpu_process_speedos[threshold_index][iv]) {
			break;
		}
	}
	cpu_process_id = iv -1;

	if (cpu_process_id == -1) {
		pr_err("****************************************************");
		pr_err("****************************************************");
		pr_err("* tegra3_speedo: CPU speedo value %3d out of range *",
		       cpu_speedo_val);
		pr_err("****************************************************");
		pr_err("****************************************************");

		cpu_process_id = INVALID_PROCESS_ID;
		cpu_speedo_id = 0;
	}

	for (iv = 0; iv < CORE_PROCESS_CORNERS_NUM; iv++) {
		if (core_speedo_val <
		    core_process_speedos[threshold_index][iv]) {
			break;
		}
	}
	core_process_id = iv -1;

	if (core_process_id == -1) {
		pr_err("****************************************************");
		pr_err("****************************************************");
		pr_err("* tegra3_speedo: CORE speedo value %3d out of range *",
		       core_speedo_val);
		pr_err("****************************************************");
		pr_err("****************************************************");

		core_process_id = INVALID_PROCESS_ID;
		soc_speedo_id = 0;
	}

	pr_info("Tegra3: CPU Speedo ID %d, Soc Speedo ID %d",
		 cpu_speedo_id, soc_speedo_id);
}

int tegra_cpu_process_id(void)
{
	/* FIXME: remove when ready to deprecate invalid process-id boards */
	if (cpu_process_id == INVALID_PROCESS_ID)
		return 0;
	else
		return cpu_process_id;
}

int tegra_core_process_id(void)
{
	/* FIXME: remove when ready to deprecate invalid process-id boards */
	if (core_process_id == INVALID_PROCESS_ID)
		return 0;
	else
		return core_process_id;
}

int tegra_cpu_speedo_id(void)
{
	return cpu_speedo_id;
}

int tegra_soc_speedo_id(void)
{
	return soc_speedo_id;
}

int tegra_package_id(void)
{
	return package_id;
}
