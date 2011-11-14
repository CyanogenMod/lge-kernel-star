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
#define CPU_PROCESS_CORNERS_NUM	7

#define FUSE_SPEEDO_CALIB_0	0x114
#define FUSE_PACKAGE_INFO	0X1FC

/* Maximum speedo levels for each core process corner */
static const u32 core_process_speedos[][CORE_PROCESS_CORNERS_NUM] = {
/* proc_id 0 */
	{180}, /* [0]: soc_speedo_id 0: any A01 */

/* T30 family */
	{180}, /* [1]: soc_speedo_id 1: AP30 */
	{204}, /* [2]: soc_speedo_id 2: T30  */
	{192}, /* [3]: soc_speedo_id 2: T30S */

/* Characterization SKUs */
	{168}, /* [4]: soc_speedo_id 1: AP30 char */
	{192}, /* [5]: soc_speedo_id 2: T30  char */
	{184}, /* [6]: soc_speedo_id 2: T30S char */

/* T33 family */
	{180}, /* [7]: soc_speedo_id = 1 - AP33 */
	{208}, /* [8]: soc_speedo_id = 2 - T33  */
	{192}, /* [9]: soc_speedo_id = 2 - T33S */

/* T30 'L' family */
	{192}, /* [10]: soc_speedo_id 1: T30L */
	{192}, /* [11]: soc_speedo_id 1: T30SL */

/* T30 Automotives */
	{185}, /* [12]: soc_speedo_id = 3 - Automotives */
	{185}, /* [13]: soc_speedo_id = 3 - Automotives */
};

/* Maximum speedo levels for each CPU process corner */
static const u32 cpu_process_speedos[][CPU_PROCESS_CORNERS_NUM] = {
/* proc_id 0    1    2    3    4*/
	{306, 338, 360, 376, UINT_MAX}, /* [0]: cpu_speedo_id 0: any A01 */

/* T30 family */
	{304, 336, 359, 375, UINT_MAX}, /* [1]: cpu_speedo_id 1: AP30 */
	{336, 336, 359, 375, UINT_MAX}, /* [2]: cpu_speedo_id 2: T30  */
	{336, 336, 359, 375, UINT_MAX}, /* [3]: cpu_speedo_id 3: T30S */

/* Characterization SKUs */
	{292, 324, 348, 364, UINT_MAX}, /* [4]: cpu_speedo_id 1: AP30char */
	{324, 324, 348, 364, UINT_MAX}, /* [5]: cpu_speedo_id 2: T30char  */
	{324, 324, 348, 364, UINT_MAX}, /* [6]: cpu_speedo_id 3: T30Schar */

/* T33 family */
	{305, 337, 359, 376, UINT_MAX}, /* [7]: cpu_speedo_id = 4 - AP33 */
	{368, 368, 368, 368, 392},	/* [8]: cpu_speedo_id = 5 - T33  */
	{376, 376, 376, 376, 392},	/* [9]: cpu_speedo_id = 6 - T33S */

/* T30 'L' family */
	{305, 337, 359, 376, 392},	/* [10]: cpu_speedo_id 7: T30L  */
	{305, 337, 359, 376, 392},	/* [11]: cpu_speedo_id 8: T30SL */

/* T30 Automotives */
	/* threshold_index 12: cpu_speedo_id 9 & 10
	 * 0,1,2 values correspond to speedo_id  9
	 * 3,4,5 values correspond to speedo_id 10
	 */
	{300, 311, 360, 371, 381, 415, 431},
	{300, 311, 410, 431}, /* threshold_index 13: cpu_speedo_id = 11 */
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
		case 0x82: /* T30V */
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
				cpu_speedo_id = 4;
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
				cpu_speedo_id = 5;
				soc_speedo_id = 2;
				threshold_index = 8;
				break;
			case 2: /* DSC => T33S */
				cpu_speedo_id = 6;
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

		case 0x83: /* T30L or T30S */
			switch (package_id) {
			case 1: /* MID => T30L */
				cpu_speedo_id = 7;
				soc_speedo_id = 1;
				threshold_index = 10;
				break;
			case 2: /* DSC => T30S */
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

		case 0x8F: /* T30SL */
			cpu_speedo_id = 8;
			soc_speedo_id = 1;
			threshold_index = 11;
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

		case 0x91: /* T30AGS-Ax */
		case 0xb0: /* T30IQS-Ax */
		case 0xb1: /* T30MQS-Ax */
		case 0x90: /* T30AQS-Ax */
			soc_speedo_id = 3;
			threshold_index = 12;
			break;
		case 0x93: /* T30AG-Ax */
			cpu_speedo_id = 11;
			soc_speedo_id = 3;
			threshold_index = 13;
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
		cpu_speedo_id = 1;
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
		soc_speedo_id = 1;
	}
	if (threshold_index == 12 && cpu_process_id != INVALID_PROCESS_ID) {
		if (cpu_process_id <= 2)
			cpu_speedo_id = 9;
		else if (cpu_process_id >= 3 && cpu_process_id < 6)
			cpu_speedo_id = 10;
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

/*
 * CPU and core nominal voltage levels as determined by chip SKU and speedo
 * (not final - can be lowered by dvfs tables and rail dependencies; the
 * latter is resolved by the dvfs code)
 */
static const int cpu_speedo_nominal_millivolts[] =
/* speedo_id 0,    1,    2,    3,    4,    5,    6,    7,    8,   9,  10,  11 */
	{ 1125, 1150, 1150, 1150, 1237, 1237, 1237, 1150, 1150, 912, 850, 850};

int tegra_cpu_speedo_mv(void)
{
	BUG_ON(cpu_speedo_id >= ARRAY_SIZE(cpu_speedo_nominal_millivolts));
	return cpu_speedo_nominal_millivolts[cpu_speedo_id];
}

int tegra_core_speedo_mv(void)
{
	switch (soc_speedo_id) {
	case 0:
		return 1200;
	case 1:
		if ((cpu_speedo_id != 7) && (cpu_speedo_id != 8))
			return 1200;
		/* fall thru for T30L or T30SL */
	case 2:
		return 1300;
	case 3:
		return 1250;
	default:
		BUG();
	}
}
