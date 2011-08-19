/*
 * arch/arm/mach-tegra/tegra3_tsensor.c
 *
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include "board.h"

#ifdef CONFIG_SENSORS_TEGRA_TSENSOR
#include <mach/tsensor.h>
#include "devices.h"
#include "fuse.h"

static struct tegra_tsensor_platform_data tsensor_data = {
	.hysteresis = 5,
	.sw_intr_temperature = 75,
	.hw_clk_div_temperature = 80,
	.hw_reset_temperature = 90,
};

/* fuse revision constants used for tsensor */
#define FUSE_TEST_PROGRAM_REVISION_0 0x128
#define TSENSOR_FUSE_REVISION_DECIMAL 8
#define TSENSOR_FUSE_REVISION_INTEGER 0

void __init tegra_tsensor_init(void)
{
	unsigned int reg, fuse_rev_integer, fuse_rev_decimal;
	/* tsensor driver is instantiated based on fuse revision */
	reg = tegra_fuse_readl(FUSE_TEST_PROGRAM_REVISION_0);
	fuse_rev_decimal = (reg & 0xf);
	fuse_rev_integer = ((reg >> 4) & 0x7);
	pr_info("\nTegra3 fuse revision %d.%d \n", fuse_rev_integer,
		fuse_rev_decimal);
	if ((fuse_rev_decimal >= TSENSOR_FUSE_REVISION_DECIMAL) &&
		(fuse_rev_integer >= TSENSOR_FUSE_REVISION_INTEGER)) {
		/* set platform data for device before register */
		tegra_tsensor_device.dev.platform_data = &tsensor_data;
		platform_device_register(&tegra_tsensor_device);
	}
}
#else
void __init tegra_tsensor_init(void) { }
#endif

