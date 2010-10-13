/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/memblock.h>

#include <asm/hardware/cache-l2x0.h>

#include <mach/gpio.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include "board.h"
#include "clock.h"
#include "fuse.h"
#include "pm.h"

unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;

void (*arch_reset)(char mode, const char *cmd) = tegra_assert_system_reset;

void tegra_assert_system_reset(char mode, const char *cmd)
{
	void __iomem *reset = IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x04);
	u32 reg;

	/* use *_related to avoid spinlock since caches are off */
	reg = readl_relaxed(reset);
	reg |= 0x04;
	writel_relaxed(reg, reset);
}

static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
	{ "pll_p",	"clk_m",	216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_p_out4",	"pll_p",	108000000,	true },
	{ "pll_m_out1",	"pll_m",	240000000,	true },
	{ "sclk",	"pll_m_out1",	240000000,	true },
	{ "hclk",	"sclk",		240000000,	true },
	{ "pclk",	"hclk",		120000000,	true },
	{ "csite",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ "pll_u",	"clk_m",	480000000,	false },
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "sdmmc2",	"pll_p",	48000000,	false},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	false},
	{ NULL,		NULL,		0,		0},
};

void tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;

	writel_relaxed(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(0x441, p + L2X0_DATA_LATENCY_CTRL);

	l2x0_init(p, 0x6C480001, 0x8200c3fe);
#endif

}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
}

void __init tegra_init_early(void)
{
	tegra_init_fuse();
	tegra_gpio_resume_init();
	tegra_init_clock();
	tegra_init_pinmux();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
}

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

void __init tegra_reserve(void)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_start + tegra_lp0_vec_size - 1);
}
