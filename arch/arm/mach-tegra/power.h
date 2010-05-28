/*
 * arch/arm/mach-tegra/power.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_POWER_H
#define __MACH_TEGRA_POWER_H

#include <asm/page.h>

#define TEGRA_POWER_SDRAM_SELFREFRESH	0x400 /* SDRAM is in self-refresh */

#define TEGRA_POWER_PWRREQ_POLARITY	0x1   /* core power request polarity */
#define TEGRA_POWER_PWRREQ_OE		0x2   /* core power request enable */
#define TEGRA_POWER_SYSCLK_POLARITY	0x4   /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		0x8   /* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		0x10  /* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		0x40  /* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY 0x80  /* CPU power request polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	0x100 /* CPU power request enable */
#define TEGRA_POWER_PMC_SHIFT		8
#define TEGRA_POWER_PMC_MASK		0x1ff

/* layout of IRAM used for LP1 save & restore */
#define TEGRA_IRAM_CODE_AREA		TEGRA_IRAM_BASE + SZ_4K
#define TEGRA_IRAM_CODE_SIZE		SZ_4K

#ifndef __ASSEMBLY__
void tegra_lp2_set_trigger(unsigned long cycles);
void __cortex_a9_save(unsigned int mode);
void tegra_lp2_startup(void);

struct tegra_suspend_platform_data {
	unsigned long cpu_timer;   /* CPU power good time in us,  LP2/LP1 */
	unsigned long cpu_off_timer;	/* CPU power off time us, LP2/LP1 */
	unsigned long core_timer;  /* core power good time in ticks,  LP0 */
	unsigned long core_off_timer;	/* core power off time ticks, LP0 */
	unsigned long wake_enb;    /* mask of enabled wake pads */
	unsigned long wake_high;   /* high-level-triggered wake pads */
	unsigned long wake_low;    /* low-level-triggered wake pads */
	unsigned long wake_any;    /* any-edge-triggered wake pads */
	bool dram_suspend;         /* platform supports DRAM self-refresh */
	bool core_off;             /* platform supports core voltage off */
	bool corereq_high;         /* Core power request active-high */
	bool sysclkreq_high;       /* System clock request is active-high */
	bool separate_req;         /* Core & CPU power request are separate */
};

#endif

#endif
