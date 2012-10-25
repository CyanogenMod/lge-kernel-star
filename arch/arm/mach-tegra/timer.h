/*
 * arch/arm/mach-tegra/timer.h
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#ifndef _MACH_TEGRA_TIMER_H_
#define _MACH_TEGRA_TIMER_H_

#define RTC_SECONDS		0x08
#define RTC_SHADOW_SECONDS	0x0c
#define RTC_MILLISECONDS	0x10

#define TIMER_PTV		0x0
#define TIMER_PCR		0x4

#define TIMERUS_CNTR_1US	0x10
#define TIMERUS_USEC_CFG	0x14
#define TIMERUS_CNTR_FREEZE	0x4c


#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void __init tegra2_init_timer(u32 *offset, int *irq);
#else
void __init tegra3_init_timer(u32 *offset, int *irq);
#endif

struct tegra_twd_context {
	u32 twd_ctrl;
	u32 twd_load;
	u32 twd_cnt;
};

#ifdef CONFIG_HAVE_ARM_TWD
int tegra_twd_get_state(struct tegra_twd_context *context);
void tegra_twd_suspend(struct tegra_twd_context *context);
void tegra_twd_resume(struct tegra_twd_context *context);
#else
static inline int tegra_twd_get_state(struct tegra_twd_context *context)
{ return -ENODEV; }
static inline void tegra_twd_suspend(struct tegra_twd_context *context) {}
static inline void tegra_twd_resume(struct tegra_twd_context *context) {}
#endif

#endif /* _MACH_TEGRA_TIMER_H_ */
