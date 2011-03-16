/*
 * drivers/regulator/tegra-regulator.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef _MACH_TEGRA_POWERGATE_H_
#define _MACH_TEGRA_POWERGATE_H_

#define TEGRA_POWERGATE_CPU	0
#define TEGRA_POWERGATE_CPU0	TEGRA_POWERGATE_CPU
#define TEGRA_POWERGATE_3D	1
#define TEGRA_POWERGATE_3D0	TEGRA_POWERGATE_3D
#define TEGRA_POWERGATE_VENC	2
#define TEGRA_POWERGATE_PCIE	3
#define TEGRA_POWERGATE_VDEC	4
#define TEGRA_POWERGATE_L2	5
#define TEGRA_POWERGATE_MPE	6
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define TEGRA_NUM_POWERGATE	7
#define TEGRA_CPU_POWERGATE_ID(cpu)	(TEGRA_POWERGATE_CPU)
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
#define TEGRA_POWERGATE_HEG	7
#define TEGRA_POWERGATE_SATA	8
#define TEGRA_POWERGATE_CPU1	9
#define TEGRA_POWERGATE_CPU2	10
#define TEGRA_POWERGATE_CPU3	11
#define TEGRA_POWERGATE_A9LP	12
#define TEGRA_POWERGATE_3D1	13
#define TEGRA_NUM_POWERGATE	14
#define TEGRA_CPU_POWERGATE_ID(cpu)	((cpu == 0) ? TEGRA_POWERGATE_CPU0 : \
						(cpu + TEGRA_POWERGATE_CPU1 - 1))
#endif

struct clk;

int tegra_powergate_power_on(int id);
int tegra_powergate_power_off(int id);
bool tegra_powergate_is_powered(int id);
int tegra_powergate_remove_clamping(int id);
const char* tegra_powergate_get_name(int id);

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(int id, struct clk *clk);

#endif /* _MACH_TEGRA_POWERGATE_H_ */
