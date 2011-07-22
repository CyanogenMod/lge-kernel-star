/*
 * arch/arm/mach-tegra/sleep.h
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

#ifndef __MACH_TEGRA_SLEEP_H
#define __MACH_TEGRA_SLEEP_H

#include <mach/iomap.h>

#define TEGRA_IRAM_CODE_AREA		(TEGRA_IRAM_BASE + SZ_4K)

/* PMC_SCRATCH37-39 and 41 are used for tegra_pen_lock in idle */
#define PMC_SCRATCH37                   0x130
#define PMC_SCRATCH38                   0x134
/* PMC_SCRATCH39 stores the reset vector of the AVP (always 0) after LP0 */
#define PMC_SCRATCH39                   0x138
/* PMC_SCRATCH41 stores the reset vector of the CPU after LP0 and LP1 */
#define PMC_SCRATCH41                   0x140

#define CPU_RESETTABLE			2
#define CPU_RESETTABLE_SOON		1
#define CPU_NOT_RESETTABLE		0

#ifndef __ASSEMBLY__
/* assembly routines implemented in sleep.S */
void tegra_pen_lock(void);
void tegra_pen_unlock(void);
void tegra_cpu_wfi(void);
void tegra_cpu_reset(int cpu);
void tegra_cpu_set_resettable_soon(void);
int tegra_cpu_is_resettable_soon(void);

extern void tegra_lp1_reset;
extern void tegra_iram_start;
extern void tegra_iram_end;

void tegra_sleep_reset(void);
void tegra_sleep_wfi(unsigned long v2p);
void tegra_sleep_cpu(unsigned long v2p);
void tegra_sleep_core(unsigned long v2p);
void tegra_resume(void);
void tegra_secondary_resume(void);

#endif

#endif
