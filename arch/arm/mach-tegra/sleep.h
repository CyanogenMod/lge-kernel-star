/*
 * arch/arm/mach-tegra/sleep.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#define TEGRA_POWER_SDRAM_SELFREFRESH	(1 << 26) /* SDRAM is in self-refresh */
#define TEGRA_POWER_HOTPLUG_SHUTDOWN	(1 << 27) /* Hotplug shutdown */
#define TEGRA_POWER_CLUSTER_G		(1 << 28) /* G CPU */
#define TEGRA_POWER_CLUSTER_LP		(1 << 29) /* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	(TEGRA_POWER_CLUSTER_G | \
						TEGRA_POWER_CLUSTER_LP)
#define TEGRA_POWER_CLUSTER_IMMEDIATE	(1 << 30) /* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	(1 << 31) /* Force switch */

#define TEGRA_IRAM_CODE_AREA		(TEGRA_IRAM_BASE + SZ_4K)

/* PMC_SCRATCH37-39 and 41 are used for tegra_pen_lock in Tegra2 idle */
#define PMC_SCRATCH37                   0x130
#define PMC_SCRATCH38                   0x134
/* PMC_SCRATCH39 stores the reset vector of the AVP (always 0) after LP0 */
#define PMC_SCRATCH39                   0x138
/* PMC_SCRATCH41 stores the reset vector of the CPU after LP0 and LP1 */
#define PMC_SCRATCH41                   0x140

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define CPU_RESETTABLE			2
#define CPU_RESETTABLE_SOON		1
#define CPU_NOT_RESETTABLE		0
#endif

#define FLOW_CTRL_HALT_CPU0_EVENTS	0x0
#define   FLOW_CTRL_WAITEVENT		(2 << 29)
#define   FLOW_CTRL_WAIT_FOR_INTERRUPT	(4 << 29)
#define   FLOW_CTRL_JTAG_RESUME		(1 << 28)
#define   FLOW_CTRL_HALT_CPU_IRQ	(1 << 10)
#define   FLOW_CTRL_HALT_CPU_FIQ	(1 << 8)
#define FLOW_CTRL_CPU0_CSR		0x8
#define   FLOW_CTRL_CSR_INTR_FLAG	(1 << 15)
#define   FLOW_CTRL_CSR_EVENT_FLAG	(1 << 14)
#define   FLOW_CTRL_CSR_ENABLE		(1 << 0)
#define FLOW_CTRL_HALT_CPU1_EVENTS	0x14
#define FLOW_CTRL_CPU1_CSR		0x18

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define FLOW_CTRL_CSR_WFE_CPU0		(1 << 4)
#define FLOW_CTRL_CSR_WFE_BITMAP	(3 << 4)
#define FLOW_CTRL_CSR_WFI_BITMAP	0
#else
#define FLOW_CTRL_CSR_WFE_BITMAP	(0xF << 4)
#define FLOW_CTRL_CSR_WFI_CPU0		(1 << 8)
#define FLOW_CTRL_CSR_WFI_BITMAP	(0xF << 8)
#endif

#define TEGRA_FLOW_CTRL_VIRT (TEGRA_FLOW_CTRL_BASE - IO_PPSB_PHYS + IO_PPSB_VIRT)

#ifndef __ASSEMBLY__

#define FLOW_CTRL_HALT_CPU(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) +	\
	((cpu) ? (FLOW_CTRL_HALT_CPU1_EVENTS + 8 * ((cpu) - 1)) :	\
	 FLOW_CTRL_HALT_CPU0_EVENTS))

#define FLOW_CTRL_CPU_CSR(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) +	\
	((cpu) ? (FLOW_CTRL_CPU1_CSR + 8 * ((cpu) - 1)) :	\
	 FLOW_CTRL_CPU0_CSR))

static inline void flowctrl_writel(unsigned long val, void __iomem *addr)
{
	writel(val, addr);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	wmb();
#endif
	(void)__raw_readl(addr);
}

void tegra_pen_lock(void);
void tegra_pen_unlock(void);
void tegra_cpu_wfi(void);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
extern void tegra2_iram_start;
extern void tegra2_iram_end;
int  tegra2_cpu_is_resettable_soon(void);
void tegra2_cpu_reset(int cpu);
void tegra2_cpu_set_resettable_soon(void);
void tegra2_sleep_core(unsigned long v2p);
void tegra2_hotplug_shutdown(void);
void tegra2_sleep_wfi(unsigned long v2p);
#else
void tegra3_hotplug_shutdown(void);
#endif

static inline void *tegra_iram_start(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return &tegra2_iram_start;
#endif
}

static inline void *tegra_iram_end(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return &tegra2_iram_end;
#endif
}

static inline void tegra_sleep_core(unsigned long v2p)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_sleep_core(v2p);
#endif
}

void tegra_sleep_cpu(unsigned long v2p);
void tegra_resume(void);

#endif

#endif
