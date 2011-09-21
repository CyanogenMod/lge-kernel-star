/*
 *  arch/arm/mach-tegra/hotplug.c
 *
 *  Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/smp.h>

#include <asm/cpu_pm.h>
#include <asm/cacheflush.h>

#include <mach/iomap.h>

#include "gic.h"
#include "sleep.h"

#define CPU_CLOCK(cpu) (0x1<<(8+cpu))

#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
/* For Tegra2 use the software-written value of the reset register for status.*/
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET
#else
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)
#endif

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;

	do {
		reg = readl(CLK_RST_CONTROLLER_CPU_CMPLX_STATUS);
		cpu_relax();
	} while (!(reg & (1<<cpu)));

	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	return 1;
}

void platform_cpu_die(unsigned int cpu)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* Flush the L1 data cache. */
	flush_cache_all();

	/* Place the current CPU in reset. */
	tegra2_hotplug_shutdown();
#else
	/* Disable GIC CPU interface for this CPU. */
	tegra_gic_cpu_disable();

	/* Tegra3 enters LPx states via WFI - do not propagate legacy IRQs
	   to CPU core to avoid fall through WFI; then GIC output will be
	   enabled, however at this time - CPU is dying - no interrupt should
	   have affinity to this CPU. */
	tegra_gic_pass_through_disable();

	/* Flush the L1 data cache. */
	flush_cache_all();

	/* Shut down the current CPU. */
	tegra3_hotplug_shutdown();
#endif

	/* Should never return here. */
	BUG();
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
