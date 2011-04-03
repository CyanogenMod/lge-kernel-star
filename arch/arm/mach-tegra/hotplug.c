/*
 *  linux/arch/arm/mach-tegra/hotplug.c
 *
 *  Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/smp.h>

#include <asm/cpu_pm.h>

#include <mach/iomap.h>

#include "sleep.h"

#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;

	do {
		reg = readl(CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
		cpu_relax();
	} while (!(reg & (1<<cpu)));

	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | (1<<(8+cpu)), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	return 1;
}

void platform_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (cpu != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, cpu);
		BUG();
	}
#endif

	tegra_sleep_reset();

	/*
	 * tegra_cpu_suspend can return through tegra_cpu_resume, but that
	 * should never happen for a hotplugged cpu
	 */
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
