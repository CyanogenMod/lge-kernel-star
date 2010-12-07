/*
 * arch/arm/mach-tegra/headsmp-t2.S
 *
 * SMP initialization routines for Tegra3 SoCs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/domain.h>
#include <asm/ptrace.h>
#include <asm/cache.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "power-macros.S"


#define DEBUG_HOTPLUG_STARTUP	0	/* Nonzero for hotplug startup debug */
#define DEBUG_LP2_STARTUP	0	/* Nonzero for LP2 startup debug */


#ifdef CONFIG_HOTPLUG_CPU
/*
 *	tegra_hotplug_startup
 *
 *	  Secondary CPU boot vector when restarting a CPU following a
 *	  hot-unplug. Uses the page table created by smp_prepare_cpus and
 *	  stored in tegra_pgd_phys as the safe page table for
 *	  __return_to_virtual, and jumps directly to __cortex_a9_restore.
 */
	.align L1_CACHE_SHIFT
ENTRY(tegra_hotplug_startup)
#if	DEBUG_HOTPLUG_STARTUP
	b	.
#endif
	setmode	PSR_F_BIT | PSR_I_BIT | SVC_MODE, r9
	bl	__invalidate_cpu_state
	enable_coresite r1
	cpu_id	r0
	subs	r1, r0, #1
#ifdef	DEBUG
	/* !!!CHECKME!!! THIS MAY NOW BE OBSOLETE */
	bmi	.			@ should never come here for CPU0
#endif
	mov	r3, r1, lsl #3
	add	r3, r3, #0x18	 	@ CPUn CSR offset, n>0
	mov32	r2, TEGRA_FLOW_CTRL_BASE

	@ Clear the flow controller flags for this CPU.
	ldr	r1, [r2, r3]
	orr	r1, r1, #(1 << 15) | (1 << 14)	@ write to clear event & intr
	movw	r0, #0x0FFD	@ enable, cluster_switch, immed, & bitmaps
	bic	r1, r1, r0
	str	r1, [r2, r3]

	/* most of the below is a retread of what happens in __v7_setup and
	 * secondary_startup, to get the MMU re-enabled and to branch
	 * to secondary_kernel_startup */
	mrc	p15, 0, r0, c1, c0, 1
	orr	r0, r0, #(1 << 6) | (1 << 0)	@ re-enable coherency
	mcr	p15, 0, r0, c1, c0, 1

	adr	r4, __tegra_hotplug_data
	ldmia	r4, {r5, r7, r12}
	mov	r1, r12			@ ctx_restore = __cortex_a9_restore
	sub	r4, r4, r5
	ldr	r0, [r7, r4]		@ pgdir = secondary_data.pgdir
	b	__return_to_virtual
ENDPROC(tegra_hotplug_startup)


	.type	__tegra_hotplug_data, %object
__tegra_hotplug_data:
	.long	.
	.long	tegra_pgd_phys
	.long	__cortex_a9_restore
	.size	__tegra_hotplug_data, . - __tegra_hotplug_data
#endif
