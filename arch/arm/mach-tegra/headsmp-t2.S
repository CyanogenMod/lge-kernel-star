/*
 * arch/arm/mach-tegra/headsmp-t2.S
 *
 * SMP initialization routines for Tegra2 SoCs
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
#include <asm/ptrace.h>
#include <asm/cache.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "power-macros.S"

#define PMC_DPD_SAMPLE	0x20
#define PMC_DPD_ENABLE	0x24
#define PMC_SCRATCH39	0x138

#define CLK_RESET_PLLC_BASE	0x80
#define CLK_RESET_PLLM_BASE	0x90

/*        .section ".cpuinit.text", "ax"*/

/*
 *	tegra_lp2_startup
 *
 *	  Secondary CPU boot vector when restarting the master CPU following
 *	  an LP2 idle transition. Re-enable coresight access, re-enable
 *	  MMU, re-start PLLX, and PLLM, PLLC (if not running), restore
 *	  processor context.
 */
	.align L1_CACHE_SHIFT
ENTRY(tegra_lp2_startup)
	setmode PSR_F_BIT | PSR_I_BIT | SVC_MODE, r9

	mov32	r0, TEGRA_TMRUS_BASE
	ldr	r1, [r0]
	mov32	r0, TEGRA_PMC_BASE
	str	r1, [r0, #PMC_SCRATCH39]	@ save off exact lp2 exit time
	mov	r1, #0
	str	r1, [r0, #PMC_DPD_SAMPLE]
	str	r1, [r0, #PMC_DPD_ENABLE]

	mov32	r0, TEGRA_CLK_RESET_BASE
	ldr	r1, [r0, #CLK_RESET_PLLM_BASE]
	tst	r1, #(1<<30)
	orreq	r1, r1, #(1<<30)
	streq	r1, [r0, #CLK_RESET_PLLM_BASE]
	ldr	r1, [r0, #CLK_RESET_PLLC_BASE]
	tst	r1, #(1<<30)
	orreq	r1, r1, #(1<<30)
	streq	r1, [r0, #CLK_RESET_PLLC_BASE]

	bl	__invalidate_cpu_state
	bl	__enable_coresite_access

	mrc	p15, 0, r0, c1, c0, 1
	orr	r0, r0, #(1 << 6) | (1 << 0)	@ re-enable coherency
	mcr	p15, 0, r0, c1, c0, 1

	/* enable SCU */
	mov32	r0, TEGRA_ARM_PERIF_BASE
	ldr	r1, [r0]
	orr	r1, r1, #1
	str	r1, [r0]

	adr	r4, __tegra_lp2_data
	ldmia	r4, {r5, r7, r12}
	mov	r1, r12			@ ctx_restore = __cortex_a9_restore
	sub	r4, r4, r5
	ldr	r0, [r7, r4]		@ pgdir = tegra_pgd_phys
	b	__return_to_virtual
ENDPROC(tegra_lp2_startup)
	.type	__tegra_lp2_data, %object
__tegra_lp2_data:
	.long	.
	.long	tegra_pgd_phys
	.long	__restart_pllx
	.size	__tegra_lp2_data, . - __tegra_lp2_data

/*
 *	__restart_pllx
 *
 *	  Loads the saved PLLX parameters from tegra_sctx into PLLX, to
 *	  allow it to stabilize while the rest of the CPU state is restored.
 *	  Should be called after the MMU is enabled. Jumps directly
 *	  to __cortex_a9_restore
 */
	.align L1_CACHE_SHIFT
__restart_pllx:
	mov32	r0, tegra_sctx
	ldr	r1, [r0, #0x8]	@ pllx_base
	ldr	r2, [r0, #0xC]	@ pllx_misc
	mov32	r3, (TEGRA_CLK_RESET_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	mov32	r4, (TEGRA_TMRUS_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	str	r2, [r3, #0xe4]	@ pllx_misc
	str	r1, [r3, #0xe0] @ pllx_base
	/* record the time that PLLX will be stable */
	ldr	r1, [r4]
	add	r1, r1, #300
	str	r1, [r0, #0x10]
	/* FIXME: need to record actual power transition here */
	mov	r0, #0
	b	__cortex_a9_l2x0_restart
ENDPROC(__restart_pllx)

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
	setmode	PSR_F_BIT | PSR_I_BIT | SVC_MODE, r9
	bl	__invalidate_cpu_state
	enable_coresite r1

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
