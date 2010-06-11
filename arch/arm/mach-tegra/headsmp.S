/*
 * arch/arm/mach-tegra/headsmp.S
 *
 * SMP initialization routines for Tegra SoCs
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

#define TTB_FLAGS 0x6A	@ IRGN_WBWA, OC_RGN_WBWA, S, NOS

#define PMC_DPD_SAMPLE	0x20
#define PMC_DPD_ENABLE	0x24
#define PMC_SCRATCH1	0x54
#define PMC_SCRATCH39	0x138
#define RST_DEVICES_U	0xc

/*        .section ".cpuinit.text", "ax"*/

.macro	poke_ev, val, tmp
	mov32	\tmp, (TEGRA_EXCEPTION_VECTORS_BASE + 0x100)
	str	\val, [\tmp]
.endm

/*
 *	__invalidate_l1
 *
 *	  Invalidates the L1 data cache (no clean) during initial boot of
 *	  a secondary processor
 *
 *	  Corrupted registers: r0-r6
 */
__invalidate_l1:
	mov	r0, #0
	mcr	p15, 2, r0, c0, c0, 0
	mrc	p15, 1, r0, c0, c0, 0

	movw	r1, #0x7fff
	and	r2, r1, r0, lsr #13

	movw	r1, #0x3ff

	and	r3, r1, r0, lsr #3  @ NumWays - 1
	add	r2, r2, #1	@ NumSets

	and	r0, r0, #0x7
	add	r0, r0, #4	@ SetShift

	clz	r1, r3		@ WayShift
	add	r4, r3, #1	@ NumWays
1:	sub	r2, r2, #1	@ NumSets--
	mov	r3, r4		@ Temp = NumWays
2:	subs    r3, r3, #1	@ Temp--
	mov	r5, r3, lsl r1
	mov	r6, r2, lsl r0
	orr	r5, r5, r6	@ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
	mcr	p15, 0, r5, c7, c6, 2
	bgt	2b
	cmp	r2, #0
	bgt	1b
	dsb
	isb
	mov	pc, lr
ENDPROC(__invalidate_l1)

/*
 *	__invalidate_cpu_state
 *
 *	 Invalidates volatile CPU state (SCU tags, caches, branch address
 *	 arrays, exclusive monitor, etc.) so that they can be safely enabled
 *	 instruction caching and branch predicition enabled as early as
 *	 possible to improve performance
 */
__invalidate_cpu_state:
	clrex
	mov	r0, #0
	mcr	p15, 0, r0, c1, c0, 1	@ disable SMP, prefetch, broadcast
	isb
	mcr	p15, 0, r0, c7, c5, 0	@ invalidate BTAC, i-cache
	mcr	p15, 0, r0, c7, c5, 6	@ invalidate branch pred array
	mcr	p15, 0, r0, c8, c7, 0	@ invalidate unified TLB
	dsb
	isb

	cpu_id	r0
	cmp	r0, #0
	mov32	r1, (TEGRA_ARM_PERIF_BASE + 0xC)
	movne	r0, r0, lsl #2
	movne	r2, #0xf
	movne	r2, r2, lsl r0
	strne	r2, [r1]		@ invalidate SCU tags for CPU

	dsb
	mov	r0, #0x1800
	mcr	p15, 0, r0, c1, c0, 0	@ enable branch prediction, i-cache
	isb
	mov	r10, lr			@ preserve lr of caller
	bl	__invalidate_l1		@ invalidate data cache
	mov	pc, r10			@ return
ENDPROC(__invalidate_cpu_state)

/*
 *	tegra_secondary_startup
 *
 *	 Initial secondary processor boot vector; jumps to kernel's
 *	 secondary_startup routine
 */
ENTRY(tegra_secondary_startup)
	msr	cpsr_fsxc, #0xd3
	bl	__invalidate_cpu_state
	cpu_id	r0
	poke_ev	r0, r1
	b	secondary_startup
ENDPROC(tegra_secondary_startup)

/*
 *	__return_to_virtual(unsigned long pgdir, void (*ctx_restore)(void))
 *
 *	  Restores a CPU to the world of virtual addressing, using the
 *	  specified page tables (which must ensure that a VA=PA mapping
 *	  exists for the __enable_mmu function), and then jumps to
 *	  ctx_restore to restore CPU context and return control to the OS
 */
	.align L1_CACHE_SHIFT
__return_to_virtual:
	orr	r8, r0, #TTB_FLAGS
	mov	lr, r1			@ "return" to ctx_restore
	mov	r3, #0
	mcr	p15, 0, r3, c2, c0, 2	@ TTB control register

	mcr	p15, 0, r8, c2, c0, 1	@ load TTBR1

	mov	r0, #0x1f
	mcr	p15, 0, r0, c3, c0, 0	@ domain access register

	mov32	r0, 0xff0a89a8
	mov32	r1, 0x40e044e0
	mcr	p15, 0, r0, c10, c2, 0	@ PRRR
	mcr	p15, 0, r1, c10, c2, 1	@ NMRR
	mrc	p15, 0, r0, c1, c0, 0
	mov32	r1, 0x0120c302
	bic	r0, r0, r1
	mov32	r1, 0x10c03c7d
	orr	r0, r0, r1

#ifdef CONFIG_ALIGNMENT_TRAP
	orr	r0, r0, #0x2
#else
	bic	r0, r0, #0x2
#endif
	mov	r1, #(domain_val(DOMAIN_USER, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_KERNEL, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_TABLE, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_IO, DOMAIN_CLIENT))
	mcr	p15, 0, r1, c3, c0, 0	@ domain access register
	mcr	p15, 0, r8, c2, c0, 0	@ TTBR0
	b	__turn_mmu_on_again
	andeq	r0, r0, r0
	andeq	r0, r0, r0
	andeq	r0, r0, r0
	andeq	r0, r0, r0
ENDPROC(__return_to_virtual)

/*
 *	__turn_mmu_on_again
 *
 *	  does exactly what it advertises: turns the MMU on, again
 *	  jumps to the *virtual* address lr after the MMU is enabled.
 */
	.align	L1_CACHE_SHIFT
__turn_mmu_on_again:
	mov	r0, r0
	mcr	p15, 0, r0, c1, c0, 0
	mrc	p15, 0, r3, c0, c0, 0
	mov	r3, r3
	mov	r3, lr
	mov	pc, lr
ENDPROC(__turn_mmu_on_again)

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
 *	__enable_coresite_access
 *
 *	  Takes the coresite debug interface out of reset, enables
 *	  access to all CPUs. Called with MMU disabled.
 */
	.align L1_CACHE_SHIFT
__enable_coresite_access:
	mov32	r0, (TEGRA_CLK_RESET_BASE + RST_DEVICES_U)
	mov32	r2, (TEGRA_TMRUS_BASE)

	/* assert reset for 2usec */
	ldr	r1, [r0]
	orr	r1, #(1<<9)
	str	r1, [r0]
	wait_for_us r3, r2, r4
	add	r3, r3, #2
	bic	r1, r1, #(1<<9)
	mov32	r5, 0xC5ACCE55
	mov32	r6, (TEGRA_CSITE_BASE + 0x10fb0)	@ CPUDBG0_LAR
	mov	r7, #CONFIG_NR_CPUS
	wait_until r3, r2, r4
	str	r1, [r0]
access:	str	r5, [r6]
	add	r6, r6, #0x2000
	subs	r7, r7, #1
	bhi	access
	mov	pc, lr
ENDPROC(__enable_coresite_access)
/*
 *	tegra_lp2_startup
 *
 *	  Secondary CPU boot vector when restarting the master CPU following
 *	  an LP2 idle transition. Re-enable coresight access, re-enable
 *	  MMU, re-start PLLX, restore processor context.
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
	setmode	PSR_F_BIT | PSR_I_BIT | SVC_MODE, r9
	bl	__invalidate_cpu_state

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