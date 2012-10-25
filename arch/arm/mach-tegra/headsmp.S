/*
 * arch/arm/mach-tegra/headsmp.S
 *
 * CPU initialization routines for Tegra SoCs
 *
 * Copyright (c) 2009-2011, NVIDIA Corporation.
 * Copyright (c) 2011 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *         Gary King <gking@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/cache.h>
#include <asm/page.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "asm_macros.h"
#include "reset.h"
#include "sleep.h"

#define DEBUG_CPU_RESET_HANDLER	0	/* Non-zero enables debug code */

#define PMC_SCRATCH41		0x140

#define RESET_DATA(x)		((TEGRA_RESET_##x)*4)


#ifdef CONFIG_SMP
/*
 *	tegra_secondary_startup
 *
 *	 Initial secondary processor boot vector; jumps to kernel's
 *	 secondary_startup routine. Used for initial boot and hotplug
 *	 of secondary CPUs.
 */
ENTRY(tegra_secondary_startup)
	bl	tegra_enable_coresite
	bl	__invalidate_cpu_state
	b	secondary_startup
ENDPROC(tegra_secondary_startup)
#endif

#ifdef CONFIG_PM_SLEEP
/*
 *	tegra_resume
 *
 *	  CPU boot vector when restarting the a CPU following
 *	  an LP2 transition. Also branched to by LP0 and LP1 resume after
 *	  re-enabling sdram.
 */
ENTRY(tegra_resume)
	bl	tegra_enable_coresite
	bl	__invalidate_cpu_state

	cpu_id	r0
	cmp	r0, #0				@ CPU0?
	bne	tegra_cpu_resume_phys		@ no

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	@ Clear the flow controller flags for this CPU.
	mov32	r2, TEGRA_FLOW_CTRL_BASE+8	@ CPU0 CSR
	ldr	r1, [r2]
	orr	r1, r1, #(1 << 15) | (1 << 14)	@ write to clear event & intr
	movw	r0, #0x0FFD	@ enable, cluster_switch, immed, & bitmaps
	bic	r1, r1, r0
	str	r1, [r2]
#endif

	/* enable SCU */
	mov32	r0, TEGRA_ARM_PERIF_BASE
	ldr	r1, [r0]
	orr	r1, r1, #1
	str	r1, [r0]

#ifdef CONFIG_TRUSTED_FOUNDATIONS
	/* wake up (should have specified args?) */
	bl	tegra_generic_smc
#endif

	b	tegra_cpu_resume_phys
ENDPROC(tegra_resume)
#endif

/*
 *	__invalidate_cpu_state
 *
 *	  Invalidates volatile CPU state (SCU tags, caches, branch address
 *	  arrays, exclusive monitor, etc.) so that they can be safely enabled
 *	  instruction caching and branch predicition enabled
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
	/* fall through */

/*
 *	tegra_invalidate_l1
 *
 *	  Invalidates the L1 data cache (no clean) during initial boot of a cpu
 *
 *	  Corrupted registers: r0-r6
 */
tegra_invalidate_l1:
	mov	r0, #0
	mcr	p15, 2, r0, c0, c0, 0
	mrc	p15, 1, r0, c0, c0, 0

	movw	r1, #0x7fff
	and	r2, r1, r0, lsr #13

	movw	r1, #0x3ff

	and	r3, r1, r0, lsr #3	@ NumWays - 1
	add	r2, r2, #1	@ NumSets

	and	r0, r0, #0x7
	add	r0, r0, #4	@ SetShift

	clz	r1, r3		@ WayShift
	add	r4, r3, #1	@ NumWays
1:	sub	r2, r2, #1	@ NumSets--
	mov	r3, r4		@ Temp = NumWays
2:	subs	r3, r3, #1	@ Temp--
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

	/* Enable Coresight access on cpu */
tegra_enable_coresite:
	mov32	r0, 0xC5ACCE55
	mcr	p14, 0, r0, c7, c12, 6
	mov	pc, lr

/*
 * __tegra_cpu_reset_handler_halt_failed:
 *
 * Alternate entry point for reset handler for cases where the
 * WFI halt failed to take effect.
 *
 */
	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_start)

/*
 * __tegra_cpu_reset_handler:
 *
 * Common handler for all CPU reset events.
 *
 * Register usage within the reset handler:
 *
 *	R7  = CPU present (to the OS) mask
 *	R8  = CPU in LP1 state mask
 *	R9  = CPU in LP2 state mask
 *	R10 = CPU number
 *	R11 = CPU mask
 *	R12 = pointer to reset handler data
 *
 * NOTE: This code is copied to IRAM. All code and data accesses
 *	 must be position-independent.
 */

	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler)

#if DEBUG_CPU_RESET_HANDLER
	mov32	r0, 0xC5ACCE55
	mcr	p14, 0, r0, c7, c12, 6		@ Enable CoreSight access
	b	.
#endif
#ifndef CONFIG_TRUSTED_FOUNDATIONS
	cpsid	aif, 0x13			@ SVC mode, interrupts disabled
	mrc	p15, 0, r0, c0, c0, 0		@ read main ID register
	and	r5, r0, #0x00f00000		@ variant
	and	r6, r0, #0x0000000f		@ revision
	orr	r6, r6, r5, lsr #20-4		@ combine variant and revision
#ifdef CONFIG_ARM_ERRATA_743622
	teq	r6, #0x20			@ present in r2p0
	teqne	r6, #0x21			@ present in r2p1
	teqne	r6, #0x22			@ present in r2p2
	teqne	r6, #0x27			@ present in r2p7
	teqne	r6, #0x29			@ present in r2p9
	mrceq	p15, 0, r10, c15, c0, 1		@ read diagnostic register
	orreq	r10, r10, #1 << 6		@ set bit #6
	mcreq	p15, 0, r10, c15, c0, 1		@ write diagnostic register
#endif
#endif
	mrc	p15, 0, r10, c0, c0, 5		@ MPIDR
	and	r10, r10, #0x3			@ R10 = CPU number
	mov	r11, #1
	mov	r11, r11, lsl r10  		@ R11 = CPU mask
	adr	r12, __tegra_cpu_reset_handler_data

#ifdef CONFIG_SMP
	/* Does the OS know about this CPU? */
	ldr	r7, [r12, #RESET_DATA(MASK_PRESENT)]
	tst	r7, r11 			@ if !present
	bleq	__die				@ CPU not present (to OS)
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* If CPU1, don't let CPU0 reset CPU1 now that CPU1 is coming up. */
	mov32	r6, TEGRA_PMC_BASE
	mov	r0, #0
	cmp	r10, #0
	strne	r0, [r6, #PMC_SCRATCH41]
#endif

#ifdef CONFIG_PM_SLEEP
	/* Waking up from LP1? */
	ldr	r8, [r12, #RESET_DATA(MASK_LP1)]
	tst	r8, r11				@ if in_lp1
	beq	__is_not_lp1
	cmp	r10, #0
	bne	__die				@ only CPU0 can be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_LP1)]
	cmp	lr, #0
	bleq	__die				@ no LP1 startup handler
	bx	lr
__is_not_lp1:
#endif

	/* Waking up from LP2? */
	ldr	r9, [r12, #RESET_DATA(MASK_LP2)]
	tst	r9, r11				@ if in_lp2
	beq	__is_not_lp2
	ldr	lr, [r12, #RESET_DATA(STARTUP_LP2)]
	cmp	lr, #0
	bleq	__die				@ no LP2 startup handler
	bx	lr

__is_not_lp2:

#ifdef CONFIG_SMP
	/* Can only be secondary boot (initial or hotplug) but CPU 0
	   cannot be here. */
	cmp	r10, #0
	bleq	__die				@ CPU0 cannot be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_SECONDARY)]
	cmp	lr, #0
	bleq	__die				@ no secondary startup handler
	bx	lr
#endif

/*
 * We don't know why the CPU reset. Just kill it.
 * The LR register will contain the address we died at + 4.
 */

__die:
	sub	lr, lr, #4
	mov32	r7, TEGRA_PMC_BASE
	str	lr, [r7, #PMC_SCRATCH41]

	mov32	r7, TEGRA_CLK_RESET_BASE

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	mov32	r0, 0x1111
	mov	r1, r0, lsl r10
	str	r1, [r7, #0x340]		@ CLK_RST_CPU_CMPLX_SET
#else
	mov32	r6, TEGRA_FLOW_CTRL_BASE

	cmp	r10, #0
	moveq	r1, #FLOW_CTRL_HALT_CPU0_EVENTS
	moveq	r2, #FLOW_CTRL_CPU0_CSR
	movne	r1, r10, lsl #3
	addne	r2, r1, #(FLOW_CTRL_CPU1_CSR-8)
	addne	r1, r1, #(FLOW_CTRL_HALT_CPU1_EVENTS-8)

	/* Clear CPU "event" and "interrupt" flags and power gate
	   it when halting but not before it is in the "WFI" state. */
	ldr	r0, [r6, +r2]
	orr	r0, r0, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	orr	r0, r0, #FLOW_CTRL_CSR_ENABLE
	str	r0, [r6, +r2]

	/* Unconditionally halt this CPU */
	mov	r0, #FLOW_CTRL_WAITEVENT
	str	r0, [r6, +r1]
	ldr	r0, [r6, +r1]			@ memory barrier

	dsb
	isb
	wfi					@ CPU should be power gated here

	/* If the CPU didn't power gate above just kill it's clock. */

	mov	r0, r11, lsl #8
	str	r0, [r7, #348]			@ CLK_CPU_CMPLX_SET
#endif
	/* If the CPU still isn't dead, just spin here. */
	b	.

ENDPROC(__tegra_cpu_reset_handler)
	.align L1_CACHE_SHIFT
	.type	__tegra_cpu_reset_handler_data, %object
	.globl	__tegra_cpu_reset_handler_data
__tegra_cpu_reset_handler_data:
	.rept	TEGRA_RESET_DATA_SIZE
	.long	0
	.endr
	.size	__tegra_cpu_reset_handler_data, .-tegra_cpu_reset_handler_data
	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_end)
