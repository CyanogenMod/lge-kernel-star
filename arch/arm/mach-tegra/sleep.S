/*
 * arch/arm/mach-tegra/sleep.S
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 * Copyright (c) 2011, Google, Inc.
 *
 * Author: Colin Cross <ccross@android.com>
 *         Gary King <gking@nvidia.com>
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

#include <linux/const.h>
#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/cache.h>
#include <asm/domain.h>
#include <asm/memory.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include <asm/asm-offsets.h>
#include <asm/glue-cache.h>
#include <asm/glue-proc.h>
#include <asm/system.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "asm_macros.h"
#include "sleep.h"

#define CLK_RESET_CCLK_BURST		0x20
#define CLK_RESET_CCLK_DIVIDER		0x24

#define TEGRA_PMC_VIRT		(TEGRA_PMC_BASE - IO_APB_PHYS + IO_APB_VIRT)
#define TEGRA_CLK_RESET_VIRT	(TEGRA_CLK_RESET_BASE - IO_PPSB_PHYS + IO_PPSB_VIRT)

/*
 * tegra_pen_lock
 *
 * spinlock implementation with no atomic test-and-set and no coherence
 * using Peterson's algorithm on strongly-ordered registers
 * used to synchronize a cpu waking up from wfi with entering lp2 on idle
 *
 * SCRATCH37 = r1 = !turn (inverted from Peterson's algorithm)
 * on cpu 0:
 * SCRATCH38 = r2 = flag[0]
 * SCRATCH39 = r3 = flag[1]
 * on cpu1:
 * SCRATCH39 = r2 = flag[1]
 * SCRATCH38 = r3 = flag[0]
 *
 * must be called with MMU on
 * corrupts r0-r3, r12
 */
ENTRY(tegra_pen_lock)
	mov32	r3, TEGRA_PMC_VIRT
	cpu_id	r0
	add	r1, r3, #PMC_SCRATCH37
	cmp	r0, #0
	addeq	r2, r3, #PMC_SCRATCH38
	addeq	r3, r3, #PMC_SCRATCH39
	addne	r2, r3, #PMC_SCRATCH39
	addne	r3, r3, #PMC_SCRATCH38

	mov	r12, #1
	str	r12, [r2]		@ flag[cpu] = 1
	dsb
	str	r12, [r1]		@ !turn = cpu
1:	dsb
	ldr	r12, [r3]
	cmp	r12, #1			@ flag[!cpu] == 1?
	ldreq	r12, [r1]
	cmpeq	r12, r0			@ !turn == cpu?
	beq	1b			@ while !turn == cpu && flag[!cpu] == 1

	mov	pc, lr			@ locked
ENDPROC(tegra_pen_lock)

ENTRY(tegra_pen_unlock)
	dsb
	mov32	r3, TEGRA_PMC_VIRT
	cpu_id	r0
	cmp	r0, #0
	addeq	r2, r3, #PMC_SCRATCH38
	addne	r2, r3, #PMC_SCRATCH39
	mov	r12, #0
	str	r12, [r2]
	mov	pc, lr
ENDPROC(tegra_pen_unlock)

/*
 * tegra_cpu_wfi
 *
 * puts current CPU in wfi
 * CPU core clock-gates itself during wfi
 *
 * corrupts r0-r3
 * must be called with MMU on
 */
ENTRY(tegra_cpu_wfi)
	dsb
	wfi
	mov	pc, lr
ENDPROC(tegra_cpu_wfi)

/*
 * tegra_cpu_exit_coherency
 *
 * Exits SMP coherency.
 * corrupts r4-r5
 */
ENTRY(tegra_cpu_exit_coherency)
	exit_smp r4, r5
	mov	pc, lr
ENDPROC(tegra_cpu_exit_coherency)

#ifdef CONFIG_PM_SLEEP
/*
 * Restore CPU state for a suspend
 *
 * NOTE: This is a copy of cpu_resume in arch/arm/sleep.S that has been
 *	 modified to work with an L2 cache.
 */
	.align	L1_CACHE_SHIFT
ENTRY(tegra_cpu_resume_phys)
#if USE_TEGRA_CPU_SUSPEND
#ifdef CONFIG_SMP
	adr	r0, tegra_phys_sleep_sp
	ALT_SMP(mrc p15, 0, r1, c0, c0, 5)
	ALT_UP(mov r1, #0)
	and	r1, r1, #15
	ldr	r0, [r0, r1, lsl #2]		@ stack phys addr
#else
	ldr	r0, tegra_phys_sleep_sp		@ stack phys addr
#endif
	setmode	PSR_I_BIT | PSR_F_BIT | SVC_MODE, r1  @ set SVC, irqs off
#ifdef MULTI_CPU
	@ load v:p, stack, return fn, resume fn
  ARM(	ldmia	r0!, {r1, sp, lr, pc}	)
THUMB(	ldmia	r0!, {r1, r2, r3, r4}	)
THUMB(	mov	sp, r2			)
THUMB(	mov	lr, r3			)
THUMB(	bx	r4			)
#else
	@ load v:p, stack, return fn
  ARM(	ldmia	r0!, {r1, sp, lr}	)
THUMB(	ldmia	r0!, {r1, r2, lr}	)
THUMB(	mov	sp, r2			)
	b	cpu_do_resume
#endif
#else
	/* Use the standard cpu_resume. */
	b	cpu_resume
#endif
ENDPROC(tegra_cpu_resume_phys)

#if USE_TEGRA_CPU_SUSPEND
	.align	L1_CACHE_SHIFT
	.globl	tegra_phys_sleep_sp
tegra_phys_sleep_sp:
	.rept	4
	.long	0		@ preserve stack phys ptr here
	.endr
	.align	L1_CACHE_SHIFT  @ nothing else must be in this cache line
#endif

/*
 * tegra_cpu_suspend
 *
 * Save CPU suspend state
 * NOTE: This is a copy of cpu_suspend in arch/arm/sleep.S that has been
 *	 modified to work with an L2 cache.
 *
 * Input:
 *	r1 = v:p offset
 *	r3 = virtual return function
 * Output:
 *	sp is decremented to allocate space for CPU state on stack
 * r0-r3,r8-r10,lr corrupted
 */
	.align	L1_CACHE_SHIFT
ENTRY(tegra_cpu_suspend)
#if USE_TEGRA_CPU_SUSPEND
	mov	r9, lr
#ifdef MULTI_CPU
	mov32	r10, processor
	mov	r2, sp			@ current virtual SP
	ldr	r0, [r10, #CPU_SLEEP_SIZE] @ size of CPU sleep state
	ldr	ip, [r10, #CPU_DO_RESUME] @ virtual resume function
	sub	sp, sp, r0		@ allocate CPU state on stack
	mov	r0, sp			@ save pointer
	add	ip, ip, r1		@ convert resume fn to phys
	stmfd	sp!, {r1, r2, r3, ip}	@ save v:p, virt SP, retfn, phys resume fn
	mov	lr, pc
	ldr	pc, [r10, #CPU_DO_SUSPEND] @ save CPU state
#else
	mov	r2, sp			@ current virtual SP
	mov32	r0, cpu_suspend_size
	sub	sp, sp, r0		@ allocate CPU state on stack
	mov	r0, sp			@ save pointer
	stmfd	sp!, {r1, r2, r3}	@ save v:p, virt SP, return fn
	bl	cpu_do_suspend
#endif
	dsb

	/* Disable the data cache */
	mrc	p15, 0, r10, c1, c0, 0
	bic	r10, r10, #CR_C
	dsb
	mcr	p15, 0, r10, c1, c0, 0
	isb

	/* Flush data cache */
#ifdef MULTI_CACHE
	mov32	r10, cpu_cache
	mov	lr, pc
	ldr	pc, [r10, #CACHE_FLUSH_KERN_ALL]
#else
	bl	__cpuc_flush_kern_all
#endif
#ifdef CONFIG_CACHE_L2X0
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	cpu_id  r2
	cmp r2, #0
	bne no_l2_sync
#endif
	/* Issue a PL310 cache sync operation */
	dsb
	mov32	r2, TEGRA_PL310_VIRT
	movw	r1, 0x730		@ cache sync
	add	r2, r2, r1
	mov	r1, #0
	str	r1, [r2]
#endif

no_l2_sync:
	/* Invalidate the TLBs & BTAC */
	mov	r1, #0
	mcr	p15, 0, r1, c8, c3, 0	@ invalidate shared TLBs
	mcr	p15, 0, r1, c7, c1, 6	@ invalidate shared BTAC
	dsb
	isb

	/* Turn off SMP coherency */
	exit_smp r1, r2

	/* Convert SP from virtual to physical address. */
	movw	r1, #0xFFF
	bic	r2, sp, r1		@ VA & 0xFFFFF000
	mcr	p15, 0, r2, c7, c8, 0	@ V2PPRPC
	mrc	p15, 0, r2, c7, c4, 0	@ PAR
	bic	r2, r2, r1		@ PA & 0xFFFFF000
	and	r0, sp, r1		@ VA & 0x00000FFF
	orr	r2, r0, r2		@ (PA & 0xFFFFF000) | (VA & 0x00000FFF)

	mov32	r3, tegra_phys_sleep_sp	@ per-CPU phys SP save area

#ifdef CONFIG_SMP
	ALT_SMP(mrc p15, 0, lr, c0, c0, 5)
	ALT_UP(mov lr, #0)
	and	lr, lr, #15
#else
	mov	lr, #0
#endif

	/* Save the normal PRRR value */
	mrc	p15, 0, r0, c10, c2, 0	@ PRRR

	/* Override all remappings to strongly ordered */
	mov	r1, #0
	mcr	p15, 0, r1, c10, c2, 0	@ PRRR
	mcr	p15, 0, r1, c8, c7, 0	@ invalidate local TLBs
	dsb
	isb

	/* Save the physical stack pointer */
	str	r2, [r3, lr, lsl #2]	@ save phys SP

	/* Restore the regular remappings */
	mcr	p15, 0, r0, c10, c2, 0	@ PRRR
	mcr	p15, 0, r1, c8, c7, 0	@ invalidate local TLBs
	dsb
	isb

	mov	pc, r9
#else
	/* Use the standard cpu_suspend. */
	mov	r8, lr
	bl	cpu_suspend
	exit_smp r0, r2
	mov	pc, r8
#endif
ENDPROC(tegra_cpu_suspend)

/*
 * tegra_cpu_save
 *
 * Input:
 *	r0 = v:p offset
 *	r12 = return to the caller of this function
 *	lr = resume address
 * Output:
 *	r0 = v:p offset
 *	r7 = SP after saving the registers but before cpu_suspend, suitable
 *	     for restoring an aborted suspend
 *	sp = SP after tegra_cpu_suspend (the 'real' SP)
 * Saves r4-r11 on the stack
 * Corrupts r1, r3-r10
 */

ENTRY(tegra_cpu_save)
	push_ctx_regs r1		@ save context registers

	adr	r3, tegra_cpu_resume

	mov	r7, sp			@ SP after reg save, before suspend

#if USE_TEGRA_CPU_SUSPEND
	cpu_id	r4
	mov32	r5, tegra_cpu_context   @ address of non-cacheable context page
	ldr	r5, [r5]		@ non-cacheable context save area
	mov	r6, #0x400		@ size of one CPU context stack area
	add	r4, r4, #1
	smlabb	sp, r6, r4, r5		@ context area for this CPU
	push_stack_token r4		@ debug check word
	stmfd	sp!, {r7}		@ save the real stack pointer
	push_stack_token r4		@ debug check word
#endif

	mov	r4, r12
	mov	r5, r0
	mov	r6, r2
	mov	r1, r0
	bl	tegra_cpu_suspend
	mov	r0, r5
	mov	r2, r6
	mov	pc, r4
ENDPROC(tegra_cpu_save)

/*
 * tegra_sleep_cpu_save(unsigned long v2p)
 *
 * enters suspend in LP2 by turning off the mmu and jumping to
 * tegra?_tear_down_cpu
 */
ENTRY(tegra_sleep_cpu_save)
	mov	r12, pc 		@ return here is via r12
	b	tegra_cpu_save

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	mov32	r1, tegra2_tear_down_cpu
#else
	mov32	r1, tegra3_tear_down_cpu
#endif
	add	r1, r1, r0
	b	tegra_turn_off_mmu
ENDPROC(tegra_sleep_cpu_save)

/*
 * tegra_cpu_resume
 *
 * reloads the volatile CPU state from the context area
 * initializes the processor mode stacks
 * the mmu should be on and the CPU should be coherent before this is called
 */
	.align L1_CACHE_SHIFT
tegra_cpu_resume:
	mov	r0, #0
	mcr	p15, 0, r0, c8, c3, 0	@ invalidate TLB
	mcr	p15, 0, r0, c7, c5, 6	@ flush BTAC
	mcr	p15, 0, r0, c7, c5, 0	@ flush instruction cache
	dsb
	isb

#if USE_TEGRA_CPU_SUSPEND
	pop_stack_token r4, r5		@ check stack debug token
	ldmfd	sp!, {r0}		@ get the real stack pointer
	pop_stack_token r4, r5		@ check stack debug token
	mov	sp, r0			@ switch to the real stack pointer
#endif

	bl	cpu_init

	pop_ctx_regs r1, r2		@ restore context registers
	mov	pc, lr

/*
 * tegra_turn_off_mmu
 *
 * r0 = v2p
 * r1 = physical address to jump to with mmu off
 */
ENTRY(tegra_turn_off_mmu)
	mov32	r3, tegra_shut_off_mmu
	add	r3, r3, r0
	mov	r0, r1
	mov	pc, r3
ENDPROC(tegra_turn_off_mmu)

tegra_pgd_phys_address:
	.word	tegra_pgd_phys

/*
 * tegra_shut_off_mmu
 *
 * r0 = physical address to jump to with mmu off
 *
 * called with VA=PA mapping
 * turns off MMU, icache, dcache and branch prediction
 */
	.align	L1_CACHE_SHIFT
tegra_shut_off_mmu:
	mrc	p15, 0, r3, c1, c0, 0
	movw	r2, #CR_I | CR_Z | CR_C | CR_M
	bic	r3, r3, r2
	dsb
	mcr	p15, 0, r3, c1, c0, 0
	isb
	mov	pc, r0

/*
 * tegra_cpu_clk32k
 *
 * In LP2 the normal cpu clock pllx will be turned off. Switch the CPU to pllp
 */
ENTRY(tegra_cpu_pllp)
	/* in LP2 idle (SDRAM active), set the CPU burst policy to PLLP */
	mov32	r5, TEGRA_CLK_RESET_BASE
	mov	r0, #(2 << 28)			@ burst policy = run mode
	orr	r0, r0, #(4 << 4)		@ use PLLP in run mode burst
	str	r0, [r5, #CLK_RESET_CCLK_BURST]
	mov	r0, #0
	str	r0, [r5, #CLK_RESET_CCLK_DIVIDER]
	mov	pc, lr
ENDPROC(tegra_cpu_pllp)
#endif

#ifdef CONFIG_TRUSTED_FOUNDATIONS
/*
 * tegra_generic_smc
 *
 * r0 = smc type
 * r1 = smc subtype
 * r2 = argument passed to smc
 *
 * issues SMC (secure monitor call) instruction with
 * the specified parameters.
 */
ENTRY(tegra_generic_smc)
	adr	r3, __tegra_smc_stack
	stmia	r3, {r4-r12, lr}
	mov	r3, #0
	mov	r4, #0
	dsb
	smc	#0
	adr	r3, __tegra_smc_stack
	ldmia	r3, {r4-r12, pc}
ENDPROC(tegra_generic_smc)
	.type   __tegra_smc_stack, %object
__tegra_smc_stack:
	.long   0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.size   __tegra_smc_stack, . - __tegra_smc_stack
#endif
