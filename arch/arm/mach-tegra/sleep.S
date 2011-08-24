/*
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include "sleep.h"

#define EMC_CFG				0xc
#define EMC_ADR_CFG			0x10
#define EMC_REFRESH			0x70
#define EMC_NOP				0xdc
#define EMC_SELF_REF			0xe0
#define EMC_REQ_CTRL			0x2b0
#define EMC_EMC_STATUS			0x2b4

#define CLK_RESET_CCLK_BURST		0x20
#define CLK_RESET_CCLK_DIVIDER		0x24
#define CLK_RESET_SCLK_BURST		0x28
#define CLK_RESET_SCLK_DIVIDER		0x2c

#define CLK_RESET_PLLC_BASE		0x80
#define CLK_RESET_PLLM_BASE		0x90
#define CLK_RESET_PLLP_BASE		0xa0
#define CLK_RESET_PLLP_OUTA		0xa4
#define CLK_RESET_PLLP_OUTB		0xa8
#define CLK_RESET_PLLP_MISC		0xac
#define CLK_RESET_PLLX_BASE		0xe0
#define CLK_RESET_PLLX_MISC		0xe4

#define FLOW_CTRL_WAITEVENT		(2 << 29)
#define FLOW_CTRL_STOP_UNTIL_IRQ	(4 << 29)
#define FLOW_CTRL_JTAG_RESUME		(1 << 28)
#define FLOW_CTRL_IRQ_RESUME		(1 << 10)
#define FLOW_CTRL_FIQ_RESUME		(1 << 8)

#define TEGRA_PMC_VIRT		(TEGRA_PMC_BASE - IO_APB_PHYS + IO_APB_VIRT)
#define TEGRA_ARM_PERIF_VIRT	(TEGRA_ARM_PERIF_BASE - IO_CPU_PHYS + IO_CPU_VIRT)
#define TEGRA_FLOW_CTRL_VIRT	(TEGRA_FLOW_CTRL_BASE - IO_PPSB_PHYS + IO_PPSB_VIRT)
#define TEGRA_CLK_RESET_VIRT	(TEGRA_CLK_RESET_BASE - IO_PPSB_PHYS + IO_PPSB_VIRT)

/* returns the offset of the flow controller halt register for a cpu */
.macro cpu_to_halt_reg rd, rcpu
	cmp	\rcpu, #0
	subne	\rd, \rcpu, #1
	movne	\rd, \rd, lsl #3
	addne	\rd, \rd, #0x14
	moveq	\rd, #0
.endm

/* returns the ID of the current processor */
.macro cpu_id, rd
	mrc	p15, 0, \rd, c0, c0, 5
	and	\rd, \rd, #0xF
.endm

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
	ldr	r3, =TEGRA_PMC_VIRT
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
	ldr	r3, =TEGRA_PMC_VIRT
	cpu_id	r0
	cmp	r0, #0
	addeq	r2, r3, #PMC_SCRATCH38
	addne	r2, r3, #PMC_SCRATCH39
	mov	r12, #0
	str	r12, [r2]
	mov	pc, lr
ENDPROC(tegra_pen_unlock)

ENTRY(tegra_cpu_set_resettable_soon)
	ldr	r3, =TEGRA_PMC_VIRT
	add	r1, r3, #PMC_SCRATCH41
	mov	r12, #CPU_RESETTABLE_SOON
	str	r12, [r1]
	mov	pc, lr
ENDPROC(tegra_cpu_set_resettable_soon)

ENTRY(tegra_cpu_is_resettable_soon)
	ldr	r3, =TEGRA_PMC_VIRT
	add	r1, r3, #PMC_SCRATCH41
	ldr	r12, [r1]
	cmp	r12, #CPU_RESETTABLE_SOON
	moveq	r0, #1
	movne	r0, #0
	mov	pc, lr
ENDPROC(tegra_cpu_is_resettable_soon)

/*
 * tegra_cpu_save
 *
 * r0 = v2p
 * r3 = resume address
 * saves r4-r11 on the stack
 * modifies sp, returns the sp after saving registers but before cpu_suspend,
 *   appropriate for restoring an aborted suspend that does not call cpu_resume
 * corrupts r1, r3-r6, r9, r10
 */

ENTRY(tegra_cpu_save)
	stmfd	sp!, {r4 - r11}
	stmfd	sp!, {r3}
	mrc	p15, 0, r4, c15, c0, 1	@ read diagnostic register
	stmfd	sp!, {r4}

	adr	r3, tegra_cpu_resume

	mov	r7, sp
	mov	r4, lr
	mov	r5, r0
	mov	r6, r2
	mov	r1, r0
	bl	cpu_suspend
	mov	lr, r4
	mov	r0, r5
	mov	r2, r6
	mov	sp, r7

	mov	r4, #0
	mcr	p15, 0, r4, c1, c0, 1	@ exit coherency
	isb
	cpu_id	r4
	ldr	r5, =TEGRA_ARM_PERIF_VIRT + 0xC
	mov	r4, r4, lsl #2
	mov	r6, #0xf
	mov	r6, r6, lsl r4
	str	r6, [r5]		@ invalidate SCU tags for CPU

	mov	pc, lr
ENDPROC(tegra_cpu_save)

/*
 * tegra_cpu_wfi
 *
 * puts current CPU in clock-gated wfi using the flow controller
 * if the state is saved before calling tegra_cpu_wfi, the cpu can go from
 * wfi directly to reset
 *
 * corrupts r0-r2
 * must be called with MMU on
 */
ENTRY(tegra_cpu_wfi)
	cpu_id	r0
	cpu_to_halt_reg r1, r0
	ldr	r0, =TEGRA_FLOW_CTRL_VIRT
	mov	r2, #FLOW_CTRL_WAITEVENT | FLOW_CTRL_JTAG_RESUME
	str	r2, [r0, r1]		@ put flow controller in wait event mode
	ldr	r2, [r0, r1]
	isb
	dsb
	wfi
	mov	r2, #0
	str	r2, [r0, r1]
	ldr	r2, [r0, r1]
	mov	pc, lr
ENDPROC(tegra_cpu_wfi)

/*
 * tegra_sleep_reset(unsigned long v2p)
 *
 * puts the current cpu in reset
 * uses tegra_cpu_save to take the cpu out of coherence
 * should never return
 */
ENTRY(tegra_sleep_reset)
	bl	tegra_cpu_save
	cpu_id	r0
	bl	tegra_cpu_reset
	mov	pc, lr
ENDPROC(tegra_sleep_reset)

/*
 * tegra_sleep_core(unsigned long v2p)
 *
 * enters suspend in LP0 or LP1 by turning off the mmu and jumping to
 * tegra_tear_down_core in IRAM
 */
ENTRY(tegra_sleep_core)
	mov	r3, lr			@ set resume address to lr
	bl	tegra_cpu_save

	ldr	r1, =tegra_tear_down_core
	ldr	r2, =tegra_iram_start
	sub	r1, r1, r2
	ldr	r2, =TEGRA_IRAM_CODE_AREA
	add	r1, r1, r2
	b	tegra_turn_off_mmu
ENDPROC(tegra_sleep_core)

.word

/*
 * tegra_sleep_cpu(unsigned long v2p)
 *
 * enters suspend in LP2 by turning off the mmu and jumping to
 * tegra_tear_down_cpu
 */
ENTRY(tegra_sleep_cpu)
	mov	r3, lr			@ set resume address to lr
	bl	tegra_cpu_save

	ldr	r1, =tegra_tear_down_cpu
	add	r1, r1, r0
	b	tegra_turn_off_mmu
ENDPROC(tegra_sleep_cpu)

/*
 * tegra_sleep_wfi(unsigned long v2p)
 */
ENTRY(tegra_sleep_wfi)
	mov	r3, lr			@ set resume address to lr
	mrc	p15, 0, r2, c1, c0, 1	@ save actlr before exiting coherency
	bl	tegra_cpu_save

	mov	r11, r2

	ldr	r3, =TEGRA_PMC_VIRT
	add	r0, r3, #PMC_SCRATCH41
	mov	r3, #CPU_RESETTABLE
	str	r3, [r0]

	bl	tegra_cpu_wfi

	/*
	 * cpu may be reset while in wfi, which will return through
	 * tegra_secondary_resume to cpu_resume to tegra_cpu_resume
	 * or interrupt may wake wfi, which will return here
	 * cpu state is unchanged - MMU is on, cache is on, coherency is off
	 *
	 * r11 contains the original actlr
	 */

	bl	tegra_pen_lock

	ldr	r3, =TEGRA_PMC_VIRT
	add	r0, r3, #PMC_SCRATCH41
	mov	r3, #CPU_NOT_RESETTABLE
	str	r3, [r0]

	bl	tegra_pen_unlock

	mcr	p15, 0, r11, c1, c0, 1	@ reenable coherency

	@ the cpu was running with coherency disabled, caches may be out of date
	mov     r0, #0
	mcr     p15, 0, r0, c8, c3, 0   @ invalidate TLB
	mcr     p15, 0, r0, c7, c5, 6   @ flush BTAC
#ifdef MULTI_CACHE
	ldr	r10, =cpu_cache
	mov	lr, pc
	ldr	pc, [r10, #CACHE_FLUSH_KERN_ALL]
#else
	bl	__cpuc_flush_kern_all
#endif

	b	tegra_cpu_sleep_abort
ENDPROC(tegra_sleep_wfi)

/*
 * tegra_cpu_resume
 *
 * reloads the volatile CPU state from the context area
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

	bl cpu_init

tegra_cpu_sleep_abort:
	ldmfd	sp!, {r4}
	mcr	p15, 0, r4, c15, c0, 1	@ write diagnostic register
	ldmfd	sp!, {lr}
	ldmfd	sp!, {r4 - r11}
	mov	pc, lr

/*
 * tegra_cpu_reset
 *
 * r0 is cpu to reset
 *
 * puts the specified CPU in wait-for-event mode on the flow controller
 * and puts the CPU in reset
 * can be called on the current cpu or another cpu
 * if called on the current cpu, does not return
 *
 * corrupts r0-r3, r12
 */
ENTRY(tegra_cpu_reset)
	ldr	r3, =TEGRA_PMC_VIRT
	add	r1, r3, #PMC_SCRATCH41
	mov	r12, #CPU_RESETTABLE
	str	r12, [r1]

	cpu_to_halt_reg r1, r0
	ldr	r3, =TEGRA_FLOW_CTRL_VIRT
	mov	r2, #FLOW_CTRL_WAITEVENT | FLOW_CTRL_JTAG_RESUME
	str	r2, [r3, r1]		@ put flow controller in wait event mode
	ldr	r2, [r3, r1]
	isb
	dsb
	movw	r1, 0x1011
	mov	r1, r1, lsl r0
	ldr	r3, =TEGRA_CLK_RESET_VIRT
	str	r1, [r3, #0x340]	@ put slave CPU in reset
	isb
	dsb
	cpu_id	r3
	cmp	r3, r0
	beq	.
	mov	pc, lr
ENDPROC(tegra_cpu_reset)

/*
 * tegra_turn_off_mmu
 *
 * r0 = v2p
 * r1 = physical address to jump to with mmu off
 */
tegra_turn_off_mmu:
	/*
	 * change page table pointer to tegra_pgd_phys, so that IRAM
	 * and MMU shut-off will be mapped virtual == physical
	 */
	mrc	p15, 0, r2, c2, c0, 0	@ TTB 0
	ldr	r3, =~PAGE_MASK
	and	r2, r2, r3
	ldr	r3, tegra_pgd_phys_address
	ldr	r3, [r3]
	orr	r3, r3, r2
	mov	r2, #0
	mcr	p15, 0, r2, c13, c0, 1	@ reserved context
	isb
	mcr	p15, 0, r3, c2, c0, 0	@ TTB 0
	isb

	mov	r2, #0
	mcr	p15, 0, r2, c8, c3, 0	@ invalidate TLB
	mcr	p15, 0, r2, c7, c5, 6	@ flush BTAC
	mcr	p15, 0, r2, c7, c5, 0	@ flush instruction cache

	ldr	r3, =tegra_shut_off_mmu
	add	r3, r3, r0
	mov	r0, r1
	mov	pc, r3

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
tegra_shut_off_mmu:
	mrc	p15, 0, r3, c1, c0, 0
	movw	r2, #(1 << 12) | (1 << 11) | (1 << 2) | (1 << 0)
	bic	r3, r3, r2
	dsb
	mcr	p15, 0, r3, c1, c0, 0
	isb
	mov	pc, r0

	.ltorg

/*
 * tegra_cpu_clk32k
 *
 * In LP2 the normal cpu clock pllx will be turned off. Switch the CPU to pllp
 */
 tegra_cpu_pllp:
	/* in LP2 idle (SDRAM active), set the CPU burst policy to PLLP */
	ldr	r5, =TEGRA_CLK_RESET_BASE
	mov	r0, #(2 << 28)	/* burst policy = run mode */
	orr	r0, r0, #(4 << 4) /* use PLLP in run mode burst */
	str	r0, [r5, #CLK_RESET_CCLK_BURST]
	mov	r0, #0
	str	r0, [r5, #CLK_RESET_CCLK_DIVIDER]
	mov	pc, lr

tegra_tear_down_cpu:
	bl	tegra_cpu_pllp
	b	tegra_enter_sleep

/* START OF ROUTINES COPIED TO IRAM */
	.align L1_CACHE_SHIFT
	.globl tegra_iram_start
tegra_iram_start:

/*
 * tegra_tear_down_core
 *
 * copied into and executed from IRAM
 * puts memory in self-refresh for LP0 and LP1
 */
tegra_tear_down_core:
	bl	tegra_sdram_self_refresh
	bl	tegra_cpu_clk32k
	b	tegra_enter_sleep

/*
 * tegra_cpu_clk32k
 *
 * In LP0 and LP1 all plls will be turned off.  Switch the CPU and system clock
 * to the 32khz clock (clks)
 */
tegra_cpu_clk32k:
	/* start by jumping to clkm to safely disable PLLs, then jump
	 * to clks */
	mov	r0, #(1 << 28)
	str	r0, [r5, #CLK_RESET_SCLK_BURST]
	str	r0, [r5, #CLK_RESET_CCLK_BURST]
	mov	r0, #0
	str	r0, [r5, #CLK_RESET_CCLK_DIVIDER]
	str	r0, [r5, #CLK_RESET_SCLK_DIVIDER]

	/* 2 us delay between changing sclk and disabling PLLs */
	ldr	r7, =TEGRA_TMRUS_BASE
	ldr	r1, [r7]
	add	r1, r1, #3

1:	ldr	r0, [r7]
	cmp	r0, r1
	dmb
	bmi	1b

	/* switch to CLKS */
	mov	r0, #0	/* burst policy = 32KHz */
	str	r0, [r5, #CLK_RESET_SCLK_BURST]

	/* disable PLLP, PLLM, PLLC in LP0 and LP1 states */
	ldr	r0, [r5, #CLK_RESET_PLLM_BASE]
	bic	r0, r0, #(1 << 30)
	str	r0, [r5, #CLK_RESET_PLLM_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLP_BASE]
	bic	r0, r0, #(1 << 30)
	str	r0, [r5, #CLK_RESET_PLLP_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLC_BASE]
	bic	r0, r0, #(1 << 30)
	str	r0, [r5, #CLK_RESET_PLLC_BASE]
	mov	pc, lr

/*
 * tegra_enter_sleep
 *
 * uses flow controller to enter sleep state
 * executes from IRAM with SDRAM in selfrefresh when target state is LP0 and LP1
 * executes from SDRAM with target state is LP2
 */
tegra_enter_sleep:
	ldr	r7, =TEGRA_TMRUS_BASE
	ldr	r1, [r7]
	ldr	r4, =TEGRA_PMC_BASE
	str	r1, [r4, #PMC_SCRATCH38]
	dsb
	ldr	r6, =TEGRA_FLOW_CTRL_BASE

	mov	r0, #FLOW_CTRL_STOP_UNTIL_IRQ
	orr	r0, r0, #FLOW_CTRL_IRQ_RESUME | FLOW_CTRL_FIQ_RESUME
	cpu_id	r1
	cpu_to_halt_reg r1, r1
	str	r0, [r6, r1]
	dsb
	ldr	r0, [r6, r1] /* memory barrier */

halted: dsb
	wfe	/* CPU should be power gated here */
	isb
	b	halted

/*
 * tegra_lp1_reset
 *
 * reset vector for LP1 restore; copied into IRAM during suspend.
 * brings the system back up to a safe starting point (SDRAM out of
 * self-refresh, PLLC, PLLM and PLLP reenabled, CPU running on PLLP,
 * system clock running on the same PLL that it suspended at), and
 * jumps to tegra_lp2_startup to restore PLLX and virtual addressing.
 * physical address of tegra_lp2_startup expected to be stored in
 * PMC_SCRATCH41
 */
ENTRY(tegra_lp1_reset)
	/*
	 * the CPU and system bus are running at 32KHz and executing from
	 * IRAM when this code is executed; immediately switch to CLKM and
	 * enable PLLP.
	 */
	ldr	r0, =TEGRA_CLK_RESET_BASE
	mov	r1, #(1 << 28)
	str	r1, [r0, #CLK_RESET_SCLK_BURST]
	str	r1, [r0, #CLK_RESET_CCLK_BURST]
	mov	r1, #0
	str	r1, [r0, #CLK_RESET_SCLK_DIVIDER]
	str	r1, [r0, #CLK_RESET_CCLK_DIVIDER]

	ldr	r1, [r0, #CLK_RESET_PLLM_BASE]
	tst	r1, #(1 << 30)
	orreq	r1, r1, #(1 << 30)
	streq	r1, [r0, #CLK_RESET_PLLM_BASE]
	ldr	r1, [r0, #CLK_RESET_PLLP_BASE]
	tst	r1, #(1 << 30)
	orreq	r1, r1, #(1 << 30)
	streq	r1, [r0, #CLK_RESET_PLLP_BASE]
	ldr	r1, [r0, #CLK_RESET_PLLC_BASE]
	tst	r1, #(1 << 30)
	orreq	r1, r1, #(1 << 30)
	streq	r1, [r0, #CLK_RESET_PLLC_BASE]

	adr	r2, tegra_sdram_pad_address
	adr	r4, tegra_sdram_pad_save
	mov	r5, #0

padload:
	ldr	r0, [r2, r5]		@ r0 is emc register address

	ldr	r1, [r4, r5]
	str	r1, [r0]		@ set emc register to safe vals

	add	r5, r5, #4
	ldr	r0, tegra_sdram_pad_size
	cmp	r0, r5
	bne	padload

padload_done:
	ldr	r7, =TEGRA_TMRUS_BASE
	ldr	r1, [r7]
	add	r1, r1, #0xff		@ 255uS delay for PLL stabilization

1:	ldr	r0, [r7]
	cmp	r0, r1
	dmb
	bmi	1b

	adr	r4, tegra_sclk_save
	ldr	r4, [r4]
	ldr	r0, =TEGRA_CLK_RESET_BASE
	str	r4, [r0, #CLK_RESET_SCLK_BURST]
	ldr	r4, =((1 << 28) | (4))	@ burst policy is PLLP
	str	r4, [r0, #CLK_RESET_CCLK_BURST]

	ldr	r0, =TEGRA_EMC_BASE
	ldr	r1, [r0, #EMC_CFG]
	bic	r1, r1, #(1 << 31)	@ disable DRAM_CLK_STOP
	str	r1, [r0, #EMC_CFG]

	mov	r1, #0
	str	r1, [r0, #EMC_SELF_REF]	@ take DRAM out of self refresh
	mov	r1, #1
	str	r1, [r0, #EMC_NOP]
	str	r1, [r0, #EMC_NOP]
	str	r1, [r0, #EMC_REFRESH]

	ldr	r1, [r0, #EMC_ADR_CFG]
	tst	r1, #(0x3 << 24)
	moveq	r1, #(0x1 << 8)		@ just 1 device
	movne	r1, #(0x3 << 8)		@ 2 devices

exit_selfrefresh_loop:
	ldr	r2, [r0, #EMC_EMC_STATUS]
	ands	r2, r2, r1
	bne	exit_selfrefresh_loop

	mov	r1, #0
	str	r1, [r0, #EMC_REQ_CTRL]

	ldr	r0, =TEGRA_PMC_BASE
	ldr	r0, [r0, #PMC_SCRATCH41]
	mov	pc, r0
ENDPROC(tegra_lp1_reset)

/*
 * tegra_sdram_self_refresh
 *
 * called with MMU off and caches disabled
 * puts sdram in self refresh
 * must execute from IRAM
 */
tegra_sdram_self_refresh:
	ldr	r1, =TEGRA_EMC_BASE
	mov	r2, #3
	str	r2, [r1, #EMC_REQ_CTRL]		@ stall incoming DRAM requests

emcidle:ldr	r2, [r1, #EMC_EMC_STATUS]
	tst	r2, #4
	beq	emcidle

	mov	r2, #1
	str	r2, [r1, #EMC_SELF_REF]

	ldr	r2, [r1, #EMC_ADR_CFG]
	tst	r2, #(0x3 << 24)
	moveq	r2, #(0x1 << 8)			@ just 1 device
	movne	r2, #(0x3 << 8)			@ 2 devices

emcself:ldr	r3, [r1, #EMC_EMC_STATUS]
	and	r3, r3, r2
	cmp	r3, r2
	bne	emcself				@ loop until DDR in self-refresh

	adr	r2, tegra_sdram_pad_address
	adr	r3, tegra_sdram_pad_safe
	adr	r4, tegra_sdram_pad_save
	mov	r5, #0

padsave:
	ldr	r0, [r2, r5]			@ r0 is emc register address

	ldr	r1, [r0]
	str	r1, [r4, r5]			@ save emc register

	ldr	r1, [r3, r5]
	str	r1, [r0]			@ set emc register to safe vals

	add	r5, r5, #4
	ldr	r0, tegra_sdram_pad_size
	cmp	r0, r5
	bne	padsave
padsave_done:

	ldr	r5, =TEGRA_CLK_RESET_BASE
	ldr	r0, [r5, #CLK_RESET_SCLK_BURST]
	adr	r2, tegra_sclk_save
	str	r0, [r2]
	dsb
	mov	pc, lr

tegra_sdram_pad_address:
	.word	TEGRA_APB_MISC_BASE + 0x8c8 /* XM2CFGCPADCTRL */
	.word	TEGRA_APB_MISC_BASE + 0x8cc /* XM2CFGDPADCTRL */
	.word	TEGRA_APB_MISC_BASE + 0x8d0 /* XM2CLKCFGPADCTRL */
	.word	TEGRA_APB_MISC_BASE + 0x8d4 /* XM2COMPPADCTRL */
	.word	TEGRA_APB_MISC_BASE + 0x8d8 /* XM2VTTGENPADCTRL */
	.word	TEGRA_APB_MISC_BASE + 0x8e4 /* XM2CFGCPADCTRL2 */
	.word	TEGRA_APB_MISC_BASE + 0x8e8 /* XM2CFGDPADCTRL2 */

tegra_sdram_pad_size:
	.word	tegra_sdram_pad_size - tegra_sdram_pad_address

tegra_sdram_pad_safe:
	.word	0x8
	.word	0x8
	.word	0x0
	.word	0x8
	.word	0x5500
	.word	0x08080040
	.word	0x0

tegra_sclk_save:
	.word	0x0

tegra_sdram_pad_save:
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0

	.ltorg
/* dummy symbol for end of IRAM */
	.align L1_CACHE_SHIFT
	.globl tegra_iram_end
tegra_iram_end:
	b	.
