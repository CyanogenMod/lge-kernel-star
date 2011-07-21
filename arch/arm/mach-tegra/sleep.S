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
#define TEGRA_ARM_PERIF_VIRT	(TEGRA_ARM_PERIF_BASE - IO_CPU_PHYS + IO_CPU_VIRT)
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
 * puts current CPU in clock-gated wfi using the flow controller
 *
 * corrupts r0-r3
 * must be called with MMU on
 */
ENTRY(tegra_cpu_wfi)
	cpu_id	r0
	cpu_to_halt_reg r1, r0
	cpu_to_csr_reg r2, r0
	mov32	r0, TEGRA_FLOW_CTRL_VIRT
	mov	r3, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	str	r3, [r0, r2]		@ clear event & interrupt status
	mov	r3, #FLOW_CTRL_WAIT_FOR_INTERRUPT | FLOW_CTRL_JTAG_RESUME
	str	r3, [r0, r1]		@ put flow controller in wait irq mode
	dsb
	wfi
	mov	r3, #0
	str	r3, [r0, r1]		@ clear flow controller halt status
	mov	r3, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	str	r3, [r0, r2]		@ clear event & interrupt status
	dsb
	mov	pc, lr
ENDPROC(tegra_cpu_wfi)

#ifdef CONFIG_PM_SLEEP
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
	mov32	r5, TEGRA_ARM_PERIF_VIRT + 0xC
	mov	r4, r4, lsl #2
	mov	r6, #0xf
	mov	r6, r6, lsl r4
	str	r6, [r5]		@ invalidate SCU tags for CPU

	mov	pc, lr
ENDPROC(tegra_cpu_save)

/*
 * tegra_sleep_cpu(unsigned long v2p)
 *
 * enters suspend in LP2 by turning off the mmu and jumping to
 * tegra?_tear_down_cpu
 */
ENTRY(tegra_sleep_cpu)
	mov	r3, lr			@ set resume address to lr
	bl	tegra_cpu_save

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	mov32	r1, tegra2_tear_down_cpu
#endif
	add	r1, r1, r0
	b	tegra_turn_off_mmu
ENDPROC(tegra_sleep_cpu)

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

	bl	cpu_init

	ldmfd	sp!, {r4}
	mcr	p15, 0, r4, c15, c0, 1	@ write diagnostic register
	ldmfd	sp!, {lr}
	ldmfd	sp!, {r4 - r11}
	mov	pc, lr

/*
 * tegra_turn_off_mmu
 *
 * r0 = v2p
 * r1 = physical address to jump to with mmu off
 */
ENTRY(tegra_turn_off_mmu)
	/*
	 * change page table pointer to tegra_pgd_phys, so that IRAM
	 * and MMU shut-off will be mapped virtual == physical
	 */
	mrc	p15, 0, r2, c2, c0, 0	@ TTB 0
	mov32	r3, ~PAGE_MASK
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
	movw	r2, #(1 << 12) | (1 << 11) | (1 << 2) | (1 << 0)
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
