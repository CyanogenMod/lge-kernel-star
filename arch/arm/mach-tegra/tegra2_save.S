/*
 * arch/arm/mach-tegra/cortex_a9_save.S
 *
 * CPU state save & restore routines for CPU hotplug
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
#include <asm/vfpmacros.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "power.h"

/*	.section ".cpuinit.text", "ax"*/

#define TTB_FLAGS 0x6A	@ IRGN_WBWA, OC_RGN_WBWA, S, NOS

#define CONTEXT_SIZE_WORDS_SHIFT 7
#define CONTEXT_SIZE_WORDS (1<<CONTEXT_SIZE_WORDS_SHIFT)

#define EMC_CFG				0xc
#define EMC_ADR_CFG			0x10
#define EMC_REFRESH			0x70
#define EMC_NOP				0xdc
#define EMC_SELF_REF			0xe0
#define EMC_REQ_CTRL			0x2b0
#define EMC_EMC_STATUS			0x2b4

#define PMC_CTRL			0x0
#define PMC_CTRL_BFI_SHIFT		8
#define PMC_CTRL_BFI_WIDTH		9
#define PMC_SCRATCH1			0x54
#define PMC_SCRATCH38			0x134

#define CLK_RESET_CCLK_BURST		0x20
#define CLK_RESET_CCLK_DIVIDER		0x24
#define CLK_RESET_SCLK_BURST		0x28
#define CLK_RESET_SCLK_DIVIDER		0x2c

#define CLK_RESET_PLLC_BASE		0x80
#define CLK_RESET_PLLM_BASE		0x90
#define CLK_RESET_PLLP_BASE		0xa0

#define FLOW_CTRL_HALT_CPU_EVENTS	0x0

/*
 * spooled CPU context is 512B / CPU
 */
#define CTX_SP		0
#define CTX_CPSR	4
#define CTX_SPSR	8
#define CTX_CPACR	12
#define CTX_CSSELR	16
#define CTX_SCTLR	20
#define CTX_ACTLR	24
#define CTX_PCTLR	28

#define CTX_FPEXC	32
#define CTX_FPSCR	36

#define CTX_TTBR0	48
#define CTX_TTBR1	52
#define CTX_TTBCR	56
#define CTX_DACR	60
#define CTX_PAR		64
#define CTX_PRRR	68
#define CTX_NMRR	72
#define CTX_VBAR	76
#define CTX_CONTEXTIDR	80
#define CTX_TPIDRURW	84
#define CTX_TPIDRURO	88
#define CTX_TPIDRPRW	92

#define CTX_SVC_SP	0
#define CTX_SVC_LR	-1	@ stored on stack
#define CTX_SVC_SPSR	8

#define CTX_SYS_SP	96
#define CTX_SYS_LR	100

#define CTX_ABT_SPSR	112
#define CTX_ABT_SP	116
#define CTX_ABT_LR	120

#define CTX_UND_SPSR	128
#define CTX_UND_SP	132
#define CTX_UND_LR	136

#define CTX_IRQ_SPSR	144
#define CTX_IRQ_SP	148
#define CTX_IRQ_LR	152

#define CTX_FIQ_SPSR	160
#define CTX_FIQ_R8	164
#define CTX_FIQ_R9	168
#define CTX_FIQ_R10	172
#define CTX_FIQ_R11	178
#define CTX_FIQ_R12	180
#define CTX_FIQ_SP	184
#define CTX_FIQ_LR	188

/* context only relevant for master cpu */
#ifdef CONFIG_CACHE_L2X0
#define CTX_L2_CTRL	224
#define CTX_L2_AUX	228
#define CTX_L2_TAG_CTRL	232
#define CTX_L2_DAT_CTRL	236
#define CTX_L2_PREFETCH 240
#endif

#define CTX_VFP_REGS	256
#define CTX_VFP_SIZE	(32 * 8)

#include "power-macros.S"

.macro	ctx_ptr, rd, tmp
	cpu_id	\tmp
	mov32	\rd, tegra_context_area
	ldr	\rd, [\rd]
	add	\rd, \rd, \tmp, lsl #(CONTEXT_SIZE_WORDS_SHIFT+2)
.endm

.macro	translate, pa, va, tmp
	mov	\tmp, #0x1000
	sub	\tmp, \tmp, #1
	bic	\pa, \va, \tmp
	mcr	p15, 0, \pa, c7, c8, 1
	mrc	p15, 0, \pa, c7, c4, 0
	bic	\pa, \pa, \tmp
	and	\tmp, \va, \tmp
	orr	\pa, \pa, \tmp
.endm

.macro emc_device_mask, rd, base
	ldr	\rd, [\base, #EMC_ADR_CFG]
	tst	\rd, #(0x3<<24)
	moveq	\rd, #(0x1<<8)		@ just 1 device
	movne	\rd, #(0x3<<8)		@ 2 devices
.endm


/*
 *	__cortex_a9_save(unsigned int mode)
 *
 *	 spools out the volatile processor state to memory, so that
 *	 the CPU may be safely powered down. does not preserve:
 *	 - CP15 c0 registers (except cache size select 2,c0/c0,0)
 *	 - CP15 c1 secure registers (c1/c1, 0-3)
 *	 - CP15 c5 fault status registers (c5/c0 0&1, c5/c1 0&1)
 *	 - CP15 c6 fault address registers (c6/c0 0&2)
 *	 - CP15 c9 performance monitor registers (c9/c12 0-5,
 *	     c9/c13 0-2, c9/c14 0-2)
 *	 - CP15 c10 TLB lockdown register (c10/c0, 0)
 *	 - CP15 c12 MVBAR (c12/c0, 1)
 *	 - CP15 c15 TLB lockdown registers
 */
	.align L1_CACHE_SHIFT
ENTRY(__cortex_a9_save)
	mrs	r3, cpsr
	cps	0x13			@ save off svc registers
	mov	r1, sp
	stmfd	sp!, {r3-r12, lr}

	bic	r2, sp, #(L1_CACHE_BYTES-1)

1:	mcr	p15, 0, r2, c7, c14, 1	@ clean out dirty stack cachelines
	add	r2, r2, #L1_CACHE_BYTES
	cmp	r2, r1
	ble	1b
	dsb

	ctx_ptr	r8, r9
	mov	r12, r0

	/* zero-out context area */
	mov	r9, r8
	add	r10, r8, #(CONTEXT_SIZE_WORDS*4)
	mov	r0, #0
	mov	r1, #0
	mov	r2, #0
	mov	r3, #0
	mov	r4, #0
	mov	r5, #0
	mov	r6, #0
	mov	r7, #0
2:	stmia	r9!, {r0-r7}
	cmp	r9, r10
	blo	2b

	mov	r0, sp
	mov	sp, r12			@ sp holds the power mode
	mrs	r1, cpsr
	mrs	r2, spsr

	mrc	p15, 0, r3, c1, c0, 2	@ cpacr
	stmia	r8, {r0-r3}
	mrc	p15, 2, r0, c0, c0, 0	@ csselr
	mrc	p15, 0, r1, c1, c0, 0	@ sctlr
	mrc	p15, 0, r2, c1, c0, 1	@ actlr
	mrc	p15, 0, r4, c15, c0, 0	@ pctlr
	add	r9, r8, #CTX_CSSELR
	stmia	r9, {r0-r2, r4}

#ifdef CONFIG_VFPv3
	orr	r2, r3, #0xF00000
	mcr	p15, 0, r2, c1, c0, 2	@ enable access to FPU
	VFPFMRX	r2, FPEXC
	str	r2, [r8, #CTX_FPEXC]
	mov	r1, #0x40000000		@ enable access to FPU
	VFPFMXR	FPEXC, r1
	VFPFMRX	r1, FPSCR
	str	r1, [r8, #CTX_FPSCR]
	isb
	add	r9, r8, #CTX_VFP_REGS

	VFPFSTMIA r9, r12	@ save out (16 or 32)*8B of FPU registers
	VFPFMXR	FPEXC, r2
	mrc	p15, 0, r3, c1, c0, 2	@ restore original FPEXC/CPACR
#endif

	add	r9, r8, #CTX_TTBR0
	mrc	p15, 0, r0, c2, c0, 0	@ TTBR0
	mrc	p15, 0, r1, c2, c0, 1	@ TTBR1
	mrc	p15, 0, r2, c2, c0, 2	@ TTBCR
	mrc	p15, 0, r3, c3, c0, 0	@ domain access control reg
	mrc	p15, 0, r4, c7, c4, 0	@ PAR
	mrc	p15, 0, r5, c10, c2, 0	@ PRRR
	mrc	p15, 0, r6, c10, c2, 1	@ NMRR
	mrc	p15, 0, r7, c12, c0, 0	@ VBAR
	stmia	r9!, {r0-r7}
	mrc	p15, 0, r0, c13, c0, 1	@ CONTEXTIDR
	mrc	p15, 0, r1, c13, c0, 2	@ TPIDRURW
	mrc	p15, 0, r2, c13, c0, 3	@ TPIDRURO
	mrc	p15, 0, r3, c13, c0, 4	@ TPIDRPRW
	stmia	r9, {r0-r3}

	cps	0x1f			@ SYS mode
	add	r9, r8, #CTX_SYS_SP
	stmia	r9, {sp,lr}

	cps	0x17			@ Abort mode
	mrs	r12, spsr
	add	r9, r8, #CTX_ABT_SPSR
	stmia	r9, {r12,sp,lr}

	cps	0x12			@ IRQ mode
	mrs	r12, spsr
	add	r9, r8, #CTX_IRQ_SPSR
	stmia	r9, {r12,sp,lr}

	cps	0x1b			@ Undefined mode
	mrs	r12, spsr
	add	r9, r8, #CTX_UND_SPSR
	stmia	r9, {r12,sp,lr}

	mov	r0, r8
	add	r1, r8, #CTX_FIQ_SPSR
	cps	0x11			@ FIQ mode
	mrs	r7, spsr
	stmia	r1, {r7-r12,sp,lr}

	cps	0x13			@ back to SVC
	mov	r8, r0

#ifdef CONFIG_CACHE_L2X0
	cpu_id	r4
	cmp	r4, #0
	bne	__cortex_a9_save_clean_cache
	mov32	r4, (TEGRA_ARM_PL310_BASE-IO_CPU_PHYS+IO_CPU_VIRT)
	add	r9, r8, #CTX_L2_CTRL
	ldr	r0, [r4, #L2X0_CTRL]
	ldr	r1, [r4, #L2X0_AUX_CTRL]
	ldr	r2, [r4, #L2X0_TAG_LATENCY_CTRL]
	ldr	r3, [r4, #L2X0_DATA_LATENCY_CTRL]
	ldr	r4, [r4, #L2X0_PREFETCH_OFFSET]
	stmia	r9, {r0-r4}
#endif


__cortex_a9_save_clean_cache:
	mov	r10, r8
	add	r9, r10, #(CONTEXT_SIZE_WORDS*4)
	add	r9, r9, #(L1_CACHE_BYTES-1)
	bic	r10, r10, #(L1_CACHE_BYTES-1)
	bic	r9, r9, #(L1_CACHE_BYTES-1)

3:	mcr	p15, 0, r10, c7, c10, 1
	add	r10, r10, #L1_CACHE_BYTES
	cmp	r10, r9
	blo	3b
	dsb

	translate r10, r8, r1

	mov	r0, #0
	mcr	p15, 0, r0, c1, c0, 1	@ exit coherency
	isb
	cpu_id	r0
	mov32	r1, (TEGRA_ARM_PERIF_BASE-IO_CPU_PHYS+IO_CPU_VIRT+0xC)
	mov	r3, r0, lsl #2
	mov	r2, #0xf
	mov	r2, r2, lsl r3
	str	r2, [r1]		@ invalidate SCU tags for CPU

	cmp	r0, #0
	bne	__put_cpu_in_reset
	mov	r8, r10
	b	__tear_down_master
ENDPROC(__cortex_a9_save)

/*
 *	__cortex_a9_l2x0_restart(bool invalidate)
 *
 *	 Reconfigures the L2 cache following a power event.
 */
	.align L1_CACHE_SHIFT
ENTRY(__cortex_a9_l2x0_restart)
#ifdef CONFIG_CACHE_L2X0
	ctx_ptr	r8, r9
	mov32	r9, (TEGRA_ARM_PL310_BASE-IO_CPU_PHYS+IO_CPU_VIRT)
	add	r10, r8, #CTX_L2_CTRL
	ldmia	r10, {r3-r7}
	str	r5, [r9, #L2X0_TAG_LATENCY_CTRL]
	str	r6, [r9, #L2X0_DATA_LATENCY_CTRL]
	str	r7, [r9, #L2X0_PREFETCH_OFFSET]
	str	r4, [r9, #L2X0_AUX_CTRL]
	cmp	r0, #0

	beq	__reenable_l2x0

	mov	r0, #0xff
	str	r0, [r9, #L2X0_INV_WAY]
1:	ldr	r1, [r9, #L2X0_INV_WAY]
	tst	r1, r0
	bne	1b
	mov	r0, #0
	str	r0, [r9, #L2X0_CACHE_SYNC]
__reenable_l2x0:
	mov	r5, #0
	mcr	p15, 0, r5, c8, c3, 0	@ invalidate TLB
	mcr	p15, 0, r5, c7, c5, 6	@ flush BTAC
	mcr	p15, 0, r5, c7, c5, 0	@ flush instruction cache
	dsb
	isb
	str	r3, [r9, #L2X0_CTRL]
#endif
	b	__cortex_a9_restore

/*
 *	__cortex_a9_restore
 *
 *	 reloads the volatile CPU state from the context area
 *	 the MMU should already be enabled using the secondary_data
 *	 page tables for cpu_up before this function is called, and the
 *	 CPU should be coherent with the SMP complex
 */
	.align L1_CACHE_SHIFT
ENTRY(__cortex_a9_restore)
	cps	0x13
	ctx_ptr	r0, r9

	cps	0x11			@ FIQ mode
	add	r1, r0, #CTX_FIQ_SPSR
	ldmia	r1, {r7-r12,sp,lr}
	msr	spsr_fsxc, r7

	cps	0x12			@ IRQ mode
	add	r1, r0, #CTX_IRQ_SPSR
	ldmia	r1, {r12, sp, lr}
	msr	spsr_fsxc, r12

	cps	0x17			@ abort mode
	add	r1, r0, #CTX_ABT_SPSR
	ldmia	r1, {r12, sp, lr}
	msr	spsr_fsxc, r12

	cps	0x1f			@ SYS mode
	add	r1, r0, #CTX_SYS_SP
	ldmia	r1, {sp, lr}

	cps	0x1b			@ Undefined mode
	add	r1, r0, #CTX_UND_SPSR
	ldmia	r1, {r12, sp, lr}
	msr	spsr_fsxc, r12

	cps	0x13			@ back to SVC
	mov	r8, r0

	add	r9, r8, #CTX_CSSELR
	ldmia	r9, {r0-r3}

	mcr	p15, 2, r0, c0, c0, 0	@ csselr
	mcr	p15, 0, r1, c1, c0, 0	@ sctlr
	mcr	p15, 0, r2, c1, c0, 1	@ actlr
	mcr	p15, 0, r3, c15, c0, 0	@ pctlr

	add	r9, r8, #CTX_TTBR0
	ldmia	r9!, {r0-r7}

	mcr	p15, 0, r4, c7, c4, 0	@ PAR
	mcr	p15, 0, r7, c12, c0, 0	@ VBAR
	mcr	p15, 0, r3, c3, c0, 0	@ domain access control reg
	isb
	mcr	p15, 0, r2, c2, c0, 2	@ TTBCR
	isb
	mcr	p15, 0, r5, c10, c2, 0	@ PRRR
	isb
	mcr	p15, 0, r6, c10, c2, 1	@ NMRR
	isb

	ldmia	r9, {r4-r7}

	mcr	p15, 0, r5, c13, c0, 2	@ TPIDRURW
	mcr	p15, 0, r6, c13, c0, 3	@ TPIDRURO
	mcr	p15, 0, r7, c13, c0, 4	@ TPIDRPRW

	ldmia	r8, {r5-r7, lr}

	/* perform context switch to previous context */
	mov	r9, #0
	mcr	p15, 0, r9, c13, c0, 1	@ set reserved context
	isb
	mcr	p15, 0, r0, c2, c0, 0	@ TTBR0
	isb
	mcr	p15, 0, r4, c13, c0, 1	@ CONTEXTIDR
	isb
	mcr	p15, 0, r1, c2, c0, 1	@ TTBR1
	isb

	mov	r4, #0
	mcr	p15, 0, r4, c8, c3, 0	@ invalidate TLB
	mcr	p15, 0, r4, c7, c5, 6	@ flush BTAC
	mcr	p15, 0, r4, c7, c5, 0	@ flush instruction cache
	dsb
	isb

	mov	sp, r5
	msr	cpsr_cxsf, r6
	msr	spsr_cxsf, r7

#ifdef CONFIG_VFPv3
	orr	r4, lr, #0xF00000
	mcr	p15, 0, r4, c1, c0, 2	@ enable coproc access
	mov	r5, #0x40000000
	VFPFMXR	FPEXC, r5		@ enable FPU access
	add	r9, r8, #CTX_VFP_REGS
	add	r7, r8, #CTX_FPEXC
	VFPFLDMIA r9, r10
	ldmia	r7, {r0, r4}
	VFPFMXR	FPSCR, r4
	VFPFMXR	FPEXC, r0
#endif
	mcr	p15, 0, lr, c1, c0, 2	@ cpacr (loaded before VFP)

	/* finally, restore the stack and return */
	ldmfd	sp!, {r3-r12, lr}
	msr	cpsr_fsxc, r3		@ restore original processor mode
	mov	pc, lr
ENDPROC(__cortex_a9_restore)


/*
 *
 *	__tear_down_master( r8 = context_pa, sp = power state )
 *
 *	  Set the clock burst policy to the selected wakeup source
 *	  Enable CPU power-request mode in the PMC
 *	  Put the CPU in wait-for-event mode on the flow controller
 *	  Trigger the PMC state machine to put the CPU in reset
 */
__tear_down_master:
#ifdef CONFIG_CACHE_L2X0
	/* clean out the dirtied L2 lines, since all power transitions
	 * cause the cache state to get invalidated (although LP1 & LP2
	 * preserve the data in the L2, the control words (L2X0_CTRL,
	 * L2X0_AUX_CTRL, etc.) need to be cleaned to L3 so that they
	 * will be visible on reboot.  skip this for LP0, since the L2 cache
	 * will be shutdown before we reach this point */
	tst	sp, #TEGRA_POWER_EFFECT_LP0
	bne	__l2_clean_done
	mov32	r0, (TEGRA_ARM_PL310_BASE-IO_CPU_PHYS+IO_CPU_VIRT)
	add	r3, r8, #(CONTEXT_SIZE_WORDS*4)
	bic	r8, r8, #0x1f
	add	r3, r3, #0x1f
11:	str	r8, [r0, #L2X0_CLEAN_LINE_PA]
	add	r8, r8, #32
	cmp	r8, r3
	blo	11b
12:	ldr	r1, [r0, #L2X0_CLEAN_LINE_PA]
	tst	r1, #1
	bne	12b
	mov	r1, #0
	str	r1, [r0, #L2X0_CACHE_SYNC]
13:	ldr	r1, [r0, #L2X0_CACHE_SYNC]
	tst	r1, #1
	bne	13b
__l2_clean_done:
#endif

	tst	sp, #TEGRA_POWER_SDRAM_SELFREFRESH

	/* preload all the address literals that are needed for the
	 * CPU power-gating process, to avoid loads from SDRAM (which are
	 * not supported once SDRAM is put into self-refresh.
	 * LP0 / LP1 use physical address, since the MMU needs to be
	 * disabled before putting SDRAM into self-refresh to avoid
	 * memory access due to page table walks */
	mov32	r0, (IO_APB_VIRT-IO_APB_PHYS)
	mov32	r4, TEGRA_PMC_BASE
	addeq	r4, r4, r0
	mov32	r0, (IO_PPSB_VIRT-IO_PPSB_PHYS)
	mov32	r5, TEGRA_CLK_RESET_BASE
	mov32	r6, TEGRA_FLOW_CTRL_BASE
	mov32	r7, TEGRA_TMRUS_BASE
	addeq	r5, r5, r0
	addeq	r6, r6, r0
	addeq	r7, r7, r0

	beq	__tear_down_master_pll_cpu

	/* change page table pointer to tegra_pgd_phys, so that IRAM
	 * will be mapped virtual == physical */
	adr	r3, __tear_down_master_data
	ldr	r3, [r3]		@ &tegra_pgd_phys
	ldr	r3, [r3]
	orr	r3, r3, #TTB_FLAGS
	mov	r2, #0
	mcr	p15, 0, r2, c13, c0, 1	@ reserved context
	isb
	mcr	p15, 0, r3, c2, c0, 0	@ TTB 0
	isb
	mov32	r2, __tegra_lp1_reset
	mov32	r3, __tear_down_master_sdram
	sub	r2, r3, r2
	mov32	r3, (TEGRA_IRAM_CODE_AREA)
	add	r3, r2, r3
	movne	pc, r3
ENDPROC(__tear_down_master)
	.type	__tear_down_master_data, %object
__tear_down_master_data:
	.long	tegra_pgd_phys
	.size	__tear_down_master_data, . - __tear_down_master_data

/*  START OF ROUTINES COPIED TO IRAM  */
/*
 *	__tegra_lp1_reset
 *
 *	  reset vector for LP1 restore; copied into IRAM during suspend.
 *	  brings the system back up to a safe starting point (SDRAM out of
 *	  self-refresh, PLLC, PLLM and PLLP reenabled, CPU running on PLLP,
 *	  system clock running on the same PLL that it suspended at), and
 *	  jumps to tegra_lp2_startup to restore PLLX and virtual addressing.
 *	  physical address of tegra_lp2_startup expected to be stored in
 *	  PMC_SCRATCH1
 */
	.align L1_CACHE_SHIFT
ENTRY(__tegra_lp1_reset)
__tegra_lp1_reset:
	/* the CPU and system bus are running at 32KHz and executing from
	 * IRAM when this code is executed; immediately switch to CLKM and
	 * enable PLLP. */
	mov32	r0, TEGRA_CLK_RESET_BASE
	mov	r1, #(1<<28)
	str	r1, [r0, #CLK_RESET_SCLK_BURST]
	str	r1, [r0, #CLK_RESET_CCLK_BURST]
	mov	r1, #0
	str	r1, [r0, #CLK_RESET_SCLK_DIVIDER]
	str	r1, [r0, #CLK_RESET_CCLK_DIVIDER]

	ldr	r1, [r0, #CLK_RESET_PLLM_BASE]
	tst	r1, #(1<<30)
	orreq	r1, r1, #(1<<30)
	streq	r1, [r0, #CLK_RESET_PLLM_BASE]
	ldr	r1, [r0, #CLK_RESET_PLLP_BASE]
	tst	r1, #(1<<30)
	orreq	r1, r1, #(1<<30)
	streq	r1, [r0, #CLK_RESET_PLLP_BASE]
	ldr	r1, [r0, #CLK_RESET_PLLC_BASE]
	tst	r1, #(1<<30)
	orreq	r1, r1, #(1<<30)
	streq	r1, [r0, #CLK_RESET_PLLC_BASE]
	mov32	r7, TEGRA_TMRUS_BASE
	ldr	r1, [r7]

	/* since the optimized settings are still in SDRAM, there is
	 * no need to store them back into the IRAM-local __lp1_pad_area */
	add	r2, pc, #__lp1_pad_area-(.+8)
padload:ldmia	r2!, {r3-r4}
	cmp	r3, #0
	beq	padload_done
	str	r4, [r3]
	b	padload
padload_done:
	ldr	r2, [r7]
	add	r2, r2, #0x4	@ 4uS delay for DRAM pad restoration
	wait_until r2, r7, r3
	add	r1, r1, #0xff	@ 255uS delay for PLL stabilization
	wait_until r1, r7, r3

	str	r4, [r0, #CLK_RESET_SCLK_BURST]
	mov32	r4, ((1<<28) | (4))	@ burst policy is PLLP
	str	r4, [r0, #CLK_RESET_CCLK_BURST]

	mov32	r0, TEGRA_EMC_BASE
	ldr	r1, [r0, #EMC_CFG]
	bic	r1, r1, #(1<<31)	@ disable DRAM_CLK_STOP
	str	r1, [r0, #EMC_CFG]

	mov	r1, #0
	str	r1, [r0, #EMC_SELF_REF]	@ take DRAM out of self refresh
	mov	r1, #1
	str	r1, [r0, #EMC_NOP]
	str	r1, [r0, #EMC_NOP]
	str	r1, [r0, #EMC_REFRESH]

	emc_device_mask r1, r0

exit_selfrefresh_loop:
	ldr	r2, [r0, #EMC_EMC_STATUS]
	ands	r2, r2, r1
	bne	exit_selfrefresh_loop

	mov	r1, #0
	str	r1, [r0, #EMC_REQ_CTRL]

	mov32	r0, TEGRA_PMC_BASE
	ldr	r0, [r0, #PMC_SCRATCH1]
	mov	pc, r0
ENDPROC(__tegra_lp1_reset)

/*
 *	__tear_down_master_sdram
 *
 *	  disables MMU, data cache, and puts SDRAM into self-refresh.
 *	  must execute from IRAM.
 */
	.align L1_CACHE_SHIFT
__tear_down_master_sdram:
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, r3, #((1<<0) | (1<<2))	@ disable MMU and data-cache
	mcr	p15, 0, r3, c1, c0, 0		@ to avoid page faults & races
	dsb
	isb

	mov32	r1, TEGRA_EMC_BASE
	mov	r2, #3
	str	r2, [r1, #EMC_REQ_CTRL]		@ stall incoming DRAM requests

emcidle:ldr	r2, [r1, #EMC_EMC_STATUS]
	tst	r2, #4
	beq	emcidle

	mov	r2, #1
	str	r2, [r1, #EMC_SELF_REF]

	emc_device_mask r2, r1

emcself:ldr	r3, [r1, #EMC_EMC_STATUS]
	and	r3, r3, r2
	cmp	r3, r2
	bne	emcself				@ loop until DDR in self-refresh

	add	r2, pc, #__lp1_pad_area-(.+8)

padsave:ldm	r2, {r0-r1}
	cmp	r0, #0
	beq	padsave_done
	ldr	r3, [r0]
	str	r1, [r0]
	str	r3, [r2, #4]
	add	r2, r2, #8
	b	padsave
padsave_done:

	ldr	r0, [r5, #CLK_RESET_SCLK_BURST]
	str	r0, [r2, #4]
	dsb
	b	__tear_down_master_pll_cpu
ENDPROC(__tear_down_master_sdram)

	.align	L1_CACHE_SHIFT
	.type	__lp1_pad_area, %object
__lp1_pad_area:
	.word	TEGRA_APB_MISC_BASE + 0x8c8 /* XM2CFGCPADCTRL */
	.word	0x8
	.word	TEGRA_APB_MISC_BASE + 0x8cc /* XM2CFGDPADCTRL */
	.word	0x8
	.word	TEGRA_APB_MISC_BASE + 0x8d0 /* XM2CLKCFGPADCTRL */
	.word	0x0
	.word	TEGRA_APB_MISC_BASE + 0x8d4 /* XM2COMPPADCTRL */
	.word	0x8
	.word	TEGRA_APB_MISC_BASE + 0x8d8 /* XM2VTTGENPADCTRL */
	.word	0x5500
	.word	TEGRA_APB_MISC_BASE + 0x8e4 /* XM2CFGCPADCTRL2 */
	.word	0x08080040
	.word	TEGRA_APB_MISC_BASE + 0x8e8 /* XM2CFGDPADCTRL2 */
	.word	0x0
	.word	0x0	/* end of list */
	.word	0x0	/* sclk_burst_policy */
	.size	__lp1_pad_area, . - __lp1_pad_area

	.align L1_CACHE_SHIFT
__tear_down_master_pll_cpu:
	ldr	r0, [r4, #PMC_CTRL]
	bfi	r0, sp, #PMC_CTRL_BFI_SHIFT, #PMC_CTRL_BFI_WIDTH
	str	r0, [r4, #PMC_CTRL]
	tst	sp, #TEGRA_POWER_SDRAM_SELFREFRESH

	/* in LP2 idle (SDRAM active), set the CPU burst policy to PLLP */
	moveq	r0, #(2<<28)    /* burst policy = run mode */
	orreq	r0, r0, #(4<<4) /* use PLLP in run mode burst */
	streq	r0, [r5, #CLK_RESET_CCLK_BURST]
	moveq	r0, #0
	streq	r0, [r5, #CLK_RESET_CCLK_DIVIDER]
	beq	__cclk_burst_set

	/* in other modes, set system & CPU burst policies to 32KHz.
	 * start by jumping to CLKM to safely disable PLLs, then jump
	 * to CLKS */
	mov	r0, #(1<<28)
	str	r0, [r5, #CLK_RESET_SCLK_BURST]
	str	r0, [r5, #CLK_RESET_CCLK_BURST]
	mov	r0, #0
	str	r0, [r5, #CLK_RESET_CCLK_DIVIDER]
	str	r0, [r5, #CLK_RESET_SCLK_DIVIDER]

	/* 2 us delay between changing sclk and disabling PLLs */
	wait_for_us r1, r7, r9
	add	r1, r1, #2
	wait_until r1, r7, r9

	/* switch to CLKS */
	mov	r0, #0	/* burst policy = 32KHz */
	str	r0, [r5, #CLK_RESET_SCLK_BURST]

	/* disable PLLP, PLLM, PLLC in LP0 and LP1 states */
	ldr	r0, [r5, #CLK_RESET_PLLM_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLM_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLP_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLP_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLC_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLC_BASE]

__cclk_burst_set:
	mov	r0, #(4<<29)			/* STOP_UNTIL_IRQ */
	orr	r0, r0, #(1<<10) | (1<<8)	/* IRQ_0, FIQ_0	*/
	ldr	r1, [r7]
	str	r1, [r4, #PMC_SCRATCH38]
	dsb
	str	r0, [r6, #FLOW_CTRL_HALT_CPU_EVENTS]
	dsb
	ldr	r0, [r6, #FLOW_CTRL_HALT_CPU_EVENTS] /* memory barrier */

halted:	dsb
	wfe	/* CPU should be power gated here */
	isb
	b	halted
ENDPROC(__tear_down_master_pll_cpu)

/*
 *	__put_cpu_in_reset(cpu_nr)
 *
 *	 puts the specified CPU in wait-for-event mode on the flow controller
 *	 and puts the CPU in reset
 */
__put_cpu_in_reset:
	cmp	r0, #0
	subne	r1, r0, #1
	movne	r1, r1, lsl #3
	addne	r1, r1, #0x14
	moveq	r1, #0			@ r1 = CPUx_HALT_EVENTS register offset
	mov32	r7, (TEGRA_FLOW_CTRL_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	mov	r2, #(0x2<<29)
	str	r2, [r7, r1]		@ put flow controller in wait event mode
	isb
	dsb
	movw	r1, 0x1011
	mov	r1, r1, lsl r0
	mov32	r7, (TEGRA_CLK_RESET_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	str	r1, [r7, #0x340]	@ put slave CPU in reset
	isb
	dsb
	b	.
ENDPROC(__put_cpu_in_reset)

/* dummy symbol for end of IRAM */
	.align L1_CACHE_SHIFT
ENTRY(__tegra_iram_end)
__tegra_iram_end:
	b	.
ENDPROC(__tegra_iram_end)
