/*
 * arch/arm/mach-tegra/tegra3_save.S
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
#include <asm/memory.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "clock.h"
#include "power.h"

/*	.section ".cpuinit.text", "ax"*/

#define TTB_FLAGS 0x6A	@ IRGN_WBWA, OC_RGN_WBWA, S, NOS

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
#define PMC_SCRATCH41			0x140

#define CLK_RESET_CCLK_BURST		0x20
#define CLK_RESET_CCLK_DIVIDER		0x24
#define CLK_RESET_SCLK_BURST		0x28
#define CLK_RESET_SCLK_DIVIDER		0x2c

#define CLK_RESET_PLLC_BASE		0x80
#define CLK_RESET_PLLM_BASE		0x90
#define CLK_RESET_PLLP_BASE		0xa0
#define CLK_RESET_PLLX_BASE		0xe0

#define CLK_RESET_PLLC_MISC		0x8c
#define CLK_RESET_PLLM_MISC		0x9c
#define CLK_RESET_PLLP_MISC		0xac
#define CLK_RESET_PLLX_MISC		0xe4

#define FLOW_CTRL_HALT_CPU_EVENTS	0x0

#include "power-macros.S"

#define DEBUG_SLAVE_HALT_FAIL	1	/* enable slave halt failure debug code */
#define DEBUG_FORCE_RTC_WAKEUP_SEC 0	/* nonzero: force wake up via RTC */

#if USE_PLL_LOCK_BITS
.macro pll_locked, rd, car, base
1:
	ldr	\rd, [\car, #\base]
	tst	\rd, #(1<<27)
	beq	1b
.endm
#endif

.macro pll_enable, rd, car, base, misc
	ldr	\rd, [\car, #\base]
	tst	\rd, #(1<<30)
	orreq	\rd, \rd, #(1<<30)
	streq	\rd, [\car, #\base]
#if USE_PLL_LOCK_BITS
	ldr	\rd, [\car, #\misc]
	orr	\rd, \rd, #(1<<18)
	str	\rd, [\car, #\misc]
#endif
.endm

.macro emc_device_mask, rd, base
	ldr	\rd, [\base, #EMC_ADR_CFG]
	tst	\rd, #(0x3<<24)
	moveq	\rd, #(0x1<<8)		@ just 1 device
	movne	\rd, #(0x3<<8)		@ 2 devices
.endm

/*
 *
 *	__tear_down_master( r8 = context_pa, sp = power state )
 *
 *	  Set the clock burst policy to the selected wakeup source
 *	  Enable CPU power-request mode in the PMC
 *	  Put the CPU in wait-for-event mode on the flow controller
 *	  Trigger the PMC state machine to put the CPU in reset
 */
ENTRY(__tear_down_master)
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
	add	r3, r8, #(CONTEXT_SIZE_BYTES)
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

	/* preload all the address literals that are needed for the
	 * CPU power-gating process, to avoid loads from SDRAM (which are
	 * not supported once SDRAM is put into self-refresh.
	 * LP0 / LP1 use physical address, since the MMU needs to be
	 * disabled before putting SDRAM into self-refresh to avoid
	 * memory access due to page table walks */
	mov32	r4, TEGRA_PMC_BASE
	mov32	r5, TEGRA_CLK_RESET_BASE
	mov32	r6, TEGRA_FLOW_CTRL_BASE
	mov32	r7, TEGRA_TMRUS_BASE

	/* change page table pointer to tegra_pgd_phys, so that IRAM
	 * and MMU shut-off will be mapped virtual == physical */
	adr	r3, __tear_down_master_data
	ldr	r3, [r3]		@ &tegra_pgd_phys
	ldr	r3, [r3]
	orr	r3, r3, #TTB_FLAGS
	mov	r2, #0
	mcr	p15, 0, r2, c13, c0, 1	@ reserved context
	isb
	mcr	p15, 0, r3, c2, c0, 0	@ TTB 0
	isb

	/* Obtain LP1 information.
	 * R10 = LP1 branch target */
	mov32	r2, __tegra_lp1_reset
	mov32	r3, __tear_down_master_sdram
	sub	r2, r3, r2
	mov32	r3, (TEGRA_IRAM_CODE_AREA)
	add	r10, r2, r3

	mov32	r3, __shut_off_mmu

	/* R9 = LP2 branch target */
	mov32	r9, __tear_down_master_pll_cpu

	/* Convert the branch targets
	 * to physical addresses */
	sub	r3, r3, #(PAGE_OFFSET - PHYS_OFFSET)
	sub	r9, r9, #(PAGE_OFFSET - PHYS_OFFSET)
	tst	sp, #TEGRA_POWER_SDRAM_SELFREFRESH
	movne	r9, r10
	bx	r3
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
	/* the CPU and system bus are running at 32KHz and executing from
	 * IRAM when this code is executed; immediately switch to CLKM and
	 * enable PLLP, PLLM, PLLC, and PLLX. */
	mov32	r0, TEGRA_CLK_RESET_BASE
	mov	r1, #(1<<28)
	str	r1, [r0, #CLK_RESET_SCLK_BURST]
	str	r1, [r0, #CLK_RESET_CCLK_BURST]
	mov	r1, #0
	str	r1, [r0, #CLK_RESET_SCLK_DIVIDER]
	str	r1, [r0, #CLK_RESET_CCLK_DIVIDER]

	pll_enable r1, r0, CLK_RESET_PLLM_BASE, CLK_RESET_PLLM_MISC
	pll_enable r1, r0, CLK_RESET_PLLP_BASE, CLK_RESET_PLLP_MISC
	pll_enable r1, r0, CLK_RESET_PLLC_BASE, CLK_RESET_PLLC_MISC
	pll_enable r1, r0, CLK_RESET_PLLX_BASE, CLK_RESET_PLLX_MISC
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
#if USE_PLL_LOCK_BITS
	pll_locked r1, r0, CLK_RESET_PLLM_BASE
	pll_locked r1, r0, CLK_RESET_PLLP_BASE
	pll_locked r1, r0, CLK_RESET_PLLC_BASE
	pll_locked r1, r0, CLK_RESET_PLLX_BASE
#else
	add	r1, r1, #0xff	@ 255uS delay for PLL stabilization
	wait_until r1, r7, r3
#endif

	str	r4, [r0, #CLK_RESET_SCLK_BURST]
	mov32	r4, ((1<<28) | (8))	@ burst policy is PLLX
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
	ldr	r0, [r0, #PMC_SCRATCH41]
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
	/* FIXME: Need pad control (address,value) list */
	.word	0x0	/* end of pad list */
	.word	0x0	/* sclk_burst_policy */
	.size	__lp1_pad_area, . - __lp1_pad_area

/*
 *	__tear_down_master_pll_cpu()
 *
 *	LP2 entry code:
 *
 *	Physical addressing is in effect:
 *
 *	r4 = TEGRA_PMC_BASE
 *	r5 = TEGRA_CLK_RESET_BASE
 *	r6 = TEGRA_FLOW_CTRL_BASE
 *	r7 = TEGRA_TMRUS_BASE
 *	sp = power mode flags
 */
	.align L1_CACHE_SHIFT
__tear_down_master_pll_cpu:
	ldr	r0, [r4, #PMC_CTRL]
	bfi	r0, sp, #PMC_CTRL_BFI_SHIFT, #PMC_CTRL_BFI_WIDTH
	str	r0, [r4, #PMC_CTRL]
	/* in LP2 idle (SDRAM active) leave CPU burst policy as it is */
	tst	sp, #TEGRA_POWER_SDRAM_SELFREFRESH
	beq	__do_halt

	/* In other modes, set system & CPU burst policies to 32KHz.
	 * Start by switching to CLKM to safely disable PLLs, then
	 * switch to CLKS. */
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

	/* disable PLLP, PLLM, PLLC, and PLLX in LP0 and LP1 states */
	ldr	r0, [r5, #CLK_RESET_PLLM_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLM_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLP_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLP_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLC_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLC_BASE]
	ldr	r0, [r5, #CLK_RESET_PLLX_BASE]
	bic	r0, r0, #(1<<30)
	str	r0, [r5, #CLK_RESET_PLLX_BASE]

#if DEBUG_FORCE_RTC_WAKEUP_SEC
	mov32	r0, TEGRA_RTC_BASE
        /* Clear all RTC interrupts */
        mov	r2, #0x1f
	str	r2, [r0, #0x2c]
        mov	r2, #0x0
        str	r2, [r0, #0x28]
	//setup rtc wake
	ldr	r2, [r0, #0x10]	/* milli */
	ldr	r2, [r0, #0x8]	/* shadow */

	add	r2, r2, #DEBUG_FORCE_RTC_WAKEUP_SEC
rtc_idle1:
	ldr	r1, [r0, #0x4]
	tst	r1, #0x1
	bne	rtc_idle1
	str	r2, [r0, #0x14]
rtc_idle2:
	ldr	r1, [r0, #0x4]
	tst	r1, #0x1
	bne	rtc_idle2
	/* intr mask alarm0 */
	mov	r2, #1
	str	r2, [r0, #0x28]
rtc_idle3:
	ldr	r1, [r0, #0x4]
	tst	r1, #0x1
	bne	rtc_idle3
#endif

__do_halt:
	mov	r0, #(4<<29)			/* STOP_UNTIL_IRQ */
	orr	r0, r0, #(1<<10) | (1<<8)	/* IRQ_0, FIQ_0	*/
	ldr	r1, [r7]
	str	r1, [r4, #PMC_SCRATCH38]
	dsb
	str	r0, [r6, #FLOW_CTRL_HALT_CPU_EVENTS]
	dsb
	ldr	r0, [r6, #FLOW_CTRL_HALT_CPU_EVENTS] /* memory barrier */

__halted:
	dsb
	wfi	/* CPU should be power gated here */
	isb
	b	__halted
ENDPROC(__tear_down_master_pll_cpu)

/*
 *	__put_cpu_in_reset(cpu_nr)
 *
 *	 puts the specified CPU in wait-for-event mode on the flow controller
 *	 and puts the CPU in reset
 */
ENTRY(__put_cpu_in_reset)
	subs	r1, r0, #1
	bmi	.			@ should never come here for CPU0
	mov	r1, r1, lsl #3
	add	r2, r1, #0x18
	add	r1, r1, #0x14
	mov32	r7, (TEGRA_FLOW_CTRL_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)

	/* Clear this CPU's "event" and "interrupt" flags and power gate
	   it when halting but not before it is in the "WFI" state. */
	mov32	r3, (1 << 15) | (1 << 14) | 1
	mov	r4, #(1 << 8)
	orr	r3, r3, r4, lsl r0
	str	r3, [r7, r2]

	/* Halt this CPU unconditionally */
	mov	r3, #(2<<29)
	str	r3, [r7, r1]
	dsb
	wfi

#if	DEBUG_SLAVE_HALT_FAIL
	b	.
#endif

        /* If the WFI doesn't take effect force the CPU into reset. */
	movw	r1, 0x1011
	mov	r1, r1, lsl r0
	mov32	r7, (TEGRA_CLK_RESET_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	str	r1, [r7, #0x340]
	isb
	dsb
	b	.			@ should never get here
ENDPROC(__put_cpu_in_reset)

/* dummy symbol for end of IRAM */
	.align L1_CACHE_SHIFT
ENTRY(__tegra_iram_end)
	b	.
ENDPROC(__tegra_iram_end)
