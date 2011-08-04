/*
 * arch/arm/mach-tegra/include/mach/sleep-t3.S
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#define TEGRA_ARM_PERIF_VIRT	(TEGRA_ARM_PERIF_BASE - IO_CPU_PHYS + IO_CPU_VIRT)

#ifdef CONFIG_HOTPLUG_CPU
/*
 * tegra3_hotplug_shutdown(void)
 *
 * Powergates the current CPU.
 * Should never return.
 */
ENTRY(tegra3_hotplug_shutdown)
	mov	r6, lr
	bl	tegra_cpu_exit_coherency

	/* Powergate this CPU. */
	mov	r0, #TEGRA_POWER_HOTPLUG_SHUTDOWN
	bl	tegra3_cpu_reset
	mov	pc, r6				@ should never get here
ENDPROC(tegra3_hotplug_shutdown)
#endif

#if defined(CONFIG_HOTPLUG_CPU) || defined(CONFIG_PM_SLEEP)
/*
 * tegra3_cpu_reset(unsigned long flags)
 *
 * Puts the current CPU in wait-for-event mode on the flow controller
 * and powergates it -- flags (in R0) indicate the request type.
 * Must never be called for CPU 0.
 *
 * corrupts r0-r4, r12
 */
ENTRY(tegra3_cpu_reset)
	cpu_id	r3
	cmp	r3, #0
	moveq	pc, lr		@ Must never be called for CPU 0

	mov32	r12, TEGRA_FLOW_CTRL_VIRT
	cpu_to_csr_reg r1, r3
	add	r1, r1, r12	@ virtual CSR address for this CPU
	cpu_to_halt_reg r2, r3
	add	r2, r2, r12	@ virtual HALT_EVENTS address for this CPU

	/* Clear this CPU's "event" and "interrupt" flags and power gate
	   it when halting but not before it is in the "WFE" state. */
	movw	r12, FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG | FLOW_CTRL_CSR_ENABLE
	mov	r4, #(1 << 4)
	orr	r12, r12, r4, lsl r3
	str	r12, [r1]

	/* Halt this CPU. */
	mov	r3, #0x400
delay_1:
	subs	r3, r3, #1				@ delay as a part of wfe war.
	bge	delay_1;
	cpsid	a					@ disable imprecise aborts.
	ldr	r3, [r1]				@ read CSR
	str	r3, [r1]				@ clear CSR
	tst	r0, #TEGRA_POWER_HOTPLUG_SHUTDOWN
	moveq	r3, #FLOW_CTRL_WAIT_FOR_INTERRUPT	@ For LP2
	movne	r3, #FLOW_CTRL_WAITEVENT		@ For hotplug
	str	r3, [r2]
	ldr	r0, [r2]
	b	wfe_war

__cpu_reset_again:
	dsb
	.align 5
	wfe						@ CPU should be power gated here
wfe_war:
	b	__cpu_reset_again

	/* 38 nop's, which fills reset of wfe cache line and 4 more cachelines with nop*/
	.rept 38
	nop
	.endr
	b 	.					@ should never get here

ENDPROC(tegra3_cpu_reset)
#endif

#ifdef CONFIG_PM_SLEEP
/*
 * tegra3_sleep_cpu_secondary(unsigned long v2p)
 *
 * Enters LP2 on secondary CPU by exiting coherency and powergating the CPU.
 */
ENTRY(tegra3_sleep_cpu_secondary)
	mov	r3, lr				@ set resume address to lr
	bl	tegra_cpu_save

	/* Powergate this CPU. */
	mov	r0, #0				@ power mode flags (!hotplug)
	bl	tegra3_cpu_reset
	b	.				@ should never get here
ENDPROC(tegra3_sleep_cpu_secondary)

/*
 * tegra3_tear_down_cpu
 *
 * Switches the CPU cluster to PLL-P and enters sleep.
 */
ENTRY(tegra3_tear_down_cpu)
	bl	tegra_cpu_pllp
	b	tegra3_enter_sleep
ENDPROC(tegra3_tear_down_cpu)

/* START OF ROUTINES COPIED TO IRAM */
	.align L1_CACHE_SHIFT
	.globl tegra3_iram_start
tegra3_iram_start:

/*
 * tegra3_lp1_reset
 *
 * reset vector for LP1 restore; copied into IRAM during suspend.
 * brings the system back up to a safe starting point (SDRAM out of
 * self-refresh, PLLC, PLLM and PLLP reenabled, CPU running on PLLP,
 * system clock running on the same PLL that it suspended at), and
 * jumps to tegra_lp2_startup to restore PLLX and virtual addressing.
 * physical address of tegra_lp2_startup expected to be stored in
 * PMC_SCRATCH41
 *
 * NOTE: THIS *MUST* BE RELOCATED TO TEGRA_IRAM_CODE_AREA AND MUST BE FIRST.
 */

/* !!!FIXME!!! Add LP1/LP1 code */

/*
 * tegra3_enter_sleep
 *
 * uses flow controller to enter sleep state
 * executes from IRAM with SDRAM in selfrefresh when target state is LP0 or LP1
 * executes from SDRAM with target state is LP2
 */
tegra3_enter_sleep:
	mov32	r7, TEGRA_TMRUS_BASE
	ldr	r1, [r7]
	mov32	r4, TEGRA_PMC_BASE
	str	r1, [r4, #PMC_SCRATCH38]
	dsb
	mov32	r6, TEGRA_FLOW_CTRL_BASE
	cpu_id	r1

	cpu_to_csr_reg	r2, r1
	ldr	r0, [r6, r2]
	orr	r0, r0, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	str	r0, [r6, r2]

	mov	r0, #FLOW_CTRL_WAIT_FOR_INTERRUPT
	orr	r0, r0, #FLOW_CTRL_HALT_CPU_IRQ | FLOW_CTRL_HALT_CPU_FIQ
	cpu_to_halt_reg r2, r1
	str	r0, [r6, r2]
	dsb
	ldr	r0, [r6, r2] /* memory barrier */

halted:
	dsb
	isb
	wfi	/* CPU should be power gated here */

	/* !!!FIXME!!! Implement halt failure handler */
	b	halted

	.ltorg
/* dummy symbol for end of IRAM */
	.align L1_CACHE_SHIFT
	.globl tegra3_iram_end
tegra3_iram_end:
	b	.
#endif
