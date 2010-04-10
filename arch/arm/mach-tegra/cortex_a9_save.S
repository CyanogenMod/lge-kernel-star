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

#include <mach/iomap.h>
#include <mach/io.h>

	.section ".cpuinit.text", "ax"

#define CONTEXT_SIZE_WORDS_SHIFT 7
#define CONTEXT_SIZE_WORDS (1<<CONTEXT_SIZE_WORDS_SHIFT)

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

#define CTX_VFP_REGS	256
#define CTX_VFP_SIZE	(32 * 8)

.macro	cpu_id, rd
	mrc	p15, 0, \rd, c0, c0, 5
	and	\rd, \rd, #0xF
.endm

.macro	ctx_ptr, rd, tmp
	cpu_id	\tmp
	adr	\rd, __cortex_a9_context_area
	add	\rd, \rd, \tmp, lsl #(CONTEXT_SIZE_WORDS_SHIFT+2)
.endm

/*
 *	__cortex_a9_save
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
	mov	r0, sp
	bic	r0, r0, #((1<<L1_CACHE_SHIFT)-1)
	add	r0, r0, #(L1_CACHE_BYTES*2)
	stmfd	sp!, {r4-r12, lr}

1:	mcr	p15, 0, r0, c7, c10, 1	@ clean out dirty stack cachelines
	sub	r0, r0, #L1_CACHE_BYTES
	cmp	r0, sp
	bge	1b
	dsb

	ctx_ptr	r8, r9

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
	stmia	r9!, {r0-r3}
	
	mov	r9, #(CONTEXT_SIZE_WORDS*4)
3:	mcr	p15, 0, r8, c7, c10, 1	@ clean out data processor data area
	add	r8, r8, #L1_CACHE_BYTES
	subs	r9, r9, #L1_CACHE_BYTES
	bhi	3b
	dsb

	mov	r0, #0
	mcr	p15, 0, r0, c1, c0, 1	@ exit coherency
	isb
	cpu_id	r0
	ldr	r1, =(TEGRA_ARM_PERIF_BASE-IO_CPU_PHYS+IO_CPU_VIRT+0xC)
	mov	r3, r0, lsl #2
	mov	r2, #0xf
	mov	r2, r2, lsl r3
	str	r2, [r1]		@ invalidate SCU tags for CPU

	cmp	r0, #0
	bne	__put_cpu_in_reset
	b	__tear_down_master
ENDPROC(__cortex_a9_save)

	/* master tear-down not yet implemented */
__tear_down_master:
	b	.
ENDPROC(__tear_down_master)

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
	ldr	r7, =(TEGRA_FLOW_CTRL_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	mov	r2, #0x2
	str	r2, [r7, r1]		@ put flow controller in wait event mode
	isb
	dsb
	ldr	r1, =0x1111
	mov	r1, r1, lsl r0
	ldr	r7, =(TEGRA_CLK_RESET_BASE-IO_PPSB_PHYS+IO_PPSB_VIRT)
	str	r1, [r7, #0x340]	@ put slave CPU in reset
	isb
	dsb
	b	.
ENDPROC(__put_cpu_in_reset)


/*
 *	__cortex_a9_restore
 *
 *	 reloads the volatile CPU state from the context area
 *	 the MMU should already be enabled using the secondary_data
 *	 page tables for cpu_up before this function is called, and the
 *	 CPU should be coherent with the SMP complex
 */
ENTRY(__cortex_a9_restore)
	ctx_ptr	r8, r9
	ldmia	r8, {r0-r3}
	mov	sp, r0
	msr	cpsr_cxsf, r1
	msr	spsr_cxsf, r2

#ifdef CONFIG_VFPv3
	orr	r4, r3, #0xF00000
	mcr	p15, 0, r3, c1, c0, 2	@ enable coproc access
	mov	r1, #0x40000000
	VFPFMXR	FPEXC, r1		@ enable FPU access
	add	r9, r8, #CTX_VFP_REGS
	add	r7, r8, #CTX_FPEXC
	VFPFLDMIA r9, r10
	ldmia	r7, {r0, r1}
	VFPFMXR	FPSCR, r1
	VFPFMXR	FPEXC, r0
#endif

	mcr	p15, 0, r3, c1, c0, 2	@ cpacr (loaded before VFP)

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

	mcr	p15, 0, r5, c13, c0, 2	@ TPIDRURW
	mcr	p15, 0, r6, c13, c0, 3	@ TPIDRURO
	mcr	p15, 0, r7, c13, c0, 4	@ TPIDRPRW
	/* finally, restore the stack and return */
	ldmfd	sp!, {r4-r12, lr}
	mov	pc, lr
ENDPROC(__cortex_a9_restore)



	.align L1_CACHE_SHIFT
	.type __cortex_a9_context_area, %object
__cortex_a9_context_area:
	.space (CONTEXT_SIZE_WORDS * CONFIG_NR_CPUS * 4)