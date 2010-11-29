/*
 * arch/arm/mach-tegra/headsmp.S
 *
 * SMP initialization routines for Tegra SoCs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Copyright (c) 2011 Google, Inc.
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
 */

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/cache.h>

#include <mach/iomap.h>

#ifdef CONFIG_SMP
/*
 *	tegra_secondary_startup
 *
 *	 Initial secondary processor boot vector; jumps to kernel's
 *	 secondary_startup routine
 */
ENTRY(tegra_secondary_startup)
	bl	tegra_invalidate_l1
	bl	tegra_enable_coresite
	b	secondary_startup
ENDPROC(tegra_secondary_startup)

#ifdef CONFIG_PM
/*
 *	tegra_secondary_resume
 *
 *	  Secondary CPU boot vector when restarting a CPU following lp2 idle.
 */
ENTRY(tegra_secondary_resume)
	bl	tegra_invalidate_l1
	bl	tegra_enable_coresite
	b	cpu_resume
ENDPROC(tegra_secondary_resume)
#endif
#endif

#ifdef CONFIG_PM
/*
 *	tegra_resume
 *
 *	  CPU boot vector when restarting the master CPU following
 *	  an LP2 transition. Also branched to by LP0 and LP1 resume after
 *	  re-enabling sdram.
 */
ENTRY(tegra_resume)
	bl	tegra_invalidate_l1
	bl	tegra_enable_coresite

	/* enable SCU */
	ldr	r0, =TEGRA_ARM_PERIF_BASE
	ldr	r1, [r0]
	orr	r1, r1, #1
	str	r1, [r0]

	b	cpu_resume
ENDPROC(tegra_resume)
#endif

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
	ldr	r0, =0xC5ACCE55
	mcr	p14, 0, r0, c7, c12, 6
	mov	pc, lr
