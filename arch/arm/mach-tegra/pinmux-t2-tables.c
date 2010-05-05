/*
 * linux/arch/arm/mach-tegra/pinmux-t2-tables.c
 *
 * Common pinmux configurations for Tegra 2 SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/string.h>

#include <mach/pinmux.h>
#include "gpio-names.h"

#define gpio_pingroup(port, pin, pingroup) \
        [TEGRA_GPIO_P##port##pin]= TEGRA_PINGROUP_##pingroup

static const int gpio_pin_pingroup[] = {
	gpio_pingroup(A, 0, DTE),
	gpio_pingroup(A, 1, UCB),
	gpio_pingroup(A, 2, DAP2),
	gpio_pingroup(A, 3, DAP2),
	gpio_pingroup(A, 4, DAP2),
	gpio_pingroup(A, 5, DAP2),
	gpio_pingroup(A, 6, SDD),
	gpio_pingroup(A, 7, SDD),

	gpio_pingroup(B, 0, GMC),
	gpio_pingroup(B, 1, GMC),
	gpio_pingroup(B, 2, LPW0),
	gpio_pingroup(B, 3, LSC0),
	gpio_pingroup(B, 4, SDC),
	gpio_pingroup(B, 5, SDC),
	gpio_pingroup(B, 6, SDC),
	gpio_pingroup(B, 7, SDC),

	gpio_pingroup(C, 0, UCB),
	gpio_pingroup(C, 1, LPW1),
	gpio_pingroup(C, 2, UAD),
	gpio_pingroup(C, 3, UAD),
	gpio_pingroup(C, 4, RM),
	gpio_pingroup(C, 5, RM),
	gpio_pingroup(C, 6, LPW2),
	gpio_pingroup(C, 7, GMB),

	gpio_pingroup(D, 0, SLXK),
	gpio_pingroup(D, 1, SLXA),
	gpio_pingroup(D, 2, DTE),
	gpio_pingroup(D, 3, SLXC),
	gpio_pingroup(D, 4, SLXD),
	gpio_pingroup(D, 5, DTA),
	gpio_pingroup(D, 6, DTC),
	gpio_pingroup(D, 7, DTC),

	gpio_pingroup(E, 0, LD0),
	gpio_pingroup(E, 1, LD1),
	gpio_pingroup(E, 2, LD2),
	gpio_pingroup(E, 3, LD3),
	gpio_pingroup(E, 4, LD4),
	gpio_pingroup(E, 5, LD5),
	gpio_pingroup(E, 6, LD6),
	gpio_pingroup(E, 7, LD7),

	gpio_pingroup(F, 0, LD8),
	gpio_pingroup(F, 1, LD9),
	gpio_pingroup(F, 2, LD10),
	gpio_pingroup(F, 3, LD11),
	gpio_pingroup(F, 4, LD12),
	gpio_pingroup(F, 5, LD13),
	gpio_pingroup(F, 6, LD14),
	gpio_pingroup(F, 7, LD15),

	gpio_pingroup(G, 0, ATC),
	gpio_pingroup(G, 1, ATC),
	gpio_pingroup(G, 2, ATC),
	gpio_pingroup(G, 3, ATC),
	gpio_pingroup(G, 4, ATC),
	gpio_pingroup(G, 5, ATC),
	gpio_pingroup(G, 6, ATC),
	gpio_pingroup(G, 7, ATC),

	gpio_pingroup(H, 0, ATD),
	gpio_pingroup(H, 1, ATD),
	gpio_pingroup(H, 2, ATD),
	gpio_pingroup(H, 3, ATD),
	gpio_pingroup(H, 4, ATE),
	gpio_pingroup(H, 5, ATE),
	gpio_pingroup(H, 6, ATE),
	gpio_pingroup(H, 7, ATE),


	gpio_pingroup(I, 0, ATC),
	gpio_pingroup(I, 1, ATC),
	gpio_pingroup(I, 2, ATB),
	gpio_pingroup(I, 3, ATA),
	gpio_pingroup(I, 4, ATA),
	gpio_pingroup(I, 5, ATB),
	gpio_pingroup(I, 6, ATA),
	gpio_pingroup(I, 7, ATC),

	gpio_pingroup(J, 0, GMD),
	gpio_pingroup(J, 1, LSPI),
	gpio_pingroup(J, 2, GMD),
	gpio_pingroup(J, 3, LHS),
	gpio_pingroup(J, 4, LVS),
	gpio_pingroup(J, 5, IRTX),
	gpio_pingroup(J, 6, IRRX),
	gpio_pingroup(J, 7, GMC),

	gpio_pingroup(K, 0, ATC),
	gpio_pingroup(K, 1, ATC),
	gpio_pingroup(K, 2, ATC),
	gpio_pingroup(K, 3, ATC),
	gpio_pingroup(K, 4, ATC),
	gpio_pingroup(K, 5, SPDO),
	gpio_pingroup(K, 6, SPDI),
	gpio_pingroup(K, 7, GMC),

	gpio_pingroup(L, 0, DTD),
	gpio_pingroup(L, 1, DTD),
	gpio_pingroup(L, 2, DTD),
	gpio_pingroup(L, 3, DTD),
	gpio_pingroup(L, 4, DTD),
	gpio_pingroup(L, 5, DTD),
	gpio_pingroup(L, 6, DTD),
	gpio_pingroup(L, 7, DTD),

	gpio_pingroup(M, 0, LD16),
	gpio_pingroup(M, 1, LD17),
	gpio_pingroup(M, 2, LHP1),
	gpio_pingroup(M, 3, LHP2),
	gpio_pingroup(M, 4, LVP1),
	gpio_pingroup(M, 5, LHP0),
	gpio_pingroup(M, 6, LD1),
	gpio_pingroup(M, 7, LPP),

	gpio_pingroup(N, 0, DAP1),
	gpio_pingroup(N, 1, DAP1),
	gpio_pingroup(N, 2, DAP1),
	gpio_pingroup(N, 3, DAP1),
	gpio_pingroup(N, 4, LCSN),
	gpio_pingroup(N, 5, LSDA),
	gpio_pingroup(N, 6, LDC),
	gpio_pingroup(N, 7, HDINT),


	gpio_pingroup(O, 0, UAB),
	gpio_pingroup(O, 1, UAA),
	gpio_pingroup(O, 2, UAA),
	gpio_pingroup(O, 3, UAA),
	gpio_pingroup(O, 4, UAA),
	gpio_pingroup(O, 5, UAB),
	gpio_pingroup(O, 6, UAB),
	gpio_pingroup(O, 7, UAB),

	gpio_pingroup(P, 0, DAP3),
	gpio_pingroup(P, 1, DAP3),
	gpio_pingroup(P, 2, DAP3),
	gpio_pingroup(P, 3, DAP3),
	gpio_pingroup(P, 4, DAP4),
	gpio_pingroup(P, 5, DAP4),
	gpio_pingroup(P, 6, DAP4),
	gpio_pingroup(P, 7, DAP4),

	gpio_pingroup(Q, 0, KBCC),
	gpio_pingroup(Q, 1, KBCC),
	gpio_pingroup(Q, 2, KBCF),
	gpio_pingroup(Q, 3, KBCF),
	gpio_pingroup(Q, 4, KBCF),
	gpio_pingroup(Q, 5, KBCF),
	gpio_pingroup(Q, 6, KBCF),
	gpio_pingroup(Q, 7, KBCE),

	gpio_pingroup(R, 0, KBCA),
	gpio_pingroup(R, 1, KBCA),
	gpio_pingroup(R, 2, KBCA),
	gpio_pingroup(R, 3, KBCD),
	gpio_pingroup(R, 4, KBCD),
	gpio_pingroup(R, 5, KBCD),
	gpio_pingroup(R, 6, KBCD),
	gpio_pingroup(R, 7, KBCB),

	gpio_pingroup(S, 0, KBCB),
	gpio_pingroup(S, 1, KBCB),
	gpio_pingroup(S, 2, KBCB),
	gpio_pingroup(S, 3, KBCB),
	gpio_pingroup(S, 4, KBCB),
	gpio_pingroup(S, 5, KBCB),
	gpio_pingroup(S, 6, KBCB),
	gpio_pingroup(S, 7, KBCB),

	gpio_pingroup(T, 0, DTD),
	gpio_pingroup(T, 1, CSUS),
	gpio_pingroup(T, 2, DTB),
	gpio_pingroup(T, 3, DTB),
	gpio_pingroup(T, 4, DTA),
	gpio_pingroup(T, 5, PTA),
	gpio_pingroup(T, 6, PTA),
	gpio_pingroup(T, 7, ATB),

	gpio_pingroup(U, 0, GPU),
	gpio_pingroup(U, 1, GPU),
	gpio_pingroup(U, 2, GPU),
	gpio_pingroup(U, 3, GPU),
	gpio_pingroup(U, 4, GPU),
	gpio_pingroup(U, 5, GPU),
	gpio_pingroup(U, 6, GPU),
	gpio_pingroup(U, 7, GPU7),

	gpio_pingroup(V, 0, UAC),
	gpio_pingroup(V, 1, UAC),
	gpio_pingroup(V, 2, UAC),
	gpio_pingroup(V, 3, UAC),
	gpio_pingroup(V, 4, GPV),
	gpio_pingroup(V, 5, GPV),
	gpio_pingroup(V, 6, GPV),
	gpio_pingroup(V, 7, LVP0),

	gpio_pingroup(W, 0, LM0),
	gpio_pingroup(W, 1, LM1),
	gpio_pingroup(W, 2, SPIG),
	gpio_pingroup(W, 3, SPIH),
	gpio_pingroup(W, 4, CDEV1),
	gpio_pingroup(W, 5, CDEV2),
	gpio_pingroup(W, 6, UCA),
	gpio_pingroup(W, 7, UCA),

	gpio_pingroup(X, 0, SPIA),
	gpio_pingroup(X, 1, SPIB),
	gpio_pingroup(X, 2, SPIC),
	gpio_pingroup(X, 3, SPIC),
	gpio_pingroup(X, 4, SPID),
	gpio_pingroup(X, 5, SPIE),
	gpio_pingroup(X, 6, SPIE),
	gpio_pingroup(X, 7, SPIF),

	gpio_pingroup(Y, 0, UDA),
	gpio_pingroup(Y, 1, UDA),
	gpio_pingroup(Y, 2, UDA),
	gpio_pingroup(Y, 3, UDA),
	gpio_pingroup(Y, 4, SDIO1),
	gpio_pingroup(Y, 5, SDIO1),
	gpio_pingroup(Y, 6, SDIO1),
	gpio_pingroup(Y, 7, SDIO1),

	gpio_pingroup(Z, 0, SDIO1),
	gpio_pingroup(Z, 1, SDIO1),
	gpio_pingroup(Z, 2, LSDI),
	gpio_pingroup(Z, 3, LSC1),
	gpio_pingroup(Z, 4, LSCK),
	gpio_pingroup(Z, 5, PMC),
	gpio_pingroup(Z, 6, I2CP),
	gpio_pingroup(Z, 7, I2CP),

	gpio_pingroup(AA, 0, GMA),
	gpio_pingroup(AA, 1, GMA),
	gpio_pingroup(AA, 2, GMA),
	gpio_pingroup(AA, 3, GMA),
	gpio_pingroup(AA, 4, GME),
	gpio_pingroup(AA, 5, GME),
	gpio_pingroup(AA, 6, GME),
	gpio_pingroup(AA, 7, GME),


	gpio_pingroup(BB, 0, PMC),
	gpio_pingroup(BB, 1, DTE),
	gpio_pingroup(BB, 2, DTF),
	gpio_pingroup(BB, 3, DTF),
	gpio_pingroup(BB, 4, DTE),
	gpio_pingroup(BB, 5, DTE)
};

int gpio_get_pinmux_group(int gpio_nr)
{
	WARN_ON(gpio_nr >= ARRAY_SIZE(gpio_pin_pingroup) || gpio_nr < 0);
	if (gpio_nr >= ARRAY_SIZE(gpio_pin_pingroup) || gpio_nr < 0)
		return -EINVAL;

	return gpio_pin_pingroup[gpio_nr];
}
