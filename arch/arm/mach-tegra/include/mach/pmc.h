/*
 * arch/arm/mach-tegra/include/mach/pmc.h
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_PMC_H
#define __MACH_TEGRA_PMC_H

/**
 * Configure the kbc used pins for the deep power down mode.
 * The enabled row and column will be passed as parameter.
 * kbc_rows: The set bit position on this tells that corresponding row is
 *           enabled. Bit 0 corresponds to row 0.
 * kbc_cols: The set bit position on this tells that corresponding col is
 *           enabled. Bit 0 corresponds to col 0.
 */
void tegra_configure_dpd_kbc(unsigned int kbc_rows, unsigned int kbc_cols);

#endif
