/*
 * arch/arm/mach-tegra/include/mach/io_dpd.h
 *
 * Copyright (C) 2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_IO_DPD_H
#define __MACH_TEGRA_IO_DPD_H

/* Tegra io dpd APIs */
#ifdef CONFIG_PM_SLEEP
struct tegra_io_dpd *tegra_io_dpd_get(struct device *dev); /* get handle */
void tegra_io_dpd_enable(struct tegra_io_dpd *hnd); /* enable dpd */
void tegra_io_dpd_disable(struct tegra_io_dpd *hnd); /* disable dpd */
#else
static inline struct tegra_io_dpd *tegra_io_dpd_get(struct device *dev)
{
	return NULL;
}
static inline void tegra_io_dpd_enable(struct tegra_io_dpd *hnd)
{
	/* Do nothing */
}
static inline void tegra_io_dpd_disable(struct tegra_io_dpd *hnd)
{
	/* Do nothing */
}
#endif

#endif /* end __MACH_TEGRA_IO_DPD_H */
