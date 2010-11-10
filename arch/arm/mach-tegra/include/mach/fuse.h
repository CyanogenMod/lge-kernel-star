/*
 * arch/arm/mach-tegra/fuse.h
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_FUSE_H
#define __MACH_TEGRA_FUSE_H

#include <linux/init.h>

void __init tegra_init_fuse_cache(void);
const u32 *tegra_kfuse_cache_get(size_t *size);

#endif
