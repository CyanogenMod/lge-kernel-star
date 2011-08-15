/*
 * arch/arm/mach-tegra/fuse.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010-2011 NVIDIA Corp.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <mach/hardware.h>

#define INVALID_PROCESS_ID	99 /* don't expect to have 100 process id's */

unsigned long long tegra_chip_uid(void);
unsigned int tegra_spare_fuse(int bit);
int tegra_sku_id(void);
void tegra_init_fuse(void);
u32 tegra_fuse_readl(unsigned long offset);
void tegra_fuse_writel(u32 value, unsigned long offset);
enum tegra_chipid tegra_get_chipid(void);
enum tegra_revision tegra_get_revision(void);
const char *tegra_get_revision_name(void);

#ifndef CONFIG_TEGRA_FPGA_PLATFORM

int tegra_cpu_process_id(void);
int tegra_core_process_id(void);
int tegra_cpu_speedo_id(void);
int tegra_soc_speedo_id(void);
void tegra_init_speedo_data(void);

#else /* CONFIG_TEGRA_FPGA_PLATFORM */

static inline int tegra_cpu_process_id(void) { return 0; }
static inline int tegra_core_process_id(void) { return 0; }
static inline int tegra_cpu_speedo_id(void) { return 0; }
static inline int tegra_soc_speedo_id(void) { return 0; }
static inline void tegra_init_speedo_data(void) { }

#endif /* CONFIG_TEGRA_FPGA_PLATFORM */
