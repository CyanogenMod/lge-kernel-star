/*
 * arch/arm/mach-tegra/fuse-cache.c
 *
 * Interface to kfuses on Tegra 200
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <mach/fuse.h>
#include <mach/iomap.h>
#include <ap20/arclk_rst.h>
#include <ap20/arfuse.h>

/* register definition */
#define KFUSE_STATE 0x80
#define KFUSE_STATE_DONE 0x10000
#define KFUSE_STATE_CRCPASS 0x20000
#define KFUSE_KEYADDR 0x88
#define KFUSE_KEYADDR_AUTOINC 0x10000
#define KFUSE_KEYADDR_ADDR(addr) (addr)
#define KFUSE_KEYS 0x8c

#define KFUSE_CACHE_SZ (144 * 4)

static u32 *kfuse_cache;
static int kfuse_cache_isvalid;

/* set start address in auto-increment mode */
static inline void kfuse_set_autoinc_addr(u16 addr)
{
	writel(KFUSE_KEYADDR_ADDR(0) | KFUSE_KEYADDR_AUTOINC,
		IO_ADDRESS(TEGRA_KFUSE_BASE) + KFUSE_KEYADDR);
}

static inline void kfuse_clock_enable(int e)
{
	if (e) {
		writel(CLK_RST_CONTROLLER_CLK_ENB_H_SET_0_SET_CLK_ENB_KFUSE_FIELD,
			IO_ADDRESS(TEGRA_CLK_RESET_BASE +
			CLK_RST_CONTROLLER_CLK_ENB_H_SET_0));
	} else {
		writel(CLK_RST_CONTROLLER_CLK_ENB_H_CLR_0_CLR_CLK_ENB_KFUSE_FIELD,
			IO_ADDRESS(TEGRA_CLK_RESET_BASE +
			CLK_RST_CONTROLLER_CLK_ENB_H_CLR_0));
	}
}

static void kfuse_wait_ready(void)
{
	int retries = 1;
	/* wait for hardware to finish loading and verifying key data */
	do {
		u32 val;
		val = readl(IO_ADDRESS(TEGRA_KFUSE_BASE) + KFUSE_STATE);
		if ((val & KFUSE_STATE_DONE) == KFUSE_STATE_DONE) {
			/* hardware does CRC check */
			kfuse_cache_isvalid = (val & KFUSE_STATE_CRCPASS) == KFUSE_STATE_CRCPASS;
			break;
		}
		msleep(10);
	} while( --retries >= 0 );
}

static void kfuse_rewind(void)
{
	/* force HW to decode and check fuses if it has not already done so */
	kfuse_set_autoinc_addr(0);
	kfuse_wait_ready();
	// kfuse_set_autoinc_addr(0);
}

static inline u32 kfuse_read(void)
{
	return readl(IO_ADDRESS(TEGRA_KFUSE_BASE) + KFUSE_KEYS);
}

/* this is called very early in init because there is a bug that can cause
 * corruption if DMA and non-DMA requests overlap on APB bus. */
void __init tegra_init_fuse_cache(void) {
	unsigned i;

	kfuse_cache = kzalloc(KFUSE_CACHE_SZ, GFP_KERNEL);

	kfuse_clock_enable(1);

	kfuse_rewind();

	printk(KERN_DEBUG "kfuse_cache_isvalid=%d\n", kfuse_cache_isvalid);

	if (!kfuse_cache_isvalid) {
		printk(KERN_ERR "kfuse CRC or ECC error\n");
	} else {
		/* load if no CRC or ECC errors */
		for (i = 0; i < KFUSE_CACHE_SZ / sizeof (u32); i ++)
			kfuse_cache[i] = kfuse_read();
	}

	kfuse_clock_enable(0);
}

const u32 *tegra_kfuse_cache_get(size_t *size) {
	if (size) *size = KFUSE_CACHE_SZ;
	return kfuse_cache;
}
