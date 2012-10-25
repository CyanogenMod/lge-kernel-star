/*
 * arch/arm/mach-tegra/common-t3.c
 *
 * Tegra 3 SoC-specific initialization (memory controller, etc.)
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "tegra3_emc.h"

#define MC_INT_STATUS			0x0
#define MC_INT_MASK			0x4
#define MC_INT_DECERR_EMEM		(1<<6)
#define MC_INT_SECURITY_VIOLATION	(1<<8)
#define MC_INT_ARBITRATION_EMEM		(1<<9)
#define MC_INT_INVALID_SMMU_PAGE	(1<<10)

#define MC_ERROR_STATUS			0x8
#define MC_ERROR_ADDRESS		0xC

#define MC_TIMING_REG_NUM1 \
	((MC_EMEM_ARB_TIMING_W2R - MC_EMEM_ARB_CFG) / 4 + 1)
#define MC_TIMING_REG_NUM2 \
	((MC_EMEM_ARB_MISC1 - MC_EMEM_ARB_DA_TURNS) / 4 + 1)

struct mc_client {
	const char *name;
};

#define client(_name)			\
	{				\
		.name = _name,		\
	}


static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);


#ifdef CONFIG_PM_SLEEP
static u32 mc_boot_timing[MC_TIMING_REG_NUM1 + MC_TIMING_REG_NUM2 + 4];

static void tegra_mc_timing_save(void)
{
	u32 off;
	u32 *ctx = mc_boot_timing;

	for (off = MC_EMEM_ARB_CFG; off <= MC_EMEM_ARB_TIMING_W2R; off += 4)
		*ctx++ = readl((u32)mc + off);

	for (off = MC_EMEM_ARB_DA_TURNS; off <= MC_EMEM_ARB_MISC1; off += 4)
		*ctx++ = readl((u32)mc + off);

	*ctx++ = readl((u32)mc + MC_EMEM_ARB_RING3_THROTTLE);
	*ctx++ = readl((u32)mc + MC_EMEM_ARB_OVERRIDE);
	*ctx++ = readl((u32)mc + MC_RESERVED_RSV);

	*ctx++ = readl((u32)mc + MC_INT_MASK);
}

void tegra_mc_timing_restore(void)
{
	u32 off;
	u32 *ctx = mc_boot_timing;

	for (off = MC_EMEM_ARB_CFG; off <= MC_EMEM_ARB_TIMING_W2R; off += 4)
		__raw_writel(*ctx++, (u32)mc + off);

	for (off = MC_EMEM_ARB_DA_TURNS; off <= MC_EMEM_ARB_MISC1; off += 4)
		__raw_writel(*ctx++, (u32)mc + off);

	__raw_writel(*ctx++, (u32)mc + MC_EMEM_ARB_RING3_THROTTLE);
	__raw_writel(*ctx++, (u32)mc + MC_EMEM_ARB_OVERRIDE);
	__raw_writel(*ctx++, (u32)mc + MC_RESERVED_RSV);

	writel(*ctx++, (u32)mc + MC_INT_MASK);
	off = readl((u32)mc + MC_INT_MASK);

	writel(0x1, (u32)mc + MC_TIMING_CONTROL);
	off = readl((u32)mc + MC_TIMING_CONTROL);
}
#else
#define tegra_mc_timing_save()
#endif


static const struct mc_client mc_clients[] = {
	client("ptc"),
	client("display0_wina"), client("display1_wina"),
	client("display0_winb"), client("display1_winb"),
	client("display0_winc"), client("display1_winc"),
	client("display0_winb_vfilter"),
	client("display1_winb_vfilter"),
	client("epp"), client("gr2d_pat"),
	client("gr2d_src"), client("mpe_unified"),
	client("vi_chroma_filter"), client("pcie"),
	client("avp"),
	client("display0_cursor"), client("display1_cursor"),
	client("gr3d0_fdc"), client("gr3d1_fdc"),
	client("gr2d_dst"), client("hda"),
	client("host1x_dma"), client("host1x_generic"),
	client("gr3d0_idx"), client("gr3d1_idx"),
	client("mpe_intrapred"), client("mpe_mpea"),
	client("mpe_mpec"), client("ahb_dma"),
	client("ahb_slave"), client("sata"),
	client("gr3d0_tex"), client("gr3d1_tex"),
	client("vde_bsev"), client("vde_mbe"),
	client("vde_mce"), client("vde_tpe"),
	client("cpu_lp"), client("cpu"),
	client("epp_u"), client("epp_v"),
	client("epp_y"), client("mpe_unified"),
	client("vi_sb"), client("vi_u"),
	client("vi_v"), client("vi_y"),
	client("gr2d_dst"), client("pcie"),
	client("avp"), client("gr3d0_fdc"),
	client("gr3d1_fdc"), client("hda"),
	client("host1x"),	client("isp"),
	client("cpu_lp"),	client("cpu"),
	client("mpe_mpec"), client("ahb_dma"),
	client("ahb_slave"), client("sata"),
	client("vde_bsev"), client("vde_dbg"),
	client("vde_mbe"), client("vde_tpm"),
};

static const char *smmu_page_attrib[] = {
	"SMMU: nr-nw-s",
	"SMMU: nr-nw-ns",
	"SMMU: nr-wr-s",
	"SMMU: nr-wr-ns",
	"SMMU: rd-nw-s",
	"SMMU: rd-nw-ns",
	"SMMU: rd-wr-s",
	"SMMU: rd-wr-ns"
};

static DEFINE_SPINLOCK(mc_lock);
static unsigned long error_count = 0;
#define MAX_PRINTS 5

static void unthrottle_prints(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&mc_lock, flags);
	error_count = 0;
	spin_unlock_irqrestore(&mc_lock, flags);
}

static DECLARE_DELAYED_WORK(unthrottle_prints_work, unthrottle_prints);

static irqreturn_t tegra_mc_error_isr(int irq, void *data)
{
	const struct mc_client *client = NULL;
	const char *mc_err;
	const char *mc_err_info;
	unsigned long count;
	u32 stat;
	u32 addr;
	u32 err;
	u32 type;
	u32 is_write;
	u32 is_secure;
	u32 client_id;

	stat = readl(mc + MC_INT_STATUS);
	stat &= (MC_INT_DECERR_EMEM |
		 MC_INT_SECURITY_VIOLATION |
		 MC_INT_INVALID_SMMU_PAGE);

	__cancel_delayed_work(&unthrottle_prints_work);

	spin_lock(&mc_lock);
	count = ++error_count;
	spin_unlock(&mc_lock);

	if (count >= MAX_PRINTS) {
		if (count == MAX_PRINTS)
			pr_err("Too many MC errors; throttling prints\n");
		schedule_delayed_work(&unthrottle_prints_work, HZ/2);
		goto out;
	}

	err = readl(mc + MC_ERROR_STATUS);
	addr = readl(mc + MC_ERROR_ADDRESS);
	is_write = err & (1<<16);
	is_secure = err & (1<<17);
	type = (err >> 28) & 7;
	client_id = err & 0x7f;
	if (client_id < ARRAY_SIZE(mc_clients))
		client = &mc_clients[client_id];

	if (stat & MC_INT_DECERR_EMEM)
		mc_err = "MC_DECERR";
	else if (stat & MC_INT_SECURITY_VIOLATION)
		mc_err = "MC_SECURITY_ERR";
	else if (stat & MC_INT_INVALID_SMMU_PAGE)
		mc_err = "MC_SMMU_ERR";
	else
		mc_err = "unknown";

	mc_err_info = "";
	if (type == 3) {
		mc_err_info = "SECURITY_TRUSTZONE";
	} else if (type == 4) {
		mc_err_info = "SECURITY_CARVEOUT";
	} else if (type == 6) {
		u32 attrib = (err >> 25) & 7;
		mc_err_info = smmu_page_attrib[attrib];
	}

	pr_err("%s (0x%08X): %p %s (%s %s %s)\n", mc_err, err, (void*)addr,
	       (client) ? client->name : "unknown",
	       (is_secure)? "secure" : "non-secure",
	       (is_write) ? "write" : "read",
	       mc_err_info);

out:
	writel(stat, mc + MC_INT_STATUS);
	return IRQ_HANDLED;
}

int __init tegra_mc_init(void)
{
	u32 reg;
	int ret = 0;

	reg = 0x0F7F1010;
	writel(reg, mc + MC_RESERVED_RSV);

	reg = readl(mc + MC_EMEM_ARB_OVERRIDE);
	reg |= 3;
	writel(reg, mc + MC_EMEM_ARB_OVERRIDE);

	if (request_irq(INT_MC_GENERAL, tegra_mc_error_isr, 0,
			"mc_status", NULL)) {
		pr_err("%s: unable to register MC error interrupt\n", __func__);
		ret = -ENXIO;
	} else {
		reg = MC_INT_DECERR_EMEM | MC_INT_SECURITY_VIOLATION |
				MC_INT_INVALID_SMMU_PAGE;
		writel(reg, mc + MC_INT_MASK);
	}
	tegra_mc_timing_save();

	return ret;
}
arch_initcall(tegra_mc_init);
