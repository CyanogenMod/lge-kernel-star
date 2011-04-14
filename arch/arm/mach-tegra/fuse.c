/*
 * arch/arm/mach-tegra/fuse.c
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/string.h>

#include <mach/iomap.h>

#include "fuse.h"
#include "apbio.h"

#define FUSE_SKU_INFO		0x110
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define FUSE_UID_LOW		0x108
#define FUSE_UID_HIGH		0x10c
#define FUSE_SPARE_BIT		0x200
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
#define FUSE_VENDOR_CODE	0x200
#define FUSE_VENDOR_CODE_MASK	0xf
#define FUSE_FAB_CODE		0x204
#define FUSE_FAB_CODE_MASK	0x3f
#define FUSE_LOT_CODE_0		0x208
#define FUSE_LOT_CODE_1		0x20c
#define FUSE_WAFER_ID		0x210
#define FUSE_WAFER_ID_MASK	0x3f
#define FUSE_X_COORDINATE	0x214
#define FUSE_X_COORDINATE_MASK	0x1ff
#define FUSE_Y_COORDINATE	0x218
#define FUSE_Y_COORDINATE_MASK	0x1ff
#define FUSE_SPARE_BIT		0x244
#endif

static const char *tegra_revision_name[TEGRA_REVISION_MAX] = {
	[TEGRA_REVISION_UNKNOWN] = "unknown",
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	[TEGRA_REVISION_A02] = "A02",
	[TEGRA_REVISION_A03] = "A03",
	[TEGRA_REVISION_A03p] = "A03 prime",
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
	[TEGRA_REVISION_A01] = "A01",
#endif
};

u32 tegra_fuse_readl(unsigned long offset)
{
	return tegra_apb_readl(TEGRA_FUSE_BASE + offset);
}

void tegra_fuse_writel(u32 value, unsigned long offset)
{
	tegra_apb_writel(value, TEGRA_FUSE_BASE + offset);
}

static inline bool get_spare_fuse(int bit)
{
	return tegra_fuse_readl(FUSE_SPARE_BIT + bit * 4);
}

void tegra_init_fuse(void)
{
	u32 reg = readl(IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
	reg |= 1 << 28;
	writel(reg, IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
	tegra_init_speedo_data();

	pr_info("Tegra Revision: %s SKU: %d CPU Process: %d Core Process: %d\n",
		tegra_revision_name[tegra_get_revision()],
		tegra_sku_id(), tegra_cpu_process_id(),
		tegra_core_process_id());
}

unsigned long long tegra_chip_uid(void)
{
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	unsigned long long lo, hi;

	lo = tegra_fuse_readl(FUSE_UID_LOW);
	hi = tegra_fuse_readl(FUSE_UID_HIGH);
	return (hi << 32ull) | lo;
#else
	u64 uid = 0ull;
	u32 reg;
	u32 cid;
	u32 vendor;
	u32 fab;
	u32 lot;
	u32 wafer;
	u32 x;
	u32 y;
	u32 i;

	/* This used to be so much easier in prior chips. Unfortunately, there
	   is no one-stop shopping for the unique id anymore. It must be
	   constructed from various bits of information burned into the fuses
	   during the manufacturing process. The 64-bit unique id is formed
	   by concatenating several bit fields. The notation used for the
	   various fields is <fieldname:size_in_bits> with the UID composed
	   thusly:

            <CID:4><VENDOR:4><FAB:6><LOT:26><WAFER:6><X:9><Y:9>

	   Where:

		Field    Bits  Position Data
		-------  ----  -------- ----------------------------------------
		CID        4     60     Chip id (encoded as zero for T30)
		VENDOR     4     56     Vendor code
		FAB        6     50     FAB code
		LOT       26     24     Lot code (5-digit base-36-coded-decimal,
				            re-encoded to 26 bits binary)
		WAFER      6     18     Wafer id
		X          9      9     Wafer X-coordinate
		Y          9      0     Wafer Y-coordinate
		-------  ----
		Total     64
	*/

	/* Get the chip id and encode each chip variant as a unique value. */
	reg = readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE + 0x804));
	reg = (reg & 0xFF00) >> 8;

	switch (reg) {
	case 0x30:
		cid = 0;
		break;

	default:
		BUG();
		break;
	}

	vendor = tegra_fuse_readl(FUSE_VENDOR_CODE) & FUSE_VENDOR_CODE_MASK;
	fab = tegra_fuse_readl(FUSE_FAB_CODE) & FUSE_FAB_CODE_MASK;

	/* Lot code must be re-encoded from a 5 digit base-36 'BCD' number
	   to a binary number. */
	lot = 0;
	reg = tegra_fuse_readl(FUSE_LOT_CODE_1) << 2;

	for (i = 0; i < 5; ++i) {
		u32 digit = (reg & 0xFC000000) >> 26;
		BUG_ON(digit >= 36);
		lot *= 36;
		lot += digit;
		reg <<= 6;
	}

	wafer = tegra_fuse_readl(FUSE_WAFER_ID) & FUSE_WAFER_ID_MASK;
	x = tegra_fuse_readl(FUSE_X_COORDINATE) & FUSE_X_COORDINATE_MASK;
	y = tegra_fuse_readl(FUSE_Y_COORDINATE) & FUSE_Y_COORDINATE_MASK;

	uid = ((unsigned long long)cid  << 60ull)
	    | ((unsigned long long)vendor << 56ull)
	    | ((unsigned long long)fab << 50ull)
	    | ((unsigned long long)lot << 24ull)
	    | ((unsigned long long)wafer << 18ull)
	    | ((unsigned long long)x << 9ull)
	    | ((unsigned long long)y << 0ull);
	return uid;
#endif
}

unsigned int tegra_spare_fuse(int bit)
{
	BUG_ON(bit < 0 || bit > 61);
	return tegra_fuse_readl(FUSE_SPARE_BIT + bit * 4);
}

int tegra_sku_id(void)
{
	int sku_id;
	u32 reg = tegra_fuse_readl(FUSE_SKU_INFO);
	sku_id = reg & 0xFF;
	return sku_id;
}

struct tegra_id tegra_id;

static enum tegra_revision tegra_decode_revision(const struct tegra_id *id)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if ((id->chipid & 0xf0) != 0x20)
		return TEGRA_REVISION_UNKNOWN;
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	if ((id->chipid & 0xf0) != 0x30)
		return TEGRA_REVISION_UNKNOWN;

	switch (id->major) {
	case 0:
		if (id->minor != 1)
			return TEGRA_REVISION_UNKNOWN;
		else if (id->netlist == 12 && (id->patch & 0xf) == 12)
			return TEGRA_REVISION_A01;
		else if (id->netlist == 12 && (id->patch & 0xf) > 12)
			return TEGRA_REVISION_A02;
		else if (id->netlist > 12)
			return TEGRA_REVISION_A02;
		else
			return TEGRA_REVISION_UNKNOWN;
	case 1:
		break;
	default:
		return TEGRA_REVISION_UNKNOWN;
	}
#endif

	switch (id->minor) {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	case 1:
		return TEGRA_REVISION_A01;
#endif
	case 2:
		return TEGRA_REVISION_A02;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	case 3:
		return (*(id->priv) == 'p') ? TEGRA_REVISION_A03p
					    : TEGRA_REVISION_A03;
#endif
	default:
		return TEGRA_REVISION_UNKNOWN;
	}
}

static enum tegra_revision tegra_set_revision(u32 chipid,
					u32 major, u32 minor,
					u32 nlist, u32 patch, const char *priv)
{
	tegra_id.chipid  = chipid;
	tegra_id.major   = major;
	tegra_id.minor   = minor;
	tegra_id.netlist = nlist;
	tegra_id.patch   = patch;
	tegra_id.priv    = (char *)priv;
	tegra_id.revision = tegra_decode_revision(&tegra_id);
	return tegra_id.revision;
}

enum tegra_revision tegra_get_revision(void)
{
	if (tegra_id.chipid)
		return tegra_id.revision;
	else {/* Boot loader did not pass a valid chip ID */
		void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
		void __iomem *netlist = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x860;
		u32 cid = readl(chip_id);
		u32 nlist = readl(netlist);
		char *priv = NULL;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (get_spare_fuse(18) || get_spare_fuse(19))
			priv = "p";
#endif
		return tegra_set_revision((cid >> 8) & 0xff,
					(cid >> 4) & 0xf,
					(cid >> 16) & 0xf,
					(nlist >> 0) & 0xffff,
					(nlist >> 16) & 0xffff,
					priv);
	}
}

static char chippriv[16]; /* Permanent buffer for private string */
static int __init tegra_bootloader_tegraid(char *str)
{
	u32 id[5];
	int i = 0;
	char *priv = NULL;

	do {
		id[i++] = simple_strtoul(str, &str, 16);
	} while (*str++ && i < ARRAY_SIZE(id));

	if (*(str - 1) == '.') {
		strncpy(chippriv, str, sizeof(chippriv) - 1);
		priv = chippriv;
		if (strlen(str) > sizeof(chippriv) - 1)
			pr_err("### tegraid.priv in kernel arg truncated\n");
	}

	while (i < ARRAY_SIZE(id))
		id[i++] = 0;

	(void)tegra_set_revision(id[0], id[1], id[2], id[3], id[4], priv);
	return 0;
}

/* tegraid=chipid.major.minor.netlist.patch[.priv] */
early_param("tegraid", tegra_bootloader_tegraid);
