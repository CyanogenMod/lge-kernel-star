/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/memblock.h>
#include <linux/bitops.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/system.h>

#include <mach/gpio.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include "apbio.h"
#include "board.h"
#include "clock.h"
#include "fuse.h"
#include "pm.h"
#include "reset.h"
#include "tegra_smmu.h"

#define MC_SECURITY_CFG2	0x7c

#define AHB_ARBITRATION_PRIORITY_CTRL		0x4
#define   AHB_PRIORITY_WEIGHT(x)	(((x) & 0x7) << 29)
#define   PRIORITY_SELECT_USB	BIT(6)
#define   PRIORITY_SELECT_USB2	BIT(18)
#define   PRIORITY_SELECT_USB3	BIT(17)

#define AHB_GIZMO_AHB_MEM		0xc
#define   ENB_FAST_REARBITRATE	BIT(2)
#define   DONT_SPLIT_AHB_WR     BIT(7)

#define AHB_GIZMO_USB		0x1c
#define AHB_GIZMO_USB2		0x78
#define AHB_GIZMO_USB3		0x7c
#define   IMMEDIATE	BIT(18)

#define AHB_MEM_PREFETCH_CFG3	0xe0
#define AHB_MEM_PREFETCH_CFG4	0xe4
#define AHB_MEM_PREFETCH_CFG1	0xec
#define AHB_MEM_PREFETCH_CFG2	0xf0
#define   PREFETCH_ENB	BIT(31)
#define   MST_ID(x)	(((x) & 0x1f) << 26)
#define   AHBDMA_MST_ID	MST_ID(5)
#define   USB_MST_ID	MST_ID(6)
#define   USB2_MST_ID	MST_ID(18)
#define   USB3_MST_ID	MST_ID(17)
#define   ADDR_BNDRY(x)	(((x) & 0xf) << 21)
#define   INACTIVITY_TIMEOUT(x)	(((x) & 0xffff) << 0)

unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
bool tegra_lp0_vec_relocate;
unsigned long tegra_grhost_aperture = ~0ul;
static   bool is_tegra_debug_uart_hsport;
static struct board_info pmu_board_info;

static int pmu_core_edp = 1200;	/* default 1.2V EDP limit */
static int board_panel_type;
static enum power_supply_type pow_supply_type = POWER_SUPPLY_TYPE_MAINS;

void (*arch_reset)(char mode, const char *cmd) = tegra_assert_system_reset;

#define NEVER_RESET 0

void tegra_assert_system_reset(char mode, const char *cmd)
{
#if defined(CONFIG_TEGRA_FPGA_PLATFORM) || NEVER_RESET
	printk("tegra_assert_system_reset() ignored.....");
	do { } while (1);
#else
	void __iomem *reset = IO_ADDRESS(TEGRA_PMC_BASE + 0x00);
	u32 reg;

	/* use *_related to avoid spinlock since caches are off */
	reg = readl_relaxed(reset);
	reg |= 0x10;
	writel_relaxed(reg, reset);
#endif
}
static int modem_id;
static int debug_uart_port_id;

/* WARNING: There is implicit client of pllp_out3 like i2c, uart, dsi
 * and so this clock (pllp_out3) should never be disabled.
 */
static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	{ "pll_p",	NULL,		216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_p_out4",	"pll_p",	108000000,	true },
	{ "pll_m",	"clk_m",	600000000,	true },
	{ "pll_m_out1",	"pll_m",	120000000,	true },
	{ "sclk",	"pll_m_out1",	40000000,	true },
	{ "hclk",	"sclk",		40000000,	true },
	{ "pclk",	"hclk",		40000000,	true },
#else
	{ "pll_p",	NULL,		408000000,	true },
	{ "pll_p_out1",	"pll_p",	9600000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	102000000,	true },
	{ "pll_m_out1",	"pll_m",	275000000,	true },
	{ "pll_p_out4",	"pll_p",	102000000,	true },
	{ "sclk",	"pll_p_out4",	102000000,	true },
	{ "hclk",	"sclk",		102000000,	true },
	{ "pclk",	"hclk",		51000000,	true },
#endif
#else
	{ "pll_p",	NULL,		216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_m_out1",	"pll_m",	275000000,	true },
	{ "pll_c",	NULL,		ULONG_MAX,	false },
	{ "pll_c_out1",	"pll_c",	208000000,	false },
	{ "pll_p_out4",	"pll_p",	108000000,	true },
	{ "sclk",	"pll_p_out4",	108000000,	true },
	{ "hclk",	"sclk",		108000000,	true },
	{ "pclk",	"hclk",		54000000,	true },
#endif
	{ "csite",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ "kfuse",	NULL,		0,		true },
	{ "fuse",	NULL,		0,		true },
	{ "pll_u",	NULL,		480000000,	false },
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	true},
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	{ "cbus",	"pll_c",	416000000,	false },
	{ "pll_c_out1",	"pll_c",	208000000,	false },
#endif
	{ NULL,		NULL,		0,		0},
};

void tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;
	u32 aux_ctrl;

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	writel_relaxed(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(0x441, p + L2X0_DATA_LATENCY_CTRL);

#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	/* PL310 RAM latency is CPU dependent. NOTE: Changes here
	   must also be reflected in __cortex_a9_l2x0_restart */

	if (is_lp_cluster()) {
		writel(0x221, p + L2X0_TAG_LATENCY_CTRL);
		writel(0x221, p + L2X0_DATA_LATENCY_CTRL);
	} else {
		writel(0x441, p + L2X0_TAG_LATENCY_CTRL);
		writel(0x551, p + L2X0_DATA_LATENCY_CTRL);
	}
#else
	writel(0x770, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x770, p + L2X0_DATA_LATENCY_CTRL);
#endif
#endif
	aux_ctrl = readl(p + L2X0_CACHE_TYPE);
	aux_ctrl = (aux_ctrl & 0x700) << (17-8);
	aux_ctrl |= 0x7C000001;
	l2x0_init(p, aux_ctrl, 0x8200c3fe);
#endif
}

static void __init tegra_init_power(void)
{
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
        tegra_powergate_partition_with_clk_off(TEGRA_POWERGATE_SATA);
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	tegra_powergate_partition_with_clk_off(TEGRA_POWERGATE_PCIE);
#endif
}

static inline unsigned long gizmo_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}

static inline void gizmo_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}

static void __init tegra_init_ahb_gizmo_settings(void)
{
	unsigned long val;

	val = gizmo_readl(AHB_GIZMO_AHB_MEM);
	val |= ENB_FAST_REARBITRATE | IMMEDIATE | DONT_SPLIT_AHB_WR;
	gizmo_writel(val, AHB_GIZMO_AHB_MEM);

	val = gizmo_readl(AHB_GIZMO_USB);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB);

	val = gizmo_readl(AHB_GIZMO_USB2);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB2);

	val = gizmo_readl(AHB_GIZMO_USB3);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB3);

	val = gizmo_readl(AHB_ARBITRATION_PRIORITY_CTRL);
	val |= PRIORITY_SELECT_USB | PRIORITY_SELECT_USB2 | PRIORITY_SELECT_USB3
				| AHB_PRIORITY_WEIGHT(7);
	gizmo_writel(val, AHB_ARBITRATION_PRIORITY_CTRL);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG1);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | AHBDMA_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG1);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG2);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG2);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG3);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB3_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG3);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG4);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB2_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG4);
}

static bool console_flushed;

static void tegra_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	pr_emerg("Restarting %s\n", linux_banner);
	if (console_trylock()) {
		console_unlock();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (!console_trylock())
		pr_emerg("%s: Console was locked! Busting\n", __func__);
	else
		pr_emerg("%s: Console was locked!\n", __func__);
	console_unlock();
}

static void tegra_pm_restart(char mode, const char *cmd)
{
	tegra_pm_flush_console();
	arm_machine_restart(mode, cmd);
}

void __init tegra_init_early(void)
{
	arm_pm_restart = tegra_pm_restart;
#ifndef CONFIG_SMP
	/* For SMP system, initializing the reset handler here is too
	   late. For non-SMP systems, the function that calls the reset
	   handler initializer is not called, so do it here for non-SMP. */
	tegra_cpu_reset_handler_init();
#endif
	tegra_init_fuse();
	tegra_gpio_resume_init();
	tegra_init_clock();
	tegra_init_pinmux();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
	tegra_init_ahb_gizmo_settings();
}

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);
	if (!tegra_lp0_vec_size || !tegra_lp0_vec_start) {
		tegra_lp0_vec_size = 0;
		tegra_lp0_vec_start = 0;
	}

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

enum panel_type get_panel_type(void)
{
	return board_panel_type;
}
static int __init tegra_board_panel_type(char *options)
{
	if (!strcmp(options, "lvds"))
		board_panel_type = panel_type_lvds;
	else if (!strcmp(options, "dsi"))
		board_panel_type = panel_type_dsi;
	else
		return 0;
	return 1;
}
__setup("panel=", tegra_board_panel_type);

enum power_supply_type get_power_supply_type(void)
{
	return pow_supply_type;
}
static int __init tegra_board_power_supply_type(char *options)
{
	if (!strcmp(options, "Adapter"))
		pow_supply_type = POWER_SUPPLY_TYPE_MAINS;
	if (!strcmp(options, "Mains"))
		pow_supply_type = POWER_SUPPLY_TYPE_MAINS;
	else if (!strcmp(options, "Battery"))
		pow_supply_type = POWER_SUPPLY_TYPE_BATTERY;
	else
		return 0;
	return 1;
}
__setup("power_supply=", tegra_board_power_supply_type);

int get_core_edp(void)
{
	return pmu_core_edp;
}
static int __init tegra_pmu_core_edp(char *options)
{
	char *p = options;
	int core_edp = memparse(p, &p);
	if (core_edp != 0)
		pmu_core_edp = core_edp;
	return 0;
}
early_param("core_edp_mv", tegra_pmu_core_edp);

static int __init tegra_debug_uartport(char *info)
{
	char *p = info;
	if (!strncmp(p, "hsport", 6))
		is_tegra_debug_uart_hsport = true;
	else if (!strncmp(p, "lsport", 6))
		is_tegra_debug_uart_hsport = false;

	if (p[6] == ',')
		debug_uart_port_id = memparse(p + 7, &p);
	else
		debug_uart_port_id = -1;

	return 1;
}

bool is_tegra_debug_uartport_hs(void)
{
	return is_tegra_debug_uart_hsport;
}

int get_tegra_uart_debug_port_id(void)
{
	return debug_uart_port_id;
}
__setup("debug_uartport=", tegra_debug_uartport);

void tegra_get_board_info(struct board_info *bi)
{
	bi->board_id = (system_serial_high >> 16) & 0xFFFF;
	bi->sku = (system_serial_high) & 0xFFFF;
	bi->fab = (system_serial_low >> 24) & 0xFF;
	bi->major_revision = (system_serial_low >> 16) & 0xFF;
	bi->minor_revision = (system_serial_low >> 8) & 0xFF;
}

static int __init tegra_pmu_board_info(char *info)
{
	char *p = info;
	pmu_board_info.board_id = memparse(p, &p);
	pmu_board_info.sku = memparse(p+1, &p);
	pmu_board_info.fab = memparse(p+1, &p);
	pmu_board_info.major_revision = memparse(p+1, &p);
	pmu_board_info.minor_revision = memparse(p+1, &p);
	return 1;
}

void tegra_get_pmu_board_info(struct board_info *bi)
{
	memcpy(bi, &pmu_board_info, sizeof(struct board_info));
}

__setup("pmuboard=", tegra_pmu_board_info);

static int __init tegra_modem_id(char *id)
{
	char *p = id;

	modem_id = memparse(p, &p);
	return 1;
}

int tegra_get_modem_id(void)
{
	return modem_id;
}

__setup("modem_id=", tegra_modem_id);

/*
 * Tegra has a protected aperture that prevents access by most non-CPU
 * memory masters to addresses above the aperture value.  Enabling it
 * secures the CPU's memory from the GPU, except through the GART.
 */
void __init tegra_protected_aperture_init(unsigned long aperture)
{
#ifndef CONFIG_NVMAP_ALLOW_SYSMEM
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	pr_info("Enabling Tegra protected aperture at 0x%08lx\n", aperture);
	writel(aperture, mc_base + MC_SECURITY_CFG2);
#else
	pr_err("Tegra protected aperture disabled because nvmap is using "
		"system memory\n");
#endif
}

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i += 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

#ifdef CONFIG_TEGRA_IOVMM_SMMU
/* Support for Tegra3 A01 chip mask that needs to have SMMU IOVA reside in
 * the upper half of 4GB IOVA space. A02 and after use the bottom 1GB and
 * do not need to reserve memory.
 */
#define SUPPORT_TEGRA_3_IOVMM_SMMU_A01
#endif

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
#ifdef SUPPORT_TEGRA_3_IOVMM_SMMU_A01
	int smmu_reserved = 0;
	struct tegra_smmu_window *smmu_window = tegra_smmu_window(0);
#endif

	if (carveout_size) {
		tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
		if (memblock_remove(tegra_carveout_start, carveout_size)) {
			pr_err("Failed to remove carveout %08lx@%08lx "
				"from memory map\n",
				carveout_size, tegra_carveout_start);
			tegra_carveout_start = 0;
			tegra_carveout_size = 0;
		} else
			tegra_carveout_size = carveout_size;
	}

	if (fb2_size) {
		tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
		if (memblock_remove(tegra_fb2_start, fb2_size)) {
			pr_err("Failed to remove second framebuffer "
				"%08lx@%08lx from memory map\n",
				fb2_size, tegra_fb2_start);
			tegra_fb2_start = 0;
			tegra_fb2_size = 0;
		} else
			tegra_fb2_size = fb2_size;
	}

	if (fb_size) {
		tegra_fb_start = memblock_end_of_DRAM() - fb_size;
		if (memblock_remove(tegra_fb_start, fb_size)) {
			pr_err("Failed to remove framebuffer %08lx@%08lx "
				"from memory map\n",
				fb_size, tegra_fb_start);
			tegra_fb_start = 0;
			tegra_fb_size = 0;
		} else
			tegra_fb_size = fb_size;
	}

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

#ifdef SUPPORT_TEGRA_3_IOVMM_SMMU_A01
	if (!smmu_window) {
		pr_err("No SMMU resource\n");
	} else {
		size_t smmu_window_size;

		if (tegra_get_revision() == TEGRA_REVISION_A01) {
			smmu_window->start = TEGRA_SMMU_BASE_A01;
			smmu_window->end   = TEGRA_SMMU_BASE_A01 +
						TEGRA_SMMU_SIZE_A01 - 1;
		}
		smmu_window_size = smmu_window->end + 1 - smmu_window->start;
		if (smmu_window->start >= 0x80000000) {
			if (memblock_reserve(smmu_window->start,
						smmu_window_size))
				pr_err(
			"Failed to reserve SMMU I/O VA window %08lx@%08lx\n",
				(unsigned long)smmu_window_size,
				(unsigned long)smmu_window->start);
			else
				smmu_reserved = 1;
		}
	}
#endif

	if (tegra_lp0_vec_size &&
	   (tegra_lp0_vec_start < memblock_end_of_DRAM())) {
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size)) {
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);
			tegra_lp0_vec_start = 0;
			tegra_lp0_vec_size = 0;
		}
		tegra_lp0_vec_relocate = false;
	} else
		tegra_lp0_vec_relocate = true;

	/*
	 * We copy the bootloader's framebuffer to the framebuffer allocated
	 * above, and then free this one.
	 * */
	if (tegra_bootloader_fb_size) {
		tegra_bootloader_fb_size = PAGE_ALIGN(tegra_bootloader_fb_size);
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size)) {
			pr_err("Failed to reserve bootloader frame buffer "
				"%08lx@%08lx\n", tegra_bootloader_fb_size,
				tegra_bootloader_fb_start);
			tegra_bootloader_fb_start = 0;
			tegra_bootloader_fb_size = 0;
		}
	}

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:        %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_size ?
			tegra_lp0_vec_start + tegra_lp0_vec_size - 1 : 0,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_size ?
			tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1 : 0,
		tegra_fb_start,
		tegra_fb_size ?
			tegra_fb_start + tegra_fb_size - 1 : 0,
		tegra_fb2_start,
		tegra_fb2_size ?
			tegra_fb2_start + tegra_fb2_size - 1 : 0,
		tegra_carveout_start,
		tegra_carveout_size ?
			tegra_carveout_start + tegra_carveout_size - 1 : 0);

#ifdef SUPPORT_TEGRA_3_IOVMM_SMMU_A01
	if (smmu_reserved)
		pr_info("SMMU:                   %08lx - %08lx\n",
			smmu_window->start, smmu_window->end);
#endif
}

void __init tegra_release_bootloader_fb(void)
{
	/* Since bootloader fb is reserved in common.c, it is freed here. */
	if (tegra_bootloader_fb_size)
		if (memblock_free(tegra_bootloader_fb_start,
						tegra_bootloader_fb_size))
			pr_err("Failed to free bootloader fb.\n");
}

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
static char cpufreq_gov_default[32];
static char *cpufreq_gov_conservative = "conservative";
static char *cpufreq_sysfs_place_holder="/sys/devices/system/cpu/cpu%i/cpufreq/scaling_governor";
static char *cpufreq_gov_conservative_param="/sys/devices/system/cpu/cpufreq/conservative/%s";

static void cpufreq_set_governor(char *governor)
{
	struct file *scaling_gov = NULL;
	char    buf[128];
	int i;
	loff_t offset = 0;

	if (governor == NULL)
		return;

	for_each_online_cpu(i) {
		sprintf(buf, cpufreq_sysfs_place_holder, i);
		scaling_gov = filp_open(buf, O_RDWR, 0);
		if (scaling_gov != NULL) {
			if (scaling_gov->f_op != NULL &&
				scaling_gov->f_op->write != NULL)
				scaling_gov->f_op->write(scaling_gov,
						governor,
						strlen(governor),
						&offset);
			else
				pr_err("f_op might be null\n");

			filp_close(scaling_gov, NULL);
		} else {
			pr_err("%s. Can't open %s\n", __func__, buf);
		}
	}
}

void cpufreq_save_default_governor(void)
{
	struct file *scaling_gov = NULL;
	char    buf[128];
	loff_t offset = 0;

	buf[127] = 0;
	sprintf(buf, cpufreq_sysfs_place_holder,0);
	scaling_gov = filp_open(buf, O_RDONLY, 0);
	if (scaling_gov != NULL) {
		if (scaling_gov->f_op != NULL &&
			scaling_gov->f_op->read != NULL)
			scaling_gov->f_op->read(scaling_gov,
					cpufreq_gov_default,
					32,
					&offset);
		else
			pr_err("f_op might be null\n");

		filp_close(scaling_gov, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}
}

void cpufreq_restore_default_governor(void)
{
	cpufreq_set_governor(cpufreq_gov_default);
}

void cpufreq_set_conservative_governor_param(int up_th, int down_th)
{
	struct file *gov_param = NULL;
	static char buf[128],parm[8];
	loff_t offset = 0;

	if (up_th <= down_th) {
		printk(KERN_ERR "%s: up_th(%d) is lesser than down_th(%d)\n",
			__func__, up_th, down_th);
		return;
	}

	sprintf(parm, "%d", up_th);
	sprintf(buf, cpufreq_gov_conservative_param ,"up_threshold");
	gov_param = filp_open(buf, O_RDONLY, 0);
	if (gov_param != NULL) {
		if (gov_param->f_op != NULL &&
			gov_param->f_op->write != NULL)
			gov_param->f_op->write(gov_param,
					parm,
					strlen(parm),
					&offset);
		else
			pr_err("f_op might be null\n");

		filp_close(gov_param, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}

	sprintf(parm, "%d", down_th);
	sprintf(buf, cpufreq_gov_conservative_param ,"down_threshold");
	gov_param = filp_open(buf, O_RDONLY, 0);
	if (gov_param != NULL) {
		if (gov_param->f_op != NULL &&
			gov_param->f_op->write != NULL)
			gov_param->f_op->write(gov_param,
					parm,
					strlen(parm),
					&offset);
		else
			pr_err("f_op might be null\n");

		filp_close(gov_param, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}
}

void cpufreq_set_conservative_governor(void)
{
	cpufreq_set_governor(cpufreq_gov_conservative);
}
#endif /* CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND */
