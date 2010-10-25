/*
 *  arch/arm/mach-tegra/pci.c
 *
 *  PCIe host controller driver for TEGRA(2) SOCs
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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

//#define DEBUG
//#define VERBOSE_DEBUG


#include <linux/kernel.h>
#include <linux/pci.h>
#include <asm/mach/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <mach/pci.h>
#include <mach/nvrm_linux.h>
#include <mach/iomap.h>
#include <mach/clk.h>

#include "nvrm_pmu.h"
#include "nvodm_query_discovery.h"
#include "nvrm_power.h"
#include "nvrm_interrupt.h"


void __iomem * volatile pci_tegra_regs;
static bool pci_tegra_device_attached = false;
static bool pci_tegra_rp0_up = false;
static bool pci_tegra_rp1_up = false;

static void __init pci_tegra_preinit(void);
static int __init pci_tegra_setup(int nr, struct pci_sys_data *data);
static struct pci_bus __init *pci_tegra_scan_bus(int nr,
	struct pci_sys_data *sys);

static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val);
static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val);

static void pci_tegra_setup_translations(void);
static irqreturn_t pci_tegra_isr(int irq, void *arg);
static bool pci_tegra_check_rp(int rp);

static inline bool pci_tegra_is_within_rp_range(u32 bus_number, int rp)
{
	bool ret=false;
	u8 primary;
	u8 secondary;
	u8 subordinate;
	u32 busrange;

	busrange = pci_tegra_rp_readl(PCI_PRIMARY_BUS, rp);

	primary = (u8)((busrange & 0x000000ff)>>0);
	secondary = (u8)((busrange & 0x0000ff00)>>8);
	subordinate = (u8)((busrange & 0x00ff0000)>>16);

	if (((rp==0) && pci_tegra_rp0_up) || ((rp==1) && pci_tegra_rp1_up)) {
		if(primary != subordinate) {
			//otherwise it's not configured
			if((bus_number >= secondary) && (bus_number <= subordinate))
				ret=true;
		}
	}

	return ret;
}

static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val)
{
	int i;
	u32 v;
	int rp;

	pr_debug("Issuing read conf: bus %d, devfn 0x%x, where 0x%x size %d\n",
		bus->number, devfn, where, size);

	if (!pci_tegra_device_attached) goto fail;
	if (where > 4096) goto fail;

	/* Root port config registers are directly mapped in different
	 * aperture - not same as the config registers for devices on the PCI
	 * bus
	 */
	if (pci_tegra_is_rp(bus->number, &rp))  {
		/* Root port is just one bridge! */
		if (devfn != 0) goto fail;
		v = pci_tegra_rp_readl(where & ~3, rp);
	} else  {
		void __iomem *addr;
		bool is_rp_firstdev;
		bool is_valid_dev;

		/*Make sure the bus falls within one of the root ports*/
		is_valid_dev = pci_tegra_is_within_rp_range(bus->number, 0)
				 || pci_tegra_is_within_rp_range(bus->number, 1);
		if (! is_valid_dev) goto fail;

		/* Root is only attached to one device/bridge */
		is_rp_firstdev =  pci_tegra_is_rp_first_dev(bus->number, 0)
		                || pci_tegra_is_rp_first_dev(bus->number, 1);
		if (is_rp_firstdev && PCI_SLOT(devfn) != 0) goto fail;

		addr = pci_tegra_config_addr(bus->number, devfn, where & ~3);
		v = readl(addr);
	}

	switch (size) {
	case 1:
		if (where & 2) v >>= 16;
		if (where & 1) v >>= 8;
		v &= 0xff;
		break;
	case 2:
		if (where & 2) v >>= 16;
		v &= 0xffff;
		break;
	case 4:
		break;
	default:
		/* If the PCI stack is sane, we should not get here */
		BUG();
	}
	*val = v;

	pr_debug("   Value = 0x%x\n", v);

	return PCIBIOS_SUCCESSFUL;
fail:
	for (i=0; i<size; i++)
		((__u8 *)val)[i] = 0xff;
	return PCIBIOS_SUCCESSFUL;
}

static int pci_tegra_write_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 val)
{
	void __iomem *addr;
	int rp;
	u32 temp;

	pr_debug("Issuing write conf: bus %d, devfn 0x%x, "
		"where 0x%x size %d value = 0x%x\n",
		bus->number, devfn, where, size, val);

	if (!pci_tegra_device_attached) goto fail;

	if (where > 4096) goto fail;

	/* Root port config registers are directly mapped in different
	 * aperture - not same as the config registers for devices on the PCI
	 * bus.
	 */
	if (pci_tegra_is_rp(bus->number, &rp))  {
		/* Root port is just one bridge! */
		if (devfn != 0) goto fail;
		addr = pci_tegra_regs;
		if (rp == 0) addr += NV_PCIE_AXI_RP_T0C0_OFFSET;
		if (rp == 1) addr += NV_PCIE_AXI_RP_T0C1_OFFSET;
		addr += where;
	} else  {
		bool is_rp_firstdev;
		bool is_valid_dev;

		/*Make sure the bus falls within one of the root ports*/
		is_valid_dev = pci_tegra_is_within_rp_range(bus->number, 0)
				 || pci_tegra_is_within_rp_range(bus->number, 1);
		if (!is_valid_dev) goto fail;

		/* Root is only attached to one device/bridge */
		is_rp_firstdev =  pci_tegra_is_rp_first_dev(bus->number, 0)
				 || pci_tegra_is_rp_first_dev(bus->number, 1);
		if (is_rp_firstdev && PCI_SLOT(devfn) != 0) goto fail;

		addr = pci_tegra_config_addr(bus->number, devfn, where);
	}

	switch (size) {
	case 1:
		temp = readl((u32)addr & ~0x3);
		temp &= ~(0xff << ((where & 0x3) * 8));
		temp |= val << ((where & 0x3) * 8);
		writel(temp, (u32)addr & ~0x3);
		break;
	case 2:
		temp = readl((u32)addr & ~0x3);
		temp &= ~(0xffff << ((where & 0x3) * 8));
		temp |= val << ((where & 0x3) * 8);
		writel(temp, (u32)addr & ~0x3);
		break;
	case 4:
		writel(val, addr);
		break;
	default:
		/* If the PCI stack is sane, we should not get here */
		BUG();
	}
fail:
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pci_tegra_ops = {
	.read	= pci_tegra_read_conf,
	.write	= pci_tegra_write_conf,
};

static void __init pci_tegra_preinit(void)
{
	pcibios_setup("firmware");
}

static int __init pci_tegra_setup(int nr, struct pci_sys_data *data)
{
	u32 volatile reg;
	unsigned int irq;
	struct clk *clk_pcie = NULL;
	struct clk *clk_pciex = NULL;
	struct regulator *regulator = NULL;
	int ret = 0;

	if ((nr == 1) && pci_tegra_device_attached) return 1;
	else if (nr >= 1) return 0;

	pci_tegra_regs = ioremap_nocache(TEGRA_PCIE_BASE,
		PCI_TEGRA_IOMAPPED_REG_APERTURE_SIZE);
	if (pci_tegra_regs == NULL) {
		pr_err("pci_tegra_setup: Failed to map the PCI/AFI regs\n");
		return 0;
	}

	regulator = regulator_get(NULL, "pex_clk");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("%s: unable to get pex_clk regulator\n", __func__);
		goto done;
	}
	regulator_enable(regulator);

	clk_pcie = clk_get_sys("tegra_pcie", NULL);
	if (IS_ERR_OR_NULL(clk_pcie)) {
		pr_err("%s: unable to get PCIE clock\n", __func__);
		goto done;
	}
	clk_enable(clk_pcie);

	clk_pciex = clk_get_sys("tegra_pcie_xclk", NULL);
	if (IS_ERR_OR_NULL(clk_pciex)) {
		pr_err("%s: unable to get PCIE Xclock\n", __func__);
		goto done;
	}
	tegra_periph_reset_assert(clk_pciex);
	udelay(10);

	/* Enable slot clock and pulse the reset signals */
	reg = pci_tegra_afi_readl(AFI_PEX0_CTRL_0);

	reg = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_REFCLK_EN, 1, reg);
	pci_tegra_afi_writel(reg, AFI_PEX0_CTRL_0);
	reg  = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 0, reg);
	pci_tegra_afi_writel(reg, AFI_PEX0_CTRL_0);

	reg = pci_tegra_afi_readl(AFI_PEX1_CTRL_0);
	reg = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_REFCLK_EN, 1, reg);
	pci_tegra_afi_writel(reg, AFI_PEX1_CTRL_0);
	reg  = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_RST_L, 0, reg);
	pci_tegra_afi_writel(reg, AFI_PEX1_CTRL_0);

	msleep(100);

	reg = pci_tegra_afi_readl(AFI_PEX0_CTRL_0);
	reg  = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 1, reg);
	pci_tegra_afi_writel(reg, AFI_PEX0_CTRL_0);

	reg = pci_tegra_afi_readl(AFI_PEX1_CTRL_0);
	reg  = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_RST_L, 1, reg);
	pci_tegra_afi_writel(reg, AFI_PEX1_CTRL_0);

	/* Validate by reading the ROOT port IDs that we are infact working on
	 * the TEGRA root port */
	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_DEV_ID, 0);
	BUG_ON((NVPCIE_DRF_VAL(RP, DEV_ID, VENDOR_ID, reg)) !=
		NV_PROJ__PCIE2_RP_DEV_ID_VENDOR_ID_NVIDIA);
	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_DEV_ID, 1);
	BUG_ON((NVPCIE_DRF_VAL(RP, DEV_ID, VENDOR_ID, reg)) !=
		NV_PROJ__PCIE2_RP_DEV_ID_VENDOR_ID_NVIDIA);

	/* Enable dual controller and both ports*/
	reg = pci_tegra_afi_readl(AFI_PCIE_CONFIG_0);
	reg = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, PCIEC0_DISABLE_DEVICE, 0,
		reg);
	reg = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, SM2TMS0_XBAR_CONFIG, 1,
		reg);
	reg = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, PCIEC1_DISABLE_DEVICE, 0,
		reg);
	pci_tegra_afi_writel(reg, AFI_PCIE_CONFIG_0);

	reg = pci_tegra_afi_readl(AFI_FUSE_0);
	reg = NV_FLD_SET_DRF_NUM(AFI, FUSE, FUSE_PCIE_T0_GEN2_DIS, 0, reg);
	pci_tegra_afi_writel(reg, AFI_FUSE_0);

	/* Initialze AP20 internal PHY */
	/* ENABLE up to 16 PCIE lanes */
	pci_tegra_pads_writel(0x0, NV_PROJ__PCIE2_PADS_CTL_SEL_1);

	/* override IDDQ to 1 on all 4 lanes */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_CTL_1);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, IDDQ_1L, 1, reg);
	pci_tegra_pads_writel(reg, NV_PROJ__PCIE2_PADS_CTL_1);

	/* set up PHY PLL inputs select PLLE output as refclock */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_PLL_CTL1);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_REFCLK_SEL,
		NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_REFCLK_SEL_INTERNAL_CML, reg);

	/* set TX ref sel to div10 (not div5) */
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_TXCLKREF_SEL,
		NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_TXCLKREF_SEL_DIV10, reg);
	pci_tegra_pads_writel(reg, NV_PROJ__PCIE2_PADS_PLL_CTL1);

	/* take PLL out of reset  */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_PLL_CTL1);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_RST_B4SM,
		NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_RST_B4SM_DEASSERT, reg);
	pci_tegra_pads_writel(reg, NV_PROJ__PCIE2_PADS_PLL_CTL1);

	/* Hack, set the clock voltage to the DEFAULT provided by hw folks.
	 * This doesn't exist in the documentation
	 * */
	reg = 0xFA5CFA5C;
	pci_tegra_pads_writel(reg, 0xc8);

	/* Wait for the PLL to lock */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_PLL_CTL1);
	while (NVPCIE_DRF_VAL(PADS, PLL_CTL1, PLL_LOCKDET, reg)
	       != NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_LOCKDET_LOCKED) {
		reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_PLL_CTL1);
	}

	/* turn off IDDQ override */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_CTL_1);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, IDDQ_1L, 0, reg);
	pci_tegra_pads_writel(reg, NV_PROJ__PCIE2_PADS_CTL_1);

	/* ENABLE TX/RX data */
	reg = pci_tegra_pads_reedl(NV_PROJ__PCIE2_PADS_CTL_1);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, TX_DATA_EN_1L,
		NV_PROJ__PCIE2_PADS_CTL_1_TX_DATA_EN_1L_ENABLE, reg);
	reg = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, RX_DATA_EN_1L,
		NV_PROJ__PCIE2_PADS_CTL_1_RX_DATA_EN_1L_ENABLE, reg);
	pci_tegra_pads_writel(reg, NV_PROJ__PCIE2_PADS_CTL_1);

	irq = INT_PCIE_INTR;
	if (request_irq(irq, pci_tegra_isr, IRQF_SHARED, "PCIE", s_hRmGlobal)) {
		pr_err("%s: Cannot register IRQ %u: %d\n",
		       __func__, irq, ret);
		goto done;
	}
	set_irq_flags(irq, IRQF_VALID);

	/* setup the AFI address translations */
	pci_tegra_setup_translations();

	/* Take the PCIe interface module out of reset to start the PCIe
	 * training  sequence */
	tegra_periph_reset_deassert(clk_pciex);

	/* Finally enable PCIe */
	reg = pci_tegra_afi_readl(AFI_CONFIGURATION_0);
	reg = reg | AFI_CONFIGURATION_0_EN_FPCI_DEFAULT_MASK;
	pci_tegra_afi_writel(reg, AFI_CONFIGURATION_0);

	pci_tegra_rp0_up = pci_tegra_check_rp(0);
	pci_tegra_rp1_up = pci_tegra_check_rp(1);

	if(!pci_tegra_rp0_up && !pci_tegra_rp1_up) {
		pr_info("%s: no PCIE devices attached\n", __func__);
		pci_tegra_device_attached = false;
		goto done;
	}
	pci_tegra_device_attached = true;

	/* Enable PCIe interrupts */
	reg = 0;
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_INI_SLVERR, 1);
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_INI_DECERR, 1);
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_SLVERR, 1);
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_DECERR, 1);
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_WRERR, 1);
	reg |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_DFPCI_DECERR, 1);
	pci_tegra_afi_writel(reg, AFI_AFI_INTR_ENABLE_0);
	pci_tegra_afi_writel(0xffffffff, AFI_SM_INTR_ENABLE_0);

	reg = 0;
	reg |= NV_DRF_NUM(AFI, INTR_MASK, INT_MASK, 1);
	/* FIXME: No MSI for now */
	reg |= NV_DRF_NUM(AFI, INTR_MASK, MSI_MASK, 0);
	pci_tegra_afi_writel(reg, AFI_INTR_MASK_0);

	pci_tegra_enumerate();
	ret = 1;
done:
	pr_info("%s exiting: %s\n", __func__, ret ? "SUCCESS" : "FAILURE");
	if (!IS_ERR_OR_NULL(clk_pciex))
		clk_put(clk_pciex);
	if (!IS_ERR_OR_NULL(clk_pcie)) {
		if (!ret)
			clk_disable(clk_pcie);
		clk_put(clk_pcie);
	}
	if (!IS_ERR_OR_NULL(regulator)) {
		if (!ret)
			regulator_disable(regulator);
		regulator_put(regulator);
	}
	if (!ret && pci_tegra_regs) {
		iounmap(pci_tegra_regs);
		pci_tegra_regs = NULL;
	}
	return ret;
}

static struct pci_bus __init *pci_tegra_scan_bus(int nr,
	struct pci_sys_data *sys)
{
	if ((nr == 0) || (nr == 1))
		return pci_scan_bus(sys->busnr, &pci_tegra_ops, sys);

	return NULL;
}

int pci_tegra_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int ret = 0;
	pci_tegra_read_conf(dev->bus, dev->devfn, PCI_INTERRUPT_LINE,
		sizeof(int), &ret);
	return (ret & 0x000000ff);
}

static struct hw_pci pci_tegra_data __initdata = {
	.nr_controllers		= 2,
	.preinit		= pci_tegra_preinit,
	.setup			= pci_tegra_setup,
	.scan			= pci_tegra_scan_bus,
	.swizzle		= pci_std_swizzle,
	.map_irq		= pci_tegra_map_irq,
};

void __init tegra_pcie_init(void)
{
	pci_common_init(&pci_tegra_data);
}

/*
 *  PCIe support functions
 */

static void pci_tegra_setup_translations(void)
{
	u32 fpci_bar;
	u32 size;
	u32 axi_address;

	/* Bar 0: Config Bar */
	fpci_bar = ((u32)0xfdff << 16);
	size = PCIE_CONFIG_SIZE;
	axi_address = TEGRA_PCIE_BASE + PCIE_CONFIG_OFFSET;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR0_START_0);
	pci_tegra_afi_writel(size>>12, AFI_AXI_BAR0_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR0_0);

	/* Bar 1: Extended config Bar */
	fpci_bar = ((u32)0xfe1 << 20);
	size = PCIE_EXTENDED_CONFIG_SIZE;
	axi_address = TEGRA_PCIE_BASE + PCIE_EXTENDED_CONFIG_OFFSET;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR1_START_0);
	pci_tegra_afi_writel(size >>12, AFI_AXI_BAR1_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR1_0);

	/* Bar 2: Downstream IO bar */
	fpci_bar = ((__u32)0xfdfc << 16);
	size = PCIE_DOWNSTREAM_IO_SIZE;
	axi_address = TEGRA_PCIE_BASE + PCIE_DOWNSTREAM_IO_OFFSET;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR2_START_0);
	pci_tegra_afi_writel(size>>12, AFI_AXI_BAR2_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR2_0);

	/* Bar 3: Pre-fetchable memory BAR */
	/* Bits 39:12 of 40 bit FPCI address goes to bits 31:4 */
	fpci_bar = (((FPCI_PREFETCH_MEMORY_OFFSET >> 12) & 0x0FFFFFFF) << 4);
	fpci_bar |= 0x1;
	size = PCIE_PREFETCH_MEMORY_SIZE;
	axi_address = TEGRA_PCIE_BASE + PCIE_PREFETCH_MEMORY_OFFSET;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR3_START_0);
	pci_tegra_afi_writel(size >> 12, AFI_AXI_BAR3_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR3_0);

	/* Bar 4: Non pre-fetchable memory BAR */
	/* Bits 39:12 of 40 bit FPCI address goes to bits 31:4 */
	fpci_bar = (((FPCI_NON_PREFETCH_MEMORY_OFFSET >> 12)
		& 0x0FFFFFFF) << 4);
	fpci_bar |= 0x1;
	size = PCIE_NON_PREFETCH_MEMORY_SIZE;
	axi_address = TEGRA_PCIE_BASE
		+ PCIE_NON_PREFETCH_MEMORY_OFFSET;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR4_START_0);
	pci_tegra_afi_writel(size >> 12, AFI_AXI_BAR4_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR4_0);

	/* Bar 5: NULL out the remaining BAR as it is not used */
	fpci_bar = 0;
	size = 0;
	axi_address = 0;
	pci_tegra_afi_writel(axi_address, AFI_AXI_BAR5_START_0);
	pci_tegra_afi_writel(size >> 12, AFI_AXI_BAR5_SZ_0);
	pci_tegra_afi_writel(fpci_bar, AFI_FPCI_BAR5_0);

	/* map all upstream transactions as uncached */
	pci_tegra_afi_writel(FPCI_SYSTEM_MEMORY_OFFSET, AFI_CACHE_BAR0_ST_0);
	pci_tegra_afi_writel(0, AFI_CACHE_BAR0_SZ_0);
	pci_tegra_afi_writel(0, AFI_CACHE_BAR1_ST_0);
	pci_tegra_afi_writel(0, AFI_CACHE_BAR1_SZ_0);

	/* Map MSI bar */
	pci_tegra_afi_writel(0, AFI_MSI_FPCI_BAR_ST_0);
	pci_tegra_afi_writel(0, AFI_MSI_BAR_SZ_0);
	pci_tegra_afi_writel(0, AFI_MSI_AXI_BAR_ST_0);
	pci_tegra_afi_writel(0, AFI_MSI_BAR_SZ_0);
}

static irqreturn_t pci_tegra_isr(int irq, void *arg)
{
	u32 intr_info, intr_extended_info;
	irqreturn_t ret = IRQ_HANDLED;

	intr_info = pci_tegra_afi_readl(AFI_INTR_CODE_0);
	intr_info = NV_DRF_VAL(AFI, INTR_CODE, INT_CODE, intr_info);
	intr_extended_info = pci_tegra_afi_readl(AFI_INTR_SIGNATURE_0);

	/* pr_err("+pci_tegra_isr\n"); */

	switch (intr_info) {
	case 6: /* legacy */
		ret = IRQ_NONE;
		break;
	case 1: /* SLVERR */
		pr_err("pci_tegra_isr: AXI slave error\n");
		break;
	case 2: /* DECERR */
		pr_err("pci_tegra_isr: AXI decode error\n");
		break;
	case 3: /* PCIE target abort */
		pr_err("pci_tegra_isr: Target abort\n");
		break;
	case 4: /* PCIE master abort */
		/* Don't print this, as this error is a common error during
		 * enumeration.
		 */
		/* pr_err("pci_tegra_isr: Master abort\n"); */
		break;
	case 5: /* Bufferable write to non-posted write */
		pr_err("pci_tegra_isr: Invalid write"
			" - Bufferable write to non-posted region\n");
		break;
	case 7: /* Response address mapping error */
		pr_err("pci_tegra_isr: Response decoding error \n");
		break;
	case 8: /* Response address mapping error */
		pr_err("pci_tegra_isr: AXI response decoding error\n");
		break;
	case 9: /* PCIE timeout */
		pr_err("pci_tegra_isr: Transcation timeout\n");
		break;
	default:
		pr_err("pci_tegra_isr: Unknown interrupt\n");
		break;
	}

	/* Clear the interrupt code register to sample the next interrupt */
	pci_tegra_afi_writel(0, AFI_INTR_CODE_0);

	/* pr_err("-pci_tegra_isr\n"); */
	return ret;
}


/*
 *	FIXME: If there are no PCIe cards attached, then calling this function
 *	can result in the increase of the bootup time as there are big timeout
 *	loops.
 */
static bool pci_tegra_check_rp(int rp)
{
#define PCI_TEGRA_LINKUP_TIMEOUT	50 /* i.e .2 seconds */
	u32 reg;
	int retry_count = 0;
	int loop_counter;

	BUG_ON(rp != 0 && rp != 1);
retry:
	if (retry_count > 1 ) {
		pr_err("pci_tegra_check_rp: RP %d Failed\n", rp);
		return false;
	}

	if (retry_count != 0) {
		u32 offset=0;

		/* Reset before retrying again. */
		if (rp == 0) offset = AFI_PEX0_CTRL_0;
		else if (rp == 1) offset = AFI_PEX1_CTRL_0;
		else BUG();

		BUG_ON(AFI_PEX0_CTRL_0_PEX0_RST_L_SHIFT !=
			AFI_PEX1_CTRL_0_PEX1_RST_L_SHIFT);

		/* Pulse the PEX reset */
		reg = pci_tegra_afi_readl(offset);
		reg  = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 1, reg);
		pci_tegra_afi_writel(reg, offset);

		msleep(100);

		reg = pci_tegra_afi_readl(offset);
		reg  = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 0, reg);
		pci_tegra_afi_writel(reg, offset);
	}

	loop_counter = PCI_TEGRA_LINKUP_TIMEOUT;

	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_VEND_XP, rp);
	while (loop_counter && (NVPCIE_DRF_VAL(RP, VEND_XP, DL_UP, reg)!= 1)) {
		msleep(1);
		reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_VEND_XP, rp);
		loop_counter --;
	}

	if (!loop_counter)  {
		retry_count++;
		pr_err("pci_tegra_check_rp: "
			"RP %d LINK is not up...retrying...\n", rp);
		goto retry;
	}

	loop_counter = PCI_TEGRA_LINKUP_TIMEOUT;

	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_LINK_CONTROL_STATUS, rp);
	reg = NVPCIE_DRF_VAL(RP, LINK_CONTROL_STATUS, LINKSTAT, reg);
	while (loop_counter && ((reg & 0x2000) != 0x2000)) {
		msleep(1);
		reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_LINK_CONTROL_STATUS,
			rp);
		reg = NVPCIE_DRF_VAL(RP, LINK_CONTROL_STATUS, LINKSTAT, reg);
		pr_err("pci_tegra_check_rp: "
			"RP %d LINK status 0x%x...retrying...\n", rp, reg);
		loop_counter --;
	}

	if (!loop_counter)  {
		retry_count++;
		goto retry;
	}

	pr_info("pci_tegra_check_rp: RP %d success\n", rp);
	return true;
}
