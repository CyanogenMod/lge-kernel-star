/*
 *  arch/arm/mach-tegra/include/mach/pci.h
 *
 *  Header file containing constants for the tegra PCIe driver.
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

#ifndef __MACH_TEGRA_PCI_H

#include <linux/pci.h>

#include "nvrm_drf.h"
#include "ap20/dev_ap_pcie2_root_port.h"
#include "ap20/dev_ap_pcie2_pads.h"
#include "ap20/arafi.h"

extern void __iomem * volatile pci_tegra_regs;

/*
 * AXI address map for the PCIe aperture.  AP20, defines 1GB in the AXI
 *  address map for PCIe.
 *
 *  That address space is split into different regions, with sizes and
 *  offsets as follows. Exepct for the Register space, SW is free to slice the
 *  regions as it chooces.
 *
 *  The split below seems to work fine for now.
 *
 *  0x8000_0000 to 0x80ff_ffff - Register space          16MB.
 *  0x8100_0000 to 0x81ff_ffff - Config space            16MB.
 *  0x8200_0000 to 0x82ff_ffff - Extended config space   16MB.
 *  0x8300_0000 to 0x83ff_ffff - Downstream IO space
 *   ... Will be filled with other BARS like MSI/upstream IO etc.
 *  0x9000_0000 to 0x9fff_ffff - non-prefetchable memory aperture
 *  0xa000_0000 to 0xbfff_ffff - Prefetchable memory aperture
 *
 *  Config and Extended config sizes are choosen to support
 *  maximum of 256 devices,
 *  which is good enough for all the AP20 use cases.
 * */

#define PCIE_REGS_SIZE			0x01000000UL
#define PCIE_CONFIG_OFFSET		PCIE_REGS_SIZE
#define PCIE_CONFIG_SIZE		0x01000000UL
#define PCIE_EXTENDED_CONFIG_OFFSET	(PCIE_CONFIG_SIZE + PCIE_CONFIG_OFFSET)
#define PCIE_EXTENDED_CONFIG_SIZE	0x01000000UL
#define PCIE_DOWNSTREAM_IO_OFFSET	(PCIE_EXTENDED_CONFIG_SIZE + \
					PCIE_EXTENDED_CONFIG_OFFSET)
#define PCIE_DOWNSTREAM_IO_SIZE		0x00100000UL

#define PCIE_NON_PREFETCH_MEMORY_OFFSET	0x10000000UL
#define PCIE_NON_PREFETCH_MEMORY_SIZE	0x10000000UL
#define PCIE_PREFETCH_MEMORY_OFFSET	(PCIE_NON_PREFETCH_MEMORY_OFFSET + \
					PCIE_NON_PREFETCH_MEMORY_SIZE)
#define PCIE_PREFETCH_MEMORY_SIZE	0x20000000UL

/* PCIe registers can be classified into 4 regions.
 *
 * 1. AFI registers - AFI is a wrapper between PCIE and ARM AXI bus. These
 * registers define the address translation registers, interrupt registers and
 * some configuration (a.k.a CYA) registers.
 * 2. PAD registers - PAD control registers which are inside the PCIE CORE.
 * 3. Configuration 0 and Configuration 1 registers - These registers are PCIe
 * configuration registers of Root port 0 and root port 1.
 *
 * Check the PcieRegType enumeration for the list of Registers banks inside the
 * PCIE aperture.
 *
 * */
#define NV_PCIE_AXI_AFI_REGS_OFSET	0x3800UL
#define NV_PCIE_AXI_PADS_OFSET		0x3000UL
#define NV_PCIE_AXI_RP_T0C0_OFFSET	0x0000UL
#define NV_PCIE_AXI_RP_T0C1_OFFSET	0x1000UL

/* During the boot only registers/config and extended config apertures are
 * mapped. Rest are mapped on demand by the PCI device drivers.
 */
#define PCI_TEGRA_IOMAPPED_REG_APERTURE_SIZE	\
	(PCIE_REGS_SIZE + PCIE_CONFIG_SIZE + PCIE_EXTENDED_CONFIG_SIZE)

/*
 *  PCI address map for memory mapped devices. Still using 32-bit aperture.
 *
 *  1GB for the system memory.
 *  Everything mapped as cpu physical = pci
 *
 */
#define FPCI_SYSTEM_MEMORY_OFFSET           0x0UL
#define FPCI_SYSTEM_MEMORY_SIZE             0x40000000UL
#define FPCI_NON_PREFETCH_MEMORY_OFFSET     0x90000000UL
#define FPCI_NON_PREFETCH_MEMORY_SIZE       PCIE_NON_PREFETCH_MEMORY_SIZE
#define FPCI_PREFETCH_MEMORY_OFFSET         (FPCI_NON_PREFETCH_MEMORY_OFFSET+ \
						FPCI_NON_PREFETCH_MEMORY_SIZE)
#define FPCI_PREFETCH_MEMORY_SIZE           0x40000000UL




/* PCIE DRF macros to read and write PRI registers */

/** NVPCIE_DRF_DEF - define a new register value.

	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param c defined value for the field
 */
#define NVPCIE_DRF_DEF(d,r,f,c) \
	((NV_PROJ__PCIE2_##d##_##r##_##f##_##c)  \
	<< NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_DRF_NUM - define a new register value.

	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param n numeric value for the field
 */
#define NVPCIE_DRF_NUM(d,r,f,n) \
	(((n)& NV_FIELD_MASK(NV_PROJ__PCIE2_##d##_##r##_##f)) << \
	NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_DRF_VAL - read a field from a register.

	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param v register value
 */
#define NVPCIE_DRF_VAL(d,r,f,v) \
	(((v)>> NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f)) & \
        NV_FIELD_MASK(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_FLD_SET_DRF_NUM - modify a register field.

	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param n numeric field value
	@param v register value
 */
#define NVPCIE_FLD_SET_DRF_NUM(d,r,f,n,v) \
	((v & ~NV_FIELD_SHIFTMASK(NV_PROJ__PCIE2_##d##_##r##_##f)) | \
	NVPCIE_DRF_NUM(d,r,f,n))

/** NVPCIE_FLD_SET_DRF_DEF - modify a register field.

	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param c defined field value
	@param v register value
 */
#define NVPCIE_FLD_SET_DRF_DEF(d,r,f,c,v) \
    (((v) & ~NV_FIELD_SHIFTMASK(NV_PROJ__PCIE2_##d##_##r##_##f)) | \
        NVPCIE_DRF_DEF(d,r,f,c))

/** NVPCIE_RESETVAL - get the reset value for a register.

	@param d register domain (hardware block)
	@param r register name
 */
#define NVPCIE_RESETVAL(d,r)    (d##_##r##_0_RESET_VAL)

/* Register access inline functions */

static inline void pci_tegra_afi_writel(u32 value,unsigned long offset)
{
	writel(value, offset + NV_PCIE_AXI_AFI_REGS_OFSET + pci_tegra_regs);
}

static inline void pci_tegra_rp_writel(u32 value, unsigned long offset, int rp)
{
	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	writel(value, offset + pci_tegra_regs);
}

static inline void pci_tegra_rp_writew(u16 value, unsigned long offset, int rp)
{
	u32 reg;

	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	reg = readl((offset & ~0x3) + pci_tegra_regs);
	reg &= ~(0xffff << ((offset & 0x3) * 8));
	reg |= (u32)value << ((offset & 0x3) * 8);
	writel(reg, (offset & ~0x3) + pci_tegra_regs);
}

static inline void pci_tegra_rp_writeb(u8 value, unsigned long offset, int rp)
{
	u32 reg;

	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	reg = readl((offset & ~0x3) + pci_tegra_regs);
	reg &= ~(0xff << ((offset & 0x3) * 8));
	reg |= (u32)value << ((offset & 0x3) * 8);
	writel(reg, (offset & ~0x3) + pci_tegra_regs);
}

static inline void pci_tegra_pads_writel(u32 value, unsigned long offset)
{
	writel(value, offset + NV_PCIE_AXI_PADS_OFSET + pci_tegra_regs);
}

static inline u32 pci_tegra_afi_readl(unsigned long offset)
{
	return readl(offset + NV_PCIE_AXI_AFI_REGS_OFSET + pci_tegra_regs);
}

static inline u32 pci_tegra_rp_readl(unsigned long offset, int rp)
{
	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	return readl(offset + pci_tegra_regs);
}

static inline u16 pci_tegra_rp_readw(unsigned long offset, int rp)
{
	u32 val;

	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	val = readl((offset & ~0x3) + pci_tegra_regs);
	val >>=  8 * (offset & 3);
	val &= 0xffff;

	return (u16)val;
}

static inline u8 pci_tegra_rp_readb(unsigned long offset, int rp)
{
	u32 val;

	BUG_ON(rp != 0 && rp != 1);

	if (rp == 0) offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
	if (rp == 1) offset += NV_PCIE_AXI_RP_T0C1_OFFSET;

	val = readl((offset & ~0x3) + pci_tegra_regs);
	val >>=  8 * (offset & 3);
	val &= 0xff;

	return (u8)val;
}

static inline u32 pci_tegra_pads_reedl(unsigned long offset)
{
	return readl(offset + NV_PCIE_AXI_PADS_OFSET + pci_tegra_regs);
}

static inline bool pci_tegra_is_rp(u32 bus_number, int *rp)
{
	if (bus_number == pci_tegra_rp_readb(PCI_PRIMARY_BUS, 0)) {
		*rp = 0;
		return true;
	} else if (bus_number == pci_tegra_rp_readb(PCI_PRIMARY_BUS, 1)) {
		*rp = 1;
		return true;
	} else
		return false;
}

// return true if this is the first device on an (active) rootport
static inline bool pci_tegra_is_rp_first_dev(u32 bus_number, int rp)
{
	u8 primary, secondary;

	BUG_ON((rp != 0) && (rp != 1));

	primary = pci_tegra_rp_readb(PCI_PRIMARY_BUS, rp);
	secondary = pci_tegra_rp_readb(PCI_SECONDARY_BUS, rp);
	if ((primary < secondary) && (bus_number == secondary))
		return true;

	return false;
}

/*
 *	Given the bus number, devfn and the offset this API returns the mapped
 *	address of the config space.
 */
static inline void __iomem *pci_tegra_config_addr(u8 bus_number,
	u32 devfn, u32 where)
{
	void *addr;
	u32 function;
	u32 device;

	function = PCI_FUNC(devfn);
	device = PCI_SLOT(devfn);

	addr = pci_tegra_regs;
	addr += (where < 256) ? PCIE_CONFIG_OFFSET
		   : PCIE_EXTENDED_CONFIG_OFFSET;
	addr += bus_number << 16;
	addr += device << 11;
	addr += function << 8;
	addr += where;
	return addr;
}

void pci_tegra_enumerate(void);

#endif
