/*
 *  arch/arm/mach-tegra/pci-enum.c
 *
 *	Code to enumerate the PCI devices on the PCI bus. Unlike x86 we cannot
 *	rely on BIOS to allocate the PCIe resources for the devices.
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

#include <mach/pci.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <linux/delay.h>
#include <linux/ioport.h>

struct pci_tegra_device
{
	/* Bus number */
	u8 bus;

	/* Device + function encoding.
	 *	Use macros PCI_DEVFN/PCI_SLOT/PCI_FUNC to encode and decode
	 * */
	u32 devfn;

	/* Secondary bus nummber. Non-zero only for bridge devices. */
	u32 sec_bus;

	/* Subordinate bus number. Non-zero only for the bridge devices. */
	u32 sub_bus;

	/* Device ID/vendor ID of the PCI device/bridge.
	 * Upper 16 bits are device ID and lower 16 bits are vendor ID.
	 */
	u32 id;

	/*	For a bridge device only 3 bars are used.
	 */
#define PCI_BRIDGE_IO_RES	0
#define PCI_BRIDGE_MEM_RES	1
#define PCI_BRIDGE_PREFETCH_RES	2

	/* Here we are limiting to the standard PCI resources */
	struct resource res[PCI_STD_RESOURCE_END + 1];

	bool disabled;

	struct pci_tegra_device *parent;
	struct pci_tegra_device *next;
	struct pci_tegra_device *prev;
	struct pci_tegra_device *child;
	bool root_port;
};

#define TEGRA_MAX_PCI_DEVICES	64
static struct pci_tegra_device pci_devices[TEGRA_MAX_PCI_DEVICES];
static int max_devices;
static struct pci_tegra_device *pci_root;

static u32 pci_tegra_io_base;
static u32 pci_tegra_mem_base;
static u32 pci_tegra_prefetch_base;

static u32 pci_tegra_io_limt;
static u32 pci_tegra_mem_limit;
static u32 pci_tegra_prefetch_limit;

static void pci_tegra_print_device_tree(struct pci_tegra_device *dev);
static void pcie_scanbus(struct pci_tegra_device *dev_parent);
static void pci_tegra_allocate_resources(struct pci_tegra_device *dev);


static struct pci_tegra_device *alloc_pci_tegra_device(void)
{
	static u32 index = 0;
	struct pci_tegra_device *dev;

	if (index == 0)
		memset(pci_devices, 0, sizeof(pci_devices));

	dev = &pci_devices[index];
	index++;
	max_devices = index;
	return dev;
}

static inline void pci_conf_write8(u8 bus, u32 devfn, u32 where , u8 val)
{
	u32 addr;
	u32 temp;

	addr = (u32)pci_tegra_config_addr(bus, devfn, where);
	pr_err("Issuing pci_conf_write8 at addr 0x%x with data 0x%x\n",
		addr, val);

	temp = readl((addr & ~0x3));
	temp &= ~(0xff << ((addr & 0x3) * 8));
	temp |= (u32)val << ((addr & 0x3) * 8);
	writel(temp, (addr & ~0x3));
}

static inline void  pci_conf_write16(u8 bus, u32 devfn, u32 where, u16 val)
{
	u32 addr;
	u32 temp;

	BUG_ON(where & 0x1);

	addr = (u32)pci_tegra_config_addr(bus, devfn, where);
	pr_err("Issuing pci_conf_write16 at addr 0x%x with data 0x%x\n",
		addr, val);

	temp = readl((addr & ~0x3));
	temp &= ~(0xffff << ((addr& 0x3) * 8));
	temp |= (u32)val << ((addr & 0x3) * 8);
	writel(temp, (addr & ~0x3));
}

static inline void pci_conf_write32(u8 bus, u32 devfn, u32 where, u32 val)
{
	u32 addr;

	BUG_ON(where & 0x3);

	addr = (u32)pci_tegra_config_addr(bus, devfn, where);
	pr_err("Issuing pci_conf_write32 at addr 0x%x with data 0x%x\n",
		addr, val);
	writel(val, addr);
}

static inline u8 pci_conf_read8(u8 bus, u32 devfn, u32 where)
{
	u32 temp;
	u32 addr;

	addr = (u32)pci_tegra_config_addr(bus, devfn, where);
	pr_err("Issuing pci_conf_read8 at 0x%x\n", addr);
	temp = readl(addr & ~0x3);
	temp >>=  8 * (addr & 3);
	temp &= 0xff;
	pr_err("pci_conf_read8 at 0x%x = %d\n", addr, temp);

	return (u8)temp;
}

static u32 pci_conf_read32(u8 bus, u32 devfn, u32 where)
{
	u32 temp;

	BUG_ON(where & 0x3);

	pr_err("Issuing pci_conf_read32 at 0x%x\n",
		(u32)(pci_tegra_config_addr(bus, devfn, where)));

	temp = readl(pci_tegra_config_addr(bus, devfn, where));

	pr_err("pci_conf_read32 at 0x%x = %d\n", where, temp);
	return temp;
}

static void pcie_scanbus(struct pci_tegra_device *dev_parent)
{
	u8 subordinate_bus;
	u8 hdr_type;
	u8 next_bus_number;
	u32 device = 0;
	u32 id;
	struct pci_tegra_device *dev;
	u32 retry_count;

	next_bus_number = dev_parent->sec_bus;

next_device:
	retry_count = 6;
	if (device == 0x20) {
		/* Termination condition: Max number of devices reached.
		 * PCIe bus segment can only have 32 devices.
		 * */
		dev_parent->sub_bus = next_bus_number;
		if (!dev_parent->root_port) {
			/* Change the subordinate bus-number to the actual
			 * value of all buses on the hierarcy.
			*
			* Do this execpt for the root port.
			*/
			pci_conf_write8(dev_parent->bus, dev_parent->devfn,
				PCI_SUBORDINATE_BUS, next_bus_number);
		}
		return;
	}

	if (dev_parent->root_port && device != 0) {
		/* Sepcial Exit condition for root port.
		 * Root port only connect to one bridge or device.
		 */
		dev_parent->sub_bus = dev_parent->sec_bus;
		return;
	}

	while (--retry_count) {
		id = pci_conf_read32(dev_parent->sec_bus,
			PCI_DEVFN(device, 0), 0);
		if (id != 0xFFFFFFFF)
		{
		/* Found a valid device, break. Otherwise, retry a couple of
		 * times. It is possible that the bridges can take some time
		 * to settle and it will take couple of transcations to find
		 * the devcies behind the bridge.
		 * */
		/* FIXME: What should be the delay? */
			msleep(100);
			break;
		}
	}
	if (id == 0xFFFFFFFF) {
		/* Invalid device. Skip that one and look for next device */
		device++;
		goto next_device;
	}

	dev = alloc_pci_tegra_device();

	/* Fill the device information */
	dev->parent = dev_parent;
	dev->id = id;
	dev->bus = dev_parent->sec_bus;
	dev->devfn = PCI_DEVFN(device, 0);
	if (dev_parent->child == NULL) {
		dev_parent->child = dev;
		dev->prev = NULL;
	} else {
		/* Add dev to the list of devices on the same bus */
		struct pci_tegra_device *temp;

		temp = dev_parent->child;
		BUG_ON(temp != NULL);
		while (temp->next != NULL)
		temp = temp->next;
		temp->next = dev;
		dev->prev = temp;
	}

	hdr_type = pci_conf_read8(dev->bus, dev->devfn, PCI_HEADER_TYPE);
	if ((hdr_type & 0x7f) == 0x1) {
		/* Bridge device */

		/* Temporarily assign 0xff for the subordinate bus number as
		 * we don't * know how many devices are present behind this
		 * bridge.
		 * */
		subordinate_bus = 0xff;
		dev->sec_bus = next_bus_number + 1;

		pci_conf_write8(dev->bus, dev->devfn, PCI_PRIMARY_BUS,
			dev_parent->sec_bus);
		pci_conf_write8(dev->bus, dev->devfn, PCI_SECONDARY_BUS,
			dev->sec_bus);
		pci_conf_write8(dev->bus, dev->devfn, PCI_SUBORDINATE_BUS,
			subordinate_bus);

		/* Scan all the buses behind this bridge */
		pcie_scanbus(dev);

		next_bus_number = dev->sub_bus;
	} else if ((hdr_type & 0x7f) == 0x0) {

		/* PCI endpoint - Can be single function or multie function */
		pr_info("PCI endpoint (0x%x) is on bus = %d, device = %d\n",
			id, dev_parent->sec_bus, device);

	} else if ((hdr_type & 0x7f) == 0x2) {
		/* PC card device - Not handled */
		BUG();
	} else {
		BUG();
	}
	device++;
	goto next_device;
}

static void pci_tegra_enumerate_root_port(int rp)
{
	struct pci_tegra_device *root;
	u32 reg;

	root = alloc_pci_tegra_device();

	if (pci_root) {
		pci_root->next = root;
		root->bus = pci_root->sub_bus + 1;
	} else {
		pci_root = root;
		root->bus = 0;
	}

	root->sec_bus = root->bus + 1;
	root->root_port = true;
	/* Set the Inital value to the max bus number */
	root->sub_bus = 0xff;
	root->id = pci_tegra_rp_readl(0, rp);

	pci_tegra_rp_writeb(root->bus, PCI_PRIMARY_BUS, rp);
	pci_tegra_rp_writeb(root->sec_bus, PCI_SECONDARY_BUS, rp);
	pci_tegra_rp_writeb(root->sub_bus, PCI_SUBORDINATE_BUS, rp);

	/* Just assigns the bus numbers and sets up the SW hirerarchy */
	pcie_scanbus(root);

	/* Write the udpated root port subordinate bus number */
	pci_tegra_rp_writeb(root->sub_bus, PCI_SUBORDINATE_BUS, rp);

	pci_tegra_allocate_resources(root);

	/* IO base and limits */
	reg = root->res[PCI_BRIDGE_IO_RES].start;
	reg = ALIGN(reg, 0x1000);
	pci_tegra_rp_writeb((((reg & 0xf000) >> 8) | PCI_IO_RANGE_TYPE_32),
		PCI_IO_BASE, rp);
	pci_tegra_rp_writew(reg>>16, PCI_IO_BASE_UPPER16, rp);

	reg = root->res[PCI_BRIDGE_IO_RES].end;
	reg = ALIGN(reg, 0x1000) - 1;
	pci_tegra_rp_writeb((((reg & 0xf000) >> 8) | PCI_IO_RANGE_TYPE_32),
		PCI_IO_LIMIT, rp);
	pci_tegra_rp_writew(reg>>16, PCI_IO_LIMIT_UPPER16, rp);

	/* Memory base and limits */
	if (root->res[PCI_BRIDGE_MEM_RES].start != root->res[PCI_BRIDGE_MEM_RES].end) {
		reg = root->res[PCI_BRIDGE_MEM_RES].start;
		reg = ALIGN(reg, 0x100000);
		pci_tegra_rp_writew(reg >> 16, PCI_MEMORY_BASE, rp);
		reg = root->res[PCI_BRIDGE_MEM_RES].end;
		reg = ALIGN(reg, 0x100000) - 1;
		pci_tegra_rp_writew(reg >> 16, PCI_MEMORY_LIMIT, rp);
	} else {
		pci_tegra_rp_writew(0xffff, PCI_MEMORY_BASE, rp);
		pci_tegra_rp_writew(0x0000, PCI_MEMORY_LIMIT, rp);
	}

	/* Prefetch base and limit - 32 bit addressing */
	if (root->res[PCI_BRIDGE_PREFETCH_RES].start != root->res[PCI_BRIDGE_PREFETCH_RES].end) {
		reg = root->res[PCI_BRIDGE_PREFETCH_RES].start;
		reg = ALIGN(reg, 0x100000);
		pci_tegra_rp_writew(reg >> 16, PCI_PREF_MEMORY_BASE, rp);
		reg = root->res[PCI_BRIDGE_PREFETCH_RES].end;
		reg = ALIGN(reg, 0x100000) - 1;
		pci_tegra_rp_writew(reg >> 16, PCI_PREF_MEMORY_LIMIT, rp);
	} else {
		pci_tegra_rp_writew(0xffff, PCI_PREF_MEMORY_BASE, rp);
		pci_tegra_rp_writew(0, PCI_PREF_MEMORY_LIMIT, rp);
	}
	pci_tegra_rp_writel(0, PCI_PREF_BASE_UPPER32, rp);
	pci_tegra_rp_writel(0, PCI_PREF_LIMIT_UPPER32, rp);

	reg = 0;
	reg |= PCI_COMMAND_IO;
	reg |= PCI_COMMAND_MEMORY;
	reg |= PCI_COMMAND_MASTER;
	reg |= PCI_COMMAND_SERR;
	pci_tegra_rp_writew(reg, PCI_COMMAND, rp);
}

static void pci_tegra_setup_pci_bridge(struct pci_tegra_device *dev)
{
	u32 reg;

	dev->res[PCI_BRIDGE_IO_RES].end = pci_tegra_io_base;
	dev->res[PCI_BRIDGE_MEM_RES].end = pci_tegra_mem_base;
	dev->res[PCI_BRIDGE_PREFETCH_RES].end =
		pci_tegra_prefetch_base;

	/* Only set here for the non-root port devices */
	if (dev->root_port)
		return;

	/* IO base and limits */
	reg = dev->res[PCI_BRIDGE_IO_RES].start;
	reg = ALIGN(reg, 0x1000);
	pci_conf_write8(dev->bus, dev->devfn, PCI_IO_BASE,
		(((reg & 0xf000) >> 8) | PCI_IO_RANGE_TYPE_32));
	pci_conf_write16(dev->bus, dev->devfn, PCI_IO_BASE_UPPER16, reg>>16);

	reg = dev->res[PCI_BRIDGE_IO_RES].end;
	reg = ALIGN(reg, 0x1000);
	pci_conf_write8(dev->bus, dev->devfn, PCI_IO_LIMIT,
		(((reg & 0xf000) >> 8) | PCI_IO_RANGE_TYPE_32));
	pci_conf_write16(dev->bus, dev->devfn, PCI_IO_LIMIT_UPPER16, reg>>16);

	/* Memory base and limits */
	if (dev->res[PCI_BRIDGE_MEM_RES].start != dev->res[PCI_BRIDGE_MEM_RES].end) {
		reg = dev->res[PCI_BRIDGE_MEM_RES].start;
		reg = ALIGN(reg, 0x100000);
		pci_conf_write16(dev->bus, dev->devfn, PCI_MEMORY_BASE, reg >> 16);

		reg = dev->res[PCI_BRIDGE_MEM_RES].end;
		reg = ALIGN(reg, 0x100000);
		pci_conf_write16(dev->bus, dev->devfn, PCI_MEMORY_LIMIT, reg >> 16);
	} else {
		pci_conf_write16(dev->bus, dev->devfn, PCI_MEMORY_BASE, 0xffff);
		pci_conf_write16(dev->bus, dev->devfn, PCI_MEMORY_LIMIT, 0);
	}

	/* Prefetch base and limit - 32 bit addressing */
	if (dev->res[PCI_BRIDGE_PREFETCH_RES].start != dev->res[PCI_BRIDGE_PREFETCH_RES].end) {
		reg = dev->res[PCI_BRIDGE_PREFETCH_RES].start;
		reg = ALIGN(reg, 0x100000);
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_MEMORY_BASE,
			reg >> 16);
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_BASE_UPPER32, 0);

		reg = dev->res[PCI_BRIDGE_PREFETCH_RES].end;
		reg = ALIGN(reg, 0x100000);
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_MEMORY_LIMIT,
			reg >> 16);
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_LIMIT_UPPER32, 0);
	} else {
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_MEMORY_BASE, 0xffff);
		pci_conf_write16(dev->bus, dev->devfn, PCI_PREF_MEMORY_LIMIT, 0);
	}

	reg = 0;
	reg |= PCI_COMMAND_IO;
	reg |= PCI_COMMAND_MEMORY;
	reg |= PCI_COMMAND_MASTER;
	reg |= PCI_COMMAND_SERR;
	pci_conf_write16(dev->bus, dev->devfn, PCI_COMMAND, reg);

	pci_conf_write8(dev->bus, dev->devfn, PCI_INTERRUPT_LINE, INT_PCIE_INTR);
	pci_conf_write8(dev->bus, dev->devfn, PCI_INTERRUPT_PIN, 0xa);
}

static void pci_tegra_setup_pci_device(struct pci_tegra_device *dev)
{
	u8 flags;
	u32 bar_index;
	u32 reg;
	u32 addr;

	for (bar_index = 0x0; bar_index  <= PCI_STD_RESOURCE_END;
		bar_index ++) {
		u32 size;
		pci_conf_write32(dev->bus, dev->devfn, bar_index * 4
			+ PCI_BASE_ADDRESS_0, 0xFFFFFFFFUL);

		size = pci_conf_read32(dev->bus, dev->devfn, bar_index * 4
			+ PCI_BASE_ADDRESS_0);

		if (size == 0xFFFFFFFFUL) continue;
		if (size == 0) continue;  /* A broken device? */
		flags = (size & 0x000f);

		/* Size align the addr and write that BAR offset */
		if (flags & 0x1) {
			size &= ~0xF;   /* Ignore the last 4 bits */
			/* some devices hardwire the high bits of IO bars to 0
			 * So, ignore those bits.
			 */
			size |= 0xffff0000;
			size = ~size + 1;   /* Do the 1's complement */

			addr = ALIGN(pci_tegra_io_base, size);

			if (addr + size > pci_tegra_io_limt) {
				pr_err("pci_tegra: "
					"Cannot asign IO res\n");
				continue;
			}
			dev->res[bar_index].flags = IORESOURCE_IO;
			dev->res[bar_index].start = addr;
			dev->res[bar_index].end = addr + size -1;

			pci_tegra_io_base =  addr + size;
		} else {
			size &= ~0xF;   /* Ignore the last 4 bits */
			size = ~size + 1;   /* Do the 1's complement */

			if (flags & 0x08) {
				addr = ALIGN(pci_tegra_mem_base, size);

				if (addr + size > pci_tegra_mem_limit) {
					pr_err("pci_tegra: "
						"Cannot asign mem res\n");
					continue;
				}

				dev->res[bar_index].flags = IORESOURCE_MEM;
				dev->res[bar_index].start = addr;
				dev->res[bar_index].end =
					dev->res[bar_index].start + size - 1;

				pci_tegra_mem_base = addr + size;
			} else {
				addr = ALIGN(pci_tegra_prefetch_base, size);

				if (addr + size > pci_tegra_prefetch_limit) {
					pr_err("pci_tegra: "
						"Cannot asign prefetch res\n");
					continue;
				}

				dev->res[bar_index].flags =
					IORESOURCE_MEM | IORESOURCE_PREFETCH;
				dev->res[bar_index].start = addr;
				dev->res[bar_index].end = addr + size - 1;

				pci_tegra_prefetch_base = addr + size;
			}
		}
		pci_conf_write32(dev->bus, dev->devfn, bar_index * 4
			+ PCI_BASE_ADDRESS_0, dev->res[bar_index].start);

		/* Handle 64 bit addresses by forcing to 32 bit addresses */
		if ((flags == 0x0c) || (flags==0x04)) {
			bar_index++;
			BUG_ON(bar_index > PCI_STD_RESOURCE_END);
			pci_conf_write32(dev->bus, dev->devfn, bar_index * 4
				+ PCI_BASE_ADDRESS_0, 0);
		}
	}

	reg = 0;
	reg |= PCI_COMMAND_IO;
	reg |= PCI_COMMAND_MEMORY;
	reg |= PCI_COMMAND_MASTER;
	reg |= PCI_COMMAND_SERR;
	pci_conf_write16(dev->bus, dev->devfn, PCI_COMMAND, reg);

	pci_conf_write8(dev->bus, dev->devfn, PCI_INTERRUPT_LINE, INT_PCIE_INTR);
	pci_conf_write8(dev->bus, dev->devfn, PCI_INTERRUPT_PIN, 0xa);
}

static void pci_tegra_print_device_tree(struct pci_tegra_device *dev)
{
	u32 i;

	if (!dev)
		return;

	if (dev->sub_bus)
		pr_err("PCIe bridge/Root port\n");
	else
		pr_err("PCIe device\n");

	pr_err("   Vendor/Device = 0x%x bus = %d sec bus %d sub bus %d\n",
		dev->id, dev->bus, dev->sec_bus, dev->sub_bus);
	if (dev->disabled) {
		pr_err("   Slot disabled\n");
	} else {
		for (i=0; i<= PCI_STD_RESOURCE_END; i++) {
			/* Skip printing the empty ones */
			if (!dev->res[i].start)
				continue;
			pr_err("   bar(%d) \n", i);
			pr_err("      start  = 0x%x\n", dev->res[i].start);
			pr_err("      end = 0x%x\n", dev->res[i].end);
			pr_err("      flags = 0x%lx\n", dev->res[i].flags);
		}
	}

	if (dev->child != NULL)
		pci_tegra_print_device_tree(dev->child);

	if (dev->next != NULL)
		pci_tegra_print_device_tree(dev->next);
}

static void pci_tegra_allocate_resources(struct pci_tegra_device *dev)
{
	/* Employing a depth first search for resource allocation. */
	if (!dev)
		return;

	if (dev->sub_bus) {

		dev->res[PCI_BRIDGE_IO_RES].flags = IORESOURCE_IO;
		dev->res[PCI_BRIDGE_IO_RES].start = pci_tegra_io_base;

		dev->res[PCI_BRIDGE_PREFETCH_RES].flags =
			IORESOURCE_MEM | IORESOURCE_PREFETCH;
		dev->res[PCI_BRIDGE_PREFETCH_RES].start =
			pci_tegra_prefetch_base;

		dev->res[PCI_BRIDGE_MEM_RES].flags = IORESOURCE_MEM;
		dev->res[PCI_BRIDGE_MEM_RES].start = pci_tegra_mem_base;
	}

	if (dev->child)
		pci_tegra_allocate_resources(dev->child);
	if (dev->next)
		pci_tegra_allocate_resources(dev->next);

	if (dev->sub_bus)
		pci_tegra_setup_pci_bridge(dev);
	else {
		pci_tegra_setup_pci_device(dev);
		pci_tegra_io_base = ALIGN(pci_tegra_io_base, 0x1000);
		pci_tegra_mem_base = ALIGN(pci_tegra_mem_base, 0x1000000);
		pci_tegra_prefetch_base =
			ALIGN(pci_tegra_prefetch_base, 0x1000000);
	}
}

void pci_tegra_enumerate(void)
{
	u32 reg;

	/* Disable all execptions */
	pci_tegra_afi_writel(0, AFI_FPCI_ERROR_MASKS_0);

	/* Set the base and limits for the resources */

	/* Starting the IO offset from non-zero value as linux equating a value
	 * of 0 as unallocated resoruce and bailing out!
	 */
	pci_tegra_io_base = TEGRA_PCIE_BASE + PCIE_DOWNSTREAM_IO_OFFSET + 16;
	pci_tegra_io_limt = pci_tegra_io_base + PCIE_DOWNSTREAM_IO_SIZE;

	pci_tegra_mem_base = FPCI_NON_PREFETCH_MEMORY_OFFSET;
	pci_tegra_mem_limit = FPCI_NON_PREFETCH_MEMORY_OFFSET
		+ PCIE_NON_PREFETCH_MEMORY_SIZE;

	pci_tegra_prefetch_base = FPCI_PREFETCH_MEMORY_OFFSET;
	pci_tegra_prefetch_limit = FPCI_PREFETCH_MEMORY_OFFSET
		+ PCIE_PREFETCH_MEMORY_SIZE;

	/* Enumerate only if the Link is UP. */
	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_VEND_XP, 0);
	if (NVPCIE_DRF_VAL(RP, VEND_XP, DL_UP, reg) == 1)
		pci_tegra_enumerate_root_port(0);

	reg = pci_tegra_rp_readl(NV_PROJ__PCIE2_RP_VEND_XP, 1);
	if (NVPCIE_DRF_VAL(RP, VEND_XP, DL_UP, reg) == 1)
		pci_tegra_enumerate_root_port(1);

	pci_tegra_print_device_tree(pci_root);
}
