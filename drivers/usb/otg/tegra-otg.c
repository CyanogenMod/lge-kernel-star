/*
 * tegra-otg.c
 *
 * OTG driver for detecting the USB ID and VBUS for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#define NV_DEBUG 0

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/usb-otg.h>
#include <mach/nvrm_linux.h>
#include "../core/hcd.h"
#include "nvddk_usbphy.h"

#define TEGRA_USB_ID_INT_ENABLE			(1 << 0)
#define TEGRA_USB_ID_INT_STATUS			(1 << 1)
#define TEGRA_USB_ID_STATUS			(1 << 2)
#define TEGRA_USB_ID_PIN_WAKEUP_ENABLE		(1 << 6)
#define TEGRA_USB_VBUS_WAKEUP_ENABLE		(1 << 30)
#define TEGRA_USB_VBUS_INT_ENABLE		(1 << 8)
#define TEGRA_USB_VBUS_INT_STATUS		(1 << 9)
#define TEGRA_USB_VBUS_STATUS			(1 << 10)
#define TEGRA_USB_WAKEUP_REG_OFFSET		(0x408)

static const char driver_name[] = "tegra-otg";

/*
 * Needs to be loaded before the UDC and Host driver that will use it.
 */
struct tegra_otg_data {
	struct otg_transceiver otg;
	struct device          *dev;
	spinlock_t lock;
	int			irq;	/* irq allocated */
	void __iomem		*regs;	/* device memory/io */
	int instance; /* instance number of the controller */
	NvDdkUsbPhyHandle usb_phy; /* handle to the USB phy */
};

static struct tegra_otg_data *sg_tegra_otg = NULL;


/* VBUS change IRQ handler */
static irqreturn_t tegra_otg_irq(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = (struct usb_hcd *)tegra_otg->otg.host;
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&tegra_otg->lock, flags);

	status = readl(tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET);
	writel(status, tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET);

	if (tegra_otg->otg.host) {
		/* Check if there is any ID pin interrupt */
		if (status & TEGRA_USB_ID_INT_STATUS) {
			if (status & TEGRA_USB_ID_STATUS) {
				tegra_otg->otg.state = OTG_STATE_A_SUSPEND;
			} else {
				tegra_otg->otg.state = OTG_STATE_A_HOST;
				/* set HCD flags to start host ISR */
				set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
			}
		}
	}

	if (tegra_otg->otg.gadget && (tegra_otg->otg.state != OTG_STATE_A_HOST)) {
		if (status & TEGRA_USB_VBUS_INT_STATUS) {
			if (status & TEGRA_USB_VBUS_STATUS) {
				tegra_otg->otg.state = OTG_STATE_B_PERIPHERAL;
			} else {
				tegra_otg->otg.state = OTG_STATE_A_SUSPEND;
			}
		}
	}
	spin_unlock_irqrestore(&tegra_otg->lock, flags);
	return IRQ_HANDLED;
}

/* OTG transceiver interface */
static int tegra_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	unsigned int temp;
	unsigned long flags;

	otg->gadget = gadget;
	temp = readl(sg_tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET);
	temp |= (TEGRA_USB_VBUS_INT_ENABLE | TEGRA_USB_VBUS_WAKEUP_ENABLE);
	temp &= ~TEGRA_USB_VBUS_INT_STATUS;
	writel(temp, (sg_tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET));

	/* Check if we detect any device connected */
	if (!(temp & TEGRA_USB_ID_STATUS)) {
		struct usb_hcd *hcd = (struct usb_hcd *)otg->host;
		spin_lock_irqsave(&sg_tegra_otg->lock, flags);
		otg->state = OTG_STATE_A_HOST;
		/* set HCD flags to start host ISR */
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		spin_unlock_irqrestore(&sg_tegra_otg->lock, flags);
		NvDdkUsbPhyPowerUp(sg_tegra_otg->usb_phy, NV_TRUE, 0);
	}

	return 0;
}

static int tegra_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host)
{
	unsigned int temp;
	struct tegra_otg_platform_data *pdata;

	pdata = sg_tegra_otg->dev->platform_data;
	otg->host = host;

	if (pdata->usb_property->IdPinDetectionType ==
		NvOdmUsbIdPinType_CableId) {
		/* enable the cable ID interrupt */
		temp = readl(sg_tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET);
		temp |= (TEGRA_USB_ID_INT_ENABLE | TEGRA_USB_ID_PIN_WAKEUP_ENABLE);
		writel(temp, (sg_tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET));
	}

	return 0;
}

/* effective for B devices, ignored for A-peripheral */
static int tegra_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	/* Draw from the Host in device mode */

	return 0;
}

/* for non-OTG B devices: set/clear transceiver suspend mode */
static int tegra_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	/* Draw 0mA in the suspend mode */
	return 0;
}

/* platform driver interface */
static int __init tegra_otg_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	int instance = pdev->id;
	NvError e;

	sg_tegra_otg = kzalloc(sizeof(struct tegra_otg_data), GFP_KERNEL);
	if (!sg_tegra_otg)
		return -ENOMEM;

	spin_lock_init(&sg_tegra_otg->lock);
	platform_set_drvdata(pdev, sg_tegra_otg);

	NV_CHECK_ERROR_CLEANUP(
		NvDdkUsbPhyOpen(s_hRmGlobal, instance, &sg_tegra_otg->usb_phy)
	);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkUsbPhyPowerUp(sg_tegra_otg->usb_phy, NV_FALSE, 0)
	);
	sg_tegra_otg->instance = pdev->id;
	sg_tegra_otg->dev = &pdev->dev;
	sg_tegra_otg->otg.label = driver_name;
	sg_tegra_otg->otg.state = OTG_STATE_UNDEFINED;
	sg_tegra_otg->otg.set_peripheral = tegra_otg_set_peripheral;
	sg_tegra_otg->otg.set_host = tegra_otg_set_host;
	sg_tegra_otg->otg.set_power = tegra_otg_set_power;
	sg_tegra_otg->otg.set_suspend = tegra_otg_set_suspend;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENXIO;
		goto err_irq;
	}

	sg_tegra_otg->regs = ioremap(res->start, resource_size(res));
	if (!sg_tegra_otg->regs) {
		err = -ENOMEM;
		goto err_irq;
	}

	sg_tegra_otg->irq = platform_get_irq(pdev, 0);
	if (!sg_tegra_otg->irq) {
		err = -ENODEV;
		goto err_irq;
	}

	err = request_irq(sg_tegra_otg->irq, tegra_otg_irq, IRQF_SHARED,
			driver_name, pdev);
	if (err) {
		printk("cannot request irq %d err %d\n",
				sg_tegra_otg->irq, err);
		goto err_mem_map;
	}

	/* only active when a gadget is registered */
	err = otg_set_transceiver(&sg_tegra_otg->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		goto err_otg;
	}

	NvDdkUsbPhyPowerDown(sg_tegra_otg->usb_phy, NV_FALSE, 0);

	return 0;

err_otg:
	free_irq(sg_tegra_otg->irq, &pdev->dev);
err_mem_map:
	iounmap(sg_tegra_otg->regs);
err_irq:
	NvDdkUsbPhyClose(sg_tegra_otg->usb_phy);
fail:
	platform_set_drvdata(pdev, NULL);
	kfree(sg_tegra_otg);
	return err;
}

static int __exit tegra_otg_remove(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);

	otg_set_transceiver(NULL);
	free_irq(tegra_otg->irq, &pdev->dev);
	iounmap(tegra_otg->regs);
	NvDdkUsbPhyClose(tegra_otg->usb_phy);
	platform_set_drvdata(pdev, NULL);
	kfree(tegra_otg);
	sg_tegra_otg = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int tegra_otg_resume(struct platform_device * pdev)
{
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
	unsigned int temp;

	/* enable the cable ID and VBUS interrupts */
	temp = readl(tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET);
	temp |= (TEGRA_USB_ID_INT_ENABLE | TEGRA_USB_ID_PIN_WAKEUP_ENABLE);
	temp |= (TEGRA_USB_VBUS_INT_ENABLE | TEGRA_USB_VBUS_WAKEUP_ENABLE);
	temp &= ~TEGRA_USB_VBUS_INT_STATUS;
	writel(temp, (tegra_otg->regs + TEGRA_USB_WAKEUP_REG_OFFSET));

	return 0;
}
#endif

static struct platform_driver tegra_otg_driver = {
	.driver = {
		.name  = driver_name,
	},
	.remove  = __exit_p(tegra_otg_remove),
	.probe   = tegra_otg_probe,
#if defined(CONFIG_PM)
	.resume = tegra_otg_resume,
#endif
};

static int __init tegra_otg_init(void)
{
	return platform_driver_register(&tegra_otg_driver);
}
module_init(tegra_otg_init);

static void __exit tegra_otg_exit(void)
{
	platform_driver_unregister(&tegra_otg_driver);
}
module_exit(tegra_otg_exit);

MODULE_DESCRIPTION("Tegra OTG driver");
