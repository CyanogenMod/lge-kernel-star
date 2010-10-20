/*
 * ehci-tegra.c
 *
 * EHCI-compliant USB host controller driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
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

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <mach/usb-hcd.h>
#include <mach/nvrm_linux.h>
#include "nvrm_pmu.h"
#include "nvrm_analog.h"
#include "nvassert.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_hardware_access.h"
#include "nvddk_usbphy.h"

#define TEGRA_USB_ID_INT_ENABLE			(1 << 0)
#define TEGRA_USB_ID_INT_STATUS			(1 << 1)
#define TEGRA_USB_ID_PIN_STATUS			(1 << 2)
#define TEGRA_USB_ID_PIN_WAKEUP_ENABLE		(1 << 6)
#define TEGRA_USB_PHY_WAKEUP_REG_OFFSET		(0x408)
#define TEGRA_USB_USBMODE_REG_OFFSET		(0x1a8)
#define TEGRA_USB_USBMODE_HOST			(3)

/*
 * Work thread function for setting the usb busy hints.
 *
 * This work thread is created to avoid the pre-emption from the ISR context.
 * Busy hints are controlled based on the USB transcations on the bus .
 * Busy hints function cannot be called from ISR as NvRmPowerBusyHintMulti()
 * uses vfree and vmalloc functions which are not supposed to call from the
 * ISR context
 */
static void tegra_ehci_busy_hint_work(struct work_struct* work)
{
	struct tegra_hcd_platform_data *pdata =
				container_of(work, struct tegra_hcd_platform_data, work);
	NvDdkUsbPhyIoctl_UsbBusyHintsOnOffInputArgs busyhint;
	busyhint.OnOff = true;

	/* USB transfers will be done with in 1 sec, this need to be *
	* fine tuned (if required). with safe limit set to 2 sec    */
	busyhint.BoostDurationMs = 2000;
	NvDdkUsbPhyIoctl(pdata->hUsbPhy,
					NvDdkUsbPhyIoctlType_UsbBusyHintsOnOff,
					&busyhint,
					NULL);
}

static void tegra_ehci_power_up(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(pdata->hUsbPhy, true, 0));
	ehci->host_resumed = 1;
}

static void tegra_ehci_power_down(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerDown(pdata->hUsbPhy, true, 0));
	ehci->host_resumed = 0;
}

static int tegra_ehci_hub_control (
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ehci_hcd	*ehci = hcd_to_ehci (hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	u32		temp;
	struct tegra_hcd_platform_data *pdata;
	unsigned long	flags;
	int		retval = 0;

	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

	/* if hardware is not accessable then don't read the registers */
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) || !ehci->host_resumed) {
		if (buf)
			memset (buf, 0, wLength);
		return retval;
	}

	/* In ehci_hub_control() for USB_PORT_FEAT_ENABLE clears the other bits
	 * that are write on clear, by wrting back the register read value, so
	 * USB_PORT_FEAT_ENABLE is handled here by masking the set on clear bits */
	if ((typeReq == ClearPortFeature) && (wValue == USB_PORT_FEAT_ENABLE)) {
		spin_lock_irqsave (&ehci->lock, flags);
		temp = ehci_readl(ehci, status_reg);
		ehci_writel(ehci, (temp & ~PORT_RWC_BITS) & ~PORT_PE, status_reg);
		spin_unlock_irqrestore (&ehci->lock, flags);
		return retval;
	}

	/* Handle the hub control events here */
	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

	/* Power down the USB phy when there is no port connection and all
	 * HUB events are cleared by checking the lower four bits
	 * (PORT_CONNECT | PORT_CSC | PORT_PE | PORT_PEC) */
#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state == OTG_STATE_A_SUSPEND) {
			temp = ehci_readl(ehci, status_reg);
			if (!(temp & (PORT_CONNECT | PORT_CSC | PORT_PE | PORT_PEC))
				&& ehci->host_reinited) {
				/* indicate hcd flags, that hardware is not accessable now */
				clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
				ehci_halt(ehci);
				tegra_ehci_power_down(hcd);
				ehci->transceiver->state = OTG_STATE_UNDEFINED;
				ehci->host_reinited = 0;
			}
		}
	}
#endif

	return retval;
}

#if defined(CONFIG_USB_OTG_UTILS) || defined(CONFIG_PM)
static void tegra_ehci_restart (struct usb_hcd *hcd)
{
	unsigned int temp;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	/* Set to Host mode by setting bit 0-1 of USB device mode register */
	temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
	writel((temp | TEGRA_USB_USBMODE_HOST),
		(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));

	/* reset the ehci controller */
	ehci->controller_resets_phy = 0;
	ehci_reset(ehci);
	ehci->controller_resets_phy = 1;
	/* setup the frame list and Async q heads */
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);
	/* setup the command register and set the controller in RUN mode */
	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/* Enable the root Port Power */
	if (HCS_PPC (ehci->hcs_params)) {
		temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
		ehci_writel(ehci, temp | PORT_POWER, &ehci->regs->port_status[0]);
	}

	down_write(&ehci_cf_port_reset_rwsem);
	hcd->state = HC_STATE_RUNNING;
	/* unblock posted writes */
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);
	up_write(&ehci_cf_port_reset_rwsem);

	/* Turn On Interrupts */
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
}
#endif

static void tegra_ehci_shutdown (struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	if (ehci->host_resumed) {
		/* call ehci shut down */
		ehci_shutdown(hcd);
		/* we are ready to shut down, powerdown the phy */
		tegra_ehci_power_down(hcd);
	}
}

/*
 * Work thread function for handling the USB power sequence.
 *
 * This work thread is created to avoid the pre-emption from the ISR context.
 * USB Power Rail and Vbus are controlled based on the USB cable connection.
 * USB Power rail function and VBUS control function cannot be called from ISR
 * as NvRmPmuSetVoltage() uses I2C driver, that waits on semaphore
 * during the I2C transaction this will cause the pre-emption if called in ISR.
 */
static void tegra_ehci_irq_work(struct work_struct* irq_work)
{
	struct ehci_hcd *ehci = container_of(irq_work, struct ehci_hcd, irq_work);
	struct usb_hcd *hcd = ehci_to_hcd(ehci);
	struct tegra_hcd_platform_data *pdata;
	u32 status;
	bool kick_rhub = false;

	pdata = hcd->self.controller->platform_data;

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state == OTG_STATE_A_HOST) {
			if (!ehci->host_reinited) {
				ehci->host_reinited = 1;
				tegra_ehci_power_up(hcd);
				if (hcd->state == HC_STATE_SUSPENDED)
					kick_rhub = true;
				tegra_ehci_restart(hcd);
			}
		} else if (ehci->transceiver->state == OTG_STATE_A_SUSPEND) {
			if (ehci->host_reinited) {
				/* indicate hcd flags, that hardware is not accessible now */
				clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
				ehci_halt(ehci);
				tegra_ehci_power_down(hcd);
				ehci->transceiver->state = OTG_STATE_UNDEFINED;
				ehci->host_reinited = 0;
			}
		}
	} else
#endif
	{
		if (pdata->id_detect == ID_PIN_CABLE_ID) {
			/* read otgsc register for ID pin status change */
			status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
			/* Check pin status and enable/disable the power */
			if (status & TEGRA_USB_ID_PIN_STATUS) {
				tegra_ehci_power_down(hcd);
			} else {
				tegra_ehci_power_up(hcd);
				if (hcd->state == HC_STATE_SUSPENDED)
					kick_rhub = true;
			}
		}
	}

	if (kick_rhub) {
		hcd->state = HC_STATE_SUSPENDED;
		usb_hcd_resume_root_hub(hcd);
	}
}



static irqreturn_t tegra_ehci_irq (struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci (hcd);
	struct tegra_hcd_platform_data *pdata;
	u32 status;

	pdata = hcd->self.controller->platform_data;

	spin_lock (&ehci->lock);

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state == OTG_STATE_A_HOST) {
			if (!ehci->host_reinited) {
				schedule_work(&ehci->irq_work);
				spin_unlock (&ehci->lock);
				return IRQ_HANDLED;
			}
		} else if (ehci->transceiver->state == OTG_STATE_A_SUSPEND) {
			if (!ehci->host_reinited) {
				spin_unlock (&ehci->lock);
				return IRQ_HANDLED;
			} else {
				schedule_work(&ehci->irq_work);
				spin_unlock (&ehci->lock);
				return IRQ_HANDLED;
			}
		} else {
			spin_unlock (&ehci->lock);
			return IRQ_HANDLED;
		}
	} else
#endif
	{
		if (pdata->id_detect == ID_PIN_CABLE_ID) {
			/* read otgsc register for ID pin status change */
			status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
			writel(status, (hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET));

			/* Check if there is any ID pin interrupt */
			if (status & TEGRA_USB_ID_INT_STATUS) {
				schedule_work(&ehci->irq_work);
				spin_unlock (&ehci->lock);
				return IRQ_HANDLED;
			}
		}
	}

	spin_unlock (&ehci->lock);

	return ehci_irq(hcd);
}

static int tegra_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
		HC_LENGTH(readl(&ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval) {
		return retval;
	}

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval) {
		return retval;
	}

	hcd->has_tt = 1;
	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	/* Resetting the controller has the side effect of resetting the PHY.
	 * So, never reset the controller after the calling
	 * tegra_ehci_reinit API. */
	ehci->controller_resets_phy = 1;

	ehci_port_power(ehci, 0);
	return retval;
}

#if defined(CONFIG_PM)
static int tegra_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;
	int error_status = 0;

	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state != OTG_STATE_A_HOST) {
			/* we are not in host mode, return */
			return 0;
		} else {
			ehci->transceiver->state = OTG_STATE_A_SUSPEND;
			ehci->host_reinited = 0;
		}
	}
#endif

	if (!pdata->otg_mode && pdata->id_detect==ID_PIN_CABLE_ID) {
		u32 status;
		/* check for ID pin status */
		status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
		/* If Id pin is high means host is not connected return */
		if (status & TEGRA_USB_ID_PIN_STATUS) {
			return 0;
		}
	}

	if (ehci->host_resumed) {
		error_status = ehci_bus_suspend(hcd);
		if (!error_status)
			tegra_ehci_power_down(hcd);
	}

	return error_status;
}

static int tegra_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;

	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state != OTG_STATE_A_HOST) {
			/* we are not in host mode, return */
			return 0;
		}
	}
#endif

	if (!pdata->otg_mode && pdata->id_detect==ID_PIN_CABLE_ID) {
		u32 status;
		/* read ID pin status */
		status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
		/* If Id pin is high no host then return */
		if (status & TEGRA_USB_ID_PIN_STATUS) {
			return 0;
		}
	}

	if (!ehci->host_resumed) {
		tegra_ehci_power_up(hcd);
	}

	return ehci_bus_resume(hcd);
}
#endif

static int tegra_ehci_urb_enqueue(
	struct usb_hcd	*hcd,
	struct urb		*urb,
	gfp_t			mem_flags
) {
	struct tegra_hcd_platform_data *pdata;
	int xfertype;
	int transfer_buffer_length;

	pdata = hcd->self.controller->platform_data;

	xfertype = usb_endpoint_type(&urb->ep->desc);
	transfer_buffer_length = urb->transfer_buffer_length;
	/* Turn on the USB busy hints */
	switch (xfertype) {
		case USB_ENDPOINT_XFER_INT:
			if (transfer_buffer_length < 255) {
				/* Do nothing for interrupt buffers < 255 */
			} else {
				// signal to set the busy hints
				schedule_work(&pdata->work);
			}
		break;
		case USB_ENDPOINT_XFER_ISOC:
		case USB_ENDPOINT_XFER_BULK:
			// signal to set the busy hints
			schedule_work(&pdata->work);
		break;
		case USB_ENDPOINT_XFER_CONTROL:
		default:
		/* Do nothing special here */
		break;
	}

	return ehci_urb_enqueue(hcd, urb, mem_flags);
}

static const struct hc_driver tegra_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Tegra Ehci host controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	.flags			= HCD_USB2,
	/* lifecycle management */
	.reset			= tegra_ehci_setup,
	.irq			= tegra_ehci_irq,

	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= tegra_ehci_shutdown,
	.urb_enqueue		= tegra_ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= tegra_ehci_hub_control,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
#if defined(CONFIG_PM)
	.bus_suspend		= tegra_ehci_bus_suspend,
	.bus_resume 		= tegra_ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};
static int tegra_ehci_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct tegra_hcd_platform_data *pdata = pdev->dev.platform_data;
	struct usb_hcd *hcd;
	int e = 0;
	int irq;
	unsigned int temp;
	struct ehci_hcd *ehci;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data must be specified\n");
		return -EINVAL;
	}

	WARN_ON(!pdev->dev.coherent_dma_mask || !pdev->dev.dma_mask);

	hcd = usb_create_hcd(&tegra_ehci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev));
	if (IS_ERR_OR_NULL(hcd)) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}

	if (pdata->id_detect == ID_PIN_GPIO) {
		e = gpio_request(pdata->gpio_nr, dev_name(&pdev->dev));
		if (e < 0) {
			dev_err(&pdev->dev, "request ID pin GPIO failed\n");
			goto fail_hcd;
		}

		e = gpio_direction_input(pdata->gpio_nr);
		if (e < 0) {
			dev_err(&pdev->dev, "failed to set ID pin as input\n");
			goto fail_gpio;
		}
	}

	INIT_WORK(&pdata->work, tegra_ehci_busy_hint_work);

	/* Init the tegra USB phy */
	if (NvDdkUsbPhyOpen(s_hRmGlobal, pdata->instance,
			    &pdata->hUsbPhy) != NvSuccess) {
		dev_err(&pdev->dev, "failed to open USB phy DDK\n");
		e = -ENODEV;
		goto fail_gpio;
	}
	tegra_ehci_power_up(hcd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		e = -ENXIO;
		goto fail_phy;
	}
	if (!pdata->otg_mode) {
		res = request_mem_region(res->start, resource_size(res),
					 dev_name(&pdev->dev));
		if (!res) {
			dev_err(&pdev->dev, "resource in use\n");
			e = -EBUSY;
			goto fail_phy;
		}
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(res->start, resource_size(res));
	if (!hcd->regs) {
		e = -ENOMEM;
		goto fail_mem;
	}

	/* Set to Host mode by setting bit 0-1 of USB device mode register */
	temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
	writel((temp | TEGRA_USB_USBMODE_HOST),
			(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		e = -ENODEV;
		goto fail_iomap;
	}

	set_irq_flags(irq, IRQF_VALID);

	ehci = hcd_to_ehci(hcd);
	INIT_WORK(&ehci->irq_work, tegra_ehci_irq_work);

	e = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (e != 0) {
		dev_err(&pdev->dev, "failed to add HCD\n");
		goto fail_iomap;
	}
	platform_set_drvdata(pdev, hcd);

#ifdef CONFIG_DMABOUNCE
	e = dmabounce_register_dev(&pdev->dev, 1024, 32768);
	if (e != 0) {
		dev_err(&pdev->dev, "failed to register DMA bounce\n");
		goto fail_add;
	}
#endif

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode) {
		ehci->transceiver = otg_get_transceiver();
		if (!ehci->transceiver) {
			dev_err(&pdev->dev, "Failed to get OTG transceiver\n");
			e = -ENODEV;
			goto fail_dmabounce;
		}

		otg_set_host(ehci->transceiver, (struct usb_bus *)hcd);
		/* Stop the controller and power down the phy, OTG will
		 * start the host driver based on the ID pin
		 * detection */
		ehci_halt(ehci);
		/* reset the host and put the controller in idle mode */
		temp = ehci_readl(ehci, &ehci->regs->command);
		temp |= CMD_RESET;
		ehci_writel(ehci, temp, &ehci->regs->command);
		temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
		writel((temp & ~TEGRA_USB_USBMODE_HOST),
			(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));
		/* indicate hcd flags, that hardware is not accessable now
		 * in host mode*/
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		tegra_ehci_power_down(hcd);
		ehci->host_reinited = 0;
	} else
#endif
	{
		if (pdata->id_detect == ID_PIN_CABLE_ID) {
			/* enable the cable ID interrupt */
			temp = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
			temp |= TEGRA_USB_ID_INT_ENABLE;
			temp |= TEGRA_USB_ID_PIN_WAKEUP_ENABLE;
			writel(temp, (hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET));

			/* Check if we detect any device connected */
			if (temp & TEGRA_USB_ID_PIN_STATUS) {
				tegra_ehci_power_down(hcd);
			} else {
				tegra_ehci_power_up(hcd);
			}
		}
	}

	return 0;

fail_dmabounce:
#ifdef CONFIG_DMABOUNCE
	dmabounce_unregister_dev(&pdev->dev);
#endif
fail_add:
	usb_remove_hcd(hcd);
fail_iomap:
	iounmap(hcd->regs);
fail_mem:
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
fail_phy:
	NvDdkUsbPhyClose(pdata->hUsbPhy);
fail_gpio:
	if (pdata->gpio_nr)
		gpio_free(pdata->gpio_nr);
fail_hcd:
	usb_put_hcd(hcd);
	return e;
}

#if defined(CONFIG_PM)
static int tegra_ehci_resume(struct platform_device * pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;
	u32 status;

	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

	/* read otgsc register for ID pin status */
	status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->otg_mode && ehci->transceiver) {
		/* check if ID pin is high then no host return */
		if (status & TEGRA_USB_ID_PIN_STATUS) {
			return 0;
		}
		else {
			/* set HCD flags to start host ISR */
			set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
			ehci->host_reinited = 1;
			ehci->transceiver->state = OTG_STATE_A_HOST;
		}
	}
#endif

	if (!pdata->otg_mode && pdata->id_detect==ID_PIN_CABLE_ID) {
		/* If no Id pin then return */
		if (status & TEGRA_USB_ID_PIN_STATUS) {
			return 0;
		}
	}

	if (!ehci->host_resumed) {
		tegra_ehci_power_up(hcd);
		/* restart the controller in OTG mode only */
		if(pdata->otg_mode)
			tegra_ehci_restart(hcd);
	}

	return 0;
}

static int tegra_ehci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

#ifdef CONFIG_USB_OTG_UTILS
	struct tegra_hcd_platform_data *pdata;
	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;
	if (pdata->otg_mode && ehci->transceiver) {
		if (ehci->transceiver->state != OTG_STATE_A_HOST) {
			/* we are not in host mode, return */
			return 0;
		}
		else {
			ehci->transceiver->state = OTG_STATE_UNDEFINED;
			ehci->host_reinited = 0;
			/* indicate hcd flags, that hardware is not accessable now */
			clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		}
	}
#endif

	if (ehci->host_resumed) {
		/* halt the controller before powering down the phy */
		ehci_halt(ehci);
		tegra_ehci_power_down(hcd);
	}

	return 0;
}
#endif

static int tegra_ehci_remove(struct platform_device *pdev)
{
	struct tegra_hcd_platform_data *pdata = pdev->dev.platform_data;
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	if (pdata == NULL || hcd == NULL)
		return -EINVAL;

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

#ifdef CONFIG_DMABOUNCE
	dmabounce_unregister_dev(&pdev->dev);
#endif

	NvDdkUsbPhyClose(pdata->hUsbPhy);

	iounmap(hcd->regs);


	if (pdata->gpio_nr)
		gpio_free(pdata->gpio_nr);

	return 0;
}

static struct platform_driver tegra_ehci_driver =
{
	.probe		= tegra_ehci_probe,
	.remove		= tegra_ehci_remove,
#if defined(CONFIG_PM)
	.suspend = tegra_ehci_suspend,
	.resume = tegra_ehci_resume,
#endif
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "tegra-ehci",
	}
};
