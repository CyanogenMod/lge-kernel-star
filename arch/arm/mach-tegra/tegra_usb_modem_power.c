/*
 * arch/arm/mach-tegra/tegra_usb_modem_power.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <mach/tegra_usb_modem_power.h>

struct tegra_usb_modem {
	unsigned int wake_gpio;	/* remote wakeup gpio */
	unsigned int wake_cnt;	/* remote wakeup counter */
	int irq;		/* remote wakeup irq */
	struct mutex lock;
	struct wake_lock wake_lock;	/* modem wake lock */
	unsigned int vid;	/* modem vendor id */
	unsigned int pid;	/* modem product id */
	struct usb_device *udev;	/* modem usb device */
	struct usb_interface *intf;	/* first modem usb interface */
	struct workqueue_struct *wq;	/* modem workqueue */
	struct delayed_work recovery_work;	/* modem recovery work */
	const struct tegra_modem_operations *ops;	/* modem operations */
	unsigned int capability;	/* modem capability */
};

static struct tegra_usb_modem tegra_mdm;

/* supported modems */
static const struct usb_device_id modem_list[] = {
	{USB_DEVICE(0x1983, 0x0310),	/* Icera 450 rev1 */
	 .driver_info = 0,
	 },
	{USB_DEVICE(0x1983, 0x0321),	/* Icera 450 rev2 */
	 .driver_info = 0,
	 },
	{}
};

static irqreturn_t tegra_usb_modem_wake_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	wake_lock_timeout(&modem->wake_lock, HZ);
	mutex_lock(&modem->lock);
	if (modem->udev) {
		usb_lock_device(modem->udev);
		pr_info("modem wake (%u)\n", ++(modem->wake_cnt));
		if (usb_autopm_get_interface(modem->intf) == 0)
			usb_autopm_put_interface_async(modem->intf);
		usb_unlock_device(modem->udev);
	}
	mutex_unlock(&modem->lock);

	return IRQ_HANDLED;
}

static void tegra_usb_modem_recovery(struct work_struct *ws)
{
	struct tegra_usb_modem *modem = container_of(ws, struct tegra_usb_modem,
						     recovery_work.work);

	mutex_lock(&modem->lock);
	if (!modem->udev) {	/* assume modem crashed */
		if (modem->ops && modem->ops->reset)
			modem->ops->reset();
	}
	mutex_unlock(&modem->lock);
}

static void device_add_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id = usb_match_id(intf, modem_list);

	if (id) {
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&tegra_mdm.lock);
		tegra_mdm.udev = udev;
		tegra_mdm.intf = intf;
		tegra_mdm.vid = desc->idVendor;
		tegra_mdm.pid = desc->idProduct;
		tegra_mdm.wake_cnt = 0;
		tegra_mdm.capability = id->driver_info;
		mutex_unlock(&tegra_mdm.lock);

		pr_info("persist_enabled: %u\n", udev->persist_enabled);

		if (tegra_mdm.capability & TEGRA_MODEM_AUTOSUSPEND) {
			usb_enable_autosuspend(udev);
			pr_info("enable autosuspend for %s %s\n",
				udev->manufacturer, udev->product);
		}
	}
}

static void device_remove_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == tegra_mdm.vid &&
	    desc->idProduct == tegra_mdm.pid) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&tegra_mdm.lock);
		tegra_mdm.udev = NULL;
		tegra_mdm.intf = NULL;
		tegra_mdm.vid = 0;
		mutex_unlock(&tegra_mdm.lock);

		if (tegra_mdm.capability & TEGRA_MODEM_RECOVERY)
			queue_delayed_work(tegra_mdm.wq,
					   &tegra_mdm.recovery_work, HZ * 10);
	}
}

static int usb_notify(struct notifier_block *self, unsigned long action,
		      void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block usb_nb = {
	.notifier_call = usb_notify,
};

static int tegra_usb_modem_probe(struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	/* get modem operations from platform data */
	tegra_mdm.ops = (const struct tegra_modem_operations *)pdata->ops;

	if (tegra_mdm.ops) {
		/* modem init */
		if (tegra_mdm.ops->init) {
			ret = tegra_mdm.ops->init();
			if (ret)
				return ret;
		}

		/* start modem */
		if (tegra_mdm.ops->start)
			tegra_mdm.ops->start();
	}

	mutex_init(&(tegra_mdm.lock));
	wake_lock_init(&(tegra_mdm.wake_lock), WAKE_LOCK_SUSPEND,
		       "tegra_usb_mdm_lock");

	/* create work queue */
	tegra_mdm.wq = create_workqueue("tegra_usb_mdm_queue");
	INIT_DELAYED_WORK(&(tegra_mdm.recovery_work), tegra_usb_modem_recovery);

	/* create threaded irq for remote wakeup */
	if (pdata->wake_gpio) {
		/* get remote wakeup gpio from platform data */
		tegra_mdm.wake_gpio = pdata->wake_gpio;

		ret = gpio_request(tegra_mdm.wake_gpio, "usb_mdm_wake");
		if (ret)
			return ret;

		tegra_gpio_enable(tegra_mdm.wake_gpio);

		/* enable IRQ for remote wakeup */
		tegra_mdm.irq = gpio_to_irq(tegra_mdm.wake_gpio);

		ret =
		    request_threaded_irq(tegra_mdm.irq, NULL,
					 tegra_usb_modem_wake_thread,
					 pdata->flags, "tegra_usb_mdm_wake",
					 &tegra_mdm);
		if (ret < 0) {
			dev_err(&pdev->dev, "%s: request_threaded_irq error\n",
				__func__);
			return ret;
		}

		ret = enable_irq_wake(tegra_mdm.irq);
		if (ret) {
			dev_err(&pdev->dev, "%s: enable_irq_wake error\n",
				__func__);
			free_irq(tegra_mdm.irq, &tegra_mdm);
			return ret;
		}
	}

	usb_register_notify(&usb_nb);
	dev_info(&pdev->dev, "Initialized tegra_usb_modem_power\n");

	return 0;
}

static int __exit tegra_usb_modem_remove(struct platform_device *pdev)
{
	usb_unregister_notify(&usb_nb);
	free_irq(tegra_mdm.irq, &tegra_mdm);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_usb_modem_suspend(struct platform_device *pdev,
				   pm_message_t state)
{
	/* send L3 hint to modem */
	if (tegra_mdm.ops && tegra_mdm.ops->suspend)
		tegra_mdm.ops->suspend();
	return 0;
}

static int tegra_usb_modem_resume(struct platform_device *pdev)
{
	/* send L3->L0 hint to modem */
	if (tegra_mdm.ops && tegra_mdm.ops->resume)
		tegra_mdm.ops->resume();
	return 0;
}
#endif

static struct platform_driver tegra_usb_modem_power_driver = {
	.driver = {
		   .name = "tegra_usb_modem_power",
		   .owner = THIS_MODULE,
		   },
	.probe = tegra_usb_modem_probe,
	.remove = __exit_p(tegra_usb_modem_remove),
#ifdef CONFIG_PM
	.suspend = tegra_usb_modem_suspend,
	.resume = tegra_usb_modem_resume,
#endif
};

static int __init tegra_usb_modem_power_init(void)
{
	return platform_driver_register(&tegra_usb_modem_power_driver);
}

subsys_initcall(tegra_usb_modem_power_init);

static void __exit tegra_usb_modem_power_exit(void)
{
	platform_driver_unregister(&tegra_usb_modem_power_driver);
}

module_exit(tegra_usb_modem_power_exit);

MODULE_DESCRIPTION("Tegra usb modem power management driver");
MODULE_LICENSE("GPL");
