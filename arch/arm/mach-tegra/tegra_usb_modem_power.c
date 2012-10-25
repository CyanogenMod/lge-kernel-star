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
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/slab.h>
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
	int system_suspend;	/* system suspend flag */
	struct notifier_block pm_notifier;	/* pm event notifier */
	struct notifier_block usb_notifier;	/* usb event notifier */
};

/* supported modems */
static const struct usb_device_id modem_list[] = {
	{USB_DEVICE(0x1983, 0x0310),	/* Icera 450 rev1 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{USB_DEVICE(0x1983, 0x0321),	/* Icera 450 rev2 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{}
};

static irqreturn_t tegra_usb_modem_wake_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	wake_lock_timeout(&modem->wake_lock, HZ);
	mutex_lock(&modem->lock);
	if (modem->udev) {
		pr_info("modem wake (%u)\n", ++(modem->wake_cnt));

		if (!modem->system_suspend) {
			usb_lock_device(modem->udev);
			if (usb_autopm_get_interface(modem->intf) == 0)
				usb_autopm_put_interface_async(modem->intf);
			usb_unlock_device(modem->udev);
		}
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

static void device_add_handler(struct tegra_usb_modem *modem,
			       struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id = usb_match_id(intf, modem_list);

	if (id) {
		/* hold wakelock to ensure ril has enough time to restart */
		wake_lock_timeout(&modem->wake_lock, HZ * 10);

		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&modem->lock);
		modem->udev = udev;
		modem->intf = intf;
		modem->vid = desc->idVendor;
		modem->pid = desc->idProduct;
		modem->wake_cnt = 0;
		modem->capability = id->driver_info;
		mutex_unlock(&modem->lock);

		pr_info("persist_enabled: %u\n", udev->persist_enabled);

#ifdef CONFIG_PM
		if (modem->capability & TEGRA_MODEM_AUTOSUSPEND) {
			pm_runtime_set_autosuspend_delay(&udev->dev, 2000);
			usb_enable_autosuspend(udev);
			pr_info("enable autosuspend for %s %s\n",
				udev->manufacturer, udev->product);
		}
#endif
	}
}

static void device_remove_handler(struct tegra_usb_modem *modem,
				  struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == modem->vid &&
	    desc->idProduct == modem->pid) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&modem->lock);
		modem->udev = NULL;
		modem->intf = NULL;
		modem->vid = 0;
		mutex_unlock(&modem->lock);

		if (modem->capability & TEGRA_MODEM_RECOVERY)
			queue_delayed_work(modem->wq,
					   &modem->recovery_work, HZ * 10);
	}
}

static int mdm_usb_notifier(struct notifier_block *notifier,
			    unsigned long usb_event,
			    void *udev)
{
	struct tegra_usb_modem *modem =
		container_of(notifier, struct tegra_usb_modem, usb_notifier);

	switch (usb_event) {
	case USB_DEVICE_ADD:
		device_add_handler(modem, udev);
		break;
	case USB_DEVICE_REMOVE:
		device_remove_handler(modem, udev);
		break;
	}
	return NOTIFY_OK;
}

static int mdm_pm_notifier(struct notifier_block *notifier,
			   unsigned long pm_event,
			   void *unused)
{
	struct tegra_usb_modem *modem =
		container_of(notifier, struct tegra_usb_modem, pm_notifier);

	mutex_lock(&modem->lock);
	if (!modem->udev) {
		mutex_unlock(&modem->lock);
		return NOTIFY_DONE;
	}

	pr_info("%s: event %ld\n", __func__, pm_event);
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		if (wake_lock_active(&modem->wake_lock)) {
			pr_warn("%s: wakelock was active, aborting suspend\n",
				__func__);
			return NOTIFY_STOP;
		}

		modem->system_suspend = 1;
		mutex_unlock(&modem->lock);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		modem->system_suspend = 0;
		mutex_unlock(&modem->lock);
		return NOTIFY_OK;
	}

	mutex_unlock(&modem->lock);
	return NOTIFY_DONE;
}

static int mdm_init(struct tegra_usb_modem *modem, struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	int ret = 0;

	/* get modem operations from platform data */
	modem->ops = (const struct tegra_modem_operations *)pdata->ops;

	if (modem->ops) {
		/* modem init */
		if (modem->ops->init) {
			ret = modem->ops->init();
			if (ret)
				return ret;
		}

		/* start modem */
		if (modem->ops->start)
			modem->ops->start();
	}

	mutex_init(&(modem->lock));
	wake_lock_init(&modem->wake_lock, WAKE_LOCK_SUSPEND,
		       "tegra_usb_mdm_lock");

	/* create work queue platform_driver_registe*/
	modem->wq = create_workqueue("tegra_usb_mdm_queue");
	INIT_DELAYED_WORK(&(modem->recovery_work), tegra_usb_modem_recovery);

	/* create threaded irq for remote wakeup */
	if (gpio_is_valid(pdata->wake_gpio)) {
		/* get remote wakeup gpio from platform data */
		modem->wake_gpio = pdata->wake_gpio;

		ret = gpio_request(modem->wake_gpio, "usb_mdm_wake");
		if (ret)
			return ret;

		tegra_gpio_enable(modem->wake_gpio);

		/* enable IRQ for remote wakeup */
		modem->irq = gpio_to_irq(modem->wake_gpio);

		ret =
		    request_threaded_irq(modem->irq, NULL,
					 tegra_usb_modem_wake_thread,
					 pdata->flags, "tegra_usb_mdm_wake",
					 modem);
		if (ret < 0) {
			dev_err(&pdev->dev, "%s: request_threaded_irq error\n",
				__func__);
			return ret;
		}

		ret = enable_irq_wake(modem->irq);
		if (ret) {
			dev_err(&pdev->dev, "%s: enable_irq_wake error\n",
				__func__);
			free_irq(modem->irq, modem);
			return ret;
		}
	}

	modem->pm_notifier.notifier_call = mdm_pm_notifier;
	modem->usb_notifier.notifier_call = mdm_usb_notifier;

	usb_register_notify(&modem->usb_notifier);
	register_pm_notifier(&modem->pm_notifier);

	return ret;
}

static int tegra_usb_modem_probe(struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	struct tegra_usb_modem *modem;
	int ret = 0;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	modem = kzalloc(sizeof(struct tegra_usb_modem), GFP_KERNEL);
	if (!modem) {
		dev_dbg(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = mdm_init(modem, pdev);
	if (ret) {
		kfree(modem);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, modem);

	return ret;
}

static int __exit tegra_usb_modem_remove(struct platform_device *pdev)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	unregister_pm_notifier(&modem->pm_notifier);
	usb_unregister_notify(&modem->usb_notifier);

	if (modem->irq) {
		disable_irq_wake(modem->irq);
		free_irq(modem->irq, modem);
	}
	kfree(modem);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_usb_modem_suspend(struct platform_device *pdev,
				   pm_message_t state)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	/* send L3 hint to modem */
	if (modem->ops && modem->ops->suspend)
		modem->ops->suspend();
	return 0;
}

static int tegra_usb_modem_resume(struct platform_device *pdev)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	/* send L3->L0 hint to modem */
	if (modem->ops && modem->ops->resume)
		modem->ops->resume();
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
