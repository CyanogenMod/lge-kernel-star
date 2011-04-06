/*
 * drivers/input/touchscreen/tegra_odm.c
 *
 * Touchscreen class input driver for platforms using NVIDIA's Tegra ODM kit
 * driver interface
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
//#include <linux/tegra_devices.h>
#include <nvodm_services.h>
#include <nvodm_onetouch.h>


#define NVODM_TOUCH_NAME "nvodm_onetouch"

struct tegra_onetouch_driver_data
{
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOdmOsSemaphoreHandle	semaphore;
	NvOdmOneTouchDeviceHandle	hOneTouchDevice;
	NvBool			bPollingMode;
	NvU32			pollingIntervalMS;
	int			shutdown;
	int			isReset;
	struct early_suspend	early_suspend;
};

#ifdef CONFIG_STAR_TOUCH_LED
extern void touchLED_enable(NvBool status);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tegra_onetouch_early_suspend(struct early_suspend *es)
{
	struct tegra_onetouch_driver_data *onetouch;
	onetouch = container_of(es, struct tegra_onetouch_driver_data, early_suspend);

	printk("[ONETOUCH] tegra_onetouch_early_suspend\n");
	
	if (onetouch && onetouch->hOneTouchDevice) {
		NvOdmOneTouchInterruptMask(onetouch->hOneTouchDevice, NV_TRUE);
		NvOdmOsSleepMS(50);
		if(onetouch->isReset){
			NvOdmOsSleepMS(50);
			onetouch->isReset = 0;
		}
			
		if (!NvOdmOneTouchSleepMode(onetouch->hOneTouchDevice, NV_TRUE))
			pr_err("[ONETOUCH] tegra_onetouch_suspend: fail onetouch sleepmode\n");
	}
	else {
		pr_err("[ONETOUCH] tegra_touch_early_suspend: NULL handles passed\n");
	}
		
	return;
}

static void tegra_onetouch_late_resume(struct early_suspend *es)
{
	struct tegra_onetouch_driver_data *onetouch;
	onetouch = container_of(es, struct tegra_onetouch_driver_data, early_suspend);

	printk("[ONETOUCH] tegra_onetouch_late_resume\n");

	if (onetouch && onetouch->hOneTouchDevice) {
		if (!NvOdmOneTouchSleepMode(onetouch->hOneTouchDevice, NV_FALSE))
		{
			NvOdmOneTouchDeviceClose(onetouch->hOneTouchDevice);
			NvOdmOneTouchDeviceOpen(&onetouch->hOneTouchDevice, &onetouch->semaphore);
		}
		else
			NvOdmOneTouchInterruptMask(onetouch->hOneTouchDevice, NV_FALSE);
	}
	else {
		pr_err("[ONETOUCH] tegra_touch_late_resume: NULL handles passed\n");
	}
}
#else
static int tegra_onetouch_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_onetouch_driver_data *onetouch = (struct tegra_onetouch_driver_data*)pdata;

	if (NvOdmOneTouchSleepMode(onetouch->hOneTouchDevice, NV_TRUE))
		return 0;

	pr_err("tegra_onetouch_suspend: NULL handles passed\n");
	return -1;
}

static int tegra_onetouch_resume(struct platform_device *pdev)
{
	struct tegra_onetouch_driver_data *onetouch = (struct tegra_onetouch_driver_data*)pdata;

	if (NvOdmOneTouchSleepMode(onetouch->hOneTouchDevice, NV_FALSE))
		return 0;

	pr_err("tegra_onetouch_resume: NULL handles passed\n");
	return -1;
}
#endif

static int tegra_onetouch_thread(void *pdata)
{
	struct tegra_onetouch_driver_data *onetouch = (struct tegra_onetouch_driver_data*)pdata;
	
	NvOdmOneTouchButtonInfo button = {0};
	NvBool bKeepReadingSamples;

	NvU16 pressed_menu_button = NV_FALSE;
	NvU16 pressed_back_button = NV_FALSE;	
	
	/* onetouch event thread should be frozen before suspend */
	set_freezable_with_signal();
	
	for (;;)
	{
		if (onetouch->bPollingMode)
		msleep(onetouch->pollingIntervalMS); 
		else
		/* FIXME should have a timeout so, that we can exit thread */
		if (!NvOdmOsSemaphoreWaitTimeout(onetouch->semaphore, NV_WAIT_INFINITE))
		BUG();

		bKeepReadingSamples = NV_TRUE;
		while (bKeepReadingSamples)
		{
			if (!NvOdmOneTouchReadButton(onetouch->hOneTouchDevice, &button))
			{
				pr_err("Couldn't read onetouch sample\n");
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
				onetouch->isReset = 1;
				NvOdmOneTouchDeviceClose(onetouch->hOneTouchDevice);
				NvOdmOneTouchDeviceOpen(&onetouch->hOneTouchDevice, &onetouch->semaphore);
				onetouch->isReset = 0;
				break;
#else
				bKeepReadingSamples = NV_FALSE;
				goto DoneWithSample;
#endif
			}

			if(button.menu == NV_TRUE && !pressed_menu_button)
			{
				input_report_key(onetouch->input_dev, KEY_MENU, 1);
				pressed_menu_button = NV_TRUE;
			}
			else if(button.menu == NV_FALSE && pressed_menu_button)
			{
				input_report_key(onetouch->input_dev, KEY_MENU, 0);
				pressed_menu_button = NV_FALSE;
			}

			if(button.back == NV_TRUE && !pressed_back_button)
			{
				input_report_key(onetouch->input_dev, KEY_BACK, 1);
				pressed_back_button = NV_TRUE;
			}
			else if (button.back == NV_FALSE && pressed_back_button)
			{
				input_report_key(onetouch->input_dev, KEY_BACK, 0);
				pressed_back_button = NV_FALSE;
			}			
			
#ifdef CONFIG_STAR_TOUCH_LED
	   		touchLED_enable(NV_TRUE);
#endif

			input_mt_sync(onetouch->input_dev);
			input_sync(onetouch->input_dev);

#if !defined(CONFIG_MACH_STAR_SKT_REV_E) && !defined(CONFIG_MACH_STAR_SKT_REV_F)
			DoneWithSample:
#endif
			bKeepReadingSamples = NV_FALSE;
			
			if (!onetouch->bPollingMode && 
			!NvOdmOneTouchHandleInterrupt(onetouch->hOneTouchDevice))
			{
				/* Some more data to read keep going */
				bKeepReadingSamples = NV_TRUE;
			}
		}
	}

    return 0;
}


static int __init tegra_onetouch_probe(struct platform_device *pdev)
{
	struct tegra_onetouch_driver_data *onetouch = NULL;
	struct input_dev *input_dev = NULL;
	int err;
//	NvOdmTouchCapabilities *caps;

	printk("[ONETOUCH] tegra_onetouch_probe start\n");
	onetouch = kzalloc(sizeof(struct tegra_onetouch_driver_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (input_dev == NULL || onetouch == NULL) {
		input_free_device(input_dev);
		kfree(onetouch);
		err = -ENOMEM;
		pr_err("tegra_onetouch_probe: Failed to allocate input device\n");
		return err;
	}
	
	onetouch->semaphore = NvOdmOsSemaphoreCreate(0);
	if (!onetouch->semaphore) {
		err = -1;
		pr_err("tegra_onetouch_probe: Semaphore creation failed\n");
		goto err_semaphore_create_failed;
	}

// 20100423  for Touch Interrupt Issue at booting [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
	onetouch->bPollingMode = NV_FALSE;
	if (!NvOdmOneTouchDeviceOpen(&onetouch->hOneTouchDevice, &onetouch->semaphore)) {
		err = -1;
		pr_err("tegra_onetouch_probe: NvOdmOneTouchDeviceOpen failed\n");
		goto err_open_failed;
	}
#else
	if (!NvOdmOneTouchDeviceOpen(&onetouch->hOneTouchDevice)) {
		err = -1;
		pr_err("tegra_onetouch_probe: NvOdmOneTouchDeviceOpen failed\n");
		goto err_open_failed;
	}
	onetouch->bPollingMode = NV_FALSE;
	if (!NvOdmTouchEnableInterrupt(onetouch->hOneTouchDevice, onetouch->semaphore)) {
		err = -1;
		pr_err("tegra_onetouch_probe: Interrupt failed, polling mode\n");
		onetouch->bPollingMode = NV_TRUE;
		onetouch->pollingIntervalMS = 10;
	}
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100423  for Touch Interrupt Issue at booting [END]

	onetouch->task =
		kthread_create(tegra_onetouch_thread, onetouch, "tegra_onetouch_thread");

	if(onetouch->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	wake_up_process( onetouch->task );

	onetouch->input_dev = input_dev;
	onetouch->input_dev->name = NVODM_TOUCH_NAME;

	/* Will generate sync at the end of all input */
	set_bit(EV_SYN, onetouch->input_dev->evbit);
	/* Event is key input type */
	set_bit(EV_KEY, onetouch->input_dev->evbit);
	/* virtual key is BTN_TOUCH */
	set_bit(BTN_TOUCH, onetouch->input_dev->keybit);

	set_bit(KEY_MENU, onetouch->input_dev->keybit);
//	set_bit(KEY_HOME, onetouch->input_dev->keybit);
	set_bit(KEY_BACK, onetouch->input_dev->keybit);
//	set_bit(KEY_SEARCH, onetouch->input_dev->keybit);
	set_bit(KEY_REJECT, onetouch->input_dev->keybit);

	platform_set_drvdata(pdev, onetouch);

	err = input_register_device(input_dev);
	if (err)
	{
		pr_err("tegra_onetouch_probe: Unable to register input device\n");
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
        onetouch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        onetouch->early_suspend.suspend = tegra_onetouch_early_suspend;
        onetouch->early_suspend.resume = tegra_onetouch_late_resume;
        register_early_suspend(&onetouch->early_suspend);
#endif

	printk(KERN_INFO NVODM_TOUCH_NAME ": Successfully registered the ODM onetouch driver %x\n", (NvU32)onetouch->hOneTouchDevice);
	return 0;

	input_unregister_device(input_dev);
	
err_input_register_device_failed:
	NvOdmOneTouchDeviceClose(onetouch->hOneTouchDevice);
err_kthread_create_failed:
	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
err_open_failed:
	NvOdmOsSemaphoreDestroy(onetouch->semaphore);
err_semaphore_create_failed:
	kfree(onetouch);
	input_free_device(input_dev);
	return err;
}

static int tegra_onetouch_remove(struct platform_device *pdev)
{
	struct tegra_onetouch_driver_data *onetouch = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&onetouch->early_suspend);
#endif
	onetouch->shutdown = 1;


	NvOdmOneTouchDeviceClose(onetouch->hOneTouchDevice);

	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	input_unregister_device(onetouch->input_dev);
	/* NvOsSemaphoreDestroy(onetouch->semaphore); */
	input_unregister_device(onetouch->input_dev);
	kfree(onetouch);
	return 0;
}


static void tegra_onetouch_shutdown(struct  platform_device *pdev)
{
	struct tegra_onetouch_driver_data *onetouch = platform_get_drvdata(pdev);
 
	printk("[ONETOUCH] tegra_onetouch_shutdown() \n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&onetouch->early_suspend);
#endif
	onetouch->shutdown = 1;

	NvOdmOneTouchInterruptMask(onetouch->hOneTouchDevice, NV_TRUE);
	NvOdmOsSleepMS(50);


	NvOdmOsSemaphoreDestroy(onetouch->semaphore);
	NvOdmOneTouchDeviceClose(onetouch->hOneTouchDevice);

	input_unregister_device(onetouch->input_dev);
	kfree(onetouch);
	return;
}

static struct platform_driver tegra_onetouch_driver = {
	.probe	  = tegra_onetouch_probe,
	.remove	 = tegra_onetouch_remove,
	.shutdown = tegra_onetouch_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = tegra_onetouch_suspend,
	.resume	 = tegra_onetouch_resume,
#endif

	.driver	 = {
		.name   = "tegra_onetouch",
	},
};

static int __devinit tegra_onetouch_init(void)
{
	printk("[ONETOUCH] platform_driver_register\n");
	return platform_driver_register(&tegra_onetouch_driver);
}

static void __exit tegra_onetouch_exit(void)
{
	printk("[ONETOUCH] platform_driver_unregister\n");
	platform_driver_unregister(&tegra_onetouch_driver);
}

module_init(tegra_onetouch_init);
module_exit(tegra_onetouch_exit);

MODULE_DESCRIPTION("Tegra ODM onetouch driver");

