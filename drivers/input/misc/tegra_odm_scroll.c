/*
 * drivers/input/misc/tegra_odm_scroll.c
 *
 * Scrollwheel input device using NVIDIA Tegra ODM kit
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

#define NV_DEBUG 0

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>

#include <mach/nvrm_linux.h>
#include <nvos.h>

#include <nvodm_scrollwheel.h>

struct tegra_scroll_dev {
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOdmOsSemaphoreHandle	sem;
	NvOdmScrollWheelHandle	odm_dev;
};

static int scroll_thread(void *pdata)
{
	struct tegra_scroll_dev *scroll = pdata;
	NvOdmScrollWheelEvent Events;

	/* FIXME add a terminating condition */
	while(1) {
		NvOdmOsSemaphoreWait(scroll->sem);

		Events = NvOdmScrollWheelGetEvent(scroll->odm_dev);
		if (Events & NvOdmScrollWheelEvent_RotateAntiClockWise) {
			input_report_key(scroll->input_dev, KEY_UP, 1);
			input_report_key(scroll->input_dev, KEY_UP, 0);
		}
		if (Events & NvOdmScrollWheelEvent_RotateClockWise) {
			input_report_key(scroll->input_dev, KEY_DOWN, 1);
			input_report_key(scroll->input_dev, KEY_DOWN, 0);
		}

		if (Events & NvOdmScrollWheelEvent_Press)
			input_report_key(scroll->input_dev, KEY_ENTER, 1);
		else if (Events & NvOdmScrollWheelEvent_Release)
			input_report_key(scroll->input_dev, KEY_ENTER, 0);
	}
	return 0;
}

static int __init tegra_scroll_probe(struct platform_device *pdev)
{
	struct tegra_scroll_dev *scroll = NULL;
	struct input_dev *input_dev = NULL;
	NvOdmOsSemaphoreHandle sem = NULL;
	NvOdmScrollWheelHandle odm_dev = NULL;
	int err;
	NvOdmScrollWheelEvent events;

	sem = NvOdmOsSemaphoreCreate(0);
	if (!sem) {
		pr_err("tegra_scroll_probe: Semaphore creation failed\n");
		return -ENOMEM;
	}

	events = NvOdmScrollWheelEvent_Press |
		NvOdmScrollWheelEvent_Release  |
		NvOdmScrollWheelEvent_RotateAntiClockWise |
		NvOdmScrollWheelEvent_RotateClockWise;

	odm_dev = NvOdmScrollWheelOpen(sem, events);
	if (odm_dev == NULL) {
		pr_err("tegra_scroll_probe: scroll wheel not found\n");
		err = -ENXIO;
		goto err_scrollWheelnotfound;
	}

	scroll = kzalloc(sizeof(struct tegra_scroll_dev), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (input_dev == NULL || scroll == NULL) {
		err = -ENOMEM;
		pr_err("tegra_scroll_probe: Failed to allocate input device\n");
		goto err_alloc_failed;
	}

	scroll->input_dev = input_dev;
	scroll->sem = sem;
	scroll->odm_dev = odm_dev;
	scroll->task = kthread_create(scroll_thread, scroll, "tegra_scroll");
	if (scroll->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	wake_up_process( scroll->task );

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_ENTER, input_dev->keybit);
	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_DOWN, input_dev->keybit);

	platform_set_drvdata(pdev, scroll);
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_scroll_probe: Unable to register input device\n");
		goto err_input_register_device_failed;
	}

	return 0;

err_input_register_device_failed:
	NvOdmScrollWheelClose(scroll->odm_dev);
err_kthread_create_failed:
	/* What to do? */
err_alloc_failed:
	kfree(scroll);
	input_free_device(input_dev);
	NvOdmScrollWheelClose(odm_dev);
err_scrollWheelnotfound:
	NvOdmOsSemaphoreDestroy(sem);
	return err;
}

static int tegra_scroll_remove(struct platform_device *pdev)
{
	struct tegra_scroll_dev *scroll = platform_get_drvdata(pdev);

	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	input_unregister_device(scroll->input_dev);
	NvOdmScrollWheelClose(scroll->odm_dev);
	NvOdmOsSemaphoreDestroy(scroll->sem);
	kfree(scroll);
	return 0;
}

static struct platform_driver tegra_scroll_driver = {
	.probe	= tegra_scroll_probe,
	.remove	= tegra_scroll_remove,
	.driver	= {
		.name = "tegra_scrollwheel",
	},
};

static int __devinit tegra_scroll_init(void)
{
	return platform_driver_register(&tegra_scroll_driver);
}

static void __exit tegra_scroll_exit(void)
{
	platform_driver_unregister(&tegra_scroll_driver);
}

module_init(tegra_scroll_init);
module_exit(tegra_scroll_exit);

MODULE_DESCRIPTION("Tegra scrollwheel driver");
