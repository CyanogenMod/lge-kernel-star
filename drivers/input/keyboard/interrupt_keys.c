/*
 * drivers/input/keyboard/interrupt_keys.c
 * Key driver for keys directly connected to interrupt lines.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */


#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt_keys.h>
#include <linux/spinlock.h>

enum {
	KEY_RELEASED = 0,
	KEY_PRESSED,
};

struct interrupt_button_data {
	struct interrupt_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	int timer_debounce;	/* in msecs */
	bool disabled;
	int key_state;
	spinlock_t lock;
};

struct interrupt_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_int_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct interrupt_button_data data[0];
};

static void interrupt_keys_timer(unsigned long _data)
{
	struct interrupt_button_data *bdata =
			(struct interrupt_button_data *)_data;
	struct interrupt_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	unsigned long iflags;

	spin_lock_irqsave(&bdata->lock, iflags);
	if (bdata->key_state == KEY_PRESSED) {
		input_event(input, type, button->code, 0);
		input_sync(input);
		bdata->key_state = KEY_RELEASED;
	} else
		dev_info(&input->dev, "Key state is in release, not sending "
					"any event\n");
	spin_unlock_irqrestore(&bdata->lock, iflags);
	return;
}

static irqreturn_t interrupt_keys_isr(int irq, void *dev_id)
{
	struct interrupt_button_data *bdata = dev_id;
	struct interrupt_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	unsigned long iflags;

	BUG_ON(irq != button->irq);

	spin_lock_irqsave(&bdata->lock, iflags);
	if (bdata->key_state == KEY_RELEASED) {
		input_event(input, type, button->code, 1);
		input_sync(input);
		if (!bdata->timer_debounce) {
			input_event(input, type, button->code, 0);
			input_sync(input);
			spin_unlock_irqrestore(&bdata->lock, iflags);
			return IRQ_HANDLED;
		}
		bdata->key_state = KEY_PRESSED;
	}

	if ((bdata->key_state == KEY_PRESSED) && (bdata->timer_debounce)) {
		spin_unlock_irqrestore(&bdata->lock, iflags);
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&bdata->lock, iflags);

	/* Should not reach to this point */
	WARN_ON(1);
	return IRQ_HANDLED;
}

static int __devinit interrupt_keys_setup_key(struct platform_device *pdev,
					 struct interrupt_button_data *bdata,
					 struct interrupt_keys_button *button)
{
	char *desc = button->desc ? button->desc : "int_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	setup_timer(&bdata->timer, interrupt_keys_timer, (unsigned long)bdata);
	spin_lock_init(&bdata->lock);

	irq = button->irq;
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Invalid irq number %d\n", button->irq);
		goto fail;
	}

	irqflags = 0;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_threaded_irq(irq, NULL, interrupt_keys_isr,
						irqflags, desc, bdata);
	if (error) {
		dev_err(dev, "Unable to register irq %d; error %d\n",
			irq, error);
		goto fail;
	}
	return 0;

fail:
	return error;
}

static int interrupt_keys_open(struct input_dev *input)
{
	struct interrupt_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void interrupt_keys_close(struct input_dev *input)
{
	struct interrupt_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static int __devinit interrupt_keys_probe(struct platform_device *pdev)
{
	struct interrupt_keys_platform_data *pdata = pdev->dev.platform_data;
	struct interrupt_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	ddata = kzalloc(sizeof(struct interrupt_keys_drvdata) +
			pdata->nbuttons * sizeof(struct interrupt_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_int_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->phys = "int-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = interrupt_keys_open;
	input->close = interrupt_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct interrupt_keys_button *button = &pdata->int_buttons[i];
		struct interrupt_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
		bdata->timer_debounce = button->debounce_interval;

		error = interrupt_keys_setup_key(pdev, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

fail2:
	while (--i >= 0) {
		free_irq(pdata->int_buttons[i].irq, &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
	}

	platform_set_drvdata(pdev, NULL);
fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit interrupt_keys_remove(struct platform_device *pdev)
{
	struct interrupt_keys_platform_data *pdata = pdev->dev.platform_data;
	struct interrupt_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		free_irq(pdata->int_buttons[i].irq, &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int interrupt_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct interrupt_keys_platform_data *pdata = pdev->dev.platform_data;
	struct interrupt_keys_button *button;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			button = &pdata->int_buttons[i];
			if (button->wakeup)
				enable_irq_wake(button->irq);
		}
	}
	return 0;
}

static int interrupt_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct interrupt_keys_platform_data *pdata = pdev->dev.platform_data;
	struct interrupt_keys_button *button;
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->int_buttons[i];
		if (button->wakeup && device_may_wakeup(&pdev->dev)) {
			int irq = button->irq;
			disable_irq_wake(irq);
		}
	}
	return 0;
}

static const struct dev_pm_ops interrupt_keys_pm_ops = {
	.suspend	= interrupt_keys_suspend,
	.resume		= interrupt_keys_resume,
};
#endif

static struct platform_driver interrupt_keys_device_driver = {
	.probe		= interrupt_keys_probe,
	.remove		= __devexit_p(interrupt_keys_remove),
	.driver		= {
		.name	= "interrupt-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &interrupt_keys_pm_ops,
#endif
	}
};

static int __init interrupt_keys_init(void)
{
	return platform_driver_register(&interrupt_keys_device_driver);
}

static void __exit interrupt_keys_exit(void)
{
	platform_driver_unregister(&interrupt_keys_device_driver);
}

module_init(interrupt_keys_init);
module_exit(interrupt_keys_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Keyboard driver for CPU interrupts");
MODULE_ALIAS("platform:interrupt-keys");
