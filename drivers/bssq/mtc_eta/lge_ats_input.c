/*
 *  Copyright (c) 2010 LGE.
 *  
 *  All source code in this file is licensed under the following license
 *  except where indicated.
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/input.h>

#define DRIVER_NAME "mtc_eta_input"

static struct input_dev *ats_input_dev;

/* add interface get ATS_INPUT_DEVICE [younchan.kim, 2010-06-11] */
struct input_dev *get_ats_input_dev(void)
{
	return ats_input_dev;
}
EXPORT_SYMBOL(get_ats_input_dev);
static int  __init ats_input_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i;
	/* XXX: what are correct values? */
	int fuzz_x = 0;
	int fuzz_y = 0;
	int fuzz_w = 0;

	ats_input_dev = input_allocate_device();
	if (!ats_input_dev) {
		printk(KERN_ERR "%s: not enough memory for input device\n", __func__);
		return -ENOMEM;
	}
	ats_input_dev->name = "ats_input";

	for(i=0; i<EV_CNT; i++)
		set_bit(i, ats_input_dev->evbit);
	for(i=0; i<KEY_CNT; i++)
		set_bit(i, ats_input_dev->keybit);
	set_bit(ABS_MT_TOUCH_MAJOR, ats_input_dev->absbit);
	clear_bit(EV_REP, ats_input_dev->evbit);

	rc = input_register_device(ats_input_dev);
	if (rc)
		printk(KERN_ERR"%s : input_register_device failed\n", __func__);

	/* FIXME: Touch resolution should be given by platform data */
	input_set_abs_params(ats_input_dev, ABS_MT_POSITION_X, 0, 986, fuzz_x, 0);
        input_set_abs_params(ats_input_dev, ABS_MT_POSITION_Y, 0, 1644, fuzz_y, 0);
        input_set_abs_params(ats_input_dev, ABS_MT_TOUCH_MAJOR, 0, 15, fuzz_w, 0);
        input_set_abs_params(ats_input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, fuzz_w, 0);

	return rc;
}

static int ats_input_remove(struct platform_device *pdev)
{
	input_unregister_device(ats_input_dev);
	return 0;
}

static struct platform_driver ats_input_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	 = ats_input_probe,
	.remove = ats_input_remove,
};

static int __init ats_input_init(void)
{
	return platform_driver_register(&ats_input_driver);
}


static void __exit ats_input_exit(void)
{
	platform_driver_unregister(&ats_input_driver);
}

module_init(ats_input_init);
module_exit(ats_input_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("ATS_INPUT driver");
MODULE_LICENSE("GPL v2");
