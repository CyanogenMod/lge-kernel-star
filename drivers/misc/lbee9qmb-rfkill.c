/*
 * Bluetooth+WiFi Murata LBEE19QMBC rfkill power control via GPIO
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/lbee9qmb-rfkill.h>

static int lbee9qmb_rfkill_set_power(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct regulator *regulator;

	regulator = regulator_get(&pdev->dev, "Vdd");

	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "Unable to get regulator Vdd\n");
		return PTR_ERR(regulator);
	}

	if (!blocked) {
		regulator_enable(regulator);
		gpio_set_value(plat->gpio_reset, 0);
		if (plat->gpio_pwr!=-1)
			gpio_set_value(plat->gpio_pwr, 0);
		msleep(plat->delay);
		if (plat->gpio_pwr!=-1)
			gpio_set_value(plat->gpio_pwr, 1);
		gpio_set_value(plat->gpio_reset, 1);
	} else {
		gpio_set_value(plat->gpio_reset, 0);
		regulator_disable(regulator);
	}

	regulator_put(regulator);
	return 0;
}

static struct rfkill_ops lbee9qmb_rfkill_ops = {
	.set_block = lbee9qmb_rfkill_set_power,
};

static int lbee9qmb_rfkill_probe(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill;

	int rc;

	if (!plat) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOSYS;
	}

	rc = gpio_request(plat->gpio_reset, "lbee9qmb_reset");
	if (rc < 0) {
		dev_err(&pdev->dev, "gpio_request failed\n");
		return rc;
	}
	if (plat->gpio_pwr!=-1)
	{
		rc = gpio_request(plat->gpio_pwr, "lbee9qmb_pwr");
		gpio_direction_output(plat->gpio_pwr,0);
	}

	rfkill = rfkill_alloc("lbee9qmb-rfkill", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &lbee9qmb_rfkill_ops, pdev);
	if (!rfkill) {
		rc = -ENOMEM;
		goto fail_gpio;
	}
	platform_set_drvdata(pdev, rfkill);
	gpio_direction_output(plat->gpio_reset, 0);
	
	rc = rfkill_register(rfkill);
	if (rc < 0)
		goto fail_alloc;

	return 0;

fail_alloc:
	rfkill_destroy(rfkill);
fail_gpio:
	gpio_free(plat->gpio_reset);
	if (plat->gpio_pwr!=-1)
		gpio_free(plat->gpio_pwr);
	return rc;
		
}

static int lbee9qmb_rfkill_remove(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	gpio_free(plat->gpio_reset);
	if (plat->gpio_pwr!=-1)
		gpio_free(plat->gpio_pwr);
	return 0;
	
}

static struct platform_driver lbee9qmb_rfkill_driver = {
	.probe = lbee9qmb_rfkill_probe,
	.remove = lbee9qmb_rfkill_remove,
	.driver = {
		.name = "lbee9qmb-rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init lbee9qmb_rfkill_init(void)
{
	return platform_driver_register(&lbee9qmb_rfkill_driver);
}

static void __exit lbee9qmb_rfkill_exit(void)
{
	platform_driver_unregister(&lbee9qmb_rfkill_driver);
}

module_init(lbee9qmb_rfkill_init);
module_exit(lbee9qmb_rfkill_exit);

MODULE_DESCRIPTION("Murata LBEE9QMBC rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
