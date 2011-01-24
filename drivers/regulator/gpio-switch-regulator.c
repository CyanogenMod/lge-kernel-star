/*
 * driver/regulator/gpio-switch-regulator.c
 * GPIO based switch regulator to enable/disable power rails.
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

/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/gpio-switch-regulator.h>

struct gpio_switch_regulator {
	struct regulator_desc		reg_desc;
	struct regulator_init_data	reg_init_data;
	struct regulator		*input_regulator;
	struct regulator_dev		*rdev;
	struct device			*dev;
	int				gpio_nr;
	bool				is_gpio_init;
	bool				is_enable;
	bool				active_low;
	int				*voltages;
	void				*pdata;
	unsigned			curr_vol_sel;
};

static int gpio_switch_list_voltage(struct regulator_dev *rdev,
				     unsigned selector)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);

	if (selector < ri->reg_desc.n_voltages)
		return ri->voltages[selector] * 1000;
	else
		return 0;
}

static int gpio_switch_set_voltage(struct regulator_dev *rdev,
				    int min_uV, int max_uV)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int uV;
	bool found = false;
	unsigned val;

	for (val = 0; val < ri->reg_desc.n_voltages; val++) {
		uV = ri->voltages[val] * 1000;
		if (min_uV <= uV && uV <= max_uV) {
			found = true;
			ri->curr_vol_sel = val;
			break;
		}
	}
	if (found && ri->input_regulator)
		return regulator_set_voltage(ri->input_regulator, min_uV,
			max_uV);
	ri->curr_vol_sel = 0;
	return -EINVAL;
}

static int gpio_switch_get_voltage(struct regulator_dev *rdev)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	if (ri->input_regulator)
		return regulator_get_voltage(ri->input_regulator);

	if (ri->curr_vol_sel < ri->reg_desc.n_voltages)
		return ri->voltages[ri->curr_vol_sel];
	return 0;
}

static int gpio_switch_regulator_enable(struct regulator_dev *rdev)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int ret = 0;
	if (ri->is_enable)
		return 0;

	if (ri->input_regulator) {
		ret = regulator_enable(ri->input_regulator);
		if (ret < 0) {
			dev_err(&rdev->dev, "%s:Failed to enable regulator"
				" Error %d\n", __func__, ret);
			return ret;
		}
	}

	gpio_set_value(ri->gpio_nr, (ri->active_low) ? 0 : 1);
	ri->is_enable = true;
	return 0;
}

static int gpio_switch_regulator_disable(struct regulator_dev *rdev)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int ret = 0;

	if (!ri->is_enable)
		return 0;

	gpio_set_value(ri->gpio_nr, (ri->active_low) ? 1 : 0);
	if (ri->input_regulator) {
		ret = regulator_disable(ri->input_regulator);
		if (ret < 0) {
			dev_err(&rdev->dev, "%s:Failed to disable regulator"
				" Error %d\n", __func__, ret);
			return ret;
		}
	}

	ri->is_enable = false;
	return 0;
}

static int gpio_switch_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int ret = 0;
	if (ri->input_regulator) {
		ret = regulator_is_enabled(ri->input_regulator);
		if (!ret)
			return !ret;
	}
	return (ri->is_enable) ? 1 : 0;
}

static struct regulator_ops gpio_switch_regulator_ops = {
	.list_voltage	= gpio_switch_list_voltage,
	.get_voltage	= gpio_switch_get_voltage,
	.set_voltage	= gpio_switch_set_voltage,
	.is_enabled	= gpio_switch_regulator_is_enabled,
	.enable		= gpio_switch_regulator_enable,
	.disable	= gpio_switch_regulator_disable,
};

static int __devinit gpio_switch_regulator_probe(struct platform_device *pdev)
{
	struct gpio_switch_regulator *ri = NULL;
	struct gpio_switch_regulator_platform_data *pdata;
	int id = pdev->id;
	int ret = 0;

	dev_dbg(&pdev->dev, "Probing regulator %d\n", id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "%s:No platform data Exiting\n", __func__);
		return -ENODEV;
	}

	ri = kzalloc(sizeof(struct gpio_switch_regulator), GFP_KERNEL);
	if (!ri) {
		dev_err(&pdev->dev, "%s:Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	/* Initialize the regulator parameter */
	ri->reg_desc.name = pdata->regulator_name;
	ri->reg_desc.ops = &gpio_switch_regulator_ops;
	ri->reg_desc.type = REGULATOR_VOLTAGE;
	ri->reg_desc.id = pdev->id;
	ri->reg_desc.n_voltages = pdata->n_voltages;
	ri->reg_desc.owner = THIS_MODULE;

	memcpy(&ri->reg_init_data.constraints, &pdata->constraints,
			sizeof(struct regulation_constraints));

	/* Initialize min and maximum contraint voltage if it is not
	 * define in platform device */
	if (!pdata->constraints.min_uV)
		ri->reg_init_data.constraints.min_uV = pdata->voltages[0] *1000;

	if (!pdata->constraints.max_uV)
		ri->reg_init_data.constraints.max_uV =
			pdata->voltages[pdata->n_voltages - 1] *1000;

	ri->reg_init_data.num_consumer_supplies =
					pdata->num_consumer_supplies;
	ri->reg_init_data.consumer_supplies = pdata->consumer_supplies;

	ri->input_regulator = NULL;
	ri->is_gpio_init = false;
	ri->is_enable = false;
	ri->voltages = pdata->voltages;
	ri->pdata = pdev->dev.platform_data;
	ri->gpio_nr = pdata->gpio_nr;
	ri->active_low = pdata->active_low;
	ri->dev = &pdev->dev;

	/* Get the regulator structure if input supply is available */
	if (pdata->input_supply) {
		ri->input_regulator = regulator_get(NULL, pdata->input_supply);
		if (IS_ERR_OR_NULL(ri->input_regulator)) {
			dev_err(&pdev->dev, "Unable to get regulator %s\n",
					pdata->input_supply);
			ret = -ENODEV;
			goto reg_get_fail;
		}
	}

	/* Initialize gpios */
	ret = gpio_request(ri->gpio_nr, pdata->regulator_name);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to request gpio %d\n", ri->gpio_nr);
		goto gpio_init_fail;
	}

	ret = gpio_direction_output(ri->gpio_nr, (ri->active_low) ? 1 : 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to set direction %d\n",
							 ri->gpio_nr);
		goto gpio_dir_fail;
	}
	ri->is_gpio_init = true;

	ri->rdev = regulator_register(&ri->reg_desc, &pdev->dev,
				&ri->reg_init_data, ri);
	if (IS_ERR_OR_NULL(ri->rdev)) {
		dev_err(&pdev->dev, "Failed to register regulator %s\n",
						ri->reg_desc.name);
		ret = PTR_ERR(ri->rdev);
		goto gpio_dir_fail;
	}
	platform_set_drvdata(pdev, ri);
	return 0;

gpio_dir_fail:
	gpio_free(ri->gpio_nr);
gpio_init_fail:
	if (ri->input_regulator)
		regulator_put(ri->input_regulator);
reg_get_fail:
	kfree(ri);
	return ret;
}

static int __devexit gpio_switch_regulator_remove(struct platform_device *pdev)
{
	struct gpio_switch_regulator *ri = platform_get_drvdata(pdev);;

	regulator_unregister(ri->rdev);
	gpio_free(ri->gpio_nr);
	if (ri->input_regulator)
		regulator_put(ri->input_regulator);
	kfree(ri);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver gpio_switch_regulator_driver = {
	.driver	= {
		.name	= "gpio-switch-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= gpio_switch_regulator_probe,
	.remove		= __devexit_p(gpio_switch_regulator_remove),
};

static int __init gpio_switch_regulator_init(void)
{
	return platform_driver_register(&gpio_switch_regulator_driver);
}

static void __exit gpio_switch_regulator_exit(void)
{
	platform_driver_unregister(&gpio_switch_regulator_driver);
}

subsys_initcall_sync(gpio_switch_regulator_init);
module_exit(gpio_switch_regulator_exit);
