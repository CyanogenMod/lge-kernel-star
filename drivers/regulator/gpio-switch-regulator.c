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
#include <linux/regulator/gpio-switch-regulator.h>

struct gpio_switch_regulator {
	struct regulator_desc		reg_desc;
	struct regulator_init_data	reg_init_data;
	struct regulator		*input_regulator;
	struct regulator_dev		*rdev;
	struct device			*dev;
	int				gpio_nr;
	int				pin_group;
	bool				is_gpio_init;
	bool				is_enable;
	bool				active_low;
	bool				is_init_success;
	int				*voltages;
	unsigned			curr_vol_sel;
	struct gpio_switch_regulator_subdev_data *psubdev_data;
	int (*enable_rail)(struct gpio_switch_regulator_subdev_data *sdata);
	int (*disable_rail)(struct gpio_switch_regulator_subdev_data *sdata);
};

static int _gpio_regulator_enable(struct device *dev,
		struct gpio_switch_regulator *ri)
{
	int init_val;
	int ret;

	if (ri->enable_rail) {
		ret = ri->enable_rail(ri->psubdev_data);
		if (ret < 0)
			dev_err(dev, "Unable to enable rail through board api"
			" error %d\n", ret);
	} else {
		init_val = (ri->active_low) ? 0 : 1;
		ret = gpio_direction_output(ri->gpio_nr, init_val);
		if (ret < 0)
			dev_err(dev, "Unable to set direction %d\n",
				ri->gpio_nr);
	}
	return ret;
}

static int _gpio_regulator_disable(struct device *dev,
		struct gpio_switch_regulator *ri)
{
	int init_val;
	int ret;

	if (ri->disable_rail) {
		ret = ri->disable_rail(ri->psubdev_data);
		if (ret < 0)
			dev_err(dev, "Unable to disable rail through "
				"board api %d\n", ret);
	} else {
		init_val = (ri->active_low) ? 1 : 0;
		ret = gpio_direction_output(ri->gpio_nr, init_val);
		if (ret < 0)
			dev_err(dev, "Unable to set direction %d\n",
				ri->gpio_nr);
	}
	return ret;
}

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
				   int min_uV, int max_uV, unsigned *selector)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int uV;
	bool found = false;
	unsigned val;

	for (val = 0; val < ri->reg_desc.n_voltages; val++) {
		uV = ri->voltages[val] * 1000;
		if (min_uV <= uV && uV <= max_uV) {
			found = true;
			*selector = ri->curr_vol_sel = val;
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
		return ri->voltages[ri->curr_vol_sel] * 1000;
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

	ret = _gpio_regulator_enable(&rdev->dev, ri);
	if (ret < 0)
		return ret;
	ri->is_enable = true;
	return 0;
}

static int gpio_switch_regulator_disable(struct regulator_dev *rdev)
{
	struct gpio_switch_regulator *ri = rdev_get_drvdata(rdev);
	int ret = 0;

	if (!ri->is_enable)
		return 0;

	ret = _gpio_regulator_disable(&rdev->dev, ri);
	if (ret < 0)
		return ret;

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
			return ret;
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
	struct gpio_switch_regulator *gswitch_reg = NULL;
	struct gpio_switch_regulator_platform_data *pdata;
	struct gpio_switch_regulator_subdev_data *sdata = NULL;
	int id = pdev->id;
	int ret = 0;
	int rcount;

	dev_dbg(&pdev->dev, "Probing regulator %d\n", id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "%s:No platform data Exiting\n", __func__);
		return -ENODEV;
	}

	BUG_ON(!pdata->num_subdevs);

	gswitch_reg = kzalloc(sizeof(struct gpio_switch_regulator) *
				pdata->num_subdevs, GFP_KERNEL);
	if (!gswitch_reg) {
		dev_err(&pdev->dev, "%s:Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	for (rcount = 0; rcount < pdata->num_subdevs; ++rcount) {
		ri = &gswitch_reg[rcount];
		sdata = pdata->subdevs[rcount];

		/* Initialize the regulator parameter */
		ri->reg_desc.name = sdata->regulator_name;
		ri->reg_desc.ops = &gpio_switch_regulator_ops;
		ri->reg_desc.type = REGULATOR_VOLTAGE;
		ri->reg_desc.id = sdata->id;
		ri->reg_desc.n_voltages = sdata->n_voltages;
		ri->reg_desc.owner = THIS_MODULE;
		ri->is_init_success = false;

		memcpy(&ri->reg_init_data.constraints, &sdata->constraints,
				sizeof(struct regulation_constraints));

		/* Initialize min and maximum contraint voltage if it is not
		 * define in platform device */
		if (!sdata->constraints.min_uV)
			ri->reg_init_data.constraints.min_uV = 1000 *
						sdata->voltages[0];

		if (!sdata->constraints.max_uV)
			ri->reg_init_data.constraints.max_uV = 1000 *
					sdata->voltages[sdata->n_voltages - 1];

		ri->reg_init_data.num_consumer_supplies =
						sdata->num_consumer_supplies;
		ri->reg_init_data.consumer_supplies = sdata->consumer_supplies;

		ri->input_regulator = NULL;
		ri->is_gpio_init = false;
		ri->is_enable = (sdata->init_state) ? true : false;
		ri->voltages = sdata->voltages;
		ri->psubdev_data = sdata;
		ri->gpio_nr = sdata->gpio_nr;
		ri->active_low = sdata->active_low;
		ri->dev = &pdev->dev;
		ri->enable_rail = sdata->enable_rail;
		ri->disable_rail = sdata->disable_rail;
		ri->pin_group = sdata->pin_group;

		/* Checking for board APIs enable/disable rail */
		if (ri->enable_rail || ri->disable_rail)
			BUG_ON(!(ri->enable_rail && ri->disable_rail));

		/* Get the regulator structure if input supply is available */
		if (sdata->input_supply) {
			ri->input_regulator = regulator_get(NULL,
							sdata->input_supply);
			if (IS_ERR_OR_NULL(ri->input_regulator)) {
				dev_err(&pdev->dev, "Unable to get regu"
					"lator %s\n", sdata->input_supply);
				ret = -ENODEV;
				goto reg_get_fail;
			}
			if (ri->is_enable) {
				ret = regulator_enable(ri->input_regulator);
				if (ret < 0) {
					dev_err(&pdev->dev, "Unable to enable "
						"regulator %s\n",
						sdata->input_supply);
					goto reg_en_fail;
				}
			}
		}

		/* Initialize gpios */
		ret = gpio_request(ri->gpio_nr, sdata->regulator_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Unable to request gpio %d\n",
					ri->gpio_nr);
			goto gpio_req_fail;
		}

		if (ri->is_enable)
			ret = _gpio_regulator_enable(&pdev->dev, ri);
		else
			ret = _gpio_regulator_disable(&pdev->dev, ri);
		if (ret < 0)
			goto reg_cont_fail;

		ri->is_gpio_init = true;

		ri->rdev = regulator_register(&ri->reg_desc, &pdev->dev,
					&ri->reg_init_data, ri);
		if (IS_ERR_OR_NULL(ri->rdev)) {
			dev_err(&pdev->dev, "Failed to register regulator %s\n",
							ri->reg_desc.name);
			ret = PTR_ERR(ri->rdev);
			goto reg_reg_fail;
		}

		/* If everything success then continue for next registration */
		ri->is_init_success = true;
		continue;

		/* Cleanup the current registration and continue for next
		 * registration*/
reg_reg_fail:
		if (ri->is_enable)
			_gpio_regulator_disable(&pdev->dev, ri);
reg_cont_fail:
		gpio_free(ri->gpio_nr);
gpio_req_fail:
		if (ri->is_enable && ri->input_regulator)
			regulator_disable(ri->input_regulator);
reg_en_fail:
		if (ri->input_regulator) {
			regulator_put(ri->input_regulator);
			ri->input_regulator = NULL;
		}
reg_get_fail:
		dev_err(&pdev->dev, "Unable to register regulator %s\n",
							sdata->regulator_name);
	}

	platform_set_drvdata(pdev, gswitch_reg);
	return 0;
}

static int __devexit gpio_switch_regulator_remove(struct platform_device *pdev)
{
	struct gpio_switch_regulator *ri = NULL;
	struct gpio_switch_regulator *gswitch_reg = platform_get_drvdata(pdev);
	int i;
	struct gpio_switch_regulator_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	/* Unregister devices in reverse order */
	for (i = pdata->num_subdevs; i; --i) {
		ri = &gswitch_reg[i - 1];
		/* If registration was not success, then do not release */
		if (!ri->is_init_success)
			continue;

		if (ri->is_enable)
			_gpio_regulator_disable(&pdev->dev, ri);

		if (ri->input_regulator) {
			if (ri->is_enable)
				regulator_disable(ri->input_regulator);
			regulator_put(ri->input_regulator);
		}

		regulator_unregister(ri->rdev);
		gpio_free(ri->gpio_nr);
	}

	kfree(gswitch_reg);
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
