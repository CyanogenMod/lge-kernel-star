/*
 * drivers/power/tps80031_battery_gauge.c
 *
 * Gas Gauge driver for TI's tps80031
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps80031.h>

#define CONTROLLER_STAT1	0xe3
#define LINEAR_CHARGE_STS	0xde
#define STS_HW_CONDITIONS	0x21
#define TOGGLE1			0x90
#define GPCH0_LSB		0x3b
#define GPCH0_MSB		0x3c
#define GPSELECT_ISB		0x35
#define GPADC_CTRL		0x2e
#define MISC1			0xe4
#define CTRL_P1			0x36

#define TPS80031_VBUS_DET	BIT(2)
#define TPS80031_VAC_DET	BIT(3)
#define TPS80031_STS_VYSMIN_HI	BIT(4)
#define END_OF_CHARGE		BIT(5)

#define DRIVER_VERSION		"1.1.0"
#define BATTERY_POLL_PERIOD	30000

struct tps80031_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct power_supply	bat;
	struct power_supply	usb;
	struct timer_list	battery_poll_timer;
	uint32_t vsys;
	uint8_t usb_online;
	uint8_t usb_status;
	uint8_t capacity_sts;
	uint8_t health;
	uint8_t sys_vlow_intr;
};

static enum power_supply_property tps80031_bat_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property tps80031_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static uint32_t vsys_min_hi = 3700;

static int tps80031_battery_capacity(struct tps80031_device_info *di,
			union power_supply_propval *val)
{
	uint8_t hwsts, ret;

	ret = tps80031_read(di->dev->parent, SLAVE_ID2, LINEAR_CHARGE_STS,
								&hwsts);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, LINEAR_CHARGE_STS);
		return ret;
	}
	if (di->vsys < vsys_min_hi)
		di->capacity_sts = 10;
	else {
		if (hwsts & END_OF_CHARGE)
			di->capacity_sts = 100;
		else
			di->capacity_sts = 50;
	}

	if (di->sys_vlow_intr) {
		di->capacity_sts = 10;
		di->sys_vlow_intr = 0;
	}

	if (di->capacity_sts <= 10)
		di->health = POWER_SUPPLY_HEALTH_DEAD;
	else
		di->health = POWER_SUPPLY_HEALTH_GOOD;

	return  di->capacity_sts;
}

static int tps80031_gpadc_config(struct tps80031_device_info *di, uint8_t gpadc)
{

	uint8_t ret = 0;
	ret = tps80031_write(di->dev->parent, SLAVE_ID2, TOGGLE1, 0x02);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, TOGGLE1);
		return ret;
	}

	ret = tps80031_write(di->dev->parent, SLAVE_ID2, GPSELECT_ISB, gpadc);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, GPSELECT_ISB);
		return ret;
	}

	ret = tps80031_write(di->dev->parent, SLAVE_ID2, GPADC_CTRL, 0xef);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, GPADC_CTRL);
		return ret;
	}

	ret = tps80031_write(di->dev->parent, SLAVE_ID1, MISC1, 0x02);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, MISC1);
		return ret;
	}

	ret = tps80031_write(di->dev->parent, SLAVE_ID2, CTRL_P1, 0x08);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, CTRL_P1);
		return ret;
	}
	return ret;
	}

static int tps80031_battery_voltage(struct tps80031_device_info *di,
			union power_supply_propval *val)
{
	uint32_t voltage, ret = 0;
	uint8_t lsb, msb;

	ret = tps80031_gpadc_config(di, 0x07);
	if (ret < 0) {
		dev_err(di->dev, "Failed in gpadc config\n");
		return ret;
	}

	ret = tps80031_read(di->dev->parent, SLAVE_ID2, GPCH0_LSB, &lsb);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, GPCH0_LSB);
		return ret;
	}

	ret = tps80031_read(di->dev->parent, SLAVE_ID2, GPCH0_MSB, &msb);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, GPCH0_MSB);
		return ret;
	}

	voltage = ((msb << 8) | lsb);
	voltage = ((voltage * 1000) / 4096) * 5;

	di->vsys = voltage;

	return voltage;
}

#define to_tps80031_device_info_bat(x) container_of((x), \
				struct tps80031_device_info, bat);

static int tps80031_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_bat(psy);

	switch (psp) {

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->health;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval =  tps80031_battery_capacity(di, val);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval =  tps80031_battery_voltage(di, val);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->usb_status;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

#define to_tps80031_device_info_usb(x) container_of((x), \
				struct tps80031_device_info, usb);

static int tps80031_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct tps80031_device_info *di = to_tps80031_device_info_usb(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t tps80031_sys_vlow(int irq, void *data)
{
	struct tps80031_device_info *di = data;

	di->sys_vlow_intr = 1;
	power_supply_changed(&di->bat);
	return IRQ_HANDLED;
}

static irqreturn_t tps80031_vbus_det(int irq, void *data)
{
	uint8_t value, ret;
	struct tps80031_device_info *di = data;

	ret = tps80031_read(di->dev->parent, SLAVE_ID2, CONTROLLER_STAT1,
								&value);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, CONTROLLER_STAT1);
		return ret;
	}

	if ((value & TPS80031_VAC_DET) | (value & TPS80031_VBUS_DET)) {
		di->usb_status = POWER_SUPPLY_STATUS_CHARGING;
		di->health = POWER_SUPPLY_HEALTH_GOOD;
		di->usb_online = 1;
	} else {
		di->usb_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->usb_online = 0;
	}
	power_supply_changed(&di->usb);
	power_supply_changed(&di->bat);

	return IRQ_HANDLED;
}

static void battery_poll_timer_func(unsigned long pdi)
{
	struct tps80031_device_info *di = (void *)pdi;
	power_supply_changed(&di->bat);
	mod_timer(&di->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));
}

static int tps80031_battery_probe(struct platform_device *pdev)
{
	int err, ret = 0;
	uint8_t retval;
	struct device *dev = &pdev->dev;
	struct tps80031_device_info *di;
	struct tps80031_bg_platform_data *pdata = pdev->dev.platform_data;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(dev->parent, "failed to allocate device info data\n");
		ret = -ENOMEM;
	}

	di->dev =  &pdev->dev;

	ret = tps80031_read(di->dev->parent, SLAVE_ID2, CONTROLLER_STAT1,
								&retval);
	if (ret < 0) {
		dev_err(di->dev, "%s(): Failed in writin register 0x%02x\n",
			__func__, CONTROLLER_STAT1);
		return ret;
	}

	if ((retval & TPS80031_VAC_DET) | (retval & TPS80031_VBUS_DET)) {
		di->usb_status = POWER_SUPPLY_STATUS_CHARGING;
		di->usb_online = 1;
	} else {
		di->usb_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->usb_online = 0;
	}

	di->capacity_sts = 50;
	di->health = POWER_SUPPLY_HEALTH_GOOD;

	di->bat.name		= "tps80031-bat";
	di->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties	= tps80031_bat_props;
	di->bat.num_properties	= ARRAY_SIZE(tps80031_bat_props);
	di->bat.get_property	= tps80031_bat_get_property;

	retval = power_supply_register(dev->parent, &di->bat);
	if (retval)
		dev_err(dev->parent, "failed to register bat power supply\n");

	di->usb.name		= "tps80031-usb";
	di->usb.type		= POWER_SUPPLY_TYPE_USB;
	di->usb.properties	= tps80031_usb_props;
	di->usb.num_properties	= ARRAY_SIZE(tps80031_usb_props);
	di->usb.get_property	= tps80031_usb_get_property;

	retval = power_supply_register(dev->parent, &di->usb);
	if (retval)
		dev_err(dev->parent, "failed to register ac power supply\n");

	dev_set_drvdata(&pdev->dev, di);

	err = request_threaded_irq(pdata->irq_base + TPS80031_INT_VBUS_DET,
				NULL, tps80031_vbus_det,
					IRQF_ONESHOT, "tps80031_vbus_det", di);
	if (err < 0)
		dev_err(dev->parent, "request IRQ %d fail\n", pdata->irq_base);

	err = request_threaded_irq(pdata->irq_base + TPS80031_INT_SYS_VLOW,
				NULL, tps80031_sys_vlow,
					IRQF_ONESHOT, "tps80031_sys_vlow", di);
	if (err < 0)
		dev_err(dev->parent, "request IRQ %d fail\n", pdata->irq_base);

	setup_timer(&di->battery_poll_timer,
		battery_poll_timer_func, (unsigned long) di);
	mod_timer(&di->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));

	dev_info(dev->parent, "support ver. %s enabled\n", DRIVER_VERSION);

	return ret;
}

static int tps80031_battery_remove(struct platform_device *pdev)
{
	struct tps80031_device_info *di = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	kfree(di);

	return 0;
}

static struct platform_driver tps80031_battery_driver = {
	.driver	= {
		.name	= "tps80031-battery-gauge",
		.owner	= THIS_MODULE,
	},
	.probe	= tps80031_battery_probe,
	.remove = tps80031_battery_remove,
};

static int __init tps80031_battery_init(void)
{
	return platform_driver_register(&tps80031_battery_driver);
}

static void __exit tps80031_battery_exit(void)
{
	platform_driver_unregister(&tps80031_battery_driver);
}

module_init(tps80031_battery_init);
module_exit(tps80031_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com> ");
MODULE_DESCRIPTION("tps80031 battery gauge driver");
