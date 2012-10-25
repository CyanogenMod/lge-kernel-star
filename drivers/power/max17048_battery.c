/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2012 Nvidia Cooperation
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/max17048_battery.h>

#define MAX17048_VCELL_MSB	0x02
#define MAX17048_VCELL_LSB	0x03
#define MAX17048_SOC_MSB	0x04
#define MAX17048_SOC_LSB	0x05
#define MAX17048_MODE_MSB	0x06
#define MAX17048_MODE_LSB	0x07
#define MAX17048_VER_MSB	0x08
#define MAX17048_VER_LSB	0x09
#define MAX17048_RCOMP_MSB	0x0C
#define MAX17048_RCOMP_LSB	0x0D
#define MAX17048_CMD_MSB	0xFE
#define MAX17048_CMD_LSB	0xFF
#define RESET_MSB 0x54
#define RESET_LSB 0x00
#define MAX17048_DELAY		1000
#define MAX17048_BATTERY_FULL	95

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	struct max17048_platform_data	*pdata;

	/* State Of Connect */
	int ac_online;
	int usb_online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;

	int lasttime_vcell;
	int lasttime_soc;
	int lasttime_status;
	int lasttime_ac_online;
	int lasttime_usb_online;
};

static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
	return -EINVAL;
	}
	return 0;
}

static int max17048_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->ac_online;
	else
		return -EINVAL;

	return 0;
}

static int max17048_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
					struct max17048_chip, usb);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->usb_online;
	else
		return -EINVAL;

	return 0;
}

static int max17048_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17048_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void max17048_reset(struct i2c_client *client)
{
	max17048_write_reg(client, MAX17048_CMD_MSB, RESET_MSB);
	max17048_write_reg(client, MAX17048_CMD_LSB, RESET_LSB);
}

static void max17048_get_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17048_read_reg(client, MAX17048_VCELL_MSB);
	lsb = max17048_read_reg(client, MAX17048_VCELL_LSB);

	chip->vcell = (msb << 4) + (lsb >> 4);
}

static void max17048_get_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17048_read_reg(client, MAX17048_SOC_MSB);

	chip->soc = msb;
}

static void max17048_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;

	msb = max17048_read_reg(client, MAX17048_VER_MSB);
	lsb = max17048_read_reg(client, MAX17048_VER_LSB);

	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver %d%d\n", msb, lsb);
}

static void max17048_get_online(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->charging_status && chip->pdata->charging_status()) {
		if (chip->pdata->charger_online &&
					chip->pdata->charger_online()) {
			chip->ac_online = 1;
			chip->usb_online = 0;
		} else {
			chip->ac_online = 0;
			chip->usb_online = 1;
		}
	} else {
		chip->ac_online = 0;
		chip->usb_online = 0;
	}
}

static void max17048_get_status(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->charging_status())
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (chip->soc > MAX17048_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;

	chip = container_of(work, struct max17048_chip, work.work);

	max17048_get_vcell(chip->client);
	max17048_get_soc(chip->client);
	max17048_get_online(chip->client);
	max17048_get_status(chip->client);

	if (chip->vcell != chip->lasttime_vcell ||
		chip->soc != chip->lasttime_soc ||
		chip->ac_online != chip->lasttime_ac_online ||
		chip->usb_online != chip->lasttime_usb_online ||
		chip->status !=	chip->lasttime_status) {

		chip->lasttime_vcell = chip->vcell;
		chip->lasttime_soc = chip->soc;
		chip->lasttime_status = chip->status;
		chip->lasttime_ac_online = chip->ac_online;
		chip->lasttime_usb_online = chip->usb_online;

		power_supply_changed(&chip->battery);
		power_supply_changed(&chip->usb);
		power_supply_changed(&chip->ac);
	}
	schedule_delayed_work(&chip->work, MAX17048_DELAY);
}

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property max17048_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property max17048_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int __devinit max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17048_chip *chip;
	int ret;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	chip->ac_online = 0;
	chip->usb_online = 0;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17048_get_property;
	chip->battery.properties	= max17048_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17048_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error2;
	}

	chip->ac.name		= "maxim-ac";
	chip->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	chip->ac.get_property	= max17048_ac_get_property;
	chip->ac.properties	= max17048_ac_props;
	chip->ac.num_properties	= ARRAY_SIZE(max17048_ac_props);

	ret = power_supply_register(&client->dev, &chip->ac);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error1;
	}

	chip->usb.name		= "maxim-usb";
	chip->usb.type		= POWER_SUPPLY_TYPE_USB;
	chip->usb.get_property	= max17048_usb_get_property;
	chip->usb.properties	= max17048_usb_props;
	chip->usb.num_properties	= ARRAY_SIZE(max17048_usb_props);

	ret = power_supply_register(&client->dev, &chip->usb);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error;
	}
	max17048_reset(client);
	max17048_get_version(client);

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);
	schedule_delayed_work(&chip->work, MAX17048_DELAY);

	return 0;
error:
	power_supply_unregister(&chip->ac);
error1:
	power_supply_unregister(&chip->battery);
error2:
	kfree(chip);
	return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	power_supply_unregister(&chip->usb);
	power_supply_unregister(&chip->ac);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, MAX17048_DELAY);
	return 0;
}

#else

#define max17048_suspend NULL
#define max17048_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
	},
	.probe		= max17048_probe,
	.remove		= __devexit_p(max17048_remove),
	.suspend	= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
module_init(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
