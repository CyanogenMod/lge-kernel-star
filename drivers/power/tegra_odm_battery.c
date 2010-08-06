/*
 * drivers/power/tegra_odm_battery.c
 *
 * Battery driver for batteries implemented using NVIDIA Tegra ODM kit PMU
 * adaptation interface
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

#include "nvcommon.h"
#include "nvos.h"
#include "nvrm_pmu.h"
#include "nvodm_battery.h"
#include "mach/nvrm_linux.h" // for s_hRmGlobal

#define NVBATTERY_POLLING_INTERVAL 30000 /* 30 Seconds */

typedef enum {
	NvCharger_Type_Battery = 0,
	NvCharger_Type_USB,
	NvCharger_Type_AC,
	NvCharger_Type_Num,
	NvCharger_Type_Force32 = 0x7FFFFFFF
} NvCharger_Type;

typedef enum {
	NvCharge_Control_Charging_Disable = 0,
	NvCharge_Control_Charging_Enable,
	NvCharge_Control_Num,
	NvCharge_Control_Force32 = 0x7FFFFFFF
} NvCharge_Control;

static enum power_supply_property tegra_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP
};

static enum power_supply_property tegra_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static struct power_supply tegra_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = tegra_battery_properties,
		.num_properties = ARRAY_SIZE(tegra_battery_properties),
		.get_property = tegra_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
};

typedef struct tegra_battery_dev {
	struct	timer_list	battery_poll_timer;
	NvU32	batt_id;		/* battery ID from ADC */
	NvU32	batt_vol;		/* voltage from ADC */
	NvU32	batt_temp;		/* temperature (degrees C) */
	NvU32	batt_current;		/* current from ADC */
	NvU32	charging_source;	/* 0: no cable, 1:usb, 2:AC */
	NvU32	charging_enabled;	/* 0: Disable, 1: Enable */
	NvU32	full_bat;		/* max capacity of battery (mAh) */
	NvU32	BatteryLifePercent;
	NvU32	BatteryLifeTime;
	NvU32	BatteryMahConsumed;
	NvU32	ACLineStatus;
	NvU32	batt_status_poll_period;
	NvBool	present;
	NvOdmBatteryDeviceHandle hOdmBattDev;
} tegra_battery_dev;

static tegra_battery_dev *batt_dev;

static ssize_t tegra_battery_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", batt_dev->batt_status_poll_period);
}

static ssize_t tegra_battery_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	unsigned int value = 0;
	value = simple_strtoul(buf, NULL, 0);
	batt_dev->batt_status_poll_period = value;
	mod_timer(&(batt_dev->battery_poll_timer),
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));
	return count;
}

static struct device_attribute tegra_battery_attr = {
	.attr = { .name = "status_poll_period", .mode = S_IRUGO | S_IWUSR,
			  .owner = THIS_MODULE },
	.show = tegra_battery_show_property,
	.store = tegra_battery_store_property,
};

static void tegra_get_battery_tech(int *Value,
	NvRmPmuBatteryInstance NvBatteryInst)
{
	NvRmPmuBatteryChemistry Chemistry = {0};

	NvRmPmuGetBatteryChemistry(s_hRmGlobal, NvBatteryInst, &Chemistry);

	switch(Chemistry)
	{
		case NvRmPmuBatteryChemistry_NICD:
			 *Value = POWER_SUPPLY_TECHNOLOGY_NiCd;
			 break;

		case NvRmPmuBatteryChemistry_NIMH:
			 *Value = POWER_SUPPLY_TECHNOLOGY_NiMH;
			 break;

		case NvRmPmuBatteryChemistry_LION:
			 *Value = POWER_SUPPLY_TECHNOLOGY_LION;
			 break;

		case NvRmPmuBatteryChemistry_LIPOLY:
			 *Value = POWER_SUPPLY_TECHNOLOGY_LIPO;
			 break;

		default:
			 *Value = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
			 break;
	}
}

static void tegra_battery_convert(NvRmPmuBatteryData *pPmuData)
{
	if (pPmuData->batteryLifePercent == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryLifePercent = 0;

	if (pPmuData->batteryLifeTime == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryLifeTime = 0;

	if (pPmuData->batteryVoltage == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryVoltage = 0;

	if (pPmuData->batteryCurrent == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryCurrent = 0;

	if (pPmuData->batteryMahConsumed == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryMahConsumed = 0;

	if (pPmuData->batteryTemperature == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryTemperature = 0;
}

static int tegra_battery_data(NvRmPmuBatteryInstance NvBatteryInst)
{
	NvRmPmuBatteryData pPmuData = {0};

	if (!NvRmPmuGetBatteryData(s_hRmGlobal, NvBatteryInst, &pPmuData))
		return -1;

	if (NvBatteryInst == NvRmPmuBatteryInst_Main) {
		tegra_battery_convert(&pPmuData);
		batt_dev->batt_vol = pPmuData.batteryVoltage;
		batt_dev->batt_vol *= 1000; // Convert volt to mV
		batt_dev->batt_current = pPmuData.batteryCurrent;
		batt_dev->batt_temp = pPmuData.batteryTemperature;
		batt_dev->batt_temp *= 10;  // FIXME : Why is this here?
		batt_dev->BatteryLifePercent = pPmuData.batteryLifePercent;
		batt_dev->BatteryLifeTime = pPmuData.batteryLifeTime;
		batt_dev->BatteryMahConsumed = pPmuData.batteryMahConsumed;
	}

	return 0;
}

static int tegra_get_ac_status(NvCharger_Type *charger)
{
	NvRmPmuAcLineStatus ACStatus = NvRmPmuAcLine_Offline;

	if (!NvRmPmuGetAcLineStatus(s_hRmGlobal, &ACStatus))
		return -1;

	if (ACStatus == NvRmPmuAcLine_Offline)
		batt_dev->ACLineStatus = NV_FALSE;
	else if (ACStatus == NvRmPmuAcLine_Online) {
		batt_dev->ACLineStatus = NV_TRUE;
		batt_dev->charging_source = NvCharger_Type_AC;
		if (charger)
			*charger = batt_dev->charging_source;
        } else
		batt_dev->ACLineStatus = NV_FALSE;

	return 0;
}

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	NvCharger_Type charger;

	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (tegra_get_ac_status(&charger))
			return -EINVAL;

		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  NvCharger_Type_AC);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  NvCharger_Type_USB);
		else
			val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	NvU8 state[4] = { 0 };
	int batt_tech = 0;

	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
		if (!NvRmPmuGetBatteryStatus(s_hRmGlobal,
			NvRmPmuBatteryInst_Main, state))
			return -EINVAL;

		switch(state[0]) {

		case NVODM_BATTERY_STATUS_HIGH:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			batt_dev->present = NV_TRUE;
			break;

		case NVODM_BATTERY_STATUS_NO_BATTERY:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			batt_dev->present = NV_FALSE;
			break;

		case NVODM_BATTERY_STATUS_CHARGING:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			batt_dev->present = NV_TRUE;
			batt_dev->charging_enabled = NV_TRUE;
			batt_dev->charging_source = NvCharger_Type_AC;
			break;

		case NVODM_BATTERY_STATUS_UNKNOWN:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			batt_dev->present = NV_FALSE;
			break;

		case NVODM_BATTERY_STATUS_LOW:
		case NVODM_BATTERY_STATUS_CRITICAL:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			batt_dev->present = NV_TRUE;
			break;

		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			batt_dev->present = NV_FALSE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = batt_dev->present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if (batt_dev->present)
			tegra_get_battery_tech(&batt_tech,
				NvRmPmuBatteryInst_Main);
		else
			batt_tech = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

		val->intval = batt_tech;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (batt_dev->present)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TEMP:
		if (!batt_dev->present) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
		if (tegra_battery_data(NvRmPmuBatteryInst_Main))
			return -EINVAL;

		if (psp == POWER_SUPPLY_PROP_CAPACITY)
			val->intval = batt_dev->BatteryLifePercent;
		else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW)
			val->intval = batt_dev->batt_vol;
		if (psp == POWER_SUPPLY_PROP_TEMP)
			val->intval = batt_dev->batt_temp;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static void tegra_battery_poll_timer_func(unsigned long unused)
{
	power_supply_changed(&tegra_supplies[NvCharger_Type_Battery]);
	power_supply_changed(&tegra_supplies[NvCharger_Type_USB]);
	power_supply_changed(&tegra_supplies[NvCharger_Type_AC]);

	mod_timer(&(batt_dev->battery_poll_timer),
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));
}

static int tegra_battery_probe(struct platform_device *pdev)
{
	int i, rc;
	NvBool result;

	batt_dev = kzalloc(sizeof(*batt_dev), GFP_KERNEL);
	if (!batt_dev) {
		return -ENOMEM;
	}
	memset(batt_dev, 0, sizeof(*batt_dev));

	/* Assume battery is present at start */
	batt_dev->present = 1;
	batt_dev->batt_id = 0;
	batt_dev->charging_source = NvCharger_Type_AC;
	batt_dev->charging_enabled = NvCharge_Control_Charging_Enable;

	result = NvOdmBatteryDeviceOpen(&(batt_dev->hOdmBattDev), NULL);
	if (!result) {
		pr_err("NvOdmBatteryDeviceOpen FAILED\n");
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &tegra_supplies[i]);
		if (rc) {
			printk(KERN_ERR "Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&tegra_supplies[i]);
			kfree(batt_dev);
			return rc;
		}
	}
	printk(KERN_INFO "%s: battery driver registered\n", pdev->name);

	batt_dev->batt_status_poll_period = NVBATTERY_POLLING_INTERVAL;
	setup_timer(&(batt_dev->battery_poll_timer), tegra_battery_poll_timer_func, 0);
	mod_timer(&(batt_dev->battery_poll_timer),
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));

	rc = device_create_file(&pdev->dev, &tegra_battery_attr);
	if (rc) {
		for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
			power_supply_unregister(&tegra_supplies[i]);
		}

		del_timer_sync(&(batt_dev->battery_poll_timer));

		pr_err("tegra_battery_probe:device_create_file FAILED");
		return rc;
	}

	return 0;
err:
	if (batt_dev) {
		kfree(batt_dev);
		batt_dev = NULL;
	}
	return -1;
}

static int tegra_battery_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
		power_supply_unregister(&tegra_supplies[i]);
	}

	if (batt_dev) {
		device_remove_file(&pdev->dev, &tegra_battery_attr);

		del_timer_sync(&(batt_dev->battery_poll_timer));
                if (batt_dev->hOdmBattDev) {
                        NvOdmBatteryDeviceClose(batt_dev->hOdmBattDev);
                        batt_dev->hOdmBattDev = NULL;
                }
		kfree(batt_dev);
		batt_dev = NULL;
	}
	return 0;
}

static int tegra_battery_suspend(struct platform_device *dev,
	pm_message_t state)
{
	/* Kill the Battery Polling timer */
	del_timer_sync(&batt_dev->battery_poll_timer);
	return 0;
}

static int tegra_battery_resume(struct platform_device *dev)
{
	/*Create Battery Polling timer */
	setup_timer(&batt_dev->battery_poll_timer, tegra_battery_poll_timer_func, 0);
	mod_timer(&batt_dev->battery_poll_timer,
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));
	return 0;
}

static struct platform_driver tegra_battery_driver =
{
	.probe	= tegra_battery_probe,
	.remove	= tegra_battery_remove,
	.suspend= tegra_battery_suspend,
	.resume	= tegra_battery_resume,
	.driver	= {
		.name	= "tegra_battery",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_battery_init(void)
{
	platform_driver_register(&tegra_battery_driver);
	return 0;
}

static void __exit tegra_battery_exit(void)
{
	platform_driver_unregister(&tegra_battery_driver);
}

module_init(tegra_battery_init);
module_exit(tegra_battery_exit);
MODULE_DESCRIPTION("TEGRA Battery Driver");
