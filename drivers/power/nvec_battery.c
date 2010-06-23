/*
 * drivers/power/nvec_battery.c
 *
 * Battery driver for batteries connected to NVIDIA NvEc-compliant embedded
 * controller
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
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/reboot.h>

#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_battery.h"
#include "nvec_device.h"


/* This defines the manufacturer name and model name length */
#define BATTERY_INFO_NAME_LEN 30

/* This macro indicates the default battery status polling interval.
 * This value can be changed run-time by writing desired
 * value to /sys/devices/nvec/nvec_battery/status_poll_period
 */
#define NVBATTERY_POLLING_INTERVAL_MILLISECS 30000

typedef enum
{
	NvPowerSupply_TypeBattery = 0,
	NvPowerSupply_TypeAC,
	NvPowerSupply_TypeNum,
	NvPowerSupply_TypeForce32 = 0x7FFFFFFF
} NvPowerSupply_Type;

typedef enum
{
	NvCharge_Control_Charging_Disable = 0,
	NvCharge_Control_Charging_Enable,
	NvCharge_Control_Num,
	NvCharge_Control_Force32 = 0x7FFFFFFF
} NvCharge_Control;

static enum power_supply_property tegra_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_TEMP,
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

static struct power_supply tegra_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = tegra_battery_properties,
		.num_properties = ARRAY_SIZE(tegra_battery_properties),
		.get_property = tegra_battery_get_property,
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

/* This is the Battery context structure */
struct tegra_battery_dev {
	struct	timer_list	battery_poll_timer;
	NvOdmBatteryDeviceHandle hOdmBattDev;
	NvOsSemaphoreHandle		 hOdmSemaphore;
	NvOsThreadHandle		 hBattEventThread;
	NvU32	batt_id;
	NvU32	voltage;		/* voltage */
	NvU32	temp;			/* Temperature */
	NvU32	current_ma;		/* Battery current */
	NvU32	current_avg;		/* average current */
	NvU32	charging_source;	/* 0: no cable, 1:usb, 2:AC */
	NvU32	charging_enabled;	/* 0: Disable, 1: Enable */
	NvU32	capacity;		/* full capacity of battery (mAh) */
	NvU32	capacity_crit;		/* critical capacity level */
	NvU32	capacity_remain;	/* remaining battery capacity */
	NvU32	percent_remain;		/* percentage of battery remaining */
	NvU32	lifetime;
	NvU32	consumed;
	NvU32	batt_status_poll_period;
	NvBool	present;
	NvBool	exitThread;
};

static struct tegra_battery_dev *batt_dev;

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

void NvBatteryEventHandlerThread(void *args)
{
	NvU8	BatteryState = 0, BatteryEvent = 0;

	for (;;) {
		NvOsSemaphoreWait(batt_dev->hOdmSemaphore);

		if (batt_dev->exitThread)
			break;

		if (!batt_dev->hOdmBattDev)
			continue;

		NvOdmBatteryGetBatteryStatus(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main,
			&BatteryState);

		NvOdmBatteryGetEvent(batt_dev->hOdmBattDev, &BatteryEvent);

		if ((BatteryState == NVODM_BATTERY_STATUS_UNKNOWN) ||
			(BatteryEvent == NvOdmBatteryEventType_Num)) {
			/* Do nothing */
		} else {
			if (BatteryEvent & NvOdmBatteryEventType_RemainingCapacityAlarm) {
				if (BatteryState == (NVODM_BATTERY_STATUS_CRITICAL |
					NVODM_BATTERY_STATUS_VERY_CRITICAL |
					NVODM_BATTERY_STATUS_DISCHARGING)) {
					pr_info("nvec_battery:calling kernel_power_off...\n");
					kernel_power_off();
				}
			} else {
				/* Update the battery and power supply info for other events */
				power_supply_changed(&tegra_power_supplies[NvPowerSupply_TypeBattery]);
				power_supply_changed(&tegra_power_supplies[NvPowerSupply_TypeAC]);
			}
		}
	}
}

static void tegra_get_battery_tech(int *val, NvOdmBatteryInstance inst)
{
	NvOdmBatteryChemistry chemistry = NvOdmBatteryChemistry_Num;

	NvOdmBatteryGetBatteryChemistry(batt_dev->hOdmBattDev,
		inst, &chemistry);

	switch(chemistry) {
	case NvOdmBatteryChemistry_NICD:
		*val = POWER_SUPPLY_TECHNOLOGY_NiCd;
		break;

	case NvOdmBatteryChemistry_NIMH:
		*val = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;

	case NvOdmBatteryChemistry_LION:
		*val = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case NvOdmBatteryChemistry_LIPOLY:
		*val = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case NvOdmBatteryChemistry_XINCAIR:
	case NvOdmBatteryChemistry_Alkaline:
	default:
		*val = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	}
}

static void tegra_battery_convert(NvOdmBatteryData *data)
{
	if (data->BatteryLifePercent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLifePercent = 0;

	if (data->BatteryLifeTime == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLifeTime = 0;

	if (data->BatteryVoltage == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryVoltage = 0;

	if (data->BatteryCurrent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryCurrent = 0;

	if (data->BatteryMahConsumed == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryMahConsumed = 0;

	if (data->BatteryTemperature == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryTemperature = 0;

	if (data->BatteryAverageCurrent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryAverageCurrent = 0;

	if (data->BatteryLastChargeFullCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLastChargeFullCapacity = 0;

	if (data->BatteryCriticalCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryCriticalCapacity = 0;

	if (data->BatteryRemainingCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryRemainingCapacity = 0;
}

static NvBool tegra_battery_data(NvOdmBatteryInstance NvBatteryInst)
{
	NvOdmBatteryData data = {0};
	NvBool RetValue = NV_FALSE;

	if (NvOdmBatteryGetBatteryData(batt_dev->hOdmBattDev,
		NvBatteryInst, &data))
		RetValue = NV_TRUE;

	tegra_battery_convert(&data);

	if (NvBatteryInst == NvOdmBatteryInst_Main) {
		batt_dev->voltage = data.BatteryVoltage;
		batt_dev->current_ma = data.BatteryCurrent;
		batt_dev->current_avg  = data.BatteryAverageCurrent;
		batt_dev->temp = data.BatteryTemperature;
		batt_dev->percent_remain = data.BatteryLifePercent;
		batt_dev->lifetime = data.BatteryLifeTime;
		batt_dev->consumed = data.BatteryMahConsumed;
		batt_dev->capacity = data.BatteryLastChargeFullCapacity;
		batt_dev->capacity_crit = data.BatteryCriticalCapacity;
		batt_dev->capacity_remain = data.BatteryRemainingCapacity;
	}

	return RetValue;
}

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			NvOdmBatteryAcLineStatus status;
			if (!NvOdmBatteryGetAcLineStatus(batt_dev->hOdmBattDev,
				&status))
				return -ENODEV;

			val->intval = (status == NvOdmBatteryAcLine_Online); 
		}
		else {
			printk(KERN_INFO "Only AC mains is used as external power\n");
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	NvU8 name[BATTERY_INFO_NAME_LEN] = {0};
	int technology = 0;
	NvU8 state = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		if (!NvOdmBatteryGetBatteryStatus(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, &state))
			return -ENODEV;

		if (state == NVODM_BATTERY_STATUS_UNKNOWN) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else if (state == NVODM_BATTERY_STATUS_NO_BATTERY) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else if (state & NVODM_BATTERY_STATUS_CHARGING) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if (state & NVODM_BATTERY_STATUS_DISCHARGING) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else if (state & NVODM_BATTERY_STATUS_IDLE) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (!batt_dev->present ) {
			batt_dev->voltage = 0;
			batt_dev->current_ma = 0;
			batt_dev->current_avg  = 0;
			batt_dev->temp = 0;
			batt_dev->percent_remain = 0;
			batt_dev->lifetime = 0;
			batt_dev->consumed = 0;
			batt_dev->capacity = 0;
			batt_dev->capacity_crit = 0;
			batt_dev->capacity_remain = 0;
		}
		else {
			/*
			 * Getting the battery info once here so for the other property
			 * requests there will not be lot of ec req
			 */
			if (tegra_battery_data(NvOdmBatteryInst_Main)) {
				if (batt_dev->percent_remain == 100) {
					val->intval = POWER_SUPPLY_STATUS_FULL;
				}
			}
		}
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (batt_dev->present)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		if (!NvOdmBatteryGetBatteryStatus(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, &state))
			return -EINVAL;

		if (state == NVODM_BATTERY_STATUS_UNKNOWN) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else {
			if (state == NVODM_BATTERY_STATUS_NO_BATTERY) {
				 batt_dev->present = NV_FALSE;
				 val->intval = NV_FALSE;
			}

			if (state & (NVODM_BATTERY_STATUS_HIGH |
					NVODM_BATTERY_STATUS_LOW |
					NVODM_BATTERY_STATUS_CRITICAL |
					NVODM_BATTERY_STATUS_CHARGING |
					NVODM_BATTERY_STATUS_DISCHARGING |
					NVODM_BATTERY_STATUS_IDLE)) {
				batt_dev->present = NV_TRUE;
				val->intval = NV_TRUE;
			}
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		tegra_get_battery_tech(&technology, NvOdmBatteryInst_Main);
		val->intval = technology;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = batt_dev->percent_remain;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_dev->voltage*1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt_dev->current_ma;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = batt_dev->current_avg;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = batt_dev->capacity_remain;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = batt_dev->capacity;
		break;

	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		val->intval = batt_dev->capacity_crit;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* returned value is degrees C * 10 */
		val->intval = batt_dev->temp/10;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (!NvOdmBatteryGetModel(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, name))
			return -EINVAL;

		strncpy((char *)val->strval, name, strlen(name));
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (!NvOdmBatteryGetManufacturer(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, name))
			return -EINVAL;

		strncpy((char *)val->strval, name, strlen(name));
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void tegra_battery_poll_timer_func(unsigned long unused)
{
	power_supply_changed(&tegra_power_supplies[NvPowerSupply_TypeBattery]);
	power_supply_changed(&tegra_power_supplies[NvPowerSupply_TypeAC]);

	mod_timer(&(batt_dev->battery_poll_timer),
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));
}

static int nvec_battery_probe(struct nvec_device *pdev)
{
	int i, rc;
	NvError ErrorStatus = NvSuccess;
	NvBool result = NV_FALSE;

	batt_dev = kzalloc(sizeof(struct tegra_battery_dev), GFP_KERNEL);
	if (!batt_dev) {
		pr_err("nvec_battery_probe:NOMEM\n");
		return -ENOMEM;
	}

	ErrorStatus = NvOsSemaphoreCreate(&batt_dev->hOdmSemaphore, 0);
	if (NvSuccess != ErrorStatus) {
		pr_err("NvOsSemaphoreCreate Failed!\n");
		goto Cleanup;
	}

	batt_dev->exitThread = NV_FALSE;
	ErrorStatus = NvOsThreadCreate(NvBatteryEventHandlerThread,
					batt_dev,
					&(batt_dev->hBattEventThread));
	if (NvSuccess != ErrorStatus) {
		pr_err("NvOsThreadCreate FAILED\n");
		goto Cleanup;
	}

	result = NvOdmBatteryDeviceOpen(&(batt_dev->hOdmBattDev),
		(NvOdmOsSemaphoreHandle *)&batt_dev->hOdmSemaphore);
	if (!result || !batt_dev->hOdmBattDev) {
		pr_err("NvOdmBatteryDeviceOpen FAILED\n");
		goto Cleanup;
	}

	for (i = 0; i < ARRAY_SIZE(tegra_power_supplies); i++) {
		if (power_supply_register(&pdev->dev, &tegra_power_supplies[i]))
			pr_err("Failed to register power supply\n");
	}

	batt_dev->batt_status_poll_period = NVBATTERY_POLLING_INTERVAL_MILLISECS;
	setup_timer(&(batt_dev->battery_poll_timer), tegra_battery_poll_timer_func, 0);
	mod_timer(&(batt_dev->battery_poll_timer),
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));

	rc = device_create_file(&pdev->dev, &tegra_battery_attr);
	if (rc) {
		for (i = 0; i < ARRAY_SIZE(tegra_power_supplies); i++) {
			power_supply_unregister(&tegra_power_supplies[i]);
		}

		del_timer_sync(&(batt_dev->battery_poll_timer));

		pr_err("nvec_battery_probe:device_create_file FAILED");
		goto Cleanup;
	}

	return 0;

Cleanup:
	batt_dev->exitThread = NV_TRUE;
	if (batt_dev->hOdmSemaphore) {
		NvOsSemaphoreSignal(batt_dev->hOdmSemaphore);
		NvOsSemaphoreDestroy(batt_dev->hOdmSemaphore);
		batt_dev->hOdmSemaphore = NULL;
	}

	if (batt_dev->hBattEventThread) {
		NvOsThreadJoin(batt_dev->hBattEventThread);
	}

	if (batt_dev->hOdmBattDev) {
		NvOdmBatteryDeviceClose(batt_dev->hOdmBattDev);
		batt_dev->hOdmBattDev = NULL;
	}

	kfree(batt_dev);
	batt_dev = NULL;
	return -1;
}

static void nvec_battery_remove(struct nvec_device *pdev)
{
	unsigned int i = 0;

	if (batt_dev) {
		batt_dev->exitThread = NV_TRUE;
		if (batt_dev->hOdmSemaphore) {
			NvOsSemaphoreSignal(batt_dev->hOdmSemaphore);
			NvOsSemaphoreDestroy(batt_dev->hOdmSemaphore);
			batt_dev->hOdmSemaphore = NULL;
		}

		if (batt_dev->hBattEventThread) {
			NvOsThreadJoin(batt_dev->hBattEventThread);
		}

		if (batt_dev->hOdmBattDev) {
			device_remove_file(&pdev->dev, &tegra_battery_attr);

			del_timer_sync(&(batt_dev->battery_poll_timer));

			NvOdmBatteryDeviceClose(batt_dev->hOdmBattDev);
			batt_dev->hOdmBattDev = NULL;

			for (i = 0; i < ARRAY_SIZE(tegra_power_supplies); i++) {
				power_supply_unregister(&tegra_power_supplies[i]);
			}
		}

		kfree(batt_dev);
		batt_dev = NULL;

	}
}

static int nvec_battery_suspend(struct nvec_device *dev,
	pm_message_t state)
{
	/* Kill the Battery Polling timer */
	del_timer_sync(&batt_dev->battery_poll_timer);
	return 0;
}

static int nvec_battery_resume(struct nvec_device *dev)
{
	/*Create Battery Polling timer */
	setup_timer(&batt_dev->battery_poll_timer, tegra_battery_poll_timer_func, 0);
	mod_timer(&batt_dev->battery_poll_timer,
		jiffies + msecs_to_jiffies(batt_dev->batt_status_poll_period));
	return 0;
}

static struct nvec_driver nvec_battery_driver = {
	.name		= "nvec_battery",
	.probe		= nvec_battery_probe,
	.remove		= nvec_battery_remove,
	.suspend	= nvec_battery_suspend,
	.resume		= nvec_battery_resume,
};

static struct nvec_device nvec_battery_device = {
	.name	= "nvec_battery",
	.driver	= &nvec_battery_driver,
};

static int __init nvec_battery_init(void)
{
	int err;

	err = nvec_register_driver(&nvec_battery_driver);
	if (err)
	{
		pr_err("**nvec_battery_init: nvec_register_driver: fail\n");
		return err;
	}

	err = nvec_register_device(&nvec_battery_device);
	if (err)
	{
		pr_err("**nvec_battery_init: nvec_device_add: fail\n");
		nvec_unregister_driver(&nvec_battery_driver);
		return err;
	}

	return 0;
}

static void __exit nvec_battery_exit(void)
{
	nvec_unregister_device(&nvec_battery_device);
	nvec_unregister_driver(&nvec_battery_driver);
}

module_init(nvec_battery_init);
module_exit(nvec_battery_exit);
MODULE_DESCRIPTION("TEGRA EC based Battery Driver");
