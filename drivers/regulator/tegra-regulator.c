/*
 * drivers/regulator/tegra-regulator.c
 *
 * Regulator driver for NVIDIA Tegra NvRm PMU APIs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <mach/nvrm_linux.h>
#include <mach/regulator.h>

#include <nvassert.h>
#include <nvrm_pmu.h>
#include <nvrm_power.h>
#include <nvodm_query_discovery.h>

struct tegra_vdd {
	NvU32 id;
	unsigned long request_uV;
};

struct tegra_charger {
	struct regulator_desc rdesc;
	struct regulator_init_data init;
	struct regulator_dev *rdev;
	struct device *dev;
	struct work_struct work;
	struct list_head node;
	NvRmPmuChargingPath path;
	unsigned long request_uA;
};

struct tegra_regulator {
	struct regulator_desc rdesc;
	struct regulator_init_data init;
	struct regulator_dev *rdev;
	struct device *dev;
	struct list_head node;
	bool enable;
	int nr_vdd;
	struct tegra_vdd vdd_list[1];
};

struct tegra_regulator_drv {
	struct list_head list_reg;
	struct list_head list_chg;
};

static void tegra_charger_work(struct work_struct *w)
{
	struct tegra_charger *c= container_of(w, struct tegra_charger, work);

	NvRmPmuSetChargingCurrentLimit(s_hRmGlobal, c->path,
		       c->request_uA/1000, NvOdmUsbChargerType_SE0);
}

static int tegra_charger_set_current_limit(struct regulator_dev *rdev,
					    int min_uA, int max_uA)

{
	struct tegra_charger *c = rdev_get_drvdata(rdev);

	c->request_uA = max_uA;
	schedule_work(&c->work);

	return 0;
}

static int tegra_regulator_enable(struct regulator_dev *rdev)
{
	struct tegra_regulator *data = rdev_get_drvdata(rdev);
	NvU32 settle;
	int i;

	dev_dbg(data->dev, "%s: %s->%luuV enabled:%s\n", __func__,
		data->rdesc.name, data->vdd_list[0].request_uV,
		data->enable ? "true" : "false");

	if (!data->enable) {
		for (i=0; i<data->nr_vdd; i++) {
			const struct tegra_vdd *vdd = &data->vdd_list[i];
			NvRmPmuSetVoltage(s_hRmGlobal, vdd->id,
					  vdd->request_uV / 1000, &settle);
			udelay(settle);
		}
	}

	data->enable = true;
	return 0;
}

static int tegra_regulator_disable(struct regulator_dev *rdev)
{
	struct tegra_regulator *data = rdev_get_drvdata(rdev);
	int i;

	dev_dbg(data->dev, "%s: %s enabled:%s\n", __func__,
		data->rdesc.name, data->enable ? "true" : "false");

	if (data->enable) {
		for (i=0; i<data->nr_vdd; i++) {
			NvRmPmuSetVoltage(s_hRmGlobal, data->vdd_list[i].id,
					  NVODM_VOLTAGE_OFF, NULL);
		}
	}
	data->enable = false;
	return 0;
}

static int tegra_regulator_set_voltage(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	struct tegra_regulator *data = rdev_get_drvdata(rdev);

	BUG_ON(data->nr_vdd > 1);

	dev_dbg(data->dev, "%s: %s->[%d..%d]uV enabled:%s\n", __func__,
		data->rdesc.name, min_uV, max_uV,
		data->enable ? "true" : "false");

	if (data->enable) {
		NvU32 settle;
		NvRmPmuSetVoltage(s_hRmGlobal, data->vdd_list[0].id,
				  min_uV, &settle);
		udelay(settle);
	}
	data->vdd_list[0].request_uV = min_uV;
	return 0;
}

static struct regulator_ops tegra_soc_dynamic_vdd_ops = {
	.enable = tegra_regulator_enable,
	.disable = tegra_regulator_disable,
	.set_voltage = tegra_regulator_set_voltage,
};

static struct regulator_ops tegra_soc_fixed_vdd_ops = {
	.enable = tegra_regulator_enable,
	.disable = tegra_regulator_disable,
};

static struct regulator_ops tegra_charger_ops = {
	.set_current_limit = tegra_charger_set_current_limit,
};

static int tegra_charger_register(struct platform_device *pdev,
				  struct tegra_regulator_entry *entry)
{
	struct tegra_charger *chg;
	struct regulator_dev *rdev;
	struct tegra_regulator_drv *drv;

	drv = platform_get_drvdata(pdev);

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	chg->path = entry->charging_path;

	chg->rdesc.type = REGULATOR_CURRENT;
	chg->rdesc.name = kstrdup(entry->name, GFP_KERNEL);
	chg->rdesc.owner = THIS_MODULE;
	chg->dev = &pdev->dev;
	chg->init.consumer_supplies = entry->consumers;
	chg->init.num_consumer_supplies = entry->nr_consumers;
	chg->rdesc.id = entry->id;
	chg->init.constraints.valid_ops_mask = REGULATOR_CHANGE_CURRENT;
	chg->rdesc.ops = &tegra_charger_ops;
	chg->init.constraints.min_uA = 0;
	chg->init.constraints.max_uA = 1800000;
	INIT_WORK(&chg->work, tegra_charger_work);

	rdev = regulator_register(&chg->rdesc, &pdev->dev, &chg->init, chg);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "unable to register %s\n", entry->name);
		kfree(chg);
		return PTR_ERR(chg);
	}
	chg->rdev = rdev;

	list_add_tail(&chg->node, &drv->list_chg);

	return 0;
}

static int tegra_regulator_register(struct platform_device *pdev,
				    struct tegra_regulator_entry *entry)
{
	const NvOdmPeripheralConnectivity *con;
	struct tegra_regulator *reg;
	struct tegra_regulator_drv *drv;
	struct regulator_dev *rdev;
	NvRmPmuVddRailCapabilities cap;
	unsigned int i;
	unsigned int cnt;

	drv = platform_get_drvdata(pdev);

	con = NvOdmPeripheralGetGuid(entry->guid);
	if (!con)
		return -ENODEV;

	for (i=0, cnt=0; i<con->NumAddress; i++) {
		if (con->AddressList[i].Interface == NvOdmIoModule_Vdd)
			cnt++;
	}

	if (!cnt) {
		char guid_str[9];
		memcpy(guid_str, &entry->guid, 8);
		guid_str[8] = 0;
		dev_err(&pdev->dev, "(%s): no VDDs defined for %s\n",
			entry->name, guid_str);
		return -ENOSYS;
	}

	reg = kzalloc(sizeof(*reg) + sizeof(struct tegra_vdd)*(cnt-1), GFP_KERNEL);
	if (!reg) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}
	reg->nr_vdd = cnt;
	for (i=0, cnt=0; i<con->NumAddress; i++) {
		NvU32 vdd;
		if (con->AddressList[i].Interface != NvOdmIoModule_Vdd)
			continue;

		vdd = con->AddressList[i].Address;
		NvRmPmuGetCapabilities(s_hRmGlobal, vdd, &cap);
		reg->vdd_list[cnt].id = vdd;
		reg->vdd_list[cnt].request_uV = cap.requestMilliVolts * 1000;
		cnt++;
	}

	reg->rdesc.type = REGULATOR_VOLTAGE;
	reg->rdesc.name = kstrdup(entry->name, GFP_KERNEL);
	reg->rdesc.owner = THIS_MODULE;
	reg->enable = false;
	reg->dev = &pdev->dev;
	reg->init.consumer_supplies = entry->consumers;
	reg->init.num_consumer_supplies = entry->nr_consumers;
	reg->rdesc.id = entry->id;

	if (entry->guid == NV_VDD_SoC_ODM_ID) {
		reg->init.constraints.boot_on = true;
		reg->enable = true;
	}

	if (cnt>1 || cap.RmProtected) {
		reg->rdesc.ops = &tegra_soc_fixed_vdd_ops;
		reg->init.constraints.min_uV = reg->vdd_list[0].request_uV;
		reg->init.constraints.max_uV = reg->vdd_list[0].request_uV;
		reg->init.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
	} else {
		reg->rdesc.ops = &tegra_soc_dynamic_vdd_ops;
		reg->init.constraints.min_uV = cap.MinMilliVolts * 1000;
		reg->init.constraints.max_uV = cap.MaxMilliVolts * 1000;
		reg->init.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE;
	}

	rdev = regulator_register(&reg->rdesc, &pdev->dev, &reg->init, reg);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "unable to register %s\n", entry->name);
		kfree(reg);
		return PTR_ERR(reg);
	}
	reg->rdev = rdev;

	list_add_tail(&reg->node, &drv->list_reg);

	return 0;
}

static int tegra_regulator_probe(struct platform_device *pdev)
{
	struct tegra_regulator_platform_data *plat = pdev->dev.platform_data;
	struct tegra_regulator_drv *drv;
	int i;

	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	INIT_LIST_HEAD(&drv->list_reg);
	INIT_LIST_HEAD(&drv->list_chg);
	platform_set_drvdata(pdev, drv);

	for (i=0; i<plat->nr_regs; i++) {
		if (plat->regs[i].is_charger)
			tegra_charger_register(pdev, &plat->regs[i]);
		else
			tegra_regulator_register(pdev, &plat->regs[i]);
	}

	return 0;
}

static int tegra_regulator_remove(struct platform_device *pdev)
{
	struct tegra_regulator_drv *drv = platform_get_drvdata(pdev);

	while (!list_empty(&drv->list_reg)) {
		struct tegra_regulator *reg;
		reg = list_first_entry(&drv->list_reg, struct tegra_regulator, node);
		list_del_init(&drv->list_reg);
		regulator_unregister(reg->rdev);
		kfree(reg);
	}
	while (!list_empty(&drv->list_chg)) {
		struct tegra_charger *reg;
		reg = list_first_entry(&drv->list_chg, struct tegra_charger, node);
		list_del_init(&drv->list_chg);
		regulator_unregister(reg->rdev);
		cancel_work_sync(&reg->work);
		kfree(reg);
	}

	kfree(drv);

	return 0;
}

static struct platform_driver tegra_regulator_driver = {
	.driver = {
		.name = "tegra_regulator",
	},
	.probe = tegra_regulator_probe,
	.remove = tegra_regulator_remove,
};

static int __init tegra_regulator_init(void)
{
	return platform_driver_register(&tegra_regulator_driver);
}
postcore_initcall_sync(tegra_regulator_init);

static void __exit tegra_regulator_exit(void)
{
	platform_driver_unregister(&tegra_regulator_driver);
}
module_exit(tegra_regulator_exit);

MODULE_DESCRIPTION("Tegra regulator driver");
MODULE_LICENSE("GPL");
