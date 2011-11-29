/*
 * driver/regultor/tps6236x.c
 *
 * Driver for processor core supply tps62360 and tps62361B
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

/*#define DEBUG		1*/
/*#define VERBOSE_DEBUG	1*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6236x-regulator.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

/* Register definitions */
#define REG_VSET0		0
#define REG_VSET1		1
#define REG_VSET2		2
#define REG_VSET3		3
#define REG_CONTROL		4
#define REG_TEMP		5
#define REG_RAMPCTRL		6
#define REG_CHIPID		8

enum chips {TPS62360, TPS62361B};

/* Supported voltage values for regulators */
static const u16 TPS62360_VOLTAGES[] = {
	 770,  780,  790,  800,  810,  820,  830,  840,  850,  860,
	 870,  880,  890,  900,  910,  920,  930,  940,  950,  960,
	 970,  980,  990, 1000, 1010, 1020, 1030, 1040, 1050, 1060,
	1070, 1080, 1090, 1110, 1110, 1120, 1130, 1140, 1150, 1160,
	1170, 1180, 1190, 1200, 1210, 1220, 1230, 1240, 1250, 1260,
	1270, 1280, 1290, 1300, 1310, 1320, 1330, 1340, 1350, 1360,
	1370, 1380, 1390, 1400,
};

static const u16 TPS62361_VOLTAGES[] = {
	 500,  510,  520,  530,  540,  550,  560,  570,  580,  590,
	 600,  610,  620,  630,  640,  650,  660,  670,  680,  690,
	 700,  710,  720,  730,  740,  750,  760,  770,  780,  790,
	 800,  810,  820,  830,  840,  850,  860,  870,  880,  890,
	 900,  910,  920,  930,  940,  950,  960,  970,  980,  990,
	1000, 1010, 1020, 1030, 1040, 1050, 1060, 1070, 1080, 1090,
	1110, 1110, 1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190,
	1200, 1210, 1220, 1230, 1240, 1250, 1260, 1270, 1280, 1290,
	1300, 1310, 1320, 1330, 1340, 1350, 1360, 1370, 1380, 1390,
	1400, 1410, 1420, 1430, 1440, 1450, 1460, 1470, 1480, 1490,
	1500, 1510, 1520, 1530, 1540, 1550, 1560, 1570, 1580, 1590,
	1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670, 1680, 1690,
	1700, 1710, 1720, 1730, 1740, 1750, 1760, 1770,
};

/* tps 6236x chip information */
struct tps6236x_chip {
	const char *name;
	struct device *dev;
	struct regulator_desc desc;
	struct i2c_client *client;
	struct regulator_dev *rdev;
	struct mutex io_lock;
	int chip_id;
	unsigned int curr_uV;
	int vsel_id;
	int internal_pulldn_en;
	const u16 *voltages;
	u8 voltage_reg_mask;
	bool is_force_pwm;
	bool enable_discharge;
};
static inline int tps6236x_read(struct tps6236x_chip *tps, u8 reg)
{
	return i2c_smbus_read_byte_data(tps->client, reg);
}

static inline int tps6236x_write(struct tps6236x_chip *tps, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(tps->client, reg, val);
}

static int tps6236x_reg_read(struct tps6236x_chip *tps, u8 reg)
{
	int data;

	mutex_lock(&tps->io_lock);

	data = tps6236x_read(tps, reg);
	if (data < 0)
		dev_err(tps->dev, "Read from reg 0x%x failed\n", reg);

	mutex_unlock(&tps->io_lock);

	return data;
}

static int tps6236x_reg_write(struct tps6236x_chip *tps, u8 reg, u8 val)
{
	int err;

	mutex_lock(&tps->io_lock);

	err = tps6236x_write(tps, reg, val);
	if (err < 0)
		dev_err(tps->dev, "Write for reg 0x%x failed\n", reg);

	mutex_unlock(&tps->io_lock);

	return err;
}

static int tps6236x_reg_update(struct tps6236x_chip *tps, u8 reg, u8 val,
		u8 mask)
{
	int err;
	int data;
	u8 reg_val;

	mutex_lock(&tps->io_lock);
	data = tps6236x_read(tps, reg);
	if (data < 0) {
		dev_err(tps->dev, "Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}
	reg_val = (u8)data;
	reg_val = (reg_val & ~mask) | (val & mask);

	err = tps6236x_write(tps, reg, reg_val);
	if (err < 0)
		dev_err(tps->dev, "Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&tps->io_lock);
	return err;
}

static int __tps6236x_dcdc_set_voltage(struct tps6236x_chip *tps,
				       int min_uV, int max_uV,
				       unsigned *selector)
{
	int vsel;

	if (max_uV < min_uV)
		return -EINVAL;

	if (min_uV > tps->voltages[tps->desc.n_voltages - 1] * 1000)
		return -EINVAL;

	if (max_uV < tps->voltages[0] * 1000)
		return -EINVAL;

	for (vsel = 0; vsel < tps->desc.n_voltages; ++vsel) {
		int mV = tps->voltages[vsel];
		int uV = mV * 1000;
		if (min_uV <= uV && uV <= max_uV) {
			if (selector)
				*selector = vsel;
			if (tps->is_force_pwm)
				vsel |= (1 << 7);
			return tps6236x_reg_write(tps, REG_VSET0 + tps->vsel_id,
					vsel);
		}
	}
	return -EINVAL;
}

static int tps6236x_dcdc_is_enabled(struct regulator_dev *dev)
{
	/* Always return 1 as the EN is not controlled by register
	   programming */
	return 1;
}

static int tps6236x_dcdc_enable(struct regulator_dev *dev)
{
	/* No way to enable dc-dc converter through register programming */
	return 0;

}

static int tps6236x_dcdc_disable(struct regulator_dev *dev)
{
	/* No way to disable dc-dc converter through register programming */
	return 0;
}

static int tps6236x_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct tps6236x_chip *tps = rdev_get_drvdata(dev);
	int data;

	data = tps6236x_reg_read(tps, REG_VSET0 + tps->vsel_id);
	if (data < 0)
		return data;
	data &= tps->voltage_reg_mask;

	return tps->voltages[data] * 1000;
}

static int tps6236x_dcdc_set_voltage(struct regulator_dev *dev,
				     int min_uV, int max_uV,
				     unsigned *selector)
{
	struct tps6236x_chip *tps = rdev_get_drvdata(dev);

	return __tps6236x_dcdc_set_voltage(tps, min_uV, max_uV, selector);
}

static int tps6236x_dcdc_list_voltage(struct regulator_dev *dev,
					unsigned selector)
{
	struct tps6236x_chip *tps = rdev_get_drvdata(dev);

	if ((selector < 0) || (selector >= tps->desc.n_voltages))
		return -EINVAL;

	return tps->voltages[selector] * 1000;
}

/* Operations permitted on VDCDCx */
static struct regulator_ops tps6236x_dcdc_ops = {
	.is_enabled = tps6236x_dcdc_is_enabled,
	.enable = tps6236x_dcdc_enable,
	.disable = tps6236x_dcdc_disable,
	.get_voltage = tps6236x_dcdc_get_voltage,
	.set_voltage = tps6236x_dcdc_set_voltage,
	.list_voltage = tps6236x_dcdc_list_voltage,
};

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_regs(const char *header, struct seq_file *s,
		struct tps6236x_chip *tps, int start_offset,
		int end_offset)
{
	int reg_val;
	int i;

	seq_printf(s, "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		reg_val = tps6236x_reg_read(tps, i);
		if (reg_val >= 0)
			seq_printf(s, "Reg 0x%02x Value 0x%02x\n", i, reg_val);
	}
	seq_printf(s, "------------------\n");
}

static int dbg_tps_show(struct seq_file *s, void *unused)
{
	struct tps6236x_chip *tps = s->private;

	seq_printf(s, "TPS6236x Registers\n");
	seq_printf(s, "------------------\n");

	print_regs("Voltage Regs",  s, tps, 0x0, 0x3);
	print_regs("Config Regs",   s, tps, 0x4, 0x6);
	print_regs("ManId Regs",    s, tps, 0x8, 0x9);
	return 0;
}

static int dbg_tps_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_tps_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_tps_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __init tps6236x_debuginit(struct tps6236x_chip *tps)
{
	(void)debugfs_create_file("tps6236x", S_IRUGO, NULL,
			tps, &debug_fops);
}
#else
static void __init tps6236x_debuginit(struct tps6236x_chip *tps)
{
	return;
}
#endif
static int tps6236x_init_dcdc(struct i2c_client *client,
		struct tps6236x_regulator_platform_data *pdata,
		struct tps6236x_chip *tps)
{
	int st;
	int init_mV;
	int data;

	if (pdata->internal_pd_enable)
		st = tps6236x_write(tps, REG_CONTROL, 0xE0);
	else
		st = tps6236x_write(tps, REG_CONTROL, 0x0);
	if (st < 0) {
		dev_err(tps->dev, "%s() fails in writing reg %d\n",
			__func__, REG_CONTROL);
		return st;
	}

	data = tps6236x_reg_read(tps, REG_VSET0 + tps->vsel_id);
	if (data < 0) {
		dev_err(tps->dev, "%s() fails in reading reg %d\n",
			__func__, REG_VSET0 + tps->vsel_id);
		return data;
	}
	if (pdata->is_force_pwm)
		data |= (1 << 7);
	else
		data &= ~(1 << 7);
	st = tps6236x_reg_write(tps, REG_VSET0 + tps->vsel_id, data);
	if (st < 0) {
		dev_err(tps->dev, "%s() fails in writing reg %d\n",
			__func__, REG_VSET0 + tps->vsel_id);
		return st;
	}

	/* Reset output discharge path */
	st = tps6236x_reg_update(tps, REG_RAMPCTRL, 0, 1 << 2);
	if (st < 0) {
		dev_err(tps->dev, "%s() fails in updating reg %d\n",
			__func__, REG_RAMPCTRL);
		return st;
	}

	if (!pdata->init_apply)
		return 0;

	init_mV = pdata->init_uV;
	return __tps6236x_dcdc_set_voltage(tps, init_mV, init_mV, 0);
}

static int __devinit tps6236x_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct tps6236x_regulator_platform_data *pdata;
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct tps6236x_chip *tps;
	int err;
	int chip_id;
	int part_id;

	dev_dbg(&client->dev, "%s() is called\n", __func__);
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "Err: The I2c functionality is"
					" not supported\n");
		return -EIO;
	}

	/**
	 * init_data points to array of regulator_init structures
	 * coming from the board-evm file.
	 */
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "Err: Platform data not found\n");
		return -EIO;
	}

	init_data = &pdata->reg_init_data;
	tps = kzalloc(sizeof(*tps), GFP_KERNEL);
	if (!tps) {
		dev_err(&client->dev, "Err: Memory allocation fails\n");
		return -ENOMEM;
	}

	mutex_init(&tps->io_lock);

	tps->is_force_pwm = pdata->is_force_pwm;
	tps->enable_discharge = pdata->enable_discharge;
	tps->chip_id = id->driver_data;
	tps->client = client;
	tps->dev = &client->dev;
	tps->internal_pulldn_en = pdata->internal_pd_enable;
	tps->vsel_id = pdata->vsel;
	tps->name = id->name;
	tps->voltages = (tps->chip_id == TPS62360) ?
				TPS62360_VOLTAGES : TPS62361_VOLTAGES;
	tps->voltage_reg_mask = (tps->chip_id == TPS62360) ? 0x3F : 0x7F;

	tps->desc.name = id->name;
	tps->desc.id = 0;
	tps->desc.n_voltages = (tps->chip_id == TPS62360) ?
				ARRAY_SIZE(TPS62360_VOLTAGES) :
				ARRAY_SIZE(TPS62361_VOLTAGES);
	tps->desc.ops = &tps6236x_dcdc_ops;
	tps->desc.type = REGULATOR_VOLTAGE;
	tps->desc.owner = THIS_MODULE;

	i2c_set_clientdata(client, tps);

	/* Read version number and compare with chipid */
	chip_id = tps6236x_read(tps, REG_CHIPID);
	if (chip_id < 0) {
		err = chip_id;
		dev_err(tps->dev, "Error in reading device %d\n", err);
		goto fail;
	}
	part_id = (chip_id >> 2) & 0x3;
	if (((part_id == 0) && (tps->chip_id != TPS62360)) ||
		((part_id == 1) && (tps->chip_id != TPS62361B)) ||
		(part_id == 2) || (part_id == 3)) {
		dev_err(tps->dev, "Err: Mismatch of partid and driver chip-id"
				  " 0x%x\n", chip_id);
		err = -ENODEV;
		goto fail;
	}

	err = tps6236x_init_dcdc(client, pdata, tps);
	if (err < 0) {
		dev_err(tps->dev, "TPS6236X init fails with = %d\n", err);
		goto fail;
	}

	/* Register the regulators */
	rdev = regulator_register(&tps->desc, &client->dev, init_data, tps);
	if (IS_ERR(rdev)) {
		dev_err(tps->dev, "Failed to register %s\n", id->name);
		err = PTR_ERR(rdev);
		goto fail;
	}

	tps->rdev = rdev;

	tps6236x_debuginit(tps);
	return 0;

fail:
	kfree(tps);
	return err;
}

/**
 * tps6236x_remove - tps6236x driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister TPS driver as an i2c client device driver
 */
static int __devexit tps6236x_remove(struct i2c_client *client)
{
	struct tps6236x_chip *tps = i2c_get_clientdata(client);

	regulator_unregister(tps->rdev);
	kfree(tps);
	return 0;
}

static void tps6236x_shutdown(struct i2c_client *client)
{
	struct tps6236x_chip *tps = i2c_get_clientdata(client);
	int st;

	if (!tps->enable_discharge)
		return;

	/* Configure the output discharge path */
	st = tps6236x_reg_update(tps, REG_RAMPCTRL, (1 << 2), (1 << 2));
	if (st < 0)
		dev_err(tps->dev, "%s() fails in updating reg %d\n",
			__func__, REG_RAMPCTRL);
}

static const struct i2c_device_id tps6236x_id[] = {
	{.name = "tps62360",  .driver_data = TPS62360},
	{.name = "tps62361B", .driver_data = TPS62361B},
	{},
};

MODULE_DEVICE_TABLE(i2c, tps6236x_id);

static struct i2c_driver tps6236x_i2c_driver = {
	.driver = {
		.name = "tps6236x",
		.owner = THIS_MODULE,
	},
	.probe = tps6236x_probe,
	.remove = __devexit_p(tps6236x_remove),
	.shutdown = tps6236x_shutdown,
	.id_table = tps6236x_id,
};

/* Module init function */
static int __init tps6236x_init(void)
{
	return i2c_add_driver(&tps6236x_i2c_driver);
}
subsys_initcall_sync(tps6236x_init);

/* Module exit function */
static void __exit tps6236x_cleanup(void)
{
	i2c_del_driver(&tps6236x_i2c_driver);
}
module_exit(tps6236x_cleanup);
