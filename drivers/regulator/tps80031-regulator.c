/*
 * driver/regulator/tps80031-regulator.c
 *
 * Regulator driver for TI TPS80031
 *
 * Copyright (C) 2011 NVIDIA Corporation
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
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/mfd/tps80031.h>

#define TPS80031ID_VIO_BASE_ADD		0x47
#define TPS80031ID_SMPS1_BASE_ADD	0x53
#define TPS80031ID_SMPS2_BASE_ADD	0x59
#define TPS80031ID_SMPS3_BASE_ADD	0x65
#define TPS80031ID_SMPS4_BASE_ADD	0x41
#define TPS80031ID_VANA_BASE_ADD	0x81
#define TPS80031ID_VRTC_BASE_ADD	0xC3
#define TPS80031ID_LDO1_BASE_ADD	0x9D
#define TPS80031ID_LDO2_BASE_ADD	0x85
#define TPS80031ID_LDO3_BASE_ADD	0x8D
#define TPS80031ID_LDO4_BASE_ADD	0x89
#define TPS80031ID_LDO5_BASE_ADD	0x99
#define TPS80031ID_LDO6_BASE_ADD	0x91
#define TPS80031ID_LDO7_BASE_ADD	0xA5
#define TPS80031ID_LDOLN_BASE_ADD	0x95
#define TPS80031ID_LDOUSB_BASE_ADD	0xA1

#define VREG_GRP		0

/* Register offsets */
#define VREG_TRANS		0
#define VREG_STATE		1
#define VREG_VOLTAGE		2
#define VREG_VOLTAGE_DCDC	3

/* Flags for DCDC Voltage reading */
#define DCDC_OFFSET_EN		BIT(0)
#define DCDC_EXTENDED_EN	BIT(1)

#define SMPS_MULTOFFSET_VIO	BIT(1)
#define SMPS_MULTOFFSET_SMPS1	BIT(3)
#define SMPS_MULTOFFSET_SMPS2	BIT(4)
#define SMPS_MULTOFFSET_SMPS3	BIT(6)
#define SMPS_MULTOFFSET_SMPS4	BIT(0)

#define PMC_SMPS_OFFSET_ADD	0xE0
#define PMC_SMPS_MULT_ADD	0xE3

#define STATE_OFF	0x00
#define STATE_ON	0x01
#define STATE_MASK	0x03

struct tps80031_regulator {

	/* start of regulator's PM_RECEIVER control register bank */
	u8			base;

	/* twl resource ID, for resource control state machine */
	u8			id;

	/* chip constraints on regulator behavior */
	u16			min_mV;
	u16			max_mV;

	/* regulator specific turn-on delay */
	u16			delay;

	u8			flags;

	/* used by regulator core */
	struct regulator_desc	desc;

	/* Device */
	struct device		*dev;
};

static inline struct device *to_tps80031_dev(struct regulator_dev *rdev)
{
	return rdev_get_dev(rdev)->parent->parent;
}

static int tps80031_regulator_enable_time(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);

	return ri->delay;
}

static u8 tps80031_get_smps_offset(struct device *parent)
{
	u8 value;
	int ret;

	ret = tps80031_read(parent, PMC_SMPS_OFFSET_ADD, &value);
	if (ret < 0) {
		dev_err(parent, "Error in reading smps offset register\n");
		return 0;
	}
	return value;
}

static u8 tps80031_get_smps_mult(struct device *parent)
{
	u8 value;
	int ret;

	ret = tps80031_read(parent, PMC_SMPS_MULT_ADD, &value);
	if (ret < 0) {
		dev_err(parent, "Error in reading smps mult register\n");
		return 0;
	}
	return value;
}

static int tps80031_reg_is_enabled(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	uint8_t state;
	int ret;

	ret = tps80031_read(parent, ri->base + VREG_STATE, &state);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in reading the STATE register\n");
		return ret;
	}
	return ((state & STATE_MASK) == STATE_ON);
}

static int tps80031_reg_enable(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	int ret;

	ret = tps80031_update(parent, ri->base + VREG_STATE, STATE_ON,
					STATE_MASK);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in updating the STATE register\n");
		return ret;
	}
	udelay(ri->delay);
	return ret;
}

static int tps80031_reg_disable(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	int ret;

	ret = tps80031_update(parent, ri->base + VREG_STATE, STATE_OFF,
					STATE_MASK);
	if (ret < 0)
		dev_err(&rdev->dev, "Error in updating the STATE register\n");

	return ret;
}

/*
 * DCDC status and control
 */
static int tps80031dcdc_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	int voltage = 0;

	switch (ri->flags) {
	case 0:
		if (index == 0)
			voltage = 0;
		else if (index < 58)
			voltage = (600000 + (12500 * (index - 1)));
		else if (index == 58)
			voltage = 1350 * 1000;
		else if (index == 59)
			voltage = 1500 * 1000;
		else if (index == 60)
			voltage = 1800 * 1000;
		else if (index == 61)
			voltage = 1900 * 1000;
		else if (index == 62)
			voltage = 2100 * 1000;
		break;

	case DCDC_OFFSET_EN:
		if (index == 0)
			voltage = 0;
		else if (index < 58)
			voltage = (700000 + (12500 * (index - 1)));
		else if (index == 58)
			voltage = 1350 * 1000;
		else if (index == 59)
			voltage = 1500 * 1000;
		else if (index == 60)
			voltage = 1800 * 1000;
		else if (index == 61)
			voltage = 1900 * 1000;
		else if (index == 62)
			voltage = 2100 * 1000;
		break;

	case DCDC_EXTENDED_EN:
		if (index == 0)
			voltage = 0;
		else if (index < 58)
			voltage = (1852000 + (38600 * (index - 1)));
		else if (index == 58)
			voltage = 2084 * 1000;
		else if (index == 59)
			voltage = 2315 * 1000;
		else if (index == 60)
			voltage = 2778 * 1000;
		else if (index == 61)
			voltage = 2932 * 1000;
		else if (index == 62)
			voltage = 3241 * 1000;
		break;

	case DCDC_OFFSET_EN|DCDC_EXTENDED_EN:
		if (index == 0)
			voltage = 0;
		else if (index < 58)
			voltage = (2161000 + (38600 * (index - 1)));
		else if (index == 58)
			voltage = 4167 * 1000;
		else if (index == 59)
			voltage = 2315 * 1000;
		else if (index == 60)
			voltage = 2778 * 1000;
		else if (index == 61)
			voltage = 2932 * 1000;
		else if (index == 62)
			voltage = 3241 * 1000;
		break;
	}

	return voltage;
}

static int __tps80031_dcdc_set_voltage(struct device *parent,
		struct tps80031_regulator *ri, int min_uV, int max_uV)
{
	int vsel = 0;
	int ret;

	switch (ri->flags) {
	case 0:
		if (min_uV == 0)
			vsel = 0;
		else if ((min_uV >= 600000) && (max_uV <= 1300000)) {
			vsel = (min_uV - 600000) / 125;
			if (vsel % 100)
				vsel += 100;
			vsel /= 100;
			vsel++;
		} else if ((min_uV > 1900000) && (max_uV >= 2100000))
			vsel = 62;
		else if ((min_uV > 1800000) && (max_uV >= 1900000))
			vsel = 61;
		else if ((min_uV > 1500000) && (max_uV >= 1800000))
			vsel = 60;
		else if ((min_uV > 1350000) && (max_uV >= 1500000))
			vsel = 59;
		else if ((min_uV > 1300000) && (max_uV >= 1350000))
			vsel = 58;
		else
			return -EINVAL;
		break;

	case DCDC_OFFSET_EN:
		if (min_uV == 0)
			vsel = 0;
		else if ((min_uV >= 700000) && (max_uV <= 1420000)) {
			vsel = (min_uV - 600000) / 125;
			if (vsel % 100)
				vsel += 100;
			vsel /= 100;
			vsel++;
		} else if ((min_uV > 1900000) && (max_uV >= 2100000))
			vsel = 62;
		else if ((min_uV > 1800000) && (max_uV >= 1900000))
			vsel = 61;
		else if ((min_uV > 1350000) && (max_uV >= 1800000))
			vsel = 60;
		else if ((min_uV > 1350000) && (max_uV >= 1500000))
			vsel = 59;
		else if ((min_uV > 1300000) && (max_uV >= 1350000))
			vsel = 58;
		else
			return -EINVAL;
		break;

	case DCDC_EXTENDED_EN:
		if (min_uV == 0)
			vsel = 0;
		else if ((min_uV >= 1852000) && (max_uV <= 4013600)) {
			vsel = (min_uV - 1852000) / 386;
			if (vsel % 100)
				vsel += 100;
			vsel /= 100;
			vsel++;
		}
		break;

	case DCDC_OFFSET_EN|DCDC_EXTENDED_EN:
		if (min_uV == 0)
			vsel = 0;
		else if ((min_uV >= 2161000) && (max_uV <= 4321000)) {
			vsel = (min_uV - 1852000) / 386;
			if (vsel % 100)
				vsel += 100;
			vsel /= 100;
			vsel++;
		}
		break;
	}

	ret = tps80031_write(parent, ri->base + VREG_VOLTAGE_DCDC, vsel);
	if (ret < 0)
		dev_err(ri->dev, "Error in updating the Voltage register\n");
	return ret;
}

static int tps80031dcdc_set_voltage(struct regulator_dev *rdev,
			int min_uV, int max_uV)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	return __tps80031_dcdc_set_voltage(parent, ri, min_uV, max_uV);
}

static int tps80031dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	uint8_t vsel = 0;
	int ret;
	int voltage = 0;

	ret = tps80031_read(parent, ri->base + VREG_VOLTAGE_DCDC, &vsel);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in reading the Voltage register\n");
		return ret;
	}

	switch (ri->flags) {
	case 0:
		if (vsel == 0)
			voltage = 0;
		else if (vsel < 58)
			voltage = (600000 + (12500 * (vsel - 1)));
		else if (vsel == 58)
			voltage = 1350 * 1000;
		else if (vsel == 59)
			voltage = 1500 * 1000;
		else if (vsel == 60)
			voltage = 1800 * 1000;
		else if (vsel == 61)
			voltage = 1900 * 1000;
		else if (vsel == 62)
			voltage = 2100 * 1000;
		break;

	case DCDC_OFFSET_EN:
		if (vsel == 0)
			voltage = 0;
		else if (vsel < 58)
			voltage = (700000 + (12500 * (vsel - 1)));
		else if (vsel == 58)
			voltage = 1350 * 1000;
		else if (vsel == 59)
			voltage = 1500 * 1000;
		else if (vsel == 60)
			voltage = 1800 * 1000;
		else if (vsel == 61)
			voltage = 1900 * 1000;
		else if (vsel == 62)
			voltage = 2100 * 1000;
		break;

	case DCDC_EXTENDED_EN:
		if (vsel == 0)
			voltage = 0;
		else if (vsel < 58)
			voltage = (1852000 + (38600 * (vsel - 1)));
		else if (vsel == 58)
			voltage = 2084 * 1000;
		else if (vsel == 59)
			voltage = 2315 * 1000;
		else if (vsel == 60)
			voltage = 2778 * 1000;
		else if (vsel == 61)
			voltage = 2932 * 1000;
		else if (vsel == 62)
			voltage = 3241 * 1000;
		break;

	case DCDC_EXTENDED_EN|DCDC_OFFSET_EN:
		if (vsel == 0)
			voltage = 0;
		else if (vsel < 58)
			voltage = (2161000 + (38600 * (vsel - 1)));
		else if (vsel == 58)
			voltage = 4167 * 1000;
		else if (vsel == 59)
			voltage = 2315 * 1000;
		else if (vsel == 60)
			voltage = 2778 * 1000;
		else if (vsel == 61)
			voltage = 2932 * 1000;
		else if (vsel == 62)
			voltage = 3241 * 1000;
		break;
	}

	return voltage;
}

static int tps80031ldo_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);

	if (index == 0)
		return 0;

	return (ri->min_mV + (index * 100)) * 1000;
}

static int __tps80031_ldo_set_voltage(struct device *parent,
		struct tps80031_regulator *ri, int min_uV, int max_uV)
{
	int vsel;
	int ret;

	if ((min_uV/1000 < ri->min_mV) || (max_uV/1000 > ri->max_mV))
		return -EDOM;

	/*
	 * Use the below formula to calculate vsel
	 * mV = 1000mv + 100mv * (vsel - 1)
	 */
	vsel = (min_uV/1000 - 1000)/100 + 1;
	ret = tps80031_write(parent, ri->base + VREG_VOLTAGE, vsel);
	if (ret < 0)
		dev_err(ri->dev, "Error in writing the Voltage register\n");
	return ret;
}

static int tps80031ldo_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);

	return __tps80031_ldo_set_voltage(parent, ri, min_uV, max_uV);
}

static int tps80031ldo_get_voltage(struct regulator_dev *rdev)
{
	struct tps80031_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_tps80031_dev(rdev);
	uint8_t vsel;
	int ret;

	ret = tps80031_read(parent, ri->base + VREG_VOLTAGE, &vsel);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in reading the Voltage register\n");
		return ret;
	}
	/*
	 * Use the below formula to calculate vsel
	 * mV = 1000mv + 100mv * (vsel - 1)
	 */
	return (1000 + (100 * (vsel - 1))) * 1000;
}

static struct regulator_ops tps80031dcdc_ops = {
	.list_voltage	= tps80031dcdc_list_voltage,
	.set_voltage	= tps80031dcdc_set_voltage,
	.get_voltage	= tps80031dcdc_get_voltage,
	.enable		= tps80031_reg_enable,
	.disable	= tps80031_reg_disable,
	.is_enabled	= tps80031_reg_is_enabled,
	.enable_time	= tps80031_regulator_enable_time,
};

static struct regulator_ops tps80031ldo_ops = {
	.list_voltage	= tps80031ldo_list_voltage,
	.set_voltage	= tps80031ldo_set_voltage,
	.get_voltage	= tps80031ldo_get_voltage,
	.enable		= tps80031_reg_enable,
	.disable	= tps80031_reg_disable,
	.is_enabled	= tps80031_reg_is_enabled,
	.enable_time	= tps80031_regulator_enable_time,
};

#define TPS80031_REG(_id, min_mVolts, max_mVolts, _ops, _n_volt, _delay) \
{								\
	.base = TPS80031ID_##_id##_BASE_ADD,			\
	.id = TPS80031_ID_##_id,				\
	.min_mV = min_mVolts,					\
	.max_mV = max_mVolts,					\
	.desc = {						\
		.name = tps80031_rails(_id),			\
		.id = TPS80031_ID_##_id,			\
		.n_voltages = _n_volt,				\
		.ops = &_ops,					\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
	},							\
	.delay = _delay,					\
}

static struct tps80031_regulator tps80031_regulator[] = {
	TPS80031_REG(VIO,   600, 2100, tps80031dcdc_ops, 63, 500),
	TPS80031_REG(SMPS1, 600, 2100, tps80031dcdc_ops, 63, 500),
	TPS80031_REG(SMPS2, 600, 2100, tps80031dcdc_ops, 63, 500),
	TPS80031_REG(SMPS3, 600, 2100, tps80031dcdc_ops, 63, 500),
	TPS80031_REG(SMPS4, 600, 2100, tps80031dcdc_ops, 63, 500),

	TPS80031_REG(LDO1,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO2,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO3,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO4,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO5,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO6,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDO7,   1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDOUSB, 1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(LDOLN,  1100, 3300, tps80031ldo_ops, 24, 500),
	TPS80031_REG(VANA,   1100, 3300, tps80031ldo_ops, 24, 500),
};


static inline int tps80031_regulator_preinit(struct device *parent,
		struct tps80031_regulator *ri,
		struct tps80031_regulator_platform_data *tps80031_pdata)
{
	int ret;

	if (!tps80031_pdata->init_apply)
		return 0;

	if (tps80031_pdata->init_uV >= 0) {
		switch (ri->desc.id) {
		case TPS80031_ID_VIO:
		case TPS80031_ID_SMPS1:
		case TPS80031_ID_SMPS2:
		case TPS80031_ID_SMPS3:
		case TPS80031_ID_SMPS4:
			ret = __tps80031_dcdc_set_voltage(parent, ri,
					tps80031_pdata->init_uV,
					tps80031_pdata->init_uV);
			break;

		case TPS80031_ID_LDO1:
		case TPS80031_ID_LDO2:
		case TPS80031_ID_LDO3:
		case TPS80031_ID_LDO4:
		case TPS80031_ID_LDO5:
		case TPS80031_ID_LDO6:
		case TPS80031_ID_LDO7:
		case TPS80031_ID_LDOUSB:
		case TPS80031_ID_LDOLN:
		case TPS80031_ID_VANA:
			ret = __tps80031_ldo_set_voltage(parent, ri,
					tps80031_pdata->init_uV,
					tps80031_pdata->init_uV);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (ret < 0) {
			dev_err(ri->dev, "Not able to initialize voltage %d "
				"for rail %d err %d\n", tps80031_pdata->init_uV,
				ri->desc.id, ret);
			return ret;
		}
	}

	if (tps80031_pdata->init_enable)
		ret = tps80031_update(parent, ri->base + VREG_STATE, STATE_ON,
								STATE_MASK);
	else
		ret = tps80031_update(parent, ri->base + VREG_STATE, STATE_OFF,
								STATE_MASK);
	if (ret < 0)
		dev_err(ri->dev, "Not able to %s rail %d err %d\n",
			(tps80031_pdata->init_enable) ? "enable" : "disable",
			ri->desc.id, ret);
	return ret;
}

static inline struct tps80031_regulator *find_regulator_info(int id)
{
	struct tps80031_regulator *ri;
	int i;

	for (i = 0; i < ARRAY_SIZE(tps80031_regulator); i++) {
		ri = &tps80031_regulator[i];
		if (ri->desc.id == id)
			return ri;
	}
	return NULL;
}
static void check_smps_mode_mult(struct device *parent,
	struct tps80031_regulator *ri)
{
	int mult_offset;
	switch (ri->desc.id) {
	case TPS80031_ID_VIO:
		mult_offset = SMPS_MULTOFFSET_VIO;
		break;
	case TPS80031_ID_SMPS1:
		mult_offset = SMPS_MULTOFFSET_SMPS1;
		break;
	case TPS80031_ID_SMPS2:
		mult_offset = SMPS_MULTOFFSET_SMPS2;
		break;
	case TPS80031_ID_SMPS3:
		mult_offset = SMPS_MULTOFFSET_SMPS3;
		break;
	case TPS80031_ID_SMPS4:
		mult_offset = SMPS_MULTOFFSET_SMPS4;
		break;
	default:
		return;
	}

	ri->flags = (tps80031_get_smps_offset(parent) & mult_offset) ?
						DCDC_OFFSET_EN : 0;
	ri->flags |= (tps80031_get_smps_mult(parent) & mult_offset) ?
						DCDC_EXTENDED_EN : 0;
	return;
}

static int __devinit tps80031_regulator_probe(struct platform_device *pdev)
{
	struct tps80031_regulator *ri = NULL;
	struct regulator_dev *rdev;
	struct tps80031_regulator_platform_data *tps_pdata;
	int id = pdev->id;
	int err;

	dev_dbg(&pdev->dev, "Probing reulator %d\n", id);

	ri = find_regulator_info(id);
	if (ri == NULL) {
		dev_err(&pdev->dev, "invalid regulator ID specified\n");
		return -EINVAL;
	}
	tps_pdata = pdev->dev.platform_data;
	ri->dev = &pdev->dev;

	check_smps_mode_mult(pdev->dev.parent, ri);

	err = tps80031_regulator_preinit(pdev->dev.parent, ri, tps_pdata);
	if (err)
		return err;

	rdev = regulator_register(&ri->desc, &pdev->dev,
				&tps_pdata->regulator, ri);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
				ri->desc.name);
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int __devexit tps80031_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver tps80031_regulator_driver = {
	.driver	= {
		.name	= "tps80031-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= tps80031_regulator_probe,
	.remove		= __devexit_p(tps80031_regulator_remove),
};

static int __init tps80031_regulator_init(void)
{
	return platform_driver_register(&tps80031_regulator_driver);
}
subsys_initcall(tps80031_regulator_init);

static void __exit tps80031_regulator_exit(void)
{
	platform_driver_unregister(&tps80031_regulator_driver);
}
module_exit(tps80031_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Regulator Driver for TI TPS80031 PMIC");
MODULE_ALIAS("platform:tps80031-regulator");
