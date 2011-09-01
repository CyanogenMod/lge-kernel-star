/*
 * drivers/regulator/max77663-regulator.c
 * Maxim LDO and Buck regulators driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max77663-regulator.h>

/* Regulator types */
#define REGULATOR_TYPE_SD		0
#define REGULATOR_TYPE_LDO		1

/* SD and LDO Registers */
#define MAX77663_REG_SD0		0x16
#define MAX77663_REG_SD1		0x17
#define MAX77663_REG_SD2		0x18
#define MAX77663_REG_SD3		0x19
#define MAX77663_REG_SD4		0x1A
#define MAX77663_REG_DVSSD0		0x1B
#define MAX77663_REG_DVSSD1		0x1C
#define MAX77663_REG_SD0_CFG		0x1D
#define MAX77663_REG_DVSSD0_CFG		MAX77663_REG_SD0_CFG
#define MAX77663_REG_SD1_CFG		0x1E
#define MAX77663_REG_DVSSD1_CFG		MAX77663_REG_SD1_CFG
#define MAX77663_REG_SD2_CFG		0x1F
#define MAX77663_REG_SD3_CFG		0x20
#define MAX77663_REG_SD4_CFG		0x21
#define MAX77663_REG_LDO0_CFG		0x23
#define MAX77663_REG_LDO0_CFG2		0x24
#define MAX77663_REG_LDO1_CFG		0x25
#define MAX77663_REG_LDO1_CFG2		0x26
#define MAX77663_REG_LDO2_CFG		0x27
#define MAX77663_REG_LDO2_CFG2		0x28
#define MAX77663_REG_LDO3_CFG		0x29
#define MAX77663_REG_LDO3_CFG2		0x2A
#define MAX77663_REG_LDO4_CFG		0x2B
#define MAX77663_REG_LDO4_CFG2		0x2C
#define MAX77663_REG_LDO5_CFG		0x2D
#define MAX77663_REG_LDO5_CFG2		0x2E
#define MAX77663_REG_LDO6_CFG		0x2F
#define MAX77663_REG_LDO6_CFG2		0x30
#define MAX77663_REG_LDO7_CFG		0x31
#define MAX77663_REG_LDO7_CFG2		0x32
#define MAX77663_REG_LDO8_CFG		0x33
#define MAX77663_REG_LDO8_CFG2		0x34

/* Power Mode */
#define POWER_MODE_NORMAL		3
#define POWER_MODE_LPM			2
#define POWER_MODE_GLPM			1
#define POWER_MODE_DISABLE		0
#define SD_POWER_MODE_MASK		0x30
#define SD_POWER_MODE_SHIFT		4
#define LDO_POWER_MODE_MASK		0xC0
#define LDO_POWER_MODE_SHIFT		6

/* SD Slew Rate */
#define SD_SR_13_75			0
#define SD_SR_27_5			1
#define SD_SR_55			2
#define SD_SR_100			3
#define SD_SR_MASK			0xC0
#define SD_SR_SHIFT			6

/* SD Forced PWM Mode */
#define SD_FPWM_MASK			0x04
#define SD_FPWM_SHIFT			2

/* SD Failling slew rate Active-Discharge Mode */
#define SD_FSRADE_MASK			0x01
#define SD_FSRADE_SHIFT		0

/* Voltage */
#define SDX_VOLT_MASK			0xFF
#define SD1_VOLT_MASK			0x3F
#define LDO_VOLT_MASK			0x3F

/* FPS Registers */
#define MAX77663_REG_FPS_CFG0		0x43
#define MAX77663_REG_FPS_CFG1		0x44
#define MAX77663_REG_FPS_CFG2		0x45
#define MAX77663_REG_FPS_LDO0		0x46
#define MAX77663_REG_FPS_LDO1		0x47
#define MAX77663_REG_FPS_LDO2		0x48
#define MAX77663_REG_FPS_LDO3		0x49
#define MAX77663_REG_FPS_LDO4		0x4A
#define MAX77663_REG_FPS_LDO5		0x4B
#define MAX77663_REG_FPS_LDO6		0x4C
#define MAX77663_REG_FPS_LDO7		0x4D
#define MAX77663_REG_FPS_LDO8		0x4E
#define MAX77663_REG_FPS_SD0		0x4F
#define MAX77663_REG_FPS_SD1		0x50
#define MAX77663_REG_FPS_SD2		0x51
#define MAX77663_REG_FPS_SD3		0x52
#define MAX77663_REG_FPS_SD4		0x53
#define MAX77663_REG_FPS_NONE		0

#define FPS_TIME_PERIOD_MASK		0x38
#define FPS_TIME_PERIOD_SHIFT		3
#define FPS_EN_SRC_MASK			0x06
#define FPS_EN_SRC_SHIFT		1
#define FPS_SW_EN_MASK			0x01
#define FPS_SW_EN_SHIFT			0
#define FPS_SRC_MASK			0xC0
#define FPS_SRC_SHIFT			6
#define FPS_PU_PERIOD_MASK		0x38
#define FPS_PU_PERIOD_SHIFT		3
#define FPS_PD_PERIOD_MASK		0x07
#define FPS_PD_PERIOD_SHIFT		0

struct max77663_regulator {
	struct regulator_dev *rdev;
	struct device *dev;

	u8 id;
	u8 type;
	u32 min_uV;
	u32 max_uV;
	u32 step_uV;
	u32 regulator_mode;

	u8 volt_reg;
	u8 cfg_reg;
	u8 fps_reg;

	enum max77663_regulator_fps_src fps_src;

	u8 volt_mask;

	u8 power_mode;
	u8 power_mode_mask;
	u8 power_mode_shift;
};

#define fps_src_name(fps_src)	\
	(fps_src == FPS_SRC_0 ? "FPS_SRC_0" :	\
	fps_src == FPS_SRC_1 ? "FPS_SRC_1" :	\
	fps_src == FPS_SRC_2 ? "FPS_SRC_2" : "FPS_SRC_NONE")

static int fps_cfg_init;
static u8 fps_cfg_reg[] = {
	MAX77663_REG_FPS_CFG0,
	MAX77663_REG_FPS_CFG1,
	MAX77663_REG_FPS_CFG2
};

static inline struct max77663_regulator_platform_data
*_to_pdata(struct max77663_regulator *reg)
{
	return reg->dev->platform_data;
}

static inline struct device *_to_parent(struct max77663_regulator *reg)
{
	return reg->dev->parent;
}

static int
max77663_regulator_set_fps_src(struct max77663_regulator *reg,
			       enum max77663_regulator_fps_src fps_src)
{
	struct device *parent = _to_parent(reg);
	int ret;

	if (reg->fps_reg == MAX77663_REG_FPS_NONE)
		return 0;

	switch (fps_src) {
	case FPS_SRC_0:
	case FPS_SRC_1:
	case FPS_SRC_2:
	case FPS_SRC_NONE:
		break;
	case FPS_SRC_DEF:
		return 0;
	default:
		return -EINVAL;
	}

	ret = max77663_set_bits(parent, reg->fps_reg, FPS_SRC_MASK,
				fps_src << FPS_SRC_SHIFT, 0);
	if (ret < 0)
		return ret;

	reg->fps_src = fps_src;
	return 0;
}

static int max77663_regulator_set_fps(struct max77663_regulator *reg)
{
	struct max77663_regulator_platform_data *pdata = _to_pdata(reg);
	struct device *parent = _to_parent(reg);
	u8 fps_val = 0, fps_mask = 0;
	int ret = 0;

	if (reg->fps_reg == MAX77663_REG_FPS_NONE)
		return 0;

	if (reg->fps_src == FPS_SRC_NONE)
		return 0;

	/* FPS power up period setting */
	if (pdata->fps_pu_period != FPS_POWER_PERIOD_DEF) {
		fps_val |= (pdata->fps_pu_period << FPS_PU_PERIOD_SHIFT);
		fps_mask |= FPS_PU_PERIOD_MASK;
	}

	/* FPS power down period setting */
	if (pdata->fps_pd_period != FPS_POWER_PERIOD_DEF) {
		fps_val |= (pdata->fps_pd_period << FPS_PD_PERIOD_SHIFT);
		fps_mask |= FPS_PD_PERIOD_MASK;
	}

	if (fps_val)
		ret = max77663_set_bits(parent, reg->fps_reg, fps_mask,
					fps_val, 0);

	return ret;
}

static int
max77663_regulator_set_fps_cfg(struct max77663_regulator *reg,
			       struct max77663_regulator_fps_cfg *fps_cfg)
{
	struct device *parent = _to_parent(reg);
	u8 addr, val, mask;

	if ((fps_cfg->src < FPS_SRC_0) || (fps_cfg->src > FPS_SRC_2))
		return -EINVAL;

	addr = fps_cfg_reg[fps_cfg->src];
	val = (fps_cfg->en_src << FPS_EN_SRC_SHIFT);
	mask = FPS_EN_SRC_MASK;

	if (fps_cfg->time_period != FPS_TIME_PERIOD_DEF) {
		val |= (fps_cfg->time_period << FPS_TIME_PERIOD_SHIFT);
		mask |= FPS_TIME_PERIOD_MASK;
	}

	return max77663_set_bits(parent, addr, mask, val, 0);
}

static int
max77663_regulator_set_fps_cfgs(struct max77663_regulator *reg,
				struct max77663_regulator_fps_cfg *fps_cfgs,
				int num_fps_cfgs)
{
	int i, ret;

	if (fps_cfg_init)
		return 0;

	for (i = 0; i < num_fps_cfgs; i++) {
		ret = max77663_regulator_set_fps_cfg(reg, &fps_cfgs[i]);
		if (ret < 0)
			return ret;
	}
	fps_cfg_init = 1;

	return 0;
}

static int
max77663_regulator_set_power_mode(struct max77663_regulator *reg, u8 power_mode)
{
	struct device *parent = _to_parent(reg);
	u8 addr;
	u8 mask = reg->power_mode_mask;
	u8 shift = reg->power_mode_shift;
	int ret;

	if (reg->type == REGULATOR_TYPE_SD)
		addr = reg->cfg_reg;
	else
		addr = reg->volt_reg;

	ret = max77663_set_bits(parent, addr, mask, power_mode << shift, 0);
	if (ret < 0)
		return ret;

	reg->power_mode = power_mode;
	return ret;
}

static u8 max77663_regulator_get_power_mode(struct max77663_regulator *reg)
{
	struct device *parent = _to_parent(reg);
	u8 addr, val;
	u8 mask = reg->power_mode_mask;
	u8 shift = reg->power_mode_shift;
	int ret;

	if (reg->type == REGULATOR_TYPE_SD)
		addr = reg->cfg_reg;
	else
		addr = reg->volt_reg;

	ret = max77663_read(parent, addr, &val, 1, 0);
	if (ret < 0)
		return ret;

	reg->power_mode = (val & mask) >> shift;
	return reg->power_mode;
}

static int max77663_regulator_do_set_voltage(struct max77663_regulator *reg,
					     int min_uV, int max_uV)
{
	struct device *parent = _to_parent(reg);
	u8 val;

	if (min_uV < reg->min_uV || max_uV > reg->max_uV)
		return -EDOM;

	val = (min_uV - reg->min_uV) / reg->step_uV;
	return max77663_set_bits(parent, reg->volt_reg, reg->volt_mask, val, 0);
}

static int max77663_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uV, int max_uV)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);

	dev_dbg(&rdev->dev, "set_voltage: name=%s, min_uV=%d, max_uV=%d\n",
		rdev->desc->name, min_uV, max_uV);
	return max77663_regulator_do_set_voltage(reg, min_uV, max_uV);
}

static int max77663_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);
	struct device *parent = _to_parent(reg);
	u8 val;
	int volt;
	int ret;

	ret = max77663_read(parent, reg->volt_reg, &val, 1, 0);
	if (ret < 0)
		return ret;

	volt = (val & reg->volt_mask) * reg->step_uV + reg->min_uV;

	dev_dbg(&rdev->dev, "get_voltage: name=%s, volt=%d, val=0x%02x\n",
		rdev->desc->name, volt, val);
	return volt;
}

static int max77663_regulator_enable(struct regulator_dev *rdev)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);
	struct max77663_regulator_platform_data *pdata = _to_pdata(reg);
	int power_mode = POWER_MODE_NORMAL;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "enable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}

	if ((reg->id == MAX77663_REGULATOR_ID_SD0)
			&& (pdata->flags & EN2_CTRL_SD0)) {
		dev_dbg(&rdev->dev,
			"enable: Regulator %s is controlled by EN2\n",
			rdev->desc->name);
		return 0;
	}

	if (reg->regulator_mode == REGULATOR_MODE_STANDBY)
		power_mode = POWER_MODE_LPM;

	return max77663_regulator_set_power_mode(reg, power_mode);
}

static int max77663_regulator_disable(struct regulator_dev *rdev)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);
	struct max77663_regulator_platform_data *pdata = _to_pdata(reg);
	int power_mode = POWER_MODE_DISABLE;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "disable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}

	if ((reg->id == MAX77663_REGULATOR_ID_SD0)
			&& (pdata->flags & EN2_CTRL_SD0)) {
		dev_dbg(&rdev->dev,
			"disable: Regulator %s is controlled by EN2\n",
			rdev->desc->name);
		return 0;
	}

	return max77663_regulator_set_power_mode(reg, power_mode);;
}

static int max77663_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);
	struct max77663_regulator_platform_data *pdata = _to_pdata(reg);
	int ret = 1;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "is_enable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 1;
	}

	if ((reg->id == MAX77663_REGULATOR_ID_SD0)
			&& (pdata->flags & EN2_CTRL_SD0)) {
		dev_dbg(&rdev->dev,
			"is_enable: Regulator %s is controlled by EN2\n",
			rdev->desc->name);
		return 1;
	}

	if (max77663_regulator_get_power_mode(reg) == POWER_MODE_DISABLE)
		ret = 0;

	return ret;
}

static int max77663_regulator_set_mode(struct regulator_dev *rdev,
				       unsigned int mode)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);
	u8 power_mode;
	int ret;

	if (mode == REGULATOR_MODE_NORMAL)
		power_mode = POWER_MODE_NORMAL;
	else if (mode == REGULATOR_MODE_STANDBY)
		power_mode = POWER_MODE_LPM;
	else
		return -EINVAL;

	ret = max77663_regulator_set_power_mode(reg, power_mode);
	if (!ret)
		reg->regulator_mode = mode;

	return ret;
}

static unsigned int max77663_regulator_get_mode(struct regulator_dev *rdev)
{
	struct max77663_regulator *reg = rdev_get_drvdata(rdev);

	return reg->regulator_mode;
}

static struct regulator_ops max77663_ldo_ops = {
	.set_voltage = max77663_regulator_set_voltage,
	.get_voltage = max77663_regulator_get_voltage,
	.enable = max77663_regulator_enable,
	.disable = max77663_regulator_disable,
	.is_enabled = max77663_regulator_is_enabled,
	.set_mode = max77663_regulator_set_mode,
	.get_mode = max77663_regulator_get_mode,
};

static int max77663_regulator_preinit(struct max77663_regulator *reg)
{
	struct max77663_regulator_platform_data *pdata = _to_pdata(reg);
	struct device *parent = _to_parent(reg);
	u8 val;
	int ret;

	/* Update FPS source */
	if (reg->fps_reg == MAX77663_REG_FPS_NONE)
		reg->fps_src = FPS_SRC_NONE;
	else {
		ret = max77663_read(parent, reg->fps_reg, &val, 1, 0);
		if (ret < 0) {
			dev_err(reg->dev,
				"preinit: Failed to get FPS source\n");
			return ret;
		}
		reg->fps_src = (val & FPS_SRC_MASK) >> FPS_SRC_SHIFT;
	}

	/* Set initial state */
	if (!pdata->init_apply)
		goto skip_init_apply;

	if (pdata->init_uV >= 0) {
		ret = max77663_regulator_do_set_voltage(reg, pdata->init_uV,
							pdata->init_uV);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to set voltage to "
				"%d\n", pdata->init_uV);
			return ret;
		}
	}

	if (pdata->init_enable)
		val = POWER_MODE_NORMAL;
	else
		val = POWER_MODE_DISABLE;

	ret = max77663_regulator_set_power_mode(reg, val);
	if (ret < 0) {
		dev_err(reg->dev,
			"preinit: Failed to set power mode to %d\n", val);
		return ret;
	}


skip_init_apply:
	if (reg->type == REGULATOR_TYPE_SD) {
		if (pdata->flags & SD_SLEW_RATE_MASK) {
			if (pdata->flags & SD_SLEW_RATE_SLOWEST)
				val = SD_SR_13_75 << SD_SR_SHIFT;
			else if (pdata->flags & SD_SLEW_RATE_SLOW)
				val = SD_SR_27_5 << SD_SR_SHIFT;
			else if (pdata->flags & SD_SLEW_RATE_FAST)
				val = SD_SR_55 << SD_SR_SHIFT;
			else
				val = SD_SR_100 << SD_SR_SHIFT;

			ret = max77663_set_bits(parent, reg->cfg_reg,
						SD_SR_MASK, val, 0);
			if (ret < 0) {
				dev_err(reg->dev,
					"preinit: Failed to set slew rate\n");
				return ret;
			}
		}

		if (pdata->flags & SD_FORCED_PWM_MODE) {
			ret = max77663_set_bits(parent, reg->cfg_reg,
						SD_FPWM_MASK, SD_FPWM_MASK, 0);
			if (ret < 0) {
				dev_err(reg->dev, "preinit: "
					"Failed to set forced pwm mode\n");
				return ret;
			}
		}

		if (pdata->flags & SD_FSRADE_DISABLE) {
			ret = max77663_set_bits(parent, reg->cfg_reg,
						SD_FSRADE_MASK, SD_FSRADE_MASK, 0);
			if (ret < 0) {
				dev_err(reg->dev, "preinit: "
					"Failed to set falling slew-rate discharge mode\n");
				return ret;
			}
		}

		if ((reg->id == MAX77663_REGULATOR_ID_SD0)
				&& (pdata->flags & EN2_CTRL_SD0)) {
			val = POWER_MODE_DISABLE;
			ret = max77663_regulator_set_power_mode(reg, val);
			if (ret < 0) {
				dev_err(reg->dev, "preinit: "
					"Failed to set power mode to %d for "
					"EN2_CTRL_SD0\n", val);
				return ret;
			}

			if (reg->fps_src == FPS_SRC_NONE)
				return 0;

			ret = max77663_regulator_set_fps_src(reg, FPS_SRC_NONE);
			if (ret < 0) {
				dev_err(reg->dev, "preinit: "
					"Failed to set FPSSRC to FPS_SRC_NONE "
					"for EN2_CTRL_SD0\n");
				return ret;
			}
		}
	}

	ret = max77663_regulator_set_fps_cfgs(reg, pdata->fps_cfgs,
					      pdata->num_fps_cfgs);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPSCFG\n");
		return ret;
	}

	ret = max77663_regulator_set_fps_src(reg, pdata->fps_src);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPSSRC to %d\n",
			pdata->fps_src);
		return ret;
	}

	ret = max77663_regulator_set_fps(reg);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPS\n");
		return ret;
	}

	return 0;
}

#define REGULATOR_SD(_id, _volt_mask, _fps_reg, _min_uV, _max_uV, _step_uV) \
	[MAX77663_REGULATOR_ID_##_id] = {			\
		.id = MAX77663_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_SD,			\
		.volt_reg = MAX77663_REG_##_id,			\
		.volt_mask = _volt_mask##_VOLT_MASK,		\
		.cfg_reg = MAX77663_REG_##_id##_CFG,		\
		.fps_reg = MAX77663_REG_FPS_##_fps_reg,		\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.regulator_mode = REGULATOR_MODE_NORMAL,	\
		.power_mode = POWER_MODE_NORMAL,		\
		.power_mode_mask = SD_POWER_MODE_MASK,		\
		.power_mode_shift = SD_POWER_MODE_SHIFT,	\
	}

#define REGULATOR_LDO(_id, _min_uV, _max_uV, _step_uV)		\
	[MAX77663_REGULATOR_ID_##_id] = {			\
		.id = MAX77663_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_LDO,			\
		.volt_reg = MAX77663_REG_##_id##_CFG,		\
		.volt_mask = LDO_VOLT_MASK,			\
		.cfg_reg = MAX77663_REG_##_id##_CFG2,		\
		.fps_reg = MAX77663_REG_FPS_##_id,		\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.regulator_mode = REGULATOR_MODE_NORMAL,	\
		.power_mode = POWER_MODE_NORMAL,		\
		.power_mode_mask = LDO_POWER_MODE_MASK,		\
		.power_mode_shift = LDO_POWER_MODE_SHIFT,	\
	}

static struct max77663_regulator max77663_regs[MAX77663_REGULATOR_ID_NR] = {
	REGULATOR_SD(SD0,    SDX, SD0,  600000, 3387500, 12500),
	REGULATOR_SD(DVSSD0, SDX, NONE, 600000, 3387500, 12500),
	REGULATOR_SD(SD1,    SD1, SD1,  800000, 1587500, 12500),
	REGULATOR_SD(DVSSD1, SD1, NONE, 800000, 1587500, 12500),
	REGULATOR_SD(SD2,    SDX, SD2,  600000, 3387500, 12500),
	REGULATOR_SD(SD3,    SDX, SD3,  600000, 3387500, 12500),
	REGULATOR_SD(SD4,    SDX, SD4,  600000, 3387500, 12500),

	REGULATOR_LDO(LDO0, 800000, 2350000, 25000),
	REGULATOR_LDO(LDO1, 800000, 2350000, 25000),
	REGULATOR_LDO(LDO2, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO3, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO4, 800000, 1587500, 12500),
	REGULATOR_LDO(LDO5, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO6, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO7, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO8, 800000, 3950000, 50000),
};

#define REGULATOR_DESC(_id, _name)			\
	[MAX77663_REGULATOR_ID_##_id] = {		\
		.name = max77663_rails(_name),		\
		.id = MAX77663_REGULATOR_ID_##_id,	\
		.ops = &max77663_ldo_ops,		\
		.type = REGULATOR_VOLTAGE,		\
		.owner = THIS_MODULE,			\
	}

static struct regulator_desc max77663_rdesc[MAX77663_REGULATOR_ID_NR] = {
	REGULATOR_DESC(SD0, sd0),
	REGULATOR_DESC(DVSSD0, dvssd0),
	REGULATOR_DESC(SD1, sd1),
	REGULATOR_DESC(DVSSD1, dvssd1),
	REGULATOR_DESC(SD2, sd2),
	REGULATOR_DESC(SD3, sd3),
	REGULATOR_DESC(SD4, sd4),
	REGULATOR_DESC(LDO0, ldo0),
	REGULATOR_DESC(LDO1, ldo1),
	REGULATOR_DESC(LDO2, ldo2),
	REGULATOR_DESC(LDO3, ldo3),
	REGULATOR_DESC(LDO4, ldo4),
	REGULATOR_DESC(LDO5, ldo5),
	REGULATOR_DESC(LDO6, ldo6),
	REGULATOR_DESC(LDO7, ldo7),
	REGULATOR_DESC(LDO8, ldo8),
};

static int max77663_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct max77663_regulator *reg;
	struct max77663_regulator_platform_data *pdata;
	int ret = 0;

	if ((pdev->id < 0) || (pdev->id >= MAX77663_REGULATOR_ID_NR)) {
		dev_err(&pdev->dev, "Invalid device id %d\n", pdev->id);
		return -ENODEV;
	}

	rdesc = &max77663_rdesc[pdev->id];
	reg = &max77663_regs[pdev->id];
	reg->dev = &pdev->dev;
	pdata = reg->dev->platform_data;

	dev_dbg(&pdev->dev, "probe: name=%s\n", rdesc->name);

	ret = max77663_regulator_preinit(reg);
	if (ret) {
		dev_err(&pdev->dev, "probe: Failed to preinit regulator %s\n",
			rdesc->name);
		return ret;
	}

	reg->rdev = regulator_register(rdesc, &pdev->dev, &pdata->init_data,
				       reg);
	if (IS_ERR(reg->rdev)) {
		dev_err(&pdev->dev, "probe: Failed to register regulator %s\n",
			rdesc->name);
		return PTR_ERR(reg->rdev);
	}

	return 0;
}

static int max77663_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver max77663_regulator_driver = {
	.probe = max77663_regulator_probe,
	.remove = __devexit_p(max77663_regulator_remove),
	.driver = {
		.name = "max77663-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init max77663_regulator_init(void)
{
	return platform_driver_register(&max77663_regulator_driver);
}
subsys_initcall(max77663_regulator_init);

static void __exit max77663_reg_exit(void)
{
	platform_driver_unregister(&max77663_regulator_driver);
}
module_exit(max77663_reg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("max77663 regulator driver");
MODULE_VERSION("1.0");
