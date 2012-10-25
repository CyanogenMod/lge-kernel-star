/*
 * drivers/mfd/max77663-core.c
 * Max77663 mfd driver (I2C bus access)
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <linux/mfd/max77663-core.h>

/* RTC i2c slave address */
#define MAX77663_RTC_I2C_ADDR		0x48

/* Registers */
#define MAX77663_REG_IRQ_TOP		0x05
#define MAX77663_REG_LBT_IRQ		0x06
#define MAX77663_REG_SD_IRQ		0x07
#define MAX77663_REG_LDOX_IRQ		0x08
#define MAX77663_REG_LDO8_IRQ		0x09
#define MAX77663_REG_GPIO_IRQ		0x0A
#define MAX77663_REG_ONOFF_IRQ		0x0B
#define MAX77663_REG_NVER		0x0C
#define MAX77663_REG_IRQ_TOP_MASK	0x0D
#define MAX77663_REG_LBT_IRQ_MASK	0x0E
#define MAX77663_REG_SD_IRQ_MASK	0x0F
#define MAX77663_REG_LDOX_IRQ_MASK	0x10
#define MAX77663_REG_LDO8_IRQ_MASK	0x11
#define MAX77663_REG_ONOFF_IRQ_MASK	0x12
#define MAX77663_REG_GPIO_CTRL0		0x36
#define MAX77663_REG_GPIO_CTRL1		0x37
#define MAX77663_REG_GPIO_CTRL2		0x38
#define MAX77663_REG_GPIO_CTRL3		0x39
#define MAX77663_REG_GPIO_CTRL4		0x3A
#define MAX77663_REG_GPIO_CTRL5		0x3B
#define MAX77663_REG_GPIO_CTRL6		0x3C
#define MAX77663_REG_GPIO_CTRL7		0x3D
#define MAX77663_REG_GPIO_PU		0x3E
#define MAX77663_REG_GPIO_PD		0x3F
#define MAX77663_REG_GPIO_ALT		0x40
#define MAX77663_REG_ONOFF_CFG1		0x41
#define MAX77663_REG_ONOFF_CFG2		0x42

#define IRQ_TOP_GLBL_MASK		(1 << 7)
#define IRQ_TOP_GLBL_SHIFT		7
#define IRQ_TOP_SD_MASK			(1 << 6)
#define IRQ_TOP_SD_SHIFT		6
#define IRQ_TOP_LDO_MASK		(1 << 5)
#define IRQ_TOP_LDO_SHIFT		5
#define IRQ_TOP_GPIO_MASK		(1 << 4)
#define IRQ_TOP_GPIO_SHIFT		4
#define IRQ_TOP_RTC_MASK		(1 << 3)
#define IRQ_TOP_RTC_SHIFT		3
#define IRQ_TOP_32K_MASK		(1 << 2)
#define IRQ_TOP_32K_SHIFT		2
#define IRQ_TOP_ONOFF_MASK		(1 << 1)
#define IRQ_TOP_ONOFF_SHIFT		1
#define IRQ_TOP_NVER_MASK		(1 << 0)
#define IRQ_TOP_NVER_SHIFT		0

#define IRQ_GLBL_MASK			(1 << 0)

#define IRQ_LBT_BASE			MAX77663_IRQ_LBT_LB
#define IRQ_LBT_END			MAX77663_IRQ_LBT_THERM_ALRM2

#define IRQ_GPIO_BASE			MAX77663_IRQ_GPIO0
#define IRQ_GPIO_END			MAX77663_IRQ_GPIO7

#define IRQ_ONOFF_BASE			MAX77663_IRQ_ONOFF_HRDPOWRN
#define IRQ_ONOFF_END			MAX77663_IRQ_ONOFF_ACOK_RISING

#define GPIO_REG_ADDR(offset)		(MAX77663_REG_GPIO_CTRL0 + offset)

#define GPIO_CTRL_DBNC_MASK		(3 << 6)
#define GPIO_CTRL_DBNC_SHIFT		6
#define GPIO_CTRL_REFE_IRQ_MASK		(3 << 4)
#define GPIO_CTRL_REFE_IRQ_SHIFT	4
#define GPIO_CTRL_DOUT_MASK		(1 << 3)
#define GPIO_CTRL_DOUT_SHIFT		3
#define GPIO_CTRL_DIN_MASK		(1 << 2)
#define GPIO_CTRL_DIN_SHIFT		2
#define GPIO_CTRL_DIR_MASK		(1 << 1)
#define GPIO_CTRL_DIR_SHIFT		1
#define GPIO_CTRL_OUT_DRV_MASK		(1 << 0)
#define GPIO_CTRL_OUT_DRV_SHIFT		0

#define GPIO_REFE_IRQ_NONE		0
#define GPIO_REFE_IRQ_EDGE_FALLING	1
#define GPIO_REFE_IRQ_EDGE_RISING	2
#define GPIO_REFE_IRQ_EDGE_BOTH		3

#define GPIO_DBNC_NONE			0
#define GPIO_DBNC_8MS			1
#define GPIO_DBNC_16MS			2
#define GPIO_DBNC_32MS			3

#define ONOFF_SFT_RST_MASK		(1 << 7)
#define ONOFF_SLPEN_MASK		(1 << 2)

#define ONOFF_SLP_LPM_MASK		(1 << 5)

enum {
	CACHE_IRQ_LBT,
	CACHE_IRQ_SD,
	CACHE_IRQ_LDO,
	CACHE_IRQ_ONOFF,
	CACHE_IRQ_NR,
};

struct max77663_irq_data {
	int mask_reg;
	u16 mask;
	u8 top_mask;
	u8 top_shift;
	int cache_idx;
	bool is_rtc;
	bool is_unmask;
	u8 trigger_type;
};

struct max77663_chip {
	struct device *dev;
	struct i2c_client *i2c_power;
	struct i2c_client *i2c_rtc;

	struct max77663_platform_data *pdata;
	struct mutex io_lock;

	struct irq_chip irq;
	struct mutex irq_lock;
	int irq_base;
	int irq_top_count[8];
	u8 cache_irq_top_mask;
	u16 cache_irq_mask[CACHE_IRQ_NR];

	struct gpio_chip gpio;
	int gpio_base;
	u8 cache_gpio_ctrl[MAX77663_GPIO_NR];
	u8 cache_gpio_pu;
	u8 cache_gpio_pd;
	u8 cache_gpio_alt;
};

struct max77663_chip *max77663_chip;

#define IRQ_DATA_LBT(_name, _shift)			\
	[MAX77663_IRQ_LBT_##_name] = {			\
		.mask_reg = MAX77663_REG_LBT_IRQ_MASK,	\
		.mask = (1 << _shift),			\
		.top_mask = IRQ_TOP_GLBL_MASK,		\
		.top_shift = IRQ_TOP_GLBL_SHIFT,	\
		.cache_idx = CACHE_IRQ_LBT,		\
	}

#define IRQ_DATA_GPIO(_name)				\
	[MAX77663_IRQ_GPIO##_name] = {			\
		.mask = (1 << _name),			\
		.top_mask = IRQ_TOP_GPIO_MASK,		\
		.top_shift = IRQ_TOP_GPIO_SHIFT,	\
		.cache_idx = -1,			\
	}

#define IRQ_DATA_ONOFF(_name, _shift)			\
	[MAX77663_IRQ_ONOFF_##_name] = {		\
		.mask_reg = MAX77663_REG_ONOFF_IRQ_MASK,\
		.mask = (1 << _shift),			\
		.top_mask = IRQ_TOP_ONOFF_MASK,		\
		.top_shift = IRQ_TOP_ONOFF_SHIFT,	\
		.cache_idx = CACHE_IRQ_ONOFF,		\
	}

static struct max77663_irq_data max77663_irqs[MAX77663_IRQ_NR] = {
	IRQ_DATA_LBT(LB, 3),
	IRQ_DATA_LBT(THERM_ALRM1, 2),
	IRQ_DATA_LBT(THERM_ALRM2, 1),
	IRQ_DATA_GPIO(0),
	IRQ_DATA_GPIO(1),
	IRQ_DATA_GPIO(2),
	IRQ_DATA_GPIO(3),
	IRQ_DATA_GPIO(4),
	IRQ_DATA_GPIO(5),
	IRQ_DATA_GPIO(6),
	IRQ_DATA_GPIO(7),
	IRQ_DATA_ONOFF(HRDPOWRN,     0),
	IRQ_DATA_ONOFF(EN0_1SEC,     1),
	IRQ_DATA_ONOFF(EN0_FALLING,  2),
	IRQ_DATA_ONOFF(EN0_RISING,   3),
	IRQ_DATA_ONOFF(LID_FALLING,  4),
	IRQ_DATA_ONOFF(LID_RISING,   5),
	IRQ_DATA_ONOFF(ACOK_FALLING, 6),
	IRQ_DATA_ONOFF(ACOK_RISING,  7),
	[MAX77663_IRQ_RTC] = {
		.top_mask = IRQ_TOP_RTC_MASK,
		.top_shift = IRQ_TOP_RTC_SHIFT,
		.cache_idx = -1,
		.is_rtc = 1,
	},
	[MAX77663_IRQ_SD_PF] = {
		.mask_reg = MAX77663_REG_SD_IRQ_MASK,
		.mask = 0xF8,
		.top_mask = IRQ_TOP_SD_MASK,
		.top_shift = IRQ_TOP_SD_SHIFT,
		.cache_idx = CACHE_IRQ_SD,
	},
	[MAX77663_IRQ_LDO_PF] = {
		.mask_reg = MAX77663_REG_LDOX_IRQ_MASK,
		.mask = 0x1FF,
		.top_mask = IRQ_TOP_LDO_MASK,
		.top_shift = IRQ_TOP_LDO_SHIFT,
		.cache_idx = CACHE_IRQ_LDO,
	},
	[MAX77663_IRQ_32K] = {
		.top_mask = IRQ_TOP_32K_MASK,
		.top_shift = IRQ_TOP_32K_SHIFT,
		.cache_idx = -1,
	},
	[MAX77663_IRQ_NVER] = {
		.top_mask = IRQ_TOP_NVER_MASK,
		.top_shift = IRQ_TOP_NVER_SHIFT,
		.cache_idx = -1,
	},
};

static inline int max77663_i2c_write(struct i2c_client *client, u8 addr,
				     void *src, u32 bytes)
{
	u8 buf[bytes + 1];
	int ret;

	dev_dbg(&client->dev, "i2c_write: addr=0x%02x, src=0x%02x, bytes=%u\n",
		addr, *((u8 *)src), bytes);

	if (client->addr == MAX77663_RTC_I2C_ADDR) {
		/* RTC registers support sequential writing */
		buf[0] = addr;
		memcpy(&buf[1], src, bytes);
	} else {
		/* Power registers support register-data pair writing */
		u8 *src8 = (u8 *)src;
		int i;

		for (i = 0; i < (bytes * 2); i++) {
			if (i % 2)
				buf[i] = *src8++;
			else
				buf[i] = addr++;
		}
		bytes = (bytes * 2) - 1;
	}

	ret = i2c_master_send(client, buf, bytes + 1);
	if (ret < 0)
		return ret;
	return 0;
}

static inline int max77663_i2c_read(struct i2c_client *client, u8 addr,
				    void *dest, u32 bytes)
{
	int ret;

	if (bytes > 1) {
		ret = i2c_smbus_read_i2c_block_data(client, addr, bytes, dest);
		if (ret < 0)
			return ret;
	} else {
		ret = i2c_smbus_read_byte_data(client, addr);
		if (ret < 0)
			return ret;

		*((u8 *)dest) = (u8)ret;
	}

	dev_dbg(&client->dev, "i2c_read: addr=0x%02x, dest=0x%02x, bytes=%u\n",
		addr, *((u8 *)dest), bytes);
	return 0;
}

int max77663_read(struct device *dev, u8 addr, void *values, u32 len,
		  bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = NULL;
	int ret;

	mutex_lock(&chip->io_lock);
	if (!is_rtc)
		client = chip->i2c_power;
	else
		client = chip->i2c_rtc;

	ret = max77663_i2c_read(client, addr, values, len);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_read);

int max77663_write(struct device *dev, u8 addr, void *values, u32 len,
		   bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = NULL;
	int ret;

	mutex_lock(&chip->io_lock);
	if (!is_rtc)
		client = chip->i2c_power;
	else
		client = chip->i2c_rtc;

	ret = max77663_i2c_write(client, addr, values, len);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_write);

int max77663_set_bits(struct device *dev, u8 addr, u8 mask, u8 value,
		      bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = NULL;
	u8 tmp;
	int ret;

	mutex_lock(&chip->io_lock);
	if (!is_rtc)
		client = chip->i2c_power;
	else
		client = chip->i2c_rtc;

	ret = max77663_i2c_read(client, addr, &tmp, 1);
	if (ret == 0) {
		value = (tmp & ~mask) | (value & mask);
		ret = max77663_i2c_write(client, addr, &value, 1);
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_set_bits);

int max77663_power_off(void)
{
	struct max77663_chip *chip = max77663_chip;

	if (!chip)
		return -EINVAL;

	dev_info(chip->dev, "%s: Global shutdown\n", __func__);
	return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
				 ONOFF_SFT_RST_MASK, ONOFF_SFT_RST_MASK, 0);
}
EXPORT_SYMBOL(max77663_power_off);

static int max77663_sleep(struct max77663_chip *chip, bool on)
{
	int ret = 0;

	if (chip->pdata->flags & SLP_LPM_ENABLE) {
		/* Put the power rails into Low-Power mode during sleep mode,
		 * if the power rail's power mode is GLPM. */
		ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG2,
					ONOFF_SLP_LPM_MASK,
					on ? ONOFF_SLP_LPM_MASK : 0, 0);
		if (ret < 0)
			return ret;
	}

	/* Enable sleep that AP can be placed into sleep mode
	 * by pulling EN1 low */
	return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
				 ONOFF_SLPEN_MASK,
				 on ? ONOFF_SLPEN_MASK : 0, 0);
}

static inline int max77663_cache_write(struct device *dev, u8 addr, u8 mask,
				       u8 val, u8 *cache)
{
	u8 new_val;
	int ret;

	new_val = (*cache & ~mask) | (val & mask);
	if (*cache != new_val) {
		ret = max77663_write(dev, addr, &new_val, 1, 0);
		if (ret < 0)
			return ret;
		*cache = new_val;
	}
	return 0;
}

static inline
struct max77663_chip *max77663_chip_from_gpio(struct gpio_chip *gpio)
{
	return container_of(gpio, struct max77663_chip, gpio);
}

static int max77663_gpio_set_pull_up(struct max77663_chip *chip, int offset,
				     int pull_up)
{
	u8 val = 0;

	if ((offset < MAX77663_GPIO0) || (MAX77663_GPIO7 < offset))
		return -EINVAL;

	if (pull_up == GPIO_PU_ENABLE)
		val = (1 << offset);

	return max77663_cache_write(chip->dev, MAX77663_REG_GPIO_PU,
				    (1 << offset), val, &chip->cache_gpio_pu);
}

static int max77663_gpio_set_pull_down(struct max77663_chip *chip, int offset,
				       int pull_down)
{
	u8 val = 0;

	if ((offset < MAX77663_GPIO0) || (MAX77663_GPIO7 < offset))
		return -EINVAL;

	if (pull_down == GPIO_PD_ENABLE)
		val = (1 << offset);

	return max77663_cache_write(chip->dev, MAX77663_REG_GPIO_PD,
				    (1 << offset), val, &chip->cache_gpio_pd);
}

static inline
int max77663_gpio_is_alternate(struct max77663_chip *chip, int offset)
{
	return (chip->cache_gpio_alt & (1 << offset)) ? 1 : 0;
}

int max77663_gpio_set_alternate(int gpio, int alternate)
{
	struct max77663_chip *chip = max77663_chip;
	u8 val = 0;
	int ret = 0;

	if (!chip)
		return -ENXIO;

	gpio -= chip->gpio_base;
	if ((gpio < MAX77663_GPIO0) || (MAX77663_GPIO7 < gpio))
		return -EINVAL;

	if (alternate == GPIO_ALT_ENABLE) {
		val = (1 << gpio);
		if (gpio == MAX77663_GPIO7) {
			ret = max77663_gpio_set_pull_up(chip, gpio, 0);
			if (ret < 0)
				return ret;

			ret = max77663_gpio_set_pull_down(chip, gpio, 0);
			if (ret < 0)
				return ret;
		}
	}

	return max77663_cache_write(chip->dev, MAX77663_REG_GPIO_ALT,
				    (1 << gpio), val, &chip->cache_gpio_alt);
}
EXPORT_SYMBOL(max77663_gpio_set_alternate);

static int max77663_gpio_dir_input(struct gpio_chip *gpio, unsigned offset)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);

	if (max77663_gpio_is_alternate(chip, offset)) {
		dev_warn(chip->dev, "gpio_dir_input: "
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	return max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset),
				    GPIO_CTRL_DIR_MASK, GPIO_CTRL_DIR_MASK,
				    &chip->cache_gpio_ctrl[offset]);
}

static int max77663_gpio_get(struct gpio_chip *gpio, unsigned offset)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);
	u8 val;
	int ret;

	if (max77663_gpio_is_alternate(chip, offset)) {
		dev_warn(chip->dev, "gpio_get: "
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	ret = max77663_read(chip->dev, GPIO_REG_ADDR(offset), &val, 1, 0);
	if (ret < 0)
		return ret;

	chip->cache_gpio_ctrl[offset] = val;
	return (val & GPIO_CTRL_DIN_MASK) >> GPIO_CTRL_DIN_SHIFT;
}

static int max77663_gpio_dir_output(struct gpio_chip *gpio, unsigned offset,
				    int value)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);
	u8 mask = GPIO_CTRL_DIR_MASK | GPIO_CTRL_DOUT_MASK;
	u8 val = (value ? 1 : 0) << GPIO_CTRL_DOUT_SHIFT;

	if (max77663_gpio_is_alternate(chip, offset)) {
		dev_warn(chip->dev, "gpio_dir_output: "
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	return max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset), mask, val,
				    &chip->cache_gpio_ctrl[offset]);
}

static int max77663_gpio_set_debounce(struct gpio_chip *gpio, unsigned offset,
				      unsigned debounce)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);
	u8 shift = GPIO_CTRL_DBNC_SHIFT;
	u8 val = 0;

	if (max77663_gpio_is_alternate(chip, offset)) {
		dev_warn(chip->dev, "gpio_set_debounce: "
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	if (debounce == 0)
		val = 0;
	else if ((0 < debounce) && (debounce <= 8))
		val = (GPIO_DBNC_8MS << shift);
	else if ((8 < debounce) && (debounce <= 16))
		val = (GPIO_DBNC_16MS << shift);
	else if ((16 < debounce) && (debounce <= 32))
		val = (GPIO_DBNC_32MS << shift);
	else
		return -EINVAL;

	return max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset),
				    GPIO_CTRL_DBNC_MASK, val,
				    &chip->cache_gpio_ctrl[offset]);
}

static void max77663_gpio_set(struct gpio_chip *gpio, unsigned offset,
			      int value)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);
	u8 val = (value ? 1 : 0) << GPIO_CTRL_DOUT_SHIFT;

	if (max77663_gpio_is_alternate(chip, offset)) {
		dev_warn(chip->dev, "gpio_set: "
			"gpio%u is used as alternate mode\n", offset);
		return;
	}

	max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset),
			     GPIO_CTRL_DOUT_MASK, val,
			     &chip->cache_gpio_ctrl[offset]);
}

static int max77663_gpio_to_irq(struct gpio_chip *gpio, unsigned offset)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);

	return chip->irq_base + IRQ_GPIO_BASE + offset;
}

#ifdef CONFIG_DEBUG_FS
static void max77663_gpio_dbg_show(struct seq_file *s, struct gpio_chip *gpio)
{
	struct max77663_chip *chip = max77663_chip_from_gpio(gpio);
	int i;

	for (i = 0; i < gpio->ngpio; i++) {
		u8 ctrl_val;
		const char *label;
		int is_out;
		int ret;

		label = gpiochip_is_requested(gpio, i);
		if (!label)
			label = "Unrequested";

		seq_printf(s, " gpio-%-3d (%-20.20s) ", i + chip->gpio_base,
			   label);

		if (chip->cache_gpio_alt & (1 << i)) {
			seq_printf(s, "alt\n");
			continue;
		}

		ret = max77663_read(chip->dev, GPIO_REG_ADDR(i), &ctrl_val, 1,
				    0);
		if (ret < 0) {
			seq_printf(s, "\n");
			continue;
		}

		is_out = ctrl_val & GPIO_CTRL_DIR_MASK ? 0 : 1;
		seq_printf(s, "%s %s", (is_out ? "out" : "in"), (is_out ?
			   (ctrl_val & GPIO_CTRL_DOUT_MASK ? "hi" : "lo")
			   : (ctrl_val & GPIO_CTRL_DIN_MASK ? "hi" : "lo")));

		if (!is_out) {
			int irq = gpio_to_irq(i + chip->gpio_base);
			struct irq_desc *desc = irq_to_desc(irq);
			u8 dbnc;

			if (irq >= 0 && desc->action) {
				u8 mask = GPIO_CTRL_REFE_IRQ_MASK;
				u8 shift = GPIO_CTRL_REFE_IRQ_SHIFT;
				char *trigger;

				switch ((ctrl_val & mask) >> shift) {
				case GPIO_REFE_IRQ_EDGE_FALLING:
					trigger = "edge-falling";
					break;
				case GPIO_REFE_IRQ_EDGE_RISING:
					trigger = "edge-rising";
					break;
				case GPIO_REFE_IRQ_EDGE_BOTH:
					trigger = "edge-both";
					break;
				default:
					trigger = "masked";
					break;
				}

				seq_printf(s, " irq-%d %s", irq, trigger);
			}

			dbnc = (ctrl_val & GPIO_CTRL_DBNC_MASK)
				>> GPIO_CTRL_DBNC_SHIFT;
			seq_printf(s, " debounce-%s",
				   dbnc ==  GPIO_DBNC_8MS ? "8ms" :
				   dbnc ==  GPIO_DBNC_16MS ? "16ms" :
				   dbnc ==  GPIO_DBNC_32MS ? "32ms" : "none");
		} else {
			seq_printf(s, " %s",
				   (ctrl_val & GPIO_CTRL_OUT_DRV_MASK ?
				   "output-drive" : "open-drain"));
		}

		seq_printf(s, "\n");
	}
}
#else
#define max77663_gpio_dbg_show		NULL
#endif /* CONFIG_DEBUG_FS */

static int max77663_gpio_set_config(struct max77663_chip *chip,
				    struct max77663_gpio_config *gpio_cfg)
{
	int gpio = gpio_cfg->gpio;
	u8 val = 0, mask = 0;
	int ret = 0;

	if ((gpio < MAX77663_GPIO0) || (MAX77663_GPIO7 < gpio))
		return -EINVAL;

	if (gpio_cfg->pull_up != GPIO_PU_DEF) {
		ret = max77663_gpio_set_pull_up(chip, gpio, gpio_cfg->pull_up);
		if (ret < 0) {
			dev_err(chip->dev, "gpio_set_config: "
				"Failed to set gpio%d pull-up\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->pull_down != GPIO_PD_DEF) {
		ret = max77663_gpio_set_pull_down(chip, gpio,
						  gpio_cfg->pull_down);
		if (ret < 0) {
			dev_err(chip->dev, "gpio_set_config: "
				"Failed to set gpio%d pull-down\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->dir != GPIO_DIR_DEF) {
		mask = GPIO_CTRL_DIR_MASK;
		if (gpio_cfg->dir == GPIO_DIR_IN) {
			val |= GPIO_CTRL_DIR_MASK;
		} else {
			if (gpio_cfg->dout != GPIO_DOUT_DEF) {
				mask |= GPIO_CTRL_DOUT_MASK;
				if (gpio_cfg->dout == GPIO_DOUT_HIGH)
					val |= GPIO_CTRL_DOUT_MASK;
			}

			if (gpio_cfg->out_drv != GPIO_OUT_DRV_DEF) {
				mask |= GPIO_CTRL_OUT_DRV_MASK;
				if (gpio_cfg->out_drv == GPIO_OUT_DRV_PUSH_PULL)
					val |= GPIO_CTRL_OUT_DRV_MASK;
			}
		}

		ret = max77663_cache_write(chip->dev, GPIO_REG_ADDR(gpio), mask,
					   val, &chip->cache_gpio_ctrl[gpio]);
		if (ret < 0) {
			dev_err(chip->dev, "gpio_set_config: "
				"Failed to set gpio%d control\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->alternate != GPIO_ALT_DEF) {
		ret = max77663_gpio_set_alternate(gpio + chip->gpio_base,
						  gpio_cfg->alternate);
		if (ret < 0) {
			dev_err(chip->dev, "gpio_set_config: "
				"Failed to set gpio%d alternate\n", gpio);
			return ret;
		}
	}

	return 0;
}

static int max77663_gpio_init(struct max77663_chip *chip)
{
	int i;
	int ret;

	chip->gpio.label = chip->i2c_power->name;
	chip->gpio.dev = chip->dev;
	chip->gpio.owner = THIS_MODULE;
	chip->gpio.direction_input = max77663_gpio_dir_input;
	chip->gpio.get = max77663_gpio_get;
	chip->gpio.direction_output = max77663_gpio_dir_output;
	chip->gpio.set_debounce = max77663_gpio_set_debounce;
	chip->gpio.set = max77663_gpio_set;
	chip->gpio.to_irq = max77663_gpio_to_irq;
	chip->gpio.dbg_show = max77663_gpio_dbg_show;
	chip->gpio.ngpio = MAX77663_GPIO_NR;
	chip->gpio.can_sleep = 1;
	if (chip->gpio_base)
		chip->gpio.base = chip->gpio_base;
	else
		chip->gpio.base = -1;

	ret = max77663_read(chip->dev, MAX77663_REG_GPIO_CTRL0,
			    chip->cache_gpio_ctrl, MAX77663_GPIO_NR, 0);
	if (ret < 0) {
		dev_err(chip->dev, "gpio_init: Failed to get gpio control\n");
		return ret;
	}

	ret = max77663_read(chip->dev, MAX77663_REG_GPIO_PU,
			    &chip->cache_gpio_pu, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "gpio_init: Failed to get gpio pull-up\n");
		return ret;
	}

	ret = max77663_read(chip->dev, MAX77663_REG_GPIO_PD,
			    &chip->cache_gpio_pd, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "gpio_init: Failed to get gpio pull-down\n");
		return ret;
	}

	ret = max77663_read(chip->dev, MAX77663_REG_GPIO_ALT,
			    &chip->cache_gpio_alt, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "gpio_init: Failed to get gpio alternate\n");
		return ret;
	}

	ret = gpiochip_add(&chip->gpio);
	if (ret < 0) {
		dev_err(chip->dev, "gpio_init: Failed to add gpiochip\n");
		return ret;
	}
	chip->gpio_base = chip->gpio.base;

	for (i = 0; i < chip->pdata->num_gpio_cfgs; i++) {
		ret = max77663_gpio_set_config(chip,
					       &chip->pdata->gpio_cfgs[i]);
		if (ret < 0) {
			dev_err(chip->dev,
				"gpio_init: Failed to set gpio config\n");
			return ret;
		}
	}

	return 0;
}

static void max77663_gpio_exit(struct max77663_chip *chip)
{
	if (gpiochip_remove(&chip->gpio) < 0)
		dev_err(chip->dev, "gpio_exit: Failed to remove gpiochip\n");
}

static void max77663_irq_mask(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	max77663_irqs[data->irq - chip->irq_base].is_unmask = 0;
}

static void max77663_irq_unmask(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	max77663_irqs[data->irq - chip->irq_base].is_unmask = 1;
}

static void max77663_irq_lock(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->irq_lock);
}

static void max77663_irq_sync_unlock(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);
	struct max77663_irq_data *irq_data =
			&max77663_irqs[data->irq - chip->irq_base];
	int idx = irq_data->cache_idx;
	u8 irq_top_mask = chip->cache_irq_top_mask;
	u16 irq_mask = chip->cache_irq_mask[idx];
	int update_irq_top = 0;
	u32 len = 1;
	int ret;

	if (irq_data->is_unmask) {
		if (chip->irq_top_count[irq_data->top_shift] == 0)
			update_irq_top = 1;
		chip->irq_top_count[irq_data->top_shift]++;

		if (irq_data->top_mask != IRQ_TOP_GLBL_MASK)
			irq_top_mask &= ~irq_data->top_mask;

		if (idx != -1)
			irq_mask &= ~irq_data->mask;
	} else {
		if (chip->irq_top_count[irq_data->top_shift] == 1)
			update_irq_top = 1;

		if (--chip->irq_top_count[irq_data->top_shift] < 0)
			chip->irq_top_count[irq_data->top_shift] = 0;

		if (irq_data->top_mask != IRQ_TOP_GLBL_MASK)
			irq_top_mask |= irq_data->top_mask;

		if (idx != -1)
			irq_mask |= irq_data->mask;
	}

	if ((idx != -1) && (irq_mask != chip->cache_irq_mask[idx])) {
		if (irq_data->top_mask == IRQ_TOP_LDO_MASK)
			len = 2;

		ret = max77663_write(chip->dev, irq_data->mask_reg,
				     &irq_mask, len, irq_data->is_rtc);
		if (ret < 0)
			goto out;

		chip->cache_irq_mask[idx] = irq_mask;
	} else if ((idx == -1) && (irq_data->top_mask == IRQ_TOP_GPIO_MASK)) {
		unsigned offset = data->irq - chip->irq_base - IRQ_GPIO_BASE;
		u8 shift = GPIO_CTRL_REFE_IRQ_SHIFT;

		if (irq_data->is_unmask) {
			if (irq_data->trigger_type)
				irq_mask = irq_data->trigger_type;
			else
				irq_mask = GPIO_REFE_IRQ_EDGE_FALLING << shift;
		}

		ret = max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset),
					   GPIO_CTRL_REFE_IRQ_MASK, irq_mask,
					   &chip->cache_gpio_ctrl[offset]);
		if (ret < 0)
			goto out;

		if (irq_data->is_unmask)
			irq_data->trigger_type = irq_mask;
	}

	if (update_irq_top && (irq_top_mask != chip->cache_irq_top_mask)) {
		ret = max77663_cache_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
					   irq_data->top_mask, irq_top_mask,
					   &chip->cache_irq_top_mask);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&chip->irq_lock);
}

static int max77663_irq_gpio_set_type(struct irq_data *data, unsigned int type)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);
	struct max77663_irq_data *irq_data =
			&max77663_irqs[data->irq - chip->irq_base];
	unsigned offset = data->irq - chip->irq_base - IRQ_GPIO_BASE;
	u8 shift = GPIO_CTRL_REFE_IRQ_SHIFT;
	u8 val;

	switch (type) {
	case IRQ_TYPE_NONE:
	case IRQ_TYPE_EDGE_FALLING:
		val = (GPIO_REFE_IRQ_EDGE_FALLING << shift);
		break;
	case IRQ_TYPE_EDGE_RISING:
		val = (GPIO_REFE_IRQ_EDGE_RISING << shift);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		val = (GPIO_REFE_IRQ_EDGE_BOTH << shift);
		break;
	default:
		return -EINVAL;
	}

	irq_data->trigger_type = val;
	if (!(chip->cache_gpio_ctrl[offset] & GPIO_CTRL_REFE_IRQ_MASK))
		return 0;

	return max77663_cache_write(chip->dev, GPIO_REG_ADDR(offset),
				    GPIO_CTRL_REFE_IRQ_MASK, val,
				    &chip->cache_gpio_ctrl[offset]);
}

static inline int max77663_do_irq(struct max77663_chip *chip, u8 addr,
				  int irq_base, int irq_end)
{
	struct max77663_irq_data *irq_data = NULL;
	int irqs_to_handle[irq_end - irq_base + 1];
	int handled = 0;
	u16 val;
	u32 len = 1;
	int i;
	int ret;

	ret = max77663_read(chip->dev, addr, &val, len, 0);
	if (ret < 0)
		return ret;

	for (i = irq_base; i <= irq_end; i++) {
		irq_data = &max77663_irqs[i];
		if (val & irq_data->mask) {
			irqs_to_handle[handled] = i + chip->irq_base;
			handled++;
		}
	}

	for (i = 0; i < handled; i++)
		handle_nested_irq(irqs_to_handle[i]);

	return 0;
}

static irqreturn_t max77663_irq(int irq, void *data)
{
	struct max77663_chip *chip = data;
	u8 irq_top;
	int ret;

	ret = max77663_read(chip->dev, MAX77663_REG_IRQ_TOP, &irq_top, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "irq: Failed to get irq top status\n");
		return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_GLBL_MASK) {
		ret = max77663_do_irq(chip, MAX77663_REG_LBT_IRQ, IRQ_LBT_BASE,
				      IRQ_LBT_END);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_GPIO_MASK) {
		ret = max77663_do_irq(chip, MAX77663_REG_GPIO_IRQ,
				      IRQ_GPIO_BASE, IRQ_GPIO_END);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_ONOFF_MASK) {
		ret = max77663_do_irq(chip, MAX77663_REG_ONOFF_IRQ,
				      IRQ_ONOFF_BASE, IRQ_ONOFF_END);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_RTC_MASK)
		handle_nested_irq(MAX77663_IRQ_RTC + chip->irq_base);

	if (irq_top & IRQ_TOP_SD_MASK)
		handle_nested_irq(MAX77663_IRQ_SD_PF + chip->irq_base);

	if (irq_top & IRQ_TOP_LDO_MASK)
		handle_nested_irq(MAX77663_IRQ_LDO_PF + chip->irq_base);

	if (irq_top & IRQ_TOP_32K_MASK)
		handle_nested_irq(MAX77663_IRQ_32K + chip->irq_base);

	if (irq_top & IRQ_TOP_NVER_MASK)
		handle_nested_irq(MAX77663_IRQ_NVER + chip->irq_base);

	return IRQ_HANDLED;
}

static struct irq_chip max77663_irq_gpio_chip = {
	.name = "max77663-irq",
	.irq_mask = max77663_irq_mask,
	.irq_unmask = max77663_irq_unmask,
	.irq_set_type = max77663_irq_gpio_set_type,
	.irq_bus_lock = max77663_irq_lock,
	.irq_bus_sync_unlock = max77663_irq_sync_unlock,
};

static struct irq_chip max77663_irq_chip = {
	.name = "max77663-irq",
	.irq_mask = max77663_irq_mask,
	.irq_unmask = max77663_irq_unmask,
	.irq_bus_lock = max77663_irq_lock,
	.irq_bus_sync_unlock = max77663_irq_sync_unlock,
};

static int max77663_irq_init(struct max77663_chip *chip)
{
	u32 temp;
	int i, ret = 0;

	mutex_init(&chip->irq_lock);

	/* Mask all interrupts */
	chip->cache_irq_top_mask = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_LBT] = 0x0F;
	chip->cache_irq_mask[CACHE_IRQ_SD] = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_LDO] = 0xFFFF;
	chip->cache_irq_mask[CACHE_IRQ_ONOFF] = 0xFF;

	max77663_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
		       &chip->cache_irq_top_mask, 1, 0);
	max77663_write(chip->dev, MAX77663_REG_LBT_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LBT], 1, 0);
	max77663_write(chip->dev, MAX77663_REG_SD_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_SD], 1, 0);
	max77663_write(chip->dev, MAX77663_REG_LDOX_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LDO], 2, 0);
	max77663_write(chip->dev, MAX77663_REG_ONOFF_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_ONOFF], 1, 0);

	/* Clear all interrups */
	max77663_read(chip->dev, MAX77663_REG_LBT_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_SD_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_LDOX_IRQ, &temp, 2, 0);
	max77663_read(chip->dev, MAX77663_REG_GPIO_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_ONOFF_IRQ, &temp, 1, 0);

	for (i = chip->irq_base; i < (MAX77663_IRQ_NR + chip->irq_base); i++) {
		if (i >= NR_IRQS) {
			dev_err(chip->dev,
				"irq_init: Can't set irq chip for irq %d\n", i);
			continue;
		}

		irq_set_chip_data(i, chip);

		if ((IRQ_GPIO_BASE <= i - chip->irq_base) &&
				(i - chip->irq_base <= IRQ_GPIO_END))
			irq_set_chip_and_handler(i, &max77663_irq_gpio_chip,
						 handle_edge_irq);
		else
			irq_set_chip_and_handler(i, &max77663_irq_chip,
						 handle_edge_irq);
#ifdef CONFIG_ARM
		set_irq_flags(i, IRQF_VALID);
#else
		irq_set_noprobe(i);
#endif
		irq_set_nested_thread(i, 1);
	}

	ret = request_threaded_irq(chip->i2c_power->irq, NULL, max77663_irq,
				   IRQF_ONESHOT, "max77663", chip);
	if (ret) {
		dev_err(chip->dev, "irq_init: Failed to request irq %d\n",
			chip->i2c_power->irq);
		return ret;
	}

	device_init_wakeup(chip->dev, 1);
	enable_irq_wake(chip->i2c_power->irq);

	chip->cache_irq_top_mask &= ~IRQ_TOP_GLBL_MASK;
	max77663_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
		       &chip->cache_irq_top_mask, 1, 0);

	chip->cache_irq_mask[CACHE_IRQ_LBT] &= ~IRQ_GLBL_MASK;
	max77663_write(chip->dev, MAX77663_REG_LBT_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LBT], 1, 0);

	return 0;
}

static void max77663_irq_exit(struct max77663_chip *chip)
{
	if (chip->i2c_power->irq)
		free_irq(chip->i2c_power->irq, chip);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *max77663_dentry_regs;

static int max77663_debugfs_dump_regs(struct max77663_chip *chip, char *label,
				      u8 *addrs, int num_addrs, char *buf,
				      ssize_t *len, int is_rtc)
{
	ssize_t count = *len;
	u8 val;
	int ret = 0;
	int i;

	count += sprintf(buf + count, "%s\n", label);
	if (count >= PAGE_SIZE - 1)
		return -ERANGE;

	for (i = 0; i < num_addrs; i++) {
		count += sprintf(buf + count, "0x%02x: ", addrs[i]);
		if (count >= PAGE_SIZE - 1)
			return -ERANGE;

		ret = max77663_read(chip->dev, addrs[i], &val, 1, is_rtc);
		if (ret == 0)
			count += sprintf(buf + count, "0x%02x\n", val);
		else
			count += sprintf(buf + count, "<read fail: %d>\n", ret);

		if (count >= PAGE_SIZE - 1)
			return -ERANGE;
	}

	*len = count;
	return 0;
}

static int max77663_debugfs_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t max77663_debugfs_regs_read(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77663_chip *chip = file->private_data;
	char *buf;
	size_t len = 0;
	ssize_t ret;

	/* Excluded interrupt status register to prevent register clear */
	u8 global_regs[] = { 0x00, 0x01, 0x02, 0x05, 0x0D, 0x0E, 0x13 };
	u8 sd_regs[] = {
		0x07, 0x0F, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
		0x1E, 0x1F, 0x20, 0x21, 0x22
	};
	u8 ldo_regs[] = {
		0x10, 0x11, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
		0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
		0x35
	};
	u8 gpio_regs[] = {
		0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
		0x40
	};
	u8 rtc_regs[] = {
		0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
		0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B
	};
	u8 osc_32k_regs[] = { 0x03 };
	u8 bbc_regs[] = { 0x04 };
	u8 onoff_regs[] = { 0x12, 0x15, 0x41, 0x42 };
	u8 fps_regs[] = {
		0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
		0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
		0x57
	};
	u8 cid_regs[] = { 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D };

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += sprintf(buf + len, "MAX77663 Registers\n");
	max77663_debugfs_dump_regs(chip, "[Global]", global_regs,
				   ARRAY_SIZE(global_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Step-Down]", sd_regs,
				   ARRAY_SIZE(sd_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[LDO]", ldo_regs,
				   ARRAY_SIZE(ldo_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[GPIO]", gpio_regs,
				   ARRAY_SIZE(gpio_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[RTC]", rtc_regs,
				   ARRAY_SIZE(rtc_regs), buf, &len, 1);
	max77663_debugfs_dump_regs(chip, "[32kHz Oscillator]", osc_32k_regs,
				   ARRAY_SIZE(osc_32k_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Backup Battery Charger]", bbc_regs,
				   ARRAY_SIZE(bbc_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[On/OFF Controller]", onoff_regs,
				   ARRAY_SIZE(onoff_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Flexible Power Sequencer]", fps_regs,
				   ARRAY_SIZE(fps_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Chip Identification]", cid_regs,
				   ARRAY_SIZE(cid_regs), buf, &len, 0);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static const struct file_operations max77663_debugfs_regs_fops = {
	.open = max77663_debugfs_regs_open,
	.read = max77663_debugfs_regs_read,
};

static void max77663_debugfs_init(struct max77663_chip *chip)
{
	max77663_dentry_regs = debugfs_create_file(chip->i2c_power->name,
						   0444, 0, chip,
						   &max77663_debugfs_regs_fops);
	if (!max77663_dentry_regs)
		dev_warn(chip->dev,
			 "debugfs_init: Failed to create debugfs file\n");
}

static void max77663_debugfs_exit(struct max77663_chip *chip)
{
	debugfs_remove(max77663_dentry_regs);
}
#else
static inline void max77663_debugfs_init(struct max77663_chip *chip)
{
}

static inline void max77663_debugfs_exit(struct max77663_chip *chip)
{
}
#endif /* CONFIG_DEBUG_FS */

static int max77663_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max77663_platform_data *pdata = client->dev.platform_data;
	struct max77663_chip *chip;
	int ret = 0;

	if (pdata == NULL) {
		dev_err(&client->dev, "probe: Invalid platform_data\n");
		ret = -ENODEV;
		goto out;
	}

	chip = kzalloc(sizeof(struct max77663_chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "probe: kzalloc() failed\n");
		ret = -ENOMEM;
		goto out;
	}
	max77663_chip = chip;

	chip->i2c_power = client;
	i2c_set_clientdata(client, chip);

	chip->i2c_rtc = i2c_new_dummy(client->adapter, MAX77663_RTC_I2C_ADDR);
	i2c_set_clientdata(chip->i2c_rtc, chip);

	chip->dev = &client->dev;
	chip->pdata = pdata;
	chip->irq_base = pdata->irq_base;
	chip->gpio_base = pdata->gpio_base;
	mutex_init(&chip->io_lock);

	max77663_gpio_init(chip);
	max77663_irq_init(chip);
	max77663_debugfs_init(chip);
	ret = max77663_sleep(chip, false);
	if (ret < 0) {
		dev_err(&client->dev, "probe: Failed to disable sleep\n");
		goto out_exit;
	}

	ret = mfd_add_devices(&client->dev, 0, pdata->sub_devices,
			      pdata->num_subdevs, NULL, 0);
	if (ret != 0) {
		dev_err(&client->dev, "probe: Failed to add subdev: %d\n", ret);
		goto out_exit;
	}

	return 0;

out_exit:
	max77663_debugfs_exit(chip);
	max77663_gpio_exit(chip);
	max77663_irq_exit(chip);
	mutex_destroy(&chip->io_lock);
	max77663_chip = NULL;
	kfree(chip);
out:
	return ret;
}

static int __devexit max77663_remove(struct i2c_client *client)
{
	struct max77663_chip *chip = i2c_get_clientdata(client);

	mfd_remove_devices(chip->dev);
	max77663_debugfs_exit(chip);
	max77663_irq_exit(chip);
	max77663_gpio_exit(chip);
	mutex_destroy(&chip->io_lock);
	max77663_chip = NULL;
	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM
static int max77663_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77663_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (client->irq)
		disable_irq(client->irq);

	ret = max77663_sleep(chip, true);
	if (ret < 0)
		dev_err(dev, "suspend: Failed to enable sleep\n");

	return ret;
}

static int max77663_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77663_chip *chip = i2c_get_clientdata(client);
	int ret;

	ret = max77663_sleep(chip, false);
	if (ret < 0) {
		dev_err(dev, "resume: Failed to disable sleep\n");
		return ret;
	}

	if (client->irq)
		enable_irq(client->irq);

	return 0;
}
#else
#define max77663_suspend      NULL
#define max77663_resume       NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id max77663_id[] = {
	{"max77663", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max77663_id);

static const struct dev_pm_ops max77663_pm = {
	.suspend = max77663_suspend,
	.resume = max77663_resume,
};

static struct i2c_driver max77663_driver = {
	.driver = {
		.name = "max77663",
		.owner = THIS_MODULE,
		.pm = &max77663_pm,
	},
	.probe = max77663_probe,
	.remove = __devexit_p(max77663_remove),
	.id_table = max77663_id,
};

static int __init max77663_init(void)
{
	return i2c_add_driver(&max77663_driver);
}
subsys_initcall(max77663_init);

static void __exit max77663_exit(void)
{
	i2c_del_driver(&max77663_driver);
}
module_exit(max77663_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77663 Multi Function Device Core Driver");
MODULE_VERSION("1.0");
