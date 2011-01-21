/*
 * Battery driver for Maxim MAX8907C
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 * Based on driver/mfd/max8925-core.c, Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max8907c.h>

struct max8907c_irq_data {
	int	reg;
	int	mask_reg;
	int	enable;		/* enable or not */
	int	offs;		/* bit offset in mask register */
	bool	is_rtc;
};

static struct max8907c_irq_data max8907c_irqs[] = {
	[MAX8907C_IRQ_VCHG_DC_OVP] = {
		.reg		= MAX8907C_REG_CHG_IRQ1,
		.mask_reg	= MAX8907C_REG_CHG_IRQ1_MASK,
		.offs		= 1 << 0,
	},
	[MAX8907C_IRQ_VCHG_DC_F] = {
		.reg		= MAX8907C_REG_CHG_IRQ1,
		.mask_reg	= MAX8907C_REG_CHG_IRQ1_MASK,
		.offs		= 1 << 1,
	},
	[MAX8907C_IRQ_VCHG_DC_R] = {
		.reg		= MAX8907C_REG_CHG_IRQ1,
		.mask_reg	= MAX8907C_REG_CHG_IRQ1_MASK,
		.offs		= 1 << 2,
	},
	[MAX8907C_IRQ_VCHG_THM_OK_R] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 0,
	},
	[MAX8907C_IRQ_VCHG_THM_OK_F] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 1,
	},
	[MAX8907C_IRQ_VCHG_MBATTLOW_F] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 2,
	},
	[MAX8907C_IRQ_VCHG_MBATTLOW_R] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 3,
	},
	[MAX8907C_IRQ_VCHG_RST] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 4,
	},
	[MAX8907C_IRQ_VCHG_DONE] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 5,
	},
	[MAX8907C_IRQ_VCHG_TOPOFF] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 6,
	},
	[MAX8907C_IRQ_VCHG_TMR_FAULT] = {
		.reg		= MAX8907C_REG_CHG_IRQ2,
		.mask_reg	= MAX8907C_REG_CHG_IRQ2_MASK,
		.offs		= 1 << 7,
	},
	[MAX8907C_IRQ_GPM_RSTIN] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 0,
	},
	[MAX8907C_IRQ_GPM_MPL] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 1,
	},
	[MAX8907C_IRQ_GPM_SW_3SEC] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 2,
	},
	[MAX8907C_IRQ_GPM_EXTON_F] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 3,
	},
	[MAX8907C_IRQ_GPM_EXTON_R] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 4,
	},
	[MAX8907C_IRQ_GPM_SW_1SEC] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 5,
	},
	[MAX8907C_IRQ_GPM_SW_F] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 6,
	},
	[MAX8907C_IRQ_GPM_SW_R] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ1,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ1_MASK,
		.offs		= 1 << 7,
	},
	[MAX8907C_IRQ_GPM_SYSCKEN_F] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ2,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ2_MASK,
		.offs		= 1 << 0,
	},
	[MAX8907C_IRQ_GPM_SYSCKEN_R] = {
		.reg		= MAX8907C_REG_ON_OFF_IRQ2,
		.mask_reg	= MAX8907C_REG_ON_OFF_IRQ2_MASK,
		.offs		= 1 << 1,
	},
	[MAX8907C_IRQ_RTC_ALARM1] = {
		.reg		= MAX8907C_REG_RTC_IRQ,
		.mask_reg	= MAX8907C_REG_RTC_IRQ_MASK,
		.offs		= 1 << 2,
		.is_rtc		= true,
	},
	[MAX8907C_IRQ_RTC_ALARM0] = {
		.reg		= MAX8907C_REG_RTC_IRQ,
		.mask_reg	= MAX8907C_REG_RTC_IRQ_MASK,
		.offs		= 1 << 3,
		.is_rtc		= true,
	},
};

static inline struct max8907c_irq_data *irq_to_max8907c(struct max8907c *chip,
						      int irq)
{
	return &max8907c_irqs[irq - chip->irq_base];
}

static irqreturn_t max8907c_irq(int irq, void *data)
{
	struct max8907c *chip = data;
	struct max8907c_irq_data *irq_data;
	struct i2c_client *i2c;
	int read_reg = -1, value = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(max8907c_irqs); i++) {
		irq_data = &max8907c_irqs[i];

		if (irq_data->is_rtc)
			i2c = chip->i2c_rtc;
		else
			i2c = chip->i2c_power;

		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = max8907c_reg_read(i2c, irq_data->reg);
		}

		if (value & irq_data->enable)
			handle_nested_irq(chip->irq_base + i);
	}
	return IRQ_HANDLED;
}

static void max8907c_irq_lock(unsigned int irq)
{
	struct max8907c *chip = get_irq_chip_data(irq);

	mutex_lock(&chip->irq_lock);
}

static void max8907c_irq_sync_unlock(unsigned int irq)
{
	struct max8907c *chip = get_irq_chip_data(irq);
	struct max8907c_irq_data *irq_data;
	static unsigned char cache_chg[2] = {0xff, 0xff};
	static unsigned char cache_on[2] = {0xff, 0xff};
	static unsigned char cache_rtc = 0xff;
	unsigned char irq_chg[2], irq_on[2];
	unsigned char irq_rtc;
	int i;

	/* Load cached value. In initial, all IRQs are masked */
	irq_chg[0] = cache_chg[0];
	irq_chg[1] = cache_chg[1];
	irq_on[0] = cache_on[0];
	irq_on[1] = cache_on[1];
	irq_rtc = cache_rtc;
	for (i = 0; i < ARRAY_SIZE(max8907c_irqs); i++) {
		irq_data = &max8907c_irqs[i];
		/* 1 -- disable, 0 -- enable */
		switch (irq_data->mask_reg) {
		case MAX8907C_REG_CHG_IRQ1_MASK:
			irq_chg[0] &= ~irq_data->enable;
			break;
		case MAX8907C_REG_CHG_IRQ2_MASK:
			irq_chg[1] &= ~irq_data->enable;
			break;
		case MAX8907C_REG_ON_OFF_IRQ1_MASK:
			irq_on[0] &= ~irq_data->enable;
			break;
		case MAX8907C_REG_ON_OFF_IRQ2_MASK:
			irq_on[1] &= ~irq_data->enable;
			break;
		case MAX8907C_REG_RTC_IRQ_MASK:
			irq_rtc &= ~irq_data->enable;
			break;
		default:
			dev_err(chip->dev, "wrong IRQ\n");
			break;
		}
	}
	/* update mask into registers */
	if (cache_chg[0] != irq_chg[0]) {
		cache_chg[0] = irq_chg[0];
		max8907c_reg_write(chip->i2c_power, MAX8907C_REG_CHG_IRQ1_MASK,
			irq_chg[0]);
	}
	if (cache_chg[1] != irq_chg[1]) {
		cache_chg[1] = irq_chg[1];
		max8907c_reg_write(chip->i2c_power, MAX8907C_REG_CHG_IRQ2_MASK,
			irq_chg[1]);
	}
	if (cache_on[0] != irq_on[0]) {
		cache_on[0] = irq_on[0];
		max8907c_reg_write(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ1_MASK,
				irq_on[0]);
	}
	if (cache_on[1] != irq_on[1]) {
		cache_on[1] = irq_on[1];
		max8907c_reg_write(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ2_MASK,
				irq_on[1]);
	}
	if (cache_rtc != irq_rtc) {
		cache_rtc = irq_rtc;
		max8907c_reg_write(chip->i2c_rtc, MAX8907C_REG_RTC_IRQ_MASK,
				   irq_rtc);
	}

	mutex_unlock(&chip->irq_lock);
}

static void max8907c_irq_enable(unsigned int irq)
{
	struct max8907c *chip = get_irq_chip_data(irq);
	max8907c_irqs[irq - chip->irq_base].enable
		= max8907c_irqs[irq - chip->irq_base].offs;
}

static void max8907c_irq_disable(unsigned int irq)
{
	struct max8907c *chip = get_irq_chip_data(irq);
	max8907c_irqs[irq - chip->irq_base].enable = 0;
}

static struct irq_chip max8907c_irq_chip = {
	.name		= "max8907c",
	.bus_lock	= max8907c_irq_lock,
	.bus_sync_unlock = max8907c_irq_sync_unlock,
	.enable		= max8907c_irq_enable,
	.disable	= max8907c_irq_disable,
};

int max8907c_irq_init(struct max8907c *chip, int irq, int irq_base)
{
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	struct irq_desc *desc;
	int i, ret;
	int __irq;

	if (!irq_base || !irq) {
		dev_warn(chip->dev, "No interrupt support\n");
		return -EINVAL;
	}
	/* clear all interrupts */
	max8907c_reg_read(chip->i2c_power, MAX8907C_REG_CHG_IRQ1);
	max8907c_reg_read(chip->i2c_power, MAX8907C_REG_CHG_IRQ2);
	max8907c_reg_read(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ1);
	max8907c_reg_read(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ2);
	max8907c_reg_read(chip->i2c_rtc, MAX8907C_REG_RTC_IRQ);
	/* mask all interrupts */
	max8907c_reg_write(chip->i2c_rtc, MAX8907C_REG_ALARM0_CNTL, 0);
	max8907c_reg_write(chip->i2c_rtc, MAX8907C_REG_ALARM1_CNTL, 0);
	max8907c_reg_write(chip->i2c_power, MAX8907C_REG_CHG_IRQ1_MASK, 0xff);
	max8907c_reg_write(chip->i2c_power, MAX8907C_REG_CHG_IRQ2_MASK, 0xff);
	max8907c_reg_write(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ1_MASK, 0xff);
	max8907c_reg_write(chip->i2c_power, MAX8907C_REG_ON_OFF_IRQ2_MASK, 0xff);
	max8907c_reg_write(chip->i2c_rtc, MAX8907C_REG_RTC_IRQ_MASK, 0xff);

	mutex_init(&chip->irq_lock);
	chip->core_irq = irq;
	chip->irq_base = irq_base;
	desc = irq_to_desc(chip->core_irq);

	/* register with genirq */
	for (i = 0; i < ARRAY_SIZE(max8907c_irqs); i++) {
		__irq = i + chip->irq_base;
		set_irq_chip_data(__irq, chip);
		set_irq_chip_and_handler(__irq, &max8907c_irq_chip,
					 handle_edge_irq);
		set_irq_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#else
		set_irq_noprobe(__irq);
#endif
	}

	ret = request_threaded_irq(irq, NULL, max8907c_irq, flags,
				   "max8907c", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request core IRQ: %d\n", ret);
		chip->core_irq = 0;
	}

	return ret;
}

int max8907c_suspend(struct i2c_client *i2c, pm_message_t state)
{
	struct max8907c *max8907c = i2c_get_clientdata(i2c);

	disable_irq(max8907c->core_irq);

	return 0;
}

int max8907c_resume(struct i2c_client *i2c)
{
	struct max8907c *max8907c = i2c_get_clientdata(i2c);

	enable_irq(max8907c->core_irq);

	return 0;
}

void max8907c_irq_free(struct max8907c *chip)
{
	if (chip->core_irq)
		free_irq(chip->core_irq, chip);
}

