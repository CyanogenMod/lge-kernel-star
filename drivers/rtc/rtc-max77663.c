/*
 * drivers/rtc/rtc-max77663.c
 * Max77663 RTC driver
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/mfd/max77663-core.h>

/* Registers */
#define MAX77663_RTC_IRQ		0x00
#define MAX77663_RTC_IRQ_MASK		0x01
#define MAX77663_RTC_CTRL_MODE		0x02
#define MAX77663_RTC_CTRL		0x03
#define MAX77663_RTC_UPDATE0		0x04
#define MAX77663_RTC_UPDATE1		0x05
#define MAX77663_RTC_SEC		0x07
#define MAX77663_RTC_MIN		0x08
#define MAX77663_RTC_HOUR		0x09
#define MAX77663_RTC_WEEKDAY		0x0A
#define MAX77663_RTC_MONTH		0x0B
#define MAX77663_RTC_YEAR		0x0C
#define MAX77663_RTC_MONTHDAY		0x0D
#define MAX77663_RTC_ALARM_SEC1		0x0E
#define MAX77663_RTC_ALARM_MIN1		0x0F
#define MAX77663_RTC_ALARM_HOUR1	0x10
#define MAX77663_RTC_ALARM_WEEKDAY1	0x11
#define MAX77663_RTC_ALARM_MONTH1	0x12
#define MAX77663_RTC_ALARM_YEAR1	0x13
#define MAX77663_RTC_ALARM_MONTHDAY1	0x14

#define RTC_IRQ_60SEC_MASK		(1 << 0)
#define RTC_IRQ_ALARM1_MASK		(1 << 1)
#define RTC_IRQ_ALARM2_MASK		(1 << 2)
#define RTC_IRQ_SMPL_MASK		(1 << 3)
#define RTC_IRQ_1SEC_MASK		(1 << 4)
#define RTC_IRQ_MASK			0x1F

#define BCD_MODE_MASK			(1 << 0)
#define HR_MODE_MASK			(1 << 1)

#define WB_UPDATE_MASK			(1 << 0)
#define FLAG_AUTO_CLEAR_MASK		(1 << 1)
#define FREEZE_SEC_MASK			(1 << 2)
#define RTC_WAKE_MASK			(1 << 3)
#define RB_UPDATE_MASK			(1 << 4)

#define WB_UPDATE_FLAG_MASK		(1 << 0)
#define RB_UPDATE_FLAG_MASK		(1 << 1)

#define SEC_MASK			0x7F
#define MIN_MASK			0x7F
#define HOUR_MASK			0x3F
#define WEEKDAY_MASK			0x7F
#define MONTH_MASK			0x1F
#define YEAR_MASK			0xFF
#define MONTHDAY_MASK			0x3F

#define ALARM_EN_MASK			0x80
#define ALARM_EN_SHIFT			7

#define RTC_UPDATE_RETRIES		20
#define RTC_YEAR_BASE			100
#define RTC_YEAR_MAX			99

enum {
	RTC_SEC,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_MONTH,
	RTC_YEAR,
	RTC_MONTHDAY,
	RTC_NR
};

struct max77663_rtc {
	struct rtc_device *rtc;
	struct device *dev;

	struct mutex io_lock;
	int irq;
	u8 irq_mask;
};

static inline struct device *_to_parent(struct max77663_rtc *rtc)
{
	return rtc->dev->parent;
}

static inline int max77663_rtc_update_buffer(struct max77663_rtc *rtc,
					     int write)
{
	struct device *parent = _to_parent(rtc);
	int retries = RTC_UPDATE_RETRIES;
	u8 val = FLAG_AUTO_CLEAR_MASK | RTC_WAKE_MASK | RB_UPDATE_MASK;
	u8 flag_mask = RB_UPDATE_FLAG_MASK;
	int ret;

	if (write) {
		val = FLAG_AUTO_CLEAR_MASK | RTC_WAKE_MASK | WB_UPDATE_MASK;
		flag_mask = WB_UPDATE_FLAG_MASK;
	}

	dev_dbg(rtc->dev, "rtc_update_buffer: write=%d, addr=0x%x, val=0x%x\n",
		write, MAX77663_RTC_UPDATE0, val);
	ret = max77663_write(parent, MAX77663_RTC_UPDATE0, &val, 1, 1);
	if (ret < 0) {
		dev_err(rtc->dev, "rtc_update_buffer: "
			"Failed to get rtc update0\n");
		return ret;
	}

	do {
		ret = max77663_read(parent, MAX77663_RTC_UPDATE1, &val, 1, 1);
		schedule_timeout_uninterruptible(msecs_to_jiffies(1));
	} while (!ret && retries-- && !(val & flag_mask));

	if (ret < 0) {
		dev_err(rtc->dev,
			"rtc_update_buffer: Failed to get rtc update1\n");
		return ret;
	}

	if (retries <= 0) {
		dev_err(rtc->dev, "rtc_update_buffer: "
			"Timeout waiting for buffer update\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int max77663_rtc_write(struct max77663_rtc *rtc, u8 addr,
				     void *values, u32 len, int update_buffer)
{
	struct device *parent = _to_parent(rtc);
	int ret;

	mutex_lock(&rtc->io_lock);

	dev_dbg(rtc->dev, "rtc_write: addr=0x%x, values=0x%x, len=%u, "
		"update_buffer=%d\n",
		addr, *((u8 *)values), len, update_buffer);
	ret = max77663_write(parent, addr, values, len, 1);
	if (ret < 0)
		goto out;

	if (update_buffer)
		ret = max77663_rtc_update_buffer(rtc, 1);

out:
	mutex_unlock(&rtc->io_lock);
	return ret;
}

static inline int max77663_rtc_read(struct max77663_rtc *rtc, u8 addr,
				    void *values, u32 len, int update_buffer)
{
	struct device *parent = _to_parent(rtc);
	int ret;

	mutex_lock(&rtc->io_lock);

	if (update_buffer) {
		ret = max77663_rtc_update_buffer(rtc, 0);
		if (ret < 0)
			goto out;
	}

	ret = max77663_read(parent, addr, values, len, 1);
	dev_dbg(rtc->dev, "rtc_read: addr=0x%x, values=0x%x, len=%u, "
		"update_buffer=%d\n",
		addr, *((u8 *)values), len, update_buffer);

out:
	mutex_unlock(&rtc->io_lock);
	return ret;
}

static inline int max77663_rtc_reg_to_tm(struct max77663_rtc *rtc, u8 *buf,
					 struct rtc_time *tm)
{
	int wday = buf[RTC_WEEKDAY] & WEEKDAY_MASK;

	if (unlikely(!wday)) {
		dev_err(rtc->dev,
			"rtc_reg_to_tm: Invalid day of week, %d\n", wday);
		return -EINVAL;
	}

	tm->tm_sec = (int)(buf[RTC_SEC] & SEC_MASK);
	tm->tm_min = (int)(buf[RTC_MIN] & MIN_MASK);
	tm->tm_hour = (int)(buf[RTC_HOUR] & HOUR_MASK);
	tm->tm_mday = (int)(buf[RTC_MONTHDAY] & MONTHDAY_MASK);
	tm->tm_mon = (int)(buf[RTC_MONTH] & MONTH_MASK) - 1;
	tm->tm_year = (int)(buf[RTC_YEAR] & YEAR_MASK) + RTC_YEAR_BASE;
	tm->tm_wday = ffs(wday);

	dev_dbg(rtc->dev, "rtc_reg_to_tm: buf sec=%u, min=%u, hour=%u, "
		"mday=%u, month=%u, year=%u, wday=%u\n",
		buf[RTC_SEC] & SEC_MASK, buf[RTC_MIN] & MIN_MASK,
		buf[RTC_HOUR] & HOUR_MASK, buf[RTC_MONTHDAY] & MONTHDAY_MASK,
		buf[RTC_MONTH] & MONTH_MASK, buf[RTC_YEAR] & YEAR_MASK,
		buf[RTC_WEEKDAY] & WEEKDAY_MASK);
	dev_dbg(rtc->dev, "rtc_reg_to_tm: tm_sec=%d, tm_min=%d, tm_hour=%d, "
		"tm_mday=%d, tm_mon=%d, tm_year=%d, tm_wday=%d\n",
		tm->tm_sec, tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon,
		tm->tm_year, tm->tm_wday);

	return 0;
}

static inline int max77663_rtc_tm_to_reg(struct max77663_rtc *rtc, u8 *buf,
					 struct rtc_time *tm, int alarm)
{
	u8 alarm_mask = alarm ? ALARM_EN_MASK : 0;

	if (unlikely((tm->tm_year < RTC_YEAR_BASE) ||
			(tm->tm_year > RTC_YEAR_BASE + RTC_YEAR_MAX))) {
		dev_err(rtc->dev,
			"rtc_tm_to_reg: Invalid year, %d\n", tm->tm_year);
		return -EINVAL;
	}

	buf[RTC_SEC] = tm->tm_sec | alarm_mask;
	buf[RTC_MIN] = tm->tm_min | alarm_mask;
	buf[RTC_HOUR] = tm->tm_hour | alarm_mask;
	buf[RTC_MONTHDAY] = tm->tm_mday | alarm_mask;
	buf[RTC_MONTH] = (tm->tm_mon + 1) | alarm_mask;
	buf[RTC_YEAR] = (tm->tm_year - RTC_YEAR_BASE) | alarm_mask;
	buf[RTC_WEEKDAY] = (1 << tm->tm_wday) | alarm_mask;

	dev_dbg(rtc->dev, "rtc_tm_to_reg: tm_sec=%d, tm_min=%d, tm_hour=%d, "
		"tm_mday=%d, tm_mon=%d, tm_year=%d, tm_wday=%d, alarm=%d\n",
		tm->tm_sec, tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon,
		tm->tm_year, tm->tm_wday, alarm);
	dev_dbg(rtc->dev, "rtc_tm_to_reg: buf sec=%u, min=%u, hour=%u, "
		"mday=%u, month=%u, year=%u, wday=%u\n",
		buf[RTC_SEC] & SEC_MASK, buf[RTC_MIN] & MIN_MASK,
		buf[RTC_HOUR] & HOUR_MASK, buf[RTC_MONTHDAY] & MONTHDAY_MASK,
		buf[RTC_MONTH] & MONTH_MASK, buf[RTC_YEAR] & YEAR_MASK,
		buf[RTC_WEEKDAY] & WEEKDAY_MASK);

	return 0;
}

static irqreturn_t max77663_rtc_irq(int irq, void *data)
{
	struct max77663_rtc *rtc = (struct max77663_rtc *)data;
	u8 val = 0;
	int ret;

	ret = max77663_rtc_read(rtc, MAX77663_RTC_IRQ, &val, 1, 0);
	if (ret < 0)
		dev_err(rtc->dev, "rtc_irq: Failed to get rtc irq status\n");

	if (val & RTC_IRQ_ALARM1_MASK)
		rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);

	if (val & RTC_IRQ_1SEC_MASK)
		rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_UF);

	return IRQ_HANDLED;
}

static int max77663_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	struct device *parent = _to_parent(rtc);
	u8 val = 0;
	int ret = 0;

	if (rtc->irq < 0)
		return -ENXIO;

	mutex_lock(&rtc->io_lock);
	if (enabled)
		val = rtc->irq_mask & ~RTC_IRQ_ALARM1_MASK;
	else
		val = rtc->irq_mask | RTC_IRQ_ALARM1_MASK;

	if (val != rtc->irq_mask) {
		ret = max77663_write(parent, MAX77663_RTC_IRQ_MASK, &val, 1, 1);
		if (ret < 0) {
			dev_err(rtc->dev, "rtc_alarm_irq_enable: "
				"Failed to set rtc irq mask\n");
			mutex_unlock(&rtc->io_lock);
			return ret;
		}

		rtc->irq_mask = val;
	}
	mutex_unlock(&rtc->io_lock);

	return 0;
}

static int max77663_rtc_update_irq_enable(struct device *dev,
					  unsigned int enabled)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	struct device *parent = _to_parent(rtc);
	u8 val = 0;
	int ret = 0;

	if (rtc->irq < 0)
		return -ENXIO;

	mutex_lock(&rtc->io_lock);
	if (enabled)
		val = rtc->irq_mask & ~RTC_IRQ_1SEC_MASK;
	else
		val = rtc->irq_mask | RTC_IRQ_1SEC_MASK;

	if (val != rtc->irq_mask) {
		ret = max77663_write(parent, MAX77663_RTC_IRQ_MASK, &val, 1, 1);
		if (ret < 0) {
			dev_err(rtc->dev, "rtc_update_irq_enable: "
				"Failed to set rtc irq mask\n");
			mutex_unlock(&rtc->io_lock);
			return ret;
		}

		rtc->irq_mask = val;
	}
	mutex_unlock(&rtc->io_lock);

	return 0;
}

static int max77663_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	u8 buf[RTC_NR];
	int ret;

	ret = max77663_rtc_read(rtc, MAX77663_RTC_SEC, buf, sizeof(buf), 1);
	if (ret < 0) {
		dev_err(rtc->dev, "rtc_read_time: Failed to read rtc time\n");
		return ret;
	}

	return max77663_rtc_reg_to_tm(rtc, buf, tm);
}

static int max77663_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	u8 buf[RTC_NR];
	int ret;

	ret = max77663_rtc_tm_to_reg(rtc, buf, tm, 0);
	if (ret < 0) {
		dev_err(rtc->dev, "rtc_set_time: "
			"Failed to convert time format into register format\n");
		return ret;
	}

	return max77663_rtc_write(rtc, MAX77663_RTC_SEC, buf, sizeof(buf), 1);
}

static int max77663_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	u8 buf[RTC_NR];
	int ret;

	ret = max77663_rtc_read(rtc, MAX77663_RTC_ALARM_SEC1, buf, sizeof(buf),
				1);
	if (ret < 0) {
		dev_err(rtc->dev,
			"rtc_read_alarm: Failed to read rtc alarm time\n");
		return ret;
	}

	ret = max77663_rtc_reg_to_tm(rtc, buf, &alrm->time);
	if (ret < 0) {
		dev_err(rtc->dev, "rtc_read_alarm: "
			"Failed to convert register format into time format\n");
		return ret;
	}

	if (rtc->irq_mask & RTC_IRQ_ALARM1_MASK)
		alrm->enabled = 1;
	else
		alrm->enabled = 0;

	return 0;
}

static int max77663_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77663_rtc *rtc = dev_get_drvdata(dev);
	u8 buf[RTC_NR];
	int ret;

	ret = max77663_rtc_tm_to_reg(rtc, buf, &alrm->time, 1);
	if (ret < 0) {
		dev_err(rtc->dev, "rtc_set_alarm: "
			"Failed to convert time format into register format\n");
		return ret;
	}

	ret = max77663_rtc_write(rtc, MAX77663_RTC_ALARM_SEC1, buf, sizeof(buf),
				 1);
	if (ret < 0) {
		dev_err(rtc->dev,
			"rtc_set_alarm: Failed to write rtc alarm time\n");
		return ret;
	}

	ret = max77663_rtc_alarm_irq_enable(dev, alrm->enabled);
	if (ret < 0) {
		dev_err(rtc->dev,
			"rtc_set_alarm: Failed to set rtc alarm irq\n");
		return ret;
	}

	return 0;
}

static const struct rtc_class_ops max77663_rtc_ops = {
	.read_time = max77663_rtc_read_time,
	.set_time = max77663_rtc_set_time,
	.read_alarm = max77663_rtc_read_alarm,
	.set_alarm = max77663_rtc_set_alarm,
	.alarm_irq_enable = max77663_rtc_alarm_irq_enable,
	.update_irq_enable = max77663_rtc_update_irq_enable,
};

static int max77663_rtc_preinit(struct max77663_rtc *rtc)
{
	u8 val;
	int ret;

	/* Mask all interrupts */
	rtc->irq_mask = RTC_IRQ_MASK;
	ret = max77663_rtc_write(rtc, MAX77663_RTC_IRQ_MASK, &rtc->irq_mask, 1,
				 0);
	if (ret < 0) {
		dev_err(rtc->dev, "preinit: Failed to set rtc irq mask\n");
		return ret;
	}

	/* Configure Binary mode and 24hour mode */
	val = HR_MODE_MASK;
	ret = max77663_rtc_write(rtc, MAX77663_RTC_CTRL, &val, 1, 0);
	if (ret < 0) {
		dev_err(rtc->dev, "preinit: Failed to set rtc control\n");
		return ret;
	}

	return 0;
}

static int max77663_rtc_probe(struct platform_device *pdev)
{
	struct max77663_platform_data *parent_pdata =
						pdev->dev.parent->platform_data;
	static struct max77663_rtc *rtc;
	int ret = 0;

	rtc = kzalloc(sizeof(struct max77663_rtc), GFP_KERNEL);
	if (!rtc) {
		dev_err(&pdev->dev, "probe: kzalloc() failed\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, rtc);
	rtc->dev = &pdev->dev;
	mutex_init(&rtc->io_lock);

	ret = max77663_rtc_preinit(rtc);
	if (ret) {
		dev_err(&pdev->dev, "probe: Failed to rtc preinit\n");
		goto out_kfree;
	}

	rtc->rtc = rtc_device_register("max77663-rtc", &pdev->dev,
				       &max77663_rtc_ops, THIS_MODULE);
	if (IS_ERR_OR_NULL(rtc->rtc)) {
		dev_err(&pdev->dev, "probe: Failed to register rtc\n");
		ret = PTR_ERR(rtc->rtc);
		goto out_kfree;
	}

	if (parent_pdata->irq_base < 0)
		goto out;

	rtc->irq = parent_pdata->irq_base + MAX77663_IRQ_RTC;
	ret = request_threaded_irq(rtc->irq, NULL, max77663_rtc_irq,
				   IRQF_ONESHOT, "max77663-rtc", rtc);
	if (ret < 0) {
		dev_err(rtc->dev, "probe: Failed to request irq %d\n",
			rtc->irq);
		rtc->irq = -1;
	} else {
		device_init_wakeup(rtc->dev, 1);
		enable_irq_wake(rtc->irq);
	}

	return 0;

out_kfree:
	mutex_destroy(&rtc->io_lock);
	kfree(rtc->rtc);
out:
	return ret;
}

static int __devexit max77663_rtc_remove(struct platform_device *pdev)
{
	struct max77663_rtc *rtc = dev_get_drvdata(&pdev->dev);

	if (rtc->irq != -1)
		free_irq(rtc->irq, rtc);

	rtc_device_unregister(rtc->rtc);
	mutex_destroy(&rtc->io_lock);
	kfree(rtc);

	return 0;
}

static struct platform_driver max77663_rtc_driver = {
	.probe = max77663_rtc_probe,
	.remove = __devexit_p(max77663_rtc_remove),
	.driver = {
		   .name = "max77663-rtc",
		   .owner = THIS_MODULE,
	},
};

static int __init max77663_rtc_init(void)
{
	return platform_driver_register(&max77663_rtc_driver);
}
module_init(max77663_rtc_init);

static void __exit max77663_rtc_exit(void)
{
	platform_driver_unregister(&max77663_rtc_driver);
}
module_exit(max77663_rtc_exit);

MODULE_DESCRIPTION("max77663 RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
