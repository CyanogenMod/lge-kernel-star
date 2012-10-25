/*
 * RTC driver for Maxim MAX8907c
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/max8907c.h>
#include <linux/mfd/max8907_reg.h>
#include <linux/mfd/max8907_adc.h>

/* Inner Defines */
#define MAX8907_SECONDS_PER_DAY    (60*60*24)
#define MAX8907_SECONDS_PER_HOUR   (60*60)
#define MAX8907_SECONDS_PER_MINUTE (60)

#define LINUX_RTC_BASE_YEAR 1900

/* Macro for conversion of BCD number to decimal format */
#define BCD_TO_DECIMAL(BCD) \
	((((BCD) & 0xF0) >> 4) * 10 + ((BCD) & 0xF))
/* Macro for conversion of decimal number to BCD format */
#define DECIMAL_TO_BCD(DEC) \
	((((DEC) / 10) << 4) | ((DEC) % 10))

/* Device Structure */
struct max8907c_rtc_info {
	struct i2c_client	*i2c_power;
	struct i2c_client	*i2c_rtc;
	struct max8907c		*chip;
};

/* Static Inner Client */
static struct i2c_client *max8907c_i2c_rtc_client = NULL;
static struct i2c_client *max8907c_i2c_power_client = NULL;

/* Mutexes */
struct mutex rtc_en_lock;
struct mutex rtc_en_temp_lock;

/* Utulity Functions */
static enum {
	RTC_SEC = 0,
        RTC_MIN,
        RTC_HOUR,
        RTC_WEEKDAY,
        RTC_DATE,
        RTC_MONTH,
        RTC_YEAR1,
        RTC_YEAR2,
};
#define	BUF_LENGTH		8
#define ALARM_1SEC		(1 << 7)
#define HOUR_12			(1 << 7)
#define HOUR_AM_PM		(1 << 5)
#define ALARM0_IRQ		(1 << 3)
#define ALARM1_IRQ		(1 << 2)
#define ALARM0_STATUS		(1 << 2)
#define ALARM1_STATUS		(1 << 1)
static int tm_calc(struct rtc_time *tm, unsigned char *buf, int len)
{
        if (len < BUF_LENGTH)
                return -EINVAL;

        tm->tm_year = (buf[RTC_YEAR2] >> 4) * 1000
                        + (buf[RTC_YEAR2] & 0xf) * 100
                        + (buf[RTC_YEAR1] >> 4) * 10
                        + (buf[RTC_YEAR1] & 0xf);
        tm->tm_mon = ((buf[RTC_MONTH] >> 4) & 0x01) * 10
                        + (buf[RTC_MONTH] & 0x0f);
        tm->tm_mday = ((buf[RTC_DATE] >> 4) & 0x03) * 10
                        + (buf[RTC_DATE] & 0x0f);
	tm->tm_wday = buf[RTC_WEEKDAY] & 0x07;
	if (buf[RTC_HOUR] & HOUR_12) {
		tm->tm_hour = ((buf[RTC_HOUR] >> 4) & 0x1) * 10
				+ (buf[RTC_HOUR] & 0x0f);
		if (buf[RTC_HOUR] & HOUR_AM_PM)
			tm->tm_hour += 12;
	} else {
		tm->tm_hour = ((buf[RTC_HOUR] >> 4) & 0x03) * 10
				+ (buf[RTC_HOUR] & 0x0f);
	}
        tm->tm_min = ((buf[RTC_MIN] >> 4) & 0x7) * 10
                        + (buf[RTC_MIN] & 0x0f);
        tm->tm_sec = ((buf[RTC_SEC] >> 4) & 0x7) * 10
                        + (buf[RTC_SEC] & 0x0f);
        return 1;
}
static int data_calc(unsigned char *buf, struct rtc_time *tm, int len)
{
        unsigned char high, low;

        if (len < BUF_LENGTH)
                return -EINVAL;

        high = (tm->tm_year + 1900) / 1000;
        low = (tm->tm_year + 1900) / 100;
        low = low - high * 10;
        buf[RTC_YEAR2] = (high << 4) + low;
        high = (tm->tm_year + 1900) / 10;
        low = tm->tm_year + 1900;
        low = low - high * 10;
        high = high - (high / 10) * 10;
        buf[RTC_YEAR1] = (high << 4) + low;
        high = tm->tm_mon / 10;
        low = tm->tm_mon;
        low = low - high * 10;
        buf[RTC_MONTH] = (high << 4) + low;
        high = tm->tm_mday / 10;
        low = tm->tm_mday;
        low = low - high * 10;
        buf[RTC_DATE] = (high << 4) + low;
        buf[RTC_WEEKDAY] = tm->tm_wday;
        high = tm->tm_hour / 10;
        low = tm->tm_hour;
        low = low - high * 10;
        buf[RTC_HOUR] = (high << 4) + low;
        high = tm->tm_min / 10;
        low = tm->tm_min;
        low = low - high * 10;
        buf[RTC_MIN] = (high << 4) + low;
        high = tm->tm_sec / 10;
        low = tm->tm_sec;
        low = low - high * 10;
        buf[RTC_SEC] = (high << 4) + low;
        return 1;
}
/* Export Functions */
int max8907c_rtc_count_read(unsigned int *count)
{
	struct rtc_time tm;
	unsigned char buf[BUF_LENGTH];
	int ret;

	ret = max8907c_reg_bulk_read(max8907c_i2c_rtc_client, 
					MAX8907C_REG_RTC_SEC, BUF_LENGTH, buf);
	if(ret < 0) 
		return ret;

	/* Calculate */
	ret = tm_calc(&tm, buf, BUF_LENGTH);
	if(ret < 0)
		return ret;

	/* Make Time */
	pr_info("Time [%04d][%02d][%02d][%02d][%02d][%02d]\n", 
			tm.tm_year, tm.tm_mon, tm.tm_mday, 
			tm.tm_hour, tm.tm_min, tm.tm_sec);
	*count = mktime(tm.tm_year, tm.tm_mon, tm.tm_mday, 
			tm.tm_hour, tm.tm_min, tm.tm_sec);
	
	return *count;
}
EXPORT_SYMBOL(max8907c_rtc_count_read);

int max8907c_rtc_alarm_count_read(unsigned int *count) 
{
	struct rtc_wkalrm alrm;
        unsigned char buf[BUF_LENGTH];
        int ret;

	ret = max8907c_reg_bulk_read(max8907c_i2c_rtc_client, 
					MAX8907C_REG_ALARM0_SEC, BUF_LENGTH, buf);
        if (ret < 0)
                return ret;

	/* Calculate */
        ret = tm_calc(&alrm.time, buf, BUF_LENGTH);
        if (ret < 0)
                return ret;

	/* Read IRQ */
        ret = max8907c_reg_read(max8907c_i2c_rtc_client, 
				MAX8907C_REG_RTC_IRQ_MASK);
        if (ret < 0)
                return ret;

	/* Set Other Variables */
        if ((ret & ALARM0_IRQ) == 0)
                alrm.enabled = 1;
        else
                alrm.enabled = 0;
        ret = max8907c_reg_read(max8907c_i2c_rtc_client,
				MAX8907C_REG_RTC_STATUS);
        if (ret < 0)
                return ret;
        if (ret & ALARM0_STATUS)
                alrm.pending = 1;
        else
                alrm.pending = 0;

        /* Make Time */
	pr_info("Time [%04d][%02d][%02d][%02d][%02d][%02d]\n", 
			alrm.time.tm_year, alrm.time.tm_mon, alrm.time.tm_mday, 
			alrm.time.tm_hour, alrm.time.tm_min, alrm.time.tm_sec);
        *count = mktime(alrm.time.tm_year, alrm.time.tm_mon, alrm.time.tm_mday,
                        alrm.time.tm_hour, alrm.time.tm_min, alrm.time.tm_sec);

        return *count;
}
EXPORT_SYMBOL(max8907c_rtc_alarm_count_read);

int max8907c_rtc_count_write(unsigned int count) 
{
	struct rtc_time tm;
        unsigned char buf[BUF_LENGTH];
        int ret;

	/* Count to tm */
	rtc_time_to_tm(count, &tm);

	/* Fill Buffer */
	ret = data_calc(buf, &tm, BUF_LENGTH);
        if (ret < 0)
                return ret;

        ret = max8907c_reg_bulk_write(max8907c_i2c_rtc_client, 
					MAX8907C_REG_RTC_SEC, BUF_LENGTH, buf);
	if(ret < 0)
		return ret;

        return 1;
}
EXPORT_SYMBOL(max8907c_rtc_count_write);

int max8907c_rtc_alarm_write(unsigned int count) 
{
	struct rtc_time tm;
        unsigned char buf[BUF_LENGTH];
        int ret;

	/* Count to tm */
	rtc_time_to_tm(count, &tm);

	/* Fill Buffer */
	ret = data_calc(buf, &tm, BUF_LENGTH);
        if (ret < 0)
                return ret;

        ret = max8907c_reg_bulk_write(max8907c_i2c_rtc_client, 
					MAX8907C_REG_ALARM0_SEC, BUF_LENGTH, buf);
        if (ret < 0)
                return ret;

        /* only enable alarm on year/month/day/hour/min/sec */
        ret = max8907c_reg_write(max8907c_i2c_rtc_client, 
				 MAX8907C_REG_ALARM0_CNTL, 0x77);

        return 1;
}
EXPORT_SYMBOL(max8907c_rtc_alarm_write);

/* Device Probing & Removing Code */
static int __devinit max8907c_rtc_probe(struct platform_device *pdev)
{
	struct max8907c *chip = dev_get_drvdata(pdev->dev.parent);
	struct max8907c_rtc_info *info;
	pr_info("MAX8907C_rtc_probe\n");

	info = kzalloc(sizeof(struct max8907c_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->i2c_rtc	= chip->i2c_rtc;
	info->i2c_power = chip->i2c_power;
	info->chip = chip;

	dev_set_drvdata(&pdev->dev, info);

	platform_set_drvdata(pdev, info);
	max8907c_i2c_rtc_client   = info->i2c_rtc;
	max8907c_i2c_power_client = info->i2c_power;

	mutex_init(&rtc_en_lock);
	if (!max8907c_i2c_rtc_client){
		pr_info("MAX8907C_rtc_probe adc_en_lock\n");
		return -EINVAL;
	}

	mutex_init(&rtc_en_temp_lock);
	if (!max8907c_i2c_rtc_client){
		pr_info("MAX8907C_adc_probe adc_en_temp_lock\n");
		return -EINVAL;
	}

	pr_info("MAX8907C_rtc_probe ok !!!!\n");
	return 0;
}
static int __devexit max8907c_rtc_remove(struct platform_device *pdev)
{
	struct max8907c_rtc_info *info = platform_get_drvdata(pdev);

	if (info)
		kfree(info);
	return 0;
}

/* RTC Init Structure */
static struct platform_driver max8907c_rtc_driver = {
	.driver		= {
		.name	= "max8907c-rtc",
		.owner	= THIS_MODULE,
	},
	.probe		= max8907c_rtc_probe,
	.remove		= __devexit_p(max8907c_rtc_remove),
};
static int __init max8907c_rtc_init(void)
{
	return platform_driver_register(&max8907c_rtc_driver);

}
static void __exit max8907c_rtc_exit(void)
{
	platform_driver_unregister(&max8907c_rtc_driver);
}
subsys_initcall(max8907c_rtc_init);
module_exit(max8907c_rtc_exit);

MODULE_DESCRIPTION("Maxim MAX8907C RTC driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("sanghyun.hong@lge.com");
