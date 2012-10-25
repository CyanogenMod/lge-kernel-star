/*
 * linux/drivers/video/backlight/bd6084_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/bd6084_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>  // 20110726 sangki.hyun@lge.com backlight wakelock 

#if defined (CONFIG_LU6500) 
	#define HITACH_PANEL	0
	#define LGD_PANEL		1
#else
	#define HITACH_PANEL	1
	#define LGD_PANEL		0
#endif

/*
 * Debug Level
 *	2 : Print all debug messages
 *	1 : Print only dbg() messages
 *	0 : No debug messages
 */
#define BD6084_BL_DEBUG_LEVEL	2

/* 아래 define을 살리면 adb 로 값을 write 하여 ALC 관련 튜닝 가능 */
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
#define ALC_TEST_CONFIG  // 20110623 sangki.hyun@lge.com alc config
#else
//#define ALC_TEST_CONFIG  // 20110623 sangki.hyun@lge.com alc config
#endif

#if (BD6084_BL_DEBUG_LEVEL == 2)
#define dbg(format, arg...) \
	printk(KERN_ALERT BD6084_BL_DRV_NAME \
	       ": Debug: %s(): " format, __func__, ## arg)
#define enter() \
	printk(KERN_ALERT BD6084_BL_DRV_NAME ": Enter: %s()\n", __func__)
#define leave() \
	printk(KERN_ALERT BD6084_BL_DRV_NAME ": Leave: %s()\n", __func__)
#elif (BD6084_BL_DEBUG_LEVEL == 1)
#define dbg(format, arg...) \
	printk(KERN_ALERT BD6084_BL_DRV_NAME \
	       ": Debug: %s(): " format, __func__, ## arg)
#define enter()
#define leave()
#else
#define dbg(format, arg...)
#define enter()
#define leave()
#endif

// 20110622 deukgi.shin@lge.com Setting alc mode option [start]
#define MANUAL_MODE_BRIGHTNESS          -1
#define ALC_OPTION_OPTIMUM_BRIGHTNESS    1
#define ALC_OPTION_POWER_SAVE_BRIGHTNESS 0
// 20110623 sangki.hyun@lge.com alc config [S] {

#ifdef ALC_TEST_CONFIG
#define ALC_MODE_CONFIG_TEST    2
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }

static struct wake_lock bd_wake_lock;  // 20110726 sangki.hyun@lge.com backlight wakelock 

// 20110622 deukgi.shin@lge.com Setting alc mode option [end]

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bd6084_bl_early_suspend(struct early_suspend *h);
static void bd6084_bl_late_resume(struct early_suspend *h);
#endif

static int bd6084_bl_get_brightness(struct backlight_device *bd);
static int bd6084_bl_update_status(struct backlight_device *bd);

static int set_auto_brightness = 0;
static int old_auto_brightness = -1;
static int set_alc_option =  -1;

struct bd6084_bl_driver_data *bd6084_bl_drvdata = NULL;

// BD6084 register address and data
struct bd6084_bl_command {
	u8	addr;
	u8	data;
};

static DEFINE_MUTEX(bl_mutex);

// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
int b_reg_value = 0x04;  // alc_slope
int d_reg_value = 0x0C;  // min
int e_reg_value = 0x37;  // max
#else
int b_reg_value = 0x00;  // alc_slope
int d_reg_value = 0x00;  // min
int e_reg_value = 0x00;  // max
#endif
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }

static bool is_suspended = false;
#if 0
// 20110712 youngjin.yoo@lge.com For LP1 state [S]
extern int in_call_state(void);

bool bd6084_bl_is_LP1(void)
{
	if (in_call_state()) {
		if (is_suspended) {
			return true;
		}
	}

	return false;
}
EXPORT_SYMBOL(bd6084_bl_is_LP1);
// 20110712 youngjin.yoo@lge.com For LP1 state [S]
#endif

// 20110526 bg80.song@lge.com Max LED Current = 20mA, K1001 H/W Team Request [S]
static int bd6084_bl_adjust_brightness(int brightness) //input range 0 ~ 255  
{
           int val;
#if !defined (CONFIG_KS1103) //20120530 kim.youngmin@lge.com (20~255 : 7~26~99)
           int rate1 = 76;              //0.76              30~99  (99/127)*10000 compiler operation error
           int rate2 = 369;             // 0.369            100 ~230  
           int rate3 = 295;
           
           if(brightness < 100)
                     val = ((brightness/2) * rate1) / 100;
           else if(brightness < 230)
                     val = (((brightness/2) * rate2) / 1000)+20;
           else 
                     val = (((brightness/2) * rate3) / 100)-275;
#else
           int rate1 = 24;
           int rate2 = 28;
           int rate3 = 142;
           
           if(brightness < 100)
                     val = ((brightness * rate1) / 100)+2;
           else if(brightness < 230)
                     val = ((brightness * rate2) / 100)-2;
           else 
                     val = ((brightness * rate3) / 100)-263;
#endif
           printk("[SG] brightness = %d, val = %d\n", brightness, val);
           
           return val;
}
// 20110536 bg80.song@lge.com Max LED Current = 20mA, K1001 H/W Team Request [E]

static int bd6084_bl_write(struct backlight_device *bd, int addr, int data)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	struct i2c_client *client = drvdata->client;

	char msg[2];
	int ret = 0;

	msg[0] = (char)addr;
	msg[1] = (char)data;

	ret = i2c_master_send(client, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "Failed to i2c master send, %d\n", ret);
		goto out;
	}

	drvdata->reg_cache[addr] = data;
	ret = 0;
out:
	return ret;
}

static int bd6084_bl_read(struct backlight_device *bd, int addr)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);

	return drvdata->reg_cache[addr];
}

static int bd6084_bl_read_reg(struct i2c_client *client, unsigned char index)
{
	return i2c_smbus_read_byte_data(client,index);

}

static void bd6084_bl_enable(struct backlight_device *bd)
{

}

static void bd6084_bl_disable(struct backlight_device *bd)
{

}

int bd6084_bl_set_ldo(enum bd6084_ldo ldo, int en)
{
	struct bd6084_bl_driver_data *drvdata;
	struct backlight_device *bd;
	int data, shift;
	int ret = 0;

	mutex_lock(&bl_mutex);
	if (!bd6084_bl_drvdata) {
		pr_err("%s: bd6084_bl_drvdata points NULL\n",
		       BD6084_BL_DRV_NAME);
		ret = -ENXIO;
		goto out;
	}

	drvdata = bd6084_bl_drvdata;
	bd = drvdata->bd;

	switch (ldo) {
	case BD6084_LDO1:
	case BD6084_LDO12:	
		// LDO 12 voltage control
// 20110625 sangki.hyun@lge.com X2 LCD VCC 2.8V [S] {
#if (HITACH_PANEL)
		bd6084_bl_write(bd, 0x14, 0xA4);
#else
		bd6084_bl_write(bd, 0x14, 0xD4);
#endif
// 20110625 sangki.hyun@lge.com X2 LCD VCC 2.8V [E] }
		shift = 0;
		break;
	case BD6084_LDO2:
		// LDO 12 voltage control
// 20110625 sangki.hyun@lge.com X2 LCD VCC 2.8V [S] {
#if (HITACH_PANEL)  
		bd6084_bl_write(bd, 0x14, 0xA4);
#else
		bd6084_bl_write(bd, 0x14, 0xD4);
#endif
// 20110625 sangki.hyun@lge.com X2 LCD VCC 2.8V [E] }

		shift = 1;
		break;
	case BD6084_LDO3:
	case BD6084_LDO34:
		// LDO 34 voltage control
		bd6084_bl_write(bd, 0x15, 0xC4);
		shift = 2;
		break;
	case BD6084_LDO4:
		// LDO 34 voltage control
		bd6084_bl_write(bd, 0x15, 0xC4);
		shift = 3;
		break;
	default:
		dev_err(&drvdata->bd->dev, "invalid LDO ID: %d\n", ldo);
		ret = -EINVAL;
		goto out;
	}

	data = bd6084_bl_read(bd, 0x13);
	
	printk("bd6084_bl_get_ldo : %X\r\n", data);
	
	if((ldo == BD6084_LDO12) || (ldo == BD6084_LDO34))
	{
		if(en)
		{	
			data |= (0x3 << shift);
		}
		else
		{
			data &= ~(0x3 << shift);
		}
	}
	else
	{
		if(en)
		{	
			data |= (1 << shift);
		}
		else
		{
			data &= ~(1 << shift);
		}
	}
	
	printk("bd6084_bl_set_ldo : %X\r\n", data);

	if (bd6084_bl_write(bd, 0x13, data & 0xff) < 0) {
		ret = -EIO;
		goto out;
	}
out:
	mutex_unlock(&bl_mutex);
	return ret;
}
EXPORT_SYMBOL(bd6084_bl_set_ldo);

int bd6084_forced_off(void)
{
	struct bd6084_bl_driver_data *drvdata;
	struct backlight_device *bd;
	int ret = 0;

	mutex_lock(&bl_mutex);
	if (!bd6084_bl_drvdata) {
		pr_err("%s: bd6084_bl_drvdata points NULL\n",
		       BD6084_BL_DRV_NAME);
		mutex_unlock(&bl_mutex);
		ret = -EINVAL;
		return ret;
	}

	drvdata = bd6084_bl_drvdata;
	bd = drvdata->bd;

	bd6084_bl_write(bd, 0x02, 0x00);

out:
	mutex_unlock(&bl_mutex);
	return ret;
}
EXPORT_SYMBOL(bd6084_forced_off);

int bd6084_forced_resume(void)
{
	struct bd6084_bl_driver_data *drvdata;
	struct backlight_device *bd;
	int ret = 0;

	mutex_lock(&bl_mutex);
	if (!bd6084_bl_drvdata) {
		pr_err("%s: bd6084_bl_drvdata points NULL\n",
		       BD6084_BL_DRV_NAME);
		mutex_unlock(&bl_mutex);
		ret = -EINVAL;
		return ret;
	}

	drvdata = bd6084_bl_drvdata;
	bd = drvdata->bd;

	mutex_unlock(&bl_mutex);

	backlight_update_status(bd);

	return ret;
}
EXPORT_SYMBOL(bd6084_forced_resume);

static int bd6084_bl_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int bd6084_bl_update_status(struct backlight_device *bd)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	int ret = 0;

//	dbg("props: brightness=%d, power=%d, state=%d\n",
//	    bd->props.brightness, bd->props.power, bd->props.state);

	mutex_lock(&bl_mutex);

	drvdata->brightness = brightness;

	if(is_suspended == true)
	{
		mutex_unlock(&bl_mutex);
		return 0;
	}

//	dbg(" brightness=%d", bd->props.brightness);

	// 20110622 deukgi.shin@lge.com Setting backlight brightness auto mode [start]
	if (set_auto_brightness > 0) {
		if ((set_alc_option == -1) || (set_alc_option != old_auto_brightness)) { // 20110715 deukgi.shin@lge.com //add codition for diag test mode          
            old_auto_brightness = set_alc_option;
			if(set_alc_option !=  ALC_OPTION_POWER_SAVE_BRIGHTNESS){
				printk("Here is optimum brightness mode");
				bd6084_bl_write(bd, 0x01, 0x3F);	
// 20110623 sangki.hyun@lge.com THL : 65.54ms, TLH : 32.77ms [S] {
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_LU6500) || defined (CONFIG_KS1103)
				bd6084_bl_write(bd, 0x09, 0x87);    // Main Current up/down slope time setting
#else
				bd6084_bl_write(bd, 0x09, 0x55); 	// Main Current up/down slope time setting 8.192 ms
#endif
// 20110623 sangki.hyun@lge.com THL : 65.54ms, TLH : 32.77ms [E] }
// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
				bd6084_bl_write(bd, 0x0B, b_reg_value); 	// ALC slope curve setting
				bd6084_bl_write(bd, 0x0D, d_reg_value);   // Main Group Min Current
				bd6084_bl_write(bd, 0x0E, e_reg_value);    // Main Group Max Current
#else  /* Not ALC_TEST_CONFIG */
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
				bd6084_bl_write(bd, 0x0B, 0x03);     // ALC slope curve setting
				bd6084_bl_write(bd, 0x0D, 0x0D);     // Main Group Min Current
				bd6084_bl_write(bd, 0x0E, 0x63);     // Main Group Max Current
#elif defined (CONFIG_KS1001)
				bd6084_bl_write(bd, 0x0B, 0x01);   // ALC slope curve setting = 1.1mA
				bd6084_bl_write(bd, 0x0D, 0x0C);   // Main Group Min Current  = 2.6mA
				bd6084_bl_write(bd, 0x0E, 0x63);   // Main Group Max Current  = 20mA
#elif defined (CONFIG_KS1103)
#if 0  // GB merge kdh
				bd6084_bl_write(bd, 0x0B, 0x07);   // ALC slope curve setting = 1.1mA
				bd6084_bl_write(bd, 0x0D, 0x10);   // Main Group Min Current  = 2.6mA
				bd6084_bl_write(bd, 0x0E, 0x4B);   // Main Group Max Current  = 20mA
#else
				bd6084_bl_write(bd, 0x0B, 0x0B);   // ALC slope curve setting = 1.1mA
				bd6084_bl_write(bd, 0x0D, 0x08);   // Main Group Min Current  = 2.6mA
				bd6084_bl_write(bd, 0x0E, 0x63);   // Main Group Max Current  = 20mA
#endif				
				
#elif defined (CONFIG_LU6500)
				bd6084_bl_write(bd, 0x0B, 0x01);   // ALC slope curve setting -> init value is '0x00'  log curve, 1mA step
				bd6084_bl_write(bd, 0x0D, 0x0C);   // Main Group Min Current  -> init value is '0x0E'  3mA
				bd6084_bl_write(bd, 0x0E, 0x63);   // Main Group Max Current -> init value is '0x63'  20 mA
#endif
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }
			}else{
				printk("Here is power saving mode");
				bd6084_bl_write(bd, 0x01, 0x3F);	
// 20110623 sangki.hyun@lge.com THL : 65.54ms, TLH : 32.77ms [S] {
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_LU6500) || defined (CONFIG_KS1103)
				bd6084_bl_write(bd, 0x09, 0x87);    // Main Current up/down slope time setting
#else
				bd6084_bl_write(bd, 0x09, 0x55);    // Main Current up/down slope time setting 8.192 ms
#endif
// 20110623 sangki.hyun@lge.com THL : 65.54ms, TLH : 32.77ms [E] }
// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
				bd6084_bl_write(bd, 0x0B, b_reg_value);     // ALC slope curve setting
				bd6084_bl_write(bd, 0x0D, d_reg_value); // Main Group Min Current
				bd6084_bl_write(bd, 0x0E, e_reg_value); // Main Group Max Current
#else
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
				bd6084_bl_write(bd, 0x0B, 0x00);     // ALC slope curve setting
				bd6084_bl_write(bd, 0x0D, 0x0A); // Main Group Min Current
				bd6084_bl_write(bd, 0x0E, 0x20); // Main Group Max Current
#elif defined (CONFIG_KS1001) 
				bd6084_bl_write(bd, 0x0B, 0x00); // ALC slope curve setting = 1.0mA
				bd6084_bl_write(bd, 0x0D, 0x08); // Main Group Min Current	= 1.8mA
				bd6084_bl_write(bd, 0x0E, 0x1F); // Main Group Max Current	= 6.4mA
#elif defined (CONFIG_KS1103)
#if 0  // GB merge kdh
				bd6084_bl_write(bd, 0x0B, 0x03); // ALC slope curve setting      = 1.0mA
				bd6084_bl_write(bd, 0x0D, 0x0F); // Main Group Min Current	= 3.2mA
				bd6084_bl_write(bd, 0x0E, 0x2D); // Main Group Max Current	= 9.4mA
#else
				bd6084_bl_write(bd, 0x0B, 0x00); // ALC slope curve setting      = 1.0mA
				bd6084_bl_write(bd, 0x0D, 0x08); // Main Group Min Current	= 3.2mA
				bd6084_bl_write(bd, 0x0E, 0x2D); // Main Group Max Current	= 9.4mA
#endif
#elif defined (CONFIG_LU6500)
				bd6084_bl_write(bd, 0x0B, 0x00); // ALC slope curve setting ->  log curve, 1.1mA step
				bd6084_bl_write(bd, 0x0D, 0x08); // Main Group Min Current = 2.6 mA
				bd6084_bl_write(bd, 0x0E, 0x1F); // Main Group Max Current = 7 mA
#endif
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }
			}
		}
		
	} else {
	    old_auto_brightness = MANUAL_MODE_BRIGHTNESS;
		bd6084_bl_write(bd, 0x01, 0x3E); // all main group control, not ALC
		bd6084_bl_write(bd, 0x09, 0x00); // slope time control
	}
	// 20110622 deukgi.shin@lge.com Setting backlight brightness auto mode [end]
	
	if ((bd->props.power != FB_BLANK_UNBLANK)
			|| (bd->props.state & BL_CORE_FBBLANK)
			|| (bd->props.state & BL_CORE_SUSPENDED))
		brightness = 0;

	if (brightness == 0) {
		// LED main group off
		bd6084_bl_write(bd, 0x02, 0x00);
//		is_suspended = true;
	} else {
		if (set_auto_brightness > 0) { //Auto Brightness SET!!!
//20111108 - woo.jung@lge.com Auto Brightness Can set until 20.
			// LED main group on
#if defined (CONFIG_KS1103)		
			//20111124 - woo.jung@lge.com 20 -> 15 for Setting & Dimming Light [S]
			if(brightness < 15) 
#else
			if(brightness <= 15 )
#endif
			{
				bd6084_bl_write(bd, 0x01, 0x3E); // all main group control, not ALC
				bd6084_bl_write(bd, 0x03, bd6084_bl_adjust_brightness(brightness)); //20mA
			}
#if defined (CONFIG_KS1103)			
			else if(brightness >= 15) // LCD Brightness is set Setting until 20ms
#else
			else if(brightness > 15) // LCD Brightness is set Setting until 20ms			
#endif
			//20111124 - woo.jung@lge.com 20 -> 15 for Setting & Dimming Light [E]

			{
				bd6084_bl_write(bd, 0x01, 0x3F);	
			}
			bd6084_bl_write(bd, 0x02, 0x41);
		} else {
			// LED main group current
// 20110524 bg80.song@lge.com Max LED Current = 20mA, K1001 H/W Team Request [S]
			bd6084_bl_write(bd, 0x03, bd6084_bl_adjust_brightness(brightness)); //20mA
// 20110524 bg80.song@lge.com Max LED Current = 20mA, K1001 H/W Team Request [E]				
			// LED main group on
			bd6084_bl_write(bd, 0x02, 0x01);
		}
	}
	
	
	//mdelay(1);
	msleep(1);// 20110622 deukgi.shin@lge.com for tic sound 

	mutex_unlock(&bl_mutex);

	//drvdata->brightness = brightness;
out:
	return ret;
}

static int bd6084_bl_check_fb(struct backlight_device *bd, struct fb_info *info)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	return !drvdata->check_fb || drvdata->check_fb(&drvdata->client->dev, info);
}

static ssize_t bd6084_bl_auto_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	dbg("show: set_auto_brightness=%d\n", set_auto_brightness);
	return sprintf(buf, "%d\n", set_auto_brightness);
}

static ssize_t bd6084_bl_auto_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;	

	if (value > 0)
		set_auto_brightness = 1;
	else
		set_auto_brightness = 0;

	backlight_update_status(bd);
	
	dbg("store: set_auto_brightness=%d\n", value);
	return size;
}

static ssize_t bd6084_bl_ambient_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int level = 0;
	struct bd6084_bl_driver_data* data = dev_get_drvdata(dev);

	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	mutex_lock(&bl_mutex);
	level = bd6084_bl_read_reg(data->client,0x0C); 
	mutex_unlock(&bl_mutex);
	
	return sprintf(buf, "%d\n", level);
}

static ssize_t bd6084_bl_led_current_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long brightness = simple_strtoul(buf, NULL, 10);
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;

	mutex_lock(&bl_mutex);
	brightness &= 0x7F;
	bd6084_bl_write(bd, 0x01, 0x3E); // all main group control, not ALC
	bd6084_bl_write(bd, 0x09, 0x00); // slope time control
	bd6084_bl_write(bd, 0x03, (int)brightness); // LED main group current
	bd6084_bl_write(bd, 0x02, 0x01); // LED main group on
	mutex_unlock(&bl_mutex);
	
	return 0;
}


////deukgi.shin@lge.com 0608 add sysfs for alc mode option. [start]
static ssize_t bd6084_alc_option_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	dbg("show: ALC MODE option =%d\n", set_alc_option);

	return sprintf(buf, "%d\n", set_alc_option);

}

static ssize_t bd6084_alc_option_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;	


	if ((value == 1) || (value == 0))
	{
		if (value == 0)
			set_alc_option = ALC_OPTION_POWER_SAVE_BRIGHTNESS;
		else
			set_alc_option = ALC_OPTION_OPTIMUM_BRIGHTNESS;

		set_auto_brightness = 1;
// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
		if(set_alc_option == ALC_OPTION_OPTIMUM_BRIGHTNESS)
		{
		    /* ALC_OPTION_OPTIMUM_BRIGHTNESS */
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
		    b_reg_value = 0x03;
		    d_reg_value = 0x0D;
		    e_reg_value = 0x63;
#elif defined (CONFIG_KS1001)
		    b_reg_value = 0x01;
		    d_reg_value = 0x0C;
		    e_reg_value = 0x63;
#elif defined (CONFIG_KS1103)
#if 0	// kdh merge
		    b_reg_value = 0x07;
		    d_reg_value = 0x10;
		    e_reg_value = 0x4B;
#else
		    b_reg_value = 0x0B;
		    d_reg_value = 0x08;
		    e_reg_value = 0x63;
#endif
#elif defined (CONFIG_LU6500)
		    b_reg_value = 0x00;
		    d_reg_value = 0x0E;
		    e_reg_value = 0x63;
#endif
		}
		else
		{
		    /* ALC_OPTION_POWER_SAVE_BRIGHTNESS */
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
		    b_reg_value = 0x00;
		    d_reg_value = 0x0A;
		    e_reg_value = 0x20;
#elif defined (CONFIG_KS1001)
		    b_reg_value = 0x00;
		    d_reg_value = 0x08;
		    e_reg_value = 0x1F;
#elif defined (CONFIG_KS1103)
#if 0	// GB merge kdh
		    b_reg_value = 0x03;
		    d_reg_value = 0x0F;
		    e_reg_value = 0x2D;
#else 
		    b_reg_value = 0x00;
		    d_reg_value = 0x08;
		    e_reg_value = 0x2D;
#endif
#elif defined (CONFIG_LU6500)
		    b_reg_value = 0x01;
		    d_reg_value = 0x0C;
		    e_reg_value = 0x22;
#endif
		}
#endif  /* END ALC_TEST_CONFIG */
// 20110623 sangki.hyun@lge.com alc config [E] }

		backlight_update_status(bd);
		dbg("store: alc_option =%d\n", value);
	}


	return size;

}

////deukgi.shin@lge.com 0608 add sysfs for alc mode option. [end]

// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
static ssize_t bd6084_alc_slope_config_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;	

	b_reg_value = value;
	/* 변경된 값이 바로 적용될 수 있도록  */
	old_auto_brightness = ALC_MODE_CONFIG_TEST;
    
	backlight_update_status(bd);
	dbg("store: b_reg_value =%d\n", value);


	return size;

}

static ssize_t bd6084_alc_current_min_config_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;	

	d_reg_value = value;
    
	/* 변경된 값이 바로 적용될 수 있도록  */
	old_auto_brightness = ALC_MODE_CONFIG_TEST;

	backlight_update_status(bd);
	dbg("store: b_reg_value =%d\n", value);


	return size;

}

static ssize_t bd6084_alc_current_max_config_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bd = drvdata->bd;	

	e_reg_value = value;

	/* 변경된 값이 바로 적용될 수 있도록  */
	old_auto_brightness = ALC_MODE_CONFIG_TEST;

	backlight_update_status(bd);
	dbg("store: b_reg_value =%d\n", value);


	return size;

}
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }

// 20110826 woo.jung@lge.com H/W request to check ALC option [S] }
#ifdef ALC_TEST_CONFIG
static ssize_t bd6084_alc_slope_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bd6084_bl_driver_data* data = dev_get_drvdata(dev);

	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	return sprintf(buf, "%d\n", b_reg_value);
}

static ssize_t bd6084_alc_current_min_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)		
{
	struct bd6084_bl_driver_data* data = dev_get_drvdata(dev);

	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	return sprintf(buf, "%d\n", d_reg_value);
}

static ssize_t bd6084_alc_current_max_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bd6084_bl_driver_data* data = dev_get_drvdata(dev);

	if (buf == NULL) {
		dbg("show: buf is null\n");
		return -ENOMEM;
	}
	
	return sprintf(buf, "%d\n", e_reg_value);
}
#endif
// 20110826 woo.jung@lge.com H/W request to check ALC option [E] }


static DEVICE_ATTR(auto_onoff, S_IRUGO|S_IWUGO, bd6084_bl_auto_show, bd6084_bl_auto_store);
static DEVICE_ATTR(ambient_level, S_IRUGO|S_IWUGO, bd6084_bl_ambient_level_show, NULL);
static DEVICE_ATTR(led_current, S_IRUGO|S_IWUGO, NULL, bd6084_bl_led_current_store);
static DEVICE_ATTR(alc_option, S_IRUGO|S_IWUGO, bd6084_alc_option_show, bd6084_alc_option_store);
// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
#if 0	// GB merge kdh
static DEVICE_ATTR(alc_slope, S_IRUGO|S_IWUGO, bd6084_alc_slope_config_show, bd6084_alc_slope_config_store);  // b_reg_value
static DEVICE_ATTR(alc_current_min, S_IRUGO|S_IWUGO, bd6084_alc_current_min_config_show, bd6084_alc_current_min_config_store);  // d_reg_value
static DEVICE_ATTR(alc_current_max, S_IRUGO|S_IWUGO, bd6084_alc_current_max_config_show, bd6084_alc_current_max_config_store);  // e_reg_value
#else
static DEVICE_ATTR(alc_slope, 0664, bd6084_alc_slope_config_show, bd6084_alc_slope_config_store);  // b_reg_value
static DEVICE_ATTR(alc_current_min, 0664, bd6084_alc_current_min_config_show, bd6084_alc_current_min_config_store);  // d_reg_value
static DEVICE_ATTR(alc_current_max, 0664, bd6084_alc_current_max_config_show, bd6084_alc_current_max_config_store);  // e_reg_value
#endif

// 20110623 sangki.hyun@lge.com alc config [E] }
#endif

static struct attribute *bd6084_bl_attributes[] = {
	&dev_attr_auto_onoff.attr,
	&dev_attr_ambient_level.attr,
	&dev_attr_led_current.attr,
	&dev_attr_alc_option.attr,
// 20110623 sangki.hyun@lge.com alc config [S] {
#ifdef ALC_TEST_CONFIG
	&dev_attr_alc_slope.attr,
	&dev_attr_alc_current_min.attr,
	&dev_attr_alc_current_max.attr,	
#endif
// 20110623 sangki.hyun@lge.com alc config [E] }
	NULL
};

static const struct attribute_group bd6084_bl_group = {
	.attrs = bd6084_bl_attributes,
};

static const struct backlight_ops bd6084_bl_ops = {
	.get_brightness = bd6084_bl_get_brightness,
	.update_status  = bd6084_bl_update_status,
	.check_fb = bd6084_bl_check_fb,
};

static int bd6084_bl_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct bd6084_bl_platform_data *pdata = client->dev.platform_data;
	struct bd6084_bl_driver_data *drvdata;
	struct backlight_device *bd;
	struct backlight_properties props;
	int ret = 0;

	dev_info(&client->dev, "probe\n");

	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		ret = -ENXIO;
		goto out;
	}

	drvdata = kzalloc(sizeof(struct bd6084_bl_driver_data), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&client->dev, "Can't allocate memory for drvdata\n");
		ret = -ENOMEM;
		goto out;
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	//props.max_brightness = 256 - 1;
	
	bd = backlight_device_register(BD6084_BL_DRV_NAME, &client->dev,
				       drvdata, &bd6084_bl_ops, &props);
	if (!bd) {
		dev_err(&client->dev,
			"Can't allocate memory for backlight device\n");
		ret = -ENOMEM;
		goto out_free_drvdata;
	}

	bd6084_bl_drvdata = drvdata;
	drvdata->client = client;
	i2c_set_clientdata(client, drvdata);
	drvdata->bd = bd;
	drvdata->en_pin = pdata->en_pin;
	drvdata->avail_ch = pdata->avail_ch;
	drvdata->max_current = pdata->max_current;
	drvdata->init = pdata->init;
	drvdata->uninit = pdata->uninit;
	drvdata->check_fb = pdata->check_fb;

	if (drvdata->init) {
		ret = drvdata->init(bd);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to init\n");
			goto out_dev_unregister;
		}
	}
	
	bd->props.max_brightness = pdata->max_brightness;

	drvdata->brightness = 0;
	bd->props.power = FB_BLANK_UNBLANK;
	
	bd->props.brightness = 176; //default brightness in bootup
	drvdata->brightness = bd->props.brightness;
	dbg("brightness=%d\n", bd->props.brightness);


#ifdef CONFIG_HAS_EARLYSUSPEND
	drvdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	drvdata->early_suspend.suspend = bd6084_bl_early_suspend;
	drvdata->early_suspend.resume = bd6084_bl_late_resume;
	register_early_suspend(&drvdata->early_suspend);
#endif

	if ((ret = sysfs_create_group(&client->dev.kobj, &bd6084_bl_group))) {
		ret = -ENOMEM;
		goto out_sysfs_group_fail;
	}
	
	wake_lock_init(&bd_wake_lock, WAKE_LOCK_SUSPEND, "bd_wake_lock");  // 20110726 sangki.hyun@lge.com backlight wakelock 
	
	dev_info(&client->dev, "probe end\n");
	
	goto out;

out_sysfs_group_fail:
out_gpio_free:
	if (drvdata->en_pin >= 0)
		gpio_free(drvdata->en_pin);
out_uninit:
	if (drvdata->uninit)
		drvdata->uninit(bd);
out_dev_unregister:
	backlight_device_unregister(bd);
out_free_drvdata:
	kfree(drvdata);
	bd6084_bl_drvdata = NULL;
out:
	return ret;
}

static int bd6084_bl_remove(struct i2c_client *client)
{
	struct bd6084_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

	dev_info(&client->dev, "remove\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&drvdata->early_suspend);
#endif

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	sysfs_remove_group(&client->dev.kobj, &bd6084_bl_group);
	
	if (drvdata->en_pin >= 0)
		gpio_free(drvdata->en_pin);

	if (drvdata->uninit)
		drvdata->uninit(bd);

	backlight_device_unregister(bd);

	kfree(drvdata);
	bd6084_bl_drvdata = NULL;

	return 0;
}

static int bd6084_bl_suspend(struct i2c_client *client, pm_message_t state)
{
	struct bd6084_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

mutex_lock(&bl_mutex);
	is_suspended = true;
	printk("is_suspended true\n");

	dev_info(&client->dev, "suspend\n");
	//backlight_update_status(bd);
#if 0
	drvdata->brightness = 0;
	bd6084_bl_write(bd, 0x02, 0x00);
	mdelay(1);
#endif
mutex_unlock(&bl_mutex);
	return 0;
}

static int bd6084_bl_resume(struct i2c_client *client)
{
	struct bd6084_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

mutex_lock(&bl_mutex);
	dev_info(&client->dev, "resume\n");
	
#if (HITACH_PANEL)
	msleep(50);
#else
	msleep(100); // prevent LGD noise display - sometimes repeat press power key fast, noise is shown.
#endif

	/* Restore backlight */
	old_auto_brightness = -1;

	is_suspended = false;
	printk("is_suspended false\n");
mutex_unlock(&bl_mutex);

	backlight_update_status(bd);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bd6084_bl_early_suspend(struct early_suspend *h)
{
	struct bd6084_bl_driver_data *drvdata;
	drvdata = container_of(h, struct bd6084_bl_driver_data, early_suspend);
	bd6084_bl_suspend(drvdata->client, PMSG_SUSPEND);
	
	wake_lock_timeout(&bd_wake_lock, 5 * HZ);  // 20110726 sangki.hyun@lge.com backlight wakelock 
}

static void bd6084_bl_late_resume(struct early_suspend *h)
{
	struct bd6084_bl_driver_data *drvdata;
	drvdata = container_of(h, struct bd6084_bl_driver_data, early_suspend);
	
	wake_unlock(&bd_wake_lock);  // 20110726 sangki.hyun@lge.com backlight wakelock 
	bd6084_bl_resume(drvdata->client);
}
#endif

static struct i2c_device_id bd6084_bl_idtable[] = {
	{ BD6084_BL_DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bd6084_bl_idtable);

static struct i2c_driver bd6084_bl_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= BD6084_BL_DRV_NAME,
	},
	.id_table	= bd6084_bl_idtable,
	.probe		= bd6084_bl_probe,
	.remove		= bd6084_bl_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= bd6084_bl_suspend,
	.resume		= bd6084_bl_resume,
#endif

};

static int __init bd6084_bl_init(void)
{
	return i2c_add_driver(&bd6084_bl_driver);
}

static void __exit bd6084_bl_exit(void)
{
	wake_lock_destroy(&bd_wake_lock);  // 20110726 sangki.hyun@lge.com backlight wakelock 
	i2c_del_driver(&bd6084_bl_driver);
}

module_init(bd6084_bl_init);
module_exit(bd6084_bl_exit);

MODULE_DESCRIPTION("AnalogicTech BD6084 Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bd6084-backlight");
