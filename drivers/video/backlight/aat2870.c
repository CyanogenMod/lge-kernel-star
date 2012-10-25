/*
 * linux/drivers/video/backlight/aat2870_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/aat2870.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

/* Input Device Event Type */
#define	EVENT_TYPE_LIGHT	ABS_HAT2Y

/*
 * Debug Level
 *	2 : Print all debug messages
 *	1 : Print only dbg() messages
 *	0 : No debug messages
 */
#define AAT2870_BL_DEBUG_LEVEL	0

#if (AAT2870_BL_DEBUG_LEVEL == 2)
#define dbg(format, arg...) \
	printk(KERN_ALERT AAT2870_BL_DRV_NAME \
	       "[LIGHT] : Debug: %s(): " format, __func__, ## arg)
#define enter() \
	printk(KERN_ALERT AAT2870_BL_DRV_NAME "[LIGHT] : Enter: %s()\n", __func__)
#define leave() \
	printk(KERN_ALERT AAT2870_BL_DRV_NAME "[LIGHT] : Leave: %s()\n", __func__)
#elif (AAT2870_BL_DEBUG_LEVEL == 1)
#define dbg(format, arg...) \
	printk(KERN_ALERT AAT2870_BL_DRV_NAME \
	       "[LIGHT] : Debug: %s(): " format, __func__, ## arg)
#define enter()
#define leave()
#else
#define dbg(format, arg...) 	do {} while(0)
#define enter()			do {} while(0)
#define leave()			do {} while(0)	
#endif

static bool is_suspended = false;

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aat2870_bl_early_suspend(struct early_suspend *h);
static void aat2870_bl_late_resume(struct early_suspend *h);
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend

struct aat2870_bl_driver_data *aat2870_bl_drvdata;

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
#ifdef CONFIG_MACH_LGE

// flag indicating ALS enable
static bool als_enabled = false;

#if defined (CONFIG_PANICRPT)    
extern int panicrpt_ispanic (void);
#endif /* CONFIG_PANICRPT */


// AAT2870 register address and data
struct aat2870_bl_command {
	u8	addr;
	u8	data;
};

enum {
	UNINIT_STATE=-1,
	POWERON_STATE,
	NORMAL_STATE,
	SLEEP_STATE,
	POWEROFF_STATE,
	DIMMING_START,
	DIMMING_DONE,
	DIMMING_NONE,
} AAT2870BL_STATE;

#define AAT2870_OP_MODE_NORMAL  (1 << 0)
#define AAT2870_OP_MODE_ALC     (1 << 1)
#define AAT2870_MAX_LIGHT_INTENSITY 0x16

static struct aat2870_ctl_tbl_t aat2870bl_fade_in_tbl[] = {
	{ 0x0c, 0x02 }, //Fade, Disabled
	{ 0x01, 0x00 }, 
	{ 0x00, 0xff },	//LED turn on
	//{ 0x0c, 0x09 }, //Fade In, Enabled
	{ 0x0c, AAT2870_FM_REG_FADEIN_EN_VAL }, //Fade In, Enabled
	{ 0xFF, 0xFE },  /* end of command */		
};


static struct aat2870_ctl_tbl_t aat2870bl_fade_out_tbl[] = {
	{ 0x0B, 0x01 }, 
	{ 0x0C, AAT2870_FM_REG_FADEOUT_EN_VAL },  /* FMT=0.6s, DISABLE_FADE_MAIN=0, FADE_MAIN=fade out */
	{ 0xFF, 0xFE },  /* end of command */		
};


static struct aat2870_ctl_tbl_t aat2870bl_stop_fade_tbl[] = {
	{ 0x0C, 0x03 },  /* FMT=0.6s, DISABLE_FADE_MAIN=0, FADE_MAIN=fade out */
	{ 0xFF, 0xFE },  /* end of command */		
};

/* Set to sleep mode */
static struct aat2870_ctl_tbl_t aat2870bl_sleep_tbl[] = {
	{ 0x0E, 0x26 },  /* SNSR_LIN_LOG=linear, ALSOUT_LIN_LOG=linear, RSET=1k~4k,
	                               * GAIN=low, GM=auto gain, ALS_EN=off */
	{ 0x0F, 0x06 },  /* SBIAS=2.6V, SBIAS=off */
	{ 0x00, 0x00 },  /* Channel Enable=disable */
	{ 0xFF, 0xFE },  /* end of command */	
};

/* Set to Normal mode */
static struct aat2870_ctl_tbl_t aat2870bl_normal_tbl[] = {
	{ 0x00, 0xFF },  /* Channel Enable=ALL */
	{ 0x0E, 0x26 },  /* SNSR_LIN_LOG=linear, ALSOUT_LIN_LOG=linear, RSET=1k~4k,
	                               * GAIN=low, GM=auto gain, ALS_EN=off */
	{ 0x0F, 0x06 },  /* SBIAS=2.6V, SBIAS=off */
	{ 0xFF, 0xFE }	 /* end of command */
};

/* Set to ALC mode HW-high gain mode*/
static struct aat2870_ctl_tbl_t aat2870bl_alc_tbl[] = {
    /* ALC table 0~15 20101218 tunning ver. */
//MOBII_CHNANGE 20120819 ih.han@mobii.co.kr : Modify ALS table value
    {0x12,0x19},  /* ALS current setting 5.6mA */
    {0x13,0x1C},  /* ALS current setting 6.53mA */
    {0x14,0x1E},  /* ALS current setting 7.20mA */
    {0x15,0x20},  /* ALS current setting 7.65mA */
    {0x16,0x22},  /* ALS current setting 7.88mA */
    {0x17,0x23},  /* ALS current setting 8.33mA */
    {0x18,0x25},  /* ALS current setting 9.0mA */
    {0x19,0x27},  /* ALS current setting 9.45mA */
    {0x1A,0x29},  /* ALS current setting 9.68mA */
    {0x1B,0x2A},  /* ALS current setting 10.13mA */
    {0x1C,0x2C},  /* ALS current setting 10.58mA */
    {0x1D,0x30},  /* ALS current setting 11.48mA */
    {0x1E,0x33},  /* ALS current setting 12.15mA */
    {0x1F,0x36},  /* ALS current setting 12.83mA */
    {0x20,0x39},  /* ALS current setting 13.50mA */
    {0x21,0x3C},  /* ALS current setting 14.18mA */

    { 0x0E, 0x73 },  /* SNSR_LIN_LOG=linear, ALSOUT_LIN_LOG=log, RSET=16k~64k,
                                   * GAIN=low, GM=man gain, ALS_EN=on */
    { 0x0F, 0x01 },  /* SBIAS=3.0V, SBIAS=on */
    { 0x10, 0x90 },  /* pwm inactive, auto polling, 1sec, +0% */
    { 0x00, 0xFF },  /* Channel Enable : ALL */
    { 0xFF, 0xFE }   /* end or command */
};


static struct aat2870_lux_tbl_t  aat2870_lux_tbl[] = {
	{0x00,	0},
	{0x01,	50},
	{0x02,	100},
	{0x03,	130},
	{0x04,	160},
	{0x05,	200},
	{0x06,	250},
	{0x07,	300},
	{0x08,	400},
	{0x09,	500},
	{0x0a,	650},
	{0x0b,	800},
	{0x0c,	1000},
	{0x0d,	1400},
	{0x0e,	2000},
	{0x0f,	3000},

	{0x1f,	0x00},
	{0x20,	0x00}
};

// initial ALC current Setting
static struct aat2870_bl_command init_alc_currents[] =
{
//MOBII_CHNANGE 20120819 ih.han@mobii.co.kr : Modify ALS table value
	{0x12,0x19},	/* ALS current setting  5.0mA */
	{0x13,0x1C},	/* ALS current setting  6.0mA */
	{0x14,0x1E},	/* ALS current setting  7.0mA */
	{0x15,0x20},	/* ALS current setting  8.0mA */
	{0x16,0x22},	/* ALS current setting  9.0mA */
	{0x17,0x23},	/* ALS current setting 10.0mA */
	{0x18,0x25},	/* ALS current setting 11.0mA */
	{0x19,0x27},	/* ALS current setting 12.0mA */
	{0x1A,0x29},	/* ALS current setting 13.0mA */
	{0x1B,0x2A},	/* ALS current setting 14.0mA */
	{0x1C,0x2C},	/* ALS current setting 15.0mA */
	{0x1D,0x30},	/* ALS current setting 16.0mA */
	{0x1E,0x33},	/* ALS current setting 17.0mA */
	{0x1F,0x36},	/* ALS current setting 18.0mA */
	{0x20,0x39},	/* ALS current setting 19.0mA */
	{0x21,0x3C},	/* ALS current setting 20.0mA */
	{0xFF,0x00},	/* End of array */
};
#endif
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light

/* Static Utility Functions Here */
static int aat2870_bl_read(struct backlight_device *bd, int addr);
static int aat2870_bl_write(struct backlight_device *bd, int addr, int data);
static int aat2870_bl_send_cmd(struct aat2870_bl_driver_data *drv, struct aat2870_ctl_tbl_t *tbl);
static void aat2870_bl_switch_mode(int op_mode);
static unsigned int aat2870_bl_conv_to_lux(int lev);
static int aat2870_bl_brightness_linearized(int intensity, int *level);
static int calc_brightness(struct backlight_device *bd, int brightness);

/* Static sysfs Functions Here */
static int calc_brightness(struct backlight_device *bd, int brightness)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	int val;

	val = (brightness * drvdata->max_current);
	if ((val % bd->props.max_brightness) >= (bd->props.max_brightness / 2))
		val = (val / bd->props.max_brightness) + 1;
	else
		val = (val / bd->props.max_brightness);

	dbg("result=%d\n", val);

	return val;
}

static int aat2870_bl_write(struct backlight_device *bd, int addr, int data)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
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

static int aat2870_bl_read(struct backlight_device *bd, int addr)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	struct i2c_client *client = drvdata->client;
	int val = 0;
	
	val = i2c_smbus_read_byte_data(client, (char)addr);
	drvdata->reg_cache[addr] = val;

	return drvdata->reg_cache[addr];	
}

static void aat2870_bl_enable(struct backlight_device *bd)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	int i;

	dbg("enable\n");

	if (drvdata->en_pin >= 0) {
// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
#ifdef CONFIG_MACH_LGE
		gpio_set_value(drvdata->en_pin, 0);
		mdelay(1);
#endif
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light

		gpio_set_value(drvdata->en_pin, 1);

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
#ifdef CONFIG_MACH_LGE
		udelay(100);

		if (als_enabled == false) {
			dbg("Debugging Here : Default Flows\n");

			/* Non ALC mode */
			aat2870_bl_write(bd, AAT2870_BL_ALSF_, 0x00);
			/* backlight magnitude */
			aat2870_bl_write(bd, AAT2870_BL_BLM,
				calc_brightness(bd, bd->props.brightness));
			// set backlight magnitude because this function
			// can be called from aat2870_bl_set_ldo()
			// which does not perform magnitude setting

		} else {	/* Auto Brightness Mode */
			for (i=0 ; init_alc_currents[i].addr != 0xFF ; i++) {
				aat2870_bl_write(bd,
					init_alc_currents[i].addr,
					init_alc_currents[i].data);
				udelay(10);
			}

			/* ALS Function, Log Scale, Rset 4K/16K, High gain input setting */
			aat2870_bl_write(bd, AAT2870_BL_ALSF_, 0x71);
			/* SBIAS 3.0V, Offset No adjustment */
			aat2870_bl_write(bd, AAT2870_BL_SBIAS, 0x01);
			/* Auto gain control, polling time 2sec, ALC gain -18.25% adjustment */
			aat2870_bl_write(bd, AAT2870_BL_ALS_G, 0x07);
		}

		/* enable chennel */
		aat2870_bl_write(bd, AAT2870_BL_EN, drvdata->avail_ch);

		if(drvdata->op_mode == AAT2870_OP_MODE_ALC)
		{
			dbg("Debugging op_mode [%d]\n", drvdata->op_mode);
			aat2870_bl_switch_mode(AAT2870_OP_MODE_ALC);	
			dbg("Debugging ALC:ON\n");
		}

		mdelay(1);
#endif
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
	}
}

static void aat2870_bl_disable(struct backlight_device *bd)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);

	dbg("disable\n");

	if (drvdata->en_pin >= 0) {
		gpio_set_value(drvdata->en_pin, 0);

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] reset variable
#ifdef CONFIG_MACH_LGE
		// to trigger enable during resume
		drvdata->brightness = 0;
#endif
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] reset variable
	}
}

int aat2870_bl_set_ldo(enum aat2870_bl_ldo ldo,
		       enum aat2870_bl_ldo_voltage volt, int en)
{
	struct aat2870_bl_driver_data *drvdata;
	struct backlight_device *bd;
	int addr, data, shift;
	int ret = 0;

	if (!aat2870_bl_drvdata) {
		pr_err("%s: aat2870_bl_drvdata points NULL\n",
		       AAT2870_BL_DRV_NAME);
		ret = -ENXIO;
		goto out;
	}

	drvdata = aat2870_bl_drvdata;
	bd = drvdata->bd;

	if ((volt < AAT2870_BL_LDO_1_2V) || (volt > AAT2870_BL_LDO_3_3V)) {
		dev_err(&bd->dev, "invalid LDO voltage: 0x%x\n", volt);
		ret = -EINVAL;
		goto out;
	}

	switch (ldo) {
	case AAT2870_BL_LDOA:
		addr = AAT2870_BL_LDO_AB;
		shift = 4;
		break;
	case AAT2870_BL_LDOB:
		addr = AAT2870_BL_LDO_AB;
		shift = 0;
		break;
	case AAT2870_BL_LDOC:
		addr = AAT2870_BL_LDO_CD;
		shift = 4;
		break;
	case AAT2870_BL_LDOD:
		addr = AAT2870_BL_LDO_CD;
		shift = 0;
		break;
	default:
		dev_err(&drvdata->bd->dev, "invalid LDO ID: %d\n", ldo);
		ret = -EINVAL;
		goto out;
	}

	if (drvdata->brightness == 0)
		aat2870_bl_enable(bd);

	data = aat2870_bl_read(bd, addr);
	data &= ~(AAT2870_BL_LDO_MASK << shift);
	data |= (volt << shift);

	if (aat2870_bl_write(bd, addr, data) < 0) {
		ret = -EIO;
		goto out;
	}

	addr = AAT2870_BL_EN_LDO;
	data = aat2870_bl_read(bd, addr);
	if (en)
		data |= (1 << ldo);
	else
		data &= ~(1 << ldo);

	if (aat2870_bl_write(bd, addr, data) < 0) {
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}
EXPORT_SYMBOL(aat2870_bl_set_ldo);

static int aat2870_bl_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

// MOBII_S [shhong@mobii.co.kr] 2012-05-07 : Auto Brightness Setting From P990.
#if defined (CONFIG_MACH_STAR_P990) || (CONFIG_MACH_STAR_SU660)
static int aat2870_bl_update_modestatus(struct backlight_device *bd)
{
//MOBII_CHNANGE_S 20120819 ih.han@mobii.co.kr : Modify ALS table value
    struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
    int brightness_mode = bd->props.brightness_mode;
    int next_mode;
    static struct aat2870_bl_driver_data *drv;
    drv = aat2870_bl_drvdata;

    if(brightness_mode == 1)
    {
        next_mode = AAT2870_OP_MODE_ALC;
        if (drv->lsensor_enable) {
            schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
        }
    }
    else
    {
        next_mode = AAT2870_OP_MODE_NORMAL;
    }
    aat2870_bl_switch_mode(next_mode);
    drv->op_mode = next_mode;
//MOBII_CHNANGE_E 20120819 ih.han@mobii.co.kr : Modify ALS table value
}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-05-07 : Auto Brightness Setting From P990.

static int aat2870_bl_update_status(struct backlight_device *bd)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	int ret = 0;

	if(is_suspended == true)
	{
		return 0;
	}

	if ((brightness < 0) || (brightness > bd->props.max_brightness)) {
		dev_err(&bd->dev,
			"invalid brightness: %d(0 <= brightness <= %d)\n",
			brightness, bd->props.max_brightness);
		ret = -EINVAL;
		goto out;
	}


#if defined (CONFIG_PANICRPT)    
	if( panicrpt_ispanic() == 1)
		goto out;
#endif
	dbg("props: brightness=%d, power=%d, state=%d\n",
	    bd->props.brightness, bd->props.power, bd->props.state);

	if ((bd->props.power != FB_BLANK_UNBLANK)
			|| (bd->props.state & BL_CORE_FBBLANK)
			|| (bd->props.state & BL_CORE_SUSPENDED))
	{
		brightness = 0;
	}

	if (brightness == 0) {
		aat2870_bl_disable(bd);
	} else {
		if (drvdata->brightness == 0)
		{
			dbg("SEQ3 : enabled Funk!\n");
			aat2870_bl_enable(bd);
		}

		if (aat2870_bl_write(bd, AAT2870_BL_BLM,
				     calc_brightness(bd, brightness)) < 0) {
			ret = -EIO;
			goto out;
		}
// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
// channel enable is done during aat2870_bl_enable()
#ifndef CONFIG_MACH_LGE
		if (drvdata->brightness == 0) {
			if (aat2870_bl_write(bd, AAT2870_BL_EN,
					     drvdata->avail_ch) < 0) {
				ret = -EIO;
				goto out;
			}
		}
#endif
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-17 [LGE_AP20] back-light
	}
	drvdata->brightness = brightness;
out:
	return ret;
}

static int aat2870_bl_check_fb(struct backlight_device *bd, struct fb_info *fi)
{
	return 1;
}

static const struct backlight_ops aat2870_bl_ops = {
	.get_brightness = aat2870_bl_get_brightness,
	.update_status  = aat2870_bl_update_status,
// MOBII_S [shhong@mobii.co.kr] 2012-05-07 : Auto Brightness Setting From P990.
#if defined (CONFIG_MACH_STAR_P990) || (CONFIG_MACH_STAR_SU660)
	.update_modestatus  = aat2870_bl_update_modestatus,
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-05-07 : Auto Brightness Setting From P990.
	.check_fb = aat2870_bl_check_fb,
};

static unsigned int aat2870_bl_conv_to_lux(int lev)
{
        struct aat2870_lux_tbl_t *tbl;
        unsigned val = 0;

        tbl = aat2870_lux_tbl;
        for (;;) {
                if (tbl->lev == lev) {
                        val = tbl->lux;
                        break;
                } else if (tbl->lev >= 0x1f) {
                        break;
                }
                tbl++;
        }
        return val;
}

static int aat2870_bl_send_cmd(struct aat2870_bl_driver_data *drv, struct aat2870_ctl_tbl_t *tbl)
{
        unsigned long delay = 0;

        if (tbl == NULL) {
                dbg("input ptr is null\n");
                return -EIO;
        }

        mutex_lock(&drv->cmd_lock);
        for( ;;) {
                if (tbl->reg == 0xFF) {
                        if (tbl->val != 0xFE) {
                                delay = (unsigned long)tbl->val;
                        }
                        else
                                break;
                }
                else {
                        if (aat2870_bl_write(drv->bd, tbl->reg, tbl->val) != 0)
                                dbg("i2c failed addr:%d, value:%d\n", tbl->reg, tbl->val);
                }
                tbl++;
        }
        mutex_unlock(&drv->cmd_lock);
        return 0;
}

static void aat2870_bl_switch_mode(int op_mode)
{
        struct aat2870_bl_driver_data *drv;

        drv = aat2870_bl_drvdata;
        if (drv->dim_status != DIMMING_NONE) {
                aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
                drv->dim_status = DIMMING_NONE;
                dbg("[BL] DIMMING_OFF \n");
        }

        if (op_mode == AAT2870_OP_MODE_NORMAL) {
                aat2870_bl_send_cmd(drv, drv->cmds.normal);
                dbg("[BL] Mode: normal\n");
        } else if (op_mode == AAT2870_OP_MODE_ALC) {
                aat2870_bl_send_cmd(drv, drv->cmds.alc);
                dbg("[BL] Mode: alc\n");
                if (drv->lsensor_enable == TRUE) 
                        schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
        } 

        if (drv->status == AAT2870_POWER_STATE_OFF) {
                dbg("[BL] Weird: how to use without display\n");
                drv->status = AAT2870_POWER_STATE_ON;
        }

        return;
}

static ssize_t aat2870_bl_show_intensity(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int ret;

	drv = aat2870_bl_drvdata;
	if (!drv) 
		return 0;
	ret = sprintf(buf, "%d\n", drv->intensity);
	return ret;
}

static int aat2870_bl_brightness_linearized(int intensity, int *level)
{
	int ret = 0;
	int remainder;
	int last_intensity; 

	//101017, sk.jang@lge.com Set the Backlight Brightness to be linearized.[START] 
	if (intensity < BRIGHTNESS_MIN) {
		//Too low for intensity value
		//ret = -EINVAL;
		*level = 0;//Handle intensity as 0 level if the value is below the minimum
	} else if ((intensity >= BRIGHTNESS_MIN)&&(intensity<TURNING_POINT)) {
		remainder =((intensity-BRIGHTNESS_MIN-2) % NUMERATOR1);
		last_intensity = ((intensity-BRIGHTNESS_MIN-2)-remainder);
		*level=(last_intensity/NUMERATOR1); // adjust default value to 102(BLM 11stages)
	} else if ((intensity >= TURNING_POINT)&& (intensity <=255)) {
		remainder = ((intensity-TURNING_POINT) % NUMERATOR2);
		last_intensity = ((intensity-TURNING_POINT)-remainder);
		*level=(last_intensity/NUMERATOR2)+((TURNING_POINT-BRIGHTNESS_MIN-2) / NUMERATOR1);
	} else {
		//Too High for intensity value
		ret = -EINVAL; 
	}
	//101017, sk.jang@lge.com Set the Backlight Brightness to be linearized.[END]
	return ret;
}  


static ssize_t aat2870_bl_store_intensity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{    
	int level, intensity;
	static struct aat2870_bl_driver_data *drv;
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &intensity);	//level range: 0 to 22 from aat2870 ds

	//101103, kyungsik.lee@lge.com, Replaced with function.
	if (aat2870_bl_brightness_linearized(intensity, &level)) {
		dbg("[BL] Invalid Intensity value: %d\n", intensity);
		goto intensity_input_err;
	}

	//if (drv->op_mode == AAT2870_OP_MODE_ALC && drv->hw_dimming_enable) {
	if (drv->hw_dimming_enable) {
		if (drv->dim_status == DIMMING_NONE && 
			drv->intensity > level && intensity == 20) {
			dbg("[BL] DIMMING_START \n");
			aat2870_bl_send_cmd(drv, aat2870bl_fade_out_tbl);
			drv->dim_status = DIMMING_START;
			mdelay(AAT2870_FADE_OUT_DELAY);
		} else if (drv->dim_status != DIMMING_NONE && drv->intensity < level) {
			aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			dbg("[BL] DIMMING_OFF \n");
		}
	}

	if (drv->power_onoff_ref == TRUE) {
		aat2870_bl_send_cmd(drv, aat2870bl_fade_in_tbl);
		if (drv->op_mode != AAT2870_OP_MODE_ALC) {
			aat2870_bl_write(drv->bd, AAT2870_BL_BLM, level);
		} else {
			aat2870_bl_write(drv->bd, AAT2870_BL_BLM, 0x06);//lowest
		}
		mdelay(AAT2870_FADE_IN_DELAY - 50);//Fade-In delay
		aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
		drv->power_onoff_ref = FALSE;
	}

	if (level > drv->intensity_max) {
		level = drv->intensity_max;
	}
	dbg("[BL] intensity: %d, level: %d(prev: %d)\n",intensity, level, drv->intensity);
	aat2870_bl_write(drv->bd, AAT2870_BL_BLM, level);
	drv->intensity = level;
	
intensity_input_err:
	return count;
}


static ssize_t aat2870_bl_show_alc_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int alc_level = 0;
	int ret;


	drv = aat2870_bl_drvdata;
	if (!drv)
		return 0;

	if (drv->op_mode == AAT2870_OP_MODE_NORMAL) {
		alc_level = -1;
	} else if (drv->op_mode == AAT2870_OP_MODE_ALC) {
		alc_level = (aat2870_bl_read(drv->bd, AAT2870_BL_AMB) >> 3);
	} 
	ret = sprintf(buf, "%d\n", aat2870_bl_conv_to_lux(alc_level));

	return ret;
}

static ssize_t aat2870_bl_show_hwdim(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int r;

	drv = aat2870_bl_drvdata;
	if (!drv)
		return 0;
	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->hw_dimming_enable == TRUE)  ? "1":"0");
	return r;
}

static ssize_t aat2870_bl_store_hwdim(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int dimming;
	static struct aat2870_bl_driver_data *drv;
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &dimming);

	if (drv->dim_status != DIMMING_NONE) {
		aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
		drv->dim_status = DIMMING_NONE;
		dbg("[BL] DIMMING_OFF \n");
	}

	if (dimming) {
		drv->hw_dimming_enable = TRUE;
	} else {
		drv->hw_dimming_enable = FALSE;
	}

	return count;
}

static ssize_t aat2870_bl_show_lsensor_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int ret;

	drv = aat2870_bl_drvdata;
	if (!drv) 
		return 0;
	ret = snprintf(buf, PAGE_SIZE, "%s\n", (drv->lsensor_enable == TRUE)  ? "1":"0");
	return ret;
}

static ssize_t aat2870_bl_store_lsensor_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	static struct aat2870_bl_driver_data *drv;
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &onoff);
	if (onoff) {
		drv->lsensor_enable = TRUE;
		if (drv->op_mode == AAT2870_OP_MODE_ALC) {
			schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
		}
	} else {
		drv->lsensor_enable = FALSE;
	}
	return count;
}

static ssize_t aat2870_bl_show_alc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int r;

	drv = aat2870_bl_drvdata;
	if (!drv) 
		return 0;
	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->op_mode == AAT2870_OP_MODE_ALC)  ? "1":"0");
	return r;
}

static ssize_t aat2870_bl_store_alc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int alc;
	int next_mode;
	static struct aat2870_bl_driver_data *drv;
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;
	sscanf(buf, "%d", &alc);

	if (alc) {
		next_mode = AAT2870_OP_MODE_ALC;
		if (drv->lsensor_enable) {
			schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
		}
	} else {
		next_mode = AAT2870_OP_MODE_NORMAL;
	}
	aat2870_bl_switch_mode(next_mode);
	drv->op_mode = next_mode;
	return count;
}

static ssize_t aat2870_bl_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_bl_driver_data *drv;
	int r;

	drv = aat2870_bl_drvdata;
	if (!drv) 
		return 0;
	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->status == AAT2870_POWER_STATE_ON)  ? "1":"0");
	return r;
}

static ssize_t aat2870_bl_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	static struct aat2870_bl_driver_data *drv;
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;
	sscanf(buf, "%d", &onoff);

	if (onoff && drv->status == AAT2870_POWER_STATE_OFF) {
		if(drv->dim_status != DIMMING_NONE)	{
			aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			dbg("[BL] DIMMING_OFF \n");
		}

		if (drv->op_mode == AAT2870_OP_MODE_NORMAL) {
			aat2870_bl_send_cmd(drv, drv->cmds.normal);
		} else if (drv->op_mode == AAT2870_OP_MODE_ALC) {
			aat2870_bl_send_cmd(drv, drv->cmds.alc);
		} 
		drv->status = AAT2870_POWER_STATE_ON;
		drv->power_onoff_ref = TRUE;
		dbg("[BL] Power On\n");
	} else if (!onoff && drv->status == AAT2870_POWER_STATE_ON) {
		if(drv->dim_status != DIMMING_NONE)	{
			aat2870_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			dbg("[BL] DIMMING_OFF \n");
		}
		aat2870_bl_send_cmd(drv, drv->cmds.sleep);
		drv->status = AAT2870_POWER_STATE_OFF;
		drv->power_onoff_ref = FALSE;
		dbg("[BL] Power Off\n");
	}

	return count;
}

// MOBII_S [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery
#if defined(CONFIG_MACH_STAR_SU660)
static ssize_t aat2870_bl_store_foff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	static struct aat2870_bl_driver_data *drv;	
	
	drv = aat2870_bl_drvdata;
	if (!count)
		return -EINVAL;
	sscanf(buf, "%d", &onoff);
    
	aat2870_bl_send_cmd(drv, drv->cmds.sleep);
	drv->status = AAT2870_POWER_STATE_OFF;
	drv->power_onoff_ref = FALSE;
	dbg("[BL] Power Off\n");
    
	// aat2870_bl_reset_init();
	aat2870_bl_enable(drv->bd);
	dbg("[BL] aat2870_bl_reset\n");
	return count;
}
static ssize_t aat2870_bl_show_panel_info(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifndef	TEGRA_GPIO_PJ5
#define TEGRA_GPIO_PJ5	77
#endif
	unsigned int panel_info = 0;
	int ret = -1;
	tegra_gpio_enable(TEGRA_GPIO_PJ5);
	gpio_direction_input(TEGRA_GPIO_PJ5);
	panel_info = gpio_get_value(TEGRA_GPIO_PJ5);
	if( panel_info == 0 || panel_info == 1) {
		dbg("[BD] Panel Info Value [%d].", panel_info);
		sprintf(buf, "%d\n", panel_info);
                ret = (ssize_t)(strlen(buf) + 1);
	} else {
		dbg("[BD] Invalid Panel Info Value.");
		sprintf(buf, "%d\n", panel_info);
		ret = -1;
	}
	return ret;
#ifdef	TEGRA_GPIO_PJ5
#undef	TEGRA_GPIO_PJ5
#endif
}
static ssize_t aat2870_bl_store_panel_info(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	return 0;
}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery

/* Sysfs Block */
static DEVICE_ATTR(intensity, 0666, aat2870_bl_show_intensity, aat2870_bl_store_intensity);
static DEVICE_ATTR(alc_level, 0444, aat2870_bl_show_alc_level, NULL);
static DEVICE_ATTR(alc,       0664, aat2870_bl_show_alc, aat2870_bl_store_alc);
static DEVICE_ATTR(onoff,     0666, aat2870_bl_show_onoff, aat2870_bl_store_onoff);
static DEVICE_ATTR(hwdim,     0666, aat2870_bl_show_hwdim, aat2870_bl_store_hwdim);
static DEVICE_ATTR(lsensor_onoff, 0666, aat2870_bl_show_lsensor_onoff, aat2870_bl_store_lsensor_onoff);
// MOBII_S [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery
#if defined(CONFIG_MACH_STAR_SU660)
static DEVICE_ATTR(panel_info,0666, aat2870_bl_show_panel_info, aat2870_bl_store_panel_info);
static DEVICE_ATTR(foff,      0666, aat2870_bl_show_onoff, aat2870_bl_store_foff);
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery

/* Sysfs Struct */
static struct attribute *aat2870_bl_attributes[] = {
        &dev_attr_intensity.attr,
        &dev_attr_alc_level.attr,
        &dev_attr_alc.attr,
        &dev_attr_onoff.attr,
        &dev_attr_hwdim.attr,
        &dev_attr_lsensor_onoff.attr,
// MOBII_S [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery
#if defined(CONFIG_MACH_STAR_SU660)
	&dev_attr_panel_info.attr,
	&dev_attr_foff.attr,
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-05-10 : Rollbacked Sysfs Recovery
        NULL,
};

static const struct attribute_group aat2870_bl_group = {
        .attrs = aat2870_bl_attributes,
};

static void aat2870_bl_work_func(struct work_struct *wq)
{
        struct aat2870_bl_driver_data *drv;
        int alc_level = 0;

        drv = aat2870_bl_drvdata;
        if ((drv->op_mode == AAT2870_OP_MODE_ALC) && drv->lsensor_enable) {
                alc_level = (aat2870_bl_read(drv->bd, AAT2870_BL_AMB) >> 3);

		/* Send Input Event to Upper Layer */
		input_report_abs(drv->input_dev, EVENT_TYPE_LIGHT, aat2870_bl_conv_to_lux(alc_level));
		input_sync(drv->input_dev);
                schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
                dbg("ALC Level [%d]\n", aat2870_bl_conv_to_lux(alc_level));
        }
}

static int aat2870_bl_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct aat2870_bl_platform_data *pdata = client->dev.platform_data;
	struct aat2870_bl_driver_data *drvdata;
	struct backlight_device *bd;
	struct backlight_properties props;
	int ret = 0;

	dev_info(&client->dev, "probe\n");

	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		ret = -ENXIO;
		goto out;
	}

	drvdata = kzalloc(sizeof(struct aat2870_bl_driver_data), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&client->dev, "Can't allocate memory for drvdata\n");
		ret = -ENOMEM;
		goto out;
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	
	bd = backlight_device_register(AAT2870_BL_DRV_NAME, &client->dev,
				       drvdata, &aat2870_bl_ops, &props);
	if (!bd) {
		dev_err(&client->dev,
			"Can't allocate memory for backlight device\n");
		ret = -ENOMEM;
		goto out_free_drvdata;
	}

	/* Allocate Input Device */
	drvdata->input_dev = input_allocate_device();
        if (!drvdata->input_dev) {
                dbg("Error allocate input device\n");
                ret = -ENOMEM;
                goto out_free_drvdata;
        }

        drvdata->input_dev->name = AAT2870_BL_DRV_NAME;
        set_bit(EV_SYN|EV_ABS, drvdata->input_dev->evbit);
        input_set_abs_params(drvdata->input_dev, EVENT_TYPE_LIGHT, 0, 3000, 0, 0);

	ret = input_register_device(drvdata->input_dev);
        if (ret){
                dbg("Error register input device [%d]\n", ret);
                ret = -ENODEV;
                goto out_input_unregister;
        }

	/* Driver Data Information : Orig */
	aat2870_bl_drvdata = drvdata;
	drvdata->client = client;
	i2c_set_clientdata(client, drvdata);
	drvdata->bd = bd;
	drvdata->en_pin = pdata->en_pin;
	drvdata->avail_ch = pdata->avail_ch;
	drvdata->max_current = pdata->max_current;
	drvdata->init = pdata->init;
	drvdata->uninit = pdata->uninit;

	/* Driver Data Information */
	drvdata->cmds.normal = aat2870bl_normal_tbl;
        drvdata->cmds.alc = aat2870bl_alc_tbl;
        drvdata->cmds.sleep = aat2870bl_sleep_tbl;
        drvdata->op_mode = AAT2870_OP_MODE_NORMAL;
        drvdata->dim_status = DIMMING_NONE;
        drvdata->intensity = BACKLIGHT_DEFAULT;
        drvdata->intensity_max = AAT2870_INTENSITY_MAX;
        drvdata->hw_dimming_enable = TRUE;
        drvdata->lsensor_enable = FALSE;
        drvdata->lsensor_poll_time = AAT2870_DEFAULT_LSENSOR_POLL_TIME;
        drvdata->power_onoff_ref = FALSE;	
	/* End Driver Data Information */

	if (drvdata->init) {
		ret = drvdata->init(bd);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to init\n");
			goto out_dev_unregister;
		}
	}

	if (drvdata->en_pin >= 0) {
		ret = gpio_request(drvdata->en_pin, "aat2870-en");
		if (ret) {
			dev_err(&client->dev,
				"Failed to request GPIO %d\n", drvdata->en_pin);
			goto out_uninit;
		}
		gpio_direction_output(drvdata->en_pin, 0);
	}

	bd->props.max_brightness = 100;
	if (0 < pdata->max_brightness)
		bd->props.max_brightness = pdata->max_brightness;

	drvdata->brightness = 0;
	bd->props.power = FB_BLANK_UNBLANK;
#ifdef CONFIG_MACH_LGE
	bd->props.brightness = (bd->props.max_brightness) / 2;
	dbg("brightness=%d\n", bd->props.brightness);
#else
	bd->props.brightness = bd->props.max_brightness;
#endif

	ret = aat2870_bl_update_status(bd);
	if (ret < 0) {
		dev_err(&client->dev, "Can't initialize\n");
		goto out_gpio_free;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	drvdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;	//Mobii_Change sgkim@mobii.co.kr 20120608- Speed up LCD On : Add [+1]
	drvdata->early_suspend.suspend = aat2870_bl_early_suspend;
	drvdata->early_suspend.resume = aat2870_bl_late_resume;
	register_early_suspend(&drvdata->early_suspend);
#endif

	if ((ret = sysfs_create_group(&client->dev.kobj, &aat2870_bl_group))) {
		ret = -ENOMEM;
		goto out_sysfs_group_fail;
	}

	mutex_init(&drvdata->lock);
        mutex_init(&drvdata->cmd_lock);

        aat2870_bl_write(drvdata->bd, AAT2870_BL_BLM, drvdata->intensity);//default intensity
        aat2870_bl_send_cmd(drvdata, drvdata->cmds.normal);

	INIT_DELAYED_WORK(&drvdata->delayed_work_bl, aat2870_bl_work_func);

	goto out;

out_sysfs_group_fail:
out_gpio_free:
	if (drvdata->en_pin >= 0)
		gpio_free(drvdata->en_pin);
out_uninit:
	if (drvdata->uninit)
		drvdata->uninit(bd);
	/* Input Device Unreg.*/
	input_unregister_device(drvdata->input_dev);
out_input_unregister:
	/* Input Device Free */
	input_free_device(drvdata->input_dev);
out_dev_unregister:
	backlight_device_unregister(bd);
out_free_drvdata:
	kfree(drvdata);
	aat2870_bl_drvdata = NULL;
out:
	return ret;
}

static int aat2870_bl_remove(struct i2c_client *client)
{
	struct aat2870_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

	dev_info(&client->dev, "remove\n");

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&drvdata->early_suspend);
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	sysfs_remove_group(&client->dev.kobj, &aat2870_bl_group);
	if (drvdata->en_pin >= 0)
		gpio_free(drvdata->en_pin);

	if (drvdata->uninit)
		drvdata->uninit(bd);

	backlight_device_unregister(bd);

	kfree(drvdata);
	aat2870_bl_drvdata = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int aat2870_bl_suspend(struct i2c_client *client, pm_message_t state)
{
	struct aat2870_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

	dev_info(&client->dev, "suspend\n");
	is_suspended = true;

	
#if defined (CONFIG_PANICRPT)    
	if( panicrpt_ispanic() == 1)
		return 0;
#endif
	aat2870_bl_disable(bd);
	return 0;
}

static int aat2870_bl_resume(struct i2c_client *client)
{
	struct aat2870_bl_driver_data *drvdata = i2c_get_clientdata(client);
	struct backlight_device *bd = drvdata->bd;

	dev_info(&client->dev, "resume\n");
	is_suspended = false;

	/* Restore backlight */
	backlight_update_status(bd);
	// aat2870_bl_enable() will be called during backlight_update_status()

	/* Restore LDOs */
	aat2870_bl_write(bd, AAT2870_BL_LDO_AB,
			 aat2870_bl_read(bd, AAT2870_BL_LDO_AB));
	aat2870_bl_write(bd, AAT2870_BL_LDO_CD,
			 aat2870_bl_read(bd, AAT2870_BL_LDO_CD));
	aat2870_bl_write(bd, AAT2870_BL_EN_LDO,
			 aat2870_bl_read(bd, AAT2870_BL_EN_LDO));

	return 0;
}

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aat2870_bl_early_suspend(struct early_suspend *h)
{
	struct aat2870_bl_driver_data *drvdata;
	drvdata = container_of(h, struct aat2870_bl_driver_data, early_suspend);
	aat2870_bl_suspend(drvdata->client, PMSG_SUSPEND);
}

static void aat2870_bl_late_resume(struct early_suspend *h)
{
	struct aat2870_bl_driver_data *drvdata;
	drvdata = container_of(h, struct aat2870_bl_driver_data, early_suspend);
	aat2870_bl_resume(drvdata->client);
}
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#else
#define aat2870_bl_suspend	NULL
#define aat2870_bl_resume	NULL
#endif /* CONFIG_PM */

static struct i2c_device_id aat2870_bl_idtable[] = {
	{ AAT2870_BL_DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, aat2870_bl_idtable);

static struct i2c_driver aat2870_bl_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= AAT2870_BL_DRV_NAME,
	},
	.id_table	= aat2870_bl_idtable,
	.probe		= aat2870_bl_probe,
	.remove		= aat2870_bl_remove,
// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= aat2870_bl_suspend,
	.resume		= aat2870_bl_resume,
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
};

static int __init aat2870_bl_init(void)
{
	return i2c_add_driver(&aat2870_bl_driver);
}

static void __exit aat2870_bl_exit(void)
{
	i2c_del_driver(&aat2870_bl_driver);
}

module_init(aat2870_bl_init);
module_exit(aat2870_bl_exit);

MODULE_DESCRIPTION("AnalogicTech AAT2870 Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:aat2870-backlight");
