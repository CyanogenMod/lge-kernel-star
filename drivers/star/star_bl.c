

/*
 * drivers/star/star_bl.c
 *
 * Star Backlight Driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */


#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <mach/lprintk.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define NV_DEBUG 0

#include "nvcommon.h"

#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "nvrm_gpio.h"

#define SLAVE_ADDR	0x60 //star backlight

#include "aat2870.h"
#include "star_bl.h"
#include "star_gpioi2c.h"


/*
 * Debug
 */
//#define DEBUG_BL

#ifdef DEBUG_BL
#define ENABLE_DEBUG_MESSAGE 0x01
#define ENABLE_TEST_MODE 0x02
static int debug_enable_flag = 0x01;
#define DBG(x...) if (debug_enable_flag & ENABLE_DEBUG_MESSAGE) { \
						printk(x); \
				  }
#else
#define DBG(x...) do { } while(0)
#endif


#if !defined(TRUE)
#define TRUE 0x01
#endif
#if !defined(FALSE)
#define FALSE 0x00 
#endif

/*
 * Driver Parameters
 */
#define STAR_AMBIENT_LIGHT_SENSOR_DRV_NAME "star_light_sensor"
#define EVENT_TYPE_LIGHT ABS_HAT2Y

/*
 * Parameters
 */
#define BL_HW_RESET_DELAY 100 //us
#define BL_INTENSITY_MAX 0x16
#define BL_DEFAULT_LSENSOR_POLL_TIME msecs_to_jiffies(1000)
#define BL_FADE_IN_DELAY 400
#define BL_FADE_OUT_DELAY 400

/*
 * Test
 */
#define BL_ALS_LEV_TO_LUX_TBL_APPROXIMATION



#define BL_POWER_STATE_ON 0x01
#define BL_POWER_STATE_OFF 0x00

// 101103 , Minimum Brightness level for HW Dimming
#define LCD_LED_DIM 1
// 101017  added some variables to adjust backlight brightness
// 101103 , Define parameters from global variables
#define BRIGHTNESS_MIN 30
#define NUMERATOR1 6
#define NUMERATOR2 14
#define TURNING_POINT 104


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


struct aat2870_ctl_tbl_t {
	unsigned char reg;
	unsigned char val;
};


struct aat2870_lux_tbl_t {
	unsigned int lev;
	unsigned int lux;
};


struct aat2870_cmds_t {
	struct aat2870_ctl_tbl_t *normal;
	struct aat2870_ctl_tbl_t *alc;
	struct aat2870_ctl_tbl_t *sleep;
};


struct aat2870_drvdata_t {
		
	int op_mode;
	int intensity;
	int intensity_max;
	int status;
	int dim_status;//For dimming in ALC Mode
	int hw_dimming_enable;
	int lsensor_enable;
	int lsensor_poll_time;
	int version;
	struct mutex lock;
	struct mutex cmd_lock;
	struct aat2870_cmds_t cmds;
	struct delayed_work delayed_work_bl;
	struct input_dev *input_dev;
	unsigned char power_onoff_ref;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend    early_suspend;
#endif

};


static struct aat2870_drvdata_t *drvdata; 

//static spinlock_t intensity_lock; //km.lee


static NvU8 BACKLIGHT_DEFAULT = 0x0B; //101017,  set the value to the current consumption '9.9mA'


NvBool IsReadThreadStart = NV_TRUE;


struct AAT2870_initial_ALC_cur {
	NvU8 reg;
	NvU8 val;
};



#define BL_FM_REG_FMT 2
#define BL_FM_REG_EN_FM 1
#define BL_FM_REG_INIT_FM 0
#define BL_FM_REG_FADEIN_EN_VAL (((BL_FADE_IN_DELAY == 1000? 0: \
							BL_FADE_IN_DELAY == 800?    1: \
							BL_FADE_IN_DELAY == 600?    2: \
					 		BL_FADE_IN_DELAY == 400?    3: 3) << BL_FM_REG_FMT) | \
					   		(1 << BL_FM_REG_INIT_FM))
#define BL_FM_REG_FADEOUT_EN_VAL (((BL_FADE_OUT_DELAY == 1000? 0: \
							BL_FADE_IN_DELAY == 800?    1: \
							BL_FADE_IN_DELAY == 600?    2: \
					 		BL_FADE_IN_DELAY == 400?    3: 3) << BL_FM_REG_FMT) | \
					   		(0 << BL_FM_REG_INIT_FM))
static struct aat2870_ctl_tbl_t aat2870bl_fade_in_tbl[] = {
	{ 0x0c, 0x02 }, //Fade, Disabled
	{ 0x01, 0x00 }, 
	{ 0x00, 0xff },	//LED turn on
	//{ 0x0c, 0x09 }, //Fade In, Enabled
	{ 0x0c, BL_FM_REG_FADEIN_EN_VAL }, //Fade In, Enabled
	{ 0xFF, 0xFE },  /* end of command */		
};


static struct aat2870_ctl_tbl_t aat2870bl_fade_out_tbl[] = {
	{ 0x0B, 0x01 }, 
	{ 0x0C, BL_FM_REG_FADEOUT_EN_VAL },  /* FMT=0.6s, DISABLE_FADE_MAIN=0, FADE_MAIN=fade out */
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
#if 1
	{ 0x00, 0xFF },  /* Channel Enable=ALL */
	{ 0x0E, 0x26 },  /* SNSR_LIN_LOG=linear, ALSOUT_LIN_LOG=linear, RSET=1k~4k,
	                               * GAIN=low, GM=auto gain, ALS_EN=off */
	{ 0x0F, 0x06 },  /* SBIAS=2.6V, SBIAS=off */
	{ 0xFF, 0xFE }	 /* end of command */
#else
	{ 0x0E, 0x00 },
	{ 0x01, 0x11 },
	{ 0x00, 0xFF },
	{ 0xFF, 0xFE }
#endif
};

/* Set to ALC mode HW-high gain mode*/
static struct aat2870_ctl_tbl_t aat2870bl_alc_tbl[] = {
    /* ALC table 0~15 20101218 tunning ver. */
    {0x12,0x19},  /* ALS current setting 5.6mA */
    {0x13,0x20},  /* ALS current setting 7.2mA */
    {0x14,0x21},  /* ALS current setting 7.4mA */
    {0x15,0x23},  /* ALS current setting 7.9mA */
    {0x16,0x24},  /* ALS current setting 8.1mA */
    {0x17,0x25},  /* ALS current setting 8.3mA */
    {0x18,0x27},  /* ALS current setting 9.0mA */
    {0x19,0x28},  /* ALS current setting 9.5mA */
    {0x1A,0x29},  /* ALS current setting 10.1mA */
    {0x1B,0x2A},  /* ALS current setting 10.8mA */
    {0x1C,0x2F},  /* ALS current setting 11.5mA */
    {0x1D,0x30},  /* ALS current setting 12.2mA */
    {0x1E,0x32},  /* ALS current setting 12.8mA */
    {0x1F,0x35},  /* ALS current setting 13.5mA */
    {0x20,0x36},  /* ALS current setting 14.2mA */
    {0x21,0x37},  /* ALS current setting 14.6mA */

    { 0x0E, 0x73 },  /* SNSR_LIN_LOG=linear, ALSOUT_LIN_LOG=log, RSET=16k~64k,
                                   * GAIN=low, GM=man gain, ALS_EN=on */
    { 0x0F, 0x01 },  /* SBIAS=3.0V, SBIAS=on */
    { 0x10, 0x90 },  /* pwm inactive, auto polling, 1sec, +0% */
    { 0x00, 0xFF },  /* Channel Enable : ALL */
    { 0xFF, 0xFE }   /* end or command */
};


static struct aat2870_lux_tbl_t  aat2870_lux_tbl[] = {

#ifdef BL_ALS_LEV_TO_LUX_TBL_APPROXIMATION
	//{0x00,	10},
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
#else
	{0x00,	10},
	{0x01,	15},
	{0x02,	21},
	{0x03,	31},
	{0x04,	46},
	{0x05,	67},
	{0x06,	98},
	{0x07,	143},
	{0x08,	209},
	{0x09,	306},
	{0x0a,	448},
	{0x0b,	655},
	{0x0c,	959},
	{0x0d,	1402},
	{0x0e,	2051},
	{0x0f,	3000}, //lev 15
#endif
	{0x1f,	0x00},
	{0x20,	0x00}
};


static NvOdmServicesGpioHandle	hBLResetGpio = 0;
static NvOdmGpioPinHandle hBLResetGpioPin = 0;


static unsigned char star_bl_read(unsigned char reg_addr);
static int star_bl_write(unsigned char reg_addr, unsigned char reg_data);
static int star_bl_io_init(void);
static void star_aat2870_reset(void);

static int
star_bl_send_cmd(struct aat2870_drvdata_t *drv, struct aat2870_ctl_tbl_t *tbl);


static unsigned int star_bl_conv_to_lux(int lev)
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


static void star_bl_work_func(struct work_struct *wq)
{
	struct aat2870_drvdata_t *drv;
	int alc_level = 0;


	drv = drvdata;
	if ((drv->op_mode == AAT2870_OP_MODE_ALC) && drv->lsensor_enable) {

		alc_level = (star_bl_read(AAT2870_REG_AMB) >> 3);

		input_report_abs(drv->input_dev, EVENT_TYPE_LIGHT, star_bl_conv_to_lux(alc_level));
		input_sync(drv->input_dev);
		DBG("[BL] %s()\n", __FUNCTION__);
		schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
	}
}


static void star_bl_switch_mode(int op_mode)
{
	struct aat2870_drvdata_t *drv;


	drv = drvdata;
	if (drv->dim_status != DIMMING_NONE) {
		star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
		drv->dim_status = DIMMING_NONE;
		DBG("[BL] DIMMING_OFF \n");
	}

	if (op_mode == AAT2870_OP_MODE_NORMAL) {

		star_bl_send_cmd(drv, drv->cmds.normal);
		DBG("[BL] Mode: normal\n");
	} else if (op_mode == AAT2870_OP_MODE_ALC) {

		star_bl_send_cmd(drv, drv->cmds.alc);
		//drv->dim_status = DIMMING_NONE;
		DBG("[BL] Mode: alc\n");
		if (drv->lsensor_enable == TRUE) {
			schedule_delayed_work(&drv->delayed_work_bl, drv->lsensor_poll_time);
		}
	} else {
	}

	if (drv->status == BL_POWER_STATE_OFF) {

		DBG("[BL] Weird: how to use without display\n");
		drv->status = BL_POWER_STATE_ON;
	}

	return;
}


static int
star_bl_send_cmd(struct aat2870_drvdata_t *drv, struct aat2870_ctl_tbl_t *tbl)
{
	unsigned long delay = 0;

	if (tbl == NULL) {
		printk("input ptr is null\n");
		return -EIO;
	}

	mutex_lock(&drvdata->cmd_lock);
	for( ;;) {
		if (tbl->reg == 0xFF) {
			if (tbl->val != 0xFE) {
				delay = (unsigned long)tbl->val;
				//udelay(delay);
			}
			else
				break;
		}	
		else {
			if (star_bl_write(tbl->reg, tbl->val) != 0)
				printk("i2c failed addr:%d, value:%d\n", tbl->reg, tbl->val);
		}
		tbl++;
	}
	mutex_unlock(&drvdata->cmd_lock);

	return 0;
}


static int star_bl_io_init()
{
	int retval = 0;


	hStarI2csim->sclport = SCL_PORT;
	hStarI2csim->sclpin = SCL_PIN;
	
	hStarI2csim->sdaport = SDA_PORT;
	hStarI2csim->sdapin = SDA_PIN;


	if (hStarI2csim->hI2cServiceGpioHandle) {

		retval = -EBUSY;
		goto exit;
	}

	hStarI2csim->hI2cServiceGpioHandle = (NvOdmServicesGpioHandle) NvOdmGpioOpen();
	if (!(hStarI2csim->hI2cServiceGpioHandle)) {

		retval = -EBUSY;
		goto exit;
	}

	hStarI2csim->hSclGpioPinHandle = NvOdmGpioAcquirePinHandle(hStarI2csim->hI2cServiceGpioHandle, 
            hStarI2csim->sclport,
            hStarI2csim->sclpin);
	if (!(hStarI2csim->hSclGpioPinHandle)) {

		printk("[doncopy] : hSclGpioPinHandle error  \n");
		retval = -EBUSY;
		goto exit;
	}
	 
	hStarI2csim->hSdaGpioPinHandle = NvOdmGpioAcquirePinHandle(hStarI2csim->hI2cServiceGpioHandle,
            hStarI2csim->sdaport,
            hStarI2csim->sdapin);
	
	if (!(hStarI2csim->hSdaGpioPinHandle)) {

		printk("[doncopy] : hSdaGpioPinHandle error  \n");
		retval = -EBUSY;
		goto exit;
	}

	g_hdelay = HALF_DELAY;
	g_fdelay = FULL_DELAY;
	
	I2C_SDA_HI;
	I2C_SCL_HI;
	I2C_SDA_DIR(DIR_OUT);
	I2C_SCL_DIR(DIR_OUT);

	return retval;

exit:
	return retval;
}


static int star_bl_io_deinit(void)
{
	int retval = 0;

      	I2C_SDA_LO;
	I2C_SCL_LO;
	I2C_SDA_DIR(DIR_OUT);
	I2C_SCL_DIR(DIR_OUT);

	if (hStarI2csim->hSdaGpioPinHandle) NvOdmGpioReleasePinHandle(hStarI2csim->hI2cServiceGpioHandle, hStarI2csim->hSdaGpioPinHandle);
	if (hStarI2csim->hSclGpioPinHandle) NvOdmGpioReleasePinHandle(hStarI2csim->hI2cServiceGpioHandle, hStarI2csim->hSclGpioPinHandle);
	if (hStarI2csim->hI2cServiceGpioHandle) NvOdmGpioClose(hStarI2csim->hI2cServiceGpioHandle);

	hStarI2csim->hI2cServiceGpioHandle = 0;
	hStarI2csim->hSclGpioPinHandle = 0;
	hStarI2csim->hSdaGpioPinHandle = 0;

	return retval;
}

	
static int star_bl_write(unsigned char reg_addr, unsigned char reg_data)
{
	unsigned char data[2]={0,};
	int ret = -1;
	data[0] = reg_addr;
	data[1] = reg_data;


	mutex_lock(&drvdata->lock);
	ret = simi2c_write(SLAVE_ADDR,data,2,1);
	mutex_unlock(&drvdata->lock);
	if (ret < 0)
		printk("Error: simi2c_write in %s() %d\n", __FUNCTION__, __LINE__);

	return ret;
}


static unsigned char star_bl_read(unsigned char reg_addr)
{
	int ret = -1;
	unsigned char data = 0;


	mutex_lock(&drvdata->lock);
#ifdef NEW_GPIO_I2C_READ
	simi2c_read(reg_addr, &data, 1, 1);//10bytes read
#else //old
	if ((ret = simi2c_write(SLAVE_ADDR,&reg_addr, 1, 0)) < 0) {

		printk("i2c write error\n");
		return ret;
	}

	//simi2c_read(SLAVE_ADDR, &data, 1, 1);
	simi2c_read(reg_addr, &data, 1, 1);//10bytes read
#endif //BL_READ_NEW
	mutex_unlock(&drvdata->lock);

	return data;
}


static ssize_t
star_bl_show_intensity(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int ret;


	drv = drvdata;
	if (!drv) return 0;

	ret = sprintf(buf, "%d\n", drv->intensity);
	
	return ret;
}


static int
star_bl_brightness_linearized(int intensity, int *level)
{
	int ret = 0;
	int remainder;
    int last_intensity; 


	//101017,  Set the Backlight Brightness to be linearized.[START] 
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
	//101017,  Set the Backlight Brightness to be linearized.[END]

	return ret;
}  


static ssize_t
star_bl_store_intensity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{    
	int level, intensity;
	static struct aat2870_drvdata_t *drv;

	
	drv = drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &intensity);//level range: 0 to 22 from aat2870 ds

	//101103, , Replaced with function.
	if (star_bl_brightness_linearized(intensity, &level)) {

		printk("[BL] Invalid Intensity value: %d\n", intensity);
		goto intensity_input_err;
	}

	//if (drv->op_mode == AAT2870_OP_MODE_ALC && drv->hw_dimming_enable) {
	if (drv->hw_dimming_enable) {

		if (drv->dim_status == DIMMING_NONE && 
			drv->intensity > level && intensity == 20) {

			DBG("[BL] DIMMING_START \n");
			star_bl_send_cmd(drv, aat2870bl_fade_out_tbl);
			drv->dim_status = DIMMING_START;
			mdelay(BL_FADE_OUT_DELAY);
		} else if (drv->dim_status != DIMMING_NONE && drv->intensity < level) {

			star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			//star_bl_send_cmd(drv, drv->cmds.alc);//'Power On' will do it.
			drv->dim_status = DIMMING_NONE;
			DBG("[BL] DIMMING_OFF \n");
		}
	}

	if (drv->power_onoff_ref == TRUE) {

		star_bl_send_cmd(drv, aat2870bl_fade_in_tbl);
		if (drv->op_mode != AAT2870_OP_MODE_ALC) {
			star_bl_write(AAT2870_REG_BLM, level);
		} else {
			//star_bl_write(AAT2870_REG_BLM, 0x0a);//median
			star_bl_write(AAT2870_REG_BLM, 0x06);//lowest
		}
		mdelay(BL_FADE_IN_DELAY - 50);//Fade-In delay
		star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
		drv->power_onoff_ref = FALSE;
	}

	if (level > drv->intensity_max) {
		level = drv->intensity_max;
	}
	DBG("[BL] intensity: %d, level: %d(prev: %d)\n",intensity, level, drv->intensity);
	//spin_lock( &intensity_lock ); //km.lee
	//if (drv->intensity != level) { //Deep sleep Issue: need to be updated everytime
	star_bl_write(AAT2870_REG_BLM, level);
	drv->intensity = level;
	//}
	//spin_unlock( &intensity_lock ); //km.lee
	
intensity_input_err:

	return count;
}


static ssize_t
star_bl_show_alc_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int alc_level = 0;
	int ret;


	drv = drvdata;
	if (!drvdata)
		return 0;

	if (drv->op_mode == AAT2870_OP_MODE_NORMAL) {

		alc_level = -1;
	} else if (drv->op_mode == AAT2870_OP_MODE_ALC) {

		alc_level = (star_bl_read(AAT2870_REG_AMB) >> 3);
	} else {

	}
	ret = sprintf(buf, "%d\n", star_bl_conv_to_lux(alc_level));

	return ret;
}


static ssize_t 
star_bl_show_hwdim(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int r;


	drv = drvdata;
	if (!drv) return 0;

	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->hw_dimming_enable == TRUE)  ? "1":"0");
	
	return r;
}


static ssize_t 
star_bl_store_hwdim(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int dimming;
	static struct aat2870_drvdata_t *drv;
	
	
	drv = drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &dimming);

	if (drv->dim_status != DIMMING_NONE) {
		star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
		drv->dim_status = DIMMING_NONE;
		DBG("[BL] DIMMING_OFF \n");
	}

	if (dimming) {

		drv->hw_dimming_enable = TRUE;
	} else {

		drv->hw_dimming_enable = FALSE;
	}

	return count;
}


static ssize_t 
star_bl_show_lsensor_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int ret;


	drv = drvdata;
	if (!drv) return 0;

	ret = snprintf(buf, PAGE_SIZE, "%s\n", (drv->lsensor_enable == TRUE)  ? "1":"0");
	
	return ret;
}


static ssize_t 
star_bl_store_lsensor_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	static struct aat2870_drvdata_t *drv;
	
	
	drv = drvdata;
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


static ssize_t 
star_bl_show_alc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int r;


	drv = drvdata;

	if (!drv) return 0;

	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->op_mode == AAT2870_OP_MODE_ALC)  ? "1":"0");
	
	return r;
}


static ssize_t 
star_bl_store_alc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int alc;
	int next_mode;
	static struct aat2870_drvdata_t *drv;
	
	
	drv = drvdata;

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
	//printk("[KERNEL] alc = %d  %s() %d\n",alc , __FUNCTION__, __LINE__);
	star_bl_switch_mode(next_mode);
	drv->op_mode = next_mode;

	return count;
}


static ssize_t
star_bl_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aat2870_drvdata_t *drv;
	int r;


	drv = drvdata;
	if (!drv) return 0;

	r = snprintf(buf, PAGE_SIZE, "%s\n", (drv->status == BL_POWER_STATE_ON)  ? "1":"0");
	
	return r;
}


static ssize_t
star_bl_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	static struct aat2870_drvdata_t *drv;
	
	
	drv = drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &onoff);

	if (onoff && drv->status == BL_POWER_STATE_OFF) {

		if(drv->dim_status != DIMMING_NONE)	{

			star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			DBG("[BL] DIMMING_OFF \n");
		}

		if (drv->op_mode == AAT2870_OP_MODE_NORMAL) {

			star_bl_send_cmd(drv, drv->cmds.normal);
		} else if (drv->op_mode == AAT2870_OP_MODE_ALC) {

			star_bl_send_cmd(drv, drv->cmds.alc);
		} else {
		}
		drv->status = BL_POWER_STATE_ON;
		drv->power_onoff_ref = TRUE;
		printk("[BL] Power On\n");
	} else if (!onoff && drv->status == BL_POWER_STATE_ON) {

		if(drv->dim_status != DIMMING_NONE)	{

			star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			DBG("[BL] DIMMING_OFF \n");
		}
		star_bl_send_cmd(drv, drv->cmds.sleep);
		drv->status = BL_POWER_STATE_OFF;
		drv->power_onoff_ref = FALSE;
		printk("[BL] Power Off\n");
	} else {
	}

	return count;
}

//20110202, , force off [START]
static void star_aat2870_reset(void);

static ssize_t
star_bl_show_foff(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t
star_bl_store_foff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int onoff;
	static struct aat2870_drvdata_t *drv;	
	
	drv = drvdata;
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &onoff);
    
    star_bl_send_cmd(drv, drv->cmds.sleep);
	drv->status = BL_POWER_STATE_OFF;
	drv->power_onoff_ref = FALSE;
	printk("[BL] Power Off\n");
    
	star_aat2870_reset();
	printk("[BL] star_aat2870_reset\n");

	return count;
}
//20110202, , force off [END]

static DEVICE_ATTR(intensity, 0666, star_bl_show_intensity, star_bl_store_intensity);
static DEVICE_ATTR(alc_level, 0444, star_bl_show_alc_level, NULL);
static DEVICE_ATTR(alc, 0664, star_bl_show_alc, star_bl_store_alc);
static DEVICE_ATTR(onoff, 0666, star_bl_show_onoff, star_bl_store_onoff);
static DEVICE_ATTR(hwdim, 0666, star_bl_show_hwdim, star_bl_store_hwdim);
static DEVICE_ATTR(lsensor_onoff, 0666, star_bl_show_lsensor_onoff, star_bl_store_lsensor_onoff);
//static DEVICE_ATTR(alc_reg, 0666, alc_reg_show, alc_reg_store);
//20110202, , force off [START]
static DEVICE_ATTR(foff, 0666, star_bl_show_onoff, star_bl_store_foff);
//20110202, , force off [END]


static struct attribute *star_bl_attributes[] = {
	&dev_attr_intensity.attr,
	&dev_attr_alc_level.attr,
	&dev_attr_alc.attr,
	&dev_attr_onoff.attr,
	&dev_attr_hwdim.attr,
	&dev_attr_lsensor_onoff.attr,
    //20110202, , force off [START]
	&dev_attr_foff.attr,
    //20110202, , force off [END]
	NULL,
};


static const struct attribute_group star_bl_group = {
	.attrs = star_bl_attributes,
};


static void star_aat2870_reset_init(void)
{
	NvU32 Port,Pin;


	Port = 'r' - 'a';
	Pin = 3;
	if (!hBLResetGpio)
		hBLResetGpio = NvOdmGpioOpen();

	if(!hBLResetGpioPin) 
		hBLResetGpioPin = NvOdmGpioAcquirePinHandle(hBLResetGpio, Port, Pin);

	if (!hBLResetGpioPin)
		printk("error: %s() %d\n", __FUNCTION__, __LINE__);

	return;
}


static void star_aat2870_reset(void)
{
	if (hBLResetGpio && hBLResetGpioPin) {

		NvOdmGpioSetState( hBLResetGpio, hBLResetGpioPin, 0x0);

		NvOdmGpioConfig( hBLResetGpio, hBLResetGpioPin, NvOdmGpioPinMode_Output);

		NvOdmOsWaitUS(BL_HW_RESET_DELAY);
		NvOdmGpioSetState( hBLResetGpio, hBLResetGpioPin, 0x1);
		NvOdmOsWaitUS(BL_HW_RESET_DELAY);
	}

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void star_aat2870_early_suspend(struct early_suspend *es)
{
	static struct aat2870_drvdata_t *drv;
	
	drv = drvdata;

	if (drv->status == BL_POWER_STATE_ON) {
		if(drv->dim_status != DIMMING_NONE)	{
			star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			DBG("star_aat2870_early_suspend : [BL] DIMMING_OFF \n");
		}
		star_bl_send_cmd(drv, drv->cmds.sleep);
		drv->status = BL_POWER_STATE_OFF;
		drv->power_onoff_ref = FALSE;
		printk("star_aat2870_early_suspend : [BL] Power Off\n");
	}

	return;
}

static void star_aat2870_late_resume(struct early_suspend *es)
{
	static struct aat2870_drvdata_t *drv;
	
	drv = drvdata;

	if (drv->status == BL_POWER_STATE_OFF) {
		if(drv->dim_status != DIMMING_NONE)	{
			star_bl_send_cmd(drv, aat2870bl_stop_fade_tbl);
			drv->dim_status = DIMMING_NONE;
			DBG("star_aat2870_early_suspend : [BL] DIMMING_OFF \n");
		}

		if (drv->op_mode == AAT2870_OP_MODE_NORMAL) {
			star_bl_send_cmd(drv, drv->cmds.normal);
		} else if (drv->op_mode == AAT2870_OP_MODE_ALC) {
			star_bl_send_cmd(drv, drv->cmds.alc);
		} else {
		}
		drv->status = BL_POWER_STATE_ON;
		drv->power_onoff_ref = TRUE;
		printk("star_aat2870_late_resume : [BL] Power On\n");
	}

	return;
}
#endif

static int star_aat2870_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct device *dev = &pdev->dev;
	struct aat2870_drvdata_t *drv;


	drv = kzalloc(sizeof(struct aat2870_drvdata_t), GFP_KERNEL);
	if (drv == NULL) {

		retval = -ENOMEM;
		goto err;
	}

	//intensity_lock = SPIN_LOCK_UNLOCKED; //km.lee
	drvdata = drv;
	
	drv->cmds.normal = aat2870bl_normal_tbl;
	drv->cmds.alc = aat2870bl_alc_tbl;
	drv->cmds.sleep = aat2870bl_sleep_tbl;

	drv->op_mode = AAT2870_OP_MODE_NORMAL;
	drv->dim_status = DIMMING_NONE;
	drv->intensity = BACKLIGHT_DEFAULT;
	drv->intensity_max = BL_INTENSITY_MAX;
	drv->hw_dimming_enable = TRUE;
	drv->lsensor_enable = FALSE;
	drv->lsensor_poll_time = BL_DEFAULT_LSENSOR_POLL_TIME;
	drv->power_onoff_ref = FALSE;

#ifdef CONFIG_HAS_EARLYSUSPEND
    drv->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    drv->early_suspend.suspend = star_aat2870_early_suspend;
    drv->early_suspend.resume = star_aat2870_late_resume;
    register_early_suspend(&drv->early_suspend);
#endif

	drv->input_dev = input_allocate_device();
	if (!drv->input_dev) {

		printk("[BL] err, allocate input device\n");
		retval = -ENOMEM;
		goto err_input_device;
	}

	drv->input_dev->name = STAR_AMBIENT_LIGHT_SENSOR_DRV_NAME;
	set_bit(EV_SYN, drv->input_dev->evbit);
	set_bit(EV_ABS, drv->input_dev->evbit);
	input_set_abs_params(drv->input_dev, EVENT_TYPE_LIGHT, 0, 3000, 0, 0);

	if (input_register_device(drv->input_dev)) {

		printk("[BL] err, register input device\n");
		retval = -ENODEV;
		goto err_input_device;
	}
	 
	hStarI2csim = NvOdmOsAlloc(sizeof(StarI2csim));
	if (!hStarI2csim) {

		retval = -ENOMEM;
		goto err;
	}
	NvOdmOsMemset(hStarI2csim, 0, sizeof(StarI2csim));

	star_aat2870_reset_init();
	star_aat2870_reset();

	star_bl_io_init();
		
	if (sysfs_create_group(&dev->kobj, &star_bl_group)) {

		printk("[BL] Failed to create sys filesystem\n");
		retval = -ENOSYS;
		goto err;
	}

	mutex_init(&drv->lock);
	mutex_init(&drv->cmd_lock);


	star_bl_write(AAT2870_REG_BLM, drv->intensity);//default intensity
	star_bl_send_cmd(drv, drv->cmds.normal);

	INIT_DELAYED_WORK(&drv->delayed_work_bl, star_bl_work_func);
	//schedule_delayed_work(&drvdata->delayed_work_bl, 100);
   
    return 0;


err_input_device:

err:
	return retval; 
}


static void star_aat2870_shutdown(struct platform_device *pdev)
{

	struct aat2870_drvdata_t *drv;
        printk("star_att2870_shutdown\n"); 

	drv = drvdata;
#if 0
    if (&drv->delayed_work_bl)
	cancel_delayed_work_sync(&drv->delayed_work_bl);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&drv->early_suspend);
#endif

	NvOdmGpioSetState( hBLResetGpio, hBLResetGpioPin, 0x0);
	NvOdmGpioConfig( hBLResetGpio, hBLResetGpioPin,5);

	if (hBLResetGpioPin) NvOdmGpioReleasePinHandle(hBLResetGpio, hBLResetGpioPin);
	if (hBLResetGpio) NvOdmGpioClose(hBLResetGpio);

	star_bl_io_deinit();
}


static struct platform_device star_aat2870_device = {
	.name = "star_aat2870",
	.id = 0, 
};


static struct platform_driver star_aat2870_driver = {
    .probe		= star_aat2870_probe,
    .shutdown		= star_aat2870_shutdown,
    .driver		= {
        .name = "star_aat2870",
        .owner = THIS_MODULE,
    },
};


static int __init star_aat2870_init(void)    
{
    int retval = 0;
	
	  
    retval = platform_device_register(&star_aat2870_device);
    if (retval < 0) {

		printk(KERN_ERR "platform_device_register failed!\n");
		goto out;
	}

    retval =  platform_driver_register(&star_aat2870_driver);  
    if (retval < 0) {

		printk(KERN_ERR "platform_driver_register failed!\n");
		goto out;
	}

out:
	return retval;
}  


static void __exit star_aat2870_exit(void)
{ 
	star_bl_io_deinit();//Release Handle

	return platform_driver_unregister(&star_aat2870_driver);
}


module_init(star_aat2870_init);
module_exit(star_aat2870_exit);

MODULE_LICENSE("Dual BSD/GPL");


