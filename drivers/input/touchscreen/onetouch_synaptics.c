/* drivers/input/touchscreen/onetouch_synaptics.c
 *
 * Copyright (C) 2012 LGE. 
 * 
 * version 1.0
 * 
 * Writer: jingyeong.noh@lge.com
 * This device driver is written for only STAR_ICS - synaptics_onetouch
 *
 * Supports:
 *	1. early-suspend / delayed-resume <optional>
 *	2. only IRQ-mode (not polling-mode)
 *	3. 2-keys(menu,back)
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
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/onetouch_synaptics.h>
#include <mach/gpio.h>

#ifdef STAR_TIME_CALC
#include <linux/time.h>
struct timeval tv;
long t_start, t_end;
#endif

#define SYNAPTICS_I2C_RETRY_COUNT 3

// OneTouch Register Definitions
#define OT_CONFIG_REG_START_ADDR		0x0000
#define OT_CONFIG_REG_END_ADDR			0x0025
#define OT_NUM_CONFIG_REG_BYTES			((OT_CONFIG_REG_END_ADDR - OT_CONFIG_REG_START_ADDR + 1)*2)

// Total number of Configuration Bytes
#define OT_NUM_CONFIG_BYTES			(OT_NUM_CONFIG_REG_BYTES + 2)

#define		OT_DATA_REG_START_ADDR			0x0109
#define		OT_NUM_DATA_REG_BYTES			4		// (0x109 - 0x10A)*2

#define		OT_DATA_REG_START_ADDR_HIGH		0x01
#define		OT_DATA_REG_START_ADDR_LOW		0x09

#define		OT_DATA_REG_SLEEP_ADDR_HIGH		0x00
#define		OT_DATA_REG_SLEEP_ADDR_LOW		0x01

#define		OT_DATA_REG_SLEEP_DATA_HIGH		0x00
#define		OT_DATA_REG_SLEEP_DATA_LOW		0x80

#define		OT_DATA_REG_ACTIVE_DATA_HIGH		0x00
#define		OT_DATA_REG_ACTIVE_DATA_LOW		0x00

#define		OT_BUTTON_OFFSET				0

static u16	g_OT_StartAddr = 0x0901;

static bool	pressed_back, pressed_menu;

extern bool is_star_touch_enable;

static u8	g_OT_Config[OT_NUM_CONFIG_BYTES]= {
	// High byte, followed by low Byte
	0x00, 0x00,		// base address of config registers
	0x00, 0x07,		// values of 0x0000
	0x00, 0x00,		// values of 0x0001
	0x00, 0x00,		// values of 0x0002
	0x00, 0x00,		// values of 0x0003
	0x00, 0x09,		// values of 0x0004
	0x00, 0x00,		// values of 0x0005
	0x00, 0x00,		// values of 0x0006
	0x00, 0x00,		// values of 0x0007
	0x00, 0x00,		// values of 0x0008
	0x00, 0x00,		// values of 0x0009
	0x00, 0x00,		// values of 0x000A
	0x00, 0x00,		// values of 0x000B
	0x00, 0x00,		// values of 0x000C
	0x00, 0x00,		// values of 0x000D
	0x00, 0x00,		// values of 0x000E
	0x00, 0x00,		// values of 0x000F
	0x00, 0xFF,		// values of 0x0010
	0xFF, 0x00,		// values of 0x0011
	0x00, 0x00,		// values of 0x0012
	0x00, 0x00,		// values of 0x0013
	0x00, 0x00,		// values of 0x0014
	0x00, 0x00,		// values of 0x0015
	0x00, 0x00,		// values of 0x0016
	0x00, 0x00,		// values of 0x0017
	0x00, 0x00,		// values of 0x0018
	0x00, 0x00,		// values of 0x0019
	0x00, 0x00,		// values of 0x001A
	0x00, 0x00,		// values of 0x001B
	0x00, 0x00,		// values of 0x001C
	0x00, 0x00,		// values of 0x001D
	0x00, 0x00,		// values of 0x001E
	0x00, 0x00,		// values of 0x001F
	0x00, 0x00,		// values of 0x0020
	0x00, 0x00,		// values of 0x0021
	0x00, 0x00,		// values of 0x0022
	0x00, 0x00,		// values of 0x0023
	0x00, 0x00,		// values of 0x0024
	0x00, 0x00		// values of 0x0025
};

#define SYNAPTICS_TOUCH_DEBUG_LEVEL 3

#ifdef SYNAPTICS_TOUCH_DEBUG_LEVEL
#define DEBUG_MSG(_level, args...)  \
		if(_level >= SYNAPTICS_TOUCH_DEBUG_LEVEL)	\
			printk(args);
#else
#define DEBUG_MSG(_level, args...)
#endif

#define DO_SAFE(_statement, _return, args...)	\
if(unlikely(_statement)){	\
	DEBUG_MSG(3,args);	\
	return _return;		\
}

struct onetouch_synaptics_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct work_struct  work;
	int use_irq;
	u16 irq_gpio;
	uint32_t flags;
	int (*power)(char* reg_id, int on);
	struct early_suspend early_suspend;
	struct mutex	mutex;
};

static struct workqueue_struct *onetouch_synaptics_wq;

static bool init_setting_onetouch_device(struct i2c_client *client);

/* POWER MANAGEMENT
  *
  * When Power is turned on, device needs time to reset itself.
  * WAIT_TOUCH_POWER_READY waits until device returns ACK (ACK means that device is ready to work)
  */

static int power_control(struct onetouch_synaptics_ts_data *ts, bool on_off)
{
    DO_SAFE(ts->power("vdd_onetouch", on_off) < 0, -1, "[TOUCH] Power Control Error!\n")
    DO_SAFE(ts->power("vcc_touch_3v1", on_off) < 0, -1, "[TOUCH] Power Control Error!\n");    

	if(on_off)
		msleep(300);

	DO_SAFE(ts->power("vcc_touch_1v8", on_off) < 0, -1, "[TOUCH] Power Control Error!\n");    

    if(on_off == true && is_star_touch_enable == false)   
        msleep(400);

	is_star_touch_enable = on_off;

	return 0;
}

static bool onetouch_synaptics_read_reg(struct i2c_client *client, char *buffer, int len)
{
	int i, ret = 0;

	for(i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && ret != len ; i++)
	{
		ret =  i2c_master_recv(client, (char *)buffer, len);
	}

	if(ret != len)
{
	struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);

		DEBUG_MSG(2, "[ONETOUCH] I2C Read Failure = %d \n", ret);

		disable_irq_nosync(382);

	if (ts->power) {
		DO_SAFE(power_control(ts, false) < 0, -1, "[TOUCH] Power Control [ON] Error!\n")
	}
	
    	tegra_gpio_disable(ts->irq_gpio);
		msleep(20);

		if (ts->power) {
			DO_SAFE(power_control(ts, true) < 0, -1, "[TOUCH] Power Control [OFF] Error!\n")
		}

		init_setting_onetouch_device(ts->client);

		if (ts->use_irq)
			enable_irq(client->irq);

		enable_irq(382);
		return false;
	}
	return true;
}

static bool onetouch_synaptics_write_reg(struct i2c_client *client, char *buffer, int len)
{
	int i, ret = 0;

	for(i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && ret != len ; i++)
	{
		ret =  i2c_master_send(client, (char *)buffer, len);
	}

	if(ret != len)
	{
		struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);

		DEBUG_MSG(2, "[ONETOUCH] I2C Write Failure = %d \n", ret);
		disable_irq_nosync(382);

		if (ts->power) {
			DO_SAFE(power_control(ts, false) < 0, -1, "[TOUCH] Power Control [ON] Error!\n")
		}
		
		tegra_gpio_disable(ts->irq_gpio);
		msleep(20);

		if (ts->power) {
			DO_SAFE(power_control(ts, true) < 0, -1, "[TOUCH] Power Control [OFF] Error!\n")
		}

		init_setting_onetouch_device(ts->client);

		if (ts->use_irq)
			enable_irq(client->irq);

		enable_irq(382);
		return false;
	}
	return true;
}

static bool onetouch_synaptics_sleep_mode(struct i2c_client *client, bool onoff)
{
	u8 mode_buf[4];

	mode_buf[0] = OT_DATA_REG_SLEEP_ADDR_HIGH;
	mode_buf[1] = OT_DATA_REG_SLEEP_ADDR_LOW;

	if(onoff)
	{
		mode_buf[2] = OT_DATA_REG_SLEEP_DATA_HIGH;
		mode_buf[3] = OT_DATA_REG_SLEEP_DATA_LOW;
	}
	else
	{
		mode_buf[2] = OT_DATA_REG_ACTIVE_DATA_HIGH;
		mode_buf[3] = OT_DATA_REG_ACTIVE_DATA_LOW;
	}
	
	onetouch_synaptics_write_reg(client, (char *)&mode_buf, sizeof(mode_buf));

    	onetouch_synaptics_write_reg(client, (char *)&g_OT_StartAddr, 2);
}

static bool init_setting_onetouch_device(struct i2c_client *client)
{
    struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);
	u8 buffer[OT_NUM_CONFIG_BYTES];
	int i;

	memset(buffer, 0, sizeof(buffer));

    // Write the configuration to the OneTouch
	onetouch_synaptics_write_reg(client, (char *)&g_OT_Config, OT_NUM_CONFIG_BYTES);

    // Read the entire configuration back from the device
	onetouch_synaptics_read_reg(client, (char *)&buffer, OT_NUM_CONFIG_BYTES-2);

    // Verify the configuration registers are written correctly
    for( i = 0; i < OT_NUM_CONFIG_BYTES-2; i++)
    {
		if(buffer[i] != g_OT_Config[i+2])
		{
			DEBUG_MSG(3, "[ONETOUCH] Init Failure \n");
	    return false;
    }
	}

	// set base address
	onetouch_synaptics_write_reg(client, (char *)&g_OT_StartAddr, 2);

	// read the data register to deassert the attention line
	onetouch_synaptics_read_reg(client, (char *)&buffer, 1);

	tegra_gpio_enable(ts->irq_gpio);
    return true;
}

static int onetouch_synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);

	DEBUG_MSG(2, "[ONETOUCH] ts_suspend() called %s \n", __func__);

	disable_irq(client->irq);

/*
	if (ts->power) {
		DO_SAFE(power_control(ts, false) < 0, -1, "[TOUCH] Power Control [ON] Error!\n")
	}
	
    	tegra_gpio_disable(ts->irq_gpio);
*/
	onetouch_synaptics_sleep_mode(client, true);
	return 0;
}

static int onetouch_synaptics_ts_resume(struct i2c_client *client)
{
	struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);

	DEBUG_MSG(2, "[ONETOUCH] ts_resume() called %s \n", __func__);
      
	onetouch_synaptics_sleep_mode(client, false);
/*      
	if (ts->power) {
		DO_SAFE(power_control(ts, true) < 0, -1, "[TOUCH] Power Control [OFF] Error!\n")
	}

	init_setting_onetouch_device(ts->client);
*/
	if (ts->use_irq)
		enable_irq(client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void onetouch_synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct onetouch_synaptics_ts_data *ts;
	ts = container_of(h, struct onetouch_synaptics_ts_data, early_suspend);
	onetouch_synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void onetouch_synaptics_ts_late_resume(struct early_suspend *h)
{
	struct onetouch_synaptics_ts_data *ts;

	DEBUG_MSG(2, "[ONETOUCH] ts_late_resume() called %s \n", __func__);
    
	ts = container_of(h, struct onetouch_synaptics_ts_data, early_suspend);
	onetouch_synaptics_ts_resume(ts->client);
}
#endif

static void onetouch_synaptics_ts_work_func(struct work_struct *work)
{
	struct onetouch_synaptics_ts_data *ts = container_of(work, struct onetouch_synaptics_ts_data, work);
	bool ret =0;
	u16 nRev = 0;

	mutex_lock(&ts->mutex);
	DEBUG_MSG(2, "[ONETOUCH] ***  ONE TOUCH WORK FUNC \n");

	if(!onetouch_synaptics_write_reg(ts->client, (char *)&g_OT_StartAddr, 2))
	{
		mutex_unlock(&ts->mutex);
		return;
	}
	if(!onetouch_synaptics_read_reg(ts->client, (char *)&nRev, 2))
	{
		mutex_unlock(&ts->mutex);
		return;
	}
	DEBUG_MSG(2, "[ONETOUCH] ***  ONE TOUCH WORK FUNC 0x[%x] Pre Menu[%d] Back[%d] \n", nRev, pressed_menu, pressed_back);

	if((nRev >> 8)& 0x8)
	{
	    if(!pressed_back)
	    {
		input_report_key(ts->input_dev, KEY_BACK, 1);
		pressed_back = true;
	    }
	}
	else if((nRev >> 8) & 0x1)
	{
//MOBII_CHANGE_S 20120424 : func position changed. and original func removed.
	    //input_report_key(ts->input_dev, KEY_MENU, 1);
	    if(!pressed_menu)
	    {
		//input_report_key(ts->input_dev, KEY_BACK, 1);
		input_report_key(ts->input_dev, KEY_MENU, 1);
//MOBII_CHANGE_E 20120424 : func position changed. and original func removed.
		pressed_menu = true;
	    }
	}
	else
	{
	    if(pressed_menu)
		input_report_key(ts->input_dev, KEY_MENU, 0);
	    else if(pressed_back)
		input_report_key(ts->input_dev, KEY_BACK, 0);

	    pressed_menu = pressed_back = false;
	}

	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);

#ifdef STAR_TIME_CALC
	do_gettimeofday(&tv);
	t_end = tv.tv_usec;
#endif

	enable_irq(ts->client->irq);
	mutex_unlock(&ts->mutex);
}


static irqreturn_t onetouch_synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct onetouch_synaptics_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);

#ifdef STAR_TIME_CALC
	do_gettimeofday(&tv);
	t_start = tv.tv_usec;
#endif
	
	DEBUG_MSG(2, "[ONETOUCH] %s \n", __func__);
	queue_work(onetouch_synaptics_wq, &ts->work);

	return IRQ_HANDLED;
}

static int __init onetouch_synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct onetouch_synaptics_ts_data *ts;
	int ret = 0;
	struct star_onetouch_synaptics_platform_data *pdata;
	unsigned long irqflags = 0;

	DEBUG_MSG(2, "[ONETOUCH] %s \n", __func__);
	// 1. check support of I2C
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_MSG(2, "[ONETOUCH] probe: need I2C_FUNC_I2C, I2C_FUNC_SMBUS_BYTE_DATA\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// 2. get data from board_star.c
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		DEBUG_MSG(2, "[ONETOUCH] probe: ts == NULL\n");
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	pdata = client->dev.platform_data;
	if (pdata) {
		irqflags = pdata->irqflags;
		ts->power = pdata->power;
		ts->irq_gpio = pdata->gpio;
		if (ts->power) {
			DEBUG_MSG(2, "[ONETOUCH] onetouch_synaptics_ts_probe power on start\n");
			ret = power_control(ts, true);
			if (ret < 0) {
				DEBUG_MSG(2, "[ONETOUCH] onetouch_synaptics_ts_probe power on failed\n");
				goto	err_power_failed;
			}
		}
	}

	mutex_init(&ts->mutex);

	// 4. allocate method that will be used to handle irq_request.
	INIT_WORK(&ts->work, onetouch_synaptics_ts_work_func);

	// 5. set events.
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		DEBUG_MSG(2, "[ONETOUCH] onetouch_synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = ONETOUCH_SYNAPTICS_NAME;

	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		DEBUG_MSG(2, "[ONETOUCH] onetouch_synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	// 6. initialize touch-device
	if(init_setting_onetouch_device(ts->client) != true)
	{
	    goto err_device_init;
	}

	// 7. request_irq
	if (client->irq) {
		DEBUG_MSG(2, "[ONETOUCH] probe: %s,%d -- irq : %d\n", __FUNCTION__, __LINE__, client->irq);
		ret = request_irq(client->irq, onetouch_synaptics_ts_irq_handler, irqflags, client->name, ts);
		if (ret == 0){
			ts->use_irq	=	1;
		} else{
			DEBUG_MSG(2, "[ONETOUCH] probe: request_irq failed\n");
			goto err_irq_request;
		}
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;		//Mobii_Change sgkim@mobii.co.kr 20120608- Speed up LCD On : remove [+1]
	ts->early_suspend.suspend = onetouch_synaptics_ts_early_suspend;
	ts->early_suspend.resume = onetouch_synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	return 0;

err_device_init:
err_irq_request:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int onetouch_synaptics_ts_remove(struct i2c_client *client)
{
	struct onetouch_synaptics_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id onetouch_synaptics_ts_id[] = {
	{ ONETOUCH_SYNAPTICS_NAME, 0 },
};

static struct i2c_driver onetouch_synaptics_ts_driver = {
	.probe		= onetouch_synaptics_ts_probe,
	.remove		= onetouch_synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= onetouch_synaptics_ts_suspend,
	.resume 	= onetouch_synaptics_ts_resume,
#endif
	.id_table	= onetouch_synaptics_ts_id,
	.driver = {
		.name	= ONETOUCH_SYNAPTICS_NAME,
		.owner 	= THIS_MODULE,
	},
};

static int __devinit onetouch_synaptics_ts_init(void)
{
	DEBUG_MSG (2, "\n [ONETOUCH] %s %s\n", __func__, ONETOUCH_SYNAPTICS_NAME);

	onetouch_synaptics_wq	=	create_singlethread_workqueue("onetouch_synaptics_wq");
	if (!onetouch_synaptics_wq)
		return -ENOMEM;

	return i2c_add_driver(&onetouch_synaptics_ts_driver);
}

static void __exit onetouch_synaptics_ts_exit(void)
{
	i2c_del_driver(&onetouch_synaptics_ts_driver);
    
	if (onetouch_synaptics_wq)
		destroy_workqueue(onetouch_synaptics_wq);
}

//module_init(onetouch_synaptics_ts_init);
subsys_initcall(onetouch_synaptics_ts_init);
module_exit(onetouch_synaptics_ts_exit);

MODULE_AUTHOR("jingyeong.noh@lge.com");
MODULE_DESCRIPTION("Synaptics OneTouchscreen Driver");
MODULE_LICENSE("GPL");
