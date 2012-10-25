 /*
  * Proximity sensor driver for GP2AP002S00F
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/sysfs.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <../gpio-names.h>
#include <lge/bssq_proximity.h>
#include <lge/lge_hw_rev.h>

/*********** for debug **********************************************************/
#if 1 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* global var */
static struct workqueue_struct *gp2a_wq;
static struct regulator *regulator= NULL;
static bool proximity_enable = OFF;
static struct wake_lock gp2a_wake_lock;

static int proximity_status = 1; //20110706 deukgi.shin@lge.com : for diag test mode. 250.49
#define FAR 1

int opt_i2c_read(struct gp2a_data *gp2a, int reg) {
	int val;
	int err;
	u8 buf[2];
	struct i2c_msg msg[2];

	buf[0] = reg; 
	msg[0].addr = gp2a->client->addr;
	msg[0].flags = 1;
	msg[0].len = 2;
	msg[0].buf = buf;
	err = i2c_transfer(gp2a->client->adapter, msg, 1);
	
	val = buf[1];
	
	if (err >= 0) return val;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

int opt_i2c_write(struct gp2a_data *gp2a, u8 reg, int val) {
	int err;

	if( (gp2a == NULL) || (gp2a->client == NULL) ){
		return -ENODEV;
	}

	err = i2c_smbus_write_byte_data(gp2a->client, reg, val);
	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

/*****************************************************************************************
 *  
 *  function    : gp2a_work_func_prox 
 *  description : This function is for proximity sensor (using B-1 Mode ). 
 *                when INT signal is occured , it gets value from VO register.   
 */
void gp2a_work_func_prox(struct work_struct *work) {
	struct gp2a_data *gp2a = container_of(work, struct gp2a_data, work_prox);
	
	unsigned char value;
	unsigned char vout = 0;

	if (proximity_enable == OFF) {
		gprintk("proximity is not enable\n");
		return;
	}

	//20111001 woo.jung@lge.com : change disable step for Mode B1
	disable_irq_nosync(gp2a->irq);
	
	//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
	/* Procedure 5 - Read VO & INT Clear */
	value = opt_i2c_read(gp2a, REGS_PROX);
	
	//Report Data toward HAL [S]
	vout = (value & 0x01)?0:1;
	//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [S]
	if(vout)
		proximity_status = 0;
	else proximity_status = 1;
	//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [E]
	gprintk("value = %x, vout = %d\n",value, vout);

	wake_lock_timeout(&gp2a_wake_lock, 3 * HZ);
	input_report_abs(gp2a->input_dev,ABS_DISTANCE,(int)vout);
	input_sync(gp2a->input_dev);
	mdelay(1);
	//Reposrt Data toward HAL [E]
	
	/* Procedure 6 - Write HYS Register */
//20111110 - woo.jung@lge.com - Revision 贸府 眠啊! - HW Tuning for Touch Window[S]	
#if defined(CONFIG_KS1103)
	if(get_lge_pcb_revision() > REV_C) {
		if(vout) 
			value = 0x00;
		else 
			value = 0xC3;
	} else {
		if(vout)
			value = 0x40;
		else
			value = 0x23;
	}
#else
	if(vout)
		value = 0x40;
	else
		value = 0x23;
#endif
//20111110 - woo.jung@lge.com - Revision 贸府 眠啊! - HW Tuning for Touch Window[E]
	opt_i2c_write(gp2a, (u8)(REGS_HYS), value);

	/* Procedure 7 - Forcing vout terminal to go high */
	opt_i2c_write(gp2a, (u8)(REGS_CON), 0x18);

	/* Procedure 8 - enable INT */
	enable_irq(gp2a->irq);
	mdelay(2);

	/* Procedure 9 - enabling VOUT terminal in nomal operation */
	opt_i2c_write(gp2a, (u8)(REGS_CON), 0x00);
	//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]
}

irqreturn_t gp2a_irq_handler(int irq, void *dev_id) {
	struct gp2a_data *gp2a = dev_id;
	
	if(gp2a->irq !=-1) {
		mutex_lock(&gp2a->lock);
		queue_work(gp2a_wq, &gp2a->work_prox);
		mutex_unlock(&gp2a->lock);
	}

	return IRQ_HANDLED;
}

void gp2a_read_forcely(struct gp2a_data *gp2a) {
	mutex_lock(&gp2a->lock);
	queue_work(gp2a_wq, &gp2a->work_prox);
	mutex_unlock(&gp2a->lock);
}

void gp2a_power_control(bool enable) {
	gprintk("\n");

#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
	if (!regulator) {
		regulator = regulator_get(NULL, "vcc_prox_3v0");

		if (!regulator) {
			printk(KERN_INFO "Proximity Sensor: vcc_prox_3v0 failed\n");
			return;
		} else {
			regulator_set_voltage(regulator, 3000000, 3000000);
		}
	}

	if (enable) {
		if (!regulator_is_enabled(regulator)) {
			regulator_enable(regulator);
		}
	} else {
		if (regulator_is_enabled(regulator)) {
			regulator_disable(regulator);
		}
	}
#else
	regulator = regulator_get(NULL, "vcc_prox_3v0");
	if (!regulator)
		printk(KERN_INFO "Proximity Sensor: vcc_prox_3v0 failed\n");	
	else
		regulator_set_voltage(regulator, 3000000, 3000000);
	regulator_enable(regulator);
#endif
}
EXPORT_SYMBOL(gp2a_power_control);

void gp2a_on(struct gp2a_data *gp2a) {
	int i;

	if(proximity_enable == OFF) {
		gprintk("gp2a power on\n");
#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
		gp2a_power_control(ON);
		msleep(5);	
#endif
		proximity_enable = ON;
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
		//Shutdown Mode(0x02) -> Operation Mode(ox03) Procedure 1~5
		opt_i2c_write(gp2a, (u8)(REGS_CON),  0x18);//1
		opt_i2c_write(gp2a, (u8)(REGS_HYS),  0x40);//2
		opt_i2c_write(gp2a, (u8)(REGS_OPMOD),  0x03);//3

		enable_irq(gp2a->irq);//4
		msleep(2);
		opt_i2c_write(gp2a, (u8)(REGS_CON),  0x00);//5
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]
		
		gp2a_read_forcely(gp2a);
	}
}

void gp2a_off(struct gp2a_data *gp2a) {

	gprintk("gp2a_off\n");
	if(proximity_enable == ON) {
		gprintk("disable irq for proximity \n");
	
		//20110921 woo.jung@lge.com : for Phone Call and First Status in Framework[S]
		wake_lock_timeout(&gp2a_wake_lock, 3 * HZ);
		input_report_abs(gp2a->input_dev,ABS_DISTANCE, FAR);
	        input_sync(gp2a->input_dev);
	        mdelay(1);
		//20110921 woo.jung@lge.com : for Phone Call and First Status in Framework[E]	
		
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
		//Operation Mode(0x03) -> Shutdown Mode(0x02)
		disable_irq_nosync(gp2a->irq);
		opt_i2c_write(gp2a, (u8)(REGS_OPMOD), 0x02);
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]
		proximity_enable = OFF;
	}
}
//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [S]
static ssize_t gp2a_onoff_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	return sprintf(buf, "%d\n", (int)proximity_enable);
}
//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [E]

static ssize_t gp2a_onoff_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	value?gp2a_on(gp2a):gp2a_off(gp2a);
	
	return size;
}
//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [S]
static ssize_t gp2a_status_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	gp2a_read_forcely(gp2a);
	msleep(100);
	return sprintf(buf, "%d\n",  proximity_status);
}
//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [E]
static DEVICE_ATTR(enable, S_IRUGO|S_IWUGO, gp2a_onoff_show, gp2a_onoff_store);
static DEVICE_ATTR(status, S_IRUGO|S_IWUGO, gp2a_status_show, NULL);//20110706 deukgi.shin@lge.com : for diag test mode. 250.49


static struct attribute *gp2a_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_status.attr,	//20110706 deukgi.shin@lge.com : for diag test mode. 250.49 [S]
	NULL
};

static const struct attribute_group gp2a_group = {
	.attrs = gp2a_attributes,
};

static int gp2a_opt_probe(struct i2c_client *client,
			    const struct i2c_device_id *id) {
	struct gp2a_data *gp2a;
	int i;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
			return -EIO;
	}

	/* allocate driver_data */
	gp2a = kzalloc(sizeof(struct gp2a_data),GFP_KERNEL);
	if(!gp2a) {
		pr_err("kzalloc error\n");
		return -ENOMEM;
	}

	gprintk("in %s \n",__func__);

	gp2a->client = client;
	gp2a->irq = client->irq;
	i2c_set_clientdata(client, gp2a);

	/* Input device Settings */
	gp2a->input_dev = input_allocate_device();
	if (gp2a->input_dev == NULL) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}
	gp2a->input_dev->name = GP2A_NAME;

	set_bit(EV_SYN,gp2a->input_dev->evbit);
	set_bit(EV_ABS,gp2a->input_dev->evbit);
	
	input_set_abs_params(gp2a->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(gp2a->input_dev);
	if (ret) {
		pr_err("Unable to register %s input device\n", gp2a->input_dev->name);
		input_free_device(gp2a->input_dev);
		kfree(gp2a);
		return -1;
	}

	/* WORK QUEUE Settings */
	gp2a_wq = create_singlethread_workqueue("gp2a_wq");
	if (!gp2a_wq)
		return -ENOMEM;
	INIT_WORK(&gp2a->work_prox, gp2a_work_func_prox);
	mutex_init(&gp2a->lock);
	gprintk("Workqueue Settings complete\n");
		
	/* INT Settings */	
	ret = request_irq(gp2a->irq, gp2a_irq_handler, IRQ_TYPE_EDGE_FALLING, "gp2a_int", gp2a);
	if (ret) {
		pr_err("unable to request irq %d\n", gp2a->irq);
		return ret;
	}
	disable_irq_nosync(gp2a->irq);
// 20110718 sangki.hyun@lge.com proximity on/off [S] {
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
	//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
	/* Proximity Initiate Operation */
	for(i = 1;i <= 4;i++) 
		opt_i2c_write(gp2a, (u8)(i), gp2a_original_image[i]);

	// 0x08,  //Gain(LED) - REGS_GAIN
	// 0xc2,  //Hysteresis - REGS_HYS
	// 0x14,  //Detection Cycle - REGS_CYCLE ->0x04 : 8ms,0x0c : 16ms, 0x14 : 32ms, 0x1c : 64ms, 0x24 : 128ms ...
	// 0x02,  //Operation Mode -> Suspend Mode
	
	//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]
#endif
// 20110718 sangki.hyun@lge.com proximity on/off [E] }
	wake_lock_init(&gp2a_wake_lock, WAKE_LOCK_SUSPEND, "gp2a_wake_lock");
	
	if ((ret = sysfs_create_group(&client->dev.kobj, &gp2a_group))) {
		return -ENOMEM;
	}	
		
	return 0;
}

static int gp2a_opt_remove(struct i2c_client *client)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &gp2a_group);
	
	if (gp2a_wq)
		destroy_workqueue(gp2a_wq);

	wake_lock_destroy(&gp2a_wake_lock);
	free_irq(gp2a->irq, gp2a);

	input_unregister_device(gp2a->input_dev);

	kfree(gp2a);
	return 0;
}

static int gp2a_opt_suspend(struct i2c_client *client, pm_message_t state) {
	struct gp2a_data *gp2a = i2c_get_clientdata(client);

	gprintk("suspend, proximity_enable:%d\n", proximity_enable);

//20110702 youngjin.yoo@lge.com nothing [S]
#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001) || defined(CONFIG_KS1103) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
	// nothing
#else
	if(proximity_enable) {
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
		//Operation Mode(0x03) -> Shutdown Mode(0x02)
		disable_irq_nosync(gp2a->irq);
		opt_i2c_write(gp2a, (u8)(REGS_OPMOD), 0x02);
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]
		proximity_enable = OFF;
	}
#endif
//20110702 youngjin.yoo@lge.com nothing [E]

	return 0;
}

static int gp2a_opt_resume(struct i2c_client *client) {
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	int i;

	gprintk("resume, proximity_enable:%d\n", proximity_enable);

//20110702 youngjin.yoo@lge.com nothing [S]
#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001) || defined(CONFIG_KS1103) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
	// nothing
#else
	if(!proximity_enable) {

		/* GP2A Regs INIT SETTINGS */
		for(i=1; i<=4; i++) 
			opt_i2c_write(gp2a, (u8)(i), gp2a_original_image[i]);
		
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [S]
		//Shutdown Mode(0x02) -> Operation Mode(ox03) Procedure 1~5
		opt_i2c_write(gp2a, (u8)(REGS_CON),  0x18);//1
		opt_i2c_write(gp2a, (u8)(REGS_HYS),  0x40);//2
		opt_i2c_write(gp2a, (u8)(REGS_OPMOD),  0x03);//3

		enable_irq(gp2a->irq);//4
		msleep(2);
		opt_i2c_write(gp2a, (u8)(REGS_CON),  0x00);//5
		//20111001 woo.jung@lge.com : Set Operation mode B1 for X2 New HW Revision [E]	

		proximity_enable = ON;
	}
#endif
//20110702 youngjin.yoo@lge.com nothing [E]

	return 0;
}

static const struct i2c_device_id proxi_ids[] = {
	{ GP2A_NAME, 0 },
	{ },
};

static struct i2c_driver gp2a_opt_driver = {
	.probe 	 = gp2a_opt_probe,
	.remove  = gp2a_opt_remove,
	.suspend = gp2a_opt_suspend,
	.resume  = gp2a_opt_resume,
	.id_table = proxi_ids,
	.driver  = {
		.name = GP2A_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init gp2a_opt_init(void) {
	int ret;
	
	printk(KERN_INFO "enter %s\n", __func__);
#if !defined(CONFIG_LU6500) && !defined(CONFIG_KS1001)
	gp2a_power_control(ON);
#endif
	return i2c_add_driver(&gp2a_opt_driver);
}

static void __exit gp2a_opt_exit(void) {
	printk(KERN_INFO "enter %s\n", __func__);
	i2c_del_driver(&gp2a_opt_driver);
}

module_init( gp2a_opt_init );
module_exit( gp2a_opt_exit );

MODULE_DESCRIPTION("Optical Sensor driver for gp2ap002s00f");
MODULE_LICENSE("GPL");
