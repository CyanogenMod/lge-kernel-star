/* drivers/input/keyboard/lge_touch_driver.c
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * This device driver can be used for any touch device (K36, use native-I2C)
 * You should check the document or contact the writer in order to understand whole process.
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
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/lge_touch_driver.h>

#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
//#include <linux/bssq_proximity.h>
#include <linux/input/gp2a.h>
extern bool in_call_state(void);
static bool already_call_state = false;
#endif

// 20110701 sangki.hyun@lge.com touch_fw_upgrade [S] {
#if !defined (SYNAPTICS_TOUCH_FW_UPGRADE)
struct lge_touch_driver
{
	void *							h_touch;
	struct touch_inputdev_func*		h_inputDev;
	struct touch_task_func*			h_task;
	struct touch_power_func* 		h_powerCtrl;
	struct touch_interrupt_func*	h_interrupt;
	struct touch_device_caps*		h_caps;
	struct touch_finger_data_func*	h_fingerData;
	struct touch_finger_data		finger_data;
	struct early_suspend			early_suspend;
};
#endif
// 20110701 sangki.hyun@lge.com touch_fw_upgrade [E] }

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touch_early_suspend(struct early_suspend *es);
static void touch_late_resume(struct early_suspend *es);
#endif

static bool create_touch_object(struct lge_touch_driver **touch_object);
static void remove_touch_object(struct lge_touch_driver *touch);


static bool create_touch_object(struct lge_touch_driver **touch_object)
{
	struct lge_touch_driver *touch;
	DO_A(touch = kzalloc(sizeof(struct lge_touch_driver), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_inputDev = kzalloc(sizeof(struct touch_inputdev_func), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_task = kzalloc(sizeof(struct touch_task_func), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_powerCtrl = kzalloc(sizeof(struct touch_power_func), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_interrupt = kzalloc(sizeof(struct touch_interrupt_func), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_fingerData = kzalloc(sizeof(struct touch_finger_data_func), GFP_KERNEL), err_Create_Touch_Object);
	*touch_object = touch;
	return true;

err_Create_Touch_Object:
	remove_touch_object(touch);
	return false;
}

static void remove_touch_object(struct lge_touch_driver *touch)
{
	if(touch->h_inputDev) kfree(touch->h_inputDev);
	if(touch->h_task) kfree(touch->h_task);
	if(touch->h_powerCtrl) kfree(touch->h_powerCtrl);
	if(touch->h_interrupt) kfree(touch->h_interrupt);
	if(touch->h_fingerData) kfree(touch->h_fingerData);

	if(touch) kfree(touch);
}

irqreturn_t interrupt_handler(int irq, void *dev_id)
{
	struct lge_touch_driver *touch = (struct lge_touch_driver*)dev_id;
	struct touch_interrupt_func* h_int = touch->h_interrupt;

	h_int->disable(touch->h_touch);
	h_int->start(touch->h_touch);

	return IRQ_HANDLED;
}

int task_handler(void *pdata)
{
	struct lge_touch_driver *touch = (struct lge_touch_driver*)pdata;
	struct touch_finger_data_func*	h_finger = touch->h_fingerData;
	struct touch_interrupt_func* h_int = touch->h_interrupt;
	struct touch_inputdev_func* h_input = touch->h_inputDev;
	struct touch_finger_data finger_data = touch->finger_data;
	u8	prev_single_finger_data_state = DO_NOT_ANYTHING;
	u8	prev_multi_finger_data_total_num = FINGER_RELEASE;

	DO_F(h_finger->additional_job(touch->h_touch, &finger_data, BEFORE_WHILE), err_task_handler);
	
	while(1){
		DO_F(h_int->wait(touch->h_touch), err_in_while);
		
		DO_F(h_finger->get_finger_data(touch->h_touch, &finger_data), err_in_while);
		DO_F(h_finger->additional_job(touch->h_touch, &finger_data, AFTER_GET_DATA), err_in_while);

		finger_data.state = h_finger->check_button(touch->h_touch, &finger_data);

		DEBUG_MSG(B, "[TOUCH] finger_num[%d], state[%d], c_button[%d], p_button[%d]", finger_data.total_num, finger_data.state, finger_data.curr_button, finger_data.prev_button);
		
		if(finger_data.total_num == FINGER_RELEASE){			
			printk("[TOUCH] FINGER_RELEASE : %d\n", finger_data.state);
			switch(finger_data.state){
				case ABS_RELEASE:
					h_input->send_ABS(touch->h_touch, finger_data.curr_data[0], FINGER_RELEASE); break;
				case BUTTON_RELEASE:
					h_input->send_Button(touch->h_touch, finger_data.prev_button, 0); break;
				default: break;
			}
			prev_single_finger_data_state = finger_data.state;
		}
		else if(finger_data.total_num == SINGLE_FINGER){			
			if(finger_data.state != prev_single_finger_data_state)
				printk("[TOUCH] SINGLE_FINGER : %d\n", finger_data.state);

			switch(finger_data.state){
				case ABS_PRESS: 
					h_input->send_ABS(touch->h_touch, finger_data.curr_data[0], SINGLE_FINGER); break;
				case ABS_RELEASE:
					h_input->send_ABS(touch->h_touch, finger_data.curr_data[0], FINGER_RELEASE); break;
				case BUTTON_PRESS:
					h_input->send_Button(touch->h_touch, finger_data.curr_button, 1); break;
				case BUTTON_RELEASE:
					h_input->send_Button(touch->h_touch, finger_data.prev_button, 0); break;
				default: break;
			}
			prev_single_finger_data_state = finger_data.state;
		}
		else{ //multi-finger
			if(finger_data.total_num != prev_multi_finger_data_total_num)
				printk("[TOUCH] MULTI_FINGER : %d\n", finger_data.state);

			switch(finger_data.state){
				case ABS_PRESS:
					h_input->send_ABS_Multi(touch->h_touch, finger_data.curr_data, finger_data.total_num); break;
				case BUTTON_RELEASE:
					h_input->send_Button(touch->h_touch, finger_data.prev_button, 0); break;
				default: break;
			}
		}
		prev_multi_finger_data_total_num = finger_data.total_num;

		h_input->send_Sync(touch->h_touch, finger_data.state);
		DO_F(h_finger->additional_job(touch->h_touch, &finger_data, AFTER_SYNC), err_in_while);
		
		memcpy(finger_data.prev_data, finger_data.curr_data, sizeof(finger_data.curr_data));
		finger_data.prev_button = finger_data.curr_button;
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_KS1103) // 20110903 sangki.hyun@lge.com fast_touch
		finger_data.prev_touch_finger_bit_mask = finger_data.touch_finger_bit_mask;
#endif
err_in_while:
		DO_F(h_int->enable(touch->h_touch), err_task_handler);
	}
err_task_handler:
	return 0;
}

static int __init touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lge_touch_driver *touch = NULL;

	DO_F(create_touch_object(&touch), err_kzalloc_probe);
	
	DO_F(device_open(&touch->h_touch, client, (u32)touch), err_Device_Open_probe);
	
	init_touch_inputdev(touch->h_touch, touch->h_inputDev);
	init_touch_task(touch->h_touch, touch->h_task);
	init_touch_power(touch->h_touch, touch->h_powerCtrl);
	init_touch_interrupt(touch->h_touch, touch->h_interrupt);
	init_touch_finger_data(touch->h_touch, touch->h_fingerData);
	init_touch_device_caps(touch->h_touch, touch->h_caps);
	
	DO_F(touch->h_powerCtrl->on(touch->h_touch), err_powerCtrl_probe);
	DO_F(touch->h_inputDev->open(touch->h_touch), err_inputDev_probe);
	DO_F(touch->h_interrupt->open(touch->h_touch), err_interrupt_probe);	
	DO_F(init_touch_device_setting(touch->h_touch, true), err_Device_Open_probe);
	DO_F(touch->h_task->open(touch->h_touch), err_task_probe);
	DO_F(touch->h_task->start(touch->h_touch), err_inputDev_probe);

	i2c_set_clientdata(client, touch);

#ifdef CONFIG_HAS_EARLYSUSPEND
	touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	touch->early_suspend.suspend = touch_early_suspend;
	touch->early_suspend.resume = touch_late_resume;
	register_early_suspend(&touch->early_suspend);
#endif

	DEBUG_MSG(M, "[TOUCH] touch_driver is initialized.\n");

	return 0;

err_inputDev_probe:
	touch->h_inputDev->close(touch->h_touch);
err_task_probe:
	touch->h_task->close(touch->h_touch);
err_interrupt_probe:
	touch->h_interrupt->close(touch->h_touch);
err_powerCtrl_probe:
	touch->h_powerCtrl->off(touch->h_touch);
err_Device_Open_probe:
	device_close(touch->h_touch);
err_kzalloc_probe:
	remove_touch_object(touch);
	return -1;
}

static int touch_remove(struct i2c_client *client)
{
	struct lge_touch_driver *touch = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch->early_suspend);
#endif
	touch->h_inputDev->close(touch->h_touch);
	touch->h_task->close(touch->h_touch);
	touch->h_interrupt->close(touch->h_touch);
	device_close(touch->h_touch);
	remove_touch_object(touch);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touch_early_suspend(struct early_suspend *es)
{
	struct lge_touch_driver *touch = container_of(es, struct lge_touch_driver, early_suspend);

	touch->h_interrupt->disable(touch->h_touch);
	touch_device_sleep_mode_setting(touch->h_touch, SLEEP_IN); // 20110725 sangki.hyun@lge.com Touch_Sleep_Mode
}

static void touch_late_resume(struct early_suspend *es)
{
	struct lge_touch_driver *touch = container_of(es, struct lge_touch_driver, early_suspend);

	touch->h_interrupt->enable(touch->h_touch);
	touch_device_sleep_mode_setting(touch->h_touch, SLEEP_OUT); // 20110725 sangki.hyun@lge.com Touch_Sleep_Mode

#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
	if (!already_call_state)
		init_touch_device_setting(touch->h_touch, false);
#else
	init_touch_device_setting(touch->h_touch, false);
#endif
}
#endif

static int touch_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lge_touch_driver *touch = i2c_get_clientdata(client);
	
#if 0//defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
	#ifndef CONFIG_HAS_EARLYSUSPEND
	touch->h_interrupt->disable(touch->h_touch);
	#endif

	already_call_state = false;
	if (!in_call_state()) {
		touch->h_powerCtrl->off(touch->h_touch);
		gp2a_power_control(0);
	} else
		already_call_state = true;
#else
	#ifndef CONFIG_HAS_EARLYSUSPEND
	touch->h_interrupt->disable(touch->h_touch);
	#endif
	touch->h_powerCtrl->off(touch->h_touch);
#endif
	return 0;
}

static int touch_resume(struct i2c_client *client)
{
	struct lge_touch_driver *touch = i2c_get_clientdata(client);

#if 0//defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
	if (!already_call_state) {
		gp2a_power_control(1);
		touch->h_powerCtrl->on(touch->h_touch);
	}

	#ifndef CONFIG_HAS_EARLYSUSPEND
	touch->h_interrupt->enable(touch->h_touch);
	init_touch_device_setting(touch->h_touch, false);
	#endif
#else
	touch->h_powerCtrl->on(touch->h_touch);
	#ifndef CONFIG_HAS_EARLYSUSPEND
	touch->h_interrupt->enable(touch->h_touch);
	init_touch_device_setting(touch->h_touch, false);
	#endif
#endif
	return 0;
}

static const struct i2c_device_id lge_ts_id[] = {
	{ LGE_TOUCH_NAME, 0 },
};

static struct i2c_driver lge_touch_driver = {
	.probe   = touch_probe,
	.remove	 = touch_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND //20120525 kim.youngmin@lge.com
	.suspend = touch_suspend,
	.resume	 = touch_resume,
#endif
	.id_table = lge_ts_id,
	.driver	 = {
		.name   = LGE_TOUCH_NAME,
	},
};

static int __devinit touch_init(void)
{
	DEBUG_MSG(M, "[TOUCH] touch_init\n");
	
	return i2c_add_driver(&lge_touch_driver);
}

static void __exit touch_exit(void)
{
	i2c_del_driver(&lge_touch_driver);
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("YEHAN AHN <yehan.ahn@lge.com>");
MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

