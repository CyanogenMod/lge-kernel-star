/* drivers/input/keyboard/LGE_touch_driver.c
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


#include <linux/LGE_touch_driver.h>

extern bool is_star_touch_enable;

typedef struct LGE_touch_driver
{
	LGE_Device_Handle			h_touch;
	LGE_Touch_InputDev_Handle	h_inputDev;
	LGE_Touch_Task_Handle		h_task;
	LGE_Touch_PowerCtrl_Handle 	h_powerCtrl;
	LGE_Touch_Interrupt_Handle	h_interrupt;
	LGE_Touch_DeviceCap_Handle	h_caps;
	LGE_Touch_FingerData_Handle	h_fingerData;
	LGE_Touch_FingerData		finger_data;
	LGE_Touch_SleepMode_Handle	h_sleepMode;
	struct early_suspend		early_suspend;
	struct mutex			mutex;
}LGE_Touch_Driver_Data;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void LGE_touch_early_suspend(struct early_suspend *es);
static void LGE_touch_late_resume(struct early_suspend *es);
#endif

static bool Create_Touch_Object(LGE_Touch_Driver_Data **touch_object);
static void Remove_Touch_Object(LGE_Touch_Driver_Data *touch);


static bool Create_Touch_Object(LGE_Touch_Driver_Data **touch_object)
{
	LGE_Touch_Driver_Data *touch;
	DO_A(touch = kzalloc(sizeof(LGE_Touch_Driver_Data), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_inputDev = kzalloc(sizeof(LGE_Touch_InputDev), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_task = kzalloc(sizeof(LGE_Touch_Task), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_powerCtrl = kzalloc(sizeof(LGE_Touch_PowerCtrl), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_sleepMode = kzalloc(sizeof(LGE_Touch_SleepMode), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_interrupt = kzalloc(sizeof(LGE_Touch_Interrupt), GFP_KERNEL), err_Create_Touch_Object);
	DO_A(touch->h_fingerData = kzalloc(sizeof(LGE_Touch_FingerData_Func), GFP_KERNEL), err_Create_Touch_Object);
	*touch_object = touch;
	return true;

err_Create_Touch_Object:
	Remove_Touch_Object(touch);
	return false;
}

static void Remove_Touch_Object(LGE_Touch_Driver_Data *touch)
{
	if(touch->h_inputDev) kfree(touch->h_inputDev);
	if(touch->h_task) kfree(touch->h_task);
	if(touch->h_powerCtrl) kfree(touch->h_powerCtrl);
	if(touch->h_sleepMode) kfree(touch->h_sleepMode);
	if(touch->h_interrupt) kfree(touch->h_interrupt);
	if(touch->h_fingerData) kfree(touch->h_fingerData);

	if(touch) kfree(touch);
}

irqreturn_t Interrupt_Handler(int irq, void *dev_id)
{
	LGE_Touch_Driver_Data *touch = (LGE_Touch_Driver_Data*)dev_id;
	LGE_Touch_Interrupt_Handle h_int = touch->h_interrupt;

	h_int->disable(touch->h_touch);
	h_int->start(touch->h_touch);

	return IRQ_HANDLED;
}

int Task_Handler(void *pdata)
{
	LGE_Touch_Driver_Data *touch = (LGE_Touch_Driver_Data*)pdata;
	LGE_Touch_FingerData_Handle	h_finger = touch->h_fingerData;
	LGE_Touch_Interrupt_Handle h_int = touch->h_interrupt;
	LGE_Touch_InputDev_Handle h_input = touch->h_inputDev;
	LGE_Touch_FingerData finger_data = touch->finger_data;

	mutex_lock(&touch->mutex);

	DO_F(h_finger->additional_job(touch->h_touch, &finger_data, BEFORE_WHILE), err_task_handler);
	
	while(1){
		DO_F(h_int->wait(touch->h_touch), err_in_while);
		
		DO_F(h_finger->get_finger_data(touch->h_touch, &finger_data), err_in_while);
		DO_F(h_finger->additional_job(touch->h_touch, &finger_data, AFTER_GET_DATA), err_in_while);

		finger_data.state = h_finger->check_button(touch->h_touch, &finger_data);

		DEBUG_MSG(B, "[TOUCH] finger_num[%d], state[%d], c_button[%d], p_button[%d]", finger_data.total_num, finger_data.state, finger_data.curr_button, finger_data.prev_button);
		
		if(finger_data.total_num == FINGER_RELEASE){
			switch(finger_data.state){
				case ABS_RELEASE:
					h_input->send_ABS(touch->h_touch, finger_data.curr_data[0], FINGER_RELEASE); break;
				case BUTTON_RELEASE:
					h_input->send_Button(touch->h_touch, finger_data.prev_button, 0); break;
				default: break;
			}
		}
		else if(finger_data.total_num == SINGLE_FINGER){
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
		}
		else{ //multi-finger
			switch(finger_data.state){
				case ABS_PRESS:
					h_input->send_ABS_Multi(touch->h_touch, finger_data.curr_data, finger_data.total_num); break;
				case BUTTON_RELEASE:
					h_input->send_Button(touch->h_touch, finger_data.prev_button, 0); break;
				default: break;
			}
		}
		
		h_input->send_Sync(touch->h_touch, finger_data.state);
		DO_F(h_finger->additional_job(touch->h_touch, &finger_data, AFTER_SYNC), err_in_while);
		
		memcpy(finger_data.prev_data, finger_data.curr_data, sizeof(finger_data.curr_data));
		finger_data.prev_button = finger_data.curr_button;
err_in_while:
		DO_F(h_int->enable(touch->h_touch), err_task_handler);
	}
err_task_handler:
	mutex_unlock(&touch->mutex);
	return 0;
}

static int __init LGE_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LGE_Touch_Driver_Data *touch = NULL;

	DO_F(Create_Touch_Object(&touch), err_kzalloc_probe);
	
	DO_F(Device_Open(&touch->h_touch, client, (u32)touch), err_Device_Open_probe);
	
	Init_InputDev(touch->h_touch, touch->h_inputDev);
	Init_Task(touch->h_touch, touch->h_task);
	Init_PowerCtrl(touch->h_touch, touch->h_powerCtrl);
	Init_SleepMode(touch->h_touch, touch->h_sleepMode);
	Init_Interrupt(touch->h_touch, touch->h_interrupt);
	Init_FingerData(touch->h_touch, touch->h_fingerData);
	Init_DeviceCap(touch->h_touch, touch->h_caps);

	DO_F(touch->h_powerCtrl->on(touch->h_touch), err_powerCtrl_probe);
// MOBII_CHANGE_S 20120430 dk.han@mobii.co.kr : move to above 
        DO_F(touch->h_inputDev->open(touch->h_touch), err_inputDev_probe);
        DO_F(touch->h_interrupt->open(touch->h_touch), err_interrupt_probe);
// MOBII_CHANGE_S 20120430 dk.han@mobii.co.kr : move to above 

	DO_F(Init_Specific_Device_Setting(touch->h_touch), err_Device_Open_probe);

	DO_F(touch->h_task->open(touch->h_touch), err_task_probe);
	DO_F(touch->h_task->start(touch->h_touch), err_inputDev_probe);
// MOBII_CHANGE_S 20120430 dk.han@mobii.co.kr : move to above
	//DO_F(touch->h_interrupt->open(touch->h_touch), err_interrupt_probe);

	//DO_F(touch->h_inputDev->open(touch->h_touch), err_inputDev_probe);
 // MOBII_CHANGE_S 20120430 dk.han@mobii.co.kr : move to above

	mutex_init(&touch->mutex);
	i2c_set_clientdata(client, touch);

#ifdef CONFIG_HAS_EARLYSUSPEND
	touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN; //Mobii_Change sgkim@mobii.co.kr 20120608- Speed up LCD On : remove [+1]
	touch->early_suspend.suspend = LGE_touch_early_suspend;
	touch->early_suspend.resume = LGE_touch_late_resume;
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
	Device_Close(touch->h_touch);
err_kzalloc_probe:
	Remove_Touch_Object(touch);
	return -1;
}

static int LGE_touch_remove(struct i2c_client *client)
{
	LGE_Touch_Driver_Data *touch = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch->early_suspend);
#endif
	touch->h_inputDev->close(touch->h_touch);
	touch->h_task->close(touch->h_touch);
	touch->h_interrupt->close(touch->h_touch);
	Device_Close(touch->h_touch);
	Remove_Touch_Object(touch);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void LGE_touch_early_suspend(struct early_suspend *es)
{
	LGE_Touch_Driver_Data *touch = container_of(es, LGE_Touch_Driver_Data, early_suspend);
    printk("LGE_touch_early_suspend() called\n");

	touch->h_interrupt->disable(touch->h_touch);
	//touch->h_powerCtrl->off(touch->h_touch);
	touch->h_sleepMode->control(touch->h_touch, true, SYNAPTICS_POWERMODE_3);
}

static void LGE_touch_late_resume(struct early_suspend *es)
{
	LGE_Touch_Driver_Data *touch = container_of(es, LGE_Touch_Driver_Data, early_suspend);
    printk("LGE_touch_late_resume() called\n");

	//touch->h_powerCtrl->on(touch->h_touch);
	if(!touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_0))
	{
		touch->h_powerCtrl->off(touch->h_touch);
	touch->h_powerCtrl->on(touch->h_touch);
	Init_Specific_Device_Setting(touch->h_touch);    

		touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_3);
		touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_1);
	}
	touch->h_interrupt->enable(touch->h_touch);
	//Init_Specific_Device_Setting(touch->h_touch);    
}
#else
static int LGE_touch_suspend(struct i2c_client *client, pm_message_t state)
{
	LGE_Touch_Driver_Data *touch = i2c_get_clientdata(client);
    printk("LGE_touch_suspend() called\n");

	DO_F(touch->h_interrupt->disable(touch->h_touch), err_touch_suspend);
	//DO_F(touch->h_powerCtrl->off(touch->h_touch), err_touch_suspend);
	touch->h_sleepMode->control(touch->h_touch, true, SYNAPTICS_POWERMODE_3);

	return 0;
	
err_touch_suspend:
	return -1;
}

static int LGE_touch_resume(struct i2c_client *client)
{
	LGE_Touch_Driver_Data *touch = i2c_get_clientdata(client);
    printk("LGE_touch_resume() called\n");

	//DO_F(touch->h_powerCtrl->on(touch->h_touch), err_touch_resume);
	if(!touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_0))
	{
		//touch->h_powerCtrl->off(touch->h_touch);
		//touch->h_powerCtrl->on(touch->h_touch);
		//Init_Specific_Device_Setting(touch->h_touch);    

		touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_3);
		touch->h_sleepMode->control(touch->h_touch, false, SYNAPTICS_POWERMODE_1);
	}
	DO_F(touch->h_interrupt->enable(touch->h_touch), err_touch_resume);
	//DO_F(Init_Specific_Device_Setting(touch->h_touch), err_touch_resume);
	return 0;

err_touch_resume:
	return -1;
}
#endif

static const struct i2c_device_id LGE_ts_id[] = {
	{ LGE_TOUCH_NAME, 0 },
};

static struct i2c_driver LGE_touch_driver = {
	.probe   = LGE_touch_probe,
	.remove	 = LGE_touch_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = LGE_touch_suspend,
	.resume	 = LGE_touch_resume,
#endif
	.id_table = LGE_ts_id,
	.driver	 = {
		.name   = LGE_TOUCH_NAME,
	},
};

static int __devinit LGE_touch_init(void)
{
	DEBUG_MSG(M, "[TOUCH] LGE_touch_init\n");
	
	return i2c_add_driver(&LGE_touch_driver);
}

static void __exit LGE_touch_exit(void)
{
	i2c_del_driver(&LGE_touch_driver);
}

module_init(LGE_touch_init);
module_exit(LGE_touch_exit);

MODULE_AUTHOR("YEHAN AHN <yehan.ahn@lge.com>");
MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

