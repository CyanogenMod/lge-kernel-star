/*
 *  arch/arm/mach-msm/lge/lge_ats_event_log.c
 *
 *  Copyright (c) 2010 LGE.
 *
 *  All source code in this file is licensed under the following license
 *  except where indicated.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

#define DRIVER_NAME "ats_event_log"

struct ats_mtc_key_log_type{
    unsigned char log_id;
    unsigned short log_len;
    unsigned int x_hold;
    unsigned int y_code;
    unsigned char action;
};

static struct input_dev *ats_input_dev;

/* add ETA  key event logging for vs660 [younchan.kim 2010-05-31]*/
static struct input_handler input_handler;
static struct work_struct event_log_work;
struct ats_mtc_key_log_type ats_mtc_key_log1;

extern int ats_mtc_log_mask;
extern void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type *);

/* AT%GKPD [START] */
extern int get_gkpd_mode();
extern void add_gkpd_buf(int code);
/* AT%GKPD [END] */

int is_started = 0;

/* LGE_CHANGE
 * support MTC using diag port
 * 2010-07-11 taehung.kim@lge.com
 */
#if defined (CONFIG_MACH_MSM7X27_THUNDERC) || defined(LG_FW_MTC)
extern unsigned char g_diag_mtc_check;
extern void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);
#endif

#define ETA_KEY_MAX 8
#define ETA_ABS_MAX	7

/* key list for VS660 & LGP-500 */
int eta_key_list[ETA_KEY_MAX]={
	KEY_MENU,
	KEY_HOME,
	KEY_VOLUMEUP,
	KEY_SEARCH,
	KEY_BACK,
	KEY_VOLUMEDOWN,
    KEY_HOOK,
    KEY_POWER
};

int eta_abs_event[ETA_ABS_MAX]={
	ABS_X,
	ABS_Y,
	ABS_Z,
	ABS_MT_TOUCH_MAJOR,
	ABS_MT_TOUCH_MINOR,
	ABS_MT_POSITION_X,
	ABS_MT_POSITION_Y,
};

//ACTION filed information
typedef enum{
	ETA_TOUCH_MOVETO = 0, /*Move the pointer to the specified location*/
	ETA_TOUCH_MOVEBY = 1, /*Move the pointer by the specified values*/
	ETA_TOUCH_TAB = 2, /*Tab at the current location*/
	ETA_TOUCH_DOUBLETAB = 3, /*Double tab at the current location*/
	ETA_TOUCH_DOWN = 4, /*Touch down at the current location*/
	ETA_TOUCH_UP = 5, /*Touch up at the current location*/
	ETA_TOUCH_DEFAULT = 0xff,
}eta_touch_event_action_type;

int touch_status = 0 ;

static int ats_event_log_connect(struct input_handler *handler,struct input_dev *dev,const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;

	if (dev->name && strcmp(dev->name, "ats_input") == 0) return 0;
    printk("ats_event_log_connect(): %s\n", dev->name ? dev->name : "unnamed");

	for (i = 0 ; i < ETA_KEY_MAX - 1 ; i++){
		if (!test_bit(eta_key_list[i], dev->keybit))
			continue;
	}
	for (i = 0 ; i < ETA_ABS_MAX - 1 ; i++){
		if (!test_bit(eta_abs_event[i], dev->absbit))
			continue;
	}
	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if(!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "event_log";
	handle->private = NULL;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	return 0;
err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void ats_event_log_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id ats_event_log_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};


static void event_log_work_func(struct work_struct *work)
{
    printk("event_log_work_func\n");
    ats_mtc_send_key_log_to_eta(&ats_mtc_key_log1);
}

static int last_x = 0;
static int last_y = 0;
static int move_x = 0;
static int move_y = 0;
static int touch_event_timer_active = 0;

static void touch_event_timeout(unsigned long data)
{
    ats_mtc_key_log1.log_id = 2;
    ats_mtc_key_log1.log_len = 22;
    ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_UP;
	ats_mtc_key_log1.x_hold = last_x;
	ats_mtc_key_log1.y_code = last_y;

    schedule_work(&event_log_work);

    touch_event_timer_active = 0;
}

static struct timer_list touch_event_timer = TIMER_INITIALIZER(touch_event_timeout, 0, 0);

static void ats_event_log_event(struct input_handle *handle, unsigned int type,unsigned int code, int value)
{
    /* AT%GKPD [START] */
    if (type == EV_KEY && value == 1 && get_gkpd_mode())
    {
        add_gkpd_buf(code);
        return;
    }
    /* AT%GKPD [END] */

	if ( (type == EV_KEY) && (0x00000001 & ats_mtc_log_mask) ){
        printk("ats_event_log_event(): key event!\n");
		ats_mtc_key_log1.log_id = 1; /* LOG_ID, 1 key, 2 touch */
		ats_mtc_key_log1.log_len = 18; /* LOG_LEN */
		ats_mtc_key_log1.x_hold = value; /* hold */
		ats_mtc_key_log1.y_code = code;
		schedule_work(&event_log_work);
	}
	else if ( (type == EV_ABS || type == EV_SYN) && (0x00000002 & ats_mtc_log_mask) ){
		switch(code){
			case ABS_MT_TOUCH_MAJOR:
				{
					touch_status++;

                    if (touch_event_timer_active)
                    {
				        ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_MOVETO;
                        mod_timer(&touch_event_timer, jiffies + 1 * HZ / 2);
                    }
                    else
                    {
				        ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_DOWN;

                        touch_event_timer.expires = jiffies + 1 * HZ / 2;
                        add_timer(&touch_event_timer);
                        touch_event_timer_active = 1;
                    }
					break;
				}
			case ABS_MT_POSITION_X :
				{
					ats_mtc_key_log1.x_hold = value;
                    move_x = last_x - value; if (move_x < 0) move_x = -move_x;
                    last_x = value;
					touch_status++;
					break;
				}
			case ABS_MT_POSITION_Y:
				{
					ats_mtc_key_log1.y_code = value;
                    move_y = last_y - value; if (move_y < 0) move_y = -move_y;
                    last_y = value;
					touch_status++;
					break;
				}
		}

		if (touch_status == 3) {
            touch_status = 0;
            if (ats_mtc_key_log1.action == (unsigned char)ETA_TOUCH_MOVETO && move_x < 7 && move_y < 7)
            {
                ; // ignore tiny move
            }
            else
            {
                ats_mtc_key_log1.log_id = 2; /* LOG_ID, 1 key, 2 touch */
                ats_mtc_key_log1.log_len = 22; /*LOG_LEN */
                schedule_work(&event_log_work);
            }
		}
	}
}

int event_log_start(void)
{
	int ret = 0;

    if (is_started == 1) return 0;

	input_handler.name = "key_log";
	input_handler.connect = ats_event_log_connect;
	input_handler.disconnect = ats_event_log_disconnect;
	input_handler.event = ats_event_log_event;
	input_handler.id_table = ats_event_log_ids;
	ret = input_register_handler(&input_handler);
	if (ret != 0)
		printk("%s:fail to registers input handler\n", __func__);

	INIT_WORK(&event_log_work,event_log_work_func);

    is_started = 1;

	return 0;
}
EXPORT_SYMBOL(event_log_start);

int event_log_end(void)
{
    if (is_started == 0) return 0;

	input_unregister_handler(&input_handler);

    is_started = 0;

	return 0 ;
}
EXPORT_SYMBOL(event_log_end);

int event_log_mask(unsigned int mask)
{
    switch (mask)
    {
        case 0x00000000: // ETA_LOGMASK_DISABLE_ALL:
        case 0xFFFFFFFF: // ETA_LOGMASK_ENABLE_ALL:
        case 0x00000001: // ETA_LOGITEM_KEY:
        case 0x00000002: // ETA_LOGITEM_TOUCHPAD:
        case 0x00000003: // ETA_LOGITME_KEYTOUCH:
            ats_mtc_log_mask = mask;
            break;
        default:
            ats_mtc_log_mask = 0x00000000; // ETA_LOGMASK_DISABLE_ALL
            break;
    }

    if (mask & 0xFFFFFFFF) event_log_start();
    else event_log_end();
}
EXPORT_SYMBOL(event_log_mask);

/* [END] add ETA  key event logging for vs660 [younchan.kim 2010-05-31]*/


static int  __init ats_event_log_probe(struct platform_device *pdev)
{
	int rc = 0 ;
	return rc;
}

static int ats_event_log_remove(struct platform_device *pdev)
{
	input_unregister_device(ats_input_dev);
	return 0;
}

static struct platform_driver ats_input_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	 = ats_event_log_probe,
	.remove = ats_event_log_remove,
};

static int __init ats_input_init(void)
{
	return platform_driver_register(&ats_input_driver);
}


static void __exit ats_input_exit(void)
{
	platform_driver_unregister(&ats_input_driver);
}

module_init(ats_input_init);
module_exit(ats_input_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("ATS_EVENT_LOG driver");
MODULE_LICENSE("GPL v2");
