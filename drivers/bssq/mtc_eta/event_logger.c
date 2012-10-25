/*
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

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "lge_mtc_eta.h"

//#define MTC_ETA_DEBUG
#ifdef MTC_ETA_DEBUG
#define PDEBUG(fmt, args...) printk("mtc_eta_logger: " fmt, ## args)
#else
#define PDEBUG(fmt, args...)
#endif

extern int mtc_eta_start_touch_logging(void);
extern int mtc_eta_start_key_logging(void);
extern void mtc_eta_stop_touch_logging(void);
extern void mtc_eta_stop_key_logging(void);

unsigned int lge_mtc_eta_log_mask = 0;

static LIST_HEAD(mtc_eta_event_log_q);
static DEFINE_SPINLOCK(mtc_eta_event_log_lock);
static DECLARE_WAIT_QUEUE_HEAD(mtc_eta_event_log_wq);

void mtc_eta_add_logging_event(struct mtc_eta_log *log)
{
	struct mtc_eta_event_log_q_entry *new_entry;

	new_entry =  kmalloc(sizeof(struct mtc_eta_event_log_q_entry), GFP_ATOMIC);
	if (!new_entry) {
		printk("mtc_eta_event_logger: cannot allocate new entry buffer\n");
		return;
	}

	log->time = jiffies_to_msecs(jiffies);
	new_entry->log = log;

	spin_lock(&mtc_eta_event_log_lock);
	list_add_tail(&new_entry->list, &mtc_eta_event_log_q);
	spin_unlock(&mtc_eta_event_log_lock);

	wake_up_interruptible(&mtc_eta_event_log_wq);
}

static long mtc_eta_get_log(void __user *arg)
{
	struct mtc_eta_event_log_q_entry *event_log = NULL;
	long rc;

	rc = wait_event_interruptible(mtc_eta_event_log_wq,
			!list_empty(&mtc_eta_event_log_q));
	if (rc < 0) {
                printk("mtc_eta_logger: rc = %ld\n", rc);
                return -ERESTARTSYS;
        }

	spin_lock(&mtc_eta_event_log_lock);
	if (!list_empty(&mtc_eta_event_log_q)) {
		event_log = list_first_entry(&mtc_eta_event_log_q,
				struct mtc_eta_event_log_q_entry, list);

#ifdef MTC_ETA_DEBUG
		PDEBUG("getting time %u\n", event_log->log->time);
		switch(event_log->log->id) {
			case MTC_ETA_LOG_ID_KEY:
				PDEBUG("getting key hold %d, code: 0x%x\n",
						event_log->log->data.key.hold, event_log->log->data.key.key_code);
				break;
			case MTC_ETA_LOG_ID_TOUCH:
				PDEBUG("getting touch action 0x%x, x: %d, y: %d\n", 
						event_log->log->data.touch.action, event_log->log->data.touch.x, event_log->log->data.touch.y);
				break;
			default:
				PDEBUG("wrong event\n");
		}
#endif /* MTC_ETA_DEBUG */

		/* Copy event log to user space */
		if (copy_to_user((void *)arg,
					event_log->log, sizeof(struct mtc_eta_log)))
			rc = -EFAULT;

		list_del_init(&event_log->list);
		kfree(event_log->log);
		kfree(event_log);
	}
	spin_unlock(&mtc_eta_event_log_lock);

	return rc;
}

static void mtc_eta_cleanup_log_list(void)
{
	struct mtc_eta_event_log_q_entry *event_log = NULL;

	PDEBUG("clean up log list\n");
	spin_lock(&mtc_eta_event_log_lock);
	if (!list_empty(&mtc_eta_event_log_q)) {
		event_log = list_first_entry(&mtc_eta_event_log_q,
				struct mtc_eta_event_log_q_entry, list);

#ifdef MTC_ETA_DEBUG
		PDEBUG("getting time %u\n", event_log->log->time);
		switch(event_log->log->id) {
			case MTC_ETA_LOG_ID_KEY:
				PDEBUG("getting key hold: %d, code: 0x%x\n",
						event_log->log->data.key.hold, event_log->log->data.key.key_code);
				break;
			case MTC_ETA_LOG_ID_TOUCH:
				PDEBUG("getting touch action 0x%x, x: %d, y: %d\n", 
						event_log->log->data.touch.action, event_log->log->data.touch.x, event_log->log->data.touch.y);
				break;
			default:
				PDEBUG("wrong event\n");
		}
#endif /* MTC_ETA_DEBUG */
		list_del_init(&event_log->list);
		kfree(event_log->log);
		kfree(event_log);

	}
	spin_unlock(&mtc_eta_event_log_lock);
}

static long mtc_eta_log_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0;
	unsigned int log_mask = 0;

	switch (cmd) {
		case LGE_ETA_IOC_CHANGE_LOG_MASK:
			if (copy_from_user((void *)&log_mask,
						argp, sizeof(unsigned int)))
				rc = -EFAULT;

			PDEBUG("current log_mask: 0x%x, new log_mask: 0x%x\n",
					log_mask, lge_mtc_eta_log_mask);

			if ((log_mask & MTC_ETA_LOG_ID_KEY)
					^ (lge_mtc_eta_log_mask & MTC_ETA_LOG_ID_KEY)) {
				PDEBUG("KEY log mask changed: %d\n",
						log_mask & MTC_ETA_LOG_ID_KEY);
				if (log_mask & MTC_ETA_LOG_ID_KEY) {
					mtc_eta_start_key_logging();
				} else {
					mtc_eta_stop_key_logging();
				}

			}

			if ((log_mask & MTC_ETA_LOG_ID_TOUCH)
					^ (lge_mtc_eta_log_mask & MTC_ETA_LOG_ID_TOUCH)) {
				PDEBUG("TOUCH log mask changed: %d\n",
						log_mask & MTC_ETA_LOG_ID_TOUCH);
				if (log_mask & MTC_ETA_LOG_ID_TOUCH) {
					mtc_eta_start_touch_logging();
				} else {
					mtc_eta_stop_touch_logging();
				}
			}

			lge_mtc_eta_log_mask = log_mask;
			if (!lge_mtc_eta_log_mask) {
				mtc_eta_cleanup_log_list();
			}
			break;
		case LGE_ETA_IOC_GETLOG:
			rc =  mtc_eta_get_log(argp);
			break;
		default:
			printk("mtc_eta_log: wrong ioctl cmd\n");
			rc = -EINVAL;
	}

	return rc;
}

static struct file_operations mtc_eta_log_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mtc_eta_log_ioctl,
};

static struct miscdevice mtc_eta_log_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtc_eta_log",
	.fops = &mtc_eta_log_fops,
};

static int  __init mtc_eta_logger_probe(struct platform_device *pdev)
{
	return misc_register(&mtc_eta_log_dev);
}

static int mtc_eta_logger_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mtc_eta_logger_driver = {
	.driver = {
		.name = "lge_mtc_eta_logger",
		.owner = THIS_MODULE,
	},
	.probe   = mtc_eta_logger_probe,
	.remove = mtc_eta_logger_remove,
};

static int __init mtc_eta_log_init(void)
{
	return platform_driver_register(&mtc_eta_logger_driver);
}


static void __exit mtc_eta_log_exit(void)
{
	platform_driver_unregister(&mtc_eta_logger_driver);
}

module_init(mtc_eta_log_init);
module_exit(mtc_eta_log_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("MTC/ETA event logging driver");
MODULE_LICENSE("GPL v2");
