/* drivers/rtc/alarm-dev.c
 *
 * Copyright (C) 2007-2009 Google, Inc.
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

#include <asm/mach/time.h>
#include <linux/android_alarm.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>



DECLARE_WAIT_QUEUE_HEAD(drm_wait_queue);
unsigned long drm_diffTime;
int drm_sign;

DECLARE_MUTEX(drm_mutex);

EXPORT_SYMBOL_GPL(drm_wait_queue);
EXPORT_SYMBOL_GPL(drm_diffTime);
EXPORT_SYMBOL_GPL(drm_sign);
EXPORT_SYMBOL_GPL(drm_mutex);




static long drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rv = 0;
	drm_diffTime = 0;

	switch (cmd) {
	default:
		interruptible_sleep_on(&drm_wait_queue);
		rv = drm_diffTime;
		up(&drm_mutex);

		// the changed time is less than the previous time (go to past)
		if(drm_sign>0)
			rv = 0;
		// otherwise
		else
			rv = 1;
		
		if (copy_to_user((void __user *)arg, &drm_diffTime,
		    sizeof(drm_diffTime))) {
			rv = -1;
		}
		
		break;
	}
	return rv;

}

static int drm_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static int drm_release(struct inode *inode, struct file *file)
{
	
	return 0;
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = drm_ioctl,
	.open = drm_open,
	.release = drm_release,
};

static struct miscdevice drm_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "drm",
	.fops = &drm_fops,
};

static int __init drm_dev_init(void)
{
	int err;
	int i;

	err = misc_register(&drm_device);
	if (err)
		return err;

	return 0;
}

static void  __exit drm_dev_exit(void)
{
	misc_deregister(&drm_device);
}

module_init(drm_dev_init);
module_exit(drm_dev_exit);

