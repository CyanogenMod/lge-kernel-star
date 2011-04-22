#include <linux/fs.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/current.h>
#include "star_sensors.h"

#if 1
DECLARE_WAIT_QUEUE_HEAD(sensor_wait);
spinlock_t wait_queue_lock;
int sensor_boot_flag; 
struct task_struct *sensor_task = NULL;

void sensors_wake_up_now(void)
{
	printk("%s %d is called\n", __func__, __LINE__);
	wake_up(&sensor_wait);
}
EXPORT_SYMBOL(sensors_wake_up_now);
#endif

static int star_sensor_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag = 0;

	sensor_task = current; 

	switch (cmd) {
		case MOTION_IOCTL_WAIT_SENSOR_INIT:
			{
#if 1
				/* wait for all sensor initialization */
				DECLARE_WAITQUEUE(wait_queue, current);
				while (!sensor_boot_flag)
//				while (!(gyro_init_done && accel_init_done && compass_init_done))
				{

					add_wait_queue(&sensor_wait, &wait_queue);
					set_current_state(TASK_INTERRUPTIBLE);
					schedule();
//					schedule_timeout(HZ/2);
					set_current_state(TASK_RUNNING);
					remove_wait_queue(&sensor_wait, &wait_queue);
					if (sensor_boot_flag) {
						break;
					}
#if 0
					if (gyro_init_done && accel_init_done && compass_init_done) {
						if (boot_flag) {
							break;
						}
					}
#endif
				}

				flag = 1;
#else
				if (atomic_read(&accel_init) && atomic_read(&compass_init) && atomic_read(&gyro_init)) {
					flag = 1;
				}
#endif
				copy_to_user(argp, &flag, sizeof(flag));

				break;
			}
		default:
			break;
	}
	return 0;
}

static int star_sensor_open(struct inode *inode, struct file *file)
{
	int status = -1;
	status = nonseekable_open(inode,file);

	return status;
}

static int star_sensor_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations star_sensor_fops = {
	.owner    = THIS_MODULE,
	.open     = star_sensor_open,
	.release  = star_sensor_release,
	.ioctl    = star_sensor_ioctl,
};

static struct miscdevice  star_sensor_misc_device = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = STAR_SENSOR_IOCTL_NAME,
	.fops   = &star_sensor_fops,
};

static int __init star_sensor_init(void)
{
	int err;

	err = misc_register(&star_sensor_misc_device);
	return 0;
}
static void __exit star_sensor_exit(void)
{
#if 1
	printk(KERN_INFO "[MPU3050] lge star_motion_exit was unloaded!\nHave a nice day!\n");
#endif
	misc_deregister(&star_sensor_misc_device);
	return;
}

module_init(star_sensor_init);
module_exit(star_sensor_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Check module for sensor initialization");
MODULE_LICENSE("GPL");

