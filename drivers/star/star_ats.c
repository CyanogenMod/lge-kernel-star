/*
 * drivers/star/star_ats.c
 *
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
#include <mach/lprintk.h>


#define NV_DEBUG 0

#include "nvcommon.h"

#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "nvrm_gpio.h"

extern int event_log_mask(unsigned int mask);
extern int event_log_start(void);
extern int event_log_end(void);

/* AT%GKPD [START] */
#include <linux/input.h>

int gkpd_mode = 0;
char gkpd_buf[21] = "";

int get_gkpd_mode()
{
    return gkpd_mode;
}
EXPORT_SYMBOL(get_gkpd_mode);

void add_gkpd_buf(int code)
{
    char c;
    int i;

    printk("%s: code[%d]\n", __func__, code);

    if (code == KEY_MENU) c = 'O';
    else if (code == KEY_HOME) c = '!';
    else if (code == KEY_VOLUMEUP) c = 'U';
    else if (code == KEY_VOLUMEDOWN) c = 'D';
    else if (code == KEY_SEARCH) c = 'T';
    else if (code == KEY_BACK) c = '^';
    else if (code == KEY_HOOK) c = 'K';
    else if (code == KEY_POWER) c = 'E';
    else return;

    for (i = 0; i < 20; i++)
    {
        if (gkpd_buf[i] == 0)
        {
            gkpd_buf[i] = c;
            break;
        }
    }
}
EXPORT_SYMBOL(add_gkpd_buf);
/* AT%GKPD [END] */

static ssize_t
star_ats_show_control(struct device *dev, struct device_attribute *attr, char *buf)
{
    strcpy(buf, gkpd_buf);
    printk("%s: gkpd_buf[%s]\n", __func__, gkpd_buf);
    printk("%s: strlen(gkpd_buf)[%d]\n", __func__, strlen(gkpd_buf));
	return strlen(gkpd_buf);
}

static ssize_t
star_ats_store_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int mask;

	sscanf(buf, "%u", &mask);
	printk("star_ats_store_control(): mask = %u\n", mask);

    /* AT%GKPD [START] */
    if (mask == 10) { gkpd_mode = 0; memset(gkpd_buf, 0, 21); event_log_end(); return count; }
    else if (mask == 11) { gkpd_mode = 1; memset(gkpd_buf, 0, 21); event_log_start(); return count; }
    else if (mask == 12) { memset(gkpd_buf, 0, 21); return count; }
    /* AT%GKPD [END] */

    event_log_mask(mask);

	return count;
}

static DEVICE_ATTR(control, 0666, star_ats_show_control, star_ats_store_control);


static struct attribute *star_ats_attributes[] = {
	&dev_attr_control.attr,	
	NULL,
};


static const struct attribute_group star_ats_group = {
	.attrs = star_ats_attributes,
};

static int star_ats_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
        int retval; 		
	if (sysfs_create_group(&dev->kobj, &star_ats_group)) {

		printk("[star ats] Failed to create sys filesystem\n");
		retval = -ENOSYS;
		goto err;
	}

    return 0;

err:
	return retval;
}


static struct platform_device star_ats_device = {
	.name = "star_ats",
	.id = 0,
};


static struct platform_driver star_ats_driver = {
    .probe		= star_ats_probe,
    .driver		= {
        .name = "star_ats",
        .owner = THIS_MODULE,
    },
};


static int __init star_ats_init(void)
{
    int retval = 0;
	
	
    retval = platform_device_register(&star_ats_device);
    if (retval < 0) {

		printk(KERN_ERR "platform_device_register failed!\n");
		goto out;
	}

    retval =  platform_driver_register(&star_ats_driver);
    if (retval < 0) {

		printk(KERN_ERR "platform_driver_register failed!\n");
		goto out;
	}

out:
	return retval;
}


static void __exit star_ats_exit(void)
{
	return platform_driver_unregister(&star_ats_driver);
}


module_init(star_ats_init);
module_exit(star_ats_exit);

MODULE_LICENSE("Dual BSD/GPL");


