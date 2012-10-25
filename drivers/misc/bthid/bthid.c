/************************************************************************************
 *
 *  Copyright (C) 2009-2011 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2, as published by
 *  the Free Software Foundation (the "GPL").
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php,
 *  or by writing to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 *  Boston, MA  02111-1307, USA.
 *
 ************************************************************************************/
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/hid.h>


MODULE_AUTHOR("Daniel McDowell <mcdowell@broadcom.com>");
MODULE_DESCRIPTION("User level driver support for Bluetooth HID input");
MODULE_SUPPORTED_DEVICE("bthid");
MODULE_LICENSE("GPL");


#define BTHID_NAME              "bthid"
#define BTHID_MINOR             224
#define BTHID_IOCTL_HID_INFO    1
#define BTHID_MAX_HID_INFO_LEN  884
#define BTHID_MAX_DEV_NAME_LEN  128


struct bthid_ctrl {
    int   dscp_len;
    char  dscp_buf[BTHID_MAX_HID_INFO_LEN];
    char  dev_name[BTHID_MAX_DEV_NAME_LEN];
    unsigned short vendor_id;
    unsigned short product_id;
    unsigned short version;
    unsigned short ctry_code;
};

struct bthid_device {
    struct input_dev   *dev;
    struct hid_device  *hid;
    int                dscp_set;
};


static int bthid_ll_start(struct hid_device *hid)
{
    printk("######## bthid_ll_start: hid = %p ########\n", hid);
    return 0;
}

static void bthid_ll_stop(struct hid_device *hid)
{
    printk("######## bthid_ll_stop: hid = %p ########\n", hid);
}

static int bthid_ll_open(struct hid_device *hid)
{
    printk("######## bthid_ll_open: hid = %p ########\n", hid);
    return 0;
}

static void bthid_ll_close(struct hid_device *hid)
{
    printk("######## bthid_ll_close: hid = %p ########\n", hid);
}

static int bthid_ll_hidinput_event(struct input_dev *dev, unsigned int type, 
                                   unsigned int code, int value)
{
    /*
    printk("######## bthid_ll_hidinput_event: dev = %p, type = %d, code = %d, value = %d ########\n",
           dev, type, code, value);
    */
    return 0;
}

static int bthid_ll_parse(struct hid_device *hid)
{
    int ret;
    unsigned char *buf;
    struct bthid_ctrl *p_ctrl = hid->driver_data;

    printk("######## bthid_ll_parse: hid = %p ########\n", hid);
    
    buf = kmalloc(p_ctrl->dscp_len, GFP_KERNEL);
    if (!buf)
    {
        return -ENOMEM;
    }

    memcpy(buf, p_ctrl->dscp_buf, p_ctrl->dscp_len);

    ret = hid_parse_report(hid, buf, p_ctrl->dscp_len);
    kfree(buf);

    printk("######## bthid_ll_parse: status = %d, ret = %d ########\n", hid->status, ret);

    return ret;
}

static struct hid_ll_driver bthid_ll_driver = {
    .start                = bthid_ll_start,
    .stop                 = bthid_ll_stop,
    .open                 = bthid_ll_open,
    .close                = bthid_ll_close,
    .hidinput_input_event = bthid_ll_hidinput_event,
    .parse                = bthid_ll_parse,
};


static int bthid_open(struct inode *inode, struct file *file)
{
    struct bthid_device *p_dev;

    printk("######## bthid_open: ########\n");

    p_dev = kzalloc(sizeof(struct bthid_device), GFP_KERNEL);
    if (!p_dev)
    {
        return -ENOMEM;
    }

    file->private_data = p_dev;
    
    printk("######## bthid_open: done ########\n");
    return 0;
}

static int bthid_release(struct inode *inode, struct file *file)
{
    struct bthid_device *p_dev = file->private_data;

    printk("######## bthid_release: ########\n");
    
    if (p_dev->hid) 
    {
        if (p_dev->hid->status == (HID_STAT_ADDED | HID_STAT_PARSED))
        {
            hidinput_disconnect(p_dev->hid);
        }

        if (p_dev->hid->driver_data != NULL)
        {
            kfree(p_dev->hid->driver_data);
        }

        hid_destroy_device(p_dev->hid);
        p_dev->hid = NULL;
    }

    kfree(p_dev);
    file->private_data = NULL;

    printk("######## bthid_release: done ########\n");
    return 0;
}

static ssize_t bthid_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    unsigned char *buf;
    struct bthid_device *p_dev = file->private_data;

    /*
    printk("######## bthid_write: count = %d ########\n", count);
    */

    if (p_dev->dscp_set == 0)
    {
        printk("bthid_write: Error:, HID report descriptor not configured\n");
        return 0;
    }

    buf = kmalloc(count + 1, GFP_KERNEL);
    if (!buf)
    {
        return -ENOMEM;
    }

    if (copy_from_user(buf, buffer, count))
    {
        kfree(buf);
        return -EFAULT;
    }

    if (p_dev->hid) 
    {
        hid_input_report(p_dev->hid, HID_INPUT_REPORT, buf, count, 1);
    }

    kfree(buf);

    /*
    printk("######## bthid_write: done ########\n");
    */

    return 0;
}

static int bthid_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret;
    struct bthid_ctrl *p_ctrl;
    struct bthid_device *p_dev = file->private_data;

    printk("######## bthid_unlocked_ioctl: cmd = %d ########\n", cmd);

    if (cmd != BTHID_IOCTL_HID_INFO || p_dev == NULL)
    {
        return -EINVAL;
    }

    p_ctrl = kmalloc(sizeof(struct bthid_ctrl), GFP_KERNEL);
    if (p_ctrl == NULL)
    {
        return -ENOMEM;
    }

    if (copy_from_user(p_ctrl, (void __user *) arg, sizeof(struct bthid_ctrl)) != 0)
    {
        kfree(p_ctrl);
        return -EFAULT;
    }

    printk("bthid: name = [%s], VID/PID = %04x/%04x, version = 0x%04x, country = 0x%02x\n",
           p_ctrl->dev_name, p_ctrl->vendor_id, p_ctrl->product_id,
           p_ctrl->version, p_ctrl->ctry_code);

    if (p_ctrl->dscp_len <= 0) 
    {
        printk("Oops: Invalid BT HID report descriptor size %d\n", p_ctrl->dscp_len); 

        kfree(p_ctrl);
        return -EINVAL;
    }
    
    p_dev->hid = hid_allocate_device();
    if (p_dev->hid == NULL)
    {
        printk("Oops: Failed to allocation HID device.\n");

        kfree(p_ctrl);
        return -ENOMEM;
    }
    
    p_dev->hid->bus         = BUS_BLUETOOTH;
    p_dev->hid->vendor      = p_ctrl->vendor_id;
    p_dev->hid->product     = p_ctrl->product_id;
    p_dev->hid->version     = p_ctrl->version;
    p_dev->hid->country     = p_ctrl->ctry_code;
    p_dev->hid->ll_driver   = &bthid_ll_driver;
    p_dev->hid->driver_data = p_ctrl;

    // strcpy(p_dev->hid->name, "Broadcom Bluetooth HID");
    strncpy(p_dev->hid->name, p_ctrl->dev_name, BTHID_MAX_DEV_NAME_LEN);

    ret = hid_add_device(p_dev->hid);

    printk("hid_add_device: ret = %d, hid->status = %d\n", ret, p_dev->hid->status);

    if (ret != 0)
    {
        printk("Oops: Failed to add HID device");

        kfree(p_ctrl);
        hid_destroy_device(p_dev->hid);
        p_dev->hid = NULL;
        return -EINVAL;
    }
    p_dev->hid->claimed |= HID_CLAIMED_INPUT;

    if (p_dev->hid->status != (HID_STAT_ADDED | HID_STAT_PARSED))
    {
        printk("Oops: Failed to process HID report descriptor");
        return -EINVAL;
    }

    p_dev->dscp_set = 1;

    printk("######## bthid_unlocked_ioctl: done ########\n");
    return 0;
}


static const struct file_operations bthid_fops = {
    .owner   = THIS_MODULE,
    .open    = bthid_open,
    .release = bthid_release,
    .write   = bthid_write,
    .unlocked_ioctl = bthid_unlocked_ioctl,
};

static struct miscdevice bthid_misc = {
    .name  = BTHID_NAME,
    .minor = BTHID_MINOR,
    .fops  = &bthid_fops,
};


static int __init bthid_init(void)
{
    int ret;

    printk("######## bthid_init: ########\n");

    ret = misc_register(&bthid_misc);
    if (ret != 0)
    {
        printk("Error: failed to register Misc driver, ret = %d\n", ret);
        return ret;
    }

    printk("######## bthid_init: done ########\n");
    
    return ret;
}

static void __exit bthid_exit(void)
{
    printk("bthid_exit:\n");

    misc_deregister(&bthid_misc);
    printk("bthid_exit: done\n");
}

module_init(bthid_init);
module_exit(bthid_exit);
