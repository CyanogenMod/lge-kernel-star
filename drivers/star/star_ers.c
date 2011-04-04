#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define STAR_ERS_TEST_PROC_FILE "driver/ers_test"
extern int create_star_ers_test_proc_file(void);
extern void remove_star_ers_test_proc_file(void);

static ssize_t ers_test_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
    int *p = 0;

    *p = 0; // kernel crash;

    return 0;
}

static ssize_t ers_test_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
    int *p = 0; 

    *p = 0; // kernel crash;

    return 0;
}

static struct file_operations star_ers_test_proc_ops = 
{
    .read = ers_test_proc_read,
    .write = ers_test_proc_write,
};

int create_star_ers_test_proc_file(void)
{
    struct proc_dir_entry *star_ers_test_proc_file = NULL;
    star_ers_test_proc_file = create_proc_entry(STAR_ERS_TEST_PROC_FILE, 0777, NULL);
    
    if (star_ers_test_proc_file) 
    {
//      star_ers_test_proc_file->owner = THIS_MODULE;
        star_ers_test_proc_file->proc_fops = &star_ers_test_proc_ops;
    } 
    else
    { 
        printk(KERN_INFO "LGE: Star ers_test proc file create failed!\n");
    }

    return 0;
}

void remove_star_ers_test_proc_file(void)
{
    remove_proc_entry(STAR_ERS_TEST_PROC_FILE, NULL);
}

static int __init star_ers_test_init(void)
{
    return create_star_ers_test_proc_file();
}

static void __exit star_ers_test_exit(void)
{
    remove_star_ers_test_proc_file();
}

module_init(star_ers_test_init);
module_exit(star_ers_test_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Star ERS Test Driver");
MODULE_LICENSE("GPL");
