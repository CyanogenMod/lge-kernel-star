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

#include "nvrm_init.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "mach/nvrm_linux.h"
#include "nvodm_query_discovery.h"

#include "star_muic.h"

#define STAR_ERS_TEST_PROC_FILE "driver/fota_test"
extern int create_star_fota_test_proc_file(void);
extern void remove_star_fota_test_proc_file(void);
extern int fota_ebl_download(void);

typedef enum
{
    USIF_UART,
    USIF_IPC,
    USIF_NONE,
} USIF_MODE_TYPE;

extern void USIF_ctrl(USIF_MODE_TYPE mode);
extern void DP3T_Switch_ctrl(DP3T_MODE_TYPE mode);

void ifx_uart_sw_ctrl( void )
{
    printk("%s\n", __func__);
    USIF_ctrl(USIF_IPC);
    DP3T_Switch_ctrl(DP3T_NC);
}

void ifx_power_low(void)
{
    unsigned int pin_val = 0;
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s LOW\n", __func__);
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 1);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x0);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_power_low - [CP POWER]: pin %d\n", pin_val);

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
}
void ifx_power_high(void)
{
    unsigned int pin_val = 0;    
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s HIGH\n", __func__);
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 1);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x1);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_power_high - [CP POWER]: pin %d\n", pin_val);    

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
}
void ifx_reset_low(void)
{
    unsigned int pin_val = 0;        
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s LOW\n", __func__);    
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 0);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x0);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_reset_low - [CP RESET]: pin %d\n", pin_val);        

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
}
void ifx_reset_high(void)
{
    unsigned int pin_val = 0;    
    NvOdmServicesGpioHandle h_gpio;	
    NvOdmGpioPinHandle h_gpiopin;

    printk("%s HIGH\n", __func__);    
    h_gpio = NvOdmGpioOpen();
    h_gpiopin = NvOdmGpioAcquirePinHandle(h_gpio, 'v' - 'a', 0);
    NvOdmGpioConfig( h_gpio, h_gpiopin, NvOdmGpioPinMode_Output);

    NvOdmGpioSetState( h_gpio, h_gpiopin, 0x1);
    NvOdmGpioGetState( h_gpio, h_gpiopin, &pin_val);
    printk("ifx_reset_high - [CP RESET]: pin %d\n", pin_val);         

    NvOdmGpioReleasePinHandle(h_gpio, h_gpiopin);
    NvOdmGpioClose(h_gpio);
}

void ifx_fota_reset(void)
{
    fota_ebl_download();
}



static ssize_t fota_test_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
    
    return 1;
}

static ssize_t fota_test_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
    char messages[10];
    NvU32 reg, val;
    int err;
    char cmd;

    if (len > 10)
        len = 10;

    if (copy_from_user(messages, buf, len))
        return -EFAULT;

    sscanf(buf, "%c", &cmd);

    printk("%s: FOTA_proc_write \n", __func__);

    switch(cmd){
        case '0':
            ifx_power_low();
            break;
        case '1':
            ifx_power_high();
            break;
        case '2':
            ifx_reset_low();
            break;
        case '3':
            ifx_reset_high();
            break;
        case '4':
	    ifx_uart_sw_ctrl();
            break;
        case '5':
	    ifx_fota_reset();
            break;	    

	default :
            printk("FOTA Driver invalid arg\n");
            break;
    }
    return len;
}

static struct file_operations star_fota_test_proc_ops = 
{
    .read = fota_test_proc_read,
    .write = fota_test_proc_write,
};

int create_star_fota_test_proc_file(void)
{
    struct proc_dir_entry *star_fota_test_proc_file = NULL;
    star_fota_test_proc_file = create_proc_entry(STAR_ERS_TEST_PROC_FILE, 0777, NULL);
    
    if (star_fota_test_proc_file) 
    {
        star_fota_test_proc_file->proc_fops = &star_fota_test_proc_ops;
    } 
    else
    { 
        printk(KERN_INFO "LGE: Star fota_test proc file create failed!\n");
    }

    return 0;
}

void remove_star_fota_test_proc_file(void)
{
    remove_proc_entry(STAR_ERS_TEST_PROC_FILE, NULL);
}

static int __init star_fota_test_init(void)
{
    return create_star_fota_test_proc_file();
}

static void __exit star_fota_test_exit(void)
{
    remove_star_fota_test_proc_file();
}

module_init(star_fota_test_init);
module_exit(star_fota_test_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Star ERS Test Driver");
MODULE_LICENSE("GPL");
