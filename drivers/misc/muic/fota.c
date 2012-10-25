/*
 * Cosmo FOTA driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Chulhwheeshim <Chulhwhee.shim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <asm/system.h>

/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
//#include <linux/spi/ifx_n721_spi.h>
#include <linux/spi/ifx_modem.h>

//TIME INFORMATION ADD. 2011-05-02 eunae.kim
#include <linux/rtc.h>
/*
#define MODEM_PWR_ON		TEGRA_GPIO_PO0
#define MODEM_RESET		TEGRA_GPIO_PV1
*/

#include "../gpio-names.h"

	
#include <linux/muic.h>
	
//#define GPIO_CP_POWER  	    TEGRA_GPIO_PV1  //CP_POWER
#define MODEM_GPIO_PWRON_SW  	TEGRA_GPIO_PO0  //CP_POWER
//#define GPIO_CP_RESET  	    TEGRA_GPIO_PV0  //CP_RESET
#define MODEM_GPIO_PWRON  	    TEGRA_GPIO_PV1  //CP_RESET




#define COSMO_FOTA_TEST_PROC_FILE "driver/fota"

extern int fota_ebl_download(void);
extern void usif_switch_ctrl(TYPE_USIF_MODE mode);
extern void dp3t_switch_ctrl(TYPE_DP3T_MODE mode);
extern void fota_cp_reset(void);


void cp_power_down(void){
	printk("=============== cp_power_down ================\n");
	// PMU Reset Pin Low
	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 0);
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON, 0);
	// Power On Pin Low
	gpio_direction_output(MODEM_GPIO_PWRON_SW, 0);  //GPIO_3 OE
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON_SW, 0);
}

void ifx_uart_sw_ctrl( void )
{
    printk("%s\n", __func__);
    usif_switch_ctrl(0);
   // dp3t_switch_ctrl(0);
}

void ifx_power_low(void)
{
	int status;
	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 0);
	mdelay(500);
	gpio_set_value(MODEM_GPIO_PWRON, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("ifx_power_low - [CP POWER]: pin %d\n",status);  
	printk("%s: FOTA_proc_excute \n", __func__);
	return;
}

void ifx_power_high(void)
{
	int status;
	printk("%s: FOTA_proc_excute \n", __func__);
	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 1);
	mdelay(500);
	gpio_set_value(MODEM_GPIO_PWRON, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("ifx_power_high - [CP POWER]: pin %d\n",status);
	return;	
}

void ifx_reset_low(void)
{
	int status;
	return;
}
void ifx_reset_high(void)
{
	int status;
	return;
}

void ifx_fota_reset(void)
{
#if 1 // jdpark : fota reset
#else
	usif_switch_ctrl(0);
#endif

	fota_cp_reset();
	
/*
	int status;

	printk("%s: CP Reset IN\n", __func__);

	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 0);
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("%s: MODEM_GPIO_PWRON low- [CP POWER]: pin %d\n", __func__, status);

	mdelay(500); // 500mS delay
	
	gpio_direction_output(MODEM_GPIO_PWRON_SW, 0);  //GPIO_3 OE
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON_SW, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON_SW);
	printk("%s: MODEM_GPIO_PWRON_SW low- [CP POWER]: pin %d\n", __func__, status);
	
	mdelay(500); // 500mS delay

	gpio_set_value(MODEM_GPIO_PWRON, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("%s: MODEM_GPIO_PWRON high+ [CP POWER]: pin %d\n", __func__, status);

	mdelay(100); // 1000mS delay

	gpio_set_value(MODEM_GPIO_PWRON_SW, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON_SW);
	printk("%s: MODEM_GPIO_PWRON_SW high+ [CP POWER]: pin %d\n", __func__, status);

	usif_switch_ctrl(0);

	printk("%s: CP Reset OUT\n", __func__);
	*/
}

void ifx_pmu_reset(void)
{
	int status;

// TIME INFORMATION ADD. 2011-05-02 eunae.kim
        struct timespec ts;
        struct rtc_time tm;
        getnstimeofday(&ts);
        rtc_time_to_tm(ts.tv_sec, &tm);

	printk("%s:", __func__);
	printk("(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                        tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 0);
    mdelay(200);
	gpio_set_value(MODEM_GPIO_PWRON, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("ifx_power_low- [CP POWER]: pin %d\n",status);
	mdelay(200);
	gpio_set_value(MODEM_GPIO_PWRON, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("ifx_power_high- [CP POWER]: pin %d\n",status);
//cheolgwak 2011 02 01
gpio_direction_output(MODEM_GPIO_PWRON_SW, 0);  //GPIO_3 OE
udelay(100);
gpio_set_value(MODEM_GPIO_PWRON_SW, 0);
mdelay(200);  //wait 10ms
gpio_set_value(MODEM_GPIO_PWRON_SW, 1);
mdelay(200);
//cheolgwak 2011 02 01
}


void ifx_fota_reset_long(void)
{
	int status;

	printk("%s: CP Reset IN\n", __func__);

	gpio_request(MODEM_GPIO_PWRON, "ifx pwron");
	gpio_direction_output(MODEM_GPIO_PWRON, 0);
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("%s: MODEM_GPIO_PWRON low- [CP POWER]: pin %d\n", __func__, status);

	mdelay(4000); // 4000mS delay

	gpio_set_value(MODEM_GPIO_PWRON, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON);
	printk("%s: MODEM_GPIO_PWRON high+ [CP POWER]: pin %d\n", __func__, status);

	mdelay(200);

	gpio_direction_output(MODEM_GPIO_PWRON_SW, 0);  //GPIO_3 OE
	udelay(100);
	gpio_set_value(MODEM_GPIO_PWRON_SW, 0);
	status = gpio_get_value(MODEM_GPIO_PWRON_SW);
	printk("%s: MODEM_GPIO_PWRON_SW low- [CP POWER]: pin %d\n", __func__, status);

	mdelay(1000); // 1000mS delay

	gpio_set_value(MODEM_GPIO_PWRON_SW, 1);
	status = gpio_get_value(MODEM_GPIO_PWRON_SW);
	printk("%s: MODEM_GPIO_PWRON_SW high+ [CP POWER]: pin %d\n", __func__, status);
	mdelay(200);

	usif_switch_ctrl(0);

	printk("%s: CP Reset OUT\n", __func__);
}

static ssize_t fota_test_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
    return 1;
}

static ssize_t fota_test_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
    char messages[10];
    u32 reg, val;
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
        case '6':
	    ifx_pmu_reset();
            break;	    
		case '7':
	    ifx_fota_reset_long();
            break;	   

	default :
            printk("FOTA Driver invalid arg\n");
            break;
    }
    return len;
}

static struct file_operations Cosmo_fota_test_proc_ops = 
{
    .read = fota_test_proc_read,
    .write = fota_test_proc_write,
};

int create_Cosmo_fota_test_proc_file(void)
{
    struct proc_dir_entry *Cosmo_fota_test_proc_file = NULL;
    Cosmo_fota_test_proc_file = create_proc_entry(COSMO_FOTA_TEST_PROC_FILE, 0777, NULL);
    
    if (Cosmo_fota_test_proc_file) 
    {
        Cosmo_fota_test_proc_file->proc_fops = &Cosmo_fota_test_proc_ops;
    } 
    else
    { 
        printk(KERN_INFO "LGE: Star fota_test proc file create failed!\n");
    }

    return 0;
}

void remove_Cosmo_fota_test_proc_file(void)
{
    remove_proc_entry(COSMO_FOTA_TEST_PROC_FILE, NULL);
}

static int __init Cosmo_fota_test_init(void)
{
    return create_Cosmo_fota_test_proc_file();
}

static void __exit Cosmo_fota_test_exit(void)
{
    remove_Cosmo_fota_test_proc_file();
}

module_init(Cosmo_fota_test_init);
module_exit(Cosmo_fota_test_exit);


MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Cosmo FOTA Driver");
MODULE_LICENSE("GPL");

