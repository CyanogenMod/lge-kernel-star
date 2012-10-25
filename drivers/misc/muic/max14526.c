/*
 * drivers/misc/muic/max14526.c
 *
 * max14526 MUIC i2c client driver
 *
 * Copyright (c) 2010, LG Electronics Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/freezer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

//#include <mach/gpio-names.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include <mach/hardware.h>
#include <asm/setup.h>
#include <linux/syscalls.h>
#include <asm/system.h>


#include <linux/max14526.h>


#define GPIO_MUIC_INT_N				TEGRA_GPIO_PU0

//=== DP3T switch=========================
//AP20_UART_SW
#define GPIO_AP20_UART_SW			TEGRA_GPIO_PU1

//MDM_UART_SW
#define GPIO_CP_UART_SW			TEGRA_GPIO_PU2 

//MDM_USB_VBUS_EN
#define GPIO_CP_USB_VBUS_EN		TEGRA_GPIO_PR7

//=== DP3T switch=========================


#define ALLOW_SUSPEND(status) 	(((status & VBUS_M) == 0 && (status & IDNO_M) == IDNO_OPEN) || is_host)

typedef struct max14526_muic_dev {
	struct i2c_client  *client;
	struct work_struct  work;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock  wlock;
#endif
} max14526_muic_dev;

int is_host = 0;
static max14526_muic_dev *muic_dev;
static struct work_struct muic_wq;

int muic_path;
extern int muic_status;



max14526_device_type current_device = DEVICE_NONE;

static struct proc_dir_entry *max14526_muic_proc_file;
#define    MAX14526_MUIC_PROC_FILE    "driver/muic-status"

static ssize_t muic_proc_read (struct file *filp, char *buf, size_t len, loff_t *offset)
{
    printk(KERN_WARNING "%d\n", current_device);

    return 0;
}

static struct file_operations max14526_muic_proc_ops = {
    .read = muic_proc_read,
    //.write = muic_proc_write,
};

static void create_max14526_muic_proc_file(void)
{
    max14526_muic_proc_file = create_proc_entry(MAX14526_MUIC_PROC_FILE, 0777, NULL);
    if (max14526_muic_proc_file) {
        max14526_muic_proc_file->proc_fops = &max14526_muic_proc_ops;
    } else
        printk(KERN_INFO "%s: MUIC Proc file create failed!\n", __func__);
        //lprintk(D_MUIC, KERN_INFO "%s: MUIC Proc file create failed!\n", __func__);
}

static void remove_max14526_muic_proc_file(void)
{
    remove_proc_entry(MAX14526_MUIC_PROC_FILE, NULL);
}

max14526_device_type read_device_type (void)
{
    return current_device;
}
EXPORT_SYMBOL(read_device_type);

//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk
static void DP3T_Switch_ctrl(DP3T_MODE_TYPE mode)
{
    if(mode == DP3T_S1_AP) {
#if defined(CONFIG_MACH_MAX14526)
				gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);
//   		  NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
				gpio_set_value(GPIO_AP20_UART_SW, 1);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x1);
				gpio_set_value(GPIO_CP_UART_SW, 0);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x0);
		printk("[MUIC] DP3T Switch is S1_AP\n");
//        lprintk(D_MUIC, "%s: DP3T Switch is S1_AP\n", __func__);
    }
    else if(mode == DP3T_S2_CP_SW) {
#if defined(CONFIG_MACH_MAX14526)
				gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);
//			NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
				gpio_set_value(GPIO_AP20_UART_SW, 0);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x0);

				gpio_set_value(GPIO_CP_UART_SW, 1);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x1);

		printk("[MUIC] DP3T Switch is CP_SW\n");
//        lprintk(D_MUIC, "%s: DP3T Switch is CP_SW\n", __func__);
    }
    else if(mode == DP3T_S3_CP_USB) {
#if defined(CONFIG_MACH_MAX14526)
				gpio_set_value(GPIO_CP_USB_VBUS_EN, 1);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, ENABLE);
#endif
				gpio_set_value(GPIO_AP20_UART_SW, 1);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x1);

				gpio_set_value(GPIO_CP_UART_SW, 1);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x1);
		printk("[MUIC] DP3T Switch is CP_USB\n");
//        lprintk(D_MUIC, "%s: DP3T Switch is CP_USB\n", __func__);
    }
    else if (mode == DP3T_NC) {
#if defined(CONFIG_MACH_MAX14526)
				gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
				gpio_set_value(GPIO_AP20_UART_SW, 0);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x0);

				gpio_set_value(GPIO_CP_UART_SW, 0);
//        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x0);
		printk("[MUIC] DP3T Switch is NC\n");
//        lprintk(D_MUIC, "%s: DP3T Switch is NC \n", __func__);
    }
    //    DP3T_mode = mode;
}
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk



static int muic_write_register(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int muic_read_register(struct i2c_client *client, int reg)
{
	unsigned char value = 0xff;

	struct i2c_msg msg[] = {
		{ .addr = client->addr, .flags = 0, .buf = (u8 *)&reg, .len = 1 },
		{ .addr = client->addr, .flags = I2C_M_RD, .buf = &value, .len = 1 }
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2)
	{
		dev_err(&client->dev, "i2c read error\n");
		return -EIO;
	}

	return value;
}

#if 0
static void muic_switch_gpio_control(enum muic_port_setting_type mode)
{
	if(mode == PORT_SETTING_AP_UART)
	{
		gpio_set_value(GPIO_CP_UART_SW, 0);
	}
	else if(mode == PORT_SETTING_CP_UART)
	{
		gpio_set_value(GPIO_CP_UART_SW, 1);
	}

	/* Handle GPIO to enable VBUS for CP USB */
	if(mode == PORT_SETTING_CP_USB)
	{
		gpio_set_value(GPIO_CP_USB_VBUS_EN, (muic_read_register(muic_dev->client, INT_STAT_REG) & VBUS_M) ? 1 : 0);
	}
	else
	{
		gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);
	}
}
#endif

void muic_set_ap_usb_mode(void)
{
	printk("[MUIC] Set AP USB mode\n");
	DP3T_Switch_ctrl(DP3T_NC);
//	muic_switch_gpio_control(PORT_SETTING_AP_USB);

	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_DP2|COMN1_TO_DN1);
	muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);
    current_device = DEVICE_USB_CABLE;
}

//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] COMP2_TO_AUD2, COMN1_TO_AUD1 is changed to COMP2_TO_U2, COMN1_TO_U1
void muic_set_ap_uart_mode(void)
{
	printk("[MUIC] Set AP UART mode\n");
	DP3T_Switch_ctrl(DP3T_S1_AP);
//	muic_switch_gpio_control(PORT_SETTING_AP_UART);

	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);
	muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);
    current_device = DEVICE_UART_CABLE;
}

void muic_set_cp_uart_mode(void)
{
	printk("[MUIC] Set CP UART mode\n");
//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] DP3T_CP_UART is be changed to DP3T_S2_CP_SW
	DP3T_Switch_ctrl(DP3T_S2_CP_SW);
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20]
//	muic_switch_gpio_control(PORT_SETTING_CP_UART);

	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);
	muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);
    current_device = DEVICE_UART_CABLE;
}
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] COMP2_TO_AUD2, COMN1_TO_AUD1 is changed to COMP2_TO_U2, COMN1_TO_U1

void muic_set_cp_usb_mode(void)
{
	printk("[MUIC] Set CP USB mode\n");
//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] DP3T_CP_USB is be changed to DP3T_S3_CP_USB
	DP3T_Switch_ctrl(DP3T_S3_CP_USB);
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20]
//	muic_switch_gpio_control(PORT_SETTING_CP_USB);

	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);
	muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);
    current_device = DEVICE_CP_USB_CABLE;
}


void muic_set_switch_mode(int mode)
{
	muic_path = mode;
	switch(mode)
	{
		case PORT_SETTING_AUTO:
		case PORT_SETTING_AP_USB:
			muic_set_ap_usb_mode();
			break;
		case PORT_SETTING_AP_UART:
			muic_set_ap_uart_mode();
			break;
		case PORT_SETTING_CP_UART:
			muic_set_cp_uart_mode();
			break;
		case PORT_SETTING_CP_USB:
			muic_set_cp_usb_mode();
			break;
		default:
			break;
	}
}

void muic_device_init(struct i2c_client *client)
{
	muic_set_ap_usb_mode();
}

void muic_device_detection(struct i2c_client *client)
{
	int reg_value;
	static int prev_reg_value = -1;

	/* Read INT_STAT(0x04) register to check cable status such as USB_ID and VBUS.
	* It also clears pending interrupts by reading INT_STAT register.
	* NOTE that the MAX14526 only report the status of the accessory detection
	* in the INT_STAT and STATUS register. No automatic actions are done.
	* The user must read the status registers to identify the accessory
	* and write to the control retisters to set the working mode
	*/
	reg_value = muic_read_register(muic_dev->client, INT_STAT_REG);
	printk("[MUIC] MAX14526 INT_STAT_REG : 0x%2x\n", reg_value);

#ifdef CONFIG_HAS_WAKELOCK
	if(ALLOW_SUSPEND(reg_value))
	{
//		wake_unlock(&muic_dev->wlock);
	}
	else
	{
//		wake_lock(&muic_dev->wlock);
	}
#endif

	if(muic_path == PORT_SETTING_CP_USB)
		gpio_set_value(GPIO_CP_USB_VBUS_EN, (reg_value & VBUS_M) ? 1 : 0);

	/* If muic_path is not auto mode, skip below process. */
	if(muic_path != PORT_SETTING_AUTO) return;

	reg_value &= IDNO_M;
	if(reg_value != prev_reg_value)
	{
		/* Check USB id pin to determine the path on the MUIC.
		* These configuration is valid only for LGE max14526 Tablet device.
		*/
		switch((reg_value & IDNO_M))
		{
			/* IDNO_0010 : 56k ohm cable, connect to CP UART port */
			case IDNO_0010:
				muic_set_cp_uart_mode();
				break;

			/* IDNO_0100 : 130k ohm cable, connect to AP UART port */
			case IDNO_0100:
				muic_set_ap_uart_mode();
				break;

			/* IDNO_1010 : 910k ohm cable, restart the machine to entering download mode */
			case IDNO_1010:
				arm_machine_restart('h', NULL);
				break;

			case IDNO_1011:
				current_device = DEVICE_NONE;//disconnected
				break;

			default:
				muic_device_init(client);
				break;
		}
	}

	


	prev_reg_value = reg_value;
}

#ifdef CONFIG_SYSFS
void muic_nv_write(int mode)
{
	int fd;
	char buf[20];
	mm_segment_t old_fs = get_fs();

	memset(buf, 0x0, sizeof(buf));
	set_fs(KERNEL_DS);
	fd = sys_open("/proc/nvdata/MUIC_PATH", O_WRONLY | O_CREAT, 0666);
	if(fd < 0)
	{
		printk("[MUIC] Could not open NV partition.\n");
		return;
	}
	sys_lseek(fd, 0, SEEK_SET);
	sprintf(buf, "%d", mode);
	sys_write(fd, buf, sizeof(buf));
	sys_close(fd);
	set_fs(old_fs);
}

static ssize_t muic_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", muic_path);
}

static ssize_t muic_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int mode =0;

	if(sscanf(buf, "%d", &mode) != 1)
		return size;

	muic_set_switch_mode(mode);

	muic_nv_write(mode);


	if(mode == PORT_SETTING_AUTO)
	{
		muic_device_init(muic_dev->client);
		muic_device_detection(muic_dev->client);
	}
	return size;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO, muic_mode_show, muic_mode_store);
#endif

static void muic_wq_func(struct work_struct *muic_wq)
{
	muic_device_detection(muic_dev->client);
}


static irqreturn_t muic_interrupt_handler(void* arg)
{
	schedule_work(&muic_wq);
	return IRQ_HANDLED; 
}

static int __init muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;
	int reg_value;

	muic_dev = kzalloc(sizeof(*muic_dev), GFP_KERNEL);
	if (!muic_dev)
	{
		return -ENOMEM;
	}
	memset(muic_dev, 0, sizeof(*muic_dev));
	muic_dev->client = client;

	i2c_set_clientdata(client, muic_dev);

	gpio_request(GPIO_MUIC_INT_N, "muic_int_n");
	tegra_gpio_enable(GPIO_MUIC_INT_N);
	gpio_direction_input(GPIO_MUIC_INT_N);

	gpio_request(GPIO_CP_UART_SW, "cp_uart_sw");
	tegra_gpio_enable(GPIO_CP_UART_SW);
	gpio_direction_output(GPIO_CP_UART_SW, 0);

	gpio_request(GPIO_CP_USB_VBUS_EN, "cp_usb_vbus_en");
	tegra_gpio_enable(GPIO_CP_USB_VBUS_EN);
	gpio_direction_output(GPIO_CP_USB_VBUS_EN, 0);

#if 0
	if (get_hw_rev() >= REV_G)
	{
		gpio_request(GPIO_CP_USB_SW, "cp_usb_sw");
		tegra_gpio_enable(GPIO_CP_USB_SW);
		gpio_direction_output(GPIO_CP_USB_SW, 0);
	}
#endif

	INIT_WORK(&muic_wq, muic_wq_func);
	error = request_irq(gpio_to_irq(GPIO_MUIC_INT_N), (irq_handler_t)muic_interrupt_handler,
						IRQF_TRIGGER_FALLING, "muic_int_n", (void*)&muic_dev);
	if(error)
	{
		printk(KERN_ERR "[MUIC] interrupt registeration fail! (err:%d)\n", error);
		goto err_request_irq_fail;
	}

#ifdef CONFIG_HAS_WAKELOCK
//	wake_lock_init(&muic_dev->wlock, WAKE_LOCK_SUSPEND, "muic_wakelock");
#endif

#ifdef CONFIG_SYSFS
	if(device_create_file(&client->dev, &dev_attr_mode))
	{
		printk("[MUIC] mode file create -error \n");
	}
#endif
    create_max14526_muic_proc_file();


	if(muic_path != PORT_SETTING_AUTO)
	{
		muic_set_switch_mode(muic_path);
	}

	reg_value = muic_read_register(muic_dev->client, INT_STAT_REG);
	printk("[MUIC] %s : muic_path = %d, reg_value = %02x\n", __func__, muic_path, reg_value);
#ifdef CONFIG_HAS_WAKELOCK
	if(!ALLOW_SUSPEND(reg_value))
	{
//		wake_lock(&muic_dev->wlock);
	}
#endif
	muic_device_detection(muic_dev->client);

	return 0;

err_request_irq_fail:
	free_irq(gpio_to_irq(GPIO_MUIC_INT_N), muic_dev);

	return -ENOSYS;
}

static int muic_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_WAKELOCK
//	wake_lock_destroy(&muic_dev->wlock);
#endif
	free_irq(gpio_to_irq(GPIO_MUIC_INT_N), muic_dev);

	if (muic_dev)
	{
		kfree(muic_dev);
		muic_dev = NULL;
	}
	device_remove_file(&client->dev, &dev_attr_mode);
    remove_max14526_muic_proc_file();
	return 0;
}

//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk
static int muic_suspend(struct i2c_client *client, pm_message_t state)
{
	printk("[MUIC] MUIC is Suspend\n");
//	lprintk(D_MUIC, "%s \n", __func__);
	// Disable Interrupts (0x02 = 0x00)
	muic_write_register(muic_dev->client, CTRL2_REG, INT_DIS_M);
	printk("[MUIC] MUIC Control2 Register Interrupt Disable\n");
//	lprintk(D_MUIC, "%s Interrupt Disable\n", __func__);
	return 0;
}
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk

static int muic_resume(struct i2c_client *client)
{
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);

	muic_device_detection(muic_dev->client);
	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{ "max14526", 0 },
	{ /* end of list */ },
};

static struct i2c_driver muic_driver = {
	.probe     = muic_probe,
	.remove    = muic_remove,
	.suspend   = muic_suspend,
	.resume    = muic_resume,
	.id_table  = muic_ids,
	.driver    = {
		.name = "max14526",
		.owner = THIS_MODULE,
	},
};

static int __devinit muic_init(void)
{
	return i2c_add_driver(&muic_driver);
}

static void __exit muic_exit(void)
{
	i2c_del_driver(&muic_driver);
}

module_init(muic_init);
module_exit(muic_exit);

MODULE_AUTHOR("sangmin978.lee@lge.com");
MODULE_DESCRIPTION("MUIC Driver for LGE MAX14526");
MODULE_LICENSE("GPL");
