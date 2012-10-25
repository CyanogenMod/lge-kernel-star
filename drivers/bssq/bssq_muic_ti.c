/*
 * drivers/bssq/bssq_muic_ti.c
 *
 * bssq MUIC i2c client driver
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

#include <mach/gpio-names.h>
#include <asm/setup.h>
#include <linux/syscalls.h>
#include <asm/system.h>

#include <mach/delay.h>


#include "bssq_muic_ti.h"

#if defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
#include <linux/lge_hw_rev.h>
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#if defined(CONFIG_BSSQ_CHARGER_RT)
#include <lge/bssq_charger_rt.h>
#endif
#if defined(CONFIG_BSSQ_CHARGER_MAX)
#include <lge/bssq_charger_max.h>
#endif


//#define DEBUG_AP_UART_MODE

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE  0

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
#define GPIO_MUIC_INT_N				TEGRA_GPIO_PU5 // MUIC_INT_N
#define GPIO_CP_USB_DP_DM_SW	TEGRA_GPIO_PM7 // CP USB DP DM Switch
#elif defined (CONFIG_KS1103)
#define GPIO_MUIC_INT_N				TEGRA_GPIO_PU5 // MUIC_INT_N
#else
#define GPIO_MUIC_INT_N				TEGRA_GPIO_PU0 // MUIC_INT_N
#endif

#if defined(CONFIG_LU6500)
#define GPIO_MUIC_INT_N_REV_E		TEGRA_GPIO_PW2 // MUIC_INT_N
#endif

#ifdef DEBUG_AP_UART_MODE
#define GPIO_SIDE_UP_KEY  		  TEGRA_GPIO_PG0 // SIDE_UP
#define GPIO_SIDE_DOWN_KEY  		TEGRA_GPIO_PG1 // SIDE_DOWN
#endif

//=== DP3T switch=========================
//AP20_UART_SW
//#define GPIO_AP20_UART_SW			TEGRA_GPIO_PU1

//MDM_UART_SW
#define GPIO_CP_UART_SW			TEGRA_GPIO_PU2 // UART_SW

//MDM_USB_VBUS_EN
#define GPIO_CP_USB_VBUS_EN		TEGRA_GPIO_PF1	// MDM_USB_VBUS_EN

//=== DP3T switch=========================

#define MUIC_CHGDET_MASK(reg) (reg & CHGDET_M)
#define MUIC_VBUS_MASK(reg) (reg & VBUS_M)

#define ID_PIN_56K(reg)   ((reg & IDNO_M) == IDPIN_56K)
#define ID_PIN_130K(reg)  ((reg & IDNO_M) == IDPIN_130K)
#define ID_PIN_910K(reg)  ((reg & IDNO_M) == IDPIN_910K)

#define ID_PIN_IS_FACTORY(reg) (ID_PIN_56K(reg) || ID_PIN_130K(reg) || ID_PIN_910K(reg))

#if defined(CONFIG_SU880) || defined(CONFIG_KU8800)
#define ALLOW_SUSPEND(reg) 	(!MUIC_VBUS_MASK(reg) || MUIC_CHGDET_MASK(reg))
#else
#define ALLOW_SUSPEND(reg) 	(!MUIC_VBUS_MASK(reg) || MUIC_CHGDET_MASK(reg) || ID_PIN_56K(reg))
#endif

//#define ALLOW_SUSPEND(reg_value) 	(((reg_value & VBUS_M) == 0 && (reg_value & IDNO_M) == IDNO_OPEN) || is_host)

typedef struct star_muic_dev {
	struct i2c_client  *client;
	struct work_struct  work;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock  wlock;
  struct wake_lock  wlock_timeout;
#endif
} star_muic_dev;


static star_muic_dev *muic_dev;
static struct delayed_work muic_wq;

muic_port_setting_type muic_path = PORT_SETTING_AUTO;

extern void notification_of_changes_to_battery(void);	// from bssq_battery.c

void muic_set_open(void);


MUIC_CHARGER_TYPE g_charger_type = MUIC_CHARGER_NONE;
int g_charger_conected = FALSE;

MUIC_CHARGER_TYPE get_muic_charger_type(void)
{
  return g_charger_type;
}

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
	printk("[MUIC] %s\n", __func__);

	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_DP2|COMN1_TO_DN1);

  muic_path = PORT_SETTING_AP_USB;
}

//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] COMP2_TO_AUD2, COMN1_TO_AUD1 is changed to COMP2_TO_U2, COMN1_TO_U1
void muic_set_ap_uart_mode(void)
{
	printk("[MUIC] %s\n", __func__);

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#elif defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_LU8800)
  if(get_lge_pcb_revision() > REV_A)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#elif defined(CONFIG_KS1103)
	if(1)
		muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
	else
#endif
	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2|COMN1_TO_U1);

  muic_path = PORT_SETTING_AP_UART;
}

void muic_set_cp_uart_mode(void)
{
	printk("[MUIC] Set CP UART mode\n");

	gpio_set_value(GPIO_CP_UART_SW, 1);

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#elif defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_LU8800)
  if(get_lge_pcb_revision() > REV_A)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#elif defined(CONFIG_KS1103)
	  if(1)
		  muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);
	  else
#endif
	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2|COMN1_TO_U1);

  muic_path = PORT_SETTING_CP_UART;
}
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] COMP2_TO_AUD2, COMN1_TO_AUD1 is changed to COMP2_TO_U2, COMN1_TO_U1

void muic_set_cp_usb_mode(void)
{
	printk("[MUIC] Set CP USB mode\n");

	gpio_set_value(GPIO_CP_USB_VBUS_EN, 1);
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
  gpio_set_value(GPIO_CP_USB_DP_DM_SW, 0);
#endif

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2|COMN1_TO_U1);
  else
#elif defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_LU8800)
  if(get_lge_pcb_revision() > REV_A)
    muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2|COMN1_TO_U1);
  else
#elif defined(CONFIG_KS1103)
	if(1)
		muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_U2|COMN1_TO_U1);
	else
#endif
	muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_AUD2|COMN1_TO_AUD1);

  muic_path = PORT_SETTING_CP_USB;
}

void muic_set_open(void)
{
	printk("[MUIC] %s\n", __func__);

	gpio_set_value(GPIO_CP_UART_SW, 0);
	gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);

  muic_write_register(muic_dev->client, SW_CTRL_REG, COMP2_TO_HZ|COMN1_TO_HZ);
	muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | SEMREN_M | CP_EN_M);	

  muic_path = PORT_SETTING_OPEN;

	mdelay(1);
}

extern int muic_boot_keeping;
extern int muic_boot_path;

void muic_set_switch_mode(muic_port_setting_type mode, int rev_check)
{
  if(rev_check && muic_boot_keeping)
    return;
  
  printk("[MUIC] %s mode : %d, muic_path : %d\n", __func__, mode, muic_path);
  
  if(muic_path == mode)
    return;

#if defined(CONFIG_LU6500)
  if(rev_check && get_lge_pcb_revision() < REV_D)
  {
    printk("[MUIC] HW Rev : %d\n", get_lge_pcb_revision());
    muic_path = PORT_SETTING_AP_USB;
    muic_set_ap_usb_mode();
    return;
  }
#endif

  if(muic_path != PORT_SETTING_OPEN)
    muic_set_open();

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
#ifdef DEBUG_AP_UART_MODE
      if(gpio_get_value(GPIO_SIDE_UP_KEY) == GPIO_LOW_VALUE && 
         gpio_get_value(GPIO_SIDE_DOWN_KEY) == GPIO_HIGH_VALUE)
        muic_set_ap_uart_mode();
      else
#endif
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
  muic_write_register(muic_dev->client, SW_CTRL_REG, 0x3F);
  muic_write_register(muic_dev->client, CTRL1_REG, ID_200_M | SEMREN_M | CP_EN_M);

  muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);

  msleep(200);

  if(muic_boot_keeping)
    muic_set_switch_mode(muic_boot_path, FALSE);
}

extern int batt_factory_mode;
void muic_set_charger_type(int reg_value)
{
	int vbus, chgdet;

	vbus = reg_value & VBUS_M;
	chgdet = reg_value & CHGDET_M;

	if(chgdet || vbus)
	{
	  if(g_charger_conected == TRUE)
      	return;
      
	  g_charger_conected = TRUE;
    
		if(ID_PIN_IS_FACTORY(reg_value))
		{
			g_charger_type = MUIC_CHARGER_FACTORY;
			printk("[MUIC] CHARGER_FACTORY\n");
			batt_factory_mode = TRUE;
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
#if defined(CONFIG_BSSQ_CHARGER_RT) || defined(CONFIG_BSSQ_CHARGER_MAX)
			charger_ic_set_mode(CHARGER_FACTORY);
#endif
#endif
		}
		else if(chgdet)
		{
			g_charger_type = MUIC_CHARGER_TA;
			printk("[MUIC] CHARGER_TA\n");
#if defined(CONFIG_BSSQ_CHARGER_RT) || defined(CONFIG_BSSQ_CHARGER_MAX)
			charger_ic_set_mode(CHARGER_ISET);
#endif
		}
		else
		{
			g_charger_type = MUIC_CHARGER_USB;
			printk("[MUIC] CHARGER_USB\n");
#if defined(CONFIG_BSSQ_CHARGER_RT) || defined(CONFIG_BSSQ_CHARGER_MAX)
			charger_ic_set_mode(CHARGER_USB500);	  
#endif
		}
	}
	else
	{
	  if(g_charger_conected == FALSE)
		return;
      
	  g_charger_conected = FALSE;
		g_charger_type = MUIC_CHARGER_NONE;
		batt_factory_mode = FALSE;
		printk("[MUIC] CHARGER_NONE\n");
#if defined(CONFIG_BSSQ_CHARGER_RT) || defined(CONFIG_BSSQ_CHARGER_MAX)
		charger_ic_disable();	
#endif
	}

#ifdef CONFIG_HAS_WAKELOCK
  wake_lock_timeout(&muic_dev->wlock_timeout, 100);
#endif

#ifdef CONFIG_BSSQ_BATTERY
	notification_of_changes_to_battery();
#endif
}

int g_cp_sleep_mode = FALSE;

#ifdef CONFIG_HAS_WAKELOCK
int muic_wake_lock_state = FALSE;

static void muic_wake_lock()
{
  if(muic_wake_lock_state)
    return;

  muic_wake_lock_state = TRUE;
  wake_lock(&muic_dev->wlock);
  printk("[MUIC] wake_lock\n");
}

static void muic_wake_unlock()
{
  if(!muic_wake_lock_state)
    return;

  muic_wake_lock_state = FALSE;
  wake_unlock(&muic_dev->wlock);
  printk("[MUIC] wake_unlock\n");
}
#endif

void muic_device_detection(struct i2c_client *client)
{
	int reg_value, vbus, chgdet;

  printk("[MUIC] %s\n", __func__);

	/* Read INT_STAT(0x04) register to check cable status such as USB_ID and VBUS.
	* It also clears pending interrupts by reading INT_STAT register.
	* NOTE that the MAX14526 only report the status of the accessory detection
	* in the INT_STAT and STATUS register. No automatic actions are done.
	* The user must read the status registers to identify the accessory
	* and write to the control retisters to set the working mode
	*/
	mdelay(1);
  
	reg_value = muic_read_register(muic_dev->client, INT_STAT_REG);
	printk("[MUIC] Detect device , INT_STAT_REG : 0x%2x\n", reg_value);

#ifdef CONFIG_HAS_WAKELOCK
	if(ALLOW_SUSPEND(reg_value) && muic_boot_keeping==FALSE)
	{ 
		muic_wake_unlock();
	}
	else
	{
		muic_wake_lock();
	}
#endif

  if(ID_PIN_56K(reg_value))
    g_cp_sleep_mode = TRUE;
  else
    g_cp_sleep_mode = FALSE;

  muic_set_charger_type(reg_value);

	vbus = reg_value & VBUS_M;
	chgdet = reg_value & CHGDET_M;

  if(chgdet)
  {
    muic_set_open();
    if(muic_boot_keeping)
      muic_wake_unlock();
  }
	else if(vbus)
	{
		/* Check USB id pin to determine the path on the MUIC.
		* These configuration is valid only for LGE STAR Tablet device.
		*/
		switch((reg_value & IDNO_M))
		{
			case IDPIN_56K:
        muic_set_switch_mode(PORT_SETTING_CP_USB, TRUE);
				break;

			case IDPIN_130K:
        muic_set_switch_mode(PORT_SETTING_CP_UART, TRUE);
				break;

			/* IDNO_1010 : 910k ohm cable, restart the machine to entering download mode */
#if 0   // BQ and X2 is not use.
			case IDPIN_910K:
				arm_machine_restart('h', NULL);
				break;
#endif

			default:
				//muic_device_init(client);
				muic_set_switch_mode(PORT_SETTING_AP_USB, TRUE);

				break;
		}
	}
  else
  {
    // always any path open, 50uA loss
    //muic_set_open();  
    if(muic_boot_keeping)
      muic_wake_unlock();
  }
  
}

#ifdef CONFIG_SYSFS
void muic_nv_write(int mode)
{
	int fd;
	char buf[20];
  printk("[MUIC] %s\n", __func__);
  
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
  printk("[MUIC] %s\n", __func__);
	return sprintf(buf, "%d\n", muic_path);
}

static ssize_t muic_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	muic_port_setting_type mode;

  printk("[MUIC] %s\n", __func__);

	if(sscanf(buf, "%d", &mode) != 1)
		return size;

  muic_set_open();

#ifdef CONFIG_HAS_WAKELOCK
  if(mode == PORT_SETTING_AP_USB)
    muic_wake_lock();
#endif

	muic_set_switch_mode(mode, FALSE);

//	muic_nv_write(mode);

	return size;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO, muic_mode_show, muic_mode_store);
// 20110903 hg.park@lge.com CTS File Permission
static ssize_t muic_mode_set_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("[MUIC] %s\n", __func__);

	muic_set_open();
#ifdef CONFIG_HAS_WAKELOCK
	muic_wake_lock();
#endif
	muic_set_switch_mode(PORT_SETTING_AP_USB, FALSE);


	return sprintf(buf, "%d\n", muic_path);
}

static DEVICE_ATTR(mode_set, 0644, muic_mode_set_show, NULL);

#endif

static void muic_wq_func(struct work_struct *muic_wq)
{
  printk("[MUIC] %s\n", __func__);
	muic_device_detection(muic_dev->client);
}

unsigned int muic_int_gpio = GPIO_MUIC_INT_N;

static irqreturn_t muic_interrupt_handler(void* arg)
{
//  printk("[MUIC] %s\n", __func__);
  if(gpio_get_value(muic_int_gpio) == GPIO_LOW_VALUE) //low value
    schedule_delayed_work(&muic_wq, msecs_to_jiffies(70));
  
	return IRQ_HANDLED;
}

static int __init muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;  

  printk("[MUIC] %s\n", __func__);
  
	muic_dev = kzalloc(sizeof(*muic_dev), GFP_KERNEL);
	if (!muic_dev)
	{
		return -ENOMEM;
	}
	memset(muic_dev, 0, sizeof(*muic_dev));
	muic_dev->client = client;

	i2c_set_clientdata(client, muic_dev);

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_int_gpio = GPIO_MUIC_INT_N_REV_E;
#endif

	gpio_request(muic_int_gpio, "muic_int_n");
	tegra_gpio_enable(muic_int_gpio);
	gpio_direction_input(muic_int_gpio);

	gpio_request(GPIO_CP_UART_SW, "uart_sw");
	tegra_gpio_enable(GPIO_CP_UART_SW);
	gpio_direction_output(GPIO_CP_UART_SW, 0);

	gpio_request(GPIO_CP_USB_VBUS_EN, "mdm_usb_vbus_en");
	tegra_gpio_enable(GPIO_CP_USB_VBUS_EN);
	gpio_direction_output(GPIO_CP_USB_VBUS_EN, 0);

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
  gpio_request(GPIO_CP_USB_DP_DM_SW, "cp_usb_dp_dm_sw");
	tegra_gpio_enable(GPIO_CP_USB_DP_DM_SW);
	gpio_direction_output(GPIO_CP_USB_DP_DM_SW, 0);  
#endif

	INIT_DELAYED_WORK(&muic_wq, muic_wq_func);
	error = request_irq(gpio_to_irq(muic_int_gpio), (irq_handler_t)muic_interrupt_handler,
						IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "muic_int_n", (void*)&muic_dev);
	if(error)
	{
		printk(KERN_ERR "[MUIC] interrupt registeration fail! GPIO : %d, (err:%d)\n", muic_int_gpio, error);
		goto err_request_irq_fail;
	}

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
#endif
  enable_irq_wake(gpio_to_irq(muic_int_gpio));

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&muic_dev->wlock, WAKE_LOCK_SUSPEND, "muic_wakelock");
  wake_lock_init(&muic_dev->wlock_timeout, WAKE_LOCK_SUSPEND, "muic_wakelock_timeout");
#endif

#ifdef CONFIG_SYSFS
	if(device_create_file(&client->dev, &dev_attr_mode))
	{
		printk("[MUIC] mode file create -error \n");
	}

	if(device_create_file(&client->dev, &dev_attr_mode_set))
	{
		printk("[MUIC] mode file create -error \n");
	}
#endif

	muic_device_init(muic_dev->client);

#if 0
	if(muic_path != PORT_SETTING_AUTO)
	{
		muic_set_switch_mode(muic_path, TRUE);
	}
#endif

  muic_device_detection(muic_dev->client);

	return 0;

err_request_irq_fail:
	free_irq(gpio_to_irq(muic_int_gpio), muic_dev);

	return -ENOSYS;
}

static int muic_remove(struct i2c_client *client)
{
  unsigned int muic_int_gpio = GPIO_MUIC_INT_N;
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&muic_dev->wlock);
  wake_lock_destroy(&muic_dev->wlock_timeout);
#endif

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_int_gpio = GPIO_MUIC_INT_N_REV_E;
#endif
	free_irq(gpio_to_irq(muic_int_gpio), muic_dev);

	if (muic_dev)
	{
		kfree(muic_dev);
		muic_dev = NULL;
	}
	device_remove_file(&client->dev, &dev_attr_mode);
	device_remove_file(&client->dev, &dev_attr_mode_set);
	return 0;
}

//LGE_CHANGE_S [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk
static int muic_suspend(struct i2c_client *client, pm_message_t state)
{
//	printk("[MUIC] %s\n", __func__);
	cancel_delayed_work_sync(&muic_wq); //20111031 seki.park@lge.com : cancel_delayed_work -> cancel_delayed_work_sync 
//	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);
//	printk("[MUIC] MUIC Control2 Register Interrupt Disable\n");
	return 0;
}
//LGE_CHANGE_E [peelsang.yoo@lge.com] 2011-01-24 [LGE_AP20] lprintk is changed to printk

static int muic_resume(struct i2c_client *client)
{
//  printk("[MUIC] %s\n", __func__);
	muic_write_register(muic_dev->client, CTRL2_REG, INT_EN_M);

  if(g_cp_sleep_mode)
    wake_lock_timeout(&muic_dev->wlock_timeout, 500);

	muic_device_detection(muic_dev->client);
	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{ "bssq_muic_ti", 0 },
	{ /* end of list */ },
};

static struct i2c_driver muic_driver = {
	.probe     = muic_probe,
	.remove    = muic_remove,
	.suspend   = muic_suspend,
	.resume    = muic_resume,
	.id_table  = muic_ids,
	.driver    = {
		.name = "bssq_muic_ti",
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
MODULE_DESCRIPTION("MUIC Driver for LGE STAR");
MODULE_LICENSE("GPL");
