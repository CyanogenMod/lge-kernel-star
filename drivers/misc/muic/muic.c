/*
 * X3 MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
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
#include <linux/interrupt.h>		/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <asm/system.h>
#include "../gpio-names.h"

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
#include <linux/mutex.h>

#include <linux/notifier.h>
#include <linux/reboot.h>

// LGE_UPDATE yoolje.cho@lge.com [[ 4 charger
#include <linux/power_supply.h>

#include <linux/muic.h>

#ifdef	CONFIG_BATTERY_CHARGER
#include <linux/max8922l.h>
#endif

#ifdef	CONFIG_BATTERY_CHARGER
#include <linux/su660_battery.h> 
#endif

#ifdef  CONFIG_BSSQ_BATTERY
#include "../../bssq/bssq_battery.h"
#endif

#define DEBUG_MUIC
#ifdef	DEBUG_MUIC
#define DBG(fmt, arg...) printk(KERN_WARNING "%s : " fmt "\n", __func__, ## arg)
#else
#define DBG(fmt, arg...) do {} while (0)
#endif

#if defined(CONFIG_MACH_STAR)
#include "../../../arch/arm/mach-tegra/lge/star/include/lge/board-star-nv.h"
#endif

#if defined(CONFIG_MACH_BSSQ)
#include <lge/board-bssq-nv.h>
#if defined(CONFIG_KS1103)
#include <lge/bssq_charger_max.h>
#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880)
#include <lge/bssq_charger_rt.h>
#endif
#include <linux/lge_hw_rev.h>
extern int muic_boot_keeping;
extern int muic_boot_path;

#define TRUE 1
//MDM_UART_SW
#define GPIO_CP_UART_SW         TEGRA_GPIO_PU2 // UART_SW

//MDM_USB_VBUS_EN
#define GPIO_CP_USB_VBUS_EN     TEGRA_GPIO_PF1  // MDM_USB_VBUS_EN
#if defined(CONFIG_LU6500)
unsigned int muic_intr_gpio = TEGRA_GPIO_PU0; 
#else
unsigned int muic_intr_gpio = TEGRA_GPIO_PU5; 
#endif//CONFIG_LU6500

#if defined(CONFIG_LU6500)
#define GPIO_MUIC_INT_N_REV_E		TEGRA_GPIO_PW2 // MUIC_INT_N
#endif

#endif

const char *retain_mode_str[] = {
	"RETAIN_NO",
	"RETAIN_AP_USB",
	"RETAIN_CP_USB",
	"RETAIN_AP_UART",
	"RETAIN_CP_UART",
};

static struct i2c_client *muic_client;
#if 0//def CONFIG_MACH_STAR_SU660
static DEFINE_SPINLOCK(muic_spin_lock);
bool muic_irq_already_run;
struct workqueue_struct *muic_wq;
static struct work_struct muic_work;
#else
static struct work_struct muic_wq;
#endif
static u8 muic_device;

static unsigned int muic_gpio_irq = NULL;
TYPE_USIF_MODE usif_mode = USIF_AP;
TYPE_DP3T_MODE dp3t_mode = DP3T_NC;
TYPE_MUIC_MODE muic_mode = MUIC_UNKNOWN;
TYPE_CHARGING_MODE charging_mode = CHARGING_NONE;
TYPE_UPON_IRQ  upon_irq  = NOT_UPON_IRQ;

static TYPE_RETAIN_MODE retain_mode = RETAIN_NO;
static TYPE_RETAIN_MODE boot_retain_mode = RETAIN_NO;


const char *muic_mode_str[] = {
	"MUIC_UNKNOWN",			// 0
	"MUIC_NONE",   			// 1
	"MUIC_NA_TA",   		// 2
	"MUIC_LG_TA",   		// 3
	"MUIC_TA_1A", 	  		// 4
	"MUIC_INVALID_CHG",		// 5
	"MUIC_AP_UART",   		// 6
	"MUIC_CP_UART",			// 7
	"MUIC_AP_USB", 			// 8
	"MUIC_CP_USB",			// 9
	"MUIC_TV_OUT_NO_LOAD",		// 10
	"MUIC_EARMIC",			// 11
	"MUIC_TV_OUT_LOAD",		// 12
	"MUIC_OTG",   			// 13
	"MUIC_MHL",			// 14
	"MUIC_RESERVE1",		// 15
	"MUIC_RESERVE2",		// 16
};

const char *charging_mode_str[] = {
	"CHARGING_UNKNOWN",
	"CHARGING_NONE",
	"CHARGING_NA_TA",
	"CHARGING_LG_TA",
	"CHARGING_TA_1A",
	"CHARGING_INVALID_CHG",
	"CHARGING_USB",
	"CHARGING_FACTORY",
};

typedef enum
{
    PORT_SETTING_AUTO = 0,
    PORT_SETTING_AP_USB,
    PORT_SETTING_AP_UART,
    PORT_SETTING_CP_UART,
    PORT_SETTING_CP_USB,
    PORT_SETTING_DLOAD_USB,
    PORT_SETTING_OPEN
} muic_port_setting_type;

#if defined (MUIC_SLEEP)
struct wake_lock muic_wake_lock;
static char wake_lock_enable = 1;
#endif

#if defined(CONFIG_MACH_STAR)
int g_half_charging_control=0;
#endif
void (*muic_init_device)(TYPE_RESET) = muic_init_unknown;
s32 (*muic_detect_accessory)(s32) = muic_unknown_detect_accessory;

extern void muic_init_ts5usba33402(TYPE_RESET reset);
extern void muic_init_max14526(TYPE_RESET reset);

extern s32 muic_ts5usba33402_detect_accessory(s32 upon_irq);
extern s32 muic_max14526_detect_accessory(s32 upon_irq);


static s32 muic_proc_set_ap_uart(void);
static s32 muic_proc_set_cp_uart(void);
static s32 muic_proc_set_ap_usb(void);
static s32 muic_proc_set_cp_usb(void);

void muic_send_charger_type(TYPE_CHARGING_MODE mode);


#if defined (CONFIG_MACH_STAR) || (CONFIG_MACH_BSSQ)
	//check muic condition for fuel gauge reset.
static DEFINE_MUTEX(muic_mutex);
static int muic_set_state(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	return ret;
}
static int muic_get_state(char *buffer, struct kernel_param *kp)
{
	int ret;
	mutex_lock(&muic_mutex);
	ret = sprintf(buffer, "%d", muic_mode);
	mutex_unlock(&muic_mutex);
	return ret;
}
module_param_call(muic_current_device, 
		muic_set_state, 
		muic_get_state, 
		&muic_mode, 0664);
MODULE_PARM_DESC(muic_mode, "MUIC current device");

//for half boot charging
int half_boot_enable=0;
#define ON 1
#define OFF 0
#endif //CONFIG_MACH_STAR

#ifdef CONFIG_PROC_FS
/*
 * --------------------------------------------------------------------
 *  BEGINS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#define	LG_MUIC_PROC_FILE "driver/cmuic"
static struct proc_dir_entry *lg_muic_proc_file;

#ifndef CP_RESET_TEST
static int muic_cp_request(void)
{
	s32 ret = 0;

	ret = gpio_request(TEGRA_GPIO_PV1, "TEGRA_GPIO_PR5 switch control 1 GPIO");
	if (ret < 0) {
		DBG( "[MUIC] GPIO 11 TEGRA_GPIO_PR5 is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PV1, 0);
	if (ret < 0) {
		DBG( "[MUIC] gpio_11 TEGRA_GPIO_PR5 direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(TEGRA_GPIO_PV1);


	ret = gpio_request(TEGRA_GPIO_PO0, "TEGRA_GPIO_PR5 switch control 1 GPIO");
	if (ret < 0) {
		DBG( "[MUIC] GPIO 11 TEGRA_GPIO_PR5 is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PO0, 0);
	if (ret < 0) {
		DBG( "[MUIC] gpio_11 TEGRA_GPIO_PR5 direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(TEGRA_GPIO_PO0);

	return 0;
}

static void muic_cp_reset(void)
{
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);

	gpio_set_value(TEGRA_GPIO_PO0, 0);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PV1, 0);
	mdelay(2000);

	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	//gpio_set_value(TEGRA_GPIO_PO0, 0);

	DBG("[MUIC] muic_cp_reset: TEGRA_GPIO_PO0/PV1 ");
	return;

}

#if 1 // jdpark : fota reset
#define GPIO_CP_RESET       TEGRA_GPIO_PV0  //CP_RESET
#endif

//MOBII_CHNANGE_S 20120416 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
//LGE_CHANGE_S	jinook.hong	2012-02-21	for FOTA
void fota_cp_reset(void)
{
#if 1 // jdpark : fota reset
    int ret;

    gpio_request(GPIO_CP_RESET, "ifx_reset_n");
    tegra_gpio_enable(GPIO_CP_RESET);
    gpio_direction_output(GPIO_CP_RESET, 1);

    dp3t_switch_ctrl(DP3T_NC);
    mdelay(5000);

	gpio_set_value(TEGRA_GPIO_PV0, 0);
	ret = gpio_get_value(TEGRA_GPIO_PV0);
	printk(KERN_INFO "[FOTA] fota_cp_reset: ret = %d ", ret);
	mdelay(10);
	gpio_set_value(TEGRA_GPIO_PV0, 1);
	ret = gpio_get_value(TEGRA_GPIO_PV0);
	printk(KERN_INFO "[FOTA] fota_cp_reset: ret = %d ", ret);
#else
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	
	gpio_set_value(TEGRA_GPIO_PO0, 0);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PV1, 0);
	mdelay(2000);

	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	//gpio_set_value(TEGRA_GPIO_PO0, 0);

	printk(KERN_INFO "[FOTA] fota_cp_reset: TEGRA_GPIO_PO0/PV1 ");
		return;
#endif
}
//LGE_CHANGE_E	jinook.hong	2012-02-21	for FOTA
//MOBII_CHNANGE_E 20120416 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

#endif

#if defined(CONFIG_MHL_TX_SII9244)	
//[gieseo.park@lge.com] - moved muic_ti.c to here
void muic_set_mhl_mode_detect(void) 
{ 

	DBG("[MUIC] muic_set_mhl_mode_detect entry. \n"); 

	/* Connect CP UART signals to AP */ 
	usif_switch_ctrl(USIF_AP); 
	/* 109 		 * AP USB does not pass through DP3T. 110		 
	 * Just connect AP UART to MUIC UART.  111		  */

	dp3t_switch_ctrl(DP3T_NC);

	//muic_i2c_write_byte(SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);  //Path USB
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); //Path OPEN


	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */ 
	//muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);   

	MHL_On(1); 

	//muic_mode = MUIC_MHL;  //[gieseo.park@lge.com] - Set by caller. NOT here.
}
#endif

static ssize_t muic_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
	s32 i;
	u32 val;

	for (i = 0; i <= 5; i++) {
		val = i2c_smbus_read_byte_data(muic_client, (u8)i);
		DBG("[MUIC] reg 0x%02x, val = 0x%02x\n", i, val);
	}

	len = sprintf(buf, "%d\n", muic_mode);
	DBG("[MUIC] mode = %s\n", muic_mode_str[muic_mode]);

	return 0;
}

#if defined (MUIC_SLEEP)
static void muic_wakeup_lock(void)
{
#if 1
	if(wake_lock_enable)
	{
		switch(charging_mode)
		{
			default:
			case CHARGING_UNKNOWN:
			case CHARGING_NONE:
			case CHARGING_INVALID_CHG:
				if(!half_boot_enable) //hijeong@mobii.co.kr : Fixed power off charging suspend problem
				    wake_unlock(&muic_wake_lock);
				printk(KERN_INFO "[MUIC] wake_unlock(): charging_mode = %s (%d)\n" , charging_mode_str[charging_mode], charging_mode);
				break;
			case CHARGING_NA_TA:
			case CHARGING_LG_TA:
			case CHARGING_TA_1A:
			case CHARGING_USB:
			case CHARGING_FACTORY:
				wake_lock(&muic_wake_lock);
				printk(KERN_INFO "[MUIC] wake_lock(): charging_mode = %s (%d)\n" , charging_mode_str[charging_mode], charging_mode);
				break;
		}
	}
	else
	{
		wake_unlock(&muic_wake_lock);
		printk(KERN_INFO "[MUIC] wake_unlock(): always unlock mode\n");
	}
#else
	if( wake_lock_enable && (muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE) )
	{
		wake_unlock(&muic_wake_lock);
		DBG("[MUIC] wake_unlock(): muic_mode = %s (%d)" , muic_mode_str[muic_mode], muic_mode);
	} else {
		wake_lock(&muic_wake_lock);
		DBG("[MUIC] wake_lock(): muic_mode = %s (%d)" , muic_mode_str[muic_mode], muic_mode);
	}
#endif
}
#endif

static ssize_t muic_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
#if defined (CONFIG_MACH_BSSQ)|| defined (CONFIG_MACH_STAR)
	char nv_msg[1];
#endif 
#if 0 //dongho70.kim@lge.com for Fixed
	char cmd;
	char nv_msg[1];

	len = sscanf(buf, "%c", &cmd/*, &dummy*/);	//[gieseo.park@lge.com] - what is dummy ?
#else
    char messages[10];
    s8 reg, val;
	char cmd;

    if (len > 12)
        len = 12;

    if (copy_from_user(messages, buf, len))
        return -EFAULT;

    sscanf(buf, "%c %x %x", &cmd, &reg, &val);
#endif
#if defined(CONFIG_LU6500)
    //port open for TI
	gpio_set_value(GPIO_CP_UART_SW, 0);
	gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);

	muic_i2c_write_byte(SW_CONTROL, DP_OPEN | DM_OPEN);
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);

	mdelay(1);
#endif
	switch (cmd) {
		/* AP_UART mode*/
		case '6' :
		retain_mode = RETAIN_AP_UART;
			muic_proc_set_ap_uart();
			break;

			/* CP_UART mode*/
		case '7' :
		retain_mode = RETAIN_CP_UART;
			muic_proc_set_cp_uart();
			break;

			/* AP_USB mode*/
		case '8' :
		retain_mode = RETAIN_AP_USB;
                boot_retain_mode = RETAIN_NO;
			muic_proc_set_ap_usb();
			break;

			/* CP_USB mode*/
		case '9' :
		retain_mode = RETAIN_CP_USB;
			muic_proc_set_cp_usb();
			mdelay(20);
			break;

#ifndef CP_RESET_TEST
			/* CP_DOWNLOAD mode*/
	case '0' :
		retain_mode = RETAIN_CP_USB;
		case 'c' :
			muic_proc_set_cp_usb();
			mdelay(1000);
			muic_cp_reset();
			//mdelay(400);
			break;
		case 'q' :
			muic_cp_request();
			break;
		case 'r' :
			muic_cp_reset();
			break;
#endif//		

#if defined(CONFIG_MUIC_RETAIN)
			/* Rebooting should be performed by HiddenMenu Application. */
		case '1':
		retain_mode = RETAIN_NO;
			/* NO retain mode after reboot */
			//set_misc_msg(msg_type_muic_path,retain_mode_str[NO_RETAIN], RETAIN_MODE_STR_LENGTH+1);
			nv_msg[0] = RETAIN_NO;
#ifdef CONFIG_MACH_STAR
                        lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
#else
			lge_nvdata_write(LGE_NVDATA_MUIC_PATH_STR_OFFSET, "muic-path-none", 32);
#endif
			break;

		case 'a' :
			/* AP USB retain mode after reboot */
			//set_misc_msg(msg_type_muic_path,retain_mode_str[BOOT_AP_USB], RETAIN_MODE_STR_LENGTH+1);
			nv_msg[0] = RETAIN_AP_USB;
#ifdef CONFIG_MACH_STAR
                        lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
#else
			lge_nvdata_write(LGE_NVDATA_MUIC_PATH_STR_OFFSET, "muic-path-ap", 32);
#endif
			break;

		case 'b' :
			/* CP USB retain mode after reboot */
			//set_misc_msg(msg_type_muic_path,retain_mode_str[BOOT_CP_USB], RETAIN_MODE_STR_LENGTH+1);
			nv_msg[0] = RETAIN_CP_USB;
#ifdef CONFIG_MACH_STAR
                        lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
#else
			lge_nvdata_write(LGE_NVDATA_MUIC_PATH_STR_OFFSET,"muic-path-cp", 32);
#endif
			break;
#endif//

#if defined (MUIC_SLEEP)
	        case 'u':
		        /* Just unlock the wakelock ! */
		        wake_lock_enable = 0;
	         	muic_wakeup_lock();
         		break;
	        case 'l':
         		/* Just lock the wakelock ! */
        		wake_lock_enable = 1;
        		muic_wakeup_lock();
        		break;
#endif		
	
#ifdef CONFIG_MACH_STAR
			case 'y':
					//NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, DISABLE);
				printk("AP <==> CP uart connection\n");
				gpio_set_value(USIF_IN_1_GPIO, 0);

				break;
			case 'x':
				//NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, ENABLE);
				printk("AP =\\= CP uart disconnection\n");
				gpio_set_value(USIF_IN_1_GPIO, 1);
				break;
			case 'n':
				printk("TA <==> DEVICE charger connection [%d] [%d]\n",g_half_charging_control,charging_mode);
					switch (g_half_charging_control) {
							case CHARGING_USB:
								charger_ic_set_mode_for_muic(CHARGER_USB500); 
								break;

							case CHARGING_NA_TA:
							case CHARGING_LG_TA:
							case CHARGING_TA_1A:
								charger_ic_set_mode_for_muic(CHARGER_ISET);
								break;

							case CHARGING_FACTORY:
								//	charger_ic_set_mode(CHARGER_FACTORY);
								break;

							case CHARGING_NONE:
								charger_ic_disable_for_muic();
								break;

							default:
								charger_ic_disable_for_muic();
								break;
						}				
				break;		
			case 'd':
				printk("TA =\\=  DEVICE charger disconnection %d\n",charging_mode);
				g_half_charging_control=charging_mode;
				charger_ic_disable_for_muic();
				break;	

			case 'k':
				printk("######[MUIC] Boot complete check charger cable and update charging_mode %d\n",charging_mode);
				muic_send_charger_type(charging_mode);
				break;
#endif
		default : 
			DBG("[MUIC] LGE: ap20 MUIC invalid command: [cmd=%c] 6=AP_UART, 7=CP_UART, 8=AP_USB, 9=CP_USB\n", cmd);
			break;
	}
	check_charging_mode();	//check the charging mode, but NOT send to charger !!!

	return len;
}

static struct file_operations lg_muic_proc_ops = {
	.read	= muic_proc_read,
	.write	= muic_proc_write,
};

static void create_lg_muic_proc_file(void)
{
	lg_muic_proc_file = create_proc_entry(LG_MUIC_PROC_FILE, 0777, NULL);
	if (lg_muic_proc_file) {
		lg_muic_proc_file->proc_fops = &lg_muic_proc_ops;
	} else
		DBG("[MUIC] LGE: X3-AP30 MUIC proc file create failed!\n");
}

static void remove_lg_muic_proc_file(void)
{
	remove_proc_entry(LG_MUIC_PROC_FILE, NULL);
}
/*
 * --------------------------------------------------------------------
 *  ENDS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#endif	// CONFIG_PROC_FS

void check_charging_mode(void)
{
	s32 value;

	value = i2c_smbus_read_byte_data(muic_client, INT_STAT);
	if (value & V_VBUS) {
		if ((value & IDNO) == IDNO_0010 || 
				(value & IDNO) == IDNO_0100 ||
				(value & IDNO) == IDNO_1001 ||
			(value & IDNO) == IDNO_1010 /*||
			(boot_retain_mode == RETAIN_AP_USB && retain_mode == RETAIN_AP_USB)*/)	//[gieseo.park@lge.com] force set CHARGING_FACTORY on Boot RETAIN_AP_USB mode. (for MilkyU device)
			charging_mode = CHARGING_FACTORY;
		else if (value & CHGDET) 
			charging_mode = CHARGING_LG_TA;
		else
			charging_mode = CHARGING_USB;
	} else
		charging_mode = CHARGING_NONE;

	//set_muic_charger_detected();	//not done here. Moved to other functions
}
EXPORT_SYMBOL(check_charging_mode);

/* 
 * Linux bug: If a constant value larger than 20000,
 * compiler issues a __bad_udelay error.
 */
void muic_mdelay(u32 microsec)
{
	do {
		udelay(1000);
	} while (microsec--);
}
EXPORT_SYMBOL(muic_mdelay);

#if 0 // [gieseo.park@lge.com] - Cosmo only
// [jongho3.lee@lge.com] get muic mode for charger
TYPE_MUIC_MODE get_muic_mode(void)
{
	return charging_mode;
}
EXPORT_SYMBOL(get_muic_mode);
#endif

#if 0	//[gieseo.park@lge.com] - changed to check charger type, NOT muic-path
// LGE_UPDATE_S yoolje.cho@lge.com [[ 4 charger
int muic_send_cable_type(TYPE_MUIC_MODE mode)
{
	// should use platform data..
	struct power_supply *psy=NULL;
	union power_supply_propval value;
	int ret;

	DBG("%s : mode = (%d)\n",  __func__, mode);

	switch (mode) {
		case MUIC_AP_USB:
		case MUIC_MHL:
			psy = power_supply_get_by_name("usb");
			value.intval = POWER_SUPPLY_TYPE_USB;
			break;
		case MUIC_NA_TA:
		case MUIC_LG_TA:
		case MUIC_TA_1A:
			psy = power_supply_get_by_name("ac");
			value.intval = POWER_SUPPLY_TYPE_MAINS;
			break;
		case MUIC_CP_UART:
		case MUIC_CP_USB:
		case MUIC_AP_UART:
			psy = power_supply_get_by_name("factory");
			value.intval = POWER_SUPPLY_TYPE_FACTORY;
			break;
		case MUIC_NONE:			
			psy = power_supply_get_by_name("battery");
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;
		default:
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;
	}

	if (!psy) {
		DBG("%s: fail to get battery/charger psy\n", __func__);
		return -ENODEV;
	}

	switch(value.intval){
		case POWER_SUPPLY_TYPE_USB:
		case POWER_SUPPLY_TYPE_MAINS:
		case POWER_SUPPLY_TYPE_FACTORY:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
			break;
		case POWER_SUPPLY_TYPE_BATTERY:
		default:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
			break;
	}
	return ret;
}
EXPORT_SYMBOL(muic_send_cable_type);
// LGE_UPDATE_E yoolje.cho@lge.com [[ 4 charger
#endif

/* Get MUIC Charger TYPE */
TYPE_CHARGING_MODE get_muic_charger_type(void)
{
	return charging_mode;  
}
EXPORT_SYMBOL(get_muic_charger_type);

void muic_send_charger_type(TYPE_CHARGING_MODE mode)
{
#ifdef CONFIG_BATTERY_CHARGER
	switch (mode) {
		case CHARGING_USB:
			charger_ic_set_mode_for_muic(CHARGER_USB500); 
			break;

		case CHARGING_NA_TA:
		case CHARGING_LG_TA:
		case CHARGING_TA_1A:
			charger_ic_set_mode_for_muic(CHARGER_ISET);
			break;

		case CHARGING_FACTORY:
			//	charger_ic_set_mode(CHARGER_FACTORY);
			break;

		case CHARGING_NONE:
			charger_ic_disable_for_muic();
			break;

		default:
			charger_ic_disable_for_muic();
			break;
	}
#elif CONFIG_MACH_BSSQ
	switch (mode) {
		case CHARGING_USB:	
			charger_ic_set_mode(CHARGER_USB500); 
			break;

		case CHARGING_NA_TA:
		case CHARGING_LG_TA:
		case CHARGING_TA_1A:
			charger_ic_set_mode(CHARGER_ISET);
			break;

		case CHARGING_FACTORY:
			//charger_ic_set_mode(CHARGER_IC_GSM_TEST);
			break;

		case CHARGING_NONE:
			charger_ic_disable();
			break;

		default:
			charger_ic_disable();
			break;
	}

#endif  
	// Notify To Battery 
	notification_of_changes_to_battery();
}



// LGE_CHANGE_E[jongho3.lee] get muic detecting for charger.
void set_muic_charger_detected(void)
{
#if 0 // this is for wait event which is not unsing just now. block it temporary..
	//extern wait_queue_head_t muic_event;

	DBG("[MUIC] LGE: set_muic_charger_detected\n");

	//atomic_set(&muic_charger_detected,1);
	//wake_up_interruptible(&muic_event);
	muic_chager_event = 1;
	//wake_up(&muic_event);
#endif
	//charger_fsm(CHARG_FSM_CAUSE_ANY);	//[gieseo.park@lge.com] - Cosmo code

	muic_send_charger_type(charging_mode);	//[gieseo.park@lge.com] - X3 code
#if defined (MUIC_SLEEP)
	muic_wakeup_lock();
#endif//
}
EXPORT_SYMBOL(set_muic_charger_detected);




/*
 * Function: Read the MUIC register whose internal address is addr
 * 			and save the u8 content into value.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_read_byte(u8 addr, u8 *value)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(muic_client, (u8)addr);
	if (ret < 0) {
		DBG("[MUIC] muic_i2c_read_byte failed.\n");
		return ret;
	} else {
		*value = (u8)ret;
		return 0;
	}
}
EXPORT_SYMBOL(muic_i2c_read_byte);


/*
 * Function: Write u8 value to the MUIC register whose internal address is addr.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_write_byte(u8 addr, u8 value)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(muic_client, (u8)addr, (u8)value);
	if (ret < 0)
		DBG("[MUIC] muic_i2c_write_byte failed.\n");
	return ret;
}
EXPORT_SYMBOL(muic_i2c_write_byte);


void usif_switch_ctrl(TYPE_USIF_MODE mode)
{
#if defined (CONFIG_USIF)
	if (mode == USIF_AP) {
		gpio_set_value(USIF_IN_1_GPIO, 0);
		DBG("[MUIC] usif_switch_ctrl, CP UART is connected to AP\n");
	} else if (mode == USIF_DP3T) {
		gpio_set_value(USIF_IN_1_GPIO, 1);
		DBG("[MUIC] usif_switch_ctrl, CP UART is connected to DP3T (then, MUIC)\n");
	} else {
		/* Just keep the current path */
	}

	usif_mode = mode;
	DBG("[MUIC] usif_switch_ctrl(): usif_mode = %d\n", usif_mode);
#else
	DBG("[MUIC] usif_switch_ctrl(): usif_mode   Not suport !!!  \n" );
#endif//	 
}
EXPORT_SYMBOL(usif_switch_ctrl);


void dp3t_switch_ctrl(TYPE_DP3T_MODE mode)
{
	//DP3T_IN_2_GPIO is no more used in X3 because there is NO AP_UART path and using DP2T.
#if defined (CONFIG_LGE_MUIC_DP3T)

	if (mode == DP3T_AP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		DBG("[MUIC] dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
		DBG("[MUIC] dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_USB) {
		int gpio_val=0, old_val = 0;
		old_val= gpio_get_value(IFX_USB_VBUS_EN_GPIO);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 1);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 1);

		gpio_val = gpio_get_value(IFX_USB_VBUS_EN_GPIO);
		DBG("[MUIC] dp3t_switch_ctrl, CP USB is connected to MUIC UART: ifx_usb_vbus = %d->%d\n",old_val ,gpio_val);
	} else if (mode == DP3T_NC) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		DBG("[MUIC] dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}

	dp3t_mode = mode;
	DBG("[MUIC] dp3t_switch_ctrl(): dp3t_mode = %d\n", dp3t_mode);
#else
	DBG("[MUIC] dp3t_switch_ctrl(): dp3t Not suport !!! \n" );
#endif//	 

}
EXPORT_SYMBOL(dp3t_switch_ctrl);


static s32 muic_proc_set_ap_uart(void)
{
	s32 ret;

#if defined (CONFIG_MACH_BSSQ)
#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_i2c_write_byte(SW_CONTROL, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#endif
#else
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);
#endif
	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);

	muic_mode = MUIC_AP_UART;	
	charging_mode = CHARGING_UNKNOWN;

	DBG("[MUIC] muic_proc_set_ap_uart(): AP_UART\n");

	return ret;
}


static s32 muic_proc_set_ap_usb(void)
{
	s32 ret;

#if defined (CONFIG_MACH_BSSQ)
#else
	ret = muic_i2c_write_byte(SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART.
	 */
	dp3t_switch_ctrl(DP3T_AP_UART); //dp3t_switch_ctrl(DP3T_NC);  //[gieseo.park@lge.com] - Changed to X3 Code. CP USB OFF is only thing.
#endif
	/* Connect DP, DM to USB_DP, USB_DM */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);

	muic_mode = MUIC_AP_USB;	
	charging_mode = CHARGING_USB;
	DBG("[MUIC] muic_proc_set_ap_usb(): AP_USB\n");

	return ret;
}


static s32 muic_proc_set_cp_uart(void)
{
	s32 ret;
#if defined (CONFIG_MACH_BSSQ)

#if defined(CONFIG_LU6500)
	gpio_set_value(GPIO_CP_UART_SW, 1);

  if(get_lge_pcb_revision() > REV_D)
    muic_i2c_write_byte(SW_CONTROL, COMP2_TO_AUD2|COMN1_TO_AUD1);
  else
#else
	gpio_set_value(GPIO_CP_UART_SW, 1);
#endif
 

#else
	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);

	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);
#endif
	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);

	muic_mode = MUIC_CP_UART;	
	charging_mode = CHARGING_UNKNOWN;

	DBG("[MUIC] muic_proc_set_cp_uart(): CP_UART\n");

	return ret;
}

static s32 muic_proc_set_cp_usb(void)
{
	s32 ret;

#if defined (CONFIG_MACH_BSSQ)
	gpio_set_value(GPIO_CP_USB_VBUS_EN, 1);
#else	
	ret = muic_i2c_write_byte(SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);//usif_switch_ctrl(USIF_DP3T); //[gieseo.park@lge.com] - Changed to X3 code.

	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */		

	// muic_i2c_write_byte(CONTROL_1,  ID_200 |ADC_EN  | CP_EN ); //13 //[gieseo.park@lge.com] - disabled because NOT in Cosmo Code
	//	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN); //15
	//	muic_i2c_write_byte(CONTROL_1,  ID_2P2 | SEMREN );//44
#endif 	
	/* Enable USB Path (0x03=0x00) --- Connect DP, DM to UART_TX, UART_RX */	
	ret = muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);


	muic_mode = MUIC_CP_USB;		
	charging_mode = CHARGING_USB;
	DBG("[MUIC] muic_proc_set_cp_usb(): CP_USB\n");

	return ret;
}

void muic_proc_set_cp_usb_force(void)
{
	//regardless of wakelock, ONLY muic path switch !
	retain_mode = RETAIN_CP_USB;
	muic_proc_set_cp_usb();
}

void muic_init_unknown(TYPE_RESET x)
{
	printk("[MUIC] muic_init_unknown: You should add codes for new MUIC chip");
	return;
}
s32 muic_unknown_detect_accessory(s32 x)
{
	printk("[MUIC] muic_unknown_detect_accessory: You should add codes for new MUIC chip");
	return -1;
}

static void muic_detect_device(void)
{
	s32 ret;

	DBG("[MUIC] muic_detect_device()\n");

	ret = muic_i2c_read_byte(DEVICE_ID, &muic_device);
	if ((muic_device & 0xf0) == TS5USBA33402)
		muic_device = TS5USBA33402;
	else if ((muic_device & 0xf0) == MAX14526)
		muic_device = MAX14526;
	else if ((muic_device & 0xf0) == ANY_VENDOR)
		muic_device = ANY_VENDOR;

	if (muic_device == TS5USBA33402) {
		muic_init_device = muic_init_ts5usba33402;
		muic_detect_accessory = muic_ts5usba33402_detect_accessory;
		DBG("[MUIC] muic chip: TS5USBA33402\n");
	} else if (muic_device == MAX14526) {
		//	muic_init_device = muic_init_max14526(RESET);
		muic_init_device = muic_init_max14526;
		muic_detect_accessory = muic_max14526_detect_accessory;
		DBG("[MUIC] muic chip: MAX14526\n");
	} else {
		muic_init_device = muic_init_unknown;
		muic_detect_accessory = muic_unknown_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip: ANY VENDOR\n");
	}
}

#if 0//def CONFIG_MACH_STAR_SU660
static void muic_work_inside_operation(void)//from X3
#else
static void muic_wq_func(struct work_struct *muic_wq)
#endif
{
	s32 ret = 0;

	DBG("[MUIC] muic_wq_func(): retain_mode = %s (%d)", retain_mode_str[retain_mode], retain_mode);

	if (retain_mode == RETAIN_NO) {
		ret = muic_detect_accessory(UPON_IRQ);
		set_muic_charger_detected();	// [jongho3.lee@lge.com]

		DBG("[MUIC] muic_detect_accessory(UPON_IRQ) result:  muic_mode = %s (%d), charing = %s (%d)", 
				muic_mode_str[muic_mode], muic_mode, charging_mode_str[charging_mode], charging_mode);
	}
	else 
	{
		muic_mdelay(250);

		ret = muic_detect_accessory(UPON_IRQ);
	//	ret = i2c_smbus_read_byte_data(muic_client, INT_STAT);
		if (muic_mode == MUIC_CP_USB)
			muic_proc_set_cp_usb();
		check_charging_mode();

		set_muic_charger_detected();	
		DBG("[MUIC] Now...path retain mode,  muic_mode = %s (%d), charing = %s (%d)\n", 
				muic_mode_str[muic_mode], muic_mode, charging_mode_str[charging_mode], charging_mode);
	}
}
#if 0//def CONFIG_MACH_STAR_SU660
static void muic_work_func(struct work_struct *muic_work)
{
    muic_work_inside_operation();
}
#endif
static irqreturn_t muic_interrupt_handler(s32 irq, void *data)
{
	/* Make the interrupt on MUIC INT wake up OMAP which is in suspend mode */
#if defined (CONFIG_MACH_BSSQ)
	if(gpio_get_value(muic_intr_gpio) == 0) //low value
		schedule_work(&muic_wq);
#else	
#if 0//def CONFIG_MACH_STAR_SU660
	queue_work(muic_wq, &muic_work);
        muic_irq_already_run = true;
#else
	schedule_work(&muic_wq);
#endif
#endif
	return IRQ_HANDLED;
}


static int muic_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
    switch (event) {
    case SYS_RESTART:
    case SYS_HALT:
    case SYS_POWER_OFF:
		{	
			printk("%s : %d\n",__func__, event);
			
			disable_irq(muic_gpio_irq);
			free_irq(muic_gpio_irq, &muic_client->dev);
			
			printk("%s : disable_irq\n",__func__);

			//flush_workqueue(muic_wq);
			//destroy_workqueue(muic_wq);
			
			//printk("%s : flush_workqueue\n",__func__);
						
			muic_i2c_write_byte(CONTROL_1, 0x00);
		}
	    return NOTIFY_OK;
    }
    return NOTIFY_DONE;
}


static struct notifier_block muic_reboot_nb = {
	.notifier_call = muic_reboot_notify,
};



/*
 * muic_probe() is called in the middle of booting sequence due to '__init'.
 * '__init' causes muic_probe() to be released after the booting.
 */
static s32 __devinit muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = 0;
	muic_client = client;

	DBG("[MUIC] LG muic_probe() Begin\n");
#if defined (CONFIG_USIF)
	/* 
	 * Initializes gpio_165 (USIF1_SW).
	 * Checks if other driver already occupied it.
	 */
	ret = gpio_request(USIF_IN_1_GPIO, "USIF switch control GPIO");
	if (ret < 0) {
		DBG("[MUIC] GPIO 165 USIF1_SW is already occupied by other driver!\n");
		/*
		 * We know board_cosmo.c:ifx_n721_configure_gpio() performs a gpio_request on this pin first.
		 * Because the prior gpio_request is also for the analog switch control, this is not a confliction.
		 */
		return -ENOSYS;
	}
#if 0 // dongho70.kim@lge.com
	ret = gpio_direction_output(USIF_IN_1_GPIO, 0);
#else
     ret = gpio_direction_output(USIF_IN_1_GPIO, 1);
#endif
	if (ret < 0) {
		DBG("[MUIC] gpio_16 USIF_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(USIF_IN_1_GPIO);
#endif	//

#if defined (CONFIG_LGE_MUIC_DP3T)
	/*
	 * Initializes gpio_11 (OMAP_UART_SW) and gpio_12 (IFX_UART_SW).
	 * Checks if other driver already occupied them.
	 */
	ret = gpio_request(DP3T_IN_1_GPIO, "DP3T switch control 1 GPIO");
	if (ret < 0) {
		DBG("[MUIC] GPIO 11 DP3T_IN_1_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_1_GPIO, 0);
	if (ret < 0) {
		DBG("[MUIC] gpio_11 DP3T_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(DP3T_IN_1_GPIO);


	DBG("[MUIC] Old REV_D has AP_UART --- initalizing DP3T_IN_2_GPIO\n");
	ret = gpio_request(DP3T_IN_2_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		DBG("[MUIC] gpio_12 DP3T_IN_2_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_2_GPIO, 0);
	if (ret < 0) {
		DBG("[MUIC] gpio_12 DP3T_IN_2_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(DP3T_IN_2_GPIO);


	ret = gpio_request(IFX_USB_VBUS_EN_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		DBG("[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(IFX_USB_VBUS_EN_GPIO, 0);
	if (ret < 0) {
		DBG("[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(IFX_USB_VBUS_EN_GPIO);
#endif//

#if defined (CONFIG_MACH_BSSQ)

#if defined(CONFIG_LU6500)
  if(get_lge_pcb_revision() > REV_D)
    muic_intr_gpio = GPIO_MUIC_INT_N_REV_E;
#endif
	printk("[kdh] config_mach_bssq\n");
	gpio_request(muic_intr_gpio, "muic_int_n");
	tegra_gpio_enable(muic_intr_gpio);
	gpio_direction_input(muic_intr_gpio);

	gpio_request(GPIO_CP_UART_SW, "uart_sw");
	tegra_gpio_enable(GPIO_CP_UART_SW);
	gpio_direction_output(GPIO_CP_UART_SW, 0);

	gpio_request(GPIO_CP_USB_VBUS_EN, "mdm_usb_vbus_en");
	tegra_gpio_enable(GPIO_CP_USB_VBUS_EN);
	gpio_direction_output(GPIO_CP_USB_VBUS_EN, 0);
#else
	ret = gpio_request(MUIC_GPIO, "pu0");

	if (ret < 0)
	{
		DBG("[MUIC] MUIC_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret=gpio_direction_input(MUIC_GPIO);
	if (ret < 0)
	{
		DBG("[MUIC] MUIC_GPIO direction initialization failed!\n");
		gpio_free(MUIC_GPIO);
		return -ENOSYS;
	}
	else
		tegra_gpio_enable(MUIC_GPIO);
#endif

#if defined (MUIC_SLEEP)
	wake_lock_init(&muic_wake_lock, WAKE_LOCK_SUSPEND, "muic_lock");
#endif

	/* Registers MUIC work queue function */
#if 0//def CONFIG_MACH_STAR_SU660
	INIT_WORK(&muic_work, muic_work_func);
#else
	INIT_WORK(&muic_wq, muic_wq_func);
#endif
	/* 
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
#if defined (CONFIG_MACH_BSSQ)
	muic_gpio_irq = gpio_to_irq(muic_intr_gpio);
	ret = request_irq(muic_gpio_irq, (irq_handler_t)muic_interrupt_handler,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, client->name, &client->dev);
#else
	muic_gpio_irq = gpio_to_irq(MUIC_GPIO);
	ret = request_irq(muic_gpio_irq, (irq_handler_t)muic_interrupt_handler, IRQF_TRIGGER_FALLING  , client->name, &client->dev);
#endif
	if (ret < 0) {
		DBG("[MUIC] MUIC_GPIO IRQ line set up failed!\n");
		free_irq(muic_gpio_irq, &client->dev);
		return -ENOSYS;
	}
	//disable_irq_wake(MUIC_GPIO);
//#if defined(CONFIG_LU6500)
//  if(get_lge_pcb_revision() > REV_D)  
//  enable_irq_wake(gpio_to_irq(muic_intr_gpio)); 
//#endif

	/* Prepares a human accessible method to control MUIC */
	create_lg_muic_proc_file();

	/* Selects one of the possible muic chips */
	muic_detect_device();

#if defined (CONFIG_MACH_BSSQ)
	if(muic_boot_keeping == TRUE)
	{
		if(muic_boot_path == PORT_SETTING_CP_USB)
			retain_mode = RETAIN_CP_USB;
		else if(muic_boot_path == PORT_SETTING_AP_USB)
			retain_mode = RETAIN_AP_USB;
	}
	printk("[KDH] muic keeping %d boot_path %d\n", muic_boot_keeping,muic_boot_path);
#endif

	/* Initializes MUIC - Finally MUIC INT becomes enabled */
	if (retain_mode == RETAIN_AP_USB) {
		muic_proc_set_ap_usb();	//[gieseo.park@lge.com] was not in cosmo code.
		muic_mode = MUIC_AP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		DBG("[MUIC] muic_init_device... retain mode = AP_USB\n");
	} else if (retain_mode == RETAIN_CP_USB) {
		muic_proc_set_cp_usb();
		muic_mode = MUIC_CP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		DBG("[MUIC] muic_detect_accessory... retain mode = CP_USB\n");
	} else {

		if (muic_init_device)
			muic_init_device(DEFAULT);	//[gieseo.park@lge.com] default is OK(already initialized at bootloader)
		else
			DBG("[MUIC] You should add codes for new MUIC chip");
		if (muic_detect_accessory)
			muic_detect_accessory(NOT_UPON_IRQ);
		else
			DBG("[MUIC] You should add codes for new MUIC chip");
	}

	//[jongho3.lee@lge.com]
	set_muic_charger_detected();

	/* Makes the interrupt on MUIC_GPIO INT wake up AP which is in suspend mode */
#if 0
	ret = enable_irq_wake(gpio_to_irq(MUIC_GPIO));
	if (ret < 0) {
		DBG("[MUIC] MUIC_GPIO wake up source setting failed!\n");
		disable_irq_wake(gpio_to_irq(MUIC_GPIO));
		return -ENOSYS;
	}
#endif//
        register_reboot_notifier(&muic_reboot_nb);
	DBG();

	return ret;
}

static s32 __devexit muic_remove(struct i2c_client *client)
{
	free_irq(muic_gpio_irq, &client->dev);
	gpio_free(MUIC_GPIO);
	//[gieseo.park@lge.com] - Should I free other GPIOs? : USIF_IN_1_GPIO, DP3T_IN_1_GPIO, DP3T_IN_2_GPIO, IFX_USB_VBUS_EN_GPIO
	i2c_set_clientdata(client, NULL);	// For what?
	remove_lg_muic_proc_file();
	return 0;
}

static s32 muic_suspend(struct i2c_client *client, pm_message_t state)
{
	unsigned long flags;



	printk("[MUIC] MUIC : suspend is called \n");

#if 0
	spin_lock_irqsave(&muic_spin_lock, flags);
	muic_irq_already_run = false;
	spin_unlock_irqrestore(&muic_spin_lock, flags);
#endif

	client->dev.power.power_state = state;
if(muic_device == MAX14526)
	muic_i2c_write_byte(CONTROL_2, 0);

#if 0	//[gieseo.park@lge.com] - why syspend?
	muic_i2c_write_byte(CONTROL_2, USB_DET_DIS);
	muic_i2c_write_byte(CONTROL_1, 0x00);
	muic_i2c_write_byte(SW_CONTROL, OPEN);
	//	DBG("[MUIC] COSMO MUIC : Suspend \n");
#endif
	return 0;
}

static s32 muic_resume(struct i2c_client *client)
{
#if 0//def CONFIG_MACH_STAR_SU660
	unsigned long flags;
	printk("[MUIC] UIC : Resume , MUIC_GPIO level %d, muic_irq_already_run %d\n", gpio_get_value(MUIC_GPIO), muic_irq_already_run);
#endif
	client->dev.power.power_state = PMSG_ON;
	if(muic_device == MAX14526)
		muic_i2c_write_byte(CONTROL_2, INT_EN);

#if 0
	spin_lock_irqsave(&muic_spin_lock, flags);
	if (gpio_get_value(MUIC_GPIO) == 0 && muic_irq_already_run == false) {
	queue_work(muic_wq, &muic_work);
	}
	spin_unlock_irqrestore(&muic_spin_lock, flags);
#endif


#if 0	//[gieseo.park@lge.com] - why suspend?
	//	DBG("[MUIC] COSMO MUIC : Resume \n");
	muic_init_device(RESET);
#endif

	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{"max14526" /* "lg_i2c_muic" */, 0},
	{/* end of list */},
};

//LGE_Changes_S chulhwhee.shim@lge.com, 2010.12.7  FOTA update
int fota_ebl_download(void)
{
	return 0;
}
//LGE_Changes_E chulhwhee.shim@lge.com, 2010.12.7  FOTA update

/* LGE_CHANGE_S [kenneth.kang@lge.com] 2011-07-26, CP retain mode */
static s32 __init muic_state(char *str)
{
	s32 muic_value = simple_strtol(str, NULL, 0);
	retain_mode = muic_value;
        boot_retain_mode = muic_value;	//[gieseo.park@lge.com] for keeping retain_mode at boot_time
	DBG("[MUIC] muic_state(): retain_mode = %s (%d)\n", retain_mode_str[retain_mode], retain_mode);
	return 1;
}
__setup("muic_state=", muic_state);
/* LGE_CHANGE_E [kenneth.kang@lge.com] 2011-07-26, CP retain mode */

/*
 * Allow user space tools to figure out which devices this driver can control.
 * The first parameter should be 'i2c' for i2c client chip drivers.
 */
static struct i2c_driver muic_driver = {
	.probe	  = muic_probe,
	.remove	  = __devexit_p(muic_remove),
	.suspend  = muic_suspend,
	.resume   = muic_resume,
	.id_table = muic_ids,
	.driver	  = {
		.name	= "max14526",	 // "lg_i2c_muic",
		.owner	= THIS_MODULE,
	},
};

static s32 __init muic_init(void)
{
#if 0//def CONFIG_MACH_STAR_SU660 //from x3
	printk(KERN_WARNING "[MUIC] muic_init()\n");
    muic_wq = create_singlethread_workqueue("muic_wq");
    if (!muic_wq) {
        printk(KERN_WARNING "[MUIC] Failed to create a thread(muic_wq)\n");
    }
#endif
	return i2c_add_driver(&muic_driver);
}

static void __exit muic_exit(void)
{
	DBG();
#if defined (MUIC_SLEEP)
	wake_lock_destroy(&muic_wake_lock);
#endif	
	i2c_del_driver(&muic_driver);
#if 0//def CONFIG_MACH_STAR_SU660 //from x3
destroy_workqueue(muic_wq);
muic_wq = NULL;
#endif
}

#ifdef CONFIG_MACH_STAR
static int __init muic_half_charge_state(char *str)
{
	if(strcmp(str,"charger")==0)
	{
		half_boot_enable=1;
	}
	return 1;
}
__setup("androidboot.mode=", muic_half_charge_state);
#endif

#if defined (CONFIG_MACH_BSSQ)
late_initcall(muic_init);
#else 
module_init(muic_init);
#endif
module_exit(muic_exit);

MODULE_DESCRIPTION("MUIC Driver");
MODULE_LICENSE("GPL");

