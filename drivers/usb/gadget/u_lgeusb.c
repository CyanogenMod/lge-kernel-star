/* linux/drivers/usb/gadget/u_lgeusb.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2011 LGE.
 * Author : Hyeon H. Park <hyunhui.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

/* #define LGEUSB_DEBUG */
/* #define LGEUSB_DYNAMIC_DEBUG */

#include "u_lgeusb.h"

/* LGE_CHANGE
 * To check factory mode in user space.
 * 2011-02-10, hyunhui.park@lge.com
 */
static struct mutex lock;

static int lgeusb_get_mode(char *buffer, struct kernel_param *kp);
/* Read only */
module_param_call(mode, NULL, lgeusb_get_mode, NULL, S_IRUGO);
MODULE_PARM_DESC(mode, "LGE USB Specific mode");

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
/* LGE_CHANGE
 * To set/get USB user mode to/from user space.
 * 2011-03-09, hyunhui.park@lge.com
 */
static u16 user_mode;
static int lgeusb_set_usermode(const char *val, struct kernel_param *kp);
static int lgeusb_get_usermode(char *buffer, struct kernel_param *kp);
module_param_call(user_mode, lgeusb_set_usermode, lgeusb_get_usermode,
					&user_mode, 0664);
MODULE_PARM_DESC(user_mode, "USB Autorun user mode");
#endif

static struct lgeusb_info *usb_info;

/* FIXME: This length must be same as MAX_STR_LEN in android.c */
#define MAX_SERIAL_NO_LEN 20

static int get_serial_number(char *serial_number)
{
	serial_number[0] = '\0';
	return -1;

/* Temporary comment out */
/* FIXME: This depends on machine type(e.g Qualcomm or nVidia) */
#if 0
	unsigned char nv_imei_ptr[MAX_IMEI_LEN];
	int ret = -1;

	ret = msm_nv_imei_get(nv_imei_ptr);
	if (ret < 0) {
		nv_imei_ptr[0] = '\0';
		lgeusb_info("IMEI is NULL\n");
	} else {
		lgeusb_info("IMEI %s\n", nv_imei_ptr);
	}

	if (nv_imei_ptr[0] != '\0') {
		if ((nv_imei_ptr[0] == '8') && (nv_imei_ptr[1] == '0') &&
				(nv_imei_ptr[2] == 'A')) {
			memset(serial_number, 0, MAX_SERIAL_NO_LEN);
			/* We set serialno include header "80A" */
			memcpy(serial_number, nv_imei_ptr, MAX_IMEI_LEN);
			return 0;
		} else {
			serial_number[0] = '\0';
		}
	} else {
		serial_number[0] = '\0';
	}

	return ret;
#endif
}

static int get_factory_cable(void)
{

	return 0;

/* Temporary comment out */
/* FIXME: This depends on machine type(e.g Qualcomm or nVidia) */
#if 0
	int pif_detect = 0;

#ifdef CONFIG_LGE_DETECT_PIF_PATCH
	pif_detect = lge_get_pif_info();
#endif
	lgeusb_info("Using PIF ZIG (%d)\n", pif_detect);

	if (pif_detect == LGE_PIF_CABLE)
		return LGE_FACTORY_CABLE_TYPE;
	else
		return 0;
#endif
}

static int lgeusb_get_mode(char *buffer, struct kernel_param *kp)
{
	int ret;
	struct lgeusb_info *info = usb_info;

	mutex_lock(&lock);
	ret = sprintf(buffer, "%s",
			(info->current_mode == LGEUSB_FACTORY_MODE
			 ? "factory" : "normal"));
	mutex_unlock(&lock);

	return ret;
}

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
/* LGE_CHANGE
 * To set/get USB user mode to/from user space for autorun.
 * 2011-03-09, hyunhui.park@lge.com
 */
static int lgeusb_set_usermode(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;

	ret = strict_strtoul(val, 16, &tmp);
	if (ret)
		return ret;

	user_mode = (unsigned int)tmp;
	pr_info("autorun user mode : %d\n", user_mode);

	return ret;
}

static int lgeusb_get_usermode(char *buffer, struct kernel_param *kp)
{
	int ret;

	mutex_lock(&lock);
	ret = sprintf(buffer, "%d", user_mode);
	mutex_unlock(&lock);

	return ret;
}

int lgeusb_get_usb_usermode(void)
{
	return user_mode;
}
#endif

static void do_switch_mode(int pid, int need_reset)
{
	struct lgeusb_info *info = usb_info;

	lgeusb_info("do_switch_mode : pid %x, need_reset %d\n", pid, need_reset);
	info->switch_func(pid, need_reset);
}

/* LGE_CHANGE
 * If factory cable (PIF or LT) is connected,
 * return 1, otherwise return 0.
 * 2011-01-13, hyunhui.park@lge.com
 */
int lgeusb_detect_factory_cable(void)
{
	return get_factory_cable();
}

/* LGE_CHANGE
 * If factory dedicated cable is connected,
 * switch to LGE usb factory mode.
 * 2011-01-13, hyunhui.park@lge.com
 */
void lgeusb_switch_factory_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;

	info->current_mode = LGEUSB_FACTORY_MODE;
	info->current_pid = info->get_pid();

	do_switch_mode(LGE_FACTORY_PID, need_reset);
}

/* LGE_CHANGE
 * If a normal cable is connected,
 * switch to android mode back.
 * 2011-01-13, hyunhui.park@lge.com
 */
void lgeusb_switch_android_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;
	int restore_pid = info->current_pid;

	info->current_mode = LGEUSB_ANDROID_MODE;
	do_switch_mode(restore_pid, need_reset);
}

/* LGE_CHANGE
 * Get current mode(factory or android).
 * 2011-01-24, hyunhui.park@lge.com
 */
int lgeusb_get_current_mode(void)
{
	struct lgeusb_info *info = usb_info;

	return (int)info->current_mode;
}

/* LGE_CHANGE
 * 1. If cable is factory cable, switch manufacturing mode.
 * 2. Get serial number from CP and set product id to CP.
 * 2011-01-13, hyunhui.park@lge.com
 */
int lgeusb_set_current_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;
	int ret;

	if (!info->serialno || !info->defaultno) {
		lgeusb_info("serial numbers are invalid, skip configuration.\n");
		return -EINVAL;
	}

	if (get_factory_cable()) {
		/* We already are in factory mode, skip it. */
		if (info->current_mode == LGEUSB_FACTORY_MODE)
			return LGE_FACTORY_PID;

		/* When manufacturing, do not use serial number */
		lgeusb_info("We detect LGE factory cable......\n");
		lgeusb_switch_factory_mode(need_reset);
/* FIXME: It is QCT solution, must not used. */
#if 0
		msm_hsusb_send_productID(LGE_FACTORY_PID);
		msm_hsusb_is_serial_num_null(1);
#endif
		info->serialno[0] = '\0';
		return LGE_FACTORY_PID;
	}

	/* We already are in android mode, skip it. */
	if (info->current_mode == LGEUSB_ANDROID_MODE)
		return info->current_pid;

	lgeusb_info("We detect Normal USB cable......\n");
	lgeusb_switch_android_mode(need_reset);

	ret = get_serial_number(info->serialno);

/* FIXME: It is QCT solution, must not used. */
#if 0
	msm_hsusb_send_productID(info->current_pid);
	msm_hsusb_is_serial_num_null(0);

	if (!ret && (info->serialno[0] != '\0'))
		msm_hsusb_send_serial_number(info->serialno);
	else
		msm_hsusb_send_serial_number(info->defaultno);
#endif

	if (ret < 0)
		lgeusb_info("fail to get serial number, set to default.\n");

	return info->current_pid;
}

/* LGE_CHANGE
 * Register lge usb information(which include callback functions).
 * 2011-01-14, hyunhui.park@lge.com
 */
void lgeusb_register_usbinfo(struct lgeusb_info *info)
{
	if (info) {
		usb_info = info;
		lgeusb_info("Registering infomation for lgeusb is success\n");

		lgeusb_debug("switch_func %p, get_pid %p\n",
				usb_info->switch_func,
				usb_info->get_pid);
	} else {
		lgeusb_info("Registering infomation for lgwusb is failed\n");
	}
}

static int __init lgeusb_init(void)
{
	lgeusb_info("u_lgeusb init\n");
	mutex_init(&lock);

	return 0;
}
module_init(lgeusb_init);
