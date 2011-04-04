/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

#include <linux/if_ether.h>

#define ANDROID_ADB		0x0001
#define ANDROID_MSC		0x0002
#define ANDROID_ACM_MODEM	0x0003
#define ANDROID_DIAG		0x0004
#define ANDROID_ACM_NMEA	0x0005
#define ANDROID_GENERIC_MODEM	0x0006
#define ANDROID_GENERIC_NMEA	0x0007
#define ANDROID_CDC_ECM		0x0008
#define ANDROID_RMNET		0x0009
#define ANDROID_RNDIS		0x000A
#ifdef CONFIG_USB_GADGET_LG_MTP_DRIVER
#define ANDROID_MTP   0x000B
#endif
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
#define ANDROID_AUTORUN	0x000C
#endif

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	/* Product ID when adb is enabled. */
	__u16 adb_product_id;

	__u16 version;
	/* Fields for composition switch support */
	struct usb_composition *compositions;
	int num_compositions;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;
	__u16 init_product_id;
	
	/* number of LUNS for mass storage function */
	int nluns;
	int self_powered;
	/* size, in bytes, of the mass storage bulk buffer */
	size_t bulk_size;
};

/* composition support structure */
struct usb_composition {
	__u16   product_id;
	unsigned long functions;
	__u16   adb_product_id;
	unsigned long adb_functions;
};

/* Platform data for "usb_mass_storage" driver.
 * Contains values for the SC_INQUIRY SCSI command. */
struct usb_mass_storage_platform_data {
	char *vendor;
	char *product;
	int release;
	
	/* number of LUNS */
	int nluns;
        size_t bulk_size;
};

/* Platform data for USB ethernet driver. */
struct usb_ether_platform_data {
	u8	ethaddr[ETH_ALEN];
	u32	vendorID;
	const char *vendorDescr;
};

extern void android_usb_set_connected(int on);

#endif	/* __LINUX_USB_ANDROID_H */
