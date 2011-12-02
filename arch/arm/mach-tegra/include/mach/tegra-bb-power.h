/*
 * arch/arm/mach-tegra/include/mach/tegra-bb-power.h
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#define GPIO_INVALID UINT_MAX

union tegra_bb_gpio_id {
	struct {
		int mdm_reset;
		int mdm_on;
		int ap2mdm_ack;
		int mdm2ap_ack;
		int ap2mdm_ack2;
		int mdm2ap_ack2;
		int rsvd1;
		int rsvd2;
	} generic;
	struct {
		int bb_rst;
		int bb_on;
		int ipc_bb_wake;
		int ipc_ap_wake;
		int ipc_hsic_active;
		int ipc_hsic_sus_req;
		int rsvd1;
		int rsvd2;
	} xmm;
	struct {
		int pwr_status;
		int pwr_on;
		int uart_awr;
		int uart_cwr;
		int usb_awr;
		int usb_cwr;
		int service;
		int resout2;
	} m7400;
};

typedef struct platform_device* (*ehci_register_cb)(void);
typedef void (*ehci_unregister_cb)(struct platform_device *);

struct tegra_bb_pdata {
	union tegra_bb_gpio_id *id;
	struct platform_device *device;
	ehci_register_cb ehci_register;
	ehci_unregister_cb ehci_unregister;
	int bb_id;
};
