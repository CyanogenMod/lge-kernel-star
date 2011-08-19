/*
 * arch/arm/mach-tegra/baseband-xmm-power.h
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

#include <linux/pm.h>
#include <linux/suspend.h>

enum baseband_type {
	BASEBAND_XMM,
};

struct baseband_power_platform_data {
	enum baseband_type baseband_type;
	union {
		struct {
			int mdm_reset;
			int mdm_on;
			int ap2mdm_ack;
			int mdm2ap_ack;
			int ap2mdm_ack2;
			int mdm2ap_ack2;
			struct platform_device *device;
		} generic;
		struct {
			int bb_rst;
			int bb_on;
			int ipc_bb_wake;
			int ipc_ap_wake;
			int ipc_hsic_active;
			int ipc_hsic_sus_req;
			struct platform_device *hsic_device;
		} xmm;
	} modem;
};

static enum {
	BBXMM_PS_UNINIT	= 0,
	BBXMM_PS_INIT	= 1,
	BBXMM_PS_L0	= 2,
	BBXMM_PS_L0TOL2	= 3,
	BBXMM_PS_L2	= 4,
	BBXMM_PS_L2TOL0	= 5,
	BBXMM_PS_L2TOL3	= 6,
	BBXMM_PS_L3	= 7,
	BBXMM_PS_L3TOL0	= 8,
	BBXMM_PS_LAST	= -1,
} baseband_xmm_powerstate;

void baseband_xmm_set_power_status(unsigned int status);
