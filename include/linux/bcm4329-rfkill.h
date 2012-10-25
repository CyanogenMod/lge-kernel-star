/*
 * Bluetooth+WiFi Murata LBEE19QMBC rfkill power control via GPIO
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
#ifndef __BCM4329_RFKILL_H
#define __BCM4329_RFKILL_H


#define BRCM_LPM
#ifdef BRCM_LPM
#define BRCM_HOST_WAKE
#define BRCM_BT_WAKE
#endif


struct bcm4329_rfkill_data {
	int gpio_reset;
#ifdef BRCM_BT_WAKE
	int gpio_btwake;
#endif
#ifdef BRCM_HOST_WAKE
	int gpio_hostwake;
#endif		
	int gpio_shutdown;
	int delay;
	struct clk *bt_32k_clk;
};

#endif
//LGE_CHANGE_E
