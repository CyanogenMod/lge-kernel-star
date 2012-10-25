/*
 * Bluetooth+WiFi Murata LBEE19QMBC rfkill power control via GPIO
 *
 * Copyright (C) 2012 LGE Inc.
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

#ifndef __LBEE9QMB_RFKILL_H
#define __LBEE9QMB_RFKILL_H

struct lbee9qmb_platform_data {
	int gpio_reset;
	int gpio_btwake;
	int gpio_hostwake;
	int gpio_pwr;
	int active_low;
	int delay;
// LGE_SJIT_S 11/18/2011 [mohamed.khadri@lge.com] BT UART Enable
	int (*chip_enable)(void);
	int (*chip_disable)(void);
// LGE_SJIT_E 11/18/2011 [mohamed.khadri@lge.com] BT UART Enable
};

#endif
