/*
 * include/linux/smb349-charger.h
 *
 * Battery charger driver interface for Summit SMB349
 *
 * Copyright (C) 2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __LINUX_SMB349_CHARGER_H
#define __LINUX_SMB349_CHARGER_H

#include <linux/regulator/machine.h>

struct smb349_charger {
	struct i2c_client	*client;
	struct device	*dev;
};

/*
 * Register the callback function for the client.
 */
extern int smb349_battery_online(void);
extern int smb349_charging_status(void);
extern int smb349_charger_type(void);

#endif /*__LINUX_SMB349_CHARGER_H */
