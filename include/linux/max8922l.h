/*
 * Charging IC driver (MAX8922L)
 *
 * Copyright (C) 2011 LGE, Inc.
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
 *
 */

#ifndef	MAX8922L_H
#define MAX8922L_H

/* Platform data */
struct charger_ic_platform_data {
	int gpio_en_set;
	int gpio_status;
	int gpio_pgb;
	int irqflags;
};

/*
Pulses	Charge condition	Mode control
---------------------------------------------
0	Default Mode		400mA Default
1	SETI Mode		SETI
2	USB Charge Mode		90mA Mode
3	GSM Test Mode		2.3A GSM RF Test Mode	
4 	Deactive Mode		Deactive
*/

typedef enum {
	CHARGER_USB500 = 0,
	CHARGER_ISET,
	CHARGER_USB100,
	CHARGER_FACTORY,
	CHARGER_DISABLE
} charger_ic_status;

typedef enum {
	CHARGER_STATE_SHUTDOWN = 0,
	CHARGER_STATE_STANDBY,
	CHARGER_STATE_CHARGE,
	CHARGER_STATE_FULLBATTERY,
	CHARGER_STATE_RECHARGE,
} charger_ic_state_machine;

/* GPIO State Getters */
extern int read_gpio_en_set(void);
extern int read_gpio_status(void);
extern int read_gpio_pgb(void);

/* General Functions */
extern void charger_ic_disable(void);
extern void charger_ic_set_mode(charger_ic_status mode);
extern charger_ic_status charger_ic_get_status(void);
extern void charger_ic_set_irq(bool enable);

/* General Charger State */
charger_ic_state_machine charger_ic_get_state(void);
charger_ic_state_machine charger_ic_set_state(charger_ic_state_machine state);
#endif
