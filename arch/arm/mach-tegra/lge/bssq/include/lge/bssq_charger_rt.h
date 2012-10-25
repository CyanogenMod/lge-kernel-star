/*
 * hub Charging IC driver (RT9524)
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

#ifndef RT9524_H
#define RT9524_H

struct charger_rt_platform_data {
	int	gpio_en_set;
	int gpio_status;
	int gpio_pgb;
	int irqflags;
};

/*
Pulses 	Charge Condition 	MODE Control
-----------------------------------------
0 		USB500 Mode 		Charge Current Limit
1 		ISET Mode 			Charge Current Limit
2 		USB100 Mode 		Charge Current Limit
3 		Factory Mode 		Enabled
4 		USB100 Mode 		Charge Current Limit
*/

typedef enum {
	CHARGER_USB500,  	// (395mA) 
	CHARGER_ISET,		// (1A)
	CHARGER_USB100,		// (95mA)
	CHARGER_FACTORY,	// (up to 2.3A)
	CHARGER_DISBALE		
} charger_ic_status;


extern void charger_ic_disable(void);
extern void charger_ic_set_mode(charger_ic_status mode);
extern charger_ic_status charger_ic_get_status(void);
extern void charger_ic_set_irq(bool enable);
#endif
