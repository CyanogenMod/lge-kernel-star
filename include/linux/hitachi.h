/*
 * linux/include/linux/hitachi_lcd.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_HITACHI_LCD_H
#define __LINUX_HITACHI_LCD_H

#define HITACHI_LCD_DRV_NAME	"hitachi-lcd"

struct hitachi_lcd_platform_data {
	int reset_pin;
	int cs_pin;
	int dc_pin;
	int wr_pin;
	int data_pin[8];
#if defined (CONFIG_MACH_STAR)
	int maker_id_pin;
#endif // MOBII_CHANGE_S [jg.noh@mobii.co.kr] 2012.01.31 Supporting dual lcd driver that including both hitachi and lgd pannels
};

#endif /* __LINUX_HITACHI_LCD_H */
