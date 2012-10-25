/*
 *  Copyright (C) 2009 LG Electronics
 *  Dajin Kim <dajin.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17043_FUELGAUGE_H_
#define __MAX17043_FUELGAUGE_H_

typedef enum {
	MAX17403_UNKNOWN,
	MAX17043_RESET,
	MAX17043_QUICKSTART,
	MAX17043_WORKING,
	MAX17043_STATE_MAX
} max17043_status;

extern int max17043_get_capacity(void);
extern int max17043_get_voltage(void);
extern int max17043_do_calibrate(void);
extern int max17043_set_rcomp_by_temperature(int temp);
extern int max17043_set_alert_level(int alert_level);

#endif
