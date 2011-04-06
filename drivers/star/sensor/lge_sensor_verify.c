/*
 *  lg_sensor_verify.c
 *  star motion sensor driver to verify devices(Accelerometer,Gyroscope Sensor)
 *  Copyright (C) Hee Seo 2010 LGE Inc.
 *
 *  NOTE! This module has to be loaded as module_init_sync 
 *  after the initialization of drivers for star sensor deivces.  
 *
 *  Now kernel macro module_init will be invoked prior to module_init_sync! 
 * 
 *  #define module_init_sync __initcall_sync 
 *  #define __initcall_sync device_initcall_sync 
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <asm/system.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/reboot.h>
#include <asm/uaccess.h>
#include "lge_sensor_verify.h"

#define TIMEOUT 5
#define RESET_TIMEOUT 3 
extern int lge_sensor_verify_gyro(void); 
extern int lge_sensor_verify_kxtf9(void); 
extern int lge_sensor_verify_compass(void); 
extern int lge_sensor_reset_gyro(void);
static int __init lge_sensor_verify_init(void)
{
#if 0
	int timeout = 0; 
	unsigned short flag = 0;  
	unsigned int rtVal = 0;

	for(timeout = 0 ; timeout < TIMEOUT ; timeout++){ 

		if(!lge_sensor_verify_gyro())
			rtVal <<= SENSOR_GYRO_ID; 

		if(!lge_sensor_verify_kxtf9())
			rtVal <<= SENSOR_KXTF9_ID; 

		if(!lge_sensor_verify_compass())
			rtVal <<= SENSOR_COMPASS_ID; 

		if(timeout == RESET_TIMEOUT)
			lge_sensor_reset_gyro();

		if((rtVal)){
			rtVal = 0;
			timeout++;
			continue;
		} else {
#if LG_SENSOR_DEBUG 
			lprintk(KERN_INFO "[%s] the verification of sensor devices are OK!\n",MOD_TAG);
#endif
			return 0;
		}
	}
	// do reboot system 
	emergency_restart();
#endif
	return 0;
}

static void __exit lge_sensor_verify_exit(void)
{
#if LG_SENSOR_DEBUG 
	lprintk(KERN_INFO "[%s] moudule is unloaded\n",MOD_TAG);
#endif
	return;
}

late_initcall(lge_sensor_verify_init);
module_exit(lge_sensor_verify_exit);

MODULE_AUTHOR("HEE SEO from LG Electronics");
MODULE_DESCRIPTION("Star Driver for verification");
MODULE_LICENSE("GPL");


