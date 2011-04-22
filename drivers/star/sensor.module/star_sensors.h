#ifndef __STAR_SENSOR_H
#define __STAR_SENSOR_H

#include "mach/lprintk.h"

/*==================================================================================================
  LGE Definitions
  ==================================================================================================*/

#define  MAX_MOTION_DATA_LENGTH	 	10
//#define  MIN_MOTION_POLLING_TIME    70//  motion daemon update every 70mS
#define  MIN_MOTION_POLLING_TIME    70//  motion daemon update every 70mS
#define  DEFAULT_MOTION_POLLING_TIME    200

#define  STAR_MOTION_SENSOR_NAME		"motion_sensor"		/* platform device*/
#define  STAR_MOTION_IOCTL_NAME			"motion_daemon"		/* for star_motion(gyro) daemon process */
#define  STAR_ACCEL_IOCTL_NAME			"accel_daemon"		/* for star_motion(accel) daemon process */
#define  STAR_COMPASS_IOCTL_NAME		"compass_daemon"	/* for star_motion(compass) daemon process */
#define  STAR_MOTION_INPUT_NAME			"gyro_accel"		/*"lge_gesture"*/
#define  STAR_I2C_ACCEL_NAME			"star_accel"
#define  STAR_I2C_GYRO_NAME				"star_gyro"

#define  MISC_DYNAMIC_MINOR         255
#define  MAX_MOTION_DATA_LENGTH     10

typedef enum {
	STAR_SENSOR_NONE	= 0x00,
	STAR_ACCELEROMETER	= 0x01,
	STAR_ORIENTATION	= 0x02,
	STAR_TILT			= 0x04,
	STAR_SHAKE			= 0x08,
	STAR_SNAP			= 0x10,
	STAR_FLIP			= 0x20,
	STAR_TAP			= 0x40,
	STAR_YAWIMAGE		= 0x80,
	STAR_GYRO			= 0x100,
	STAR_COMPASS		= 0x200,
	STAR_COMPOSITE	= 0x800,	
} star_sensor_enum_type;

#define STAR_SENSOR_IOCTL_NAME			"sensors"			/* for check sensor initialization */
#define STAR_IOCTL_BASE		0x90
#define	MOTION_IOCTL_WAIT_SENSOR_INIT	    		_IOWR(STAR_IOCTL_BASE, 0x01, int)

#endif
