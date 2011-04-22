/*
 * Copyright (C) 2009 LGE Inc.
 *
 * MPU3050 gyroscope sensor driver header file.
 *
 * 2009/09/05 : created by <>
 *
 */

#ifndef __MPU3050_H__
#define __MPU3050_H__

#include <linux/i2c.h>


/*==================================================================================================
  MPU3050 GYRO REGISTER MAP
  ==================================================================================================*/

#define MPU3050_GYRO_I2C_WHO_AM_I       		0x00
#define MPU3050_GYRO_I2C_PRODUCT_ID    		0x01

#define MPU3050_GYRO_I2C_X_OFFS_USRH   		0x0C
#define MPU3050_GYRO_I2C_X_OFFS_USRL   		0x0D
#define MPU3050_GYRO_I2C_Y_OFFS_USRH   		0x0E
#define MPU3050_GYRO_I2C_Y_OFFS_USRL   		0x0F
#define MPU3050_GYRO_I2C_Z_OFFS_USRH   		0x10
#define MPU3050_GYRO_I2C_Z_OFFS_USRL   		0x11

#define MPU3050_GYRO_I2C_ACCEL_SLAVE_ADDR   0x14

#define MPU3050_GYRO_I2C_SMPLR_DIV      		0x15
#define MPU3050_GYRO_I2C_DLPF_FS_SYNC      	0x16
#define MPU3050_GYRO_I2C_INT_CFG      			0x17
#define MPU3050_GYRO_I2C_ACCEL_BURST_ADDR   0x18
#define MPU3050_GYRO_I2C_INT_STATUS      	       0x1A

#define MPU3050_GYRO_I2C_TEMP_OUT_H      		0x1B
#define MPU3050_GYRO_I2C_TEMP_OUT_L      		0x1C
#define MPU3050_GYRO_I2C_GYRO_XOUT_H      		0x1D
#define MPU3050_GYRO_I2C_GYRO_XOUT_L      		0x1E
#define MPU3050_GYRO_I2C_GYRO_YOUT_H      		0x1F
#define MPU3050_GYRO_I2C_GYRO_YOUT_L      		0x20
#define MPU3050_GYRO_I2C_GYRO_ZOUT_H      		0x21
#define MPU3050_GYRO_I2C_GYRO_ZOUT_L      		0x22
#define MPU3050_GYRO_I2C_ACCEL_XOUT_H      	0x23
#define MPU3050_GYRO_I2C_ACCEL_XOUT_L      	0x24
#define MPU3050_GYRO_I2C_ACCEL_YOUT_H      	0x25
#define MPU3050_GYRO_I2C_ACCEL_YOUT_L      	0x26
#define MPU3050_GYRO_I2C_ACCEL_ZOUT_H      	0x27
#define MPU3050_GYRO_I2C_ACCEL_ZOUT_L      	0x28
#define MPU3050_GYRO_I2C_AUX_1OUT_H      		0x29
#define MPU3050_GYRO_I2C_AUX_1OUT_L      		0x2A
#define MPU3050_GYRO_I2C_AUX_2OUT_H      		0x2B
#define MPU3050_GYRO_I2C_AUX_2OUT_L      		0x2C
#define MPU3050_GYRO_I2C_AUX_3OUT_H      		0x2D
#define MPU3050_GYRO_I2C_AUX_3OUT_L      		0x2E

#define MPU3050_GYRO_I2C_USER_CTRL      		0x3D
#define MPU3050_GYRO_I2C_PWR_MGM      		0x3E

#define MPUID_VERSIONB1						7

/*==================================================================================================
  Definitions
  ==================================================================================================*/

#define  GYRO_I2C_SLAVE_ADDR        		0x68

#define MPU3050_BYPASS_MODE_ON 		1
#define MPU3050_BYPASS_MODE_OFF 		0

/*-------------------------------------------------------------------*/
/*		   			                   IOCTL					                        */
/*-------------------------------------------------------------------*/

/*----------------- motion daemon process -------------------------*/
#define MOTION_MAGIC					     		0xA2

#define MOTION_IOCTL_ENABLE_DISABLE	    		_IOWR(MOTION_MAGIC, 0x01, short)
#define MOTION_IOCTL_SENSOR_DELAY	    		_IOWR(MOTION_MAGIC, 0x02, short)
#define MOTION_IOCTL_READ_ACCEL_DATA   			_IOWR(MOTION_MAGIC, 0x03, char[10])   // ACCEL RAW DATA
#define MOTION_IOCTL_ACCEL_RAW      	    	_IOWR(MOTION_MAGIC, 0x04, int[5])
#define MOTION_IOCTL_TILT            	    	_IOWR(MOTION_MAGIC, 0x05, int[5])
#define MOTION_IOCTL_SHAKE        	    	    _IOWR(MOTION_MAGIC, 0x06, int[5])
#define MOTION_IOCTL_SNAP        	    	    _IOWR(MOTION_MAGIC, 0x07, int[5])
#define MOTION_IOCTL_FLIP        	    	    _IOWR(MOTION_MAGIC, 0x08, int[5])
#define MOTION_IOCTL_TAP             	    	_IOWR(MOTION_MAGIC, 0x09, int[5])
#define MOTION_IOCTL_COMPOSITE         	    _IOWR(MOTION_MAGIC, 0x0A, int[5])
#define MOTION_IOCTL_YAWIMAGE         	    	_IOWR(MOTION_MAGIC, 0x0B, int[5])
//#define MOTION_IOCTL_GYRO_RAW      	    		_IOWR(MOTION_MAGIC, 0x0C, int[5])


// for  MPL library - MPU3050 Gyroscpoe
#define MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP 		_IOWR(MOTION_MAGIC, 0x0C, short)
#define MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE   		_IOWR(MOTION_MAGIC, 0x0D, short)
#define MOTION_IOCTL_MPU3050_I2C_READ     	   	_IOWR(MOTION_MAGIC, 0x0E, char[200])
#define MOTION_IOCTL_MPU3050_I2C_WRITE   	   	_IOWR(MOTION_MAGIC, 0x0F, char[200])

#define MOTION_IOCTL_GYRO_RAW      	    		_IOWR(MOTION_MAGIC, 0x11, int[5])
#define MOTION_IOCTL_SENSOR_SUSPEND_RESUME	    		_IOWR(MOTION_MAGIC, 0x12, short)
#define MOTION_IOCTL_MAGNETIC_RAW   			_IOWR(MOTION_MAGIC, 0x10, int[5]) 
//#define MOTION_IOCTL_WAIT_SENSOR_INIT	    		_IOWR(MOTION_MAGIC, 0x13, int)

void  mpu3050_reg_i2c_client(struct i2c_client *client);
void  mpu3050_initialize(void);
void  mpu3050_i2c_through_pass(int benable);
void  mpu3050_read_gyro_xyz(unsigned char *data_xyz);
void  mpu3050_sleep_mode(void);
void  mpu3050_sleep_wake_up(void);

/*----------------------------------------------------------------*/
int mpu3050_i2c_read(unsigned char reg, unsigned char *buf, int length);
int mpu3050_i2c_write(unsigned char *buffer, int length);

#endif //__MPU3050_H__
