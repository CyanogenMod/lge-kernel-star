/*
 * Copyright (C) 2009 LGE Inc.
 *
 * MPU3050 gyroscope sensor driver header file.
 *
 * 2009/09/05 : created by <chpark96@lge.com>
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

#define IME_IF_ENA_POS          0x20 // 2nd I2C interface enable/disable bit

void  mpu3050_reg_i2c_client(struct i2c_client *client);
void  mpu3050_initialize(void);
unsigned short mpu3050_i2c_through_pass(unsigned char enable);
void  mpu3050_read_gyro_xyz(unsigned char *data_xyz);
void  mpu3050_sleep_mode(void);
void  mpu3050_sleep_wake_up(void);

/*----------------------------------------------------------------*/
int mpu3050_i2c_read(unsigned char reg, unsigned char *buf, int length);
int mpu3050_i2c_write(unsigned char *buffer, int length);

#endif //__MPU3050_H__
