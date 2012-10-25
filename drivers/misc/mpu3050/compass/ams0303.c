/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   COMPASSDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   AMS0303.c
 *      @brief  Magnetometer setup and handling methods for AMS0303 compass.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

/* reg address */
#define AMS0303M_REG_POWER (0x03)
#define AMS0303M_REG_DATA (0x06)
#define AMS0303M_REG_CONTROL (0x2E)
#define AMS0303M_REG_CONTROL_2 (0x21)
#define AMS0303M_REG_CONTROL_3 (0x04)
#define AMS0303M_REG_DAC_X1H (0x22)
#define AMS0303M_REG_DAC_X2H (0x24)
#define AMS0303M_REG_DAC_Y1H (0x26)
#define AMS0303M_REG_DAC_Y2H (0x28)
#define AMS0303M_REG_DAC_Z1H (0x2A)
#define AMS0303M_REG_DAC_Z2H (0x2C)

/* reg value */
#define AMS0303M_VAL_POWER_UP (0x9F)
#define AMS0303M_VAL_OFF (0x00)
#define AMS0303M_VAL_CONT_MODE_ON (0xBE)
#define AMS0303M_VAL_GAIN (0x07)

#define AMS0303M_DEVICE_LIMIT_DAC 10
#define AMS0303M_DEVICE_DAC_MAX 1000
#define AMS0303M_DEVICE_DAC_MIN 20
#define AMS0303M_DEVICE_MAX_VALUE 1023


static int g_ams0303_sum_dac_xyz[6] = {512,512,512,512,512,512};
static int g_ams0303_dac_xyz[6]={512,512,512,512,512,512};
static int g_ams0303_d_dac_xyz[6] = {0,0,0,0,0,0};


/* amosense function */
int ams0303_power_up(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMS0303M_REG_POWER,
				  AMS0303M_VAL_POWER_UP);
	MLOSSleep(1);		/* wait at least 100us */
	ERROR_CHECK(result);
	
	return result;
}

int ams0303_power_down(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMS0303M_REG_POWER,
				  AMS0303M_VAL_OFF);
	MLOSSleep(1);		/* wait at least 100us */
	ERROR_CHECK(result);
	
	return result;
}

int ams0303_start_continuous_mode(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMS0303M_REG_CONTROL,
				  AMS0303M_VAL_CONT_MODE_ON);
	
	ERROR_CHECK(result);
	
	return result;
}

int ams0303_stop_continuous_mode(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMS0303M_REG_CONTROL,
				  AMS0303M_VAL_OFF);
	
	ERROR_CHECK(result);
	
	return result;
}

int ams0303_set_stabilized_time(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata,
		   unsigned char time)
{		
	int result = ML_SUCCESS;
	
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMS0303M_REG_CONTROL_3,
				  time);
	
	ERROR_CHECK(result);
	
	return result;
}

int ams0303_load_dac_offset(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	g_ams0303_sum_dac_xyz[0] = g_ams0303_dac_xyz[0] + g_ams0303_d_dac_xyz[0];
	g_ams0303_sum_dac_xyz[1] = g_ams0303_dac_xyz[1] + g_ams0303_d_dac_xyz[1];
	g_ams0303_sum_dac_xyz[2] = g_ams0303_dac_xyz[2] + g_ams0303_d_dac_xyz[2];
	g_ams0303_sum_dac_xyz[3] = g_ams0303_dac_xyz[3] + g_ams0303_d_dac_xyz[3];
	g_ams0303_sum_dac_xyz[4] = g_ams0303_dac_xyz[4] + g_ams0303_d_dac_xyz[4];
	g_ams0303_sum_dac_xyz[5] = g_ams0303_dac_xyz[5] + g_ams0303_d_dac_xyz[5];
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_CONTROL_2, AMS0303M_VAL_GAIN);
	ERROR_CHECK(result);
	
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_X1H,	(unsigned char)(g_ams0303_sum_dac_xyz[2]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_X1H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[2]&0xFF));
	ERROR_CHECK(result);
	
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_X2H,	(unsigned char)(g_ams0303_sum_dac_xyz[3]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_X2H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[3]&0xFF));
	ERROR_CHECK(result);
	
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Y1H,	(unsigned char)(g_ams0303_sum_dac_xyz[0]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Y1H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[0]&0xFF));
	ERROR_CHECK(result);

	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Y2H,	(unsigned char)(g_ams0303_sum_dac_xyz[1]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Y2H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[1]&0xFF));
	ERROR_CHECK(result);

	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Z1H, (unsigned char)(g_ams0303_sum_dac_xyz[4]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Z1H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[4]&0xFF));
	ERROR_CHECK(result);

	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Z2H, (unsigned char)(g_ams0303_sum_dac_xyz[5]>>8)&0xFF);
	ERROR_CHECK(result);
	
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, AMS0303M_REG_DAC_Z2H+0x01, (unsigned char)(g_ams0303_sum_dac_xyz[5]&0xFF));
	ERROR_CHECK(result);

	return result;
}
/* end amosense function */


int ams0303_init(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result = ams0303_power_up(mlsl_handle, slave, pdata);
	result = ams0303_load_dac_offset(mlsl_handle, slave, pdata);
	result = ams0303_set_stabilized_time(mlsl_handle, slave, pdata, 0x02);
	result = ams0303_power_down(mlsl_handle, slave, pdata);
	
	return result;
}

int ams0303_suspend(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result = ams0303_stop_continuous_mode(mlsl_handle, slave, pdata);
	result = ams0303_power_down(mlsl_handle, slave, pdata);
	
	return result;
}

int ams0303_resume(void *mlsl_handle,
		  struct ext_slave_descr *slave,
		  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	
	result = ams0303_power_up(mlsl_handle, slave, pdata);
	result = ams0303_start_continuous_mode(mlsl_handle, slave, pdata);	
	
	return result;
}

int ams0303_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata, unsigned char *data)
{
	int result = ML_SUCCESS;

	int	temp_Mx1=0,temp_My1=0,temp_Mz1=0;

	
	short mx00=0;
	short mx90=0;
	short my00=0;
	short my90=0;
	short mz00=0;
	short mz90=0;
	short ret = 0;
	unsigned char buffer[13];
	short ret_data[3];

	result = ams0303_stop_continuous_mode(mlsl_handle, slave, pdata);
	ERROR_CHECK(result);
	
	result = MLSLSerialRead(mlsl_handle, pdata->address, AMS0303M_REG_DATA, 12, (unsigned char *)buffer);
	ERROR_CHECK(result);

	mx00 = (short)( (short)((buffer[4]<<8) |  (buffer[5]&0xFF)) );
	my00 = (short)( (short)((buffer[0]<<8) |  (buffer[1]&0xFF)) );
	mz00 = (short)( (short)((buffer[8]<<8) |  (buffer[9]&0xFF)) );

	mx90 = (short)( (short)((buffer[6]<<8) |  (buffer[7]&0xFF)) );
	my90 = (short)( (short)((buffer[2]<<8) |  (buffer[3]&0xFF)) );
	mz90 = (short)( (short)((buffer[10]<<8) |  (buffer[11]&0xFF)) );

	// auto dac calibration
	if( (mx00 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[0]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[0]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mx00 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[0]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[0]-=AMS0303M_DEVICE_LIMIT_DAC;

	if( (my00 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[2]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[2]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (my00 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[2]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[2]-=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mz00 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[4]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[4]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mz00 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[4]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[4]-=AMS0303M_DEVICE_LIMIT_DAC;

	//
	if( (mx90 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[1]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[1]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mx90 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[1]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[1]-=AMS0303M_DEVICE_LIMIT_DAC;

	if( (my90 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[3]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[3]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (my90 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[3]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[3]-=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mz90 > AMS0303M_DEVICE_DAC_MAX) && (g_ams0303_sum_dac_xyz[5]<(AMS0303M_DEVICE_MAX_VALUE-AMS0303M_DEVICE_LIMIT_DAC)) )
		g_ams0303_d_dac_xyz[5]+=AMS0303M_DEVICE_LIMIT_DAC;

	if( (mz90 < AMS0303M_DEVICE_DAC_MIN) && (g_ams0303_sum_dac_xyz[5]>AMS0303M_DEVICE_LIMIT_DAC) )
		g_ams0303_d_dac_xyz[5]-=AMS0303M_DEVICE_LIMIT_DAC;

	result = ams0303_load_dac_offset(mlsl_handle, slave, pdata);
	result = ams0303_set_stabilized_time(mlsl_handle, slave, pdata, 0x00);
	result = ams0303_power_up(mlsl_handle, slave, pdata);
	result = ams0303_start_continuous_mode(mlsl_handle, slave, pdata);
		
	ret_data[0] = (mx90-mx00) + (g_ams0303_d_dac_xyz[1]-g_ams0303_d_dac_xyz[0]) * 15;
	ret_data[1] = (my00-my90) + (g_ams0303_d_dac_xyz[2]-g_ams0303_d_dac_xyz[3]) * 15;
	ret_data[2] = (mz90-mz00) + (g_ams0303_d_dac_xyz[5]-g_ams0303_d_dac_xyz[4]) * 15;		



	// AMS0303M 축변환.
	//temp_Mx=(MotionData.My+MotionData.Mx/2+MotionData.Mz/2);
	//temp_My=-1*(MotionData.Mz-MotionData.Mx)*0.85;	
	//temp_Mz=-1*(MotionData.Mx-MotionData.My+MotionData.Mz)/3;

	// X, Y, Z축 변환.scaling *100   2011.02.08
	temp_Mx1=ret_data[1]*100+ret_data[0]*50+ret_data[2]*50;
	temp_My1=-1*(ret_data[2]-ret_data[0])*85;
	temp_Mz1=-1*(ret_data[0]-ret_data[1]+ret_data[2])*33;

	// X, Y, Z축 변환.scaling /100
	ret_data[0]=(short)(temp_Mx1/100);
	ret_data[1]=(short)(temp_My1/100);
	ret_data[2]=(short)(temp_Mz1/100);

      // printk("[0]:%d,[1]%d,[2]=%d\n",ret_data[0],ret_data[1],ret_data[2]);


	data[0] = (unsigned char)((ret_data[0]>>8)&0xFF);
	data[1] = (unsigned char)(ret_data[0]&0xFF);
	data[2] = (unsigned char)((ret_data[1]>>8)&0xFF);
	data[3] = (unsigned char)(ret_data[1]&0xFF);
	data[4] = (unsigned char)((ret_data[2]>>8)&0xFF);
	data[5] = (unsigned char)(ret_data[2]&0xFF);	




	return result;
}

struct ext_slave_descr ams0303_descr = {
	/*.init             = */ ams0303_init,
	/*.exit             = */ NULL,
	/*.suspend          = */ ams0303_suspend,
	/*.resume           = */ ams0303_resume,
	/*.read             = */ ams0303_read,
	/*.config           = */ NULL,
	/*.get_config       = */ NULL,	
	/*.name             = */ "ams0303",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_AMS0303,
	/*.reg              = */ 0x06,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {9830, 4000}
};

struct ext_slave_descr *ams0303_get_slave_descr(void)
{
	return &ams0303_descr;
}
EXPORT_SYMBOL(ams0303_get_slave_descr);

/**
 *  @}
**/

