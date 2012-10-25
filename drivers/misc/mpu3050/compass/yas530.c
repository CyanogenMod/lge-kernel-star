/*
 $License:
 
 /*
  $License:
     Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
  $
  */
 /**
  *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
  *  @brief      Provides the interface to setup and handle an accelerometers
  *              connected to the secondary I2C interface of the gyroscope.
  *
  *  @{
  *      @file   yas530.c
  *      @brief  Magnetometer setup and handling methods for Yamaha yas530
  *              compass.
  */
 
 /* ------------------ */
 /* - Include Files. - */
 /* ------------------ */
 
#include "mpu.h"
#include "mltypes.h"
 
#include "mlsl.h"
#include "mlos.h"
 
#ifdef __KERNEL__
#include <asm/uaccess.h>
#include <asm/io.h>
#include <i2c.h>
#endif //__KERNEL__
 
#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-yas530:"
 
 
extern int geomagnetic_api_resume(void);
extern int geomagnetic_api_suspend(void);
extern int geomagnetic_api_read(int *xyz, int *raw, int *xy1y2, int *accuracy);
 
 /*****************************************
     Compass Initialization Functions
 *****************************************/
 
static int 
yas530_suspend
(
   void *mlsl_handle,
   struct ext_slave_descr *slave,
   struct ext_slave_platform_data *pdata
)
{
  int result = ML_SUCCESS;

  geomagnetic_api_suspend();

  return result;
}
 
 
static int 
yas530_resume
(
   void *mlsl_handle,
   struct ext_slave_descr *slave,
   struct ext_slave_platform_data *pdata
)
{
  int result = ML_SUCCESS;

  geomagnetic_api_resume();
  
  return result;
}
  
 
 static int 
 yas530_read
 (
     void *mlsl_handle,
     struct ext_slave_descr *slave,
     struct ext_slave_platform_data *pdata, 
     unsigned char *data
 )
 {
  int result = ML_SUCCESS;
 
  int xyz[3];
  int raw[3];
  short rawfixed[3];
  int xy1y2[3];
  int accuracy;

  geomagnetic_api_read(xyz,raw,xy1y2,&accuracy);

  rawfixed[0] = (short) (xyz[0]/100);
  rawfixed[1] = (short) (xyz[1]/100);
  rawfixed[2] = (short) (xyz[2]/100);

  data[0] = rawfixed[0]>>8;
  data[1] = rawfixed[0] & 0xFF;
  data[2] = rawfixed[1]>>8;
  data[3] = rawfixed[1] & 0xFF;
  data[4] = rawfixed[2]>>8;
  data[5] = rawfixed[2] & 0xFF;

  return result;
 }
 
 
 
 struct ext_slave_descr yas530_descr = {
     /*.init          = */ NULL,
     /*.exit          = */ NULL,
     /*.suspend          = */ yas530_suspend,
     /*.resume           = */ yas530_resume,
     /*.read             = */ yas530_read,
     /*.config           = */ NULL,
  	  /*.get_config       = */ NULL,     
     /*.name             = */ "yas530",
     /*.type             = */ EXT_SLAVE_TYPE_COMPASS,
     /*.id               = */ COMPASS_ID_YAS530,
     /*.reg              = */ 0x06,
     /*.len              = */ 6,
     /*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
     /*.range            = */ {104857, 6000},
 };
 
 struct ext_slave_descr *yas530_get_slave_descr(void)
 {
     return &yas530_descr;
 }
 
#ifdef __KERNEL__
 EXPORT_SYMBOL(yas530_get_slave_descr);
#endif
 
 /**
  *  @}
 **/
