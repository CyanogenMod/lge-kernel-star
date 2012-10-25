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

/*
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   bma250.c
 *      @brief  Accelerometer setup and handling methods.
 */
/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlos.h"
#include "mlsl.h"


/* full scale setting - register and mask */
#define ACCEL_BMA250_CTRL_REG       (0x0f)  /* BMA250 : full scale setting register */
#define ACCEL_BMA250_CTRL_MASK      (0x0f)  /* BMA250 : full scale setting mask */
#define ACCEL_BMA250_BW_REG       (0x10)  /* BMA250 : BW setting register */
#define ACCEL_BMA250_BW_MASK      (0x1f)     /* BMA250 : BW setting mask */


/*********************************************
    Accelerometer Initialization Functions
**********************************************/

static int bma250_suspend(void *mlsl_handle,
                          struct ext_slave_descr *slave,
                          struct ext_slave_platform_data *pdata)
{
    int result;
    result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, 0x14, 0xb6); /* BMA250 : Software reset */
    return result;
}

static int bma250_resume(void * mlsl_handle,
                         struct ext_slave_descr *slave,
                         struct ext_slave_platform_data *pdata)
{

    int result;

    unsigned char reg = 0;

    /* Soft reset */

    result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, 0x14, 0xb6 );		  /* BMA250 : Software reset */

    ERROR_CHECK(result);

    MLOSSleep(10);


    result = MLSLSerialRead(mlsl_handle, pdata->address, ACCEL_BMA250_CTRL_REG, 1, &reg);

    ERROR_CHECK(result);

    /* BMA250 : Full Scale */

    reg &= ~ACCEL_BMA250_CTRL_MASK;

    reg |= 0x00;

    if (slave->range.mantissa==2) {

        reg |= 0x03;

    } else if (slave->range.mantissa==4) {

        reg |= 0x05;

    } else if (slave->range.mantissa==8) {

        reg |= 0x08;

    }

    result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, ACCEL_BMA250_CTRL_REG, reg );

    ERROR_CHECK(result);

    result = MLSLSerialRead(mlsl_handle, pdata->address, ACCEL_BMA250_BW_REG, 1, &reg);

    ERROR_CHECK(result);

    reg &= ~ACCEL_BMA250_BW_MASK;

    reg |= 0x00;


    /* BMA250: Bandwidth */

    reg |= 0x0b;	  // bw=62.5Hz-8ms

    result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, ACCEL_BMA250_BW_REG, reg );

    ERROR_CHECK(result);

    return result;
}



static int bma250_read(void * mlsl_handle,
                       struct ext_slave_descr *slave,
                       struct ext_slave_platform_data *pdata,
                       unsigned char *data)
{
    int result;
    result = MLSLSerialRead(mlsl_handle, pdata->address,
                            slave->reg, slave->len, data);
    return result;
}


static struct ext_slave_descr bma250_descr = {
    /*.init             = */ NULL,
    /*.exit             = */ NULL,
    /*.suspend          = */ bma250_suspend,
    /*.resume           = */ bma250_resume,
    /*.read             = */ bma250_read,
    /*.config           = */ NULL,
    /*.get_config       = */ NULL,
    /*.name             = */ "bma250",
    /*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
    /*.id               = */ ACCEL_ID_BMA250,
    /*.reg              = */ 0x02,
    /*.len              = */ 6,
    /*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
    /*.range            = */ { 2, 0 },
};

struct ext_slave_descr *bma250_get_slave_descr(void)
{
    return &bma250_descr;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(bma250_get_slave_descr);
#endif

#ifdef __KERNEL__
MODULE_AUTHOR("Invensense");
MODULE_DESCRIPTION("User space IRQ handler for MPU3xxx devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bma");
#endif

/**
 *  @}
**/


