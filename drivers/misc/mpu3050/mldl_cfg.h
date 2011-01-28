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
 *  @addtogroup MLDL
 *
 *  @{
 *      @file   mldl_cfg.h
 *      @brief  The Motion Library Driver Layer Configuration header file.
 */

#ifndef __MLDL_CFG_H__
#define __MLDL_CFG_H__

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include "mlsl.h"
#include "mpu.h"

/* --------------------- */
/* -    Defines.       - */
/* --------------------- */

#define SAMPLING_PERIOD(mldl_cfg) \
	(((((mldl_cfg->lpf) == 0) || ((mldl_cfg->lpf) == 7))		\
		? (8000)						\
		: (1000))						\
		/ (mldl_cfg->divider + 1))

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/* Platform data for the MPU */
struct mldl_cfg {
	/* MPU related configuration */
	unsigned char addr;
	unsigned char int_config;
	unsigned char ext_sync;
	unsigned char full_scale;
	unsigned char lpf;
	unsigned char clk_src;
	unsigned char divider;
	unsigned char dmp_enable;
	unsigned char fifo_enable;
	unsigned char dmp_cfg1;
	unsigned char dmp_cfg2;
	unsigned char gyro_power;
	unsigned char offset_tc[MPU_NUM_AXES];
	unsigned short offset[MPU_NUM_AXES];
	unsigned char ram[MPU_MEM_NUM_RAM_BANKS][MPU_MEM_BANK_SIZE];

	/* MPU Related stored status and info */
	unsigned char silicon_revision;
	unsigned char product_id;
	unsigned short trim;

	/* Driver/Kernel related state information */
	int is_suspended;

	/* Slave related information */
	struct ext_slave_descr *accel;
	struct ext_slave_descr *compass;
	struct ext_slave_descr *pressure;

	struct mpu3050_platform_data *pdata;
};


int mpu3050_open(struct mldl_cfg *mldl_cfg,
		 void *mlsl_handle,
		 void *accel_handle,
		 void *compass_handle,
		 void *pressure_handle);
int mpu3050_close(struct mldl_cfg *mldl_cfg,
		  void *mlsl_handle,
		  void *accel_handle,
		  void *compass_handle,
		  void *pressure_handle);
int mpu3050_resume(struct mldl_cfg *mldl_cfg,
		   void *mlsl_handle,
		   void *accel_handle,
		   void *compass_handle,
		   void *pressure_handle,
		   bool resume_accel,
		   bool resume_compass,
		   bool resume_pressure);
int mpu3050_suspend(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
		    void *accel_handle,
		    void *compass_handle,
		    void *pressure_handle,
		    bool resume_accel,
		    bool resume_compass,
		    bool resume_pressure);
int mpu3050_read_accel(struct mldl_cfg *mldl_cfg,
		       void *accel_handle,
		       unsigned char *data);
int mpu3050_read_compass(struct mldl_cfg *mldl_cfg,
			 void *compass_handle,
			 unsigned char *data);
int mpu3050_read_pressure(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
			  unsigned char *data);

int mpu3050_config_accel(struct mldl_cfg *mldl_cfg,
			 void *accel_handle,
			 struct ext_slave_config *data);
int mpu3050_config_compass(struct mldl_cfg *mldl_cfg,
			   void *compass_handle,
			   struct ext_slave_config *data);
int mpu3050_config_pressure(struct mldl_cfg *mldl_cfg,
			    void *pressure_handle,
			    struct ext_slave_config *data);


#endif				/* __MLDL_CFG_H__ */

/**
 *@}
 */
