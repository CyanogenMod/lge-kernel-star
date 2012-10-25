/* drivers/broadcast/t3900/src/tdmb_tunerbbdrv_t3900.c
 * Copyright (C) 2011 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TDMB_TUNERDRV_T3900_H__
#define __TDMB_TUNERDRV_T3900_H__

#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_t3900.h"
#include "../inc/INC_INCLUDES.h"

/* ----------------------------------------------------------
**    1.   DEFINITIONS
-------------------------------------------------------------*/

//#define TDMB_UPLOAD_MODE_TSIF
//#define TDMB_UPLOAD_MODE_EBI
#define TDMB_UPLOAD_MODE_SPI
	
#if defined(TDMB_UPLOAD_MODE_TSIF)
#define STREAM_TS_UPLOAD
#elif defined(TDMB_UPLOAD_MODE_EBI)
#define STREAM_SLAVE_PARALLEL_UPLOAD
#elif defined(TDMB_UPLOAD_MODE_SPI)
#define STREAM_SPI_UPLOAD
#endif
	
#define TDMB_RFBB_DEV_ADDR	0x40


/* ----------------------------------------------------------
**    2.   External variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    3.   External Functions
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    4.   Local Constant Variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    5.   Local Typedef
-------------------------------------------------------------*/
typedef enum
{
	TDMB_BB_DATA_TS,
	TDMB_BB_DATA_DAB,
	TDMB_BB_DATA_PACK,
	TDMB_BB_DATA_FIC,
	TDMB_BB_DATA_FIDC
} TDMB_BB_DATA_TYPE;

typedef struct
{
	uint16	reserved;
	uint8	subch_id;
	uint16	size;
	uint8	data_type:7;
	uint8	ack_bit:1;
} TDMB_BB_HEADER_TYPE;

/* ----------------------------------------------------------
**    6.   Global Variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    7.   Static Variables
-------------------------------------------------------------*/




/* ----------------------------------------------------------
**    8.   Functions
-------------------------------------------------------------*/
void tunerbb_drv_t3900_set_userstop(int mode);
int8 tunerbb_drv_t3900_power_on(void);
int8 tunerbb_drv_t3900_power_off(void);
int8 tunerbb_drv_t3900_select_antenna(unsigned int sel);
int8 tunerbb_drv_t3900_reset_ch(void);
int8 tunerbb_drv_t3900_init(void);
int8 tunerbb_drv_t3900_stop(void);
int8 tunerbb_drv_t3900_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info);
int8 tunerbb_drv_t3900_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[ ], uint8 op_mode[ ]);
int8 tunerbb_drv_t3900_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode);
int8 tunerbb_drv_t3900_re_syncdetector(uint8 op_mode);
int8 tunerbb_drv_t3900_re_sync(void);
int8 tunerbb_drv_t3900_get_fic(uint8* buffer, uint32* buffer_size /* bool crc_onoff*/);
int8 tunerbb_drv_t3900_read_data(uint8* buffer, uint32* buffer_size);
void tunerbb_drv_lg2102_start_tii(void);
void tunerbb_drv_lg2102_stop_tii(void);
boolean tunerbb_drv_lg2102_check_tii(uint8* pmain_tii, uint8* psub_tii);
#endif 
