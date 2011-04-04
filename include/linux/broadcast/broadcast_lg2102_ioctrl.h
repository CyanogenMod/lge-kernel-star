/*****************************************************************************
 Copyright(c) 2008 LG Electronics Inc. All Rights Reserved
 
 File name : tdmb_tunerbbdrv_lg2102def.h
 
 Description : LG2102 Driver Header File
 
 History : 
 ----------------------------------------------------------------------
 Mar. 10,  2009 :   LGSIC release for LGMC
*******************************************************************************/
#ifndef	__TUNERBB_DRV_LG2102_H__
#define	__TUNERBB_DRV_LG2102_H__
/*
** Include Header File
*/

#include <linux/broadcast/broadcast_tdmb_typedef.h>
#include <linux/broadcast/broadcast_tdmb.h>

/*============================================================
**    1.   DEFINITIONS
*============================================================*/

#define TDMB_UPLOAD_MODE_SPI

#if defined(TDMB_UPLOAD_MODE_TSIF)
//#define STREAM_TS_UPLOAD
#elif defined(TDMB_UPLOAD_MODE_EBI)
//#define STREAM_SLAVE_PARALLEL_UPLOAD
#elif defined(TDMB_UPLOAD_MODE_SPI)
#define STREAM_SPI_UPLOAD
#endif
	
	//#define STREAM_MASTER_SERIAL_UPLOAD
	//#define STREAM_MASTER_PARALLEL_UPLOAD
	//#define STREAM_SLAVE_SERIAL_UPLOAD
	//#define STREAM_SPI_UPLOAD
	
#if defined(STREAM_TS_UPLOAD)
#define TSIF_EN_ACTIVE_HIGH 
#endif
	
#if defined(STREAM_SLAVE_PARALLEL_UPLOAD)	
#define	DMB_IRQ_TYPE 	       				GPIO_INT_36  /* DMB_IRQ_N */
#define 	DMB_RESET_TYPE					GPIO_INT_39  /* DMB_RESET_N  */
#define	DMB_EN_TYPE					GPIO_INT_41  /* DMB_EN */
#define	DMB_CS_TYPE					GPIO_INT_36  /*DMB_CS_N */
#define 	TDMB_RFBB_BASE_ADDR		 	EBI2_GP3_BASE 
#define	DMB_PATTER_EN					GPIO_INT_86 /* PATTERN_TDMB_ANT_EN  = GPIO_OUT(86,4) */ 
#define 	DMB_EAR_ANT_EN 					GPIO_INT_87 /* EAR_TDMB_ANT_EN		  = GPIO_OUT(87,4) */
#endif
	
#define	TDMB_RFBB_DEV_ADDR                    0x80   /*1000 0000 (7bit addr +r/w_n) */
#define	TDMB_RFBB_RW_RETRY                    3

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


/*============================================================
**    2.   External Variables
*============================================================*/

/*============================================================
**    3.   External Functions
*============================================================*/

/*============================================================
**    4.   Local constant variables
*============================================================*/

/*============================================================
**    5.   Local Typedef
*============================================================*/

/*============================================================
**    6.   Global Variables
*============================================================*/

/*============================================================
**    7.   Static Variables
*============================================================*/

/*============================================================
**    8.Function Prototype
*============================================================*/

extern	int8 tunerbb_drv_lg2102_power_on(void);
extern	int8 tunerbb_drv_lg2102_power_off(void);
extern	int8 tunerbb_drv_lg2102_init(void);
extern	int8 tunerbb_drv_lg2102_stop(void);
extern	int8 tunerbb_drv_lg2102_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info);
extern  int8 tunerbb_drv_lg2102_get_msc_ber(uint32 *msc_ber);
extern	int8 tunerbb_drv_lg2102_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode);
extern	int8 tunerbb_drv_lg2102_re_syncdetector(uint8 op_mode);
extern	int8 tunerbb_drv_lg2102_re_sync(void);
extern	int8 tunerbb_drv_lg2102_tune(int nFreqNo);
extern	int8 tunerbb_drv_lg2102_get_fic(uint8* buffer, uint32* buffer_size, boolean crc_onoff);
extern	int8 tunerbb_drv_lg2102_control_fic(uint8 enable);
extern	int8 tunerbb_drv_lg2102_read_data(uint8* buffer, uint32* buffer_size);
extern	int8 tunerbb_drv_lg2102_process_multi_data(uint8 subch_cnt,uint8* input_buf, uint32 input_size, uint32* read_size);
extern 	int8 tunerbb_drv_lg2102_get_multi_data(uint8 subch_cnt, uint8* buf_ptr, uint32 buf_size);
extern	void tunerbb_drv_lg2102_start_tii(void);
extern	boolean tunerbb_drv_lg2102_check_tii(uint8 * pmain_tii,uint8 * psub_tii);
extern	int8 tunerbb_drv_lg2102_reset_ch(void);
extern void tunerbb_drv_lg2102_set_userstop(void);
#endif	/* __TUNERBB_DRV_LG2102_H__ */

