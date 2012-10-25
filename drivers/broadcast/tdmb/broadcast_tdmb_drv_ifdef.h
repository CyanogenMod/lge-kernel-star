/*****************************************************************************

	Copyright(c) 2010 LG Electronics Inc. All Rights Reserved

	File name : broadcat_tdmb_if.h

	Description :

    Hoistory
	----------------------------------------------------------------------
	Oct. 28. 2010:		hyewon.eum		create

*******************************************************************************/
#ifndef _BROADCAST_TDMB_DRV_IF_
#define _BROADCAST_TDMB_DRV_IF_

#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>

#include "broadcast_tdmb_typedef.h"


#define LGE_BROADCAST_TDMB_IOCTL_MAGIC 'B'

#define LGE_BROADCAST_TDMB_IOCTL_ON \
	_IO(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 0)

#define LGE_BROADCAST_TDMB_IOCTL_OFF \
	_IO(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 1)

#define LGE_BROADCAST_TDMB_IOCTL_OPEN \
	_IO(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 2)

#define LGE_BROADCAST_TDMB_IOCTL_CLOSE \
	_IO(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 3)
	
#define LGE_BROADCAST_TDMB_IOCTL_TUNE \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 4, int*)

#define LGE_BROADCAST_TDMB_IOCTL_SET_CH \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 5, struct broadcast_tdmb_set_ch_info*)

#define LGE_BROADCAST_TDMB_IOCTL_RESYNC \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 6, int)

#define LGE_BROADCAST_TDMB_IOCTL_DETECT_SYNC \
	_IOR(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 7, int*)

#define LGE_BROADCAST_TDMB_IOCTL_GET_SIG_INFO \
	_IOR(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 8, struct broadcast_tdmb_sig_info*)

#define LGE_BROADCAST_TDMB_IOCTL_GET_CH_INFO \
	_IOR(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 9, struct broadcast_tdmb_ch_info*)

#define LGE_BROADCAST_TDMB_IOCTL_RESET_CH \
	_IO(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 10)

#define LGE_BROADCAST_TDMB_IOCTL_USER_STOP \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 11, int*)

#define LGE_BROADCAST_TDMB_IOCTL_GET_DMB_DATA \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 12, struct broadcast_tdmb_data_info*)

#define LGE_BROADCAST_TDMB_IOCTL_SELECT_ANTENNA \
	_IOW(LGE_BROADCAST_TDMB_IOCTL_MAGIC, 13, int*)

struct broadcast_tdmb_set_ch_info
{
	unsigned int	mode;
	unsigned int	ch_num;
	unsigned int	sub_ch_id;
};

struct broadcast_tdmb_sig_info
{
	unsigned int	dab_ok;
	unsigned int	msc_ber;
	unsigned int	sync_lock;
	unsigned int	afc_ok;
	unsigned int	cir;
	unsigned int	fic_ber;
	unsigned int	tp_lock;
	unsigned int	sch_ber;
	unsigned int	tp_err_cnt;
	unsigned int	va_ber;
	unsigned char	srv_state_flag;
	unsigned int	antenna_level;
};

struct broadcast_tdmb_ch_info
{
	unsigned char*  ch_buf;
	unsigned int	buf_len;
};

struct broadcast_tdmb_data_info
{
	unsigned char*	data_buf;
	unsigned int 	data_buf_size;
	unsigned int	copied_size;
	unsigned int	packet_cnt;
};

enum
{
	LGE_BROADCAST_TDMB_ANT_TYPE_INTENNA,
	LGE_BROADCAST_TDMB_ANT_TYPE_EARANT,
	LGE_BROADCAST_TDMB_ANT_TYPE_EXTERNAL
};

extern int broadcast_tdmb_drv_start(void);
extern int broadcast_tdmb_get_stop_mode(void);
extern boolean broadcast_drv_if_read_data(void);

extern int broadcast_drv_if_power_on(void);
extern int broadcast_drv_if_power_off(void);
extern int broadcast_drv_if_open(void);
extern int broadcast_drv_if_close(void);
extern int broadcast_drv_if_set_channel(unsigned int freq_num, unsigned int subch_id, unsigned int op_mode);
extern int broadcast_drv_if_resync(void);
extern int broadcast_drv_if_detect_sync(int op_mode);
extern int broadcast_drv_if_get_sig_info(struct broadcast_tdmb_sig_info *dmb_bb_info);
extern int broadcast_drv_if_get_ch_info(char* buffer, unsigned int* buffer_size);
extern int broadcast_drv_if_get_dmb_data(char** buffer_ptr, unsigned int* buffer_size, unsigned int user_buffer_size);
extern int broadcast_drv_if_reset_ch(void);
extern int broadcast_drv_if_user_stop(int mode);
extern int broadcast_drv_if_select_antenna(unsigned int sel);
extern int broadcast_drv_if_isr(void);
#endif
