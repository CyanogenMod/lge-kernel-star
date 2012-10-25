#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>

#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/tdmb_tunerbbdrv_t3900def.h"

static int g_ch_setting_done = ERROR;

#define T3900_USES_STATIC_BUFFER

//#define TDMB_MPI_BUF_SIZE 			((188*16*2) + 8) //LGD_INTERRUPT_SIZE + sizeof(TDMB_BB_HEADER_TYPE)
#define TDMB_MPI_BUF_SIZE 			(8120)   /* 8kbyte + sizeof(TDMB_BB_HEADER_TYPE) + dummy */
#define TDMB_MPI_BUF_CHUNK_NUM  	10
#define BROADCAST_TDMB_NUM_DEVS 	1 /**< support this many devices */

static uint8*			gpMPI_Buffer = NULL;
static uint8			gBBBuffer_ridx = 0;
static uint8			gBBBuffer_widx = 0;
static uint32			tdmb_real_read_size[TDMB_MPI_BUF_CHUNK_NUM];

#ifdef T3900_USES_STATIC_BUFFER
static uint8	gpMPI_Array[TDMB_MPI_BUF_SIZE*TDMB_MPI_BUF_CHUNK_NUM];
#endif

boolean broadcast_drv_if_read_data(void)
{
	uint8* 	read_buffer_ptr 	= NULL;
	uint32 	read_buffer_size 	= 0;

	if(gpMPI_Buffer== NULL)
	{
		printk("gpMPI_FIFO_Buffer== NULL\n");
		return FALSE;
	}

	read_buffer_ptr = gpMPI_Buffer + gBBBuffer_widx*TDMB_MPI_BUF_SIZE;
	tunerbb_drv_t3900_read_data(read_buffer_ptr, &read_buffer_size);

	if(gBBBuffer_ridx == ((gBBBuffer_widx + 1)%TDMB_MPI_BUF_CHUNK_NUM))
	{	
		//printk("======================================\n");
		printk("### buffer is full, skip the data (ridx=%d, widx=%d)  ###\n", gBBBuffer_ridx, gBBBuffer_widx);
		//printk("======================================\n");
		return FALSE;
	}

	if(read_buffer_size > 0)
	{
		tdmb_real_read_size[gBBBuffer_widx] = read_buffer_size;
		gBBBuffer_widx = ((gBBBuffer_widx + 1)%TDMB_MPI_BUF_CHUNK_NUM);
		return TRUE;
	}

	//printk("broadcast_drv_if_read_data, ridx=%d, widx=%d, wsize=%d\n",gBBBuffer_ridx, gBBBuffer_widx,  read_buffer_size);
	
	return FALSE;
}

int broadcast_drv_if_power_on(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	if(gpMPI_Buffer == NULL)
	{
#ifndef T3900_USES_STATIC_BUFFER
		gpMPI_Buffer = kmalloc(TDMB_MPI_BUF_SIZE*TDMB_MPI_BUF_CHUNK_NUM, GFP_KERNEL);
#else
		gpMPI_Buffer = (uint8*)&gpMPI_Array[0];
#endif
	}

	retval = tunerbb_drv_t3900_power_on( );

	if(retval == TRUE)
	{
		rc = OK;
	}
	//tunerbb_drv_t3900_set_userstop( 1 );

	return rc;
}


int broadcast_drv_if_power_off(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_t3900_power_off( );

	if(retval == TRUE)
	{
		rc = OK;
	}
	//tunerbb_drv_t3900_set_userstop( 0 );

	return rc;
}


int broadcast_drv_if_open(void) 
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	printk("broadcast_drv_if_open\n");
	retval = tunerbb_drv_t3900_init( );

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;	
}


int broadcast_drv_if_close(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_t3900_stop( );

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;
}


int broadcast_drv_if_set_channel(unsigned int freq_num, unsigned int subch_id, unsigned int op_mode)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	printk("broadcast_drv_if_set_channel IN( )\n");
	gBBBuffer_ridx = gBBBuffer_widx = 0;
	retval = tunerbb_drv_t3900_set_channel(freq_num, subch_id, op_mode);
	printk("broadcast_drv_if_set_channel OUT( ) result = (%d)\n", retval);
	if(retval == TRUE)
	{
		rc = OK;
	}

	g_ch_setting_done = rc;
	return rc;	
}

int broadcast_drv_if_resync(void)
{
	return ERROR;
}

int broadcast_drv_if_detect_sync(int op_mode)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	if(g_ch_setting_done == OK)
	{
		//printk("broadcast_drv_if_detect_sync. channel_set_ok = (%d)\n", g_ch_setting_done);
		return OK;
	}
	
	retval = tunerbb_drv_t3900_re_syncdetector(op_mode);

	if(retval == TRUE)
	{
		rc = OK;
	}

	g_ch_setting_done = rc;

	return rc;
}

int broadcast_drv_if_get_sig_info(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_t3900_get_ber(dmb_bb_info);

	if(retval == TRUE)
	{
		rc = OK;
	}

	if(g_ch_setting_done == ERROR)
	{
		dmb_bb_info->cir = 0;
		dmb_bb_info->msc_ber = 20000;
	}
	else
	{
		dmb_bb_info->cir = 1;
	}
	//printk("broadcast_drv_if_get_sig_info ber = (%d)\n", dmb_bb_info->msc_ber);
	return rc;
}


int broadcast_drv_if_get_ch_info(char* buffer, unsigned int* buffer_size)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	if(buffer == NULL || buffer_size == NULL)
	{
		printk("broadcast_drv_if_get_ch_info argument error\n");
		return rc;
	}

	retval = tunerbb_drv_t3900_get_fic(buffer, buffer_size);

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;	
}


int broadcast_drv_if_get_dmb_data(char** buffer_ptr, unsigned int* buffer_size, unsigned int user_buffer_size)
{
	if(gpMPI_Buffer == NULL)
	{
		printk("get_dmb_data : gpMPI_FIFO_Buffer == NULL\n");
		return ERROR;
	}

	if(buffer_ptr == NULL || buffer_size == NULL)
	{
		printk(" input arg is null\n");
		return ERROR;
	}

	if(gBBBuffer_ridx == gBBBuffer_widx)
	{
		//printk("broadcast_tdmb_get_dmb_data, data is not ready\n");
		return ERROR;
	}

	if(user_buffer_size < tdmb_real_read_size[gBBBuffer_ridx])
	{
		printk("user buffer is not enough %d", user_buffer_size);
		return ERROR;
	}

	*buffer_ptr	= gpMPI_Buffer + gBBBuffer_ridx * TDMB_MPI_BUF_SIZE;
	*buffer_size = tdmb_real_read_size[gBBBuffer_ridx];

	//printk("broadcast_tdmb_get_dmb_data, read_size %d, total ridx %d, widx %d\n", *buffer_size, gBBBuffer_ridx, gBBBuffer_widx);

	gBBBuffer_ridx = ((gBBBuffer_ridx + 1) % TDMB_MPI_BUF_CHUNK_NUM);

	return OK;
}

int broadcast_drv_if_reset_ch(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_t3900_reset_ch( );

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;
}

int broadcast_drv_if_user_stop(int mode)
{
	tunerbb_drv_t3900_set_userstop( mode );
	return OK ;
}

int broadcast_drv_if_select_antenna(unsigned int sel)
{
	tunerbb_drv_t3900_select_antenna(sel);
	return OK;
}

int broadcast_drv_if_isr(void)
{
	return ERROR;
}

