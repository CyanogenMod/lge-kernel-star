#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>        /* copy_to_user */

#include <linux/broadcast/broadcast_tdmb_typedef.h>
#include <linux/broadcast/broadcast_tdmb.h>
#include <linux/broadcast/broadcast_lg2102_ioctrl.h>
#include <linux/broadcast/broadcast_lg2102.h>

#define TDMB_MPI_BUF_SIZE 			188*16 //LGD_INTERRUPT_SIZE
#define TDMB_MPI_BUF_CHUNK_NUM  	10
#define BROADCAST_TDMB_NUM_DEVS 	1 /**< support this many devices */

static struct class *broadcast_tdmb_class;
static dev_t broadcast_tdmb_dev;
struct broadcast_tdmb_chdevice 
{
	struct cdev cdev;
	struct device *dev;
	wait_queue_head_t wq_read;
	void *cookie;
};
//static struct cdev *tdmb_cdev_p;
static struct broadcast_tdmb_chdevice tdmb_dev;

/* user stop count */
//static int user_stop_mode_cnt = 0;
static uint8*			gpMPI_Buffer = NULL;
static uint8			gBBBuffer_ridx = 0;
static uint8			gBBBuffer_widx = 0;
static uint32			tdmb_real_read_size[TDMB_MPI_BUF_CHUNK_NUM];

boolean broadcast_tdmb_read_data(void)
{
	uint8* 	read_buffer_ptr 	= NULL;
	uint32 	read_buffer_size 	= 0;

	if(gpMPI_Buffer== NULL)
	{
		printk("gpMPI_FIFO_Buffer== NULL");
		return FALSE;
	}

	if(gBBBuffer_ridx == ((gBBBuffer_widx + 1)%TDMB_MPI_BUF_CHUNK_NUM))
	{	
		printk("======================================\n");
		printk("### buffer is full, skip the data (ridx=%d, widx=%d)  ###\n", gBBBuffer_ridx, gBBBuffer_widx);
		printk("======================================\n");
		return FALSE;
	}

	read_buffer_ptr = gpMPI_Buffer + gBBBuffer_widx*TDMB_MPI_BUF_SIZE;
	
	tunerbb_drv_lg2102_read_data(read_buffer_ptr, &read_buffer_size);
	tdmb_real_read_size[gBBBuffer_widx] = read_buffer_size;

	/* update write index */
	if ( 0 < read_buffer_size )
	{
		gBBBuffer_widx = ((gBBBuffer_widx + 1)%TDMB_MPI_BUF_CHUNK_NUM);
	}

	//printk("broadcast_tdmb_read_data, ridx=%d, widx=%d, wsize=%d\n",gBBBuffer_ridx, gBBBuffer_widx,  read_buffer_size);
	
	return TRUE;		
}

static int8 broadcast_tdmb_power_on(void)
{
	int8 res = ERROR;
	boolean retval = FALSE;
	
	retval = tunerbb_drv_lg2102_power_on();

	if(retval == TRUE)
	{
		res = OK;
	}
	tunerbb_drv_lg2102_set_userstop( );
	//user_stop_mode_cnt = 0;


	return res;
}

static int8 broadcast_tdmb_power_off(void)
{
	int8 res = ERROR;
	boolean retval = FALSE;
	
	retval = tunerbb_drv_lg2102_power_off();

	if(retval == TRUE)
	{
		res = OK;
	}
	tunerbb_drv_lg2102_set_userstop( );
	//user_stop_mode_cnt = 0;

	return res;
}

static int8 broadcast_tdmb_open(void)
{
	int8 res = ERROR;
	boolean retval = FALSE;
	
	retval = tunerbb_drv_lg2102_init();

	if(retval == TRUE)
	{
		res = OK;
	}

	return res;
}

static int8 broadcast_tdmb_close(void)
{
	int8 res = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_lg2102_stop();
	
	if(retval == TRUE)
	{
		res = OK;
	}

	return res;
}

static int8 broadcast_tdmb_tune(void __user *arg)
{
	int8 rc = ERROR;
	boolean retval = FALSE;
	int udata;
	
	if(copy_from_user(&udata, arg, sizeof(int)))
	{	
		//ERR_COPY_FROM_USER();
		printk("broadcast_tdmb_tune fail!!! udata = %d\n", udata);
		rc = ERROR;
	}
	else
	{
		broadcast_tdmb_close();
		retval = tunerbb_drv_lg2102_tune(udata);
		if(retval == TRUE)
		{
			rc = OK;
		}
	}

	return rc;

}

static int broadcast_tdmb_tune_set_ch(void __user *arg)
{
	int8 rc = ERROR;
	boolean retval = FALSE;
	struct broadcast_tdmb_set_ch_info udata;

	if(copy_from_user(&udata, arg, sizeof(struct broadcast_tdmb_set_ch_info)))
	{	
		//ERR_COPY_FROM_USER();
		printk("broadcast_tdmb_set_ch fail!!! \n");
		rc = ERROR;
	}
	else
	{
		printk("broadcast_tdmb_set_ch ch_num = %d, mode = %d, sub_ch_id = %d \n", udata.ch_num, udata.mode, udata.sub_ch_id);
		retval = tunerbb_drv_lg2102_set_channel(udata.ch_num, udata.sub_ch_id, udata.mode);
		if(retval == TRUE)
		{
			gBBBuffer_ridx = gBBBuffer_widx = 0;
			rc = OK;
		}
	}

	return rc;

}

static int broadcast_tdmb_resync(void __user *arg)
{
	#if 0
	int rc;
	int udata;
	
	copy_from_user(&udata, arg, sizeof(int));
	#endif
	return 0;

}

static int broadcast_tdmb_detect_sync(void __user *arg)
{
	int8 rc = ERROR;
	boolean retval = FALSE;
	int udata;
	int __user* puser = (int __user*)arg;
	udata = *puser;

	retval = tunerbb_drv_lg2102_re_syncdetector(udata);

	if(retval == TRUE)
	{
		rc = OK;
	}
	return rc;
#if 0	
	if(copy_from_user(&udata, arg, sizeof(int)))
	{	
		//ERR_COPY_FROM_USER();
		printk("broadcast_tdmb_tune fail!!! udata = %d\n", udata);
		rc = ERROR;
	}
	else
	{
		retval = tunerbb_drv_lg2102_re_syncdetector(udata);
		if(retval == TRUE)
		{
			rc = OK;
		}
	}
#endif
	return rc;
}


static int broadcast_tdmb_get_sig_info(void __user *arg)
{
	int rc = ERROR;
	boolean retval = FALSE;
	
	struct broadcast_tdmb_sig_info udata;
	
	if(copy_from_user(&udata, arg, sizeof(struct broadcast_tdmb_sig_info)))
	{
		printk("broadcast_tdmb_get_sig_info copy_from_user error!!! \n");
		rc = ERROR;
	}
	else
	{
		retval = tunerbb_drv_lg2102_get_ber(&udata);

		if(retval == TRUE)
		{
			rc = OK;
		}
	
		if(copy_to_user((void *)arg, &udata, sizeof(struct broadcast_tdmb_sig_info)))
		{
			printk("broadcast_tdmb_get_sig_info copy_to_user error!!! \n");
			rc = ERROR;
		}
		else
		{
			rc = OK;
		}
	}

	return rc;

}

static int broadcast_tdmb_get_ch_info(void __user *arg)
{
	int rc = ERROR;
	boolean retval = FALSE;
	uint8 fic_kernel_buffer[400];
	uint32 fic_len = 0;

	struct broadcast_tdmb_get_ch_info __user* puserdata = (struct broadcast_tdmb_get_ch_info __user*)arg;

	if((puserdata == NULL)||( puserdata->ch_buf == NULL))
	{
		printk("broadcast_tdmb_get_ch_info argument error\n");
		return rc;
	}

	memset(fic_kernel_buffer, 0x00, sizeof(fic_kernel_buffer));

	retval = tunerbb_drv_lg2102_get_fic(fic_kernel_buffer, &fic_len ,TRUE);
	
	if(retval == TRUE)
	{
		if(copy_to_user((void __user*)puserdata->ch_buf, (void*)fic_kernel_buffer, fic_len))
		{
			fic_len = 0;
			rc = ERROR;
		}
		else
		{
			rc = OK;
		}
	}
	else
	{
		fic_len = 0;
		rc = ERROR;
	}
	puserdata->buf_len = fic_len;
	
	return rc;
}


static int broadcast_tdmb_get_dmb_data(void __user *arg)
{
	int 		rc 					= ERROR;
	uint8* 	read_buffer_ptr 		= NULL;
	uint32 	read_buffer_size 	= 0;
	uint32 	read_packet_cnt 	= 0;
	uint32 	copied_buffer_size 		= 0;
	
	struct broadcast_tdmb_get_dmb_data_info __user* puserdata = (struct broadcast_tdmb_get_dmb_data_info  __user*)arg;
	
	if( (gpMPI_Buffer== NULL) || (puserdata==NULL) )
	{
		printk("gpMPI_FIFO_Buffer== NULL or puserdata==NULL\n");
		return rc;
	}

	if(gBBBuffer_ridx == gBBBuffer_widx)
	{
		//printk("broadcast_tdmb_get_dmb_data, data is not ready\n");
		return rc;
	}

	while(1)
	{
		read_buffer_ptr 	= gpMPI_Buffer + gBBBuffer_ridx*TDMB_MPI_BUF_SIZE;
		read_buffer_size 	= tdmb_real_read_size[gBBBuffer_ridx];

		if ( 188*16*2 < copied_buffer_size+read_buffer_size )
		{
			printk("broadcast_tdmb_get_dmb_data, output buffer is small\n");
			break;
		}

		if(copy_to_user((void __user*)(puserdata->data_buf + copied_buffer_size), (void*)read_buffer_ptr, read_buffer_size))
		{
			puserdata->buf_len = 0;
			puserdata->packet_cnt = 0;
			rc = ERROR;
		}
		else
		{
			copied_buffer_size += read_buffer_size;
			puserdata->buf_len = copied_buffer_size;

			read_packet_cnt = copied_buffer_size/188;
			puserdata->packet_cnt = read_packet_cnt;
			rc = OK;
		}

		/* update read index */
		gBBBuffer_ridx = ((gBBBuffer_ridx + 1)%TDMB_MPI_BUF_CHUNK_NUM);
		if ( (gBBBuffer_ridx == gBBBuffer_widx) || (rc != OK))
		{
			//printk("broadcast_tdmb_get_dmb_data ridx=%d, widx=%d, size=%d \n", gBBBuffer_ridx, gBBBuffer_widx, copied_buffer_size);
			break;
		}
	}

	return OK;
}

static int8 broadcast_tdmb_reset_ch(void)
{
	int8 res = ERROR;
	boolean retval = FALSE;
	
	retval = tunerbb_drv_lg2102_reset_ch();

	if(retval == TRUE)
	{
		res = OK;
	}

	return res;
}

static int8 broadcast_tdmb_user_stop(void __user *arg)
{
	int8 rc = ERROR;
	//boolean retval = FALSE;
	int udata;
	int __user* puser = (int __user*)arg;

	udata = *puser;

#if 1
	printk("broadcast_tdmb_user_stop data =(%d) (IN)\n", udata);
	tunerbb_drv_lg2102_set_userstop( );
	rc = OK;
#else
	printk("broadcast_tdmb_user_stop - mode udata = %d\n", udata);
	//user_stop_mode_cnt = udata;
	if(udata == 1)
	{
		user_stop_mode_cnt++;
	}
	else
	{
		if(user_stop_mode_cnt > 0)
		{
			user_stop_mode_cnt--;
		}
		else
		{
			user_stop_mode_cnt = 0;
		}
	}
	rc = OK;
#endif	
	return rc;	
}

static int8 broadcast_tdmb_select_antenna(void __user *arg)
{
	int8 rc = ERROR;
	int udata;
	int __user* pmode = (int __user*)arg;

	udata = *pmode;

	printk("broadcast_tdmb_select_antenna mode =(%d) (IN)\n", udata);
	tdmb_lg2102_select_antenna( udata );
	rc = OK;
	return rc;	
}


static ssize_t broadcast_tdmb_open_control(struct inode *inode, struct file *file)
{
	struct broadcast_tdmb_chdevice *the_dev =
	       container_of(inode->i_cdev, struct broadcast_tdmb_chdevice, cdev);

	printk("broadcast_tdmb_open_control start \n");
	 
	file->private_data = the_dev;
	
	printk("broadcast_tdmb_open_control OK \n");
	
	return nonseekable_open(inode, file);
}

static long broadcast_tdmb_ioctl_control(struct file *filep, unsigned int cmd,	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	//struct broadcast_tdmb_sig_info udata;
	//copy_from_user(&udata, argp, sizeof(struct broadcast_tdmb_sig_info));
	
	switch (cmd) 
	{
	case LGE_BROADCAST_TDMB_IOCTL_ON:
		rc = broadcast_tdmb_power_on();
		printk("LGE_BROADCAST_TDMB_IOCTL_ON OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_OFF:		
		rc = broadcast_tdmb_power_off();
		printk("LGE_BROADCAST_TDMB_IOCTL_OFF OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_OPEN:
		rc = broadcast_tdmb_open();
		printk("LGE_BROADCAST_TDMB_IOCTL_OPEN OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_CLOSE:
		broadcast_tdmb_close();
		printk("LGE_BROADCAST_TDMB_IOCTL_CLOSE OK \n");
		rc = 0;
		break;
	case LGE_BROADCAST_TDMB_IOCTL_TUNE:
		rc = broadcast_tdmb_tune(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_TUNE result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_SET_CH:
		rc = broadcast_tdmb_tune_set_ch(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_SET_CH result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_RESYNC:
		rc = broadcast_tdmb_resync(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_RESYNC result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_DETECT_SYNC:
		rc = broadcast_tdmb_detect_sync(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_DETECT_SYNC result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_GET_SIG_INFO:
		rc = broadcast_tdmb_get_sig_info(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_SIG_INFO result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_GET_CH_INFO:
		rc = broadcast_tdmb_get_ch_info(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_CH_INFO result = %d \n", rc);
		break;

	case LGE_BROADCAST_TDMB_IOCTL_RESET_CH:
		rc = broadcast_tdmb_reset_ch();
		printk("LGE_BROADCAST_TDMB_IOCTL_RESET_CH result = %d \n", rc);
		break;
		
	case LGE_BROADCAST_TDMB_IOCTL_USER_STOP:
		rc = broadcast_tdmb_user_stop(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_USER_STOP !!! \n");
		break;

	case LGE_BROADCAST_TDMB_IOCTL_GET_DMB_DATA:
		rc = broadcast_tdmb_get_dmb_data(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_DMB_DATA TBD... !!! \n");
		break;

	case LGE_BROADCAST_TDMB_IOCTL_SELECT_ANTENNA:
		rc = broadcast_tdmb_select_antenna(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_SELECT_ANTENNA !!! \n");
		break;
		
	default:
		printk("broadcast_tdmb_ioctl_control OK \n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

static ssize_t broadcast_tdmb_release_control(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations broadcast_tdmb_fops_control = 
{
	.owner = THIS_MODULE,
	.open = broadcast_tdmb_open_control,
	.unlocked_ioctl = broadcast_tdmb_ioctl_control,
	.release = broadcast_tdmb_release_control,
};

static int broadcast_tdmb_device_init(struct broadcast_tdmb_chdevice *pbroadcast, int index)
{
	int rc;

	gpMPI_Buffer = kmalloc(TDMB_MPI_BUF_SIZE*TDMB_MPI_BUF_CHUNK_NUM, GFP_KERNEL);

	cdev_init(&pbroadcast->cdev, &broadcast_tdmb_fops_control);

	pbroadcast->cdev.owner = THIS_MODULE;
	//init_waitqueue_head(&pbroadcast->wq_read);

	rc = cdev_add(&pbroadcast->cdev, broadcast_tdmb_dev, 1);

	pbroadcast->dev = device_create(broadcast_tdmb_class, NULL, MKDEV(MAJOR(broadcast_tdmb_dev), 0),
					 NULL, "broadcast%d", index);

	printk("broadcast_tdmb_device_add add add%d broadcast_tdmb_dev = %d \n", rc, MKDEV(MAJOR(broadcast_tdmb_dev), 0));

	
	if (IS_ERR(pbroadcast->dev)) {
		rc = PTR_ERR(pbroadcast->dev);
		pr_err("device_create failed: %d\n", rc);
		rc = -1;
	}
	
	printk("broadcast_tdmb_device_init start %d\n", rc);

	return rc;
}


int broadcast_tdmb_drv_start(void)
{
	struct broadcast_tdmb_chdevice *pbroadcast = NULL;
	int rc = -ENODEV;
	
	if (!broadcast_tdmb_class) {

		broadcast_tdmb_class = class_create(THIS_MODULE, "broadcast_tdmb");
		if (IS_ERR(broadcast_tdmb_class)) {
			rc = PTR_ERR(broadcast_tdmb_class);
			pr_err("broadcast_tdmb_class: create device class failed: %d\n",
				rc);
			return rc;
		}

		rc = alloc_chrdev_region(&broadcast_tdmb_dev, 0, BROADCAST_TDMB_NUM_DEVS, "broadcast_tdmb");
		printk("broadcast_tdmb_drv_start add add%d broadcast_tdmb_dev = %d \n", rc, broadcast_tdmb_dev);
		if (rc < 0) {
			pr_err("broadcast_class: failed to allocate chrdev: %d\n",
				rc);
			return rc;
		}
	}

	//pbroadcast = kzalloc(sizeof(struct broadcast_tdmb_chdevice), GFP_ATOMIC);
	pbroadcast = &tdmb_dev;
	
	//if (!pbroadcast)
	//	return -ENOMEM;

	rc = broadcast_tdmb_device_init(pbroadcast, 0);
	if (rc < 0) {
		//kfree(pbroadcast);
		return rc;
	}
	
	printk("broadcast_tdmb_drv_start start %d\n", rc);

	return rc;
}

EXPORT_SYMBOL(broadcast_tdmb_drv_start);

int broadcast_tdmb_get_stop_mode(void)
{
	return 0;
#if 0
	if(user_stop_mode_cnt > 0 )
	{
		printk("broadcast_tdmb_get_stop_mode cnt = %d\n", user_stop_mode_cnt);
		return 1;
	}
	else
	{
		return 0;
	}
#endif	
}

EXPORT_SYMBOL(broadcast_tdmb_get_stop_mode);


