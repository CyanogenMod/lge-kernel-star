
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "raontv.h"
#include "raontv_internal.h"

#include "mtv818.h"
#include "mtv818_i2c.h"
#include "mtv818_ioctl.h"


#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)

//struct i2c_client *dmb_i2c_client_ptr;
//struct i2c_adapter *dmb_i2c_adapter_ptr;


static const struct i2c_device_id mtv818_i2c_device_id[] = {
	{RAONTV_DEV_NAME, 0},
	{},
};


MODULE_DEVICE_TABLE(i2c, mtv818_i2c_device_id);


static int mtv818_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	DMBMSG("[mtv818_i2c_probe] ENTERED!!!!!!!!!!!!!!\n");

	mtv250_cb_ptr->i2c_client_ptr = client;
	mtv250_cb_ptr->i2c_adapter_ptr = to_i2c_adapter(client->dev.parent); 

/*
#ifdef RTV_DUAL_CHIP_USED
	RTV_MASTER_CHIP_SEL;
#endif
	rtvOEM_PowerOn(1);
	DMBMSG("Check MTV818 0x00 = 0x%02x \n", RTV_REG_GET(0x00));
	rtvOEM_PowerOn(0);

#ifdef RTV_DUAL_CHIP_USED
	RTV_SLAVE_CHIP_SEL;
	rtvOEM_PowerOn(1);
	DMBMSG("SLAVE MTV818 0x00 = 0x%02x \n", RTV_REG_GET(0x00));
	rtvOEM_PowerOn(0);			
#endif
*/

	return 0;
}


static int mtv818_i2c_remove(struct i2c_client *client)
{
    int ret = 0;

    return ret;
}

void mtv818_i2c_read_burst(unsigned char reg, unsigned char *buf, int size)
{
	int ret;	
	u8 out_buf[2];

	struct i2c_msg msg[] = {  
		 {.addr = RAONTV_CHIP_ADDR>>1, .flags = 0, .buf = out_buf, .len = 1},	
		 {.addr = RAONTV_CHIP_ADDR>>1, .flags = I2C_M_RD, .buf = buf, .len = size} 
	};	

	out_buf[0] = reg;
	out_buf[1] = 0;

#ifndef RTV_DUAL_CHIP_USED
	ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
	if (ret != 2) {  
		 DMBMSG("[mtv818_i2c_read_burst]	i2c_transfer() error: %d\n", ret);	
	}  

#else
	if(RaonTvChipIdx == 0) /* Master MTV */
	{
		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
		if (ret != 2) {  
			 DMBMSG("[mtv818_i2c_read_burst]	i2c_transfer() error: %d\n", ret);	
		}  
	}
	else
	{
		msg[0].addr = 0xD0 >> 1;
		msg[1].addr = 0xD0 >> 1;

		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
		if (ret != 2) {  
			 DMBMSG("[mtv818_i2c_read_burst]	SLAVE i2c_transfer() error: %d\n", ret);	
		}  
	}
#endif
}

unsigned char mtv818_i2c_read(unsigned char reg)
{
	int ret;	
	u8 out_buf[2];
	u8 in_buf[2]; 

	struct i2c_msg msg[] = {  
	     {.addr = RAONTV_CHIP_ADDR>>1, .flags = 0, .buf = out_buf, .len = 1},  
	     {.addr = RAONTV_CHIP_ADDR>>1, .flags = I2C_M_RD, .buf = in_buf, .len = 1}  
	};  

	out_buf[0] = reg;
	out_buf[1] = 0;

#ifndef RTV_DUAL_CHIP_USED
	ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
	if (ret != 2) {  
	     DMBMSG("[mtv818_i2c_read]  i2c_transfer() error: %d\n", ret);  
	     return 0x00;
	}  

#else
	if(RaonTvChipIdx == 0) /* Master MTV */
	{
		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
		if (ret != 2) {  
			 DMBMSG("[mtv818_i2c_read_burst]	i2c_transfer() error: %d\n", ret);	
			 return 0xFF;
		}  
	}
	else
	{
		msg[0].addr = 0xD0 >> 1;
		msg[1].addr = 0xD0 >> 1;

		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, msg, 2);
		if (ret != 2) {  
			 DMBMSG("[mtv818_i2c_read_burst]	SLAVE i2c_transfer() error: %d\n", ret);	
			 return 0xFF;
		}  
	}
#endif	
	
	return in_buf[0];
}



void mtv818_i2c_write(unsigned char reg, unsigned char val)
{
	int ret;
	u8 out_buf[2];
	struct i2c_msg msg = {.addr = RAONTV_CHIP_ADDR>>1, .flags = 0, .buf = out_buf, .len = 2}; 
	
	out_buf[0] = reg;
	out_buf[1] = val;

#ifndef RTV_DUAL_CHIP_USED
	ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, &msg, 1);
	if (ret != 1) {  
	     DMBMSG("[mtv818_i2c_write]  i2c_transfer() error: %d\n", ret);  
	}  	
#else
	if(RaonTvChipIdx == 0) /* Master MTV */
	{
		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, &msg, 1);
		if (ret != 1) {  
			 DMBMSG("[mtv818_i2c_write]	i2c_transfer() error: %d\n", ret);	
		}  
	}
	else
	{
		msg.addr = 0xD0 >> 1;

		ret = i2c_transfer(mtv250_cb_ptr->i2c_adapter_ptr, &msg, 1);
		if (ret != 1) {  
			 DMBMSG("[mtv818_i2c_write]	SLAVE i2c_transfer() error: %d\n", ret);	
		}  
	}
#endif	
}


static int mtv818_i2c_resume(struct i2c_client *client)
{	
	switch( mtv250_cb_ptr->tv_mode )
	{
#ifdef RTV_TDMB_ENABLE	
		case DMB_TV_MODE_TDMB: /* T-DMB */
			rtvTDMB_StandbyMode(0);
			break;
#endif			

#ifdef RTV_FM_ENABLE			
		case DMB_TV_MODE_FM: /* FM */
			rtvFM_StandbyMode(0);
			break;
#endif			

		default:
			return -ENOTTY;			
	}

	return 0;
}


static int mtv818_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	switch( mtv250_cb_ptr->tv_mode )
	{
#ifdef RTV_TDMB_ENABLE	
		case DMB_TV_MODE_TDMB: /* T-DMB */
			rtvTDMB_StandbyMode(1);
			break;
#endif			

#ifdef RTV_FM_ENABLE			
		case DMB_TV_MODE_FM: /* FM */
			rtvFM_StandbyMode(1);
			break;
#endif			

		default:
			return -ENOTTY;			
	}

	return 0;
}


static struct i2c_driver mtv818_i2c_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= RAONTV_DEV_NAME,
	},
	.probe		= mtv818_i2c_probe,
	.remove		= __devexit_p(mtv818_i2c_remove),
	.suspend        = mtv818_i2c_suspend,
	.resume         = mtv818_i2c_resume,
	.id_table	       = mtv818_i2c_device_id,
};

int mtv818_i2c_init(void)
{	
	int ret;
	
	if((ret=i2c_add_driver(&mtv818_i2c_drv)) != 0)
	{
		DMBMSG("[mtv818_i2c_init] register error\n");
		return ret;
	}

	return ret;
}


void mtv818_i2c_exit(void)
{
	i2c_del_driver(&mtv818_i2c_drv);
}

#endif // #if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)

