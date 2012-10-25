#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "raontv.h"
#include "raontv_internal.h"

#include "mtv250.h"
#include "mtv250_spi.h"
#include "mtv250_ioctl.h"


#ifdef RTV_IF_SPI
//struct spi_device *dmb_spi_ptr;

void mtv250_spi_read_burst(unsigned char reg, unsigned char *buf, int size)
{
	int ret;
	u8 out_buf[2], read_out_buf[2];
	struct spi_message msg;
	struct spi_transfer msg_xfer0 = {
		.tx_buf = out_buf,
		.len		= 2,
		.cs_change	= 1,
		.delay_usecs = 0,
	};
	
	struct spi_transfer msg_xfer1 = {
		.tx_buf = read_out_buf,
		.rx_buf = buf,
		.len		= size,
		.cs_change	= 0,
		.delay_usecs = 0,
	};

	spi_message_init(&msg);	
	out_buf[0] = RAONTV_CHIP_ADDR;
	out_buf[1] = reg;	
	spi_message_add_tail(&msg_xfer0, &msg);

	ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv250_spi_read_burst]	spi_sync() error: %d\n", ret);	
	}

	spi_message_init(&msg);
	read_out_buf[0] = RAONTV_CHIP_ADDR|0x1;
	spi_message_add_tail(&msg_xfer1, &msg);

	ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv250_spi_read_burst] spi_sync() 1 error: %d\n", ret);	
	}
}

unsigned char mtv250_spi_read(unsigned char reg)
{
	int ret;
	u8 out_buf[2], read_out_buf[2];
	u8 in_buf[2];
	struct spi_message msg;
	struct spi_transfer msg_xfer0 = {
		.tx_buf = out_buf,
		.len		= 2,
		.cs_change	= 1,
		.delay_usecs = 0,
	};
	
	struct spi_transfer msg_xfer1 = {
		.tx_buf = read_out_buf,
		.rx_buf = in_buf,
		.len		= 2,
		.cs_change	= 0,
		.delay_usecs = 0,
	};

	spi_message_init(&msg);	
	out_buf[0] = RAONTV_CHIP_ADDR;
	out_buf[1] = reg;	
	spi_message_add_tail(&msg_xfer0, &msg);

	ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv250_spi_read]  spi_sync() error: %d\n", ret);  
		return 0xFF;
	}

	spi_message_init(&msg);
	read_out_buf[0] = RAONTV_CHIP_ADDR|0x1;
	spi_message_add_tail(&msg_xfer1, &msg);

	ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv250_spi_read]  spi_sync() 1 error: %d\n", ret);  
		return 0xFF;
	}

	return (in_buf[1]);
}


void mtv250_spi_write(unsigned char reg, unsigned char val)
{
	u8 out_buf[3];
	u8 in_buf[3];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.len		= 3,
		.cs_change	= 0,
		.delay_usecs = 0,
	};
	int ret;

	spi_message_init(&msg);

	out_buf[0] = RAONTV_CHIP_ADDR;
	out_buf[1] = reg;
	out_buf[2] = val;

	msg_xfer.tx_buf = out_buf;
	msg_xfer.rx_buf = in_buf;
	spi_message_add_tail(&msg_xfer, &msg);

#ifndef RTV_DUAL_CHIP_USED
	ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv250_spi_write]  mtv250_spi_write() error: %d\n", ret);  
	}
#else
	if(RaonTvChipIdx == 0) /* Master MTV */
	{
		ret = spi_sync(mtv250_cb_ptr->spi_ptr, &msg);
		if (ret)
		{
			DMBERR("[mtv250_spi_write] MASTER mtv250_spi_write() error: %d\n", ret);	
		}
	}
	else
	{
		ret = spi_sync(mtv250_cb_ptr->spi_slave_ptr, &msg);
		if (ret)
		{
			DMBERR("[mtv250_spi_write] SLAVE mtv250_spi_write() error: %d\n", ret);	
		}
	}
#endif	
}


static int mtv250_spi_probe(struct spi_device *spi)
{
	int ret;
	
	DMBMSG("[mtv250_spi_probe] ENTERED!!!!!!!!!!!!!!\n");
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	
	ret = spi_setup(spi);
	if (ret < 0)
	       return ret;

	mtv250_cb_ptr->spi_ptr = spi;

	return 0;
}


static int mtv250_spi_remove(struct spi_device *spi)
{
	return 0;
}




static struct spi_driver mtv250_spi_driver = {
	.driver = {
		.name		= RAONTV_DEV_NAME,
		.owner		= THIS_MODULE,
	},

	.probe    = mtv250_spi_probe,
	.remove	= __devexit_p(mtv250_spi_remove),
};


int mtv250_spi_init(void)
{
	int ret;

	if((ret=spi_register_driver(&mtv250_spi_driver)) != 0)
	{
		DMBERR("[mtv250_spi_init] Master register error\n");
		return ret;
	}

#ifdef RTV_DUAL_CHIP_USED
	if((ret=spi_register_driver(&mtv250_slave_spi_driver)) != 0)
	{
		DMBERR("[mtv250_spi_init] Slave register error\n");
		return ret;
	}
#endif

	return ret;
}

void mtv250_spi_exit(void)
{
	spi_unregister_driver(&mtv250_spi_driver);

#ifdef RTV_DUAL_CHIP_USED
	spi_unregister_driver(&mtv250_slave_spi_driver);
#endif
}


#endif /* #ifdef RTV_IF_SPI */

