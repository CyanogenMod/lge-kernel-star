/*
 * arch/arm/mach-tegra/tegra_spi_slave.c
 *
 * Tegra slave spi driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>

#include <mach/dma.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <mach/spi.h>
#include <mach/clk.h>

#define SLINK_COMMAND		0x000
#define   SLINK_BIT_LENGTH(x)		(((x) & 0x1f) << 0)
#define   SLINK_WORD_SIZE(x)		(((x) & 0x1f) << 5)
#define   SLINK_BOTH_EN			(1 << 10)
#define   SLINK_CS_SW			(1 << 11)
#define   SLINK_CS_VALUE		(1 << 12)
#define   SLINK_CS_POLARITY		(1 << 13)
#define   SLINK_IDLE_SDA_DRIVE_LOW	(0 << 16)
#define   SLINK_IDLE_SDA_DRIVE_HIGH	(1 << 16)
#define   SLINK_IDLE_SDA_PULL_LOW	(2 << 16)
#define   SLINK_IDLE_SDA_PULL_HIGH	(3 << 16)
#define   SLINK_IDLE_SDA_MASK		(3 << 16)
#define   SLINK_CS_POLARITY1		(1 << 20)
#define   SLINK_CK_SDA			(1 << 21)
#define   SLINK_CS_POLARITY2		(1 << 22)
#define   SLINK_CS_POLARITY3		(1 << 23)
#define   SLINK_IDLE_SCLK_DRIVE_LOW	(0 << 24)
#define   SLINK_IDLE_SCLK_DRIVE_HIGH	(1 << 24)
#define   SLINK_IDLE_SCLK_PULL_LOW	(2 << 24)
#define   SLINK_IDLE_SCLK_PULL_HIGH	(3 << 24)
#define   SLINK_IDLE_SCLK_MASK		(3 << 24)
#define   SLINK_M_S			(1 << 28)
#define   SLINK_WAIT			(1 << 29)
#define   SLINK_GO			(1 << 30)
#define   SLINK_ENB			(1 << 31)

#define SLINK_COMMAND2		0x004
#define   SLINK_LSBFE			(1 << 0)
#define   SLINK_SSOE			(1 << 1)
#define   SLINK_SPIE			(1 << 4)
#define   SLINK_BIDIROE			(1 << 6)
#define   SLINK_MODFEN			(1 << 7)
#define   SLINK_INT_SIZE(x)		(((x) & 0x1f) << 8)
#define   SLINK_CS_ACTIVE_BETWEEN	(1 << 17)
#define   SLINK_SS_EN_CS(x)		(((x) & 0x3) << 18)
#define   SLINK_SS_SETUP(x)		(((x) & 0x3) << 20)
#define   SLINK_FIFO_REFILLS_0		(0 << 22)
#define   SLINK_FIFO_REFILLS_1		(1 << 22)
#define   SLINK_FIFO_REFILLS_2		(2 << 22)
#define   SLINK_FIFO_REFILLS_3		(3 << 22)
#define   SLINK_FIFO_REFILLS_MASK	(3 << 22)
#define   SLINK_WAIT_PACK_INT(x)	(((x) & 0x7) << 26)
#define   SLINK_SPC0			(1 << 29)
#define   SLINK_TXEN			(1 << 30)
#define   SLINK_RXEN			(1 << 31)

#define SLINK_STATUS		0x008
#define   SLINK_COUNT(val)		(((val) >> 0) & 0x1f)
#define   SLINK_WORD(val)		(((val) >> 5) & 0x1f)
#define   SLINK_BLK_CNT(val)		(((val) >> 0) & 0xffff)
#define   SLINK_MODF			(1 << 16)
#define   SLINK_RX_UNF			(1 << 18)
#define   SLINK_TX_OVF			(1 << 19)
#define   SLINK_TX_FULL			(1 << 20)
#define   SLINK_TX_EMPTY		(1 << 21)
#define   SLINK_RX_FULL			(1 << 22)
#define   SLINK_RX_EMPTY		(1 << 23)
#define   SLINK_TX_UNF			(1 << 24)
#define   SLINK_RX_OVF			(1 << 25)
#define   SLINK_TX_FLUSH		(1 << 26)
#define   SLINK_RX_FLUSH		(1 << 27)
#define   SLINK_SCLK			(1 << 28)
#define   SLINK_ERR			(1 << 29)
#define   SLINK_RDY			(1 << 30)
#define   SLINK_BSY			(1 << 31)

#define SLINK_MAS_DATA		0x010
#define SLINK_SLAVE_DATA	0x014

#define SLINK_DMA_CTL		0x018
#define   SLINK_DMA_BLOCK_SIZE(x)	(((x) & 0xffff) << 0)
#define   SLINK_TX_TRIG_1		(0 << 16)
#define   SLINK_TX_TRIG_4		(1 << 16)
#define   SLINK_TX_TRIG_8		(2 << 16)
#define   SLINK_TX_TRIG_16		(3 << 16)
#define   SLINK_TX_TRIG_MASK		(3 << 16)
#define   SLINK_RX_TRIG_1		(0 << 18)
#define   SLINK_RX_TRIG_4		(1 << 18)
#define   SLINK_RX_TRIG_8		(2 << 18)
#define   SLINK_RX_TRIG_16		(3 << 18)
#define   SLINK_RX_TRIG_MASK		(3 << 18)
#define   SLINK_PACKED			(1 << 20)
#define   SLINK_PACK_SIZE_4		(0 << 21)
#define   SLINK_PACK_SIZE_8		(1 << 21)
#define   SLINK_PACK_SIZE_16		(2 << 21)
#define   SLINK_PACK_SIZE_32		(3 << 21)
#define   SLINK_PACK_SIZE_MASK		(3 << 21)
#define   SLINK_IE_TXC			(1 << 26)
#define   SLINK_IE_RXC			(1 << 27)
#define   SLINK_DMA_EN			(1 << 31)

#define SLINK_STATUS2		0x01c
#define   SLINK_TX_FIFO_EMPTY_COUNT(val)	(((val) & 0x3f) >> 0)
#define   SLINK_RX_FIFO_FULL_COUNT(val)		(((val) & 0x3f) >> 16)

#define SLINK_TX_FIFO		0x100
#define SLINK_RX_FIFO		0x180
#define SLINK_FIFO_DEPTH	0x20

static const unsigned long spi_tegra_req_sels[] = {
	TEGRA_DMA_REQ_SEL_SL2B1,
	TEGRA_DMA_REQ_SEL_SL2B2,
	TEGRA_DMA_REQ_SEL_SL2B3,
	TEGRA_DMA_REQ_SEL_SL2B4,
};

#define BB_LEN			(16384)
#define TX_FIFO_EMPTY_COUNT_MAX		SLINK_TX_FIFO_EMPTY_COUNT(0x20)
#define RX_FIFO_FULL_COUNT_ZERO		SLINK_RX_FIFO_FULL_COUNT(0)

#define SLINK_STATUS2_RESET \
	(TX_FIFO_EMPTY_COUNT_MAX | \
	RX_FIFO_FULL_COUNT_ZERO << 16)

struct spi_tegra_data {
	struct spi_master	*master;
	struct platform_device	*pdev;
	spinlock_t		lock;

	struct clk		*clk;
	void __iomem		*base;
	unsigned long		phys;

	u32			cur_speed;

	struct list_head	queue;
	struct spi_transfer	*cur;
	unsigned		cur_pos;
	unsigned		cur_len;
	unsigned		cur_bytes_per_word;

	/* The tegra spi controller has a bug which causes the first word
	 * in PIO transactions to be garbage.  Since packed DMA transactions
	 * require transfers to be 4 byte aligned we need a bounce buffer
	 * for the generic case.
	 */
	struct tegra_dma_req	rx_dma_req;
	struct tegra_dma_channel *rx_dma;
	u32			*rx_bb;
	dma_addr_t		rx_bb_phys;

	struct tegra_dma_req	tx_dma_req;
	struct tegra_dma_channel *tx_dma;
	u32			*tx_bb;
	dma_addr_t		tx_bb_phys;

	bool			is_suspended;
	unsigned long		save_slink_cmd;
	callback client_funct;
	void *client_data;

	u32 rx_complete;
	u32 tx_complete;
	bool abort_happen;

	u8 g_bits_per_word;
	struct dentry *debugfs;
};

static inline unsigned long spi_tegra_readl(struct spi_tegra_data *tspi,
					unsigned long reg)
{
	return readl(tspi->base + reg);
}

static inline void spi_tegra_writel(struct spi_tegra_data *tspi,
					unsigned long val,
					unsigned long reg)
{
	writel(val, tspi->base + reg);
}

static void spi_tegra_clear_status(struct spi_tegra_data *tspi)
{
	unsigned long val;
	unsigned long val_write = 0;

	val = spi_tegra_readl(tspi, SLINK_STATUS);
	if (val & SLINK_BSY)
		val_write |= SLINK_BSY;

	if (val & SLINK_ERR) {
		val_write |= SLINK_ERR;
		pr_err("%s ERROR bit set 0x%lx\n", __func__, val);
		if (val & SLINK_TX_OVF)
			val_write |= SLINK_TX_OVF;
		if (val & SLINK_RX_OVF)
			val_write |= SLINK_RX_OVF;
		if (val & SLINK_RX_UNF)
			val_write |= SLINK_RX_UNF;
		if (val & SLINK_TX_UNF)
			val_write |= SLINK_TX_UNF;
		if (!(val & SLINK_TX_EMPTY))
			val_write |= SLINK_TX_FLUSH;
		if (!(val & SLINK_RX_EMPTY))
			val_write |= SLINK_RX_FLUSH;
	}
	spi_tegra_writel(tspi, val_write, SLINK_STATUS);
}

static void spi_tegra_go(struct spi_tegra_data *tspi)
{
	unsigned long val;
	unsigned long test_val;
	unsigned unused_fifo_size;

	wmb();

	val = spi_tegra_readl(tspi, SLINK_DMA_CTL);
	val &= ~SLINK_DMA_BLOCK_SIZE(~0) & ~SLINK_DMA_EN;
	val |= SLINK_DMA_BLOCK_SIZE(tspi->rx_dma_req.size / 4 - 1);
	spi_tegra_writel(tspi, val, SLINK_DMA_CTL);
	tegra_dma_enqueue_req(tspi->tx_dma, &tspi->tx_dma_req);
	tegra_dma_enqueue_req(tspi->rx_dma, &tspi->rx_dma_req);

	val |= SLINK_DMA_EN;
	val &= ~SLINK_TX_TRIG_MASK & ~SLINK_RX_TRIG_MASK;

	if (tspi->rx_dma_req.size & 0xF)
		val |= SLINK_TX_TRIG_1 | SLINK_RX_TRIG_1;
	else if (((tspi->rx_dma_req.size) >> 4) & 0x1)
		val |= SLINK_TX_TRIG_4 | SLINK_RX_TRIG_4;
	else
		val |= SLINK_TX_TRIG_8 | SLINK_RX_TRIG_8;

	/*
	 * TRM 24.1.1.7 wait for the FIFO to be full
	 */
	test_val = spi_tegra_readl(tspi, SLINK_STATUS2);
	unused_fifo_size = (tspi->tx_dma_req.size/4) >= SLINK_FIFO_DEPTH ?
				0 :
				SLINK_FIFO_DEPTH - (tspi->tx_dma_req.size/4);
	while (SLINK_TX_FIFO_EMPTY_COUNT(test_val) != (unused_fifo_size))
		test_val = spi_tegra_readl(tspi, SLINK_STATUS2);

	spi_tegra_writel(tspi, val, SLINK_DMA_CTL);
}

static unsigned spi_tegra_fill_tx_fifo(struct spi_tegra_data *tspi,
				struct spi_transfer *t)
{
	unsigned len = min(t->len - tspi->cur_pos, BB_LEN *
				tspi->cur_bytes_per_word);
	u8 *tx_buf = (u8 *)t->tx_buf + tspi->cur_pos;
	int i, j;
	unsigned long val;

	val = spi_tegra_readl(tspi, SLINK_COMMAND);
	val &= ~SLINK_WORD_SIZE(~0);
	val |= SLINK_WORD_SIZE(len / tspi->cur_bytes_per_word - 1);
	spi_tegra_writel(tspi, val, SLINK_COMMAND);

	if (tspi->g_bits_per_word == 32) {
		memcpy(tspi->tx_bb, (void *)tx_buf, len);
	} else {
		for (i = 0; i < len; i += tspi->cur_bytes_per_word) {
			val = 0;
			for (j = 0; j < tspi->cur_bytes_per_word; j++)
				val |=
					tx_buf[i + j] << (tspi->cur_bytes_per_word - j - 1) * 8;

			tspi->tx_bb[i / tspi->cur_bytes_per_word] = val;
		}
	}

	tspi->tx_dma_req.size = len / tspi->cur_bytes_per_word * 4;

	return len;
}

static unsigned spi_tegra_drain_rx_fifo(struct spi_tegra_data *tspi,
				struct spi_transfer *t)
{
	unsigned len = tspi->cur_len;
	int i, j;
	u8 *rx_buf = (u8 *)t->rx_buf + tspi->cur_pos;
	unsigned long val;

	if (tspi->g_bits_per_word == 32) {
		memcpy(rx_buf, (void *)tspi->rx_bb, len);
	} else {
		for (i = 0; i < len; i += tspi->cur_bytes_per_word) {
			val = tspi->rx_bb[i / tspi->cur_bytes_per_word];
			for (j = 0; j < tspi->cur_bytes_per_word; j++)
				rx_buf[i + j] =
					(val >> (tspi->cur_bytes_per_word - j - 1) * 8) & 0xff;
		}
	}

	return len;
}

int spi_tegra_register_callback(struct spi_device *spi, callback func,
					void *client_data)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);

	if (!tspi || !func)
		return -EINVAL;
	tspi->client_funct = func;
	tspi->client_data = client_data;
	return 0;
}
EXPORT_SYMBOL(spi_tegra_register_callback);

static void spi_tegra_start_transfer(struct spi_device *spi,
					struct spi_transfer *t)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long cs_bit;
	u32 speed;
	u8 bits_per_word;
	unsigned long val;

	speed = t->speed_hz ? t->speed_hz : spi->max_speed_hz;
	bits_per_word = t->bits_per_word ? t->bits_per_word  :
		spi->bits_per_word;
	tspi->g_bits_per_word = bits_per_word;

	tspi->cur_bytes_per_word = (bits_per_word - 1) / 8 + 1;

	if (speed != tspi->cur_speed)
		clk_set_rate(tspi->clk, speed);

	if (tspi->cur_speed == 0)
		clk_enable(tspi->clk);

	tspi->cur_speed = speed;

	spi_tegra_clear_status(tspi);
	val = spi_tegra_readl(tspi, SLINK_COMMAND2);
	val &= ~(SLINK_SS_EN_CS(~0) | SLINK_RXEN | SLINK_TXEN);
	if (t->rx_buf)
		val |= SLINK_RXEN;
	if (t->tx_buf)
		val |= SLINK_TXEN;
	val |= SLINK_SS_EN_CS(spi->chip_select);
	val |= SLINK_SPIE;
	spi_tegra_writel(tspi, val, SLINK_COMMAND2);

	val = spi_tegra_readl(tspi, SLINK_COMMAND);
	switch (spi->chip_select) {
	case 0:
		cs_bit = SLINK_CS_POLARITY;
		break;

	case 1:
		cs_bit = SLINK_CS_POLARITY1;
		break;

	case 2:
		cs_bit = SLINK_CS_POLARITY2;
		break;

	case 4:
		cs_bit = SLINK_CS_POLARITY3;
		break;

	default:
		return;
	}
	if (spi->mode & SPI_CS_HIGH)
		val |= cs_bit;
	else
		val &= ~cs_bit;

	val &= ~SLINK_BIT_LENGTH(~0);
	val |= SLINK_BIT_LENGTH(bits_per_word - 1);

	/* FIXME: should probably control CS manually so that we can be sure
	 * it does not go low between transfer and to support delay_usecs
	 * correctly.
	 */
	val &= ~SLINK_IDLE_SCLK_MASK & ~SLINK_CK_SDA & ~SLINK_CS_SW;

	if (spi->mode & SPI_CPHA)
		val |= SLINK_CK_SDA;
	if (spi->mode & SPI_CPOL)
		val |= SLINK_IDLE_SCLK_DRIVE_HIGH;
	else
		val |= SLINK_IDLE_SCLK_DRIVE_LOW;

	val &= ~(SLINK_M_S); /* set slave mode */

	spi_tegra_writel(tspi, val, SLINK_COMMAND);
	spi_tegra_writel(tspi, SLINK_RX_FLUSH | SLINK_TX_FLUSH, SLINK_STATUS);
	tspi->cur = t;
	tspi->cur_pos = 0;
	tspi->cur_len = spi_tegra_fill_tx_fifo(tspi, t);
	tspi->rx_dma_req.size = tspi->tx_dma_req.size;
	tspi->rx_complete = 0;
	tspi->tx_complete = 0;
	tspi->abort_happen = false;

	spi_tegra_go(tspi);
	/* notify client that we're ready for transfer */
	if (tspi->client_funct)
		tspi->client_funct(tspi->client_data);
}

static void spi_tegra_start_message(struct spi_device *spi,
					struct spi_message *m)
{
	struct spi_transfer *t;

	m->actual_length = 0;
	m->status = 0;

	t = list_first_entry(&m->transfers, struct spi_transfer, transfer_list);
	spi_tegra_start_transfer(spi, t);
}

static void complete_operation(struct tegra_dma_req *req)
{
	struct spi_tegra_data *tspi = req->dev;
	unsigned long val;
	struct spi_message *m;
	struct spi_device *spi;
	u32 timeout = 0;
	u32 temp = 0;

	if (tspi->abort_happen == true) {
		unsigned long val_write = 0;
		val_write = spi_tegra_readl(tspi, SLINK_STATUS);
		val_write = val_write | SLINK_TX_FLUSH | SLINK_RX_FLUSH ;

		spi_tegra_writel(tspi, val_write, SLINK_STATUS);

		/*In order to make sure Tx fifo fluch is completed.*/
		while (spi_tegra_readl(tspi, SLINK_STATUS)&SLINK_TX_FLUSH)
			;
		/*In order to make sure Rx fifo fluch is completed.*/
		while (spi_tegra_readl(tspi, SLINK_STATUS)&SLINK_RX_FLUSH)
			;
		/*
		 * rx disable and tx disable
		 */
		val_write = spi_tegra_readl(tspi, SLINK_COMMAND2);
		val_write &= ~SLINK_RXEN;
		val_write &= ~SLINK_TXEN;
		spi_tegra_writel(tspi, val_write, SLINK_COMMAND2);

		/*
		 * reset the slink controller
		 * */
		tegra_periph_reset_assert(tspi->clk);
		udelay(50);
		tegra_periph_reset_deassert(tspi->clk);
		udelay(50);
	}

	/* the SPI controller may come back with both the BSY and RDY bits
	* set.  In this case we need to wait for the BSY bit to clear so
	* that we are sure the DMA is finished.  1000 reads was empirically
	* determined to be long enough.
	*/

	while ((spi_tegra_readl(tspi, SLINK_STATUS) & SLINK_BSY)) {
		if (timeout++ > 1000)
			break;
	}

	while ((spi_tegra_readl(tspi, SLINK_STATUS2)) != SLINK_STATUS2_RESET) {
		if (temp++ > 50000)
			break;
	}

	spi_tegra_clear_status(tspi);

	val = spi_tegra_readl(tspi, SLINK_STATUS);
	val |= SLINK_RDY;
	spi_tegra_writel(tspi, val, SLINK_STATUS);

	m = list_first_entry(&tspi->queue, struct spi_message, queue);

	if ((timeout >= 1000) || (temp >= 50000))
		m->status = -EIO;

	spi = m->state;

	tspi->cur_pos += spi_tegra_drain_rx_fifo(tspi, tspi->cur);
	m->actual_length += tspi->cur_pos;

	if (!list_is_last(&tspi->cur->transfer_list, &m->transfers)) {
		tspi->cur = list_first_entry(&tspi->cur->transfer_list,
			struct spi_transfer, transfer_list);
		spi_tegra_start_transfer(spi, tspi->cur);
	} else {
		list_del(&m->queue);

		m->complete(m->context);

		if (!list_empty(&tspi->queue)) {
			m = list_first_entry(&tspi->queue, struct spi_message,
				queue);
			spi = m->state;
			spi_tegra_start_message(spi, m);
		} else {
			clk_disable(tspi->clk);
			tspi->cur_speed = 0;
		}
	}
}

static void tegra_spi_tx_dma_complete(struct tegra_dma_req *req)
{
	struct spi_tegra_data *tspi = req->dev;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);

	(tspi->tx_complete)++;

	if (((tspi->rx_complete) == 1) && ((tspi->tx_complete) == 1))
		complete_operation(req);

	spin_unlock_irqrestore(&tspi->lock, flags);

}

static void tegra_spi_rx_dma_complete(struct tegra_dma_req *req)
{
	struct spi_tegra_data *tspi = req->dev;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);

	(tspi->rx_complete)++;

	if (((tspi->rx_complete) == 1) && ((tspi->tx_complete) == 1))
		complete_operation(req);

	spin_unlock_irqrestore(&tspi->lock, flags);
}

static int spi_tegra_setup(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "setup %d bpw, %scpol, %scpha, %dHz\n",
		spi->bits_per_word,
		spi->mode & SPI_CPOL ? "" : "~",
		spi->mode & SPI_CPHA ? "" : "~",
		spi->max_speed_hz);

	return 0;
}

static int spi_tegra_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	struct spi_transfer *t;
	unsigned long flags;
	int was_empty;

	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (t->bits_per_word < 0 || t->bits_per_word > 32)
			return -EINVAL;

		if (t->len == 0)
			return -EINVAL;

		if (!t->rx_buf && !t->tx_buf)
			return -EINVAL;
	}

	spin_lock_irqsave(&tspi->lock, flags);

	if (WARN_ON(tspi->is_suspended)) {
		spin_unlock_irqrestore(&tspi->lock, flags);
		return -EBUSY;
	}

	m->state = spi;

	was_empty = list_empty(&tspi->queue);
	list_add_tail(&m->queue, &tspi->queue);

	if (was_empty)
		spi_tegra_start_message(spi, m);

	spin_unlock_irqrestore(&tspi->lock, flags);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int spi_show_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define SPI_TEGRA_BUFSIZE	512
static ssize_t  spi_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct spi_tegra_data *tspi;
	char *buf;
	u32 len = 0;
	ssize_t ret;
	unsigned long flags;

	tspi = file->private_data;
	buf = kzalloc(SPI_TEGRA_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	spin_lock_irqsave(&tspi->lock, flags);
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SPI registers:\n");
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SLINK_STATUS 0x%lx\n", spi_tegra_readl(tspi, SLINK_STATUS));
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SLINK_STATUS2 0x%lx\n", spi_tegra_readl(tspi, SLINK_STATUS2));
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SLINK_COMMAND 0x%lx\n", spi_tegra_readl(tspi, SLINK_COMMAND));
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SLINK_COMMAND2 0x%lx\n", spi_tegra_readl(tspi, SLINK_COMMAND2));
	len += snprintf(buf + len, SPI_TEGRA_BUFSIZE - len,
		"SLINK_DMA_CTL 0x%lx\n", spi_tegra_readl(tspi, SLINK_DMA_CTL));
	spin_unlock_irqrestore(&tspi->lock, flags);
	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations tegra_spi_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= spi_show_regs_open,
	.read		= spi_show_regs,
};

static int tegra_spi_debugfs_init(struct spi_tegra_data *tspi)
{
	char name[20] = {0};

	sprintf(name, "%s.%d", tspi->pdev->name, tspi->pdev->id);
	tspi->debugfs = debugfs_create_dir(name, NULL);
	if (!tspi->debugfs)
		return -ENOMEM;
	debugfs_create_file("registers", S_IFREG | S_IRUGO,
		tspi->debugfs, (void *)tspi, &tegra_spi_regs_ops);
	return 0;
}

static void tegra_spi_debugfs_remove(struct spi_tegra_data *tspi)
{
	if (tspi->debugfs)
		debugfs_remove_recursive(tspi->debugfs);
}
#endif

static int __init spi_tegra_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof *tspi);
	if (master == NULL) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	master->setup = spi_tegra_setup;
	master->transfer = spi_tegra_transfer;
	master->num_chipselect = 4;

	dev_set_drvdata(&pdev->dev, master);
	tspi = spi_master_get_devdata(master);
	tspi->master = master;
	tspi->pdev = pdev;
	spin_lock_init(&tspi->lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENODEV;
		goto err0;
	}

	if (!request_mem_region(r->start, (r->end - r->start) + 1,
				dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto err0;
	}

	tspi->phys = r->start;
	tspi->base = ioremap(r->start, r->end - r->start + 1);
	if (!tspi->base) {
		dev_err(&pdev->dev, "can't ioremap iomem\n");
		ret = -ENOMEM;
		goto err1;
	}

	tspi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(tspi->clk)) {
		dev_err(&pdev->dev, "can not get clock\n");
		ret = PTR_ERR(tspi->clk);
		goto err2;
	}

	INIT_LIST_HEAD(&tspi->queue);

	tspi->rx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
	if (!tspi->rx_dma) {
		dev_err(&pdev->dev, "can not allocate rx dma channel\n");
		ret = -ENODEV;
		goto err3;
	}

	tspi->rx_bb = dma_alloc_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
					 &tspi->rx_bb_phys, GFP_KERNEL);
	if (!tspi->rx_bb) {
		dev_err(&pdev->dev, "can not allocate rx bounce buffer\n");
		ret = -ENOMEM;
		goto err4;
	}

	memset(&tspi->rx_dma_req, 0, sizeof(struct tegra_dma_req)) ;
	tspi->rx_dma_req.complete = tegra_spi_rx_dma_complete;
	tspi->rx_dma_req.to_memory = 1;
	tspi->rx_dma_req.dest_addr = tspi->rx_bb_phys;
	tspi->rx_dma_req.virt_addr = tspi->rx_bb ;
	tspi->rx_dma_req.dest_bus_width = 32;
	tspi->rx_dma_req.source_addr = tspi->phys + SLINK_RX_FIFO;
	tspi->rx_dma_req.source_bus_width = 32;
	tspi->rx_dma_req.source_wrap = 4;
	tspi->rx_dma_req.dest_wrap = 0 ;
	tspi->rx_dma_req.req_sel = spi_tegra_req_sels[pdev->id];
	tspi->rx_dma_req.dev = tspi;

	tspi->tx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
	if (IS_ERR(tspi->tx_dma)) {
		dev_err(&pdev->dev, "can not allocate tx dma channel\n");
		ret = PTR_ERR(tspi->tx_dma);
		goto err5;
	}

	tspi->tx_bb = dma_alloc_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
					 &tspi->tx_bb_phys, GFP_KERNEL);
	if (!tspi->tx_bb) {
		dev_err(&pdev->dev, "can not allocate tx bounce buffer\n");
		ret = -ENOMEM;
		goto err6;
	}

	memset(&tspi->tx_dma_req, 0, sizeof(struct tegra_dma_req)) ;
	tspi->tx_dma_req.complete = tegra_spi_tx_dma_complete;
	tspi->tx_dma_req.to_memory = 0;
	tspi->tx_dma_req.dest_addr = tspi->phys + SLINK_TX_FIFO;
	tspi->tx_dma_req.virt_addr = tspi->tx_bb ;
	tspi->tx_dma_req.dest_bus_width = 32;
	tspi->tx_dma_req.dest_wrap = 4;
	tspi->tx_dma_req.source_wrap = 0 ;
	tspi->tx_dma_req.source_addr = tspi->tx_bb_phys;
	tspi->tx_dma_req.source_bus_width = 32;
	tspi->tx_dma_req.req_sel = spi_tegra_req_sels[pdev->id];
	tspi->tx_dma_req.dev = tspi;

	ret = spi_register_master(master);
#ifdef CONFIG_DEBUG_FS
	tegra_spi_debugfs_init(tspi);
#endif
	if (ret < 0)
		goto err7;

	return ret;

err7:
	dma_free_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
				tspi->tx_bb, tspi->tx_bb_phys);
err6:
	tegra_dma_free_channel(tspi->tx_dma);
err5:
	dma_free_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
				tspi->rx_bb, tspi->rx_bb_phys);
err4:
	tegra_dma_free_channel(tspi->rx_dma);
err3:
	clk_put(tspi->clk);
err2:
	iounmap(tspi->base);
err1:
	release_mem_region(r->start, (r->end - r->start) + 1);
err0:
	spi_master_put(master);
	return ret;
}

static int __devexit spi_tegra_remove(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);
#ifdef CONFIG_DEBUG_FS
	tegra_spi_debugfs_remove(tspi);
#endif
	tegra_dma_free_channel(tspi->rx_dma);

	dma_free_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
			  tspi->rx_bb, tspi->rx_bb_phys);

	clk_put(tspi->clk);
	iounmap(tspi->base);

	spi_master_put(master);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, (r->end - r->start) + 1);

	return 0;
}

void spi_tegra_abort_transfer(struct spi_device *spi)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	struct spi_message *m;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);
	if (((tspi->rx_complete) != 0) || ((tspi->tx_complete) != 0))
		spin_unlock_irqrestore(&tspi->lock, flags);

	tspi->abort_happen = true;
	spin_unlock_irqrestore(&tspi->lock, flags);

	m = list_first_entry(&tspi->queue, struct spi_message, queue);
	m->status = -EIO;

	tegra_dma_dequeue(tspi->tx_dma);
	tegra_dma_dequeue(tspi->rx_dma);
}
EXPORT_SYMBOL(spi_tegra_abort_transfer);

#ifdef CONFIG_PM
static int spi_tegra_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	unsigned long		flags;
	unsigned		limit = 50;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);
	spin_lock_irqsave(&tspi->lock, flags);
	tspi->is_suspended = true;
	WARN_ON(!list_empty(&tspi->queue));

	while (!list_empty(&tspi->queue) && limit--) {
		spin_unlock_irqrestore(&tspi->lock, flags);
		msleep(10);
		spin_lock_irqsave(&tspi->lock, flags);
	}

	tspi->save_slink_cmd = spi_tegra_readl(tspi, SLINK_COMMAND);
	spin_unlock_irqrestore(&tspi->lock, flags);
	return 0;
}

static int spi_tegra_resume(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	unsigned long		flags;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);
	spin_lock_irqsave(&tspi->lock, flags);
	clk_enable(tspi->clk);
	spi_tegra_writel(tspi, tspi->save_slink_cmd, SLINK_COMMAND);
	clk_disable(tspi->clk);
	tspi->cur_speed = 0;
	tspi->is_suspended = false;
	spin_unlock_irqrestore(&tspi->lock, flags);
	return 0;
}
#endif

MODULE_ALIAS("platform:tegra_spi_slave");

static struct platform_driver spi_tegra_driver = {
	.driver = {
		.name =		"tegra_spi_slave",
		.owner =	THIS_MODULE,
	},
	.remove =	__devexit_p(spi_tegra_remove),
#ifdef CONFIG_PM
	.suspend =	spi_tegra_suspend,
	.resume  =	spi_tegra_resume,
#endif
};

static int __init spi_tegra_init(void)
{
	return platform_driver_probe(&spi_tegra_driver, spi_tegra_probe);
}
module_init(spi_tegra_init);

static void __exit spi_tegra_exit(void)
{
	platform_driver_unregister(&spi_tegra_driver);
}
module_exit(spi_tegra_exit);

MODULE_LICENSE("GPL");
