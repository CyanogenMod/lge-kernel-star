/*
 *  rindrv.c - This module is implemant Network interface over TTY device.
 *                   Proposel of it is support RAW IP communication. 
 *
 *                   This moule is based on SLIP driver code (Version:    0.8.3    12/24/94)
 *
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 */

#define RIN_CHECK_TRANSMIT
//#define RIN_DEINSALA_DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/delay.h>
#include <linux/init.h>
#include "rindrv.h"
#ifdef CONFIG_INET
#include <linux/ip.h>
#include <linux/tcp.h>
#include <net/slhc_vj.h>
#endif
#include <linux/workqueue.h>

static int rindrv_count = 0;
static struct net_device **rin_devs;

static int rin_maxdev = RIN_NRUNIT;
module_param(rin_maxdev, int, 0);
MODULE_PARM_DESC(rin_maxdev, "Maximum number of rin devices");

static struct workqueue_struct *rin_tx_wq = NULL;

struct rin_work
{
	struct work_struct   work;
	struct net_device*   dev;
	struct sk_buff*      skb;
};

/********************************
*  Buffer administration routines:
*	rin_alloc_bufs()
*	rin_free_bufs()
*	rin_realloc_bufs()
*
* NOTE: rin_realloc_bufs != rin_free_bufs + rin_alloc_bufs, because
*	rin_realloc_bufs provides strong atomicity and reallocation
*	on actively running device.
*********************************/

/*
   Allocate channel buffers.
 */

static int rin_alloc_bufs(struct rin_st *sl, int mtu)
{
	int err = -ENOBUFS;
	unsigned long len;
	char *rbuff = NULL;
	char *xbuff = NULL;

	/*
	 * Allocate the RIN frame buffers:
	 *
	 * rbuff	Receive buffer.
	 * xbuff	Transmit buffer.
	 * cbuff        Temporary compression buffer.
	 */
	len = mtu * 2;

	/*
	 * allow for arrival of larger UDP packets, even if we say not to
	 * also fixes a bug in which SunOS sends 512-byte packets even with
	 * an MSS of 128
	 */
	if (len < 576 * 2)
		len = 576 * 2;
	rbuff = kmalloc(len + 4, GFP_KERNEL);
	if (rbuff == NULL)
		goto err_exit;
	xbuff = kmalloc(len + 4, GFP_KERNEL);
	if (xbuff == NULL)
		goto err_exit;
	spin_lock_bh(&sl->lock);
	if (sl->tty == NULL) {
		spin_unlock_bh(&sl->lock);
		err = -ENODEV;
		goto err_exit;
	}
	sl->mtu	     = mtu;
	sl->buffsize = len;
	sl->rcount   = 0;
	sl->xleft    = 0;
	rbuff = xchg(&sl->rbuff, rbuff);
	xbuff = xchg(&sl->xbuff, xbuff);
	spin_unlock_bh(&sl->lock);
	err = 0;

	/* Cleanup */
err_exit:
	kfree(xbuff);
	kfree(rbuff);
	return err;
}

/* Free a RIN channel buffers. */
static void rin_free_bufs(struct rin_st *sl)
{
	/* Free all RIN frame buffers. */
	kfree(xchg(&sl->rbuff, NULL));
	kfree(xchg(&sl->xbuff, NULL));
}

/*
   Reallocate rin channel buffers.
 */

static int rin_realloc_bufs(struct rin_st *sl, int mtu)
{
	int err = 0;
	struct net_device *dev = sl->dev;
	unsigned char *xbuff, *rbuff;
	int len = mtu * 2;

/*
 * allow for arrival of larger UDP packets, even if we say not to
 * also fixes a bug in which SunOS sends 512-byte packets even with
 * an MSS of 128
 */
	if (len < 576 * 2)
		len = 576 * 2;

	xbuff = kmalloc(len + 4, GFP_ATOMIC);
	rbuff = kmalloc(len + 4, GFP_ATOMIC);


	if (xbuff == NULL || rbuff == NULL)  {
		if (mtu >= sl->mtu) {
			printk(KERN_WARNING "%s: unable to grow rin buffers, MTU change cancelled.\n",
			       dev->name);
			err = -ENOBUFS;
		}
		goto done;
	}
	spin_lock_bh(&sl->lock);

	err = -ENODEV;
	if (sl->tty == NULL)
		goto done_on_bh;

	xbuff    = xchg(&sl->xbuff, xbuff);
	rbuff    = xchg(&sl->rbuff, rbuff);
	if (sl->xleft)  {
		if (sl->xleft <= len)  {
			memcpy(sl->xbuff, sl->xhead, sl->xleft);
		} else  {
			sl->xleft = 0;
			sl->tx_dropped++;
		}
	}
	sl->xhead = sl->xbuff;

	if (sl->rcount)  {
		if (sl->rcount <= len) {
			memcpy(sl->rbuff, rbuff, sl->rcount);
		} else  {
			sl->rcount = 0;
			sl->rx_over_errors++;
			set_bit(SLF_ERROR, &sl->flags);
		}
	}
	sl->mtu      = mtu;
	dev->mtu      = mtu;
	sl->buffsize = len;
	err = 0;

done_on_bh:
	spin_unlock_bh(&sl->lock);

done:
	kfree(xbuff);
	kfree(rbuff);
	return err;
}


/* Set the "sending" flag.  This must be atomic hence the set_bit. */
static inline void rin_lock(struct rin_st *sl)
{
	netif_stop_queue(sl->dev);
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: sl_locked\n", sl->dev->name);
#endif
}


/* Clear the "sending" flag.  This must be atomic, hence the ASM. */
static inline void rin_unlock(struct rin_st *sl)
{
	netif_wake_queue(sl->dev);
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: sl_UNlocked\n", sl->dev->name);
#endif
}

/* Encapsulate one IP datagram and stuff into a TTY queue. */
static void rin_encaps(struct rin_st *sl, unsigned char *icp, int len)
{
	unsigned char *p;
	int actual, count;

	if (len > sl->mtu) {		/* Sigh, shouldn't occur BUT ... */
		printk(KERN_WARNING "%s: truncating oversized transmit packet!\n", sl->dev->name);
		sl->tx_dropped++;
		rin_unlock(sl);
		return;
	}

	p = icp;


	memcpy(sl->xbuff,p,len);
	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	actual = sl->tty->ops->write(sl->tty, sl->xbuff, len);
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_encaps: sent %d bytes of packet of %d bytes to TTY\n", sl->dev->name, actual, len);
#endif

#ifdef RIN_CHECK_TRANSMIT
	sl->dev->trans_start = jiffies;
#endif
	sl->xleft = count - actual;
#ifdef RIN_DEINSALA_DEBUG
    if(sl->xleft>0) printk(KERN_INFO "%s: rin_encaps: remaining %d bytes\n", sl->dev->name, sl->xleft);
#endif
	sl->xhead = sl->xbuff + actual;
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_encaps: tty_chars_in_buffer(sl->tty): %d\n", sl->dev->name, tty_chars_in_buffer(sl->tty));
    printk(KERN_INFO "%s: rin_encaps: sl->xleft: %d\n", sl->dev->name, sl->xleft);
    printk(KERN_INFO "%s: rin_encaps: sl->rcount: %d\n", sl->dev->name, sl->rcount);
    printk(KERN_INFO "%s: rin_encaps: sl->mtu: %d\n", sl->dev->name, sl->mtu);
    printk(KERN_INFO "%s: rin_encaps: sl->buffsize: %d\n", sl->dev->name, sl->buffsize);
#endif
}

/*
 * Called by the driver when there's room for more data.  If we have
 * more packets to send, we send them here.
 */
static void rin_write_wakeup(struct tty_struct *tty)
{
	int actual;
	struct rin_st *sl = tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != RIN_MAGIC || !netif_running(sl->dev))
        printk(KERN_INFO "rin_write_wakeup: not connected!!!");
		return;

	if (sl->xleft <= 0)  {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet */
		sl->tx_packets++;
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
#ifdef RIN_DEINSALA_DEBUG
        printk(KERN_INFO "%s: rin_write_wakeup: tty_chars_in_buffer(sl->tty): %d\n", sl->dev->name, tty_chars_in_buffer(sl->tty));
        printk(KERN_INFO "%s: rin_write_wakeup: sl->xleft: %d\n", sl->dev->name, sl->xleft);
        printk(KERN_INFO "%s: rin_write_wakeup: sl->rcount: %d\n", sl->dev->name, sl->rcount);
        printk(KERN_INFO "%s: rin_write_wakeup: sl->mtu: %d\n", sl->dev->name, sl->mtu);
        printk(KERN_INFO "%s: rin_write_wakeup: sl->buffsize: %d\n", sl->dev->name, sl->buffsize);
        printk(KERN_INFO "%s: rin_write_wakeup: unlocking net queue\n", sl->dev->name);
#endif
		rin_unlock(sl);
		return;
	}

	actual = tty->ops->write(tty, sl->xhead, sl->xleft);
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_write_wakeup: sent remaining %d bytes of packet to TTY\n", sl->dev->name, sl->xleft);
#endif
	sl->xleft -= actual;
	sl->xhead += actual;
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_write_wakeup: tty_chars_in_buffer(sl->tty): %d\n", sl->dev->name, tty_chars_in_buffer(sl->tty));
    printk(KERN_INFO "%s: rin_write_wakeup: sl->xleft: %d\n", sl->dev->name, sl->xleft);
    printk(KERN_INFO "%s: rin_write_wakeup: sl->rcount: %d\n", sl->dev->name, sl->rcount);
    printk(KERN_INFO "%s: rin_write_wakeup: sl->mtu: %d\n", sl->dev->name, sl->mtu);
    printk(KERN_INFO "%s: rin_write_wakeup: sl->buffsize: %d\n", sl->dev->name, sl->buffsize);
#endif
}

static void rin_nd_tx_timeout(struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_nd_tx_timeout: tty_chars_in_buffer(sl->tty): %d\n", sl->dev->name, tty_chars_in_buffer(sl->tty));
    printk(KERN_INFO "%s: rin_nd_tx_timeout: sl->xleft: %d\n", sl->dev->name, sl->xleft);
    printk(KERN_INFO "%s: rin_nd_tx_timeout: sl->rcount: %d\n", sl->dev->name, sl->rcount);
    printk(KERN_INFO "%s: rin_nd_tx_timeout: sl->mtu: %d\n", sl->dev->name, sl->mtu);
    printk(KERN_INFO "%s: rin_nd_tx_timeout: sl->buffsize: %d\n", sl->dev->name, sl->buffsize);
#endif
	spin_lock(&sl->lock);

	if (netif_queue_stopped(dev)) {
		if (!netif_running(dev))
			goto out;

		/* May be we must check transmitter timeout here ?
		 *      14 Oct 1994 Dmitry Gorodchanin.
		 */
#ifdef RIN_CHECK_TRANSMIT
		if (time_before(jiffies, dev->trans_start + 20 * HZ))  {
			/* 20 sec timeout not reached */
			goto out;
		}
		printk(KERN_WARNING "%s: transmit timed out, %s?\n",
			dev->name,
			(tty_chars_in_buffer(sl->tty) || sl->xleft) ?
				"bad line quality" : "driver error");
		sl->xleft = 0;
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
		rin_unlock(sl);
#endif
	}
out:
	spin_unlock(&sl->lock);
}

/* async tx task */
static void rin_tx_task(struct work_struct *pwork)
{
	struct rin_work *work = container_of(pwork, struct rin_work, work);
        struct sk_buff* skb;
	struct net_device *dev;
	struct rin_st *sl;
        /**/
        skb = work->skb;
	dev = work->dev;
        sl = netdev_priv(dev);
	/* release work */
	kfree(work);
        /* handle packet */
	rin_encaps(sl, skb->data, skb->len);
	dev_kfree_skb(skb);
        /* resume TX */
	rin_unlock(sl);
}

/* Encapsulate an IP datagram and kick it into a TTY queue. */
static int
rin_nd_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);
	struct rin_work* work;

	spin_lock(&sl->lock);
	if (!netif_running(dev)) {
		spin_unlock(&sl->lock);
		printk(KERN_WARNING "%s: xmit call when iface is down\n", dev->name);
		dev_kfree_skb(skb);
		return 0;
	}
	if (sl->tty == NULL) {
		spin_unlock(&sl->lock);
		dev_kfree_skb(skb);
		return 0;
	}

	rin_lock(sl);
	sl->tx_bytes += skb->len;
	sl->tx_packets++;
        /**/
	work = (struct rin_work*)kmalloc(sizeof(struct rin_work),GFP_ATOMIC);
	if (work) {
		work->dev = dev;
		work->skb = skb;
		INIT_WORK(&(work->work), rin_tx_task);
	        queue_work(rin_tx_wq, &(work->work));
	} else {
	 	/**/
		sl->tx_dropped++;
		dev_kfree_skb_irq(skb);
		rin_unlock(sl);
               	/**/
	}
	spin_unlock(&sl->lock);
        /**/
	return 0;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */

static int
rin_nd_close(struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);

	spin_lock_bh(&sl->lock);
	if (sl->tty)
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	netif_stop_queue(dev);
	sl->rcount   = 0;
	sl->xleft    = 0;
	spin_unlock_bh(&sl->lock);

	printk("%s - line : %d\n", __FUNCTION__, __LINE__);
	return 0;
}

/* Netdevice DOWN -> UP routine */

static int rin_nd_open(struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);

	if (sl->tty == NULL)
		return -ENODEV;

	sl->flags &= (1 << SLF_INUSE);
	netif_start_queue(dev);
	printk("%s - line : %d\n", __FUNCTION__, __LINE__);
	return 0;
}

/* Netdevice change MTU request */

static int rin_nd_change_mtu(struct net_device *dev, int new_mtu)
{
	struct rin_st *sl = netdev_priv(dev);

	if (new_mtu < 68 || new_mtu > 65534)
		return -EINVAL;

	if (new_mtu != dev->mtu)
		return rin_realloc_bufs(sl, new_mtu);
	return 0;
}

/* Netdevice get statistics request */

static struct net_device_stats *
rin_nd_get_stats(struct net_device *dev)
{
	static struct net_device_stats stats;
	struct rin_st *sl = netdev_priv(dev);

	memset(&stats, 0, sizeof(struct net_device_stats));

	stats.rx_packets     = sl->rx_packets;
	stats.tx_packets     = sl->tx_packets;
	stats.rx_bytes	     = sl->rx_bytes;
	stats.tx_bytes	     = sl->tx_bytes;
	stats.rx_dropped     = sl->rx_dropped;
	stats.tx_dropped     = sl->tx_dropped;
	stats.tx_errors      = sl->tx_errors;
	stats.rx_errors      = sl->rx_errors;
	stats.rx_over_errors = sl->rx_over_errors;
	return (&stats);
}

/* Netdevice register callback */

static int rin_nd_init(struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);

	/*
	 *	Finish setting up the DEVICE info.
	 */

	dev->mtu		= sl->mtu;
	dev->type		= ARPHRD_TUNNEL;
#ifdef RIN_CHECK_TRANSMIT
	dev->watchdog_timeo	= 20*HZ;
#endif
	return 0;
}


static void rin_nd_uninit(struct net_device *dev)
{
	struct rin_st *sl = netdev_priv(dev);

	rin_free_bufs(sl);
}

static const struct net_device_ops rin_netdev_ops = {
	.ndo_init		= rin_nd_init,
	.ndo_uninit	  	= rin_nd_uninit,
	.ndo_open		= rin_nd_open,
	.ndo_stop		= rin_nd_close,
	.ndo_start_xmit		= rin_nd_xmit,
	.ndo_get_stats	        = rin_nd_get_stats,
	.ndo_change_mtu		= rin_nd_change_mtu,
	.ndo_tx_timeout		= rin_nd_tx_timeout,
};


static void rin_setup(struct net_device *dev)
{
	dev->netdev_ops		= &rin_netdev_ops;
	dev->destructor		= free_netdev;

	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;

	/* New-style flags. */
	dev->flags		= IFF_NOARP|IFF_POINTOPOINT|IFF_MULTICAST;
}

/******************************************
  Routines looking at TTY side.
 ******************************************/


/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of RIN data has been received, which can now be decapsulated
 * and sent on to some IP layer for further processing. This will not
 * be re-entered while running but other ldisc functions may be called
 * in parallel
 */

static void rin_receive_buf(struct tty_struct *tty, const unsigned char *cp,
							char *fp, int count)
{
	struct rin_st *sl = tty->disc_data;
	struct sk_buff *skb;

	if (!sl || sl->magic != RIN_MAGIC || !netif_running(sl->dev))
		return;

// 2011.2.2 [ril] improve the performance of TCP Throughput [start]
	skb = dev_alloc_skb(count);
	if (skb == NULL) {
		printk(KERN_WARNING "%s: memory squeeze, dropping packet.\n", sl->dev->name);
        sl->rx_bytes += count;
		sl->rx_dropped++;
		return;
	}
	skb->dev = sl->dev;
	memcpy(skb_put(skb, count), cp, count);
	skb_reset_mac_header(skb);
	skb->protocol = htons(ETH_P_IP);

    spin_lock_bh(&sl->lock);
	sl->rx_bytes += count;
	netif_rx(skb);
	sl->rx_packets++;
    spin_unlock_bh(&sl->lock);

// 2011.2.2 [ril] improve the performance of TCP Throughput [end]
#ifdef RIN_DEINSALA_DEBUG
    printk(KERN_INFO "%s: rin_receive_buf: netif_rx(skb) received packet of %d bytes from TTY\n", sl->dev->name, count);
    printk(KERN_INFO "%s: rin_receive_buf: tty_chars_in_buffer(sl->tty): %d\n", sl->dev->name, tty_chars_in_buffer(sl->tty));
    printk(KERN_INFO "%s: rin_receive_buf: sl->xleft: %d\n", sl->dev->name, sl->xleft);
    printk(KERN_INFO "%s: rin_receive_buf: sl->rcount: %d\n", sl->dev->name, sl->rcount);
    printk(KERN_INFO "%s: rin_receive_buf: sl->mtu: %d\n", sl->dev->name, sl->mtu);
    printk(KERN_INFO "%s: rin_receive_buf: sl->buffsize: %d\n", sl->dev->name, sl->buffsize);
#endif
}

/************************************
 *  rin_open helper routines.
 ************************************/

/* Collect hanged up channels */
static void rin_sync(void)
{
	int i;
	struct net_device *dev;
	struct rin_st	  *sl;

	for (i = 0; i < rin_maxdev; i++) {
		dev = rin_devs[i];
		if (dev == NULL)
			break;

		sl = netdev_priv(dev);
		if (sl->tty || sl->leased)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}


/* Find a free RIN channel, and link in this `tty' line. */
static struct rin_st *rin_alloc(dev_t line)
{
	int i;
	int sel = -1;
	int score = -1;
	struct net_device *dev = NULL;
	struct rin_st       *sl;

	if (rin_devs == NULL)
		return NULL;	/* Master array missing ! */

	for (i = 0; i < rin_maxdev; i++) {
		dev = rin_devs[i];
		if (dev == NULL)
			break;

		sl = netdev_priv(dev);
		if (sl->leased) {
			if (sl->line != line)
				continue;
			if (sl->tty)
				return NULL;

			/* Clear ESCAPE & ERROR flags */
			sl->flags &= (1 << SLF_INUSE);
			return sl;
		}

		if (sl->tty)
			continue;

		if (current->pid == sl->pid) {
			if (sl->line == line && score < 3) {
				sel = i;
				score = 3;
				continue;
			}
			if (score < 2) {
				sel = i;
				score = 2;
			}
			continue;
		}
		if (sl->line == line && score < 1) {
			sel = i;
			score = 1;
			continue;
		}
		if (score < 0) {
			sel = i;
			score = 0;
		}
	}

	if (sel >= 0) {
		i = sel;
		dev = rin_devs[i];
		if (score > 1) {
			sl = netdev_priv(dev);
			sl->flags &= (1 << SLF_INUSE);
			return sl;
		}
	}

	/* Sorry, too many, all slots in use */
	if (i >= rin_maxdev)
		return NULL;

	if (dev) {
		sl = netdev_priv(dev);
		if (test_bit(SLF_INUSE, &sl->flags)) {
			unregister_netdevice(dev);
			dev = NULL;
			rin_devs[i] = NULL;
		}
	}

	if (!dev) {
		char name[IFNAMSIZ];
		sprintf(name, "vsnet%d", i);

		dev = alloc_netdev(sizeof(*sl), name, rin_setup);
		if (!dev)
			return NULL;
		dev->base_addr  = i;
	}

	sl = netdev_priv(dev);

	/* Initialize channel control data */
	sl->magic       = RIN_MAGIC;
	sl->dev	      	= dev;
	spin_lock_init(&sl->lock);
	rin_devs[i] = dev;
	
	return sl;
}

/*
 * Open the high-level part of the RIN channel.
 * This function is called by the TTY module when the
 * RIN line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free RIN channel...
 *
 * Called in process context serialized from other ldisc calls.
 */

static int rin_open(struct tty_struct *tty)
{
	struct rin_st *sl;
	int err;
	static int realloc_count = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	/* RTnetlink lock is misused here to serialize concurrent
	   opens of rin channels. There are better ways, but it is
	   the simplest one.
	 */
	rtnl_lock();

	/* Collect hanged up channels. */
	rin_sync();

	sl = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (sl && sl->magic == RIN_MAGIC)
		goto err_exit;

	/* OK.  Find a free RIN channel to use. */
	err = -ENFILE;
	if(rindrv_count==0)
	{
		sl = rin_alloc(tty_devnum(tty));
		printk("%s - line : %d, sl->dev = %x\n", __FUNCTION__, __LINE__, sl->dev);
	}
	else if(rindrv_count>0)
	{
		if(realloc_count<rindrv_count)
		{
			sl = netdev_priv(rin_devs[realloc_count]);
			printk("%s - line : %d, realloc sl->dev = %x\n", __FUNCTION__, __LINE__, sl->dev);
			realloc_count ++;
		}
	}
	else
	{
		printk("%s - line : %d, error rindrv_count = %d\n", __FUNCTION__, __LINE__, rindrv_count);
		sl = NULL;
	}
	if (sl == NULL)
		goto err_exit;

	sl->tty = tty;
	tty->disc_data = sl;
	sl->line = tty_devnum(tty);
	sl->pid = current->pid;

	if(rindrv_count==0 && realloc_count == 0)
	{
		if (!test_bit(SLF_INUSE, &sl->flags)) {
			/* Perform the low-level RIN initialization. */
			err = rin_alloc_bufs(sl, RIN_MTU);
			if (err)
				goto err_free_chan;

			set_bit(SLF_INUSE, &sl->flags);

			err = register_netdevice(sl->dev);
			if (err)
				goto err_free_bufs;
		}
	}
	else if( rindrv_count == realloc_count)
	{
		rindrv_count = 0;
		realloc_count = 0;
		printk("%s - line : %d, realloc done!!\n", __FUNCTION__, __LINE__);
		
	}
	/* Done.  We have linked the TTY line to a channel. */
	rtnl_unlock();
	tty->receive_room = 65536;	/* We don't flow control */
	return sl->dev->base_addr;

err_free_bufs:
	rin_free_bufs(sl);

err_free_chan:
	sl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &sl->flags);

err_exit:
	rtnl_unlock();
	printk("%s - line : %d - error exit\n", __FUNCTION__, __LINE__);

	/* Count references from TTY module */
	return err;
}

/*

  FIXME: 1,2 are fixed 3 was never true anyway.

   Let me to blame a bit.
   1. TTY module calls this funstion on soft interrupt.
   2. TTY module calls this function WITH MASKED INTERRUPTS!
   3. TTY module does not notify us about line discipline
      shutdown,

   Seems, now it is clean. The solution is to consider netdevice and
   line discipline sides as two independent threads.

   By-product (not desired): sl? does not feel hangups and remains open.
   It is supposed, that user level program (dip, diald, slattach...)
   will catch SIGHUP and make the rest of work.

   I see no way to make more with current tty code. --ANK
 */

/*
 * Close down a RIN channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions. 
 */
static void rin_close(struct tty_struct *tty)
{
	struct rin_st *sl = tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != RIN_MAGIC || sl->tty != tty)
		return;

	tty_ldisc_flush(tty);
	tty->disc_data = NULL;
	sl->tty = NULL;
	if (!sl->leased)
		sl->line = 0;
	
	rindrv_count++;
	printk("%s - line : %d - rindrv_count = %d\n", __FUNCTION__, __LINE__, rindrv_count);
}

/* Perform I/O control on an active RIN channel. */
static int rin_ioctl(struct tty_struct *tty, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct rin_st *sl = tty->disc_data;
	unsigned int tmp;
	int __user *p = (int __user *)arg;

	/* First make sure we're connected. */
	if (!sl || sl->magic != RIN_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		tmp = strlen(sl->dev->name) + 1;
		if (copy_to_user((void __user *)arg, sl->dev->name, tmp))
			return -EFAULT;
		return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static struct tty_ldisc_ops rin_ldisc = {
	.owner 		= THIS_MODULE,
	.magic 		= TTY_LDISC_MAGIC,
	.name 		= "rin",
	.open 		= rin_open,
	.close	 	= rin_close,
	.ioctl			= rin_ioctl,
	.receive_buf	= rin_receive_buf,
	.write_wakeup	= rin_write_wakeup,
};

static int __init rin_init(void)
{
	int status;

	if (rin_maxdev < 4)
		rin_maxdev = 4; /* Sanity */

	rin_devs = kzalloc(sizeof(struct net_device *)*rin_maxdev,
								GFP_KERNEL);
	if (!rin_devs) {
		printk(KERN_ERR "RIN: Can't allocate slip devices array.\n");
		return -ENOMEM;
	}

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_RIN, &rin_ldisc);
	if (status != 0) {
		printk(KERN_ERR "RIN: can't register line discipline (err = %d)\n", status);
		kfree(rin_devs);
	}
        if (status == 0) {
	       	rin_tx_wq = create_singlethread_workqueue("rintx");
        	if (!rin_tx_wq) {
			printk(KERN_ERR "RIN: failed to allocate workqueue");
			tty_unregister_ldisc(N_RIN);
			kfree(rin_devs);
			status = -ENOMEM;
        	}
        }
	return status;
}

static void __exit rin_exit(void)
{
	int i;
	struct net_device *dev;
	struct rin_st *sl;
	unsigned long timeout = jiffies + HZ;
	int busy = 0;

	if (rin_devs == NULL)
		return;

	/* First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < rin_maxdev; i++) {
			dev = rin_devs[i];
			if (!dev)
				continue;
			sl = netdev_priv(dev);
			spin_lock_bh(&sl->lock);
			if (sl->tty) {
				busy++;
				tty_hangup(sl->tty);
			}
			spin_unlock_bh(&sl->lock);
		}
	} while (busy && time_before(jiffies, timeout));


	for (i = 0; i < rin_maxdev; i++) {
		dev = rin_devs[i];
		if (!dev)
			continue;
		rin_devs[i] = NULL;

		sl = netdev_priv(dev);
		if (sl->tty) {
			printk(KERN_ERR "%s: tty discipline still running\n",
			       dev->name);
			/* Intentionally leak the control block. */
			dev->destructor = NULL;
		}

		unregister_netdev(dev);
	}

	kfree(rin_devs);
	rin_devs = NULL;

	i = tty_unregister_ldisc(N_RIN);
	if (i != 0) {
		printk(KERN_ERR "RIN: can't unregister line discipline (err = %d)\n", i);
}
	if (rin_tx_wq)	{
               	flush_workqueue(rin_tx_wq);
		destroy_workqueue(rin_tx_wq);
		rin_tx_wq = NULL;
	}
}

module_init(rin_init);
module_exit(rin_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_RIN);
