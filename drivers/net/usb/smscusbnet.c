/*
 * USB Network driver infrastructure
 * Copyright (C) 2000-2005 by David Brownell
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a generic "USB networking" framework that works with several
 * kinds of full and high speed networking devices:  host-to-host cables,
 * smart usb peripherals, and actual Ethernet adapters.
 *
 * These devices usually differ in terms of control protocols (if they
 * even have one!) and sometimes they define new framing to wrap or batch
 * Ethernet packets.  Otherwise, they talk to USB pretty much the same,
 * so interface (un)binding, endpoint I/O queues, fault handling, and other
 * issues can usefully be addressed by this framework.
 */

// #define	DEBUG			// error path messages, extra info
// #define	VERBOSE			// more; success messages

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13))
#include <linux/config.h>
#endif

#include <linux/module.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#include <linux/moduleparam.h>
#endif

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/if_vlan.h>
#include "smscusbnet.h"
#include "version.h"
/*-------------------------------------------------------------------------*/

/*
 * Nineteen USB 1.1 max size bulk transactions per frame (ms), max.
 * Several dozen bytes of IPv4 data can fit in two such transactions.
 * One maximum size Ethernet packet takes twenty four of them.
 * For high speed, each frame comfortably fits almost 36 max size
 * Ethernet packets (so queues should be bigger).
 *
 * REVISIT qlens should be members of 'struct usbnet'; the goal is to
 * let the USB host controller be busy for 5msec or more before an irq
 * is required, under load.  Jumbograms change the equation.
 */
#define	RX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? rx_queue_size : 4)
#define	TX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? tx_queue_size : 4)

// reawaken network queue this soon after stopping; else watchdog barks
#define TX_TIMEOUT_JIFFIES	(5*HZ)

#define Link_Check_Delay      (1*HZ)

// throttle rx/tx briefly after some faults, so khubd might disconnect()
// us (it polls at HZ/4 usually) before we report too many false errors.
#define THROTTLE_JIFFIES	(HZ/8)

// between wakeups
#define UNLINK_TIMEOUT_MS	3

#define TX_PENDING_LEN    30
/*-------------------------------------------------------------------------*/

// randomly generated ethernet address
static u8	node_id [ETH_ALEN];

static const char driver_name [] = "smscusbnet";

/* use ethtool to change the level for any given device */
static int msg_level = -1;
module_param (msg_level, int, 0);
MODULE_PARM_DESC (msg_level, "Override default message level");

//operational_mode = 0----> low latency
//operational_mode = 1----> low power
static int operational_mode = 0;
module_param(operational_mode,int, 0);
MODULE_PARM_DESC(operational_mode,"Enable operational mode");

static unsigned long rx_queue_size = 60UL;
module_param(rx_queue_size, ulong, 0);
MODULE_PARM_DESC(rx_queue_size,"Specifies the size of the rx queue lenght");

static unsigned long tx_queue_size = 60UL;
module_param(tx_queue_size, ulong, 0);
MODULE_PARM_DESC(tx_queue_size,"Specifies the size of the tx queue lenght");

int tx_hold_on_completion = TRUE;
module_param(tx_hold_on_completion, bool, 0);
MODULE_PARM_DESC(tx_hold_on_completion,"Hold tx until USB completion if Enable");


static int smscusbnet_xmit (struct sk_buff *skb, struct net_device *net);
static int smscusbnet_bundle_skb_ximt(struct usbnet *dev, struct sk_buff_head *q);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static struct net_device_stats *smscusbnet_get_stats (struct net_device *net);
static int smscusbnet_stop (struct net_device *net);
static int smscusbnet_open (struct net_device *net);
static int smscusbnet_start_xmit (struct sk_buff *skb, struct net_device *net);
static void smscusbnet_tx_timeout (struct net_device *net);
static int smscusbnet_change_mtu (struct net_device *net, int new_mtu);
#endif

static inline struct urb *smscusbnet_get_rx_urb(struct usbnet *dev)
{
	unsigned long lockflags;
	struct urb * urb = NULL;

	spin_lock_irqsave (&dev->rx_urblist_lock, lockflags);
	if(dev->rx_urb_pool_head != dev->rx_urb_pool_tail){
		urb = dev->rx_urb_pool[dev->rx_urb_pool_head];
		dev->rx_urb_pool[dev->rx_urb_pool_head] = NULL;
		dev->rx_urb_pool_head = (dev->rx_urb_pool_head + 1) % dev->rx_urb_pool_size;
	}
	spin_unlock_irqrestore (&dev->rx_urblist_lock, lockflags);

	return urb;
}

static inline int smscusbnet_return_rx_urb(struct usbnet *dev, struct urb *urb)
{
	unsigned long lockflags;

	//return this urb to rx urb pool
	if(urb == NULL){
		return 0;
	}

	spin_lock_irqsave(&dev->rx_urblist_lock, lockflags);
	dev->rx_urb_pool[dev->rx_urb_pool_tail] = urb;
	dev->rx_urb_pool_tail = (dev->rx_urb_pool_tail + 1) % dev->rx_urb_pool_size;

	spin_unlock_irqrestore (&dev->rx_urblist_lock, lockflags);

	return 0;
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
static const struct net_device_ops smscusbnet_netdev_ops =
{
        .ndo_open               = smscusbnet_open,
        .ndo_stop               = smscusbnet_stop,
        .ndo_start_xmit         = smscusbnet_start_xmit,
        .ndo_tx_timeout         = smscusbnet_tx_timeout,
        .ndo_change_mtu         = smscusbnet_change_mtu,
        .ndo_get_stats          = smscusbnet_get_stats,
        .ndo_validate_addr      = eth_validate_addr,
};
#endif  //linux 2.6.29

/*-------------------------------------------------------------------------*/

int smscusbnet_IsOperationalMode(struct usbnet *dev)
{

	return operational_mode;
}
/*-------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------*/

/* handles CDC Ethernet and many other network "bulk data" interfaces */
int smscusbnet_get_endpoints(struct usbnet *dev, struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt = NULL;
	struct usb_host_endpoint	*in = NULL, *out = NULL;
	struct usb_host_endpoint	*status = NULL;

	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		in = out = status = NULL;
		alt = intf->altsetting + tmp;

		/* take the first altsetting with in-bulk + out-bulk;
		 * remember any status endpoint, just in case;
		 * ignore other endpoints and altsetttings.
		 */
		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
			struct usb_host_endpoint	*e;
			int				intr = 0;

			e = alt->endpoint + ep;
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_INT:
				if (!(e->desc.bEndpointAddress & USB_DIR_IN))
					continue;
				intr = 1;
				/* FALLTHROUGH */
			case USB_ENDPOINT_XFER_BULK:
				break;
			default:
				continue;
			}
			if (e->desc.bEndpointAddress & USB_DIR_IN) {
				if (!intr && !in)
					in = e;
				else if (intr && !status)
					status = e;
			} else {
				if (!out)
					out = e;
			}
		}
		if (in && out)
			break;
	}
	if (!alt || !in || !out)
	{
        devdbg (dev, "return EINVAL = %d", EINVAL);
		return -EINVAL;
	}
	if (alt->desc.bAlternateSetting != 0
			|| !(dev->driver_info->flags & FLAG_NO_SETINT)) {
		tmp = usb_set_interface (dev->udev, alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (tmp < 0)
		{
            devdbg (dev, "return tmp = %d", tmp);
			return tmp;

		}
	}

	dev->in = usb_rcvbulkpipe (dev->udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->out = usb_sndbulkpipe (dev->udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->status = status;

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void intr_complete (struct urb *urb, struct pt_regs *regs);
#else
static void intr_complete (struct urb *urb);
#endif

static int init_status (struct usbnet *dev, struct usb_interface *intf)
{
	unsigned	pipe = 0;
	unsigned	maxp;
	unsigned	period;
	int	 i;

	if (!dev->driver_info->status)
		return 0;

	if(dev->chipID == ID_REV_7500_CHIPID){
		dev->device_id = DEV_SMSC7500;
		dev->tx_hold_on_completion = 0; //LAN7500 doesn't need this feature
	}else{
		dev->device_id = DEV_SMSC9500;
		dev->tx_hold_on_completion = tx_hold_on_completion;
	}

	pipe = usb_rcvintpipe (dev->udev,
			dev->status->desc.bEndpointAddress
				& USB_ENDPOINT_NUMBER_MASK);
	maxp = usb_maxpacket (dev->udev, pipe, 0);


	period = dev->status->desc.bInterval;
	dev->interrupt_urb_buffer = NULL;
	if(operational_mode){

		dev->interrupt_urb_buffer = kmalloc (maxp, GFP_KERNEL);
		if (dev->interrupt_urb_buffer) {
			dev->interrupt = usb_alloc_urb (0, GFP_KERNEL);
			if (!dev->interrupt) {
                devdbg (dev, "failed to alloc interrupt urb\n!!!");
				kfree (dev->interrupt_urb_buffer);
				dev->interrupt_urb_buffer = NULL;
				return -ENOMEM;
			} else {
				usb_fill_int_urb(dev->interrupt, dev->udev, pipe,
					dev->interrupt_urb_buffer, maxp, intr_complete, dev, period);
				devdbg (dev,"status ep%din, %d bytes period %d\n",
					usb_pipeendpoint(pipe), maxp, period);
			}
		}
	}

	dev->rx_urb_pool_head = 0;
	dev->rx_urb_pool_tail = 0;
	dev->rx_urb_pool_size = RX_QLEN (dev);
	//Allocate extra one to distinguish between empty and full list
	dev->rx_urb_pool_size += 1;
	dev->rx_urb_pool = kmalloc(sizeof(struct urb*) * dev->rx_urb_pool_size, GFP_KERNEL);
	memset(dev->rx_urb_pool, 0, sizeof(struct urb*) * dev->rx_urb_pool_size);

	if(!dev->rx_urb_pool){
		devwarn(dev, "unable to allocate memory for rx_urb_pool");
		return -ENOMEM;
	}

	for(i=0; i<(dev->rx_urb_pool_size-1); i++){
		dev->rx_urb_pool[i] = usb_alloc_urb (0, GFP_KERNEL);
		if(!dev->rx_urb_pool[i]){
			devwarn(dev, "failed to alloc rx urb");
			return -ENOMEM;
		}
		 //URB_ASYNC_UNLINK: usb_unlink_urb will return immediately without waiting for complete handler
		 //	which might result in deadlock problem on muitl-core cpu
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))
		dev->rx_urb_pool[i]->transfer_flags |= URB_ASYNC_UNLINK;
#endif

	}
	dev->rx_urb_pool_tail = dev->rx_urb_pool_size - 1;
	spin_lock_init(&(dev->rx_urblist_lock));
	devdbg (dev, "init_status, rx_urb_pool_head = %d, rx_urb_pool_tail = %d", dev->rx_urb_pool_head, dev->rx_urb_pool_tail);

	return  0;
}

/* Passes this packet up the stack, updating its accounting.
 * Some link protocols batch packets, so their rx_fixup paths
 * can return clones as well as just modify the original skb.
 */
void smscusbnet_skb_return (struct usbnet *dev, struct sk_buff *skb)
{
	int	status;
	u16 vlan_tag = *((u16 *)&skb->cb[0]);

	skb->dev = dev->net;
	skb->protocol = eth_type_trans (skb, dev->net);

	if (netif_msg_rx_status (dev))
		devdbg (dev, "< rx, len %zu, type 0x%x",
			skb->len + sizeof (struct ethhdr), skb->protocol);
	memset (skb->cb, 0, sizeof (struct skb_data));

	if(vlan_tag == VLAN_DUMMY){//No vlan tag acceleration
		status = netif_rx (skb);
	}else{//Vlan tag acceleration
		status = vlan_hwaccel_rx(skb, dev->vlgrp, vlan_tag);
	}

	if(status == NET_RX_DROP){
		dev->stats.rx_dropped++;
	}else{
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += skb->len;
	}

	if (status != NET_RX_SUCCESS && netif_msg_rx_err (dev))
		devdbg (dev, "netif_rx status %d", status);
}

/*-------------------------------------------------------------------------
 *
 * Network Device Driver (peer link to "Host Device", from USB host)
 *
 *-------------------------------------------------------------------------*/

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static
#endif
int smscusbnet_change_mtu (struct net_device *net, int new_mtu)
{
	struct usbnet	*dev = netdev_priv(net);
	int		ll_mtu = new_mtu + net->hard_header_len;

#ifdef JUMBO_FRAME
	if (new_mtu <= 0)
		return -EINVAL;
	// no second zero-length packet read wanted after mtu-sized packets
	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;
	if(dev->driver_info->set_max_frame_size(dev, net->hard_header_len + new_mtu - EXTRA_HEADER_LEN) < 0){
		return -EINVAL;
	}
	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;
#else
	if (new_mtu <= 0 || ll_mtu > dev->hard_mtu)
		return -EINVAL;
	// no second zero-length packet read wanted after mtu-sized packets
	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;
	net->mtu = new_mtu;
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static
#endif
struct net_device_stats *smscusbnet_get_stats (struct net_device *net)
{
	struct usbnet	*dev = netdev_priv(net);
	return &dev->stats;
}

/*-------------------------------------------------------------------------*/

/* some LK 2.4 HCDs oopsed if we freed or resubmitted urbs from
 * completion callbacks.  2.5 should have fixed those bugs...
 */

static void defer_bh(struct usbnet *dev, struct sk_buff *skb, struct sk_buff_head *list)
{
	unsigned long		flags;

	spin_lock_irqsave(&list->lock, flags);
	__skb_unlink(skb, list);
	spin_unlock(&list->lock);
	spin_lock(&dev->done.lock);
	__skb_queue_tail(&dev->done, skb);
	if (dev->done.qlen == 1)
		tasklet_schedule(&dev->bh);
	spin_unlock_irqrestore(&dev->done.lock, flags);
}

/* some work can't be done in tasklets, so we use keventd
 *
 * NOTE:  annoying asymmetry:  if it's active, schedule_work() fails,
 * but tasklet_schedule() doesn't.  hope the failure is rare.
 */
void smscusbnet_defer_kevent (struct usbnet *dev, int work)
{
	set_bit (work, &dev->flags);
	if (!schedule_work (&dev->kevent)){
		devdbg (dev, "kevent %d may have been dropped", work);
	}
//	else
//		devdbg (dev, "kevent %d scheduled", work);
}

/* some work can't be done in tasklets, so we use keventd
 *
 * We use myevent to schedule Rx Bulk in
 *
 */

void smscusbnet_defer_myevent (struct usbnet *dev, int work)
{
	set_bit (work, &dev->flags);
	if (!queue_work(dev->MyWorkQueue,&dev->myevent)){
		//deverr (dev, "myevent %d may have been dropped", work);
	}
	//else
	//	devdbg (dev,"myevent %d scheduled", work);
}

void smscusbnet_linkpolling(unsigned long ptr)
{

	struct usbnet * dev= (struct usbnet *) ptr;
	smscusbnet_defer_myevent(dev, EVENT_LINK_RESET);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
		if(dev->dynamicSuspend){
		   smscusbnet_defer_myevent(dev, EVENT_IDLE_CHECK);
		}
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
		if(dev->linkDownSuspend || dev->smartDetach){
#else
		if(dev->smartDetach){
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
			if(!netif_carrier_ok(dev->net) && dev->linkcheck++ > 2){//Check 2 times in case of conflict with resume
				smscusbnet_defer_myevent(dev, EVENT_LINK_DOWN);
				dev->linkcheck = 0;
			}
		}
}



/*-------------------------------------------------------------------------*/

//static void rx_complete (struct urb *urb, struct pt_regs *regs);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void rx_complete (struct urb *urb, struct pt_regs *regs );
#else
static void rx_complete (struct urb *urb);
#endif
static void rx_submit (struct usbnet *dev, struct urb *urb, int flags)
{
	struct sk_buff		*skb;
	struct skb_data		*entry;
	int			retval = 0;
	unsigned long		lockflags;
	size_t			size = dev->rx_urb_size;

#ifndef RX_OFFSET
	if ((skb = alloc_skb (size + NET_IP_ALIGN , flags)) == NULL) {
#else
	if ((skb = alloc_skb (size, flags)) == NULL) {
#endif //RX_OFFSET
		if (netif_msg_rx_err (dev))
			devdbg (dev,"no rx skb\n");
		smscusbnet_defer_kevent (dev, EVENT_RX_MEMORY);
		smscusbnet_return_rx_urb(dev, urb);

		return;
	}

#ifndef RX_OFFSET
	skb_reserve (skb, NET_IP_ALIGN);
#endif

	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->dev = dev;
	entry->state = rx_start;
	entry->length = 0;

	usb_fill_bulk_urb (urb, dev->udev, dev->in,
		skb->data, size, rx_complete, skb);

	spin_lock_irqsave (&dev->rxq.lock, lockflags);

	if (netif_running (dev->net)
			&& netif_device_present (dev->net)
			&& !test_bit (EVENT_RX_HALT, &dev->flags)) {
		switch (retval = usb_submit_urb (urb, GFP_ATOMIC)){
		case -EPIPE:
			smscusbnet_defer_kevent (dev, EVENT_RX_HALT);
			break;
		case -ENOMEM:
			smscusbnet_defer_kevent (dev, EVENT_RX_MEMORY);
			break;
		case -ENODEV:
			if (netif_msg_ifdown (dev))
				devdbg (dev, "device gone");
			netif_device_detach (dev->net);
			break;
		default:
			if (netif_msg_rx_err (dev))
				devdbg (dev, "rx submit, %d", retval);
			tasklet_schedule (&dev->bh);
			break;
		case 0:
			__skb_queue_tail (&dev->rxq, skb);
		}
	} else {
		if (netif_msg_ifdown (dev))
			devdbg (dev, "rx: stopped");
		retval = -ENOLINK;
	}
	spin_unlock_irqrestore (&dev->rxq.lock, lockflags);
	if (retval) {
		dev_kfree_skb_any (skb);
		smscusbnet_return_rx_urb(dev, urb);
	}
}

//-------------------------------------------------------------------------

static inline void rx_process (struct usbnet *dev, struct sk_buff *skb)
{
	int ret;

	if (!dev->driver_info->rx_fixup)return;

	ret = dev->driver_info->rx_fixup (dev, skb);

	if(ret == RX_FIXUP_INVALID_SKB)skb_queue_tail(&dev->done, skb);
	else if(ret == RX_FIXUP_ERROR){
		dev->stats.rx_errors++;
		skb_queue_tail (&dev->done, skb);
	}
	else{
		if (skb->len) {
			smscusbnet_skb_return (dev, skb);

	    } else {
			if (netif_msg_rx_err (dev))
				devdbg (dev, "drop");
			dev->stats.rx_errors++;
			skb_queue_tail (&dev->done, skb);
		}
	}

}

/*-------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void rx_complete (struct urb *urb, struct pt_regs *regs)
#else
static void rx_complete (struct urb *urb)
#endif

{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct usbnet		*dev = entry->dev;
	int			urb_status = urb->status;

    dev->idleCount = 0;

	skb_put (skb, urb->actual_length);
	entry->state = rx_done;
	entry->urb = NULL;

	switch (urb_status) {
	    // success
	    case 0:
		if (operational_mode) {
			if ((skb->len < dev->net->hard_header_len) && (skb->len != 0)) {
				entry->state = rx_cleanup;
				dev->stats.rx_errors++;
				dev->stats.rx_length_errors++;
				if (netif_msg_rx_err (dev))
					devdbg (dev, "rx length %d", skb->len);
			}

			if (skb->len == 0) {
				entry->state = rx_cleanup;
				dev->StopSummitUrb=1;
			}
			else {
				dev->StopSummitUrb=0;
			}

		} else {
			if (skb->len < dev->net->hard_header_len) {
				entry->state = rx_cleanup;
				dev->stats.rx_errors++;
				dev->stats.rx_length_errors++;
				if (netif_msg_rx_err (dev))
					devdbg (dev, "rx length %d", skb->len);
			}

		}
		break;

	    // stalls need manual reset. this is rare ... except that
	    // when going through USB 2.0 TTs, unplug appears this way.
	    // we avoid the highspeed version of the ETIMEOUT/EILSEQ
	    // storm, recovering as needed.
	    case -EPIPE:

        dev->extra_error_cnts.rx_epipe++;
		dev->stats.rx_errors++;
		devdbg (dev,"in rx_complete,case -EPIPE, dev->stats.rx_errors=0x%08lx\n", dev->stats.rx_errors);
		smscusbnet_defer_kevent (dev, EVENT_RX_HALT);
		// FALLTHROUGH

	    // software-driven interface shutdown
	    case -ECONNRESET:		// async unlink
	    case -ESHUTDOWN:		// hardware gone
		if (netif_msg_ifdown (dev))
			devdbg (dev, "rx shutdown, code %d", urb_status);
		goto block;

	    // we get controller i/o faults during khubd disconnect() delays.
	    // throttle down resubmits, to avoid log floods; just temporarily,
	    // so we still recover when the fault isn't a khubd delay.
        case -EPROTO:       // ehci
            dev->extra_error_cnts.rx_eproto++;
            goto _HANDLE;
        case -ETIMEDOUT:    // ohci
            dev->extra_error_cnts.rx_etimeout++;
            goto _HANDLE;
        case -EILSEQ:       // uhci
            dev->extra_error_cnts.rx_eilseq++;
_HANDLE:

		dev->stats.rx_errors++;
		devdbg (dev,"in rx_complete,case -EPROTO, dev->stats.rx_errors=0x%08lx\n", dev->stats.rx_errors);
		if (!timer_pending (&dev->delay)) {
			mod_timer (&dev->delay, jiffies + THROTTLE_JIFFIES);
			if (netif_msg_link (dev))
				devdbg (dev, "rx throttle %d", urb_status);
		}
        //Set recovery flag
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);

block:
		entry->state = rx_cleanup;
		entry->urb = urb;
		urb = NULL;
		break;

	    // data overrun ... flush fifo?
	    case -EOVERFLOW:
        dev->extra_error_cnts.rx_eoverflow++;
		dev->stats.rx_over_errors++;
		// FALLTHROUGH
	    //Set recovery flag
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);

	    default:
		entry->state = rx_cleanup;
		dev->stats.rx_errors++;

		devdbg (dev,"in rx_complete,case default dev->stats.rx_errors=0x%08lx\n", dev->stats.rx_errors);
		if (netif_msg_rx_err (dev))
			devdbg (dev, "rx status %d", urb_status);
		break;
	}

	defer_bh(dev, skb, &dev->rxq);

	if (urb) {
		if (operational_mode) {
			if (netif_running (dev->net)
				&& !test_bit (EVENT_RX_HALT, &dev->flags)
				&& !dev->StopSummitUrb) {
				rx_submit (dev, urb, GFP_ATOMIC);
				return;
			}

		} else {

			if (netif_running (dev->net)
				&& !test_bit (EVENT_RX_HALT, &dev->flags)) {
				rx_submit (dev, urb, GFP_ATOMIC);
				return;
			}
		}
		//return this urb to rx urb pool
		smscusbnet_return_rx_urb(dev, urb);
	}
	if (netif_msg_rx_err (dev))
		devdbg (dev, "no read resubmitted");
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void intr_complete (struct urb *urb, struct pt_regs *regs)
#else
static void intr_complete (struct urb *urb)
#endif

{
	struct usbnet	*dev = urb->context;
	int		status = urb->status;

	switch (status) {
	    /* success */
	    case 0:
		dev->driver_info->status(dev, urb);
		break;

	    /* software-driven interface shutdown */
	    case -ENOENT:		// urb killed
	    case -ESHUTDOWN:		// hardware gone
		if (netif_msg_ifdown (dev))
			devdbg (dev,"intr shutdown, code %d", status);
		return;

	    // we get controller i/o faults during khubd disconnect() delays.
        // throttle down resubmits, to avoid log floods; just temporarily,
        // so we still recover when the fault isn't a khubd delay.
        /* We need throttling here like RX/TX, because flood events occur on slow ppc platform
         * when the device is being disconnected.
         */
        case -EPROTO:       // ehci
        case -ETIMEDOUT:        // ohci
        case -EILSEQ:       // uhci

        devdbg (dev,"in intr_complete,case -EPROTO\n");
        if (!timer_pending (&dev->delay)) {
            mod_timer (&dev->delay, jiffies + THROTTLE_JIFFIES);
            if (netif_msg_link (dev))
                devdbg (dev, "intr throttle %d", status);
        }
        dev->intr_urb_delay_submit = TRUE;
        return;

	    default:
		devdbg (dev,"intr status %d", status);
		break;
	}

	if (!netif_running (dev->net)){
		return;
    }

	memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status != 0 && netif_msg_timer (dev))
		deverr(dev, "intr resubmit --> %d", status);
}

/*-------------------------------------------------------------------------*/

// unlink pending rx/tx; completion handlers do all other cleanup

static int unlink_urbs (struct usbnet *dev, struct sk_buff_head *q)
{
	unsigned long		flags;
	struct sk_buff		*skb, *skbnext;
	int			count = 0;

	spin_lock_irqsave (&q->lock, flags);
	for (skb = q->next; skb != (struct sk_buff *) q; skb = skbnext) {
		struct skb_data		*entry;
		struct urb		*urb;
		int			retval;

		entry = (struct skb_data *) skb->cb;
		urb = entry->urb;
		skbnext = skb->next;

		// during some PM-driven resume scenarios,
		// these (async) unlinks complete immediately
		retval = usb_unlink_urb (urb);
		if (retval != -EINPROGRESS && retval != 0)
			devdbg (dev, "unlink urb err, %d", retval);
		else
			count++;
	}
	spin_unlock_irqrestore (&q->lock, flags);
	return count;
}

/*-------------------------------------------------------------------------*/

// tasklet (work deferred from completions, in_irq) or timer

static void  smscusbnet_bh (unsigned long param)
{
	struct usbnet		*dev = (struct usbnet *) param;
	struct sk_buff		*skb = NULL;
	struct skb_data		*entry = NULL;

	while ((skb = skb_dequeue (&dev->done))) {
		entry = (struct skb_data *) skb->cb;
		switch (entry->state) {
		    case rx_done:
			entry->state = rx_cleanup;
			rx_process (dev, skb);
			continue;
		    case tx_done:
			usb_free_urb (entry->urb);
			dev_kfree_skb (skb);

			if(dev->device_id == DEV_SMSC7500){
				if(!dev->txq.qlen){
					skb = skb_dequeue(&dev->tx_pending_q);
					if(skb){
						smscusbnet_xmit(skb, dev->net);
					}
				}
			}else{//DEV_SMSC9500
				unsigned long		flags;
				spin_lock_irqsave (&dev->txq.lock, flags);
				if(!dev->txq.qlen)
					smscusbnet_bundle_skb_ximt(dev, &dev->tx_pending_q);
				spin_unlock_irqrestore (&dev->txq.lock, flags);
			}

			continue;
		    case rx_cleanup:
			if(entry->urb)smscusbnet_return_rx_urb(dev, entry->urb);
			dev_kfree_skb (skb);
			continue;
		    default:
			devdbg (dev, "bogus skb state %d", entry->state);
		}
	}

    if (netif_running (dev->net) && netif_device_present (dev->net) && !timer_pending (&dev->delay)) {
        //submit delayed interrupt urb
        if(operational_mode && dev->interrupt && dev->intr_urb_delay_submit){
            int status = usb_submit_urb (dev->interrupt, GFP_ATOMIC);
            dev->intr_urb_delay_submit = FALSE;
            if (status != 0 && netif_msg_timer (dev))
                deverr(dev, "intr resubmit --> %d", status);

        }
    }

	// waiting for all pending urbs to complete?
	if (dev->wait) {
		if ((dev->txq.qlen + dev->rxq.qlen + dev->done.qlen) == 0) {
			wake_up (dev->wait);
		}

	// or are we maybe short a few urbs?
	} else if (netif_running (dev->net)
			&& netif_device_present (dev->net)
			&& !timer_pending (&dev->delay)
			&& !test_bit (EVENT_RX_HALT, &dev->flags)) {

		if (((operational_mode)&&(!dev->StopSummitUrb)) ||(!(operational_mode) ))  {
			int	temp = dev->rxq.qlen;
			int	qlen = RX_QLEN (dev);

			if (temp < qlen) {
				struct urb	*urb;
				int		i;

				// don't refill the queue all at once
				for (i = 0; i < 10 && dev->rxq.qlen < qlen; i++) {
					urb = smscusbnet_get_rx_urb(dev);
					if (urb != NULL) {
						rx_submit (dev, urb, GFP_ATOMIC);
					}
				}

				//if (temp != dev->rxq.qlen && netif_msg_link (dev))
					//devdbg (dev, "in dev->bh, rxqlen %d --> %d",
					//		temp, dev->rxq.qlen);
				if (dev->rxq.qlen < qlen)
					tasklet_schedule (&dev->bh);
			}

		}
		if(!dev->tx_hold_on_completion){
			if (dev->txq.qlen < TX_QLEN (dev))
				netif_wake_queue (dev->net);
		}
	}
}

/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

int smscusbnet_stop (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	int			temp;

	DECLARE_WAIT_QUEUE_HEAD (unlink_wakeup);
	DECLARE_WAITQUEUE (wait, current);

	netif_stop_queue (net);
	if (netif_msg_ifdown (dev))
		devinfo (dev, "stop stats: rx/tx %ld/%ld, errs %ld/%ld",
			dev->stats.rx_packets, dev->stats.tx_packets,
			dev->stats.rx_errors, dev->stats.tx_errors
			);

	if(!skb_queue_empty(&dev->tx_pending_q)){
		skb_queue_purge(&dev->tx_pending_q);
	}
	// ensure there are no more active urbs
	add_wait_queue (&unlink_wakeup, &wait);
	dev->wait = &unlink_wakeup;
	temp = unlink_urbs (dev, &dev->txq) + unlink_urbs (dev, &dev->rxq);
	// maybe wait for deletions to finish.
	while (!skb_queue_empty(&dev->rxq) &&
	       !skb_queue_empty(&dev->txq) &&
	       !skb_queue_empty(&dev->done)) {
		msleep(UNLINK_TIMEOUT_MS);
		if (netif_msg_ifdown (dev))
			devdbg (dev, "waited for %d urb completions", temp);
	}
	dev->wait = NULL;
	remove_wait_queue (&unlink_wakeup, &wait);
	if(operational_mode){
		usb_kill_urb(dev->interrupt);
	}
	devdbg (dev, "smscusbnet_stop, rx_urb_pool_head = %d, rx_urb_pool_tail = %d", dev->rx_urb_pool_head, dev->rx_urb_pool_tail);

	/* deferred work (task, timer, softirq) must also stop.
	 * can't flush_scheduled_work() until we drop rtnl (later),
	 * else workers could deadlock; so make workers a NOP.
	 */
	dev->flags = 0;
	//Clean up the counters
	memset(&dev->stats, 0, sizeof(dev->stats));
	memset(&dev->extra_error_cnts, 0, sizeof(dev->extra_error_cnts));

	dev->StopLinkPolling=TRUE;
	del_timer_sync (&dev->LinkPollingTimer);
	del_timer_sync (&dev->delay);
	tasklet_kill (&dev->bh);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
    //if thera is in suspend0 status already, resume it first. So we can set suspend2 in Smsc9500_suspend();
    down(&dev->pm_mutex);
    if(!dev->pmLock){
        dev->pmLock = TRUE;
        usb_autopm_get_interface(dev->uintf);
    }
    up(&dev->pm_mutex);

    msleep(100);

    down(&dev->pm_mutex);
    if(dev->pmLock){
        usb_autopm_put_interface(dev->uintf);
    }
    dev->pmLock = FALSE;
    up(&dev->pm_mutex);
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

// posts reads, and enables write queuing

// precondition: never called in_interrupt

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static
#endif
int smscusbnet_open (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	int			retval = 0;
	struct driver_info	*info = dev->driver_info;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
    down(&dev->pm_mutex);
    dev->pmLock = TRUE;
    usb_autopm_get_interface(dev->uintf);
    up(&dev->pm_mutex);
#endif

	dev->delay.function = smscusbnet_bh;
	dev->delay.data = (unsigned long) dev;
	init_timer (&dev->delay);

	dev->suspendFlag = 0;

	// put into "known safe" state
	if (info->reset && (retval = info->reset (dev)) < 0) {
	devdbg (dev,"info->reset && (retval = info->reset (dev)) \n");
		if (netif_msg_ifup (dev))
			devinfo (dev,
				"open reset fail (%d) usbnet usb-%s-%s, %s",
				retval,
				dev->udev->bus->bus_name, dev->udev->devpath,
			info->description);
		goto done;
	}

    dev->intr_urb_delay_submit = FALSE;
	init_timer(&(dev->LinkPollingTimer));
	dev->StopLinkPolling=FALSE;
	dev->LinkPollingTimer.function=smscusbnet_linkpolling;
	dev->LinkPollingTimer.data=(unsigned long) dev;
	dev->LinkPollingTimer.expires=jiffies+HZ;
	add_timer(&(dev->LinkPollingTimer));

	// insist peer be connected
	if (info->check_connect && (retval = info->check_connect (dev)) < 0) {
		if (netif_msg_ifup (dev))
			devdbg (dev, "can't open; %d", retval);
		goto done;
	}

	/* start any status interrupt transfer */
	if(operational_mode){
		if (dev->interrupt) {
		devdbg (dev,"dev->interrupt \n");
			retval = usb_submit_urb (dev->interrupt, GFP_KERNEL);
			if (retval < 0) {
				if (netif_msg_ifup (dev))
					deverr (dev, "intr submit %d", retval);
				goto done;
			}
		}
	}

	netif_start_queue (net);
	if (netif_msg_ifup (dev)) {
		char	*framing;
		devdbg (dev,"netif_msg_ifup \n");
		if (dev->driver_info->flags & FLAG_FRAMING_NC)
			framing = "NetChip";
		else if (dev->driver_info->flags & FLAG_FRAMING_GL)
			framing = "GeneSys";
		else if (dev->driver_info->flags & FLAG_FRAMING_Z)
			framing = "Zaurus";
		else if (dev->driver_info->flags & FLAG_FRAMING_RN)
			framing = "RNDIS";
		else if (dev->driver_info->flags & FLAG_FRAMING_AX)
			framing = "ASIX";
		else
			framing = "simple";

		devinfo (dev, "open: enable queueing "
				"(rx %lx, tx %lx) mtu %d %s framing",
			RX_QLEN (dev), TX_QLEN (dev), dev->net->mtu,
			framing);
	}

	// delay posting reads until we're fully open
	tasklet_schedule (&dev->bh);
done:
	return retval;
}

/*-------------------------------------------------------------------------*/

/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */
 #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
static void kevent (void *data)
{
	struct usbnet		*dev = data;
#else
static void kevent(struct work_struct *work)
{
	struct usbnet		*dev = container_of(work,struct usbnet,kevent);
#endif

	int			status;

	/* usb_clear_halt() needs a thread context */
	if (test_bit (EVENT_TX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->txq);
		status = usb_clear_halt (dev->udev, dev->out);

        //Set recovery flag, the device will be reset in the smsc9500.c
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);

		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_tx_err (dev))
				deverr (dev, "can't clear tx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_TX_HALT, &dev->flags);
			if (status != -ESHUTDOWN){
				netif_wake_queue (dev->net);
            }
		}
	}
	if (test_bit (EVENT_RX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->rxq);
		status = usb_clear_halt (dev->udev, dev->in);

        //Set recovery flag.
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);

		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_rx_err (dev))
				deverr (dev, "can't clear rx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_RX_HALT, &dev->flags);
			tasklet_schedule (&dev->bh);
		}
	}

	/* tasklet could resubmit itself forever if memory is tight */
	if (test_bit (EVENT_RX_MEMORY, &dev->flags)) {
		struct urb	*urb = NULL;

		if (netif_running (dev->net))
			urb = smscusbnet_get_rx_urb(dev);
		else
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
		if (urb != NULL) {
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
			rx_submit (dev, urb, GFP_KERNEL);
			tasklet_schedule (&dev->bh);
		}
	}

	if (dev->flags)
		devdbg (dev, "kevent done, flags = 0x%lx \n",
			dev->flags);
}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
/*-------------------------------------------------------------------------*/

/* Stop all traffic so kernel will suspend our device.
 *
 */
#ifdef CONFIG_PM
static int suspendDevice(struct usbnet *dev)
{
	if(operational_mode){
		usb_kill_urb(dev->interrupt); //stop interrupt urb
	 }else{
		 DECLARE_WAIT_QUEUE_HEAD (unlink_wakeup);
		 DECLARE_WAITQUEUE (wait, current);

		 add_wait_queue (&unlink_wakeup, &wait);
		 dev->wait = &unlink_wakeup;
		 unlink_urbs (dev, &dev->txq);
		 unlink_urbs (dev, &dev->rxq);
		 // maybe wait for deletions to finish.
		 while (!skb_queue_empty(&dev->rxq) &&
				!skb_queue_empty(&dev->txq) &&
				!skb_queue_empty(&dev->done)) {
			 msleep(UNLINK_TIMEOUT_MS);
		 }
		 dev->wait = NULL;
		 remove_wait_queue (&unlink_wakeup, &wait);
	 }

	 dev->StopLinkPolling = TRUE;  //stop accessing registers
	 down(&dev->pm_mutex);
	 if(dev->pmLock){
		 dev->pmLock = FALSE;
		 usb_autopm_put_interface(dev->uintf);
	 }
	 up(&dev->pm_mutex);

	 return 0;
}
#endif //CONFIG_PM
#endif

/*-------------------------------------------------------------------------*/

/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
static void myevent(void *data)
{
	struct usbnet		*dev = data;

#else

static void myevent(struct work_struct *work)
{

	struct usbnet		*dev = container_of(work,struct usbnet,myevent);
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
    if (test_bit (EVENT_IDLE_CHECK, &dev->flags)) {
        clear_bit (EVENT_IDLE_CHECK, &dev->flags);
        //autosuspend_disabled should be enabled by shell cmd "echo auto > /sys/bus/usb/devices/X-XX/power/level"
        if(dev->dynamicSuspend && !dev->udev->autosuspend_disabled){
			if((dev->idleCount >= PM_IDLE_DELAY)/* && (dev->uintf->pm_usage_cnt > 0)*/){

				if(dev->device_id == DEV_SMSC7500){
					dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
				}else{
					if(dev->chipDependFeatures[FEATURE_SUSPEND3]){
						dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
					}else{
						dev->suspendFlag |= AUTOSUSPEND_DYNAMIC;
					}
				}
				suspendDevice(dev);
			}
			if(dev->idleCount < PM_IDLE_DELAY)dev->idleCount++;
        }
    }
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
	if (test_bit (EVENT_LINK_DOWN, &dev->flags)) {
	clear_bit (EVENT_LINK_DOWN, &dev->flags);
		if(dev->smartDetach){
		dev->suspendFlag |= AUTOSUSPEND_DETACH;
		}
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
		else if(dev->linkDownSuspend && !dev->udev->autosuspend_disabled){
		dev->suspendFlag |= AUTOSUSPEND_LINKDOWN;
		suspendDevice(dev);
        }
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
    }
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
    if (test_bit (EVENT_IDLE_RESUME, &dev->flags)) {
        clear_bit (EVENT_IDLE_RESUME, &dev->flags);

        if(dev->dynamicSuspend || dev->linkDownSuspend){
			down(&dev->pm_mutex);
			if(!dev->pmLock){
				dev->pmLock = TRUE;
				usb_autopm_get_interface(dev->uintf);
			}
			up(&dev->pm_mutex);

			if(operational_mode){
				if (dev->interrupt) {
					if(usb_submit_urb (dev->interrupt, GFP_KERNEL) < 0){
						deverr(dev, "intr submit ");
					}
				}
			}
			if( (!timer_pending(&dev->LinkPollingTimer))){
			   dev->LinkPollingTimer.expires=jiffies+HZ;
			   add_timer(&(dev->LinkPollingTimer));
			}
			dev->StopLinkPolling = FALSE;
			netif_wake_queue (dev->net);
        }
    }

#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)) && defined(CONFIG_PM)
	if (test_bit (EVENT_LINK_RESET, &dev->flags)) {
		struct driver_info 	*info = dev->driver_info;
		int			retval = 0;

		clear_bit (EVENT_LINK_RESET, &dev->flags);

        if(info->link_reset && (retval = info->link_reset(dev)) < 0) {
			devinfo(dev, "link reset failed (%d) usbnet usb-%s-%s, %s",
				retval,
				dev->udev->bus->bus_name, dev->udev->devpath,
				info->description);
		}

	}

	if (test_bit (EVENT_SET_MULTICAST, &dev->flags)) {
		struct driver_info 	*info = dev->driver_info;
		int			retval = 0;

		clear_bit (EVENT_SET_MULTICAST, &dev->flags);
		if(info->rx_setmulticastlist&& (retval = info->rx_setmulticastlist(dev)) < 0) {
			devinfo(dev, "Set Multicast failed (%d) usbnet usb-%s-%s, %s",
				retval,
				dev->udev->bus->bus_name, dev->udev->devpath,
				info->description);
		}
	}

	if (dev->flags)
		devdbg (dev, "myevent done, flags = 0x%lx", dev->flags);
}

/*-------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void tx_complete (struct urb *urb, struct pt_regs *regs)
#else
static void tx_complete (struct urb *urb)
#endif

{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct usbnet		*dev = entry->dev;

    dev->idleCount = 0;

	if (urb->status == 0) {
		if(dev->tx_hold_on_completion){
			if (dev->tx_pending_q.qlen < TX_PENDING_LEN)
				netif_wake_queue (dev->net);

			dev->stats.tx_packets += entry->pkt_cnt;
		}else{
			dev->stats.tx_packets++;
		}
		dev->stats.tx_bytes += entry->length;
	} else {

		dev->stats.tx_errors++;

		switch (urb->status) {
		case -EPIPE:
			smscusbnet_defer_kevent (dev, EVENT_TX_HALT);
            dev->extra_error_cnts.tx_epipe++;
			break;

		/* software-driven interface shutdown */
		case -ECONNRESET:		// async unlink
		case -ESHUTDOWN:		// hardware gone
			break;

		// like rx, tx gets controller i/o faults during khubd delays
		// and so it uses the same throttling mechanism.
		case -EPROTO:		// ehci
            dev->extra_error_cnts.tx_eproto++;
            goto _HANDLE;
		case -ETIMEDOUT:	// ohci
            dev->extra_error_cnts.tx_etimeout++;
            goto _HANDLE;
		case -EILSEQ:		// uhci
            dev->extra_error_cnts.tx_eilseq++;
_HANDLE:
			if (!timer_pending (&dev->delay)) {
				mod_timer (&dev->delay,
					jiffies + THROTTLE_JIFFIES);
				if (netif_msg_link (dev))
					devdbg (dev, "tx throttle %d",
							urb->status);
			}
			netif_stop_queue (dev->net);
            //Set recovery flag
            set_bit (EVENT_DEV_RECOVERY, &dev->flags);

			break;
		default:
			if (netif_msg_tx_err (dev))
				devdbg (dev, "tx err %d", entry->urb->status);
			break;
		}
	}

	urb->dev = NULL;
	entry->state = tx_done;
	defer_bh(dev, skb, &dev->txq);
}

/*-------------------------------------------------------------------------*/

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static
#endif
void smscusbnet_tx_timeout (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);

	unlink_urbs (dev, &dev->txq);
	tasklet_schedule (&dev->bh);

	// FIXME: device recovery -- reset?
}

/*-------------------------------------------------------------------------*/
/*txq.lock will be acquired by caller*/
static int smscusbnet_bundle_skb_ximt(struct usbnet *dev, struct sk_buff_head *q)
{
	unsigned long		flags;
	struct sk_buff		*skb = NULL, *skbnext = NULL;
	int			count = 0, pkt_cnt = 0;
	int SkbSize = 0;
	struct skb_data		*entry;
	int			retval = NET_XMIT_SUCCESS;
	struct urb		*urb = NULL;

	if((struct sk_buff *)q == q->next)return 	retval;

	spin_lock_irqsave(&q->lock, flags);

	for(skbnext = q->next; skbnext != (struct sk_buff *) q; skbnext = skbnext->next){
		SkbSize += skbnext->len;
		pkt_cnt++;
	}
	if(SkbSize == 0){
		spin_unlock_irqrestore(&q->lock, flags);
		return retval;
	}

	if(pkt_cnt > 1){
		skb = dev_alloc_skb(SkbSize);

		if (!skb){
			devdbg (dev, "smscusbnet_merge_skb, skb is NULL, CX_SkbSize = %d", SkbSize);
			spin_unlock_irqrestore(&q->lock, flags);
			return retval;
		}
		skb_put(skb, SkbSize);
		count = 0;

		for(skbnext = q->next; skbnext != (struct sk_buff *) q; skbnext = skbnext->next){
			memcpy(skb->data + count, skbnext->data, skbnext->len);
			count += skbnext->len;
		}
		while((skbnext = __skb_dequeue(q)) != NULL){
			kfree_skb(skbnext);
		}

	}else{ //one packet
		skb = __skb_dequeue(q);
	}
	spin_unlock_irqrestore(&q->lock, flags);

	netif_wake_queue (dev->net);

	if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
		if (netif_msg_tx_err (dev))
			devdbg (dev, "no urb");
		retval = NET_XMIT_SUCCESS;
		dev->stats.tx_dropped += pkt_cnt;
		if (skb){
			dev_kfree_skb_any (skb);
			skb = NULL;
		}

		goto drop;
	}else{

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))
		urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

		entry = (struct skb_data *) skb->cb;
		entry->urb = urb;
		entry->dev = dev;
		entry->state = tx_start;
		entry->length = SkbSize;
		entry->pkt_cnt = pkt_cnt;

		usb_fill_bulk_urb (urb, dev->udev, dev->out,
				skb->data, skb->len, tx_complete, skb);
	}

	switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
	case -EPIPE:
		devdbg (dev," in smscusbnet_bundle_skb_ximt,-EPIPE\n");
		netif_stop_queue (dev->net);
		smscusbnet_defer_kevent (dev, EVENT_TX_HALT);
		break;
	default:
		if (netif_msg_tx_err (dev))
			devdbg (dev, "tx: submit urb err %d", retval);
		break;
	case 0:
		dev->net->trans_start = jiffies;
		__skb_queue_tail (&dev->txq, skb);
		if (dev->txq.qlen >= TX_QLEN (dev))
			netif_stop_queue (dev->net);
	}

	if (retval) {
		if (netif_msg_tx_err (dev))
			devdbg (dev, "drop, code %d", retval);
drop:
		retval = NET_XMIT_SUCCESS;

		dev->stats.tx_dropped += pkt_cnt;
		if (skb)
			dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	} else if (netif_msg_tx_queued (dev)) {
		devdbg (dev, "> tx, len %d, type 0x%x",
			SkbSize, skb->protocol);
	}

	return retval;
}

/*-------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
static
#endif
int smscusbnet_start_xmit (struct sk_buff *skb, struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	int			retval = NET_XMIT_SUCCESS;
	struct driver_info	*info = dev->driver_info;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#if defined(CONFIG_PM) && defined(CONFIG_USB_SUSPEND)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
    if(atomic_read(&dev->uintf->pm_usage_cnt)<=0){
#else
    if(dev->uintf->pm_usage_cnt <= 0){
#endif
	netif_stop_queue (net);
        smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
        return NET_XMIT_DROP;
    }
#endif //CONFIG_PM && CONFIG_USB_SUSPEND
#endif

	// some devices want funky USB-level framing, for
	// win32 driver (usually) and/or hardware quirks
	if (info->tx_fixup) {
		skb = info->tx_fixup (dev, skb, GFP_ATOMIC);
		if (!skb) {
			if (netif_msg_tx_err (dev))
				devdbg (dev, "can't tx_fixup skb");
			retval = NET_XMIT_SUCCESS;
			dev->stats.tx_dropped++;
			return NET_XMIT_DROP;
		}
	}

	if(dev->tx_hold_on_completion){

		if(dev->device_id == DEV_SMSC7500){
			if(dev->txq.qlen != 0){

				skb_queue_tail (&dev->tx_pending_q, skb);
				if (dev->tx_pending_q.qlen >= TX_PENDING_LEN){
					netif_stop_queue (net);
				}
				return retval;
			}else{

				if(dev->tx_pending_q.qlen != 0){
					skb_queue_tail (&dev->tx_pending_q, skb);
					skb = skb_dequeue(&dev->tx_pending_q);
					if(!skb)return retval;
				}
			}
			retval = smscusbnet_xmit(skb, net);
		}else{
			unsigned long		flags;
			skb_queue_tail (&dev->tx_pending_q, skb);

			spin_lock_irqsave (&dev->txq.lock, flags);
			if(dev->txq.qlen != 0){
				if (dev->tx_pending_q.qlen >= TX_PENDING_LEN){
					netif_stop_queue (net);
				}
			}else{
				retval = smscusbnet_bundle_skb_ximt(dev, &dev->tx_pending_q);
			}
			spin_unlock_irqrestore (&dev->txq.lock, flags);
		}
	}else{

		retval = smscusbnet_xmit(skb, net);
	}

	return retval;
}

static int smscusbnet_xmit (struct sk_buff *skb, struct net_device *net)
{
	int			retval = NET_XMIT_SUCCESS;
	struct usbnet		*dev = netdev_priv(net);
	struct urb		*urb = NULL;
	struct skb_data		*entry;
	unsigned long		flags;

		if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
			if (netif_msg_tx_err (dev))
				devdbg (dev, "no urb");
			goto drop;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))
		urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

		entry = (struct skb_data *) skb->cb;
		entry->urb = urb;
		entry->dev = dev;
		entry->state = tx_start;
		entry->length = skb->len;

		usb_fill_bulk_urb (urb, dev->udev, dev->out,
				skb->data, skb->len, tx_complete, skb);

		/* don't assume the hardware handles USB_ZERO_PACKET
		 * NOTE:  strictly conforming cdc-ether devices should expect
		 * the ZLP here, but ignore the one-byte packet.
		 *
		 * FIXME zero that byte, if it doesn't require a new skb.
		 */
		if ((skb->len % dev->maxpacket) == 0) {
			urb->transfer_buffer_length++;
			if (skb_tailroom(skb)) {
				skb->data[skb->len] = 0;
				__skb_put(skb, 1);
			}
		}
		spin_lock_irqsave (&dev->txq.lock, flags);

		switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
		case -EPIPE:
			devdbg (dev," in tx_fixup,-EPIPE\n");
			netif_stop_queue (net);
			smscusbnet_defer_kevent (dev, EVENT_TX_HALT);
			break;
		default:
			if (netif_msg_tx_err (dev))
				devdbg (dev, "tx: submit urb err %d", retval);
			break;
		case 0:
			net->trans_start = jiffies;
			__skb_queue_tail (&dev->txq, skb);

			if (dev->txq.qlen >= TX_QLEN (dev))
				netif_stop_queue (net);
		}
		spin_unlock_irqrestore (&dev->txq.lock, flags);

		if (retval) {
			devdbg (dev," in tx_fixup,drop\n");
			if (netif_msg_tx_err (dev))
				devdbg (dev, "drop, code %d", retval);
drop:
			retval = NET_XMIT_SUCCESS;
			dev->stats.tx_dropped++;
			if (skb)
				dev_kfree_skb_any (skb);
			usb_free_urb (urb);
		} else if (netif_msg_tx_queued (dev)) {
			devdbg (dev, "> tx, len %d, type 0x%x",
				skb->len, skb->protocol);
		}

		return retval;
}

/*-------------------------------------------------------------------------
 *
 * USB Device Driver support
 *
 *-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

void smscusbnet_disconnect (struct usb_interface *intf)
{
	struct usbnet		*dev = NULL;
	struct usb_device	*xdev = NULL;
	struct net_device	*net = NULL;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	if (!dev)
		return;

	xdev = interface_to_usbdev (intf);

	if (netif_msg_probe (dev))
		devinfo (dev, "unregister '%s' usb-%s-%s, %s",
			intf->dev.driver->name,
			xdev->bus->bus_name, xdev->devpath,
			dev->driver_info->description);

	net = dev->net;
	unregister_netdev (net);

	devdbg (dev, "smscusbnet_disconnect, rx_urb_pool_head = %d, rx_urb_pool_tail = %d", dev->rx_urb_pool_head, dev->rx_urb_pool_tail);

	if(dev->rx_urb_pool){
		while(dev->rx_urb_pool_head != dev->rx_urb_pool_tail){
			if(dev->rx_urb_pool[dev->rx_urb_pool_head])usb_free_urb(dev->rx_urb_pool[dev->rx_urb_pool_head]);
			dev->rx_urb_pool[dev->rx_urb_pool_head] = NULL;
			dev->rx_urb_pool_head = (dev->rx_urb_pool_head + 1) % dev->rx_urb_pool_size;
		}
		if(dev->rx_urb_pool[dev->rx_urb_pool_head]){
			usb_free_urb(dev->rx_urb_pool[dev->rx_urb_pool_head]);
			dev->rx_urb_pool[dev->rx_urb_pool_head] = NULL;
		}
		kfree(dev->rx_urb_pool);
	}

	if(operational_mode && dev->interrupt){
		usb_free_urb(dev->interrupt);
	}
	if(dev->interrupt_urb_buffer){
		kfree(dev->interrupt_urb_buffer);
		dev->interrupt_urb_buffer = NULL;
	}

	if (dev->driver_info->unbind) {
		devinfo (dev,"unbind usb\n");
		dev->driver_info->unbind (dev, intf);
	}

	/* we don't hold rtnl here ... */
	flush_scheduled_work ();
	flush_workqueue(dev->MyWorkQueue);
	destroy_workqueue(dev->MyWorkQueue);

	free_netdev(net);
	usb_put_dev (xdev);

}

/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

int
smscusbnet_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct usbnet			*dev = NULL;
	struct net_device 		*net = NULL;
	struct usb_host_interface	*interface = NULL;
	struct driver_info		*info = NULL;
	struct usb_device		*xdev = NULL;
	int				status = 0;
    char version[15];

	info = (struct driver_info *) prod->driver_info;
	if (!info) {
		return -ENODEV;
	}
	xdev = interface_to_usbdev (udev);
	interface = udev->cur_altsetting;

	usb_get_dev (xdev);

	status = -ENOMEM;

    sprintf(version,"%lX.%02lX.%02lX", (DRIVER_VERSION>>16),(DRIVER_VERSION>>8)&0xFF,(DRIVER_VERSION&0xFFUL));
    printk("Driver smscusbnet.ko verison %s, built on %s, %s\n",version, __TIME__, __DATE__);

	// set up our own records
	net = alloc_etherdev(sizeof(*dev));
	if (!net) {
		devdbg (dev,"can't kmalloc dev");
		goto out;
	}
	dev = netdev_priv(net);

    init_MUTEX(&dev->pm_mutex);

	dev->udev = xdev;
    dev->uintf = udev;
	dev->driver_info = info;
	dev->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);
	skb_queue_head_init (&dev->rxq);
	skb_queue_head_init (&dev->txq);
	skb_queue_head_init (&dev->tx_pending_q);
	skb_queue_head_init (&dev->done);
	dev->bh.func = smscusbnet_bh;
	dev->bh.data = (unsigned long) dev;

	dev->idVendor = prod->idVendor;
	dev->idProduct = prod->idProduct;

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
		INIT_WORK (&dev->kevent, kevent,dev);
	#else
		INIT_WORK (&dev->kevent, kevent);
	#endif

    if(operational_mode)devdbg (dev,"Operational mode enabled\n");

	dev->MyWorkQueue=create_singlethread_workqueue("intr_work");
	if (!dev->MyWorkQueue) {
		devdbg (dev,"can't create MyWorkQueue!\n");
		goto out1;
	}
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
	INIT_WORK (&dev->myevent, myevent,dev);
	#else
	INIT_WORK (&dev->myevent, myevent);
	#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
	SET_MODULE_OWNER (net);
#endif
	dev->net = net;
	strcpy (net->name, "usb%d");
	memcpy (net->dev_addr, node_id, sizeof node_id);

	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 */
	net->hard_header_len += EXTRA_HEADER_LEN; //Reserve extra 8 bytes for control word to eliminate memcpy in tx_fixup()
	devdbg (dev, "hard_header_len = %d\n", (int)net->hard_header_len);
	dev->hard_mtu = net->mtu + net->hard_header_len;
#if 0
// dma_supported() is deeply broken on almost all architectures
	// possible with some EHCI controllers
	if (dma_supported (&udev->dev, DMA_64BIT_MASK))
		net->features |= NETIF_F_HIGHDMA;
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
	net->change_mtu = smscusbnet_change_mtu;
	net->get_stats = smscusbnet_get_stats;
	net->hard_start_xmit = smscusbnet_start_xmit;
	net->open = smscusbnet_open;
	net->stop = smscusbnet_stop;
	net->tx_timeout = smscusbnet_tx_timeout;
#else
        net->netdev_ops = &smscusbnet_netdev_ops;
#endif //linux 2.6.29
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;

	// allow device-specific bind/init procedures
	// NOTE net->name still not usable ...
	if (info->bind) {
		status = info->bind (dev, udev);

		if (status < 0)
		{
		devdbg (dev,"info->bind,status<0 \n");
		goto out2;
		}
		// heuristic:  "usb%d" for links we know are two-host,
		// else "eth%d" when there's reasonable doubt.  userspace
		// can rename the link if it knows better.
		if ((dev->driver_info->flags & FLAG_ETHER) != 0
				&& (net->dev_addr [0] & 0x02) == 0)
			{
			strcpy (net->name, "eth%d");
			}

		/* maybe the remote can't receive an Ethernet MTU */
		if (net->mtu > (dev->hard_mtu - net->hard_header_len))
			net->mtu = dev->hard_mtu - net->hard_header_len;
	} else if (!info->in || !info->out)
		{
		status = smscusbnet_get_endpoints (dev, udev);
		}
	else {
		dev->in = usb_rcvbulkpipe (xdev, info->in);
		dev->out = usb_sndbulkpipe (xdev, info->out);
		if (!(info->flags & FLAG_NO_SETINT))
			status = usb_set_interface (xdev,
				interface->desc.bInterfaceNumber,
				interface->desc.bAlternateSetting);
		else
			status = 0;

	}
	if (status >= 0 && dev->status)
	{
		devdbg (dev,"status==0 \n");
		status = init_status (dev, udev);
	}
	if (status < 0)
	{
		devdbg (dev,"status<0 \n");
		goto out3;
	}

	if (!dev->rx_urb_size)
		dev->rx_urb_size = dev->hard_mtu;

	devdbg (dev, "rx_urb_size = %d\n", (int)dev->rx_urb_size);
	dev->StopSummitUrb=1;

	dev->maxpacket = usb_maxpacket (dev->udev, dev->out, 1);
	SET_NETDEV_DEV(net, &udev->dev);
	status = register_netdev (net);

	if (status)
		{
		goto out3;
		}
	if (netif_msg_probe (dev))
		devinfo (dev, "register '%s' at usb-%s-%s, %s, "
				"%02x:%02x:%02x:%02x:%02x:%02x",
			udev->dev.driver->name,
			xdev->bus->bus_name, xdev->devpath,
			dev->driver_info->description,
			net->dev_addr [0], net->dev_addr [1],
			net->dev_addr [2], net->dev_addr [3],
			net->dev_addr [4], net->dev_addr [5]);

	// ok, it's ready to go.
	usb_set_intfdata (udev, dev);

	// start as if the link is up
	netif_device_attach (net);

	return 0;

out3:
	if (info->unbind)
		info->unbind (dev, udev);
out2:
	if (dev->MyWorkQueue) {
		destroy_workqueue(dev->MyWorkQueue);
	}
out1:
	free_netdev(net);
out:
	usb_put_dev(xdev);

	return status;
}

/*-------------------------------------------------------------------------*/

/* FIXME these suspend/resume methods assume non-CDC style
 * devices, with only one interface.
 */

int smscusbnet_FreeQueue (struct usbnet		*dev)
{
	DECLARE_WAIT_QUEUE_HEAD (unlink_wakeup);
	DECLARE_WAITQUEUE (wait, current);

	/* accelerate emptying of the rx and queues, to avoid
	 * having everything error out.
	 */
	// ensure there are no more active urbs
	add_wait_queue (&unlink_wakeup, &wait);
	dev->wait = &unlink_wakeup;
	(void) unlink_urbs (dev, &dev->rxq);
	(void) unlink_urbs (dev, &dev->txq);

	// maybe wait for deletions to finish.
	while (!skb_queue_empty(&dev->rxq) &&
	       !skb_queue_empty(&dev->txq) &&
	       !skb_queue_empty(&dev->done)) {
		msleep(UNLINK_TIMEOUT_MS);
	}
	dev->wait = NULL;
	remove_wait_queue (&unlink_wakeup, &wait);

	return 0;
}

/*-------------------------------------------------------------------------*/

/* FIXME these suspend/resume methods assume non-CDC style
 * devices, with only one interface.
 */
 #ifndef pm_message_t
#define pm_message_t u32
#endif
int smscusbnet_suspend (struct usb_interface *intf, pm_message_t state)
{
	struct usbnet		*dev = usb_get_intfdata(intf);

	/* accelerate emptying of the rx and queues, to avoid
	 * having everything error out.
	 */
	netif_device_detach (dev->net);

	(void) unlink_urbs (dev, &dev->txq);
	(void) unlink_urbs (dev, &dev->rxq);

	return 0;
}

int smscusbnet_resume (struct usb_interface *intf)
{
	struct usbnet		*dev = usb_get_intfdata(intf);
	netif_device_attach (dev->net);
	tasklet_schedule (&dev->bh);
	return 0;
}

EXPORT_SYMBOL_GPL(smscusbnet_IsOperationalMode);
EXPORT_SYMBOL_GPL(smscusbnet_get_endpoints);
EXPORT_SYMBOL_GPL(smscusbnet_skb_return);
EXPORT_SYMBOL_GPL(smscusbnet_defer_kevent);
EXPORT_SYMBOL_GPL(smscusbnet_defer_myevent);
EXPORT_SYMBOL_GPL(smscusbnet_disconnect);
EXPORT_SYMBOL_GPL(smscusbnet_probe);
EXPORT_SYMBOL_GPL(smscusbnet_FreeQueue);
EXPORT_SYMBOL_GPL(smscusbnet_linkpolling);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
EXPORT_SYMBOL(smscusbnet_stop);
EXPORT_SYMBOL(smscusbnet_get_stats);
EXPORT_SYMBOL(smscusbnet_open);
EXPORT_SYMBOL(smscusbnet_start_xmit);
EXPORT_SYMBOL(smscusbnet_tx_timeout);
EXPORT_SYMBOL(smscusbnet_change_mtu);
#endif
/*-------------------------------------------------------------------------*/

static int __init smscusbnet_init(void)
{
	/* compiler should optimize this out */
	BUG_ON (sizeof (((struct sk_buff *)0)->cb)
			< sizeof (struct skb_data));

	random_ether_addr(node_id);
	return 0;
}
module_init(smscusbnet_init);

static void __exit smscusbnet_exit(void)
{
}
module_exit(smscusbnet_exit);

MODULE_AUTHOR("David Brownell");
MODULE_DESCRIPTION("USB network driver framework");
MODULE_LICENSE("GPL");
