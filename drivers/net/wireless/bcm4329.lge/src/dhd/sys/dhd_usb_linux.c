/*
 * DHD Bus Module for USB/Linux
 * Supersedes usb-cdc.c and usb-rndis.c
 *
 * Copyright (C) 2010, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id: dhd_usb_linux.c,v 1.18.4.2.2.5.2.9.20.3.2.8 2010/09/16 10:46:58 Exp $
 */

#include <linux/module.h>
#include <typedefs.h>
#include <osl.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>

#include <bcmutils.h>
#include <bcmendian.h>
#include <bcmdefs.h>
#include <bcmdevs.h>

#include <proto/ethernet.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_proto.h>

#include <dhd_dbg.h>
#include <proto/802.1d.h>
#include "usbrdl.h"
#include <usbstd.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define KERNEL26
#define USB_ALLOC_URB()		usb_alloc_urb(0, GFP_ATOMIC)
#define USB_SUBMIT_URB(urb)	usb_submit_urb(urb, GFP_ATOMIC)
#define USB_UNLINK_URB(urb)	usb_kill_urb(urb)
#define USB_BUFFER_ALLOC(dev, size, mem, dma) \
				usb_buffer_alloc(dev, size, mem, dma)
#define USB_BUFFER_FREE(dev, size, data, dma) \
				usb_buffer_free(dev, size, data, dma)
#define USB_QUEUE_BULK		URB_ZERO_PACKET
#define CALLBACK_ARGS		struct urb *urb, struct pt_regs *regs
#define CONFIGDESC(usb)		(&((usb)->actconfig)->desc)
#define IFPTR(usb, idx)		((usb)->actconfig->interface[idx])
#define IFALTS(usb, idx)	(IFPTR((usb), (idx))->altsetting[0])
#define IFDESC(usb, idx)	IFALTS((usb), (idx)).desc
#define IFEPDESC(usb, idx, ep)	(IFALTS((usb), (idx)).endpoint[ep]).desc
#else /* 2.4 */
#define USB_ALLOC_URB()		usb_alloc_urb(0)
#define USB_SUBMIT_URB(urb)	usb_submit_urb(urb)
#define USB_UNLINK_URB(urb)	usb_unlink_urb(urb)
#define USB_BUFFER_ALLOC(dev, size, mem, dma) \
				kmalloc(size, mem)
#define USB_BUFFER_FREE(dev, size, data, dma) \
				kfree(data)
#define CALLBACK_ARGS		struct urb *urb
#define CONFIGDESC(usb)		((usb)->actconfig)
#define IFPTR(usb, idx)		(&(usb)->actconfig->interface[idx])
#define IFALTS(usb, idx)	((usb)->actconfig->interface[idx].altsetting[0])
#define IFDESC(usb, idx)	IFALTS((usb), (idx))
#define IFEPDESC(usb, idx, ep)	(IFALTS((usb), (idx)).endpoint[ep])
#endif /* 2.4 */

#define RX_QLEN			10	/* rx bulk queue length */
#define TX_QLEN			10	/* tx bulk queue length */
#define TRIES 			2	/* # of tries for submitting ctrl reads & writes */
#define QLEN			128
#define FCHI		((QLEN) - 10)
#define FCLOW		((QLEN) - ((QLEN)/4))

#define PRIOMASK	7

#define CONTROL_IF		0
#define BULK_IF			0
#define POSTBOOT_ID     0xA123  /* ID to detect if dongle has boot up */

/* Private data kept in skb */
#define SKB_PRIV(skb, idx)	(&((void **)skb->cb)[idx])
#define SKB_PRIV_URB(skb)	(*(struct urb **)SKB_PRIV(skb, 0))

static struct usb_driver dhd_usb;

typedef struct {
	uint32 notification;
	uint32 reserved;
} intr_t;

/* Private data for bus interaction */
typedef struct dhd_bus {
	dhd_pub_t *dhd;
	struct usb_device *usb;	/* USB device pointer from OS */

	struct urb *intr_urb;		/* URB for interrupt endpoint */
	struct urb *urb_freelist;	/* Free list for static URB allocation */
	spinlock_t urb_freelist_lock;	/* Lock for free list */
	uint rx_pipe, tx_pipe, intr_pipe;	/* Pipe numbers for USB I/O */

	struct pktq rxq;	/* Preload queue for receive */
	spinlock_t rxlock;	/* Lock for rxq management */

	struct pktq txq;	/* Queue length used for flow-controle */
	spinlock_t txlock;	/* Lock for txq management */

	uint8		flowcontrol;	/* per prio flow control bitmask */
	bool		dpc_sched; /* Indicates DPC schedule (intrpt rcvd) */
	int intr_size;		/* Size of interrupt message */
	int interval;		/* Interrupt polling interval */
	intr_t intr;		/* Data buffer for interrupt endpoint */
	spinlock_t lock;	/* Lock for intr buffer */
	wait_queue_head_t intr_wait; /* To block awaiting interrupt input */

	uint32 ramsize;
	uint32 rambase;
} dhd_bus_t;

/* Private context info, maintained in skb */
typedef struct {
	dhd_bus_t *bus;
	struct urb *urb;
} dhdusb_ctx_t;

/* IOVar table */
enum {
	IOV_SET_DOWNLOAD_STATE = 1,
	IOV_MEMBYTES,
	IOV_VARS
};

const bcm_iovar_t dhdusb_iovars[] = {
	{"vars",	IOV_VARS,	0,	IOVT_BUFFER,	0 },
	{"dwnldstate",	IOV_SET_DOWNLOAD_STATE,	0,	IOVT_BOOL,	0 },
	{"membytes",	IOV_MEMBYTES,	0,	IOVT_BUFFER,	2 * sizeof(int) },
	{NULL, 0, 0, 0, 0 }
};

static void dhdusb_rx_complete(CALLBACK_ARGS);
static bool dhdusb_sendfromq(dhd_bus_t *bus);

static int dhdusb_doiovar(dhd_bus_t *bus, const bcm_iovar_t *vi, uint32 actionid, const char *name,
                void *params, int plen, void *arg, int len, int val_size);

static int dhdusb_downloadvars(dhd_bus_t *bus, void *arg, int len);

/* URB allocation routines (may not assume bus->dhd is available) */

static int dhdusb_membytes(dhd_bus_t *bus, int read_write, uint32 address, uint8 *data, uint size);

static struct urb *
dhdusb_urb_get(dhd_bus_t *bus)
{
	unsigned long flags;
	struct urb *urb;

	spin_lock_irqsave(&bus->urb_freelist_lock, flags);

	if ((urb = bus->urb_freelist) != NULL)
		bus->urb_freelist = urb->context;		/* context is next pointer */

	spin_unlock_irqrestore(&bus->urb_freelist_lock, flags);

	return urb;
}

static void
dhdusb_urb_put(dhd_bus_t *bus, struct urb *urb)
{
	unsigned long flags;

	spin_lock_irqsave(&bus->urb_freelist_lock, flags);

	urb->context = bus->urb_freelist;			/* context is next pointer */
	bus->urb_freelist = urb;

	spin_unlock_irqrestore(&bus->urb_freelist_lock, flags);
}

static int
dhdusb_urb_alloc(dhd_bus_t *bus)
{
	int i;

	if (!(bus->intr_urb = USB_ALLOC_URB())) {
		DHD_ERROR(("%s: usb_alloc_urb (tx) failed\n", __FUNCTION__));
		return -ENOMEM;
	}

	for (i = 0; i < TX_QLEN + RX_QLEN; i++) {
		struct urb *urb;

		if (!(urb = USB_ALLOC_URB())) {
			DHD_ERROR(("%s: usb_alloc_urb failed\n", __FUNCTION__));
			return -ENOMEM;
		}

		dhdusb_urb_put(bus, urb);
	}

	return 0;
}

static void
dhdusb_urb_free(dhd_bus_t *bus)		/* Don't call until all URBs unlinked */
{
	struct urb *urb;

	if (bus->intr_urb) {
		usb_free_urb(bus->intr_urb);
		bus->intr_urb = NULL;
	}

	while ((urb = dhdusb_urb_get(bus)) != NULL)
		usb_free_urb(urb);
}

static void
dhdusb_tx_complete(CALLBACK_ARGS)
{
	void *pktbuf = urb->context;
#ifdef PROP_TXSTATUS
	struct dhd_bus *bus = (struct dhd_bus *)
		((dhd_pkttag_t*)PKTTAG(pktbuf))->bus_specific.usb.bus;
#else
	struct dhd_bus *bus = ((dhdusb_ctx_t *)PKTTAG(pktbuf))->bus;
#endif
	struct dhd_pub *dhd = bus->dhd;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (urb->status) {
		dhd->tx_errors++;
		DHD_ERROR(("%s: tx error %d\n", dhd_ifname(dhd, 0), urb->status));
	} else
		dhd->dstats.tx_bytes += PKTLEN(dhd->osh, pktbuf);

	dhdusb_urb_put(bus, urb);

	if (!bus->dpc_sched) {
		bus->dpc_sched = TRUE;
		dhd_sched_dpc(bus->dhd);
	}

	PKTFREE(dhd->osh, pktbuf, TRUE);
	/* Adjust flow-control */
	if (dhd->busstate == DHD_BUS_DATA && dhd->txoff && (pktq_len(&bus->txq) < FCLOW))
		dhd_txflowcontrol(dhd, 0, OFF);
}

static bool
dhdusb_sendfromq(dhd_bus_t *bus)
{
	dhd_pub_t *dhd = bus->dhd;
	struct urb *urb;
	dhdusb_ctx_t *ctx;
	uint8 tx_prec_map = 0;
	int prec_out;
	int ret_urb = 0;
	void *pkt;
	unsigned long flags;
	bool resched = TRUE;	  /* Flag indicating resched wanted */

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	do
	{
		/* Allocate URB */
		if (!(urb = dhdusb_urb_get(bus)))
		{
			resched = FALSE;
			break;
		}

		dhd_os_sdlock_txq(bus->dhd);
		tx_prec_map = ~bus->flowcontrol;
		if ((pkt = pktq_mdeq(&bus->txq, tx_prec_map, &prec_out)) == NULL)
		{
			dhd_os_sdunlock_txq(bus->dhd);
			dhdusb_urb_put(bus, urb);
			resched = FALSE;
			break;
		}
		dhd_os_sdunlock_txq(bus->dhd);

		/* Save context in packet tag (URB and DHD bus) */
#ifdef PROP_TXSTATUS
		ctx = (dhdusb_ctx_t *)
		&(((dhd_pkttag_t*)PKTTAG(pkt))->bus_specific.usb);
#else
		ctx = (dhdusb_ctx_t *)PKTTAG(pkt);
#endif
		ctx->urb = urb;
		ctx->bus = bus;
		/* Prepare the URB */
		usb_fill_bulk_urb(urb, bus->usb, bus->tx_pipe, PKTDATA(osh, pkt),
		       PKTLEN(osh, pkt), (usb_complete_t)dhdusb_tx_complete, pkt);
		urb->transfer_flags |= USB_QUEUE_BULK;
		spin_lock_irqsave(&bus->txlock, flags);
		if ((ret_urb = USB_SUBMIT_URB(urb)))
		{
			DHD_ERROR(("%s: usb_submit_urb tx failed-status %d\n", dhd_ifname(dhd, 0),
			ret_urb));
			spin_unlock_irqrestore(&bus->txlock, flags);
			dhdusb_urb_put(bus, urb);
			PKTFREE(dhd->osh, pkt, TRUE);
			dhd->dstats.tx_dropped++;
			break;
		}
		spin_unlock_irqrestore(&bus->txlock, flags);
	} while (TRUE);

	bus->dpc_sched = resched;
	return resched;
}

static bool
dhdusb_dpc(dhd_bus_t *bus)
{
	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	return dhdusb_sendfromq(bus);
}


int
dhd_bus_txdata(struct dhd_bus *bus, void *pktbuf)
{
	dhd_pub_t *dhd = bus->dhd;
	uint prec;
	osl_t *osh;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	osh = bus->dhd->osh;
	if (bus->dhd->busstate != DHD_BUS_DATA) {
		DHD_TRACE(("%s: bus down\n", __FUNCTION__));
		PKTFREE(dhd->osh, pktbuf, TRUE);
		dhd->dstats.tx_dropped++;
		return -EIO;
	}

	prec = PRIO2PREC((PKTPRIO(pktbuf) & PRIOMASK));

	/* Priority based enq */
	dhd_os_sdlock_txq(bus->dhd);
	if (dhd_prec_enq(dhd, &bus->txq, pktbuf, prec >> 1) == FALSE)
	{
		printk(" pkt not getting queued - len %d\n", pktq_len(&bus->txq));
		PKTFREE(osh, pktbuf, TRUE);
	}
	dhd_os_sdunlock_txq(bus->dhd);

	/* Schedule DPC if needed to send queued packet(s) */
	if (!bus->dpc_sched)
	{
		bus->dpc_sched = TRUE;
		dhd_sched_dpc(bus->dhd);
	}

	/* Flow control */
	if ((pktq_len(&bus->txq) >= FCHI))
		dhd_txflowcontrol(bus->dhd, 0, ON);

	/* We've eaten the skb */
	return 0;
}

static int
dhdusb_rx_submit(dhd_bus_t *bus)
{
	dhd_pub_t *dhd = bus->dhd;
	int len = dhd->rxsz;
	struct urb *urb;
	void *pktbuf;
	int ret = 0;
	unsigned long flags;
	dhdusb_ctx_t *ctx;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(OSL_PKTTAG_SZ >= sizeof(dhdusb_ctx_t));

	spin_lock_irqsave(&bus->rxlock, flags);

	/* Refill rx queue up to RX_QLEN */
	while (pktq_len(&bus->rxq) < RX_QLEN) {
		/* Allocate URB for this packet */
		if (!(urb = dhdusb_urb_get(bus))) {
			DHD_ERROR(("%s: out of URBs for rx\n", dhd_ifname(dhd, 0)));
			ret = -ENOMEM;
			break;
		}

		/* Allocate a packet buffer */
		if (!(pktbuf = PKTGET(dhd->osh, len, FALSE))) {
			DHD_ERROR(("%s: PKTGET (rx) failed\n", dhd_ifname(dhd, 0)));
			dhdusb_urb_put(bus, urb);
			ret = -ENOMEM;
			break;
		}

		/* Save context (URB and DHD bus) */
#ifdef PROP_TXSTATUS
		ctx = (dhdusb_ctx_t *)
		&(((dhd_pkttag_t*)PKTTAG(pktbuf))->bus_specific.usb);
#else
		ctx = (dhdusb_ctx_t *)PKTTAG(pktbuf);
#endif
		ctx->urb = urb;
		ctx->bus = bus;

		/* Prepare the URB */
		usb_fill_bulk_urb(urb, bus->usb, bus->rx_pipe, PKTDATA(dhd->osh, pktbuf),
		                  PKTLEN(dhd->osh, pktbuf),
		                  (usb_complete_t)dhdusb_rx_complete, pktbuf);
		urb->transfer_flags |= USB_QUEUE_BULK;

		if ((ret = USB_SUBMIT_URB(urb))) {
			DHD_ERROR(("%s: usb_submit_urb rx failed with status %d\n",
			           dhd_ifname(dhd, 0), ret));
			dhdusb_urb_put(bus, urb);
			PKTFREE(dhd->osh, pktbuf, FALSE);
			break;
		}

		/* Enqueue packet */
		pktq_penq(&bus->rxq, 0, pktbuf);
	}

	spin_unlock_irqrestore(&bus->rxlock, flags);

	return ret;
}

static void
dhdusb_rx_complete(CALLBACK_ARGS)
{
	void *pktbuf = urb->context;
#ifdef PROP_TXSTATUS
	dhd_bus_t* bus = (dhd_bus_t *)
		((dhd_pkttag_t*)PKTTAG(pktbuf))->bus_specific.usb.bus;
#else
	dhd_bus_t *bus = ((dhdusb_ctx_t *)PKTTAG(pktbuf))->bus;
#endif
	dhd_pub_t *dhd = bus->dhd;
	unsigned long flags;
	uint8 prevflowctrlval;
	int ifidx = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	/* Remove from queue */
	spin_lock_irqsave(&bus->rxlock, flags);
	pktq_pdel(&bus->rxq, pktbuf, 0);
	spin_unlock_irqrestore(&bus->rxlock, flags);

	/* Handle errors */
	if (urb->status) {
		/*
		 * Linux 2.4 disconnect: -ENOENT or -EILSEQ for CRC error; rmmod: -ENOENT
		 * Linux 2.6 disconnect: -EPROTO, rmmod: -ESHUTDOWN
		 */
		if (urb->status == -ENOENT || urb->status == -ESHUTDOWN || urb->status == -EPROTO)
			dhd->busstate = DHD_BUS_DOWN;
		else {
			DHD_ERROR(("%s: rx error %d\n", dhd_ifname(bus->dhd, 0), urb->status));
			dhd->rx_errors++;
		}
		PKTFREE(dhd->osh, pktbuf, FALSE);
		dhdusb_urb_put(bus, urb);
		/* On error, don't submit more URBs yet */
		return;
	}

	/* Make the skb represent the received urb */
	PKTSETLEN(dhd->osh, pktbuf, urb->actual_length);

	/* If the protocol uses a data header, check and remove it */
	prevflowctrlval = bus->flowcontrol;
	if (dhd_proto_fcinfo(dhd, pktbuf, &bus->flowcontrol)) {
		PKTFREE(dhd->osh, pktbuf, FALSE);
		if (prevflowctrlval & ~bus->flowcontrol) {
			if (!bus->dpc_sched) {
				bus->dpc_sched = TRUE;
				dhd_sched_dpc(bus->dhd);
			}
		}
		dhd->fc_packets++;
		goto done;
	}

	if (prevflowctrlval & ~bus->flowcontrol) {
		if (!bus->dpc_sched) {
			bus->dpc_sched = TRUE;
			dhd_sched_dpc(bus->dhd);
		}
	}

	if (dhd_prot_hdrpull(dhd, &ifidx, pktbuf) != 0) {
		DHD_ERROR(("%s: rx protocol error\n", dhd_ifname(bus->dhd, ifidx)));
		PKTFREE(dhd->osh, pktbuf, FALSE);
		dhd->rx_errors++;
		goto done;
	}

	/* All ok -- call to deliver up the stack */
	dhd_rx_frame(dhd, ifidx, pktbuf, 1);

done:
	dhdusb_urb_put(bus, urb);

	if (dhd->busstate == DHD_BUS_DATA)
		(void)dhdusb_rx_submit(bus);
}

int
dhd_bus_txctl(struct dhd_bus *bus, uchar *msg, uint msglen)
{
	int ret = 0, try;
	unsigned long ifnum;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (bus->dhd->busstate != DHD_BUS_DATA) {
		DHD_CTL(("%s: bus down\n", __FUNCTION__));
		return -EIO;
	}

	ifnum = IFDESC(bus->usb, CONTROL_IF).bInterfaceNumber;

	for (try = 0; try < TRIES; try++) {
		ret = usb_control_msg(bus->usb, usb_sndctrlpipe(bus->usb, 0), 0,
		                      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		                      cpu_to_le16(0), cpu_to_le16(ifnum),
		                      msg, msglen,
		                      HZ / TRIES
				      ); /* CSTYLED */
		if (ret != -ETIMEDOUT)
			break;
	}

	if (ret < 0) {
		DHD_ERROR(("%s: %s: usb_control_msg failed with status %d\n",
		           dhd_ifname(bus->dhd, 0), __FUNCTION__, ret));
		bus->dhd->tx_ctlerrs++;
		return ret;
	}

#ifdef KERNEL26
	if (((bus->dhd->busstate == DHD_BUS_DATA) &&
	     (ret = USB_SUBMIT_URB(bus->intr_urb))))
		DHD_ERROR(("%s: intr usb_submit_urb (intr) failed with status %d\n",
		           dhd_ifname(bus->dhd, 0), ret));
#endif /* KERNEL26 */

	bus->dhd->tx_ctlpkts++;
	return 0;
}

#define INTERRUPT_TIMEOUT (HZ * 6)
int
dhd_bus_rxctl(struct dhd_bus *bus, uchar *msg, uint msglen)
{
	DECLARE_WAITQUEUE(wait, current);
	int timeout = INTERRUPT_TIMEOUT, ret = 0, try;
	u32 intr;
	unsigned long flags;
	unsigned long ifnum;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ifnum = cpu_to_le16(IFDESC(bus->usb, CONTROL_IF).bInterfaceNumber);

	/* Wait for interrupt */
	add_wait_queue(&bus->intr_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_lock_irqsave(&bus->lock, flags);

		while (!(intr = le32_to_cpu(bus->intr.notification)) &&
		       (!signal_pending(current) && timeout)) {
			spin_unlock_irqrestore(&bus->lock, flags);
			timeout = schedule_timeout(timeout);
			spin_lock_irqsave(&bus->lock, flags);
		}
	bzero(&bus->intr, sizeof(bus->intr));
	spin_unlock_irqrestore(&bus->lock, flags);
	remove_wait_queue(&bus->intr_wait, &wait);
	set_current_state(TASK_RUNNING);

	if (intr)
		DHD_CTL(("%s: resumed on interrupt\n", __FUNCTION__));
	else if (timeout == 0)
		DHD_ERROR(("%s: resumed on timeout\n", __FUNCTION__));
	else if (signal_pending(current)) {
		DHD_ERROR(("%s: cancelled\n", __FUNCTION__));
		return -ERESTARTSYS;
	} else
		DHD_ERROR(("%s: resumed for unknown reason?\n", __FUNCTION__));

	for (try = 0; try < TRIES; try++) {
		/* Get response */
		ret = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0), 1,
		                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		                      cpu_to_le16(0), ifnum, msg, msglen,
		                      HZ / TRIES
				      ); /* CSTYLED */
		if (ret != -ETIMEDOUT)
			break;
	}

	if (ret < 0) {
		DHD_ERROR(("%s: usb_control_msg failed with status %d\n",
		           dhd_ifname(bus->dhd, 0), ret));
		bus->dhd->rx_ctlerrs++;
		return ret;
	}

	bus->dhd->rx_ctlpkts++;
	return ret;
}

static void
dhdusb_intr_complete(CALLBACK_ARGS)
{
	struct dhd_bus *bus = (struct dhd_bus *)urb->context;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));


	if (waitqueue_active(&bus->intr_wait))
		wake_up_interruptible(&bus->intr_wait);
}

static void
dhdusb_unlink(struct pktq *q)
{
	void *pktbuf;
	dhdusb_ctx_t *ctx;

	/* Completion function(s) will move unlinked urbs back to the free list */
	while ((pktbuf = pktq_ppeek(q, 0))) {
#ifdef PROP_TXSTATUS
		ctx = (dhdusb_ctx_t *)
		&(((dhd_pkttag_t*)PKTTAG(pktbuf))->bus_specific.usb);
#else
		ctx = (dhdusb_ctx_t *)PKTTAG(pktbuf);
#endif
		USB_UNLINK_URB(ctx->urb);
	}
}

int
dhd_bus_iovar_op(dhd_pub_t *dhdp, const char *name,
                 void *params, int plen, void *arg, int len, bool set)
{
	dhd_bus_t *bus = dhdp->bus;
	const bcm_iovar_t *vi = NULL;
	int bcmerror = 0;
	int val_size;
	uint32 actionid;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(name);
	ASSERT(len >= 0);

	/* Get MUST have return space */
	ASSERT(set || (arg && len));

	/* Set does NOT take qualifiers */
	ASSERT(!set || (!params && !plen));

	/* Look up var locally */
	if ((vi = bcm_iovar_lookup(dhdusb_iovars, name)) == NULL) {
		/* Not Supported */
		bcmerror = BCME_UNSUPPORTED;
		DHD_TRACE(("%s: IOVAR %s is not supported\n", name, __FUNCTION__));
		goto exit;
	}
	DHD_CTL(("%s: %s %s, len %d plen %d\n", __FUNCTION__,
	         name, (set ? "set" : "get"), len, plen));

	/* set up 'params' pointer in case this is a set command so that
	 * the convenience int and bool code can be common to set and get
	 */
	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		/* all other types are integer sized */
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);
	bcmerror = dhdusb_doiovar(bus, vi, actionid, name, params, plen, arg, len, val_size);

exit:
	return bcmerror;
}

static int
dhdusb_run_cpu(dhd_bus_t *bus)
{
	rdl_state_t rdl;
	bootrom_id_t id;
	int retval = -2;
	int status;
	int attempts = 0;
	/* Check if the CPU is runnable. */
	status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0), DL_GETSTATE,
		UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&rdl, sizeof(rdl_state_t),
		USB_CTRL_EP_TIMEOUT);

	if (status < 0)
		goto err;

	/* check if CPU is runnable? */
	if ((rdl.state == DL_RUNNABLE)) {

		status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0), DL_GO,
			UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&rdl, sizeof(rdl_state_t),
			USB_CTRL_EP_TIMEOUT);

		if (status < 0)
			goto err;

		while (attempts++ < 4) {
			/* Success in running CPU. */
			OSL_DELAY(30*1000); /* Sleep for a while */

			id.chip = 0xDEAD;
			/* For NODISC dongles the chip ID will be POSTBOOT_ID */
			status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0),  DL_GETVER,
				UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&id, sizeof(bootrom_id_t),
				USB_CTRL_EP_TIMEOUT);

			if (status < 0) {
				if (attempts > 4) {
					DHD_ERROR(("command DL_GETVER is not succeeded."
						"Possible re-enumeration\n"));
					/* If DL_GETVER is not success, the dongle might
					 * go for re-enumeration. Which takes longer time
					 */
					retval = 0;
					goto err;
				}
			} else
				break;
		}

		if (id.chip == POSTBOOT_ID) {

			status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0),
				DL_RESETCFG, UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&id,
				sizeof(bootrom_id_t), USB_CTRL_EP_TIMEOUT);

			if (status < 0) {
				DHD_ERROR(("Error in command DL_RESETCFG\n"));
				goto err;
			}
			retval = 0;
		} else {
			/* There is possible re-enumeration */
			retval = 0;
		}

	} else {
		retval = -1;
		DHD_ERROR(("CPU is not RUNNABLE\n"));
		goto err;
	}
err:
	return retval;
}

static int
dhdusb_download_start(dhd_bus_t *bus)
{
	int status;
	int bcmerror = BCME_ERROR;
	bootrom_id_t id;
	rdl_state_t rdl;
	int started = 0, cont = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	/* the IOVAR string "download" is used to reset the ARM core */
	status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0),  DL_GETVER,
		UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&id, sizeof(bootrom_id_t),
		USB_CTRL_EP_TIMEOUT);

	if (status < 0)
	{
		DHD_ERROR((" DL_GETVAR Failed\n"));
		goto exit;
	}
	DHD_TRACE(("ID : Chip 0x%x Rev 0x%x\n", id.chip, id.chiprev));

	/* based on the CHIP Id, store the ram size which is needed for NVRAM download. */
	switch (id.chip) {

		case 0x4319:
			bus->ramsize = RAM_SIZE_4319;
			bus->rambase = RDL_RAM_BASE_4319;
			break;

		case 0x4329:
			bus->ramsize = RAM_SIZE_4329;
			bus->rambase = RDL_RAM_BASE_4329;
			break;
	}

	/* Now we got the Board ID and version.. Put the ARM in download mode. */
	do {
		/* Limit the amount of times we attempt to start  */
		started++;
		bcmerror = BCME_ERROR;

		if (!cont && (started > 5))
			goto exit;

		if (cont) {
			DHD_TRACE(("%d", started));
		} else {
			DHD_TRACE(("start\n"));
		}

		status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0), DL_START,
			UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&rdl, sizeof(rdl_state_t),
			USB_CTRL_EP_TIMEOUT);

		if (status < 0)
			goto exit;

		bcmerror = BCME_OK;

	} while (rdl.state != DL_WAITING);
exit:
	return bcmerror;
}

static int
dhdusb_doiovar(dhd_bus_t *bus, const bcm_iovar_t *vi, uint32 actionid, const char *name,
                void *params, int plen, void *arg, int len, int val_size)
{
	int bcmerror = 0;
	int32 int_val = 0;
	bool bool_val = 0;

	DHD_TRACE(("%s: Enter, action %d name %s params %p plen %d arg %p len %d val_size %d\n",
	           __FUNCTION__, actionid, name, params, plen, arg, len, val_size));

	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid))) != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	bool_val = (int_val != 0) ? TRUE : FALSE;

	switch (actionid) {

	case IOV_SVAL(IOV_MEMBYTES):
	case IOV_GVAL(IOV_MEMBYTES):
	{
		uint32 address;
		uint size, dsize;
		uint8 *data;

		bool set = (actionid == IOV_SVAL(IOV_MEMBYTES));

		ASSERT(plen >= 2*sizeof(int));

		address = (uint32)int_val;
		bcopy((char *)params + sizeof(int_val), &int_val, sizeof(int_val));
		size = (uint)int_val;

		/* Do some validation */
		dsize = set ? plen - (2 * sizeof(int)) : len;
		if (dsize < size) {
			DHD_ERROR(("%s: error on %s membytes, addr 0x%08x size %d dsize %d\n",
			           __FUNCTION__, (set ? "set" : "get"), address, size, dsize));
			bcmerror = BCME_BADARG;
			break;
		}
		DHD_INFO(("%s: Request to %s %d bytes at address 0x%08x\n", __FUNCTION__,
		          (set ? "write" : "read"), size, address));

		/* Generate the actual data pointer */
		data = set ? (uint8*)params + 2 * sizeof(int): (uint8*)arg;

		/* Call to do the transfer */
		bcmerror = dhdusb_membytes(bus, set, address, data, size);
	}
		break;


	case IOV_SVAL(IOV_SET_DOWNLOAD_STATE):

		if (bool_val == TRUE) {
			bcmerror = dhdusb_download_start(bus);
		} else {
			bcmerror = dhdusb_run_cpu(bus);
		}
		break;

	case IOV_SVAL(IOV_VARS):
		bcmerror = dhdusb_downloadvars(bus, arg, len);
		break;

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}

exit:
	return bcmerror;
}

static int
dhdusb_downloadvars(dhd_bus_t *bus, void *arg, int len)
{
	int bcmerror = 0;
	uint32 varsize;
	uint32 varaddr;
	uint32 varsizew;

	if (!len) {
		bcmerror = BCME_BUFTOOSHORT;
		goto err;
	}

	/* Even if there are no vars are to be written, we still need to set the ramsize. */
	varsize = len ? ROUNDUP(len, 4) : 0;
	varaddr = (bus->ramsize - 4) - varsize;

	/* Write the vars list */
	DHD_INFO(("WriteVars: @%x varsize=%d\n", varaddr, varsize));
	bcmerror = dhd_bus_membytes(bus->dhd, TRUE, varaddr, arg, varsize);

	/* adjust to the user specified RAM */
	DHD_INFO(("Usable memory size: %d\n", bus->ramsize));
	DHD_INFO(("Vars are at %d, orig varsize is %d\n", varaddr, varsize));

	varsize = ((bus->ramsize - 4) - varaddr);

	/*
	 * Determine the length token:
	 * Varsize, converted to words, in lower 16-bits, checksum in upper 16-bits.
	 */
	if (bcmerror) {
		varsizew = 0;
	} else {
		varsizew = varsize / 4;
		varsizew = (~varsizew << 16) | (varsizew & 0x0000FFFF);
		varsizew = htol32(varsizew);
	}

	DHD_INFO(("New varsize is %d, length token=0x%08x\n", varsize, varsizew));

	/* Write the length token to the last word */
	bcmerror = dhd_bus_membytes(bus->dhd, TRUE, (bus->ramsize - 4),
		(uint8*)&varsizew, 4);
err:
	return bcmerror;
}

void
dhd_bus_dump(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf)
{
	bcm_bprintf(strbuf, "Bus USB\n");
}

void
dhd_bus_clearcounts(dhd_pub_t *dhdp)
{
}

void
dhd_bus_stop(struct dhd_bus *bus, bool enforce_mutex)
{
	struct dhd_pub *dhd = bus->dhd;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (dhd->busstate == DHD_BUS_DOWN)
		return;

	dhd->busstate = DHD_BUS_DOWN;

	USB_UNLINK_URB(bus->intr_urb);

	dhdusb_unlink(&bus->txq);
	dhdusb_unlink(&bus->rxq);
}

int
dhd_bus_init(dhd_pub_t *dhd, bool enforce_mutex)
{
	dhd_bus_t *bus = dhd->bus;
	int ret = 0;
	int status;
	bootrom_id_t id;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (dhd->busstate == DHD_BUS_DATA)
		return 0;

	status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0),
		DL_GETVER, UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&id,
		sizeof(bootrom_id_t), USB_CTRL_EP_TIMEOUT);

	if (status < 0) {
		DHD_ERROR(("Error in command DL_RESETCFG\n"));
		return status;
	}
	/* Check if the FW is booted up. */
	if (id.chip != POSTBOOT_ID) {
		DHD_TRACE((" FW is not UP\n"));
		return -1;
	}

	usb_fill_int_urb(bus->intr_urb, bus->usb, bus->intr_pipe,
	                 &bus->intr, bus->intr_size,
	                 (usb_complete_t)dhdusb_intr_complete, bus, bus->interval);

#ifndef KERNEL26
	/* kernels prior to 2.6 automatically resubmit the interrupt URB */
	if ((ret = USB_SUBMIT_URB(bus->intr_urb))) {
		DHD_ERROR(("%s: usb_submit_urb failed with status %d\n",
		           dhd_ifname(bus->dhd, 0), ret));
		return ret;
	}
#endif /* !KERNEL26 */

	/* Start receiving data packets (bulk in) */
	if ((ret = dhdusb_rx_submit(bus))) {
		USB_UNLINK_URB(bus->intr_urb);
		dhdusb_unlink(&bus->rxq);
		return ret;
	}

	/* Success, indicate bus is fully up */
	dhd->busstate = DHD_BUS_DATA;

	return 0;
}

bool
dhd_bus_dpc(struct dhd_bus *bus)
{
	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	return dhdusb_dpc(bus);
}

#ifdef KERNEL26
static int
dhdusb_probe(struct usb_interface *intf,
             const struct usb_device_id *id)
#else
static void *
dhdusb_probe(struct usb_device *usb, unsigned int ifnum,
             const struct usb_device_id *id)
#endif
{
	int ep;
	osl_t *osh;
	dhd_bus_t *bus = NULL;
	struct usb_endpoint_descriptor *endpoint;
	int ret = 0;
	dhd_cmn_t *cmn;
#ifdef KERNEL26
	struct usb_device *usb = interface_to_usbdev(intf);
#else
	int claimed = 0;
#endif

	DHD_ERROR(("%s: Enter for Vendor 0x%x, Product 0x%x\n", __FUNCTION__,
	           id->idVendor, id->idProduct));

	/* Ask the OS interface part for an OSL handle */
	if (!(osh = dhd_osl_attach(usb, USB_BUS))) {
		DHD_ERROR(("%s: OSL attach failed\n", __FUNCTION__));
		ret = -ENOMEM;
		goto fail;
	}

	/* Allocate private bus interface state */
	if (!(bus = MALLOC(osh, sizeof(dhd_bus_t)))) {
		DHD_ERROR(("%s: MALLOC failed\n", __FUNCTION__));
		ret = -ENOMEM;
		goto fail;
	}

	memset(bus, 0, sizeof(dhd_bus_t));

#ifdef KERNEL26
	usb_set_intfdata(intf, bus);
#endif

	bus->usb = usb;
	init_waitqueue_head(&bus->intr_wait);
	spin_lock_init(&bus->urb_freelist_lock);
	spin_lock_init(&bus->lock);
	spin_lock_init(&bus->rxlock);
	spin_lock_init(&bus->txlock);
	pktq_init(&bus->rxq, 1, RX_QLEN);
	pktq_init(&bus->txq, (PRIOMASK+1), QLEN);

	/* Default error code while checking device */
	ret = -EIO;

	/* Check that the device supports only one configuration */
	if (usb->descriptor.bNumConfigurations != 1) {
		DHD_ERROR(("%s: invalid number of configurations %d\n",
		           __FUNCTION__, usb->descriptor.bNumConfigurations));
		goto fail;
	}

	if (usb->descriptor.bDeviceClass != USB_CLASS_VENDOR_SPEC) {
		DHD_ERROR(("%s: unsupported device class %d\n",
		           __FUNCTION__, usb->descriptor.bDeviceClass));
		goto fail;
	}

	/*
	 * Only the BDC interface configuration is supported:
	 *	Device class: USB_CLASS_VENDOR_SPEC
	 *	if0 class: USB_CLASS_VENDOR_SPEC
	 *	if0/ep0: control
	 *	if0/ep1: bulk in
	 *	if0/ep2: bulk out (ok if swapped with bulk in)
	 */

	if (CONFIGDESC(usb)->bNumInterfaces != 1) {
		DHD_ERROR(("%s: invalid number of interfaces %d\n",
		           __FUNCTION__, CONFIGDESC(usb)->bNumInterfaces));
		goto fail;
	}

	/* Check interface */

#ifndef KERNEL26
	if (usb_interface_claimed(IFPTR(usb, CONTROL_IF))) {
		DHD_ERROR(("%s: interface already claimed\n",
		           __FUNCTION__));
		goto fail;
	}
#endif

	if (IFDESC(usb, CONTROL_IF).bInterfaceClass != USB_CLASS_VENDOR_SPEC ||
	    IFDESC(usb, CONTROL_IF).bInterfaceSubClass != 2 ||
	    IFDESC(usb, CONTROL_IF).bInterfaceProtocol != 0xff) {
		DHD_ERROR(("%s: invalid control interface: class %d, subclass %d, proto %d\n",
		           __FUNCTION__,
		           IFDESC(usb, CONTROL_IF).bInterfaceClass,
		           IFDESC(usb, CONTROL_IF).bInterfaceSubClass,
		           IFDESC(usb, CONTROL_IF).bInterfaceProtocol));
		goto fail;
	}

	/* Check control endpoint */
	endpoint = &IFEPDESC(usb, CONTROL_IF, 0);
	if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT) {
		DHD_ERROR(("%s: invalid control endpoint %d\n",
		           __FUNCTION__, endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK));
		goto fail;
	}

	bus->intr_pipe = usb_rcvintpipe(usb, endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);

#ifndef KERNEL26
	/* Claim interface */
	usb_driver_claim_interface(&dhd_usb, IFPTR(usb, CONTROL_IF), bus);
	claimed = 1;
#endif

	/* Check data endpoints and get pipes */
	for (ep = 1; ep <= 2; ep++) {
		endpoint = &IFEPDESC(usb, BULK_IF, ep);
		if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) !=
		    USB_ENDPOINT_XFER_BULK) {
			DHD_ERROR(("%s: invalid data endpoint %d\n",
			           __FUNCTION__, ep));
			goto fail;
		}

		if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
			bus->rx_pipe = usb_rcvbulkpipe(usb, (endpoint->bEndpointAddress &
			                                     USB_ENDPOINT_NUMBER_MASK));
		else
			bus->tx_pipe = usb_sndbulkpipe(usb, (endpoint->bEndpointAddress &
			                                     USB_ENDPOINT_NUMBER_MASK));
	}

	/* Allocate interrupt URB and data buffer */
	/* RNDIS says 8-byte intr, our old drivers used 4-byte */
	bus->intr_size = (IFEPDESC(usb, CONTROL_IF, 0).wMaxPacketSize == 16) ? 8 : 4;

	bus->interval = IFEPDESC(usb, CONTROL_IF, 0).bInterval;

#ifndef KERNEL26
	/* usb_fill_int_urb does the interval decoding in 2.6 */
	if (usb->speed == USB_SPEED_HIGH)
		bus->interval = 1 << (bus->interval - 1);
#endif

	/* Allocate URBs statically */
	dhdusb_urb_alloc(bus);

	/* attach the common module */
	if (!(cmn = dhd_common_init(osh))) {
		DHD_ERROR(("%s: dhd_common_init failed\n", __FUNCTION__));
		ret = -ENXIO;
		goto fail;
	}

	/* Attach to the dhd/OS interface */
	if (!(bus->dhd = dhd_attach(osh, bus, 0))) {
		DHD_ERROR(("%s: dhd_attach failed\n", __FUNCTION__));
		ret = -ENXIO;
		goto fail;
	}

	bus->dhd->cmn = cmn;
	cmn->dhd = bus->dhd;

	/* Ok, finish the attach to the OS network interface */
	if (dhd_net_attach(bus->dhd, 0) != 0) {
		DHD_ERROR(("%s: dhd_net_attach failed\n", __FUNCTION__));
		ret = -ENXIO;
		goto fail;
	}

	/* Success */
#ifdef KERNEL26
	return 0;
#else
	usb_inc_dev_use(usb);
	return bus;
#endif

fail:
	DHD_ERROR(("%s: failed with errno %d\n", __FUNCTION__, ret));

	/* Release resources in reverse order */
	if (osh) {
		if (bus) {
			if (bus->dhd) {
				dhd_detach(bus->dhd);
				dhd_free(bus->dhd);
				bus->dhd = NULL;
			}

			dhdusb_urb_free(bus);
#ifndef KERNEL26
			if (claimed)
				usb_driver_release_interface(&dhd_usb, IFPTR(usb, CONTROL_IF));
#endif
			MFREE(osh, bus, sizeof(dhd_bus_t));
		}
#ifdef KERNEL26
		usb_set_intfdata(intf, NULL);
#endif
		dhd_osl_detach(osh);
	}

#ifdef KERNEL26
	return ret;
#else
	return NULL;
#endif
}

#ifdef KERNEL26
static void
dhdusb_disconnect(struct usb_interface *intf)
#else
static void
dhdusb_disconnect(struct usb_device *usb, void *ptr)
#endif
{
	osl_t *osh;
#ifdef KERNEL26
	struct usb_device *usb = interface_to_usbdev(intf);
	dhd_bus_t *bus = usb_get_intfdata(intf);
#else
	dhd_bus_t *bus = (dhd_bus_t *)ptr;
#endif

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (bus == NULL || usb == NULL) {
		DHD_ERROR(("%s: null structure bus=%p usb=%p\n",  __FUNCTION__, bus, usb));
		return;
	}

	dhd_bus_stop(bus, TRUE);

	ASSERT(bus->dhd);
	osh = bus->dhd->osh;

	dhd_common_deinit(bus->dhd);

	dhd_detach(bus->dhd);
	dhd_free(bus->dhd);
	bus->dhd = NULL;

	dhdusb_urb_free(bus);

	MFREE(osh, bus, sizeof(dhd_bus_t));
	if (MALLOCED(osh))
		DHD_ERROR(("%s: MEMORY LEAK %d bytes\n", __FUNCTION__, MALLOCED(osh)));

	dhd_osl_detach(osh);

#ifndef KERNEL26
	usb_driver_release_interface(&dhd_usb, IFPTR(usb, CONTROL_IF));
	usb_dec_dev_use(usb);
#endif

	DHD_TRACE(("%s: Disconnected\n", __FUNCTION__));
}


#if defined(BDC)
static struct usb_device_id devid_table[] = {
	{ USB_DEVICE(0x0a5c, 0x0bdc) },
	{ USB_DEVICE(0x0a5c, 0x048f) },
	{ USB_DEVICE(0x0a5c, 0xbd16) },
	{ }
};
#elif defined(CDC)
static struct usb_device_id devid_table[] = {
	{ USB_DEVICE(0x0a5c, 0x0cdc) },
	{ USB_DEVICE(0x0a5c, 0x048f) },
	{ USB_DEVICE(0x0a5c, 0xbd16) },
	{ }
};
#elif defined(RNDIS)
static struct usb_device_id devid_table[] = {
	{ USB_DEVICE(0x0a5c, 0xd11b) },
	{ USB_DEVICE(0x0565, 0x0041) },
	{ USB_DEVICE(0x04f9, 0x01a1) },
	{ }
};
#else
#error No Protocol Specified: No device ID!
#endif /* protocol */

MODULE_DEVICE_TABLE(usb, devid_table);

static struct usb_driver dhd_usb = {
	name:		"dhd-" DHD_PROTOCOL "-linux",
	probe:		dhdusb_probe,
	disconnect:	dhdusb_disconnect,
	id_table:	devid_table
};

int
dhd_bus_register(void)
{
	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	return usb_register(&dhd_usb);
}

void
dhd_bus_unregister(void)
{
	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	usb_deregister(&dhd_usb);
}

bool
dhd_bus_watchdog(dhd_pub_t *dhdp)
{
	return FALSE;
}

extern int
dhd_bus_console_in(dhd_pub_t *dhdp, uchar *msg, uint msglen)
{
	return 0;
}

void
dhd_bus_set_nvram_params(struct dhd_bus * bus, const char *nvram_params)
{
}

void *
dhd_bus_pub(struct dhd_bus *bus)
{
	return bus->dhd;
}

void *
dhd_bus_txq(struct dhd_bus *bus)
{
	return &bus->txq;
}


uint
dhd_bus_hdrlen(struct dhd_bus *bus)
{
	return 0;
}

int
dhd_bus_membytes(dhd_pub_t *dhdp, bool set, uint32 address, uint8 *data, uint size)
{
	dhd_bus_t *bus;
	hwacc_t hwacc;
	int write_bytes = 4;
	int status;
	int retval = 0;
	bus = dhdp->bus;

	DHD_TRACE(("Enter:%s\n", __FUNCTION__));


	/* Read is not supported */
	if (0 == set) {
		DHD_ERROR(("Currently read is not supported!!\n"));
		return -1;
	}

	/* Add RAM base address */
	address += bus->rambase;

	hwacc.cmd = DL_CMD_WRHW;
	hwacc.addr = address;

	DHD_TRACE(("Address:%x size:%d", hwacc.addr, size));
	do {
		if (size >= 4) {
			write_bytes = 4;
		} else if (size >= 2) {
			write_bytes = 2;
		} else {
			write_bytes = 1;
		}

		hwacc.len = write_bytes;

		while (size >= write_bytes) {
			hwacc.data = *((unsigned int*)data);

			status = usb_control_msg(bus->usb, usb_sndctrlpipe(bus->usb, 0), DL_WRHW,
				UT_WRITE_VENDOR_INTERFACE, 1, 0, (char *)&hwacc, sizeof(hwacc_t),
				USB_CTRL_EP_TIMEOUT);

			if (status < 0) {
				retval = -1;
				DHD_ERROR((" Ctrl write hwacc failed w/status %d @ address:%x \n",
					status, hwacc.addr));
				goto err;
			}

			hwacc.addr += write_bytes;
			data += write_bytes;
			size -= write_bytes;
		}
	} while (size > 0);

err:
	return retval;
}

static int
dhdusb_membytes(dhd_bus_t *bus, int read_write, uint32 address, uint8 *data, uint size)
{
	int status = 0;
	int to_send = 0;
	int bytes_sent = 0;
	int total_bytes_sent = 0;
	rdl_state_t rdl;


	/* Read is not supportd. */
	if (0 == read_write) {
		DHD_ERROR(("Currently read is not supported!!\n"));
		return -1;
	}

	/* Write 'size' number of data to address 'address' */
	total_bytes_sent = bytes_sent = status = 0;

	while ((total_bytes_sent != size) && (status >= 0)) {
		if ((size - total_bytes_sent) < RDL_CHUNK) {
			to_send = size - total_bytes_sent;
		}
		else
			to_send = RDL_CHUNK;
		/* ensure we are not an even multiple of 64 for usb 1.1 */
		/* which also covers the usb2.0 case (512) */
		if (!(to_send % 64))
			to_send -= 4;

		/* use standard usb request command */

		DHD_TRACE((" Bulk write  at %x\n", address));

		status = usb_bulk_msg(bus->usb, bus->tx_pipe, (void*)data, to_send, &bytes_sent,
			USB_CTRL_EP_TIMEOUT);

		if (status < 0) {
			DHD_ERROR((" Bulk write failed w/status %d\n", status));
			goto err;
		}
		if (to_send != bytes_sent) {
			DHD_ERROR((" Bulk write not complete! to_send != bytes_sent!!!\n"));
			status = -1;
			goto err;
		}

		address += bytes_sent;

		total_bytes_sent += bytes_sent;
		data += bytes_sent;

		/*  Get the status of write */
		status = usb_control_msg(bus->usb, usb_rcvctrlpipe(bus->usb, 0), DL_GETSTATE,
			UT_READ_VENDOR_INTERFACE, 1, 0, (char*)&rdl, sizeof(rdl_state_t),
			USB_CTRL_EP_TIMEOUT);

		if (status < 0)
			goto err;

		ASSERT(address == rdl.bytes);

		/* restart if an error is reported */
		if ((rdl.state == DL_BAD_HDR) || (rdl.state == DL_BAD_CRC) ||
		    (rdl.state == DL_NVRAM_TOOBIG)) {
		    status = -1;
			DHD_ERROR(("ERROR:DL_GETSTATE %d\n", rdl.state));
			goto err;
		}

		DHD_TRACE((" Bulk write cmplt @%x status=%d\n", rdl.bytes, status));

	}
err:
	return status;
}

#if defined(DONGLEOVERLAYS)
int
dhd_bus_overlay_dl(dhd_pub_t *dhdp, int ifindex, uint8 *overlay, uint osize,
                   int32 idx, uint32 region)
{
	wl_ioctl_overlay_t *obuf;
	int obuflen;
	int offset = 0;
	wl_ioctl_t ovl_ioc;
	int chunksize = OVERLAY_DOWNLOAD_CHUNKSIZE;
	int ret = BCME_OK;

	DHD_TRACE(("Enter %s\n", __FUNCTION__));

	/* pass down overlay code in chunks */
	while (osize) {
		if (osize <= chunksize) {
			chunksize = osize;
			osize = 0;
		} else
			osize -= chunksize;
		/* create the overlay buf */
		obuf = dhd_mkoverlay(dhdp, overlay, idx, offset,
		                     chunksize, &obuflen);
		if (!obuf) {
			DHD_ERROR(("%s: dhd_mkoverlay failed\n", __FUNCTION__));
			return BCME_NOMEM;
		}

		ovl_ioc.cmd = WLC_OVERLAY_IOCTL;
		ovl_ioc.buf = obuf;
		ovl_ioc.len = obuflen;
		ovl_ioc.action = WL_IOCTL_ACTION_SET;

		ret = dhd_prot_ioctl(dhdp, ifindex, &ovl_ioc,
		                     obuf, obuflen);
		MFREE(dhdp->osh, obuf, obuflen);
		if (ret < 0) {
			DHD_ERROR(("%s: OVERLAY_IOCTL failed w/status %d\n",
			           __FUNCTION__, ret));
			return ret;
		}
		offset += chunksize;
	}

	return ret;
}
#endif /* DONGLEOVERLAYS */
