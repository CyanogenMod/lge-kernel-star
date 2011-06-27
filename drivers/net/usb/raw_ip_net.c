/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>

#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

/* ethernet packet ethertype for IP packets */
#define NET_IP_ETHERTYPE		0x08, 0x00

#define	TX_TIMEOUT		10

#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  /* USB_NET_BUFSIZ */

MODULE_LICENSE("GPL");

unsigned long usb_net_raw_ip_vid = 0x1519;
unsigned long usb_net_raw_ip_pid = 0x0020;
unsigned long usb_net_raw_ip_intf = 0x03;
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

module_param(usb_net_raw_ip_vid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_vid, "usb net (raw-ip) - USB VID");
module_param(usb_net_raw_ip_pid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_pid, "usb net (raw-ip) - USB PID");
module_param(usb_net_raw_ip_intf, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_intf, "usb net (raw-ip) - USB interface");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");

struct baseband_usb {
	struct {
		struct usb_driver *driver;
		struct usb_device *device;
		struct usb_interface *interface;
		struct {
			struct {
				unsigned int in;
				unsigned int out;
			} isoch, bulk, interrupt;
		} pipe;
		/* currently active rx urb */
		struct urb *rx_urb;
		/* currently active tx urb */
		struct urb *tx_urb;
	} usb;
};

static struct baseband_usb *baseband_usb_net;

static struct net_device *usb_net_raw_ip_dev;

static unsigned int g_usb_interface_index;
static struct usb_interface *g_usb_interface;

static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	pr_debug("%s(%d) { intf %p id %p\n", __func__, __LINE__, intf, id);

	pr_debug("intf->cur_altsetting->desc.bInterfaceNumber %02x\n",
		intf->cur_altsetting->desc.bInterfaceNumber);
	pr_debug("intf->cur_altsetting->desc.bAlternateSetting %02x\n",
		intf->cur_altsetting->desc.bAlternateSetting);
	pr_debug("intf->cur_altsetting->desc.bNumEndpoints %02x\n",
		intf->cur_altsetting->desc.bNumEndpoints);
	pr_debug("intf->cur_altsetting->desc.bInterfaceClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceSubClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceSubClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceProtocol %02x\n",
		intf->cur_altsetting->desc.bInterfaceProtocol);
	pr_debug("intf->cur_altsetting->desc.iInterface %02x\n",
		intf->cur_altsetting->desc.iInterface);

	if (g_usb_interface_index !=
		intf->cur_altsetting->desc.bInterfaceNumber) {
		pr_debug("%s(%d) } -ENODEV\n", __func__, __LINE__);
		return -ENODEV;
	} else {
		g_usb_interface = intf;
	}

	pr_debug("%s(%d) }\n", __func__, __LINE__);
	return 0;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	pr_debug("%s(%d) { intf %p\n", __func__, __LINE__, intf);
	pr_debug("%s(%d) }\n", __func__, __LINE__);
}

static struct usb_device_id baseband_usb_driver_id_table[2];

static char baseband_usb_driver_name[32];

static struct usb_driver baseband_usb_driver = {
	.name = baseband_usb_driver_name,
	.probe = baseband_usb_driver_probe,
	.disconnect = baseband_usb_driver_disconnect,
	.id_table = baseband_usb_driver_id_table,
};

static void find_usb_pipe(struct baseband_usb *usb)
{
	struct usb_device *usbdev = usb->usb.device;
	struct usb_interface *intf = usb->usb.interface;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_isoc_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous in\n", n);
			usb->usb.pipe.isoch.in = usb_rcvisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_isoc_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous out\n", n);
			usb->usb.pipe.isoch.out = usb_sndisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk in\n", n);
			usb->usb.pipe.bulk.in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk out\n", n);
			usb->usb.pipe.bulk.out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt in\n", n);
			usb->usb.pipe.interrupt.in = usb_rcvintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt out\n", n);
			usb->usb.pipe.interrupt.out = usb_sndintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else {
			pr_debug("endpoint[%d] skipped\n", n);
		}
	}
}

void baseband_usb_close(struct baseband_usb *usb);

struct baseband_usb *baseband_usb_open(unsigned int vid,
	unsigned int pid,
	unsigned int intf)
{
	struct baseband_usb *usb;
	int err;

	pr_debug("baseband_usb_open {\n");

	/* allocate baseband usb structure */
	usb = (struct baseband_usb *) kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb)
		return ((struct baseband_usb *) 0);

	/* open usb driver */
	sprintf(baseband_usb_driver_name,
		"baseband_usb_%x_%x_%x",
		vid, pid, intf);
	baseband_usb_driver_id_table[0].match_flags = USB_DEVICE_ID_MATCH_DEVICE;
	baseband_usb_driver_id_table[0].idVendor = vid;
	baseband_usb_driver_id_table[0].idProduct = pid;
	g_usb_interface_index = intf;
	g_usb_interface = (struct usb_interface *) 0;
	err = usb_register(&baseband_usb_driver);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		goto error_exit;
	}
	usb->usb.driver = &baseband_usb_driver;
	if (!g_usb_interface) {
		pr_err("cannot open usb driver - !g_usb_interface\n");
		goto error_exit;
	}
	usb->usb.device = interface_to_usbdev(g_usb_interface);
	usb->usb.interface = g_usb_interface;
	find_usb_pipe(usb);
	usb->usb.rx_urb = (struct urb *) 0;
	usb->usb.tx_urb = (struct urb *) 0;
	g_usb_interface_index = ~0U;
	g_usb_interface = (struct usb_interface *) 0;
	pr_debug("usb->usb.driver->name %s\n", usb->usb.driver->name);
	pr_debug("usb->usb.device %p\n", usb->usb.device);
	pr_debug("usb->usb.interface %p\n", usb->usb.interface);
	pr_debug("usb->usb.pipe.isoch.in %x\n", usb->usb.pipe.isoch.in);
	pr_debug("usb->usb.pipe.isoch.out %x\n", usb->usb.pipe.isoch.out);
	pr_debug("usb->usb.pipe.bulk.in %x\n", usb->usb.pipe.bulk.in);
	pr_debug("usb->usb.pipe.bulk.out %x\n", usb->usb.pipe.bulk.out);
	pr_debug("usb->usb.pipe.interrupt.in %x\n", usb->usb.pipe.interrupt.in);
	pr_debug("usb->usb.pipe.interrupt.out %x\n", usb->usb.pipe.interrupt.out);

	pr_debug("baseband_usb_open }\n");
	return usb;

error_exit:
	return ((struct baseband_usb *) 0);
}

void baseband_usb_close(struct baseband_usb *usb)
{
	pr_debug("baseband_usb_close {\n");

	/* check input */
	if (!usb)
		return;

	/* close usb driver */
	if (usb->usb.driver) {
		pr_debug("close usb driver {\n");
		usb_deregister(usb->usb.driver);
		usb->usb.driver = (struct usb_driver *) 0;
		pr_debug("close usb driver }\n");
	}

	/* free baseband usb structure */
	kfree(usb);

	pr_debug("baseband_usb_close }\n");
}

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_rx_urb_comp(struct urb *urb);
static void usb_net_raw_ip_tx_urb_comp(struct urb *urb);

static int baseband_usb_netdev_init(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_init\n");
	return 0;
}

static void baseband_usb_netdev_uninit(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_uninit\n");
}

static int baseband_usb_netdev_open(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_open\n");
	netif_start_queue(dev);
	return 0;
}

static int baseband_usb_netdev_stop(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_stop\n");
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t baseband_usb_netdev_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct baseband_usb *usb = baseband_usb_net;
	struct urb *urb;
	unsigned char *buf, *src;
	int err;

	pr_debug("baseband_usb_netdev_start_xmit\n");

	/* check input */
	if (!skb) {
		pr_err("no skb\n");
		return -EINVAL;
	}

	/* allocate urb */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		return -ENOMEM;
	}
	buf = kzalloc(skb->len - 14, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		return -ENOMEM;
	}
	err = skb_copy_bits(skb, 14, buf, skb->len - 14);
	if (err < 0) {
		pr_err("skb_copy_bits() failed - %d\n", err);
		kfree(buf);
		usb_free_urb(urb);
		return err;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.out,
		buf, skb->len - 14,
		usb_net_raw_ip_tx_urb_comp,
		usb);
	urb->transfer_flags = URB_ZERO_PACKET;

	/* submit tx urb */
	usb->usb.tx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.tx_urb = (struct urb *) 0;
		kfree(urb->transfer_buffer);
		usb_free_urb(urb);
		kfree_skb(skb);
		return err;
	}

	/* free skb */
	consume_skb(skb);

	return NETDEV_TX_OK;
}

static struct net_device_ops usb_net_raw_ip_ops = {
	ndo_init: baseband_usb_netdev_init,
	ndo_uninit: baseband_usb_netdev_uninit,
	ndo_open: baseband_usb_netdev_open,
	ndo_stop: baseband_usb_netdev_stop,
	ndo_start_xmit: baseband_usb_netdev_start_xmit,
};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb)
{
	struct urb *urb;
	void *buf;
	int err;

	pr_debug("usb_net_raw_ip_rx_urb_submit { usb %p\n", usb);

	/* check input */
	if (usb->usb.rx_urb) {
		pr_err("previous urb still active\n");
		return -1;
	}

	/* allocate rx urb */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		return -ENOMEM;
	}
	buf = kzalloc(USB_NET_BUFSIZ, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		return -ENOMEM;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.in,
		buf, USB_NET_BUFSIZ,
		usb_net_raw_ip_rx_urb_comp,
		usb);
	urb->transfer_flags = 0;

	/* submit rx urb */
	usb->usb.rx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.rx_urb = (struct urb *) 0;
		kfree(urb->transfer_buffer);
		usb_free_urb(urb);
		return err;
	}

	pr_debug("usb_net_raw_ip_rx_urb_submit }\n");
	return err;
}

static void usb_net_raw_ip_rx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = (struct baseband_usb *) urb->context;
	struct sk_buff *skb;
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
		/* Destination MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* Source MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* EtherType */
		NET_IP_ETHERTYPE,
	};

	pr_debug("usb_net_raw_ip_rx_urb_comp { urb %p\n", urb);

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}

	/* put rx urb data in rx buffer */
	if (urb->actual_length) {
		pr_debug("usb_net_raw_ip_rx_urb_comp - "
			"urb->actual_length %d\n", urb->actual_length);
		/* allocate skb with space for
		 * - dummy ethernet header
		 * - rx IP packet from modem
		 */
		skb = netdev_alloc_skb(usb_net_raw_ip_dev,
			NET_IP_ALIGN + 14 + urb->actual_length);
		if (skb) {
			/* generate a dummy ethernet header
			 * since modem sends IP packets without
			 * any ethernet headers
			 */
			memcpy(ethernet_header + 0,
				usb_net_raw_ip_dev->dev_addr, 6);
			memcpy(ethernet_header + 6,
				"0x01\0x02\0x03\0x04\0x05\0x06", 6);
			/* fill skb with
			 * - dummy ethernet header
			 * - rx IP packet from modem
			 */
			skb_reserve(skb, NET_IP_ALIGN);
			dst = skb_put(skb, 14);
			memcpy(dst, ethernet_header, 14);
			dst = skb_put(skb, urb->actual_length);
			memcpy(dst, urb->transfer_buffer, urb->actual_length);
			skb->protocol = eth_type_trans(skb,
				usb_net_raw_ip_dev);
			/* pass skb to network stack */
			if (netif_rx(skb) < 0) {
				pr_err("usb_net_raw_ip_rx_urb_comp_work - "
					"netif_rx(%p) failed\n", skb);
				kfree_skb(skb);
			}
		} else {
			pr_err("usb_net_raw_ip_rx_urb_comp_work - "
				"netdev_alloc_skb() failed\n");
		}
	}

	/* free rx urb */
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}
	usb_free_urb(urb);
	usb->usb.rx_urb = (struct urb *) 0;

	/* submit next rx urb */
	usb_net_raw_ip_rx_urb_submit(usb);

	pr_debug("usb_net_raw_ip_rx_urb_comp }\n");
}

static void usb_net_raw_ip_tx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = (struct baseband_usb *) urb->context;

	pr_debug("usb_net_raw_ip_tx_urb_comp {\n");

	/* free tx urb */
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}
	usb_free_urb(urb);
	usb->usb.tx_urb = (struct urb *) 0;

	pr_debug("usb_net_raw_ip_tx_urb_comp }\n");
}

static int usb_net_raw_ip_init(void)
{
	int err;

	pr_debug("usb_net_raw_ip_init {\n");

	/* open baseband usb */
	baseband_usb_net = baseband_usb_open
		(usb_net_raw_ip_vid, usb_net_raw_ip_pid, usb_net_raw_ip_intf);
	if (!baseband_usb_net) {
		pr_err("cannot open baseband usb net\n");
		err = -1;
		goto err1;
	}

	/* register network device */
	usb_net_raw_ip_dev = alloc_netdev(0,
		BASEBAND_USB_NET_DEV_NAME,
		ether_setup);
	if (!usb_net_raw_ip_dev) {
		pr_err("alloc_netdev() failed\n");
		err = -ENOMEM;
		goto err2;
	}
	usb_net_raw_ip_dev->netdev_ops = &usb_net_raw_ip_ops;
	usb_net_raw_ip_dev->watchdog_timeo = TX_TIMEOUT;
	random_ether_addr(usb_net_raw_ip_dev->dev_addr);
	err = register_netdev(usb_net_raw_ip_dev);
	if (err < 0) {
		pr_err("cannot register network device - %d\n", err);
		goto err3;
	}
	pr_debug("registered baseband usb network device - dev %p name %s\n",
		usb_net_raw_ip_dev, BASEBAND_USB_NET_DEV_NAME);

	/* start usb rx */
	err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net);
	if (err < 0) {
		pr_err("submit rx failed - err %d\n", err);
		goto err4;
	}

	pr_debug("usb_net_raw_ip_init }\n");
	return 0;
err4:	unregister_netdev(usb_net_raw_ip_dev);
err3:	free_netdev(usb_net_raw_ip_dev);
	usb_net_raw_ip_dev = (struct net_device *) 0;
err2:
	baseband_usb_close(baseband_usb_net);
	baseband_usb_net = (struct baseband_usb *) 0;
err1:	return err;
}

static void usb_net_raw_ip_exit(void)
{
	pr_debug("usb_net_raw_ip_exit {\n");

	/* unregister network device */
	if (usb_net_raw_ip_dev) {
		unregister_netdev(usb_net_raw_ip_dev);
		free_netdev(usb_net_raw_ip_dev);
		usb_net_raw_ip_dev = (struct net_device *) 0;
	}

	/* close baseband usb */
	if (baseband_usb_net) {
		baseband_usb_close(baseband_usb_net);
		baseband_usb_net = (struct baseband_usb *) 0;
	}

	pr_debug("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)

