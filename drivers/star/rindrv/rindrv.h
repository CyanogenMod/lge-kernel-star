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
 
#ifndef _LINUX_RIN_H
#define _LINUX_RIN_H

/* RIN configuration. */
#define RIN_NRUNIT	256		/* MAX number of RIN channels;
					   This can be overridden with
					   insmod -oslip_maxdev=nnn	*/
#define RIN_MTU		1500

struct rin_st {
  int			magic;

  /* Various fields. */
  struct tty_struct	*tty;		/* ptr to TTY structure		*/
  struct net_device	*dev;		/* easy for intr handling	*/
  spinlock_t		lock;

  /* These are pointers to the malloc()ed frame buffers. */
  unsigned char		*rbuff;		/* receiver buffer		*/
  int                   rcount;         /* received chars counter       */
  unsigned char		*xbuff;		/* transmitter buffer		*/
  unsigned char         *xhead;         /* pointer to next byte to XMIT */
  int                   xleft;          /* bytes left in XMIT queue     */

  /* RIN interface statistics. */
  unsigned long		rx_packets;	/* inbound frames counter	*/
  unsigned long         tx_packets;     /* outbound frames counter      */
  unsigned long		rx_bytes;	/* inbound byte counte		*/
  unsigned long         tx_bytes;       /* outbound byte counter	*/
  unsigned long         rx_errors;      /* Parity, etc. errors          */
  unsigned long         tx_errors;      /* Planned stuff                */
  unsigned long         rx_dropped;     /* No memory for skb            */
  unsigned long         tx_dropped;     /* When MTU change              */
  unsigned long         rx_over_errors; /* Frame bigger than RIN buf.  */

  /* Detailed RIN statistics. */

  int			mtu;		/* Our mtu (to spot changes!)   */
  int                   buffsize;       /* Max buffers sizes            */

  unsigned long		flags;		/* Flag values/ mode etc	*/
#define SLF_INUSE	0		/* Channel in use               */
#define SLF_ESCAPE	1               /* ESC received                 */
#define SLF_ERROR	2               /* Parity, etc. error           */
#define SLF_KEEPTEST	3		/* Keepalive test flag		*/
#define SLF_OUTWAIT	4		/* is outpacket was flag	*/

  unsigned char		leased;
  dev_t			line;
  pid_t			pid;

};

#define RIN_MAGIC 0x5305

#endif	/* _LINUX_RIN.H */
