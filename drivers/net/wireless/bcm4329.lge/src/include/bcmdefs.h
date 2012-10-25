/*
 * Misc system wide definitions
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 * $Id: bcmdefs.h,v 13.38.4.10.2.7.6.7.4.8.2.2 2010/10/01 18:12:43 Exp $
 */


#ifndef	_bcmdefs_h_
#define	_bcmdefs_h_







#define bcmreclaimed 		0
#define r2_reclaimed		0
#define _data	_data
#define _fn	_fn
#define _data	_data
#define _fn		_fn
#define _fn	_fn
#define BCMPREATTACHDATA(_data)	_data
#define BCMPREATTACHFN(_fn)		_fn
#define CONST	const



#define _data	_data
#define _fn		_fn
#define _fn	_fn
#define STATIC	static



#define OVERLAY_INLINE
#define OSTATIC			static
#define BCMOVERLAYDATA(_ovly, _sym)	_sym
#define BCMOVERLAYFN(_ovly, _fn)	_fn
#define BCMOVERLAYERRFN(_fn)	_fn
#define BCMROMOVERLAYDATA(_ovly, _data)	_data
#define BCMROMOVERLAYFN(_ovly, _fn)		_fn
#define BCMATTACHOVERLAYDATA(_ovly, _sym)	_sym
#define BCMATTACHOVERLAYFN(_ovly, _fn)		_fn
#define BCMINITOVERLAYDATA(_ovly, _sym)		_sym
#define BCMINITOVERLAYFN(_ovly, _fn)		_fn
#define BCMUNINITOVERLAYFN(_ovly, _fn)		_fn



#define	SI_BUS			0	
#define	PCI_BUS			1	
#define	PCMCIA_BUS		2	
#define SDIO_BUS		3	
#define JTAG_BUS		4	
#define USB_BUS			5	
#define SPI_BUS			6	


#ifdef BCMBUSTYPE
#define BUSTYPE(bus) 	(BCMBUSTYPE)
#else
#define BUSTYPE(bus) 	(bus)
#endif


#ifdef BCMCHIPTYPE
#define CHIPTYPE(bus) 	(BCMCHIPTYPE)
#else
#define CHIPTYPE(bus) 	(bus)
#endif



#if defined(BCMSPROMBUS)
#define SPROMBUS	(BCMSPROMBUS)
#elif defined(SI_PCMCIA_SROM)
#define SPROMBUS	(PCMCIA_BUS)
#else
#define SPROMBUS	(PCI_BUS)
#endif


#ifdef BCMCHIPID
#define CHIPID(chip)	(BCMCHIPID)
#else
#define CHIPID(chip)	(chip)
#endif


#define DMADDR_MASK_32 0x0		
#define DMADDR_MASK_30 0xc0000000	
#define DMADDR_MASK_0  0xffffffff	

#define	DMADDRWIDTH_30  30 
#define	DMADDRWIDTH_32  32 
#define	DMADDRWIDTH_63  63 
#define	DMADDRWIDTH_64  64 




#if defined(BCMCHIPID) && (BCMCHIPID == BCM4319_CHIP_ID)
#define BCMEXTRAHDROOM 172
#else
#define BCMEXTRAHDROOM 164
#endif 


#define BCMDONGLEHDRSZ 12
#define BCMDONGLEPADSZ 16

#define BCMDONGLEOVERHEAD	(BCMDONGLEHDRSZ + BCMDONGLEPADSZ)



#define BITFIELD_MASK(width) \
		(((unsigned)1 << (width)) - 1)
#define GFIELD(val, field) \
		(((val) >> field ## _S) & field ## _M)
#define SFIELD(val, field, bits) \
		(((val) & (~(field ## _M << field ## _S))) | \
		 ((unsigned)(bits) << field ## _S))


#ifdef BCMSMALL
#undef	BCMSPACE
#define bcmspace	FALSE	
#else
#define	BCMSPACE
#define bcmspace	TRUE	
#endif


#define	MAXSZ_NVRAM_VARS	4096

#define LOCATOR_EXTERN static

#endif 
