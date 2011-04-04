/*
 * The only header file for Hub TI TS5USBA33402 MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef _MUIC_H_
#define _MUIC_H_

#if defined (CONFIG_MACH_STAR)
#define _MUIC_GPIO_I2C_
#endif

#define TD_INT_STAT	70000	// INT_STAT bits settle down time since MUIC INT falls
#define TD_STATUS	250000	// STATUS bits settle down time since MUIC INT falls
#define TD_DP_DM	1000	// DP, DM path settle down time since SW_CONTROL writing

/* I2C addresses of MUIC internal registers */
#define	DEVICE_ID	(u8) 0x00
#define	CONTROL_1	(u8) 0x01
#define	CONTROL_2	(u8) 0x02
#define	SW_CONTROL	(u8) 0x03
#define	INT_STAT	(u8) 0x04
#define	STATUS		(u8) 0x05

/* Masks for the each bit of CONTROL_1 register */
#define	MID_2P2		(u8) 0x40
#define	MID_620		(u8) 0x20
#define	MID_200		(u8) 0x10
#define	MVLDO		(u8) 0x08
#define	MSEMREN		(u8) 0x04
#define	MADC_EN		(u8) 0x02
#define	MCP_EN		(u8) 0x01

/* Masks for the each bit of CONTROL_2 register */
#define	MINTPOL		(u8) 0x80
#define	MINT_EN		(u8) 0x40
#define	MMIC_LP		(u8) 0x20
#define	MCP_AUD		(u8) 0x10
#define	MCHG_TYP	(u8) 0x02
#define	MUSB_DET_DIS	(u8) 0x01

/* Masks for the each bit of SW_CONTROL register */
#define	MMIC_ON		(u8) 0x40
#define MDP		(u8) 0x38
#define MDM		(u8) 0x07
// DP,DM settings
#define DP_USB		(u8) 0x00
#define	DP_UART		(u8) 0x08
#define	DP_AUDIO	(u8) 0x10
#define	DP_OPEN		(u8) 0x38
#define DM_USB		(u8) 0x00
#define	DM_UART		(u8) 0x01
#define	DM_AUDIO	(u8) 0x02
#define	DM_OPEN		(u8) 0x07

/* Masks for the each bit of INT_STATUS register */
#define	MCHGDET		(u8) 0x80
#define	MMR_COMP	(u8) 0x40
#define	MSENDEND	(u8) 0x20
#define	MVBUS		(u8) 0x10
#define	MIDNO		(u8) 0x0f

/* Masks for the each bit of STATUS register */
#define	MDCPORT		(u8) 0x80
#define	MCHPORT		(u8) 0x40

/* Initialize MUIC - Default setting.
 *
 * CONTROL_1:
 * 
 * 	ID_2P2 	= 0. Enable to distinguish MUIC_EARMIC from MUIC_TV_OUT_LOAD and MUIC_OTG.
 * 		     Enable for MUIC_EARMIC operation.
 *	ID_620 	= 0. Enable only to distinguish MUIC_TV_OUT_LOAD from MUIC_OTG.
 *	ID_200 	= 1.
 *	VLDO 	= 0. Enable to apply 2.3V for MUIC_EARMIC operation.
 *	SEMREN 	= 1.
 *	ADC_EN 	= 0. Because it is automatically enabled upon any change in ID resistor.
 *	CP_EN 	= 0. Enalbe for USB 2.0 (MUIC_AP_USB, MUIC_CP_USB, and MUIC_OTG).
 *		     Enable for Audio charge pump (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
 * 
 * CONTROL_2: 
 *
 * 	INTPOL 	= 0.
 * 	INT_EN	= 1.
 * 	MIC_LP	= 0.
 * 	CP_AUD 	= 1. Disable for Audio operation (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
 * 	CHG_TYP	= 1.
 * 	USB_DET_DIS = 0. Negative enabled.
 *
 * SW_CONTROL: 
 *
 * 	MIC_ON	= 0. Enable for MUIC_EARMIC and MUIC_TV_OUT_LOAD.
 * 	DP	= 111 (open).
 * 	DM	= 111 (open).
 */

/* MUIC MODE
 *
 * V C D C IDNO IDNO IDNO 
 * B H C H 200K 2.2K 620
 * U G P P       
 * S D O O       
 *   E R R       
 *   T T T       
 * 1 1 1 - 0101 ---- ---- NA_TA (ID resistor 180KOhm) - Not used actually.
 * 1 1 1 - ---- ---- ---- LG_TA
 * 1 1 0 1 ---- ---- ---- HCHH (High current host/hub charger) - Not used actually.
 * 1 1 0 0 ---- ---- ---- Invalid charger
 * 1 0 0 0 0010 ---- ---- AP_UART (ID resistor 56KOhm)
 * 1 0 0 0 0100 ---- ---- CP_UART (ID resistor 130KOhm)
 * 1 0 0 0 1011 ---- ---- AP_USB (ID resistor open)
 * 1 0 0 0 ???? ---- ---- CP_USB (ID resistor ????) - Not defined yet.
 * 0 0 - - 0001 ---- ---- TV_OUT_NO_LOAD (ID resistor 24KOhm.) - Not used.
 * 0 0 - - 0000 01XX ---- EARMIC (ID resistor ground)
 * 0 0 - - 0000 0000 0001 TV_OUT_LOAD (ID resistor ground) - Not used.
 * 0 0 - - 0000 0000 0000 OTG (ID resistor ground) - Not used.
 */
typedef enum {
	MUIC_UNKNOWN,		// 0 - Error in detection or unsupported accessory.
	MUIC_NONE,		// 1 - No accessory is plugged in.
	MUIC_NA_TA,		// 2 - Not used actually. Special TA for North America.
	MUIC_LG_TA,		// 3
	MUIC_HCHH,		// 4 - Not used actually.
	MUIC_INVALID_CHG,	// 5
	MUIC_AP_UART,		// 6
	MUIC_CP_UART,		// 7
	MUIC_AP_USB,		// 8
	MUIC_CP_USB,		// 9 - Not defined yet.
	MUIC_TV_OUT_NO_LOAD,	// 10 - Not used.
	MUIC_EARMIC,		// 11
	MUIC_TV_OUT_LOAD,	// 12 - Not used.
	MUIC_OTG,		// 13 - Not used.
	MUIC_RESERVE1,		// 14
	MUIC_RESERVE2,		// 15
	MUIC_RESERVE3,		// 16
	MUIC_MODE_NO,		// 17
} TYPE_MUIC_MODE;

TYPE_MUIC_MODE get_muic_mode(void);

typedef enum {
	USIF_AP,	// 0
	USIF_DP3T,	// 1
} TYPE_USIF_MODE;

typedef enum {
	DP3T_NC,	// 0
	DP3T_AP_UART,	// 1
	DP3T_CP_UART,	// 2
	DP3T_CP_USB,	// 3
} TYPE_DP3T_MODE;

typedef enum {
	NOT_UPON_IRQ,	// 0
	UPON_IRQ,	// 1
} TYPE_UPON_IRQ;

typedef enum {
	DEFAULT,	// 0 - Just apply the default register settings.
	RESET,		// 1 - Fully reset the MUIC. It takes 250msec.
} TYPE_RESET;

#endif
