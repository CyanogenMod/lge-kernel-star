/*
 * PHY module internal interface crossing different PHY types
 *
 * Copyright (C) 2010, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id: wlc_phy_int.h,v 1.25 2010/11/08 21:15:53 Exp $
 */

#ifndef _wlc_phy_int_h_
#define _wlc_phy_int_h_

#include <typedefs.h>
#include <bcmwifi.h>
#include <wlioctl.h>
#include <bcmutils.h>
#include <siutils.h>

#include <bcmsrom_fmt.h>
#include <wlc_phy_hal.h>
#define ROMTERMPHY 1
#define PHYHAL_ERROR	0x0001
#define PHYHAL_TRACE	0x0002
#define PHYHAL_INFORM	0x0004
#define PHYHAL_TMP	0x0008
#define PHYHAL_TXPWR	0x0010
#define PHYHAL_CAL	0x0020
#define PHYHAL_ACI	0x0040
#define PHYHAL_RADAR	0x0080
#define PHYHAL_THERMAL	0x0100
#define PHYHAL_PAPD     0x0200

extern uint32 phyhal_msg_level;

#define	PHY_ERROR(args)
#define	PHY_TRACE(args)
#define	PHY_INFORM(args)
#define	PHY_TMP(args)
#define	PHY_TXPWR(args)
#define	PHY_CAL(args)
#define	PHY_ACI(args)
#define	PHY_RADAR(args)
#define PHY_THERMAL(args)
#define PHY_PAPD(args)
#define	PHY_NONE(args)

#define PHY_INFORM_ON()		(phyhal_msg_level & PHYHAL_INFORM)
#define PHY_THERMAL_ON()	(phyhal_msg_level & PHYHAL_THERMAL)
#define PHY_CAL_ON()		(phyhal_msg_level & PHYHAL_CAL)

#ifdef BOARD_TYPE
#define BOARDTYPE(_type) BOARD_TYPE
#else
#define BOARDTYPE(_type) _type
#endif

#define LCNXN_BASEREV		16

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  inter-module connection					*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* forward declarations */
typedef struct wlc_hw_info wlc_hw_info_phy_t;

struct sslpnphy_tx_gain_tbl{
	uchar gm;
	uchar pga;
	uchar pad;
	uchar dac;
	uchar bb_mult;
};

typedef struct sslpnphy_tx_gain_tbl sslpnphy_tx_gain_tbl_entry;
typedef struct phy_info phy_info_t;
typedef void (*initfn_t)(phy_info_t *);
typedef void (*chansetfn_t)(phy_info_t *, chanspec_t);
typedef int (*longtrnfn_t)(phy_info_t *, int);
typedef void (*txiqccgetfn_t)(phy_info_t *, uint16 *, uint16 *);
typedef void (*txiqccsetfn_t)(phy_info_t *, uint16, uint16);
typedef uint16 (*txloccgetfn_t)(phy_info_t *);
typedef void (*radioloftgetfn_t)(phy_info_t *, uint8 *, uint8 *, uint8 *, uint8 *);
typedef int32 (*rxsigpwrfn_t)(phy_info_t *, int32);
typedef void (*detachfn_t)(phy_info_t *);

/* redefine some wlc_cfg.h macros to take the internal phy_info_t instead of wlc_phy_t */
#if TBD_YE
#undef ISAPHY
#undef ISGPHY
#undef ISNPHY
#undef ISLPPHY
#undef ISSSLPNPHY
#undef ISLCNPHY
#undef ISHTPHY
#define ISAPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_A)
#define ISGPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_G)
#define ISNPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_N)
#define ISLPPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_LP)
#define ISSSLPNPHY(pi)  PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_SSN)
#define ISLCNPHY(pi)  	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_LCN)
#define ISHTPHY(pi)  	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_HT)
#endif /* TBD_YE */


#define ISABGPHY(pi)	(ISAPHY(pi) || ISGPHY(pi))
#define ISLPSSNPHY(pi)	(ISLPPHY(pi) || ISSSLPNPHY(pi))

#define ISPHY_11N_CAP(pi)	(ISNPHY(pi) || ISSSLPNPHY(pi) || ISLCNPHY(pi) || ISHTPHY(pi))

#define IS20MHZ(pi)	((pi)->bw == WL_CHANSPEC_BW_20)
#define IS40MHZ(pi)	((pi)->bw == WL_CHANSPEC_BW_40)

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  macro, typedef, enum, structure, global variable		*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* %%%%%% shared */
#define PHY_GET_RFATTN(rfgain)	((rfgain) & 0x0f)
#define PHY_GET_PADMIX(rfgain)	(((rfgain) & 0x10) >> 4)
#define PHY_GET_RFGAINID(rfattn, padmix, width)	((rfattn) + ((padmix)*(width)))
#define PHY_SAT(x, n)		((x) > ((1<<((n)-1))-1) ? ((1<<((n)-1))-1) : \
				((x) < -(1<<((n)-1)) ? -(1<<((n)-1)) : (x)))
#define PHY_SHIFT_ROUND(x, n)	((x) >= 0 ? ((x)+(1<<((n)-1)))>>(n) : (x)>>(n))
#define PHY_HW_ROUND(x, s)		((x >> s) + ((x >> (s-1)) & (s != 0)))

/* channels */
#define CH_5G_GROUP	3	/* A band, channel groups: low, mid, high in A band */
#define A_LOW_CHANS	0	/* Index for low channels in A band */
#define A_MID_CHANS	1	/* Index for mid channels in A band */
#define A_HIGH_CHANS	2	/* Index for high channels in A band */
#define CH_2G_GROUP	1	/* B band, channel groups, just one */
#define G_ALL_CHANS	0	/* Index for all channels in G band */

#define FIRST_REF5_CHANNUM	149	/* Lower bound of disable channel-range for srom rev 1 */
#define LAST_REF5_CHANNUM	165	/* Upper bound of disable channel-range for srom rev 1 */
#define	FIRST_5G_CHAN		14	/* First allowed channel index for 5G band */
#define	LAST_5G_CHAN		50	/* Last allowed channel for 5G band */
#define	FIRST_MID_5G_CHAN	14	/* Lower bound of channel for using m_tssi_to_dbm */
#define	LAST_MID_5G_CHAN	35	/* Upper bound of channel for using m_tssi_to_dbm */
#define	FIRST_HIGH_5G_CHAN	36	/* Lower bound of channel for using h_tssi_to_dbm */
#define	LAST_HIGH_5G_CHAN	41	/* Upper bound of channel for using h_tssi_to_dbm */
#define	FIRST_LOW_5G_CHAN	42	/* Lower bound of channel for using l_tssi_to_dbm */
#define	LAST_LOW_5G_CHAN	50	/* Upper bound of channel for using l_tssi_to_dbm */

#define BASE_LOW_5G_CHAN	4900
#define BASE_MID_5G_CHAN	5100
#define BASE_HIGH_5G_CHAN	5500

#define CHAN5G_FREQ(chan)  (5000 + chan*5)
#define CHAN2G_FREQ(chan)  (2407 + chan*5)

/* power per rate array index */
#define CCK_20_PO		0
#define CCK_20UL_PO		1
#define OFDM_20_PO		2
#define OFDM_20UL_PO		3
#define OFDM_40DUP_PO		4
#define MCS_20_PO		5
#define MCS_20UL_PO		6
#define MCS_40_PO		7
#define MCS32_PO		8
#define PWR_OFFSET_SIZE		9

/* power rate */
#define TXP_FIRST_CCK		0	/* Index for first CCK rate */
#define TXP_LAST_CCK		3	/* Index for last CCK rate */
#define TXP_FIRST_OFDM	        4	/* Index for first 20MHz OFDM SISO rate */
#define TXP_LAST_OFDM	        11	/* Index for last 20MHz OFDM SISO rate */
#define TXP_FIRST_OFDM_20_CDD   12	/* Index for first 20MHz OFDM CDD rate */
#define TXP_LAST_OFDM_20_CDD    19	/* Index for last 20 MHz OFDM CDD rate */
#define TXP_FIRST_MCS_20_SISO   20	/* Index for first 20MHz MCS SISO rate */
#define TXP_LAST_MCS_20_SISO    27	/* Index for last 20MHz MCS SISO rate */
#define TXP_FIRST_MCS_20_CDD    28	/* Index for first 20MHz MCS CDD rate */
#define TXP_LAST_MCS_20_CDD     35	/* Index for last 20MHz MCS CDD rate */
#define TXP_FIRST_MCS_20_STBC   36	/* Index for first 20MHz MCS STBC rate */
#define TXP_LAST_MCS_20_STBC    43	/* Index for last 20MHz MCS STBC rate */
#define TXP_FIRST_MCS_20_SDM    44	/* Index for first 20MHz MCS SDM rate */
#define TXP_LAST_MCS_20_SDM     51	/* Index for last 20MHz MCS SDM rate */
#define TXP_FIRST_OFDM_40_SISO  52	/* Index for first 40MHz OFDM SISO rate */
#define TXP_LAST_OFDM_40_SISO   59	/* Index for last 40MHz OFDM SISO rate */
#define TXP_FIRST_OFDM_40_CDD   60	/* Index for first 40MHz OFDM CDD rate */
#define TXP_LAST_OFDM_40_CDD    67	/* Index for last 40 MHz OFDM CDD rate */
#define TXP_FIRST_MCS_40_SISO   68	/* Index for first 40MHz MCS SISO rate */
#define TXP_LAST_MCS_40_SISO    75	/* Index for last 40MHz MCS SISO rate */
#define TXP_FIRST_MCS_40_CDD    76	/* Index for first 40MHz MCS CDD rate */
#define TXP_LAST_MCS_40_CDD     83	/* Index for last 40MHz MCS CDD rate */
#define TXP_FIRST_MCS_40_STBC   84	/* Index for first 40MHz MCS STBC rate */
#define TXP_LAST_MCS_40_STBC    91	/* Index for last 40MHz MCS STBC rate */
#define TXP_FIRST_MCS_40_SDM    92	/* Index for first 40MHz MCS SDM rate */
#define TXP_LAST_MCS_40_SDM     99	/* Index for last 40MHz MCS SDM rate */
#define TXP_MCS_32	        100	/* Index for 40MHz rate MCS 32 */
#define TXP_NUM_RATES_MIMO	101	/* number of rates in MIMO mode */
#define TXP_FIRST_20UL_CCK      101	/* Index for first 20in40MHz CCK rate */
#define TXP_LAST_20UL_CCK	104	/* Index for last 20in40MHz CCK rate */
#define TXP_FIRST_20UL_OFDM     105	/* Index for first 20in40MHz OFDM rate */
#define TXP_LAST_20UL_OFDM	112	/* Index for last 20in40MHz OFDM rate */
#define TXP_FIRST_20UL_OFDM_CDD	113	/* Index for first 20in40MHz OFDM rate */
#define TXP_LAST_20UL_OFDM_CDD	120	/* Index for last 20in40MHz OFDM rate */
#define TXP_FIRST_20UL_MCS0     121	/* Index for first 20in40MHz MCS rate */
#define TXP_LAST_20UL_MCS0	128	/* Index for last 20in40MHz MCS rate */
#define TXP_FIRST_20UL_MCS1     129	/* Index for first 20in40MHz MCS rate */
#define TXP_LAST_20UL_MCS1	138	/* Index for last 20in40MHz MCS rate */
#define TXP_FIRST_20UL_STBC     137	/* Index for first 20in40MHz MCS rate */
#define TXP_LAST_20UL_STBC	144	/* Index for last 20in40MHz MCS rate */
#define TXP_FIRST_20UL_MCS2     145	/* Index for first 20in40MHz MCS rate */
#define TXP_LAST_20UL_MCS2	152	/* Index for last 20in40MHz MCS rate */
#define TXP_NUM_RATES_HT        153	/* number of rates in HT mode */

/* alias to SISO, CDD, SDM */
#define TXP_FIRST_20_MCS0	TXP_FIRST_MCS_20_SISO
#define TXP_LAST_20_MCS0	TXP_LAST_MCS_20_SISO
#define TXP_FIRST_20_MCS1	TXP_FIRST_MCS_20_STBC
#define TXP_LAST_20_MCS1   	TXP_LAST_MCS_20_STBC
#define TXP_FIRST_20_MCS2	TXP_FIRST_MCS_20_SDM
#define TXP_LAST_20_MCS2	TXP_LAST_MCS_20_SDM
#define TXP_FIRST_40_MCS0	TXP_FIRST_MCS_40_SISO
#define TXP_LAST_40_MCS0	TXP_LAST_MCS_40_SISO
#define TXP_FIRST_40_MCS1	TXP_FIRST_MCS_40_STBC
#define TXP_LAST_40_MCS1	TXP_LAST_MCS_40_STBC
#define TXP_FIRST_40_MCS2	TXP_FIRST_MCS_40_SDM
#define TXP_LAST_40_MCS2	TXP_LAST_MCS_40_SDM

#ifdef HTCONF
#define TXP_NUM_RATES		TXP_NUM_RATES_HT
#else
#define TXP_NUM_RATES		TXP_NUM_RATES_MIMO
#endif

#define ADJ_PWR_TBL_LEN		84	/* number of phy-capable rates */

/* sslpnphy/ for siso devices */
#define TXP_FIRST_SISO_MCS_20	20	/* Index for first SISO MCS at 20 MHz */
#define TXP_LAST_SISO_MCS_20	27	/* Index for last SISO MCS at 20 MHz */

/* PHY/RADIO core(chains) */
#define PHY_CORE_NUM_1	1	/* 1 stream */
#define PHY_CORE_NUM_2	2	/* 2 streams */
#define PHY_CORE_NUM_3	3	/* 3 streams */
#define PHY_CORE_NUM_4	4	/* 4 streams */
#define PHY_CORE_MAX	PHY_CORE_NUM_4
#define PHY_CORE_0	0	/* array index for core 0 */
#define PHY_CORE_1	1	/* array index for core 1 */
#define PHY_CORE_2	2	/* array index for core 2 */
#define PHY_CORE_3	3	/* array index for core 3 */

/* macros to loop over TX/RX cores */
#define FOREACH_ACTV_CORE(pi, coremask, idx)	\
	for (idx = 0; (int) idx < pi->pubpi.phy_corenum; idx++) \
		if ((coremask >> idx) & 0x1)

#define FOREACH_CORE(pi, idx)	\
	for (idx = 0; (int) idx < pi->pubpi.phy_corenum; idx++)

/* aci_state state bits */
/* #define ACI_ACTIVE	1 */	/* enabled either manually or automatically */
#define ACI_CHANNEL_DELTA 5	/* How far a signal can bleed */
#define ACI_CHANNEL_SKIP 2	/* Num of immediately surrounding channels to skip */
#define ACI_FIRST_CHAN 1 /* Index for first channel */
#define ACI_LAST_CHAN 13 /* Index for last channel */
#define ACI_INIT_MA 100 /* Initial moving average for glitch for ACI */
#define ACI_SAMPLES 100 /* Number of samples for ACI */
#define ACI_MAX_UNDETECT_WINDOW_SZ 40

#define MA_WINDOW_SZ		8	/* moving average window size */

/* aci scan period */
#define NPHY_ACI_CHECK_PERIOD 5

/* noise only scan period */
#define NPHY_NOISE_CHECK_PERIOD 3

/* noise/RSSI state */
#define PHY_NOISE_SAMPLE_MON		1	/* sample phy noise for watchdog */
#define PHY_NOISE_SAMPLE_EXTERNAL	2	/* sample phy noise for scan, CQ/RM */
#define PHY_NOISE_WINDOW_SZ	16	/* NPHY noisedump window size */
#define PHY_NOISE_GLITCH_INIT_MA 10	/* Initial moving average for glitch cnt */
#define PHY_NOISE_GLITCH_INIT_MA_BADPlCP 10	/* Initial moving average for badplcp cnt */
#define PHY_NOISE_STATE_MON		0x1
#define PHY_NOISE_STATE_EXTERNAL	0x2
#define PHY_NOISE_SAMPLE_LOG_NUM_NPHY	10
#define PHY_NOISE_SAMPLE_LOG_NUM_UCODE	9	/* ucode uses smaller value to speed up process */
/* G-band: 30 (dbm) - 10*log10(50*(2^9/0.4)^2/16) + 2 (front-end-loss) - 68 (init_gain)
 * A-band: 30 (dbm) - 10*log10(50*(2^9/0.4)^2/16) + 3 (front-end-loss) - 69 (init_gain)
 */
#define PHY_NOISE_OFFSETFACT_4322  (-103)
#define PHY_NOISE_MA_WINDOW_SZ	2	/* moving average window size */


#define	PHY_RSSI_TABLE_SIZE	64	/* Table size for PHY RSSI Table */
#define RSSI_ANT_MERGE_MAX	0	/* pick max rssi of all antennas */
#define RSSI_ANT_MERGE_MIN	1	/* pick min rssi of all antennas */
#define RSSI_ANT_MERGE_AVG	2	/* pick average rssi of all antennas */

/* TSSI/txpower */
#define	PHY_TSSI_TABLE_SIZE	64	/* Table size for PHY TSSI */
#define	APHY_TSSI_TABLE_SIZE	256	/* Table size for A-PHY TSSI */
#define	TX_GAIN_TABLE_LENGTH	64	/* Table size for gain_table */
#define	DEFAULT_11A_TXP_IDX	24	/* Default index for 11a Phy tx power */
#define NUM_TSSI_FRAMES        4	/* Num ucode frames for TSSI estimation */
#define	NULL_TSSI		0x7f	/* Default value for TSSI - byte */
#define	NULL_TSSI_W		0x7f7f	/* Default value for word TSSI */

#define PHY_PAPD_EPS_TBL_SIZE_LPPHY 64
#define PHY_PAPD_EPS_TBL_SIZE_LCNPHY 64

/* the generic PHY_PERICAL defines are in wlc_phy_hal.h */
#define LCNPHY_PERICAL_TEMPBASED_TXPWRCTRL 9

#define PHY_TXPWR_MIN		10	/* default min tx power */
#define PHY_TXPWR_MIN_NPHY	8	/* for nphy devices */
#define RADIOPWR_OVERRIDE_DEF	(-1)

#define PWRTBL_NUM_COEFF	3	/* b0, b1, a1 */

/* calibraiton */
#define SPURAVOID_AUTO		-1	/* enable on certain channels, disable elsewhere */
#define SPURAVOID_DISABLE	0	/* disabled */
#define SPURAVOID_FORCEON	1	/* on mode1 */
#define SPURAVOID_FORCEON2	2	/* on mode2 (different freq) */

#define PHY_SW_TIMER_FAST		15	/* 15 second timeout */
#define PHY_SW_TIMER_SLOW		60	/* 60 second timeout */
#define PHY_SW_TIMER_GLACIAL	120	/* 120 second timeout */

#define PHY_PERICAL_AUTO	0	/* cal type: let PHY decide */
#define PHY_PERICAL_FULL	1	/* cal type: full */
#define PHY_PERICAL_PARTIAL	2	/* cal type: partial (save time) */

#define PHY_PERICAL_NODELAY	0	/* multiphase cal gap, in unit of ms */
#define PHY_PERICAL_INIT_DELAY	5
#define PHY_PERICAL_ASSOC_DELAY	5
#define PHY_PERICAL_WDOG_DELAY	5

#define PHY_PERICAL_MPHASE_PENDING(pi) \
	(pi->cal_info->cal_phase_id > MPHASE_CAL_STATE_IDLE)

#define PHY_CAL_SEARCHMODE_RESTART   0  /* cal search mode (former FULL) */
#define PHY_CAL_SEARCHMODE_REFINE    1 /* cal search mode (former PARTIAL) */


enum {
	PHY_ACI_PWR_NOTPRESENT,   /* ACI is not present */
	PHY_ACI_PWR_LOW,
	PHY_ACI_PWR_MED,
	PHY_ACI_PWR_HIGH
};

#define SSLPNPHY_NOISE_PWR_FIFO_DEPTH 6

/* Multiphase calibration states and cmds per Tx Phase (for NPHY) */
#define MPHASE_TXCAL_NUMCMDS	2  /* Number of Tx cal cmds per phase */
enum {
	MPHASE_CAL_STATE_IDLE = 0,
	MPHASE_CAL_STATE_INIT = 1,
	MPHASE_CAL_STATE_TXPHASE0,
	MPHASE_CAL_STATE_TXPHASE1,
	MPHASE_CAL_STATE_TXPHASE2,
	MPHASE_CAL_STATE_TXPHASE3,
	MPHASE_CAL_STATE_TXPHASE4,
	MPHASE_CAL_STATE_TXPHASE5,
	MPHASE_CAL_STATE_PAPDCAL,	/* IPA */
	MPHASE_CAL_STATE_RXCAL,
	MPHASE_CAL_STATE_RSSICAL,
	MPHASE_CAL_STATE_IDLETSSI
};


/* mphase phases for HTPHY */
enum {
	CAL_PHASE_IDLE = 0,
	CAL_PHASE_INIT = 1,
	CAL_PHASE_TX0,
	CAL_PHASE_TX1,
	CAL_PHASE_TX2,
	CAL_PHASE_TX3,
	CAL_PHASE_TX4,
	CAL_PHASE_TX5,
	CAL_PHASE_TX6,
	CAL_PHASE_TX7,
	CAL_PHASE_TX_LAST,
	CAL_PHASE_PAPDCAL,	/* IPA */
	CAL_PHASE_RXCAL,
	CAL_PHASE_RSSICAL,
	CAL_PHASE_IDLETSSI
};

typedef enum {
	CAL_FULL,
	CAL_RECAL,
	CAL_CURRECAL,
	CAL_DIGCAL,
	CAL_GCTRL,
	CAL_SOFT,
	CAL_DIGLO
} phy_cal_mode_t;


/* Radar detect scratchpad area, RDR_NTIER_SIZE must be bigger than RDR_TIER_SIZE */
#define RDR_NTIERS  1	   /* Number of tiers */
#define RDR_TIER_SIZE 64   /* Size per tier, aphy  */
#define RDR_LIST_SIZE 512/3  /* Size of the list (rev 3 fifo size = 512) */
#define RDR_EPOCH_SIZE 40
#define RDR_NANTENNAS 2
#define RDR_NTIER_SIZE  RDR_LIST_SIZE  /* Size per tier, nphy */
#define RDR_LP_BUFFER_SIZE 64
#define LP_LEN_HIS_SIZE 10

/* For bounding the size of the baseband lo comp results array */
#define STATIC_NUM_RF 32	/* Largest number of RF indexes */
#define STATIC_NUM_BB 9		/* Largest number of BB indexes */

#define BB_MULT_MASK		0x0000ffff
#define BB_MULT_VALID_MASK	0x80000000

#define CORDIC_AG	39797
#define	CORDIC_NI	18
#define	FIXED(X)	((int32)((X) << 16))
#define	FLOAT(X)	(((X) >= 0) ? ((((X) >> 15) + 1) >> 1) : -((((-(X)) >> 15) + 1) >> 1))

#define PHY_CHAIN_TX_DISABLE_TEMP	115
#define PHY_HYSTERESIS_DELTATEMP	5

#define PHY_BITSCNT(x)	bcm_bitcount((uint8 *)&(x), sizeof(uint8))

#define MOD_PHY_REG(pi, phy_type, reg_name, field, value) \
	mod_phy_reg(pi, phy_type##_##reg_name, phy_type##_##reg_name##_##field##_MASK, \
	(value) << phy_type##_##reg_name##_##field##_##SHIFT)
#define READ_PHY_REG(pi, phy_type, reg_name, field) \
	((read_phy_reg(pi, phy_type##_##reg_name) & phy_type##_##reg_name##_##field##_##MASK)\
	>> phy_type##_##reg_name##_##field##_##SHIFT)

/* validation macros */
#define	VALID_PHYTYPE(phytype)	(((uint)phytype == PHY_TYPE_A) || \
				 ((uint)phytype == PHY_TYPE_B) || \
				 ((uint)phytype == PHY_TYPE_G) || \
				 ((uint)phytype == PHY_TYPE_N) || \
				 ((uint)phytype == PHY_TYPE_LP)|| \
				 ((uint)phytype == PHY_TYPE_SSN))

#define VALID_G_RADIO(radioid)	(radioid == BCM2050_ID)
#define VALID_A_RADIO(radioid)	(radioid == BCM2060_ID)
#define VALID_N_RADIO(radioid)  ((radioid == BCM2055_ID) || (radioid == BCM2056_ID) || \
				(radioid == BCM2057_ID))
#define VALID_HT_RADIO(radioid)  (radioid == BCM2059_ID)

#define LPPHY_RADIO_ID(pi)	(LPREV_GE((pi)->pubpi.phy_rev, 2) ? BCM2063_ID : BCM2062_ID)
#define VALID_LP_RADIO(pi, radioid) (LPPHY_RADIO_ID(pi) == (radioid))
#define VALID_SSLPN_RADIO(radioid)  (radioid == BCM2063_ID)
#define VALID_LCN_RADIO(radioid)  ((radioid == BCM2064_ID) || (radioid == BCM2066_ID))

#define	VALID_RADIO(pi, radioid) \
	(ISGPHY(pi) ? VALID_G_RADIO(radioid) : \
	 (ISAPHY(pi) ? VALID_A_RADIO(radioid) : \
	  (ISNPHY(pi) ? VALID_N_RADIO(radioid) : \
	   (ISSSLPNPHY(pi) ? VALID_SSLPN_RADIO(radioid) : \
	    VALID_LP_RADIO(pi, radioid)))))

#define SCAN_INPROG_PHY(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_SCAN))
#define RM_INPROG_PHY(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_RM))
#define PLT_INPROG_PHY(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_PLT))
#define ASSOC_INPROG_PHY(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_ASSOC))
#define SCAN_RM_IN_PROGRESS(pi) (mboolisset(pi->measure_hold, PHY_HOLD_FOR_SCAN | PHY_HOLD_FOR_RM))
#define PHY_MUTED(pi)		(mboolisset(pi->measure_hold, PHY_HOLD_FOR_MUTE))
#define PUB_NOT_ASSOC(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_NOT_ASSOC))
#define ACI_SCAN_INPROG_PHY(pi)	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_ACI_SCAN))

#if defined(EXT_CBALL)
#define NORADIO_ENAB(pub) ((pub).radioid == NORADIO_ID)
#else
#define NORADIO_ENAB(pub) 0
#endif

#define BCMECICOEX_ENAB_PHY(pi)         0
#define BCMECISECICOEX_ENAB_PHY(pi)	0

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  phy_info_t and its prerequisite                             */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */


typedef struct _phy_table_info {
	uint	table;
	int	q;
	uint	max;
} phy_table_info_t;

/* phy table structure used for MIMOPHY */
typedef struct phytbl_info {
	const void   *tbl_ptr;
	uint32  tbl_len;
	uint32  tbl_id;
	uint32  tbl_offset;
	uint32  tbl_width;
} phytbl_info_t;

/* phy txcal coeffs structure used for HTPHY */
typedef struct txcal_coeffs {
	uint16 txa;
	uint16 txb;
	uint16 txd;	/* contain di & dq */
	uint8 txei;
	uint8 txeq;
	uint8 txfi;
	uint8 txfq;
	uint16 rxa;
	uint16 rxb;
} txcal_coeffs_t;

/* phy table structure used for LPPHY */
typedef struct lpphytbl_info {
	const void   *tbl_ptr;
	uint32  tbl_len;
	uint32  tbl_id;
	uint32  tbl_offset;
	uint32  tbl_width;
	uint8	tbl_phywidth;
} lpphytbl_info_t;

typedef struct {
	uint8  curr_home_channel;
	uint16 crsminpwrthld_40_stored;
	uint16 crsminpwrthld_20L_stored;
	uint16 crsminpwrthld_20U_stored;
	uint16 crsminpwr1thld_40_stored;
	uint16 crsminpwr1thld_20L_stored;
	uint16 crsminpwr1thld_20U_stored;
	uint16 init_gain_code_core1_stored;
	uint16 init_gain_code_core2_stored;
	uint16 init_gain_codeb_core1_stored;
	uint16 init_gain_codeb_core2_stored;
	uint16 init_gain_table_stored[4];

	uint16 clip1_hi_gain_code_core1_stored;
	uint16 clip1_hi_gain_code_core2_stored;
	uint16 clip1_hi_gain_codeb_core1_stored;
	uint16 clip1_hi_gain_codeb_core2_stored;
	uint16 nb_clip_thresh_core1_stored;
	uint16 nb_clip_thresh_core2_stored;
	uint16 init_ofdmlna2gainchange_stored[4];
	uint16 init_ccklna2gainchange_stored[4];
	uint16 clip1_lo_gain_code_core1_stored;
	uint16 clip1_lo_gain_code_core2_stored;
	uint16 clip1_lo_gain_codeb_core1_stored;
	uint16 clip1_lo_gain_codeb_core2_stored;
	uint16 w1_clip_thresh_core1_stored;
	uint16 w1_clip_thresh_core2_stored;
	uint16 radio_2056_core1_rssi_gain_stored;
	uint16 radio_2056_core2_rssi_gain_stored;
	uint16 energy_drop_timeout_len_stored;

	uint16 ed_crs40_assertthld0_stored;
	uint16 ed_crs40_assertthld1_stored;
	uint16 ed_crs40_deassertthld0_stored;
	uint16 ed_crs40_deassertthld1_stored;
	uint16 ed_crs20L_assertthld0_stored;
	uint16 ed_crs20L_assertthld1_stored;
	uint16 ed_crs20L_deassertthld0_stored;
	uint16 ed_crs20L_deassertthld1_stored;
	uint16 ed_crs20U_assertthld0_stored;
	uint16 ed_crs20U_assertthld1_stored;
	uint16 ed_crs20U_deassertthld0_stored;
	uint16 ed_crs20U_deassertthld1_stored;

	uint	scanroamtimer;

	int8	rssi;
	int8 	rssi_index;
	int8 	rssi_buffer[10];
	int16	max_hpvga_acioff_2G;
	int16	max_hpvga_acion_2G;
	int16	max_hpvga_acioff_5G;
	int16	max_hpvga_acion_5G;

	uint16  badplcp_ma;
	uint16  badplcp_ma_previous;
	uint16  badplcp_ma_total;
	uint16  badplcp_ma_list[MA_WINDOW_SZ];
	int  badplcp_ma_index;
	int16 pre_badplcp_cnt;
	int16  bphy_pre_badplcp_cnt;

	uint16 init_gain_core1;
	uint16 init_gain_core2;
	uint16 init_gainb_core1;
	uint16 init_gainb_core2;
	uint16 init_gain_rfseq[4];

	uint16 init_gain_core1_aci_on;
	uint16 init_gain_core2_aci_on;
	uint16 init_gainb_core1_aci_on;
	uint16 init_gainb_core2_aci_on;
	uint16 init_gain_rfseq_aci_on[4];
	uint16 init_gain_core1_aci_off;
	uint16 init_gain_core2_aci_off;
	uint16 init_gainb_core1_aci_off;
	uint16 init_gainb_core2_aci_off;
	uint16 init_gain_rfseq_aci_off[4];

	uint16 crsminpwr0;
	uint16 crsminpwrl0;
	uint16 crsminpwru0;

	uint16 crsminpwr0_aci_on;
	uint16 crsminpwrl0_aci_on;
	uint16 crsminpwru0_aci_on;

	uint16 crsminpwr0_aci_off;
	uint16 crsminpwrl0_aci_off;
	uint16 crsminpwru0_aci_off;

	int16 crsminpwr_index;
	int16 crsminpwr_index_aci_on;
	int16 crsminpwr_index_aci_off;

	uint16 radio_2057_core1_rssi_wb1a_gc_stored;
	uint16 radio_2057_core2_rssi_wb1a_gc_stored;
	uint16 radio_2057_core1_rssi_wb1g_gc_stored;
	uint16 radio_2057_core2_rssi_wb1g_gc_stored;
	uint16 radio_2057_core1_rssi_wb2_gc_stored;
	uint16 radio_2057_core2_rssi_wb2_gc_stored;
	uint16 radio_2057_core1_rssi_nb_gc_stored;
	uint16 radio_2057_core2_rssi_nb_gc_stored;

	bool  aci_on_firsttime;

	struct {
		uint16  ofdm_glitch_ma;
		uint16  ofdm_glitch_ma_previous;
		int16  bphy_pre_glitch_cnt;
		uint16  ofdm_ma_total;
		uint16  ofdm_glitch_ma_list[PHY_NOISE_MA_WINDOW_SZ];
		int  ofdm_ma_index;

		uint16  ofdm_badplcp_ma;
		uint16  ofdm_badplcp_ma_previous;
		uint16  ofdm_badplcp_ma_total;
		uint16  ofdm_badplcp_ma_list[PHY_NOISE_MA_WINDOW_SZ];
		int  ofdm_badplcp_ma_index;


		uint16 noise_glitch_low_detect_total;
		uint16 noise_glitch_high_detect_total;

		uint16 newgain_initgain;
		uint16 newgainb_initgain;
		uint16 newgain_rfseq;
		bool changeinitgain;
		uint16 newcrsminpwr_20U;
		uint16 newcrsminpwr_20L;
		uint16 newcrsminpwr_40;

		uint16 nphy_noise_noassoc_glitch_th_up; /* wl interference 4 */
		uint16 nphy_noise_noassoc_glitch_th_dn;
		uint16 nphy_noise_assoc_glitch_th_up;
		uint16 nphy_noise_assoc_glitch_th_dn;
		uint16 nphy_noise_assoc_aci_glitch_th_up;
		uint16 nphy_noise_assoc_aci_glitch_th_dn;
		uint16 nphy_noise_assoc_enter_th;
		uint16 nphy_noise_noassoc_enter_th;
		uint16 nphy_noise_assoc_rx_glitch_badplcp_enter_th;
		uint16 nphy_noise_noassoc_crsidx_incr;
		uint16 nphy_noise_assoc_crsidx_incr;
		uint16 nphy_noise_crsidx_decr;

	} noise;

	struct {
		uint16  glitch_ma;
		uint16  glitch_ma_previous;
		int  pre_glitch_cnt;
		uint16  ma_total;
		uint16  ma_list[MA_WINDOW_SZ];
		int  ma_index;
		int  exit_thresh;		/* Lowater to exit aci */
		int  enter_thresh;		/* hiwater to enter aci mode */
		int  usec_spintime;		/* Spintime between samples */
		int  glitch_delay;		/* delay between ACI scans when glitch count is
								 * continously high
								 */
		int  countdown;
		char rssi_buf[13][ACI_SAMPLES + 1];	/* Save rssi vals for debugging */

		struct {
			uint16 radio_2055_core1_rxrf_spc1;
			uint16 radio_2055_core2_rxrf_spc1;

			uint16 radio_2055_core1_rxbb_rxcal_ctrl;
			uint16 radio_2055_core2_rxbb_rxcal_ctrl;

			uint16 overrideDigiGain1;
			uint16 bphy_peak_energy_lo;

			uint16 init_gain_code_core1;
			uint16 init_gain_code_core2;
			uint16 init_gain_table[4];

			uint16 clip1_hi_gain_code_core1;
			uint16 clip1_hi_gain_code_core2;
			uint16 clip1_md_gain_code_core1;
			uint16 clip1_md_gain_code_core2;
			uint16 clip1_lo_gain_code_core1;
			uint16 clip1_lo_gain_code_core2;

			uint16 nb_clip_thresh_core1;
			uint16 nb_clip_thresh_core2;

			uint16 w1_clip_thresh_core1;
			uint16 w1_clip_thresh_core2;

			uint16 cck_compute_gain_info_core1;
			uint16 cck_compute_gain_info_core2;

			uint16 energy_drop_timeout_len;
			uint16 crs_threshold2u;

			bool gain_boost;

			/* Phyreg 0xc33 (bphy energy thresh values for ACI pwr levels */
			/*  (lo, md, hi) */
			uint16 b_energy_lo_aci;
			uint16 b_energy_md_aci;
			uint16 b_energy_hi_aci;

			bool detection_in_progress;
			/* can be modified using ioctl/iovar */
			uint16 adcpwr_enter_thresh;
			uint16 adcpwr_exit_thresh;
			uint16 detect_repeat_ctr;
			uint16 detect_num_samples;
			uint16 undetect_window_sz;

		} nphy;

		/* history of ACI detects */
		int  detect_index;
		int  detect_total;
		int  detect_list[ACI_MAX_UNDETECT_WINDOW_SZ];
		int  detect_acipwr_lt_list[ACI_MAX_UNDETECT_WINDOW_SZ];
		int  detect_acipwr_max;

	} aci;


} interference_info_t;


#define LPPHY_ACI_MAX_REGS 16
typedef struct {
	int on_thresh; /* number of glitches */
	int on_timeout; /* in seconds */
	int off_thresh; /* number of glitches */
	int off_timeout; /* in seconds */
	int glitch_timeout; /* in microseconds */
	int glitch_cnt;
	int chan_scan_cnt; /* number of measurements to take on a given channel */
	int chan_scan_pwr_thresh; /* pwr metric threshold */
	int chan_scan_cnt_thresh; /* count threshold */
	int chan_scan_timeout; /* minimum time between channel scans */
	int state;
	int t;
	int t_scan;
	const uint32* aci_reg;
	uint8 dflt_reg[LPPHY_ACI_MAX_REGS];
	int last_chan;
} lpphy_aci_t;

typedef struct {
	wl_radar_args_t radar_args;	/* radar detection parametners */
//	wl_radar_thr_t radar_thrs;	/* radar thresholds */
	int min_tint;			/* minimum t_int (1/prf) (20 MHz clocks) */
	int max_tint;			/* maximum t_int (20 MHz clocks) */
	int min_blen;			/* minimum burst length (20 MHz clocks) */
	int max_blen;			/* maximum burst length (20 MHz clocks) */
	int sdepth_extra_pulses;
	int min_deltat_lp;
	int max_deltat_lp;
	int max_type1_pw;		/* max fcc type 1 radar pulse width */
	int jp2_1_intv;		/* min fcc type 1 radar pulse repetition interval */
	int type1_intv;
	int max_type2_pw;
	int max_type2_intv;
	int min_type2_intv;
	int max_type4_pw;
	int min_type3_4_intv;
	int max_type3_4_intv;
	int max_jp1_2_pw;
	int jp1_2_intv;
	int jp2_3_intv;
} radar_params_t;

typedef struct {
	int length;
	uint32 tstart_list[2*RDR_LIST_SIZE];
	int width_list[2*RDR_LIST_SIZE];
	int fm_list[2*RDR_LIST_SIZE];
	int epoch_start[RDR_EPOCH_SIZE];
	int epoch_finish[RDR_EPOCH_SIZE];
	int tiern_list[RDR_NTIERS][RDR_NTIER_SIZE]; /* increased size of tiern list */
	int tiern_pw[RDR_NTIERS][RDR_NTIER_SIZE];
	int nepochs;
	int nphy_length[RDR_NANTENNAS];
	uint32 tstart_list_n[RDR_NANTENNAS][RDR_LIST_SIZE];
	int width_list_n[RDR_NANTENNAS][RDR_LIST_SIZE];
	int fm_list_n[RDR_NANTENNAS][RDR_LIST_SIZE];
	int nphy_length_bin5[RDR_NANTENNAS];
	uint32 tstart_list_bin5[RDR_NANTENNAS][RDR_LIST_SIZE];
	int width_list_bin5[RDR_NANTENNAS][RDR_LIST_SIZE];
	int lp_length;
	uint32 lp_buffer[RDR_LP_BUFFER_SIZE];
	uint32 last_tstart;
	int lp_cnt;
	int lp_skip_cnt;
	int lp_len_his[LP_LEN_HIS_SIZE];
	int lp_len_his_idx;
} radar_work_t;

typedef struct _lo_complex_t {
	int8 i;
	int8 q;
} lo_complex_abgphy_info_t;

typedef struct _nphy_iq_comp {
	int16 a0;
	int16 b0;
	int16 a1;
	int16 b1;
} nphy_iq_comp_t;

typedef struct {
	uint16		txcal_interm_coeffs[11]; /* best coefficients */
	chanspec_t	txiqlocal_chanspec;	/* Last Tx IQ/LO cal chanspec */
	uint16 		txiqlocal_coeffs[11]; /* best coefficients */
	bool		txiqlocal_coeffsvalid;   /* bit flag */
} nphy_cal_result_t;

typedef struct {
	/* TX IQ LO cal results */
	uint16 txiqlocal_bestcoeffs[11];
	uint16 txiqlocal_bestcoeffs_valid;
	uint16 didq_cck;

	/* PAPD results */
	uint32 papd_eps_tbl[PHY_PAPD_EPS_TBL_SIZE_LPPHY];
	uint16 papd_indexOffset;
	uint16 papd_startstopindex;

	/* RX IQ cal results */
	uint16 rxiqcal_coeffs;
} lpphy_cal_results_t;

typedef struct {
	/* TX IQ LO cal results */
	uint16 txiqlocal_a;
	uint16 txiqlocal_b;
	uint16 txiqlocal_didq;
	uint8 txiqlocal_ei0;
	uint8 txiqlocal_eq0;
	uint8 txiqlocal_fi0;
	uint8 txiqlocal_fq0;
	/* TX IQ LO cal results */
	uint16 txiqlocal_bestcoeffs[11];
	uint16 txiqlocal_bestcoeffs_valid;

	/* PAPD results */
	uint32 papd_eps_tbl[PHY_PAPD_EPS_TBL_SIZE_LCNPHY];
	uint16 analog_gain_ref;
	uint16 lut_begin;
	uint16 lut_end;
	uint16 lut_step;
	uint16 rxcompdbm;
	uint16 papdctrl;
	uint16 sslpnCalibClkEnCtrl;

	/* RX IQ cal results */
	uint16 rxiqcal_coeff_a0;
	uint16 rxiqcal_coeff_b0;
} lcnphy_cal_results_t;

typedef struct {
	uint8	disable_temp; /* temp at which to drop to 1-Tx chain */
	uint8	hysteresis;   /* temp hysteresis to enable multi-Tx chains */
	uint8	enable_temp;  /* temp at which to enable multi-Tx chains */
	bool	heatedup;     /* indicates if chip crossed tempthresh */
} phy_txcore_temp_t;

/* htphy: tx gain settings */
typedef struct {
	uint16 rad_gain; /* Radio gains */
	uint16 dac_gain; /* DAC attenuation */
	uint16 bbmult;   /* BBmult */
} txgain_setting_t;

typedef struct {
	uint16  txiqlocal_coeffs[20]; /* best ("final") comp coeffs (up to 4 cores) */
	uint16  txiqlocal_interm_coeffs[20]; /* intermediate comp coeffs */
	bool    txiqlocal_coeffsvalid;   /* bit flag */
	uint8   txiqlocal_ladder_updated[PHY_CORE_MAX]; /* up to 4 cores */
	txgain_setting_t cal_orig_txgain[PHY_CORE_MAX];
	txgain_setting_t txcal_txgain[PHY_CORE_MAX];
} htphy_cal_result_t;

typedef struct {
	uint8	cal_searchmode;
	uint8	cal_phase_id;	/* mphase cal state */
	uint8	txcal_cmdidx;
	uint8	txcal_numcmds;

	union {
		nphy_cal_result_t ncal;
		htphy_cal_result_t htcal;
	} u;

	uint       last_papd_cal_time; /* Used for LP Phy only till now */
	uint       last_cal_time; /* in [sec], covers 136 years if 32 bit */
	int16      last_cal_temp;
} phy_cal_info_t;

/* hold cached values for all channels, useful for debug */
#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
typedef struct {
	uint16 txcal_coeffs[8];
	uint16 txcal_radio_regs[8];
	nphy_iq_comp_t rxcal_coeffs;
	uint16 rssical_radio_regs[2];
	uint16 rssical_phyregs[12];
} nphy_calcache_t;

typedef struct ch_calcache {
	struct ch_calcache *next;
	bool valid;
	chanspec_t chanspec; /* which chanspec are these values for? */
	union {
		nphy_calcache_t nphy_cache;
	} u;
	phy_cal_info_t cal_info;
} ch_calcache_t;
#endif /* PHYCAL_CACHING || WLMCHAN */

#define PHY_NOISEVAR_BUFSIZE 10	/* Increase this if we need to save more noise vars */

typedef struct {
	uint8 tssipos;		/* TSSI positive slope, 1: positive, 0: negative */
	uint8 extpagain;	/* Ext PA gain-type: full-gain: 0, pa-lite: 1, no_pa: 2 */
	uint8 pdetrange;	/* support 32 combinations of different Pdet dynamic ranges */
	uint8 triso;		/* TR switch isolation */
	uint8 antswctrllut;	/* antswctrl lookup table configuration: 32 possible choices */
} srom_fem_t;


/* phy state that is per device instance */
struct shared_phy {
	struct phy_info	*phy_head;      /* head of phy list */
	uint	fast_timer;		/* Periodic timeout for 'fast' timer */
	uint	slow_timer;		/* Periodic timeout for 'slow' timer */
	uint	glacial_timer;		/* Periodic timeout for 'glacial' timer */
	int	interference_mode;	/* enable interference mitigation mode */
	uint8	rx_antdiv;		/* .11b Ant. diversity (rx) selection override */
	int8	phy_noise_window[MA_WINDOW_SZ];	/* noise moving average window */
	uint	phy_noise_index;	/* noise moving average window index */
	int8	ant_avail_aa2g;
	int8	ant_avail_aa5g;
	bool	japan_wide_filter;
} shared_phy;

/* Public phy info structure */
struct phy_pub {
	uint		phy_type;		/* PHY_TYPE_XX */
	uint		phy_rev;		/* phy revision */
	uint		coreflags;		/* sbtml core/phy specific flags */
	uint		ana_rev;		/* analog core revision */
	uint16		radioid;		/* radio id */
	uint8		radiorev;		/* radio revision */
	bool		encore;			/* true if chipset is encore enabled */
} phy_pub;

struct phy_info {
	wlc_phy_t	pubpi_ro;		/* public attach time constant phy state */
	shared_phy_t	*sh;			/* shared phy state pointer */
	wlc_phy_t	pubpi;			/* private attach time constant phy state */
	chanspec_t	radio_chanspec;		/* current radio chanspec */
	uint16		radio_code;		/* radio code */
	uint8		bw;			/* b/w (10, 20 or 40) [only 20MHZ on non NPHY] */
	uint8		tx_power_max;		/* max power for all rates on this channel. Power
						 * offsets use this.
						 */
	int8		n_preamble_override;	/* preamble override for both TX & RX */
	bool		hwpwrctrl;		/* ucode controls txpower instead of driver */
	uint8		nphy_txpwrctrl;		/* tx power control setting */
	int8		nphy_txrx_chain;	/* chain override for both TX & RX */
	bool		nphy_5g_pwrgain;	/* flag to indicate 5G Power Gain is enabled */
	bool		phy_init_por;		/* power on reset prior to phy init call */
	/* Have we been initialized? */
	bool		init_in_progress;	/* init in progress */
	bool		initialized;
	bool		sbtml_gm;
	void		*wlc;
	wlc_hw_info_phy_t	*wlc_hw;
	wlc_pub_t	*pub;
	d11regs_t	*regs;
	struct phy_info	*next;
	uint		refcnt;
	bool		watchdog_override;	/* to disable/enable phy watchdog */
	bool		measure_hold;		/* should we hold off measurements/calibrations */

	srom_fem_t	srom_fem2g;		/* 2G band FEM attributes */
	srom_fem_t	srom_fem5g;		/* 5G band FEM attributes */

	/* power control */
	int16	txpa_2g[PWRTBL_NUM_COEFF];	/* tx power amp values for 2G:     pa0b%d   */
	int16	txpa_2g_low_temp[PWRTBL_NUM_COEFF];	/* tssi coeffs for low temperature  */
	int16	txpa_2g_high_temp[PWRTBL_NUM_COEFF];	/* tssi coeffs for high temperature */
	int16	txpa_5g_low[PWRTBL_NUM_COEFF];	/* tx power amp values for 5G low: pa1lob%d */
	int16	txpa_5g_mid[PWRTBL_NUM_COEFF];	/* tx power amp values for 5G mid: pa1b%d   */
	int16	txpa_5g_hi[PWRTBL_NUM_COEFF];	/* tx power amp values for 5G hi:  pa1hib%d */
	uint8	tx_srom_max_2g;		/* srom max board value (.25 dBm) for 2G:     pa0maxpwr   */
	uint8	tx_srom_max_5g_low;	/* srom max board value (.25 dBm) for 5G low: pa1lomaxpwr */
	uint8	tx_srom_max_5g_mid;	/* srom max board value (.25 dBm) for 5G mid: pa1maxpwr   */
	uint8	tx_srom_max_5g_hi;	/* srom max board value (.25 dBm) for 5G hi:  pa1himaxpwr */
	uint8	tx_srom_max_rate_2g[TXP_NUM_RATES];	/* Max power for all 2.4G channels */
	uint8	tx_srom_max_rate_5g_low[TXP_NUM_RATES];	/* Max power for low 5G band channels */
	uint8	tx_srom_max_rate_5g_mid[TXP_NUM_RATES];	/* Max power for mid 5G band channels */
	uint8	tx_srom_max_rate_5g_hi[TXP_NUM_RATES];	/* Max power for hi 5G band channels */
	uint8	tx_user_target[TXP_NUM_RATES];		/* Current user override target */
	int8	tx_power_offset[TXP_NUM_RATES];		/* Offset from base power */
	uint8	tx_power_target[TXP_NUM_RATES];		/* Current target power */
	int8		idle_tssi[CH_5G_GROUP];		/* Measured idle_tssi for lo,mid,hi */

	int8		target_idle_tssi;		/* target idle tssi value */
	int8		txpwr_est_Pout;			/* Best guess at current txpower */
	uint8		tx_power_min;			/* min power for all rates on this chan */
	int8		txpwr_limit[TXP_NUM_RATES];	/* reg and local power limit */
	uint8		adj_pwr_tbl_nphy[ADJ_PWR_TBL_LEN];	/* Adjusted power table of NPHY */

	uint8		ldo_voltage;			/* valid iff lpphy rev0 */
	bool		PAD;		/* valid iff lpphy */

	bool		txpwroverride;			/* override */
	bool		txpwridx_override_aphy;		/* override power index on A-phy */
	int16		radiopwr_override;		/* phy PWR_CTL override, -1=default */
	uint16		hwpwr_txcur;		/* hwpwrctl: current tx power index */

	uint16		radiopwr;		/* radio power */
	uint16		bb_atten;		/* baseband attentuation */
	uint16		txctl1;			/* iff 2050 */


	uint8		phynoise_state;		/* phy noise sample state */
	uint		phynoise_now;		/* timestamp to run sampling */
	int		phynoise_chan_watchdog;	/* sampling target channel for watchdog */
	int		phynoise_chan_scan;	/* sampling target channel for scan */
	int		phynoise_chan_cqrm;	/* sampling target channel for CQ/RM */
	bool		phynoise_polling;	/* polling or interrupt for sampling */

	/* nrssi calibration: */
	uint16		rc_cal;
	int		nrssi_table_delta;
	int		nrssi_slope_scale;
	int		nrssi_slope_offset;
	int		min_rssi;
	int		max_rssi;

	struct wl_timer *phynoise_timer;	/* timer for multiphase noise measurement */

	/* 11a Power control */
	int8		txpwridx;

	/* high channels in a band to be disabled for srom ver 1 */
	uint8		a_band_high_disable;

	/* 11a LOFT */
	uint16		tx_vos;
	uint16		global_tx_bb_dc_bias_loft;

	/* measlo for 11b */
	int		rf_max;
	int		bb_max;
	int		rf_list_size;
	int		bb_list_size;
	uint16		*rf_attn_list;
	uint16		*bb_attn_list;
	uint16		padmix_mask;
	uint16		padmix_reg;
	uint16		*txmag_list;
	uint		txmag_len;
	bool		txmag_enable;


	/* tssi to dbm translation table */
	int8		*a_tssi_to_dbm;	/* a-band rev 1 srom */
	int8		*m_tssi_to_dbm;	/* mid band rev 2 srom */
	int8		*l_tssi_to_dbm;	/* low band rev 2 srom */
	int8		*h_tssi_to_dbm;	/* high band rev 2 srom */
	uint8		*hwtxpwr;

	/* Original values of the PHY regs, that we modified, to increase the freq tracking b/w */
	uint16		freqtrack_saved_regs [2];
	int		cur_interference_mode;	/* Track interference mode of phy */
	bool 		hwpwrctrl_capable;

	uint	phycal_nslope;		/* time of last nrssislope calibration */
	uint	phycal_noffset;		/* time of last nrssioffset calibration */
	uint	phycal_mlo;		/* last time measurelow calibration was done */
	uint	phycal_txpower;		/* last time txpower calibration was done */

	uint	interference_mode_crs_time; /* time at which crs was turned off */
	uint16	crsglitch_prev;		/* crsglitch count at last watchdog */
	bool	interference_mode_crs;	/* aphy crs state for interference mitigation mode */
	uint	aci_start_time;		/* adjacent channel interference start time */
	int	aci_exit_check_period;
	uint	aci_state;
	int	rssi_ma;		/* RSSI moving average total */
	uint16	rssi_ma_count;	/* RSSI moving average count (maximum MA_WINDOW_SZ) */
	uint16	rssi_ma_win_sz;	/* RSSI moving average window size maximum MA_WINDOW_SZ) */
	int	rssi_window[MA_WINDOW_SZ];     	/* RSSI moving average window */
	int	rssi_index;		/* RSSI moving average window index */

	uint32  phy_tx_tone_freq;
	uint	phy_lastcal;   /* last time PHY periodic calibration ran */
	bool 	phy_forcecal; /* run calibration at the earliest opportunity */
	uint32 xtalfreq; /* Xtal frequency */
	uint8 pdiv;
	int8 carrier_suppr_disable;	/* disable carrier suppression */

	/* SSLPNPHY */
	uint16 sslpnphy_bphyctrl;
	uint8 sslpnphy_full_cal_channel[2];
	uint16 sslpnphy_full_cal_chanspec[2];
	uint8 sslpnphy_rc_cap;

	uint8 sslpnphy_tr_isolation_mid;	/* TR switch isolation for each sub-band */
	uint8 sslpnphy_tr_isolation_low;
	uint8 sslpnphy_tr_isolation_hi;

	uint8 sslpnphy_bx_arch;		/* Board switch architecture */

	uint8 sslpnphy_rx_power_offset;	 /* Input power offset */
	uint8 sslpnphy_rx_power_offset_5g; /* Input power offset 5G */
	uint16 sslpnphy_rxpo2gchnflg;
	uint8 sslpnphy_fabid; /* 0 is default ,1 is CSM, 2 is TSMC etc */
	uint16 sslpnphy_fabid_otp;

#ifdef BAND5G
	uint8 sslpnphy_rssi_5g_vf;	/* 5G RSSI Vmid fine */
	uint8 sslpnphy_rssi_5g_vc; /* 5G RSSI Vmid coarse */
	uint8 sslpnphy_rssi_5g_gs; /* 5G RSSI gain select */
#endif
	uint8 sslpnphy_rssi_vf;	/* RSSI Vmid fine */
	uint8 sslpnphy_rssi_vc; /* RSSI Vmid coarse */
	uint8 sslpnphy_rssi_gs; /* RSSI gain select */
	uint8 sslpnphy_tssi_val; /* tssi value */
	uint8 sslpnphy_auxadc_val; /* aux adc  value */
	uint8 sslpnphy_rssi_vf_lowtemp;	/* RSSI Vmid fine */
	uint8 sslpnphy_rssi_vc_lowtemp; /* RSSI Vmid coarse */
	uint8 sslpnphy_rssi_gs_lowtemp; /* RSSI gain select */

	uint8 sslpnphy_rssi_vf_hightemp;	/* RSSI Vmid fine */
	uint8 sslpnphy_rssi_vc_hightemp;	/* RSSI Vmid coarse */
	uint8 sslpnphy_rssi_gs_hightemp;	/* RSSI gain select */

	uint16 sslpnphy_tssi_tx_cnt; /* Tx frames at that level for NPT calculations */
	uint16 sslpnphy_tssi_idx;	/* Estimated index for target power */
	uint16 sslpnphy_tssi_npt;	/* NPT for TSSI averaging */

	uint16 sslpnphy_target_tx_freq;	/* Target frequency for which LUT's were calculated */
	uint16 sslpnphy_last_tx_freq;
	int8 sslpnphy_tx_power_idx_override; /* Forced tx power index */
	uint16 sslpnphy_noise_samples;

	uint32 sslpnphy_papdRxGnIdx;
	uint32 sslpnphy_papd_rxGnCtrl_init;

	uint32  sslpnphy_gain_idx_14_lowword;
	uint32  sslpnphy_gain_idx_14_hiword;
	uint32  sslpnphy_gain_idx_27_lowword;
	uint32  sslpnphy_gain_idx_27_hiword;
	int16   sslpnphy_ofdmgainidxtableoffset;  /* reference ofdm gain index table offset */
	int16   sslpnphy_dsssgainidxtableoffset;  /* reference dsss gain index table offset */
	uint32  sslpnphy_tr_R_gain_val;  /* reference value of gain_val_tbl at index 64 */
	uint32  sslpnphy_tr_T_gain_val;	/* reference value of gain_val_tbl at index 65 */
	int8  sslpnphy_input_pwr_offset_db;
	uint16  sslpnphy_Med_Low_Gain_db;
	uint16  sslpnphy_Very_Low_Gain_db;
	int8    sslpnphy_lastsensed_temperature;
	int8	sslpnphy_pkteng_rssi_slope;
	uint8	sslpnphy_saved_tx_user_target[TXP_NUM_RATES];
	uint8   sslpnphy_volt_winner;
	uint8   sslpnphy_volt_low;
	uint8 	sslpnphy_54_48_36_24mbps_backoff;
	uint8 	sslpnphy_11n_backoff;
	uint8 	sslpnphy_lowerofdm;
	uint8	sslpnphy_cck;
	uint8 	sslpnphy_psat_2pt3_detected;
	int32 	sslpnphy_lowest_Re_div_Im;
	int32 	sslpnphy_max_gain;
	int32	sslpnphy_lowest_gain_diff;
	int8 	sslpnphy_final_papd_cal_idx;
	uint16 sslpnphy_extstxctrl4;
	uint16 sslpnphy_extstxctrl5;
	uint16 sslpnphy_extstxctrl0;
	uint16 sslpnphy_extstxctrl1;
	uint16 sslpnphy_extstxctrl3;
	uint16 sslpnphy_extstxctrl2;
	uint16 sslpnphy_filt_bw;
	uint16 sslpnphy_ofdm_filt_bw;
	/* Periodic cal flags */
	int8 sslpnphy_last_cal_temperature;
	int8 sslpnphy_last_full_cal_temperature;
	int32 sslpnphy_last_cal_voltage;
	uint sslpnphy_vbat_ripple;
	uint sslpnphy_force_1_idxcal;
	uint8 sslpnphy_papd_nxt_cal_idx;
	uint sslpnphy_recal;
	uint8 sslpnphy_last_cal_tia_gain;
	uint8 sslpnphy_last_cal_lna2_gain;
	uint8 sslpnphy_last_cal_vga_gain;
	int8 sslpnphy_last_cal_tx_idx;
	uint sslpnphy_percal_ctr;
	uint8 sslpnphy_tx_idx_prev_cal;
	uint sslpnphy_txidx_drift;
	uint8 sslpnphy_cur_idx;
	uint sslpnphy_force_percal;
	uint sslpnphy_volt_track;

	uint8 sslpnphy_cck_filt_sel;
	uint8 sslpnphy_ofdm_filt_sel;
	bool sslpnphy_OLYMPIC;
	bool sslpnphy_vbat_ripple_check;

	/* uCode Dynamic Noise Cal Flags */
	uint8   Listen_GaindB_BfrNoiseCal;
	uint8   Listen_GaindB_BASE;
	uint8   Listen_GaindB_AfrNoiseCal;
	uint8   Listen_RF_Gain;
	uint16  NfSubtractVal_BfrNoiseCal;
	uint16  NfSubtractVal_BASE;
	uint16  NfSubtractVal_AfrNoiseCal;
	int8    RxpowerOffset_Required_BASE;
	int8    rxpo_required_AfrNoiseCal;
	/* Will be TRUE After FirstTime NoiseCal Happens in a Channel */
	bool    sslpnphy_init_noise_cal_done;
	uint32 sslpnphy_noisepwr_fifo_Min[SSLPNPHY_NOISE_PWR_FIFO_DEPTH];
	uint32 sslpnphy_noisepwr_fifo_Max[SSLPNPHY_NOISE_PWR_FIFO_DEPTH];
	uint8  sslpnphy_noisepwr_fifo_filled;
	uint8  sslpnphy_NPwr_MinLmt;
	uint8  sslpnphy_NPwr_MaxLmt;
	uint8  sslpnphy_NPwr_LGC_MinLmt;
	uint8  sslpnphy_NPwr_LGC_MaxLmt;
	bool   sslpnphy_disable_noise_percal;
	uint sslpnphy_last_noise_cal;
	uint sslpnphy_last_idletssi_cal;
	uint16 sslpnphy_noise_measure_window;
	uint8  sslpnphy_max_listen_gain_change_lmt;
	uint8  sslpnphy_max_rxpo_change_lmt;

	/* used for debug */
	uint8 	sslpnphy_psat_pwr;
	uint8 	sslpnphy_psat_indx;
	int32	sslpnphy_min_phase;
	uint8	sslpnphy_final_idx;
	uint8	sslpnphy_start_idx;
	uint8	sslpnphy_current_index;
	uint16 sslpnphy_logen_buf_1;
	uint16 sslpnphy_local_ovr_2;
	uint16 sslpnphy_local_oval_6;
	uint16 sslpnphy_local_oval_5;
	uint16 sslpnphy_logen_mixer_1;
	uint sslpnphy_dummy_tx_done;
	bool sslpnphy_radio_classA;
	bool sslpnphy_papd_tweaks_enable;
	struct {
		uint32	final_idx_thresh;
		uint16	papd_track_pa_lut_begin;
		uint16	papd_track_pa_lut_step;
		uint16  min_final_idx_thresh;
	} sslpnphy_papd_tweaks;

	uint8	sslpnphy_aci_stat;
	uint	sslpnphy_aci_start_time;
	int8	sslpnphy_tx_power_offset[TXP_NUM_RATES];		/* Offset from base power */
	bool	sslpnphy_papd_cal_done;

	bool pkteng_in_progress;
	int8 sslpnphy_tssi_max_pwr_limit;
	uint sslpnphy_restore_papd_cal_results;
	struct {
		uint8 bb_mult_old;
		uint16 Core1TxControl_old, sslpnCtrl3_old;
		uint16 pa_sp1_old_5_4, rxbb_ctrl2_old;
		uint16 rf_common_03_old, rf_grx_sp_1_old;
		uint16 SSLPNPHY_sslpnCalibClkEnCtrl_old;
		uint16 lpfbwlut0, lpfbwlut1, rf_txbb_sp_3, rf_pa_ctrl_14;
		uint8 CurTxGain;
		} sslpnphy_store;

	struct {
		/* TX IQ LO cal results */
		uint16 txiqlocal_bestcoeffs[11];
		uint txiqlocal_bestcoeffs_valid;
		/* PAPD results */
		uint32 papd_compdelta_tbl[64];
		uint papd_table_valid;
		uint16 analog_gain_ref, lut_begin,
			lut_end, lut_step, rxcompdbm, papdctrl;

		/* RX IQ cal results */
		uint16 rxiqcal_coeffa0;
		uint16 rxiqcal_coeffb0;
		uint16 rxiq_enable;
		uint8 rxfe;
		uint8 loopback2, loopback1;

	} sslpnphy_cal_results[2];

	uint16 sslpnphy_end_location;
	uint16 sslpnphy_start_location;
	uint8 sslpnphy_ant1_max_pwr;
	int8 sslpnphy_cga_5g[24];
	int8 sslpnphy_cga_2g[14];

	struct wl_timer *phycal_timer;	/* timer for multiphase cal, can be generalized later */

	/* variables for saving classifier and clip detect registers */
	uint16 classifier_state;
	uint16 clip_state[2];
	uint nphy_deaf_count;
	uint8 rxiq_samps;
	uint8 rxiq_antsel;

#ifdef PR43338WAR
	bool	war_b_ap;
	uint16	war_b_ap_cthr_save;	/* b only AP WAR to cache cthr bit 14 */
#endif

	uint16 rfctrlIntc1_save;
	uint16 rfctrlIntc2_save;
	bool idle_tssi_meas_reqd;
	uint16 tx_rx_cal_radio_saveregs[22];
	uint16 tx_rx_cal_phy_saveregs[10];
	bool disable_percal;	/* phy agnostic iovar to disable watchdog driven cals */
	int16 lpphy_ofdm_dig_filt_type;
	int lpphy_txrf_sp_9_override;
	bool restore_lpphy_clsfr;
	uint16 lpphy_clsfrCtrl_old;
	uint8 lpphy_tssi_pwr_limit; /* Maximum power supported by the TSSI table */
	int16 ofdm_analog_filt_bw_override;
	int16 cck_analog_filt_bw_override;
	int16 ofdm_rccal_override;
	int16 cck_rccal_override;
	uint16 extlna_type;
	uint32 *sslpnphy_papdIntlut;
	uint8 *sslpnphy_papdIntlutVld;
	uint32 *sslpnphy_gain_idx_2g;
	uint32 *sslpnphy_gain_idx_5g;
	uint16 *sslpnphy_gain_tbl_2g;
	uint16 *sslpnphy_gain_tbl_5g;
	uint16 *sslpnphy_swctrl_lut_2g;
	uint16 *sslpnphy_swctrl_lut_5g;
	sslpnphy_tx_gain_tbl_entry * sslpnphy_tx_gaintbl_5GHz_midband;
	sslpnphy_tx_gain_tbl_entry * sslpnphy_tx_gaintbl_5GHz_hiband;
	/* Estimated per-channel index for target power */
	uint8 sslpnphy_tssi_idx_ch[LAST_5G_CHAN + 1];
	uint8 sslpnphy_ch_cache;
#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
	ch_calcache_t *calcache; /* Head of the list */
	bool calcache_on;
#endif
	phy_cal_info_t def_cal_info;	/* Default cal info (not allocated) */
	phy_cal_info_t *cal_info;	/* Multiple instances of Multi-phase support */
	int8 sslpnphy_tssi_min_pwr_limit;
#ifdef DONGLEOVERLAYS
	/* needed to sequentially init/cal on both bands */	
	uint8 phyinit_state;
#endif
	uint8 fem_combiner_target_state;
	uint8 fem_combiner_current_state;
	int8 samp_collect_agc_gain;
	int8 sslpnphy_tssi_min_pwr_nvram;
	int8 sslpnphy_tssi_max_pwr_nvram;
	bool   sslpnphy_force_noise_cal;
	int8   sslpnphy_force_lgain;
	int8   sslpnphy_force_rxpo;
	int32 sslpnphy_spbdump_pwr_dBm;
	uint32 sslpnphy_spbdump_raw_pwr;

	bool	rxiq_agc_en;
	uint8	rxiq_log2_samps; /* No.Of samples in log2 domain */
	uint8	rxiq_num_iter; /*Num. Of measurement Iterations */
	uint8	phy_rxiq_resln; /* 0--> Coarse, 1--> Fine(0.25dB steps) */
};

#if TBD_YE
/* phy state that is per device instance */
struct shared_phy {
	struct phy_info	*phy_head;      /* head of phy list */
	uint	unit;			/* device instance number */
	osl_t	*osh;			/* pointer to os handle */
	si_t	*sih;			/* si handle (cookie for siutils calls) */
	void    *physhim;		/* phy <-> wl shim layer for wlapi */
	uint	corerev;		/* d11corerev, shadow of wlc_hw->corerev */
	uint32	machwcap;		/* mac hw capability */
	bool	up;			/* main driver is up and running */
	bool	clk;			/* main driver make the clk available */
	uint	now;			/* # elapsed seconds */
	uint16	vid;			/* vendorid */
	uint16  did;			/* deviceid */
	uint	chip;			/* chip number */
	uint	chiprev;		/* chip revision */
	uint	chippkg;		/* chip package option */
	uint	sromrev;		/* srom revision */
	uint	boardtype;		/* board type */
	uint	boardrev;		/* board revision */
	uint	boardvendor;		/* board vendor */
	uint32	boardflags;		/* board specific flags from srom */
	uint32	boardflags2;		/* more board flags if sromrev >= 4 */
	uint	bustype;		/* SI_BUS, PCI_BUS  */
	uint	buscorerev; 		/* buscore rev */
	uint	fast_timer;		/* Periodic timeout for 'fast' timer */
	uint	slow_timer;		/* Periodic timeout for 'slow' timer */
	uint	glacial_timer;		/* Periodic timeout for 'glacial' timer */
	int	interference_mode;	/* interference mitigation mode */
	int	interference_mode_2G;	/* 2G interference mitigation mode */
	int	interference_mode_5G;	/* 5G interference mitigation mode */
	int	interference_mode_2G_override;	/* 2G interference mitigation mode */
	int	interference_mode_5G_override;	/* 5G interference mitigation mode */
	bool	interference_mode_override;	/* override */
	uint8	rx_antdiv;		/* .11b Ant. diversity (rx) selection override */
	int8	phy_noise_window[MA_WINDOW_SZ];	/* noise moving average window */
	uint	phy_noise_index;	/* noise moving average window index */
	uint8	hw_phytxchain;	/* HW tx chain cfg */
	uint8	hw_phyrxchain;	/* HW rx chain cfg */
	uint8	phytxchain;	/* tx chain being used */
	uint8	phyrxchain;	/* rx chain being used */
	uint8	rssi_mode;
	bool	radar;		/* radar detection: on or off */
	bool	_rifs_phy;	/* per-pkt rifs flag passed down from wlc_info */
#ifdef WLMEDIA_TXFILTER_OVERRIDE
	int txfilter_sm_override; /* TX filter spectral mask override */
#endif
};

/* %%%%%% phy_info_t */
struct phy_pub {
	uint		phy_type;		/* PHY_TYPE_XX */
	uint		phy_rev;		/* phy revision */
	uint8		phy_corenum;		/* number of cores */
	uint16		radioid;		/* radio id */
	uint8		radiorev;		/* radio revision */
	uint8		radiover;		/* radio version */

	uint		coreflags;		/* sbtml core/phy specific flags */
	uint		ana_rev;		/* analog core revision */
	bool		abgphy_encore;			/* true if chipset is encore enabled */
};
#endif /* TBD_YE */

struct phy_info_htphy;
typedef struct phy_info_htphy phy_info_htphy_t;

struct phy_info_nphy;
typedef struct phy_info_nphy phy_info_nphy_t;

struct phy_info_lcnphy;
typedef struct phy_info_lcnphy phy_info_lcnphy_t;

struct phy_info_lpphy;
typedef struct phy_info_lpphy phy_info_lpphy_t;

struct phy_info_sslpnphy;
typedef struct phy_info_sslpnphy phy_info_sslpnphy_t;

struct phy_info_abgphy;
typedef struct phy_info_abgphy phy_info_abgphy_t;

struct phy_info_n;
typedef struct phy_info_n phy_info_n_t;

struct phy_info_ht;
typedef struct phy_info_ht phy_info_ht_t;

struct phy_func_ptr {
	initfn_t init;
	initfn_t calinit;
	chansetfn_t chanset;
	initfn_t txpwrrecalc;
	longtrnfn_t longtrn;
	txiqccgetfn_t txiqccget;
	txiqccsetfn_t txiqccset;
	txloccgetfn_t txloccget;
	radioloftgetfn_t radioloftget;
	initfn_t carrsuppr;
	rxsigpwrfn_t rxsigpwr;
	detachfn_t detach;
};
typedef struct phy_func_ptr phy_func_ptr_t;

struct srom8_ppr {
	uint16	cck2gpo;	/* 2G CCK Power offset */
	uint32	ofdm2gpo;	/* 2G OFDM Power offset */
	uint32	ofdm5gpo;	/* 5G OFDM Power offset */
	uint32	ofdm5glpo;	/* 5GL OFDM Power offset */
	uint32	ofdm5ghpo;	/* 5GH OFDM Power offset */
	uint8   bw402gpo; 	/* bw40 2g power offset */
	uint8   bw405gpo; 	/* bw40 5g mid power offset */
	uint8   bw405glpo; 	/* bw40 5g low power offset */
	uint8   bw405ghpo; 	/* bw40 5g high power offset */
	uint8   cdd2gpo; 	/* cdd 2g power offset */
	uint8   cdd5gpo; 	/* cdd 5g mid power offset */
	uint8   cdd5glpo; 	/* cdd 5g low power offset */
	uint8   cdd5ghpo; 	/* cdd 5g high power offset */
	uint8   stbc2gpo; 	/* stbc 2g power offset */
	uint8   stbc5gpo; 	/* stbc 5g mid power offset */
	uint8   stbc5glpo; 	/* stbc 5g low power offst */
	uint8   stbc5ghpo; 	/* stbc 5g high power offset */
	uint8   bwdup2gpo; 	/* bwdup 2g power offset */
	uint8   bwdup5gpo; 	/* bwdup 5g mid power offset */
	uint8   bwdup5glpo; 	/* bwdup 5g low power offset */
	uint8   bwdup5ghpo; 	/* bwdup 5g high power offset */
	uint16	mcs2gpo[8];	/* mcs 2g power offset */
	uint16	mcs5gpo[8];	/* mcs 5g mid power offset */
	uint16	mcs5glpo[8];	/* mcs 5g low power offset */
	uint16	mcs5ghpo[8];	/* mcs 5g high power offset */
};

struct ppbw {
	uint32	bw20;
	uint32	bw20ul;
	uint32	bw40;
};

struct srom9_ppr {
	uint16	cckbw202gpo;	/* 2G CCK Power offset */
	uint16	cckbw20ul2gpo;	/* 2G CCK 20UL Power offset */
	struct ppbw ofdm2g;
	struct ppbw ofdm5gl;
	struct ppbw ofdm5gm;
	struct ppbw ofdm5gh;
	struct ppbw mcs2g;
	struct ppbw mcs5gl;
	struct ppbw mcs5gm;
	struct ppbw mcs5gh;
	uint16	mcs32po;	/* MCS32 power offset */
	uint16	ofdm40duppo;	/* OFMD DUP power offset */
};

typedef struct _nphy_txgains {
	uint16 txlpf[2];
	uint16 txgm[2];
	uint16 pga[2];
	uint16 pad[2];
	uint16 ipa[2];
} nphy_txgains_t;

typedef struct _nphy_txpwrindex {
	int8   index;
	int8   index_internal;     /* store initial or user specified txpwr index */
	int8   index_internal_save;
	uint16 AfectrlOverride;
	uint16 AfeCtrlDacGain;
	uint16 rad_gain;
	uint8  bbmult;
	uint16 iqcomp_a;
	uint16 iqcomp_b;
	uint16 locomp;
} phy_txpwrindex_t;

/* This structure is used to save/modify/restore the noise vars for specific tones */
typedef struct _nphy_noisevar_buf {
	int bufcount;   /* number of valid entries in the buffer */
	int tone_id[PHY_NOISEVAR_BUFSIZE];
	uint32 noise_vars[PHY_NOISEVAR_BUFSIZE];
	uint32 min_noise_vars[PHY_NOISEVAR_BUFSIZE];
} phy_noisevar_buf_t;

typedef struct _nphy_pwrctrl {
	int8    max_pwr_2g;
	int8    idle_targ_2g;
	int16   pwrdet_2g_a1;
	int16   pwrdet_2g_b0;
	int16   pwrdet_2g_b1;
	int8    max_pwr_5gm;
	int8    idle_targ_5gm;
	int8    max_pwr_5gh;
	int8    max_pwr_5gl;
	int16   pwrdet_5gm_a1;
	int16   pwrdet_5gm_b0;
	int16   pwrdet_5gm_b1;
	int16   pwrdet_5gl_a1;
	int16   pwrdet_5gl_b0;
	int16   pwrdet_5gl_b1;
	int16   pwrdet_5gh_a1;
	int16   pwrdet_5gh_b0;
	int16   pwrdet_5gh_b1;
	int8    idle_targ_5gl;
	int8    idle_targ_5gh;
	int8    idle_tssi_2g;
	int8    idle_tssi_5g;
	int8    idle_tssi;
	int16   a1;
	int16   b0;
	int16   b1;
} phy_pwrctrl_t;

typedef struct {
	uint16 rssical_radio_regs_2G[2];
	uint16 rssical_phyregs_2G[12];

	uint16 rssical_radio_regs_5G[2];
	uint16 rssical_phyregs_5G[12];
} rssical_cache_t;

typedef struct {
	/* calibration coefficients for the last 2 GHz channel that was calibrated */
	uint16 txcal_coeffs_2G[8];
	uint16 txcal_radio_regs_2G[8];
	nphy_iq_comp_t rxcal_coeffs_2G;

	/* calibration coefficients for the last 5 GHz channel that was calibrated */
	uint16 txcal_coeffs_5G[8];
	uint16 txcal_radio_regs_5G[8];
	nphy_iq_comp_t rxcal_coeffs_5G;
} txiqcal_cache_t;


#if TBD_YE
struct phy_info {
	wlc_phy_t	pubpi_ro;	/* public attach time constant phy state */
	shared_phy_t	*sh;		/* shared phy state pointer */
	phy_func_ptr_t	pi_fptr;
	void		*pi_ptr;

	union {
		phy_info_lcnphy_t *pi_lcnphy;
		phy_info_lpphy_t *pi_lpphy;
		phy_info_sslpnphy_t *pi_sslpnphy;
		phy_info_abgphy_t *pi_abgphy;
		phy_info_n_t *pi_nphy;
		phy_info_ht_t *pi_htphy;
	} u;
	bool	user_txpwr_at_rfport;

#ifdef WLNOKIA_NVMEM
	void	*noknvmem;
#endif /* WLNOKIA_NVMEM */
	d11regs_t	*regs;
	struct phy_info	*next;
	char		*vars;			/* phy attach time only copy of vars */
	wlc_phy_t	pubpi;			/* private attach time constant phy state */

	bool		do_initcal;		/* to enable/disable phy init cal */
	bool		phytest_on;		/* whether a PHY test is running */
	bool		ofdm_rateset_war;	/* ofdm rateset war */

	chanspec_t	radio_chanspec;		/* current radio chanspec */
	uint8		antsel_type;		/* Type of boardlevel mimo antenna switch-logic
						 * 0 = N/A, 1 = 2x4 board, 2 = 2x3 CB2 board
						 */
	uint16		bw;			/* b/w (10, 20 or 40) [only 20MHZ on non NPHY] */
	uint8		txpwr_percent;		/* power output percentage */
	bool		phy_init_por;		/* power on reset prior to phy init call */

	bool		init_in_progress;	/* init in progress */
	bool		initialized;		/* Have we been initialized ? */
	uint		refcnt;
	bool		phywatchdog_override;	/* to disable/enable phy watchdog */
	uint8		phynoise_state;		/* phy noise sample state */
	uint		phynoise_now;		/* timestamp to run sampling */
	int		phynoise_chan_watchdog;	/* sampling target channel for watchdog */
	bool		phynoise_polling;	/* polling or interrupt for sampling */
	bool		disable_percal;		/* phy agnostic iovar to disable watchdog cals */
	mbool		measure_hold;		/* should we hold off measurements/calibrations */

/* NVRAM values */
	/* PA tssi coefficiencies */
	int16		txpa_2g[PWRTBL_NUM_COEFF];		/* 2G: pa0b%d */
	int16		txpa_2g_lo[PWRTBL_NUM_COEFF];		/* For 2nd LUT */
	int16		txpa_2g_low_temp[PWRTBL_NUM_COEFF];	/* low temperature  */
	int16		txpa_2g_high_temp[PWRTBL_NUM_COEFF];	/* high temperature */
	int16		txpa_5g_low[PWRTBL_NUM_COEFF];		/* 5G low: pa1lob%d */
	int16		txpa_5g_mid[PWRTBL_NUM_COEFF];		/* 5G mid: pa1b%d   */
	int16		txpa_5g_hi[PWRTBL_NUM_COEFF];		/* 5G hi:  pa1hib%d */
	int8		pmin;                               /* pwrMin_range2 */
	int8		pmax;                               /* pwrMax_range2 */

	/* srom max board value (.25 dBm) */
	uint8		tx_srom_max_2g;				/* 2G:     pa0maxpwr   */
	uint8		tx_srom_max_5g_low;			/* 5G low: pa1lomaxpwr */
	uint8		tx_srom_max_5g_mid;			/* 5G mid: pa1maxpwr   */
	uint8		tx_srom_max_5g_hi;			/* 5G hi:  pa1himaxpwr */
	uint8		tx_srom_max_rate_2g[TXP_NUM_RATES];	/* 2.4G channels */
	uint8		tx_srom_max_rate_5g_low[TXP_NUM_RATES];	/* 5G mid band channels */
	uint8		tx_srom_max_rate_5g_mid[TXP_NUM_RATES];	/* 5G low band channels */
	uint8		tx_srom_max_rate_5g_hi[TXP_NUM_RATES];	/* 5G hi band channels */
	uint8		tx_user_target[TXP_NUM_RATES];		/* Current user override target */
	int8		tx_power_offset[TXP_NUM_RATES];		/* Offset from base power */
	uint8		tx_power_target[TXP_NUM_RATES];		/* Current target power */

	uint8	elna2g;
	uint8	elna5g;
#if TBD_YE
	srom_fem_t	srom_fem2g;		/* 2G band FEM attributes */
	srom_fem_t	srom_fem5g;		/* 5G band FEM attributes */
#endif
/* PHY states */
	uint8		tx_power_max;		/* max target power among all rates on this channel.
						 * Power offsets use this.
						 */
	uint8		tx_power_max_rate_ind;  /* Index of the rate amongst the TXP_NUM_RATES rates
						 * with the max target power on this channel. If
						 * multiple rates have the max target power, the
						 * lowest rate index among them is reported.
						 */
	bool		hwpwrctrl;		/* ucode controls txpower instead of driver */
	uint8		nphy_txpwrctrl;		/* tx power control setting */
	int8		nphy_txrx_chain;	/* chain override for both TX & RX */
	bool		phy_5g_pwrgain;	/* flag to indicate 5G Power Gain is enabled */

	uint16		phy_wreg;
	uint16		phy_wreg_limit;


	int8		n_preamble_override;	/* preamble override for both TX & RX, both band */
	uint8		antswitch;              /* Antswitch field from SROM */
	uint8           aa2g, aa5g;             /* antennas available for 2G, 5G */


	int8		idle_tssi[CH_5G_GROUP];		/* Measured idle_tssi for lo,mid,hi */

	int8		txpwr_est_Pout;			/* Best guess at current txpower */
	uint8		tx_power_min;			/* min power for all rates on this chan */
	uint8		txpwr_limit[TXP_NUM_RATES];	/* regulatory power limit */
	uint8		txpwr_env_limit[TXP_NUM_RATES];	/* env based txpower per rate limit */
	uint8		ldo_voltage;			/* valid iff lpphy rev0 */
	bool		japan_wide_filter;		/* valid iff lpphy */
	bool		channel_14_wide_filter;		/* used in abg phy code */
	bool		txpwroverride;			/* override */
	int16		radiopwr_override;		/* phy PWR_CTL override, -1=default */
	uint16		hwpwr_txcur;			/* hwpwrctl: current tx power index */
	uint8		saved_txpwr_idx;		/* saved current hwpwrctl txpwr index */
	interference_info_t interf;

	bool	edcrs_threshold_lock;	/* lock the edcrs detection threshold */

#if defined(AP) && defined(RADAR)
	radar_work_t	radar_work;		/* radar work area */
	phy_radar_detect_mode_t rdm;            /* current radar detect mode FCC/EU */
#endif
	radar_params_t	*rparams;
	radar_params_t	rargs[2];	/* different parameters for 20Mhz, 40Mhz channel */

	uint32	tr_R_gain_val;	/* reference value of gain_val_tbl at index 64 */
	uint32	tr_T_gain_val;	/* reference value of gain_val_tbl at index 65 */

	int16	ofdm_analog_filt_bw_override;
	int16	cck_analog_filt_bw_override;
	int16	ofdm_rccal_override;
	int16	cck_rccal_override;
	uint16	extlna_type;

	uint	interference_mode_crs_time; /* time at which crs was turned off */
	uint16	crsglitch_prev;		/* crsglitch count at last watchdog */
	bool	interference_mode_crs;	/* aphy crs state for interference mitigation mode */
	uint	aci_start_time;		/* adjacent channel interference start time */
	int	aci_exit_check_period;
	uint	aci_state;

	uint32  phy_tx_tone_freq;
	uint	phy_lastcal;   /* last time PHY periodic calibration ran */
	bool 	phy_forcecal; /* run calibration at the earliest opportunity */
	bool    phy_fixed_noise; /* flag to report PHY_NOISE_FIXED_VAL noise */
	uint32	xtalfreq; /* Xtal frequency */
	uint8	pdiv;
	int8	carrier_suppr_disable;	/* disable carrier suppression */

	bool	phy_bphy_evm;     /* flag to report if continuous CCK transmission is ON/OFF */
	bool	phy_bphy_rfcs;  /* flag to report if nphy BPHY RFCS testpattern is ON/OFF */
	int8	phy_scraminit;
	uint16	phy_gpiosel;

	/* tempsense */
	phy_txcore_temp_t txcore_temp;
	int8	phy_tempsense_offset;

/* ABGPHY */
	uint16		radiopwr;		/* radio power */

	/* wlc_gphy_measurelo: */
	uint16		mintxbias;
	lo_complex_abgphy_info_t	gphy_locomp_iq[STATIC_NUM_RF][STATIC_NUM_BB];
	int8		stats_11b_txpower[STATIC_NUM_RF][STATIC_NUM_BB]; /* settings in use */

	/* nrssi calibration: */
	uint16		rc_cal;
	int		nrssi_table_delta;
	int		nrssi_slope_offset;

	/* 11a Power control */
	int8		txpwridx;
	uint8		min_txpower;	/* minimum allowed tx power */

	/* high channels in a band to be disabled for srom ver 1 */
	uint8		a_band_high_disable;

	/* 11a LOFT */
	uint16		tx_vos;

	/* measlo for 11b */
	int		rf_max;
	int		bb_max;

	/* tssi to dbm translation table */
	uint8		*hwtxpwr;

	/* Original values of the PHY regs, that we modified, to increase the freq tracking b/w */
	uint16		freqtrack_saved_regs [2];
	int		cur_interference_mode;	/* Track interference mode of phy */
	bool 		hwpwrctrl_capable;
	bool 		temppwrctrl_capable;

	uint	phycal_nslope;		/* time of last nrssislope calibration */
	uint	phycal_noffset;		/* time of last nrssioffset calibration */
	uint	phycal_mlo;		/* last time measurelow calibration was done */
	uint	phycal_txpower;		/* last time txpower calibration was done */

	bool	pkteng_in_progress;
	uint8   phy_aa2g;
/* NPHY */
	bool	nphy_tableloaded;	/* one time nphy static tables downloaded to PHY */
	int8	nphy_rssisel;
	phy_txpwrindex_t nphy_txpwrindex[PHY_CORE_NUM_2]; /* independent override per core */
	phy_pwrctrl_t nphy_pwrctrl_info[PHY_CORE_NUM_2]; /* Tx pwr ctrl info per Tx chain */
	union {
		struct srom8_ppr n;
		struct srom9_ppr ht;
	} ppr;

	int8	phy_spuravoid;      /* spur avoidance, 0: disable, 1: auto, 2: on, 3: on2 */
	bool	phy_isspuravoid;    /* TRUE if spur avoidance is ON for the current channel,
								 * else FALSE
								 */
	uint8	phy_pabias;	    /* override PA bias value, 0: no override */
	uint8	nphy_papd_skip;     /* skip papd calibration for IPA case */
	uint8	nphy_tssi_slope;    /* TSSI pwr detector slope */

	int16	phy_noise_win[PHY_CORE_MAX][PHY_NOISE_WINDOW_SZ]; /* noise per antenna */
	uint8	phy_noise_index;	/* noise moving average window index */

	bool	nphy_gain_boost; /* flag to reduce 2055 NF via higher LNA gain */
	bool	nphy_elna_gain_config; /* flag to reduce Rx gains for external LNA */
	uint16	old_bphy_test;
	uint16	old_bphy_testcontrol;

	/* nphy calibration */
	bool	rssical_nphy;		/* enable/disable nphy rssical(for rev3 only for now) */
	uint8	cal_type_override;      /* cal override set from command line */
	phy_cal_info_t def_cal_info;	/* Default cal info (not allocated) */
	phy_cal_info_t *cal_info;	/* Multiple instances of Multi-phase support */

	chanspec_t	nphy_iqcal_chanspec_2G;	/* 0: invalid, other: last valid cal chanspec */
	chanspec_t	nphy_iqcal_chanspec_5G;	/* 0: invalid, other: last valid cal chanspec */
	chanspec_t	nphy_rssical_chanspec_2G; /* 0: invalid, other: last valid cal chanspec */
	chanspec_t	nphy_rssical_chanspec_5G; /* 0: invalid, other: last valid cal chanspec */
	struct wlapi_timer *phycal_timer; /* timer for multiphase cal, can be generalized later */
	bool use_int_tx_iqlo_cal_nphy; /* Flag to determine if the Tx IQ/LO cal should be performed
					* using the radio's internal envelope detectors.
					*/
	bool internal_tx_iqlo_cal_tapoff_intpa_nphy; /*	Flag to determine whether the internal Tx
							* IQ/LO cal would be use the signal from the
							* radio's internal envelope detector at the
							* PAD tapoff or the intPA tapoff point.
							*/	

#if defined(WLMCHAN) || defined(PHYCAL_CACHING)
	ch_calcache_t *calcache; /* Head of the list */
	uint8 calcache_num;			/* Indicates the num of active contexts */
	bool calcache_on;
#endif

	txiqcal_cache_t 		calibration_cache;
	rssical_cache_t			rssical_cache;

	uint8	nphy_txpwr_idx_2G[3];    /* to store the power control txpwr index for 2G band */
	uint8	nphy_txpwr_idx_5G[3];    /* to store the power control txpwr index for 5G band */
	uint8	nphy_papd_cal_type;
	int16	nphy_papd_epsilon_offset[2];
	bool	nphy_papd_recal_enable;
	bool	ipa2g_on;   /* using 2G internal PA */
	bool	ipa5g_on;   /* using internal PA */

	/* variables for saving classifier and clip detect registers */
	uint16	classifier_state;
	uint16	clip_state[2];
	uint8	rxiq_samps;
	uint8	rxiq_antsel;

#ifdef PR43338WAR
	bool	war_b_ap;
	uint16	war_b_ap_cthr_save;	/* b only AP WAR to cache cthr bit 14 */
#endif
	bool	first_cal_after_assoc;
	uint16	tx_rx_cal_phy_saveregs[16];
	txcal_coeffs_t txcal_cache[PHY_CORE_MAX];
	uint16	txcal_cache_cookie;

	/* buffers used for ch11 40MHz spur avoidance WAR for 4322 */
	bool	nphy_gband_spurwar_en;
	bool	nphy_gband_spurwar2_en;
	bool	nphy_aband_spurwar_en;
	phy_noisevar_buf_t	nphy_saved_noisevars;

	/* new flag to signal periodic_cal is running to blank radar */
	uint16 radar_percal_mask;
	bool dfs_lp_buffer_nphy;		/* enable/disable clearing DFS LP buffer */

	/* Entry Id in Rx2Tx seq table that has the CLR_RXTX_BIAS opcode */
	int8	rx2tx_biasentry;

	uint16	crsminpwr0;
	uint16	crsminpwrl0;
	uint16	crsminpwru0;
	int16	noise_crsminpwr_index;
	uint16	init_gain_core1;
	uint16	init_gain_core2;
	uint16	init_gainb_core1;
	uint16	init_gainb_core2;
	uint8	aci_noise_curr_channel;
	uint16	init_gain_rfseq[4];

	bool	radio_is_on;

	/* Phy table addr/data register offsets */
	uint16 tbl_data_hi;
	uint16 tbl_data_lo;
	uint16 tbl_addr;
	/* Phy table addr/data split access state */
	uint tbl_save_id;
	uint tbl_save_offset;

/* PHY independent */
	uint8	txpwrctrl; /* tx power control setting */
	int8    txpwrindex[PHY_CORE_MAX]; /* index if hwpwrctrl if OFF */
	bool    btclock_tune;			/* WAR to stabilize btclock  */
	bool    bt_active;
	uint16  bt_period;
	uint16  bt_shm_addr;
/* debug */


	uint8	phy_cal_mode;		/* periodic calibration mode: disable, single, mphase */
	chanspec_t last_cal_chanspec;
	uint8      radar_cal_active; /* to mask radar detect during cal's tone-play */


	uint8 phycal_tempdelta; /* temperature delta below which phy calibration will not run */
	uint32  mcs20_po;
	uint32  mcs40_po;
};

#endif /* TBD_YE */

/* %%%%%% shared functions */

typedef int32    fixed;	/* s15.16 fixed-point */

typedef struct _cint32 {
	fixed	q;
	fixed	i;
} cint32;

typedef struct radio_regs {
    uint16 address;
    uint32 init_a;
    uint32 init_g;
    uint8  do_init_a;
    uint8  do_init_g;
} radio_regs_t;

/* radio regs that do not have band-specific values */
typedef struct radio_20xx_regs {
	uint16 address;
	uint8  init;
	uint8  do_init;
} radio_20xx_regs_t;

typedef struct sslpnphy_radio_regs {
    uint16 address;
    uint8 init_a;
    uint8 init_g;
} sslpnphy_radio_regs_t;
typedef struct lcnphy_radio_regs {
    uint16 address;
    uint8 init_a;
	uint8 init_g;
	uint8 do_init_a;
	uint8 do_init_g;
} lcnphy_radio_regs_t;
typedef struct {
	uint32 iq_prod;
	uint32 i_pwr;
	uint32 q_pwr;
} sslpnphy_iq_est_t;


extern radio_regs_t regs_2063_rev0[];
extern radio_regs_t regs_2062[];
extern radio_regs_t regs_2063_rev0[];
extern radio_regs_t regs_2063_rev1[];
extern sslpnphy_radio_regs_t sslpnphy_radio_regs_2063[];
extern lcnphy_radio_regs_t lcnphy_radio_regs_2064[];
extern lcnphy_radio_regs_t lcnphy_radio_regs_2066[];
extern radio_regs_t regs_2055[], regs_SYN_2056[], regs_TX_2056[], regs_RX_2056[];
extern radio_regs_t regs_SYN_2056_A1[], regs_TX_2056_A1[], regs_RX_2056_A1[];
extern radio_regs_t regs_SYN_2056_rev5[], regs_TX_2056_rev5[], regs_RX_2056_rev5[];
extern radio_regs_t regs_SYN_2056_rev6[], regs_TX_2056_rev6[], regs_RX_2056_rev6[];
extern radio_regs_t regs_SYN_2056_rev7[], regs_TX_2056_rev7[], regs_RX_2056_rev7[];
extern radio_regs_t regs_SYN_2056_rev8[], regs_TX_2056_rev8[], regs_RX_2056_rev8[];
extern radio_20xx_regs_t regs_2057_rev4[], regs_2057_rev5[], regs_2057_rev5v1[];
extern radio_20xx_regs_t regs_2057_rev7[], regs_2057_rev8[], regs_2057_rev9[];
extern radio_20xx_regs_t regs_2059_rev0[];


/* %%%%%% utilities */




#ifdef PHYHAL
extern char *phy_getvar(phy_info_t *pi, const char *name);
extern int phy_getintvar(phy_info_t *pi, const char *name);
#define PHY_GETVAR(pi, name)	phy_getvar(pi, name)
#define PHY_GETINTVAR(pi, name)	phy_getintvar(pi, name)
#endif /* PHYHAL */
extern bool wlc_phy_attach_sslpnphy(phy_info_t *pi);
extern void wlc_phy_init_sslpnphy(phy_info_t *pi);
extern void wlc_phy_cal_init_sslpnphy(phy_info_t *pi);
extern uint16 read_phy_reg(phy_info_t *pi, uint16 addr);
extern void write_phy_reg(phy_info_t *pi, uint16 addr, uint16 val);
extern void and_phy_reg(phy_info_t *pi, uint16 addr, uint16 val);
extern void or_phy_reg(phy_info_t *pi, uint16 addr, uint16 val);
extern void mod_phy_reg(phy_info_t *pi, uint16 addr, uint16 mask, uint16 val);

extern uint16 read_radio_reg(phy_info_t *pi, uint16 addr);
extern void or_radio_reg(phy_info_t *pi, uint16 addr, uint16 val);
extern void and_radio_reg(phy_info_t *pi, uint16 addr, uint16 val);
extern void mod_radio_reg(phy_info_t *pi, uint16 addr, uint16 mask, uint16 val);
extern void xor_radio_reg(phy_info_t *pi, uint16 addr, uint16 mask);

extern void write_radio_reg(phy_info_t *pi, uint16 addr, uint16 val);

extern void wlc_phyreg_enter(wlc_phy_t *pih);
extern void wlc_phyreg_exit(wlc_phy_t *pih);
extern void wlc_radioreg_enter(wlc_phy_t *pih);
extern void wlc_radioreg_exit(wlc_phy_t *pih);

extern void wlc_phy_read_table(phy_info_t *pi, CONST phytbl_info_t *ptbl_info, uint16 tblAddr,
	uint16 tblDataHi, uint16 tblDatalo);
extern void wlc_phy_write_table(phy_info_t *pi, CONST phytbl_info_t *ptbl_info, uint16 tblAddr,
	uint16 tblDataHi, uint16 tblDatalo);

extern void wlc_phy_table_addr(phy_info_t *pi, uint tbl_id, uint tbl_offset,
	uint16 tblAddr, uint16 tblDataHi, uint16 tblDataLo);
extern void wlc_phy_table_data_write(phy_info_t *pi, uint width, uint32 val);
extern void write_phy_channel_reg(phy_info_t *pi, uint val);
extern void
BCMOVERLAYFN(0, wlc_sslpnphy_detection_disable)(phy_info_t *pi, bool mode);
extern void wlc_sslpnphy_percal_flags_off(phy_info_t *pi);
extern void wlc_phy_chanspec_set_sslpnphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_sslpnphy_set_chanspec_tweaks(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_sslpnphy_CmRxAciGainTbl_Tweaks(void *args);
extern void wlc_sslpnphy_rx_offset_init(phy_info_t *pi); /* ROMTERM only */
extern void wlc_sslpnphy_setchan_cal(phy_info_t *pi, int32 int_val);
extern void wlc_sslpnphy_temp_adj(phy_info_t *pi);

extern void wlc_phy_cordic(fixed theta, cint32 *val);
extern uint8 wlc_phy_nbits(int32 value);
extern uint32 wlc_phy_sqrt_int(uint32 value);
extern bool wlc_sslpnphy_btcx_override_enable(phy_info_t *pi);
extern void wlc_sslpnphy_set_tx_pwr_ctrl(phy_info_t *pi, uint16 mode);
extern void wlc_sslpnphy_full_cal(phy_info_t *pi);
extern void BCMROMOVERLAYFN(1, wlc_sslpnphy_tx_pu)(phy_info_t *pi, bool bEnable);
extern void BCMROMOVERLAYFN(1, wlc_sslpnphy_stop_tx_tone)(phy_info_t *pi);
extern void wlc_sslpnphy_start_tx_tone(phy_info_t *pi, int32 f_kHz, uint16 max_val, bool iqcalmode);
extern void wlc_sslpnphy_set_tx_pwr_by_index(phy_info_t *pi, int index);
extern void wlc_sslpnphy_txpower_recalc_target(phy_info_t *pi);
extern int wlc_get_ssn_lp_band_range(uint);
/* %%%%%% SSLPNCONF function */
extern void wlc_sslpnphy_noise_measure(phy_info_t *pi);
extern void wlc_phy_detach_sslpnphy(phy_info_t *pi);
extern void wlc_sslpnphy_get_tssi(phy_info_t *pi, int8 *ofdm_pwr, int8 *cck_pwr);
extern bool wlc_phy_tpc_isenabled_sslpnphy(phy_info_t *pi);
extern void wlc_sslpnphy_tx_pwr_update_npt(phy_info_t *pi);

extern void wlc_sslpnphy_periodic_cal_top(phy_info_t *pi); /* ROMTERM */
extern void wlc_sslpnphy_auxadc_measure(wlc_phy_t *ppi, bool readVal);	/* ROMTERM */
extern int8 wlc_sslpnphy_get_rx_pwr_offset(phy_info_t *pi);	/* ROMTERM */
extern void wlc_sslpnphy_deaf_mode(phy_info_t *pi, bool mode);

extern void wlc_sslpnphy_set_tx_locc(phy_info_t *pi, uint16 didq);
extern void wlc_sslpnphy_clear_tx_power_offsets(phy_info_t *pi);
extern void wlc_sslpnphy_rx_gain_override_enable(phy_info_t *pi, bool enable);
extern bool BCMROMOVERLAYFN(1, wlc_sslpnphy_rx_iq_est)(phy_info_t *pi, uint16 num_samps,
	uint8 wait_time, sslpnphy_iq_est_t *iq_est);
extern void wlc_sslpnphy_write_table(phy_info_t *pi, CONST phytbl_info_t *pti);
extern void wlc_sslpnphy_read_table(phy_info_t *pi, CONST phytbl_info_t *pti);
extern void wlc_sslpnphy_cck_filt_load(phy_info_t *pi, uint8 filtsel);
extern void wlc_sslpnphy_save_papd_calibration_results(phy_info_t *pi);
extern int32 wlc_sslpnphy_vbatsense(phy_info_t *pi);
extern int wlc_sslpnphy_tempsense(phy_info_t *pi);
extern void wlc_2063_vco_cal(phy_info_t *pi);
extern void wlc_sslpnphy_papd_recal(phy_info_t *ppi);
extern void wlc_sslpnphy_tx_pwr_ctrl_init(phy_info_t *pi);
extern void wlc_sslpnphy_txpwrtbl_iqlo_cal(phy_info_t *pi);
extern void wlc_sslpnphy_rxiqcal_iovar(phy_info_t *pi);
extern int wlc_sslpnphy_txpwrindex_iovar(phy_info_t *pi, int32 int_val, uint8 band_idx);
extern void wlc_sslpnphy_spbrun_iovar(phy_info_t *pi, bool bool_val);
extern void wlc_sslpnphy_spbdump_iovar(phy_info_t *pi, void *a);
extern void wlc_sslpnphy_channel_gain_adjust(phy_info_t *pi);
extern void wlc_sslpnphy_recalc_tssi2dbm_tbl(phy_info_t *pi, int32 a1, int32 b0, int32 b1);
extern void wlc_load_bt_fem_combiner_sslpnphy(phy_info_t *pi, bool force_update);
extern int32 wlc_sslpnphy_tssi2dbm(int32 tssi, int32 a1, int32 b0, int32 b1);
extern void wlc_sslpnphy_set_tx_iqcc(phy_info_t *pi, uint16 a, uint16 b);
extern void wlc_phy_watchdog_sslpnphy(phy_info_t *pi);
extern int8 wlc_sslpnphy_samp_collect_agc(phy_info_t *pi, bool agc_en);


#if defined(DONGLEOVERLAYS)
extern void wlc_sslpnphy_reset_radio_loft(phy_info_t *pi);
extern void wlc_sslpnphy_noise_measure_iovar(phy_info_t *pi);
extern void wlc_sslpnphy_percal_iovar(phy_info_t *pi, int32 int_val);
extern void wlc_sslpnphy_papd_recal_iovar(phy_info_t *pi);
extern uint32 wlc_sslpnphy_rx_signal_power(phy_info_t *pi, int32 gain_index);
extern void wlc_sslpnphy_rssi_snr_calc(phy_info_t *pi, void * hdl, int32 * rssi, int32 * snr);
extern int wlc_phy_sslpnphy_long_train(phy_info_t *pi, int channel);
#endif 

#if TBD_YE
extern void wlc_phy_txpower_update_shm(phy_info_t *pi);

extern void wlc_phy_cordic(fixed theta, cint32 *val);
extern uint8 wlc_phy_nbits(int32 value);
extern uint32 wlc_phy_sqrt_int(uint32 value);
extern void wlc_phy_compute_dB(uint32 *cmplx_pwr, int8 *p_dB, uint8 core);

extern uint wlc_phy_init_radio_regs_allbands(phy_info_t *pi, radio_20xx_regs_t *radioregs);
extern uint wlc_phy_init_radio_regs(phy_info_t *pi, radio_regs_t *radioregs,
	uint16 core_offset);

extern void wlc_phy_txpower_ipa_upd(phy_info_t *pi);

extern void wlc_phy_do_dummy_tx(phy_info_t *pi, bool ofdm, bool pa_on);
extern void wlc_phy_papd_decode_epsilon(uint32 epsilon, int32 *eps_real, int32 *eps_imag);

extern void wlc_phy_cal_perical_mphase_reset(phy_info_t *pi);
extern void wlc_phy_cal_perical_mphase_restart(phy_info_t *pi);
extern void wlc_phy_chanspec_set_sslpnphy(phy_info_t *pi, chanspec_t chanspec);

/* %%%%%% common flow function */
extern bool wlc_phy_attach_nphy(phy_info_t *pi);
extern bool wlc_phy_attach_htphy(phy_info_t *pi);
extern bool wlc_phy_attach_lpphy(phy_info_t *pi);
extern bool wlc_phy_attach_sslpnphy(phy_info_t *pi);
extern bool wlc_phy_attach_lcnphy(phy_info_t *pi);
extern bool wlc_phy_attach_abgphy(phy_info_t *pi, int bandtype);

extern void wlc_phy_detach_lcnphy(phy_info_t *pi);
extern void wlc_phy_detach_lpphy(phy_info_t *pi);

extern void wlc_phy_init_nphy(phy_info_t *pi);
extern void wlc_phy_init_htphy(phy_info_t *pi);
extern void wlc_phy_init_lpphy(phy_info_t *pi);
extern void wlc_phy_init_sslpnphy(phy_info_t *pi);
extern void wlc_phy_init_lcnphy(phy_info_t *pi);
extern void WLBANDINITFN(wlc_phy_init_aphy)(phy_info_t *pi);
extern void WLBANDINITFN(wlc_phy_init_gphy)(phy_info_t *pi);

extern void wlc_phy_cal_init_nphy(phy_info_t *pi);
extern void wlc_phy_cal_init_htphy(phy_info_t *pi);
extern void wlc_phy_cal_init_lpphy(phy_info_t *pi);
extern void wlc_phy_cal_init_sslpnphy(phy_info_t *pi);
extern void wlc_phy_cal_init_lcnphy(phy_info_t *pi);
extern void wlc_phy_cal_init_gphy(phy_info_t *pi);

extern void wlc_phy_chanspec_set_nphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_htphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_abgphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_lpphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_sslpnphy_percal_flags_off(phy_info_t *pi);
extern void wlc_phy_chanspec_set_sslpnphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_fixup_sslpnphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_lcnphy(phy_info_t *pi, chanspec_t chanspec);
extern void wlc_phy_chanspec_set_fixup_lcnphy(phy_info_t *pi, chanspec_t chanspec);
extern int  wlc_phy_channel2freq(uint channel);
extern int  wlc_phy_chanspec_freq2bandrange_lpssn(uint);
extern int  wlc_phy_chanspec_bandrange_get(phy_info_t*, chanspec_t);

extern void wlc_phy_set_tx_pwr_ctrl_lpphy(phy_info_t *pi, uint16 mode);
extern void wlc_sslpnphy_set_tx_pwr_ctrl(phy_info_t *pi, uint16 mode);
extern void wlc_lcnphy_set_tx_pwr_ctrl(phy_info_t *pi, uint16 mode);
extern int8 wlc_lcnphy_get_current_tx_pwr_idx(phy_info_t *pi);

extern void wlc_phy_txpower_recalc_target_nphy(phy_info_t *pi);
extern void wlc_phy_txpower_recalc_target_htphy(phy_info_t *pi);
extern void wlc_phy_txpower_recalc_target_lpphy(phy_info_t *pi);
extern void wlc_sslpnphy_txpower_recalc_target(phy_info_t *pi);
extern void wlc_lcnphy_txpower_recalc_target(phy_info_t *pi);
extern int wlc_lcnphy_idle_tssi_est_iovar(phy_info_t *pi, bool type);
extern void wlc_phy_txpower_recalc_target_sslpnphy(phy_info_t *pi);
extern void wlc_phy_txpower_recalc_target_lcnphy(phy_info_t *pi);
extern bool wlc_phy_cal_txpower_recalc_sw_abgphy(phy_info_t *pi);

extern bool wlc_phy_aci_scan_gphy(phy_info_t *pi);
extern void wlc_phy_aci_interf_nwlan_set_gphy(phy_info_t *pi, bool on);
extern void wlc_phy_aci_ctl_gphy(phy_info_t *pi, bool on);
extern void wlc_phy_aci_upd_nphy(phy_info_t *pi);
extern void wlc_phy_aci_ctl_nphy(phy_info_t *pi, bool enable, int aci_pwr);
extern void wlc_phy_aci_inband_noise_reduction_nphy(phy_info_t *pi, bool on, bool raise);

extern void wlc_phy_aci_sw_reset_nphy(phy_info_t *pi);
extern void wlc_phy_noisemode_reset_nphy(phy_info_t *pi);
extern void wlc_phy_acimode_reset_nphy(phy_info_t *pi); /* reset ACI mode */
extern void wlc_phy_aci_noise_upd_nphy(phy_info_t *pi);
extern void wlc_phy_acimode_upd_nphy(phy_info_t *pi);
extern void wlc_phy_noisemode_upd_nphy(phy_info_t *pi);
extern void wlc_phy_acimode_set_nphy(phy_info_t *pi, bool aci_miti_enable, int aci_pwr);
extern void wlc_phy_aci_init_nphy(phy_info_t *pi);
extern void wlc_phy_aci_enable_lpphy(phy_info_t *pi, bool on);
extern void wlc_phy_aci_upd_lpphy(phy_info_t *pi);
extern void wlc_phy_periodic_cal_lpphy(phy_info_t *pi);
extern void wlc_phy_papd_cal_txpwr_lpphy(phy_info_t *pi, bool full_cal);
extern void wlc_phy_set_deaf_lpphy(phy_info_t *pi, bool user_flag);
extern void wlc_phy_clear_deaf_lpphy(phy_info_t *pi, bool user_flag);
extern void wlc_phy_stop_tx_tone_lpphy(phy_info_t *pi);
extern void wlc_phy_start_tx_tone_lpphy(phy_info_t *pi, int32 f_kHz, uint16 max_val);
extern void wlc_phy_tx_pu_lpphy(phy_info_t *pi, bool bEnable);
extern void wlc_sslpnphy_set_tx_pwr_by_index(phy_info_t *pi, int index);
extern void wlc_sslpnphy_tx_pu(phy_info_t *pi, bool bEnable);
extern void wlc_sslpnphy_stop_tx_tone(phy_info_t *pi);

extern void wlc_sslpnphy_start_tx_tone(phy_info_t *pi, int32 f_kHz, uint16 max_val, bool iqcalmode);
extern void wlc_lcnphy_set_tx_pwr_by_index(phy_info_t *pi, int index);
extern void wlc_lcnphy_tx_pu(phy_info_t *pi, bool bEnable);
extern void wlc_lcnphy_stop_tx_tone(phy_info_t *pi);
extern void wlc_lcnphy_start_tx_tone(phy_info_t *pi, int32 f_kHz, uint16 max_val, bool iqcalmode);
extern void wlc_lcnphy_set_tx_tone_and_gain_idx(phy_info_t *pi);
extern void wlc_lcnphy_set_radio_loft(phy_info_t *pi, uint8 ei0, uint8 eq0, uint8 fi0, uint8 fq0);

extern void wlc_phy_txpower_sromlimit_get_nphy(phy_info_t *pi, uint chan, uint8 *max_pwr,
	uint8 rate_id);
extern void wlc_phy_txpower_sromlimit_get_htphy(phy_info_t *pi, uint chan, uint8 *max_pwr,
	uint8 rate_id);
extern void wlc_phy_ofdm_to_mcs_powers_nphy(uint8 *power, uint8 rate_mcs_start, uint8 rate_mcs_end,
	uint8 rate_ofdm_start);
extern void wlc_phy_mcs_to_ofdm_powers_nphy(uint8 *power, uint8 rate_ofdm_start,
	uint8 rate_ofdm_end,  uint8 rate_mcs_start);
extern bool wlc_phy_txpwr_srom_read_gphy(phy_info_t *pi);
extern bool wlc_phy_txpwr_srom_read_aphy(phy_info_t *pi);

extern uint16 wlc_lcnphy_tempsense(phy_info_t *pi, bool mode);
extern int16 wlc_lcnphy_tempsense_new(phy_info_t *pi, bool mode);
extern int8 wlc_lcnphy_tempsense_degree(phy_info_t *pi, bool mode);
extern int8 wlc_lcnphy_vbatsense(phy_info_t *pi, bool mode);
extern void wlc_phy_carrier_suppress_lcnphy(phy_info_t *pi);
extern void wlc_lcnphy_crsuprs(phy_info_t *pi, int channel);
extern void wlc_lcnphy_epa_switch(phy_info_t *pi, bool mode);
extern void wlc_2064_vco_cal(phy_info_t *pi);

/* %%%%%% common testing */


/* %%%%%% ABGPHY functions */
extern void wlc_phy_switch_radio_abgphy(phy_info_t *pi, bool on);
extern void wlc_phy_txpower_get_instant_abgphy(phy_info_t *pi, void *pwr);
extern void wlc_phy_txpower_hw_ctrl_set_abgphy(phy_info_t *pi);
extern void wlc_synth_pu_war(phy_info_t *pi, uint channel);
extern void wlc_phy_ant_rxdiv_set_abgphy(phy_info_t *pi, uint8 val);
extern void wlc_phy_detach_abgphy(phy_info_t *pi);
extern uint16 wlc_default_radiopwr_gphy(phy_info_t *pi);


extern void wlc_set_11a_txpower(phy_info_t *pi, int8 tpi, bool override);
extern void wlc_phy_cal_measurelo_gphy(phy_info_t *pi);
extern void wlc_phy_cal_txpower_stats_clr_gphy(phy_info_t *pi);
extern void wlc_phy_cal_radio2050_nrssioffset_gmode1(phy_info_t *pi);
extern void wlc_phy_cal_radio2050_nrssislope(phy_info_t *pi);
extern int8 wlc_phy_noise_sample_aphy_meas(phy_info_t *pi);
extern int8 wlc_phy_noise_sample_gphy(phy_info_t *pi);
extern int wlc_get_a_band_range(phy_info_t*);
extern int8 wlc_jssi_to_rssi_dbm_abgphy(phy_info_t *pi, int crs_state, int *jssi, int jssi_count);

/* %%%%%% LPCONF function */
#define LPPHY_TBL_ID_PAPD_EPS	0x0
#define LCNPHY_TBL_ID_PAPDCOMPDELTATBL	0x18

#define LPPHY_TX_POWER_TABLE_SIZE	128
#define LPPHY_MAX_TX_POWER_INDEX	(LPPHY_TX_POWER_TABLE_SIZE - 1)
#define LPPHY_TX_PWR_CTRL_OFF	0
#define LPPHY_TX_PWR_CTRL_SW	LPPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK
#define LPPHY_TX_PWR_CTRL_HW \
				(LPPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
				LPPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK)

extern void wlc_phy_table_write_lpphy(phy_info_t *pi, const lpphytbl_info_t *ptbl_info);
extern void wlc_phy_table_read_lpphy(phy_info_t *pi, const lpphytbl_info_t *ptbl_info);
extern bool wlc_phy_tpc_isenabled_lpphy(phy_info_t *pi);
extern uint16 wlc_phy_get_current_tx_pwr_idx_lpphy(phy_info_t *pi);
extern void wlc_phy_tx_pwr_update_npt_lpphy(phy_info_t *pi);
extern int32 wlc_phy_rx_signal_power_lpphy(phy_info_t *pi, int32 gain_index);
extern void wlc_phy_tx_dig_filt_ofdm_setup_lpphy(phy_info_t *pi, bool set_now);
extern void wlc_phy_set_tx_pwr_by_index_lpphy(phy_info_t *pi, int index);
extern void wlc_phy_aci_init_lpphy(phy_info_t *pi, bool sys);
extern void wlc_phy_get_tx_iqcc_lpphy(phy_info_t *pi, uint16 *a, uint16 *b);
extern void wlc_phy_set_tx_iqcc_lpphy(phy_info_t *pi, uint16 a, uint16 b);
extern void wlc_phy_set_tx_locc_lpphy(phy_info_t *pi, uint16 didq);
extern uint16 wlc_phy_get_tx_locc_lpphy(phy_info_t *pi);
extern void wlc_phy_get_radio_loft_lpphy(phy_info_t *pi, uint8 *ei0, uint8 *eq0,
	uint8 *fi0, uint8 *fq0);
extern void wlc_phy_set_radio_loft_lpphy(phy_info_t *pi, uint8 ei0, uint8 eq0, uint8, uint8);
extern int wlc_phy_tempsense_lpphy(phy_info_t *pi);
extern int wlc_phy_vbatsense_lpphy(phy_info_t *pi);
extern void wlc_phy_rx_gain_temp_adj_lpphy(phy_info_t *pi);
extern void wlc_phy_tx_dig_filt_cck_setup_lpphy(phy_info_t *pi, bool set_now);

extern void wlc_phy_set_tx_locc_ucode_lpphy(phy_info_t *pi, bool iscck, uint16 didq);
extern void wlc_phy_table_lock_lpphy(phy_info_t *pi);
extern void wlc_phy_table_unlock_lpphy(phy_info_t *pi);

extern void wlc_phy_get_tssi_lpphy(phy_info_t *pi, int8 *ofdm_pwr, int8 *cck_pwr);

extern void wlc_phy_radio_2062_check_vco_cal(phy_info_t *pi);

extern void wlc_phy_txpower_recalc_target(phy_info_t *pi);

extern uint32 wlc_phy_qdiv_roundup(uint32 dividend, uint32 divisor, uint8 precision);


/* %%%%%% SSLPNCONF function */
extern void wlc_sslpnphy_noise_measure(phy_info_t *pi);
extern void wlc_phy_detach_sslpnphy(phy_info_t *pi);
extern void wlc_sslpnphy_get_tssi(phy_info_t *pi, int8 *ofdm_pwr, int8 *cck_pwr);
extern bool wlc_phy_tpc_isenabled_sslpnphy(phy_info_t *pi);
extern void wlc_sslpnphy_tx_pwr_update_npt(phy_info_t *pi);

extern void wlc_sslpnphy_periodic_cal_top(phy_info_t *pi); /* ROMTERM */
extern void wlc_sslpnphy_auxadc_measure(wlc_phy_t *ppi, bool readVal);	/* ROMTERM */
extern int8 wlc_sslpnphy_get_rx_pwr_offset(phy_info_t *pi);	/* ROMTERM */
extern void wlc_sslpnphy_deaf_mode(phy_info_t *pi, bool mode);

/* sslpnphy filter control table for 40MHz */
extern CONST uint32 (fltr_ctrl_tbl_40Mhz)[];

#define SSLPNPHY_TX_POWER_TABLE_SIZE	128
#define SSLPNPHY_MAX_TX_POWER_INDEX	(SSLPNPHY_TX_POWER_TABLE_SIZE - 1)
#define SSLPNPHY_TBL_ID_TXPWRCTL 	0x07
#define SSLPNPHY_TX_PWR_CTRL_OFF	0
#define SSLPNPHY_TX_PWR_CTRL_SW		SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK
#define SSLPNPHY_TX_PWR_CTRL_HW         (SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
					SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK | \
					SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK)

extern void wlc_sslpnphy_write_table(phy_info_t *pi, const phytbl_info_t *pti);
extern void wlc_sslpnphy_read_table(phy_info_t *pi, phytbl_info_t *pti);
extern void wlc_sslpnphy_deaf_mode(phy_info_t *pi, bool mode);
extern void wlc_sslpnphy_periodic_cal_top(phy_info_t *pi);
extern void wlc_sslpnphy_periodic_cal(phy_info_t *pi);
extern bool wlc_phy_tpc_isenabled_sslpnphy(phy_info_t *pi);
extern void wlc_sslpnphy_tx_pwr_update_npt(phy_info_t *pi);
extern int32 wlc_sslpnphy_tssi2dbm(int32 tssi, int32 a1, int32 b0, int32 b1);
extern void wlc_sslpnphy_get_tx_iqcc(phy_info_t *pi, uint16 *a, uint16 *b);
extern void wlc_sslpnphy_get_tssi(phy_info_t *pi, int8 *ofdm_pwr, int8 *cck_pwr);
extern int wlc_sslpnphy_rssi_compute(phy_info_t *pi, int rssi, d11rxhdr_t *rxh);
extern void wlc_sslpnphy_txpwr_target_adj(phy_info_t *pi, uint8 *tx_pwr_target, uint8 rate);
extern int wlc_sslpnphy_txpwr_idx_get(phy_info_t *pi);
extern void wlc_sslpnphy_iovar_papd_debug(phy_info_t *pi, void *a);
extern void wlc_sslpnphy_iovar_txpwrctrl(phy_info_t *pi, int32 int_val);
extern void wlc_phy_detach_sslpnphy(phy_info_t *pi);

#define NPHY_MAX_HPVGA1_INDEX		10
#define NPHY_DEF_HPVGA1_INDEXLIMIT	7

typedef struct _phy_iq_est {
	int32  iq_prod;
	uint32 i_pwr;
	uint32 q_pwr;
} phy_iq_est_t;

typedef struct _phy_iq_comp {
	int16  a;
	int16  b;
} phy_iq_comp_t;

#define CHANNEL_ISRADAR(channel)  ((((channel) >= 52) && ((channel) <= 64)) || \
				   (((channel) >= 100) && ((channel) <= 140)))

extern void wlc_phy_stay_in_carriersearch_nphy(phy_info_t *pi, bool enable);
extern void wlc_nphy_deaf_mode(phy_info_t *pi, bool mode);

#define wlc_phy_write_table_nphy(pi, pti)	wlc_phy_write_table(pi, pti, NPHY_TableAddress, \
	NPHY_TableDataHi, NPHY_TableDataLo)
#define wlc_phy_read_table_nphy(pi, pti)	wlc_phy_read_table(pi, pti, NPHY_TableAddress, \
	NPHY_TableDataHi, NPHY_TableDataLo)
#define wlc_nphy_table_addr(pi, id, off)	wlc_phy_table_addr((pi), (id), (off), \
	NPHY_TableAddress, NPHY_TableDataHi, NPHY_TableDataLo)
#define wlc_nphy_table_data_write(pi, w, v)	wlc_phy_table_data_write((pi), (w), (v))

extern void wlc_phy_table_read_nphy(phy_info_t *pi, uint32, uint32 l, uint32 o, uint32 w, void *d);
extern void wlc_phy_table_write_nphy(phy_info_t *pi, uint32, uint32, uint32, uint32, const void *);

/* please use this macro extensively for IPA feature to enable compile to optimize codesize */
#define	PHY_IPA(pi) \
	((pi->ipa2g_on && CHSPEC_IS2G(pi->radio_chanspec)) || \
	 (pi->ipa5g_on && CHSPEC_IS5G(pi->radio_chanspec)))


#define WLC_PHY_WAR_PR51571(pi) \
	if ((BUSTYPE((pi)->sh->bustype) == PCI_BUS) && NREV_LT((pi)->pubpi.phy_rev, 3)) \
		(void)R_REG((pi)->sh->osh, &(pi)->regs->maccontrol)

extern void wlc_phy_cal_perical_nphy_run(phy_info_t *pi, uint8 caltype);
extern void wlc_phy_aci_reset_nphy(phy_info_t *pi);
extern void wlc_phy_pa_override_nphy(phy_info_t *pi, bool en);

extern uint8 wlc_phy_get_chan_freq_range_nphy(phy_info_t *pi, uint chan);

extern void wlc_phy_switch_radio_nphy(phy_info_t *pi, bool on);

extern void wlc_phy_stf_chain_upd_nphy(phy_info_t *pi);

extern void wlc_phy_force_rfseq_nphy(phy_info_t *pi, uint8 cmd);
extern int16 wlc_phy_tempsense_nphy(phy_info_t *pi);

extern uint16 wlc_phy_classifier_nphy(phy_info_t *pi, uint16 mask, uint16 val);

extern void wlc_phy_rx_iq_est_nphy(phy_info_t *pi, phy_iq_est_t *est, uint16 num_samps,
	uint8 wait_time, uint8 wait_for_crs);

extern void wlc_phy_rx_iq_coeffs_nphy(phy_info_t *pi, uint8 write, nphy_iq_comp_t *comp);
extern void wlc_phy_aci_and_noise_reduction_nphy(phy_info_t *pi);

extern void wlc_phy_rxcore_setstate_nphy(wlc_phy_t *pih, uint8 rxcore_bitmask);
extern uint8 wlc_phy_rxcore_getstate_nphy(wlc_phy_t *pih);

extern void wlc_phy_txpwrctrl_enable_nphy(phy_info_t *pi, uint8 ctrl_type);
extern void wlc_phy_txpwr_fixpower_nphy(phy_info_t *pi);
extern void wlc_phy_txpwr_apply_nphy(phy_info_t *pi);
extern void wlc_phy_txpwr_papd_cal_nphy(phy_info_t *pi);
extern uint16 wlc_phy_txpwr_idx_get_nphy(phy_info_t *pi);

extern nphy_txgains_t wlc_phy_get_tx_gain_nphy(phy_info_t *pi);
extern int  wlc_phy_cal_txiqlo_nphy(phy_info_t *pi, nphy_txgains_t target_gain, bool full, bool m);
extern int  wlc_phy_cal_rxiq_nphy(phy_info_t *pi, nphy_txgains_t target_gain, uint8 type, bool d);
extern void wlc_phy_txpwr_index_nphy(phy_info_t *pi, uint8 core_mask, int8 txpwrindex, bool res);
extern void wlc_phy_rssisel_nphy(phy_info_t *pi, uint8 core, uint8 rssi_type);
extern int  wlc_phy_poll_rssi_nphy(phy_info_t *pi, uint8 rssi_type, int32 *rssi_buf, uint8 nsamps);
extern void wlc_phy_rssi_cal_nphy(phy_info_t *pi);
extern int  wlc_phy_aci_scan_nphy(phy_info_t *pi);
extern nphy_txgains_t wlc_phy_cal_txgainctrl_inttssi_nphy(phy_info_t *pi, int8 target_tssi,
                                                          int8 init_gc_idx);
extern void wlc_phy_cal_txgainctrl_nphy(phy_info_t *pi, int32 dBm_targetpower, bool debug);
extern int
wlc_phy_tx_tone_nphy(phy_info_t *pi, uint32 f_kHz, uint16 max_val, uint8 mode, uint8, bool);
extern void wlc_phy_papd_enable_nphy(phy_info_t *pi, bool papd_state);
extern void wlc_phy_stopplayback_nphy(phy_info_t *pi);
extern void wlc_phy_est_tonepwr_nphy(phy_info_t *pi, int32 *qdBm_pwrbuf, uint8 num_samps);
extern void wlc_phy_radio205x_vcocal_nphy(phy_info_t *pi);
extern void wlc_phy_radio205x_check_vco_cal_nphy(phy_info_t *pi);

extern int wlc_phy_rssi_compute_nphy(phy_info_t *pi, wlc_d11rxhdr_t *wlc_rxh);

#ifdef SAMPLE_COLLECT
extern int wlc_phy_sample_collect_nphy(phy_info_t *pi, wl_samplecollect_args_t *p, uint32 *b);
extern int wlc_phy_sample_data_nphy(phy_info_t *pi, wl_sampledata_t *p, void *b);

extern void wlc_phy_sample_collect_start_nphy(phy_info_t *pi, uint8 coll_us,
	uint16 *crsctl, uint16 *crsctlu, uint16 *crsctll);

extern void wlc_phy_sample_collect_end_nphy(phy_info_t *pi,
	uint16 crsctl, uint16 crsctlu, uint16 crsctll);
#endif


#define NPHY_TESTPATTERN_BPHY_EVM   0
#define NPHY_TESTPATTERN_BPHY_RFCS  1

#define HTPHY_TESTPATTERN_BPHY_EVM   0
#define HTPHY_TESTPATTERN_BPHY_RFCS  1




#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
extern int wlc_phy_cal_cache_restore(phy_info_t *pi);
extern int wlc_phy_cal_cache_restore_nphy(phy_info_t *pih);
#endif

#if defined(AP) && defined(RADAR)
extern void wlc_phy_radar_detect_init(phy_info_t *pi, bool on);
extern void wlc_phy_update_radar_detect_param_nphy(phy_info_t *pi);
#endif /* defined(AP) && defined(RADAR) */

/* %%%%%% HTCONF function */
extern void wlc_phy_cals_htphy(phy_info_t *pi, uint8 caltype);
extern void wlc_phy_scanroam_cache_cal_htphy(phy_info_t *pi, bool set);


extern void wlc_phy_stay_in_carriersearch_htphy(phy_info_t *pi, bool enable);
extern void wlc_phy_deaf_htphy(phy_info_t *pi, bool mode);

#define wlc_phy_write_table_htphy(pi, pti)	wlc_phy_write_table(pi, pti, HTPHY_TableAddress, \
	HTPHY_TableDataHi, HTPHY_TableDataLo)
#define wlc_phy_read_table_htphy(pi, pti)	wlc_phy_read_table(pi, pti, HTPHY_TableAddress, \
	HTPHY_TableDataHi, HTPHY_TableDataLo)
extern void wlc_phy_table_read_htphy(phy_info_t *pi, uint32, uint32 l, uint32 o, uint32 w, void *d);
extern void wlc_phy_table_write_htphy(phy_info_t *pi, uint32, uint32, uint32, uint32, const void *);

extern void wlc_phy_pa_override_htphy(phy_info_t *pi, bool en);
extern void wlc_phy_anacore_htphy(phy_info_t *pi,  bool on);
extern void wlc_phy_rxcore_setstate_htphy(wlc_phy_t *pih, uint8 rxcore_bitmask);
extern uint8 wlc_phy_rxcore_getstate_htphy(wlc_phy_t *pih);

extern uint8 wlc_phy_get_chan_freq_range_htphy(phy_info_t *pi, uint chan);
extern void wlc_phy_switch_radio_htphy(phy_info_t *pi, bool on);

extern void wlc_phy_force_rfseq_htphy(phy_info_t *pi, uint8 cmd);
extern int16 wlc_phy_tempsense_htphy(phy_info_t *pi);

extern uint16 wlc_phy_classifier_htphy(phy_info_t *pi, uint16 mask, uint16 val);
extern void wlc_phy_rx_iq_est_htphy(phy_info_t *pi, phy_iq_est_t *est, uint16 num_samps,
                                    uint8 wait_time, uint8 wait_for_crs);

extern void wlc_phy_txpwr_apply_htphy(phy_info_t *pi);
void wlc_phy_txpwr_est_pwr_htphy(phy_info_t *pi, uint8 *Pout, uint8 *Pout_act);
extern uint32 wlc_phy_txpwr_idx_get_htphy(phy_info_t *pi);
extern void wlc_phy_txpwrctrl_enable_htphy(phy_info_t *pi, uint8 ctrl_type);
extern void wlc_phy_txpwr_by_index_htphy(phy_info_t *pi, uint8 core_mask, int8 txpwrindex);

extern int
wlc_phy_tx_tone_htphy(phy_info_t *pi, uint32 f_kHz, uint16 max_val, uint8 mode, uint8, bool);
extern void wlc_phy_stopplayback_htphy(phy_info_t *pi);

extern int
wlc_phy_tx_tone_htphy(phy_info_t *pi, uint32 f_kHz, uint16 max_val, uint8 mode, uint8, bool);
extern void wlc_phy_stopplayback_htphy(phy_info_t *pi);


extern int wlc_phy_rssi_compute_htphy(phy_info_t *pi, wlc_d11rxhdr_t *wlc_rxh);
#ifdef SAMPLE_COLLECT
extern int wlc_phy_sample_collect_htphy(phy_info_t *pi, wl_samplecollect_args_t *p, uint32 *b);
extern int wlc_phy_sample_data_htphy(phy_info_t *pi, wl_sampledata_t *p, void *b);

#endif /* SAMPLE_COLLECT */


#define wlc_phy_bphy_testpattern_htphy(a, b, c, d) do {} while (0)



#if defined(AP) && defined(RADAR)
extern void wlc_phy_radar_detect_init_htphy(phy_info_t *pi, bool on);
extern void wlc_phy_update_radar_detect_param_htphy(phy_info_t *pi);
extern int  wlc_phy_radar_detect_run_htphy(phy_info_t *pi);
#endif /* defined(AP) && defined(RADAR) */
extern void wlc_phy_update_rxldpc_htphy(phy_info_t *pi, bool ldpc);
extern void wlc_phy_nphy_tkip_rifs_war(phy_info_t *pi, uint8 rifs);

#ifndef BCMNODOWN
#endif	/* BCMNODOWN */


/* LPCONF || SSLPNCONF */
extern void wlc_phy_radio_2063_vco_cal(phy_info_t *pi);

/* SSLPNCONF */

extern void wlc_sslpnphy_papd_recal(phy_info_t *pi);
extern void wlc_sslpnphy_tx_pwr_ctrl_init(phy_info_t *pi);

#ifdef PHYMON
extern int wlc_phycal_state_nphy(phy_info_t *pi, void* buff, int len);
#endif /* PHYMON */

void wlc_phy_get_pwrdet_offsets(phy_info_t *pi, int8 *cckoffset, int8 *ofdmoffset);
extern int8 wlc_phy_upd_rssi_offset(phy_info_t *pi, int8 rssi, chanspec_t chanspec);

extern bool wlc_phy_n_txpower_ipa_ison(phy_info_t *pih);

#if defined(WLMCHAN) || defined(PHYCAL_CACHING)
/* Get the calcache entry given the chanspec */
extern ch_calcache_t *wlc_phy_get_chanctx(phy_info_t *phi, chanspec_t chanspec);
extern void wlc_phydump_cal_cache_nphy(phy_info_t *pih, ch_calcache_t *ctx, struct bcmstrbuf *b);
extern ch_calcache_t *wlc_phy_get_chanctx_oldest(phy_info_t *phi);
extern int wlc_phy_create_chanctx(wlc_phy_t *ppi, chanspec_t chanspec);
extern int wlc_phy_reinit_chanctx(phy_info_t *pi, ch_calcache_t *ctx, chanspec_t chanspec);
#endif
#endif /* TBD_YE */
#endif	/* _wlc_phy_int_h_ */
