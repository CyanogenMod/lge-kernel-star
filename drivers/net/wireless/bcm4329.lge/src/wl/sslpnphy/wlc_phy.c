/*
 * PHY and RADIO specific portion of Broadcom BCM43XX 802.11abgn
 * Networking Device Driver.
 *
 * Copyright (C) 2010, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id: wlc_phy.c,v 1.89 2010/09/01 21:12:32 Exp $
 */


#include <wlc_cfg.h>
#include <typedefs.h>
#include <qmath.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <bcmendian.h>
#include <wlioctl.h>
#if PHYHAL
#include <wlc_phy_radio.h>
#else
#include <bcm20xx.h>
#endif

#include <bitfuncs.h>
#include <bcmdevs.h>
#include <proto/802.11.h>
#include <sbhndpio.h>
#include <sbhnddma.h>
#include <hnddma.h>
#include <hndpmu.h>
#include <d11.h>
#include <wlc_rate.h>
#include <wlc_key.h>
#include <wlc_channel.h>
#include <wlc_pub.h>
#include <wlc_bsscfg.h>
#include <wl_dbg.h>
#include <wl_export.h>
#include <wlc_phy_int.h>
#if !PHYHAL
#include <wlc.h>
#include <wlc_phy.h>
#include <mimophytbls.h>
#include <lpphyregs.h>
#include <lpphytbls.h>
#include <sslpnphytbls.h>
#include <sslpnphyregs.h>
#else
#include <wlc_phytbl_ssn.h>
#include <wlc_phyreg_ssn.h>
#include <wlc_phy_ssn.h>
#endif /* PHYHAL */
#include <bcmnvram.h>

#ifdef WLMINIOCTL
#include <wl_minioctl.h>
#endif


#include <sbsprom.h>
#include <sbchipc.h>

#ifdef PHYHAL
#define BOARDFLAGS(flag) (flag)
#define GENERIC_PHY_INFO(pi) ((pi)->sh)
#define WL_SUSPEND_MAC_AND_WAIT(pi) wlapi_suspend_mac_and_wait(pi->sh->physhim)
#define WL_ENABLE_MAC(pi) wlapi_enable_mac(pi->sh->physhim)
#define WL_WRITE_SHM(pi, addr, val)  wlapi_bmac_write_shm(pi->sh->physhim, addr, val)
#define WL_READ_SHM(pi, addr)  wlapi_bmac_read_shm(pi->sh->physhim, addr)
#define WL_MCTRL(pi, addr, val) wlapi_bmac_mctrl(pi->sh->physhim, addr, val)
#define WL_PHYCAL PHY_TRACE
#define SCAN_IN_PROGRESS(pi) SCAN_INPROG_PHY(pi)
#define WLC_RM_IN_PROGRESS(pi) RM_INPROG_PHY(pi)
#define BCMECICOEX_ENAB(pi) BCMECICOEX_ENAB_PHY(pi)
#define PHY_GETINTVAR_ARRAY(name, type, idx)  getintvararray(pi->vars, name , idx)

#define wlc_pi (pi)
#define NON_BT_CHIP(wlc) 0

#ifdef XTAL_FREQ
#define XTALFREQ(_freq)  XTAL_FREQ
#else
#define XTALFREQ(_freq)  _freq
#endif

#ifdef WLSINGLE_ANT
#define ANT_AVAIL(_ant) 1
#else
#define ANT_AVAIL(_ant) (_ant)
#endif
#ifdef OLYMPIC
#define IS_OLYMPIC(_pi) TRUE
#else 
#define IS_OLYMPIC(_pi) (_pi->u.pi_sslpnphy->sslpnphy_OLYMPIC)
#endif
#define si_pmu_res_4319_swctrl_war(sih, osh, on) 
#define si_otp_fabid(sih, var, on) BCME_OK
#else  /* PHYHAL */
#define sslpnphy_specific (pi)
#define GENERIC_PHY_INFO(pi) ((pi)->pub)
#define WL_SUSPEND_MAC_AND_WAIT(pi) wlc_suspend_mac_and_wait(pi->wlc)
#define WL_ENABLE_MAC(pi) wlc_enable_mac(pi->wlc)
#define WL_WRITE_SHM(pi, addr, val)  wlc_write_shm(pi->wlc, addr, val)
#define WL_READ_SHM(pi, addr)  wlc_read_shm(pi->wlc, addr)
#define WL_MCTRL(pi, addr, val) wlc_mctrl(pi->wlc, addr, val)
#define PHY_GETVAR(pi, name) getvar(pi->pub->vars, name)
#define PHY_GETINTVAR(pi, name) getintvar(pi->pub->vars, name)
#define PHY_GETINTVAR_ARRAY(name, type, idx) read_nvram_array(name, type, idx)

#define SCAN_IN_PROGRESS(wlc)	(wlc_scan_inprog(wlc))
#define WLC_RM_IN_PROGRESS(wlc)	(wlc_rminprog(wlc))
#endif /* PHYHAL */

#ifdef DONGLEOVERLAYS
#include <wlc_event.h>
#define PHYINIT_STATE_GBAND	0
#define PHYINIT_STATE_ABAND	1
#define PHYINIT_STATE_DONE	2
#endif




/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  inter-module connection					*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */


#undef IS20MHZ
#undef IS40MHZ
#ifdef WL20MHZ_ONLY
#define IS20MHZ(pi) (TRUE)
#define IS40MHZ(pi) (FALSE)
#else
#define IS20MHZ(pi)	((pi)->bw == WLC_20_MHZ)
#define IS40MHZ(pi)	((pi)->bw == WLC_40_MHZ)
#endif

#define wlc_radio_2063_rc_cal_done(pi) (0 != (read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_6) & 0x02))

/* Helper callbacks to get wlc state */


#ifdef WLPLT
#define PLT_IN_PROGRESS(wlc) (wlc_pltinprog(wlc))
#else
#define PLT_IN_PROGRESS(wlc) (FALSE)
#endif
#ifdef STA
#define ASSOC_IN_PROGRESS(wlc)	(wlc_associnprog(wlc))
#else
#define ASSOC_IN_PROGRESS(wlc)	(FALSE)
#endif /* STA */



/* %%%%%%%%%%%%%%%%%%%% */
/*  common function	*/
/* %%%%%%%%%%%%%%%%%%%% */

extern uint wlc_phy_channel2idx(uint channel);
#ifdef BAND5G
extern int wlc_get_band_range(phy_info_t*, chanspec_t);
#else
#define wlc_get_band_range(_pi, _ch) WL_CHAN_FREQ_RANGE_2G
#endif

extern void wlc_phy_do_dummy_tx(phy_info_t *pi, bool ofdm, bool pa_on);
void wlc_sslpnphy_periodic_cal_top(phy_info_t *pi); /* ROMTERM */
void wlc_sslpnphy_auxadc_measure(wlc_phy_t *ppi, bool readVal);	/* ROMTERM */
int8 wlc_sslpnphy_get_rx_pwr_offset(phy_info_t *pi);	/* ROMTERM */
void wlc_sslpnphy_rx_offset_init(phy_info_t *pi); /* ROMTERM only */
void wlc_sslpnphy_cck_filt_load(phy_info_t *pi, uint8 filtsel);	/* ROMTERM only */
void wlc_sslpnphy_channel_gain_adjust(phy_info_t *pi);	/* ROMTERM only */
void wlc_sslpnphy_CmRxAciGainTbl_Tweaks(void *args);	/* ROMTERM only */
void wlc_sslpnphy_clear_tx_power_offsets(phy_info_t *pi);	/* ROMTERM only */
bool wlc_sslpnphy_btcx_override_enable(phy_info_t *pi);	/* ROMTERM only */
void wlc_sslpnphy_recalc_tssi2dbm_tbl(phy_info_t *pi, int32 a1, int32 b0, int32 b1); /* ROMTERM only */
void wlc_sslpnphy_set_tx_locc(phy_info_t *pi, uint16 didq); /* ROMTERM only */
void wlc_sslpnphy_txpwrtbl_iqlo_cal(phy_info_t *pi); /* ROMTERM only */
int32 wlc_sslpnphy_vbatsense(phy_info_t *pi); /* ROMTERM only */
void wlc_sslpnphy_save_papd_calibration_results(phy_info_t *pi); /* ROMTERM only */
int wlc_sslpnphy_tempsense(phy_info_t *pi); /* ROMTERM only */
void wlc_sslpnphy_temp_adj(phy_info_t *pi); /* ROMTERM only */
void wlc_sslpnphy_set_chanspec_tweaks(phy_info_t *pi, chanspec_t chanspec); /* ROMTERM only */
int wlc_get_ssn_lp_band_range(uint); /* ROMTERM only */
void wlc_2063_vco_cal(phy_info_t *pi); /* ROMTERM only */
void wlc_sslpnphy_setchan_cal(phy_info_t *pi, int32 int_val); /* ROMTERM only */
/* %%%%%%%%%%%%%%%%%%%% */
/*  debugging		*/
/* %%%%%%%%%%%%%%%%%%%% */


/* %%%%%%%%%%%%%%%%%%%% */
/*  radio control	*/
/* %%%%%%%%%%%%%%%%%%%% */

static void wlc_radio_2063_init_sslpnphy(phy_info_t *pi);

static void wlc_sslpnphy_radio_2063_channel_tune(phy_info_t *pi, uint8 channel);


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  macro							*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/* TX Power indexes */
#define TXP_FIRST_CCK		0	/* Index for first CCK rate */
#define TXP_LAST_CCK		3	/* Index for last CCK rate */
#define TXP_FIRST_OFDM		4	/* Index for first OFDM rate */
#define TXP_LAST_OFDM		11	/* Index for last OFDM rate */
#define TXP_FIRST_MCS_20	12	/* Index for first MCS at 20 MHz */
#define TXP_LAST_MCS_SISO_20	19	/* Index for last SISO MCS at 20 MHz */
#define TXP_FIRST_MCS_SISO_20_CDD 20 /* Index for first MCS_CDD at 20 MHz */
#define TXP_LAST_MCS_20		27	/* Index for last MCS at 20 MHz */
#define TXP_FIRST_MCS_40	28	/* Index for first MCS at 40 MHz */
#define TXP_LAST_MCS_SISO_40	35	/* Index for last SISO MCS at 40 MHz */
#define TXP_FIRST_MCS_SISO_40_CDD  36 /* Index for first MCS_CDD at 40 MHz */
#define TXP_LAST_MCS_40		44	/* Index for last MCS at 40 MHz */

#define LPPHY_IQLOCC_READ(val) ((uint8)(-(int8)(((val) & 0xf0) >> 4) + (int8)((val) & 0x0f)))
#define PLL_2063_LOW_END_VCO 	3000
#define PLL_2063_LOW_END_KVCO 	27
#define PLL_2063_HIGH_END_VCO	4200
#define PLL_2063_HIGH_END_KVCO	68
#define PLL_2063_LOOP_BW			300
#define PLL_2063_LOOP_BW_ePA			500
#define PLL_2063_D30				3000
#define PLL_2063_D30_ePA			1500
#define PLL_2063_CAL_REF_TO		8
#define PLL_2063_MHZ				1000000
#define PLL_2063_OPEN_LOOP_DELAY	5

/* %%%%%%%%%%%%%%% */
/* SSLPNPHY macros */
/* %%%%%%%%%%%%%%% */

#define VBAT_RIPPLE_CHECK(_pi) 0


#define SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT \
	(SSLPNPHY_txgainctrlovrval1_txgainctrl_ovr_val1_SHIFT + 8)
#define SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_MASK \
	(0x7f << SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT)

#define SSLPNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_SHIFT \
	(SSLPNPHY_stxtxgainctrlovrval1_stxtxgainctrl_ovr_val1_SHIFT + 8)
#define SSLPNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_MASK \
	(0x7f << SSLPNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_SHIFT)

#define wlc_sslpnphy_enable_tx_gain_override(pi) \
	wlc_sslpnphy_set_tx_gain_override(pi, TRUE)
#define wlc_sslpnphy_disable_tx_gain_override(pi) \
	wlc_sslpnphy_set_tx_gain_override(pi, FALSE)

#define wlc_sslpnphy_set_start_tx_pwr_idx(pi, idx) \
	mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlCmd, \
		SSLPNPHY_TxPwrCtrlCmd_pwrIndex_init_MASK, \
		(uint16)(idx) << SSLPNPHY_TxPwrCtrlCmd_pwrIndex_init_SHIFT)

#define wlc_sslpnphy_set_tx_pwr_npt(pi, npt) \
	mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlNnum, \
		SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_MASK, \
		(uint16)(npt) << SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_SHIFT)

#define wlc_sslpnphy_get_tx_pwr_ctrl(pi) \
	(read_phy_reg((pi), SSLPNPHY_TxPwrCtrlCmd) & \
			(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
			SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK | \
			SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK))


#define wlc_sslpnphy_get_tx_pwr_npt(pi) \
	((read_phy_reg(pi, SSLPNPHY_TxPwrCtrlNnum) & \
		SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_MASK) >> \
		SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_SHIFT)

#define wlc_sslpnphy_get_current_tx_pwr_idx(pi) \
	((read_phy_reg(pi, SSLPNPHY_TxPwrCtrlStatus) & \
		SSLPNPHY_TxPwrCtrlStatus_baseIndex_MASK) >> \
		SSLPNPHY_TxPwrCtrlStatus_baseIndex_SHIFT)

#define wlc_sslpnphy_get_target_tx_pwr(pi) \
	((read_phy_reg(pi, SSLPNPHY_TxPwrCtrlTargetPwr) & \
		SSLPNPHY_TxPwrCtrlTargetPwr_targetPwr0_MASK) >> \
		SSLPNPHY_TxPwrCtrlTargetPwr_targetPwr0_SHIFT)

#ifdef PHYHAL
#define wlc_sslpnphy_validated_tssi_pwr(pi, pwr) \
	MIN((pi->u.pi_sslpnphy)->sslpnphy_tssi_max_pwr_limit, MAX((pi->u.pi_sslpnphy)->sslpnphy_tssi_min_pwr_limit, (pwr)))
#else
#define  wlc_sslpnphy_validated_tssi_pwr(pi, pwr) \
	MIN((pi)->sslpnphy_tssi_max_pwr_limit, MAX((pi)->sslpnphy_tssi_min_pwr_limit, (pwr)))
#endif /* PHYHAL */

#define wlc_sslpnphy_force_target_tx_pwr(pi, target) \
	mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlTargetPwr, \
		SSLPNPHY_TxPwrCtrlTargetPwr_targetPwr0_MASK, \
		(uint16)(target) << SSLPNPHY_TxPwrCtrlTargetPwr_targetPwr0_SHIFT)

#define wlc_sslpnphy_set_target_tx_pwr(pi, target) \
	wlc_sslpnphy_force_target_tx_pwr(pi, wlc_sslpnphy_validated_tssi_pwr(pi, target))

/* Turn off all the crs signals to the MAC */
#define wlc_sslpnphy_set_deaf(pi)	wlc_sslpnphy_deaf_mode(pi, TRUE)

/* Restore all the crs signals to the MAC */
#define wlc_sslpnphy_clear_deaf(pi)	 wlc_sslpnphy_deaf_mode(pi, FALSE)

#define wlc_sslpnphy_iqcal_active(pi)	\
	(read_phy_reg((pi), SSLPNPHY_iqloCalCmd) & \
	(SSLPNPHY_iqloCalCmd_iqloCalCmd_MASK | SSLPNPHY_iqloCalCmd_iqloCalDFTCmd_MASK))

#define wlc_sslpnphy_tssi_enabled(pi) \
	(SSLPNPHY_TX_PWR_CTRL_OFF != wlc_sslpnphy_get_tx_pwr_ctrl((pi)))

#define SWCTRL_BT_TX		0x18
#define SWCTRL_OVR_DISABLE	0x40

#define	AFE_CLK_INIT_MODE_TXRX2X	1
#define	AFE_CLK_INIT_MODE_PAPD		0

#define SSLPNPHY_TX_PWR_CTRL_OFF	0
#define SSLPNPHY_TX_PWR_CTRL_SW SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK
#define SSLPNPHY_TX_PWR_CTRL_HW \
	(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
	SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK | \
	SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK)

#define SSLPNPHY_TBL_ID_IQLOCAL			0x00
#define SSLPNPHY_TBL_ID_TXPWRCTL 		0x07
#define SSLPNPHY_TBL_ID_GAIN_IDX		0x0d
#define SSLPNPHY_TBL_ID_GAIN_TBL		0x12
#define SSLPNPHY_TBL_ID_GAINVALTBL_IDX		0x11
#define SSLPNPHY_TBL_ID_SW_CTRL			0x0f
#define SSLPNPHY_TBL_ID_SPUR			0x14
#define SSLPNPHY_TBL_ID_SAMPLEPLAY		0x15
#define SSLPNPHY_TBL_ID_SAMPLEPLAY1		0x16
#define SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL	0x18

#define SSLPNPHY_TX_PWR_CTRL_RATE_OFFSET 	64
#define SSLPNPHY_TX_PWR_CTRL_MAC_OFFSET 	128
#define SSLPNPHY_TX_PWR_CTRL_GAIN_OFFSET 	192
#define SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET		320
#define SSLPNPHY_TX_PWR_CTRL_LO_OFFSET		448
#define SSLPNPHY_TX_PWR_CTRL_PWR_OFFSET		576

#define SSLPNPHY_TX_PWR_CTRL_START_INDEX_2G	60
#define SSLPNPHY_TX_PWR_CTRL_START_INDEX_5G	70
#define SSLPNPHY_TX_PWR_CTRL_START_INDEX_2G_PAPD	100
#define SSLPNPHY_TX_PWR_CTRL_START_INDEX_5G_PAPD	70

#define SSLPNPHY_TX_PWR_CTRL_START_NPT		1
#define SSLPNPHY_TX_PWR_CTRL_MAX_NPT		1

#define SSLPNPHY_NUM_DIG_FILT_COEFFS 9

#define SSLPNPHY_TX_POWER_TABLE_SIZE	128
#define SSLPNPHY_MAX_TX_POWER_INDEX	(SSLPNPHY_TX_POWER_TABLE_SIZE - 1)

#define SSLPNPHY_NOISE_SAMPLES_DEFAULT 5000

#define SSLPNPHY_ACI_DETECT_START      1
#define SSLPNPHY_ACI_DETECT_PROGRESS   2
#define SSLPNPHY_ACI_DETECT_STOP       3

#define SSLPNPHY_ACI_CRSHIFRMLO_TRSH 100
#define SSLPNPHY_ACI_GLITCH_TRSH 2000
#define	SSLPNPHY_ACI_TMOUT 250		/* Time for CRS HI and FRM LO (in micro seconds) */
#define SSLPNPHY_ACI_DETECT_TIMEOUT  2	/* in  seconds */
#define SSLPNPHY_ACI_START_DELAY 0

#define SSLPNPHY_NOISE_PWR_FIFO_DEPTH 6
#define SSLPNPHY_INIT_NOISE_CAL_TMOUT 38000 /* In uS */
#define SSLPNPHY_NPWR_MINLMT 1
#define SSLPNPHY_NPWR_MAXLMT_2G 50
#define SSLPNPHY_NPWR_MAXLMT_5G 200
#define SSLPNPHY_NPWR_LGC_MINLMT_20MHZ 7
#define SSLPNPHY_NPWR_LGC_MAXLMT_20MHZ 21
#define SSLPNPHY_NPWR_LGC_MINLMT_40MHZ 4
#define SSLPNPHY_NPWR_LGC_MAXLMT_40MHZ 14
#define SSLPNPHY_NOISE_MEASURE_WINDOW_2G 1800 /* In uS */
#define SSLPNPHY_NOISE_MEASURE_WINDOW_5G 1400 /* In uS */
#define SSLPNPHY_MAX_GAIN_CHANGE_LMT_2G 9
#define SSLPNPHY_MAX_GAIN_CHANGE_LMT_5G 15
#define SSLPNPHY_MAX_RXPO_CHANGE_LMT_2G 12
#define SSLPNPHY_MAX_RXPO_CHANGE_LMT_5G 18

#define wlc_sslpnphy_tx_gain_override_enabled(pi) \
	(0 != (read_phy_reg((pi), SSLPNPHY_AfeCtrlOvr) & SSLPNPHY_AfeCtrlOvr_dacattctrl_ovr_MASK))

#define wlc_sslpnphy_total_tx_frames(pi) \
	WL_READ_SHM(pi, M_UCODE_MACSTAT + OFFSETOF(macstat_t, txallfrm))

typedef struct {
	uint16 gm_gain;
	uint16 pga_gain;
	uint16 pad_gain;
	uint16 dac_gain;
} sslpnphy_txgains_t;

typedef struct {
	sslpnphy_txgains_t gains;
	bool useindex;
	int8 index;
} sslpnphy_txcalgains_t;

typedef struct {
	uint8 chan;
	int16 a;
	int16 b;
} sslpnphy_rx_iqcomp_t;

typedef struct {
	uint32 iq_prod;
	uint32 i_pwr;
	uint32 q_pwr;
} sslpnphy_iq_est_t;

typedef enum {
	SSLPNPHY_CAL_FULL,
	SSLPNPHY_CAL_RECAL,
	SSLPNPHY_CAL_CURRECAL,
	SSLPNPHY_CAL_DIGCAL,
	SSLPNPHY_CAL_GCTRL
} sslpnphy_cal_mode_t;

typedef enum {
	SSLPNPHY_PAPD_CAL_CW,
	SSLPNPHY_PAPD_CAL_OFDM
} sslpnphy_papd_cal_type_t;

/* SSLPNPHY IQCAL parameters for various Tx gain settings */
/* table format: */
/*	target, gm, pga, pad, ncorr for each of 5 cal types */
typedef uint16 iqcal_gain_params_sslpnphy[9];

STATIC const iqcal_gain_params_sslpnphy BCMOVERLAYDATA(1, tbl_iqcal_gainparams_sslpnphy_2G)[] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
	};

STATIC const iqcal_gain_params_sslpnphy BCMOVERLAYDATA(1, tbl_iqcal_gainparams_sslpnphy_5G)[] = {
	{0x7ef, 7, 0xe, 0xe, 0, 0, 0, 0, 0},
	};

STATIC const iqcal_gain_params_sslpnphy *BCMOVERLAYDATA(1, tbl_iqcal_gainparams_sslpnphy)[2] = {
	tbl_iqcal_gainparams_sslpnphy_2G,
	tbl_iqcal_gainparams_sslpnphy_5G
	};

STATIC const uint16 BCMOVERLAYDATA(1, iqcal_gainparams_numgains_sslpnphy)[2] = {
	sizeof(tbl_iqcal_gainparams_sslpnphy_2G) / sizeof(*tbl_iqcal_gainparams_sslpnphy_2G),
	sizeof(tbl_iqcal_gainparams_sslpnphy_5G) / sizeof(*tbl_iqcal_gainparams_sslpnphy_5G),
	};

/* LO Comp Gain ladder. Format: {m genv} */
STATIC CONST
uint16 sslpnphy_iqcal_loft_gainladder[]  = {
	((2 << 8) | 0),
	((3 << 8) | 0),
	((4 << 8) | 0),
	((6 << 8) | 0),
	((8 << 8) | 0),
	((11 << 8) | 0),
	((16 << 8) | 0),
	((16 << 8) | 1),
	((16 << 8) | 2),
	((16 << 8) | 3),
	((16 << 8) | 4),
	((16 << 8) | 5),
	((16 << 8) | 6),
	((16 << 8) | 7),
	((23 << 8) | 7),
	((32 << 8) | 7),
	((45 << 8) | 7),
	((64 << 8) | 7),
	((91 << 8) | 7),
	((128 << 8) | 7)
};

/* Image Rejection Gain ladder. Format: {m genv} */
STATIC CONST
uint16 sslpnphy_iqcal_ir_gainladder[] = {
	((1 << 8) | 0),
	((2 << 8) | 0),
	((4 << 8) | 0),
	((6 << 8) | 0),
	((8 << 8) | 0),
	((11 << 8) | 0),
	((16 << 8) | 0),
	((23 << 8) | 0),
	((32 << 8) | 0),
	((45 << 8) | 0),
	((64 << 8) | 0),
	((64 << 8) | 1),
	((64 << 8) | 2),
	((64 << 8) | 3),
	((64 << 8) | 4),
	((64 << 8) | 5),
	((64 << 8) | 6),
	((64 << 8) | 7),
	((91 << 8) | 7),
	((128 << 8) | 7)
};

/* Autogenerated by 2063_chantbl_tcl2c.tcl */
OSTATIC const
sslpnphy_rx_iqcomp_t BCMOVERLAYDATA(1, sslpnphy_rx_iqcomp_table_rev0)[] = {
	{ 1, 0, 0 },
	{ 2, 0, 0 },
	{ 3, 0, 0 },
	{ 4, 0, 0 },
	{ 5, 0, 0 },
	{ 6, 0, 0 },
	{ 7, 0, 0 },
	{ 8, 0, 0 },
	{ 9, 0, 0 },
	{ 10, 0, 0 },
	{ 11, 0, 0 },
	{ 12, 0, 0 },
	{ 13, 0, 0 },
	{ 14, 0, 0 },
	{ 34, 0, 0 },
	{ 38, 0, 0 },
	{ 42, 0, 0 },
	{ 46, 0, 0 },
	{ 36, 0, 0 },
	{ 40, 0, 0 },
	{ 44, 0, 0 },
	{ 48, 0, 0 },
	{ 52, 0, 0 },
	{ 56, 0, 0 },
	{ 60, 0, 0 },
	{ 64, 0, 0 },
	{ 100, 0, 0 },
	{ 104, 0, 0 },
	{ 108, 0, 0 },
	{ 112, 0, 0 },
	{ 116, 0, 0 },
	{ 120, 0, 0 },
	{ 124, 0, 0 },
	{ 128, 0, 0 },
	{ 132, 0, 0 },
	{ 136, 0, 0 },
	{ 140, 0, 0 },
	{ 149, 0, 0 },
	{ 153, 0, 0 },
	{ 157, 0, 0 },
	{ 161, 0, 0 },
	{ 165, 0, 0 },
	{ 184, 0, 0 },
	{ 188, 0, 0 },
	{ 192, 0, 0 },
	{ 196, 0, 0 },
	{ 200, 0, 0 },
	{ 204, 0, 0 },
	{ 208, 0, 0 },
	{ 212, 0, 0 },
	{ 216, 0, 0 },
	};

uint32 sslpnphy_gaincode_table[] = {
	0x100800,
	0x100050,
	0x100150,
	0x100250,
	0x100950,
	0x100255,
	0x100955,
	0x100a55,
	0x110a55,
	0x1009f5,
	0x10095f,
	0x100a5f,
	0x110a5f,
	0x10305,
	0x10405,
	0x10b05,
	0x1305,
	0x35a,
	0xa5a,
	0xb5a,
	0x125a,
	0x135a,
	0x10b5f,
	0x135f,
	0x1135f,
	0x1145f,
	0x1155f,
	0x33af,
	0x132ff,
	0x232ff,
	0x215ff,
	0x216ff,
	0x235ff,
	0x236ff,
	0x255ff,
	0x256ff,
	0x2d5ff
};

uint8 sslpnphy_gain_table[] = {
	-14,
	-11,
	-8,
	-6,
	-2,
	0,
	4,
	6,
	9,
	12,
	16,
	18,
	21,
	25,
	27,
	31,
	34,
	37,
	39,
	43,
	45,
	49,
	52,
	55,
	58,
	60,
	63,
	65,
	68,
	71,
	74,
	77,
	80,
	83,
	86,
	89,
	92
};

int8 sslpnphy_gain_index_offset_for_rssi[] = {
	7,	/* 0 */
	7,	/* 1 */
	7,	/* 2 */
	7,	/* 3 */
	7,	/* 4 */
	7,	/* 5 */
	7,	/* 6 */
	8,	/* 7 */
	7,	/* 8 */
	7,	/* 9 */
	6,	/* 10 */
	7,	/* 11 */
	7,	/* 12 */
	4,	/* 13 */
	4,	/* 14 */
	4,	/* 15 */
	4,	/* 16 */
	4,	/* 17 */
	4,	/* 18 */
	4,	/* 19 */
	4,	/* 20 */
	3,	/* 21 */
	3,	/* 22 */
	3,	/* 23 */
	3,	/* 24 */
	3,	/* 25 */
	3,	/* 26 */
	4,	/* 27 */
	2,	/* 28 */
	2,	/* 29 */
	2,	/* 30 */
	2,	/* 31 */
	2,	/* 32 */
	2,	/* 33 */
	-1,	/* 34 */
	-2,	/* 35 */
	-2,	/* 36 */
	-2	/* 37 */
};

int8 sslpnphy_gain_index_offset_for_pkt_rssi[] = {
	8,	/* 0 */
	8,	/* 1 */
	8,	/* 2 */
	8,	/* 3 */
	8,	/* 4 */
	8,	/* 5 */
	8,	/* 6 */
	9,	/* 7 */
	10,	/* 8 */
	8,	/* 9 */
	8,	/* 10 */
	7,	/* 11 */
	7,	/* 12 */
	1,	/* 13 */
	2,	/* 14 */
	2,	/* 15 */
	2,	/* 16 */
	2,	/* 17 */
	2,	/* 18 */
	2,	/* 19 */
	2,	/* 20 */
	2,	/* 21 */
	2,	/* 22 */
	2,	/* 23 */
	2,	/* 24 */
	2,	/* 25 */
	2,	/* 26 */
	2,	/* 27 */
	2,	/* 28 */
	2,	/* 29 */
	2,	/* 30 */
	2,	/* 31 */
	1,	/* 32 */
	1,	/* 33 */
	0,	/* 34 */
	0,	/* 35 */
	0,	/* 36 */
	0	/* 37 */
};


extern CONST uint8 spur_tbl_rev0[];
extern CONST uint8 spur_tbl_rev2[];

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  typedef, enum, structure, global variable			*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* channel info type for 2063 radio */
typedef struct _chan_info_2063 {
	uint   chan;            /* channel number */
	uint   freq;            /* in Mhz */
	uint8 RF_logen_vcobuf_1;
	uint8 RF_logen_mixer_2;
	uint8 RF_logen_buf_2;
	uint8 RF_logen_rccr_1;
	uint8 RF_grx_1st_3;
	uint8 RF_grx_2nd_2;
	uint8 RF_arx_1st_3;
	uint8 RF_arx_2nd_1;
	uint8 RF_arx_2nd_4;
	uint8 RF_arx_2nd_7;
	uint8 RF_arx_ps_6;
	uint8 RF_txrf_ctrl_2;
	uint8 RF_txrf_ctrl_5;
	uint8 RF_pa_ctrl_11;
	uint8 RF_arx_mix_4;
	uint8 RF_wrf_slna_RX_2G_1st_VT_STG1;
	uint8 RF_txrf_sp_9;
	uint8 RF_txrf_sp_6;
} chan_info_2063_t;

/* channel info type for 2063 radio used in sslpnphy */
typedef struct _chan_info_2063_sslpnphy {
#ifdef BCM4329B1
	uint16 chan;            /* channel number */
	uint16 freq;            /* in Mhz */
#else
	uint chan;            /* channel number */
	uint freq;            /* in Mhz */
#endif
	uint8 RF_logen_vcobuf_1;
	uint8 RF_logen_mixer_2;
	uint8 RF_logen_buf_2;
	uint8 RF_logen_rccr_1;
	uint8 RF_grx_1st_3;
	uint8 RF_grx_2nd_2;
	uint8 RF_arx_1st_3;
	uint8 RF_arx_2nd_1;
	uint8 RF_arx_2nd_4;
	uint8 RF_arx_2nd_7;
	uint8 RF_arx_ps_6;
	uint8 dummy2;
	uint8 RF_txrf_ctrl_2;
	uint8 RF_txrf_ctrl_5;
	uint8 RF_pa_ctrl_11;
	uint8 RF_arx_mix_4;
	uint8 dummy4;
} chan_info_2063_sslpnphy_t;

#ifdef BAND5G
typedef struct _chan_info_2063_sslpnphy_ninja_aband_tweaks {
	uint freq;
	uint16 valid_tweak;
	uint8 RF_PA_CTRL_2;
	uint8 RF_PA_CTRL_5;
	uint8 RF_PA_CTRL_7;
	uint8 RF_PA_CTRL_11;
	uint8 RF_TXRF_CTRL_2;
} chan_info_2063_sslpnphy_ninja_aband_tweaks_t;
typedef struct _chan_info_2063_sslpnphy_X17_aband_tweaks {
	uint freq;
	uint16 valid_tweak;
	uint8 BCM_RF_PA_CTRL_5;
	uint8 BCM_RF_TXRF_CTRL_8;
	uint8 BCM_RF_TXRF_CTRL_5;
	uint8 BCM_RF_TXRF_CTRL_2;
	uint8 MRT_RF_PA_CTRL_5;
	uint8 MRT_RF_TXRF_CTRL_8;
	uint8 MRT_RF_TXRF_CTRL_5;
	uint8 MRT_RF_TXRF_CTRL_2;
	uint8 ePA_RF_PA_CTRL_11;
	uint8 ePA_RF_TXRF_CTRL_8;
	uint8 ePA_RF_TXRF_CTRL_5;
	uint8 ePA_RF_TXRF_CTRL_2;
} chan_info_2063_sslpnphy_X17_aband_tweaks_t;
typedef struct _chan_info_2063_sslpnphy_aband_tweaks {
	uint freq;
	uint16 valid_tweak;
	uint8 RF_PA_CTRL_11;
	uint8 RF_TXRF_CTRL_8;
	uint8 RF_TXRF_CTRL_5;
	uint8 RF_TXRF_CTRL_2;
	uint8 RF_TXRF_CTRL_4;
	uint8 RF_TXRF_CTRL_7;
	uint8 RF_TXRF_CTRL_6;
	uint8 RF_PA_CTRL_2;
	uint8 RF_PA_CTRL_5;
	uint8 RF_TXRF_CTRL_15;
	uint8 RF_TXRF_CTRL_14;
	uint8 RF_PA_CTRL_7;
	uint8 RF_PA_CTRL_15;
} chan_info_2063_sslpnphy_aband_tweaks_t;
typedef struct _chan_info_2063_sslpnphy_X17_epa_tweaks {
	uint freq;
	uint8 ePA_JTAG_PLL_CP_2;
	uint8 ePA_JTAG_PLL_CP_3;
} chan_info_2063_sslpnphy_X17_epa_tweaks_t;

STATIC chan_info_2063_sslpnphy_X17_epa_tweaks_t chan_info_2063_sslpnphy_X17_epa_tweaks[] = {
	{5180, 0x6d, 0x12},
	{5200, 0x6d, 0x12},
	{5220, 0x6d, 0x12},
	{5240, 0x6c, 0x11},
	{5260, 0x6c, 0x11},
	{5280, 0x6c, 0x11},
	{5300, 0x6b, 0x10},
	{5320, 0x6b, 0x10},
	{5500, 0x68, 0xf},
	{5520, 0x68, 0xf},
	{5540, 0x67, 0xe},
	{5560, 0x68, 0xf},
	{5580, 0x67, 0xe},
	{5600, 0x67, 0xe},
	{5620, 0x67, 0xe},
	{5640, 0x66, 0xe},
	{5660, 0x66, 0xe},
	{5680, 0x66, 0xe},
	{5700, 0x66, 0xe},
	{5745, 0x65, 0xd},
	{5765, 0x65, 0xd},
	{5785, 0x65, 0xd},
	{5805, 0x65, 0xd},
	{5825, 0x64, 0xd}
};


/* default */
STATIC chan_info_2063_sslpnphy_ninja_aband_tweaks_t chan_info_2063_sslpnphy_ninja_aband_tweaks[] = {
	{5180, 0x1f, 0x90, 0x90, 0x02, 0x50, 0xf8 },
	{5190, 0x1f, 0x90, 0x90, 0x02, 0x50, 0xf8 },
	{5200, 0x1f, 0x90, 0x80, 0x02, 0x40, 0xf8 },
	{5220, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5230, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5240, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5260, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5270, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5280, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5300, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5310, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5320, 0x1f, 0x90, 0x70, 0x02, 0x40, 0xf8 },
	{5500, 0x1f, 0x80, 0x70, 0x02, 0x10, 0x94 },
	{5510, 0x1f, 0x80, 0x70, 0x02, 0x10, 0x94 },
	{5520, 0x1f, 0x70, 0x70, 0x02, 0x70, 0x94 },
	{5540, 0x1f, 0x90, 0x60, 0x02, 0x10, 0x94 },
	{5550, 0x1f, 0x90, 0x60, 0x02, 0x10, 0x94 },
	{5560, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x84 },
	{5580, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x84 },
	{5590, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x84 },
	{5600, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x84 },
	{5620, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x84 },
	{5630, 0x1f, 0x90, 0x60, 0x02, 0x30, 0x74 },
	{5640, 0x1f, 0x90, 0x60, 0x02, 0x20, 0x74 },
	{5660, 0x1f, 0x90, 0x50, 0x02, 0x30, 0x74 },
	{5670, 0x1f, 0x90, 0x50, 0x02, 0x30, 0x74 },
	{5680, 0x1f, 0x90, 0x50, 0x02, 0x30, 0x74 },
	{5700, 0x1f, 0x90, 0x50, 0x02, 0x20, 0x74 },
	{5745, 0x1f, 0x90, 0x50, 0x02, 0x40, 0x64 },
	{5755, 0x1f, 0x90, 0x50, 0x02, 0x40, 0x64 },
	{5765, 0x1f, 0x90, 0x50, 0x02, 0x40, 0x64 },
	{5785, 0x1f, 0x90, 0x50, 0x02, 0x20, 0x64 },
	{5795, 0x1f, 0x90, 0x50, 0x02, 0x20, 0x64 },
	{5805, 0x1f, 0x90, 0x50, 0x02, 0x20, 0x64 },
	{5825, 0x1f, 0x90, 0x50, 0x02, 0x20, 0x64 }
};

STATIC chan_info_2063_sslpnphy_X17_aband_tweaks_t chan_info_2063_sslpnphy_X17_aband_tweaks[] = {
	{5170, 0xfff, 0x30, 0x6a, 0xe8, 0xf8, 0x20, 0x6a, 0xf9, 0xf8, 0x60, 0x60, 0xc0, 0xd0},
	{5180, 0xfff, 0x30, 0x6a, 0xe8, 0xf8, 0x20, 0x6a, 0xf9, 0xf8, 0x60, 0x60, 0xc0, 0xd0},
	{5190, 0xfff, 0x30, 0x6a, 0xe8, 0xf8, 0x20, 0x6a, 0xf9, 0xf8, 0x60, 0x60, 0xc0, 0xd0},
	{5200, 0xfff, 0x30, 0x6a, 0xd8, 0xf8, 0x20, 0x6a, 0xd8, 0xf8, 0x60, 0x60, 0xb0, 0xd0},
	{5210, 0xfff, 0x30, 0x6a, 0xc8, 0xf8, 0x20, 0x6a, 0xc8, 0xf8, 0x60, 0x60, 0xb0, 0xd0},
	{5220, 0xfff, 0x30, 0x6a, 0xc8, 0xf8, 0x20, 0x6a, 0xc8, 0xf8, 0x60, 0x60, 0xb0, 0xd0},
	{5230, 0xfff, 0x30, 0x6a, 0xc8, 0xf8, 0x20, 0x6a, 0xc8, 0xf8, 0x60, 0x60, 0xb0, 0xd0},
	{5240, 0xfff, 0x30, 0x6a, 0xc8, 0xf8, 0x20, 0x6a, 0xc8, 0xf8, 0x60, 0x60, 0xb0, 0xc0},
	{5260, 0xfff, 0x30, 0x6a, 0xb8, 0xf8, 0x20, 0x6a, 0xb8, 0xf8, 0x60, 0x60, 0xb0, 0xc0},
	{5280, 0xfff, 0x30, 0x6a, 0xb7, 0xf8, 0x20, 0x6a, 0xb7, 0xf8, 0x60, 0x60, 0xb0, 0xc0},
	{5300, 0xfff, 0x30, 0x6a, 0xb7, 0xf8, 0x20, 0x6a, 0xb7, 0xf8, 0x60, 0x60, 0xb0, 0xc0},
	{5320, 0xfff, 0x30, 0x6a, 0xb7, 0xf8, 0x20, 0x6a, 0xb7, 0xf8, 0x60, 0x60, 0xb0, 0xc0},

	{5500, 0xfff, 0x30, 0x6a, 0x66, 0x94, 0x30, 0x6a, 0x66, 0x94, 0x30, 0x60, 0x60, 0x90},
	{5520, 0xfff, 0x30, 0x6a, 0x66, 0x94, 0x30, 0x6a, 0x66, 0x94, 0x30, 0x60, 0x60, 0x90},
	{5540, 0xfff, 0x30, 0x6a, 0x56, 0x94, 0x30, 0x6a, 0x56, 0x94, 0x30, 0x60, 0x50, 0x90},
	{5560, 0xfff, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x60, 0x50, 0x80},
	{5580, 0xfff, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x60, 0x50, 0x80},
	{5600, 0xfff, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x60, 0x50, 0x80},
	{5620, 0xfff, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x6a, 0x56, 0x84, 0x30, 0x60, 0x50, 0x80},
	{5640, 0xfff, 0x30, 0x6a, 0x56, 0x74, 0x30, 0x6a, 0x36, 0x74, 0x30, 0x60, 0x30, 0x70},
	{5660, 0xfff, 0x30, 0x6a, 0x56, 0x74, 0x30, 0x6a, 0x36, 0x74, 0x30, 0x60, 0x30, 0x70},
	{5680, 0xfff, 0x30, 0x6a, 0x56, 0x74, 0x30, 0x6a, 0x36, 0x74, 0x30, 0x60, 0x30, 0x70},
	{5700, 0xfff, 0x30, 0x6a, 0x56, 0x74, 0x40, 0x6a, 0x36, 0x74, 0x20, 0x60, 0x30, 0x70},

	{5745, 0xfff, 0x30, 0x6a, 0xe8, 0x48, 0x40, 0x6a, 0x36, 0x04, 0x20, 0x60, 0x30, 0x00},
	{5765, 0xfff, 0x30, 0x6a, 0xe8, 0x48, 0x40, 0x6a, 0x36, 0x04, 0x20, 0x60, 0x30, 0x00},
	{5785, 0xfff, 0x30, 0x6a, 0xe8, 0x48, 0x40, 0x6a, 0x36, 0x04, 0x20, 0x60, 0x30, 0x00},
	{5805, 0xfff, 0x30, 0x6a, 0xe8, 0x48, 0x40, 0x6a, 0x36, 0x04, 0x20, 0x60, 0x30, 0x00},
	{5825, 0xfff, 0x30, 0x6a, 0xe8, 0x48, 0x40, 0x6a, 0x36, 0x04, 0x20, 0x60, 0x30, 0x00}
};

STATIC chan_info_2063_sslpnphy_aband_tweaks_t chan_info_2063_sslpnphy_aband_tweaks[] = {
	{5170, 0x1e3f, 0x40, 0x68, 0xf9, 0xf9, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5180, 0x1e3f, 0x40, 0x68, 0xf9, 0xf9, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5190, 0x1e3f, 0x40, 0x68, 0xf9, 0xf9, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5200, 0x1e3f, 0x40, 0x68, 0xf8, 0xf9, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5210, 0x1e3f, 0x40, 0x68, 0xf7, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5220, 0x1e3f, 0x40, 0x68, 0xf7, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5230, 0x1e3f, 0x40, 0x68, 0xf7, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5240, 0x1e3f, 0x40, 0x68, 0xf7, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5260, 0x1e3f, 0x40, 0x68, 0xf7, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5280, 0x1e3f, 0x40, 0x68, 0xe6, 0xf8, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5300, 0x1e3f, 0x40, 0x68, 0xe6, 0xf7, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5320, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},

	{5500, 0x1e3f, 0x10, 0x68, 0xb6, 0xf4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5520, 0x1e3f, 0x10, 0x68, 0xb6, 0xf4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5540, 0x1e3f, 0x10, 0x68, 0xa6, 0xf4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5560, 0x1e3f, 0x30, 0x68, 0x96, 0xe4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5580, 0x1e3f, 0x30, 0x68, 0x96, 0xe4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5600, 0x1e3f, 0x30, 0x68, 0x96, 0xd4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5620, 0x1e3f, 0x30, 0x68, 0xa6, 0xd4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5640, 0x1e3f, 0x20, 0x68, 0x96, 0xd4, 0xb8, 0x79, 0x77, 0x90, 0x30, 0x80, 0x80, 0x2, 0xee},
	{5660, 0x1e3f, 0x30, 0x68, 0x90, 0xe2, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5680, 0x1e3f, 0x30, 0x68, 0x90, 0xe2, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5700, 0x1e3f, 0x20, 0x68, 0x80, 0xd2, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},

	{5745, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5765, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5785, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5805, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee},
	{5825, 0x1e3f, 0x40, 0x68, 0xe6, 0xf6, 0xb8, 0x79, 0x77, 0x90, 0x50, 0x80, 0x80, 0x2, 0xee}
	};

STATIC chan_info_2063_sslpnphy_t chan_info_2063_sslpnphy_aband[] = {
	{  34, 5170, 0xfa, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0F, 0x08,
	0x0F, 0x07, 0x77, 0xdd, 0xCF, 0x70, 0x10, 0xF3, 0x00 },
	{  36, 5180, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0F, 0x07,
	0x0F, 0x07, 0x77, 0xdd, 0xCF, 0x70, 0x10, 0xF3, 0x55 },
	{  38, 5190, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0F, 0x07,
	0x0F, 0x07, 0x77, 0xdd, 0xCF, 0x70, 0x10, 0xF3, 0x55 },
	{  40, 5200, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0F, 0x07,
	0x0F, 0x07, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  42, 5210, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0E, 0x06,
	0x0E, 0x06, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  44, 5220, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0E, 0x06,
	0x0E, 0x06, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  46, 5230, 0xf9, 0x05, 0x0d, 0x05, 0x05, 0x55, 0x0E, 0x06,
	0x0E, 0x06, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  48, 5240, 0xf9, 0x04, 0x0C, 0x05, 0x05, 0x55, 0x0D, 0x05,
	0x0D, 0x05, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  52, 5260, 0xf8, 0x04, 0x0C, 0x05, 0x05, 0x55, 0x0C, 0x04,
	0x0C, 0x04, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  54, 5270, 0xf8, 0x04, 0x0C, 0x05, 0x05, 0x55, 0x0C, 0x04,
	0x0C, 0x04, 0x77, 0xdd, 0xBF, 0x70, 0x10, 0xF3, 0x55 },
	{  56, 5280, 0xf8, 0x04, 0x0b, 0x05, 0x05, 0x55, 0x0B, 0x03,
	0x0B, 0x03, 0x77, 0xdd, 0xAF, 0x70, 0x10, 0xF3, 0x55 },
	{  60, 5300, 0xf8, 0x03, 0x0b, 0x05, 0x05, 0x55, 0x0A, 0x02,
	0x0A, 0x02, 0x77, 0xdd, 0xAF, 0x60, 0x10, 0xF3, 0x55 },
	{  62, 5310, 0xf8, 0x03, 0x0b, 0x05, 0x05, 0x55, 0x0A, 0x02,
	0x0A, 0x02, 0x77, 0xdd, 0xAF, 0x60, 0x10, 0xF3, 0x55 },
	{  64, 5320, 0xf7, 0x03, 0x0a, 0x05, 0x05, 0x55, 0x09, 0x01,
	0x09, 0x01, 0x77, 0xdd, 0xAF, 0x60, 0x10, 0xF3, 0x55 },
	{ 100, 5500, 0xf4, 0x01, 0x06, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x8F, 0x50, 0x00, 0x03, 0x66 },
	{ 102, 5510, 0xf4, 0x01, 0x06, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x8F, 0x50, 0x00, 0x03, 0x66 },
	{ 104, 5520, 0xf4, 0x01, 0x06, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 108, 5540, 0xf3, 0x01, 0x05, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 110, 5550, 0xf3, 0x01, 0x05, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 112, 5560, 0xf3, 0x01, 0x05, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 116, 5580, 0xf3, 0x00, 0x05, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 118, 5590, 0xf3, 0x00, 0x04, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 120, 5600, 0xf3, 0x00, 0x04, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x7F, 0x50, 0x00, 0x03, 0x66 },
	{ 124, 5620, 0xf3, 0x00, 0x04, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x66 },
	{ 126, 5630, 0xf3, 0x00, 0x04, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x66 },
	{ 128, 5640, 0xf3, 0x00, 0x03, 0x06, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x66 },
	{ 132, 5660, 0xf2, 0x00, 0x03, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x77 },
	{ 134, 5670, 0xf2, 0x00, 0x03, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x77 },
	{ 136, 5680, 0xf2, 0x00, 0x03, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x6F, 0x40, 0x00, 0x03, 0x77 },
	{ 140, 5700, 0xf2, 0x00, 0x02, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x5F, 0x40, 0x00, 0x03, 0x77 },
	{ 149, 5745, 0xf1, 0x00, 0x01, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x5F, 0x40, 0x00, 0x03, 0x77 },
	{ 151, 5755, 0xf1, 0x00, 0x01, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x5F, 0x40, 0x00, 0x03, 0x77 },
	{ 153, 5765, 0xf1, 0x00, 0x01, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x4F, 0x30, 0x00, 0x03, 0x77 },
	{ 157, 5785, 0xf0, 0x00, 0x01, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x4F, 0x30, 0x00, 0x03, 0x77 },
	{ 159, 5795, 0xf0, 0x00, 0x01, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x4F, 0x30, 0x00, 0x03, 0x77 },
	{ 161, 5805, 0xf0, 0x00, 0x00, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x4F, 0x30, 0x00, 0x03, 0x77 },
	{ 165, 5825, 0xf0, 0x00, 0x00, 0x07, 0x05, 0x55, 0x00, 0x00,
	0x00, 0x00, 0x77, 0xdd, 0x3F, 0x30, 0x00, 0x03, 0x77 }
};
#endif /* BAND5G */

/* Autogenerated by 2063_chantbl_tcl2c.tcl */
STATIC chan_info_2063_sslpnphy_t chan_info_2063_sslpnphy[] = {
	{1, 2412, 0xff, 0x3c, 0x3c, 0x04, 0x0e, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x0c },
	{2, 2417, 0xff, 0x3c, 0x3c, 0x04, 0x0e, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x0b },
	{3, 2422, 0xff, 0x3c, 0x3c, 0x04, 0x0e, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x09 },
	{4, 2427, 0xff, 0x2c, 0x2c, 0x04, 0x0d, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x08 },
	{5, 2432, 0xff, 0x2c, 0x2c, 0x04, 0x0d, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x07 },
	{6, 2437, 0xff, 0x2c, 0x2c, 0x04, 0x0c, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x06 },
	{7, 2442, 0xff, 0x2c, 0x2c, 0x04, 0x0b, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x05 },
	{8, 2447, 0xff, 0x2c, 0x2c, 0x04, 0x0b, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x04 },
	{9, 2452, 0xff, 0x1c, 0x1c, 0x04, 0x0a, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x04 },
	{10, 2457, 0xff, 0x1c, 0x1c, 0x04, 0x09, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x03 },
	{11, 2462, 0xfe, 0x1c, 0x1c, 0x04, 0x08, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x03 },
	{12, 2467, 0xfe, 0x1c, 0x1c, 0x04, 0x07, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x02 },
	{13, 2472, 0xfe, 0x1c, 0x1c, 0x04, 0x06, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x02 },
	{14, 2484, 0xfe, 0x0c, 0x0c, 0x04, 0x02, 0x55, 0x05, 0x05, 0x05, 0x05,
	0x77, 0x44, 0x80, 0x80, 0x70, 0xf3, 0x01 }
};

sslpnphy_radio_regs_t WLBANDINITDATA(sslpnphy_radio_regs_2063)[] = {
	{ 0x4000,		0,		0 },
	{ 0x800A,		0x1,		0 },
	{ 0x4010,		0,		0 },
	{ 0x4011,		0,		0 },
	{ 0x4012,		0,		0 },
	{ 0x4013,		0,		0 },
	{ 0x4014,		0,		0 },
	{ 0x4015,		0,		0 },
	{ 0x4016,		0,		0 },
	{ 0x4017,		0,		0 },
	{ 0x4018,		0,		0 },
	{ 0xC01C,		0xe8,		0xd4 },
	{ 0xC01D,		0xa7,		0x53 },
	{ 0xC01F,		0xf0,		0xf },
	{ 0xC021,		0x5e,		0x5e },
	{ 0xC022,		0x7e,		0x7e },
	{ 0xC023,		0xf0,		0xf0 },
	{ 0xC026,		0x2,		0x2 },
	{ 0xC027,		0x7f,		0x7f },
	{ 0xC02A,		0xc,		0xc },
	{ 0x802C,		0x3c,		0x3f },
	{ 0x802D,		0xfc,		0xfe },
	{ 0xC032,		0x8,		0x8 },
	{ 0xC036,		0x60,		0x60 },
	{ 0xC03A,		0x30,		0x30 },
	{ 0xC03D,		0xc,		0xb },
	{ 0xC03E,		0x10,		0xf },
	{ 0xC04C,		0x3d,		0xfd },
	{ 0xC053,		0x2,		0x2 },
	{ 0xC057,		0x56,		0x56 },
	{ 0xC076,		0xf7,		0xf7 },
	{ 0xC0B2,		0xf0,		0xf0 },
	{ 0xC0C4,		0x71,		0x71 },
	{ 0xC0C5,		0x71,		0x71 },
	{ 0x80CF,		0xf0,		0x30 },
	{ 0xC0DF,		0x77,		0x77 },
	{ 0xC0E3,		0x3,		0x3 },
	{ 0xC0E4,		0xf,		0xf },
	{ 0xC0E5,		0xf,		0xf },
	{ 0xC0EC,		0x77,		0x77 },
	{ 0xC0EE,		0x77,		0x77 },
	{ 0xC0F3,		0x4,		0x4 },
	{ 0xC0F7,		0x9,		0x9 },
	{ 0x810B,		0,		0x4 },
	{ 0xC11D,		0x3,		0x3 },
	{ 0xFFFF,		0,		0}
};

#if !defined(PHYHAL)
typedef struct {
	/* TX IQ LO cal results */
	uint16 txiqlocal_bestcoeffs[11];
	uint txiqlocal_bestcoeffs_valid;
	/* PAPD results */
	uint32 papd_compdelta_tbl[64];
	uint papd_table_valid;
	uint16 analog_gain_ref, lut_begin, lut_end, lut_step, rxcompdbm, papdctrl;

	/* RX IQ cal results */
	uint16 rxiqcal_coeffa0;
	uint16 rxiqcal_coeffb0;
	uint16 rxiq_enable;
	uint8 rxfe;
	uint8 loopback2, loopback1;

} sslpnphy_cal_results_t;

#endif /* PHYHAL */

/* %%%%%%%%%%%%%%%%%%%%%%%% */
/* SSLPNPHY local functions */
/* %%%%%%%%%%%%%%%%%%%%%%%% */
static void wlc_sslpnphy_load_filt_coeff(phy_info_t *pi, uint16 reg_address,
	uint16 *coeff_val, uint count);
STATIC void wlc_sslpnphy_set_radio_loft(phy_info_t *pi, uint8 ei0,
	uint8 eq0, uint8 fi0, uint8 fq0);
static void wlc_sslpnphy_restore_calibration_results(phy_info_t *pi);
static void wlc_sslpnphy_restore_papd_calibration_results(phy_info_t *pi);
void wlc_sslpnphy_periodic_cal(phy_info_t *pi);	/* nonstatic in PHYBOM */

static void wlc_sslpnphy_noise_init(phy_info_t *pi);

static void wlc_sslpnphy_set_rx_iq_comp(phy_info_t *pi, uint16 a0, uint16 b0);
static void wlc_sslpnphy_get_rx_iq_comp(phy_info_t *pi, uint16 *a0, uint16 *b0);

STATIC void wlc_sslpnphy_pktengtx(wlc_phy_t *ppi, wl_pkteng_t *pkteng,
	uint8 rate,	struct ether_addr *sa, uint32 wait_delay);

static void wlc_sslpnphy_papd_cal_txpwr(phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	bool frcRxGnCtrl,
	bool frcTxGnCtrl,
	uint16 frcTxidx);

STATIC void BCMROMOVERLAYFN(1, wlc_sslpnphy_set_rx_gain_by_distribution)(phy_info_t *pi,
	uint16 pga, uint16 biq2, uint16 pole1, uint16 biq1, uint16 tia, uint16 lna2,
	uint16 lna1);
STATIC void wlc_sslpnphy_set_swctrl_override(phy_info_t *pi, uint8 index);
void wlc_sslpnphy_get_radio_loft(phy_info_t *pi, uint8 *ei0,
	uint8 *eq0, uint8 *fi0, uint8 *fq0);	/* nonstatic in PHYBOM */

STATIC uint32 wlc_lpphy_qdiv_roundup(uint32 divident, uint32 divisor, uint8 precision);
STATIC void wlc_sslpnphy_set_pa_gain(phy_info_t *pi, uint16 gain);
STATIC void wlc_sslpnphy_set_trsw_override(phy_info_t *pi, bool tx, bool rx);
STATIC void wlc_sslpnphy_stop_ddfs(phy_info_t *pi);
STATIC void wlc_sslpnphy_set_bbmult(phy_info_t *pi, uint8 m0);
STATIC uint8 wlc_sslpnphy_get_bbmult(phy_info_t *pi);

STATIC void wlc_sslpnphy_get_tx_gain(phy_info_t *pi,  sslpnphy_txgains_t *gains);
STATIC void wlc_sslpnphy_set_tx_gain_override(phy_info_t *pi, bool bEnable);
STATIC void wlc_sslpnphy_toggle_afe_pwdn(phy_info_t *pi);
void BCMROMOVERLAYFN(1, wlc_sslpnphy_stop_tx_tone)(phy_info_t *pi);
STATIC void wlc_sslpnphy_rx_gain_override_enable(phy_info_t *pi, bool enable);
STATIC void wlc_sslpnphy_set_tx_gain(phy_info_t *pi,  sslpnphy_txgains_t *target_gains);
STATIC void wlc_sslpnphy_set_rx_gain(phy_info_t *pi, uint32 gain);
STATIC void BCMROMOVERLAYFN(1, wlc_sslpnphy_saveIntpapdlut)(phy_info_t *pi, int8 Max, int8 Min,
	uint32 *papdIntlut, uint8 *papdIntlutVld);
STATIC void BCMROMOVERLAYFN(1, wlc_sslpnphy_GetpapdMaxMinIdxupdt)(phy_info_t *pi,
	int8 *maxUpdtIdx, int8 *minUpdtIdx);
STATIC bool BCMROMOVERLAYFN(1, wlc_sslpnphy_rx_iq_est)(phy_info_t *pi, uint16 num_samps,
	uint8 wait_time, sslpnphy_iq_est_t *iq_est);
STATIC int wlc_sslpnphy_aux_adc_accum(phy_info_t *pi, uint32 numberOfSamples,
    uint32 waitTime, int32 *sum, int32 *prod);
STATIC void wlc_sslpnphy_rx_pu(phy_info_t *pi, bool bEnable);
void BCMROMOVERLAYFN(1, wlc_sslpnphy_tx_pu)(phy_info_t *pi, bool bEnable);


static void wlc_sslpnphy_afe_clk_init(phy_info_t *pi, uint8 mode);
void wlc_sslpnphy_set_tx_pwr_ctrl(phy_info_t *pi, uint16 mode);	/* nonstatic in PHYBOM */
STATIC bool BCMROMOVERLAYFN(1, wlc_sslpnphy_calc_rx_iq_comp)(phy_info_t *pi,  uint16 num_samps);

/* %%%%%%%%%%%%%%%%%%%% */
/*  power control	*/
/* %%%%%%%%%%%%%%%%%%%% */

static bool wlc_sslpnphy_txpwr_srom_read(phy_info_t *pi);
static void wlc_sslpnphy_store_tbls(phy_info_t *pi);
static void wlc_sslpnphy_txpwr_srom_convert(uint8 *srom_max, uint16 *pwr_offset, uint8 tmp_max_pwr,
	uint8 rate_start, uint8 rate_end);

STATIC uint16 wlc_sslpnphy_get_pa_gain(phy_info_t *pi);

static void wlc_sslpnphy_lock_ucode_phyreg(phy_info_t *pi, int wait);
static void wlc_sslpnphy_unlock_ucode_phyreg(phy_info_t *pi);

typedef enum {
	SSLPNPHY_TSSI_PRE_PA,
	SSLPNPHY_TSSI_POST_PA,
	SSLPNPHY_TSSI_EXT
} sslpnphy_tssi_mode_t;

/* START functions that may be put into overlays */
OSTATIC void wlc_sslpnphy_idle_tssi_est(phy_info_t *pi);
OSTATIC uint16 sslpnphy_iqlocc_write(phy_info_t *pi, uint8 data);
OSTATIC void wlc_sslpnphy_run_samples(phy_info_t *pi, uint16 num_samps, uint16 num_loops,
                              uint16 wait, bool iqcalmode);
void wlc_sslpnphy_start_tx_tone(phy_info_t *pi, int32 f_kHz, uint16 max_val,
                                bool iqcalmode);
OSTATIC bool wlc_sslpnphy_iqcal_wait(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_clear_trsw_override(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_tx_iqlo_cal(phy_info_t *pi, sslpnphy_txgains_t *target_gains,
                              sslpnphy_cal_mode_t cal_mode, bool keep_tone);
void wlc_sslpnphy_get_tx_iqcc(phy_info_t *pi, uint16 *a, uint16 *b);	/* nonstatic in PHYBOM */
uint16 wlc_sslpnphy_get_tx_locc(phy_info_t *pi);	/* nonstatic in PHYBOM */

OSTATIC void wlc_sslpnphy_set_tx_filter_bw(phy_info_t *pi, uint16 bw);
OSTATIC void wlc_sslpnphy_papd_cal_setup_cw(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_papd_cal_core(phy_info_t *pi, sslpnphy_papd_cal_type_t calType,
                                bool rxGnCtrl, uint8 num_symbols4lpgn, bool init_papd_lut,
                                uint16 papd_bbmult_init, uint16 papd_bbmult_step,
                                bool papd_lpgn_ovr, uint16 LPGN_I, uint16 LPGN_Q);
OSTATIC uint32 wlc_sslpnphy_papd_rxGnCtrl(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                                  bool frcRxGnCtrl, uint8 CurTxGain);
OSTATIC void InitIntpapdlut(uint8 Max, uint8 Min, uint8 *papdIntlutVld);
OSTATIC void wlc_sslpnphy_compute_delta(phy_info_t *pi);
OSTATIC void genpapdlut(phy_info_t *pi, uint32 *papdIntlut, uint8 *papdIntlutVld);
OSTATIC void wlc_sslpnphy_papd_cal(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                           sslpnphy_txcalgains_t *txgains, bool frcRxGnCtrl,
                           uint16 num_symbols, uint8 papd_lastidx_search_mode);
OSTATIC void wlc_sslpnphy_vbatsense_papd_cal(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                                     sslpnphy_txcalgains_t *txgains);
OSTATIC int8 wlc_sslpnphy_gain_based_psat_detect(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                                    bool frcRxGnCtrl, sslpnphy_txcalgains_t *txgains,
                                    uint8 cur_pwr);
OSTATIC void wlc_sslpnphy_min_pd_search(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                                bool frcRxGnCtrl, sslpnphy_txcalgains_t *txgains);
OSTATIC int8 wlc_sslpnphy_psat_detect(phy_info_t *pi, uint8 cur_index, uint8 cur_pwr);
OSTATIC void wlc_sslpnphy_run_ddfs(phy_info_t *pi, int i_on, int q_on, int incr1, int incr2,
                           int scale_index);
OSTATIC bool wlc_sslpnphy_rx_iq_cal(phy_info_t *pi, const sslpnphy_rx_iqcomp_t *iqcomp,
                            int iqcomp_sz, bool use_noise, bool tx_switch, bool rx_switch,
                            bool pa, int tx_gain_idx);
void wlc_sslpnphy_full_cal(phy_info_t *pi);


OSTATIC void wlc_sslpnphy_detection_disable(phy_info_t *pi, bool mode);
OSTATIC void wlc_sslpnphy_noise_fifo_init(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_noise_measure_setup(phy_info_t *pi);
OSTATIC uint32 wlc_sslpnphy_get_rxiq_accum(phy_info_t *pi);
OSTATIC uint32 wlc_sslpnphy_abs_time(uint32 end, uint32 start);
OSTATIC void wlc_sslpnphy_noise_measure_time_window(phy_info_t *pi, uint32 window_time, uint32 *minpwr,
                                            uint32 *maxpwr, bool *measurement_valid);
OSTATIC uint32 wlc_sslpnphy_noise_fifo_min(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_noise_fifo_avg(phy_info_t *pi, uint32 *avg_noise);
OSTATIC void wlc_sslpnphy_noise_measure_chg_listen_gain(phy_info_t *pi, int8 change_sign);
OSTATIC void wlc_sslpnphy_noise_measure_change_rxpo(phy_info_t *pi, uint32 avg_noise);
OSTATIC void wlc_sslpnphy_noise_measure_computeNf(phy_info_t *pi);
OSTATIC uint8 wlc_sslpnphy_rx_noise_lut(phy_info_t *pi, uint8 noise_val, uint8 ptr[][2], uint8 array_size);
OSTATIC void wlc_sslpnphy_reset_radioctrl_crsgain(phy_info_t *pi);

#ifdef BAND5G
OSTATIC void wlc_sslpnphy_disable_pad(phy_info_t *pi);
OSTATIC void wlc_sslpnphy_pll_aband_tune(phy_info_t *pi, uint8 channel);
#endif /* BAND5G */
OSTATIC void wlc_sslpnphy_papd_cal_txpwr(phy_info_t *pi, sslpnphy_papd_cal_type_t cal_type,
                                        bool frcRxGnCtrl, bool frcTxGnCtrl, uint16 frcTxidx);

OSTATIC void wlc_sslpnphy_get_rx_iq_comp(phy_info_t *pi, uint16 *a0, uint16 *b0);
void wlc_sslpnphy_periodic_cal(phy_info_t *pi);	/* nonstatic in PHYBOM */
OSTATIC void wlc_sslpnphy_restore_papd_calibration_results(phy_info_t *pi);

/* END functions that may be put into overlays */

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*  function implementation   					*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

#if defined(LMAC_HNDRTE_CONSOLE) && !defined(HNDRTE_CONSOLE)
#error "Can't have LMAC console commands defined when HNDRTE_CONSOLE is not defined"
#endif

bool
wlc_phy_tpc_isenabled_sslpnphy(phy_info_t *pi) {
	return SSLPNPHY_TX_PWR_CTRL_HW == wlc_sslpnphy_get_tx_pwr_ctrl(pi);
}

static void
wlc_sslpnphy_lock_ucode_phyreg(phy_info_t *pi, int wait)
{
	WL_MCTRL(pi, MCTL_PHYLOCK, MCTL_PHYLOCK );
	(void)R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol);

	OSL_DELAY(wait);
}

static void
wlc_sslpnphy_unlock_ucode_phyreg(phy_info_t *pi)
{
	(void) R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->phyversion);
	WL_MCTRL(pi, MCTL_PHYLOCK, 0);
}

bool 
wlc_phy_attach_sslpnphy(phy_info_t *pi)
{
	int i;
#ifdef PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#else
	#define ph (pi)
#endif /* PHYHAL */

	if ((0 == (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_NOPA)) && !NORADIO_ENAB(pi->pubpi)) {
		pi->hwpwrctrl = TRUE;
		pi->hwpwrctrl_capable = TRUE;
	}

	/* Get xtal frequency from PMU */
	pi->xtalfreq = si_alp_clock(GENERIC_PHY_INFO(pi)->sih);
	ASSERT(0 == (XTALFREQ(pi->xtalfreq) % 1000));

	/* set papd_rxGnCtrl_init to 0 */
	sslpnphy_specific->sslpnphy_papd_rxGnCtrl_init = 0;

	WL_INFORM(("wl%d: %s: using %d.%d MHz xtalfreq for RF PLL\n",
			   GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
			   XTALFREQ(pi->xtalfreq) / 1000000, XTALFREQ(pi->xtalfreq) % 1000000));

	if (!wlc_sslpnphy_txpwr_srom_read(pi))
		return NULL;
	wlc_sslpnphy_store_tbls(pi);

	/* Initialize default power indexes */
	for (i = 0; i <= LAST_5G_CHAN; i++) {
		sslpnphy_specific->sslpnphy_tssi_idx_ch[i] =  (i >= FIRST_5G_CHAN) ?
			SSLPNPHY_TX_PWR_CTRL_START_INDEX_5G : SSLPNPHY_TX_PWR_CTRL_START_INDEX_2G;
	}
	return TRUE;
}

void
wlc_phy_detach_sslpnphy(phy_info_t *pi)
{
#ifdef PHYHAL					    /* noise timer is removed in PHYHAL */
	MFREE(pi->sh->osh, pi->u.pi_sslpnphy, sizeof(phy_info_sslpnphy_t));
#else		
	if (pi->phynoise_timer) {
		wl_free_timer(((wlc_info_t *)pi->wlc)->wl, pi->phynoise_timer);
		pi->phynoise_timer = NULL;
	}
#endif /* PHYHAL */

}

void
#ifdef PHYHAL
wlc_sslpnphy_write_table(phy_info_t *pi, const phytbl_info_t *pti)
#else
wlc_sslpnphy_write_table(phy_info_t *pi, CONST phytbl_info_t *pti)
#endif /* PHYHAL */
{
	wlc_phy_write_table(pi, pti, SSLPNPHY_TableAddress,
	                    SSLPNPHY_TabledataHi, SSLPNPHY_TabledataLo);
}

void
#ifdef PHYHAL
wlc_sslpnphy_read_table(phy_info_t *pi, phytbl_info_t *pti)
#else
wlc_sslpnphy_read_table(phy_info_t *pi, CONST phytbl_info_t *pti)
#endif /* PHYHAL */
{
	wlc_phy_read_table(pi, pti, SSLPNPHY_TableAddress,
	                   SSLPNPHY_TabledataHi, SSLPNPHY_TabledataLo);
}

static uint
wlc_sslpnphy_init_radio_regs(phy_info_t *pi, sslpnphy_radio_regs_t *radioregs, uint16 core_offset)
{
	uint i = 0;
#ifdef PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	do {
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			if (radioregs[i].address & 0x8000) {
				write_radio_reg(pi, (radioregs[i].address & 0x3fff) | core_offset,
					(uint16)radioregs[i].init_a);
			}
		} else {
			if (radioregs[i].address & 0x4000) {
				write_radio_reg(pi, (radioregs[i].address & 0x3fff) | core_offset,
					(uint16)radioregs[i].init_g);
			}
		}

		i++;
	} while (radioregs[i].address != 0xffff);
	if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
		write_radio_reg(pi, RADIO_2063_GRX_SP_6, 0);
		write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0x33);
		/* write_radio_reg(pi, RADIO_2063_LOCAL_OVR_1, 0xc0);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_4, 0x0);
		*/
	}
#ifdef SSLPNLOWPOWER
	write_radio_reg(pi, RADIO_2063_LOCAL_OVR_1, 0xc0);
	write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_4, 0x0);
#endif
	return i;
}


static void
wlc_sslpnphy_common_write_table(phy_info_t *pi, uint32 tbl_id,
	CONST void *tbl_ptr, uint32 tbl_len, uint32 tbl_width,
	uint32 tbl_offset);
static void
wlc_sslpnphy_load_tx_gain_table(phy_info_t *pi,
        CONST sslpnphy_tx_gain_tbl_entry * gain_table)
{
	uint16 j;
	uint32 val;
	uint16 pa_gain;

	if (CHSPEC_IS5G(pi->radio_chanspec))
		pa_gain = 0x70;
	else
		pa_gain = 0x70;
	if (CHSPEC_IS5G(pi->radio_chanspec) && (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)) {
		pa_gain = 0x10;
	}
	for (j = 0; j < 128; j++) {
		val = ((uint32)pa_gain << 24) |
			(gain_table[j].pad << 16) |
			(gain_table[j].pga << 8) |
			(gain_table[j].gm << 0);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_GAIN_OFFSET + j);

		val = (gain_table[j].dac << 28) |
			(gain_table[j].bb_mult << 20);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + j);
	}
}
static void
wlc_sslpnphy_load_rfpower(phy_info_t *pi)
{
	uint32 val;
	uint8 index;

	for (index = 0; index < 128; index++) {
		val = index * 32 / 10;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_PWR_OFFSET + index);
	}
}
static void wlc_sslpnphy_clear_papd_comptable(phy_info_t *pi)
{
	uint32 j;
	uint32 temp_offset[128];

	bzero(temp_offset, sizeof(temp_offset));
	for (j = 1; j < 128; j += 2)
		temp_offset[j] = 0x80000;

	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
		temp_offset, 128, 32, 0);
}

extern CONST uint16 sw_ctrl_tbl_rev02[];

/* initialize all the tables defined in auto-generated sslpnphytbls.c,
 * see sslpnphyprocs.tcl, proc sslpnphy_tbl_init
 */
static void
WLBANDINITFN(wlc_sslpnphy_restore_tbls)(phy_info_t *pi)
{
#ifdef PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
			sslpnphy_specific->sslpnphy_gain_idx_2g, 152, 32, 0);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_TBL,
			sslpnphy_specific->sslpnphy_gain_tbl_2g, 96, 16, 0);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
			sslpnphy_specific->sslpnphy_swctrl_lut_2g, 64, 16, 0);
	}
#ifdef BAND5G
	else {
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
			sslpnphy_specific->sslpnphy_gain_idx_5g, 152, 32, 0);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_TBL,
			sslpnphy_specific->sslpnphy_gain_tbl_5g, 96, 16, 0);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
			sslpnphy_specific->sslpnphy_swctrl_lut_5g, 64, 16, 0);

	}
#endif
}

static void
BCMATTACHOVERLAYFN(1, wlc_sslpnphy_store_tbls)(phy_info_t *pi)
{
#ifdef PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */

	bool x17_board_flag = ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ? 1 : 0);
	bool N90_board_flag = ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID) ? 1 : 0);

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		/* Allocate software memory for the 2G tables */
		sslpnphy_specific->sslpnphy_gain_idx_2g = (uint32 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 152 * sizeof(uint32));
		sslpnphy_specific->sslpnphy_gain_tbl_2g = (uint16 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 96 * sizeof(uint16));
		sslpnphy_specific->sslpnphy_swctrl_lut_2g = (uint16 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 64 * sizeof(uint16));

		bcopy(gain_idx_tbl_rev0, sslpnphy_specific->sslpnphy_gain_idx_2g,
			gain_idx_tbl_rev0_sz);
		bcopy(gain_tbl_rev0, sslpnphy_specific->sslpnphy_gain_tbl_2g,
			gain_tbl_rev0_sz);
		bcopy(sw_ctrl_tbl_rev0, sslpnphy_specific->sslpnphy_swctrl_lut_2g,
			sw_ctrl_tbl_rev0_sz);
		if ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
			bcopy(sslpnphy_gain_idx_extlna_cmrxaci_tbl, sslpnphy_specific->sslpnphy_gain_idx_2g,
				sslpnphy_gain_idx_extlna_cmrxaci_tbl_sz);
			bcopy(sslpnphy_gain_extlna_cmrxaci_tbl, sslpnphy_specific->sslpnphy_gain_tbl_2g,
				sslpnphy_gain_extlna_cmrxaci_tbl_sz);
			if (x17_board_flag || N90_board_flag ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID)) {
				bcopy(sslpnphy_gain_idx_extlna_2g_x17, sslpnphy_specific->sslpnphy_gain_idx_2g,
					sslpnphy_gain_idx_extlna_2g_x17_sz);
				bcopy(sslpnphy_gain_tbl_extlna_2g_x17, sslpnphy_specific->sslpnphy_gain_tbl_2g,
					sslpnphy_gain_tbl_extlna_2g_x17_sz);

			}
			if ((x17_board_flag) || (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
				BCM94329MOTOROLA_SSID)) {
				bcopy(sw_ctrl_tbl_rev0_olympic_x17_2g, sslpnphy_specific->sslpnphy_swctrl_lut_2g,
					sw_ctrl_tbl_rev0_olympic_x17_2g_sz);
			}
		}
#ifdef BAND5G
		/* Allocate software memory for the 5G tables */
		sslpnphy_specific->sslpnphy_gain_idx_5g = (uint32 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 152 * sizeof(uint32));
		sslpnphy_specific->sslpnphy_gain_tbl_5g = (uint16 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 96 * sizeof(uint16));
		sslpnphy_specific->sslpnphy_swctrl_lut_5g = (uint16 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 64 * sizeof(uint16));
		sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband = (sslpnphy_tx_gain_tbl_entry *)
			MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * 5 * sizeof(uchar));
		sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband = (sslpnphy_tx_gain_tbl_entry *)
			MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * 5 * sizeof(uchar));

		/* Copy all rx tables */
		bcopy(dot11lpphy_rx_gain_init_tbls_A_tbl, sslpnphy_specific->sslpnphy_gain_idx_5g,
			dot11lpphy_rx_gain_init_tbls_A_tbl_sz);
		bcopy(gain_tbl_rev0, sslpnphy_specific->sslpnphy_gain_tbl_5g,
			gain_tbl_rev0_sz);
		bcopy(sw_ctrl_tbl_rev1_5Ghz_tbl, sslpnphy_specific->sslpnphy_swctrl_lut_5g,
			sw_ctrl_tbl_rev1_5Ghz_tbl_sz);
		if (x17_board_flag) {
			bcopy(sslpnphy_gain_idx_extlna_5g_x17, sslpnphy_specific->sslpnphy_gain_idx_5g,
				sslpnphy_gain_idx_extlna_5g_x17_sz);
			bcopy(sslpnphy_gain_tbl_extlna_5g_x17, sslpnphy_specific->sslpnphy_gain_tbl_5g,
				sslpnphy_gain_tbl_extlna_5g_x17_sz);
		}
		if ((x17_board_flag) || (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
			BCM94329MOTOROLA_SSID)) {
			bcopy(sw_ctrl_tbl_rev0_olympic_x17_5g, sslpnphy_specific->sslpnphy_swctrl_lut_5g,
				sw_ctrl_tbl_rev0_olympic_x17_5g_sz);
		}

		/* Copy tx gain tables */
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) {
			bcopy(dot11sslpnphy_2GHz_gaintable_rev0, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband,
				128 * 5 * sizeof(uchar));
			bcopy(dot11sslpnphy_2GHz_gaintable_rev0, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband,
				128 * 5 * sizeof(uchar));
		} else {
			bcopy(dot11lpphy_5GHz_gaintable_MidBand, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband,
				128 * 5 * sizeof(uchar));
			bcopy(dot11lpphy_5GHz_gaintable_HiBand, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband,
				128 * 5 * sizeof(uchar));
		}
		if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA) {
			bcopy(dot11lpphy_5GHz_gaintable_X17_ePA,
				sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband,
				128 * 5 * sizeof(uchar));
			bcopy(dot11lpphy_5GHz_gaintable_X17_ePA,
				sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband,
				128 * 5 * sizeof(uchar));
		}
#endif /* BAND 5G */
	}
#ifdef BAND5G
	else {
		if (SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {
			/* Allocate software memory for the 5G tables */
			sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband = (sslpnphy_tx_gain_tbl_entry *)
			    MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * 5 * sizeof(uchar));
			sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband = (sslpnphy_tx_gain_tbl_entry *)
			    MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * 5 * sizeof(uchar));

			/* Copy tx gain tables */
			bcopy(dot11lpphy_5GHz_gaintable_4319_midband,
			      sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband,
			      128 * 5 * sizeof(uchar));
			bcopy(dot11lpphy_5GHz_gaintable_4319_hiband,
			      sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband,
			      128 * 5 * sizeof(uchar));
		}
	}
#endif /* BAND 5G */


}

/* This routine is to load palm's btcx fem. */
/* This routine is to load palm's btcx fem. */
/* force_update will ensure that the control lines are driven correctly */
void
wlc_load_bt_fem_combiner_sslpnphy(phy_info_t *pi, bool force_update)
{
	bool suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	uint8 band_idx;
	uint16 tempsense;
#ifdef PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
        /* Skip update when called during init, called repeatedly with the same value,
	 * and when the band isn't 2.4G
	 */
	if ((!force_update &&
		sslpnphy_specific->fem_combiner_target_state == sslpnphy_specific->fem_combiner_current_state)
		|| (!CHSPEC_IS2G(pi->radio_chanspec))) {
		return;
	}

	if (!suspend)
		WL_SUSPEND_MAC_AND_WAIT(pi);
	if (sslpnphy_specific->fem_combiner_target_state) {
		tempsense = read_phy_reg(pi, SSLPNPHY_TempSenseCorrection);
		write_phy_reg(pi, SSLPNPHY_TempSenseCorrection, tempsense + 6);
		/* Program front-end control lines for 'combine' mode */
		si_pmu_res_4319_swctrl_war(GENERIC_PHY_INFO(pi)->sih, GENERIC_PHY_INFO(pi)->osh, TRUE);
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319BHEMU3_SSID)
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
				sw_ctrl_tbl_rev02_shared_mlap_emu3_combiner, 32, 16, 0);
		else
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
				sw_ctrl_tbl_rev02_shared_mlap_combiner, 32, 16, 0);
	} else {
		/* Adjust power index back to neutal for 'bypass' mode */
		tempsense = read_phy_reg(pi, SSLPNPHY_TempSenseCorrection);
		write_phy_reg(pi, SSLPNPHY_TempSenseCorrection,
			tempsense > 6 ? tempsense - 6 : 0);
		/* Program front-end control lines for 'bypass' mode */
		si_pmu_res_4319_swctrl_war(GENERIC_PHY_INFO(pi)->sih, GENERIC_PHY_INFO(pi)->osh, FALSE);
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319BHEMU3_SSID)
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
				sw_ctrl_tbl_rev02_shared_mlap_emu3, 32, 16, 0);
		else
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
				sw_ctrl_tbl_rev02_shared_mlap, 32, 16, 0);
	}

	sslpnphy_specific->fem_combiner_current_state = sslpnphy_specific->fem_combiner_target_state;

	if(!force_update){
		/* force_update is really being used as, 'suppress_cal',
		 * so that we will skip calibration during init, and only
		 * calibrate when the user executes coex_profile iovar
		 */
		band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);
		wlc_phy_chanspec_set((wlc_phy_t*)pi, CH20MHZ_CHSPEC(pi->radio_chanspec));
	}

	if (!suspend)
		WL_ENABLE_MAC(pi);
}


static void
WLBANDINITFN(wlc_sslpnphy_tbl_init)(phy_info_t *pi)
{
	uint idx, val;
	uint16 j;
	uint32 tbl_val[2];

	uint8 phybw40 = IS40MHZ(pi);
	bool x17_board_flag = ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ? 1 : 0);
	bool N90_board_flag = ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID) ? 1 : 0);
	bool ninja_board_flag = (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID);

	/* Resetting the txpwrctrl tbl */
	val = 0;
	for (j = 0; j < 703; j++) {
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, j);
	}
	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		for (idx = 0; idx < dot11sslpnphytbl_info_sz_rev0; idx++) {
			wlc_sslpnphy_write_table(pi, &dot11sslpnphytbl_info_rev0[idx]);
		}
		/* Restore the tables which were reclaimed */
		wlc_sslpnphy_restore_tbls(pi);
	} else {
		for (idx = 0; idx < dot11sslpnphytbl_info_sz_rev2; idx++)
			wlc_sslpnphy_write_table(pi, &dot11sslpnphytbl_info_rev2[idx]);

		if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_FEM_BT) {
			if (CHSPEC_IS2G(pi->radio_chanspec)) {
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) >= BCM94319WINDSOR_SSID) &&
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) <= BCM94319BHEMU3_SSID))
					wlc_load_bt_fem_combiner_sslpnphy(pi, TRUE);
				else
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
						sw_ctrl_tbl_rev02_shared, 64, 16, 0);
			} else {
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319WINDSOR_SSID) &&
					(GENERIC_PHY_INFO(pi)->boardrev <= 0x1101))
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
						sw_ctrl_tbl_rev02_shared_mlap_windsor_5g,
						64, 16, 0);
				else if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319BHEMU3_SSID)
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
						sw_ctrl_tbl_rev02_shared_mlap_emu3_5g, 32, 16, 0);
				else
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
						sw_ctrl_tbl_rev02_shared_mlap_5g, 32, 16, 0);
			}
		}
		/* load NINJA board trsw table */
		if (ninja_board_flag) {
			wlc_sslpnphy_write_table(pi, &sw_ctrl_tbl_info_ninja6l);
		}
	}

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		wlc_sslpnphy_load_tx_gain_table(pi, dot11sslpnphy_2GHz_gaintable_rev0);
		if ((GENERIC_PHY_INFO(pi)->boardflags & BFL_EXTLNA) && (SSLPNREV_LT(pi->pubpi.phy_rev, 2))) {
			if (x17_board_flag || N90_board_flag || ninja_board_flag ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID)) {
				tbl_val[0] = 0xee;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAINVALTBL_IDX,
					tbl_val, 1, 32, 65);
			}
		}
#ifndef BAND5G
	}
#else
	} else {
		tbl_val[0] = 0x00E38208;
		tbl_val[1] = 0x00E38208;
		wlc_sslpnphy_common_write_table(pi, 12, tbl_val, 2, 32, 0);
		tbl_val[0] = 0xfa;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAINVALTBL_IDX,
			tbl_val, 1, 32, 64);
		if (x17_board_flag) {
			tbl_val[0] = 0xf2;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAINVALTBL_IDX,
				tbl_val, 1, 32, 65);
		}

	}
#endif /* BAND5G */
	if ((SSLPNREV_GE(pi->pubpi.phy_rev, 2)) && (phybw40 == 1)) {
		for (idx = 0; idx < dot11lpphy_rx_gain_init_tbls_40Mhz_sz; idx++) {
			wlc_sslpnphy_write_table(pi, &dot11lpphy_rx_gain_init_tbls_40Mhz[idx]);
		}
	}

#ifdef BAND5G
	/* 4319 (REV4) 5G gaintable (ninja board) */
	if (CHSPEC_IS5G(pi->radio_chanspec) && SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {
		if (phybw40 == 1) {
			for (idx = 0; idx < dot11lpphy_rx_gain_extlna_tbls_A_40Mhz_sz; idx++) {
				wlc_sslpnphy_write_table(pi,
				     &dot11lpphy_rx_gain_extlna_tbls_A_40Mhz[idx]);
			}
		} else {
			for (idx = 0; idx < dot11lpphy_rx_gain_extlna_tbls_A_sz; idx++) {
				wlc_sslpnphy_write_table(pi,
				     &dot11lpphy_rx_gain_extlna_tbls_A[idx]);
			}
		}
	}
#endif

	wlc_sslpnphy_load_rfpower(pi);
	/* clear our PAPD Compensation table */
	wlc_sslpnphy_clear_papd_comptable(pi);
}

/* Reclaimable strings used by wlc_phy_txpwr_srom_read_sslpnphy */
static const char rstr_opo[] = "opo";
static const char rstr_mcs5gpo[] = "mcs5gpo";
#ifdef BAND5G
static const char rstr_tri5gl[] = "tri5gl";
static const char rstr_tri5g[] = "tri5g";
static const char rstr_tri5gh[] = "tri5gh";
static const char rstr_bxa5g[] = "bxa5g";
static const char rstr_rxpo5g[] = "rxpo5g";
static const char rstr_rssismf5g[] = "rssismf5g";
static const char rstr_rssismc5g[] = "rssismc5g";
static const char rstr_rssisav5g[] = "rssisav5g";
static const char rstr_pa1maxpwr[] = "pa1maxpwr";
static const char rstr_pa1b_d[] = "pa1b%d";
static const char rstr_pa1lob_d[] = "pa1lob%d";
static const char rstr_pa1hib_d[] = "pa1hib%d";
static const char rstr_ofdmapo[] = "ofdmapo";
static const char rstr_ofdmalpo[] = "ofdmalpo";
static const char rstr_ofdm5gpo[] = "ofdm5gpo";
static const char rstr_ofdm5glpo[] = "ofdm5glpo";
static const char rstr_ofdm5ghpo[] = "ofdm5ghpo";
static const char rstr_maxp5g[] = "maxp5g";
static const char rstr_maxp5gl[] = "maxp5gl";
static const char rstr_maxp5gh[] = "maxp5gh";
static const char rstr_mcs5gpo0[] = "mcs5gpo0";
static const char rstr_mcs5gpo4[] = "mcs5gpo4";
static const char rstr_mcs5glpo0[] = "mcs5glpo0";
static const char rstr_mcs5glpo4[] = "mcs5glpo4";
static const char rstr_mcs5ghpo0[] = "mcs5ghpo0";
static const char rstr_mcs5ghpo4[] = "mcs5ghpo4";
static const char rstr_bwduppo[] = "bwduppo";
#endif /* BAND5G */
static const char rstr_pa1lomaxpwr[] = "pa1lomaxpwr";
static const char rstr_ofdmahpo[] = "ofdmahpo";
static const char rstr_pa1himaxpwr[] = "pa1himaxpwr";
static const char rstr_tri2g[] = "tri2g";
static const char rstr_bxa2g[] = "bxa2g";
static const char rstr_rxpo2g[] = "rxpo2g";
static const char rstr_cckdigfilttype[] = "cckdigfilttype";
static const char rstr_ofdmdigfilttype[] = "ofdmdigfilttype";
static const char rstr_rxpo2gchnflg[] = "rxpo2gchnflg";
static const char rstr_forcepercal[] = "forcepercal";
static const char rstr_rssismf2g[] = "rssismf2g";
static const char rstr_rssismc2g[] = "rssismc2g";
static const char rstr_rssisav2g[] = "rssisav2g";
static const char rstr_rssismf2g_low0[] = "rssismf2g_low0";
static const char rstr_rssismc2g_low1[] = "rssismc2g_low1";
static const char rstr_rssisav2g_low2[] = "rssisav2g_low2";
static const char rstr_rssismf2g_hi0[] = "rssismf2g_hi0";
static const char rstr_rssismc2g_hi1[] = "rssismc2g_hi1";
static const char rstr_rssisav2g_hi2[] = "rssisav2g_hi2";
static const char rstr_pa0maxpwr[] = "pa0maxpwr";
static const char rstr_pa0b_d[] = "pa0b%d";
static const char rstr_cckpo[] = "cckpo";
static const char rstr_ofdmpo[] = "ofdmpo";
static const char rstr_mcs2gpo0[] = "mcs2gpo0";
static const char rstr_mcs2gpo1[] = "mcs2gpo1";
static const char rstr_mcs2gpo4[] = "mcs2gpo4";
static const char rstr_mcs2gpo5[] = "mcs2gpo5";
static const char rstr_5g_cga[] = "5g_cga";
static const char rstr_2g_cga[] = "2g_cga";
static const char rstr_tssi_min[] = "tssi_min";
static const char rstr_tssi_max[] = "tssi_max";

/* Read band specific data from the SROM */
static bool
wlc_sslpnphy_txpwr_srom_read(phy_info_t *pi)
{
	char varname[32];
	int8 txpwr;
	int i;
	uint16 pwr_offsets[2], pwr_offsets_40m[2];
#ifdef BAND5G
	uint32 offset_mcs;
	uint32 saved_offset_mcs;
	int8 saved_txpwr;
	uint32 offset;
	uint16 bwduppo = 0;
	uint8 opo = 0;
#endif
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* Fab specific Tuning */
	if (!(si_otp_fabid(GENERIC_PHY_INFO(pi)->sih, &sslpnphy_specific->sslpnphy_fabid_otp, TRUE) == BCME_OK))
	{
		WL_ERROR(("Reading fabid from otp failed.\n"));
	}

	/* Optional TSSI limits */
	if (PHY_GETVAR(pi, rstr_tssi_max))
		sslpnphy_specific->sslpnphy_tssi_max_pwr_nvram = (int8)PHY_GETINTVAR(pi, rstr_tssi_max);
	else
		sslpnphy_specific->sslpnphy_tssi_max_pwr_nvram = 127;
	if (PHY_GETVAR(pi, rstr_tssi_min))
		sslpnphy_specific->sslpnphy_tssi_min_pwr_nvram = (int8)PHY_GETINTVAR(pi, rstr_tssi_min);
	else
		sslpnphy_specific->sslpnphy_tssi_min_pwr_nvram = -128;

	/* Band specific setup */
#ifdef BAND5G
		opo = (uint8)PHY_GETINTVAR(pi, rstr_opo);
		offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5gpo);
		saved_offset_mcs = offset_mcs;


		/* TR switch isolation */
		sslpnphy_specific->sslpnphy_tr_isolation_low = (uint8)PHY_GETINTVAR(pi, rstr_tri5gl);
		sslpnphy_specific->sslpnphy_tr_isolation_mid = (uint8)PHY_GETINTVAR(pi, rstr_tri5g);
		sslpnphy_specific->sslpnphy_tr_isolation_hi = (uint8)PHY_GETINTVAR(pi, rstr_tri5gh);
		/* Board switch architecture */
		sslpnphy_specific->sslpnphy_bx_arch = (uint8)PHY_GETINTVAR(pi, rstr_bxa5g);

		/* Input power offset */
		sslpnphy_specific->sslpnphy_rx_power_offset_5g = PHY_GETINTVAR_ARRAY(rstr_rxpo5g,
			IOVT_UINT8, sslpnphy_specific->sslpnphy_fabid_otp);

		/* RSSI */
		sslpnphy_specific->sslpnphy_rssi_5g_vf = PHY_GETINTVAR_ARRAY(rstr_rssismf5g,
			IOVT_UINT8, sslpnphy_specific->sslpnphy_fabid_otp);
		sslpnphy_specific->sslpnphy_rssi_5g_vc = PHY_GETINTVAR_ARRAY(rstr_rssismc5g,
			IOVT_UINT8, sslpnphy_specific->sslpnphy_fabid_otp);
		sslpnphy_specific->sslpnphy_rssi_5g_gs = PHY_GETINTVAR_ARRAY(rstr_rssisav5g,
			IOVT_UINT8, sslpnphy_specific->sslpnphy_fabid_otp);
		/* Max tx power */
		txpwr = (int8)PHY_GETINTVAR(pi, rstr_pa1maxpwr);
		saved_txpwr = txpwr;

		/* PA coeffs */
		for (i = 0; i < 3; i++) {
			snprintf(varname, sizeof(varname), rstr_pa1b_d, i);
			pi->txpa_5g_mid[i] = (int16)PHY_GETINTVAR_ARRAY(varname,
				IOVT_UINT16, sslpnphy_specific->sslpnphy_fabid_otp);
		}

		/* Low channels */
		for (i = 0; i < 3; i++) {
			snprintf(varname, sizeof(varname), rstr_pa1lob_d, i);
			pi->txpa_5g_low[i] = (int16)PHY_GETINTVAR_ARRAY(varname, IOVT_UINT16,
				sslpnphy_specific->sslpnphy_fabid_otp);
		}

		/* High channels */
		for (i = 0; i < 3; i++) {
			snprintf(varname, sizeof(varname), rstr_pa1hib_d, i);
			pi->txpa_5g_hi[i] = (int16)PHY_GETINTVAR_ARRAY(varname, IOVT_UINT16,
				sslpnphy_specific->sslpnphy_fabid_otp);
		}

		/* The *po variables introduce a seperate max tx power for reach rate.
		 * Each per-rate txpower is specified as offset from the maxtxpower
		 * from the maxtxpwr in that band (lo,mid,hi).
		 * The offsets in the variables is stored in half dbm units to save
		 * srom space, which need to be doubled to convert to quarter dbm units
		 * before using.
		 * For small 1Kbit sroms of PCI/PCIe cards, the getintav will always return 0;
		 * For bigger sroms or NVRAM or CIS, they are present
		 */

		/* Mid band channels */
		/* Extract 8 OFDM rates for mid channels */
		offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdmapo);

		/* MCS32 power offset for each of the 5G sub-bands */
		if (PHY_GETVAR(pi, rstr_bwduppo)) {
			bwduppo = (uint16)PHY_GETINTVAR(pi, rstr_bwduppo);
		}

		/* Override the maxpwr and offset for 5G mid-band if the SROM
		 * entry exists, otherwise use the default txpwr & offset setting
		 * from above
		 */
		if (PHY_GETVAR(pi, rstr_maxp5g)) {
			txpwr = (int8)PHY_GETINTVAR(pi, rstr_maxp5g);
		}
		if (PHY_GETVAR(pi, rstr_ofdm5gpo)) {
			offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdm5gpo);
		}
		if (PHY_GETVAR(pi, rstr_mcs5gpo0)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5gpo0);
		}

		pi->tx_srom_max_5g_mid = txpwr;

		for (i = TXP_FIRST_OFDM; i <= TXP_LAST_OFDM; i++) {
			pi->tx_srom_max_rate_5g_mid[i] = txpwr - ((offset & 0xf) * 2);
			offset >>= 4;
		}
		for (i = TXP_FIRST_MCS_20; i <= TXP_LAST_MCS_SISO_20;  i++) {
			pi->tx_srom_max_rate_5g_mid[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* 5GHz 40MHz MCS rates */
		offset_mcs = saved_offset_mcs;
		if (PHY_GETVAR(pi, rstr_mcs5gpo4)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5gpo4);
		}

		for (i = TXP_FIRST_MCS_40; i <= TXP_LAST_MCS_SISO_40;  i++) {
			pi->tx_srom_max_rate_5g_mid[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* MCS32 5G mid-band */
		pi->tx_srom_max_rate_5g_mid[TXP_LAST_MCS_40] = txpwr - (((bwduppo >> 4) & 0xf) * 2);

		/* Extract 8 OFDM rates for low channels */
		offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdmalpo);
		offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5gpo);

		/* Override the maxpwr and offset for 5G low-band if the SROM
		 * entry exists, otherwise use the default txpwr & offset setting
		 * from above
		 */
		txpwr = saved_txpwr;
		if (PHY_GETVAR(pi, rstr_maxp5gl)) {
			txpwr = (int8)PHY_GETINTVAR(pi, rstr_maxp5gl);
		}
		if (PHY_GETVAR(pi, rstr_ofdm5glpo)) {
			offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdm5glpo);
		}
		if (PHY_GETVAR(pi, rstr_mcs5glpo0)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5glpo0);
		}

		for (i = TXP_FIRST_OFDM; i <= TXP_LAST_OFDM; i++) {
			pi->tx_srom_max_rate_5g_low[i] = txpwr - ((offset & 0xf) * 2);
			offset >>= 4;
		}
		for (i = TXP_FIRST_MCS_20; i <= TXP_LAST_MCS_SISO_20;  i++) {
			pi->tx_srom_max_rate_5g_low[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* 5GHz 40MHz MCS rates */
		offset_mcs = saved_offset_mcs;
		if (PHY_GETVAR(pi, rstr_mcs5glpo4)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5glpo4);
		}

		for (i = TXP_FIRST_MCS_40; i <= TXP_LAST_MCS_SISO_40;  i++) {
			pi->tx_srom_max_rate_5g_low[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* MCS32 5G low-band */
		pi->tx_srom_max_rate_5g_low[TXP_LAST_MCS_40] = txpwr - (((bwduppo >> 8) & 0xf) * 2);

		/* Extract 8 OFDM rates for hi channels */
		offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdmahpo);
		offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5gpo);

		/* Override the maxpwr and offset for 5G high-band if the SROM
		 * entry exists, otherwise use the default txpwr & offset setting
		 * from above
		 */
		txpwr = saved_txpwr;
		if (PHY_GETVAR(pi, rstr_maxp5gh)) {
			txpwr = (int8)PHY_GETINTVAR(pi, rstr_maxp5gh);
		}
		if (PHY_GETVAR(pi, rstr_ofdm5ghpo)) {
			offset = (uint32)PHY_GETINTVAR(pi, rstr_ofdm5ghpo);
		}
		if (PHY_GETVAR(pi, rstr_mcs5ghpo0)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5ghpo0);
		}


		for (i = TXP_FIRST_OFDM; i <= TXP_LAST_OFDM; i++) {
			pi->tx_srom_max_rate_5g_hi[i] = txpwr - ((offset & 0xf) * 2);
			offset >>= 4;
		}
		for (i = TXP_FIRST_MCS_20; i <= TXP_LAST_MCS_SISO_20;  i++) {
			pi->tx_srom_max_rate_5g_hi[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* 5GHz 40MHz MCS rates */
		offset_mcs = saved_offset_mcs;
		if (PHY_GETVAR(pi, rstr_mcs5ghpo4)) {
			offset_mcs = (uint32)PHY_GETINTVAR(pi, rstr_mcs5ghpo4);
		}

		for (i = TXP_FIRST_MCS_40; i <= TXP_LAST_MCS_SISO_40;  i++) {
			pi->tx_srom_max_rate_5g_hi[i] = txpwr -
				((offset_mcs & 0xf) * 2);
			offset_mcs >>= 4;
		}

		/* MCS32 5G high-band */
		pi->tx_srom_max_rate_5g_hi[TXP_LAST_MCS_40] = txpwr - (((bwduppo >> 12) & 0xf) * 2);

		for (i = 0; i < 24; i++) {
			sslpnphy_specific->sslpnphy_cga_5g[i] = (int8)PHY_GETINTVAR_ARRAY(rstr_5g_cga,
				IOVT_UINT16, i);
		}
#endif /* BAND5G */

		uint16 cckpo;
		uint32 offset_ofdm;
		/* TR switch isolation */
		sslpnphy_specific->sslpnphy_tr_isolation_mid = (uint8)PHY_GETINTVAR(pi, rstr_tri2g);

		/* Board switch architecture */
		sslpnphy_specific->sslpnphy_bx_arch = (uint8)PHY_GETINTVAR(pi, rstr_bxa2g);

		/* Input power offset */
		sslpnphy_specific->sslpnphy_rx_power_offset = PHY_GETINTVAR_ARRAY(rstr_rxpo2g, IOVT_UINT8,
			sslpnphy_specific->sslpnphy_fabid_otp);

		/* Sslpnphy  filter select */
		sslpnphy_specific->sslpnphy_cck_filt_sel = (uint8)PHY_GETINTVAR(pi, rstr_cckdigfilttype);
		sslpnphy_specific->sslpnphy_ofdm_filt_sel = (uint8)PHY_GETINTVAR(pi, rstr_ofdmdigfilttype);

		/* Channel based selection for rxpo2g */
		sslpnphy_specific->sslpnphy_rxpo2gchnflg = (uint16)PHY_GETINTVAR(pi, rstr_rxpo2gchnflg);

		sslpnphy_specific->sslpnphy_fabid = (uint8)PHY_GETINTVAR(pi, "fabid");

		/* force periodic cal */
		sslpnphy_specific->sslpnphy_force_percal = (uint8)PHY_GETINTVAR(pi, rstr_forcepercal);

		/* RSSI */
		sslpnphy_specific->sslpnphy_rssi_vf = PHY_GETINTVAR_ARRAY(rstr_rssismf2g, IOVT_UINT8,
			sslpnphy_specific->sslpnphy_fabid_otp);
		sslpnphy_specific->sslpnphy_rssi_vc = PHY_GETINTVAR_ARRAY(rstr_rssismc2g, IOVT_UINT8,
			sslpnphy_specific->sslpnphy_fabid_otp);
		sslpnphy_specific->sslpnphy_rssi_gs = PHY_GETINTVAR_ARRAY(rstr_rssisav2g, IOVT_UINT8,
			sslpnphy_specific->sslpnphy_fabid_otp);
		sslpnphy_specific->sslpnphy_rssi_vf_lowtemp = sslpnphy_specific->sslpnphy_rssi_vf;
		sslpnphy_specific->sslpnphy_rssi_vc_lowtemp = sslpnphy_specific->sslpnphy_rssi_vc;
		sslpnphy_specific->sslpnphy_rssi_gs_lowtemp = sslpnphy_specific->sslpnphy_rssi_gs;

		sslpnphy_specific->sslpnphy_rssi_vf_hightemp = sslpnphy_specific->sslpnphy_rssi_vf;
		sslpnphy_specific->sslpnphy_rssi_vc_hightemp = sslpnphy_specific->sslpnphy_rssi_vc;
		sslpnphy_specific->sslpnphy_rssi_gs_hightemp = sslpnphy_specific->sslpnphy_rssi_gs;

		/* Max tx power */
		txpwr = (int8)PHY_GETINTVAR(pi, rstr_pa0maxpwr);
		/* Make sure of backward compatibility for bellatrix OLD NVRAM's */
		/* add dB to compensate for 1.5dBbackoff (that will be done) in in older boards */
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2) &&
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319WLUSBN4L_SSID)) {
				if ((GENERIC_PHY_INFO(pi)->boardrev) <= 0x1512)
					txpwr = txpwr + 6;
		}
		pi->tx_srom_max_2g = txpwr;

		/* PA coeffs */
		for (i = 0; i < PWRTBL_NUM_COEFF; i++) {
			snprintf(varname, sizeof(varname), rstr_pa0b_d, i);
			pi->txpa_2g[i] = (int16)PHY_GETINTVAR_ARRAY(varname, IOVT_UINT16,
				sslpnphy_specific->sslpnphy_fabid_otp);
		}
		if ((GENERIC_PHY_INFO(pi)->boardrev == 0x1307) || (GENERIC_PHY_INFO(pi)->boardrev == 0x1306)) {
			pi->txpa_2g[0] = 5779;
			pi->txpa_2g[1] = 64098;
			pi->txpa_2g[2] = 65140;
		}
		for (i = 0; i < PWRTBL_NUM_COEFF; i++) {
			pi->txpa_2g_low_temp[i] = pi->txpa_2g[i];
			pi->txpa_2g_high_temp[i] = pi->txpa_2g[i];
		}
		cckpo = (uint16)PHY_GETINTVAR(pi, rstr_cckpo);
		if (cckpo) {
			uint max_pwr_chan = txpwr;

			/* Extract offsets for 4 CCK rates. Remember to convert from
			* .5 to .25 dbm units
			*/
			for (i = TXP_FIRST_CCK; i <= TXP_LAST_CCK; i++) {
				pi->tx_srom_max_rate_2g[i] = max_pwr_chan -
					((cckpo & 0xf) * 2);
				cckpo >>= 4;
			}

		} else {
			uint8 opo = 0;

			opo = (uint8)PHY_GETINTVAR(pi, rstr_opo);

			/* Populate max power array for CCK rates */
			for (i = TXP_FIRST_CCK; i <= TXP_LAST_CCK; i++) {
				pi->tx_srom_max_rate_2g[i] = txpwr;
			}
		}
		/* Extract offsets for 8 OFDM rates */
		offset_ofdm = (uint32)PHY_GETINTVAR(pi, rstr_ofdmpo);
		for (i = TXP_FIRST_OFDM; i <= TXP_LAST_OFDM; i++) {
			pi->tx_srom_max_rate_2g[i] = txpwr -
				((offset_ofdm & 0xf) * 2);
			offset_ofdm >>= 4;
		}
		/* Now MCS2GPO is only 2 Bytes, ajust accordingly */
		pwr_offsets[0] = (uint16)PHY_GETINTVAR(pi, rstr_mcs2gpo0);
		pwr_offsets[1] = (uint16)PHY_GETINTVAR(pi, rstr_mcs2gpo1);
		wlc_sslpnphy_txpwr_srom_convert(pi->tx_srom_max_rate_2g, pwr_offsets, txpwr,
			TXP_FIRST_MCS_20, TXP_LAST_MCS_SISO_20);

		pwr_offsets_40m[0] = (uint16)PHY_GETINTVAR(pi, rstr_mcs2gpo4);
		pwr_offsets_40m[1] = (uint16)PHY_GETINTVAR(pi, rstr_mcs2gpo5);

		/* If 40Mhz srom entries not available use 20Mhz mcs power offsets */
		if (pwr_offsets_40m[0] == 0)
			wlc_sslpnphy_txpwr_srom_convert(pi->tx_srom_max_rate_2g, pwr_offsets, txpwr,
				TXP_FIRST_MCS_40, TXP_LAST_MCS_SISO_40);
		else
			wlc_sslpnphy_txpwr_srom_convert(pi->tx_srom_max_rate_2g, pwr_offsets_40m,
				txpwr, TXP_FIRST_MCS_40, TXP_LAST_MCS_SISO_40);

		/* for MCS32 select power same as mcs7 40Mhz rate */
		pi->tx_srom_max_rate_2g[TXP_LAST_MCS_40] =
			pi->tx_srom_max_rate_2g[TXP_LAST_MCS_SISO_40];

		for (i = 0; i < 14; i++) {
			sslpnphy_specific->sslpnphy_cga_2g[i] = (int8)PHY_GETINTVAR_ARRAY(rstr_2g_cga,
				IOVT_UINT16, i);
		}
	return TRUE;
}

static void
WLBANDINITFN(wlc_sslpnphy_rev0_baseband_init)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		or_phy_reg(pi, SSLPNPHY_lpphyCtrl, SSLPNPHY_lpphyCtrl_muxGmode_MASK);
		or_phy_reg(pi, SSLPNPHY_crsgainCtrl, SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_MASK);
	} else {
		and_phy_reg(pi, SSLPNPHY_lpphyCtrl, (uint16)~SSLPNPHY_lpphyCtrl_muxGmode_MASK);
		and_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			(uint16)~SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_MASK);
	}

	mod_phy_reg(pi, SSLPNPHY_lpphyCtrl,
		SSLPNPHY_lpphyCtrl_txfiltSelect_MASK,
		1 << SSLPNPHY_lpphyCtrl_txfiltSelect_SHIFT);

	/* Enable DAC/ADC and disable rf overrides */
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2))
		write_phy_reg(pi, SSLPNPHY_AfeDACCtrl, 0x54);
	else
		write_phy_reg(pi, SSLPNPHY_AfeDACCtrl, 0x50);

	write_phy_reg(pi, SSLPNPHY_AfeCtrl, 0x8800);
	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvr, 0x0000);
	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal, 0x0000);
	write_phy_reg(pi, SSLPNPHY_RFinputOverride, 0x0000);
	write_phy_reg(pi, SSLPNPHY_RFOverride0, 0x0000);
	write_phy_reg(pi, SSLPNPHY_rfoverride2, 0x0000);
	write_phy_reg(pi, SSLPNPHY_rfoverride3, 0x0000);
	write_phy_reg(pi, SSLPNPHY_swctrlOvr, 0x0000);

	mod_phy_reg(pi, SSLPNPHY_RxIqCoeffCtrl,
		SSLPNPHY_RxIqCoeffCtrl_RxIqCrsCoeffOverRide_MASK,
		1 << SSLPNPHY_RxIqCoeffCtrl_RxIqCrsCoeffOverRide_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_RxIqCoeffCtrl,
		SSLPNPHY_RxIqCoeffCtrl_RxIqCrsCoeffOverRide11b_MASK,
		1 << SSLPNPHY_RxIqCoeffCtrl_RxIqCrsCoeffOverRide11b_SHIFT);

	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);

	/* RSSI settings */
	write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl0, 0xA954);

	write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1,
		((uint16)sslpnphy_specific->sslpnphy_rssi_vf_lowtemp << 0) | /* selmid_rssi: RSSI Vmid fine */
		((uint16)sslpnphy_specific->sslpnphy_rssi_vc_lowtemp << 4) | /* selmid_rssi: RSSI Vmid coarse */
		(0x00 << 8) | /* selmid_rssi: default value from AMS */
		((uint16)sslpnphy_specific->sslpnphy_rssi_gs_lowtemp << 10) | /* selav_rssi: RSSI gain select */
		(0x01 << 13)); /* slpinv_rssi */

}
static void
wlc_sslpnphy_common_read_table(phy_info_t *pi, uint32 tbl_id,
	CONST void *tbl_ptr, uint32 tbl_len, uint32 tbl_width, uint32 tbl_offset);


int8
wlc_sslpnphy_get_rx_pwr_offset(phy_info_t *pi)
{
	int16 temp;
	
	if (!IS40MHZ(pi)) {
		temp = (int16)(read_phy_reg(pi, SSLPNPHY_InputPowerDB)
					& SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK);
	} else {
	        temp = (int16)(read_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40)
	                & SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK);
	}

	if (temp > 127)
		temp -= 256;

	return (int8)temp;
}

void
wlc_sslpnphy_rx_offset_init(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_input_pwr_offset_db = wlc_sslpnphy_get_rx_pwr_offset(pi);
}


static void
wlc_sslpnphy_agc_temp_init(phy_info_t *pi)
{
	int16 temp;
	uint32 tableBuffer[2];
	uint8 phybw40 = IS40MHZ(pi);
	int8 delta_T_change;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* reference ofdm gain index table offset */
	temp = (int16) read_phy_reg(pi, SSLPNPHY_gainidxoffset);
	sslpnphy_specific->sslpnphy_ofdmgainidxtableoffset =
	    (temp & SSLPNPHY_gainidxoffset_ofdmgainidxtableoffset_MASK) >>
	    SSLPNPHY_gainidxoffset_ofdmgainidxtableoffset_SHIFT;

	if (sslpnphy_specific->sslpnphy_ofdmgainidxtableoffset > 127) sslpnphy_specific->sslpnphy_ofdmgainidxtableoffset -= 256;

	/* reference dsss gain index table offset */
	sslpnphy_specific->sslpnphy_dsssgainidxtableoffset =
	    (temp & SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_MASK) >>
	    SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_SHIFT;

	if (sslpnphy_specific->sslpnphy_dsssgainidxtableoffset > 127) sslpnphy_specific->sslpnphy_dsssgainidxtableoffset -= 256;

	wlc_sslpnphy_common_read_table(pi, 17, tableBuffer, 2, 32, 64);

	/* reference value of gain_val_tbl at index 64 */
	if (tableBuffer[0] > 63) tableBuffer[0] -= 128;
	sslpnphy_specific->sslpnphy_tr_R_gain_val = tableBuffer[0];

	/* reference value of gain_val_tbl at index 65 */
	if (tableBuffer[1] > 63) tableBuffer[1] -= 128;
	sslpnphy_specific->sslpnphy_tr_T_gain_val = tableBuffer[1];
	if (phybw40 == 0) {
	        sslpnphy_specific->sslpnphy_Med_Low_Gain_db = (read_phy_reg(pi, SSLPNPHY_LowGainDB)
	                & SSLPNPHY_LowGainDB_MedLowGainDB_MASK)
	                >> SSLPNPHY_LowGainDB_MedLowGainDB_SHIFT;

	        sslpnphy_specific->sslpnphy_Very_Low_Gain_db = (read_phy_reg(pi, SSLPNPHY_VeryLowGainDB)
	                & SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK)
	                >> SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT;
	} else {
	        sslpnphy_specific->sslpnphy_Med_Low_Gain_db = (read_phy_reg(pi, SSLPNPHY_Rev2_LowGainDB_40)
	                & SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_MASK)
	                >> SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_SHIFT;

	        sslpnphy_specific->sslpnphy_Very_Low_Gain_db = (read_phy_reg(pi, SSLPNPHY_Rev2_VeryLowGainDB_40)
	                & SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_MASK)
	                >> SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_SHIFT;
	}

	wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
		tableBuffer, 2, 32, 28);

	sslpnphy_specific->sslpnphy_gain_idx_14_lowword = tableBuffer[0];
	sslpnphy_specific->sslpnphy_gain_idx_14_hiword = tableBuffer[1];
	/* tr isolation adjustments */
	if (sslpnphy_specific->sslpnphy_tr_isolation_mid && (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94319WLUSBN4L_SSID)) {

		tableBuffer[0] = sslpnphy_specific->sslpnphy_tr_R_gain_val;
		tableBuffer[1] = sslpnphy_specific->sslpnphy_tr_isolation_mid;
		if (tableBuffer[1] > 63)
			tableBuffer[1] -= 128;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAINVALTBL_IDX,
			tableBuffer, 2, 32, 64);

		delta_T_change = sslpnphy_specific->sslpnphy_tr_T_gain_val - tableBuffer[1];

		sslpnphy_specific->sslpnphy_Very_Low_Gain_db += delta_T_change;
		sslpnphy_specific->sslpnphy_tr_T_gain_val = tableBuffer[1];

		if (phybw40) {
			mod_phy_reg(pi, SSLPNPHY_Rev2_VeryLowGainDB_40,
				SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_MASK,
				(sslpnphy_specific->sslpnphy_Very_Low_Gain_db <<
				 SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_SHIFT));
		} else {
			mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
				SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
				(sslpnphy_specific->sslpnphy_Very_Low_Gain_db  <<
				 SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT));
		}

	}
	/* Added To Increase The 1Mbps Sense for Temps @Around */
	/* -15C Temp With CmRxAciGainTbl */
	sslpnphy_specific->sslpnphy_gain_idx_27_lowword = 0xf1e64d96;
	sslpnphy_specific->sslpnphy_gain_idx_27_hiword  = 0xf1e60018;

	/* Storing Input rx offset */
	wlc_sslpnphy_rx_offset_init(pi);

	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);
}

static void
wlc_sslpnphy_bu_tweaks(phy_info_t *pi)
{

	uint8 phybw40 = IS40MHZ(pi);
	int8 aa;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (!NORADIO_ENAB(pi->pubpi)) {
	/* Sequence of register writes. Can be optimized. */

	/* CRS Parameters tuning */
	mod_phy_reg(pi, SSLPNPHY_gaindirectMismatch,
		SSLPNPHY_gaindirectMismatch_medGainGmShftVal_MASK,
		3 << SSLPNPHY_gaindirectMismatch_medGainGmShftVal_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
		SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK |
		SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_MASK,
		30 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT |
		20 << SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_HiGainDB,
		SSLPNPHY_HiGainDB_HiGainDB_MASK |
		SSLPNPHY_HiGainDB_MedHiGainDB_MASK,
		70 << SSLPNPHY_HiGainDB_HiGainDB_SHIFT |
		45 << SSLPNPHY_HiGainDB_MedHiGainDB_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
		SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK |
		SSLPNPHY_VeryLowGainDB_NominalPwrDB_MASK,
		6 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT |
		95 << SSLPNPHY_VeryLowGainDB_NominalPwrDB_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
		SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_MASK |
		SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
		1 << SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_SHIFT |
		25 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs2,
		SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_MASK,
		12 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_gainMismatch,
		SSLPNPHY_gainMismatch_GainMismatchHigain_MASK,
		10 << SSLPNPHY_gainMismatch_GainMismatchHigain_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
		SSLPNPHY_PwrThresh1_LargeGainMismatchThresh_MASK,
		9 << SSLPNPHY_PwrThresh1_LargeGainMismatchThresh_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
		SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
		3 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_crsMiscCtrl2,
		SSLPNPHY_crsMiscCtrl2_eghtSmplFstPwrLogicEn_MASK,
		0 << SSLPNPHY_crsMiscCtrl2_eghtSmplFstPwrLogicEn_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_crsTimingCtrl,
		SSLPNPHY_crsTimingCtrl_gainThrsh4Timing_MASK |
		SSLPNPHY_crsTimingCtrl_gainThrsh4MF_MASK,
		0 << SSLPNPHY_crsTimingCtrl_gainThrsh4Timing_SHIFT |
		73 << SSLPNPHY_crsTimingCtrl_gainThrsh4MF_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh1,
		SSLPNPHY_ofdmSyncThresh1_ofdmSyncThresh2_MASK,
		2 << SSLPNPHY_ofdmSyncThresh1_ofdmSyncThresh2_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_SyncPeakCnt,
		SSLPNPHY_SyncPeakCnt_MaxPeakCntM1_MASK,
		7 << SSLPNPHY_SyncPeakCnt_MaxPeakCntM1_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_DSSSConfirmCnt,
		SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGain_MASK,
		3 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGain_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
		SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
		255 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_MinPwrLevel,
		SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_MASK,
		162 << SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_LowGainDB,
		SSLPNPHY_LowGainDB_MedLowGainDB_MASK,
		29 << SSLPNPHY_LowGainDB_MedLowGainDB_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_gainidxoffset,
		SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_MASK,
		244 << SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
		SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
		10 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_crsMiscCtrl0,
		SSLPNPHY_crsMiscCtrl0_usePreFiltPwr_MASK,
		0 << SSLPNPHY_crsMiscCtrl0_usePreFiltPwr_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_ofdmPwrThresh1,
		SSLPNPHY_ofdmPwrThresh1_ofdmPwrThresh3_MASK,
		48 << SSLPNPHY_ofdmPwrThresh1_ofdmPwrThresh3_SHIFT);

	write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6033);
	write_phy_reg(pi, SSLPNPHY_ClipThresh, 108);
	write_phy_reg(pi, SSLPNPHY_SgiprgReg, 3);

	if (phybw40 == 1)
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrl,
			SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_MASK,
			0 << SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);
	else
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrl,
			SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_MASK,
			1 << SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);

	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrl,
			SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_MASK,
			0 << SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
			SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_MASK,
			0 << SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_SHIFT);
		/* WAR to the Higher A-band Channels Rxper Hump @-60 to -70dBm Signal Levels
		   From Aniritsu8860C Tester
		*/
		mod_phy_reg(pi, SSLPNPHY_crsMiscCtrl0,
			SSLPNPHY_crsMiscCtrl0_cfoCalcEn_MASK,
			0 << SSLPNPHY_crsMiscCtrl0_cfoCalcEn_SHIFT);
#ifdef PHYHAL
		pi->aa2g = (uint8)PHY_GETINTVAR(pi, "aa2g");
		aa = (int8)ANT_AVAIL(pi->aa2g);
#else
		aa = (int8)ANT_AVAIL(pi->sh->ant_avail_aa2g);
#endif /* PHYHAL */
	} else {
#ifdef PHYHAL
		pi->aa5g = (uint8)PHY_GETINTVAR(pi, "aa5g");
		aa = (int8)ANT_AVAIL(pi->aa5g);
#else
		aa = (int8)ANT_AVAIL(pi->sh->ant_avail_aa5g);
#endif /* PHYHAL */
		/* Dflt Value */
		mod_phy_reg(pi, SSLPNPHY_crsMiscCtrl0,
			SSLPNPHY_crsMiscCtrl0_cfoCalcEn_MASK,
			1 << SSLPNPHY_crsMiscCtrl0_cfoCalcEn_SHIFT);
	}
	if (aa > 1) {

		/* Antenna diveristy related changes */
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_wlpriogainChangeEn_MASK |
			SSLPNPHY_crsgainCtrl_preferredAntEn_MASK,
			0 << SSLPNPHY_crsgainCtrl_wlpriogainChangeEn_SHIFT |
			0 << SSLPNPHY_crsgainCtrl_preferredAntEn_SHIFT);
		write_phy_reg(pi, SSLPNPHY_lnaputable, 0x5555);
		mod_phy_reg(pi, SSLPNPHY_radioCtrl,
			SSLPNPHY_radioCtrl_auxgaintblEn_MASK,
			0 << SSLPNPHY_radioCtrl_auxgaintblEn_SHIFT);
		write_phy_reg(pi, SSLPNPHY_slnanoisetblreg0, 0x4210);
		write_phy_reg(pi, SSLPNPHY_slnanoisetblreg1, 0x4210);
		write_phy_reg(pi, SSLPNPHY_slnanoisetblreg2, 0x0270);
		/* mod_phy_reg(pi, SSLPNPHY_PwrThresh1, */
		/*	SSLPNPHY_PwrThresh1_LoPwrMismatchThresh_MASK, */
		/*	20 << SSLPNPHY_PwrThresh1_LoPwrMismatchThresh_SHIFT); */
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
				SSLPNPHY_Rev2_crsgainCtrl_40_wlpriogainChangeEn_MASK |
				SSLPNPHY_Rev2_crsgainCtrl_40_preferredAntEn_MASK,
				0 << SSLPNPHY_Rev2_crsgainCtrl_40_wlpriogainChangeEn_SHIFT |
				0 << SSLPNPHY_Rev2_crsgainCtrl_40_preferredAntEn_SHIFT);
		}
		/* enable Diversity for  Dual Antenna Boards */
		if (aa > 2) {
			if (phybw40)
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
					SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_MASK,
				0x01 << SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_SHIFT);
			else
				mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
					SSLPNPHY_crsgainCtrl_DiversityChkEnable_MASK,
				0x01 << SSLPNPHY_crsgainCtrl_DiversityChkEnable_SHIFT);
		}
	}
	if (IS_OLYMPIC(pi)) {
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID)
			mod_phy_reg(pi, SSLPNPHY_BphyControl3,
				SSLPNPHY_BphyControl3_bphyScale_MASK,
				0x6 << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
		else
			mod_phy_reg(pi, SSLPNPHY_BphyControl3,
				SSLPNPHY_BphyControl3_bphyScale_MASK,
				0x7 << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
	} else
		mod_phy_reg(pi, SSLPNPHY_BphyControl3,
			SSLPNPHY_BphyControl3_bphyScale_MASK,
			0xc << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		write_phy_reg(pi, SSLPNPHY_ClipThresh, 72);
		mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
			SSLPNPHY_PwrThresh1_LargeGainMismatchThresh_MASK,
			4 << SSLPNPHY_PwrThresh1_LargeGainMismatchThresh_SHIFT);
		if (SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {
			if (phybw40)
			{
				mod_phy_reg(pi, SSLPNPHY_BphyControl3,
					SSLPNPHY_BphyControl3_bphyScale_MASK,
					0x8 << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
			} else {
				mod_phy_reg(pi, SSLPNPHY_BphyControl3,
					SSLPNPHY_BphyControl3_bphyScale_MASK,
					0xa << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
			}
		}
		else {
			mod_phy_reg(pi, SSLPNPHY_BphyControl3,
				SSLPNPHY_BphyControl3_bphyScale_MASK,
				0x13 << SSLPNPHY_BphyControl3_bphyScale_SHIFT);
		}
		mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
			SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_MASK,
			18 << SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_MinPwrLevel,
			SSLPNPHY_MinPwrLevel_dsssMinPwrLevel_MASK,
			158 << SSLPNPHY_MinPwrLevel_dsssMinPwrLevel_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_phycrsctrl_MASK,
			11 << SSLPNPHY_crsgainCtrl_phycrsctrl_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_SyncPeakCnt,
			SSLPNPHY_SyncPeakCnt_MaxPeakCntM1_MASK,
			7 << SSLPNPHY_SyncPeakCnt_MaxPeakCntM1_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs2,
			SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_MASK,
			11 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
			SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
			20 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
			SSLPNPHY_PwrThresh1_LoPwrMismatchThresh_MASK,
			18 << SSLPNPHY_PwrThresh1_LoPwrMismatchThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
			SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
			0 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_lnsrOfParam1,
			SSLPNPHY_lnsrOfParam1_ofdmSyncConfirmAdjst_MASK,
			5 << SSLPNPHY_lnsrOfParam1_ofdmSyncConfirmAdjst_SHIFT);
		write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6366);

		if (phybw40 == 1) {
			mod_phy_reg(pi, SSLPNPHY_Rev2_radioCtrl_40mhz,
				SSLPNPHY_Rev2_radioCtrl_40mhz_round_control_40mhz_MASK |
				SSLPNPHY_Rev2_radioCtrl_40mhz_gainReqTrAttOnEnByCrs40_MASK,
				((0 << SSLPNPHY_Rev2_radioCtrl_40mhz_round_control_40mhz_SHIFT) |
				(0 <<
				SSLPNPHY_Rev2_radioCtrl_40mhz_gainReqTrAttOnEnByCrs40_SHIFT)));
			mod_phy_reg(pi, SSLPNPHY_Rev2_gaindirectMismatch_40,
				SSLPNPHY_Rev2_gaindirectMismatch_40_medGainGmShftVal_MASK,
				3 << SSLPNPHY_Rev2_gaindirectMismatch_40_medGainGmShftVal_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_ClipCtrThresh_40,
				SSLPNPHY_Rev2_ClipCtrThresh_40_clipCtrThreshLoGain_MASK,
				36 << SSLPNPHY_Rev2_ClipCtrThresh_40_clipCtrThreshLoGain_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_radioTRCtrl,
				SSLPNPHY_Rev2_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT,
				0  << SSLPNPHY_Rev2_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40,
				SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_MASK,
				70  << SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_VeryLowGainDB_40,
				SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_MASK,
				9  << SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_gainMismatch_40,
				SSLPNPHY_Rev2_gainMismatch_40_GainMismatchHigain_MASK,
				10  << SSLPNPHY_Rev2_gainMismatch_40_GainMismatchHigain_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_crsMiscCtrl2_40,
				SSLPNPHY_Rev2_crsMiscCtrl2_40_eghtSmplFstPwrLogicEn_MASK,
				0  << SSLPNPHY_Rev2_crsMiscCtrl2_40_eghtSmplFstPwrLogicEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_PwrThresh1_40,
				SSLPNPHY_Rev2_PwrThresh1_40_LargeGainMismatchThresh_MASK,
				9  << SSLPNPHY_Rev2_PwrThresh1_40_LargeGainMismatchThresh_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_MinPwrLevel_40,
					SSLPNPHY_Rev2_MinPwrLevel_40_ofdmMinPwrLevel_MASK,
					164 << SSLPNPHY_Rev2_MinPwrLevel_40_ofdmMinPwrLevel_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_MinPwrLevel_40,
					SSLPNPHY_Rev2_MinPwrLevel_40_dsssMinPwrLevel_MASK,
					159 << SSLPNPHY_Rev2_MinPwrLevel_40_dsssMinPwrLevel_SHIFT);
				write_phy_reg(pi, SSLPNPHY_Rev2_gainBackOffVal_40, 0x6366);
				mod_phy_reg(pi, SSLPNPHY_Rev2_VeryLowGainDB_40,
					SSLPNPHY_Rev2_VeryLowGainDB_40_NominalPwrDB_MASK,
					103  << SSLPNPHY_Rev2_VeryLowGainDB_40_NominalPwrDB_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_LowGainDB_40,
					SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_MASK,
					29 << SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40,
					SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK,
				255 << SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_PwrThresh0_40,
					SSLPNPHY_Rev2_PwrThresh0_40_SlowPwrLoThresh_MASK,
					11 << SSLPNPHY_Rev2_PwrThresh0_40_SlowPwrLoThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
					SSLPNPHY_Rev2_transFreeThresh_20U_SlowPwrLoThresh_MASK,
				11 << SSLPNPHY_Rev2_transFreeThresh_20U_SlowPwrLoThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
					SSLPNPHY_Rev2_transFreeThresh_20L_SlowPwrLoThresh_MASK,
				11 << SSLPNPHY_Rev2_transFreeThresh_20L_SlowPwrLoThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_gainMismatchMedGainEx_40,
		SSLPNPHY_Rev2_gainMismatchMedGainEx_40_medHiGainDirectMismatchOFDMDet_MASK,
			3 <<
		SSLPNPHY_Rev2_gainMismatchMedGainEx_40_medHiGainDirectMismatchOFDMDet_SHIFT);

				/* SGI -56 to -64dBm Hump Fixes */
				write_phy_reg(pi, SSLPNPHY_Rev2_ClipThresh_40, 72);
				mod_phy_reg(pi, SSLPNPHY_Rev2_ClipCtrThresh_40,
					SSLPNPHY_Rev2_ClipCtrThresh_40_ClipCtrThreshHiGain_MASK,
				44 << SSLPNPHY_Rev2_ClipCtrThresh_40_ClipCtrThreshHiGain_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40,
					SSLPNPHY_Rev2_HiGainDB_40_MedHiGainDB_MASK,
					45  << SSLPNPHY_Rev2_HiGainDB_40_MedHiGainDB_SHIFT);
				/* SGI -20 to -32dBm Hump Fixes */
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsTimingCtrl_40,
					SSLPNPHY_Rev2_crsTimingCtrl_40_gainThrsh4MF_MASK,
					73 << SSLPNPHY_Rev2_crsTimingCtrl_40_gainThrsh4MF_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsTimingCtrl_40,
					SSLPNPHY_Rev2_crsTimingCtrl_40_gainThrsh4Timing_MASK,
					0 << SSLPNPHY_Rev2_crsTimingCtrl_40_gainThrsh4Timing_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsMiscParams_40,
					SSLPNPHY_Rev2_crsMiscParams_40_incSyncCntVal_MASK,
					0 << SSLPNPHY_Rev2_crsMiscParams_40_incSyncCntVal_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_lpParam2_40,
					SSLPNPHY_Rev2_lpParam2_40_gainSettleDlySmplCnt_MASK,
					60 << SSLPNPHY_Rev2_lpParam2_40_gainSettleDlySmplCnt_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsMiscCtrl0_40,
					SSLPNPHY_Rev2_crsMiscCtrl0_40_usePreFiltPwr_MASK,
					0 << SSLPNPHY_Rev2_crsMiscCtrl0_40_usePreFiltPwr_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams2_20U,
					SSLPNPHY_Rev2_syncParams2_20U_gainThrsh4MF_MASK,
					66 << SSLPNPHY_Rev2_syncParams2_20U_gainThrsh4MF_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams2_20U,
					SSLPNPHY_Rev2_syncParams2_20U_gainThrsh4Timing_MASK,
					0 << SSLPNPHY_Rev2_syncParams2_20U_gainThrsh4Timing_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams1_20U,
					SSLPNPHY_Rev2_syncParams1_20U_incSyncCntVal_MASK,
					0 << SSLPNPHY_Rev2_syncParams1_20U_incSyncCntVal_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams2_20L,
					SSLPNPHY_Rev2_syncParams2_20L_gainThrsh4MF_MASK,
				66 << SSLPNPHY_Rev2_syncParams2_20L_gainThrsh4MF_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams2_20L,
					SSLPNPHY_Rev2_syncParams2_20L_gainThrsh4Timing_MASK,
				0 << SSLPNPHY_Rev2_syncParams2_20L_gainThrsh4Timing_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_syncParams1_20L,
					SSLPNPHY_Rev2_syncParams1_20L_incSyncCntVal_MASK,
				0 << SSLPNPHY_Rev2_syncParams1_20L_incSyncCntVal_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_bndWdthClsfy2_40,
					SSLPNPHY_Rev2_bndWdthClsfy2_40_bwClsfyGainThresh_MASK,
				66 << SSLPNPHY_Rev2_bndWdthClsfy2_40_bwClsfyGainThresh_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmPwrThresh0_20L,
					SSLPNPHY_Rev2_ofdmPwrThresh0_20L_ofdmPwrThresh0_MASK,
				3 << SSLPNPHY_Rev2_ofdmPwrThresh0_20L_ofdmPwrThresh0_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmPwrThresh1_20L,
					SSLPNPHY_Rev2_ofdmPwrThresh1_20L_ofdmPwrThresh3_MASK,
				48 << SSLPNPHY_Rev2_ofdmPwrThresh1_20L_ofdmPwrThresh3_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmPwrThresh0_20U,
					SSLPNPHY_Rev2_ofdmPwrThresh0_20U_ofdmPwrThresh0_MASK,
				3 << SSLPNPHY_Rev2_ofdmPwrThresh0_20U_ofdmPwrThresh0_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmPwrThresh1_20U,
					SSLPNPHY_Rev2_ofdmPwrThresh1_20U_ofdmPwrThresh3_MASK,
				48 << SSLPNPHY_Rev2_ofdmPwrThresh1_20U_ofdmPwrThresh3_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_radioTRCtrl,
					SSLPNPHY_Rev2_radioTRCtrl_gainrequestTRAttnOnOffset_MASK,
				7 << SSLPNPHY_Rev2_radioTRCtrl_gainrequestTRAttnOnOffset_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_trGainthresh_40,
					SSLPNPHY_Rev2_trGainthresh_40_trGainThresh_MASK,
				20 << SSLPNPHY_Rev2_trGainthresh_40_trGainThresh_SHIFT);

				/* TO REDUCE PER HUMPS @HIGH Rx Powers */
				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmSyncThresh1_40,
					SSLPNPHY_Rev2_ofdmSyncThresh1_40_ofdmSyncThresh2_MASK,
				2 << SSLPNPHY_Rev2_ofdmSyncThresh1_40_ofdmSyncThresh2_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmSyncThresh1_20U,
					SSLPNPHY_Rev2_ofdmSyncThresh1_20U_ofdmSyncThresh2_MASK,
				2 << SSLPNPHY_Rev2_ofdmSyncThresh1_20U_ofdmSyncThresh2_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_ofdmSyncThresh1_20L,
					SSLPNPHY_Rev2_ofdmSyncThresh1_20L_ofdmSyncThresh2_MASK,
				2 << SSLPNPHY_Rev2_ofdmSyncThresh1_20L_ofdmSyncThresh2_SHIFT);

				mod_phy_reg(pi, SSLPNPHY_Rev2_DSSSConfirmCnt_40,
					SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntHiGain_MASK,
				4 << SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntHiGain_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_DSSSConfirmCnt_40,
					SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntLoGain_MASK,
				4 << SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntLoGain_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_DSSSConfirmCnt_40,
			SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntHiGainCnfrm_MASK,
			2 << SSLPNPHY_Rev2_DSSSConfirmCnt_40_DSSSConfirmCntHiGainCnfrm_SHIFT);
		}

	}
	} else {
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2))
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 500);
	}

	/* Change timing to 11.5us */
	wlc_sslpnphy_set_tx_pwr_by_index(pi, 40);
	sslpnphy_specific->sslpnphy_current_index = 40;
	write_phy_reg(pi, SSLPNPHY_TxMacIfHoldOff, 23);
	write_phy_reg(pi, SSLPNPHY_TxMacDelay, 1002);
	/* Adjust RIFS timings */
	if (phybw40 == 0) {
		write_phy_reg(pi, SSLPNPHY_rifsSttimeout, 0x1214);
		write_phy_reg(pi, SSLPNPHY_readsym2resetCtrl, 0x7800);
	}
}

static void
WLBANDINITFN(wlc_sslpnphy_baseband_init)(phy_info_t *pi)
{
	/* Initialize SSLPNPHY tables */
	wlc_sslpnphy_tbl_init(pi);
	wlc_sslpnphy_rev0_baseband_init(pi);
	wlc_sslpnphy_bu_tweaks(pi);
}


typedef struct {
	uint16 phy_addr;
	uint8 phy_shift;
	uint8 rf_addr;
	uint8 rf_shift;
	uint8 mask;
} sslpnphy_extstxdata_t;

static sslpnphy_extstxdata_t
WLBANDINITDATA(sslpnphy_extstxdata)[] = {
	{SSLPNPHY_extstxctrl0 + 2, 6, 0x3d, 3, 0x1},
	{SSLPNPHY_extstxctrl0 + 1, 12, 0x4c, 1, 0x1},
	{SSLPNPHY_extstxctrl0 + 1, 8, 0x50, 0, 0x7f},
	{SSLPNPHY_extstxctrl0 + 0, 8, 0x44, 0, 0xff},
	{SSLPNPHY_extstxctrl0 + 1, 0, 0x4a, 0, 0xff},
	{SSLPNPHY_extstxctrl0 + 0, 4, 0x4d, 0, 0xff},
	{SSLPNPHY_extstxctrl0 + 1, 4, 0x4e, 0, 0xff},
	{SSLPNPHY_extstxctrl0 + 0, 12, 0x4f, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 1, 0, 0x4f, 4, 0xf},
	{SSLPNPHY_extstxctrl0 + 3, 0, 0x49, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 4, 3, 0x46, 4, 0x7},
	{SSLPNPHY_extstxctrl0 + 3, 15, 0x46, 0, 0x1},
	{SSLPNPHY_extstxctrl0 + 4, 0, 0x46, 1, 0x7},
	{SSLPNPHY_extstxctrl0 + 3, 8, 0x48, 4, 0x7},
	{SSLPNPHY_extstxctrl0 + 3, 11, 0x48, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 3, 4, 0x49, 4, 0xf},
	{SSLPNPHY_extstxctrl0 + 2, 15, 0x45, 0, 0x1},
	{SSLPNPHY_extstxctrl0 + 5, 13, 0x52, 4, 0x7},
	{SSLPNPHY_extstxctrl0 + 6, 0, 0x52, 7, 0x1},
	{SSLPNPHY_extstxctrl0 + 5, 3, 0x41, 5, 0x7},
	{SSLPNPHY_extstxctrl0 + 5, 6, 0x41, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 5, 10, 0x42, 5, 0x7},
	{SSLPNPHY_extstxctrl0 + 4, 15, 0x42, 0, 0x1},
	{SSLPNPHY_extstxctrl0 + 5, 0, 0x42, 1, 0x7},
	{SSLPNPHY_extstxctrl0 + 4, 11, 0x43, 4, 0xf},
	{SSLPNPHY_extstxctrl0 + 4, 7, 0x43, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 4, 6, 0x45, 1, 0x1},
	{SSLPNPHY_extstxctrl0 + 2, 7, 0x40, 4, 0xf},
	{SSLPNPHY_extstxctrl0 + 2, 11, 0x40, 0, 0xf},
	{SSLPNPHY_extstxctrl0 + 1, 14, 0x3c, 3, 0x3},
	{SSLPNPHY_extstxctrl0 + 2, 0, 0x3c, 5, 0x7},
	{SSLPNPHY_extstxctrl0 + 2, 3, 0x3c, 0, 0x7},
	{SSLPNPHY_extstxctrl0 + 0, 0, 0x52, 0, 0xf},
	};

static void
WLBANDINITFN(wlc_sslpnphy_synch_stx)(phy_info_t *pi)
{
	uint i;

	mod_radio_reg(pi, RADIO_2063_COMMON_04, 0xf8, 0xff);
	write_radio_reg(pi, RADIO_2063_COMMON_05, 0xff);
	write_radio_reg(pi, RADIO_2063_COMMON_06, 0xff);
	write_radio_reg(pi, RADIO_2063_COMMON_07, 0xff);
	mod_radio_reg(pi, RADIO_2063_COMMON_08, 0x7, 0xff);

	for (i = 0; i < ARRAYSIZE(sslpnphy_extstxdata); i++) {
		mod_phy_reg(pi,
			sslpnphy_extstxdata[i].phy_addr,
			(uint16)sslpnphy_extstxdata[i].mask << sslpnphy_extstxdata[i].phy_shift,
			(uint16)(read_radio_reg(pi, sslpnphy_extstxdata[i].rf_addr) >>
			sslpnphy_extstxdata[i].rf_shift) << sslpnphy_extstxdata[i].phy_shift);
	}

	mod_radio_reg(pi, RADIO_2063_COMMON_04, 0xf8, 0);
	write_radio_reg(pi, RADIO_2063_COMMON_05, 0);
	write_radio_reg(pi, RADIO_2063_COMMON_06, 0);
	write_radio_reg(pi, RADIO_2063_COMMON_07, 0);
	mod_radio_reg(pi, RADIO_2063_COMMON_08, 0x7, 0);
}

static void
WLBANDINITFN(sslpnphy_run_jtag_rcal)(phy_info_t *pi)
{
	int RCAL_done;
	int RCAL_timeout = 10;

	/* global RCAL override Enable */
	write_radio_reg(pi, RADIO_2063_COMMON_13, 0x10);

	/* put in override and power down RCAL */
	write_radio_reg(pi, RADIO_2063_COMMON_16, 0x10);

	/* Run RCAL */
	write_radio_reg(pi, RADIO_2063_COMMON_16, 0x14);

	/* Wait for RCAL Valid bit to be set */
	RCAL_done = (read_radio_reg(pi, RADIO_2063_COMMON_17) & 0x20) >> 5;

	while (RCAL_done == 0 && RCAL_timeout > 0) {
		OSL_DELAY(1);
		RCAL_done = (read_radio_reg(pi, RADIO_2063_COMMON_17) & 0x20) >> 5;
		RCAL_timeout--;
	}

	ASSERT(RCAL_done != 0);

	/* RCAL is done, now power down RCAL to save 100uA leakage during IEEE PS
	 * sleep, last RCAL value will remain valid even if RCAL is powered down
	*/
	write_radio_reg(pi, RADIO_2063_COMMON_16, 0x10);
}

static void
WLBANDINITFN(wlc_sslpnphy_radio_init)(phy_info_t *pi)
{
	uint32 macintmask;
	uint8 phybw40;

	phybw40 = IS40MHZ(pi);

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	if (NORADIO_ENAB(pi->pubpi))
		return;

	/* Toggle radio reset */
	or_phy_reg(pi, SSLPNPHY_fourwireControl, SSLPNPHY_fourwireControl_radioReset_MASK);
	OSL_DELAY(1);
	and_phy_reg(pi, SSLPNPHY_fourwireControl, ~SSLPNPHY_fourwireControl_radioReset_MASK);
	OSL_DELAY(1);

	/* Initialize 2063 radio */
	wlc_radio_2063_init_sslpnphy(pi);

	/* Synchronize phy overrides for RF registers that are mapped through the CLB */
	wlc_sslpnphy_synch_stx(pi);

	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		or_radio_reg(pi, RADIO_2063_COMMON_04, 0x40);
		or_radio_reg(pi, RADIO_2063_TXRF_SP_3, 0x08);
	} else
		and_radio_reg(pi, RADIO_2063_COMMON_04, ~(uint8)0x40);

	/* Shared RX clb signals */
	write_phy_reg(pi, SSLPNPHY_extslnactrl0, 0x5f80);
	write_phy_reg(pi, SSLPNPHY_extslnactrl1, 0x0);

	/* Set Tx Filter Bandwidth */
	mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
		SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
		3 << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg1,
		SSLPNPHY_lpfbwlutreg1_lpfbwlut5_MASK,
		2 << SSLPNPHY_lpfbwlutreg1_lpfbwlut5_SHIFT);

	if (IS_OLYMPIC(pi))
		mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
			SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
			1 << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (phybw40 == 1) {
			mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg1,
				SSLPNPHY_lpfbwlutreg1_lpfbwlut5_MASK,
				4 << SSLPNPHY_lpfbwlutreg1_lpfbwlut5_SHIFT);
		} else {
			mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
				SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
				0 << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);
		}
	}
	write_radio_reg(pi, RADIO_2063_PA_CTRL_14, 0xee);

	/* Run RCal */
#ifdef BCMRECLAIM
	if (!bcmreclaimed) {
#endif /* BCMRECLAIM */
#ifdef PHYHAL
	    macintmask = wlapi_intrsoff(pi->sh->physhim);
#else
		macintmask = wl_intrsoff(((wlc_info_t *)(pi->wlc))->wl);
#endif
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			sslpnphy_run_jtag_rcal(pi);
		} else {
			si_pmu_rcal(GENERIC_PHY_INFO(pi)->sih, GENERIC_PHY_INFO(pi)->osh);
		}
#ifdef PHYHAL	
		wlapi_intrsrestore(pi->sh->physhim, macintmask);
#else
		wl_intrsrestore(((wlc_info_t *)(pi->wlc))->wl, macintmask);
#endif
#ifdef BCMRECLAIM
	}
#endif
}
static void
wlc_sslpnphy_rc_cal(phy_info_t *pi)
{
	uint8 rxbb_sp8, txbb_sp_3;
	uint8 save_pll_jtag_pll_xtal;
	uint16 epa_ovr, epa_ovr_val;

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	if (NORADIO_ENAB(pi->pubpi))
		return;

	/* RF_PLL_jtag_pll_xtal_1_2 */
	save_pll_jtag_pll_xtal = (uint8) read_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 0x4);

	/* Save old cap value incase RCCal fails */
	rxbb_sp8 = (uint8)read_radio_reg(pi, RADIO_2063_RXBB_SP_8);

	/* Save RF overide values */
	epa_ovr = read_phy_reg(pi, SSLPNPHY_RFOverride0);
	epa_ovr_val = read_phy_reg(pi, SSLPNPHY_RFOverrideVal0);

	/* Switch off ext PA */
	if (CHSPEC_IS5G(pi->radio_chanspec) &&
		(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)) {
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_MASK,
			0 << SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_SHIFT);
	}
	/* Clear the RCCal Reg Override */
	write_radio_reg(pi, RADIO_2063_RXBB_SP_8, 0x0);

	/* Power down RC CAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7e);

	/* Power up PLL_cal_out_pd_0 (bit 4) */
	and_radio_reg(pi, RADIO_2063_PLL_SP_1, 0xf7);

	/* Power Up RC CAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7c);

	/* setup to run RX RC Cal and setup R1/Q1/P1 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_2, 0x15);

	/* set X1 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_3, 0x70);

	/* set Trc1 */

	if ((XTALFREQ(pi->xtalfreq) == 38400000) || (XTALFREQ(pi->xtalfreq) == 37400000)) {
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0xa0);
	} else if (XTALFREQ(pi->xtalfreq) == 30000000) {
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x52);   /* only for 30MHz in 4319 */
	} else if (XTALFREQ(pi->xtalfreq) == 26000000) {
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x32);   /* For 26MHz Xtal */
	}
	/* set Trc2 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_5, 0x1);

	/* Start rx RCCAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7d);

	/* Wait for rx RCCAL completion */
	OSL_DELAY(50);
	SPINWAIT(!wlc_radio_2063_rc_cal_done(pi), 10 * 1000 * 1000);

	if (!wlc_radio_2063_rc_cal_done(pi)) {
		WL_ERROR(("wl%d: %s: Rx RC Cal failed\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
		write_radio_reg(pi, RADIO_2063_RXBB_SP_8, rxbb_sp8);
		/* Put an infinite while loop and get into a time out error */
		/* instead of proceeding  with R cal failure */
		while (1) {
		}
	} else
		WL_INFORM(("wl%d: %s:  Rx RC Cal completed: N0: %x%x, N1: %x%x, code: %x\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_8),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_7),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_10),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_9),
			read_radio_reg(pi, RADIO_2063_COMMON_11) & 0x1f));

	/* Save old cap value incase RCCal fails */
	txbb_sp_3 = (uint8)read_radio_reg(pi, RADIO_2063_TXBB_SP_3);

	/* Clear the RCCal Reg Override */
	write_radio_reg(pi, RADIO_2063_TXBB_SP_3, 0x0);

	/* Power down RC CAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7e);

	/* Power Up RC CAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7c);

	/* setup to run TX RC Cal and setup R1/Q1/P1 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_2, 0x55);

	/* set X1 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_3, 0x76);

	if (XTALFREQ(pi->xtalfreq) == 26000000) {
		/* set Trc1 */
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x30);
	} else if ((XTALFREQ(pi->xtalfreq) == 38400000) || (XTALFREQ(pi->xtalfreq) == 37400000)) {
		/* set Trc1 */
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x96);
	} else if (XTALFREQ(pi->xtalfreq) == 30000000) {
		/* set Trc1 */
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x3d);  /* for 30Mhz 4319 */
	} else {
		/* set Trc1 */
		write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_4, 0x30);
	}
	/* set Trc2 */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_5, 0x1);

	/* Start tx RCCAL */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7d);

	/* Wait for tx RCCAL completion */
	OSL_DELAY(50);
	SPINWAIT(!wlc_radio_2063_rc_cal_done(pi), 10 * 1000 * 1000);

	if (!wlc_radio_2063_rc_cal_done(pi)) {
		WL_ERROR(("wl%d: %s: Tx RC Cal failed\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
		write_radio_reg(pi, RADIO_2063_TXBB_SP_3, txbb_sp_3);
		/* Put an infinite while loop and get into a time out error */
		/* instead of proceeding  with R cal failure */
		while (1) {
		}
	} else
		WL_INFORM(("wl%d: %s:  Tx RC Cal completed: N0: %x%x, N1: %x%x, code: %x\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_8),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_7),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_10),
			read_radio_reg(pi, RADIO_2063_RCCAL_CTRL_9),
			read_radio_reg(pi, RADIO_2063_COMMON_12) & 0x1f));

	/* Power down RCCAL after it is done */
	write_radio_reg(pi, RADIO_2063_RCCAL_CTRL_1, 0x7e);

	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, save_pll_jtag_pll_xtal);
	/* Restore back amode tx pu */
	write_phy_reg(pi, SSLPNPHY_RFOverride0, epa_ovr);
	write_phy_reg(pi, SSLPNPHY_RFOverrideVal0, epa_ovr_val);
}

STATIC void
wlc_sslpnphy_toggle_afe_pwdn(phy_info_t *pi)
{
	uint16 save_AfeCtrlOvrVal, save_AfeCtrlOvr;

	save_AfeCtrlOvrVal = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal);
	save_AfeCtrlOvr = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvr);

	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal, save_AfeCtrlOvrVal | 0x1);
	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvr, save_AfeCtrlOvr | 0x1);

	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal, save_AfeCtrlOvrVal & 0xfffe);
	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvr, save_AfeCtrlOvr & 0xfffe);

	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal, save_AfeCtrlOvrVal);
	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvr, save_AfeCtrlOvr);
}
static void
wlc_sslpnphy_common_read_table(phy_info_t *pi, uint32 tbl_id,
	CONST void *tbl_ptr, uint32 tbl_len, uint32 tbl_width,
	uint32 tbl_offset) {

	phytbl_info_t tab;
	tab.tbl_id = tbl_id;
	tab.tbl_ptr = tbl_ptr;	/* ptr to buf */
	tab.tbl_len = tbl_len;			/* # values   */
	tab.tbl_width = tbl_width;			/* gain_val_tbl_rev3 */
	tab.tbl_offset = tbl_offset;		/* tbl offset */
	wlc_sslpnphy_read_table(pi, &tab);
}
static void
wlc_sslpnphy_common_write_table(phy_info_t *pi, uint32 tbl_id, CONST void *tbl_ptr,
	uint32 tbl_len, uint32 tbl_width, uint32 tbl_offset) {

	phytbl_info_t tab;
	tab.tbl_id = tbl_id;
	tab.tbl_ptr = tbl_ptr;	/* ptr to buf */
	tab.tbl_len = tbl_len;			/* # values   */
	tab.tbl_width = tbl_width;			/* gain_val_tbl_rev3 */
	tab.tbl_offset = tbl_offset;		/* tbl offset */
	wlc_sslpnphy_write_table(pi, &tab);
}
static void
wlc_sslpnphy_set_chanspec_default_tweaks(phy_info_t *pi)
{

	if (!NORADIO_ENAB(pi->pubpi)) {
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
				spur_tbl_rev2, 192, 8, 0);
		} else {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
				spur_tbl_rev0, 64, 8, 0);
		}

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		si_pmu_chipcontrol(GENERIC_PHY_INFO(pi)->sih, 0, 0xfff, ((0x8 << 0) | (0x1f << 6)));
		if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
			si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 3,
				((1 << 26) | (1 << 21)), ((1 << 26) | (1 << 21)));
			si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 5, (0x1ff << 9), (0x1ff << 9));
		}
	}

	mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
		SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
		255 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_lnsrOfParam1,
			SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_MASK,
			11 << SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_lnsrOfParam2,
			SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_MASK,
			1 << SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
			SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
			25 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
			SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
			100 << SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_HiGainDB,
			SSLPNPHY_HiGainDB_HiGainDB_MASK,
			70 << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
	}
	write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 360);

	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);
	}
}

void
wlc_sslpnphy_channel_gain_adjust(phy_info_t *pi)
{
	uint8 i;
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
	uint8 pwr_correction = 0;
	uint16 tempsense = 0;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
#ifdef BAND5G	
	uint16 chan_info_sslpnphy_cga_5g[24] = { 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320,
						 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640,
						 5660, 5680, 5700, 5745, 5765, 5785, 5805, 5825,
						};
#endif

	if (sslpnphy_specific->fem_combiner_current_state) {
		/* Saving tempsense value for combiner mode */
		tempsense = read_phy_reg(pi, SSLPNPHY_TempSenseCorrection);
	}

	/* Reset the tempsense offset */
	write_phy_reg(pi, SSLPNPHY_TempSenseCorrection, 0);
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		for (i = 0; i < ARRAYSIZE(chan_info_2063_sslpnphy); i++)
			if (chan_info_2063_sslpnphy[i].freq == freq) {
				pwr_correction = (uint8)sslpnphy_specific->sslpnphy_cga_2g[i];
				break;
			}
	}
#ifdef BAND5G
	 else {
		for (i = 0; i < ARRAYSIZE(chan_info_sslpnphy_cga_5g); i++)
			if (freq <= chan_info_sslpnphy_cga_5g[i]) {
				pwr_correction = (uint8)sslpnphy_specific->sslpnphy_cga_5g[i];
				break;
			}
	}
#endif

	/* Apply the channel based offset to each 5G channel + original tempsense */
	write_phy_reg(pi, SSLPNPHY_TempSenseCorrection, pwr_correction + tempsense);

}
#ifdef BAND5G
static void wlc_sslpnphy_radio_2063_channel_tweaks_A_band(phy_info_t *pi, uint freq);
#endif

static bool wlc_sslpnphy_fcc_chan_check(phy_info_t *pi, uint channel);
void
wlc_phy_chanspec_set_sslpnphy(phy_info_t *pi, chanspec_t chanspec)
{
	uint16 m_cur_channel = 0;
	uint8 channel = CHSPEC_CHANNEL(chanspec); /* see wlioctl.h */
	uint8 bw = CHSPEC_WLC_BW(chanspec);
	uint32 centreTs20, centreFactor;
	uint freq = 0;
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	/* Resetting OLYMPIC flag */
	WL_WRITE_SHM(pi, M_SSLPN_OLYMPIC, 0);

	wlc_phy_chanspec_radio_set((wlc_phy_t *)pi, chanspec);
	/* Set the phy bandwidth as dictated by the chanspec */
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (bw != pi->bw) {
#ifdef PHYHAL
			wlapi_bmac_bw_set(pi->sh->physhim, bw);
#else
			wlc_set_bw(pi->wlc, bw);
#endif
		}
	}

	wlc_sslpnphy_set_chanspec_default_tweaks(pi);

	freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
	/* Tune radio for the channel */
	if (!NORADIO_ENAB(pi->pubpi)) {
		wlc_sslpnphy_radio_2063_channel_tune(pi, channel);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (IS40MHZ(pi)) {
		if (CHSPEC_SB_UPPER(chanspec)) {
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
		                SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
		                1 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
		                SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
		                0 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		} else {
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
		                SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
		                0 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
		                SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
		                1 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		}
		}
	}
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		if (channel == 14) {
			mod_phy_reg(pi, SSLPNPHY_extstxctrl1, 0xfff, 0x15 << 4);
			mod_phy_reg(pi, SSLPNPHY_extstxctrl0, 0xfff << 4, 0x240 << 4);
			mod_phy_reg(pi, SSLPNPHY_extstxctrl4, 0xff << 7, 0x80 << 7);
			mod_phy_reg(pi, SSLPNPHY_lpphyCtrl,
				SSLPNPHY_lpphyCtrl_txfiltSelect_MASK,
				2 << SSLPNPHY_lpphyCtrl_txfiltSelect_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
				SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
				2 << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);
			wlc_sslpnphy_cck_filt_load(pi, 4);
		} else {
			write_phy_reg(pi, SSLPNPHY_extstxctrl0, sslpnphy_specific->sslpnphy_extstxctrl0);
			write_phy_reg(pi, SSLPNPHY_extstxctrl1, sslpnphy_specific->sslpnphy_extstxctrl1);
			write_phy_reg(pi, SSLPNPHY_extstxctrl4, sslpnphy_specific->sslpnphy_extstxctrl4);
			mod_phy_reg(pi, SSLPNPHY_lpphyCtrl,
				SSLPNPHY_lpphyCtrl_txfiltSelect_MASK,
				1 << SSLPNPHY_lpphyCtrl_txfiltSelect_SHIFT);
			wlc_sslpnphy_cck_filt_load(pi, sslpnphy_specific->sslpnphy_cck_filt_sel);
		}
		/* Do not do these FCC tunings for 40Mhz */
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2) && IS20MHZ(pi) && (channel != 14)) {
			/* Gurus FCC changes */
			if ((channel == 1) || (channel == 11)) {
				mod_phy_reg(pi, SSLPNPHY_extstxctrl0, 0xfff << 4, 0x040 << 4);
				mod_phy_reg(pi, SSLPNPHY_extstxctrl1, 0xf, 0);
			} else {
				mod_phy_reg(pi, SSLPNPHY_extstxctrl0, 0xfff << 4, 0x035 << 4);
				mod_phy_reg(pi, SSLPNPHY_extstxctrl1, 0xf, 0);
			}
		}
		if (SSLPNREV_LE(pi->pubpi.phy_rev, 1) && (channel != 14)) {
			if (wlc_sslpnphy_fcc_chan_check(pi, channel)) {
				mod_phy_reg(pi, SSLPNPHY_extstxctrl0,
					0xfff << 4, 0xf00 << 4);
				mod_phy_reg(pi, SSLPNPHY_extstxctrl1, 0xfff, 0x0c3);
				mod_phy_reg(pi, SSLPNPHY_extstxctrl3, 0xff, 0x90);

			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
				BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) {

					mod_phy_reg(pi, SSLPNPHY_extstxctrl0,
						0xfff << 4, 0x800 << 4);
					mod_phy_reg(pi, SSLPNPHY_extstxctrl1, 0xfff, 0x254);
					mod_phy_reg(pi, SSLPNPHY_extstxctrl3, 0xff, 0xc0);
					mod_phy_reg(pi, SSLPNPHY_extstxctrl2, 0xf << 7, 0xa << 7);
					mod_phy_reg(pi, SSLPNPHY_extstxctrl4, 0xf << 11, 0x9 << 11);
					mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg1,
						SSLPNPHY_lpfbwlutreg1_lpfbwlut5_MASK,
						1 << SSLPNPHY_lpfbwlutreg1_lpfbwlut5_SHIFT);
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
						sslpnphy_rev1_cx_ofdm_fcc, 10);
					wlc_sslpnphy_load_filt_coeff(pi,
						SSLPNPHY_txrealfilt_ofdm_tap0,
						sslpnphy_rev1_real_ofdm_fcc, 5);
					wlc_sslpnphy_load_filt_coeff(pi,
						SSLPNPHY_txrealfilt_ht_tap0,
						sslpnphy_rev1_real_ht_fcc, 5);

					/* ch 1 and ch11 is set to class A and requires special PAPD tweaks */
					sslpnphy_specific->sslpnphy_radio_classA = TRUE;
					sslpnphy_specific->sslpnphy_papd_tweaks_enable = TRUE;
					sslpnphy_specific->sslpnphy_papd_tweaks.final_idx_thresh = 42000; /* 1.6 max in PAPD LUT */
					sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_begin = 5500;
					sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_step = 0x222; /* 0.5dB step to cover broader range */ 
					sslpnphy_specific->sslpnphy_papd_tweaks.min_final_idx_thresh = 5000; /* lower limit for papd cal index search */


				} else if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICLOCO_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID)) {
					/* Moving to more class A radio settings */
					uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
					WL_WRITE_SHM(pi, M_SSLPN_OLYMPIC, 2);
					write_phy_reg(pi, SSLPNPHY_extstxctrl2, 0x82d8);
					write_phy_reg(pi, SSLPNPHY_extstxctrl4, 0x405c);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_CCK)), 0x2280);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_CCK)), 0x3150);

					if (channel == 1) {
						write_phy_reg(pi, SSLPNPHY_extstxctrl0, 0xA290);
						write_phy_reg(pi, SSLPNPHY_extstxctrl1, 0x3330);
						mod_phy_reg(pi, SSLPNPHY_extstxctrl3, 0xff, 0xF0);
						mod_phy_reg(pi, SSLPNPHY_extstxctrl5, 0xf<<8, 0xC<<8);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F2_16_64)), 0xA290);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F3_16_64)), 0x3330);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F2_2_4)), 0xF000);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F3_2_4)), 0x33A3);
						/* ch 1 is set to class A and requires special PAPD tweaks */
						sslpnphy_specific->sslpnphy_radio_classA = TRUE;
						sslpnphy_specific->sslpnphy_papd_tweaks_enable = TRUE;
						sslpnphy_specific->sslpnphy_papd_tweaks.final_idx_thresh = 40000; /* 1.55 max in PAPD LUT */
						sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_begin = 5500;
						sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_step = 0x222; /* 0.5dB step to cover broader range */ 
						sslpnphy_specific->sslpnphy_papd_tweaks.min_final_idx_thresh = 10000; /* lower limit for papd cal index search */
					}
					else if (channel == 11) {
						mod_phy_reg(pi, SSLPNPHY_extstxctrl3, 0xff, 0xC0);
						mod_phy_reg(pi, SSLPNPHY_extstxctrl5, 0xf<<8, 0xc<<8);
						write_phy_reg(pi, SSLPNPHY_extstxctrl0, 0x9210);
						write_phy_reg(pi, SSLPNPHY_extstxctrl1, 0x3150);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F2_16_64)), 0x9210);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F3_16_64)), 0x3150);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F2_2_4)), 0x1000);
						WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
							M_SSLPNPHY_REG_4F3_2_4)), 0x30c3);
					}
				}

			} else {
				write_phy_reg(pi, SSLPNPHY_extstxctrl0,
					sslpnphy_specific->sslpnphy_extstxctrl0);
				write_phy_reg(pi, SSLPNPHY_extstxctrl1,
					sslpnphy_specific->sslpnphy_extstxctrl1);
				write_phy_reg(pi, SSLPNPHY_extstxctrl3,
					sslpnphy_specific->sslpnphy_extstxctrl3);
				write_phy_reg(pi, SSLPNPHY_extstxctrl2,
					sslpnphy_specific->sslpnphy_extstxctrl2);
				write_phy_reg(pi, SSLPNPHY_extstxctrl4,
					sslpnphy_specific->sslpnphy_extstxctrl4);
				write_phy_reg(pi, SSLPNPHY_extstxctrl5,
					sslpnphy_specific->sslpnphy_extstxctrl5);
				write_phy_reg(pi, SSLPNPHY_lpfbwlutreg1, sslpnphy_specific->sslpnphy_ofdm_filt_bw);

				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
					BCM94329OLYMPICX17M_SSID ||
					BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
					BCM94329OLYMPICX17U_SSID) {
					if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
					    (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_ofdm_tap0_i,
					        sslpnphy_rev1_cx_ofdm_sec, 10);
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_txrealfilt_ofdm_tap0,
					        sslpnphy_rev1_real_ofdm_sec, 5);
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_txrealfilt_ht_tap0,
					        sslpnphy_rev1_real_ht_sec, 5);
					} else {
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_ofdm_tap0_i,
					        sslpnphy_rev1_cx_ofdm, 10);
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_txrealfilt_ofdm_tap0,
					        sslpnphy_rev1_real_ofdm, 5);
					    wlc_sslpnphy_load_filt_coeff(pi,
					        SSLPNPHY_txrealfilt_ht_tap0,
					        sslpnphy_rev1_real_ht, 5);
					}
				}
			}
		}
	}
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if (freq < 5725)
			wlc_sslpnphy_load_tx_gain_table(pi, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_midband);
		else
			wlc_sslpnphy_load_tx_gain_table(pi, sslpnphy_specific->sslpnphy_tx_gaintbl_5GHz_hiband);

		wlc_sslpnphy_radio_2063_channel_tweaks_A_band(pi, freq);
	}
#endif /* BAND5G */
		OSL_DELAY(1000);
	}
	/* apply hannel based power offset to reduce power control errors */
	wlc_sslpnphy_channel_gain_adjust(pi);
	/* toggle the afe whenever we move to a new channel */
	wlc_sslpnphy_toggle_afe_pwdn(pi);

#if !defined(PHYHAL)
	pi->radio_code = channel;
#endif
	m_cur_channel = channel;

	if (CHSPEC_IS5G(chanspec))
		m_cur_channel |= D11_CURCHANNEL_5G;
	if (CHSPEC_IS40(chanspec))
		m_cur_channel |= D11_CURCHANNEL_40;

	centreTs20 = wlc_lpphy_qdiv_roundup(freq * 2, 5, 0);
	centreFactor = wlc_lpphy_qdiv_roundup(2621440, freq, 0);
	write_phy_reg(pi, SSLPNPHY_ptcentreTs20, (uint16)centreTs20);
	write_phy_reg(pi, SSLPNPHY_ptcentreFactor, (uint16)centreFactor);

	WL_WRITE_SHM(pi, M_CURCHANNEL, m_cur_channel);
	write_phy_channel_reg(pi, channel);

	/* Indicate correct antdiv register offset to ucode */
	if (CHSPEC_IS40(chanspec))
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr + M_SSLPNPHY_ANTDIV_REG)),
			SSLPNPHY_Rev2_crsgainCtrl_40);
	else
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr + M_SSLPNPHY_ANTDIV_REG)),
			SSLPNPHY_Rev2_crsgainCtrl);

	/* Enable Dmof Bhy ACI filter for warrior alone */
	if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319LCSDN4L_SSID)
		write_phy_reg(pi, SSLPNPHY_bphyacireg, 0x4);

	sslpnphy_specific->sslpnphy_papd_cal_done = 0;
}
static void
wlc_sslpnphy_RxNvParam_Adj(phy_info_t *pi)
{
	int8 path_loss = 0, temp1;
	uint8 channel = CHSPEC_CHANNEL(pi->radio_chanspec); /* see wlioctl.h */
	uint8 phybw40 = IS40MHZ(pi);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		path_loss = (int8)sslpnphy_specific->sslpnphy_rx_power_offset;
#ifdef BAND5G
	} else {
		path_loss = (int8)sslpnphy_specific->sslpnphy_rx_power_offset_5g;
#endif
	}
	temp1 = (int8)(read_phy_reg(pi, SSLPNPHY_InputPowerDB)
		& SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (phybw40)
			temp1 = (int8)(read_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40)
				& SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK);
	}
	if ((sslpnphy_specific->sslpnphy_rxpo2gchnflg) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		if (sslpnphy_specific->sslpnphy_rxpo2gchnflg & (0x1 << (channel - 1)))
			temp1 -= path_loss;
	} else {
		temp1 -= path_loss;
	}
	mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
		SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
		(temp1 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (phybw40)
			mod_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40,
				SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK,
				(temp1 << SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_SHIFT));
	}


}

STATIC uint16 ELNA_CCK_ACI_GAINTBL_TWEAKS [][2] = {
	{50,  0x24C},
	{51,  0x233},
	{52,  0x24D},
	{53,  0x24E},
	{54,  0x240},
	{55,  0x24F},
	{56,  0x2C0},
	{57,  0x2D0},
	{58,  0x2D8},
	{59,  0x2AF},
	{60,  0x2B9},
	{61,  0x32F},
	{62,  0x33A},
	{63,  0x33B},
	{64,  0x33C},
	{65,  0x33D},
	{66,  0x33E},
	{67,  0x3BF},
	{68,  0x3BE},
	{69,  0x3C2},
	{70,  0x3C1},
	{71,  0x3C4},
	{72,  0x3B0},
	{73,  0x3B1}
};
STATIC uint16 ELNA_OLYMPIC_OFDM_ACI_GAINTBL_TWEAKS [][2] = {
	{23,  0x381},
	{24,  0x385},
	{25,  0x3D4},
	{26,  0x3D5},
	{27,  0x389},
	{28,  0x3AB},
	{29,  0x3D6},
	{30,  0x3D8},
	{31,  0x38E},
	{32,  0x38C},
	{33,  0x38A}
};
STATIC uint16 ELNA_OLYMPIC_CCK_ACI_GAINTBL_TWEAKS [][2] = {
	{61,  0x28F },
	{62,  0x32C },
	{63,  0x33E },
	{64,  0x33F },
	{65,  0x3C3 },
	{66,  0x3AE },
	{67,  0x3CA },
	{68,  0x3C9 },
	{69,  0x387 },
	{70,  0x3CB },
	{71,  0x3CE },
	{72,  0x3CF },
	{73,  0x3D0 }
};
STATIC uint16 ELNA_OLYMPIC_OFDM_5G_GAINTBL_TWEAKS [][2] = {
	{0,  0x200 },
	{1,  0x200 },
	{2,  0x200 },
	{3,  0x200 },
	{4,  0x200 },
	{5,  0x200 },
	{6,  0x200 },
	{7,  0x200 },
	{8,  0x200 },
	{9,  0x200 },
	{10,  0x251 },
	{11,  0x252 },
	{12,  0x2D1 },
	{13,  0x2D2 },
	{14,  0x2CC },
	{15,  0x2B3 },
	{16,  0x2A7 },
	{17,  0x286 },
	{18,  0x28B },
	{19,  0x290 },
	{20,  0x38F },
	{21,  0x393 },
	{22,  0x38B },
	{23,  0x390 },
	{24,  0x38C },
	{25,  0x38E },
	{26,  0x38D },
	{27,  0x3A3 },
	{28,  0x388 }
};
STATIC uint16 ELNA_OLYMPIC_OFDM_5G_GAINTBL_TSMC_TWEAKS [][2] = {
	{18,  0x387 },
	{19,  0x38A },
	{20,  0x38F },
	{21,  0x3A6 },
	{22,  0x3BE },
	{23,  0x3BF },
	{24,  0x3C0 },
	{25,  0x3C5 },
	{26,  0x397 }
};
STATIC uint16 UNO_LOCO_ELNA_CCK_GAINTBL_TWEAKS [][2] = {
	{39,  0x651},
	{40,  0x680},
	{41,  0x6D1},
	{42,  0x700},
	{43,  0x751},
	{44,  0x780},
	{45,  0x7D1},
	{46,  0x7D2}
};
uint8 ELNA_CCK_ACI_GAINTBL_TWEAKS_sz = ARRAYSIZE(ELNA_CCK_ACI_GAINTBL_TWEAKS);
uint8 ELNA_OLYMPIC_OFDM_ACI_GAINTBL_TWEAKS_sz = ARRAYSIZE(ELNA_OLYMPIC_OFDM_ACI_GAINTBL_TWEAKS);
uint8 ELNA_OLYMPIC_CCK_ACI_GAINTBL_TWEAKS_sz = ARRAYSIZE(ELNA_OLYMPIC_CCK_ACI_GAINTBL_TWEAKS);
uint8 ELNA_OLYMPIC_OFDM_5G_GAINTBL_TWEAKS_sz = ARRAYSIZE(ELNA_OLYMPIC_OFDM_5G_GAINTBL_TWEAKS);
uint8 ELNA_OLYMPIC_OFDM_5G_GAINTBL_TSMC_TWEAKS_sz = ARRAYSIZE(ELNA_OLYMPIC_OFDM_5G_GAINTBL_TSMC_TWEAKS);
uint8 UNO_LOCO_ELNA_CCK_GAINTBL_TWEAKS_sz = ARRAYSIZE(UNO_LOCO_ELNA_CCK_GAINTBL_TWEAKS);
static void
wlc_sslpnphy_rx_gain_table_tweaks(phy_info_t *pi, uint8 idx, uint16 ptr[][2], uint8 array_size)
{
	uint8 i = 0, tbl_entry;
	uint16 wlprio_11bit_code;
	uint32 tableBuffer[2];
	for (i = 0; i < array_size; i++) {
		if (ptr[i][0] == idx) {
			wlprio_11bit_code = ptr[i][1];
			tbl_entry = idx << 1;
			wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tableBuffer, 2, 32, tbl_entry);
			tableBuffer[0] = (tableBuffer[0] & 0x0FFFFFFF)
				| ((wlprio_11bit_code & 0x00F) << 28);
			tableBuffer[1] = (tableBuffer[1] & 0xFFFFFF80)
				| ((wlprio_11bit_code & 0x7F0) >> 4);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tableBuffer, 2, 32, tbl_entry);
		}
	}
}

void
wlc_sslpnphy_CmRxAciGainTbl_Tweaks(void *arg)
{
	phy_info_t *pi = (phy_info_t *)arg;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */

	uint32 tblBuffer[2];
	uint32 lnaBuffer[2] = {0, 9};
	uint8 phybw40 = IS40MHZ(pi), i;

	bool extlna = ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA_5GHz) &&
		CHSPEC_IS5G(pi->radio_chanspec)) ||
		((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA) &&
		CHSPEC_IS2G(pi->radio_chanspec));

#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
			SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_MASK,
			18 << SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_MinPwrLevel,
			SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_MASK,
			162 << SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_SHIFT);
		write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6333);
		mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
			SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
			8 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
			SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
			6 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_crsMiscCtrl2,
			SSLPNPHY_crsMiscCtrl2_eghtSmplFstPwrLogicEn_MASK,
			1 << SSLPNPHY_crsMiscCtrl2_eghtSmplFstPwrLogicEn_SHIFT);

		if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17_SSID) ||
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID)) {
			lnaBuffer[0] = 0xDD;
			lnaBuffer[1] = 0x9;
			mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
				SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
				5 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_gaindirectMismatch,
				SSLPNPHY_gaindirectMismatch_MedHigainDirectMismatch_MASK,
				12 << SSLPNPHY_gaindirectMismatch_MedHigainDirectMismatch_SHIFT);
			write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6363);

			for (i = 0; i <= 28; i++) {
				wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
					ELNA_OLYMPIC_OFDM_5G_GAINTBL_TWEAKS,
					ELNA_OLYMPIC_OFDM_5G_GAINTBL_TWEAKS_sz);
			}
			if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
				write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6666);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
					SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_MASK,
					15 << SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_gainMismatch,
					SSLPNPHY_gainMismatch_GainmisMatchPktRx_MASK,
					9 << SSLPNPHY_gainMismatch_GainmisMatchPktRx_SHIFT);
				for (i = 18; i <= 26; i++) {
					wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
						ELNA_OLYMPIC_OFDM_5G_GAINTBL_TSMC_TWEAKS,
						ELNA_OLYMPIC_OFDM_5G_GAINTBL_TSMC_TWEAKS_sz);
				}
			}
		}
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID) {
			mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
				SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
				9 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
				SSLPNPHY_crsgainCtrl_phycrsctrl_MASK,
				0xf << SSLPNPHY_crsgainCtrl_phycrsctrl_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
				SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
				0x14 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
			SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
			0 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_MinPwrLevel,
				SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_MASK,
				0xa4 << SSLPNPHY_MinPwrLevel_ofdmMinPwrLevel_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
		}
	}
#endif /* BAND5G */
	if ((SSLPNREV_IS(pi->pubpi.phy_rev, 1)) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		mod_phy_reg(pi, SSLPNPHY_DSSSConfirmCnt,
			SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_MASK,
			4 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
			SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
			11 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
			SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
			6 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);
		write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6666);
		write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 360);
		if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
			write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0x33);
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 340);
		}

		if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
			if ((sslpnphy_specific->sslpnphy_fabid != 2) && (sslpnphy_specific->sslpnphy_fabid_otp != TSMC_FAB12)) {
				/* write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0xF6); */
				mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
					SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
					8 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
				/* write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 330); */
			}
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID) {
				mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
					SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
					12 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
					write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 12);
			}
		} else if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA) {
			mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs2,
				SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_MASK |
				SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_MASK,
				6 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_SHIFT |
				6 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_LowGainDB,
				SSLPNPHY_LowGainDB_MedLowGainDB_MASK,
				32 << SSLPNPHY_LowGainDB_MedLowGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
				SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
				18 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
			write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 12);

			/* Rx ELNA Boards ACI Improvements W/o uCode Interventions */
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94329OLYMPICN18_SSID) {
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94329OLYMPICLOCO_SSID) &&
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94329OLYMPICUNO_SSID)) {
					write_phy_reg(pi, SSLPNPHY_ClipThresh, 63);
					mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
						SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
						4 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT);
				} else {
					mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
						SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
						7 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_gainidxoffset,
						SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_MASK,
						247 << SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs2,
						SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_MASK,
						9 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_SHIFT);
					for (i = 39; i <= 46; i++) {
						wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
							UNO_LOCO_ELNA_CCK_GAINTBL_TWEAKS,
							UNO_LOCO_ELNA_CCK_GAINTBL_TWEAKS_sz);
					}
				}
				mod_phy_reg(pi, SSLPNPHY_DSSSConfirmCnt,
					SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_MASK,
					2 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_SHIFT);
				write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 20);
				mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
					SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
					12 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
			}
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) {
				/* Gain idx tweaking for 2g band of dual band board */
				tblBuffer[0] = 0xc0000001;
				tblBuffer[1] = 0x0000006c;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 14);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 88);
				tblBuffer[0] = 0x70000002;
				tblBuffer[1] = 0x0000006a;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 16);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 90);
				tblBuffer[0] = 0x20000002;
				tblBuffer[1] = 0x0000006b;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 18);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 92);
				tblBuffer[0] = 0xc020c287;
				tblBuffer[1] = 0x0000002c;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 30);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 104);
				tblBuffer[0] = 0x30410308;
				tblBuffer[1] = 0x0000002b;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 32);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 106);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs2,
					SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_MASK |
					SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_MASK,
					9 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtOfdm_SHIFT |
					9 << SSLPNPHY_radioTRCtrlCrs2_trTransAddrLmtDsss_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
					SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
					22 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
					SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
					120 << SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
					SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_MASK,
					18 << SSLPNPHY_ClipCtrThresh_ClipCtrThreshHiGain_SHIFT);
				tblBuffer[0] = 0x51e64d96;
				tblBuffer[1] = 0x0000003c;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 54);
				tblBuffer[0] = 0x6204ca9e;
				tblBuffer[1] = 0x0000003c;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 56);
				for (i = 50; i <= 73; i++) {
					wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
						ELNA_CCK_ACI_GAINTBL_TWEAKS,
						ELNA_CCK_ACI_GAINTBL_TWEAKS_sz);
				}
			} else if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID)) {
				lnaBuffer[0] = 0;
				lnaBuffer[1] = 12;

				for (i = 23; i <= 33; i++) {
					wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
						ELNA_OLYMPIC_OFDM_ACI_GAINTBL_TWEAKS,
						ELNA_OLYMPIC_OFDM_ACI_GAINTBL_TWEAKS_sz);
				}
				for (i = 61; i <= 71; i++) {
					wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
						ELNA_OLYMPIC_CCK_ACI_GAINTBL_TWEAKS,
						ELNA_OLYMPIC_CCK_ACI_GAINTBL_TWEAKS_sz);
				}
				mod_phy_reg(pi, SSLPNPHY_HiGainDB,
					SSLPNPHY_HiGainDB_HiGainDB_MASK,
					73 << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 360);
			mod_phy_reg(pi, SSLPNPHY_gainMismatchMedGainEx,
			SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_MASK,
			3 << SSLPNPHY_gainMismatchMedGainEx_medHiGainDirectMismatchOFDMDet_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
					SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_MASK,
					0 << SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrl,
					SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_MASK,
					0 << SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
					SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_MASK,
					15 << SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_gainMismatch,
					SSLPNPHY_gainMismatch_GainmisMatchPktRx_MASK,
					9 << SSLPNPHY_gainMismatch_GainmisMatchPktRx_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
					SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
					8 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
					SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
					15 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
					SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
					9 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT);
				/* if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
					(sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
				tblBuffer[0] = 0x651123C7;
				tblBuffer[1] = 0x00000008;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 30);
				}
				*/
			}
		}
	} else if ((SSLPNREV_GE(pi->pubpi.phy_rev, 2)) &&
		(CHSPEC_IS2G(pi->radio_chanspec))) {
		if (!phybw40) {
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 360);
			write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0xF0); /* Dflt Value */
			mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
				SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_MASK,
				0 << SSLPNPHY_radioTRCtrlCrs1_gainReqTrAttOnEnByCrs_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_radioTRCtrl,
				SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_MASK,
				0 << SSLPNPHY_radioTRCtrl_gainrequestTRAttnOnEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
				SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
				11 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_DSSSConfirmCnt,
				SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGain_MASK |
				SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_MASK |
				SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGainCnfrm_MASK,
				((4 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGain_SHIFT) |
				(4 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntLoGain_SHIFT) |
				(2 << SSLPNPHY_DSSSConfirmCnt_DSSSConfirmCntHiGainCnfrm_SHIFT)));
			mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
				SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
				20 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
				SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
				9 << SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_lnsrOfParam4,
				SSLPNPHY_lnsrOfParam4_ofMaxPThrUpdtThresh_MASK |
				SSLPNPHY_lnsrOfParam4_oFiltSyncCtrShft_MASK,
				((0 << SSLPNPHY_lnsrOfParam4_ofMaxPThrUpdtThresh_SHIFT) |
				(2 << SSLPNPHY_lnsrOfParam4_oFiltSyncCtrShft_SHIFT)));

			mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
				SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
				120 << SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
		}
		else if (phybw40) {
		        write_phy_reg(pi, SSLPNPHY_Rev2_nfSubtractVal_40, 320);
			write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0xF6);
		}
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94319WLUSBN4L_SSID) {
			tblBuffer[0] = 0x0110;
			tblBuffer[1] = 0x0101;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SW_CTRL,
				tblBuffer, 2, 32, 0);

			tblBuffer[0] = 0xb0000000;
			tblBuffer[1] = 0x00000040;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 0);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 74);
			tblBuffer[0] = 0x00000000;
			tblBuffer[1] = 0x00000048;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 2);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 76);
			tblBuffer[0] = 0xb0000000;
			tblBuffer[1] = 0x00000048;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 4);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 78);
			tblBuffer[0] = 0xe0000000;
			tblBuffer[1] = 0x0000004d;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 6);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 80);
			tblBuffer[0] = 0xc0000000;
			tblBuffer[1] = 0x0000004c;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 8);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 82);
			tblBuffer[0] = 0x00000000;
			tblBuffer[1] = 0x00000058;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tblBuffer, 2, 32, 10);
			if (phybw40) {
				tblBuffer[0] = 0xb0000000;
				tblBuffer[1] = 0x00000058;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 12);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tblBuffer, 2, 32, 86);
			}
			mod_phy_reg(pi, SSLPNPHY_ClipCtrThresh,
				SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_MASK,
				12 << SSLPNPHY_ClipCtrThresh_clipCtrThreshLoGain_SHIFT);
		}
	}
	if (extlna) {
		mod_phy_reg(pi, SSLPNPHY_radioCtrl,
			SSLPNPHY_radioCtrl_extlnaen_MASK,
			1 << SSLPNPHY_radioCtrl_extlnaen_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_extlnagainvalue0,
			SSLPNPHY_extlnagainvalue0_extlnagain0_MASK,
			lnaBuffer[0] << SSLPNPHY_extlnagainvalue0_extlnagain0_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_extlnagainvalue0,
			SSLPNPHY_extlnagainvalue0_extlnagain1_MASK,
			lnaBuffer[1] << SSLPNPHY_extlnagainvalue0_extlnagain1_SHIFT);
		wlc_sslpnphy_common_write_table(pi, 17, lnaBuffer, 2, 32, 66);
	}
#ifdef SSLPNLOWPOWER
	write_radio_reg(pi, RADIO_2063_GRX_1ST_1, 0x0f);
#endif
	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);
}

static void
wlc_sslpnphy_set_dac_gain(phy_info_t *pi, uint16 dac_gain)
{
	uint16 dac_ctrl;

	dac_ctrl = (read_phy_reg(pi, SSLPNPHY_AfeDACCtrl) >> SSLPNPHY_AfeDACCtrl_dac_ctrl_SHIFT);
	dac_ctrl = dac_ctrl & 0xc7f;
	dac_ctrl = dac_ctrl | (dac_gain << 7);
	mod_phy_reg(pi, SSLPNPHY_AfeDACCtrl,
		SSLPNPHY_AfeDACCtrl_dac_ctrl_MASK,
		dac_ctrl << SSLPNPHY_AfeDACCtrl_dac_ctrl_SHIFT);
}

STATIC void
wlc_sslpnphy_set_tx_gain_override(phy_info_t *pi, bool bEnable)
{
	uint16 bit = bEnable ? 1 : 0;

	mod_phy_reg(pi, SSLPNPHY_rfoverride2,
		SSLPNPHY_rfoverride2_txgainctrl_ovr_MASK,
		bit << SSLPNPHY_rfoverride2_txgainctrl_ovr_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_rfoverride2,
		SSLPNPHY_rfoverride2_stxtxgainctrl_ovr_MASK,
		bit << SSLPNPHY_rfoverride2_stxtxgainctrl_ovr_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_dacattctrl_ovr_MASK,
		bit << SSLPNPHY_AfeCtrlOvr_dacattctrl_ovr_SHIFT);
}

STATIC uint16
wlc_sslpnphy_get_pa_gain(phy_info_t *pi)
{
	uint16 pa_gain;

	pa_gain = (read_phy_reg(pi, SSLPNPHY_txgainctrlovrval1) &
		SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_MASK) >>
		SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT;

	return pa_gain;
}

STATIC void
wlc_sslpnphy_set_tx_gain(phy_info_t *pi,  sslpnphy_txgains_t *target_gains)
{
	uint16 pa_gain = wlc_sslpnphy_get_pa_gain(pi);

	mod_phy_reg(pi, SSLPNPHY_txgainctrlovrval0,
		SSLPNPHY_txgainctrlovrval0_txgainctrl_ovr_val0_MASK,
		((target_gains->gm_gain) | (target_gains->pga_gain << 8)) <<
		SSLPNPHY_txgainctrlovrval0_txgainctrl_ovr_val0_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_txgainctrlovrval1,
		SSLPNPHY_txgainctrlovrval1_txgainctrl_ovr_val1_MASK,
		((target_gains->pad_gain) | (pa_gain << 8)) <<
		SSLPNPHY_txgainctrlovrval1_txgainctrl_ovr_val1_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_stxtxgainctrlovrval0,
		SSLPNPHY_stxtxgainctrlovrval0_stxtxgainctrl_ovr_val0_MASK,
		((target_gains->gm_gain) | (target_gains->pga_gain << 8)) <<
		SSLPNPHY_stxtxgainctrlovrval0_stxtxgainctrl_ovr_val0_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_stxtxgainctrlovrval1,
		SSLPNPHY_stxtxgainctrlovrval1_stxtxgainctrl_ovr_val1_MASK,
		((target_gains->pad_gain) | (pa_gain << 8)) <<
		SSLPNPHY_stxtxgainctrlovrval1_stxtxgainctrl_ovr_val1_SHIFT);

	wlc_sslpnphy_set_dac_gain(pi, target_gains->dac_gain);

	/* Enable gain overrides */
	wlc_sslpnphy_enable_tx_gain_override(pi);
}

STATIC void
wlc_sslpnphy_set_bbmult(phy_info_t *pi, uint8 m0)
{
	uint16 m0m1 = (uint16)m0 << 8;
	phytbl_info_t tab;

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	tab.tbl_ptr = &m0m1; /* ptr to buf */
	tab.tbl_len = 1;        /* # values   */
	tab.tbl_id = SSLPNPHY_TBL_ID_IQLOCAL;         /* iqloCaltbl      */
	tab.tbl_offset = 87; /* tbl offset */
	tab.tbl_width = 16;     /* 16 bit wide */
	wlc_sslpnphy_write_table(pi, &tab);
}

void
wlc_sslpnphy_clear_tx_power_offsets(phy_info_t *pi)
{
	uint32 data_buf[64];
	phytbl_info_t tab;

	/* Clear out buffer */
	bzero(data_buf, sizeof(data_buf));

	/* Preset txPwrCtrltbl */
	tab.tbl_id = SSLPNPHY_TBL_ID_TXPWRCTL;
	tab.tbl_width = 32;	/* 32 bit wide	*/
	tab.tbl_ptr = data_buf; /* ptr to buf */

	/* Per rate power offset */
	tab.tbl_len = 24; /* # values   */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_RATE_OFFSET;
	wlc_sslpnphy_write_table(pi, &tab);

	/* Per index power offset */
	tab.tbl_len = 64; /* # values   */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_MAC_OFFSET;
	wlc_sslpnphy_write_table(pi, &tab);
}


static void
wlc_sslpnphy_set_tssi_mux(phy_info_t *pi, sslpnphy_tssi_mode_t pos)
{
	if (SSLPNPHY_TSSI_EXT == pos) {
		and_phy_reg(pi, SSLPNPHY_extstxctrl1, 0x0 << 12);
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			write_radio_reg(pi, RADIO_2063_EXTTSSI_CTRL_2, 0x20);
		} else {
			write_radio_reg(pi, RADIO_2063_EXTTSSI_CTRL_2, 0x21);
		}
		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
			SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_MASK,
			0x01 << SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
			SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
			0x02 << SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);
			write_radio_reg(pi, RADIO_2063_EXTTSSI_CTRL_1, 0x51);

	} else {
		/* Power up internal TSSI */
		or_phy_reg(pi, SSLPNPHY_extstxctrl1, 0x01 << 12);
#ifdef BAND5G
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			mod_radio_reg(pi, RADIO_2063_PA_CTRL_1, 0x1 << 2, 0 << 2);
			write_radio_reg(pi, RADIO_2063_PA_CTRL_10, 0x51);

			or_radio_reg(pi, RADIO_2063_PA_SP_1, 0x2);
			or_radio_reg(pi, RADIO_2063_COMMON_07, 0x10);
		} else
#endif
		{
			mod_radio_reg(pi, RADIO_2063_PA_CTRL_1, 0x1 << 2, 1 << 2);
			write_radio_reg(pi, RADIO_2063_PA_CTRL_10, 0x51);
		}

		/* Set TSSI/RSSI mux */
		if (SSLPNPHY_TSSI_POST_PA == pos) {
			mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
				SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
				0x00 << SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);
		} else {
			mod_radio_reg(pi, RADIO_2063_PA_SP_1, 0x01, 1);
			mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
				SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
				0x04 << SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);
		}
	}
}

#define BTCX_FLUSH_WAIT_MAX_MS	  500

bool
wlc_sslpnphy_btcx_override_enable(phy_info_t *pi)
{
#if !defined(PHYHAL)
	wlc_info_t * wlc_pi = pi->wlc;
	#define wlc_hw (wlc_pi)
#else
	#define wlc_hw (pi->sh) 
#endif

	bool val = TRUE;
	int delay, eci_busy_cnt;
	uint32 eci_m = 0, a2dp;


	if (CHSPEC_IS2G(pi->radio_chanspec) && (wlc_hw->machwcap & MCAP_BTCX)) {
		/* Ucode better be suspended when we mess with BTCX regs directly */
		ASSERT(0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));

		/* Enable manual BTCX mode */
		OR_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->btcx_ctrl, BTCX_CTRL_EN | BTCX_CTRL_SW);

		/* Set BT priority & antenna to allow A2DP to catchup */
		AND_REG(GENERIC_PHY_INFO(pi)->osh,
			&pi->regs->btcx_trans_ctrl, ~(BTCX_TRANS_TXCONF | BTCX_TRANS_ANTSEL));

		if (BCMECICOEX_ENAB(wlc_pi)) {
			/* Wait for A2DP to flush all pending data */
			W_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->btcx_eci_addr, 3);
			for (delay = 0, eci_busy_cnt = 0;
				delay < BTCX_FLUSH_WAIT_MAX_MS * 10; delay++) {
				/* Make sure ECI update is not in progress */
				if ((eci_m = R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->btcx_eci_data))
					& 0x4000) {
					if (!(a2dp = (eci_m & 0xf))) {
						/* All A2DP data is flushed */
						goto pri_wlan;
					}
					eci_busy_cnt = 0;
				} else {
					if (++eci_busy_cnt > 1)
						goto pri_wlan;
				}
				OSL_DELAY(100);
			}
			if (delay == (BTCX_FLUSH_WAIT_MAX_MS * 10)) {
				WL_ERROR(("wl%d: %s: A2DP flush failed, eci_m: 0x%x\n",
					GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, eci_m));
				val = FALSE;
			}
		} else {
		}

pri_wlan:
		/* Set WLAN priority */
		OR_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->btcx_trans_ctrl, BTCX_TRANS_TXCONF);

		/* Wait for BT activity to finish */
		delay = 0;
		while (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->btcx_stat) & BTCX_STAT_RA) {
			if (delay++ > BTCX_FLUSH_WAIT_MAX_MS) {
				WL_ERROR(("wl%d: %s: BT still active\n",
					GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
				val = FALSE;
				break;
			}
			OSL_DELAY(100);
		}

		/* Set WLAN antenna & priority */
		OR_REG(GENERIC_PHY_INFO(pi)->osh,
			&pi->regs->btcx_trans_ctrl, BTCX_TRANS_ANTSEL | BTCX_TRANS_TXCONF);
	}


	return val;
}

void
wlc_sslpnphy_tx_pwr_update_npt(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (wlc_phy_tpc_isenabled_sslpnphy(pi)) {
		uint16 tx_cnt, tx_total, npt;

		tx_total = wlc_sslpnphy_total_tx_frames(pi);
		tx_cnt = tx_total - sslpnphy_specific->sslpnphy_tssi_tx_cnt;
		npt = sslpnphy_specific->sslpnphy_tssi_npt;

		if (tx_cnt > (1 << npt)) {
			/* Set new NPT */
			if (npt < SSLPNPHY_TX_PWR_CTRL_MAX_NPT) {
				npt++;
				wlc_sslpnphy_set_tx_pwr_npt(pi, npt);
				sslpnphy_specific->sslpnphy_tssi_npt = npt;
			}
			/* Update power index cache */
			sslpnphy_specific->sslpnphy_tssi_idx = wlc_sslpnphy_get_current_tx_pwr_idx(pi);
			sslpnphy_specific->sslpnphy_tssi_idx_ch[wlc_phy_channel2idx(CHSPEC_CHANNEL(pi->radio_chanspec))]
				= (uint8)sslpnphy_specific->sslpnphy_tssi_idx;

			/* Reset frame counter */
			sslpnphy_specific->sslpnphy_tssi_tx_cnt = tx_total;
		}
		WL_INFORM(("wl%d: %s: Index: %d\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, sslpnphy_specific->sslpnphy_tssi_idx));
	}
}

int32
wlc_sslpnphy_tssi2dbm(int32 tssi, int32 a1, int32 b0, int32 b1)
{
	int32 a, b, p;

	a = 32768 + (a1 * tssi);
	b = (512 * b0) + (32 * b1 * tssi);
	p = ((2 * b) + a) / (2 * a);

	if (p > 127)
		p = 127;
	else if (p < -128)
		p = -128;

	return p;
}

static void
wlc_sslpnphy_txpower_reset_npt(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_tssi_tx_cnt = wlc_sslpnphy_total_tx_frames(pi);
	sslpnphy_specific->sslpnphy_tssi_npt = SSLPNPHY_TX_PWR_CTRL_START_NPT;
}

void
wlc_sslpnphy_txpower_recalc_target(phy_info_t *pi)
{
	uint idx;
	int8 mac_pwr;
	uint16 sslpnphy_shm_ptr;
	uint8 plcp_pwr_offset_order[] = {3, 1, 2, 0, 7, 11, 6, 10, 5, 9, 4, 8, 12, 13, 14, 15, 16, 17, 18, 19};
	int target_pwr;
	uint i, rate;
	uint32 rate_table[24] = {0};

	bool ninja_board_flag = (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* The processing for Ninja boards (5GHz band) is different since we are using positive
	   Tx power offsets for the rate based tx pwrctrl. */
	if (ninja_board_flag && (CHSPEC_IS5G(pi->radio_chanspec))) {
		/* Rate Table part of the Tx pwrctrl table has the following entries:
		 * 0  -> CCK
		 * 4  -> BPSK  R=1/2
		 * 5  -> BPSK  R=2/3
		 * 6  -> BPSK  R=3/4
		 * 7  -> BPSK  R=5/6
		 * 8  -> QPSK  R=1/2
		 * 9  -> QPSK  R=2/3
		 * 10 -> QPSK  R=3/4
		 * 11 -> QPSK  R=5/6
		 * 12 -> QAM16 R=1/2
		 * 13 -> QAM16 R=2/3
		 * 14 -> QAM16 R=3/4
		 * 15 -> QAM16 R=5/6
		 * 16 -> QAM64 R=1/2
		 * 17 -> QAM64 R=2/3
		 * 18 -> QAM64 R=3/4
		 * 19 -> QAM64 R=5/6
		 */
		i = (IS40MHZ(pi))? TXP_FIRST_MCS_40 : TXP_FIRST_OFDM;

		for (rate = 0; rate < ARRAYSIZE(rate_table); ) {
			if ((rate < TXP_FIRST_OFDM) || (rate > 19)) {
				rate_table[rate++] = 0;
			} else {
					rate_table[rate++] = pi->tx_power_offset[i];
					rate_table[rate++] = pi->tx_power_offset[i++];
			}

		}
		rate_table[19] = pi->tx_power_offset[TXP_LAST_MCS_SISO_20];

		/*
		for (rate = 0; rate < ARRAYSIZE(rate_table); rate++) {
			printf("rate_table index=%d, offset=%d\n", rate, rate_table[rate]);
		}
		*/

		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL, rate_table,
		    ARRAYSIZE(rate_table), 32, SSLPNPHY_TX_PWR_CTRL_RATE_OFFSET);
		wlc_sslpnphy_set_target_tx_pwr(pi, pi->tx_power_max);

		/* Ninja board processing completes here */
		return;
	}

	sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
	target_pwr = wlc_sslpnphy_validated_tssi_pwr(pi, pi->tx_power_min);

	ASSERT(0 != sslpnphy_shm_ptr);

	/* Fill MAC offset table to support offsets from -8dBm to 7.75dBm relative to the target power */
	for (idx = 0, mac_pwr = 32; idx < 64; idx++) {
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&mac_pwr, 1, 8, SSLPNPHY_TX_PWR_CTRL_MAC_OFFSET + idx);
		mac_pwr--;
	}


	/* Calculate offset for each rate relative to the target power */
	for (idx = 0; idx <= TXP_LAST_MCS_SISO_20; idx++) {
		uint16 ant0_offset, ant1_offset;
		int ant0_pwr, ant1_pwr;

		/* Calculate separate offsets for ant0 and ant1 as they might be different */
		ant0_pwr = (int)pi->tx_power_min + (int)pi->tx_power_offset[idx];
		ant0_pwr = wlc_sslpnphy_validated_tssi_pwr(pi, ant0_pwr);
		ant0_offset = (uint16)MAX(0, MIN(63, (32 + (ant0_pwr - target_pwr))));

		ant1_pwr = MIN(ant0_pwr, (int)sslpnphy_specific->sslpnphy_ant1_max_pwr);
		ant1_pwr = wlc_sslpnphy_validated_tssi_pwr(pi, ant1_pwr);
		ant1_offset = (uint16)MAX(0, MIN(63, (32 + (ant1_pwr - target_pwr))));

		/* In shm upper byte is offset for ant1 and lower byte is offset for ant0 */	
		WL_WRITE_SHM(pi,
			2 * (sslpnphy_shm_ptr + M_SSLPNPHY_TXPWR_BLK + plcp_pwr_offset_order[idx]),
			(ant1_offset << 8) | ant0_offset);
	}


	/* Set new target power */
	wlc_sslpnphy_set_target_tx_pwr(pi, target_pwr);
}

void
wlc_sslpnphy_set_tx_pwr_ctrl(phy_info_t *pi, uint16 mode)
{
	uint16 old_mode = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	ASSERT(
		(SSLPNPHY_TX_PWR_CTRL_OFF == mode) ||
		(SSLPNPHY_TX_PWR_CTRL_SW == mode) ||
		(SSLPNPHY_TX_PWR_CTRL_HW == mode));

	/* Setting txfront end clock also along with hwpwr control */
	MOD_PHY_REG(pi, SSLPNPHY, sslpnCalibClkEnCtrl, txFrontEndCalibClkEn,
		(SSLPNPHY_TX_PWR_CTRL_HW == mode) ? 1 : 0);

	/* Feed back RF power level to PAPD block */
	MOD_PHY_REG(pi, SSLPNPHY, papd_control2, papd_analog_gain_ovr,
		(SSLPNPHY_TX_PWR_CTRL_HW == mode) ? 0 : 1);

	if (old_mode != mode) {
		if (SSLPNPHY_TX_PWR_CTRL_HW == old_mode) {
			/* Clear out all power offsets */
			wlc_sslpnphy_clear_tx_power_offsets(pi);
		} else if (SSLPNPHY_TX_PWR_CTRL_HW == mode) {
			/* Recalculate target power to restore power offsets */
			wlc_sslpnphy_txpower_recalc_target(pi);

			/* Set starting index to the best known value for that target */
			wlc_sslpnphy_set_start_tx_pwr_idx(pi,
				sslpnphy_specific->sslpnphy_tssi_idx_ch[wlc_phy_channel2idx(CHSPEC_CHANNEL(pi->radio_chanspec))]);
			/* Reset NPT */
			wlc_sslpnphy_txpower_reset_npt(pi);
			wlc_sslpnphy_set_tx_pwr_npt(pi, sslpnphy_specific->sslpnphy_tssi_npt);

			/* Disable any gain overrides */
			wlc_sslpnphy_disable_tx_gain_override(pi);
			sslpnphy_specific->sslpnphy_tx_power_idx_override = -1;
		}

		/* Set requested tx power control mode */
		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlCmd,
			(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK |
			SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK |
			SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK),
			mode);

		WL_INFORM(("wl%d: %s: %s \n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
			mode ? ((SSLPNPHY_TX_PWR_CTRL_HW == mode) ? "Auto" : "Manual") : "Off"));
	}
}
OSTATIC void
wlc_sslpnphy_idle_tssi_est(phy_info_t *pi)
{
	uint16 status, tssi_val;
	wl_pkteng_t pkteng;
	struct ether_addr sa;
	wlc_phy_t *ppi = (wlc_phy_t *)pi;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);

	sa.octet[0] = 10;
	sslpnphy_specific->sslpnphy_tssi_val = 0;

	WL_ERROR(("Pkteng TX Start Called\n"));
	pkteng.flags = WL_PKTENG_PER_TX_START;
	if (sslpnphy_specific->sslpnphy_recal)
		pkteng.delay = 2;              /* Inter packet delay */
	else
		pkteng.delay = 50;              /* Inter packet delay */
		pkteng.nframes = 50;            /* number of frames */
	pkteng.length = 0;              /* packet length */
	pkteng.seqno = FALSE;                   /* enable/disable sequence no. */
	wlc_sslpnphy_pktengtx(ppi, &pkteng, 108, &sa, (1000*10));

	status = read_phy_reg(pi, SSLPNPHY_TxPwrCtrlStatus);
	if (status & SSLPNPHY_TxPwrCtrlStatus_estPwrValid_MASK) {
		tssi_val = (status & SSLPNPHY_TxPwrCtrlStatus_estPwr_MASK) >>
			SSLPNPHY_TxPwrCtrlStatus_estPwr_SHIFT;
		WL_INFORM(("wl%d: %s: Measured idle TSSI: %d\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, (int8)tssi_val - 32));
		sslpnphy_specific->sslpnphy_tssi_val = (uint8)tssi_val;
	} else {
		WL_INFORM(("wl%d: %s: Failed to measure idle TSSI\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
		}

	/* Trigger uCode for doing AuxADC measurements */
	WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
		M_SSLPNPHY_TSSICAL_EN)), 0x0);
	wlc_sslpnphy_auxadc_measure((wlc_phy_t *) pi, 0);
}

void
wlc_sslpnphy_recalc_tssi2dbm_tbl(phy_info_t *pi, int32 a1, int32 b0, int32 b1)
{
	int32 tssi, pwr;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* Convert tssi to power LUT */
	for (tssi = 0; tssi < 64; tssi++) {
		pwr = wlc_sslpnphy_tssi2dbm(tssi, a1, b0, b1);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&pwr, 1, 32, tssi);
	}

	/* For max power limit we need to account for idle tssi */
	sslpnphy_specific->sslpnphy_tssi_max_pwr_limit =
		(int8)wlc_sslpnphy_tssi2dbm((64 - sslpnphy_specific->sslpnphy_tssi_val), a1, b0, b1) - 1;
	sslpnphy_specific->sslpnphy_tssi_min_pwr_limit =
		(int8)MIN((8 * 4), (wlc_sslpnphy_tssi2dbm(60, a1, b0, b1) + 1));

	/* Validate against NVRAM limits */
	sslpnphy_specific->sslpnphy_tssi_max_pwr_limit =
		MIN(sslpnphy_specific->sslpnphy_tssi_max_pwr_limit, sslpnphy_specific->sslpnphy_tssi_max_pwr_nvram);
	sslpnphy_specific->sslpnphy_tssi_min_pwr_limit =
		MAX(sslpnphy_specific->sslpnphy_tssi_min_pwr_limit, sslpnphy_specific->sslpnphy_tssi_min_pwr_nvram);


	/* Final sanity check */
	ASSERT(sslpnphy_specific->sslpnphy_tssi_max_pwr_limit > sslpnphy_specific->sslpnphy_tssi_min_pwr_limit);
}

void
wlc_sslpnphy_tx_pwr_ctrl_init(phy_info_t *pi)
{
	sslpnphy_txgains_t tx_gains;
	sslpnphy_txgains_t ltx_gains;
	uint8 bbmult;
	int32 a1, b0, b1;
	uint freq;
	bool suspend;
	uint32 ind;
	uint16 tssi_val = 0;
	uint8 phybw40 = IS40MHZ(pi);
	uint16 tempsense;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	/* Saving tempsense value before starting idle tssi est */
	/* Reset Tempsese to 0 . Idle tssi est expects it to be 0 */
	tempsense = read_phy_reg(pi, SSLPNPHY_TempSenseCorrection);
	write_phy_reg(pi, SSLPNPHY_TempSenseCorrection, 0);

	suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	if (!suspend)
		WL_SUSPEND_MAC_AND_WAIT(pi);
#ifdef PS4319XTRA
	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)
		WL_WRITE_SHM(pi, M_PS4319XTRA, 0);
#endif /* PS4319XTRA */
	a1 = b0 = b1 = 0;
	if (NORADIO_ENAB(pi->pubpi)) {
		wlc_sslpnphy_set_bbmult(pi, 0x30);
		return;
	}

	freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
	if (!pi->hwpwrctrl_capable) {
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			tx_gains.gm_gain = 4;
			tx_gains.pga_gain = 12;
			tx_gains.pad_gain = 12;
			tx_gains.dac_gain = 0;

			bbmult = 150;
		} else {
			tx_gains.gm_gain = 7;
			tx_gains.pga_gain = 15;
			tx_gains.pad_gain = 14;
			tx_gains.dac_gain = 0;

			bbmult = 150;
		}
		wlc_sslpnphy_set_tx_gain(pi, &tx_gains);
		wlc_sslpnphy_set_bbmult(pi, bbmult);
	} else {
		/* Adjust power LUT's */
		if (sslpnphy_specific->sslpnphy_target_tx_freq != (uint16)freq) {
			if (freq < 2500) {
			/* 2.4 GHz */
					b0 = pi->txpa_2g_low_temp[0];
					b1 = pi->txpa_2g_low_temp[1];
					a1 = pi->txpa_2g_low_temp[2];

					write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1,
						((uint16)sslpnphy_specific->sslpnphy_rssi_vf_lowtemp << 0) |
						((uint16)sslpnphy_specific->sslpnphy_rssi_vc_lowtemp << 4) |
						(0x00 << 8) |
						((uint16)sslpnphy_specific->sslpnphy_rssi_gs_lowtemp << 10) |
						(0x01 << 13));
			}
#ifdef BAND5G
			if (CHSPEC_IS5G(pi->radio_chanspec)) {
				if (freq <= 5320) {
					/* 5 GHz low */
					b0 = pi->txpa_5g_low[0];
					b1 = pi->txpa_5g_low[1];
					a1 = pi->txpa_5g_low[2];
				} else if (freq <= 5700) {
					/* 5 GHz medium */
					b0 = pi->txpa_5g_mid[0];
					b1 = pi->txpa_5g_mid[1];
					a1 = pi->txpa_5g_mid[2];
				} else {
					/* 5 GHz high */
					b0 = pi->txpa_5g_hi[0];
					b1 = pi->txpa_5g_hi[1];
					a1 = pi->txpa_5g_hi[2];
				}
				write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1,
					((uint16)sslpnphy_specific->sslpnphy_rssi_5g_vf << 0) |
					((uint16)sslpnphy_specific->sslpnphy_rssi_5g_vc << 4) |
					(0x00 << 8) |
					((uint16)sslpnphy_specific->sslpnphy_rssi_5g_gs << 10) |
					(0x01 << 13));
			}
#endif /* BAND5G */
			/* Save new target frequency */
			sslpnphy_specific->sslpnphy_target_tx_freq = (uint16)freq;
			sslpnphy_specific->sslpnphy_last_tx_freq = (uint16)freq;
		}
		/* Clear out all power offsets */
		wlc_sslpnphy_clear_tx_power_offsets(pi);

		/* Setup estPwrLuts for measuring idle TSSI */
		for (ind = 0; ind < 64; ind++) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
				&ind, 1, 32, ind);
		}

		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlNnum,
			SSLPNPHY_TxPwrCtrlNnum_Ntssi_delay_MASK |
			SSLPNPHY_TxPwrCtrlNnum_Ntssi_intg_log2_MASK |
			SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_MASK,
			((255 << SSLPNPHY_TxPwrCtrlNnum_Ntssi_delay_SHIFT) |
			(5 << SSLPNPHY_TxPwrCtrlNnum_Ntssi_intg_log2_SHIFT) |
			(0 << SSLPNPHY_TxPwrCtrlNnum_Npt_intg_log2_SHIFT)));

		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlIdleTssi,
			SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_MASK,
			0x1f << SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_SHIFT);

		{
			uint8 iqcal_ctrl2;

			mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
				SSLPNPHY_auxadcCtrl_rssifiltEn_MASK |
				SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK |
				SSLPNPHY_auxadcCtrl_txpwrctrlEn_MASK,
				((0 << SSLPNPHY_auxadcCtrl_rssifiltEn_SHIFT) |
				(1 << SSLPNPHY_auxadcCtrl_rssiformatConvEn_SHIFT) |
				(1 << SSLPNPHY_auxadcCtrl_txpwrctrlEn_SHIFT)));

			/* Set IQCAL mux to TSSI */
			iqcal_ctrl2 = (uint8)read_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2);
			iqcal_ctrl2 &= (uint8)~(0x0c);
			iqcal_ctrl2 |= 0x01;
			write_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, iqcal_ctrl2);

			/* Use PA output for TSSI */
			if ((CHSPEC_IS5G(pi->radio_chanspec)) &&
				(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)) {
				mod_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, 0xf, 0x8);
				wlc_sslpnphy_set_tssi_mux(pi, SSLPNPHY_TSSI_EXT);
			} else {
				/* Use PA output for TSSI */
				wlc_sslpnphy_set_tssi_mux(pi, SSLPNPHY_TSSI_POST_PA);
			}

		}
		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlIdleTssi,
			SSLPNPHY_TxPwrCtrlIdleTssi_rawTssiOffsetBinFormat_MASK,
			1 << SSLPNPHY_TxPwrCtrlIdleTssi_rawTssiOffsetBinFormat_SHIFT);

		/* Synch up with tcl */
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_crseddisable_MASK,
			1 << SSLPNPHY_crsgainCtrl_crseddisable_SHIFT);
		/* CCK calculation offset */
		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
			SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
			0 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);

		/* Set starting index & NPT to 0 for idle TSSI measurments */
		wlc_sslpnphy_set_start_tx_pwr_idx(pi, 0);
		wlc_sslpnphy_set_tx_pwr_npt(pi, 0);

		/* Force manual power control */
		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlCmd,
			(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK |
			SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK |
			SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK),
			SSLPNPHY_TX_PWR_CTRL_SW);

		{
			/* Force WLAN antenna */
			wlc_sslpnphy_btcx_override_enable(pi);
			wlc_sslpnphy_set_tx_gain_override(pi, TRUE);
		}
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_internalrftxpu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_MASK,
			0 << SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_SHIFT);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
			b0 = pi->txpa_2g[0];
			b1 = pi->txpa_2g[1];
			a1 = pi->txpa_2g[2];
			if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
				/* save */
				wlc_sslpnphy_get_tx_gain(pi, &ltx_gains);

				/* Idle tssi WAR */
				tx_gains.gm_gain = 0;
				tx_gains.pga_gain = 0;
				tx_gains.pad_gain = 0;
				tx_gains.dac_gain = 0;

				wlc_sslpnphy_set_tx_gain(pi, &tx_gains);
			}

			write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1,
				((uint16)sslpnphy_specific->sslpnphy_rssi_vf << 0) |
				((uint16)sslpnphy_specific->sslpnphy_rssi_vc << 4) |
				(0x00 << 8) |
				((uint16)sslpnphy_specific->sslpnphy_rssi_gs << 10) |
				(0x01 << 13));
			wlc_sslpnphy_idle_tssi_est(pi);

			while ((sslpnphy_specific->sslpnphy_tssi_val > 60) && (sslpnphy_specific->sslpnphy_rssi_vc >= 6)) {
				sslpnphy_specific->sslpnphy_rssi_vc = sslpnphy_specific->sslpnphy_rssi_vc - 1;
				write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1,
					((uint16)sslpnphy_specific->sslpnphy_rssi_vf << 0) |
					((uint16)sslpnphy_specific->sslpnphy_rssi_vc << 4) |
					(0x00 << 8) |
					((uint16)sslpnphy_specific->sslpnphy_rssi_gs << 10) |
					(0x01 << 13));
				wlc_sslpnphy_idle_tssi_est(pi);
			}
	} else if (CHSPEC_IS5G(pi->radio_chanspec)) {
		wlc_sslpnphy_idle_tssi_est(pi);
	}
		tssi_val = sslpnphy_specific->sslpnphy_tssi_val;
		tssi_val -= 32;
			/* Write measured idle TSSI value */
			mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlIdleTssi,
				SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_MASK,
				tssi_val << SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_SHIFT);

		/* Sych up with tcl */
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_crseddisable_MASK,
			0 << SSLPNPHY_crsgainCtrl_crseddisable_SHIFT);

		/* Clear tx PU override */
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK,
			0 << SSLPNPHY_RFOverride0_internalrftxpu_ovr_SHIFT);
		/* Invalidate target frequency */
		sslpnphy_specific->sslpnphy_target_tx_freq = 0;

		/* CCK calculation offset */
		if (IS_OLYMPIC(pi)) {
			/* TSMC requires 0.5 dB lower power due to its RF tuning */
			if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12))
				mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
					SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
					7 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);
			else
				mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
					SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
					6 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);
		}
		else if (SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {
			if (phybw40)
				mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
					SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
					2 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);
			else
				mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
					SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
					1 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);
		}

		else
			mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
				SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_MASK,
				3 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_cckPwrOffset_SHIFT);

		/* Restore back Tempsense */
		write_phy_reg(pi, SSLPNPHY_TempSenseCorrection, tempsense);

		/* Program TSSI lookup table */
		wlc_sslpnphy_recalc_tssi2dbm_tbl(pi, a1, b0, b1);

		/* Initialize default NPT */
		wlc_sslpnphy_txpower_reset_npt(pi);

		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			wlc_sslpnphy_set_tx_gain(pi, &ltx_gains);
		}
		/* Enable hardware power control */
		wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_HW);
	}

	if (!suspend)
		WL_ENABLE_MAC(pi);
#ifdef PS4319XTRA
	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)
		WL_WRITE_SHM(pi, M_PS4319XTRA, PS4319XTRA);
#endif /* PS4319XTRA */
}


STATIC uint8
wlc_sslpnphy_get_bbmult(phy_info_t *pi)
{
	uint16 m0m1;
	phytbl_info_t tab;

	tab.tbl_ptr = &m0m1; /* ptr to buf */
	tab.tbl_len = 1;        /* # values   */
	tab.tbl_id = SSLPNPHY_TBL_ID_IQLOCAL;         /* iqloCaltbl      */
	tab.tbl_offset = 87; /* tbl offset */
	tab.tbl_width = 16;     /* 16 bit wide */
	wlc_sslpnphy_read_table(pi, &tab);

	return (uint8)((m0m1 & 0xff00) >> 8);
}

STATIC void
wlc_sslpnphy_set_pa_gain(phy_info_t *pi, uint16 gain)
{
	mod_phy_reg(pi, SSLPNPHY_txgainctrlovrval1,
		SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_MASK,
		gain << SSLPNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_stxtxgainctrlovrval1,
		SSLPNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_MASK,
		gain << SSLPNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_SHIFT);

}
OSTATIC uint16
sslpnphy_iqlocc_write(phy_info_t *pi, uint8 data)
{
	int32 data32 = (int8)data;
	int32 rf_data32;
	int32 ip, in;
	ip = 8 + (data32 >> 1);
	in = 8 - ((data32+1) >> 1);
	rf_data32 = (in << 4) | ip;
	return (uint16)(rf_data32);
}
STATIC void
wlc_sslpnphy_set_radio_loft(phy_info_t *pi,
	uint8 ei0,
	uint8 eq0,
	uint8 fi0,
	uint8 fq0)
{
	write_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_BB_I, sslpnphy_iqlocc_write(pi, ei0));
	write_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_BB_Q, sslpnphy_iqlocc_write(pi, eq0));
	write_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_RF_I, sslpnphy_iqlocc_write(pi, fi0));
	write_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_RF_Q, sslpnphy_iqlocc_write(pi, fq0));

}
void
wlc_sslpnphy_get_radio_loft(phy_info_t *pi,
	uint8 *ei0,
	uint8 *eq0,
	uint8 *fi0,
	uint8 *fq0)
{
	*ei0 = LPPHY_IQLOCC_READ(
		read_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_BB_I));
	*eq0 = LPPHY_IQLOCC_READ(
		read_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_BB_Q));
	*fi0 = LPPHY_IQLOCC_READ(
		read_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_RF_I));
	*fq0 = LPPHY_IQLOCC_READ(
		read_radio_reg(pi, RADIO_2063_TXRF_IDAC_LO_RF_Q));
}

STATIC void
wlc_sslpnphy_get_tx_gain(phy_info_t *pi,  sslpnphy_txgains_t *gains)
{
	uint16 dac_gain;

	dac_gain = read_phy_reg(pi, SSLPNPHY_AfeDACCtrl) >>
		SSLPNPHY_AfeDACCtrl_dac_ctrl_SHIFT;
	gains->dac_gain = (dac_gain & 0x380) >> 7;

	{
		uint16 rfgain0, rfgain1;

		rfgain0 = (read_phy_reg(pi, SSLPNPHY_txgainctrlovrval0) &
			SSLPNPHY_txgainctrlovrval0_txgainctrl_ovr_val0_MASK) >>
			SSLPNPHY_txgainctrlovrval0_txgainctrl_ovr_val0_SHIFT;
		rfgain1 = (read_phy_reg(pi, SSLPNPHY_txgainctrlovrval1) &
			SSLPNPHY_txgainctrlovrval1_txgainctrl_ovr_val1_MASK) >>
			SSLPNPHY_txgainctrlovrval1_txgainctrl_ovr_val1_SHIFT;

		gains->gm_gain = rfgain0 & 0xff;
		gains->pga_gain = (rfgain0 >> 8) & 0xff;
		gains->pad_gain = rfgain1 & 0xff;
	}
}

void
wlc_sslpnphy_set_tx_iqcc(phy_info_t *pi, uint16 a, uint16 b)
{
	uint16 iqcc[2];

	/* Fill buffer with coeffs */
	iqcc[0] = a;
	iqcc[1] = b;

	/* Update iqloCaltbl */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		iqcc, 2, 16, 80);
}

#ifdef WLSINGLE_ANT

#define wlc_phy_get_txant 0
#define wlc_sslpnphy_set_ant_override(pi, ant) 0
#define wlc_sslpnphy_restore_ant_override(pi, ant_ovr) do {} while (0)

#else
static  uint16
wlc_phy_get_txant(phy_info_t *pi)
{
#ifdef PHYHAL
	return wlapi_bmac_get_txant(pi->sh->physhim);
#else
	wlc_info_t *wlc = (wlc_info_t *)pi->wlc;

	return ((wlc->txant == ANT_TX_FORCE_1) ? 1 : 0);
#endif /* PHYHAL */
}

static uint32
wlc_sslpnphy_set_ant_override(phy_info_t *pi, uint16 ant)
{
	uint16 val, ovr;
	uint32 ret;

	ASSERT(ant < 2);

	/* Save original values */
	val = read_phy_reg(pi, SSLPNPHY_RFOverrideVal0);
	ovr = read_phy_reg(pi, SSLPNPHY_RFOverride0);
	ret = ((uint32)ovr << 16) | val;

	/* Write new values */
	val &= ~SSLPNPHY_RFOverrideVal0_ant_selp_ovr_val_MASK;
	val |= (ant << SSLPNPHY_RFOverrideVal0_ant_selp_ovr_val_SHIFT);
	ovr |= SSLPNPHY_RFOverride0_ant_selp_ovr_MASK;
	write_phy_reg(pi, SSLPNPHY_RFOverrideVal0, val);
	write_phy_reg(pi,  SSLPNPHY_RFOverride0, ovr);

	return ret;
}

static void
wlc_sslpnphy_restore_ant_override(phy_info_t *pi, uint32 ant_ovr)
{
	uint16 ovr, val;

	ovr = (uint16)(ant_ovr >> 16);
	val = (uint16)(ant_ovr & 0xFFFF);

	mod_phy_reg(pi,
		SSLPNPHY_RFOverrideVal0,
		SSLPNPHY_RFOverrideVal0_ant_selp_ovr_val_MASK,
		val);
	mod_phy_reg(pi,
		SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_ant_selp_ovr_MASK,
		ovr);
}

#endif /* WLSINGLE_ANT */

void
wlc_sslpnphy_set_tx_locc(phy_info_t *pi, uint16 didq)
{
	phytbl_info_t tab;

	/* Update iqloCaltbl */
	tab.tbl_id = SSLPNPHY_TBL_ID_IQLOCAL;			/* iqloCaltbl	*/
	tab.tbl_width = 16;	/* 16 bit wide	*/
	tab.tbl_ptr = &didq;
	tab.tbl_len = 1;
	tab.tbl_offset = 85;
	wlc_sslpnphy_write_table(pi, &tab);
}

#ifdef BAND5G

/* only disable function exists */
OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_disable_pad)(phy_info_t *pi)
{
	sslpnphy_txgains_t current_gain;

	wlc_sslpnphy_get_tx_gain(pi, &current_gain);
	current_gain.pad_gain = 0;
	wlc_sslpnphy_set_tx_gain(pi, &current_gain);
}
#endif /* BAND5G */

void
wlc_sslpnphy_set_tx_pwr_by_index(phy_info_t *pi, int index)
{
	phytbl_info_t tab;
	uint16 a, b;
	uint8 bb_mult;
	uint32 bbmultiqcomp, txgain, locoeffs, rfpower;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */

	ASSERT(index <= SSLPNPHY_MAX_TX_POWER_INDEX);

	/* Save forced index */
	sslpnphy_specific->sslpnphy_tx_power_idx_override = (int8)index;
	sslpnphy_specific->sslpnphy_current_index = (uint8)index;


	/* Preset txPwrCtrltbl */
	tab.tbl_id = SSLPNPHY_TBL_ID_TXPWRCTL;
	tab.tbl_width = 32;	/* 32 bit wide	*/
	tab.tbl_len = 1;        /* # values   */

	/* Turn off automatic power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

	/* Read index based bb_mult, a, b from the table */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + index; /* iqCoefLuts */
	tab.tbl_ptr = &bbmultiqcomp; /* ptr to buf */
	wlc_sslpnphy_read_table(pi,  &tab);

	/* Read index based tx gain from the table */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_GAIN_OFFSET + index; /* gainCtrlLuts */
	tab.tbl_ptr = &txgain; /* ptr to buf */
	wlc_sslpnphy_read_table(pi,  &tab);

	/* Apply tx gain */
	{
		sslpnphy_txgains_t gains;

		gains.gm_gain = (uint16)(txgain & 0xff);
		gains.pga_gain = (uint16)(txgain >> 8) & 0xff;
		gains.pad_gain = (uint16)(txgain >> 16) & 0xff;
		gains.dac_gain = (uint16)(bbmultiqcomp >> 28) & 0x07;

		wlc_sslpnphy_set_tx_gain(pi, &gains);
		wlc_sslpnphy_set_pa_gain(pi,  (uint16)(txgain >> 24) & 0x7f);
	}

	/* Apply bb_mult */
	bb_mult = (uint8)((bbmultiqcomp >> 20) & 0xff);
	wlc_sslpnphy_set_bbmult(pi, bb_mult);

	/* Apply iqcc */
	a = (uint16)((bbmultiqcomp >> 10) & 0x3ff);
	b = (uint16)(bbmultiqcomp & 0x3ff);
	wlc_sslpnphy_set_tx_iqcc(pi, a, b);

	/* Read index based di & dq from the table */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_LO_OFFSET + index; /* loftCoefLuts */
	tab.tbl_ptr = &locoeffs; /* ptr to buf */
	wlc_sslpnphy_read_table(pi,  &tab);

	/* Apply locc */
	wlc_sslpnphy_set_tx_locc(pi, (uint16)locoeffs);

	/* Apply PAPD rf power correction */
	tab.tbl_offset = SSLPNPHY_TX_PWR_CTRL_PWR_OFFSET + index;
	tab.tbl_ptr = &rfpower; /* ptr to buf */
	wlc_sslpnphy_read_table(pi,  &tab);

	MOD_PHY_REG(pi, SSLPNPHY, papd_analog_gain_ovr_val, papd_analog_gain_ovr_val, rfpower * 8);
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		wlc_sslpnphy_set_pa_gain(pi, 116);
		if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)
			wlc_sslpnphy_set_pa_gain(pi, 0x10);
	}
#endif

	/* Enable gain overrides */
	wlc_sslpnphy_enable_tx_gain_override(pi);
}

STATIC void
wlc_sslpnphy_set_trsw_override(phy_info_t *pi, bool tx, bool rx)
{
	/* Set TR switch */
	mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
		SSLPNPHY_RFOverrideVal0_trsw_tx_pu_ovr_val_MASK |
		SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_MASK,
		(tx ? SSLPNPHY_RFOverrideVal0_trsw_tx_pu_ovr_val_MASK : 0) |
		(rx ? SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_MASK : 0));

	/* Enable overrides */
	or_phy_reg(pi, SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_trsw_tx_pu_ovr_MASK |
		SSLPNPHY_RFOverride0_trsw_rx_pu_ovr_MASK);
}

STATIC void
wlc_sslpnphy_set_swctrl_override(phy_info_t *pi, uint8 index)
{
	phytbl_info_t tab;
	uint16 swctrl_val;

	if (index == SWCTRL_OVR_DISABLE)
	{
		write_phy_reg(pi, SSLPNPHY_swctrlOvr, 0);
	} else {
		tab.tbl_id = SSLPNPHY_TBL_ID_SW_CTRL;
		tab.tbl_width = 16;	/* 16 bit wide	*/
		tab.tbl_ptr = &swctrl_val ; /* ptr to buf */
		tab.tbl_len = 1;        /* # values   */
		tab.tbl_offset = index; /* tbl offset */
		wlc_sslpnphy_read_table(pi, &tab);

		write_phy_reg(pi, SSLPNPHY_swctrlOvr, 0xff);
		mod_phy_reg(pi, SSLPNPHY_swctrlOvr_val,
			SSLPNPHY_swctrlOvr_val_swCtrl_p_ovr_val_MASK,
			(swctrl_val & 0xf) << SSLPNPHY_swctrlOvr_val_swCtrl_p_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_swctrlOvr_val,
			SSLPNPHY_swctrlOvr_val_swCtrl_n_ovr_val_MASK,
			((swctrl_val >> 4) & 0xf) << SSLPNPHY_swctrlOvr_val_swCtrl_n_ovr_val_SHIFT);
	}
}

STATIC void
BCMROMOVERLAYFN(1, wlc_sslpnphy_set_rx_gain_by_distribution)(phy_info_t *pi,
	uint16 pga,
	uint16 biq2,
	uint16 pole1,
	uint16 biq1,
	uint16 tia,
	uint16 lna2,
	uint16 lna1)
{
	uint16 gain0_15, gain16_19;

	gain16_19 = pga & 0xf;
	gain0_15 = ((biq2 & 0x1) << 15) |
		((pole1 & 0x3) << 13) |
		((biq1 & 0x3) << 11) |
		((tia & 0x7) << 8) |
		((lna2 & 0x3) << 6) |
		((lna2 & 0x3) << 4) |
		((lna1 & 0x3) << 2) |
		((lna1 & 0x3) << 0);

	mod_phy_reg(pi, SSLPNPHY_rxgainctrl0ovrval,
		SSLPNPHY_rxgainctrl0ovrval_rxgainctrl_ovr_val0_MASK,
		gain0_15 << SSLPNPHY_rxgainctrl0ovrval_rxgainctrl_ovr_val0_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rxlnaandgainctrl1ovrval,
		SSLPNPHY_rxlnaandgainctrl1ovrval_rxgainctrl_ovr_val1_MASK,
		gain16_19 << SSLPNPHY_rxlnaandgainctrl1ovrval_rxgainctrl_ovr_val1_SHIFT);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
			SSLPNPHY_rfoverride2val_slna_gain_ctrl_ovr_val_MASK,
			lna1 << SSLPNPHY_rfoverride2val_slna_gain_ctrl_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverrideVal,
			SSLPNPHY_RFinputOverrideVal_wlslnagainctrl_ovr_val_MASK,
			lna1 << SSLPNPHY_RFinputOverrideVal_wlslnagainctrl_ovr_val_SHIFT);
	}
}

STATIC void
wlc_sslpnphy_rx_gain_override_enable(phy_info_t *pi, bool enable)
{
	uint16 ebit = enable ? 1 : 0;

	mod_phy_reg(pi, SSLPNPHY_rfoverride2,
		SSLPNPHY_rfoverride2_rxgainctrl_ovr_MASK |
		SSLPNPHY_rfoverride2_gmode_ext_lna_gain_ovr_MASK |
		SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_MASK,
		((ebit << SSLPNPHY_rfoverride2_rxgainctrl_ovr_SHIFT) |
		(ebit << SSLPNPHY_rfoverride2_gmode_ext_lna_gain_ovr_SHIFT) |
		(ebit << SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_SHIFT)));
	mod_phy_reg(pi, SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_trsw_rx_pu_ovr_MASK |
		SSLPNPHY_RFOverride0_gmode_rx_pu_ovr_MASK |
		SSLPNPHY_RFOverride0_amode_rx_pu_ovr_MASK,
		((ebit << SSLPNPHY_RFOverride0_trsw_rx_pu_ovr_SHIFT) |
		(ebit << SSLPNPHY_RFOverride0_gmode_rx_pu_ovr_SHIFT) |
		(ebit << SSLPNPHY_RFOverride0_amode_rx_pu_ovr_SHIFT)));

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_slna_gain_ctrl_ovr_MASK,
			ebit << SSLPNPHY_rfoverride2_slna_gain_ctrl_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverride,
			SSLPNPHY_RFinputOverride_wlslnagainctrl_ovr_MASK,
			ebit << SSLPNPHY_RFinputOverride_wlslnagainctrl_ovr_SHIFT);
	}
}

STATIC void
wlc_sslpnphy_rx_pu(phy_info_t *pi, bool bEnable)
{
	if (!bEnable) {
		and_phy_reg(pi, SSLPNPHY_RFOverride0,
			~(uint16)(SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK));
		and_phy_reg(pi, SSLPNPHY_rfoverride2,
			~(uint16)(SSLPNPHY_rfoverride2_rxgainctrl_ovr_MASK));
		wlc_sslpnphy_rx_gain_override_enable(pi, FALSE);
	} else {
		/* Force on the transmit chain */
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_internalrfrxpu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_internalrfrxpu_ovr_val_MASK,
			1 << SSLPNPHY_RFOverrideVal0_internalrfrxpu_ovr_val_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_rxgainctrl_ovr_MASK,
			1 << SSLPNPHY_rfoverride2_rxgainctrl_ovr_SHIFT);

		wlc_sslpnphy_set_rx_gain_by_distribution(pi, 15, 1, 3, 3, 7, 3, 3);
		wlc_sslpnphy_rx_gain_override_enable(pi, TRUE);
	}
}

void
BCMROMOVERLAYFN(1, wlc_sslpnphy_tx_pu)(phy_info_t *pi, bool bEnable)
{
	if (!bEnable) {
		/* Clear overrides */
		and_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
			~(uint16)(SSLPNPHY_AfeCtrlOvr_pwdn_dac_ovr_MASK |
			SSLPNPHY_AfeCtrlOvr_dac_clk_disable_ovr_MASK));

		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
			SSLPNPHY_AfeCtrlOvrVal_pwdn_dac_ovr_val_MASK,
			1 << SSLPNPHY_AfeCtrlOvrVal_pwdn_dac_ovr_val_SHIFT);

		and_phy_reg(pi, SSLPNPHY_RFOverride0,
			~(uint16)(SSLPNPHY_RFOverride0_gmode_tx_pu_ovr_MASK |
			SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK |
			SSLPNPHY_RFOverride0_trsw_rx_pu_ovr_MASK |
			SSLPNPHY_RFOverride0_trsw_tx_pu_ovr_MASK |
			SSLPNPHY_RFOverride0_ant_selp_ovr_MASK));
		/* Switch off A band PA ( ePA) */
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
			0 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT);

		and_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			~(uint16)(SSLPNPHY_RFOverrideVal0_gmode_tx_pu_ovr_val_MASK |
			SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_MASK));
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_ant_selp_ovr_val_MASK,
			1 << SSLPNPHY_RFOverrideVal0_ant_selp_ovr_val_SHIFT);

			/* Set TR switch */
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_trsw_tx_pu_ovr_val_MASK |
			SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_MASK,
			SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_MASK);
		and_phy_reg(pi, SSLPNPHY_rfoverride3,
			~(uint16)(SSLPNPHY_rfoverride3_stxpapu_ovr_MASK |
			SSLPNPHY_rfoverride3_stxpadpu2g_ovr_MASK |
			SSLPNPHY_rfoverride3_stxpapu2g_ovr_MASK));

		and_phy_reg(pi, SSLPNPHY_rfoverride3_val,
			~(uint16)(SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_MASK |
			SSLPNPHY_rfoverride3_val_stxpadpu2g_ovr_val_MASK |
			SSLPNPHY_rfoverride3_val_stxpapu2g_ovr_val_MASK));
	} else {
		uint32 ant_ovr;

		/* Force on DAC */
		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
			SSLPNPHY_AfeCtrlOvr_pwdn_dac_ovr_MASK |
			SSLPNPHY_AfeCtrlOvr_dac_clk_disable_ovr_MASK,
			((1 << SSLPNPHY_AfeCtrlOvr_pwdn_dac_ovr_SHIFT) |
			(1 << SSLPNPHY_AfeCtrlOvr_dac_clk_disable_ovr_SHIFT)));
		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
			SSLPNPHY_AfeCtrlOvrVal_pwdn_dac_ovr_val_MASK |
			SSLPNPHY_AfeCtrlOvrVal_dac_clk_disable_ovr_val_MASK,
			((0 << SSLPNPHY_AfeCtrlOvrVal_pwdn_dac_ovr_val_SHIFT) |
			(0 << SSLPNPHY_AfeCtrlOvrVal_dac_clk_disable_ovr_val_SHIFT)));

		/* Force on the transmit chain */
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_internalrftxpu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_MASK,
			1 << SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_SHIFT);


		/* Force the TR switch to transmit */
		wlc_sslpnphy_set_trsw_override(pi, TRUE, FALSE);

		/* Force default antenna */
		ant_ovr = wlc_sslpnphy_set_ant_override(pi, wlc_phy_get_txant(pi));

		/* PAD PU */ /* PGA PU */ /* PA PU */
		mod_phy_reg(pi, SSLPNPHY_rfoverride3,
			SSLPNPHY_rfoverride3_stxpadpu2g_ovr_MASK |
			SSLPNPHY_rfoverride3_stxpapu2g_ovr_MASK |
			SSLPNPHY_rfoverride3_stxpapu_ovr_MASK,
			((1 << SSLPNPHY_rfoverride3_stxpadpu2g_ovr_SHIFT) |
			(1 << SSLPNPHY_rfoverride3_stxpapu2g_ovr_SHIFT) |
			(1 << SSLPNPHY_rfoverride3_stxpapu_ovr_SHIFT)));

		/* PAD PU */ /* PGA PU */ /* PA PU */
		mod_phy_reg(pi, SSLPNPHY_rfoverride3_val,
			SSLPNPHY_rfoverride3_val_stxpadpu2g_ovr_val_MASK |
			SSLPNPHY_rfoverride3_val_stxpapu2g_ovr_val_MASK |
			SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_MASK,
			(((CHSPEC_IS2G(pi->radio_chanspec) ? 1 : 0) <<
			SSLPNPHY_rfoverride3_val_stxpadpu2g_ovr_val_SHIFT) |
			((CHSPEC_IS2G(pi->radio_chanspec) ? 1 : 0) <<
			SSLPNPHY_rfoverride3_val_stxpapu2g_ovr_val_SHIFT) |
			((CHSPEC_IS2G(pi->radio_chanspec) ? 1 : 0) <<
			SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_SHIFT)));

		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			/* Switch on PA for g band */
			mod_phy_reg(pi, SSLPNPHY_RFOverride0,
				SSLPNPHY_RFOverride0_gmode_tx_pu_ovr_MASK,
				1 << SSLPNPHY_RFOverride0_gmode_tx_pu_ovr_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
				SSLPNPHY_RFOverrideVal0_gmode_tx_pu_ovr_val_MASK,
				1 << SSLPNPHY_RFOverrideVal0_gmode_tx_pu_ovr_val_SHIFT);
		} else {
			/* Switch on A band PA ( ePA) */
			mod_phy_reg(pi, SSLPNPHY_RFOverride0,
				SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
				1 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
				SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_MASK,
				1  << SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_SHIFT);
		}
	}
}

/*
 * Play samples from sample play buffer
 */
OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_run_samples)(phy_info_t *pi,
                         uint16 num_samps,
                         uint16 num_loops,
                         uint16 wait,
                         bool iqcalmode)
{
	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_forceaphytxFeclkOn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_forceaphytxFeclkOn_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sampleDepthCount,
		SSLPNPHY_sampleDepthCount_DepthCount_MASK,
		(num_samps - 1) << SSLPNPHY_sampleDepthCount_DepthCount_SHIFT);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
			SSLPNPHY_sslpnCalibClkEnCtrl_papdTxBbmultPapdifClkEn_MASK,
			1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxBbmultPapdifClkEn_SHIFT);
	}

	if (num_loops != 0xffff)
		num_loops--;
	mod_phy_reg(pi, SSLPNPHY_sampleLoopCount,
		SSLPNPHY_sampleLoopCount_LoopCount_MASK,
		num_loops << SSLPNPHY_sampleLoopCount_LoopCount_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sampleInitWaitCount,
		SSLPNPHY_sampleInitWaitCount_InitWaitCount_MASK,
		wait << SSLPNPHY_sampleInitWaitCount_InitWaitCount_SHIFT);

	if (iqcalmode) {
		/* Enable calibration */
		and_phy_reg(pi,
			SSLPNPHY_iqloCalCmdGctl,
			(uint16)~SSLPNPHY_iqloCalCmdGctl_iqlo_cal_en_MASK);
		or_phy_reg(pi, SSLPNPHY_iqloCalCmdGctl, SSLPNPHY_iqloCalCmdGctl_iqlo_cal_en_MASK);
	} else {
		write_phy_reg(pi, SSLPNPHY_sampleCmd, 1);
		wlc_sslpnphy_tx_pu(pi, 1);
	}
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_detection_disable)(phy_info_t *pi, bool mode)
{
	uint8 phybw40 = IS40MHZ(pi);

	wlc_sslpnphy_lock_ucode_phyreg(pi, 5);

	if (phybw40 == 0) {
		mod_phy_reg((pi), SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_MASK |
			SSLPNPHY_crsgainCtrl_OFDMDetectionEnable_MASK,
			((CHSPEC_IS2G(pi->radio_chanspec)) ? (!mode) : 0) <<
			SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_SHIFT |
			(!mode) << SSLPNPHY_crsgainCtrl_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_crseddisable_MASK,
			(mode) << SSLPNPHY_crsgainCtrl_crseddisable_SHIFT);
	} else {
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_crseddisable_MASK,
			(mode) << SSLPNPHY_Rev2_crsgainCtrl_40_crseddisable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_eddisable20ul,
			SSLPNPHY_Rev2_eddisable20ul_crseddisable_20U_MASK,
			(mode) << SSLPNPHY_Rev2_eddisable20ul_crseddisable_20U_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_eddisable20ul,
			SSLPNPHY_Rev2_eddisable20ul_crseddisable_20L_MASK,
			(mode) << SSLPNPHY_Rev2_eddisable20ul_crseddisable_20L_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_crsgainCtrl_40_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20U_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20L_OFDMDetectionEnable_SHIFT);
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
		if (CHSPEC_SB_UPPER(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
			0 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		} else {
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
			0 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		}
		} else {
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
			0x00 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
			0x00 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		}
		}
	}

	wlc_sslpnphy_unlock_ucode_phyreg(pi);
}

void
wlc_sslpnphy_deaf_mode(phy_info_t *pi, bool mode)
{
	uint8 phybw40 = IS40MHZ(pi);
	mod_phy_reg(pi, SSLPNPHY_rfoverride2,
		SSLPNPHY_rfoverride2_gmode_ext_lna_gain_ovr_MASK,
		(mode) << SSLPNPHY_rfoverride2_gmode_ext_lna_gain_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
		SSLPNPHY_rfoverride2val_gmode_ext_lna_gain_ovr_val_MASK,
		0 << SSLPNPHY_rfoverride2val_gmode_ext_lna_gain_ovr_val_SHIFT);
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_MASK,
			(mode) << SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
			SSLPNPHY_rfoverride2val_amode_ext_lna_gain_ovr_val_MASK,
			0 << SSLPNPHY_rfoverride2val_amode_ext_lna_gain_ovr_val_SHIFT);
	}
#endif
	if (phybw40 == 0) {
		mod_phy_reg((pi), SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_MASK |
			SSLPNPHY_crsgainCtrl_OFDMDetectionEnable_MASK,
			((CHSPEC_IS2G(pi->radio_chanspec)) ? (!mode) : 0) <<
			SSLPNPHY_crsgainCtrl_DSSSDetectionEnable_SHIFT |
			(!mode) << SSLPNPHY_crsgainCtrl_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
			SSLPNPHY_crsgainCtrl_crseddisable_MASK,
			(mode) << SSLPNPHY_crsgainCtrl_crseddisable_SHIFT);
	} else {
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_crseddisable_MASK,
			(mode) << SSLPNPHY_Rev2_crsgainCtrl_40_crseddisable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_eddisable20ul,
			SSLPNPHY_Rev2_eddisable20ul_crseddisable_20U_MASK,
			(mode) << SSLPNPHY_Rev2_eddisable20ul_crseddisable_20U_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_eddisable20ul,
			SSLPNPHY_Rev2_eddisable20ul_crseddisable_20L_MASK,
			(mode) << SSLPNPHY_Rev2_eddisable20ul_crseddisable_20L_SHIFT);
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			if (CHSPEC_SB_UPPER(pi->radio_chanspec)) {
			mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
				SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
				1 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
				SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
				0 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
			} else {
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
		        SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
		        0 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		        mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
		        SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
		        1 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
			}
		} else {
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_MASK,
			0x00 << SSLPNPHY_Rev2_transFreeThresh_20U_DSSSDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_MASK,
			0x00 << SSLPNPHY_Rev2_transFreeThresh_20L_DSSSDetectionEnable_SHIFT);
		}
		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_crsgainCtrl_40_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20U,
			SSLPNPHY_Rev2_transFreeThresh_20U_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20U_OFDMDetectionEnable_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_Rev2_transFreeThresh_20L,
			SSLPNPHY_Rev2_transFreeThresh_20L_OFDMDetectionEnable_MASK,
			(!mode) << SSLPNPHY_Rev2_transFreeThresh_20L_OFDMDetectionEnable_SHIFT);
	}
	}
}
/*
* Given a test tone frequency, continuously play the samples. Ensure that num_periods
* specifies the number of periods of the underlying analog signal over which the
* digital samples are periodic
*/
void
BCMOVERLAYFN(1, wlc_sslpnphy_start_tx_tone)(phy_info_t *pi, int32 f_kHz, uint16 max_val, bool iqcalmode)
{
	uint8 phy_bw;
	uint16 num_samps, t, k;
	uint32 bw;
	fixed theta = 0, rot = 0;
	cint32 tone_samp;
	uint32 data_buf[64];
	uint16 i_samp, q_samp;

	/* Save active tone frequency */
	pi->phy_tx_tone_freq = f_kHz;

	/* Turn off all the crs signals to the MAC */
	wlc_sslpnphy_set_deaf(pi);

	phy_bw = 40;

	/* allocate buffer */
	if (f_kHz) {
		k = 1;
		do {
			bw = phy_bw * 1000 * k;
			num_samps = bw / ABS(f_kHz);
			ASSERT(num_samps <= ARRAYSIZE(data_buf));
			k++;
		} while ((num_samps * (uint32)(ABS(f_kHz))) !=  bw);
	} else
		num_samps = 2;

	WL_INFORM(("wl%d: %s: %d kHz, %d samples\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
		f_kHz, num_samps));

	/* set up params to generate tone */
	rot = FIXED((f_kHz * 36)/phy_bw) / 100; /* 2*pi*f/bw/1000  Note: f in KHz */
	theta = 0;			/* start angle 0 */

	/* tone freq = f_c MHz ; phy_bw = phy_bw MHz ; # samples = phy_bw (1us) ; max_val = 151 */
	/* TCL: set tone_buff [mimophy_gen_tone $f_c $phy_bw $phy_bw $max_val] */
	for (t = 0; t < num_samps; t++) {
		/* compute phasor */
		wlc_phy_cordic(theta, &tone_samp);
		/* update rotation angle */
		theta += rot;
		/* produce sample values for play buffer */
		i_samp = (uint16)(FLOAT(tone_samp.i * max_val) & 0x3ff);
		q_samp = (uint16)(FLOAT(tone_samp.q * max_val) & 0x3ff);
		data_buf[t] = (i_samp << 10) | q_samp;
	}

	/* in SSLPNPHY, we need to bring SPB out of standby before using it */
	mod_phy_reg(pi, SSLPNPHY_sslpnCtrl3,
		SSLPNPHY_sslpnCtrl3_sram_stby_MASK,
		0 << SSLPNPHY_sslpnCtrl3_sram_stby_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_samplePlayClkEn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_samplePlayClkEn_SHIFT);

	/* load sample table */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SAMPLEPLAY,
		data_buf, num_samps, 32, 0);

	/* run samples */
	wlc_sslpnphy_run_samples(pi, num_samps, 0xffff, 0, iqcalmode);
}

OSTATIC bool
BCMOVERLAYFN(1, wlc_sslpnphy_iqcal_wait)(phy_info_t *pi)
{
	uint delay_count = 0;

	while (wlc_sslpnphy_iqcal_active(pi)) {
		OSL_DELAY(100);
		delay_count++;

		if (delay_count > (10 * 500)) /* 500 ms */
			break;
	}

	WL_NONE(("wl%d: %s: %u us\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, delay_count * 100));

	return (0 == wlc_sslpnphy_iqcal_active(pi));
}

void
BCMROMOVERLAYFN(1, wlc_sslpnphy_stop_tx_tone)(phy_info_t *pi)
{
	int16 playback_status;

	pi->phy_tx_tone_freq = 0;

	/* Stop sample buffer playback */
	playback_status = read_phy_reg(pi, SSLPNPHY_sampleStatus);
	if (playback_status & SSLPNPHY_sampleStatus_NormalPlay_MASK) {
		wlc_sslpnphy_tx_pu(pi, 0);
		mod_phy_reg(pi, SSLPNPHY_sampleCmd,
			SSLPNPHY_sampleCmd_stop_MASK,
			1 << SSLPNPHY_sampleCmd_stop_SHIFT);
	} else if (playback_status & SSLPNPHY_sampleStatus_iqlocalPlay_MASK)
		mod_phy_reg(pi, SSLPNPHY_iqloCalCmdGctl,
			SSLPNPHY_iqloCalCmdGctl_iqlo_cal_en_MASK,
			0 << SSLPNPHY_iqloCalCmdGctl_iqlo_cal_en_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_samplePlayClkEn_MASK,
		0 << SSLPNPHY_sslpnCalibClkEnCtrl_samplePlayClkEn_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_forceaphytxFeclkOn_MASK,
		0 << SSLPNPHY_sslpnCalibClkEnCtrl_forceaphytxFeclkOn_SHIFT);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
			SSLPNPHY_sslpnCalibClkEnCtrl_papdTxBbmultPapdifClkEn_MASK,
			0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxBbmultPapdifClkEn_SHIFT);
	}
	/* in SSLPNPHY, we need to bring SPB out of standby before using it */
	mod_phy_reg(pi, SSLPNPHY_sslpnCtrl3,
		SSLPNPHY_sslpnCtrl3_sram_stby_MASK,
		1 << SSLPNPHY_sslpnCtrl3_sram_stby_SHIFT);



	/* Restore all the crs signals to the MAC */
	wlc_sslpnphy_clear_deaf(pi);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_clear_trsw_override)(phy_info_t *pi)
{
	/* Clear overrides */
	and_phy_reg(pi, SSLPNPHY_RFOverride0,
		(uint16)~(SSLPNPHY_RFOverride0_trsw_tx_pu_ovr_MASK |
		SSLPNPHY_RFOverride0_trsw_rx_pu_ovr_MASK));
}

/*
 * TX IQ/LO Calibration
 *
 * args: target_gains = Tx gains *for* which the cal is done, not necessarily *at* which it is done
 *       If not specified, will use current Tx gains as target gains
 *
 */
OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_tx_iqlo_cal)(
	phy_info_t *pi,
	sslpnphy_txgains_t *target_gains,
	sslpnphy_cal_mode_t cal_mode,
	bool keep_tone)
{
	/* starting values used in full cal
	 * -- can fill non-zero vals based on lab campaign (e.g., per channel)
	 * -- format: a0,b0,a1,b1,ci0_cq0_ci1_cq1,di0_dq0,di1_dq1,ei0_eq0,ei1_eq1,fi0_fq0,fi1_fq1
	 */
	sslpnphy_txgains_t cal_gains, temp_gains;
	uint16 hash;
	uint8 band_idx;
	int j;
	uint16 ncorr_override[5];
	uint16 syst_coeffs[] =
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

	/* cal commands full cal and recal */
	uint16 commands_fullcal[] =  { 0x8434, 0x8334, 0x8084, 0x8267, 0x8056, 0x8234 };
	uint16 commands_recal[] =  { 0x8312, 0x8055, 0x8212 };

	/* calCmdNum register: log2 of settle/measure times for search/gain-ctrl, 4 bits each */
	uint16 command_nums_fullcal[] = { 0x7a97, 0x7a97, 0x7a97, 0x7a87, 0x7a87, 0x7b97 };
	uint16 command_nums_recal[] = {  0x7997, 0x7987, 0x7a97 };
	uint16 *command_nums = command_nums_fullcal;


	uint16 *start_coeffs = NULL, *cal_cmds = NULL, cal_type, diq_start;
	uint16 tx_pwr_ctrl_old, rssi_old;
	uint16 papd_ctrl_old = 0, auxadc_ctrl_old = 0;
	uint16 muxsel_old, pa_ctrl_1_old, extstxctrl1_old;
	uint8 iqcal_old;
	bool tx_gain_override_old;
	sslpnphy_txgains_t old_gains;
	uint i, n_cal_cmds = 0, n_cal_start = 0;
	uint16 ccktap0, ccktap1, ccktap2, ccktap3, ccktap4;
	uint16 epa_ovr, epa_ovr_val;
#ifdef BAND5G
	uint16 sslpnCalibClkEnCtrl_old = 0;
	uint16 Core1TxControl_old = 0;
#endif
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (NORADIO_ENAB(pi->pubpi))
		return;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		/* Saving the default states of realfilter coefficients */
		ccktap0 = read_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap0);
		ccktap1 = read_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap1);
		ccktap2 = read_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap2);
		ccktap3 = read_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap3);
		ccktap4 = read_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap4);

		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap0, 255);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap1, 0);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap2, 0);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap3, 0);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap4, 0);
	}

	switch (cal_mode) {
		case SSLPNPHY_CAL_FULL:
			start_coeffs = syst_coeffs;
			cal_cmds = commands_fullcal;
			n_cal_cmds = ARRAYSIZE(commands_fullcal);
			break;

		case SSLPNPHY_CAL_RECAL:
			ASSERT(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs_valid);
			start_coeffs = sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs;
			cal_cmds = commands_recal;
			n_cal_cmds = ARRAYSIZE(commands_recal);
			command_nums = command_nums_recal;
			break;

		default:
			ASSERT(FALSE);
	}

	/* Fill in Start Coeffs */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		start_coeffs, 11, 16, 64);

	/* Save original tx power control mode */
	tx_pwr_ctrl_old = wlc_sslpnphy_get_tx_pwr_ctrl(pi);

	/* Save RF overide values */
	epa_ovr = read_phy_reg(pi, SSLPNPHY_RFOverride0);
	epa_ovr_val = read_phy_reg(pi, SSLPNPHY_RFOverrideVal0);

	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

	/* Save old and apply new tx gains if needed */
	tx_gain_override_old = wlc_sslpnphy_tx_gain_override_enabled(pi);
	if (tx_gain_override_old)
		wlc_sslpnphy_get_tx_gain(pi, &old_gains);

	if (!target_gains) {
		if (!tx_gain_override_old)
			wlc_sslpnphy_set_tx_pwr_by_index(pi, sslpnphy_specific->sslpnphy_tssi_idx);
		wlc_sslpnphy_get_tx_gain(pi, &temp_gains);
		target_gains = &temp_gains;
	}

	hash = (target_gains->gm_gain << 8) |
		(target_gains->pga_gain << 4) |
		(target_gains->pad_gain);

	cal_gains = *target_gains;
	bzero(ncorr_override, sizeof(ncorr_override));
	for (j = 0; j < iqcal_gainparams_numgains_sslpnphy[band_idx]; j++) {
		if (hash == tbl_iqcal_gainparams_sslpnphy[band_idx][j][0]) {
			cal_gains.gm_gain = tbl_iqcal_gainparams_sslpnphy[band_idx][j][1];
			cal_gains.pga_gain = tbl_iqcal_gainparams_sslpnphy[band_idx][j][2];
			cal_gains.pad_gain = tbl_iqcal_gainparams_sslpnphy[band_idx][j][3];
			bcopy(&tbl_iqcal_gainparams_sslpnphy[band_idx][j][3], ncorr_override,
				sizeof(ncorr_override));
			break;
		}
	}

	wlc_sslpnphy_set_tx_gain(pi, &cal_gains);

	WL_INFORM(("wl%d: %s: target gains: %d %d %d %d, cal_gains: %d %d %d %d\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
		target_gains->gm_gain,
		target_gains->pga_gain,
		target_gains->pad_gain,
		target_gains->dac_gain,
		cal_gains.gm_gain,
		cal_gains.pga_gain,
		cal_gains.pad_gain,
		cal_gains.dac_gain));

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_SHIFT);

	/* Open TR switch */
	wlc_sslpnphy_set_swctrl_override(pi, SWCTRL_BT_TX);

	muxsel_old = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal);
	pa_ctrl_1_old = read_radio_reg(pi, RADIO_2063_PA_CTRL_1);
	extstxctrl1_old = read_phy_reg(pi, SSLPNPHY_extstxctrl1);
	/* Removing all the radio reg intervention in selecting the mux */
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_MASK,
		1 << SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
		SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
		4  << SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		or_radio_reg(pi, RADIO_2063_PA_SP_1, 0x2);
		or_radio_reg(pi, RADIO_2063_COMMON_07, 0x10);

		mod_radio_reg(pi, RADIO_2063_PA_CTRL_1, 0x1 << 2, 0 << 2);
	} else
#endif
		mod_radio_reg(pi, RADIO_2063_PA_CTRL_1, 0x1 << 2, 1 << 2);

	or_phy_reg(pi, SSLPNPHY_extstxctrl1, 0x1000);

	/* Adjust ADC common mode */
	rssi_old = read_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1);

	/* crk: sync up with tcl */
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec))
		mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, 0x3fff, 0x28af);
	else
#endif
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, 0x3ff, 0xaf);

	/* Set tssi switch to use IQLO */
	iqcal_old = (uint8)read_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2);
	and_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, (uint8)~0x0d);

	/* Power on IQLO block */
	and_radio_reg(pi, RADIO_2063_IQCAL_GVAR, (uint8)~0x80);

	/* Turn off PAPD */

	papd_ctrl_old = read_phy_reg(pi, SSLPNPHY_papd_control);
	mod_phy_reg(pi, SSLPNPHY_papd_control,
		SSLPNPHY_papd_control_papdCompEn_MASK,
		0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
			sslpnCalibClkEnCtrl_old = read_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl);
			Core1TxControl_old = read_phy_reg(pi, SSLPNPHY_Core1TxControl);
			mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
				SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_MASK,
				0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
				0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
				SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_MASK,
				0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
				SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_MASK,
				0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txrealfilten_MASK,
				0 << SSLPNPHY_Core1TxControl_txrealfilten_SHIFT);
		}
	}
#endif /* BAND5G */
	auxadc_ctrl_old = read_phy_reg(pi, SSLPNPHY_auxadcCtrl);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_iqlocalEn_MASK,
		1 << SSLPNPHY_auxadcCtrl_iqlocalEn_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK,
		0 << SSLPNPHY_auxadcCtrl_rssiformatConvEn_SHIFT);

	/* Load the LO compensation gain table */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		sslpnphy_iqcal_loft_gainladder, ARRAYSIZE(sslpnphy_iqcal_loft_gainladder),
		16, 0);

	/* Load the IQ calibration gain table */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		sslpnphy_iqcal_ir_gainladder, ARRAYSIZE(sslpnphy_iqcal_ir_gainladder),
		16, 32);

	/* Set Gain Control Parameters */
	/* iqlocal_en<15> / start_index / thresh_d2 / ladder_length_d2 */
	write_phy_reg(pi, SSLPNPHY_iqloCalCmdGctl, 0x0aa9);

	/* Send out calibration tone */
	if (!pi->phy_tx_tone_freq) {
		wlc_sslpnphy_start_tx_tone(pi, 3750, 88, 1);
	}
	mod_phy_reg(pi, SSLPNPHY_rfoverride3,
		SSLPNPHY_rfoverride3_stxpapu_ovr_MASK,
		1 << SSLPNPHY_rfoverride3_stxpapu_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rfoverride3_val,
		SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_MASK,
		0 << SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_SHIFT);
	/* Disable epa during calibrations */
	if ((CHSPEC_IS5G(pi->radio_chanspec)) &&
		(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)) {
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_MASK,
			0  << SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_SHIFT);
	}

	/*
	 * Cal Steps
	 */
	for (i = n_cal_start; i < n_cal_cmds; i++) {
		uint16 zero_diq = 0;
		uint16 best_coeffs[11];
		uint16 command_num;

		cal_type = (cal_cmds[i] & 0x0f00) >> 8;


		/* get & set intervals */
		command_num = command_nums[i];
		if (ncorr_override[cal_type])
			command_num = ncorr_override[cal_type] << 8 | (command_num & 0xff);

		write_phy_reg(pi, SSLPNPHY_iqloCalCmdNnum, command_num);

		WL_NONE(("wl%d: %s: running cmd: %x, cmd_num: %x\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, cal_cmds[i], command_nums[i]));

		/* need to set di/dq to zero if analog LO cal */
		if ((cal_type == 3) || (cal_type == 4)) {
			wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
				&diq_start, 1, 16, 69);

			/* Set to zero during analog LO cal */
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL, &zero_diq,
				1, 16, 69);
		}

		/* Issue cal command */
		write_phy_reg(pi, SSLPNPHY_iqloCalCmd, cal_cmds[i]);

		/* Wait until cal command finished */
		if (!wlc_sslpnphy_iqcal_wait(pi)) {
			WL_ERROR(("wl%d: %s: tx iqlo cal failed to complete\n",
				GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
			/* No point to continue */
			goto cleanup;
		}

		/* Copy best coefficients to start coefficients */
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
			best_coeffs, ARRAYSIZE(best_coeffs), 16, 96);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL, best_coeffs,
			ARRAYSIZE(best_coeffs), 16, 64);

		/* restore di/dq in case of analog LO cal */
		if ((cal_type == 3) || (cal_type == 4)) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
				&diq_start, 1, 16, 69);
		}
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs,
			ARRAYSIZE(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs), 16, 96);
	}

	/*
	 * Apply Results
	 */

	/* Save calibration results */
	wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs,
		ARRAYSIZE(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs), 16, 96);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs_valid = TRUE;

	/* Apply IQ Cal Results */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		&sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[0], 4, 16, 80);

	/* Apply Digital LOFT Comp */
	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_IQLOCAL,
		&sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[5], 2, 16, 85);

	/* Dump results */
	WL_INFORM(("wl%d: %s %d complete, IQ %d %d LO %d %d %d %d %d %d\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, pi->radio_chanspec,
		(int16)sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[0],
		(int16)sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[1],
		(int8)((sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[5] & 0xff00) >> 8),
		(int8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[5] & 0x00ff),
		(int8)((sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[7] & 0xff00) >> 8),
		(int8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[7] & 0x00ff),
		(int8)((sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[9] & 0xff00) >> 8),
		(int8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[9] & 0x00ff)));

cleanup:
	/* Switch off test tone */
	if (!keep_tone)
		wlc_sslpnphy_stop_tx_tone(pi);

	/* Reset calibration  command register */
	write_phy_reg(pi, SSLPNPHY_iqloCalCmdGctl, 0);

	{
		/* RSSI ADC selection */
		mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
			SSLPNPHY_auxadcCtrl_iqlocalEn_MASK,
			0 << SSLPNPHY_auxadcCtrl_iqlocalEn_SHIFT);

		/* Power off IQLO block */
		or_radio_reg(pi, RADIO_2063_IQCAL_GVAR, 0x80);

		/* Adjust ADC common mode */
		write_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, rssi_old);

		/* Restore tssi switch */
		write_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, iqcal_old);

		/* Restore PAPD */
		write_phy_reg(pi, SSLPNPHY_papd_control, papd_ctrl_old);

		/* Restore epa after cal */
		write_phy_reg(pi, SSLPNPHY_RFOverride0, epa_ovr);
		write_phy_reg(pi, SSLPNPHY_RFOverrideVal0, epa_ovr_val);

#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
			write_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, sslpnCalibClkEnCtrl_old);
			write_phy_reg(pi, SSLPNPHY_Core1TxControl, Core1TxControl_old);
		}
	}
#endif
		/* Restore ADC control */
		write_phy_reg(pi, SSLPNPHY_auxadcCtrl, auxadc_ctrl_old);
	}

	write_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal, muxsel_old);
	write_radio_reg(pi, RADIO_2063_PA_CTRL_1, pa_ctrl_1_old);
	write_phy_reg(pi, SSLPNPHY_extstxctrl1, extstxctrl1_old);

	/* TR switch */
	wlc_sslpnphy_set_swctrl_override(pi, SWCTRL_OVR_DISABLE);

	/* RSSI on/off */
	and_phy_reg(pi, SSLPNPHY_AfeCtrlOvr, (uint16)~SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK);

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_MASK,
		0 << SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_SHIFT);

	/* Restore tx power and reenable tx power control */
	if (tx_gain_override_old)
		wlc_sslpnphy_set_tx_gain(pi, &old_gains);
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl_old);
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		/* Restoring the origianl values */
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap0, ccktap0);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap1, ccktap1);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap2, ccktap2);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap3, ccktap3);
		write_phy_reg(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap4, ccktap4);
	}
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_MASK,
		0 << SSLPNPHY_AfeCtrlOvr_rssi_muxsel_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rfoverride3,
		SSLPNPHY_rfoverride3_stxpapu_ovr_MASK,
		0 << SSLPNPHY_rfoverride3_stxpapu_ovr_SHIFT);
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_get_tx_iqcc)(phy_info_t *pi, uint16 *a, uint16 *b)
{
	uint16 iqcc[2];

	wlc_sslpnphy_common_read_table(pi, 0, iqcc, 2, 16, 80);

	*a = iqcc[0];
	*b = iqcc[1];
}

uint16
BCMOVERLAYFN(1, wlc_sslpnphy_get_tx_locc)(phy_info_t *pi)
{
	uint16 didq;

	/* Update iqloCaltbl */
	wlc_sslpnphy_common_read_table(pi, 0, &didq, 1, 16, 85);

	return didq;
}

/* Run iqlo cal and populate iqlo portion of tx power control table */
void
BCMOVERLAYFN(1, wlc_sslpnphy_txpwrtbl_iqlo_cal)(phy_info_t *pi)
{
	sslpnphy_txgains_t target_gains;
	uint8 save_bb_mult;
	uint16 a, b, didq, save_pa_gain = 0;
	uint idx;
	uint32 val;
	uint8 gm, pga, pad;
	uint16 tx_pwr_ctrl;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* Store state */
	save_bb_mult = wlc_sslpnphy_get_bbmult(pi);

	/* Save tx power control mode */
	tx_pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);

	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

	/* Set up appropriate target gains */
	{
		/* PA gain */
		save_pa_gain = wlc_sslpnphy_get_pa_gain(pi);

		wlc_sslpnphy_set_pa_gain(pi, 0x10);

#ifdef BAND5G
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			/* Since we can not switch off pa from phy put gain to 0 */
			wlc_sslpnphy_set_pa_gain(pi, 0x00);
			target_gains.gm_gain = 7;
			target_gains.pga_gain = 200;
			target_gains.pad_gain = 245;
			target_gains.dac_gain = 0;
			if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA) {
				target_gains.gm_gain = 3;
				target_gains.pga_gain = 105;
				target_gains.pad_gain = 240;
				target_gains.dac_gain = 0;
			}
		} else
#endif /* BAND5G */
		{
			target_gains.gm_gain = 7;
			target_gains.pga_gain = 76;
			target_gains.pad_gain = 241;
			target_gains.dac_gain = 0;

			/* Do the second tx iq cal with gains corresponding to 14dBm */
			if (sslpnphy_specific->sslpnphy_papd_cal_done) {
				/* Read gains corresponding to 14dBm index */
				wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
					&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_GAIN_OFFSET + sslpnphy_specific->sslpnphy_start_idx);
					gm = (val & 0xff);
					pga = ((val & 0xff00) >> 8);
					pad = ((val & 0xff0000) >> 16);
				target_gains.gm_gain = gm;
				target_gains.pga_gain = pga;
				target_gains.pad_gain = pad;
				target_gains.dac_gain = 0;
			}
		}
	}

	/* Run cal */
	wlc_sslpnphy_tx_iqlo_cal(pi, &target_gains, (sslpnphy_specific->sslpnphy_recal ?
		SSLPNPHY_CAL_RECAL : SSLPNPHY_CAL_FULL), FALSE);

	{
		uint8 ei0, eq0, fi0, fq0;

		wlc_sslpnphy_get_radio_loft(pi, &ei0, &eq0, &fi0, &fq0);
		if ((ABS((int8)fi0) == 15) && (ABS((int8)fq0) == 15)) {
			WL_ERROR(("wl%d: %s: tx iqlo cal failed, retrying...\n",
				GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
#ifdef BAND5G
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			target_gains.gm_gain = 255;
			target_gains.pga_gain = 255;
			target_gains.pad_gain = 0xf0;
			target_gains.dac_gain = 0;
		} else
#endif
		{
			target_gains.gm_gain = 7;
			target_gains.pga_gain = 45;
			target_gains.pad_gain = 186;
			target_gains.dac_gain = 0;
		}
			/* Re-run cal */
			wlc_sslpnphy_tx_iqlo_cal(pi, &target_gains, SSLPNPHY_CAL_FULL, FALSE);
		}
	}

	/* Get calibration results */
	wlc_sslpnphy_get_tx_iqcc(pi, &a, &b);
	didq = wlc_sslpnphy_get_tx_locc(pi);

	/* Populate tx power control table with coeffs */
	for (idx = 0; idx < 128; idx++) {
		/* iq */
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL, &val,
			1, 32, SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + idx);
		val = (val & 0xfff00000) |
			((uint32)(a & 0x3FF) << 10) | (b & 0x3ff);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + idx);

		/* loft */
		val = didq;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&val, 1, 32, SSLPNPHY_TX_PWR_CTRL_LO_OFFSET + idx);
	}

	/* Restore state */
	wlc_sslpnphy_set_bbmult(pi, save_bb_mult);
	wlc_sslpnphy_set_pa_gain(pi, save_pa_gain);
	/* Restore power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_set_tx_filter_bw)(phy_info_t *pi, uint16 bw)
{
	uint8 idac_setting;

	/* cck/all non-ofdm setting */
	mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
		SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
		bw << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);
	/* ofdm setting */
	mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg1,
		SSLPNPHY_lpfbwlutreg1_lpfbwlut5_MASK,
		bw << SSLPNPHY_lpfbwlutreg1_lpfbwlut5_SHIFT);

	if (0) {
	if (bw <= 1)
		idac_setting = 0x0e;
	else if (bw <= 3)
		idac_setting = 0x13;
	else
		idac_setting = 0x1b;
	mod_radio_reg(pi, RADIO_2063_TXBB_CTRL_2, 0x1f, idac_setting);
	}
}

STATIC void
wlc_sslpnphy_set_rx_gain(phy_info_t *pi, uint32 gain)
{
	uint16 trsw, ext_lna, lna1, lna2, gain0_15, gain16_19;

	trsw = (gain & ((uint32)1 << 20)) ? 0 : 1;
	ext_lna = (uint16)(gain >> 21) & 0x01;
	lna1 = (uint16)(gain >> 2) & 0x03;
	lna2 = (uint16)(gain >> 6) & 0x03;
	gain0_15 = (uint16)gain & 0xffff;
	gain16_19 = (uint16)(gain >> 16) & 0x0f;

	mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
		SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_MASK,
		trsw << SSLPNPHY_RFOverrideVal0_trsw_rx_pu_ovr_val_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
		SSLPNPHY_rfoverride2val_gmode_ext_lna_gain_ovr_val_MASK,
		ext_lna << SSLPNPHY_rfoverride2val_gmode_ext_lna_gain_ovr_val_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
		SSLPNPHY_rfoverride2val_amode_ext_lna_gain_ovr_val_MASK,
		ext_lna << SSLPNPHY_rfoverride2val_amode_ext_lna_gain_ovr_val_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rxgainctrl0ovrval,
		SSLPNPHY_rxgainctrl0ovrval_rxgainctrl_ovr_val0_MASK,
		gain0_15 << SSLPNPHY_rxgainctrl0ovrval_rxgainctrl_ovr_val0_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_rxlnaandgainctrl1ovrval,
		SSLPNPHY_rxlnaandgainctrl1ovrval_rxgainctrl_ovr_val1_MASK,
		gain16_19 << SSLPNPHY_rxlnaandgainctrl1ovrval_rxgainctrl_ovr_val1_SHIFT);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
			SSLPNPHY_rfoverride2val_slna_gain_ctrl_ovr_val_MASK,
			lna1 << SSLPNPHY_rfoverride2val_slna_gain_ctrl_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverrideVal,
			SSLPNPHY_RFinputOverrideVal_wlslnagainctrl_ovr_val_MASK,
			lna1 << SSLPNPHY_RFinputOverrideVal_wlslnagainctrl_ovr_val_SHIFT);
	}
	wlc_sslpnphy_rx_gain_override_enable(pi, TRUE);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_cal_setup_cw)(phy_info_t *pi)
{
	uint32 papd_buf[] = {0x7fc00, 0x5a569, 0x1ff, 0xa5d69, 0x80400, 0xa5e97, 0x201, 0x5a697};
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* Tune the hardware delay */
	write_phy_reg(pi, SSLPNPHY_papd_spb2papdin_dly, 33);
	/* Set samples/cycle/4 for q delay */
		mod_phy_reg(pi, SSLPNPHY_papd_variable_delay,
			(SSLPNPHY_papd_variable_delay_papd_pre_int_est_dly_MASK	|
			SSLPNPHY_papd_variable_delay_papd_int_est_ovr_or_cw_dly_MASK),
			(((4-1) << SSLPNPHY_papd_variable_delay_papd_pre_int_est_dly_SHIFT) |
			0 << SSLPNPHY_papd_variable_delay_papd_int_est_ovr_or_cw_dly_SHIFT));

	write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 100);
	/* Set LUT begin gain, step gain, and size (Reset values, remove if possible) */
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
		if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID)) {
			write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 100);
			if (freq >= 5520)
				write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 100);
			if (freq >= 5600)
				write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 200);
			if (freq >= 5745)
				write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 300);
		}
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) {
			write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm, 500);
			write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin, 6000);
			write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step, 0x444);
		} else {
			write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin, 5000);
			write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step, 0x222);
		}
	} else
#endif /* BAND 5G */
	{
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin, 6000);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step, 0x444);
	}

	if (sslpnphy_specific->sslpnphy_papd_tweaks_enable) {
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin,
			sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_begin);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step,
			sslpnphy_specific->sslpnphy_papd_tweaks.papd_track_pa_lut_step);
	}

	write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_end, 0x3f);

	/* Set papd constants (reset values, remove if possible) */
	write_phy_reg(pi, SSLPNPHY_papd_dbm_offset, 0x681);
	write_phy_reg(pi, SSLPNPHY_papd_track_dbm_adj_mult_factor, 0xcd8);
	write_phy_reg(pi, SSLPNPHY_papd_track_dbm_adj_add_factor_lsb, 0xc15c);
	write_phy_reg(pi, SSLPNPHY_papd_track_dbm_adj_add_factor_msb, 0x1b);

	/* Dc estimation samples */
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_dc_est, 0x49);

	/* Processing parameters */
	write_phy_reg(pi, SSLPNPHY_papd_num_skip_count, 0x27);
	write_phy_reg(pi, SSLPNPHY_papd_num_samples_count, 255);
	write_phy_reg(pi, SSLPNPHY_papd_sync_count, 319);
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_index_num_cnt, 255);
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_corelator_run_cnt, 1);
	write_phy_reg(pi, SSLPNPHY_smoothenLut_max_thr, 0x7ff);
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_sync_clip_threshold, 0);

	/* Overide control Params */
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_loop_gain_offset_ovr_15_0, 0x0000);
	write_phy_reg(pi, SSLPNPHY_papd_ofdm_loop_gain_offset_ovr_18_16, 0x0007);
	write_phy_reg(pi, SSLPNPHY_papd_dcest_i_ovr, 0x0000);
	write_phy_reg(pi, SSLPNPHY_papd_dcest_q_ovr, 0x0000);

	/* PAPD Update */
	write_phy_reg(pi, SSLPNPHY_papd_lut_update_beta, 0x0008);

	/* Spb parameters */
	write_phy_reg(pi, SSLPNPHY_papd_spb_num_vld_symbols_n_dly, 0x60);
	write_phy_reg(pi, SSLPNPHY_sampleDepthCount, 8-1);

	/* Load Spb - Remove it latter when CW waveform gets a fixed place inside SPB. */
	write_phy_reg(pi, SSLPNPHY_papd_spb_rd_address, 0x0000);

	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SAMPLEPLAY,
		&papd_buf, 8, 32, 0);

	/* BBMULT parameters */
	write_phy_reg(pi, SSLPNPHY_papd_bbmult_num_symbols, 1-1);
	write_phy_reg(pi, SSLPNPHY_papd_rx_sm_iqmm_gain_comp, 0x100);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_cal_core)(
	phy_info_t *pi,
	sslpnphy_papd_cal_type_t calType,
	bool rxGnCtrl,
	uint8 num_symbols4lpgn,
	bool init_papd_lut,
	uint16 papd_bbmult_init,
	uint16 papd_bbmult_step,
	bool papd_lpgn_ovr,
	uint16 LPGN_I,
	uint16 LPGN_Q)
{
	uint32 papdcompdeltatbl_init_val;

	mod_phy_reg(pi, SSLPNPHY_papd_control2,
		SSLPNPHY_papd_control2_papd_loop_gain_cw_ovr_MASK,
		papd_lpgn_ovr << SSLPNPHY_papd_control2_papd_loop_gain_cw_ovr_SHIFT);

	/* Load papd comp delta table */

	papdcompdeltatbl_init_val = 0x80000;

	mod_phy_reg(pi, SSLPNPHY_papd_control,
		SSLPNPHY_papd_control_papdCompEn_MASK |
		SSLPNPHY_papd_control_papd_use_pd_out4learning_MASK,
		((0 << SSLPNPHY_papd_control_papdCompEn_SHIFT) |
		(0 << SSLPNPHY_papd_control_papd_use_pd_out4learning_SHIFT)));

	/* Reset the PAPD Hw to reset register values */
	/* Check if this is what tcl meant */

	if (calType == SSLPNPHY_PAPD_CAL_CW) {

		/* Overide control Params */
		/* write_phy_reg(pi, SSLPNPHY_papd_control2, 0); */
		write_phy_reg(pi, SSLPNPHY_papd_loop_gain_ovr_cw_i, LPGN_I);
		write_phy_reg(pi, SSLPNPHY_papd_loop_gain_ovr_cw_q, LPGN_Q);

		/* Spb parameters */
		write_phy_reg(pi, SSLPNPHY_papd_track_num_symbols_count, num_symbols4lpgn);
		write_phy_reg(pi, SSLPNPHY_sampleLoopCount, (num_symbols4lpgn+1)*20-1);

		/* BBMULT parameters */
		write_phy_reg(pi, SSLPNPHY_papd_bbmult_init, papd_bbmult_init);
		write_phy_reg(pi, SSLPNPHY_papd_bbmult_step, papd_bbmult_step);
		/* Run PAPD HW Cal */
		write_phy_reg(pi, SSLPNPHY_papd_control, 0xa021);

#ifndef SSLPNPHY_PAPD_OFDM
	}
#else
	} else {

		/* Number of Sync and Training Symbols */
		write_phy_reg(pi, SSLPNPHY_papd_track_num_symbols_count, 255);
		write_phy_reg(pi, SSLPNPHY_papd_sync_symbol_count, 49);

		/* Load Spb */
		write_phy_reg(pi, SSLPNPHY_papd_spb_rd_address, 0x0000);


		for (j = 0; j < 16; j++) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SAMPLEPLAY,
				&sslpnphy_papd_cal_ofdm_tbl[j][0], 64, 32, j * 64);
		}

		for (; j < 25; j++) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SAMPLEPLAY,
				&sslpnphy_papd_cal_ofdm_tbl[j][0], 64, 32, j * 64);
		}

		/* Number of CW samples in spb - 1; Num of OFDM samples per symbol in SPB */

		write_phy_reg(pi, SSLPNPHY_sampleDepthCount, 160-1);

		/* Number of loops - 1 for CW; 2-1 for replay with rotation by -j in OFDM */
		write_phy_reg(pi, SSLPNPHY_sampleLoopCount,  1);

		write_phy_reg(pi, SSLPNPHY_papd_bbmult_init, 20000);
		write_phy_reg(pi, SSLPNPHY_papd_bbmult_step, 22000);
		write_phy_reg(pi, SSLPNPHY_papd_bbmult_ofdm_sync, 8192);

		/* If Cal is done at a gain other than the ref gain
		 * (incremental cal over an earlier cal)
		 * then gain difference needs to be subracted here
		*/
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin, 6700);

		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step, 0x222);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_end,  0x3f);
		write_phy_reg(pi, SSLPNPHY_papd_lut_update_beta, 0x8);

		/* [3:0] vld sym in spb         -1, ofdm; [7:4] spb delay */
		write_phy_reg(pi, SSLPNPHY_papd_spb_num_vld_symbols_n_dly, 0x69);

		if (rxGnCtrl) {
			/* Only run the synchronizer - no tracking */
			write_phy_reg(pi, SSLPNPHY_papd_track_num_symbols_count, 0);
		}

		/* Run PAPD HW Cal */
		write_phy_reg(pi, SSLPNPHY_papd_control, 0xb083);
	}
#endif /* SSLPNPHY_PAPD_OFDM */

	/* Wait for completion, around 1ms */
	SPINWAIT(
		read_phy_reg(pi, SSLPNPHY_papd_control) & SSLPNPHY_papd_control_papd_cal_run_MASK,
		1 * 1000);

}

OSTATIC uint32
BCMOVERLAYFN(1, wlc_sslpnphy_papd_rxGnCtrl)(
	phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	bool frcRxGnCtrl,
	uint8 CurTxGain)
{
	/* Square of Loop Gain (inv) target for CW (reach as close to tgt, but be more than it) */
	/* dB Loop gain (inv) target for OFDM (reach as close to tgt,but be more than it) */
	int32 rxGnInit = 8;
	uint8  bsStep = 4; /* Binary search initial step size */
	uint8  bsDepth = 5; /* Binary search depth */
	uint8  bsCnt;
	int16  lgI, lgQ;
	int32  cwLpGn2;
	int32  cwLpGn2_min = 8192, cwLpGn2_max = 16384;
	uint8  num_symbols4lpgn;
	int32 volt_start, volt_end;
	uint8 counter = 0;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	for (bsCnt = 0; bsCnt < bsDepth; bsCnt++) {
		if (rxGnInit > 15)
			rxGnInit = 15; /* out-of-range correction */

		wlc_sslpnphy_set_rx_gain_by_distribution(pi, (uint16)rxGnInit, 0, 0, 0, 0, 0, 0);

		num_symbols4lpgn = 90;
		counter = 0;
		do {
			if (counter >= 5)
				break;
			volt_start = wlc_sslpnphy_vbatsense(pi);
			wlc_sslpnphy_papd_cal_core(pi, cal_type,
				TRUE,
				num_symbols4lpgn,
				1,
				1400,
				16640,
				0,
				128,
				0);
			volt_end = wlc_sslpnphy_vbatsense(pi);
			if ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
				(volt_end < sslpnphy_specific->sslpnphy_volt_winner))
				counter ++;
		} while ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
			(volt_end < sslpnphy_specific->sslpnphy_volt_winner));

		if (cal_type == SSLPNPHY_PAPD_CAL_CW)
		{
			lgI = ((int16) read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_i)) << 6;
			lgI = lgI >> 6;
			lgQ = ((int16) read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_q)) << 6;
			lgQ = lgQ >> 6;
			cwLpGn2 = (lgI * lgI) + (lgQ * lgQ);

			if (cwLpGn2 < cwLpGn2_min) {
				rxGnInit = rxGnInit - bsStep;
				if (bsCnt == 4)
					rxGnInit = rxGnInit - 1;
			} else if (cwLpGn2 >= cwLpGn2_max) {
				rxGnInit = rxGnInit + bsStep;
			} else {
				break;
			}
#ifndef SSLPNPHY_PAPD_OFDM
		}
#else
		} else {
			int32 lgLow, lgHigh;
			int32 ofdmLpGnDb, ofdmLpGnDbTgt = 0;

			/* is this correct ? */
			lgLow = (uint32) read_phy_reg(pi, SSLPNPHY_papd_ofdm_loop_gain_offset_15_0);
			if (lgLow < 0)
				lgLow = lgLow + 65536;

			/* SSLPNPHY_papd_ofdm_loop_gain_offset_18_16_
			 * papd_ofdm_loop_gain_offset_18_16_MASK
			 * is a veeery long register mask. substituting its value instead
			 */
			lgHigh = ((int16)
				 read_phy_reg(pi, SSLPNPHY_papd_ofdm_loop_gain_offset_18_16)) &
				 0x7;


			ofdmLpGnDb = lgHigh*65536 + lgLow;
			if (ofdmLpGnDb < ofdmLpGnDbTgt) {
				rxGnInit = rxGnInit - bsStep;
				if (bsCnt == 4)
					rxGnInit = rxGnInit - 1;
			} else {
				rxGnInit = rxGnInit + bsStep;
			}

		}
#endif /* SSLPNPHY_PAPD_OFDM */
		bsStep = bsStep >> 1;
	}
	if (rxGnInit < 0)
		rxGnInit = 0; /* out-of-range correction */

	sslpnphy_specific->sslpnphy_papdRxGnIdx = rxGnInit;
	return rxGnInit;
}

static void
wlc_sslpnphy_afe_clk_init(phy_info_t *pi, uint8 mode)
{
	uint8 phybw40 = IS40MHZ(pi);

	if (0) {
	/* Option 1 : IQ SWAP @ ADC OUTPUT */
	mod_phy_reg(pi, SSLPNPHY_adcsync,
		SSLPNPHY_adcsync_flip_adcsyncoutiq_MASK,
		1 << SSLPNPHY_adcsync_flip_adcsyncoutiq_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_adcsync,
		SSLPNPHY_adcsync_flip_adcsyncoutvlds_MASK,
		1 << SSLPNPHY_adcsync_flip_adcsyncoutvlds_SHIFT);
	}

	if (1) {
	if (!NORADIO_ENAB(pi->pubpi)) {
		/* Option 2 : NO IQ SWAP for QT @ ADC INPUT */
		mod_phy_reg(pi, SSLPNPHY_rxfe,
			SSLPNPHY_rxfe_swap_rxfiltout_iq_MASK,
			1 << SSLPNPHY_rxfe_swap_rxfiltout_iq_SHIFT);
	} else {
		/* Option 2 : IQ SWAP @ ADC INPUT */
		mod_phy_reg(pi, SSLPNPHY_rxfe,
			SSLPNPHY_rxfe_swap_rxfiltout_iq_MASK,
			0 << SSLPNPHY_rxfe_swap_rxfiltout_iq_SHIFT);
	}
	}

	if (!mode && (phybw40 == 1)) {
		write_phy_reg(pi, SSLPNPHY_adc_2x, 0);
	} else {
		/* Setting adc in 2x mode */
		write_phy_reg(pi, SSLPNPHY_adc_2x, 0x7);
	}

#ifdef SSLPNLOWPOWER
	if (!mode && (phybw40 == 0)) {
		write_phy_reg(pi, SSLPNPHY_adc_2x, 0x7);
	} else {
		/* Setting adc in 1x mode */
		write_phy_reg(pi, SSLPNPHY_adc_2x, 0x0);
	}
#endif
	/* Selecting pos-edge of dac clock for driving the samples to dac */
	mod_phy_reg(pi, SSLPNPHY_sslpnCtrl4,
		SSLPNPHY_sslpnCtrl4_flip_dacclk_edge_MASK,
		0 << SSLPNPHY_sslpnCtrl4_flip_dacclk_edge_SHIFT);

	/* Selecting neg-edge of adc clock for sampling the samples from adc (in adc-presync),
	 * to meet timing
	*/
	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_sslpnCtrl4,
			SSLPNPHY_sslpnCtrl4_flip_adcclk2x_edge_MASK |
			SSLPNPHY_sslpnCtrl4_flip_adcclk1x_edge_MASK,
			(1 << SSLPNPHY_sslpnCtrl4_flip_adcclk2x_edge_SHIFT)|
			(1 << SSLPNPHY_sslpnCtrl4_flip_adcclk1x_edge_SHIFT));
	} else {
		mod_phy_reg(pi, SSLPNPHY_sslpnCtrl4,
			SSLPNPHY_sslpnCtrl4_flip_adcclk2x_edge_MASK |
			SSLPNPHY_sslpnCtrl4_flip_adcclk1x_edge_MASK,
			(0 << SSLPNPHY_sslpnCtrl4_flip_adcclk2x_edge_SHIFT)|
			(0 << SSLPNPHY_sslpnCtrl4_flip_adcclk1x_edge_SHIFT));
	}

	 /* Selecting pos-edge of 80Mhz phy clock for sampling the samples
	  * from adc (in adc-presync)
	  */
	mod_phy_reg(pi, SSLPNPHY_sslpnCtrl4,
		SSLPNPHY_sslpnCtrl4_flip_adcclk2x_80_edge_MASK |
		SSLPNPHY_sslpnCtrl4_flip_adcclk1x_80_edge_MASK,
		((0 << SSLPNPHY_sslpnCtrl4_flip_adcclk2x_80_edge_SHIFT)|
		(0 << SSLPNPHY_sslpnCtrl4_flip_adcclk1x_80_edge_SHIFT)));

	 /* Selecting pos-edge of aux-adc clock, 80Mhz phy clock for sampling the samples
	  * from aux adc (in auxadc-presync)
	  */
	mod_phy_reg(pi, SSLPNPHY_sslpnCtrl4,
		SSLPNPHY_sslpnCtrl4_flip_auxadcclk_edge_MASK |
		SSLPNPHY_sslpnCtrl4_flip_auxadcclkout_edge_MASK |
		SSLPNPHY_sslpnCtrl4_flip_auxadcclk80_edge_MASK,
		((0 << SSLPNPHY_sslpnCtrl4_flip_auxadcclk_edge_SHIFT) |
		(0 << SSLPNPHY_sslpnCtrl4_flip_auxadcclkout_edge_SHIFT) |
		(0 << SSLPNPHY_sslpnCtrl4_flip_auxadcclk80_edge_SHIFT)));
	/* Setting the adc-presync mux to select the samples registered with adc-clock */
	mod_phy_reg(pi, SSLPNPHY_sslpnAdcCtrl,
		SSLPNPHY_sslpnAdcCtrl_sslpnAdcCtrlMuxAdc2x_MASK |
		SSLPNPHY_sslpnAdcCtrl_sslpnAdcCtrlMuxAdc1x_MASK,
		((mode << SSLPNPHY_sslpnAdcCtrl_sslpnAdcCtrlMuxAdc2x_SHIFT) |
		(mode << SSLPNPHY_sslpnAdcCtrl_sslpnAdcCtrlMuxAdc1x_SHIFT)));

	 /* Setting the auxadc-presync muxes to select
	  * the samples registered with auxadc-clockout
	  */
	mod_phy_reg(pi, SSLPNPHY_sslpnAuxAdcCtrl,
		SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl0_MASK |
		SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl1_MASK |
		SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl2_MASK,
		((0 << SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl0_SHIFT) |
		(1 << SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl1_SHIFT) |
		(1 << SSLPNPHY_sslpnAuxAdcCtrl_sslpnAuxAdcMuxCtrl2_SHIFT)));

	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
		1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
		0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	wlc_sslpnphy_toggle_afe_pwdn(pi);
}


OSTATIC void
BCMOVERLAYFN(1, InitIntpapdlut)(uint8 Max, uint8 Min, uint8 *papdIntlutVld)
{
	uint16 a;

	for (a = Min; a <= Max; a++) {
		papdIntlutVld[a] = 0;
	}
}

STATIC void
BCMROMOVERLAYFN(1, wlc_sslpnphy_saveIntpapdlut)(phy_info_t *pi, int8 Max,
	int8 Min, uint32 *papdIntlut, uint8 *papdIntlutVld)
{
	phytbl_info_t tab;
	uint16 a;

	tab.tbl_id = SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL;
	tab.tbl_width = 32;     /* 32 bit wide */

	/* Max should be in range of 0 to 127 */
	/* Min should be in range of 0 to 126 */
	/* else no updates are available */

	if ((Min < 64) && (Max >= 0)) {
		Max = Max * 2 + 1;
		Min = Min * 2;

		tab.tbl_ptr = papdIntlut + Min; /* ptr to buf */
		tab.tbl_len = Max - Min + 1;        /* # values   */
		tab.tbl_offset = Min; /* tbl offset */
		wlc_sslpnphy_read_table(pi, &tab);

		for (a = Min; a <= Max; a++) {
			papdIntlutVld[a] = 1;
		}
	}
}

STATIC void
BCMROMOVERLAYFN(1, wlc_sslpnphy_GetpapdMaxMinIdxupdt)(phy_info_t *pi,
	int8 *maxUpdtIdx,
	int8 *minUpdtIdx)
{
	uint16 papd_lut_index_updt_63_48, papd_lut_index_updt_47_32;
	uint16 papd_lut_index_updt_31_16, papd_lut_index_updt_15_0;
	int8 MaxIdx, MinIdx;
	uint8 MaxIdxUpdated, MinIdxUpdated;
	uint8 i;

	papd_lut_index_updt_63_48 = read_phy_reg(pi, SSLPNPHY_papd_lut_index_updated_63_48);
	papd_lut_index_updt_47_32 = read_phy_reg(pi, SSLPNPHY_papd_lut_index_updated_47_32);
	papd_lut_index_updt_31_16 = read_phy_reg(pi, SSLPNPHY_papd_lut_index_updated_31_16);
	papd_lut_index_updt_15_0  = read_phy_reg(pi, SSLPNPHY_papd_lut_index_updated_15_0);

	MaxIdx = 63;
	MinIdx = 0;
	MinIdxUpdated = 0;
	MaxIdxUpdated = 0;

	for (i = 0; i < 16 && MinIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_15_0 & (1 << i)) == 0) {
				if (MinIdxUpdated == 0)
					MinIdx = MinIdx + 1;
			} else {
				MinIdxUpdated = 1;
			}
	}
	for (; i < 32 && MinIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_31_16 & (1 << (i - 16))) == 0) {
				if (MinIdxUpdated == 0)
					MinIdx = MinIdx + 1;
			} else {
				MinIdxUpdated = 1;
			}
	}
	for (; i < 48 && MinIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_47_32 & (1 << (i - 32))) == 0) {
				if (MinIdxUpdated == 0)
					MinIdx = MinIdx + 1;
			} else {
				MinIdxUpdated = 1;
			}
	}
	for (; i < 64 && MinIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_63_48 & (1 << (i - 48))) == 0) {
				if (MinIdxUpdated == 0)
					MinIdx = MinIdx + 1;
			} else {
				MinIdxUpdated = 1;
			}
	}

	/* loop for getting max index updated */
	for (i = 0; i < 16 && MaxIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_63_48 & (1 << (15 - i))) == 0) {
				if (MaxIdxUpdated == 0)
					MaxIdx = MaxIdx - 1;
			} else {
				MaxIdxUpdated = 1;
			}
	}
	for (; i < 32 && MaxIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_47_32 & (1 << (31 - i))) == 0) {
				if (MaxIdxUpdated == 0)
					MaxIdx = MaxIdx - 1;
			} else {
				MaxIdxUpdated = 1;
			}
	}
	for (; i < 48 && MaxIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_31_16 & (1 << (47 - i))) == 0) {
				if (MaxIdxUpdated == 0)
					MaxIdx = MaxIdx - 1;
			} else {
				MaxIdxUpdated = 1;
			}
	}
	for (; i < 64 && MaxIdxUpdated == 0; i++) {
			if ((papd_lut_index_updt_15_0 & (1 << (63 - i))) == 0) {
				if (MaxIdxUpdated == 0)
					MaxIdx = MaxIdx - 1;
			} else {
				MaxIdxUpdated = 1;
			}
	}
	*maxUpdtIdx = MaxIdx;
	*minUpdtIdx = MinIdx;
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_compute_delta)(phy_info_t *pi)
{
	uint32 papdcompdeltatblval;
	uint8 b;
	uint8 present, next;
	uint32 present_comp, next_comp;
	int32 present_comp_I, present_comp_Q;
	int32 next_comp_I, next_comp_Q;
	int32 delta_I, delta_Q;

	/* Writing Deltas */
	for (b = 0; b <= 124; b = b + 2) {
		present = b + 1;
		next = b + 3;

		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
			&papdcompdeltatblval, 1, 32, present);
		present_comp = papdcompdeltatblval;

		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
			&papdcompdeltatblval, 1, 32, next);
		next_comp = papdcompdeltatblval;

		present_comp_I = (present_comp & 0x00fff000) << 8;
		present_comp_Q = (present_comp & 0x00000fff) << 20;

		present_comp_I = present_comp_I >> 20;
		present_comp_Q = present_comp_Q >> 20;

		next_comp_I = (next_comp & 0x00fff000) << 8;
		next_comp_Q = (next_comp & 0x00000fff) << 20;

		next_comp_I = next_comp_I >> 20;
		next_comp_Q = next_comp_Q >> 20;

		delta_I = next_comp_I - present_comp_I;
		delta_Q = next_comp_Q - present_comp_Q;

		if (delta_I > 2048)
			delta_I = 2048;
		else if (delta_I < -2048)
			delta_I = -2048;

		if (delta_Q > 2048)
			delta_Q = 2048;
		else if (delta_Q < -2048)
			delta_Q = -2048;

		papdcompdeltatblval = ((delta_I << 12) & 0xfff000) | (delta_Q & 0xfff);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
			&papdcompdeltatblval, 1, 32, b);
	}
}

OSTATIC void
BCMOVERLAYFN(1, genpapdlut)(phy_info_t *pi, uint32 *papdIntlut, uint8 *papdIntlutVld)
{
	uint32 papdcompdeltatblval;
	uint8 a;

	papdcompdeltatblval = 128 << 12;

	wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
		&papdcompdeltatblval, 1, 32, 1);

	for (a = 3; a < 128; a = a + 2) {
		if (papdIntlutVld[a] == 1) {
			papdcompdeltatblval = papdIntlut[a];
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&papdcompdeltatblval, 1, 32, a);
		} else {
			wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&papdcompdeltatblval, 1, 32, a - 2);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&papdcompdeltatblval, 1, 32, a);
		}
	}
	/* Writing Delta */
	wlc_sslpnphy_compute_delta(pi);
}
static void
wlc_sslpnphy_pre_papd_cal_setup(phy_info_t *pi, sslpnphy_txcalgains_t *txgains, bool restore);

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_cal)(
	phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	sslpnphy_txcalgains_t *txgains,
	bool frcRxGnCtrl,
	uint16 num_symbols,
	uint8 papd_lastidx_search_mode)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint16 AphyControl_old;
	uint32 rxGnIdx;
	uint32 tmpVar;
	uint32 refTxAnGn;

	uint8 papd_peak_curr_mode = 1;
	uint8 lpgn_ovr;
	uint8 peak_curr_num_symbols_th;
	uint16 bbmult_init, bbmult_step;
	int8 maxUpdtIdx, minUpdtIdx;
	uint16 LPGN_I, LPGN_Q;
	uint16 tmp;
	uint32 bbmult_init_tmp;
	uint16 bbmult_list[16];
	uint8 stepcnt;
	uint8 curstep;
	uint8 prevstep;
	uint8 outofrange, cnt_search_64indx;
	uint16 papd_lut_index_updt_63_48;
	uint16 rem_symb;
	int32 volt_start, volt_end;
	uint8 counter = 0;
	ASSERT((cal_type == SSLPNPHY_PAPD_CAL_CW) || (cal_type == SSLPNPHY_PAPD_CAL_OFDM));

	WL_PHYCAL(("Running papd cal, channel: %d cal type: %d\n",
		CHSPEC_CHANNEL(pi->radio_chanspec),
		cal_type));


	if (0) {
	/* Disable CRS */
	wlc_sslpnphy_set_deaf(pi);

	/* Force WLAN antenna */
	if (!NON_BT_CHIP(wlc))
		wlc_sslpnphy_btcx_override_enable(pi);
	}

	if (0) {
	/* enables phy loopback */
	AphyControl_old = read_phy_reg(pi, SSLPNPHY_AphyControlAddr);
	mod_phy_reg(pi, SSLPNPHY_AphyControlAddr,
		SSLPNPHY_AphyControlAddr_phyloopbackEn_MASK,
		1 << SSLPNPHY_AphyControlAddr_phyloopbackEn_SHIFT);
	}


	wlc_sslpnphy_pre_papd_cal_setup(pi, txgains, FALSE);
	/* Do Rx Gain Control */
	wlc_sslpnphy_papd_cal_setup_cw(pi);
	rxGnIdx = wlc_sslpnphy_papd_rxGnCtrl(pi, cal_type, frcRxGnCtrl,
		sslpnphy_specific->sslpnphy_store.CurTxGain);

	/* Set Rx Gain */
	wlc_sslpnphy_set_rx_gain_by_distribution(pi, (uint16)rxGnIdx, 0, 0, 0, 0, 0, 0);

	/* clear our PAPD Compensation table */
	wlc_sslpnphy_clear_papd_comptable(pi);

	/* Do PAPD Operation */
	if (papd_peak_curr_mode == 1) {
		lpgn_ovr = 0;
		peak_curr_num_symbols_th = 70;
		bbmult_init = 1400;
		bbmult_step = 16640;
		if (VBAT_RIPPLE_CHECK(pi)) {
			counter = 0;
			do {
				if (counter >= 5)
					break;
				volt_start = wlc_sslpnphy_vbatsense(pi);
				wlc_sslpnphy_papd_cal_core(pi, cal_type,
					FALSE,
					peak_curr_num_symbols_th,
					1,
					bbmult_init,
					bbmult_step,
					0,
					128,
					0);

				volt_end = wlc_sslpnphy_vbatsense(pi);
				if ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
					(volt_end < sslpnphy_specific->sslpnphy_volt_winner)) {
					OSL_DELAY(300);
					counter ++;
				}
			} while ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
				(volt_end < sslpnphy_specific->sslpnphy_volt_winner));
		} else {
			wlc_sslpnphy_papd_cal_core(pi, cal_type,
				FALSE,
				peak_curr_num_symbols_th,
				1,
				bbmult_init,
				bbmult_step,
				0,
				128,
				0);
		}

		if (0) {
if (0)
{
			LPGN_I = read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_i);
			LPGN_Q = read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_q);

			for (tmp = 0; tmp < 110; tmp++) {
				bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
				if (bbmult_init_tmp >= 65535) {
					bbmult_init = 65535;
				} else {
					bbmult_init = (uint16) bbmult_init_tmp;
				}
			}
			bbmult_list[0] = bbmult_init;
			stepcnt = 110;
			for (tmp = 1; tmp < 16; tmp++) {
				bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
				if (bbmult_init_tmp >= 65535) {
					bbmult_init = 65535;
				} else {
					bbmult_init = (uint16) bbmult_init_tmp;
				}
				bbmult_list[tmp] = bbmult_init;
				stepcnt++;
			}
			curstep = stepcnt;
			prevstep = stepcnt;
			outofrange = 1;
			papd_lut_index_updt_63_48 = read_phy_reg(pi,
				SSLPNPHY_papd_lut_index_updated_63_48);
			cnt_search_64indx = 0;
			while (papd_lut_index_updt_63_48 < 32768 || outofrange) {
				cnt_search_64indx++;
				if (cnt_search_64indx > 15)
					break;
				lpgn_ovr = 1;
				counter = 0;
				do {
					if (counter >= 5)
						break;
					volt_start = wlc_sslpnphy_vbatsense(pi);
					wlc_sslpnphy_papd_cal_core(pi, cal_type,
						FALSE,
						0,
						1,
						bbmult_init,
						bbmult_step,
						lpgn_ovr,
						LPGN_I,
						LPGN_Q);

					volt_end = wlc_sslpnphy_vbatsense(pi);
					if ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
						(volt_end < sslpnphy_specific->sslpnphy_volt_winner)) {

						OSL_DELAY(600);
						counter ++;
					}
				} while ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
					(volt_end < sslpnphy_specific->sslpnphy_volt_winner));
				papd_lut_index_updt_63_48 = read_phy_reg(pi,
					SSLPNPHY_papd_lut_index_updated_63_48);

				if (papd_lut_index_updt_63_48 >= 32768) {
					if (outofrange) {

					curstep -= 5;
					if (curstep == prevstep)
						curstep--;
					if (curstep <= 125 && curstep >= 110)
						bbmult_init = bbmult_list[curstep-110];
					else {
						bbmult_init = 1400;
						for (tmp = 0; tmp < curstep; tmp++) {
						bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
						if (bbmult_init_tmp >= 65535)
							bbmult_init = 65535;
						else
							bbmult_init = (uint16) bbmult_init_tmp;
						}
					}
					} else {
						break;
					}
				} else if (papd_lut_index_updt_63_48 >= 8192) {
					outofrange = 0;
					bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
					if (bbmult_init_tmp >= 65535) {
						bbmult_init = 65535;
					} else {
						bbmult_init = (uint16) bbmult_init_tmp;
					}
					curstep++;
				} else {
					uint8 loop_limit;
					wlc_sslpnphy_GetpapdMaxMinIdxupdt(pi, &maxUpdtIdx,
						&minUpdtIdx);
					maxUpdtIdx = 2 * maxUpdtIdx + 1;
					minUpdtIdx = 2 * minUpdtIdx;
					loop_limit = 61 - (maxUpdtIdx - 1) / 2;
					for (tmp = 0; tmp < loop_limit; tmp++) {
						bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
						if (bbmult_init_tmp >= 65535) {
							bbmult_init = 65535;
						} else {
							bbmult_init = (uint16) bbmult_init_tmp;
						}
					}
					prevstep = curstep;
					curstep += loop_limit;
				}
			}
}
		} else {
			wlc_sslpnphy_GetpapdMaxMinIdxupdt(pi, &maxUpdtIdx, &minUpdtIdx);

			InitIntpapdlut(127, 0, sslpnphy_specific->sslpnphy_papdIntlutVld);
			wlc_sslpnphy_saveIntpapdlut(pi, maxUpdtIdx, minUpdtIdx,
				sslpnphy_specific->sslpnphy_papdIntlut, sslpnphy_specific->sslpnphy_papdIntlutVld);

			LPGN_I = read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_i);
			LPGN_Q = read_phy_reg(pi, SSLPNPHY_papd_loop_gain_cw_q);

			if (papd_lastidx_search_mode == 1) {
				for (tmp = 0; tmp < 219; tmp++) {
					bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
					if (bbmult_init_tmp >= 65535) {
						bbmult_init = 65535;
					} else {
						bbmult_init = (uint16) bbmult_init_tmp;
					}
				}
				rem_symb = 1;
			} else {
				for (tmp = 0; tmp < peak_curr_num_symbols_th; tmp++) {
					bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
					if (bbmult_init_tmp >= 65535) {
						bbmult_init = 65535;
					} else {
						bbmult_init = (uint16) bbmult_init_tmp;
					}
				}
				rem_symb = num_symbols- peak_curr_num_symbols_th;
			}
			while (rem_symb != 0) {
				lpgn_ovr = 1;
				bbmult_init_tmp = (bbmult_init * bbmult_step) >> 14;
				if (bbmult_init_tmp >= 65535) {
					bbmult_init = 65535;
				} else {
					bbmult_init = (uint16) bbmult_init_tmp;
				}
				if (VBAT_RIPPLE_CHECK(pi)) {
					counter = 0;
					do {
						if (counter >= 5)
							break;
						volt_start = wlc_sslpnphy_vbatsense(pi);
						wlc_sslpnphy_papd_cal_core(pi, cal_type,
							FALSE,
							0,
							1,
							bbmult_init,
							bbmult_step,
							lpgn_ovr,
							LPGN_I,
							LPGN_Q);

						volt_end = wlc_sslpnphy_vbatsense(pi);
						if ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
							(volt_end < sslpnphy_specific->sslpnphy_volt_winner)) {
							OSL_DELAY(600);
							counter ++;
						}
					} while ((volt_start < sslpnphy_specific->sslpnphy_volt_winner) ||
						(volt_end < sslpnphy_specific->sslpnphy_volt_winner));
				} else {
					wlc_sslpnphy_papd_cal_core(pi, cal_type,
						FALSE,
						0,
						1,
						bbmult_init,
						bbmult_step,
						lpgn_ovr,
						LPGN_I,
						LPGN_Q);
				}

				wlc_sslpnphy_GetpapdMaxMinIdxupdt(pi, &maxUpdtIdx, &minUpdtIdx);
				wlc_sslpnphy_saveIntpapdlut(pi, maxUpdtIdx, minUpdtIdx,
					sslpnphy_specific->sslpnphy_papdIntlut, sslpnphy_specific->sslpnphy_papdIntlutVld);
				maxUpdtIdx = 2 * maxUpdtIdx + 1;
				minUpdtIdx = 2 * minUpdtIdx;
				if (maxUpdtIdx > 0) {
				wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
					&refTxAnGn, 1, 32, maxUpdtIdx);
				}
				if (maxUpdtIdx == 127)
					break;

				rem_symb = rem_symb - 1;
			}
			genpapdlut(pi, sslpnphy_specific->sslpnphy_papdIntlut, sslpnphy_specific->sslpnphy_papdIntlutVld);
		}
	} else {
	if (0)
		{
		wlc_sslpnphy_papd_cal_core(pi, cal_type,
			FALSE,
			219,
			1,
			1400,
			16640,
			0,
			128,
			0);

		if (cal_type == SSLPNPHY_PAPD_CAL_CW) {
			wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 125);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 127);

			tmpVar = 0;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 124);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 126);


			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 0);
			wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 3);
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&tmpVar, 1, 32, 1);
		}
	}
	}

	WL_PHYCAL(("wl%d: %s: PAPD cal completed\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	if (0) {
	write_phy_reg(pi, SSLPNPHY_AphyControlAddr, AphyControl_old);
	}

	if (0) {
	/* Restore CRS */
	wlc_sslpnphy_clear_deaf(pi);
	}
	wlc_sslpnphy_pre_papd_cal_setup(pi, txgains, TRUE);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_pre_papd_cal_setup)(phy_info_t *pi,
                                               sslpnphy_txcalgains_t *txgains,
                                               bool restore)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint16  rf_common_02_old, rf_common_07_old;
	uint32 refTxAnGn;
#ifdef BAND5G
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
#endif
	if (!restore) {
		sslpnphy_specific->sslpnphy_store.bb_mult_old = wlc_sslpnphy_get_bbmult(pi);
		wlc_sslpnphy_tx_pu(pi, TRUE);
		wlc_sslpnphy_rx_pu(pi, TRUE);
		sslpnphy_specific->sslpnphy_store.lpfbwlut0 = read_phy_reg(pi, SSLPNPHY_lpfbwlutreg0);
		sslpnphy_specific->sslpnphy_store.lpfbwlut1 = read_phy_reg(pi, SSLPNPHY_lpfbwlutreg1);
		sslpnphy_specific->sslpnphy_store.rf_txbb_sp_3 = read_radio_reg(pi, RADIO_2063_TXBB_SP_3);
		sslpnphy_specific->sslpnphy_store.rf_pa_ctrl_14 = read_radio_reg(pi, RADIO_2063_PA_CTRL_14);
		/* Widen tx filter */
		wlc_sslpnphy_set_tx_filter_bw(pi, 5);
		sslpnphy_specific->sslpnphy_store.CurTxGain = 0; /* crk: Need to fill this correctly */
		/* Set tx gain */
		if (txgains) {
			if (txgains->useindex) {
				wlc_sslpnphy_set_tx_pwr_by_index(pi, txgains->index);
				sslpnphy_specific->sslpnphy_store.CurTxGain = txgains->index;
			} else {
				wlc_sslpnphy_set_tx_gain(pi, &txgains->gains);
			}
		}
		/* Set TR switch to transmit */
		/* wlc_sslpnphy_set_trsw_override(pi, FALSE, FALSE); */
		/* Set Rx path mux to PAPD and turn on PAPD mixer */
		sslpnphy_specific->sslpnphy_store.rxbb_ctrl2_old = read_radio_reg(pi, RADIO_2063_RXBB_CTRL_2);
		{
			int aa;
#ifdef PHYHAL
			aa = (int8)ANT_AVAIL(pi->aa2g);
#else	
			aa = (int8)ANT_AVAIL(pi->sh->ant_avail_aa2g);
#endif /* PHYHAL */
			if (CHSPEC_IS2G(pi->radio_chanspec)) {
				if ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA) || (aa >= 2) ||
					/* Askey case where there is -no TR switch, dedicated */
					/* antenna for TX and RX */
				((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319WLUSBN4L_SSID) && (aa == 1))) {
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2,
						(3 << 3), (uint8)(2 << 3));
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2,
						(1 << 1), (uint8)(1 << 1));
				} else {
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, (3 << 3),
						(uint8)(3 << 3));
				}
#ifndef BAND5G
		}
#else
			} else {
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
					BCM94329MOTOROLA_SSID) ||
					(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID) ||
					(freq >= 5500)) {
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2,
						(3 << 3), (uint8)(2 << 3));
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2,
						(1 << 1), (uint8)(1 << 1));
				} else {
					mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, (3 << 3),
						(uint8)(3 << 3));
				}
			}
#endif /* BAND5G */
		}
		/* turn on PAPD mixer */
		/* no overide for bit 4 & 5 */
		rf_common_02_old = read_radio_reg(pi, RADIO_2063_COMMON_02);
		rf_common_07_old = read_radio_reg(pi, RADIO_2063_COMMON_07);
		or_radio_reg(pi, RADIO_2063_COMMON_02, 0x1);
		or_radio_reg(pi, RADIO_2063_COMMON_07, 0x18);
		sslpnphy_specific->sslpnphy_store.pa_sp1_old_5_4 = (read_radio_reg(pi,
			RADIO_2063_PA_SP_1)) & (3 << 4);
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			mod_radio_reg(pi, RADIO_2063_PA_SP_1, (3 << 4), (uint8)(2 << 4));
		} else {
			mod_radio_reg(pi, RADIO_2063_PA_SP_1, (3 << 4), (uint8)(1 << 4));
		}
		write_radio_reg(pi, RADIO_2063_COMMON_02, rf_common_02_old);
		write_radio_reg(pi, RADIO_2063_COMMON_07, rf_common_07_old);
		wlc_sslpnphy_afe_clk_init(pi, AFE_CLK_INIT_MODE_PAPD);
		sslpnphy_specific->sslpnphy_store.Core1TxControl_old = read_phy_reg(pi, SSLPNPHY_Core1TxControl);
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			SSLPNPHY_Core1TxControl_BphyFrqBndSelect_MASK	|
			SSLPNPHY_Core1TxControl_iqImbCompEnable_MASK	|
			SSLPNPHY_Core1TxControl_loft_comp_en_MASK,
			(1 << SSLPNPHY_Core1TxControl_BphyFrqBndSelect_SHIFT)	|
			(1 << SSLPNPHY_Core1TxControl_iqImbCompEnable_SHIFT)	|
			(1 << SSLPNPHY_Core1TxControl_loft_comp_en_SHIFT));
		/* in SSLPNPHY, we need to bring SPB out of standby before using it */
		sslpnphy_specific->sslpnphy_store.sslpnCtrl3_old = read_phy_reg(pi, SSLPNPHY_sslpnCtrl3);
		mod_phy_reg(pi, SSLPNPHY_sslpnCtrl3,
			SSLPNPHY_sslpnCtrl3_sram_stby_MASK,
			0 << SSLPNPHY_sslpnCtrl3_sram_stby_SHIFT);
		sslpnphy_specific->sslpnphy_store.SSLPNPHY_sslpnCalibClkEnCtrl_old = read_phy_reg(pi,
			SSLPNPHY_sslpnCalibClkEnCtrl);
		or_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, 0x8f);
		/* Set PAPD reference analog gain */
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL,
			&refTxAnGn, 1, 32,
			SSLPNPHY_TX_PWR_CTRL_PWR_OFFSET + txgains->index);
		refTxAnGn = refTxAnGn * 8;
		write_phy_reg(pi, SSLPNPHY_papd_tx_analog_gain_ref,
			(uint16)refTxAnGn);
		/* Turn off LNA */
		sslpnphy_specific->sslpnphy_store.rf_common_03_old = read_radio_reg(pi, RADIO_2063_COMMON_03);
		rf_common_02_old = read_radio_reg(pi, RADIO_2063_COMMON_02);
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			or_radio_reg(pi, RADIO_2063_COMMON_03, 0x18);
			or_radio_reg(pi, RADIO_2063_COMMON_02, 0x2);
			sslpnphy_specific->sslpnphy_store.rf_grx_sp_1_old = read_radio_reg(pi,
				RADIO_2063_GRX_SP_1);
			write_radio_reg(pi, RADIO_2063_GRX_SP_1, 0x1e);
			/* sslpnphy_rx_pu sets some bits which needs */
			/* to be override here for papdcal . so dont reset common_03 */
			write_radio_reg(pi, RADIO_2063_COMMON_02, rf_common_02_old);
		}

	} else {

		/* restore saved registers */
		write_radio_reg(pi, RADIO_2063_COMMON_03, sslpnphy_specific->sslpnphy_store.rf_common_03_old);
		rf_common_02_old = read_radio_reg(pi, RADIO_2063_COMMON_02);
		or_radio_reg(pi, RADIO_2063_COMMON_03, 0x18);
		or_radio_reg(pi, RADIO_2063_COMMON_02, 0x2);
		write_radio_reg(pi, RADIO_2063_GRX_SP_1, sslpnphy_specific->sslpnphy_store.rf_grx_sp_1_old);
		write_radio_reg(pi, RADIO_2063_COMMON_03, sslpnphy_specific->sslpnphy_store.rf_common_03_old);
		write_radio_reg(pi, RADIO_2063_COMMON_02, rf_common_02_old);
		write_phy_reg(pi, SSLPNPHY_lpfbwlutreg0, sslpnphy_specific->sslpnphy_store.lpfbwlut0);
		write_phy_reg(pi, SSLPNPHY_lpfbwlutreg1, sslpnphy_specific->sslpnphy_store.lpfbwlut1);
		write_radio_reg(pi, RADIO_2063_TXBB_SP_3, sslpnphy_specific->sslpnphy_store.rf_txbb_sp_3);
		write_radio_reg(pi, RADIO_2063_PA_CTRL_14, sslpnphy_specific->sslpnphy_store.rf_pa_ctrl_14);
		write_phy_reg(pi, SSLPNPHY_Core1TxControl, sslpnphy_specific->sslpnphy_store.Core1TxControl_old);
		write_phy_reg(pi, SSLPNPHY_sslpnCtrl3, sslpnphy_specific->sslpnphy_store.sslpnCtrl3_old);
		/* restore calib ctrl clk */
		/* switch on PAPD clk */
		write_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
			sslpnphy_specific->sslpnphy_store.SSLPNPHY_sslpnCalibClkEnCtrl_old);
		wlc_sslpnphy_afe_clk_init(pi, AFE_CLK_INIT_MODE_TXRX2X);
		/* TR switch */
		wlc_sslpnphy_clear_trsw_override(pi);
		/* Restore rx path mux and turn off PAPD mixer */
		rf_common_02_old = read_radio_reg(pi, RADIO_2063_COMMON_02);
		rf_common_07_old = read_radio_reg(pi, RADIO_2063_COMMON_07);
		or_radio_reg(pi, RADIO_2063_COMMON_02, 0x1);
		or_radio_reg(pi, RADIO_2063_COMMON_07, 0x18);
		mod_radio_reg(pi, RADIO_2063_PA_SP_1, (3 << 4), sslpnphy_specific->sslpnphy_store.pa_sp1_old_5_4);
		write_radio_reg(pi, RADIO_2063_COMMON_02, rf_common_02_old);
		write_radio_reg(pi, RADIO_2063_COMMON_07, rf_common_07_old);
		write_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, sslpnphy_specific->sslpnphy_store.rxbb_ctrl2_old);
		/* Clear rx PU override */
		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK,
			0 << SSLPNPHY_RFOverride0_internalrfrxpu_ovr_SHIFT);
		wlc_sslpnphy_rx_pu(pi, FALSE);
		wlc_sslpnphy_tx_pu(pi, FALSE);
		/* Clear rx gain override */
		wlc_sslpnphy_rx_gain_override_enable(pi, FALSE);
		/* Clear ADC override */
		mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
			SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_MASK,
			0 << SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_SHIFT);
		/* restore bbmult */
		wlc_sslpnphy_set_bbmult(pi, sslpnphy_specific->sslpnphy_store.bb_mult_old);

	}
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_vbatsense_papd_cal)(
	phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	sslpnphy_txcalgains_t *txgains)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	int32 cnt, volt_high_cnt, volt_mid_cnt, volt_low_cnt;
	int32 volt_avg;
	int32 voltage_samples[50];
	int32 volt_high_thresh, volt_low_thresh;

	wlc_sslpnphy_pre_papd_cal_setup(pi, txgains, FALSE);

	{
		volt_high_cnt = 0;
		volt_low_cnt = 0;
		volt_mid_cnt = 0;

		volt_avg = 0;

		for (cnt = 0; cnt < 32; cnt++) {
			voltage_samples[cnt] = wlc_sslpnphy_vbatsense(pi);
			volt_avg += voltage_samples[cnt];
			OSL_DELAY(120);
			/* assuming a 100us time for executing wlc_sslpnphy_vbatsense */
		}
		volt_avg = volt_avg >> 5;

		volt_high_thresh = 0;
		volt_low_thresh = volt_avg;
		for (cnt = 0; cnt < 32; cnt++) {
			if (voltage_samples[cnt] > volt_high_thresh)
				volt_high_thresh = voltage_samples[cnt];
			if (voltage_samples[cnt] < volt_low_thresh)
				volt_low_thresh = voltage_samples[cnt];
		}
		/* for taking care of vhat dip conditions */

		sslpnphy_specific->sslpnphy_volt_low = (uint8)volt_low_thresh;
		sslpnphy_specific->sslpnphy_volt_winner = (uint8)(volt_high_thresh - 2);

	}


	sslpnphy_specific->sslpnphy_last_cal_voltage = volt_low_thresh;
	wlc_sslpnphy_pre_papd_cal_setup(pi, txgains, TRUE);
}

OSTATIC int8
BCMOVERLAYFN(1, wlc_sslpnphy_gain_based_psat_detect)(phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type, bool frcRxGnCtrl,
	sslpnphy_txcalgains_t *txgains,	uint8 cur_pwr)
{
	phytbl_info_t tab;
	uint8 papd_lastidx_search_mode = 0;
	int32 Re_div_Im = 60000;
	int32 lowest_gain_diff_local = 59999;
	int32 thrsh_gain = 67600;
	uint16 thrsh_pd = 180;
	int32 gain, gain_diff, psat_check_gain = 0;
	uint32 temp_offset;
	uint32 temp_read[128];
	uint32 papdcompdeltatblval;
	int32 papdcompRe, psat_check_papdcompRe = 0;
	int32 papdcompIm, psat_check_papdcompIm = 0;
	uint8 max_gain_idx = 97;
	uint8 psat_thrsh_num, psat_detected = 0;
	uint8 papdlut_endidx = 97;
	uint8 cur_index = txgains->index;
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));

	tab.tbl_id = SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL;
	tab.tbl_ptr = temp_read;  /* ptr to buf */
	tab.tbl_width = 32;     /* 32 bit wide */
	tab.tbl_len = 87;        /* # values   */
	tab.tbl_offset = 11;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		tab.tbl_len = 47;        /* # values   */
		tab.tbl_offset = 81;
		papdlut_endidx = 127;
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2))
			thrsh_gain = 25600;
		else
			thrsh_gain = 40000;
		thrsh_pd = 350;
	}
	wlc_sslpnphy_papd_cal(pi, cal_type, txgains,
		frcRxGnCtrl, 219,
		papd_lastidx_search_mode);
	wlc_sslpnphy_read_table(pi, &tab);
	for (temp_offset = 0; temp_offset < tab.tbl_len; temp_offset += 2) {
		papdcompdeltatblval = temp_read[temp_offset];
		papdcompRe = (papdcompdeltatblval & 0x00fff000) << 8;
		papdcompIm = (papdcompdeltatblval & 0x00000fff) << 20;
		papdcompRe = (papdcompRe >> 20);
		papdcompIm = (papdcompIm >> 20);
		gain = papdcompRe * papdcompRe + papdcompIm * papdcompIm;
		if (temp_offset == (tab.tbl_len - 1)) {
			psat_check_gain = gain;
			psat_check_papdcompRe = papdcompRe;
			psat_check_papdcompIm = papdcompIm;
		}
		gain_diff = gain - thrsh_gain;
		if (gain_diff < 0) {
			gain_diff = gain_diff * (-1);
		}
		if ((gain_diff < lowest_gain_diff_local) || (temp_offset == 0)) {
			sslpnphy_specific->sslpnphy_max_gain = gain;
			max_gain_idx = tab.tbl_offset + temp_offset;
			lowest_gain_diff_local = gain_diff;
		}
	}
	/* Psat Calculation based on gain threshold */
	if (psat_check_gain >= thrsh_gain)
		psat_detected = 1;
	if (psat_detected == 0) {
		/* Psat Calculation based on PD threshold */
		if (psat_check_papdcompIm != 0) {
			if (psat_check_papdcompIm < 0)
				psat_check_papdcompIm = psat_check_papdcompIm * -1;
			Re_div_Im = (psat_check_papdcompRe / psat_check_papdcompIm) * 100;
		} else {
			Re_div_Im = 60000;
		}
		if (Re_div_Im < thrsh_pd)
			psat_detected = 1;
	}
	if ((psat_detected == 0) && (cur_index <= 4)) {
		psat_detected = 1;
	}
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID)
			|| (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)) { /* ninja */
			max_gain_idx = max_gain_idx + 16;
		} else {
			if (freq < 5640)
				max_gain_idx = max_gain_idx + 6;
			else if (freq <= 5825)
				max_gain_idx = max_gain_idx + 10;
		}
	}
	if (psat_detected) {
		sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr;
		sslpnphy_specific->sslpnphy_psat_indx = cur_index;
		psat_thrsh_num = (papdlut_endidx - max_gain_idx)/ 2;
		if (psat_thrsh_num > 6) {
			sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 2;
			sslpnphy_specific->sslpnphy_psat_indx = cur_index + 8;
		} else if (psat_thrsh_num > 2) {
			sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 1;
			sslpnphy_specific->sslpnphy_psat_indx = cur_index + 4;
		}
	}

	if ((lowest_gain_diff_local < sslpnphy_specific->sslpnphy_lowest_gain_diff) || (cur_pwr == 17)) {
		sslpnphy_specific->sslpnphy_final_papd_cal_idx = txgains->index +
			(papdlut_endidx - max_gain_idx)/2;
		sslpnphy_specific->sslpnphy_lowest_gain_diff = lowest_gain_diff_local;
	}
	return psat_detected;
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_min_pd_search)(phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	bool frcRxGnCtrl,
	sslpnphy_txcalgains_t *txgains)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 papd_lastidx_search_mode = 0;
	int32 Re_div_Im = 60000;
	int32 lowest_Re_div_Im_local = 59999;
	int32 temp_offset;
	uint32 temp_read[30];
	uint32 papdcompdeltatblval;
	int32 papdcompRe;
	int32 papdcompIm;
	uint8 MinPdIdx = 127;
	uint8 tbl_offset = 101;
	wlc_sslpnphy_papd_cal(pi, cal_type, txgains,
		frcRxGnCtrl, 219,
		papd_lastidx_search_mode);
	wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
		temp_read, 27, 32, 101);
	for (temp_offset = 0; temp_offset < 27; temp_offset += 2) {
		papdcompdeltatblval = temp_read[temp_offset];
		papdcompRe = (papdcompdeltatblval & 0x00fff000) << 8;
		papdcompIm = (papdcompdeltatblval & 0x00000fff) << 20;
		papdcompRe = papdcompRe >> 20;
		papdcompIm = papdcompIm >> 20;
		if (papdcompIm < 0) {
			Re_div_Im = papdcompRe * 100 / papdcompIm * -1;
			if (Re_div_Im < lowest_Re_div_Im_local) {
				lowest_Re_div_Im_local = Re_div_Im;
				MinPdIdx = tbl_offset + temp_offset;
			}
		}
	}
	if (!sslpnphy_specific->sslpnphy_force_1_idxcal) {
		if (lowest_Re_div_Im_local < sslpnphy_specific->sslpnphy_lowest_Re_div_Im) {
			sslpnphy_specific->sslpnphy_final_papd_cal_idx = txgains->index + (127 - MinPdIdx)/2;
			sslpnphy_specific->sslpnphy_lowest_Re_div_Im = lowest_Re_div_Im_local;
		}
	} else {
		sslpnphy_specific->sslpnphy_final_papd_cal_idx = sslpnphy_specific->sslpnphy_papd_nxt_cal_idx;
		sslpnphy_specific->sslpnphy_lowest_Re_div_Im = lowest_Re_div_Im_local;
	}
}

OSTATIC int8
BCMOVERLAYFN(1, wlc_sslpnphy_psat_detect)(phy_info_t *pi,
	uint8 cur_index,
	uint8 cur_pwr)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 pd_ph_cnt = 0;
	int8 psat_detected = 0;
	uint32 temp_read[50];
	int32 temp_offset;
	uint32 papdcompdeltatblval;
	int32 papdcompRe;
	int32 papdcompIm;
	int32 voltage;
	bool gain_psat_det_in_phase_papd = 0;
	int32 thrsh_gain = 40000;
	int32 gain;
	uint thrsh1, thrsh2, pd_thresh = 0;
	uint gain_xcd_cnt = 0;
	uint32 num_elements = 0;
	voltage = wlc_sslpnphy_vbatsense(pi);
	if (voltage < 52)
		num_elements = 17;
	else
		num_elements = 39;
	thrsh1 = thrsh2 = 0;
	wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL, temp_read,
		num_elements, 32, (128 - num_elements));
	if (voltage < 52)
		num_elements = 17;
	else
		num_elements = 39;
	num_elements = num_elements -1;
	for (temp_offset = num_elements; temp_offset >= 0; temp_offset -= 2) {
		papdcompdeltatblval = temp_read[temp_offset];
		papdcompRe = (papdcompdeltatblval & 0x00fff000) << 8;
		papdcompIm = (papdcompdeltatblval & 0x00000fff) << 20;
		papdcompRe = papdcompRe >> 20;
		papdcompIm = papdcompIm >> 20;
		if (papdcompIm >= 0) {
			pd_ph_cnt ++;
			psat_detected = 1;
		}
		gain = (papdcompRe * papdcompRe) + (papdcompIm * papdcompIm);
		if (gain > thrsh_gain) {
			gain_xcd_cnt ++;
			psat_detected = 1;
		}

	}
	if (cur_pwr <= 21)
		pd_thresh = 12;
	if (cur_pwr <= 19)
		pd_thresh = 10;
	if (cur_pwr <= 17)
		pd_thresh = 8;
	if ((voltage > 52) && (cur_pwr >= 17) && (psat_detected == 1)) {
		gain_psat_det_in_phase_papd = 1;
		if (cur_pwr == 21) {
			thrsh1 = 8;
			thrsh2 = 12;
		} else {
			thrsh1 = 2;
			thrsh2 =  5;
		}
	}


	if (psat_detected == 1) {

		if (gain_psat_det_in_phase_papd == 0) {
			sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr;
			sslpnphy_specific->sslpnphy_psat_indx = cur_index;
			if (pd_ph_cnt > 2) {
				sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 1;
				sslpnphy_specific->sslpnphy_psat_indx = cur_index + 4;
			}
			if (pd_ph_cnt > 6) {
				sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 2;
				sslpnphy_specific->sslpnphy_psat_indx = cur_index + 8;
			}
		} else {
			if ((gain_xcd_cnt >  0) || (pd_ph_cnt > pd_thresh))  {
				sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr;
				sslpnphy_specific->sslpnphy_psat_indx = cur_index;
			} else {
				psat_detected = 0;
			}
			if ((gain_xcd_cnt >  thrsh1) || (pd_ph_cnt > (pd_thresh + 2))) {
				sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 1;
				sslpnphy_specific->sslpnphy_psat_indx = cur_index + 4;
			}
			if ((gain_xcd_cnt >  thrsh2) || (pd_ph_cnt > (pd_thresh + 6))) {
				sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr - 2;
				sslpnphy_specific->sslpnphy_psat_indx = cur_index + 8;
			}

		}
	}
	if ((psat_detected == 0) && (cur_index <= 4)) {
		psat_detected = 1;
		sslpnphy_specific->sslpnphy_psat_pwr = cur_pwr;
		sslpnphy_specific->sslpnphy_psat_indx = cur_index;
	}
	return (psat_detected);
}


/* Run PAPD cal at power level appropriate for tx gain table */
OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_cal_txpwr)(phy_info_t *pi,
	sslpnphy_papd_cal_type_t cal_type,
	bool frcRxGnCtrl,
	bool frcTxGnCtrl,
	uint16 frcTxidx)
{
	sslpnphy_txcalgains_t txgains;
	bool tx_gain_override_old;
	sslpnphy_txgains_t old_gains;

	uint8 bbmult_old;
	uint16 tx_pwr_ctrl_old;
	uint8 papd_lastidx_search_mode = 0;
	uint8 psat_detected = 0;
	uint8 psat_pwr = 255;
	uint8 TxIdx_14;
	int32 lowest_Re_div_Im;
	uint8 flag = 0;  /* keeps track of upto what dbm can the papd calib be done. */
	uint8 papd_gain_based = 0;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));

	if ((CHSPEC_IS5G(pi->radio_chanspec)) && (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA))
		return;

	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319LCUSBSDN4L_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319LCSDN4L_SSID) ||
	    (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319MLAP_SSID))
		papd_gain_based = 1;

	/* Initial gain based scheme enabled only for 4319 now */
	/* Verify if 4319 5G performance improve with new cal for class A operation */
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec))
		papd_gain_based = 1;
	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319MLAP_SSID)) {
		papd_gain_based = 1;
	}
#endif

	sslpnphy_specific->sslpnphy_lowest_Re_div_Im = 60000;
	sslpnphy_specific->sslpnphy_final_papd_cal_idx = 30;
	sslpnphy_specific->sslpnphy_psat_indx = 255;
	if (!sslpnphy_specific->sslpnphy_force_1_idxcal)
		sslpnphy_specific->sslpnphy_psat_pwr = 25;

	/* Save current bbMult and txPwrCtrl settings and turn txPwrCtrl off. */
	bbmult_old  = wlc_sslpnphy_get_bbmult(pi);

	/* Save original tx power control mode */
	tx_pwr_ctrl_old = wlc_sslpnphy_get_tx_pwr_ctrl(pi);

	/* Save old tx gains if needed */
	tx_gain_override_old = wlc_sslpnphy_tx_gain_override_enabled(pi);
	if (tx_gain_override_old)
		wlc_sslpnphy_get_tx_gain(pi, &old_gains);

	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);
	txgains.useindex = TRUE;
	if (!sslpnphy_specific->sslpnphy_force_1_idxcal) {
		if (frcTxGnCtrl)
			txgains.index = (uint8) frcTxidx;
		TxIdx_14 = txgains.index;
	#ifdef BAND5G
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			if (freq <= 5320)
				TxIdx_14 = 30;
			else
				TxIdx_14 = 45;

			if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) ||
				(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)) { /* ninja */
				TxIdx_14 = txgains.index + 20;
			}
		}
	#endif
		if (TxIdx_14 <= 0)
			flag = 13;
		else if (TxIdx_14 <= 28)
			flag = 14 + (int) ((TxIdx_14-1) >> 2);
		else if (TxIdx_14 > 28)
			flag = 21;

		/* If radio is tuned with class A settings, go for AM-AM based papd cals */
		if (!sslpnphy_specific->sslpnphy_radio_classA) {
			/* Cal at 17dBm */
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) != BCM94319WLUSBN4L_SSID) {
				if (flag >= 17) {
					txgains.index = TxIdx_14 - 12;
					if (papd_gain_based) {
						psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
							cal_type, FALSE, &txgains, 17);
					} else
					{
					wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
					psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 17);
					}
					/* Calib for 18dbm */
					if ((psat_detected == 0) && (flag == 18)) {
						txgains.index = TxIdx_14 - 16;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 18);
						} else
						{
						wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
						psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 18);
						}
					}
					/* Calib for 19dbm */
					if ((psat_detected == 0) && (flag >= 19)) {
						txgains.index = TxIdx_14 - 20;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 19);
						} else
						{
						wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
						psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 19);
						}
						/* Calib for 20dbm */
						if ((psat_detected == 0) && (flag == 20)) {
							txgains.index = TxIdx_14 - 24;
							if (papd_gain_based) {
								psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
									cal_type, FALSE, &txgains, 20);
							} else
							{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 20);
							}
						}
						/* Calib for 21dBm */
						if (psat_detected == 0 && flag >= 21) {
							txgains.index = TxIdx_14 - 28;
							if (papd_gain_based) {
								psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
									cal_type, FALSE, &txgains, 21);
							} else
							{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 21);
							}
						}
					} else {
						/* Calib for 13dBm */
						if ((psat_detected == 1) && (flag >= 13)) {
							txgains.index = TxIdx_14 + 4;
							if (papd_gain_based) {
								psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
									cal_type, FALSE, &txgains, 13);
							} else
							{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 13);
							}
						}
						/* Calib for 14dBm */
						if ((psat_detected == 0) && (flag == 14)) {
							txgains.index = TxIdx_14;
							if (papd_gain_based) {
								psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
									cal_type, FALSE, &txgains, 14);
							} else
							{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 14);
							}
						}
						/* Calib for 15dBm */
						if ((psat_detected == 0) && (flag >= 15)) {
							txgains.index = TxIdx_14 - 4;
							if (papd_gain_based) {
								psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
									cal_type, FALSE, &txgains, 15);
							} else
							{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 15);
							}
						}
					}
				} else {
					/* Calib for 13dBm */
					if ((flag >= 13) && (psat_detected == 0)) {
						if (TxIdx_14 < 2)
							txgains.index = TxIdx_14 + 1;
						else
							txgains.index = TxIdx_14 + 4;
						sslpnphy_specific->sslpnphy_psat_pwr = 13;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 13);
						} else
						{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 13);
						}
					}
					/* Calib for 14dBm */
					if ((flag >= 14) && (psat_detected == 0)) {
						txgains.index = TxIdx_14;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 14);
						} else
						{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 14);
						}
					}
					/* Calib for 15dBm */
					if ((flag >= 15) && (psat_detected == 0)) {
						txgains.index = TxIdx_14 - 4;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 15);
						} else
						{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 15);
						}
					}
					/* Calib for 16dBm */
					if ((flag == 16) && (psat_detected == 0)) {
						txgains.index = TxIdx_14 - 8;
						if (papd_gain_based) {
							psat_detected = wlc_sslpnphy_gain_based_psat_detect(pi,
								cal_type, FALSE, &txgains, 16);
						} else
						{
							wlc_sslpnphy_min_pd_search(pi, cal_type, FALSE, &txgains);
							psat_detected = wlc_sslpnphy_psat_detect(pi, txgains.index, 16);
						}
					}
				}
				/* Final PAPD Cal with selected Tx Gain */
				txgains.index =  sslpnphy_specific->sslpnphy_final_papd_cal_idx;
			} else {
				txgains.index = TxIdx_14 - 12;
			}
			wlc_sslpnphy_papd_cal(pi, cal_type, &txgains,
				frcRxGnCtrl, 219,
				papd_lastidx_search_mode);

		} else { /* sslpnphy_specific->sslpnphy_radio_classA */

			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) {
				txgains.index =  sslpnphy_specific->sslpnphy_start_idx;
				wlc_sslpnphy_papd_cal(pi, cal_type, &txgains,
					frcRxGnCtrl, 219,
					papd_lastidx_search_mode);
			} else {
				uint32 lastval;
				int32 lreal, limag;
				uint32 mag;
				uint32 final_idx_thresh = 32100;
				uint16 min_final_idx_thresh = 10000;
				uint8 start, stop, mid;
				final_idx_thresh = 37000; /* 1.5 */

				if (freq >= 5180)
					final_idx_thresh = 42000; /* 1.6 */

				if (sslpnphy_specific->sslpnphy_papd_tweaks_enable) {
					final_idx_thresh = sslpnphy_specific->sslpnphy_papd_tweaks.final_idx_thresh;
					min_final_idx_thresh = sslpnphy_specific->sslpnphy_papd_tweaks.min_final_idx_thresh;
				}

				txgains.useindex = TRUE;
				start = 0;
				stop = 90;
				while (1) {
					mid = (start + stop) / 2;
					txgains.index = mid;
					wlc_sslpnphy_papd_cal(pi, cal_type, &txgains,
						frcRxGnCtrl, 219,
						0);

					wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
						&lastval, 1, 32, 127);

					lreal = lastval & 0x00fff000;
					limag = lastval & 0x00000fff;
					lreal = lreal << 8;
					limag = limag << 20;
					lreal = lreal >> 20;
					limag = limag >> 20;

					mag = (lreal * lreal) + (limag * limag);
					if (mag <= final_idx_thresh) {
						stop = mid;
					} else {
						start = mid;
					}
					if (CHSPEC_IS2G(pi->radio_chanspec)) {
						if ((mag > (final_idx_thresh - min_final_idx_thresh)) && (mag < final_idx_thresh))
							break;
					}
					if ((stop - start) < 2)
						break;
				}
			}
		}
	} else {
		sslpnphy_specific->sslpnphy_psat_indx = sslpnphy_specific->sslpnphy_papd_nxt_cal_idx;
		txgains.index =  sslpnphy_specific->sslpnphy_papd_nxt_cal_idx;
		psat_pwr = flag;
		wlc_sslpnphy_papd_cal(pi, cal_type, &txgains,
			frcRxGnCtrl, 219,
			papd_lastidx_search_mode);
	}
	sslpnphy_specific->sslpnphy_11n_backoff = 0;
	sslpnphy_specific->sslpnphy_lowerofdm = 0;
	sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 0;
	sslpnphy_specific->sslpnphy_cck = 0;
	/* New backoff scheme */
	psat_pwr = sslpnphy_specific->sslpnphy_psat_pwr;
	lowest_Re_div_Im = sslpnphy_specific->sslpnphy_lowest_Re_div_Im;
	/* Taking a snap shot for debugging purpose */
	sslpnphy_specific->sslpnphy_psat_pwr = psat_pwr;
	sslpnphy_specific->sslpnphy_min_phase = lowest_Re_div_Im;
	sslpnphy_specific->sslpnphy_final_idx = txgains.index;

	/* Save papd lut and regs */
	wlc_sslpnphy_save_papd_calibration_results(pi);

	/* Restore tx power and reenable tx power control */
	if (tx_gain_override_old)
		wlc_sslpnphy_set_tx_gain(pi, &old_gains);
	wlc_sslpnphy_set_bbmult(pi, bbmult_old);
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl_old);
}

/*
* Get Rx IQ Imbalance Estimate from modem
*/
STATIC bool
BCMROMOVERLAYFN(1, wlc_sslpnphy_rx_iq_est)(phy_info_t *pi,
	uint16 num_samps,
	uint8 wait_time,
	sslpnphy_iq_est_t *iq_est)
{
	int wait_count = 0;
	bool result = TRUE;
	uint8 phybw40 = IS40MHZ(pi);

	/* Turn on clk to Rx IQ */
	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_SHIFT);

	/* Force OFDM receiver on */
	mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
		SSLPNPHY_crsgainCtrl_APHYGatingEnable_MASK,
		0 << SSLPNPHY_crsgainCtrl_APHYGatingEnable_SHIFT);

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
	if (phybw40 == 1) {
		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_APHYGatingEnable_MASK,
			0 << SSLPNPHY_Rev2_crsgainCtrl_40_APHYGatingEnable_SHIFT);
	}
	}

	mod_phy_reg(pi, SSLPNPHY_IQNumSampsAddress,
		SSLPNPHY_IQNumSampsAddress_numSamps_MASK,
		num_samps << SSLPNPHY_IQNumSampsAddress_numSamps_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_IQEnableWaitTimeAddress,
		SSLPNPHY_IQEnableWaitTimeAddress_waittimevalue_MASK,
		(uint16)wait_time << SSLPNPHY_IQEnableWaitTimeAddress_waittimevalue_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_IQEnableWaitTimeAddress,
		SSLPNPHY_IQEnableWaitTimeAddress_iqmode_MASK,
		0 << SSLPNPHY_IQEnableWaitTimeAddress_iqmode_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_IQEnableWaitTimeAddress,
		SSLPNPHY_IQEnableWaitTimeAddress_iqstart_MASK,
		1 << SSLPNPHY_IQEnableWaitTimeAddress_iqstart_SHIFT);

	/* Wait for IQ estimation to complete */
	while (read_phy_reg(pi, SSLPNPHY_IQEnableWaitTimeAddress) &
		SSLPNPHY_IQEnableWaitTimeAddress_iqstart_MASK) {
		/* Check for timeout */
		if (wait_count > (10 * 500)) { /* 500 ms */
			WL_ERROR(("wl%d: %s: IQ estimation failed to complete\n",
				GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
			result = FALSE;
			goto cleanup;
		}
		OSL_DELAY(100);
		wait_count++;
	}

	/* Save results */
	iq_est->iq_prod = ((uint32)read_phy_reg(pi, SSLPNPHY_IQAccHiAddress) << 16) |
		(uint32)read_phy_reg(pi, SSLPNPHY_IQAccLoAddress);
	iq_est->i_pwr = ((uint32)read_phy_reg(pi, SSLPNPHY_IQIPWRAccHiAddress) << 16) |
		(uint32)read_phy_reg(pi, SSLPNPHY_IQIPWRAccLoAddress);
	iq_est->q_pwr = ((uint32)read_phy_reg(pi, SSLPNPHY_IQQPWRAccHiAddress) << 16) |
		(uint32)read_phy_reg(pi, SSLPNPHY_IQQPWRAccLoAddress);
	WL_NONE(("wl%d: %s: IQ estimation completed in %d us,"
		"i_pwr: %d, q_pwr: %d, iq_prod: %d\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
		wait_count * 100, iq_est->i_pwr, iq_est->q_pwr, iq_est->iq_prod));

cleanup:
	mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
		SSLPNPHY_crsgainCtrl_APHYGatingEnable_MASK,
		1 << SSLPNPHY_crsgainCtrl_APHYGatingEnable_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_MASK,
		0 << SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_SHIFT);
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
	if (phybw40 == 1) {
		mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
			SSLPNPHY_Rev2_crsgainCtrl_40_APHYGatingEnable_MASK,
			1 << SSLPNPHY_Rev2_crsgainCtrl_40_APHYGatingEnable_SHIFT);
	}
	}
	return result;
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_get_rx_iq_comp)(phy_info_t *pi, uint16 *a0, uint16 *b0)
{
	*a0 = ((read_phy_reg(pi, SSLPNPHY_RxCompcoeffa0) & SSLPNPHY_RxCompcoeffa0_a0_MASK) >>
		SSLPNPHY_RxCompcoeffa0_a0_SHIFT);
	*b0 = ((read_phy_reg(pi, SSLPNPHY_RxCompcoeffb0) & SSLPNPHY_RxCompcoeffb0_b0_MASK) >>
		SSLPNPHY_RxCompcoeffb0_b0_SHIFT);
}

static void
wlc_sslpnphy_set_rx_iq_comp(phy_info_t *pi, uint16 a0, uint16 b0)
{
	/* Apply new coeffs */
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffa0,
		SSLPNPHY_RxCompcoeffa0_a0_MASK,
		a0 << SSLPNPHY_RxCompcoeffa0_a0_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffb0,
		SSLPNPHY_RxCompcoeffb0_b0_MASK,
		b0 << SSLPNPHY_RxCompcoeffb0_b0_SHIFT);

	/* Fill ANT1 and MRC coeffs as well */
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffa1,
		SSLPNPHY_RxCompcoeffa1_a1_MASK,
		a0 << SSLPNPHY_RxCompcoeffa1_a1_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffb1,
		SSLPNPHY_RxCompcoeffb1_b1_MASK,
		b0 << SSLPNPHY_RxCompcoeffb1_b1_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffa2,
		SSLPNPHY_RxCompcoeffa2_a2_MASK,
		a0 << SSLPNPHY_RxCompcoeffa2_a2_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RxCompcoeffb2,
		SSLPNPHY_RxCompcoeffb2_b2_MASK,
		b0 << SSLPNPHY_RxCompcoeffb2_b2_SHIFT);
}

/*
* Compute Rx compensation coeffs
*   -- run IQ est and calculate compensation coefficients
*/
STATIC bool
BCMROMOVERLAYFN(1, wlc_sslpnphy_calc_rx_iq_comp)(phy_info_t *pi,  uint16 num_samps)
{
#define SSLPNPHY_MAX_RXIQ_PWR 30000000
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_iq_est_t iq_est;
	bool result;
	uint16 a0_new, b0_new;
	int32  a, b, temp;
	int16  iq_nbits, qq_nbits, arsh, brsh;
	int32  iq = 0;
	uint32 ii, qq;
	uint8  band_idx;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	bzero(&iq_est, sizeof(iq_est));

	/* Get original a0 & b0 */
	wlc_sslpnphy_get_rx_iq_comp(pi, &a0_new, &b0_new);

	mod_phy_reg(pi, SSLPNPHY_rxfe,
		SSLPNPHY_rxfe_bypass_iqcomp_MASK,
		0 << SSLPNPHY_rxfe_bypass_iqcomp_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_RxIqCoeffCtrl,
		SSLPNPHY_RxIqCoeffCtrl_RxIqComp11bEn_MASK,
		1 << SSLPNPHY_RxIqCoeffCtrl_RxIqComp11bEn_SHIFT);

	/* Zero out comp coeffs and do "one-shot" calibration */
	wlc_sslpnphy_set_rx_iq_comp(pi, 0, 0);

	if (!(result = wlc_sslpnphy_rx_iq_est(pi, num_samps, 32, &iq_est)))
		goto cleanup;

	iq = (int32)iq_est.iq_prod;
	ii = iq_est.i_pwr;
	qq = iq_est.q_pwr;

	/* bounds check estimate info */
	if ((ii + qq) > SSLPNPHY_MAX_RXIQ_PWR) {
		WL_ERROR(("wl%d: %s: RX IQ imbalance estimate power too high\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
		result = FALSE;
		goto cleanup;
	}

	/* Calculate new coeffs */
	iq_nbits = wlc_phy_nbits(iq);
	qq_nbits = wlc_phy_nbits(qq);

	arsh = 10-(30-iq_nbits);
	if (arsh >= 0) {
		a = (-(iq << (30 - iq_nbits)) + (ii >> (1 + arsh)));
		temp = (int32) (ii >>  arsh);
		if (temp == 0) {
			WL_ERROR(("Aborting Rx IQCAL! ii=%d, arsh=%d\n", ii, arsh));
			return FALSE;
		}
	} else {
		a = (-(iq << (30 - iq_nbits)) + (ii << (-1 - arsh)));
		temp = (int32) (ii << -arsh);
		if (temp == 0) {
			WL_ERROR(("Aborting Rx IQCAL! ii=%d, arsh=%d\n", ii, arsh));
			return FALSE;
		}
	}
	a /= temp;

	brsh = qq_nbits-31+20;
	if (brsh >= 0) {
		b = (qq << (31-qq_nbits));
		temp = (int32) (ii >>  brsh);
		if (temp == 0) {
			WL_ERROR(("Aborting Rx IQCAL! ii=%d, brsh=%d\n", ii, brsh));
			return FALSE;
		}
	} else {
		b = (qq << (31-qq_nbits));
		temp = (int32) (ii << -brsh);
		if (temp == 0) {
			WL_ERROR(("Aborting Rx IQCAL! ii=%d, brsh=%d\n", ii, brsh));
			return FALSE;
		}
	}
	b /= temp;
	b -= a*a;
	b = (int32)wlc_phy_sqrt_int((uint32) b);
	b -= (1 << 10);

	a0_new = (uint16)(a & 0x3ff);
	b0_new = (uint16)(b & 0x3ff);
	/* Save calibration results */
	sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiqcal_coeffa0 = a0_new;
	sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiqcal_coeffb0 = b0_new;
	sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiq_enable = read_phy_reg(pi, SSLPNPHY_RxIqCoeffCtrl);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].rxfe = (uint8)read_phy_reg(pi, SSLPNPHY_rxfe);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].loopback1 = (uint8)read_radio_reg(pi,
		RADIO_2063_TXRX_LOOPBACK_1);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].loopback2 = (uint8)read_radio_reg(pi,
		RADIO_2063_TXRX_LOOPBACK_2);

cleanup:
	/* Apply new coeffs */
	wlc_sslpnphy_set_rx_iq_comp(pi, a0_new, b0_new);

	return result;
}

STATIC void
wlc_sslpnphy_stop_ddfs(phy_info_t *pi)
{
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_playoutEn_MASK,
		0 << SSLPNPHY_afe_ddfs_playoutEn_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_lpphyCtrl,
		SSLPNPHY_lpphyCtrl_afe_ddfs_en_MASK,
		0 << SSLPNPHY_lpphyCtrl_afe_ddfs_en_SHIFT);

	/* switch ddfs clock off */
	and_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, 0xffef);
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_run_ddfs)(phy_info_t *pi, int i_on, int q_on,
	int incr1, int incr2, int scale_index)
{
	wlc_sslpnphy_stop_ddfs(pi);

	mod_phy_reg(pi, SSLPNPHY_afe_ddfs_pointer_init,
		SSLPNPHY_afe_ddfs_pointer_init_lutPointer1Init_MASK,
		0 << SSLPNPHY_afe_ddfs_pointer_init_lutPointer1Init_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs_pointer_init,
		SSLPNPHY_afe_ddfs_pointer_init_lutPointer2Init_MASK,
		0 << SSLPNPHY_afe_ddfs_pointer_init_lutPointer2Init_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_afe_ddfs_incr_init,
		SSLPNPHY_afe_ddfs_incr_init_lutIncr1Init_MASK,
		incr1 << SSLPNPHY_afe_ddfs_incr_init_lutIncr1Init_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs_incr_init,
		SSLPNPHY_afe_ddfs_incr_init_lutIncr2Init_MASK,
		incr2 << SSLPNPHY_afe_ddfs_incr_init_lutIncr2Init_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_chanIEn_MASK,
		i_on << SSLPNPHY_afe_ddfs_chanIEn_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_chanQEn_MASK,
		q_on << SSLPNPHY_afe_ddfs_chanQEn_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_scaleIndex_MASK,
		scale_index << SSLPNPHY_afe_ddfs_scaleIndex_SHIFT);

	/* Single tone */
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_twoToneEn_MASK,
		0x0 << SSLPNPHY_afe_ddfs_twoToneEn_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_afe_ddfs,
		SSLPNPHY_afe_ddfs_playoutEn_MASK,
		0x1 << SSLPNPHY_afe_ddfs_playoutEn_SHIFT);

	/* switch ddfs clock on */
	or_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, 0x10);

	mod_phy_reg(pi, SSLPNPHY_lpphyCtrl,
		SSLPNPHY_lpphyCtrl_afe_ddfs_en_MASK,
		1 << SSLPNPHY_lpphyCtrl_afe_ddfs_en_SHIFT);
}


/*
* RX IQ Calibration
*/
bool
BCMOVERLAYFN(1, wlc_sslpnphy_rx_iq_cal)(phy_info_t *pi, const sslpnphy_rx_iqcomp_t *iqcomp, int iqcomp_sz,
	bool use_noise, bool tx_switch, bool rx_switch, bool pa, int tx_gain_idx)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_txgains_t old_gains;
	uint16 tx_pwr_ctrl;
	uint8 tx_gain_index_old = 0;
	uint ddfs_scale;
	bool result = FALSE, tx_gain_override_old = FALSE;
#ifdef BAND5G
	uint16 papd_ctrl_old = 0;
	uint16 Core1TxControl_old = 0;
	uint16 sslpnCalibClkEnCtrl_old = 0;
#define	MAX_IQ_PWR_LMT		536870912
#define	RX_PWR_THRSH_MAX	30000000
#define	RX_PWR_THRSH_MIN	4200000
#endif

	if (iqcomp) {
		ASSERT(iqcomp_sz);

		while (iqcomp_sz--) {
			if (iqcomp[iqcomp_sz].chan == CHSPEC_CHANNEL(pi->radio_chanspec)) {
				/* Apply new coeffs */
				wlc_sslpnphy_set_rx_iq_comp(pi,
					(uint16)iqcomp[iqcomp_sz].a, (uint16)iqcomp[iqcomp_sz].b);
				result = TRUE;
				break;
			}
		}
		ASSERT(result);
		goto cal_done;
	}
	/* PA driver override PA Over ride */
	mod_phy_reg(pi, SSLPNPHY_rfoverride3,
		SSLPNPHY_rfoverride3_stxpadpu2g_ovr_MASK |
		SSLPNPHY_rfoverride3_stxpapu_ovr_MASK,
		((1 << SSLPNPHY_rfoverride3_stxpadpu2g_ovr_SHIFT) |
		(1 << SSLPNPHY_rfoverride3_stxpapu_ovr_SHIFT)));
	mod_phy_reg(pi, SSLPNPHY_rfoverride3_val,
		SSLPNPHY_rfoverride3_val_stxpadpu2g_ovr_val_MASK |
		SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_MASK,
		((0 << SSLPNPHY_rfoverride3_val_stxpadpu2g_ovr_val_SHIFT) |
		(0 << SSLPNPHY_rfoverride3_val_stxpapu_ovr_val_SHIFT)));

	if (use_noise) {
		tx_switch = TRUE;
		rx_switch = FALSE;
		pa = FALSE;
	}

	/* Set TR switch */
	wlc_sslpnphy_set_trsw_override(pi, tx_switch, rx_switch);

	/* turn on PA */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
			SSLPNPHY_rfoverride2val_slna_pu_ovr_val_MASK,
			0 << SSLPNPHY_rfoverride2val_slna_pu_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_slna_pu_ovr_MASK,
			1 << SSLPNPHY_rfoverride2_slna_pu_ovr_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_rxlnaandgainctrl1ovrval,
			SSLPNPHY_rxlnaandgainctrl1ovrval_lnapuovr_Val_MASK,
			0x20 << SSLPNPHY_rxlnaandgainctrl1ovrval_lnapuovr_Val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_lna_pu_ovr_MASK,
			1 << SSLPNPHY_rfoverride2_lna_pu_ovr_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_RFinputOverrideVal,
			SSLPNPHY_RFinputOverrideVal_wlslnapu_ovr_val_MASK,
			0 << SSLPNPHY_RFinputOverrideVal_wlslnapu_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverride,
			SSLPNPHY_RFinputOverride_wlslnapu_ovr_MASK,
			1 << SSLPNPHY_RFinputOverride_wlslnapu_ovr_SHIFT);

		write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_1, 0x8c);
		write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_2, 0);
#ifndef BAND5G
	}
#else
	} else {
		/* In A-band As Play_Tone being used,Tx-Pu Override Regs inside */
			/* that proc used To turn on Tx RF Chain. */

		mod_phy_reg(pi, SSLPNPHY_RFOverride0,
			SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
			1 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
			SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_MASK,
			0  << SSLPNPHY_RFOverrideVal0_amode_tx_pu_ovr_val_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_rfoverride2val,
			SSLPNPHY_rfoverride2val_slna_pu_ovr_val_MASK,
			0  << SSLPNPHY_rfoverride2val_slna_pu_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_slna_pu_ovr_MASK,
			1 << SSLPNPHY_rfoverride2_slna_pu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rxlnaandgainctrl1ovrval,
			SSLPNPHY_rxlnaandgainctrl1ovrval_lnapuovr_Val_MASK,
			0x04 << SSLPNPHY_rxlnaandgainctrl1ovrval_lnapuovr_Val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_rfoverride2,
			SSLPNPHY_rfoverride2_lna_pu_ovr_MASK,
			1 << SSLPNPHY_rfoverride2_lna_pu_ovr_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverrideVal,
			SSLPNPHY_RFinputOverrideVal_wlslnapu_ovr_val_MASK,
			0 << SSLPNPHY_RFinputOverrideVal_wlslnapu_ovr_val_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_RFinputOverride,
			SSLPNPHY_RFinputOverride_wlslnapu_ovr_MASK,
			1 << SSLPNPHY_RFinputOverride_wlslnapu_ovr_SHIFT);
		write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_1, 0);
		write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_2, 0x8c);
	}
#endif /* BAND5G */
	/* Save tx power control mode */
	tx_pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
			papd_ctrl_old = read_phy_reg(pi, SSLPNPHY_papd_control);
			mod_phy_reg(pi, SSLPNPHY_papd_control,
				SSLPNPHY_papd_control_papdCompEn_MASK,
				0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
			sslpnCalibClkEnCtrl_old = read_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl);
			Core1TxControl_old = read_phy_reg(pi, SSLPNPHY_Core1TxControl);
			mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
				SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_MASK |
				SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_MASK |
				SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_MASK,
				((0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_SHIFT) |
				(0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_SHIFT) |
				(0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_SHIFT)));
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txcomplexfilten_MASK |
				SSLPNPHY_Core1TxControl_txrealfilten_MASK,
				((0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT) |
				(0 << SSLPNPHY_Core1TxControl_txrealfilten_SHIFT)));
		}
	}
#endif /* BAND5G */
	if (use_noise) {
		 wlc_sslpnphy_set_rx_gain(pi, 0x2d5d);
	} else {

		/* crk: papd ? */

		/* Save old tx gain settings */
		tx_gain_override_old = wlc_sslpnphy_tx_gain_override_enabled(pi);
		if (tx_gain_override_old) {
			wlc_sslpnphy_get_tx_gain(pi, &old_gains);
			tx_gain_index_old = sslpnphy_specific->sslpnphy_current_index;
		}
		/* Apply new tx gain */
		wlc_sslpnphy_set_tx_pwr_by_index(pi, tx_gain_idx);
		wlc_sslpnphy_set_rx_gain_by_distribution(pi, 0, 0, 0, 0, 6, 3, 0);
		wlc_sslpnphy_rx_gain_override_enable(pi, TRUE);
	}

	/* Force ADC on */
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_MASK,
		1 << SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
		SSLPNPHY_AfeCtrlOvrVal_pwdn_adc_ovr_val_MASK,
		0 << SSLPNPHY_AfeCtrlOvrVal_pwdn_adc_ovr_val_SHIFT);

	/* Force Rx on	 */
	mod_phy_reg(pi, SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK,
		1 << SSLPNPHY_RFOverride0_internalrfrxpu_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RFOverrideVal0,
		SSLPNPHY_RFOverrideVal0_internalrfrxpu_ovr_val_MASK,
		1 << SSLPNPHY_RFOverrideVal0_internalrfrxpu_ovr_val_SHIFT);
	if (read_radio_reg(pi, RADIO_2063_TXBB_CTRL_1) == 0x10)
		ddfs_scale = 2;
	else
		ddfs_scale = 0;

	/* Run calibration */
	if (use_noise) {
		wlc_sslpnphy_set_deaf(pi);
		result = wlc_sslpnphy_calc_rx_iq_comp(pi, 0xfff0);
		wlc_sslpnphy_clear_deaf(pi);
	} else {
		int tx_idx = 80;
		uint8 tia_gain = 8, lna2_gain = 3, vga_gain = 0;

		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			wlc_sslpnphy_run_ddfs(pi, 1, 1, 5, 5, ddfs_scale);
			if (sslpnphy_specific->sslpnphy_recal)
				tia_gain = sslpnphy_specific->sslpnphy_last_cal_tia_gain;

			while (tia_gain > 0) {
			wlc_sslpnphy_set_rx_gain_by_distribution(pi, 0, 0, 0, 0, tia_gain-1, 3, 0);
				result = wlc_sslpnphy_calc_rx_iq_comp(pi, 0xffff);
				if (result)
					break;
				tia_gain--;
			}
			wlc_sslpnphy_stop_ddfs(pi);
		} else {
#ifdef BAND5G
			int tx_idx_init, tx_idx_low_lmt;
			uint32 pwr;
			sslpnphy_iq_est_t iq_est;

			bzero(&iq_est, sizeof(iq_est));

			wlc_sslpnphy_start_tx_tone(pi, 4000, 100, 1);

			if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)
				tx_idx_init = 23;
			else
				tx_idx_init = 60;

			tx_idx = tx_idx_init;
			tx_idx_low_lmt = tx_idx_init - 28;
			if (sslpnphy_specific->sslpnphy_recal) {
				tx_idx = sslpnphy_specific->sslpnphy_last_cal_tx_idx;
				tia_gain = sslpnphy_specific->sslpnphy_last_cal_tia_gain;
				lna2_gain = sslpnphy_specific->sslpnphy_last_cal_lna2_gain;
				vga_gain = sslpnphy_specific->sslpnphy_last_cal_vga_gain;
			}
			while ((tx_idx > tx_idx_low_lmt) && ((tx_idx - 8) > 0)) {
				tx_idx -= 8;
				wlc_sslpnphy_set_tx_pwr_by_index(pi, tx_idx);
				wlc_sslpnphy_disable_pad(pi);

				wlc_sslpnphy_set_rx_gain_by_distribution(pi, 0, 0, 0, 0, 7, 3, 0);

				if (!(wlc_sslpnphy_rx_iq_est(pi, 0xffff, 32, &iq_est)))
					break;
				pwr = iq_est.i_pwr + iq_est.q_pwr;

				if (pwr > MAX_IQ_PWR_LMT) {
					tx_idx += 40;
					if (tx_idx > 127) {
						tx_idx = 127;
						wlc_sslpnphy_set_tx_pwr_by_index(pi, tx_idx);
						wlc_sslpnphy_disable_pad(pi);

						break;
					}
				} else if (pwr > RX_PWR_THRSH_MIN) {
					break;
				}
			}
			while (((tia_gain > 0) || (lna2_gain > 1)) && (vga_gain < 10)) {
				if (!vga_gain) {
					if (tia_gain != 0)
						tia_gain--;
					else if (tia_gain == 0)
						lna2_gain--;
				}

				wlc_sslpnphy_set_rx_gain_by_distribution(pi, vga_gain, 0, 0, 0,
					tia_gain, lna2_gain, 0);

				if (!(wlc_sslpnphy_rx_iq_est(pi, 0xffff, 32, &iq_est)))
					break;
				pwr = iq_est.i_pwr + iq_est.q_pwr;

				if (pwr < RX_PWR_THRSH_MIN)
					vga_gain++;
				else if (pwr < RX_PWR_THRSH_MAX)
					break;
			}
			wlc_sslpnphy_calc_rx_iq_comp(pi, 0xffff);
			wlc_sslpnphy_stop_tx_tone(pi);
#endif /* BAND5G */
		}
		sslpnphy_specific->sslpnphy_last_cal_tx_idx = (int8) tx_idx;
		sslpnphy_specific->sslpnphy_last_cal_tia_gain = tia_gain;
		sslpnphy_specific->sslpnphy_last_cal_lna2_gain = lna2_gain;
		sslpnphy_specific->sslpnphy_last_cal_vga_gain = vga_gain;
	}

	/* Resore TR switch */
	wlc_sslpnphy_clear_trsw_override(pi);

	/* Restore PA */
	mod_phy_reg(pi, SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_gmode_tx_pu_ovr_MASK |
		SSLPNPHY_RFOverride0_amode_tx_pu_ovr_MASK,
		((0 << SSLPNPHY_RFOverride0_gmode_tx_pu_ovr_SHIFT) |
		(0 << SSLPNPHY_RFOverride0_amode_tx_pu_ovr_SHIFT)));
	mod_phy_reg(pi, SSLPNPHY_rfoverride3,
		SSLPNPHY_rfoverride3_stxpadpu2g_ovr_MASK |
		SSLPNPHY_rfoverride3_stxpapu_ovr_MASK,
		((0 << SSLPNPHY_rfoverride3_stxpadpu2g_ovr_SHIFT) |
		(0 << SSLPNPHY_rfoverride3_stxpapu_ovr_SHIFT)));

	/* Resore Tx gain */
	if (!use_noise) {
		if (tx_gain_override_old) {
			wlc_sslpnphy_set_tx_pwr_by_index(pi, tx_gain_index_old);
		} else
			wlc_sslpnphy_disable_tx_gain_override(pi);
	}
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
			write_phy_reg(pi, SSLPNPHY_papd_control, papd_ctrl_old);
			write_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, sslpnCalibClkEnCtrl_old);
			write_phy_reg(pi, SSLPNPHY_Core1TxControl, Core1TxControl_old);
		}
	}
#endif
	/* Clear various overrides */
	wlc_sslpnphy_rx_gain_override_enable(pi, FALSE);

	mod_phy_reg(pi, SSLPNPHY_rfoverride2,
		SSLPNPHY_rfoverride2_slna_pu_ovr_MASK |
		SSLPNPHY_rfoverride2_lna_pu_ovr_MASK |
		SSLPNPHY_rfoverride2_ps_ctrl_ovr_MASK,
		((0 << SSLPNPHY_rfoverride2_slna_pu_ovr_SHIFT) |
		(0 << SSLPNPHY_rfoverride2_lna_pu_ovr_SHIFT) |
		(0 << SSLPNPHY_rfoverride2_ps_ctrl_ovr_SHIFT)));

	mod_phy_reg(pi, SSLPNPHY_RFinputOverride,
		SSLPNPHY_RFinputOverride_wlslnapu_ovr_MASK,
		0 << SSLPNPHY_RFinputOverride_wlslnapu_ovr_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_MASK,
		0 << SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_RFOverride0,
		SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK,
		0 << SSLPNPHY_RFOverride0_internalrfrxpu_ovr_SHIFT);

cal_done:
	WL_INFORM(("wl%d: %s: Rx IQ cal complete, coeffs: A0: %d, B0: %d\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__,
		(int16)((read_phy_reg(pi, SSLPNPHY_RxCompcoeffa0) & SSLPNPHY_RxCompcoeffa0_a0_MASK)
		>> SSLPNPHY_RxCompcoeffa0_a0_SHIFT),
		(int16)((read_phy_reg(pi, SSLPNPHY_RxCompcoeffb0) & SSLPNPHY_RxCompcoeffb0_b0_MASK)
		>> SSLPNPHY_RxCompcoeffb0_b0_SHIFT)));

	return result;
}

static int
wlc_sslpnphy_wait_phy_reg(phy_info_t *pi, uint16 addr,
    uint32 val, uint32 mask, int shift,
    int timeout_us)
{
	int timer_us, done;

	for (timer_us = 0, done = 0; (timer_us < timeout_us) && (!done);
		timer_us = timer_us + 1) {

	  /* wait for poll interval in units of microseconds */
	  OSL_DELAY(1);

	  /* check if the current field value is same as the required value */
	  if (val == (uint32)(((read_phy_reg(pi, addr)) & mask) >> shift)) {
	    done = 1;
	  }
	}
	return done;
}

STATIC int
wlc_sslpnphy_aux_adc_accum(phy_info_t *pi, uint32 numberOfSamples,
    uint32 waitTime, int32 *sum, int32 *prod)
{
	uint32 save_pwdn_rssi_ovr, term0, term1;
	int done;

	save_pwdn_rssi_ovr = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvr) &
		SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK;

	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
		SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK,
		1 << SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
		SSLPNPHY_AfeCtrlOvrVal_pwdn_rssi_ovr_val_MASK,
		0 << SSLPNPHY_AfeCtrlOvrVal_pwdn_rssi_ovr_val_SHIFT);

	/* clear accumulators */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
		1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
		SSLPNPHY_auxadcCtrl_auxadcreset_MASK |
		SSLPNPHY_auxadcCtrl_rssiestStart_MASK,
		((0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT) |
		(1 << SSLPNPHY_auxadcCtrl_rssiestStart_SHIFT)));

	mod_phy_reg(pi, SSLPNPHY_NumrssiSamples,
		SSLPNPHY_NumrssiSamples_numrssisamples_MASK,
		numberOfSamples << SSLPNPHY_NumrssiSamples_numrssisamples_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_rssiwaittime,
		SSLPNPHY_rssiwaittime_rssiwaittimeValue_MASK,
		100 << SSLPNPHY_rssiwaittime_rssiwaittimeValue_SHIFT);
	done = wlc_sslpnphy_wait_phy_reg(pi, SSLPNPHY_auxadcCtrl, 0,
		SSLPNPHY_auxadcCtrl_rssiestStart_MASK,
		SSLPNPHY_auxadcCtrl_rssiestStart_SHIFT,
		1000);

	if (done) {
		term0 = read_phy_reg(pi, SSLPNPHY_rssiaccValResult0);
		term0 = (term0 & SSLPNPHY_rssiaccValResult0_rssiaccResult0_MASK);
		term1 = read_phy_reg(pi, SSLPNPHY_rssiaccValResult1);
		term1 = (term1 & SSLPNPHY_rssiaccValResult1_rssiaccResult1_MASK);
		*sum = (term1 << 16) + term0;
		term0 = read_phy_reg(pi, SSLPNPHY_rssiprodValResult0);
		term0 = (term0 & SSLPNPHY_rssiprodValResult0_rssiProdResult0_MASK);
		term1 = read_phy_reg(pi, SSLPNPHY_rssiprodValResult1);
		term1 = (term1 & SSLPNPHY_rssiprodValResult1_rssiProdResult1_MASK);
		*prod = (term1 << 16) + term0;
	}
	else {
		*sum = 0;
		*prod = 0;
	}

	/* restore result */
	mod_phy_reg(pi, (uint16)SSLPNPHY_AfeCtrlOvr,
		(uint16)SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK,
		(uint16)save_pwdn_rssi_ovr);

	return done;
}

int32
BCMOVERLAYFN(1, wlc_sslpnphy_vbatsense)(phy_info_t *pi)
{
	uint32 save_rssi_settings, save_rssiformat;
	uint16 sslpnCalibClkEnCtr;
	uint32 savemux;
	int32 sum, prod, x, voltage;
	uint32 save_reg0, save_reg5;
	uint16 save_iqcal_ctrl_2;
	uint16 num_wait;
	uint16 num_rssi;

#define numsamps 40	/* 40 samples can be accumulated in 1us timeout */
#define one_by_numsamps 26214	/* 1/40 in q.20 format */
#define qone_by_numsamps 20	/* q format of one_by_numsamps */

#define c1 (int16)((0.0580833 * (1<<19)) + 0.5)	/* polynomial coefficient in q.19 format */
#define qc1 19									/* qformat of c1 */
#define c0 (int16)((3.9591333 * (1<<13)) + 0.5) 	/* polynomial coefficient in q.14 format */
#define qc0 13									/* qformat of c0 */

	num_wait = read_phy_reg(pi, SSLPNPHY_rssiwaittime);
	num_rssi = read_phy_reg(pi, SSLPNPHY_NumrssiSamples);

	save_reg0 = si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 0, 0, 0);
	save_reg5 = si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 5, 0, 0);
	si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 0, 1, 1);
	si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 5, (1 << 31), (1 << 31));

	sslpnCalibClkEnCtr = read_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl);
	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
	            SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_MASK,
	            1 << SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_SHIFT);

	save_rssi_settings = read_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1);
	save_rssiformat = read_phy_reg(pi, SSLPNPHY_auxadcCtrl) &
	        SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK;

	/* set the "rssiformatConvEn" field in the auxadcCtrl to 1 */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK,
	            1<<SSLPNPHY_auxadcCtrl_rssiformatConvEn_SHIFT);

	/* slpinv_rssi */
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (1<<13), (0<<13));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0xf<<0), (13<<0));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0xf<<4), (8<<4));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0x7<<10), (4<<10));

	/* set powerdetector before PA and rssi mux to tempsense */
	savemux = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal) &
	        SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK;
	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
	            SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
	            4<<SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);

	/* set iqcal mux to select VBAT */
	save_iqcal_ctrl_2 = read_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2);
	mod_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, (0xF<<0), (0x4<<0));

	/* reset auxadc */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
	            SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	            1<<SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
	            SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	            0<<SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	wlc_sslpnphy_aux_adc_accum(pi, numsamps, 0, &sum, &prod);

	/* restore rssi settings */
	write_phy_reg(pi, (uint16)SSLPNPHY_AfeRSSICtrl1, (uint16)save_rssi_settings);
	mod_phy_reg(pi, (uint16)SSLPNPHY_auxadcCtrl,
	            (uint16)SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK,
	            (uint16)save_rssiformat);
	mod_phy_reg(pi, (uint16)SSLPNPHY_AfeCtrlOvrVal,
	            (uint16)SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
	            (uint16)savemux);

	write_radio_reg(pi, RADIO_2063_IQCAL_CTRL_2, save_iqcal_ctrl_2);
	write_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, sslpnCalibClkEnCtr);
	/* sum = sum/numsamps in qsum=0+qone_by_numsamps format
	 *as the accumulated values are always less than 200, 6 bit values, the
	 *sum always fits into 16 bits
	 */
	x = qm_mul321616((int16)sum, one_by_numsamps);

	/* compute voltagte = c1*sum + co */
	voltage = qm_mul323216(x, c1); /* volatage in q.qone_by_numsamps+qc1-16 format */

	/* bring sum to qc0 format */
	voltage = voltage >> (qone_by_numsamps+qc1-16 - qc0);

	/* comute c1*x + c0 */
	voltage = voltage + c0;

	/* bring voltage to q.4 format */
	voltage = voltage >> (qc0 - 4);

	si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 0, ~0, save_reg0);
	si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 5, ~0, save_reg5);

	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	write_phy_reg(pi, SSLPNPHY_rssiwaittime, num_wait);
	write_phy_reg(pi, SSLPNPHY_NumrssiSamples, num_rssi);

	return voltage;

#undef numsamps
#undef one_by_numsamps
#undef qone_by_numsamps
#undef c1
#undef qc1
#undef c0
#undef qc0
}

int
wlc_sslpnphy_tempsense(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	  uint32 save_rssi_settings, save_rssiformat;
	  uint16  sslpnCalibClkEnCtr;
	  uint32 rcalvalue, savemux;
	  int32 sum0, prod0, sum1, prod1, sum;
	  int32 temp32 = 0;
	  bool suspend;
	  uint16 num_rssi;
	  uint16 num_wait;

#define m 461.5465

	  /* b = -12.0992 in q11 format */
#define b ((int16)(-12.0992*(1<<11)))
#define qb 11

	  /* thousand_by_m = 1000/m = 1000/461.5465 in q13 format */
#define thousand_by_m ((int16)((1000/m)*(1<<13)))
#define qthousand_by_m 13

#define numsamps 400	/* 40 samples can be accumulated in 1us timeout */
#define one_by_numsamps 2621	/* 1/40 in q.20 format */
#define qone_by_numsamps 20	/* q format of one_by_numsamps */

	num_wait = read_phy_reg(pi, SSLPNPHY_rssiwaittime);
	num_rssi = read_phy_reg(pi, SSLPNPHY_NumrssiSamples);
	/* suspend the mac if it is not already suspended */
	suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	if (!suspend)
		WL_SUSPEND_MAC_AND_WAIT(pi);

	sslpnCalibClkEnCtr = read_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl);
	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_txFrontEndCalibClkEn_SHIFT);

	save_rssi_settings = read_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1);
	save_rssiformat = read_phy_reg(pi, SSLPNPHY_auxadcCtrl) &
		SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK;

	/* set the "rssiformatConvEn" field in the auxadcCtrl to 1 */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
	      SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK,
	      1 << SSLPNPHY_auxadcCtrl_rssiformatConvEn_SHIFT);

	/* slpinv_rssi */
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (1<<13), (0<<13));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0xf<<0), (0<<0));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0xf<<4), (11<<4));
	mod_phy_reg(pi, SSLPNPHY_AfeRSSICtrl1, (0x7<<10), (5<<10));

	/* read the rcal value */
	rcalvalue = read_radio_reg(pi, RADIO_2063_COMMON_13) & 0xf;

	/* set powerdetector before PA and rssi mux to tempsense */
	savemux = read_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal) &
	    SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK;

	mod_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
	      SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
	      5 << SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_SHIFT);

	/* reset auxadc */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
	     SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	     1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl,
	     SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	     0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	/* set rcal override */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<7), (1<<7));

	/* power up temp sense */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<6), (0<<6));

	/* set temp sense mux */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<5), (0<<5));

	/* set rcal value */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (0xf<<0), (rcalvalue<<0));

	/* set TPSENSE_swap to 0 */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<4), (0<<4));

	wlc_sslpnphy_aux_adc_accum(pi, numsamps, 0, &sum0, &prod0);

	/* reset auxadc */
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	/* set TPSENSE swap to 1 */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<4), (1<<4));

	wlc_sslpnphy_aux_adc_accum(pi, numsamps, 0, &sum1, &prod1);

	sum = (sum0 + sum1) >> 1;

	/* restore rssi settings */
	write_phy_reg(pi, (uint16)SSLPNPHY_AfeRSSICtrl1, (uint16)save_rssi_settings);
	mod_phy_reg(pi, (uint16)SSLPNPHY_auxadcCtrl,
	      (uint16)SSLPNPHY_auxadcCtrl_rssiformatConvEn_MASK,
	      (uint16)save_rssiformat);
	mod_phy_reg(pi, (uint16)SSLPNPHY_AfeCtrlOvrVal,
	      (uint16)SSLPNPHY_AfeCtrlOvrVal_rssi_muxsel_ovr_val_MASK,
	      (uint16)savemux);

	/* powerdown tempsense */
	mod_radio_reg(pi, RADIO_2063_TEMPSENSE_CTRL_1, (1<<6), (1<<6));

	/* sum = sum/numsamps in qsum=0+qone_by_numsamps format
	 *as the accumulated values are always less than 200, 6 bit values, the
	 *sum always fits into 16 bits
	 */

	sum = qm_mul321616((int16)sum, one_by_numsamps);

	/* bring sum into qb format */
	sum = sum >> (qone_by_numsamps-qb);

	/* sum-b in qb format */
	temp32 = sum - b;

	/* calculate (sum-b)*1000/m in qb+qthousand_by_m-15=11+13-16 format */
	temp32 = qm_mul323216(temp32, (int16)thousand_by_m);

	/* bring temp32 into q0 format */
	temp32 = (temp32+(1<<7)) >> 8;

	write_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl, sslpnCalibClkEnCtr);

	/* enable the mac if it is suspended by tempsense function */
	if (!suspend)
		WL_ENABLE_MAC(pi);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      1 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);
	mod_phy_reg(pi, SSLPNPHY_auxadcCtrl, SSLPNPHY_auxadcCtrl_auxadcreset_MASK,
	      0 << SSLPNPHY_auxadcCtrl_auxadcreset_SHIFT);

	write_phy_reg(pi, SSLPNPHY_rssiwaittime, num_wait);
	write_phy_reg(pi, SSLPNPHY_NumrssiSamples, num_rssi);

	sslpnphy_specific->sslpnphy_lastsensed_temperature = (int8)temp32;
	return temp32;
#undef m
#undef b
#undef qb
#undef thousand_by_m
#undef qthousand_by_m
#undef numsamps
#undef one_by_numsamps
#undef qone_by_numsamps
}
static void
wlc_sslpnphy_temp_adj_offset(phy_info_t *pi, int8 temp_adj)
{
	uint32 tableBuffer[2];
	uint8 phybw40 = IS40MHZ(pi);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	/* adjust the reference ofdm gain index table offset */
	mod_phy_reg(pi, SSLPNPHY_gainidxoffset,
		SSLPNPHY_gainidxoffset_ofdmgainidxtableoffset_MASK,
		((sslpnphy_specific->sslpnphy_ofdmgainidxtableoffset +  temp_adj) <<
		 SSLPNPHY_gainidxoffset_ofdmgainidxtableoffset_SHIFT));

	/* adjust the reference dsss gain index table offset */
	mod_phy_reg(pi, SSLPNPHY_gainidxoffset,
		SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_MASK,
		((sslpnphy_specific->sslpnphy_dsssgainidxtableoffset +  temp_adj) <<
		 SSLPNPHY_gainidxoffset_dsssgainidxtableoffset_SHIFT));

	/* adjust the reference gain_val_tbl at index 64 and 65 in gain_val_tbl */
	tableBuffer[0] = sslpnphy_specific->sslpnphy_tr_R_gain_val + temp_adj;
	tableBuffer[1] = sslpnphy_specific->sslpnphy_tr_T_gain_val + temp_adj;

	wlc_sslpnphy_common_write_table(pi, 17, tableBuffer, 2, 32, 64);

	if (phybw40 == 0) {
		mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
			SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
			((sslpnphy_specific->sslpnphy_input_pwr_offset_db + temp_adj) <<
			SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
	        mod_phy_reg(pi, SSLPNPHY_LowGainDB,
	                SSLPNPHY_LowGainDB_MedLowGainDB_MASK,
	                ((sslpnphy_specific->sslpnphy_Med_Low_Gain_db + temp_adj) <<
	                SSLPNPHY_LowGainDB_MedLowGainDB_SHIFT));
	        mod_phy_reg(pi, SSLPNPHY_VeryLowGainDB,
	                SSLPNPHY_VeryLowGainDB_veryLowGainDB_MASK,
	                ((sslpnphy_specific->sslpnphy_Very_Low_Gain_db + temp_adj) <<
	                SSLPNPHY_VeryLowGainDB_veryLowGainDB_SHIFT));
	} else {
		mod_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40,
			SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK,
			((sslpnphy_specific->sslpnphy_input_pwr_offset_db + temp_adj) <<
			SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_SHIFT));
	        mod_phy_reg(pi, SSLPNPHY_Rev2_LowGainDB_40,
	                SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_MASK,
	                ((sslpnphy_specific->sslpnphy_Med_Low_Gain_db + temp_adj) <<
	                SSLPNPHY_Rev2_LowGainDB_40_MedLowGainDB_SHIFT));
	        mod_phy_reg(pi, SSLPNPHY_Rev2_VeryLowGainDB_40,
	                SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_MASK,
	                ((sslpnphy_specific->sslpnphy_Very_Low_Gain_db + temp_adj) <<
	                SSLPNPHY_Rev2_VeryLowGainDB_40_veryLowGainDB_SHIFT));
	}
}
STATIC uint8 chan_spec_spur_85degc_38p4Mhz [][3] = {
	{0, 1, 2},
	{0, 1, 22},
	{0, 1, 6},
	{0, 1, 54},
	{0, 1, 26},
	{0, 1, 38}
};
STATIC uint16 UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS [][2] = {
	{25,  0x3AC},
	{26,  0x3C3}
};
uint8 UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS_sz = ARRAYSIZE(UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS);
uint8 chan_spec_spur_85degc_38p4Mhz_sz = ARRAYSIZE(chan_spec_spur_85degc_38p4Mhz);
static void
wlc_sslpnphy_chanspec_spur_weight(phy_info_t *pi, uint channel, uint8 ptr[][3], uint8 array_size);
void
wlc_sslpnphy_temp_adj(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	int32 temperature;
	uint32 tableBuffer[2];
	uint freq;
	int16 thresh1, thresh2;
	uint8 spur_weight;
	uint8 i;
	uint16 minsig = 0x01bc;
	if (CHSPEC_IS5G(pi->radio_chanspec))
		minsig = 0x0184;

	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		thresh1 = -23;
		thresh2 = 60;
	} else {
		thresh1 = -30;
		thresh2 = 55;
	}
	temperature = sslpnphy_specific->sslpnphy_lastsensed_temperature;
	freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
	if ((temperature - 15) <= thresh1) {
		wlc_sslpnphy_temp_adj_offset(pi, 6);
		sslpnphy_specific->sslpnphy_pkteng_rssi_slope = ((((temperature - 15) + 30) * 286) >> 12) - 0;
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
				wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 26);
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 28);
			}
			if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
				/* for 4329 only */
				tableBuffer[0] = 0xd1a4099c;
				tableBuffer[1] = 0xd1a40018;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 52);
				tableBuffer[0] = 0xf1e64d96;
				tableBuffer[1] = 0xf1e60018;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 54);
				if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
					if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
						(sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
						tableBuffer[0] = 0x204ca9e;
						tableBuffer[1] = 0x2040019;
						wlc_sslpnphy_common_write_table(pi,
							SSLPNPHY_TBL_ID_GAIN_IDX, tableBuffer, 2, 32, 56);
						tableBuffer[0] = 0xa246cea1;
						tableBuffer[1] = 0xa246001c;
						wlc_sslpnphy_common_write_table(pi,
							SSLPNPHY_TBL_ID_GAIN_IDX, tableBuffer, 2, 32, 132);
					}
				}
				write_phy_reg(pi, SSLPNPHY_gainBackOffVal, 0x6666);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
					SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
					31 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
					SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_MASK,
					15 << SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_SHIFT);

				if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						(1 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
					mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
						SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
						8 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
				} else {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						(0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
					write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 12);
				}
			} else {
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
					SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
					30 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
					SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
					6 << SSLPNPHY_PwrThresh0_SlowPwrLoThresh_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(4 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID)) {
					for (i = 0; i < 64; i++)
						wlc_sslpnphy_common_write_table(pi, 2, &minsig, 1, 16, i);
				}
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICUNO_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICLOCO_SSID)) {
					for (i = 25; i <= 26; i++) {
						wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
							UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS,
							UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS_sz);
					}
				}
			}
		} else {
			if (((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) &&
				(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA_5GHz)) {
				for (i = 0; i < 64; i++)
					wlc_sslpnphy_common_write_table(pi, 2, &minsig, 1, 16, i);
			}
		}
	} else if ((temperature - 15) < 4) {
		wlc_sslpnphy_temp_adj_offset(pi, 3);
		sslpnphy_specific->sslpnphy_pkteng_rssi_slope = ((((temperature - 15) - 4) * 286) >> 12) - 0;
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
		tableBuffer[0] = sslpnphy_specific->sslpnphy_gain_idx_14_lowword;
		tableBuffer[1] = sslpnphy_specific->sslpnphy_gain_idx_14_hiword;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
			tableBuffer, 2, 32, 28);
		if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
			/* Added To Increase The 1Mbps Sense for Temps @Around */
			/* -15C Temp With CmRxAciGainTbl */
			tableBuffer[0] = sslpnphy_specific->sslpnphy_gain_idx_27_lowword;
			tableBuffer[1] = sslpnphy_specific->sslpnphy_gain_idx_27_hiword;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tableBuffer, 2, 32, 54);
			if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
				if (freq <= 2427) {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						253 <<
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				} else if (freq < 2472) {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				} else {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						254 <<
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				}
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID)
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
					SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
					27 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
			} else if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
				for (i = 63; i <= 73; i++) {
					wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
						tableBuffer, 2, 32, ((i - 37) *2));
					wlc_sslpnphy_common_write_table(pi,
						SSLPNPHY_TBL_ID_GAIN_IDX,
						tableBuffer, 2, 32, (i * 2));
				}
				write_phy_reg(pi, SSLPNPHY_slnanoisetblreg2, 0x03F0);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(2 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));

			}
		} else {
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				(2 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
			mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
				SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
				27 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
			if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID)) {
				for (i = 0; i < 64; i++)
					wlc_sslpnphy_common_write_table(pi, 2, &minsig, 1, 16, i);
			}
			if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICUNO_SSID) ||
				(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICLOCO_SSID)) {
				for (i = 25; i <= 26; i++) {
					wlc_sslpnphy_rx_gain_table_tweaks(pi, i,
						UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS,
						UNO_LOW_TEMP_OFDM_GAINTBL_TWEAKS_sz);
				}
			}
		}
		} else {
			if (((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) &&
				(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA_5GHz)) {
				for (i = 0; i < 64; i++)
					wlc_sslpnphy_common_write_table(pi, 2, &minsig, 1, 16, i);
			}
		}
	} else if ((temperature - 15) < thresh2) {
		wlc_sslpnphy_temp_adj_offset(pi, 0);
		sslpnphy_specific->sslpnphy_pkteng_rssi_slope = (((temperature - 15) - 25) * 286) >> 12;
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
		tableBuffer[0] = sslpnphy_specific->sslpnphy_gain_idx_14_lowword;
		tableBuffer[1] = sslpnphy_specific->sslpnphy_gain_idx_14_hiword;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
			tableBuffer, 2, 32, 28);
		if (((temperature) >= 50) & ((temperature) < (thresh2 + 15))) {
			if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					((freq <= 2427) ? 255 : 0 <<
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
			} else {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					3 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_PwrThresh0,
					SSLPNPHY_PwrThresh0_SlowPwrLoThresh_MASK,
					7 << SSLPNPHY_Rev2_PwrThresh0_SlowPwrLoThresh_SHIFT);
			}
		}
		}
	} else {
		wlc_sslpnphy_temp_adj_offset(pi, -3);
		sslpnphy_specific->sslpnphy_pkteng_rssi_slope = ((((temperature - 10) - 55) * 286) >> 12) - 2;
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			mod_phy_reg(pi, SSLPNPHY_radioTRCtrlCrs1,
				SSLPNPHY_radioTRCtrlCrs1_trGainThresh_MASK,
				23 << SSLPNPHY_radioTRCtrlCrs1_trGainThresh_SHIFT);
		if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
			write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 12);
			tableBuffer[0] = 0xd0008206;
			tableBuffer[1] = 0xd0000009;
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
				tableBuffer, 2, 32, 28);
			 if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
				tableBuffer[0] = 0x41a4099c;
				tableBuffer[1] = 0x41a4001d;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 52);
				tableBuffer[0] = 0x51e64d96;
				tableBuffer[1] = 0x51e6001d;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_GAIN_IDX,
					tableBuffer, 2, 32, 54);
			} else {
				mod_phy_reg(pi, SSLPNPHY_PwrThresh1,
					SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_MASK,
					15 << SSLPNPHY_PwrThresh1_PktRxSignalDropThresh_SHIFT);
			}
			if (XTALFREQ(pi->xtalfreq) == 38400000) {
				/* Special Tuning As The 38.4Mhz Xtal Boards */
				/* SpurProfile Changes Drastically At Very High */
				/* Temp(Especially @85degC) */
				wlc_sslpnphy_chanspec_spur_weight(pi, 0,
					chan_spec_spur_85degc_38p4Mhz,
					chan_spec_spur_85degc_38p4Mhz_sz);
				if (freq == 2452 || freq == 2462) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(253 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				mod_phy_reg(pi, SSLPNPHY_dsssPwrThresh0,
					SSLPNPHY_dsssPwrThresh0_dsssPwrThresh0_MASK,
					21 << SSLPNPHY_dsssPwrThresh0_dsssPwrThresh0_SHIFT);
				spur_weight = 4;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
					&spur_weight, 1, 8, ((freq == 2452) ? 18 : 50));
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(254 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				spur_weight = 2;
				wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
					&spur_weight, 1, 8, ((freq == 2452) ? 18 : 50));
				}
				} else if (freq == 2467) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(253 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(254 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				}
				} else {
				if (freq >= 2472)
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(253 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				else
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(254 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				}
			} else if (XTALFREQ(pi->xtalfreq) == 26000000) {
				if (freq <= 2467) {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						254 <<
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				} else if ((freq > 2467) && (freq <= 2484)) {
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						253 <<
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				}
			} else {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					253 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
			}
		} else {
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				(0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN18_SSID)
				write_phy_reg(pi, SSLPNPHY_ClipCtrDefThresh, 12);
		}
		}
	}
	wlc_sslpnphy_RxNvParam_Adj(pi);

	WL_INFORM(("InSide TempAdj: Temp = %d:Init_noise_cal = %d\n", temperature,
		sslpnphy_specific->sslpnphy_init_noise_cal_done));

	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);

	wlc_sslpnphy_txpower_recalc_target(pi);
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_periodic_cal_top)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	bool full_cal, suspend;
	int8 current_temperature;
	int32 current_voltage;
	int txidx_drift, volt_drift, temp_drift, temp_drift1;
	int32 cnt, volt_high_cnt, volt_low_cnt;
	int32 volt_avg;
	int32 voltage_samples[50];
	int32 volt_high_thresh, volt_low_thresh = 0;
	uint cal_done = 0;
	wl_pkteng_t pkteng;
	struct ether_addr sa;
	uint16 max_pwr_idx, min_pwr_idx;
	uint twopt3_detected = 0;
	uint8 band_idx;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));

	if (!suspend) {
		/* Set non-zero duration for CTS-to-self */
		WL_WRITE_SHM(pi, M_CTS_DURATION, 10000);
		WL_SUSPEND_MAC_AND_WAIT(pi);
	}

	if (!NON_BT_CHIP(wlc))
		wlc_sslpnphy_btcx_override_enable(pi);

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	pi->phy_lastcal = GENERIC_PHY_INFO(pi)->now;
	sslpnphy_specific->sslpnphy_restore_papd_cal_results = 0;
	sslpnphy_specific->sslpnphy_recal = 1;
	pi->phy_forcecal = FALSE;
	full_cal = (sslpnphy_specific->sslpnphy_full_cal_channel[band_idx] != CHSPEC_CHANNEL(pi->radio_chanspec));
	sslpnphy_specific->sslpnphy_full_cal_channel[band_idx] = CHSPEC_CHANNEL(pi->radio_chanspec);
	sslpnphy_specific->sslpnphy_full_cal_chanspec[band_idx] = pi->radio_chanspec;

	if (sslpnphy_specific->sslpnphy_percal_ctr == 0) {
		sslpnphy_specific->sslpnphy_tx_idx_prev_cal = (uint8)((read_phy_reg(pi, 0x4ab) & 0x7f00) >> 8);
	}

	current_temperature = (int8)wlc_sslpnphy_tempsense(pi);
	current_voltage = wlc_sslpnphy_vbatsense(pi);
	sslpnphy_specific->sslpnphy_cur_idx = (uint8)((read_phy_reg(pi, 0x4ab) & 0x7f00) >> 8);

	sslpnphy_specific->sslpnphy_percal_ctr ++;

	if ((sslpnphy_specific->sslpnphy_force_1_idxcal == 0) && (sslpnphy_specific->sslpnphy_force_percal == 0)) {
		temp_drift = current_temperature - sslpnphy_specific->sslpnphy_last_cal_temperature;
		temp_drift1 = current_temperature - sslpnphy_specific->sslpnphy_last_full_cal_temperature;
		/* Temperature change of 25 degrees or at an interval of 20 minutes do a full cal */
		if ((temp_drift1 < - 25) || (temp_drift1 > 25) ||
			(sslpnphy_specific->sslpnphy_percal_ctr == 100)) {
			wlc_2063_vco_cal(pi);
			wlc_sslpnphy_periodic_cal(pi);
			wlc_sslpnphy_tx_pwr_ctrl_init(pi);
			wlc_sslpnphy_papd_recal(pi);
			wlc_sslpnphy_temp_adj(pi);
			sslpnphy_specific->sslpnphy_percal_ctr = 0;
			sslpnphy_specific->sslpnphy_last_full_cal_temperature = current_temperature;
			cal_done = 1;
		}
		if (((temp_drift > 10) || (temp_drift < -10)) && (cal_done == 0)) {
			wlc_sslpnphy_tx_pwr_ctrl_init(pi);
			sslpnphy_specific->sslpnphy_force_1_idxcal = 1;
			sslpnphy_specific->sslpnphy_papd_nxt_cal_idx = sslpnphy_specific->sslpnphy_final_idx -
				((current_temperature - sslpnphy_specific->sslpnphy_last_cal_temperature) / 3);
			if (sslpnphy_specific->sslpnphy_papd_nxt_cal_idx >= 128)
				sslpnphy_specific->sslpnphy_papd_nxt_cal_idx = 1;
			wlc_sslpnphy_papd_recal(pi);
			wlc_sslpnphy_temp_adj(pi);
			cal_done = 1;
		}
		if (!cal_done) {
			volt_drift = current_voltage - sslpnphy_specific->sslpnphy_last_cal_voltage;
			txidx_drift = sslpnphy_specific->sslpnphy_cur_idx - sslpnphy_specific->sslpnphy_tx_idx_prev_cal;
			if ((txidx_drift > 6) || (txidx_drift < - 6)) {
				if ((volt_drift < 3) &&	(volt_drift > -3)) {
					sslpnphy_specific->sslpnphy_papd_nxt_cal_idx  = sslpnphy_specific->sslpnphy_final_idx +
					(sslpnphy_specific->sslpnphy_cur_idx - sslpnphy_specific->sslpnphy_tx_idx_prev_cal);
					if (sslpnphy_specific->sslpnphy_papd_nxt_cal_idx >= 128)
						sslpnphy_specific->sslpnphy_papd_nxt_cal_idx = 1;
					sslpnphy_specific->sslpnphy_force_1_idxcal = 1;
					wlc_sslpnphy_papd_recal(pi);
					if (txidx_drift < 0)
						sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 4;
					else
						sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 0;
					sslpnphy_specific->sslpnphy_tx_idx_prev_cal = sslpnphy_specific->sslpnphy_cur_idx;
					cal_done = 1;
				}
			}
		}
		if (!cal_done) {
			if (!IS_OLYMPIC(pi)) {
				volt_high_cnt = 0;
				volt_low_cnt = 0;
				volt_avg = 0;
				for (cnt = 0; cnt < 32; cnt++) {
					voltage_samples[cnt] = wlc_sslpnphy_vbatsense(pi);
					volt_avg += voltage_samples[cnt];
					OSL_DELAY(120);
				/* assuming a 100us time for executing wlc_sslpnphy_vbatsense */
				}
				volt_avg = volt_avg >> 5;
				volt_high_thresh = 0;
				volt_low_thresh = volt_avg;
				for (cnt = 0; cnt < 32; cnt++) {
					if (voltage_samples[cnt] > volt_high_thresh)
						volt_high_thresh = voltage_samples[cnt];
					if (voltage_samples[cnt] < volt_low_thresh)
						volt_low_thresh = voltage_samples[cnt];
				}
				sslpnphy_specific->sslpnphy_volt_winner = (uint8)(volt_high_thresh - 2);
				if ((volt_high_thresh - volt_low_thresh) > 5)
					sslpnphy_specific->sslpnphy_vbat_ripple = 1;
				else
					sslpnphy_specific->sslpnphy_vbat_ripple = 0;

				if (sslpnphy_specific->sslpnphy_vbat_ripple) {
					sa.octet[0] = 10;
					pkteng.flags = WL_PKTENG_PER_TX_START;
					pkteng.delay = 30;	/* Inter packet delay */
					pkteng.length = 0;	/* packet length */
					pkteng.seqno = FALSE;	/* enable/disable sequence no. */
					/* vbat ripple detetction */
					twopt3_detected = 0;
					/* to clear out min and max readings after 30 frames */
					WL_WRITE_SHM(pi, M_SSLPN_PWR_IDX_MAX, 0);
					WL_WRITE_SHM(pi, M_SSLPN_PWR_IDX_MIN, 127);

					/* sending out 100 frames to caluclate min & max index */
					pkteng.nframes = 100;		/* number of frames */
					wlc_sslpnphy_pktengtx((wlc_phy_t *)pi, &pkteng,
						108, &sa, (1000*10));
					max_pwr_idx = WL_READ_SHM(pi, M_SSLPN_PWR_IDX_MAX);
					min_pwr_idx = WL_READ_SHM(pi, M_SSLPN_PWR_IDX_MIN);
					/* 10 is the value chosen for a start power of 14dBm */
					if (!((max_pwr_idx == 0) && (min_pwr_idx == 127))) {
						if (((max_pwr_idx - min_pwr_idx) > 10) ||
							(min_pwr_idx == 0)) {
							twopt3_detected = 1;
						}
					}
					if (twopt3_detected) {
						sslpnphy_specific->sslpnphy_psat_2pt3_detected = 1;
						sslpnphy_specific->sslpnphy_11n_backoff = 17;
						sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 26;
						sslpnphy_specific->sslpnphy_lowerofdm = 20;
						sslpnphy_specific->sslpnphy_cck = 20;
						sslpnphy_specific->sslpnphy_last_cal_voltage = volt_low_thresh;
						sslpnphy_specific->sslpnphy_tx_idx_prev_cal = (uint8)max_pwr_idx;
					} else if (volt_low_thresh >
						sslpnphy_specific->sslpnphy_last_cal_voltage + 6) {
						sslpnphy_specific->sslpnphy_psat_2pt3_detected = 0;
						sslpnphy_specific->sslpnphy_lowerofdm = 0;
						sslpnphy_specific->sslpnphy_cck = 0;
						if (volt_low_thresh > 57) {
							sslpnphy_specific->sslpnphy_11n_backoff = 0;
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 0;
						} else {
							sslpnphy_specific->sslpnphy_11n_backoff = 8;
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
						}
						sslpnphy_specific->sslpnphy_last_cal_voltage = volt_low_thresh;
						sslpnphy_specific->sslpnphy_tx_idx_prev_cal = (uint8)max_pwr_idx;
					} else if ((volt_low_thresh <= 57) && (volt_low_thresh <
								sslpnphy_specific->sslpnphy_last_cal_voltage)) {
						sslpnphy_specific->sslpnphy_psat_2pt3_detected = 0;
						if (current_temperature >= 50) {
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
							sslpnphy_specific->sslpnphy_11n_backoff = 6;
						} else {
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
							sslpnphy_specific->sslpnphy_11n_backoff = 4;
						}
						sslpnphy_specific->sslpnphy_last_cal_voltage = volt_low_thresh;
						sslpnphy_specific->sslpnphy_tx_idx_prev_cal = (uint8)max_pwr_idx;
					}
				} else {
					sslpnphy_specific->sslpnphy_psat_2pt3_detected = 0;
					if (current_voltage > sslpnphy_specific->sslpnphy_last_cal_voltage + 6) {
						if (current_voltage > 57) {
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 0;
							sslpnphy_specific->sslpnphy_11n_backoff = 0;
						} else {
							sslpnphy_specific->sslpnphy_11n_backoff = 8;
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
							sslpnphy_specific->sslpnphy_lowerofdm = 0;
							sslpnphy_specific->sslpnphy_cck = 0;
						}
						sslpnphy_specific->sslpnphy_last_cal_voltage = current_voltage;
						sslpnphy_specific->sslpnphy_tx_idx_prev_cal = sslpnphy_specific->sslpnphy_cur_idx;
					} else if ((current_voltage <= 57) && (current_voltage <
								sslpnphy_specific->sslpnphy_last_cal_voltage)) {
						if (current_temperature >= 50) {
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
							sslpnphy_specific->sslpnphy_11n_backoff = 6;
						} else {
							sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff = 10;
							sslpnphy_specific->sslpnphy_11n_backoff = 4;
						}
						sslpnphy_specific->sslpnphy_last_cal_voltage = current_voltage;
						sslpnphy_specific->sslpnphy_tx_idx_prev_cal = sslpnphy_specific->sslpnphy_cur_idx;
					}

				}
			}
		}
	}
	if (sslpnphy_specific->sslpnphy_force_1_idxcal) {
		sslpnphy_specific->sslpnphy_last_cal_temperature = current_temperature;
		sslpnphy_specific->sslpnphy_tx_idx_prev_cal = sslpnphy_specific->sslpnphy_cur_idx;
		sslpnphy_specific->sslpnphy_last_cal_voltage = current_voltage;
		sslpnphy_specific->sslpnphy_force_1_idxcal = 0;
	}
	sslpnphy_specific->sslpnphy_recal = 0;
#ifdef PHYHAL
	wlc_phy_txpower_recalc_target(pi);
#else
	wlc_phy_txpower_recalc_target((wlc_phy_t *)pi, -1, NULL);
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_restore_papd_cal_results = 1;
	if (!suspend)
		WL_ENABLE_MAC(pi);
	if (NORADIO_ENAB(pi->pubpi))
		return;

}


void
BCMOVERLAYFN(1, wlc_sslpnphy_periodic_cal)(phy_info_t *pi)
{
	bool suspend, full_cal;
	uint16 tx_pwr_ctrl;
	uint8 band_idx;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	pi->phy_lastcal = GENERIC_PHY_INFO(pi)->now;
	pi->phy_forcecal = FALSE;
	full_cal = (sslpnphy_specific->sslpnphy_full_cal_chanspec[band_idx] != pi->radio_chanspec);
	sslpnphy_specific->sslpnphy_full_cal_channel[band_idx] = CHSPEC_CHANNEL(pi->radio_chanspec);
	sslpnphy_specific->sslpnphy_full_cal_chanspec[band_idx] = pi->radio_chanspec;

	if (NORADIO_ENAB(pi->pubpi))
		return;

	suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	if (!suspend) {
		/* Set non-zero duration for CTS-to-self */
		WL_WRITE_SHM(pi, M_CTS_DURATION, 10000);
		WL_SUSPEND_MAC_AND_WAIT(pi);
	}
	if (!NON_BT_CHIP(wlc))
		wlc_sslpnphy_btcx_override_enable(pi);


	/* Save tx power control mode */
	tx_pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

	/* Tx iqlo calibration */
	wlc_sslpnphy_txpwrtbl_iqlo_cal(pi);

	/* Restore tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);


	/* Rx iq calibration */
	{
		const sslpnphy_rx_iqcomp_t *rx_iqcomp;
		int rx_iqcomp_sz;

		rx_iqcomp = sslpnphy_rx_iqcomp_table_rev0;
		rx_iqcomp_sz = ARRAYSIZE(sslpnphy_rx_iqcomp_table_rev0);

		wlc_sslpnphy_set_deaf(pi);
		wlc_sslpnphy_rx_iq_cal(pi,
			NULL,
			0,
			FALSE, TRUE, FALSE, TRUE, 127);
		wlc_sslpnphy_clear_deaf(pi);
	}

	if (!suspend)
		WL_ENABLE_MAC(pi);
	return;
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_full_cal)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 band_idx;

	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);

	WL_TRACE(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	wlc_sslpnphy_set_deaf(pi);

	/* Force full calibration run */
	sslpnphy_specific->sslpnphy_full_cal_channel[band_idx] = 0;
	sslpnphy_specific->sslpnphy_full_cal_chanspec[band_idx] = 0;

	/* Run sslpnphy cals */
	wlc_sslpnphy_periodic_cal(pi);

	wlc_sslpnphy_clear_deaf(pi);

	/* Trigger uCode for doing AuxADC measurements */
	WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
		M_SSLPNPHY_TSSICAL_EN)), 0x0);
	wlc_sslpnphy_auxadc_measure((wlc_phy_t *) pi, 0);

	return;
}


#if defined(WLCURPOWER) || defined(PHYHAL)
void
wlc_sslpnphy_get_tssi(phy_info_t *pi, int8 *ofdm_pwr, int8 *cck_pwr)
{
	int8 cck_offset;
	uint16 status;

	if (wlc_sslpnphy_tssi_enabled(pi) &&
		((status = (read_phy_reg(pi, SSLPNPHY_TxPwrCtrlStatus))) &
		SSLPNPHY_TxPwrCtrlStatus_estPwrValid_MASK)) {
		*ofdm_pwr = (int8)((status &
			SSLPNPHY_TxPwrCtrlStatus_estPwr_MASK) >>
			SSLPNPHY_TxPwrCtrlStatus_estPwr_SHIFT);
		if (wlc_phy_tpc_isenabled_sslpnphy(pi))
			cck_offset = pi->tx_power_offset[TXP_FIRST_CCK];
		else
			cck_offset = 0;
		*cck_pwr = *ofdm_pwr + cck_offset;
	} else {
		*ofdm_pwr = 0;
		*cck_pwr = 0;
	}
}
#endif 

void
WLBANDINITFN(wlc_phy_cal_init_sslpnphy)(phy_info_t *pi)
{
}
STATIC uint8 chan_spec_spur_nokref_38p4Mhz [][3] = {
	{1, 5, 23},
	{2, 5, 7},
	{3, 5, 55},
	{4, 5, 39},
	{9, 3, 18},
	{10, 2, 2},
	{10, 3, 22},
	{11, 2, 50},
	{11, 3, 6},
	{12, 3, 54},
	{13, 3, 38},
	{13, 3, 26}
};
uint8 chan_spec_spur_nokref_38p4Mhz_sz = ARRAYSIZE(chan_spec_spur_nokref_38p4Mhz);
STATIC uint8 chan_spec_spur_tdkmdl_38p4Mhz [][3] = {
	{1, 3, 23},
	{2, 3, 7},
	{3, 3, 55},
	{4, 3, 39},
	{13, 3, 26}
};
uint8 chan_spec_spur_tdkmdl_38p4Mhz_sz = ARRAYSIZE(chan_spec_spur_tdkmdl_38p4Mhz);
STATIC uint8 chan_spec_spur_26Mhz [][3] = {
	{1, 3, 19},
	{2, 5, 3},
	{3, 4, 51},
	{11, 3, 26},
	{12, 3, 10},
	{13, 3, 58}
};
uint8 chan_spec_spur_26Mhz_sz = ARRAYSIZE(chan_spec_spur_26Mhz);
STATIC uint8 chan_spec_spur_37p4Mhz [][3] = {
	{4, 3, 13},
	{5, 3, 61},
	{6, 3, 45},
	{11, 3, 20},
	{12, 3, 4},
	{13, 3, 52}
};
uint8 chan_spec_spur_37p4Mhz_sz = ARRAYSIZE(chan_spec_spur_37p4Mhz);
STATIC uint8 chan_spec_spur_xtlna38p4Mhz [][3] = {
	{7, 2, 57},
	{7, 2, 58},
	{11, 2, 6},
	{11, 2, 7},
	{13, 2, 25},
	{13, 2, 26},
	{13, 2, 38},
	{13, 2, 39}
};
uint8 chan_spec_spur_xtlna38p4Mhz_sz = ARRAYSIZE(chan_spec_spur_xtlna38p4Mhz);
STATIC uint8 chan_spec_spur_xtlna26Mhz [][3] = {
	{1, 2, 19},
	{2, 2, 3},
	{3, 2, 51},
	{11, 2, 26},
	{12, 2, 10},
	{13, 2, 58}
};
uint8 chan_spec_spur_xtlna26Mhz_sz = ARRAYSIZE(chan_spec_spur_xtlna26Mhz);
STATIC uint8 chan_spec_spur_xtlna37p4Mhz [][3] = {
	{4, 2, 13},
	{5, 2, 61},
	{6, 2, 45}
}; 
uint8 chan_spec_spur_xtlna37p4Mhz_sz = ARRAYSIZE(chan_spec_spur_xtlna37p4Mhz);
STATIC uint8 chan_spec_spur_rev2_26Mhz [][3] = {
	{1, 3, 147},
	{2, 3, 131},
	{3, 3, 179},
	{3, 3, 115},
	{4, 3, 99},
	{5, 3, 83},
	{5, 3, 38},
	{6, 3, 150},
	{6, 3, 22},
	{7, 3, 134},
	{7, 3, 6},
	{8, 3, 182},
	{8, 3, 118},
	{9, 3, 166},
	{9, 3, 102},
	{9, 3, 58},
	{10, 3, 42},
	{10, 3, 86},
	{11, 3, 154},
	{11, 3, 26},
	{11, 3, 70},
	{12, 3, 138},
	{13, 3, 186}
};
uint8 chan_spec_spur_rev2_26Mhz_sz = ARRAYSIZE(chan_spec_spur_rev2_26Mhz);
static void
wlc_sslpnphy_chanspec_spur_weight(phy_info_t *pi, uint channel, uint8 ptr[][3], uint8 array_size)
{
	uint8 i = 0;
	for (i = 0; i < array_size; i++) {
		if (ptr[i][0] == channel) {
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
				&ptr[i][1], 1, 8, ptr[i][2]);
		}
	}
}

void
wlc_sslpnphy_set_chanspec_tweaks(phy_info_t *pi, chanspec_t chanspec)
{
	uint8 spur_weight;
	uint8 channel = CHSPEC_CHANNEL(chanspec); /* see wlioctl.h */
	uint8 phybw40 = IS40MHZ(pi);
#ifdef BAND5G
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
#endif

	/* Below are some of the settings required for reducing
	   the spur levels on the 4329 reference board
	 */

	if (!NORADIO_ENAB(pi->pubpi)) {
		if ((SSLPNREV_LT(pi->pubpi.phy_rev, 2)) &&
			(CHSPEC_IS2G(pi->radio_chanspec))) {
			si_pmu_chipcontrol(GENERIC_PHY_INFO(pi)->sih, 0, 0xfff, ((0x8 << 0) | (0x1f << 6)));
		}
#ifdef BAND5G
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		mod_phy_reg(pi, SSLPNPHY_HiGainDB,
			SSLPNPHY_HiGainDB_HiGainDB_MASK,
			70 << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
			SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
			0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
		if (freq < 5000) {
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 350);
		} else if (freq < 5180) {
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 320);
		} else if (freq < 5660) {
			if (freq <= 5500)
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 320);
			else
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 300);
		}
		else {
			write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 240);
		}

		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17_SSID ||
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID)) {
			if (freq <= 5500) {
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 400);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					7 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
					BCM94329OLYMPICX17U_SSID) {
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 320);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					3 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				}
			} else if (freq < 5660) {
				write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 320);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					2 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) ==
					BCM94329OLYMPICX17U_SSID) {
					MOD_PHY_REG(pi, SSLPNPHY,
						InputPowerDB, inputpwroffsetdb, 254);
				}
			} else if (freq < 5775) {
				if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) {
					write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 300);
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					252 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				} else if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) {
					write_phy_reg(pi, SSLPNPHY_nfSubtractVal, 280);
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						0 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
				}
			} else if (freq >= 5775) {
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					252 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
			}
		}
	} else
#endif /* BAND5G */
	 {
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
			if (!(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA)) {
				if (XTALFREQ(pi->xtalfreq) == 38400000) {
					if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID)
						wlc_sslpnphy_chanspec_spur_weight(pi, channel,
							chan_spec_spur_tdkmdl_38p4Mhz,
							chan_spec_spur_tdkmdl_38p4Mhz_sz);
					else
						wlc_sslpnphy_chanspec_spur_weight(pi, channel,
							chan_spec_spur_nokref_38p4Mhz,
							chan_spec_spur_nokref_38p4Mhz_sz);
				} else if (XTALFREQ(pi->xtalfreq) == 26000000) {
					wlc_sslpnphy_chanspec_spur_weight(pi, channel,
						chan_spec_spur_26Mhz, chan_spec_spur_26Mhz_sz);
				} else if (XTALFREQ(pi->xtalfreq) == 37400000)
					wlc_sslpnphy_chanspec_spur_weight(pi, channel,
						chan_spec_spur_37p4Mhz,
						chan_spec_spur_37p4Mhz_sz);

				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(253 <<
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				if (XTALFREQ(pi->xtalfreq) == 38400000) {
				if (channel <= 4) {
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam1,
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_MASK,
						0 <<
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam2,
						SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_MASK,
						2 << SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
						120 <<
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
				} else
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					(254 <<
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));

				} else if (XTALFREQ(pi->xtalfreq) == 37400000) {
				if ((channel >= 3) && (channel <= 6)) {
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam1,
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_MASK,
						0 <<
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam2,
						SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_MASK,
						2 << SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
						120 <<
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
				}
				} else {
				if (((channel <= 4) && (channel != 2)) ||
					((channel >= 11) && (channel <= 13))) {
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam1,
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_MASK,
						0 <<
						SSLPNPHY_lnsrOfParam1_ofMaxPThrUpdtThresh_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_lnsrOfParam2,
						SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_MASK,
						2 << SSLPNPHY_lnsrOfParam2_oFiltSyncCtrShft_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_ofdmSyncThresh0,
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_MASK,
						120 <<
						SSLPNPHY_ofdmSyncThresh0_ofdmSyncThresh0_SHIFT);
					mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
						(252 <<
						SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
				}
				}
			} else if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA) {
				if (XTALFREQ(pi->xtalfreq) == 38400000)
					wlc_sslpnphy_chanspec_spur_weight(pi, channel,
						chan_spec_spur_xtlna38p4Mhz,
						chan_spec_spur_xtlna38p4Mhz_sz);
				else if (XTALFREQ(pi->xtalfreq) == 26000000)
					wlc_sslpnphy_chanspec_spur_weight(pi, channel,
						chan_spec_spur_xtlna26Mhz,
						chan_spec_spur_xtlna26Mhz_sz);
				else if (XTALFREQ(pi->xtalfreq) == 37400000)
					wlc_sslpnphy_chanspec_spur_weight(pi, channel,
						chan_spec_spur_xtlna37p4Mhz,
						chan_spec_spur_xtlna37p4Mhz_sz);
				mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
					((channel != 14) ? 1 : 0 <<
					SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
			}

		} else if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) { 	/* 4319 SSLPNPHY REV > 2 */
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
			254 << SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);

			if (phybw40 == 1) {
				mod_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40,
				    SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK,
				    (1 << SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_SHIFT));
			}
			if (XTALFREQ(pi->xtalfreq) == 30000000) {
				if (channel == 13) {
					spur_weight = 2;
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
						&spur_weight, 1, 8, 153);
					spur_weight = 2;
					wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_SPUR,
						&spur_weight, 1, 8, 154);
				}
			}
		}
	}
	}
}

void
wlc_sslpnphy_pktengtx(wlc_phy_t *ppi, wl_pkteng_t *pkteng, uint8 rate,
	struct ether_addr *sa, uint32 wait_delay)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	uint8 counter = 0;
	uint16 max_pwr_idx = 0;
	uint16 min_pwr_idx = 127;
	uint16 current_txidx = 0;
	uint32 ant_ovr;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_psat_2pt3_detected = 0;
	wlc_sslpnphy_btcx_override_enable(pi);
	wlc_sslpnphy_set_deaf(pi);

	/* Force default antenna */
	ant_ovr = wlc_sslpnphy_set_ant_override(pi, wlc_phy_get_txant(pi));

	for (counter = 0; counter < pkteng->nframes; counter ++) {
		wlc_phy_do_dummy_tx(pi, TRUE, OFF);
		OSL_DELAY(pkteng->delay);
		current_txidx = wlc_sslpnphy_get_current_tx_pwr_idx(pi);
		if (current_txidx > max_pwr_idx)
			max_pwr_idx = current_txidx;
		if (current_txidx < min_pwr_idx)
			min_pwr_idx = current_txidx;
	}
	wlc_sslpnphy_clear_deaf(pi);

	/* Restore antenna override */
	wlc_sslpnphy_restore_ant_override(pi, ant_ovr);

	if (pkteng->nframes == 100) {
		/* 10 is the value chosen for a start power of 14dBm */
		if (!((max_pwr_idx == 0) && (min_pwr_idx == 127))) {
			if (((max_pwr_idx - min_pwr_idx) > 10) ||
			(min_pwr_idx == 0)) {
				sslpnphy_specific->sslpnphy_psat_2pt3_detected = 1;
				current_txidx =  max_pwr_idx;
			}
		}
	}
	sslpnphy_specific->sslpnphy_start_idx = (uint8)current_txidx; 	/* debug information */

	WL_INFORM(("wl%d: %s: Max idx %d  Min idx %d \n", GENERIC_PHY_INFO(pi)->unit,
		__FUNCTION__, max_pwr_idx, min_pwr_idx));
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_recal)(phy_info_t *pi)
{

#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	wlc_phy_t * ppi = (wlc_phy_t *)pi;
	uint16 tx_pwr_ctrl;
	bool suspend;
	uint16 current_txidx = 0;
	wl_pkteng_t pkteng;
	struct ether_addr sa;
	uint8 phybw40;
	uint8 channel = CHSPEC_CHANNEL(pi->radio_chanspec); /* see wlioctl.h */
	sslpnphy_txcalgains_t txgains;
	phybw40 = IS40MHZ(pi);

	suspend = (0 == (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	if (!suspend) {
		/* Set non-zero duration for CTS-to-self */
		WL_WRITE_SHM(pi, M_CTS_DURATION, 10000);
		WL_SUSPEND_MAC_AND_WAIT(pi);
	}

	/* temporary arrays needed in child functions of papd cal */
	sslpnphy_specific->sslpnphy_papdIntlut = (uint32 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * sizeof(uint32));
	sslpnphy_specific->sslpnphy_papdIntlutVld = (uint8 *)MALLOC(GENERIC_PHY_INFO(pi)->osh, 128 * sizeof(uint8));

	/* if we dont have enough memory, then exit gracefully */
	if ((sslpnphy_specific->sslpnphy_papdIntlut == NULL) || (sslpnphy_specific->sslpnphy_papdIntlutVld == NULL)) {
		if (sslpnphy_specific->sslpnphy_papdIntlut != NULL) {
			MFREE(GENERIC_PHY_INFO(pi)->osh, sslpnphy_specific->sslpnphy_papdIntlut, 128 * sizeof(uint32));
		}
		if (sslpnphy_specific->sslpnphy_papdIntlutVld != NULL) {
			MFREE(GENERIC_PHY_INFO(pi)->osh, sslpnphy_specific->sslpnphy_papdIntlutVld, 128 * sizeof(uint8));
		}
		WL_ERROR(("wl%d: %s: MALLOC failure\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
		return;
	}

	if ((CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4329_CHIP_ID) ||
		(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID))
		si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 2, 0x00000007, 0x0);
	if (NORADIO_ENAB(pi->pubpi)) {
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			mod_phy_reg(pi, SSLPNPHY_txfefilterctrl,
				SSLPNPHY_txfefilterctrl_txfefilterconfig_en_MASK,
				0 << SSLPNPHY_txfefilterctrl_txfefilterconfig_en_SHIFT);

			mod_phy_reg(pi, SSLPNPHY_txfefilterconfig,
				(SSLPNPHY_txfefilterconfig_cmpxfilt_use_ofdmcoef_4ht_MASK |
				SSLPNPHY_txfefilterconfig_realfilt_use_ofdmcoef_4ht_MASK),
				((1 << SSLPNPHY_txfefilterconfig_cmpxfilt_use_ofdmcoef_4ht_SHIFT) |
				(1 << SSLPNPHY_txfefilterconfig_realfilt_use_ofdmcoef_4ht_SHIFT)));
		}
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
			0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_papd_control,
			SSLPNPHY_papd_control_papdCompEn_MASK,
			0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);

		return;
	}

#ifdef PS4319XTRA
	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)
		 WL_WRITE_SHM(pi, M_PS4319XTRA, 0);
#endif /* PS4319XTRA */
	if ((SSLPNREV_LT(pi->pubpi.phy_rev, 2)) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		/* cellular emission fixes */
		write_radio_reg(pi, RADIO_2063_LOGEN_BUF_1, sslpnphy_specific->sslpnphy_logen_buf_1);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVR_2, sslpnphy_specific->sslpnphy_local_ovr_2);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_6, sslpnphy_specific->sslpnphy_local_oval_6);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_5, sslpnphy_specific->sslpnphy_local_oval_5);
		write_radio_reg(pi, RADIO_2063_LOGEN_MIXER_1, sslpnphy_specific->sslpnphy_logen_mixer_1);
	}
	if ((channel != 14) && (!wlc_sslpnphy_fcc_chan_check(pi, channel)) && IS_OLYMPIC(pi)) {
		/* Resetting all Olympic related microcode settings */
		WL_WRITE_SHM(pi, M_SSLPN_OLYMPIC, 0);
		write_phy_reg(pi, SSLPNPHY_extstxctrl0, sslpnphy_specific->sslpnphy_extstxctrl0);
		write_phy_reg(pi, SSLPNPHY_extstxctrl1, sslpnphy_specific->sslpnphy_extstxctrl1);
	}


	{
	/* Save tx power control mode */
	tx_pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);
	/* Restore pwr ctrl */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);

	wlc_sslpnphy_clear_tx_power_offsets(pi);
	wlc_sslpnphy_set_target_tx_pwr(pi, 56);
	/* Setting npt to 0 for index settling with 30 frames */
	wlc_sslpnphy_set_tx_pwr_npt(pi, 0);

	}
	if (!sslpnphy_specific->sslpnphy_force_1_idxcal) {
		/* Enabling Complex filter before transmitting dummy frames */
		/* Check if this is redundant because ucode already does this */
		mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
			(SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_MASK |
			SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_MASK |
			SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_MASK),
			((1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_SHIFT) |
			(1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_SHIFT) |
			(1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_SHIFT)));
		if (SSLPNREV_IS(pi->pubpi.phy_rev, 0)) {
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
				1  << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_papd_control,
				SSLPNPHY_papd_control_papdCompEn_MASK,
				0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
		}

		if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				(SSLPNPHY_Core1TxControl_txrealfilten_MASK |
				SSLPNPHY_Core1TxControl_txcomplexfilten_MASK |
				SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_MASK),
				((1 << SSLPNPHY_Core1TxControl_txrealfilten_SHIFT) |
				(1  << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT) |
				(1 << SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_SHIFT)));

			mod_phy_reg(pi, SSLPNPHY_papd_control,
				SSLPNPHY_papd_control_papdCompEn_MASK,
				0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
		}
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			mod_phy_reg(pi, SSLPNPHY_txfefilterctrl,
				SSLPNPHY_txfefilterctrl_txfefilterconfig_en_MASK,
				1 << SSLPNPHY_txfefilterctrl_txfefilterconfig_en_SHIFT);

			mod_phy_reg(pi, SSLPNPHY_txfefilterconfig,
				(SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_MASK |
				SSLPNPHY_txfefilterconfig_ofdm_realfilten_MASK |
				SSLPNPHY_txfefilterconfig_ofdm_papden_MASK),
				((1 << SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_SHIFT) |
				(1 << SSLPNPHY_txfefilterconfig_ofdm_realfilten_SHIFT) |
				(0 << SSLPNPHY_txfefilterconfig_ofdm_papden_SHIFT)));
		}
		/* clear our PAPD Compensation table */
		wlc_sslpnphy_clear_papd_comptable(pi);

		mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
			SSLPNPHY_TxPwrCtrlDeltaPwrLimit_DeltaPwrLimit_MASK,
			3 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_DeltaPwrLimit_SHIFT);

		if (SSLPNREV_LE(pi->pubpi.phy_rev, 1)) {
			write_radio_reg(pi, RADIO_2063_PA_CTRL_14, 0xee);
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txrealfiltcoefsel_MASK,
				1 << SSLPNPHY_Core1TxControl_txrealfiltcoefsel_SHIFT);
		}
		current_txidx = wlc_sslpnphy_get_current_tx_pwr_idx(pi);
		if (!sslpnphy_specific->sslpnphy_restore_papd_cal_results) {
			{
				sa.octet[0] = 10;

				pkteng.flags = WL_PKTENG_PER_TX_START;
				pkteng.delay = 2;		/* Inter packet delay */
				pkteng.nframes = 50;		/* number of frames */
				pkteng.length = 0;		/* packet length */
				pkteng.seqno = FALSE;	/* enable/disable sequence no. */

				wlc_sslpnphy_pktengtx(ppi, &pkteng, 108, &sa, (1000*10));
				sslpnphy_specific->sslpnphy_dummy_tx_done = 1;
			}
			/* sending out 100 frames to caluclate min & max index */
			if (VBAT_RIPPLE_CHECK(pi)) {
				pkteng.delay = 30;		/* Inter packet delay */
				pkteng.nframes = 100;		/* number of frames */
				wlc_sslpnphy_pktengtx(ppi, &pkteng, 108, &sa, (1000*10));
			}
			current_txidx = sslpnphy_specific->sslpnphy_start_idx; 	/* debug information */
		}
	}

	/* disabling complex filter for PAPD calibration */
	if (SSLPNREV_IS(pi->pubpi.phy_rev, 0)) {
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
			0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
	}

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		(SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_MASK |
		SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_MASK |
		SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_MASK),
		((0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_SHIFT) |
		(0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_SHIFT) |
		(0 << SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_SHIFT)));

	if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			(SSLPNPHY_Core1TxControl_txcomplexfilten_MASK |
			SSLPNPHY_Core1TxControl_txrealfilten_MASK |
			SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_MASK),
			((0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT) |
			(0 << SSLPNPHY_Core1TxControl_txrealfilten_SHIFT) |
			(0 << SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_SHIFT)));
	}
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_txfefilterctrl,
			SSLPNPHY_txfefilterctrl_txfefilterconfig_en_MASK,
			0 << SSLPNPHY_txfefilterctrl_txfefilterconfig_en_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_txfefilterconfig,
			(SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_MASK |
			SSLPNPHY_txfefilterconfig_ofdm_realfilten_MASK),
			((0 << SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_SHIFT) |
			(0 << SSLPNPHY_txfefilterconfig_ofdm_realfilten_SHIFT)));

	}
	wlc_sslpnphy_set_deaf(pi);

	if (!sslpnphy_specific->sslpnphy_restore_papd_cal_results) {
		if (!NON_BT_CHIP(wlc))
			wlc_sslpnphy_btcx_override_enable(pi);
	}

	/* Setting npt to 1 for normal transmission */
	wlc_sslpnphy_set_tx_pwr_npt(pi, 1);

	/* Save tx power control mode */
	tx_pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
	/* Disable tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);

	sslpnphy_specific->sslpnphy_papd_rxGnCtrl_init = 0;

	txgains.useindex = TRUE;
	txgains.index = (uint8) current_txidx;
	if (!sslpnphy_specific->sslpnphy_restore_papd_cal_results) {
		if (!sslpnphy_specific->sslpnphy_force_1_idxcal)
			wlc_sslpnphy_vbatsense_papd_cal(pi, SSLPNPHY_PAPD_CAL_CW, &txgains);
		wlc_sslpnphy_papd_cal_txpwr(pi, SSLPNPHY_PAPD_CAL_CW, FALSE, TRUE, current_txidx);
	} else {
		wlc_sslpnphy_restore_papd_calibration_results(pi);
	}
	/* Restore tx power control */
	wlc_sslpnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);

	mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlDeltaPwrLimit,
		SSLPNPHY_TxPwrCtrlDeltaPwrLimit_DeltaPwrLimit_MASK,
		1 << SSLPNPHY_TxPwrCtrlDeltaPwrLimit_DeltaPwrLimit_SHIFT);

	if (SSLPNREV_IS(pi->pubpi.phy_rev, 0)) {
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
			1 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_papd_control,
			SSLPNPHY_papd_control_papdCompEn_MASK,
			0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
	}

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		(SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_MASK |
		SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_MASK |
		SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_MASK),
		((1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdFiltClkEn_SHIFT) |
		(1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdTxClkEn_SHIFT) |
		(1 << SSLPNPHY_sslpnCalibClkEnCtrl_papdRxClkEn_SHIFT)));

	if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			(SSLPNPHY_Core1TxControl_txrealfilten_MASK |
			SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_MASK |
			SSLPNPHY_Core1TxControl_txcomplexfilten_MASK),
			((1 << SSLPNPHY_Core1TxControl_txrealfilten_SHIFT) |
			(1 << SSLPNPHY_Core1TxControl_txcomplexfiltb4papd_SHIFT) |
			(1 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT)));

		mod_phy_reg(pi, SSLPNPHY_papd_control,
			SSLPNPHY_papd_control_papdCompEn_MASK,
			1 << SSLPNPHY_papd_control_papdCompEn_SHIFT);

		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID) {
			write_phy_reg(pi, SSLPNPHY_txClipBpsk, 0x078f);
			write_phy_reg(pi, SSLPNPHY_txClipQpsk, 0x078f);
		}
	}
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_txfiltctrl,
			SSLPNPHY_txfiltctrl_txcomplexfiltb4papd_MASK,
			1 << SSLPNPHY_txfiltctrl_txcomplexfiltb4papd_SHIFT);

		mod_phy_reg(pi, SSLPNPHY_txfefilterconfig,
			(SSLPNPHY_txfefilterconfig_cmpxfilt_use_ofdmcoef_4ht_MASK |
			SSLPNPHY_txfefilterconfig_realfilt_use_ofdmcoef_4ht_MASK |
			SSLPNPHY_txfefilterconfig_ofdm_papden_MASK |
			SSLPNPHY_txfefilterconfig_ht_papden_MASK |
			SSLPNPHY_txfefilterconfig_cck_realfilten_MASK |
			SSLPNPHY_txfefilterconfig_cck_cmpxfilten_MASK |
			SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_MASK |
			SSLPNPHY_txfefilterconfig_ofdm_realfilten_MASK |
			SSLPNPHY_txfefilterconfig_ht_cmpxfilten_MASK |
			SSLPNPHY_txfefilterconfig_ht_realfilten_MASK),
			(((!phybw40) << SSLPNPHY_txfefilterconfig_cmpxfilt_use_ofdmcoef_4ht_SHIFT) |
			((!phybw40) << SSLPNPHY_txfefilterconfig_realfilt_use_ofdmcoef_4ht_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ofdm_papden_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ht_papden_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_cck_realfilten_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_cck_cmpxfilten_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ofdm_cmpxfilten_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ofdm_realfilten_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ht_cmpxfilten_SHIFT) |
			(1 << SSLPNPHY_txfefilterconfig_ht_realfilten_SHIFT)));

		mod_phy_reg(pi, SSLPNPHY_txfefilterctrl,
			SSLPNPHY_txfefilterctrl_txfefilterconfig_en_MASK,
			1 << SSLPNPHY_txfefilterctrl_txfefilterconfig_en_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_papd_control,
			SSLPNPHY_papd_control_papdCompEn_MASK,
			0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
	}
	sslpnphy_specific->sslpnphy_papd_cal_done = 1;
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
			SSLPNPHY_Core1TxControl_txClipEnable_ofdm_MASK,
			1 << SSLPNPHY_Core1TxControl_txClipEnable_ofdm_SHIFT);

		if (phybw40 == 1) {
			write_phy_reg(pi, SSLPNPHY_txClipBpsk, 0x0aff);
			write_phy_reg(pi, SSLPNPHY_txClipQpsk, 0x0bff);
			write_phy_reg(pi, SSLPNPHY_txClip16Qam, 0x7fff);
			write_phy_reg(pi, SSLPNPHY_txClip64Qam, 0x7fff);
		} else { /* No clipping for 20Mhz */
				write_phy_reg(pi, SSLPNPHY_txClipBpsk, 0x7fff);
				write_phy_reg(pi, SSLPNPHY_txClipQpsk, 0x7fff);
				write_phy_reg(pi, SSLPNPHY_txClip16Qam, 0x7fff);
				write_phy_reg(pi, SSLPNPHY_txClip64Qam, 0x7fff);
		}
	}
	if ((SSLPNREV_LT(pi->pubpi.phy_rev, 2)) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		/* cellular emission fixes */
		write_radio_reg(pi, RADIO_2063_LOGEN_BUF_1, 0x06);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVR_2, 0x0f);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_6, 0xff);
		write_radio_reg(pi, RADIO_2063_LOCAL_OVAL_5, 0xff);
		write_radio_reg(pi, RADIO_2063_LOGEN_MIXER_1, 0x66);
	}
	if (IS_OLYMPIC(pi)) {
		uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
		uint16 olympic_flag;
		olympic_flag = WL_READ_SHM(pi, M_SSLPN_OLYMPIC);
		WL_WRITE_SHM(pi, M_SSLPN_OLYMPIC, olympic_flag | 1);
		if (channel != 14) {
			if (!(wlc_sslpnphy_fcc_chan_check(pi, channel))) {
				if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
					(sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_16_64)), 0x1600);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_16_64)), 0x3300);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_2_4)), 0x1550);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_2_4)), 0x3300);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_CCK)), 0x2500);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_CCK)), 0x30c0);
				} else {
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_16_64)), 0x9210);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_16_64)), 0x3150);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_2_4)), 0xf000);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_2_4)), 0x30c3);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F2_CCK)), 0x2280);
					WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
						M_SSLPNPHY_REG_4F3_CCK)), 0x3150);
				}

				olympic_flag = WL_READ_SHM(pi, M_SSLPN_OLYMPIC);
				WL_WRITE_SHM(pi, M_SSLPN_OLYMPIC,
					(olympic_flag | (0x1 << 1)));
			}
		}

	}

	if (channel == 14) {
		mod_phy_reg(pi, SSLPNPHY_lpfbwlutreg0,
			SSLPNPHY_lpfbwlutreg0_lpfbwlut0_MASK,
			2 << SSLPNPHY_lpfbwlutreg0_lpfbwlut0_SHIFT);
		mod_phy_reg(pi, SSLPNPHY_papd_control,
			SSLPNPHY_papd_control_papdCompEn_MASK,
			0 << SSLPNPHY_papd_control_papdCompEn_SHIFT);
		/* Disable complex filter for 4329 and 4319 */
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2))
			mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
				SSLPNPHY_Core1TxControl_txcomplexfilten_MASK,
				0 << SSLPNPHY_Core1TxControl_txcomplexfilten_SHIFT);
		else
			mod_phy_reg(pi, SSLPNPHY_txfefilterconfig,
				SSLPNPHY_txfefilterconfig_cck_cmpxfilten_MASK,
				0 << SSLPNPHY_txfefilterconfig_cck_cmpxfilten_SHIFT);
	} else
		write_phy_reg(pi, SSLPNPHY_lpfbwlutreg0, sslpnphy_specific->sslpnphy_filt_bw);

	wlc_sslpnphy_tempsense(pi);
	sslpnphy_specific->sslpnphy_last_cal_temperature = sslpnphy_specific->sslpnphy_lastsensed_temperature;
	sslpnphy_specific->sslpnphy_last_full_cal_temperature = sslpnphy_specific->sslpnphy_lastsensed_temperature;
	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);
	if ((CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4329_CHIP_ID) ||
		(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID))
		si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 2, 0x00000007, 0x00000005);
#ifdef PS4319XTRA
	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)
		WL_WRITE_SHM(pi, M_PS4319XTRA, PS4319XTRA);
#endif /* PS4319XTRA */

	if (!suspend)
		WL_ENABLE_MAC(pi);
	wlc_sslpnphy_clear_deaf(pi);

	MFREE(GENERIC_PHY_INFO(pi)->osh, sslpnphy_specific->sslpnphy_papdIntlut, 128 * sizeof(uint32));
	MFREE(GENERIC_PHY_INFO(pi)->osh, sslpnphy_specific->sslpnphy_papdIntlutVld, 128 * sizeof(uint8));
}
static void
wlc_sslpnphy_load_filt_coeff(phy_info_t *pi, uint16 reg_address, uint16 *coeff_val, uint count)
{
	uint i;
	for (i = 0; i < count; i++)
		write_phy_reg(pi, reg_address + i, coeff_val[i]);
}
static void wlc_sslpnphy_ofdm_filt_load(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
		wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
			sslpnphy_cx_ofdm[sslpnphy_specific->sslpnphy_ofdm_filt_sel], 10);
		wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_txrealfilt_ofdm_tap0,
			sslpnphy_real_ofdm[sslpnphy_specific->sslpnphy_ofdm_filt_sel], 5);
	}
}

void wlc_sslpnphy_cck_filt_load(phy_info_t *pi, uint8 filtsel)
{
	if (SSLPNREV_GT(pi->pubpi.phy_rev, 0)) {
		wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_cck_tap0_i,
			sslpnphy_cx_cck[filtsel], 10);
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_cck_tap0,
				sslpnphy_real_cck[filtsel], 5);
		} else {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_txrealfilt_cck_tap0,
				sslpnphy_real_cck[filtsel], 5);
		}
	}

}
static void
wlc_sslpnphy_restore_txiqlo_calibration_results(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint16 a, b;
	uint16 didq;
	uint32 val;
	uint idx;
	uint8 ei0, eq0, fi0, fq0;
	uint8 band_idx;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	ASSERT(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs_valid);

	a = sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[0];
	b = sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[1];
	didq = sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[5];

	wlc_sslpnphy_set_tx_iqcc(pi, a, b);
	wlc_sslpnphy_set_tx_locc(pi, didq);

	/* restore iqlo portion of tx power control tables */
	/* remaining element */
	for (idx = 0; idx < 128; idx++) {
		/* iq */
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL, &val,
			1, 32, SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + idx);
		val = (val & 0xfff00000) |
			((uint32)(a & 0x3FF) << 10) | (b & 0x3ff);
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL, &val,
			1, 32, SSLPNPHY_TX_PWR_CTRL_IQ_OFFSET + idx);
		/* loft */
		val = didq;
		wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_TXPWRCTL, &val,
			1, 32, SSLPNPHY_TX_PWR_CTRL_LO_OFFSET + idx);
	}
	/* Do not move the below statements up */
	/* We need atleast 2us delay to read phytable after writing radio registers */
	/* Apply analog LO */
	ei0 = (uint8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[7] >> 8);
	eq0 = (uint8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[7]);
	fi0 = (uint8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[9] >> 8);
	fq0 = (uint8)(sslpnphy_specific->sslpnphy_cal_results[band_idx].txiqlocal_bestcoeffs[9]);
	wlc_sslpnphy_set_radio_loft(pi, ei0, eq0, fi0, fq0);
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_save_papd_calibration_results)(phy_info_t *pi)
{
	uint8 band_idx;
	uint8 a, i;
	uint32 papdcompdeltatblval;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	/* Save papd calibration results */
	sslpnphy_specific->sslpnphy_cal_results[band_idx].analog_gain_ref = read_phy_reg(pi,
		SSLPNPHY_papd_tx_analog_gain_ref);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_begin = read_phy_reg(pi,
		SSLPNPHY_papd_track_pa_lut_begin);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_end = read_phy_reg(pi,
		SSLPNPHY_papd_track_pa_lut_end);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_step = read_phy_reg(pi,
		SSLPNPHY_papd_track_pa_lut_step);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].rxcompdbm = read_phy_reg(pi,
		SSLPNPHY_papd_rx_gain_comp_dbm);
	sslpnphy_specific->sslpnphy_cal_results[band_idx].papdctrl = read_phy_reg(pi, SSLPNPHY_papd_control);
	/* Save papdcomp delta table */
	for (a = 1, i = 0; a < 128; a = a + 2, i ++) {
		wlc_sslpnphy_common_read_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
			&papdcompdeltatblval, 1, 32, a);
		sslpnphy_specific->sslpnphy_cal_results[band_idx].papd_compdelta_tbl[i] = papdcompdeltatblval;
	}
	sslpnphy_specific->sslpnphy_cal_results[band_idx].papd_table_valid = 1;
}

OSTATIC void
BCMOVERLAYFN(1, wlc_sslpnphy_restore_papd_calibration_results)(phy_info_t *pi)
{
	uint8 band_idx;
	uint8 a, i;
	uint32 papdcompdeltatblval;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);
	if (sslpnphy_specific->sslpnphy_cal_results[band_idx].papd_table_valid) {
		/* Apply PAPD cal results */
		for (a = 1, i = 0; a < 128; a = a + 2, i ++) {
			papdcompdeltatblval = sslpnphy_specific->sslpnphy_cal_results
				[band_idx].papd_compdelta_tbl[i];
			wlc_sslpnphy_common_write_table(pi, SSLPNPHY_TBL_ID_PAPDCOMPDELTATBL,
				&papdcompdeltatblval, 1, 32, a);
		}
		/* Writing the deltas */
		wlc_sslpnphy_compute_delta(pi);

		/* Restore saved papd regs */
		write_phy_reg(pi, SSLPNPHY_papd_tx_analog_gain_ref,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].analog_gain_ref);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_begin,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_begin);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_end,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_end);
		write_phy_reg(pi, SSLPNPHY_papd_track_pa_lut_step,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].lut_step);
		write_phy_reg(pi, SSLPNPHY_papd_rx_gain_comp_dbm,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].rxcompdbm);
		write_phy_reg(pi, SSLPNPHY_papd_control,
			sslpnphy_specific->sslpnphy_cal_results[band_idx].papdctrl);
	}
}

static void
wlc_sslpnphy_restore_calibration_results(phy_info_t *pi)
{
	uint8 band_idx;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	wlc_sslpnphy_restore_txiqlo_calibration_results(pi);

	/* restore rx iq cal results */
	wlc_sslpnphy_set_rx_iq_comp(pi,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiqcal_coeffa0,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiqcal_coeffb0);

	write_phy_reg(pi, SSLPNPHY_RxIqCoeffCtrl,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].rxiq_enable);
	write_phy_reg(pi, SSLPNPHY_rxfe, sslpnphy_specific->sslpnphy_cal_results[band_idx].rxfe);
	write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_1,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].loopback1);
	write_radio_reg(pi, RADIO_2063_TXRX_LOOPBACK_2,
		sslpnphy_specific->sslpnphy_cal_results[band_idx].loopback2);

}

void 
wlc_sslpnphy_percal_flags_off(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_recal = 0;
	sslpnphy_specific->sslpnphy_force_1_idxcal = 0;
	sslpnphy_specific->sslpnphy_vbat_ripple = 0;
	sslpnphy_specific->sslpnphy_percal_ctr = 0;
	sslpnphy_specific->sslpnphy_papd_nxt_cal_idx = 0;
	sslpnphy_specific->sslpnphy_tx_idx_prev_cal = 0;
	sslpnphy_specific->sslpnphy_txidx_drift = 0;
	sslpnphy_specific->sslpnphy_cur_idx = 0;
	sslpnphy_specific->sslpnphy_restore_papd_cal_results = 0;
	sslpnphy_specific->sslpnphy_dummy_tx_done = 0;
	sslpnphy_specific->sslpnphy_papd_cal_done = 0;
	sslpnphy_specific->sslpnphy_init_noise_cal_done = FALSE;
	sslpnphy_specific->sslpnphy_papd_tweaks_enable = FALSE;

	if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12))
		sslpnphy_specific->sslpnphy_radio_classA = TRUE;
	else
		sslpnphy_specific->sslpnphy_radio_classA = FALSE;
}
static bool wlc_sslpnphy_fcc_chan_check(phy_info_t *pi, uint channel)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */

	/* No tunings required currently for TSMC */
	if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12))
		return FALSE;

	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICLOCO_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN18_SSID)) {

		if ((channel == 1) || (channel == 11))
			return TRUE;
	} else if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGB_SSID)) {
		if ((channel == 1) || (channel == 11) || (channel == 13))
			return TRUE;
	}
	return FALSE;

}

void
WLBANDINITFN(wlc_phy_init_sslpnphy)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#else
	wlc_info_t * wlc_pi = pi->wlc;
#endif /* PHYHAL */
	uint8 i;
	uint8 phybw40;
	uint8 band_idx;
	uint channel = CHSPEC_CHANNEL(pi->radio_chanspec);
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	sslpnphy_specific->sslpnphy_OLYMPIC = ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90U_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN90M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICN18_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICUNO_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICLOCO_SSID ||
		((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) &&
		(CHSPEC_IS5G(pi->radio_chanspec))) ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
		BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ? 1 : 0);
	phybw40 = IS40MHZ(pi);

	wlc_sslpnphy_percal_flags_off(pi);

	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)
		si_pmu_chipcontrol(GENERIC_PHY_INFO(pi)->sih, 2, 0x0003f000, (0xa << 12));

	if ((CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4329_CHIP_ID) ||
		(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID))
		si_pmu_regcontrol(GENERIC_PHY_INFO(pi)->sih, 2, 0x00000007, 0x00000005);

	/* initializing the adc-presync and auxadc-presync for 2x sampling */
	wlc_sslpnphy_afe_clk_init(pi, AFE_CLK_INIT_MODE_TXRX2X);

	/* Initialize baseband */
	wlc_sslpnphy_baseband_init(pi);

	/* Initialize radio */
	wlc_sslpnphy_radio_init(pi);

	/* Run RC Cal */
	wlc_sslpnphy_rc_cal(pi);

	if (!NORADIO_ENAB(pi->pubpi)) {
		write_radio_reg(pi, RADIO_2063_TXBB_SP_3, 0x3f);
	}
	sslpnphy_specific->sslpnphy_filt_bw = read_phy_reg(pi, SSLPNPHY_lpfbwlutreg0);
	sslpnphy_specific->sslpnphy_ofdm_filt_bw = read_phy_reg(pi, SSLPNPHY_lpfbwlutreg1);
	sslpnphy_specific->sslpnphy_extstxctrl0 = read_phy_reg(pi, SSLPNPHY_extstxctrl0);
	sslpnphy_specific->sslpnphy_extstxctrl1 = read_phy_reg(pi, SSLPNPHY_extstxctrl1);
	sslpnphy_specific->sslpnphy_extstxctrl2 = read_phy_reg(pi, SSLPNPHY_extstxctrl2);
	sslpnphy_specific->sslpnphy_extstxctrl3 = read_phy_reg(pi, SSLPNPHY_extstxctrl3);
	sslpnphy_specific->sslpnphy_extstxctrl4 = read_phy_reg(pi, SSLPNPHY_extstxctrl4);
	sslpnphy_specific->sslpnphy_extstxctrl5 = read_phy_reg(pi, SSLPNPHY_extstxctrl5);

	/* Tune to the current channel */
	wlc_phy_chanspec_set_sslpnphy(pi, pi->radio_chanspec);

	/* Some of the CRS/AGC values are dependent on Channel and VT. So initialise here
	 *  to known values
	*/
	wlc_sslpnphy_set_chanspec_tweaks(pi, pi->radio_chanspec);

	wlc_sslpnphy_CmRxAciGainTbl_Tweaks(pi);

	wlc_sslpnphy_agc_temp_init(pi);

	{
		for (i = 0; i < TXP_NUM_RATES; i++)
			sslpnphy_specific->sslpnphy_saved_tx_user_target[i] = pi->txpwr_limit[i];
	}

	/* Run initial calibration */
	if (sslpnphy_specific->sslpnphy_full_cal_chanspec[band_idx] != pi->radio_chanspec) {
#ifdef DONGLEOVERLAYS
		if (pi->phyinit_state != PHYINIT_STATE_DONE) {
			uint32 arg = 0;
			int ret;

			ret = wlc_send_overlay_event(pi->wlc, WLC_SET_VAR, PHYCAL_OVERLAY,
			                             "sslpnphy_fullcal", &arg, sizeof(uint32),
			                             WLC_E_OVL_DOWNLOAD);
			if (ret)
				WL_ERROR(("wl%d: %s: wlc_send_overlay_event failed w/status %d\n",
				          GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, ret));
		}
#else
		wlc_sslpnphy_full_cal(pi);
#endif /* DONGLEOVERLAYS */
	} else {
		wlc_sslpnphy_restore_calibration_results(pi);
		sslpnphy_specific->sslpnphy_restore_papd_cal_results = 1;
	}

	wlc_sslpnphy_tempsense(pi);
	wlc_sslpnphy_temp_adj(pi);
	wlc_sslpnphy_cck_filt_load(pi, sslpnphy_specific->sslpnphy_cck_filt_sel);
	if (SSLPNREV_IS(pi->pubpi.phy_rev, 0)) {
		wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_cck_tap0_i,
			sslpnphy_rev0_cx_cck, 10);
		wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
			sslpnphy_rev0_cx_ofdm, 10);

	}
	if (SSLPNREV_IS(pi->pubpi.phy_rev, 1)) {
		if (!NORADIO_ENAB(pi->pubpi)) {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
				sslpnphy_rev1_cx_ofdm, 10);
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_txrealfilt_ofdm_tap0,
				sslpnphy_rev1_real_ofdm, 5);
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_txrealfilt_ht_tap0,
				sslpnphy_rev1_real_ht, 5);

			/* NOK ref board sdagb  and TDK module Es2.11 requires */
			/*  special tuning for spectral flatness */
			if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGB_SSID) ||
				((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) &&
				(CHSPEC_IS2G(pi->radio_chanspec))))
				wlc_sslpnphy_ofdm_filt_load(pi);
			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329TDKMDL11_SSID)
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_txrealfilt_ofdm_tap0,
					sslpnphy_tdk_mdl_real_ofdm, 5);

			if (IS_OLYMPIC(pi)) {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
				sslpnphy_olympic_cx_ofdm, 10);
				if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
					(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID))
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
						sslpnphy_rev1_cx_ofdm, 10);
			}
			if (wlc_sslpnphy_fcc_chan_check(pi, channel)) {
			    if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
			        BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) {
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_ofdm_tap0_i,
					    sslpnphy_rev1_cx_ofdm_fcc, 10);
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_txrealfilt_ofdm_tap0,
					    sslpnphy_rev1_real_ofdm_fcc, 5);
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_txrealfilt_ht_tap0,
					    sslpnphy_rev1_real_ht_fcc, 5);
			    }
			}

			if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID ||
				BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) {
				if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
					(sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_ofdm_tap0_i,
					    sslpnphy_rev1_cx_ofdm_sec, 10);
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_txrealfilt_ofdm_tap0,
					    sslpnphy_rev1_real_ofdm_sec, 5);
					wlc_sslpnphy_load_filt_coeff(pi,
					    SSLPNPHY_txrealfilt_ht_tap0,
					    sslpnphy_rev1_real_ht_sec, 5);
			    }
			}
		}
	}
	if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
		if (!NORADIO_ENAB(pi->pubpi)) {
			if (SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {
				if (phybw40 == 1)
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_ofdm_tap0,
					sslpnphy_rev4_phybw40_real_ofdm, 5);
				else
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_ofdm_tap0,
					sslpnphy_rev4_real_ofdm, 5);
			} else {
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_ofdm_tap0,
					sslpnphy_rev2_real_ofdm, 5);
			}
			if (phybw40 == 1) {
				if (SSLPNREV_IS(pi->pubpi.phy_rev, 4))
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
						sslpnphy_rev4_phybw40_cx_ofdm, 10);
				else
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
						sslpnphy_rev2_phybw40_cx_ofdm, 10);
			} else {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
				sslpnphy_rev4_cx_ofdm, 10);
			}

			if (phybw40 == 1) {
				if (SSLPNREV_IS(pi->pubpi.phy_rev, 4))
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_ht_tap0_i,
						sslpnphy_rev4_phybw40_cx_ht, 10);
				else
					wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_ht_tap0_i,
						sslpnphy_rev2_phybw40_cx_ht, 10);
			} else {
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_ht_tap0_i,
					sslpnphy_rev2_cx_ht, 10);
			}
			if (phybw40 == 1) {
				if (SSLPNREV_IS(pi->pubpi.phy_rev, 4))
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_ht_tap0,
					sslpnphy_rev4_phybw40_real_ht, 5);
				else
				wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_Rev2_txrealfilt_ht_tap0,
					sslpnphy_rev2_phybw40_real_ht, 5);
			}
		} else {
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_cck_tap0_i,
				sslpnphy_rev2_default, 10);
			wlc_sslpnphy_load_filt_coeff(pi, SSLPNPHY_ofdm_tap0_i,
				sslpnphy_rev2_default, 10);

		}
	}
	mod_phy_reg(pi, SSLPNPHY_Core1TxControl,
		SSLPNPHY_Core1TxControl_txClipEnable_ofdm_MASK,
		0 << SSLPNPHY_Core1TxControl_txClipEnable_ofdm_SHIFT);
	write_phy_reg(pi, SSLPNPHY_txClipBpsk, 0x7fff);
	write_phy_reg(pi, SSLPNPHY_txClipQpsk, 0x7fff);

	if (channel == 14)
		wlc_sslpnphy_cck_filt_load(pi, 4);

	sslpnphy_specific->sslpnphy_noise_samples = SSLPNPHY_NOISE_SAMPLES_DEFAULT;

	if ((SSLPNREV_LT(pi->pubpi.phy_rev, 2)) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		/* cellular emission fixes */
		sslpnphy_specific->sslpnphy_logen_buf_1 = read_radio_reg(pi, RADIO_2063_LOGEN_BUF_1);
		sslpnphy_specific->sslpnphy_local_ovr_2 = read_radio_reg(pi, RADIO_2063_LOCAL_OVR_2);
		sslpnphy_specific->sslpnphy_local_oval_6 = read_radio_reg(pi, RADIO_2063_LOCAL_OVAL_6);
		sslpnphy_specific->sslpnphy_local_oval_5 = read_radio_reg(pi, RADIO_2063_LOCAL_OVAL_5);
		sslpnphy_specific->sslpnphy_logen_mixer_1 = read_radio_reg(pi, RADIO_2063_LOGEN_MIXER_1);
	}
	/* Switch on the power control */
	WL_INFORM(("init pre  t=%d, %d \n", sslpnphy_specific->sslpnphy_auxadc_val, sslpnphy_specific->sslpnphy_tssi_val));
	wlc_sslpnphy_tx_pwr_ctrl_init(pi);
	WL_INFORM(("init post  t=%d, %d \n", sslpnphy_specific->sslpnphy_auxadc_val, sslpnphy_specific->sslpnphy_tssi_val));

#ifdef DONGLEOVERLAYS
	if (pi->phyinit_state != PHYINIT_STATE_DONE) {
		uint32 arg = 0;
		int ret;

		ret = wlc_send_overlay_event(wlc_pi, WLC_SET_VAR, PHYCAL_OVERLAY,
		                             "sslpnphy_papd_recal", &arg, sizeof(uint32),
		                             WLC_E_OVL_DOWNLOAD);
		if (ret)
			WL_ERROR(("wl%d: %s: wlc_send_overlay_event sslpnphy_papd_recal "
			          "failed w/status %d\n",
			          GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, ret));
	}
#else
	/* PAPD Calibration during init time */
	if (!(SCAN_IN_PROGRESS(wlc_pi) || WLC_RM_IN_PROGRESS(wlc_pi))) {
		wlc_sslpnphy_papd_recal(pi);
		/* Skip tx iq if init is happening on same channel (Time savings) */
		if (!sslpnphy_specific->sslpnphy_restore_papd_cal_results)
			wlc_sslpnphy_txpwrtbl_iqlo_cal(pi);
	} else {
		WL_INFORM((" %s : Not doing a full cal: Restoring the "
			"previous cal results for channel %d ",
			__FUNCTION__, sslpnphy_specific->sslpnphy_full_cal_channel[band_idx]));
		sslpnphy_specific->sslpnphy_restore_papd_cal_results = 1;
		wlc_sslpnphy_papd_recal(pi);
	}
#endif /* DONGLEOVERLAYS */

	wlc_sslpnphy_noise_init(pi);

	/* For olympic UNO Boards, control the turning off of eLNA during Tx */
	/* This code can be moved to a better place */
	if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_EXTLNA_TX)
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
			M_SSLPNPHY_LNA_TX)), 1);
#ifdef SSLPNLOWPOWER
	mod_radio_reg(pi, RADIO_2063_COMMON_03, 0x20, 0x20);
	mod_radio_reg(pi, RADIO_2063_GRX_SP_3, 0xf0, 0x00);
	write_radio_reg(pi, RADIO_2063_RXBB_CTRL_4, 0x00);
	write_radio_reg(pi, RADIO_2063_RXBB_CTRL_3, 0x00);
	mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_7, 0xfc, 0x00);
	write_radio_reg(pi, RADIO_2063_GRX_PS_1, 0x00);
	write_radio_reg(pi, RADIO_2063_RXBB_CTRL_1, 0xf4);
	/* ADC Low Power Mode */
	write_phy_reg(pi, SSLPNPHY_AfeADCCtrl0, 0x8022);
	write_phy_reg(pi, SSLPNPHY_AfeADCCtrl1, 0x422);
	write_phy_reg(pi, SSLPNPHY_AfeADCCtrl2, 0x0040);
#endif
}

static void
wlc_sslpnphy_noise_init(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 phybw40 = IS40MHZ(pi);

	sslpnphy_specific->sslpnphy_NPwr_MinLmt = SSLPNPHY_NPWR_MINLMT;
	sslpnphy_specific->sslpnphy_NPwr_MaxLmt = SSLPNPHY_NPWR_MAXLMT_2G;
	sslpnphy_specific->sslpnphy_noise_measure_window = SSLPNPHY_NOISE_MEASURE_WINDOW_2G;
	sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt = SSLPNPHY_MAX_GAIN_CHANGE_LMT_2G;
	sslpnphy_specific->sslpnphy_max_rxpo_change_lmt = SSLPNPHY_MAX_RXPO_CHANGE_LMT_2G;
	if (phybw40 || (CHSPEC_IS5G(pi->radio_chanspec))) {
		sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt = SSLPNPHY_NPWR_LGC_MINLMT_40MHZ;
		sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt = SSLPNPHY_NPWR_LGC_MAXLMT_40MHZ;
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			sslpnphy_specific->sslpnphy_NPwr_MaxLmt = SSLPNPHY_NPWR_MAXLMT_5G;
			sslpnphy_specific->sslpnphy_noise_measure_window = SSLPNPHY_NOISE_MEASURE_WINDOW_5G;
			sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt = SSLPNPHY_MAX_GAIN_CHANGE_LMT_5G;
			sslpnphy_specific->sslpnphy_max_rxpo_change_lmt = SSLPNPHY_MAX_RXPO_CHANGE_LMT_5G;
		}
	} else {
		sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt = SSLPNPHY_NPWR_LGC_MINLMT_20MHZ;
		sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt = SSLPNPHY_NPWR_LGC_MAXLMT_20MHZ;
	}
}

OSTATIC uint8 BCMOVERLAYDATA(0, NOISE_ARRAY) [][2] = {
	{1, 62 },
	{2, 65 },
	{3, 67 },
	{4, 68 },
	{5, 69 },
	{6, 70 },
	{7, 70 },
	{8, 71 },
	{9, 72 },
	{10, 72 },
	{11, 72 },
	{12, 73 },
	{13, 73 },
	{14, 74 },
	{15, 74 },
	{16, 74 },
	{17, 74 },
	{18, 75 },
	{19, 75 },
	{20, 75 },
	{21, 75 },
	{22, 75 },
	{23, 76 },
	{24, 76 },
	{25, 76 },
	{26, 76 },
	{27, 76 },
	{28, 77 },
	{29, 77 },
	{30, 77 },
	{31, 77 },
	{32, 77 },
	{33, 77 },
	{34, 77 },
	{35, 77 },
	{36, 78 },
	{37, 78 },
	{38, 78 },
	{39, 78 },
	{40, 78 },
	{41, 78 },
	{42, 78 },
	{43, 78 },
	{44, 78 },
	{45, 79 },
	{46, 79 },
	{47, 79 },
	{48, 79 },
	{49, 79 },
	{50, 79 }
};
uint8 BCMOVERLAYDATA(0, NOISE_ARRAY_sz) = ARRAYSIZE(NOISE_ARRAY);

OSTATIC uint8
BCMOVERLAYFN(0, wlc_sslpnphy_rx_noise_lut)(phy_info_t *pi, uint8 noise_val, uint8 ptr[][2], uint8 array_size)
{
	uint8 i = 1;
	uint8 rxpoWoListenGain = 0;
	for (i = 1; i < array_size; i++) {
		if (ptr[i][0] == noise_val) {
			rxpoWoListenGain = ptr[i][1];
		}
	}
	return rxpoWoListenGain;
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_reset_radioctrl_crsgain)(phy_info_t *pi)
{
	/* Reset radio ctrl and crs gain */
	or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
	write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_fifo_init)(phy_info_t *pi)
{
	uint8 i;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->sslpnphy_noisepwr_fifo_filled = 0;

	for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
		sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i] = 32767;
		sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[i] = 0;
	}
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_setup)(phy_info_t *pi)
{
	int16 temp;
	uint8 phybw40 = IS40MHZ(pi);
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (phybw40 == 0) {
		sslpnphy_specific->Listen_GaindB_BfrNoiseCal = (uint8)(read_phy_reg(pi, SSLPNPHY_HiGainDB) &
			SSLPNPHY_HiGainDB_HiGainDB_MASK) >> SSLPNPHY_HiGainDB_HiGainDB_SHIFT;
		sslpnphy_specific->NfSubtractVal_BfrNoiseCal = (read_phy_reg(pi, SSLPNPHY_nfSubtractVal) & 0x3ff);
	} else {
		sslpnphy_specific->Listen_GaindB_BfrNoiseCal =
			(uint8)((read_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40) &
			SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_MASK) >>
			SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_SHIFT);
		sslpnphy_specific->NfSubtractVal_BfrNoiseCal =
			(read_phy_reg(pi, SSLPNPHY_Rev2_nfSubtractVal_40) & 0x3ff);
	}

	sslpnphy_specific->Listen_GaindB_AfrNoiseCal = sslpnphy_specific->Listen_GaindB_BfrNoiseCal;

	if (sslpnphy_specific->sslpnphy_init_noise_cal_done == 0) {
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
			M_SSLPNPHY_NOISE_SAMPLES)), 128 << phybw40);
		if (phybw40 == 0) {
			sslpnphy_specific->Listen_GaindB_BASE = (uint8)(read_phy_reg(pi, SSLPNPHY_HiGainDB) &
				SSLPNPHY_HiGainDB_HiGainDB_MASK) >>
				SSLPNPHY_HiGainDB_HiGainDB_SHIFT;

			temp = (int16)(read_phy_reg(pi, SSLPNPHY_InputPowerDB)
				& SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK);

			sslpnphy_specific->NfSubtractVal_BASE = (read_phy_reg(pi, SSLPNPHY_nfSubtractVal) &
				0x3ff);
		} else {
			sslpnphy_specific->Listen_GaindB_BASE =
				(uint8)((read_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40) &
				SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_MASK) >>
				SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_SHIFT);

			temp = (int16)(read_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40)
				& SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK);

			sslpnphy_specific->NfSubtractVal_BASE =
				(read_phy_reg(pi, SSLPNPHY_Rev2_nfSubtractVal_40) & 0x3ff);
		}
		temp = temp << 8;
		temp = temp >> 8;

		sslpnphy_specific->RxpowerOffset_Required_BASE = (int8)temp;

		wlc_sslpnphy_detection_disable(pi, TRUE);
		wlc_sslpnphy_reset_radioctrl_crsgain(pi);

		wlc_sslpnphy_noise_fifo_init(pi);
	}

	sslpnphy_specific->rxpo_required_AfrNoiseCal = sslpnphy_specific->RxpowerOffset_Required_BASE;
}

OSTATIC uint32
BCMOVERLAYFN(0, wlc_sslpnphy_get_rxiq_accum)(phy_info_t *pi)
{
	uint32 IPwr, QPwr, IQ_Avg_Pwr = 0;
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);

	IPwr = ((uint32)read_phy_reg(pi, SSLPNPHY_IQIPWRAccHiAddress) << 16) |
		(uint32)read_phy_reg(pi, SSLPNPHY_IQIPWRAccLoAddress);
	QPwr = ((uint32)read_phy_reg(pi, SSLPNPHY_IQQPWRAccHiAddress) << 16) |
		(uint32)read_phy_reg(pi, SSLPNPHY_IQQPWRAccLoAddress);

	IQ_Avg_Pwr = (uint32)wlc_lpphy_qdiv_roundup((IPwr + QPwr),
		WL_READ_SHM(pi, (2 * (sslpnphy_shm_ptr +  M_SSLPNPHY_NOISE_SAMPLES))), 0);

	return IQ_Avg_Pwr;
}

OSTATIC uint32
BCMOVERLAYFN(0, wlc_sslpnphy_abs_time)(uint32 end, uint32 start)
{
	uint32 timediff;
	uint32 max32 = (uint32)((int)(0) - (int)(1));

	if (end >= start)
		timediff = end - start;
	else
		timediff = (1 + end) + (max32 - start);

	return timediff;
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_time_window)(phy_info_t *pi, uint32 window_time, uint32 *minpwr,
	uint32 *maxpwr, bool *measurement_valid)
{
	uint32 start_time;
	uint32 IQ_Avg_Pwr = 0;
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	*measurement_valid = FALSE;

	*minpwr = 32767;
	*maxpwr = 0;

	start_time = R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow);

	WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
		M_55f_REG_VAL)), 0);
	OR_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand, MCMD_BG_NOISE);

	while (wlc_sslpnphy_abs_time(R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow),
		start_time) < window_time) {

		if (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand) & MCMD_BG_NOISE) {
			OSL_DELAY(8);
		} else {
			IQ_Avg_Pwr = wlc_sslpnphy_get_rxiq_accum(pi);

			*minpwr = MIN(*minpwr, IQ_Avg_Pwr);
			*maxpwr = MAX(*maxpwr, IQ_Avg_Pwr);

			OSL_DELAY(6);
			WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
				M_55f_REG_VAL)), 0);
			OR_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand, MCMD_BG_NOISE);
		}
	}

	if ((*minpwr >= sslpnphy_specific->sslpnphy_NPwr_MinLmt) && (*minpwr <= sslpnphy_specific->sslpnphy_NPwr_MaxLmt))
		*measurement_valid = TRUE;
}

OSTATIC uint32
BCMOVERLAYFN(0, wlc_sslpnphy_noise_fifo_min)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 i;
	uint32 minpwr = 32767;

	for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
		WL_PHYCAL(("I is %d:MIN_FIFO = %d:MAX_FIFO = %d \n", i,
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i],
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[i]));
		minpwr = MIN(minpwr, sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i]);
	}

	return minpwr;
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_fifo_avg)(phy_info_t *pi, uint32 *avg_noise)
{
	uint8 i;
	uint8 max_min_Idx_1 = 0, max_min_Idx_2 = 0;
	uint32 Min_Min = 65535, Max_Max = 0, Max_Min_2 = 0,  Max_Min_1 = 0;
	uint32 Sum = 0;
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (sslpnphy_specific->sslpnphy_init_noise_cal_done) {
		*avg_noise = wlc_sslpnphy_noise_fifo_min(pi);
		return;
	}

	for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
		WL_PHYCAL(("I is %d:MIN_FIFO = %d:MAX_FIFO = %d \n", i,
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i],
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[i]));

		Min_Min = MIN(Min_Min, sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i]);
		Max_Min_1 = MAX(Max_Min_1, sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i]);
		Max_Max = MAX(Max_Max, sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[i]);
	}

	if (Max_Max >= ((Min_Min * 5) >> 1))
		*avg_noise = Min_Min;
	else {
		for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
			if (Max_Min_1 == sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i])
				max_min_Idx_1 = i;
		}

		for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
			if (i != max_min_Idx_1)
				Max_Min_2 = MAX(Max_Min_2, sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i]);
		}

		for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
			if ((Max_Min_2 == sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i]) &&
				(i != max_min_Idx_1))
				max_min_Idx_2 = i;
		}

		for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
			if ((i != max_min_Idx_1) && (i != max_min_Idx_2))
				Sum += sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i];
		}

		/* OutOf Six values of MinFifo,Two big values are eliminated
		and averaged out remaining four values of Min-FIFO	
		*/

		*avg_noise = wlc_lpphy_qdiv_roundup(Sum, 4, 0);

		WL_PHYCAL(("Sum = %d: Max_Min_1 = %d: Max_Min_2 = %d"
			" max_min_Idx_1 = %d: max_min_Idx_2 = %d\n", Sum,
			Max_Min_1, Max_Min_2, max_min_Idx_1, max_min_Idx_2));
	}
	WL_PHYCAL(("Avg_Min_NoisePwr = %d\n", *avg_noise));
}

typedef enum {
	INIT_FILL_FIFO = 0,
	CHK_LISTEN_GAIN_CHANGE,
	CHANGE_RXPO,
	PERIODIC_CAL
} sslpnphy_noise_measure_t;


void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_chg_listen_gain)(phy_info_t *pi, int8 change_sign)
{
	uint8 phybw40 = IS40MHZ(pi);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	sslpnphy_specific->Listen_GaindB_AfrNoiseCal = sslpnphy_specific->Listen_GaindB_AfrNoiseCal + (3 * change_sign);

	if (sslpnphy_specific->Listen_GaindB_AfrNoiseCal <= (sslpnphy_specific->Listen_GaindB_BASE -
		sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt))
		sslpnphy_specific->Listen_GaindB_AfrNoiseCal = sslpnphy_specific->Listen_GaindB_BASE -
			sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt;
	else if (sslpnphy_specific->Listen_GaindB_AfrNoiseCal >= (sslpnphy_specific->Listen_GaindB_BASE +
		sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt))
		sslpnphy_specific->Listen_GaindB_AfrNoiseCal = sslpnphy_specific->Listen_GaindB_BASE +
			sslpnphy_specific->sslpnphy_max_listen_gain_change_lmt;

	if (phybw40 == 0)
		mod_phy_reg(pi, SSLPNPHY_HiGainDB,
			SSLPNPHY_HiGainDB_HiGainDB_MASK,
			((sslpnphy_specific->Listen_GaindB_AfrNoiseCal) <<
			SSLPNPHY_HiGainDB_HiGainDB_SHIFT));
	else
		mod_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40,
			SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_MASK,
			((sslpnphy_specific->Listen_GaindB_AfrNoiseCal) <<
			SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_SHIFT));

	wlc_sslpnphy_reset_radioctrl_crsgain(pi);
	OSL_DELAY(10);
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_change_rxpo)(phy_info_t *pi, uint32 avg_noise)
{
	uint8 rxpo_Wo_Listengain;
	uint8 phybw40 = IS40MHZ(pi);
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (!sslpnphy_specific->sslpnphy_init_noise_cal_done)
		sslpnphy_specific->Listen_RF_Gain = (read_phy_reg(pi, SSLPNPHY_crsGainRespVal) & 0xff);
	else
		sslpnphy_specific->Listen_RF_Gain = (WL_READ_SHM(pi,
			(2 * (sslpnphy_shm_ptr + M_55f_REG_VAL))) & 0xff);

	if ((sslpnphy_specific->Listen_RF_Gain > 45) && (sslpnphy_specific->Listen_RF_Gain < 85) &&
		(ABS(sslpnphy_specific->Listen_GaindB_AfrNoiseCal - sslpnphy_specific->Listen_RF_Gain) < 12)) {

		rxpo_Wo_Listengain = wlc_sslpnphy_rx_noise_lut(pi, avg_noise, NOISE_ARRAY,
			NOISE_ARRAY_sz);

		sslpnphy_specific->rxpo_required_AfrNoiseCal = (int8)(sslpnphy_specific->Listen_RF_Gain - rxpo_Wo_Listengain);

		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) {
			if (phybw40 == 0)
				sslpnphy_specific->rxpo_required_AfrNoiseCal = sslpnphy_specific->rxpo_required_AfrNoiseCal - 1;
			else
				sslpnphy_specific->rxpo_required_AfrNoiseCal = sslpnphy_specific->rxpo_required_AfrNoiseCal + 3;
		}

		if (sslpnphy_specific->rxpo_required_AfrNoiseCal <= (-1 * sslpnphy_specific->sslpnphy_max_rxpo_change_lmt))
			sslpnphy_specific->rxpo_required_AfrNoiseCal = -1 * sslpnphy_specific->sslpnphy_max_rxpo_change_lmt;
		else if (sslpnphy_specific->rxpo_required_AfrNoiseCal >= sslpnphy_specific->sslpnphy_max_rxpo_change_lmt)
			sslpnphy_specific->rxpo_required_AfrNoiseCal = sslpnphy_specific->sslpnphy_max_rxpo_change_lmt;

		if (phybw40 == 0)
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				(sslpnphy_specific->rxpo_required_AfrNoiseCal <<
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT));
		else
			mod_phy_reg(pi, SSLPNPHY_Rev2_InputPowerDB_40,
				SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_MASK,
				sslpnphy_specific->rxpo_required_AfrNoiseCal <<
				SSLPNPHY_Rev2_InputPowerDB_40_inputpwroffsetdb_SHIFT);
	}
}

OSTATIC void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_computeNf)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	int8 Delta_Listen_GaindB_Change, Delta_NfSubtractVal_Change;
	uint8 phybw40 = IS40MHZ(pi);

	if (phybw40 == 0)
		sslpnphy_specific->Listen_GaindB_AfrNoiseCal = (uint8)(read_phy_reg(pi, SSLPNPHY_HiGainDB)
			& SSLPNPHY_HiGainDB_HiGainDB_MASK) >>
			SSLPNPHY_HiGainDB_HiGainDB_SHIFT;
	else
		sslpnphy_specific->Listen_GaindB_AfrNoiseCal = (uint8)((read_phy_reg(pi, SSLPNPHY_Rev2_HiGainDB_40)
			& SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_MASK) >>
			SSLPNPHY_Rev2_HiGainDB_40_HiGainDB_SHIFT);

	Delta_Listen_GaindB_Change = (int8)(sslpnphy_specific->Listen_GaindB_AfrNoiseCal -
		sslpnphy_specific->Listen_GaindB_BfrNoiseCal);

	if ((CHSPEC_IS2G(pi->radio_chanspec)) && ((Delta_Listen_GaindB_Change > 3) ||
		(Delta_Listen_GaindB_Change < -3)))
		Delta_NfSubtractVal_Change = 4 * Delta_Listen_GaindB_Change;
	else
		Delta_NfSubtractVal_Change = 0;

	sslpnphy_specific->NfSubtractVal_AfrNoiseCal = (sslpnphy_specific->NfSubtractVal_BfrNoiseCal +
		Delta_NfSubtractVal_Change) & 0x3ff;

	if (phybw40 == 0)
		write_phy_reg(pi, SSLPNPHY_nfSubtractVal, sslpnphy_specific->NfSubtractVal_AfrNoiseCal);
	else
		write_phy_reg(pi, SSLPNPHY_Rev2_nfSubtractVal_40,
			sslpnphy_specific->NfSubtractVal_AfrNoiseCal);
}

void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure)(phy_info_t *ppi)
{

	uint32 start_time = 0, timeout = 0;
	phy_info_t *pi = (phy_info_t *)ppi;

	uint8 noise_measure_state, i;
	uint32 min_anp, max_anp, avg_noise = 0;
	bool measurement_valid;
	bool measurement_done = FALSE;
	uint32 IQ_Avg_Pwr = 0;
	uint8 phybw40 = IS40MHZ(pi);
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	WL_TRACE(("wl%d: %s: begin\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	if (NORADIO_ENAB(pi->pubpi))
		return;

	sslpnphy_specific->sslpnphy_last_noise_cal = GENERIC_PHY_INFO(pi)->now;

	mod_phy_reg(pi, SSLPNPHY_sslpnCalibClkEnCtrl,
		SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_MASK,
		1 << SSLPNPHY_sslpnCalibClkEnCtrl_iqEstClkEn_SHIFT);

	if (sslpnphy_specific->sslpnphy_init_noise_cal_done == 0) {
		noise_measure_state = INIT_FILL_FIFO;
		timeout = SSLPNPHY_INIT_NOISE_CAL_TMOUT;
	} else {
		noise_measure_state = PERIODIC_CAL;
		timeout = 0; /* a very small value for just one iteration */
	}

	wlc_sslpnphy_noise_measure_setup(pi);

	start_time = R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow);

	do {
		switch (noise_measure_state) {

		case INIT_FILL_FIFO:
			wlc_sslpnphy_noise_measure_time_window(pi,
				sslpnphy_specific->sslpnphy_noise_measure_window, &min_anp, &max_anp,
				&measurement_valid);

			if (measurement_valid) {
				sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[sslpnphy_specific->sslpnphy_noisepwr_fifo_filled]
					= min_anp;
				sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[sslpnphy_specific->sslpnphy_noisepwr_fifo_filled]
					= max_anp;

				sslpnphy_specific->sslpnphy_noisepwr_fifo_filled++;
			}

			if (sslpnphy_specific->sslpnphy_noisepwr_fifo_filled == SSLPNPHY_NOISE_PWR_FIFO_DEPTH) {
				noise_measure_state = CHK_LISTEN_GAIN_CHANGE;
				sslpnphy_specific->sslpnphy_noisepwr_fifo_filled = 0;
			}

			break;
		case PERIODIC_CAL:
			if (!(R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand) & MCMD_BG_NOISE)) {
				IQ_Avg_Pwr = wlc_sslpnphy_get_rxiq_accum(pi);
				if ((IQ_Avg_Pwr >= sslpnphy_specific->sslpnphy_NPwr_MinLmt) &&
					(IQ_Avg_Pwr <= sslpnphy_specific->sslpnphy_NPwr_MaxLmt)) {
				sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[sslpnphy_specific->sslpnphy_noisepwr_fifo_filled] =
					IQ_Avg_Pwr;
				sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[sslpnphy_specific->sslpnphy_noisepwr_fifo_filled] =
					IQ_Avg_Pwr;

				sslpnphy_specific->sslpnphy_noisepwr_fifo_filled++;
				if (sslpnphy_specific->sslpnphy_noisepwr_fifo_filled ==
					SSLPNPHY_NOISE_PWR_FIFO_DEPTH)
					sslpnphy_specific->sslpnphy_noisepwr_fifo_filled = 0;
				}
			}
			break;

		case CHK_LISTEN_GAIN_CHANGE:
			wlc_sslpnphy_noise_fifo_avg(pi, &avg_noise);


			if ((avg_noise < sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt) &&
				(avg_noise >= sslpnphy_specific->sslpnphy_NPwr_MinLmt)) {

				wlc_sslpnphy_noise_measure_chg_listen_gain(pi, +1);

				wlc_sslpnphy_noise_fifo_init(pi);
				noise_measure_state = INIT_FILL_FIFO;

			} else if ((avg_noise > sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt) &&
				(avg_noise <= sslpnphy_specific->sslpnphy_NPwr_MaxLmt)) {

				wlc_sslpnphy_noise_measure_chg_listen_gain(pi, -1);

				wlc_sslpnphy_noise_fifo_init(pi);
				noise_measure_state = INIT_FILL_FIFO;

			} else if ((avg_noise >= sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt) &&
				(avg_noise <= sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt)) {

				noise_measure_state = CHANGE_RXPO;
			}

			break;

		case CHANGE_RXPO:
			wlc_sslpnphy_noise_measure_change_rxpo(pi, avg_noise);
			measurement_done = TRUE;
			break;

		default:
			break;

		}
	} while ((wlc_sslpnphy_abs_time(R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow), start_time) <=
		timeout) && (!measurement_done));

	sslpnphy_specific->Listen_RF_Gain = (WL_READ_SHM(pi,
		(2 * (sslpnphy_shm_ptr + M_55f_REG_VAL))) & 0xff);

	if (!measurement_done) {

		if (!sslpnphy_specific->sslpnphy_init_noise_cal_done && (sslpnphy_specific->sslpnphy_noisepwr_fifo_filled == 0) &&
			(noise_measure_state == 0)) {
			WL_PHYCAL(("Init Noise Cal Timedout After T %d uS And Noise_Cmd = %d:\n",
				wlc_sslpnphy_abs_time(R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow),
				start_time), (R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand) &
				MCMD_BG_NOISE)));
		} else {

			avg_noise = wlc_sslpnphy_noise_fifo_min(pi);

			if ((avg_noise < sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt) &&
				(avg_noise >= sslpnphy_specific->sslpnphy_NPwr_MinLmt)) {

				wlc_sslpnphy_noise_measure_chg_listen_gain(pi, +1);

				wlc_sslpnphy_noise_fifo_init(pi);

			} else if ((avg_noise > sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt) &&
				(avg_noise <= sslpnphy_specific->sslpnphy_NPwr_MaxLmt)) {

				wlc_sslpnphy_noise_measure_chg_listen_gain(pi, -1);

				wlc_sslpnphy_noise_fifo_init(pi);

			} else if ((avg_noise >= sslpnphy_specific->sslpnphy_NPwr_LGC_MinLmt) &&
				(avg_noise <= sslpnphy_specific->sslpnphy_NPwr_LGC_MaxLmt)) {

				wlc_sslpnphy_noise_measure_change_rxpo(pi, avg_noise);
			}
		}
	}


	wlc_sslpnphy_noise_measure_computeNf(pi);

	WL_PHYCAL(("Phy Bw40:%d Noise Cal Stats After T %d uS:Npercal = %d"
		" FifoFil = %d Npwr = %d Rxpo = %d Gain_Set = %d"
		" Delta_Gain_Change = %d\n", IS40MHZ(pi),
		wlc_sslpnphy_abs_time(R_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->tsf_timerlow), start_time),
		sslpnphy_specific->sslpnphy_init_noise_cal_done, sslpnphy_specific->sslpnphy_noisepwr_fifo_filled, avg_noise,
		pi->rxpo_required_AfrNoiseCal, pi->Listen_GaindB_AfrNoiseCal,
		(int8)(pi->Listen_GaindB_AfrNoiseCal - pi->Listen_GaindB_BfrNoiseCal)));

	WL_PHYCAL(("Get_RF_Gain = %d NfVal_Set = %d Base_Gain = %d Base_Rxpo = %d"
		" Base_NfVal = %d Noise_Measure_State = %d\n", pi->Listen_RF_Gain,
		pi->NfSubtractVal_AfrNoiseCal, pi->Listen_GaindB_BASE,
		pi->RxpowerOffset_Required_BASE, pi->NfSubtractVal_BASE, noise_measure_state));

	for (i = 0; i < SSLPNPHY_NOISE_PWR_FIFO_DEPTH; i++) {
		WL_PHYCAL(("I is %d:MIN_FIFO = %d:MAX_FIFO = %d \n", i,
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Min[i],
			sslpnphy_specific->sslpnphy_noisepwr_fifo_Max[i]));
	}

	if (!sslpnphy_specific->sslpnphy_init_noise_cal_done) {
		wlc_sslpnphy_detection_disable(pi, FALSE);

		sslpnphy_specific->sslpnphy_init_noise_cal_done = TRUE;
	}

	if ((sslpnphy_specific->sslpnphy_init_noise_cal_done == 1) && !sslpnphy_specific->sslpnphy_disable_noise_percal) {
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
			M_SSLPNPHY_NOISE_SAMPLES)), 80 << phybw40);

		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
			M_55f_REG_VAL)), 0);
		OR_REG(GENERIC_PHY_INFO(pi)->osh, &pi->regs->maccommand, MCMD_BG_NOISE);
	}
}

void wlc_sslpnphy_auxadc_measure(wlc_phy_t *ppi, bool readVal)
{
	uint16 tssi_val;
	phy_info_t *pi = (phy_info_t *)ppi;
	uint16 sslpnphy_shm_ptr = WL_READ_SHM(pi, M_SSLPNPHYREGS_PTR);
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	if (0 == WL_READ_SHM(pi, (2 * (sslpnphy_shm_ptr +
		M_SSLPNPHY_TSSICAL_EN)))) {

		if (readVal) {
			tssi_val = read_phy_reg(pi, SSLPNPHY_rssiaccValResult0)	+ (read_phy_reg(pi, SSLPNPHY_rssiaccValResult1) <<16);
			tssi_val = tssi_val >> 6;
			if (tssi_val > 31) {
				tssi_val = 31;
			}
			sslpnphy_specific->sslpnphy_auxadc_val = tssi_val+32;
			/* Write measured idle TSSI value */
			mod_phy_reg(pi, SSLPNPHY_TxPwrCtrlIdleTssi,
				SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_MASK,
				tssi_val << SSLPNPHY_TxPwrCtrlIdleTssi_idleTssi0_SHIFT);

		}

		/* write the regsisters */
		write_phy_reg(pi, SSLPNPHY_NumrssiSamples, 0x40);
		write_phy_reg(pi, SSLPNPHY_rssiwaittime, 0x50);

		/* trigger the ucode again */
		WL_WRITE_SHM(pi, (2 * (sslpnphy_shm_ptr +
			M_SSLPNPHY_TSSICAL_EN)), 0x1);
		sslpnphy_specific->sslpnphy_last_idletssi_cal = GENERIC_PHY_INFO(pi)->now;
	}


}

/* don't use this directly. use wlc_get_band_range whenever possible */
int
wlc_get_ssn_lp_band_range(uint freq)
{
	int range = -1;

	if (freq < 2500)
		range = WL_CHAN_FREQ_RANGE_2G;
	else if (freq <= 5320)
		range = WL_CHAN_FREQ_RANGE_5GL;
	else if (freq <= 5700)
		range = WL_CHAN_FREQ_RANGE_5GM;
	else
		range = WL_CHAN_FREQ_RANGE_5GH;

	return range;
}




static void
WLBANDINITFN(wlc_radio_2063_init_sslpnphy)(phy_info_t *pi)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 phybw40 = IS40MHZ(pi);

	WL_INFORM(("wl%d: %s\n", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));

	/* Load registers from the table */
	wlc_sslpnphy_init_radio_regs(pi, sslpnphy_radio_regs_2063, RADIO_DEFAULT_CORE);

	/* Set some PLL registers overridden by DC/CLB */
	write_radio_reg(pi, RADIO_2063_LOGEN_SP_5, 0x0);

	or_radio_reg(pi, RADIO_2063_COMMON_08, (0x07 << 3));
	write_radio_reg(pi, RADIO_2063_BANDGAP_CTRL_1, 0x56);

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, 0x1 << 1, 0);
	} else {
		if (phybw40 == 0) {
			/* Set rx lpf bw to 9MHz */
			mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, 0x1 << 1, 0);
		} else if (phybw40 == 1) {
			/* Set rx lpf bw to 19MHz for 40Mhz operation */
			mod_radio_reg(pi, RADIO_2063_RXBB_CTRL_2, 3 << 1, 1 << 1);
			or_radio_reg(pi, RADIO_2063_COMMON_02, (0x1 << 1));
			mod_radio_reg(pi, RADIO_2063_RXBB_SP_4, 7 << 4, 0x30);
			and_radio_reg(pi, RADIO_2063_COMMON_02, (0x0 << 1));
		}
	}

	/*
	 * Apply rf reg settings to mitigate 2063 spectrum
	 * asymmetry problems, including setting
	 * PA and PAD in class A mode
	 */
	write_radio_reg(pi, RADIO_2063_PA_SP_7, 0);
	/* pga/pad */
	write_radio_reg(pi, RADIO_2063_TXRF_SP_6, 0x20);

	write_radio_reg(pi, RADIO_2063_TXRF_SP_9, 0x40);
	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		/*  pa cascode voltage */
		write_radio_reg(pi, RADIO_2063_COMMON_05, 0x82);
		write_radio_reg(pi, RADIO_2063_TXRF_SP_6, 0x50);
		write_radio_reg(pi, RADIO_2063_TXRF_SP_9, 0x80);
	}

	/*  PA, PAD class B settings */
	write_radio_reg(pi, RADIO_2063_PA_SP_3, 0x15);

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
		write_radio_reg(pi, RADIO_2063_PA_SP_4, 0x09);
		write_radio_reg(pi, RADIO_2063_PA_SP_2, 0x21);

		if ((sslpnphy_specific->sslpnphy_fabid == 2) ||
			(sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
			write_radio_reg(pi, RADIO_2063_PA_SP_3, 0x30);
			write_radio_reg(pi, RADIO_2063_PA_SP_2, 0x60);
			write_radio_reg(pi, RADIO_2063_PA_SP_4, 0x1);
			write_radio_reg(pi, RADIO_2063_TXBB_CTRL_1, 0x00);
			write_radio_reg(pi, RADIO_2063_TXRF_SP_6, 0x00);
			write_radio_reg(pi, RADIO_2063_TXRF_SP_9, 0xF0);
			sslpnphy_specific->sslpnphy_radio_classA = TRUE;
		}
	} else /* if (SSLPNREV_GE(pi->pubpi.phy_rev, 2)) */{
		write_radio_reg(pi, RADIO_2063_PA_SP_4, 0x09);
		write_radio_reg(pi, RADIO_2063_PA_SP_2, 0x21);
		write_radio_reg(pi, RADIO_2063_TXRF_SP_15, 0xc8);
		if (phybw40 == 1) {
			/*  PA, PAD class B settings */
			write_radio_reg(pi, RADIO_2063_TXBB_CTRL_1, 0x10);
			write_radio_reg(pi, RADIO_2063_TXRF_SP_6, 0xF0);
			write_radio_reg(pi, RADIO_2063_TXRF_SP_9, 0xF0);
			write_radio_reg(pi, RADIO_2063_PA_SP_3, 0x10);
			write_radio_reg(pi, RADIO_2063_PA_SP_4, 0x1);
			write_radio_reg(pi, RADIO_2063_PA_SP_2, 0x30);
		}
	}

}

typedef struct {
	uint16 fref_khz;
	uint8 c1;
	uint8 c2;
	uint8 c3;
	uint8 c4;
	uint8 r1;
	uint8 r2;
} loop_filter_2062_t;

static const
loop_filter_2062_t WLBANDINITDATA(loop_filter_2062)[] = {
	{12000, 6, 6, 6, 6, 10, 6 },
	{13000, 4, 4, 4, 4, 11, 7 },
	{14400, 3, 3, 3, 3, 12, 7 },
	{16200, 3, 3, 3, 3, 13, 8 },
	{18000, 2, 2, 2, 2, 14, 8 },
	{19200, 1, 1, 1, 1, 14, 9 }
};



STATIC uint32
wlc_lpphy_qdiv_roundup(uint32 divident, uint32 divisor, uint8 precision)
{
	uint32 quotient, remainder, roundup, rbit;

	ASSERT(divisor);

	quotient = divident / divisor;
	remainder = divident % divisor;
	rbit = divisor & 1;
	roundup = (divisor >> 1) + rbit;

	while (precision--) {
		quotient <<= 1;
		if (remainder >= roundup) {
			quotient++;
			remainder = ((remainder - roundup) << 1) + rbit;
		} else {
			remainder <<= 1;
		}
	}

	/* Final rounding */
	if (remainder >= roundup)
		quotient++;

	return quotient;
}

void
wlc_2063_vco_cal(phy_info_t *pi)
{
	uint8 calnrst;

	/* Power up VCO cal clock */
	mod_radio_reg(pi, RADIO_2063_PLL_SP_1, 1 << 6, 0);

	calnrst = read_radio_reg(pi, RADIO_2063_PLL_JTAG_CALNRST) & 0xf8;
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_CALNRST, calnrst);
	OSL_DELAY(1);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_CALNRST, calnrst | 0x04);
	OSL_DELAY(1);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_CALNRST, calnrst | 0x06);
	OSL_DELAY(1);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_CALNRST, calnrst | 0x07);
	OSL_DELAY(300);

	/* Power down VCO cal clock */
	mod_radio_reg(pi, RADIO_2063_PLL_SP_1, 1 << 6, 1 << 6);
}
#ifdef BAND5G
static void
aband_tune_radio_reg(phy_info_t *pi, uint16 address, uint8 val, uint valid)
{
	if (valid) {
		write_radio_reg(pi, address, val);
	} else {
		return;
	}
}
static void
wlc_sslpnphy_radio_2063_channel_tweaks_A_band(phy_info_t *pi, uint freq)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint8 i;
	const chan_info_2063_sslpnphy_aband_tweaks_t *ci;
	const chan_info_2063_sslpnphy_X17_aband_tweaks_t * ci_x17;
	const chan_info_2063_sslpnphy_ninja_aband_tweaks_t * ci_ninja;

	write_radio_reg(pi, RADIO_2063_PA_SP_6, 0x7f);
	write_radio_reg(pi, RADIO_2063_TXRF_SP_17, 0xff);
	write_radio_reg(pi, RADIO_2063_TXRF_SP_13, 0xff);
	write_radio_reg(pi, RADIO_2063_TXRF_SP_5, 0xff);
	write_radio_reg(pi, RADIO_2063_PA_CTRL_5, 0x50);
	wlc_sslpnphy_set_pa_gain(pi, 116);
	if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)
		wlc_sslpnphy_set_pa_gain(pi, 0x10);

	for (i = 0; i < ARRAYSIZE(chan_info_2063_sslpnphy_aband_tweaks); i++) {
		if (freq <= chan_info_2063_sslpnphy_aband_tweaks[i].freq)
			break;
	}
	ci = &chan_info_2063_sslpnphy_aband_tweaks[i];
	ci_x17 = &chan_info_2063_sslpnphy_X17_aband_tweaks[i];
	ci_ninja = &chan_info_2063_sslpnphy_ninja_aband_tweaks[i];

	i = 12;
	aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_11,
		ci->RF_PA_CTRL_11, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_8,
		ci->RF_TXRF_CTRL_8, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_5,
		ci->RF_TXRF_CTRL_5, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_2,
		ci->RF_TXRF_CTRL_2, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_4,
		ci->RF_TXRF_CTRL_4, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_7,
		ci->RF_TXRF_CTRL_7, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_6,
		ci->RF_TXRF_CTRL_6, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_2,
		ci->RF_PA_CTRL_2, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_5,
		ci->RF_PA_CTRL_5, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_15,
		ci->RF_TXRF_CTRL_15, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_14,
		ci->RF_TXRF_CTRL_14, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_7,
		ci->RF_PA_CTRL_7, (ci->valid_tweak & (0x1 << i--)));
	aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_15,
		ci->RF_PA_CTRL_15, (ci->valid_tweak & (0x1 << i)));

	write_radio_reg(pi, RADIO_2063_PA_CTRL_5, 0x40);

	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17M_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID) ||
		(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329MOTOROLA_SSID)) {
		i = 7;
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_5,
			ci_x17->MRT_RF_PA_CTRL_5, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_8,
			ci_x17->MRT_RF_TXRF_CTRL_8, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_5,
			ci_x17->MRT_RF_TXRF_CTRL_5, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_2,
			ci_x17->MRT_RF_TXRF_CTRL_2, (ci_x17->valid_tweak & (0x1 << i)));
	}
	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) ||
		(CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID)) { /* 4319 5G */
		i = 3;
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_5,
			ci_x17->BCM_RF_PA_CTRL_5, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_8,
			ci_x17->BCM_RF_TXRF_CTRL_8, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_5,
			ci_x17->BCM_RF_TXRF_CTRL_5, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_2,
			ci_x17->BCM_RF_TXRF_CTRL_2, (ci_x17->valid_tweak & (0x1 << i)));
	}
	if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA) {
		i = 11;
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_11,
			ci_x17->ePA_RF_PA_CTRL_11, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_8,
			ci_x17->ePA_RF_TXRF_CTRL_8, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_5,
			ci_x17->ePA_RF_TXRF_CTRL_5, (ci_x17->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_TXRF_CTRL_2,
			ci_x17->ePA_RF_TXRF_CTRL_2, (ci_x17->valid_tweak & (0x1 << i--)));

	}
	write_radio_reg(pi, RADIO_2063_PA_CTRL_7, 0x2);
	if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) {
		if (freq == 5600)
			write_radio_reg(pi, RADIO_2063_PA_CTRL_7, 0x10);
	}

	write_radio_reg(pi, RADIO_2063_PA_CTRL_2, 0x90);
	write_radio_reg(pi, RADIO_2063_PA_CTRL_7, 0x0);
	if (freq == 5680)
		write_radio_reg(pi, RADIO_2063_TXRF_CTRL_1, 0xa1);
	write_radio_reg(pi, RADIO_2063_TXBB_CTRL_1, 0x10);
	if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA) {
		write_radio_reg(pi, RADIO_2063_PA_CTRL_7, 0x20);
		write_radio_reg(pi, RADIO_2063_PA_CTRL_2, 0x20);
	}

	sslpnphy_specific->sslpnphy_radio_classA = TRUE;
	if (CHIPID(GENERIC_PHY_INFO(pi)->sih->chip) == BCM4319_CHIP_ID) {
		/* 4319 5G iPA tuning */
		i = 4;
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_2,
		    ci_ninja->RF_PA_CTRL_2, (ci_ninja->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_5,
		    ci_ninja->RF_PA_CTRL_5, (ci_ninja->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_7,
		    ci_ninja->RF_PA_CTRL_7, (ci_ninja->valid_tweak & (0x1 << i--)));
		aband_tune_radio_reg(pi, RADIO_2063_PA_CTRL_11,
		    ci_ninja->RF_PA_CTRL_11, (ci_ninja->valid_tweak & (0x1 << i--)));

		/* 4319 5G RX LNA tuning */
		if (freq <= 5300) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xF);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0xF);
		} else if (freq < 5500) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0xc);
		} else if (freq < 5520) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x9);
		} else if (freq < 5560) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x8);
		} else if (freq < 5580) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x7);
		} else if (freq < 5600) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x6);
		} else if (freq < 5620) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x3);
		} else if (freq < 5640) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x2);
		} else if (freq < 5680) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xf);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x1);
		} else if (freq < 5700) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xd);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else if (freq < 5745) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0xc);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else if (freq < 5765) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0x7);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else if (freq < 5785) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0x5);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else if (freq < 5805) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0x4);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else if (freq < 5825) {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0x2);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		} else {
			write_radio_reg(pi, RADIO_2063_ARX_1ST_3, 0x1);
			write_radio_reg(pi, RADIO_2063_ARX_2ND_1, 0x0);
		}
	}
}
OSTATIC void
wlc_sslpnphy_pll_aband_tune(phy_info_t *pi, uint8 channel)
{
	uint8 i;
	uint freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
	const chan_info_2063_sslpnphy_X17_epa_tweaks_t * ci_x17;
	for (i = 0; i < ARRAYSIZE(chan_info_2063_sslpnphy_X17_epa_tweaks); i++) {
		if (freq <= chan_info_2063_sslpnphy_X17_epa_tweaks[i].freq)
			break;
	}
	if (i >= ARRAYSIZE(chan_info_2063_sslpnphy_X17_epa_tweaks)) {
		WL_ERROR(("wl%d: %s: freq %d not found in channel table\n",
			GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, freq));
		return;
	}
	ci_x17 = &chan_info_2063_sslpnphy_X17_epa_tweaks[i];

	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_2, ci_x17->ePA_JTAG_PLL_CP_2);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_3, ci_x17->ePA_JTAG_PLL_CP_3);
}
#endif /* BAND5G */
static void
wlc_sslpnphy_radio_2063_channel_tune(phy_info_t *pi, uint8 channel)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#endif /* PHYHAL */
	uint i;
	const chan_info_2063_sslpnphy_t *ci;
	uint8 rfpll_doubler = 1;
	uint16 rf_common15;
	fixed qFxtal, qFref, qFvco, qFcal, qVco, qVal;
	uint8  to, refTo, cp_current, kpd_scale, ioff_scale, offset_current;
	uint32 setCount, div_int, div_frac, iVal, fvco3, fref, fref3, fcal_div;
	uint16 loop_bw = 0;
	uint16 d30 = 0;
	uint16 temp_pll, temp_pll_1;
	uint16  h29, h30;
	bool e44, e45;

	ci = &chan_info_2063_sslpnphy[0];

	if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329AGBF_SSID) &&
	    CHSPEC_IS5G(pi->radio_chanspec) && !(GENERIC_PHY_INFO(pi)->boardflags &
	    BFL_HGPA)) {
	    rfpll_doubler  = 0;
	}

	if (rfpll_doubler)
	    si_pmu_chipcontrol(GENERIC_PHY_INFO(pi)->sih, PMU1_PLL0_CHIPCTL0, 0x20000, 0x00000);
	else
	    si_pmu_chipcontrol(GENERIC_PHY_INFO(pi)->sih, PMU1_PLL0_CHIPCTL0, 0x20000, 0x20000);

	/* lookup radio-chip-specific channel code */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		for (i = 0; i < ARRAYSIZE(chan_info_2063_sslpnphy); i++)
			if (chan_info_2063_sslpnphy[i].chan == channel)
				break;

		if (i >= ARRAYSIZE(chan_info_2063_sslpnphy)) {
			WL_ERROR(("wl%d: %s: channel %d not found in channel table\n",
				GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, channel));
			return;
		}
		ci = &chan_info_2063_sslpnphy[i];
#ifndef BAND5G
	}
#else
	} else {
		for (i = 0; i < ARRAYSIZE(chan_info_2063_sslpnphy_aband); i++)
			if (chan_info_2063_sslpnphy_aband[i].chan == channel)
				break;
		if (i >= ARRAYSIZE(chan_info_2063_sslpnphy_aband)) {
			WL_ERROR(("wl%d: %s: channel %d not found in channel table\n",
				GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, channel));
			return;
		}
		ci = &chan_info_2063_sslpnphy_aband[i];
	}
#endif
	/* Radio tunables */
	write_radio_reg(pi, RADIO_2063_LOGEN_VCOBUF_1, ci->RF_logen_vcobuf_1);
	write_radio_reg(pi, RADIO_2063_LOGEN_MIXER_2, ci->RF_logen_mixer_2);
	write_radio_reg(pi, RADIO_2063_LOGEN_BUF_2, ci->RF_logen_buf_2);
	write_radio_reg(pi, RADIO_2063_LOGEN_RCCR_1, ci->RF_logen_rccr_1);
	write_radio_reg(pi, RADIO_2063_GRX_1ST_3, ci->RF_grx_1st_3);
	if ((sslpnphy_specific->sslpnphy_fabid == 2) || (sslpnphy_specific->sslpnphy_fabid_otp == TSMC_FAB12)) {
		if (channel == 4)
			write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x0b);
		else if (channel == 5)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x0b);
		else if (channel == 6)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x0a);
		else if (channel == 7)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x09);
		else if (channel == 8)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x09);
		else if (channel == 9)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x08);
		else if (channel == 10)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x05);
		else if (channel == 11)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x04);
		else if (channel == 12)
		        write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x03);
		else if (channel == 13)
			write_radio_reg(pi, RADIO_2063_GRX_1ST_3, 0x02);
	}
	write_radio_reg(pi, RADIO_2063_GRX_2ND_2, ci->RF_grx_2nd_2);
	write_radio_reg(pi, RADIO_2063_ARX_1ST_3, ci->RF_arx_1st_3);
	write_radio_reg(pi, RADIO_2063_ARX_2ND_1, ci->RF_arx_2nd_1);
	write_radio_reg(pi, RADIO_2063_ARX_2ND_4, ci->RF_arx_2nd_4);
	write_radio_reg(pi, RADIO_2063_ARX_2ND_7, ci->RF_arx_2nd_7);
	write_radio_reg(pi, RADIO_2063_ARX_PS_6, ci->RF_arx_ps_6);
	write_radio_reg(pi, RADIO_2063_TXRF_CTRL_2, ci->RF_txrf_ctrl_2);
	write_radio_reg(pi, RADIO_2063_TXRF_CTRL_5, ci->RF_txrf_ctrl_5);
	write_radio_reg(pi, RADIO_2063_PA_CTRL_11, ci->RF_pa_ctrl_11);
	write_radio_reg(pi, RADIO_2063_ARX_MIX_4, ci->RF_arx_mix_4);

	/* write_radio_reg(pi, RADIO_2063_LOGEN_SPARE_2, ci->dummy4); */
	/* Turn on PLL power supplies */
	rf_common15 = read_radio_reg(pi, RADIO_2063_COMMON_15);
	write_radio_reg(pi, RADIO_2063_COMMON_15, rf_common15 | (0x0f << 1));

	/* Calculate various input frequencies */
	fref = rfpll_doubler ? XTALFREQ(pi->xtalfreq) : (XTALFREQ(pi->xtalfreq) << 1);
	if (rfpll_doubler == 0) {
	    e44 = 1;
	} else {
	    if (XTALFREQ(pi->xtalfreq) > 26000000)
	        e44 = 1;
	    else
	        e44 = 0;
	}

	if (e44 == 0) {
	    e45 = 0;
	} else {
	    if (fref > 52000000)
	        e45 = 1;
	    else
	        e45 = 0;
	}

	if (e44 == 0) {
	    fcal_div = 1;
	} else {
	    if (e45 == 0)
	        fcal_div = 2;
	    else
	        fcal_div = 4;
	}

	if (ci->freq > 2484)
		fvco3 = (ci->freq << 1);
	else
		fvco3 = (ci->freq << 2);
	fref3 = 3 * fref;

	/* Convert into Q16 MHz */
	qFxtal = wlc_lpphy_qdiv_roundup(XTALFREQ(pi->xtalfreq), PLL_2063_MHZ, 16);
	qFref =  wlc_lpphy_qdiv_roundup(fref, PLL_2063_MHZ, 16);
	qFcal = wlc_lpphy_qdiv_roundup(fref, fcal_div * PLL_2063_MHZ, 16);
	qFvco = wlc_lpphy_qdiv_roundup(fvco3, 3, 16);

	/* PLL_delayBeforeOpenLoop */
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_3, 0x02);

	/* PLL_enableTimeout */
	to = (uint8)((((fref * PLL_2063_CAL_REF_TO) /
		(PLL_2063_OPEN_LOOP_DELAY * fcal_div * PLL_2063_MHZ)) + 1) >> 1) - 1;
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_6, (0x07 << 0), to >> 2);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_7, (0x03 << 5), to << 5);


	/* PLL_cal_ref_timeout */
	refTo = (uint8)((((fref * PLL_2063_CAL_REF_TO) / (fcal_div * (to + 1))) +
		(PLL_2063_MHZ - 1)) / PLL_2063_MHZ) - 1;
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_5, refTo);

	/* PLL_calSetCount */
	setCount = (uint32)FLOAT(
		(fixed)wlc_lpphy_qdiv_roundup(qFvco, qFcal * 16, 16) * (refTo + 1) * (to + 1)) - 1;
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_7, (0x0f << 0), (uint8)(setCount >> 8));
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCOCAL_8, (uint8)(setCount & 0xff));

	/* Divider, integer bits */
	div_int = ((fvco3 * (PLL_2063_MHZ >> 4)) / fref3) << 4;

	/* Divider, fractional bits */
	div_frac = ((fvco3 * (PLL_2063_MHZ >> 4)) % fref3) << 4;
	while (div_frac >= fref3) {
		div_int++;
		div_frac -= fref3;
	}
	div_frac = wlc_lpphy_qdiv_roundup(div_frac, fref3, 20);

	/* Program PLL */
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_SG_1, (0x1f << 0), (uint8)(div_int >> 4));
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_SG_2, (0x1f << 4), (uint8)(div_int << 4));
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_SG_2, (0x0f << 0), (uint8)(div_frac >> 16));
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_SG_3, (uint8)(div_frac >> 8) & 0xff);
	write_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_SG_4, (uint8)div_frac & 0xff);

	/* REmoving the hard coded values for PLL registers and make it */
	/* programmable with loop bw and d30 */

	/* PLL_cp_current */
	qVco = ((PLL_2063_HIGH_END_KVCO - PLL_2063_LOW_END_KVCO) *
		((qFvco - FIXED(PLL_2063_LOW_END_VCO)) /
		(PLL_2063_HIGH_END_VCO - PLL_2063_LOW_END_VCO))) +
		FIXED(PLL_2063_LOW_END_KVCO);
	if ((CHSPEC_IS5G(pi->radio_chanspec) && (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA)) ||
		(!rfpll_doubler)) {
		loop_bw = PLL_2063_LOOP_BW_ePA;
		d30 = PLL_2063_D30_ePA;
	} else {
		loop_bw = PLL_2063_LOOP_BW;
		d30 = PLL_2063_D30;
	}
	h29 = (uint16) wlc_lpphy_qdiv_roundup(loop_bw * 10, 270, 0); /* h29 * 10 */
	h30 = (uint16) wlc_lpphy_qdiv_roundup(d30 * 10, 2640, 0); /* h30 * 10 */

	/* PLL_lf_r1 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup((d30 - 680), 490, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_3, 0x1f << 3, temp_pll << 3);

	/* PLL_lf_r2 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup((1660 * h30 - 6800), 4900, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_3, 0x7, (temp_pll >> 2));
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_4, 0x3 << 5, (temp_pll & 0x3) << 5);

	/* PLL_lf_r3 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup((1660 * h30 - 6800), 4900, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_4, 0x1f, temp_pll);

	/* PLL_lf_c1 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup(1046500, h30 * h29, 0);
	temp_pll_1 = (uint16) wlc_lpphy_qdiv_roundup((temp_pll - 1775), 555, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_1, 0xf << 4, temp_pll_1 << 4);

	/* PLL_lf_c2 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup(61700, h29 * h30, 0);
	temp_pll_1 = (uint16) wlc_lpphy_qdiv_roundup((temp_pll - 123), 38, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_1, 0xf, temp_pll_1);

	/* PLL_lf_c3 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup(27000, h29 * h30, 0);
	temp_pll_1 = (uint16) wlc_lpphy_qdiv_roundup((temp_pll - 61), 19, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_2, 0xf << 4, temp_pll_1 << 4);

	/* PLL_lf_c4 */
	temp_pll = (uint16) wlc_lpphy_qdiv_roundup(26400, h29 * h30, 0);
	temp_pll_1 = (uint16) wlc_lpphy_qdiv_roundup((temp_pll - 55), 19, 0);
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_LF_2, 0xf, temp_pll_1);


	iVal = ((d30 - 680)  + (490 >> 1))/ 490;
	qVal = wlc_lpphy_qdiv_roundup(
		440 * loop_bw * div_int,
		27 * (68 + (iVal * 49)), 16);
	kpd_scale = ((qVal + qVco - 1) / qVco) > 60 ? 1 : 0;
	if (kpd_scale)
		cp_current = ((qVal + qVco) / (qVco << 1)) - 8;
	else
		cp_current = ((qVal + (qVco >> 1)) / qVco) - 8;
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_2, 0x3f, cp_current);

	/*  PLL_Kpd_scale2 */
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_2, 1 << 6, (kpd_scale << 6));

	/* PLL_offset_current */
	qVal = wlc_lpphy_qdiv_roundup(100 * qFref, qFvco, 16) * (cp_current + 8) * (kpd_scale + 1);
	ioff_scale = (qVal > FIXED(150)) ? 1 : 0;
	qVal = (qVal / (6 * (ioff_scale + 1))) - FIXED(2);
	if (qVal < 0)
		offset_current = 0;
	else
		offset_current = FLOAT(qVal + (FIXED(1) >> 1));
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_3, 0x1f, offset_current);

	/*  PLL_ioff_scale2 */
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_CP_3, 1 << 5, ioff_scale << 5);
#ifdef BAND5G
	if ((CHSPEC_IS5G(pi->radio_chanspec)) && (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_HGPA))
		wlc_sslpnphy_pll_aband_tune(pi, channel);
#endif
	/* PLL_pd_div2_BB */
	mod_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 1 << 2, rfpll_doubler << 2);

	/* PLL_cal_xt_endiv */
	if (!rfpll_doubler || (XTALFREQ(pi->xtalfreq) > 26000000))
		or_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 0x02);
	else
		and_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 0xfd);

	/* PLL_cal_xt_sdiv */
	if (!rfpll_doubler && (XTALFREQ(pi->xtalfreq) > 26000000))
		or_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 0x01);
	else
		and_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_XTAL_1_2, 0xfe);

	/* PLL_sel_short */
	if (qFref > FIXED(45))
		or_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCO_1, 0x02);
	else
		and_radio_reg(pi, RADIO_2063_PLL_JTAG_PLL_VCO_1, 0xfd);

	mod_radio_reg(pi, RADIO_2063_PLL_SP_2, 0x03, 0x03);
	OSL_DELAY(1);
	mod_radio_reg(pi, RADIO_2063_PLL_SP_2, 0x03, 0);

	/* Force VCO cal */
	wlc_2063_vco_cal(pi);

	/* Restore state */
	write_radio_reg(pi, RADIO_2063_COMMON_15, rf_common15);
}

#ifdef PHYHAL
/* these two routines are obsolete */
int8
wlc_phy_get_tx_power_offset(wlc_phy_t *ppi, uint8 tbl_offset)
{
	return 0;
}

int8
wlc_phy_get_tx_power_offset_by_mcs(wlc_phy_t *ppi, uint8 mcs_offset)
{
	return 0;
}
#endif /* PHYHAL */

#ifdef TBD_YE
/*
 * Converts channel number to channel frequency.
 * Returns 0 if the channel is out of range.
 * Also used by some code in wlc_iw.c
 */
uint
wlc_phy_channel2freq(uint channel)
{
	uint i;

	for (i = 0; i < ARRAYSIZE(wlc_phy_chan_info); i++)
		if (wlc_phy_chan_info[i].chan == channel)
			return (wlc_phy_chan_info[i].freq);
	return (0);
}


/* Converts channel number into the wlc_phy_chan_info index */
static uint
wlc_phy_channel2idx(uint channel)
{
	uint i;

	for (i = 0; i < ARRAYSIZE(wlc_phy_chan_info); i++) {
		if (wlc_phy_chan_info[i].chan == channel)
			return i;
	}

	ASSERT(FALSE);
	return (0);
}


/*
 * Converts radio specific channel code to channel number
 * Returns 0 if the radio code does not match any channel definition.
 */
uint8
wlc_phy_radiocode2channel(uint8 radiocode, uint phytype)
{
	if (!PHYTYPE_IS(phytype, PHY_TYPE_G)) {
		/* radiocodes for all newer phys (i.e. non g) are the actual channel numbers */
		return radiocode;
	}

	/* Radiocodes for b/g are the frequency offsets from 2400 MHz. Each channel
	 * is 5 MHz apart starting with channel 1 at 2412 MHz, except the oddball
	 * channel 14 at 2484 MHz.
	 */
	if (radiocode >= 12 && radiocode <= 72) {
		return (radiocode - 7) / 5;
	} else if (radiocode == 84) {
		return 14;
	} else {
		WL_ERROR(("wlc_phy_radiocode2channel: Cannot find radiocode %d\n", radiocode));
		return 0;
	}
}

/*
 * Converts channel number to radio specific channel code
 * return 0 if the channel is out of range
 */
uint16
wlc_phy_channel2radiocode(uint16 channel, uint phytype)
{
	if (!PHYTYPE_IS(phytype, PHY_TYPE_G)) {
		/* radiocodes for aphy are the actual channel numbers */
		return channel;
	}
}

/* fill out a chanvec_t with all the supported channels for the band. */
void
wlc_phy_band_channels(wlc_phy_t *ppi, uint band, chanvec_t *channels)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	uint i;
	uint channel;

	ASSERT((band == WLC_BAND_2G) || (band == WLC_BAND_5G));

	bzero(channels, sizeof(chanvec_t));

	for (i = 0; i < ARRAYSIZE(wlc_phy_chan_info); i++) {
		channel = wlc_phy_chan_info[i].chan;

		/* disable the high band channels [149-165] for srom ver 1 */
		if ((pi->a_band_high_disable) && (channel >= FIRST_REF5_CHANNUM) &&
		    (channel <= LAST_REF5_CHANNUM))
			continue;

		if (((band == WLC_BAND_2G) && (channel <= WLC_MAX_2G_CHANNEL)) ||
		    ((band == WLC_BAND_5G) && (channel > WLC_MAX_2G_CHANNEL)))
			setbit(channels->vec, channel);
	}
}

#if defined(BAND5G) || !defined(WL20MHZ_ONLY)
/* returns the first hw supported channel in the band */
chanspec_t
wlc_phy_band_first_chanspec(wlc_phy_t *ppi, uint band)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	uint i;
	uint channel;
	chanspec_t chspec;

	ASSERT((band == WLC_BAND_2G) || (band == WLC_BAND_5G));

	for (i = 0; i < ARRAYSIZE(wlc_phy_chan_info); i++) {
		channel = wlc_phy_chan_info[i].chan;

		/* If 40MHX b/w then check if there is an upper 20Mhz adjacent channel */
		if ((ISNPHY(pi) || ISSSLPNPHY(pi)) && IS40MHZ(pi)) {
			uint j;
			/* check if the upper 20Mhz channel exists */
			for (j = 0; j < ARRAYSIZE(wlc_phy_chan_info); j++) {
				if (wlc_phy_chan_info[j].chan == channel + CH_10MHZ_APART)
					break;
			}
			/* did we find an adjacent channel */
			if (j == ARRAYSIZE(wlc_phy_chan_info))
				continue;
			/* Convert channel from 20Mhz num to 40 Mhz number */
			channel = UPPER_20_SB(channel);
			chspec = channel | WL_CHANSPEC_BW_40 | WL_CHANSPEC_CTL_SB_LOWER;
			if (band == WLC_BAND_2G)
				chspec |= WL_CHANSPEC_BAND_2G;
			else
				chspec |= WL_CHANSPEC_BAND_5G;
		}
		else
			chspec = CH20MHZ_CHSPEC(channel);

		/* disable the high band channels [149-165] for srom ver 1 */
		if ((pi->a_band_high_disable) && (channel >= FIRST_REF5_CHANNUM) &&
		    (channel <= LAST_REF5_CHANNUM))
			continue;

		if (((band == WLC_BAND_2G) && (channel <= WLC_MAX_2G_CHANNEL)) ||
		    ((band == WLC_BAND_5G) && (channel > WLC_MAX_2G_CHANNEL)))
			return chspec;
	}

	/* should never come here */
	ASSERT(0);

	/* to avoid warning */
	return (chanspec_t)INVCHANSPEC;
}
#endif /* defined(BAND5G)  || !defined (WL20MHZ_ONLY) */


/* get sromlimit per rate for given channel. Routine does not account for ant gain */
void
wlc_phy_txpower_sromlimit(wlc_phy_t *ppi, uint channel, uint8 *min_pwr, uint8 *max_pwr,
	int txp_rate_idx)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	/* minimum reliable txpwr target is 8 dBm */
	*min_pwr = 8 * WLC_TXPWR_DB_FACTOR;

	if (ISGPHY(pi) || (channel <= WLC_MAX_2G_CHANNEL)) {
		/* until we cook the maxtxpwr value into the channel table,
		 * use the one global B band maxtxpwr
		 */
		if (txp_rate_idx < 0)
			txp_rate_idx = TXP_FIRST_CCK;

		/* legacy phys don't have valid MIMO rate entries */
		/* SSLPNPHY does have 8 more entries for MCS 0-7 (exists for 40Mhz) */
		/* Cannot re-use nphy logic right away 2nd opinion: Can use nphy logic for  */
		/* sake of simplicity */
		ASSERT(txp_rate_idx <= TXP_NUM_RATES);

		*max_pwr = pi->tx_srom_max_rate_2g[txp_rate_idx];
	}
#if defined(BAND5G)
	else {
		uint i;

		/* in case we fall out of the channel loop */
		*max_pwr = WLC_TXPWR_MAX;

		if (txp_rate_idx < 0)
			txp_rate_idx = TXP_FIRST_OFDM;
		/* max txpwr is channel dependent */
		for (i = 0; i < ARRAYSIZE(wlc_phy_chan_info); i++) {
			if (channel == wlc_phy_chan_info[i].chan) {
				break;
			}
		}
		ASSERT(i < ARRAYSIZE(wlc_phy_chan_info));

		/* legacy phys don't have valid MIMO rate entries */
		/* legacy phys don't have valid MIMO rate entries */
		/* SSLPNPHY does have 8 more entries for MCS 0-7 except for 40Mhz */

		ASSERT(txp_rate_idx <= TXP_NUM_RATES);


		if (pi->hwtxpwr) {
			*max_pwr = pi->hwtxpwr[i];
		} else {
			/* When would we get here?  B only? */
			if ((i >= FIRST_MID_5G_CHAN) && (i <= LAST_MID_5G_CHAN))
				*max_pwr = pi->tx_srom_max_rate_5g_mid[txp_rate_idx];
			if ((i >= FIRST_HIGH_5G_CHAN) && (i <= LAST_HIGH_5G_CHAN))
				*max_pwr = pi->tx_srom_max_rate_5g_hi[txp_rate_idx];
			if ((i >= FIRST_LOW_5G_CHAN) && (i <= LAST_LOW_5G_CHAN))
				*max_pwr = pi->tx_srom_max_rate_5g_low[txp_rate_idx];
		}
		/* SSLPNPHY has different sub-band range limts for the A-band compared to MIMOPHY
		 * (see sslpnphy_get_paparams in sslpnphyprocs.tcl)
		 */
		if ((channel >= FIRST_LOW_5G_CHAN_SSLPNPHY) && (channel <= LAST_LOW_5G_CHAN_SSLPNPHY)) {
		        *max_pwr = pi->tx_srom_max_rate_5g_low[txp_rate_idx];
		}
		if ((channel >= FIRST_MID_5G_CHAN_SSLPNPHY) && (channel <= LAST_MID_5G_CHAN_SSLPNPHY)) {
		        *max_pwr = pi->tx_srom_max_rate_5g_mid[txp_rate_idx];
		}
		if ((channel >= FIRST_HIGH_5G_CHAN_SSLPNPHY) && (channel <= LAST_HIGH_5G_CHAN_SSLPNPHY)) {
			*max_pwr = pi->tx_srom_max_rate_5g_hi[txp_rate_idx];
		}

	}
#endif /* BAND5G */
	WL_NONE(("%s: chan %d rate idx %d, sromlimit %d\n", __FUNCTION__, channel, txp_rate_idx,
		*max_pwr));
}


uint8 sslpnphy_mcs_to_legacy_map[8] = {0, 2, 3, 4, 5, 6, 7, 7 + 8};

int8
wlc_phy_get_tx_power_offset(wlc_phy_t *ppi, uint8 tbl_offset)
{
	phy_info_t *pi = (phy_info_t*)ppi;

	return pi->tx_power_offset[sslpnphy_mcs_to_legacy_map[tbl_offset] + 4];
}

int8
wlc_phy_get_tx_power_offset_by_mcs(wlc_phy_t *ppi, uint8 mcs_offset, bool is40m)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	if (is40m)
		if (mcs_offset == 32)
			return pi->tx_power_offset[TXP_LAST_MCS_40];
		else
			return pi->tx_power_offset[mcs_offset + TXP_FIRST_MCS_40];
	else
		return pi->tx_power_offset[mcs_offset + TXP_FIRST_MCS_20];
}

uint8 sslpnphy_legacy_rate_to_index_map[12] = {2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108};

int16
wlc_phy_get_legacy_txpwroffset(wlc_phy_t *ppi, uint8 offset)
{
	uint i, index = 0;
	phy_info_t *pi = (phy_info_t*) ppi;
	for (i = 0; i < 12; i++) {
		if (sslpnphy_legacy_rate_to_index_map[i] == offset) {
			index = i;
			break;
		}
	}
	return (pi->tx_power_offset[index]);
}
int16
wlc_phy_get_mcs_txpwroffset(wlc_phy_t *ppi, uint8 offset)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	return (pi->tx_power_offset[12 + offset]);
}
/* Recalc target power all phys.
 * this function needs to be called whenever user_target, regulatatory, srom limit are changed
 *   If channel == -1, use current radio_channel, derive txpower and push to hardware
 *   If a channel is specified, return the maximum tx power for the channel and
 *      do not change hardware i.e query only, XXXBMAC, only ccx uses this, need to resolve
 */
#define ANT1MAXPWR_5G   34
#define ANT1MAXPWR_2G   50

void
wlc_phy_txpower_recalc_target(wlc_phy_t *ppi, int channel, uint8 *chan_txpwr)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	uint8 maxtxpwr, mintxpwr, rate, pactrl;
	uint target_chan;
	uint8 tx_pwr_target[TXP_NUM_RATES];
	uint8 tx_pwr_max = 0;
	uint8 tx_pwr_min = 255;
	uint8 max_num_rate, start_rate = 0;

	int voltage;
	bool ninja_board_flag = (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94319SDELNA6L_SSID);

	if (channel == -1) {
		chanspec_t chspec = pi->radio_chanspec;

		/* use current radio channel since no channel given */
		if (CHSPEC_CTL_SB(chspec) == WL_CHANSPEC_CTL_SB_NONE)
			target_chan = CHSPEC_CHANNEL(chspec);
		else if (CHSPEC_CTL_SB(chspec) == WL_CHANSPEC_CTL_SB_UPPER)
			target_chan = UPPER_20_SB(CHSPEC_CHANNEL(chspec));
		else
			target_chan = LOWER_20_SB(CHSPEC_CHANNEL(chspec));
	} else {
		/* use given channel(may different with current radio channel) */
		target_chan = channel;
	}

	pactrl = 0;
	if (ISGPHY(pi) && (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_PACTRL))
		pactrl = 3;

	max_num_rate = (ISNPHY(pi) || ISSSLPNPHY(pi))  ? (TXP_NUM_RATES) : (TXP_LAST_OFDM + 1);

	voltage = sslpnphy_specific->sslpnphy_volt_low;
	/* voltage sensor has an error of 0.2V for temperature belowe 0C */
	if (sslpnphy_specific->sslpnphy_lastsensed_temperature < 0)
		voltage = voltage - 3;
#ifdef BAND5G
	start_rate =  ((CHSPEC_IS5G(pi->radio_chanspec)) ? 4 : 0);
#endif /* BAND5G */

	/* Factor in various constraints such as regulatory, hardware and power percentage
	 * to determine target power for each rate.  Don't introduce side effects by changing
	 * globals since this may called to just query power, not actually set power.
	 */
	for (rate = start_rate; rate < max_num_rate; rate++) {
		/* The user target is the starting point for determining the transmit
		 * power.  If pi->txoverride is true, then use the user target as the
		 * tx power target for all rates.
		 */
		 /* skip cdd_mimo rates for SSLPNPHY */
		if ((rate == TXP_FIRST_MCS_SISO_20_CDD) || (rate == TXP_FIRST_MCS_SISO_40_CDD))	{
		/* If 20MHz only, no more rate is left. Skip all the rest */
#ifdef WL20MHZ_ONLY
			break ;
#else
			rate = rate + 7;
			continue;
#endif
		}
		 tx_pwr_target[rate] = pi->tx_user_target[rate];

		{
			/* Get hw limit */
			wlc_phy_txpower_sromlimit(ppi, target_chan, &mintxpwr, &maxtxpwr,
				rate);

			WL_NONE((" %d:    %d	 %d\n", rate, maxtxpwr, pi->txpwr_limit[rate]));
			maxtxpwr = (maxtxpwr > pactrl) ? (maxtxpwr - pactrl) : 0;

			/* Don't go over regulatory limits */
			/* Subtract 6 (1.5db) to ensure we don't go over */
			/* the limit given a noisy power detector  */

			/* Do not go over regulatory, board limits */
			maxtxpwr = MIN(maxtxpwr, (pi->txpwr_limit[rate]));

			/* add 1.5 dB backoff */	
			maxtxpwr = (maxtxpwr > 6) ? (maxtxpwr - 6) : 0;

			/* Choose least of USER, regulatatory and hardware targets */
			tx_pwr_target[rate] = MIN(tx_pwr_target[rate], maxtxpwr);

		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
#if defined(WLPLT)
			if (tx_pwr_target[rate] < 40) {
				tx_pwr_target[rate] = tx_pwr_target[rate] - 4;
			} else
#endif /* WLPLT */
			{
				if (rate > 11)
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						sslpnphy_specific->sslpnphy_11n_backoff;
				else if ((rate >= 8) && (rate <= 11))
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						sslpnphy_specific->sslpnphy_54_48_36_24mbps_backoff;
				else if (rate <= 3)
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						sslpnphy_specific->sslpnphy_cck;
				else
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						sslpnphy_specific->sslpnphy_lowerofdm;
			}
		}


			/* power output percentage */
			tx_pwr_target[rate] = (tx_pwr_target[rate] * GENERIC_PHY_INFO(pi)->txpwr_percent) / 100;
		}
		tx_pwr_max = MAX(tx_pwr_max, tx_pwr_target[rate]);
		tx_pwr_min = MIN(tx_pwr_min, tx_pwr_target[rate]);
	}

		{
		/* Limit X17 ANT1 targert power to 8.5/12.5 dBm per Olympic */
		if (BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) == BCM94329OLYMPICX17U_SSID)
			sslpnphy_specific->sslpnphy_ant1_max_pwr =
				CHSPEC_IS5G(pi->radio_chanspec) ? 34 : 50;
		else
			sslpnphy_specific->sslpnphy_ant1_max_pwr = tx_pwr_max;
		}

	/* if channel is given, get power for the channel and finish */
	if (channel != -1) {
		ASSERT(chan_txpwr != NULL);
		*chan_txpwr = tx_pwr_max;
		return;
	}

	/* Now calculate the tx_power_offset and update the hardware... */
	bzero(pi->tx_power_offset, sizeof(pi->tx_power_offset));
	pi->tx_power_max = tx_pwr_max;
	pi->tx_power_min = tx_pwr_min;

	WL_NONE(("wl%d: %s: channel %d rate - targets - offsets - limits - user_target\n",
		GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, target_chan));
	for (rate = start_rate; rate < max_num_rate; rate++) {
		/* For swpwrctrl, the offset is OFDM w.r.t. CCK.
		 * For hwpwrctl, otherway around
		 */
		 /* skip cdd/mimo rates for SSLPNPHY */
		if ((rate == TXP_FIRST_MCS_SISO_20_CDD)|| (rate == TXP_FIRST_MCS_SISO_40_CDD)) {
			rate = rate + 7;
			continue;
		}
		pi->tx_power_target[rate] = tx_pwr_target[rate];

		if (!pi->hwpwrctrl || ISNPHY(pi)) {
			pi->tx_power_offset[rate] = pi->tx_power_max -
				pi->tx_power_target[rate];
		} else {
			pi->tx_power_offset[rate] = pi->tx_power_target[rate] - pi->tx_power_min;
			/* For ninja boards, target power is set to the max power found by scanning
			 * all the rates. The positive offsets are calculated as the difference
			 * between the max power and the target power for each rate. These positive
			 * offsets are written to the rate table
			 */
			if (ninja_board_flag && (CHSPEC_IS5G(pi->radio_chanspec))) {
				pi->tx_power_offset[rate] = pi->tx_power_max - pi->tx_power_target[rate];
			}
		}
		WL_NONE(("    %d:    %d    %d    %d    %d\n", rate, pi->tx_power_target[rate],
			pi->tx_power_offset[rate], pi->txpwr_limit[rate],
			pi->tx_user_target[rate]));
	}

	if (ISSSLPNPHY(pi) && wlc_sslpnphy_tssi_enabled(pi)) {
		uint16 pwr_ctrl;

		/* Temporary disable power control to update settings */
		pwr_ctrl = wlc_sslpnphy_get_tx_pwr_ctrl(pi);
		wlc_sslpnphy_set_tx_pwr_ctrl(pi, SSLPNPHY_TX_PWR_CTRL_OFF);
		wlc_sslpnphy_txpower_recalc_target(pi);
		/* Restore power control */
		wlc_sslpnphy_set_tx_pwr_ctrl(pi, pwr_ctrl);
	}
}

/* Set tx power limits */
/* BMAC_NOTE: this only needs a chanspec so that it can choose which 20/40 limits
 * to save in phy state. Would not need this if we ether saved all the limits and
 * applied them only when we were on the correct channel, or we restricted this fn
 * to be called only when on the correct channel.
 */
void
wlc_phy_txpower_limit_set(wlc_phy_t *ppi, struct txpwr_limits *txpwr, chanspec_t chanspec)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	int i, j;
	WL_NONE(("wl%d: %s", GENERIC_PHY_INFO(pi)->unit, __FUNCTION__));
	WL_NONE(("cck rates\n"));
	for (i = TXP_FIRST_CCK, j = i; i <= TXP_LAST_CCK; i++, j++) {
		WL_NONE(("    %d%s -> %d%s\n",
		         pi->txpwr_limit[i] / WLC_TXPWR_DB_FACTOR,
		         fraction[pi->txpwr_limit[i] % WLC_TXPWR_DB_FACTOR],
		         txpwr->cck[j] / WLC_TXPWR_DB_FACTOR,
		         fraction[txpwr->cck[j] % WLC_TXPWR_DB_FACTOR]));
		pi->txpwr_limit[i] = txpwr->cck[j];
	}
	WL_NONE(("ofdm rates\n"));
	for (i = TXP_FIRST_OFDM, j = 0; i <= TXP_LAST_OFDM; i++, j++) {
		WL_NONE(("    %d%s -> %d%s\n",
		         pi->txpwr_limit[i] / WLC_TXPWR_DB_FACTOR,
		         fraction[pi->txpwr_limit[i] % WLC_TXPWR_DB_FACTOR],
		         txpwr->ofdm[j] / WLC_TXPWR_DB_FACTOR,
		         fraction[txpwr->ofdm[j] % WLC_TXPWR_DB_FACTOR]));

		pi->txpwr_limit[i] = txpwr->ofdm[j];


	}



	if (ISSSLPNPHY(pi)) {
		for (i = TXP_FIRST_MCS_20, j = 0; j < WLC_NUM_RATES_MCS_SISO; i++, j++) {
			if (txpwr->mcs_20_siso[j])
				pi->txpwr_limit[i] = txpwr->mcs_20_siso[j];
			else
				pi->txpwr_limit[i] = txpwr->ofdm[j];
		}

#ifndef WL20MHZ_ONLY
		for (i = TXP_FIRST_MCS_40, j = 0; j < WLC_NUM_RATES_MCS_SISO; i++, j++) {
		/* condition to take care if a country does not know the limits of its */
		/*	power for 40Mhz */	
			if (txpwr->mcs_40_siso[j])
				pi->txpwr_limit[i] = txpwr->mcs_40_siso[j];
			else
				pi->txpwr_limit[i] = txpwr->ofdm[j];
		}
		/* txpwr for mcs32. if mcs32 does not exist use 40Mhz m7 rate */
		if (txpwr->mcs32)
			pi->txpwr_limit[TXP_LAST_MCS_40] = txpwr->mcs32;
		else
			pi->txpwr_limit[TXP_LAST_MCS_40] = pi->txpwr_limit[TXP_LAST_MCS_SISO_40];
#endif
	}
}




#endif /* TBD_YE */



#if TBD_YE





/* handle phy related iovars */
int
wlc_phy_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
	void *p, uint plen, void *a, int alen, int vsize, struct wlc_if *wlcif)
{
	wlc_info_t *wlc = (wlc_info_t *)hdl;
	phy_info_t *pi;
	int32 int_val = 0;
	bool bool_val;
	int err = 0;
	int32 *ret_int_ptr = (int32 *)a;
	uint8 band_idx;

	/* Get the current phy */
	pi = wlc_cur_phy(wlc);

	if ((err = wlc_iovar_check(pi->pub, vi, a, alen, IOV_ISSET(actionid))) != 0)
		return err;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

	switch (actionid) {
	case IOV_GVAL(IOV_QTXPOWER): {
		uint qdbm;
		bool override;

		GET_GATE(IOV_QTXPOWER);
		if ((err = wlc_phy_txpower_get((wlc_phy_t *)pi, &qdbm, &override)) != BCME_OK)
			return err;

		/* Return qdbm units */
		*ret_int_ptr = qdbm | (override ? WL_TXPWR_OVERRIDE : 0);
		break;
	}

	case IOV_SVAL(IOV_QTXPOWER): {
		uint8 qdbm;
		bool override;

		SET_GATE(IOV_QTXPOWER);
		/* Remove override bit and clip to max qdbm value */
		qdbm = (uint8)MIN((int_val & ~WL_TXPWR_OVERRIDE), 0xff);
		/* Extract override setting */
		override = (int_val & WL_TXPWR_OVERRIDE) ? TRUE : FALSE;
		err = wlc_phy_txpower_set((wlc_phy_t *)pi, qdbm, override);
		break;
	}
#if !SSLPNCONF
#if defined(AP) && defined(RADAR)
	case IOV_GVAL(IOV_RADAR_ARGS):
		GET_GATE(IOV_RADAR_ARGS);
		bcopy((char*)&pi->rargs[0].radar_args, (char*)a, sizeof(wl_radar_args_t));
		break;

	case IOV_SVAL(IOV_RADAR_ARGS):
		SET_GATE(IOV_RADAR_ARGS);
		bcopy((char*)p, (char*)&pi->rargs[0].radar_args, sizeof(wl_radar_args_t));
		/* apply radar inits to hardware if we are on the A/LP/NPHY */
		if (GENERIC_PHY_INFO(pi)->up && (ISAPHY(pi) || ISLPPHY(pi) || ISNPHY(pi) || ISSSLPNPHY(pi)))
			wlc_phy_radar_detect_init((wlc_phy_t *)pi, GENERIC_PHY_INFO(pi)->radar != 0);
		break;
	case IOV_GVAL(IOV_RADAR_ARGS_40MHZ):
		GET_GATE(IOV_RADAR_ARGS_40MHZ);
		/* any other phy supports 40Mhz channel ? */
		if (!ISNPHY(pi)) {
			err = BCME_UNSUPPORTED;
			break;
		}
		bcopy((char*)&pi->rargs[1].radar_args, (char*)a, sizeof(wl_radar_args_t));
		break;

	case IOV_SVAL(IOV_RADAR_ARGS_40MHZ):
		SET_GATE(IOV_RADAR_ARGS_40MHZ);
		/* any other phy supports 40Mhz channel ? */
		if (!ISNPHY(pi)) {
			err = BCME_UNSUPPORTED;
			break;
		}

		bcopy((char*)p, (char*)&pi->rargs[1].radar_args, sizeof(wl_radar_args_t));
		/* apply radar inits to hardware if we are NPHY */
		if (GENERIC_PHY_INFO(pi)->up)
			wlc_phy_radar_detect_init((wlc_phy_t *)pi, GENERIC_PHY_INFO(pi)->radar != 0);
		break;
#endif /* defined(AP) && defined(RADAR) */
#endif /* SSLPNCONF */

#if defined(WLTIMER)
	case IOV_GVAL(IOV_FAST_TIMER):
		GET_GATE(IOV_FAST_TIMER);
		*ret_int_ptr = (int32)pi->sh->fast_timer;
		break;

	case IOV_SVAL(IOV_FAST_TIMER):
		SET_GATE(IOV_FAST_TIMER);
		pi->sh->fast_timer = (uint32)int_val;
		break;

	case IOV_GVAL(IOV_SLOW_TIMER):
		GET_GATE(IOV_SLOW_TIMER);
		*ret_int_ptr = (int32)pi->sh->slow_timer;
		break;

	case IOV_SVAL(IOV_SLOW_TIMER):
		SET_GATE(IOV_SLOW_TIMER);
		pi->sh->slow_timer = (uint32)int_val;
		break;

	case IOV_GVAL(IOV_GLACIAL_TIMER):
		GET_GATE(IOV_GLACIAL_TIMER);
		*ret_int_ptr = (int32)pi->sh->glacial_timer;
		break;

	case IOV_SVAL(IOV_GLACIAL_TIMER):
		SET_GATE(IOV_GLACIAL_TIMER);
		pi->sh->glacial_timer = (uint32)int_val;
		break;
#endif 

	case IOV_GVAL(IOV_PHY_RSSI_ANT): {
		wl_rssi_ant_t rssi_ant;
#if NCONF
		uint32 i, idx;
		 /* use int32 to avoid overflow when accumulate int8 */
		int32 rssi_sum[ANTENNA_RX_MAX_NPHY] = { 0 };
#endif /* NCONF */

		GET_GATE(IOV_PHY_RSSI_ANT);
		bzero((char *)&rssi_ant, sizeof(wl_rssi_ant_t));
		rssi_ant.version = WL_RSSI_ANT_VERSION;

		/* only get RSSI for one antenna for all SISO PHY */
		if (ISAPHY(pi) || ISGPHY(pi) || ISLPPHY(pi)) {
			rssi_ant.count = 1;
			rssi_ant.rssi_ant[0] = (int8) wlc->cfg.link->rssi;
		}
#if NCONF
		else if (ISNPHY(pi)) {
			rssi_ant.count = ANTENNA_RX_MAX_NPHY;
			idx = pi->nphy_rssi_index;
			for (i = 0; i < NPHY_RSSI_WINDOW_SZ; i++) {
				rssi_sum[ANTENNA_IDX_1] += pi->nphy_rssi_win[ANTENNA_IDX_1][idx];
				rssi_sum[ANTENNA_IDX_2] += pi->nphy_rssi_win[ANTENNA_IDX_2][idx];
				idx = MODINC_POW2(idx, NPHY_RSSI_WINDOW_SZ);
			}
			rssi_ant.rssi_ant[ANTENNA_IDX_1] = (int8)(rssi_sum[ANTENNA_IDX_1]
				/ NPHY_RSSI_WINDOW_SZ);
			rssi_ant.rssi_ant[ANTENNA_IDX_2] = (int8)(rssi_sum[ANTENNA_IDX_2]
				/ NPHY_RSSI_WINDOW_SZ);
		}
#endif /* NCONF */
		else {
			rssi_ant.count = 0;
		}

		bcopy((char*)&rssi_ant, (char*)a, sizeof(wl_rssi_ant_t));
		break;
	}



#if NCONF
	case IOV_GVAL(IOV_NPHY_TXRX_CHAIN):
		GET_GATE(IOV_NPHY_TXRX_CHAIN);
		*ret_int_ptr = (int)pi->nphy_txrx_chain;
		break;

	case IOV_SVAL(IOV_NPHY_TXRX_CHAIN):
	{
		SET_GATE(IOV_NPHY_TXRX_CHAIN);
		if ((int_val != AUTO) && (int_val != WLC_N_TXRX_CHAIN0) &&
			(int_val != WLC_N_TXRX_CHAIN1)) {
			err = BCME_RANGE;
			break;
		}
		if (pi->nphy_txrx_chain != (int8)int_val) {
			pi->nphy_txrx_chain = (int8)int_val;
			if (GENERIC_PHY_INFO(pi)->up) {
				WL_SUSPEND_MAC_AND_WAIT(pi);
				wlc_phyreg_enter((wlc_phy_t *)pi);
				wlc_nphy_update_txrx_chain(pi);
				wlc_phy_force_rfseq_nphy(pi, NPHY_RFSEQ_RESET2RX);
				wlc_phyreg_exit((wlc_phy_t *)pi);
				WL_ENABLE_MAC(pi);
			}
		}
		break;
	}


	case IOV_GVAL(IOV_NPHY_TEMPSENSE):
		GET_GATE(IOV_NPHY_TEMPSENSE);
		WL_SUSPEND_MAC_AND_WAIT(pi);
		wlc_phyreg_enter((wlc_phy_t *)pi);
		*ret_int_ptr = (int32)wlc_nphy_tempsense(pi);
		wlc_phyreg_exit((wlc_phy_t *)pi);
		WL_ENABLE_MAC(pi);
		break;


#endif /* NCONF */

#if WL11N
	case IOV_GVAL(IOV_NPHY_PREAMBLE):
		GET_GATE(IOV_NPHY_PREAMBLE);
		*ret_int_ptr = (int32)wlc_phy_get_n_preamble_override((wlc_phy_t *)pi);
		break;

	case IOV_SVAL(IOV_NPHY_PREAMBLE):
	{
		SET_GATE(IOV_NPHY_PREAMBLE);
		if ((int_val != AUTO) && (int_val != WLC_N_PREAMBLE_MIXEDMODE) &&
		    (int_val != WLC_N_PREAMBLE_GF)) {
			err = BCME_RANGE;
			break;
		}

		if (ISNPHY(pi) && NREV_IS(pi->pubpi.phy_rev, 3))
			break;

		wlc_phy_set_n_preamble_override((wlc_phy_t *)pi, int_val);
		break;
	}

#endif /* NCONF */


	case IOV_GVAL(IOV_SSLPNPHY_FORCE_NOISE_CAL):
		GET_GATE(IOV_SSLPNPHY_FORCE_NOISE_CAL);
		*ret_int_ptr = (int32)sslpnphy_specific->sslpnphy_force_noise_cal;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_NOISE_CAL):
		SET_GATE(IOV_SSLPNPHY_FORCE_NOISE_CAL);
		sslpnphy_specific->sslpnphy_disable_noise_percal = bool_val;
		sslpnphy_specific->sslpnphy_force_noise_cal = bool_val;
		if (sslpnphy_specific->sslpnphy_force_noise_cal) {
			mod_phy_reg(pi, SSLPNPHY_HiGainDB,
				SSLPNPHY_HiGainDB_HiGainDB_MASK,
				sslpnphy_specific->sslpnphy_force_lgain << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				sslpnphy_specific->sslpnphy_force_rxpo <<
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
		} else {
			mod_phy_reg(pi, SSLPNPHY_HiGainDB,
				SSLPNPHY_HiGainDB_HiGainDB_MASK,
				pi->Listen_GaindB_AfrNoiseCal << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				pi->rxpo_required_AfrNoiseCal <<
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_SHIFT);
		}
		break;
	case IOV_GVAL(IOV_SSLPNPHY_FORCE_LGAIN):
		GET_GATE(IOV_SSLPNPHY_FORCE_LGAIN);
		*ret_int_ptr = (int8)sslpnphy_specific->sslpnphy_force_lgain;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_LGAIN):
		SET_GATE(IOV_SSLPNPHY_FORCE_LGAIN);
		sslpnphy_specific->sslpnphy_force_lgain = (int8)int_val;
		break;
	case IOV_GVAL(IOV_SSLPNPHY_FORCE_RXPO):
		GET_GATE(IOV_SSLPNPHY_FORCE_RXPO);
		*ret_int_ptr = (int8)sslpnphy_specific->sslpnphy_force_rxpo;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_RXPO):
		SET_GATE(IOV_SSLPNPHY_FORCE_RXPO);
		sslpnphy_specific->sslpnphy_force_rxpo = (int8)int_val;
		break;

#if SSLPNCONF
#if defined(DONGLEOVERLAYS)
	case IOV_SVAL(IOV_SSLPNPHY_PER_CAL):
		SET_GATE(IOV_SSLPNPHY_PER_CAL);
		wlc_sslpnphy_percal_iovar(pi, int_val);
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FULL_CAL):
		SET_GATE(IOV_SSLPNPHY_FULL_CAL);
		wlc_sslpnphy_full_cal(pi);
		break;
	case IOV_SVAL(IOV_SSLPNPHY_SETCHAN_CAL):
		SET_GATE(IOV_SSLPNPHY_SETCHAN_CAL);
		wlc_sslpnphy_setchan_cal(pi, int_val);
		break;
	case IOV_SVAL(IOV_SSLPNPHY_TXPWRINIT):
		SET_GATE(IOV_SSLPNPHY_TXPWRINIT);
		if (CHSPEC_BAND(pi->radio_chanspec) != CHSPEC_BAND((chanspec_t) int_val))
			break;
		sslpnphy_specific->sslpnphy_recal = 1;
		wlc_sslpnphy_tx_pwr_ctrl_init((wlc_phy_t *)pi);
		sslpnphy_specific->sslpnphy_recal = 0;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_NOISE_MEASURE):
		SET_GATE(IOV_SSLPNPHY_NOISE_MEASURE);
		wlc_sslpnphy_noise_measure_iovar(pi);
		break;
	case IOV_SVAL(IOV_SSLPNPHY_PAPD_RECAL):
		SET_GATE(IOV_SSLPNPHY_PAPD_RECAL);
		wlc_sslpnphy_papd_recal_iovar(pi);
		break;
#endif 
#endif /* SSLPNCONF */

	case IOV_GVAL(IOV_PHY_RXIQ_EST):
	{
		GET_GATE(IOV_PHY_RXIQ_EST);
		if (ISSSLPNPHY(pi))
		{
			return BCME_UNSUPPORTED;        /* lpphy support only for now */
		}

		break;
	}

	case IOV_SVAL(IOV_PHY_RXIQ_EST):
		SET_GATE(IOV_PHY_RXIQ_EST);
		if (ISSSLPNPHY(pi))
		{
			return BCME_UNSUPPORTED;        /* lpphy support only for now */
		}

		break;

#if LPCONF
	case IOV_GVAL(IOV_LPPHY_TEMPSENSE):
		GET_GATE(IOV_LPPHY_TEMPSENSE);
		int_val = wlc_lpphy_tempsense(pi);
		bcopy(&int_val, a, sizeof(int_val));
		break;

	case IOV_GVAL(IOV_LPPHY_CAL_DELTA_TEMP):
		GET_GATE(IOV_LPPHY_CAL_DELTA_TEMP);
		int_val = pi->lpphy_cal_delta_temp;
		bcopy(&int_val, a, sizeof(int_val));
		break;

	case IOV_SVAL(IOV_LPPHY_CAL_DELTA_TEMP):
		SET_GATE(IOV_LPPHY_CAL_DELTA_TEMP);
		pi->lpphy_cal_delta_temp = (int8)int_val;
		break;

	case IOV_GVAL(IOV_LPPHY_VBATSENSE):
		GET_GATE(IOV_LPPHY_VBATSENSE);
		int_val = wlc_lpphy_vbatsense(pi);
		bcopy(&int_val, a, sizeof(int_val));
		break;
#endif /* LPCONF */

	case IOV_GVAL(IOV_NUM_STREAM):
		GET_GATE(IOV_NUM_STREAM);
		if (ISNPHY(pi)) {
			int_val = 2;
		} else if (ISAPHY(pi) || ISGPHY(pi) || ISLPPHY(pi) || ISSSLPNPHY(pi)) {
			int_val = 1;
		} else {
			int_val = -1;
		}
		bcopy(&int_val, a, vsize);
		break;

	case IOV_GVAL(IOV_BAND_RANGE):
		GET_GATE(IOV_BAND_RANGE);
		int_val = wlc_get_band_range(pi, pi->radio_chanspec);
		bcopy(&int_val, a, vsize);
		break;

#ifdef STA
	case IOV_SVAL(IOV_RSSI_WINDOW_SZ):
		SET_GATE(IOV_RSSI_WINDOW_SZ);
		if (int_val > MA_WINDOW_SZ) {
			err = BCME_RANGE;
			break;
		}

		if ((int_val & (int_val - 1)) != 0) {
			/* Value passed is not power of 2 */
			err = BCME_BADARG;
			break;
		}
		pi->rssi_ma_win_sz = (uint16)int_val;
		wlc_phy_reset_rssi_ma(wlc->band->pi);
		break;

	case IOV_GVAL(IOV_RSSI_WINDOW_SZ):
		GET_GATE(IOV_RSSI_WINDOW_SZ);
		int_val = pi->rssi_ma_win_sz;
		bcopy(&int_val, a, vsize);
		break;
#endif /* STA */
#if LPCONF
	case IOV_SVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE):
		SET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE);
		pi->lpphy_rx_gain_temp_adj_tempsense = (int8)int_val;
		break;

	case IOV_GVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE):
		GET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE);
		int_val = (int32)pi->lpphy_rx_gain_temp_adj_tempsense;
		bcopy(&int_val, a, sizeof(int_val));
		break;

	case IOV_SVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH):
	  {
	    uint32 thresh = (uint32)int_val;
	    SET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH);
	    pi->lpphy_rx_gain_temp_adj_thresh[0] = (thresh & 0xff);
	    pi->lpphy_rx_gain_temp_adj_thresh[1] = ((thresh >> 8) & 0xff);
	    pi->lpphy_rx_gain_temp_adj_thresh[2] = ((thresh >> 16) & 0xff);
	    wlc_lpphy_rx_gain_temp_adj(pi);
	  }
	  break;

	case IOV_GVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH):
	  {
	    uint32 thresh;
	    GET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH);
	    thresh = (uint32)pi->lpphy_rx_gain_temp_adj_thresh[0];
	    thresh |= ((uint32)pi->lpphy_rx_gain_temp_adj_thresh[1])<<8;
	    thresh |= ((uint32)pi->lpphy_rx_gain_temp_adj_thresh[2])<<16;
	    bcopy(&thresh, a, sizeof(thresh));
	  }
	  break;

	case IOV_SVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC):
		SET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC);
		pi->lpphy_rx_gain_temp_adj_metric = (int8)(int_val & 0xff);
		pi->lpphy_rx_gain_temp_adj_tempsense_metric = (int8)((int_val >> 8) & 1);
		wlc_lpphy_rx_gain_temp_adj(pi);
		break;

	case IOV_GVAL(IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC):
		GET_GATE(IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC);
		int_val = (int32)pi->lpphy_rx_gain_temp_adj_metric;
		int_val |= (int32)((pi->lpphy_rx_gain_temp_adj_tempsense_metric << 8) & 0x100);
		bcopy(&int_val, a, sizeof(int_val));
		break;
#endif /* LPCONF */

	case IOV_SVAL(IOV_BT_FEM_COMBINER):
		SET_GATE(IOV_BT_FEM_COMBINER);
		pi->fem_combiner_target_state = int_val;
		if ((BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) >= BCM94319WINDSOR_SSID) &&
			(BOARDTYPE(GENERIC_PHY_INFO(pi)->sih->boardtype) <= BCM94319BHEMU3_SSID))
			wlc_load_bt_fem_combiner(pi, FALSE);
		break;
#ifdef WLMINIOCTL
default_label:
#endif
	default:
		err = BCME_UNSUPPORTED;
	}

	return err;
}

int
wlc_phy_ioctl(wlc_phy_t *pih, int cmd, int len, int *pval, bool bool_val, void *arg, bool *ta_ok)
{
	phy_info_t *pi = (phy_info_t *)pih;
	int bcmerror = 0;
	int val = pval ? *pval : 0;

	switch (cmd) {

#ifdef STA
	case WLC_GET_PHY_NOISE:
		GATE(WLC_GET_PHY_NOISE);
		*pval = wlc_phy_noise_avg(pih);
		break;
#endif

	case WLC_RESTART:
		GATE(WLC_RESTART);
		/* Reset calibration results to uninitialized state in order to
		 * trigger recalibration next time wlc_init() is called.
		 */
		if (GENERIC_PHY_INFO(pi)->up) {
			bcmerror = BCME_NOTDOWN;
			break;
		}
		wlc_set_phy_uninitted(pi);
		break;






	case WLC_GET_PWROUT_PERCENTAGE:
		GATE(WLC_GET_PWROUT_PERCENTAGE);
		*pval = GENERIC_PHY_INFO(pi)->txpwr_percent;
		break;

	case WLC_SET_PWROUT_PERCENTAGE:
		GATE(WLC_SET_PWROUT_PERCENTAGE);
		if ((uint)val > 100) {
			bcmerror = BCME_RANGE;
			break;
		}
		GENERIC_PHY_INFO(pi)->txpwr_percent = (uint8)val;
		if (GENERIC_PHY_INFO(pi)->up) {
			if (SCAN_IN_PROGRESS(pi->wlc)) {
				WL_TXPWR(("wl%d: Scan in progress, skipping txpower control\n",
					GENERIC_PHY_INFO(pi)->unit));
			} else {
				wlc_phy_txpower_recalc_target((wlc_phy_t *)pi, -1, NULL);
#if GCONF
				wlc_phy_cal_txpower_recalc_sw(pi);
#endif
			}
		}
		break;

	case WLC_GET_INTERFERENCE_MODE:
		GATE(WLC_GET_INTERFERENCE_MODE);
		*pval = pi->sh->interference_mode;
		if (pi->aci_state & ACI_ACTIVE)
			*pval |= AUTO_ACTIVE;
		break;

	case WLC_SET_INTERFERENCE_MODE:
		GATE(WLC_SET_INTERFERENCE_MODE);
		if (val < INTERFERE_NONE || val > WLAN_AUTO) {
			bcmerror = BCME_RANGE;
			break;
		}

		if (pi->sh->interference_mode == val)
			break;

		pi->sh->interference_mode = val;

		if (!GENERIC_PHY_INFO(pi)->up)
			break;

		WL_SUSPEND_MAC_AND_WAIT(pi);

		/* turn interference mode to off before entering another mode */
		if (val != INTERFERE_NONE)
			wlc_phy_interference(pi, INTERFERE_NONE, TRUE);

		if (!wlc_phy_interference(pi, pi->sh->interference_mode, TRUE))
			bcmerror = BCME_BADOPTION;

		WL_ENABLE_MAC(pi);
		break;

#ifdef WLMINIOCTL
#ifdef BCME_DISABLED
	default_label:
		bcmerror = BCME_DISABLED;
		break;
#else
	default_label:
		bcmerror = BCME_UNSUPPORTED;
		break;
#endif /* BCME_DISABLED */
#endif /* WLMINIOCTL */
	default:
		bcmerror = BCME_UNSUPPORTED;
	}

	return bcmerror;
}






#endif /* TBD_YE */

#define WAIT_FOR_SCOPE	4000000 /* in unit of us */

/* BCMATTACHFN like wlc_phy_txpwr_srom_read_nphy because it used exclusively by it. */
static void
wlc_sslpnphy_txpwr_srom_convert(uint8 *srom_max, uint16 *pwr_offset, uint8 tmp_max_pwr,
	uint8 rate_start, uint8 rate_end)
{
	uint8 rate;
	uint8 word_num, nibble_num;
	uint8 tmp_nibble;

	for (rate = rate_start; rate <= rate_end; rate++) {
		word_num = (rate - rate_start) >> 2;
		nibble_num = (rate - rate_start) & 0x3;
		tmp_nibble = (pwr_offset[word_num] >> 4 * nibble_num) & 0xf;
		/* nibble info indicates offset in 0.5dB units */
		srom_max[rate] = tmp_max_pwr - 2*tmp_nibble;
	}
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_setchan_cal)(phy_info_t *pi, int32 int_val)
{
#if PHYHAL
	phy_info_sslpnphy_t *sslpnphy_specific = pi->u.pi_sslpnphy;
#else
	wlc_info_t * wlc_pi = pi->wlc;
#endif /* PHYHAL */
	if (int_val != pi->radio_chanspec) {
		WL_ERROR(("%s: chanspec 0x%x != radio chanspec 0x%x\n",
		       __FUNCTION__, int_val, pi->radio_chanspec));
		return;
	}

	if (!(SCAN_IN_PROGRESS(wlc_pi) || WLC_RM_IN_PROGRESS(wlc_pi))) {
		uint freq;

		WL_INFORM(("%s : Doing a full cal for channel %d ",
		           __FUNCTION__, CHSPEC_CHANNEL(pi->radio_chanspec)));
		wlc_sslpnphy_full_cal(pi);
		freq = wlc_channel2freq(CHSPEC_CHANNEL(pi->radio_chanspec));
		if (sslpnphy_specific->sslpnphy_last_tx_freq != (uint16)freq) {
			int old_band, new_band;
			old_band = wlc_get_ssn_lp_band_range
			    (sslpnphy_specific->sslpnphy_last_tx_freq);
			new_band = wlc_get_band_range(pi, pi->radio_chanspec);
			if (old_band != new_band)
				wlc_sslpnphy_tx_pwr_ctrl_init(pi);
		}
		wlc_sslpnphy_papd_recal(pi);
		/* Tx iqlo calibration */
		wlc_sslpnphy_txpwrtbl_iqlo_cal(pi);
		WL_INFORM(("IOV_SSLPNPHY_SETCHAN_CAL: 0x%x\n", int_val));
	}
}

#if defined(DONGLEOVERLAYS)
void
BCMOVERLAYFN(1, wlc_sslpnphy_percal_iovar)(phy_info_t *pi, int32 int_val)
{
	if (int_val) {
		sslpnphy_specific->sslpnphy_force_1_idxcal = 1;
		sslpnphy_specific->sslpnphy_papd_nxt_cal_idx = int_val;
	}
	if (!NORADIO_ENAB(pi->pubpi))
		if (!(SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc) ||
		      PLT_IN_PROGRESS(pi->wlc) ||
		      ASSOC_IN_PROGRESS(pi->wlc) || pi->carrier_suppr_disable ||
		      pi->pkteng_in_progress || pi->disable_percal)) {
			wlc_sslpnphy_periodic_cal_top(pi);
			WL_INFORM(("IOV_SSLPNPHY_PER_CAL: %d\n", int_val));
		}
}

void
BCMOVERLAYFN(0, wlc_sslpnphy_noise_measure_iovar)(phy_info_t *pi)
{
	if (!NORADIO_ENAB(pi->pubpi))
		if (!(SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc) ||
		      PLT_IN_PROGRESS(pi->wlc) ||
		      ASSOC_IN_PROGRESS(pi->wlc) || pi->carrier_suppr_disable ||
		      pi->pkteng_in_progress || pi->disable_percal ||
		      sslpnphy_specific->sslpnphy_disable_noise_percal)) {
			wlc_sslpnphy_noise_measure(pi);
			WL_INFORM(("IOV_SSLPNPHY_NOISE_MEASURE\n"));
		}
}

void
BCMOVERLAYFN(1, wlc_sslpnphy_papd_recal_iovar)(phy_info_t *pi)
{
	if (!(SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc))) {
		wlc_sslpnphy_papd_recal(pi);
		/* Skip tx iq if init is happening on same channel (Time savings) */
		if (!sslpnphy_specific->sslpnphy_restore_papd_cal_results)
			wlc_sslpnphy_txpwrtbl_iqlo_cal(pi);
		WL_INFORM(("IOV_SSLPNPHY_PAPD_RECAL\n"));
#if defined(DONGLEOVERLAYS) && defined(BAND5G)
		/* need to init the other band at first init time */
		if (pi->phyinit_state == PHYINIT_STATE_GBAND) {
			int ret, val;
			val = WLC_BAND_5G;
			pi->phyinit_state = PHYINIT_STATE_ABAND;
			ret = wlc_send_overlay_event(pi->wlc, WLC_SET_BAND, IOCTL_OVERLAY,
			                             NULL, (void*)&val,
			                             sizeof(int), WLC_E_OVL_DOWNLOAD);
			if (ret)
				WL_ERROR(("wl%d: %s: wlc_send_overlay_event WLC_SET_BAND "
				          "failed w/status %d\n\n",
				          GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, ret));
		} else if (pi->phyinit_state == PHYINIT_STATE_ABAND) {
			int ret, val;
			val = WLC_BAND_AUTO;
			pi->phyinit_state = PHYINIT_STATE_DONE;
			ret = wlc_send_overlay_event(pi->wlc, WLC_SET_BAND, IOCTL_OVERLAY,
			                             NULL, (void*)&val,
			                             sizeof(int), WLC_E_OVL_DOWNLOAD);
			if (ret)
				WL_ERROR(("wl%d: %s: wlc_send_overlay_event WLC_SET_BAND "
				          "failed w/status %d\n\n",
				          GENERIC_PHY_INFO(pi)->unit, __FUNCTION__, ret));
		}
#endif /*  defined(DONGLEOVERLAYS) && defined(BAND5G) */
	} else {
		WL_INFORM((" %s : Not doing a full cal: Restoring the "
			"previous cal results for channel %d ", __FUNCTION__,
			sslpnphy_specific->sslpnphy_full_cal_channel[(CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0)]));
		sslpnphy_specific->sslpnphy_restore_papd_cal_results = 1;
		wlc_sslpnphy_papd_recal(pi);
	}
}
#endif 

#ifdef PHYHAL
int BCMFASTPATH
wlc_sslpnphy_rssi_compute(phy_info_t *pi, int rssi, d11rxhdr_t *rxh)
{
	phy_info_sslpnphy_t *ph = pi->u.pi_sslpnphy;
	uint8 gidx = (ltoh16(rxh->PhyRxStatus_2) & 0xFC00) >> 10;

	if (rssi > 127)
		rssi -= 256;

	/* RSSI adjustment */
	rssi = rssi + sslpnphy_gain_index_offset_for_pkt_rssi[gidx];
	if ((rssi > -46) && (gidx > 18))
		rssi = rssi + 7;

	/* temperature compensation */
	rssi = rssi + ph->sslpnphy_pkteng_rssi_slope;

	/* 2dB compensation of path loss for 4329 on Ref Boards */
	rssi = rssi + 2;

	return rssi;
}
void
wlc_sslpnphy_txpwr_target_adj(phy_info_t *pi, uint8 *tx_pwr_target, uint8 rate)
{

	uint8 cur_channel = CHSPEC_CHANNEL(pi->radio_chanspec); /* see wlioctl.h */
	phy_info_sslpnphy_t *pi_sslpn = pi->u.pi_sslpnphy;

	if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
#if defined(WLPLT)
		if (cur_channel == 7)	/* addressing corner lot power issues */
			tx_pwr_target[rate] = tx_pwr_target[rate] + 2;
		if (tx_pwr_target[rate] < 40) {
			tx_pwr_target[rate] = tx_pwr_target[rate] - 4;
		} else
#endif /* WLPLT */
		{
			if (rate > 11)
				tx_pwr_target[rate] = tx_pwr_target[rate] -
					pi_sslpn->sslpnphy_11n_backoff;
			else if ((rate >= 8) && (rate <= 11))
				tx_pwr_target[rate] = tx_pwr_target[rate] -
					pi_sslpn->sslpnphy_54_48_36_24mbps_backoff;
			else if (rate <= 3)
				tx_pwr_target[rate] = tx_pwr_target[rate] -
					pi_sslpn->sslpnphy_cck;
			else
				tx_pwr_target[rate] = tx_pwr_target[rate] -
					pi_sslpn->sslpnphy_lowerofdm;
		}
	} else if (SSLPNREV_IS(pi->pubpi.phy_rev, 4)) {

		if (cur_channel == 1) {
			if (rate > 3)
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 68);
			else
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 70);
		} else if (cur_channel == 11) {
			if (rate <= 3)
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 70);
			else
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 64);
		} else {
			if (rate <= 3)
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 72);
			else
				tx_pwr_target[rate] = MIN(tx_pwr_target[rate], 72);
		}
	}
}

void
wlc_sslpnphy_iovar_papd_debug(phy_info_t *pi, void *a)
{

	wl_sslpnphy_papd_debug_data_t papd_debug_data;
	phy_info_sslpnphy_t *ph = pi->u.pi_sslpnphy;
	papd_debug_data.psat_pwr = ph->sslpnphy_psat_pwr;
	papd_debug_data.psat_indx = ph->sslpnphy_psat_indx;
	papd_debug_data.min_phase = ph->sslpnphy_min_phase;
	papd_debug_data.final_idx = ph->sslpnphy_final_idx;
	papd_debug_data.start_idx = ph->sslpnphy_start_idx;

	bcopy(&papd_debug_data, a, sizeof(wl_sslpnphy_papd_debug_data_t));
}
#endif /* PHYHAL */
