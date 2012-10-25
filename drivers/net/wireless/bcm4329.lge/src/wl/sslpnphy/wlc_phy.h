/*
 * Required functions exported by the wlc_phy.c (phy-dependent)
 * to common (os-independent) driver code.
 *
 * Copyright (C) 2010, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id: wlc_phy.h,v 1.17 2010/09/01 21:04:41 Exp $
 */

#ifndef _wlc_phy_h_
#define _wlc_phy_h_

#include <wlc_phy_int.h>
/* Read/Write baseband attenuation */
#define	BBATT_11G_MASK2		0x3c	/* > ANA_11G_018 */
#define	BBATT_11G_SHIFT2	2       /* Shift 2 bits */
#define	BBATT_11G_MASK		0x78	/* ANA_11G_018 */
#define	BBATT_11G_SHIFT		3	/* Shift 3 bits */
#define	BBATT_11B_MASK		0xf	/* ANA_43XX */

/* Max Radio tx attenuation settings */
#define MAX_TX_ATTN_2050	9	/* 2050 */
#define MAX_TX_ATTN_2060	4	/* 2060 */
#define MAX_TX_ATTN_4318	0x1f	/* 4318 == radio rev 8 */

/* Max BB tx attenuation settings */
#define MAX_TX_ATTN		11	/* All analog FE revs */

/* Default tx attenuation settings */
#define DEF_TX_ATTN		2	/* Tx Attenuation setting */

#define PHY_NOISE_SAMPLE_MON	1	/* sample phy noise for watchdog */
#define PHY_NOISE_SAMPLE_SCAN	2	/* sample phy noise for scan */
#define PHY_NOISE_SAMPLE_CQRM	3	/* sample phy noise for CQ/RM */

#define PHY_PERICAL_DRIVERUP	1	/* periodic cal for driver up */
#define PHY_PERICAL_WATCHDOG	2	/* periodic cal for watchdog */
#define PHY_PERICAL_PHYINIT	3	/* periodic cal for phy_init */
#define PHY_PERICAL_JOIN_BSS	4	/* periodic cal for join BSS */
#define PHY_PERICAL_START_IBSS	5	/* periodic cal for join IBSS */
#define PHY_PERICAL_UP_BSS	6	/* periodic cal for up BSS */

#define PHY_PERICAL_DISABLE	0	/* periodic cal disabled */
#define PHY_PERICAL_SPHASE	1	/* periodic cal enabled, single phase only */
#define PHY_PERICAL_MPHASE	2	/* periodic cal enabled, can do multiphase */
#define PHY_PERICAL_MANUAL	3	/* disable periodic cal, only run it from iovar */

#define PHY_HOLD_FOR_ASSOC	1	/* hold PHY activities(like cal) during association */
#define PHY_HOLD_FOR_SCAN	2	/* hold PHY activities(like cal) during scan */
#define PHY_HOLD_FOR_RM		4	/* hold PHY activities(like cal) during radio measure */
#define PHY_HOLD_FOR_PLT	8	/* hold PHY activities(like cal) during plt */
#undef PHY_HOLD_FOR_MUTE
#define PHY_HOLD_FOR_MUTE	16
#define PHY_HOLD_FOR_NOT_ASSOC 0x20

/* NPHY rssidump and noisedump params */
#define NPHY_RSSI_WINDOW_SZ	16	/* NPHY rssidump window size */
#define NPHY_NOISE_WINDOW_SZ	16	/* NPHY noisedump window size */
#define NPHY_NOISE_FIXED_VAL 	(-92)	/* reported fixed noise */

/* LPPHY noisedump params */
#define LPPHY_NOISE_FIXED_VAL 	(-95)	/* reported fixed noise */

/* SSLPNPHY noisedump params */
#define SSLPNPHY_NOISE_FIXED_VAL 	(-95)	/* reported fixed noise */

/* phy_mode bit defs for high level phy mode state information */
#define PHY_MODE_ACI            0x0001  /* set if phy is in ACI mitigation mode */

#ifdef TBD_YE
#if defined(EXT_CBALL)
#define NORADIO_ENAB(pub) ((pub).radioid == NORADIO_ID)
#else
#define NORADIO_ENAB(pub) 0
#endif
#endif

#ifdef WL20MHZ_ONLY
#define WLC_PHY_GET_BW(ppi)	(WLC_20_MHZ)
#else
#define WLC_PHY_GET_BW(ppi)	(wlc_phy_get_bw(ppi))
#endif

#ifdef XTAL_FREQ
#define XTALFREQ(_freq)  XTAL_FREQ
#else
#define XTALFREQ(_freq)  _freq
#endif

#ifdef BOARD_TYPE
#define BOARDTYPE(_type) BOARD_TYPE
#else
#define BOARDTYPE(_type) _type
#endif

#ifdef BOARD_FLAGS
#define BOARDFLAGS(_flags) BOARD_FLAGS
#else
#define BOARDFLAGS(_flags) _flags
#endif

#ifdef OLYMPIC
#define IS_OLYMPIC(_pi) TRUE
#else 
#define IS_OLYMPIC(_pi) (_pi->sslpnphy_OLYMPIC)
#endif

#ifdef WLSINGLE_ANT
#define ANT_AVAIL(_ant) 1
#else
#define ANT_AVAIL(_ant) (_ant)
#endif

#define NPHY_TXPWRCTRL_OFF FALSE
#define NPHY_TXPWRCTRL_HW  TRUE


/*
 * Routines to support phy code
 */
extern void *wlc_phy_shared_attach(wlc_pub_t *pub);	/* different parameter moved*/
extern void wlc_phy_shared_detach(wlc_pub_t *pub, void *phy_sh); /* different parameter moved*/
extern void *wlc_phy_attach(wlc_pub_t *pub, void *regs, int bandtype, void *wlc, void *phy_sh); /* different parameter moved */
extern void wlc_phy_detach(wlc_phy_t *ppi);	/* moved */
extern void wlc_phy_init(wlc_phy_t *ppi);  /* different parameter moved */
extern void wlc_phy_reset(wlc_phy_t *ppi, si_t *sih); /* different parameter moved */
extern int wlc_phy_ioctl(wlc_phy_t *ppi, int cmd, int len, int *pval, bool bool_val,
                         void *arg, bool *ta_ok); /* different parameter */

#ifdef STA
extern void wlc_phy_BSSinit(wlc_phy_t *ppi, bool bonlyap, int rssi);	/* moved */
extern void wlc_phy_reset_rssi_ma(wlc_phy_t *pih); /* move to wlc_phy.c */
extern int wlc_phy_update_rssi_ma(wlc_phy_t *ppi, int nval); /* move to wlc_phy.c */
extern int8 wlc_phy_noise_avg(wlc_phy_t *wpi);	/* moved */
#endif

extern uint8 wlc_phy_radiocode2channel(uint8 radiocode, uint phytype); /* legacy not needed for TOT */
extern uint16 wlc_phy_channel2radiocode(uint16 channel, uint phytype); /* legacy not needed for TOT */

extern int wlc_phy_compute_rssi(wlc_phy_t *ppi, d11rxhdr_t *rxh); /* different name and parameter */


extern void wlc_phy_band_channels(wlc_phy_t *ppi, uint band, chanvec_t *channels); /* legacy not needed for TOT, stay */

#if !defined(BAND5G)  && defined(WL20MHZ_ONLY)
#define wlc_phy_band_first_chanspec(_pi, _bd) CH20MHZ_CHSPEC(1) /* legacy not needed for TOT */
#else
extern chanspec_t wlc_phy_band_first_chanspec(wlc_phy_t *ppi, uint band); /* legacy not needed for TOT */
#endif

extern void wlc_phy_cal_init(wlc_phy_t *ppi);	/* moved */

extern void wlc_phy_por_inform(wlc_phy_t *ppi);	/* moved */
extern void wlc_phy_power_on_reset_inform(wlc_phy_t *ppi);	/* moved */

extern void wlc_phy_chanspec_set(wlc_phy_t *ppi, chanspec_t chanspec);	/* moved */
extern chanspec_t wlc_phy_chanspec_get(wlc_phy_t *ppi);	/* moved */

extern void wlc_phy_set_chanspec(wlc_phy_t *ppi, chanspec_t chanspec);	/* moved */
extern chanspec_t wlc_phy_get_chanspec(wlc_phy_t *ppi);	/* moved */

extern int8 wlc_phy_bw_state_get(wlc_phy_t *ppi);	/* moved */
extern void wlc_phy_bw_state_set(wlc_phy_t *ppi, int8 bw);	/* moved */

extern int8 wlc_phy_get_bw(wlc_phy_t *ppi);	/* moved */
extern void wlc_phy_set_bw(wlc_phy_t *ppi, int8 bw); /* moved */

extern void wlc_phy_chanspec_radio_set(wlc_phy_t *ppi, chanspec_t newch); /* moved */

extern int8 wlc_phy_preamble_override_get(wlc_phy_t *ppi); /* moved */
extern void wlc_phy_preamble_override_set(wlc_phy_t *ppi, int8 override); /* moved */

extern int8 wlc_phy_get_n_preamble_override(wlc_phy_t *ppi); /* moved */
extern void wlc_phy_set_n_preamble_override(wlc_phy_t *ppi, int override); /* moved */

extern void wlc_phy_switch_radio(wlc_phy_t *ppi, bool on); /* BCMECI */

extern void wlc_phy_ant_rxdiv_set(wlc_phy_t *ppi, uint8 val); /* moved */
extern bool wlc_phy_ant_rxdiv_get(wlc_phy_t *ppi, uint8 *pval); /* moved */

extern void wlc_phy_superswitch_init_nphy(wlc_phy_t *ppi, bool lut_init); /* not needed */

extern void wlc_phy_clear_tssi(wlc_phy_t *ppi); /* moved */
extern void wlc_phy_hold_upd(wlc_phy_t *ppi, mbool id, bool val); /* moved */

/* power stuff */
extern void wlc_phy_txpower_sromlimit(wlc_phy_t *ppi, uint chan, uint8 *min_pwr, uint8 *max_pwr,
	int txp_rate_idx); /* wlc_chan_info */
extern void wlc_phy_txpower_sromlimit_max_get(wlc_phy_t *ppi, uint chan,
	uint8 *max_txpower, uint8 *min_txpower); /* moved */

extern void wlc_phy_txpower_boardlimit_band(wlc_phy_t *ppi, uint band,
	int32 *max_pwr, int32 *min_pwr, uint32 *step_pwr); /* moved */

extern void wlc_phy_bandtxpower_boardlimits(wlc_phy_t *ppi, uint band,
	int32 *max_pwr, int32 *min_pwr, uint32 *step_pwr); /* moved */

#if defined(WLCURPOWER)
extern void wlc_phy_txpower_get_current(wlc_phy_t *ppi, tx_power_t *power, uint channel); /* moved */
#endif

extern void wlc_phy_txpower_limit_set(wlc_phy_t *ppi, struct txpwr_limits*, chanspec_t chanspec); /* different parameter */
extern void wlc_phy_txpower_recalc_target(wlc_phy_t *ppi, int channel, uint8 *chan_txpwr); /* different parameter */
extern uint32 wlc_phy_txpower_get_target_min(wlc_phy_t *ppi); /* moved */
extern uint32 wlc_phy_txpower_get_target_max(wlc_phy_t *ppi); /* moved */
extern int wlc_phy_txpower_get(wlc_phy_t *ppi, uint *qdbm, bool *override); /* moved */
extern int wlc_phy_txpower_set(wlc_phy_t *ppi, uint qdbm, bool override);
extern void wlc_phy_txpower_target_set(wlc_phy_t *ppi, struct txpwr_limits *txpwr); /* different parameter */
extern bool wlc_phy_txpower_hw_ctrl_get(wlc_phy_t *ppi); /* moved */
extern void wlc_phy_txpower_hw_ctrl_set(wlc_phy_t *ppi, bool hwpwrctrl); /* moved */

#if defined(AP) && defined(RADAR)
extern void wlc_phy_radar_detect_init(wlc_phy_t *ppi, bool on); /* moved */
extern int wlc_phy_radar_detect_run(wlc_phy_t *ppi); /* moved */
#endif

#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
extern int wlc_phy_cal_cache_init(wlc_phy_t *ppi); /* moved */
extern void wlc_phy_cal_cache_deinit(wlc_phy_t *ppi); /* moved */
extern int wlc_phy_create_chanctx(wlc_phy_t *ppi, chanspec_t chanspec); /* moved */
extern void wlc_phy_destroy_chanctx(wlc_phy_t *ppi, chanspec_t chanspec); /* moved */
#endif

extern void wlc_phy_noise_sample_request(wlc_phy_t *ppi, uint8 reason, uint8 channel); /* different parameter */
extern void wlc_phy_noise_sample_intr(wlc_phy_t *ppi); /* different parameter */

extern void wlc_phy_freqtrack_start(wlc_phy_t *ppi); /* not applicable for SSLPNPHY */
extern void wlc_phy_freqtrack_end(wlc_phy_t *ppi); /* not applicable for SSLPNPHY */

extern void wlc_phy_anacore(wlc_phy_t *ppi, bool on); /* moved */

extern bool wlc_phy_bist_check_nphy(wlc_phy_t *ppi); /* not needed */
extern void wlc_phy_perical_nphy(wlc_phy_t *ppi, uint8 reason);	/* not needed */

extern void wlc_phy_noise_measure(wlc_phy_t *ppi); /* not yet in TOT */

//extern int8 wlc_phy_get_tx_power_offset_by_mcs(wlc_phy_t *ppi, uint8 mcs_offset, bool is40m); /* different parameter */

//extern int8 wlc_phy_get_tx_power_offset(wlc_phy_t *ppi, uint8 tbl_offset);

//extern int16 wlc_phy_get_mcs_txpwroffset(wlc_phy_t *ppi, uint8 offset);	/* obsolete */

//extern int16 wlc_phy_get_legacy_txpwroffset(wlc_phy_t *ppi, uint8 offset); /* obsolete */

#endif	/* _wlc_phy_h_ */
