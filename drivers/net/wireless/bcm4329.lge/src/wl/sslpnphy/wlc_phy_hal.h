/*
 * Required functions exported by the PHY module (phy-dependent)
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
 * $Id: wlc_phy_hal.h,v 1.3 2010/08/18 02:08:10 Exp $
 */

#ifndef _wlc_phy_hal_h_
#define _wlc_phy_hal_h_

#include <typedefs.h>
#include <bcmwifi.h>
#include <wlioctl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <d11.h>
//#include <wlc_phy_shim.h>

/* Radio macros */

/* Radio ID */
#define	IDCODE_VER_MASK		0x0000000f
#define	IDCODE_VER_SHIFT	0
#define	IDCODE_MFG_MASK		0x00000fff
#define	IDCODE_MFG_SHIFT	0
#define	IDCODE_ID_MASK		0x0ffff000
#define	IDCODE_ID_SHIFT		12
#define	IDCODE_REV_MASK		0xf0000000
#define	IDCODE_REV_SHIFT	28

#define	NORADIO_ID		0xe4f5
#define	NORADIO_IDCODE		0x4e4f5246

#define	BCM2050_ID		0x2050
#define	BCM2050_IDCODE		0x02050000
#define	BCM2050A0_IDCODE	0x1205017f
#define	BCM2050A1_IDCODE	0x2205017f
#define	BCM2050R8_IDCODE	0x8205017f

#define BCM2055_ID		0x2055
#define BCM2055_IDCODE		0x02055000
#define BCM2055A0_IDCODE	0x1205517f

#define BCM2056_ID		0x2056
#define BCM2056_IDCODE		0x02056000

#define BCM2057_ID		0x2057
#define BCM2057_IDCODE		0x02057000
#define BCM2057A0_IDCODE	0x1205717f

#define BCM2059_ID		0x2059
#define BCM2059A0_IDCODE	0x0205917f

#define	BCM2060_ID		0x2060
#define	BCM2060_IDCODE		0x02060000
#define	BCM2060WW_IDCODE	0x1206017f

#define BCM2062_ID		0x2062
#define BCM2062_IDCODE		0x02062000
#define BCM2062A0_IDCODE	0x0206217f

#define BCM2063_ID		0x2063
#define BCM2063_IDCODE		0x02063000
#define BCM2063A0_IDCODE	0x0206317f

#define BCM2064_ID		0x2064
#define BCM2064_IDCODE		0x02064000
#define BCM2064A0_IDCODE	0x0206417f

#define BCM2066_ID		0x2066
#define BCM2066_IDCODE		0x02066000
#define BCM2066A0_IDCODE	0x0206617f

/* PHY macros */
#define PHY_TPC_HW_OFF		FALSE
#define PHY_TPC_HW_ON		TRUE

#define PHY_PERICAL_DRIVERUP	1	/* periodic cal for driver up */
#define PHY_PERICAL_WATCHDOG	2	/* periodic cal for watchdog */
#define PHY_PERICAL_PHYINIT	3	/* periodic cal for phy_init */
#define PHY_PERICAL_JOIN_BSS	4	/* periodic cal for join BSS */
#define PHY_PERICAL_START_IBSS	5	/* periodic cal for join IBSS */
#define PHY_PERICAL_UP_BSS	6	/* periodic cal for up BSS */
#define PHY_PERICAL_CHAN	7 	/* periodic cal for channel change */
#define PHY_FULLCAL	8		/* full calibration routine */
#define PHY_PAPDCAL	10		/* papd calibration routine */

#define PHY_PERICAL_DISABLE	0	/* periodic cal disabled */
#define PHY_PERICAL_SPHASE	1	/* periodic cal enabled, single phase only */
#define PHY_PERICAL_MPHASE	2	/* periodic cal enabled, can do multiphase */
#define PHY_PERICAL_MANUAL	3	/* disable periodic cal, only run it from iovar */

#define PHY_HOLD_FOR_ASSOC	1	/* hold PHY activities(like cal) during association */
#define PHY_HOLD_FOR_SCAN	2	/* hold PHY activities(like cal) during scan */
#define PHY_HOLD_FOR_RM		4	/* hold PHY activities(like cal) during radio measure */
#define PHY_HOLD_FOR_PLT	8	/* hold PHY activities(like cal) during plt */
#define PHY_HOLD_FOR_MUTE		0x10
#define PHY_HOLD_FOR_NOT_ASSOC	0x20
#define PHY_HOLD_FOR_ACI_SCAN	0x40 /* hold PHY activities(like cal) during ACI scan */

#define PHY_MUTE_FOR_PREISM	1
#define PHY_MUTE_ALL		0xffffffff

/* Fixed Noise for APHY/BPHY/GPHY */
#define PHY_NOISE_FIXED_VAL 		(-95)	/* reported fixed noise */
#define PHY_NOISE_FIXED_VAL_NPHY       	(-92)	/* reported fixed noise */
#define PHY_NOISE_FIXED_VAL_LCNPHY     	(-92)	/* reported fixed noise */

/* phy_mode bit defs for high level phy mode state information */
#define PHY_MODE_ACI		0x0001	/* set if phy is in ACI mitigation mode */
#define PHY_MODE_CAL		0x0002	/* set if phy is in Calibration mode */
#define PHY_MODE_NOISEM		0x0004	/* set if phy is in Noise Measurement mode */

#define WLC_TXPWR_DB_FACTOR	4	/* most txpower parameters are in 1/4 dB unit */


typedef enum  phy_radar_detect_mode {
	RADAR_DETECT_MODE_FCC,
	RADAR_DETECT_MODE_EU,
	RADAR_DETECT_MODE_MAX
} phy_radar_detect_mode_t;

/* forward declaration */
struct rpc_info;

typedef struct shared_phy shared_phy_t;

/* Public phy info structure */
struct phy_pub;

#ifdef WLC_HIGH_ONLY
typedef struct wlc_rpc_phy wlc_phy_t;
#else
typedef struct phy_pub wlc_phy_t;
#endif

typedef struct shared_phy_params {
	void 	*osh;		/* pointer to OS handle */
	si_t	*sih;		/* si handle (cookie for siutils calls) */
	void	*physhim;	/* phy <-> wl shim layer for wlapi */
	uint	unit;		/* device instance number */
	uint	corerev;	/* core revision */
	uint	bustype;	/* SI_BUS, PCI_BUS  */
	uint	buscorerev; 	/* buscore rev */
	char	*vars;		/* phy attach time only copy of vars */
	uint16	vid;		/* vendorid */
	uint16	did;		/* deviceid */
	uint	chip;		/* chip number */
	uint	chiprev;	/* chip revision */
	uint	chippkg;	/* chip package option */
	uint	sromrev;	/* srom revision */
	uint	boardtype;	/* board type */
	uint	boardrev;	/* board revision */
	uint	boardvendor;	/* board vendor */
	uint32	boardflags;	/* board specific flags from srom */
	uint32	boardflags2;	/* more board flags if sromrev >= 4 */
} shared_phy_params_t;

#ifdef WLC_LOW	    /* BMAC driver interface */

/* attach/detach/init/deinit */
extern shared_phy_t *wlc_phy_shared_attach(shared_phy_params_t *shp);
extern void  wlc_phy_shared_detach(shared_phy_t *phy_sh);
extern wlc_phy_t *wlc_phy_attach(shared_phy_t *sh, void *regs, int bandtype, char *vars);
extern void  wlc_phy_detach(wlc_phy_t *ppi);

extern bool wlc_phy_get_phyversion(wlc_phy_t *pih, uint16 *phytype, uint16 *phyrev,
	uint16 *radioid, uint16 *radiover);
extern bool wlc_phy_get_encore(wlc_phy_t *pih);
extern uint32 wlc_phy_get_coreflags(wlc_phy_t *pih);

extern void  wlc_phy_hw_clk_state_upd(wlc_phy_t *ppi, bool newstate);
extern void  wlc_phy_hw_state_upd(wlc_phy_t *ppi, bool newstate);
extern void  wlc_phy_init(wlc_phy_t *ppi, chanspec_t chanspec);
extern int32  wlc_phy_watchdog(wlc_phy_t *ppi);
extern int   wlc_phy_down(wlc_phy_t *ppi);
extern uint32 wlc_phy_clk_bwbits(wlc_phy_t *pih);
extern void wlc_phy_cal_init(wlc_phy_t *ppi);
extern void wlc_phy_antsel_init(wlc_phy_t *ppi, bool lut_init);

/* bandwidth/chanspec */
extern void wlc_phy_chanspec_set(wlc_phy_t *ppi, chanspec_t chanspec);
extern chanspec_t wlc_phy_chanspec_get(wlc_phy_t *ppi);
extern void wlc_phy_chanspec_radio_set(wlc_phy_t *ppi, chanspec_t newch);
extern uint16 wlc_phy_bw_state_get(wlc_phy_t *ppi);
extern void wlc_phy_bw_state_set(wlc_phy_t *ppi, uint16 bw);

/* states sync */
extern void wlc_phy_rssi_compute(wlc_phy_t *pih, void *ctx);
extern void wlc_phy_por_inform(wlc_phy_t *ppi);
extern void wlc_phy_noise_sample_intr(wlc_phy_t *ppi);
extern bool wlc_phy_bist_check_phy(wlc_phy_t *ppi);

extern void wlc_phy_set_deaf(wlc_phy_t *ppi, bool user_flag);
extern void wlc_phy_clear_deaf(wlc_phy_t *ppi, bool user_flag);

extern void wlc_phy_switch_radio(wlc_phy_t *ppi, bool on);
extern void wlc_phy_anacore(wlc_phy_t *ppi, bool on);

#endif /* WLC_LOW */

/* flow */
extern void wlc_phy_BSSinit(wlc_phy_t *ppi, bool bonlyap, int rssi);
#ifdef TBD_YE
extern int  wlc_phy_ioctl(wlc_phy_t *ppi, int cmd, int len, void *arg, bool *ta_ok);

extern int  wlc_phy_iovar_dispatch(wlc_phy_t *ppi, uint32 id, uint16, void *, uint, void *,
	int, int);

/* chanspec */
extern void wlc_phy_chanspec_ch14_widefilter_set(wlc_phy_t *ppi, bool wide_filter);
extern void wlc_phy_chanspec_band_validch(wlc_phy_t *ppi, uint band, chanvec_t *channels);
extern chanspec_t wlc_phy_chanspec_band_firstch(wlc_phy_t *ppi, uint band);

extern void wlc_phy_txpower_sromlimit(wlc_phy_t *ppi, uint chan,
	uint8 *_min_, uint8 *_max_, int rate);
extern void wlc_phy_txpower_sromlimit_max_get(wlc_phy_t *ppi, uint chan,
	uint8 *_max_, uint8 *_min_);
extern void wlc_phy_txpower_boardlimit_band(wlc_phy_t *ppi, uint band, int32 *, int32 *, uint32 *);

extern void wlc_phy_txpower_limit_set(wlc_phy_t *ppi, txppr_t* txpwr, chanspec_t chanspec);
extern void wlc_phy_txpower_target_set(wlc_phy_t *ppi, txppr_t* txpwr);
#endif
extern int  wlc_phy_txpower_get(wlc_phy_t *ppi, uint *qdbm, bool *override);
extern int  wlc_phy_txpower_set(wlc_phy_t *ppi, uint qdbm, bool override);

extern bool wlc_phy_txpower_hw_ctrl_get(wlc_phy_t *ppi);
extern void wlc_phy_txpower_hw_ctrl_set(wlc_phy_t *ppi, bool hwpwrctrl);
#ifdef TBD_YE
extern uint8 wlc_phy_txpower_get_target_min(wlc_phy_t *ppi);
extern uint8 wlc_phy_txpower_get_target_max(wlc_phy_t *ppi);
#endif
extern bool wlc_phy_txpower_ipa_ison(wlc_phy_t *pih);

extern void wlc_phy_stf_chain_init(wlc_phy_t *pih, uint8 txchain, uint8 rxchain);
extern void wlc_phy_stf_chain_set(wlc_phy_t *pih, uint8 txchain, uint8 rxchain);
extern void wlc_phy_stf_chain_get(wlc_phy_t *pih, uint8 *txchain, uint8 *rxchain);
extern uint8 wlc_phy_stf_chain_active_get(wlc_phy_t *pih);
extern int8 wlc_phy_stf_ssmode_get(wlc_phy_t *pih, chanspec_t chanspec);
extern void wlc_phy_ldpc_override_set(wlc_phy_t *ppi, bool val);

extern void wlc_phy_cal_perical(wlc_phy_t *ppi, uint8 reason);
extern int8 wlc_phy_noise_avg(wlc_phy_t *wpi);
extern void wlc_phy_noise_sample_request_external(wlc_phy_t *ppi);
extern void wlc_phy_interference_set(wlc_phy_t *ppi, bool init);
extern void wlc_phy_acimode_noisemode_reset(wlc_phy_t *ppi, uint channel,
	bool clear_aci_state, bool clear_noise_state, bool disassoc);
extern void wlc_phy_edcrs_lock(wlc_phy_t *pih, bool lock);
extern void wlc_phy_cal_papd_recal(wlc_phy_t *ppi);

/* misc */
extern int8 wlc_phy_preamble_override_get(wlc_phy_t *ppi);
extern void wlc_phy_preamble_override_set(wlc_phy_t *ppi, int8 override);
extern void wlc_phy_ant_rxdiv_set(wlc_phy_t *ppi, uint8 val);
extern bool wlc_phy_ant_rxdiv_get(wlc_phy_t *ppi, uint8 *pval);
extern void wlc_phy_clear_tssi(wlc_phy_t *ppi);
extern void wlc_phy_hold_upd(wlc_phy_t *ppi, mbool id, bool val);
extern bool wlc_phy_ismuted(wlc_phy_t *pih);
extern void wlc_phy_mute_upd(wlc_phy_t *ppi, bool val, mbool flags);
extern bool wlc_phy_get_tempsense_degree(wlc_phy_t *ppi, int8 *pval);

extern void wlc_phy_radar_detect_enable(wlc_phy_t *ppi, bool on);
extern int  wlc_phy_radar_detect_run(wlc_phy_t *ppi);
extern void wlc_phy_radar_detect_mode_set(wlc_phy_t *pih, phy_radar_detect_mode_t mode);

extern void wlc_phy_antsel_type_set(wlc_phy_t *ppi, uint8 antsel_type);

#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
extern int  wlc_phy_cal_cache_init(wlc_phy_t *ppi);
extern void wlc_phy_cal_cache_deinit(wlc_phy_t *ppi);
extern int wlc_phy_create_chanctx(wlc_phy_t *ppi, chanspec_t chanspec);
extern void wlc_phy_destroy_chanctx(wlc_phy_t *ppi, chanspec_t chanspec);
#endif

#ifdef PHYCAL_CACHING
extern void wlc_phy_cal_cache_set(wlc_phy_t *ppi, bool state);
extern bool wlc_phy_cal_cache_get(wlc_phy_t *ppi);
#endif

extern void wlc_phy_txpower_get_current(wlc_phy_t *ppi, tx_power_t *power, uint channel);

extern void wlc_phy_initcal_enable(wlc_phy_t *pih, bool initcal);
extern bool wlc_phy_test_ison(wlc_phy_t *ppi);
extern void wlc_phy_txpwr_percent_set(wlc_phy_t *ppi, uint8 txpwr_percent);
extern void wlc_phy_ofdm_rateset_war(wlc_phy_t *pih, bool war);
extern void wlc_phy_bf_preempt_enable(wlc_phy_t *pih, bool bf_preempt);
extern void wlc_phy_machwcap_set(wlc_phy_t *ppi, uint32 machwcap);

extern void wlc_phy_runbist_config(wlc_phy_t *ppi, bool start_end);

/* old GPHY stuff */
extern void wlc_phy_freqtrack_start(wlc_phy_t *ppi);
extern void wlc_phy_freqtrack_end(wlc_phy_t *ppi);

extern const uint8 * wlc_phy_get_ofdm_rate_lookup(void);

extern void wlc_phy_tkip_rifs_war(wlc_phy_t *ppi, uint8 rifs);
#ifdef TBD_YE
extern int8 wlc_phy_get_tx_power_offset_by_mcs(wlc_phy_t *ppi, uint8 mcs_offset);
#endif
extern int8 wlc_phy_get_tx_power_offset(wlc_phy_t *ppi, uint8 tbl_offset);
extern void wlc_phy_btclock_war(wlc_phy_t *ppi, bool enable);


#endif	/* _wlc_phy_hal_ */
