/*
 * PHY and RADIO specific portion of Broadcom BCM43XX 802.11 Networking Device Driver.
 *
 * Copyright (C) 2010, Broadcom Corporation
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 *
 * $Id: wlc_phy_cmn.c,v 1.25 2010/11/08 21:15:53 Exp $
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
#include <bitfuncs.h>
#include <bcmdevs.h>
#include <bcmnvram.h>
#include <proto/802.11.h>
#include <sbhndpio.h>
#include <sbhnddma.h>
#include <hnddma.h>
#include <sbchipc.h>
#include <hndpmu.h>
#include <bcmsrom_fmt.h>
#include <sbsprom.h>
#include <bcm20xx.h>
#include <d11.h>
#include <wlc_rate.h>
#include <wlc_key.h>
#include <wlc_channel.h>
#include <wlc_pub.h>
#include <wlc_bsscfg.h>
#include <wl_dbg.h>
#include <wlc.h>
#include <wlc_phy_hal.h>
#include <wlc_phy.h>
#include <wlc_phy_int.h>
#include <sslpnphyregs.h>
#include <lpphyregs.h>
#include <wl_export.h>
#include <wlc_event.h>
#include <wl_minioctl.h>

#include <bcmwifi.h>
#include <bcmotp.h>


#ifdef WLNOKIA_NVMEM
#include <wlc_phy_noknvmem.h>
#endif /* WLNOKIA_NVMEM */
#include <wl_dbg.h>

void wlc_phy_do_dummy_tx(phy_info_t *pi, bool ofdm, bool pa_on);
/* forward declaration */
extern bool wlc_btcparam_index_to_shmem_offset(wlc_info_t *wlc, uint32 *pval);
#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
int wlc_phy_cal_cache_restore(phy_info_t *pih);
/* Get the calcache entry given the chanspec */
ch_calcache_t *wlc_phy_get_chanctx(phy_info_t *phi, chanspec_t chanspec);
void wlc_phydump_cal_cache_nphy(phy_info_t *pih, ch_calcache_t *ctx, struct bcmstrbuf *b);
#endif



uint wlc_phy_channel2freq(uint channel);
uint wlc_phy_channel2idx(uint channel);
#ifdef BAND5G
int wlc_get_band_range(phy_info_t*, chanspec_t);
#else
#define wlc_get_band_range(_pi, _ch) WL_CHAN_FREQ_RANGE_2G
#endif

void wlc_set_phy_uninitted(phy_info_t *pi);

void wlc_phy_watchdog(void *wlc);
int wlc_phy_down(void *wlc);
bool wlc_phy_interference(phy_info_t *pi, int wanted_mode, bool init);
int wlc_phy_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid,
	const char *name, void *p, uint plen, void *a, int alen, int vsize,
	struct wlc_if *wlcif);

uint32 wlc_phy_rx_iq_est(phy_info_t *pi, bool rxiq_agc_en, uint8 rxiq_log2_samps, uint8 rxiq_num_iter, uint8 resolution);
uint32 wlc_calc_log(uint32 power);
uint32 wlc_calc_log_fine_resln(uint32 power);

/* redefine some wlc_cfg.h macros to take the internal phy_info_t instead of wlc_phy_t */
#undef ISAPHY
#undef ISGPHY
#undef ISNPHY
#undef ISLPPHY
#undef ISSSLPNPHY
#define ISAPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_A)
#define ISGPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_G)
#define ISNPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_N)
#define ISLPPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_LP)
#define ISSSLPNPHY(pi)  PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_SSN)


#undef IS20MHZ
#undef IS40MHZ
#ifdef WL20MHZ_ONLY
#define IS20MHZ(pi) (TRUE)
#define IS40MHZ(pi) (FALSE)
#else
#define IS20MHZ(pi)	((pi)->bw == WLC_20_MHZ)
#define IS40MHZ(pi)	((pi)->bw == WLC_40_MHZ)
#endif

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
#undef TXP_NUM_RATES
#define TXP_NUM_RATES		45	/* number of rates */
#define ACI_ACTIVE	(1 << 1)	/* enabled either manually or automatically */
#define ACI_AUTO	(1 << 2)	/* Auto or manual */
#define SW_TIMER_FAST		15	/* 15 second timeout */
#define SW_TIMER_SLOW		60	/* 60 second timeout */
#define SW_TIMER_GLACIAL	120	/* 120 second timeout */
#define PHY_NOISE_STATE_MON	0x1
#define PHY_NOISE_STATE_SCAN	0x2
#define PHY_NOISE_STATE_CQRM	0x4
#define PHY_NOISE_SAMPLE_LOG_NUM_NPHY	10
#define PHY_NOISE_SAMPLE_LOG_NUM_UCODE	9	/* ucode uses smaller value to speed up process */

#define SSLPNPHY_TX_PWR_CTRL_OFF	0
#define SSLPNPHY_TX_PWR_CTRL_SW SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK
#define SSLPNPHY_TX_PWR_CTRL_HW \
	(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
	SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK | \
	SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK)

/* SSLPNPHY has different sub-band range limts for the A-band compared to MIMOPHY
 * (see sslpnphy_get_paparams in sslpnphyprocs.tcl)
 */
#define FIRST_LOW_5G_CHAN_SSLPNPHY      34
#define LAST_LOW_5G_CHAN_SSLPNPHY       64
#define FIRST_MID_5G_CHAN_SSLPNPHY      100
#define LAST_MID_5G_CHAN_SSLPNPHY       140
#define FIRST_HIGH_5G_CHAN_SSLPNPHY     149
#define LAST_HIGH_5G_CHAN_SSLPNPHY      165

#define MACSTATOFF(name) ((uint)((char *)(&wlc->pub._cnt.name) - (char *)(&wlc->pub._cnt.txallfrm)))

#define SCAN_IN_PROGRESS(wlc)	(wlc_scan_inprog(wlc))
#define WLC_RM_IN_PROGRESS(wlc)	(wlc_rminprog(wlc))
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
/* Turn off all the crs signals to the MAC */
#define wlc_sslpnphy_set_deaf(pi)	wlc_sslpnphy_deaf_mode(pi, TRUE)

/* Restore all the crs signals to the MAC */
#define wlc_sslpnphy_clear_deaf(pi)	 wlc_sslpnphy_deaf_mode(pi, FALSE)

#define wlc_sslpnphy_tssi_enabled(pi) \
	(SSLPNPHY_TX_PWR_CTRL_OFF != wlc_sslpnphy_get_tx_pwr_ctrl((pi)))
#define wlc_sslpnphy_get_tx_pwr_ctrl(pi) \
	(read_phy_reg((pi), SSLPNPHY_TxPwrCtrlCmd) & \
			(SSLPNPHY_TxPwrCtrlCmd_txPwrCtrl_en_MASK | \
			SSLPNPHY_TxPwrCtrlCmd_hwtxPwrCtrl_en_MASK | \
			SSLPNPHY_TxPwrCtrlCmd_use_txPwrCtrlCoefs_MASK))

/* BTC notifications */
#define wlc_phy_update_bt_chanspec(pi, chanspec)

#define BCMECICOEX_ENAB_PHY(pi)         0
#define BCMECISECICOEX_ENAB_PHY(pi)	0

const uint8 ofdm_rate_lookup[] = {
            /* signal */
        WLC_RATE_48M, /* 8: 48Mbps */
        WLC_RATE_24M, /* 9: 24Mbps */
        WLC_RATE_12M, /* A: 12Mbps */
        WLC_RATE_6M,  /* B:  6Mbps */
        WLC_RATE_54M, /* C: 54Mbps */
        WLC_RATE_36M, /* D: 36Mbps */
        WLC_RATE_18M, /* E: 18Mbps */
        WLC_RATE_9M   /* F:  9Mbps */
};


/* iovar table */
enum {
	IOV_QTXPOWER,
	IOV_RADAR_ARGS, 	/* radar detection parameter control */
	IOV_RADAR_ARGS_40MHZ, 	/* radar detection parameter control */
	IOV_FAST_TIMER,
	IOV_SLOW_TIMER,
	IOV_GLACIAL_TIMER,
	IOV_TXINSTPWR,
	IOV_PHY_WATCHDOG,
	IOV_PHY_RSSI_ANT,
	IOV_PHYNOISE_POLL,
	IOV_NPHY_SCRAMINIT,
	IOV_NPHY_RFSEQ,
	IOV_NPHY_TXIQLOCAL,
	IOV_NPHY_RXIQCAL,
	IOV_NPHY_TXPWRCTRL,
	IOV_NPHY_TXPWRINDEX,
	IOV_NPHY_RSSISEL,
	IOV_NPHY_RSSICAL,
	IOV_NPHY_GPIOSEL,
	IOV_NPHY_TX_TONE,
	IOV_NPHY_PREAMBLE,
	IOV_NPHY_TXRX_CHAIN,
	IOV_NPHY_FIXED_NOISE,
	IOV_NPHY_GAIN_BOOST,
	IOV_NPHY_ELNA_GAIN_CONFIG,
	IOV_NPHY_TEST_TSSI,
	IOV_NPHY_TEST_TSSI_OFFS,
	IOV_NPHY_RXCALPARAMS,
	IOV_NPHY_5G_PWRGAIN,
	IOV_NPHY_PERICAL,
	IOV_NPHY_FORCEPERCAL,
	IOV_NPHY_ACI_SCAN,
	IOV_NPHY_INITGAIN,
	IOV_NPHY_TEMPSENSE,
	IOV_NPHY_CAL_SANITY,
	IOV_NPHY_BPHY_EVM,
	IOV_NPHY_BPHY_RFCS,
	IOV_NPHY_HPVGA1GAIN,
	IOV_LPPHY_TX_TONE,
	IOV_LPPHY_TXIQLOCAL,
	IOV_LPPHY_RXIQCAL,
	IOV_LPPHY_FULLCAL,
	IOV_LPPHY_PAPDCAL,
	IOV_LPPHY_PAPDCALTYPE,
	IOV_LPPHY_TXPWRCTRL,
	IOV_LPPHY_TXPWRINDEX,
	IOV_LPPHY_CRS,
	IOV_LPPHY_PAPD_SLOW_CAL,
	IOV_LPPHY_PAPD_RECAL_GAIN_DELTA,
	IOV_LPPHY_PAPD_RECAL_MAX_INTERVAL,
	IOV_LPPHY_PAPD_RECAL_MIN_INTERVAL,
	IOV_LPPHY_PAPD_RECAL_ENABLE,
	IOV_LPPHY_PAPD_RECAL_COUNTER,
	IOV_LPPHY_CCK_DIG_FILT_TYPE,
	IOV_LPPHY_NOISE_SAMPLES,
	IOV_CARRIER_SUPPRESS,
	IOV_DISABLE_PERCAL,
	IOV_UNMOD_RSSI,
	IOV_PKTENG,
	IOV_PKTENG_TMO,
	IOV_PKTENG_STATS,
	IOV_PHY_RXIQ_EST,
	IOV_LPPHY_PAPDEPSTBL,
	IOV_LPPHY_TEMPSENSE,
	IOV_LPPHY_CAL_DELTA_TEMP,
	IOV_LPPHY_VBATSENSE,
	IOV_LPPHY_TXIQCC,
	IOV_LPPHY_TXLOCC,
	IOV_PHYTABLE,
	IOV_LPPHY_IDLE_TSSI_UPDATE_DELTA_TEMP,
	IOV_LPPHY_ACI_ON_THRESH,
	IOV_LPPHY_ACI_OFF_THRESH,
	IOV_LPPHY_ACI_ON_TIMEOUT,
	IOV_LPPHY_ACI_OFF_TIMEOUT,
	IOV_LPPHY_ACI_GLITCH_TIMEOUT,
	IOV_LPPHY_ACI_CHAN_SCAN_CNT,
	IOV_LPPHY_ACI_CHAN_SCAN_PWR_THRESH,
	IOV_LPPHY_ACI_CHAN_SCAN_CNT_THRESH,
	IOV_LPPHY_ACI_CHAN_SCAN_TIMEOUT,
	IOV_SSLPNPHY_CCKFILTSEL,
	IOV_SSLPNPHY_PAPD_DEBUG,
	IOV_SSLPNPHY_PAPD_DUMPLUT,
	IOV_SSLPNPHY_TX_TONE,
	IOV_SSLPNPHY_PAPDCAL,
	IOV_SSLPNPHY_TXIQLOCAL,
	IOV_SSLPNPHY_TXPWRCTRL,
	IOV_SSLPNPHY_VBATSENSE,
	IOV_SSLPNPHY_TEMPSENSE,
	IOV_SSLPNPHY_RCPI,
	IOV_SSLPNPHY_SNR,
#if defined(DONGLEOVERLAYS)
	IOV_SSLPNPHY_PERCAL_DEBUG,
	IOV_SSLPNPHY_VCO_CAL,
	IOV_SSLPNPHY_TXIQCAL,
	IOV_SSLPNPHY_PAPD_CAL,
	IOV_SSLPNPHY_RXIQCAL,
	IOV_SSLPNPHY_TXPWRINIT,
	IOV_SSLPNPHY_TXPWRCACHE,
	IOV_SSLPNPHY_TXPWRCACHE_CH,
	IOV_SSLPNPHY_PER_CAL,
	IOV_SSLPNPHY_FULL_CAL,
	IOV_SSLPNPHY_SETCHAN_CAL,
	IOV_SSLPNPHY_PAPD_RECAL,
#endif 
	IOV_SSLPNPHY_FORCECAL,
	IOV_SSLPNPHY_NOISE_MEASURE,
	IOV_SSLPNPHY_DISABLE_NOISE_PERCAL,
	IOV_SSLPNPHY_FORCE_NOISE_CAL,
	IOV_SSLPNPHY_FORCE_LGAIN,
	IOV_SSLPNPHY_FORCE_RXPO,
	IOV_SSLPNPHY_SPBRUN,
	IOV_SSLPNPHY_SPBDUMP,
	IOV_SSLPNPHY_SPBRANGE,
	IOV_SSLPNPHY_REPORT_SPBDUMP_PWR,
	IOV_SSLPNPHY_REPORT_SPBDUMP_PWR_RAW,
	IOV_SSLPNPHY_SAMPLE_COLLECT_AGC_GAIN,
	IOV_SSLPNPHY_TXPWRINDEX,
	IOV_SSLPNPHY_AUXADC,
	IOV_SSLPNPHY_IDLETSSI,
	IOV_SSLPNPHY_CRS,
	IOV_SSLPNPHY_NOISE_SAMPLES,
	IOV_SSLPNPHY_CARRIER_SUPPRESS,
	IOV_SSLPNPHY_UNMOD_RSSI,
	IOV_SSLPNPHY_PKTENG_STATS,
	IOV_SSLPNPHY_PKTENG,
	IOV_SSLPNPHY_PAPARAMS,
	IOV_SSLPNPHY_FULLCAL,
	IOV_SSLPNPHY_CGA,
	IOV_SSLPNPHY_CGA_5G,
	IOV_SSLPNPHY_CGA_2G,
	IOV_SSLPNPHY_TX_IQCC,
	IOV_SSLPNPHY_TX_LOCC,
	IOV_PAVARS,
	IOV_NUM_STREAM,
	IOV_BAND_RANGE,
	IOV_LPPHY_OFDM_DIG_FILT_TYPE,
	IOV_LPPHY_TXRF_SP_9_OVR,
	IOV_RSSI_WINDOW_SZ,
	IOV_LPPHY_PAPARAMS,
	IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE,
	IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH,
	IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC,
	IOV_OFDM_ANALOG_FILT_BW_OVERRIDE,
	IOV_CCK_ANALOG_FILT_BW_OVERRIDE,
	IOV_OFDM_RCCAL_OVERRIDE,
	IOV_CCK_RCCAL_OVERRIDE,
	IOV_BT_FEM_COMBINER
};

const bcm_iovar_t phy_iovars[] = {
#if IOV_SUPPORTED(IOV_QTXPOWER)
	{"qtxpower", IOV_QTXPOWER,
	(IOVF_WHL), IOVT_UINT32, 0
	},	/* constructed in wlu.c with txpwr or txpwr1 */
#endif /* IOV_QTXPOWER */
#if IOV_SUPPORTED(IOV_PHY_RSSI_ANT)
	{"phy_rssi_ant", IOV_PHY_RSSI_ANT,
	(0), IOVT_BUFFER, sizeof(wl_rssi_ant_t)
	},
#endif /* IOV_PHY_RSSI_ANT */

#if defined(WLTIMER)
#if IOV_SUPPORTED(IOV_FAST_TIMER)
	{"fast_timer", IOV_FAST_TIMER,
	(IOVF_NTRL), IOVT_UINT32, 0
	},
#endif /* IOV_FAST_TIMER */
#if IOV_SUPPORTED(IOV_SLOW_TIMER)
	{"slow_timer", IOV_SLOW_TIMER,
	(IOVF_NTRL), IOVT_UINT32, 0
	},
#endif /* IOV_SLOW_TIMER */
#if IOV_SUPPORTED(IOV_GLACIAL_TIMER)
	{"glacial_timer", IOV_GLACIAL_TIMER,
	(IOVF_NTRL), IOVT_UINT32, 0
	},
#endif /* IOV_GLACIAL_TIMER */
#endif 
#if defined(WLNINTENDO2DBG)
#if IOV_SUPPORTED(IOV_PHY_WATCHDOG)
	{"phy_watchdog", IOV_PHY_WATCHDOG,
	(0), IOVT_BOOL, 0
	},
#endif /* IOV_PHY_WATCHDOG */
#if IOV_SUPPORTED(IOV_PHYNOISE_POLL)
	{"phynoise_polling", IOV_PHYNOISE_POLL,
	(0), IOVT_BOOL, 0
	},
#endif /* IOV_PHYNOISE_POLL */
#if IOV_SUPPORTED(IOV_CARRIER_SUPPRESS)
	{"carrier_suppress", IOV_CARRIER_SUPPRESS,
	IOVF_SET_UP, IOVT_BOOL, 0
	},
#endif /* IOV_CARRIER_SUPPRESS */
#if IOV_SUPPORTED(IOV_DISABLE_PERCAL)
	{"disable_percal", IOV_DISABLE_PERCAL,
	0, IOVT_BOOL, 0
	},
#endif /* IOV_DISABLE_PERCAL */
#if IOV_SUPPORTED(IOV_UNMOD_RSSI)
	{"unmod_rssi", IOV_UNMOD_RSSI,
	0, IOVT_INT32, 0
	},
#endif /* IOV_UNMOD_RSSI */
#if IOV_SUPPORTED(IOV_PKTENG)
	{"pkteng", IOV_PKTENG,
	IOVF_SET_UP, IOVT_BUFFER, sizeof(wl_pkteng_t)
	},
#endif /* IOV_PKTENG */
#if defined(LPCONF) || defined(SSLPNCONF)
#if IOV_SUPPORTED(IOV_PKTENG_TMO)
	{"pkteng_tmo", IOV_PKTENG_TMO,
	(IOVF_WHL), IOVT_INT32, 0
	},
#endif /* IOV_PKTENG_TMO */
#endif /* LPCONF  || SSLPNCONF */
#if IOV_SUPPORTED(IOV_PKTENG_STATS)
	{"pkteng_stats", IOV_PKTENG_STATS,
	IOVF_GET_UP, IOVT_BUFFER, sizeof(wl_pkteng_stats_t)
	},
#endif /* IOV_PKTENG_STATS */
#if IOV_SUPPORTED(IOV_PHYTABLE)
	{"phytable", IOV_PHYTABLE,
	IOVF_GET_UP | IOVF_SET_UP, IOVT_BUFFER, 4*4
	},
#endif /* IOV_PHYTABLE */
#if IOV_SUPPORTED(IOV_PAVARS)
	{"pavars", IOV_PAVARS,
	IOVF_SET_DOWN, IOVT_BUFFER, WL_PHY_PAVARS_LEN * sizeof(uint16)
	},
#endif /* IOV_PAVARS */

#if SSLPNCONF
#if IOV_SUPPORTED(IOV_SSLPNPHY_CCKFILTSEL)
	{"sslpnphy_cckfiltsel", IOV_SSLPNPHY_CCKFILTSEL,
	0, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_CCKFILTSEL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TX_TONE)
	{"sslpnphy_tx_tone", IOV_SSLPNPHY_TX_TONE,
	0, IOVT_INT32, 0
	},
#endif /* IOV_SSLPNPHY_TX_TONE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_NOISE_MEASURE)
	{"sslpnphy_noise_measure", IOV_SSLPNPHY_NOISE_MEASURE,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
#endif /* IOV_SSLPNPHY_NOISE_MEASURE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_DISABLE_NOISE_PERCAL)
	{"sslpnphy_disable_noise_percal", IOV_SSLPNPHY_DISABLE_NOISE_PERCAL,
	0, IOVT_BOOL, 0
	},
#endif /* IOV_SSLPNPHY_DISABLE_NOISE_PERCAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_RCPI)
	{"sslpnphy_rcpi", IOV_SSLPNPHY_RCPI,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_SSLPNPHY_RCPI */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TEMPSENSE)
	{"sslpnphy_tempsense", IOV_SSLPNPHY_TEMPSENSE,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_SSLPNPHY_TEMPSENSE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_VBATSENSE)
	{"sslpnphy_vbatsense", IOV_SSLPNPHY_VBATSENSE,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_SSLPNPHY_VBATSENSE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_SPBRUN)
	{"sslpnphy_spbrun", IOV_SSLPNPHY_SPBRUN,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
#endif /* IOV_SSLPNPHY_SPBRUN */
#if IOV_SUPPORTED(IOV_SSLPNPHY_SPBDUMP)
	{"sslpnphy_spbdump", IOV_SSLPNPHY_SPBDUMP,
	IOVF_GET_UP, IOVT_BUFFER, sizeof(wl_sslpnphy_spbdump_data_t)
	},
#endif /* IOV_SSLPNPHY_SPBDUMP */
#if IOV_SUPPORTED(IOV_SSLPNPHY_SPBRANGE)
	{"sslpnphy_spbrange", IOV_SSLPNPHY_SPBRANGE,
	IOVF_SET_UP, IOVT_BUFFER, 2*sizeof(uint16)
	},
#endif /* IOV_SSLPNPHY_SPBRANGE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_REPORT_SPBDUMP_PWR)
	{"sslpnphy_report_spbdump_pwr", IOV_SSLPNPHY_REPORT_SPBDUMP_PWR,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_SSLPNPHY_REPORT_SPBDUMP_PWR */
#if IOV_SUPPORTED(IOV_SSLPNPHY_REPORT_SPBDUMP_PWR_RAW)
	{"sslpnphy_report_spbdump_pwr_raw", IOV_SSLPNPHY_REPORT_SPBDUMP_PWR_RAW,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_REPORT_SPBDUMP_PWR_RAW */
#if IOV_SUPPORTED(IOV_SSLPNPHY_SAMPLE_COLLECT_AGC_GAIN)
	{"sslpnphy_sample_collect_agc_gain", IOV_SSLPNPHY_SAMPLE_COLLECT_AGC_GAIN,
	IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_SAMPLE_COLLECT_AGC_GAIN */
#if IOV_SUPPORTED(IOV_SSLPNPHY_PAPD_DEBUG)
	{"sslpnphy_papd_debug", IOV_SSLPNPHY_PAPD_DEBUG,
	IOVF_GET_UP, IOVT_BUFFER, sizeof(wl_sslpnphy_papd_debug_data_t)
	},
#endif /* IOV_SSLPNPHY_PAPD_DEBUG */
#if IOV_SUPPORTED(IOV_SSLPNPHY_PAPD_DUMPLUT)
	{"sslpnphy_papd_dumplut", IOV_SSLPNPHY_PAPD_DUMPLUT,
	IOVF_GET_UP, IOVT_BUFFER, sizeof(wl_sslpnphy_debug_data_t)
	},
#endif /* IOV_SSLPNPHY_PAPD_DUMPLUT */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TXPWRINDEX)
	{"sslpnphy_txpwrindex", IOV_SSLPNPHY_TXPWRINDEX,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_TXPWRINDEX */
#if IOV_SUPPORTED(IOV_SSLPNPHY_AUXADC)
	{"sslpnphy_auxadc", IOV_SSLPNPHY_AUXADC,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_AUXADC */
#if IOV_SUPPORTED(IOV_SSLPNPHY_IDLETSSI)
	{"sslpnphy_idletssi", IOV_SSLPNPHY_IDLETSSI,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_IDLETSSI */
#if IOV_SUPPORTED(IOV_SSLPNPHY_PAPARAMS)
	{"sslpnphy_paparams", IOV_SSLPNPHY_PAPARAMS,
	IOVF_SET_UP, IOVT_BUFFER, 3*sizeof(int32)
	},
#endif /* IOV_SSLPNPHY_PAPARAMS */
#if IOV_SUPPORTED(IOV_SSLPNPHY_FULLCAL)
	{"sslpnphy_fullcal", IOV_SSLPNPHY_FULLCAL,
	IOVF_GET_UP, IOVT_BUFFER, 0
	},
#endif /* IOV_SSLPNPHY_FULLCAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_CGA)
	{"sslpnphy_cga", IOV_SSLPNPHY_CGA,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_CGA */
#if IOV_SUPPORTED(IOV_SSLPNPHY_CGA_5G)
	{"sslpnphy_cga_5g", IOV_SSLPNPHY_CGA_5G,
	IOVF_SET_UP, IOVT_BUFFER, 24*sizeof(int8)
	},
#endif /* IOV_SSLPNPHY_CGA_5G */
#if IOV_SUPPORTED(IOV_SSLPNPHY_CGA_2G)
	{"sslpnphy_cga_2g", IOV_SSLPNPHY_CGA_2G,
	IOVF_SET_UP, IOVT_BUFFER, 14*sizeof(int8)
	},
#endif /* IOV_SSLPNPHY_CGA_2G */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TX_IQCC)
	{"sslpnphy_tx_iqcc", IOV_SSLPNPHY_TX_IQCC,
	IOVF_SET_UP, IOVT_BUFFER, 2*sizeof(int8)
	},
#endif /* IOV_SSLPNPHY_TX_IQCC */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TX_LOCC)
	{"sslpnphy_tx_locc", IOV_SSLPNPHY_TX_LOCC,
	IOVF_SET_UP, IOVT_BUFFER, 6*sizeof(int8)
	},
#endif /* IOV_SSLPNPHY_TX_LOCC */
#endif /* SSLPNCONF */
#if IOV_SUPPORTED(IOV_NPHY_5G_PWRGAIN)
	{"nphy_5g_pwrgain", IOV_NPHY_5G_PWRGAIN,
	(IOVF_SET_DOWN), IOVT_UINT8, 0
	},
#endif /* IOV_NPHY_5G_PWRGAIN */
#endif	
#if IOV_SUPPORTED(IOV_SSLPNPHY_FORCE_NOISE_CAL)
	{"sslpnphy_force_noise_cal", IOV_SSLPNPHY_FORCE_NOISE_CAL,
	0, IOVT_BOOL, 0
	},
#endif /* IOV_SSLPNPHY_FORCE_NOISE_CAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_FORCE_LGAIN)
	{"sslpnphy_force_lgain", IOV_SSLPNPHY_FORCE_LGAIN,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_FORCE_LGAIN */
#if IOV_SUPPORTED(IOV_SSLPNPHY_FORCE_RXPO)
	{"sslpnphy_force_rxpo", IOV_SSLPNPHY_FORCE_RXPO,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_SSLPNPHY_FORCE_RXPO */
#if IOV_SUPPORTED(IOV_NPHY_PREAMBLE)
	{"mimo_preamble", IOV_NPHY_PREAMBLE,
	(IOVF_SET_DOWN), IOVT_INT8, 0
	},
#endif /* IOV_NPHY_PREAMBLE */
#if NCONF	    /* move some to internal ?? */
#if IOV_SUPPORTED(IOV_NPHY_TXRX_CHAIN)
	{"nphy_txrx_chain", IOV_NPHY_TXRX_CHAIN,
	(0), IOVT_INT8, 0
	},
#endif /* IOV_NPHY_TXRX_CHAIN */
#if IOV_SUPPORTED(IOV_NPHY_TEMPSENSE)
	{"nphy_tempsense", IOV_NPHY_TEMPSENSE,
	IOVF_GET_UP, IOVT_INT8, 0
	},
#endif /* IOV_NPHY_TEMPSENSE */
#if IOV_SUPPORTED(IOV_NPHY_CAL_SANITY)
	{"nphy_cal_sanity", IOV_NPHY_CAL_SANITY,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_NPHY_CAL_SANITY */
#endif /* NCONF */
#if IOV_SUPPORTED(IOV_PHY_RXIQ_EST)
	{"phy_rxiqest", IOV_PHY_RXIQ_EST,
	IOVF_SET_UP | IOVF_GET_UP, IOVT_UINT32, IOVT_UINT32
	},
#endif /* IOV_PHY_RXIQ_EST */
#if IOV_SUPPORTED(IOV_NUM_STREAM)
	{"num_stream", IOV_NUM_STREAM,
	(0), IOVT_INT32, 0
	},
#endif /* IOV_NUM_STREAM */
#if IOV_SUPPORTED(IOV_BAND_RANGE)
	{"band_range", IOV_BAND_RANGE,
	0, IOVT_INT8, 0
	},
#endif /* IOV_BAND_RANGE */
#ifdef STA
#if IOV_SUPPORTED(IOV_RSSI_WINDOW_SZ)
	{"rssi_win", IOV_RSSI_WINDOW_SZ,
	(0), IOVT_UINT16, 0
	},
#endif /* IOV_RSSI_WINDOW_SZ */
#endif /* STA */
#if LPCONF
#if IOV_SUPPORTED(IOV_LPPHY_TEMPSENSE)
	{"lpphy_tempsense", IOV_LPPHY_TEMPSENSE,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_LPPHY_TEMPSENSE */
#if IOV_SUPPORTED(IOV_LPPHY_CAL_DELTA_TEMP)
	{"lpphy_cal_delta_temp", IOV_LPPHY_CAL_DELTA_TEMP,
	0, IOVT_INT32, 0
	},
#endif /* IOV_LPPHY_CAL_DELTA_TEMP */
#if IOV_SUPPORTED(IOV_LPPHY_VBATSENSE)
	{"lpphy_vbatsense", IOV_LPPHY_VBATSENSE,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* IOV_LPPHY_VBATSENSE */
#if IOV_SUPPORTED(IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE)
	{"lpphy_rx_gain_temp_adj_tempsense", IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE,
	(0), IOVT_INT8, 0
	},
#endif /* IOV_LPPHY_RX_GAIN_TEMP_ADJ_TEMPSENSE */
#if IOV_SUPPORTED(IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH)
	{"lpphy_rx_gain_temp_adj_thresh", IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH,
	(0), IOVT_UINT32, 0
	},
#endif /* IOV_LPPHY_RX_GAIN_TEMP_ADJ_THRESH */
#if IOV_SUPPORTED(IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC)
	{"lpphy_rx_gain_temp_adj_metric", IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC,
	(0), IOVT_INT16, 0
	},
#endif /* IOV_LPPHY_RX_GAIN_TEMP_ADJ_METRIC */
#endif /* LPCONF */
#if SSLPNCONF
#if defined(DONGLEOVERLAYS)
#if IOV_SUPPORTED(IOV_SSLPNPHY_PER_CAL)
	{"sslpnphy_percal", IOV_SSLPNPHY_PER_CAL,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
#endif /* IOV_SSLPNPHY_PER_CAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_FULL_CAL)
	{"sslpnphy_fullcal", IOV_SSLPNPHY_FULL_CAL,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_FULL_CAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_SETCHAN_CAL)
	{"sslpnphy_setchancal", IOV_SSLPNPHY_SETCHAN_CAL,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_SETCHAN_CAL */
#if IOV_SUPPORTED(IOV_SSLPNPHY_TXPWRINIT)
	{"sslpnphy_txpwr_init", IOV_SSLPNPHY_TXPWRINIT,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_TXPWRINIT */
#if IOV_SUPPORTED(IOV_SSLPNPHY_NOISE_MEASURE)
	{"sslpnphy_noise_measure", IOV_SSLPNPHY_NOISE_MEASURE,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_NOISE_MEASURE */
#if IOV_SUPPORTED(IOV_SSLPNPHY_PAPD_RECAL)
	{"sslpnphy_papd_recal", IOV_SSLPNPHY_PAPD_RECAL,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
#endif /* IOV_SSLPNPHY_PAPD_RECAL */
#endif 
#endif /* SSLPNCONF */
#if IOV_SUPPORTED(IOV_BT_FEM_COMBINER)
	{"bt_fem_combiner", IOV_BT_FEM_COMBINER,
	IOVF_SET_UP, IOVT_INT8, 0
	},
#endif /* IOV_BT_FEM_COMBINER */
	{NULL, 0, 0, 0, 0 }
};

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
		if (pi->pub->up && (ISAPHY(pi) || ISLPPHY(pi) || ISNPHY(pi) || ISSSLPNPHY(pi)))
			wlc_phy_radar_detect_init((wlc_phy_t *)pi, pi->pub->radar != 0);
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
		if (pi->pub->up)
			wlc_phy_radar_detect_init((wlc_phy_t *)pi, pi->pub->radar != 0);
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


#if defined(WLNINTENDO2DBG)
	case IOV_GVAL(IOV_PHY_WATCHDOG):
		GET_GATE(IOV_PHY_WATCHDOG);
		*ret_int_ptr = (int32)pi->watchdog_override;
		break;

	case IOV_SVAL(IOV_PHY_WATCHDOG):
		SET_GATE(IOV_PHY_WATCHDOG);
		pi->watchdog_override = bool_val;
		break;

	case IOV_GVAL(IOV_PHYNOISE_POLL):
		GET_GATE(IOV_PHYNOISE_POLL);
		*ret_int_ptr = (int32)pi->phynoise_polling;
		break;

	case IOV_SVAL(IOV_PHYNOISE_POLL):
		SET_GATE(IOV_PHYNOISE_POLL);
		pi->phynoise_polling = bool_val;
		break;

#endif 

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
			if (pi->pub->up) {
				wlc_suspend_mac_and_wait(pi->wlc);
				wlc_phyreg_enter((wlc_phy_t *)pi);
				wlc_nphy_update_txrx_chain(pi);
				wlc_phy_force_rfseq_nphy(pi, NPHY_RFSEQ_RESET2RX);
				wlc_phyreg_exit((wlc_phy_t *)pi);
				wlc_enable_mac(pi->wlc);
			}
		}
		break;
	}


	case IOV_GVAL(IOV_NPHY_TEMPSENSE):
		GET_GATE(IOV_NPHY_TEMPSENSE);
		wlc_suspend_mac_and_wait(pi->wlc);
		wlc_phyreg_enter((wlc_phy_t *)pi);
		*ret_int_ptr = (int32)wlc_nphy_tempsense(pi);
		wlc_phyreg_exit((wlc_phy_t *)pi);
		wlc_enable_mac(pi->wlc);
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
		*ret_int_ptr = (int32)pi->sslpnphy_force_noise_cal;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_NOISE_CAL):
		SET_GATE(IOV_SSLPNPHY_FORCE_NOISE_CAL);
		pi->sslpnphy_disable_noise_percal = bool_val;
		pi->sslpnphy_force_noise_cal = bool_val;
		if (pi->sslpnphy_force_noise_cal) {
			mod_phy_reg(pi, SSLPNPHY_HiGainDB,
				SSLPNPHY_HiGainDB_HiGainDB_MASK,
				pi->sslpnphy_force_lgain << SSLPNPHY_HiGainDB_HiGainDB_SHIFT);
			mod_phy_reg(pi, SSLPNPHY_InputPowerDB,
				SSLPNPHY_InputPowerDB_inputpwroffsetdb_MASK,
				pi->sslpnphy_force_rxpo <<
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
		*ret_int_ptr = (int8)pi->sslpnphy_force_lgain;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_LGAIN):
		SET_GATE(IOV_SSLPNPHY_FORCE_LGAIN);
		pi->sslpnphy_force_lgain = (int8)int_val;
		break;
	case IOV_GVAL(IOV_SSLPNPHY_FORCE_RXPO):
		GET_GATE(IOV_SSLPNPHY_FORCE_RXPO);
		*ret_int_ptr = (int8)pi->sslpnphy_force_rxpo;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_FORCE_RXPO):
		SET_GATE(IOV_SSLPNPHY_FORCE_RXPO);
		pi->sslpnphy_force_rxpo = (int8)int_val;
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
		pi->sslpnphy_recal = 1;
		wlc_sslpnphy_tx_pwr_ctrl_init(pi);
		pi->sslpnphy_recal = 0;
		break;
	case IOV_SVAL(IOV_SSLPNPHY_NOISE_MEASURE):
		SET_GATE(IOV_SSLPNPHY_NOISE_MEASURE);
		wlc_sslpnphy_noise_measure_iovar(pi);
		break;
	case IOV_SVAL(IOV_SSLPNPHY_PAPD_RECAL):
		SET_GATE(IOV_SSLPNPHY_PAPD_RECAL);
		wlc_sslpnphy_papd_recal_iovar(pi);
		break;

	case IOV_GVAL(IOV_PHY_RXIQ_EST):
	{
		GET_GATE(IOV_PHY_RXIQ_EST);
		{
			if (!pi->pub->up) {
				err = BCME_NOTUP;
				break;
			}
			wlc_suspend_mac_and_wait(pi->wlc);
			wlc_phyreg_enter((wlc_phy_t *)pi);
			WL_TRACE(("GET IOV_PHY_RXIQ_EST: agc_en = %d log2_samps = %d num_iter = %d resolution = %d\n",
				pi->rxiq_agc_en, pi->rxiq_log2_samps, pi->rxiq_num_iter, pi->phy_rxiq_resln));
			/* get IQ power measurements */
			*ret_int_ptr = wlc_phy_rx_iq_est(pi, pi->rxiq_agc_en, pi->rxiq_log2_samps,
					pi->rxiq_num_iter, pi->phy_rxiq_resln);
			wlc_phyreg_exit((wlc_phy_t *)pi);
			wlc_enable_mac(pi->wlc);

		}
		break;
	}

	case IOV_SVAL(IOV_PHY_RXIQ_EST):
		SET_GATE(IOV_PHY_RXIQ_EST);
		{
			uint8 rxiq_log2_samps,rxiq_num_iter, resolution;
			bool rxiq_agc_en;

			rxiq_log2_samps = int_val >> 8;
			rxiq_num_iter = 1;//int_val >> 8;
			rxiq_agc_en = 0;//(int_val & 0x1fff) >> 12;
			resolution = 1;//(int_val >> 16) & 0xff;

			WL_TRACE(("SET IOV_PHY_RXIQ_EST: agc_en = %d log2_samps = %d num_iter = %d resolution = %d\n",
				rxiq_agc_en, rxiq_log2_samps, rxiq_num_iter, resolution));

                        if ((resolution != 0) && (resolution != 1)) {
                                err = BCME_RANGE;
                                break;
                        }

			if (rxiq_log2_samps < 5 || rxiq_log2_samps > 15) {
				err = BCME_RANGE;
				break;
			}

			pi->rxiq_log2_samps = rxiq_log2_samps;
			pi->rxiq_num_iter = rxiq_num_iter;
			pi->rxiq_agc_en = rxiq_agc_en;
			pi->phy_rxiq_resln = resolution;
		}
		break;
#endif 
#endif /* SSLPNCONF */
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
#ifdef PALM
	case IOV_SVAL(IOV_BT_FEM_COMBINER):
		SET_GATE(IOV_BT_FEM_COMBINER);
		pi->fem_combiner_target_state = int_val;
		if ((BOARDTYPE(pi->pub->sih->boardtype) >= BCM94319WINDSOR_SSID) &&
			(BOARDTYPE(pi->pub->sih->boardtype) <= BCM94319BHEMU3_SSID))
			wlc_load_bt_fem_combiner_sslpnphy(pi, FALSE);
		break;
#endif /* PALM */
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
		if (pi->pub->up) {
			bcmerror = BCME_NOTDOWN;
			break;
		}
		wlc_set_phy_uninitted(pi);
		break;






	case WLC_GET_PWROUT_PERCENTAGE:
		GATE(WLC_GET_PWROUT_PERCENTAGE);
		*pval = pi->pub->txpwr_percent;
		break;

	case WLC_SET_PWROUT_PERCENTAGE:
		GATE(WLC_SET_PWROUT_PERCENTAGE);
		if ((uint)val > 100) {
			bcmerror = BCME_RANGE;
			break;
		}
		pi->pub->txpwr_percent = (uint8)val;
		if (pi->pub->up) {
			if (SCAN_IN_PROGRESS(pi->wlc)) {
				WL_TXPWR(("wl%d: Scan in progress, skipping txpower control\n",
					pi->pub->unit));
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

		if (!pi->pub->up)
			break;

		wlc_suspend_mac_and_wait(pi->wlc);

		/* turn interference mode to off before entering another mode */
		if (val != INTERFERE_NONE)
			wlc_phy_interference(pi, INTERFERE_NONE, TRUE);

		if (!wlc_phy_interference(pi, pi->sh->interference_mode, TRUE))
			bcmerror = BCME_BADOPTION;

		wlc_enable_mac(pi->wlc);
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


struct chan_info;
typedef struct chan_info chan_info_t;

#if defined(BCM4329B1)
/* channel info structure */
struct chan_info {
	uint16	freq;		/* in Mhz */
	uint8	chan;		/* channel number */
};
const chan_info_t wlc_phy_chan_info[] = {
	/* 11b/11g */
/* 0 */		{2412,	1},
/* 1 */		{2417,	2},
/* 2 */		{2422,	3},
/* 3 */		{2427,	4},
/* 4 */		{2432,	5},
/* 5 */		{2437,	6},
/* 6 */		{2442,	7},
/* 7 */		{2447,	8},
/* 8 */		{2452,	9},
/* 9 */		{2457,	10},
/* 10 */	{2462,	11},
/* 11 */	{2467,	12},
/* 12 */	{2472,	13},
/* 13 */	{2484,	14},

#ifdef BAND5G
/* 11a japan high */
/* The 0x80 bit in pdiv means these are REF5, other entries are REF20 */
/* 14 */	{5170,	34},
/* 15 */	{5190,	38},
/* 16 */	{5210,	42},
/* 17 */	{5230,	46},

/* 11a usa low */
/* 18 */	{5180,	36},
/* 19 */	{5200,	40},
/* 20 */	{5220,	44},
/* 21 */	{5240,	48},
/* 22 */	{5260,	52},
/* 23 */	{5280,	56},
/* 24 */	{5300,	60},
/* 25 */	{5320,	64},

/* 11a Europe */
/* 26 */	{5500,	100},
/* 27 */	{5520,	104},
/* 28 */	{5540,	108},
/* 29 */	{5560,	112},
/* 30 */	{5580,	116},
/* 31 */	{5600,	120},
/* 32 */	{5620,	124},
/* 33 */	{5640,	128},
/* 34 */	{5660,	132},
/* 35 */	{5680,	136},
/* 36 */	{5700,	140},

/* 11a usa high, ref5 only */
/* The 0x80 bit in pdiv means these are REF5, other entries are REF20 */
/* 37 */	{5745,	149},
/* 38 */	{5765,	153},
/* 39 */	{5785,	157},
/* 40 */	{5805,	161},
/* 41 */	{5825,	165},

/* 11a japan */
/* 42 */	{4920,	184},
/* 43 */	{4940,	188},
/* 44 */	{4960,	192},
/* 45 */	{4980,	196},
/* 46 */	{5000,	200},
/* 47 */	{5020,	204},
/* 48 */	{5040,	208},
/* 49 */	{5060,	212},
/* 50 */	{5080,	216},
#endif /* BAND5G */
};

#else /* !BCM4329B1 */

/* channel info structure */
struct chan_info {
	uint	chan;		/* channel number */
	uint	freq;		/* in Mhz */
	uint16	radiocode;	/* radio-chip-specific code for this channel */
	uint8	pdiv;		/* pdiv for 2060WW */
	uint8	sdiv;		/* sdiv for 2060WW */
	uint8	cal_val;	/* cal_val for 2060WW */
	uint8	rxiqidx;	/* Index into rx iq compensation table for 2060WW */
	uint8	txiqidx;	/* Index into tx iq compensation table for 2060WW */
	uint8	hwtxpwr;	/* Hardware limit for TX power for 2060WW */
	int8	pwr_est_delta;	/* Board dep output power estimate adj. */
};

const chan_info_t wlc_phy_chan_info[] = {
	/* 11b/11g */
/* 0 */		{1,	2412,	12,	0,	0,	0,	0,	0,	0,	0},
/* 1 */		{2,	2417,	17,	0,	0,	0,	0,	0,	0,	0},
/* 2 */		{3,	2422,	22,	0,	0,	0,	0,	0,	0,	0},
/* 3 */		{4,	2427,	27,	0,	0,	0,	0,	0,	0,	0},
/* 4 */		{5,	2432,	32,	0,	0,	0,	0,	0,	0,	0},
/* 5 */		{6,	2437,	37,	0,	0,	0,	0,	0,	0,	0},
/* 6 */		{7,	2442,	42,	0,	0,	0,	0,	0,	0,	0},
/* 7 */		{8,	2447,	47,	0,	0,	0,	0,	0,	0,	0},
/* 8 */		{9,	2452,	52,	0,	0,	0,	0,	0,	0,	0},
/* 9 */		{10,	2457,	57,	0,	0,	0,	0,	0,	0,	0},
/* 10 */	{11,	2462,	62,	0,	0,	0,	0,	0,	0,	0},
/* 11 */	{12,	2467,	67,	0,	0,	0,	0,	0,	0,	0},
/* 12 */	{13,	2472,	72,	0,	0,	0,	0,	0,	0,	0},
/* 13 */	{14,	2484,	84,	0,	0,	0,	0,	0,	0,	0},

#ifdef BAND5G
/* 11a japan high */
/* The 0x80 bit in pdiv means these are REF5, other entries are REF20 */
/* 14 */	{34,	5170,	0, 0x80 + 65,	27,	0x17,	0,	0,	56,	0},
/* 15 */	{38,	5190,	0, 0x80 + 65,	31,	0x17,	0,	0,	56,	0},
/* 16 */	{42,	5210,	0, 0x80 + 65,	35,	0x17,	0,	0,	56,	0},
/* 17 */	{46,	5230,	0, 0x80 + 65,	39,	0x17,	0,	0,	56,	0},

/* 11a usa low */
/* 18 */	{36,	5180,	0,	15,	2,	0x17,	0,	0,	56,	0},
/* 19 */	{40,	5200,	0,	15,	3,	0x17,	0,	0,	56,	0},
/* 20 */	{44,	5220,	0,	15,	4,	0x17,	0,	0,	56,	0},
/* 21 */	{48,	5240,	0,	15,	5,	0x17,	0,	0,	56,	0},
/* 22 */	{52,	5260,	0,	15,	6,	0x16,	0,	0,	56,	0},
/* 23 */	{56,	5280,	0,	15,	7,	0x16,	0,	0,	56,	0},
/* 24 */	{60,	5300,	0,	15,	8,	0x16,	0,	0,	56,	0},
/* 25 */	{64,	5320,	0,	15,	9,	0x16,	0,	0,	56,	0},

/* 11a Europe */
/* 26 */	{100,	5500,	0,	16,	3,	0x99,	1,	1,	52,	2},
/* 27 */	{104,	5520,	0,	16,	4,	0x99,	1,	1,	52,	2},
/* 28 */	{108,	5540,	0,	16,	5,	0x99,	1,	1,	52,	2},
/* 29 */	{112,	5560,	0,	16,	6,	0x98,	1,	1,	52,	2},
/* 30 */	{116,	5580,	0,	16,	7,	0x98,	1,	1,	52,	2},
/* 31 */	{120,	5600,	0,	16,	8,	0x98,	1,	1,	52,	2},
/* 32 */	{124,	5620,	0,	16,	9,	0x97,	1,	1,	52,	2},
/* 33 */	{128,	5640,	0,	16,	10,	0x97,	1,	1,	52,	2},
/* 34 */	{132,	5660,	0,	16,	11,	0x97,	1,	0,	52,	2},
/* 35 */	{136,	5680,	0,	16,	12,	0x96,	1,	0,	52,	2},
/* 36 */	{140,	5700,	0,	16,	13,	0x96,	1,	0,	52,	2},

/* 11a usa high, ref5 only */
/* The 0x80 bit in pdiv means these are REF5, other entries are REF20 */
/* 37 */	{149,	5745,	0, 0x80 + 73,	22,	0x95,	1,	0,	52,	2},
/* 38 */	{153,	5765,	0, 0x80 + 73,	26,	0x95,	4,	0,	52,	2},
/* 39 */	{157,	5785,	0, 0x80 + 73,	30,	0x95,	4,	0,	52,	2},
/* 40 */	{161,	5805,	0, 0x80 + 73,	34,	0x95,	4,	0,	52,	2},
/* 41 */	{165,	5825,	0, 0x80 + 73,	38,	0x95,	4,	0,	52,	2},

/* 11a japan */
/* 42 */	{184,	4920,	0,	14,	4,	0x57,	2,	1,	56,	0},
/* 43 */	{188,	4940,	0,	14,	5,	0x57,	2,	1,	56,	0},
/* 44 */	{192,	4960,	0,	14,	6,	0x57,	2,	1,	56,	0},
/* 45 */	{196,	4980,	0,	14,	7,	0x56,	2,	1,	56,	0},
/* 46 */	{200,	5000,	0,	14,	8,	0x56,	2,	1,	56,	0},
/* 47 */	{204,	5020,	0,	14,	9,	0x56,	2,	1,	56,	0},
/* 48 */	{208,	5040,	0,	14,	10,	0x55,	3,	1,	56,	0},
/* 49 */	{212,	5060,	0,	14,	11,	0x55,	3,	1,	56,	0},
/* 50 */	{216,	5080,	0,	14,	12,	0x55,	3,	1,	56,	0}
#endif /* BAND5G */
};

#endif  /* !BCM4329B1 */

static uint32 wlc_phy_get_radio_ver(phy_info_t *pi);
static uint16 read_radio_reg_h(phy_info_t *pi, uint16 addr);
STATIC uint16 read_radio_reg_low(phy_info_t *pi, uint16 addr);
STATIC uint16 radio_raddr(phy_info_t *pi, uint16 addr);
static void wlc_phy_timercb_phynoise(void *arg);
static void wlc_phy_noise_cb(phy_info_t *pi, uint8 channel, int8 noise_dbm, bool polling);
#ifdef LMAC_HNDRTE_CONSOLE
static void wlc_conscmd_phyreg(void *wlcp, int argc, char **argv);
#endif /* LMAC_HNDRTE_CONSOLE */

/* %%%%%%%%%%%%%%%%%%%% */
/*  ACI 		*/
/* %%%%%%%%%%%%%%%%%%%% */
static void wlc_phy_aci_upd(phy_info_t *pi);

/* Atan table for cordic >> num2str(atan(1./(2.^[0:17]))/pi*180,8) */
STATIC const fixed AtanTbl[] = {
	2949120,
	1740967,
	919879,
	466945,
	234379,
	117304,
	58666,
	29335,
	14668,
	7334,
	3667,
	1833,
	917,
	458,
	229,
	115,
	57,
	29
};



void
WLBANDINITFN(wlc_set_phy_uninitted)(phy_info_t *pi)
{
	/* Prepare for one-time initializations */
	pi->initialized = FALSE;

	pi->tx_vos = 0xffff;
	pi->nrssi_table_delta = 0x7fffffff;
	pi->rc_cal = 0xffff;

	pi->txpwridx = -1;
	pi->radiopwr = 0xffff;
}

/* returns a pointer to per interface instance data */
void *
wlc_phy_shared_attach(wlc_pub_t *pub)
{
	shared_phy_t *phy_sh;

	/* allocate wlc_info_t state structure */
	if ((phy_sh = (shared_phy_t*) MALLOC(pub->osh, sizeof(shared_phy_t))) == NULL) {
		WL_ERROR(("wl%d: wlc_phy_shared_state: out of memory, malloced %d bytes\n",
			pub->unit, MALLOCED(pub->osh)));
		return NULL;
	}
	bzero((char*)phy_sh, sizeof(shared_phy_t));

	/* create our timers */
	phy_sh->fast_timer = SW_TIMER_FAST;
	phy_sh->slow_timer = SW_TIMER_SLOW;
	phy_sh->glacial_timer = SW_TIMER_GLACIAL;

	/* ACI mitigation mode is auto by default */
	phy_sh->interference_mode = WLAN_AUTO;

	if (wlc_module_register(pub, phy_iovars, "phy", pub, wlc_phy_doiovar,
		(watchdog_fn_t)wlc_phy_watchdog, wlc_phy_down)) {
		WL_ERROR(("wl%d: wlc_phy_shared_attach failed\n", pub->unit));
	}
	phy_sh->ant_avail_aa2g = (int8)ANT_AVAIL(getintvar(pub->vars, "aa2g"));
	phy_sh->ant_avail_aa5g = (int8)ANT_AVAIL(getintvar(pub->vars, "aa5g"));


	if (!getvar(pub->vars, "japanwidefilter")) {
		/* Default if not present in SROM */
		phy_sh->japan_wide_filter = TRUE;
	} else {
		phy_sh->japan_wide_filter = (bool)getintvar(pub->vars, "japanwidefilter");
	}

	return (void*)phy_sh;
}

void
wlc_phy_shared_detach(wlc_pub_t *pub, void *phy_sh)
{
	shared_phy_t *physh;

	/* unregister module */
	wlc_module_unregister(pub, "phy", pub);

	if (phy_sh) {
		physh = (shared_phy_t *)phy_sh;

		/* phy_head must have been all detached */
		if (physh->phy_head) {
			WL_ERROR(("wl%d: %s non NULL phy_head\n", pub->unit, __FUNCTION__));
			ASSERT(!physh->phy_head);
		}
		MFREE(pub->osh, phy_sh, sizeof(shared_phy_t));
	}
}


/* Figure out if we have a phy for the requested band and attach to it */
void *
wlc_phy_attach(wlc_pub_t *pub, void *regs, int bandtype, void *wlc, void *phy_sh_hdl)
{
	phy_info_t *pi;
	shared_phy_t *sh = (shared_phy_t *)phy_sh_hdl;
	uint32 sflags = 0;
	uint phyversion;
	int i;

        /* Reference functions which may be defined but not used.
         * Putting them under the correct #ifdefs is unmaintainably complex.
         */
        (void)wlc_phyreg_enter;
        (void)wlc_phyreg_exit;
        (void)wlc_radioreg_enter;
        (void)wlc_radioreg_exit;

	WL_TRACE(("wl: %s(%p, %p, %d, %p)\n", __FUNCTION__, pub, regs, bandtype, wlc));

	if (D11REV_IS(pub->corerev, 4))
		sflags = SISF_2G_PHY | SISF_5G_PHY;
	else
		sflags = si_core_sflags(pub->sih, 0, 0);

	if (BAND_5G(bandtype)) {
		if ((sflags & (SISF_5G_PHY | SISF_DB_PHY)) == 0) {
			WL_ERROR(("wl%d: %s: No phy available for 5G\n", pub->unit, __FUNCTION__));
			return NULL;
		}
	}

	if ((sflags & SISF_DB_PHY) && (pi = sh->phy_head)) {
		/* For the second band in dualband phys, just bring the core back out of reset */
		wlc_corereset((wlc_info_t *)wlc, pi->pubpi.coreflags);
		pi->refcnt++;
		return pi;
	}

	if ((pi = (phy_info_t *)MALLOC(pub->osh, sizeof(phy_info_t))) == NULL) {
		WL_ERROR(("wl%d: %s: out of memory, malloced %d bytes", pub->unit,
		          __FUNCTION__, MALLOCED(pub->osh)));
		return NULL;
	}
	bzero((char *)pi, sizeof(phy_info_t));

	pi->wlc = wlc;
	pi->wlc_hw = &((wlc_info_t*)wlc)->hw;
	pi->pub = pub;
	pi->regs = (d11regs_t *)regs;
	pi->sh = sh;
	pi->phy_init_por = TRUE;

	if (BAND_2G(bandtype) && (sflags & SISF_2G_PHY)) {
		/* Set the sflags gmode indicator */
		pi->pubpi.coreflags = SICF_GMODE;
	}

	/* get the phy type & revison */
	wlc_corereset((wlc_info_t *)wlc, pi->pubpi.coreflags);
	phyversion = R_REG(pi->pub->osh, &pi->regs->phyversion);
	pi->pubpi.phy_type = PHY_TYPE(phyversion);
	pi->pubpi.phy_rev = phyversion & PV_PV_MASK;

	pi->pubpi.ana_rev = (phyversion & PV_AV_MASK) >> PV_AV_SHIFT;
	if (!VALID_PHYTYPE(pi->pubpi.phy_type)) {
		WL_ERROR(("wl%d: %s: invalid phy_type = %d\n",
		          pi->pub->unit, __FUNCTION__, pi->pubpi.phy_type));
		goto err;
	}
	if (BAND_5G(bandtype)) {
		if (!ISAPHY(pi) && !ISNPHY(pi) && !ISLPPHY(pi) && !ISSSLPNPHY(pi)) {
			WL_ERROR(("wl%d: %s: invalid phy_type = %d for band 5G\n",
			          pi->pub->unit, __FUNCTION__, pi->pubpi.phy_type));
			goto err;
		}
	} else {
		if (!ISGPHY(pi) && !ISNPHY(pi) && !ISLPPHY(pi) && !ISSSLPNPHY(pi)) {
			WL_ERROR(("wl%d: %s: invalid phy_type = %d for band 2G\n",
			          pi->pub->unit, __FUNCTION__, pi->pubpi.phy_type));
			goto err;
		}
	}

	/* read the radio idcode */
	if (ISSIM_ENAB(pi->pub->sih)) {
		WL_INFORM(("wl%d: Assuming NORADIO, chip 0x%x pkgopt 0x%x\n", pi->pub->unit,
		           pi->pub->sih->chip, pi->pub->sih->chippkg));
		pi->pubpi.radioid = NORADIO_ID;
	} else {
		uint32 idcode;

		wlc_phy_anacore((wlc_phy_t*)pi, ON);

		idcode = wlc_phy_get_radio_ver(pi);
		pi->pubpi.radioid = (idcode & IDCODE_ID_MASK) >> IDCODE_ID_SHIFT;
		pi->pubpi.radiorev = (idcode & IDCODE_REV_MASK) >> IDCODE_REV_SHIFT;
		if (!VALID_RADIO(pi, pi->pubpi.radioid)) {
			WL_ERROR(("wl%d: %s: Unknown radio ID: 0x%x rev 0x%x phy %d, phyrev %d\n",
			          pub->unit, __FUNCTION__, pi->pubpi.radioid, pi->pubpi.radiorev,
			          pi->pubpi.phy_type, pi->pubpi.phy_rev));
			goto err;
		}

		/* make sure the radio is off until we do an "up" */
		wlc_phy_switch_radio((wlc_phy_t*)pi, OFF);
	}

	/* Prepare for one-time initializations */
	wlc_set_phy_uninitted(pi);

	pi->aci_exit_check_period = 60;
	pi->aci_state = 0;

	/* Set the default channel bandwidth to 20 MHZ */
	pi->bw = WLC_20_MHZ;

	/* set default rx iq est antenna/samples */
	pi->rxiq_samps = PHY_NOISE_SAMPLE_LOG_NUM_NPHY;
	pi->rxiq_antsel = ANT_RX_DIV_DEF;

	pi->watchdog_override = TRUE;

	/* Set RSSI moving average window size default */
	pi->rssi_ma_win_sz = MA_WINDOW_SZ;

	/* only NPHY/LPPHY support interrupt based noise measurement */
	pi->phynoise_polling = TRUE;
	if (ISNPHY(pi) || ISLPPHY(pi))
		pi->phynoise_polling = FALSE;

	/* initialize our txpwr limit to a large value until we know what band/channel
	 * we settle on in wlc_up() set the txpwr user override to the max
	 */
	for (i = 0; i < TXP_NUM_RATES; i++) {
		pi->txpwr_limit[i] = WLC_TXPWR_MAX;
		pi->tx_user_target[i] = WLC_TXPWR_MAX;
	}

	/* default radio power */
	pi->radiopwr_override = RADIOPWR_OVERRIDE_DEF;

#if SSLPNCONF
	if (ISSSLPNPHY(pi)) {
		wlc_phy_attach_sslpnphy(pi);
	} else
#endif /* SSLPNCONF */
	{
		/* This is here to complete the preceeding if */
		WL_ERROR(("wlc_phy_attach: unknown phytype\n"));
	}

	/* Good phy, increase refcnt and put it in list */
	pi->refcnt++;
	pi->next = pi->sh->phy_head;
	sh->phy_head = pi;

	if (!(pi->phynoise_timer = wl_init_timer(((wlc_info_t *)wlc)->wl,
		wlc_phy_timercb_phynoise, pi, "phynoise"))) {
		WL_ERROR(("wlc_timers_init: wl_init_timer for phynoise_timer failed\n"));
		MFREE(pi->pub->osh, pi, sizeof(phy_info_t));
		return NULL;
	}




	/* Make a public copy of the attach time constant phy attributes */
	bcopy(&pi->pubpi, &pi->pubpi_ro, sizeof(wlc_phy_t));

#ifdef LMAC_HNDRTE_CONSOLE
	hndrte_cons_addcmd("phyreg", (cons_fun_t)wlc_conscmd_phyreg, (uint32)pi);
#endif /* LMAC_HNDRTE_CONSOLE */

	return pi;

err:
	MFREE(pub->osh, pi, sizeof(phy_info_t));
	return NULL;
}

void
wlc_phy_detach(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *)pih;

	WL_TRACE(("wl: %s: pi = %p\n", __FUNCTION__, pi));

	if (pih) {
		if (--pi->refcnt) {
			return;
		}
        
		wlc_phy_detach_sslpnphy(pi);

		/* Quick-n-dirty remove from list */
		if (pi->sh->phy_head == pi)
			pi->sh->phy_head = pi->next;
		else if (pi->sh->phy_head->next == pi)
			pi->sh->phy_head->next = NULL;
		else
			ASSERT(0);

		MFREE(pi->pub->osh, pi, sizeof(phy_info_t));
	}
}

void
WLBANDINITFN(wlc_phy_reset)(wlc_phy_t *pih, si_t *sih)
{
	phy_info_t *pi = (phy_info_t*)pih;
	uint32 phyclk_bits = 0;

	/* select the phy speed according to selected channel b/w applies to NPHY's only */
	/* SSLPNPHY too needs this */
	if (pi && (ISNPHY(pi) || ISSSLPNPHY(pi))) {
		switch (pi->bw) {
			case WLC_10_MHZ:
				phyclk_bits = SICF_BW10;
				break;
			case WLC_20_MHZ:
				phyclk_bits = SICF_BW20;
				break;
			case WLC_40_MHZ:
				phyclk_bits = SICF_BW40;
				break;
			default:
				ASSERT(0); /* should never get here */
				break;
		}
	}

	/* For sslpnphy 40MHz bw program the pll */
	if (pi && (SSLPNREV_IS(pi->pubpi.phy_rev, 2) || SSLPNREV_IS(pi->pubpi.phy_rev, 4))) {
		si_core_cflags(sih, SICF_BWMASK, phyclk_bits);

		if (phyclk_bits == SICF_BW40) {
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL1, 0xff000000, 0x09000000);
			OSL_DELAY(5);
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL2, 0x0000ffff, 0x0000120C);
			OSL_DELAY(5);
		} else if (phyclk_bits == SICF_BW20) {
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL1, 0xff000000, 0x12000000);
			OSL_DELAY(5);
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL2, 0x0000ffff, 0x00001212);
			OSL_DELAY(5);
		} else if (phyclk_bits == SICF_BW10) {
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL1, 0xff000000, 0x24000000);
			OSL_DELAY(5);
			si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL2, 0x0000ffff, 0x00002424);
			OSL_DELAY(5);
		}

		/* update the pll settings now */
		si_pmu_pllupd(sih);
		OSL_DELAY(5);

		si_pmu_chipcontrol(sih, PMU1_PLL0_CHIPCTL0, 0x40000000, 0x40000000);
		OSL_DELAY(5);
		si_pmu_chipcontrol(sih, PMU1_PLL0_CHIPCTL0, 0x40000000, 0);
	}


	/* put phy into reset */
	si_core_cflags(sih, (SICF_PRST | SICF_PCLKE | SICF_BWMASK),
		(SICF_PRST | SICF_PCLKE | phyclk_bits));
	OSL_DELAY(2);
	/* take phy out of reset */
	si_core_cflags(sih, (SICF_PRST | SICF_FGC), SICF_FGC);
	OSL_DELAY(1);
	si_core_cflags(sih, SICF_FGC, 0);
	OSL_DELAY(1);

	if (pih)
		wlc_phy_anacore(pih, ON);
}

void
WLBANDINITFN(wlc_phy_init)(wlc_phy_t *pih)
{
	uint32	mc;
	initfn_t phy_init = NULL;
	phy_info_t *pi = (phy_info_t *)pih;
#ifndef WL20MHZ_ONLY
	uint8 bw;
#endif

	WL_TRACE(("wl%d: %s\n", pi->pub->unit, __FUNCTION__));
	WL_MPC(("wl%d: %s\n", pi->pub->unit, __FUNCTION__));

	/* skip if this function is called recursively(e.g. when bw is changed) */
	if (pi->init_in_progress)
		return;

	pi->init_in_progress = TRUE;

	mc = R_REG(pi->pub->osh, &pi->regs->maccontrol);
	if ((mc & MCTL_EN_MAC) != 0) {
		if (mc == 0xffffffff)
			WL_ERROR(("wl%d: wlc_phy_init: chip is dead !!!\n", pi->pub->unit));
		else
			WL_ERROR(("wl%d: wlc_phy_init:MAC running! mc=0x%x\n", pi->pub->unit, mc));
		ASSERT((const char*)"wlc_phy_init: Called with the MAC running!" == NULL);
	}

	ASSERT(pi != NULL);

	/* clear during init. To be set by higher level wlc code */
	pi->cur_interference_mode = INTERFERE_NONE;

	/* check D11 is running on Fast Clock */
	if (D11REV_GE(pi->pub->corerev, 5))
		ASSERT(si_core_sflags(pi->pub->sih, 0, 0) & SISF_FCLKA);

	if (ISSSLPNPHY(pi))
		phy_init = wlc_phy_init_sslpnphy;

	if (phy_init == NULL) {
		WL_ERROR(("wl%d: %s: No phy_init found for phy_type %d, rev %d\n",
		          pi->pub->unit, __FUNCTION__, pi->pubpi.phy_type, pi->pubpi.phy_rev));
		ASSERT(phy_init != NULL);
		return;
	}

	wlc_phy_anacore(pih, ON);

	/* ensure the phy has a chanspec thats valid for this band */
	if (!VALID_CHANSPEC(pi->wlc, pi->radio_chanspec))
		wlc_phy_chanspec_radio_set((wlc_phy_t *)pi, wlc_default_chanspec(pi->wlc, TRUE));

#ifndef WL20MHZ_ONLY
	/* sanitize bw here to avoid later mess. wlc_set_bw will invoke phy_reset,
	 *  but phy_init recursion is avoided by using init_in_progress
	 */
	bw = (CHSPEC_BW(pi->radio_chanspec) == WL_CHANSPEC_BW_40) ? WLC_40_MHZ : WLC_20_MHZ;
	if (bw != pi->bw)
		wlc_set_bw(pi->wlc, bw);
#endif
	/* radio on */
	wlc_phy_switch_radio((wlc_phy_t*)pi, ON);

	/* !! kick off the phy init !! */
	(*phy_init)(pi);

	/* Indicate a power on reset isn't needed for future phy init's */
	pi->phy_init_por = FALSE;

	if (D11REV_IS(pi->pub->corerev, 11) || D11REV_IS(pi->pub->corerev, 12))
		wlc_phy_do_dummy_tx(pi, TRUE, OFF);

	/* Save the w/b frequency tracking registers */
	if (ISGPHY(pi)) {
		pi->freqtrack_saved_regs[0] = read_phy_reg(pi, BPHY_COEFFS);
		pi->freqtrack_saved_regs[1] = read_phy_reg(pi, BPHY_STEP);
	}


	/* initialize interference algorithms */
	wlc_phy_interference(pi, pi->sh->interference_mode, FALSE);

	/* initialize rx antenna diversity */
	wlc_phy_ant_rxdiv_set((wlc_phy_t *)pi, pi->sh->rx_antdiv);

	pi->init_in_progress = FALSE;
}

/*
 * Do one-time phy initializations and calibration.
 *
 * Note: no register accesses allowed; we have not yet waited for PLL
 * since the last corereset.
 */
void
BCMINITOVERLAYFN(1, wlc_phy_cal_init)(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *)pih;

	WL_TRACE(("wl%d: %s\n", pi->pub->unit, __FUNCTION__));

	ASSERT((R_REG(pi->pub->osh, &pi->regs->maccontrol) & MCTL_EN_MAC) == 0);

	if (ISAPHY(pi))
		pi->txpwridx = DEFAULT_11A_TXP_IDX;


	if (!pi->initialized) {
		if (ISSSLPNPHY(pi)) {
			wlc_phy_cal_init_sslpnphy(pi);
			return;
		}
		pi->initialized = TRUE;
	}
}

int
BCMUNINITOVERLAYFN(1, wlc_phy_down)(void *wlc)
{
#ifndef BCMNODOWN
	phy_info_t *pi = (phy_info_t *)wlc_cur_phy(wlc);
	int callbacks = 0;

#if NCONF
#endif /* NCONF */

#if GCONF
	/* cancel phycal timer if exists */
	if (pi->phycal_timer && !wl_del_timer(((wlc_info_t *)pi->wlc)->wl, pi->phycal_timer))
		callbacks++;
#endif

	/* cancel phycal timer if exists */
	if (pi->phynoise_timer && !wl_del_timer(((wlc_info_t *)pi->wlc)->wl, pi->phynoise_timer))
		callbacks++;

#if NCONF
	pi->nphy_cal_chanspec_2G = 0;
	pi->nphy_cal_chanspec_5G = 0;
#endif /* NCONF */

	return callbacks;
#else
	return 0;
#endif /* BCMNODOWN */
}

/* WARNING: check (radioid != NORADIO_ID) before doing any radio related calibrations */
void
wlc_phy_watchdog(void *wlc)
{
	phy_info_t *pi;
	/* flag to avoid more than one phy calibrations at same wlc->now instant */
	bool delay_phy_cal = FALSE;
	uint8 band_idx;

	pi = (phy_info_t *)wlc_cur_phy(wlc);

#if SSLPNCONF
	if (ISSSLPNPHY(pi) && pi->sslpnphy_init_noise_cal_done &&
		((R_REG(pi->pub->osh, &pi->regs->maccommand) & MCMD_BG_NOISE) != 0))
		WL_INFORM(("Something Wrong With uCode Noise Measurement!\n"));
#endif /* SSLPNCONF */

	/* Update current power index */
	if (ISSSLPNPHY(pi)) {
		wlc_sslpnphy_tx_pwr_update_npt(pi);
	}

	if (!pi->watchdog_override)
		return;

	band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);
	/* defer interference checking, scan and update if RM is progress */
	if (!WLC_RM_IN_PROGRESS(wlc) && (ISNPHY(pi) || ISGPHY(pi)))
		wlc_phy_aci_upd(pi);

	/* Noise interference mitigation for lpphy */


	/* Stuff we do more frequently */
	if ((!pi->phycal_txpower) || ((pi->pub->now - pi->phycal_txpower) >= pi->sh->fast_timer)) {
		/* Keep attempting txpowr recalc until it has run successfully */
		if (!SCAN_IN_PROGRESS(wlc))

			pi->phycal_txpower = pi->pub->now;
	}
	/* Stuff we do least frequently */
	if (!pi->disable_percal &&
	    (pi->pub->now - pi->phycal_mlo) >= pi->sh->glacial_timer) {
		if (ISGPHY(pi) && (GREV_GT(pi->pubpi.phy_rev, 1)) &&
		    !(SCAN_IN_PROGRESS(wlc) ||
		      PLT_IN_PROGRESS(wlc) ||
		      WLC_RM_IN_PROGRESS(wlc) || ASSOC_IN_PROGRESS(wlc))) {

			wlc_suspend_mac_and_wait(wlc);


			pi->phycal_mlo = pi->pub->now;
			wlc_enable_mac(wlc);



			delay_phy_cal = TRUE;
		}
	}



	if ((pi->pub->now % pi->sh->fast_timer) == 0) {
		wlc_phy_update_bt_chanspec(pi, pi->radio_chanspec);
	}

	/* This is done based on a delta because in dualband cards we may be off the
	 * band for extended periods of time (think bandlocking) and therefore not
	 * run the calibration for that long.
	 */
	if (ISSSLPNPHY(pi) && !NORADIO_ENAB(pi->pubpi)) 
		wlc_phy_watchdog_sslpnphy(pi);


	/* update phy noise moving average only if no scan or rm in progress */
	if (!(SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc) || PLT_IN_PROGRESS(pi->wlc)))
	{
		wlc_phy_noise_sample_request((wlc_phy_t*)pi, PHY_NOISE_SAMPLE_MON,
			CHSPEC_CHANNEL(pi->radio_chanspec));
	}

	/* reset phynoise state if ucode interrupt doesn't arrive for so long */
	if (pi->phynoise_state && (pi->pub->now - pi->phynoise_now) > 5) {
		WL_INFORM(("wlc_phy_watchdog: ucode phy noise sampling overdue\n"));
		pi->phynoise_state = 0;
	}
}

#ifdef STA
void
wlc_phy_BSSinit(wlc_phy_t *pih, bool bonlyap, int rssi)
{
	phy_info_t *pi = (phy_info_t*)pih;
	uint i;

	if (bonlyap) {
	}
	for (i = 0; i < MA_WINDOW_SZ; i++) {
		/* watchdog idle phy noise */
		pi->sh->phy_noise_window[i] = (int8)(rssi & 0xff);
	}
	pi->sh->phy_noise_index = 0;

	if ((pi->sh->interference_mode == WLAN_AUTO) &&
	     (pi->aci_state & ACI_ACTIVE)) {
		/* Reset the clock to check again after the moving average buffer has filled
		 */
		pi->aci_start_time = pi->pub->now + MA_WINDOW_SZ;
	}
}

int8
wlc_phy_noise_avg(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *)pih;
	int tot = 0;
	int i = 0;

	for (i = 0; i < MA_WINDOW_SZ; i++)
		tot += pi->sh->phy_noise_window[i];

	tot /= MA_WINDOW_SZ;
	return (int8)tot;

}
#endif /* STA */

uint32
wlc_calc_log(uint32 power)
{
	uint32 msb1, msb2, val1, val2, diff1, diff2;

	msb1 = find_msbit(power);
	msb2 = msb1 + 1;

	val1 = 1 << msb1;
	val2 = 1 << msb2;

	diff1 = (power - val1);
	diff2 = (val2 - power);

	if (diff1 < diff2)
		return msb1;
	else
		return msb2;
}

uint32
wlc_calc_log_fine_resln(uint32 cmplx_pwr)
{
	uint32 log_val = 0;

        /* lookup table for computing the dB contribution from the first
         * 4 bits after MSB (most significant NONZERO bit) in cmplx_pwr
         * (entries in multiples of 0.25dB):
         */
        uint8 dB_LUT[] = {0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9,
                10, 10, 11, 11};
        uint8 LUT_correction[] = {13, 12, 12, 13, 16, 20, 25,
                5, 12, 19, 2, 11, 20, 5, 15, 1};

	/* Convert sample-power to dB scale: */
	{
		uint8 shift_ct, lsb, msb_loc;
		uint8 msb2345 = 0x0;
		uint32 tmp;
		tmp = cmplx_pwr;
		shift_ct = msb_loc = 0;
		while (tmp != 0) {
			tmp = tmp >> 1;
			shift_ct++;
			lsb = (uint8)(tmp & 1);
			if (lsb == 1)
				msb_loc = shift_ct;
		}

		/* Store first 4 bits after MSB: */
		if (msb_loc <= 4) {
			msb2345 = (cmplx_pwr << (4-msb_loc)) & 0xf;
		} else {
			/* Need to first round cmplx_pwr to 5 MSBs: */
			tmp = cmplx_pwr + (1U << (msb_loc-5));
			/* Check if MSB has shifted in the process: */
			if (tmp & (1U << (msb_loc+1))) {
				msb_loc++;
			}
			msb2345 = (tmp >> (msb_loc-4)) & 0xf;
		}

		/* Power in 0.25 dB steps: */
		log_val = ((3*msb_loc) << 2) + dB_LUT[msb2345];

		/* Apply a possible +0.25dB (1 step) correction depending
		 * on MSB location in cmplx_pwr[core]:
		 */
		log_val += (uint32)((msb_loc >= LUT_correction[msb2345]) ? 1 : 0);
	}
	return log_val;
}

typedef struct _phy_iq_est {
	int32  iq_prod;
	uint32 i_pwr;
	uint32 q_pwr;
} phy_iq_est_t;

uint32 
wlc_phy_rx_iq_est(phy_info_t *pi, bool rxiq_agc_en, uint8 rxiq_log2_samps, uint8 rxiq_num_iter, uint8 resolution)
{
	uint32 cmplx_pwr0 = 0;
	uint8 wait_time = 32;
	uint16 num_samps, i;
	int8 noise_dbm_ant = 0;
	int16 noise_dbm_ant_fine = 0;
	bool sampling_in_progress = 0;//(pi->phynoise_state != 0);
	uint32 log_val = 0;
	uint32 result = 0;
	int8 sslpnphy_rx_iq_est_agc_gain;
#if SSLPNCONF
	sslpnphy_iq_est_t iq_est;
#endif

	if (sampling_in_progress)
		return 0;

	pi->phynoise_state |= PHY_NOISE_STATE_MON;
	/* choose num_samps to be some power of 2 */
	num_samps = 1 << rxiq_log2_samps;

	/* get IQ power measurements */
	if (ISSSLPNPHY(pi)) {
		/* Force WLAN antenna */
		wlc_sslpnphy_btcx_override_enable(pi);

		/* Doing Agc Before h/w rx iq based Noise Pwr Measurement */
		sslpnphy_rx_iq_est_agc_gain = wlc_sslpnphy_samp_collect_agc(pi, rxiq_agc_en);

		wlc_sslpnphy_detection_disable(pi, TRUE);

		if (rxiq_num_iter == 0) /* Making sure Atleast happens for once */
			rxiq_num_iter = 1;

		for (i = 0;i < rxiq_num_iter;i++) {
			bzero(&iq_est, sizeof(iq_est));
			if (!wlc_sslpnphy_rx_iq_est(pi, num_samps, wait_time, &iq_est))
				WL_ERROR(("wlc_phy_rx_iq_est: IQ estimation failed to complete\n"));
			/* sum I and Q powers for each core, average over num_samps(With Rounding) and
			that is accumulated over num_iter */
			cmplx_pwr0 += (((iq_est.i_pwr + iq_est.q_pwr) + (1U << (rxiq_log2_samps-1))) >> rxiq_log2_samps);
		}
		cmplx_pwr0 = (cmplx_pwr0/rxiq_num_iter) * 16;
		if (resolution == 0) {
			log_val = wlc_calc_log(cmplx_pwr0);
			log_val = log_val * 3;
			noise_dbm_ant = (int8) (log_val - 49 - sslpnphy_rx_iq_est_agc_gain);
			result = (noise_dbm_ant & 0xff);
		} else if (resolution == 1) {
			log_val = wlc_calc_log_fine_resln(cmplx_pwr0);
			//log_val = log_val >> 2;
			noise_dbm_ant_fine = (int16) (log_val - (49 << 2) - (sslpnphy_rx_iq_est_agc_gain << 2));
			result = (noise_dbm_ant_fine & 0x3ff);
		}
		wlc_sslpnphy_detection_disable(pi, FALSE);
		wlc_sslpnphy_rx_gain_override_enable(pi, FALSE);
	} else {
		WL_ERROR(("wlc_phy_rx_iq_est: For This Phy supported not yet added\n"));
	}

	WL_TRACE(("wlc_phy_rx_iq_est: channel = %d agc_en = %d samples = %d num_iter = %d noise_dBm = %d agc_gain_dB = %d"
		"cmplx_pwr0 = %d resolution = %d\n",(int)CHSPEC_CHANNEL(pi->radio_chanspec), rxiq_agc_en, num_samps,
		rxiq_num_iter, result, sslpnphy_rx_iq_est_agc_gain, cmplx_pwr0, resolution));

	pi->phynoise_state &= ~PHY_NOISE_STATE_MON;

	return result;
}

void
WLBANDINITFN(wlc_phy_power_on_reset_inform)(wlc_phy_t *ppi)
{
	wlc_phy_por_inform(ppi);
}

void
WLBANDINITFN(wlc_phy_por_inform)(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	pi->phy_init_por = TRUE;
}



int8
wlc_phy_get_bw(wlc_phy_t *pii)
{
	return wlc_phy_bw_state_get(pii);
}


void
wlc_phy_set_bw(wlc_phy_t *pii, int8 bw)
{
	wlc_phy_bw_state_set(pii, bw);
}




chanspec_t
wlc_phy_get_chanspec(wlc_phy_t *pii)
{
	return wlc_phy_chanspec_get(pii);
}
void
wlc_phy_set_chanspec(wlc_phy_t *pii, chanspec_t chanspec)
{
	wlc_phy_chanspec_set(pii, chanspec);
}

int8
wlc_phy_bw_state_get(wlc_phy_t *pii)
{
	phy_info_t *pi = (phy_info_t *)pii;

	return pi->bw;
}

void
wlc_phy_bw_state_set(wlc_phy_t *pii, int8 bw)
{
	phy_info_t *pi = (phy_info_t *)pii;

	pi->bw = bw;
}

void
wlc_phy_chanspec_radio_set(wlc_phy_t *pii, chanspec_t newch)
{
	phy_info_t *pi = (phy_info_t *)pii;

	pi->radio_chanspec = newch;
}
int8
wlc_phy_preamble_override_get(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	return pi->n_preamble_override;
}

void
wlc_phy_preamble_override_set(wlc_phy_t *ppi, int8 override)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	pi->n_preamble_override = override;
}

int8
wlc_phy_get_n_preamble_override(wlc_phy_t *ppi)
{
	return wlc_phy_preamble_override_get(ppi);
}

void
wlc_phy_set_n_preamble_override(wlc_phy_t *ppi, int override)
{
	wlc_phy_preamble_override_set(ppi, (int8)override);
}

chanspec_t
wlc_phy_chanspec_get(wlc_phy_t *pii)
{
	phy_info_t *pi = (phy_info_t *)pii;

	return pi->radio_chanspec;
}
void
wlc_phy_chanspec_set(wlc_phy_t *pii, chanspec_t chanspec)
{
	phy_info_t *pi = (phy_info_t*)pii;

	WL_TRACE(("wl%d: %s: chanspec %x\n", pi->pub->unit, __FUNCTION__, chanspec));
	ASSERT(!wlc_malformed_chanspec(pi->wlc, chanspec));

	if (ISSSLPNPHY(pi)) {
		uint8 band_idx;

		band_idx = (CHSPEC_IS5G(pi->radio_chanspec) ? 1 : 0);

#ifdef PS4319XTRA
		if (CHIPID(pi->pub->sih->chip) == BCM4319_CHIP_ID)
			wlc_write_shm(pi->wlc, M_PS4319XTRA, 0);
#endif /* PS4319XTRA */
		wlc_sslpnphy_percal_flags_off(pi);

		wlc_phy_chanspec_set_sslpnphy(pi, chanspec);
		/* Some of the CRS/AGC values are dependent on Channel and VT. So initialise here
		 * to  known values
		*/
		wlc_sslpnphy_set_chanspec_tweaks(pi, pi->radio_chanspec);
		/* Common GainTable For Rx/ACI Tweaks Adding Here */
		wlc_sslpnphy_CmRxAciGainTbl_Tweaks(pi);

		wlc_sslpnphy_rx_offset_init(pi);
		if (!(SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc))) {
#ifdef DONGLEOVERLAYS
			uint32 arg = (uint32)chanspec;
			int ret;

			ret = wlc_send_overlay_event(pi->wlc, WLC_SET_VAR, PHYCAL_OVERLAY,
			                             "sslpnphy_setchancal", &arg, sizeof(uint32),
			                             WLC_E_OVL_DOWNLOAD);
			if (ret)
				WL_ERROR(("wl%d: %s: wlc_send_overlay_event failed w/status %d\n",
				          pi->pub->unit, __FUNCTION__));
#else
			wlc_sslpnphy_setchan_cal(pi, chanspec);
#endif /* DONGLEOVERLAYS */
		}
		wlc_sslpnphy_temp_adj(pi);

#ifdef PS4319XTRA
		if (CHIPID(pi->pub->sih->chip) == BCM4319_CHIP_ID)
			wlc_write_shm(pi->wlc, M_PS4319XTRA, PS4319XTRA);
#endif /* PS4319XTRA */
	}

	wlc_phy_update_bt_chanspec(pi, chanspec);
}


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
uint
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

#ifndef WLSINGLE_ANT
bool
BCMOVERLAYFN(1, wlc_phy_ant_rxdiv_get)(wlc_phy_t *ppi, uint8 *pval)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	*pval = pi->sh->rx_antdiv;
	return TRUE;
}
#endif /* WLSINGLE_ANT */

void
#ifdef WLSINGLE_ANT
WLBANDINITFN(wlc_phy_ant_rxdiv_set)(wlc_phy_t *ppi, uint8 val)
#else
wlc_phy_ant_rxdiv_set(wlc_phy_t *ppi, uint8 val)
#endif
{
	phy_info_t *pi = (phy_info_t *)ppi;
	bool suspend;
	uint8 phybw40 = IS40MHZ(pi);
	uint32 btcx_offset;

	pi->sh->rx_antdiv = val;

	/* update ucode flag for non-4322(phy has antdiv by default) */
	if (!(ISNPHY(pi) && D11REV_IS(pi->pub->corerev, 16))) {
		if (val > ANT_RX_DIV_FORCE_1)
			wlc_mhf(pi->wlc, MHF1, MHF1_ANTDIV, MHF1_ANTDIV, WLC_BAND_ALL);
		else
			wlc_mhf(pi->wlc, MHF1, MHF1_ANTDIV, 0, WLC_BAND_ALL);
	}

	if (ISNPHY(pi)) {
		/* no need to set phy reg for nphy */
		return;
	}

	/* no more to do if down */
	if (!pi->pub->up)
		return;

	suspend = (0 == (R_REG(pi->pub->osh, &pi->regs->maccontrol) & MCTL_EN_MAC));
	if (!suspend)
		wlc_suspend_mac_and_wait(pi->wlc);

	if (ISSSLPNPHY(pi)) {
		if (val > ANT_RX_DIV_FORCE_1) {
			if (phybw40) {
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
					SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_MASK,
				0x01 << SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
					SSLPNPHY_Rev2_crsgainCtrl_40_DefaultAntenna_MASK,
					((ANT_RX_DIV_START_1 == val) ? 1 : 0) <<
					SSLPNPHY_Rev2_crsgainCtrl_40_DefaultAntenna_SHIFT);
			} else {
				mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
					SSLPNPHY_crsgainCtrl_DiversityChkEnable_MASK,
				0x01 << SSLPNPHY_crsgainCtrl_DiversityChkEnable_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
					SSLPNPHY_crsgainCtrl_DefaultAntenna_MASK,
					((ANT_RX_DIV_START_1 == val) ? 1 : 0) <<
					SSLPNPHY_crsgainCtrl_DefaultAntenna_SHIFT);
			}
		} else {
			if (phybw40) {
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
					SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_MASK,
				0x00 << SSLPNPHY_Rev2_crsgainCtrl_40_DiversityChkEnable_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_Rev2_crsgainCtrl_40,
					SSLPNPHY_Rev2_crsgainCtrl_40_DefaultAntenna_MASK,
				(uint16) val << SSLPNPHY_Rev2_crsgainCtrl_40_DefaultAntenna_SHIFT);
			} else {
				mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
					SSLPNPHY_crsgainCtrl_DiversityChkEnable_MASK,
					0x00 << SSLPNPHY_crsgainCtrl_DiversityChkEnable_SHIFT);
				mod_phy_reg(pi, SSLPNPHY_crsgainCtrl,
					SSLPNPHY_crsgainCtrl_DefaultAntenna_MASK,
					(uint16)val << SSLPNPHY_crsgainCtrl_DefaultAntenna_SHIFT);
			}
		}
		/* Reset radio ctrl and crs gain */
		or_phy_reg(pi, SSLPNPHY_resetCtrl, 0x44);
		write_phy_reg(pi, SSLPNPHY_resetCtrl, 0x80);

		/* Clear ucode btcx diversity state */
		btcx_offset = (M_BTCX_DIVERSITY_SAVE >> 1);
		if (wlc_btcparam_index_to_shmem_offset(pi->wlc, &btcx_offset))
			wlc_write_shm(pi->wlc, (uint16)btcx_offset, 0);

	} else {
		WL_ERROR(("wl%d: %s: PHY_TYPE= %d is Unsupported ",
		          pi->pub->unit, __FUNCTION__, pi->pubpi.phy_type));
		ASSERT(0);
	}

	if (!suspend)
		wlc_enable_mac(pi->wlc);

	return;
}

void
wlc_phy_clear_tssi(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *)pih;

	if (ISNPHY(pi)) {
		/* NPHY doesn't use sw or ucode powercontrol */
		return;
	} else if (ISAPHY(pi)) {
		wlc_write_shm(pi->wlc, M_A_TSSI_0, NULL_TSSI_W);
		wlc_write_shm(pi->wlc, M_A_TSSI_1, NULL_TSSI_W);
	} else {
		wlc_write_shm(pi->wlc, M_B_TSSI_0, NULL_TSSI_W);
		wlc_write_shm(pi->wlc, M_B_TSSI_1, NULL_TSSI_W);
		wlc_write_shm(pi->wlc, M_G_TSSI_0, NULL_TSSI_W);
		wlc_write_shm(pi->wlc, M_G_TSSI_1, NULL_TSSI_W);
	}
}
void
wlc_phy_hold_upd(wlc_phy_t *pih, mbool id, bool set)
{
	phy_info_t *pi = (phy_info_t *)pih;
	ASSERT(id);

	WL_TRACE(("%s: id %d val %d old pi->measure_hold 0%x\n", __FUNCTION__, id, set,
		pi->measure_hold));

	if (set) {
		mboolset(pi->measure_hold, id);
	} else {
		mboolclr(pi->measure_hold, id);
	}

	return;
}
void
wlc_phy_anacore(wlc_phy_t *pih, bool on)
{
	phy_info_t *pi = (phy_info_t*)pih;

	if (ISSSLPNPHY(pi))  {
		if (on) {
			and_phy_reg(pi, LPPHY_AfeCtrlOvr,
				~(LPPHY_AfeCtrlOvr_pwdn_adc_ovr_MASK |
				LPPHY_AfeCtrlOvr_pwdn_dac_ovr_MASK |
				LPPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK));
		} else  {
			or_phy_reg(pi, SSLPNPHY_AfeCtrlOvrVal,
				SSLPNPHY_AfeCtrlOvrVal_pwdn_adc_ovr_val_MASK |
				SSLPNPHY_AfeCtrlOvrVal_pwdn_dac_ovr_val_MASK |
				SSLPNPHY_AfeCtrlOvrVal_pwdn_rssi_ovr_val_MASK);
			or_phy_reg(pi, SSLPNPHY_AfeCtrlOvr,
				SSLPNPHY_AfeCtrlOvr_pwdn_adc_ovr_MASK |
				SSLPNPHY_AfeCtrlOvr_pwdn_dac_ovr_MASK |
				SSLPNPHY_AfeCtrlOvr_pwdn_rssi_ovr_MASK);
		}
	} else {
		if (on)
			W_REG(pi->pub->osh, &pi->regs->phyanacore, 0x0);
		else
			W_REG(pi->pub->osh, &pi->regs->phyanacore, 0xF4);
	}
}

/* user txpower limit: in qdbm units with override flag */
int
wlc_phy_txpower_get(wlc_phy_t *ppi, uint *qdbm, bool *override)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	ASSERT(qdbm != NULL);
	*qdbm = pi->tx_user_target[0];
	if (override != NULL)
		*override = pi->txpwroverride;
	return (0);
}

uint32
wlc_phy_txpower_get_target_min(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t*)ppi;

	return pi->tx_power_min;
}

uint32
wlc_phy_txpower_get_target_max(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t*)ppi;

	return pi->tx_power_max;
}

/* save, change and restore tx power control */
static void
wlc_phy_pwrctrl_mode_upd(phy_info_t *pi, bool set_hwpwrctrl)
{

	if (ISSSLPNPHY(pi)) {
		WL_ERROR(("wl%d: Not yet supported for SSLPNPHY \n", pi->pub->unit));
	}
}


bool
wlc_phy_txpower_hw_ctrl_get(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t *)ppi;

	return (ISNPHY(pi) ? pi->nphy_txpwrctrl : pi->hwpwrctrl);
}

void
wlc_phy_txpower_hw_ctrl_set(wlc_phy_t *ppi, bool hwpwrctrl)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	bool cur_hwpwrctrl = pi->hwpwrctrl;

	/* validate if hardware power control is capable */
	if (!pi->hwpwrctrl_capable && hwpwrctrl) {
		WL_ERROR(("wl%d: hwpwrctrl not capable\n", pi->pub->unit));
		ASSERT(pi->hwpwrctrl_capable != hwpwrctrl);
		return;
	}

	WL_INFORM(("wl%d: setting the hwpwrctrl to %d\n", pi->pub->unit, hwpwrctrl));
	pi->hwpwrctrl = hwpwrctrl;
	pi->nphy_txpwrctrl = hwpwrctrl;

	/* if power control mode is changed, propogate it */

	if (hwpwrctrl != cur_hwpwrctrl) {
		wlc_phy_pwrctrl_mode_upd(pi, pi->hwpwrctrl);
	}
}
void
wlc_phy_txpower_sromlimit_max_get(wlc_phy_t *ppi, uint chan, uint8 *max_txpwr, uint8 *min_txpwr)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	uint8 tx_pwr_max = 0;
	uint8 tx_pwr_min = 255;
	uint8 max_num_rate;
	uint8 maxtxpwr, mintxpwr, rate, pactrl;

	pactrl = 0;
	if (ISGPHY(pi) && (BOARDFLAGS(pi->pub->boardflags) & BFL_PACTRL))
		pactrl = 3;

	max_num_rate = (ISNPHY(pi) || ISSSLPNPHY(pi)) ? TXP_NUM_RATES : (TXP_LAST_OFDM + 1);

	for (rate = 0; rate < max_num_rate; rate++) {

		wlc_phy_txpower_sromlimit(ppi, chan, &mintxpwr, &maxtxpwr, rate);

		maxtxpwr = (maxtxpwr > pactrl) ? (maxtxpwr - pactrl) : 0;
		/* Subtract 6 (1.5db) to ensure we don't go over
		 * the limit given a noisy power detector
		 */
		maxtxpwr = (maxtxpwr > 6) ? (maxtxpwr - 6) : 0;

		tx_pwr_max = MAX(tx_pwr_max, maxtxpwr);
		tx_pwr_min = MIN(tx_pwr_min, maxtxpwr);
	}
	*max_txpwr = tx_pwr_max;
	*min_txpwr = tx_pwr_min;
}
void
wlc_phy_txpower_boardlimit_band(wlc_phy_t *ppi, uint bandunit, int32 *max_pwr,
	int32 *min_pwr, uint32 *step_pwr)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	int32 local_max;

	if (ISLPPHY(pi)) {
		if (bandunit == 1)
			*max_pwr = pi->tx_srom_max_2g;
		else {
			local_max = pi->tx_srom_max_5g_low;
			if (local_max <  pi->tx_srom_max_5g_mid)
				local_max =  pi->tx_srom_max_5g_mid;
			if (local_max <  pi->tx_srom_max_5g_hi)
				local_max =  pi->tx_srom_max_5g_hi;
			*max_pwr = local_max;
		}
		*min_pwr = 8;
		*step_pwr = 1;
	}
}

void wlc_phy_bandtxpower_boardlimits(wlc_phy_t *ppi, uint band,
									 int32 *max_pwr, int32 *min_pwr, uint32 *step_pwr)
{
	wlc_phy_txpower_boardlimit_band(ppi, band, max_pwr, min_pwr, step_pwr);
}

#ifdef BAND5G
int
wlc_get_band_range(phy_info_t *pi, chanspec_t chanspec)
{
	int range = -1;
	uint channel = CHSPEC_CHANNEL(chanspec);
	uint freq = wlc_phy_channel2freq(channel);

	if (ISSSLPNPHY(pi) || ISLPPHY(pi)) {
		range = wlc_get_ssn_lp_band_range(freq);
	} else
		ASSERT(0);

	return range;
}
#endif /* BAND5G */

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
#if SSLPNCONF
		ASSERT(txp_rate_idx <= TXP_NUM_RATES);
#else
		ASSERT(txp_rate_idx <= TXP_LAST_OFDM);
#endif

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
#if SSLPNCONF
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
#endif /* SSLPNCONF */

	}
#endif /* BAND5G */
	WL_NONE(("%s: chan %d rate idx %d, sromlimit %d\n", __FUNCTION__, channel, txp_rate_idx,
		*max_pwr));
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

#if SSLPNCONF
	int voltage;
	bool ninja_board_flag = (BOARDTYPE(pi->pub->sih->boardtype) == BCM94319SDELNA6L_SSID);
	bool sdna_board_flag =
	    (BOARDTYPE(pi->pub->sih->boardtype) == BCM94319SDNA_SSID);
#endif /* SSLPNCONF */

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
	if (ISGPHY(pi) && (BOARDFLAGS(pi->pub->boardflags) & BFL_PACTRL))
		pactrl = 3;

	max_num_rate = (ISNPHY(pi) || ISSSLPNPHY(pi))  ? (TXP_NUM_RATES) : (TXP_LAST_OFDM + 1);

#if SSLPNCONF
	voltage = pi->sslpnphy_volt_low;
	/* voltage sensor has an error of 0.2V for temperature belowe 0C */
	if (pi->sslpnphy_lastsensed_temperature < 0)
		voltage = voltage - 3;
#ifdef BAND5G
	start_rate =  ((CHSPEC_IS5G(pi->radio_chanspec)) ? 4 : 0);
#endif /* BAND5G */
#endif /* SSLPNCONF */

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

#if SSLPNCONF
		if (SSLPNREV_LT(pi->pubpi.phy_rev, 2)) {
#if defined(WLPLT)
			if (tx_pwr_target[rate] < 40) {
				tx_pwr_target[rate] = tx_pwr_target[rate] - 4;
			} else
#endif /* WLPLT */
			{
				if (rate > 11)
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						pi->sslpnphy_11n_backoff;
				else if ((rate >= 8) && (rate <= 11))
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						pi->sslpnphy_54_48_36_24mbps_backoff;
				else if (rate <= 3)
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						pi->sslpnphy_cck;
				else
					tx_pwr_target[rate] = tx_pwr_target[rate] -
						pi->sslpnphy_lowerofdm;
			}
		}

#endif /* SSLPNCONF */

			/* power output percentage */
			tx_pwr_target[rate] = (tx_pwr_target[rate] * pi->pub->txpwr_percent) / 100;
		}
		tx_pwr_max = MAX(tx_pwr_max, tx_pwr_target[rate]);
		tx_pwr_min = MIN(tx_pwr_min, tx_pwr_target[rate]);
	}

		{
		/* Limit X17 ANT1 targert power to 8.5/12.5 dBm per Olympic */
		if (BOARDTYPE(pi->pub->sih->boardtype) == BCM94329OLYMPICX17U_SSID)
			pi->sslpnphy_ant1_max_pwr =
				CHSPEC_IS5G(pi->radio_chanspec) ? 34 : 50;
		else
			pi->sslpnphy_ant1_max_pwr = tx_pwr_max;
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
		pi->pub->unit, __FUNCTION__, target_chan));
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
#if SSLPNCONF
			/* For ninja boards, target power is set to the max power found by scanning
			 * all the rates. The positive offsets are calculated as the difference
			 * between the max power and the target power for each rate. These positive
			 * offsets are written to the rate table
			 */
			if ((ninja_board_flag || sdna_board_flag) && (CHSPEC_IS5G(pi->radio_chanspec))) {
				pi->tx_power_offset[rate] = pi->tx_power_max - pi->tx_power_target[rate];
			}
#endif /* SSLPNCONF */
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
	WL_NONE(("wl%d: %s", pi->pub->unit, __FUNCTION__));
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

void
wlc_phy_txpower_target_set(wlc_phy_t *ppi, struct txpwr_limits *txpwr)
{
	phy_info_t *pi = (phy_info_t*)ppi;

	/* fill the txpwr from the struct to the right offsets */
	/* cck */
	bcopy(&txpwr->cck[0], &pi->tx_user_target[TXP_FIRST_CCK], WLC_NUM_RATES_CCK);

	/* ofdm */
	bcopy(&txpwr->ofdm[0], &pi->tx_user_target[TXP_FIRST_OFDM],
		WLC_NUM_RATES_OFDM);

	/* mcs 20MHz */
	bcopy(&txpwr->mcs_20_siso[0], &pi->tx_user_target[TXP_FIRST_MCS_20],
		WLC_NUM_RATES_MCS_SISO);
	bcopy(&txpwr->mcs_20_mimo[0], &pi->tx_user_target[TXP_LAST_MCS_SISO_20 + 1],
		WLC_NUM_RATES_MCS_MIMO);

	/* mcs 40MHz */
	bcopy(&txpwr->mcs_40_siso[0], &pi->tx_user_target[TXP_FIRST_MCS_40],
		WLC_NUM_RATES_MCS_SISO);
	bcopy(&txpwr->mcs_40_mimo[0], &pi->tx_user_target[TXP_LAST_MCS_SISO_40 + 1],
		WLC_NUM_RATES_MCS_MIMO);

	if (pi->pub->up) {
		if (SCAN_IN_PROGRESS(pi->wlc)) {
			WL_TXPWR(("wl%d: Scan in progress, skipping txpower control\n",
				pi->pub->unit));
		} else {
			wlc_phy_txpower_recalc_target((wlc_phy_t *)pi, -1, NULL);
		}
	}
}

/* user txpower limit: in qdbm units with override flag */
int
wlc_phy_txpower_set(wlc_phy_t *ppi, uint qdbm, bool override)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	int i;

	if (qdbm > 127)
		return WLC_ERANGE;

	/* No way for user to set maxpower on individual rates yet.
	 * Same max power is used for all rates
	 */
	for (i = 0; i < TXP_NUM_RATES; i++)
		pi->tx_user_target[i] = (uint8)qdbm;

	/* Restrict external builds to 100% Tx Power */
	pi->txpwroverride = FALSE;


	if (pi->pub->up) {
		if (SCAN_IN_PROGRESS(pi->wlc)) {
			WL_TXPWR(("wl%d: Scan in progress, skipping txpower control\n",
				pi->pub->unit));
		} else {
			wlc_phy_txpower_recalc_target((wlc_phy_t *)pi, -1, NULL);
#if GCONF
			wlc_phy_cal_txpower_recalc_sw(pi);
#endif
		}
	}
	return (0);
}


void
wlc_phy_switch_radio(wlc_phy_t *pih, bool on)
{
	phy_info_t *pi = (phy_info_t *)pih;

	if (NORADIO_ENAB(pi->pubpi))
		return;

	{
		uint mc;

		mc = R_REG(pi->pub->osh, &pi->regs->maccontrol);
		if (mc & MCTL_EN_MAC) {
			WL_ERROR(("wl%d: %s: maccontrol 0x%x has EN_MAC set\n",
			          pi->pub->unit, __FUNCTION__, mc));
		}
	}

	/* If the radio is disabled always make sure it is off */
	if (mboolisset(pi->pub->radio_disabled, WL_RADIO_SW_DISABLE))
		on = FALSE;


	if (ISSSLPNPHY(pi)) {
		if (on) {
			and_phy_reg(pi, SSLPNPHY_RFOverride0,
				~(SSLPNPHY_RFOverride0_rfpll_pu_ovr_MASK	|
				SSLPNPHY_RFOverride0_wrssi_pu_ovr_MASK 		|
				SSLPNPHY_RFOverride0_nrssi_pu_ovr_MASK 		|
				SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK 	|
				SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK));
			and_phy_reg(pi, SSLPNPHY_rfoverride2,
				~(SSLPNPHY_rfoverride2_lna_pu_ovr_MASK |
				SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_MASK |
				SSLPNPHY_rfoverride2_slna_pu_ovr_MASK));
			if (BCMECICOEX_ENAB(pi->wlc))
				and_phy_reg(pi, SSLPNPHY_rfoverride3,
					~SSLPNPHY_rfoverride3_rfactive_ovr_MASK);
			else {
				or_phy_reg(pi, SSLPNPHY_rfoverride3_val,
					SSLPNPHY_rfoverride3_val_rfactive_ovr_val_MASK);
				or_phy_reg(pi,  SSLPNPHY_rfoverride3,
					SSLPNPHY_rfoverride3_rfactive_ovr_MASK);
			}
		} else {
			and_phy_reg(pi,  SSLPNPHY_RFOverrideVal0,
				~(SSLPNPHY_RFOverrideVal0_rfpll_pu_ovr_val_MASK |
				SSLPNPHY_RFOverrideVal0_wrssi_pu_ovr_val_MASK 	|
				SSLPNPHY_RFOverrideVal0_nrssi_pu_ovr_val_MASK 	|
				SSLPNPHY_RFOverrideVal0_internalrfrxpu_ovr_val_MASK 	|
				SSLPNPHY_RFOverrideVal0_internalrftxpu_ovr_val_MASK));
			or_phy_reg(pi, SSLPNPHY_RFOverride0,
				SSLPNPHY_RFOverride0_rfpll_pu_ovr_MASK 		|
				SSLPNPHY_RFOverride0_wrssi_pu_ovr_MASK 		|
				SSLPNPHY_RFOverride0_nrssi_pu_ovr_MASK 		|
				SSLPNPHY_RFOverride0_internalrfrxpu_ovr_MASK 	|
				SSLPNPHY_RFOverride0_internalrftxpu_ovr_MASK);

			and_phy_reg(pi, SSLPNPHY_rxlnaandgainctrl1ovrval,
				~(SSLPNPHY_rxlnaandgainctrl1ovrval_lnapuovr_Val_MASK));
			and_phy_reg(pi, SSLPNPHY_rfoverride2val,
				~(SSLPNPHY_rfoverride2val_slna_pu_ovr_val_MASK |
				SSLPNPHY_rfoverride2val_amode_ext_lna_gain_ovr_val_MASK));
			or_phy_reg(pi, SSLPNPHY_rfoverride2,
				SSLPNPHY_rfoverride2_lna_pu_ovr_MASK |
				SSLPNPHY_rfoverride2_amode_ext_lna_gain_ovr_MASK |
				SSLPNPHY_rfoverride2_slna_pu_ovr_MASK);
			if (BCMECICOEX_ENAB(pi->wlc)) {
				and_phy_reg(pi, SSLPNPHY_rfoverride3_val,
					~(SSLPNPHY_rfoverride3_val_rfactive_ovr_val_MASK));
				or_phy_reg(pi,  SSLPNPHY_rfoverride3,
					SSLPNPHY_rfoverride3_rfactive_ovr_MASK);
			} else {
				or_phy_reg(pi, SSLPNPHY_rfoverride3_val,
					SSLPNPHY_rfoverride3_val_rfactive_ovr_val_MASK);
				or_phy_reg(pi,  SSLPNPHY_rfoverride3,
					SSLPNPHY_rfoverride3_rfactive_ovr_MASK);
			}
		}
	}
}


#if defined(WLCURPOWER)
void
wlc_phy_txpower_get_current(wlc_phy_t *ppi, tx_power_t *power, uint channel)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	uint rate, num_rates;
	uint8 min_pwr, max_pwr;

#if WL_TX_POWER_RATES != TXP_NUM_RATES
#error "tx_power_t struct out of sync with this fn"
#endif

	if (ISSSLPNPHY(pi)) {
		power->rf_cores = 1;
		power->flags |= (WL_TX_POWER_F_SISO);
		if (pi->radiopwr_override == RADIOPWR_OVERRIDE_DEF)
			power->flags |= WL_TX_POWER_F_ENABLED;
		if (pi->hwpwrctrl)
			power->flags |= WL_TX_POWER_F_HW;
		/* 40Mhz supported only for 4319. Dont display this in IOCTL for 4329 */
		if (SSLPNREV_GE(pi->pubpi.phy_rev, 2))
			power->flags |= (WL_TX_POWER_F_40M_CAP);
	} 

	num_rates = (ISNPHY(pi)) || (ISSSLPNPHY(pi))  ? (TXP_NUM_RATES) : (TXP_LAST_OFDM + 1);

	for (rate = 0; rate < num_rates; rate++) {
		power->user_limit[rate] = pi->tx_user_target[rate];
		wlc_phy_txpower_sromlimit(ppi, channel, &min_pwr, &max_pwr, rate);
		power->board_limit[rate] = (uint8)max_pwr;
		power->target[rate] = pi->tx_power_target[rate];
	}
	
	if (pi->pub->up) {
		/* If hw (ucode) based, read the hw based estimate in realtime */
		wlc_phyreg_enter(ppi);
		if (ISSSLPNPHY(pi)) {
			if (wlc_phy_tpc_isenabled_sslpnphy(pi))
				power->flags |= (WL_TX_POWER_F_HW | WL_TX_POWER_F_ENABLED);
			else
				power->flags &= ~(WL_TX_POWER_F_HW | WL_TX_POWER_F_ENABLED);

			wlc_sslpnphy_get_tssi(pi, (int8*)&power->est_Pout[0],
				(int8*)&power->est_Pout_cck);
		}
		wlc_phyreg_exit(ppi);
	}
}
#endif 
#if defined(PHYCAL_CACHING) || defined(WLMCHAN)
int
wlc_phy_cal_cache_init(wlc_phy_t *ppi)
{
	return 0;
}

void
wlc_phy_cal_cache_deinit(wlc_phy_t *ppi)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	ch_calcache_t *ctx = pi->calcache;

	while (ctx) {
		pi->calcache = ctx->next;
		MFREE(pi->pub->osh, ctx,
		      sizeof(ch_calcache_t));
		ctx = pi->calcache;
	}

	pi->calcache = NULL;

	/* No more per-channel contexts, switch in the default one */
	pi->cal_info = &pi->def_cal_info;
	/* Reset the parameters */
	pi->cal_info->last_cal_temp = -50;
	pi->cal_info->last_cal_time = 0;
}

int
wlc_phy_create_chanctx(wlc_phy_t *ppi, chanspec_t chanspec)
{
	ch_calcache_t *ctx;
	phy_info_t *pi = (phy_info_t *)ppi;

	/* Check for existing */
	if (wlc_phy_get_chanctx(pi, chanspec))
		return 0;

	if (!(ctx = (ch_calcache_t *)MALLOC(pi->pub->osh, sizeof(ch_calcache_t)))) {
		PHY_ERROR(("%s: out of memory %d\n", __FUNCTION__, MALLOCED(pi->pub->osh)));
		return BCME_NOMEM;
	}
	bzero(ctx, sizeof(ch_calcache_t));

	ctx->chanspec = chanspec;
	ctx->cal_info.last_cal_temp = -50;
	ctx->cal_info.txcal_numcmds = pi->def_cal_info.txcal_numcmds;

	/* Add it to the list */
	ctx->next = pi->calcache;

	/* For the first context, switch out the default context */
	if (pi->calcache == NULL &&
	    (pi->radio_chanspec == chanspec))
		pi->cal_info = &ctx->cal_info;

	pi->calcache = ctx;
	return 0;
}

void
wlc_phy_destroy_chanctx(wlc_phy_t *ppi, chanspec_t chanspec)
{
	phy_info_t *pi = (phy_info_t *)ppi;
	ch_calcache_t *ctx = pi->calcache, *rem = pi->calcache;

	while (rem) {
		if (rem->chanspec == chanspec) {
			if (rem == pi->calcache)
				pi->calcache = rem->next;
			else
				ctx->next = rem->next;

			/* If the current cal_info points to the one being removed
			 * then switch NULL it
			 */
			if (pi->cal_info == &rem->cal_info)
				pi->cal_info = NULL;

			MFREE(pi->pub->osh, rem,
			      sizeof(ch_calcache_t));
			rem = NULL;
			break;
		}
		ctx = rem;
		rem = rem->next;
	}

	/* Set the correct context if one exists, otherwise,
	 * switch in the default one
	 */
	if (pi->cal_info == NULL) {
		ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
		if (!ctx) {
			pi->cal_info = &pi->def_cal_info;
			/* Reset the parameters */
			pi->cal_info->last_cal_temp = -50;
			pi->cal_info->last_cal_time = 0;
		} else
			pi->cal_info = &ctx->cal_info;
	}
}

ch_calcache_t *
wlc_phy_get_chanctx(phy_info_t *phi, chanspec_t chanspec)
{
	ch_calcache_t *ctx = phi->calcache;
	while (ctx) {
		if (ctx->chanspec == chanspec)
			return ctx;
		ctx = ctx->next;
	}
	return NULL;
}

#endif /* PHYCAL_CACHING || WLMCHAN */

#if defined(AP) && defined(RADAR)
int
wlc_phy_radar_detect_run(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *)pih;


	if (ISSSLPNPHY(pi))
		return (RADAR_TYPE_NONE);

	ASSERT(0);
	return (RADAR_TYPE_NONE);
}
#endif /* #if defined(AP) && defined(RADAR) */





/* Takes the table name, list of entries, offset to load the table,
 * see xxxphyprocs.tcl, proc xxxphy_write_table
 */

void
wlc_phy_write_table
(phy_info_t *pi,
	CONST phytbl_info_t *ptbl_info, uint16 tblAddr, uint16 tblDataHi, uint16 tblDatalo)
{
	uint    idx;
	uint    tbl_id     = ptbl_info->tbl_id;
	uint    tbl_offset = ptbl_info->tbl_offset;
	const uint8  *ptbl_8b    = (const uint8  *)ptbl_info->tbl_ptr;
	const uint16 *ptbl_16b   = (const uint16 *)ptbl_info->tbl_ptr;
	const uint32 *ptbl_32b   = (const uint32 *)ptbl_info->tbl_ptr;

	ASSERT((ptbl_info->tbl_width == 8) || (ptbl_info->tbl_width == 16) ||
		(ptbl_info->tbl_width == 32));

	WL_TRACE(("wl%d: %s\n", pi->pub->unit, __FUNCTION__));

	write_phy_reg(pi, tblAddr, (tbl_id << 10) | tbl_offset);

	for (idx = 0; idx < ptbl_info->tbl_len; idx++) {
		if (ptbl_info->tbl_width == 32) {
			/* width is 32-bit */
			write_phy_reg(pi, tblDataHi, (ptbl_32b[idx] >> 16) & 0xffff);
			write_phy_reg(pi, tblDatalo, ptbl_32b[idx] & 0xffff);
		} else if (ptbl_info->tbl_width == 16) {
			/* width is 16-bit */
			write_phy_reg(pi, tblDatalo, ptbl_16b[idx] & 0xffff);
		} else {
			/* width is 8-bit */
			write_phy_reg(pi, tblDatalo, ptbl_8b[idx] & 0xffff);
		}
	}

}

void
wlc_phy_read_table
(phy_info_t *pi,
	CONST phytbl_info_t *ptbl_info, uint16 tblAddr, uint16 tblDataHi, uint16 tblDatalo)
{
	uint    idx;
	uint    tbl_id     = ptbl_info->tbl_id;
	uint    tbl_offset = ptbl_info->tbl_offset;
	uint8  *ptbl_8b    = (uint8  *)(uintptr)ptbl_info->tbl_ptr;
	uint16 *ptbl_16b   = (uint16 *)(uintptr)ptbl_info->tbl_ptr;
	uint32 *ptbl_32b   = (uint32 *)(uintptr)ptbl_info->tbl_ptr;

	ASSERT((ptbl_info->tbl_width == 8) || (ptbl_info->tbl_width == 16) ||
		(ptbl_info->tbl_width == 32));

	write_phy_reg(pi, tblAddr, (tbl_id << 10) | tbl_offset);

	for (idx = 0; idx < ptbl_info->tbl_len; idx++) {
		if (ptbl_info->tbl_width == 32) {
			/* width is 32-bit */
			ptbl_32b[idx]  =  read_phy_reg(pi, tblDatalo);
			ptbl_32b[idx] |= (read_phy_reg(pi, tblDataHi) << 16);
		} else if (ptbl_info->tbl_width == 16) {
			/* width is 16-bit */
			ptbl_16b[idx]  =  read_phy_reg(pi, tblDatalo);
		} else {
			/* width is 8-bit */
			ptbl_8b[idx]   =  (uint8)read_phy_reg(pi, tblDatalo);
		}
	}
}


static uint32
wlc_phy_get_radio_ver(phy_info_t *pi)
{
	uint ver = 0;

	ver = read_radio_reg(pi, RADIO_IDCODE);
	ver |= read_radio_reg_h(pi, RADIO_IDCODE) << 16;

	WL_INFORM(("wl%d: %s: IDCODE = 0x%x\n", pi->pub->unit, __FUNCTION__, ver));
	return ver;
}

/* All radio regs other than idcode are less than 16bits, so
 * {read, write}_radio_reg access the low 16bits only.
 * When reading the idcode use read_radio_reg_h to get the other half.
 * There is no write_radio_reg_h since the idcode is not writable.
 */
STATIC uint16
read_radio_reg_low(phy_info_t *pi, uint16 addr)
{
	W_REG(pi->pub->osh, &pi->regs->phy4waddr, addr);

#ifdef __ARM_ARCH_4T__
	__asm__(" .align 4 ");
	__asm__(" nop ");
#endif

	return R_REG(pi->pub->osh, &pi->regs->phy4wdatalo);
}

static void
write_radio_reg_low(phy_info_t *pi, uint16 addr, uint16 val)
{
	volatile uint16 dummy;
	osl_t *osh;

	osh = pi->pub->osh;

	W_REG(osh, &pi->regs->phy4waddr, addr);
	W_REG(osh, &pi->regs->phy4wdatalo, val);

	if ((BUSTYPE(pi->pub->sih->bustype) == PCMCIA_BUS) && (pi->pub->sih->buscorerev <= 3)) {
		dummy = R_REG(osh, &pi->regs->phyversion);
	}
}

void
write_radio_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	if (NORADIO_ENAB(pi->pubpi))
		return;

	write_radio_reg_low(pi, addr, val);
}

STATIC uint16
radio_raddr(phy_info_t *pi, uint16 addr)
{
	switch (pi->pubpi.phy_type) {
	case PHY_TYPE_A:
		CASECHECK(PHYTYPE, PHY_TYPE_A);
		if (addr != RADIO_IDCODE)
			addr |= RADIO_2060WW_READ_OFF;
		break;

	case PHY_TYPE_G:
		CASECHECK(PHYTYPE, PHY_TYPE_G);
		if (addr != RADIO_IDCODE)
			addr |= RADIO_2050_READ_OFF;
		break;

	case PHY_TYPE_N:
		CASECHECK(PHYTYPE, PHY_TYPE_N);
		if (addr != RADIO_IDCODE)
			addr |= RADIO_2055_READ_OFF;  /* works for 2056 too */
		break;

	case PHY_TYPE_LP:
		CASECHECK(PHYTYPE, PHY_TYPE_LP);
		if (BCM2063_ID == LPPHY_RADIO_ID(pi)) {
			if (addr != RADIO_2063_IDCODE)
				addr |= RADIO_2063_READ_OFF;
		} else {
			if ((addr != RADIO_2062_IDCODE_NORTH) && (addr != RADIO_2062_IDCODE_SOUTH))
				addr |= RADIO_2062_READ_OFF;
		}
		break;

	case PHY_TYPE_SSN:
		CASECHECK(PHYTYPE, PHY_TYPE_SSN);
		if (addr != RADIO_IDCODE)
			addr |= RADIO_2063_READ_OFF;
		break;

	default:
		ASSERT(VALID_PHYTYPE(pi->pubpi.phy_type));
	}

	return (addr);
}

uint16
read_radio_reg(phy_info_t *pi, uint16 addr)
{
	if (NORADIO_ENAB(pi->pubpi))
		return (NORADIO_IDCODE & 0xffff);

	addr = radio_raddr(pi, addr);
	return read_radio_reg_low(pi, addr);
}

static uint16
read_radio_reg_h(phy_info_t *pi, uint16 addr)
{
	/* Change/remove this if we ever get another register > 16bits */
	ASSERT(addr == RADIO_IDCODE);

	if (NORADIO_ENAB(pi->pubpi))
		return (NORADIO_IDCODE >> 16);

	W_REG(pi->pub->osh, &pi->regs->phy4waddr, addr);
	return (R_REG(pi->pub->osh, &pi->regs->phy4wdatahi));
}

void
and_radio_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	uint16 raddr, rval;

	if (NORADIO_ENAB(pi->pubpi))
		return;

	raddr = radio_raddr(pi, addr);
	rval = read_radio_reg_low(pi, raddr);
	write_radio_reg_low(pi, addr, (rval & val));
}

void
or_radio_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	uint16 raddr, rval;

	if (NORADIO_ENAB(pi->pubpi))
		return;

	raddr = radio_raddr(pi, addr);
	rval = read_radio_reg_low(pi, raddr);
	write_radio_reg_low(pi, addr, (rval | val));
}

void
mod_radio_reg(phy_info_t *pi, uint16 addr, uint16 mask, uint16 val)
{
	uint16 raddr, rval;

	if (NORADIO_ENAB(pi->pubpi))
		return;

	raddr = radio_raddr(pi, addr);
	rval = read_radio_reg_low(pi, raddr);
	write_radio_reg_low(pi, addr, (rval & ~mask) | (val & mask));
}

void
write_phy_channel_reg(phy_info_t *pi, uint val)
{
	volatile uint16 dummy;

	W_REG(pi->pub->osh, &pi->regs->phychannel, val);

	if ((BUSTYPE(pi->pub->sih->bustype) == PCMCIA_BUS) && (pi->pub->sih->buscorerev <= 3)) {
		dummy = R_REG(pi->pub->osh, &pi->regs->phyversion);
	}
}

uint16
read_phy_reg(phy_info_t *pi,  uint16 addr)
{
	osl_t *osh;
	d11regs_t *regs;

	osh = pi->pub->osh;
	regs = pi->regs;

	W_REG(osh, &regs->phyregaddr, addr);

	return (R_REG(osh, &regs->phyregdata));
}

void
write_phy_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	osl_t *osh;
	d11regs_t *regs;

	osh = pi->pub->osh;
	regs = pi->regs;

	W_REG(osh, &regs->phyregaddr, addr);
	W_REG(osh, &regs->phyregdata, val);
}

void
and_phy_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	osl_t *osh;
	d11regs_t *regs;

	osh = pi->pub->osh;
	regs = pi->regs;

	W_REG(osh, &regs->phyregaddr, addr);
	W_REG(osh, &regs->phyregdata, (R_REG(osh, &regs->phyregdata) & val));
}

void
or_phy_reg(phy_info_t *pi, uint16 addr, uint16 val)
{
	osl_t *osh;
	d11regs_t *regs;

	osh = pi->pub->osh;
	regs = pi->regs;

	W_REG(osh, &regs->phyregaddr, addr);
	W_REG(osh, &regs->phyregdata, (R_REG(osh, &regs->phyregdata) | val));
}

void
mod_phy_reg(phy_info_t *pi, uint16 addr, uint16 mask, uint16 val)
{
	osl_t *osh;
	d11regs_t *regs;

	osh = pi->pub->osh;
	regs = pi->regs;

	W_REG(osh, &regs->phyregaddr, addr);
	W_REG(osh, &regs->phyregdata, ((R_REG(osh, &regs->phyregdata) & ~mask) | (val & mask)));
}

bool
wlc_phy_interference(phy_info_t *pi, int wanted_mode, bool init)
{
	if (init) {
		pi->interference_mode_crs_time = 0;
		pi->crsglitch_prev = 0;
	}

	pi->cur_interference_mode = wanted_mode;
	return TRUE;
}

static void
wlc_phy_timercb_phynoise(void *arg)
{
#if defined(WLCQ) || defined(WLRM)
	phy_info_t *pi = (phy_info_t*)arg;

	WL_PHYCAL(("wlc_phy_timercb_phynoise: kick off another measurement\n"));

	wlc_noise_sample(pi->wlc);
#endif /* defined(WLCQ) || defined(WLRM) */
}
static void
wlc_phy_aci_upd(phy_info_t *pi)
{

	switch (pi->sh->interference_mode) {

	case NON_WLAN:
		/* TODO: support NON_WLAN NPHY */
		break;
	case WLAN_AUTO:
		if (SCAN_IN_PROGRESS(pi->wlc) || WLC_RM_IN_PROGRESS(pi->wlc))
			break;

		break;
	default:
		break;
	}
}

#ifdef WLCQ
static int
wlc_channel_quality_eval(phy_info_t *pi)
{
	int k;
	int sample_count;
	int rssi_avg;
	int noise_est;
	int quality_metric;
	wlc_info_t *wlc = (wlc_info_t *)pi->wlc;

	sample_count = (int)wlc->channel_qa_sample_num;
	rssi_avg = 0;
	for (k = 0; k < sample_count; k++)
		rssi_avg += wlc->channel_qa_sample[k];
	rssi_avg = (rssi_avg + sample_count/2) / sample_count;

	noise_est = rssi_avg;

	if (noise_est < -85)
		quality_metric = 3;
	else if (noise_est < -75)
		quality_metric = 2;
	else if (noise_est < -65)
		quality_metric = 1;
	else
		quality_metric = 0;

	WL_INFORM(("wl%d: wlc_channel_quality_eval: samples rssi {%d %d} avg %d qa %d\n",
		wlc->pub.unit,
		wlc->channel_qa_sample[0], wlc->channel_qa_sample[1],
		rssi_avg, quality_metric));

	return (quality_metric);
}
#endif	/* WLCQ */

#if defined(WLCQ) || defined(WLRM)
/* this callback chain must defer calling phy_noise_sample_request */
static bool
wlc_phy_noise_sample_done_cqrm(phy_info_t *pi, uint8 channel, int8 noise_dbm)
{
	wlc_info_t *wlc = (wlc_info_t *)pi->wlc;
	bool moretest = FALSE;

#ifdef WLCQ
	if (wlc->channel_qa_active) {
		if (wlc->channel_qa_active && (channel != wlc->channel_qa_channel)) {
			/* bad channel, try again */
			WL_INFORM(("wl%d: wlc_channel_qa_sample: retry, samples from channel"
				" %d instead of channel %d\n",
				wlc->pub.unit, channel, wlc->channel_qa_channel));
			moretest = TRUE;
		} else {
			/* save the sample */
			wlc->channel_qa_sample[wlc->channel_qa_sample_num++] = (int8)noise_dbm;
			if (wlc->channel_qa_sample_num < WLC_CHANNEL_QA_NSAMP) {
				/* still need more samples */
				moretest = TRUE;
			} else {
				/* done with the channel quality measurement */
				wlc->channel_qa_active = FALSE;

				/* evaluate the samples to a quality metric */
				wlc->channel_quality = wlc_channel_quality_eval(pi);
			}
		}
	}
#endif	/* WLCQ */

#if defined(STA) && defined(WLRM)
	if (wlc->rm_state.rpi_active) {
		if (wlc_rm_rpi_sample(pi->wlc, noise_dbm))
			moretest = TRUE;
	}
#endif
	return moretest;
}

#endif	/* defined(WLCQ) || defined(WLRM) */

void
wlc_phy_noise_sample_request(wlc_phy_t *pih, uint8 reason, uint8 ch)
{
	phy_info_t *pi = (phy_info_t*)pih;
	wlc_info_t *wlc = (wlc_info_t *)pi->wlc;
	int8 noise_dbm = NPHY_NOISE_FIXED_VAL;
	bool sampling_in_progress = (pi->phynoise_state != 0);
	bool wait_for_intr = TRUE;

	WL_NONE(("wlc_phy_noise_sample_request: state %d reason %d, channel %d\n",
		pi->phynoise_state, reason, ch));

	if (NORADIO_ENAB(pi->pubpi)) {
		return;
	}

	switch (reason) {
	case PHY_NOISE_SAMPLE_MON:

		pi->phynoise_chan_watchdog = ch;
		pi->phynoise_state |= PHY_NOISE_STATE_MON;

		break;
	case PHY_NOISE_SAMPLE_SCAN:

		/* fill in dummy value in case the sampling failed or channel mismatch */
		wlc->phy_noise_list[ch] = NPHY_NOISE_FIXED_VAL;

		pi->phynoise_chan_scan = ch;
		pi->phynoise_state |= PHY_NOISE_STATE_SCAN;
		break;

	case PHY_NOISE_SAMPLE_CQRM:

		pi->phynoise_chan_cqrm = ch;
		pi->phynoise_state |= PHY_NOISE_STATE_CQRM;
		break;

	default:
		ASSERT(0);
		break;
	}

	/* since polling is atomic, sampling_in_progress equals to interrupt sampling ongoing
	 *  In these collision cases, always yield and wait interrupt to finish, where the results
	 *  maybe be sharable if channel matches in common callback progressing.
	 */
	if (sampling_in_progress)
		return;

	/* start test, save the timestamp to recover in case ucode gets stuck */
	pi->phynoise_now = pi->pub->now;

	if (ISLPPHY(pi)) {
		/* CQRM always use interrupt since ccx can issue many requests and
		 * suspend_mac can't finish intime
		 */
		if (!pi->phynoise_polling || (reason == PHY_NOISE_SAMPLE_CQRM)) {
			wlc_write_shm(pi->wlc, M_JSSI_0, 0);
			wlc_write_shm(pi->wlc, M_JSSI_1, 0);
			wlc_write_shm(pi->wlc, M_PWRIND_MAP0, 0);
			wlc_write_shm(pi->wlc, M_PWRIND_MAP1, 0);
			wlc_write_shm(pi->wlc, M_PWRIND_MAP2, 0);
			wlc_write_shm(pi->wlc, M_PWRIND_MAP3, 0);

			OR_REG(pi->pub->osh, &pi->regs->maccommand, MCMD_BG_NOISE);
		} else {

		}
	} else if (ISSSLPNPHY(pi)) {
		noise_dbm = SSLPNPHY_NOISE_FIXED_VAL;
	}

	/* if no interrupt scheduled, populate noise results now */
	if (!wait_for_intr)
		wlc_phy_noise_cb(pi, ch, noise_dbm, TRUE);
}

static void
wlc_phy_noise_cb(phy_info_t *pi, uint8 channel, int8 noise_dbm, bool polling)
{
	wlc_info_t *wlc = (wlc_info_t *)pi->wlc;

	if (!pi->phynoise_state)
		return;

	WL_NONE(("wlc_phy_noise_cb: state %d noise %d channel %d\n",
		pi->phynoise_state, noise_dbm, channel));

	if (pi->phynoise_state & PHY_NOISE_STATE_MON) {
		if (pi->phynoise_chan_watchdog == channel) {
			pi->sh->phy_noise_window[pi->sh->phy_noise_index] = noise_dbm;
			pi->sh->phy_noise_index = MODINC(pi->sh->phy_noise_index, MA_WINDOW_SZ);
		}
		pi->phynoise_state &= ~PHY_NOISE_STATE_MON;
	}

	if (pi->phynoise_state & PHY_NOISE_STATE_SCAN) {
		if (pi->phynoise_chan_scan == channel)
			wlc->phy_noise_list[channel] = noise_dbm;

		/* TODO - probe responses may have been constructed, fixup those dummy values
		 *  if being blocked by CQRM sampling at different channels, make another request
		 *     if we are still in the requested scan channel and scan hasn't finished yet
		 */
		pi->phynoise_state &= ~PHY_NOISE_STATE_SCAN;
	}

	if (pi->phynoise_state & PHY_NOISE_STATE_CQRM) {
		pi->phynoise_state &= ~PHY_NOISE_STATE_CQRM;

#if defined(WLCQ) || defined(WLRM)
		/* if CQRM requires more test, schedule to avoid reccursive call
		 *   ??? validate channel
		 */
		if (wlc_phy_noise_sample_done_cqrm(pi, channel, noise_dbm))
			wl_add_timer(((wlc_info_t *)pi->wlc)->wl, pi->phynoise_timer, 0, 0);
#endif
	}
}

/* ucode finished phy noise measurement and raised interrupt */
void
wlc_phy_noise_sample_intr(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t*)pih;


	uint8 channel = 0;
	int8 noise_dbm;

	if (ISSSLPNPHY(pi)) {
		noise_dbm = NPHY_NOISE_FIXED_VAL;

	}

	/* rssi dbm computed, invoke all callbacks */
	wlc_phy_noise_cb(pi, channel, noise_dbm, FALSE);
}


#ifdef STA
/* Reset RSSI moving average */
void
wlc_phy_reset_rssi_ma(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t*)pih;
	int i;

	pi->rssi_ma = 0;
	pi->rssi_ma_count = 0;
	for (i = 0; i < MA_WINDOW_SZ; i++)
		pi->rssi_window[i] = 0;
	pi->rssi_index = 0;
}


#ifdef WLC_PHY_RSSI_LOG
static uint8 rssi_log[128];
static int rssi_log_ind;
static int rssi_log_count;
#endif /* WLC_PHY_RSSI_LOG */

int
wlc_phy_update_rssi_ma(wlc_phy_t *pih, int nval)
{
	phy_info_t *pi = (phy_info_t*)pih;

#ifdef WLC_PHY_RSSI_LOG
	rssi_log[rssi_log_ind] = nval;
	rssi_log_ind = (rssi_log_ind + 1) % 128;

	rssi_log_count++;
#endif /* WLC_PHY_RSSI_LOG */


	if (nval != WLC_RSSI_INVALID) {
		/* evict old value */
		pi->rssi_ma -= pi->rssi_window[pi->rssi_index];

		/* admit new value */
		pi->rssi_ma += nval;
		pi->rssi_window[pi->rssi_index] = nval;
		pi->rssi_index = MODINC_POW2(pi->rssi_index, pi->rssi_ma_win_sz);
		if (pi->rssi_ma_count < pi->rssi_ma_win_sz)
			pi->rssi_ma_count++;
	}

	if (pi->rssi_ma_count == 0)
		return WLC_RSSI_INVALID;
	else
		return (pi->rssi_ma / pi->rssi_ma_count);
}

#ifdef WLC_PHY_RSSI_LOG
void
wlc_phy_dump_rssi_log(wlc_phy_t *pih)
{
	int i;

	printf("RSSI ind=%d cnt=%d:\n", rssi_log_ind, rssi_log_count);
	for (i = 0; i < 128; i++) {
		printf(" %d", rssi_log[i]);
		if ((i + 1) % 16 == 0)
			printf("\n");
	}
}
#endif /* WLC_PHY_RSSI_LOG */

#endif /* STA */

int
wlc_phy_compute_rssi(wlc_phy_t *pih, d11rxhdr_t *rxh)
{
	phy_info_t *pi = (phy_info_t*)pih;
	int rssi = rxh->PhyRxStatus_1 & PRXS1_JSSI_MASK;

	if (NORADIO_ENAB(pi->pubpi))
		return WLC_RSSI_INVALID;

	if (!(rxh->RxStatus2 & RXS_PHYRXST_VALID))
		return WLC_RSSI_INVALID;

#if SSLPNCONF
	if (ISSSLPNPHY(pi)) {
		if (rssi > 127)
			rssi -= 256;

		/* Add/subtract input power offset to keep reported RSSI stable */
		rssi -= wlc_sslpnphy_get_rx_pwr_offset(pi);
	}
#endif /* SSLPNCONF */


	return rssi;
}
/* phy common portion */
void    
wlc_phy_noise_measure(wlc_phy_t *pi)
{
	wlc_sslpnphy_noise_measure((phy_info_t *)pi);
}



/* coordinate with MAC before access PHY register */
void
wlc_phyreg_enter(wlc_phy_t *pih)
{
#ifdef STA
	phy_info_t *pi = (phy_info_t*)pih;

	wlc_ucode_wake_override_set(pi->wlc, WLC_WAKE_OVERRIDE_PHYREG);
#endif	/* STA */
}

void
wlc_phyreg_exit(wlc_phy_t *pih)
{
#ifdef STA
	phy_info_t *pi = (phy_info_t*)pih;

	wlc_ucode_wake_override_clear(pi->wlc_hw, WLC_WAKE_OVERRIDE_PHYREG);
#endif	/* STA */
}

/* coordinate with MAC before access RADIO register */
void
wlc_radioreg_enter(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t*)pih;
	wlc_mctrl(pi->wlc, MCTL_LOCK_RADIO, MCTL_LOCK_RADIO);

	/* allow any ucode radio reg access to complete */
	OSL_DELAY(10);
}

void
wlc_radioreg_exit(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t*)pih;
	volatile uint16 dummy;

	/* allow our radio reg access to complete */
	dummy = R_REG(pi->pub->osh, &pi->regs->phyversion);
	wlc_mctrl(pi->wlc, MCTL_LOCK_RADIO, 0);
}

void
wlc_phy_cordic(fixed theta, cint32 *val)
{
	fixed angle, valtmp;
	unsigned iter;
	int signx = 1;
	int signtheta;

	val[0].i = CORDIC_AG;
	val[0].q = 0;
	angle    = 0;

	/* limit angle to -180 .. 180 */
	signtheta = (theta < 0) ? -1 : 1;
	theta = ((theta+FIXED(180)*signtheta)% FIXED(360))-FIXED(180)*signtheta;

	/* rotate if not in quadrant one or four */
	if (FLOAT(theta) > 90) {
		theta -= FIXED(180);
		signx = -1;
	} else if (FLOAT(theta) < -90) {
		theta += FIXED(180);
		signx = -1;
	}

	/* run cordic iterations */
	for (iter = 0; iter < CORDIC_NI; iter++) {
		if (theta > angle) {
			valtmp = val[0].i - (val[0].q >> iter);
			val[0].q = (val[0].i >> iter) + val[0].q;
			val[0].i = valtmp;
			angle += AtanTbl[iter];
		} else {
			valtmp = val[0].i + (val[0].q >> iter);
			val[0].q = -(val[0].i >> iter) + val[0].q;
			val[0].i = valtmp;
			angle -= AtanTbl[iter];
		}
	}

	/* re-rotate quadrant two and three points */
	val[0].i = val[0].i*signx;
	val[0].q = val[0].q*signx;
}


uint8
wlc_phy_nbits(int32 value)
{
	int32 abs_val;
	uint8 nbits = 0;

	abs_val = ABS(value);
	while ((abs_val >> nbits) > 0) nbits++;

	return nbits;
}

uint32
wlc_phy_sqrt_int(uint32 value)
{
	uint32 root = 0, shift = 0;

	/* Compute integer nearest to square root of input integer value */
	for (shift = 0; shift < 32; shift += 2) {
		if (((0x40000000 >> shift) + root) <= value) {
			value -= ((0x40000000 >> shift) + root);
			root = (root >> 1) | (0x40000000 >> shift);
		} else {
			root = root >> 1;
		}
	}

	/* round to the nearest integer */
	if (root < value) ++root;

	return root;
}


void
BCMOVERLAYFN(1, wlc_phy_do_dummy_tx)(phy_info_t *pi, bool ofdm, bool pa_on)
{
#define	DUMMY_PKT_LEN	20 /* Dummy packet's length */
	d11regs_t *regs = pi->regs;
	int	i, count;
	uint8	ofdmpkt[DUMMY_PKT_LEN] = {
		0xcc, 0x01, 0x02, 0x00, 0x00, 0x00, 0xd4, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00
	};
	uint8	cckpkt[DUMMY_PKT_LEN] = {
		0x6e, 0x84, 0x0b, 0x00, 0x00, 0x00, 0xd4, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00
	};
	uint32 *dummypkt;

	ASSERT((R_REG(pi->pub->osh, &pi->regs->maccontrol) & MCTL_EN_MAC) == 0);

	dummypkt = (uint32 *)(ofdm ? ofdmpkt : cckpkt);
	wlc_write_template_ram(pi->wlc_hw, 0, DUMMY_PKT_LEN, dummypkt);

	/* set up the TXE transfer */

	W_REG(pi->pub->osh, &regs->xmtsel, 0);
	/* Assign the WEP to the transmit path */
	if (D11REV_GE(pi->pub->corerev, 11))
		W_REG(pi->pub->osh, &regs->wepctl, 0x100);
	else
		W_REG(pi->pub->osh, &regs->wepctl, 0);

	/* Set/clear OFDM bit in PHY control word */
	W_REG(pi->pub->osh, &regs->txe_phyctl, (ofdm ? 1 : 0) | PHY_TXC_ANT_0);
	if (ISNPHY(pi) || ISLPPHY(pi) || ISSSLPNPHY(pi)) {
		ASSERT(ofdm);
		W_REG(pi->pub->osh, &regs->txe_phyctl1, 0x1A02);
	}

	W_REG(pi->pub->osh, &regs->txe_wm_0, 0);		/* No substitutions */
	W_REG(pi->pub->osh, &regs->txe_wm_1, 0);

	/* Set transmission from the TEMPLATE where we loaded the frame */
	W_REG(pi->pub->osh, &regs->xmttplatetxptr, 0);
	W_REG(pi->pub->osh, &regs->xmttxcnt, DUMMY_PKT_LEN);

	/* Set Template as source, length specified as a count and destination
	 * as Serializer also set "gen_eof"
	 */
	W_REG(pi->pub->osh, &regs->xmtsel, ((8 << 8) | (1 << 5) | (1 << 2) | 2));

	/* Instruct the MAC to not calculate FCS, we'll supply a bogus one */
	W_REG(pi->pub->osh, &regs->txe_ctl, 0);

	/* Start transmission and wait until sendframe goes away */
	/* Set TX_NOW in AUX along with MK_CTLWRD */
	if (ISNPHY(pi) || ISSSLPNPHY(pi))
		W_REG(pi->pub->osh, &regs->txe_aux, 0xD0);
	else if (ISLPPHY(pi))
		W_REG(pi->pub->osh, &regs->txe_aux, ((1 << 6) | (1 << 4)));
	else
		W_REG(pi->pub->osh, &regs->txe_aux, ((1 << 5) | (1 << 4)));

	/* Wait for 10 x ack time, enlarge it for vsim of QT */
	i = 0;
	count = ofdm ? 30 : 250;

#ifndef BCMQT_CPU
	if (ISSIM_ENAB(pi->pub->sih)) {
		count *= 100;
	}
#endif
	/* wait for txframe to be zero */
	while ((i++ < count) && (R_REG(pi->pub->osh, &regs->txe_status) & (1 << 7))) {
		OSL_DELAY(10);
	}
	if (i >= count)
		WL_ERROR(("wl%d: %s: Waited %d uS for %s txframe\n",
		          pi->pub->unit, __FUNCTION__, 10 * i, (ofdm ? "ofdm" : "cck")));
	/* Wait for the mac to finish (this is 10x what is supposed to take) */
	i = 0;
	/* wait for txemend */
	while ((i++ < 10) && ((R_REG(pi->pub->osh, &regs->txe_status) & (1 << 10)) == 0)) {
		OSL_DELAY(10);
	}
	if (i >= 10)
		WL_ERROR(("wl%d: %s: Waited %d uS for txemend\n",
		          pi->pub->unit, __FUNCTION__, 10 * i));

	/* Wait for the phy to finish */
	i = 0;
	/* wait for txcrs */
	while ((i++ < 10) && ((R_REG(pi->pub->osh, &regs->ifsstat) & (1 << 8)))) {
		OSL_DELAY(10);
	}
	if (i >= 10)
		WL_ERROR(("wl%d: %s: Waited %d uS for txcrs\n",
		          pi->pub->unit, __FUNCTION__, 10 * i));
}







#ifdef LMAC_HNDRTE_CONSOLE

static void
wlc_conscmd_phyreg(void *pip, int argc, char **argv)
{
	phy_info_t *pi = (phy_info_t *)pip;
	uint16 data, offset;

	if ((argc > 3) || (argc < 2)) {
		printf("%s: offset data\n", argv[0]);
		return;
	}
	if (!argv[1])
		return;
	offset = bcm_strtoul(argv[1], NULL, 0);
	if (argv[2]) {
		data = bcm_strtoul(argv[1], NULL, 0);
		write_phy_reg(pi, offset, data);
	}
	printf("offset: 0x%2x, val 0x%2x\n", offset, read_phy_reg(pi, offset));
}
#endif /* LMAC_HNDRTE_CONSOLE */
