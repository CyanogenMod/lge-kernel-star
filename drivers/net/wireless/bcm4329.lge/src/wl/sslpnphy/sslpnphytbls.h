/*
 * Declarations for Broadcom PHY core tables,
 * Networking Adapter Device Driver.
 *
 * THIS IS A GENERATED FILE - DO NOT EDIT
 * Generated on Thu Jul  5 08:24:00 PDT 2007
 *
 * Copyright(c) 2007 Broadcom Corp.
 * All Rights Reserved.
 *
 * $Id: sslpnphytbls.h,v 1.6 2010/09/17 23:49:11 Exp $
 */
/* FILE-CSTYLED */

typedef phytbl_info_t dot11sslpnphytbl_info_t;


extern CONST dot11sslpnphytbl_info_t dot11sslpnphytbl_info_rev0[];
extern CONST dot11sslpnphytbl_info_t dot11sslpnphytbl_info_rev2[];
extern CONST dot11sslpnphytbl_info_t dot11sslpnphytbl_info_rev2_shared[];
extern CONST uint32 dot11sslpnphytbl_info_sz_rev0;
extern CONST uint32 dot11sslpnphytbl_info_sz_rev2;
extern CONST uint32 dot11sslpnphytbl_aci_sz;
extern CONST uint32 dot11sslpnphytbl_no_aci_sz;
extern CONST uint32 dot11sslpnphytbl_cmrxaci_sz;
extern CONST uint32 dot11sslpnphytbl_extlna_cmrxaci_sz;
extern CONST uint32 dot11lpphy_rx_gain_init_tbls_40Mhz_sz;
extern CONST uint32 dot11lpphy_rx_gain_extlna_tbls_A_sz;
extern CONST uint32 dot11lpphy_rx_gain_extlna_tbls_A_40Mhz_sz;
extern CONST dot11sslpnphytbl_info_t dot11sslpnphy_gain_tbl_extlna_cmrxaci[];
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_olympic_x17_2g;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_olympic_x17_5g;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_4319_usbb;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_4319_sdio;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_4319_arcadyan;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_ninja6l;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_info_sdna;
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_init_tbls_5GHz_x17[];
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_init_tbls_2GHz_x17[];
extern CONST uint32 dot11lpphy_rx_gain_init_tbls_5GHz_x17_sz;
extern CONST uint32 dot11lpphy_rx_gain_init_tbls_2GHz_x17_sz;
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_init_tbls_A;
extern CONST dot11sslpnphytbl_info_t sw_ctrl_tbl_rev1_5Ghz;
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_init_tbls_40Mhz[];
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_extlna_tbls_A[];
extern CONST dot11sslpnphytbl_info_t dot11lpphy_rx_gain_extlna_tbls_A_40Mhz[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_5g[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_emu3[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_emu3_5g[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_windsor_5g[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_combiner[];
extern CONST uint16 sw_ctrl_tbl_rev02_shared_mlap_emu3_combiner[];
/*
struct sslpnphy_tx_gain_tbl{
	uchar gm;
	uchar pga;
	uchar pad;
	uchar dac;
	uchar bb_mult;
};

typedef struct sslpnphy_tx_gain_tbl sslpnphy_tx_gain_tbl_entry;
*/
extern CONST sslpnphy_tx_gain_tbl_entry dot11sslpnphy_2GHz_gaintable_rev0[];

extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable[];
extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_MidBand[];
extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_HiBand[];
extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_HighBand[];
extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_X17_ePA[];

extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_4319_midband[];
extern CONST sslpnphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_4319_hiband[];

extern CONST sslpnphy_tx_gain_tbl_entry dot11sslpnphy_noPA_gaintable_rev0[];

extern CONST uint32 sslpnphy_papd_cal_ofdm_tbl[25][64];
extern uint16 sslpnphy_rev1_cx_ofdm[10];
extern uint16 sslpnphy_rev1_cx_ofdm_fcc[10];
extern uint16 sslpnphy_rev1_real_ofdm[5];
extern uint16 sslpnphy_rev1_real_ofdm_fcc[5];
extern uint16 sslpnphy_rev1_real_cck[5];
extern uint16 sslpnphy_rev1_real_ht[5];
extern uint16 sslpnphy_rev1_real_ht_fcc[5];
extern uint16 sslpnphy_olympic_cx_ofdm[10];
extern uint16 sslpnphy_rev2_real_ofdm[5];
extern uint16 sslpnphy_rev4_real_ofdm[5];
extern uint16 sslpnphy_rev2_real_cck[5];
extern uint16 sslpnphy_rev4_real_cck[5];
extern uint16 sslpnphy_phybw40_real_ht[5];
extern uint16 sslpnphy_rev2_cx_ht[10]; 
extern uint16 sslpnphy_rev2_default[10];
extern uint16 sslpnphy_rev0_cx_cck[10];
extern uint16 sslpnphy_rev0_cx_ofdm[10];
extern uint16 sslpnphy_rev4_cx_ofdm[10];
extern uint16 sslpnphy_cx_cck[10][10];
extern uint16 sslpnphy_real_cck[10][5];
extern uint16 sslpnphy_rev4_phybw40_real_ofdm[5];
extern uint16 sslpnphy_rev4_phybw40_cx_ofdm[10];
extern uint16 sslpnphy_rev2_phybw40_cx_ofdm[10];
extern uint16 sslpnphy_rev4_phybw40_cx_ht[10];
extern uint16 sslpnphy_rev2_phybw40_cx_ht[10];
extern uint16 sslpnphy_rev4_phybw40_real_ht[5];
extern uint16 sslpnphy_rev2_phybw40_real_ht[5];
extern uint16 sslpnphy_real_ofdm[2][5];
extern uint16 sslpnphy_cx_ofdm[2][10];
extern uint16 sslpnphy_tdk_mdl_real_ofdm[5];
extern uint16 sslpnphy_rev1_cx_ofdm_sec[10];
extern uint16 sslpnphy_rev1_real_ofdm_sec[5];
extern uint16 sslpnphy_rev1_real_ht_sec[5];

extern CONST uint32 sslpnphy_gain_idx_extlna_cmrxaci_tbl[]; 
extern CONST uint16 sslpnphy_gain_extlna_cmrxaci_tbl[];
extern CONST uint32 sslpnphy_gain_idx_extlna_2g_x17[];
extern CONST uint16 sslpnphy_gain_tbl_extlna_2g_x17[];
extern CONST uint16 sw_ctrl_tbl_rev0_olympic_x17_2g[];
extern CONST uint32 dot11lpphy_rx_gain_init_tbls_A_tbl[];
extern CONST uint16 gain_tbl_rev0[];
extern CONST uint32 gain_idx_tbl_rev0[];
extern CONST uint16 sw_ctrl_tbl_rev0[];
extern CONST uint32 sslpnphy_gain_idx_extlna_5g_x17[];
extern CONST uint16 sslpnphy_gain_tbl_extlna_5g_x17[];
extern CONST uint16 sw_ctrl_tbl_rev0_olympic_x17_5g[];
extern CONST uint16 sw_ctrl_tbl_rev1_5Ghz_tbl[];
extern CONST uint32 sslpnphy_gain_idx_extlna_5g[];
extern CONST uint16 sslpnphy_gain_tbl_extlna_5g[];
extern CONST uint16 sw_ctrl_tbl_ninja6l[];

extern uint16 sslpnphy_gain_idx_extlna_cmrxaci_tbl_sz; 
extern uint16 sslpnphy_gain_extlna_cmrxaci_tbl_sz;
extern uint16 sslpnphy_gain_idx_extlna_2g_x17_sz;
extern uint16 sslpnphy_gain_tbl_extlna_2g_x17_sz;
extern uint16 sw_ctrl_tbl_rev0_olympic_x17_2g_sz;
extern uint16 dot11lpphy_rx_gain_init_tbls_A_tbl_sz;
extern uint16 dot11lpphy_rx_gain_extlna_tbls_A_tbl_sz;
extern uint16 dot11lpphy_rx_gain_extlna_tbls_A_40Mhz_tbl_sz;
extern uint16 gain_tbl_rev0_sz;
extern uint16 gain_idx_tbl_rev0_sz;
extern uint16 sw_ctrl_tbl_rev0_sz;
extern uint16 sslpnphy_gain_idx_extlna_5g_x17_sz;
extern uint16 sslpnphy_gain_tbl_extlna_5g_x17_sz;
extern uint16 sslpnphy_gain_idx_extlna_5g_sz;
extern uint16 sslpnphy_gain_tbl_extlna_5g_sz;
extern uint16 sslpnphy_gain_idx_extlna_5g_40mhz_sz;
extern uint16 sslpnphy_gain_tbl_extlna_5g_40mhz_sz;
extern uint16 sw_ctrl_tbl_rev0_olympic_x17_5g_sz;
extern uint16 sw_ctrl_tbl_rev1_5Ghz_tbl_sz;
extern uint16 sw_ctrl_tbl_ninja6l_sz;
