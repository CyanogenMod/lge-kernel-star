/*
 * Declarations for Broadcom PHY core tables,
 * Networking Adapter Device Driver.
 *
 * THIS IS A GENERATED FILE - DO NOT EDIT
 * Generated on Thu May 22 18:59:33 PDT 2008
 *
 * Copyright(c) 2007 Broadcom Corp.
 * All Rights Reserved.
 *
 * $Id: lpphytbls.h,v 1.1 2010/06/03 22:03:37 Exp $
 */
/* FILE-CSTYLED */

typedef lpphytbl_info_t dot11lpphytbl_info_t;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_ext_lna_info_rev0[];
extern CONST uint32 dot11lpphytbl_ext_lna_info_sz_rev0;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_info_rev0[];
extern CONST uint32 dot11lpphytbl_info_sz_rev0;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_info_rev1[];
extern CONST uint32 dot11lpphytbl_info_sz_rev1;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_A0_info_rev2[];
extern CONST uint32 dot11lpphytbl_A0_info_sz_rev2;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_info_rev2[];
extern CONST uint32 dot11lpphytbl_rx_gain_info_sz_rev2;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_aci_info_rev2[];
extern CONST uint32 dot11lpphytbl_rx_gain_aci_info_sz_rev2;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_ext_lna_g_info_rev2[];
extern CONST uint32 dot11lpphytbl_rx_gain_ext_lna_g_info_sz_rev2;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_info_rev2[];
extern CONST uint32 dot11lpphytbl_info_sz_rev2;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_A0_info_rev3[];
extern CONST uint32 dot11lpphytbl_A0_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_info_rev3[];
extern CONST uint32 dot11lpphytbl_rx_gain_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_aci_info_rev3[];
extern CONST uint32 dot11lpphytbl_rx_gain_aci_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_ext_lna_g_info_rev3[];
extern CONST uint32 dot11lpphytbl_rx_gain_ext_lna_g_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_rx_gain_ext_lna_g_aci_info_rev3[];
extern CONST uint32 dot11lpphytbl_rx_gain_ext_lna_g_aci_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_bringup_board_4315_sw_ctrl_info_rev3[];
extern CONST uint32 dot11lpphytbl_bringup_board_4315_sw_ctrl_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_ref_board_4315_sw_ctrl_info_rev3[];
extern CONST uint32 dot11lpphytbl_ref_board_4315_sw_ctrl_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_x7_board_4325_sw_ctrl_info_rev3[];
extern CONST uint32 dot11lpphytbl_x7_board_4325_sw_ctrl_info_sz_rev3;


extern CONST dot11lpphytbl_info_t dot11lpphytbl_info_rev3[];
extern CONST uint32 dot11lpphytbl_info_sz_rev3;


typedef struct {
	uchar gm;
	uchar pga;
	uchar pad;
	uchar dac;
	uchar bb_mult;
} lpphy_tx_gain_tbl_entry;

extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_2GHz_gaintable_rev0[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_2GHz_gaintable_rev1[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_2GHz_gaintable_rev2[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_2GHz_gaintable_rev3[];

extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_rev0[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_rev1[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_rev2[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_5GHz_gaintable_rev3[];

extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_noPA_gaintable_rev0[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_noPA_gaintable_rev1[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_noPA_gaintable_rev2[];
extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_noPA_gaintable_rev3[];

extern CONST lpphy_tx_gain_tbl_entry dot11lpphy_noPA_gaintable_5354smic_rev0[];
