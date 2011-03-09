/*
 * arch/arm/mach-tegra/tegra2_mc.c
 *
 * Memory controller bandwidth profiling interface
 *
 * Copyright (c) 2009-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _INCLUDE_TEGRA2_MC_H_
#define _INCLUDE_TEGRA2_MC_H_

#define SAMPLE_ENABLE_DEFAULT		0
#define SAMPLE_LOG_SIZE			1024 /* need to be DWORD aligned */
#define SAMPLE_QUANTUM_DEFAULT		1 /* in milliseconds */
#define CLIENT_ENABLED_DEFAULT		false
#define CLIENT_ON_SCHEDULE_LENGTH	256
#define SHIFT_4K			12

typedef enum {
	FILTER_NONE,
	FILTER_ADDR,
	FILTER_CLIENT,
} FILTER_MODE;

#define MC_COUNTER_CLIENT_SIZE					256

#define MC_STAT_CONTROL_0					0x90
#define MC_STAT_CONTROL_0_EMC_GATHER_SHIFT			8
#define MC_STAT_CONTROL_0_EMC_GATHER_CLEAR			1
#define MC_STAT_CONTROL_0_EMC_GATHER_DISABLE			2
#define MC_STAT_CONTROL_0_EMC_GATHER_ENABLE			3

#define MC_STAT_EMC_ADDR_LOW_0					0x98
#define MC_STAT_EMC_ADDR_HIGH_0					0x9c
#define MC_STAT_EMC_CLOCK_LIMIT_0				0xa0
#define MC_STAT_EMC_CONTROL_0_0					0xa8
#define MC_STAT_EMC_COUNT_0_0					0xb8
#define MC_STAT_EMC_COUNT_1_0					0xbc

#define ARMC_STAT_CONTROL_FILTER_ADDR_SHIFT			27
#define ARMC_STAT_CONTROL_FILTER_ADDR_DISABLE			0
#define ARMC_STAT_CONTROL_FILTER_ADDR_ENABLE			1
#define ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT			26
#define ARMC_STAT_CONTROL_FILTER_CLIENT_DISABLE			0
#define ARMC_STAT_CONTROL_FILTER_CLIENT_ENABLE			1
#define ARMC_STAT_CONTROL_FILTER_PRI_SHIFT			28
#define ARMC_STAT_CONTROL_FILTER_PRI_DISABLE			0
#define ARMC_STAT_CONTROL_FILTER_COALESCED_SHIFT		30
#define ARMC_STAT_CONTROL_FILTER_COALESCED_DISABLE		0
#define ARMC_STAT_CONTROL_CLIENT_ID_SHIFT			8
#define ARMC_STAT_CONTROL_MODE_SHIFT				0
#define ARMC_STAT_CONTROL_MODE_BANDWIDTH			0
#define ARMC_STAT_CONTROL_EVENT_SHIFT				16
#define ARMC_STAT_CONTROL_EVENT_QUALIFIED			0

#define EMC_STAT_CONTROL_0					0x160
#define EMC_STAT_CONTROL_0_LLMC_GATHER_SHIFT			0
#define EMC_STAT_CONTROL_0_LLMC_GATHER_CLEAR			1
#define EMC_STAT_CONTROL_0_LLMC_GATHER_DISABLE			2
#define EMC_STAT_CONTROL_0_LLMC_GATHER_ENABLE			3
#define EMC_STAT_CONTROL_0_DRAM_GATHER_SHIFT			16
#define EMC_STAT_CONTROL_0_DRAM_GATHER_CLEAR			1
#define EMC_STAT_CONTROL_0_DRAM_GATHER_DISABLE			2
#define EMC_STAT_CONTROL_0_DRAM_GATHER_ENABLE			3

#define AREMC_STAT_CONTROL_MODE_SHIFT				0
#define AREMC_STAT_CONTROL_MODE_BANDWIDTH			0
#define AREMC_STAT_CONTROL_FILTER_ADDR_SHIFT			27
#define AREMC_STAT_CONTROL_FILTER_ADDR_ENABLE			1
#define AREMC_STAT_CONTROL_CLIENT_TYPE_SHIFT			8
#define AREMC_STAT_CONTROL_CLIENT_TYPE_MPCORER			0
#define AREMC_STAT_CONTROL_FILTER_CLIENT_SHIFT			26
#define AREMC_STAT_CONTROL_FILTER_CLIENT_DISABLE		0
#define AREMC_STAT_CONTROL_FILTER_ADDR_DISABLE			0
#define AREMC_STAT_CONTROL_EVENT_SHIFT				16
#define AREMC_STAT_CONTROL_EVENT_QUALIFIED			0

#define EMC_STAT_LLMC_ADDR_LOW_0				0x168
#define EMC_STAT_LLMC_ADDR_HIGH_0				0x16c
#define EMC_STAT_LLMC_CLOCK_LIMIT_0				0x170
#define EMC_STAT_LLMC_CONTROL_0_0				0x178
#define EMC_STAT_LLMC_COUNT_0_0					0x188

#define EMC_STAT_DRAM_CLOCK_LIMIT_LO_0				0x1a4
#define EMC_STAT_DRAM_CLOCK_LIMIT_HI_0				0x1a8
#define EMC_STAT_DRAM_DEV0_ACTIVATE_CNT_LO_0			0x1b4
#define EMC_STAT_DRAM_DEV0_ACTIVATE_CNT_HI_0			0x1b8
#define EMC_STAT_DRAM_DEV0_READ_CNT_LO_0			0x1bc
#define EMC_STAT_DRAM_DEV0_READ_CNT_HI_0			0x1c0
#define EMC_STAT_DRAM_DEV0_WRITE_CNT_LO_0			0x1c4
#define EMC_STAT_DRAM_DEV0_WRITE_CNT_HI_0			0x1c8
#define EMC_STAT_DRAM_DEV0_REF_CNT_LO_0				0x1cc
#define EMC_STAT_DRAM_DEV0_REF_CNT_HI_0				0x1d0
#define EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_0	0x1d4
#define EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_0	0x1d8
#define EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_0	0x1dc
#define EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_0	0x1e0
#define EMC_STAT_DRAM_DEV0_CKE_EQ1_CLKS_LO_0			0x1e4
#define EMC_STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_0			0x1e8
#define EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_LO_0			0x1ec
#define EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_0			0x1f0
#define EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_LO_0			0x1f4
#define EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_0			0x1f8
#define EMC_STAT_DRAM_DEV1_ACTIVATE_CNT_LO_0			0x1fc
#define EMC_STAT_DRAM_DEV1_ACTIVATE_CNT_HI_0			0x200
#define EMC_STAT_DRAM_DEV1_READ_CNT_LO_0			0x204
#define EMC_STAT_DRAM_DEV1_READ_CNT_HI_0			0x208
#define EMC_STAT_DRAM_DEV1_WRITE_CNT_LO_0			0x20c
#define EMC_STAT_DRAM_DEV1_WRITE_CNT_HI_0			0x210
#define EMC_STAT_DRAM_DEV1_REF_CNT_LO_0				0x214
#define EMC_STAT_DRAM_DEV1_REF_CNT_HI_0				0x218
#define EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_0	0x21c
#define EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_0	0x220
#define EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_0	0x224
#define EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_0	0x228
#define EMC_STAT_DRAM_DEV1_CKE_EQ1_CLKS_LO_0			0x22c
#define EMC_STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_0			0x230
#define EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_LO_0			0x234
#define EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_0			0x238
#define EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_LO_0			0x23c
#define EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_0			0x240
#define EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_LO_0		0x244
#define EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_0		0x248
#define EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_LO_0		0x24c
#define EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_0		0x250
#define EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_LO_0		0x254
#define EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_0		0x258
#define EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_LO_0		0x25c
#define EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_0		0x260

typedef struct tegra_mc_counter {
	/* constants during sampling */
	bool		enabled;
	bool		reschedule;
	u32		period;
	FILTER_MODE	mode;
	u32		address_low;
	u32		address_length_1; /* This represents (length - 1) */
	u32		address_window_size_1; /* This represents (size - 1) */
	u8		client_number;
	u8		clients[MC_COUNTER_CLIENT_SIZE];

	/* variables during sampling */
	bool		address_range_change;
	u32		current_address_low;
	u32		current_address_high;
	u32		value;
	u8		current_client;
	u32		sample_count;
} tegra_mc_counter_t;

typedef struct tegra_emc_dram_counter {
	/* constants during sampling */
	bool	enabled;
	u8	device_mask;

	/* variables during sampling */
	u32	value;
} tegra_emc_dram_counter_t;

#pragma pack(push)
#pragma pack(1)
#define LOG_EVENT_NUMBER_MAX		16
#define MILLISECONDS_TO_TIME_QUANTUM	10
typedef struct {
	u16	time_quantum;
	u16	event_state_change;
} log_header_t;

typedef struct {
	struct _word0 {
		u32 enabled			: 1; /* 0:0 */
		u32 address_range_change	: 1; /* 1:1 */
		u32 reserved1			: 2; /* 3:2 */
		u32 event_id			: 8; /* 11:4 */
		u32 address_range_low_pfn	: 20; /* 31:12 */
	} word0;
	struct _word1 {
		u32 address_range_length_pfn	: 20; /* 19:0 */
		u32 reserved1			: 12; /* 31:20 */
	} word1;
} log_event_t;
#pragma pack(pop)

/* client ids of mc/emc */
typedef enum {
	MC_STAT_BEGIN = 0,
	CBR_DISPLAY0A = 0,
	CBR_DISPLAY0AB,
	CBR_DISPLAY0B,
	CBR_DISPLAY0BB,
	CBR_DISPLAY0C,
	CBR_DISPLAY0CB,
	CBR_DISPLAY1B,
	CBR_DISPLAY1BB,
	CBR_EPPUP,
	CBR_G2PR,
	CBR_G2SR,
	CBR_MPEUNIFBR,
	CBR_VIRUV,
	CSR_AVPCARM7R,
	CSR_DISPLAYHC,
	CSR_DISPLAYHCB,
	CSR_FDCDRD,
	CSR_G2DR,
	CSR_HOST1XDMAR,
	CSR_HOST1XR,
	CSR_IDXSRD,
	CSR_MPCORER,
	CSR_MPE_IPRED,
	CSR_MPEAMEMRD,
	CSR_MPECSRD,
	CSR_PPCSAHBDMAR,
	CSR_PPCSAHBSLVR,
	CSR_TEXSRD,
	CSR_VDEBSEVR,
	CSR_VDEMBER,
	CSR_VDEMCER,
	CSR_VDETPER,
	CBW_EPPU,
	CBW_EPPV,
	CBW_EPPY,
	CBW_MPEUNIFBW,
	CBW_VIWSB,
	CBW_VIWU,
	CBW_VIWV,
	CBW_VIWY,
	CCW_G2DW,
	CSW_AVPCARM7W,
	CSW_FDCDWR,
	CSW_HOST1XW,
	CSW_ISPW,
	CSW_MPCOREW,
	CSW_MPECSWR,
	CSW_PPCSAHBDMAW,
	CSW_PPCSAHBSLVW,
	CSW_VDEBSEVW,
	CSW_VDEMBEW,
	CSW_VDETPMW,
	MC_STAT_END,
	EMC_DRAM_STAT_BEGIN = 128,
	ACTIVATE_CNT = 128,
	READ_CNT,
	WRITE_CNT,
	REF_CNT,
	CUMM_BANKS_ACTIVE_CKE_EQ1,
	CUMM_BANKS_ACTIVE_CKE_EQ0,
	CKE_EQ1_CLKS,
	EXTCLKS_CKE_EQ1,
	EXTCLKS_CKE_EQ0,
	NO_BANKS_ACTIVE_CKE_EQ1,
	NO_BANKS_ACTIVE_CKE_EQ0,
	EMC_DRAM_STAT_END,
	MC_STAT_AGGREGATE = 255,
} device_id;

#endif
