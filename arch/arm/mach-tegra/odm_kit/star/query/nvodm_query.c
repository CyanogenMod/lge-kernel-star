/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * <b>NVIDIA APX ODM Kit:
 *         Implementation of the ODM Query API</b>
 *
 * @b Description: Implements the query functions for ODMs that may be
 *                 accessed at boot-time, runtime, or anywhere in between.
 */

#include <star_hw_definition.h>
#include "nvodm_query.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_memc.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_pins.h"
#include "nvodm_query_pins_ap20.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"

#define NVODM_ENABLE_EMC_DVFS (1)

static const NvU8
s_NvOdmQueryDeviceNamePrefixValue[] = { 'T','e','g','r','a',0};

static const NvU8
s_NvOdmQueryManufacturerSetting[] = {'L','G','E',0};

static const NvU8
s_NvOdmQueryModelSetting[] = {'A','P','2','0',0};

static const NvU8
s_NvOdmQueryPlatformSetting[] = {'S','T','A','R',0};

static const NvU8
s_NvOdmQueryProjectNameSetting[] = {'O','D','M',' ','K','i','t',0};

static const NvOdmDownloadTransport
s_NvOdmQueryDownloadTransportSetting = NvOdmDownloadTransport_None;



#if defined(CONFIG_MACH_STAR)	//	Updated by nVidia, 2010.12.7
static const NvOdmSdramControllerConfigAdv s_NvOdmStarSmartphoneHynixEmcConfigTable[] =
{
    {
                  0x20,   /* Rev 2.0 */
                 18000,   /* SDRAM frquency */
                   900,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000002,   /* RC */
            0x00000006,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x00000038,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000003,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x0000004b,   /* TREFBW */
            0x00000004,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xa06804ae,   /* CFG_DIG_DLL */
            0x0001f000,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000002,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 27000,   /* SDRAM frquency */
                   950,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000002,   /* RC */
            0x00000006,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x00000054,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000004,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000071,   /* TREFBW */
            0x00000004,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xa06804ae,   /* CFG_DIG_DLL */
            0x0001f000,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000003,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 54000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000004,   /* RC */
            0x00000008,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x000000a8,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000008,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000000e1,   /* TREFBW */
            0x00000005,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xa06804ae,   /* CFG_DIG_DLL */
            0x0001f000,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000005,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                108000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000007,   /* RC */
            0x0000000f,   /* RFC */
            0x00000005,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x0000017f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000010,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000001c2,   /* TREFBW */
            0x00000005,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xa06804ae,   /* CFG_DIG_DLL */
            0x007f9010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000a,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                150000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000009,   /* RC */
            0x00000014,   /* RFC */
            0x00000007,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000c,   /* RDV */
            0x0000021f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000015,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000270,   /* TREFBW */
            0x00000005,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000082,   /* FBIO_CFG5 */
            0xa04c04ae,   /* CFG_DIG_DLL */
            0x007fb010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000e,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                300000,   /* SDRAM frquency */
                  1200,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
            0x00000012,   /* RC */
            0x00000027,   /* RFC */
            0x0000000d,   /* RAS */
            0x00000006,   /* RP */
            0x00000007,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000006,   /* RD_RCD */
            0x00000006,   /* WR_RCD */
            0x00000003,   /* RRD */
            0x00000003,   /* REXT */
            0x00000003,   /* WDV */
            0x00000007,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000d,   /* RDV */
            0x0000045f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000004,   /* PDEX2WR */
            0x00000004,   /* PDEX2RD */
            0x00000006,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000f,   /* RW2PDEN */
            0x0000002a,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000f,   /* TFAW */
            0x00000007,   /* TRPAB */
            0x00000007,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000004e0,   /* TREFBW */
            0x00000006,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xe03c048b,   /* CFG_DIG_DLL */
            0x007fb010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000001b,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    }
};
#else	// old version
static const NvOdmSdramControllerConfigAdv s_NvOdmStarSmartphoneHynixEmcConfigTable[] =
{
    {
                  0x20,   /* Rev 2.0 */
                 18000,   /* SDRAM frquency */
                   900,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000002,	/* RC */
					  0x00000006,	/* RFC */
					  0x00000003,	/* RAS */
					  0x00000003,	/* RP */
					  0x00000006,	/* R2W */
					  0x00000004,	/* W2R */
					  0x00000002,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000003,	/* RD_RCD */
					  0x00000003,	/* WR_RCD */
					  0x00000002,	/* RRD */
					  0x00000002,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000006,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000008,	/* QSAFE */
					  0x0000000c,	/* RDV */
					  0x00000038,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000003,	/* PDEX2WR */
					  0x00000003,	/* PDEX2RD */
					  0x00000003,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000b,	/* RW2PDEN */
					  0x00000003,	/* TXSR */
					  0x00000003,	/* TCKE */
					  0x00000008,	/* TFAW */
					  0x00000004,	/* TRPAB */
					  0x00000008,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x0000004B,	/* TREFBW */
					  0x00000005,	/* QUSE_EXTRA */
					  0x00000001,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xA06A04AE,	/* CFG_DIG_DLL */
					  0x00007810,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x00000002,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 27000,   /* SDRAM frquency */
                   950,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000002,	/* RC */
					  0x00000006,	/* RFC */
					  0x00000003,	/* RAS */
					  0x00000003,	/* RP */
					  0x00000006,	/* R2W */
					  0x00000004,	/* W2R */
					  0x00000002,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000003,	/* RD_RCD */
					  0x00000003,	/* WR_RCD */
					  0x00000002,	/* RRD */
					  0x00000002,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000006,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000008,	/* QSAFE */
					  0x0000000c,	/* RDV */
					  0x00000054,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000003,	/* PDEX2WR */
					  0x00000003,	/* PDEX2RD */
					  0x00000003,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000b,	/* RW2PDEN */
					  0x00000004,	/* TXSR */
					  0x00000003,	/* TCKE */
					  0x00000008,	/* TFAW */
					  0x00000004,	/* TRPAB */
					  0x00000008,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x00000071,	/* TREFBW */
					  0x00000005,	/* QUSE_EXTRA */
					  0x00000001,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xA06A04AE,	/* CFG_DIG_DLL */
					  0x00007810,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x00000003,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                 54000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000004,	/* RC */
					  0x00000008,	/* RFC */
					  0x00000003,	/* RAS */
					  0x00000003,	/* RP */
					  0x00000006,	/* R2W */
					  0x00000004,	/* W2R */
					  0x00000002,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000003,	/* RD_RCD */
					  0x00000003,	/* WR_RCD */
					  0x00000002,	/* RRD */
					  0x00000002,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000006,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000008,	/* QSAFE */
					  0x0000000c,	/* RDV */
					  0x000000a8,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000003,	/* PDEX2WR */
					  0x00000003,	/* PDEX2RD */
					  0x00000003,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000b,	/* RW2PDEN */
					  0x00000008,	/* TXSR */
					  0x00000003,	/* TCKE */
					  0x00000008,	/* TFAW */
					  0x00000004,	/* TRPAB */
					  0x00000008,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x000000e1,	/* TREFBW */
					  0x00000005,	/* QUSE_EXTRA */
					  0x00000002,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xA06A04AE,	/* CFG_DIG_DLL */
					  0x00007810,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x00000005,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                108000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000007,	/* RC */
					  0x0000000f,	/* RFC */
					  0x00000005,	/* RAS */
					  0x00000003,	/* RP */
					  0x00000006,	/* R2W */
					  0x00000004,	/* W2R */
					  0x00000002,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000003,	/* RD_RCD */
					  0x00000003,	/* WR_RCD */
					  0x00000002,	/* RRD */
					  0x00000002,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000006,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000008,	/* QSAFE */
					  0x0000000c,	/* RDV */
					  0x0000017f,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000003,	/* PDEX2WR */
					  0x00000003,	/* PDEX2RD */
					  0x00000003,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000b,	/* RW2PDEN */
					  0x00000010,	/* TXSR */
					  0x00000003,	/* TCKE */
					  0x00000008,	/* TFAW */
					  0x00000004,	/* TRPAB */
					  0x00000008,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x000001c2,	/* TREFBW */
					  0x00000005,	/* QUSE_EXTRA */
					  0x00000003,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xA06A04AE,	/* CFG_DIG_DLL */
					  0x007FC010,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x0000000a,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                150000,   /* SDRAM frquency */
                  1000,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000009,	/* RC */
					  0x00000014,	/* RFC */
					  0x00000007,	/* RAS */
					  0x00000003,	/* RP */
					  0x00000006,	/* R2W */
					  0x00000004,	/* W2R */
					  0x00000002,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000003,	/* RD_RCD */
					  0x00000003,	/* WR_RCD */
					  0x00000002,	/* RRD */
					  0x00000002,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000006,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000008,	/* QSAFE */
					  0x0000000c,	/* RDV */
					  0x0000021f,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000003,	/* PDEX2WR */
					  0x00000003,	/* PDEX2RD */
					  0x00000003,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000b,	/* RW2PDEN */
					  0x000000015,	 /* TXSR */
					  0x00000003,	/* TCKE */
					  0x00000008,	/* TFAW */
					  0x00000004,	/* TRPAB */
					  0x00000008,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x00000270,	/* TREFBW */
					  0x00000005,	/* QUSE_EXTRA */
					  0x00000003,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xA04C04AE,	/* CFG_DIG_DLL */
					  0x007FF010,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x0000000e,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    },
    {
                  0x20,   /* Rev 2.0 */
                300000,   /* SDRAM frquency */
                  1200,   /* EMC core voltage */
                    46,   /* Number of EMC parameters below */
        {
					  0x00000012,	/* RC */
					  0x00000027,	/* RFC */
					  0x0000000d,	/* RAS */
					  0x00000006,	/* RP */
					  0x00000007,	/* R2W */
					  0x00000005,	/* W2R */
					  0x00000003,	/* R2P */
					  0x0000000b,	/* W2P */
					  0x00000006,	/* RD_RCD */
					  0x00000006,	/* WR_RCD */
					  0x00000003,	/* RRD */
					  0x00000003,	/* REXT */
					  0x00000003,	/* WDV */
					  0x00000007,	/* QUSE */
					  0x00000004,	/* QRST */
					  0x00000009,	/* QSAFE */
					  0x0000000d,	/* RDV */
					  0x0000045f,	/* REFRESH */
					  0x00000000,	/* BURST_REFRESH_NUM */
					  0x00000004,	/* PDEX2WR */
					  0x00000004,	/* PDEX2RD */
					  0x00000006,	/* PCHG2PDEN */
					  0x00000008,	/* ACT2PDEN */
					  0x00000001,	/* AR2PDEN */
					  0x0000000f,	/* RW2PDEN */
					  0x0000002a,	/* TXSR */
					  0x00000003,	/* TCKE */
					  0x0000000f,	/* TFAW */
					  0x00000007,	/* TRPAB */
					  0x00000007,	/* TCLKSTABLE */
					  0x00000002,	/* TCLKSTOP */
					  0x000004e0,	/* TREFBW */
					  0x00000006,	/* QUSE_EXTRA */
					  0x00000002,	/* FBIO_CFG6 */
					  0x00000000,	/* ODT_WRITE */
					  0x00000000,	/* ODT_READ */
					  0x00000282,	/* FBIO_CFG5 */
					  0xE03C048B,	/* CFG_DIG_DLL */
					  0x007FF010,	/* DLL_XFORM_DQS */
					  0x00000000,	/* DLL_XFORM_QUSE */
					  0x00000000,	/* ZCAL_REF_CNT */
					  0x0000001b,	/* ZCAL_WAIT_CNT */
					  0x00000000,	/* AUTO_CAL_INTERVAL */
					  0x00000000,	/* CFG_CLKTRIM_0 */
					  0x00000000,	/* CFG_CLKTRIM_1 */
					  0x00000000,	/* CFG_CLKTRIM_2 */
        }
    }
};
#endif


static NvOdmQuerySdioInterfaceProperty s_NvOdmQuerySdioInterfaceProperty[4] =
{
    //SDIO2 is not used, so write NULL.
    //Need to check AlwayON part for each instance.

    {NV_FALSE, 10, NV_TRUE,  6, NvOdmQuerySdioSlotUsage_wlan},
    {NV_FALSE,  0, NV_FALSE, 6, NvOdmQuerySdioSlotUsage_unused}, //SDIO2 is not used.
//20100928, , change removable option and setting time [START]
#if defined (CONFIG_MACH_STAR)
    {NV_TRUE, 10, NV_TRUE, 6, NvOdmQuerySdioSlotUsage_Media}, // micro SD, Removable, Settling time is 10ms. Dedicated to LDO10 power
#else
    {NV_FALSE, 500, NV_FALSE, 6, NvOdmQuerySdioSlotUsage_Media}, // micro SD, Removable, Settling time is 500ms. 500ms should be adjusted/optimized later again. Dedicated to LDO10 power
#endif
//20100928, , change removable option and setting time [END]
    {NV_FALSE, 10, NV_TRUE,  6, NvOdmQuerySdioSlotUsage_Boot},
};


// Wake Events
//20100413, , wakeup control [START]
#if defined (CONFIG_MACH_STAR)
static NvOdmWakeupPadInfo s_NvOdmWakeupPadInfo[] =
{
#ifdef CONFIG_MACH_STAR_TMUS
    {NV_FALSE,  0, NvOdmWakeupPadPolarity_High},    // Wake Event  0 - ulpi_data4 (IPC_SRDY2)    high??
#else
    {NV_TRUE,  0, NvOdmWakeupPadPolarity_High},    // Wake Event  0 - ulpi_data4 (IPC_SRDY2)    high??
#endif
    {NV_FALSE,  1, NvOdmWakeupPadPolarity_High},    // Wake Event  1 - gp3_pv[3] (IPC_RESET_FLAG)
    {NV_FALSE,  2, NvOdmWakeupPadPolarity_High},    // Wake Event  2 - dvi_d3
    {NV_FALSE,  3, NvOdmWakeupPadPolarity_Low},     // Wake Event  3 - sdio3_dat1
    {NV_FALSE,  4, NvOdmWakeupPadPolarity_High},    // Wake Event  4 - hdmi_int (HDMI_INT_N)
    {NV_FALSE/*NV_TRUE*/,   5, NvOdmWakeupPadPolarity_High},    // Wake Event  5 - vgp[6] (proxi_out)
    {NV_FALSE,  6, NvOdmWakeupPadPolarity_High},    // Wake Event  6 - gp3_pu[5] (VIB_EN)
#ifdef CONFIG_MACH_STAR_TMUS
    {NV_TRUE,   7, NvOdmWakeupPadPolarity_High}, // Wake Event  7 - gp3_pu[6] (IPC_SRDY1)
#else
    {NV_FALSE,  7, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  7 - gp3_pu[6] (VIB_PWM)
#endif
    {NV_TRUE,   8, NvOdmWakeupPadPolarity_AnyEdge},    // Wake Event  8 - gmi_wp_n (BT_HOST_WAKEUP)
    {NV_TRUE,   9, NvOdmWakeupPadPolarity_AnyEdge},     // Wake Event  9 - gp3_ps[2] (CHG_STATUS_N_AP20)
    {NV_FALSE, 10, NvOdmWakeupPadPolarity_High},    // Wake Event 10 - gmi_ad21
    {NV_TRUE,  11, NvOdmWakeupPadPolarity_Low},     // Wake Event 11 - spi2_cs2 (BATT_LOW_INT_N)
    {NV_TRUE,  12, NvOdmWakeupPadPolarity_Low},     // Wake Event 12 - proxi_out
    {NV_TRUE/*NV_FALSE*/, 13, NvOdmWakeupPadPolarity_Low},     // Wake Event 13 - sdio1_dat1
    {NV_FALSE, 14, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 14 - gp3_pv[6]
    {NV_FALSE, 15, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 15 - gmi_ad16
    {NV_FALSE, 16, NvOdmWakeupPadPolarity_High},    // Wake Event 16 - rtc_irq
    {NV_FALSE, 17, NvOdmWakeupPadPolarity_High},    // Wake Event 17 - kbc_interrupt
    {NV_TRUE,  18, NvOdmWakeupPadPolarity_Low},     // Wake Event 18 - pwr_int (PMIC_INT)  //20100928, , PMU interrupt enable for RTC alarm
    {NV_TRUE,  19, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 19 - usb_vbus_wakeup[0] (USB_OUT_5V)
    {NV_FALSE, 20, NvOdmWakeupPadPolarity_High},    // Wake Event 20 - usb_vbus_wakeup[1]
    {NV_FALSE, 21, NvOdmWakeupPadPolarity_Low},     // Wake Event 21 - usb_iddig[0]
    {NV_FALSE, 22, NvOdmWakeupPadPolarity_Low},     // Wake Event 22 - usb_iddig[1]
    {NV_FALSE, 23, NvOdmWakeupPadPolarity_Low},     // Wake Event 23 - gmi_iordy (MICRO_SD_DET_N)
    {NV_TRUE,  24, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 24 - gp3_pv[2] (Powerkey)
    {NV_FALSE, 25, NvOdmWakeupPadPolarity_High},    // Wake Event 25 - gp3_ps[4]
    {NV_FALSE, 26, NvOdmWakeupPadPolarity_High},    // Wake Event 26 - gp3_ps[5] (KB_COL10)
    {NV_FALSE/*NV_TRUE*/, 27, NvOdmWakeupPadPolarity_High},    // Wake Event 27 - gp3_ps[0] (WLAN_HOST_WAKEUP_N)
    {NV_FALSE, 28, NvOdmWakeupPadPolarity_Low},     // Wake Event 28 - gp3_pq[6] (KB_ROW6)
    {NV_FALSE, 29, NvOdmWakeupPadPolarity_Low},     // Wake Event 29 - gp3_pq[7] (KB_ROW6)
    {NV_FALSE, 30, NvOdmWakeupPadPolarity_High},    // Wake Event 30 - dap1_dout (DAP1_DOUT)
};
#else
#error STAR_HW not assigned
#endif
//20100413, , wakeup control [END]

/* --- Function Implementations ---*/

static NvU32
GetBctKeyValue(void)
{
    NvOdmServicesKeyListHandle hKeyList = NULL;
    NvU32 BctCustOpt = 0;

    hKeyList = NvOdmServicesKeyListOpen();
    if (hKeyList)
    {
        BctCustOpt =
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
    }

    return BctCustOpt;
}

NvOdmDebugConsole
NvOdmQueryDebugConsole(void)
{
    NvU32 CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DEFAULT:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DCC:
        return NvOdmDebugConsole_Dcc;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_NONE:
        return NvOdmDebugConsole_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_UART:
        return NvOdmDebugConsole_UartA +
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE_OPTION, CustOpt);
    default:
        return NvOdmDebugConsole_None;
    }
}

NvOdmDownloadTransport
NvOdmQueryDownloadTransport(void)
{
    NvU32 CustOpt = GetBctKeyValue();

    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, TRANSPORT, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_NONE:
        return NvOdmDownloadTransport_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_USB:
        return NvOdmDownloadTransport_Usb;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_ETHERNET:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, ETHERNET_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_SPI:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_DEFAULT:
        default:
            return NvOdmDownloadTransport_SpiEthernet;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_UART:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, UART_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_B:
            return NvOdmDownloadTransport_UartB;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_C:
            return NvOdmDownloadTransport_UartC;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_DEFAULT:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_A:
        default:
            return NvOdmDownloadTransport_UartA;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_DEFAULT:
    default:
        return s_NvOdmQueryDownloadTransportSetting;
    }
}

const NvU8*
NvOdmQueryDeviceNamePrefix(void)
{
    return s_NvOdmQueryDeviceNamePrefixValue;
}

static const NvOdmQuerySpdifInterfaceProperty s_NvOdmQuerySpdifInterfacePropertySetting =
{
    NvOdmQuerySpdifDataCaptureControl_FromLeft
};

const NvOdmQuerySpiDeviceInfo *

NvOdmQuerySpiGetDeviceInfo(
    NvOdmIoModule OdmIoModule,
    NvU32 ControllerId,
    NvU32 ChipSelect)
{
#ifdef CONFIG_SPI_TEGRA
    static const NvOdmQuerySpiDeviceInfo s_Spi1Cs0Info_IfxRil =
	 {NvOdmQuerySpiSignalMode_1, NV_FALSE, NV_FALSE};//{NvOdmQuerySpiSignalMode_1, NV_TRUE, NV_FALSE, NV_FALSE, 0, 0};
#else
    static const NvOdmQuerySpiDeviceInfo s_Spi1Cs0Info_IfxRil =
        {NvOdmQuerySpiSignalMode_1, NV_TRUE, NV_TRUE, NV_FALSE, 0, 0};	//	{NvOdmQuerySpiSignalMode_1, NV_FALSE, NV_FALSE};
#endif
//20100809-1, , Add SPI2 for AP-CP IPC [START]
#ifdef CONFIG_DUAL_SPI
	static const NvOdmQuerySpiDeviceInfo s_Spi2Cs0Info =
		{NvOdmQuerySpiSignalMode_1, NV_TRUE, NV_TRUE, NV_FALSE, 0, 0};
#else
	static const NvOdmQuerySpiDeviceInfo s_Spi2Cs0Info =
        {NvOdmQuerySpiSignalMode_3, NV_TRUE, NV_FALSE};//{NvOdmQuerySpiSignalMode_3, NV_TRUE, NV_FALSE, NV_FALSE, 0, 0};
#endif
//20100809, , Add SPI2 for AP-CP IPC [END]

    if ((OdmIoModule == NvOdmIoModule_Spi) &&
        (ControllerId == 0 ) && (ChipSelect == 0))
        return &s_Spi1Cs0Info_IfxRil;

    if ((OdmIoModule == NvOdmIoModule_Spi) &&
        (ControllerId == 1 ) && (ChipSelect == 0))
        return &s_Spi2Cs0Info;

    return NULL;
}

//20101204-1, , NVIDIA's patch for setting signal level during idle state [START]
static const NvOdmQuerySpiIdleSignalState s_NvOdmQuerySpiIdleSignalStateLevel[] = 
{ 
	{NV_FALSE, NvOdmQuerySpiSignalMode_1, NV_FALSE} // Spi 1 
}; 

const NvOdmQuerySpiIdleSignalState * 
NvOdmQuerySpiGetIdleSignalState( 
	NvOdmIoModule OdmIoModule, 
	NvU32 ControllerId) 
{ 
	if (OdmIoModule == NvOdmIoModule_Spi) 
	{ 
		if (ControllerId == 0) 
			return &s_NvOdmQuerySpiIdleSignalStateLevel[0]; 
#ifdef CONFIG_MACH_STAR_TMUS
		else if (ControllerId == 1) 
			return &s_NvOdmQuerySpiIdleSignalStateLevel[0]; 
#endif
	} 
	return NULL; 
} 
//20101204-1, , NVIDIA's patch for setting signal level during idle state [END]

const NvOdmQueryI2sInterfaceProperty *
NvOdmQueryI2sGetInterfaceProperty(
    NvU32 I2sInstanceId)
{
    static const NvOdmQueryI2sInterfaceProperty s_Property =
    {
        NvOdmQueryI2sMode_Master, //i.e. CODEC_SLAVE
        NvOdmQueryI2sLRLineControl_LeftOnLow,
        NvOdmQueryI2sDataCommFormat_I2S,
        NV_FALSE,
        0
    };

    if ((!I2sInstanceId) || (I2sInstanceId == 1))
        return &s_Property;

    return NULL;
}

const NvOdmQueryDapPortProperty *
NvOdmQueryDapPortGetProperty(
    NvU32 DapPortId)
{
    static const NvOdmQueryDapPortProperty s_Property[] =
    {
        { NvOdmDapPort_None, NvOdmDapPort_None, { 0, 0, 0, 0 } },
        // I2S1 (DAC1) <-> DAP1 <-> HIFICODEC
        { NvOdmDapPort_I2s1, NvOdmDapPort_HifiCodecType,
          { 2, 16, 44100, NvOdmQueryI2sDataCommFormat_I2S } }, // Dap1
        // I2S2 (DAC2) <-> DAP2 <-> VOICECODEC
        {NvOdmDapPort_I2s2, NvOdmDapPort_VoiceCodecType,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_Dsp } },   // Dap2
        // I2S2 (DAC2) <-> DAP3 <-> BASEBAND
        {NvOdmDapPort_I2s2, NvOdmDapPort_BaseBand,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap3
        // I2S2 (DAC2) <-> DAP4 <-> BLUETOOTH
        {NvOdmDapPort_I2s2, NvOdmDapPort_BlueTooth,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap4
    };

    if (DapPortId && DapPortId<NV_ARRAY_SIZE(s_Property))
        return &s_Property[DapPortId];

    return NULL;
}

const NvOdmQueryDapPortConnection*
NvOdmQueryDapPortGetConnectionTable(
    NvU32 ConnectionIndex)
{
    static const NvOdmQueryDapPortConnection s_Property[] =
    {
        { NvOdmDapConnectionIndex_Music_Path, 
          2, { {NvOdmDapPort_I2s1, NvOdmDapPort_Dap1, NV_TRUE},
               {NvOdmDapPort_Dap1, NvOdmDapPort_I2s1, NV_FALSE} } },

        // Voicecall without Bluetooth
        { NvOdmDapConnectionIndex_VoiceCall_NoBlueTooth,
          3, { {NvOdmDapPort_Dap3, NvOdmDapPort_Dap2, NV_TRUE},
               {NvOdmDapPort_Dap2, NvOdmDapPort_Dap3, NV_FALSE},
               {NvOdmDapPort_Dap2, NvOdmDapPort_I2s2, NV_FALSE} } },

        // Voicecall with Bluetooth
        { NvOdmDapConnectionIndex_VoiceCall_WithBlueTooth,
          2, { {NvOdmDapPort_Dap4, NvOdmDapPort_Dap3, NV_TRUE},
              {NvOdmDapPort_Dap3, NvOdmDapPort_Dap4, NV_FALSE}
            }},

    };
    NvU32 TableIndex = 0;

    for( TableIndex = 0; TableIndex < NV_ARRAY_SIZE(s_Property); TableIndex++)
    {
        if (s_Property[TableIndex].UseIndex == ConnectionIndex)
            return &s_Property[TableIndex];
    }
    return NULL;
}

const NvOdmQuerySpdifInterfaceProperty *
NvOdmQuerySpdifGetInterfaceProperty(
    NvU32 SpdifInstanceId)
{
    if (SpdifInstanceId == 0)
        return &s_NvOdmQuerySpdifInterfacePropertySetting;

    return NULL;
}

const NvOdmQueryAc97InterfaceProperty *
NvOdmQueryAc97GetInterfaceProperty(
    NvU32 Ac97InstanceId)
{
    return NULL;
}

const NvOdmQueryI2sACodecInterfaceProp *
NvOdmQueryGetI2sACodecInterfaceProperty(
    NvU32 AudioCodecId)
{
    static const NvOdmQueryI2sACodecInterfaceProp s_Property =
    {
        NV_FALSE, //i.e. Codec is slave.
        0,
        0x34, //This is I2C write address for audio codec device. WM8994
        NV_FALSE, // IsUsbmode
        NvOdmQueryI2sLRLineControl_LeftOnLow,
        NvOdmQueryI2sDataCommFormat_I2S
    };
    if (!AudioCodecId)
        return &s_Property;
    return NULL;
}

NvBool NvOdmQueryAsynchMemConfig(
    NvU32 ChipSelect,
    NvOdmAsynchMemConfig *pMemConfig)
{
    return NV_FALSE;
}

const void*
NvOdmQuerySdramControllerConfigGet(NvU32 *pEntries, NvU32 *pRevision)
{
#if NVODM_ENABLE_EMC_DVFS
    if (pRevision)
        *pRevision = s_NvOdmStarSmartphoneHynixEmcConfigTable[0].Revision;
    if (pEntries)
        *pEntries = NV_ARRAY_SIZE(s_NvOdmStarSmartphoneHynixEmcConfigTable);
    return (const void*)s_NvOdmStarSmartphoneHynixEmcConfigTable;
#endif
    if (pEntries)
        *pEntries = 0;
    return NULL;
}

NvOdmQueryOscillator NvOdmQueryGetOscillatorSource(void)
{
    return NvOdmQueryOscillator_Xtal;
}

NvU32 NvOdmQueryGetOscillatorDriveStrength(void)
{
//    return 0x04;
// Set to max for initial bringup
    return 0x3F;
}

const NvOdmWakeupPadInfo *NvOdmQueryGetWakeupPadTable(NvU32 *pSize)
{
    if (pSize)
        *pSize = NV_ARRAY_SIZE(s_NvOdmWakeupPadInfo);

    return (const NvOdmWakeupPadInfo *) s_NvOdmWakeupPadInfo;
}

const NvU8* NvOdmQueryManufacturer(void)
{
    return s_NvOdmQueryManufacturerSetting;
}

const NvU8* NvOdmQueryModel(void)
{
    return s_NvOdmQueryModelSetting;
}

const NvU8* NvOdmQueryPlatform(void)
{
    return s_NvOdmQueryPlatformSetting;
}

const NvU8* NvOdmQueryProjectName(void)
{
    return s_NvOdmQueryProjectNameSetting;
}


#define EXT 0     // external pull-up/down resistor
#define INT_PU 1  // internal pull-up
#define INT_PD 2  // internal pull-down

#define HIGHSPEED 1
#define SCHMITT 1
#define VREF    1
#define OHM_50 3
#define OHM_100 2
#define OHM_200 1
#define OHM_400 0

 // Pin attributes
 static const NvOdmPinAttrib s_pin_config_attributes[] = {

    { NvOdmPinRegister_Ap20_PullUpDown_A,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_A(0x2, 0x2, 0x2, 0x2, 0x2, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0, 0x2, 0x00) },
     
    // Pull ups for the kbc pins
    { NvOdmPinRegister_Ap20_PullUpDown_B,
#if defined (CONFIG_MACH_STAR)
     //KBCD : pull up -> pull down
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_B(0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x2, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0) },
#else
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_B(0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0) },
#endif

    { NvOdmPinRegister_Ap20_PullUpDown_C,
#ifdef CONFIG_MACH_STAR_REV_F
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_C(0x1, 0x1, 0x1, 0x1, 0x2, 0x1, 0x2, 0x1, 0x2, 0x2, 0x2, 0x2, 0x0, 0x0, 0x0) },
#else
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_C(0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x2, 0x1, 0x2, 0x2, 0x2, 0x2, 0x0, 0x0, 0x0) },	//20110120-2, , SPI2 pulldown setting : 5th 0x2->0x1
#endif

	//20100810  LCD one shot mode
    { NvOdmPinRegister_Ap20_PullUpDown_D,
#ifdef CONFIG_MACH_STAR_REV_F
     //UAB : pull up -> pull down
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_D(0x2, 0x1, 0x0, 0x2, 0x2, 0x2, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x1, 0x0, 0x2, 0x2) },
#else
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_D(0x2, 0x2, 0x0, 0x2, 0x2, 0x2, 0x1, 0x1, 0x1, 0x1, 0x2, 0x0, 0x1, 0x0, 0x2, 0x2) },
#endif


    // Pull ups for the kbc pins
    { NvOdmPinRegister_Ap20_PullUpDown_E,
#ifdef CONFIG_MACH_STAR_REV_F
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_E(0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x2) },
#else
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_E(0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x2) },
#endif

    // Set pad control for the sdio2 - - AOCFG1 and AOCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio1 - SDIO1 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO1CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio3 - SDIO2 and SDIO3 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO2CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_SDIO3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio4- ATCCFG1 and ATCCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for I2C1 pins
    { NvOdmPinRegister_Ap20_PadCtrl_DBGCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for I2C2 (DDC) pins
    { NvOdmPinRegister_Ap20_PadCtrl_DDCCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for Dap pins 1 - 4
    { NvOdmPinRegister_Ap20_PadCtrl_DAP1CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP2CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },
//20101210-1, , Nvidia's patch for spi signal strength [START]
#ifdef CONFIG_MACH_STAR_TMUS
	{ NvOdmPinRegister_Ap20_PadCtrl_SPICFGPADCTRL, // for spi2
	NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

	{ NvOdmPinRegister_Ap20_PadCtrl_UADCFGPADCTRL, // for spi1
	NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },
#endif
//20101210-1, , Nvidia's patch for spi signal strength [END]
};


NvU32
NvOdmQueryPinAttributes(const NvOdmPinAttrib** pPinAttributes)
{
    *pPinAttributes = &s_pin_config_attributes[0];
    return NV_ARRAY_SIZE(s_pin_config_attributes);
}

NvBool NvOdmQueryGetPmuProperty(NvOdmPmuProperty* pPmuProperty)
{
#ifdef CONFIG_TEGRA_USB_VBUS_DETECT_BY_PMU
    pPmuProperty->IrqConnected = NV_TRUE;
#else
    pPmuProperty->IrqConnected = NV_FALSE;
#endif

    pPmuProperty->IrqConnected = NV_TRUE; // 20100928, , RTC alarm enable 

    pPmuProperty->PowerGoodCount = 0x7E;
    pPmuProperty->IrqPolarity = NvOdmInterruptPolarity_Low;
    
    // Not there yet, add it later ...
    //pPmuProperty->CpuPowerReqPolarity = ;

    pPmuProperty->CorePowerReqPolarity = NvOdmCorePowerReqPolarity_High;
    pPmuProperty->SysClockReqPolarity = NvOdmSysClockReqPolarity_High;
#ifndef CONFIG_MACH_STAR
	//20100917 one PMIC (max8907B)
    pPmuProperty->CombinedPowerReq = NV_TRUE;
#else
	//20100917 CPU power separated (max8907C + max8952)
    pPmuProperty->CombinedPowerReq = NV_FALSE;
#endif
    pPmuProperty->CpuPowerGoodUs = 2000;
    pPmuProperty->AccuracyPercent = 3;

    //20100917 system hang issue.
#ifndef CONFIG_MACH_STAR
	//20100917 voltage output is restored to default level (max8907)
    pPmuProperty->VCpuOTPOnWakeup = NV_TRUE;
#else
	//20100917 voltage output is restored to before status (max8952)
    pPmuProperty->VCpuOTPOnWakeup = NV_FALSE;
#endif

    /* Setting Power off count for 100 ms  -32KHz clock rate*/
    pPmuProperty->PowerOffCount = 0xc00;
    //20100918  system hang issue [START]
#ifndef CONFIG_MACH_STAR
    pPmuProperty->CpuPowerOffUs = 1000;
#else
    pPmuProperty->CpuPowerOffUs = 1500;
#endif
    //20100918  system hang issue [END]
    return NV_TRUE;
}

/**
 * Gets the lowest soc power state supported by the hardware
 *
 * @returns information about the SocPowerState
 */
const NvOdmSocPowerStateInfo* NvOdmQueryLowestSocPowerState(void)
{

    static                      NvOdmSocPowerStateInfo  PowerStateInfo;
    const static                NvOdmSocPowerStateInfo* pPowerStateInfo = NULL;
    NvOdmServicesKeyListHandle  hKeyList;
    NvU32                       LPStateSelection = 0;

    if (pPowerStateInfo == NULL)
    {
        hKeyList = NvOdmServicesKeyListOpen();
        if (hKeyList)
        {
            LPStateSelection = NvOdmServicesGetKeyValue(hKeyList,
                                                NvOdmKeyListId_ReservedBctCustomerOption);
            NvOdmServicesKeyListClose(hKeyList);
            LPStateSelection = NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, LPSTATE, LPStateSelection);
        }
        // Lowest power state controlled by the flashed custom option.
        PowerStateInfo.LowestPowerState = ((LPStateSelection == TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP1)?
                                            NvOdmSocPowerState_Suspend : NvOdmSocPowerState_DeepSleep);
        pPowerStateInfo = (const NvOdmSocPowerStateInfo*) &PowerStateInfo;
    }
    return (pPowerStateInfo);
}

const NvOdmUsbProperty*
NvOdmQueryGetUsbProperty(NvOdmIoModule OdmIoModule,
                         NvU32 Instance)
{
#ifdef CONFIG_TEGRA_USB_VBUS_DETECT_BY_PMU
#define NVODM_USE_INTERNAL_PHY_VBUS_DETECTION  NV_FALSE
#else
#define NVODM_USE_INTERNAL_PHY_VBUS_DETECTION  NV_TRUE
#endif
    static const NvOdmUsbProperty Usb1Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        (NvOdmUsbChargerType_SE0 | NvOdmUsbChargerType_SE1 | NvOdmUsbChargerType_SK),
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
#ifdef CONFIG_USB_TEGRA_OTG
        NvOdmUsbModeType_OTG,
#else
        NvOdmUsbModeType_Device,
#endif
        NvOdmUsbIdPinType_CableId,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    static const NvOdmUsbProperty Usb2Property =
    {
        NvOdmUsbInterfaceType_UlpiExternalPhy,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_None,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    static const NvOdmUsbProperty Usb2NullPhyProperty =
    {
        NvOdmUsbInterfaceType_UlpiNullPhy,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_Host,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE,
    };

    static const NvOdmUsbProperty Usb3Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_None,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 0)
        return &(Usb1Property);

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 1)
        return &(Usb2Property);  // USB2 is not used.

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 2)
        return &(Usb3Property);

    return (const NvOdmUsbProperty *)NULL;
}

const NvOdmQuerySdioInterfaceProperty* NvOdmQueryGetSdioInterfaceProperty(NvU32 Instance)
{
    return &s_NvOdmQuerySdioInterfaceProperty[Instance];
}

const NvOdmQueryHsmmcInterfaceProperty* NvOdmQueryGetHsmmcInterfaceProperty(NvU32 Instance)
{
    return NULL;
}

NvU32
NvOdmQueryGetBlockDeviceSectorSize(NvOdmIoModule OdmIoModule)
{
    return 0;
}

const NvOdmQueryOwrDeviceInfo* NvOdmQueryGetOwrDeviceInfo(NvU32 Instance)
{
    return NULL;
}

const NvOdmGpioWakeupSource *NvOdmQueryGetWakeupSources(NvU32 *pCount)
{
    *pCount = 0;
    return NULL;
}

/**
 * This function is called from early boot process.
 * Therefore, it cannot use global variables.
 */
NvU32 NvOdmQueryMemSize(NvOdmMemoryType MemType)
{
    
    switch (MemType)
    {
        // NOTE:
        // For Windows CE/WM operating systems the total size of SDRAM may
        // need to be reduced due to limitations in the virtual address map.
        // Under the legacy physical memory manager, Windows OSs have a
        // maximum 512MB statically mapped virtual address space. Under the
        // new physical memory manager, Windows OSs have a maximum 1GB
        // statically mapped virtual address space. Out of that virtual
        // address space, the upper 32 or 36 MB (depending upon the SOC)
        // of the virtual address space is reserved for SOC register
        // apertures.
        //
        // Refer to virtual_tables_apxx.arm for the reserved aperture list.
        // If the cumulative size of the reserved apertures changes, the
        // maximum size of SDRAM will also change.
        case NvOdmMemoryType_Sdram:
            return 0x20000000;
       
        case NvOdmMemoryType_Nor:
        case NvOdmMemoryType_Nand:
        case NvOdmMemoryType_I2CEeprom:
        case NvOdmMemoryType_Hsmmc:
        case NvOdmMemoryType_Mio:
        default:
            return 0;
    }
}

NvU32 NvOdmQueryCarveoutSize(void)
{
    //20100802  increase carveout memory
    return 0x08000000; // 128 MB <- 64MB
}

NvU32 NvOdmQuerySecureRegionSize(void)
{
    return 0x00800000;// 8 MB
}

