/*
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/init.h>

#include <lge/board-bssq.h>
#include <mach-tegra/tegra2_emc.h>
#include <mach-tegra/board.h>

static const struct tegra_emc_table bssq_emc_tables_1gb[] = {
    {
//                  0x20,   /* Rev 2.0 */
                 23750,   /* SDRAM frquency */
//                   950,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
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
            0x00000047,   /* REFRESH */
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
            0x00000060,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xa0b804ae,   /* CFG_DIG_DLL */
            0x007d0810,   /* DLL_XFORM_DQS */
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
//                  0x20,   /* Rev 2.0 */
/* +++ Simon.Je */
#if 1
                 63334,   /* SDRAM frquency */
#else
                 63333,   /* SDRAM frquency */
#endif
/* --- Simon.Je */
//                  1000,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
        {
            0x00000004,   /* RC */
            0x00000009,   /* RFC */
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
            0x000000c4,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x00000009,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000107,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xa0b804ae,   /* CFG_DIG_DLL */
            0x007de010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000006,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
//                  0x20,   /* Rev 2.0 */
                 95000,   /* SDRAM frquency */
//                  1000,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
        {
            0x00000006,   /* RC */
            0x0000000d,   /* RFC */
            0x00000004,   /* RAS */
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
            0x0000013f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000b,   /* RW2PDEN */
            0x0000000e,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000182,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xa0b804ae,   /* CFG_DIG_DLL */
            0x007de010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000009,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },
    {
//                  0x20,   /* Rev 2.0 */
                190000,   /* SDRAM frquency */
//                  1100,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
        {
            0x0000000c,   /* RC */
            0x00000019,   /* RFC */
            0x00000008,   /* RAS */
            0x00000004,   /* RP */
            0x00000007,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000004,   /* RD_RCD */
            0x00000004,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000003,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000d,   /* RDV */
            0x000002bf,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000004,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000c,   /* RW2PDEN */
            0x0000001b,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000a,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000317,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xa06204ae,   /* CFG_DIG_DLL */
            0x007d6810,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000012,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    },

#if 0 // hyongbink@nvidia.com - This can not use with HDMI + DSI panel - 2012/04/02
    { /* LGE_CHANGE_S, bae.cheolhwan@lge.com. 2012-03-20. Add 300Mhz. */
//                  0x20,   /* Rev 2.0 */
                300000,   /* SDRAM frquency */
//                  1200,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
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
            0xe056048b,   /* CFG_DIG_DLL */
            0x007e2010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000001b,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    }, /* LGE_CHANGE_E, bae.cheolhwan@lge.com. 2012-03-20. Add 300Mhz. */
#endif

    {
//                  0x20,   /* Rev 2.0 */
                380000,   /* SDRAM frquency */
//                  1300,   /* EMC core voltage */
//                    46,   /* Number of EMC parameters below */
        {
            0x00000017,   /* RC */
            0x00000032,   /* RFC */
            0x00000010,   /* RAS */
            0x00000007,   /* RP */
            0x00000008,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x0000000b,   /* W2P */
            0x00000007,   /* RD_RCD */
            0x00000007,   /* WR_RCD */
            0x00000004,   /* RRD */
            0x00000003,   /* REXT */
            0x00000003,   /* WDV */
            0x00000007,   /* QUSE */
            0x00000004,   /* QRST */
            0x0000000a,   /* QSAFE */
            0x0000000e,   /* RDV */
            0x0000059f,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000004,   /* PDEX2WR */
            0x00000004,   /* PDEX2RD */
            0x00000007,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x00000011,   /* RW2PDEN */
            0x00000036,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000013,   /* TFAW */
            0x00000008,   /* TRPAB */
            0x00000007,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x0000062d,   /* TREFBW */
            0x00000006,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xe044048b,   /* CFG_DIG_DLL */
            0x007e4010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000023,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
        }
    }
};

static const struct tegra_emc_chip bssq_emc_chips[] = {
	{
		.description = "Samsung 380MHz",
		.mem_manufacturer_id = 0x0101,
		.mem_revision_id1 = 0x0101,
		.mem_revision_id2 = 0x0000,
		.mem_pid = 0x1818,
		.table = bssq_emc_tables_1gb,
		.table_size = ARRAY_SIZE(bssq_emc_tables_1gb)
	},
};

int __init bssq_emc_init(void)
{
	tegra_init_emc(bssq_emc_chips,
		ARRAY_SIZE(bssq_emc_chips));

	return 0;
}
