/*
 *       Filename:  lprintk.c
 *    Description:  
 *
 *         Author:  Jugwan Eom, zugwan@lge.com
 *        Company:  MAS Group, LG Electronics, Inc.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/lprintk.h>

#if defined(CONFIG_DEBUG_FS)
/* XXX: the index must be synced with D_XXX definition in lprintk.h */
struct lprintk_info_struct lge_debug[D_MAX] = {
#ifdef CONFIG_LPRINTK_ALL
        { "KRL", 1 },
        { "SD", 1 },
        { "KEY", 1 },
        { "FS", 1 },
        { "LCD", 1 },
        { "CAM", 1 },
        { "AUDIO", 1 },
        { "TOUCH", 1 },
        { "USB", 1 },
        { "BATT", 1 },
        { "CHGR", 1 },
        { "PWR", 1 },
        { "WIFI", 1 },
        { "SNSR", 1 },
        { "BT", 1 },
        { "FM", 1 },
        { "GPIO", 1 },
        { "MUIC", 1 },
        { "MES", 1 },
        { "SPI", 1 },
        { "MUX", 1 },
        { "RIL", 1 },
        { "PROXI", 1 },
#else
        { "KRL", 0 },
        { "SD", 0 },
        { "KEY", 0 },
        { "FS", 0 },
        { "LCD", 0 },
        { "CAM", 0 },
        { "AUDIO", 0 },
        { "TOUCH", 0 },
        { "USB", 0 },
        { "BATT", 0 },
        { "CHGR", 0 },
        { "PWR", 0 },
        { "WIFI", 0 },
        { "SNSR", 0 },
        { "BT", 0 },
        { "FM", 0 },
        { "GPIO", 0 },
        { "MUIC", 0 },
        { "MES", 0 },
        { "SPI", 0 },
        { "MUX", 0 },
        { "RIL", 0 },
        { "PROXI", 0 },

#endif  /* CONFIG_LPRINTK_ALL */
};

static int lprintk_enable_set(void *data, u64 val)
{
        struct lprintk_info_struct *dbg  = data;

        if (val)
                dbg->enable = 1;
        else
                dbg->enable = 0;

        return 0;
}

static int lprintk_enable_get(void *data, u64 *val)
{
        struct lprintk_info_struct *dbg  = data;

        *val = dbg->enable;

        return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(lprintk_enable_fops, 
                        lprintk_enable_get, lprintk_enable_set, "%llu\n");

static int __init lprintk_init(void)
{
        struct dentry *dent_lprintk;
        unsigned i;

        dent_lprintk = debugfs_create_dir("lprintk", 0);
        if (IS_ERR(dent_lprintk))
                return PTR_ERR(dent_lprintk);

        for (i = 0; i < D_MAX; i++) {
                debugfs_create_file(lge_debug[i].name, 0644, dent_lprintk,
                                    &lge_debug[i], &lprintk_enable_fops);
        }

#ifdef CONFIG_LPRINTK_KERNEL
        lge_debug[D_KERNEL].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_SD
        lge_debug[D_SD].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_KEY
        lge_debug[D_KEY].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_FS
        lge_debug[D_FS].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_LCD
        lge_debug[D_LCD].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_CAMERA
        lge_debug[D_CAMERA].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_AUDIO
        lge_debug[D_AUDIO].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_TOUCH
        lge_debug[D_TOUCH].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_USB
        lge_debug[D_USB].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_BATT
        lge_debug[D_BATT].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_CHARGER
        lge_debug[D_CHARGER].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_POWER
        lge_debug[D_POWER].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_WIFI
        lge_debug[D_WIFI].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_SENSOR
        lge_debug[D_SENSOR].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_BT
        lge_debug[D_BT].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_FM
        lge_debug[D_FM].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_GPIO
        lge_debug[D_GPIO].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_MUIC
        lge_debug[D_MUIC].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_SPI
        lge_debug[D_SPI].enable = 0;
#endif
#ifdef CONFIG_LPRINTK_MUX
        lge_debug[D_MUX].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_RIL
        lge_debug[D_RIL].enable = 1;
#endif
#ifdef CONFIG_LPRINTK_PROXI
        lge_debug[D_PROXI].enable = 1;
#endif
        return 0;
}

late_initcall(lprintk_init);

#endif  /* CONFIG_DEBUG_FS */
