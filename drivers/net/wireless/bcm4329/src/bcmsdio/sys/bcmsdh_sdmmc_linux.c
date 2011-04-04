/*
 * BCMSDH Function Driver for the native SDIO/MMC driver in the Linux Kernel
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: bcmsdh_sdmmc_linux.c,v 1.1.2.5.6.17 2010/08/13 00:36:19 Exp $
 */

#include <typedefs.h>
#include <bcmutils.h>
#include <sdio.h>	/* SDIO Specs */
#include <bcmsdbus.h>	/* bcmsdh to/from specific controller APIs */
#include <sdiovar.h>	/* to get msglevel bit values */

#include <linux/sched.h>	/* request_irq() */

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-11-19, Support Host Wakeup */
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)

#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/wakelock.h>

#define GPIO_WLAN_HOST_WAKE CONFIG_BCM4329_GPIO_WL_HOSTWAKEUP

//struct wake_lock wlan_host_wakelock; 		//by sjpark 10-12-22 : idle time current (not used)
struct wake_lock wlan_host_wakelock_resume;
int dhd_suspend_context = FALSE;

extern int del_wl_timers(void);
extern void register_mmc_card_pm(struct early_suspend *);
extern void unregister_mmc_card_pm(void);
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-11-19, Support Host Wakeup */

#if !defined(SDIO_VENDOR_ID_BROADCOM)
#define SDIO_VENDOR_ID_BROADCOM		0x02d0
#endif /* !defined(SDIO_VENDOR_ID_BROADCOM) */

#define SDIO_DEVICE_ID_BROADCOM_DEFAULT	0x0000

#if !defined(SDIO_DEVICE_ID_BROADCOM_4325_SDGWB)
#define SDIO_DEVICE_ID_BROADCOM_4325_SDGWB	0x0492	/* BCM94325SDGWB */
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4325_SDGWB) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4325)
#define SDIO_DEVICE_ID_BROADCOM_4325	0x0493
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4325) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4329)
#define SDIO_DEVICE_ID_BROADCOM_4329	0x4329
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4329) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4319)
#define SDIO_DEVICE_ID_BROADCOM_4319	0x4319
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4319) */

#include <bcmsdh_sdmmc.h>

#include <dhd_dbg.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)
extern volatile bool dhd_mmc_suspend;
#endif

extern void sdioh_sdmmc_devintr_off(sdioh_info_t *sd);
extern void sdioh_sdmmc_devintr_on(sdioh_info_t *sd);

int sdio_function_init(void);
void sdio_function_cleanup(void);

#define DESCRIPTION "bcmsdh_sdmmc Driver"
#define AUTHOR "Broadcom Corporation"

/* module param defaults */
static int clockoverride = 0;

module_param(clockoverride, int, 0644);
MODULE_PARM_DESC(clockoverride, "SDIO card clock override");

PBCMSDH_SDMMC_INSTANCE gInstance;

/* Maximum number of bcmsdh_sdmmc devices supported by driver */
#define BCMSDH_SDMMC_MAX_DEVICES 1

extern int bcmsdh_probe(struct device *dev);
extern int bcmsdh_remove(struct device *dev);
struct device sdmmc_dev;

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
extern int dhdsdio_bussleep(void *bus, bool sleep);
extern bool dhdsdio_dpc(void *bus);
extern int dhd_os_proto_block(void *pub);
extern int dhd_os_proto_unblock(void * pub);
extern void *dhd_es_get_dhd_pub(void);
extern void *dhd_es_get_dhd_bus_sdh(void);
static int dhd_register_early_suspend(void);
static int dhd_unregister_early_suspend(void);
void dhd_early_suspend(struct early_suspend *h);
void dhd_late_resume(struct early_suspend *h);
static struct early_suspend early_suspend_data = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 20,
	.suspend = dhd_early_suspend,
	.resume = dhd_late_resume
};
DECLARE_WAIT_QUEUE_HEAD(bussleep_wake);
typedef struct dhd_early_suspend {
	bool state;
	bool drv_loaded;
	struct dhd_bus_t *bus;
} dhd_early_suspend_t;
dhd_early_suspend_t dhd_early_suspend_ctrl = { 0, 0, 0};
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

static int bcmsdh_sdmmc_probe(struct sdio_func *func,
                              const struct sdio_device_id *id)
{
	int ret = 0;
	static struct sdio_func sdio_func_0;
	sd_trace(("bcmsdh_sdmmc: %s Enter\n", __FUNCTION__));
	sd_trace(("sdio_bcmsdh: func->class=%x\n", func->class));
	sd_trace(("sdio_vendor: 0x%04x\n", func->vendor));
	sd_trace(("sdio_device: 0x%04x\n", func->device));
	sd_trace(("Function#: 0x%04x\n", func->num));

	if (func->num == 1) {
		sdio_func_0.num = 0;
		sdio_func_0.card = func->card;
		gInstance->func[0] = &sdio_func_0;
		if(func->device == 0x4) { /* 4318 */
			gInstance->func[2] = NULL;
			sd_trace(("NIC found, calling bcmsdh_probe...\n"));
			ret = bcmsdh_probe(&sdmmc_dev);
		}
	}

	gInstance->func[func->num] = func;

	if (func->num == 2) {
		sd_trace(("F2 found, calling bcmsdh_probe...\n"));
		ret = bcmsdh_probe(&sdmmc_dev);
	}

	return ret;
}

static void bcmsdh_sdmmc_remove(struct sdio_func *func)
{
	sd_trace(("bcmsdh_sdmmc: %s Enter\n", __FUNCTION__));
	sd_info(("sdio_bcmsdh: func->class=%x\n", func->class));
	sd_info(("sdio_vendor: 0x%04x\n", func->vendor));
	sd_info(("sdio_device: 0x%04x\n", func->device));
	sd_info(("Function#: 0x%04x\n", func->num));

	if (func->num == 2) {
		sd_trace(("F2 found, calling bcmsdh_remove...\n"));
		bcmsdh_remove(&sdmmc_dev);
	}
}

/* devices we support, null terminated */
static const struct sdio_device_id bcmsdh_sdmmc_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_DEFAULT) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4325_SDGWB) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4325) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4329) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4319) },
	{ /* end: all zeroes */				},
};

MODULE_DEVICE_TABLE(sdio, bcmsdh_sdmmc_ids);

static struct sdio_driver bcmsdh_sdmmc_driver = {
	.probe		= bcmsdh_sdmmc_probe,
	.remove		= bcmsdh_sdmmc_remove,
	.name		= "bcmsdh_sdmmc",
	.id_table	= bcmsdh_sdmmc_ids,
	};

struct sdos_info {
	sdioh_info_t *sd;
	spinlock_t lock;
};


int
sdioh_sdmmc_osinit(sdioh_info_t *sd)
{
	struct sdos_info *sdos;

	sdos = (struct sdos_info*)MALLOC(sd->osh, sizeof(struct sdos_info));
	sd->sdos_info = (void*)sdos;
	if (sdos == NULL)
		return BCME_NOMEM;

	sdos->sd = sd;
	spin_lock_init(&sdos->lock);
	return BCME_OK;
}

void
sdioh_sdmmc_osfree(sdioh_info_t *sd)
{
	struct sdos_info *sdos;
	ASSERT(sd && sd->sdos_info);

	sdos = (struct sdos_info *)sd->sdos_info;
	MFREE(sd->osh, sdos, sizeof(struct sdos_info));
}

#if defined(CONFIG_HAS_EARLYSUSPEND)

void
dhd_es_set_dhd_bus(void *bus)
{
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
	dhd_early_suspend_ctrl.bus = bus;
#endif
}

void *
dhd_es_get_dhd_bus(void)
{
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
	return dhd_early_suspend_ctrl.bus;
#else
	return NULL;
#endif
}

bool
dhd_early_suspend_state(void)
{
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
	return dhd_early_suspend_ctrl.state;
#else
	return 0;
#endif
}

#endif

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)

static int
dhd_es_lock_dhd_bus(void)
{
/* LGE_CHANGE_S, [dongp.kim@lge.com], 2010-04-22, WBT Fix */
// WBT Fix TD# 36996
    	void *bus;
	bus = dhd_es_get_dhd_pub();
	if ( bus )
		dhd_os_proto_block(bus);
/* LGE_CHANGE_S, [dongp.kim@lge.com], 2010-04-22, WBT Fix */
	return 0;
}

static int
dhd_es_unlock_dhd_bus(void)
{
/* LGE_CHANGE_S, [dongp.kim@lge.com], 2010-04-22, WBT Fix */
// WBT Fix TD# 36997
	void *bus;
	bus = dhd_es_get_dhd_pub();
	if ( bus )
		dhd_os_proto_unblock(bus);
/* LGE_CHANGE_S, [dongp.kim@lge.com], 2010-04-22, WBT Fix */
	return 0;
}

static void dhd_enable_sdio_irq(int enable)
{
	struct mmc_card *card;
	struct mmc_host *host;

	if(gInstance->func[0] == NULL){
		printk("[sdmmc]%s:%d - PBCMSDH_SDMMC_INSTANCE func[0] is NULL\n",__func__,__LINE__);
		return;
	}

	card = gInstance->func[0]->card;
	host = card->host;

	host->ops->enable_sdio_irq(host, enable);
}

static int dhd_suspend(void)
{
	int bus_state;
	int max_tries = 3;
	int gpio = 0;

	printk("[sdmmc]%s:%d - enter\n",__func__,__LINE__);
	DHD_TRACE(("%s: SUSPEND Enter\n", __FUNCTION__));

	if (NULL != dhd_early_suspend_ctrl.bus) {
		dhd_es_lock_dhd_bus();

		do {
			bus_state = dhdsdio_bussleep(dhd_early_suspend_ctrl.bus, TRUE);
			if (bus_state == BCME_BUSY)
			{

				/* 250ms timeout */
				wait_event_timeout(bussleep_wake, FALSE, HZ/4);
				DHD_TRACE(("%s in loop\n", __FUNCTION__));
			}
		} while ((bus_state == BCME_BUSY) && (max_tries-- > 0));

		if(max_tries <= 0)
		{
			DHD_ERROR(("[sdmmc]%s:%d - bus_state is BUSY, dhdsdio_bussleep Fail\n",__func__,__LINE__));
			dhd_es_unlock_dhd_bus();
			/* This value should be returned to mmc_suspend*/
			return -1;
		}

		dhd_early_suspend_ctrl.state = TRUE;
		gpio = gpio_get_value(GPIO_WLAN_HOST_WAKE);
		printk("[sdmmc]%s:%d - GPIO_WLAN_HOST_WAKE [%d:%d]\n",__func__,__LINE__,GPIO_WLAN_HOST_WAKE,gpio);
		DHD_TRACE(("%s: SUSPEND Done gpio->%d\n", __FUNCTION__, gpio));
	}
	else
		DHD_ERROR(("[sdmmc]%s:%d - dhd_early_suspend_t bus is NULL\n",__func__,__LINE__));

	printk("[sdmmc]%s:%d - end\n",__func__,__LINE__);
	return 0;
}

static int dhd_resume(void)
{
	int gpio = 0;

	dhd_enable_sdio_irq(FALSE);
	dhd_suspend_context = FALSE;

	printk("[sdmmc]%s:%d - enter\n",__func__,__LINE__);
	DHD_TRACE(("%s: RESUME Enter\n", __FUNCTION__));

	if (NULL != dhd_early_suspend_ctrl.bus) {
		dhd_early_suspend_ctrl.state = FALSE;

		//wake_lock_timeout(&wlan_host_wakelock_resume, 2*HZ);
		wake_lock(&wlan_host_wakelock_resume);			//by sjpark 10-12-22 : idle time current 

		dhdsdio_dpc(dhd_early_suspend_ctrl.bus);
		dhd_es_unlock_dhd_bus();

		gpio = gpio_get_value(GPIO_WLAN_HOST_WAKE);
		printk("[sdmmc]%s:%d - GPIO_WLAN_HOST_WAKE [%d:%d]\n",__func__,__LINE__,GPIO_WLAN_HOST_WAKE,gpio);
		DHD_TRACE(("%s: RESUME Done gpio->%d\n", __FUNCTION__, gpio));
		wake_unlock(&wlan_host_wakelock_resume);			//by sjpark 10-12-22 : idle time current 
	}
	else
		DHD_ERROR(("[sdmmc]%s:%d - dhd_early_suspend_t bus is NULL\n",__func__,__LINE__));

	return 0;
}

#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

/* Interrupt enable/disable */
SDIOH_API_RC
sdioh_interrupt_set(sdioh_info_t *sd, bool enable)
{
	ulong flags;
	struct sdos_info *sdos;

	sd_trace(("%s: %s\n", __FUNCTION__, enable ? "Enabling" : "Disabling"));

	sdos = (struct sdos_info *)sd->sdos_info;
	ASSERT(sdos);

#if !defined(OOB_INTR_ONLY)
	if (enable && !(sd->intr_handler && sd->intr_handler_arg)) {
		sd_err(("%s: no handler registered, will not enable\n", __FUNCTION__));
		return SDIOH_API_RC_FAIL;
	}
#endif /* !defined(OOB_INTR_ONLY) */

	/* Ensure atomicity for enable/disable calls */
	spin_lock_irqsave(&sdos->lock, flags);

	sd->client_intr_enabled = enable;
	if (enable) {
		sdioh_sdmmc_devintr_on(sd);
	} else {
		sdioh_sdmmc_devintr_off(sd);
	}

	spin_unlock_irqrestore(&sdos->lock, flags);

	return SDIOH_API_RC_SUCCESS;
}


#ifdef BCMSDH_MODULE
static int __init
bcmsdh_module_init(void)
{
	int error = 0;
	sdio_function_init();
	return error;
}

static void __exit
bcmsdh_module_cleanup(void)
{
	sdio_function_cleanup();
}

module_init(bcmsdh_module_init);
module_exit(bcmsdh_module_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_AUTHOR(AUTHOR);

#endif /* BCMSDH_MODULE */
/*
 * module init
*/
int sdio_function_init(void)
{
	int error = 0;
	sd_trace(("bcmsdh_sdmmc: %s Enter\n", __FUNCTION__));

	gInstance = kzalloc(sizeof(BCMSDH_SDMMC_INSTANCE), GFP_KERNEL);
	if (!gInstance)
		return -ENOMEM;

	bzero(&sdmmc_dev, sizeof(sdmmc_dev));
	error = sdio_register_driver(&bcmsdh_sdmmc_driver);

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
	if (!error) {
		dhd_register_early_suspend();
		DHD_TRACE(("%s: registered with Android PM\n", __FUNCTION__));
	}
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

	return error;
}

/*
 * module cleanup
*/
extern int bcmsdh_remove(struct device *dev);
void sdio_function_cleanup(void)
{
	sd_trace(("%s Enter\n", __FUNCTION__));

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
	sdio_claim_host(gInstance->func[0]);
	dhd_enable_sdio_irq(FALSE);
	sdio_release_host(gInstance->func[0]);
	dhd_unregister_early_suspend();
	dhd_suspend_context = TRUE;
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

	sdio_unregister_driver(&bcmsdh_sdmmc_driver);

	if (gInstance)
		kfree(gInstance);
}


#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)

extern int dhdsdio_set_dtim(void *bus, int enalbe);		//by sjpark 10-12-15

void dhd_early_suspend(struct early_suspend *h)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)
            dhd_mmc_suspend = FALSE;
#endif
	DHD_TRACE(("%s: enter\n", __FUNCTION__));

	dhd_suspend_context = TRUE;
	
	/* If chip active is done, do put the device to suspend */
	del_wl_timers();

	dhdsdio_set_dtim(dhd_early_suspend_ctrl.bus, TRUE);		//by sjpark 10-12-15

	if(dhd_suspend() < 0) {
		dhd_enable_sdio_irq(TRUE); /* make sure one more for testing, later */
		DHD_ERROR(("%s: dhd_suspend() failed\n", __FUNCTION__));
		return;
	} 
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)
        dhd_mmc_suspend = TRUE;
#endif
}
EXPORT_SYMBOL(dhd_early_suspend);

void dhd_late_resume(struct early_suspend *h)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)
            dhd_mmc_suspend = FALSE;
#endif

	printk(KERN_ERR "%s: enter\n", __FUNCTION__);
	DHD_TRACE(("%s: enter\n", __FUNCTION__));

	if(dhd_suspend_context == TRUE ){

	/*Do the resume operations*/
	dhd_resume();
	}
	else 
		printk("%s: Do not dhd_suspend mode setting.\n",__FUNCTION__);

	dhdsdio_set_dtim(dhd_early_suspend_ctrl.bus, FALSE);		//by sjpark 10-12-15

	return;
}
EXPORT_SYMBOL(dhd_late_resume);

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int
dhd_register_hwakeup(void)
{
	int ret=0;

	printk("[sdmmc]%s:%d - enter\n",__func__,__LINE__);

	/* wake lock initialize */
	printk("[sdmmc]%s:%d - wake lock initialize\n",__func__,__LINE__);
   	//wake_lock_init(&wlan_host_wakelock, WAKE_LOCK_SUSPEND, "WLAN_HOST_WAKE");		//by sjpark 10-12-22 : idle time current (not used)
   	wake_lock_init(&wlan_host_wakelock_resume, WAKE_LOCK_SUSPEND, "WLAN_HOST_WAKE_RESUME");	

	printk("[sdmmc]%s:%d - end\n",__func__,__LINE__);
	return ret;
}

static void
dhd_unregister_hwakeup(void)
{
	printk("[sdmmc]%s:%d - Do not anything\n",__func__,__LINE__);
}

static int
dhd_register_early_suspend(void)
{
	printk("[sdmmc]%s:%d - enter\n",__func__,__LINE__);

	/* LGE_CHANGE_S [yoohoo@lge.com] 2009-01-14, Support Host Wakeup */
	dhd_early_suspend_ctrl.drv_loaded = TRUE;

	register_mmc_card_pm(&early_suspend_data);
	dhd_register_hwakeup();
	printk("[sdmmc]%s:%d - GPIO_WLAN_HOST_WAKE [%d:%d]\n",
			__func__,__LINE__, GPIO_WLAN_HOST_WAKE, gpio_get_value(GPIO_WLAN_HOST_WAKE));
	/* LGE_CHANGE_E [yoohoo@lge.com] 2010-01-14, Support Host Wakeup */

	printk("[sdmmc]%s:%d - end\n",__func__,__LINE__);
	return 0;
}

static int
dhd_unregister_early_suspend(void)
{
	printk("[sdmmc]%s:%d - enter\n",__func__,__LINE__);

	/* LGE_CHANGE_S [yoohoo@lge.com] 2009-01-14, Support Host Wakeup */
	if (dhd_early_suspend_ctrl.drv_loaded == FALSE) 
		return 0;
	dhd_unregister_hwakeup();
	printk("[sdmmc]%s:%d - GPIO_WLAN_HOST_WAKE [%d:%d]\n",
			__func__,__LINE__, GPIO_WLAN_HOST_WAKE, gpio_get_value(GPIO_WLAN_HOST_WAKE));
	unregister_mmc_card_pm();
	/* Destroy the wake lock */
	//wake_lock_destroy(&wlan_host_wakelock);			//by sjpark 10-12-22 : idle time current (not used)
	wake_lock_destroy(&wlan_host_wakelock_resume);
	/* LGE_CHANGE_E [yoohoo@lge.com] 2010-01-14, Support Host Wakeup */

	printk("[sdmmc]%s:%d - end\n",__func__,__LINE__);
	return 0;
}

#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */