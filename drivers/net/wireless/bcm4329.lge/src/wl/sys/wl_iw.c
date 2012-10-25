/*
 * Linux Wireless Extensions support
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *         Unless you and Broadcom execute a separate written software license
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
 * $Id: wl_iw.c,v 1.51.4.9.2.6.4.36.2.29.2.1 2010/05/04 01:52:50 Exp $
 */


#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>

#include <bcmutils.h>
#include <bcmendian.h>
#include <proto/ethernet.h>

#include <linux/if_arp.h>
#include <asm/uaccess.h>

#ifndef BCMDONGLEHOST
#include <wlc_cfg.h>
#else
#include <dngl_stats.h>
#include <dhd.h>
#endif
#include <dhdioctl.h>

typedef void wlc_info_t;
typedef void wl_info_t;
typedef const struct si_pub  si_t;
#include <wlioctl.h>

#ifndef BCMDONGLEHOST
#include <wlc_pub.h>
#include <wl_dbg.h>
#else
#include <proto/ethernet.h>
#include <dngl_stats.h>
#include <dhd.h>
#define WL_ERROR(x)	printk x // LGE
#define WL_TRACE(x)
#define WL_ASSOC(x)
#define WL_INFORM(x)
#define WL_WSEC(x)
#endif
#define LGE_ROAM_PARAMETER

#include <wl_iw.h>

// louislee : added for WPS {
#ifdef BCMWAPI_WPI
/* these items should evetually go into wireless.h of the linux system headfile dir */
#ifndef IW_ENCODE_ALG_SM4
#define IW_ENCODE_ALG_SM4 0x20
#endif

#ifndef IW_AUTH_WAPI_ENABLED
#define IW_AUTH_WAPI_ENABLED 0x20
#endif

#ifndef IW_AUTH_WAPI_VERSION_1
#define IW_AUTH_WAPI_VERSION_1	0x00000008
#endif

#ifndef IW_AUTH_CIPHER_SMS4
#define IW_AUTH_CIPHER_SMS4	0x00000020
#endif

#ifndef IW_AUTH_KEY_MGMT_WAPI_PSK
#define IW_AUTH_KEY_MGMT_WAPI_PSK 4
#endif

#ifndef IW_AUTH_KEY_MGMT_WAPI_CERT
#define IW_AUTH_KEY_MGMT_WAPI_CERT 8
#endif
#endif /* BCMWAPI_WPI */

/* XXX because for dhd we don't include bcmwpa.h, define this here */
/* XXX this does not look at CCX ciphers, e.g. CKIP_*...if this is relevant in this file,
 * please add the definitions here
 */

#ifdef BCMWAPI_WPI
#define IW_WSEC_ENABLED(wsec)	((wsec) & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED | SMS4_ENABLED))
#else /* BCMWAPI_WPI */
#define IW_WSEC_ENABLED(wsec)	((wsec) & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED))
#endif /* BCMWAPI_WPI */
// louislee : added for WPS }

#include <linux/rtnetlink.h>

#if defined(BCMDONGLEHOST)
#ifdef  CUSTOMER_HW2
//#define WL_IW_USE_ISCAN
#define WL_IW_USE_ESCAN // louislee : modified for TI
#else
#define WL_IW_USE_ESCAN
#endif
#define ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS  1
#endif 

#if defined(SOFTAP)
inline static void MUTEX_LOCK_SOFTAP_SET_INIT(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
        mutex_init(&dhdp->wl_softap_lock);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) */
}

inline static void MUTEX_LOCK_SOFTAP_SET(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
        mutex_lock(&dhdp->wl_softap_lock);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) */
}

inline static void MUTEX_UNLOCK_SOFTAP_SET(dhd_pub_t * dhdp)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)) && 1
        mutex_unlock(&dhdp->wl_softap_lock);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) */
}

#define WL_SOFTAP(x) printk x
static struct net_device *priv_dev;
static bool 	ap_cfg_running = FALSE;
static bool 	ap_fw_loaded = FALSE;
struct net_device *ap_net_dev = NULL;
struct semaphore  ap_eth_sema;
static int wl_iw_set_ap_security(struct net_device *dev, struct ap_profile *ap);
static int wl_iw_softap_deassoc_stations(struct net_device *dev);
#endif /* SOFTAP */

static int wl_keep_alive_set(struct net_device *dev, char* extra);
#define WL_IW_IOCTL_CALL(func_call) \
	do {				\
		func_call;		\
	} while (0)

static int		g_onoff = G_WLAN_SET_ON;

#if defined(STA) || defined(BCMDONGLEHOST)
extern bool wl_iw_conn_status_str(uint32 event_type, uint32 status,
	uint32 reason, char* stringBuf, uint buflen);
#include <bcmsdbus.h>
extern void dhd_customer_gpio_wlan_ctrl(int onoff);
extern uint dhd_dev_reset(struct net_device *dev, uint8 flag);
extern void dhd_dev_init_ioctl(struct net_device *dev);
#endif 

#ifdef BCMDONGLEHOST
uint wl_msg_level = WL_ERROR_VAL;
#endif

#define MAX_WLIW_IOCTL_LEN 1024


#if defined(BCMDONGLEHOST) && defined(IL_BIGENDIAN)
#include <bcmendian.h>
#define htod32(i) (bcmswap32(i))
#define htod16(i) (bcmswap16(i))
#define dtoh32(i) (bcmswap32(i))
#define dtoh16(i) (bcmswap16(i))
#define htodchanspec(i) htod16(i)
#define dtohchanspec(i) dtoh16(i)
#else
#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i
#define htodchanspec(i) i
#define dtohchanspec(i) i
#endif

#ifdef CONFIG_WIRELESS_EXT

#if defined(BCMDONGLEHOST)
extern struct iw_statistics *dhd_get_wireless_stats(struct net_device *dev);
extern int dhd_wait_pend8021x(struct net_device *dev);
#else
extern struct iw_statistics *wl_get_wireless_stats(struct net_device *dev);
#endif 
#endif 

#if WIRELESS_EXT < 19
#define IW_IOCTL_IDX(cmd)	((cmd) - SIOCIWFIRST)
#define IW_EVENT_IDX(cmd)	((cmd) - IWEVFIRST)
#endif 

static void *g_scan = NULL;
static uint g_scan_specified_ssid;

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-08-16, Fix the getting ISCAN/SCAN results */
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN)
static uint g_completed_scan_type = 0; /* 0: ISCAN, 1: SCAN */
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-08-16, Fix the getting ISCAN/SCAN results */

#if !defined(CONFIG_BRCM_USE_DEEPSLEEP)
/* Do not use GPIO Reset at On/Off. Use mpc. */
//#define CONFIG_BRCM_USE_GPIO_RESET
#endif	/* !defined(CONFIG_BRCM_USE_DEEPSLEEP) */

static wlc_ssid_t g_ssid;

wl_iw_ss_cache_ctrl_t g_ss_cache_ctrl;	

#if defined(BCMDONGLEHOST)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define DAEMONIZE(a) daemonize(a); \
	allow_signal(SIGKILL); \
	allow_signal(SIGTERM);
#else 
#define RAISE_RX_SOFTIRQ() \
	cpu_raise_softirq(smp_processor_id(), NET_RX_SOFTIRQ)
#define DAEMONIZE(a) daemonize(); \
	do { if (a) \
		strncpy(current->comm, a, MIN(sizeof(current->comm), (strlen(a) + 1))); \
	} while (0);
#endif 

#if defined(WL_IW_USE_ISCAN)

#define ISCAN_STATE_IDLE   0
#define ISCAN_STATE_SCANING 1


#define WLC_IW_ISCAN_MAXLEN   2048
typedef struct iscan_buf {
	struct iscan_buf * next;
	char   iscan_buf[WLC_IW_ISCAN_MAXLEN];
} iscan_buf_t;

typedef struct iscan_info {
	struct net_device *dev;
	struct timer_list timer;
	uint32 timer_ms;
	uint32 timer_on;
	int    iscan_state;
	iscan_buf_t * list_hdr;
	iscan_buf_t * list_cur;

	
	long sysioc_pid;
	struct semaphore sysioc_sem;
	struct completion sysioc_exited;

	uint32 scan_flag;	

	char ioctlbuf[WLC_IOCTL_SMLEN];
} iscan_info_t;
iscan_info_t *g_iscan = NULL;
static void wl_iw_timerfunc(ulong data);
static int
wl_iw_iscan(iscan_info_t *iscan, wlc_ssid_t *ssid, uint16 action);
#endif 

#if defined(WL_IW_USE_ESCAN)

#define ESCAN_STATE_IDLE   0
#define ESCAN_STATE_SCANING 1
#define ESCAN_BUF_SIZE (64 * 1024)
#ifdef CONFIG_LGE_BCM432X_PATCH // escan buf to array
char escan_array[ESCAN_BUF_SIZE]={0,};
#endif /* CONFIG_LGE_BCM432X_PATCH escan buf to array */
typedef struct escan_info {
	struct net_device *dev;
	int escan_state;
#ifdef CONFIG_LGE_BCM432X_PATCH // escan buf to array
	char *escan_buf;
#else
	char escan_buf[ESCAN_BUF_SIZE];
#endif /* CONFIG_LGE_BCM432X_PATCH escan buf to array */

	uint32 scan_flag;	

	char ioctlbuf[WLC_IOCTL_SMLEN];
} escan_info_t;
escan_info_t *g_escan = NULL;
static int wl_iw_escan(escan_info_t *escan, wlc_ssid_t *ssid, uint16 action);
#endif 

#define  COEX_DHCP 1 
#ifdef COEX_DHCP
static void wl_iw_bt_flag_set(struct net_device *dev, bool set);
static void wl_iw_bt_release(void);

typedef enum bt_coex_status {
	BT_DHCP_IDLE = 0,
	BT_DHCP_START,
	BT_DHCP_OPPORTUNITY_WINDOW,
	BT_DHCP_FLAG_FORCE_TIMEOUT
} coex_status_t;
#define BT_DHCP_OPPORTUNITY_WINDOW_TIEM	2500	
#define BT_DHCP_FLAG_FORCE_TIME				5500 	

typedef struct bt_info {
	struct net_device *dev;
	struct timer_list timer;
	uint32 timer_ms;
	uint32 timer_on;
	int	bt_state;

	
	long bt_pid;
	struct semaphore bt_sem;
	struct completion bt_exited;
} bt_info_t;

bt_info_t *g_bt = NULL;
static void wl_iw_bt_timerfunc(ulong data);
#endif 
static void wl_iw_set_event_mask(struct net_device *dev);
#endif 
static int
wl_iw_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
);
static int
wl_iw_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
);

static uint
wl_iw_get_scan_prep(
	wl_scan_results_t *list,
	struct iw_request_info *info,
	char *extra,
	short max_size
);

static void swap_key_from_BE(
	        wl_wsec_key_t *key
)
{
	key->index = htod32(key->index);
	key->len = htod32(key->len);
	key->algo = htod32(key->algo);
	key->flags = htod32(key->flags);
	key->rxiv.hi = htod32(key->rxiv.hi);
	key->rxiv.lo = htod16(key->rxiv.lo);
	key->iv_initialized = htod32(key->iv_initialized);
}

static void swap_key_to_BE(
	        wl_wsec_key_t *key
)
{
	key->index = dtoh32(key->index);
	key->len = dtoh32(key->len);
	key->algo = dtoh32(key->algo);
	key->flags = dtoh32(key->flags);
	key->rxiv.hi = dtoh32(key->rxiv.hi);
	key->rxiv.lo = dtoh16(key->rxiv.lo);
	key->iv_initialized = dtoh32(key->iv_initialized);
}

static int
dev_wlc_ioctl(
	struct net_device *dev,
	int cmd,
	void *arg,
	int len
)
{
	struct ifreq ifr;
	wl_ioctl_t ioc;
	mm_segment_t fs;
	int ret = -1;

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return ret;
	}

	WL_TRACE(("\n%s, PID:%x: send Local IOCTL -> dhd: cmd:0x%x, buf:%p, len:%d ,\n",
		__FUNCTION__, current->pid, cmd, arg, len));
	memset(&ioc, 0, sizeof(ioc));
	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;

	strcpy(ifr.ifr_name, dev->name);
	ifr.ifr_data = (caddr_t) &ioc;

	
	dev_open(dev);

	fs = get_fs();
	set_fs(get_ds());
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	ret = dev->do_ioctl(dev, &ifr, SIOCDEVPRIVATE);
#else
	ret = dev->netdev_ops->ndo_do_ioctl(dev, &ifr, SIOCDEVPRIVATE);
#endif
	set_fs(fs);

	return ret;
}


#if !defined(CONFIG_LGE_BCM432X_PATCH)
static int
dev_wlc_intvar_get_reg(
	struct net_device *dev,
	char *name,
	uint  reg,
	int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	uint len;
	len = bcm_mkiovar(name, (char *)(&reg), sizeof(reg), (char *)(&var), sizeof(var.buf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)&var, len);

	*retval = dtoh32(var.val);
	return (error);
}
#endif	/* !defined(CONFIG_LGE_BCM432X_PATCH) */




static int
dev_wlc_intvar_set(
	struct net_device *dev,
	char *name,
	int val)
{
	char buf[WLC_IOCTL_SMLEN];
	uint len;

	val = htod32(val);
	len = bcm_mkiovar(name, (char *)(&val), sizeof(val), buf, sizeof(buf));
	ASSERT(len);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len));
}

#if defined(BCMDONGLEHOST) && (defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN))
static int
dev_iw_iovar_setbuf(
	struct net_device *dev,
	char *iovar,
	void *param,
	int paramlen,
	void *bufptr,
	int buflen)
{
	int iolen;

	iolen = bcm_mkiovar(iovar, param, paramlen, bufptr, buflen);
	ASSERT(iolen);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, bufptr, iolen));
}

#endif 

#if defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN)
static int
dev_iw_iovar_getbuf(
	struct net_device *dev,
	char *iovar,
	void *param,
	int paramlen,
	void *bufptr,
	int buflen)
{
	int iolen;

	iolen = bcm_mkiovar(iovar, param, paramlen, bufptr, buflen);
	ASSERT(iolen);

	return (dev_wlc_ioctl(dev, WLC_GET_VAR, bufptr, buflen));
}
#endif 

#if WIRELESS_EXT > 17
static int
dev_wlc_bufvar_set(
	struct net_device *dev,
	char *name,
	char *buf, int len)
{
	char ioctlbuf[WLC_IOCTL_SMLEN];
	uint buflen;

	buflen = bcm_mkiovar(name, buf, len, ioctlbuf, sizeof(ioctlbuf));
	ASSERT(buflen);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, ioctlbuf, buflen));
}
#endif


static int
dev_wlc_bufvar_get(
	struct net_device *dev,
	char *name,
	char *buf, int buflen)
{
	char ioctlbuf[MAX_WLIW_IOCTL_LEN];
	int error;

	uint len;

	len = bcm_mkiovar(name, NULL, 0, ioctlbuf, sizeof(ioctlbuf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)ioctlbuf, MAX_WLIW_IOCTL_LEN);
	if (!error)
		bcopy(ioctlbuf, buf, buflen);

	return (error);
}



static int
dev_wlc_intvar_get(
	struct net_device *dev,
	char *name,
	int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	uint len;
	uint data_null;

	len = bcm_mkiovar(name, (char *)(&data_null), 0, (char *)(&var), sizeof(var.buf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)&var, len);

	*retval = dtoh32(var.val);

	return (error);
}


#if WIRELESS_EXT > 12
static int
wl_iw_set_active_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int as = 0;
	int error = 0;
	char *p = extra;

#if defined(WL_IW_USE_ISCAN)
	if (g_iscan->iscan_state == ISCAN_STATE_IDLE)
#elif defined(WL_IW_USE_ESCAN)
	if (g_escan->escan_state == ESCAN_STATE_IDLE)
#endif 
		error = dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as));
#if defined(WL_IW_USE_ISCAN)
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-04-15, fixed passive/active scan */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	else
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-04-15, fixed passive/active scan */
		g_iscan->scan_flag = as;
#elif defined(WL_IW_USE_ESCAN)
	else
		g_escan->scan_flag = as;
#endif 
	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_set_passive_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int ps = 1;
	int error = 0;
	char *p = extra;

#if defined(WL_IW_USE_ISCAN)
	if (g_iscan->iscan_state == ISCAN_STATE_IDLE) {
#elif defined(WL_IW_USE_ESCAN)
	if (g_escan->escan_state == ESCAN_STATE_IDLE) {
#endif 

		 
		if (g_scan_specified_ssid == 0) {
			error = dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &ps, sizeof(ps));
		}
#if defined(WL_IW_USE_ISCAN)
	}
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-04-15, fixed passive/active scan */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	else
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-04-15, fixed passive/active scan */
		g_iscan->scan_flag = ps;
#elif defined(WL_IW_USE_ESCAN)
	}
	else
		g_escan->scan_flag = ps;
#endif 

	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_get_macaddr(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error;
	char buf[128];
	struct ether_addr *id;
	char *p = extra;

	
	strcpy(buf, "cur_etheraddr");
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, buf, sizeof(buf));
	id = (struct ether_addr *) buf;
	p += snprintf(p, MAX_WX_STRING, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
		id->octet[0], id->octet[1], id->octet[2],
		id->octet[3], id->octet[4], id->octet[5]);
	wrqu->data.length = p - extra + 1;

	return error;
}

/*
static int
wl_iw_set_scan_unassoc_time_ms(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
    int timems = 0;
    int ret = 0;
	char *p = extra;

    ASSERT( dev && wrqu );


	if (sscanf(extra, "%*s %d", &timems) != 1)
		return -EINVAL;
	
	
    if( timems < 0 ) 
    {
		printk(KERN_ERR "%s: time value is out of range. value[%d]\n",\
                __FUNCTION__, timems);
        return -1;
    }

	if( ( ret = dev_wlc_ioctl(dev, WLC_SET_SCAN_UNASSOC_TIME, &timems, sizeof(timems)) ) < 0 )
    {
		printk(KERN_ERR "%s: set failed. value[%d], ret[%d]\n",\
                __FUNCTION__, timems, ret);
    }

	p += snprintf(p, MAX_WX_STRING, ret < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
		
    return ret;
}
*/

/*
static int
wl_iw_set_scan_unassoc_time_30_ms(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
    int timems = 30;
    int ret = 0;
	char *p = extra;

    ASSERT( dev && wrqu );

	if( ( ret = dev_wlc_ioctl(dev, WLC_SET_SCAN_UNASSOC_TIME, &timems, sizeof(timems)) ) < 0 )
    {
		printk(KERN_ERR "%s: set failed. value[%d], ret[%d]\n",\
                __FUNCTION__, timems, ret);
    }

	p += snprintf(p, MAX_WX_STRING, ret < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
		
    return ret;
}


static int
wl_iw_set_scan_unassoc_time_80_ms(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
    int timems = 80;
    int ret = 0;
	char *p = extra;

    ASSERT( dev && wrqu );

	if( ( ret = dev_wlc_ioctl(dev, WLC_SET_SCAN_UNASSOC_TIME, &timems, sizeof(timems)) ) < 0 )
    {
		printk(KERN_ERR "%s: set failed. value[%d], ret[%d]\n",\
                __FUNCTION__, timems, ret);
    }

	p += snprintf(p, MAX_WX_STRING, ret < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
		
    return ret;
}
*/


/* BRCM_UPDATE_S for KEEP_ALIVE */
static int wl_keep_alive_set(struct net_device *dev, char* extra)
{
	wl_keep_alive_pkt_t keep_alive_pkt;
	wl_keep_alive_pkt_t *keep_alive_pktp;
	int                     buf_len;
	int                     str_len;
	int ret = 0;
	char buf[100]  = {0,};
    uint period_msec = 0;

    if( extra == NULL ) 
    {
        WL_ERROR(( "%s: extra is NULL\n", __FUNCTION__ ));
        return -1;
    }

	if (sscanf(extra, "%*s %d", &period_msec) != 1)
    {
        WL_ERROR(( "%s: sscanf error. check period_msec value\n", __FUNCTION__ ));
		return -EINVAL;
    }
	
    WL_TRACE(( "%s: period_msec is %d\n", __FUNCTION__, period_msec ));

	memset(&keep_alive_pkt, 0, sizeof(wl_keep_alive_pkt_t));

    strcpy( buf, "keep_alive" );
    str_len = strlen( buf );
    buf[ str_len ] = '\0';

	keep_alive_pktp = (wl_keep_alive_pkt_t *) (buf + str_len + 1);
	keep_alive_pkt.period_msec = htod32(period_msec);		
	keep_alive_pkt.len_bytes = 0;

	buf_len = str_len + 1;
	buf_len += (WL_KEEP_ALIVE_FIXED_LEN + keep_alive_pkt.len_bytes);

	memcpy(keep_alive_pktp, &keep_alive_pkt, WL_KEEP_ALIVE_FIXED_LEN);

	if( (ret= dev_wlc_ioctl(dev, WLC_SET_VAR, buf, buf_len)) < 0)
		WL_ERROR(("%s:keep_alive set failed. ret[%d]\n",__FUNCTION__, ret));
	else
		WL_TRACE(("%s:keep_alive set ok. ret[%d]\n",__FUNCTION__, ret));

	return ret;
}
/* BRCM_UPDATE_E for KEEP_ALIVE */
/* BRCM_UPDATE_S for PM2_SLEEP_RET */
static int
wl_iw_set_pm2_sleep_ret(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	uint msec = 200; // Default value
	int error = 0;
	char *p = extra;
	char buf[100]  = {0};
	int len = 0;


	WL_ERROR(("%s: wl_iw_set_pm2_sleep_ret requst = %s\n",
		dev->name, extra));

	if (sscanf(extra, "%*s %d", &msec) != 1)
	{
		return -EINVAL;
	}
//	WL_ERROR("%swl_iw_set_pm2_sleep_ret : extra %s",__FUNCTION__,extra);

	len = bcm_mkiovar("pm2_sleep_ret", (char *)&msec, sizeof(msec), buf, sizeof(buf));
	ASSERT(len);

	if ((error = dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len)) < 0)
	{
		WL_ERROR(("%s:pm2_sleep_ret set failed. msec[%d]:error[%d]\n",__FUNCTION__, msec, error));
	}
	
	p += snprintf(p, MAX_WX_STRING, error < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
	return error;
}
/* BRCM_UPDATE_E for PM2_SLEEP_RET */
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP) && defined(CONFIG_BRCM_LGE_WL_ARPOFFLOAD)
static int
wl_iw_set_hostip(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	char *p = extra;
	int ip_offset	= 0;
        int received_ip_size = 4;
	char ipaddr[4] = {0};
	char buf[100]  = {0};
	int len = 0;
	int ret = 0;
	long int ipaddr_int = 0;

	
	ip_offset = strcspn(extra, " ");

	received_ip_size = strlen(extra) - ip_offset;

	ret = strict_strtol((extra+1+ip_offset), 0, &ipaddr_int);

//	printk("%s: Received IP Address: " NIPQUAD_FMT "\n" ,__FUNCTION__, NIPQUAD(ipaddr_int));

	ipaddr[0] = ((unsigned char *)&ipaddr_int)[0];
	ipaddr[1] = ((unsigned char *)&ipaddr_int)[1];
	ipaddr[2] = ((unsigned char *)&ipaddr_int)[2];
	ipaddr[3] = ((unsigned char *)&ipaddr_int)[3];

	/*Clear the present Hostip [if any]*/
	len = bcm_mkiovar("arp_hostip_clear", NULL, 0, buf, sizeof(buf));
	ASSERT(len);

	if( (ret= dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len)) < 0)
		printk(KERN_ERR "%s:arp_hostip_clear failed.ret[%d]\n",__FUNCTION__, ret);


	/* Set the new hostip*/
	len = bcm_mkiovar("arp_hostip", (char *)ipaddr, sizeof(ipaddr), buf, sizeof(buf));
	ASSERT(len);

	if( (ret= dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len)) < 0)
		printk(KERN_ERR "%s:arp_hostip set failed. ret[%d]\n",__FUNCTION__, ret);
		
#if 0  // no more use.
/* BRCM_UPDATE_S for KEEP_ALIVE */
	wl_keep_alive_set(dev);
/* BRCM_UPDATE_E for KEEP_ALIVE */
#endif

       p += snprintf(p, MAX_WX_STRING, "OK");
       wrqu->data.length = p - extra + 1;

    return 0;
}
#endif	/* defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP) && defined(CONFIG_BRCM_LGE_WL_ARPOFFLOAD) */

static int
wl_iw_set_country(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	char country_code[WLC_CNTRY_BUF_SZ];
	int error = 0;
	char *p = extra;
	int country_offset;
	int country_code_size;

	memset(country_code, 0, sizeof(country_code));

	
	country_offset = strcspn(extra, " ");
	country_code_size = strlen(extra) - country_offset;

	
	if (country_offset != 0) {
		strncpy(country_code, extra + country_offset +1,
			MIN(country_code_size, sizeof(country_code)));

		
		if ((error = dev_wlc_ioctl(dev, WLC_SET_COUNTRY,
			&country_code, sizeof(country_code))) >= 0) {
			p += snprintf(p, MAX_WX_STRING, "OK");
			WL_TRACE(("%s: set country %s OK\n", __FUNCTION__, country_code));
			goto exit;
		}
	}

	WL_ERROR(("%s: set country %s failed code %d\n", __FUNCTION__, country_code, error));
	p += snprintf(p, MAX_WX_STRING, "FAIL");

exit:
	wrqu->data.length = p - extra + 1;
	return error;
#else
	return 0;
#endif
}

#if !defined(CONFIG_LGE_BCM432X_PATCH)
static int
wl_iw_set_btcoex_dhcp(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error = 0;
	char *p = extra;
	uint val;
#ifdef  CUSTOMER_HW2
	static int  pm = PM_FAST;
	int  pm_local = PM_OFF;
#endif
	char powermode_val = 0;
	char buf_reg66va_dhcp_on[8] = { 66, 00, 00, 00, 0x10, 0x27, 0x00, 0x00 };
	char buf_reg41va_dhcp_on[8] = { 41, 00, 00, 00, 0x33, 0x00, 0x00, 0x00 };
	char buf_reg68va_dhcp_on[8] = { 68, 00, 00, 00, 0x90, 0x01, 0x00, 0x00 };

	char buf_reg66val_defualt[8] = { 66, 00, 00, 00, 0x88, 0x13, 0x00, 0x00 };
	char buf_reg41val_defualt[8] = { 41, 00, 00, 00, 0x13, 0x00, 0x00, 0x00 };
	char buf_reg68val_defualt[8] = { 68, 00, 00, 00, 0x14, 0x00, 0x00, 0x00 };

#ifdef COEX_DHCP
	char buf_flag7_default[8] =   { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};
#endif 

	
	strncpy((char *)&powermode_val, extra + strlen("POWERMODE") +1, 1);

	
	dev_wlc_intvar_get_reg(dev, "btc_params", 68,  &val);

	if (strnicmp((char *)&powermode_val, "1", strlen("1")) == 0) {

		WL_TRACE(("%s: DHCP session starts\n", __FUNCTION__));

#ifdef  CUSTOMER_HW2

		dev_wlc_ioctl(dev, WLC_GET_PM, &pm, sizeof(pm));

		dev_wlc_ioctl(dev, WLC_SET_PM, &pm_local, sizeof(pm_local));
#endif

		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg66va_dhcp_on[0], sizeof(buf_reg66va_dhcp_on));
		
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg41va_dhcp_on[0], sizeof(buf_reg41va_dhcp_on));
		
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg68va_dhcp_on[0], sizeof(buf_reg68va_dhcp_on));

#ifdef COEX_DHCP
		
		g_bt->bt_state = BT_DHCP_START;
		g_bt->timer_on = 1;
		mod_timer(&g_bt->timer, g_bt->timer.expires);
		WL_TRACE(("%s enable BT DHCP Timer\n", __FUNCTION__));
#endif 

	}
	else if (strnicmp((char *)&powermode_val, "0", strlen("0")) == 0) {

		WL_TRACE(("%s: DHCP session done\n", __FUNCTION__));

#ifdef  CUSTOMER_HW2

		dev_wlc_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm));
#endif
#ifdef COEX_DHCP
		
		WL_TRACE(("%s disable BT DHCP Timer\n", __FUNCTION__));
		if (g_bt->timer_on) {
			g_bt->timer_on = 0;
			del_timer_sync(&g_bt->timer);
		}

		
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_default[0], sizeof(buf_flag7_default));
#endif 

		
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg66val_defualt[0], sizeof(buf_reg66val_defualt));
		
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg41val_defualt[0], sizeof(buf_reg41val_defualt));
		
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg68val_defualt[0], sizeof(buf_reg68val_defualt));
	}
	else {
		WL_ERROR(("Unkwown yet power setting, ignored\n"));
	}

	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;

	return error;
}
#endif	/* !defined(CONFIG_LGE_BCM432X_PATCH) */

int
wl_format_ssid(char* ssid_buf, uint8* ssid, int ssid_len)
{
	int i, c;
	char *p = ssid_buf;

	if (ssid_len > 32) ssid_len = 32;

	for (i = 0; i < ssid_len; i++) {
		c = (int)ssid[i];
		if (c == '\\') {
			*p++ = '\\';
			*p++ = '\\';
		} else if (isprint((uchar)c)) {
			*p++ = (char)c;
		} else {
			p += sprintf(p, "\\x%02X", c);
		}
	}
	*p = '\0';

	return p - ssid_buf;
}

static int
wl_iw_get_link_speed(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error = 0;
	char *p = extra;
	static int link_speed;

	
	if (g_onoff == G_WLAN_SET_ON) {
		error = dev_wlc_ioctl(dev, WLC_GET_RATE, &link_speed, sizeof(link_speed));
		link_speed *= 500000;
	}

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-04-07, showing the 5.5 Mbps */
#if defined(CONFIG_LGE_BCM432X_PATCH)
	if (link_speed == 5500000)
		p += snprintf(p, MAX_WX_STRING, "LinkSpeed 5.5");
	else
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-04-07, showing the 5.5 Mbps */
	p += snprintf(p, MAX_WX_STRING, "LinkSpeed %d", link_speed/1000000);

	wrqu->data.length = p - extra + 1;

	return error;
}

static int
wl_iw_get_rssi(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	static int rssi = 0;
	int error = 0;
	wlc_ssid_t ssid;
	char *p = extra;
/* LGE_CHANGE_S, [yoohoo@lge.com], 2009-05-13,
 * <some ssid use '<' character sometimes and it cause response discard
 * in wpa_supplicant (wpa_ctrl_request())> */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	static char ssidbuf[SSID_FMT_BUF_LEN];
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E, [yoohoo@lge.com], 2009-05-13,
 * <some ssid use '<' character sometimes and it cause response discard
 * in wpa_supplicant (wpa_ctrl_request())> */
	scb_val_t scb_val;

	scb_val.val = 0;

	bzero(&ssid, sizeof(ssid));
	if (g_onoff == G_WLAN_SET_ON) {
//		error = dev_wlc_ioctl(dev, WLC_GET_RSSI, &scb_val, sizeof(scb_val_t));
//		rssi = dtoh32(scb_val.val);
		error = dev_wlc_ioctl(dev, WLC_GET_RSSI, &rssi, sizeof(rssi));
		rssi = dtoh32(rssi);

		error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid));

		ssid.SSID_len = dtoh32(ssid.SSID_len);
	}

	if (!ssid.SSID_len) {
		return 0;
	}

/* LGE_CHANGE_S, [yoohoo@lge.com], 2009-05-13,
 * <some ssid use '<' character sometimes and it cause response discard
 * in wpa_supplicant (wpa_ctrl_request())> */
#if defined(CONFIG_LGE_BCM432X_PATCH)
	p += snprintf(p, MAX_WX_STRING, "ssid rssi %d", rssi);
#else /* CONFIG_LGE_BCM432X_PATCH */
	wl_format_ssid(ssidbuf, ssid.SSID, dtoh32(ssid.SSID_len));
	p += snprintf(p, MAX_WX_STRING, "%s rssi %d ", ssidbuf, rssi);
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E, [yoohoo@lge.com], 2009-05-13,
 * <some ssid use '<' character sometimes and it cause response discard
 * in wpa_supplicant (wpa_ctrl_request())> */

	wrqu->data.length = p - extra + 1;

	return error;
}

int
wl_iw_send_priv_event(
	struct net_device *dev,
	char *flag
)
{
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];
	int cmd;

	cmd = IWEVCUSTOM;
	memset(&wrqu, 0, sizeof(wrqu));
	if (strlen(flag) > sizeof(extra))
		return -1;

	strcpy(extra, flag);
	wrqu.data.length = strlen(extra);
	wireless_send_event(dev, cmd, &wrqu, extra);
	WL_TRACE(("Send IWEVCUSTOM Event as %s\n", extra));

	return 0;
}

/* LGE_CHANGE_S, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BRCM_USE_DEEPSLEEP)
extern int dhd_deep_sleep(struct net_device *dev, int flag);
#endif /* CONFIG_LGE_BCM432X_PATCH && CONFIG_BRCM_USE_DEEPSLEEP */
/* LGE_CHANGE_E, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */
/* Do not use GPIO Reset at On/Off. Use mpc. */
#if defined(CONFIG_BRCM_USE_GPIO_RESET) || defined(CONFIG_BRCM_USE_DEEPSLEEP)
static int
_wl_control_sysioc_thread_wl_off(void *data)
{
	struct wl_ctrl *wl_ctl = (struct wl_ctrl *)data;

#if defined(BCMDONGLEHOST)
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(wl_ctl->dev);
#endif 
	DAEMONIZE("wlcontrol_sysioc");

	WL_TRACE(("%s Entered\n", __FUNCTION__));

#if defined(BCMDONGLEHOST)
	//WAKE_LOCK_INIT(iw->pub, WAKE_LOCK_OFF, "sysioc_thread_wl_off");
	//WAKE_LOCK(iw->pub, WAKE_LOCK_OFF);
	DHD_OS_WAKE_LOCK(iw->pub);
#endif 
	while (down_interruptible(&wl_ctl->timer_sem) == 0) {

		WL_TRACE(("%s Turning off wifi dev\n", __FUNCTION__));

		g_onoff = G_WLAN_SET_OFF;

#if defined(WL_IW_USE_ISCAN)
		g_iscan->iscan_state = ISCAN_STATE_IDLE;
#elif defined(WL_IW_USE_ESCAN)
		g_escan->escan_state = ESCAN_STATE_IDLE;
#endif

/* LGE_CHANGE_S, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */
#if defined(CONFIG_BRCM_USE_DEEPSLEEP)
		/* Use Deep Sleep instead of WL Reset*/
		dhd_deep_sleep(wl_ctl->dev, TRUE);
#elif defined(CONFIG_BRCM_USE_GPIO_RESET)
		dhd_dev_reset(wl_ctl->dev, 1);

#if defined(BCMLXSDMMC)
		sdioh_stop(NULL);
#endif

		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);
#endif
/* LGE_CHANGE_E, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */

		wl_iw_send_priv_event(wl_ctl->dev, "STOP");

		break;
	}

	WL_TRACE(("%s Exited\n", __FUNCTION__));
#if defined(BCMDONGLEHOST)
	DHD_OS_WAKE_UNLOCK(iw->pub);
	//WAKE_LOCK_DESTROY(iw->pub, WAKE_LOCK_OFF);
#endif 

	complete_and_exit(&wl_ctl->sysioc_exited, 0);
	KILL_PROC(wl_ctl->sysioc_pid, SIGTERM);
}

static void
wl_iw_stop_timerfunc(ulong data)
{
	struct wl_ctrl * wl_ctl = (struct wl_ctrl *)data;

	WL_TRACE(("%s\n", __FUNCTION__));

	del_timer(wl_ctl->timer);

	up(&wl_ctl->timer_sem);
}
#endif	/* CONFIG_LGE_BCM432X_PATCH */

static int
wl_iw_control_wl_off(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;
#if 0 // defined(CONFIG_BRCM_USE_GPIO_RESET) || defined(CONFIG_BRCM_USE_DEEPSLEEP)  // louislee : for ping+suspend(lcd off) error
	static struct wl_ctrl ctl;
	static struct timer_list timer;

	WL_TRACE(("Enter %s\n", __FUNCTION__));

	
	
	ctl.timer = &timer;
	ctl.dev = dev;
	sema_init(&ctl.timer_sem, 0);
	init_completion(&ctl.sysioc_exited);

	
	ctl.sysioc_pid = kernel_thread(_wl_control_sysioc_thread_wl_off, &ctl, 0);

	timer.data = (ulong)&ctl;
	timer.function = wl_iw_stop_timerfunc;
	init_timer(&timer);
#if defined(CONFIG_LGE_BCM432X_PATCH)
	timer.expires = jiffies + 100 * HZ / 1000;
#else
	timer.expires = jiffies + 2000 * HZ / 1000;
#endif	/* CONFIG_LGE_BCM432X_PATCH */
	add_timer(&timer);

	WL_TRACE(("Exited %s\n", __FUNCTION__));
#else	/* defined(CONFIG_BRCM_USE_GPIO_RESET) || defined(CONFIG_BRCM_USE_DEEPSLEEP) */

	scb_val_t scbval;

	printk("Enter %s\n", __FUNCTION__);
#ifdef SOFTAP
	ap_cfg_running = FALSE;
#endif /* SOFTAP */

	if(g_onoff == G_WLAN_SET_ON){
		g_onoff = G_WLAN_SET_OFF;

		bzero(&scbval, sizeof(scb_val_t));
		(void) dev_wlc_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t));

#if defined(CONFIG_BRCM_LGE_WL_ARPOFFLOAD)
        do
        {   
            char buf[32] = {0,};
            int len = 0;
            /*Clear the present Hostip [if any]*/
            len = bcm_mkiovar("arp_hostip_clear", NULL, 0, buf, sizeof(buf));
            ASSERT(len);

            dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len);
        } while(0);
#endif //CONFIG_BRCM_LGE_WL_ARPOFFLOAD

		wl_iw_send_priv_event(dev, "STOP");
		
		printk("Exit %s\n", __FUNCTION__);
	}
#endif

	printk("Enter %s -5 \n", __FUNCTION__); 	


	return ret;
}

#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BRCM_USE_GPIO_RESET)
#include <linux/sched.h>
#endif/* defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BRCM_USE_GPIO_RESET) */

static int
wl_iw_control_wl_on(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;

	WL_TRACE(("Enter %s \n", __FUNCTION__));

	if (g_onoff == G_WLAN_SET_OFF) {
/* LGE_CHANGE_S, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */
#if defined(CONFIG_BRCM_USE_DEEPSLEEP)
		/* Use Deep Sleep instead of WL RESET */
		dhd_deep_sleep(dev, FALSE);
#elif defined(CONFIG_BRCM_USE_GPIO_RESET)
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);

#if defined(BCMLXSDMMC)
		 sdioh_start(NULL, 0);
#endif

		dhd_dev_reset(dev, 0);

#if defined(BCMLXSDMMC)
		 sdioh_start(NULL, 1);
#endif

#if 0 // defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BRCM_USE_GPIO_RESET) // louislee : for ping+suspend(lcd off) error
		 schedule_timeout((100*HZ)/1000);
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BRCM_USE_GPIO_RESET) */
		 dhd_dev_init_ioctl(dev);
#else	/* #elif defined(CONFIG_BRCM_USE_GPIO_RESET) */
		g_onoff = G_WLAN_SET_ON;
		wl_iw_send_priv_event(dev, "START");
		printk("Exited %s \n", __FUNCTION__);
#endif	/* CONFIG_BRCM_USE_DEEPSLEEP *//* Do not use GPIO Reset at On/Off. Use mpc. */
/* LGE_CHANGE_E, [yoohoo@lge.com], 2009-11-19, Use deepsleep instead of dhd_dev_reset when driver start or stop */

		g_onoff = G_WLAN_SET_ON;
	}

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-07-09, send wl_iw_send_priv_event only if receiving regular START command */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	/* Modify not sending wl_iw_send_priv_event in wl_iw_control_wl_on() */
	wl_iw_send_priv_event(dev, "START");
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-07-09, send wl_iw_send_priv_event only if receiving regular START command */

	WL_TRACE(("Exited %s \n", __FUNCTION__));

	return ret;
}

#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP)
int
wl_control_wl_start(struct net_device *dev)
{
	int ret = 0;
	wl_iw_t *iw;

	WL_ERROR(("Enter %s \n", __FUNCTION__));

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -1;
	}

	iw = *(wl_iw_t **)netdev_priv(dev);
//	MUTEX_LOCK(iw->pub);

	if (g_onoff == G_WLAN_SET_OFF) {
//bill.jung@lge.com - Don't restart. 
#if 0
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);
#if defined(BCMLXSDMMC)
		sdioh_start(NULL, 0);
#endif

		dhd_dev_reset(dev, 0);

#if defined(BCMLXSDMMC)
		sdioh_start(NULL, 1);
#endif

		dhd_dev_init_ioctl(dev);
#endif
		g_onoff = G_WLAN_SET_ON;
	}
	WL_ERROR(("Exited %s \n", __FUNCTION__));

//	MUTEX_UNLOCK(iw->pub);
	return ret;
}
static int
wl_iw_control_wl_off_softap(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;
	wl_iw_t *iw;

	WL_ERROR(("Enter %s\n", __FUNCTION__));

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -1;
	}

	iw = *(wl_iw_t **)netdev_priv(dev);
//	MUTEX_LOCK(iw->pub);

//#ifdef SOFTAP
	ap_cfg_running = FALSE;
//#endif /* SOFTAP */

	if (g_onoff == G_WLAN_SET_ON) {
		g_onoff = G_WLAN_SET_OFF;

#if defined(WL_IW_USE_ISCAN)
		g_iscan->iscan_state = ISCAN_STATE_IDLE;
#endif 

		dhd_dev_reset(dev, 1);

#if defined(WL_IW_USE_ISCAN)
		wl_iw_free_ss_cache();
		wl_iw_run_ss_cache_timer(0);
		memset(g_scan, 0, G_SCAN_RESULTS);
		
		g_ss_cache_ctrl.m_link_down = 1;
		g_scan_specified_ssid = 0;
		
		g_first_broadcast_scan = BROADCAST_SCAN_FIRST_IDLE;
#endif

#if defined(BCMLXSDMMC)
		sdioh_stop(NULL);
#endif
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);

		wl_iw_send_priv_event(dev, "STOP");
	}

//	MUTEX_UNLOCK(iw->pub);

	WL_TRACE(("Exited %s\n", __FUNCTION__));

	return ret;
}

static int
wl_iw_control_wl_on_softap(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;

	WL_TRACE(("Enter %s \n", __FUNCTION__));

	ret = wl_control_wl_start(dev);

	wl_iw_send_priv_event(dev, "START");

	WL_TRACE(("Exited %s \n", __FUNCTION__));

	return ret;
}
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP) */

#ifdef SOFTAP
static struct ap_profile my_ap;
static int set_ap_cfg(struct net_device *dev, struct ap_profile *ap); 
static int get_assoc_sta_list(struct net_device *dev, char *buf, int len);
static int set_ap_mac_list(struct net_device *dev, char *buf);

#define PTYPE_STRING 0
#define PTYPE_INTDEC 1   
#define PTYPE_INTHEX 2
#define PTYPE_STR_HEX 3  
int get_parmeter_from_string(
	char **str_ptr, const char *token, int param_type, void  *dst, int param_max_len);

int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

int hex2byte(const char *hex)
{
	int a, b;
	a = hex2num(*hex++);
	if (a < 0)
		return -1;
	b = hex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}

int hstr_2_buf(const char *txt, u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		int a, b;

		a = hex2num(*txt++);
		if (a < 0)
			return -1;
		b = hex2num(*txt++);
		if (b < 0)
			return -1;
		*buf++ = (a << 4) | b;
	}

	return 0;
}

int init_ap_profile_from_string(char *param_str, struct ap_profile *ap_cfg)
{
	char *str_ptr = param_str;
	char sub_cmd[16];
	int ret = 0;

	memset(sub_cmd, 0, sizeof(sub_cmd));
	memset(ap_cfg, 0, sizeof(struct ap_profile));

	if (get_parmeter_from_string(&str_ptr, "ASCII_CMD=",
		PTYPE_STRING, sub_cmd, SSID_LEN) != 0) {
	 return -1;
	}
	if (strncmp(sub_cmd, "AP_CFG", 6)) {
	   WL_ERROR(("ERROR: sub_cmd:%s != 'AP_CFG'!\n", sub_cmd));
		return -1;
	}

	ret = get_parmeter_from_string(&str_ptr, "SSID=", PTYPE_STRING, ap_cfg->ssid, SSID_LEN);

	ret |= get_parmeter_from_string(&str_ptr, "SEC=", PTYPE_STRING,  ap_cfg->sec, SEC_LEN);

	ret |= get_parmeter_from_string(&str_ptr, "KEY=", PTYPE_STRING,  ap_cfg->key, KEY_LEN);

	ret |= get_parmeter_from_string(&str_ptr, "CHANNEL=", PTYPE_INTDEC, &ap_cfg->channel, 5);

	ret |= get_parmeter_from_string(&str_ptr, "PREAMBLE=", PTYPE_INTDEC, &ap_cfg->preamble, 5);

	ret |= get_parmeter_from_string(&str_ptr, "MAX_SCB=", PTYPE_INTDEC,  &ap_cfg->max_scb, 5);

	return ret;
}

static int iwpriv_set_ap_config(struct net_device *dev,
            struct iw_request_info *info,
            union iwreq_data *wrqu,
            char *ext)
{
	int res = 0;
	char  *extra = NULL;
	struct ap_profile *ap_cfg = &my_ap;

	WL_TRACE(("> Got IWPRIV SET_AP IOCTL: info->cmd:%x, info->flags:%x, u.data:%p, u.len:%d\n",
		info->cmd, info->flags,
		wrqu->data.pointer, wrqu->data.length));


	if (wrqu->data.length != 0) {

		char *str_ptr;

		if (!(extra = kmalloc(wrqu->data.length+1, GFP_KERNEL)))
			return -ENOMEM;

		if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) {
			kfree(extra);
			return -EFAULT;
		}

		extra[wrqu->data.length] = 0;
		WL_SOFTAP((" Got str param in iw_point:\n %s\n", extra));

		memset(ap_cfg, 0, sizeof(struct ap_profile));

		

		str_ptr = extra;

		if ((res = init_ap_profile_from_string(extra, ap_cfg)) < 0) {
			WL_ERROR(("%s failed to parse %d\n", __FUNCTION__, res));
			kfree(extra);
			return -1;
		}

	} else {
	 
	  WL_ERROR(("IWPRIV argument len = 0 \n"));
	  return -1;
	}

	if ((res = set_ap_cfg(dev, ap_cfg)) < 0)
		WL_ERROR(("%s failed to set_ap_cfg %d\n", __FUNCTION__, res));

	kfree(extra);

	return res;
}

static int iwpriv_get_assoc_list(struct net_device *dev,
        struct iw_request_info *info,
        union iwreq_data *p_iwrq,
        char *extra)
{
	int i, ret = 0;
	char mac_buf[256];
	struct maclist *sta_maclist = (struct maclist *)mac_buf;

	char mac_lst[256];
	char *p_mac_str;

	WL_TRACE(("\n %s: IWPRIV IOCTL: cmd:%hx, flags:%hx, extra:%p, iwp.len:%d, \
		iwp.len:%p, iwp.flags:%x  \n", __FUNCTION__, info->cmd, info->flags, \
		extra, p_iwrq->data.length, p_iwrq->data.pointer, p_iwrq->data.flags));

	WL_SOFTAP(("extra:%s\n", extra));
	print_buf((u8 *)p_iwrq, 16, 0);

	memset(sta_maclist, 0, sizeof(mac_buf));

	sta_maclist->count = 8;

	WL_TRACE((" net device:%s, buf_sz:%d\n", dev->name, sizeof(mac_buf)));
	get_assoc_sta_list(dev, mac_buf, 256);
	WL_TRACE((" got %d stations\n", sta_maclist->count));

	memset(mac_lst, 0, sizeof(mac_lst));
	p_mac_str = mac_lst;

	for (i = 0; i < 8; i++) {
		struct ether_addr * id = &sta_maclist->ea[i];

		WL_SOFTAP(("dhd_drv>> sta_mac[%d] :", i));
		print_buf((unsigned char *)&sta_maclist->ea[i], 6, 0);

		p_mac_str += snprintf(p_mac_str, MAX_WX_STRING,
			"Mac[%d]=%02X:%02X:%02X:%02X:%02X:%02X\n", i,
			id->octet[0], id->octet[1], id->octet[2],
			id->octet[3], id->octet[4], id->octet[5]);
	}

	p_iwrq->data.length = strlen(mac_lst);

	WL_TRACE(("u.pointer:%p\n", p_iwrq->data.pointer));
	WL_TRACE(("resulting str:\n%s \n len:%d\n\n", mac_lst, p_iwrq->data.length));

	if (p_iwrq->data.length) {
		if (copy_to_user(p_iwrq->data.pointer, mac_lst, p_iwrq->data.length)) {
			WL_ERROR(("%s: Can't copy to user\n", __FUNCTION__));
			return -EFAULT;
		}
	}

	WL_ERROR(("Exited %s \n", __FUNCTION__));
	return ret;
}

static int iwpriv_set_mac_filters(struct net_device *dev,
        struct iw_request_info *info,
        union iwreq_data *wrqu,
        char *ext)
{
	int i, ret = -1;
	char  * extra = NULL;
	u8  macfilt[8][6];
	int mac_cnt = 0; 
	char sub_cmd[16];

	WL_TRACE((">>> Got IWPRIV SET_MAC_FILTER IOCTL:  info->cmd:%x, \
			info->flags:%x, u.data:%p, u.len:%d\n",
			info->cmd, info->flags,
			wrqu->data.pointer, wrqu->data.length));

	if (wrqu->data.length != 0) {

		char *str_ptr;

		if (!(extra = kmalloc(wrqu->data.length+1, GFP_KERNEL)))
			return -ENOMEM;

		if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) {
			kfree(extra);
			return -EFAULT;
		}

		extra[wrqu->data.length] = 0;
		WL_SOFTAP((" Got parameter string in iw_point:\n %s \n", extra));

		memset(macfilt, 0, sizeof(macfilt));
		memset(sub_cmd, 0, sizeof(sub_cmd));

		str_ptr = extra;

		if (get_parmeter_from_string(&str_ptr, "ASCII_CMD=", PTYPE_STRING, sub_cmd, 15) != 0) {
			goto exit_proc;
		}

#define MAC_FILT_MAX 8
	   
		if (strncmp(sub_cmd, "MAC_FLT_W", strlen("MAC_FLT_W"))) {
		   WL_ERROR(("ERROR: sub_cmd:%s != 'MAC_FLT_W'!\n", sub_cmd));
			goto exit_proc;
		}

		if (get_parmeter_from_string(&str_ptr, "MAC_CNT=",
			PTYPE_INTDEC, &mac_cnt, 4) != 0) {
			WL_ERROR(("ERROR: MAC_CNT param is missing \n"));
			goto exit_proc;
		}

		if (mac_cnt > MAC_FILT_MAX) {
			WL_ERROR(("ERROR: number of MAC filters > MAX\n"));
			goto exit_proc;
		}

		for (i=0; i< mac_cnt; i++)	
		if (get_parmeter_from_string(&str_ptr, "MAC=",
			PTYPE_STR_HEX, macfilt[i], 12) != 0) {
			WL_ERROR(("ERROR: MAC_filter[%d] is missing !\n", i));
			goto exit_proc;
		}

		for (i = 0; i < mac_cnt; i++) {
		   WL_SOFTAP(("mac_filt[%d]:", i));
		   print_buf(macfilt[i], 6, 0);
		}

		wrqu->data.pointer = NULL;
		wrqu->data.length = 0;
		ret = 0;
	} else {
	  WL_ERROR(("IWPRIV argument len is 0\n"));
	  return -1;
	}

	exit_proc:
	kfree(extra);
	return ret;
}
#endif /* SOFTAP */

#endif 

#if WIRELESS_EXT < 13
struct iw_request_info
{
	__u16		cmd;		
	__u16		flags;		
};

typedef int (*iw_handler)(struct net_device *dev, struct iw_request_info *info,
	void *wrqu, char *extra);
#endif 

static int
wl_iw_config_commit(
	struct net_device *dev,
	struct iw_request_info *info,
	void *zwrq,
	char *extra
)
{
	wlc_ssid_t ssid;
	int error;
	struct sockaddr bssid;

	WL_TRACE(("%s: SIOCSIWCOMMIT\n", dev->name));

// WBT Fix TD# 37078
	memset(&ssid, 0, sizeof(wlc_ssid_t));
//

	if ((error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid))))
		return error;

	ssid.SSID_len = dtoh32(ssid.SSID_len);

	if (!ssid.SSID_len)
		return 0;

	bzero(&bssid, sizeof(struct sockaddr));
	if ((error = dev_wlc_ioctl(dev, WLC_REASSOC, &bssid, ETHER_ADDR_LEN))) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	return 0;
}

static int
wl_iw_get_name(
	struct net_device *dev,
	struct iw_request_info *info,
	char *cwrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWNAME\n", dev->name));

	strcpy(cwrq, "IEEE 802.11-DS");

	return 0;
}

static int
wl_iw_set_freq(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_freq *fwrq,
	char *extra
)
{
	int error, chan;
	uint sf = 0;

	WL_TRACE(("%s: SIOCSIWFREQ\n", dev->name));

#if defined(SOFTAP)
	if (ap_cfg_running) {
		WL_TRACE(("%s:>> not executed, 'SOFT_AP is active' \n", __FUNCTION__));
		return 0;
	}
#endif /* SOFTAP */
	
	if (fwrq->e == 0 && fwrq->m < MAXCHANNEL) {
		chan = fwrq->m;
	}

	
	else {
		
		if (fwrq->e >= 6) {
			fwrq->e -= 6;
			while (fwrq->e--)
				fwrq->m *= 10;
		} else if (fwrq->e < 6) {
			while (fwrq->e++ < 6)
				fwrq->m /= 10;
		}
	
	if (fwrq->m > 4000 && fwrq->m < 5000)
		sf = WF_CHAN_FACTOR_4_G; 

		chan = wf_mhz2channel(fwrq->m, sf);
	}
	chan = htod32(chan);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_CHANNEL, &chan, sizeof(chan))))
		return error;

	
	return -EINPROGRESS;
}

static int
wl_iw_get_freq(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_freq *fwrq,
	char *extra
)
{
	channel_info_t ci;
	int error;

// WBT Fix TD# 37079
	memset(&ci, 0, sizeof(channel_info_t));
//

	WL_TRACE(("%s: SIOCGIWFREQ\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(ci))))
		return error;

	
	fwrq->m = dtoh32(ci.hw_channel);
	fwrq->e = dtoh32(0);
	return 0;
}

static int
wl_iw_set_mode(
	struct net_device *dev,
	struct iw_request_info *info,
	__u32 *uwrq,
	char *extra
)
{
	int infra = 0, ap = 0, error = 0;

	WL_TRACE(("%s: SIOCSIWMODE\n", dev->name));

	switch (*uwrq) {
	case IW_MODE_MASTER:
		infra = ap = 1;
		break;
	case IW_MODE_ADHOC:
	case IW_MODE_AUTO:
		break;
	case IW_MODE_INFRA:
		infra = 1;
		break;
	default:
		return -EINVAL;
	}
	infra = htod32(infra);
	ap = htod32(ap);

	if ((error = dev_wlc_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(infra))) ||
	    (error = dev_wlc_ioctl(dev, WLC_SET_AP, &ap, sizeof(ap))))
		return error;

	
	return -EINPROGRESS;
}

static int
wl_iw_get_mode(
	struct net_device *dev,
	struct iw_request_info *info,
	__u32 *uwrq,
	char *extra
)
{
	int error, infra = 0, ap = 0;

	WL_TRACE(("%s: SIOCGIWMODE\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_INFRA, &infra, sizeof(infra))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_AP, &ap, sizeof(ap))))
		return error;

	infra = dtoh32(infra);
	ap = dtoh32(ap);
	*uwrq = infra ? ap ? IW_MODE_MASTER : IW_MODE_INFRA : IW_MODE_ADHOC;

	return 0;
}

static int
wl_iw_get_range(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	struct iw_range *range = (struct iw_range *) extra;
	int channels[MAXCHANNEL+1];
	wl_uint32_list_t *list = (wl_uint32_list_t *) channels;
	wl_rateset_t rateset;
	int error, i, k;
	uint sf, ch;

// WBT Fix TD# 37082
	int phytype = 0;
//
	uint8 bw_cap = 0, sgi_tx = 0, nmode = 0;
	channel_info_t ci;
	char iovar_buf[4];
	uint8 nrate_list2copy = 0;
	uint16 nrate_list[4][8] = { {13, 26, 39, 52, 78, 104, 117, 130}, 
		{14, 29, 43, 58, 87, 116, 130, 144}, 
		{27, 54, 81, 108, 162, 216, 243, 270}, 
		{30, 60, 90, 120, 180, 240, 270, 300}}; 

	WL_TRACE(("%s: SIOCGIWRANGE\n", dev->name));

	if (!extra)
		return -EINVAL;

	dwrq->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(range));

// WBT Fix TD# 37080, 37081
	memset(&rateset, 0, sizeof(wl_rateset_t));
//
// WBT Fix TD# 37083
	memset(&ci, 0, sizeof(channel_info_t));
//
	
	range->min_nwid = range->max_nwid = 0;

	
	list->count = htod32(MAXCHANNEL);
	if ((error = dev_wlc_ioctl(dev, WLC_GET_VALID_CHANNELS, channels, sizeof(channels))))
		return error;
	for (i = 0; i < dtoh32(list->count) && i < IW_MAX_FREQUENCIES; i++) {
		range->freq[i].i = dtoh32(list->element[i]);

		ch = dtoh32(list->element[i]);
		if (ch <= CH_MAX_2G_CHANNEL)
			sf = WF_CHAN_FACTOR_2_4_G;
		else
			sf = WF_CHAN_FACTOR_5_G;

		range->freq[i].m = wf_channel2mhz(ch, sf);
		range->freq[i].e = 6;
	}
	range->num_frequency = range->num_channels = i;

	
	range->max_qual.qual = 5;
	
	range->max_qual.level = 0x100 - 200;	
	
	range->max_qual.noise = 0x100 - 200;	
	
	range->sensitivity = 65535;

#if WIRELESS_EXT > 11
	
	range->avg_qual.qual = 3;
	
	range->avg_qual.level = 0x100 + WL_IW_RSSI_GOOD;
	
	range->avg_qual.noise = 0x100 - 75;	
#endif 

	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_CURR_RATESET, &rateset, sizeof(rateset))))
		return error;
	rateset.count = dtoh32(rateset.count);
	range->num_bitrates = rateset.count;
	for (i = 0; i < rateset.count && i < IW_MAX_BITRATES; i++)
		range->bitrate[i] = (rateset.rates[i]& 0x7f) * 500000; 
	dev_wlc_bufvar_get(dev, "nmode", iovar_buf, sizeof(iovar_buf));
	nmode = iovar_buf[0];

	if (nmode == 1) {
		dev_wlc_bufvar_get(dev, "mimo_bw_cap", iovar_buf, sizeof(iovar_buf));
		bw_cap = iovar_buf[0];
		dev_wlc_bufvar_get(dev, "sgi_tx", iovar_buf, sizeof(iovar_buf));
		sgi_tx = iovar_buf[0];
		dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(channel_info_t));
		ci.hw_channel = dtoh32(ci.hw_channel);
		dev_wlc_ioctl(dev, WLC_GET_PHYTYPE, &phytype, sizeof(phytype));
		if (phytype == WLC_PHY_TYPE_SSN && (bw_cap == 0 ||
			(bw_cap == 2 && ci.hw_channel <= 14))) {
			if (sgi_tx == 0)
				nrate_list2copy = 0;
			else
				nrate_list2copy = 1;
		}
		if (phytype == WLC_PHY_TYPE_SSN && (bw_cap == 1 ||
			(bw_cap == 2 && ci.hw_channel > 14))) {
			if (sgi_tx == 0)
				nrate_list2copy = 2;
			else
				nrate_list2copy = 3;
		}
		range->num_bitrates += 8;
		for (k = 0; i < range->num_bitrates; k++, i++) {
			
			range->bitrate[i] = (nrate_list[nrate_list2copy][k]) * 500000;
		}
	}

	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_PHYTYPE, &i, sizeof(i))))
		return error;
	i = dtoh32(i);
	if (i == WLC_PHY_TYPE_A)
		range->throughput = 24000000;	
	else
		range->throughput = 1500000;	

	
	range->min_rts = 0;
	range->max_rts = 2347;
	range->min_frag = 256;
	range->max_frag = 2346;

	range->max_encoding_tokens = DOT11_MAX_DEFAULT_KEYS;
	range->num_encoding_sizes = 4;
	range->encoding_size[0] = WEP1_KEY_SIZE;
	range->encoding_size[1] = WEP128_KEY_SIZE;
#if WIRELESS_EXT > 17
	range->encoding_size[2] = TKIP_KEY_SIZE;
#else
	range->encoding_size[2] = 0;
#endif
	range->encoding_size[3] = AES_KEY_SIZE;

	
	range->min_pmp = 0;
	range->max_pmp = 0;
	range->min_pmt = 0;
	range->max_pmt = 0;
	range->pmp_flags = 0;
	range->pm_capa = 0;

	
	range->num_txpower = 2;
	range->txpower[0] = 1;
	range->txpower[1] = 255;
	range->txpower_capa = IW_TXPOW_MWATT;

#if WIRELESS_EXT > 10
	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 19;

	
	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->r_time_flags = 0;
	
	range->min_retry = 1;
	range->max_retry = 255;
	
	range->min_r_time = 0;
	range->max_r_time = 0;
#endif 

#if WIRELESS_EXT > 17
	range->enc_capa = IW_ENC_CAPA_WPA;
	range->enc_capa |= IW_ENC_CAPA_CIPHER_TKIP;
	range->enc_capa |= IW_ENC_CAPA_CIPHER_CCMP;
#ifdef BCMWPA2
	range->enc_capa |= IW_ENC_CAPA_WPA2;
#endif
	
	IW_EVENT_CAPA_SET_KERNEL(range->event_capa);
	
	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWAP);
	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWSCAN);
	IW_EVENT_CAPA_SET(range->event_capa, IWEVTXDROP);
	IW_EVENT_CAPA_SET(range->event_capa, IWEVMICHAELMICFAILURE);
#ifdef BCMWPA2
	IW_EVENT_CAPA_SET(range->event_capa, IWEVPMKIDCAND);
#endif
#endif 

	return 0;
}

static int
rssi_to_qual(int rssi)
{
	if (rssi <= WL_IW_RSSI_NO_SIGNAL)
		return 0;
	else if (rssi <= WL_IW_RSSI_VERY_LOW)
		return 1;
	else if (rssi <= WL_IW_RSSI_LOW)
		return 2;
	else if (rssi <= WL_IW_RSSI_GOOD)
		return 3;
	else if (rssi <= WL_IW_RSSI_VERY_GOOD)
		return 4;
	else
		return 5;
}

static int
wl_iw_set_spy(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);
	struct sockaddr *addr = (struct sockaddr *) extra;
	int i;

	WL_TRACE(("%s: SIOCSIWSPY\n", dev->name));

	if (!extra)
		return -EINVAL;

	iw->spy_num = MIN(ARRAYSIZE(iw->spy_addr), dwrq->length);
	for (i = 0; i < iw->spy_num; i++)
		memcpy(&iw->spy_addr[i], addr[i].sa_data, ETHER_ADDR_LEN);
	memset(iw->spy_qual, 0, sizeof(iw->spy_qual));

	return 0;
}

static int
wl_iw_get_spy(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);
	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality *qual = (struct iw_quality *) &addr[iw->spy_num];
	int i;

	WL_TRACE(("%s: SIOCGIWSPY\n", dev->name));

	if (!extra)
		return -EINVAL;

	dwrq->length = iw->spy_num;
	for (i = 0; i < iw->spy_num; i++) {
		memcpy(addr[i].sa_data, &iw->spy_addr[i], ETHER_ADDR_LEN);
		addr[i].sa_family = AF_UNIX;
		memcpy(&qual[i], &iw->spy_qual[i], sizeof(struct iw_quality));
		iw->spy_qual[i].updated = 0;
	}

	return 0;
}

static int
wl_iw_set_wap(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	int error = -EINVAL;
	wl_join_params_t join_params;

	WL_TRACE(("%s: SIOCSIWAP\n", dev->name));

	if (awrq->sa_family != ARPHRD_ETHER) {
		WL_ERROR(("Invalid Header...sa_family\n"));
		return -EINVAL;
	}

	
	if (ETHER_ISBCAST(awrq->sa_data) || ETHER_ISNULLADDR(awrq->sa_data)) {
		scb_val_t scbval;
		
		bzero(&scbval, sizeof(scb_val_t));
		dev_wlc_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t));
		return 0;
	}


	
	memset(&join_params, 0, sizeof(join_params));

	memcpy(join_params.ssid.SSID, g_ssid.SSID, g_ssid.SSID_len);
	join_params.ssid.SSID_len = htod32(g_ssid.SSID_len);
	memcpy(&join_params.params.bssid, awrq->sa_data, ETHER_ADDR_LEN);

	if ((error = dev_wlc_ioctl(dev, WLC_SET_SSID, &join_params, sizeof(join_params)))) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	
	memset(&g_ssid, 0, sizeof(g_ssid));
	return 0;
}

static int
wl_iw_get_wap(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWAP\n", dev->name));

	awrq->sa_family = ARPHRD_ETHER;
	memset(awrq->sa_data, 0, ETHER_ADDR_LEN);

	
	(void) dev_wlc_ioctl(dev, WLC_GET_BSSID, awrq->sa_data, ETHER_ADDR_LEN);

	return 0;
}

#if WIRELESS_EXT > 17
static int
wl_iw_mlme(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	struct iw_mlme *mlme;
	scb_val_t scbval;
	int error  = -EINVAL;

	WL_TRACE(("%s: SIOCSIWMLME DISASSOC/DEAUTH\n", dev->name));

	mlme = (struct iw_mlme *)extra;
	if (mlme == NULL) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	scbval.val = mlme->reason_code;
	bcopy(&mlme->addr.sa_data, &scbval.ea, ETHER_ADDR_LEN);

	if (mlme->cmd == IW_MLME_DISASSOC) {
		scbval.val = htod32(scbval.val);
		error = dev_wlc_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t));
	}
	else if (mlme->cmd == IW_MLME_DEAUTH) {
		scbval.val = htod32(scbval.val);
		error = dev_wlc_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
			sizeof(scb_val_t));
	}
	else {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	return error;
}
#endif 

static int
wl_iw_get_aplist(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality qual[IW_MAX_AP];
	wl_bss_info_t *bi = NULL;
	int error, i;
	uint buflen = dwrq->length;

	WL_TRACE(("%s: SIOCGIWAPLIST\n", dev->name));

	if (!extra)
		return -EINVAL;

	
	list = kmalloc(buflen, GFP_KERNEL);
	if (!list)
		return -ENOMEM;
	memset(list, 0, buflen);
	list->buflen = htod32(buflen);
	if ((error = dev_wlc_ioctl(dev, WLC_SCAN_RESULTS, list, buflen))) {
		WL_ERROR(("%d: Scan results error %d\n", __LINE__, error));
		kfree(list);
		return error;
	}
	list->buflen = dtoh32(list->buflen);
	list->version = dtoh32(list->version);
	list->count = dtoh32(list->count);
	ASSERT(list->version == WL_BSS_INFO_VERSION);

	for (i = 0, dwrq->length = 0; i < list->count && dwrq->length < IW_MAX_AP; i++) {
		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;
		ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
			buflen));

		
		if (!(dtoh16(bi->capability) & DOT11_CAP_ESS))
			continue;

		
		memcpy(addr[dwrq->length].sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		addr[dwrq->length].sa_family = ARPHRD_ETHER;
		qual[dwrq->length].qual = rssi_to_qual(dtoh16(bi->RSSI));
		qual[dwrq->length].level = 0x100 + dtoh16(bi->RSSI);
		qual[dwrq->length].noise = 0x100 + bi->phy_noise;

		
#if WIRELESS_EXT > 18
		qual[dwrq->length].updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
#else
		qual[dwrq->length].updated = 7;
#endif 

		dwrq->length++;
	}

	kfree(list);

	if (dwrq->length) {
		memcpy(&addr[dwrq->length], qual, sizeof(struct iw_quality) * dwrq->length);
		
		dwrq->flags = 1;
	}

	return 0;
}

#if defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN)
static int
wl_iw_scan_get_aplist(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
#if defined(WL_IW_USE_ISCAN)
	iscan_buf_t * buf;
	iscan_info_t *iscan = g_iscan;
#endif

	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality qual[IW_MAX_AP];
	wl_bss_info_t *bi = NULL;
	int i;

	WL_TRACE(("%s: SIOCGIWAPLIST\n", dev->name));

// WBT Fix TD# 37084
	memset(qual, 0, sizeof(struct iw_quality) * IW_MAX_AP);
//

	if (!extra)
		return -EINVAL;

#if defined(WL_IW_USE_ISCAN)
	if ((!iscan) || (iscan->sysioc_pid < 0)) {
#elif defined(WL_IW_USE_ESCAN)
	if (!g_escan) {
#endif
		return wl_iw_get_aplist(dev, info, dwrq, extra);
	}

#if defined(WL_IW_USE_ISCAN)
	buf = iscan->list_hdr;
	
	while (buf) {
		list = &((wl_iscan_results_t*)buf->iscan_buf)->results;
		ASSERT(list->version == WL_BSS_INFO_VERSION);
#elif defined(WL_IW_USE_ESCAN)
	{
		list = (wl_scan_results_t *) g_escan->escan_buf;
#endif

		bi = NULL;
		for (i = 0, dwrq->length = 0; i < list->count && dwrq->length < IW_MAX_AP; i++) {
			bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length))
			          : list->bss_info;
#if defined(WL_IW_USE_ISCAN)
			ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
				WLC_IW_ISCAN_MAXLEN));
#endif

			
			if (!(dtoh16(bi->capability) & DOT11_CAP_ESS))
				continue;

			
			memcpy(addr[dwrq->length].sa_data, &bi->BSSID, ETHER_ADDR_LEN);
			addr[dwrq->length].sa_family = ARPHRD_ETHER;
			qual[dwrq->length].qual = rssi_to_qual(dtoh16(bi->RSSI));
			qual[dwrq->length].level = 0x100 + dtoh16(bi->RSSI);
			qual[dwrq->length].noise = 0x100 + bi->phy_noise;

			
#if WIRELESS_EXT > 18
			qual[dwrq->length].updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
#else
			qual[dwrq->length].updated = 7;
#endif 

			dwrq->length++;
		}
#if defined(WL_IW_USE_ISCAN)
		buf = buf->next;
#endif
	}
	if (dwrq->length) {
		memcpy(&addr[dwrq->length], qual, sizeof(struct iw_quality) * dwrq->length);
		
		dwrq->flags = 1;
	}

	return 0;
}

static int
wl_iw_scan_prep(wl_scan_params_t *params, wlc_ssid_t *ssid)
{
	int err = 0;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;
	params->nprobes = -1;
	params->active_time = -1;
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;

	params->nprobes = htod32(params->nprobes);
	params->active_time = htod32(params->active_time);
	params->passive_time = htod32(params->passive_time);
	params->home_time = htod32(params->home_time);
	if (ssid && ssid->SSID_len)
		memcpy(&params->ssid, ssid, sizeof(wlc_ssid_t));

	return err;
}
#endif 

#if defined(WL_IW_USE_ISCAN)
static int
wl_iw_iscan(iscan_info_t *iscan, wlc_ssid_t *ssid, uint16 action)
{
	int params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_iscan_params_t, params));
	wl_iscan_params_t *params;
	int err = 0;

	if (ssid && ssid->SSID_len) {
		params_size += sizeof(wlc_ssid_t);
	}
	params = (wl_iscan_params_t*)kmalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		return -ENOMEM;
	}
	memset(params, 0, params_size);
	ASSERT(params_size < WLC_IOCTL_SMLEN);

	err = wl_iw_scan_prep(&params->params, ssid);

	if (!err) {
		params->version = htod32(ISCAN_REQ_VERSION);
		params->action = htod16(action);
		params->scan_duration = htod16(0);

		
		(void) dev_iw_iovar_setbuf(iscan->dev, "iscan", params, params_size,
			iscan->ioctlbuf, WLC_IOCTL_SMLEN);
	}

	kfree(params);
	return err;
}

static void
wl_iw_timerfunc(ulong data)
{
	iscan_info_t *iscan = (iscan_info_t *)data;
	if (iscan) {
		iscan->timer_on = 0;
		if (iscan->iscan_state != ISCAN_STATE_IDLE) {
			WL_TRACE(("timer trigger\n"));
			up(&iscan->sysioc_sem);
		}
	}
}
#endif 

#if defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN)
static void wl_iw_set_event_mask(struct net_device *dev)
{
	char eventmask[WL_EVENTING_MASK_LEN];
	char iovbuf[WL_EVENTING_MASK_LEN + 12];	

	dev_iw_iovar_getbuf(dev, "event_msgs", "", 0, iovbuf, sizeof(iovbuf));
	bcopy(iovbuf, eventmask, WL_EVENTING_MASK_LEN);
#if defined(WL_IW_USE_ISCAN)
	setbit(eventmask, WLC_E_SCAN_COMPLETE);
#elif defined(WL_IW_USE_ESCAN)
	setbit(eventmask, WLC_E_ESCAN_RESULT);
#endif
	dev_iw_iovar_setbuf(dev, "event_msgs", eventmask, WL_EVENTING_MASK_LEN,
		iovbuf, sizeof(iovbuf));
}
#endif 

#if defined(WL_IW_USE_ISCAN)
static uint32
wl_iw_iscan_get(iscan_info_t *iscan)
{
	iscan_buf_t * buf;
	iscan_buf_t * ptr;
	wl_iscan_results_t * list_buf;
	wl_iscan_results_t list;
	wl_scan_results_t *results;
	uint32 status;

	
	if (iscan->list_cur) {
		buf = iscan->list_cur;
		iscan->list_cur = buf->next;
	}
	else {
		buf = kmalloc(sizeof(iscan_buf_t), GFP_KERNEL);
		if (!buf)
			return WL_SCAN_RESULTS_ABORTED;
		buf->next = NULL;
		if (!iscan->list_hdr)
			iscan->list_hdr = buf;
		else {
			ptr = iscan->list_hdr;
			while (ptr->next) {
				ptr = ptr->next;
			}
			ptr->next = buf;
		}
	}
	memset(buf->iscan_buf, 0, WLC_IW_ISCAN_MAXLEN);
	list_buf = (wl_iscan_results_t*)buf->iscan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WLC_IW_ISCAN_MAXLEN);
	(void) dev_iw_iovar_getbuf(
		iscan->dev,
		"iscanresults",
		&list,
		WL_ISCAN_RESULTS_FIXED_SIZE,
		buf->iscan_buf,
		WLC_IW_ISCAN_MAXLEN);
	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	results->count = dtoh32(results->count);
	WL_TRACE(("results->count = %d\n", results->count));

	WL_TRACE(("results->buflen = %d\n", results->buflen));
	status = dtoh32(list_buf->status);
	return status;
}

static void wl_iw_send_scan_complete(iscan_info_t *iscan)
{
#ifndef SANDGATE2G
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];

		memset(&wrqu, 0, sizeof(wrqu));
		memset(extra, 0, sizeof(extra));
	//20120409_Broadcom CS [Case ID]519476 [START]
		wireless_send_event(iscan->dev, SIOCGIWSCAN, &wrqu, NULL);//extra);
	//20120409_Broadcom CS [Case ID]519476 [END]
		WL_TRACE(("Send Event SCAN complete\n"));
#endif
}
static int
_iscan_sysioc_thread(void *data)
{
	uint32 status;
	iscan_info_t *iscan = (iscan_info_t *)data;

	DAEMONIZE("iscan_sysioc");

	status = WL_SCAN_RESULTS_PARTIAL;
	while (down_interruptible(&iscan->sysioc_sem) == 0) {
#if defined(SOFTAP)
		if (ap_cfg_running) {
			WL_TRACE(("%s skipping SCAN ops in AP mode !!!\n", __FUNCTION__));
			continue;
		}
#endif /* SOFTAP */
		if (iscan->timer_on) {
			del_timer(&iscan->timer);
			iscan->timer_on = 0;
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
		rtnl_lock();
#endif
		status = wl_iw_iscan_get(iscan);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
		rtnl_unlock();
#endif

		switch (status) {
			case WL_SCAN_RESULTS_PARTIAL:
				WL_TRACE(("iscanresults incomplete\n"));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
				rtnl_lock();
#endif
				
				wl_iw_iscan(iscan, NULL, WL_SCAN_ACTION_CONTINUE);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
				rtnl_unlock();
#endif
				
				mod_timer(&iscan->timer, jiffies + iscan->timer_ms*HZ/1000);
				iscan->timer_on = 1;
				break;
			case WL_SCAN_RESULTS_SUCCESS:
				WL_TRACE(("iscanresults complete\n"));
				iscan->iscan_state = ISCAN_STATE_IDLE;
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
#if defined(CONFIG_LGE_BCM432X_PATCH)
				g_completed_scan_type = 0; /* 0 is ISCAN */
#endif /* CONFIG_LGE_BCM432X_PATCH */				
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
				wl_iw_send_scan_complete(iscan);
				break;
			case WL_SCAN_RESULTS_PENDING:
				WL_TRACE(("iscanresults pending\n"));
				
				mod_timer(&iscan->timer, jiffies + iscan->timer_ms*HZ/1000);
				iscan->timer_on = 1;
				break;
			case WL_SCAN_RESULTS_ABORTED:
				WL_TRACE(("iscanresults aborted\n"));
				iscan->iscan_state = ISCAN_STATE_IDLE;
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
#if defined(CONFIG_LGE_BCM432X_PATCH)
				g_completed_scan_type = 1; /* 1 is SCAN */
				if(g_completed_scan_type != g_scan_specified_ssid) {
					printk(KERN_ERR "[%s:%d]  ?????? [ISCAN results are not valid:%d]\n", 
						__func__, __LINE__, g_scan_specified_ssid);
					g_completed_scan_type = g_scan_specified_ssid;
				}
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */				
				wl_iw_send_scan_complete(iscan);
				break;
			default:
				WL_TRACE(("iscanresults returned unknown status %d\n", status));
				break;
		 }
	}

	if (iscan->timer_on) {
		del_timer(&iscan->timer);
		iscan->timer_on = 0;
	}
	complete_and_exit(&iscan->sysioc_exited, 0);
}
#endif 

#ifdef WL_IW_USE_ESCAN
static int
wl_iw_escan(escan_info_t *escan, wlc_ssid_t *ssid, uint16 action)
{
	int params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_escan_params_t, params));
	wl_escan_params_t *params;
	int err = 0;

	if (ssid && ssid->SSID_len) {
		params_size += sizeof(wlc_ssid_t);
	}
	params = (wl_escan_params_t*)kmalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		return -ENOMEM;
	}
	memset(params, 0, params_size);
	ASSERT(params_size < WLC_IOCTL_SMLEN);

	err = wl_iw_scan_prep(&params->params, ssid);

	if (!err) {
		params->version = htod32(ISCAN_REQ_VERSION);
		params->action = htod16(action);
		params->sync_id = 0x1234;

#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
		err = dev_iw_iovar_setbuf(escan->dev, "escan", params, params_size,
			escan->ioctlbuf, WLC_IOCTL_SMLEN);
#else
		(void) dev_iw_iovar_setbuf(escan->dev, "escan", params, params_size,
			escan->ioctlbuf, WLC_IOCTL_SMLEN);
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */
	}

	kfree(params);
	return err;
}

#endif 



void
wl_iw_init_ss_cache_ctrl(void)
{
	g_ss_cache_ctrl.m_prev_scan_mode = 0;
	g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
	g_ss_cache_ctrl.m_cache_head = NULL;
	g_ss_cache_ctrl.m_link_down = 0;
	g_ss_cache_ctrl.m_timer_expired = 0;
	g_ss_cache_ctrl.m_timer = NULL;
	memset(g_ss_cache_ctrl.m_active_bssid, 0, ETHER_ADDR_LEN);
}



static void
wl_iw_free_ss_cache(void)
{
	wl_iw_ss_cache_t *node, *cur;
	wl_iw_ss_cache_t **spec_scan_head;

	WL_TRACE(("%s called\n", __FUNCTION__));

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;

	for (;node;) {
		WL_TRACE(("%s : SSID - %s\n", __FUNCTION__, node->bss_info->SSID));
		cur = node;
		node = cur->next;
		kfree(cur);
	}
	*spec_scan_head = NULL;
}

static void
wl_iw_set_ss_cache_timer_flag(void)
{
	g_ss_cache_ctrl.m_timer_expired = 1;
	WL_TRACE(("%s called\n", __FUNCTION__));
}


static int
wl_iw_run_ss_cache_timer(int kick_off)
{
	struct timer_list **timer;

	timer = &g_ss_cache_ctrl.m_timer;

	if (kick_off) {
		*timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
		if (!(*timer)) {
			return -ENOMEM;
		}
		(*timer)->function = (void *)wl_iw_set_ss_cache_timer_flag;
		init_timer(*timer);
		(*timer)->expires = jiffies + 30000 * HZ / 1000;	
		add_timer(*timer);
		WL_TRACE(("%s : timer starts \n", __FUNCTION__));
	}
	else {
		if (*timer) {
			del_timer(*timer);
			kfree(*timer);
			*timer = NULL;
		}
		WL_TRACE(("%s : timer stops \n", __FUNCTION__));
	}

	return 0;
}


void
wl_iw_release_ss_cache_ctrl(void)
{
	wl_iw_free_ss_cache();
	wl_iw_run_ss_cache_timer(0);
}



static void
wl_iw_reset_ss_cache(void)
{
	wl_iw_ss_cache_t *node, *prev, *cur;
	wl_iw_ss_cache_t **spec_scan_head;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;
	prev = node;

	for (;node;) {
		WL_TRACE(("%s : node SSID %s \n", __FUNCTION__, node->bss_info->SSID));
		if (!node->dirty) {
			cur = node;
			if (cur == *spec_scan_head) {
				*spec_scan_head = cur->next;
				prev = *spec_scan_head;
			}
			else {
				prev->next = cur->next;
			}
			node = cur->next;

			WL_TRACE(("%s : Del node : SSID %s\n", __FUNCTION__, cur->bss_info->SSID));
			kfree(cur);
			continue;
		}

		node->dirty = 0;
		prev = node;
		node = node->next;
	}

}


static int
wl_iw_add_bss_to_ss_cache(wl_scan_results_t *ss_list)
{

	wl_iw_ss_cache_t *node, *prev, *leaf;
	wl_iw_ss_cache_t **spec_scan_head;
	wl_bss_info_t *bi = NULL;
	int i;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	
	if (!ss_list->count) {
		return 0;
	}

	
	for (i = 0; i < ss_list->count; i++) {

		node = *spec_scan_head;
		prev = node;

		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : ss_list->bss_info;

		WL_TRACE(("%s : %dth SSID %s\n", __FUNCTION__, i, bi->SSID));
		for (;node;) {
			if (!memcmp(&node->bss_info->BSSID, &bi->BSSID, ETHER_ADDR_LEN)) {
				
				WL_TRACE(("dirty marked : SSID %s\n", bi->SSID));
				node->dirty = 1;
				break;
			}
			prev = node;
			node = node->next;
		}

		if (node) {
			continue;
		}
		leaf = kmalloc(WLC_IW_SS_CACHE_MAXLEN, GFP_KERNEL);
		if (!leaf) {
			return -ENOMEM;
		}

		if (bi->length > WLC_IW_BSS_INFO_MAXLEN) {
			WL_TRACE(("bss info length is too long : %d\n", bi->length));
			kfree(leaf);
			continue;
		}

		memcpy(leaf->bss_info, bi, bi->length);
		leaf->next = NULL;
		leaf->dirty = 1;
		leaf->count = 1;
		leaf->version = ss_list->version;

		if (!prev) {
			*spec_scan_head = leaf;
		}
		else {
			prev->next = leaf;
		}
	}
	return 0;

}


static int
wl_iw_merge_scan_cache(struct iw_request_info *info, char *extra, uint buflen_from_user,
__u16 *merged_len)
{
	wl_iw_ss_cache_t *node;
	wl_scan_results_t *list_merge;

	node = g_ss_cache_ctrl.m_cache_head;
	for (;node;) {
		list_merge = (wl_scan_results_t *)node;
		WL_TRACE(("%s: Bcast APs list=%d\n", __FUNCTION__, list_merge->count));
		if (buflen_from_user - *merged_len > 0) {
			*merged_len += (__u16) wl_iw_get_scan_prep(list_merge, info,
				extra + *merged_len, buflen_from_user - *merged_len);
		}
		else {
			break;
		}
		node = node->next;
	}
	return 0;
}


static int
wl_iw_delete_bss_from_ss_cache(void *addr)
{

	wl_iw_ss_cache_t *node, *prev;
	wl_iw_ss_cache_t **spec_scan_head;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;
	prev = node;
	for (;node;) {
		if (!memcmp(&node->bss_info->BSSID, addr, ETHER_ADDR_LEN)) {
			if (node == *spec_scan_head) {
				*spec_scan_head = node->next;
			}
			else {
				prev->next = node->next;
			}

			WL_TRACE(("%s : Del node : %s\n", __FUNCTION__, node->bss_info->SSID));
			kfree(node);
			break;
		}

		prev = node;
		node = node->next;
	}

	memset(addr, 0, ETHER_ADDR_LEN);
	return 0;

}



static int
wl_iw_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	wlc_ssid_t ssid;

	WL_TRACE(("%s: SIOCSIWSCAN\n", dev->name));

#if defined(SOFTAP)
	if (ap_cfg_running) {
		WL_TRACE(("\n>%s: Not executed, reason -'SOFTAP is active'\n", __FUNCTION__));
		return 0;
	}
#endif /* SOFTAP */
	
	if (g_onoff == G_WLAN_SET_OFF)
		return 0;

	
	memset(&ssid, 0, sizeof(ssid));
	g_scan_specified_ssid	= 0;

#if WIRELESS_EXT > 17
	
	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			ssid.SSID_len = MIN(sizeof(ssid.SSID), req->essid_len);
			memcpy(ssid.SSID, req->essid, ssid.SSID_len);
			ssid.SSID_len = htod32(ssid.SSID_len);
			g_scan_specified_ssid = 1;
			WL_TRACE(("Specific scan ssid=%s len=%d\n", ssid.SSID, ssid.SSID_len));
		}
	}
#endif
	
	(void) dev_wlc_ioctl(dev, WLC_SCAN, &ssid, sizeof(ssid));

	return 0;
}

#ifdef WL_IW_USE_ISCAN
static int
wl_iw_iscan_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	wlc_ssid_t ssid;
	iscan_info_t *iscan = g_iscan;

	WL_TRACE(("%s: SIOCSIWSCAN\n", dev->name));

	
	if (g_onoff == G_WLAN_SET_OFF)
		return 0;

	
	if ((!iscan) || (iscan->sysioc_pid < 0)) {
		return wl_iw_set_scan(dev, info, wrqu, extra);
	}
	if (iscan->iscan_state == ISCAN_STATE_SCANING) {
		return 0;
	}

	
	memset(&ssid, 0, sizeof(ssid));

	g_scan_specified_ssid = 0;
#if WIRELESS_EXT > 17
	
	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			int as = 0;
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			ssid.SSID_len = MIN(sizeof(ssid.SSID), req->essid_len);
			memcpy(ssid.SSID, req->essid, ssid.SSID_len);
			ssid.SSID_len = htod32(ssid.SSID_len);
			g_scan_specified_ssid = 1;
			dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as));
			return wl_iw_set_scan(dev, info, wrqu, extra);
		}
	}
#endif 

	iscan->list_cur = iscan->list_hdr;
	iscan->iscan_state = ISCAN_STATE_SCANING;
	(void) dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN,
		&iscan->scan_flag, sizeof(iscan->scan_flag));
	wl_iw_set_event_mask(dev);
	wl_iw_iscan(iscan, &ssid, WL_SCAN_ACTION_START);
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-05-08, to avoid duplicated add_timer */
#if defined(CONFIG_LGE_BCM432X_PATCH)
	if (iscan->timer_on) {
		del_timer(&iscan->timer);
		iscan->timer_on = 0;
	}
#else /* CONFIG_LGE_BCM432X_PATCH */
	del_timer(&iscan->timer);
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-05-08, to avoid duplicated add_timer */
	iscan->timer.expires = jiffies + iscan->timer_ms*HZ/1000;
	add_timer(&iscan->timer);
	iscan->timer_on = 1;

	return 0;
}
#endif 

#ifdef WL_IW_USE_ESCAN
static int
wl_iw_escan_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	wlc_ssid_t ssid;
	escan_info_t *escan = g_escan;
	wl_scan_results_t *results;
#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
	int err = 0;
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */

	WL_TRACE(("%s: SIOCSIWSCAN\n", dev->name));

#if defined(SOFTAP)
	if (ap_cfg_running) {
		WL_TRACE(("\n>%s: Not executed, reason -'SOFTAP is active'\n", __FUNCTION__));
		return 0;
	}
#endif /* SOFTAP */
	
	if (g_onoff == G_WLAN_SET_OFF)
		return 0;

	if (!escan)
		return wl_iw_set_scan(dev, info, wrqu, extra);
	if (escan->escan_state == ESCAN_STATE_SCANING) {
		return 0;
	}

	
	memset(&ssid, 0, sizeof(ssid));
	g_scan_specified_ssid = 0;

#if WIRELESS_EXT > 17
	
	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			int as = 0;
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			ssid.SSID_len = MIN(sizeof(ssid.SSID), req->essid_len);
			memcpy(ssid.SSID, req->essid, ssid.SSID_len);
			ssid.SSID_len = htod32(ssid.SSID_len);
			g_scan_specified_ssid = 1;
			dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as));
			return wl_iw_set_scan(dev, info, wrqu, extra);
		}
	}
#endif 

	escan->escan_state = ESCAN_STATE_SCANING;
	results = (wl_scan_results_t *) escan->escan_buf;
	results->version = 0;
	results->count = 0;
	results->buflen = WL_SCAN_RESULTS_FIXED_SIZE;

#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
	err = dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN,
		&escan->scan_flag, sizeof(escan->scan_flag));
#else
	(void) dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN,
		&escan->scan_flag, sizeof(escan->scan_flag));
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */
	wl_iw_set_event_mask(dev);
#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
	err = wl_iw_escan(escan, &ssid, WL_SCAN_ACTION_START);
#else
	wl_iw_escan(escan, &ssid, WL_SCAN_ACTION_START);
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */

#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
	if(err < 0)
		escan->escan_state = ESCAN_STATE_IDLE;
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */

#ifdef CONFIG_LGE_BCM432X_PATCH // for escan fail
	return err;
#else
	return 0;
#endif /* CONFIG_LGE_BCM432X_PATCH for escan fail */
}
#endif 

#if WIRELESS_EXT > 17
static bool
ie_is_wpa_ie(uint8 **wpaie, uint8 **tlvs, int *tlvs_len)
{


	uint8 *ie = *wpaie;

	
	if ((ie[1] >= 6) &&
		!bcmp((const void *)&ie[2], (const void *)(WPA_OUI "\x01"), 4)) {
		return TRUE;
	}

	
	ie += ie[1] + 2;
	
	*tlvs_len -= (int)(ie - *tlvs);
	
	*tlvs = ie;
	return FALSE;
}

static bool
ie_is_wps_ie(uint8 **wpsie, uint8 **tlvs, int *tlvs_len)
{


	uint8 *ie = *wpsie;

	
	if ((ie[1] >= 4) &&
		!bcmp((const void *)&ie[2], (const void *)(WPA_OUI "\x04"), 4)) {
		return TRUE;
	}

	
	ie += ie[1] + 2;
	
	*tlvs_len -= (int)(ie - *tlvs);
	
	*tlvs = ie;
	return FALSE;
}
#endif 


static int
wl_iw_handle_scanresults_ies(char **event_p, char *end,
	struct iw_request_info *info, wl_bss_info_t *bi)
{
#if WIRELESS_EXT > 17
	struct iw_event	iwe;
	char *event;

	event = *event_p;
	if (bi->ie_length) {
		
		bcm_tlv_t *ie;
		uint8 *ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
		int ptr_len = bi->ie_length;

#ifdef BCMWPA2
		if ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_RSN_ID))) {
			iwe.cmd = IWEVGENIE;
			iwe.u.data.length = ie->len + 2;
			event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
		}
		ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
#endif 

		while ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_WPA_ID))) {
			
			if (ie_is_wps_ie(((uint8 **)&ie), &ptr, &ptr_len)) {
				iwe.cmd = IWEVGENIE;
				iwe.u.data.length = ie->len + 2;
				event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
				break;
			}
		}

		ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
		ptr_len = bi->ie_length;
		while ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_WPA_ID))) {
			if (ie_is_wpa_ie(((uint8 **)&ie), &ptr, &ptr_len)) {
				iwe.cmd = IWEVGENIE;
				iwe.u.data.length = ie->len + 2;
				event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
				break;
			}
		}

	*event_p = event;
	}
#endif 

	return 0;
}
static uint
wl_iw_get_scan_prep(
	wl_scan_results_t *list,
	struct iw_request_info *info,
	char *extra,
	short max_size)
{
	int  i, j;
	struct iw_event  iwe;
	wl_bss_info_t *bi = NULL;
	char *event = extra, *end = extra + max_size - WE_ADD_EVENT_FIX, *value;
	uint	ret = 0;

	ASSERT(list);

	

	for (i = 0; i < list->count && i < IW_MAX_AP; i++)
	{
		ASSERT(list->version == WL_BSS_INFO_VERSION);

		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;

		
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_ADDR_LEN);
		
		iwe.u.data.length = dtoh32(bi->SSID_len);
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.flags = 1;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, bi->SSID);

		
		if (dtoh16(bi->capability) & (DOT11_CAP_ESS | DOT11_CAP_IBSS)) {
			iwe.cmd = SIOCGIWMODE;
			if (dtoh16(bi->capability) & DOT11_CAP_ESS)
				iwe.u.mode = IW_MODE_INFRA;
			else
				iwe.u.mode = IW_MODE_ADHOC;
			event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_UINT_LEN);
		}

		
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->chanspec),
			CHSPEC_CHANNEL(bi->chanspec) <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		iwe.u.freq.e = 6;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_FREQ_LEN);

		
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.qual = rssi_to_qual(dtoh16(bi->RSSI));
		iwe.u.qual.level = 0x100 + dtoh16(bi->RSSI);
		iwe.u.qual.noise = 0x100 + bi->phy_noise;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_QUAL_LEN);

		
		 wl_iw_handle_scanresults_ies(&event, end, info, bi);

		
		iwe.cmd = SIOCGIWENCODE;
		if (dtoh16(bi->capability) & DOT11_CAP_PRIVACY)
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		iwe.u.data.length = 0;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)event);

		
		if (bi->rateset.count) {
			if (((event - extra) + IW_EV_LCP_LEN) <= (uintptr)end) {
				value = event + IW_EV_LCP_LEN;
				iwe.cmd = SIOCGIWRATE;
				
				iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
				for (j = 0; j < bi->rateset.count && j < IW_MAX_BITRATES; j++) {
					iwe.u.bitrate.value =
						(bi->rateset.rates[j] & 0x7f) * 500000;
					value = IWE_STREAM_ADD_VALUE(info, event, value, end, &iwe,
						IW_EV_PARAM_LEN);
				}
				event = value;
			}
		}
	} 

	if ((ret = (event - extra)) < 0) {
		WL_ERROR(("==> Wrong size\n"));
		ret = 0;
	}
	WL_TRACE(("%s: size=%d bytes prepared \n", __FUNCTION__, (unsigned int)(event - extra)));
	return ret;

}

static int
wl_iw_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	channel_info_t ci;
	wl_scan_results_t *list_merge;
	wl_scan_results_t *list = (wl_scan_results_t *) g_scan;
	int error;
	uint buflen_from_user = dwrq->length;
#if defined(CONFIG_LGE_BCM432X_PATCH) && (DHD_USE_STATIC_BUF)
	uint len =  G_SCAN_RESULTS - 32;
#else
	uint len =  G_SCAN_RESULTS;
#endif //DHD_USE_STATIC_BUF
	__u16 len_ret = 0;
	__u16 merged_len = 0;

#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = g_iscan;
	iscan_buf_t * p_buf;
#endif

	WL_TRACE(("%s: buflen_from_user %d: \n", dev->name, buflen_from_user));

// WBT Fix TD# 37096
	memset(&ci, 0, sizeof(channel_info_t));
//

	if (!extra) {
		WL_TRACE(("%s: wl_iw_get_scan return -EINVAL\n", dev->name));
		return -EINVAL;
	}

	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(ci)))) {
		return error;
	}
	ci.scan_channel = dtoh32(ci.scan_channel);
	if (ci.scan_channel) {
		return -EAGAIN;
	}
	if (g_ss_cache_ctrl.m_timer_expired) {
		wl_iw_free_ss_cache();
		g_ss_cache_ctrl.m_timer_expired ^= 1;
	}
	if ((!g_scan_specified_ssid && g_ss_cache_ctrl.m_prev_scan_mode) ||
		g_ss_cache_ctrl.m_cons_br_scan_cnt > 4) {
		g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
		
		wl_iw_reset_ss_cache();
	}
	g_ss_cache_ctrl.m_prev_scan_mode = g_scan_specified_ssid;
	if (g_scan_specified_ssid) {
		g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
	}
	else {
		g_ss_cache_ctrl.m_cons_br_scan_cnt++;
	}


	
	if (g_scan_specified_ssid) {
		
		list = kmalloc(len, GFP_KERNEL);
		if (!list) {
			WL_TRACE(("%s: wl_iw_get_scan return -ENOMEM\n", dev->name));
			return -ENOMEM;
		}
	}

	memset(list, 0, len);
	list->buflen = htod32(len);
	if ((error = dev_wlc_ioctl(dev, WLC_SCAN_RESULTS, list, len))) {
		WL_TRACE(("%s: %s : Scan_results too big %d\n", dev->name, __FUNCTION__, len));
		dwrq->length = len;
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN)
		if (g_scan_specified_ssid) {
			list->count = 0;
			printk(KERN_ERR "[%s: %d] : You may only get ISCAN results, list->count: %d\n",
				__func__, __LINE__, list->count);
		}
#else /* defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN) */
		if (g_scan_specified_ssid)
			kfree(list);
		return 0;
#endif /* defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN) */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
	}
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-08-16, fix the getting ISCAN/SCAN results */
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN)
	else {
#endif /* defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN) */		
	list->buflen = dtoh32(list->buflen);
	list->version = dtoh32(list->version);
	list->count = dtoh32(list->count);
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN)
	}
#endif /* defined(CONFIG_LGE_BCM432X_PATCH) && defined(WL_IW_USE_ISCAN) */		

	if (g_scan_specified_ssid) {
		
		wl_iw_add_bss_to_ss_cache(list);
		kfree(list);
	}

#if defined(WL_IW_USE_ISCAN)
	p_buf = iscan->list_hdr;
	
	while (p_buf != iscan->list_cur) {
		list_merge = &((wl_iscan_results_t*)p_buf->iscan_buf)->results;
		WL_TRACE(("%s: Bcast APs list=%d\n", __FUNCTION__, list_merge->count));
		if (list_merge->count > 0)
			len_ret += (__u16) wl_iw_get_scan_prep(list_merge, info,
				extra+len_ret, buflen_from_user -len_ret);
		p_buf = p_buf->next;
	}
#elif defined(WL_IW_USE_ESCAN)
	list_merge = (wl_scan_results_t *) g_escan->escan_buf;
	if (list_merge->count > 0) {
		// louislee add len_ret for scan list count
		len_ret = (__u16) wl_iw_get_scan_prep(list_merge, info, extra, buflen_from_user);
	}
#else
	list_merge = (wl_scan_results_t *) g_scan;
	len_ret = (__u16) wl_iw_get_scan_prep(list_merge, info, extra, buflen_from_user);
#endif 
	if (g_ss_cache_ctrl.m_link_down) {
		
		wl_iw_delete_bss_from_ss_cache(g_ss_cache_ctrl.m_active_bssid);
	}
	
	wl_iw_merge_scan_cache(info, extra+len_ret, buflen_from_user-len_ret, &merged_len);
	len_ret += merged_len;
	wl_iw_run_ss_cache_timer(0);
	wl_iw_run_ss_cache_timer(1);


	
	if ((len_ret + WE_ADD_EVENT_FIX) < buflen_from_user)
		len = len_ret;

	dwrq->length = len;
	dwrq->flags = 0;	

	WL_TRACE(("%s return to WE %d bytes APs=%d\n", __FUNCTION__, dwrq->length, list->count));
	return 0;
}

#if defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN)
static int
wl_iw_scan_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
	struct iw_event	iwe;
	wl_bss_info_t *bi = NULL;
	int ii, j;
	int apcnt;
	char *event = extra, *end = extra + dwrq->length, *value;
#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = g_iscan;
	iscan_buf_t * p_buf;
#endif
	uint32  counter = 0;
	__u16 merged_len = 0;
	uint buflen_from_user = dwrq->length;


	WL_TRACE(("%s %s buflen_from_user %d:\n", dev->name, __FUNCTION__, dwrq->length));

#if defined(SOFTAP)
	if (ap_cfg_running) {
		WL_TRACE(("%s: Not executed, reason -'SOFTAP is active'\n", __FUNCTION__));
		return -EINVAL;
	}
#endif /* SOFTAP */

	if (!extra) {
		WL_TRACE(("%s: SIOCGIWSCAN GET bad parameter\n", dev->name));
		return -EINVAL;
	}

#if defined(WL_IW_USE_ISCAN)
	
	if ((!iscan) || (iscan->sysioc_pid < 0)) {
#elif defined(WL_IW_USE_ESCAN)
	if (!g_escan) {
#endif
		return wl_iw_get_scan(dev, info, dwrq, extra);
	}

	

	if (g_ss_cache_ctrl.m_timer_expired) {
		wl_iw_free_ss_cache();
		g_ss_cache_ctrl.m_timer_expired ^= 1;
	}
	if (g_scan_specified_ssid) {
		return wl_iw_get_scan(dev, info, dwrq, extra);
	}
	else {
		if (g_ss_cache_ctrl.m_link_down) {
			
			wl_iw_delete_bss_from_ss_cache(g_ss_cache_ctrl.m_active_bssid);
		}
		if (g_ss_cache_ctrl.m_prev_scan_mode || g_ss_cache_ctrl.m_cons_br_scan_cnt > 4) {
			g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
			
			wl_iw_reset_ss_cache();
		}
		g_ss_cache_ctrl.m_prev_scan_mode = g_scan_specified_ssid;
		g_ss_cache_ctrl.m_cons_br_scan_cnt++;
	}


	WL_TRACE(("%s: SIOCGIWSCAN GET broadcast results\n", dev->name));
	apcnt = 0;
#if defined(WL_IW_USE_ISCAN)
	p_buf = iscan->list_hdr;
	
	while (p_buf != iscan->list_cur) {
	    list = &((wl_iscan_results_t*)p_buf->iscan_buf)->results;
#elif defined(WL_IW_USE_ESCAN)
//	{	LGE_CHANGE_S [WiFi][kyoohyun.kim@lge.com] Ioctl Fail bug fix  
		list = (wl_scan_results_t *)g_escan->escan_buf;
/* LGE_CHANGE_S [WiFi][kyoohyun.kim@lge.com] Ioctl Fail bug fix  */
		if (list->count > 0)
		{
/* LGE_CHANGE_E [WiFi][kyoohyun.kim@lge.com] Ioctl Fail bug fix  */ 	
#endif
	    counter += list->count;

	    if (list->version != WL_BSS_INFO_VERSION) {
			WL_ERROR(("list->version %d != WL_BSS_INFO_VERSION\n", list->version));
			return -EINVAL;
	    }

	    bi = NULL;
	    for (ii = 0; ii < list->count && apcnt < IW_MAX_AP; apcnt++, ii++) {
		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;
#if defined(WL_IW_USE_ISCAN)
		ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
			WLC_IW_ISCAN_MAXLEN));
#endif

		
		if (event + ETHER_ADDR_LEN + bi->SSID_len + IW_EV_UINT_LEN + IW_EV_FREQ_LEN +
			IW_EV_QUAL_LEN >= end)
			return -E2BIG;
		
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_ADDR_LEN);

		
		iwe.u.data.length = dtoh32(bi->SSID_len);
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.flags = 1;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, bi->SSID);

		
		if (dtoh16(bi->capability) & (DOT11_CAP_ESS | DOT11_CAP_IBSS)) {
			iwe.cmd = SIOCGIWMODE;
			if (dtoh16(bi->capability) & DOT11_CAP_ESS)
				iwe.u.mode = IW_MODE_INFRA;
			else
				iwe.u.mode = IW_MODE_ADHOC;
			event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_UINT_LEN);
		}

		
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->chanspec),
			CHSPEC_CHANNEL(bi->chanspec) <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		iwe.u.freq.e = 6;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_FREQ_LEN);

		
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.qual = rssi_to_qual(dtoh16(bi->RSSI));
		iwe.u.qual.level = 0x100 + dtoh16(bi->RSSI);
		iwe.u.qual.noise = 0x100 + bi->phy_noise;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_QUAL_LEN);

		
		wl_iw_handle_scanresults_ies(&event, end, info, bi);

		
		iwe.cmd = SIOCGIWENCODE;
		if (dtoh16(bi->capability) & DOT11_CAP_PRIVACY)
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		iwe.u.data.length = 0;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)event);

		
		if (bi->rateset.count) {
			if (event + IW_MAX_BITRATES*IW_EV_PARAM_LEN >= end)
				return -E2BIG;

			value = event + IW_EV_LCP_LEN;
			iwe.cmd = SIOCGIWRATE;
			
			iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
			for (j = 0; j < bi->rateset.count && j < IW_MAX_BITRATES; j++) {
				iwe.u.bitrate.value = (bi->rateset.rates[j] & 0x7f) * 500000;
				value = IWE_STREAM_ADD_VALUE(info, event, value, end, &iwe,
					IW_EV_PARAM_LEN);
			}
			event = value;
		}
	    }
#if defined(WL_IW_USE_ISCAN)
	    p_buf = p_buf->next;
#endif
	} 

	dwrq->length = event - extra;
	dwrq->flags = 0;	

	
	wl_iw_merge_scan_cache(info, event, buflen_from_user - dwrq->length, &merged_len);
	dwrq->length += merged_len;
	wl_iw_run_ss_cache_timer(0);
	wl_iw_run_ss_cache_timer(1);

	WL_TRACE(("%s return to WE %d bytes APs=%d\n", __FUNCTION__, dwrq->length, counter));

	return 0;
}
#endif 

static int
wl_iw_set_essid(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	int error;

	WL_TRACE(("%s: SIOCSIWESSID\n", dev->name));

	
	memset(&g_ssid, 0, sizeof(g_ssid));

	if (dwrq->length && extra) {
#if WIRELESS_EXT > 20
		g_ssid.SSID_len = MIN(sizeof(g_ssid.SSID), dwrq->length);
#else
		g_ssid.SSID_len = MIN(sizeof(g_ssid.SSID), dwrq->length-1);
#endif
		memcpy(g_ssid.SSID, extra, g_ssid.SSID_len);
	} else {
		
		g_ssid.SSID_len = 0;
	}
	g_ssid.SSID_len = htod32(g_ssid.SSID_len);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_SSID, &g_ssid, sizeof(g_ssid))))
		return error;

	return 0;
}

static int
wl_iw_get_essid(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wlc_ssid_t ssid;
	int error;

	WL_TRACE(("%s: SIOCGIWESSID\n", dev->name));

// WBT Fix TD# 37085, 37086
	memset(&ssid, 0, sizeof(wlc_ssid_t));
//

	if (!extra)
		return -EINVAL;

	if ((error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid)))) {
		WL_ERROR(("Error getting the SSID\n"));
		return error;
	}

	ssid.SSID_len = dtoh32(ssid.SSID_len);

	
	memcpy(extra, ssid.SSID, ssid.SSID_len);

	dwrq->length = ssid.SSID_len;

	dwrq->flags = 1; 

	return 0;
}

static int
wl_iw_set_nick(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCSIWNICKN\n", dev->name));

	if (!extra)
		return -EINVAL;

	
	if (dwrq->length > sizeof(iw->nickname))
		return -E2BIG;

	memcpy(iw->nickname, extra, dwrq->length);
	iw->nickname[dwrq->length - 1] = '\0';

	return 0;
}

static int
wl_iw_get_nick(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCGIWNICKN\n", dev->name));

	if (!extra)
		return -EINVAL;

	strcpy(extra, iw->nickname);
	dwrq->length = strlen(extra) + 1;

	return 0;
}

static int wl_iw_set_rate(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	wl_rateset_t rateset;
	int error, rate, i, error_bg, error_a;

	WL_TRACE(("%s: SIOCSIWRATE\n", dev->name));

// WBT Fix TD# 37087, 37088
	memset(&rateset, 0, sizeof(wl_rateset_t));
//
	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_CURR_RATESET, &rateset, sizeof(rateset))))
		return error;

	rateset.count = dtoh32(rateset.count);

	if (vwrq->value < 0) {
		
		rate = rateset.rates[rateset.count - 1] & 0x7f;
	} else if (vwrq->value < rateset.count) {
		
		rate = rateset.rates[vwrq->value] & 0x7f;
	} else {
		
		rate = vwrq->value / 500000;
	}

	if (vwrq->fixed) {
		
		error_bg = dev_wlc_intvar_set(dev, "bg_rate", rate);
		error_a = dev_wlc_intvar_set(dev, "a_rate", rate);

		if (error_bg && error_a)
			return (error_bg | error_a);
	} else {
		
		
		error_bg = dev_wlc_intvar_set(dev, "bg_rate", 0);
		
		error_a = dev_wlc_intvar_set(dev, "a_rate", 0);

		if (error_bg && error_a)
			return (error_bg | error_a);

		
		for (i = 0; i < rateset.count; i++)
			if ((rateset.rates[i] & 0x7f) > rate)
				break;
		rateset.count = htod32(i);

		
		if ((error = dev_wlc_ioctl(dev, WLC_SET_RATESET, &rateset, sizeof(rateset))))
			return error;
	}

	return 0;
}

static int wl_iw_get_rate(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
// WBT Fix TD# 37089
	int error, rate = 0;
//

	WL_TRACE(("%s: SIOCGIWRATE\n", dev->name));

	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_RATE, &rate, sizeof(rate))))
		return error;
	rate = dtoh32(rate);
	vwrq->value = rate * 500000;

	return 0;
}

static int
wl_iw_set_rts(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, rts;

	WL_TRACE(("%s: SIOCSIWRTS\n", dev->name));

	if (vwrq->disabled)
		rts = DOT11_DEFAULT_RTS_LEN;
	else if (vwrq->value < 0 || vwrq->value > DOT11_DEFAULT_RTS_LEN)
		return -EINVAL;
	else
		rts = vwrq->value;

	if ((error = dev_wlc_intvar_set(dev, "rtsthresh", rts)))
		return error;

	return 0;
}

static int
wl_iw_get_rts(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, rts;

	WL_TRACE(("%s: SIOCGIWRTS\n", dev->name));

	if ((error = dev_wlc_intvar_get(dev, "rtsthresh", &rts)))
		return error;

	vwrq->value = rts;
	vwrq->disabled = (rts >= DOT11_DEFAULT_RTS_LEN);
	vwrq->fixed = 1;

	return 0;
}

static int
wl_iw_set_frag(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, frag;

	WL_TRACE(("%s: SIOCSIWFRAG\n", dev->name));

	if (vwrq->disabled)
		frag = DOT11_DEFAULT_FRAG_LEN;
	else if (vwrq->value < 0 || vwrq->value > DOT11_DEFAULT_FRAG_LEN)
		return -EINVAL;
	else
		frag = vwrq->value;

	if ((error = dev_wlc_intvar_set(dev, "fragthresh", frag)))
		return error;

	return 0;
}

static int
wl_iw_get_frag(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, fragthreshold;

	WL_TRACE(("%s: SIOCGIWFRAG\n", dev->name));

	if ((error = dev_wlc_intvar_get(dev, "fragthresh", &fragthreshold)))
		return error;

	vwrq->value = fragthreshold;
	vwrq->disabled = (fragthreshold >= DOT11_DEFAULT_FRAG_LEN);
	vwrq->fixed = 1;

	return 0;
}

static int
wl_iw_set_txpow(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, disable;
	uint16 txpwrmw;
	WL_TRACE(("%s: SIOCSIWTXPOW\n", dev->name));

	
	disable = vwrq->disabled ? WL_RADIO_SW_DISABLE : 0;
	disable += WL_RADIO_SW_DISABLE << 16;

	disable = htod32(disable);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_RADIO, &disable, sizeof(disable))))
		return error;

	
	if (disable & WL_RADIO_SW_DISABLE)
		return 0;

	
	if (!(vwrq->flags & IW_TXPOW_MWATT))
		return -EINVAL;

	
	if (vwrq->value < 0)
		return 0;

	if (vwrq->value > 0xffff) txpwrmw = 0xffff;
	else txpwrmw = (uint16)vwrq->value;


	error = dev_wlc_intvar_set(dev, "qtxpower", (int)(bcm_mw_to_qdbm(txpwrmw)));
	return error;
}

static int
wl_iw_get_txpow(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
// WBT Fix TD# 37090
	int error, disable = 0, txpwrdbm = 0;
//
	uint8 result;

	WL_TRACE(("%s: SIOCGIWTXPOW\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_RADIO, &disable, sizeof(disable))) ||
	    (error = dev_wlc_intvar_get(dev, "qtxpower", &txpwrdbm)))
		return error;

	disable = dtoh32(disable);
	result = (uint8)(txpwrdbm & ~WL_TXPWR_OVERRIDE);
	vwrq->value = (int32)bcm_qdbm_to_mw(result);
	vwrq->fixed = 0;
	vwrq->disabled = (disable & (WL_RADIO_SW_DISABLE | WL_RADIO_HW_DISABLE)) ? 1 : 0;
	vwrq->flags = IW_TXPOW_MWATT;

	return 0;
}

#if WIRELESS_EXT > 10
static int
wl_iw_set_retry(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, lrl, srl;

	WL_TRACE(("%s: SIOCSIWRETRY\n", dev->name));

	
	if (vwrq->disabled || (vwrq->flags & IW_RETRY_LIFETIME))
		return -EINVAL;

	
	if (vwrq->flags & IW_RETRY_LIMIT) {

		
#if WIRELESS_EXT > 20
	if ((vwrq->flags & IW_RETRY_LONG) ||(vwrq->flags & IW_RETRY_MAX) ||
		!((vwrq->flags & IW_RETRY_SHORT) || (vwrq->flags & IW_RETRY_MIN))) {
#else
	if ((vwrq->flags & IW_RETRY_MAX) || !(vwrq->flags & IW_RETRY_MIN)) {
#endif 
			lrl = htod32(vwrq->value);
			if ((error = dev_wlc_ioctl(dev, WLC_SET_LRL, &lrl, sizeof(lrl))))
				return error;
		}

		
#if WIRELESS_EXT > 20
	if ((vwrq->flags & IW_RETRY_SHORT) ||(vwrq->flags & IW_RETRY_MIN) ||
		!((vwrq->flags & IW_RETRY_LONG) || (vwrq->flags & IW_RETRY_MAX))) {
#else
		if ((vwrq->flags & IW_RETRY_MIN) || !(vwrq->flags & IW_RETRY_MAX)) {
#endif 
			srl = htod32(vwrq->value);
			if ((error = dev_wlc_ioctl(dev, WLC_SET_SRL, &srl, sizeof(srl))))
				return error;
		}
	}

	return 0;
}

static int
wl_iw_get_retry(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
// WBT Fix TD# 37091, 37092
	int error, lrl = 0, srl = 0;
//

	WL_TRACE(("%s: SIOCGIWRETRY\n", dev->name));

	vwrq->disabled = 0;      

	
	if ((vwrq->flags & IW_RETRY_TYPE) == IW_RETRY_LIFETIME)
		return -EINVAL;

	
	if ((error = dev_wlc_ioctl(dev, WLC_GET_LRL, &lrl, sizeof(lrl))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_SRL, &srl, sizeof(srl))))
		return error;

	lrl = dtoh32(lrl);
	srl = dtoh32(srl);

	
	if (vwrq->flags & IW_RETRY_MAX) {
		vwrq->flags = IW_RETRY_LIMIT | IW_RETRY_MAX;
		vwrq->value = lrl;
	} else {
		vwrq->flags = IW_RETRY_LIMIT;
		vwrq->value = srl;
		if (srl != lrl)
			vwrq->flags |= IW_RETRY_MIN;
	}

	return 0;
}
#endif 

static int
wl_iw_set_encode(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
	int error, val, wsec;

	WL_TRACE(("%s: SIOCSIWENCODE\n", dev->name));

	memset(&key, 0, sizeof(key));

	if ((dwrq->flags & IW_ENCODE_INDEX) == 0) {
		
		for (key.index = 0; key.index < DOT11_MAX_DEFAULT_KEYS; key.index++) {
			val = htod32(key.index);
			if ((error = dev_wlc_ioctl(dev, WLC_GET_KEY_PRIMARY, &val, sizeof(val))))
				return error;
			val = dtoh32(val);
			if (val)
				break;
		}
		
		if (key.index == DOT11_MAX_DEFAULT_KEYS)
			key.index = 0;
	} else {
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;
		if (key.index >= DOT11_MAX_DEFAULT_KEYS)
			return -EINVAL;
	}

	
	if (!extra || !dwrq->length || (dwrq->flags & IW_ENCODE_NOKEY)) {
		
		val = htod32(key.index);
		if ((error = dev_wlc_ioctl(dev, WLC_SET_KEY_PRIMARY, &val, sizeof(val))))
			return error;
	} else {
		key.len = dwrq->length;

		if (dwrq->length > sizeof(key.data))
			return -EINVAL;

		memcpy(key.data, extra, dwrq->length);

		key.flags = WL_PRIMARY_KEY;
		switch (key.len) {
		case WEP1_KEY_SIZE:
			key.algo = CRYPTO_ALGO_WEP1;
			break;
		case WEP128_KEY_SIZE:
			key.algo = CRYPTO_ALGO_WEP128;
			break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
		case TKIP_KEY_SIZE:
			key.algo = CRYPTO_ALGO_TKIP;
			break;
#endif
		case AES_KEY_SIZE:
			key.algo = CRYPTO_ALGO_AES_CCM;
			break;
		default:
			return -EINVAL;
		}

		
		swap_key_from_BE(&key);
		if ((error = dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key))))
			return error;
	}

	
	val = (dwrq->flags & IW_ENCODE_DISABLED) ? 0 : WEP_ENABLED;

	if ((error = dev_wlc_intvar_get(dev, "wsec", &wsec)))
		return error;

	wsec  &= ~(WEP_ENABLED);
	wsec |= val;

	if ((error = dev_wlc_intvar_set(dev, "wsec", wsec)))
		return error;

	
	val = (dwrq->flags & IW_ENCODE_RESTRICTED) ? 1 : 0;
	val = htod32(val);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_AUTH, &val, sizeof(val))))
		return error;

	return 0;
}

static int
wl_iw_get_encode(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
// WBT Fix TD# 37093, 37094
	int error, val, wsec = 0, auth = 0;
//

	WL_TRACE(("%s: SIOCGIWENCODE\n", dev->name));

	
	bzero(&key, sizeof(wl_wsec_key_t));

	if ((dwrq->flags & IW_ENCODE_INDEX) == 0) {
		
		for (key.index = 0; key.index < DOT11_MAX_DEFAULT_KEYS; key.index++) {
			val = key.index;
			if ((error = dev_wlc_ioctl(dev, WLC_GET_KEY_PRIMARY, &val, sizeof(val))))
				return error;
			val = dtoh32(val);
			if (val)
				break;
		}
	} else
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;

	if (key.index >= DOT11_MAX_DEFAULT_KEYS)
		key.index = 0;

	

	if ((error = dev_wlc_ioctl(dev, WLC_GET_WSEC, &wsec, sizeof(wsec))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_AUTH, &auth, sizeof(auth))))
		return error;

	swap_key_to_BE(&key);

	wsec = dtoh32(wsec);
	auth = dtoh32(auth);
	
	dwrq->length = MIN(IW_ENCODING_TOKEN_MAX, key.len);

	
	dwrq->flags = key.index + 1;
	if (!(wsec & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED))) {
		
		dwrq->flags |= IW_ENCODE_DISABLED;
	}
	if (auth) {
		
		dwrq->flags |= IW_ENCODE_RESTRICTED;
	}

	
	if (dwrq->length && extra)
		memcpy(extra, key.data, dwrq->length);

	return 0;
}

static int
wl_iw_set_power(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, pm;

	WL_TRACE(("%s: SIOCSIWPOWER\n", dev->name));

	//bill.jung@lge.com - Don't use legacy power save mode.
	//pm = vwrq->disabled ? PM_OFF : PM_MAX;
	pm = vwrq->disabled ? PM_OFF : PM_FAST;

	pm = htod32(pm);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm))))
		return error;

	return 0;
}

static int
wl_iw_get_power(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
// WBT Fix TD# 37095
	int error, pm = 0;

	WL_TRACE(("%s: SIOCGIWPOWER\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_PM, &pm, sizeof(pm))))
		return error;

	pm = dtoh32(pm);
	vwrq->disabled = pm ? 0 : 1;
	vwrq->flags = IW_POWER_ALL_R;

	return 0;
}

#if WIRELESS_EXT > 17
static int
wl_iw_set_wpaie(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *iwp,
	char *extra
)
{
		dev_wlc_bufvar_set(dev, "wpaie", extra, iwp->length);

	return 0;
}

static int
wl_iw_get_wpaie(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *iwp,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWGENIE\n", dev->name));
	iwp->length = 64;
	dev_wlc_bufvar_get(dev, "wpaie", extra, iwp->length);
	return 0;
}

static int
wl_iw_set_encodeext(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
	int error;
	struct iw_encode_ext *iwe;

	WL_TRACE(("%s: SIOCSIWENCODEEXT\n", dev->name));

	memset(&key, 0, sizeof(key));
	iwe = (struct iw_encode_ext *)extra;

	
	if (dwrq->flags & IW_ENCODE_DISABLED) {

	}

	
	key.index = 0;
	if (dwrq->flags & IW_ENCODE_INDEX)
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;

	key.len = iwe->key_len;

	
	if (!ETHER_ISMULTI(iwe->addr.sa_data))
		bcopy((void *)&iwe->addr.sa_data, (char *)&key.ea, ETHER_ADDR_LEN);

	
	if (key.len == 0) {
		if (iwe->ext_flags & IW_ENCODE_EXT_SET_TX_KEY) {
			WL_WSEC(("Changing the the primary Key to %d\n", key.index));
			
			key.index = htod32(key.index);
			error = dev_wlc_ioctl(dev, WLC_SET_KEY_PRIMARY,
				&key.index, sizeof(key.index));
			if (error)
				return error;
		}
		
		else {
			swap_key_from_BE(&key);
			dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));
		}
	}
	else {
		if (iwe->key_len > sizeof(key.data))
			return -EINVAL;

		WL_WSEC(("Setting the key index %d\n", key.index));
		if (iwe->ext_flags & IW_ENCODE_EXT_SET_TX_KEY) {
			WL_WSEC(("key is a Primary Key\n"));
			key.flags = WL_PRIMARY_KEY;
		}

		bcopy((void *)iwe->key, key.data, iwe->key_len);

		if (iwe->alg == IW_ENCODE_ALG_TKIP) {
			uint8 keybuf[8];
			bcopy(&key.data[24], keybuf, sizeof(keybuf));
			bcopy(&key.data[16], &key.data[24], sizeof(keybuf));
			bcopy(keybuf, &key.data[16], sizeof(keybuf));
		}

		
		if (iwe->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) {
			uchar *ivptr;
			ivptr = (uchar *)iwe->rx_seq;
			key.rxiv.hi = (ivptr[5] << 24) | (ivptr[4] << 16) |
				(ivptr[3] << 8) | ivptr[2];
			key.rxiv.lo = (ivptr[1] << 8) | ivptr[0];
			key.iv_initialized = TRUE;
		}

		switch (iwe->alg) {
			case IW_ENCODE_ALG_NONE:
				key.algo = CRYPTO_ALGO_OFF;
				break;
			case IW_ENCODE_ALG_WEP:
				if (iwe->key_len == WEP1_KEY_SIZE)
					key.algo = CRYPTO_ALGO_WEP1;
				else
					key.algo = CRYPTO_ALGO_WEP128;
				break;
			case IW_ENCODE_ALG_TKIP:
				key.algo = CRYPTO_ALGO_TKIP;
				break;
			case IW_ENCODE_ALG_CCMP:
				key.algo = CRYPTO_ALGO_AES_CCM;
				break;
			default:
				break;
		}
		swap_key_from_BE(&key);

#if defined(BCMDONGLEHOST)
		dhd_wait_pend8021x(dev);
#endif 

		error = dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));
		if (error)
			return error;
	}
	return 0;
}
#if WIRELESS_EXT > 17
struct {
	pmkid_list_t pmkids;
	pmkid_t foo[MAXPMKID-1];
} pmkid_list;

static int
wl_iw_set_pmksa(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	struct iw_pmksa *iwpmksa;
	uint i;
	int ret = 0;
	char eabuf[ETHER_ADDR_STR_LEN];

	WL_WSEC(("%s: SIOCSIWPMKSA\n", dev->name));

//	RETURN_IF_EXTRA_NULL(extra);
	if( extra == NULL ) return -1;

	iwpmksa = (struct iw_pmksa *)extra;
	bzero((char *)eabuf, ETHER_ADDR_STR_LEN);

	if (iwpmksa->cmd == IW_PMKSA_FLUSH) {
		WL_WSEC(("wl_iw_set_pmksa - IW_PMKSA_FLUSH\n"));
		bzero((char *)&pmkid_list, sizeof(pmkid_list));
	}

	else if (iwpmksa->cmd == IW_PMKSA_REMOVE) {
		{
			pmkid_list_t pmkid, *pmkidptr;
			uint j;
			pmkidptr = &pmkid;

			bcopy(&iwpmksa->bssid.sa_data[0], &pmkidptr->pmkid[0].BSSID,
				ETHER_ADDR_LEN);
			bcopy(&iwpmksa->pmkid[0], &pmkidptr->pmkid[0].PMKID, WPA2_PMKID_LEN);

			WL_WSEC(("wl_iw_set_pmksa,IW_PMKSA_REMOVE - PMKID: %s = ",
				bcm_ether_ntoa(&pmkidptr->pmkid[0].BSSID,
				eabuf)));
			for (j = 0; j < WPA2_PMKID_LEN; j++)
				WL_WSEC(("%02x ", pmkidptr->pmkid[0].PMKID[j]));
			WL_WSEC(("\n"));
		}

		for (i = 0; i < pmkid_list.pmkids.npmkid; i++)
			if (!bcmp(&iwpmksa->bssid.sa_data[0], &pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN))
				break;

		if ((pmkid_list.pmkids.npmkid > 0) && (i < pmkid_list.pmkids.npmkid)) {
			bzero(&pmkid_list.pmkids.pmkid[i], sizeof(pmkid_t));
			for (; i < (pmkid_list.pmkids.npmkid - 1); i++) {
				bcopy(&pmkid_list.pmkids.pmkid[i+1].BSSID,
					&pmkid_list.pmkids.pmkid[i].BSSID,
					ETHER_ADDR_LEN);
				bcopy(&pmkid_list.pmkids.pmkid[i+1].PMKID,
					&pmkid_list.pmkids.pmkid[i].PMKID,
					WPA2_PMKID_LEN);
			}
			pmkid_list.pmkids.npmkid--;
		}
		else
			ret = -EINVAL;
	}

	else if (iwpmksa->cmd == IW_PMKSA_ADD) {
		for (i = 0; i < pmkid_list.pmkids.npmkid; i++)
			if (!bcmp(&iwpmksa->bssid.sa_data[0], &pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN))
				break;
		if (i < MAXPMKID) {
			bcopy(&iwpmksa->bssid.sa_data[0],
				&pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN);
			bcopy(&iwpmksa->pmkid[0], &pmkid_list.pmkids.pmkid[i].PMKID,
				WPA2_PMKID_LEN);
			if (i == pmkid_list.pmkids.npmkid)
				pmkid_list.pmkids.npmkid++;
		}
		else
			ret = -EINVAL;

		{
			uint j;
			uint k;
			k = pmkid_list.pmkids.npmkid;
			WL_WSEC(("wl_iw_set_pmksa,IW_PMKSA_ADD - PMKID: %s = ",
				bcm_ether_ntoa(&pmkid_list.pmkids.pmkid[k].BSSID,
				eabuf)));
			for (j = 0; j < WPA2_PMKID_LEN; j++)
				WL_WSEC(("%02x ", pmkid_list.pmkids.pmkid[k].PMKID[j]));
			WL_WSEC(("\n"));
		}
	}
	WL_WSEC(("PRINTING pmkid LIST - No of elements %d", pmkid_list.pmkids.npmkid));
	for (i = 0; i < pmkid_list.pmkids.npmkid; i++) {
		uint j;
		WL_WSEC(("\nPMKID[%d]: %s = ", i,
			bcm_ether_ntoa(&pmkid_list.pmkids.pmkid[i].BSSID,
			eabuf)));
		for (j = 0; j < WPA2_PMKID_LEN; j++)
			WL_WSEC(("%02x ", pmkid_list.pmkids.pmkid[i].PMKID[j]));
	}
	WL_WSEC(("\n"));

	if (!ret)
		ret = dev_wlc_bufvar_set(dev, "pmkid_info", (char *)&pmkid_list,
			sizeof(pmkid_list));
	return ret;
}
#endif /* WIRELESS_EXT > 17 */

static int
wl_iw_get_encodeext(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWENCODEEXT\n", dev->name));
	return 0;
}

static int
wl_iw_set_wpaauth(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error = 0;
	int paramid;
	int paramval;
	int val = 0;
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCSIWAUTH\n", dev->name));

#if defined(SOFTAP)
	if (ap_cfg_running) {
		WL_TRACE(("%s: Not executed, reason -'SOFTAP is active'\n", __FUNCTION__));
		return 0;
	}
#endif /* SOFTAP */

	paramid = vwrq->flags & IW_AUTH_INDEX;
	paramval = vwrq->value;

	switch (paramid) {
	case IW_AUTH_WPA_VERSION:
		
		if (paramval & IW_AUTH_WPA_VERSION_DISABLED)
			val = WPA_AUTH_DISABLED;
		else if (paramval & (IW_AUTH_WPA_VERSION_WPA))
			val = WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED;
#ifdef BCMWPA2
		else if (paramval & IW_AUTH_WPA_VERSION_WPA2)
			val = WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED;
#endif 
		WL_INFORM(("%s: %d: setting wpa_auth to 0x%0x\n", __FUNCTION__, __LINE__, val));
		if ((error = dev_wlc_intvar_set(dev, "wpa_auth", val)))
			return error;
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
		
		
		if (paramval & (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104))
			val = 	WEP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_TKIP)
			val = TKIP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_CCMP)
			val = AES_ENABLED;

		if (paramid == IW_AUTH_CIPHER_PAIRWISE) {
			iw->pwsec = val;
			val |= iw->gwsec;
		}
		else {
			iw->gwsec = val;
			val |= iw->pwsec;
		}

		if ((error = dev_wlc_intvar_set(dev, "wsec", val)))
			return error;

		break;

	case IW_AUTH_KEY_MGMT:
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;

		if (val & (WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED)) {
			if (paramval & IW_AUTH_KEY_MGMT_PSK)
				val = WPA_AUTH_PSK;
			else
				val = WPA_AUTH_UNSPECIFIED;
		}
#ifdef BCMWPA2
		else if (val & (WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED)) {
			if (paramval & IW_AUTH_KEY_MGMT_PSK)
				val = WPA2_AUTH_PSK;
			else
				val = WPA2_AUTH_UNSPECIFIED;
		}
#endif 
		WL_INFORM(("%s: %d: setting wpa_auth to %d\n", __FUNCTION__, __LINE__, val));
		if ((error = dev_wlc_intvar_set(dev, "wpa_auth", val)))
			return error;
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		dev_wlc_bufvar_set(dev, "tkip_countermeasures", (char *)&paramval, 1);
		break;

	case IW_AUTH_80211_AUTH_ALG:
		/* open shared */
		WL_INFORM(("Setting the D11auth %d\n", paramval));
		if (paramval == IW_AUTH_ALG_OPEN_SYSTEM)
			val = DOT11_OPEN_SYSTEM;
		else if (paramval == IW_AUTH_ALG_SHARED_KEY)
			val = DOT11_SHARED_KEY;
		else if (paramval == (IW_AUTH_ALG_OPEN_SYSTEM | IW_AUTH_ALG_SHARED_KEY))
			val = DOT11_OPEN_SHARED;
		else
			error = 1;

		if (!error && (error = dev_wlc_intvar_set(dev, "auth", val)))
			return error;

		break;

	case IW_AUTH_WPA_ENABLED:
		if (paramval == 0) {
			iw->pwsec = 0;
			iw->gwsec = 0;
			if ((error = dev_wlc_intvar_get(dev, "wsec", &val)))
				return error;
			if (val & (TKIP_ENABLED | AES_ENABLED)) {
				val &= ~(TKIP_ENABLED | AES_ENABLED);
				dev_wlc_intvar_set(dev, "wsec", val);
			}
			val = 0;
		WL_INFORM(("%s: %d: setting wpa_auth to %d\n", __FUNCTION__, __LINE__, val));
			dev_wlc_intvar_set(dev, "wpa_auth", 0);
			return error;
		}

		
		break;

	case IW_AUTH_DROP_UNENCRYPTED:
		dev_wlc_bufvar_set(dev, "wsec_restrict", (char *)&paramval, 1);
		break;

	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		dev_wlc_bufvar_set(dev, "rx_unencrypted_eapol", (char *)&paramval, 1);
		break;

#if WIRELESS_EXT > 17
	case IW_AUTH_ROAMING_CONTROL:
		WL_INFORM(("%s: IW_AUTH_ROAMING_CONTROL\n", __FUNCTION__));
		
		break;
	case IW_AUTH_PRIVACY_INVOKED: {
		// louislee : modified for WPS {
		int wsec;

		if (paramval == 0) {
			iw->privacy_invoked = FALSE;
			if ((error = dev_wlc_intvar_set(dev, "is_WPS_enrollee", FALSE))) {
				WL_WSEC(("Failed to clear iovar is_WPS_enrollee\n"));
				return error;
			}
		} else {
			iw->privacy_invoked = TRUE;
			if ((error = dev_wlc_intvar_get(dev, "wsec", &wsec)))
				return error;
			if (!(IW_WSEC_ENABLED(wsec))) {
				/* if privacy is true, but wsec is false, we are a WPS enrollee */
				if ((error = dev_wlc_intvar_set(dev, "is_WPS_enrollee", TRUE))) {
					WL_WSEC(("Failed to set iovar is_WPS_enrollee\n"));
					return error;
				}
			} else {
				if ((error = dev_wlc_intvar_set(dev, "is_WPS_enrollee", FALSE))) {
					WL_WSEC(("Failed to clear iovar is_WPS_enrollee\n"));
					return error;
				}
			}
		}
		// louislee : modified for WPS }

		break;
	}
#endif 
	default:
		break;
	}
	return 0;
}
#ifdef BCMWPA2
#define VAL_PSK(_val) (((_val) & WPA_AUTH_PSK) || ((_val) & WPA2_AUTH_PSK))
#else
#define VAL_PSK(_val) (((_val) & WPA_AUTH_PSK))
#endif 

static int
wl_iw_get_wpaauth(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error;
	int paramid;
	int paramval = 0;
	int val;
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCGIWAUTH\n", dev->name));

	paramid = vwrq->flags & IW_AUTH_INDEX;

	switch (paramid) {
	case IW_AUTH_WPA_VERSION:
		
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (val & (WPA_AUTH_NONE | WPA_AUTH_DISABLED))
			paramval = IW_AUTH_WPA_VERSION_DISABLED;
		else if (val & (WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED))
			paramval = IW_AUTH_WPA_VERSION_WPA;
#ifdef BCMWPA2
		else if (val & (WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED))
			paramval = IW_AUTH_WPA_VERSION_WPA2;
#endif 
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
		val = 0; // louislee : added
		/* none, wep40, tkip, ccmp, wep104 */
		/* wsec: none, wep40, tkip, aes, wep104 */
		if (paramval & (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104))
			val =   WEP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_TKIP)
			val = TKIP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_CCMP)
			val = AES_ENABLED;
#ifdef BCMWAPI_WPI
		if (paramval & IW_AUTH_CIPHER_SMS4)
			val = SMS4_ENABLED;
#endif
		// louislee : modified for WPS {
		if (paramid == IW_AUTH_CIPHER_PAIRWISE) {
			iw->pwsec = val;
			val |= iw->gwsec;
		} else {
			iw->gwsec = val;
			val |= iw->pwsec;
		}

		if (iw->privacy_invoked && !val) {
			WL_WSEC(("%s: %s: 'Privacy invoked' TRUE but clearing wsec, assuming "
						"we're a WPS enrollee\n", dev->name, __FUNCTION__));
			if ((error = dev_wlc_intvar_set(dev, "is_WPS_enrollee", TRUE))) {
				WL_WSEC(("Failed to set iovar is_WPS_enrollee\n"));
				return error;
		}
		} else if (val) {
			if ((error = dev_wlc_intvar_set(dev, "is_WPS_enrollee", FALSE))) {
				WL_WSEC(("Failed to clear iovar is_WPS_enrollee\n"));
				return error;
			}
		}
		
		if ((error = dev_wlc_intvar_set(dev, "wsec", val)))
			return error;
		// louislee : modified for WPS }
		
		break;
	case IW_AUTH_KEY_MGMT:
		
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (VAL_PSK(val))
			paramval = IW_AUTH_KEY_MGMT_PSK;
		else
			paramval = IW_AUTH_KEY_MGMT_802_1X;

		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		dev_wlc_bufvar_get(dev, "tkip_countermeasures", (char *)&paramval, 1);
		break;

	case IW_AUTH_DROP_UNENCRYPTED:
		dev_wlc_bufvar_get(dev, "wsec_restrict", (char *)&paramval, 1);
		break;

	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		dev_wlc_bufvar_get(dev, "rx_unencrypted_eapol", (char *)&paramval, 1);
		break;

	case IW_AUTH_80211_AUTH_ALG:
		
		if ((error = dev_wlc_intvar_get(dev, "auth", &val)))
			return error;
		if (!val)
			paramval = IW_AUTH_ALG_OPEN_SYSTEM;
		else
			paramval = IW_AUTH_ALG_SHARED_KEY;
		break;
	case IW_AUTH_WPA_ENABLED:
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (val)
			paramval = TRUE;
		else
			paramval = FALSE;
		break;
#if WIRELESS_EXT > 17
	case IW_AUTH_ROAMING_CONTROL:
		WL_ERROR(("%s: IW_AUTH_ROAMING_CONTROL\n", __FUNCTION__));
		
		break;
	case IW_AUTH_PRIVACY_INVOKED:
		WL_ERROR(("%s: IW_AUTH_PRIVACY_INVOKED\n", __FUNCTION__));
		break;
#endif 
	}
	vwrq->value = paramval;
	return 0;
}
#endif 

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-05-14, support private command */
#if defined(CONFIG_LGE_BCM432X_PATCH)
static int
wl_iw_set_powermode(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int mode = -1;
	int error;
	char *p = extra;

	if (sscanf(extra, "%*s %d", &mode) != 1)
		return -EINVAL;

	switch (mode) {
	case 0: mode = 2; break; /* Fast PS mode */
	case 1: mode = 0; break; /* No PS mode */
	default: return -EINVAL;
	}
	error = dev_wlc_ioctl(dev, WLC_SET_PM, &mode, sizeof(mode));
	p += snprintf(p, MAX_WX_STRING, error < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_set_scan_channels(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	uint32 chan_buf[WL_NUMCHANNELS + 1];
	wl_uint32_list_t *list;
	int nchan;
	char buf[WLC_CNTRY_BUF_SZ];
	int error;
	char *p = extra;

	if (sscanf(extra, "%*s %d", &nchan) != 1) {
		list = (wl_uint32_list_t *)(void *)chan_buf;
		list->count = htod32(WL_NUMCHANNELS);
		if ((error = dev_wlc_ioctl(dev, WLC_GET_VALID_CHANNELS,
				chan_buf, sizeof(chan_buf))) < 0)
			return error;
		p += snprintf(p, MAX_WX_STRING,
				"Scan-Channels = %d", dtoh32(list->count));
		wrqu->data.length = p - extra + 1;
		return error;
	}

	switch (nchan) {
	//bill.jung TX power issue. In north america we use 'XW'
	//case 11: strcpy(buf, "US"); break;
	case 11: strcpy(buf, "XW"); break;	
	case 13: strcpy(buf, "AU"); break;
	case 14: strcpy(buf, "JP"); break;
	default: return -EINVAL;
	}
	error = dev_wlc_ioctl(dev, WLC_SET_COUNTRY, buf, sizeof(buf));
	p += snprintf(p, MAX_WX_STRING, error < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_set_roam_off(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	uint roamvar;
	int error;
	char *p = extra;
	char iovbuf[WLC_IOCTL_SMLEN];	/*  Room for "event_msgs" + '\0' + bitvec  */
	int iolen;

	if (sscanf(extra, "%*s %d", &roamvar) != 1)
		return -EINVAL;

	switch (roamvar) {
		case 0: break; /* roam_off 0 */
		case 1: break; /* roam_off 1 */
		default: return -EINVAL;
	}

	iolen = bcm_mkiovar("roam_off", (char *)&roamvar, 4, iovbuf, sizeof(iovbuf));
	ASSERT(iolen);	
	error = dev_wlc_ioctl(dev, WLC_SET_VAR, iovbuf, iolen);
	
	p += snprintf(p, MAX_WX_STRING, error < 0 ? "FAIL\n" : "OK\n");
	wrqu->data.length = p - extra + 1;
	return error;
}

//PM=1, wme_apsd=1, wme_qosinfo=0x3, wme_apsd_trigger=1000, wme_auto_trigger=0
static int wl_iw_voip_start(struct net_device *dev)
{
	int error = 0;
	int val;

	//by sjark 100818
	val = 1;
	error = dev_wlc_ioctl(dev, WLC_SET_PM, &val, sizeof(val));
	if(error) WL_ERROR(("SET PM fail. error = %d",error));

	val = 20;
	error =dev_wlc_intvar_set(dev, "pspoll_prd", val);
	if(error) WL_ERROR(("SET pspoll_prd fail. error = %d",error));

#if 0
	val = 1;
	error =dev_wlc_intvar_set(dev, "wme_apsd", val);                //need wl down
	if(error) WL_ERROR(("SET wme_apsd fail. error = %d",error));

	val = 0x3;
	error =dev_wlc_intvar_set(dev, "wme_qosinfo", val);             //need wl down
	if(error) WL_ERROR(("SET wme_qosinfo fail. error = %d",error));

	val = 1000;
	error =dev_wlc_intvar_set(dev, "wme_apsd_trigger", val);
	if(error) WL_ERROR(("SET wme_apsd_trigger fail. error = %d",error));

	val = 0;
	error =dev_wlc_intvar_set(dev, "wme_auto_trigger", val);
	if(error) WL_ERROR(("SET wme_auto_trigger fail. error = %d",error));
#endif

	return error;
}

//PM=2, wme_apsd=0, wme_qosinfo=0x0, wme_apsd_trigger=0, wme_auto_trigger=1
static int wl_iw_voip_stop(struct net_device *dev)
{
	int error = 0;
	int val;

	//by sjark 100818
	val = 2;
	error = dev_wlc_ioctl(dev, WLC_SET_PM, &val, sizeof(val));
	if(error) WL_ERROR(("SET PM fail. error = %d",error));


	val = 0;
	error =dev_wlc_intvar_set(dev, "pspoll_prd", val);
	if(error) WL_ERROR(("SET pspoll_prd fail. error = %d",error));

#if 0
	val = 0;
	error =dev_wlc_intvar_set(dev, "wme_apsd", val);                //need wl down
	if(error) WL_ERROR(("SET wme_apsd fail. error = %d",error));

	val = 0x0;
	error =dev_wlc_intvar_set(dev, "wme_qosinfo", val);             //need wl down
	if(error) WL_ERROR(("SET wme_qosinfo fail. error = %d",error));

	val = 0;
	error =dev_wlc_intvar_set(dev, "wme_apsd_trigger", val);
	if(error) WL_ERROR(("SET wme_apsd_trigger fail. error = %d",error));

	val = 1;
	error =dev_wlc_intvar_set(dev, "wme_auto_trigger", val);
	if(error) WL_ERROR(("SET wme_auto_trigger fail. error = %d",error));
#endif

	return error;
}




#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-05-14, support private command */

#ifdef SOFTAP

static int ap_macmode = MACLIST_MODE_DISABLED;
static struct mflist ap_black_list;
static int
wl_iw_parse_wep(char *keystr, wl_wsec_key_t *key)
{
	char hex[] = "XX";
	unsigned char *data = key->data;

	switch (strlen(keystr)) {
	case 5:
	case 13:
	case 16:
		key->len = strlen(keystr);
		memcpy(data, keystr, key->len + 1);
		break;
	case 12:
	case 28:
	case 34:
	case 66:
		
		if (!strnicmp(keystr, "0x", 2))
			keystr += 2;
		else
			return -1;
		
	case 10:
	case 26:
	case 32:
	case 64:
		key->len = strlen(keystr) / 2;
		while (*keystr) {
			strncpy(hex, keystr, 2);
			*data++ = (char) bcm_strtoul(hex, NULL, 16);
			keystr += 2;
		}
		break;
	default:
		return -1;
	}

	switch (key->len) {
	case 5:
		key->algo = CRYPTO_ALGO_WEP1;
		break;
	case 13:
		key->algo = CRYPTO_ALGO_WEP128;
		break;
	case 16:
		
		key->algo = CRYPTO_ALGO_AES_CCM;
		break;
	case 32:
		key->algo = CRYPTO_ALGO_TKIP;
		break;
	default:
		return -1;
	}

	key->flags |= WL_PRIMARY_KEY;

	return 0;
}

#ifdef EXT_WPA_CRYPTO
#define SHA1HashSize 20
extern void pbkdf2_sha1(const char *passphrase, const char *ssid, size_t ssid_len, \
						int iterations, u8 *buf, size_t buflen);

#else

#define SHA1HashSize 20
int pbkdf2_sha1(const char *passphrase, const char *ssid, size_t ssid_len, \
						int iterations, u8 *buf, size_t buflen)
{
	WL_ERROR(("WARNING: %s is not implemented !!!\n", __FUNCTION__));
	return -1;
}
#endif 

int dev_iw_write_cfg1_bss_var(struct net_device *dev, int val)
{
	struct {
		int cfg;
		int val;
	} bss_setbuf;

	int bss_set_res;
	char smbuf[WLC_IOCTL_SMLEN];
	memset(smbuf, 0, sizeof(smbuf));

	bss_setbuf.cfg = 1;
	bss_setbuf.val = val;

	bss_set_res = dev_iw_iovar_setbuf(dev, "bss",
		&bss_setbuf, sizeof(bss_setbuf), smbuf, sizeof(smbuf));
	WL_TRACE(("%s: bss_set_result:%d set with %d\n", __FUNCTION__, bss_set_res, val));

	return bss_set_res;
}

int dev_iw_read_cfg1_bss_var(struct net_device *dev, int *val)
{
	int bsscfg_idx = 1;
	int bss_set_res;
	char smbuf[WLC_IOCTL_SMLEN];
	memset(smbuf, 0, sizeof(smbuf));

	bss_set_res = dev_iw_iovar_getbuf(dev, "bss", \
		 &bsscfg_idx, sizeof(bsscfg_idx), smbuf, sizeof(smbuf));
	*val = *(int*)smbuf;
	*val = dtoh32(*val);
	WL_TRACE(("%s: status=%d bss_get_result=%d\n", __FUNCTION__, bss_set_res, *val));
	return bss_set_res;
}

#ifndef AP_ONLY
static int wl_bssiovar_mkbuf(
			const char *iovar,
			int bssidx,
			void *param,
			int paramlen,
			void *bufptr,
			int buflen,
			int *perr)
{
	const char *prefix = "bsscfg:";
	int8* p;
	uint prefixlen;
	uint namelen;
	uint iolen;

	prefixlen = strlen(prefix);	
	namelen = strlen(iovar) + 1;	
	iolen = prefixlen + namelen + sizeof(int) + paramlen;

	if (buflen < 0 || iolen > (uint)buflen) {
		*perr = BCME_BUFTOOSHORT;
		return 0;
	}

	p = (int8*)bufptr;

	memcpy(p, prefix, prefixlen);
	p += prefixlen;

	memcpy(p, iovar, namelen);
	p += namelen;

	bssidx = htod32(bssidx);
	memcpy(p, &bssidx, sizeof(int32));
	p += sizeof(int32);

	if (paramlen)
		memcpy(p, param, paramlen);

	*perr = 0;
	return iolen;
}
#endif /* AP_ONLY */

int get_user_params(char *user_params, struct iw_point *dwrq)
{
	int ret = 0;

	if (copy_from_user(user_params, dwrq->pointer, dwrq->length)) {
		WL_ERROR(("\n%s: no user params: uptr:%p, ulen:%d\n",
			__FUNCTION__, dwrq->pointer, dwrq->length));
		return -EFAULT;
	}

	WL_TRACE(("\n%s: iwpriv user params:%s\n", __FUNCTION__, user_params));

	return ret;
}

#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))

#ifdef SOFTAP

#ifndef AP_ONLY
static int thr_wait_for_2nd_eth_dev(void *data)
{
	int ret = 0;

	DAEMONIZE("wl0_eth_wthread");

	WL_TRACE(("\n>%s threda started:, PID:%x\n", __FUNCTION__, current->pid));

	if (down_timeout(&ap_eth_sema,  msecs_to_jiffies(5000)) != 0) {
		WL_ERROR(("\n%s: sap_eth_sema timeout \n", __FUNCTION__));
		ret = -1;
		goto fail;
	}

	if (!ap_net_dev) {
		WL_ERROR((" ap_net_dev is null !!!"));
		ret = -1;
		goto fail;
	}

	WL_TRACE(("\n>%s: Thread:'softap ethdev IF:%s is detected !!!'\n\n",
		__FUNCTION__, ap_net_dev->name));

	ap_cfg_running = TRUE;

	bcm_mdelay(500); 

	wl_iw_send_priv_event(priv_dev, "AP_SET_CFG_OK");

fail:

	WL_TRACE(("\n>%s, thread completed\n", __FUNCTION__));

	return ret;
}
#endif /* AP_ONLY */

#if 1
#ifndef AP_ONLY
static int last_auto_channel = 6;
#endif /* AP_ONLY */
static int get_softap_auto_channel(struct net_device *dev, struct ap_profile *ap)
{
	int chosen = 0;
	wl_uint32_list_t request;
	int rescan = 0;
	int retry = 0;
	int updown = 0;
	int ret = 0;
	wlc_ssid_t null_ssid;
	int res = 0;
#ifndef AP_ONLY
	int iolen = 0;
	int mkvar_err = 0;
	int bsscfg_index = 1;
	char buf[WLC_IOCTL_SMLEN];
#endif
	WL_SOFTAP(("Enter %s\n", __FUNCTION__));

#ifndef AP_ONLY
	if (ap_cfg_running) {
		ap->channel = last_auto_channel;
		return res;
	}
#endif
	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	res |= dev_wlc_ioctl(dev, WLC_UP, &updown, sizeof(updown));
#ifdef AP_ONLY
	res |= dev_wlc_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid));
#else
	iolen = wl_bssiovar_mkbuf("ssid", bsscfg_index, (char *)(&null_ssid), \
		null_ssid.SSID_len+4, buf, sizeof(buf), &mkvar_err);
	ASSERT(iolen);
	res |= dev_wlc_ioctl(dev, WLC_SET_VAR, buf, iolen);
#endif
	auto_channel_retry:
			request.count = htod32(0);
			ret = dev_wlc_ioctl(dev, WLC_START_CHANNEL_SEL, &request, sizeof(request));
			if (ret < 0) {
				WL_ERROR(("can't start auto channel scan\n"));
				res = -1;
				goto fail;
			}

	get_channel_retry:
			bcm_mdelay(500);

			ret = dev_wlc_ioctl(dev, WLC_GET_CHANNEL_SEL, &chosen, sizeof(chosen));
			if (ret < 0 || dtoh32(chosen) == 0) {
				if (retry++ < 5)
					goto get_channel_retry;
				else {
					WL_ERROR(("can't get auto channel sel, err = %d, \
						chosen = %d\n", ret, chosen));
					res = -1;
					goto fail;
				}
			}
			if ((chosen == 1) && (!rescan++))
				goto auto_channel_retry;
			WL_SOFTAP(("Set auto channel = %d\n", chosen));
			ap->channel = chosen;
			if ((res = dev_wlc_ioctl(dev, WLC_DOWN, &updown, sizeof(updown))) < 0) {
				WL_ERROR(("%s fail to set up err =%d\n", __FUNCTION__, res));
				goto fail;
			}
#ifndef AP_ONLY
	if (!res)
		last_auto_channel = ap->channel;
#endif

fail :
	return res;
} 
#endif

static int set_ap_cfg(struct net_device *dev, struct ap_profile *ap)
{
	int updown = 0;
	int channel = 0;

	wlc_ssid_t ap_ssid;
	int max_assoc = 8;

	int res = 0;
	int apsta_var = 0;
#ifndef AP_ONLY
	int mpc = 0;
	int iolen = 0;
	int mkvar_err = 0;
	int bsscfg_index = 1;
	char buf[WLC_IOCTL_SMLEN];
#endif
	wl_iw_t *iw;

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -1;
	}

	WL_SOFTAP(("wl_iw: set ap profile:\n"));
	WL_SOFTAP(("	ssid = '%s'\n", ap->ssid));
	WL_SOFTAP(("	security = '%s'\n", ap->sec));
	if (ap->key[0] != '\0')
		WL_SOFTAP(("	key = '%s'\n", ap->key));
	WL_SOFTAP(("	channel = %d\n", ap->channel));
	WL_SOFTAP(("	max scb = %d\n", ap->max_scb));

	iw = *(wl_iw_t **)netdev_priv(dev);
	MUTEX_LOCK_SOFTAP_SET(iw->pub);
	//WAKE_LOCK_INIT(iw->pub, WAKE_LOCK_SOFTAP_SET, "SoftAP_SET"); // louislee
	//WAKE_LOCK(iw->pub, WAKE_LOCK_SOFTAP_SET); // louislee
	DHD_OS_WAKE_LOCK(iw->pub); // louislee

#ifdef AP_ONLY
	if (ap_cfg_running) {
		wl_iw_softap_deassoc_stations(dev);
		ap_cfg_running = FALSE;
	}
#endif	

	if (ap_cfg_running == FALSE) {
#ifndef AP_ONLY
		sema_init(&ap_eth_sema, 0);
		mpc = 0;
		if ((res = dev_wlc_intvar_set(dev, "mpc", mpc))) {
			WL_ERROR(("%s fail to set mpc\n", __FUNCTION__));
			goto fail;
		}
#endif

		updown = 0;
		if ((res = dev_wlc_ioctl(dev, WLC_DOWN, &updown, sizeof(updown)))) {
			WL_ERROR(("%s fail to set updown\n", __FUNCTION__));
			goto fail;
		}

#ifdef AP_ONLY
		apsta_var = 0;
		if ((res = dev_wlc_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var)))) {
			WL_ERROR(("%s fail to set apsta_var 0\n", __FUNCTION__));
			goto fail;
		}
		apsta_var = 1;
		if ((res = dev_wlc_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var)))) {
			WL_ERROR(("%s fail to set apsta_var 1\n", __FUNCTION__));
			goto fail;
		}
		res = dev_wlc_ioctl(dev, WLC_GET_AP, &apsta_var, sizeof(apsta_var));
#else
		apsta_var = 1;
		iolen = wl_bssiovar_mkbuf("apsta",
			bsscfg_index,  &apsta_var, sizeof(apsta_var)+4,
			buf, sizeof(buf), &mkvar_err);
		ASSERT(iolen);
		if ((res = dev_wlc_ioctl(dev, WLC_SET_VAR, buf, iolen)) < 0) {
			WL_ERROR(("%s fail to set apsta \n", __FUNCTION__));
			goto fail;
		}
		WL_TRACE(("\n>in %s: apsta set result: %d \n", __FUNCTION__, res));
#endif 

		updown = 1;
		if ((res = dev_wlc_ioctl(dev, WLC_UP, &updown, sizeof(updown))) < 0) {
			WL_ERROR(("%s fail to set apsta \n", __FUNCTION__));
			goto fail;
		}
	} else {
		if (!ap_net_dev) {
			WL_ERROR(("%s: ap_net_dev is null\n", __FUNCTION__));
			goto fail;
		}

		res = wl_iw_softap_deassoc_stations(ap_net_dev);

		if ((res = dev_iw_write_cfg1_bss_var(dev, 0)) < 0) {
			WL_ERROR(("%s fail to set bss down\n", __FUNCTION__));
			goto fail;
		}
	}

	if ((ap->channel == 0) && (get_softap_auto_channel(dev, ap) < 0)) {
		ap->channel = 1;
		WL_ERROR(("%s auto channel failed, pick up channel=%d\n", \
			__FUNCTION__, ap->channel));
	}
	channel = ap->channel;
	if ((res = dev_wlc_ioctl(dev, WLC_SET_CHANNEL, &channel, sizeof(channel)))) {
			WL_ERROR(("%s fail to set channel\n", __FUNCTION__));
			goto fail;
	}

	if (ap_cfg_running == FALSE) {
		updown = 0;
		if ((res = dev_wlc_ioctl(dev, WLC_UP, &updown, sizeof(updown)))) {
			WL_ERROR(("%s fail to set up\n", __FUNCTION__));
			goto fail;
		}
	}

	max_assoc = ap->max_scb;
	if ((res = dev_wlc_intvar_set(dev, "maxassoc", max_assoc))) {
			WL_ERROR(("%s fail to set maxassoc\n", __FUNCTION__));
			goto fail;
	}

	ap_ssid.SSID_len = strlen(ap->ssid);
	strncpy(ap_ssid.SSID, ap->ssid, ap_ssid.SSID_len);

#ifdef AP_ONLY
	if ((res = wl_iw_set_ap_security(dev, &my_ap)) != 0) {
		WL_ERROR(("ERROR:%d in:%s, wl_iw_set_ap_security is skipped\n", \
		res, __FUNCTION__));
		goto fail;
	}
	wl_iw_send_priv_event(dev, "ASCII_CMD=AP_BSS_START");
	ap_cfg_running = TRUE;
#else
	iolen = wl_bssiovar_mkbuf("ssid", bsscfg_index, (char *)(&ap_ssid),
		ap_ssid.SSID_len+4, buf, sizeof(buf), &mkvar_err);
	ASSERT(iolen);
	if ((res = dev_wlc_ioctl(dev, WLC_SET_VAR, buf, iolen)) != 0) {
		WL_ERROR(("ERROR:%d in:%s, Security & BSS reconfiguration is skipped\n", \
		res, __FUNCTION__));
		goto fail;
	}
	if (ap_cfg_running == FALSE) {
		kernel_thread(thr_wait_for_2nd_eth_dev, 0, 0);
	} else {
		if (ap_net_dev == NULL) {
			WL_ERROR(("%s ERROR: ap_net_dev is NULL !!!\n", __FUNCTION__));
			goto fail;
		}

		WL_ERROR(("%s: %s Configure security & restart AP bss \n", \
			 __FUNCTION__, ap_net_dev->name));

		if ((res = wl_iw_set_ap_security(ap_net_dev, &my_ap)) < 0) {
			WL_ERROR(("%s fail to set security : %d\n", __FUNCTION__, res));
			goto fail;
		}

		if ((res = dev_iw_write_cfg1_bss_var(dev, 1)) < 0) {
			WL_ERROR(("%s fail to set bss up\n", __FUNCTION__));
			goto fail;
		}
	}
#endif 
fail:
	DHD_OS_WAKE_UNLOCK(iw->pub);
	MUTEX_UNLOCK_SOFTAP_SET(iw->pub);
	WL_SOFTAP(("%s exit with %d\n", __FUNCTION__, res));
	return res;
}
#endif 

static int wl_iw_set_ap_security(struct net_device *dev, struct ap_profile *ap)
{
	int wsec = 0;
	int wpa_auth = 0;
	int res = 0;
	int i;
	char *ptr;
#ifdef AP_ONLY
	int mpc = 0;
	wlc_ssid_t ap_ssid;
#endif
	wl_wsec_key_t key;

	WL_SOFTAP(("\nsetting SOFTAP security mode:\n"));
	WL_SOFTAP(("wl_iw: set ap profile:\n"));
	WL_SOFTAP(("	ssid = '%s'\n", ap->ssid));
	WL_SOFTAP(("	security = '%s'\n", ap->sec));
	if (ap->key[0] != '\0')
		WL_SOFTAP(("	key = '%s'\n", ap->key));
	WL_SOFTAP(("	channel = %d\n", ap->channel));
	WL_SOFTAP(("	max scb = %d\n", ap->max_scb));

	if (strnicmp(ap->sec, "open", strlen("open")) == 0) {
		wsec = 0;
		res = dev_wlc_intvar_set(dev, "wsec", wsec);
		wpa_auth = WPA_AUTH_DISABLED;
		res |= dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);

		WL_SOFTAP(("=====================\n"));
		WL_SOFTAP((" wsec & wpa_auth set 'OPEN', result:&d %d\n", res));
		WL_SOFTAP(("=====================\n"));
	} else if (strnicmp(ap->sec, "wep", strlen("wep")) == 0) {
		memset(&key, 0, sizeof(key));

		wsec = WEP_ENABLED;
		res = dev_wlc_intvar_set(dev, "wsec", wsec);

		key.index = 0;
		if (wl_iw_parse_wep(ap->key, &key)) {
			WL_SOFTAP(("wep key parse err!\n"));
			return -1;
		}

		key.index = htod32(key.index);
		key.len = htod32(key.len);
		key.algo = htod32(key.algo);
		key.flags = htod32(key.flags);

		res |= dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));

		wpa_auth = WPA_AUTH_DISABLED;
		res |= dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);

		WL_SOFTAP(("=====================\n"));
		WL_SOFTAP((" wsec & auth set 'WEP', result:&d %d\n", res));
		WL_SOFTAP(("=====================\n"));

	} else if (strnicmp(ap->sec, "wpa2-psk", strlen("wpa2-psk")) == 0) {
		wsec_pmk_t psk;
		size_t key_len;

		wsec = AES_ENABLED;
		dev_wlc_intvar_set(dev, "wsec", wsec);

		key_len = strlen(ap->key);
		if (key_len < WSEC_MIN_PSK_LEN || key_len > WSEC_MAX_PSK_LEN) {
			WL_SOFTAP(("passphrase must be between %d and %d characters long\n",
			WSEC_MIN_PSK_LEN, WSEC_MAX_PSK_LEN));
			return -1;
		}

		if (key_len < WSEC_MAX_PSK_LEN) {
			unsigned char output[2*SHA1HashSize];
			char key_str_buf[WSEC_MAX_PSK_LEN+1];

			memset(output, 0, sizeof(output));
			pbkdf2_sha1(ap->key, ap->ssid, strlen(ap->ssid), 4096, output, 32);
			
			ptr = key_str_buf;
			for (i = 0; i < (WSEC_MAX_PSK_LEN/8); i++) {
				
				sprintf(ptr, "%02x%02x%02x%02x", (uint)output[i*4], \
					 (uint)output[i*4+1], (uint)output[i*4+2], \
					 (uint)output[i*4+3]);
				ptr += 8;
			}
			WL_SOFTAP(("%s: passphase = %s\n", __FUNCTION__, key_str_buf));

			psk.key_len = htod16((ushort)WSEC_MAX_PSK_LEN);
			memcpy(psk.key, key_str_buf, psk.key_len);
		} else {
			psk.key_len = htod16((ushort) key_len);
			memcpy(psk.key, ap->key, key_len);
		}
		psk.flags = htod16(WSEC_PASSPHRASE);
		dev_wlc_ioctl(dev, WLC_SET_WSEC_PMK, &psk, sizeof(psk));

		wpa_auth = WPA2_AUTH_PSK;
		dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);

	} else if (strnicmp(ap->sec, "wpa-psk", strlen("wpa-psk")) == 0) {
		wsec_pmk_t psk;
		size_t key_len;

		wsec = TKIP_ENABLED;
		res = dev_wlc_intvar_set(dev, "wsec", wsec);

		key_len = strlen(ap->key);
		if (key_len < WSEC_MIN_PSK_LEN || key_len > WSEC_MAX_PSK_LEN) {
			WL_SOFTAP(("passphrase must be between %d and %d characters long\n",
			WSEC_MIN_PSK_LEN, WSEC_MAX_PSK_LEN));
			return -1;
		}

		if (key_len < WSEC_MAX_PSK_LEN) {
			unsigned char output[2*SHA1HashSize];
			char key_str_buf[WSEC_MAX_PSK_LEN+1];

			WL_SOFTAP(("%s: do passhash...\n", __FUNCTION__));
			
			pbkdf2_sha1(ap->key, ap->ssid, strlen(ap->ssid), 4096, output, 32);
			
			ptr = key_str_buf;
			for (i = 0; i < (WSEC_MAX_PSK_LEN/8); i++) {
				WL_SOFTAP(("[%02d]: %08x\n", i, *((unsigned int*)&output[i*4])));
				
				sprintf(ptr, "%02x%02x%02x%02x", (uint)output[i*4], \
					(uint)output[i*4+1], (uint)output[i*4+2], \
					 (uint)output[i*4+3]);
				ptr += 8;
			}
			printk("%s: passphase = %s\n", __FUNCTION__, key_str_buf);

			psk.key_len = htod16((ushort)WSEC_MAX_PSK_LEN);
			memcpy(psk.key, key_str_buf, psk.key_len);
		} else {
			psk.key_len = htod16((ushort) key_len);
			memcpy(psk.key, ap->key, key_len);
		}

		psk.flags = htod16(WSEC_PASSPHRASE);
		res |= dev_wlc_ioctl(dev, WLC_SET_WSEC_PMK, &psk, sizeof(psk));

		wpa_auth = WPA_AUTH_PSK;
		res |= dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);

		WL_SOFTAP((" wsec & auth set 'wpa-psk' (TKIP), result:&d %d\n", res));
	}

#ifdef AP_ONLY
		ap_ssid.SSID_len = strlen(ap->ssid);
		strncpy(ap_ssid.SSID, ap->ssid, ap_ssid.SSID_len);
		res |= dev_wlc_ioctl(dev, WLC_SET_SSID, &ap_ssid, sizeof(ap_ssid));
		mpc = 0;
		res |= dev_wlc_intvar_set(dev, "mpc", mpc);
		if (strnicmp(ap->sec, "wep", strlen("wep")) == 0) {
			res |= dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));
		}
#endif
	return res;
}

int get_parmeter_from_string(
			char **str_ptr, const char *token,
			int param_type, void  *dst, int param_max_len)
{
	char int_str[7] = "0";
	int parm_str_len;
	char  *param_str_begin;
	char  *param_str_end;
	char  *orig_str = *str_ptr;

	if ((*str_ptr) && !strncmp(*str_ptr, token, strlen(token))) {

		strsep(str_ptr, "=,"); 
		param_str_begin = *str_ptr;
		strsep(str_ptr, "=,"); 

		if (*str_ptr == NULL) {
			
			parm_str_len = strlen(param_str_begin);
		} else {
			param_str_end = *str_ptr-1;  
			parm_str_len = param_str_end - param_str_begin;
		}

		WL_TRACE((" 'token:%s', len:%d, ", token, parm_str_len));

		if (parm_str_len > param_max_len) {
			WL_ERROR((" WARNING: extracted param len:%d is > MAX:%d\n",
				parm_str_len, param_max_len));

			parm_str_len = param_max_len;
		}

		switch (param_type) {

			case PTYPE_INTDEC: {
			
				int *pdst_int = dst;
				char *eptr;
				if (parm_str_len > sizeof(int_str))
					 parm_str_len = sizeof(int_str);

				memcpy(int_str, param_str_begin, parm_str_len);

				*pdst_int = simple_strtoul(int_str, &eptr, 10);

				WL_TRACE((" written as integer:%d\n",  *pdst_int));
			}
			break;
			case PTYPE_STR_HEX: {
				u8 *buf = dst;
				
				param_max_len = param_max_len >> 1;  
				hstr_2_buf(param_str_begin, buf, param_max_len);
				print_buf(buf, param_max_len, 0);
			}
			break;
			default:
				
				memcpy(dst, param_str_begin, parm_str_len);
				*((char *)dst + parm_str_len) = 0; 
				WL_ERROR((" written as a string:%s\n", (char *)dst));
			break;

		}

		return 0;
	} else {
		WL_ERROR(("\n %s: ERROR: can't find token:%s in str:%s \n",
			__FUNCTION__, token, orig_str));

	 return -1;
	}
}

static int wl_iw_softap_deassoc_stations(struct net_device *dev)
{
	int i;
	int res = 0;

	char mac_buf[128] = {0};
	struct maclist *assoc_maclist = (struct maclist *) mac_buf;

	memset(assoc_maclist, 0, sizeof(mac_buf));
	assoc_maclist->count = 8; 

	res = dev_wlc_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist, 128);
	if (res != 0) {

		WL_SOFTAP((" Error:%d in :%s, Couldn't get ASSOC List\n", res, __FUNCTION__));
		return res;
	}

		if (assoc_maclist->count)
			for (i = 0; i < assoc_maclist->count; i++) {
			scb_val_t scbval;
			scbval.val = htod32(1);
			
			bcopy(&assoc_maclist->ea[i], &scbval.ea, ETHER_ADDR_LEN);

			WL_SOFTAP(("deauth STA:%d \n", i));
			res |= dev_wlc_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON,
				&scbval, sizeof(scb_val_t));

		} else WL_SOFTAP((" STA ASSOC list is empty\n"));

	if (res != 0)
		WL_SOFTAP((" Error:%d in :%s\n", res, __FUNCTION__));
	else if (assoc_maclist->count) {
		
		bcm_mdelay(200);
	}
	return res;
}

static int iwpriv_softap_stop(struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *ext)
{
	int res = 0;
	wl_iw_t *iw;

	WL_SOFTAP(("got iwpriv AP_BSS_STOP \n"));

	if ((!dev) && (!ap_net_dev)) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return res;
	}

	iw = *(wl_iw_t **)netdev_priv(dev);
	MUTEX_LOCK_SOFTAP_SET(iw->pub);
	//WAKE_LOCK_INIT(iw->pub, WAKE_LOCK_SOFTAP_STOP, "SoftAP_STOP");
	DHD_OS_WAKE_LOCK(iw->pub);

	if  ((ap_cfg_running == TRUE)) {
#ifdef AP_ONLY
		 wl_iw_softap_deassoc_stations(dev);
#else
		 wl_iw_softap_deassoc_stations(ap_net_dev);

		if ((res = dev_iw_write_cfg1_bss_var(dev, 2)) < 0)
			WL_ERROR(("%s failed to del BSS err = %d", __FUNCTION__, res));
#endif

		bcm_mdelay(100);

		wrqu->data.length = 0;
		ap_cfg_running = FALSE;
	}
	else
		WL_ERROR(("%s: was called when SoftAP is OFF : move on\n", __FUNCTION__));

	DHD_OS_WAKE_UNLOCK(iw->pub);
	MUTEX_UNLOCK_SOFTAP_SET(iw->pub);

	WL_SOFTAP(("%s Done with %d\n", __FUNCTION__, res));
	return res;
}

static int iwpriv_fw_reload(struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *ext)
{
#if 1
	int ret = -1;
	char extra[256];
#endif
	char *fwstr = fw_path ; 

	WL_SOFTAP(("current firmware_path[]=%s\n", fwstr));
#if 1
	WL_TRACE((">Got FW_RELOAD cmd:"
				"info->cmd:%x, info->flags:%x, u.data:%p, u.len:%d, \
				fw_path:%p, len:%d \n",
				info->cmd, info->flags,
				wrqu->data.pointer, wrqu->data.length, fwstr, strlen(fwstr)));

	if ((wrqu->data.length > 4) && (wrqu->data.length < sizeof(extra))) {

		char *str_ptr;

		if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) {
			ret = -EFAULT;
			goto exit_proc;
		}

		extra[wrqu->data.length] = 8;
		str_ptr = extra;

		if (get_parmeter_from_string(&str_ptr, "FW_PATH=", PTYPE_STRING, fwstr, 255) != 0) {
			WL_ERROR(("Error: extracting FW_PATH='' string\n"));
			goto exit_proc;
		}

		if  (strstr(fwstr, "apsta") != NULL) {
			  WL_SOFTAP(("GOT APSTA FIRMWARE\n"));
			  ap_fw_loaded = TRUE;
		} else {
			WL_SOFTAP(("GOT STA FIRMWARE\n"));
			ap_fw_loaded = FALSE;
		}

		WL_SOFTAP(("SET firmware_path[]=%s , str_p:%p\n", fwstr, fwstr));
		ret = 0;
	} else {
		WL_ERROR(("Error: ivalid param len:%d\n", wrqu->data.length));
	}
exit_proc:
	return ret;

#else
	/* We used apsta firmware. */
	ap_fw_loaded = TRUE;
	return 0;
#endif
}

#ifdef SOFTAP

static int iwpriv_wpasupp_loop_tst(struct net_device *dev,
            struct iw_request_info *info,
            union iwreq_data *wrqu,
            char *ext)
{
	int res = 0;
	char  *params = NULL;

	WL_TRACE((">Got IWPRIV  wp_supp loopback cmd test:"
				"info->cmd:%x, info->flags:%x, u.data:%p, u.len:%d\n",
				info->cmd, info->flags,
				wrqu->data.pointer, wrqu->data.length));

	if (wrqu->data.length != 0) {

		if (!(params = kmalloc(wrqu->data.length+1, GFP_KERNEL)))
			return -ENOMEM;

		if (copy_from_user(params, wrqu->data.pointer, wrqu->data.length)) {
			kfree(params);
			return -EFAULT;
		}

		params[wrqu->data.length] = 0;
		WL_SOFTAP(("\n>> copied from user:\n %s\n", params));
	} else {
		WL_ERROR(("ERROR param length is 0\n"));
		return -EFAULT;
	}

	res = wl_iw_send_priv_event(dev, params);
	kfree(params);

	return res;
}
#endif 

static int
	iwpriv_en_ap_bss(
		struct net_device *dev,
		struct iw_request_info *info,
		void *wrqu,
		char *extra)
{
	int res = 0;
	wl_iw_t *iw;

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -1;
	}

	WL_TRACE(("%s: rcvd IWPRIV IOCTL:  for dev:%s\n", __FUNCTION__, dev->name));

	iw = *(wl_iw_t **)netdev_priv(dev);
	MUTEX_LOCK_SOFTAP_SET(iw->pub);
	//WAKE_LOCK_INIT(iw->pub, WAKE_LOCK_SOFTAP_START, "SoftAP_START");
	DHD_OS_WAKE_LOCK(iw->pub);

#ifndef AP_ONLY
	if ((res = wl_iw_set_ap_security(dev, &my_ap)) != 0) {
		WL_ERROR((" %s ERROR setting SOFTAP security in :%d\n", __FUNCTION__, res));
	}
	else {
		if ((res = dev_iw_write_cfg1_bss_var(dev, 1)) < 0)
			WL_ERROR(("%s fail to set bss up err=%d\n", __FUNCTION__, res));
		else
			bcm_mdelay(100);
	}
#endif 

	WL_SOFTAP(("%s done with res %d \n", __FUNCTION__, res));
	DHD_OS_WAKE_UNLOCK(iw->pub);
	//WAKE_LOCK_DESTROY(iw->pub, WAKE_LOCK_SOFTAP_START);
	MUTEX_UNLOCK_SOFTAP_SET(iw->pub);
	return res;
}

static int
get_assoc_sta_list(struct net_device *dev, char *buf, int len)
{
	WL_TRACE(("calling dev_wlc_ioctl(dev:%p, cmd:%d, buf:%p, len:%d)\n",
		dev, WLC_GET_ASSOCLIST, buf, len));
	dev_wlc_ioctl(dev, WLC_GET_ASSOCLIST, buf, len);
	return 0;
}

static int
set_ap_mac_list(struct net_device *dev, char *buf)
{
	struct mac_list_set *mac_list_set = (struct mac_list_set *)buf;
	struct maclist *white_maclist = (struct maclist *)&mac_list_set->white_list;
	struct maclist *black_maclist = (struct maclist *)&mac_list_set->black_list;
	int mac_mode = mac_list_set->mode;
	int length;
	int i;

	ap_macmode = mac_mode;
	if (mac_mode == MACLIST_MODE_DISABLED) {
		bzero(&ap_black_list, sizeof(struct mflist));
		dev_wlc_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode));
	} else {
		scb_val_t scbval;
		char mac_buf[256] = {0};
		struct maclist *assoc_maclist = (struct maclist *) mac_buf;

		mac_mode = MACLIST_MODE_ALLOW;
		
		dev_wlc_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode));

		length = sizeof(white_maclist->count)+white_maclist->count*ETHER_ADDR_LEN;
		dev_wlc_ioctl(dev, WLC_SET_MACLIST, white_maclist, length);
		WL_SOFTAP(("White List, length %d:\n", length));
		for (i = 0; i < white_maclist->count; i++)
			WL_SOFTAP(("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, white_maclist->ea[i].octet[0], white_maclist->ea[i].octet[1], \
				white_maclist->ea[i].octet[2], \
				white_maclist->ea[i].octet[3], white_maclist->ea[i].octet[4], \
				white_maclist->ea[i].octet[5]));

		bcopy(black_maclist, &ap_black_list, sizeof(ap_black_list));

		WL_SOFTAP(("Black List, size %d:\n", sizeof(ap_black_list)));
		for (i = 0; i < ap_black_list.count; i++)
			WL_SOFTAP(("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, ap_black_list.ea[i].octet[0], ap_black_list.ea[i].octet[1], \
				ap_black_list.ea[i].octet[2], \
				ap_black_list.ea[i].octet[3], \
				ap_black_list.ea[i].octet[4], ap_black_list.ea[i].octet[5]));

		dev_wlc_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist, 256);
		if (assoc_maclist->count) {
			int j;
			for (i = 0; i < assoc_maclist->count; i++) {
				for (j = 0; j < white_maclist->count; j++) {
					if (!bcmp(&assoc_maclist->ea[i], &white_maclist->ea[j], \
						ETHER_ADDR_LEN)) {
						WL_SOFTAP(("match allow, let it be\n"));
						break;
					}
				}
				if (j == white_maclist->count) {
						WL_SOFTAP(("match black, deauth it\n"));
						scbval.val = htod32(1);
						bcopy(&assoc_maclist->ea[i], &scbval.ea, \
						ETHER_ADDR_LEN);
						dev_wlc_ioctl(dev, \
							WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
							sizeof(scb_val_t));
				}
			}
		}
	}
	return 0;
}
#endif 

#ifdef SOFTAP
int set_macfilt_from_string(struct mflist *pmflist, char **param_str)
{
	return 0;
}
#endif

#ifdef SOFTAP
#define PARAM_OFFSET PROFILE_OFFSET
int wl_iw_process_private_ascii_cmd(
			struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *dwrq,
			char *cmd_str)
{
	int ret = 0;
	char *sub_cmd = cmd_str + PROFILE_OFFSET + strlen("ASCII_CMD=");

	WL_SOFTAP(("\n %s: ASCII_CMD: offs_0:%s, offset_32:\n'%s'\n",
		__FUNCTION__, cmd_str, cmd_str + PROFILE_OFFSET));

	if (strnicmp(sub_cmd, "AP_CFG", strlen("AP_CFG")) == 0) {

		WL_SOFTAP((" AP_CFG \n"));

		if (init_ap_profile_from_string(cmd_str+PROFILE_OFFSET, &my_ap) != 0) {
				WL_ERROR(("ERROR: SoftAP CFG prams !\n"));
				ret = -1;
		} else {
			ret = set_ap_cfg(dev, &my_ap);
		}
	} else if (strnicmp(sub_cmd, "AP_BSS_START", strlen("AP_BSS_START")) == 0) {

		WL_SOFTAP(("\n SOFTAP - ENABLE BSS \n"));
		WL_SOFTAP(("\n!!! got 'WL_AP_EN_BSS' from WPA supplicant, dev:%s\n", dev->name));

#ifndef AP_ONLY
		if (ap_net_dev == NULL) {
				 printf("\n ERROR: SOFTAP net_dev* is NULL !!!\n");
		} else {
			if ((ret = iwpriv_en_ap_bss(ap_net_dev, info, dwrq, cmd_str)) < 0) {
				WL_ERROR(("%s line %d fail to set bss up\n", \
					__FUNCTION__, __LINE__));
			}
		}
#else
		if ((ret = iwpriv_en_ap_bss(dev, info, dwrq, cmd_str)) < 0) {
				WL_ERROR(("%s line %d fail to set bss up\n", \
					__FUNCTION__, __LINE__));
		}
#endif
	} else if (strnicmp(sub_cmd, "ASSOC_LST", strlen("ASSOC_LST")) == 0) {

	} else if (strnicmp(sub_cmd, "AP_BSS_STOP", strlen("AP_BSS_STOP")) == 0) {
		WL_SOFTAP((" \n temp DOWN SOFTAP\n"));
#ifndef AP_ONLY
			if ((ret = dev_iw_write_cfg1_bss_var(dev, 0)) < 0) {
				WL_ERROR(("%s line %d fail to set bss down\n", \
					__FUNCTION__, __LINE__));
			}
#endif
	}

	return ret;
}
#endif /* SOFTAP */
/* LGE_UPDATE_S 20110708 BRCM patch */
#ifdef LGE_ROAM_PARAMETER
static int
wl_iw_process_private_roam_cmd(
			struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *dwrq,
			char *extra)
{
	int ret = -1;
    unsigned int value = 0;
    unsigned int iocmd = 0;
	char *sub_cmd = extra + strlen("ROAM_CMD=");

	WL_TRACE(("\n%s: ROAM_CMD: offs_0:%s, offset_32:\n'%s'\n",
		__FUNCTION__, extra, extra + strlen("ROAM_CMD=")));

	if( strnicmp(sub_cmd, "GET_ROAM_TRIGGER", strlen("GET_ROAM_TRIGGER")) == 0 ) 
        iocmd = WLC_GET_ROAM_TRIGGER;
    else if( strnicmp(sub_cmd, "SET_ROAM_TRIGGER", strlen("SET_ROAM_TRIGGER")) == 0 )
        iocmd = WLC_SET_ROAM_TRIGGER;
    else if( strnicmp(sub_cmd, "GET_ROAM_DELTA", strlen("GET_ROAM_DELTA")) == 0 )
        iocmd = WLC_GET_ROAM_DELTA;
    else if( strnicmp(sub_cmd, "SET_ROAM_DELTA", strlen("SET_ROAM_DELTA")) == 0 )
        iocmd = WLC_SET_ROAM_DELTA;
    else if( strnicmp(sub_cmd, "GET_ROAM_SCAN_PERIOD", strlen("GET_ROAM_SCAN_PERIOD")) == 0 ) 
        iocmd = WLC_GET_ROAM_SCAN_PERIOD;
    else if( strnicmp(sub_cmd, "SET_ROAM_SCAN_PERIOD", strlen("SET_ROAM_SCAN_PERIOD")) == 0 ) 
        iocmd = WLC_SET_ROAM_SCAN_PERIOD;
    else
    {
        WL_ERROR(("%s: command parsing error %s\n",__FUNCTION__, extra));
        return ret;
    }

    if( strnicmp( sub_cmd, "GET", strlen("GET") ) == 0 )
    {
        char* p = extra;
		WL_TRACE(("%s: iocmd = %d\n", __FUNCTION__, iocmd));

        if ((ret = dev_wlc_ioctl(dev, iocmd, &value, sizeof(value) )))
        {
            WL_ERROR(("%s: dev_wlc_ioctl returned error %d\n",__FUNCTION__, ret));
            WL_TRACE(("%d, %d\n", value, dtoh32(value)));
            return ret;
        }

		WL_TRACE(("%d\n", dtoh32(value)));

        p += snprintf(p, MAX_WX_STRING, "%d", dtoh32(value) );
        dwrq->data.length = p - extra + 1;
    }
    else if( strnicmp( sub_cmd, "SET", strlen("SET") ) == 0 )
    {
		WL_TRACE(("%s: %s ", __FUNCTION__, sub_cmd));
        if( sscanf(sub_cmd, "%*s %d", &value) != 1 ) 
        {
            WL_ERROR(("%s: sscanf parsing error %s\n",__FUNCTION__, sub_cmd));
            return -1;
        }

        if ((ret = dev_wlc_ioctl(dev, iocmd, &value, sizeof(value) )))
        {
            WL_ERROR(("%s: dev_wlc_ioctl returned error %d, value = %d\n",__FUNCTION__, ret, value ));
            return ret;
        }

		WL_TRACE(("%d has been set\n", value));
    }

	return ret;
}
#endif //LGE_ROAM_PARAMETER
/* LGE_UPDATE_E 20110708 BRCM patch */


static s32
wl_iw_vndr_ie(struct net_device *ndev, s32 pktflag,
            s8 *oui, s32 ie_id, s8 *data, s32 data_len, s32 delete)
{
	s32 err = BCME_OK;
	s32 buf_len;
	s32 iecount;

	vndr_ie_setbuf_t *ie_setbuf;
	char iovbuf[512];

	memset(iovbuf, 0, sizeof(iovbuf));

	/* Validate the pktflag parameter */
	if ((pktflag & ~(VNDR_IE_BEACON_FLAG | VNDR_IE_PRBRSP_FLAG |
	            VNDR_IE_ASSOCRSP_FLAG | VNDR_IE_AUTHRSP_FLAG |
	            VNDR_IE_PRBREQ_FLAG | VNDR_IE_ASSOCREQ_FLAG))) {
		WL_ERROR(("p2pwl_vndr_ie: Invalid packet flag 0x%x\n", pktflag));
		return -1;
	}

	buf_len = sizeof(vndr_ie_setbuf_t) + data_len - 1;
	ie_setbuf = (vndr_ie_setbuf_t *) kzalloc(buf_len, GFP_KERNEL);

	WL_ERROR((" ie_id : %02x, data length : %d\n", ie_id, data_len));
	if (!ie_setbuf) {

		WL_ERROR(("Error allocating buffer for IE\n"));
		return -ENOMEM;
	}
	if (delete)
		strcpy(ie_setbuf->cmd, "del");
	else
		strcpy(ie_setbuf->cmd, "add");
	/* Buffer contains only 1 IE */
	iecount = htod32(1);
	memcpy((void *)&ie_setbuf->vndr_ie_buffer.iecount, &iecount, sizeof(int));
	pktflag = htod32(pktflag);
	memcpy((void *)&ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].pktflag,
	    &pktflag, sizeof(uint32));
	ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.id = ie_id;
	ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.len
	        = (uchar)(data_len + VNDR_IE_MIN_LEN);
	memcpy(ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.oui, oui, 3);
	memcpy(ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.data, data, data_len);
	err = dev_iw_iovar_setbuf(ndev, "vndr_ie", ie_setbuf, buf_len,iovbuf, 512);
	WL_ERROR(("vndr_ie iovar returns %d\n", err));
	kfree(ie_setbuf);
	return err;
}

static bool
wl_iw_has_ie(u8 *ie, u8 **tlvs, u32 *tlvs_len, const u8 *oui, u32 oui_len, u8 type)
{
    /* If the contents match the OUI and the type */
    if (ie[TLV_LEN_OFF] >= oui_len + 1 &&
            !bcmp(&ie[TLV_BODY_OFF], oui, oui_len) &&
            type == ie[TLV_BODY_OFF + oui_len]) {
        return TRUE;
    }

    if (tlvs == NULL)
        return FALSE;
    /* point to the next ie */
    ie += ie[TLV_LEN_OFF] + TLV_HDR_LEN;
    /* calculate the length of the rest of the buffer */
    *tlvs_len -= (int)(ie - *tlvs);
    /* update the pointer to the start of the buffer */
    *tlvs = ie;

    return FALSE;
}


/* Check whether pointed-to IE looks like WPA. */
#define wl_iw_is_wpa_ie(ie, tlvs, len)  wl_iw_has_ie(ie, tlvs, len, \
		(const uint8 *)WPS_OUI, WPS_OUI_LEN, WPA_OUI_TYPE)

/* Check whether pointed-to IE looks like WPS. */
#define wl_iw_is_wps_ie(ie, tlvs, len)  wl_iw_has_ie(ie, tlvs, len,      \
		(const uint8 *)WPS_OUI, WPS_OUI_LEN, WPS_OUI_TYPE)
/* Check whether the given IE looks like WFA P2P IE. */
#define wl_iw_is_p2p_ie(ie, tlvs, len)  wl_iw_has_ie(ie, tlvs, len, \
		(const uint8 *)WFA_OUI, WFA_OUI_LEN, WFA_OUI_TYPE_P2P)
/* Check whether the given IE looks like WFA WFDisplay IE. */
#define WFA_OUI_TYPE_WFD    0x0a            /* WiFi Display OUI TYPE */
#define wl_iw_is_wfd_ie(ie, tlvs, len)  wl_iw_has_ie(ie, tlvs, len, \
		(const uint8 *)WFA_OUI, WFA_OUI_LEN, WFA_OUI_TYPE_WFD)


int wl_iw_add_vndr_ie(
			struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu,
			char *extra)
{

#define VNDR_SPEC_ELEMENT_ID 0xdd

	int err=0;
	u32 pos;
	u8 * ie_buf;
	u32 ie_id, ie_len, data_len; 
	u8  vndr_ie[512] = {0,};
	u8 delete;
	
	
	WL_TRACE(("wl_iw_add_vndr_ie info->cmd:%x, info->flags:%x, u.data:%p, u.len:%d\n",
		info->cmd, info->flags,
		wrqu->data.pointer, wrqu->data.length));

	data_len = wrqu->data.length - strlen("add_vndrie");

	/* we assume that extra_ie come from wpa_supplicant by this private cmd is always wps_ie. */
	
	if (data_len != 0) {
		
		memcpy(vndr_ie, extra+strlen("add_vndrie")+1,data_len);
		pos = 0;		
		ie_buf = (u8 *) vndr_ie;
		ie_id = ie_buf[pos++];
		ie_len = ie_buf[pos++];

		if( wl_iw_is_wps_ie(ie_buf, NULL, 0) == FALSE )
		{
			WL_ERROR(("%s: no wps_ie. return\n", __FUNCTION__ ));
			return 0;
		}

		/* delete wps_ie */
		delete = 1;
		if (ie_id == DOT11_MNG_VS_ID) {
			WL_TRACE(("WPS:  DELETE_IE : %d, Len : %d , OUI :"
					"%02x:%02x:%02x\n", ie_id, ie_len, ie_buf[pos],
					ie_buf[pos+1], ie_buf[pos+2]));
			err = wl_iw_vndr_ie(dev,VNDR_IE_PRBREQ_FLAG|VNDR_IE_ASSOCREQ_FLAG,
				ie_buf+pos, VNDR_SPEC_ELEMENT_ID, ie_buf+pos+3,ie_len-3, delete);
			WL_ERROR(("WPS:  wl_iw_vndr_ie delete err=%d\n",err));
		}
		
		pos =0;
		ie_buf = (u8 *) vndr_ie;
		ie_id = ie_buf[pos++];
		ie_len = ie_buf[pos++];
		/* add wps_ie */
		delete = 0;
		WL_TRACE(("WPS:  ADD_IE : %d, Len : %d , OUI :"
				"%02x:%02x:%02x\n", ie_id, ie_len, ie_buf[pos],
				ie_buf[pos+1], ie_buf[pos+2]));
		err = wl_iw_vndr_ie(dev,VNDR_IE_PRBREQ_FLAG | VNDR_IE_ASSOCREQ_FLAG ,ie_buf+pos, VNDR_SPEC_ELEMENT_ID, ie_buf+pos+3,ie_len-3, delete);
		WL_ERROR(("WPS:  wl_iw_vndr_ie add err=%d\n",err));
	}
	
	return err;
}

static int
wl_iw_set_priv(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *ext
)
{
	int ret = 0;
	char * extra;
/* LGE_CHANGE_S, [dongp.kim@lge.com] 2010-04-02 */
/* [WLAN] Fixing wl_iw_set_priv function */
#if defined(CONFIG_LGE_BCM432X_PATCH)
	wl_iw_t *iw;

	if (!dev) {
		printk("%s: dev is null\n", __FUNCTION__);
		return -EFAULT;
	}
	iw = *(wl_iw_t **)netdev_priv(dev);
#else
#if defined(BCMDONGLEHOST)
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);
#endif /* defined(BCMDONGLEHOST) */
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) */
/* LGE_CHANGE_E, [dongp.kim@lge.com] 2010-04-02 */

	if (!(extra = kmalloc(dwrq->length, GFP_KERNEL)))
	    return -ENOMEM;

	if (copy_from_user(extra, dwrq->pointer, dwrq->length)) {
	    kfree(extra);
	    return -EFAULT;
	}

	WL_TRACE(("%s: SIOCSIWPRIV requst = %s\n",
		dev->name, extra));

	
	if (dwrq->length && extra) {
#if defined(BCMDONGLEHOST)
		//WAKE_LOCK_INIT(iw->pub, WAKE_LOCK_PRIV, "wl_iw_set_priv");
		DHD_OS_WAKE_LOCK(iw->pub);
#endif 

/* LGE_CHANGE_S [yoohoo@lge.com] 2009-07-09, send wl_iw_send_priv_event  */
/* only if receiving regular START command */
#if defined(CONFIG_LGE_BCM432X_PATCH)
		if (g_onoff == G_WLAN_SET_OFF) {
			if (strnicmp(extra, "START", strlen("START")) != 0) {
				WL_ERROR(("%s First IOCTL after stop is NOT START \n",
				__FUNCTION__));
				DHD_OS_WAKE_UNLOCK(iw->pub);
				//WAKE_LOCK_DESTROY(iw->pub, WAKE_LOCK_PRIV);
				kfree(extra);
				return -EFAULT;
			} else {
				wl_iw_control_wl_on(dev, info);
				WL_TRACE(("%s, Received regular START command\n", __FUNCTION__));
			}

		} else if(strnicmp(extra, "START", strlen("START")) == 0) {
			/* if g_onoff is G_WLAN_SET_ON, then notify wl_iw_send_priv_event unconditionly */	
			wl_iw_send_priv_event(dev, "START");
			WL_TRACE(("wl_iw_send_priv_event response to START PRIVATE command\n"));
		}
#else /* CONFIG_LGE_BCM432X_PATCH */
		if (g_onoff == G_WLAN_SET_OFF) {
			if (strnicmp(extra, "START", strlen("START")) != 0) {
				WL_ERROR(("%s First IOCTL after stop is NOT START \n",
				__FUNCTION__));
#if defined(BCMDONGLEHOST)
				DHD_OS_WAKE_UNLOCK(iw->pub);
				//WAKE_LOCK_DESTROY(iw->pub, WAKE_LOCK_PRIV);
#endif
				kfree(extra);
				return -EFAULT;
			} else {
				wl_iw_control_wl_on(dev, info);
				WL_TRACE(("%s, Received regular START command\n", __FUNCTION__));
			}
		}
#endif /* CONFIG_LGE_BCM432X_PATCH */
		/* LGE_CHANGE_E [yoohoo@lge.com] 2009-07-09, send wl_iw_send_priv_event only if receiving regular START command */		

	    if (strnicmp(extra, "SCAN-ACTIVE", strlen("SCAN-ACTIVE")) == 0) {
#ifdef ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS
			WL_TRACE(("%s: active scan setting supppressed\n", dev->name));
#else
			ret = wl_iw_set_active_scan(dev, info, (union iwreq_data *)dwrq, extra);
#endif 
	    }
	    else if (strnicmp(extra, "SCAN-PASSIVE", strlen("SCAN-PASSIVE")) == 0)
#ifdef ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS
			WL_TRACE(("%s: passive scan setting supppressed\n", dev->name));
#else
			ret = wl_iw_set_passive_scan(dev, info, (union iwreq_data *)dwrq, extra);
#endif 
	    else if (strnicmp(extra, "RSSI", strlen("RSSI")) == 0)
			ret = wl_iw_get_rssi(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "LINKSPEED", strlen("LINKSPEED")) == 0)
			ret = wl_iw_get_link_speed(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "MACADDR", strlen("MACADDR")) == 0)
			ret = wl_iw_get_macaddr(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "COUNTRY", strlen("COUNTRY")) == 0)
			ret = wl_iw_set_country(dev, info, (union iwreq_data *)dwrq, extra);
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP) && defined(CONFIG_BRCM_LGE_WL_ARPOFFLOAD)
	    else if (strnicmp(extra, "IPADDR", strlen("IPADDR")) == 0)
			ret = wl_iw_set_hostip(dev, info, (union iwreq_data *)dwrq, extra);
#endif	/* defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP) && defined(CONFIG_BRCM_LGE_WL_ARPOFFLOAD) */
	    else if (strnicmp(extra, "STOP", strlen("STOP")) == 0)
			ret = wl_iw_control_wl_off(dev, info);
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	    else if (strnicmp(extra, "POWERMODE", strlen("POWERMODE")) == 0)
			ret = wl_iw_set_btcoex_dhcp(dev, info, (union iwreq_data *)dwrq, extra);
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) */
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-05-14, support private command */
#if defined(CONFIG_LGE_BCM432X_PATCH)
		else if (strnicmp(extra, "POWERMODE", 9) == 0)
			ret = wl_iw_set_powermode(dev, info,
					(union iwreq_data *)dwrq, extra);
		else if (strnicmp(extra, "SCAN-CHANNELS", 13) == 0)
			ret = wl_iw_set_scan_channels(dev, info,
					(union iwreq_data *)dwrq, extra);
		else if (strnicmp(extra, "ROAM-OFF", 8) == 0)
			ret = wl_iw_set_roam_off(dev, info,
					(union iwreq_data *)dwrq, extra);		
		else if (strnicmp(extra, "DEV-STATS", 9) == 0) { /* 2009-09-29 get TX/RX bytes */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
			struct net_device_stats *stats = dev->get_stats(dev);
#else
			struct net_device_stats *stats = dev->netdev_ops->ndo_get_stats(dev);
#endif
			//printk(KERN_ERR "%6s: TX:%8lu B(%7lu Pkts) RX:%8lu B(%7lu Pkts)\n",
			//	dev->name, stats->tx_bytes, stats->tx_packets,
			//	stats->rx_bytes, stats->rx_packets);
			if(stats) {
#if 1				
				sprintf(extra, "stats %lu %lu", stats->tx_bytes, stats->rx_bytes); //tx_bytes, rx_bytes
				//printk(KERN_ERR "DEV-STATS extra: %s\n", extra);
				dwrq->length = strlen(extra) + 1;
#else /* include "OK" string */
				snprintf(extra, MAX_WX_STRING, "OK");
				memcpy(extra + strlen("OK"), (char *)&(stats->tx_bytes), sizeof(unsigned long)); //tx_bytes
				memcpy(extra + strlen("OK") + sizeof(unsigned long), (char *)&(stats->rx_bytes), sizeof(unsigned long)); //rx_bytes
				printk(KERN_ERR "extra[%d]:%02X, extra[%d]:%02X, extra[%d]:%02X, extra[%d]:%02X\n",
					strlen("OK"), extra[strlen("OK")], strlen("OK")+1, extra[strlen("OK")+1],
					strlen("OK")+2, extra[strlen("OK")+2], strlen("OK")+3, extra[strlen("OK")+3]);
				printk(KERN_ERR "extra[%d]:%02X, extra[%d]:%02X, extra[%d]:%02X, extra[%d]:%02X\n",
					strlen("OK")+sizeof(unsigned long), extra[strlen("OK")+sizeof(unsigned long)], 
					strlen("OK")+sizeof(unsigned long)+1, extra[strlen("OK")+sizeof(unsigned long)+1],
					strlen("OK")+sizeof(unsigned long)+2, extra[strlen("OK")+sizeof(unsigned long)+2],
					strlen("OK")+sizeof(unsigned long)+3, extra[strlen("OK")+sizeof(unsigned long)+3]);
				extra[strlen("OK") + sizeof(unsigned long)*2] = '\0';
				dwrq->length = strlen("OK") + sizeof(unsigned long)*2 + 1;
#endif				
			} else {
				snprintf(extra, MAX_WX_STRING, "OK");
				dwrq->length = strlen("OK") + 1;
			}
		}		
		else if (strnicmp(extra, "VOIP-START", 10) == 0) {
			wl_iw_voip_start(dev);
		}else if (strnicmp(extra, "VOIP-STOP", 9) == 0) {
			wl_iw_voip_stop(dev);
		}
		/*
	    else if (strnicmp(extra, "SCAN_UNASSOC_TIME_MS", strlen("SCAN_UNASSOC_TIME_MS")) == 0)
    	{
			ret = wl_iw_set_scan_unassoc_time_ms(dev, info, (union iwreq_data *)dwrq, extra);
		}
		*/
		/*
		else if (strnicmp(extra, "BTCOEXSCAN-START", strlen("BTCOEXSCAN-START")) == 0)
		{
			ret = wl_iw_set_scan_unassoc_time_30_ms(dev, info, (union iwreq_data *)dwrq, extra);
		}
		
		else if (strnicmp(extra, "BTCOEXSCAN-STOP", strlen("BTCOEXSCAN-STOP")) == 0)
		{
			ret = wl_iw_set_scan_unassoc_time_80_ms(dev, info, (union iwreq_data *)dwrq, extra);
		}
		*/
		else if (strnicmp(extra, "KEEP_ALIVE", strlen("KEEP_ALIVE")) == 0)
		{
			ret = wl_keep_alive_set(dev, extra);
		}
	else if (strnicmp(extra, "PM2_SLEEP_RET", strlen("PM2_SLEEP_RET")) == 0)
	{
	//	WL_ERROR("%s PM2_SLEEP_RET",__FUNCTION__);
		ret = wl_iw_set_pm2_sleep_ret(dev, info, (union iwreq_data *)dwrq, extra);
	}
		
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-05-14, support private command */
#ifdef SOFTAP
	    else if (strnicmp(extra, "ASCII_CMD", strlen("ASCII_CMD")) == 0) {
		    wl_iw_process_private_ascii_cmd(dev, info, (union iwreq_data *)dwrq, extra);
	    }
		else if (strnicmp(extra, "AP_MAC_LIST_SET", strlen("AP_MAC_LIST_SET")) == 0) {
			WL_SOFTAP(("penguin, set AP_MAC_LIST_SET\n"));
			set_ap_mac_list(dev, (extra + PROFILE_OFFSET));
	    }
#endif /* SOFTAP */
/* LGE_UPDATE_S 20110708 BRCM patch */
#ifdef LGE_ROAM_PARAMETER
        else if( strnicmp( extra, "ROAM_CMD", strlen("ROAM_CMD") ) == 0 )
            ret = wl_iw_process_private_roam_cmd( dev, info, (union iwreq_data *)dwrq, extra );
#endif //LGE_ROAM_PARAMETER
	    else if (strnicmp(extra, "add_vndrie", strlen("add_vndrie")) == 0) {
			ret = wl_iw_add_vndr_ie(dev, info, (union iwreq_data *)dwrq, extra);
	    }
	    else {
			snprintf(extra, MAX_WX_STRING, "OK");
			dwrq->length = strlen("OK") + 1;
			WL_TRACE(("Unkown PRIVATE command , ignored\n"));
		}
#if defined(BCMDONGLEHOST)
		DHD_OS_WAKE_UNLOCK(iw->pub);
		//WAKE_LOCK_DESTROY(iw->pub, WAKE_LOCK_PRIV);
#endif 
	}

	if (extra) {
	    if (copy_to_user(dwrq->pointer, extra, dwrq->length)) {
			kfree(extra);
			return -EFAULT;
	    }

	    kfree(extra);
	}

	return ret;
}

static const iw_handler wl_iw_handler[] =
{
	(iw_handler) wl_iw_config_commit,	
	(iw_handler) wl_iw_get_name,		
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_set_freq,		
	(iw_handler) wl_iw_get_freq,		
	(iw_handler) wl_iw_set_mode,		
	(iw_handler) wl_iw_get_mode,		
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_get_range,		
	(iw_handler) wl_iw_set_priv,		
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_set_spy,		
	(iw_handler) wl_iw_get_spy,		
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_set_wap,		
	(iw_handler) wl_iw_get_wap,		
#if WIRELESS_EXT > 17
	(iw_handler) wl_iw_mlme,		
#else
	(iw_handler) NULL,			
#endif
#if defined(WL_IW_USE_ISCAN) || defined(WL_IW_USE_ESCAN)
	(iw_handler) wl_iw_scan_get_aplist,	
#else
	(iw_handler) wl_iw_get_aplist,		
#endif 
#if WIRELESS_EXT > 13
#if defined(WL_IW_USE_ISCAN)
	(iw_handler) wl_iw_iscan_set_scan,	
	(iw_handler) wl_iw_scan_get_scan,	
#elif defined(WL_IW_USE_ESCAN)
	(iw_handler) wl_iw_escan_set_scan,	
	(iw_handler) wl_iw_scan_get_scan,	
#else
	(iw_handler) wl_iw_set_scan,		
	(iw_handler) wl_iw_get_scan,		
#endif
#else	
	(iw_handler) NULL,			
	(iw_handler) NULL,			
#endif	
	(iw_handler) wl_iw_set_essid,		
	(iw_handler) wl_iw_get_essid,		
	(iw_handler) wl_iw_set_nick,		
	(iw_handler) wl_iw_get_nick,		
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_set_rate,		
	(iw_handler) wl_iw_get_rate,		
	(iw_handler) wl_iw_set_rts,		
	(iw_handler) wl_iw_get_rts,		
	(iw_handler) wl_iw_set_frag,		
	(iw_handler) wl_iw_get_frag,		
	(iw_handler) wl_iw_set_txpow,		
	(iw_handler) wl_iw_get_txpow,		
#if WIRELESS_EXT > 10
	(iw_handler) wl_iw_set_retry,		
	(iw_handler) wl_iw_get_retry,		
#endif 
	(iw_handler) wl_iw_set_encode,		
	(iw_handler) wl_iw_get_encode,		
	(iw_handler) wl_iw_set_power,		
	(iw_handler) wl_iw_get_power,		
#if WIRELESS_EXT > 17
	(iw_handler) NULL,			
	(iw_handler) NULL,			
	(iw_handler) wl_iw_set_wpaie,		
	(iw_handler) wl_iw_get_wpaie,		
	(iw_handler) wl_iw_set_wpaauth,		
	(iw_handler) wl_iw_get_wpaauth,		
	(iw_handler) wl_iw_set_encodeext,	
	(iw_handler) wl_iw_get_encodeext,	
#ifdef BCMWPA2
	(iw_handler) wl_iw_set_pmksa,			/* SIOCSIWPMKSA */
#endif
#endif 
};

#if WIRELESS_EXT > 12
static const iw_handler wl_iw_priv_handler[] = {
	NULL,
	(iw_handler)wl_iw_set_active_scan,
	NULL,
	(iw_handler)wl_iw_get_rssi,
	NULL,
	(iw_handler)wl_iw_set_passive_scan,
	NULL,
	(iw_handler)wl_iw_get_link_speed,
	NULL,
	(iw_handler)wl_iw_get_macaddr,
	NULL,
	(iw_handler)wl_iw_control_wl_off,
	NULL,
	(iw_handler)wl_iw_control_wl_on,
#ifdef SOFTAP       
	NULL,
	(iw_handler)iwpriv_set_ap_config,
	NULL,
	(iw_handler)iwpriv_get_assoc_list,
	NULL,
	(iw_handler)iwpriv_set_mac_filters,
	NULL,
	(iw_handler)iwpriv_en_ap_bss,
	NULL,
	(iw_handler)iwpriv_wpasupp_loop_tst,
	NULL,
	(iw_handler)iwpriv_softap_stop,
	NULL,
	(iw_handler)iwpriv_fw_reload,
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP)
	NULL,
	(iw_handler)wl_iw_control_wl_off_softap,
	NULL,
	(iw_handler)wl_iw_control_wl_on_softap,
#endif /* defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP) */
#endif /* SOFTAP */
};

static const struct iw_priv_args wl_iw_priv_args[] = {
	{
		WL_IW_SET_ACTIVE_SCAN,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"SCAN-ACTIVE"
	},
	{
		WL_IW_GET_RSSI,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"RSSI"
	},
	{
		WL_IW_SET_PASSIVE_SCAN,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"SCAN-PASSIVE"
	},
	{
		WL_IW_GET_LINK_SPEED,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"LINKSPEED"
	},
	{
		WL_IW_GET_CURR_MACADDR,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"Macaddr"
	},
	{
		WL_IW_SET_STOP,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"STOP"
	},
	{
		WL_IW_SET_START,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"START"
	},
#ifdef SOFTAP
	{
		WL_SET_AP_CFG,
		IW_PRIV_TYPE_CHAR |  256,      
		0,
		"AP_SET_CFG"
	},
	{
		WL_AP_STA_LIST,
		0,                     
		IW_PRIV_TYPE_CHAR | 0, 
		"AP_GET_STA_LIST"
	},
	{
		WL_AP_MAC_FLTR,
		IW_PRIV_TYPE_CHAR | 256,                      
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 0,    
		"AP_SET_MAC_FLTR"
	},
	{ 
		WL_AP_BSS_START,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"AP_BSS_START"
	},
	{
		AP_LPB_CMD,
		IW_PRIV_TYPE_CHAR | 256,   
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 0,    
		"AP_LPB_CMD"
	},
	{ 
		WL_AP_STOP,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 0,   
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 0,   
		"AP_BSS_STOP"
	},
	{ 
		WL_FW_RELOAD,
		IW_PRIV_TYPE_CHAR | 256,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 0,
		"WL_FW_RELOAD"
	},
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP)
	{
		WL_IW_SET_STOP_SOFTAP,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"STOP-SOFTAP"
	},
	{
		WL_IW_SET_START_SOFTAP,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"START-SOFTAP"
	},
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) && defined(SOFTAP) */
#endif /* SOFTAP */
};

const struct iw_handler_def wl_iw_handler_def =
{
	.num_standard = ARRAYSIZE(wl_iw_handler),
	.standard = (iw_handler *) wl_iw_handler,
	.num_private = ARRAYSIZE(wl_iw_priv_handler),
	.num_private_args = ARRAY_SIZE(wl_iw_priv_args),
	.private = (iw_handler *)wl_iw_priv_handler,
	.private_args = (void *) wl_iw_priv_args,

#if WIRELESS_EXT >= 19
#if defined(BCMDONGLEHOST)
	get_wireless_stats: dhd_get_wireless_stats,
#else
	get_wireless_stats: wl_get_wireless_stats,
#endif 
#endif 
	};
#endif 

int
wl_iw_ioctl(
	struct net_device *dev,
	struct ifreq *rq,
	int cmd
)
{
	struct iwreq *wrq = (struct iwreq *) rq;
	struct iw_request_info info;
	iw_handler handler;
	char *extra = NULL;
	int token_size = 1, max_tokens = 0, ret = 0;

	if (cmd < SIOCIWFIRST ||
		IW_IOCTL_IDX(cmd) >= ARRAYSIZE(wl_iw_handler) ||
		!(handler = wl_iw_handler[IW_IOCTL_IDX(cmd)]))
		return -EOPNOTSUPP;

	switch (cmd) {

	case SIOCSIWESSID:
	case SIOCGIWESSID:
	case SIOCSIWNICKN:
	case SIOCGIWNICKN:
		max_tokens = IW_ESSID_MAX_SIZE + 1;
		break;

	case SIOCSIWENCODE:
	case SIOCGIWENCODE:
#if WIRELESS_EXT > 17
	case SIOCSIWENCODEEXT:
	case SIOCGIWENCODEEXT:
#endif
		max_tokens = IW_ENCODING_TOKEN_MAX;
		break;

	case SIOCGIWRANGE:
		max_tokens = sizeof(struct iw_range);
		break;

	case SIOCGIWAPLIST:
		token_size = sizeof(struct sockaddr) + sizeof(struct iw_quality);
		max_tokens = IW_MAX_AP;
		break;

#if WIRELESS_EXT > 13
	case SIOCGIWSCAN:
#if defined(WL_IW_USE_ISCAN)
	if (g_iscan)
		max_tokens = wrq->u.data.length;
	else
#elif defined(WL_IW_USE_ESCAN)
	if (g_escan)
		max_tokens = wrq->u.data.length;
	else
#endif
		max_tokens = IW_SCAN_MAX_DATA;
		break;
#endif 

	case SIOCSIWSPY:
		token_size = sizeof(struct sockaddr);
		max_tokens = IW_MAX_SPY;
		break;

	case SIOCGIWSPY:
		token_size = sizeof(struct sockaddr) + sizeof(struct iw_quality);
		max_tokens = IW_MAX_SPY;
		break;

#if WIRELESS_EXT > 17
        case SIOCSIWPMKSA:
        case SIOCSIWGENIE:
#endif /* WIRELESS_EXT > 17 */

	case SIOCSIWPRIV:
		max_tokens = wrq->u.data.length;
		break;
	}

	if (max_tokens && wrq->u.data.pointer) {
		if (wrq->u.data.length > max_tokens)
			return -E2BIG;

		if (!(extra = kmalloc(max_tokens * token_size, GFP_KERNEL)))
			return -ENOMEM;

		if (copy_from_user(extra, wrq->u.data.pointer, wrq->u.data.length * token_size)) {
			kfree(extra);
			return -EFAULT;
		}
	}

	info.cmd = cmd;
	info.flags = 0;

	ret = handler(dev, &info, &wrq->u, extra);

	if (extra) {
		if (copy_to_user(wrq->u.data.pointer, extra, wrq->u.data.length * token_size)) {
			kfree(extra);
			return -EFAULT;
		}

		kfree(extra);
	}

	return ret;
}

#if defined(STA) || defined(BCMDONGLEHOST)

bool
wl_iw_conn_status_str(uint32 event_type, uint32 status, uint32 reason,
	char* stringBuf, uint buflen)
{
	typedef struct conn_fail_event_map_t {
		uint32 inEvent;			
		uint32 inStatus;		
		uint32 inReason;		
		const char* outName;	
		const char* outCause;	
	} conn_fail_event_map_t;

	
#	define WL_IW_DONT_CARE	9999
	const conn_fail_event_map_t event_map [] = {
		
		
		{WLC_E_SET_SSID,     WLC_E_STATUS_SUCCESS,   WL_IW_DONT_CARE,
		"Conn", "Success"},
		{WLC_E_SET_SSID,     WLC_E_STATUS_NO_NETWORKS, WL_IW_DONT_CARE,
		"Conn", "NoNetworks"},
		{WLC_E_SET_SSID,     WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "ConfigMismatch"},
		{WLC_E_PRUNE,        WL_IW_DONT_CARE,        WLC_E_PRUNE_ENCR_MISMATCH,
		"Conn", "EncrypMismatch"},
		{WLC_E_PRUNE,        WL_IW_DONT_CARE,        WLC_E_RSN_MISMATCH,
		"Conn", "RsnMismatch"},
		{WLC_E_AUTH,         WLC_E_STATUS_TIMEOUT,   WL_IW_DONT_CARE,
		"Conn", "AuthTimeout"},
		{WLC_E_AUTH,         WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "AuthFail"},
		{WLC_E_AUTH,         WLC_E_STATUS_NO_ACK,    WL_IW_DONT_CARE,
		"Conn", "AuthNoAck"},
		{WLC_E_REASSOC,      WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "ReassocFail"},
		{WLC_E_REASSOC,      WLC_E_STATUS_TIMEOUT,   WL_IW_DONT_CARE,
		"Conn", "ReassocTimeout"},
		{WLC_E_REASSOC,      WLC_E_STATUS_ABORT,     WL_IW_DONT_CARE,
		"Conn", "ReassocAbort"},
#if defined(BCMSUP_PSK) || defined(BCMDONGLEHOST)
		{WLC_E_PSK_SUP,      WLC_SUP_KEYED,          WL_IW_DONT_CARE,
		"Sup", "ConnSuccess"},
		{WLC_E_PSK_SUP,      WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Sup", "WpaHandshakeFail"},
#endif 
		{WLC_E_DEAUTH_IND,   WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "Deauth"},
		{WLC_E_DISASSOC_IND, WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "DisassocInd"},
		{WLC_E_DISASSOC,     WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "Disassoc"}
	};

	const char* name = "";
	const char* cause = NULL;
	int i;

	
	for (i = 0;  i < sizeof(event_map)/sizeof(event_map[0]);  i++) {
		const conn_fail_event_map_t* row = &event_map[i];
		if (row->inEvent == event_type &&
		    (row->inStatus == status || row->inStatus == WL_IW_DONT_CARE) &&
		    (row->inReason == reason || row->inReason == WL_IW_DONT_CARE)) {
			name = row->outName;
			cause = row->outCause;
			break;
		}
	}

	
	if (cause) {
		memset(stringBuf, 0, buflen);
		snprintf(stringBuf, buflen, "%s %s %02d %02d",
			name, cause, status, reason);
		WL_INFORM(("Connection status: %s\n", stringBuf));
		return TRUE;
	} else {
		return FALSE;
	}
}

#if WIRELESS_EXT > 14

static bool
wl_iw_check_conn_fail(wl_event_msg_t *e, char* stringBuf, uint buflen)
{
#ifdef BCMDONGLEHOST
	uint32 event = ntoh32(e->event_type);
	uint32 status =  ntoh32(e->status);
	uint32 reason =  ntoh32(e->reason);
#else
	uint32 event = e->event_type;
	uint32 status =  e->status;
	uint32 reason =  e->reason;
#endif

	if (wl_iw_conn_status_str(event, status, reason, stringBuf, buflen)) {
		return TRUE;
	}
	else
		return FALSE;
}
#endif 
#endif 

#ifndef IW_CUSTOM_MAX
#define IW_CUSTOM_MAX 256 
#endif 

#if defined(WL_IW_USE_ESCAN)
static void
wl_iw_escan_update(wl_escan_result_t *escan_result)
{
	wl_bss_info_t *bi = escan_result->bss_info;
	escan_info_t *escan = g_escan;
	wl_bss_info_t *bss = NULL;
	wl_scan_results_t *list;
	int bi_length;
	int i;


	if (dtoh16(escan_result->bss_count) != 1) {
		WL_TRACE(("Invalid bss_count %d: ignoring\n", escan_result->bss_count));
		return;
	}
	bi_length = dtoh32(bi->length);
	if (bi_length != dtoh32(escan_result->buflen) - WL_ESCAN_RESULTS_FIXED_SIZE) {
		WL_TRACE(("Invalid bss_info length %d: ignorning\n", bi_length));
		return;
	}
	list = (wl_scan_results_t *)escan->escan_buf;
	if (bi_length > ESCAN_BUF_SIZE - list->buflen) {
		WL_TRACE(("Buffer is too small: ignorning\n"));
		return;
	}

#define WLC_BSS_RSSI_ON_CHANNEL 0x0002 

	for (i = 0; i < list->count; i++) {
		bss = bss ? (wl_bss_info_t *)((uintptr)bss + dtoh32(bss->length)) : list->bss_info;

		if (!bcmp(&bi->BSSID, &bss->BSSID, ETHER_ADDR_LEN) &&
			CHSPEC_BAND(bi->chanspec) == CHSPEC_BAND(bss->chanspec) &&
			bi->SSID_len == bss->SSID_len &&
			!bcmp(bi->SSID, bss->SSID, bi->SSID_len)) {
			if ((bss->flags & WLC_BSS_RSSI_ON_CHANNEL) ==
			    (bi->flags & WLC_BSS_RSSI_ON_CHANNEL)) {
				
				bss->RSSI = MAX(bss->RSSI, bi->RSSI);
			} else if ((bss->flags & WLC_BSS_RSSI_ON_CHANNEL) &&
			           (bi->flags & WLC_BSS_RSSI_ON_CHANNEL) == 0) {
				
				bss->RSSI = bi->RSSI;
				bss->flags |= WLC_BSS_RSSI_ON_CHANNEL;
			}

			return;
		}
	}

	memcpy(&escan->escan_buf[list->buflen], bi, bi_length);
	list->version = dtoh32(bi->version); 
	list->buflen += bi_length;
	list->count++;

	return;
}
#endif 

void
wl_iw_event(struct net_device *dev, wl_event_msg_t *e, void* data)
{
#if WIRELESS_EXT > 13
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];
	int cmd = 0;
#ifdef BCMDONGLEHOST
	uint32 event_type = ntoh32(e->event_type);
	uint16 flags =  ntoh16(e->flags);
	uint32 datalen = ntoh32(e->datalen);
	uint32 status =  ntoh32(e->status);
#else
	uint32 event_type = e->event_type;
	uint16 flags =  e->flags;
	uint32 datalen = e->datalen;
	uint32 status =  e->status;
#endif
	wl_iw_t *iw;
	uint32 toto;

	memset(&wrqu, 0, sizeof(wrqu));
	memset(extra, 0, sizeof(extra));
	iw = 0;

	

	if (!dev) {
		WL_ERROR(("%s: dev is null\n", __FUNCTION__));
		return;
	}

	iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: dev=%s event=%d \n", __FUNCTION__, dev->name, event_type));

	switch (event_type) {
#if defined(SOFTAP)
	case WLC_E_PRUNE:
		if (ap_cfg_running) {
			char *macaddr = (char *)&e->addr;
			WL_SOFTAP(("PRUNE received, %02X:%02X:%02X:%02X:%02X:%02X!\n",
				macaddr[0], macaddr[1], macaddr[2], macaddr[3], \
				macaddr[4], macaddr[5]));

			if (ap_macmode)
			{
				int i;
				for (i = 0; i < ap_black_list.count; i++) {
					if (!bcmp(macaddr, &ap_black_list.ea[i], \
						sizeof(struct ether_addr))) {
						WL_SOFTAP(("mac in black list, ignore it\n"));
						break;
					}
				}
				if (i == ap_black_list.count) {
					char mac_buf[32] = {0};
					sprintf(mac_buf, "STA_BLOCK %02X:%02X:%02X:%02X:%02X:%02X",
						macaddr[0], macaddr[1], macaddr[2],
						macaddr[3], macaddr[4], macaddr[5]);
					wl_iw_send_priv_event(priv_dev, mac_buf);
				}
			}
		}
		break;
#endif /* SOFTAP */
	case WLC_E_TXFAIL:
		cmd = IWEVTXDROP;
		memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		break;
#if WIRELESS_EXT > 14
	case WLC_E_JOIN:
	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:
#if defined(SOFTAP)
		WL_SOFTAP(("STA connect received %d\n", event_type));
		if (ap_cfg_running) {
			wl_iw_send_priv_event(priv_dev, "STA_JOIN");
			return;
		}
#endif /* SOFTAP */
		memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		cmd = IWEVREGISTERED;
		break;
	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
#if defined(SOFTAP)
		WL_SOFTAP(("STA disconnect received %d\n", event_type));
		if (ap_cfg_running) {
			wl_iw_send_priv_event(priv_dev, "STA_LEAVE");
			return;
		}
#endif /* SOFTAP */
		cmd = SIOCGIWAP;
		bzero(wrqu.addr.sa_data, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		bzero(&extra, ETHER_ADDR_LEN);
		break;
	case WLC_E_LINK:
	case WLC_E_NDIS_LINK:
		cmd = SIOCGIWAP;
		if (!(flags & WLC_EVENT_MSG_LINK)) {
			
#ifdef SOFTAP
#ifdef AP_ONLY
		if (ap_cfg_running) {
#else
		if (ap_cfg_running && !strncmp(dev->name, "wl0.1", 5)) {
#endif	
			
			WL_SOFTAP(("AP DOWN %d\n", event_type));
			wl_iw_send_priv_event(priv_dev, "AP_DOWN");
		} else {
			WL_TRACE(("STA_Link Down\n"));
			g_ss_cache_ctrl.m_link_down = 1;
		}
#else		
			g_ss_cache_ctrl.m_link_down = 1;
#endif /* SOFTAP */
			WL_TRACE(("Link Down\n"));

			bzero(wrqu.addr.sa_data, ETHER_ADDR_LEN);
			bzero(&extra, ETHER_ADDR_LEN);
#if defined(BCMDONGLEHOST)
			DHD_OS_WAKE_LOCK_TIMEOUT_ENABLE(iw->pub);
			DHD_OS_WAKE_LOCK_TIMEOUT_WITH_TIME(iw->pub, 20 * HZ);
#endif
		}
		else {
			
			memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
			g_ss_cache_ctrl.m_link_down = 0;
			
			memcpy(g_ss_cache_ctrl.m_active_bssid, &e->addr, ETHER_ADDR_LEN);
#ifdef SOFTAP

#ifdef AP_ONLY
			if (ap_cfg_running)
#else
			if (ap_cfg_running && !strncmp(dev->name, "wl0.1", 5))
#endif
			{
				WL_SOFTAP(("AP UP %d\n", event_type));
				wl_iw_send_priv_event(priv_dev, "AP_UP");
			} else {
				WL_TRACE(("STA_LINK_UP\n"));
			}
#endif /* SOFTAP */
			WL_TRACE(("Link UP\n"));

		}
		wrqu.addr.sa_family = ARPHRD_ETHER;
		break;
	case WLC_E_ACTION_FRAME:
		cmd = IWEVCUSTOM;
		if (datalen + 1 <= sizeof(extra)) {
			wrqu.data.length = datalen + 1;
			extra[0] = WLC_E_ACTION_FRAME;
			memcpy(&extra[1], data, datalen);
			printf("WLC_E_ACTION_FRAME len %d \n", wrqu.data.length);
		}
		break;

	case WLC_E_ACTION_FRAME_COMPLETE:
		cmd = IWEVCUSTOM;
		memcpy(&toto, data, 4);
		if (sizeof(status) + 1 <= sizeof(extra)) {
			wrqu.data.length = sizeof(status) + 1;
			extra[0] = WLC_E_ACTION_FRAME_COMPLETE;
			memcpy(&extra[1], &status, sizeof(status));
			printf("wl_iw_event status %d PacketId %d \n", status, toto);
			printf("WLC_E_ACTION_FRAME_COMPLETE len %d \n", wrqu.data.length);
		}
		break;
#endif 
#if WIRELESS_EXT > 17
	case WLC_E_MIC_ERROR: {
		struct	iw_michaelmicfailure  *micerrevt = (struct  iw_michaelmicfailure  *)&extra;
		cmd = IWEVMICHAELMICFAILURE;
		wrqu.data.length = sizeof(struct iw_michaelmicfailure);
		if (flags & WLC_EVENT_MSG_GROUP)
			micerrevt->flags |= IW_MICFAILURE_GROUP;
		else
			micerrevt->flags |= IW_MICFAILURE_PAIRWISE;
		memcpy(micerrevt->src_addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		micerrevt->src_addr.sa_family = ARPHRD_ETHER;

		break;
	}
#ifdef BCMWPA2
	case WLC_E_PMKID_CACHE: {
		struct iw_pmkid_cand *iwpmkidcand = (struct iw_pmkid_cand *)&extra;
		pmkid_cand_list_t *pmkcandlist;
		pmkid_cand_t	*pmkidcand;
		int count;

		if (data == NULL)
			break;

		cmd = IWEVPMKIDCAND;
		pmkcandlist = data;
		count = ntoh32_ua((uint8 *)&pmkcandlist->npmkid_cand);
		wrqu.data.length = sizeof(struct iw_pmkid_cand);
		pmkidcand = pmkcandlist->pmkid_cand;
		while (count) {
			bzero(iwpmkidcand, sizeof(struct iw_pmkid_cand));
			if (pmkidcand->preauth)
				iwpmkidcand->flags |= IW_PMKID_CAND_PREAUTH;
			bcopy(&pmkidcand->BSSID, &iwpmkidcand->bssid.sa_data,
			      ETHER_ADDR_LEN);
#ifndef SANDGATE2G
			wireless_send_event(dev, cmd, &wrqu, extra);
#endif
			pmkidcand++;
			count--;
		}
		break;
	}
#endif 
#endif 

	case WLC_E_SCAN_COMPLETE:
#if defined(WL_IW_USE_ISCAN)
		if ((g_iscan) && (g_iscan->sysioc_pid >= 0) &&
			(g_iscan->iscan_state != ISCAN_STATE_IDLE))
		{
			up(&g_iscan->sysioc_sem);
		} else {
			cmd = SIOCGIWSCAN;
			wrqu.data.length = strlen(extra);
			WL_TRACE(("Event WLC_E_SCAN_COMPLETE from specific scan\n"));
		}
#else
			cmd = SIOCGIWSCAN;
			wrqu.data.length = strlen(extra);
			WL_TRACE(("Event WLC_E_SCAN_COMPLETE\n"));
#endif 
	break;

#if defined(WL_IW_USE_ESCAN)
	case WLC_E_ESCAN_RESULT:
		if (g_escan->escan_state == ESCAN_STATE_IDLE) {
			
			break;
		}

		if (status == WLC_E_STATUS_PARTIAL) {
			wl_iw_escan_update((wl_escan_result_t *)data); 
		}
		else if (status == WLC_E_STATUS_SUCCESS) {
			g_escan->escan_state = ESCAN_STATE_IDLE; 
#if defined(CONFIG_LGE_BCM432X_PATCH)
			cmd = SIOCGIWSCAN;
			wrqu.data.length = strlen(extra);
			printk("escan success !!!\n");
#endif	/* defined(CONFIG_LGE_BCM432X_PATCH) */
		}
		else
			WL_TRACE(("Unknown escan status %d: ignoring\n", status));

		break;

	
	case WLC_E_SET_SSID: {
			if ((status == WLC_E_STATUS_NO_NETWORKS) &&
				(g_escan->escan_state == ESCAN_STATE_SCANING)) {
				WL_TRACE(("ESCAN was under progress. Resetting ESCAN_STATE\n"));
				g_escan->escan_state = ESCAN_STATE_IDLE;
			}

			break;
		}
#endif 

	default:
		
		WL_TRACE(("Unknown Event %d: ignoring\n", event_type));
		break;
	}
#if defined(CONFIG_LGE_BCM432X_PATCH) && !( defined(CONFIG_BRCM_USE_GPIO_RESET) || defined(CONFIG_BRCM_USE_DEEPSLEEP) )
	if( g_onoff == G_WLAN_SET_OFF)
		return;
#endif	/* CONFIG_LGE_BCM432X_PATCH */
#ifndef SANDGATE2G
		if (cmd)
	//20120409_Broadcom CS [Case ID]519476 [START]
		{
			if(cmd == SIOCGIWSCAN) wireless_send_event(dev, cmd, &wrqu, NULL);
			else						wireless_send_event(dev, cmd, &wrqu, extra);
		}
	//20120409_Broadcom CS [Case ID]519476 [END]
#endif

#if defined(STA) || defined(BCMDONGLEHOST)
#if WIRELESS_EXT > 14
	
	memset(extra, 0, sizeof(extra));
	if (wl_iw_check_conn_fail(e, extra, sizeof(extra))) {
		cmd = IWEVCUSTOM;
		wrqu.data.length = strlen(extra);
#ifndef SANDGATE2G
		wireless_send_event(dev, cmd, &wrqu, extra);
#endif
	}
#endif 
#endif 

#endif 
}

int wl_iw_get_wireless_stats(struct net_device *dev, struct iw_statistics *wstats)
{
	int res = 0;
	wl_cnt_t cnt;
	int phy_noise;
	int rssi;
	scb_val_t scb_val;
	phy_noise = 0;
	if ((res = dev_wlc_ioctl(dev, WLC_GET_PHY_NOISE, &phy_noise, sizeof(phy_noise))))
		goto done;

	phy_noise = dtoh32(phy_noise);
	WL_TRACE(("wl_iw_get_wireless_stats phy noise=%d\n *****", phy_noise));

	bzero(&scb_val, sizeof(scb_val_t));
	if ((res = dev_wlc_ioctl(dev, WLC_GET_RSSI, &scb_val, sizeof(scb_val_t))))
		goto done;

	rssi = dtoh32(scb_val.val);
	WL_TRACE(("wl_iw_get_wireless_stats rssi=%d ****** \n", rssi));
	if (rssi <= WL_IW_RSSI_NO_SIGNAL)
		wstats->qual.qual = 0;
	else if (rssi <= WL_IW_RSSI_VERY_LOW)
		wstats->qual.qual = 1;
	else if (rssi <= WL_IW_RSSI_LOW)
		wstats->qual.qual = 2;
	else if (rssi <= WL_IW_RSSI_GOOD)
		wstats->qual.qual = 3;
	else if (rssi <= WL_IW_RSSI_VERY_GOOD)
		wstats->qual.qual = 4;
	else
		wstats->qual.qual = 5;

	
	wstats->qual.level = 0x100 + rssi;
	wstats->qual.noise = 0x100 + phy_noise;
#if WIRELESS_EXT > 18
	wstats->qual.updated |= (IW_QUAL_ALL_UPDATED | IW_QUAL_DBM);
#else
	wstats->qual.updated |= 7;
#endif 

#if WIRELESS_EXT > 11
	WL_TRACE(("wl_iw_get_wireless_stats counters=%d\n *****", (int)sizeof(wl_cnt_t)));

	memset(&cnt, 0, sizeof(wl_cnt_t));
	res = dev_wlc_bufvar_get(dev, "counters", (char *)&cnt, sizeof(wl_cnt_t));
	if (res)
	{
		WL_ERROR(("wl_iw_get_wireless_stats counters failed error=%d ****** \n", res));
		goto done;
	}

	cnt.version = dtoh16(cnt.version);
	if (cnt.version & WL_MIN_CNT_ENABLED) {
		cnt.version &= ~WL_MIN_CNT_ENABLED;
	}
	if (cnt.version != WL_CNT_T_VERSION) {
		WL_TRACE(("\tIncorrect version of counters struct: expected %d; got %d\n",
			WL_CNT_T_VERSION, cnt.version));
		goto done;
	}

	wstats->discard.nwid = 0;
	wstats->discard.code = dtoh32(cnt.rxundec);
	wstats->discard.fragment = dtoh32(cnt.rxfragerr);
	wstats->discard.retries = dtoh32(cnt.txfail);
	wstats->discard.misc = dtoh32(cnt.rxrunt) + dtoh32(cnt.rxgiant);
	wstats->miss.beacon = 0;

	WL_TRACE(("wl_iw_get_wireless_stats counters txframe=%d txbyte=%d\n",
		dtoh32(cnt.txframe), dtoh32(cnt.txbyte)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxfrmtoolong=%d\n", dtoh32(cnt.rxfrmtoolong)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxbadplcp=%d\n", dtoh32(cnt.rxbadplcp)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxundec=%d\n", dtoh32(cnt.rxundec)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxfragerr=%d\n", dtoh32(cnt.rxfragerr)));
	WL_TRACE(("wl_iw_get_wireless_stats counters txfail=%d\n", dtoh32(cnt.txfail)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxrunt=%d\n", dtoh32(cnt.rxrunt)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxgiant=%d\n", dtoh32(cnt.rxgiant)));

#endif 

done:
	return res;
}
#if defined(COEX_DHCP)
static void
wl_iw_bt_flag_set(
	struct net_device *dev,
	bool set)
{
	char buf_flag7_dhcp_on[8] = { 7, 00, 00, 00, 0x1, 0x0, 0x00, 0x00 };
	char buf_flag7_default[8]   = { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_lock();
#endif

	if (set == TRUE) {
		
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_dhcp_on[0], sizeof(buf_flag7_dhcp_on));
	}
	else  {
		
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_default[0], sizeof(buf_flag7_default));
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_unlock();
#endif
}

static void
wl_iw_bt_timerfunc(ulong data)
{
	bt_info_t  *bt_local = (bt_info_t *)data;
	bt_local->timer_on = 0;
	WL_TRACE(("%s\n", __FUNCTION__));
	
	up(&bt_local->bt_sem);
}

static int
_bt_dhcp_sysioc_thread(void *data)
{
	DAEMONIZE("dhcp_sysioc");

	while (down_interruptible(&g_bt->bt_sem) == 0) {
		if (g_bt->timer_on) {
			del_timer(&g_bt->timer);
			g_bt->timer_on = 0;
		}

		switch (g_bt->bt_state) {
			case BT_DHCP_START:
				
				g_bt->bt_state = BT_DHCP_OPPORTUNITY_WINDOW;
				mod_timer(&g_bt->timer, jiffies +
				           BT_DHCP_OPPORTUNITY_WINDOW_TIEM*HZ/1000);
				g_bt->timer_on = 1;
				break;
			case BT_DHCP_OPPORTUNITY_WINDOW:
				
				WL_TRACE(("%s waiting for %d msec expired, force bt flag\n",
					__FUNCTION__, BT_DHCP_OPPORTUNITY_WINDOW_TIEM));
				if (g_bt->dev) wl_iw_bt_flag_set(g_bt->dev, TRUE);
				g_bt->bt_state = BT_DHCP_FLAG_FORCE_TIMEOUT;
				mod_timer(&g_bt->timer, jiffies + BT_DHCP_FLAG_FORCE_TIME*HZ/1000);
				g_bt->timer_on = 1;
				break;
			case BT_DHCP_FLAG_FORCE_TIMEOUT:
				
				WL_TRACE(("%s waiting for %d msec expired remove bt flag\n",
					__FUNCTION__, BT_DHCP_FLAG_FORCE_TIME));
				
				if (g_bt->dev)  wl_iw_bt_flag_set(g_bt->dev, FALSE);
				g_bt->bt_state = BT_DHCP_IDLE;
				g_bt->timer_on = 0;
				break;
			default:
				WL_ERROR(("%s error g_status=%d !!!\n", __FUNCTION__,
				          g_bt->bt_state));
				if (g_bt->dev) wl_iw_bt_flag_set(g_bt->dev, FALSE);
				g_bt->bt_state = BT_DHCP_IDLE;
				g_bt->timer_on = 0;
				break;
		 }
	}

	if (g_bt->timer_on) {
		del_timer(&g_bt->timer);
		g_bt->timer_on = 0;
	}
	complete_and_exit(&g_bt->bt_exited, 0);
}

static void
wl_iw_bt_release(void)
{
	bt_info_t *bt_local = g_bt;

	if (!bt_local) {
		return;
	}

	if (bt_local->bt_pid >= 0) {
		KILL_PROC(bt_local->bt_pid, SIGTERM);
		wait_for_completion(&bt_local->bt_exited);
	}
	kfree(bt_local);
	g_bt = NULL;
}

static int
wl_iw_bt_init(struct net_device *dev)
{
	bt_info_t *bt_dhcp = NULL;

	bt_dhcp = kmalloc(sizeof(bt_info_t), GFP_KERNEL);
	if (!bt_dhcp)
		return -ENOMEM;

	memset(bt_dhcp, 0, sizeof(bt_info_t));
	bt_dhcp->bt_pid = -1;
	g_bt = bt_dhcp;
	bt_dhcp->dev = dev;
	bt_dhcp->bt_state = BT_DHCP_IDLE;

	
	bt_dhcp->timer_ms    = 10;
	init_timer(&bt_dhcp->timer);
	bt_dhcp->timer.data = (ulong)bt_dhcp;
	bt_dhcp->timer.function = wl_iw_bt_timerfunc;

	sema_init(&bt_dhcp->bt_sem, 0);
	init_completion(&bt_dhcp->bt_exited);
	bt_dhcp->bt_pid = kernel_thread(_bt_dhcp_sysioc_thread, bt_dhcp, 0);
	if (bt_dhcp->bt_pid < 0) {
		WL_ERROR(("Failed in %s\n", __FUNCTION__));
#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
		kfree(bt_dhcp);
		g_bt = NULL;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
		return -ENOMEM;
	}

	return 0;
}

#endif 

int wl_iw_attach(struct net_device *dev, void * dhdp)
{
#if defined(BCMDONGLEHOST)
	wl_iw_t *iw;
#endif 
#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = NULL;
#endif

	if (!dev)
		return 0;

#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
	g_scan = NULL;

	
	g_scan = (void *)kmalloc(G_SCAN_RESULTS, GFP_KERNEL);
	if (!g_scan)
		return -ENOMEM;

	memset(g_scan, 0, G_SCAN_RESULTS);
	g_scan_specified_ssid = 0;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */

#if defined(WL_IW_USE_ISCAN)
	iscan = kmalloc(sizeof(iscan_info_t), GFP_KERNEL);
	if (!iscan)
#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
		goto fail;
#else
		return -ENOMEM;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
	memset(iscan, 0, sizeof(iscan_info_t));
	iscan->sysioc_pid = -1;
	
	g_iscan = iscan;
	iscan->dev = dev;
	iscan->iscan_state = ISCAN_STATE_IDLE;
	g_iscan->scan_flag = 0;

	
	iscan->timer_ms    = 3000;
	init_timer(&iscan->timer);
	iscan->timer.data = (ulong)iscan;
	iscan->timer.function = wl_iw_timerfunc;

	sema_init(&iscan->sysioc_sem, 0);
	init_completion(&iscan->sysioc_exited);
	iscan->sysioc_pid = kernel_thread(_iscan_sysioc_thread, iscan, 0);
	if (iscan->sysioc_pid < 0) {
#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
		kfree(iscan);
		g_iscan = NULL;
		goto fail;
#else
		return -ENOMEM;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
	}
#elif defined(WL_IW_USE_ESCAN)
	g_escan = kmalloc(sizeof(escan_info_t), GFP_KERNEL);
	if (!g_escan)
#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
		goto fail;
#else
		return -ENOMEM;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
	memset(g_escan, 0, sizeof(escan_info_t));
	g_escan->dev = dev;
	g_escan->escan_state = ESCAN_STATE_IDLE;
	g_escan->scan_flag = 0;
#ifdef CONFIG_LGE_BCM432X_PATCH // escan buf to array
	g_escan->escan_buf = escan_array;
#endif /* CONFIG_LGE_BCM432X_PATCH escan buf to array */
#endif 

#if defined(BCMDONGLEHOST)
	iw = *(wl_iw_t **)netdev_priv(dev);
	iw->pub = (dhd_pub_t *)dhdp;
#endif 
#ifdef SOFTAP
	priv_dev = dev;
	MUTEX_LOCK_SOFTAP_SET_INIT(iw->pub);
#endif /* SOFTAP */
#ifndef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
	g_scan = NULL;

	
	g_scan = (void *)kmalloc(G_SCAN_RESULTS, GFP_KERNEL);
	if (!g_scan)
		return -ENOMEM;

	memset(g_scan, 0, G_SCAN_RESULTS);
	g_scan_specified_ssid = 0;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
	
	wl_iw_init_ss_cache_ctrl();
#ifdef COEX_DHCP
	
	wl_iw_bt_init(dev);
#endif 

	return 0;
#ifdef CONFIG_LGE_BCM432X_PATCH // pecan kmalloc fail
fail:
	kfree(g_scan);
	g_scan = NULL;
	return -ENOMEM;
#endif /* CONFIG_LGE_BCM432X_PATCH pecan kmalloc fail */
}

void wl_iw_detach(void)
{
#if defined(WL_IW_USE_ISCAN)
	iscan_buf_t  *buf;
	iscan_info_t *iscan = g_iscan;

	if (!iscan)
		return;
	if (iscan->sysioc_pid >= 0) {
		KILL_PROC(iscan->sysioc_pid, SIGTERM);
		wait_for_completion(&iscan->sysioc_exited);
	}

	while (iscan->list_hdr) {
		buf = iscan->list_hdr->next;
		kfree(iscan->list_hdr);
		iscan->list_hdr = buf;
	}
	kfree(iscan);
	g_iscan = NULL;
#elif defined(WL_IW_USE_ESCAN)
	if (!g_escan)
		return;
	kfree(g_escan);
	g_escan = NULL;
#endif 

	if (g_scan)
		kfree(g_scan);

	g_scan = NULL;

	wl_iw_release_ss_cache_ctrl();
#ifdef COEX_DHCP
	wl_iw_bt_release();
#endif 
#ifdef SOFTAP
	if (ap_cfg_running) {
		WL_TRACE(("\n%s AP is going down\n", __FUNCTION__));
		wl_iw_send_priv_event(priv_dev, "AP_DOWN");
	}
#endif /* SOFTAP */
}
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-11-19, Support Host Wakeup */
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
int del_wl_timers(void)
{
#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = NULL;

	if(g_iscan)
		iscan = g_iscan;
	else
	{
		printk("[WiFi] Error g_scan is NULL \n");
		return -1;
	}

	del_timer_sync(&iscan->timer);
#endif	/* defined(WL_IW_USE_ISCAN) */
	return 0;
}
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-11-19, Support Host Wakeup */
