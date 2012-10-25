/*
 * Broadcom Dongle Host Driver (DHD), common DHD core.
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
 * $Id: dhd_common.c,v 1.5.6.8.2.6.6.30.2.31.2.19 2010/11/09 03:31:47 Exp $
 */
#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_bus.h>
#include <dhd_proto.h>
#include <dhd_dbg.h>
#include <msgtrace.h>



#ifdef PROP_TXSTATUS
#include <wlfc_proto.h>
#include <dhd_wlfc.h>
#endif

#if defined(DONGLEOVERLAYS)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
#include <linux/rtnetlink.h>
#endif
#endif /* DONGLEOVERLAYS */

int dhd_msg_level;


#include <wl_iw.h>

char fw_path[MOD_PARAM_PATHLEN];
char nv_path[MOD_PARAM_PATHLEN];
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-04-03, configs */
#if defined(CONFIG_LGE_BCM432X_PATCH)
char config_path[MOD_PARAM_PATHLEN] = "";
#endif /* CONFIG_LGE_BCM432X_PATCH */
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-04-03, configs */

/* Last connection success/failure status */
uint32 dhd_conn_event;
uint32 dhd_conn_status;
uint32 dhd_conn_reason;

#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i

extern void dhd_ind_scan_confirm(void *h, bool status);
extern int dhd_iscan_in_progress(void *h);
void dhd_iscan_lock(void);
void dhd_iscan_unlock(void);

/* Packet alignment for most efficient SDIO (can change based on platform) */
#ifndef DHD_SDALIGN
#define DHD_SDALIGN	32
#endif
#if !ISPOWEROF2(DHD_SDALIGN)
#error DHD_SDALIGN is not a power of 2!
#endif

#ifdef DHD_DEBUG
const char dhd_version[] = "Dongle Host Driver, version " EPI_VERSION_STR "\nCompiled on "
	__DATE__ " at " __TIME__;
#else
const char dhd_version[] = "Dongle Host Driver, version " EPI_VERSION_STR;
#endif

void dhd_set_timer(void *bus, uint wdtick);

#if defined(DONGLEOVERLAYS)
static int _overlay_req_sysioc_thread(void *data);
#endif


/* IOVar table */
enum {
	IOV_VERSION = 1,
	IOV_MSGLEVEL,
	IOV_BCMERRORSTR,
	IOV_BCMERROR,
	IOV_WDTICK,
	IOV_DCONSOLE_POLL,
	IOV_DUMP,
	IOV_CLEARCOUNTS,
	IOV_LOGDUMP,
	IOV_LOGCAL,
	IOV_LOGSTAMP,
	IOV_GPIOOB,
	IOV_IOCTLTIMEOUT,
#if defined(DHD_DEBUG)
	IOV_CONS,
#endif 
#ifdef PROP_TXSTATUS
	IOV_PROPTXSTATUS_MODE,
#endif
#if defined(DONGLEOVERLAYS)
	IOV_OVERLAYTAB,
#endif
	IOV_BUS_TYPE,
	IOV_LAST
};

const bcm_iovar_t dhd_iovars[] = {
	{"version", 	IOV_VERSION,	0,	IOVT_BUFFER,	sizeof(dhd_version) },
#ifdef DHD_DEBUG
	{"msglevel",	IOV_MSGLEVEL,	0,	IOVT_UINT32,	0 },
#endif /* DHD_DEBUG */
	{"bcmerrorstr", IOV_BCMERRORSTR, 0, IOVT_BUFFER,	BCME_STRLEN },
	{"bcmerror",	IOV_BCMERROR,	0,	IOVT_INT8,	0 },
	{"wdtick",	IOV_WDTICK, 0,	IOVT_UINT32,	0 },
	{"dump",	IOV_DUMP,	0,	IOVT_BUFFER,	DHD_IOCTL_MAXLEN },
#ifdef DHD_DEBUG
	{"dconpoll",	IOV_DCONSOLE_POLL, 0,	IOVT_UINT32,	0 },
	{"cons",	IOV_CONS,	0,	IOVT_BUFFER,	0 },
#endif
	{"clearcounts", IOV_CLEARCOUNTS, 0, IOVT_VOID,	0 },
	{"gpioob",	IOV_GPIOOB,	0,	IOVT_UINT32,	0 },
	{"ioctl_timeout",	IOV_IOCTLTIMEOUT,	0,	IOVT_UINT32,	0 },
#ifdef PROP_TXSTATUS
	/*
	set the proptxtstatus operation mode:
	0 - Do not do any proptxtstatus flow control
	1 - Use implied credit from a packet status
	2 - Use explicit credit
	*/
	{"ptxmode",	IOV_PROPTXSTATUS_MODE,	0,	IOVT_UINT32,	0 },
#endif
#if defined(DONGLEOVERLAYS)
	{"overlaytab", IOV_OVERLAYTAB, 0,	IOVT_BUFFER,	0},
#endif
	{"bustype", IOV_BUS_TYPE, 0, IOVT_UINT32, 0},
	{NULL, 0, 0, 0, 0 }
};

struct dhd_cmn *
dhd_common_init(osl_t *osh)
{
	dhd_cmn_t *cmn;

	/* Init global variables at run-time, not as part of the declaration.
	 * This is required to support init/de-init of the driver. Initialization
	 * of globals as part of the declaration results in non-deterministic
	 * behaviour since the value of the globals may be different on the
	 * first time that the driver is initialized vs subsequent initializations.
	 */
	dhd_msg_level = DHD_ERROR_VAL;
	/* Allocate private bus interface state */
	if (!(cmn = MALLOC(osh, sizeof(dhd_cmn_t)))) {
		DHD_ERROR(("%s: MALLOC failed\n", __FUNCTION__));
		return NULL;
	}
	memset(cmn, 0, sizeof(dhd_cmn_t));
	cmn->osh = osh;

#if defined(DONGLEOVERLAYS)
	sema_init(&cmn->sysioc_sem, 0);
	init_completion(&cmn->sysioc_exited);
	spin_lock_init(&cmn->lock);
	pktq_init(&cmn->overlay_req, 1, 100);
	cmn->sysioc_pid = kernel_thread(_overlay_req_sysioc_thread, cmn, 0);
	if (cmn->sysioc_pid < 0) {
		DHD_ERROR(("%s: unable to start _overlay_req_sysioc_thread\n",
		           __FUNCTION__));
	}
#endif /* DONGLEOVERLAYS */

#ifdef CONFIG_BCM4329_FW_PATH
	strncpy(fw_path, CONFIG_BCM4329_FW_PATH, MOD_PARAM_PATHLEN-1);
#else
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-09-03, don't init */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	fw_path[0] = '\0';
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-09-03, don't init */
#endif /* CONFIG_LGE_BCM432X_PATCH */
#endif
#ifdef CONFIG_BCM4329_NVRAM_PATH
	strncpy(nv_path, CONFIG_BCM4329_NVRAM_PATH, MOD_PARAM_PATHLEN-1);
#else
/* LGE_CHANGE_S [yoohoo@lge.com] 2009-09-03, don't init */
#if !defined(CONFIG_LGE_BCM432X_PATCH)
	nv_path[0] = '\0';
/* LGE_CHANGE_E [yoohoo@lge.com] 2009-09-03, don't init */
#endif /* CONFIG_LGE_BCM432X_PATCH */
#endif

	return cmn;
}

void
dhd_common_deinit(dhd_pub_t *dhd_pub)
{
	osl_t *osh;
	dhd_cmn_t *cmn = dhd_pub->cmn;

	if (!cmn)
		return;

	osh = cmn->osh;
#if defined(DONGLEOVERLAYS)
	if (cmn->sysioc_pid >= 0) {
		cmn->overlay_thread_terminate = 1;
		KILL_PROC(cmn->sysioc_pid, SIGTERM);
		wait_for_completion(&cmn->sysioc_exited);
	}
	if (cmn->overlaytab) {
		MFREE(cmn->osh, cmn->overlaytab, cmn->overlaytabsz);
		cmn->overlaytab = NULL;
		cmn->overlaytabsz = 0;
	}
#endif /* DONGLEOVERLAYS */
	dhd_pub->cmn = NULL;
	MFREE(osh, cmn, sizeof(dhd_cmn_t));
}

static int
dhd_dump(dhd_pub_t *dhdp, char *buf, int buflen)
{
	char eabuf[ETHER_ADDR_STR_LEN];

	struct bcmstrbuf b;
	struct bcmstrbuf *strbuf = &b;

	bcm_binit(strbuf, buf, buflen);

	/* Base DHD info */
	bcm_bprintf(strbuf, "%s\n", dhd_version);
	bcm_bprintf(strbuf, "\n");
	bcm_bprintf(strbuf, "pub.up %d pub.txoff %d pub.busstate %d\n",
	            dhdp->up, dhdp->txoff, dhdp->busstate);
	bcm_bprintf(strbuf, "pub.hdrlen %d pub.maxctl %d pub.rxsz %d\n",
	            dhdp->hdrlen, dhdp->maxctl, dhdp->rxsz);
	bcm_bprintf(strbuf, "pub.iswl %d pub.drv_version %ld pub.mac %s\n",
	            dhdp->iswl, dhdp->drv_version, bcm_ether_ntoa(&dhdp->mac, eabuf));
	bcm_bprintf(strbuf, "pub.bcmerror %d tickcnt %d\n", dhdp->bcmerror, dhdp->tickcnt);

	bcm_bprintf(strbuf, "dongle stats:\n");
	bcm_bprintf(strbuf, "tx_packets %ld tx_bytes %ld tx_errors %ld tx_dropped %ld\n",
	            dhdp->dstats.tx_packets, dhdp->dstats.tx_bytes,
	            dhdp->dstats.tx_errors, dhdp->dstats.tx_dropped);
	bcm_bprintf(strbuf, "rx_packets %ld rx_bytes %ld rx_errors %ld rx_dropped %ld\n",
	            dhdp->dstats.rx_packets, dhdp->dstats.rx_bytes,
	            dhdp->dstats.rx_errors, dhdp->dstats.rx_dropped);
	bcm_bprintf(strbuf, "multicast %ld\n", dhdp->dstats.multicast);

	bcm_bprintf(strbuf, "bus stats:\n");
	bcm_bprintf(strbuf, "tx_packets %ld tx_multicast %ld tx_errors %ld\n",
	            dhdp->tx_packets, dhdp->tx_multicast, dhdp->tx_errors);
	bcm_bprintf(strbuf, "tx_ctlpkts %ld tx_ctlerrs %ld\n",
	            dhdp->tx_ctlpkts, dhdp->tx_ctlerrs);
	bcm_bprintf(strbuf, "rx_packets %ld rx_multicast %ld rx_errors %ld \n",
	            dhdp->rx_packets, dhdp->rx_multicast, dhdp->rx_errors);
	bcm_bprintf(strbuf, "rx_ctlpkts %ld rx_ctlerrs %ld rx_dropped %ld rx_flushed %ld\n",
	            dhdp->rx_ctlpkts, dhdp->rx_ctlerrs, dhdp->rx_dropped, dhdp->rx_flushed);
	bcm_bprintf(strbuf, "rx_readahead_cnt %ld tx_realloc %ld fc_packets %ld\n",
	            dhdp->rx_readahead_cnt, dhdp->tx_realloc, dhdp->fc_packets);
	bcm_bprintf(strbuf, "wd_dpc_sched %ld\n", dhdp->wd_dpc_sched);
	bcm_bprintf(strbuf, "\n");

	/* Add any prot info */
	dhd_prot_dump(dhdp, strbuf);
	bcm_bprintf(strbuf, "\n");

	/* Add any bus info */
	dhd_bus_dump(dhdp, strbuf);

	return (!strbuf->size ? BCME_BUFTOOSHORT : 0);
}

int
dhd_wl_ioctl_cmd(dhd_pub_t *dhd_pub, int cmd, void *arg, int len, uint8 set, int ifindex)
{
	wl_ioctl_t ioc;

	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;
#ifdef DONGLEOVERLAYS
	ioc.action = set ? WL_IOCTL_ACTION_SET : WL_IOCTL_ACTION_GET;
#else
	ioc.set = set;
#endif

	return dhd_wl_ioctl(dhd_pub, ifindex, &ioc, arg, len);
}

#if defined(DONGLEOVERLAYS)
wl_ioctl_overlay_t *
dhd_mkoverlay(dhd_pub_t *dhd_pub, uint8 *overlay, uint32 oidx, int offset, int osize, int *obuflen)
{
	wl_ioctl_overlay_t *op;

	op = (wl_ioctl_overlay_t *) MALLOC(dhd_pub->osh, sizeof(wl_ioctl_overlay_t) + osize);
	if (!op) {
		DHD_ERROR(("%s: malloc failed for size %d\n", __FUNCTION__, offset + osize));
		return NULL;
	}

	/* HTOL32 should byte swap for BIG ENDIAN host */
	op->flags_idx = HTOL32(oidx);
	op->offset = HTOL32(offset);
	op->len = HTOL32(osize);
	memcpy((uint8*)op + sizeof(wl_ioctl_overlay_t), overlay + offset, osize);
	DHD_TRACE(("%s: overlay idx %d, offset %d, osize %d\n",
	           __FUNCTION__, oidx, offset, osize));

	*obuflen = sizeof(wl_ioctl_overlay_t) + osize;

	return op;
}

static uint8 *
dhd_find_overlay(dhd_pub_t *dhd_pub, uint32 cmd, char *buf, int32 *idx, int *osize,
                 bool match_flags)
{
	dhd_cmn_t *cmn = dhd_pub->cmn;
	uint32 *vals = (uint32 *)cmn->overlaytab;
	uint32 header_sz = vals[0];
	uint32 ocount = vals[1];
	uint32 *offsets = &vals[2];
	uint8 *obase = cmn->overlaytab + sizeof(uint32) + header_sz;
	uint8 *overlay = obase;
	int i = 0;
	uint32 ocmd1, ocmd2, oidx;
	bool match = FALSE;
	uint32 index = (uint32) *idx;
	bool firmware_req = ((int32)index != -1);
	bool iovar;
	char *name = NULL;

	/*
	 * cmn->overlaytab contains:
	 *
	 * uint32 header size
	 * uint32 offset count
	 * uint32 * (offset count) overlay offsets
	 * uint32 region count
	 * uint32 * (region count) overlay region addresses
	 * variable length overlay code/data
	 */

	/* offsets (start, end) must account for at least one overlay */
	ASSERT(ocount > 1);

	if (firmware_req && !match_flags)
		index &= OVERLAY_IDX_MASK;

	/* check if it's an iovar */
	if (cmd == WLC_SET_VAR || cmd == WLC_GET_VAR) {
		iovar = TRUE;
		/* some iovars have an xx: prefix. skip it */
		if ((name = strchr(buf, ':')))
			++name;
		else
			name = buf;
		DHD_TRACE(("%s: got iovar %s %s\n", __FUNCTION__,
		           cmd == WLC_SET_VAR ? "set" : "get",
		           name));
	} else
		iovar = FALSE;

	/* assumes cmn->overlaytab is 4-byte aligned */
	while (i < (ocount - 1)) {
		ocmd1 = ((uint32*)overlay)[0];
		ocmd2 = ((uint32*)overlay)[1];
		oidx = ((uint32*)overlay)[2];
		if (!match_flags)
			oidx &= OVERLAY_IDX_MASK;

		DHD_TRACE(("%s: check cmd %d against %d and %d, offset %d\n", __FUNCTION__,
		           cmd, ocmd1, ocmd2, offsets[i]));
		if (cmd == ocmd1 || cmd == ocmd2) {
			/* match the command name if it's an iovar */
			if (iovar) {
				char *cp = overlay + (sizeof(uint32) * 3);
				while (*cp) {
					DHD_TRACE(("%s: compare against %s\n", __FUNCTION__,
					           cp));
					if (!strcmp(name, cp)) {
						DHD_TRACE(("%s: found iovar %s %s\n", __FUNCTION__,
						           cmd == WLC_SET_VAR ? "set" : "get",
						           cp));
						match = TRUE;
						break;
					}
					cp += strlen(cp) + 1;
				}
			} else {
				DHD_TRACE(("%s: found ioctl %d\n", __FUNCTION__, cmd));
				match = TRUE;
			}
			/* if index was specified, make sure it matches */
			if (match) {
				if (firmware_req && index != oidx)
					match = FALSE;
				else {
					/* size = diff betw. offsets, minus table lookup overhead */
					*osize = offsets[i + 1] - offsets[i];
					*idx = ((uint32*)overlay)[2];
					return overlay;
				}
			}
		}
		overlay = obase + offsets[++i];
	}
	if (firmware_req)
		DHD_ERROR(("%s: fw req for idx 0x%x; name %s not found\n", __FUNCTION__,
		           *idx, name ? name : "<NULL>"));

	return NULL;
}

int
dhd_wl_ioctl_overlay(dhd_pub_t *dhd_pub, int ifindex, wl_ioctl_t *ioc, void *buf, int len,
                     int32 idx, bool *found)
{
	uint32 cmd = ioc->cmd;
	uint8 *overlay;
	int osize;
	bool firmware_req = (idx != -1);
	int ret = BCME_OK;

	*found = FALSE;

	overlay = dhd_find_overlay(dhd_pub, cmd, buf, &idx, &osize, FALSE);

	if (overlay) {
		*found = TRUE;

		/* add overlay index and "overlay" bits to the action field */
		ioc->action |= ((idx & OVERLAY_IDX_MASK) << WL_IOCTL_ACTION_OVL_SHIFT) |
		    WL_IOCTL_ACTION_OVL;

		/* just issue the IOCTL if DEFER_DL set and the request is not from the device */
		/* (DEFER_DL means the device will request it later via WLC_E_OVL_DOWNLOAD) */
		if ((idx & OVERLAY_FLAG_DEFER_DL) && !firmware_req) {
			/* tell the firmware which overlay region to reserve */
			ioc->action |= WL_IOCTL_ACTION_OVL_RSV;
			DHD_TRACE(("%s: defer idx %x, action 0x%x\n", __FUNCTION__,
			           idx, ioc->action));
			goto sendreq;
		}
		if ((idx & OVERLAY_FLAG_PRESLEEP) && (firmware_req)) {
			idx &= ~OVERLAY_FLAG_PRESLEEP;
		}

		if (idx & OVERLAY_FLAG_PRESLEEP) {
			/* the IOCTL/iovar is sent first to flush the overlay region */
			DHD_ERROR(("%s: PRESLEEP IOCTL issued for idx 0x%x\n",
			           __FUNCTION__, idx));
			/* include the overlay size after the "presleep" string */
			memcpy((char *)buf + 9, &osize, sizeof(osize));
			if ((ret = dhd_prot_ioctl(dhd_pub, ifindex, ioc, buf, len)) != BCME_OK)
				return ret;
		}

		/* let the bus layer decide how best to download the overlay code/data */
		ret = dhd_bus_overlay_dl(dhd_pub, ifindex, overlay, osize, idx,
		                         dhd_pub->cmn->regions[idx & OVERLAY_IDX_MASK]);
		if (ret != BCME_OK)
			return ret;

		if (idx & OVERLAY_FLAG_PRESLEEP) {
			/* tell the device to start using the presleep overlay */
			uint32 presleep_resident = 1;
			/* gPresleepResident flag is uint32 immediately before region 0 */
			return dhd_bus_membytes(dhd_pub, TRUE,
			                        dhd_pub->cmn->regions[0] - sizeof(uint32),
			                        (uint8 *)&presleep_resident, sizeof(uint32));
		}

sendreq:
		/* now call the original ioctl */
		ret = dhd_prot_ioctl(dhd_pub, ifindex, ioc, buf, len);

		return ret;
	}

	return BCME_NOTFOUND;
}
#endif /* DONGLEOVERLAYS */

int
dhd_wl_ioctl(dhd_pub_t *dhd_pub, int ifindex, wl_ioctl_t *ioc, void *buf, int len)
{
	int ret;

#if defined(DONGLEOVERLAYS)
	if (!dhd_pub->cmn)
		return BCME_ERROR;
	/* need to serialize here since overlays involve an initial (over)write to overlay memory */
	dhd_os_proto_block(dhd_pub);

	if (dhd_pub->cmn->overlaytab) {
		bool found;
		ret = dhd_wl_ioctl_overlay(dhd_pub, ifindex, ioc, buf, len, -1, &found);
		if (found)
			goto done;
	}
#endif /* DONGLEOVERLAYS */

	ret = dhd_prot_ioctl(dhd_pub, ifindex, ioc, buf, len);

#if defined(DONGLEOVERLAYS)
done:
	dhd_os_proto_unblock(dhd_pub);
#endif

	return ret;
}

static int
dhd_doiovar(dhd_pub_t *dhd_pub, const bcm_iovar_t *vi, uint32 actionid, const char *name,
            void *params, int plen, void *arg, int len, int val_size)
{
	int bcmerror = 0;
	int32 int_val = 0;
#if defined(DONGLEOVERLAYS)
	dhd_cmn_t *cmn = dhd_pub->cmn;
#endif

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	DHD_TRACE(("%s: actionid = %d\n", __FUNCTION__, actionid));
	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid))) != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	switch (actionid) {
	case IOV_GVAL(IOV_VERSION):
		/* Need to have checked buffer length */
		strncpy((char*)arg, dhd_version, len);
		break;

	case IOV_GVAL(IOV_MSGLEVEL):
		int_val = (int32)dhd_msg_level;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_MSGLEVEL):
		dhd_msg_level = int_val;
		break;
	case IOV_GVAL(IOV_BCMERRORSTR):
		strncpy((char *)arg, bcmerrorstr(dhd_pub->bcmerror), BCME_STRLEN);
		((char *)arg)[BCME_STRLEN - 1] = 0x00;
		break;

	case IOV_GVAL(IOV_BCMERROR):
		int_val = (int32)dhd_pub->bcmerror;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_WDTICK):
		int_val = (int32)dhd_watchdog_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WDTICK):
		if (!dhd_pub->up) {
			bcmerror = BCME_NOTUP;
			break;
		}
		dhd_os_wd_timer(dhd_pub, (uint)int_val);
		break;

	case IOV_GVAL(IOV_DUMP):
		bcmerror = dhd_dump(dhd_pub, arg, len);
		break;

#ifdef DHD_DEBUG
	case IOV_GVAL(IOV_DCONSOLE_POLL):
		int_val = (int32)dhd_console_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DCONSOLE_POLL):
		dhd_console_ms = (uint)int_val;
		break;

	case IOV_SVAL(IOV_CONS):
		if (len > 0)
			bcmerror = dhd_bus_console_in(dhd_pub, arg, len - 1);
		break;
#endif

	case IOV_SVAL(IOV_CLEARCOUNTS):
		dhd_pub->tx_packets = dhd_pub->rx_packets = 0;
		dhd_pub->tx_errors = dhd_pub->rx_errors = 0;
		dhd_pub->tx_ctlpkts = dhd_pub->rx_ctlpkts = 0;
		dhd_pub->tx_ctlerrs = dhd_pub->rx_ctlerrs = 0;
		dhd_pub->rx_dropped = 0;
		dhd_pub->rx_readahead_cnt = 0;
		dhd_pub->tx_realloc = 0;
		dhd_pub->wd_dpc_sched = 0;
		memset(&dhd_pub->dstats, 0, sizeof(dhd_pub->dstats));
		dhd_bus_clearcounts(dhd_pub);
#ifdef PROP_TXSTATUS
		/* clear proptxstatus related counters */
		{
			wlfc_hanger_t* hanger;

			memset(&((athost_wl_status_info_t*)(dhd_pub->wlfc_state))->stats, 0,
				sizeof(athost_wl_stat_counters_t));
			hanger = (wlfc_hanger_t*)((athost_wl_status_info_t*)
				(dhd_pub->wlfc_state))->hanger;
			hanger->pushed = 0;
			hanger->popped = 0;
			hanger->failed_slotfind = 0;
			hanger->failed_to_pop = 0;
			hanger->failed_to_push = 0;
		}
#endif /* PROP_TXSTATUS */
		break;


	case IOV_GVAL(IOV_IOCTLTIMEOUT): {
		int_val = (int32)dhd_os_get_ioctl_resp_timeout();
		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_IOCTLTIMEOUT): {
		if (int_val <= 0)
			bcmerror = BCME_BADARG;
		else
			dhd_os_set_ioctl_resp_timeout((unsigned int)int_val);
		break;
	}


#ifdef PROP_TXSTATUS
	case IOV_GVAL(IOV_PROPTXSTATUS_MODE):
		int_val = (int32)((athost_wl_status_info_t*)dhd_pub->wlfc_state)->proptxstatus_mode;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_MODE):
		((athost_wl_status_info_t*)dhd_pub->wlfc_state)->proptxstatus_mode = int_val & 0xff;
		break;
#endif

	case IOV_GVAL(IOV_BUS_TYPE):
	/* The dhd application query the driver to check if its usb or sdio.  */
#ifdef BCMDHDUSB
		int_val = BUS_TYPE_USB;
#endif
		int_val = BUS_TYPE_SDIO;
		bcopy(&int_val, arg, val_size);
		break;

#if defined(DONGLEOVERLAYS)
	case IOV_SVAL(IOV_OVERLAYTAB):
	{
		static uint32 offset;

		/* first int in first chunk is the overall size */
		if (cmn->overlaytab == NULL) {
			ASSERT(plen > sizeof(int_val));
			if (!(cmn->overlaytab =
			      MALLOC(dhd_pub->osh, int_val - sizeof(uint32)))) {
				bcmerror = BCME_NOMEM;
				break;
			}
			DHD_TRACE(("%s: overlaytabsize %d\n", __FUNCTION__,
			           int_val - (int)sizeof(uint32)));
			/* table access code assumes malloc was 4-byte aligned */
			ASSERT(ISALIGNED((uintptr)cmn->overlaytab, 4));
			params = (void*) (uint8*)params + sizeof(uint32);
			plen -= sizeof(uint32);
			cmn->overlaytabsz = int_val - sizeof(uint32);
			offset = 0;
		}

		bcopy((char *)params, cmn->overlaytab + offset, plen);
		offset += plen;

		/* got it all: now write overlay code/data that's marked POSTLOAD */
		if (offset == cmn->overlaytabsz) {
			uint32 idx, i;
			uint32 *vals = (uint32 *)cmn->overlaytab;
			uint32 ocount = vals[1];
			uint32 rcount = vals[2 + ocount];
			uint32 *region = &vals[3 + ocount];
			uint8 *overlay;
			int osize;

			/* store away a pointer to the regions for easy access */
			cmn->regions = region;

			/* write the POSTLOAD code/data for each overlay region */
			for (i = 0; i < rcount; ++i) {
				idx = OVERLAY_FLAG_POSTLOAD | i;
				overlay = dhd_find_overlay(dhd_pub, 0xfffffffe, NULL, &idx,
				                           &osize, TRUE);
				if (overlay) {
					int ret = dhd_bus_membytes(dhd_pub, TRUE,
					                           region[i], overlay,
					                           osize);
					if (ret) {
						DHD_ERROR(("%s: dhd_bus_membytes failed for "
						           "region %d w/status %d\n",
						           __FUNCTION__, i, ret));
						break;
					} else {
						DHD_TRACE(("%s: wrote POSTLOAD 0x%x; size %d "
						           "to addr 0x%x\n",
						           __FUNCTION__, idx, osize, region[i]));
					}
				}
			}
		}
		break;
	}
#endif /* DONGLEOVERLAYS */

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}

exit:
	DHD_TRACE(("%s: actionid %d, bcmerror %d\n", __FUNCTION__, actionid, bcmerror));
	return bcmerror;
}

/* Store the status of a connection attempt for later retrieval by an iovar */
void
dhd_store_conn_status(uint32 event, uint32 status, uint32 reason)
{
	/* Do not overwrite a WLC_E_PRUNE with a WLC_E_SET_SSID
	 * because an encryption/rsn mismatch results in both events, and
	 * the important information is in the WLC_E_PRUNE.
	 */
	if (!(event == WLC_E_SET_SSID && status == WLC_E_STATUS_FAIL &&
	      dhd_conn_event == WLC_E_PRUNE)) {
		dhd_conn_event = event;
		dhd_conn_status = status;
		dhd_conn_reason = reason;
	}
}

bool
dhd_prec_enq(dhd_pub_t *dhdp, struct pktq *q, void *pkt, int prec)
{
	void *p;
	int eprec = -1;		/* precedence to evict from */
	bool discard_oldest;

	/* Fast case, precedence queue is not full and we are also not
	 * exceeding total queue length
	 */
	if (!pktq_pfull(q, prec) && !pktq_full(q)) {
		pktq_penq(q, prec, pkt);
		return TRUE;
	}

	/* Determine precedence from which to evict packet, if any */
	if (pktq_pfull(q, prec))
		eprec = prec;
	else if (pktq_full(q)) {
		p = pktq_peek_tail(q, &eprec);
		ASSERT(p);
		if (eprec > prec)
			return FALSE;
	}

	/* Evict if needed */
	if (eprec >= 0) {
		/* Detect queueing to unconfigured precedence */
		ASSERT(!pktq_pempty(q, eprec));
		discard_oldest = AC_BITMAP_TST(dhdp->wme_dp, eprec);
		if (eprec == prec && !discard_oldest)
			return FALSE;		/* refuse newer (incoming) packet */
		/* Evict packet according to discard policy */
		p = discard_oldest ? pktq_pdeq(q, eprec) : pktq_pdeq_tail(q, eprec);
		if (p == NULL) {
			DHD_ERROR(("%s: pktq_penq() failed, oldest %d.",
				__FUNCTION__, discard_oldest));
			ASSERT(p);
		}

		PKTFREE(dhdp->osh, p, TRUE);
	}

	/* Enqueue */
	p = pktq_penq(q, prec, pkt);
	if (p == NULL) {
		DHD_ERROR(("%s: pktq_penq() failed.", __FUNCTION__));
		ASSERT(p);
	}

	return TRUE;
}

static int
dhd_iovar_op(dhd_pub_t *dhd_pub, const char *name,
             void *params, int plen, void *arg, int len, bool set)
{
	int bcmerror = 0;
	int val_size;
	const bcm_iovar_t *vi = NULL;
	uint32 actionid;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(name);
	ASSERT(len >= 0);

	/* Get MUST have return space */
	ASSERT(set || (arg && len));

	/* Set does NOT take qualifiers */
	ASSERT(!set || (!params && !plen));

	if ((vi = bcm_iovar_lookup(dhd_iovars, name)) == NULL) {
		bcmerror = BCME_UNSUPPORTED;
		goto exit;
	}

	DHD_CTL(("%s: %s %s, len %d plen %d\n", __FUNCTION__,
	         name, (set ? "set" : "get"), len, plen));

	/* set up 'params' pointer in case this is a set command so that
	 * the convenience int and bool code can be common to set and get
	 */
	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		/* all other types are integer sized */
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);
	bcmerror = dhd_doiovar(dhd_pub, vi, actionid, name, params, plen, arg, len, val_size);

exit:
	return bcmerror;
}

int
dhd_ioctl(dhd_pub_t *dhd_pub, dhd_ioctl_t *ioc, void *buf, uint buflen)
{
	int bcmerror = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (!buf) {
		return BCME_BADARG;
	}

	switch (ioc->cmd) {
	case DHD_GET_MAGIC:
		if (buflen < sizeof(int))
			bcmerror = BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_MAGIC;
		break;

	case DHD_GET_VERSION:
		if (buflen < sizeof(int))
			bcmerror = -BCME_BUFTOOSHORT;
		else
			*(int*)buf = DHD_IOCTL_VERSION;
		break;

	case DHD_GET_VAR:
	case DHD_SET_VAR: {
		char *arg;
		uint arglen;

		/* scan past the name to any arguments */
		for (arg = buf, arglen = buflen; *arg && arglen; arg++, arglen--);

		if (*arg) {
			bcmerror = BCME_BUFTOOSHORT;
			break;
		}

		/* account for the NUL terminator */
		arg++, arglen--;

		/* call with the appropriate arguments */
		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_iovar_op(dhd_pub, buf, arg, arglen,
			buf, buflen, IOV_GET);
		else
			bcmerror = dhd_iovar_op(dhd_pub, buf, NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		/* not in generic table, try protocol module */
		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf, arg,
			                             arglen, buf, buflen, IOV_GET);
		else
			bcmerror = dhd_prot_iovar_op(dhd_pub, buf,
			                             NULL, 0, arg, arglen, IOV_SET);
		if (bcmerror != BCME_UNSUPPORTED)
			break;

		/* if still not found, try bus module */
		if (ioc->cmd == DHD_GET_VAR)
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
			                            arg, arglen, buf, buflen, IOV_GET);
		else
			bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
			                            NULL, 0, arg, arglen, IOV_SET);

		break;
	}

	default:
		bcmerror = BCME_UNSUPPORTED;
	}

	return bcmerror;
}

#ifdef SHOW_EVENTS
static void
wl_show_host_event(wl_event_msg_t *event, void *event_data)
{
	uint i, status, reason;
	bool group = FALSE, flush_txq = FALSE, link = FALSE;
	char *auth_str, *event_name;
	uchar *buf;
	char err_msg[256], eabuf[ETHER_ADDR_STR_LEN];
	static struct {uint event; char *event_name;} event_names[] = {
		{WLC_E_SET_SSID, "SET_SSID"},
		{WLC_E_JOIN, "JOIN"},
		{WLC_E_START, "START"},
		{WLC_E_AUTH, "AUTH"},
		{WLC_E_AUTH_IND, "AUTH_IND"},
		{WLC_E_DEAUTH, "DEAUTH"},
		{WLC_E_DEAUTH_IND, "DEAUTH_IND"},
		{WLC_E_ASSOC, "ASSOC"},
		{WLC_E_ASSOC_IND, "ASSOC_IND"},
		{WLC_E_REASSOC, "REASSOC"},
		{WLC_E_REASSOC_IND, "REASSOC_IND"},
		{WLC_E_DISASSOC, "DISASSOC"},
		{WLC_E_DISASSOC_IND, "DISASSOC_IND"},
		{WLC_E_QUIET_START, "START_QUIET"},
		{WLC_E_QUIET_END, "END_QUIET"},
		{WLC_E_BEACON_RX, "BEACON_RX"},
		{WLC_E_LINK, "LINK"},
		{WLC_E_MIC_ERROR, "MIC_ERROR"},
		{WLC_E_NDIS_LINK, "NDIS_LINK"},
		{WLC_E_ROAM, "ROAM"},
		{WLC_E_TXFAIL, "TXFAIL"},
		{WLC_E_PMKID_CACHE, "PMKID_CACHE"},
		{WLC_E_RETROGRADE_TSF, "RETROGRADE_TSF"},
		{WLC_E_PRUNE, "PRUNE"},
		{WLC_E_AUTOAUTH, "AUTOAUTH"},
		{WLC_E_EAPOL_MSG, "EAPOL_MSG"},
		{WLC_E_SCAN_COMPLETE, "SCAN_COMPLETE"},
		{WLC_E_ADDTS_IND, "ADDTS_IND"},
		{WLC_E_DELTS_IND, "DELTS_IND"},
		{WLC_E_BCNSENT_IND, "BCNSENT_IND"},
		{WLC_E_BCNRX_MSG, "BCNRX_MSG"},
		{WLC_E_BCNLOST_MSG, "BCNLOST_MSG"},
		{WLC_E_ROAM_PREP, "ROAM_PREP"},
		{WLC_E_PFN_NET_FOUND, "PNO_NET_FOUND"},
		{WLC_E_PFN_NET_LOST, "PNO_NET_LOST"},
		{WLC_E_RESET_COMPLETE, "RESET_COMPLETE"},
		{WLC_E_JOIN_START, "JOIN_START"},
		{WLC_E_ROAM_START, "ROAM_START"},
		{WLC_E_ASSOC_START, "ASSOC_START"},
		{WLC_E_IBSS_ASSOC, "IBSS_ASSOC"},
		{WLC_E_RADIO, "RADIO"},
		{WLC_E_PSM_WATCHDOG, "PSM_WATCHDOG"},
		{WLC_E_PROBREQ_MSG, "PROBREQ_MSG"},
		{WLC_E_SCAN_CONFIRM_IND, "SCAN_CONFIRM_IND"},
		{WLC_E_PSK_SUP, "PSK_SUP"},
		{WLC_E_COUNTRY_CODE_CHANGED, "COUNTRY_CODE_CHANGED"},
		{WLC_E_EXCEEDED_MEDIUM_TIME, "EXCEEDED_MEDIUM_TIME"},
		{WLC_E_ICV_ERROR, "ICV_ERROR"},
		{WLC_E_UNICAST_DECODE_ERROR, "UNICAST_DECODE_ERROR"},
		{WLC_E_MULTICAST_DECODE_ERROR, "MULTICAST_DECODE_ERROR"},
		{WLC_E_TRACE, "TRACE"},
#ifdef PROP_TXSTATUS
		{WLC_E_FIFO_CREDIT_MAP, "FIFO CREDIT MAP"},
#endif
		{WLC_E_ACTION_FRAME, "ACTION FRAME"},
		{WLC_E_ACTION_FRAME_COMPLETE, "ACTION FRAME TX COMPLETE"},
		{WLC_E_IF, "IF"},
		{WLC_E_RSSI, "RSSI"},
		{WLC_E_PFN_SCAN_COMPLETE, "PFN_SCAN_COMPLETE"},
#if defined(DONGLEOVERLAYS)
		{WLC_E_OVERLAY_REQ, "OVERLAY_REQ"}
#endif
	};
	uint event_type, flags, auth_type, datalen;
	event_type = ntoh32(event->event_type);
	flags = ntoh16(event->flags);
	status = ntoh32(event->status);
	reason = ntoh32(event->reason);
	auth_type = ntoh32(event->auth_type);
	datalen = ntoh32(event->datalen);
	/* debug dump of event messages */
	sprintf(eabuf, "%02x:%02x:%02x:%02x:%02x:%02x",
	        (uchar)event->addr.octet[0]&0xff,
	        (uchar)event->addr.octet[1]&0xff,
	        (uchar)event->addr.octet[2]&0xff,
	        (uchar)event->addr.octet[3]&0xff,
	        (uchar)event->addr.octet[4]&0xff,
	        (uchar)event->addr.octet[5]&0xff);

	event_name = "UNKNOWN";
	for (i = 0; i < ARRAYSIZE(event_names); i++) {
		if (event_names[i].event == event_type)
			event_name = event_names[i].event_name;
	}

	DHD_EVENT(("EVENT: %s, event ID = %d\n", event_name, event_type));

	if (flags & WLC_EVENT_MSG_LINK)
		link = TRUE;
	if (flags & WLC_EVENT_MSG_GROUP)
		group = TRUE;
	if (flags & WLC_EVENT_MSG_FLUSHTXQ)
		flush_txq = TRUE;

	switch (event_type) {
	case WLC_E_START:
	case WLC_E_DEAUTH:
	case WLC_E_DISASSOC:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		break;

	case WLC_E_ASSOC:
	case WLC_E_REASSOC:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, SUCCESS\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, TIMEOUT\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, FAILURE, reason %d\n",
			       event_name, eabuf, (int)reason));
		} else {
			DHD_EVENT(("MACEVENT: %s, MAC %s, unexpected status %d\n",
			       event_name, eabuf, (int)status));
		}
		break;

	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
		DHD_EVENT(("MACEVENT: %s, MAC %s, reason %d\n", event_name, eabuf, (int)reason));
		break;

	case WLC_E_AUTH:
	case WLC_E_AUTH_IND:
		if (auth_type == DOT11_OPEN_SYSTEM)
			auth_str = "Open System";
		else if (auth_type == DOT11_SHARED_KEY)
			auth_str = "Shared Key";
		else {
			sprintf(err_msg, "AUTH unknown: %d", (int)auth_type);
			auth_str = err_msg;
		}
		if (event_type == WLC_E_AUTH_IND) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s\n", event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, SUCCESS\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, TIMEOUT\n",
				event_name, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, MAC %s, %s, FAILURE, reason %d\n",
			       event_name, eabuf, auth_str, (int)reason));
		}

		break;

	case WLC_E_JOIN:
	case WLC_E_ROAM:
	case WLC_E_SET_SSID:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, MAC %s\n", event_name, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, failed\n", event_name));
		} else if (status == WLC_E_STATUS_NO_NETWORKS) {
			DHD_EVENT(("MACEVENT: %s, no networks found\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, unexpected status %d\n",
				event_name, (int)status));
		}
		break;

	case WLC_E_BEACON_RX:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s, SUCCESS\n", event_name));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s, FAIL\n", event_name));
		} else {
			DHD_EVENT(("MACEVENT: %s, status %d\n", event_name, status));
		}
		break;

	case WLC_E_LINK:
		DHD_EVENT(("MACEVENT: %s %s\n", event_name, link?"UP":"DOWN"));
		break;

	case WLC_E_MIC_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s, Group %d, Flush %d\n",
		       event_name, eabuf, group, flush_txq));
		break;

	case WLC_E_ICV_ERROR:
	case WLC_E_UNICAST_DECODE_ERROR:
	case WLC_E_MULTICAST_DECODE_ERROR:
		DHD_EVENT(("MACEVENT: %s, MAC %s\n",
		       event_name, eabuf));
		break;

	case WLC_E_TXFAIL:
		DHD_EVENT(("MACEVENT: %s, RA %s\n", event_name, eabuf));
		break;

	case WLC_E_SCAN_COMPLETE:
	case WLC_E_PMKID_CACHE:
		DHD_EVENT(("MACEVENT: %s\n", event_name));
		break;

	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
	case WLC_E_PFN_SCAN_COMPLETE:
		DHD_EVENT(("PNOEVENT: %s\n", event_name));
		break;

	case WLC_E_PSK_SUP:
	case WLC_E_PRUNE:
		DHD_EVENT(("MACEVENT: %s, status %d, reason %d\n",
		           event_name, (int)status, (int)reason));
		break;

	case WLC_E_TRACE:
		{
			static uint32 seqnum_prev = 0;
			msgtrace_hdr_t hdr;
			uint32 nblost;
			char *s, *p;

			buf = (uchar *) event_data;
			memcpy(&hdr, buf, MSGTRACE_HDRLEN);

			if (hdr.version != MSGTRACE_VERSION) {
				printf("\nMACEVENT: %s [unsupported version --> "
				       "dhd version:%d dongle version:%d]\n",
				       event_name, MSGTRACE_VERSION, hdr.version);
				/* Reset datalen to avoid display below */
				datalen = 0;
				break;
			}

			/* There are 2 bytes available at the end of data */
			buf[MSGTRACE_HDRLEN + ntoh16(hdr.len)] = '\0';

			if (ntoh32(hdr.discarded_bytes) || ntoh32(hdr.discarded_printf)) {
				printf("\nWLC_E_TRACE: [Discarded traces in dongle -->"
				       "discarded_bytes %d discarded_printf %d]\n",
				       ntoh32(hdr.discarded_bytes), ntoh32(hdr.discarded_printf));
			}

			nblost = ntoh32(hdr.seqnum) - seqnum_prev - 1;
			if (nblost > 0) {
				printf("\nWLC_E_TRACE: [Event lost --> seqnum %d nblost %d\n",
				        ntoh32(hdr.seqnum), nblost);
			}
			seqnum_prev = ntoh32(hdr.seqnum);

			/* Display the trace buffer. Advance from \n to \n to avoid display big
			 * printf (issue with Linux printk )
			 */
			p = (char *)&buf[MSGTRACE_HDRLEN];
			while ((s = strstr(p, "\n")) != NULL) {
				*s = '\0';
				printf("%s\n", p);
				p = s + 1;
			}
			printf("%s\n", p);

			/* Reset datalen to avoid display below */
			datalen = 0;
		}
		break;


	case WLC_E_RSSI:
		DHD_EVENT(("MACEVENT: %s %d\n", event_name, ntoh32(*((int *)event_data))));
		break;

	default:
		DHD_EVENT(("MACEVENT: %s %d, MAC %s, status %d, reason %d, auth %d\n",
		       event_name, event_type, eabuf, (int)status, (int)reason,
		       (int)auth_type));
		break;
	}

	/* show any appended data */
	if (datalen) {
		prhex("MACEVENT: Appended data", event_data, datalen);
	}
}
#endif /* SHOW_EVENTS */

#if defined(WLP2P)
extern int dhd_use_p2p;
#define P2P_INTERFACE_NAME "p2p"
#endif /* defined(WLP2P) */

int
wl_host_event(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata,
              wl_event_msg_t *event, void **data_ptr)
{
	/* check whether packet is a BRCM event pkt */
	bcm_event_t *pvt_data = (bcm_event_t *)pktdata;
	char *event_data;
	uint32 type, status, reason, datalen;
	uint16 flags;
	int evlen;
#if defined(DONGLEOVERLAYS)
	dhd_cmn_t *cmn = dhd_pub->cmn;
#endif

	if (bcmp(BRCM_OUI, &pvt_data->bcm_hdr.oui[0], DOT11_OUI_LEN)) {
		DHD_ERROR(("%s: mismatched OUI, bailing\n", __FUNCTION__));
		return (BCME_ERROR);
	}

	/* BRCM event pkt may be unaligned - use xxx_ua to load user_subtype. */
	if (ntoh16_ua((void *)&pvt_data->bcm_hdr.usr_subtype) != BCMILCP_BCM_SUBTYPE_EVENT) {
		DHD_ERROR(("%s: mismatched subtype, bailing\n", __FUNCTION__));
		return (BCME_ERROR);
	}

	*data_ptr = &pvt_data[1];
	event_data = *data_ptr;

	/* memcpy since BRCM event pkt may be unaligned. */
	memcpy(event, &pvt_data->event, sizeof(wl_event_msg_t));

	type = ntoh32_ua((void *)&event->event_type);
	flags = ntoh16_ua((void *)&event->flags);
	status = ntoh32_ua((void *)&event->status);
	reason = ntoh32_ua((void *)&event->reason);
	datalen = ntoh32_ua((void *)&event->datalen);
	evlen = datalen + sizeof(bcm_event_t);

	switch (type) {
#ifdef PROP_TXSTATUS
		case WLC_E_FIFO_CREDIT_MAP:
			dhd_wlfc_FIFOcreditmap_event(dhd_pub->info, event_data);
			WLFC_DBGMESG(("WLC_E_FIFO_CREDIT_MAP:(AC0,AC1,AC2,AC3),(BC_MC),(OTHER): "
				"(%d,%d,%d,%d),(%d),(%d)\n", event_data[0], event_data[1],
				event_data[2],
				event_data[3], event_data[4], event_data[5]));
			break;
#endif

		case WLC_E_IF:
			{
				dhd_if_event_t *ifevent = (dhd_if_event_t *)event_data;
				DHD_TRACE(("%s: if event\n", __FUNCTION__));
#if defined(WLP2P)
             if (dhd_use_p2p) {
                if (strncmp(pvt_data->event.ifname,"wl",2) == 0) {
                     memmove(&(pvt_data->event.ifname[1]),&(pvt_data->event.ifname[0]),6);
                     memcpy(pvt_data->event.ifname,P2P_INTERFACE_NAME,strlen(P2P_INTERFACE_NAME));
					 memcpy(event, &pvt_data->event, sizeof(wl_event_msg_t));
                }
             }
#endif

#ifdef PROP_TXSTATUS
{
				uint8* ea = pvt_data->eth.ether_dhost;
				WLFC_DBGMESG(("WLC_E_IF: idx:%d, action:%s, iftype:%s, "
					"[%02x:%02x:%02x:%02x:%02x:%02x]\n",
					ifevent->ifidx,
					((ifevent->action == WLC_E_IF_ADD) ? "ADD":"DEL"),
					((ifevent->is_AP == 0) ? "STA":"AP "),
					ea[0], ea[1], ea[2], ea[3], ea[4], ea[5]));
				(void)ea;

				dhd_wlfc_interface_event(dhd_pub->info,
					((ifevent->action == WLC_E_IF_ADD) ?
					eWLFC_MAC_ENTRY_ACTION_ADD : eWLFC_MAC_ENTRY_ACTION_DEL),
					ifevent->ifidx, ifevent->is_AP, ea);

				/* dhd already has created an interface by deafult, for 0 */
				if (ifevent->ifidx == 0)
					break;
}
#endif /* PROP_TXSTATUS */
				if (ifevent->ifidx > 0 && ifevent->ifidx < DHD_MAX_IFS) {
					if (ifevent->action == WLC_E_IF_ADD)
						dhd_add_if(dhd_pub->info, ifevent->ifidx,
							NULL, event->ifname, event->addr.octet,
							ifevent->flags, ifevent->bssidx);
					else
						dhd_del_if(dhd_pub->info, ifevent->ifidx);
				} else {
#ifndef PROP_TXSTATUS
					DHD_ERROR(("%s: Invalid ifidx %d for %s\n",
						__FUNCTION__, ifevent->ifidx, event->ifname));
#endif
				}
			}
			/* send up the if event: btamp user needs it */
			*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
			/* push up to external supp/auth */
			dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
			break;

#if defined(DONGLEOVERLAYS)
		case WLC_E_OVERLAY_REQ:
		{
			int datalen = ntoh32(event->datalen);
			void *p = NULL;

			switch (reason) {
			case WLC_E_OVL_DOWNLOAD:
				if (!pktq_full(&cmn->overlay_req)) {
					/* packetize the data to place it on a work queue */
					p = PKTGET(dhd_pub->osh, sizeof(wl_event_msg_t) + datalen,
					           TRUE);
				} else {
					DHD_ERROR(("WLC_E_OVERLAY_REQ: pkt queue full!\n"));
					break;
				}

				if (!p)
					DHD_ERROR(("WLC_E_OVERLAY_REQ: pkt malloc failure\n"));
				else {
					unsigned long flags;
					char *cp = (char *)PKTDATA(dhd_pub->osh, p);
					memcpy(cp, &pvt_data->event, sizeof(wl_event_msg_t));
					memcpy(cp + sizeof(wl_event_msg_t), event_data, datalen);

					spin_lock_irqsave(&cmn->lock, flags);
					pktenq(&cmn->overlay_req, p);
					spin_unlock_irqrestore(&cmn->lock, flags);

					/* signal the thread to do the work */
					up(&cmn->sysioc_sem);
				}
				break;
			case WLC_E_OVL_UPDATE_IND:
				{
					uint32 offset = ntoh32_ua((void *)event_data);
					uint32 cmd = ntoh32_ua((void *)(event_data +
					                                sizeof(uint32)));
					uint32 idx = ntoh32_ua((void *)(event_data +
					                                sizeof(uint32) * 2));
					char *buf = (char *)(event_data +
					                     sizeof(uint32) * 3);
					uint8 *overlay;
					int osize;

					overlay = dhd_find_overlay(dhd_pub, cmd, buf, &idx,
					                           &osize, FALSE);
					if (overlay) {
						if (cmd == WLC_SET_VAR || cmd == WLC_GET_VAR)
							buf += strlen(buf) + 1;
						datalen -= (uint32)(buf - event_data);
						memcpy(overlay + offset, buf, datalen);
					} else {
						DHD_ERROR(("OVL_UPDATE_IND: overlay not found\n"));
					}
				}
				break;
			default:
				DHD_ERROR(("OVERLAY_REQ: reason code %d not found\n",
				           reason));
				break;
			}
			break;
		}
#endif /* DONGLEOVERLAYS */

		case WLC_E_NDIS_LINK:
		{
			uint32 temp = hton32(WLC_E_LINK);

			memcpy((void *)(&pvt_data->event.event_type), &temp,
				sizeof(pvt_data->event.event_type));
		}
		/* fall through */
		/* These are what external supplicant/authenticator wants */
		case WLC_E_LINK:
		case WLC_E_ASSOC_IND:
		case WLC_E_REASSOC_IND:
		case WLC_E_DISASSOC_IND:
		case WLC_E_MIC_ERROR:
		default:
		/* Fall through: this should get _everything_  */

			*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
			/* push up to external supp/auth */
			dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
			DHD_TRACE(("%s: MAC event %d, flags %x, status %x\n",
			           __FUNCTION__, type, flags, status));

			/* put it back to WLC_E_NDIS_LINK */
			if (type == WLC_E_NDIS_LINK) {
				uint32 temp;

				temp = ntoh32_ua((void *)&event->event_type);
				DHD_TRACE(("Converted to WLC_E_LINK type %d\n", temp));

				temp = ntoh32(WLC_E_NDIS_LINK);
				memcpy((void *)(&pvt_data->event.event_type), &temp,
					sizeof(pvt_data->event.event_type));
			}
			break;
	}

#ifdef SHOW_EVENTS
	wl_show_host_event(event, event_data);
#endif /* SHOW_EVENTS */

	return (BCME_OK);
}

void
wl_event_to_host_order(wl_event_msg_t *evt)
{
	/* Event struct members passed from dongle to host are stored in network
	 * byte order. Convert all members to host-order.
	 */
	evt->event_type = ntoh32(evt->event_type);
	evt->flags = ntoh16(evt->flags);
	evt->status = ntoh32(evt->status);
	evt->reason = ntoh32(evt->reason);
	evt->auth_type = ntoh32(evt->auth_type);
	evt->datalen = ntoh32(evt->datalen);
	evt->version = ntoh16(evt->version);
}

void print_buf(void *pbuf, int len, int bytes_per_line)
{
	int i, j = 0;
	unsigned char *buf = pbuf;

	if (bytes_per_line == 0) {
		bytes_per_line = len;
	}

	for (i = 0; i < len; i++) {
		printf("%2.2x", *buf++);
		j++;
		if (j == bytes_per_line) {
			printf("\n");
			j = 0;
		} else {
			printf(":");
		}
	}
	printf("\n");
}

#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))

/* Convert user's input in hex pattern to byte-size mask */
static int
wl_pattern_atoh(char *src, char *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 &&
	    strncmp(src, "0X", 2) != 0) {
		DHD_ERROR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + 2; /* Skip past 0x */
	if (strlen(src) % 2 != 0) {
		DHD_ERROR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[3];
		strncpy(num, src, 2);
		num[2] = '\0';
		dst[i] = (uint8)strtoul(num, NULL, 16);
		src += 2;
	}
	return i;
}

extern int dhd_preinit_ioctls(dhd_pub_t *dhd);

void
dhd_pktfilter_offload_enable(dhd_pub_t * dhd, char *arg, int enable, int master_mode)
{
	char				*argv[8];
	int					i = 0;
	const char 			*str;
	int					buf_len;
	int					str_len;
	char				*arg_save = 0, *arg_org = 0;
	int					rc;
	char				buf[128];
	wl_pkt_filter_enable_t	enable_parm;
	wl_pkt_filter_enable_t	* pkt_filterp;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}
	arg_org = arg_save;
	memcpy(arg_save, arg, strlen(arg) + 1);

	argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (NULL == argv[i]) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_enable";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[str_len] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_enable_t *)(buf + str_len + 1);

	/* Parse packet filter id. */
	enable_parm.id = htod32(strtoul(argv[i], NULL, 0));

	/* Parse enable/disable value. */
	enable_parm.enable = htod32(enable);

	buf_len += sizeof(enable_parm);
	memcpy((char *)pkt_filterp,
	       &enable_parm,
	       sizeof(enable_parm));

	/* Enable/disable the specified filter. */
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

	/* Contorl the master mode */
	bcm_mkiovar("pkt_filter_mode", (char *)&master_mode, 4, buf, sizeof(buf));
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, sizeof(buf), TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);
}

void
dhd_pktfilter_offload_set(dhd_pub_t * dhd, char *arg)
{
	const char 			*str;
	wl_pkt_filter_t		pkt_filter;
	wl_pkt_filter_t		*pkt_filterp;
	int					buf_len;
	int					str_len;
	int 				rc;
	uint32				mask_size;
	uint32				pattern_size;
	char				*argv[8], * buf = 0;
	int					i = 0;
	char				*arg_save = 0, *arg_org = 0;
#define BUF_SIZE		2048

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}

	arg_org = arg_save;

	if (!(buf = MALLOC(dhd->osh, BUF_SIZE))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}

	memcpy(arg_save, arg, strlen(arg) + 1);

	if (strlen(arg) > BUF_SIZE) {
		DHD_ERROR(("Not enough buffer %d < %d\n", (int)strlen(arg), (int)sizeof(buf)));
		goto fail;
	}

	argv[i] = bcmstrtok(&arg_save, " ", 0);
	while (argv[i++])
		argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (NULL == argv[i]) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_add";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	/* Parse packet filter id. */
	pkt_filter.id = htod32(strtoul(argv[i], NULL, 0));

	if (NULL == argv[++i]) {
		DHD_ERROR(("Polarity not provided\n"));
		goto fail;
	}

	/* Parse filter polarity. */
	pkt_filter.negate_match = htod32(strtoul(argv[i], NULL, 0));

	if (NULL == argv[++i]) {
		DHD_ERROR(("Filter type not provided\n"));
		goto fail;
	}

	/* Parse filter type. */
	pkt_filter.type = htod32(strtoul(argv[i], NULL, 0));

	if (NULL == argv[++i]) {
		DHD_ERROR(("Offset not provided\n"));
		goto fail;
	}

	/* Parse pattern filter offset. */
	pkt_filter.u.pattern.offset = htod32(strtoul(argv[i], NULL, 0));

	if (NULL == argv[++i]) {
		DHD_ERROR(("Bitmask not provided\n"));
		goto fail;
	}

	/* Parse pattern filter mask. */
	mask_size =
		htod32(wl_pattern_atoh(argv[i], (char *) pkt_filterp->u.pattern.mask_and_pattern));

	if (NULL == argv[++i]) {
		DHD_ERROR(("Pattern not provided\n"));
		goto fail;
	}

	/* Parse pattern filter pattern. */
	pattern_size =
		htod32(wl_pattern_atoh(argv[i],
	         (char *) &pkt_filterp->u.pattern.mask_and_pattern[mask_size]));

	if (mask_size != pattern_size) {
		DHD_ERROR(("Mask and pattern not the same size\n"));
		goto fail;
	}

	pkt_filter.u.pattern.size_bytes = mask_size;
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);

	/* Keep-alive attributes are set in local	variable (keep_alive_pkt), and
	** then memcpy'ed into buffer (keep_alive_pktp) since there is no
	** guarantee that the buffer is properly aligned.
	*/
	memcpy((char *)pkt_filterp,
	       &pkt_filter,
	       WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);

	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;

	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		__FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		__FUNCTION__, arg));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);

	if (buf)
		MFREE(dhd->osh, buf, BUF_SIZE);
}

void
dhd_arp_offload_set(dhd_pub_t * dhd, int arp_mode)
{
	char iovbuf[32];
	int retcode;

	bcm_mkiovar("arp_ol", (char *)&arp_mode, 4, iovbuf, sizeof(iovbuf));
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to set ARP offload mode to 0x%x, retcode = %d\n",
		__FUNCTION__, arp_mode, retcode));
	else
		DHD_TRACE(("%s: successfully set ARP offload mode to 0x%x\n",
		__FUNCTION__, arp_mode));
}

void
dhd_arp_offload_enable(dhd_pub_t * dhd, int arp_enable)
{
	char iovbuf[32];
	int retcode;

	bcm_mkiovar("arpoe", (char *)&arp_enable, 4, iovbuf, sizeof(iovbuf));
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, sizeof(iovbuf), TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_TRACE(("%s: failed to enabe ARP offload to %d, retcode = %d\n",
		__FUNCTION__, arp_enable, retcode));
	else
		DHD_TRACE(("%s: successfully enabed ARP offload to %d\n",
		__FUNCTION__, arp_enable));
}

/* send up locally generated event */
void
dhd_sendup_event_common(dhd_pub_t *dhdp, wl_event_msg_t *event, void *data)
{
	switch (ntoh32(event->event_type)) {

	default:
		break;
	}

	/* Call per-port handler. */
	dhd_sendup_event(dhdp, event, data);
}

#if defined(DONGLEOVERLAYS)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define DAEMONIZE(a) daemonize(a); \
	allow_signal(SIGKILL); \
	allow_signal(SIGTERM);
#else /* Linux 2.4 (w/o preemption patch) */
#define RAISE_RX_SOFTIRQ() \
	cpu_raise_softirq(smp_processor_id(), NET_RX_SOFTIRQ)
#define DAEMONIZE(a) daemonize(); \
	do { if (a) \
		strncpy(current->comm, a, MIN(sizeof(current->comm), (strlen(a) + 1))); \
	} while (0);
#endif /* LINUX_VERSION_CODE  */

static void *
pktdeq_safe(dhd_cmn_t *cmn)
{
	void *p;
	unsigned long flags;

	spin_lock_irqsave(&cmn->lock, flags);
	p = pktdeq(&cmn->overlay_req);
	spin_unlock_irqrestore(&cmn->lock, flags);

	return p;
}

static int
_overlay_req_sysioc_thread(void *data)
{
	dhd_cmn_t *cmn = (dhd_cmn_t *)data;
	wl_ioctl_t ioc;
	bool found;
	uint32 idx;
	char *buf;
	wl_event_msg_t event;
	char *event_data;
	int datalen, ret = BCME_OK;
	void *p;
	char *cp;

	DAEMONIZE("overlay_req_sysioc");

	while (down_interruptible(&cmn->sysioc_sem) == 0) {
		/* signals can accumulate (one goes with each packet) */
		if (pktq_empty(&cmn->overlay_req) || cmn->overlay_thread_terminate)
			continue;

		while ((p = pktdeq_safe(cmn))) {
			cp = (char *)PKTDATA(cmn->osh, p);

			event_data = cp + sizeof(wl_event_msg_t);

			/* memcpy since BRCM event pkt may be unaligned. */
			memcpy(&event, cp, sizeof(wl_event_msg_t));

			ioc.cmd = ntoh32_ua((void*)event_data);
			idx = ntoh32_ua(((void*)(event_data + sizeof(uint32))));
			buf = event_data + (sizeof(uint32) * 2);
			ioc.buf = buf;
			datalen = ntoh32(event.datalen);
			ioc.len = datalen - (sizeof(uint32) * 2);
			ioc.action = WL_IOCTL_ACTION_SET;

			DHD_TRACE(("%s: WLC_E_OVERLAY_REQ cmd %d, idx %d: %s\n",
			           __FUNCTION__, ioc.cmd, idx,
			           bcm_isalnum(*buf) ? buf : ""));

			ASSERT(cmn->dhd);
			/* serialize here since overlays entail an initial overlay memory write */
			dhd_os_proto_block(cmn->dhd);
			ret = dhd_wl_ioctl_overlay(cmn->dhd, 0, &ioc, buf, ioc.len, idx, &found);
			dhd_os_proto_unblock(cmn->dhd);
			DHD_ERROR(("%s: WLC_E_OVERLAY_REQ cmd %d, idx %d: %s done\n",
			           __FUNCTION__, ioc.cmd, idx,
			           bcm_isalnum(*buf) ? buf : ""));
			if (ret) {
				DHD_ERROR(("%s: OVERLAY_REQ failed with status %d; ioctl %d %s\n",
				           __FUNCTION__, ret, ioc.cmd,
				           bcm_isalnum(*buf) ? buf : ""));
			}
			PKTFREE(cmn->osh, p, TRUE);
		}
	}

	/* clean up */
	while ((p = pktdeq_safe(cmn)))
		PKTFREE(cmn->osh, p, TRUE);

	complete_and_exit(&cmn->sysioc_exited, 0);
	DHD_TRACE(("%s: complete_and_exit\n", __FUNCTION__));
	return 0;
}
#endif /* DONGLEOVERLAYS */


#ifdef SIMPLE_ISCAN

uint iscan_thread_id;
iscan_buf_t * iscan_chain = 0;

iscan_buf_t *
dhd_iscan_allocate_buf(dhd_pub_t *dhd, iscan_buf_t **iscanbuf)
{
	iscan_buf_t *iscanbuf_alloc = 0;
	iscan_buf_t *iscanbuf_head;

	DHD_TRACE(("%s: Entered\n", __FUNCTION__));
	dhd_iscan_lock();

	iscanbuf_alloc = (iscan_buf_t*)MALLOC(dhd->osh, sizeof(iscan_buf_t));
	if (iscanbuf_alloc == NULL)
		goto fail;

	iscanbuf_alloc->next = NULL;
	iscanbuf_head = *iscanbuf;

	DHD_ISCAN(("%s: addr of allocated node = 0x%X"
		   "addr of iscanbuf_head = 0x%X dhd = 0x%X\n",
		   __FUNCTION__, iscanbuf_alloc, iscanbuf_head, dhd));

	if (iscanbuf_head == NULL) {
		*iscanbuf = iscanbuf_alloc;
		DHD_ISCAN(("%s: Head is allocated\n", __FUNCTION__));
		goto fail;
	}

	while (iscanbuf_head->next)
		iscanbuf_head = iscanbuf_head->next;

	iscanbuf_head->next = iscanbuf_alloc;

fail:
	dhd_iscan_unlock();
	return iscanbuf_alloc;
}

void
dhd_iscan_free_buf(void *dhdp, iscan_buf_t *iscan_delete)
{
	iscan_buf_t *iscanbuf_free = 0;
	iscan_buf_t *iscanbuf_prv = 0;
	iscan_buf_t *iscanbuf_cur;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	DHD_TRACE(("%s: Entered\n", __FUNCTION__));

	dhd_iscan_lock();

	iscanbuf_cur = iscan_chain;

	/* If iscan_delete is null then delete the entire 
	 * chain or else delete specific one provided
	 */
	if (!iscan_delete) {
		while (iscanbuf_cur) {
			iscanbuf_free = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
			iscanbuf_free->next = 0;
			MFREE(dhd->osh, iscanbuf_free, sizeof(iscan_buf_t));
		}
		iscan_chain = 0;
	} else {
		while (iscanbuf_cur) {
			if (iscanbuf_cur == iscan_delete)
				break;
			iscanbuf_prv = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
		}
		if (iscanbuf_prv)
			iscanbuf_prv->next = iscan_delete->next;

		iscan_delete->next = 0;
		MFREE(dhd->osh, iscan_delete, sizeof(iscan_buf_t));

		if (!iscanbuf_prv)
			iscan_chain = 0;
	}
	dhd_iscan_unlock();
}

iscan_buf_t *
dhd_iscan_result_buf(void)
{
	return iscan_chain;
}

/*
* delete disappeared AP from specific scan cache
*/
int
dhd_iscan_delete_bss(/* TBD void *dhdp, */ void *addr)
{
	int i = 0, j = 0, l = 0;
	iscan_buf_t *iscan_cur;
	wl_iscan_results_t *list;
	wl_scan_results_t *results;
	wl_bss_info_t UNALIGNED *bi, *bi_new, *bi_next;

	uchar *s_addr = addr;
	DHD_TRACE(("%s: Entered\n", __FUNCTION__));

	dhd_iscan_lock();
	DHD_TRACE(("%s: BSS to remove %X:%X:%X:%X:%X:%X\n",
	__FUNCTION__, s_addr[0], s_addr[1], s_addr[2],
	s_addr[3], s_addr[4], s_addr[5]));

	DHD_TRACE(("%s: Scan cache before delete\n",
	__FUNCTION__));

	iscan_cur = dhd_iscan_result_buf();

	while (iscan_cur) {
			list = (wl_iscan_results_t *)iscan_cur->iscan_buf;
			if (!list)
				break;

			results = (wl_scan_results_t *)&list->results;
			if (!results)
				break;

			if (results->version != WL_BSS_INFO_VERSION) {
			DHD_ERROR(("%s: results->version %d != WL_BSS_INFO_VERSION\n",
				__FUNCTION__, results->version));
				goto done;
			}

			bi = results->bss_info;
			for (i = 0; i < results->count; i++) {
				if (!bi)
					break;

				if (!memcmp(bi->BSSID.octet, addr, ETHER_ADDR_LEN)) {
					DHD_TRACE(("%s: Del BSS[%2.2d:%2.2d] %X:%X:%X:%X:%X:%X\n",
					__FUNCTION__, l, i, bi->BSSID.octet[0], bi->BSSID.octet[1],
					bi->BSSID.octet[2], bi->BSSID.octet[3], bi->BSSID.octet[4],
					bi->BSSID.octet[5]));

					bi_new = bi;
					bi = (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length));


					for (j = i; j < results->count; j++) {
					DHD_TRACE(("%s: Moved up BSS[%2.2d:%2.2d]"
					" %X:%X:%X:%X:%X:%X\n",
					__FUNCTION__, l, j, bi->BSSID.octet[0], bi->BSSID.octet[1],
					bi->BSSID.octet[2], bi->BSSID.octet[3], bi->BSSID.octet[4],
					bi->BSSID.octet[5]));

					bi_next = (wl_bss_info_t *)((uintptr)bi +
						dtoh32(bi->length));
							bcopy(bi, bi_new, dtoh32(bi->length));
					bi_new = (wl_bss_info_t *)((uintptr)bi_new +
						dtoh32(bi_new->length));
							bi = bi_next;
						}
				results->count--;
					if (results->count == 0) {
						/* Prune now empty partial scan list */
						goto done;
					}
					break;
				}

				bi = (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length));
			}
		iscan_cur = iscan_cur->next;
		l++;
	}

done:
	DHD_TRACE(("%s: Scan cache after delete\n",
		__FUNCTION__));
	dhd_iscan_unlock();
	return 0;
}

int
dhd_iscan_request(void * dhdp, uint16 action)
{
	int rc;
	wl_iscan_params_t params;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	char buf[WLC_IOCTL_SMLEN];

	DHD_TRACE(("%s: Entered\n", __FUNCTION__));

	memset(&params, 0, sizeof(wl_iscan_params_t));
	memcpy(&params.params.bssid, &ether_bcast, ETHER_ADDR_LEN);

	params.params.bss_type = DOT11_BSSTYPE_ANY;
	params.params.scan_type = DOT11_SCANTYPE_ACTIVE;

	params.params.nprobes = htod32(-1);
	params.params.active_time = htod32(-1);
	params.params.passive_time = htod32(-1);
	params.params.home_time = htod32(-1);
	params.params.channel_num = htod32(0);

	params.version = htod32(ISCAN_REQ_VERSION);
	params.action = htod16(action);
	params.scan_duration = htod16(0);

	bcm_mkiovar("iscan", (char *)&params, sizeof(wl_iscan_params_t), buf, WLC_IOCTL_SMLEN);
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, WLC_IOCTL_SMLEN, TRUE, 0);

	return rc;
}

static int
dhd_iscan_get_partial_result(void *dhdp, uint *scan_count)
{
	wl_iscan_results_t *list_buf;
	wl_iscan_results_t list;
	wl_scan_results_t *results;
	iscan_buf_t *iscan_cur;
	int status = -1;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	int rc;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	iscan_cur = dhd_iscan_allocate_buf(dhd, &iscan_chain);
	if (!iscan_cur) {
		DHD_ERROR(("%s: Failed to allocate node\n", __FUNCTION__));
		dhd_iscan_free_buf(dhdp, 0);
		dhd_iscan_request(dhdp, WL_SCAN_ACTION_ABORT);
		dhd_ind_scan_confirm(dhdp, FALSE);
		goto fail;
	}

	dhd_iscan_lock();

	memset(iscan_cur->iscan_buf, 0, WLC_IW_ISCAN_MAXLEN);
	list_buf = (wl_iscan_results_t*)iscan_cur->iscan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WLC_IW_ISCAN_MAXLEN);
	bcm_mkiovar("iscanresults", (char *)&list, WL_ISCAN_RESULTS_FIXED_SIZE,
		iscan_cur->iscan_buf, WLC_IW_ISCAN_MAXLEN);
	rc = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, iscan_cur->iscan_buf,
	                      WLC_IW_ISCAN_MAXLEN, FALSE, 0);

	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	*scan_count = results->count = dtoh32(results->count);
	status = dtoh32(list_buf->status);
	DHD_TRACE(("%s: Got %d resuls\n", __FUNCTION__, results->count));

	dhd_iscan_unlock();

	if (!(*scan_count)) {
		dhd_iscan_free_buf(dhdp, iscan_cur);
	}
fail:
	return status;
}

#endif 

/*
 * returns = TRUE if associated, FALSE if not associated
 */
bool dhd_is_associated(dhd_pub_t *dhd, void *bss_buf)
{
	char bssid[6], zbuf[6];
	int ret = -1;

	bzero(bssid, 6);
	bzero(zbuf, 6);

	ret  = dhd_wl_ioctl_cmd(dhd, WLC_GET_BSSID, (char *)&bssid, ETHER_ADDR_LEN, FALSE, 0);
	DHD_TRACE((" %s WLC_GET_BSSID ioctl res = %d\n", __FUNCTION__, ret));

	if (ret == BCME_NOTASSOCIATED) {
		DHD_TRACE(("%s: not associated! res:%d\n", __FUNCTION__, ret));
	}

	if (ret < 0)
		return FALSE;

	if ((memcmp(bssid, zbuf, ETHER_ADDR_LEN) != 0)) {
		/*  STA is assocoated BSSID is non zero */

		if (bss_buf) {
			/* return bss if caller provided buf */
			memcpy(bss_buf, bssid, ETHER_ADDR_LEN);
		}
		return TRUE;
	} else {
		DHD_TRACE(("%s: WLC_GET_BSSID ioctl returned zero bssid\n", __FUNCTION__));
		return FALSE;
	}
}


/* Function to estimate possible DTIM_SKIP value */
int
dhd_get_dtim_skip(dhd_pub_t *dhd)
{
	int bcn_li_dtim;
	int ret = -1;
	int dtim_assoc = 0;

	if ((dhd->dtim_skip == 0) || (dhd->dtim_skip == 1))
		bcn_li_dtim = 3;
	else
		bcn_li_dtim = dhd->dtim_skip;

	/* Check if associated */
	if (dhd_is_associated(dhd, NULL) == FALSE) {
		DHD_ERROR(("%s NOT assoc ret %d\n", __FUNCTION__, ret));
		goto exit;
	}

	/* if assoc grab ap's dtim value */
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_DTIMPRD,
		&dtim_assoc, sizeof(dtim_assoc), FALSE, 0)) < 0) {
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	DHD_ERROR(("%s bcn_li_dtim=%d DTIM=%d Listen=%d\n",
		__FUNCTION__, bcn_li_dtim, dtim_assoc, LISTEN_INTERVAL));

	/* if not assocated just eixt */
	if (dtim_assoc == 0) {
		goto exit;
	}

	/* check if sta listen interval fits into AP dtim */
	if (dtim_assoc > LISTEN_INTERVAL) {
		/* AP DTIM to big for our Listen Interval : no dtim skiping */
		bcn_li_dtim = 1;
		DHD_ERROR(("%s DTIM=%d > Listen=%d : too big ...\n",
			__FUNCTION__, dtim_assoc, LISTEN_INTERVAL));
		goto exit;
	}

	if ((bcn_li_dtim * dtim_assoc) > LISTEN_INTERVAL) {
		/* Round up dtim_skip to fit into STAs Listen Interval */
		bcn_li_dtim = (int)(LISTEN_INTERVAL / dtim_assoc);
		DHD_TRACE(("%s agjust dtim_skip as %d\n", __FUNCTION__, bcn_li_dtim));
	}

exit:
	return bcn_li_dtim;
}
