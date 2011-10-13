/*
 * arch/arm/mach-tegra/nvrm_user.c
 *
 * User-land access to NvRm APIs
 *
 * Copyright (c) 2008-2010, NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/percpu.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_memmgr.h"
#include "nvrm_dma.h"
#include "nvrm_i2c.h"
#include "nvrm_ioctls.h"
#include "nvrm_power_private.h"
#include "mach/nvrm_linux.h"
#include "nvos_ioctl.h"
#include "nvreftrack.h"
#include "board.h"

NvRmDeviceHandle s_hRmGlobal = NULL;
pid_t s_nvrm_daemon_pid = 0;

NvError NvRm_Dispatch(void *InBuffer,
                      NvU32 InSize,
                      void *OutBuffer,
                      NvU32 OutSize,
                      NvDispatchCtx* Ctx);
NvError NvRm_Dispatch_Others(void *InBuffer,
                      NvU32 InSize,
                      void *OutBuffer,
                      NvU32 OutSize,
                      NvDispatchCtx* Ctx);

static int nvrm_open(struct inode *inode, struct file *file);
static int nvrm_close(struct inode *inode, struct file *file);
static long nvrm_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg);
static int nvrm_mmap(struct file *file, struct vm_area_struct *vma);
extern void reset_cpu(unsigned int cpu, unsigned int reset);
extern void NvRmPrivDvsStop(void);
extern void NvRmPrivDvsRun(void);

//Variables for AVP suspend operation
extern NvRmDeviceHandle s_hRmGlobal;
extern NvRmPrivLockSharedPll();
extern NvRmPrivUnlockSharedPll();
static NvRtHandle s_RtHandle = NULL;

#define DEVICE_NAME "nvrm"

static const struct file_operations nvrm_fops =
{
    .owner = THIS_MODULE,
    .open = nvrm_open,
    .release = nvrm_close,
    .unlocked_ioctl = nvrm_unlocked_ioctl,
    .mmap = nvrm_mmap
};

static struct miscdevice nvrm_dev =
{
    .name = DEVICE_NAME,
    .fops = &nvrm_fops,
    .minor = MISC_DYNAMIC_MINOR,
};

static const struct file_operations knvrm_fops =
{
    .owner = THIS_MODULE,
    .open = nvrm_open,
    .release = nvrm_close,
    .unlocked_ioctl = nvrm_unlocked_ioctl,
    .mmap = nvrm_mmap
};

static struct miscdevice knvrm_dev =
{
    .name = "knvrm",
    .fops = &knvrm_fops,
    .minor = MISC_DYNAMIC_MINOR,
};


struct nvrm_file_priv {
    NvRtClientHandle rt_client;
    bool su;
};

static void client_detach(NvRtClientHandle client)
{
    void *ptr;

    if (NvRtUnregisterClient(s_RtHandle, client))
    {
        NvDispatchCtx dctx;

        dctx.Rt = s_RtHandle;
        dctx.Client = client;
        dctx.PackageIdx = 0;

        for (;;)
        {

            ptr = NvRtFreeObjRef(&dctx,
                                 NvRtObjType_NvRm_GpioHandle,
                                 NULL);
            if (!ptr) break;
            NVRT_LEAK("NvRm", "GpioHandle", (NvU32)ptr);
            NvRmGpioReleasePinHandles((NvRmGpioHandle)s_hRmGlobal, (NvRmGpioPinHandle *)&ptr, 1);
        }

        for (;;)
        {
            ptr = NvRtFreeObjRef(&dctx,
                                 NvRtObjType_NvRm_NvRmMemHandle,
                                 NULL);
            if (!ptr) break;
            NVRT_LEAK("NvRm", "NvRmMemHandle", (NvU32)ptr);
            NvRmMemHandleFree(ptr);
        }

        for(;;)
        {
            ptr = NvRtFreeObjRef(&dctx,
                                 NvRtObjType_NvRm_NvRmI2cHandle,
                                 NULL);
            if (!ptr) break;
            NVRT_LEAK("NvRm", "NvRmI2cHandle", (NvU32)ptr);
            NvRmI2cClose((NvRmI2cHandle)ptr);
        }

        for(;;)
        {
            ptr = NvRtFreeObjRef(&dctx,
                                 NvRtObjType_NvRm_NvRmDmaHandle,
                                 NULL);
            if(!ptr) break;
            NVRT_LEAK("NvRm", "NvRmDmaHandle", (NvU32)ptr);
            NvRmDmaFree(ptr);
        }

        for(;;)
        {
            NvRmExternalClockObj *obj = NULL;
            ptr = NvRtFreeObjRef(&dctx,
                                 NvRtObjType_NvRm_PinmuxClkHandle,
                                 NULL);
            if(!ptr) break;
            NVRT_LEAK("NvRm", "PinmuxClkHandle", (NvU32)ptr);

            obj = (NvRmExternalClockObj *)ptr;
            NvRmExternalClockConfig(g_NvRmHandle,
                                    NvOdmIoModule_ExternalClock,
                                    obj->Instance,
                                    obj->Config,
                                    NV_TRUE);
            /*
             * this "obj" is a static struct in nvrm_pinmux_dispatch.c
             * reset Instance number so it can be reused for next clock
             * config
             */
            obj->Instance = NVRM_EXT_CLK_CNT;
        }

        NvRtUnregisterClient(s_RtHandle, client);
    }
}

int nvrm_open(struct inode *inode, struct file *file)
{
    struct nvrm_file_priv *priv;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv) return -ENOMEM;

    if (NvRtRegisterClient(s_RtHandle, &priv->rt_client) != NvSuccess)
    {
        return -ENOMEM;
    }

    priv->su = (file->f_op == &knvrm_fops);
    file->private_data = priv;

    return 0;
}

int nvrm_close(struct inode *inode, struct file *file)
{
    struct nvrm_file_priv *priv = file->private_data;

    client_detach(priv->rt_client);
    kfree(priv);
    return 0;
}

long nvrm_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    NvError err;
    NvOsIoctlParams p;
    NvU32 size;
    NvU32 small_buf[8];
    void *ptr = 0;
    long e;
    NvBool bAlloc = NV_FALSE;
    struct nvrm_file_priv *priv = file->private_data;

    switch( cmd ) {
    case NvRmIoctls_Generic:
    {
        NvDispatchCtx dctx;

        dctx.Rt         = s_RtHandle;
        dctx.Client     = priv->rt_client;
        dctx.PackageIdx = 0;

        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: copy in failed\n" );
            goto fail;
        }

        //printk( "NvRmIoctls_Generic: %d %d %d\n", p.InBufferSize,
        //    p.InOutBufferSize, p.OutBufferSize );

        size = p.InBufferSize + p.InOutBufferSize + p.OutBufferSize;
        if( size <= sizeof(small_buf) )
        {
            ptr = small_buf;
        }
        else
        {
            ptr = NvOsAlloc( size );
            if( !ptr )
            {
                printk( "NvRmIoctls_Generic: alloc failure (%d bytes)\n",
                    size );
                goto fail;
            }

            bAlloc = NV_TRUE;
        }

        err = NvOsCopyIn( ptr, p.pBuffer, p.InBufferSize +
            p.InOutBufferSize );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: copy in failure\n" );
            goto fail;
        }

        if (priv->su) {
            err = NvRm_Dispatch( ptr, p.InBufferSize + p.InOutBufferSize,
                ((NvU8 *)ptr) + p.InBufferSize, p.InOutBufferSize +
                p.OutBufferSize, &dctx );
        } else {
            err = NvRm_Dispatch_Others( ptr, p.InBufferSize + p.InOutBufferSize,
                ((NvU8 *)ptr) + p.InBufferSize, p.InOutBufferSize +
                p.OutBufferSize, &dctx );
        }
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: dispatch failure\n" );
            goto fail;
        }

        if( p.InOutBufferSize || p.OutBufferSize )
        {
            err = NvOsCopyOut( ((NvU8 *)((NvOsIoctlParams *)arg)->pBuffer)
                + p.InBufferSize,
                ((NvU8 *)ptr) + p.InBufferSize,
                p.InOutBufferSize + p.OutBufferSize );
            if( err != NvSuccess )
            {
                printk( "NvRmIoctls_Generic: copy out failure\n" );
                goto fail;
            }
        }

        break;
    }
    case NvRmIoctls_NvRmGraphics:
        printk( "NvRmIoctls_NvRmGraphics: not supported\n" );
        goto fail;
    case NvRmIoctls_NvRmFbControl:
        printk( "NvRmIoctls_NvRmFbControl: deprecated \n" );
        break;

    case NvRmIoctls_NvRmMemRead:
    case NvRmIoctls_NvRmMemWrite:
    case NvRmIoctls_NvRmMemReadStrided:
    case NvRmIoctls_NvRmGetCarveoutInfo:
    case NvRmIoctls_NvRmMemWriteStrided:
        goto fail;

    case NvRmIoctls_NvRmMemMapIntoCallerPtr:
        // FIXME: implement?
        printk( "NvRmIoctls_NvRmMemMapIntoCallerPtr: not supported\n" );
        goto fail;
    case NvRmIoctls_NvRmBootDone:
        return tegra_start_dvfsd();
    case NvRmIoctls_NvRmGetClientId:
        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmGetClientId: copy in failed\n");
            goto fail;
        }

        NV_ASSERT(p.InBufferSize == 0);
        NV_ASSERT(p.OutBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.InOutBufferSize == 0);

        if (NvOsCopyOut(p.pBuffer,
                        &priv->rt_client,
                        sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }
        break;
    case NvRmIoctls_NvRmClientAttach:
    {
        NvRtClientHandle Client;

        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmClientAttach: copy in failed\n");
            goto fail;
        }

        NV_ASSERT(p.InBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.OutBufferSize == 0);
        NV_ASSERT(p.InOutBufferSize == 0);

        if (NvOsCopyIn((void*)&Client,
                       p.pBuffer,
                       sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }

        NV_ASSERT(Client || !"Bad client");

        if (Client == priv->rt_client)
        {
            // The daemon is attaching to itself, no need to add refcount
            break;
        }
        if (NvRtAddClientRef(s_RtHandle, Client) != NvSuccess)
        {
            NvOsDebugPrintf("Client ref add unsuccessful\n");
            goto fail;
        }
        break;
    }
    case NvRmIoctls_NvRmClientDetach:
    {
        NvRtClientHandle Client;

        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmClientAttach: copy in failed\n");
            goto fail;
        }

        NV_ASSERT(p.InBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.OutBufferSize == 0);
        NV_ASSERT(p.InOutBufferSize == 0);

        if (NvOsCopyIn((void*)&Client,
                       p.pBuffer,
                       sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }

        NV_ASSERT(Client || !"Bad client");

        if (Client == priv->rt_client)
        {
            // The daemon is detaching from itself, no need to dec refcount
            break;
        }

        client_detach(Client);
        break;
    }
    // FIXME: power ioctls?
    default:
        printk( "unknown ioctl code\n" );
        goto fail;
    }

    e = 0;
    goto clean;

fail:
    e = -EINVAL;

clean:
    if( bAlloc )
    {
        NvOsFree( ptr );
    }

    return e;
}

int nvrm_mmap(struct file *file, struct vm_area_struct *vma)
{
    return 0;
}

static int nvrm_probe(struct platform_device *pdev)
{
    int e = 0;
    NvU32 NumTypes = NvRtObjType_NvRm_Num;

    printk("nvrm probe\n");

    NV_ASSERT(s_RtHandle == NULL);

    if (NvRtCreate(1, &NumTypes, &s_RtHandle) != NvSuccess)
    {
        e = -ENOMEM;
    }

    if (e == 0)
    {
        e = misc_register( &nvrm_dev );
    }

    if (e == 0)
    {
        e = misc_register( &knvrm_dev );
    }

    if( e < 0 )
    {
        if (s_RtHandle)
        {
            NvRtDestroy(s_RtHandle);
            s_RtHandle = NULL;
        }
        printk("nvrm probe failed to open\n");
    }
    return e;
}

static int nvrm_remove(struct platform_device *pdev)
{
    misc_deregister( &nvrm_dev );
    misc_deregister( &knvrm_dev );
    NvRtDestroy(s_RtHandle);
    s_RtHandle = NULL;
    return 0;
}

static struct platform_driver nvrm_driver =
{
    .probe     = nvrm_probe,
    .remove     = nvrm_remove,
    .driver     = { .name = "nvrm" }
};

#if defined(CONFIG_PM)
//
// /sys/power/nvrm/notifier
//

wait_queue_head_t tegra_pm_notifier_wait;
wait_queue_head_t sys_nvrm_notifier_wait;

int tegra_pm_notifier_continue_ok;

struct kobject *nvrm_kobj;

const char* sys_nvrm_notifier;

static const char *STRING_PM_SUSPEND_PREPARE = "PM_SUSPEND_PREPARE";
static const char *STRING_PM_POST_SUSPEND    = "PM_POST_SUSPEND";
static const char *STRING_PM_DISPLAY_OFF     = "PM_DISPLAY_OFF";
static const char *STRING_PM_DISPLAY_ON      = "PM_DISPLAY_ON";
static const char *STRING_PM_CONTINUE        = "PM_CONTINUE";
static const char *STRING_PM_SIGNAL          = "PM_SIGNAL";

// Reading blocks if the value is not available.
static ssize_t
nvrm_notifier_show(struct kobject *kobj, struct kobj_attribute *attr,
                   char *buf)
{
    int nchar;

    // Block if the value is not available yet.
    if (! sys_nvrm_notifier)
    {
        printk(KERN_INFO "%s: blocking\n", __func__);
        wait_event_interruptible(sys_nvrm_notifier_wait, sys_nvrm_notifier);
    }

    // In case of false wakeup, return "".
    if (! sys_nvrm_notifier)
    {
        printk(KERN_INFO "%s: false wakeup, returning with '\\n'\n", __func__);
        nchar = sprintf(buf, "\n");
        return nchar;
    }

    // Return the value, and clear.
    printk(KERN_INFO "%s: returning with '%s'\n", __func__, sys_nvrm_notifier);
    nchar = sprintf(buf, "%s\n", sys_nvrm_notifier);
    sys_nvrm_notifier = NULL;
    return nchar;
}

// Writing is no blocking.
static ssize_t
nvrm_notifier_store(struct kobject *kobj, struct kobj_attribute *attr,
                    const char *buf, size_t count)
{
    if (!strncmp(buf, STRING_PM_CONTINUE, strlen(STRING_PM_CONTINUE))) {
        // Wake up pm_notifier.
        tegra_pm_notifier_continue_ok = 1;
        wake_up(&tegra_pm_notifier_wait);
    }
    else if (!strncmp(buf, STRING_PM_SIGNAL, strlen(STRING_PM_SIGNAL))) {
        s_nvrm_daemon_pid = 0;
        sscanf(buf, "%*s %d", &s_nvrm_daemon_pid);
        printk(KERN_INFO "%s: nvrm_daemon=%d\n", __func__, s_nvrm_daemon_pid);
    }
    else {
        printk(KERN_ERR "%s: wrong value '%s'\n", __func__, buf);
    }

    return count;
}

static struct kobj_attribute nvrm_notifier_attribute =
       __ATTR(notifier, 0666, nvrm_notifier_show, nvrm_notifier_store);

//
// PM notifier
//

static void notify_daemon(const char* notice)
{
    long timeout = HZ * 30;

    // In case daemon's pid is not reported, do not signal or wait.
    if (!s_nvrm_daemon_pid) {
        printk(KERN_ERR "%s: don't know nvrm_daemon's PID\n", __func__);
        return;
    }

    // Clear before kicking nvrm_daemon.
    tegra_pm_notifier_continue_ok = 0;

    // Notify nvrm_daemon.
    sys_nvrm_notifier = notice;
    wake_up(&sys_nvrm_notifier_wait);

    // Wait for the reply from nvrm_daemon.
    printk(KERN_INFO "%s: wait for nvrm_daemon\n", __func__);
    if (wait_event_timeout(tegra_pm_notifier_wait,
                   tegra_pm_notifier_continue_ok, timeout) == 0) {
        printk(KERN_ERR "%s: timed out. nvrm_daemon did not reply\n", __func__);
    }

    // Go back to the initial state.
    sys_nvrm_notifier = NULL;
}

int tegra_pm_notifier(struct notifier_block *nb,
                      unsigned long event, void *nouse)
{
    printk(KERN_INFO "%s: start processing event=%lx\n", __func__, event);

    // Notify the event to nvrm_daemon.
    switch (event) {
    case PM_SUSPEND_PREPARE:
	NvRmPrivLockSharedPll();
	NvRmPrivDvsStop();
	NvRmPrivUnlockSharedPll();
#ifndef CONFIG_HAS_EARLYSUSPEND
        notify_daemon(STRING_PM_DISPLAY_OFF);
#endif
        notify_daemon(STRING_PM_SUSPEND_PREPARE);
        break;
    case PM_POST_SUSPEND:
        notify_daemon(STRING_PM_POST_SUSPEND);
#ifndef CONFIG_HAS_EARLYSUSPEND
        notify_daemon(STRING_PM_DISPLAY_ON);
#endif
        NvRmPrivDvsRun();
        break;
    default:
        printk(KERN_ERR "%s: unknown event %ld\n", __func__, event);
        return NOTIFY_DONE;
    }

    printk(KERN_INFO "%s: finished processing event=%ld\n", __func__, event);
    return NOTIFY_OK;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void tegra_display_off(struct early_suspend *h)
{
    notify_daemon(STRING_PM_DISPLAY_OFF);
}

void tegra_display_on(struct early_suspend *h)
{
    notify_daemon(STRING_PM_DISPLAY_ON);
}

static struct early_suspend tegra_display_power =
{
    .suspend = tegra_display_off,
    .resume = tegra_display_on,
    .level = EARLY_SUSPEND_LEVEL_DISABLE_FB
};
#endif

/*
 * NVRM CPU power gating (LP2) policy
 */
static ssize_t
nvrm_lp2policy_show(struct kobject *kobj, struct kobj_attribute *attr,
                    char *buf)
{
    return sprintf(buf, "%u\n", g_Lp2Policy);
}

static ssize_t
nvrm_lp2policy_store(struct kobject *kobj, struct kobj_attribute *attr,
                     const char *buf, size_t count)
{
    unsigned int n, policy;

    n = sscanf(buf, "%u", &policy);
    if ((n != 1) || (policy >= NvRmLp2Policy_Num))
        return -1;

    g_Lp2Policy = policy;
    return count;
}

static struct kobj_attribute nvrm_lp2policy_attribute =
              __ATTR(lp2policy, 0644, nvrm_lp2policy_show, nvrm_lp2policy_store);

/*
 * NVRM lowest power state run-time selection
 */
static ssize_t
nvrm_core_lock_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", core_lock_on);
}

static ssize_t
nvrm_core_lock_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int n, lock;

	n = sscanf(buf, "%u", &lock);
	if (n != 1)
		return -1;

	core_lock_on = (bool)lock;
	return count;
}

static struct kobj_attribute nvrm_core_lock_attribute =
	__ATTR(core_lock, 0666, nvrm_core_lock_show, nvrm_core_lock_store);

#endif

static int __init nvrm_init(void)
{
    int ret = 0;
    printk(KERN_INFO "%s called\n", __func__);

    #if defined(CONFIG_PM)
    // Register PM notifier.
    pm_notifier(tegra_pm_notifier, 0);
    tegra_pm_notifier_continue_ok = 0;
    init_waitqueue_head(&tegra_pm_notifier_wait);

    #if defined(CONFIG_HAS_EARLYSUSPEND)
    register_early_suspend(&tegra_display_power);
    #endif

    // Create /sys/power/nvrm/notifier.
    nvrm_kobj = kobject_create_and_add("nvrm", power_kobj);
    sysfs_create_file(nvrm_kobj, &nvrm_core_lock_attribute.attr);
    sysfs_create_file(nvrm_kobj, &nvrm_lp2policy_attribute.attr);
    sysfs_create_file(nvrm_kobj, &nvrm_notifier_attribute.attr);
    sys_nvrm_notifier = NULL;
    init_waitqueue_head(&sys_nvrm_notifier_wait);
    #endif

    // Register NvRm platform driver.
    ret = platform_driver_register(&nvrm_driver);

    return ret;
}

static void __exit nvrm_deinit(void)
{
    printk(KERN_INFO "%s called\n", __func__);
    platform_driver_unregister(&nvrm_driver);
}

module_init(nvrm_init);
module_exit(nvrm_deinit);
