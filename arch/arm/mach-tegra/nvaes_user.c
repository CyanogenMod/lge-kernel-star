/*
 * arch/arm/mach-tegra/nvaes_user.c
 *
 * User-land access to AES Cryptographic engine.
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#include <mach/nvrm_linux.h>

#include <nvddk_aes_ioctls.h>
#include <nvos_ioctl.h>
#include <nvos.h>
#include <nvassert.h>
#include <nvreftrack.h>
#include <nvddk_aes.h>

static NvRtHandle s_hRt = NULL;
static struct platform_device *tegra_aes_device = NULL;
NvError NvDdkAes_Dispatch( void *InBuffer, NvU32 InSize, void *OutBuffer, NvU32 OutSize, NvDispatchCtx* Ctx );

static int nvaes_user_open(struct inode *inode, struct file *file)
{
    NvRtClientHandle hClient = (NvRtClientHandle)NULL;
    NvError e = NvRtRegisterClient(s_hRt, &hClient);
    if (NvSuccess != e)
    {
        printk(KERN_INFO "nvaes_user_open: NvRtRegisterClient returned error 0x%x.\n", e);
        return -1;
    }

    file->private_data = (void*)hClient;

    printk(KERN_INFO "nvaes_user_open: hClient = 0x%x.\n", hClient);

    return 0;
}

static int nvaes_user_release(struct inode *inode, struct file *file)
{
    NvRtClientHandle hClient = (NvRtClientHandle)file->private_data;

    if (NvRtUnregisterClient(s_hRt, hClient))
    {
        NvDispatchCtx dctx;

        NvOsMemset(&dctx, 0, sizeof(NvDispatchCtx));
        dctx.Rt = s_hRt;
        dctx.Client = hClient;

        for (;;)
        {
            void* ptr = NvRtFreeObjRef(&dctx, NvRtObjType_NvDdkAes_NvDdkAesHandle, NULL);
            if (!ptr)
                break;
            NvDdkAesClose((NvDdkAesHandle)ptr);
        }

        if (NvRtUnregisterClient(s_hRt, hClient))
            panic("nvaes_user_release: Unable to free resourceTracker client!\n");
    }
    return 0;
}

static long nvaes_user_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    NvError e = NvSuccess;
    NvOsIoctlParams IoctlParams;
    NvU32 size = 0;
    NvU32 SmallBuf[8];
    void *ptr = NULL;
    NvBool IsAlloc = NV_FALSE;
    long status = 0;

    switch (cmd)
    {
        case NvDdkAesKernelIoctls_Generic:
        {
            NvDispatchCtx dctx;

            dctx.Rt = s_hRt;
            dctx.Client = (NvRtClientHandle)file->private_data;
            dctx.PackageIdx = 0;

            e = NvOsCopyIn(&IoctlParams, (void *)arg, sizeof(IoctlParams));
            if (e != NvSuccess)
            {
                printk("NvDdkAesKernelIoctls_Generic: copy in failed\n");
                goto fail;
            }

            size = IoctlParams.InBufferSize + IoctlParams.InOutBufferSize + IoctlParams.OutBufferSize;
            if (size <= sizeof(SmallBuf))
            {
                ptr = SmallBuf;
            }
            else
            {
                ptr = NvOsAlloc(size);
                if (!ptr)
                {
                    printk("NvDdkAesKernelIoctls_Generic: alloc err\n");
                    goto fail;
                }
                IsAlloc = NV_TRUE;
            }

            e = NvOsCopyIn(ptr, IoctlParams.pBuffer, IoctlParams.InBufferSize + IoctlParams.InOutBufferSize);
            if (e != NvSuccess)
            {
                printk("NvDdkAesKernelIoctls_Generic: copy in failure\n");
                goto fail;
            }

            e = NvDdkAes_Dispatch(ptr,
                IoctlParams.InBufferSize + IoctlParams.InOutBufferSize,
                ((NvU8 *)ptr) + IoctlParams.InBufferSize, IoctlParams.InOutBufferSize + IoctlParams.OutBufferSize,
                &dctx);
            if (e != NvSuccess)
            {
                printk("NvDdkAesKernelIoctls_Generic: dispatch failure, err = 0x%x\n", e);
                goto fail;
            }

            if (IoctlParams.InOutBufferSize || IoctlParams.OutBufferSize)
            {
                e = NvOsCopyOut(
                    ((NvU8 *)((NvOsIoctlParams *)arg)->pBuffer) +
                    IoctlParams.InBufferSize,
                    ((NvU8 *)ptr) + IoctlParams.InBufferSize,
                    IoctlParams.InOutBufferSize + IoctlParams.OutBufferSize);
                if (e != NvSuccess)
                {
                    printk("NvDdkAesKernelIoctls_Generic: copyout err\n");
                    goto fail;
                }
            }
            break;
        }
        default:
        printk("unknown ioctl code\n");
        goto fail;
    }
    goto clean;

fail:
    status = -EINVAL;

clean:
    if (IsAlloc)
        NvOsFree(ptr);
    return status;
}

static const struct file_operations nvaes_user_fops =
{
    .owner        = THIS_MODULE,
    .open        = nvaes_user_open,
    .release    = nvaes_user_release,
    .unlocked_ioctl    = nvaes_user_unlocked_ioctl,
};

static struct miscdevice nvaes_user_dev =
{
    .name    = "nvaes",
    .fops    = &nvaes_user_fops,
    .minor    = MISC_DYNAMIC_MINOR,
};

static int __devinit nvaes_user_probe(struct platform_device *pdev)
{
    NvU32 NumTypes = NvRtObjType_NvDdkAes_Num;

    NV_ASSERT(s_hRt == NULL);

    if (NvSuccess != NvRtCreate(1, &NumTypes, &s_hRt))
    {
        printk(KERN_INFO "nvaes_user_init: NvRtCreate returned error.\n");
        goto fail2;
    }

    if (misc_register(&nvaes_user_dev))
    {
        printk(KERN_INFO "nvaes_user_init: misc_register returned error.\n");
        goto fail;
    }

    return 0;

fail:
    NvRtDestroy(s_hRt);
    s_hRt = NULL;

fail2:
    return -1;

}

static int __devexit nvaes_user_remove(struct platform_device *pdev)
{
    misc_deregister(&nvaes_user_dev);
    if (s_hRt)
    {
        NvRtDestroy(s_hRt);
        s_hRt = NULL;
    }
    return 0;
}

#if defined(CONFIG_PM)
static int tegra_aes_suspend(struct platform_device *pdev, pm_message_t state)
{
	NvDdkAesSuspend();
	return 0;
}

static int tegra_aes_resume(struct platform_device *pdev)
{
	NvDdkAesResume();
	return 0;
}
#endif

struct platform_driver tegra_aes_driver = {
	.probe		= nvaes_user_probe,
	.remove		= __devexit_p(nvaes_user_remove),
#if defined(CONFIG_PM)
	.suspend = tegra_aes_suspend,
	.resume = tegra_aes_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver    = {
		.name	= "nvaes",
		.owner	= THIS_MODULE,
	},
};

static int __init nvaes_user_init(void)
{
	int err;
	err = platform_driver_register(&tegra_aes_driver);
	if (err) {
		pr_err("nvaes: Platform driver registration failed \n");
		goto error;
	}

	tegra_aes_device = platform_device_alloc("nvaes", -1);

	if (!tegra_aes_device) {
		err = -ENOMEM;
		pr_err("nvaes: Platform device allocation failed \n");
		goto error_unregister_driver;
	}
	err = platform_device_add(tegra_aes_device);
	if (err) {
		pr_err("nvaes: Platform device addiotion failed\n");
		goto error_free_device;
	}
	return 0;

error_free_device:
	platform_device_put(tegra_aes_device);

error_unregister_driver:
	platform_driver_unregister(&tegra_aes_driver);

error:
	return err;
}

static void __exit nvaes_user_exit(void)
{
	platform_driver_unregister(&tegra_aes_driver);
}

module_init(nvaes_user_init);
module_exit(nvaes_user_exit);
MODULE_LICENSE("GPL");

