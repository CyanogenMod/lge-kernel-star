/*
 * arch/arm/mach-tegra/nvec_user.c
 *
 * User-land access to NvEc embedded controller features
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
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include "nvos_ioctl.h"
#include "nvec.h"
#include "nvec_ioctls.h"
#include "nvreftrack.h"
#include "nvassert.h"
#include "nvec_device.h"

static NvRtHandle s_RtHandle = NULL;
int device_count = 0;

#define dev_to_nvec_driver(d)	container_of(d, struct nvec_driver, driver)
#define to_nvec_device(x) container_of((x), struct nvec_device, dev)

NvError NvECPackage_Dispatch(void *InBuffer, NvU32 InSize, void *OutBuffer,
	NvU32 OutSize, NvDispatchCtx* Ctx);

static int nvec_open(struct inode *inode, struct file *file);
static int nvec_close(struct inode *inode, struct file *file);
static long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg);

int nvec_open(struct inode *inode, struct file *file)
{
	NvRtClientHandle Client;

	if (NvRtRegisterClient(s_RtHandle, &Client) != NvSuccess)
		return -ENOMEM;

	file->private_data = (void*)Client;
	return 0;
}

int nvec_close(struct inode *inode, struct file *file)
{
	NvRtClientHandle client = (NvRtClientHandle)file->private_data;

	if (NvRtUnregisterClient(s_RtHandle, client)) {
		NvDispatchCtx dctx;

		dctx.Rt = s_RtHandle;
		dctx.Client = client;
		dctx.PackageIdx = 0;

		// TODO: Enable this code for freeing up leaked handles
		#if 0
		for (;;)
		{
			void* ptr = NvRtFreeObjRef(&dctx,
				NvRtObjType_NvEc_NvEcHandle, NULL);
			if (!ptr) break;
			NVRT_LEAK("NvEc", "NvEcHandle", ptr);
			NvEcClose(ptr);
		}
		#endif

		NvRtUnregisterClient(s_RtHandle, client);
	}
	return 0;
}

long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	NvError err;
	NvOsIoctlParams p;
	NvU32 size;
	NvU32 small_buf[8];
	void *ptr = 0;
	long e;
	NvBool bAlloc = NV_FALSE;

	switch( cmd ) {
	case NvECKernelIoctls_Generic:
	{
		NvDispatchCtx dctx;

		dctx.Rt = s_RtHandle;
		dctx.Client = (NvRtClientHandle)file->private_data;
		dctx.PackageIdx = 0;

		err = NvOsCopyIn(&p, (void *)arg, sizeof(p));
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: copy in failed\n");
			goto fail;
		}

		size = p.InBufferSize + p.InOutBufferSize + p.OutBufferSize;
		if (size <= sizeof(small_buf)) {
			ptr = small_buf;
		} else {
			ptr = NvOsAlloc(size);
			if (!ptr) {
				printk("NvECKernelIoctls_Generic: alloc err\n");
				goto fail;
			}

			bAlloc = NV_TRUE;
		}

		err = NvOsCopyIn(ptr, p.pBuffer, p.InBufferSize +
			p.InOutBufferSize);
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: copy in failure\n");
			goto fail;
		}

		err = NvECPackage_Dispatch(ptr,
			p.InBufferSize + p.InOutBufferSize,
			((NvU8 *)ptr) + p.InBufferSize, p.InOutBufferSize +
			p.OutBufferSize, &dctx);
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: dispatch failure\n");
			goto fail;
		}

		if (p.InOutBufferSize || p.OutBufferSize) {
			err = NvOsCopyOut(
				((NvU8 *)((NvOsIoctlParams *)arg)->pBuffer) +
					p.InBufferSize,
				((NvU8 *)ptr) + p.InBufferSize,
				p.InOutBufferSize + p.OutBufferSize);
			if (err != NvSuccess) {
				printk("NvECKernelIoctls_Generic: copyout err\n");
				goto fail;
			}
		}

		break;
	}
	default:
		printk("unknown ioctl code\n");
		goto fail;
	}
	e = 0;
	goto clean;

fail:
	e = -EINVAL;

clean:
	if (bAlloc)
		NvOsFree(ptr);

	return e;
}

#define DEVICE_NAME "nvec"

static const struct file_operations nvec_fops =
{
	.owner		= THIS_MODULE,
	.open		= nvec_open,
	.release	= nvec_close,
	.unlocked_ioctl	= nvec_unlocked_ioctl,
};

static struct miscdevice nvec_dev =
{
	.name	= DEVICE_NAME,
	.fops	= &nvec_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static NvEcHandle s_NvEcHandle = NULL;

static int nvec_bus_probe(struct device *_dev)
{
	struct nvec_driver *drv = dev_to_nvec_driver(_dev->driver);
	struct nvec_device *dev = to_nvec_device(_dev);

	return drv->probe(dev);
}

static int nvec_bus_remove(struct device *_dev)
{
    return 0;
}

int nvec_bus_match(struct device *_dev, struct device_driver *drv)
{
	struct nvec_device *dev = to_nvec_device(_dev);

	return (strcmp(dev->name, drv->name) == 0);
}

static int nvec_bus_suspend(struct device *_dev, pm_message_t state)
{
	struct nvec_driver *drv = dev_to_nvec_driver(_dev->driver);
	struct nvec_device *dev = to_nvec_device(_dev);
	NvError e = NvError_Success;

	device_count--;
	drv->suspend(dev, state);

	if (!device_count)
	{
		e = NvEcPowerSuspend(NvEcPowerState_Suspend);
		if (e != NvSuccess) {
			return -1;
		}
	}

	return 0;
}

static int nvec_bus_resume(struct device *_dev)
{
	struct nvec_driver *drv = dev_to_nvec_driver(_dev->driver);
	struct nvec_device *dev = to_nvec_device(_dev);

	if (!device_count)
	{
		NvError e = NvEcPowerResume();
		if (e != NvSuccess) {
			return -1;
		}
	}

	device_count++;
	drv->resume(dev);
	return 0;
}

static void nvec_bus_shutdown(struct device *pdev)
{
    NvEcPowerSuspend(NvEcPowerState_PowerDown);
}

static struct bus_type nvec_bus_type = {
	.name		= DEVICE_NAME,
	.match		= nvec_bus_match,
	.probe		= nvec_bus_probe,
	.remove		= nvec_bus_remove,
	.suspend	= nvec_bus_suspend,
	.resume		= nvec_bus_resume,
	.shutdown	= nvec_bus_shutdown,
};

static struct device nvec_bus_dev = {
	.init_name	= DEVICE_NAME
};

int nvec_register_driver(struct nvec_driver *drv)
{
	drv->driver.name = drv->name;
	drv->driver.bus = &nvec_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(nvec_register_driver);

void nvec_unregister_driver(struct nvec_driver *drv)
{
	drv->driver.bus = &nvec_bus_type;
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(nvec_unregister_driver);

int nvec_register_device(struct nvec_device *pdev)
{
	if (!pdev)
		return -EINVAL;

	if (!pdev->dev.parent)
		pdev->dev.parent = &nvec_bus_dev;

	pdev->dev.bus = &nvec_bus_type;

	dev_set_name(&pdev->dev, pdev->name);

	device_count++;
	return device_register(&pdev->dev);
}
EXPORT_SYMBOL_GPL(nvec_register_device);

void nvec_unregister_device(struct nvec_device *pdev)
{
	if (pdev) {
		device_count--;
		device_unregister(&pdev->dev);
	}
}
EXPORT_SYMBOL_GPL(nvec_unregister_device);

static int __init nvec_init(void)
{
	int err = 0;
	NvError status = NvSuccess;
	NvU32 NumTypes = 1; // TODO: must have NvRtObjType_NvEc_Num instead;

	err = device_register(&nvec_bus_dev);
	if (err)
		return err;

	err = bus_register(&nvec_bus_type);
	if (err){
		device_unregister(&nvec_bus_dev);
		return err;
	}


	NV_ASSERT(s_RtHandle == NULL);

	if (NvRtCreate(1, &NumTypes, &s_RtHandle) != NvSuccess) {
		printk("nvec NvRtCreate returned error\n");
		bus_unregister(&nvec_bus_type);
		device_unregister(&nvec_bus_dev);
		return -ENOMEM;
	}

	status = NvEcOpen(&s_NvEcHandle, 0);
	if (status != NvError_Success) {
		printk("nvec NvEcOpen returned 0x%x\n", status);
		NvRtDestroy(s_RtHandle);
		s_RtHandle = NULL;
		bus_unregister(&nvec_bus_type);
		device_unregister(&nvec_bus_dev);
		return -EINVAL;
	}

	err = misc_register(&nvec_dev);
	if (err < 0) {
		if (s_RtHandle) {
			NvEcClose(s_NvEcHandle);
			s_NvEcHandle = NULL;
			NvRtDestroy(s_RtHandle);
			s_RtHandle = NULL;
			bus_unregister(&nvec_bus_type);
			device_unregister(&nvec_bus_dev);
		}
		printk("nvec failed to open\n");
	}

	return err;
}

static void __exit nvec_exit(void)
{
	NvEcClose(s_NvEcHandle);
	s_NvEcHandle = NULL;
	misc_deregister( &nvec_dev );
	NvRtDestroy(s_RtHandle);
	s_RtHandle = NULL;

	bus_unregister(&nvec_bus_type);
	device_unregister(&nvec_bus_dev);
}

module_init(nvec_init);
module_exit(nvec_exit);

MODULE_LICENSE("GPL");
