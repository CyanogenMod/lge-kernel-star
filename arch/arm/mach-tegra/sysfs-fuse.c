/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/mm.h>
#include <mach/fuse.h>
#include "nvddk_fuse.h"
#include "mach/nvrm_linux.h"

#define SYSFS_FUSE_DEBUG_PRINTS 0

#if SYSFS_FUSE_DEBUG_PRINTS
#define PRINT_FUSE(x) printk x
#else
#define PRINT_FUSE(x)
#endif

#define BUF_SIZE 66 // Max_fuse_size(32) * AsciiCharsPerFuseByte(2) + \n\0 (2)

static struct kobject *nvfuse_kobj;

typedef enum
{
    TegraFuseSizeInBytes_DeviceKey = 4,
    TegraFuseSizeInBytes_JtagDisable = 1, // 1 bit
    TegraFuseSizeInBytes_KeyProgrammed = 1,
    TegraFuseSizeInBytes_OdmProduction = 1, // 1 bit
    TegraFuseSizeInBytes_SecBootDeviceConfig = 2, // 14 bits
    TegraFuseSizeInBytes_SecBootDeviceSelect = 1, // 3 bits
    TegraFuseSizeInBytes_SecureBootKey = 16,
    TegraFuseSizeInBytes_Sku = 4,
    TegraFuseSizeInBytes_SpareBits = 4,
    TegraFuseSizeInBytes_SwReserved = 1, // 4 bit
    TegraFuseSizeInBytes_SkipDevSelStraps = 1,  // 1 bit
    TegraFuseSizeInBytes_SecBootDeviceSelectRaw = 4,
    TegraFuseSizeInBytes_ReservedOdm = 32
} TegraFuseSizeInBytes;


static ssize_t nvfuse_raw_read(struct kobject *kobj,
    struct bin_attribute *attr, char *buf, loff_t off, size_t count);

static int nvfuse_raw_mmap(struct kobject *kobj,
    struct bin_attribute *attr, struct vm_area_struct *vma);

static ssize_t sysfsfuse_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf);

static ssize_t sysfsfuse_store(struct kobject *kobj,
    struct kobj_attribute *attr, const char *buf, size_t count);


static struct bin_attribute nvfuse_raw_attr = {
        .attr = {
                .name = "kfuse_raw",
                .mode = 0440,
        },
        .read = &nvfuse_raw_read,
        .mmap = &nvfuse_raw_mmap,
};

static struct kobj_attribute nvfuse_DeviceKey_attr =
    __ATTR(DeviceKey, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_JtagDisable_attr =
    __ATTR(JtagDisable, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_KeyProgrammed_attr =
    __ATTR(KeyProgrammed, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_OdmProduction_attr =
    __ATTR(OdmProduction, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SecBootDeviceConfig_attr =
    __ATTR(SecBootDeviceConfig, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SecBootDeviceSelect_attr =
    __ATTR(SecBootDeviceSelect, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SecureBootKey_attr =
    __ATTR(SecureBootKey, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_sku_attr =
    __ATTR(sku, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SpareBits_attr =
    __ATTR(SpareBits, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SwReserved_attr =
    __ATTR(SwReserved, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SkipDevSelStraps_attr =
    __ATTR(SkipDevSelStraps, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_SecBootDeviceSelectRaw_attr =
    __ATTR(SecBootDeviceSelectRaw, 0440, sysfsfuse_show, sysfsfuse_store);

static struct kobj_attribute nvfuse_ReservedOdm_attr =
    __ATTR(ReservedOdm, 0440, sysfsfuse_show, sysfsfuse_store);

static ssize_t nvfuse_raw_read(struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    memcpy(buf, attr->private + off, count);
    return count;
}

static int nvfuse_raw_mmap(struct kobject *kobj, struct bin_attribute *attr, struct vm_area_struct *vma)
{
    if(remap_pfn_range(vma, vma->vm_start, virt_to_phys(attr->private) >> PAGE_SHIFT, attr->size, vma->vm_page_prot)) {
        printk(KERN_ERR "nvfuse_raw_mmap failed\n");
        return -EIO;
    }
    return 0;
}

// return the fuse type based on the fuse name.
NvDdkFuseDataType GetFuseType(const char *str, unsigned int* pSize)
{
    NvDdkFuseDataType type;

    if(!strcmp(str, "DeviceKey"))
    {
        type = NvDdkFuseDataType_DeviceKey;
        *pSize = TegraFuseSizeInBytes_DeviceKey;
    }
    else if (!strcmp(str, "JtagDisable"))
    {
        type = NvDdkFuseDataType_JtagDisable;
        *pSize = TegraFuseSizeInBytes_JtagDisable;
    }
    else if (!strcmp(str, "KeyProgrammed"))
    {
        type = NvDdkFuseDataType_KeyProgrammed;
        *pSize = TegraFuseSizeInBytes_KeyProgrammed;
    }
    else if (!strcmp(str, "OdmProduction"))
    {
        type = NvDdkFuseDataType_OdmProduction;
        *pSize = TegraFuseSizeInBytes_OdmProduction;
    }
    else if (!strcmp(str, "SecBootDeviceConfig"))
    {
        type = NvDdkFuseDataType_SecBootDeviceConfig;
        *pSize = TegraFuseSizeInBytes_SecBootDeviceConfig;
    }
    else if (!strcmp(str, "SecBootDeviceSelect"))
    {
        type = NvDdkFuseDataType_SecBootDeviceSelect;
        *pSize = TegraFuseSizeInBytes_SecBootDeviceSelect;
    }
    else if (!strcmp(str, "SecureBootKey"))
    {
        type = NvDdkFuseDataType_SecureBootKey;
        *pSize = TegraFuseSizeInBytes_SecureBootKey;
    }
    else if (!strcmp(str, "sku"))
    {
        type = NvDdkFuseDataType_Sku;
        *pSize = TegraFuseSizeInBytes_Sku;
    }
    else if (!strcmp(str, "SpareBits"))
    {
        type = NvDdkFuseDataType_SpareBits;
        *pSize = TegraFuseSizeInBytes_SpareBits;
    }
    else if (!strcmp(str, "SwReserved"))
    {
        type = NvDdkFuseDataType_SwReserved;
        *pSize = TegraFuseSizeInBytes_SwReserved;
    }
    else if (!strcmp(str, "SkipDevSelStraps"))
    {
        type = NvDdkFuseDataType_SkipDevSelStraps;
        *pSize = TegraFuseSizeInBytes_SkipDevSelStraps;
    }
    else if (!strcmp(str, "SecBootDeviceSelectRaw"))
    {
        type = NvDdkFuseDataType_SecBootDeviceSelectRaw;
        *pSize = TegraFuseSizeInBytes_SecBootDeviceSelectRaw;
    }
    else if (!strcmp(str, "ReservedOdm"))
    {
        type = NvDdkFuseDataType_ReservedOdm;
        *pSize = TegraFuseSizeInBytes_ReservedOdm;
    }
    else
    {
        type = NvDdkFuseDataType_None;
        PRINT_FUSE(("\r\n Invalid Fuse type:%s... Find out reason\r",str));
    }
    return type;
}

/*
 * return values:
 *  0 - ODM production fuse is not blown
 *  1 - ODM production fuse is blown
 *  2 - Fuse Get API returned error
 */
static char OdmProductionFuseVal(void)
{
    NvU8 FuseVal = 0;
    NvError Err;
    NvU32 size = 1;

    // check if ODM production fuse is burned already. If so, return here.
    Err = NvDdkFuseGet(NvDdkFuseDataType_OdmProduction, &FuseVal, &size);
    if (Err != NvSuccess)
    {
        PRINT_FUSE(("\r\n NvDdkFuseGet failed in sysfsfuse_store\n"));
        return 2;
    }
    if (FuseVal)
    {
        PRINT_FUSE(("\r\n ODM production fuse is already blown\n"));
        return 1;
    }
    else
        return 0;
}

void IntToStr(char *arr, char *str, unsigned int MaxSize)
{
    int i = 0, j =  0;
    char tmp[BUF_SIZE]={0};
    strcpy(tmp,"0");

    for(i = 0; i < MaxSize; i++)
    {
        if ((arr[i] == 0) && (j==0))
            continue;

        sprintf(&tmp[j*2],"%02X",arr[i]);
        j++;
    }
    strcat(tmp,"\n");

    // if first char contains 0, skip it.
    if ((*tmp == '0') && (j != 0))
    {
        strcpy(str, &tmp[1]);
    }
    else
        strcpy(str, &tmp[0]);

    PRINT_FUSE(("\n string out: %s \n",str));
}

static ssize_t sysfsfuse_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
#if SYSFS_FUSE_DEBUG_PRINTS
    unsigned char i = 0;
#endif
    unsigned int size = 0;
    NvError Err;
    NvDdkFuseDataType FuseType;
    char ReadFuseBuf[BUF_SIZE];
    char OdmProdFuseVal = 0;

    FuseType = GetFuseType(attr->attr.name, &size);

    // Reading SecureBootKey should be allowed only till ODM production is not blown
    if (FuseType == NvDdkFuseDataType_SecureBootKey)
    {
        OdmProdFuseVal = OdmProductionFuseVal();
        if(OdmProdFuseVal == 1)
        {
            PRINT_FUSE(("\r\n ODM production fuse is already blown\n"));
            return 0;
        }
    }

    NvDdkFuseSense();
    NvDdkFuseClear();
    Err = NvDdkFuseGet(FuseType, ReadFuseBuf, &size);
    if (Err != NvSuccess)
    {
        PRINT_FUSE(("\r\n NvDdkFuseGet failed in sysfsfuse_show 2\n"));
        return 0;
    }
    IntToStr(ReadFuseBuf, buf, size);
#if SYSFS_FUSE_DEBUG_PRINTS
    if (Err == NvSuccess)
        PRINT_FUSE(("\n Fuse Get success \n"));
    else
        PRINT_FUSE(("\n Fuse Get failed Err: 0x%x \n",Err));
    PRINT_FUSE(("\n\n Data read from Fuse:\n"));
    for (i = 0; i < size; i++)
        PRINT_FUSE(("  0x%x",ReadFuseBuf[i]));
#endif
    return (strlen(buf));
}

void StrToInt(const char *str,
        char *arr, unsigned char NumChars, unsigned int MaxSize)
{
    int i = 0, j = 0;
    char tmp[BUF_SIZE]={0};
    unsigned char Iterations = (NumChars + 1)/2;
    unsigned int t;
    for(j = 0; j < (MaxSize - Iterations); j++)
    {
        arr[j] = 0;
    }
    for(i = 0; i < Iterations; i++)
    {
        if (NumChars & 1)
        {
            if (!i)
                strncpy(tmp, &str[0], 1);
            else
                strncpy(tmp, &str[i*2 - 1], 2);
        }
        else
            strncpy(tmp, &str[i*2], 2);
        sscanf(tmp,"%02X",&t);
        arr[j++]=(char)t;
    }

    PRINT_FUSE(("\r\n Input string after conversion to int: 0x%x", *arr));
}

static ssize_t sysfsfuse_store(struct kobject *kobj,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret;
    unsigned int size = 0;
    NvDdkFuseDataType FuseType;
    NvError Err;
    char WriteFuseBuf[BUF_SIZE];
#if SYSFS_FUSE_DEBUG_PRINTS
    unsigned char i = 0;
#endif
    unsigned char OdmProdFuseVal = 0;

    OdmProdFuseVal = OdmProductionFuseVal();
    if(OdmProdFuseVal == 1)
    {
        PRINT_FUSE(("\r\n ODM production fuse is already blown\n"));
        return 0;
    }

    FuseType = GetFuseType(attr->attr.name, &size);

    // As count includes data bytes followed by 0xA (line feed character)
    // and two chars can be stored in a fuse byte
    if ((count - 1) > (2 * size))
    {
        PRINT_FUSE(("\r\n Requested data size[%d] > fuse size [%d]\n",count/2,size));
        return 0;
    }

    if (buf != NULL)
    {
        StrToInt(buf, WriteFuseBuf, (count - 1), size);
    }
    else
        printk("\r\n buf is NULL");

    PRINT_FUSE(("\r\n Input string: %s", buf));
    PRINT_FUSE(("\n Fuse data of size [%d] to write\n", size));
#if SYSFS_FUSE_DEBUG_PRINTS
    for (i = 0; i < size; i++)
        PRINT_FUSE(("0x%x\n",WriteFuseBuf[i]));
#endif
    NvDdkFuseClear();
    Err = NvDdkFuseSet(FuseType, WriteFuseBuf, &size);
    if (Err == NvSuccess)
    {
        NvDdkFuseProgram();
        NvDdkFuseSense();
        NvDdkFuseVerify();
    }
    else
    {
        PRINT_FUSE(("\r\n Fuse programming failed with error 0x%x",Err));
    }
    if (Err == NvSuccess)
    {
        ret = count;
        // if ODM production fuse is blown, change file permissions to 0440
        if (FuseType == NvDdkFuseDataType_OdmProduction)
        {
            sysfs_chmod_file(kobj, &attr->attr, 0440);

            sysfs_chmod_file(kobj, &nvfuse_DeviceKey_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_JtagDisable_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_OdmProduction_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SecBootDeviceConfig_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SecBootDeviceSelect_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SecureBootKey_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SwReserved_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SkipDevSelStraps_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_SecBootDeviceSelectRaw_attr.attr, 0440);
            sysfs_chmod_file(kobj, &nvfuse_ReservedOdm_attr.attr, 0440);
        }
        PRINT_FUSE(("\r\n fuse set success \n"));
    }
    else
    {
        ret = 0;
        PRINT_FUSE(("\r\n fuse set failed Err: 0x%x\n", Err));
    }

    return ret;
}

#define CHK_ERR(x)  \
{ \
    if(x) \
    { \
        PRINT_FUSE(("Fuse: sysfs_create_file failed!")); \
        return x; \
    } \
}

static int __init sysfsfuse_init(void)
{
    nvfuse_kobj = kobject_create_and_add("fuse", firmware_kobj);
    PRINT_FUSE(("\n Fuse Init"));

    CHK_ERR(NvDdkFuseOpen(s_hRmGlobal));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_OdmProduction_attr.attr));

    // change fuse file permissions, if ODM production fuse is not blown
    if (OdmProductionFuseVal() == 0)
    {
        nvfuse_DeviceKey_attr.attr.mode = 0640;
        nvfuse_JtagDisable_attr.attr.mode = 0640;
        nvfuse_OdmProduction_attr.attr.mode = 0640;
        nvfuse_SecBootDeviceConfig_attr.attr.mode = 0640;
        nvfuse_SecBootDeviceSelect_attr.attr.mode = 0640;
        nvfuse_SecureBootKey_attr.attr.mode = 0640;
        nvfuse_SwReserved_attr.attr.mode = 0640;
        nvfuse_SkipDevSelStraps_attr.attr.mode = 0640;
        nvfuse_SecBootDeviceSelectRaw_attr.attr.mode = 0640;
        nvfuse_ReservedOdm_attr.attr.mode = 0640;
        sysfs_chmod_file(nvfuse_kobj, &nvfuse_OdmProduction_attr.attr, 0640);
    }

    nvfuse_raw_attr.private = tegra_kfuse_cache_get(&nvfuse_raw_attr.size);
    CHK_ERR(sysfs_create_bin_file(nvfuse_kobj, &nvfuse_raw_attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_DeviceKey_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_JtagDisable_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_KeyProgrammed_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SecBootDeviceConfig_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SecBootDeviceSelect_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SecureBootKey_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_sku_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SpareBits_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SwReserved_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SkipDevSelStraps_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_SecBootDeviceSelectRaw_attr.attr));
    CHK_ERR(sysfs_create_file(nvfuse_kobj, &nvfuse_ReservedOdm_attr.attr));
    PRINT_FUSE(("\n Fuse Init Exiting"));
    return 0;
}

static void __exit sysfsfuse_exit(void)
{
    sysfs_remove_file(nvfuse_kobj, &nvfuse_DeviceKey_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_JtagDisable_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_KeyProgrammed_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_OdmProduction_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SecBootDeviceConfig_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SecBootDeviceSelect_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SecureBootKey_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_sku_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SpareBits_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SwReserved_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SkipDevSelStraps_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_SecBootDeviceSelectRaw_attr.attr);
    sysfs_remove_file(nvfuse_kobj, &nvfuse_ReservedOdm_attr.attr);
    kobject_del(nvfuse_kobj);
    NvDdkFuseClose();
}

module_init(sysfsfuse_init);
module_exit(sysfsfuse_exit);
MODULE_LICENSE("GPL");
