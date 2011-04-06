
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
//#include <linux/tegra_devices.h>	//20100716  blocking for compile error [LGE]

#include <nvodm_services.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#include <linux/kobject.h>
#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include <linux/wakelock.h>

#define WM8994_I2C_RETRY_COUNT 5
#define WM8994_I2C_TIMEOUT 20

typedef struct star_wm8994_device_data
{
	NvOdmServicesI2cHandle h_gen2_i2c;
	NvU32 i2c_address;
	struct wake_lock wm8994_wake_lock;
}star_wm8994_device;

static star_wm8994_device *g_wm8994;

static NvBool 
WriteWolfsonRegister(star_wm8994_device *wm8994, NvU32 RegIndex, NvU32 Data)
{
    int i;
    NvOdmI2cStatus I2cTransStatus = NvOdmI2cStatus_Timeout;    
    NvU8 pTxBuffer[4];
    NvOdmI2cTransactionInfo TransactionInfo;
    
    for (i = 0; i < WM8994_I2C_RETRY_COUNT && I2cTransStatus != NvOdmI2cStatus_Success; i++)
    {
        pTxBuffer[0] = (NvU8)((RegIndex >> 8) & 0xFF); 
        pTxBuffer[1] = (NvU8)(RegIndex & 0xFF); 
        pTxBuffer[2] = (NvU8)((Data >> 8) & 0xFF);
        pTxBuffer[3] = (NvU8)((Data) & 0xFF);

        TransactionInfo.Address = wm8994->i2c_address;
        TransactionInfo.Buf = pTxBuffer;
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 4;

        I2cTransStatus = NvOdmI2cTransaction(wm8994->h_gen2_i2c, &TransactionInfo, 1, 400, WM8994_I2C_TIMEOUT);
    }

    if (I2cTransStatus == NvOdmI2cStatus_Success)
    {
        return NV_TRUE;
    }    
    printk("[star wm8994 driver] i2c transaction error\n");
    return NV_FALSE;
}

static NvBool ReadWolfsonRegister(star_wm8994_device *wm8994, NvU32 RegIndex, NvU32 *Data)
{
    int i;
    NvU8 *pReadBuffer;
    NvOdmI2cStatus status = NvOdmI2cStatus_Timeout;
    NvOdmI2cTransactionInfo *pTransactionInfo;
    
    pReadBuffer = NvOdmOsAlloc(2);
    if (!pReadBuffer)
    {
        return NV_FALSE;
    }

    pTransactionInfo = NvOdmOsAlloc(sizeof(NvOdmI2cTransactionInfo) *2 );
    if (!pTransactionInfo)
    {
        NvOdmOsFree(pReadBuffer);
        return NV_FALSE;
    }

    for (i = 0; i < WM8994_I2C_RETRY_COUNT && status != NvOdmI2cStatus_Success; i++)
    {
        pReadBuffer[0] = (NvU8)((RegIndex >> 8) & 0xFF); 
        pReadBuffer[1] = (NvU8)(RegIndex & 0xFF); 

        pTransactionInfo[0].Address = wm8994->i2c_address;
        pTransactionInfo[0].Buf = pReadBuffer;
        pTransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
        pTransactionInfo[0].NumBytes = 2;

        pTransactionInfo[1].Address = (wm8994->i2c_address | 0x1);
        pTransactionInfo[1].Buf = pReadBuffer;
        pTransactionInfo[1].Flags = 0;
        pTransactionInfo[1].NumBytes = 2;

        status = NvOdmI2cTransaction(wm8994->h_gen2_i2c, pTransactionInfo, 2,
                                        400, WM8994_I2C_TIMEOUT);
    }

    if (status != NvOdmI2cStatus_Success)
    {
        printk("NvOdmWM8994I2cRead Failed: %d\n", status);
        NvOdmOsFree(pReadBuffer);
        NvOdmOsFree(pTransactionInfo);
        return NV_FALSE;
    }

    *Data = (NvU32)((pReadBuffer[0] << 8) | pReadBuffer[1]);

    NvOdmOsFree(pReadBuffer);
    NvOdmOsFree(pTransactionInfo);
    return NV_TRUE;
}

static ssize_t wm8994_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int r =0;
	int cnt =0;
	NvU32 r_data;
	for(cnt = 0 ;cnt < 0x60;cnt++){
        ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	    r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	}
	cnt= 0x0210;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0420;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0601;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0610;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0611;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0200;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
	cnt= 0x0208;
	ReadWolfsonRegister(g_wm8994, cnt, &r_data);
	r += sprintf(buf+r, "wm8994 reg 0x%4x : 0x%4x\n",cnt, r_data);
		
	return r;
}

ssize_t wm8994_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int reg, data;
	char *r, *d;
	
	r= &buf[0];
	d= &buf[7];
		
	reg = simple_strtoul(r, NULL, 16);
	data = simple_strtoul(d, NULL, 16);

    if(reg == 0){
        return count; //bolck reset cmd.
    }
	else{
	WriteWolfsonRegister(g_wm8994, reg, data);
	}

	return count;
}


static DEVICE_ATTR(data, 0666, wm8994_reg_show, wm8994_reg_store);

ssize_t wm8994_wakelock_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int n, lock;

	n = sscanf(buf, "%u", &lock);
	if (n != 1)
		return -1;

	if((bool)lock)
	{
        wake_lock(&g_wm8994->wm8994_wake_lock);
	}
	else
	{
        wake_unlock(&g_wm8994->wm8994_wake_lock);
	}
	
	return count;
}

static DEVICE_ATTR(wm8994_wakelock, 0666, NULL, wm8994_wakelock_store);

void star_headsetdet_bias(int bias)
{
    NvU32 r_data = 0;
    ReadWolfsonRegister(g_wm8994, 0x0001, &r_data);
    if(bias == 0)
    {
        r_data = r_data & (~0x0020);
		printk("star_headsetdet_bias headset disabled %4x\n",r_data);
    }
	else
	{
        r_data = r_data | (0x0020);
		printk("star_headsetdet_bias headset enabled %4x\n",r_data);
	}
	WriteWolfsonRegister(g_wm8994, 0x0001, r_data);
	return;
}

/**
 * All the device spefic initializations happen here. 
 */
static NvS32 __init wm8994_probe(struct platform_device *pdev)
{

	NvS32 err = 0;

	
	const NvOdmPeripheralConnectivity *pcon = NULL;
		
	pcon = NvOdmPeripheralGetGuid(NV_ODM_GUID('w','o','l','f','8','9','9','4'));
	if(pcon == NULL)
	{
		return err;
	}
	g_wm8994 = kzalloc(sizeof(*g_wm8994), GFP_KERNEL);
	g_wm8994->i2c_address = pcon->AddressList[2].Address;
	g_wm8994->h_gen2_i2c = NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2 );

	if(g_wm8994->h_gen2_i2c == NULL){
		return err;
	}
		
	//reset wm8994 codec
	WriteWolfsonRegister(g_wm8994, 0x0000, 0x0001);	
	WriteWolfsonRegister(g_wm8994, 0x0001, 0x0003);
	wake_lock_init(&g_wm8994->wm8994_wake_lock, WAKE_LOCK_SUSPEND, "wm8994_call_wakelock");
	err = device_create_file(&pdev->dev, &dev_attr_data);
	err = device_create_file(&pdev->dev, &dev_attr_wm8994_wakelock);
	
	return err;
}



static NvS32 wm8994_remove(struct platform_device *pdev)
{
	
	return 0;
}

static struct platform_driver star_wm8994_driver = {
	.probe	= wm8994_probe,
	.remove = wm8994_remove,
	.driver = {
		.name = "star_wm8994",
	},
};

static NvS32 __devinit  wm8994_init(void)
{
	return platform_driver_register(&star_wm8994_driver);
}

static void __exit wm8994_exit(void)
{
	platform_driver_unregister(&star_wm8994_driver);
}

late_initcall(wm8994_init);
module_exit(wm8994_exit);

MODULE_DESCRIPTION("WM8994 test driver");
MODULE_LICENSE("GPL");

