/*
 * drivers/star/star_muic.c
 *
 * Muic class driver for platforms 
 *
 * Copyright (c) 2010, LG Electronics Corporation.
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
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>  
#include <linux/freezer.h>
//#include <linux/tegra_devices.h>
#include <nvodm_services.h>

#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "mach/nvrm_linux.h"
#include "nvodm_query_discovery.h"

#include <mach/lprintk.h>
#include "star_muic_TI.h"

#include "mach/lprintk.h"

#define MUIC_I2C_SPEED_KHZ       400
#define MUIC_I2C_TIMEOUT         500
#define MUIC_GUID                NV_ODM_GUID('s','t','a','r','m','u','i','c')
#define NVODMMUIC_PRINTF(x)      NvOdmOsDebugPrintf x

static const char name_muic_mode[MUIC_MODE_NO][30] = {
    "MUIC_UNKNOWN",         // 0
    "MUIC_NONE",            // 1
    "MUIC_NA_TA",           // 2
    "MUIC_LG_TA",           // 3
    "MUIC_HCHH",            // 4
    "MUIC_INVALID_CHG",     // 5
    "MUIC_AP_UART",         // 6
    "MUIC_CP_UART",         // 7
    "MUIC_AP_USB",          // 8
    "MUIC_CP_USB",          // 9
    "MUIC_TV_OUT_NO_LOAD",  // 10
    "MUIC_EARMIC",          // 11
    "MUIC_TV_OUT_LOAD",     // 12
    "MUIC_OTG",             // 13
    "MUIC_DESK_CRADLE",     // 14
    "MUIC_RESERVE1",        // 15
    "MUIC_RESERVE2",        // 16
};

static struct work_struct muic_wq;
//struct wake_lock muic_wakelock; 	// FIXME: What is this for?

TYPE_USIF_MODE usif_mode = USIF_AP;
TYPE_DP3T_MODE dp3t_mode = DP3T_NC;
TYPE_UPON_IRQ  upon_irq = NOT_UPON_IRQ;

TYPE_MUIC_MODE muic_mode = MUIC_UNKNOWN;

static NvU8 int_stat_val;
static NvU8 status_val;

typedef struct NvOdmServicesGpioRec
{
    NvRmDeviceHandle hRmDev;
    NvRmGpioHandle hGpio;
} NvOdmServicesGpio;

typedef struct MuicDeviceRec
{
#ifndef _MUIC_GPIO_I2C_
    NvOdmServicesI2cHandle hOdmI2c;
#endif
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle h_INT_N_MUIC;
    NvOdmGpioPinHandle h_AP20_UART_SW;
    NvOdmGpioPinHandle h_IFX_UART_SW;
    NvOdmGpioPinHandle h_USIF1_SW;    
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
    NvU32 DeviceAddr;    
    NvU32 VddId;
#ifdef _MUIC_GPIO_I2C_
    NvOdmGpioPinHandle      hSclGpioPinHandle;
    NvOdmGpioPinHandle      hSdaGpioPinHandle;
#endif
} Muic_Device;

void dp3t_switch_ctrl(Muic_Device *s_hMuicHandle, TYPE_DP3T_MODE mode);
void usif_switch_ctrl(Muic_Device *s_hMuicHandle, TYPE_USIF_MODE mode);
void muic_AP_UART_set(Muic_Device *s_hMuicHandle);
void muic_CP_UART_set(Muic_Device *s_hMuicHandle);
void muic_AP_USB_set(Muic_Device *s_hMuicHandle);
void muic_CP_USB_set(Muic_Device *s_hMuicHandle);

//20100526, jh.ahn@lge.com, charging_ic Function [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
typedef enum {
  CHG_IC_DEFAULT_MODE=0,    		/* 0  */
  CHG_IC_TA_MODE,
  CHG_IC_USB_LO_MODE,
  CHG_IC_FACTORY_MODE,

  CHG_IC_DEACTIVE_MODE,			/* 4  */
  CHG_IC_INIT_MODE,
} max8922_status;

extern void charging_ic_active(NvU32);
extern void charging_ic_deactive(void);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, charging_ic Function [END]

NvBool muic_factory_audio_mode = NV_FALSE;

static Muic_Device *hMuicHandle = NULL;
//static struct work_struct muic_wq;
//static NvBool send_key_muic_hook_pressed = NV_FALSE;
//DP3T_MODE_TYPE  DP3T_mode = DP3T_NC;

#ifdef _MUIC_GPIO_I2C_

//#define SIMI2C_DEBUG(X)	printk(X)
//MUIC_SDA GPIO_PK4
#define MUIC_SDA_PORT  'k'-'a'
#define MUIC_SDA_PIN    4
//MUIC_SCL GPIO_PI7
#define MUIC_SCL_PORT   'i'-'a'
#define MUIC_SCL_PIN    7

#define HALF_DELAY      1
#define FULL_DELAY      1

#define I2C_SCL_LO      NvOdmGpioSetState(s_hMuicHandle->hGpio,s_hMuicHandle->hSclGpioPinHandle , 0x0)
#define I2C_SCL_HI      NvOdmGpioSetState(s_hMuicHandle->hGpio,s_hMuicHandle->hSclGpioPinHandle , 0x1)
#define I2C_SDA_LO      NvOdmGpioSetState(s_hMuicHandle->hGpio,s_hMuicHandle->hSdaGpioPinHandle , 0x0)
#define I2C_SDA_HI      NvOdmGpioSetState(s_hMuicHandle->hGpio,s_hMuicHandle->hSdaGpioPinHandle , 0x1)
#define I2C_SDA_LV      NvOdmGpioGetState(s_hMuicHandle->hGpio,s_hMuicHandle->hSdaGpioPinHandle,&val)

#define I2C_SCL_DIR(d)  NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->hSclGpioPinHandle,(d+4))
#define I2C_SDA_DIR(d)  NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->hSdaGpioPinHandle,(d+4))
#define I2C_SDA_IN      I2C_SDA_DIR(DIR_IN)
#define I2C_SDA_OUT     I2C_SDA_DIR(DIR_OUT)

static unsigned int val;
#define I2cSimDelay(X)  NvOdmOsWaitUS(X)

#define GPIOF_TPS_OUTPUT	0x00040000
#define GPIOF_TPS_INPUT		0x00020000
//#define I2cSimDelay(X)		mdelay(X)
#define DIR_OUT	1
#define DIR_IN 0

////////////////////////////////////////////////////////////////////////////////
// Adjust macro, global variable, 
// I2cSimInit and I2cSimDelay functions for your system

#define I2C_IGNORE_ACK          0
#define I2C_DELAY_HALF          I2cSimDelay(g_hdelay)
#define I2C_DELAY_FULL          I2cSimDelay(g_fdelay)

static int g_hdelay;
static int g_fdelay;

#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------

//#define _SIM_I2C_DEBUG_

__inline static void simi2c_sendstart(Muic_Device* s_hMuicHandle)
{
#ifdef _SIM_I2C_DEBUG_
    SIMI2C_DEBUG("[START]");
#endif   		
    I2C_SDA_HI;
    I2C_SDA_OUT;
    I2C_DELAY_HALF;
    I2C_SCL_HI;
    I2C_DELAY_HALF;
    I2C_SDA_LO;
    I2C_DELAY_HALF;
    I2C_SCL_LO;
    I2C_DELAY_HALF;
}

__inline static void simi2c_sendstop(Muic_Device* s_hMuicHandle)
{
    I2C_SDA_LO;
    I2C_SDA_OUT;
    I2C_DELAY_HALF;
    I2C_SCL_HI;
    I2C_DELAY_HALF;
    I2C_SDA_HI;
#ifdef _SIM_I2C_DEBUG_
    SIMI2C_DEBUG("[STOP]");
#endif   		
    
}

__inline static unsigned int simi2c_getack(Muic_Device* s_hMuicHandle)             
{
    unsigned char ret;
    I2C_SDA_IN;
    I2C_SCL_LO;
    I2C_DELAY_HALF;
    I2C_SCL_HI;
    I2C_DELAY_FULL;

    I2C_SDA_LV;
    ret = ( 0 == val);		
    //ret = (0==I2C_SDA_LV);
#if I2C_IGNORE_ACK
    ret = 1;
#endif
    I2C_SCL_LO;
    I2C_DELAY_HALF;
#if I2C_IGNORE_ACK
    ret = 1;
#endif

#ifdef _SIM_I2C_DEBUG_
    if (ret) SIMI2C_DEBUG("<A>"); else SIMI2C_DEBUG("<N>");
    if (ret!=1)
        SIMI2C_DEBUG("NAK!!!!\n");
#endif
       
    return ret;
}

__inline static void simi2c_sendack(Muic_Device* s_hMuicHandle)
{
    I2C_SDA_LO;
    I2C_SDA_OUT;
    I2C_SCL_HI;
    I2C_DELAY_FULL;
    I2C_SCL_LO;
    I2C_DELAY_HALF;
#ifdef _SIM_I2C_DEBUG_
    SIMI2C_DEBUG("[A]");
#endif     	    
}

__inline static unsigned char simi2c_getbyte(Muic_Device* s_hMuicHandle)
{ 
    unsigned char tmp, ret = 0;
    int i;
    I2C_SDA_IN; // config as intput
    I2C_SCL_LO;
    I2C_DELAY_HALF;
    for( i = 7 ; i >=0 ; i--  ) 
    {
        I2C_SCL_HI;
        I2C_DELAY_FULL;
	val = -1;
	I2C_SDA_LV;
        tmp = (0 != val);
#ifdef _SIM_I2C_DEBUG_
	if (tmp) SIMI2C_DEBUG("<H>"); else SIMI2C_DEBUG("<L>");
#endif	        
        tmp <<= i;
        ret |= tmp;
        I2C_SCL_LO;
        I2C_DELAY_FULL;
    }
    return ret;
}

__inline static void simi2c_sendbyte(Muic_Device* s_hMuicHandle,unsigned char data)
{
    int i;
 
    I2C_SDA_LO;
    I2C_SDA_OUT; // config as output
    I2C_SCL_LO;
    for(i = 7; i >= 0; i--) 
    {   
        if(data & (1 << i))
        {
            I2C_SDA_HI;
#ifdef _SIM_I2C_DEBUG_
	    SIMI2C_DEBUG("[H]");
#endif	    	
        }
        else
        {     	
            I2C_SDA_LO;
#ifdef _SIM_I2C_DEBUG_
	    SIMI2C_DEBUG("[L]");
#endif            
        }
        I2C_SCL_HI;
        I2C_DELAY_FULL;
        I2C_SCL_LO;
        I2C_DELAY_FULL;
    }
}

__inline static void simi2c_sendaddr(Muic_Device* s_hMuicHandle,unsigned char addr, unsigned char readflag)
{
    simi2c_sendbyte(s_hMuicHandle,((addr << 1) | readflag));
}

__inline static unsigned int simi2c_read(Muic_Device* s_hMuicHandle,unsigned char addr, unsigned char *pBuf, unsigned int size, unsigned char sendstop)
{
    unsigned int i = -1;
   
    simi2c_sendstart(s_hMuicHandle);
    simi2c_sendaddr(s_hMuicHandle,addr, 1);
    if (simi2c_getack(s_hMuicHandle))
    {
        for (i = 0; i < size; i++)
        {
            pBuf[i] = simi2c_getbyte(s_hMuicHandle);
            simi2c_sendack(s_hMuicHandle);
        }
        if (sendstop)
        {
            simi2c_sendstop(s_hMuicHandle);
        }
    }
#ifdef _SIM_I2C_DEBUG_
	   // SIMI2C_DEBUG("<0x%x:0x%x>",addr,pBuf[0]);
#endif     
    return i;
}

__inline static unsigned int simi2c_write(Muic_Device* s_hMuicHandle,unsigned char addr, unsigned char *pbuf, unsigned int size, unsigned char sendstop)
{
    unsigned int i = -1;
#ifdef _SIM_I2C_DEBUG_
	    //SIMI2C_DEBUG("[0x%x:0x%x]",addr,pbuf[0]);
#endif     
    
    simi2c_sendstart(s_hMuicHandle);
    simi2c_sendaddr(s_hMuicHandle, addr, 0);
    if (simi2c_getack(s_hMuicHandle))
    {
        for (i = 0; i < size; i++)
        {
            simi2c_sendbyte(s_hMuicHandle, pbuf[i]);
            if (!simi2c_getack(s_hMuicHandle))
            {
#ifdef _SIM_I2C_DEBUG_
		SIMI2C_DEBUG("write error\n");
#endif
                goto clean;
            }
        }
        if (sendstop)
        {
            simi2c_sendstop(s_hMuicHandle);
        }
    }
    
clean:
    return i;
}

//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

static int MUIC_Gpio_i2c_init(Muic_Device *s_hMuicHandle)
{
    if(!s_hMuicHandle->hGpio)
    {
        return -1;
    }

    s_hMuicHandle->hSdaGpioPinHandle = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, MUIC_SDA_PORT, MUIC_SDA_PIN);
    s_hMuicHandle->hSclGpioPinHandle = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, MUIC_SCL_PORT, MUIC_SCL_PIN);

    g_hdelay = HALF_DELAY;
    g_fdelay = FULL_DELAY;

    I2C_SDA_HI;
    I2C_SCL_HI;
    I2C_SDA_DIR(DIR_OUT);
    I2C_SCL_DIR(DIR_OUT);

    return NV_TRUE;
}

static NvBool MUIC_Gpio_Read(Muic_Device *s_hMuicHandle, NvU8 reg, NvU8 *val)
{
//	NvBool ret  = NV_FALSE;
	NvU8 data = 0;

	if ((simi2c_write(s_hMuicHandle,0x44,&reg,1,0)) < 0)
	{
		printk("i2c read error\n");
		return NV_FALSE;
	}

	simi2c_read(s_hMuicHandle,0x44,&data,1,1);
	*val = data;
	return NV_TRUE;
}

static NvBool MUIC_Gpio_Write (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 val)
{
	unsigned char data[2] = {0,};
	NvBool ret = NV_FALSE;
	data[0] = reg;
	data[1] = val;

	if ( simi2c_write(s_hMuicHandle,0x44,data,2,1) < 1)
	{
		printk("i2c read error\n");
		return ret;
	}

	return NV_TRUE;
}
#endif

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *star_muic_proc_file;
#define    STAR_MUIC_PROC_FILE    "driver/hmuic"

#ifndef _MUIC_GPIO_I2C_
static NvBool MUIC_Reg_Write (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 val);
static NvBool MUIC_Reg_Read (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 *val);
#endif

TYPE_MUIC_MODE get_muic_mode(void){
    return muic_mode;
}

EXPORT_SYMBOL(get_muic_mode);

static ssize_t muic_proc_read (struct file *filp, char *buf, size_t len, loff_t *offset)
{
    int i;
    NvU8 val;
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;

    for (i=0; i <= 5; i++) {
#ifdef _MUIC_GPIO_I2C_
        MUIC_Gpio_Read(s_hMuicHandle,i,&val);
#else
        MUIC_Reg_Read(s_hMuicHandle, i, &val);
#endif
        lprintk(D_MUIC, "%s: MUIC Reg 0x%X, Val=0x%X\n", __func__, i, val);
    }
    return 0;
}

static ssize_t muic_proc_write (struct file *filp, const char *buf, size_t len, loff_t *off)
{
    char messages[10];
    NvU32 reg, val;
    int err;
    char cmd;
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;
    
    if(len > 12)
        len = 12;

    if(copy_from_user(messages, buf, len))
        return -EFAULT;

    sscanf(buf, "%c 0x%x 0x%x", &cmd, &reg, &val);

    printk(KERN_INFO "[MUIC] LGE: MUIC_proc_write \n");

    switch(cmd){

        /* AP_UART mode*/
        case '6':
            muic_AP_UART_set(s_hMuicHandle);
            break;

        /* CP_UART mode*/
        case '7':
            muic_CP_UART_set(s_hMuicHandle);
            break;

        /* AP_USB mode*/
        case '8':
            muic_AP_USB_set(s_hMuicHandle);
            break;

        /* CP_USB mode*/
        case '9':
            muic_CP_USB_set(s_hMuicHandle);
            break;

        case 'w':
            dp3t_switch_ctrl(s_hMuicHandle, DP3T_CP_USB);
#ifdef _MUIC_GPIO_I2C_
            err = MUIC_Gpio_Write(s_hMuicHandle, (NvU8)reg, (NvU8)val);
#else
            err = MUIC_Reg_Write(s_hMuicHandle, (NvU8)reg, (NvU8)val);
#endif
            printk(KERN_INFO "[MUIC] LGE: Hub MUIC Write Reg. = 0x%X, Value 0x%X\n", reg, val);
            break;

        default :
                printk(KERN_INFO "[MUIC] LGE: Hub MUIC invalid command\n");
                printk(KERN_INFO "[MUIC] 6: AP_UART, 7: CP_UART, 8: AP_USB, 9: CP_USB\n");
                printk(KERN_INFO "[MUIC] or \"w address value\"\n");
                break;
        }
    return len;
}

static struct file_operations star_muic_proc_ops = {
    .read = muic_proc_read,
    .write = muic_proc_write,
};

static void create_star_muic_proc_file(void)
{
    star_muic_proc_file = create_proc_entry(STAR_MUIC_PROC_FILE, 0777, NULL);
    if (star_muic_proc_file) {
        //star_muic_proc_file->owner = THIS_MODULE;
        star_muic_proc_file->proc_fops = &star_muic_proc_ops;
    } else
        lprintk(D_MUIC, KERN_INFO "%s: MUIC Proc file create failed!\n", __func__);
}

static void remove_star_muic_proc_file(void)
{
    remove_proc_entry(STAR_MUIC_PROC_FILE, NULL);
}
#endif //CONFIG_PROC_FS

void muic_udelay(int A)
{
    A = A/2;
    NvOdmOsWaitUS(A) ;
}

#ifndef _MUIC_GPIO_I2C_
static NvBool MUIC_Reg_Write (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 val)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];

    arr[0] = reg;        // register address
    arr[1] =val;        // u16 value (lsb-msb)
    
    TransactionInfo.Address = s_hMuicHandle->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;
    
    do
    {
        Error = NvOdmI2cTransaction(s_hMuicHandle->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    MUIC_I2C_SPEED_KHZ,
                                    MUIC_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout); 

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMMUIC_PRINTF(("I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, 
                           s_hMuicHandle->DeviceAddr, reg, val));
        return NV_FALSE;
    }

    //NvOdmOsWaitUS( 10000 );  // 10ms
    return NV_TRUE;
}

static NvBool MUIC_Reg_Read (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 *val)
{
    
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    ReadBuffer = reg & 0xFF;

    TransactionInfo[0].Address = (s_hMuicHandle->DeviceAddr);
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;

    TransactionInfo[1].Address = (s_hMuicHandle->DeviceAddr|0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

    do
    {
         status = NvOdmI2cTransaction(s_hMuicHandle->hOdmI2c, TransactionInfo, 2, MUIC_I2C_SPEED_KHZ, MUIC_I2C_TIMEOUT);

    } while(status == NvOdmI2cStatus_Timeout);


    if (status == NvOdmI2cStatus_Success)
    {
         *val = ReadBuffer;
         return NV_TRUE;
    }
    
    return NV_FALSE;

}
#endif

void usif_switch_ctrl(Muic_Device *s_hMuicHandle, TYPE_USIF_MODE mode)
{
    if(mode == USIF_AP)
    {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_USIF1_SW, 0x0);
        lprintk(D_MUIC, "%s: USIF Switch is USIF_UART\n", __func__);
    }
    else if(mode == USIF_DP3T)
    {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_USIF1_SW, 0x1);
        lprintk(D_MUIC, "%s: USIF Switch is USIF_IPC\n", __func__);
    }
    else
    {
        lprintk(D_MUIC, "%s: USIF Switch is USIF_NONE\n", __func__);
    }
}

void dp3t_switch_ctrl(Muic_Device *s_hMuicHandle, TYPE_DP3T_MODE mode)
{
    if(mode == DP3T_AP_UART) {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW, 0x1);
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW, 0x0);
        lprintk(D_MUIC, "%s: DP3T Switch is S1_AP\n", __func__);
    }
    else if(mode == DP3T_CP_UART) {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW, 0x0);
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW, 0x1);
        lprintk(D_MUIC, "%s: DP3T Switch is CP_SW\n", __func__);
    }
    else if(mode == DP3T_CP_USB) {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW, 0x1);
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW, 0x1);
        lprintk(D_MUIC, "%s: DP3T Switch is CP_USB\n", __func__);
    }
    else if (mode == DP3T_NC) {
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW, 0x0);
        NvOdmGpioSetState(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW, 0x0);
        lprintk(D_MUIC, "%s: DP3T Switch is NC \n", __func__);
    }
//    DP3T_mode = mode;
}

NvBool muic_i2c_write_byte (NvU8 reg_addr, NvU8 value)
{
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;
    
#ifdef _MUIC_GPIO_I2C_
    return MUIC_Gpio_Write(s_hMuicHandle, reg_addr, value);
#else
    return MUIC_Reg_Write(s_hMuicHandle, reg_addr, value);
#endif
}


NvBool muic_i2c_read_byte(NvU8 reg_addr, NvU8 *value)
{
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;

#ifdef _MUIC_GPIO_I2C_
    return MUIC_Gpio_Read(s_hMuicHandle, reg_addr, value);
#else
    return MUIC_Reg_Read(s_hMuicHandle, reg_addr, value);
#endif
}

void muic_AP_UART_set(Muic_Device *s_hMuicHandle){

    NvS32 ret;

    /* Turn on charger IC with FACTORY mode */
//20100526, jh.ahn@lge.com, Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_FACTORY_MODE(%d) \n", __func__, CHG_IC_FACTORY_MODE);	
    charging_ic_active(CHG_IC_FACTORY_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Turn on charger IC with FACTORY mode [END]

    /* Connect CP UART signals to AP */
    usif_switch_ctrl(s_hMuicHandle, USIF_AP);
    /* Connect AP UART to MUIC UART */
    dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);
    // Connect DP, DM to UART_TX, UART_RX
    ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
    muic_mode = MUIC_AP_UART;
    lprintk(D_MUIC,KERN_WARNING "%s\n", __func__);

}

void muic_AP_USB_set(Muic_Device *s_hMuicHandle){

    NvS32 ret;

    /* Turn on charger IC with TA mode */
    //	charging_ic_set_ta_mode();
//20100526, jh.ahn@lge.com, Turn on charger IC with Standard USB mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_DEFAULT_MODE(%d) \n", __func__, CHG_IC_DEFAULT_MODE);	
    charging_ic_active(CHG_IC_DEFAULT_MODE); 
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Turn on charger IC with Standard USB mode [END]
	
    /* Connect CP UART signals to AP */
    usif_switch_ctrl(s_hMuicHandle, USIF_AP);
    /* AP USB does not pass through DP3T.
     * Just connect AP UART to MUIC UART. */
    dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
    // USB 2.0 also needs the charge pump (CP_EN) on.
    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);
    // Connect DP, DM to USB_DP, USB_DM
    ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
    muic_mode = MUIC_AP_USB;
    lprintk(D_MUIC,KERN_WARNING "%s\n", __func__);
}

void muic_CP_UART_set(Muic_Device *s_hMuicHandle){

    NvS32 ret;

    /* Turn on charger IC with FACTORY mode */
//20100526, jh.ahn@lge.com, Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_FACTORY_MODE(%d) \n", __func__, CHG_IC_FACTORY_MODE);	
    charging_ic_active(CHG_IC_FACTORY_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Turn on charger IC with FACTORY mode [END]

    /* Connect CP UART signals to DP3T */
    usif_switch_ctrl(s_hMuicHandle, USIF_DP3T);
    /* Connect CP UART to MUIC UART */
    dp3t_switch_ctrl(s_hMuicHandle, DP3T_CP_UART);

    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);
    // Connect DP, DM to UART_TX, UART_RX
    ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
    muic_mode = MUIC_CP_UART;
    lprintk(D_MUIC,KERN_WARNING "%s\n", __func__);
}

void muic_CP_USB_set(Muic_Device *s_hMuicHandle){

    NvS32 ret;

    /* Turn on charger IC with TA mode */
    //	charging_ic_set_ta_mode();
    
//20100526, jh.ahn@lge.com, Turn on charger IC with Standard USB mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_DEFAULT_MODE(%d) \n", __func__, CHG_IC_DEFAULT_MODE);	
    charging_ic_active(CHG_IC_DEFAULT_MODE); 
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Turn on charger IC with Standard USB mode [END]

    /* Connect CP UART signals to AP */
    usif_switch_ctrl(s_hMuicHandle, USIF_AP);
    /* Connect CP USB to MUIC UART */
    dp3t_switch_ctrl(s_hMuicHandle, DP3T_CP_USB);

    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
    // USB 2.0 also needs the charge pump (CP_EN) on.
    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP); 
    // Connect DP, DM to UART_TX, UART_RX
    ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
    muic_mode = MUIC_CP_USB;
    lprintk(D_MUIC,KERN_ERR "%s\n", __func__);
}


/* Initialize MUIC, i.e., the CONTROL_1,2 and SW_CONTROL registers.
 * 1) Prepare to sense INT_STAT and STATUS bits.
 * 2) Open MUIC paths. -> To keep the path from uboot setting, remove this stage.
 */ 
void muic_initialize(TYPE_RESET reset){

    NvBool ret;

    lprintk(D_MUIC,KERN_WARNING "%s\n", __func__);

    /* Reset MUIC - Disable all */
    if(reset){
        ret = muic_i2c_write_byte(CONTROL_1, 0x00);
        ret = muic_i2c_write_byte(CONTROL_2, MUSB_DET_DIS);	// USB_DET_DIS is neg enabled
    }

//20100526, jh.ahn@lge.com, Charging IC Reset [START]
    #if defined(CONFIG_STAR_BATTERY_CHARGER)
    else {
    lprintk(D_MUIC, "%s: charging_ic_deactive call \n", __func__ );
    charging_ic_deactive();
    }
    #endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Charging IC Reset [END]

    /* Initialize MUIC - Default setting.
     *
     * CONTROL_1:
     * 
     * 	ID_2P2 	= 0. Enable to distinguish MUIC_EARMIC from MUIC_TV_OUT_LOAD and MUIC_OTG.
     * 		     Enable for MUIC_EARMIC operation.
     *	ID_620 	= 0. Enable only to distinguish MUIC_TV_OUT_LOAD from MUIC_OTG.
     *	ID_200 	= 1.
     *	VLDO 	= 0. Enable to apply 2.3V for MUIC_EARMIC operation.
     *	SEMREN 	= 1.
     *	ADC_EN 	= 0. Because it is automatically enabled upon any change in ID resistor.
     *	CP_EN 	= 0. Enalbe for USB 2.0 (MUIC_AP_USB, MUIC_CP_USB, and MUIC_OTG).
     *		     Enable for Audio charge pump (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
     * 
     * CONTROL_2: 
     *
     * 	INTPOL 	= 0.
     * 	INT_EN	= 1.
     * 	MIC_LP	= 0.
     * 	CP_AUD 	= 1. Disable for Audio operation (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
     * 	CHG_TYP	= 1.
     * 	USB_DET_DIS = 0. Negative enabled.
     *
     * SW_CONTROL: 
     *
     * 	MIC_ON	= 0. Enable for MUIC_EARMIC and MUIC_TV_OUT_LOAD.
     * 	DP	= 111 (open).
     * 	DM	= 111 (open).
     */
    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
    ret = muic_i2c_write_byte(CONTROL_2, MCP_AUD | MCHG_TYP);
    //	ret = muic_i2c_write_byte(SW_CONTROL, DP_OPEN | DM_OPEN);
    //	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);	// FIXME: For debugging

    /* The initialization time for the facility to set STATUS register's
     * DCPORT and CHPORT = 250msec since CHG_TYP on.
     * The initialization times for the facilities to set INT_STAT register bits
     * since CONTROL_1, 2 settings are much shorter (< 70msec).
     * The settle down time for INT_STAT and STATUS register bits
     * since an interrupt occurs = 70msec.
     * 
     * Thus,
     * we need to wait 250msec if we enable CHG_TYP from its inactive state.
     * And, we need to wait just 70msec for all other cases including
     * interrupt and CONTROL register settings.
     */
    if(reset)
        muic_udelay(250000);
    else
        muic_udelay(70000);

    /* Enable interrupt (INT_EN = 1). Keep other bits the same */
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);

    return;
}

/* Distinguish charger type.
 * This function is called *ONLY IF* INT_STAT's CHGDET == 1, i.e., a charger is detected.
 *
 * Chargers becomes to use the default MUIC setting
 * because the detection always happens only after the MUIC_NONE mode
 * which leads the default MUIC setting.
 * Thus, we don't need to explictly set MUIC registers here.
 */ 
NvBool muic_distinguish_charger(Muic_Device *s_hMuicHandle){

    NvBool ret = NV_TRUE;

    /* Enable charger IC in TA mode */
//20100526, jh.ahn@lge.com, Enable charger IC in TA mode [START]
    #if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_TA_MODE(%d) \n", __func__, CHG_IC_TA_MODE);	
    charging_ic_active(CHG_IC_TA_MODE);
    #endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, jh.ahn@lge.com, Enable charger IC in TA mode [END]

    /* Connect CP UART signals to AP */
    usif_switch_ctrl(s_hMuicHandle, USIF_AP);

    /* Connect AP UART to MUIC UART */
    dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

    /* Read STATUS */
    ret = muic_i2c_read_byte(STATUS, &status_val);
    if(ret == NV_FALSE){
        lprintk(D_MUIC,KERN_INFO "%s: STATUS reading failed\n", __func__);
        muic_mode = MUIC_UNKNOWN;
        return ret;
    }

    /* DCPORT == 1. NA_TA or LG_TA */
    if((status_val & MDCPORT) != 0){

        /* DCPORT == 1 && CHPORT == 1. Failed to detect charger type. Try again.
         * CAUTION!!! Trying again can hang the system. Keep your eye on it closely.
         */
        if((status_val & MCHPORT) != 0){
            lprintk(D_MUIC,KERN_WARNING "%s: Failed to detect charger type. Try again!\n", __func__);
            muic_mode = MUIC_UNKNOWN;
            lprintk(D_MUIC,KERN_WARNING "%s: muic_distinguish_charger(): MUIC_UNKNOWN\n", __func__);
            return NV_FALSE;
        }
        /* DCPORT == 1 && CHPORT == 0. Definitely NA_TA or LG_TA */
        else{
            /* IDNO == 0x05, i.e., 180KOhm. NA_TA */
            if((int_stat_val & MIDNO) == 0x05){
                muic_mode = MUIC_NA_TA;
                lprintk(D_MUIC,KERN_WARNING "%s: MUIC_NA_TA\n", __func__);
            }
            /* IDNO != 0x05. LG_TA by default */
            else{
                muic_mode = MUIC_LG_TA;
                lprintk(D_MUIC,KERN_WARNING "%s: MUIC_LG_TA\n", __func__);
            }
        }
    }
    /* DCPORT == 0. HCHH or INVALID_CHG */
    else{
        /* DCPORT == 0 && CHPORT == 1. Definitely HCHH */
        if((status_val & MCHPORT) != 0){
            muic_mode = MUIC_HCHH;
            lprintk(D_MUIC,KERN_WARNING "%s: MUIC_HCHH\n", __func__);
        }
        /* DCPORT == 0 && CHPORT == 0. Definitely INVALID_CHG */
        else{
            muic_mode = MUIC_INVALID_CHG;
            lprintk(D_MUIC,KERN_WARNING "%s: MUIC_INVALID_CHG\n", __func__);
        }
    }
    return ret;
}

/* Distinguish accessory type which supplies VBUS.
 * These accessories includes AP_UART, CP_UART, AP_USB, and CP_USB.
 * It should be called *ONLY IF* VBUS == 1.
 */
NvBool muic_distinguish_vbus_accessory(Muic_Device *s_hMuicHandle){

    NvBool ret = NV_TRUE;

    switch(int_stat_val & MIDNO){

        /* AP_UART */
        case 0x02:
            muic_AP_UART_set(s_hMuicHandle);
            break;

            /* CP_UART */
        case 0x04:
            muic_CP_UART_set(s_hMuicHandle);
            break;

            /* AP_USB */
        case 0x05:  //20100501 ks.kwon@lge.com Add 180K mode for VBUS Accessory
        case 0x0b:
            muic_AP_USB_set(s_hMuicHandle);
            break;

#if 0
            // CP USB's condition doesn't need to be detected because it's a user scenario.
            /* CP_USB */
        case 0x??:
            muic_CP_USB_set();

            break;
#endif

            /* Default = Unknown accessory */
        default:
            lprintk(D_MUIC,KERN_WARNING "%s: Failed to detect VBUS accessory type. Try again!\n", __func__);
            muic_mode = MUIC_UNKNOWN;
            return NV_FALSE;
            break;
    }
    muic_udelay(1000);	// MUIC DP/DM path settle down and CP_EN delay

    return ret;
}

/* Distinguish accessory type which does not supply VBUS.
 * These accessories includes TV_OUT_NO_LOAD, EARMIC, TV_OUT_LOAD, and OTG.
 * It should be called *ONLY IF* VBUS == 0.
 */
NvBool muic_distinguish_non_vbus_accessory(Muic_Device *s_hMuicHandle){

    NvBool ret = NV_TRUE;
    u8 reg_val = 0;

    switch(int_stat_val & MIDNO){

        /* IDNO == 0x01, i.e., 24KOhm. MUIC_TV_OUT_NO_LOAD */
        case 0x01:
            /* Connect CP UART signals to AP */
            usif_switch_ctrl(s_hMuicHandle, USIF_AP);
            /* DP3T switch has nothing to do with.
             * Just connect AP UART to MUIC UART. */
            dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

            // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
            ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
            // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
            ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);
            // MIC_ON, Connect DP, DM to AUDIO_R, AUDIO_L
            ret = muic_i2c_write_byte(SW_CONTROL, MMIC_ON | DP_AUDIO | DM_AUDIO);
            muic_mode = MUIC_TV_OUT_NO_LOAD;
            break;

            /* IDNO == 0x00. MUIC_EARMIC, MUIC_TV_OUT_LOAD, or MUIC_OTG */
        case 0x00:
            /* Apply 2.2KOhm resistor */
            // ID_2P2, VLDO 2.6V, SEMREN on. ADC is auto.
            ret = muic_i2c_write_byte(CONTROL_1, MID_2P2 | MSEMREN);
            // INT_EN disable. Keep CP_AUD, CHG_TYP, USB_DET_DIS the same.
            ret = muic_i2c_write_byte(CONTROL_2, MCP_AUD | MCHG_TYP);

            /* Wait for 70msec */
            muic_udelay(70000);

            /* Read INT_STAT */
            ret = muic_i2c_read_byte(INT_STAT, &reg_val);
            if(ret == NV_FALSE){
                lprintk(D_MUIC,KERN_INFO "%s: INT_STAT reading failed\n", __func__);
                muic_mode = MUIC_UNKNOWN;
                return ret;
            }

            /* If IDNO with 2.2KOhm is 0x04, 0x05, 0x06, or 0x07, it's EARMIC */
            if((reg_val & MIDNO) != 0x00){
                /* Connect CP UART signals to AP */
                usif_switch_ctrl(s_hMuicHandle, USIF_AP);
                /* DP3T switch has nothing to do with.
                 * Just connect AP UART to MUIC UART. */
                dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

                // ID_2P2, VLDO 2.3V, SEMREN on. ADC is auto.
                // EARMIC, TV_OUT_LOAD also requires the charge pump (CP_EN) on.
                ret = muic_i2c_write_byte(CONTROL_1, MID_2P2 | MVLDO | MSEMREN | MCP_EN);
                // INT_EN, CP_AUD off, CHG_TYP, USB_DET_DIS enable.
                ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP);
                // MIC_ON, Connect DP, DM to AUDIO_R, AUDIO_L
                ret = muic_i2c_write_byte(SW_CONTROL, MMIC_ON | DP_AUDIO | DM_AUDIO);
                muic_mode = MUIC_EARMIC;
            }
            /* Otherwise, it can be either TV_OUT_LOAD or OTG */
            else{
                /* Apply 620Ohm resistor */
                // ID_620, VLDO 2.6V, SEMREN on. ADC is auto.
                ret = muic_i2c_write_byte(CONTROL_1, MID_620 | MSEMREN);
                // INT_EN disable. Keep CP_AUD, CHG_TYP, USB_DET_DIS the same.
                ret = muic_i2c_write_byte(CONTROL_2, MCP_AUD | MCHG_TYP);

                /* Wait for 70msec */
                muic_udelay(70000);

                /* Read INT_STAT */
                ret = muic_i2c_read_byte(INT_STAT, &reg_val);
                if(ret == NV_FALSE){
                    lprintk(D_MUIC,KERN_INFO "%s: INT_STAT reading failed\n", __func__);
                    muic_mode = MUIC_UNKNOWN;
                    return ret;
                }

                /* If IDNO with 620Ohm is 0x01, it's TV_OUT_LOAD */
                if((reg_val & MIDNO) == 0x01){
                    /* Connect CP UART signals to AP */
                    usif_switch_ctrl(s_hMuicHandle, USIF_AP);
                    /* DP3T switch has nothing to do with.
                     * Just connect AP UART to MUIC UART. */
                    dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

                    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
                    // EARMIC, TV_OUT_LOAD also requires the charge pump (CP_EN) on.
                    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
                    // INT_EN, CP_AUD off, CHG_TYP, USB_DET_DIS enable.
                    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP);
                    // MIC_ON (Video), Connect DP, DM to AUDIO_R, AUDIO_L
                    ret = muic_i2c_write_byte(SW_CONTROL, MMIC_ON | DP_AUDIO | DM_AUDIO);
                    muic_mode = MUIC_TV_OUT_LOAD;
                }
                /* If IDNO with 620Ohm is 0x00, it's OTG */
                else if((reg_val & MIDNO) == 0x01){
                    /* Connect CP UART signals to AP */
                    usif_switch_ctrl(s_hMuicHandle, USIF_AP);
                    /* DP3T switch has nothing to do with.
                     * Just connect AP UART to MUIC UART. */
                    dp3t_switch_ctrl(s_hMuicHandle, DP3T_AP_UART);

                    // ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
                    // USB 2.0 also needs the charge pump (CP_EN) on.
                    ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
                    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
                    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP);
                    // Connect DP, DM to USB_DP, USB_DM
                    ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
                    muic_mode = MUIC_OTG;
                }
                /* Otherwise, it's an unknown accessory */
                else{
                    lprintk(D_MUIC,KERN_WARNING "%s: Failed to detect non VBUS accessory. Try again!", __func__);
                    muic_mode = MUIC_UNKNOWN;
                    return NV_FALSE;
                }
            }
            break;
            /* No other accessory is supported */
        default:
            lprintk(D_MUIC,KERN_WARNING "%s: Failed to detect non VBUS accessory. Try again!", __func__);
            muic_mode = MUIC_UNKNOWN;
            return NV_FALSE;
            break;
    }
    muic_udelay(1000);	// MUIC DP/DM path settle down delay

    return ret;
}

/*
 * ID_200 Set		Connect UID LDO to ID pin via 200KOhm
 * VLDO Reset		Use 2.6V for all accessory detection (except for MIC bias)
 * SEMREN Set		Enable Send-End, Mic removal comparators and UID LDO -> Sets MR_COMP, SEND/END bits
 * ADC_EN Set		Enable ADC and UID LDO to generate IDNO. TI MUIC turns on this automatically.
 * CP_EN Set		Enable charge pump for USB2.0 and headset.
 * INT_EN Set		Enable MUIC interrupt generation to OMAP
 * CHG_TYP Set		Enable charger type recognition (TA or HCHH) -> Sets DCPORT, CHPORT bits
 * USB_DET_DIS Reset	Enable charger detection (Charger or USB) -> Sets CHGDET bit
 *
 * MUIC MODE
 *
 * V C D C IDNO IDNO IDNO
 * B H C H 200K 2.2K 620
 * U G P P
 * S D O O
 *   E R R
 *   T T T
 *Cases detected by muic_distinguish_charger():
 * 1 1 1 - 0101 ---- ---- NA_TA (ID resistor 180KOhm) - Not used actually.
 * 1 1 1 - ---- ---- ---- LG_TA
 * 1 1 0 1 ---- ---- ---- HCHH (High current host/hub charger) - Not used actually.
 * 1 1 0 0 ---- ---- ---- Invalid charger
 *
 *Cases detected by muic_distinguish_vbus_accessory():
 * 1 0 0 0 0010 ---- ---- AP_UART (ID resistor 56KOhm)
 * 1 0 0 0 0100 ---- ---- CP_UART (ID resistor 130KOhm)
 * 1 0 0 0 1011 ---- ---- AP_USB (ID resistor open)
 * 1 0 0 0 ???? ---- ---- CP_USB (ID resistor ????) - Not defined yet.
 *
 *Cases detected by muic_distinguish_non_vbus_accessory():
 * 0 0 - - 0001 ---- ---- TV_OUT_NO_LOAD (ID resistor 24KOhm.) - Not used.
 * 0 0 - - 0000 01XX ---- EARMIC (ID resistor ground)
 * 0 0 - - 0000 0000 0001 TV_OUT_LOAD (ID resistor ground) - Not used.
 * 0 0 - - 0000 0000 0000 OTG (ID resistor ground) - Not used.
 */
NvBool TS5USBA33402_device_detection(Muic_Device *s_hMuicHandle, NvS32 upon_irq){

    NvBool ret = NV_TRUE;

    /* Upon an MUIC IRQ (MUIC_INT_N falls),
     * wait 70ms before reading INT_STAT and STATUS.
     * After the reads, MUIC_INT_N returns to high
     * (but the INT_STAT and STATUS contents will be held).
     *
     * Do this only if TS5USBA33402_device_detection() was called upon IRQ. */
    if(upon_irq) muic_udelay(70000);

    /* Read INT_STAT */
    ret = muic_i2c_read_byte(INT_STAT, &int_stat_val);
    if(ret == NV_FALSE){
        lprintk(D_MUIC,KERN_INFO "%s: INT_STAT reading failed\n", __func__);
        muic_mode = MUIC_UNKNOWN;
        return ret;
    }
    lprintk(D_MUIC,KERN_INFO "%s: IDNO = 0x%x\n", __func__, (int_stat_val & MIDNO));

    /* Branch according to the previous muic_mode */
    switch(muic_mode){

        /* MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
         * First, at the initialization time where the muic_mode is not available yet.
         * Second, whenever the current muic_mode detection is failed.
         */
        case MUIC_UNKNOWN:

            /* If the previous muic_mode was MUIC_NONE,
             * the only possible condition for a MUIC IRQ is plugging in an accessory.
             */
        case MUIC_NONE:

            /* VBUS == 1.
             * MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, MUIC_INVALID_CHG,
             * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, or MUIC_CP_USB.
             */
            if((int_stat_val & MVBUS) != 0){
                /* CHGDET == 1, i.e., D+/D- are short. Charger is detected.
                 * MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, or MUIC_INVALID_CHG.
                 */
                if((int_stat_val & MCHGDET) != 0){
                    /* Charger type disctinction */
                    ret = muic_distinguish_charger(s_hMuicHandle);
                }
                /* CHGDET == 0, i.e., D+/D- are not short. No charger is detected.
                 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB.
                 */
                else{
                    /* VBUS accessory type disctinction */
                    ret = muic_distinguish_vbus_accessory(s_hMuicHandle);
                }
            }
            /* VBUS == 0.
             * MUIC_TV_OUT_NO_LOAD, MUIC_EARMIC, MUIC_TV_OUT_LOAD, MUIC_OTG, or MUIC_NONE.
             */
            else{
                /* IDNO == 0x0b, i.e., ID pin is open. No accessory is plugged in.
                 * MUIC_NONE.
                 */
                if((int_stat_val & MIDNO) == 0x0b){
                    muic_mode = MUIC_NONE;
                }
                /* IDNO != 0x0b, i.e., ID pin is not open. An accessory is plugged in.
                 * MUIC_TV_OUT_NO_LOAD, MUIC_EARMIC, MUIC_TV_OUT_LOAD, or MUIC_OTG.
                 */
                else{
                    /* Non VBUS accessory type distinction */
                    ret = muic_distinguish_non_vbus_accessory(s_hMuicHandle);
                }
            }
            break;

            /* If the previous muic_mode was MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, MUIC_INVALID_CHG,
             * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
             * the only possible condition for a MUIC IRQ is plugging out the accessory.
             * 
             * In this case, initialize MUIC and wait an IRQ.
             * We don't need to wait 250msec because this is not an erronous case
             * (we need to reset the facility to set STATUS for an erronous case and
             * have to wait 250msec) and, if this is not an erronous case, the facility
             * was already initialized at the system booting.
             */
        case MUIC_NA_TA:
        case MUIC_LG_TA:
        case MUIC_HCHH:
        case MUIC_INVALID_CHG:
        case MUIC_AP_UART:
        case MUIC_CP_UART:
        case MUIC_AP_USB:
        case MUIC_CP_USB:
            /* Check if VBUS == 0 && IDNO == 0x1011 (open).
             * If so, it's MUIC_NONE.
             * Otherwise, it's an erronous situation. MUIC_UNKNOWN.
             */
            if((int_stat_val & MVBUS) == 0 && (int_stat_val & MIDNO) == 0x0b){
                muic_mode = MUIC_NONE;
                lprintk(D_MUIC,KERN_WARNING "%s: MUIC_NONE\n", __func__);
                /* Connect CP UART signals to AP */
                usif_switch_ctrl(s_hMuicHandle, USIF_AP);
                /* DP3T switch has nothing to do with.
                 * Open DP3T */
                dp3t_switch_ctrl(s_hMuicHandle, DP3T_NC);	
                //  Open DP, DM
                ret = muic_i2c_write_byte(SW_CONTROL, DP_OPEN | DM_OPEN);				

            }
            else{
                muic_mode = MUIC_UNKNOWN;
                ret = NV_FALSE;
                lprintk(D_MUIC,KERN_WARNING "%s: Failed to distinguish muic_mode. Try again!\n", __func__);
            }
            break;

        default:
            lprintk(D_MUIC,KERN_WARNING "%s: Failed to detect an accessory. Try again!", __func__);
            muic_mode = MUIC_UNKNOWN;
            ret = NV_FALSE;
            break;
    }	

    // INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
    ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCP_AUD | MCHG_TYP); //interrupt enabled by ks.kwon@lge.com

    if(muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE){
        muic_initialize(DEFAULT);
//        charging_ic_deactive();		//20100506 ks.kwon@lge.com for charging animation, by demand from taehwan.kim
//        printk(KERN_INFO "[MUIC]charging_ic_deactive()\n");
    }
    return ret;
}

static void muic_wq_func(struct work_struct *muic_wq)
{
    NvBool ret = NV_TRUE;
    u32 retry_no;
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;
    
    ret = TS5USBA33402_device_detection(s_hMuicHandle, UPON_IRQ);
    /* If an erronous situation occurs, try again */
    retry_no = 0;
    while(ret == NV_FALSE && retry_no < 3){
        lprintk(D_MUIC,KERN_INFO "%s: TS5USBA33402_device_detection() failed %d times\n", __func__, retry_no);
        ret = TS5USBA33402_device_detection(s_hMuicHandle, NOT_UPON_IRQ);
        retry_no ++;
    }
}


static void muic_interrupt_handler(void* arg)
{
    Muic_Device *s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;
    
    lprintk(D_MUIC, "%s: MUIC interrupt occured!\n", __func__);
    schedule_work(&muic_wq);
    NvOdmGpioInterruptDone(s_hMuicHandle->hGpioInterrupt);
}

static int __init muic_probe(struct platform_device *pdev)
{
    int i, j, retry_no;
#ifdef _MUIC_GPIO_I2C_
    int ret_val;
#else
    NvU32 I2cInstance = 0;
#endif
    NvU32 I2cAdress = 0;
    NvU32 pin[4], port[4];
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvBool ret;
    Muic_Device *s_hMuicHandle;

    pConnectivity = NvOdmPeripheralGetGuid(MUIC_GUID);

    for (i = 0, j = 0 ; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
#ifndef _MUIC_GPIO_I2C_
            case NvOdmIoModule_I2c:
                I2cAdress = (pConnectivity->AddressList[i].Address << 1);
                I2cInstance = pConnectivity->AddressList[i].Instance;
                break;
#endif
            case NvOdmIoModule_Gpio:
                port[j] = pConnectivity->AddressList[i].Instance;
                pin[j] = pConnectivity->AddressList[i].Address;
                j++;
                break;
            default:
                break;
        }
    }

    s_hMuicHandle  = NvOdmOsAlloc(sizeof(Muic_Device));
    hMuicHandle = s_hMuicHandle;
    
    if(!s_hMuicHandle)
    {
        lprintk(D_MUIC, "%s: NvOdmOsAlloc Error \n", __func__);
            return -ENOSYS;
    }
     
    NvOdmOsMemset(s_hMuicHandle, 0, sizeof(Muic_Device));

    s_hMuicHandle->DeviceAddr = I2cAdress;

    s_hMuicHandle->hGpio = NvOdmGpioOpen();
    if (!s_hMuicHandle->hGpio)
    {
        lprintk(D_MUIC, "%s: NvOdmGpioOpen Error \n", __func__);
        goto err_open_gpio_fail;
    }

    s_hMuicHandle->h_INT_N_MUIC = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, port[0], pin[0]);
    if (!s_hMuicHandle->h_INT_N_MUIC)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin h_INT_N_MUIC \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle->h_AP20_UART_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, port[1], pin[1]);
    if (!s_hMuicHandle->h_AP20_UART_SW)
    {
        lprintk(D_MUIC, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin h_AP20_UART_SW\n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle->h_IFX_UART_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, port[2], pin[2]);
    if (!s_hMuicHandle->h_IFX_UART_SW)
    {
        lprintk(D_MUIC, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin h_IFX_UART_SW\n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle->h_USIF1_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle->hGpio, port[3], pin[3]);
    if (!s_hMuicHandle->h_USIF1_SW)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin h_USIF1_SW\n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->h_INT_N_MUIC, NvOdmGpioPinMode_InputData);
    NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW, NvOdmGpioPinMode_Output);
    NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW, NvOdmGpioPinMode_Output);
    NvOdmGpioConfig(s_hMuicHandle->hGpio, s_hMuicHandle->h_USIF1_SW, NvOdmGpioPinMode_Output);

#ifdef _MUIC_GPIO_I2C_
    ret_val = MUIC_Gpio_i2c_init(s_hMuicHandle);
    if (ret_val < 0)
    {   
        lprintk(D_MUIC, "%s: MUIC_Gpio_i2c_init Error \n", __func__);
        goto err_open_i2c_handle;
    }
#else
    s_hMuicHandle->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
    if (!s_hMuicHandle->hOdmI2c)
    {
        lprintk(D_MUIC, "%s: NvOdmI2cOpen Error \n", __func__);
        goto err_open_i2c_handle;
    }
#endif
    INIT_WORK(&muic_wq, muic_wq_func);

    if (NvOdmGpioInterruptRegister(s_hMuicHandle->hGpio, &s_hMuicHandle->hGpioInterrupt,
        s_hMuicHandle->h_INT_N_MUIC, NvOdmGpioPinMode_InputInterruptFallingEdge, muic_interrupt_handler,
        (void*)&s_hMuicHandle, 0) == NV_FALSE)
    {
        lprintk(D_MUIC, KERN_ERR "%s: cannot register interrupt.\n", __func__);
        goto err_get_interrupt_handler;
    }
	
    create_star_muic_proc_file();

    /* Initialize switches.
     * By default, set the analog switch so as to connect CP UART to DP3T switch
     * and set the DP3T switch so as to connect AP UART to MUIC.
     */

    /* Initialize MUIC - Finally MUIC INT becomes enabled */
    muic_initialize(RESET);
#if 1
    ret = TS5USBA33402_device_detection(s_hMuicHandle, NOT_UPON_IRQ);
    /* If an erronous situation occurs, try again */
    retry_no = 0;
    while(ret == NV_FALSE && retry_no < 3){
        lprintk(D_MUIC,KERN_INFO "%s: TS5USBA33402_device_detection() failed %d times\n", __func__, retry_no);
        ret = TS5USBA33402_device_detection(s_hMuicHandle, NOT_UPON_IRQ);
        retry_no ++;
    }
#endif

    printk(KERN_WARNING "[MUIC] muic_probe()\n");


    return 0;

err_get_interrupt_handler:
#ifndef _MUIC_GPIO_I2C_
    NvOdmI2cClose(s_hMuicHandle->hOdmI2c);
#endif
err_open_i2c_handle:
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_INT_N_MUIC);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_USIF1_SW);
err_open_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_hMuicHandle->hGpio);
err_open_gpio_fail:
    NvOdmOsFree(s_hMuicHandle);
    return -ENOSYS;
}

static int muic_remove(struct platform_device *pdev)
{
    Muic_Device * s_hMuicHandle;
    s_hMuicHandle = hMuicHandle;
    
    NvOdmGpioInterruptUnregister(s_hMuicHandle->hGpio, s_hMuicHandle->h_INT_N_MUIC,
        s_hMuicHandle->hGpioInterrupt);

    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_INT_N_MUIC);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_AP20_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_IFX_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle->hGpio, s_hMuicHandle->h_USIF1_SW);

    NvOdmGpioClose(s_hMuicHandle->hGpio);
#ifndef _MUIC_GPIO_I2C_
    NvOdmI2cClose(s_hMuicHandle->hOdmI2c);
#endif
    NvOdmOsFree(s_hMuicHandle);
    remove_star_muic_proc_file();
    return 0;
}

static int muic_suspend(struct platform_device *pdev, pm_message_t state)
{
    lprintk(D_MUIC, "%s \n", __func__);
    return 0;
}
        
static int muic_resume(struct platform_device *pdev)
{
    lprintk(D_MUIC, "%s \n", __func__);
    return 0;
}
        
static struct platform_driver muic_driver = {
    .probe     = muic_probe,
    .remove    = muic_remove,
    .suspend   = muic_suspend,
    .resume    = muic_resume,
    .driver    = {
        .name = "star_muic",
		.owner = THIS_MODULE,
    },
};

static int __devinit muic_init(void)
{
    return platform_driver_register(&muic_driver);
}

static void __exit muic_exit(void)
{
    platform_driver_unregister(&muic_driver);
}

module_init(muic_init);
module_exit(muic_exit);

MODULE_AUTHOR("jm1.lee@lge.com");
MODULE_DESCRIPTION("star MUIC Driver");
MODULE_LICENSE("GPL");
