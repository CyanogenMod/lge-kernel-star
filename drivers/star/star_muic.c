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
//20101117, , for autorun
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
#include <linux/switch.h>
#endif

#include "mach/nvrm_linux.h"
#include "nvodm_query_discovery.h"

#include <mach/lprintk.h>
#include "star_muic.h"

#include "mach/lprintk.h"

#define MUIC_I2C_SPEED_KHZ       400
#define MUIC_I2C_TIMEOUT         500
#define MUIC_GUID                NV_ODM_GUID('s','t','a','r','m','u','i','c')
#define NVODMMUIC_PRINTF(x)      NvOdmOsDebugPrintf x

#if defined(CONFIG_MACH_STAR)
enum 
{ 
    DISABLE = 0x0, 
    ENABLE  = 0x1
};
#endif
#ifndef CONFIG_MACH_STAR_TMUS
typedef enum
{
    USIF_UART,
    USIF_IPC,
    USIF_NONE,
} USIF_MODE_TYPE;
#endif

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
#if defined(CONFIG_MACH_STAR)
#ifdef CONFIG_MACH_STAR_REV_F
    NvOdmGpioPinHandle h_USIF1_SW;
#endif
    NvOdmGpioPinHandle h_USB_VBUS_EN;
#else
    NvOdmGpioPinHandle h_USIF1_SW;    
#endif
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
    NvU32 DeviceAddr;    
    NvU32 VddId;
#ifdef _MUIC_GPIO_I2C_
    NvOdmGpioPinHandle      hSclGpioPinHandle;
    NvOdmGpioPinHandle      hSdaGpioPinHandle;
#endif	
    struct wake_lock wlock;
//20101117, , for autorun
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
    struct switch_dev sdev_autorun;
#endif
} Muic_Device;

//20101117, , for autorun
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
#define	DRIVER_NAME_FOR_AUTORUN		"TEGRA_UDC_AUTORUN"
#endif

//20100526, , charging_ic Function [START]
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
extern max8922_status get_charging_ic_status(void);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
//20100526, , charging_ic Function [END]

static star_shutdown = 0;
//20101117, , for autorun [START]
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
static ssize_t print_switch_name_for_autorun(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME_FOR_AUTORUN);
}

static ssize_t print_switch_state_for_autorun(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n",
			(sdev->state ? "online" : "offline"));
}
#endif
//20101117, , for autorun [END]

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

    __inline static void simi2c_sendnack(Muic_Device* s_hMuicHandle)
    {
        I2C_SDA_HI;
        I2C_SDA_OUT;
        I2C_SCL_HI;
        I2C_DELAY_FULL;
        I2C_SCL_LO;
        I2C_DELAY_HALF;
#ifdef _SIM_I2C_DEBUG_
        SIMI2C_DEBUG("[NA]");
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
                if( i != (size-1) ) 	// if last bit, nack should be send (for MAXIM MUIC)
                {
                    simi2c_sendack(s_hMuicHandle);
                }
                else
                {
                    simi2c_sendnack(s_hMuicHandle);
                }
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
#endif

NvBool muic_factory_audio_mode = NV_FALSE;
max14526_device_type current_device = DEVICE_NONE;

static Muic_Device s_hMuicHandle;
static struct delayed_work muic_wq;

static max14526_device_type boot_muic_state = DEVICE_NONE;
//static NvBool send_key_muic_hook_pressed = NV_FALSE;
//DP3T_MODE_TYPE  DP3T_mode = DP3T_NC;

#if !defined(CONFIG_MACH_STAR_TMUS)
static void USIF_ctrl(USIF_MODE_TYPE mode);
#endif
static void DP3T_Switch_ctrl(DP3T_MODE_TYPE mode);

static DEFINE_MUTEX(muic_mutex);
static int muic_set_state(const char *val, struct kernel_param *kp);
static int muic_get_state(char *buffer, struct kernel_param *kp);
module_param_call(muic_current_device, muic_set_state, muic_get_state,
        &current_device, 0664);
MODULE_PARM_DESC(current_device, "MUIC current device");

static int muic_set_state(const char *val, struct kernel_param *kp)
{
    int ret = 0;

    return ret;
}

static int muic_get_state(char *buffer, struct kernel_param *kp)
{
    int ret;
    mutex_lock(&muic_mutex);
    ret = sprintf(buffer, "%d", current_device);
    mutex_unlock(&muic_mutex);
    return ret;
}

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *star_muic_proc_file;
#define    STAR_MUIC_PROC_FILE    "driver/hmuic"

static NvBool MUIC_Reg_Write (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 val);
static NvBool MUIC_Reg_Read (Muic_Device* s_hMuicHandle, NvU8 reg, NvU8 *val);

void Set_MAX14526_Develop_Mode_Detect(void);
void Set_MAX14526_Factory_Mode_Detect(void);
void Set_MAX14526_Usb_Mode_Detect(void);
void Set_MAX14526_CP_USB_Mode(void);
void Set_MAX14526_CP_USB_Mode_PortSetting(void);

max14526_device_type read_device_type (void)
{
    return current_device;
}
EXPORT_SYMBOL(read_device_type);

#if defined(CONFIG_MACH_STAR)
int cp_emergency_download(void)
{
    NvOdmServicesGpioHandle hCpEmergencgy;
    NvOdmGpioPinHandle hTestGpio01;
    NvOdmGpioPinHandle hMDMRest;

    hCpEmergencgy = NvOdmGpioOpen();
    if (!hCpEmergencgy)
    {
        lprintk(D_MUIC, "%s: NvOdmGpioOpen Error \n", __func__);
        goto error_open_gpio_fail;
    }

    hTestGpio01 = NvOdmGpioAcquirePinHandle(hCpEmergencgy, 'h' - 'a', 1);
    if (!hTestGpio01)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto error_open_gpio_pin_acquire_fail;
    }

    hMDMRest = NvOdmGpioAcquirePinHandle(hCpEmergencgy, 'v' - 'a', 0);
    if (!hMDMRest)
    {
        lprintk(D_MUIC, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
	    goto error_open_gpio_pin_acquire_fail;
	}

    //change TEST_GPIO state from high to low (GPIO_PH1)
    NvOdmGpioSetState(hCpEmergencgy, hTestGpio01, 0x0);
    NvOdmGpioConfig(hCpEmergencgy, hTestGpio01, NvOdmGpioPinMode_Output);

    //reset Communication Processor (GPIO_PV0: high -> 1000ms -> low -> 3000ms -> high)
    NvOdmGpioSetState(hCpEmergencgy, hMDMRest, 0x1);
    NvOdmGpioConfig(hCpEmergencgy, hMDMRest, NvOdmGpioPinMode_Output);
    msleep(1000);
    NvOdmGpioSetState(hCpEmergencgy, hMDMRest, 0x0);
    msleep(3000);
    NvOdmGpioSetState(hCpEmergencgy, hMDMRest, 0x1);

    //change USB path from AP to CP
    Set_MAX14526_CP_USB_Mode_PortSetting();

	NvOdmGpioReleasePinHandle(hCpEmergencgy, hMDMRest);
	NvOdmGpioReleasePinHandle(hCpEmergencgy, hTestGpio01);
    return 0;

error_open_gpio_pin_acquire_fail:
    NvOdmGpioClose(hCpEmergencgy);
error_open_gpio_fail:
    return -ENOSYS;
}
#endif

static ssize_t muic_proc_read (struct file *filp, char *buf, size_t len, loff_t *offset)
{
    int i;
    NvU8 val;

    for (i=0; i <= 5; i++) {
        MUIC_Reg_Read(&s_hMuicHandle, i, &val);
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

    if (len > 12)
        len = 12;

    if (copy_from_user(messages, buf, len))
        return -EFAULT;

    sscanf(buf, "%c 0x%x 0x%x", &cmd, &reg, &val);

    lprintk(D_MUIC, "%s: MUIC_proc_write \n", __func__);
    printk("-->>MUIC : buf : %s\n",buf);
    printk("-->>MUIC : %s %c 0x%x 0x%x \n",__func__, cmd, reg, val);

    switch(cmd){
#if defined(CONFIG_MACH_STAR)
        case '3':
            /* CP_Image Download mode*/
            cp_emergency_download();
            break;

        case '4':
            /* CP_ USB VBUS DISABLE*/
            NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
            break;
            
        case '5':
            /* CP USB VBUS ENABLE*/
            NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, ENABLE);
            break;
#endif
            /* AP_UART mode*/
        case '6':
            Set_MAX14526_Develop_Mode_Detect();
            break;

            /* CP_UART mode*/
        case '7':
            Set_MAX14526_Factory_Mode_Detect();
            break;

            /* AP_USB mode*/
        case '8':
            Set_MAX14526_Usb_Mode_Detect();
            break;

            /* CP_USB mode*/
        case '9':
            Set_MAX14526_CP_USB_Mode_PortSetting();
            break;

        case 'w':
            err = MUIC_Reg_Write(&s_hMuicHandle, (NvU8)reg, (NvU8)val);
            lprintk(D_MUIC, "%s: MUIC Write Reg. = 0x%X, Value 0x%X\n", __func__,  reg, val);
            break;

        default :
            printk(KERN_INFO "[MUIC] LGE: Star MUIC invalid command\n");
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
        star_muic_proc_file->proc_fops = &star_muic_proc_ops;
    } else
        lprintk(D_MUIC, KERN_INFO "%s: MUIC Proc file create failed!\n", __func__);
}

static void remove_star_muic_proc_file(void)
{
    remove_proc_entry(STAR_MUIC_PROC_FILE, NULL);
}
#endif 

int max14526_muic_init(TYPE_RESET reset)
{
    int ret;

    /* clear default switch position */ 
    // COMP2 to H-Z /COMN1 to H-Z (0x03=0x24)    
    ret=  MUIC_Reg_Write(&s_hMuicHandle, SW_CTRL_REG, COMP2_TO_HZ|COMN1_TO_HZ);
    // Enable 200K, ADC (0x01=0x12)
    ret=  MUIC_Reg_Write(&s_hMuicHandle, CTRL1_REG, ID_200_M|ADC_EN_M);
    // Enable Interrupts (0x02 = 0x40)
    ret=  MUIC_Reg_Write(&s_hMuicHandle, CTRL2_REG, INT_EN_M);
   
    DP3T_Switch_ctrl(DP3T_NC);
#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_IPC);
#endif
    //20100526, , Charging IC Reset [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    if ( !reset )
    {
        lprintk(D_MUIC, "%s: charging_ic_deactive call \n", __func__ );
        charging_ic_deactive();
    }
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Charging IC Reset [END]

    wake_unlock(&s_hMuicHandle.wlock);
    current_device = DEVICE_NONE;
    return ret;
}

static NvBool MUIC_Reg_Write (Muic_Device* hMuicHandle, NvU8 reg, NvU8 val)
{
#ifdef _MUIC_GPIO_I2C_
    unsigned char data[2] = {0,};
    NvBool ret = NV_FALSE;
    data[0] = reg;
    data[1] = val;

    if ( simi2c_write(hMuicHandle,0x44,data,2,1) < 1)
    {
        printk("i2c read error\n");
        return ret;
    }

    return NV_TRUE;
#else
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];

    arr[0] = reg;        // register address
    arr[1] =val;        // u16 value (lsb-msb)

    TransactionInfo.Address = hMuicHandle->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    do
    {
        Error = NvOdmI2cTransaction(hMuicHandle->hOdmI2c,
                &TransactionInfo,
                1,
                MUIC_I2C_SPEED_KHZ,
                MUIC_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout); 

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMMUIC_PRINTF(("I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, 
                    hMuicHandle->DeviceAddr, reg, val));
        return NV_FALSE;
    }

    return NV_TRUE;
#endif
}

static NvBool MUIC_Reg_Read (Muic_Device* hMuicHandle, NvU8 reg, NvU8 *val)
{
#ifdef _MUIC_GPIO_I2C_
    NvU8 data = 0;

    if ((simi2c_write(hMuicHandle,0x44,&reg,1,0)) < 0)
    {
        printk("i2c read error\n");
        return NV_FALSE;
    }

    simi2c_read(hMuicHandle,0x44,&data,1,1);
    *val = data;
    return NV_TRUE;
#else
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    ReadBuffer = reg & 0xFF;

    TransactionInfo[0].Address = (hMuicHandle->DeviceAddr);
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;

    TransactionInfo[1].Address = (hMuicHandle->DeviceAddr|0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

    do
    {
        status = NvOdmI2cTransaction(hMuicHandle->hOdmI2c, TransactionInfo, 2, MUIC_I2C_SPEED_KHZ, MUIC_I2C_TIMEOUT);

    } while(status == NvOdmI2cStatus_Timeout);

    if (status == NvOdmI2cStatus_Success)
    {
        *val = ReadBuffer;
        return NV_TRUE;
    }

    return NV_FALSE;
#endif
}

#if !defined(CONFIG_MACH_STAR_TMUS)
void USIF_ctrl(USIF_MODE_TYPE mode)
{
    if(mode == USIF_UART)
    {
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, 0x1);
        lprintk(D_MUIC, "%s: USIF Switch is USIF_UART\n", __func__);
    }
    else if(mode == USIF_IPC)
    {
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, 0x0);
        lprintk(D_MUIC, "%s: USIF Switch is USIF_IPC\n", __func__);
    }
    else
    {
        lprintk(D_MUIC, "%s: USIF Switch is USIF_NONE\n", __func__);
    }
}
#endif

void DP3T_Switch_ctrl(DP3T_MODE_TYPE mode)
{
    if(mode == DP3T_S1_AP) {
#if defined(CONFIG_MACH_STAR)
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x1);
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x0);
        lprintk(D_MUIC, "%s: DP3T Switch is S1_AP\n", __func__);
    }
    else if(mode == DP3T_S2_CP_SW) {
#if defined(CONFIG_MACH_STAR)
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x0);
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x1);
        lprintk(D_MUIC, "%s: DP3T Switch is CP_SW\n", __func__);
    }
    else if(mode == DP3T_S3_CP_USB) {
#if defined(CONFIG_MACH_STAR)
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, ENABLE);
#endif
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x1);
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x1);
        lprintk(D_MUIC, "%s: DP3T Switch is CP_USB\n", __func__);
    }
    else if (mode == DP3T_NC) {
#if defined(CONFIG_MACH_STAR)
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, 0x0);
        NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, 0x0);
        lprintk(D_MUIC, "%s: DP3T Switch is NC \n", __func__);
    }
    //    DP3T_mode = mode;
}


NvBool Set_MAX14526_ADDR (NvU8 reg_addr, NvU8 value)
{
    MUIC_Reg_Write(&s_hMuicHandle, reg_addr, value);
    return NV_TRUE;
}


NvU8 Get_MAX14526_ADDR(NvU8 reg_addr)
{
    NvU8 reg_buff = 0xff;
    MUIC_Reg_Read(&s_hMuicHandle, reg_addr, &reg_buff);
    return reg_buff;
}

void Set_MAX14526_Charger_Detect(NvU8 int_status_reg)
{
    NvU8 reg_value;
    printk("-->> MUIC : %s\n",__func__);
    
    // charger type detection (0x02=0x02)
    Set_MAX14526_ADDR(CTRL2_REG, CHG_TYPE_M);

    // This line should be modified by a customer.
    NvOdmOsSleepMS(250); 
    
    // Read INT_STAT_REG (0x04)
    reg_value = Get_MAX14526_ADDR(INT_STAT_REG); 
    // Read STATUS_REG (0x05)
    reg_value = Get_MAX14526_ADDR(STATUS_REG);

    if (reg_value & CHPORT_M){
        // High Current Host/Hub Detected
        printk("MUIC : %s:%d CHPORT \n",__func__,__LINE__);
        // Enable 200K, Charger Pump, and ADC (0x01=0x13)
        Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
        // COMP2 to DP2 /COMN1 to DN1 (0x03=0x00)
        Set_MAX14526_ADDR(SW_CTRL_REG, COMN1_TO_DN1 | COMP2_TO_DP2);

        /* Enable charger IC in TA mode */
        //20100526, , Enable charger IC in TA mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
        lprintk(D_MUIC, "%s: CHG_IC_TA_MODE(%d) \n", __func__, CHG_IC_TA_MODE);	
        charging_ic_active(CHG_IC_TA_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
        //20100526, , Enable charger IC in TA mode [END]
        wake_lock(&s_hMuicHandle.wlock);
        current_device = DEVICE_USB_CABLE;
    }
    else{
        // Dedicated Charger(TA) Detected 
        printk("MUIC : %s:%d ELSE \n",__func__,__LINE__);
        // COMP2 to H-Z /COMN1 to H-Z (0x03=0x24)       
        Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_HZ | COMN1_TO_HZ);
     
        /* Enable charger IC in TA mode */
        //20100526, , Enable charger IC in TA mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
        lprintk(D_MUIC, "%s: CHG_IC_TA_MODE(%d) \n", __func__, CHG_IC_TA_MODE);	
        charging_ic_active(CHG_IC_TA_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
        //20100526, , Enable charger IC in TA mode [END]
        current_device = DEVICE_TA_CHARGER;
    }
    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
    // Enable Interrupts (0x02 = 0x40)
    Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M);
}

void Set_MAX14526_Factory_Mode_Detect(void)
{
    printk("-->> MUIC : %s\n",__func__);

#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_UART);
#endif
    DP3T_Switch_ctrl(DP3T_S2_CP_SW);

    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);  
    // COMP2 to U2 /COMN1 to U1 (0x03=0x09)
    Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);
   
    /* Turn on charger IC with FACTORY mode */
    //20100526, , Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_FACTORY_MODE(%d) \n", __func__, CHG_IC_FACTORY_MODE);	
    charging_ic_active(CHG_IC_FACTORY_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Turn on charger IC with FACTORY mode [END]
    wake_lock(&s_hMuicHandle.wlock);
    current_device = DEVICE_UART_CABLE;
}

void Set_MAX14526_Develop_Mode_Detect(void)
{
    printk("-->> MUIC : %s\n",__func__);

#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_IPC);
#endif
    DP3T_Switch_ctrl(DP3T_S1_AP);

    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
    // COMP2 to U2 /COMN1 to U1 (0x03=0x09)
    Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);
   

    /* Turn on charger IC with FACTORY mode */
    //20100526, , Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_FACTORY_MODE(%d) \n", __func__, CHG_IC_FACTORY_MODE);	
    charging_ic_active(CHG_IC_FACTORY_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Turn on charger IC with FACTORY mode [END]
    wake_lock(&s_hMuicHandle.wlock);
    current_device = DEVICE_FACTORY_USB_CABLE;
}

void Set_MAX14526_Usb_Mode_Detect(void)
{
    printk("-->> MUIC : %s \n",__func__);

#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_IPC); 
#endif
    DP3T_Switch_ctrl(DP3T_NC);

    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
    // COMP2 to DP2 /COMN1 to DN1 (0x03=0x00)
    Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_DP2 | COMN1_TO_DN1);
    
    /* Turn on charger IC with Standard USB mode  */
    //20100526, , Turn on charger IC with Standard USB mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_DEFAULT_MODE(%d) \n", __func__, CHG_IC_DEFAULT_MODE);	
    charging_ic_active(CHG_IC_DEFAULT_MODE); 
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Turn on charger IC with Standard USB mode [END]
    wake_lock(&s_hMuicHandle.wlock);
    current_device = DEVICE_USB_CABLE;
}

void Set_MAX14526_CP_USB_Mode(void){

    printk("-->> MUIC : %s \n",__func__);

#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_IPC);
#endif
    DP3T_Switch_ctrl(DP3T_S3_CP_USB);

    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);   
    // COMP2 to U2 /COMN1 to U1 (0x03=0x09)
    Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);

    /* Turn on charger IC with FACTORY mode */
    //20100526, , Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_FACTORY_MODE(%d) \n", __func__, CHG_IC_FACTORY_MODE);	
    charging_ic_active(CHG_IC_FACTORY_MODE); 
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Turn on charger IC with FACTORY mode [END]
    wake_lock(&s_hMuicHandle.wlock);
    current_device = DEVICE_CP_USB_CABLE;
}

void Set_MAX14526_CP_USB_Mode_PortSetting(void){

    printk("-->> MUIC : %s \n",__func__);

#if !defined(CONFIG_MACH_STAR_TMUS)
    USIF_ctrl(USIF_IPC);
#endif
    DP3T_Switch_ctrl(DP3T_S3_CP_USB);

    // Enable 200K, Charger Pump, and ADC (0x01=0x13)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M | CP_EN_M);
    // COMP2 to U2 /COMN1 to U1 (0x03=0x09)
    Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_U2 | COMN1_TO_U1);

    /* Turn on charger IC with FACTORY mode */
    //20100526, , Turn on charger IC with FACTORY mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    lprintk(D_MUIC, "%s: CHG_IC_DEFAULT_MODE(%d) \n", __func__, CHG_IC_DEFAULT_MODE);
    charging_ic_active(CHG_IC_DEFAULT_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Turn on charger IC with FACTORY mode [END]
    wake_lock(&s_hMuicHandle.wlock);
    current_device = DEVICE_CP_USB_CABLE;
}


#ifdef NO_EXE
void Set_MAX14526_Other_Mode_Detect_Dummy()
{
    NvU8 reg_value;
    
    // Disable Interrupts (0x02 = 0x10)
    Set_MAX14526_ADDR(CTRL2_REG, CP_AUD_M);
    // Connect 2.2K external resistor (0x01 = 0x42)
    Set_MAX14526_ADDR(CTRL1_REG, ID_2P2_M | ADC_EN_M);

    // Read INT_STAT_REG register (0x04)
    reg_value = Get_MAX14526_ADDR(INT_STAT_REG);

    // IDNO>0000(GND)?
    if ((reg_value & IDNO_M) > IDNO_0000) {
        // Headset Detected

        // Connect 2.2K, set LDO to 2.3V,
        // enable SEMR comparators,
        // enable Switch Charger pump
        // (0x01 = 0x4D)
        Set_MAX14526_ADDR(CTRL1_REG, ID_2P2_M | VLDO_M | SEMREN_M | CP_EN_M);

        // Enable audio amp
        // Enable interrupts, clear AUD click/pop resistors (0x02 = 0x40)
        Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M);

        // connect MIC/ID Switch and COM to AUD (0x03 = 0x52)
        Set_MAX14526_ADDR(SW_CTRL_REG, MIC_ON_M | COMP2_TO_AUD2 | COMN1_TO_AUD1);

        current_device = DEVICE_HEADSET;
    }
    else {
        // usb otg / video cable
        // Connect 620 ohm external resistor (0x01=0x22)
        Set_MAX14526_ADDR(CTRL1_REG, ID_620_M | ADC_EN_M);

        // Read INT_STAT register
        reg_value = Get_MAX14526_ADDR(INT_STAT_REG);

        if ((reg_value & IDNO_M) == IDNO_0001) {
            // 75 ohme detected - Enable Video Output
            // Enable Charger Pump(0x01 = 0x01)
            Set_MAX14526_ADDR(CTRL1_REG, CP_EN_M);
            // Enable Interrupts, clear AUD click/pop resistors (0x02 = 0x40)
            Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M);
            // connect VID/ID Switch and COM to AUD (0x03 = 0x92)
            Set_MAX14526_ADDR(SW_CTRL_REG, VID_ON_M | COMP2_TO_AUD2 | COMN1_TO_AUD1);
            current_device = DEVICE_VIDEO_CABLE_WT_LOAD;
        }
        else {
            // USB OTG detected - Invalid accessory
            // Connect 200K (0x01 = 0x12)
            Set_MAX14526_ADDR(CTRL1_REG, ID_200_M | ADC_EN_M);
            // Re-enable interrupt(0x02 = 0x50)
            Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M | CP_AUD_M);
            current_device = DEVICE_USB_OTG;
        }
    }
}
#endif
void muic_initialize_max(TYPE_RESET reset)
{    
#if defined(CONFIG_MACH_STAR)
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, DISABLE);
#endif
    // Enable 200K, ADC (0x01=0x12)
    Set_MAX14526_ADDR(CTRL1_REG, ID_200_M| ADC_EN_M);
    // Enable Interrupts (0x02 = 0x40)
    Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M);
    
    //20100526, , Charging IC Reset [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
    if ( !reset )
    {
        lprintk(D_MUIC, "%s: charging_ic_deactive call \n", __func__ );
        charging_ic_deactive();
    }
#endif /* CONFIG_STAR_BATTERY_CHARGER */
    //20100526, , Charging IC Reset [END]
    wake_unlock(&s_hMuicHandle.wlock);
}

void Set_MAX14526_Device_None_Detect(NvU8 int_status_reg)
{
    NvU8 reg_value;

    // IDNO=0100? 130Kohm :: CP UART MODE
    if((int_status_reg & IDNO_M) == IDNO_0100) 
        Set_MAX14526_Factory_Mode_Detect(); 


    // IDNO=0010? 56Kohm  :: CP USB MODE
    else if ((int_status_reg &IDNO_M )==IDNO_0010)  
        Set_MAX14526_CP_USB_Mode();


    // IDNO=0010? 910Kohm  :: CP USB MODE
    else if ((int_status_reg &IDNO_M )==IDNO_1010)  
        Set_MAX14526_CP_USB_Mode();


    // CHGDET=1?  :: HIGH CURRENT USB or TA?
    else if (int_status_reg & CHGDET_M)
    {
        Set_MAX14526_Charger_Detect(int_status_reg);
    }

    // VBUS=1?  :: TA or AP USB?
    else if (int_status_reg & VBUS_M){
        // COMP2 to H-Z / COMN1 to C1COMP (0x03=0x23)
        Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_HZ | COMN1_TO_C1COMP);

        NvOdmOsSleepMS(200) ;          

        // Read STATUS_REG (0x05)
        reg_value = Get_MAX14526_ADDR(STATUS_REG);

        if (reg_value & C1COMP_M){ 
            // Dedicated Charger(TA) Detected
            // COMP2 to H-Z / COMN1 to H-Z (0x03=0x24)
            Set_MAX14526_ADDR(SW_CTRL_REG, COMP2_TO_HZ | COMN1_TO_HZ);
            
            //20100526, , Enable charger IC in TA mode [START]
#if defined(CONFIG_STAR_BATTERY_CHARGER)
            lprintk(D_MUIC, "%s: CHG_IC_TA_MODE(%d) \n", __func__, CHG_IC_TA_MODE);	
            charging_ic_active(CHG_IC_TA_MODE);
#endif /* CONFIG_STAR_BATTERY_CHARGER */
            //20100526, , Enable charger IC in TA mode [END]

            current_device = DEVICE_TA_CHARGER;
            printk("-->> MUIC : %s : DEVICE_TA_CHARGER  \n",__func__);
        }
        else 
        {
            // USB Detected
            Set_MAX14526_Usb_Mode_Detect();
//20101117, , for autorun
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
            switch_set_state(&s_hMuicHandle.sdev_autorun, 1);
#endif
        }
    }
    else
    {
        // Accessory Not Supported
        current_device = DEVICE_NONE;
        muic_initialize_max(DEFAULT);

    }
}

void MAX14526_Device_Detection(void)
{
    NvU8 reg_value;

    //20101002, , keep CP USB state after rebooting [START]
    if(DEVICE_CP_USB_CABLE == boot_muic_state)
    {
        lprintk(D_MUIC, "CP Retain mode. \n");

        // Read INT_STAT_REG (0x04)
        reg_value = Get_MAX14526_ADDR(INT_STAT_REG);        
        lprintk(D_MUIC, "***********************************************\n");
        lprintk(D_MUIC, "%s: 1231 MUIC int_state 0x%2x : current_device : %d \n", __func__, reg_value, current_device);
        lprintk(D_MUIC, "***********************************************\n");

        // CP USB Mode
        if (reg_value & VBUS_M){           
        Set_MAX14526_CP_USB_Mode();
        lprintk(D_MUIC, "CP Retain mode CP USB Setting. \n");
        }
        else { 
        // Exit CP USB Mode
        max14526_muic_init(DEFAULT); 
        lprintk(D_MUIC, "CP Retain mode MUIC Init. \n");
        }
        return;
    }
    //20101002, , keep CP USB state after rebooting [END]
    
    // Read INT_STAT_REG (0x04)
    reg_value = Get_MAX14526_ADDR(INT_STAT_REG); 	
    lprintk(D_MUIC, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    lprintk(D_MUIC, "%s: 1231 MUIC int_state 0x%2x : current_device : %d \n", __func__, reg_value, current_device);
    lprintk(D_MUIC, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    switch (current_device)
    {
        case DEVICE_NONE:
        case DEVICE_USB_OTG:
            Set_MAX14526_Device_None_Detect(reg_value);
            break;

        // CP UART Mode
        case DEVICE_UART_CABLE:
            if ((reg_value & VBUS_M) == 0) {
                // Exit CP UART Mode
                max14526_muic_init(DEFAULT);
            }
            else {
                Set_MAX14526_Device_None_Detect(reg_value);
            }
            break;

        // TA Mode
        case DEVICE_TA_CHARGER:
            if ((reg_value & VBUS_M) == 0){
                // Exit Charger Mode
                max14526_muic_init(DEFAULT);
            }
            else {
                Set_MAX14526_Device_None_Detect(reg_value);
            }
            break;

        // AP UART Mode
        case DEVICE_FACTORY_USB_CABLE:
            if ((reg_value & VBUS_M) == 0) {
                // Exit AP UART Mode
                max14526_muic_init(DEFAULT);
            }
            else{
                Set_MAX14526_Device_None_Detect(reg_value);
            }
            break;

        // AP USB Mode
        case DEVICE_USB_CABLE:
            if ((reg_value & VBUS_M) == 0){
                // Exit AP USB Mode
                max14526_muic_init(DEFAULT);
//20101117, , for autorun
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
                switch_set_state(&s_hMuicHandle.sdev_autorun, 0);
#endif
            }
            else{
                Set_MAX14526_Device_None_Detect(reg_value);
            }
            break;

           
        // CP USB Mode							
        case DEVICE_CP_USB_CABLE:                           							
            if ((reg_value & VBUS_M) == 0){							
                // Exit CP USB Mode						
                max14526_muic_init(DEFAULT);							
            }							
            else{							
                Set_MAX14526_Device_None_Detect(reg_value);		                					
            }							
            break;
          

       // TA Mode
       case DEVICE_LG_PROP_TA :
            if ((reg_value & CHGDET_M) == 0) {
                // Exit Charger Mode
                max14526_muic_init(DEFAULT);
            }
            else {
                Set_MAX14526_Device_None_Detect(reg_value);
            }
            break;

        default:
            current_device = DEVICE_NONE;
            break;
    }
}


static void muic_wq_func(struct work_struct *muic_wq)
{
    if (star_shutdown) return;
    MAX14526_Device_Detection();
}


static void muic_interrupt_handler(void* arg)
{
    lprintk(D_MUIC, "%s: MUIC interrupt occured!\n", __func__);
    wake_lock_timeout(&s_hMuicHandle.wlock, HZ/2/*500ms*/);
    lprintk(D_MUIC, "%s: Wake_Lock_Timeout applied!  500msec Start !\n", __func__);  
    schedule_delayed_work(&muic_wq, msecs_to_jiffies(300));
    NvOdmGpioInterruptDone(s_hMuicHandle.hGpioInterrupt);
}

static int __init muic_probe(struct platform_device *pdev)
{
    int i, j, ret;
#ifdef _MUIC_GPIO_I2C_
    int ret_val;
#else	
    NvU32 I2cInstance = 0;
#endif
#if defined(CONFIG_MACH_STAR_REV_D) || defined(CONFIG_MACH_STAR_REV_E) || defined(CONFIG_MACH_STAR_REV_F)
    NvU32 pin[5], port[5];
#else
    NvU32 pin[4], port[4];
#endif
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    printk(KERN_INFO "muic_probe\n");

    pConnectivity = NvOdmPeripheralGetGuid(MUIC_GUID);

    for (i = 0, j = 0 ; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
#ifndef _MUIC_GPIO_I2C_
            case NvOdmIoModule_I2c:
                s_hMuicHandle.DeviceAddr = (pConnectivity->AddressList[i].Address << 1);
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

    s_hMuicHandle.hGpio = NvOdmGpioOpen();
    if (!s_hMuicHandle.hGpio)
    {
        lprintk(D_MUIC, "%s: NvOdmGpioOpen Error \n", __func__);
        goto err_open_gpio_fail;
    }

    s_hMuicHandle.h_INT_N_MUIC = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[0], pin[0]);
    if (!s_hMuicHandle.h_INT_N_MUIC)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle.h_AP20_UART_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[1], pin[1]);
    if (!s_hMuicHandle.h_AP20_UART_SW)
    {
        lprintk(D_MUIC, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle.h_IFX_UART_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[2], pin[2]);
    if (!s_hMuicHandle.h_IFX_UART_SW)
    {
        lprintk(D_MUIC, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

#if defined(CONFIG_MACH_STAR_TMUS)
    s_hMuicHandle.h_USB_VBUS_EN = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[3], pin[3]);
    if (!s_hMuicHandle.h_USB_VBUS_EN)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }
#else
    s_hMuicHandle.h_USIF1_SW = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[3], pin[3]);
    if (!s_hMuicHandle.h_USIF1_SW)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    s_hMuicHandle.h_USB_VBUS_EN = NvOdmGpioAcquirePinHandle(s_hMuicHandle.hGpio, port[4], pin[4]);
    if (!s_hMuicHandle.h_USB_VBUS_EN)
    {
        lprintk(D_MUIC, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }
#endif

    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC, NvOdmGpioPinMode_InputData);
    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW, NvOdmGpioPinMode_Output);
    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW, NvOdmGpioPinMode_Output);
#if defined(CONFIG_MACH_STAR)
#ifdef CONFIG_MACH_STAR_REV_F
    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, NvOdmGpioPinMode_Output);
#endif
    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN, NvOdmGpioPinMode_Output);
#else
    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW, NvOdmGpioPinMode_Output);
#endif


#ifdef _MUIC_GPIO_I2C_
    ret_val = MUIC_Gpio_i2c_init(&s_hMuicHandle);
    if (ret_val < 0)
    {	
        lprintk(D_MUIC, "%s: MUIC_Gpio_i2c_init Error \n", __func__);
        goto err_open_i2c_handle;
    }
#else
    s_hMuicHandle.hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
    if (!s_hMuicHandle.hOdmI2c)
    {
        lprintk(D_MUIC, "%s: NvOdmI2cOpen Error \n", __func__);
        goto err_open_i2c_handle;
    }
#endif
    wake_lock_init(&s_hMuicHandle.wlock, WAKE_LOCK_SUSPEND, "muic_active");

//20101117, , for autorun [START]
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
    s_hMuicHandle.sdev_autorun.name = DRIVER_NAME_FOR_AUTORUN;
    s_hMuicHandle.sdev_autorun.print_name = print_switch_name_for_autorun;
    s_hMuicHandle.sdev_autorun.print_state = print_switch_state_for_autorun;

    ret = switch_dev_register(&s_hMuicHandle.sdev_autorun);
    if (ret) {
        goto err_sdev_unregister;
    }
#endif
//20101117, , for autorun [END]

    INIT_DELAYED_WORK(&muic_wq, muic_wq_func);

    if (NvOdmGpioInterruptRegister(s_hMuicHandle.hGpio, &s_hMuicHandle.hGpioInterrupt,
                s_hMuicHandle.h_INT_N_MUIC, NvOdmGpioPinMode_InputInterruptFallingEdge , muic_interrupt_handler,
                (void*)&s_hMuicHandle, 0) == NV_FALSE) 

    {
        lprintk(D_MUIC, KERN_ERR "%s: cannot register interrupt.\n", __func__);
        goto err_get_interrupt_handler;
    }

    create_star_muic_proc_file();
    current_device = DEVICE_NONE;

    muic_initialize_max(RESET);

    //20100915, , move to late_initcall [START]     
#if 0		
    NvOdmOsSleepMS(400) ;
    MAX14526_Device_Detection();
#endif		
    //20100915, , move to late_initcall [STOP]  

    return 0;

err_get_interrupt_handler:
#ifndef _MUIC_GPIO_I2C_	
    NvOdmI2cClose(s_hMuicHandle.hOdmI2c);
#endif
//20101117, , for autorun [START]
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
err_sdev_unregister:
	switch_dev_unregister(&s_hMuicHandle.sdev_autorun);
#endif
//20101117, , for autorun [END]
err_open_i2c_handle:
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW);
#ifdef CONFIG_MACH_STAR_REV_F
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif
#if defined(CONFIG_MACH_STAR)
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN);
#else
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif

err_open_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_hMuicHandle.hGpio);
err_open_gpio_fail:

    return -ENOSYS;
}

static int muic_remove(struct platform_device *pdev)
{
    NvOdmGpioInterruptUnregister(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC,
            s_hMuicHandle.hGpioInterrupt);

    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW);
#if defined(CONFIG_MACH_STAR)
#ifdef CONFIG_MACH_STAR_REV_F
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN);
#else
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif

    NvOdmGpioClose(s_hMuicHandle.hGpio);
#ifndef _MUIC_GPIO_I2C_
    NvOdmI2cClose(s_hMuicHandle.hOdmI2c);
#endif
    wake_lock_destroy(&s_hMuicHandle.wlock);
    remove_star_muic_proc_file();
    return 0;
}

static void muic_shutdown(struct platform_device *pdev)
{
    printk("muic_shutdown\n");
    star_shutdown = 1;
#if 0	
	if (&muic_wq)
	{
		cancel_delayed_work_sync(&muic_wq);
		printk("muic_wq canceled\n");
	}  
#endif
    NvOdmGpioInterruptUnregister(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC,
            s_hMuicHandle.hGpioInterrupt);

    NvOdmGpioConfig(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC, NvOdmGpioPinMode_Output);
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_INT_N_MUIC);
    
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_AP20_UART_SW);
   
 
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_IFX_UART_SW);
#if defined(CONFIG_MACH_STAR)
#ifdef CONFIG_MACH_STAR_REV_F
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USB_VBUS_EN);
#else
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio, s_hMuicHandle.h_USIF1_SW);
#endif

    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.hSdaGpioPinHandle , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio,  s_hMuicHandle.hSdaGpioPinHandle);
 
    NvOdmGpioSetState(s_hMuicHandle.hGpio, s_hMuicHandle.hSclGpioPinHandle , 0x0);
    NvOdmGpioReleasePinHandle(s_hMuicHandle.hGpio,  s_hMuicHandle.hSclGpioPinHandle);
 
 
    NvOdmGpioClose(s_hMuicHandle.hGpio);
#ifndef _MUIC_GPIO_I2C_
   NvOdmI2cClose(s_hMuicHandle.hOdmI2c);
#endif
}



static int muic_suspend(struct platform_device *pdev, pm_message_t state)
{
    lprintk(D_MUIC, "%s \n", __func__);
    // Disable Interrupts (0x02 = 0x00)
    Set_MAX14526_ADDR(CTRL2_REG, INT_DIS_M);
    lprintk(D_MUIC, "%s Interrupt Disable\n", __func__);
    return 0;
}

static int muic_resume(struct platform_device *pdev)
{
    lprintk(D_MUIC, "%s \n", __func__);
    // Enable Interrupts (0x02 = 0x40)
    Set_MAX14526_ADDR(CTRL2_REG, INT_EN_M);
    lprintk(D_MUIC, "%s Interrupt Enable\n", __func__);
    return 0;
}

static struct platform_driver muic_driver = {
    .probe     = muic_probe,
    .remove    = muic_remove,
    .suspend   = muic_suspend,
    .resume    = muic_resume,
    .shutdown = muic_shutdown,
    .driver    = {
        .name = "star_muic",
    },
};

static int __devinit muic_init(void)
{
    printk(KERN_INFO "muic_init\n");
    return platform_driver_register(&muic_driver);
}


static int __devinit muic_late_init(void)
{
    printk(KERN_INFO "muic_late_init\n");
    MAX14526_Device_Detection();
    return 0;
}



static void __exit muic_exit(void)
{
    platform_driver_unregister(&muic_driver);
}



//20100911, , MUIC kernel command line parsing [START]
static int __init muic_state(char *str)
{
    int muic_value = simple_strtol(str, NULL, 0);
    //20101002, , keep CP USB state after rebooting
    boot_muic_state = muic_value;
    printk(KERN_INFO "muic_state = %d\n",muic_value);
    return 1;
}
__setup("muic_state=", muic_state);
//20100911, , MUIC kernel command line parsing [END]


module_init(muic_init);
module_exit(muic_exit);
late_initcall(muic_late_init);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("star MUIC Driver");
MODULE_LICENSE("GPL");

