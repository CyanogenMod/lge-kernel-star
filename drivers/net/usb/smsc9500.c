
 /***************************************************************************
 *
 * Copyright (C) 2007-2008  SMSC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 ***************************************************************************
 * File: smsc9500.c
 ***************************************************************************
 * History:
 * Vlad Lyalikov, 10/20/2009
 * Added bulkin_delay parameter for testing pusposes
 * Vlad Lyalikov, 01/26/2010
 * Support for ndo framework
 *****************************************************************************/
#ifndef __KERNEL__
#	define __KERNEL__
#endif

#include <linux/version.h>

#define TX_SKB_FORCE_COPY

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13))
#include <linux/config.h>
#endif

#include <linux/module.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#include <linux/moduleparam.h>
#endif

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include "version.h"
#include "smscusbnet.h"
#include "smsc9500.h"
#include "ioctl_9500.h"

#define CHECK_RETURN_STATUS(A) { if((A) < 0){ goto DONE;} }

unsigned int debug_mode = DBG_WARNING | DBG_INIT | DBG_LINK_CHANGE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(debug_mode, uint, 0);
#else
	MODULE_PARM(debug_mode,"i");
#endif
MODULE_PARM_DESC(debug_mode,"bit 0 enables trace points, bit 1 enables warning points, bit 2 enables eth gpios, bit 3 enables gen gpios");

u32 link_mode=0x7fUL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(link_mode, uint, 0);
#else
	MODULE_PARM(link_mode,"i");
#endif
MODULE_PARM_DESC(link_mode,"Set Link speed and Duplex, 1=10HD,2=10FD,4=100HD,8=100FD,default=0xF");

u32 auto_mdix=0x3U;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
    module_param(auto_mdix, uint, 0);
#else
    MODULE_PARM(auto_mdix,"i");
#endif
MODULE_PARM_DESC(auto_mdix,"Set Auto-MDIX state, 0=StraightCable,1=CrossOver,2=Enable AMDIX,3=controlled by Strap");

u32 mac_addr_hi16=0xFFFFFFFFUL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(mac_addr_hi16, uint, 0);
#else
	MODULE_PARM(mac_addr_hi16,"i");
#endif
MODULE_PARM_DESC(mac_addr_hi16,"Specifies the high 16 bits of the mac address");

u32 mac_addr_lo32=0xFFFFFFFFUL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(mac_addr_lo32, uint, 0);
#else
	MODULE_PARM(mac_addr_lo32,"i");
#endif
MODULE_PARM_DESC(mac_addr_lo32,"Specifies the low 32 bits of the mac address");

u32 phy_addr=0xFFFFFFFFUL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(phy_addr, uint, 0);
#else
	MODULE_PARM(phy_addr,"i");
#endif
MODULE_PARM_DESC(phy_addr,"phy_addr, only valid if it is external phy set by strap; 0-31=external phy with specified address, else autodetect external phy addr");

int scatter_gather=FALSE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(scatter_gather,bool, 0);
#else
MODULE_PARM(scatter_gather,"bool");
#endif
MODULE_PARM_DESC(scatter_gather,"Enable Scatter Gather");

int tx_Csum=FALSE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(tx_Csum,bool, 0);
#else
	MODULE_PARM(tx_Csum,"bool");
#endif
MODULE_PARM_DESC(tx_Csum,"Enable Tx Hardware Checksum Offload");

int rx_Csum=FALSE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(rx_Csum,bool, 0);
#else
	MODULE_PARM(rx_Csum,"bool");
#endif
MODULE_PARM_DESC(tx_Csum,"Enable Rx Hardware Checksum Offload");

u32 bulkin_delay=DEFAULT_BULK_IN_DELAY;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(bulkin_delay,uint, 0);
#else
	MODULE_PARM(bulkin_delay,"i");
#endif
MODULE_PARM_DESC(bulkin_delay,"16 bit value in units of 16ns to delay UTX sending data");

int TurboMode=TRUE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(TurboMode,bool, 0);
#else
	MODULE_PARM(TurboMode,"bool");
#endif
MODULE_PARM_DESC(TurboMode,"Enable Turbo Mode");

int LinkActLedCfg=FALSE;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(LinkActLedCfg,bool, 0);
#else
	MODULE_PARM(LinkActLedCfg,"bool");
#endif
MODULE_PARM_DESC(LinkActLedCfg,"Enables separate Link and Activity LEDs in LAN9500A");

u32 LinkLedOnGpio=11;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(LinkLedOnGpio, uint, 0);
#else
	MODULE_PARM(LinkLedOnGpio,"i");
#endif
MODULE_PARM_DESC(LinkLedOnGpio,"Enable separate Link and Activity LEDs in LAN9500 and specifies gpio port for link status");

u32 LinkLedBufType=0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(LinkLedBufType, bool, 0);
#else
	MODULE_PARM(LinkLedBufType,"bool");
#endif
MODULE_PARM_DESC(LinkLedBufType,"Specifies gpio buffer type for link led");

u32 LinkLedPolarity = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(LinkLedPolarity, bool, 0);
#else
	MODULE_PARM(LinkLedPolarity,"bool");
#endif
MODULE_PARM_DESC(LinkLedPolarity,"Specifies active level on gpio port");

/*
linkdownsuspend = 0----> Disabled
linkdownsuspend = 1----> Enabled, wake up on auto-negotiation complete, device is in suspend0.
linkdownsuspend = 2----> Enabled, wake up on energy detection, device is in suspend1.
*/
static int linkdownsuspend=2;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	module_param(linkdownsuspend, uint, 0);
#else
	MODULE_PARM(linkdownsuspend,"i");
#endif
MODULE_PARM_DESC(linkdownsuspend,"Suspend device when link is down");

static int dynamicsuspend=0;
module_param(dynamicsuspend,bool, 0);
MODULE_PARM_DESC(dynamicsuspend,"Enable dynamic autosuspend mode");

static int smartdetach=0;
module_param(smartdetach,bool, 0);
MODULE_PARM_DESC(smartdetach,"Enable smart detach mode, for LAN9500A only");

/********static function and variable declartion****************/
static int smsc9500_reset(struct usbnet *dev);
static int smsc9500_get_stats(struct usbnet *dev, void *data);
static int smsc9500_private_ioctl(PADAPTER_DATA  privateData, struct usbnet *dev, PSMSC9500_IOCTL_DATA ioctlData);
static int smsc9500_device_recovery(struct usbnet *dev);
static int SetGpo(struct usbnet * dev,  u32 Gpo, u32 State);
static int Smsc9500SystemSuspend (struct usb_interface *intf, pm_message_t state);
static int Smsc9500SystemResume(struct usb_interface *intf);
static u16 CalculateCrc16(const BYTE * bpData,const u32 dwLen, const BOOLEAN fBitReverse);
static int SetLinkDownWakeupEvents(struct usbnet *dev, int wakeUpMode);
static int ResetLinkDownWakeupEvents(struct usbnet *dev);
static int Smsc9500AutoSuspend (struct usb_interface *intf, pm_message_t state);
static int Smsc9500AutoResume(struct usb_interface *intf);
static int EnablePHYWakeupInterrupt(struct usbnet *dev, u32 interrupt);
static int DisablePHYWakeupInterrupt(struct usbnet *dev, u32 interrupt);

static u32 LanRegMap[MAX_LAN_REG_NUM];
static u32 MacRegMap[MAX_MAC_REG_NUM];
static u32 PhyRegMap[MAX_PHY_REG_NUM];
/***************************************************************/

enum{
    SMSC9500_FAIL = -1,
    SMSC9500_SUCCESS = 0
};

/***************************************************************/
static int smsc9500_read_reg(struct usbnet *dev,   u32 index, u32 *data)
{
	int ret = 0;
    u32 *buf = NULL;
    u16 retry_count = 0;

    BUG_ON(!dev);

//The heap buffer should be used for usb_control_msg, because the stack might not be DMA-mappable
//Control message is very slow so it really isn't big deal to dynamically allocate the data
	buf = kmalloc (sizeof(u32), GFP_KERNEL);
    if(buf == NULL){
        return SMSC9500_FAIL;
    }

    do{
	ret=usb_control_msg(
		dev->udev,
		usb_rcvctrlpipe(dev->udev, 0),
		USB_VENDOR_REQUEST_READ_REGISTER,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		00,
		index,
		(void*)buf,
		sizeof(u32),
		USB_CTRL_GET_TIMEOUT);
    }while((ret < 0) && (retry_count++ < 3));

	if (ret<0){
		SMSC_WARNING("Failed to read register index 0x%08x, set flag to recover", index);
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);
    }else{
        le32_to_cpus(buf);
	   *data = *buf;
    }

	kfree(buf);

	return ret;
}

static int smsc9500_write_reg(struct usbnet *dev,  u32 index, u32 data)
{
	int ret = 0;
    u32* buf = NULL;
    u16 retry_count = 0;

    BUG_ON(!dev);

//The heap buffer should be used for usb_control_msg, because the stack might not be DMA-mappable
//Control message is very slow so it really isn't big deal to dynamically allocate the data
    buf = kmalloc (sizeof(u32), GFP_KERNEL);
    if(buf == NULL){
        return SMSC9500_FAIL;
    }
    *buf = data;

	cpu_to_le32s(buf);

    do{
	ret=usb_control_msg(
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		USB_VENDOR_REQUEST_WRITE_REGISTER,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		00,
		index,
		buf,
		sizeof(u32),
		USB_CTRL_SET_TIMEOUT);
    }while((ret < 0) && (retry_count++ < 3));

	if (ret<0){
		SMSC_WARNING("Failed to write register index 0x%08x, set flag to recover", index);
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);
    }

    kfree(buf);

	return ret;
}

static int smsc9500_set_feature(struct usbnet *dev,  u32 feature)
{
    BUG_ON(!dev);

	cpu_to_le32s((u32*)&feature);

	return usb_control_msg(
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		USB_REQ_SET_FEATURE,
		USB_RECIP_DEVICE,
		feature,
		0,
		NULL,
		0,
		USB_CTRL_SET_TIMEOUT);

}

static int smsc9500_clear_feature(struct usbnet *dev,  u32 feature)
{
    BUG_ON(!dev);

	cpu_to_le32s((u32*)&feature);

	return usb_control_msg(
			dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			USB_REQ_CLEAR_FEATURE,
			USB_RECIP_DEVICE,
			feature,
			0,
			NULL,
			0,
			USB_CTRL_SET_TIMEOUT);
}


static int smsc9500_read_phy(struct usbnet *dev,  u32 Register, u32 *pValue32 )

{
	int ret = SMSC9500_FAIL;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue,dwAddr;
	int Count;

    BUG_ON(!dev);

     if(down_interruptible(&adapterData->phy_mutex)){
        return -EINTR;
     }
    // confirm MII not busy
    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MII_ADDR, &dwValue));

    if ((dwValue & MII_BUSY_) != 0UL)
    {
		SMSC_WARNING("MII is busy in smsc9500_read_phy\n");
		goto DONE;
    }

    // set the address, index & direction (read from PHY)
    dwAddr = ((adapterData->dwPhyAddress & 0x1FUL)<<11) | ((Register & 0x1FUL)<<6)|MII_READ_;
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MII_ADDR, dwAddr));

	// Loop until the read is completed w/timeout
	for(Count=1;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MII_ADDR, &dwValue));

		if(!(dwValue & MII_BUSY_))
			break;
		udelay(1);

	}

	if (Count < 100)
	{
		ret = smsc9500_read_reg(dev, MII_DATA, pValue32);
	}
	else
	{
		SMSC_WARNING ("Timed out reading MII register %08X\n",Register & 0x1f);

	}
DONE:
    up(&adapterData->phy_mutex);
	return ret;

} /* smsc9500_read_phy */



static int smsc9500_write_phy(struct usbnet *dev,  u32 Register, u32 pValue32)
{

	int ret = SMSC9500_FAIL;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue,dwAddr;
	int Count;

    BUG_ON(!dev);

    if(down_interruptible(&adapterData->phy_mutex)){
        return -EINTR;
     }

	if(Register==0) {
		if(((LOWORD(pValue32))&0x1200)==0x1200) {
			adapterData->wLastADVatRestart=adapterData->wLastADV;
		}
	}
	if(Register==4) {
		adapterData->wLastADV=LOWORD(pValue32);
	}

    // confirm MII not busy
    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MII_ADDR, &dwValue));

    if ((dwValue & MII_BUSY_) != 0UL)
    {
		SMSC_WARNING ("MII is busy in smsc9500_read_phy\n");
		goto DONE;
    }

	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MII_DATA, pValue32));

    // set the address, index & direction (read from PHY)
    dwAddr = ((adapterData->dwPhyAddress & 0x1FUL)<<11) | ((Register & 0x1FUL)<<6)|MII_WRITE_;
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MII_ADDR, dwAddr));

	// Loop until the read is completed w/timeout
	for(Count=1;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MII_ADDR, &dwValue));
		if(!(dwValue & MII_BUSY_))
			break;
		udelay(1);

	}

	if (Count < 100)
	{
		ret=0;

	}
	else
	{
		SMSC_WARNING("Timed out writing MII register %08X\n",Register & 0x1f);

	}
DONE:
    up(&adapterData->phy_mutex);
	return ret;

} /* smsc9500_write_phy */

static int smsc9500_eeprom_IsBusy(struct usbnet *dev)
{
	int retVal = 0;
	u32 dwValue;
	int Count;

    BUG_ON(!dev);

	for(Count=0;Count<1000;Count++) //40ms
	{
		if(smsc9500_read_reg(dev, E2P_CMD, &dwValue) < 0){
            return SMSC9500_FAIL;
        }
	    if (!(dwValue & E2P_CMD_BUSY_) || (dwValue & E2P_CMD_TIMEOUT_))
	    {
	    	break;
	    }
		udelay(40);
	}
	if ((dwValue & E2P_CMD_TIMEOUT_) || (dwValue & E2P_CMD_BUSY_)){
		SMSC_WARNING("EEPROM read operation timeout");
		retVal = SMSC9500_FAIL;
	}

    return retVal;
}

/* Read EEPROM data
 * pbValue: buffer pointer for data
 * */
static int smsc9500_read_eeprom(struct usbnet *dev,  u32 dwOffset, u32 dwLength,  BYTE* pbValue)
{

	int ret = SMSC9500_FAIL;
	u32 dwValue,dwAddr;
	int Count, i;
	PADAPTER_DATA adapterData;

    BUG_ON(!dev);
    BUG_ON(!pbValue);

    adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(dwOffset + dwLength > adapterData->eepromSize){
		SMSC_WARNING("EEPROM: out of eeprom space range, offset = %d, dwLength = %d", dwOffset, dwLength);
	}

    if(down_interruptible(&adapterData->eeprom_mutex)){
        return -EINTR;
     }

    // confirm eeprom not busy
	for(Count=0;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, E2P_CMD, &dwValue));
	    if (!(dwValue & E2P_CMD_BUSY_) || !(dwValue & E2P_CMD_LOADED_))
	    {
	    	break;
	    }
		udelay(40);
	}
    if (!(dwValue & E2P_CMD_LOADED_))
    {
		SMSC_WARNING("No EEPROM present");
        goto DONE;
    }
    if ((dwValue & E2P_CMD_BUSY_) != 0UL)
    {
		SMSC_WARNING("EEPROM is busy ");
        goto DONE;
    }
    dwAddr = dwOffset;
    for(i=0; i<dwLength; i++){
	//Isuue command
	dwValue = E2P_CMD_BUSY_ | E2P_CMD_READ_ | (dwAddr & E2P_CMD_ADDR_);
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, E2P_CMD, dwValue));
	CHECK_RETURN_STATUS(smsc9500_eeprom_IsBusy(dev));

	//Read data when ready
	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, E2P_DATA, &dwValue));

	pbValue[i] = dwValue & 0xFF;
	dwAddr++;
    }
	ret = 0;
DONE:
	up(&adapterData->eeprom_mutex);
	return ret;

} /* smsc9500_read_eeprom */

static int smsc9500_write_eeprom(struct usbnet *dev,  u32 dwOffset, u32 dwLength,  BYTE* pbValue)
{
	int ret = SMSC9500_FAIL;
	u32 dwValue,dwAddr;
	int Count, i;
	PADAPTER_DATA adapterData;

    BUG_ON(!dev);
    BUG_ON(!pbValue);

    adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(dwOffset + dwLength > adapterData->eepromSize){
		SMSC_WARNING("EEPROM: out of eeprom space range");
	}

    if(down_interruptible(&adapterData->eeprom_mutex)){
        return -EINTR;
     }

    // confirm eeprom not busy
	for(Count=0;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, E2P_CMD, &dwValue));

	    if (!(dwValue & E2P_CMD_BUSY_))
	    {
	    	break;
	    }
		udelay(40);
	}

    if ((dwValue & E2P_CMD_BUSY_) != 0UL)
    {
		SMSC_WARNING("EEPROM is busy ");
        goto DONE;
    }

    //Iuuse write/erase enable command
    dwValue = E2P_CMD_BUSY_ | E2P_CMD_EWEN_;
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, E2P_CMD, dwValue));
	CHECK_RETURN_STATUS(smsc9500_eeprom_IsBusy(dev));

    dwAddr = dwOffset;
    for(i=0; i<dwLength; i++){

	//Fill data register
	dwValue = pbValue[i];
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, E2P_DATA, dwValue));

	//Send "write"  command
	dwValue = E2P_CMD_BUSY_ | E2P_CMD_WRITE_ | (dwAddr & E2P_CMD_ADDR_);
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, E2P_CMD, dwValue));
	CHECK_RETURN_STATUS(smsc9500_eeprom_IsBusy(dev));

	dwAddr++;
    }

	ret = SMSC9500_SUCCESS;
DONE:

	up(&adapterData->eeprom_mutex);
	return ret;

} /* smsc9500_write_eeprom */

static int IsDataPortReady(struct usbnet *dev){
	int ret = FALSE;
	int count = 0;
	u32 dwValue;

// confirm data port is not busy
	for(count=0; count<100; count++)
	{
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, DP_SEL, &dwValue));
		if (dwValue & DP_SEL_DPRDY){
			ret = TRUE;
			break;
		}
		udelay(40);
	}

	if (ret == FALSE)
	{
		SMSC_WARNING("Data port is busy ");
	}
DONE:
	return ret;
}

/* Read data from internal RAM
 * ramSel:  Choose which internal RAM to access.
 * startAddr:  The first offset to access.
 * length:   Data length in DWORD.
 * valLow:	Low 32 bits buffer pointer.
 * valHigh: High 5 bits buffer pointer. If null, will ignore.
 * */
static int ReadDataPort(struct usbnet *dev, int ramSel, u32 startAddr, u32 length, u32 *valLow, u32 *valHigh)
{
	u32 dwValue;
	int ret = SMSC9500_FAIL;
	int i;
	PADAPTER_DATA adapterData;

	BUG_ON(!dev);
	adapterData = (PADAPTER_DATA)(dev->data[0]);
	BUG_ON(!adapterData);

    if(down_interruptible(&adapterData->internal_ram_mutex)){
        return -EINTR;
     }

  // confirm data port not busy
	if(!IsDataPortReady(dev))goto DONE;

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, DP_SEL, &dwValue));
	dwValue &= ~DP_SEL_RSEL;
	switch(ramSel){
		case RAMSEL_FCT: 	dwValue |= DP_SEL_RSEL_FCT; break;
		case RAMSEL_EEPROM: dwValue |= DP_SEL_RSEL_EEPROM; break;
		case RAMSEL_TXTLI: 	dwValue |= DP_SEL_RSEL_TXTLI; break;
		case RAMSEL_RXTLI: 	dwValue |= DP_SEL_RSEL_RXTLI; break;
	}
	dwValue |= DP_SEL_TESTEN;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, DP_SEL, dwValue));

	for(i=0; i<length; i++){
		//Set device ram address
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, DP_ADDR, startAddr + i));
		//Enable reading
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, DP_CMD, DP_CMD_READ));

		if(!IsDataPortReady(dev))goto DONE;

		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, DP_DATA0, valLow + i));
		if(valHigh){
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev, DP_DATA1, valHigh + i));
		}
	}

	ret = SMSC9500_SUCCESS;
DONE:

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, DP_SEL, &dwValue));
	dwValue &= ~DP_SEL_TESTEN;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, DP_SEL, dwValue));

    up(&adapterData->internal_ram_mutex);

	return ret;
}

static void smsc9500_status(struct usbnet *dev, struct urb *urb)
{
	struct smsc9500_int_data *event;
	int hasFrame;

    BUG_ON(!dev);
    BUG_ON(!urb);

	SMSC_TRACE(DBG_INTR,"---->in smsc9500_status\n");

	if (urb->actual_length < 4) {
		SMSC_WARNING("urb->actual_length= %d",urb->actual_length);
		return;
	}

	event = urb->transfer_buffer;

	le32_to_cpus((u32*)&event->IntEndPoint);

	SMSC_TRACE(DBG_INTR, "event->IntEndPoint= 0x%08x\n", event->IntEndPoint);
	hasFrame = event->IntEndPoint &INT_END_RXFIFO_HAS_FRAME_;

	if (hasFrame) {
		dev->StopSummitUrb=0;
		tasklet_schedule (&dev->bh);
	}

	SMSC_TRACE(DBG_INTR,"<----out of smsc9500_status\n");
}


static int smsc9500_get_stats(struct usbnet *dev, void *data)
{
	int ret = 0;
    void* buf;
    u16 retry_count = 0;

    BUG_ON(!dev);
    BUG_ON(!data);

	SMSC_TRACE(DBG_RX, "in smsc9500_get_stats\n");

    buf = kmalloc (sizeof(SMSC9500_RX_STATS), GFP_KERNEL);
    if(buf == NULL){
        return SMSC9500_FAIL;
    }

    do{
	ret=usb_control_msg(
		dev->udev,
		usb_rcvctrlpipe(dev->udev, 0),
		USB_VENDOR_REQUEST_GET_STATS,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		00,
		0,
		buf,
		sizeof(SMSC9500_RX_STATS),
		USB_CTRL_SET_TIMEOUT);
    }while((ret < 0) && (retry_count++ < 3));

    if (ret < 0){
        SMSC_WARNING("Failed to get status, set flag to recover");
        set_bit (EVENT_DEV_RECOVERY, &dev->flags);
    }else{
        memcpy(data,  buf, sizeof(SMSC9500_RX_STATS));
    }

    kfree(buf);

	return ret;
}

#if 0 /* commenting as it was causing panic on harmony platform */

static void smsc9500_async_cmd_callback(struct urb *urb, struct pt_regs *regs)
{
	struct USB_CONTEXT * usb_context = (struct USB_CONTEXT *)urb->context;

	if (urb->status < 0)
		SMSC_WARNING("smsc9500_async_cmd_callback() failed with %d\n",urb->status);

	complete((struct completion *)&usb_context->notify);

	kfree(&usb_context->req);
	usb_free_urb(urb);
}

static int
smsc9500_read_reg_async(struct usbnet *dev,   u32 index, void *data, int wait)
{
	int ret = ASYNC_RW_SUCCESS, expire;
	struct USB_CONTEXT * usb_context;
	int status;
	struct urb *urb;
	u32 size=4;

    BUG_ON(!dev);
    BUG_ON(!data);

	if ((urb = usb_alloc_urb(0, GFP_ATOMIC)) == NULL) {
		SMSC_WARNING("Error allocating URB in write_cmd_async!");
		return ASYNC_RW_FAIL;
	}

	if ((usb_context = kmalloc(sizeof(struct USB_CONTEXT), GFP_ATOMIC)) == NULL) {
		SMSC_WARNING( "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return ASYNC_RW_FAIL;
	}

	usb_context->req.bRequestType = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	usb_context->req.bRequest = USB_VENDOR_REQUEST_READ_REGISTER;
	usb_context->req.wValue = 00;
	usb_context->req.wIndex = cpu_to_le32(index);
	usb_context->req.wLength = cpu_to_le32(size);
	init_completion(&usb_context->notify);

	usb_fill_control_urb(urb, dev->udev,
			     usb_rcvctrlpipe(dev->udev, 0),
			     (void *)&usb_context->req, data, size,
			     (usb_complete_t)smsc9500_async_cmd_callback, (void*)usb_context);

	if((status = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		SMSC_WARNING( "Error submitting the control message: status=%d", status);
		kfree(usb_context);
		usb_free_urb(urb);
	}

	if(wait){
//wait_for_completion_timeout only implemented in 2.6.11 and higher kernel version
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
	    expire = msecs_to_jiffies(USB_CTRL_SET_TIMEOUT);
	    if (!wait_for_completion_timeout(&usb_context->notify, expire)) {

	  		ret = ASYNC_RW_TIMEOUT;
	  		SMSC_TRACE(DBG_WARNING,"urb timeout \n");
	  		kfree(usb_context);
	        usb_free_urb(urb);
	    }
#endif
	}

	return ret;

}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
static void CalculateTxChecksumOffset(
	struct sk_buff *skb,
	int *csum_start_offset
	)
{
	unsigned int skbFragCnt;
	int i;
	u32	offset;

	skbFragCnt = skb_shinfo(skb)->nr_frags + 1;

	// Initialize csum offset locations as if it was single frag.
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
		SMSC_ASSERT(skb->h.raw);
	#else
		SMSC_ASSERT(skb->transport_header);	// Should never happen for a CHECKSUM_HW packet.
	#endif

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
		*csum_start_offset = skb->h.raw - skb->data;
	#else
		*csum_start_offset = (unsigned long)skb->transport_header - (unsigned long)skb->data;
	#endif

	offset = (skbFragCnt == 1) ? skb->len : (skb->len - skb->data_len);

	// Process all fragments
	for(i=0;i<(skbFragCnt-1);i++)
	{
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		unsigned char *frag_addr = (unsigned char *) (page_address(frag->page) + frag->page_offset);

		// Find if transport header start belongs to this fragment and if so calculate offset from start of packet.
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
			if((frag_addr <= skb->h.raw) && ((frag_addr + frag->size) >=skb->h.raw))
			{
				*csum_start_offset = offset + ((u32)skb->h.raw) - ((u32)frag_addr);
			}
		#else
			if((frag_addr <= (unsigned char *)((unsigned long)skb->transport_header)) &&
				((frag_addr + frag->size) >= (unsigned char *)((unsigned long)skb->transport_header)))
			{
				*csum_start_offset = offset + ((unsigned long)skb->transport_header) - ((unsigned long)frag_addr);
			}
		#endif

		SMSC_ASSERT((offset + frag->size) <= skb->len);

		offset += frag->size;
	}

}
#endif

static void Tx_StopQueue(
	struct usbnet *dev,u32 dwSource)
{
	unsigned long intFlags=0;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	spin_lock_irqsave(&(adapterData->TxQueueLock),intFlags);
	if(adapterData->dwTxQueueDisableMask==0) {
		netif_stop_queue(dev->net);
	}
	adapterData->dwTxQueueDisableMask|=dwSource;
	spin_unlock_irqrestore(&(adapterData->TxQueueLock),intFlags);
}

static void Tx_WakeQueue(
	struct usbnet *dev,u32 dwSource)
{
	unsigned long intFlags=0;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	spin_lock_irqsave(&(adapterData->TxQueueLock),intFlags);
	adapterData->dwTxQueueDisableMask&=(~dwSource);
	if(adapterData->dwTxQueueDisableMask==0) {
		netif_wake_queue(dev->net);
	}
	spin_unlock_irqrestore(&(adapterData->TxQueueLock),intFlags);
}


//returns hash bit number for given MAC address
//example:
//   01 00 5E 00 00 01 -> returns bit number 31
static u32 Rx_Hash(BYTE addr[6])
{
	int i;
	u32 crc=0xFFFFFFFFUL;
	u32 poly=0xEDB88320UL;
	u32 result=0;
	for(i=0;i<6;i++)
	{
		int bit;
		u32 data=((u32)addr[i]);
		for(bit=0;bit<8;bit++)
		{
			u32 p = (crc^((u32)data))&1UL;
			crc >>= 1;
			if(p!=0) crc ^= poly;
			data >>=1;
		}
	}
	result=((crc&0x01UL)<<5)|
		((crc&0x02UL)<<3)|
		((crc&0x04UL)<<1)|
		((crc&0x08UL)>>1)|
		((crc&0x10UL)>>3)|
		((crc&0x20UL)>>5);
	return result;
}

static int smsc9500_rx_setmulticastlist(struct usbnet *dev)
{

	u32 local_MACCR, dwHashHi,dwHashLo;
	u32 ret = SMSC9500_FAIL;

	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	if (dev->suspendFlag) {
		return 0;
	}
	if(down_interruptible(&adapterData->RxFilterLock)){
       return -EINTR;
    }

	SMSC_TRACE(DBG_MCAST, "---------->in smsc9500_set_multicast\n");

	if(dev->net->flags & IFF_PROMISC) {
		SMSC_TRACE(DBG_MCAST,"Promiscuous Mode Enabled");
		adapterData->set_bits_mask = MAC_CR_PRMS_;
		adapterData->clear_bits_mask = (MAC_CR_MCPAS_ | MAC_CR_HPFILT_);

		adapterData->HashHi = 0UL;
		adapterData->HashLo = 0UL;
		goto PREPARE;
	}

	if(dev->net->flags & IFF_ALLMULTI) {
		SMSC_TRACE(DBG_MCAST, "Receive all Multicast Enabled");
		adapterData->set_bits_mask = MAC_CR_MCPAS_;
		adapterData->clear_bits_mask = (MAC_CR_PRMS_ | MAC_CR_HPFILT_);

		adapterData->HashHi = 0UL;
		adapterData->HashLo = 0UL;
		goto PREPARE;
	}


	if(dev->net->mc_count>0) {
		u32 dwHashH=0;
		u32 dwHashL=0;
		u32 dwCount=0;
		struct dev_mc_list *mc_list=dev->net->mc_list;

		adapterData->set_bits_mask = MAC_CR_HPFILT_;
		adapterData->clear_bits_mask = (MAC_CR_PRMS_ | MAC_CR_MCPAS_);

		while(mc_list!=NULL) {
			dwCount++;
			if((mc_list->dmi_addrlen)==6) {
				u32 dwMask=0x01UL;
				u32 dwBitNum=Rx_Hash(mc_list->dmi_addr);

				dwMask<<=(dwBitNum&0x1FUL);
				if(dwBitNum&0x20UL) {
					dwHashH|=dwMask;
				} else {
					dwHashL|=dwMask;
				}
			} else {
				SMSC_WARNING("dmi_addrlen!=6");
			}
			mc_list=mc_list->next;
		}
		if(dwCount!=((u32)(dev->net->mc_count))) {
			SMSC_WARNING("dwCount!=dev->net->mc_count");
		}
		SMSC_TRACE(DBG_MCAST, "Multicast: HASHH=0x%08X,HASHL=0x%08X",dwHashH,dwHashL);
		adapterData->HashHi = dwHashH;
		adapterData->HashLo = dwHashL;
	}
	else
	{
		adapterData->set_bits_mask = 0L;
		adapterData->clear_bits_mask = (MAC_CR_PRMS_ | MAC_CR_MCPAS_ | MAC_CR_HPFILT_);

		SMSC_TRACE(DBG_MCAST, "Receive own packets only.");
		adapterData->HashHi = 0UL;
		adapterData->HashLo = 0UL;
	}


PREPARE:
	up(&adapterData->RxFilterLock);

	dwHashHi=adapterData->HashHi;
	dwHashLo=adapterData->HashLo;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev,HASHH,dwHashHi));
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev,HASHL,dwHashLo));
	CHECK_RETURN_STATUS(smsc9500_read_reg(dev,MAC_CR,&local_MACCR));

	local_MACCR |= adapterData->set_bits_mask;
	local_MACCR &= ~(adapterData->clear_bits_mask);

	CHECK_RETURN_STATUS(smsc9500_write_reg(dev,MAC_CR,local_MACCR));


    SMSC_TRACE(DBG_MCAST, "<---------out of smsc9500_set_multicast");
	ret = 0;
DONE:
    return ret;
}

static void smsc9500_set_multicast(struct net_device *netdev)
{
	struct usbnet *dev=netdev_priv(netdev);
	smscusbnet_defer_myevent(dev, EVENT_SET_MULTICAST);

}

static int Phy_GetLinkMode(struct usbnet *dev)
{
	u32 dwTemp, dwValue, result=LINK_OFF;
	u16 wRegBcr=0;
	u16 wRegBSR,wRegLPA;
    int ret = SMSC9500_FAIL;

	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	SMSC_TRACE(DBG_LINK, "---------->in Phy_GetLinkMode");
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_BSR,&dwTemp));
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_BSR,&dwTemp));

	wRegBSR=LOWORD(dwTemp);

	adapterData->dwLinkSettings=LINK_OFF;

	if(wRegBSR&PHY_BSR_LINK_STATUS_) {
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_BCR,&dwValue));
		wRegBcr=LOWORD(dwValue);

		if(wRegBcr & PHY_BCR_AUTO_NEG_ENABLE_) {
			u32 linkSettings=LINK_AUTO_NEGOTIATE;
			u16 wRegADV=adapterData->wLastADVatRestart;
			CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_ANEG_LPA,&dwTemp));
			wRegLPA=LOWORD(dwTemp);

			if(wRegADV & PHY_ANEG_ADV_ASYMP_) {
				linkSettings |= LINK_ASYMMETRIC_PAUSE;
			}
			if(wRegADV & PHY_ANEG_ADV_SYMP_) {
				linkSettings |= LINK_SYMMETRIC_PAUSE;
			}
			if(wRegADV & PHY_ANEG_LPA_100FDX_) {
				linkSettings |= LINK_SPEED_100FD;
			}
			if(wRegADV & PHY_ANEG_LPA_100HDX_) {
				linkSettings |= LINK_SPEED_100HD;
			}
			if(wRegADV & PHY_ANEG_LPA_10FDX_) {
				linkSettings |= LINK_SPEED_10FD;
			}
			if(wRegADV & PHY_ANEG_LPA_10HDX_) {
				linkSettings |= LINK_SPEED_10HD;
			}
			adapterData->dwLinkSettings=linkSettings;

			wRegLPA &= wRegADV;
			if(wRegLPA & PHY_ANEG_LPA_100FDX_) {
				result = LINK_SPEED_100FD;
			} else if(wRegLPA & PHY_ANEG_LPA_100HDX_) {
				result = LINK_SPEED_100HD;
			} else if(wRegLPA & PHY_ANEG_LPA_10FDX_) {
				result = LINK_SPEED_10FD;
			} else if(wRegLPA & PHY_ANEG_LPA_10HDX_) {
				result = LINK_SPEED_10HD;
			}
		} else {
			if(wRegBcr & PHY_BCR_SPEED_SELECT_) {
				if(wRegBcr & PHY_BCR_DUPLEX_MODE_) {
					adapterData->dwLinkSettings=result=LINK_SPEED_100FD;
				} else {
					adapterData->dwLinkSettings=result=LINK_SPEED_100HD;
				}
			} else {
				if(wRegBcr & PHY_BCR_DUPLEX_MODE_) {
					adapterData->dwLinkSettings=result=LINK_SPEED_10FD;
				} else {
					adapterData->dwLinkSettings=result=LINK_SPEED_10HD;
				}
			}
		}
	}
	adapterData->dwLinkSpeed=result;
	SMSC_TRACE(DBG_LINK,"<----------out of Phy_GetLinkMode");

    ret = 0;
DONE:
    return ret;
}

static int Phy_UpdateLinkMode(struct usbnet *dev)
{

	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	int ret = SMSC9500_FAIL;
	u32 dwOldLinkSpeed=adapterData->dwLinkSpeed;
	u32 dwTemp,dwValue;

	SMSC_TRACE(DBG_LINK,"---------->in Phy_UpdateLinkMode");

	Phy_GetLinkMode(dev);

	if(dwOldLinkSpeed!=(adapterData->dwLinkSpeed)) {
		if(adapterData->dwLinkSpeed!=LINK_OFF) {
			u32 dwRegVal=0;
			switch(adapterData->dwLinkSpeed) {
			case LINK_SPEED_10HD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 10Mbps HD");
				break;
			case LINK_SPEED_10FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 10Mbps FD");
				break;
			case LINK_SPEED_100HD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 100Mbps HD");
				break;
			case LINK_SPEED_100FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 100Mbps FD");
				break;
			default:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at Unknown Link Speed, dwLinkSpeed=0x%08X",
					adapterData->dwLinkSpeed);
				break;
			}

			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	MAC_CR,&dwRegVal));
			dwRegVal&=~(MAC_CR_FDPX_|MAC_CR_RCVOWN_);
			switch(adapterData->dwLinkSpeed) {
			case LINK_SPEED_10HD:
			case LINK_SPEED_100HD:
				dwRegVal|=MAC_CR_RCVOWN_;
				break;
			case LINK_SPEED_10FD:
			case LINK_SPEED_100FD:
				dwRegVal|=MAC_CR_FDPX_;
				break;
			default:break;
			}

			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	MAC_CR, dwRegVal));

			if(adapterData->dwLinkSettings&LINK_AUTO_NEGOTIATE) {
				u16 linkPartner=0;
				u16 localLink=0;

				CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_ANEG_ADV, &dwTemp));
				localLink=LOWORD(dwTemp);
				CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_ANEG_LPA, &dwTemp));
				linkPartner=LOWORD(dwTemp);
				switch(adapterData->dwLinkSpeed) {
				case LINK_SPEED_10FD:
				case LINK_SPEED_100FD:
					if(((localLink&linkPartner)&((u16)PHY_ANEG_ADV_SYMP_)) != ((u16)0U)) {
						//Enable PAUSE receive and transmit
						dwTemp=0xFFFF0002UL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,FLOW, dwTemp));

						CHECK_RETURN_STATUS(smsc9500_read_reg(dev,AFC_CFG,&dwValue));
						dwValue=dwValue|0x0000000FUL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG,dwValue));

					} else if(((localLink&((u16)0x0C00U))==((u16)0x0C00U)) &&
							((linkPartner&((u16)0x0C00U))==((u16)0x0800U)))
					{
						//Enable PAUSE receive, disable PAUSE transmit
						dwTemp=0xFFFF0002UL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
						CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
						dwValue=dwValue&(~0x0000000FUL);
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));

					} else {
						//Disable PAUSE receive and transmit
						dwTemp=0x0UL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
						CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
						dwValue=dwValue&(~0x0000000FUL);
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));

					};break;
				case LINK_SPEED_10HD:
				case LINK_SPEED_100HD:

						dwTemp=0x0UL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
						CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
						dwValue=dwValue|0x0000000FUL;
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));

					break;
				default:break;
				}
				SMSC_TRACE(DBG_LINK_CHANGE,"LAN9500: %s,%s,%s,%s,%s,%s",
					(localLink&PHY_ANEG_ADV_ASYMP_)?"ASYMP":"     ",
					(localLink&PHY_ANEG_ADV_SYMP_)?"SYMP ":"     ",
					(localLink&PHY_ANEG_ADV_100F_)?"100FD":"     ",
					(localLink&PHY_ANEG_ADV_100H_)?"100HD":"     ",
					(localLink&PHY_ANEG_ADV_10F_)?"10FD ":"     ",
					(localLink&PHY_ANEG_ADV_10H_)?"10HD ":"     ");

				SMSC_TRACE(DBG_LINK_CHANGE,"Partner: %s,%s,%s,%s,%s,%s",
					(linkPartner&PHY_ANEG_LPA_ASYMP_)?"ASYMP":"     ",
					(linkPartner&PHY_ANEG_LPA_SYMP_)?"SYMP ":"     ",
					(linkPartner&PHY_ANEG_LPA_100FDX_)?"100FD":"     ",
					(linkPartner&PHY_ANEG_LPA_100HDX_)?"100HD":"     ",
					(linkPartner&PHY_ANEG_LPA_10FDX_)?"10FD ":"     ",
					(linkPartner&PHY_ANEG_LPA_10HDX_)?"10HD ":"     ");
			} else {
				switch(adapterData->dwLinkSpeed) {
				case LINK_SPEED_10HD:
				case LINK_SPEED_100HD:

					dwTemp=0x0UL;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
                    dwValue=dwValue|0x0000000FUL;
                    CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));
					break;
				default:

					dwTemp=0x0UL;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
					dwValue=dwValue&(~0x0000000FUL);
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));
					break;
				}
			}
			netif_carrier_on(dev->net);
			Tx_WakeQueue(dev,0x01);
			SetGpo(dev, adapterData->LinkLedOnGpio, !adapterData->LinkLedOnGpioPolarity);

		} else {
			SMSC_TRACE(DBG_LINK_CHANGE,"Link is now DOWN");
			Tx_StopQueue(dev,0x01);
			netif_carrier_off(dev->net);
			SetGpo(dev,  adapterData->LinkLedOnGpio, adapterData->LinkLedOnGpioPolarity);

			dwTemp=0x0UL;
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	FLOW, dwTemp));
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	AFC_CFG,&dwValue));
			dwValue=dwValue& (~0x0000000FUL);
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	AFC_CFG, dwValue));
		}
	}
	SMSC_TRACE(DBG_LINK,"<----------out of Phy_UpdateLinkMode");

    ret = 0;
DONE:
    return ret;
}

static int Phy_CheckLink(void * ptr)
{
	struct usbnet		*dev = ptr;
	SMSC9500_RX_STATS rx_stats;
	u32 dwValue, droppedFrame = 0;
    int ret = SMSC9500_FAIL;

    BUG_ON(!dev);
	SMSC_TRACE(DBG_LINK,"-------->in Phy_CheckLink");

	if(Phy_UpdateLinkMode(dev) < 0)return ret;

	if(dev->suspendFlag & AUTOSUSPEND_DETACH){
		if(!netif_carrier_ok(dev->net)){//Link is down, detach device
			//Set wakeup event
			SetLinkDownWakeupEvents(dev, WAKEPHY_ENERGY);

			//Enable smart detach
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,	HW_CFG,&dwValue));
			dwValue |= HW_CFG_SMDET_EN;
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,	HW_CFG, dwValue));
			dev->suspendFlag &= ~AUTOSUSPEND_DETACH;
			ret = SMSC9500_SUCCESS;
			goto DONE;
		}
	}

	if(smsc9500_get_stats(dev, (void*)&rx_stats) > 0){
		le32_to_cpus((u32*)&rx_stats.RxFifoDroppedFrames);
		rx_stats.RxFifoDroppedFrames &= 0xFFFFF; //This counter has 20 bits.
		if(dev->chipDependFeatures[FEATURE_NEWSTATIS_CNT]){//Statistics counters are rollover ones in LAN9500A
			if(rx_stats.RxFifoDroppedFrames >= dev->preRxFifoDroppedFrame){
				droppedFrame = rx_stats.RxFifoDroppedFrames - dev->preRxFifoDroppedFrame;
			}else{//Rollover
				droppedFrame = 0x100000 - dev->preRxFifoDroppedFrame + rx_stats.RxFifoDroppedFrames;
			}
			dev->preRxFifoDroppedFrame = rx_stats.RxFifoDroppedFrames;
		}else{
			droppedFrame = rx_stats.RxFifoDroppedFrames;
		}
		dev->stats.rx_dropped += droppedFrame;
	}

	if( (!(dev->StopLinkPolling)) && (!timer_pending(&dev->LinkPollingTimer))) {
		dev->LinkPollingTimer.expires=jiffies+HZ;
		add_timer(&(dev->LinkPollingTimer));
	}
	SMSC_TRACE(DBG_LINK,"<---------out of Phy_CheckLink");
	ret = SMSC9500_SUCCESS;
DONE:
    return ret;
}


static int phy_SetLink(struct usbnet *dev, u32 dwLinkRequest)
{
	u32 dwValue;
	u16 wTemp=0;
    int ret = SMSC9500_FAIL;

    SMSC_TRACE(DBG_LINK,"--------->in phy_SetLink");

	if(dwLinkRequest&LINK_AUTO_NEGOTIATE) {

		CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_ANEG_ADV,&dwValue));
		wTemp=LOWORD(dwValue);
		wTemp&=~PHY_ANEG_ADV_PAUSE_;
		if(dwLinkRequest&LINK_ASYMMETRIC_PAUSE) {
			wTemp|=PHY_ANEG_ADV_ASYMP_;
		}
		if(dwLinkRequest&LINK_SYMMETRIC_PAUSE) {
			wTemp|=PHY_ANEG_ADV_SYMP_;
		}
		wTemp&=~PHY_ANEG_ADV_SPEED_;
		if(dwLinkRequest&LINK_SPEED_10HD) {
			wTemp|=PHY_ANEG_ADV_10H_;
		}
		if(dwLinkRequest&LINK_SPEED_10FD) {
			wTemp|=PHY_ANEG_ADV_10F_;
		}

		if((dwLinkRequest&LINK_SPEED_100HD) && (dev->udev->speed == USB_SPEED_HIGH)) {
			wTemp|=PHY_ANEG_ADV_100H_;
		}
		if((dwLinkRequest&LINK_SPEED_100FD) && (dev->udev->speed == USB_SPEED_HIGH)) {
			wTemp|=PHY_ANEG_ADV_100F_;
		}

		dwValue=(u32)(wTemp) &0x0000FFFFUL;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_ANEG_ADV, dwValue));

		// begin to establish link
		wTemp=PHY_BCR_AUTO_NEG_ENABLE_|PHY_BCR_RESTART_AUTO_NEG_;
		dwValue=(u32)(wTemp) &0x0000FFFFUL;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_BCR, dwValue));
	} else {

		if(dwLinkRequest&(LINK_SPEED_100FD)) {
			dwLinkRequest=LINK_SPEED_100FD;
		} else if(dwLinkRequest&(LINK_SPEED_100HD)) {
			dwLinkRequest=LINK_SPEED_100HD;
		} else if(dwLinkRequest&(LINK_SPEED_10FD)) {
			dwLinkRequest=LINK_SPEED_10FD;
		} else if(dwLinkRequest&(LINK_SPEED_10HD)) {
			dwLinkRequest=LINK_SPEED_10HD;
		}
		if(dwLinkRequest&(LINK_SPEED_10FD|LINK_SPEED_100FD)) {
			wTemp|=PHY_BCR_DUPLEX_MODE_;
		}
		if(dwLinkRequest&(LINK_SPEED_100HD|LINK_SPEED_100FD)) {
			wTemp|=PHY_BCR_SPEED_SELECT_;
		}
		dwValue=(u32)(wTemp) &0x0000FFFFUL;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_BCR, dwValue));
	}
	SMSC_TRACE(DBG_LINK,"<---------out of phy_SetLink");
    ret = 0;
DONE:
    return ret;
}

static int Phy_SetAutoMdix(
    struct usbnet *dev,
    u16 wAutoMdix
    )
{
    u32 SpecialCtrlSts=0U;
    int ret = SMSC9500_FAIL;

    if (wAutoMdix > 2)
    {
		SMSC_WARNING("LAN9500 Auto MDIX feature controlled by hardware strap\n");
    }
    else
    {
	    CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_SPECIAL_CTRL_STS, &SpecialCtrlSts));

        SpecialCtrlSts = (((wAutoMdix+4) << 13) | (SpecialCtrlSts&0x1FFF));
        CHECK_RETURN_STATUS(smsc9500_write_phy(dev, PHY_SPECIAL_CTRL_STS, SpecialCtrlSts));

        if (wAutoMdix & AMDIX_ENABLE)
        {
			SMSC_WARNING("LAN9500 Auto MDIX hardware strap was overiden by driver. AutoMdix is enabled");
        }
        else if (wAutoMdix & AMDIX_DISABLE_CROSSOVER)
        {
			SMSC_WARNING("LAN9500 Auto MDIX hardware strap was overiden by driver. AutoMdix is disabled, use crossover cable.");
        }
        else
        {
			SMSC_WARNING("LAN9500 Auto MDIX hardware strap was overiden by driver. AutoMdix is disabled, use straight cable.");
        }
    }

    ret = 0;
DONE:
    return ret;
}

static BOOLEAN Phy_Initialize(
	struct usbnet *dev,
	u32 dwPhyAddr,
	u32 dwLinkRequest)
{
	BOOLEAN result=FALSE, bConfigureAutoMdix = FALSE;
	u32 dwTemp=0,dwValue, address;
	u32 dwLoopCount=0;
    u32 phy_id_1, phy_id_2;

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	SMSC_TRACE(DBG_INIT,"-->Phy_Initialize");
	SMSC_ASSERT(dwLinkRequest<=0x7FUL);

    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, HW_CFG, &dwValue));

    if(dwValue & HW_CFG_PSEL_){
       SMSC_TRACE(DBG_INIT,"using external PHY ");

        if(dwPhyAddr <= 31){
            // Using external PHY board on MII connector
            // First isolate all PHYs we can find...

            for (address=0;address<=31;address++)
            {
                adapterData->dwPhyAddress = address;
                dwValue=(u32)PHY_BCR_ISOLATE;
                CHECK_RETURN_STATUS(smsc9500_write_phy(dev, PHY_BCR, dwValue));
            }

		// Now put the PHY at the registry parsed address out of isolation.
		adapterData->dwPhyAddress = dwPhyAddr;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev, PHY_BCR, 0x0UL));
        }else{//Auto detect PHY

            phy_id_1 = 0xFFFFU;
            phy_id_2 = 0xFFFFU;
            for (address=0; address<=31; address++)
            {
                adapterData->dwPhyAddress = address;

                CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_ID_1, &phy_id_1));
                CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_ID_2, &phy_id_2));
                if((phy_id_1 != 0x7FFFU) && (phy_id_1 != 0xFFFFU) && (phy_id_1 != 0x0000U)){
                    SMSC_TRACE(DBG_INIT,"Deteced Phy at address = 0x%02X", address);
                    break;
                }
            }
        }
		SMSC_TRACE(DBG_INIT,"using external PHY ");

	} else {
//USE_INTERNAL_PHY
		SMSC_TRACE(DBG_INIT,"using internal PHY ");
		adapterData->dwPhyAddress=1;

        bConfigureAutoMdix = TRUE;
	}

	{
		u32 dwPhyBcr;
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_ID_2,&dwTemp));
		adapterData->bPhyRev=((BYTE)(dwTemp&(0x0FUL)));
		adapterData->bPhyModel=((BYTE)((dwTemp>>4)&(0x3FUL)));
		adapterData->dwPhyId=((dwTemp&(0xFC00UL))<<8);
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_ID_1,&dwTemp));
		adapterData->dwPhyId|=((dwTemp&(0x0000FFFFUL))<<2);


		SMSC_TRACE(DBG_INIT,"dwPhyId==0x%08X,bPhyModel==0x%02X,bPhyRev==0x%02X",
			adapterData->dwPhyId,
			adapterData->bPhyModel,
			adapterData->bPhyRev);

		adapterData->dwLinkSpeed = LINK_INIT;
		adapterData->dwLinkSettings=LINK_INIT;
		//reset the PHY
		dwPhyBcr=(u32)PHY_BCR_RESET_ ;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_BCR, dwPhyBcr));

		dwLoopCount = 20;
		do {

			mdelay(100);
			CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_BCR,&dwPhyBcr));

			dwLoopCount--;
		} while((dwLoopCount>0) && ((u16)dwPhyBcr&PHY_BCR_RESET_));


		if((u16)dwPhyBcr&PHY_BCR_RESET_) {
			SMSC_WARNING("PHY reset failed to complete.");
			goto DONE;
		}
		else
			SMSC_TRACE(DBG_INIT,"PHY reset!!!");
	}

    if (bConfigureAutoMdix)
    {
		Phy_SetAutoMdix(dev, (u16)auto_mdix);
	}
	phy_SetLink(dev,dwLinkRequest);

	result=TRUE;
DONE:
	SMSC_TRACE(DBG_INIT,"<--Phy_Initialize, result=%s\n",result?"TRUE":"FALSE");
	return result;
}



static int smsc9500_get_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
			cmd->supported=
				SUPPORTED_10baseT_Half |
				SUPPORTED_10baseT_Full |
				SUPPORTED_100baseT_Half |
				SUPPORTED_100baseT_Full |
				SUPPORTED_Autoneg |
				SUPPORTED_MII;
			cmd->advertising=ADVERTISED_MII;

			if(adapterData->dwLinkSettings & LINK_SPEED_10HD)
				cmd->advertising|=ADVERTISED_10baseT_Half;
			if(adapterData->dwLinkSettings & LINK_SPEED_10FD)
				cmd->advertising|=ADVERTISED_10baseT_Full;
			if(adapterData->dwLinkSettings & LINK_SPEED_100HD)
				cmd->advertising|=ADVERTISED_100baseT_Half;
			if(adapterData->dwLinkSettings & LINK_SPEED_100FD)
				cmd->advertising|=ADVERTISED_100baseT_Full;
			if(adapterData->dwLinkSettings & LINK_AUTO_NEGOTIATE) {
				cmd->advertising|=ADVERTISED_Autoneg;
				cmd->autoneg=AUTONEG_ENABLE;
			} else cmd->autoneg=AUTONEG_DISABLE;
			if(adapterData->dwLinkSpeed & (LINK_SPEED_100HD|LINK_SPEED_100FD))
				cmd->speed=SPEED_100;
			else cmd->speed=SPEED_10;
			if(adapterData->dwLinkSpeed & (LINK_SPEED_10FD|LINK_SPEED_100FD))
				cmd->duplex=DUPLEX_FULL;
			else cmd->duplex=DUPLEX_HALF;
			cmd->port=PORT_MII;
			cmd->phy_address=(u8)adapterData->dwPhyAddress;
			cmd->transceiver=XCVR_INTERNAL;
			cmd->maxtxpkt=0;
			cmd->maxrxpkt=0;


			return 0;

}


static int smsc9500_set_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	int result=-EFAULT;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);


	u16 speed=0;
	u8 duplex=0;
	u8 autoneg=0;

	if(adapterData->dwLinkSettings&LINK_AUTO_NEGOTIATE) {
		autoneg=AUTONEG_ENABLE;
	} else {
		autoneg=AUTONEG_DISABLE;
	}
	if(adapterData->dwLinkSpeed&(LINK_SPEED_100HD|LINK_SPEED_100FD))
	{
		speed=SPEED_100;
	} else {
		speed=SPEED_10;
	}
	if(adapterData->dwLinkSpeed&(LINK_SPEED_10FD|LINK_SPEED_100FD))
	{
		duplex=DUPLEX_FULL;
	} else {
		duplex=DUPLEX_HALF;
	}
	if((cmd->speed!=100)&&(cmd->speed!=10)) {
		result=-EOPNOTSUPP;
		goto DONE;
	}
	if((cmd->duplex!=DUPLEX_FULL)&&(cmd->duplex!=DUPLEX_HALF)) {
		result=-EOPNOTSUPP;
		goto DONE;
	}
	if((cmd->autoneg!=AUTONEG_ENABLE)&&(cmd->autoneg!=AUTONEG_DISABLE)) {
		result=-EOPNOTSUPP;
		goto DONE;
	}
	if((cmd->autoneg!=autoneg)||
		(cmd->speed!=speed)||
		(cmd->duplex!=duplex))
	{
		if(cmd->autoneg==AUTONEG_ENABLE) {
			u32 dwBcrValue;

			dwBcrValue=PHY_BCR_AUTO_NEG_ENABLE_|PHY_BCR_RESTART_AUTO_NEG_;
			if(smsc9500_write_phy(dev,PHY_BCR, dwBcrValue) < 0)goto DONE;

		} else {
			u32  dwBcrValue;
			u16 wBcr;
			if(smsc9500_read_phy(dev,PHY_BCR,&dwBcrValue) < 0)goto DONE;
			wBcr=(u16)dwBcrValue;
			if(cmd->speed==SPEED_100) {
				wBcr|=PHY_BCR_SPEED_SELECT_;
			} else {
				wBcr&=(~PHY_BCR_SPEED_SELECT_);
			}
			if(cmd->duplex==DUPLEX_FULL) {
				wBcr|=PHY_BCR_DUPLEX_MODE_;
			} else {
				wBcr&=(~PHY_BCR_DUPLEX_MODE_);
			}
			wBcr &= ~PHY_BCR_AUTO_NEG_ENABLE_;
			if(smsc9500_write_phy(dev,PHY_BCR,wBcr) < 0)goto DONE;
		}
	}
	result=0;

DONE:
			return result;
}

void smsc9500_get_drvinfo (struct net_device *net, struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

			strcpy(info->driver,"Smsc9500");
			memset(&info->version,0,sizeof(info->version));
			sprintf(info->version,"%lX.%02lX.%02lX",
				(DRIVER_VERSION>>16),(DRIVER_VERSION>>8)&0xFF,(DRIVER_VERSION&0xFFUL));
			memset(&info->fw_version,0,sizeof(info->fw_version));
			sprintf(info->fw_version,"%lu",(adapterData->dwIdRev)&0xFFFFUL);
			memset(&info->bus_info,0,sizeof(info->bus_info));
			memset(&info->reserved1,0,sizeof(info->reserved1));
			memset(&info->reserved2,0,sizeof(info->reserved2));
			info->n_stats=0;
			info->testinfo_len=0;
			info->eedump_len=0;
			info->regdump_len=0;

}

static u32 smsc9500_get_link (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);


			if(adapterData->dwLinkSpeed!=LINK_OFF)
				return 1;
			else
				return 0;

}

static u32 smsc9500_get_msglevel (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return dev->msg_enable;
}


static void smsc9500_set_msglevel (struct net_device *net, u32 level)
{
	struct usbnet *dev = netdev_priv(net);

	dev->msg_enable = level;
}


static void
smsc9500_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	wolinfo->supported=(WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC);
	wolinfo->wolopts= adapterData->WolWakeupOpts;
}

static int
smsc9500_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	int result=-EFAULT;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	adapterData->WolWakeupOpts = wolinfo->wolopts;
	result=0;

	return result;
}

static int smsc9500_get_eeprom_len(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	return adapterData->eepromSize;
}

static int smsc9500_get_eeprom(struct net_device *netdev, struct ethtool_eeprom *ee, u8 *data)
{
	struct usbnet *dev=netdev_priv(netdev);
	int offset = ee->offset;
	int len = ee->len;
	int result = 0;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	ee->magic = LAN9500_EEPROM_MAGIC;

	if(len == 0){
		return 0;
	}

	if(offset + len > adapterData->eepromSize){
		SMSC_WARNING("EEPROM address is out of range");
		result = -EINVAL;
	}else{
		if(smsc9500_read_eeprom(dev, offset, len,  data) < 0){
			result = -EFAULT;
		}
	}

	return result;
}

static int smsc9500_set_eeprom(struct net_device *netdev, struct ethtool_eeprom *ee, u8 *data)
{
	struct usbnet *dev=netdev_priv(netdev);
	int offset = ee->offset;
	int len = ee->len;
	int result = 0;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(len == 0){
		return 0;
	}

	if(offset + len > adapterData->eepromSize){
		SMSC_WARNING("EEPROM address is out of range");
		result = -EINVAL;
		return result;
	}

	if(ee->magic != LAN9500_EEPROM_MAGIC){
		SMSC_WARNING("EEPROM: magic value mismatch, writing fail, magic = 0x%x", ee->magic);
		result = -EFAULT;
		return result;
	}

	if(smsc9500_write_eeprom(dev, offset, len,  data) < 0){
		result=-EFAULT;
	}

	return result;
}

static int smsc9500_eeprom_size(struct usbnet *dev)
{
#define CHECK_SIZE	4
	u32 dwValue;
	int size = 0;
	int i;
	char save[CHECK_SIZE+1];
	char saveEach[CHECK_SIZE+1];

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, E2P_CMD, &dwValue));
    if (!(dwValue & E2P_CMD_LOADED_))
    {
		size = 0;
		goto DONE;
    }

	if(smsc9500_read_eeprom(dev, 0, CHECK_SIZE,  save) < 0){//Save first 4 bytes
		goto DONE;
	}

	for(i=128; i<=MAX_EEPROM_SIZE; i+=128){
		if(smsc9500_read_eeprom(dev, i, CHECK_SIZE,  saveEach) < 0){
			goto DONE;
		}
		if(!strncmp(save, saveEach, CHECK_SIZE)){
			size = i;
			break;
		}
	}

DONE:
	return size;

}

/* We need to override some ethtool_ops so we require our
   own structure so we don't interfere with other usbnet
   devices that may be connected at the same time. */
static struct ethtool_ops smsc9500_ethtool_ops = {
	.get_drvinfo		= smsc9500_get_drvinfo,
	.get_link		= smsc9500_get_link,
	.get_msglevel		= smsc9500_get_msglevel,
	.set_msglevel		= smsc9500_set_msglevel,
	.get_wol		= smsc9500_get_wol,
	.set_wol		= smsc9500_set_wol,
	.get_settings		= smsc9500_get_settings,
	.set_settings		= smsc9500_set_settings,
	.get_eeprom_len		= smsc9500_get_eeprom_len,
	.get_eeprom			= smsc9500_get_eeprom,
	.set_eeprom			= smsc9500_set_eeprom,
};

static int Smsc9500_do_ioctl(
	struct net_device *netdev,
	struct ifreq *ifr,
	int cmd)
{
	int result=0;
	struct usbnet *dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	void __user *userAddr=NULL;

	SMSC_TRACE(DBG_IOCTL,"---->Smsc9500_do_ioctl");
	if(netdev==NULL) {
		SMSC_WARNING("netdev==NULL");
		result=-EFAULT;
		goto DONE;
	}

	if(ifr==NULL) {
		SMSC_WARNING("ifr==NULL");
		result=-EFAULT;
		goto DONE;
	}
	userAddr=ifr->ifr_data;

	switch(cmd) {

	case SIOCGMIIPHY:
	case SIOCDEVPRIVATE:
		SMSC_TRACE(DBG_IOCTL,"SIOCGMIIPHY");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			miiData->phy_id=1;
		};
		break;

	case SIOCGMIIREG:
	case SIOCDEVPRIVATE+1:
		SMSC_TRACE(DBG_IOCTL,"SIOCGMIIREG");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			{
				u32 dwValue;
				if(smsc9500_read_phy(dev,miiData->reg_num,&dwValue) < 0){
                    result = -EFAULT;
                }
				miiData->val_out=(u16)dwValue;
			}
		};break;

	case SIOCSMIIREG:
	case SIOCDEVPRIVATE+2:
		SMSC_TRACE(DBG_IOCTL,"SIOCSMIIREG");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			{
				u32 dwValue;
				dwValue=miiData->val_in;
				if(smsc9500_write_phy(dev,miiData->reg_num, dwValue) < 0){
                    result = -EFAULT;
                }
			}
		};break;

	case SMSC9500_IOCTL:
		result=smsc9500_private_ioctl(adapterData, dev, (PSMSC9500_IOCTL_DATA)userAddr);
		break;

	default:
		SMSC_WARNING("unknown cmd = 0x%08X",cmd);
		result=-EOPNOTSUPP;
		break;
	}

DONE:

	SMSC_TRACE(DBG_IOCTL,"<--Smsc9500_do_ioctl");
	return result;
}

static int smsc9500_private_ioctl(PADAPTER_DATA  privateData, struct usbnet *dev, PSMSC9500_IOCTL_DATA ioctlData)
{
	BOOLEAN success=FALSE;
    int i;
	u32 dwBuf;
    u32 offset;
	if(ioctlData->dwSignature!=SMSC9500_APP_SIGNATURE) {
		goto DONE;
	}

	switch(ioctlData->dwCommand) {
	case COMMAND_GET_SIGNATURE:
		success=TRUE;
		break;
	case COMMAND_GET_CONFIGURATION:
		ioctlData->Data[0]=DRIVER_VERSION;
		ioctlData->Data[1]=link_mode;
		ioctlData->Data[2] = privateData->macAddrHi16;
		ioctlData->Data[3] = privateData->MmacAddrLo32;
		ioctlData->Data[4]=debug_mode;
		ioctlData->Data[5]=privateData->dwIdRev;
		ioctlData->Data[6]=privateData->dwFpgaRev;
		ioctlData->Data[7] = 1;
		ioctlData->Data[8]=privateData->dwPhyId;
		ioctlData->Data[9]=privateData->bPhyModel;
		ioctlData->Data[10]=privateData->bPhyRev;
		ioctlData->Data[11]=privateData->dwLinkSpeed;
		ioctlData->Data[12] = privateData->eepromSize / 128; //Unit is 128B
		sprintf(ioctlData->Strng1,"%s, %s",__DATE__,__TIME__);

		success=TRUE;
		break;
	case COMMAND_LAN_GET_REG:
        offset = ioctlData->Data[0];

		if((ioctlData->Data[0] <= LAN_REGISTER_RANGE) && ((ioctlData->Data[0]&0x3UL)==0))
		{
			if(smsc9500_read_reg(dev, offset, &dwBuf) >= 0){
                ioctlData->Data[1] = dwBuf;
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Reading LAN9500 Mem Map Failed");
			goto MEM_MAP_ACCESS_FAILED;
		}
		break;
	case COMMAND_LAN_SET_REG:
		if((ioctlData->Data[0] <= LAN_REGISTER_RANGE) && ((ioctlData->Data[0]&0x3UL)==0))
		{
            offset = ioctlData->Data[0];
            dwBuf = ioctlData->Data[1];
			if(smsc9500_write_reg(dev, offset,  dwBuf) >= 0){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Writing LAN9500 Mem Map Failed");
MEM_MAP_ACCESS_FAILED:
			SMSC_WARNING("  Invalid offset == 0x%08lX",ioctlData->Data[0]);
			if(ioctlData->Data[0] > LAN_REGISTER_RANGE) {
				SMSC_WARNING("    Out of range");
			}
			if(ioctlData->Data[0]&0x3UL) {
				SMSC_WARNING("    Not u32 aligned");
			}
		}
		break;
	case COMMAND_MAC_GET_REG:
		if((ioctlData->Data[0] >= MAC_REGISTER_RANGE_MIN)&& (ioctlData->Data[0] <= MAC_REGISTER_RANGE_MAX) && ((ioctlData->Data[0]&0x3UL)==0)) {
            offset = ioctlData->Data[0];
			if(smsc9500_read_reg(dev, offset, &dwBuf) >= 0){
                ioctlData->Data[1] = dwBuf;
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Reading Mac Register Failed");
			goto MAC_ACCESS_FAILURE;
		}
		break;
	case COMMAND_MAC_SET_REG:
		if((ioctlData->Data[0] >= MAC_REGISTER_RANGE_MIN)&& (ioctlData->Data[0] <= MAC_REGISTER_RANGE_MAX) && ((ioctlData->Data[0]&0x3UL)==0)) {
            offset = ioctlData->Data[0];
            dwBuf = ioctlData->Data[1];
			if(smsc9500_write_reg(dev, offset, dwBuf) >= 0){
                ioctlData->Data[1] = dwBuf;
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Writing Mac Register Failed");
MAC_ACCESS_FAILURE:
			if(!(privateData->LanInitialized)) {

				SMSC_WARNING("  LAN Not Initialized,");
				SMSC_WARNING("    Use ifconfig to bring interface UP");
			}
			if(!((ioctlData->Data[0] >= MAC_REGISTER_RANGE_MIN)&& (ioctlData->Data[0] <= MAC_REGISTER_RANGE_MAX))) {
				SMSC_WARNING("  Invalid index == 0x%08lX",ioctlData->Data[0]);
			}
		}
		break;
	case COMMAND_PHY_GET_REG:
		if((ioctlData->Data[0]<32)&&(privateData->LanInitialized)) {
            offset = ioctlData->Data[0];
			if(smsc9500_read_phy(dev,offset, &dwBuf) >= 0){
				success=TRUE;
                ioctlData->Data[1] = dwBuf;
			}
		} else {
			SMSC_WARNING("Reading Phy Register Failed");
			goto PHY_ACCESS_FAILURE;
		}
		break;
	case COMMAND_PHY_SET_REG:
		if((ioctlData->Data[0]<32)&&(privateData->LanInitialized)) {
            offset = ioctlData->Data[0];
            dwBuf = ioctlData->Data[1];
			if(smsc9500_write_phy(dev,offset, dwBuf) >= 0){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Writing Phy Register Failed");
PHY_ACCESS_FAILURE:
			if(!(privateData->LanInitialized)) {
				SMSC_WARNING("  Lan Not Initialized,");
				SMSC_WARNING("    Use ifconfig to bring interface UP");
			}
			if(!(ioctlData->Data[0]<32)) {
				SMSC_WARNING("  Invalid index == 0x%ld",ioctlData->Data[0]);
			}
		}
		break;
    case COMMAND_GET_EEPROM:
    {
        BYTE cBuf;
        offset = ioctlData->Data[0];
        if(offset < privateData->eepromSize) {
            if(smsc9500_read_eeprom(dev,offset, 1, &cBuf) >= 0){
                success=TRUE;
                ioctlData->Data[1] = cBuf;
            }
        } else {
            SMSC_WARNING("Reading EEPROM Failed");
            goto PHY_ACCESS_FAILURE;
        }
    }
        break;
    case COMMAND_SET_EEPROM:
    {
        BYTE cBuf;
        offset = ioctlData->Data[0];
        cBuf = (BYTE)ioctlData->Data[1];
        if(offset < privateData->eepromSize) {
            if(smsc9500_write_eeprom(dev, offset, 1, &cBuf) >= 0){
                success=TRUE;
            }
        } else {
            SMSC_WARNING("Writing EEPROM Failed");
            if(!(offset < privateData->eepromSize)) {
                SMSC_WARNING("  Invalid eeprom offset == 0x%d",offset);
            }
        }
    }
        break;

	case COMMAND_DUMP_LAN_REGS:

        success=TRUE;
        for(i=0; i<MAX_LAN_REG_NUM; i++){
	        if(smsc9500_read_reg(dev, LanRegMap[i], &dwBuf) < 0){
	            SMSC_WARNING("Failed to read LAN reg 0x%x", (unsigned int)LanRegMap[i]);
                success = FALSE;
	        }else{
	            ioctlData->Data[i] = dwBuf;
	        }
        }
		break;
	case COMMAND_DUMP_MAC_REGS:
		if(privateData->LanInitialized) {
            success=TRUE;
	        for(i=0; i<MAX_MAC_REG_NUM; i++){
	            if(smsc9500_read_reg(dev, MacRegMap[i], &dwBuf) < 0){
	                SMSC_WARNING("Failed to read MAC reg 0x%x", (unsigned int)MacRegMap[i]);
                    success = FALSE;
	            }else{
	                ioctlData->Data[i] = dwBuf;
	            }
	        }
		} else {
			SMSC_WARNING("Mac Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		}
		break;
	case COMMAND_DUMP_PHY_REGS:
		if(privateData->LanInitialized) {
            success=TRUE;
            for(i=0; i<MAX_PHY_REG_NUM; i++){
                if(smsc9500_read_phy(dev, PhyRegMap[i], &dwBuf) < 0){
                    SMSC_WARNING("Failed to read PHY reg 0x%x", (unsigned int)PhyRegMap[i]);
                    success = FALSE;
                }else{
                    ioctlData->Data[i] = dwBuf;
                }
            }
		} else {
			SMSC_WARNING("Phy Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		}
		break;
	case COMMAND_DUMP_EEPROM:
		{
			success=TRUE;

			if(smsc9500_read_eeprom(dev, 0, privateData->eepromSize,  (BYTE*)ioctlData->Data) < 0){
				success=FALSE;
			}
		};break;
	case COMMAND_GET_MAC_ADDRESS:

		if(privateData->LanInitialized) {
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev, ADDRH, &dwBuf));
            ioctlData->Data[0] = dwBuf;
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev, ADDRL, &dwBuf));
            ioctlData->Data[1] = dwBuf;
			success=TRUE;
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		}
		break;

	case COMMAND_SET_MAC_ADDRESS:
		if(privateData->LanInitialized)
		{
			u32 dwLow32=ioctlData->Data[1];
			u32 dwHigh16=ioctlData->Data[0];

			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, ADDRH, dwHigh16));
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, ADDRL, dwLow32));

		    dev->net->dev_addr[0]=LOBYTE(LOWORD(dwLow32));
		    dev->net->dev_addr[1]=HIBYTE(LOWORD(dwLow32));
		    dev->net->dev_addr[2]=LOBYTE(HIWORD(dwLow32));
		    dev->net->dev_addr[3]=HIBYTE(HIWORD(dwLow32));
		    dev->net->dev_addr[4]=LOBYTE(LOWORD(dwHigh16));
		    dev->net->dev_addr[5]=HIBYTE(LOWORD(dwHigh16));

			success=TRUE;
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;

	case COMMAND_LOAD_MAC_ADDRESS:
		if(privateData->LanInitialized) {
            if(smsc9500_read_eeprom(dev, EEPROM_MAC_OFFSET, 6,  dev->net->dev_addr) == 0){
                dwBuf = dev->net->dev_addr[0] | dev->net->dev_addr[1] << 8 | dev->net->dev_addr[2] << 16 | dev->net->dev_addr[3] << 24;
                ioctlData->Data[1] = dwBuf;
                CHECK_RETURN_STATUS(smsc9500_write_reg(dev, ADDRL, ioctlData->Data[1]));
                dwBuf = dev->net->dev_addr[4] | dev->net->dev_addr[5] << 8;
                ioctlData->Data[0] = dwBuf;
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, ADDRH, ioctlData->Data[0]));

			    success=TRUE;
			} else {
				SMSC_WARNING("Failed to Load Mac Address");
			}
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;
	case COMMAND_SAVE_MAC_ADDRESS:
		if(privateData->LanInitialized) {
            u32 dwLow32 = ioctlData->Data[1];
            u32 dwHigh16 = ioctlData->Data[0];

            cpu_to_le32s((u32*)&dwLow32);
            cpu_to_le32s((u32*)&dwHigh16);

			if((smsc9500_write_eeprom(dev, EEPROM_MAC_OFFSET, 4, (BYTE*)&dwLow32) == 0) &&
                (smsc9500_write_eeprom(dev, EEPROM_MAC_OFFSET+4, 2, (BYTE*)&dwHigh16) == 0)){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;

	case COMMAND_SET_DEBUG_MODE:
		debug_mode=ioctlData->Data[0];
		if(debug_mode&0x04UL) {
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, GPIO_CFG, 0x00670700UL));
			success=TRUE;
		} else {
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, GPIO_CFG, 0x70070000));
			success=TRUE;
		}
		success=TRUE;
		break;
	case COMMAND_SET_LINK_MODE:
		link_mode=(ioctlData->Data[0]&0x7FUL);
		if(privateData->LanInitialized) {
			phy_SetLink(dev,link_mode);
		}
		success=TRUE;
		break;
	case COMMAND_GET_LINK_MODE:
		ioctlData->Data[0]=link_mode;
		success=TRUE;
		break;
	case COMMAND_CHECK_LINK:
		Phy_UpdateLinkMode(dev);
		success=TRUE;
		break;

    case COMMAND_GET_ERRORS:

        ioctlData->Data[0] = dev->extra_error_cnts.tx_epipe;
        ioctlData->Data[1] = dev->extra_error_cnts.tx_eproto;
        ioctlData->Data[2] = dev->extra_error_cnts.tx_etimeout;
        ioctlData->Data[3] = dev->extra_error_cnts.tx_eilseq;

        ioctlData->Data[4] = dev->extra_error_cnts.rx_epipe;
        ioctlData->Data[5] = dev->extra_error_cnts.rx_eproto;
        ioctlData->Data[6] = dev->extra_error_cnts.rx_etimeout;
        ioctlData->Data[7] = dev->extra_error_cnts.rx_eilseq;
        ioctlData->Data[8] = dev->extra_error_cnts.rx_eoverflow;

        success = TRUE;
        break;
	case COMMAND_READ_BYTE:
		ioctlData->Data[1]=(*((volatile BYTE *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_READ_WORD:
		ioctlData->Data[1]=(*((volatile u16 *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_READ_DWORD:
		ioctlData->Data[1]=(*((volatile u32 *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_WRITE_BYTE:
		(*((volatile BYTE *)(ioctlData->Data[0])))=
			((BYTE)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_WRITE_WORD:
		(*((volatile u16 *)(ioctlData->Data[0])))=
			((u16)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_WRITE_DWORD:
		(*((volatile u32 *)(ioctlData->Data[0])))=
			((u32)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_SET_AMDIX_STS:
		auto_mdix=(ioctlData->Data[0]);
		if(privateData->LanInitialized) {
			Phy_SetAutoMdix(dev, (u16)auto_mdix);
		}
		success=TRUE;
		break;
	case COMMAND_GET_AMDIX_STS:
		ioctlData->Data[0]=auto_mdix;
		success=TRUE;
		break;

	default:break;//make lint happy
	}

DONE:
	if((success)&&(ioctlData!=NULL)) {
		ioctlData->dwSignature=SMSC9500_DRIVER_SIGNATURE;
		return SMSC9500_SUCCESS;
	}
	return SMSC9500_FAIL;

}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
static const struct net_device_ops smsc95xx_netdev_ops =
{
        .ndo_open               = smscusbnet_open,
        .ndo_stop               = smscusbnet_stop,
        .ndo_start_xmit         = smscusbnet_start_xmit,
        .ndo_tx_timeout         = smscusbnet_tx_timeout,
        .ndo_change_mtu         = smscusbnet_change_mtu,
        .ndo_set_mac_address    = eth_mac_addr,
        .ndo_validate_addr      = eth_validate_addr,
        .ndo_do_ioctl           = Smsc9500_do_ioctl,
	.ndo_set_multicast_list = smsc9500_set_multicast,
        .ndo_get_stats          = smscusbnet_get_stats,
};
#endif //linux 2.6.29

static int smsc9500_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret=0;
	PADAPTER_DATA adapterData=NULL;
	u32 dwBuf;
	char version[15];

	SMSC_TRACE(DBG_INIT,"---------->in smsc9500_bind\n");

    //Init system control and status regsiter map
    LanRegMap[LAN_REG_ID_REV] = ID_REV;
    LanRegMap[LAN_REG_FPGA_REV] = FPGA_REV;
    LanRegMap[LAN_REG_INT_STS] = INT_STS;
    LanRegMap[LAN_REG_RX_CFG] = RX_CFG;
    LanRegMap[LAN_REG_TX_CFG] = TX_CFG;
    LanRegMap[LAN_REG_HW_CFG] = HW_CFG;
    LanRegMap[LAN_REG_RX_FIFO_INF] = RX_FIFO_INF;
    LanRegMap[LAN_REG_TX_FIFO_INF] = TX_FIFO_INF;
    LanRegMap[LAN_REG_PMT_CTRL] = PM_CTRL;
    LanRegMap[LAN_REG_LED_GPIO_CFG] = LED_GPIO_CFG;
    LanRegMap[LAN_REG_GPIO_CFG] = GPIO_CFG;
    LanRegMap[LAN_REG_AFC_CFG] = AFC_CFG;
    LanRegMap[LAN_REG_E2P_CMD] = E2P_CMD;
    LanRegMap[LAN_REG_E2P_DATA] = E2P_DATA;
    LanRegMap[LAN_REG_BURST_CAP] = BURST_CAP;
    LanRegMap[LAN_REG_STRAP_DBG] = STRAP_DBG;
    LanRegMap[LAN_REG_DP_SEL] = DP_SEL;
    LanRegMap[LAN_REG_DP_CMD] = DP_CMD;
    LanRegMap[LAN_REG_DP_ADDR] = DP_ADDR;
    LanRegMap[LAN_REG_DP_DATA0] = DP_DATA0;
    LanRegMap[LAN_REG_DP_DATA1] = DP_DATA1;
    LanRegMap[LAN_REG_GPIO_WAKE] = GPIO_WAKE;
    LanRegMap[LAN_REG_INT_EP_CTL] = INT_EP_CTL;
    LanRegMap[LAN_REG_BULK_IN_DLY] = BULK_IN_DLY;

    //Init MAC register map
    MacRegMap[MAC_REG_MAC_CR] = MAC_CR;
    MacRegMap[MAC_REG_ADDRH] = ADDRH;
    MacRegMap[MAC_REG_ADDRL] = ADDRL;
    MacRegMap[MAC_REG_HASHH] = HASHH;
    MacRegMap[MAC_REG_HASHL] = HASHL;
    MacRegMap[MAC_REG_MII_ADDR] = MII_ADDR;
    MacRegMap[MAC_REG_MII_DATA] = MII_DATA;
    MacRegMap[MAC_REG_FLOW] = FLOW;
    MacRegMap[MAC_REG_VLAN1] = VLAN1;
    MacRegMap[MAC_REG_VLAN2] = VLAN2;
    MacRegMap[MAC_REG_WUFF] = WUFF;
    MacRegMap[MAC_REG_WUCSR] = WUCSR;
    MacRegMap[MAC_REG_COE_CR] = COE_CR;

    //Init PHY map
    PhyRegMap[PHY_REG_BCR] = PHY_BCR;
    PhyRegMap[PHY_REG_BSR] = PHY_BSR;
    PhyRegMap[PHY_REG_ID1] = PHY_ID_1;
    PhyRegMap[PHY_REG_ID2] = PHY_ID_2;
    PhyRegMap[PHY_REG_ANEG_ADV] = PHY_ANEG_ADV;
    PhyRegMap[PHY_REG_ANEG_LPA] = PHY_ANEG_LPA;
    PhyRegMap[PHY_REG_ANEG_ER] = PHY_ANEG_REG;
    PhyRegMap[PHY_REG_SILICON_REV] = PHY_SILICON_REV;
    PhyRegMap[PHY_REG_MODE_CTRL_STS] = PHY_MODE_CTRL_STS;
    PhyRegMap[PHY_REG_SPECIAL_MODES] = PHY_SPECIAL_MODES;
    PhyRegMap[PHY_REG_TSTCNTL] = PHY_TSTCNTL;
    PhyRegMap[PHY_REG_TSTREAD1] = PHY_TSTREAD1;
    PhyRegMap[PHY_REG_TSTREAD2] = PHY_TSTREAD2;
    PhyRegMap[PHY_REG_TSTWRITE] = PHY_TSTWRITE;
    PhyRegMap[PHY_REG_SPECIAL_CTRL_STS] = PHY_SPECIAL_CTRL_STS;
    PhyRegMap[PHY_REG_SITC] = PHY_SITC;
    PhyRegMap[PHY_REG_INT_SRC] = PHY_INT_SRC;
    PhyRegMap[PHY_REG_INT_MASK] = PHY_INT_MASK;
    PhyRegMap[PHY_REG_SPECIAL] = PHY_SPECIAL;

	sprintf(version,"%lX.%02lX.%02lX",
		(DRIVER_VERSION>>16),(DRIVER_VERSION>>8)&0xFF,(DRIVER_VERSION&0xFFUL));
	SMSC_TRACE(DBG_INIT,"Driver smsc9500.ko verison %s, built on %s, %s",version, __TIME__, __DATE__);

	ret=smscusbnet_get_endpoints(dev,intf);
	if (ret<0)
		goto out1;

	dev->data[0]=(unsigned long) kmalloc(sizeof(ADAPTER_DATA),GFP_KERNEL);

	if((PADAPTER_DATA)dev->data[0]==NULL) {
		SMSC_WARNING("Unable to allocate ADAPTER_DATA");
		ret=-ENOMEM;
		goto out1;
	}
	memset((PADAPTER_DATA)dev->data[0],0,sizeof(ADAPTER_DATA));
	adapterData=(PADAPTER_DATA)(dev->data[0]);

    init_MUTEX(&adapterData->phy_mutex);
    init_MUTEX(&adapterData->eeprom_mutex);
    init_MUTEX(&adapterData->internal_ram_mutex);
    init_MUTEX(&adapterData->RxFilterLock);

	if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwBuf)< 0)) {
		SMSC_WARNING("Failed to read HW_CFG: %d", ret);
		return ret;
	}
	if(dwBuf & HW_CFG_SMDET_STS){
		SMSC_TRACE(DBG_INIT,"Come back from smart detach");
	}

	adapterData->macAddrHi16 = mac_addr_hi16;
	adapterData->MmacAddrLo32 = mac_addr_lo32;

	adapterData->eepromSize = MAX_EEPROM_SIZE + 128; //Set a initial value
	adapterData->eepromSize = smsc9500_eeprom_size(dev);
	SMSC_TRACE(DBG_INIT,"EEPROM size: %d bytes", adapterData->eepromSize);

	//Init all registers
	ret = smsc9500_reset(dev);
    if(ret < 0)goto out1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
	dev->net->do_ioctl = Smsc9500_do_ioctl;
	dev->net->set_multicast_list = smsc9500_set_multicast;
#else
        dev->net->netdev_ops = &smsc95xx_netdev_ops;
#endif  //2.6.29
	dev->net->ethtool_ops = &smsc9500_ethtool_ops;
	dev->net->flags|=IFF_MULTICAST;

	dev->linkDownSuspend = linkdownsuspend;
	dev->dynamicSuspend = dynamicsuspend;
#ifndef CONFIG_PM
	if(dev->dynamicSuspend || dev->linkDownSuspend){
		SMSC_WARNING("Power management has to be enabled in the kernel configuration to support dynamicsuspend and linkdownsuspend");
		dev->dynamicSuspend = dev->linkDownSuspend = 0;
	}
#endif //CONFIG_PM
#ifndef CONFIG_USB_SUSPEND
	if(dev->dynamicSuspend || dev->linkDownSuspend){
		SMSC_WARNING("Usb suspend has to be enabled in the kernel configuration to support dynamicsuspend and linkdownsuspend");
		dev->dynamicSuspend = dev->linkDownSuspend = 0;
	}
#endif //CONFIG_USB_SUSPEND

	if(dev->chipDependFeatures[FEATURE_SMARTDETACH]){
		dev->smartDetach = smartdetach;
		//If smart detach is enabled, link down suspend should be disabled
		if(dev->smartDetach)dev->linkDownSuspend = 0;
	}
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_PM
	if(dev->dynamicSuspend || dev->linkDownSuspend){
		if(dev->udev->autosuspend_disabled){
			SMSC_WARNING("Autosuspend should be enabled by shell cmd \"echo auto > /sys/bus/usb/devices/X-XX/power/level\"");
		}
	}
#endif //CONFIG_PM
#endif
	adapterData->UseScatterGather=scatter_gather;
	adapterData->UseTxCsum=tx_Csum;
	adapterData->UseRxCsum=rx_Csum;
	if (scatter_gather)
		SMSC_TRACE(DBG_INIT,"Tx Scatter-Gather");
	if (tx_Csum)
		SMSC_TRACE(DBG_INIT,"Tx HW Checksum");
	if (rx_Csum)
		SMSC_TRACE(DBG_INIT,"Rx HW Checksum");


	if(adapterData->UseScatterGather) {

		if(adapterData->UseTxCsum)
		dev->net->features = (NETIF_F_HW_CSUM | NETIF_F_SG | NETIF_F_FRAGLIST);
		else
		dev->net->features = (NETIF_F_SG | NETIF_F_FRAGLIST);	// Kernel will turn off SG in this case.
	}

	else {

		if(adapterData->UseTxCsum)
		dev->net->features = (NETIF_F_HW_CSUM);
		else
		dev->net->features = 0;
	}

	adapterData->dwTxQueueDisableMask=0;
	spin_lock_init(&(adapterData->TxQueueLock));
	adapterData->TxInitialized=TRUE;

	adapterData->WolWakeupOpts= 0;

	adapterData->LinkActLedCfg = LinkActLedCfg;
	adapterData->LinkLedOnGpio = LinkLedOnGpio;
	adapterData->LinkLedOnGpioBufType = LinkLedBufType;
	adapterData->LinkLedOnGpioPolarity = LinkLedPolarity;

	adapterData->LanInitialized=TRUE;

	SMSC_TRACE(DBG_INIT,"<--------out of bind, return 0\n");
	return 0;

	if (adapterData != NULL){
		kfree(adapterData);
		adapterData=NULL;
	}
out1:
	SMSC_TRACE(DBG_INIT,"<--------bind out1, return %d\n",ret);
	return ret;
}


static void smsc9500_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	SMSC_TRACE(DBG_CLOSE,"------->in smsc9500_unbind\n");

	if (adapterData != NULL){
		SMSC_TRACE(DBG_CLOSE,"free adapterData\n");
		kfree(adapterData);
		adapterData=NULL;
	}

	SMSC_TRACE(DBG_CLOSE,"<-------out of smsc9500_unbind\n");
}

static int smsc9500_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8  *head;
	u16 size;
	u32  header,AlignCount=0;
	char *packet;
	struct sk_buff *ax_skb;
	int ret = RX_FIXUP_VALID_SKB;
	u16 *vlan_tag;

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_TRACE(DBG_RX,"------->in smsc9500_rx_fixup\n");

	head = (u8 *) skb->data;
	memcpy(&header, head, sizeof(header));
	le32_to_cpus(&header);
#ifdef RX_OFFSET
	skb_pull(skb, 4 + NET_IP_ALIGN); //two extra for ip header alignment
	packet = skb->data;
#else
	packet = head + sizeof(header);
	skb_pull(skb, 4);
#endif //RX_OFFSET

	while (skb->len > 0) {
		/* get the packet length */
		size = (u16) ((header & RX_STS_FL_)>>16);

#ifdef RX_OFFSET
		AlignCount = (STATUS_WORD_LEN - ((size + NET_IP_ALIGN) % STATUS_WORD_LEN)) % STATUS_WORD_LEN;
#else
		AlignCount = (STATUS_WORD_LEN - (size % STATUS_WORD_LEN)) % STATUS_WORD_LEN;
#endif

		if(header & RX_STS_ES_){
			dev->stats.rx_errors++;
			dev->stats.rx_dropped++;
			if(header & RX_STS_CRC_){
				dev->stats.rx_crc_errors++;
			}else{
				if(header & (RX_STS_TL_ | RX_STS_RF_)){
					dev->stats.rx_frame_errors++;
				}
				if(((header & RX_STS_LE_) != 0L) && ((header & RX_STS_FT_)==0L)){
					dev->stats.rx_length_errors++;
				}
			}

			if(size == skb->len){//last packet
				return RX_FIXUP_INVALID_SKB;
			}else{

				skb_pull(skb, size+AlignCount);

				if (skb->len == 0) {
					ret = RX_FIXUP_INVALID_SKB;

					return ret;
				}

				goto NEXT_PACKET;
			}

		}

		if ((size == skb->len)){

			if (adapterData->UseRxCsum) {

			u16 wHwCsum;
#ifdef NET_SKBUFF_DATA_USES_OFFSET
            wHwCsum = *(u16*)(skb_tail_pointer(skb) - 2);
#else
            wHwCsum = *(u16*)(skb->tail - 2);
#endif
			skb->csum = wHwCsum;

			}


			if (adapterData->UseRxCsum)

				#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
					skb->ip_summed = CHECKSUM_HW;
				#else
					skb->ip_summed = CHECKSUM_COMPLETE;
				#endif

			else
				skb->ip_summed = CHECKSUM_NONE;

			if (adapterData->UseRxCsum)
			{
				skb_trim(skb,size-2-4);

			}
			else
			{
				skb_trim(skb,size-4);

			}

#ifdef RX_SKB_COPY
//FIXME: Kernel calculate received size based on skb->truesize, which holds total buffer size.
//If we allocate a big skb buffer, but only part of buffer hold valid data like turbo mode did,
//Kernel accumulate received data with skb->truesize, so total received data might be over limit. But
//actual data size isn't, then kernel may drop the subsequent packets.
			if(TurboMode){
				ax_skb = alloc_skb (skb->len + NET_IP_ALIGN, GFP_ATOMIC);
	            skb_reserve (ax_skb, NET_IP_ALIGN);
	            skb_put(ax_skb, skb->len);
	            memcpy(ax_skb->data, skb->data, skb->len);

				vlan_tag = (u16*)&ax_skb->cb[0];
				*vlan_tag = VLAN_DUMMY; //Reserved value
	            smscusbnet_skb_return(dev, ax_skb);
	            ret = RX_FIXUP_INVALID_SKB;
			}else{
				ret = RX_FIXUP_VALID_SKB;
				vlan_tag = (u16*)&skb->cb[0];
				*vlan_tag = VLAN_DUMMY; //Reserved value
			}
#else
//FIXME: We are not supposed to change truesize, but this is the easy way to cheat kernel without memory copy
			skb->truesize = skb->len + sizeof(struct sk_buff);
			vlan_tag = (u16*)&skb->cb[0];
			*vlan_tag = VLAN_DUMMY; //Reserved value
#endif
			return ret;
		}


		if (size > (ETH_FRAME_LEN+12)) {    			//  ETH_FRAME_LEN+4(CRC)+2(COE)+4(Vlan)
			SMSC_TRACE(DBG_RX,"size > (ETH_FRAME_LEN+12), hearder=  0x%08x\n", header);
			return RX_FIXUP_ERROR;
		}

#ifndef RX_SKB_COPY
               ax_skb = skb_clone(skb, GFP_ATOMIC);
#else
			ax_skb = alloc_skb (size + NET_IP_ALIGN, GFP_ATOMIC);
            skb_reserve (ax_skb, NET_IP_ALIGN);
#endif
     if (ax_skb) {
#ifndef RX_SKB_COPY
            ax_skb->len = size;
            ax_skb->data = packet;

			skb_trim(ax_skb, 0);
			skb_put(ax_skb, size);

#else
                        skb_put(ax_skb, size);
                        memcpy(ax_skb->data, packet, size);
#endif

			if (adapterData->UseRxCsum) {

			u16 wHwCsum;
#ifdef NET_SKBUFF_DATA_USES_OFFSET
            wHwCsum = *(u16*)(skb_tail_pointer(ax_skb) - 2);
#else
            wHwCsum = *(u16*)(ax_skb->tail - 2);
#endif
			ax_skb->csum = wHwCsum;

			}


			if (adapterData->UseRxCsum)

				#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
					ax_skb->ip_summed = CHECKSUM_HW;
				#else
					ax_skb->ip_summed = CHECKSUM_COMPLETE;
				#endif

			else
				ax_skb->ip_summed = CHECKSUM_NONE;

			if (adapterData->UseRxCsum)
			{
				skb_trim(ax_skb,size-2-4);

			}
			else
			{
				skb_trim(ax_skb,size-4);

			}

#ifndef RX_SKB_COPY
//FIXME: We are not supposed to change truesize, but this is the easy way to cheat kernel without memory copy
			ax_skb->truesize = ax_skb->len + sizeof(struct sk_buff);
#endif

			vlan_tag = (u16*)&ax_skb->cb[0];
			*vlan_tag = VLAN_DUMMY; //Reserved value
			smscusbnet_skb_return(dev, ax_skb);
		} else {
			SMSC_TRACE(DBG_RX,"no ax_skb\n");
			return RX_FIXUP_ERROR;
		}

		skb_pull(skb, size+AlignCount);

		if (skb->len == 0) {
			SMSC_TRACE(DBG_RX,"skb->len==0 left\n");
			break;
		}
NEXT_PACKET:
		head = (u8 *) skb->data;
		memcpy(&header, head, sizeof(header));
		le32_to_cpus(&header);
#ifdef RX_OFFSET
		skb_pull(skb, 4 + NET_IP_ALIGN); //two extra for ip header alignment
		packet = skb->data;
#else
		packet = head + sizeof(header);
		skb_pull(skb, 4);
#endif	//RX_OFFSET
	}

	if (skb->len < 0) {
		SMSC_WARNING("invalid rx length<0 %d", skb->len);
		return RX_FIXUP_ERROR;
	}


	SMSC_TRACE(DBG_RX,"<-------out of smsc9500_rx_fixup\n");
	return ret;
}

static struct sk_buff *smsc9500_tx_fixup(struct usbnet *dev, struct sk_buff *skb,
					int flags)
{

#ifndef TX_SKB_FORCE_COPY
	int headroom = skb_headroom(skb);
	int tailroom = skb_tailroom(skb);
#endif /* TX_SKB_FORCE_COPY */
	int SkbSize,CopySize,AlignmentSize;
	u8 * prt;
	int i;

	unsigned skbFragCnt = skb_shinfo(skb)->nr_frags + 1;
	u32 TxCommandA,TxCommandB;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_TRACE(DBG_TX,"in smsc9500_tx_fixup\n");

	if (adapterData->UseTxCsum) {


		u32 dwTxCsumPreamble=0;

		#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))

	 		if (skb->ip_summed == CHECKSUM_HW)
			{
			int Chsum_start_offset=0;
			CalculateTxChecksumOffset(
				skb,
				&Chsum_start_offset);

			/* tx checksum problem workaround */
			if (skb->len <= 45) {
				u32 csum;
				csum = csum_partial(skb->data + Chsum_start_offset, skb->len - (skb->data + Chsum_start_offset - skb->data), 0);
				*((u16 *)(skb->data + Chsum_start_offset + skb->csum)) = csum_fold(csum);
				goto Non_CheckSumOffLoad;
			}

			 dwTxCsumPreamble=(((u16) (Chsum_start_offset + skb->csum)) << 16) | ((u16) Chsum_start_offset);

			}
		#else

			if (skb->ip_summed == CHECKSUM_PARTIAL)
			{

			#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
			int Chsum_start_offset=0;
			CalculateTxChecksumOffset(
				skb,
				&Chsum_start_offset);

			/* tx checksum problem workaround */
			if (skb->len <= 45) {
				u32 csum;
				csum = csum_partial(skb->data + Chsum_start_offset, skb->len - (skb->data + Chsum_start_offset - skb->data), 0);
				*((u16 *)(skb->data + Chsum_start_offset + skb->csum)) = csum_fold(csum);
				goto Non_CheckSumOffLoad;
			}

			dwTxCsumPreamble=(((u16) (Chsum_start_offset + skb->csum)) << 16) | ((u16) Chsum_start_offset);

			#else

			/* tx checksum problem workaround */
			if (skb->len <= 45) {
				u32 csum;
				csum = csum_partial(skb->head + skb->csum_start, skb->len - (skb->head + skb->csum_start - skb->data), 0);
				*((u16 *)(skb->head + (skb->csum_start + skb->csum_offset))) = csum_fold(csum);
				goto Non_CheckSumOffLoad;
			}

			dwTxCsumPreamble=(((u16) (skb->csum_offset+skb->csum_start-(skb->data - skb->head))) << 16)
					 | ((u16) (skb->csum_start-(skb->data - skb->head)));

			#endif


			}
		#endif
			#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
				if(skb->ip_summed == CHECKSUM_HW)
			#else
				if(skb->ip_summed == CHECKSUM_PARTIAL)
			#endif
			{
				if (skbFragCnt ==1) {
// ip_summed, one Fragament
					SMSC_TRACE(DBG_TX,"ip_summed, Onefrag\n");

					AlignmentSize = skb->len % STATUS_WORD_LEN;
					if(AlignmentSize)AlignmentSize = STATUS_WORD_LEN - AlignmentSize;

#ifndef TX_SKB_FORCE_COPY
					if ((!skb_cloned(skb))
	   					 && ((headroom + tailroom) >= (12 + AlignmentSize))) {
						if( (headroom < 12 ) || (tailroom < AlignmentSize) ){
							skb->data = memmove(skb->head +12, skb->data, skb->len);
							SkbSize = skb->len;
							skb_trim(skb, 0);
							skb_put(skb, SkbSize);


						}
					} else
#endif /* TX_SKB_FORCE_COPY */
					{
						struct sk_buff *skb2;
						skb2 = skb_copy_expand(skb, 12, AlignmentSize, flags);
						dev_kfree_skb_any(skb);
						skb = skb2;
						if (!skb)
							return NULL;
					}

	        		skb_push(skb, STATUS_WORD_LEN);
	        		memcpy(skb->data, &dwTxCsumPreamble, STATUS_WORD_LEN);

					skb_push(skb, STATUS_WORD_LEN);
					TxCommandB=(u32)(skb->len-STATUS_WORD_LEN)|TX_CMD_B_CSUM_ENABLE;
					cpu_to_le32s((u32*)&TxCommandB);
					memcpy(skb->data, &TxCommandB, STATUS_WORD_LEN);
					skb_push(skb, STATUS_WORD_LEN);
					TxCommandA= TX_CMD_A_FIRST_SEG_ | TX_CMD_A_LAST_SEG_ | (u32)(skb->len-8);
					cpu_to_le32s((u32*)&TxCommandA);
					memcpy(skb->data, &TxCommandA, STATUS_WORD_LEN);
					skb_put(skb, AlignmentSize);
					return skb;
						}
				else {
// ip_summed, Multi Fragament

					struct sk_buff *skb2;
					SkbSize=skb->len+4*4*(skbFragCnt+1);
					skb2 = dev_alloc_skb(SkbSize);

					SMSC_TRACE(DBG_TX,"ip_summed, Multifrags\n");
					if (!skb2)
						return NULL;

					skb_put(skb2, SkbSize);


					{

						TxCommandA=TX_CMD_A_FIRST_SEG_ |((u32)sizeof(u32));

						TxCommandB=TX_CMD_B_CSUM_ENABLE |((u32)(skb->len+STATUS_WORD_LEN)) ;
						cpu_to_le32s((u32*)&TxCommandA);
						cpu_to_le32s((u32*)&TxCommandB);

						memcpy(skb2->data, &TxCommandA, STATUS_WORD_LEN);
						memcpy(skb2->data+STATUS_WORD_LEN, &TxCommandB, STATUS_WORD_LEN);
	         				memcpy(skb2->data+8, &dwTxCsumPreamble, STATUS_WORD_LEN);
	      				 }

					{


						TxCommandA =
							((((unsigned long)(skb->data))&0x03UL)<<16) | //u32 alignment adjustment
					  		  ((u32)((skb->len)-(skb->data_len)));
						TxCommandB=
							((u32)(skb->len+STATUS_WORD_LEN));
						cpu_to_le32s((u32*)&TxCommandA);
						cpu_to_le32s((u32*)&TxCommandB);
						CopySize=((((u32)(skb->len - skb->data_len))+3+(((unsigned long)(skb->data))&0x03UL))>>2)*4;
						memcpy(skb2->data + 12, &TxCommandA, STATUS_WORD_LEN);
						memcpy(skb2->data + 12 + STATUS_WORD_LEN, &TxCommandB, STATUS_WORD_LEN);
						memcpy(skb2->data + 12 + STATUS_WORD_LEN * 2, (u32 *)(((unsigned long)(skb->data))&0xFFFFFFFCUL),CopySize);
					}

					prt=(u8 *)skb2->data+20+CopySize;

					for(i=1;i<skbFragCnt;i++)
					{
						skb_frag_t *frag = &skb_shinfo(skb)->frags[i - 1];
						void *frag_addr = page_address(frag->page) + frag->page_offset;


						TxCommandA=
							((((unsigned long)(frag_addr))&0x03UL)<<16) | //alignment adjustment
				 			 ((u32)(frag->size));


						if (i==(skbFragCnt-1)){
							TxCommandA |= TX_CMD_A_LAST_SEG_ ;
						}


						TxCommandB= ((u32)(skb->len+STATUS_WORD_LEN));
						cpu_to_le32s((u32*)&TxCommandA);
						cpu_to_le32s((u32*)&TxCommandB);
						memcpy(prt, &TxCommandA, STATUS_WORD_LEN);
						prt=prt+STATUS_WORD_LEN;
						memcpy(prt, &TxCommandB, STATUS_WORD_LEN);
						prt=prt+STATUS_WORD_LEN;
						CopySize=((((unsigned long)(frag->size))+3+(((unsigned long)(frag_addr))&0x03UL))>>2)*4;
						memcpy(prt, (u32 *)(((unsigned long)(frag_addr))&0xFFFFFFFCUL),CopySize);
						prt=prt+CopySize;

					}

						skb_trim(skb2,prt-skb2->data);
						dev_kfree_skb_any(skb);
						return skb2;
					}

			}
			else {
				if (skbFragCnt >1) {
//Non ip_summed, Multifrags

					struct sk_buff *skb2;
					SkbSize=skb->len+4*4*skbFragCnt;
					skb2 = dev_alloc_skb(SkbSize);
					SMSC_TRACE(DBG_TX,"Non ip_summed, Multifrags\n");

					if (!skb2)
						return NULL;

					skb_put(skb2, SkbSize);

					{

						TxCommandA =((((unsigned long)(skb->data))&0x03UL)<<16) | //u32 alignment adjustment
										TX_CMD_A_FIRST_SEG_ |
					  		  			(u32)((skb->len)-(skb->data_len));

						TxCommandB=TX_CMD_B_CSUM_ENABLE |((u32)(skb->len));
						cpu_to_le32s((u32*)&TxCommandA);
						cpu_to_le32s((u32*)&TxCommandB);
						SMSC_TRACE(DBG_TX,"first frag. \n");
						CopySize=((((unsigned long)((skb->len)-(skb->data_len)))+3+(((unsigned long)(skb->data))&0x03UL))>>2)*4;
						memcpy(skb2->data, &TxCommandA, STATUS_WORD_LEN);
						memcpy(skb2->data+STATUS_WORD_LEN, &TxCommandB, STATUS_WORD_LEN);
						memcpy(skb2->data+STATUS_WORD_LEN*2, (void *)(((unsigned long)(skb->data))&0xFFFFFFFCUL),CopySize);
					}

					prt=(u8 *)skb2->data+8+CopySize;

					for(i=1;i<skbFragCnt;i++)
					{
						skb_frag_t *frag = &skb_shinfo(skb)->frags[i - 1];
						void *frag_addr = page_address(frag->page) + frag->page_offset;


						TxCommandA=
							((((unsigned long)(frag_addr))&0x03UL)<<16) | //u32 alignment adjustment
				 			 ((u32)(frag->size));


						if (i==(skbFragCnt-1)){
							TxCommandA |= TX_CMD_A_LAST_SEG_ ;
						}


						TxCommandB=((u32)(skb->len));
						cpu_to_le32s((u32*)&TxCommandA);
						cpu_to_le32s((u32*)&TxCommandB);
						memcpy(prt, &TxCommandA, STATUS_WORD_LEN);
						prt=prt+STATUS_WORD_LEN;
						memcpy(prt, &TxCommandB, STATUS_WORD_LEN);
						prt=prt+STATUS_WORD_LEN;

						CopySize=((((unsigned long)(frag->size))+3+(((unsigned long)(frag_addr))&0x03UL))>>2)*4;
						memcpy(prt, (void *)(((unsigned long)(frag_addr))&0xFFFFFFFCUL),CopySize);
						prt=prt+CopySize;

						}

						skb_trim(skb2,prt-skb2->data);
						dev_kfree_skb_any(skb);
						SMSC_TRACE(DBG_TX,"return from Nonip_summed, Multifrags\n");
						return skb2;
				} else {
				goto Non_CheckSumOffLoad;
				}
			}

		}
	else {
Non_CheckSumOffLoad:
//Non ip_summed, onefrag
	SMSC_TRACE(DBG_TX,"Non ip_summed, onefrag\n");
	AlignmentSize = skb->len % STATUS_WORD_LEN;
	if(AlignmentSize)AlignmentSize = STATUS_WORD_LEN - AlignmentSize;

#ifndef TX_SKB_FORCE_COPY
	if ((!skb_cloned(skb))
	    && ((headroom + tailroom) >= (2*STATUS_WORD_LEN + AlignmentSize))) {
		if ((headroom < (2*STATUS_WORD_LEN)) || (tailroom < AlignmentSize)){
			skb->data = memmove(skb->head + 2*STATUS_WORD_LEN, skb->data, skb->len);
			SkbSize = skb->len;
			skb_trim(skb, 0);
			skb_put(skb, SkbSize);
		}
	} else
#endif /* TX_SKB_FORCE_COPY */
	{
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, 2*STATUS_WORD_LEN, AlignmentSize, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	skb_push(skb, STATUS_WORD_LEN);
	TxCommandB=(u32)(skb->len - STATUS_WORD_LEN);
	cpu_to_le32s((u32*)&TxCommandB);

	memcpy(skb->data, &TxCommandB, STATUS_WORD_LEN);
	skb_push(skb, STATUS_WORD_LEN);
	TxCommandA= TX_CMD_A_FIRST_SEG_ | TX_CMD_A_LAST_SEG_ | (u32)(skb->len - 2*STATUS_WORD_LEN);
	cpu_to_le32s((u32*)&TxCommandA);
	memcpy(skb->data, &TxCommandA, STATUS_WORD_LEN);

	skb_put(skb, AlignmentSize);
	return skb;
	}

}

static int smsc9500_reset(struct usbnet *dev)
{
	int ret=0,Timeout;
	u32 dwReadBuf, dwAddrH, dwAddrL, dwWriteBuf,dwMacCr,DwTemp, dwBurstCap;
	SMSC9500_RX_STATS rx_stats;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_TRACE(DBG_INIT,"---------->smsc9500_reset\n");

	if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read HW_CFG: %d", ret);
		return ret;
	}
	dwReadBuf |= HW_CFG_LRST_;
	if ((ret = smsc9500_write_reg(dev, HW_CFG, dwReadBuf))<0)
		{
		SMSC_WARNING("Failed to write HW_CFG_LRST_ bit in HW_CFG register, ret = %d \n",ret);
		return ret;
		}

	Timeout = 0;
	do {
		if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read HW_CFG: %d", ret);
			return ret;
		}
		msleep(100);         /* wait for 100 us before trying again */
		Timeout++;
	} while ( (dwReadBuf & HW_CFG_LRST_) && (Timeout < 100));

	if(Timeout >= 100)
	{
		SMSC_WARNING("Timeout waiting for completion of Lite Reset\n");
		return ret;
	}

	if((ret = smsc9500_read_reg(dev, ID_REV, &dev->chipID)) < 0){
		SMSC_WARNING("Failed to read GPIO_CFG: %d", ret);
		return ret;
	  }
	dev->chipID = dev->chipID >> 16;

	/*******Enable new features**************/
	if(dev->chipID == ID_REV_9500A_CHIPID){
		int i;
		for(i=0; i<FEATURE_MAX_NO; i++){
			dev->chipDependFeatures[i] = TRUE;
		}
	}
    if(dev->chipID == ID_REV_9512_CHIPID)dev->chipDependFeatures[FEATURE_WUFF_8] = TRUE;
    /*******Enable new features**************/

    if ((ret = smsc9500_read_reg(dev,PM_CTRL,&DwTemp)< 0)) {
            SMSC_WARNING("Failed to read PM_CTRL: %d", ret);
            return ret;
     }

	if ((ret = smsc9500_write_reg(dev, PM_CTRL, (DwTemp | PM_CTL_PHY_RST_))< 0)) {
		SMSC_WARNING("Failed to write PM_CTRL: %d", ret);
		return ret;
	}

	Timeout = 0;
	do {
		if ((ret = smsc9500_read_reg(dev,PM_CTRL,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read PM_CTRL: %d", ret);
			return ret;
		}
		msleep(100);
		Timeout++;
	} while ( (dwReadBuf & PM_CTL_PHY_RST_) && (Timeout < 100));

	if(Timeout >= 100)
	{
		SMSC_WARNING("Timeout waiting for PHY Reset\n");
		return ret;
	}
	dwAddrH = 0x0000FFFFUL;
	dwAddrL = 0xFFFFFFFF;

	if(adapterData->macAddrHi16 != 0xFFFFFFFF || adapterData->MmacAddrLo32 != 0xFFFFFFFF){
		dwAddrH = adapterData->macAddrHi16 & 0xFFFF;
		dwAddrL = adapterData->MmacAddrLo32;
	}else{
		if(adapterData->eepromSize && smsc9500_read_eeprom(dev, EEPROM_MAC_OFFSET, 6,  dev->net->dev_addr) == 0){
			dwAddrL = dev->net->dev_addr[0] | dev->net->dev_addr[1] << 8 | dev->net->dev_addr[2] << 16 | dev->net->dev_addr[3] << 24;
			dwAddrH = dev->net->dev_addr[4] | dev->net->dev_addr[5] << 8;
		}else{//LAN9500's descriptor RAM may provide Mac address
			MAC_ADDR_IN_RAM macRam;
			if(ReadDataPort(dev, RAMSEL_EEPROM, 0, sizeof(MAC_ADDR_IN_RAM)/4, (u32*)&macRam, NULL) == SMSC9500_SUCCESS){
				cpu_to_le32s(&macRam.signature);
				cpu_to_le32s(&macRam.MacAddrL);
				cpu_to_le32s(&macRam.MacAddrH);
				cpu_to_le16s(&macRam.crc);
				cpu_to_le16s(&macRam.crcComplement);
				if(macRam.signature == 0x736D7363){//Signature "smsc"
					u16 crc = CalculateCrc16((char*)&macRam, 12, FALSE);
					if((crc == macRam.crc) && (crc == (u16)~macRam.crcComplement)){
						dwAddrL = macRam.MacAddrL;
						dwAddrH = macRam.MacAddrH;
					}
				}
			}
		}
	}

	//Mac address could be initialized by system firmware. Lan9500A will implement this way.
	if((dwAddrH==0x0000FFFFUL)&&(dwAddrL==0xFFFFFFFF)){
		if ((ret = smsc9500_read_reg(dev,ADDRL, &dwAddrL)< 0)) {
			SMSC_WARNING("Failed to read ADDRL: %d", ret);
			return ret;
		}
		if ((ret = smsc9500_read_reg(dev,ADDRH, &dwAddrH)< 0)) {
			SMSC_WARNING("Failed to read ADDRH: %d", ret);
			return ret;
		}
	}

	if(((dwAddrH & 0xFFFF) == 0x0000FFFFUL) && (dwAddrL == 0xFFFFFFFF))
	{
		dwAddrH=0x00000070UL;
		dwAddrL=0x110F8000UL;

		SMSC_TRACE(DBG_INIT,"Mac Address is set by default to 0x%04X%08X\n",
			dwAddrH,dwAddrL);
	}
	adapterData->macAddrHi16 = dwAddrH;
	adapterData->MmacAddrLo32 = dwAddrL;

	if ((ret = smsc9500_write_reg(dev,ADDRL, dwAddrL)< 0)) {
		SMSC_WARNING("Failed to write ADDRL: %d", ret);
		return ret;
	}
	if ((ret = smsc9500_write_reg(dev,ADDRH, dwAddrH)< 0)) {
		SMSC_WARNING("Failed to write ADDRH: %d", ret);
		return ret;
	}

	dev->net->dev_addr[0]=LOBYTE(LOWORD(dwAddrL));
	dev->net->dev_addr[1]=HIBYTE(LOWORD(dwAddrL));
	dev->net->dev_addr[2]=LOBYTE(HIWORD(dwAddrL));
	dev->net->dev_addr[3]=HIBYTE(HIWORD(dwAddrL));
	dev->net->dev_addr[4]=LOBYTE(LOWORD(dwAddrH));
	dev->net->dev_addr[5]=HIBYTE(LOWORD(dwAddrH));

	SMSC_TRACE(DBG_INIT,"dev->net->dev_addr %02x:%02x:%02x:%02x:%02x:%02x\n",
			dev->net->dev_addr [0], dev->net->dev_addr [1],
			dev->net->dev_addr [2], dev->net->dev_addr [3],
			dev->net->dev_addr [4], dev->net->dev_addr [5]);

	if (!(smscusbnet_IsOperationalMode(dev))) {

		if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read HW_CFG: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"Read Value from HW_CFG : 0x%08x\n",dwReadBuf);

		dwReadBuf |=HW_CFG_BIR_;
		if ((ret = smsc9500_write_reg(dev, HW_CFG, dwReadBuf))<0)
		{
			SMSC_WARNING("Failed to write HW_CFG_BIR_ bit in HW_CFG register, ret = %d ",ret);
			return ret;
		}

		if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read HW_CFG: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"Read Value from HW_CFG after writing HW_CFG_BIR_: 0x%08x\n",dwReadBuf);
	}

	if (TurboMode) {
		if(dev->udev->speed == USB_SPEED_HIGH){
			dev->rx_urb_size = DEFAULT_HS_BURST_CAP_SIZE;
			dwBurstCap = DEFAULT_HS_BURST_CAP_SIZE / HS_USB_PKT_SIZE;
		}else{
			dev->rx_urb_size = DEFAULT_FS_BURST_CAP_SIZE;
			dwBurstCap = DEFAULT_FS_BURST_CAP_SIZE / FS_USB_PKT_SIZE;
		}
	}else{
		dwBurstCap = 0;
		dev->rx_urb_size = MAX_SINGLE_PACKET_SIZE;
	}
	SMSC_TRACE(DBG_INIT,"rx_urb_size= %d\n", (int)dev->rx_urb_size);

	if ((ret = smsc9500_write_reg(dev, BURST_CAP, dwBurstCap))<0)
	{
		SMSC_WARNING("Failed to write BURST_CAP");
		return ret;
	}
	if ((ret = smsc9500_read_reg(dev,BURST_CAP,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read BURST_CAP: %d", ret);
		return ret;
	}
	SMSC_TRACE(DBG_INIT,"Read Value from BURST_CAP after writing: 0x%08x\n",dwReadBuf);

	if ((ret = smsc9500_write_reg(dev, BULK_IN_DLY, bulkin_delay))<0)
	{
		SMSC_WARNING("Failed to write BULK_IN_DLY");
		return ret;
	}
	if ((ret = smsc9500_read_reg(dev,BULK_IN_DLY,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read BULK_IN_DLY: %d", ret);
		return ret;
	}
	SMSC_TRACE(DBG_INIT,"Read Value from BULK_IN_DLY after writing: 0x%08x\n",dwReadBuf);

	if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read HW_CFG: %d", ret);
		return ret;
	}
	SMSC_TRACE(DBG_INIT,"Read Value from HW_CFG: 0x%08x\n",dwReadBuf);

	if (TurboMode) {
		dwReadBuf |= (HW_CFG_MEF_|HW_CFG_BCE_);
	}
#ifdef RX_OFFSET
	dwReadBuf &= ~HW_CFG_RXDOFF_;
	dwReadBuf |= NET_IP_ALIGN  << 9; //set Rx data offset=2, Make IP header aligns on word boundary.
#endif
	if ((ret = smsc9500_write_reg(dev, HW_CFG, dwReadBuf))<0)
		{
		SMSC_WARNING("Failed to write HW_CFG_BIR_ bit in HW_CFG register, ret = %d \n",ret);
		return ret;
	}

	if ((ret = smsc9500_read_reg(dev,HW_CFG,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read HW_CFG: %d", ret);
		return ret;
	}
	SMSC_TRACE(DBG_INIT,"Read Value from HW_CFG after writing: 0x%08x\n",dwReadBuf);

	if ((ret = smsc9500_write_reg(dev, INT_STS, 0xFFFFFFFFUL))<0)
		{
		SMSC_WARNING("Failed to write INT_STS register, ret = %d \n",ret);
		return ret;
		}


	if ((ret = smsc9500_read_reg(dev,ID_REV,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read ID_REV: %d", ret);
		return ret;
	}
	adapterData->dwIdRev = dwReadBuf;
	SMSC_TRACE(DBG_INIT,"ID_REV = 0x%08x\n",dwReadBuf);


	if ((ret = smsc9500_read_reg(dev,FPGA_REV,&dwReadBuf)< 0)) {
		SMSC_WARNING("Failed to read FPGA_REV: %d", ret);
		return ret;
	}
	adapterData->dwFpgaRev = dwReadBuf;
	SMSC_TRACE(DBG_INIT,"FPGA_REV = 0x%08x\n",dwReadBuf);

	if (smscusbnet_IsOperationalMode(dev)) {

		if ((ret = smsc9500_read_reg(dev,INT_EP_CTL,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read INT_EP_CTL: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"Read Value from INT_EP_CTL: 0x%08x\n",dwReadBuf);

		dwReadBuf|=INT_EP_CTL_RX_FIFO_EN_;
		if ((ret = smsc9500_write_reg(dev, INT_EP_CTL, dwReadBuf))<0)
		{
			SMSC_WARNING("Failed to write INT_EP_CTL register,ret = %d \n",ret);
			return ret;
		}

		if ((ret = smsc9500_read_reg(dev,INT_EP_CTL,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read INT_EP_CTL: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"Read Value from INT_EP_CTLafter writing INT_EP_CTL_RX_FIFO_EN_: 0x%08x\n",dwReadBuf);
	}

//Init Tx

	if(adapterData->UseTxCsum)

	//Set TX COE
	{
		if ((ret = smsc9500_read_reg(dev,	COE_CR,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read COE_CR: %d", ret);
			return ret;
		}
		dwReadBuf |= Tx_COE_EN_;

		if ((ret = smsc9500_write_reg(dev,	COE_CR, dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to write COE_CR: %d", ret);
			return ret;
		}

		if ((ret = smsc9500_read_reg(dev,	COE_CR,&DwTemp)< 0)) {
			SMSC_WARNING("Failed to read COE_CR: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"COE_CR = 0x%08x\n", DwTemp);
	}

	if ((ret = smsc9500_write_reg(dev, FLOW, 0x0UL)< 0)) {
			SMSC_WARNING("Failed to write FLOW: %d", ret);
			return ret;
		}

	if ((ret = smsc9500_write_reg(dev,	AFC_CFG, AFC_CFG_DEFAULT)< 0)) {
			SMSC_WARNING("Failed to write AFC_CFG: %d", ret);
			return ret;
		}

	if ((ret = smsc9500_read_reg(dev, MAC_CR,&dwMacCr)< 0)) {
			SMSC_WARNING("Failed to read MAC_CR: %d", ret);
			return ret;
	}
	dwMacCr|=(MAC_CR_TXEN_);
	if ((ret = smsc9500_write_reg(dev, MAC_CR, dwMacCr)< 0)) {
			SMSC_WARNING("Failed to read MAC_CR: %d", ret);
			return ret;
	}
	dwWriteBuf=TX_CFG_ON_;
	if ((ret = smsc9500_write_reg(dev,TX_CFG, dwWriteBuf)< 0)) {
			SMSC_WARNING("Failed to write TX_CFG: %d", ret);
			return ret;
	}

//Init Rx

	//Set Vlan
	 {
	 	dwWriteBuf=(u32)ETH_P_8021Q;
		if ((ret = smsc9500_write_reg(dev,VLAN1, dwWriteBuf)< 0)) {
			SMSC_WARNING("Failed to write VAN1: %d", ret);
			return ret;
		}

	}

	//Set Rx COE
	if (adapterData->UseRxCsum) {

		if ((ret = smsc9500_read_reg(dev,	COE_CR,&dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to read COE_CR: %d", ret);
			return ret;
		}

		#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15))
			dwReadBuf|=(Rx_COE_EN_ |  Rx_COE_MODE_);
		#else
			dwReadBuf|=Rx_COE_EN_;
		#endif

		if ((ret = smsc9500_write_reg(dev,	COE_CR, dwReadBuf)< 0)) {
			SMSC_WARNING("Failed to write COE_CR: %d", ret);
			return ret;
		}

		if ((ret = smsc9500_read_reg(dev,	COE_CR,&DwTemp)< 0)) {
			SMSC_WARNING("Failed to read COE_CR: %d", ret);
			return ret;
		}
		SMSC_TRACE(DBG_INIT,"COE_CR = 0x%08x\n", DwTemp);

	}

	if ((ret = smsc9500_read_reg(dev, MAC_CR,&dwMacCr)< 0)) {
		SMSC_WARNING("Failed to read MAC_CR: %d", ret);
		return ret;
	}

	dwMacCr|=MAC_CR_RXEN_;

	if ((ret = smsc9500_write_reg(dev, MAC_CR, dwMacCr)< 0)) {
		SMSC_WARNING("Failed to read MAC_CR: %d", ret);
		return ret;
	}

	// Enable the LEDs by default
   if ((ret = smsc9500_read_reg(dev, LED_GPIO_CFG,&DwTemp)< 0)) {
		SMSC_WARNING("Failed to read LED_GPIO_CFG: %d", ret);
		return ret;
	}
   DwTemp &= ~LED_GPIO_CFG_GPCTL_10_ | ~LED_GPIO_CFG_GPCTL_09_ | ~LED_GPIO_CFG_GPCTL_08_;

   if((ret = smsc9500_write_reg( dev, LED_GPIO_CFG,
                        (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_10_SH) |
                        (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_09_SH) |
                        (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_08_SH))) < 0)
    {
		SMSC_WARNING("Failed to write LED_GPIO_CFG: %d", ret);
		return ret;
    }

   if (adapterData->LinkActLedCfg){ // Driver parameter enables separate Link and Activity LEDs in Thera 130
       // Make sure that the device is Thera 130
	   if ((ret = smsc9500_read_reg(dev, ID_REV,&DwTemp)< 0)) {
			SMSC_WARNING("Failed to read ID_REV: %d", ret);
			return ret;
		}
	   if(dev->chipDependFeatures[FEATURE_SEP_LEDS]){
           u32 dwLedGpioCfg = (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_10_SH) |
                                (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_09_SH) |
                                (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_08_SH);

           // Enable separate Link/Act LEDs
           dwLedGpioCfg |= LED_GPIO_CFG_LED_SEL_;

           // Set LED GPIO Config
           if((ret = smsc9500_write_reg(dev, LED_GPIO_CFG, dwLedGpioCfg)) < 0){
				SMSC_WARNING("Failed to write LED_GPIO_CFG: %d", ret);
				return ret;
           }
       }else{
           // Reset to default value
	   adapterData->LinkActLedCfg = 0;
           // Disable Software control LinkLed GPIO in case it is set
           adapterData->LinkLedOnGpio = 11;
       }
   }else if (adapterData->LinkLedOnGpio <= 10){ // Software control LinkLed GPIO is enable
        // If selected GPIO is GPIO0-GPIO7, make sure external PHY is not enable.
        // GPIO0-GPIO7 pins are multiplexed with MII signals.
        if (adapterData->LinkLedOnGpio < 8)
        {
            // Check PHY Strap
            if((ret = smsc9500_read_reg(dev, HW_CFG, &DwTemp)) < 0){
                        SMSC_WARNING("Failed to read HW_CFG: %d", ret);
                        return ret;
            }

            if (DwTemp & HW_CFG_PSEL_) // External PHY is enable
            {
                SMSC_WARNING("External PHY Enable::GPIO%d can not be set as Link Up/Down Led\n", adapterData->LinkLedOnGpio);
                // Disable Software control LinkLed GPIO
                adapterData->LinkLedOnGpio = 11;
            }
            else
            {
                u32 dwGpioCfg;// = ~GPIO_CFG_GPO0_EN_ | GPIO_CFG_GPO0_DIR_;

                if((ret = smsc9500_read_reg(dev, GPIO_CFG, &dwGpioCfg)) < 0){
                        SMSC_WARNING("Failed to read GPIO_CFG: %d", ret);
                        return ret;
                }

                dwGpioCfg &= ~(GPIO_CFG_GPO0_EN_ << adapterData->LinkLedOnGpio);
                dwGpioCfg |= GPIO_CFG_GPO0_DIR_ << adapterData->LinkLedOnGpio;

                // Check GPIO buffer type
                if (!adapterData->LinkLedOnGpioBufType) // Push-Pull output
                {
                    dwGpioCfg |= GPIO_CFG_GPO0_TYPE << adapterData->LinkLedOnGpio;
                }

                // Check GPIO Polarity
                if (adapterData->LinkLedOnGpioPolarity) // Active low
                {
                    dwGpioCfg |= GPIO_CFG_GPO0_DATA_ << adapterData->LinkLedOnGpio;
                }

                // Set GPIO Config register
                if((ret = smsc9500_write_reg(dev, GPIO_CFG, dwGpioCfg)) < 0){
			SMSC_WARNING("Failed to write GPIO_CFG: %d", ret);
			return ret;
                }
            }
        }
        else
        {
            u32 dwLedGpioCfg = (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_10_SH) |
								 (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_09_SH) |
								 (LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_08_SH);

            switch (adapterData->LinkLedOnGpio)
            {
                case 8:
                    // Enable GPIO 8
                    dwLedGpioCfg &= (~(LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_08_SH));
                    // Set Output Direction
                    dwLedGpioCfg |= LED_GPIO_CFG_GPDIR_08_;
                    // Set Buffer Type
                    if (!adapterData->LinkLedOnGpioBufType) // Push-Pull Output
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPBUF_08_;
                    }

                    // Check GPIO Polarity
                    if (adapterData->LinkLedOnGpioPolarity) // Active low
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPDATA_08_;
                    }
                    break;

                case 9:
                    // Enable GPIO 9
                    dwLedGpioCfg &= (~(LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_09_SH));
                    // Set Output Direction
                    dwLedGpioCfg |= LED_GPIO_CFG_GPDIR_09_;
                    // Set Buffer Type
                    if (!adapterData->LinkLedOnGpioBufType) // Push-Pull Output
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPBUF_09_;
                    }

                    // Check GPIO Polarity
                    if (adapterData->LinkLedOnGpioPolarity) // Active low
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPDATA_09_;
                    }
                    break;

                case 10:
                    // Enable GPIO 10
                    dwLedGpioCfg &= (~(LED_GPIO_CFG_GPCTL_LED_ << LED_GPIO_CFG_GPCTL_10_SH));
                    // Set Output Direction
                    dwLedGpioCfg |= LED_GPIO_CFG_GPDIR_10_;

                    // Set Buffer Type
                    if (!adapterData->LinkLedOnGpioBufType) // Push-Pull Output
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPBUF_10_;
                    }

                    // Check GPIO Polarity
                    if (adapterData->LinkLedOnGpioPolarity) // Active low
                    {
                        dwLedGpioCfg |= LED_GPIO_CFG_GPDATA_10_;
                    }
                    break;
            }

             // Set LED GPIO Config
            if((ret = smsc9500_write_reg( dev, LED_GPIO_CFG, dwLedGpioCfg)) < 0){
                        SMSC_WARNING("Failed to write LED_GPIO_CFG: %d", ret);
                        return ret;
            }
        }
   }

   if(dev->chipDependFeatures[FEATURE_NEWSTATIS_CNT]){//Statistics counters are rollover ones in LAN9500A
		if(smsc9500_get_stats(dev, (void*)&rx_stats) > 0){//Save initial value
			le32_to_cpus((u32*)&rx_stats.RxFifoDroppedFrames);
			rx_stats.RxFifoDroppedFrames &= 0xFFFFF; //This counter has 20 bits.
			dev->preRxFifoDroppedFrame = rx_stats.RxFifoDroppedFrames;
		}
	}

	smsc9500_rx_setmulticastlist(dev);

	if (!Phy_Initialize(dev, phy_addr, link_mode))
		return SMSC9500_FAIL;

	SMSC_TRACE(DBG_INIT,"<--------out of smsc9500_reset, return 0\n");

	return 0;
}

static int smsc9500_link_reset(struct usbnet *dev)
{
    int ret = 0;
	SMSC_TRACE(DBG_LINK,"---->in smsc9500_link_reset\n");
	ret = Phy_CheckLink(dev);

	if(dev->StopLinkPolling){
		clear_bit (EVENT_DEV_RECOVERY, &dev->flags);
	}

    if (test_bit (EVENT_DEV_RECOVERY, &dev->flags)) {
        ret = smsc9500_device_recovery(dev);
        clear_bit (EVENT_DEV_RECOVERY, &dev->flags);
    }

	SMSC_TRACE(DBG_LINK,"<----out of smsc9500_link_reset\n");
	return ret;
}


static int smsc9500_stopTxPath(struct usbnet * dev)
/*++

Routine Description:

    This routine Stops the Tx Path at both Mac and USB

Arguments:

Return Value:

--*/
{
	u32 Value32;
    int ret = SMSC9500_FAIL;
    int Count = 0;

   SMSC_TRACE(DBG_TX,"--> smsc9500_stopTxPath\n");

    // Stop the Transmit path at SCSRs
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, TX_CFG, TX_CFG_STOP_));
    // The bit should self clear as soon as the packet makes it out of the Mac,
    // worst case is 10 Mbps HD and frame deferred to the maximum. This will
    // be ~100ms tops. Assuming one register read per (micro)frame the case of
    // high speed USB - 125us register read cycle time - is the worse and
    // would need up to 800 reads. Let's just round up to 1000.
    do
    {
        CHECK_RETURN_STATUS(smsc9500_read_reg(dev, TX_CFG, &Value32));
        // Let it try to do the 1000 reads even if the reg reads are failing
        // If the previous write did go thru at least this way we have a better
        // chance of making sure the transmit path did stop.
    }
    while ( (++Count<1000) && ((Value32 & (TX_CFG_STOP_ | TX_CFG_ON_)) != 0) );

	// Disable Mac TX
	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MAC_CR, &Value32));
	Value32 &= ~MAC_CR_TXEN_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MAC_CR,  Value32 ));

    SMSC_TRACE(DBG_TX,"<-- smsc9500_stopTxPath\n");
    ret = SMSC9500_SUCCESS;
DONE:
    return ret;
}

static int smsc9500_stopAndFlushTxPath(struct usbnet *dev)
/*++

Routine Description:

    This routine Stops the Tx Path at both Mac and USB and then flushes
    the Tx Fifo pointers.

Arguments:


Return Value:

--*/
{

	u32 Value32;
    int ret = SMSC9500_FAIL;
     SMSC_TRACE(DBG_TX,"--> smsc9500_stopAndFlushTxPath\n");


	if(smsc9500_stopTxPath(dev) < 0)return ret;

    // Flush the transmit path
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, TX_CFG, TX_CFG_FIFO_FLUSH_));

    // Should self clear way before the read.
    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, TX_CFG, &Value32));

    if (Value32 & TX_CFG_FIFO_FLUSH_)
    {
        // Flush did not self clear!
        goto DONE;
    }
     SMSC_TRACE(DBG_TX,"<-- smsc9500_stopAndFlushTxPath\n");
     ret = SMSC9500_SUCCESS;
DONE:
    return ret;
}


static int smsc9500_stopRxPath(struct usbnet * dev)
/*++

Routine Description:

    This routine Stops the Rx Path at both Mac and USB

Arguments:

Return Value:

--*/
{
	u32 Value32;
    int ret = SMSC9500_FAIL;
    int Count = 0;

	SMSC_TRACE(DBG_RX,"--> smsc9500_stopRxPath\n");

   // Clr the Rx Stop bit if not already
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, INT_STS, INT_STS_RX_STOP_));

    // Disable the receiver at the Mac
    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MAC_CR, &Value32));
    CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MAC_CR, Value32 & (~MAC_CR_RXEN_)));

    // The Rx Stop bit should assert as soon as the packet "in flight" makes
    // it into the Mac, worst case is 10 Mbps HD. This will be ~2ms tops
    // Assuming one register read per (micro)frame the case of high speed USB
    // - 125us register read cycle time - is the worse and would need up to
    // 16 reads. Let's just round up to 20.
    do
    {
        CHECK_RETURN_STATUS(smsc9500_read_reg(dev, INT_STS, &Value32));
        // Let it try to do the 20 reads even if the reg reads are failing
        // If the previous write did go thru at least this way we have a better
        // chance of making sure the receiver did stop.
    }
    while ( (++Count<20) && ((Value32 & INT_STS_RX_STOP_) == 0) );



	// Disable Mac RX
	//CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MAC_CR, &Value32));
	//Value32 &= ~MAC_CR_RXEN_;
	//CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MAC_CR, Value32));

    SMSC_TRACE(DBG_RX,"<-- smsc9500_stopRxPath\n");
    ret = SMSC9500_SUCCESS;
DONE:
    return ret;
}

static int smsc9500_stopAndFlushRxPath(struct usbnet *dev)
/*++

Routine Description:

    This routine Stops the Rx Path at both Mac and USB and then flushes
    the Rx Fifo pointers.

Arguments:


Return Value:

--*/
{
	u32 Value32;
    int ret = SMSC9500_FAIL;
    SMSC_TRACE(DBG_RX,"--> smsc9500_stopAndFlushRxPath\n");


	if(smsc9500_stopRxPath(dev) < 0)goto DONE;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, RX_CFG, RX_FIFO_FLUSH_));

    // Should self clear way before the read.
    CHECK_RETURN_STATUS(smsc9500_read_reg(dev, RX_CFG, &Value32));

    if (Value32 & RX_FIFO_FLUSH_)
    {
        // Flush did not self clear!
       goto DONE;
    }

    SMSC_TRACE(DBG_RX,"<-- smsc9500_stopAndFlushRxPath\n");
    ret = SMSC9500_SUCCESS;
DONE:
    return ret;
}

static u16 CalculateCrc16(const BYTE * bpData,const u32 dwLen, const BOOLEAN fBitReverse)
{
	const u16 wCrc16Poly = 0x8005U;	// s/b international standard for CRC-16
								// x^16 + x^15 + x^2 + 1
	//u16 wCrc16Poly = 0xA001;	// reverse
	u16 i, j, bit;
	u16 wCrc = 0xFFFFU;
	u16 wMsb;
	BYTE bCurrentByte;
	u16 wNumOfBits = 16U;
	u16 wCrcOut=0;

	wNumOfBits = wNumOfBits; // to avoid lint warning

	for (i=0; i<(u16)dwLen; i++)
	{
		bCurrentByte = *bpData++;

		for (bit=(u16)0U; bit<(u16)8U; bit++)
		{
			wMsb = wCrc >> 15;
			wCrc <<= 1;

			if (wMsb ^ (u16)(bCurrentByte & 1))
			{
				wCrc ^= wCrc16Poly;
				wCrc |= (u16)0x0001U;
			}
			bCurrentByte >>= 1;
		}
	}
	//bit reverse if needed
	// so far we do not need this for 117
	// but the standard CRC-16 seems to require this.
	if (fBitReverse)
	{
		j = 1;
		for (i=(u16)(1<<(wNumOfBits-(u16)1U)); i; i = i>>1) {
			if (wCrc & i)
			{
				wCrcOut |= j;
			}
			j <<= 1;
		}
		wCrc = wCrcOut;
	}

	return wCrc;
}

/*++

Routine Description:

    This routine set/reset General Purpose Output.

Arguments:

    Adapter - pointer to our Adapter
    Gpo - Gpo [0:10]
    State - 1 = ON or 0 = OFF

Return Value:

--*/

static int SetGpo(struct usbnet * dev,  u32 Gpo, u32 State)
{
    int ret = SMSC9500_FAIL;
    u32 Value32, Status;

    if ((Gpo > 10) || (State > 1))
    {
        if (State > 1)
        {
                SMSC_WARNING("Gpo%d state (%d) is out of range [0:1] in NICSetGpo\n", Gpo, State);
        }
        goto Exit_NICSetGpo;
    }

    if (Gpo < 8)
    {
        if(smsc9500_read_reg( dev, GPIO_CFG, &Value32 ) < 0){
            SMSC_WARNING("Failed to read GPIO_CFG\n");
            goto Exit_NICSetGpo;
        }

        Value32 &= (~(GPIO_CFG_GPO0_DATA_ << Gpo));
        Value32 |= (State << Gpo);

        Status = smsc9500_write_reg(dev, GPIO_CFG, Value32);
        if (Status < 0)
            goto Exit_NICSetGpo;
    }
    else
    {
        Status = smsc9500_read_reg(dev, LED_GPIO_CFG, &Value32 );
        if (Status < 0)
             goto Exit_NICSetGpo;

        Value32 &= (~(LED_GPIO_CFG_GPDATA_08_ << (Gpo-8)));
        Value32 |= (State << (Gpo-8));

        Status = smsc9500_write_reg(dev, LED_GPIO_CFG, Value32 );
        if (Status < 0)
             goto Exit_NICSetGpo;
      }

    ret = SMSC9500_SUCCESS;
Exit_NICSetGpo:
    return ret;
}

/*++

Routine Description:

    This routine set device at suspend3 state

Arguments:

    netdev - pointer to our device

Return Value:

--*/
static int SetWakeupOnSuspend3(struct net_device *netdev)
{
	struct usbnet * dev=netdev_priv(netdev);
	int ret = SMSC9500_FAIL;
	u32 Value32;
	BUG_ON(!dev);

	SMSC_TRACE(DBG_PWR,"Setting Suspend3 mode\n");

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, RX_FIFO_INF, &Value32));

	if((Value32 & 0xFFFF) != 0){
		SMSC_TRACE(DBG_PWR,"Rx FIFO is not empty, abort suspend\n");
		goto DONE;
	}else{
		SMSC_TRACE(DBG_PWR,"Rx FIFO is empty, continue suspend\n");
	}

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
	Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
	Value32 |= PM_CTL_SUS_MODE_3 | PM_CTL_RES_CLR_WKP_STS;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));
	Value32 &= ~PM_CTL_WUPS_;
	Value32 |= PM_CTL_WUPS_WOL_; //Clear wol status
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));

	CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

	ret = SMSC9500_SUCCESS;

DONE:
    return ret;
}

static int SetWakeupEvents(struct net_device *netdev)
{
	WAKEUP_FILTER sFilter;
	u32 dwValue,dwTemp,Value32;
	u16 wValue;
	int i=0, ret = SMSC9500_FAIL;
	int filterMaskCnt=0, filterCmdCnt=0, filterOffsetCnt=0, filterCrcCnt=0;

	struct usbnet * dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	u32 opts = adapterData->WolWakeupOpts;

	BYTE bBcast[BCAST_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	BYTE bMcast[MCAST_LEN] = {0x01, 0x00, 0x5E};
	BYTE bArp[ARP_LEN] = {0x08, 0x06};

	BUG_ON(!netdev);

	SMSC_TRACE(DBG_PWR,"In SetWakeupEvents. \n");

	if(opts & (WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC))
	{
		if(opts & WAKE_PHY) {

		// Clear any pending Phy interrupt and enable the mask for it
			adapterData->systemSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN_;
			if(adapterData->dwLinkSettings&LINK_AUTO_NEGOTIATE)
				adapterData->systemSuspendPHYEvent |= PHY_INT_MASK_ANEG_COMP_;

			EnablePHYWakeupInterrupt(dev, adapterData->systemSuspendPHYEvent);

			// If there's currently no link we can use Suspend1
			CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_BSR, &Value32));
			CHECK_RETURN_STATUS(smsc9500_read_phy(dev, PHY_BSR, &Value32));
			wValue=(u16)Value32;
			if (!(wValue & PHY_BSR_LINK_STATUS_))
			{
				SMSC_TRACE(DBG_PWR,"Setting PHY in PD/ED Mode.\n");

				//Enable the energy detect power-down mode
				CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_MODE_CTRL_STS,&dwValue));
				wValue=(u16)dwValue;
				wValue |= MODE_CTRL_STS_EDPWRDOWN_;
				CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_MODE_CTRL_STS, wValue));

				CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwTemp)); //Read to clear
				//Enable ENERGYON interrupt source

				CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
				wValue=(u16)dwValue;
				wValue |= PHY_INT_MASK_ENERGY_ON_;
				CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK,wValue));

				// Put Lan9500 in Suspend1
				SMSC_TRACE(DBG_PWR,"Setting Suspend1 mode\n");
				CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
				Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
				Value32 |= PM_CTL_SUS_MODE_1;
				CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));
				Value32 &= ~PM_CTL_WUPS_;
				Value32 |= (PM_CTL_WUPS_ED_ | PM_CTL_ED_EN_); //Clear wol status, enable energy detection
				CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));

				CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));
			}
			else
			{
				// Put Lan9500 in Suspend0
				SMSC_TRACE(DBG_PWR,"Setting Suspend0 mode\n");
				CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
				Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
				Value32 |=PM_CTL_SUS_MODE_0;
				CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));
				Value32 &= ~PM_CTL_WUPS_;
				Value32 |= PM_CTL_WUPS_ED_; //Clear wol status
				CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));
				CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));
			}
		}
		else
		{
			// Clear any pending Phy interrupt and disable the mask for it
			Value32 = PHY_INT_MASK_LINK_DOWN_;
			if(adapterData->dwLinkSettings&LINK_AUTO_NEGOTIATE)
				Value32 |= PHY_INT_MASK_ANEG_COMP_;
			DisablePHYWakeupInterrupt(dev, Value32);
		}


		if(opts & (WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_UCAST|WAKE_MAGIC))
		{
			if(opts & (WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_UCAST)){

					int filter = 0;

					memset(&sFilter,0,sizeof(sFilter));

					if(opts & WAKE_BCAST) {
						SMSC_TRACE(DBG_PWR,"Set broadicast detection\n");
						sFilter.dwFilterMask[filter * 4] = 0x003F;
						sFilter.dwFilterMask[filter * 4 + 1] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 2] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 3] = 0x00;
						sFilter.dwCommad[filter/4] |= 0x05UL << ((filter % 4) * 8); //set command
						sFilter.dwOffset[filter/4] |= 0x00 << ((filter % 4) * 8);
						sFilter.dwCRC[filter/2] |= CalculateCrc16(bBcast, BCAST_LEN, FALSE) << ((filter % 2) * 16);
						filter++;
					}
					if(opts & WAKE_MCAST) {
						SMSC_TRACE(DBG_PWR,"Set multicast detection\n");
						sFilter.dwFilterMask[filter * 4] = 0x0007;
						sFilter.dwFilterMask[filter * 4 + 1] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 2] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 3] = 0x00;
						sFilter.dwCommad[filter/4] |= 0x09UL << ((filter % 4) * 8); //set command
						sFilter.dwOffset[filter/4] |= 0x00  << ((filter % 4) * 8);
						sFilter.dwCRC[filter/2] |= (CalculateCrc16(bMcast, MCAST_LEN, FALSE) << ((filter % 2) * 16));
						filter++;
					}
					if(opts & WAKE_ARP) {
						SMSC_TRACE(DBG_PWR,"Set ARP detection\n");
						sFilter.dwFilterMask[filter * 4] = 0x0003; //Check two bytes for ARP
						sFilter.dwFilterMask[filter * 4 + 1] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 2] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 3] = 0x00;
						sFilter.dwCommad[filter/4] |= 0x05UL << ((filter % 4) * 8); //set command
						sFilter.dwOffset[filter/4] |= 0x0C << ((filter % 4) * 8);
						sFilter.dwCRC[filter/2]= CalculateCrc16(bArp, ARP_LEN, FALSE) << ((filter % 2) * 16); //This is ARP type
						filter++;
					}
					if(opts & WAKE_UCAST){
						SMSC_TRACE(DBG_PWR,"Set UCAST detection\n");
						sFilter.dwFilterMask[filter * 4] = 0x003F; //Check nothing
						sFilter.dwFilterMask[filter * 4 + 1] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 2] = 0x00;
						sFilter.dwFilterMask[filter * 4 + 3] = 0x00;
						sFilter.dwCommad[filter/4] |= 0x01UL << ((filter % 4) * 8); //set command
						sFilter.dwOffset[filter/4] |= 0x00 << ((filter % 4) * 8);
						sFilter.dwCRC[filter/2] |= CalculateCrc16(&netdev->dev_addr[0], 6, FALSE) << ((filter % 2) * 16);
						filter++;
					}

					if(!dev->chipDependFeatures[FEATURE_WUFF_8]){
						filterMaskCnt = LAN9500_WUFF_NUM * 4;
						filterCmdCnt = LAN9500_WUFF_NUM / 4;
						filterOffsetCnt = LAN9500_WUFF_NUM / 4;
						filterCrcCnt = LAN9500_WUFF_NUM / 2;
					}else{
						filterMaskCnt = LAN9500A_WUFF_NUM * 4;
						filterCmdCnt = LAN9500A_WUFF_NUM / 4;
						filterOffsetCnt = LAN9500A_WUFF_NUM / 4;
						filterCrcCnt = LAN9500A_WUFF_NUM / 2;
					}

					for(i=0; i<filterMaskCnt; i++){
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUFF, sFilter.dwFilterMask[i]));
					}
					for(i=0; i<filterCmdCnt; i++){
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUFF, sFilter.dwCommad[i]));
					}
					for(i=0; i<filterOffsetCnt; i++){
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUFF, sFilter.dwOffset[i]));
					}
					for(i=0; i<filterCrcCnt; i++){
						CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUFF, sFilter.dwCRC[i]));
					}

					// Clear any pending pattern match packet status
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 |= WUCSR_WUFR_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));

					// Enable pattern match packet wake

					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 |= WUCSR_WAKE_EN_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));

					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
					Value32 |= PM_CTL_WOL_EN_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));

				}
				else
				{
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 &= (~WUCSR_WAKE_EN_);
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));
				}

			if (opts & WAKE_MAGIC)
				{
					SMSC_TRACE(DBG_PWR,"Setting magic packet detection\n");
					// Clear any pending magic packet status
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 |=  WUCSR_MPR_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));

					// Enable MP wake

					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 |=  WUCSR_MPEN_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));

					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
					Value32 |=  PM_CTL_WOL_EN_;
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32 ));
				}
				else
				{
					CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
					Value32 &= (~WUCSR_MPEN_);
					CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));
				}


			//enable recevier
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,MAC_CR,&dwValue));
			dwValue |= MAC_CR_RXEN_;
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,MAC_CR, dwValue));

			SMSC_TRACE(DBG_PWR,"Setting Suspend0 mode\n");
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
			Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
			Value32 |= PM_CTL_SUS_MODE_0;
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL,  Value32));
			Value32 &= ~PM_CTL_WUPS_;
			Value32 |= PM_CTL_WUPS_WOL_; //Clear wol status, should not change suspned_mode while clearing WUPS_sts[1]
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));

			CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));

			CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));


		}

	}else {

		SMSC_TRACE(DBG_PWR,"Disabling Wake events in ESS Regs. ");

		// Disable Energy detect (Link up) & Wake up events to do USB wake
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, WUCSR, &Value32));
		Value32 |=~(WUCSR_MPEN_ | WUCSR_WAKE_EN_ );
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, WUCSR, Value32));

		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
		Value32 &= ~(PM_CTL_ED_EN_ | PM_CTL_WOL_EN_);
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));

		// Put Lan9500 in Suspend2
		SMSC_TRACE(DBG_PWR,"Setting Suspend2 mode\n");
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &Value32));
		Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
		Value32 |= PM_CTL_SUS_MODE_2;
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, Value32));
	}

    ret = 0;
DONE:
    return ret;
}

static int ResetWakeupEvents(struct net_device *netdev)
{
	u32 dwValue;
	u16 wValue;
	int ret = SMSC9500_FAIL;

	struct usbnet * dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	BUG_ON(!adapterData);
	SMSC_TRACE(DBG_PWR,"In ResetWakeupEvents. \n");

	if(!(adapterData->WolWakeupOpts & (WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC)))
		return ret;

	smsc9500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);
	if(adapterData->WolWakeupOpts & WAKE_PHY) {
		//Disable the energy detect power-down mode
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_MODE_CTRL_STS,&dwValue));
		wValue=(u16)dwValue;
		wValue &= ~MODE_CTRL_STS_EDPWRDOWN_;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_MODE_CTRL_STS,wValue));

		//Disable energy-detect wake-up
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev,PM_CTRL,&dwValue));
		dwValue &= ~PM_CTL_PHY_RST_;
		dwValue |= PM_CTL_WUPS_; //Clear wake-up status
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev,PM_CTRL, dwValue));

		DisablePHYWakeupInterrupt(dev, adapterData->systemSuspendPHYEvent);
	}else{

		if(adapterData->WolWakeupOpts & (WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_UCAST)){
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,WUCSR,&dwValue));
			dwValue &= ~WUCSR_WAKE_EN_;   //Disable Wake-up frame detection
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUCSR, dwValue));

		}
		if(adapterData->WolWakeupOpts & WAKE_MAGIC){//Set Magic packet detection
			CHECK_RETURN_STATUS(smsc9500_read_reg(dev,WUCSR,&dwValue));
			dwValue &= ~WUCSR_MPEN_;   //Disable magic frame detection
			CHECK_RETURN_STATUS(smsc9500_write_reg(dev,WUCSR, dwValue));
		}
		//Disable wake-up frame interrupt
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev,PM_CTRL,&dwValue));
		dwValue &= ~PM_CTL_WOL_EN_;
		dwValue |= PM_CTL_WUPS_; //Clear wake-up status
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev,PM_CTRL, dwValue));
	}

    ret = 0;
DONE:
    return ret;
}

static int SetLinkDownWakeupEvents(struct usbnet *dev, int wakeUpMode)
{
	u32 dwValue;
	u16 wValue;
	int ret = SMSC9500_FAIL;

	BUG_ON(!dev);

	SMSC_TRACE(DBG_PWR,"In SetLinkDownWakeupEvents. \n");

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &dwValue));
	dwValue |= PM_CTL_ED_EN_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, dwValue));

	SMSC_TRACE(DBG_PWR,"Setting PHY in PD/ED Mode\n");

	if(wakeUpMode == WAKEPHY_ENERGY){
		//Enable the energy detect power-down mode
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_MODE_CTRL_STS,&dwValue));
		wValue=(u16)dwValue;
		wValue |= MODE_CTRL_STS_EDPWRDOWN_;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_MODE_CTRL_STS, wValue));

		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read to clear
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read two times

		//Enable interrupt source
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
		wValue |= (PHY_INT_MASK_ENERGY_ON_ | PHY_INT_MASK_ANEG_COMP_);
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK,wValue));

		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &dwValue));
		dwValue &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
		dwValue |= PM_CTL_SUS_MODE_1;
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, dwValue));

		CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

		SMSC_TRACE(DBG_PWR,"Setting Suspend1 mode\n");

	}else if (wakeUpMode == WAKEPHY_NEGO_COMPLETE){
		//Disable the energy detect power-down mode
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_MODE_CTRL_STS,&dwValue));
		dwValue &= ~MODE_CTRL_STS_EDPWRDOWN_;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_MODE_CTRL_STS, dwValue));

		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read to clear
		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read two times
		//Enable interrupt source

		CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
		dwValue |= PHY_INT_MASK_ANEG_COMP_;
		CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK,dwValue));

		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &dwValue));
		dwValue &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
		dwValue |= PM_CTL_SUS_MODE_0;
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, dwValue));

		CHECK_RETURN_STATUS(smsc9500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

		SMSC_TRACE(DBG_PWR,"Setting Suspend0 mode\n");

	}

    ret = 0;
DONE:
    return ret;
}

static int ResetLinkDownWakeupEvents(struct usbnet *dev)
{
	u32 dwValue;
	u16 wValue;
	int ret = SMSC9500_FAIL;

	SMSC_TRACE(DBG_PWR,"In ResetLinkDownWakeupEvents. \n");

	smsc9500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);

	//Disable the energy detect power-down mode
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_MODE_CTRL_STS,&dwValue));
	wValue=(u16)dwValue;
	wValue &= ~MODE_CTRL_STS_EDPWRDOWN_;
	CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_MODE_CTRL_STS,wValue));

	//Disable ENERGYON interrupt source
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
	wValue=(u16)dwValue;
	wValue &= ~PHY_INT_MASK_ENERGY_ON_;
	if(dev->linkDownSuspend == WAKEPHY_ENERGY){
		wValue &= ~(PHY_INT_MASK_ENERGY_ON_ | PHY_INT_MASK_ANEG_COMP_);
	}else if (dev->linkDownSuspend == WAKEPHY_NEGO_COMPLETE){
		wValue &= ~PHY_INT_MASK_ANEG_COMP_;
	}

	CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK, wValue));

	//Disable energy-detect wake-up
	CHECK_RETURN_STATUS(smsc9500_read_reg(dev,PM_CTRL,&dwValue));
	dwValue &= ~PM_CTL_ED_EN_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev,PM_CTRL, dwValue));

    ret = 0;
DONE:
    return ret;
}

/*++

Routine Description:

    This routine enables PHY interrupt

Arguments:
	interrupt:  Bit mask for PHY interrupt
Return Value:

--*/

static int EnablePHYWakeupInterrupt(struct usbnet *dev, u32 interrupt)
{
	u32 dwValue;
	int ret = SMSC9500_FAIL;

	BUG_ON(!dev);

	SMSC_TRACE(DBG_PWR,"In EnablePHYWakeupInterrupt. \n");

	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read to clear

	//Enable interrupt source
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
	dwValue |= interrupt;
	CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK,dwValue));

	if(dwValue){
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &dwValue));
		dwValue &= ~PM_CTL_PHY_RST_;
		dwValue |= PM_CTL_ED_EN_;
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, dwValue));
	}

	ret = SMSC9500_SUCCESS;
DONE:
	return ret;

}

/*++

Routine Description:

    This routine Disables linkdown interrupt in the PHY

Arguments:

Return Value:

--*/

static int DisablePHYWakeupInterrupt(struct usbnet *dev, u32 interrupt)
{
	u32 dwValue;
	int ret = SMSC9500_FAIL;

	BUG_ON(!dev);

	SMSC_TRACE(DBG_PWR,"In DisablePHYWakeupInterrupt. \n");

	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_SRC,&dwValue)); //Read to clear

	//Disable interrupt source
	CHECK_RETURN_STATUS(smsc9500_read_phy(dev,PHY_INT_MASK,&dwValue));
	dwValue &= PHY_INT_MASK_ALL;
	dwValue &= ~interrupt;
	CHECK_RETURN_STATUS(smsc9500_write_phy(dev,PHY_INT_MASK,dwValue));

	if(dwValue == 0){ //All interrupt sources are disabled
		CHECK_RETURN_STATUS(smsc9500_read_reg(dev, PM_CTRL, &dwValue));
		dwValue &= ~(PM_CTL_PHY_RST_ | PM_CTL_ED_EN_);
		CHECK_RETURN_STATUS(smsc9500_write_reg(dev, PM_CTRL, dwValue));
	}

	ret = SMSC9500_SUCCESS;

DONE:
	return ret;

}

/*++

Routine Description:

    This routine Starts the Tx path at the MAC and sets the flag to accept
    bulk out requests

--*/
static int StartTxPath(struct usbnet * dev )
{
	u32 Value32;
	int ret = SMSC9500_FAIL;

	// Enable Tx at MAC
	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MAC_CR, &Value32));
	Value32 |= MAC_CR_TXEN_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MAC_CR, Value32));

	// Enable Tx at SCSRs
	Value32 = TX_CFG_ON_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, TX_CFG, Value32));

    ret = 0;
DONE:
    return ret;
}

/*++

Routine Description:

    Starts the Receive path.

    Note that if we are operating in USB bandwidth friendly mode we defer
    starting the bulk in requests for now (will start when operational mode pipe signals
    receive data available).

--*/
static int StartRxPath(struct usbnet *dev )
{
	u32 Value32;
    int ret = SMSC9500_FAIL;

	CHECK_RETURN_STATUS(smsc9500_read_reg(dev, MAC_CR, &Value32));
	Value32 |= MAC_CR_RXEN_;
	CHECK_RETURN_STATUS(smsc9500_write_reg(dev, MAC_CR, Value32));

    ret = 0;
DONE:
    return ret;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11))
static int Smsc9500_suspend (struct usb_interface *intf,  u32 state)
#else
static int Smsc9500_suspend (struct usb_interface *intf, pm_message_t state)
#endif
{

	struct usbnet		*dev = usb_get_intfdata(intf);
	int ret = SMSC9500_SUCCESS;
	u32 dwValue;

    SMSC_TRACE(DBG_PWR,"---->Smsc9500_suspend\n");
    BUG_ON(!dev);
    BUG_ON(!dev->udev);

    smsc9500_read_reg(dev,WUCSR,&dwValue);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_PM
    if(dev->udev->auto_pm) //Internal pm event, autosuspend
#else
    if(0)
#endif //CONFIG_PM
#else
    if(0)
#endif
    {
        ret = Smsc9500AutoSuspend(intf, state);
    }else
    {//It is system suspend
        ret = Smsc9500SystemSuspend(intf, state);
    }

    SMSC_TRACE(DBG_PWR,"<----Smsc9500_suspend\n");

	return ret;
}


static int Smsc9500AutoSuspend (struct usb_interface *intf, pm_message_t state)
{

	struct usbnet		*dev = usb_get_intfdata(intf);
	u32 Value32;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	int suspendFlag = dev->suspendFlag;
	int ret = SMSC9500_SUCCESS;

	SMSC_TRACE(DBG_PWR,"---->Smsc9500AutoSuspend, suspendFlag = 0x%x\n", suspendFlag);

	dev->suspendFlag = 0;

	if (netif_running (dev->net))
	{
		adapterData->wakeupOptsBackup = adapterData->WolWakeupOpts;

		smsc9500_read_phy(dev, PHY_BSR, &Value32);
		if(smsc9500_read_phy(dev, PHY_BSR, &Value32) < 0){
			return SMSC9500_FAIL;
		}

		if (!(Value32 & PHY_BSR_LINK_STATUS_))//last minute link check
		{//Link is down
			if(dev->smartDetach){//Highest priority
				if(!dev->pmLock){//Resume immediately, then detach device from USB bus.
					smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
				}
				ret = SMSC9500_FAIL;
				goto _SUSPEND_EXIT;
			}else if(dev->linkDownSuspend || (suspendFlag & AUTOSUSPEND_LINKDOWN)){//Always check it to save more power
				dev->suspendFlag |= AUTOSUSPEND_LINKDOWN;
				ret = SetLinkDownWakeupEvents(dev, dev->linkDownSuspend);
			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC){//suspend on s0, work for lan9500 and lan9500a
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC;
				adapterData->WolWakeupOpts = (WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP);
				ret = SetWakeupEvents(dev->net);

				//Save PHY interrupt event, so we can clear it after waking up.
				adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_ENERGY_ON_ | PHY_INT_MASK_ANEG_COMP_;
				ret = EnablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);

			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//suspend on s3, only for lan9500a
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
				ret = SetWakeupOnSuspend3(dev->net);
				if(ret != SMSC9500_FAIL){
					//Save PHY interrupt event, so we can clear it after waking up.
					adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_ENERGY_ON_ | PHY_INT_MASK_ANEG_COMP_;
					ret = EnablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
				}

			}else{
				SMSC_WARNING("auto suspend event is null\n");
				ret = SMSC9500_FAIL;
			}

		}else{//link is up

			 if(suspendFlag & AUTOSUSPEND_DYNAMIC){//suspend on s0, work for lan9500 and lan9500a
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC;
				adapterData->WolWakeupOpts = (WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP);
				ret = SetWakeupEvents(dev->net);

				//Save PHY interrupt event, so we can clear it after waking up.
				adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN_;
				ret = EnablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//suspend on s3, only for lan9500a
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
				ret = SetWakeupOnSuspend3(dev->net);
				if(ret != SMSC9500_FAIL){
					//Save PHY interrupt event, so we can clear it after waking up.
					adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN_;
					ret = EnablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
				}
			}else{
				ret = SMSC9500_FAIL;
			}
		}

		if(ret != SMSC9500_SUCCESS){
			//SMSC_WARNING("Failed to suspend device\n");
			dev->suspendFlag = 0;
			if(!dev->pmLock){//Resume immediately
				smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
				Tx_WakeQueue(dev,0x04UL);
				tasklet_schedule (&dev->bh);
			}
			goto _SUSPEND_EXIT;
		}else{
			smscusbnet_FreeQueue(dev);
			del_timer_sync(&dev->LinkPollingTimer);
		}

	}else{//Interface down
		u32 Value32;
		SMSC_TRACE(DBG_PWR,"Interface is down, set suspend2 mode\n");
		smsc9500_read_reg(dev, PM_CTRL, &Value32);
		Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
		Value32 |= PM_CTL_SUS_MODE_2;
		smsc9500_write_reg(dev, PM_CTRL, Value32);

		dev->suspendFlag |= AUTOSUSPEND_INTFDOWN;
	}

_SUSPEND_EXIT:
	SMSC_TRACE(DBG_PWR,"<----Smsc9500AutoSuspend\n");
	return ret;
}

static int Smsc9500SystemSuspend (struct usb_interface *intf, pm_message_t state)
{

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_TRACE(DBG_PWR,"---->Smsc9500SystemSuspend\n");

	dev->idleCount = 0;
	dev->suspendFlag = 0;

	if (dev->net && netif_running (dev->net) && netif_device_present (dev->net))
	{
		netif_device_detach (dev->net);
		smscusbnet_FreeQueue(dev);

		dev->StopLinkPolling=TRUE;
		del_timer_sync(&dev->LinkPollingTimer);

		Tx_StopQueue(dev,0x04UL);
		smsc9500_stopAndFlushTxPath(dev);
		smsc9500_stopAndFlushRxPath(dev);

		SetWakeupEvents(dev->net);

	}else{
		// Put Lan9500 in Suspend2
		u32 Value32;
		SMSC_TRACE(DBG_PWR,"Setting Suspend2 mode\n");
		smsc9500_read_reg(dev, PM_CTRL, &Value32);
		Value32 &= (~(PM_CTL_SUS_MODE_ | PM_CTL_WUPS_ | PM_CTL_PHY_RST_));
		Value32 |= PM_CTL_SUS_MODE_2;
		smsc9500_write_reg(dev, PM_CTRL, Value32);

		dev->suspendFlag |= AUTOSUSPEND_INTFDOWN;
	}

	SMSC_TRACE(DBG_PWR,"<----Smsc9500SystemSuspend\n");

	return SMSC9500_SUCCESS;
}

static int Smsc9500_resume(struct usb_interface *intf)
{
	int ret = SMSC9500_SUCCESS;

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_TRACE(DBG_PWR,"--->in Smsc9500_resume\n");
	BUG_ON(!dev);
	BUG_ON(!dev->udev);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	 //autoresume
    if(dev->suspendFlag)
#else
    if(0)
#endif
    {
        ret = Smsc9500AutoResume(intf);
    }else
    {
        ret = Smsc9500SystemResume(intf);
    }
	SMSC_TRACE(DBG_PWR,"------->out of in Smsc9500_resume\n");
	return ret;
}

static int Smsc9500AutoResume(struct usb_interface *intf)
{
	struct usbnet		*dev = usb_get_intfdata(intf);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue;

	SMSC_TRACE(DBG_PWR,"--->in Smsc9500AutoResume\n");

	clear_bit(EVENT_DEV_RECOVERY, &dev->flags);

	if (dev->suspendFlag & AUTOSUSPEND_INTFDOWN){
		SMSC_TRACE(DBG_PWR,"Resume from interface down\n");
		goto _EXIT_AUTORESUME;
	}

	if(!dev->pmLock){
		smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
	}

	if(dev->suspendFlag & AUTOSUSPEND_LINKDOWN){
		ResetLinkDownWakeupEvents(dev);
	}else if(dev->suspendFlag & AUTOSUSPEND_DYNAMIC){
		DisablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
		ResetWakeupEvents(dev->net);
	}else if(dev->suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//Resume from suspend3
		smsc9500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);

		DisablePHYWakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);

		if(smsc9500_read_reg(dev, PM_CTRL, &dwValue) < 0){
			SMSC_WARNING("Failed to read PM_CTRL");
		}
		dwValue &= (~(PM_CTL_SUS_MODE_ | PM_CTL_PHY_RST_));
		dwValue |= PM_CTL_SUS_MODE_2;

		if(smsc9500_write_reg(dev, PM_CTRL, dwValue) < 0){
			SMSC_WARNING("Failed to write PM_CTRL");
		}
		dwValue &= ~PM_CTL_WUPS_;
		dwValue |= PM_CTL_WUPS_WOL_;

		if(smsc9500_write_reg(dev, PM_CTRL, dwValue) < 0){ //Should not change suspend_mode while clearing WUPS_sts[1]
			SMSC_WARNING("Failed to write PM_CTRL");
		}

	}
	//if(dev->delay.function)(dev->delay.function)((unsigned long)dev);
	tasklet_schedule (&dev->bh);

	adapterData->WolWakeupOpts = adapterData->wakeupOptsBackup;
	adapterData->wakeupOptsBackup = 0;

    Tx_WakeQueue(dev,0x04UL);

_EXIT_AUTORESUME:
    dev->idleCount = 0;
    dev->suspendFlag = 0;

	SMSC_TRACE(DBG_PWR,"------->out of in Smsc9500AutoResume\n");
	return SMSC9500_SUCCESS;
}

static int Smsc9500SystemResume(struct usb_interface *intf)
{
	int ret=0;
#if 0 /* commenting as it was causing panic on harmony platform */
	u32 dwValue;
#endif

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_TRACE(DBG_PWR,"--->in Smsc9500SystemResume\n");

    if(netif_running (dev->net) && !netif_device_present (dev->net)){
       netif_device_attach (dev->net);
    }

#if 0 /* commenting as it was causing panic on harmony platform */
    //test if hcd is still alive

    if(smsc9500_read_reg_async(dev, MAC_CR, &dwValue, TRUE) == ASYNC_RW_SUCCESS){
        SMSC_TRACE(DBG_PWR,"hcd is alive\n");
#endif
        ret=smsc9500_reset(dev);
        StartTxPath(dev);
        StartRxPath(dev);

        ResetWakeupEvents(dev->net);

#if 0 /* commenting as it was causing panic on harmony platform */
    }else{//This will happen on suspend-to-disk, if we access usb bus, will hang on usb_kill_urb
	SMSC_TRACE(DBG_PWR,"no hcd\n");
    }
#endif

    Tx_WakeQueue(dev,0x04UL);

	init_timer(&(dev->LinkPollingTimer));
	dev->StopLinkPolling=FALSE;
	dev->LinkPollingTimer.function=smscusbnet_linkpolling;
	dev->LinkPollingTimer.data=(unsigned long) dev;
	dev->LinkPollingTimer.expires=jiffies+HZ;
	add_timer(&(dev->LinkPollingTimer));
	tasklet_schedule (&dev->bh);

    dev->idleCount = 0;

	SMSC_TRACE(DBG_PWR,"------->out of in Smsc9500SystemResume\n");
	return SMSC9500_SUCCESS;
}

static int smsc9500_device_recovery(struct usbnet *dev)
{
    u32 dwReadBuf;

    BUG_ON(!dev);

    if (dev->net && netif_device_present (dev->net))
    {

        SMSC_WARNING("Device recovery is in progress\n");

        if (smsc9500_read_reg(dev,INT_STS,&dwReadBuf)< 0)return SMSC9500_FAIL;

        smscusbnet_FreeQueue(dev);

        dev->StopLinkPolling=TRUE;
        del_timer_sync(&dev->LinkPollingTimer);

        Tx_StopQueue(dev,0x04UL);

        if(smsc9500_stopAndFlushTxPath(dev) < 0)return SMSC9500_FAIL;
        if(smsc9500_stopAndFlushRxPath(dev) < 0)return SMSC9500_FAIL;

        if(dwReadBuf & INT_STS_TXE_){// reset only when TXE occurs
            if(smsc9500_reset(dev) < 0)return SMSC9500_FAIL;
        }

        if(StartTxPath(dev) < 0)return SMSC9500_FAIL;
        if(StartRxPath(dev) < 0)return SMSC9500_FAIL;

        Tx_WakeQueue(dev,0x04UL);

        init_timer(&(dev->LinkPollingTimer));
        dev->StopLinkPolling=FALSE;
        dev->LinkPollingTimer.function=smscusbnet_linkpolling;
        dev->LinkPollingTimer.data=(unsigned long) dev;
        dev->LinkPollingTimer.expires=jiffies+HZ;
        add_timer(&(dev->LinkPollingTimer));

        tasklet_schedule (&dev->bh);

        SMSC_WARNING("Device recovery is done\n");
    }

    return SMSC9500_SUCCESS;
}

static const struct driver_info smsc9500_info = {
	.description = "smsc9500 USB 2.0 Ethernet",
	.bind = smsc9500_bind,
	.unbind=smsc9500_unbind,
	.status = smsc9500_status,
	.link_reset = smsc9500_link_reset,
	.reset = smsc9500_reset,
	.flags = FLAG_ETHER,
	.rx_fixup = smsc9500_rx_fixup,
	.tx_fixup = smsc9500_tx_fixup,
	.rx_setmulticastlist=smsc9500_rx_setmulticastlist,
};

static const struct usb_device_id	products [] = {
	{
		// SMSC9500 USB Ethernet Device
		USB_DEVICE (0x0424, PID_LAN9500),
		.driver_info = (unsigned long) &smsc9500_info,
	},
	{
		// SMSC9512/9514 USB hub with Ethernet Device
		USB_DEVICE (0x0424, PID_LAN9512),
		.driver_info = (unsigned long) &smsc9500_info,
	},
	{
		// SMSC9500A USB Ethernet Device
		USB_DEVICE (0x0424, PID_LAN9500A),
		.driver_info = (unsigned long) &smsc9500_info,
	},
	{ },		// END
};

MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver smsc9500_driver = {
	.name =		"smsc9500",
	.id_table =	products,
	.probe =	smscusbnet_probe,
	.suspend =	Smsc9500_suspend,
	.resume =	Smsc9500_resume,
	.disconnect =	smscusbnet_disconnect,
	.reset_resume = Smsc9500SystemResume,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
    .supports_autosuspend = 1,
#endif
};

static int __init smsc9500_init(void)
{

        return usb_register(&smsc9500_driver);
}
module_init(smsc9500_init);

static void __exit smsc9500_exit(void)
{
        usb_deregister(&smsc9500_driver);
}
module_exit(smsc9500_exit);

MODULE_AUTHOR("Nancy Lin and Sean(Xiang) Chen");
MODULE_DESCRIPTION("SMSC9500 USB 2.0 Ethernet Devices");
MODULE_LICENSE("GPL");

