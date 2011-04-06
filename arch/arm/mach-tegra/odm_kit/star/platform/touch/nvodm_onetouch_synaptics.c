/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  SYNAPTICS SO340010-16QFN Touch Device ODM Driver

EXTERNALIZED FUNCTIONS



INITIALIZATION AND SEQUENCING REQUIREMENTS

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================
                        EDIT HISTORY FOR MODULE

Date			Author				Descriptions
----------		-----------			------------------------------------
2010/09/12		hyeongwon.oh			code create 
									

===========================================================================*/

/*===========================================================================

                     INCLUDE FILES FOR ODM DRIVER

===========================================================================*/


#include "nvodm_onetouch_int.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_onetouch_synaptics.h"

#include "../../adaptations/pmu/max8907/max8907_supply_info_table.h"

#include "synaptics_OTConfig.h"

////////////////////////////////////////////////////////////////////
////////////       Synaptics Environment Define      ///////////////
////////////////////////////////////////////////////////////////////

#define SYNAPTICS_ONETOUCH_DEVICE_GUID				NV_ODM_GUID('o','n','e','t','o','u','c','h')

#define SYNAPTICS_I2C_SPEED_KHZ					400
#define SYNAPTICS_I2C_TIMEOUT					20
#define SYNAPTICS_I2C_RETRY_COUNT				5

#define SYNAPTICS_POR_DELAY_MS					10		//Delay after Power-On Reset

#define SYNAPTICS_DEBOUNCE_TIME_MS				0

////////////////////////////////////////////////////////////////////
///////////////       Synaptics EVENT Define      //////////////////
////////////////////////////////////////////////////////////////////

#define SYNAPTICS_MOVEMENT_THRESHOLD			2

////////////////////////////////////////////////////////////////////
/////////////// Synaptics Control & Data Register //////////////////
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
////////////////// Synaptics Data from Register ////////////////////
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
/////////////////  Data Structure for Synaptics  ///////////////////
////////////////////////////////////////////////////////////////////

typedef struct Synaptics_OneTouch_DeviceRec
{
    NvOdmOneTouchDevice OdmOneTouch;
//    NvOdmOneTouchCapabilities Caps;
    NvOdmServicesI2cHandle hOdmI2c;
    NvOdmServicesGpioHandle hGpio;
    NvOdmServicesPmuHandle hPmu;
    NvOdmGpioPinHandle hPin;
    NvOdmServicesGpioIntrHandle hGpioIntr;
    NvOdmOsSemaphoreHandle hIntSema;
    NvU32 DeviceAddr;
	NvU32 I2CInstance;
    NvU32 GpioPort;
    NvU32 GpioPin;
    NvU32 SampleRate;
    NvU32 SleepMode;
    NvU32 VddId;    
	NvU32 I2cVddId;
    NvU32 I2cClockSpeedKHz;
} Synaptics_OneTouch_Device;

// 20101223  [SU660] block touch interrupt when onetouch is on reset [START]
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
extern NvOdmServicesGpioIntrHandle hGpioIntr_touch;
#endif
// 20101223  [SU660] block touch interrupt when onetouch is on reset [END]

////////////////////////////////////////////////////////////////////
/////////////     Basic Local Functions Definition    //////////////
////////////////////////////////////////////////////////////////////

static NvBool Synaptics_OneTouch_WriteRegister (Synaptics_OneTouch_Device* hTouch, NvU8 val);
static NvBool Synaptics_OneTouch_WriteRegisterMulti(Synaptics_OneTouch_Device* hTouch, NvU8* buffer, NvU32 len);
static NvBool Synaptics_OneTouch_SetReadAddr (Synaptics_OneTouch_Device* hTouch);
static NvBool Synaptics_OneTouch_ReadRegisterOnce (Synaptics_OneTouch_Device* hTouch, NvU8* buffer, NvU32 len);
static NvBool Synaptics_OneTouch_ReadRegisterSafe (Synaptics_OneTouch_Device* hTouch,  NvU8* buffer, NvU32 len);
static void Synaptics_OneTouch_GpioIsr(void *arg);
static void Synaptics_OneTouch_InitOdmTouch (NvOdmOneTouchDevice* Dev);
static NvBool Synaptics_OneTouch_Init(Synaptics_OneTouch_Device* hTouch);


////////////////////////////////////////////////////////////////////
//////////////////    Fucntions for Synaptics    ///////////////////
////////////////////////////////////////////////////////////////////

// setting register
static NvBool Synaptics_OneTouch_WriteRegister (Synaptics_OneTouch_Device* hTouch, NvU8 val)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[1];

    arr[0] = val;		// register address
    
    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;
    
	for (i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && Error != NvOdmI2cStatus_Success; i++)
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    SYNAPTICS_I2C_SPEED_KHZ,
                                    SYNAPTICS_I2C_TIMEOUT);
	}

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] I2C Write Failure = %d (addr=0x%x, val=0x%0x)\n", Error, hTouch->DeviceAddr, val));
        return NV_FALSE;
    }

    return NV_TRUE;
}


// setting register
static NvBool Synaptics_OneTouch_WriteRegisterMulti(Synaptics_OneTouch_Device* hTouch, NvU8* buffer, NvU32 len)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 *arr;

    arr = (NvU8*)NvOdmOsAlloc(len);
    if(arr ==NULL)
    {
    	NVODMTOUCH_PRINTF(("[ONETOUCH] cannot alloc memory (len=%d)\n",len));
		return NV_FALSE;
    }

    NvOdmOsMemcpy(&arr[0], buffer, len);// register address
    
    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len;
    
	for (i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && Error != NvOdmI2cStatus_Success; i++)
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    SYNAPTICS_I2C_SPEED_KHZ,
                                    SYNAPTICS_I2C_TIMEOUT);
	}
    
    NvOdmOsFree(arr);
    
    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] I2C Write Failure = %d (addr=0x%x)\n", Error, hTouch->DeviceAddr));
        return NV_FALSE;
    }
    
    return NV_TRUE;
}


// set register address before read
static NvBool Synaptics_OneTouch_SetReadAddr (Synaptics_OneTouch_Device* hTouch)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;

	NvU8 pReg[2];
	
	pReg[0]	=	OT_DATA_REG_START_ADDR_HIGH;
	pReg[1]	=	OT_DATA_REG_START_ADDR_LOW;
	
	// set register address
	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = (NvU8*)&pReg;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

	for (i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && Error != NvOdmI2cStatus_Success; i++)
	{
		Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    SYNAPTICS_I2C_SPEED_KHZ,
                                    SYNAPTICS_I2C_TIMEOUT);
	}

	if (Error != NvOdmI2cStatus_Success)
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] I2C Write Failure = %d (addr=0x%x)\n", Error, hTouch->DeviceAddr));
		return NV_FALSE;
	}

	return NV_TRUE;
}


static NvBool Synaptics_OneTouch_ReadRegisterOnce (Synaptics_OneTouch_Device* hTouch, NvU8* buffer, NvU32 len)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;
        
    TransactionInfo.Address = hTouch->DeviceAddr | 0x1;	//read
    TransactionInfo.Buf = buffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

	for (i = 0; i < SYNAPTICS_I2C_RETRY_COUNT && Error != NvOdmI2cStatus_Success; i++)
	{
    	Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,		// transactioncnt
                                SYNAPTICS_I2C_SPEED_KHZ,
                                SYNAPTICS_I2C_TIMEOUT);
	}

	if (Error != NvOdmI2cStatus_Success)
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] I2C Read Failure = %d (addr=0x%x)\n", Error, hTouch->DeviceAddr));
		return NV_FALSE;
	}

	return NV_TRUE;
}


static NvBool Synaptics_OneTouch_ReadRegisterSafe (Synaptics_OneTouch_Device* hTouch, NvU8* buffer, NvU32 len)
{
    
    if (!Synaptics_OneTouch_ReadRegisterOnce(hTouch, buffer, len))
        return NV_FALSE;

    return NV_TRUE;
}

static void Synaptics_OneTouch_GpioIsr(void *arg)
{
    Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)arg;
	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_GpioIsr\n"));

    /* Signal the touch thread to read the sample. After it is done reading the
        * sample it should re-enable the interrupt. */
    NvOdmOsSemaphoreSignal(hTouch->hIntSema);
}

NvU8 pOneTouchDataRegs[OT_NUM_DATA_REG_BYTES]; // OneTouch Data Registers

static NvBool Synaptics_OneTouch_GetSamples (Synaptics_OneTouch_Device* hTouch, NvOdmOneTouchButtonInfo* button)
{
	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_GetSamples [START]\n"));

	if (!Synaptics_OneTouch_ReadRegisterSafe (hTouch, (NvU8*)(&pOneTouchDataRegs), sizeof(pOneTouchDataRegs)))
	{
		NVODMTOUCH_PRINTF(("[Touch Driver] Error read register info using I2C\n"));
		return NV_FALSE;
	}

	printk("[ONETOUCH] pOneTouchDataRegs %d, %d, %d, %d\n", pOneTouchDataRegs[0], pOneTouchDataRegs[1], pOneTouchDataRegs[2], pOneTouchDataRegs[3]);
	if(pOneTouchDataRegs[1] &0x8)
		button->back = NV_TRUE;
	else 
		button->back = NV_FALSE;

	if(pOneTouchDataRegs[1] &0x1)
		button->menu = NV_TRUE; 	
	else
		button->menu = NV_FALSE; 	


//	Synaptics_OneTouch_WriteRegisterMulti(hTouch, pButtonBuffer, 4);
	
	Synaptics_OneTouch_SetReadAddr(hTouch); 

	//NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_OneTouch_GetSamples [END]\n"));

	return NV_TRUE;
}

static void Synaptics_OneTouch_InitOdmTouch (NvOdmOneTouchDevice* Dev)
{
    Dev->Close              = Synaptics_OneTouch_Close;
    Dev->ReadButton     		= Synaptics_OneTouch_ReadButton;
    Dev->EnableInterrupt    = Synaptics_OneTouch_EnableInterrupt;
    Dev->HandleInterrupt    = Synaptics_OneTouch_HandleInterrupt;
    Dev->SleepMode	        = Synaptics_OneTouch_SleepMode;
	Dev->PowerOnOff	        = Synaptics_OneTouch_PowerOnOff;
	Dev->InterruptMask		= Synaptics_OneTouch_InterruptMask;
    Dev->OutputDebugMessage = NV_FALSE;
}

static NvBool Synaptics_OneTouch_Init(Synaptics_OneTouch_Device* hTouch)
{
	NvU8 pBuffer[OT_NUM_CONFIG_BYTES];
	NvU8 bTemp;

	NvOdmOsMemset(pBuffer, 0, sizeof(pBuffer));

	if(hTouch == NULL)
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics Driver Initialize Fail\n"));
		return NV_FALSE;
	}

	//	Write the configuration to the OneTouch
	if(!Synaptics_OneTouch_WriteRegisterMulti(hTouch, g_OT_Config, OT_NUM_CONFIG_BYTES))
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] Fail Write the configuration to the OneTouch\n" ));
		return NV_FALSE;
	}

	// read the entire configuration back from the device	
	if(!Synaptics_OneTouch_ReadRegisterSafe(hTouch, pBuffer, OT_NUM_CONFIG_BYTES -2))
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] Fail read the entire configuration back from the devicen"));
		return NV_FALSE;	

	}
	// verify the configuration registers are written correctly
	for(bTemp=0; bTemp < (OT_NUM_CONFIG_BYTES-2); bTemp++)
	{
	  NVODMTOUCH_PRINTF(("[ONETOUCH] pBuffer[%d]=%d == g_OT_Config[%d]=%d \n", bTemp, pBuffer[bTemp], bTemp, g_OT_Config[bTemp+2]));
		if(pBuffer[bTemp] != g_OT_Config[bTemp+2])
			return NV_FALSE;
	}
	// set base address
	Synaptics_OneTouch_SetReadAddr(hTouch);

	// read the data register to deassert the attention line
	Synaptics_OneTouch_ReadRegisterSafe(hTouch, pBuffer, 1);
	
	return NV_TRUE;
}

NvBool Synaptics_OneTouch_Open (NvOdmOneTouchDeviceHandle* hDevice, NvOdmOsSemaphoreHandle* hIntSema)
{
    Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)0;
    NvU32 i;
    NvU32 found = 0;
    NvU32 I2cInstance = 0;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_OneTouch_Open\n"));
		
    hTouch = NvOdmOsAlloc(sizeof(Synaptics_OneTouch_Device));

    if (!hTouch) return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(Synaptics_OneTouch_Device));

    /* set function pointers */
    Synaptics_OneTouch_InitOdmTouch(&hTouch->OdmOneTouch);

    pConnectivity = NvOdmPeripheralGetGuid(SYNAPTICS_ONETOUCH_DEVICE_GUID);
    if (!pConnectivity)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : pConnectivity is NULL Error \n"));
        goto fail;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : didn't find any periperal in discovery query for touch device Error \n"));
        goto fail;
    }

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
                hTouch->DeviceAddr = (pConnectivity->AddressList[i].Address << 1);
                hTouch->I2CInstance = pConnectivity->AddressList[i].Instance;
                found |= 1;

                NVODMTOUCH_PRINTF(("[ONETOUCH] i2c address = %x.\n", hTouch->DeviceAddr));
                break;
            case NvOdmIoModule_Gpio:
                hTouch->GpioPort = pConnectivity->AddressList[i].Instance;
                hTouch->GpioPin = pConnectivity->AddressList[i].Address;
                found |= 2;
                break;
            case NvOdmIoModule_Vdd:
                hTouch->VddId = pConnectivity->AddressList[i].Address;
                found |= 4;
                break;
			case NvOdmIoModule_I2c_Pmu:
				hTouch->I2cVddId = pConnectivity->AddressList[i].Address;
				break;
            default:
                break;
        }
    }

    if ((found & 3) != 3)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : peripheral connectivity problem \n"));
        goto fail;
    }

    if ((found & 4) != 0)
    {
// 20101120  power off when Onetouch close    
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
        if (NV_FALSE == Synaptics_OneTouch_PowerOnOff(&hTouch->OdmOneTouch, 1))
	    	goto fail;   
#endif 
    }
    else
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics Power fail \n"));
        hTouch->VddId = 0xFF; 
    }

    hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
    if (!hTouch->hOdmI2c)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : NvOdmI2cOpen Error \n"));
        goto fail;
    }

    hTouch->hGpio = NvOdmGpioOpen();

    if (!hTouch->hGpio)
    {        
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : NvOdmGpioOpen Error \n"));
        goto fail;
    }

    NVODMTOUCH_PRINTF(("[ONETOUCH] gpio port = %d. gpio pin = %d\n", hTouch->GpioPort, hTouch->GpioPin));
    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, hTouch->GpioPort, hTouch->GpioPin);
    if (!hTouch->hPin)
    {
        NVODMTOUCH_PRINTF(("[ONETOUCH] NvOdm Touch : Couldn't get GPIO pin \n"));
        goto fail;
    }

    NvOdmGpioConfig(hTouch->hGpio,
                    hTouch->hPin,
                    NvOdmGpioPinMode_InputData);

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = SYNAPTICS_I2C_SPEED_KHZ;
    NVODMTOUCH_PRINTF(("[ONETOUCH] i2c speed = %d\n", hTouch->I2cClockSpeedKHz));
	
    /* initialize */
    if(!Synaptics_OneTouch_Init(hTouch))	 goto fail;

    *hDevice = &hTouch->OdmOneTouch;
    if (Synaptics_OneTouch_EnableInterrupt(*hDevice, *hIntSema) == NV_FALSE)
          goto fail;
	
    return NV_TRUE;

 fail:
    Synaptics_OneTouch_Close(&hTouch->OdmOneTouch);
    return NV_FALSE;
}


void Synaptics_OneTouch_Close (NvOdmOneTouchDeviceHandle hDevice)
{
    Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;
	NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_OneTouch_Close\n"));

    if (!hTouch) return;

// 20101120  power off when Onetouch close    
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
    Synaptics_OneTouch_PowerOnOff(&hTouch->OdmOneTouch, NV_FALSE);
#endif 

    if (hTouch->hGpio)
    {
        if (hTouch->hPin)
        {
            if (hTouch->hGpioIntr)
                NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPin, hTouch->hGpioIntr);

            NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hPin);
        }

        NvOdmGpioClose(hTouch->hGpio);
    }

    if (hTouch->hOdmI2c)
        NvOdmI2cClose(hTouch->hOdmI2c);

    NvOdmOsFree(hTouch);
}

void Synaptics_OneTouch_InterruptMask(NvOdmOneTouchDeviceHandle hDevice, NvBool mask)
{
	/*
		If mask is NV_TRUE, then disable irq.
		If mask is NV_FALSE, then enable irq.
	*/
	Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;
	
	if (hTouch->hGpioIntr)
	{
		NvOdmGpioInterruptMask(hTouch->hGpioIntr, mask);
		printk("[ONETOUCH] Synaptics_OneTouch_InterruptMask by %d\n", mask);
	}
}

NvBool Synaptics_OneTouch_EnableInterrupt (NvOdmOneTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema)
{
	Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;

	NV_ASSERT(hIntSema);

	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_EnableInterrupt\n"));

	/* can only be initialized once */
	if (hTouch->hGpioIntr || hTouch->hIntSema)
		return NV_FALSE;

	hTouch->hIntSema = hIntSema;    

	if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
		hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow, Synaptics_OneTouch_GpioIsr,
		(void*)hTouch, SYNAPTICS_DEBOUNCE_TIME_MS) == NV_FALSE)
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] cannot register interrupt.\n"));
		return NV_FALSE;
	}

	if (!hTouch->hGpioIntr)
		return NV_FALSE;    

	return NV_TRUE;
}


NvBool Synaptics_OneTouch_HandleInterrupt(NvOdmOneTouchDeviceHandle hDevice)
{
	Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;

	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_HandleInterrupt\n"));

	NvOdmGpioInterruptDone(hTouch->hGpioIntr);

    return NV_TRUE;
}


NvBool Synaptics_OneTouch_ReadButton (NvOdmOneTouchDeviceHandle hDevice, NvOdmOneTouchButtonInfo* button)
{
	Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;

	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_ReadButton\n"));

	if (Synaptics_OneTouch_GetSamples(hTouch, button)==NV_FALSE)
		return NV_FALSE;

	return NV_TRUE;
}

NvBool Synaptics_OneTouch_PowerOnOff (NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff)
{
// 20101120  power off when Onetouch close    
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;

    Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;

    hTouch->hPmu = NvOdmServicesPmuOpen();

	printk("[ONETOUCH] Synaptics_OneTouch_PowerOnOff\n");
	
    if (!hTouch->hPmu)
    {
		printk("[ONETOUCH] NvOdmServicesPmuOpen Error\n");
		return NV_FALSE;
    }
    
	NvOdmServicesPmuGetCapabilities( hTouch->hPmu, hTouch->VddId, &vddrailcap);

	printk("[ONETOUCH] power on[%d], vol[%d]\n", OnOff, vddrailcap.requestMilliVolts);
		
	if(OnOff)
	{
// 20101223  [SU660] block touch interrupt when onetouch is on reset [START]
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
		if(hGpioIntr_touch)
			NvOdmGpioInterruptMask(hGpioIntr_touch, NV_TRUE);
#endif
// 20101223  [SU660] block touch interrupt when onetouch is on reset [END]
			
	    NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->I2cVddId, NVODM_VOLTAGE_OFF, &settletime);
        NvOdmOsWaitUS(SYNAPTICS_POR_DELAY_MS*1000); // wait to settle power
		NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO16, NVODM_VOLTAGE_OFF, &settletime);
		NvOdmOsWaitUS(SYNAPTICS_POR_DELAY_MS*2*1000); // wait to settle power

		NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO16, MAX8907_REQUESTVOLTAGE_LDO16, &settletime);
		NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->I2cVddId, MAX8907_REQUESTVOLTAGE_LDO19, &settletime);

// 20101223  [SU660] block touch interrupt when onetouch is on reset [START]
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F)
		if(hGpioIntr_touch)
			NvOdmGpioInterruptMask(hGpioIntr_touch, NV_FALSE);
#endif
// 20101223  [SU660] block touch interrupt when onetouch is on reset [END]
	}else
		NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO16, NVODM_VOLTAGE_OFF, &settletime);

		NvOdmOsWaitUS(SYNAPTICS_POR_DELAY_MS*5*1000); // wait to settle power

    NvOdmServicesPmuClose(hTouch->hPmu);
#endif
    return NV_TRUE;
}

NvBool Synaptics_OneTouch_SleepMode (NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff)
{
//	NvU8 pBuffer[OT_NUM_CONFIG_BYTES];

	Synaptics_OneTouch_Device* hTouch = (Synaptics_OneTouch_Device*)hDevice;

	NvU8 mode_buf[4];
	
	NVODMTOUCH_PRINTF(("[ONETOUCH] Synaptics_OneTouch_SleepMode\n"));
	NVODMTOUCH_PRINTF(("[ONETOUCH] sleep mode [%d]\n", OnOff));	

	mode_buf[0] = OT_DATA_REG_SLEEP_ADDR_HIGH;
	mode_buf[1] = OT_DATA_REG_SLEEP_ADDR_LOW;

  if (OnOff)
  {
		mode_buf[2] = OT_DATA_REG_SLEEP_DATA_HIGH;
		mode_buf[3] = OT_DATA_REG_SLEEP_DATA_LOW;		
	}
	else 
	{
		mode_buf[2] = OT_DATA_REG_ACTIVE_DATA_HIGH;
		mode_buf[3] = OT_DATA_REG_ACTIVE_DATA_LOW;		
	}
	
	if(!Synaptics_OneTouch_WriteRegisterMulti(hTouch, mode_buf, sizeof(mode_buf)))
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] Error write SleepMode regs\n"));
		return NV_FALSE;
	}
#if 0
	// read the entire configuration back from the device	
	if(!Synaptics_OneTouch_ReadRegisterSafe(hTouch, pBuffer, OT_NUM_CONFIG_BYTES -2))
	{
		NVODMTOUCH_PRINTF(("[ONETOUCH] Fail read the entire configuration back from the devicen"));
		return NV_FALSE;	
	}
#endif
	Synaptics_OneTouch_SetReadAddr(hTouch); 

  return NV_TRUE;
}

