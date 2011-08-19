/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  SYNAPTICS T1320 Touch Device ODM Driver

EXTERNALIZED FUNCTIONS



INITIALIZATION AND SEQUENCING REQUIREMENTS

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================
                        EDIT HISTORY FOR MODULE

Date			Author				Descriptions
----------		-----------			------------------------------------
2010/03/12		taewan.kim			code create (basic operation implement)
2010/04/02		joseph.jung			multi touch operation support
2010/04/13		joseph.jung			touch f/w upgrade support
2010/05/31		joseph.jung			adapt for designated star panel
2010/06/17		joseph.jung			interrupt recognize from falling edge to
									active low
									

===========================================================================*/

/*===========================================================================

                     INCLUDE FILES FOR ODM DRIVER

===========================================================================*/


#include "nvodm_touch_int.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_touch_synaptics.h"
#include "nvos.h"


// 20100413  for Touch Firmware Upgrade [START]
#include "synaptics_ts_firmware.h"
// 20100413  for Touch Firmware Upgrade [END]


////////////////////////////////////////////////////////////////////
////////////       Synaptics Environment Define      ///////////////
////////////////////////////////////////////////////////////////////

#define SYNAPTICS_TOUCH_DEVICE_GUID				NV_ODM_GUID('s','y','n','t','o','u','c','h')

#define SYNAPTICS_I2C_SPEED_KHZ					400
#define SYNAPTICS_I2C_TIMEOUT					10
#define SYNAPTICS_I2C_RETRY_COUNT				5
#define SYNAPTICS_LOW_SAMPLE_RATE				0		//40 reports per-second
#define SYNAPTICS_HIGH_SAMPLE_RATE				1		//80 reports per-second

#define SYNAPTICS_SCREEN_ANGLE_MODE				1		//0=Landscape, 1=Portrait

#define SYNAPTICS_POR_DELAY_MS					100		//Delay after Power-On Reset

#define SYNAPTICS_DEBOUNCE_TIME_MS				0

#define SYNAPTICS_FINGER_MAX					10

// 20100929  Ghost finger solution is applied to a ver. 4 or later for P999DW [START]
#define SYNAPTICS_MELT_SUPPORT_VER				4
// 20100929  Ghost finger solution is applied to a ver. 4 or later for P999DW [END]


// 20101130  Touch Panel is changed and base f/w version is 11 from Rev 1.0 for P999 [START]
#define SYNAPTICS_NEW_PANEL_BASE_FW_VER			11
// 20101130  Touch Panel is changed and base f/w version is 11 from Rev 1.0 for P999 [END]


#define SYNAPTICS_SUPPORT_FW_UPGRADE

#undef SYNAPTICS_SUPPORT_CAL					// Use for only resistive touch


////////////////////////////////////////////////////////////////////
///////////////       Synaptics EVENT Define      //////////////////
////////////////////////////////////////////////////////////////////

// 20101022  touch smooth moving improve
#ifdef FEATURE_LGE_TOUCH_MOVING_IMPROVE
#define SYNAPTICS_DELTA_THRESHOLD				0x01
#else 
#define SYNAPTICS_DELTA_THRESHOLD				0x05
#endif /* FEATURE_LGE_TOUCH_MOVING_IMPROVE */
// 20101022  touch smooth moving improve

////////////////////////////////////////////////////////////////////
/////////////// Synaptics Control & Data Register //////////////////
////////////////////////////////////////////////////////////////////

#define SYNAPTICS_FLASH_CONTROL_REG				0x12
#define SYNAPTICS_DATA_BASE_REG					0x13

#define SYNAPTICS_INT_STATUS_REG				0x14

#define SYNAPTICS_DEVICE_CONTROL_REG			0x4F
#define SYNAPTICS_INTERRUPT_ENABLE_REG			0x50
#define SYNAPTICS_REPORT_MODE_REG				0x51
#define SYNAPTICS_PALM_DETECT_REG				0x52
#define SYNAPTICS_DELTA_X_THRES_REG				0x53
#define SYNAPTICS_DELTA_Y_THRES_REG				0x54
#define SYNAPTICS_VELOCITY_REG					0x55
#define SYNAPTICS_ACCELERATION_REG				0x56
#define SYNAPTICS_MAX_X_POSITION_LOW_REG		0x57
#define SYNAPTICS_MAX_X_POSITION_HIGH_REG		0x58
#define SYNAPTICS_MAX_Y_POSITION_LOW_REG		0x59
#define SYNAPTICS_MAX_Y_POSITION_HIGH_REG		0x5A


#define SYNAPTICS_MAX_TAP_TIME_REG				0x9A
#define SYNAPTICS_MIN_PRESS_TIME_REG			0x9B
#define SYNAPTICS_MIN_TAP_DIST_REG				0x9C
#define SYNAPTICS_MIN_FLICK_DIST_REG			0x9D
#define SYNAPTICS_MIN_FLICK_SPEED_REG			0x9E

#define SYNAPTICS_RMI_QUERY_BASE_REG			0xE3
#define SYNAPTICS_RMI_CMD_BASE_REG				0xE4
#define SYNAPTICS_FLASH_QUERY_BASE_REG			0xE9
#define SYNAPTICS_FLASH_DATA_BASE_REG			0xEC

// 20100929  Ghost finger solution = touch f/w [START]
#define SYNAPTICS_MELT_CONTROL_REG				0xF0

#define SYNAPTICS_NO_MELT_MODE					0
#define SYNAPTICS_MELT_MODE						1<<0
#define SYNAPTICS_AUTO_MELT_MODE				1<<1
// 20100929  Ghost finger solution = touch f/w [END]

#define SYNAPTICS_INT_FLASH						1<<0
#define SYNAPTICS_INT_STATUS 					1<<1
#define SYNAPTICS_INT_ABS0 						1<<2

#define SYNAPTICS_DEVICE_NORMAL_OPERATION		0
#define SYNAPTICS_DEVICE_SENSOR_SLEEP			1


#ifdef SYNAPTICS_SUPPORT_FW_UPGRADE
//  Constants
#define SYNAPTICS_FLASH_CMD_FW_CRC				0x01
#define SYNAPTICS_FLASH_CMD_FW_WRITE			0x02
#define SYNAPTICS_FLASH_CMD_ERASEALL			0x03
#define SYNAPTICS_FLASH_CMD_CONFIG_READ			0x05
#define SYNAPTICS_FLASH_CMD_CONFIG_WRITE		0x06
#define SYNAPTICS_FLASH_CMD_CONFIG_ERASE		0x07
#define SYNAPTICS_FLASH_CMD_ENABLE				0x0F
#define SYNAPTICS_FLASH_NORMAL_RESULT			0x80
#endif /* SYNAPTICS_SUPPORT_FW_UPGRADE */

////////////////////////////////////////////////////////////////////
////////////////// Synaptics Data from Register ////////////////////
////////////////////////////////////////////////////////////////////

// 20100319  Positon Conversion : Panel resolution -> LCD resolution [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
// convert reported coordinates, X : 1036 -> 479, Y : 1681 -> 799 (1728 -> 903)
#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		( ((int)((high_reg << 4) & 0x000007F0) | (int)(low_reg&0x0F)) * (LGE_TOUCH_RESOLUTION_X - 1) / 1036)
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		( ((int)((high_reg << 4) & 0x000007F0) | (int)((low_reg >> 4) & 0x0F)) * (LGE_TOUCH_RESOLUTION_Y - 1) / 1728)
#else
#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)((high_reg << 4) & 0x000007F0) | (int)(low_reg&0x0F))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)((high_reg << 4) & 0x000007F0) | (int)((low_reg >> 4) & 0x0F))
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100319  Positon Conversion : Panel resolution -> LCD resolution [END]

#define TS_SNTS_HAS_PINCH(gesture_reg) \
		((gesture_reg&0x40)>>6)
#define TS_SNTS_HAS_PRESS(gesture_reg) \
		((gesture_reg&0x20)>>5)
#define TS_SNTS_HAS_FLICK(gesture_reg) \
		((gesture_reg&0x10)>>4)
#define TS_SNTS_HAS_DOUBLE_TAP(gesture_reg) \
		((gesture_reg&0x04)>>2)

#define TS_SNTS_GET_REPORT_RATE(device_control_reg) \
		((device_control_reg&0x40)>>6)
// 1st bit : '0' - Allow sleep mode, '1' - Full power without sleeping
// 2nd and 3rd bit : 0x00 - Normal Operation, 0x01 - Sensor Sleep
#define TS_SNTS_GET_SLEEP_MODE(device_control_reg) \
		(device_control_reg&0x07)


////////////////////////////////////////////////////////////////////
/////////////////  Data Structure for Synaptics  ///////////////////
////////////////////////////////////////////////////////////////////

typedef struct
{
	NvU8 device_status_reg;						//0x13
	NvU8 interrupt_status_reg;					//0x14
	NvU8 finger_state_reg[3];					//0x15~0x17

	NvU8 fingers_data[SYNAPTICS_FINGER_MAX][5];	//0x18 ~ 0x49
	/* 5 data per 1 finger, support 10 fingers data
	fingers_data[x][0] : xth finger's X high position
	fingers_data[x][1] : xth finger's Y high position
	fingers_data[x][2] : xth finger's XY low position
	fingers_data[x][3] : xth finger's XY width
	fingers_data[x][4] : xth finger's Z (pressure)
	*/
	// Etc...
	NvU8 gesture_flag0;							//0x4A
	NvU8 gesture_flag1;							//0x4B
	NvU8 pinch_motion_X_flick_distance;			//0x4C
	NvU8 rotation_motion_Y_flick_distance;		//0x4D
	NvU8 finger_separation_flick_time;			//0x4E
} synaptics_ts_sensor_data;

typedef struct {
	NvU32 X_position[SYNAPTICS_FINGER_MAX];
	NvU32 Y_position[SYNAPTICS_FINGER_MAX];
	NvU8 width[SYNAPTICS_FINGER_MAX];
	NvU8 pressure[SYNAPTICS_FINGER_MAX];
} synaptics_ts_finger_data;


typedef struct Synaptics_TouchDeviceRec
{
    NvOdmTouchDevice OdmTouch;
    NvOdmTouchCapabilities Caps;
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
    NvBool PowerOn;
    NvU32 CoreVddId;
	NvU32 I2cVddId;
	NvU8 FirmwareRevId;
    NvU32 I2cClockSpeedKHz;
// 20100929  Ghost finger solution = touch f/w [START]
	NvU8 MeltType;
// 20100929  Ghost finger solution = touch f/w [START]
} Synaptics_TouchDevice;


static const NvOdmTouchCapabilities Synaptics_Capabilities =
{
	1,	//IsMultiTouchSupported
	2,	//MaxNumberOfFingerCoordReported;
	0,	//IsRelativeDataSupported
	1,	//MaxNumberOfRelativeCoordReported
	15,	//MaxNumberOfWidthReported
	0xFF,	//MaxNumberOfPressureReported
	NvOdmTouchGesture_Not_Supported,	// Gesture
	1,	//IsWidthSupported
	1,	//IsPressureSupported, mandatory for multi-touch
	1,	//IsFingersSupported
	0,	//XMinPosition
	0,	//YMinPosition
	0,	//XMaxPosition
	0,	//YMaxPosition
#if SYNAPTICS_SCREEN_ANGLE_MODE
	0,
#else
	(NvU32)(NvOdmTouchOrientation_V_FLIP | NvOdmTouchOrientation_XY_SWAP)
#endif    
};


static synaptics_ts_sensor_data ts_reg_data={0};
static synaptics_ts_finger_data curr_ts_data;
static synaptics_ts_finger_data prev_ts_data;

// 20101223  improve ghost finger avoid algorithm [START]
static int synaptics_ts_melting_count = 0;
static NvBool synaptics_ts_melting_enable = NV_TRUE;

static NvU32 synaptics_ts_melting_check_time;

static NvBool synaptics_ts_first_finger_pressed = NV_FALSE;
// 20101223  improve ghost finger avoid algorithm [END]



////////////////////////////////////////////////////////////////////
//////////////////    Fucntions for Synaptics    ///////////////////
////////////////////////////////////////////////////////////////////

// setting register
static NvBool Synaptics_WriteRegister (Synaptics_TouchDevice* hTouch, NvU8 reg, NvU8 val)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;
	NvU8 arr[2];

	arr[0] = reg;		// register address
	arr[1] = val;		// 

	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = arr;
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
		printk("[TOUCH] I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, hTouch->DeviceAddr, reg, val);
		return NV_FALSE;
	}

	return NV_TRUE;
}


// setting register
static NvBool Synaptics_WriteRegisterMulti(Synaptics_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;
	NvU8 *arr;

	arr = (NvU8*)NvOdmOsAlloc(len+1);
	if(arr ==NULL)
	{
		printk("[TOUCH] cannot alloc memory (len=%d)\n", len);
		return NV_FALSE;
	}
	arr[0] = reg;	
	NvOdmOsMemcpy(&arr[1], buffer, len);// register address

	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = arr;
	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.NumBytes = len + 1;

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
		printk("[TOUCH] I2C Write Failure = %d (addr=0x%x, reg=0x%x)\n", Error, hTouch->DeviceAddr, reg);
		return NV_FALSE;
	}

	return NV_TRUE;
}


// set register address before read
static NvBool Synaptics_SetRegAddr (Synaptics_TouchDevice* hTouch, NvU8 reg)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;

	// set register address
	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = (NvU8*)&reg;
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
		printk("[TOUCH] I2C Set Addr Failure = %d (addr=0x%x, reg=0x%x)\n", Error, hTouch->DeviceAddr, reg);
		return NV_FALSE;
	}

	return NV_TRUE;
}


static NvBool Synaptics_ReadRegisterSafe (Synaptics_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
	int i=0;
	NvOdmI2cStatus Error = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;

	if(Synaptics_SetRegAddr(hTouch, reg) == NV_FALSE)
		return NV_FALSE;

	TransactionInfo.Address = hTouch->DeviceAddr | 0x1;	//read
	TransactionInfo.Buf = buffer;
	TransactionInfo.Flags = 0;
	TransactionInfo.NumBytes = len;

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
		printk("[TOUCH] I2C Read Failure (Transaction) = %d (addr=0x%x)\n", Error, hTouch->DeviceAddr);
		return NV_FALSE;
	}

	return NV_TRUE;
}


static NvBool Synaptics_Configure (Synaptics_TouchDevice* hTouch)
{
    hTouch->SleepMode = 0x0;
    hTouch->SampleRate = 0; /* this forces register write */
    return Synaptics_SetSampleRate(&hTouch->OdmTouch, SYNAPTICS_HIGH_SAMPLE_RATE);
}


static void Synaptics_GpioIsr(void *arg)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)arg;
	NVODMTOUCH_PRINTF(("[TOUCH] Synaptics_GpioIsr\n"));

    /* Signal the touch thread to read the sample. After it is done reading the
        * sample it should re-enable the interrupt. */
    NvOdmOsSemaphoreSignal(hTouch->hIntSema);
}


static NvBool Synaptics_GetSamples (Synaptics_TouchDevice* hTouch, NvOdmTouchCoordinateInfo* coord)
{
	NvU8 i;

	NVODMTOUCH_PRINTF(("[TOUCH] Synaptics_GetSamples [START]\n"));

	if (!Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_DATA_BASE_REG, (NvU8*)(&ts_reg_data), sizeof(ts_reg_data)))
	{
		NVODMTOUCH_PRINTF(("[Touch Driver] Error read register info using I2C\n"));
		return NV_FALSE;
	}
    
	NVODMTOUCH_PRINTF(("[TOUCH] ===Synaptics Touch === INT mode[0x%02x]\n", ts_reg_data.interrupt_status_reg));

	coord->additionalInfo.Fingers = 0;
	coord->fingerstate = NvOdmTouchSampleIgnore;
	coord->additionalInfo.Gesture = NvOdmTouchGesture_No_Gesture;


	if(ts_reg_data.interrupt_status_reg & SYNAPTICS_INT_ABS0)
	{
		for(i = 0; i < SYNAPTICS_FINGER_MAX; i++)
		{
			int check = 1 << ((i%4)*2);
			if((ts_reg_data.finger_state_reg[i/4] & check) == check)
			{
				curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.fingers_data[i][0], ts_reg_data.fingers_data[i][2]);
				curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.fingers_data[i][1], ts_reg_data.fingers_data[i][2]);
				//  2011/06/22 GB bug start
				if(curr_ts_data.X_position[i] == 0)
					curr_ts_data.X_position[i] = 1;
				if(curr_ts_data.Y_position[i] == 0)
					curr_ts_data.Y_position[i] = 1;
				//  2011/06/22 end    
				if ((((ts_reg_data.fingers_data[i][3] & 0xf0) >> 4) - (ts_reg_data.fingers_data[i][3] & 0x0f)) > 0)
					curr_ts_data.width[i] = (ts_reg_data.fingers_data[i][3] & 0xf0) >> 4;
				else
					curr_ts_data.width[i] = ts_reg_data.fingers_data[i][3] & 0x0f;

				curr_ts_data.pressure[i] = ts_reg_data.fingers_data[i][4];
				
				prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
				prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
				prev_ts_data.width[i] = curr_ts_data.width[i];
				prev_ts_data.pressure[i] = curr_ts_data.pressure[i];

				coord->additionalInfo.Fingers++;
				coord->additionalInfo.multi_fingerstate[i] = 1;

				coord->fingerstate = NvOdmTouchSampleValidFlag;
				coord->fingerstate |= NvOdmTouchSampleDownFlag;
				coord->xcoord = curr_ts_data.X_position[i];
				coord->ycoord = curr_ts_data.Y_position[i];
				coord->additionalInfo.multi_XYCoords[i][0] = curr_ts_data.X_position[i];
				coord->additionalInfo.multi_XYCoords[i][1] = curr_ts_data.Y_position[i];
				coord->additionalInfo.width[i] = curr_ts_data.width[i];
				coord->additionalInfo.Pressure[i] = curr_ts_data.pressure[i];
				NVODMTOUCH_PRINTF(("[TOUCH] Synaptics :: %dth Finger Press: X=%d, Y=%d, W=0x%02x, Pressure=%d\n", i+1, curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.width[i], curr_ts_data.pressure[i]));
			}
			else
			{
				if(coord->additionalInfo.multi_fingerstate[i])
				{
					coord->additionalInfo.multi_fingerstate[i] = 0;
					coord->fingerstate = NvOdmTouchSampleValidFlag;

					NVODMTOUCH_PRINTF(("[TOUCH] Synaptics :: %dth Finger Release: X=%d, Y=%d\n", i+1, curr_ts_data.X_position[i], curr_ts_data.Y_position[i]));
				}
			}
		}
// 20101223  improve ghost finger avoidance algorithm [START]
		if(hTouch->FirmwareRevId >= SYNAPTICS_MELT_SUPPORT_VER && hTouch->MeltType == SYNAPTICS_MELT_MODE)
		{
			if((coord->additionalInfo.Fingers > 1) || (coord->additionalInfo.Fingers == 1 && coord->additionalInfo.multi_fingerstate[0] == 0))
				synaptics_ts_melting_enable = NV_FALSE;
			
			if(coord->additionalInfo.Fingers == 0 && synaptics_ts_melting_enable && synaptics_ts_first_finger_pressed)
			{
				if((NvOdmOsGetTimeMS() - synaptics_ts_melting_check_time) < 2000) //2000ms
				{
					if(synaptics_ts_melting_count++ >= 3)
					{
						if(Synaptics_WriteRegister(hTouch, SYNAPTICS_MELT_CONTROL_REG, SYNAPTICS_NO_MELT_MODE))
						{
							NvU8 melt_mode = SYNAPTICS_MELT_MODE;
							Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_MELT_CONTROL_REG, (NvU8*)(&melt_mode), sizeof(melt_mode));
							hTouch->MeltType = melt_mode;
							printk("[TOUCH] melt type : %d\n", hTouch->MeltType);
						}
					}

					printk("[TOUCH] synaptics_ts_melting_count : %d\n", synaptics_ts_melting_count);
				}
			}

			if(coord->additionalInfo.Fingers == 0)
				synaptics_ts_melting_enable = NV_TRUE;

			if(coord->additionalInfo.multi_fingerstate[0])
			{
				if(!synaptics_ts_first_finger_pressed)
				{
					synaptics_ts_melting_check_time = NvOdmOsGetTimeMS();
					synaptics_ts_first_finger_pressed = NV_TRUE;
				}
			}
			else
				synaptics_ts_first_finger_pressed = NV_FALSE;
		}
// 20101223  improve ghost finger avoidance algorithm [END]
	}

	//NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_GetSamples [END]\n"));

	return NV_TRUE;
}


// 20100906  for Touch Firmware Version Check [START]
static NvBool Synaptics_GetFWVersion(Synaptics_TouchDevice* hTouch)
{
	NvU8 RMI_Query_BaseAddr;
	NvU8 FWVersion_Addr;

	hTouch->FirmwareRevId = 0x00;
	
	if(!Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_RMI_QUERY_BASE_REG, (NvU8*)(&RMI_Query_BaseAddr), sizeof(RMI_Query_BaseAddr)))
		return NV_FALSE;

	FWVersion_Addr = RMI_Query_BaseAddr+3;
	
	if(Synaptics_ReadRegisterSafe (hTouch, FWVersion_Addr, (NvU8*)(&hTouch->FirmwareRevId), sizeof(hTouch->FirmwareRevId)))
	{
		printk("[TOUCH] Touch controller Firmware Version = %x\n", hTouch->FirmwareRevId);
// 20100906  Touch F/W version [START]
		storeTouchFWversion(hTouch->FirmwareRevId);
// 20100906  Touch F/W version [END]
		return NV_TRUE;
	}
	else
		return NV_FALSE;
}
// 20100906  for Touch Firmware Version Check [END]



// 20100413  for Touch Firmware Upgrade [START]
#ifdef SYNAPTICS_SUPPORT_FW_UPGRADE
static unsigned long ExtractLongFromHeader(const NvU8 *SynaImage)  // Endian agnostic 
{
  return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}

static void CalculateChecksum(NvU16 *data, NvU16 len, NvU32 *dataBlock)
{
  unsigned long temp = *data++;
  unsigned long sum1;
  unsigned long sum2;

  *dataBlock = 0xffffffff;

  sum1 = *dataBlock & 0xFFFF;
  sum2 = *dataBlock >> 16;

  while (len--)
  {
    sum1 += temp;    
    sum2 += sum1;    
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  *dataBlock = sum2 << 16 | sum1;
}

static void SpecialCopyEndianAgnostic(NvU8 *dest, NvU16 src) 
{
  dest[0] = src%0x100;  //Endian agnostic method
  dest[1] = src/0x100;  
}


static NvBool Synaptics_UpgradeFirmware(Synaptics_TouchDevice* hTouch)
{
	int i;

	int try_count = 0;
	int try_limit = 2000;

	NvU8 FlashQueryBaseAddr, FlashDataBaseAddr;
	NvU8 RMICommandBaseAddr;
	
	NvU8 BootloaderIDAddr;
	NvU8 BlockSizeAddr;
	NvU8 FirmwareBlockCountAddr;
	NvU8 ConfigBlockCountAddr;

	NvU8 BlockNumAddr;
	NvU8 BlockDataStartAddr;
	
	NvU8 bootloader_id[2];

	NvU8 temp_array[2], temp_data, flashValue, m_firmwareImgVersion;
	NvU8 checkSumCode;

	NvU16 ts_block_size, ts_config_block_count, ts_fw_block_count;
	NvU16 m_bootloadImgID;
	
	NvU32 ts_config_img_size;
	NvU32 ts_fw_img_size;
	NvU32 pinValue, m_fileSize, m_firmwareImgSize, m_configImgSize, m_FirmwareImgFile_checkSum;

	////////////////////////////////////////////////////////////////////////////////////

	printk("[Touch Driver] Synaptics_UpgradeFirmware for only TM1576 [START]\n");


	////////////////////////	Product ID Check	///////////////////////////
	NvU8 RMI_Query_BaseAddr;
	NvU8 product_id_addr;

	NvU8 product_id[7];

	Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_RMI_QUERY_BASE_REG, (NvU8*)(&RMI_Query_BaseAddr), sizeof(RMI_Query_BaseAddr));

	product_id_addr = RMI_Query_BaseAddr+11;

	Synaptics_ReadRegisterSafe (hTouch, product_id_addr, (NvU8*)(&product_id), 6);
	product_id[6] = '\0';
	printk("[Touch Driver] Touch controller Product ID = %s\n", product_id);
	
	if(NvOsStrncmp(product_id, &SynapticsFirmware[0x10], 6) != 0)
	{
		printk("[Touch Driver] Synaptics_UpgradeFirmware can not progress!!!!\n");
		return NV_TRUE;
	}


	////////////////////////	F/W Version Check	///////////////////////////
// 20101129  Touch F/W upgrade is supported for Rev 1.0 or later for P999 [START]
#if defined(TMUS_10) || defined(CONFIG_MACH_STAR_REV_F)
	Synaptics_GetFWVersion(hTouch);

	if((hTouch->FirmwareRevId >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) || (hTouch->FirmwareRevId < 0x64 && SynapticsFirmware[0x1F] < 0x64))
	{
		if(!(hTouch->FirmwareRevId < SynapticsFirmware[0x1F]))
		{
			printk("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
			return NV_TRUE;
		}
	}
#else
	printk("[Touch Driver] Synaptics_UpgradeFirmware : Do not support anymore under Rev 1.0!!!!\n");
	return NV_TRUE;
#endif
// 20101129  Touch F/W upgrade is supported for Rev 1.0 or later for P999 [END]


	////////////////////////	Configuration	///////////////////////////
	// Address Configuration
	if(!Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_QUERY_BASE_REG, (NvU8*)(&FlashQueryBaseAddr), sizeof(FlashQueryBaseAddr)))
		return NV_FALSE;

	printk("[TOUCH] Synaptics_UpgradeFirmware ::FlashQueryBaseAddr %02x\n", FlashQueryBaseAddr);

	BootloaderIDAddr = FlashQueryBaseAddr;
	BlockSizeAddr = FlashQueryBaseAddr + 3;
	FirmwareBlockCountAddr = FlashQueryBaseAddr + 5;
	ConfigBlockCountAddr = FlashQueryBaseAddr + 7;
	
	if(!Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_DATA_BASE_REG, (NvU8*)(&FlashDataBaseAddr), sizeof(FlashDataBaseAddr)))
		return NV_FALSE;

	BlockNumAddr = FlashDataBaseAddr;
	BlockDataStartAddr = FlashDataBaseAddr + 2;


	// Get New Firmware Information from Header
	m_fileSize = sizeof(SynapticsFirmware) -1;

	checkSumCode         = ExtractLongFromHeader(&(SynapticsFirmware[0]));
	m_bootloadImgID      = (unsigned int)SynapticsFirmware[4] + (unsigned int)SynapticsFirmware[5]*0x100;
	m_firmwareImgVersion = SynapticsFirmware[7]; 
	m_firmwareImgSize    = ExtractLongFromHeader(&(SynapticsFirmware[8]));
	m_configImgSize      = ExtractLongFromHeader(&(SynapticsFirmware[12]));    

	CalculateChecksum((NvU16*)&(SynapticsFirmware[4]), (NvU16)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);

	// Get Current Firmware Information
	Synaptics_ReadRegisterSafe(hTouch, BlockSizeAddr, (NvU8*)&temp_array[0], 2);
	printk("[TOUCH] Synaptics_UpgradeFirmware :: BlockSizeAddr %02x %02x\n", temp_array[0], temp_array[1]);
	ts_block_size = temp_array[0] + (temp_array[1] << 8);
	
	Synaptics_ReadRegisterSafe(hTouch, FirmwareBlockCountAddr, (NvU8*)&temp_array[0], 2);
	printk("[TOUCH] Synaptics_UpgradeFirmware :: FirmwareBlockCountAddr %02x %02x\n", temp_array[0], temp_array[1]);
	ts_fw_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_fw_img_size = ts_block_size * ts_fw_block_count;
	
	Synaptics_ReadRegisterSafe(hTouch, ConfigBlockCountAddr, (NvU8*)&temp_array[0], 2);
	printk("[TOUCH] Synaptics_UpgradeFirmware :: ConfigBlockCountAddr %02x %02x\n", temp_array[0], temp_array[1]);
	ts_config_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_config_img_size = ts_block_size * ts_config_block_count;

	Synaptics_ReadRegisterSafe(hTouch, BootloaderIDAddr, (NvU8*)&bootloader_id[0], 2);
	printk("[TOUCH] Synaptics_UpgradeFirmware :: BootloaderID %02x %02x\n", bootloader_id[0], bootloader_id[1]);

	// Compare
	if (m_fileSize != (0x100+m_firmwareImgSize+m_configImgSize))
	{
		printk("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid FileSize\n");
		return NV_TRUE;
	}

	if (m_firmwareImgSize != ts_fw_img_size)
	{
		printk("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Firmware Image Size\n");
		return NV_TRUE;
	}

	if (m_configImgSize != ts_config_img_size)
	{
		printk("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Config Image Size\n");
		return NV_TRUE;
	}

	if(m_firmwareImgVersion == 0 && ((unsigned int)bootloader_id[0] + (unsigned int)bootloader_id[1]*0x100) != m_bootloadImgID)
	{
		printk("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Bootload Image\n");
		return NV_TRUE;
	}


	////////////////////////	Flash Command - Enable	///////////////////////////
	// Write Flash command Key(Write bootload ID)
	Synaptics_WriteRegisterMulti(hTouch, BlockDataStartAddr, &bootloader_id[0], 2);

	try_count = 0;
	do
	{
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
		NvOdmOsWaitUS(1000);
		try_count++;
	} while(((flashValue & 0x0f) != 0x00) && (try_count <= try_limit));
	if(try_count > try_limit)
	{
		return NV_FALSE;
	}

	// Issue Enable flash command
	Synaptics_WriteRegister(hTouch, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ENABLE);	// enable

	// Wait for ATTN assertion and see if it's idle and flash enabled
	try_count = 0;
	do
	{
		NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
		NvOdmOsWaitUS(1000);
		try_count++;
	} while(pinValue && (try_count <= try_limit));
	if(try_count > try_limit)
	{
		return NV_FALSE;
	}

	try_count = 0;
	do
	{
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
		NvOdmOsWaitUS(1000);
		try_count++;
	} while((flashValue) != 0x80 && (try_count <= try_limit));
	if(try_count > try_limit)
	{
		return NV_FALSE;
	}

	printk("[TOUCH] Synaptics_UpgradeFirmware :: Flash Program Enable Setup Complete\n");


	////////////////////////	Flash Command  - Eraseall	///////////////////////////
	// Write Flash command Key(Write bootload ID)
	Synaptics_WriteRegisterMulti(hTouch, BlockDataStartAddr, &bootloader_id[0], 2);

	// Issue the firmware and configuration erase command
	Synaptics_WriteRegister(hTouch, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ERASEALL);

	// Wait for ATTN assertion and see if it's idle and flash enabled
	try_count = 0;
	do
	{
		NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
		NvOdmOsWaitUS(1000);
		try_count++;
	} while(pinValue && (try_count <= try_limit));
	if(try_count > try_limit)
	{
		return NV_FALSE;
	}

	try_count = 0;
	do
	{
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
		NvOdmOsWaitUS(1000);
		try_count++;
	} while((flashValue) != 0x80 && (try_count <= try_limit));
	if(try_count > try_limit)
	{
		return NV_FALSE;
	}
	
	printk("[TOUCH] Synaptics_UpgradeFirmware :: Flash Erase Complete\n");


	////////////////////////	F/W Data Write	///////////////////////////
	// Flash Firmware Data Write
	for(i = 0; i < ts_fw_block_count; ++i)
	{
		temp_array[0] = i & 0xff;
		temp_array[1] = (i & 0xff00) >> 8;

		// Write Block Number
		Synaptics_WriteRegisterMulti(hTouch, BlockNumAddr, &temp_array[0], 2);

		// Write Data Block&SynapticsFirmware[0]
		Synaptics_WriteRegisterMulti(hTouch, BlockDataStartAddr, (NvU8 *)&SynapticsFirmware[0x100+i*ts_block_size], ts_block_size);

		// Issue Write Firmware Block command
		Synaptics_WriteRegister(hTouch, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_FW_WRITE);
		
		// Wait for ATTN assertion and see if it's idle and flash enabled
		try_count = 0;
		do
		{
			NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while(pinValue && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}

		try_count = 0;
		do
		{
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while((flashValue) != 0x80 && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}
	} //for
	
	printk("[TOUCH] Synaptics_UpgradeFirmware :: Flash Firmware Write Complete\n");


	////////////////////////	F/W Config Write	///////////////////////////
	for(i = 0; i < ts_config_block_count; i++)
	{
		SpecialCopyEndianAgnostic(&temp_array[0], i);

		// Write Configuration Block Number
		Synaptics_WriteRegisterMulti(hTouch, BlockNumAddr, &temp_array[0], 2);

		// Write Data Block
		Synaptics_WriteRegisterMulti(hTouch, BlockDataStartAddr, (NvU8 *)&SynapticsFirmware[0x100+m_firmwareImgSize+i*ts_block_size], ts_block_size);

		// Issue Write Configuration Block command to flash command register
		Synaptics_WriteRegister(hTouch, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_CONFIG_WRITE);

		// Wait for ATTN assertion and see if it's idle and flash enabled
		try_count = 0;
		do
		{
			NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while(pinValue && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}

		try_count = 0;
		do
		{
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while((flashValue) != 0x80 && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}
	}
	
	printk("[TOUCH] Synaptics_UpgradeFirmware :: Flash Config Write Complete\n");


	////////////////////////	Reset Touch IC	///////////////////////////
	if(Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_RMI_CMD_BASE_REG, (NvU8*)(&RMICommandBaseAddr), sizeof(RMICommandBaseAddr)))
	{
		// S/W reset
		Synaptics_WriteRegister(hTouch, RMICommandBaseAddr, 0x01);
		NvOdmOsWaitUS(200000);

		try_count = 0;
		do
		{
			NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while(pinValue && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}
		
		try_count = 0;
		do
		{
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_FLASH_CONTROL_REG, (NvU8*)(&flashValue), sizeof(flashValue));
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);
			NvOdmOsWaitUS(1000);
			try_count++;
		} while(((flashValue & 0x0f) != 0x00) && (try_count <= try_limit));
		if(try_count > try_limit)
		{
			return NV_FALSE;
		}

		// Clear the attention assertion by reading the interrupt status register
		Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_INT_STATUS_REG, (NvU8 *)&temp_data, 1);

		// Read F01 Status flash prog, ensure the 6th bit is '0'
		do
		{
			Synaptics_ReadRegisterSafe(hTouch, SYNAPTICS_DATA_BASE_REG, (NvU8 *)&temp_data, 1);
		} while((temp_data & 0x40) != 0);
	}
	else
	{
		// H/W reset
		if(Synaptics_PowerOnOff(&hTouch->OdmTouch, 0) == NV_FALSE)
			return NV_FALSE;

		if(Synaptics_PowerOnOff(&hTouch->OdmTouch, 1) == NV_FALSE)
			return NV_FALSE;
	}

	
	return NV_TRUE;
}
#endif
// 20100413  for Touch Firmware Upgrade [END]




static void Synaptics_InitOdmTouch (NvOdmTouchDevice* Dev)
{
    Dev->Close              = Synaptics_Close;
    Dev->GetCapabilities    = Synaptics_GetCapabilities;
    Dev->ReadCoordinate     = Synaptics_ReadCoordinate;
    Dev->EnableInterrupt    = Synaptics_EnableInterrupt;
    Dev->HandleInterrupt    = Synaptics_HandleInterrupt;
    Dev->GetSampleRate      = Synaptics_GetSampleRate;
    Dev->SetSampleRate      = Synaptics_SetSampleRate;
    Dev->PowerControl       = Synaptics_PowerControl;
    Dev->PowerOnOff         = Synaptics_PowerOnOff;
#ifdef SYNAPTICS_SUPPORT_CAL
    Dev->GetCalibrationData = Synaptics_GetCalibrationData;
#else
    Dev->GetCalibrationData = NULL;
#endif /* SYNAPTICS_SUPPORT_CAL */
	Dev->CurrentSampleRate = SYNAPTICS_HIGH_SAMPLE_RATE;
    Dev->OutputDebugMessage = NV_FALSE;
// 20101020  Interrupt Enable/Disable [START]
	Dev->InterruptMask = Synaptics_InterruptMask;
// 20101020  Interrupt Enable/Disable [END]
}


static NvBool Synaptics_Init(Synaptics_TouchDevice* hTouch)
{
	int i;

	if(hTouch == NULL)
	{
		NVODMTOUCH_PRINTF(("[TOUCH] Synaptics Driver Initialize Fail\n"));
		return NV_FALSE;
	}

	for(i = 0; i < SYNAPTICS_FINGER_MAX; i++)
	{
		prev_ts_data.X_position[i] = curr_ts_data.X_position[i] = 0;
		prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i] = 0;
		prev_ts_data.width[i] = curr_ts_data.width[i] = 0;
		prev_ts_data.pressure[i] = curr_ts_data.pressure[i] = 0;
	}

// 20100929  Ghost finger solution = touch f/w [START]
	if(hTouch->FirmwareRevId >= SYNAPTICS_MELT_SUPPORT_VER)
	{
		NvU8 melt_mode = SYNAPTICS_MELT_MODE;
		Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_MELT_CONTROL_REG, (NvU8*)(&melt_mode), sizeof(melt_mode));
		hTouch->MeltType = melt_mode;
	}
	else
		hTouch->MeltType = SYNAPTICS_MELT_MODE;
// 20100929  Ghost finger solution = touch f/w [END]

	Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_DATA_BASE_REG, (NvU8*)(&ts_reg_data), sizeof(ts_reg_data));

// 20100402  Touch Register Setting [START]
	Synaptics_WriteRegister(hTouch, SYNAPTICS_DELTA_X_THRES_REG, SYNAPTICS_DELTA_THRESHOLD);	// Delta X
	Synaptics_WriteRegister(hTouch, SYNAPTICS_DELTA_Y_THRES_REG, SYNAPTICS_DELTA_THRESHOLD);	// Delta Y
// 20100402  Touch Register Setting [END]

// 20101228  improve ghost finger avoidance algorithm [START]
	Synaptics_WriteRegister(hTouch, 0x5B, 0x00);		// 2d gesture enable1 = not use
	Synaptics_WriteRegister(hTouch, 0x5C, 0x00);		// 2d gesture enable2 = not use
// 20101228  improve ghost finger avoidance algorithm [END]

// 20110120  change to continuous mode by appl. request [START]
	Synaptics_WriteRegister(hTouch, 0x51, 0x08);
// 20110120  change to continuous mode by appl. request [END]

	return NV_TRUE;
}


NvBool Synaptics_Open (NvOdmTouchDeviceHandle* hDevice, NvOdmOsSemaphoreHandle* hIntSema)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)0;
    NvU32 i;
    NvU32 found = 0;

	NvU16 SENSOR_MAX_X_POSITION = LGE_TOUCH_RESOLUTION_X;
    NvU16 SENSOR_MAX_Y_POSITION = LGE_TOUCH_RESOLUTION_Y;  

    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

	NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_Open\n"));

    hTouch = NvOdmOsAlloc(sizeof(Synaptics_TouchDevice));
    if (!hTouch) return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(Synaptics_TouchDevice));

    /* set function pointers */
    Synaptics_InitOdmTouch(&hTouch->OdmTouch);

    pConnectivity = NvOdmPeripheralGetGuid(SYNAPTICS_TOUCH_DEVICE_GUID);
    if (!pConnectivity)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : pConnectivity is NULL Error \n"));
        goto fail;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : didn't find any periperal in discovery query for touch device Error \n"));
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
				NVODMTOUCH_PRINTF(("[TOUCH] i2c address = %x.\n", hTouch->DeviceAddr));
                break;
				
            case NvOdmIoModule_Gpio:
                hTouch->GpioPort = pConnectivity->AddressList[i].Instance;
                hTouch->GpioPin = pConnectivity->AddressList[i].Address;
                found |= 2;
                break;
				
            case NvOdmIoModule_Vdd:
                hTouch->CoreVddId = pConnectivity->AddressList[i].Address;
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
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : peripheral connectivity problem \n"));
        goto fail;
    }

    if ((found & 4) != 0)
    {
    	hTouch->PowerOn = 1;
        if (NV_FALSE == Synaptics_PowerOnOff(&hTouch->OdmTouch, 0))
	    	goto fail;   
    }
    else
    {
        NVODMTOUCH_PRINTF(("[TOUCH] Synaptics Power fail \n"));
        hTouch->CoreVddId = 0xFF; 
    }

    hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, hTouch->I2CInstance);
    if (!hTouch->hOdmI2c)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : NvOdmI2cOpen Error \n"));
        goto fail;
    }

    hTouch->hGpio = NvOdmGpioOpen();

    if (!hTouch->hGpio)
    {        
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : NvOdmGpioOpen Error \n"));
        goto fail;
    }

    NVODMTOUCH_PRINTF(("[TOUCH] gpio port = %d. gpio pin = %d\n", hTouch->GpioPort, hTouch->GpioPin));

    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, hTouch->GpioPort, hTouch->GpioPin);
    if (!hTouch->hPin)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : Couldn't get GPIO pin \n"));
        goto fail;
    }

    NvOdmGpioConfig(hTouch->hGpio,
                    hTouch->hPin,
                    NvOdmGpioPinMode_Output);

	NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 1);

    if (NV_FALSE == Synaptics_PowerOnOff(&hTouch->OdmTouch, 1))
            goto fail;

    NvOdmGpioConfig(hTouch->hGpio,
                    hTouch->hPin,
                    NvOdmGpioPinMode_InputData);

    /* set default capabilities */
    NvOdmOsMemcpy(&hTouch->Caps, &Synaptics_Capabilities, sizeof(NvOdmTouchCapabilities));

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = SYNAPTICS_I2C_SPEED_KHZ;
    NVODMTOUCH_PRINTF(("[TOUCH] i2c speed = %d\n", hTouch->I2cClockSpeedKHz));
	
// 20100413  for Touch Firmware Upgrade [START]
#ifdef SYNAPTICS_SUPPORT_FW_UPGRADE
	if(Synaptics_UpgradeFirmware(hTouch) == NV_FALSE)
		goto fail;
#endif
// 20100413  for Touch Firmware Upgrade [END]

	/* get chip revision id */
    Synaptics_GetFWVersion(hTouch);

    /* initialize */
    if(!Synaptics_Init(hTouch))	 goto fail;

    /* get max positions */
    /* There is no SMBus Aliased Address to query max position, change page to 0x10 */ 
    hTouch->Caps.XMaxPosition = (NvU32)SENSOR_MAX_X_POSITION;
    hTouch->Caps.YMaxPosition = (NvU32)SENSOR_MAX_Y_POSITION;
    
    /* configure panel */
    if (!Synaptics_Configure(hTouch)) goto fail;

	/* Register Interrupt */
    *hDevice = &hTouch->OdmTouch;
    if (Synaptics_EnableInterrupt(*hDevice, *hIntSema) == NV_FALSE)
          goto fail;
	
    return NV_TRUE;

 fail:
    Synaptics_Close(&hTouch->OdmTouch);
    return NV_FALSE;
}


void Synaptics_Close (NvOdmTouchDeviceHandle hDevice)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;
	NVODMTOUCH_PRINTF(("[Touch Driver] Synaptics_Close\n"));

    if (!hTouch) return;

	Synaptics_PowerOnOff(&hTouch->OdmTouch, NV_FALSE);
        
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

// 20101020  Interrupt Enable/Disable [START]
void Synaptics_InterruptMask(NvOdmTouchDeviceHandle hDevice, NvBool mask)
{
	/*
		If mask is NV_TRUE, then disable irq.
		If mask is NV_FALSE, then enable irq.
	*/
	Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;
	
	if (hTouch->hGpioIntr)
	{
		NvOdmGpioInterruptMask(hTouch->hGpioIntr, mask);
		printk("[TOUCH] Synaptics_InterruptMask by %d\n", mask);
	}
}
// 20101020  Interrupt Enable/Disable [END]



NvBool Synaptics_EnableInterrupt (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema)
{
	Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;

	NV_ASSERT(hIntSema);

	NVODMTOUCH_PRINTF(("[TOUCH] Synaptics_EnableInterrupt\n"));

	/* can only be initialized once */
	if (hTouch->hGpioIntr || hTouch->hIntSema)
		return NV_FALSE;

	hTouch->hIntSema = hIntSema;    

	if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
		hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow, Synaptics_GpioIsr,
		(void*)hTouch, SYNAPTICS_DEBOUNCE_TIME_MS) == NV_FALSE)
	{
		NVODMTOUCH_PRINTF(("[TOUCH] cannot register interrupt.\n"));
		return NV_FALSE;
	}

	if (!hTouch->hGpioIntr)
		return NV_FALSE;    

	return NV_TRUE;
}


NvBool Synaptics_HandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;

	NVODMTOUCH_PRINTF(("[TOUCH] Synaptics_HandleInterrupt\n"));

	NvOdmGpioInterruptDone(hTouch->hGpioIntr);

    return NV_TRUE;
}


NvBool Synaptics_ReadCoordinate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo* coord)
{
	Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;

	NVODMTOUCH_PRINTF(("[TOUCH] Synaptics_ReadCoordinate\n"));

	if (Synaptics_GetSamples(hTouch, coord)==NV_FALSE)
		return NV_FALSE;

	return NV_TRUE;
}


void Synaptics_GetCapabilities (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;
    *pCapabilities = hTouch->Caps;
}


NvBool Synaptics_GetSampleRate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate)
{
    //Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;
    pTouchSampleRate->NvOdmTouchSampleRateHigh = 80;
    pTouchSampleRate->NvOdmTouchSampleRateLow = 40;
    pTouchSampleRate->NvOdmTouchCurrentSampleRate = 80;
    return NV_TRUE;
}


NvBool Synaptics_SetSampleRate (NvOdmTouchDeviceHandle hDevice, NvU32 rate)
{
/*
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;

    if (rate != 0 && rate != 1)
        return NV_FALSE;
    
    rate = 1 << rate;
    
    if (hTouch->SampleRate == rate)
        return NV_TRUE;
    
    hTouch->SampleRate = rate;
*/

    return NV_TRUE;
}


NvBool Synaptics_PowerOnOff (NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;

    hTouch->hPmu = NvOdmServicesPmuOpen();

	printk("[TOUCH] Synaptics_PowerOnOff\n");
	
    if (!hTouch->hPmu)
    {
		printk("[TOUCH] NvOdmServicesPmuOpen Error\n");
		return NV_FALSE;
    }
    
    if (OnOff != hTouch->PowerOn)
    {
        NvOdmServicesPmuVddRailCapabilities vddrailcapCore;
		NvOdmServicesPmuVddRailCapabilities vddrailcapI2c;
        NvU32 settletime;

        NvOdmServicesPmuGetCapabilities( hTouch->hPmu, hTouch->CoreVddId, &vddrailcapCore);
		NvOdmServicesPmuGetCapabilities( hTouch->hPmu, hTouch->I2cVddId, &vddrailcapI2c);

		printk("[TOUCH] Core power on[%d], vol[%d]\n", OnOff, vddrailcapCore.requestMilliVolts);
		printk("[TOUCH] I2C power on[%d], vol[%d]\n", OnOff, vddrailcapI2c.requestMilliVolts);
		
        if(OnOff)
		{
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->CoreVddId, vddrailcapCore.requestMilliVolts, &settletime);
        	NvOdmOsWaitUS(SYNAPTICS_POR_DELAY_MS*1000); // wait to settle power
			NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->I2cVddId, vddrailcapI2c.requestMilliVolts, &settletime);
		}
        else
		{
			NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->I2cVddId, NVODM_VOLTAGE_OFF, &settletime);
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->CoreVddId, NVODM_VOLTAGE_OFF, &settletime);
		}

		if (settletime)
			NvOdmOsWaitUS(settletime); // wait to settle power

	NvOdmOsWaitUS(10*1000); //10ms
        hTouch->PowerOn = OnOff;
    }

    NvOdmServicesPmuClose(hTouch->hPmu);

    return NV_TRUE;
}


NvBool Synaptics_PowerControl (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    Synaptics_TouchDevice* hTouch = (Synaptics_TouchDevice*)hDevice;
    NvU8 SleepMode;
	NvU8 DeviceControl;

// 20100920  ESD Issue [START]
	NvU8 configValueX;
	NvU8 configValueY;
// 20100920  ESD Issue [END]

	if (!hTouch)
		return NV_FALSE;

	printk("[TOUCH] Synaptics_PowerControl\n");
    
    switch(mode)
    {
		// Normal Operation
		case NvOdmTouch_PowerMode_0:
		case NvOdmTouch_PowerMode_1:
		case NvOdmTouch_PowerMode_2:
			SleepMode = SYNAPTICS_DEVICE_NORMAL_OPERATION;
			break;
		// Sensor Sleep
		case NvOdmTouch_PowerMode_3:
			SleepMode = SYNAPTICS_DEVICE_SENSOR_SLEEP;
			break;
		default:
			SleepMode = hTouch->SleepMode;
			break;
    }

	if (hTouch->SleepMode == SleepMode)
		return NV_TRUE;
	else
		hTouch->SleepMode = (NvU32)SleepMode;

// 20100920  ESD Issue [START]
	if(mode == NvOdmTouch_PowerMode_0)
	{
		Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_DELTA_X_THRES_REG, (NvU8*)(&configValueX), sizeof(configValueX));
		printk("[TOUCH] Synaptics Delta X threshold %x\n", configValueX);
		if(configValueX != SYNAPTICS_DELTA_THRESHOLD)
			return NV_FALSE;

		Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_DELTA_Y_THRES_REG, (NvU8*)(&configValueY), sizeof(configValueY));
		printk("[TOUCH] Synaptics Delta Y threshold %x\n", configValueY);
		if(configValueY != SYNAPTICS_DELTA_THRESHOLD)
			return NV_FALSE;
	}
// 20100920  ESD Issue [END]

	if(Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_DEVICE_CONTROL_REG, (NvU8*)(&DeviceControl), sizeof(DeviceControl)))
		printk("Synaptics Device Control Read Success\n");
	if(Synaptics_WriteRegister(hTouch, SYNAPTICS_DEVICE_CONTROL_REG, ((DeviceControl & 0xFC) | SleepMode)))
		printk("Synaptics Device Control Write Success\n");

// 20101223  improve ghost finger avoidance algorithm [START]
	if(!SleepMode && hTouch->FirmwareRevId >= SYNAPTICS_MELT_SUPPORT_VER)
	{
		if(Synaptics_WriteRegister(hTouch, SYNAPTICS_MELT_CONTROL_REG, SYNAPTICS_MELT_MODE))
		{
			NvU8 melt_mode = SYNAPTICS_NO_MELT_MODE;
			Synaptics_ReadRegisterSafe (hTouch, SYNAPTICS_MELT_CONTROL_REG, (NvU8*)(&melt_mode), sizeof(melt_mode));
			hTouch->MeltType = melt_mode;
			printk("[TOUCH] melt type : %d\n", hTouch->MeltType);
			synaptics_ts_melting_count = 0;
			synaptics_ts_melting_enable = NV_TRUE;
		}
	}
// 20101223  improve ghost finger avoidance algorithm [END]
    
    return NV_TRUE;
}


#ifdef SYNAPTICS_SUPPORT_CAL
NvBool Synaptics_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer)
{
#if SYNAPTICS_SCREEN_ANGLE
    //Portrait
    static const NvS32 RawCoordBuffer[] = {2054, 3624, 3937, 809, 3832, 6546, 453, 6528, 231, 890};
#else
    //Landscape
    static NvS32 RawCoordBuffer[] = {2054, 3624, 3832, 6546, 453, 6528, 231, 890, 3937, 809};
#endif

    if (NumOfCalibrationData*2 != (sizeof(RawCoordBuffer)/sizeof(NvS32)))
    {
        NVODMTOUCH_PRINTF(("[TOUCH] WARNING: number of calibration data isn't matched\n"));
        return NV_FALSE;
    }
    
    NvOdmOsMemcpy(pRawCoordBuffer, RawCoordBuffer, sizeof(RawCoordBuffer));

    return NV_TRUE;
}
#endif

