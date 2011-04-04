/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


           TOUCH DEVICE DRIVER FUNCTIONS

GENERAL DESCRIPTION

  CYPRESS TMA300 Touch Device ODM Driver

EXTERNALIZED FUNCTIONS



INITIALIZATION AND SEQUENCING REQUIREMENTS

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================
                        EDIT HISTORY FOR MODULE

Date			Author				Descriptions
----------		-----------			------------------------------------
2010/03/12		taewan.kim			code create (basic operation implement)
2010/04/27		joseph.jung			multi touch implement
2010/05/17		joseph.jung			power control function is changed
2010/06/30		joseph.jung			adapt for designated star panel
2010/06/30		joseph.jung			I/O power control implement for Rev.B


===========================================================================*/


/*===========================================================================

                     INCLUDE FILES FOR ODM DRIVER

===========================================================================*/


#include "nvodm_touch_int.h"
#include "nvodm_services.h"
#include "nvodm_touch_cypress.h"
#include "nvodm_query_discovery.h"

#include "../../adaptations/pmu/max8907/max8907_supply_info_table.h"



////////////////////////////////////////////////////////////////////
/////////////       Cypress Environment Define       ///////////////
////////////////////////////////////////////////////////////////////


#define CYPRESS_TOUCH_DEVICE_GUID				NV_ODM_GUID('c','y','p','t','o','u','c','h')

#define CYPRESS_I2C_SPEED_KHZ					400
#define CYPRESS_I2C_TIMEOUT						500
#define CYPRESS_LOW_SAMPLE_RATE					0		//40 reports per-second
#define CYPRESS_HIGH_SAMPLE_RATE				1		//80 reports per-second

#define CYPRESS_SCREEN_ANGLE_MODE				1		//0=Landscape, 1=Portrait

#define CYPRESS_POR_DELAY_MS					100		//Dealy after Power-On Reset

#define CYPRESS_DEBOUNCE_TIME_MS				0

#define CYPRESS_FINGER_MAX						4

#define CYPRESS_MODE_BOOTLOADER					0		// default mode
#define CYPRESS_MODE_OPERATION					1

#undef CYPRESS_SUPPORT_FW_UPGRADE
#undef CYPRESS_SUPPORT_CAL


////////////////////////////////////////////////////////////////////
///////////////        Cypress EVENT Define       //////////////////
////////////////////////////////////////////////////////////////////

#define CYPRESS_MOVEMENT_THRESHOLD				0//3

////////////////////////////////////////////////////////////////////
///////////////  Cypress Control & Data Register  //////////////////
////////////////////////////////////////////////////////////////////

#define CYPRESS_BL_FILE_REG						0x00

#define CYPRESS_HST_MODE_REG					0x00
#define CYPRESS_TT_STAT_REG						0x02


////////////////////////////////////////////////////////////////////
//////////////////  Cypress Data from Register  ////////////////////
////////////////////////////////////////////////////////////////////


#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x100) + (int)(low_reg))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x100) + (int)(low_reg))

////////////////////////////////////////////////////////////////////
/////////////////   Data Structure for Cypress   ///////////////////
////////////////////////////////////////////////////////////////////

typedef struct
{
#if 1
	NvU8 tt_stat;					// 0x02
	
	NvU8 touch_1_X_high_position;	// 0x03
	NvU8 touch_1_X_low_position;	// 0x04
	NvU8 touch_1_Y_high_position;	// 0x05
	NvU8 touch_1_Y_low_position;	// 0x06
	NvU8 touch_1_Z_value;			// 0x07
	
	NvU8 touch_1_2_ID;				// 0x08
	
	NvU8 touch_2_X_high_position;	// 0x09
	NvU8 touch_2_X_low_position;	// 0x0A
	NvU8 touch_2_Y_high_position;	// 0x0B
	NvU8 touch_2_Y_low_position;	// 0x0C
	NvU8 touch_2_Z_value;			// 0x0D

	NvU8 gesture_count;				// 0x0E
	NvU8 gesture_ID;				// 0x0F

	NvU8 touch_3_X_high_position;	// 0x10
	NvU8 touch_3_X_low_position;	// 0x11
	NvU8 touch_3_Y_high_position;	// 0x12
	NvU8 touch_3_Y_low_position;	// 0x13
	NvU8 touch_3_Z_value;			// 0x14

	NvU8 touch_3_4_ID;				// 0x15

	NvU8 touch_4_X_high_position;	// 0x16
	NvU8 touch_4_X_low_position;	// 0x17
	NvU8 touch_4_Y_high_position;	// 0x18
	NvU8 touch_4_Y_low_position;	// 0x19
	NvU8 touch_4_Z_value;			// 0x1A
#else
	NvU8 tt_stat;					// 0x02

	NvU8 touch_info[10][9];			// 0x03~0x5C
	/*
	touch_info[i][0]: ith x position high
	touch_info[i][1]: ith x position low
	touch_info[i][2]: ith y position high
	touch_info[i][3]: ith y position low
	touch_info[i][4]: ith z value (pressure)
	touch_info[i][5]: ith ID
	touch_info[i][6]: ith Gesture
	touch_info[i][7]: ith distance high
	touch_info[i][8]: ith distance low
	*/

	NvU8 hover_x_high;				// 0x5D
	NvU8 hover_x_low;				// 0x5E
	NvU8 hover_y_high;				// 0x5F
	NvU8 hover_y_low;				// 0x60
	NvU8 hover_gesture;				// 0x61

	NvU8 proximity_left_right;		// 0x62
	NvU8 proximity_top_bottom;		// 0x63

	NvU8 button[2];					// 0x64~0x65
#endif
} cypress_ts_sensor_data;


typedef	struct
{
	NvU8 finger_ID[CYPRESS_FINGER_MAX]; 
	NvU32 X_position[CYPRESS_FINGER_MAX];
	NvU32 Y_position[CYPRESS_FINGER_MAX];
	NvU8 pressure[CYPRESS_FINGER_MAX];
} cypress_ts_finger_data;


typedef struct Cypress_TouchDeviceRec
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
    NvU32 SampleRate;
    NvU8 SleepMode;
    NvBool PowerOn;
    NvU32 VddId;    
    NvU8 ChipFirmVersion;                          
    NvU32 I2cClockSpeedKHz;
} Cypress_TouchDevice;


static const NvOdmTouchCapabilities Cypress_Capabilities =
{
    1, //IsMultiTouchSupported
    2, //MaxNumberOfFingerCoordReported;
    0, //IsRelativeDataSupported
    1, //MaxNumberOfRelativeCoordReported
    0, //MaxNumberOfWidthReported
    0xFF, //MaxNumberOfPressureReported
    (NvU32)NvOdmTouchGesture_Not_Supported, //Gesture
    0, //IsWidthSupported
    1, //IsPressureSupported, mandatory for multi-touch
    1, //IsFingersSupported
    0, //XMinPosition
    0, //YMinPosition
    0, //XMaxPosition
    0, //YMaxPosition
#if CYPRESS_SCREEN_ANGLE_MODE
    0,
#else
   (NvU32)(NvOdmTouchOrientation_V_FLIP | NvOdmTouchOrientation_XY_SWAP)
#endif    
};


static cypress_ts_sensor_data ts_reg_data={0};
static cypress_ts_finger_data curr_ts_data;
static cypress_ts_finger_data prev_ts_data;


////////////////////////////////////////////////////////////////////
/////////////     Basic Local Functions Definition    //////////////
////////////////////////////////////////////////////////////////////

static NvBool Cypress_WriteRegister (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8 val);
static NvBool Cypress_WriteRegisterMulti(Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len);
static NvBool Cypress_SetRegAddr (Cypress_TouchDevice* hTouch, NvU8 reg);
static NvBool Cypress_ReadRegisterOnce (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len);
static NvBool Cypress_ReadRegisterSafe (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len);
static NvBool Cypress_Configure (Cypress_TouchDevice* hTouch);
static void Cypress_GpioIsr(void *arg);
static NvBool Cypress_GetSamples (Cypress_TouchDevice* hTouch, NvOdmTouchCoordinateInfo* coord);

static void Cypress_InitOdmTouch (NvOdmTouchDevice* Dev);
static NvBool Cypress_Init(Cypress_TouchDevice* hTouch);

static void Cypress_ModeChange(Cypress_TouchDevice* hTouch, NvU8 mode);



////////////////////////////////////////////////////////////////////
//////////////////     Fucntions for Cypress     ///////////////////
////////////////////////////////////////////////////////////////////


///////////////////////   Local Functions   ////////////////////////

// setting register
static NvBool Cypress_WriteRegister (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8 val)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];

    arr[0] = reg;		// register address
    arr[1] = val;		// u16 value (lsb-msb)
    
    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;
    
    do
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    CYPRESS_I2C_SPEED_KHZ,
                                    CYPRESS_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout); 

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, hTouch->DeviceAddr, reg, val));
        return NV_FALSE;
    }
    return NV_TRUE;
}


// setting register
static NvBool Cypress_WriteRegisterMulti(Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 *arr;

    arr = (NvU8*)NvOdmOsAlloc(len+1);
    if(arr ==NULL)
    {
    	NVODMTOUCH_PRINTF(("[TOUCH] cannot alloc memory (len=%d)\n",len));
		return NV_FALSE;
    }
    arr[0] = reg;	
    NvOdmOsMemcpy(&arr[1], buffer, len);// register address
    
    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len + 1;
    
    do
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    CYPRESS_I2C_SPEED_KHZ,
                                    CYPRESS_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout); 
    
    NvOdmOsFree(arr);
    
    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] I2C Write Failure = %d (addr=0x%x, reg=0x%x)\n", Error, hTouch->DeviceAddr, reg));
        return NV_FALSE;
    }
    
    return NV_TRUE;
}


// set register address before read
static NvBool Cypress_SetRegAddr (Cypress_TouchDevice* hTouch, NvU8 reg)
{
	NvOdmI2cTransactionInfo TransactionInfo;
	NvOdmI2cStatus Error;
	// set register address
	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = (NvU8*)&reg;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

	do
	{
		Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    CYPRESS_I2C_SPEED_KHZ,
                                    CYPRESS_I2C_TIMEOUT);
	} while (Error == NvOdmI2cStatus_Timeout); 

	if (Error != NvOdmI2cStatus_Success)
	{
		NVODMTOUCH_PRINTF(("[TOUCH] I2C Write Failure = %d (addr=0x%x, reg=0x%x)\n", Error, hTouch->DeviceAddr, reg));
		return NV_FALSE;
	}
	return NV_TRUE;
}


static NvBool Cypress_ReadRegisterOnce (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
	NvOdmI2cStatus Error;
	NvOdmI2cTransactionInfo TransactionInfo;

	if(Cypress_SetRegAddr(hTouch, reg) == NV_FALSE)
		return NV_FALSE;
        
    TransactionInfo.Address = hTouch->DeviceAddr | 0x1;	//read
    TransactionInfo.Buf = buffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

	do
	{
    	Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,		// transactioncnt
                                CYPRESS_I2C_SPEED_KHZ,
                                CYPRESS_I2C_TIMEOUT);
	} while (Error == NvOdmI2cStatus_Timeout);
	

	if (Error != NvOdmI2cStatus_Success)
	{
		NVODMTOUCH_PRINTF(("[TOUCH] I2C Read Failure = %d (addr=0x%x)\n", Error, hTouch->DeviceAddr));
		return NV_FALSE;
	}

	return NV_TRUE;
}


static NvBool Cypress_ReadRegisterSafe (Cypress_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
    if (!Cypress_ReadRegisterOnce(hTouch, reg, buffer, len))
        return NV_FALSE;

    return NV_TRUE;
}


static NvBool Cypress_Configure (Cypress_TouchDevice* hTouch)
{
    hTouch->SleepMode = 0x0;
    hTouch->SampleRate = 0; /* this forces register write */
    return Cypress_SetSampleRate(&hTouch->OdmTouch, CYPRESS_HIGH_SAMPLE_RATE);
}


static void Cypress_GpioIsr(void *arg)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)arg;

    NVODMTOUCH_PRINTF(("[TOUCH] Cypress_GpioIsr\n"));

	/* Signal the touch thread to read the sample. After it is done reading the
	* sample it should re-enable the interrupt. */
	NvOdmOsSemaphoreSignal(hTouch->hIntSema);
	
	NvOdmGpioInterruptDone(hTouch->hGpioIntr);
}


static NvBool Cypress_GetSamples (Cypress_TouchDevice* hTouch, NvOdmTouchCoordinateInfo* coord)
{
	int i;

	NVODMTOUCH_PRINTF(("[TOUCH] Cypress_GetSamples\n"));

	coord->additionalInfo.Fingers = 0;
	coord->fingerstate = NvOdmTouchSampleIgnore;

	coord->additionalInfo.Gesture = NvOdmTouchGesture_No_Gesture;

	if (!Cypress_ReadRegisterSafe (hTouch, CYPRESS_TT_STAT_REG, (NvU8*)(&ts_reg_data), sizeof(ts_reg_data)))
	{
		NVODMTOUCH_PRINTF(("[TOUCH] Error read register info using I2C\n"));
       	return NV_FALSE;
	}

	NVODMTOUCH_PRINTF(("[TOUCH] Cypress :: TT_STAT 0x%02x\n", ts_reg_data.tt_stat));

	coord->additionalInfo.Fingers = ts_reg_data.tt_stat & 0x0F;	
	NVODMTOUCH_PRINTF(("[TOUCH] Cypress :: finger count=[%d]\n", coord->additionalInfo.Fingers));

	if(coord->additionalInfo.Fingers > 0)
	{
		// press
		i = 0;
		curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.touch_1_X_high_position, ts_reg_data.touch_1_X_low_position);
		curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.touch_1_Y_high_position, ts_reg_data.touch_1_Y_low_position);

		curr_ts_data.pressure[i] = ts_reg_data.touch_1_Z_value;
		curr_ts_data.finger_ID[i] = (ts_reg_data.touch_1_2_ID >> 4);

		if(coord->additionalInfo.multi_fingerstate[i] == 0 || (abs(prev_ts_data.X_position[i] - curr_ts_data.X_position[i]) > CYPRESS_MOVEMENT_THRESHOLD || abs(prev_ts_data.Y_position[i] - curr_ts_data.Y_position[i]) > CYPRESS_MOVEMENT_THRESHOLD))
		{
			prev_ts_data.finger_ID[i] = curr_ts_data.finger_ID[i];
			prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
			prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
			prev_ts_data.pressure[i] = curr_ts_data.pressure[i];

			coord->additionalInfo.multi_fingerstate[i] = 1;
		
			coord->fingerstate = NvOdmTouchSampleValidFlag;
			coord->fingerstate |= NvOdmTouchSampleDownFlag;
			coord->additionalInfo.multi_XYCoords[i][0] = curr_ts_data.X_position[i];
			coord->additionalInfo.multi_XYCoords[i][1] = curr_ts_data.Y_position[i];
			coord->additionalInfo.Pressure[i] = curr_ts_data.pressure[i];
			NVODMTOUCH_PRINTF(("[TOUCH] ===Cypress Touch === First Finger Press Data: X=%d, Y=%d, Pressure=%d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i]));
		}
	}

	if (coord->additionalInfo.Fingers > 1)
	{
		// press
		i = 1;
		curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.touch_2_X_high_position, ts_reg_data.touch_2_X_low_position);
		curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.touch_2_Y_high_position, ts_reg_data.touch_2_Y_low_position);

		curr_ts_data.pressure[i] = ts_reg_data.touch_2_Z_value;
		curr_ts_data.finger_ID[i] = ((ts_reg_data.touch_1_2_ID << 4) >> 4);

		if(coord->additionalInfo.multi_fingerstate[i] == 0 || (abs(prev_ts_data.X_position[i] - curr_ts_data.X_position[i]) > CYPRESS_MOVEMENT_THRESHOLD || abs(prev_ts_data.Y_position[i] - curr_ts_data.Y_position[i]) > CYPRESS_MOVEMENT_THRESHOLD))
		{
			prev_ts_data.finger_ID[i] = curr_ts_data.finger_ID[i];
			prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
			prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
			prev_ts_data.pressure[i] = curr_ts_data.pressure[i];

			coord->additionalInfo.multi_fingerstate[i] = 1;
		
			coord->fingerstate = NvOdmTouchSampleValidFlag;
			coord->fingerstate |= NvOdmTouchSampleDownFlag;
			coord->additionalInfo.multi_XYCoords[i][0] = curr_ts_data.X_position[i];
			coord->additionalInfo.multi_XYCoords[i][1] = curr_ts_data.Y_position[i];
			coord->additionalInfo.Pressure[i] = curr_ts_data.pressure[i];
			NVODMTOUCH_PRINTF(("[TOUCH] ===Cypress Touch === Second Finger Press Data: X=%d, Y=%d, Pressure=%d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i]));
		}
	}

	if (coord->additionalInfo.Fingers > 2)
	{
		// press
		i = 2;
		curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.touch_3_X_high_position, ts_reg_data.touch_3_X_low_position);
		curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.touch_3_Y_high_position, ts_reg_data.touch_3_Y_low_position);

		curr_ts_data.pressure[i] = ts_reg_data.touch_3_Z_value;
		curr_ts_data.finger_ID[i] = (ts_reg_data.touch_3_4_ID >> 4);

		if(coord->additionalInfo.multi_fingerstate[i] == 0 || (abs(prev_ts_data.X_position[i] - curr_ts_data.X_position[i]) > CYPRESS_MOVEMENT_THRESHOLD || abs(prev_ts_data.Y_position[i] - curr_ts_data.Y_position[i]) > CYPRESS_MOVEMENT_THRESHOLD))
		{
			prev_ts_data.finger_ID[i] = curr_ts_data.finger_ID[i];
			prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
			prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
			prev_ts_data.pressure[i] = curr_ts_data.pressure[i];

			coord->additionalInfo.multi_fingerstate[i] = 1;
		
			coord->fingerstate = NvOdmTouchSampleValidFlag;
			coord->fingerstate |= NvOdmTouchSampleDownFlag;
			coord->additionalInfo.multi_XYCoords[i][0] = curr_ts_data.X_position[i];
			coord->additionalInfo.multi_XYCoords[i][1] = curr_ts_data.Y_position[i];
			coord->additionalInfo.Pressure[i] = curr_ts_data.pressure[i];
			NVODMTOUCH_PRINTF(("[TOUCH] ===Cypress Touch === Third Finger Press Data: X=%d, Y=%d, Pressure=%d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i]));
		}
	}

	if (coord->additionalInfo.Fingers > 3)
	{
		// press
		i = 3;
		curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.touch_4_X_high_position, ts_reg_data.touch_4_X_low_position);
		curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.touch_4_Y_high_position, ts_reg_data.touch_4_Y_low_position);

		curr_ts_data.pressure[i] = ts_reg_data.touch_3_Z_value;
		curr_ts_data.finger_ID[i] = ((ts_reg_data.touch_3_4_ID << 4) >> 4);

		if(coord->additionalInfo.multi_fingerstate[i] == 0 || (abs(prev_ts_data.X_position[i] - curr_ts_data.X_position[i]) > CYPRESS_MOVEMENT_THRESHOLD || abs(prev_ts_data.Y_position[i] - curr_ts_data.Y_position[i]) > CYPRESS_MOVEMENT_THRESHOLD))
		{
			prev_ts_data.finger_ID[i] = curr_ts_data.finger_ID[i];
			prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
			prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
			prev_ts_data.pressure[i] = curr_ts_data.pressure[i];

			coord->additionalInfo.multi_fingerstate[i] = 1;
		
			coord->fingerstate = NvOdmTouchSampleValidFlag;
			coord->fingerstate |= NvOdmTouchSampleDownFlag;
			coord->additionalInfo.multi_XYCoords[i][0] = curr_ts_data.X_position[i];
			coord->additionalInfo.multi_XYCoords[i][1] = curr_ts_data.Y_position[i];
			coord->additionalInfo.Pressure[i] = curr_ts_data.pressure[i];
			NVODMTOUCH_PRINTF(("[TOUCH] ===Cypress Touch === Fourth Finger Press Data: X=%d, Y=%d, Pressure=%d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i]));
		}
	}

	for(i = coord->additionalInfo.Fingers; i < LGE_SUPPORT_FINGERS_NUM; i++)
	{
		if(coord->additionalInfo.multi_fingerstate[i])
		{
			coord->additionalInfo.multi_fingerstate[i] = 0;
			coord->fingerstate = NvOdmTouchSampleValidFlag;

			NVODMTOUCH_PRINTF(("[TOUCH] Cypress :: %dth Finger Release: X=%d, Y=%d\n", i+1, curr_ts_data.X_position[i], curr_ts_data.Y_position[i]));
		}
	}
	
	return NV_TRUE;
}


#ifdef CYPRESS_SUPPORT_FW_UPGRADE
static NvBool Cypress_CheckNeedUpgrade(NvU16 curVersion)
{
	return NV_TRUE;
}

static NvBool Cypress_UpgradeFirmware(Cypress_TouchDevice* hTouch)
{
	int i;
	NvU32 position = 0;

	Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x00);
	Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 10);
	position += 10;

	for(i = 0; i < 470; i++)
	{
		Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x00);
		Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 16);
		position += 16;

		Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x10);
		Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 16);
		position += 16;

		Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x20);
		Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 16);
		position += 16;

		Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x30);
		Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 16);
		position += 16;

		Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x40);
		Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 14);
		position += 14;
	}

	Cypress_WriteRegister(hTouch, CYPRESS_BL_FILE_REG, 0x00);
	Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, (NvU8 *)&CypressFirmware[position], 10);
	position += 10;
	
	return NV_TRUE;
}
#endif /* CYPRESS_SUPPORT_FW_UPGRADE */


static void Cypress_InitOdmTouch (NvOdmTouchDevice* Dev)
{
    Dev->Close              = Cypress_Close;
    Dev->GetCapabilities    = Cypress_GetCapabilities;
    Dev->ReadCoordinate     = Cypress_ReadCoordinate;
    Dev->EnableInterrupt    = Cypress_EnableInterrupt;
    Dev->HandleInterrupt    = Cypress_HandleInterrupt;
    Dev->GetSampleRate      = Cypress_GetSampleRate;
    Dev->SetSampleRate      = Cypress_SetSampleRate;
    Dev->PowerControl       = Cypress_PowerControl;
    Dev->PowerOnOff         = Cypress_PowerOnOff;
#ifdef CYPRESS_SUPPORT_CAL
    Dev->GetCalibrationData = Cypress_GetCalibrationData;
#else
	Dev->GetCalibrationData = NULL;
#endif /* CYPRESS_SUPPORT_CAL */
	Dev->CurrentSampleRate = CYPRESS_HIGH_SAMPLE_RATE;
    Dev->OutputDebugMessage = NV_FALSE;
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [START]
	Dev->InterruptMask = Cypress_InterruptMask;
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [END]
}


static NvBool Cypress_Init(Cypress_TouchDevice* hTouch)
{
	int i;

	if(hTouch == NULL)
	{
		NVODMTOUCH_PRINTF(("[Touch Driver] Cypress Driver Initialize Fail\n"));
		return NV_FALSE;
	}

	for(i = 0; i < CYPRESS_FINGER_MAX; i++)
	{
		prev_ts_data.finger_ID[i] = curr_ts_data.finger_ID[i] = 0;
		prev_ts_data.X_position[i] = curr_ts_data.X_position[i] = 0;
		prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i] = 0;
		prev_ts_data.pressure[i] = curr_ts_data.pressure[i] = 0;
	}
	
	return NV_TRUE;
}



static void Cypress_ModeChange(Cypress_TouchDevice* hTouch, NvU8 mode)
{
	NvU8 mode_status;
	
	NvU8 bootloader_mode[11] = {0x00, 0xFF, 0x38, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	NvU8 operation_mode[11] = {0x00, 0xFF, 0xA5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	
	switch(mode)
	{
		case CYPRESS_MODE_BOOTLOADER:
			Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, &bootloader_mode[0], 11);
			break;
		case CYPRESS_MODE_OPERATION:
			Cypress_WriteRegisterMulti(hTouch, CYPRESS_BL_FILE_REG, &operation_mode[0], 11);
			break;
	}
}


///////////////////////   Global Functions   ////////////////////////

NvBool Cypress_Open (NvOdmTouchDeviceHandle* hDevice, NvOdmOsSemaphoreHandle* hIntSema)
{
    Cypress_TouchDevice* hTouch;
    NvU32 i;
    NvU32 found = 0;
    NvU32 GpioPort = 0;
    NvU32 GpioPin = 0;
    NvU32 I2cInstance = 0;
    NvU8 CypressFirmVersion = 0x01;
	
	NvU16 SENSOR_MAX_X_POSITION = LGE_TOUCH_RESOLUTION_X-1;
    NvU16 SENSOR_MAX_Y_POSITION = LGE_TOUCH_RESOLUTION_Y-1;  

    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    NVODMTOUCH_PRINTF(("[TOUCH] Cypress_Open\n"));

    hTouch = NvOdmOsAlloc(sizeof(Cypress_TouchDevice));
    if (!hTouch) return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(Cypress_TouchDevice));

    /* set function pointers */
    Cypress_InitOdmTouch(&hTouch->OdmTouch);

    pConnectivity = NvOdmPeripheralGetGuid(CYPRESS_TOUCH_DEVICE_GUID);
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
                I2cInstance = pConnectivity->AddressList[i].Instance;
                found |= 1;

				NVODMTOUCH_PRINTF(("[TOUCH] i2c address = %x.\n", hTouch->DeviceAddr));
                break;
            case NvOdmIoModule_Gpio:
                GpioPort = pConnectivity->AddressList[i].Instance;
                GpioPin = pConnectivity->AddressList[i].Address;
                found |= 2;
                break;
            case NvOdmIoModule_Vdd:
                hTouch->VddId = pConnectivity->AddressList[i].Address;
                found |= 4;
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
		if (NV_FALSE == Cypress_PowerOnOff(&hTouch->OdmTouch, 0))
			goto fail;
    }
    else
    {
        hTouch->VddId = 0xFF; 
    }

    hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
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

    NVODMTOUCH_PRINTF(("[TOUCH] gpio port = %d. gpio pin = %d\n", GpioPort, GpioPin));

    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort, GpioPin);
    if (!hTouch->hPin)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : Couldn't get GPIO pin \n"));
        goto fail;
    }

	NvOdmGpioConfig(hTouch->hGpio,
					hTouch->hPin,
					NvOdmGpioPinMode_Output);

	NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 1);

	if (NV_FALSE == Cypress_PowerOnOff(&hTouch->OdmTouch, 1))
		goto fail;

	Cypress_ModeChange(hTouch, CYPRESS_MODE_OPERATION);

	*hDevice = &hTouch->OdmTouch;
	if (Cypress_EnableInterrupt(*hDevice, *hIntSema) == NV_FALSE)
		goto fail;

	NvOdmGpioConfig(hTouch->hGpio,
					hTouch->hPin,
					NvOdmGpioPinMode_InputData);

    /* set default capabilities */
    NvOdmOsMemcpy(&hTouch->Caps, &Cypress_Capabilities, sizeof(NvOdmTouchCapabilities));

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = CYPRESS_I2C_SPEED_KHZ;

    NVODMTOUCH_PRINTF(("[TOUCH] i2c speed = %d\n", hTouch->I2cClockSpeedKHz));

	// TODO : read firmware version
	hTouch->ChipFirmVersion = CypressFirmVersion;
	NVODMTOUCH_PRINTF(("[TOUCH] Touch controller Firmware Revision ID = %x\n", hTouch->ChipFirmVersion));
// 20100906 joseph.jung@lge.com Touch F/W version [START]
	storeTouchFWversion(hTouch->ChipFirmVersion);
// 20100906 joseph.jung@lge.com Touch F/W version [END]


    /* get chip revision id */

	// TODO : upgrade check

    /* initialize */
    if(!Cypress_Init(hTouch))	 goto fail;

    /* get max positions */
    /* There is no SMBus Aliased Address to query max position, change page to 0x10 */ 
    hTouch->Caps.XMaxPosition = (NvU32)SENSOR_MAX_X_POSITION;
    hTouch->Caps.YMaxPosition = (NvU32)SENSOR_MAX_Y_POSITION;
    
    
    /* configure panel */
    if (!Cypress_Configure(hTouch)) goto fail;

    return NV_TRUE;

 fail:
    Cypress_Close(&hTouch->OdmTouch);
    return NV_FALSE;
}


void Cypress_Close (NvOdmTouchDeviceHandle hDevice)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;

    if (!hTouch) return;
        
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


// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [START]
void Cypress_InterruptMask(NvOdmTouchDeviceHandle hDevice, NvBool mask)
{
	/*
		If mask is NV_TRUE, then disable irq.
		If mask is NV_FALSE, then enable irq.
	*/
	Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;
	
	if (hTouch->hGpioIntr)
	{
		NvOdmGpioInterruptMask(hTouch->hGpioIntr, mask);
		printk("[TOUCH] Cypress_InterruptMask by %d\n", mask);
	}
}
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [END]



NvBool Cypress_EnableInterrupt (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;

    NV_ASSERT(hIntSema);

    NVODMTOUCH_PRINTF(("[TOUCH] Cypress_EnableInterrupt\n"));

    /* can only be initialized once */
    if (hTouch->hGpioIntr || hTouch->hIntSema)
    	return NV_FALSE;

    hTouch->hIntSema = hIntSema;    
    	
    if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
        hTouch->hPin, NvOdmGpioPinMode_InputInterruptFallingEdge, Cypress_GpioIsr,
        (void*)hTouch, CYPRESS_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
		NVODMTOUCH_PRINTF(("[TOUCH] cannot register interrupt.\n"));
        return NV_FALSE;
    }

    if (!hTouch->hGpioIntr)
        return NV_FALSE;    

    return NV_TRUE;
}


NvBool Cypress_HandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
#if 0
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;
    NvU32 pinValue;
    
    NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
    if (!pinValue)
    {
		NVODMTOUCH_PRINTF(("[TOUCH] interrupt pin is still LOW, read data until interrupt pin is released.\n"));
        return NV_FALSE;
    }
    else
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);
#endif //0
    return NV_TRUE;
}


NvBool Cypress_ReadCoordinate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo* coord)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;

    NVODMTOUCH_PRINTF(("[TOUCH] Cypress_ReadCoordinate\n"));

	if (Cypress_GetSamples(hTouch, coord)==NV_FALSE)
		return NV_FALSE;
    
    return NV_TRUE;
}


void Cypress_GetCapabilities (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;
    *pCapabilities = hTouch->Caps;
}


NvBool Cypress_GetSampleRate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;
    pTouchSampleRate->NvOdmTouchSampleRateHigh = 80;
    pTouchSampleRate->NvOdmTouchSampleRateLow = 40;
    pTouchSampleRate->NvOdmTouchCurrentSampleRate = (hTouch->SampleRate >> 1);
    return NV_TRUE;
}


NvBool Cypress_SetSampleRate (NvOdmTouchDeviceHandle hDevice, NvU32 rate)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;

    if (rate != 0 && rate != 1)
        return NV_FALSE;
    
    rate = 1 << rate;
    
    if (hTouch->SampleRate == rate)
        return NV_TRUE;
    
    hTouch->SampleRate = rate;
    return NV_TRUE;
}


NvBool Cypress_PowerOnOff (NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;

    hTouch->hPmu = NvOdmServicesPmuOpen();

    if (!hTouch->hPmu)
    {
        NVODMTOUCH_PRINTF(("[TOUCH] NvOdm Touch : NvOdmServicesPmuOpen Error \n"));
        return NV_FALSE;
    }
    
    if (OnOff != hTouch->PowerOn)
    {
        NvOdmServicesPmuVddRailCapabilities vddrailcap;
        NvU32 settletime;

        NvOdmServicesPmuGetCapabilities( hTouch->hPmu, hTouch->VddId, &vddrailcap);

		NVODMTOUCH_PRINTF(("[TOUCH] power on[%d], vol[%d]\n", OnOff, vddrailcap.requestMilliVolts));

        if(OnOff)
        {
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO10, MAX8907_REQUESTVOLTAGE_LDO10, &settletime);
        	NvOdmOsWaitUS(CYPRESS_POR_DELAY_MS*1000); // wait to settle power
			NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO19, MAX8907_REQUESTVOLTAGE_LDO19, &settletime);
        }
        else
        {
			NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO19, NVODM_VOLTAGE_OFF, &settletime);
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, Max8907PmuSupply_LDO10, NVODM_VOLTAGE_OFF, &settletime);
        }

        if (settletime)
            NvOdmOsWaitUS(settletime); // wait to settle power

        hTouch->PowerOn = OnOff;
    }

    NvOdmServicesPmuClose(hTouch->hPmu);

    return NV_TRUE;
}

NvBool Cypress_PowerControl (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    Cypress_TouchDevice* hTouch = (Cypress_TouchDevice*)hDevice;
    NvU8 SleepMode;
	NvU8 hst_mode;

    NV_ASSERT(hTouch->VddId != 0xFF);
    
    switch(mode)
    {
        case NvOdmTouch_PowerMode_0:
        case NvOdmTouch_PowerMode_1:
        case NvOdmTouch_PowerMode_2:
            SleepMode = 0x00;
            break;
        case NvOdmTouch_PowerMode_3:
            SleepMode = 0x02;
            break;
        default:
            return NV_FALSE;
    }

    if (hTouch->SleepMode == SleepMode)
        return NV_TRUE;

	if(SleepMode == 0x00)	// wake-up
	{
		printk("[TOUCH] wake-up\n");
		
		NvOdmGpioConfig(hTouch->hGpio,
						hTouch->hPin,
						NvOdmGpioPinMode_Output);

		NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 0);
		NvOdmOsWaitUS(10000);
		NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 1);
		NvOdmOsWaitUS(10000);

		NvOdmGpioConfig(hTouch->hGpio,
						hTouch->hPin,
						NvOdmGpioPinMode_InputData);

		// 20100824 joseph.jung@lge.com Some Panels are entered bootload mode when go to sleep
		Cypress_ModeChange(hTouch, CYPRESS_MODE_OPERATION);
	}
	else	// sleep
	{
		printk("[TOUCH] go to bed\n");
		Cypress_ReadRegisterSafe (hTouch, CYPRESS_HST_MODE_REG, (NvU8*)(&hst_mode), sizeof(hst_mode));
		Cypress_WriteRegister(hTouch, CYPRESS_HST_MODE_REG, ((hst_mode & 0xFD) | SleepMode));
	}
    
    hTouch->SleepMode = SleepMode;    
    return NV_TRUE;
}


#ifdef CYPRESS_SUPPORT_CAL
NvBool Cypress_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer)
{
#if CYPRESS_SCREEN_ANGLE_MODE
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

