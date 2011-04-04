#ifndef _LGD_LG2102_INCLUDES_H_
#define _LGD_LG2102_INCLUDES_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

typedef unsigned char			LGD_UINT8, 			*LGD_PUINT8;
typedef unsigned char			LGD_BYTE, 			*LGD_PBYTE;
typedef unsigned char			LGD_BOOL, 			*LGD_PBOOL;
typedef unsigned short			LGD_UINT16, 		*LGD_PUINT16;
typedef unsigned short			LGD_WORD, 			*LGD_PWORD;
typedef unsigned int			LGD_UINT32, 		*LGD_PUINT32;
typedef unsigned int			LGD_DWORD, 			*LGD_PDWORD;
typedef unsigned long			LGD_ULONG32, 		*LGD_PULONG32;

typedef char					LGD_INT8, 			*LGD_PINT8;
typedef char					LGD_CHAR, 			*LGD_PCHAR;
typedef short					LGD_INT16, 			*LGD_PINT16;
typedef int						LGD_INT32, 			*LGD_PINT32;
typedef long					LGD_LONG32, 		*LGD_PLONG32;
typedef unsigned long						LGD_DOUBLE32,		*LGD_PDOUBLE32;	//inb612 double link error by inb612

#ifndef FALSE
#define FALSE	0
#endif

#ifndef TRUE
#define TRUE	1
#endif

#define LGD_AUTO_RESYNC_ENABLE
#define LGD_TII_TEST_ENABLE	// by jin 09/0731

//#define LGD_MULTI_CHANNEL_FIC_UPLOAD	//inb612 for android single channel
//#define LGD_MULTI_HEADER_ENABLE		//inb612 for android single channel	
#define LGD_MULTI_MAX_CHANNEL			3
/************************************************************************/
/* FIFO Source 사용시                                                   */
/************************************************************************/
#define LGD_FIFO_SOURCE_ENABLE


/************************************************************************/
/* GPIO Source 사용시                                                   */
/************************************************************************/
//#define LGD_I2C_GPIO_CTRL_ENABLE

/************************************************************************/
/* DLS Source 사용시                                                    */
/************************************************************************/
//#define LGD_DLS_SOURCE_ENABLE

/************************************************************************/
/* USer Application type 사용시                                         */
/************************************************************************/
//#define USER_APPLICATION_TYPE

/************************************************************************/
/* EWS 사용시           			                                    */
/************************************************************************/
//#define LGD_EWS_SOURCE_ENABLE

#define LGD_MPI_INTERRUPT_ENABLE 					0x0001	//	MPI interrupt
#define LGD_I2C_INTERRUPT_ENABLE					0x0002	//	I2C interrupt
#define LGD_EWS_INTERRUPT_ENABLE					0x0040	//	EWS interrupt
#define LGD_REC_INTERRUPT_ENABLE					0x0080	//	Reconfigration interrupt
#define LGD_TII_INTERRUPT_ENABLE					0x0100	//	TII interrupt
#define LGD_FIC_INTERRUPT_ENABLE					0x0200	//	FIC Channel TX END interrupt
#define LGD_CIFS_INTERRUPT_ENABLE					0x0400	//	CIF Start Interrupt
#define LGD_FS_INTERRUPT_ENABLE						0x0800	//	Frame start interrupt
#define LGD_EXT_INTERRUPT_ENABLE					0x1000	//	External Interrupt
#define LGD_MS_INTERRUPT_ENABLE						0x2000	//	Modem Status interrupt
#define LGD_DPLLU_INTERRUPT_ENABLE					0x4000	//	DPLL unlock interrupt
#define LGD_SSI_INTERRUPT_ENABLE					0x8000	//	Signal strength indicator interrupt
#define LGD_DISABLE_INTERRUPT						0x0000

#define LGD_INTERRUPT_POLARITY_HIGH					0x0000
#define LGD_INTERRUPT_POLARITY_LOW					0x8000
#define LGD_INTERRUPT_PULSE							0x0000
#define LGD_INTERRUPT_LEVEL							0x4000
#define LGD_INTERRUPT_AUTOCLEAR_DISABLE				0x0000
#define LGD_INTERRUPT_AUTOCLEAR_ENABLE				0x2000
#define LGD_EXT_INTERRUPT_POLARITY_HIGH				0x0000
#define LGD_EXT_INTERRUPT_POLARITY_LOW				0x1000
#define LGD_INTERRUPT_PULSE_COUNT					0x00FF
#define LGD_INTERRUPT_PULSE_COUNT_MASK				0x03FF

#define LGD_ADDRESS_IFDELAY_LSB						0x58
#define LGD_ADDRESS_IFDELAY_MSB						0x59
#define LGD_ADDRESS_RFDELAY_LSB						0x53
#define LGD_ADDRESS_RFDELAY_MSB						0x54

#define LGD_AIRPLAY_IF_DLEAY_MAX					1000
#define LGD_AIRPLAY_RF_DLEAY_MAX					1100
#define LGD_SCAN_IF_DLEAY_MAX						400
#define LGD_SCAN_RF_DLEAY_MAX						1500


#define WORD_SWAP(X)			(((X)>>8&0xff)|(((X)<<8)&0xff00))
#define DWORD_SWAP(X)			(((X)>>24&0xff)|(((X)>>8)&0xff00)|(((X)<<8)&0xff0000)|(((X)<<24)&0xff000000))

#define LGD_REGISTER_CTRL(X)			((X)*0x1000)
#define STREAM_PARALLEL_ADDRESS			(0xfad00000)
#define STREAM_PARALLEL_ADDRESS_CS		(0x50000000)
#define LGD_TDMB_EBI_ADDRESS(X)			*(volatile LGD_UINT8*)(X)

#define FIC_REF_TIME_OUT				2500
#define LGD_SUCCESS						1
#define LGD_ERROR						0
#define LGD_NULL						0


#define LGD_CER_PERIOD_TIME				1000
#define LGD_CER_PERIOD					(3000 / LGD_CER_PERIOD_TIME)
#define LGD_BIT_PERIOD					(2000 / LGD_CER_PERIOD_TIME)

#define LGD_TDMB_LENGTH_MASK			0xFFF
#define TDMB_I2C_ID80					0x80
#define TDMB_I2C_ID82					0x82

#define MPI_CS_SIZE						(188*8)
#define LGD_INTERRUPT_SIZE				(188*16)	// MAX : 188*32
#define LGD_MPI_MAX_BUFF				LGD_INTERRUPT_SIZE
#define MAX_SUBCH_SIZE					32
#define MAX_SUBCHANNEL					64
#define MAX_LABEL_CHAR					16
#define SPI_INTERRUPT_SIZE				(188*8)

#define MAX_KOREABAND_FULL_CHANNEL		21
#define MAX_KOREABAND_NORMAL_CHANNEL	6
#define MAX_BAND_III_CHANNEL			41
#define MAX_L_BAND_CHANNEL				23
#define MAX_CHINA_CHANNEL				31
#define MAX_ROAMING_CHANNEL				12


#define RF500_REG_CTRL					105

#define APB_INT_BASE					0x0100
#define APB_GEN_BASE					0x0200
#define APB_PHY_BASE					0x0500
#define APB_DEINT_BASE					0x0600
#define APB_VTB_BASE					0x0700
#define APB_I2S_BASE					0x0800
#define APB_RDI_BASE					0x0900
#define APB_MPI_BASE					0x0A00
#define APB_RS_BASE						0x0B00
#define APB_SPI_BASE					0x0C00
#define APB_I2C_BASE					0x0D00
#define APB_RF_BASE						0x0E00

#define APB_FIC_BASE					0x1000
#define APB_STREAM_BASE					0x2000
#define TS_ERR_THRESHOLD				0x014C
#define LGD_ADC_MAX						18

#define INIT_FIC_DB08					0xff
#define INIT_FIC_DB16					0xffff
#define LGD_BIT_MASK											0x3f			//redefined BIT_MASK, so define LGD_BIT_MASK by inb612
#define END_MARKER						0xff
#define FIB_SIZE						32
#define FIB_WORD_SIZE					(FIB_SIZE/2)
#define MAX_FIB_NUM						12
#define MAX_FIC_SIZE					(MAX_FIB_NUM*FIB_SIZE)
#define MAX_FRAME_DURATION				96
#define MAX_USER_APP_DATA				32

typedef enum _tagTRANSMISSION
{
	TRANSMISSION_MODE1 = 1,
	TRANSMISSION_MODE2,
	TRANSMISSION_MODE3,
	TRANSMISSION_MODE4,
	TRANSMISSION_AUTO,
	TRANSMISSION_AUTOFAST,
}ST_TRANSMISSION, *PST_TRANSMISSION;

typedef enum _tagDPD_MODE
{
	LGD_DPD_OFF = 0,
	LGD_DPD_ON,
}LGD_DPD_MODE, *PLGD_DPD_MODE;

typedef enum _tagPLL_MODE
{
	INPUT_CLOCK_24576KHZ = 0,
	INPUT_CLOCK_12000KHZ,
	INPUT_CLOCK_19200KHZ,
	INPUT_CLOCK_27000KHZ,
}PLL_MODE, *PPLL_MODE;


typedef enum {
	EXTENSION_0	= 0,
	EXTENSION_1,
	EXTENSION_2,
	EXTENSION_3,
	EXTENSION_4,
	EXTENSION_5,
	EXTENSION_6,
	EXTENSION_7,
	EXTENSION_8,
	EXTENSION_9,
	EXTENSION_10,
	EXTENSION_11,
	EXTENSION_12,
	EXTENSION_13,
	EXTENSION_14,
	EXTENSION_15,
	EXTENSION_16,
	EXTENSION_17,
	EXTENSION_18,
	EXTENSION_19,
	EXTENSION_20,
	EXTENSION_21,
	EXTENSION_22,
	EXTENSION_23,
	EXTENSION_24,
}ST_EXTENSION_TYPE, *PST_EXTENSION_TYPE;

typedef enum 
{
	SPI_REGREAD_CMD	= 0,
	SPI_REGWRITE_CMD,
	SPI_MEMREAD_CMD,
	SPI_MEMWRITE_CMD,

}ST_SPI_CONTROL, *PST_SPI_CONTROL;

typedef enum{
	TMID_0 = 0,
	TMID_1,
	TMID_2,
	TMID_3,
}ST_TMID, *PST_TMID;

typedef enum{
	PROTECTION_LEVEL0 = 0,
	PROTECTION_LEVEL1,
	PROTECTION_LEVEL2,
	PROTECTION_LEVEL3,
}ST_PROTECTION_LEVEL, *PST_PROTECTION_LEVEL;

typedef enum{
	OPTION_INDICATE0 = 0,
	OPTION_INDICATE1,

}ST_INDICATE, *PST_INDICATE;


typedef enum{
	FIG_MCI_SI = 0,
	FIG_LABLE,
	FIG_RESERVED_0,
	FIG_RESERVED_1,
	FIG_RESERVED_2,
	FIG_FICDATA_CHANNEL,
	FIG_CONDITION_ACCESS,
	FIG_IN_HOUSE,	
}ST_FICHEADER_TYPE,*PST_FICHEADER_TYPE;


typedef enum{
	SIMPLE_FIC_ENABLE = 1,
	SIMPLE_FIC_DISABLE,

}ST_SIMPLE_FIC, *PST_SIMPLE_FIC;

typedef enum {

	LGD_DMB = 1,
	LGD_DAB,
	LGD_DATA,
	LGD_MULTI,
	LGD_SINGLE,

//	FREQ_FREE = 0,
//	FREQ_LOCK,
//	FIC_OK = 1,

}LGD_CTRL, *PLGD_CTRL;

typedef enum 
{
	ERROR_NON 				= 0x0000,
	ERROR_PLL 				= 0xE000,
	ERROR_STOP 				= 0xF000,
	ERROR_READY 			= 0xFF00,
	ERROR_SYNC_NO_SIGNAL	= 0xFC01,
	ERROR_SYNC_LOW_SIGNAL	= 0xFD01,
	ERROR_SYNC_NULL 		= 0xFE01,
	ERROR_SYNC_TIMEOUT 		= 0xFF01,
	ERROR_FICDECODER 		= 0xFF02,
	ERROR_START_MODEM_CLEAR = 0xFF05,
	ERROR_USER_STOP 		= 0xFA00,

	ERROR_MULTI_CHANNEL_COUNT_OVER 	= 0x8000,
	ERROR_MULTI_CHANNEL_COUNT_NON	= 0x8001,
	ERROR_MULTI_CHANNEL_NULL		= 0x8002,
	ERROR_MULTI_CHANNEL_FREQ		= 0x8003,
	ERROR_MULTI_CHANNEL_DMB_MAX		= 0x8004,

}LGD_ERROR_INFO, *PLGD_ERROR_INFO;

typedef struct _tagCHANNEL_INFO
{
	LGD_UINT32	ulRFFreq;
	LGD_UINT16	uiEnsembleID;
	LGD_UINT16	uiBitRate;
	LGD_UINT8	uiTmID;
	LGD_INT8	aucLabel[MAX_LABEL_CHAR];
	LGD_INT8	aucEnsembleLabel[MAX_LABEL_CHAR];

	LGD_UINT8	ucSubChID;
	LGD_UINT8	ucServiceType;
	LGD_UINT16	uiStarAddr;
	LGD_UINT8	ucSlFlag;
	LGD_UINT8	ucTableIndex;
	LGD_UINT8	ucOption;
	LGD_UINT8	ucProtectionLevel;
	LGD_UINT16	uiDifferentRate;
	LGD_UINT16	uiSchSize;

	LGD_UINT32	ulServiceID;
	LGD_UINT16	uiPacketAddr;

	LGD_UINT8	ucDataType;
	LGD_UINT32	ulDataThreshold;

#ifdef USER_APPLICATION_TYPE
	LGD_UINT16	uiUserAppType;
	LGD_UINT16	uiUserAppDataLength;
	LGD_UINT8	aucUserAppData[MAX_USER_APP_DATA];
#endif

}LGD_CHANNEL_INFO, *PLGD_CHANNEL_INFO;

typedef struct _tagST_SUBCH_INFO
{
	LGD_INT16			nSetCnt;
	LGD_CHANNEL_INFO	astSubChInfo[LGD_MULTI_MAX_CHANNEL];	//for android stack --> origin [MAX_SUBCH_SIZE];
}ST_SUBCH_INFO, *PST_SUBCH_INFO;

typedef enum _tagUPLOAD_MODE_INFO
{
	STREAM_UPLOAD_MASTER_SERIAL = 0,
	STREAM_UPLOAD_MASTER_PARALLEL,
	STREAM_UPLOAD_SLAVE_SERIAL,
	STREAM_UPLOAD_SLAVE_PARALLEL,
	STREAM_UPLOAD_SPI,
	STREAM_UPLOAD_TS,

}UPLOAD_MODE_INFO, *PUPLOAD_MODE_INFO;

typedef enum _tagCLOCK_SPEED
{
	LGD_OUTPUT_CLOCK_4096 = 1,
	LGD_OUTPUT_CLOCK_2048,
	LGD_OUTPUT_CLOCK_1024,

}CLOCK_SPEED, *PCLOCK_SPEED;

typedef enum _tagENSEMBLE_BAND
{
	KOREA_BAND_ENABLE = 0,
	BANDIII_ENABLE,
	LBAND_ENABLE,
	CHINA_ENABLE,
	ROAMING_ENABLE,
	EXTERNAL_ENABLE,

}ENSEMBLE_BAND, *PENSEMBLE_BAND;

typedef enum _tagFREQ_LOCKINFO
{
	LGD_FREQUENCY_UNLOCK = 0,
	LGD_FREQUENCY_LOCK,	
}FREQ_LOCKINFO, *PFREQ_LOCKINFO;

typedef enum _tagCTRL_MODE
{
	LGD_I2C_CTRL = 0,
	LGD_SPI_CTRL,
	LGD_EBI_CTRL,
}CTRL_MODE, *PCTRL_MODE;

typedef enum _tagACTIVE_MODE
{
	LGD_ACTIVE_LOW = 0,
	LGD_ACTIVE_HIGH,

}LGD_ACTIVE_MODE, *PLGD_ACTIVE_MODE;

#define BER_BUFFER_MAX		3
#define BER_REF_VALUE		35

typedef struct _tagST_BBPINFO
{
	LGD_UINT32			ulFreq;
	LGD_ERROR_INFO		nBbpStatus;
	LGD_UINT8			ucStop;
	ST_TRANSMISSION		ucTransMode;

	LGD_UINT8		ucAntLevel;
	LGD_UINT8		ucSnr;
	LGD_UINT8		ucVber;
	LGD_UINT16  	uiCER;
	LGD_UINT16  	wRssi;
	LGD_DOUBLE32 	dPreBER;
	LGD_DOUBLE32 	dPostBER;


	LGD_UINT32		ulReConfigTime;

	LGD_UINT16		uiInCAntTick;
	LGD_UINT16		uiInCERAvg;
	LGD_UINT16		uiIncPostBER;
	LGD_UINT16		auiANTBuff[BER_BUFFER_MAX];

	LGD_UINT8		ucProtectionLevel;
	LGD_UINT16		uiInCBERTick;
	LGD_UINT16		uiBerSum;
	LGD_UINT16		auiBERBuff[BER_BUFFER_MAX];

	LGD_UINT8 		ucCERCnt;
	LGD_UINT8 		ucRetryCnt;
	LGD_UINT8 		IsReSync;
	LGD_UINT8		ucRfData[3];
	LGD_UINT16		wDiffErrCnt;
	LGD_UINT8		IsChangeEnsemble;
}ST_BBPINFO, *PST_BBPINFO;

typedef struct _tagST_FIB_INFO{
	LGD_UINT16 uiIsCRC;
	LGD_UINT8 ucDataPos;
	LGD_UINT8 aucBuff[FIB_SIZE];
}ST_FIB_INFO;

typedef struct _tagST_FIC{
	LGD_UINT8	ucBlockNum;
	ST_FIB_INFO  stBlock;
}ST_FIC;

typedef struct  _tagSCH_DB_STRT
{
	LGD_UINT8   ucShortLong;
	LGD_UINT8   ucTableSW;
	LGD_UINT8   ucTableIndex;
	LGD_UINT8   ucOption;
	LGD_UINT8	ucProtectionLevel; 
	LGD_UINT16  uiSubChSize; 
	LGD_UINT16  uiBitRate; 
	LGD_UINT16  uiDifferentRate;
}SCH_DB_STRT, *PSCH_DB_STRT;

typedef struct  _tagST_UTC_INFO
{
	LGD_UINT8	ucGet_Time;

	LGD_UINT8	ucUTC_Flag;
	LGD_UINT16  uiHours;
	LGD_UINT16  uiMinutes;
	LGD_UINT16  uiSeconds;
	LGD_UINT16  uiMilliseconds;
}ST_UTC_INFO, *PST_UTC_INFO;

#ifdef USER_APPLICATION_TYPE	
typedef struct	_tagST_USER_APP_INFO
{
	LGD_UINT8	ucSCIdS;
	LGD_UINT8	ucNomOfUserApp;
	LGD_UINT16	uiUserAppType[MAX_SUBCHANNEL];
	LGD_UINT8	ucUserAppDataLength[MAX_SUBCHANNEL];
	LGD_UINT8	aucUserAppData[MAX_SUBCHANNEL][MAX_USER_APP_DATA];
}ST_USER_APP_INFO, *PST_USER_APP_INFO;
#endif

typedef struct _tagST_FIC_DB
{
	LGD_UINT32		ulRFFreq;
	LGD_UINT8		ucSubChCnt;
	LGD_UINT8		ucServiceComponentCnt;
	LGD_UINT16  	uiEnsembleID;
	LGD_UINT16  	uiSubChOk;
	LGD_UINT16  	uiSubChInfoOk;
	LGD_UINT16  	uiUserAppTypeOk;

	LGD_UINT16		aucSetPackAddr[MAX_SUBCHANNEL];
	LGD_UINT8		aucDSCType[MAX_SUBCHANNEL];
	LGD_UINT8		aucSubChID[MAX_SUBCHANNEL];
	LGD_UINT8		aucTmID[MAX_SUBCHANNEL];
	LGD_UINT8		aucEnsembleLabel[MAX_LABEL_CHAR];
	LGD_UINT8		aucServiceLabel[MAX_SUBCHANNEL][MAX_LABEL_CHAR];
	LGD_UINT16		auiStartAddr[MAX_SUBCHANNEL];
	LGD_UINT32		aulServiceID[MAX_SUBCHANNEL];
	LGD_UINT16		auiServicePackMode[MAX_SUBCHANNEL];
	LGD_UINT8		aucSubChPackMode[MAX_SUBCHANNEL];
	LGD_UINT16		auiPacketAddr[MAX_SUBCHANNEL];
	LGD_UINT8		aucServiceTypePackMode[MAX_SUBCHANNEL];

	LGD_UINT8		aucServiceComponID[MAX_SUBCHANNEL];
	LGD_UINT8		aucServiceExtension[MAX_SUBCHANNEL];
	LGD_UINT16		auiUserAppType[MAX_SUBCHANNEL];

	SCH_DB_STRT			astSchDb[MAX_SUBCHANNEL];
#ifdef USER_APPLICATION_TYPE	
	ST_USER_APP_INFO	astUserAppInfo;
#endif
}ST_FIC_DB, *PST_FIC_DB;

typedef union _tagST_FIG_HEAD{
	LGD_UINT8 ucInfo;
	struct {
		LGD_UINT8 bitLength		:	5;
		LGD_UINT8 bitType		:	3;
	}ITEM;
}ST_FIG_HEAD, *PST_FIG_HEAD;

typedef union _tagST_TYPE_0{
	LGD_UINT8 ucInfo;
	struct{
		LGD_UINT8 bitExtension	:	5;
		LGD_UINT8 bitPD			:	1;
		LGD_UINT8 bitOE			:	1;
		LGD_UINT8 bitCN			:	1;
	}ITEM;
}ST_TYPE_0, *PST_TYPE_0;

typedef union _tagST_TYPE_1{
	LGD_UINT8 ucInfo;
	struct {
		LGD_UINT8 bitExtension	:	3;
		LGD_UINT8 bitOE			:	1;
		LGD_UINT8 bitCharset	:	4;
	}ITEM;
}ST_TYPE_1, *PST_TYPE_1;


typedef union _tagST_USER_APPSERID_16
{
	LGD_UINT16 uiInfo;
	struct {
		LGD_UINT16 bitServiceID	:	16;
	}ITEM;	

}ST_USER_APPSERID_16, *PST_USER_APPSERID_16;

typedef union _tagST_USER_APPSERID_32
{
	LGD_UINT32 ulInfo;
	struct {
		LGD_UINT32 bitServiceID	:	32;
	}ITEM;	

}ST_USER_APPSERID_32, *PST_USER_APPSERID_32;

typedef union _tagST_USER_APP_IDnNUM
{
	LGD_UINT8 ucInfo;
	struct{
		LGD_UINT8 bitNomUserApp	: 4;
		LGD_UINT8 bitSCIdS		: 4;
	}ITEM;
}ST_USER_APP_IDnNUM, *PST_USER_APP_IDnNUM;

typedef union _tagST_USER_APPTYPE
{
	LGD_UINT16 uiInfo;
	struct{
		LGD_UINT16 bitUserDataLength 	: 5;
		LGD_UINT16 bitUserAppType 		: 11;
	}ITEM;
}ST_USER_APPTYPE, *PST_USER_APPTYPE;


typedef union _tagST_TYPE0of0_INFO{
	LGD_UINT32 ulBuff;
	struct {
		LGD_UINT32 bitLow_CIFCnt	:	8;
		LGD_UINT32 bitHigh_CIFCnt	:	5;
		LGD_UINT32 bitAlFlag		:	1;
		LGD_UINT32 bitChangFlag		:	2;
		LGD_UINT32 bitEld			:	16;
	}ITEM;
}ST_TYPE0of0_INFO, *PST_TYPE0of0_INFO;

typedef union _tagST_TYPE0of1Short_INFO{
	LGD_UINT32 nBuff;
	struct {
		LGD_UINT32 bitReserved		:	8;
		LGD_UINT32 bitTableIndex	:	6;
		LGD_UINT32 bitTableSw		:	1;
		LGD_UINT32 bitShortLong		:	1;
		LGD_UINT32 bitStartAddr		:	10;
		LGD_UINT32 bitSubChId		:	6;
	}ITEM;
}ST_TYPE0of1Short_INFO, *PST_TYPE0of1Short_INFO;

typedef union _tagST_TYPE0of1Long_INFO{
	LGD_UINT32 nBuff;
	struct {
		LGD_UINT32 bitSubChanSize	:	10;
		LGD_UINT32 bitProtecLevel	:	2;
		LGD_UINT32 bitOption		:	3;
		LGD_UINT32 bitShortLong		:	1;
		LGD_UINT32 bitStartAddr		:	10;
		LGD_UINT32 bitSubChId		:	6;
	}ITEM;
}ST_TYPE0of1Long_INFO, *PST_TYPE0of1Long_INFO;

typedef union _tagST_TYPE0of3Id_INFO{
	LGD_UINT32 ulData;
	struct {
		LGD_UINT32 bitReserved	:	8;
		LGD_UINT32 bitPacketAddr:	10;
		LGD_UINT32 bitSubChId	:	6;
		LGD_UINT32 bitDScType	:	6;
		LGD_UINT32 bitRfu		:	1;
		LGD_UINT32 bitFlag		:	1;
	}ITEM;
}ST_TYPE0of3Id_INFO, *PST_TYPE0of3Id_INFO;

typedef union _tagST_TYPE0of3_INFO{
	LGD_UINT16 nData;
	struct {
		LGD_UINT16 bitSccaFlag	:	1;
		LGD_UINT16 bitReserved	:	3;
		LGD_UINT16 bitScid		:	12;
	}ITEM;
}ST_TYPE0of3_INFO, *PST_TYPE0of3_INFO;

typedef union _tagST_SERVICE_COMPONENT{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bitNumComponent:	4;
		LGD_UINT8 bitCAId		:	3;
		LGD_UINT8 bitLocalFlag	:	1;
	}ITEM;
}ST_SERVICE_COMPONENT, *PST_SERVICE_COMPONENT;

typedef union _tagST_TMId_MSCnFIDC{
	LGD_UINT16 uiData;
	struct {
		LGD_UINT16 bitCAflag	:	1;
		LGD_UINT16 bitPS		:	1;
		LGD_UINT16 bitSubChld	:	6;
		LGD_UINT16 bitAscDscTy	:	6;
		LGD_UINT16 bitTMId		:	2;
	}ITEM;
}ST_TMId_MSCnFIDC, *PST_TMId_MSCnFIDC;

typedef union _tagST_MSC_PACKET_INFO{
	LGD_UINT16 usData;
	struct {
		LGD_UINT16 bitCAflag	:	1;
		LGD_UINT16 bitPS		:	1;
		LGD_UINT16 bitSCId		:	12;
		LGD_UINT16 bitTMId		:	2;
	}ITEM;
}ST_MSC_PACKET_INFO, *PST_MSC_PACKET_INFO;

typedef union _tagST_MSC_BIT{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bitScIds		:	4;
		LGD_UINT8 bitRfa		:	3;
		LGD_UINT8 bitExtFlag	:	1;
	}ITEM;
}ST_MSC_BIT, *PST_MSC_BIT;

typedef union _tagST_MSC_LONG{
	LGD_UINT16 usData;
	struct {
		LGD_UINT16 bitScId		:	12;
		LGD_UINT16 bitDummy		:	3;
		LGD_UINT16 bitLsFlag	:	1;
	}ITEM;
}ST_MSC_LONG, *PST_MSC_LONG;

typedef union _tagST_MSC_SHORT{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bitSUBnFIDCId	:	6;
		LGD_UINT8 bitMscFicFlag	:	1;
		LGD_UINT8 bitLsFlag		:	1;
	}ITEM;
}ST_MSC_SHORT, *PST_MSC_SHORT;

typedef union _tagST_EXTENSION_TYPE14{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bitSCidS		:	4;
		LGD_UINT8 bitRfa		:	3;
		LGD_UINT8 bitPD			:	1;
	}ITEM;
}ST_EXTENSION_TYPE14, *PST_EXTENSION_TYPE14;

typedef union _tagST_EXTENSION_TYPE12{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bitReserved1	:	6;
		LGD_UINT8 bitCF_flag	:	1;
		LGD_UINT8 bitCountry	:	1;
	}ITEM;
}ST_EXTENSION_TYPE12, *PST_EXTENSION_TYPE12;

typedef union _tagST_UTC_SHORT_INFO{
	LGD_UINT32 ulBuff;
	struct {
		LGD_UINT32 bitMinutes	:	6;
		LGD_UINT32 bitHours		:	5;
		LGD_UINT32 bitUTC_Flag	:	1;
		LGD_UINT32 bitConf_Ind	:	1;
		LGD_UINT32 bitLSI		:	1;
		LGD_UINT32 bitMJD		:	17;
		LGD_UINT32 bitRfu		:	1;		
	}ITEM;
}ST_UTC_SHORT_INFO, *PST_UTC_SHORT_INFO;

typedef union _tagST_UTC_LONG_INFO{
	LGD_UINT16 unBuff;
	struct {
		LGD_UINT16 bitMilliseconds	:	10;
		LGD_UINT16 bitSeconds		:	6;
	}ITEM;
}ST_UTC_LONG_INFO, *PST_UTC_LONG_INFO;


#ifdef 		LGD_FIFO_SOURCE_ENABLE
#define		LGD_CIF_MAX_SIZE		(188*8)
#define 	LGD_FIFO_DEPTH			(LGD_INTERRUPT_SIZE*10)
#define		MAX_CHANNEL_FIFO		5
#define 	MAX_HEADER_SIZE			16
#define 	HEADER_SERACH_SIZE		(LGD_CIF_MAX_SIZE + MAX_HEADER_SIZE)
#define		HEADER_ID_0x33			0x33
#define		HEADER_ID_0x00			0x00
#define 	HEADER_SIZE_BITMASK		0x3FF
typedef enum _tagMULTI_CHANNEL_INFO
{
	MAIN_INPUT_DATA = 0,
	FIC_STREAM_DATA,
	CHANNEL1_STREAM_DATA,
	CHANNEL2_STREAM_DATA,
	CHANNEL3_STREAM_DATA,
}MULTI_CHANNEL_INFO, *PMULTI_CHANNEL_INFO;

typedef enum _tagST_HEADER_INFO
{
	LGD_HEADER_SIZE_ERROR = 0,
	LGD_HEADER_NOT_SEARACH,
	LGD_HEADER_GOOD,
}ST_HEADER_INFO, *PST_HEADER_INFO;


#define LGD_SUB_CHANNEL_ID_MASK			0xFFFF
typedef struct _tagST_FIFO{
	LGD_UINT32	ulDepth;
	LGD_UINT32	ulFront;
	LGD_UINT32	ulRear;
	LGD_UINT16 	unSubChID;
	LGD_UINT8	acBuff[LGD_FIFO_DEPTH+1];
}ST_FIFO, *PST_FIFO;

LGD_UINT8 LGD_QFIFO_INIT(PST_FIFO pFF,  LGD_UINT32 ulDepth);
LGD_UINT32 LGD_QFIFO_FREE_SIZE(PST_FIFO pFF);
LGD_UINT32 LGD_QFIFO_GET_SIZE(PST_FIFO pFF);
LGD_UINT8 LGD_QFIFO_ADD(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize);
LGD_UINT8 LGD_QFIFO_AT(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize);
LGD_UINT8* LGD_QFIFO_AT_PTR(PST_FIFO pFF);
LGD_UINT8 LGD_QFIFO_BRING(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize);
void LGD_MULTI_SORT_INIT(void);
ST_FIFO* LGD_GET_CHANNEL_FIFO(MULTI_CHANNEL_INFO ucIndex);
LGD_UINT8* LGD_GET_CHANNEL_FIFO_PTR(MULTI_CHANNEL_INFO ucIndex);		//inb612 for android compile
LGD_UINT8 LGD_MULTI_FIFO_PROCESS(LGD_UINT8* pData, LGD_UINT32 ulSize);
ST_HEADER_INFO LGD_HEADER_CHECK(ST_FIFO* pMainFifo);
LGD_UINT32	LGD_GET_IDS_SIZE(LGD_UINT16 unID);
LGD_UINT8	LGD_GET_ID_BRINGUP(LGD_UINT16 unID, LGD_UINT8* pData, LGD_UINT32 ulSize);

#endif


#ifdef LGD_I2C_GPIO_CTRL_ENABLE
#define I2C_SCL_PIN						0x01
#define I2C_SDA_PIN						0x02
#define I2C_BIT_HIGH					1
#define I2C_BIT_LOW						0

#define INTERRUPT_LOCK()
#define INTERRUPT_FREE()

#define LGD_I2C_IO_MASK       			0x01
#define LGD_I2C_IO_MASK_READ     		0x01
#define LGD_I2C_IO_MASK_WRITE     		0x00
#define LGD_I2C_ADDRMASK_R(X)       	(((X) & ~LGD_I2C_IO_MASK) | LGD_I2C_IO_MASK_READ)
#define LGD_I2C_ADDRMASK_W(X)       	(((X) & ~LGD_I2C_IO_MASK) | LGD_I2C_IO_MASK_WRITE)
#define I2C_DATA_BIT_MAX				8
typedef enum _tagLGD_I2C_ACK
{
	I2C_ACK_SUCCESS = 0,
	I2C_ACK_ERROR,
}LGD_I2C_ACK;

LGD_UINT8 I2C_SDA_READ(void);
void I2C_SDA_WRITE(LGD_UINT8 ucBit);
void I2C_SCL_WRITE(LGD_UINT8 ucBit);

void        LGD_GPIO_I2C_INIT(void);
void        LGD_GPIO_I2C_START_COMMAND(void);
void        LGD_GPIO_I2C_STOP_COMMAND(void);
void        LGD_GPIO_I2C_READ_ACK(void);
LGD_UINT8   LGD_GPIO_READ_BYTE_IO(void);
LGD_I2C_ACK LGD_GPIO_I2C_ACK(void);
LGD_I2C_ACK LGD_GPIO_WRITE_BYTE_IO(LGD_UINT8 ucData);
LGD_I2C_ACK LGD_GPIO_READ_BYTES(LGD_UINT8* pBuff, LGD_UINT16 uiSize);
LGD_I2C_ACK LGD_GPIO_WRITE_BYTES(LGD_UINT8* pBuff, LGD_UINT16 uiSize);
LGD_I2C_ACK LGD_GPIO_CTRL_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 uiSize);
LGD_I2C_ACK LGD_GPIO_CTRL_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 uiSize);
#endif


#ifdef LGD_EWS_SOURCE_ENABLE
typedef union _tagST_TYPE_5{
	LGD_UINT8 ucInfo;
	struct {
		LGD_UINT8 bitExtension	:	3;
		LGD_UINT8 bitTCId		:	3;
		LGD_UINT8 bitD2			:	1;
		LGD_UINT8 bitD1			:	1;
	}ITEM;
}ST_TYPE_5, *PST_TYPE_5;

typedef union _tagST_EWS_INFO
{
	LGD_UINT16 unData;
	struct  
	{
		LGD_UINT16	bitID			: 5;
		LGD_UINT16	bitMsgGovernment : 3;
		LGD_UINT16	bitTotalNo		: 4;
		LGD_UINT16	bitThisSeqNo	: 4;
	}ITEM;

}ST_EWS_INFO, *PST_EWS_INFO;


typedef union _tagST_EWS_TIME
{
	LGD_UINT32 unData;
	struct  
	{
		LGD_UINT32	bitUTCMinutes	: 6;
		LGD_UINT32	bitUTCHours		: 5;
		LGD_UINT32	bitMJD			: 17;
		LGD_UINT32	bitReserved		: 4;
	}ITEM;

}ST_EWS_TIME, *PST_EWS_TIME;

typedef struct _tagST_DATE_T
{
	LGD_UINT16		usYear;
	LGD_UINT8		ucMonth;
	LGD_UINT8		ucDay;
	LGD_UINT8		ucHour;
	LGD_UINT8		ucMinutes;
}ST_DATE_T, *PST_DATE_T;


typedef struct _tagEWS_INPUT_INFO
{
	LGD_UINT16 uiCnt;
	LGD_INT8	acBuff[32];
}EWS_INPUT_INFO, *PEWS_INPUT_INFO;

typedef struct _tagEWS_RESION_INFO
{
	LGD_INT8	acResionCode[11];
}EWS_RESION_INFO, *PEWS_RESION_INFO;


#define EWS_OUTPUT_BUFF_MAX	(1024)
#define		MAX_EWS_SEQUENCY		16
#define		MAX_EWS_IDS					32
#define		MAX_RESION_CODE			10
#define		MAX_RESION_COUNTER	16
typedef struct _tagST_OUTPUT_EWS
{
	LGD_UINT8	ucMsgGovernment;	//메시지 발령기관
	LGD_UINT8	ucMsgID;			//메시지 고유번호
	ST_DATE_T	stDate;				//일시

	LGD_INT8	acKinds[4];			//재난종류
	LGD_UINT8	cPrecedence;		//우선순위
	LGD_UINT32	ulTime;				//재난시간
	LGD_UINT8	ucForm;				//재난지역형식
	LGD_UINT8	ucResionCnt;		//재난 지역수
	LGD_UINT8	ucIsEWSGood;

	LGD_INT8	acTempBuff[EWS_OUTPUT_BUFF_MAX];	// EWS Message ANSI
	LGD_INT8	acEWSMessage[EWS_OUTPUT_BUFF_MAX];	// EWS Message ANSI
	EWS_INPUT_INFO	astInputBuff[MAX_EWS_SEQUENCY];
	EWS_RESION_INFO	astResionCode[MAX_RESION_COUNTER];
}ST_OUTPUT_EWS, *PST_OUTPUT_EWS;


typedef struct _tagST_OUTPUT_GROUP
{
	LGD_INT16			nMsgIDCnt;
	ST_OUTPUT_EWS	astEWSMsg[MAX_EWS_IDS];
}ST_OUTPUT_GROUP, *PST_OUTPUT_GROUP;




void MJDtoYMD(LGD_UINT16 wMJD, ST_DATE_T *pstDate);
void LGD_EWS_INIT(void);
void LGD_TYPE5_EXTENSION2(ST_FIB_INFO* pFibInfo);
void LGD_SET_TYPE_5(ST_FIB_INFO* pFibInfo);
LGD_UINT8 LGD_EWS_PARSING(LGD_UINT8 ucI2CID, LGD_UINT8* pucFicBuff, LGD_INT32 uFicLength);
LGD_UINT8 LGD_EWS_FRAMECHECK(LGD_UINT8 ucI2CID);
LGD_UINT32 YMDtoMJD(ST_DATE_T stDate);
ST_OUTPUT_EWS* LGD_GET_EWS_DB(void);

#endif


#ifdef LGD_DLS_SOURCE_ENABLE

#define XPAD_INIT				1
#define XPAD_SUCCESS			2
#define XPAD_FRAME_CHECK		4

#define XPAD_TPYE_DLS_ST		2
#define XPAD_TPYE_DLS_CT		3
#define XPAD_END_MARKER			0
#define XPAD_SIZE				250

#define FPAD_POS_BYTE_L			0
#define FPAD_POS_BYTE_L_1		1
#define XPAD_POS_TYPE			6

typedef struct _tagST_DLS{
	LGD_INT8	cTransData[XPAD_SIZE];
	LGD_UINT8	ucSegment;
	LGD_UINT8	ucPadStart;
	LGD_UINT8	ucPadCnt;
	LGD_UINT8	ucPadOk;

}ST_DLS, *PST_DLS;

void		LGD_DLS_INIT(void);
void		LGD_PAD_UNICODE_CHECK(void);
LGD_UINT8	LGD_DLS_DECODER(LGD_UINT8 *pucData, LGD_UINT16 wBitrate);
LGD_UINT16	LGD_PAD_CRC_CHECK(LGD_UINT8 *pBuf, LGD_UINT16 wPos, LGD_UINT8 ucSize); 
#endif


typedef enum tagLGD_SORT_OPTION{

	LGD_SUB_CHANNEL_ID = 0,
	LGD_START_ADDRESS,
	LGD_BIRRATE,
	LGD_FREQUENCY,
}LGD_SORT_OPTION, *PLGD_SORT_OPTION;


LGD_UINT8	LGD_DELAY(LGD_UINT8 ucI2CID, LGD_UINT16 uiDelay);
void		LGD_MPICLOCK_SET(LGD_UINT8 ucI2CID);
void		LGD_UPLOAD_MODE(LGD_UINT8 ucI2CID);
void		LGD_INTERRUPT_CLEAR(LGD_UINT8 ucI2CID, LGD_UINT16 uiClrInt);
void        LGD_INTERRUPT_CTRL(LGD_UINT8 ucI2CID);
void		LGD_INTERRUPT_SET(LGD_UINT8 ucI2CID, LGD_UINT16 uiSetInt);
void		LGD_SET_CHANNEL(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo);
void        LGD_BUBBLE_SORT(ST_SUBCH_INFO* pMainInfo, LGD_SORT_OPTION Opt);
LGD_UINT8	LGD_BUBBLE_SORT_by_TYPE(ST_SUBCH_INFO* pMainInfo);
void 		LGD_SWAP(ST_SUBCH_INFO* pMainInfo, LGD_UINT16 nNo1, LGD_UINT16 nNo2);
void		INTERFACE_USER_STOP(LGD_UINT8 ucI2CID);
void		INTERFACE_INT_ENABLE(LGD_UINT8 ucI2CID, LGD_UINT16 unSet);
LGD_UINT8	INTERFACE_INT_CHECK(LGD_UINT8 ucI2CID);
void		INTERFACE_INT_CLEAR(LGD_UINT8 ucI2CID, LGD_UINT16 unClr);
void 		LGD_INITDB(LGD_UINT8 ucI2CID);
void 		LGD_SET_SHORTFORM(ST_FIC_DB* pstFicDB, LGD_INT32 nCh, ST_TYPE0of1Short_INFO* pShort);
void 		LGD_SET_LONGFORM(ST_FIC_DB* pstFicDB, LGD_INT32 nCh, ST_TYPE0of1Long_INFO* pLong);
void 		LGD_EXTENSION_000(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_001(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_002(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_003(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_008(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void		LGD_INIT_MPI(LGD_UINT8 ucI2CID);
void 		LGD_RESET_MPI(LGD_UINT8 ucI2CID);



#ifdef USER_APPLICATION_TYPE
void 		LGD_EXTENSION_013(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
#endif




void 		LGD_EXTENSION_110(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_111(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_112(ST_FIB_INFO* pFibInfo);
void 		LGD_EXTENSION_113(ST_FIB_INFO* pFibInfo);
void 		LGD_EXTENSION_114(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_EXTENSION_115(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_SET_FICTYPE_1(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_SET_FICTYPE_5(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
void 		LGD_SET_UPDATEFIC(ST_FIB_INFO* pstDestData, LGD_UINT8* pSourData);

void INTERFACE_UPLOAD_MODE(LGD_UINT8 ucI2CID, UPLOAD_MODE_INFO ucUploadMode);
LGD_UINT8 INTERFACE_PLL_MODE(LGD_UINT8 ucI2CID, PLL_MODE ucPllMode);

LGD_UINT8	INTERFACE_DBINIT(void);
LGD_UINT8	LGD_UPDATE(LGD_CHANNEL_INFO* pChInfo, ST_FIC_DB* pFicDb, LGD_INT16 nIndex);
LGD_UINT8	INTERFACE_RESET_CH(LGD_UINT8 ucI2CID);
LGD_UINT8	INTERFACE_INIT(LGD_UINT8 ucI2CID);
LGD_UINT8	INTERFACE_RECONFIG(LGD_UINT8 ucI2CID);
LGD_UINT8	INTERFACE_STATUS_CHECK(LGD_UINT8 ucI2CID);
LGD_UINT8	INTERFACE_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo);

LGD_UINT8	INTERFACE_SCAN(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq);
LGD_UINT8	INTERFACE_GET_SNR(LGD_UINT8 ucI2CID);
LGD_UINT8	INTERFACE_ISR(LGD_UINT8 ucI2CID, LGD_UINT8* pBuff);
LGD_UINT8 	LGD_CHECK_SERVICE_DB16(LGD_UINT16* ptr, LGD_UINT16 wVal, LGD_UINT16 wNum);
LGD_UINT8 	LGD_CHECK_SERVICE_DB8(LGD_UINT8* ptr, LGD_UINT8 cVal, LGD_UINT8 cNum);
LGD_UINT8 	LGD_GET_BYTEDATA(ST_FIB_INFO* pFibInfo);
LGD_UINT8	LGD_GETAT_BYTEDATA(ST_FIB_INFO* pFibInfo);
LGD_UINT8 	LGD_GET_HEADER(ST_FIB_INFO* pInfo);
LGD_UINT8 	LGD_GET_TYPE(ST_FIB_INFO* pInfo);
LGD_UINT8 	LGD_GETAT_HEADER(ST_FIB_INFO* pInfo);
LGD_UINT8 	LGD_GETAT_TYPE(ST_FIB_INFO* pInfo);
LGD_UINT8	LGD_INIT(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_READY(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq);
LGD_UINT8	LGD_SYNCDETECTOR(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq, LGD_UINT8 ucScanMode);
LGD_UINT8	LGD_FICDECODER(LGD_UINT8 ucI2CID, ST_SIMPLE_FIC bSimpleFIC);

LGD_UINT8	LGD_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo, LGD_UINT16 IsEnsembleSame);

LGD_UINT8	LGD_STOP(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_CHANNEL_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo);

void LGD_SCAN_SETTING(LGD_UINT8 ucI2CID);
void LGD_AIRPLAY_SETTING(LGD_UINT8 ucI2CID);


LGD_UINT8	LGD_ENSEMBLE_SCAN(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq);
LGD_UINT8	LGD_FIC_RECONFIGURATION_HW_CHECK(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_STATUS_CHECK(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_GET_ANT_LEVEL(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_GET_SNR(LGD_UINT8 ucI2CID);
LGD_UINT8	LGD_GET_VBER(LGD_UINT8 ucI2CID);

LGD_UINT8 	LGD_GET_NULLBLOCK(ST_FIG_HEAD* pInfo);
LGD_UINT8 	LGD_GET_FINDLENGTH(ST_FIG_HEAD* pInfo);
LGD_UINT8 	LGD_SET_TRANSMIT_MODE(LGD_UINT8 ucMode);
LGD_UINT8 	LGD_GET_FINDTYPE(ST_FIG_HEAD* pInfo);
LGD_UINT8 	LGD_FICPARSING(LGD_UINT8 ucI2CID, LGD_UINT8* pucFicBuff, LGD_INT32 uFicLength, ST_SIMPLE_FIC bSimpleFIC);
LGD_UINT8	LGD_CMD_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData);
LGD_UINT8	LGD_I2C_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData);
LGD_UINT8	LGD_EBI_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData);
LGD_UINT16  LGD_EBI_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr);

LGD_UINT8	LGD_I2C_READ_BURST(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize);
LGD_UINT8	LGD_SPI_REG_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData);
LGD_UINT8	LGD_CMD_READ_BURST(LGD_UINT8 ucI2CID,  LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize);
LGD_UINT8	LGD_SPI_READ_BURST(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 wSize);
LGD_UINT8	LGD_CHIP_STATUS(LGD_UINT8 ucI2CID);
LGD_UINT8 	LGD_RF500_START(LGD_UINT8 ucI2CID, LGD_UINT32 ulRFChannel, ENSEMBLE_BAND ucBand);
LGD_UINT8 	LGD_RF500_I2C_WRITE(LGD_UINT8 ucI2CID, LGD_UINT8 *pucData, LGD_UINT32 uLength);
LGD_UINT8	LGD_FIC_UPDATE(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo);
LGD_UINT8	LGD_GET_FIB_CNT(ST_TRANSMISSION ucMode);
LGD_UINT8   LGD_EBI_READ_BURST(LGD_UINT8 ucI2CID,  LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize);


LGD_INT16	LGD_GET_RSSI(LGD_UINT8 ucI2CID);
LGD_UINT8 	LGD_RE_SYNC(LGD_UINT8 ucI2CID);


LGD_INT16 	LGD_CHECK_SERVICE_CNT16(LGD_UINT16* ptr, LGD_UINT16 wVal, LGD_UINT8 cNum, LGD_UINT16 wMask);
LGD_INT16 	LGD_CHECK_SERVICE_CNT8(LGD_UINT8* ptr, LGD_UINT8 cVal, LGD_UINT8 cNum, LGD_UINT8 cMask);
LGD_INT16   LGD_CHECK_SERVICE_CNT32(LGD_UINT32* ptr, LGD_UINT32 wVal, LGD_UINT8 cNum);

LGD_UINT16 	LGD_FIND_KOR_FREQ(LGD_UINT32 ulFreq);
LGD_UINT16 	LGD_FIND_LBAND_FREQ(LGD_UINT32 ulFreq);
LGD_UINT16 	LGD_FIND_CHINA_FREQ(LGD_UINT32 ulFreq);
LGD_UINT16 	LGD_FIND_BANDIII_FREQ(LGD_UINT32 ulFreq);
LGD_UINT16 	LGD_FIND_ROAMING_FREQ(LGD_UINT32 ulFreq);
LGD_UINT16 	LGD_CRC_CHECK(LGD_UINT8 *pBuf, LGD_UINT8 ucSize) ;
LGD_UINT16 	LGD_GET_BITRATE(SCH_DB_STRT * pstSchDb);
LGD_UINT16 	LGD_SET_FICTYPE_0(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB);
LGD_UINT16 	LGD_GET_WORDDATA(ST_FIB_INFO* pFibInfo);
LGD_UINT16 	LGD_GETAT_WORDDATA(ST_FIB_INFO* pFibInfo);
LGD_UINT16 	LGD_FIND_SUBCH_SIZE(LGD_UINT8 ucTableIndex);

LGD_UINT16	INTERFACE_GET_CER(LGD_UINT8 ucI2CID);
LGD_UINT16	INTERFACE_GETDMB_CNT(void);
LGD_UINT16	INTERFACE_GETDAB_CNT(void);
LGD_UINT16	INTERFACE_GETDATA_CNT(void);
LGD_UINT16	LGD_CMD_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr);
LGD_UINT16	LGD_I2C_READ(LGD_UINT8 ucI2CID, LGD_UINT16 ulAddr);
LGD_UINT16	LGD_SPI_REG_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr);
LGD_UINT16	LGD_GET_FRAME_DURATION(ST_TRANSMISSION cTrnsMode);
LGD_UINT16	LGD_GET_CER(LGD_UINT8 ucI2CID);

LGD_UINT32 	LGD_GET_KOREABAND_FULL_TABLE(LGD_UINT16 uiIndex);
LGD_UINT32 	LGD_GET_KOREABAND_NORMAL_TABLE(LGD_UINT16 uiIndex);
LGD_UINT32 	LGD_GET_LONGDATA(ST_FIB_INFO* pFibInfo);
LGD_UINT32 	LGD_GETAT_LONGDATA(ST_FIB_INFO* pFibInfo);

LGD_DOUBLE32 INTERFACE_GET_POSTBER(LGD_UINT8 ucI2CID);
LGD_DOUBLE32 INTERFACE_GET_PREBER(LGD_UINT8 ucI2CID);
LGD_DOUBLE32 LGD_GET_PREBER(LGD_UINT8 ucI2CID);
LGD_DOUBLE32 LGD_GET_POSTBER(LGD_UINT8 ucI2CID);
LGD_UINT16 LGD_GET_TPERRCNT(LGD_UINT8 ucI2CID); /* LGE MC Add for TP Error count */

ST_FIC_DB* 			LGD_GETFIC_DB(LGD_UINT8 ucI2CID);
ST_BBPINFO* 		LGD_GET_STRINFO(LGD_UINT8 ucI2CID);
LGD_ERROR_INFO		INTERFACE_ERROR_STATUS(LGD_UINT8 ucI2CID);
LGD_CHANNEL_INFO* 	INTERFACE_GETDB_DMB(LGD_INT16 uiPos);
LGD_CHANNEL_INFO* 	INTERFACE_GETDB_DAB(LGD_INT16 uiPos);
LGD_CHANNEL_INFO* 	INTERFACE_GETDB_DATA(LGD_INT16 uiPos);
LGD_UINT8*          INTERFACE_GETENSEMBLE_LABEL(LGD_UINT8 ucI2CID);

LGD_UINT8 LGD_PLL_SET(LGD_UINT8 ucI2CID);

LGD_UINT8 SAVE_CHANNEL_INFO(char* pStr);
LGD_UINT8 LOAD_CHANNEL_INFO(char* pStr);
LGD_UINT8 INTERFACE_CHANGE_BAND(LGD_UINT8 ucI2CID, LGD_UINT16 usBand);
LGD_UINT8 INTERFACE_FIC_UPDATE_CHECK(LGD_UINT8 ucI2CID);
//void	  INTERFACE_LGD_DEBUG(LGD_UINT8 ucI2CID);
LGD_UINT8 INTERFACE_RE_SYNC(LGD_UINT8 ucI2CID);											//inb612 for android compile
LGD_UINT8 INTERFACE_RE_SYNCDETECTOR(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pMultiInfo);		//inb612 for android compile
LGD_UINT16 INTERFACE_GET_TPERRCNT(LGD_UINT8 ucI2CID);									//inb612 for android compile

LGD_UINT8 LGD_RE_SYNCDETECTOR(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pMultiInfo);			//inb612 for android compile

#ifdef LGD_TII_TEST_ENABLE

#define MAX_TII_CNT 3
typedef struct _tagST_TII_INFO
{
	LGD_UINT16 uiStrength;
	LGD_UINT16 uiSubID;
	LGD_UINT16 uiZeroPad;
	LGD_UINT16 uiPattern;
}ST_TII_INFO;

LGD_INT16 LGD_TII_START(LGD_UINT8 ucI2CID);
LGD_INT16 LGD_TII_STOP(LGD_UINT8 ucI2CID);
LGD_INT16 LGD_TII_GET_INFO(LGD_UINT8 ucI2CID, ST_TII_INFO* pstTIIInfo);
#endif


#define MAX_NET_BER		(48*16)
typedef union _tagST_NETBER_INFO{
	LGD_UINT8 ucData;
	struct {
		LGD_UINT8 bit0	:	1;
		LGD_UINT8 bit1	:	1;
		LGD_UINT8 bit2	:	1;
		LGD_UINT8 bit3	:	1;
		LGD_UINT8 bit4	:	1;
		LGD_UINT8 bit5	:	1;
		LGD_UINT8 bit6	:	1;
		LGD_UINT8 bit7	:	1;
	}ITEM;
}ST_NETBER_INFO, *PST_NETBER_INFO;


#ifdef LGD_SLS_DECODER

typedef enum 
{
	DAB_DECODER_ERROR = 0,
	DAB_SYNCWORD_ERROR,
	DAB_DECODER_SUCCESS,
		
}ST_DAB_STATUS;


#define DAB_SYNC_WORD			0xFFF

typedef union _tagDAB_HEADER
{
	LGD_UINT32 	ulData;
	struct{
		LGD_UINT32		bEmphasis			: 2;
		LGD_UINT32		bOriginal			: 1;
		LGD_UINT32		bCopyright			: 1;
		LGD_UINT32		bModeExten			: 2;
		LGD_UINT32		bMode				: 2;
		LGD_UINT32		bPrivatebit			: 1;
		LGD_UINT32		bPaddBit			: 1;
		LGD_UINT32		bSampFreq			: 2;
		LGD_UINT32		bbitRateIndex 		: 4;
		LGD_UINT32		bProtecBit			: 1;
		LGD_UINT32		bLayer				: 2;
		LGD_UINT32		bID					: 1;
		LGD_UINT32		bSyncWord			: 12;
	}ITEM;

}ST_DAB_HEADER, *PST_DAB_HEADER;

void LGD_DAB_SLS_INIT();
ST_DAB_STATUS LGD_GET_HEADER_INFO();

#endif

#endif
