/*****************************************************************************
 Copyright(c) 2008 LG Electronics Inc. All Rights Reserved
 
 File name : Tunerbb_drv_lg2102.c
 
 Description : LG2102 made by LGSIC Driver Code
 
 History : 
 ----------------------------------------------------------------------
 Mar. 10,  2009 :   LGSIC release for LGMC
*******************************************************************************/
#include <linux/broadcast/broadcast_tdmb_typedef.h>
#include <linux/broadcast/broadcast_tdmb.h>

#include <linux/broadcast/broadcast_lg2102_includes.h>
#include <linux/broadcast/broadcast_lg2102_ioctrl.h>

/*============================================================
**    1.   DEFINITIONS
*============================================================*/

/*============================================================
**    2.   External Variables
*============================================================*/
extern ST_SUBCH_INFO		g_stDmbInfo;
extern ST_SUBCH_INFO		g_stDabInfo;
extern ST_SUBCH_INFO		g_stDataInfo;
extern UPLOAD_MODE_INFO		m_ucUploadMode;
extern LGD_ACTIVE_MODE		m_ucMPI_CS_Active;
extern LGD_ACTIVE_MODE		m_ucMPI_CLK_Active;
extern CTRL_MODE 			m_ucCommandMode;
extern PLL_MODE				m_ucPLL_Mode;
extern LGD_UINT16			m_unIntCtrl;
extern LGD_DPD_MODE			m_ucDPD_Mode;
extern LGD_UINT32 			g_uiKOREnsembleFullFreq[MAX_KOREABAND_FULL_CHANNEL];

extern LGD_UINT8			abyBuff[MAX_FIC_SIZE];
extern LGD_UINT16			wFicLen;
extern ST_TRANSMISSION		m_ucTransMode;

/*============================================================
**    3.   External Functions
*============================================================*/
#ifdef STREAM_SLAVE_PARALLEL_UPLOAD
extern void	tdmb_tunerbb_data_cb(void);
extern void	FIC_Send_ISR_Sig(void);
#endif
extern int tdmb_lg2102_power_on(void);
extern int tdmb_lg2102_power_off(void);
extern int tdmb_lg2102_i2c_write_burst(uint16 waddr, uint8* wdata, int length);
extern int tdmb_lg2102_i2c_read_burst(uint16 raddr, uint8* rdata, int length);
extern void tdmb_lg2102_set_userstop(void);;

/*============================================================
**    4.   Local constant variables
*============================================================*/

/*============================================================
**    5.   Local Typedef
*============================================================*/
typedef enum	lg2102_service_type
{
	LG2102_DAB = 1,
	LG2102_DMB = 2,
	LG2102_VISUAL =3,
	LG2102_DATA = 4,
	LG2102_ENSQUERY = 6,	/* LGE Added */
	LG2102_BLT_TEST = 9,	/* LGE Added */
	LG2102_SERVICE_MAX
} lg2102_service_type;

/*============================================================
**    6.   Global Variables
*============================================================*/
lg2102_service_type	serviceType;

/*============================================================
**    7.   Static Variables
*============================================================*/
#ifdef LGD_MULTI_CHANNEL_ENABLE
static LGD_UINT8 initBuff[LGD_INTERRUPT_SIZE];
static LGD_UINT8 g_subch_id[LGD_MULTI_MAX_CHANNEL];
static LGD_UINT8 g_datatype[LGD_MULTI_MAX_CHANNEL];
#endif

static ST_SUBCH_INFO g_stSubInfo;
/*============================================================
**    8.   Local Function Prototype
*============================================================*/
#if 0//ndef LGE_MASS_PRODUCTION
static void print_msc_ber_scan(boolean bnormal);
#endif
int8 tunerbb_drv_lg2102_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[], uint8 op_mode[]);

int8 tunerbb_drv_lg2102_power_on(void)
{
	return tdmb_lg2102_power_on();
}

int8 tunerbb_drv_lg2102_power_off(void)
{
	return tdmb_lg2102_power_off();
}

int8 tunerbb_drv_lg2102_reset_ch(void)
{
	return INTERFACE_RESET_CH(TDMB_RFBB_DEV_ADDR);
}

int8 tunerbb_drv_lg2102_init(void)
{
	LGD_UINT8 nRet;

#if defined(STREAM_SLAVE_PARALLEL_UPLOAD)  	// if EBI interface
	m_ucCommandMode = LGD_EBI_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_SLAVE_PARALLEL;
#elif defined(STREAM_TS_UPLOAD)	/* if TSIF interface */  
	m_ucCommandMode = LGD_I2C_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_TS;
#elif defined(STREAM_SPI_UPLOAD)	// if SPI interface 
	m_ucCommandMode = LGD_SPI_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_SPI;
#endif

	m_ucPLL_Mode		= INPUT_CLOCK_24576KHZ;
	m_ucMPI_CS_Active	= LGD_ACTIVE_HIGH;
	m_ucMPI_CLK_Active	= LGD_ACTIVE_LOW;
	m_unIntCtrl 		= (LGD_INTERRUPT_POLARITY_LOW| \
						   LGD_INTERRUPT_PULSE | \
						   LGD_INTERRUPT_AUTOCLEAR_ENABLE | \
						   (LGD_INTERRUPT_PULSE_COUNT & LGD_INTERRUPT_PULSE_COUNT_MASK));

	nRet = INTERFACE_INIT(TDMB_RFBB_DEV_ADDR);
	if(nRet!=LGD_SUCCESS)
	{
		printk("[LG2102] INTERFACE_INIT() = (%d)\n", nRet);
		//return 1; // NOT_OK
	}

	return nRet;
}


int8 tunerbb_drv_lg2102_stop(void)
{
	LGD_UINT8 nRet = LGD_ERROR;

	nRet = LGD_STOP(TDMB_RFBB_DEV_ADDR);
	if (nRet != LGD_SUCCESS)
	{
		printk("[LG2102] LGD_STOP() = (%d)\n", nRet);
		//return LGD_ERROR; // NOT_OK
	}

	return nRet;
}

int8 tunerbb_drv_lg2102_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	LGD_UINT8 nRet;

	nRet = INTERFACE_STATUS_CHECK(TDMB_RFBB_DEV_ADDR);
	
	if(nRet == LGD_SUCCESS)
	{
		dmb_bb_info->sync_lock	= 1;
		dmb_bb_info->cir		= 1;
		dmb_bb_info->dab_ok 	= 1;		
		dmb_bb_info->sch_ber	= 1;
		dmb_bb_info->afc_ok 	= 1;	
		// msc_ber : BER viterbi Befor(VB BER) 10-5
		dmb_bb_info->msc_ber	= INTERFACE_GET_CER(TDMB_RFBB_DEV_ADDR) * 10;
		if(( LG2102_DMB == serviceType) || (LG2102_VISUAL == serviceType))
		{
			// va_ber : BER viterbi after (VA BER) 10-5
			dmb_bb_info->va_ber = INTERFACE_GET_PREBER(TDMB_RFBB_DEV_ADDR);
			//printk("[LGD] ^___^ va_ber = (%d)\n", dmb_bb_info->va_ber);

			dmb_bb_info->tp_lock = (nRet==LGD_SUCCESS)?TRUE:FALSE;
			//printk("[LGD]^__^ tp_lock = (%d)=\n", dmb_bb_info->tp_lock);
		}

		//dmb_bb_info->tp_err_cnt = INTERFACE_GET_POSTBER(TDMB_RFBB_DEV_ADDR); /* GET_POSTERBER is tp error rate : LG2102 */
		dmb_bb_info->tp_err_cnt = (uint32)INTERFACE_GET_TPERRCNT(TDMB_RFBB_DEV_ADDR);
	}
	else
	{
		dmb_bb_info->sync_lock	= 0;
		dmb_bb_info->dab_ok 	= 0;
		dmb_bb_info->sch_ber	= 0;
		dmb_bb_info->afc_ok 	= 0;	
		dmb_bb_info->msc_ber 	= 20000;
		dmb_bb_info->va_ber 	= 20000;
		dmb_bb_info->tp_lock 	= 0;
		dmb_bb_info->tp_err_cnt 	= 255;
	}	
	
	return LGD_SUCCESS;
}


int8 tunerbb_drv_lg2102_get_msc_ber(uint32 *msc_ber)
{
	*msc_ber	= INTERFACE_GET_CER(TDMB_RFBB_DEV_ADDR) * 10;

	return 1;
}

int8 tunerbb_drv_lg2102_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
	int8 ret_val;

	ret_val = tunerbb_drv_lg2102_multi_set_channel(freq_num, 1, &subch_id, &op_mode);

	return ret_val;
}


//////////////////////////////////////////////////////
int8 tunerbb_drv_lg2102_re_syncdetector(uint8 op_mode)
{
	int8 ret_val = LGD_ERROR;

	if(op_mode != LG2102_ENSQUERY)
	{		
		ret_val = (int8)INTERFACE_RE_SYNCDETECTOR(TDMB_RFBB_DEV_ADDR, &g_stSubInfo);
	}

	return ret_val;
}
////////////////////////////////////////////////////
int8 tunerbb_drv_lg2102_re_sync(void)
{
	int8 ret_val;
	
	ret_val = INTERFACE_RE_SYNC(TDMB_RFBB_DEV_ADDR);

	return ret_val;
}



int8 tunerbb_drv_lg2102_control_fic(uint8 enable)
{
	return LGD_ERROR;
}



uint32 tunerbb_drv_lg2102_get_freq(int nFreqIndex)
{
	int major_ch, minor_ch, fnindex;
	
	major_ch = nFreqIndex /10;
	minor_ch = nFreqIndex %10;
	if(major_ch<7)	major_ch = 7;
	else if(major_ch>13) major_ch = 13;
	if(minor_ch<1)	minor_ch = 1;
	else if(minor_ch>3) minor_ch = 3;	
	
	fnindex = (major_ch-7)*3 + (minor_ch-1);

	return g_uiKOREnsembleFullFreq[fnindex];
}


/*********************************************************************************/
/* Sync lock 체크 함수                                                           */
/*********************************************************************************/
int8 tunerbb_drv_lg2102_tune(int nFreqNo)
{
	uint32 ulFreq;
	LGD_UINT8 nRet = LGD_ERROR;
	ulFreq = tunerbb_drv_lg2102_get_freq(nFreqNo);

	printk("tunerbb_drv_lg2102_tune nFreqNo = %d, ulFreq = %d \n", nFreqNo, ulFreq);

	nRet = LGD_READY(TDMB_RFBB_DEV_ADDR, ulFreq);
	if(nRet != LGD_ERROR)
	{
		printk("tunerbb_drv_lg2102_tune READY OK = %d \n", nRet);
		nRet = LGD_SYNCDETECTOR(TDMB_RFBB_DEV_ADDR, ulFreq, 0);
	}
	
	printk("tunerbb_drv_lg2102_tune result = %d \n", nRet);

	return nRet;
}


/*********************************************************************************/
/* FIC 데이터 읽는 함수 입니다. 98ms마다 호출                                    */
/*********************************************************************************/
int8 tunerbb_drv_lg2102_get_fic(uint8* buffer, uint32* buffer_size, boolean crc_onoff)
{
	LGD_UINT32 	uiFicSize;
	//LGD_UINT8 	aucBuff[384];

	if((LGD_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_VTB_BASE+ 0x00) & 0x4000) == LGD_ERROR)	{
		*buffer_size = 0;
		return LGD_ERROR;
	}

	uiFicSize = (LGD_UINT32)LGD_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_VTB_BASE+ 0x09) + 1;
	if(uiFicSize == 1 /*|| uiFicSize != 384*/)	{
		*buffer_size = 0;
		return LGD_ERROR;		
	}

	printk("tunerbb_drv_lg2102_get_fic first\n");
	
	if(LGD_CMD_READ_BURST(TDMB_RFBB_DEV_ADDR, APB_FIC_BASE, buffer, uiFicSize) == LGD_SUCCESS)	{
		//memcpy(buffer, aucBuff, uiFicSize);
		*buffer_size = uiFicSize;
		printk("tunerbb_drv_lg2102_get_fic = %x %x %x %x %x\n", *buffer, *(buffer+1), *(buffer+2), *(buffer+3), *(buffer+4));
		return LGD_SUCCESS;
	}

	return LGD_ERROR;	
}


int8 tunerbb_drv_lg2102_read_data(uint8* buffer, uint32* buffer_size)
{
	if(INTERFACE_ISR(TDMB_RFBB_DEV_ADDR, buffer)!=LGD_SUCCESS)
	{
		*buffer_size = 0;
		return LGD_ERROR;
	}

	*buffer_size = LGD_INTERRUPT_SIZE;
	
	return LGD_SUCCESS;
}

int8 tunerbb_drv_lg2102_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[], uint8 op_mode[])
{
	int major_ch, minor_ch, fnindex;
	int i;
	LGD_INT16 nLoop;
	LGD_CHANNEL_INFO ChInfo[LGD_MULTI_MAX_CHANNEL];
	LGD_ERROR_INFO ErrorInfo;
	ST_SUBCH_INFO stSubInfo;

	memset(&stSubInfo, 0x0, sizeof(ST_SUBCH_INFO));
	
	if(subch_cnt>LGD_MULTI_MAX_CHANNEL) 
	{
		return ERROR;
	}

	serviceType = (lg2102_service_type)op_mode[0];

	major_ch = freq_num /10;
	minor_ch = freq_num %10;
	if(major_ch<7)	major_ch = 7;
	else if(major_ch>13) major_ch = 13;
	if(minor_ch<1)	minor_ch = 1;
	else if(minor_ch>3) minor_ch = 3;

	// index of function point..
	fnindex = (major_ch-7)*3 + (minor_ch-1);

	if(op_mode[0]==LG2102_ENSQUERY)
	{
		if(INTERFACE_SCAN(TDMB_RFBB_DEV_ADDR, g_uiKOREnsembleFullFreq[fnindex])==LGD_SUCCESS)
		{
			return LGD_SUCCESS;
		}
	}
	else
	{
		for(i=0;i<subch_cnt;i++)
		{
			switch(op_mode[i])
			{
				case LG2102_DAB:					
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					ChInfo[i].ucServiceType = 0x00;
					ChInfo[i].uiTmID = TMID_0;
					ChInfo[i].ucDataType = TDMB_BB_DATA_DAB;
					ChInfo[i].ulDataThreshold = 188*10;
					break;
				case LG2102_DATA:
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					ChInfo[i].ucServiceType = 0x00;
					ChInfo[i].uiTmID = TMID_2;	
					ChInfo[i].ucDataType = TDMB_BB_DATA_PACK;
					ChInfo[i].ulDataThreshold = 288;//188;
					break;
				case LG2102_DMB:
				case LG2102_VISUAL:
				case LG2102_BLT_TEST:
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					ChInfo[i].ucServiceType = 0x18;
					ChInfo[i].uiTmID = TMID_1;
					ChInfo[i].ucDataType = TDMB_BB_DATA_TS;
					ChInfo[i].ulDataThreshold = 188*32;
					break;
				default:
					return LGD_ERROR;
			}
			memcpy(&stSubInfo.astSubChInfo[i], &ChInfo[i], sizeof(LGD_CHANNEL_INFO));
		}

		stSubInfo.nSetCnt = subch_cnt;
		LGD_MULTI_SORT_INIT();

		memcpy(&g_stSubInfo, &stSubInfo, sizeof(ST_SUBCH_INFO));
		
		for(nLoop = 0; nLoop < 2; nLoop++)
		{
			if(INTERFACE_START(TDMB_RFBB_DEV_ADDR, &stSubInfo)) return LGD_SUCCESS;
			ErrorInfo = INTERFACE_ERROR_STATUS(TDMB_RFBB_DEV_ADDR);
			printk("[LGD]^__^ INTERFACE_ERROR_STATUS = (0x%04x)\n", ErrorInfo);
			if(ErrorInfo == ERROR_SYNC_NULL || ErrorInfo == ERROR_FICDECODER || 
				ErrorInfo == ERROR_SYNC_TIMEOUT) continue; // 약전계시 한번더 호출함. 
			else return LGD_ERROR;
		}
	}

	return LGD_ERROR;
}


#ifdef LGD_MULTI_CHANNEL_ENABLE
int8 tunerbb_drv_lg2102_multi_read_data(uint8 subch_cnt, uint8* buffer, uint32* read_size)
{
	LGD_UINT8*	IntBuff = &initBuff[0];
	ST_FIFO*	pMultiFF;
	int			nDataSize;
	int			i;
	LGD_UINT32	buf_size = 0;
	TDMB_BB_HEADER_TYPE dmb_header;
	LGD_UINT16	uFIBCnt;

	if(buffer == NULL || read_size == NULL)
	{
		return LGD_ERROR;
	}

	if(INTERFACE_ISR(TDMB_RFBB_DEV_ADDR, IntBuff)!=LGD_SUCCESS)
	{
		return LGD_ERROR;
	}

	if(subch_cnt>LGD_MULTI_MAX_CHANNEL) 
	{
		return LGD_ERROR;
	}
	
	if(LGD_MULTI_FIFO_PROCESS(IntBuff, LGD_INTERRUPT_SIZE))
	{

#ifdef LGD_MULTI_CHANNEL_FIC_UPLOAD
		// (i==1) => FIC_STREAM_DATA,
		pMultiFF = LGD_GET_CHANNEL_FIFO(FIC_STREAM_DATA);
		uFIBCnt = LGD_GET_FIB_CNT(m_ucTransMode);
		nDataSize = LGD_QFIFO_GET_SIZE(pMultiFF);
		if(nDataSize >= (uFIBCnt*FIB_SIZE))
		{
			wFicLen = (uFIBCnt*FIB_SIZE);
			LGD_QFIFO_BRING(pMultiFF, abyBuff, wFicLen);
#ifdef LGD_EWS_SOURCE_ENABLE  
			if(LGD_EWS_PARSING(abyBuff, wFicLen)==LGD_SUCCESS)
			{
				ST_OUTPUT_EWS*	pstEWSMsg;
				LGD_GET_EWS_DB(pstEWSMsg);
				///////////////////////////
				// TO-DO : process EWS Data
				///////////////////////////
			}
#endif			
		}
#endif

		for(i=0;i<subch_cnt;i++)
		{
			/************************************************
			(i==2) => CHANNEL1_STREAM_DATA,
			(i==3) => CHANNEL2_STREAM_DATA,
			(i==4) => CHANNEL3_STREAM_DATA,
			************************************************/
			pMultiFF = LGD_GET_CHANNEL_FIFO(i+2);
			nDataSize = LGD_QFIFO_GET_SIZE(pMultiFF);
			if(nDataSize>=LGD_INTERRUPT_SIZE)
			{
				dmb_header.reserved = 0xDEAD;
				dmb_header.data_type = g_datatype[i];
				dmb_header.size = nDataSize;
				dmb_header.subch_id = g_subch_id[i];
				memcpy(buffer,&dmb_header,sizeof(TDMB_BB_HEADER_TYPE));
				buffer += sizeof(TDMB_BB_HEADER_TYPE);
				buf_size += sizeof(TDMB_BB_HEADER_TYPE);
				LGD_QFIFO_BRING(pMultiFF, buffer, nDataSize);
				buffer += nDataSize;
				buf_size += nDataSize;
			}			
		}
	}

	*read_size = buf_size;

	return LGD_SUCCESS;
}

int8 tunerbb_drv_lg2102_process_multi_data(uint8 subch_cnt,uint8* input_buf, uint32 input_size, uint32* read_size)
{
	ST_FIFO*	pMultiFF;
	int			nDataSize;
	int			i;
	LGD_UINT32	buf_size = 0;

	if(input_buf == NULL || read_size == NULL)
	{
		return LGD_ERROR;
	}
	
	if(subch_cnt>LGD_MULTI_MAX_CHANNEL) 
	{
		return LGD_ERROR;
	}

	if(LGD_MULTI_FIFO_PROCESS(input_buf, input_size))
	{

		for(i=0;i<subch_cnt;i++)
		{
			/************************************************
			(i==2) => CHANNEL1_STREAM_DATA,
			(i==3) => CHANNEL2_STREAM_DATA,
			(i==4) => CHANNEL3_STREAM_DATA,
			************************************************/
			pMultiFF = LGD_GET_CHANNEL_FIFO(i+2);
			nDataSize = LGD_QFIFO_GET_SIZE(pMultiFF);
			if(nDataSize>=LGD_INTERRUPT_SIZE)
			{				
				buf_size += sizeof(TDMB_BB_HEADER_TYPE) + nDataSize;
			}			
		}
	}
	else
	{
		return LGD_ERROR;
	}

	*read_size = buf_size;

	return LGD_SUCCESS;
}

int8 tunerbb_drv_lg2102_get_multi_data(uint8 subch_cnt, uint8* buf_ptr, uint32 buf_size)
{
	ST_FIFO*	pMultiFF;
	int			nDataSize;
	int			i;
	TDMB_BB_HEADER_TYPE dmb_header;
	LGD_UINT16	uFIBCnt;
	LGD_UINT32	read_size = 0;

	if(buf_ptr == NULL || buf_size == 0)
	{
		return LGD_ERROR;
	}
	
	if(subch_cnt>LGD_MULTI_MAX_CHANNEL) 
	{
		return LGD_ERROR;
	}
	
#ifdef LGD_MULTI_CHANNEL_FIC_UPLOAD
	// (i==1) => FIC_STREAM_DATA,
	pMultiFF = LGD_GET_CHANNEL_FIFO(FIC_STREAM_DATA);
	uFIBCnt = LGD_GET_FIB_CNT(m_ucTransMode);
	nDataSize = LGD_QFIFO_GET_SIZE(pMultiFF);
	if(nDataSize >= (uFIBCnt*FIB_SIZE))
	{
		wFicLen = (uFIBCnt*FIB_SIZE);
		LGD_QFIFO_BRING(pMultiFF, abyBuff, wFicLen);
#ifdef LGD_EWS_SOURCE_ENABLE  
		if(LGD_EWS_PARSING(abyBuff, wFicLen)==LGD_SUCCESS)
		{
			ST_OUTPUT_EWS*	pstEWSMsg;
			LGD_GET_EWS_DB(pstEWSMsg);
			///////////////////////////
			// TO-DO : process EWS Data
			///////////////////////////
		}
#endif			
	}
#endif

	for(i=0;i<subch_cnt;i++)
	{
		/************************************************
		(i==2) => CHANNEL1_STREAM_DATA,
		(i==3) => CHANNEL2_STREAM_DATA,
		(i==4) => CHANNEL3_STREAM_DATA,
		************************************************/
		pMultiFF = LGD_GET_CHANNEL_FIFO(i+2);
		nDataSize = LGD_QFIFO_GET_SIZE(pMultiFF);
		if(nDataSize>=LGD_INTERRUPT_SIZE)
		{
			dmb_header.reserved = 0xDEAD;
			dmb_header.data_type = g_datatype[i];
			dmb_header.size = nDataSize;
			dmb_header.subch_id = g_subch_id[i];
			memcpy(buf_ptr,&dmb_header,sizeof(TDMB_BB_HEADER_TYPE));
			buf_ptr += sizeof(TDMB_BB_HEADER_TYPE);
			read_size += sizeof(TDMB_BB_HEADER_TYPE);
			LGD_QFIFO_BRING(pMultiFF, buf_ptr, nDataSize);
			buf_ptr += nDataSize;
			read_size += nDataSize;
		}			
	}

	if(read_size != buf_size)
	{
		return LGD_ERROR;		
	}
	else
	{
		return LGD_SUCCESS;
	}
}
#endif

void tunerbb_drv_lg2102_start_tii(void)
{
	LGD_TII_START(TDMB_RFBB_DEV_ADDR);
	//MSG_FATAL("=== LGD_TII_START(%d) \n",thr);
}

void tunerbb_drv_lg2102_stop_tii(void)
{
	LGD_TII_STOP(TDMB_RFBB_DEV_ADDR);
	//MSG_FATAL("=== LGD_TII_STOP() \n");
}

boolean tunerbb_drv_lg2102_check_tii(uint8* pmain_tii, uint8* psub_tii)
{
	struct _tagST_TII_INFO stTIIInfo[MAX_TII_CNT];
	LGD_UINT16 uiStatus = 0;

	memset(stTIIInfo, 0x00, (sizeof(struct _tagST_TII_INFO)*MAX_TII_CNT));
	
	uiStatus = LGD_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_PHY_BASE+ 0x2D);

	if(uiStatus == 0x1F7F) return FALSE;
	
	LGD_TII_GET_INFO(TDMB_RFBB_DEV_ADDR,stTIIInfo);

	printk(" ==> TII0 [MainID(0x%X), SubID(0x%X), Strength(%d)] \n", stTIIInfo[0].uiPattern, stTIIInfo[0].uiSubID, stTIIInfo[0].uiStrength);

	*pmain_tii = (uint8)stTIIInfo[0].uiPattern;
	*psub_tii = (uint8)stTIIInfo[0].uiSubID;
	
	return TRUE;
}


void tunerbb_drv_lg2102_set_userstop(void)
{
	tdmb_lg2102_set_userstop( );
}

#if 0//ndef LGE_MASS_PRODUCTION
static void print_msc_ber_scan(boolean bnormal)
{
	/* For Debugging */
	/* ---------------------------------------*/
	unsigned short msc_ber;

	msc_ber = INTERFACE_GET_CER(TDMB_RFBB_DEV_ADDR);;

	if(bnormal)
	{
		printk("[LGD] ^___^ Channel Search msc_ber = (%d)", msc_ber);
	}
	else
	{
		printk("[LGD] ^___^ Channel Search  but get FIC Data msc_ber = (%d)", msc_ber);

	}
}
#endif

