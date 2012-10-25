/* drivers/broadcast/lg2102/src/tdmb_tunerbbdrv_lg2102.c
 * Copyright (C) 2011 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_lg2102.h"
#include "../inc/tdmb_tunerbbdrv_lg2102def.h"
#include "../inc/INC_INCLUDES.h"

/* ----------------------------------------------------------
**    1.   DEFINITIONS
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    2.   External variables
-------------------------------------------------------------*/

extern CTRL_MODE 			m_ucCommandMode;
extern ST_TRANSMISSION		m_ucTransMode;
extern UPLOAD_MODE_INFO	m_ucUploadMode;
extern CLOCK_SPEED			m_ucClockSpeed;
extern INC_ACTIVE_MODE		m_ucMPI_CS_Active;
extern INC_ACTIVE_MODE		m_ucMPI_CLK_Active;
extern INC_UINT16			m_unIntCtrl;
extern PLL_MODE			m_ucPLL_Mode;
extern INC_DPD_MODE		m_ucDPD_Mode;
extern INC_UINT32 			g_uiKOREnsembleFullFreq[MAX_KOREABAND_FULL_CHANNEL];


/* ----------------------------------------------------------
**    3.   External Functions
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    4.   Local Constant Variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    5.   Local Typedef
-------------------------------------------------------------*/
typedef enum lg2102_service_type
{
	LG2102_DAB = 1,
	LG2102_DMB = 2,
	LG2102_VISUAL =3,
	LG2102_DATA = 4,
	LG2102_ENSQUERY = 6,	/* LGE Added */
	LG2102_BLT_TEST = 9, /* LGE Added */
	LG2102_SERVICE_MAX
}lg2102_service_type;

/* ----------------------------------------------------------
**    6.   Global Variables
-------------------------------------------------------------*/
lg2102_service_type	serviceType;

/* ----------------------------------------------------------
**    7.   Static Variables
-------------------------------------------------------------*/
static ST_SUBCH_INFO g_stSubInfo;

void tunerbb_drv_lg2102_set_userstop(int mode)
{
	tdmb_lg2102_set_userstop( mode );
}

int8 tunerbb_drv_lg2102_power_on(void)
{
	int rc;
	rc = tdmb_lg2102_power_on( );
	return rc;
}

int8 tunerbb_drv_lg2102_power_off(void)
{
	int rc;
	rc = tdmb_lg2102_power_off( );
	return rc;
}

int8 tunerbb_drv_lg2102_select_antenna(unsigned int sel)
{
	int rc;
	rc = tdmb_lg2102_select_antenna(sel);
	return rc;
}

int8 tunerbb_drv_lg2102_reset_ch(void)
{

	return (INTERFACE_RESET(TDMB_RFBB_DEV_ADDR));

}

void tunerbb_drv_lg2102_rw_test(void)
{
	unsigned short i = 0;
	unsigned short w_val = 0;
	unsigned short r_val = 0;
	unsigned short err_cnt = 0;

	err_cnt = 0;
	for(i=1;i<30;i++)
	{
		w_val = (i%0xFF);
		INC_CMD_WRITE(TDMB_RFBB_DEV_ADDR, 0x0A00+ 0x01, w_val) ;;
		r_val = INC_CMD_READ(TDMB_RFBB_DEV_ADDR, 0x0A00+ 0x01);
		if(r_val != w_val)
		{
			err_cnt++;
			printk("[rw fail] w_val:%x, r_val:%x, err_cnt = %d\n", w_val,r_val,err_cnt);
		}
		if(err_cnt == 0){
			printk("lg2102 interface test ok...\n");
		}
	}
}

int8 tunerbb_drv_lg2102_init(void)
{
	INC_UINT8 nRet;
	
	//tunerbb_drv_lg2102_rw_test( );   // for test

#if defined(STREAM_SLAVE_PARALLEL_UPLOAD)  	// if EBI interface
	m_ucCommandMode = INC_EBI_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_SLAVE_PARALLEL;
#elif defined(STREAM_TS_UPLOAD)	/* if TSIF interface */  
	m_ucCommandMode = INC_I2C_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_TS;
#elif defined(STREAM_SPI_UPLOAD)	// if SPI interface 
	m_ucCommandMode = INC_SPI_CTRL;
	m_ucUploadMode = STREAM_UPLOAD_SPI;
#endif

	m_ucPLL_Mode = INPUT_CLOCK_24576KHZ;
	m_ucMPI_CS_Active = INC_ACTIVE_HIGH;
	m_ucMPI_CLK_Active = INC_ACTIVE_LOW;
	m_unIntCtrl 		= (INC_INTERRUPT_POLARITY_LOW| \
					INC_INTERRUPT_PULSE | \
 	  			   	INC_INTERRUPT_AUTOCLEAR_ENABLE| \
					(INC_INTERRUPT_PULSE_COUNT&INC_INTERRUPT_PULSE_COUNT_MASK));

	nRet = INTERFACE_INIT(TDMB_RFBB_DEV_ADDR);
	if(nRet!=INC_SUCCESS)
	{
		printk("[LG2102] INTERFACE_INIT() = (%d)\n", nRet);
		//return 1; // NOT_OK
	}

	return nRet;

}

int8 tunerbb_drv_lg2102_stop(void)
{
	INC_UINT8 nRet;

	nRet = INC_STOP(TDMB_RFBB_DEV_ADDR);
	if(nRet != INC_SUCCESS)
	{
		printk("[LG2102] INC_STOP Error\n");
	}
	return nRet;
}

int8 tunerbb_drv_lg2102_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	ST_BBPINFO* pInfo;
	int32 svc_type = 0;  /* DAB 1, DMB,Visual  2 */

	uint16 unCER, unLoop, unRefAntLevel = 0;
	uint16 aunAntTable[5][2] = {
		{4,    550},
		{3,    700},
		{2,    950},
		{1,    1150},   //910, 960
		{0,    10000},
	};
	
	INTERFACE_STATUS_CHECK(TDMB_RFBB_DEV_ADDR);
	pInfo = INC_GET_STRINFO(TDMB_RFBB_DEV_ADDR);

	if(LG2102_DAB == serviceType)
	{
		svc_type = 1;
	}
	else if((LG2102_DMB == serviceType) || (LG2102_VISUAL == serviceType))
	{
		svc_type = 2;
	}
	
	if(pInfo->ucSyncLock == 1)
	{
		dmb_bb_info->sync_lock	= 1;
		dmb_bb_info->cir		= 1;
		dmb_bb_info->dab_ok 	= 1;		
		dmb_bb_info->sch_ber	= 1;
	}
	else
	{
		dmb_bb_info->sync_lock	= 0;
		dmb_bb_info->dab_ok 	= 0;
		dmb_bb_info->sch_ber	= 0;
		if(pInfo->ucSyncLock != 0)
		{
			printk("[INC]pInfo->ucSyncLock is not zero need to be initialized?, %d\n", pInfo->ucSyncLock);
		}
	}
	dmb_bb_info->afc_ok 	= 1;	
	
	/* va_ber , tp_lock and tp_err_cnt are valied in DMB stream not DAB stream */
	if(svc_type == 2)
	{
		dmb_bb_info->va_ber = (uint32)(pInfo->uiPostBER);
		dmb_bb_info->tp_lock = pInfo->ucSyncLock;
		dmb_bb_info->tp_err_cnt = (uint32)INTERFACE_GET_TPERRCNT(TDMB_RFBB_DEV_ADDR);
	}
	else  /* DAB Service */
	{
		dmb_bb_info->va_ber = 0;
		dmb_bb_info->tp_lock = 0;
		dmb_bb_info->tp_err_cnt = 0;
	}

	// msc_ber : BER viterbi Befor(VB BER) 10-5
	dmb_bb_info->msc_ber	= (uint32)(pInfo->uiCER* 10);  /* MSC BER */

	/* Calculation Ant. Level INC Tech. Refer to INC_GET_ANT_LEVEL( ) function in INC_PROCESS.c*/
	/* START Calculation */
	unCER = pInfo->uiCER;

	if(svc_type == 1) /* DAB */
	{
		unCER = pInfo->uiCER + ((pInfo->uiCER/4));
	}

	for(unLoop = 0; unLoop < 4; unLoop++)
	{
		if(unCER <= aunAntTable[unLoop][1]) {
			unRefAntLevel = aunAntTable[unLoop][0];
			break ;
		}
	}

	/* These bleow routines are for DMB case not DAB */
	if((svc_type == 2) && (unRefAntLevel == 0) && (pInfo->uiCER < 1300) && (pInfo->ucVber >= 50))
	 unRefAntLevel+=1;

	if((svc_type == 2) && (unRefAntLevel == 1) && (pInfo->ucVber < 50))
		unRefAntLevel-=1;

	if((svc_type == 2) &&(unRefAntLevel == 2) && (pInfo->ucVber <= 50))
	  unRefAntLevel -=1;


	if((pInfo->ucAntLevel == unRefAntLevel) || (pInfo->ucChannelChange == 1))
	{
		pInfo->ucAntLevel = unRefAntLevel;
		pInfo->ucChannelChange = 0;
	}
	else if(pInfo->ucAntLevel >= unRefAntLevel)
	{
		if((pInfo->ucAntLevel - unRefAntLevel) >= 2) pInfo->ucAntLevel -= 1;
		else pInfo->ucAntLevel--;
	}
	else {
		if((unRefAntLevel - pInfo->ucAntLevel) >= 2) pInfo->ucAntLevel += 1;
		else pInfo->ucAntLevel++;
	}

	/* sync unlock status Ant Level is 0 */
	if(dmb_bb_info->sync_lock == 0)
	{
		pInfo->ucAntLevel = 0;
	}

	dmb_bb_info->antenna_level = pInfo->ucAntLevel;
	/* End Calculation */

	return INC_SUCCESS;
}

int8 tunerbb_drv_lg2102_get_msc_ber(uint32 *msc_ber)
{
	*msc_ber	= INTERFACE_GET_CER(TDMB_RFBB_DEV_ADDR) * 10;
	return INC_SUCCESS;
}


int8 tunerbb_drv_lg2102_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
	return tunerbb_drv_lg2102_multi_set_channel(freq_num, 1, &subch_id, &op_mode);
}

int8 tunerbb_drv_lg2102_re_syncdetector(uint8 op_mode)
{
	int8 ret_val = INC_ERROR;
	
	if(op_mode != LG2102_ENSQUERY)
	{		
		ret_val = INTERFACE_RE_SYNCDETECTOR(TDMB_RFBB_DEV_ADDR, &g_stSubInfo);
	}

	return ret_val;
}

int8 tunerbb_drv_lg2102_re_sync(void)
{
	return INTERFACE_RE_SYNC(TDMB_RFBB_DEV_ADDR);
}

int8 tunerbb_drv_lg2102_control_fic(uint8 enable)
{
	return INC_ERROR;
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

int8 tunerbb_drv_lg2102_tune(int nFreqNo)
{
	uint32 ulFreq;
	INC_UINT8 nRet = INC_ERROR;

	ulFreq = tunerbb_drv_lg2102_get_freq(nFreqNo);
	//printk("tunerbb_drv_lg2102_tune nFreqNo = %d, ulFreq = %d \n", nFreqNo, ulFreq);

	nRet = INTERFACE_SCAN(TDMB_RFBB_DEV_ADDR,ulFreq);

	return nRet;
}


/*********************************************************************************/
/* FIC 데이터 읽는 함수 입니다. 98ms마다 호출                                    */
/*********************************************************************************/
//wonhee.jeong TDMB Porting lg2102 to ICS 20120218 (crc_onoff라는 변수는 사용안하는 듯 하니 t3900 구조로 가면서 없앰..)
//int8 tunerbb_drv_lg2102_get_fic(uint8* buffer, uint32* buffer_size, boolean crc_onoff)
int8 tunerbb_drv_lg2102_get_fic(uint8* buffer, uint32* buffer_size/* bool crc_onoff*/)
{
		INC_UINT32	uiFicSize;
		INC_UINT8	ret;
		
		ret = INTERFACE_GET_FIC(TDMB_RFBB_DEV_ADDR,&uiFicSize);

		if(ret == INC_ERROR)
		{
			*buffer_size = 0;
			return INC_ERROR;
		}
		else if(ret == INC_SUCCESS)
		{
			if(uiFicSize > 384)
			{
				printk("tunerbb_drv_lg2102_get_fic uiFicSize = (%d) is over 384\n", uiFicSize);
				uiFicSize = 384;
			}	

			if(INC_CMD_READ_BURST(TDMB_RFBB_DEV_ADDR, APB_FIC_BASE, buffer, uiFicSize) == INC_SUCCESS)	{
				*buffer_size = uiFicSize;
				//printk("tunerbb_drv_T39fx_get_fic = %x %x %x %x %x\n", *buffer, *(buffer+1), *(buffer+2), *(buffer+3), *(buffer+4));
			}
		
		}
			
		return INC_SUCCESS; 	
}

int8 tunerbb_drv_lg2102_read_data(uint8* buffer, uint32* buffer_size)
{
	TDMB_BB_HEADER_TYPE dmb_header;
	uint8* r_buffer = NULL;

#if 0	
	dmb_header.data_type = (serviceType == LG2102_DAB?TDMB_BB_DATA_DAB:TDMB_BB_DATA_TS);
	dmb_header.size = INC_INTERRUPT_SIZE;
	dmb_header.subch_id = 0;
	dmb_header.reserved = 0;//0xDEAD;
			
	memcpy(buffer, &dmb_header, sizeof(TDMB_BB_HEADER_TYPE));
			
	if(INTERFACE_ISR(TDMB_RFBB_DEV_ADDR, buffer + sizeof(TDMB_BB_HEADER_TYPE))!=INC_SUCCESS)
	{
		*buffer_size = 0;
		printk(" ISR READ Error! \n");
		return INC_ERROR;
	}

	*buffer_size = INC_INTERRUPT_SIZE + sizeof(TDMB_BB_HEADER_TYPE);
	return INC_SUCCESS;
#else
	uint32 r_size = 0;
	r_buffer = (buffer + sizeof(TDMB_BB_HEADER_TYPE));
	r_size = INTERFACE_ISR(TDMB_RFBB_DEV_ADDR, r_buffer);

	if(r_size == (uint32)INC_ERROR) {
		*buffer_size = 0;
		printk("ISR READ Error!!\n");
		return INC_ERROR;
	}

	dmb_header.data_type = (serviceType == LG2102_DAB?TDMB_BB_DATA_DAB:TDMB_BB_DATA_TS);
	dmb_header.size = r_size;
	dmb_header.subch_id = 0;
	dmb_header.reserved =0; //0xDEAD;

	memcpy(buffer, &dmb_header, sizeof(TDMB_BB_HEADER_TYPE));
	*buffer_size = r_size + sizeof(TDMB_BB_HEADER_TYPE);
	return INC_SUCCESS;
#endif
}


int8 tunerbb_drv_lg2102_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[], uint8 op_mode[])
{
	int i;
	//INC_INT16 nLoop;
	INC_ERROR_INFO ErrorInfo;

	uint32 set_freq;			 
	
	if(subch_cnt>INC_MULTI_MAX_CHANNEL) 
	{
		return INC_ERROR;
	}

	serviceType = (lg2102_service_type)op_mode[0];

	set_freq = tunerbb_drv_lg2102_get_freq(freq_num);

	if(op_mode[0]==LG2102_ENSQUERY)
	{
		if(INTERFACE_SCAN(TDMB_RFBB_DEV_ADDR, set_freq)==INC_SUCCESS)
		{
			return INC_SUCCESS;
		}
	}
	else
	{
		
#ifdef LGE_FW_LARGE_STACK // for build error 
		INC_CHANNEL_INFO ChInfo[INC_MULTI_MAX_CHANNEL];
		ST_SUBCH_INFO stSubInfo;

		for(i=0;i<subch_cnt;i++)
		{
			switch(op_mode[i])
			{
				case LG2102_DAB:					
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = set_freq;
					ChInfo[i].ucServiceType = 0x00;
					ChInfo[i].uiTmID = TMID_0;
					ChInfo[i].ucDataType = TDMB_BB_DATA_DAB;
					ChInfo[i].ulDataThreshold = 188*10;
					break;
				case LG2102_DATA:
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = set_freq;
					ChInfo[i].ucServiceType = 0x3C;
					ChInfo[i].uiTmID = TMID_3;  
					ChInfo[i].ucDataType = TDMB_BB_DATA_PACK;
					ChInfo[i].ulDataThreshold = 288*10;//288;
					break;
				case LG2102_DMB:
				case LG2102_VISUAL:
				case LG2102_BLT_TEST:
					ChInfo[i].ucSubChID = subch_id[i];
					ChInfo[i].ulRFFreq = set_freq;
					ChInfo[i].ucServiceType = 0x18;
					ChInfo[i].uiTmID = TMID_1;
					ChInfo[i].ucDataType = TDMB_BB_DATA_TS;
					ChInfo[i].ulDataThreshold = 188*32;
					break;
				default:
					return INC_ERROR;
			}
			memcpy(&stSubInfo.astSubChInfo[i], &ChInfo[i], sizeof(INC_CHANNEL_INFO));
		}

		stSubInfo.nSetCnt = subch_cnt;
		INC_MULTI_SORT_INIT();

		memcpy(&g_stSubInfo, &stSubInfo, sizeof(ST_SUBCH_INFO));

		//for(nLoop = 0; nLoop < 2; nLoop++)
		{
			if(INTERFACE_START(TDMB_RFBB_DEV_ADDR, &stSubInfo))
			{
				return INC_SUCCESS;
			}
			ErrorInfo = INTERFACE_ERROR_STATUS(TDMB_RFBB_DEV_ADDR);
			printk("[INC]^__^ INTERFACE_STATUS = (0x%04x)\n", ErrorInfo);
			if(ErrorInfo == ERROR_SYNC_NULL || ErrorInfo == ERROR_FICDECODER || 
				ErrorInfo == ERROR_SYNC_TIMEOUT) continue; // 약전계시 한번더 호출함. 
			else
			{
				return INC_ERROR;
			}
		}
#else
		for(i=0;i<subch_cnt;i++)
		{
			INC_CHANNEL_INFO *_ChInfo = &g_stSubInfo.astSubChInfo[i];
			switch(op_mode[i])
			{
				case LG2102_DAB:					
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = set_freq;
					_ChInfo->ucServiceType = 0x00;
					_ChInfo->uiTmID = TMID_0;
					_ChInfo->ucDataType = TDMB_BB_DATA_DAB;
					_ChInfo->ulDataThreshold = 188*10;
					break;
				case LG2102_DATA:
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = set_freq;
					_ChInfo->ucServiceType = 0x3C;
					_ChInfo->uiTmID = TMID_3;  
					_ChInfo->ucDataType = TDMB_BB_DATA_PACK;
					_ChInfo->ulDataThreshold = 288*10;//288;
					break;
				case LG2102_DMB:
				case LG2102_VISUAL:
				case LG2102_BLT_TEST:
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = set_freq;
					_ChInfo->ucServiceType = 0x18;
					_ChInfo->uiTmID = TMID_1;
					_ChInfo->ucDataType = TDMB_BB_DATA_TS;
					_ChInfo->ulDataThreshold = 188*32;
					break;
				default:
					return INC_ERROR;
			}
		}

		g_stSubInfo.nSetCnt = subch_cnt;
		printk("[INC_set_channel] subch_cnt = %d\n", subch_cnt);
		
		INC_MULTI_SORT_INIT();


		//for(nLoop = 0; nLoop < 2; nLoop++)
		{
			if(INTERFACE_START(TDMB_RFBB_DEV_ADDR, &g_stSubInfo))
			{
				return INC_SUCCESS;
			}
			ErrorInfo = INTERFACE_ERROR_STATUS(TDMB_RFBB_DEV_ADDR);
			printk("[INC]^__^ INTERFACE_STATUS = (0x%04x)\n", ErrorInfo);
			if(ErrorInfo == ERROR_SYNC_NO_SIGNAL || ErrorInfo == ERROR_FICDECODER ||
				ErrorInfo == ERROR_SYNC_TIMEOUT) return INC_ERROR;  
			else
			{
				return INC_ERROR;
			}
		}

#endif
	}

	return INC_ERROR;
}


int8 tunerbb_drv_lg2102_get_datatype(uint8 subchid, uint8* datatype)
{
	int i;

	for(i=0;i<g_stSubInfo.nSetCnt;i++)
	{
		if(g_stSubInfo.astSubChInfo[i].ucSubChID == subchid)
		{
			*datatype = g_stSubInfo.astSubChInfo[i].ucDataType;
			return INC_SUCCESS;
		}
	}

	return INC_ERROR;
}

int8 tunerbb_drv_lg2102_get_datathreshold(uint8 subchid, uint32* threshold)
{
	int i;

	for(i=0;i<g_stSubInfo.nSetCnt;i++)
	{
		if(g_stSubInfo.astSubChInfo[i].ucSubChID == subchid)
		{
			*threshold = g_stSubInfo.astSubChInfo[i].ulDataThreshold;
			return INC_SUCCESS;
		}
	}

	return INC_ERROR;
}

//#ifdef INC_MULTI_CHANNEL_ENABLE

int8 tunerbb_drv_lg2102_process_multi_data(uint8 subch_cnt,uint8* input_buf, uint32 input_size, uint32* read_size)
{
	ST_FIFO*	pMultiFF;
	uint32	nDataSize;
	int			i;
	uint32	buf_size = 0;
	//uint8* pBuff;   /* NOT USED */
	//INC_UINT16	uFIBCnt;
	uint32 threshold;

	if(input_buf == NULL || read_size == NULL)
	{
		printk("[T39fx] input_buf Error [input_buf:0x%x]",(uint32)input_buf);
		return INC_ERROR;
	}
	
	if(subch_cnt>INC_MULTI_MAX_CHANNEL) 
	{
		printk("[T39fx] subch_cnt Error [subch_cnt:%d]",subch_cnt);
		return INC_ERROR;
	}	

	if(INC_MULTI_FIFO_PROCESS(input_buf, input_size))
	{
		for(i=1;i<=subch_cnt+1;i++)
		{
			/************************************************
			(i==1) => FIC_STREAM_DATA,
			(i==2) => CHANNEL1_STREAM_DATA,
			(i==3) => CHANNEL2_STREAM_DATA,
			(i==4) => CHANNEL3_STREAM_DATA,
			************************************************/
			pMultiFF = INC_GET_CHANNEL_FIFO(i);
			if(i==1)
			{
#ifdef INC_MULTI_CHANNEL_FIC_UPLOAD
				uFIBCnt = INC_GET_FIB_CNT(m_ucTransMode);
				nDataSize = INC_QFIFO_GET_SIZE(pMultiFF);
				threshold = (uFIBCnt*FIB_SIZE);
#else
				continue;
#endif
			}
			else
			{
				nDataSize = INC_QFIFO_GET_SIZE(pMultiFF);
				if(tunerbb_drv_lg2102_get_datathreshold(pMultiFF->unSubChID, &threshold)==INC_ERROR)
				{
					return INC_ERROR;
				}
			}
			nDataSize = (nDataSize/threshold)*threshold;
			if(nDataSize>=threshold)
			{				
				buf_size += sizeof(TDMB_BB_HEADER_TYPE) + nDataSize;
			}			
		}
	}
	else
	{
		printk("[T39fx] INC_MULTI_FIFO_PROCESS() Error [input_buf:0x%x, input_size:%d]",(uint32)input_buf,input_size);
		return INC_ERROR;
	}

	*read_size = buf_size;

	return INC_SUCCESS;
}

int8 tunerbb_drv_lg2102_get_multi_data(uint8 subch_cnt, uint8* buf_ptr, uint32 buf_size)
{
	ST_FIFO*	pMultiFF;
	uint32	nDataSize;
	int			i;
	TDMB_BB_HEADER_TYPE dmb_header;
	//INC_UINT16	uFIBCnt;
	//ST_FIC_DB* pstFicDb;   /* NOT USED */
	//INC_CHANNEL_INFO* pChInfo;   /* NOT USED */
	uint32	read_size = 0;
	uint8 datatype;
	uint32 threshold;

	if(buf_ptr == NULL || buf_size == 0)
	{
		return INC_ERROR;
	}
	
	if(subch_cnt>INC_MULTI_MAX_CHANNEL) 
	{
		return INC_ERROR;
	}
	
	for(i=1;i<=subch_cnt+1;i++)
	{
		/************************************************
		(i==1) => FIC_STREAM_DATA,
		(i==2) => CHANNEL1_STREAM_DATA,
		(i==3) => CHANNEL2_STREAM_DATA,
		(i==4) => CHANNEL3_STREAM_DATA,
		************************************************/
		pMultiFF = INC_GET_CHANNEL_FIFO(i);
		if(i==1)
		{
#ifdef INC_MULTI_CHANNEL_FIC_UPLOAD
			uFIBCnt = INC_GET_FIB_CNT(m_ucTransMode);
			nDataSize = INC_QFIFO_GET_SIZE(pMultiFF);
			threshold = (uFIBCnt*FIB_SIZE);
			datatype = TDMB_BB_DATA_FIC;
#else
			continue;
#endif
		}
		else
		{
			nDataSize = INC_QFIFO_GET_SIZE(pMultiFF);
			if(tunerbb_drv_lg2102_get_datatype(pMultiFF->unSubChID, &datatype)==INC_ERROR)
			{
				return INC_ERROR;
			}
			if(tunerbb_drv_lg2102_get_datathreshold(pMultiFF->unSubChID, &threshold)==INC_ERROR)
			{
				return INC_ERROR;
			}
		}
		nDataSize = (nDataSize/threshold)*threshold;
		if(nDataSize>=threshold)
		{
			dmb_header.reserved = 0xDEAD;
			dmb_header.data_type = datatype;
			dmb_header.size = nDataSize;
			dmb_header.subch_id = pMultiFF->unSubChID;
			memcpy(buf_ptr,&dmb_header,sizeof(TDMB_BB_HEADER_TYPE));
			buf_ptr += sizeof(TDMB_BB_HEADER_TYPE);
			read_size += sizeof(TDMB_BB_HEADER_TYPE);
			INC_QFIFO_BRING(pMultiFF, buf_ptr, nDataSize);
			buf_ptr += nDataSize;
			read_size += nDataSize;
		}			
		else
		{
			printk("[T39fx] skip INC_QFIFO_BRING()  [i:%d, nDataSize:%d, threshold:%d]",read_size,nDataSize,threshold);
		}
	}

	if(read_size != buf_size)
	{
		printk("[T39fx] tunerbb_drv_lg2102_get_multi_data() Error [read_size:%d, buf_size:%d]",read_size,buf_size);
		return INC_ERROR;		
	}
	else
	{
		return INC_SUCCESS;
	}
}

//#endif 

void tunerbb_drv_lg2102_start_tii(void)
{
	INC_TII_START(TDMB_RFBB_DEV_ADDR);
	//INC_MSG_PRINT(1,"=== INC_TII_START(%d) \n");
}

void tunerbb_drv_lg2102_stop_tii(void)
{
	INC_TII_STOP(TDMB_RFBB_DEV_ADDR);
	//INC_MSG_PRINT(1,"=== INC_TII_STOP() \n");
}

int8 tunerbb_drv_lg2102_check_tii(uint8* pmain_tii, uint8* psub_tii)
{
	ST_TII_INFO stTIIInfo[MAX_TII_CNT] = {{0,},};
	INC_UINT16 uiStatus = 0;
	
	uiStatus = INC_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_PHY_BASE+ 0x2D);

	if(uiStatus == 0x1F7F) return INC_ERROR;
	
	INC_TII_GET_INFO(TDMB_RFBB_DEV_ADDR,stTIIInfo);

	printk(" ==> TII0 [MainID(0x%X), SubID(0x%X), Strength(%d)] \n", stTIIInfo[0].uiPattern, stTIIInfo[0].uiSubID, stTIIInfo[0].uiStrength);

	*pmain_tii = (uint8)stTIIInfo[0].uiPattern;
	*psub_tii = (uint8)stTIIInfo[0].uiSubID;
	
	return INC_SUCCESS;
}

#if 0
int8 tunerbb_drv_lg2102_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	//INC_UINT8 nRet;
	uint8 nRet;
	//ST_BBPINFO* pInfo;
	
	nRet = INTERFACE_STATUS_CHECK(TDMB_RFBB_DEV_ADDR);

	if(nRet == INC_SUCCESS)
	{
		dmb_bb_info->sync_lock	= 1;
		dmb_bb_info->cir		= 1;
		dmb_bb_info->dab_ok 	= 1;		
		dmb_bb_info->sch_ber	= 1;
	}
	else
	{
		dmb_bb_info->sync_lock	= 0;
		dmb_bb_info->dab_ok 	= 0;
		dmb_bb_info->sch_ber	= 0;
	}
	
	dmb_bb_info->afc_ok 	= 1;

	pInfo = INC_GET_STRINFO(TDMB_RFBB_DEV_ADDR);
	dmb_bb_info->msc_ber = pInfo->uiCER*10;
	return INC_SUCCESS;
}
#endif

#if 0
int8 tunerbb_drv_lg2102_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[ ], uint8 op_mode[ ])
{
	int major_ch, minor_ch, fnindex;
	int i;
	INC_INT16 nLoop;
	INC_CHANNEL_INFO ChInfo[INC_MULTI_MAX_CHANNEL];
	INC_ERROR_INFO ErrorInfo;
	ST_SUBCH_INFO stSubInfo;
	
	if(subch_cnt>INC_MULTI_MAX_CHANNEL) 
	{
		return INC_ERROR;
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
		if(INTERFACE_SCAN(TDMB_RFBB_DEV_ADDR, g_uiKOREnsembleFullFreq[fnindex])==INC_SUCCESS)
		{
			return INC_SUCCESS;
		}
	}
	else
	{
		for(i=0;i<subch_cnt;i++)
		{
			switch(op_mode[i])
			{
				case LG2102_DAB:					
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					_ChInfo->ucServiceType = 0x00;
					_ChInfo->uiTmID = TMID_0;
					//_ChInfo->ucDataType = TDMB_BB_DATA_DAB;
					//_ChInfo->ulDataThreshold = 188*10;
					break;
				case LG2102_DATA:
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					_ChInfo->ucServiceType = 0x00;
					_ChInfo->uiTmID = TMID_2;	
					//_ChInfo->ucDataType = TDMB_BB_DATA_PACK;
					//_ChInfo->ulDataThreshold = 288;//188;
					break;
				case LG2102_DMB:
				case LG2102_VISUAL:
					_ChInfo->ucSubChID = subch_id[i];
					_ChInfo->ulRFFreq = g_uiKOREnsembleFullFreq[fnindex];
					_ChInfo->ucServiceType = 0x18;
					_ChInfo->uiTmID = TMID_1;
					//_ChInfo->ucDataType = TDMB_BB_DATA_TS;
					//_ChInfo->ulDataThreshold = 188*32;
					break;
				default:
					return INC_ERROR;
			}
			memcpy(&stSubInfo.astSubChInfo[i], &ChInfo[i], sizeof(INC_CHANNEL_INFO));
		}

		stSubInfo.nSetCnt = subch_cnt;
		INC_MULTI_SORT_INIT();

		memcpy(&g_stSubInfo, &stSubInfo, sizeof(ST_SUBCH_INFO));
		
		for(nLoop = 0; nLoop < 2; nLoop++)
		{
			if(INTERFACE_START(TDMB_RFBB_DEV_ADDR, &stSubInfo))
			{	
				return INC_SUCCESS;
			}
			ErrorInfo = INTERFACE_ERROR_STATUS(TDMB_RFBB_DEV_ADDR);
			printk("[INC]^__^ INTERFACE_ERROR_STATUS = (0x%04x)", ErrorInfo);
			if(ErrorInfo == ERROR_SYNC_NULL || ErrorInfo == ERROR_FICDECODER || 
				ErrorInfo == ERROR_SYNC_TIMEOUT) continue; // 약전계시 한번더 호출함. 
			else
			{
				return INC_ERROR;
			}
		}
	}

	return INC_ERROR;
}


int8 tunerbb_drv_lg2102_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
	return tunerbb_drv_lg2102_multi_set_channel(freq_num, 1, &subch_id , &op_mode);
}

int8 tunerbb_drv_lg2102_re_syncdetector(uint8 op_mode)
{
	int8 ret_val = INC_ERROR;

	if(op_mode != LG2102_ENSQUERY)
	{
		ret_val = (int8)INTERFACE_RE_SYNCDETECTOR(TDMB_RFBB_DEV_ADDR, &g_stSubInfo);
	}

	return ret_val;
}

int8 tunerbb_drv_lg2102_re_sync(void)
{
	return INC_ERROR;
	//return (int8)INTERFACE_RE_SYNC(TDMB_RFBB_DEV_ADDR);
}


int8 tunerbb_drv_lg2102_get_fic(uint8* buffer, uint32* buffer_size /*, boolean cr_onoff */)
{
	INC_UINT32 uiFicSize;

	if(buffer == NULL || buffer_size == NULL)
	{
		return INC_ERROR;
	}

	if(!(INC_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_VTB_BASE+ 0x00) & 0x4000))
	{
		*buffer_size = 0;
		return INC_ERROR;
	}

	uiFicSize = (INC_UINT32)INC_CMD_READ(TDMB_RFBB_DEV_ADDR, APB_VTB_BASE+ 0x09) +1;
	if(uiFicSize == 1)
	{
		*buffer_size = 0;
		return INC_ERROR;
	}

	if(uiFicSize > 384)
	{
		printk("tunerbb_drv_lg2102_get_fic uiFicSize = (%d) is over 384\n", uiFicSize);
		uiFicSize = 384;
	}

	if(INC_CMD_READ_BURST(TDMB_RFBB_DEV_ADDR, APB_FIC_BASE, buffer, uiFicSize) == INC_SUCCESS)
	{
		*buffer_size = uiFicSize;
		printk("tunerbb_drv_lg2102_get_fic = %x %x %x %x %x\n", *buffer, *(buffer+1), *(buffer+2), *(buffer+3), *(buffer+4));
		return INC_SUCCESS;
	}

	return INC_ERROR;
}

int8 tunerbb_drv_lg2102_read_data(uint8* buffer, uint32* buffer_size)
{
	/* SPI, EBI needed */
	*buffer_size = 0;
	return INC_ERROR;
}
#endif




#if 0
void tunerbb_drv_lg2102_start_tii(void)
{
	return;
}

void tunerbb_drv_lg2102_stop_tii(void)
{
	return;
}

boolean tunerbb_drv_lg2102_check_tii(uint8* pmain_tii, uint8* psub_tii)
{
	return;
}
#endif
