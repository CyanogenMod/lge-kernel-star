
//#include "StdAfx.h"
#include "../inc/INC_INCLUDES.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_t3900.h"

#ifdef INC_WINDOWS_SPACE // Windows XP
#include "USB_Driver.h"
USB_Driver	g_pUsbDriver;
CRITICAL_SECTION	gCS_INC;
FILE* m_hINC_LogFile = NULL;
#endif

#define INC_INTERRUPT_LOCK()		
#define INC_INTERRUPT_FREE()

#define CMD_SIZE	4	/* SPI CMD_SIZE */

ST_SUBCH_INFO		g_stDmbInfo;
ST_SUBCH_INFO		g_stDabInfo;
ST_SUBCH_INFO		g_stDataInfo;
ST_SUBCH_INFO		g_stFIDCInfo;
ST_SUBCH_INFO		g_stChannel;

/*********************************************************************************/
/*	RF Band Select																 */
/*																				 */
/*	INC_UINT8 m_ucRfBand = KOREA_BAND_ENABLE,									 */
/*				 BANDIII_ENABLE,												 */
/*				 LBAND_ENABLE,													 */
/*				 CHINA_ENABLE,													 */
/*				 EXTERNAL_ENABLE,												 */
/*********************************************************************************/
ENSEMBLE_BAND 		m_ucRfBand 		= KOREA_BAND_ENABLE;

/*********************************************************************************/
/*	MPI Chip Select and Clock Setup Part										 */
/*	                                                                             */
/*	m_ucCommandMode = INC_I2C_CTRL, INC_SPI_CTRL, INC_EBI_CTRL                   */
/*	m_ucUploadMode  = STREAM_UPLOAD_MASTER_SERIAL,                               */
/*				      STREAM_UPLOAD_SLAVE_PARALLEL,                              */
/*				      STREAM_UPLOAD_TS,                                          */
/*				      STREAM_UPLOAD_SPI,                                         */
/*																				 */
/*	m_ucClockSpeed = INC_OUTPUT_CLOCK_4096,                                      */
/*				     INC_OUTPUT_CLOCK_2048,                                      */
/*				     INC_OUTPUT_CLOCK_1024,                                      */
/*********************************************************************************/

//CTRL_MODE 			m_ucCommandMode 	= INC_I2C_CTRL;
CTRL_MODE 			m_ucCommandMode 	= INC_SPI_CTRL;
ST_TRANSMISSION		m_ucTransMode		= TRANSMISSION_MODE1;
//UPLOAD_MODE_INFO	m_ucUploadMode 		= STREAM_UPLOAD_TS;
UPLOAD_MODE_INFO	m_ucUploadMode 		= STREAM_UPLOAD_SPI;
CLOCK_SPEED			m_ucClockSpeed 		= INC_OUTPUT_CLOCK_4096;
INC_ACTIVE_MODE		m_ucMPI_CS_Active 	= INC_ACTIVE_HIGH;
INC_ACTIVE_MODE		m_ucMPI_CLK_Active 	= INC_ACTIVE_LOW;
INC_UINT16			m_unIntCtrl				= (INC_INTERRUPT_ACTIVE_POLALITY_LOW | \
											   INC_INTERRUPT_PULSE | \
											   INC_INTERRUPT_AUTOCLEAR_ENABLE| \
											   (INC_INTERRUPT_PULSE_COUNT & INC_INTERRUPT_PULSE_COUNT_MASK));

static INC_UINT8 m_auiRxBuff[4096+4] = {0};

/*********************************************************************************/
/* PLL_MODE			m_ucPLL_Mode                                                 */
/*T3900  Input Clock Setting                                                     */
/*********************************************************************************/
PLL_MODE			m_ucPLL_Mode		= INPUT_CLOCK_24576KHZ;


/*********************************************************************************/
/* INC_DPD_MODE		m_ucDPD_Mode                                                 */
/* T3900  Power Saving mode setting                                              */
/*********************************************************************************/
INC_DPD_MODE		m_ucDPD_Mode		= INC_DPD_OFF;



void              INC_MUST_DELAY(INC_UINT16 uiDelay)
{
	tdmb_t3900_must_mdelay(uiDelay);


}

INC_UINT8 INC_DELAY(INC_UINT8 ucI2CID ,INC_UINT16 uiDelay)
{          
	 //TODO Delay code here...
	if(tdmb_t3900_mdelay(uiDelay) == INC_ERROR)
	{
		INTERFACE_USER_STOP(ucI2CID);
		return INC_ERROR;
	}
	return INC_SUCCESS;
}

void INC_MSG_PRINTF(INC_INT8 nFlag, INC_INT8* pFormat, ...)
{
	va_list Ap;
	INC_UINT16 nSize;
	INC_INT8 acTmpBuff[1000] = {0};

	va_start(Ap, pFormat);
	nSize = vsprintf(acTmpBuff, pFormat, Ap);
	va_end(Ap);

#ifdef INC_KERNEL_SPACE
	if(nFlag)
		printk("%s", acTmpBuff);

#elif defined  (INC_WINDOWS_SPACE) // Windows XP
	if(nFlag)
		TRACE("%s", acTmpBuff);

	if(m_hINC_LogFile && nFlag){
		CString s;
		CTime	t;
		t = CTime::GetCurrentTime();

		s.Format("[%s] %s", t.Format("%d %H:%M:%S"), acTmpBuff);
		fwrite(s, sizeof(char), strlen(s), m_hINC_LogFile);
		fflush( (FILE*) m_hINC_LogFile );
	}
#else //WinCE
	///////////////////////////////////////////////////
	// .NET 버전일 경우 wchar_t로 변환
	wchar_t wcstring[1024] = {0};
	mbstowcs(wcstring, logstr, strlen(logstr)+1);
	RETAILMSG(nFlag, (TEXT("%s"), wcstring));
	
	if(m_hINC_LogFile && nFlag){
		SYSTEMTIME time;
		GetLocalTime(&time);

		sprintf(logstr, "[%02d.%02d %02d:%02d:%02d] %s",
			time.wMonth, time.wDay, time.wHour, time.wMinute, time.wSecond, acTmpBuff);
		fwrite(logstr, sizeof(char), strlen(logstr)+1, m_hINC_LogFile);
		fflush( (FILE*) m_hINC_LogFile ); 
	}
#endif
}



INC_UINT16 INC_I2C_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{
#if 0
#ifdef INC_I2C_GPIO_CTRL_ENABLE	
	INC_UINT8 acBuff[2];
	INC_UINT16 wData;
	INC_I2C_ACK AckStatus;
	
	AckStatus = INC_GPIO_CTRL_READ(ucI2CID, uiAddr, acBuff, 2);
	if(AckStatus == I2C_ACK_SUCCESS){
		wData = ((INC_UINT16)acBuff[0] << 8) | (INC_UINT16)acBuff[1];
		return wData;
	}
	return INC_ERROR;
#else
	//TODO I2C Read code here...
	INC_UINT8 acBuff[2];
	INC_UINT16 wData;
	INC_INT32 AckStatus;

	AckStatus= tdmb_t3900_i2c_read_burst(uiAddr, acBuff,2);	//for android compile
	if(AckStatus == TRUE){
		wData = ((INC_UINT16)acBuff[0] << 8) | (INC_UINT16)acBuff[1];
		return wData;
	}
	return INC_ERROR;	
#endif
#endif
    return 0;	
}

INC_UINT8 INC_I2C_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
#if 0
#ifdef INC_I2C_GPIO_CTRL_ENABLE	
	INC_UINT8 acBuff[2];
	INC_UINT8 ucCnt = 0;
	INC_I2C_ACK AckStatus;

	acBuff[ucCnt++] = (uiData >> 8) & 0xff;
	acBuff[ucCnt++] = uiData & 0xff;
	
	AckStatus = INC_GPIO_CTRL_WRITE(ucI2CID, uiAddr, acBuff, ucCnt);
	if(AckStatus == INC_SUCCESS)
	{
		return INC_SUCCESS;
	}
	return INC_ERROR;
#else
	//TODO I2C write code here...
	INC_UINT8 acBuff[2];
	INC_UINT8 ucCnt = 0;
	INC_INT32 AckStatus;

	acBuff[ucCnt++] = (uiData >> 8) & 0xff;
	acBuff[ucCnt++] = uiData & 0xff;

	AckStatus = tdmb_t3900_i2c_write_burst(uiAddr, acBuff, ucCnt);	//for android compile
	if(AckStatus == INC_SUCCESS)
	{
		return INC_SUCCESS;
	}
	return INC_ERROR;
#endif
#endif
    return 0;	
}

INC_UINT8 INC_I2C_READ_BURST(INC_UINT8 ucI2CID,  INC_UINT16 uiAddr, INC_UINT8* pData, INC_UINT16 nSize)
{
#if 0
#ifdef INC_I2C_GPIO_CTRL_ENABLE	
	INC_I2C_ACK AckStatus;
	AckStatus = INC_GPIO_CTRL_READ(ucI2CID, uiAddr, pData, nSize);

	if(AckStatus == I2C_ACK_SUCCESS)
		return INC_SUCCESS;
	return INC_ERROR;
#else
	//TODO I2C Read code here...
	INC_INT32 AckStatus;

	AckStatus = tdmb_t3900_i2c_read_burst(uiAddr, pData, nSize);
	if(AckStatus == INC_SUCCESS)
	{
		return INC_SUCCESS;
	}
	return INC_ERROR;
#endif
#endif
	return 0;	
}

INC_UINT8 INC_EBI_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
#if 0
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	INC_INTERRUPT_LOCK();

	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;

	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = (uiData >> 8) & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS =  uiData & 0xff;

	INC_INTERRUPT_FREE();
#endif
	return INC_SUCCESS;
}

INC_UINT16 INC_EBI_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{
#if 0
	INC_UINT16 uiRcvData = 0;
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	INC_INTERRUPT_LOCK();
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;

	uiRcvData  = (*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS  & 0xff) << 8;
	uiRcvData |= (*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff);
	
	INC_INTERRUPT_FREE();
	return uiRcvData;
#endif
	return 0;	
}

INC_UINT8 INC_EBI_READ_BURST(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT8* pData, INC_UINT16 nSize)
{
#if 0
	INC_UINT16 uiLoop, nIndex = 0, anLength[2], uiCMD, unDataCnt;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	if(nSize > INC_MPI_MAX_BUFF) return INC_ERROR;
	memset((INC_INT8*)anLength, 0, sizeof(anLength));

	if(nSize > INC_TDMB_LENGTH_MASK) {
		anLength[nIndex++] = INC_TDMB_LENGTH_MASK;
		anLength[nIndex++] = nSize - INC_TDMB_LENGTH_MASK;
	}
	else anLength[nIndex++] = nSize;

	INC_INTERRUPT_LOCK();
	for(uiLoop = 0; uiLoop < nIndex; uiLoop++){

		uiCMD = INC_REGISTER_CTRL(SPI_MEMREAD_CMD) | (anLength[uiLoop] & INC_TDMB_LENGTH_MASK);

		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;

		for(unDataCnt = 0 ; unDataCnt < anLength[uiLoop]; unDataCnt++){
			*pData++ = *(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff;
		}
	}
	INC_INTERRUPT_FREE();
#endif
	return INC_SUCCESS;
}


INC_UINT16 INC_SPI_REG_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{

	INC_UINT16 uiRcvData = 0;

#ifdef INC_WINDOWS_SPACE
	g_pUsbDriver.REG_Read(uiAddr, (INC_UINT8*)&uiRcvData);
#endif
	INC_UINT16	uiCMD = INC_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
	INC_UINT8	auiTxBuff[CMD_SIZE+2] = {0};
	INC_UINT8	auiRxBuff[CMD_SIZE+2] = {0};

	auiTxBuff[0] = uiAddr>>8;
	auiTxBuff[1] = uiAddr&0xff;
	auiTxBuff[2] = uiCMD>>8;
	auiTxBuff[3] = uiCMD&0xff;

	INC_INTERRUPT_LOCK();
	tdmb_t3900_spi_write_read(auiTxBuff, 6, auiRxBuff, 0);
	INC_INTERRUPT_FREE();

	uiRcvData = auiRxBuff[4]<<8 | auiRxBuff[5];
	return uiRcvData;
}

INC_UINT8 INC_SPI_REG_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
#if 1
#ifdef  INC_WINDOWS_SPACE 
	g_pUsbDriver.REG_Write(uiAddr, uiData);
#endif
	//printk("INC_SPI_REG_WRITE Start\n");
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
	INC_UINT8 auiTxBuff[CMD_SIZE+2] = {0};
	//Added by wonhee.jeong 110406 for B_Qwerty TDMB Porting
	INC_UINT8 auiRxBuff[CMD_SIZE+2] = {0};

	auiTxBuff[0] = uiAddr>>8 ;
	auiTxBuff[1] = uiAddr&0xff;
	auiTxBuff[2] = uiCMD>>8;
	auiTxBuff[3] = uiCMD&0xff;
	auiTxBuff[4] = uiData>>8;
	auiTxBuff[5] = uiData&0xff;

	INC_INTERRUPT_LOCK();
	//printk("INC_SPI_REG_WRITE running1\n");
	tdmb_t3900_spi_write_read(auiTxBuff, 6, auiRxBuff, 0);
	//printk("INC_SPI_REG_WRITE running2\n");
	INC_INTERRUPT_FREE();

	//printk("INC_SPI_REG_WRITE End\n");
	
	return INC_SUCCESS;

#endif
}

INC_UINT8 INC_SPI_READ_BURST(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT8* pBuff, INC_UINT16 wSize)
{
	INC_UINT16 uiLoop, nIndex = 0, anLength[2], uiCMD;
	INC_UINT8 auiBuff[6];
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	//printk("INC_SPI_READ_BURST Start\n");
	if( (pBuff==NULL) || (wSize > INC_MPI_MAX_BUFF)) return INC_ERROR;
	memset((INC_INT8*)anLength, 0, sizeof(anLength));

	if(wSize > INC_TDMB_LENGTH_MASK) {
		anLength[nIndex++] = INC_TDMB_LENGTH_MASK;
		anLength[nIndex++] = wSize - INC_TDMB_LENGTH_MASK;
	}
	else anLength[nIndex++] = wSize;

	INC_INTERRUPT_LOCK();
	for(uiLoop = 0; uiLoop < nIndex; uiLoop++){

		auiBuff[0] = uiNewAddr >> 8;
		auiBuff[1] = uiNewAddr & 0xff;
		uiCMD = INC_REGISTER_CTRL(SPI_MEMREAD_CMD) | (anLength[uiLoop] & INC_TDMB_LENGTH_MASK);
		auiBuff[2] = uiCMD >> 8;
		auiBuff[3] = uiCMD & 0xff;
		//printk("INC_SPI_READ_BURST running\n");
		tdmb_t3900_spi_write_read(auiBuff, 4, m_auiRxBuff, anLength[uiLoop]);

		// memcpy((INC_UINT8*)&pBuff[uiLoop*anLength[uiLoop]], (INC_UINT8*)&m_auiRxBuff[4], anLength[uiLoop]);
		memcpy((INC_UINT8*)pBuff, (INC_UINT8*)&m_auiRxBuff[4], anLength[uiLoop]);
		pBuff += anLength[uiLoop];
	}
	INC_INTERRUPT_FREE();

	//printk("INC_SPI_READ_BURST end\n");

	return INC_SUCCESS;
}


INC_UINT8 INTERFACE_DBINIT(void)
{
	memset(&g_stDmbInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDabInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDataInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stFIDCInfo,	0, sizeof(ST_SUBCH_INFO));
	return INC_SUCCESS;
}

void INTERFACE_UPLOAD_MODE(INC_UINT8 ucI2CID, UPLOAD_MODE_INFO ucUploadMode)
{
	m_ucUploadMode = ucUploadMode;
	INC_UPLOAD_MODE(ucI2CID);
}

INC_UINT8 INTERFACE_PLL_MODE(INC_UINT8 ucI2CID, PLL_MODE ucPllMode)
{
	m_ucPLL_Mode = ucPllMode;
	return INC_PLL_SET(ucI2CID);
}

// 초기 전원 입력시 호출
INC_UINT8 INTERFACE_INIT(INC_UINT8 ucI2CID)
{
	return INC_INIT(ucI2CID);
}

INC_UINT8 INTERFACE_RESET(INC_UINT8 ucI2CID)
{
	INC_RESET_MPI(ucI2CID);
	return INC_SUCCESS;
	
}

// 에러 발생시 에러코드 읽기
INC_ERROR_INFO INTERFACE_ERROR_STATUS(INC_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = INC_GET_STRINFO(ucI2CID);
	return pInfo->nBbpStatus;
}

/*********************************************************************************/
/* 단일 채널 선택하여 시작하기....                                               */  
/* pChInfo->uiTmID, pChInfo->ucSubChID, pChInfo->ulRFFreq 는                     */
/* 반드시 넘겨주어야 한다.                                                       */
/* DMB채널 선택시 pChInfo->uiTmID = TMID_1                                       */
/* DAB채널 선택시 pChInfo->uiTmID = TMID_0 으로 설정을 해야함.                   */
/*********************************************************************************/
INC_UINT8 INTERFACE_START(INC_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
	return INC_CHANNEL_START(ucI2CID, pChInfo);
}


INC_INT8 INTERFACE_RE_SYNCDETECTOR(INC_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
		return INC_CHANNEL_START(ucI2CID,pChInfo);
}


/*********************************************************************************/
/* 스캔시  호출한다.                                                             */
/* 주파수 값은 받드시 넘겨주어야 한다.                                           */
/*********************************************************************************/
#if 0
INC_UINT8 INTERFACE_SCAN(INC_UINT8 ucI2CID, INC_UINT32 ulFreq)
{
	INC_INT16 nIndex;
	INC_CHANNEL_INFO* pChInfo;

	//INTERFACE_DBINIT();
	memset(&g_stChannel, 0, sizeof(ST_SUBCH_INFO));

	if(!INC_ENSEMBLE_SCAN(ucI2CID, ulFreq)) return INC_ERROR;

	INC_DB_UPDATE(ulFreq, &g_stChannel);
	INC_BUBBLE_SORT(&g_stChannel,  INC_SUB_CHANNEL_ID);

	for(nIndex = 0; nIndex < g_stChannel.nSetCnt; nIndex++)
	{
		switch(g_stChannel.astSubChInfo[nIndex].uiTmID)
		{
		case TMID_1 : pChInfo = &g_stDmbInfo.astSubChInfo[g_stDmbInfo.nSetCnt++]; break;
		case TMID_0 : pChInfo = &g_stDabInfo.astSubChInfo[g_stDabInfo.nSetCnt++]; break;
		default   : pChInfo = &g_stDataInfo.astSubChInfo[g_stDataInfo.nSetCnt++]; break;
		}
		memcpy(pChInfo, &g_stChannel.astSubChInfo[nIndex], sizeof(INC_CHANNEL_INFO));	
	}

/*	for(nIndex=0; nIndex<g_stDmbInfo.nSetCnt; nIndex++)
	{
		INC_MSG_PRINTF(INC_DEBUG_LEVEL, " ==============================================\r\n");
		pChInfo = &g_stDmbInfo.astSubChInfo[nIndex];
		INC_MSG_PRINTF(1, " EL[%s] Sub channel label[%s]\r\n", pChInfo->aucEnsembleLabel, pChInfo->aucLabel);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel Service   ID[0x%.8X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ulServiceID);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel Service Type[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ucServiceType);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel           ID[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ucSubChID);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel         TmID[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->uiTmID);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel     Bit Rate[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->uiBitRate);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel   PacketAddr[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->uiPacketAddr);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel     ucSlFlag[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ucSlFlag);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel   ucProtectionLevel[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ucProtectionLevel);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel   uiDifferentRate[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->uiDifferentRate);
		INC_MSG_PRINTF(1, " EL[%s] Sub Channel   ucOption[0x%.4X]\r\n", pChInfo->aucEnsembleLabel, pChInfo->ucOption);
	}
*/	
	return INC_SUCCESS;
}

#endif

INC_UINT8 INTERFACE_SCAN(INC_UINT8 ucI2CID, INC_UINT32 ulFreq)
{
	
	if(!INC_ENSEMBLE_SCAN(ucI2CID, ulFreq)) return INC_ERROR;
	 return INC_SUCCESS;
}

// [LG_Unibar], 20110128, ASJ,   -Begin-
INC_UINT8 INTERFACE_GET_FIC(INC_UINT8 ucI2CID,INC_UINT32* nFic_Size)
{
		
	if((INC_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x00) & 0x4000) == INC_ERROR)	{
		return INC_ERROR;
	}
	
	*nFic_Size = (INC_UINT32)INC_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x09) + 1;
	if(*nFic_Size == 1 /*|| uiFicSize != 384*/)	{
		*nFic_Size = 0;
		return INC_ERROR;		
	}
	
	return INC_SUCCESS;
		
}

// [LG_Unibar], 20110128, ASJ,   -End-

/*********************************************************************************/
/* 스캔이 완료되면 앙상블 label을 리턴한다.                                      */
/*********************************************************************************/
INC_INT8* INTERFACE_GET_ENSEMBLE_LABEL(void)
{
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();
	return (INC_INT8*)pList->aucEnsembleName;
}

/*********************************************************************************/
/* 스캔이 완료되면 앙상블 ID를 리턴한다.                                         */
/*********************************************************************************/
INC_UINT16 INTERFACE_GET_ENSEMBLE_ID(void)
{
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();
	return pList->unEnsembleID;
}

/*********************************************************************************/
/* 스캔이 완료되면 전채 서브채널 개수를 리턴한다.                                */
/*********************************************************************************/
INC_UINT16 INTERFACE_GET_SUBCHANNEL_CNT(void)
{
	return INTERFACE_GETDMB_CNT() + INTERFACE_GETDAB_CNT() + INTERFACE_GETDATA_CNT();
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DMB채널 개수를 리턴한다.                             */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDMB_CNT(void)
{
	return (INC_UINT16)g_stDmbInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DAB채널 개수를 리턴한다.                             */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDAB_CNT(void)
{
	return (INC_UINT16)g_stDabInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DATA채널 개수를 리턴한다.                            */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDATA_CNT(void)
{
	return (INC_UINT16)g_stDataInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 Ensemble label을 리턴한다.                           */
/*********************************************************************************/
INC_UINT8* INTERFACE_GETENSEMBLE_LABEL(INC_UINT8 ucI2CID)
{
	ST_FICDB_LIST*	pList;
	pList = INC_GET_FICDB_LIST();
	return pList->aucEnsembleName;
}

/*********************************************************************************/
/* 단일채널 검색이 완료되면, 검색된 모든 정보를 리턴한다.						 */
/*********************************************************************************/
ST_SUBCH_INFO* INTERFACE_GETDB_CHANNEL(void)
{
	return &g_stChannel;
}

/*********************************************************************************/
/* DMB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DMB(INC_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
	if(uiPos >= g_stDmbInfo.nSetCnt) return INC_NULL;
	return &g_stDmbInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DAB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DAB(INC_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
	if(uiPos >= g_stDabInfo.nSetCnt) return INC_NULL;
	return &g_stDabInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DATA 채널 정보를 리턴한다.                                                    */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DATA(INC_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
	if(uiPos >= g_stDataInfo.nSetCnt) return INC_NULL;
	return &g_stDataInfo.astSubChInfo[uiPos];
}

// 시청 중 FIC 정보 변경되었는지를 체크
INC_UINT8 INTERFACE_RECONFIG(INC_UINT8 ucI2CID)
{
	return INC_FIC_RECONFIGURATION_HW_CHECK(ucI2CID);
}

// Check the strength of Signal
INC_UINT8 INTERFACE_STATUS_CHECK(INC_UINT8 ucI2CID)
{
	return INC_STATUS_CHECK(ucI2CID);
}

INC_INT16 INTERFACE_GET_CER(INC_UINT8 ucI2CID)
{
	return INC_GET_CER(ucI2CID);
}

INC_UINT8 INTERFACE_GET_SNR(INC_UINT8 ucI2CID)
{
	return INC_GET_SNR(ucI2CID);
}

INC_INT32 INTERFACE_GET_POSTBER(INC_UINT8 ucI2CID)
{
	return INC_GET_POSTBER(ucI2CID);
}

INC_INT32 INTERFACE_GET_PREBER(INC_UINT8 ucI2CID)
{
	return INC_GET_PREBER(ucI2CID);
}

INC_INT8 INTERFACE_RE_SYNC(INC_UINT8 ucI2CID)
{
	return INC_RE_SYNC( ucI2CID);
}

INC_UINT16 INTERFACE_GET_TPERRCNT(INC_UINT8 ucI2CID)
{
	return INC_GET_TPERRCNT(ucI2CID);
}

/*********************************************************************************/
/* Scan, 채널 시작시에 강제로 중지시 호출한다.                                      */
/*********************************************************************************/
void INTERFACE_USER_STOP(INC_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = INC_GET_STRINFO(ucI2CID);
	pInfo->nBbpStatus = ERROR_USER_STOP;
	pInfo->ucStop = 1;
	
}

// Interrupt Start...
void INTERFACE_INT_ENABLE(INC_UINT8 ucI2CID, INC_UINT16 unSet)
{
	INC_INTERRUPT_ENABLE(ucI2CID, unSet);
}

// Use when polling mode
INC_UINT8 INTERFACE_INT_CHECK(INC_UINT8 ucI2CID)
{
	INC_UINT16 nValue = 0;

	nValue = INC_CMD_READ(ucI2CID, APB_INT_BASE+ 0x01);
	if(!(nValue & INC_MPI_INTERRUPT_ENABLE))
		return INC_ERROR;

	return INC_SUCCESS;
}

// Interrupt clear.
void INTERFACE_INT_CLEAR(INC_UINT8 ucI2CID, INC_UINT16 unClr)
{
	INC_INTERRUPT_CLEAR(ucI2CID, unClr);
}

// 인터럽트 서비스 루틴... // SPI Slave Mode or MPI Slave Mode
// It's sample function..
INC_UINT16 INTERFACE_ISR(INC_UINT8 ucI2CID, INC_UINT8* pBuff)
{
	INC_UINT16 unData;
	unData = INC_CMD_READ(ucI2CID, APB_MPI_BASE + 0x08);
	/* modifying read data in remained in T3900 FIFO BUFFER aligned 188 */
	if( unData & 0x4000 ){
		//bFirstLoop = 1;
		INC_MSG_PRINTF(1, "[%s]==> FIFO FULL   : 0x%X \r\n", __FILE__, unData );
	}
	else if( !(unData & 0x3FFF ))	{
		unData = 0;
		INTERFACE_INT_CLEAR(TDMB_I2C_ID80, INC_MPI_INTERRUPT_ENABLE);
		INC_MSG_PRINTF(1, "[%s]==> FIFO Empty	: 0x%X \r\n", __FILE__, unData );
	}
	else{
		unData &= 0x3FFF;
	}

	if(unData < INC_INTERRUPT_SIZE) {
		printk("unData is smaller than INC_INTERUUPT_SIZE (%d)\n", unData);
		return INC_ERROR;
	}

	unData = ((INC_UINT16)(unData/188))*188;  /* 188byte align */
	/* pBuffer size is 8120. Refer to  TDMB_MPI_BUF_SIZE defined in broadcast_t3900_drv_if.c */
	if(unData > 7520) {	/* 188x8x5 pkts */
		printk("INC_ISR unData size (%d), limit (MPI_CS_SIZE*5)\n", unData);
		unData = 7520;
	}
	memset(pBuff, 0xab, unData);

	INC_CMD_READ_BURST(ucI2CID, APB_STREAM_BASE, pBuff, unData);

	if((m_unIntCtrl & INC_INTERRUPT_LEVEL) && (!(m_unIntCtrl & INC_INTERRUPT_AUTOCLEAR_ENABLE)))
		INTERFACE_INT_CLEAR(ucI2CID, INC_MPI_INTERRUPT_ENABLE);

	return unData;
#if 0  /* INC Original code read INC_INTERRUPT_SIZE */
	if( unData & 0x4000 ){
		//bFirstLoop = 1;
		INC_MSG_PRINTF(1, "[%s]==> FIFO FULL   : 0x%X \r\n", __FILE__, unData );
	}
	else if( !(unData & 0x3FFF ))	{
		unData = 0;
		INTERFACE_INT_CLEAR(TDMB_I2C_ID80, INC_MPI_INTERRUPT_ENABLE);
		INC_MSG_PRINTF(1, "[%s]==> FIFO Empty   : 0x%X \r\n", __FILE__, unData );
	}
	else{
		unData &= 0x3FFF;
		unData = INC_INTERRUPT_SIZE;
	}

//	printk("unData = %d \n", unData);
	if(unData < INC_INTERRUPT_SIZE) return INC_ERROR;

	memset(pBuff, 0xab, INC_INTERRUPT_SIZE);

	INC_CMD_READ_BURST(ucI2CID, APB_STREAM_BASE, pBuff, INC_INTERRUPT_SIZE);

/*
	for(i = 0 ; i < INC_INTERRUPT_SIZE/188 ; i++)
	{
		printk("sync byte [%d/%d]\n", *(pBuff + i*188), i);
	}
*/
	if((m_unIntCtrl & INC_INTERRUPT_LEVEL) && (!(m_unIntCtrl & INC_INTERRUPT_AUTOCLEAR_ENABLE)))
		INTERFACE_INT_CLEAR(ucI2CID, INC_MPI_INTERRUPT_ENABLE);

	return unData;
#endif
}

INC_UINT8 SAVE_CHANNEL_INFO(char* pStr)
{
/*	FILE* pFile = fopen(pStr, "wb+");
	if(pFile == NULL)
		return INC_ERROR;

	DWORD dwWriteLen = 0;

	dwWriteLen = fwrite(&g_stDabInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwWriteLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDabInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}

	dwWriteLen = fwrite(&g_stDmbInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwWriteLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDmbInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}

	dwWriteLen = fwrite(&g_stDataInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwWriteLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDataInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}

	fclose(pFile);
*/	return INC_SUCCESS;
}

INC_UINT8 LOAD_CHANNEL_INFO(char* pStr)
{
/*	FILE* pFile = fopen(pStr, "rb");
	if(pFile == NULL)
		return INC_ERROR;

	DWORD dwReadLen = 0;

	dwReadLen = fread(&g_stDabInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwReadLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDabInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}

	dwReadLen = fread(&g_stDmbInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwReadLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDmbInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}

	dwReadLen = fread(&g_stDataInfo, sizeof(char), sizeof(ST_SUBCH_INFO), pFile);
	if( dwReadLen != sizeof(ST_SUBCH_INFO) )
	{
		memset(&g_stDataInfo, 0x00, sizeof(ST_SUBCH_INFO));
	}
	fclose(pFile);
*/	return INC_SUCCESS;
}

void INTERFACE_INC_DEBUG(INC_UINT8 ucI2CID)
{
	INC_UINT16 nLoop = 0;
	for(nLoop = 0; nLoop < 3; nLoop++)
	{
		INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_DEINT_BASE+ 0x02+%d : 0x%X \r\n", nLoop*2, INC_CMD_READ(TDMB_I2C_ID80, APB_DEINT_BASE+ 0x02 + (nLoop*2)));
		INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_DEINT_BASE+ 0x03+%d : 0x%X \r\n", nLoop*2, INC_CMD_READ(TDMB_I2C_ID80, APB_DEINT_BASE + 0x03 + (nLoop*2)));
		INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_VTB_BASE  + 0x02+%d : 0x%X \r\n", nLoop, INC_CMD_READ(TDMB_I2C_ID80, APB_VTB_BASE+ 0x02 + nLoop));
	}

	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x00 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x00));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x01 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x01));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x02 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x02));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x03 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x03));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x04 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x04));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x05 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x05));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x06 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x06));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x07 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x07));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_MPI_BASE+ 0x08 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_MPI_BASE + 0x08));


	// INIT
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_INT_BASE+ 0x00 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_INT_BASE + 0x00));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_INT_BASE+ 0x01 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_INT_BASE + 0x01));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_INT_BASE+ 0x02 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_INT_BASE + 0x02));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_INT_BASE+ 0x03 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_INT_BASE + 0x03));

	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x3B : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x3B));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x00 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x00));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x84 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x84));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x86 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x86));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xB4 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xB4));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x1A : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x1A));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x8A : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x8A));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xC4 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xC4));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x24 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x24));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xBE : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xBE));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xB0 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xB0));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xC0 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xC0));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x8C : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x8C));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xA8 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xA8));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xAA : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xAA));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x80 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x80));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x88 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x88));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xC8 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xC8));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xBC : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xBC));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x90 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x90));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xCA : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xCA));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x40 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x40));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x24 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x24));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0x41 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0x41));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_PHY_BASE+ 0xC6 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_PHY_BASE + 0xC6));

	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_VTB_BASE+ 0x05 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_VTB_BASE + 0x05));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_VTB_BASE+ 0x01 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_VTB_BASE + 0x01));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_VTB_BASE+ 0x00 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_VTB_BASE + 0x00));

	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_RS_BASE+ 0x00 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_RS_BASE + 0x00));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_RS_BASE+ 0x07 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_RS_BASE + 0x07));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_RS_BASE+ 0x0A : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_RS_BASE + 0x0A));
	INC_MSG_PRINTF(INC_DEBUG_LEVEL, " APB_RS_BASE+ 0x01 : 0x%X \r\n", INC_CMD_READ(TDMB_I2C_ID80, APB_RS_BASE + 0x01));
}

