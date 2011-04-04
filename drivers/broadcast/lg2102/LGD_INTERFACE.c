#include <linux/broadcast/broadcast_lg2102_includes.h>
#include <linux/broadcast/broadcast_lg2102.h>

/*********************************************************************************/
/* Operating Chip set : LG2102                                                    */
/* Software version   : version 1.32                                             */
/* Software Update    : 2009.08.24                                              */
/*********************************************************************************/

#define LGD_INTERRUPT_LOCK()	{}//tdmb_lg2102_interrupt_lock()	
#define LGD_INTERRUPT_FREE()	{}//tdmb_lg2102_interrupt_free()


ST_SUBCH_INFO		g_stDmbInfo;
ST_SUBCH_INFO		g_stDabInfo;
ST_SUBCH_INFO		g_stDataInfo;

LGD_UINT16 			m_nMpiCSsize 	= MPI_CS_SIZE;
LGD_UINT16			m_nSpiIntrSize  = LGD_INTERRUPT_SIZE;
ENSEMBLE_BAND 		m_ucRfBand 		= KOREA_BAND_ENABLE;

/*********************************************************************************/
/*  RF Band Select                                                               */
/*                                                                               */
/*  LGD_UINT8 m_ucRfBand = KOREA_BAND_ENABLE,                                    */
/*						   BANDIII_ENABLE,                                                 */
/*						   LBAND_ENABLE,                                                   */
/*						   CHINA_ENABLE,                                                   */
/*						   EXTERNAL_ENABLE,                                                */
/*********************************************************************************/
CTRL_MODE 			m_ucCommandMode 	= LGD_SPI_CTRL;
ST_TRANSMISSION		m_ucTransMode		= TRANSMISSION_MODE1;
UPLOAD_MODE_INFO	m_ucUploadMode 		= STREAM_UPLOAD_SPI;
CLOCK_SPEED			m_ucClockSpeed 		= LGD_OUTPUT_CLOCK_4096;
LGD_ACTIVE_MODE		m_ucMPI_CS_Active 	= LGD_ACTIVE_HIGH;
LGD_ACTIVE_MODE		m_ucMPI_CLK_Active 	= LGD_ACTIVE_LOW;
LGD_UINT16			m_unIntCtrl			= (LGD_INTERRUPT_POLARITY_HIGH | \
										   LGD_INTERRUPT_PULSE | \
										   LGD_INTERRUPT_AUTOCLEAR_ENABLE | \
										   (LGD_INTERRUPT_PULSE_COUNT & LGD_INTERRUPT_PULSE_COUNT_MASK));


/*********************************************************************************/
/* PLL_MODE     m_ucPLL_Mode                                                     */
/*          LG2102  Input Clock Setting                                           */
/*********************************************************************************/
PLL_MODE			m_ucPLL_Mode		= INPUT_CLOCK_24576KHZ;


/*********************************************************************************/
/* LGD_DPD_MODE		m_ucDPD_Mode                                                   */
/*					LG2102  Power Saving mode setting                                     */
/*********************************************************************************/
LGD_DPD_MODE		m_ucDPD_Mode		= LGD_DPD_ON;


/*********************************************************************************/
/*  MPI Chip Select and Clock Setup Part                                         */
/*                                                                               */
/*  LGD_UINT8 m_ucCommandMode = LGD_I2C_CTRL, LGD_SPI_CTRL, LGD_EBI_CTRL         */
/*                                                                               */
/*  LGD_UINT8 m_ucUploadMode = STREAM_UPLOAD_MASTER_SERIAL,                      */
/*							   STREAM_UPLOAD_SLAVE_PARALLEL,                                 */
/*							   STREAM_UPLOAD_TS,                                             */
/*							   STREAM_UPLOAD_SPI,                                            */
/*                                                                               */
/*  LGD_UINT8 m_ucClockSpeed = LGD_OUTPUT_CLOCK_4096,                            */
/*							   LGD_OUTPUT_CLOCK_2048,                                        */
/*							   LGD_OUTPUT_CLOCK_1024,                                        */
/*********************************************************************************/

LGD_UINT8 m_auiRxBuff[4096] = {0};

/*********************************************************************************/
/* 반드시 1ms Delay함수를 넣어야 한다.                                           */
/*********************************************************************************/
LGD_UINT8 LGD_DELAY(LGD_UINT8 ucI2CID, LGD_UINT16 uiDelay)
{	
	//TODO Delay code here...
	if(tdmb_lg2102_mdelay(uiDelay)==LGD_ERROR)  	//inb612 for android compile
	{
		INTERFACE_USER_STOP(ucI2CID);
		return LGD_ERROR;
	}
	return LGD_SUCCESS;
}

LGD_UINT16 LGD_I2C_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr)
{
#ifdef LGD_I2C_GPIO_CTRL_ENABLE	
	LGD_UINT8 acBuff[2];
	LGD_UINT16 wData;
	LGD_I2C_ACK AckStatus;
	
	AckStatus = LGD_GPIO_CTRL_READ(ucI2CID, uiAddr, acBuff, 2);
	if(AckStatus == I2C_ACK_SUCCESS){
		wData = ((LGD_UINT16)acBuff[0] << 8) | (LGD_UINT16)acBuff[1];
		return wData;
	}
	return LGD_ERROR;
#else
	//TODO I2C Read code here...
	LGD_UINT8 acBuff[2];
	LGD_UINT16 wData;
	boolean AckStatus;
	
	//AckStatus= tdmb_lg2102_i2c_read_burst(uiAddr, acBuff,2);	//for android compile
	if(AckStatus == TRUE){
		wData = ((LGD_UINT16)acBuff[0] << 8) | (LGD_UINT16)acBuff[1];
		return wData;
	}
	
	return LGD_ERROR;
#endif
}

LGD_UINT8 LGD_I2C_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData)
{
#ifdef LGD_I2C_GPIO_CTRL_ENABLE	
	LGD_UINT8 acBuff[2];
	LGD_UINT8 ucCnt = 0;
	LGD_I2C_ACK AckStatus;

	acBuff[ucCnt++] = (uiData >> 8) & 0xff;
	acBuff[ucCnt++] = uiData & 0xff;
	
	AckStatus = LGD_GPIO_CTRL_WRITE(ucI2CID, uiAddr, acBuff, ucCnt);
	if(AckStatus == I2C_ACK_SUCCESS)
		return LGD_SUCCESS;
	return LGD_ERROR;
#else
	//TODO I2C write code here...
	LGD_UINT8 acBuff[2];
	LGD_UINT8 ucCnt = 0;
	boolean AckStatus;

	acBuff[ucCnt++] = (uiData >> 8) & 0xff;
	acBuff[ucCnt++] = uiData & 0xff;

	//AckStatus = tdmb_lg2102_i2c_write_burst(uiAddr, acBuff, ucCnt);	//for android compile
	if(AckStatus == TRUE)
		return LGD_SUCCESS;
	return LGD_ERROR;
#endif
}

LGD_UINT8 LGD_I2C_READ_BURST(LGD_UINT8 ucI2CID,  LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize)
{
#ifdef LGD_I2C_GPIO_CTRL_ENABLE	
	LGD_I2C_ACK AckStatus;
	AckStatus = LGD_GPIO_CTRL_READ(ucI2CID, uiAddr, pData, nSize);

	if(AckStatus == I2C_ACK_SUCCESS)
		return LGD_SUCCESS;
	return LGD_ERROR;
#else
	//TODO I2C Read code here...
	//LGD_UINT16 wData;
	boolean AckStatus;
	
	//AckStatus = tdmb_lg2102_i2c_read_burst(uiAddr, pData,nSize);	//for android compile
	if(AckStatus == TRUE)
		return LGD_SUCCESS;
	return LGD_ERROR;
#endif
}

LGD_UINT8 LGD_EBI_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData)
{
	LGD_UINT16 uiCMD = LGD_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	LGD_INTERRUPT_LOCK();

	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;

	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = (uiData >> 8) & 0xff;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS =  uiData & 0xff;
	LGD_INTERRUPT_FREE();
	return LGD_ERROR;
}

LGD_UINT16 LGD_EBI_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr)
{
	LGD_UINT16 uiRcvData = 0;
	LGD_UINT16 uiCMD = LGD_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	LGD_INTERRUPT_LOCK();
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;
	
	uiRcvData  = (*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff) << 8;
	uiRcvData |= (*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff);
	
	LGD_INTERRUPT_FREE();
	return uiRcvData;
}

LGD_UINT8 LGD_EBI_READ_BURST(LGD_UINT8 ucI2CID,  LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize)
{
	LGD_UINT16 uiLoop, nIndex = 0, anLength[2], uiCMD, unDataCnt;
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;
	
	if(nSize > LGD_MPI_MAX_BUFF) return LGD_ERROR;
	memset((LGD_INT8*)anLength, 0, sizeof(anLength));

	if(nSize > LGD_TDMB_LENGTH_MASK) {
		anLength[nIndex++] = LGD_TDMB_LENGTH_MASK;
		anLength[nIndex++] = nSize - LGD_TDMB_LENGTH_MASK;
	}
	else anLength[nIndex++] = nSize;

	LGD_INTERRUPT_LOCK();
	for(uiLoop = 0; uiLoop < nIndex; uiLoop++){

		uiCMD = LGD_REGISTER_CTRL(SPI_MEMREAD_CMD) | (anLength[uiLoop] & LGD_TDMB_LENGTH_MASK);
		
		*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
		*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
		*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
		*(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;
		
		for(unDataCnt = 0 ; unDataCnt < anLength[uiLoop]; unDataCnt++){
			*pData++ = *(volatile LGD_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff;
		}
	}
	LGD_INTERRUPT_FREE();

	return LGD_SUCCESS;
}

LGD_UINT16 LGD_SPI_REG_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr)
{
	LGD_UINT16 uiRcvData = 0;
	LGD_UINT16 uiCMD = LGD_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
	LGD_UINT8 auiTxBuff[6] = {0};
	LGD_UINT8 auiRxBuff[6] = {0};
	LGD_UINT8 cCnt = 0;
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	auiTxBuff[cCnt++] = uiNewAddr >> 8;
	auiTxBuff[cCnt++] = uiNewAddr & 0xff;
	auiTxBuff[cCnt++] = uiCMD >> 8;
	auiTxBuff[cCnt++] = uiCMD & 0xff;
	LGD_INTERRUPT_LOCK();
	tdmb_lg2102_spi_write_read(auiTxBuff, 4, auiRxBuff, 2);
	LGD_INTERRUPT_FREE();

	uiRcvData = auiRxBuff[4]<<8 | auiRxBuff[5];
	return uiRcvData;
}

LGD_UINT8 LGD_SPI_REG_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData)
{
	LGD_UINT16 uiCMD = LGD_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
	LGD_UINT8 auiTxBuff[6] = {0};
	LGD_UINT8 auiRxBuff[6] = {0};
	LGD_UINT8 cCnt = 0;
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	auiTxBuff[cCnt++] = uiNewAddr >> 8;
	auiTxBuff[cCnt++] = uiNewAddr & 0xff;
	auiTxBuff[cCnt++] = uiCMD >> 8;
	auiTxBuff[cCnt++] = uiCMD & 0xff;
	auiTxBuff[cCnt++] = uiData >> 8;
	auiTxBuff[cCnt++] = uiData & 0xff;
	LGD_INTERRUPT_LOCK();	
	tdmb_lg2102_spi_write_read(auiTxBuff, 6, auiRxBuff, 0);
	LGD_INTERRUPT_FREE();
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_SPI_READ_BURST(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 wSize)
{
	LGD_UINT16 uiLoop, nIndex = 0, anLength[2], uiCMD;
	LGD_UINT8 auiBuff[6];
	LGD_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	if( (pBuff==NULL) || (wSize > LGD_MPI_MAX_BUFF)) return LGD_ERROR;
	memset((LGD_INT8*)anLength, 0, sizeof(anLength));

	if(wSize > LGD_TDMB_LENGTH_MASK) {
		anLength[nIndex++] = LGD_TDMB_LENGTH_MASK;
		anLength[nIndex++] = wSize - LGD_TDMB_LENGTH_MASK;
	}
	else anLength[nIndex++] = wSize;

	LGD_INTERRUPT_LOCK();
	for(uiLoop = 0; uiLoop < nIndex; uiLoop++){

		auiBuff[0] = uiNewAddr >> 8;
		auiBuff[1] = uiNewAddr & 0xff;
		uiCMD = LGD_REGISTER_CTRL(SPI_MEMREAD_CMD) | (anLength[uiLoop] & LGD_TDMB_LENGTH_MASK);
		auiBuff[2] = uiCMD >> 8;
		auiBuff[3] = uiCMD & 0xff;
		tdmb_lg2102_spi_write_read(auiBuff, 4, m_auiRxBuff, anLength[uiLoop]);
		memcpy((LGD_UINT8*)&pBuff[uiLoop*anLength[uiLoop]], (LGD_UINT8*)&m_auiRxBuff[4], anLength[uiLoop]);
	}
	LGD_INTERRUPT_FREE();

	return LGD_SUCCESS;
}

LGD_UINT8 INTERFACE_DBINIT(void)
{
	memset(&g_stDmbInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDabInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDataInfo,	0, sizeof(ST_SUBCH_INFO));
	
	return LGD_SUCCESS;
}

void INTERFACE_UPLOAD_MODE(LGD_UINT8 ucI2CID, UPLOAD_MODE_INFO ucUploadMode)
{
	m_ucUploadMode = ucUploadMode;
	LGD_UPLOAD_MODE(ucI2CID);
}

LGD_UINT8 INTERFACE_PLL_MODE(LGD_UINT8 ucI2CID, PLL_MODE ucPllMode)
{
	m_ucPLL_Mode = ucPllMode;
	return LGD_PLL_SET(ucI2CID);
}

// 초기 전원 입력시 호출
LGD_UINT8 INTERFACE_INIT(LGD_UINT8 ucI2CID)
{
	return LGD_INIT(ucI2CID);
}

LGD_UINT8 INTERFACE_RESET_CH(LGD_UINT8 ucI2CID)
{
	LGD_RESET_MPI(ucI2CID);

	return LGD_SUCCESS;	
}


// 에러 발생시 에러코드 읽기
LGD_ERROR_INFO INTERFACE_ERROR_STATUS(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	return pInfo->nBbpStatus;
}

/*********************************************************************************/
/* 다중 채널 선택하여 시작하기....                                               */  
/* pChInfo->ucServiceType, pChInfo->ucSubChID, pChInfo->ulRFFreq 는              */
/* 반드시 넘겨주어야 한다.                                                       */
/* DMB채널 선택시 pChInfo->ucServiceType = 0x18                                  */
/* DAB, DATA채널 선택시 pChInfo->ucServiceType = 0으로 설정을 해야함.            */
/* pMultiInfo->nSetCnt은 선택한 서브채널 개수임.                                 */
/* FIC Data를 같이  선택시 LGD_MULTI_CHANNEL_FIC_UPLOAD 매크로를 풀어야 한다.    */
/* DMB  채널은 최대 2개채널 선택이 가능하다.                                     */
/* DAB  채널은 최대 3개채널 선택이 가능하다.                                     */
/* DATA 채널은 최대 3개채널 선택이 가능하다.                                     */
/*********************************************************************************/
LGD_UINT8 INTERFACE_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
	return LGD_CHANNEL_START(ucI2CID, pChInfo);
}

LGD_UINT8 INTERFACE_RE_SYNCDETECTOR(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pMultiInfo)
{
	return (LGD_UINT8)LGD_RE_SYNCDETECTOR(ucI2CID, pMultiInfo);
}

LGD_UINT8 INTERFACE_RE_SYNC(LGD_UINT8 ucI2CID)
{
	return LGD_RE_SYNC(ucI2CID);
}

/*********************************************************************************/
/* 스캔시  호출한다.                                                             */
/* 주파수 값은 받드시넘겨주어야 한다.                                            */
/* Band를 변경하여 스캔시는 m_ucRfBand를 변경하여야 한다.                        */
/* 주파수 값은 받드시넘겨주어야 한다.                                            */
/*********************************************************************************/
LGD_UINT8 INTERFACE_SCAN(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq)
{
	//LGD_INT16 nIndex;					//inb612 for android compile
	ST_FIC_DB* pstFicDb;
	//LGD_CHANNEL_INFO* pChInfo;
	pstFicDb = LGD_GETFIC_DB(ucI2CID);	//inb612 for android compile
	
	INTERFACE_DBINIT();

	if(!LGD_ENSEMBLE_SCAN(ucI2CID, ulFreq)) return LGD_ERROR;
 	pstFicDb->ulRFFreq = ulFreq;
	
#if 0 // kjyim
	for(nIndex = 0; nIndex < pstFicDb->ucSubChCnt; nIndex++){
		switch(pstFicDb->aucTmID[nIndex])
		{
		case TMID_1 : pChInfo = &g_stDmbInfo.astSubChInfo[g_stDmbInfo.nSetCnt++];	break;
		case TMID_0 : pChInfo = &g_stDabInfo.astSubChInfo[g_stDabInfo.nSetCnt++];	break;
		default   : pChInfo = &g_stDataInfo.astSubChInfo[g_stDataInfo.nSetCnt++];	break;
		}
		LGD_UPDATE(pChInfo, pstFicDb, nIndex);
	}
#endif	
	return LGD_SUCCESS;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DMB채널 개수를 리턴한다.                             */
/*********************************************************************************/
LGD_UINT16 INTERFACE_GETDMB_CNT(void)
{
	return (LGD_UINT16)g_stDmbInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DAB채널 개수를 리턴한다.                             */
/*********************************************************************************/
LGD_UINT16 INTERFACE_GETDAB_CNT(void)
{
	return (LGD_UINT16)g_stDabInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DATA채널 개수를 리턴한다.                            */
/*********************************************************************************/
LGD_UINT16 INTERFACE_GETDATA_CNT(void)
{
	return (LGD_UINT16)g_stDataInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 Ensemble lable을 리턴한다.                           */
/*********************************************************************************/
LGD_UINT8* INTERFACE_GETENSEMBLE_LABEL(LGD_UINT8 ucI2CID)
{
	ST_FIC_DB* pstFicDb;
	pstFicDb = LGD_GETFIC_DB(ucI2CID);
	return pstFicDb->aucEnsembleLabel;
}

/*********************************************************************************/
/* DMB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
LGD_CHANNEL_INFO* INTERFACE_GETDB_DMB(LGD_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return LGD_NULL;
	if(uiPos >= g_stDmbInfo.nSetCnt) return LGD_NULL;
	return &g_stDmbInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DAB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
LGD_CHANNEL_INFO* INTERFACE_GETDB_DAB(LGD_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return LGD_NULL;
	if(uiPos >= g_stDabInfo.nSetCnt) return LGD_NULL;
	return &g_stDabInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DATA 채널 정보를 리턴한다.                                                    */
/*********************************************************************************/
LGD_CHANNEL_INFO* INTERFACE_GETDB_DATA(LGD_INT16 uiPos)
{
	if(uiPos >= MAX_SUBCH_SIZE) return LGD_NULL;
	if(uiPos >= g_stDataInfo.nSetCnt) return LGD_NULL;
	return &g_stDataInfo.astSubChInfo[uiPos];
}

// 시청 중 FIC 정보 변경되었는지를 체크
LGD_UINT8 INTERFACE_RECONFIG(LGD_UINT8 ucI2CID)
{
	return LGD_FIC_RECONFIGURATION_HW_CHECK(ucI2CID);
}

LGD_UINT8 INTERFACE_STATUS_CHECK(LGD_UINT8 ucI2CID)
{
	return LGD_STATUS_CHECK(ucI2CID);
}

LGD_UINT16 INTERFACE_GET_CER(LGD_UINT8 ucI2CID)
{
	return LGD_GET_CER(ucI2CID);
}

LGD_UINT8 INTERFACE_GET_SNR(LGD_UINT8 ucI2CID)
{
	return LGD_GET_SNR(ucI2CID);
}

LGD_DOUBLE32 INTERFACE_GET_POSTBER(LGD_UINT8 ucI2CID)
{
	return LGD_GET_POSTBER(ucI2CID);
}

/* LGE MC Add for TP Error count*/
LGD_UINT16 INTERFACE_GET_TPERRCNT(LGD_UINT8 ucI2CID)
{
	return LGD_GET_TPERRCNT(ucI2CID);
}

LGD_DOUBLE32 INTERFACE_GET_PREBER(LGD_UINT8 ucI2CID)
{
	return LGD_GET_PREBER(ucI2CID);
}

/*********************************************************************************/
/* Scan, 채널 시작시에 강제로 중지시 호출한다.                                   */
/*********************************************************************************/
void INTERFACE_USER_STOP(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	pInfo->ucStop = 1;
}

// 인터럽트 인에이블...
void INTERFACE_INT_ENABLE(LGD_UINT8 ucI2CID, LGD_UINT16 unSet)
{
	LGD_INTERRUPT_SET(ucI2CID, unSet);
}

// Use when polling mode
LGD_UINT8 INTERFACE_INT_CHECK(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 nValue = 0;

	nValue = LGD_CMD_READ(ucI2CID, APB_INT_BASE+ 0x01);
	if(!(nValue & LGD_MPI_INTERRUPT_ENABLE))
		return FALSE;

	return TRUE;
}
// 인터럽스 클리어
void INTERFACE_INT_CLEAR(LGD_UINT8 ucI2CID, LGD_UINT16 unClr)
{
	LGD_INTERRUPT_CLEAR(ucI2CID, unClr);
}

// 인터럽트 서비스 루틴... // SPI Slave Mode or MPI Slave Mode
LGD_UINT8 INTERFACE_ISR(LGD_UINT8 ucI2CID, LGD_UINT8* pBuff)
{
	LGD_UINT16 unLoop;
	LGD_UINT32 ulAddrSelect;

	if(m_ucCommandMode != LGD_EBI_CTRL){
		if(m_ucUploadMode == STREAM_UPLOAD_SPI){
			LGD_SPI_READ_BURST(ucI2CID, APB_STREAM_BASE, pBuff, LGD_INTERRUPT_SIZE);
		}
		
		else if(m_ucUploadMode == STREAM_UPLOAD_SLAVE_PARALLEL)
		{
			ulAddrSelect = (ucI2CID == TDMB_I2C_ID80) ? STREAM_PARALLEL_ADDRESS : STREAM_PARALLEL_ADDRESS_CS;
			for(unLoop = 0; unLoop < LGD_INTERRUPT_SIZE; unLoop++){
				pBuff[unLoop] = *(volatile LGD_UINT8*)ulAddrSelect & 0xff;
			}
		}
	}
	else
	{
		LGD_EBI_READ_BURST(ucI2CID, APB_STREAM_BASE, pBuff, LGD_INTERRUPT_SIZE);
	}

	if((m_unIntCtrl & LGD_INTERRUPT_LEVEL) && (!(m_unIntCtrl & LGD_INTERRUPT_AUTOCLEAR_ENABLE)))
		INTERFACE_INT_CLEAR(ucI2CID, LGD_MPI_INTERRUPT_ENABLE);
	
	return LGD_SUCCESS;
}


LGD_UINT8 INTERFACE_CHANGE_BAND(LGD_UINT8 ucI2CID, LGD_UINT16 usBand)
{
	switch(usBand){
		case 0x01 : 
			m_ucRfBand = KOREA_BAND_ENABLE; 
			m_ucTransMode = TRANSMISSION_MODE1; 
			break;
		case 0x02 : 
			m_ucRfBand = BANDIII_ENABLE;
			m_ucTransMode = TRANSMISSION_MODE1; 
			break;
		case 0x04 : 
			m_ucRfBand = LBAND_ENABLE;
			m_ucTransMode = TRANSMISSION_MODE2; 
			break;
		case 0x08 : 
			//m_ucRfBand = CANADA_ENABLE;
			//m_ucTransMode = TRANSMISSION_MODE2; 
			//break;
		case 0x10 : 
			m_ucRfBand = CHINA_ENABLE;
			m_ucTransMode = TRANSMISSION_MODE1; break;
		default : return LGD_ERROR;
	}
	return LGD_SUCCESS;
}

LGD_UINT8 INTERFACE_FIC_UPDATE_CHECK(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);

	if(pInfo->IsChangeEnsemble == TRUE)
		return TRUE;

	return FALSE;
}
