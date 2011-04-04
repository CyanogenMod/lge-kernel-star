#include <linux/broadcast/broadcast_lg2102_includes.h>
LGD_UINT8		abyBuff[MAX_FIC_SIZE]; // kjyim
LGD_UINT16		wFicLen;

static ST_BBPINFO			g_astBBPRun[2];
extern ENSEMBLE_BAND		m_ucRfBand;
extern UPLOAD_MODE_INFO		m_ucUploadMode;
extern CLOCK_SPEED			m_ucClockSpeed;
extern LGD_ACTIVE_MODE		m_ucMPI_CS_Active;
extern LGD_ACTIVE_MODE		m_ucMPI_CLK_Active;
extern LGD_UINT16			m_nMpiCSsize;
extern LGD_UINT16			m_nSpiIntrSize;
extern CTRL_MODE 			m_ucCommandMode;
extern ST_TRANSMISSION		m_ucTransMode;
extern PLL_MODE				m_ucPLL_Mode;
extern LGD_DPD_MODE			m_ucDPD_Mode;
extern LGD_UINT16			m_unIntCtrl;

extern LGD_UINT16			m_nRSFrameCnt;

LGD_UINT8 LGD_CMD_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT16 uiData)
{
	if(m_ucCommandMode == LGD_SPI_CTRL) return LGD_SPI_REG_WRITE(ucI2CID, uiAddr, uiData);
	else if(m_ucCommandMode == LGD_I2C_CTRL) return LGD_I2C_WRITE(ucI2CID, uiAddr, uiData);
	else if(m_ucCommandMode == LGD_EBI_CTRL) return LGD_EBI_WRITE(ucI2CID, uiAddr, uiData);
	return LGD_I2C_WRITE(ucI2CID, uiAddr, uiData);
}

LGD_UINT16 LGD_CMD_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr)
{
	if(m_ucCommandMode == LGD_SPI_CTRL) return LGD_SPI_REG_READ(ucI2CID, uiAddr);
	else if(m_ucCommandMode == LGD_I2C_CTRL) return LGD_I2C_READ(ucI2CID, uiAddr);
	else if(m_ucCommandMode == LGD_EBI_CTRL) return LGD_EBI_READ(ucI2CID, uiAddr);
	return LGD_I2C_READ(ucI2CID, uiAddr);
}

LGD_UINT8 LGD_CMD_READ_BURST(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pData, LGD_UINT16 nSize)
{
	if(m_ucCommandMode == LGD_SPI_CTRL) return LGD_SPI_READ_BURST(ucI2CID, uiAddr, pData, nSize);
	else if(m_ucCommandMode == LGD_I2C_CTRL) return LGD_I2C_READ_BURST(ucI2CID, uiAddr, pData, nSize);
	else if(m_ucCommandMode == LGD_EBI_CTRL) return LGD_EBI_READ_BURST(ucI2CID, uiAddr, pData, nSize);
	return LGD_I2C_READ_BURST(ucI2CID, uiAddr, pData, nSize);
}

void LGD_RESET_MPI(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 uiMpiStatus, uiRsStatus;
	uiRsStatus = LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0);
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0, 0x0000);
	//LGD_DELAY(ucI2CID,50);
	uiMpiStatus = LGD_CMD_READ(ucI2CID, APB_MPI_BASE+ 0);
	LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0, uiMpiStatus|0x8000);
	//LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0, uiRsStatus|0x8000);
}

void LGD_INIT_MPI(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 uiMpiStatus, uiRsStatus;
	uiRsStatus = LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0);
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0, 0x0000);
	//LGD_DELAY(ucI2CID,50);
	uiMpiStatus = LGD_CMD_READ(ucI2CID, APB_MPI_BASE+ 0);
	LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0, uiMpiStatus|0x8000);
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0, uiRsStatus|0x8000);
}

void LGD_MPICLOCK_SET(LGD_UINT8 ucI2CID)
{
	if(m_ucUploadMode == STREAM_UPLOAD_TS){
		if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_1024) 	    LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x9918);
		else if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_2048) 	LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x440C);
		else if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_4096) 	LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x3308);
		else LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x3308);
		return;
	}

	if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_1024) 			LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x9918);
	else if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_2048)		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x440C);
	else if(m_ucClockSpeed == LGD_OUTPUT_CLOCK_4096)		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x2206);
	else LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE + 0x01, 0x2206);
}

void LGD_UPLOAD_MODE(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiStatus = 0x01;

	if(m_ucCommandMode != LGD_EBI_CTRL){
		if(m_ucUploadMode == STREAM_UPLOAD_SPI) 						uiStatus = 0x05;
		if(m_ucUploadMode == STREAM_UPLOAD_SLAVE_PARALLEL) 	uiStatus = 0x04;
		
		if(m_ucUploadMode == STREAM_UPLOAD_TS) 							uiStatus = 0x101;
		if(m_ucMPI_CS_Active == LGD_ACTIVE_HIGH) 						uiStatus |= 0x10;
		if(m_ucMPI_CLK_Active == LGD_ACTIVE_HIGH) 					uiStatus |= 0x20;

		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x00, uiStatus);
	}
	else
	{
		LGD_CMD_WRITE(ucI2CID, APB_SPI_BASE+ 0x00, 0x0011);
	}

	if(m_ucUploadMode == STREAM_UPLOAD_TS){
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x02, 188);
		/* change TSIF CLK 8 to 128 after TSIF EN DEACTIVED */
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x03, 128);
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x04, 188);
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x05, 0);
	}
	else {
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x02, MPI_CS_SIZE);
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x04, LGD_INTERRUPT_SIZE);
		LGD_CMD_WRITE(ucI2CID, APB_MPI_BASE+ 0x05, LGD_INTERRUPT_SIZE-188);
	}
}

void LGD_SWAP(ST_SUBCH_INFO* pMainInfo, LGD_UINT16 nNo1, LGD_UINT16 nNo2)
{
	LGD_CHANNEL_INFO  stChInfo;
	stChInfo = pMainInfo->astSubChInfo[nNo1];
	pMainInfo->astSubChInfo[nNo1] = pMainInfo->astSubChInfo[nNo2];
	pMainInfo->astSubChInfo[nNo2] = stChInfo;
}

void LGD_BUBBLE_SORT(ST_SUBCH_INFO* pMainInfo, LGD_SORT_OPTION Opt)
{
	LGD_INT16 nIndex, nLoop;
	LGD_CHANNEL_INFO* pDest;
	LGD_CHANNEL_INFO* pSour;

	if(pMainInfo->nSetCnt <= 1) return;

	for(nIndex = 0 ; nIndex < pMainInfo->nSetCnt-1; nIndex++) {
		pSour = &pMainInfo->astSubChInfo[nIndex];

		for(nLoop = nIndex + 1 ; nLoop < pMainInfo->nSetCnt; nLoop++) {
			pDest = &pMainInfo->astSubChInfo[nLoop];

			if(Opt == LGD_SUB_CHANNEL_ID){
				if(pSour->ucSubChID > pDest->ucSubChID)
					LGD_SWAP(pMainInfo, nIndex, nLoop);
			}
			else if(Opt == LGD_START_ADDRESS){
				if(pSour->uiStarAddr > pDest->uiStarAddr)
					LGD_SWAP(pMainInfo, nIndex, nLoop);
			}
			else if(Opt == LGD_BIRRATE)	{
				if(pSour->uiBitRate > pDest->uiBitRate)
					LGD_SWAP(pMainInfo, nIndex, nLoop);
			}
			else if(Opt == LGD_FREQUENCY){
				if(pSour->ulRFFreq > pDest->ulRFFreq)
					LGD_SWAP(pMainInfo, nIndex, nLoop);
			}
			else{
				if(pSour->uiStarAddr > pDest->uiStarAddr)
					LGD_SWAP(pMainInfo, nIndex, nLoop);
			}
		}
	}
}

LGD_UINT8 LGD_BUBBLE_SORT_by_TYPE(ST_SUBCH_INFO* pMainInfo)
{
	LGD_INT16 nIndex, nLoop;
	LGD_CHANNEL_INFO* pDest;
	LGD_CHANNEL_INFO* pSour;

	if(pMainInfo->nSetCnt <= 1) return FALSE;

	for(nIndex = 0 ; nIndex < pMainInfo->nSetCnt-1; nIndex++) 
	{
		if(pMainInfo->astSubChInfo[nIndex].uiTmID != TMID_1) // DMB
			continue;

		pSour = &pMainInfo->astSubChInfo[nIndex];

		for(nLoop = nIndex + 1 ; nLoop < pMainInfo->nSetCnt; nLoop++) 
		{
			if(pMainInfo->astSubChInfo[nLoop].uiTmID == TMID_1) // DMB
				continue;

			pDest = &pMainInfo->astSubChInfo[nLoop];
			LGD_SWAP(pMainInfo, nIndex, nLoop);
			return TRUE;
		}
	}
	return FALSE;
}

LGD_UINT8 LGD_UPDATE(LGD_CHANNEL_INFO* pChInfo, ST_FIC_DB* pFicDb, LGD_INT16 nIndex)
{
	LGD_INT16 		nLoop;
	SCH_DB_STRT* 	pDbStar;
	
	pDbStar = &pFicDb->astSchDb[nIndex];

	pChInfo->ulRFFreq						= pFicDb->ulRFFreq;
	pChInfo->uiEnsembleID				= pFicDb->uiEnsembleID;
	pChInfo->ucSubChID					= pFicDb->aucSubChID[nIndex];
	pChInfo->ucServiceType			= pFicDb->aucDSCType[nIndex];
	pChInfo->uiStarAddr					= pFicDb->auiStartAddr[nIndex];
	pChInfo->uiTmID							= pFicDb->aucTmID[nIndex];
	pChInfo->ulServiceID				= pFicDb->aulServiceID[nIndex];
	pChInfo->uiPacketAddr				= pFicDb->aucSetPackAddr[nIndex];
	pChInfo->uiBitRate					= pDbStar->uiBitRate;
	pChInfo->ucSlFlag						= pDbStar->ucShortLong;
	pChInfo->ucTableIndex				= pDbStar->ucTableIndex;
	pChInfo->ucOption						= pDbStar->ucOption;
	pChInfo->ucProtectionLevel	= pDbStar->ucProtectionLevel;
	pChInfo->uiDifferentRate		= pDbStar->uiDifferentRate;
	pChInfo->uiSchSize					= pDbStar->uiSubChSize;

#ifdef USER_APPLICATION_TYPE
		pChInfo->uiUserAppType			= pFicDb->astUserAppInfo.uiUserAppType[nIndex];
		pChInfo->uiUserAppDataLength= pFicDb->astUserAppInfo.ucUserAppDataLength[nIndex];
#endif
	
	for(nLoop = MAX_LABEL_CHAR-1; nLoop >= 0; nLoop--){
		if(pFicDb->aucServiceLabel[nIndex][nLoop] == 0x20)
			pFicDb->aucServiceLabel[nIndex][nLoop] = LGD_NULL;
		else{
			if(nLoop == MAX_LABEL_CHAR-1) 
				pFicDb->aucServiceLabel[nIndex][nLoop] = LGD_NULL;
			break;
		}
	}

	for(nLoop = MAX_LABEL_CHAR-1; nLoop >= 0; nLoop--){
		if(pFicDb->aucEnsembleLabel[nLoop] == 0x20)
			pFicDb->aucEnsembleLabel[nLoop] = LGD_NULL;
		else{
			if(nLoop == MAX_LABEL_CHAR-1) 
				pFicDb->aucEnsembleLabel[nLoop] = LGD_NULL;
			break;
		}
	}
	
#ifdef USER_APPLICATION_TYPE
	for(nLoop = MAX_USER_APP_DATA-1; nLoop >= 0; nLoop--){
		if(pFicDb->astUserAppInfo.aucUserAppData[nIndex][nLoop] == 0x20)
			pFicDb->astUserAppInfo.aucUserAppData[nIndex][nLoop] = LGD_NULL;
		else{
			if(nLoop == MAX_USER_APP_DATA-1) 
				pFicDb->astUserAppInfo.aucUserAppData[nIndex][nLoop] = LGD_NULL;
			break;
		}
	}
	memcpy(pChInfo->aucUserAppData, pFicDb->astUserAppInfo.aucUserAppData[nIndex], sizeof(LGD_UINT8)* MAX_USER_APP_DATA);
#endif

	memcpy(pChInfo->aucLabel, pFicDb->aucServiceLabel[nIndex], sizeof(LGD_UINT8)*MAX_LABEL_CHAR);
	memcpy(pChInfo->aucEnsembleLabel, pFicDb->aucEnsembleLabel, sizeof(LGD_UINT8)*MAX_LABEL_CHAR);
	return LGD_SUCCESS;
}

ST_BBPINFO* LGD_GET_STRINFO(LGD_UINT8 ucI2CID)
{
	if(ucI2CID == TDMB_I2C_ID80) return &g_astBBPRun[0];
	return &g_astBBPRun[1];
}

LGD_UINT16 LGD_GET_FRAME_DURATION(ST_TRANSMISSION cTrnsMode)
{
	LGD_UINT16 uPeriodFrame;
	switch(cTrnsMode){
	case TRANSMISSION_MODE1 : uPeriodFrame = MAX_FRAME_DURATION; break;
	case TRANSMISSION_MODE2 :
	case TRANSMISSION_MODE3 : uPeriodFrame = MAX_FRAME_DURATION/4; break;
	case TRANSMISSION_MODE4 : uPeriodFrame = MAX_FRAME_DURATION/2; break;
	default : uPeriodFrame = MAX_FRAME_DURATION; break;
	}
	return uPeriodFrame;
}

LGD_UINT8 LGD_GET_FIB_CNT(ST_TRANSMISSION ucMode)
{
	LGD_UINT8 ucResult = 0;
	switch(ucMode){
	case TRANSMISSION_MODE1 : ucResult = 12; break;
	case TRANSMISSION_MODE2 : ucResult = 3; break;
	case TRANSMISSION_MODE3 : ucResult = 4; break;
	case TRANSMISSION_MODE4 : ucResult = 6; break;
	default: ucResult = 12; break;
	}
	return ucResult;
}

void LGD_INTERRUPT_CTRL(LGD_UINT8 ucI2CID)
{
	LGD_CMD_WRITE(ucI2CID, APB_INT_BASE+ 0x00, m_unIntCtrl);
}

void LGD_INTERRUPT_CLEAR(LGD_UINT8 ucI2CID, LGD_UINT16 uiClrInt)
{
	LGD_CMD_WRITE(ucI2CID, APB_INT_BASE+ 0x03, uiClrInt);
}

void LGD_INTERRUPT_SET(LGD_UINT8 ucI2CID, LGD_UINT16 uiSetInt)
{
	LGD_UINT16 uiIntSet;
	uiIntSet = ~uiSetInt;
	LGD_CMD_WRITE(ucI2CID, APB_INT_BASE+ 0x02, uiIntSet);
}

LGD_UINT8 LGD_CHIP_STATUS(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiChipID;
	uiChipID = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x10);
	if((uiChipID & 0xF00) < 0xA00) return LGD_ERROR;
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_PLL_SET(LGD_UINT8 ucI2CID) 
{
	LGD_UINT16	wData;
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);

	wData = LGD_CMD_READ(ucI2CID, APB_GEN_BASE+ 0x02) & 0xFE00;
	switch(m_ucPLL_Mode){
	case INPUT_CLOCK_24576KHZ:
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x00, 0x7FFE);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x02, wData & 0xBFFF);
	break;
	
	case INPUT_CLOCK_27000KHZ:
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x02, wData | 0x41BE);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x03, 0x310A);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x00, 0x7FFF);
	break;
	
	case INPUT_CLOCK_19200KHZ:
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x02, wData | 0x413F);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x03, 0x1809);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x00, 0x7FFF);
	break;
	
	case INPUT_CLOCK_12000KHZ:
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x02, wData | 0x4200);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x03, 0x190A);
	LGD_CMD_WRITE(ucI2CID, APB_GEN_BASE+ 0x00, 0x7FFF);
	break;
	}
	if(m_ucPLL_Mode == INPUT_CLOCK_24576KHZ) return LGD_SUCCESS;
	LGD_DELAY(ucI2CID, 10);
	if(!(LGD_CMD_READ(ucI2CID, APB_GEN_BASE+ 0x04) & 0x0001)){
		pInfo->nBbpStatus = ERROR_PLL;
		return LGD_ERROR;
	}
	return LGD_SUCCESS;
}

void LGD_SET_CHANNEL(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
//	LGD_UINT16	uiStatus = 0;			//inb612 for android compile
	LGD_UINT16  uiLoop, wData;
	LGD_CHANNEL_INFO* pTempChInfo;
	
#ifdef LGD_MULTI_HEADER_ENABLE	
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0x00, 0x800 | 0x400);
#endif

	for(uiLoop = 0 ; uiLoop < pChInfo->nSetCnt; uiLoop++)
	{
		pTempChInfo = &pChInfo->astSubChInfo[uiLoop];
		wData = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x01) & (~(0x3 << uiLoop*2));
		
		if(pTempChInfo->uiTmID == TMID_1)
		{
			LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x01, wData | 0x02 << (uiLoop*2));
		}
		else {
			LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x01, wData | 0x01 << (uiLoop*2));
		}
		wData = LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x00);
		LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0x00, wData | (0x40 >> uiLoop));
	}
	
	wData = LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x00);
#ifdef LGD_MULTI_CHANNEL_FIC_UPLOAD
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0x00, wData | 0x8080);
#else
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0x00, wData | 0x8000);
#endif
}


LGD_UINT8 LGD_INIT(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	memset(pInfo, 0 , sizeof(ST_BBPINFO));

	if(m_ucTransMode < TRANSMISSION_MODE1 || m_ucTransMode > TRANSMISSION_AUTOFAST)
		return LGD_ERROR;

	pInfo->ucTransMode = m_ucTransMode;
	if(LGD_PLL_SET(ucI2CID) != LGD_SUCCESS) 		return LGD_ERROR;
	if(LGD_CHIP_STATUS(ucI2CID) != LGD_SUCCESS) 	return LGD_ERROR;
	
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x3B, 0xFFFF);
	LGD_DELAY(ucI2CID, 10);

	LGD_INTERRUPT_CTRL(ucI2CID);
	LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x00, 0x8000);
	LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x01, 0x01C1);
	LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x05, 0x0008);
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE + 0x01, TS_ERR_THRESHOLD);
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE + 0x09, 0x000C);

	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x00, 0xF0FF);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x88, 0x2210);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x98, 0x0000);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x41, 0xCCCC);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xB0, 0x8320);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xB4, 0x4C01);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xBC, 0x4088);
	
#ifdef LGD_AUTO_RESYNC_ENABLE	// VTB Auto Resync -> 0xE15C
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x40, 0xE15C); 
#else
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x40, 0xC15C);
#endif
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE + 0x0A, 0x80F0);

	switch(m_ucTransMode){
	case TRANSMISSION_AUTO:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x98, 0x8000);
		break;
	case TRANSMISSION_AUTOFAST:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x98, 0xC000);
		break;
	case TRANSMISSION_MODE1:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xD0, 0x7F1F);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x80, 0x4082);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x90, 0x0430);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xC8, 0x3FF1);
		break;
	case TRANSMISSION_MODE2:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xD0, 0x1F07);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x80, 0x4182); 
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x90, 0x0415); 
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xC8, 0x1FF1);
		break;
	case TRANSMISSION_MODE3:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xD0, 0x0F03);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x80, 0x4282);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x90, 0x0408);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xC8, 0x03F1);
		break;
	case TRANSMISSION_MODE4:
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xD0, 0x3F0F);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x80, 0x4382);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x90, 0x0420); 
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xC8, 0x3FF1);
		break;
	}

	LGD_MPICLOCK_SET(ucI2CID);
	LGD_UPLOAD_MODE(ucI2CID);

	switch(m_ucRfBand){
	case KOREA_BAND_ENABLE	: LGD_READY(ucI2CID,	175280); break;
	case BANDIII_ENABLE 	  : LGD_READY(ucI2CID,	174928); break;
	case LBAND_ENABLE		    : LGD_READY(ucI2CID,	1452960); break;
	case CHINA_ENABLE		    : LGD_READY(ucI2CID,	168160); break;
	case ROAMING_ENABLE 	  : LGD_READY(ucI2CID,	217280); break;
	case EXTERNAL_ENABLE 	  : return LGD_ERROR;		//for android compile
	}

	while((LGD_DELAY(ucI2CID, 100)==LGD_ERROR)){};
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_READY(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq)
{
	LGD_UINT16	uStatus = 0;
	LGD_UINT8   bModeFlag = 0;
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	if(ulFreq == 189008 || ulFreq == 196736 || ulFreq == 205280 || ulFreq == 213008
		|| ulFreq == 180064 || ulFreq == 188928 || ulFreq == 195936
		|| ulFreq == 204640 || ulFreq == 213360 || ulFreq == 220352
		|| ulFreq == 222064 || ulFreq == 229072 || ulFreq == 237488
		|| ulFreq == 180144 || ulFreq == 196144 || ulFreq == 205296
		|| ulFreq == 212144 || ulFreq == 213856 
		// for LBAND : Spur channel by suhheewon 09/10/20  
		|| ulFreq == 1466656 || ulFreq == 1475216 || ulFreq == 1482064 || ulFreq == 1490624) { 
		bModeFlag = 1;
	}
	if(m_ucRfBand == ROAMING_ENABLE) bModeFlag = 1;

	if(bModeFlag)
	{
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0xC6);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xC6, uStatus | 0x0008);
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x41) & 0xFC00;
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x41, uStatus | 0x111);
	}
	else
	{
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0xC6);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xC6, uStatus & 0xFFF7);
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x41) & 0xFC00;
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x41, uStatus | 0xCC);

		// for LBAND : LF/LT spectrum inversion : by suhheewon 09/10/20
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x40);
		if(ulFreq == 1461520 || ulFreq == 1485488 ){
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x40, uStatus | (1<<10));
		}
		else
		{
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x40, uStatus & (~(1<<10)));
		}
	}

	if(LGD_RF500_START(ucI2CID, ulFreq, m_ucRfBand) != LGD_SUCCESS){
		pInfo->nBbpStatus = ERROR_READY;
		return LGD_ERROR;
	}

	if(m_ucDPD_Mode == LGD_DPD_OFF){
		uStatus = LGD_CMD_READ(ucI2CID, APB_RF_BASE+ 0x03);
		LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+ 0x03, uStatus | 0x80);
	}
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_SYNCDETECTOR(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq, LGD_UINT8 ucScanMode)
{
	LGD_UINT16 wOperState ,wIsNullSync = 0 /*, wSyncRefTime = 800	for android compile*/;
	LGD_UINT16 uiTimeOut = 0, uiRefSleep = 30;
	LGD_UINT16 wData = 0, wCOSValue, wCOSCount = 0, wMode4Count=0;
	//LGD_UINT8	 IsSyncNull = 0;		for android compile
	ST_BBPINFO* pInfo;

	pInfo = LGD_GET_STRINFO(ucI2CID);

	LGD_SCAN_SETTING(ucI2CID);

	if(m_ucDPD_Mode == LGD_DPD_ON){
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xA8, 0x3000);
		LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xAA, 0x0000);
	}

	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xBC, 0x4088);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0x3A, 0x1);

#if 1	//for android user stop
	//for(tmp_delay_cnt = 0; tmp_delay_cnt < 200; tmp_delay_cnt++)
	{
		LGD_DELAY(ucI2CID, 200);
		if(pInfo->ucStop){
			pInfo->nBbpStatus = ERROR_USER_STOP;
			printk("tdmb_lg2102_mdelay user stop 200ms \n");
			return LGD_ERROR;
		}
	}
#endif

	while(1)
	{
		if(uiTimeOut++ >= (2500 / uiRefSleep)){
			pInfo->nBbpStatus = ERROR_SYNC_TIMEOUT;
			break;
		}

		//for android user stop
		LGD_DELAY(ucI2CID, uiRefSleep);
		if(pInfo->ucStop){
			pInfo->nBbpStatus = ERROR_USER_STOP;
			printk("tdmb_lg2102_mdelay user stop 30ms \n");
			return LGD_ERROR;
		}
#if 0
		for(tmp_delay_cnt = 0; tmp_delay_cnt < 10; tmp_delay_cnt++)
		{

			LGD_DELAY(ucI2CID, 3);
			if(pInfo->ucStop){
				pInfo->nBbpStatus = ERROR_USER_STOP;
				printk("tdmb_lg2102_mdelay user stop 200ms \n");
				return LGD_ERROR;
			}
		}
#endif			
		wOperState = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x10);
		wOperState = ((wOperState & 0x7000) >> 12);

		if(!wIsNullSync && wOperState >= 0x2) wIsNullSync = 1;
		if(!wIsNullSync){
			wData = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x14) & 0x0F00; 			
			if(wData < 0x0200){
				pInfo->nBbpStatus = ERROR_SYNC_NO_SIGNAL;
				break;
			}
		}
		if(wOperState >= 0x5){
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xBC, 0x4008);
			return LGD_SUCCESS;
		}

		if(wOperState >= 4 && wMode4Count == 0)
		{
			wCOSValue = LGD_CMD_READ(ucI2CID, APB_PHY_BASE + 0xD2) & 0xFFFF;
			if(wCOSValue <= 0x350 /*&& wCOSValue >= 0 for android compile*/) {
				wMode4Count++;
			}
			else	
				continue;
		}

		if(wOperState >= 4 && wMode4Count >= 1)
		{
			wCOSValue = LGD_CMD_READ(ucI2CID, APB_PHY_BASE + 0x28) & 0x7FF;
			if(wCOSValue == 0) {
				wCOSCount++;

				if(wCOSCount >= 3){
					pInfo->nBbpStatus = ERROR_SYNC_NULL;
					break;
				}
				else continue;
			}
			wCOSValue = (wCOSValue >= 0x100) ? wCOSValue - 0x100 : 0x100 - wCOSValue;

			if(wCOSValue >= 20){
				pInfo->nBbpStatus = ERROR_SYNC_NULL;
				break;
			}
		}
	}

	return LGD_ERROR;
}


LGD_UINT8 LGD_FICDECODER(LGD_UINT8 ucI2CID, ST_SIMPLE_FIC bSimpleFIC)
{
	LGD_UINT16		uPeriodFrame, uFIBCnt, uiRefSleep;
 	LGD_UINT16		nLoop, nFicCnt;
	ST_BBPINFO*		pInfo;

#ifdef USER_APPLICATION_TYPE
	LGD_UINT8		ucCnt, ucCompCnt;
	ST_FIC_DB* 		pFicDb;
	LGD_UINT8		ucIsUserApp = LGD_ERROR;
	pFicDb = LGD_GETFIC_DB(ucI2CID);
#endif
	
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	LGD_INITDB(ucI2CID);
	uFIBCnt = LGD_GET_FIB_CNT(m_ucTransMode);
	uPeriodFrame = LGD_GET_FRAME_DURATION(m_ucTransMode);
	uiRefSleep = uPeriodFrame >> 2;

	//nFicCnt = FIC_REF_TIME_OUT / uiRefSleep;
	nFicCnt = 4;

	for(nLoop = 0; nLoop < nFicCnt; nLoop++)
	{
		LGD_DELAY(ucI2CID, uiRefSleep);

		//for android user stop
		if(pInfo->ucStop == 1)	{
			pInfo->nBbpStatus = ERROR_USER_STOP;
			return LGD_ERROR; // kjyim
		}
		
		if(!(LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x00) & 0x4000))
			continue;
		
		wFicLen = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x09);
		if(!wFicLen) continue;
		wFicLen++;
		
		if(wFicLen != (uFIBCnt*FIB_SIZE)) continue;

		if(LGD_CMD_READ_BURST(ucI2CID, APB_FIC_BASE, abyBuff, wFicLen))
		{
			if(bSimpleFIC == SIMPLE_FIC_DISABLE)
			{
				return LGD_SUCCESS;
			}
		
			if(LGD_FICPARSING(ucI2CID, abyBuff, wFicLen, bSimpleFIC))
			{

#ifdef USER_APPLICATION_TYPE
				if(bSimpleFIC == SIMPLE_FIC_ENABLE) 
					return LGD_SUCCESS;
				else {
					for(ucCompCnt = ucCnt = 0; ucCnt < pFicDb->ucSubChCnt; ucCnt++){
						if((pFicDb->uiUserAppTypeOk >> ucCnt) & 0x01) ucCompCnt++;
					}
					if(pFicDb->ucServiceComponentCnt == ucCompCnt) 
						return LGD_SUCCESS;
				}
				ucIsUserApp = LGD_SUCCESS;
#else
				return LGD_SUCCESS;
#endif
			}
		}
	}

#ifdef USER_APPLICATION_TYPE
	if(ucIsUserApp == LGD_SUCCESS)
		return LGD_SUCCESS;
#endif
	
	pInfo->nBbpStatus = ERROR_FICDECODER;
	
	return LGD_ERROR;
}

LGD_UINT8 LGD_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo, LGD_UINT16 IsEnsembleSame)
{
	LGD_INT16 	nLoop, nSchID;
	LGD_UINT16 	wData;
	ST_BBPINFO* pInfo;
	LGD_CHANNEL_INFO* pTempChInfo;

	LGD_UINT16 wCeil, wIndex, wStartAddr, wEndAddr;

	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	if(IsEnsembleSame){
		for(nLoop = 0; nLoop < 20; nLoop++){
			if(LGD_CMD_READ(ucI2CID, APB_DEINT_BASE+ 0x01) & 0x8000) break;
			LGD_DELAY(ucI2CID, 3);
			#if 1//for android user stop
			if(pInfo->ucStop == 1)	
			{
				pInfo->nBbpStatus = ERROR_USER_STOP;
				return LGD_ERROR; // kjyim
			}
			#endif
		}
		if(nLoop == 20){
			pInfo->nBbpStatus = ERROR_START_MODEM_CLEAR;
			return LGD_ERROR;
		}
	}

	///////////////////////////
	// sort by startAddress
	///////////////////////////
	LGD_BUBBLE_SORT(pChInfo, LGD_START_ADDRESS);

	for(nLoop = 0; nLoop < 3; nLoop++)
	{
		if(nLoop >= pChInfo->nSetCnt){
			LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x02 + (nLoop*2), 0x00);
			LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x03 + (nLoop*2), 0x00);
			LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x02 + nLoop, 0x00);
			continue;
		}
		pTempChInfo= &pChInfo->astSubChInfo[nLoop];
		
		
		nSchID = (LGD_UINT16)pTempChInfo->ucSubChID & 0x3f;
		LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x02 + (nLoop*2), (LGD_UINT16)(((LGD_UINT16)nSchID << 10) + pTempChInfo->uiStarAddr));

		if(pTempChInfo->ucSlFlag == 0) {
			LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x03 + (nLoop*2), 0x8000 + (pTempChInfo->ucTableIndex & 0x3f));
			LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x02 + nLoop, (pTempChInfo->ucTableIndex & 0x3f));
		}
		else{
			LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE + 0x03 + (nLoop*2), 0x8000 + 0x400 + pTempChInfo->uiSchSize);
			wData = 0x8000 
					+ ((pTempChInfo->ucOption & 0x7) << 12) 
					+ ((pTempChInfo->ucProtectionLevel & 0x3) << 10) 
					+ pTempChInfo->uiDifferentRate;
			LGD_CMD_WRITE(ucI2CID, APB_VTB_BASE+ 0x02 + nLoop, wData);
		}

		if(m_ucDPD_Mode == LGD_DPD_ON){
			switch(pInfo->ucTransMode){
				default : 
				case TRANSMISSION_MODE1 : wCeil = 3072;	wIndex = 4; break; 
				case TRANSMISSION_MODE2 : wCeil = 768;	wIndex = 4;	break;
				case TRANSMISSION_MODE3 : wCeil = 384;	wIndex = 9;	break;
				case TRANSMISSION_MODE4 : wCeil = 1536;	wIndex = 4;	break;
			}

			wStartAddr = ((pTempChInfo->uiStarAddr * 64) / wCeil) + wIndex - 2;
			wEndAddr = (LGD_UINT16)(((pTempChInfo->uiStarAddr + pTempChInfo->uiSchSize) * 64) / wCeil) + wIndex + 2;
			wData = (wStartAddr & 0xFF) << 8 | (wEndAddr & 0xFF);
			
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xAA + (nLoop * 2), wData);
			wData = LGD_CMD_READ(ucI2CID, APB_PHY_BASE + 0xA8);
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xA8, wData | (1<<nLoop));
		}
		else{
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xAA + (nLoop * 2), 0x0000);
			LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE + 0xA8, 0x3000);
		}

		wData = LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x00);
		LGD_CMD_WRITE(ucI2CID, APB_RS_BASE+ 0x00, wData | (0x1 << (6-nLoop)));
	}
	
	LGD_SET_CHANNEL(ucI2CID, pChInfo);
	LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x01, 0x0011);

	return LGD_SUCCESS;
}

LGD_UINT8 LGD_STOP(LGD_UINT8 ucI2CID)
{
	LGD_INT16 nLoop;
	LGD_UINT16 uStatus;
	ST_TRANSMISSION ucTransMode;
	ST_BBPINFO* pInfo;

	pInfo = LGD_GET_STRINFO(ucI2CID);
	ucTransMode = pInfo->ucTransMode;
	memset(pInfo, 0, sizeof(ST_BBPINFO));
	pInfo->ucTransMode = ucTransMode;
	LGD_CMD_WRITE(ucI2CID, APB_RS_BASE + 0x00, 0x0000);

	uStatus = LGD_CMD_READ(ucI2CID, APB_DEINT_BASE+ 0x01) & 0xFFFF;

	if(uStatus == 0x0011){
		LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x01, 0x0001);
		if(m_ucCommandMode == LGD_SPI_CTRL)
		{
			LGD_DELAY(ucI2CID, 30); 
		}
		
		for(nLoop = 0; nLoop < 10; nLoop++){
			uStatus = LGD_CMD_READ(ucI2CID, APB_DEINT_BASE+ 0x01) & 0xFFFF;
			if(uStatus != 0xFFFF && (uStatus & 0x8000)) break;
			LGD_DELAY(ucI2CID,10);
		}
		if(nLoop >= 10){
			pInfo->nBbpStatus = ERROR_STOP;
			return LGD_ERROR;
		}
	}

	//LGD_CMD_WRITE(ucI2CID, APB_RS_BASE   + 0x00, 0x0000);
	uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x10) & 0x7000;
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE  + 0x3B, 0x4000);

	if(uStatus >= 0x5000) LGD_DELAY(ucI2CID,25);

	for(nLoop = 0; nLoop < 10; nLoop++){
		uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x3B) & 0xFFFF;
		if(uStatus == 0x8000) break;
		LGD_DELAY(ucI2CID,10);
	}

	if(nLoop >= 10){
		pInfo->nBbpStatus = ERROR_STOP;
		return LGD_ERROR;
	}
	
	return LGD_SUCCESS;
}

void LGD_SCAN_SETTING(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 uStatus;
	uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x41) & 0xFFF;
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x41, uStatus | 0xC000);
	
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_IFDELAY_LSB, LGD_SCAN_IF_DLEAY_MAX&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_IFDELAY_MSB, (LGD_SCAN_IF_DLEAY_MAX>>8)&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_RFDELAY_LSB, LGD_SCAN_RF_DLEAY_MAX&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_RFDELAY_MSB, (LGD_SCAN_RF_DLEAY_MAX>>8)&0xff);
}

void LGD_AIRPLAY_SETTING(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 uStatus;
	uStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x41) & 0xFFF;
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x41, uStatus | 0x4000);

	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_IFDELAY_LSB, LGD_AIRPLAY_IF_DLEAY_MAX&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_IFDELAY_MSB, (LGD_AIRPLAY_IF_DLEAY_MAX>>8)&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_RFDELAY_LSB, LGD_AIRPLAY_RF_DLEAY_MAX&0xff);
	LGD_CMD_WRITE(ucI2CID, APB_RF_BASE+LGD_ADDRESS_RFDELAY_MSB, (LGD_AIRPLAY_RF_DLEAY_MAX>>8)&0xff);
}


LGD_UINT8 LGD_CHANNEL_START(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
	LGD_UINT16 wEnsemble;
	LGD_UINT16 wData;
	LGD_UINT32 ulRFFreq;
	ST_BBPINFO* pInfo;
	
	LGD_INTERRUPT_SET(ucI2CID,LGD_MPI_INTERRUPT_ENABLE);
	LGD_INTERRUPT_CLEAR(ucI2CID, LGD_MPI_INTERRUPT_ENABLE);
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	pInfo->IsReSync 			= 0;
	pInfo->ucStop = 0;
	pInfo->nBbpStatus = ERROR_NON;
	ulRFFreq		= pChInfo->astSubChInfo[0].ulRFFreq;
	
	wEnsemble = pInfo->ulFreq == ulRFFreq;
	
	if(!wEnsemble){
		if(LGD_STOP(ucI2CID) != LGD_SUCCESS)return LGD_ERROR;
		if(LGD_READY(ucI2CID, ulRFFreq) != LGD_SUCCESS) return LGD_ERROR;
		if(LGD_SYNCDETECTOR(ucI2CID,ulRFFreq, 0) != LGD_SUCCESS)return LGD_ERROR;
		if(LGD_FICDECODER(ucI2CID, SIMPLE_FIC_ENABLE) != LGD_SUCCESS)return LGD_ERROR;
		if(LGD_FIC_UPDATE(ucI2CID, pChInfo) != LGD_SUCCESS)	return LGD_ERROR;
		if(LGD_START(ucI2CID, pChInfo, 0) != LGD_SUCCESS) return LGD_ERROR;
	}
	else{
		wData = LGD_CMD_READ(ucI2CID, APB_DEINT_BASE+ 0x01);
		LGD_CMD_WRITE(ucI2CID, APB_DEINT_BASE+ 0x01, wData & 0xFFEF);
		if(m_ucCommandMode == LGD_SPI_CTRL)	{
			LGD_DELAY(ucI2CID, 30); 
		}
		if(LGD_SYNCDETECTOR(ucI2CID, ulRFFreq, 0) != LGD_SUCCESS){
			pInfo->ulFreq = 0;
			return LGD_ERROR;
		}
		if(LGD_FIC_UPDATE(ucI2CID, pChInfo) != LGD_SUCCESS) return LGD_ERROR;
		if(LGD_START(ucI2CID, pChInfo, 0) != LGD_SUCCESS) return LGD_ERROR;
	}

	LGD_AIRPLAY_SETTING(ucI2CID);
	
	pInfo->ucProtectionLevel = pChInfo->astSubChInfo[0].ucProtectionLevel;
	pInfo->ulFreq = ulRFFreq;
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_RE_SYNCDETECTOR(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pMultiInfo)
{
	LGD_UINT16 wEnsemble;
	LGD_UINT32 ulRFFreq;
	ST_BBPINFO* pInfo;
	
	LGD_INTERRUPT_SET(ucI2CID,LGD_MPI_INTERRUPT_ENABLE);
	LGD_INTERRUPT_CLEAR(ucI2CID, LGD_MPI_INTERRUPT_ENABLE);
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	pInfo->IsReSync 			= 0;
	pInfo->ucStop = 0;
	pInfo->nBbpStatus = ERROR_NON;
	ulRFFreq		= pMultiInfo->astSubChInfo[0].ulRFFreq;

	wEnsemble = pInfo->ulFreq == ulRFFreq;

	if(LGD_SYNCDETECTOR(ucI2CID, ulRFFreq, 0) != LGD_SUCCESS){
			pInfo->ulFreq = 0;
			return LGD_ERROR;
		}

	if(LGD_FICDECODER(ucI2CID, SIMPLE_FIC_ENABLE) != LGD_SUCCESS)return LGD_ERROR;
	
	if(LGD_FIC_UPDATE(ucI2CID, pMultiInfo) != LGD_SUCCESS) return LGD_ERROR;

	if(LGD_START(ucI2CID, pMultiInfo, wEnsemble) != LGD_SUCCESS) return LGD_ERROR;

	LGD_AIRPLAY_SETTING(ucI2CID);
	
	pInfo->ucProtectionLevel = pMultiInfo->astSubChInfo[0].ucProtectionLevel;
	pInfo->ulFreq = ulRFFreq;

	return LGD_SUCCESS;
}

LGD_UINT8 LGD_ENSEMBLE_SCAN(LGD_UINT8 ucI2CID, LGD_UINT32 ulFreq)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	pInfo->nBbpStatus = ERROR_NON;
	pInfo->ucStop = 0;

	if(LGD_STOP(ucI2CID) != LGD_SUCCESS) return LGD_ERROR;
	if(LGD_READY(ucI2CID, ulFreq) != LGD_SUCCESS)return LGD_ERROR;
	if(LGD_SYNCDETECTOR(ucI2CID, ulFreq, 1) != LGD_SUCCESS)	return LGD_ERROR;
#if 0 // kjyim
	if(LGD_FICDECODER(ucI2CID, SIMPLE_FIC_DISABLE) != LGD_SUCCESS)	return LGD_ERROR;
#endif
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_FIC_RECONFIGURATION_HW_CHECK(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 wStatus, uiFicLen, wReconf;
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);

	wStatus = (LGD_UINT16)LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x00);

	if(!(wStatus & 0x4000)) return LGD_ERROR;
	uiFicLen = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x09);

	if(uiFicLen != MAX_FIC_SIZE) return LGD_ERROR;
	wReconf = (LGD_UINT16)LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x0A);

	if(wReconf & 0xC0){
		pInfo->ulReConfigTime = (LGD_UINT16)(LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x0B) & 0xff) * 24;
		return LGD_SUCCESS;
	}
	return LGD_ERROR;
}

LGD_UINT8 LGD_RE_SYNC(LGD_UINT8 ucI2CID)
{
	LGD_INT16 nLoop;
	LGD_UINT16 unStatus;
	ST_BBPINFO*	pInfo;

	pInfo = LGD_GET_STRINFO(ucI2CID);

	pInfo->IsReSync = 1;
	pInfo->wDiffErrCnt = 0;
	pInfo->ucCERCnt = pInfo->ucRetryCnt = 0;

	unStatus = LGD_CMD_READ(ucI2CID, APB_RF_BASE+ 0x53);
	if(unStatus != (LGD_SCAN_RF_DLEAY_MAX&0xff)){
		LGD_SCAN_SETTING(ucI2CID);
	}
	
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x3B, 0x4000);
	
	for(nLoop = 0; nLoop < 15; nLoop++){
		if(LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x3B) & 0x8000) break;
		LGD_DELAY(ucI2CID, 2);
		#if 1//for android user stop
		if(pInfo->ucStop == 1)	
		{
			pInfo->nBbpStatus = ERROR_USER_STOP;
			return LGD_ERROR; // kjyim
		}
		#endif
	}
	
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x3A, 0x1);
	
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_STATUS_CHECK(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	
	LGD_GET_CER(ucI2CID);
	LGD_GET_PREBER(ucI2CID);
	LGD_GET_POSTBER(ucI2CID);
	LGD_GET_RSSI(ucI2CID);
	LGD_GET_ANT_LEVEL(ucI2CID);


	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	if(pInfo->uiCER >= 1200) 
	{
		pInfo->ucCERCnt++;
		if(pInfo->ucCERCnt == 0)
		{
			pInfo->ucCERCnt = LGD_CER_PERIOD;
			printk("LGD_STATUS_CHECK CER count Wrap Around !!!\n");
		}
	}
	else 
	{
		pInfo->ucCERCnt = 0;
	}
	
	if(pInfo->uiCER < 700)
	{
		if((pInfo->dPostBER*100) > 99) 
		{
			pInfo->ucRetryCnt++;			//inb612 for android compile
			if(pInfo->ucRetryCnt == 0)
			{
				pInfo->ucRetryCnt = LGD_BIT_PERIOD;
				printk("LGD_STATUS_CHECK Retry count Wrap Around !!!\n");
			}
		}
		else 
		{
			pInfo->ucRetryCnt = 0;
		}
	}
	
#ifdef LGD_AUTO_RESYNC_ENABLE
	if(pInfo->ucCERCnt >= LGD_CER_PERIOD || pInfo->ucRetryCnt >= LGD_BIT_PERIOD){
		return LGD_ERROR;
	}
#else
	if(pInfo->ucCERCnt >= LGD_CER_PERIOD || pInfo->ucRetryCnt >= LGD_BIT_PERIOD){
		LGD_RE_SYNC(ucI2CID);
		return LGD_ERROR;
	}

	// by jin : 09/11/09
	if(pInfo->IsReSync == 1 && pInfo->ucCERCnt == 0 && pInfo->ucRetryCnt == 0)
	{
		pInfo->IsReSync = 0;
		LGD_AIRPLAY_SETTING(ucI2CID);
	}
#endif
	return LGD_SUCCESS;
}

LGD_UINT16 LGD_GET_CER(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 	uiVtbErr;
	LGD_UINT16	uiVtbData;
	ST_BBPINFO* pInfo;
	LGD_INT16 	nLoop;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	uiVtbErr 	= LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x06);
	uiVtbData 	= LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x08);

	pInfo->uiCER = (!uiVtbData) ? 0 : (LGD_UINT16)((LGD_DOUBLE32)(uiVtbErr * 10000) / (uiVtbData * 64));
	if(uiVtbErr == 0) pInfo->uiCER = 2000;

	if(pInfo->uiCER <= BER_REF_VALUE) pInfo->auiANTBuff[pInfo->uiInCAntTick++ % BER_BUFFER_MAX] = 0;
	else pInfo->auiANTBuff[pInfo->uiInCAntTick++ % BER_BUFFER_MAX] = (pInfo->uiCER - BER_REF_VALUE);
	
	for(nLoop = 0 , pInfo->uiInCERAvg = 0; nLoop < BER_BUFFER_MAX; nLoop++)
		pInfo->uiInCERAvg += pInfo->auiANTBuff[nLoop];

	pInfo->uiInCERAvg /= BER_BUFFER_MAX;

	return pInfo->uiCER;
}

LGD_UINT8 LGD_GET_ANT_LEVEL(LGD_UINT8 ucI2CID)
{
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	LGD_GET_SNR(ucI2CID);
	
	if(pInfo->ucSnr > 16)				pInfo->ucAntLevel = 6;
	else if(pInfo->ucSnr > 13)	pInfo->ucAntLevel = 5;
	else if(pInfo->ucSnr > 10)	pInfo->ucAntLevel = 4;
	else if(pInfo->ucSnr > 7)		pInfo->ucAntLevel = 3;
	else if(pInfo->ucSnr > 5)		pInfo->ucAntLevel = 2;
	else if(pInfo->ucSnr > 3)		pInfo->ucAntLevel = 1;
	else	pInfo->ucAntLevel = 0;
	
	return pInfo->ucAntLevel;
}

LGD_DOUBLE32 LGD_GET_PREBER(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 		uiVtbErr;
	LGD_UINT16		uiVtbData;
	LGD_DOUBLE32	dPreBER;
	
	uiVtbErr 	= LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x06);
	uiVtbData = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x08);
	
	dPreBER = (!uiVtbData) ? 0 : (LGD_UINT16)((LGD_DOUBLE32)(uiVtbErr * 10000) / (uiVtbData * 64));	//inb612 double for android compile
	return dPreBER;
}

LGD_DOUBLE32 LGD_GET_POSTBER(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiRSErrBit;
	LGD_UINT16	uiRSErrTS;
	LGD_UINT16	uiError, uiRef;
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	uiRSErrBit 	= LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x02);
	uiRSErrTS 	= LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x03);
	uiRSErrBit += (uiRSErrTS * 8);

	if(uiRSErrTS == 0){
		uiError = ((uiRSErrBit * 50)  / 1000);
		uiRef = 0;
		if(uiError > 50) uiError = 50;
	}
	else if((uiRSErrTS > 0) && (uiRSErrTS < 10)){
		uiError = ((uiRSErrBit * 10)  / 2400);
		uiRef = 50;
		if(uiError > 50) uiError = 50;
	}
	else if((uiRSErrTS >= 10) && (uiRSErrTS < 30)){
		uiError = ((uiRSErrBit * 10)  / 2400);
		uiRef = 60;
		if(uiError > 40) uiError = 40;
	}
	else if((uiRSErrTS >= 30) && (uiRSErrTS < 80)){
		uiError = ((uiRSErrBit * 10)  / 2400);
		uiRef = 70;
		if(uiError > 30) uiError = 30;
	}
	else if((uiRSErrTS >= 80) && (uiRSErrTS < 100)){
		uiError = ((uiRSErrBit * 10)  / 2400);
		uiRef = 80;
		if(uiError > 20) uiError = 20;
	}
	else{
		uiError = ((uiRSErrBit * 10)  / 2400);
		uiRef = 90;
		if(uiError > 10) uiError = 10;
	}

	pInfo->ucVber = 100 - (uiError + uiRef);
	pInfo->dPostBER = (LGD_DOUBLE32)uiRSErrTS / TS_ERR_THRESHOLD;

	return pInfo->dPostBER;
}

/* LGE MC Add */
LGD_UINT16 LGD_GET_TPERRCNT(LGD_UINT8 ucI2CID)
{
	LGD_UINT16 uiRSErrTS;
	
	uiRSErrTS 	= LGD_CMD_READ(ucI2CID, APB_RS_BASE+ 0x03);

	return uiRSErrTS;
}

LGD_UINT8 LGD_GET_SNR(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiFftVal;
	ST_BBPINFO* pInfo;
	pInfo = LGD_GET_STRINFO(ucI2CID);
	uiFftVal = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x07);
	
	if(uiFftVal == 0)		pInfo->ucSnr = 0;
	else if(uiFftVal < 11)	pInfo->ucSnr = 20;
	else if(uiFftVal < 15)	pInfo->ucSnr = 19;
	else if(uiFftVal < 20)	pInfo->ucSnr = 18;
	else if(uiFftVal < 30)	pInfo->ucSnr = 17;
	else if(uiFftVal < 45)	pInfo->ucSnr = 16;
	else if(uiFftVal < 60)	pInfo->ucSnr = 15;
	else if(uiFftVal < 75)	pInfo->ucSnr = 14;
	else if(uiFftVal < 90)	pInfo->ucSnr = 13;
	else if(uiFftVal < 110)	pInfo->ucSnr = 12;
	else if(uiFftVal < 130) pInfo->ucSnr = 11;
	else if(uiFftVal < 150) pInfo->ucSnr = 10;
	else if(uiFftVal < 200) pInfo->ucSnr = 9;
	else if(uiFftVal < 300) pInfo->ucSnr = 8;
	else if(uiFftVal < 400) pInfo->ucSnr = 7;
	else if(uiFftVal < 500) pInfo->ucSnr = 6;
	else if(uiFftVal < 600) pInfo->ucSnr = 5;
	else if(uiFftVal < 700) pInfo->ucSnr = 4;
	else if(uiFftVal < 800) pInfo->ucSnr = 3;
	else if(uiFftVal < 900) pInfo->ucSnr = 2;
	else if(uiFftVal < 1000) pInfo->ucSnr = 1;
	else pInfo->ucSnr = 0;

	return pInfo->ucSnr;
}

LGD_INT16 LGD_GET_RSSI(LGD_UINT8 ucI2CID)
{
	LGD_INT16 nLoop, nRSSI = 0;
	LGD_INT16 nResolution, nGapVal;
	LGD_UINT16 uiAdcValue;
	LGD_UINT16 aRFAGCTable[LGD_ADC_MAX][3] = 
	{
		{105,	1010, 1200},
		{100,	950, 	1010},
		{95,	908,	950},
		{90,	868,	908},
		{85, 	830, 	868},

		{80,	795, 	830},
		{75,	770, 	795},
		{70, 	750, 	770},
		{65,	730, 	750},
		{60, 	710, 	730},

		{55,	685, 	710},
		{50,	663, 	685},
		{45,	635, 	663},
		{40,	595, 	635},
		{35, 	540, 	595},

		{30,	425, 	540},
		{25,	220, 	425},
		{20,	128, 	220},
	};
	
	uiAdcValue  = (LGD_CMD_READ(ucI2CID, APB_RF_BASE+ 0x74) >> 5) & 0x3FF;
	//uiAdcValue  = (LGD_INT16)(1.17302 * (LGD_DOUBLE32)uiAdcValue );
	uiAdcValue  = (LGD_INT16)((1.17302*100000) * (LGD_DOUBLE32)uiAdcValue )/100000;		//inb612 double for android compile
	
	if(!uiAdcValue) nRSSI = 0;
	else if(uiAdcValue >= aRFAGCTable[0][1]) nRSSI = (LGD_INT16)aRFAGCTable[0][0];
	else if(uiAdcValue <= aRFAGCTable[LGD_ADC_MAX-1][1]) 	nRSSI = (LGD_INT16)aRFAGCTable[LGD_ADC_MAX-1][0];
	else 
	{
		for(nLoop = 1; nLoop < LGD_ADC_MAX; nLoop++)
		{
			if(uiAdcValue >= aRFAGCTable[nLoop][2] || uiAdcValue < aRFAGCTable[nLoop][1]) continue;	
			
			nResolution = (aRFAGCTable[nLoop][2] - aRFAGCTable[nLoop][1]) / 5;
			nGapVal = uiAdcValue - aRFAGCTable[nLoop][1];
			nRSSI = (aRFAGCTable[nLoop][0]) + (nGapVal/nResolution);
			break;
		}
	}

	return nRSSI;
}

#ifdef LGD_TII_TEST_ENABLE
LGD_INT16 LGD_TII_START(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiStatus = 0;

	uiStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0xB0) & (~0xC000);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xB0, uiStatus | 0xC000);

	uiStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x80);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x80, uiStatus | 0x4);

	return LGD_SUCCESS;
}

LGD_INT16 LGD_TII_STOP(LGD_UINT8 ucI2CID)
{
	LGD_UINT16	uiStatus = 0;

	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0xB0, 0x8320);

	uiStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x80);
	LGD_CMD_WRITE(ucI2CID, APB_PHY_BASE+ 0x80, uiStatus & (~0x0004));

	return LGD_SUCCESS;
}

LGD_INT16 LGD_TII_GET_INFO(LGD_UINT8 ucI2CID, ST_TII_INFO* pstTIIInfo)
{
	LGD_UINT16 uiStatus = 0;
	LGD_UINT16 nIndex = 0;

	for(nIndex=0; nIndex<MAX_TII_CNT; nIndex++){
		uiStatus = LGD_CMD_READ(ucI2CID, APB_PHY_BASE+ 0x2D + nIndex);

		if(uiStatus == 0x1F7F) continue;

		pstTIIInfo[nIndex].uiStrength = (uiStatus >> 13) & 0x7; 
		pstTIIInfo[nIndex].uiSubID = (uiStatus >> 8) & 0x1F;
		pstTIIInfo[nIndex].uiZeroPad = (uiStatus >> 7) & 0x1;
		pstTIIInfo[nIndex].uiPattern = uiStatus & 0x7F ;

#if 0
		TRACE("\r\n ==> TII[%d] : 0x%X [Strength(%d), SubID(0x%X), ZeroPad(%d), Pattern(0x%X)] ", 
			nIndex, uiStatus, 
			pstTIIInfo[nIndex].uiStrength, 
			pstTIIInfo[nIndex].uiSubID, 
			pstTIIInfo[nIndex].uiZeroPad, 
			pstTIIInfo[nIndex].uiPattern);
#endif			
	}

	return LGD_SUCCESS;
}
#endif

