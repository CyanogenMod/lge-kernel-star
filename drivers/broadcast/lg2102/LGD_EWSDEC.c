#include <linux/broadcast/broadcast_lg2102_includes.h>

// Emergency. Warning. Systems
#ifdef LGD_EWS_SOURCE_ENABLE  

extern ST_TRANSMISSION		m_ucTransMode;
/***********************************************************************
LGD_EWS_INIT()함수는 반드시 LGD_CHANNEL_START()함수 호출전에 호출되어야 한다.                                                     
LGD_CHANNEL_START()에 리턴값이  LGD_SUCCESS이면48ms 주기로 LGD_EWS_FRAMECHECK() 
함수를 호출한다. LGD_EWS_FRAMECHECK() == LGD_SUCCESS 이면 g_stEWSMsg변수에 
값을 복사하고, LGD_EWS_INIT()함수를 호출한다.                                 

typedef struct _tagST_OUTPUT_EWS
{
	LGD_INT16		nNextSeqNo;			Next Segment No 
	LGD_INT16		nTotalSeqNo;		Total Segment No
	LGD_UINT16		nDataPos;			data posion
	LGD_UINT8		ucEWSStartEn;		EWS starting flag
	LGD_UINT8		ucIsEWSGood;		EWS Parsing flag

	LGD_UINT8		ucMsgGovernment;	메시지 발령기관
	LGD_UINT8		ucMsgID;			메시지 고유번호
	ST_DATE_T		stDate;				일시
	
	LGD_INT8		acKinds[4];			재난 종류
	LGD_UINT8		cPrecedence;		우선 순위
	LGD_UINT32		ulTime;				재난 시간
	LGD_UINT8		ucForm;				재난 지역형식
	LGD_UINT8		ucResionCnt;		재난 지역수
	LGD_UINT8		aucResionCode[11];	지역 코드
	LGD_DOUBLE32	fEWSCode;
	
	LGD_INT8		acOutputBuff[EWS_OUTPUT_BUFF_MAX];	재난 내용.
	
}ST_OUTPUT_EWS, *PST_OUTPUT_EWS;
***********************************************************************/



ST_OUTPUT_EWS	g_stEWSMsg;

ST_OUTPUT_EWS* LGD_GET_EWS_DB(void)
{
	return &g_stEWSMsg;
}

LGD_UINT32 YMDtoMJD(ST_DATE_T stDate)
{
	LGD_UINT16 wMJD;
	LGD_UINT32 lYear, lMouth, lDay, L;
	LGD_UINT32 lTemp1, lTemp2; 
	
	lYear = (LGD_UINT32)stDate.usYear - (LGD_UINT32)1900;
	lMouth = stDate.ucMonth;
	lDay = stDate.ucDay;
	
	if(lMouth == 1 || lMouth == 2) L = 1;
	else L = 0;
	
	lTemp1 = (lYear - L) * 36525L / 100L;
	lTemp2 = (lMouth + 1L + L * 12L) * 306001L / 10000L;
	
	wMJD = (LGD_UINT16)(14956 + lDay + lTemp1 + lTemp2);

	return wMJD;
}

void MJDtoYMD(LGD_UINT16 wMJD, ST_DATE_T *pstDate)
{
	LGD_UINT32 lYear, lMouth, lTemp;
	
	lYear = (wMJD * 100L - 1507820L) / 36525L;
	lMouth = ((wMJD * 10000L - 149561000L) - (lYear * 36525L / 100L) * 10000L) / 306001L;
	
	pstDate->ucDay = (LGD_UINT8)(wMJD - 14956L - (lYear * 36525L / 100L) - (lMouth * 306001L / 10000L));
	
	if(lMouth == 14 || lMouth == 15) lTemp = 1;
	else lTemp = 0;
	
	pstDate->usYear		= (LGD_UINT16)(lYear + lTemp + 1900);
	pstDate->ucMonth	= (LGD_UINT8)(lMouth - 1 - lTemp * 12);
}

void LGD_EWS_INIT(void)
{
	memset(&g_stEWSMsg, 0 , sizeof(ST_OUTPUT_EWS));
}

void LGD_TYPE5_EXTENSION2(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_5*		pType;
	ST_EWS_INFO*	pEwsInfo;
	ST_EWS_TIME*	pstEWSTime;

	LGD_UINT16		unData, nLoop;
	LGD_UINT32		ulData;
	LGD_UINT8		aucInfoBuff[5];

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType	= (ST_TYPE_5*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	if(pType->ITEM.bitD2 == 1)
	{
		unData = LGD_GET_WORDDATA(pFibInfo);
		pEwsInfo = (ST_EWS_INFO*)&unData;

		if(!pEwsInfo->ITEM.bitThisSeqNo)
		{
			LGD_EWS_INIT();

			for(nLoop = 0; nLoop < 3; nLoop++) g_stEWSMsg.acKinds[nLoop] = LGD_GET_BYTEDATA(pFibInfo);
			for(nLoop = 5; nLoop > 0; nLoop--) aucInfoBuff[nLoop-1] = LGD_GET_BYTEDATA(pFibInfo);
	
			ulData = ((aucInfoBuff[4]&0x3f)<<24) | (aucInfoBuff[3]<<16) | (aucInfoBuff[2]<<8) | aucInfoBuff[1];
			ulData = ulData >> 2;

			pstEWSTime = (ST_EWS_TIME*)&ulData;

			MJDtoYMD(pstEWSTime->ITEM.bitMJD, &g_stEWSMsg.stDate);
			g_stEWSMsg.stDate.ucHour	= (pstEWSTime->ITEM.bitUTCHours + 9) % 24;
			g_stEWSMsg.stDate.ucMinutes = pstEWSTime->ITEM.bitUTCMinutes;

			g_stEWSMsg.nTotalSeqNo		= pEwsInfo->ITEM.bitTotalNo;
			g_stEWSMsg.ucMsgGovernment	= pEwsInfo->ITEM.bitMsgGovernment;
			g_stEWSMsg.ucMsgID			= pEwsInfo->ITEM.bitID;
			g_stEWSMsg.cPrecedence		= ((aucInfoBuff[4] >> 6) & 0x3);
			g_stEWSMsg.ulTime			= ulData;

			g_stEWSMsg.ucForm			= (((aucInfoBuff[1] & 0x3)<<1) | (aucInfoBuff[0] >> 7));
			g_stEWSMsg.ucResionCnt		= ((aucInfoBuff[0]>>3) & 0xf);
			g_stEWSMsg.nNextSeqNo++;

			if(g_stEWSMsg.nTotalSeqNo)  g_stEWSMsg.ucEWSStartEn	= LGD_SUCCESS;

			for(nLoop = 0; nLoop < 10; nLoop++)
				g_stEWSMsg.aucResionCode[nLoop] = LGD_GET_BYTEDATA(pFibInfo);

			g_stEWSMsg.fEWSCode = atof((char*)g_stEWSMsg.aucResionCode);

			for( ; nLoop < (pHeader->ITEM.bitLength - 11); nLoop++)
				g_stEWSMsg.acOutputBuff[g_stEWSMsg.nDataPos++] = LGD_GET_BYTEDATA(pFibInfo);
		}
		else if(g_stEWSMsg.ucEWSStartEn == LGD_SUCCESS)
		{
			if(g_stEWSMsg.nNextSeqNo != pEwsInfo->ITEM.bitThisSeqNo){
				LGD_EWS_INIT();
				pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1);
				return;
			}

			g_stEWSMsg.nNextSeqNo = pEwsInfo->ITEM.bitThisSeqNo + 1;

			for(nLoop = 0; nLoop < (pHeader->ITEM.bitLength - 3); nLoop++)
				g_stEWSMsg.acOutputBuff[g_stEWSMsg.nDataPos++] = LGD_GET_BYTEDATA(pFibInfo);
			
			if(pEwsInfo->ITEM.bitThisSeqNo == g_stEWSMsg.nTotalSeqNo)
				g_stEWSMsg.ucIsEWSGood = LGD_SUCCESS;
		}
		else
			pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1);
	}
	else 
		pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1);
}

void LGD_SET_TYPE_5(ST_FIB_INFO* pFibInfo)
{
	ST_TYPE_5*		pExtern;
	ST_FIG_HEAD*	pHeader;
	LGD_UINT8		ucType, ucHeader;
	
	ucHeader = LGD_GETAT_HEADER(pFibInfo);
	ucType	= LGD_GETAT_TYPE(pFibInfo);
	
	pHeader = (ST_FIG_HEAD*)&ucHeader;
	pExtern = (ST_TYPE_5*)&ucType;
	
	switch(pExtern->ITEM.bitExtension){
		case EXTENSION_2: LGD_TYPE5_EXTENSION2(pFibInfo); break;
		default: pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1); break;
	}
}

LGD_UINT8 LGD_EWS_PARSING(LGD_UINT8* pucFicBuff, LGD_INT32 uFicLength)
{
	ST_FIB_INFO* 	pstFib;
	ST_FIG_HEAD* 	pHeader;
	ST_FIC			stEWS;
	LGD_UINT8		ucLoop, ucHeader, ucBlockNum;
    LGD_UINT16		uiTempIndex = 0;
	
	ucBlockNum = uFicLength / FIB_SIZE;
	pstFib = &stEWS.stBlock;
	
	for(ucLoop = 0; ucLoop < ucBlockNum; ucLoop++)
	{
		LGD_SET_UPDATEFIC(pstFib, &pucFicBuff[ucLoop*FIB_SIZE]);
		if(!pstFib->uiIsCRC) continue;
		
		while(pstFib->ucDataPos < FIB_SIZE-2)
		{
			ucHeader = LGD_GETAT_HEADER(pstFib);
			pHeader = (ST_FIG_HEAD*)&ucHeader;
			
			if(!LGD_GET_FINDTYPE(pHeader) || !LGD_GET_NULLBLOCK(pHeader) || !LGD_GET_FINDLENGTH(pHeader)) break;
			
			switch(pHeader->ITEM.bitType) {
			case FIG_FICDATA_CHANNEL : LGD_SET_TYPE_5(pstFib); break;
			default					 : pstFib->ucDataPos += pHeader->ITEM.bitLength + 1;break;
			}
		}
		if(g_stEWSMsg.ucIsEWSGood == LGD_SUCCESS)
			return LGD_SUCCESS;
	}

	return LGD_ERROR;
}

LGD_UINT8 LGD_EWS_FRAMECHECK(LGD_UINT8 ucI2CID)
{
	LGD_UINT16		wFicLen, uFIBCnt;
	LGD_UINT8		abyBuff[MAX_FIC_SIZE];
	
	uFIBCnt = LGD_GET_FIB_CNT(m_ucTransMode);
	
	if(!(LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x00) & 0x4000))
		return LGD_ERROR;
	
	wFicLen = LGD_CMD_READ(ucI2CID, APB_VTB_BASE+ 0x09);
	if(!wFicLen) return LGD_ERROR;
	wFicLen++;
	
	if(wFicLen != (uFIBCnt*FIB_SIZE)) return LGD_ERROR;
	LGD_CMD_READ_BURST(ucI2CID, APB_FIC_BASE, abyBuff, wFicLen);
	
	if(LGD_EWS_PARSING(abyBuff, wFicLen))
		return LGD_SUCCESS;

	return LGD_ERROR;
}


#endif
