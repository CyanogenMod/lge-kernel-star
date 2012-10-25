
//#include "StdAfx.h"
#include "../inc/INC_INCLUDES.h"

ST_FICDB_LIST		g_stFicDbList;

INC_UINT8 INC_GET_BYTEDATA(ST_FIB_INFO* pFibInfo)
{
	return pFibInfo->aucBuff[pFibInfo->ucDataPos++] & 0xff;
}

INC_UINT8 INC_GETAT_BYTEDATA(ST_FIB_INFO* pFibInfo)
{
	return pFibInfo->aucBuff[pFibInfo->ucDataPos] & 0xff;
}

INC_UINT16 INC_GET_WORDDATA(ST_FIB_INFO* pFibInfo)
{
	INC_UINT16 uiData;
	uiData = (((INC_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos++] << 8) & 0xff00);
	uiData |= ((INC_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos++] & 0xff);
	return (uiData & 0xffff);
}

INC_UINT16 INC_GETAT_WORDDATA(ST_FIB_INFO* pFibInfo)
{
	INC_UINT16 uiData;
	uiData = (((INC_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos] << 8) & 0xff00) |
		((INC_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos+1] & 0xff);
	return (uiData & 0xffff);
}

INC_UINT32 INC_GET_LONGDATA(ST_FIB_INFO* pFibInfo)
{
	INC_UINT32 ulMsb, ulLsb;

	ulMsb = (INC_UINT32)INC_GET_WORDDATA(pFibInfo);
	ulLsb = (INC_UINT32)INC_GET_WORDDATA(pFibInfo);
	return (ulMsb << 16 | ulLsb);
}

INC_UINT32 INC_GETAT_LONGDATA(ST_FIB_INFO* pFibInfo)
{
	INC_UINT32 ulMsb, ulLsb;
	ulMsb = (INC_UINT32)INC_GETAT_WORDDATA(pFibInfo);
	pFibInfo->ucDataPos += 2;
	ulLsb = (INC_UINT32)INC_GETAT_WORDDATA(pFibInfo);
	pFibInfo->ucDataPos -= 2;
	return (ulMsb << 16 | ulLsb);
}

INC_UINT8 INC_GETAT_HEADER(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos];
}

INC_UINT8 INC_GETAT_TYPE(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos+1];
}

INC_UINT8 INC_GET_NULL_BLOCK(ST_FIG_HEAD* pInfo)
{
	if(pInfo->ucInfo == END_MARKER) return INC_ERROR;
	return INC_SUCCESS;
}

INC_UINT8 INC_GET_FIND_LENGTH(ST_FIG_HEAD* pInfo)
{
	if(!pInfo->ITEM.bitLength || pInfo->ITEM.bitLength > FIB_SIZE-3) 
		return INC_ERROR;
	return INC_SUCCESS;
}

INC_UINT16 INC_CRC_CHECK(INC_UINT8 *pBuf, INC_UINT8 ucSize) 
{
	INC_UINT8	ucLoop;
	INC_UINT16	nCrc = 0xFFFF;

	for(ucLoop = 0; ucLoop < ucSize; ucLoop++){
		nCrc  = 0xFFFF & (((nCrc<<8) | (nCrc>>8)) ^ (0xFF & pBuf[ucLoop]));                        
		nCrc  = nCrc ^ ((0xFF & nCrc) >> 4);
		nCrc  = 0xFFFF & (nCrc ^ ( (((((0xFF & nCrc))<<8)|(((0xFF & nCrc))>>8))) << 4) ^ ((0xFF & nCrc) << 5));
	}
	return((INC_UINT16)0xFFFF & (nCrc ^ 0xFFFF));
}

INC_UINT16 INC_FIND_SUB_CHANNEL_SIZE(INC_UINT8 ucTableIndex) 
{
	INC_UINT16 wSubCHSize = 0;

	switch(ucTableIndex)
	{
	case 0:	wSubCHSize = 16; break;
	case 1: wSubCHSize = 21; break;
	case 2: wSubCHSize = 24; break;
	case 3: wSubCHSize = 29; break;
	case 4: wSubCHSize = 35; break;
	case 5: wSubCHSize = 24; break;
	case 6: wSubCHSize = 29; break;
	case 7: wSubCHSize = 35; break;
	case 8: wSubCHSize = 42; break;
	case 9: wSubCHSize = 52; break;
	case 10: wSubCHSize = 29; break;
	case 11: wSubCHSize = 35; break;
	case 12: wSubCHSize = 42; break;
	case 13: wSubCHSize = 52; break;
	case 14: wSubCHSize = 32; break;
	case 15: wSubCHSize = 42; break;  
	case 16: wSubCHSize = 48; break;
	case 17: wSubCHSize = 58; break;
	case 18: wSubCHSize = 70; break;
	case 19: wSubCHSize = 40; break;
	case 20: wSubCHSize = 52; break;
	case 21: wSubCHSize = 58; break;
	case 22: wSubCHSize = 70; break;
	case 23: wSubCHSize = 84; break;
	case 24: wSubCHSize = 48; break;
	case 25: wSubCHSize = 58; break;
	case 26: wSubCHSize = 70; break;
	case 27: wSubCHSize = 84; break;
	case 28: wSubCHSize = 104; break;
	case 29: wSubCHSize = 58; break;
	case 30: wSubCHSize = 70; break;
	case 31: wSubCHSize = 84; break;
	case 32: wSubCHSize = 104; break;
	case 33: wSubCHSize = 64; break;  
	case 34: wSubCHSize = 84; break;
	case 35: wSubCHSize = 96; break;
	case 36: wSubCHSize = 116; break;
	case 37: wSubCHSize = 140; break;
	case 38: wSubCHSize = 80; break;
	case 39: wSubCHSize = 104; break;
	case 40: wSubCHSize = 116; break;
	case 41: wSubCHSize = 140; break;
	case 42: wSubCHSize = 168; break;
	case 43: wSubCHSize = 96; break;
	case 44: wSubCHSize = 116; break;
	case 45: wSubCHSize = 140; break;
	case 46: wSubCHSize = 168; break;
	case 47: wSubCHSize = 208; break;
	case 48: wSubCHSize = 116; break;
	case 49: wSubCHSize = 140; break;
	case 50: wSubCHSize = 168; break;
	case 51: wSubCHSize = 208; break;  
	case 52: wSubCHSize = 232; break;
	case 53: wSubCHSize = 128; break;
	case 54: wSubCHSize = 168; break;
	case 55: wSubCHSize = 192; break;
	case 56: wSubCHSize = 232; break;
	case 57: wSubCHSize = 280; break;
	case 58: wSubCHSize = 160; break;
	case 59: wSubCHSize = 208; break;
	case 60: wSubCHSize = 280; break;
	case 61: wSubCHSize = 192; break;
	case 62: wSubCHSize = 280; break;
	case 63: wSubCHSize = 416; break;
	}
	return wSubCHSize;
}


INC_UINT8 INC_ADD_USERAPP_TYPE(INC_UINT32 ulSID, ST_STREAM_INFO* pStreamInfo, ST_USERAPP_GROUP_INFO* pstData)
{
	INC_INT16	nLoop;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;

	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSID){
			pSvcComponent->stUApp = *pstData;
			return INC_SUCCESS;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSID){
			pSvcComponent->stUApp = *pstData;
			return INC_SUCCESS;
		}
	}

	return INC_ERROR;
}

void INC_FIND_USERAPP_TYPE(INC_UINT32 ulSID, ST_USERAPP_GROUP_INFO* pstData)
{
	INC_INT16					nLoop;
	ST_FICDB_LIST*				pList;
	ST_STREAM_INFO*				pStreamInfo;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;

	pList = INC_GET_FICDB_LIST();
	pStreamInfo = &pList->stDMB;

	pSvcComponent = pList->stServicIDRef.astPrimary;
	for(nLoop = 0; nLoop < pList->stServicIDRef.nPrimaryCnt; nLoop++, pSvcComponent++)	{
		if(pSvcComponent->ulSid == ulSID)	{
			pSvcComponent->stUApp = *pstData;
			break;
		}
	}

	if(nLoop == pList->stServicIDRef.nPrimaryCnt){
		pSvcComponent = &pList->stServicIDRef.astPrimary[pList->stServicIDRef.nPrimaryCnt];
		pSvcComponent->ulSid		= ulSID;
		pSvcComponent->ucExtension	= 13;
		pSvcComponent->stUApp		= *pstData;
		pList->stServicIDRef.nPrimaryCnt++;
	}

	if(INC_ADD_USERAPP_TYPE(ulSID, pStreamInfo, pstData) == INC_SUCCESS){
		return;
	}

	pStreamInfo = &pList->stDATA;
	if(INC_ADD_USERAPP_TYPE(ulSID, pStreamInfo, pstData) == INC_SUCCESS){
		return;
	}
}

void INC_EXTENSION_000(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*		pstHeader;
	ST_TYPE_0*			pstType;
	ST_TYPE0of0_INFO*	pBitStarm;
	INC_UINT32			ulBitStram = 0;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType		= (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulBitStram = INC_GET_LONGDATA(pFibInfo);
	pBitStarm = (ST_TYPE0of0_INFO*)&ulBitStram;
	INC_ADD_ENSEMBLE_ID(pBitStarm);

	if(pBitStarm->ITEM.bitChangFlag) pFibInfo->ucDataPos += 1;
}

void INC_EXTENSION_001(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_TYPE0of1Short_INFO*	pShortInfo = INC_NULL;
	ST_TYPE0of1Long_INFO*	pLongInfo = INC_NULL;
	ST_FICDB_LIST*			pList;
	INC_UINT32				ulTypeInfo;
	INC_UINT16				uiStartAddr;
	INC_UINT8				ucSubChId, ucIndex, ucShortLongFlag;

	pList = INC_GET_FICDB_LIST();

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucIndex = 0; ucIndex < pstHeader->ITEM.bitLength-1;){
		ucShortLongFlag = (pFibInfo->aucBuff[pFibInfo->ucDataPos+2] >> 7) & 0x01;

		if(ucShortLongFlag == 0) {
			ulTypeInfo = INC_GET_LONGDATA(pFibInfo);
			pShortInfo = (ST_TYPE0of1Short_INFO*)&ulTypeInfo;
			pFibInfo->ucDataPos -= 1; 

			ucSubChId = pShortInfo->ITEM.bitSubChId;
			uiStartAddr = pShortInfo->ITEM.bitStartAddr;
		}
		else {	
			ulTypeInfo = INC_GET_LONGDATA(pFibInfo);
			pLongInfo = (ST_TYPE0of1Long_INFO*)&ulTypeInfo;

			ucSubChId = pLongInfo->ITEM.bitSubChId;
			uiStartAddr = pLongInfo->ITEM.bitStartAddr;
		}

		if(pList->ucIsSetSimple == SIMPLE_FIC_ENABLE)
			INC_FIND_SIMPLE_ORGANIZAION_SUBCHANNEL_ID(ucSubChId, uiStartAddr, ulTypeInfo);
		else 
			INC_FIND_ORGANIZAION_SUBCHANNEL_ID(ucSubChId, ulTypeInfo);

		ucIndex += (!ucShortLongFlag) ? 3 : 4;
	}

	if(pList->ucIsSimpleCnt)
		pList->ucIsSimpleFIC = 1;
}

void INC_EXTENSION_002(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_SERVICE_COMPONENT*	pService;

	INC_UINT32				ulServiceId;
	INC_UINT16				uiData;
	INC_UINT8				ucService, ucLoop, ucFrame;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucFrame = 0 ; ucFrame < pstHeader->ITEM.bitLength-1; )
	{
		if(pstType->ITEM.bitPD == 0) {
			ulServiceId = (INC_UINT32)INC_GET_WORDDATA(pFibInfo);
			ucFrame += 2;
		}
		else {
			ulServiceId = INC_GET_LONGDATA(pFibInfo);
			ucFrame += 4;
		}

		ucService = INC_GET_BYTEDATA(pFibInfo);
		ucFrame += 1;
		pService = (ST_SERVICE_COMPONENT*)&ucService;

		for(ucLoop = 0; ucLoop < pService->ITEM.bitNumComponent; ucLoop++, ucFrame+=2)
		{
			uiData = INC_GET_WORDDATA(pFibInfo);
			INC_FIND_BASIC_SERVICE(ulServiceId, uiData);
		}
	}
}

void INC_EXTENSION_003(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_TYPE0of3_INFO*		pTypeInfo;
	ST_TYPE0of3Id_INFO*		pIdInfo;

	INC_UINT32				uiId, uiTypeInfo;
	INC_UINT8				ucIndex;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucIndex = 0 ; ucIndex < pstHeader->ITEM.bitLength-1; )
	{
		uiTypeInfo = INC_GET_WORDDATA(pFibInfo);
		pTypeInfo = (ST_TYPE0of3_INFO*)&uiTypeInfo;

		uiId = INC_GET_WORDDATA(pFibInfo);
		uiId = (uiId << 16) | (INC_GET_BYTEDATA(pFibInfo)<<8);
		pIdInfo = (ST_TYPE0of3Id_INFO*)&uiId;

		INC_FIND_PACKET_MODE(pTypeInfo, pIdInfo);

		ucIndex += 5;

		if(pTypeInfo->ITEM.bitCAOrgFlag == 1){
			ucIndex += 2;
			pFibInfo->ucDataPos += 2;
		}
	}
}

void INC_EXTENSION_008(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pstHeader;
	ST_TYPE_0*		pstType;
	ST_MSC_BIT*		pstMsc;
	ST_MSC_SHORT*	pstMscShort;
	ST_MSC_LONG*	pstMscLong;

	INC_UINT16		uiMsgBit;
	INC_UINT32		ulSerId = 0;
	INC_UINT8		ucMscInfo, ucIndex, ucLSFlag;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType   = (ST_TYPE_0*)  &pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucIndex = 0 ; ucIndex < pstHeader->ITEM.bitLength-1; )
	{
		if(!pstType->ITEM.bitPD){
			ulSerId = (INC_UINT32)INC_GET_WORDDATA(pFibInfo);
			ucIndex += 2;
		}
		else {
			ulSerId = INC_GET_LONGDATA(pFibInfo);
			ucIndex += 4;
		}

		ucMscInfo = INC_GET_BYTEDATA(pFibInfo); 
		pstMsc = (ST_MSC_BIT*)&ucMscInfo;
		ucIndex += 1;
		ucLSFlag = (INC_GETAT_BYTEDATA(pFibInfo) >> 7) & 0x01;

		if(ucLSFlag) {
			uiMsgBit = INC_GET_WORDDATA(pFibInfo);
			ucIndex += 2;
			pstMscLong = (ST_MSC_LONG*)&uiMsgBit;

			INC_FIND_GLOBAL_SERVICE_COMPONENT_LONG(ulSerId, pstMscLong, pstMsc);
		}
		else{
			uiMsgBit = (INC_UINT16)INC_GET_BYTEDATA(pFibInfo);
			ucIndex += 1;
			pstMscShort = (ST_MSC_SHORT*)&uiMsgBit;

			INC_FIND_GLOBAL_SERVICE_COMPONENT_SHORT(ulSerId, pstMscShort, pstMsc);
		}

		if(pstMsc->ITEM.bitExtFlag) {
			ucIndex += 1;
			pFibInfo->ucDataPos += 1;
		}
	}
}

void INC_EXTENSION_010(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*		pstHeader;
	ST_TYPE_0*			pstType;
	ST_UTC_SHORT_INFO*	pstUTC_Short_Info;
	ST_UTC_LONG_INFO*	pstUTC_Long_Info;
	INC_UINT32			ulUtcInfo;
	INC_UINT16			uiUtcLongForm;
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType   = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulUtcInfo = INC_GET_LONGDATA(pFibInfo);
	pstUTC_Short_Info = (ST_UTC_SHORT_INFO*)&ulUtcInfo;

	MJDtoYMD(pstUTC_Short_Info->ITEM.bitMJD, &pList->stDate);
	pList->stDate.ucHour = (pstUTC_Short_Info->ITEM.bitHours + 9) % 24;
	pList->stDate.ucMinutes = pstUTC_Short_Info->ITEM.bitMinutes;

	if(pstUTC_Short_Info->ITEM.bitUTC_Flag){
		uiUtcLongForm = INC_GET_WORDDATA(pFibInfo);
		pstUTC_Long_Info = (ST_UTC_LONG_INFO*)&uiUtcLongForm;

		pList->stDate.ucSeconds			 = pstUTC_Long_Info->ITEM.bitSeconds;
		pList->stDate.uiMilliseconds = pstUTC_Long_Info->ITEM.bitMilliseconds;
	}
}

void INC_EXTENSION_013(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_USER_APP_IDnNUM*		pUserAppIdNum;
	ST_USER_APPTYPE*		pUserAppType;
	static ST_USERAPP_GROUP_INFO	stUserApp;

	INC_UINT32				ulSid;
	INC_UINT8				ucUserAppIdNum;
	INC_UINT8				ucFrame, ucIndex, ucDataCnt;
	INC_UINT16				uiUserAppType;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucFrame = 0 ; ucFrame < pstHeader->ITEM.bitLength-1; ){

		if(pstType->ITEM.bitPD){
			ulSid = INC_GET_LONGDATA(pFibInfo);
			ucFrame += 4;
		}
		else {
			ulSid = INC_GET_WORDDATA(pFibInfo);
			ucFrame += 2;
		}

		ucUserAppIdNum = INC_GET_BYTEDATA(pFibInfo);
		pUserAppIdNum = (ST_USER_APP_IDnNUM*)&ucUserAppIdNum;
		ucFrame += 1;

		stUserApp.ucUAppSCId  = pUserAppIdNum->ITEM.bitSCIdS;
		stUserApp.ucUAppCount = pUserAppIdNum->ITEM.bitNomUserApp;

		for(ucIndex = 0; ucIndex < stUserApp.ucUAppCount; ucIndex++, ucFrame+=2)
		{
			uiUserAppType = INC_GET_WORDDATA(pFibInfo);
			pUserAppType = (ST_USER_APPTYPE*)&uiUserAppType;

			stUserApp.astUserApp[ucIndex].ucDataLength  = pUserAppType->ITEM.bitUserDataLength;
			stUserApp.astUserApp[ucIndex].unUserAppType = pUserAppType->ITEM.bitUserAppType;

			for(ucDataCnt = 0; ucDataCnt < pUserAppType->ITEM.bitUserDataLength; ucDataCnt++){
				stUserApp.astUserApp[ucIndex].aucData[ucDataCnt] = INC_GET_BYTEDATA(pFibInfo);
				ucFrame += 1;
			}
			
			INC_FIND_USERAPP_TYPE(ulSid, &stUserApp);
		}
	}
}



void INC_EXTENSION_110(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_1*		pType;
	INC_UINT8		ucLoop;
	INC_UINT16		uiEId;
	INC_UINT8		acBuff[16];

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType	= (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	uiEId = INC_GET_WORDDATA(pFibInfo);

	for(ucLoop = 0; ucLoop < 16 ; ucLoop++){
		acBuff[ucLoop] = INC_GET_BYTEDATA(pFibInfo);
	}
	pFibInfo->ucDataPos += 2;
	INC_ADD_ENSEMBLE_NAME(uiEId, acBuff);
}

void INC_EXTENSION_111(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_1*		pType;
	INC_UINT16		uiSId, ucIndex;
	INC_UINT8		acBuff[16];

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	uiSId = INC_GET_WORDDATA(pFibInfo);
	for(ucIndex = 0 ; ucIndex < 16 ; ucIndex++) {
		acBuff[ucIndex] = INC_GET_BYTEDATA(pFibInfo);
	}
	pFibInfo->ucDataPos += 2;

	INC_FIND_DATA_SERVICE_COMPONENT_LABEL(uiSId, acBuff);
}

void INC_EXTENSION_112(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pHeader;
	ST_TYPE_1*				pType;
	ST_EXTENSION_TYPE12*	pType12;
	INC_UINT8				ucData;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ucData = INC_GET_BYTEDATA(pFibInfo);
	pType12 = (ST_EXTENSION_TYPE12*)&ucData;

	if(pType12->ITEM.bitCF_flag) pFibInfo->ucDataPos += 1;

	pFibInfo->ucDataPos += 19;
	if(pType12->ITEM.bitCountry == 1) pFibInfo->ucDataPos += 2;
}

void INC_EXTENSION_113(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pHeader;
	ST_TYPE_1*				pType;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pFibInfo->ucDataPos += 19;
}

void INC_EXTENSION_114(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pHeader;
	ST_TYPE_1*				pType;
	ST_EXTENSION_TYPE14*	pExtenType;
	INC_UINT32				ulSId;
	INC_UINT8				ucData, ucIndex;
	INC_UINT8				acBuff[16];

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ucData = INC_GET_BYTEDATA(pFibInfo);
	pExtenType = (ST_EXTENSION_TYPE14*)&ucData;

	if(!pExtenType->ITEM.bitPD) ulSId = (INC_UINT32)INC_GET_WORDDATA(pFibInfo); 
	else ulSId = INC_GET_LONGDATA(pFibInfo);

	for(ucIndex = 0 ; ucIndex < 16 ; ucIndex++) {
		acBuff[ucIndex] = INC_GET_BYTEDATA(pFibInfo);
	}
	pFibInfo->ucDataPos += 2;
	INC_FIND_SERVICE_COMPONENT_LABEL(ulSId, acBuff);
}

void INC_EXTENSION_115(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_1*		pType;
	INC_UINT32		ulSId;
	INC_UINT8		ucIndex, acBuff[16];

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulSId = INC_GET_LONGDATA(pFibInfo);
	for(ucIndex = 0; ucIndex < 16; ucIndex++) {
		acBuff[ucIndex] = INC_GET_BYTEDATA(pFibInfo);
	}

	pFibInfo->ucDataPos += 2;
	INC_FIND_DATA_SERVICE_COMPONENT_LABEL(ulSId, acBuff);
}

void INC_SET_UPDATEFIC(ST_FIB_INFO* pstDestData, INC_UINT8* pSourData)
{
	INC_UINT8	cuIndex;
	INC_UINT16	wCRC, wCRCData;
	for(cuIndex = 0; cuIndex < FIB_SIZE; cuIndex++) 
		pstDestData->aucBuff[cuIndex] = pSourData[cuIndex];

	wCRC = INC_CRC_CHECK((INC_UINT8*)pstDestData->aucBuff, FIB_SIZE-2);
	wCRCData = ((INC_UINT16)pstDestData->aucBuff[30] << 8) | pstDestData->aucBuff[31];

	pstDestData->ucDataPos = 0;
	pstDestData->uiIsCRC = wCRC == wCRCData;
}

INC_UINT8 INC_GET_FIND_TYPE(ST_FIG_HEAD* pInfo)
{
	if(pInfo->ITEM.bitType <= FIG_IN_HOUSE) return INC_SUCCESS;
	return INC_ERROR;
}

void INC_SET_FICTYPE_1(ST_FIB_INFO* pFibInfo)
{
	ST_TYPE_1*		pExtern;
	ST_FIG_HEAD*	pHeader;
	INC_UINT8		ucType, ucHeader;

	ucHeader = INC_GETAT_HEADER(pFibInfo);
	ucType = INC_GETAT_TYPE(pFibInfo);

	pHeader = (ST_FIG_HEAD*)&ucHeader;
	pExtern = (ST_TYPE_1*)&ucType;

	switch(pExtern->ITEM.bitExtension){
	case EXTENSION_0: INC_EXTENSION_110(pFibInfo); break;
	case EXTENSION_1: INC_EXTENSION_111(pFibInfo); break;
	case EXTENSION_2: INC_EXTENSION_112(pFibInfo); break;
	case EXTENSION_3: INC_EXTENSION_113(pFibInfo); break;
	case EXTENSION_4: INC_EXTENSION_114(pFibInfo); break;
	case EXTENSION_5: INC_EXTENSION_115(pFibInfo); break;
	default: pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1); break;
	}
}

INC_UINT16 INC_SET_FICTYPE_0(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_0*		pExtern;
	INC_UINT8		ucHeader, ucType;

	ucHeader = INC_GETAT_HEADER(pFibInfo);
	pHeader  = (ST_FIG_HEAD*)&ucHeader;

	ucType  = INC_GETAT_TYPE(pFibInfo);
	pExtern = (ST_TYPE_0*)&ucType;

	switch(pExtern->ITEM.bitExtension)
	{
	case EXTENSION_0	: INC_EXTENSION_000(pFibInfo); break;
	case EXTENSION_1	: INC_EXTENSION_001(pFibInfo); break;
	case EXTENSION_2	: INC_EXTENSION_002(pFibInfo); break;
	case EXTENSION_3	: INC_EXTENSION_003(pFibInfo); break;
	//case EXTENSION_8	: INC_EXTENSION_008(pFibInfo); break;
	case EXTENSION_10	: INC_EXTENSION_010(pFibInfo); break;
	case EXTENSION_13	: INC_EXTENSION_013(pFibInfo); break;
	default: pFibInfo->ucDataPos += pHeader->ITEM.bitLength + 1; break;
	}
	return INC_SUCCESS;
}

void INC_LABEL_FILTER(INC_UINT8* pData, INC_INT16 nSize)
{
	INC_INT16 nLoop;

	for(nLoop = nSize-1; nLoop >= 0; nLoop--){
		if(pData[nLoop] == 0x20) 
			pData[nLoop] = INC_NULL;
		else{
			//if(nLoop == nSize-1) 
				//pData[nLoop] = INC_NULL;
			break;
		}
	}
}

void INC_ADD_SERVICE_LABEL(ST_STREAM_INFO* pStreamInfo, INC_UINT32 ulSID, INC_UINT8* pcLabel)
{
	ST_FICDB_LIST*				pList;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	INC_INT16					nLoop;
	INC_UINT8					ucIsService = INC_ERROR;
	pList = INC_GET_FICDB_LIST();

	pSvcComponent = pList->stServicIDRef.astPrimary;
	for(nLoop = 0; nLoop < pList->stServicIDRef.nPrimaryCnt; nLoop++, pSvcComponent++)	{
		if(pSvcComponent->ulSid == ulSID)	{
			memcpy(pSvcComponent->aucLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucLabels, MAX_LABEL_CHAR);
			ucIsService = INC_SUCCESS;
			break;
		}
	}
	if(ucIsService == INC_ERROR){
		pSvcComponent = &pList->stServicIDRef.astPrimary[pList->stServicIDRef.nPrimaryCnt];
		pSvcComponent->ucExtension = 11;
		pSvcComponent->ulSid = ulSID;
		memcpy(pSvcComponent->aucLabels, pcLabel, MAX_LABEL_CHAR);
		INC_LABEL_FILTER(pSvcComponent->aucLabels, MAX_LABEL_CHAR);
		pList->stServicIDRef.nPrimaryCnt++;
	}

	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0 ; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSID && !pSvcComponent->ucIsLable){
			memcpy(pSvcComponent->aucLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucLabels, MAX_LABEL_CHAR);
			pSvcComponent->ucIsLable = 1;
			pList->nLabelCnt++;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0 ; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSID && !pSvcComponent->ucIsLable){
			memcpy(pSvcComponent->aucLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucLabels, MAX_LABEL_CHAR);
			pSvcComponent->ucIsLable = 1;
			pList->nLabelCnt++;
		}
	}
}

void INC_ADD_SERVICE_COMPONENT_LABEL(ST_STREAM_INFO* pStreamInfo, INC_UINT32 ulSID, INC_UINT8* pcLabel)
{
	ST_FICDB_LIST*				pList;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	INC_INT16					nLoop;
	pList = INC_GET_FICDB_LIST();

	pSvcComponent = pList->stServicIDRef.astPrimary;
	for(nLoop = 0; nLoop < pList->stServicIDRef.nPrimaryCnt;nLoop++, pSvcComponent++)	{
		if(pSvcComponent->ulSid == ulSID)	{
			memcpy(pSvcComponent->aucComponentLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucComponentLabels, MAX_LABEL_CHAR);
			break;
		}
	}
	if(nLoop == pList->stServicIDRef.nPrimaryCnt){
		pSvcComponent = &pList->stServicIDRef.astPrimary[pList->stServicIDRef.nPrimaryCnt];
		memcpy(pSvcComponent->aucComponentLabels, pcLabel, MAX_LABEL_CHAR);
		INC_LABEL_FILTER(pSvcComponent->aucComponentLabels, MAX_LABEL_CHAR);
		pList->stServicIDRef.nPrimaryCnt++;
	}

	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0 ; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++)	{
		if(pSvcComponent->ulSid == ulSID && !pSvcComponent->ucIsComponentLabel)	{
			memcpy(pSvcComponent->aucComponentLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucComponentLabels, MAX_LABEL_CHAR);
			pSvcComponent->ucIsComponentLabel = 1;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0 ; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSID && !pSvcComponent->ucIsComponentLabel){
			memcpy(pSvcComponent->aucComponentLabels, pcLabel, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucComponentLabels, MAX_LABEL_CHAR);
			pSvcComponent->ucIsComponentLabel = 1;
		}
	}
}

void INC_FIND_DATA_SERVICE_COMPONENT_LABEL(INC_UINT32 ulSID, INC_UINT8* pcLabel)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;
	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDAB;
	INC_ADD_SERVICE_LABEL(pStreamInfo, ulSID, pcLabel);

	pStreamInfo = &pList->stDMB;
	INC_ADD_SERVICE_LABEL(pStreamInfo, ulSID, pcLabel);

	//pStreamInfo = &pList->stFIDC;
	//INC_ADD_SERVICE_LABEL(pStreamInfo, ulSID, pcLabel);

	pStreamInfo = &pList->stDATA;
	INC_ADD_SERVICE_LABEL(pStreamInfo, ulSID, pcLabel);
}

void INC_FIND_SERVICE_COMPONENT_LABEL(INC_UINT32 ulSID, INC_UINT8* pcLabel)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;
	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDAB;
	INC_ADD_SERVICE_COMPONENT_LABEL(pStreamInfo, ulSID, pcLabel);

	pStreamInfo = &pList->stDMB;
	INC_ADD_SERVICE_COMPONENT_LABEL(pStreamInfo, ulSID, pcLabel);

	//pStreamInfo = &pList->stFIDC;
	//INC_ADD_SERVICE_COMPONENT_LABEL(pStreamInfo, ulSID, pcLabel);

	pStreamInfo = &pList->stDATA;
	INC_ADD_SERVICE_COMPONENT_LABEL(pStreamInfo, ulSID, pcLabel);
}

void INC_ADD_GLOBAL_SERVICE_COMPONENT_SHORT(ST_STREAM_INFO* pStreamInfo, INC_UINT32 ulSvcID, ST_MSC_SHORT* pstMscShort, ST_MSC_BIT* pstMsc)
{
	INC_INT16					nLoop;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();

	if(!pstMscShort->ITEM.bitMscFicFlag){

		pSvcComponent = pList->stServicIDRef.astPrimary;
		for(nLoop = 0; nLoop < pList->stServicIDRef.nPrimaryCnt; nLoop++, pSvcComponent++)	{
			if(pSvcComponent->ulSid == ulSvcID && pSvcComponent->ucSCidS == pstMsc->ITEM.bitScIds)	{
				pSvcComponent->ucSCidS = pstMsc->ITEM.bitScIds;
				break;
			}
		}
		if(nLoop == pList->stServicIDRef.nPrimaryCnt){
			pSvcComponent = &pList->stServicIDRef.astPrimary[pList->stServicIDRef.nPrimaryCnt];
			//pSvcComponent->ucExtension	= 18;
			pSvcComponent->ucSCidS		= pstMsc->ITEM.bitScIds;
			pList->stServicIDRef.nPrimaryCnt++;
		}

		pSvcComponent = pStreamInfo->astPrimary;
		for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
			if(pSvcComponent->ulSid == ulSvcID && pSvcComponent->ucSCidS == pstMsc->ITEM.bitScIds)	{
				pSvcComponent->ucSCidS = pstMsc->ITEM.bitScIds;
			}
		}

		pSvcComponent = pStreamInfo->astSecondary;
		for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++)	{
			if(pSvcComponent->ulSid == ulSvcID)	{
				pSvcComponent->ucSCidS = pstMsc->ITEM.bitScIds;
			}
		}
	}
}

void INC_FIND_GLOBAL_SERVICE_COMPONENT_SHORT(INC_UINT32 ulSvcID, ST_MSC_SHORT* pstMscShort, ST_MSC_BIT* pstMsc)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;
	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDMB;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_SHORT(pStreamInfo, ulSvcID, pstMscShort, pstMsc);
	pStreamInfo = &pList->stDAB;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_SHORT(pStreamInfo, ulSvcID, pstMscShort, pstMsc);
	pStreamInfo = &pList->stDATA;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_SHORT(pStreamInfo, ulSvcID, pstMscShort, pstMsc);
}

void INC_ADD_GLOBAL_SERVICE_COMPONENT_LONG(ST_STREAM_INFO* pStreamInfo, INC_UINT32 ulSvcID, ST_MSC_LONG* pstMscLong, ST_MSC_BIT* pstMsc)
{
	ST_FICDB_LIST*		pList;
	INC_INT16					nLoop;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	pList = INC_GET_FICDB_LIST();

	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSvcID && pSvcComponent->ucSCidS == pstMscLong->ITEM.bitScId)	{
			pSvcComponent->ucSubChid = pstMscLong->ITEM.bitScId;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ulSid == ulSvcID)	{
			pSvcComponent->ucSubChid = pstMscLong->ITEM.bitScId;
		}
	}
}

void INC_FIND_GLOBAL_SERVICE_COMPONENT_LONG(INC_UINT32 ulSvcID, ST_MSC_LONG* pstMscLong, ST_MSC_BIT* pstMsc)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;

	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDMB;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_LONG(pStreamInfo, ulSvcID, pstMscLong, pstMsc);
	pStreamInfo = &pList->stDAB;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_LONG(pStreamInfo, ulSvcID, pstMscLong, pstMsc);
	pStreamInfo = &pList->stDATA;
	INC_ADD_GLOBAL_SERVICE_COMPONENT_LONG(pStreamInfo, ulSvcID, pstMscLong, pstMsc);
}

void INC_FIND_PACKET_MODE(ST_TYPE0of3_INFO* pTypeInfo, ST_TYPE0of3Id_INFO* pIdInfo)
{
	ST_FICDB_LIST*				pList;
	ST_STREAM_INFO*				pStreamInfo;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	INC_INT16					nLoop;

	pList = INC_GET_FICDB_LIST();
	pStreamInfo = &pList->stSubChRef;
	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSCidS == pTypeInfo->ITEM.bitScid) {
			pSvcComponent->ucSCidS		= pTypeInfo->ITEM.bitScid;
			pSvcComponent->ucDSCType	= pIdInfo->ITEM.bitDScType;
			pSvcComponent->ucSubChid	= pIdInfo->ITEM.bitSubChId;
			pSvcComponent->unPacketAddr = pIdInfo->ITEM.bitPacketAddr;
			break;
		}
	}
	
	if(nLoop == pStreamInfo->nPrimaryCnt){
		pSvcComponent = &pStreamInfo->astPrimary[pStreamInfo->nPrimaryCnt];
		pSvcComponent->ucExtension  = 3;
		pSvcComponent->ucSCidS		= pTypeInfo->ITEM.bitScid;
		pSvcComponent->ucDSCType	= pIdInfo->ITEM.bitDScType;
		pSvcComponent->ucSubChid	= pIdInfo->ITEM.bitSubChId;
		pSvcComponent->unPacketAddr = pIdInfo->ITEM.bitPacketAddr;
		pStreamInfo->nPrimaryCnt++;
	}


	pStreamInfo = &pList->stDATA;
	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSCidS == pTypeInfo->ITEM.bitScid && !pSvcComponent->IsPacketMode)	{
			pSvcComponent->ucDSCType	= pIdInfo->ITEM.bitDScType;
			pSvcComponent->ucSubChid	= pIdInfo->ITEM.bitSubChId;
			pSvcComponent->unPacketAddr = pIdInfo->ITEM.bitPacketAddr;
			pSvcComponent->IsPacketMode = 1;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSCidS == pTypeInfo->ITEM.bitScid && !pSvcComponent->IsPacketMode)	{
			pSvcComponent->ucDSCType	= pIdInfo->ITEM.bitDScType;
			pSvcComponent->ucSubChid	= pIdInfo->ITEM.bitSubChId;
			pSvcComponent->unPacketAddr = pIdInfo->ITEM.bitPacketAddr;
			pSvcComponent->IsPacketMode = 1;
		}
	}
}


INC_UINT16 INC_GET_BITRATE(ST_FICDB_SERVICE_COMPONENT* pSvcComponent)
{
	INC_UINT16 uiBitRate = 0;

	if(!pSvcComponent->ucShortLong){
		if(pSvcComponent->ucTableIndex <= 4)  uiBitRate = 32;
		else if(pSvcComponent->ucTableIndex >= 5  && pSvcComponent->ucTableIndex <= 9)  uiBitRate = 48;
		else if(pSvcComponent->ucTableIndex >= 10 && pSvcComponent->ucTableIndex <= 13) uiBitRate = 56;
		else if(pSvcComponent->ucTableIndex >= 14 && pSvcComponent->ucTableIndex <= 18) uiBitRate = 64;
		else if(pSvcComponent->ucTableIndex >= 19 && pSvcComponent->ucTableIndex <= 23) uiBitRate = 80;
		else if(pSvcComponent->ucTableIndex >= 24 && pSvcComponent->ucTableIndex <= 28) uiBitRate = 96;
		else if(pSvcComponent->ucTableIndex >= 29 && pSvcComponent->ucTableIndex <= 32) uiBitRate = 112;
		else if(pSvcComponent->ucTableIndex >= 33 && pSvcComponent->ucTableIndex <= 37) uiBitRate = 128;
		else if(pSvcComponent->ucTableIndex >= 38 && pSvcComponent->ucTableIndex <= 42) uiBitRate = 160;
		else if(pSvcComponent->ucTableIndex >= 43 && pSvcComponent->ucTableIndex <= 47) uiBitRate = 192;
		else if(pSvcComponent->ucTableIndex >= 48 && pSvcComponent->ucTableIndex <= 52) uiBitRate = 224;
		else if(pSvcComponent->ucTableIndex >= 53 && pSvcComponent->ucTableIndex <= 57) uiBitRate = 256;
		else if(pSvcComponent->ucTableIndex >= 58 && pSvcComponent->ucTableIndex <= 60) uiBitRate = 320;
		else if(pSvcComponent->ucTableIndex >= 61 && pSvcComponent->ucTableIndex <= 63) uiBitRate = 384;
		else uiBitRate = 0;
	}
	else
	{
		if(pSvcComponent->ucOption == OPTION_INDICATE0) 
		{
			switch(pSvcComponent->ucProtectionLevel){
			case PROTECTION_LEVEL0 : uiBitRate = (pSvcComponent->uiSubChSize/12)*8;break;
			case PROTECTION_LEVEL1 : uiBitRate = (pSvcComponent->uiSubChSize/8)*8;break;
			case PROTECTION_LEVEL2 : uiBitRate = (pSvcComponent->uiSubChSize/6)*8;break;
			case PROTECTION_LEVEL3 : uiBitRate = (pSvcComponent->uiSubChSize/4)*8;break;
			}
		}
		else if(pSvcComponent->ucOption == OPTION_INDICATE1){
			switch(pSvcComponent->ucProtectionLevel){
			case PROTECTION_LEVEL0 : uiBitRate = (pSvcComponent->uiSubChSize/27)*32;break;
			case PROTECTION_LEVEL1 : uiBitRate = (pSvcComponent->uiSubChSize/21)*32;break;
			case PROTECTION_LEVEL2 : uiBitRate = (pSvcComponent->uiSubChSize/18)*32;break;
			case PROTECTION_LEVEL3 : uiBitRate = (pSvcComponent->uiSubChSize/15)*32;break;
			}
		}
	}
	return uiBitRate;
}

void INC_GET_SHORT_FORM(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, ST_TYPE0of1Short_INFO* pShort)
{
	pSvcComponent->ucShortLong 		= pShort->ITEM.bitShortLong;
	pSvcComponent->ucTableSW 		= pShort->ITEM.bitTableSw;
	pSvcComponent->ucTableIndex 	= pShort->ITEM.bitTableIndex;
	pSvcComponent->ucOption 		= 0;
	pSvcComponent->ucProtectionLevel= 0;
	pSvcComponent->uiSubChSize      = INC_FIND_SUB_CHANNEL_SIZE(pSvcComponent->ucTableIndex);
	pSvcComponent->uiDifferentRate  = 0;

	pSvcComponent->uiBitRate = INC_GET_BITRATE(pSvcComponent);
}

void INC_GET_LONG_FORM(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, ST_TYPE0of1Long_INFO* pLong)
{
	pSvcComponent->ucShortLong		= pLong->ITEM.bitShortLong;
	pSvcComponent->ucTableSW		= 0;
	pSvcComponent->ucTableIndex		= 0;
	pSvcComponent->ucOption			= pLong->ITEM.bitOption;
	pSvcComponent->ucProtectionLevel= pLong->ITEM.bitProtecLevel;
	pSvcComponent->uiSubChSize		= pLong->ITEM.bitSubChanSize;

	if(pSvcComponent->ucOption == 0){
		switch(pSvcComponent->ucProtectionLevel) {
		case  0 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/12); break;
		case  1 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/8); break;
		case  2 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/6); break;
		case  3 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/4); break;
		default : pSvcComponent->uiDifferentRate = 0; break; 
		}
	}
	else {
		switch(pSvcComponent->ucProtectionLevel) {
		case  0 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/27); break;
		case  1 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/21); break;
		case  2 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/18); break;
		case  3 : pSvcComponent->uiDifferentRate = (pSvcComponent->uiSubChSize/15); break;
		default : pSvcComponent->uiDifferentRate = 0; break; 
		}
	}
	pSvcComponent->uiBitRate = INC_GET_BITRATE(pSvcComponent);
}


void INC_ADD_ORGANIZAION_SUBCHANNEL_ID(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, INC_UINT32 ulTypeInfo)
{
	ST_TYPE0of1Long_INFO*	pLongInfo;
	ST_TYPE0of1Short_INFO*	pShortInfo;

	pShortInfo = (ST_TYPE0of1Short_INFO*)&ulTypeInfo;
	pLongInfo  = (ST_TYPE0of1Long_INFO*)&ulTypeInfo;

	pSvcComponent->unStartAddr = pShortInfo->ITEM.bitStartAddr;

	if(pShortInfo->ITEM.bitShortLong) INC_GET_LONG_FORM(pSvcComponent, pLongInfo);
	else INC_GET_SHORT_FORM(pSvcComponent, pShortInfo);
}

void INC_SORT_ORGANIZAION_SUBCHANNEL_ID(ST_STREAM_INFO* pStreamInfo, INC_UINT8 ucSubChID, INC_UINT32 ulTypeInfo)
{
	INC_INT16					nLoop;
	ST_FICDB_LIST*				pList;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	pList = INC_GET_FICDB_LIST();

	pSvcComponent = pList->stSubChRef.astPrimary;
	for(nLoop = 0; nLoop < pList->stSubChRef.nPrimaryCnt; nLoop++, pSvcComponent++)	{
		if(pSvcComponent->ucSubChid == ucSubChID)	{
			INC_ADD_ORGANIZAION_SUBCHANNEL_ID(pSvcComponent, ulTypeInfo);
			break;
		}
	}
	if(nLoop == pList->stSubChRef.nPrimaryCnt){
		pSvcComponent = &pList->stSubChRef.astPrimary[pList->stSubChRef.nPrimaryCnt];
		pSvcComponent->ucSubChid = ucSubChID;
		pSvcComponent->ucExtension = 1;
		INC_ADD_ORGANIZAION_SUBCHANNEL_ID(pSvcComponent, ulTypeInfo);
		pList->stSubChRef.nPrimaryCnt++;
	}


	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSubChid == ucSubChID && !pSvcComponent->IsOrganiza){
			INC_ADD_ORGANIZAION_SUBCHANNEL_ID(pSvcComponent, ulTypeInfo);
			pSvcComponent->IsOrganiza = INC_SUCCESS;
		}
	}

	pSvcComponent = pStreamInfo->astSecondary;
	for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSubChid == ucSubChID && !pSvcComponent->IsOrganiza)	{
			INC_ADD_ORGANIZAION_SUBCHANNEL_ID(pSvcComponent, ulTypeInfo);
			pSvcComponent->IsOrganiza = INC_SUCCESS;
		}
	}
}

INC_UINT8 INC_SORT_SIMPLE_ORGANIZAION_SUBCHANNEL_ID(ST_STREAM_INFO* pStreamInfo, INC_UINT8 ucSubChID, INC_UINT16 unStartAddr, INC_UINT32 ulTypeInfo)
{
	INC_INT16					nLoop;
	ST_FICDB_LIST*				pList;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	pList = INC_GET_FICDB_LIST();

	pSvcComponent = pStreamInfo->astPrimary;
	for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++, pSvcComponent++){
		if(pSvcComponent->ucSubChid == ucSubChID && pSvcComponent->unStartAddr == unStartAddr){
			return INC_ERROR;
		}
	}

	pSvcComponent = &pStreamInfo->astPrimary[pStreamInfo->nPrimaryCnt];
	pSvcComponent->ucSubChid   = ucSubChID;
	pSvcComponent->unStartAddr = unStartAddr;

	INC_ADD_ORGANIZAION_SUBCHANNEL_ID(pSvcComponent, ulTypeInfo);
	pStreamInfo->nPrimaryCnt++;
	pList->ucIsSimpleCnt++;
	return INC_SUCCESS;
}

void INC_FIND_SIMPLE_ORGANIZAION_SUBCHANNEL_ID(INC_UINT8 ucSubChID, INC_UINT16 unStartAddr, INC_UINT32 ulTypeInfo)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;
	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDMB;
	INC_SORT_SIMPLE_ORGANIZAION_SUBCHANNEL_ID(pStreamInfo, ucSubChID, unStartAddr, ulTypeInfo);
}

void INC_FIND_ORGANIZAION_SUBCHANNEL_ID(INC_UINT8 ucSubChID, INC_UINT32 ulTypeInfo)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;
	pList = INC_GET_FICDB_LIST();

	pStreamInfo = &pList->stDMB;
	INC_SORT_ORGANIZAION_SUBCHANNEL_ID(pStreamInfo, ucSubChID, ulTypeInfo);

	pStreamInfo = &pList->stDAB;
	INC_SORT_ORGANIZAION_SUBCHANNEL_ID(pStreamInfo, ucSubChID, ulTypeInfo);

	pStreamInfo = &pList->stDATA;
	INC_SORT_ORGANIZAION_SUBCHANNEL_ID(pStreamInfo, ucSubChID, ulTypeInfo);
}

void INC_ADD_BASIC_SERVICE(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, INC_UINT32 ulServiceId, INC_UINT16 unData)
{
	ST_TMId_MSCnFIDC*		pMscnFidc;
	ST_MSC_PACKET_INFO*		pMscPacket;

	pMscnFidc  = (ST_TMId_MSCnFIDC*)&unData;
	pMscPacket = (ST_MSC_PACKET_INFO*)&unData;

	pSvcComponent->ulSid	= ulServiceId;
	pSvcComponent->ucTmID	= pMscnFidc->ITEM.bitTMId;
	pSvcComponent->ucCAFlag = pMscnFidc->ITEM.bitCAflag;
	pSvcComponent->ucPS		= pMscnFidc->ITEM.bitPS;

	if(pMscnFidc->ITEM.bitTMId == 0x03){
		pSvcComponent->ucSCidS = pMscPacket->ITEM.bitSCId;
	}
	else{
		pSvcComponent->ucSubChid = pMscnFidc->ITEM.bitSubChld;
		pSvcComponent->ucDSCType = pMscnFidc->ITEM.bitAscDscTy;
	}
}

void INC_COPY_TO_LIST_SUBCH_ID(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, ST_FICDB_SERVICE_COMPONENT* pRcvComponent)
{
	if(pSvcComponent->ucSubChid == pRcvComponent->ucSubChid)
	{
		if(pRcvComponent->ucExtension == 3){
			pSvcComponent->ucSCidS		= pRcvComponent->ucSCidS;
			pSvcComponent->ucDSCType	= pRcvComponent->ucDSCType;
			pSvcComponent->ucSubChid	= pRcvComponent->ucSubChid;
			pSvcComponent->unPacketAddr = pRcvComponent->unPacketAddr;
		}
		if(pRcvComponent->ucExtension == 1){
			pSvcComponent->ucShortLong 		= pRcvComponent->ucShortLong;
			pSvcComponent->ucTableSW 		= pRcvComponent->ucTableSW;
			pSvcComponent->ucTableIndex 	= pRcvComponent->ucTableIndex;
			pSvcComponent->ucOption 		= pRcvComponent->ucOption;
			pSvcComponent->ucProtectionLevel= pRcvComponent->ucProtectionLevel;
			pSvcComponent->uiSubChSize      = pRcvComponent->uiSubChSize;
			pSvcComponent->uiDifferentRate  = pRcvComponent->uiDifferentRate;
			pSvcComponent->uiBitRate		= pRcvComponent->uiBitRate;
		}

		pRcvComponent->ucExtension = 0;
	}
}

void __INC_COPY_TO_LIST_SUBCH_ID(ST_STREAM_INFO* pStreamInfo)
{
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	ST_FICDB_SERVICE_COMPONENT* pRcvComponent;
	INC_INT16			nIndex, nLoop;
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();

	for(nIndex = 0; nIndex < pList->stSubChRef.nPrimaryCnt; nIndex++){
		pRcvComponent = &pList->stSubChRef.astPrimary[nIndex];
		for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++) {
			pSvcComponent = &pStreamInfo->astPrimary[nLoop];
			INC_COPY_TO_LIST_SUBCH_ID(pSvcComponent, pRcvComponent);
		}

		pRcvComponent = &pList->stSubChRef.astPrimary[nIndex];
		for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++)	{
			pSvcComponent = &pStreamInfo->astSecondary[nLoop];
			INC_COPY_TO_LIST_SUBCH_ID(pSvcComponent, pRcvComponent);
		}
	}
}

void INC_COPY_TO_LIST_SERVICE_ID(ST_FICDB_SERVICE_COMPONENT* pSvcComponent, ST_FICDB_SERVICE_COMPONENT* pRcvComponent)
{
	if(pSvcComponent->ulSid == pRcvComponent->ulSid)
	{
		if(pRcvComponent->ucExtension == 18){
			//pSvcComponent->ucSCidS = pRcvComponent->ucSCidS;
		}
		if(pRcvComponent->ucExtension == 8){
			//pSvcComponent->ucSubChid = pRcvComponent->ucSubChid;
		}
		if(pRcvComponent->ucExtension == 11)	{
			memcpy(pSvcComponent->aucLabels, pRcvComponent->aucLabels, MAX_LABEL_CHAR);
			INC_LABEL_FILTER(pSvcComponent->aucLabels, MAX_LABEL_CHAR);
		}
		if(pRcvComponent->ucExtension == 13)
		{
			pSvcComponent->stUApp = pRcvComponent->stUApp;
		}
		pRcvComponent->ucExtension = 0;
	}
}

void __INC_COPY_TO_LIST_SERVICE_ID(ST_STREAM_INFO* pStreamInfo)
{
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	ST_FICDB_SERVICE_COMPONENT* pRcvComponent;
	INC_INT16			nIndex, nLoop;
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();


	for(nIndex = 0; nIndex < pList->stServicIDRef.nPrimaryCnt; nIndex++){
		pRcvComponent = &pList->stServicIDRef.astPrimary[nIndex];
		for(nLoop = 0; nLoop < pStreamInfo->nPrimaryCnt; nLoop++) {
			pSvcComponent = &pStreamInfo->astPrimary[nLoop];
	
			INC_COPY_TO_LIST_SERVICE_ID(pSvcComponent, pRcvComponent);
		}

		pRcvComponent = &pList->stServicIDRef.astPrimary[nIndex];
		for(nLoop = 0; nLoop < pStreamInfo->nSecondaryCnt; nLoop++)	{
			pSvcComponent = &pStreamInfo->astSecondary[nLoop];
			INC_COPY_TO_LIST_SERVICE_ID(pSvcComponent, pRcvComponent);
		}
	}
}

void INC_ADD_ENSEMBLE_LIST(void)
{
	ST_FICDB_LIST*		pList;
	ST_STREAM_INFO*		pStreamInfo;

	pList = INC_GET_FICDB_LIST();
	
	pStreamInfo	= &pList->stDAB;
	__INC_COPY_TO_LIST_SUBCH_ID(pStreamInfo);

	pStreamInfo	= &pList->stDMB;
	__INC_COPY_TO_LIST_SUBCH_ID(pStreamInfo);

	pStreamInfo	= &pList->stDATA;
	__INC_COPY_TO_LIST_SUBCH_ID(pStreamInfo);

	pStreamInfo	= &pList->stDAB;
	__INC_COPY_TO_LIST_SERVICE_ID(pStreamInfo);

	pStreamInfo	= &pList->stDMB;
	__INC_COPY_TO_LIST_SERVICE_ID(pStreamInfo);

	pStreamInfo	= &pList->stDATA;
	__INC_COPY_TO_LIST_SERVICE_ID(pStreamInfo);
}

void INC_FIND_BASIC_SERVICE(INC_UINT32 ulServiceId, INC_UINT16 unData)
{
	INC_INT16					nLoop, nDataCnt;
	ST_FICDB_LIST*				pList;
	ST_STREAM_INFO*				pStreamInfo;
	ST_FICDB_SERVICE_COMPONENT* pSvcComponent;
	ST_TMId_MSCnFIDC*			pMscnFidc;
	ST_MSC_PACKET_INFO*			pMscPacket;

	pList = INC_GET_FICDB_LIST();

	if(pList->ucIsSetSimple == SIMPLE_FIC_ENABLE) return;

	pMscnFidc  = (ST_TMId_MSCnFIDC*)&unData;
	pMscPacket = (ST_MSC_PACKET_INFO*)&unData;

	if(pMscnFidc->ITEM.bitTMId == 0x00)			pStreamInfo = &pList->stDAB;
	else if(pMscnFidc->ITEM.bitTMId == 0x01)	pStreamInfo = &pList->stDMB;
	else if(pMscnFidc->ITEM.bitTMId == 0x02)	pStreamInfo = &pList->stFIDC;
	else										pStreamInfo = &pList->stDATA;

	pSvcComponent	= (pMscnFidc->ITEM.bitPS) ? pStreamInfo->astPrimary : pStreamInfo->astSecondary;
	nDataCnt		= (pMscnFidc->ITEM.bitPS) ? pStreamInfo->nPrimaryCnt: pStreamInfo->nSecondaryCnt;

	for(nLoop = 0; nLoop < nDataCnt; nLoop++, pSvcComponent++) {
		if(pSvcComponent->ulSid == ulServiceId){
			INC_ADD_ENSEMBLE_LIST();
			return;
		}
	}

	if(pMscnFidc->ITEM.bitTMId != 0x02){

		INC_ADD_BASIC_SERVICE(pSvcComponent, ulServiceId, unData);

		if(pMscnFidc->ITEM.bitPS) pStreamInfo->nPrimaryCnt++;
		else pStreamInfo->nSecondaryCnt++;	
	}
	INC_ADD_ENSEMBLE_LIST();
}

void INC_ADD_ENSEMBLE_ID(ST_TYPE0of0_INFO* pIdInfo)
{
	ST_FICDB_LIST*	pList;
	pList = INC_GET_FICDB_LIST();

	pList->unEnsembleID = pIdInfo->ITEM.bitEld;
	pList->ucChangeFlag = pIdInfo->ITEM.bitChangFlag;
}

INC_UINT8 INC_ADD_ENSEMBLE_NAME(INC_UINT16 unEnID, INC_UINT8* pcLabel)
{
	ST_FICDB_LIST*	pList;
	pList = INC_GET_FICDB_LIST();
	pList->unEnsembleID = unEnID;

	pList->nEmsembleLabelFlag = 1;

	memcpy(pList->aucEnsembleName, pcLabel, MAX_LABEL_CHAR);
	INC_LABEL_FILTER(pList->aucEnsembleName, MAX_LABEL_CHAR);

	return INC_SUCCESS;
}

ST_FICDB_LIST* INC_GET_FICDB_LIST(void)
{
	return &g_stFicDbList;
}

void INC_INIT_LIST(ST_FICDB_SERVICE_COMPONENT* pstPrimary, ST_FICDB_SERVICE_COMPONENT* pstSecondary)
{
	INC_INT16 nLoop;
	for(nLoop = 0 ; nLoop < INC_SERVICE_MAX; nLoop++, pstPrimary++, pstSecondary++){
		pstPrimary->ucSubChid 	= 0xFF;
		pstPrimary->ucTmID		= 0xFF;
		pstPrimary->ucSCidS		= 0xFF;

		pstSecondary->ucSubChid = 0xff;
		pstSecondary->ucTmID	= 0xFF;
		pstPrimary->ucSCidS		= 0xFF;
	}
}

void INC_INITDB(INC_UINT8 ucI2CID) 
{
	ST_FICDB_LIST* pList;
	ST_STREAM_INFO* pStrInfo;
	ST_FICDB_SERVICE_COMPONENT* pstPrimary;
	ST_FICDB_SERVICE_COMPONENT* pstSecondary;

	pList = INC_GET_FICDB_LIST();
	if(pList == 0) return;


	memset(pList, 0 , sizeof(ST_FICDB_LIST));

	pStrInfo = &pList->stDMB;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

	pStrInfo = &pList->stDAB;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

	pStrInfo = &pList->stDATA;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

	pStrInfo = &pList->stFIDC;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

	pStrInfo = &pList->stServicIDRef;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

	pStrInfo = &pList->stSubChRef;
	pstPrimary = pStrInfo->astPrimary;
	pstSecondary = pStrInfo->astSecondary;
	INC_INIT_LIST(pstPrimary, pstSecondary);

}

INC_INT16 INC_GET_LABEL_CNT(ST_STREAM_INFO* pStrInfo)
{
	INC_INT16 nLoop, nLabelCnt = 0;
	ST_FICDB_SERVICE_COMPONENT* pstPrimary;
	ST_FICDB_SERVICE_COMPONENT* pstSecondary;

	pstPrimary 		= pStrInfo->astPrimary;
	pstSecondary 	= pStrInfo->astSecondary;

	for(nLoop = 0 ; nLoop < pStrInfo->nPrimaryCnt; nLoop++, pstPrimary++){
		if((INC_INT16)strlen((char*)pstPrimary->aucLabels)){
			if(pstPrimary->ucTmID != 2 && pstPrimary->ucTmID != 0xff)
				nLabelCnt++;
		}
	}
	return nLabelCnt;
}

INC_INT16 INC_TOTAL_LABEL_CNT(void)
{
	INC_INT16 nLabelCnt = 0;
	ST_FICDB_LIST* pList;
	ST_STREAM_INFO* pStrInfo;

	pList    = INC_GET_FICDB_LIST();
	pStrInfo = &pList->stDMB;
	nLabelCnt += INC_GET_LABEL_CNT(pStrInfo);

	pStrInfo = &pList->stDAB;
	nLabelCnt += INC_GET_LABEL_CNT(pStrInfo);

	pStrInfo = &pList->stDATA;
	nLabelCnt += INC_GET_LABEL_CNT(pStrInfo);

	return nLabelCnt;
}

INC_UINT8 INC_CHECK_ENSEMBLE_DB(INC_INT16 nPCnt, INC_INT16 nSCnt,ST_FICDB_SERVICE_COMPONENT* pP, ST_FICDB_SERVICE_COMPONENT* pS)
{
	INC_INT16 nIndex;
	ST_FICDB_LIST*	pList;
	pList = INC_GET_FICDB_LIST();

	for(nIndex = 0 ; nIndex < nPCnt; nIndex++, pP++){
		if(pP->ucTmID != 2)	{
			if(pP->uiBitRate == 0 || (INC_INT16)strlen((char*)pP->aucLabels) == 0 || !pP->ulSid) {
				return INC_ERROR;
			}
		}

		if(pP->ucTmID == 1)	{
			if(pP->uiBitRate && pP->uiBitRate < 180){
				pList->nPacketCnt++;
				if(pP->stUApp.astUserApp[0].unUserAppType)
					pList->nUserAppCnt++;
			}
		}

		if(pP->ucTmID == 3){
			pList->nPacketCnt++;
			if(pP->stUApp.astUserApp[0].unUserAppType)
				pList->nUserAppCnt++;
		}
	}

	return INC_SUCCESS;
}

INC_UINT8 INC_CHECK_ENSEMBLE(void)
{
	ST_FICDB_LIST*	pList;
	ST_STREAM_INFO* pStreamInfo;
	ST_FICDB_SERVICE_COMPONENT* pP;
	ST_FICDB_SERVICE_COMPONENT* pS;
	INC_INT16 nLabelCnt;

	pList			= INC_GET_FICDB_LIST();
	nLabelCnt		= INC_TOTAL_LABEL_CNT();
	if(nLabelCnt == 0) return INC_ERROR;
	
	pList->nUserAppCnt = pList->nPacketCnt = 0;
	
	pStreamInfo		= &pList->stDMB;
	pP				= pStreamInfo->astPrimary;
	pS				= pStreamInfo->astSecondary;
	if(INC_CHECK_ENSEMBLE_DB(pStreamInfo->nPrimaryCnt, pStreamInfo->nSecondaryCnt, pP, pS) == INC_ERROR)
		return INC_ERROR;

	pStreamInfo		= &pList->stDAB;
	pP				= pStreamInfo->astPrimary;
	pS				= pStreamInfo->astSecondary;
	if(INC_CHECK_ENSEMBLE_DB(pStreamInfo->nPrimaryCnt, pStreamInfo->nSecondaryCnt, pP, pS) == INC_ERROR)
		return INC_ERROR;

	pStreamInfo		= &pList->stDATA;
	pP				= pStreamInfo->astPrimary;
	pS				= pStreamInfo->astSecondary;
	if(INC_CHECK_ENSEMBLE_DB(pStreamInfo->nPrimaryCnt, pStreamInfo->nSecondaryCnt, pP, pS) == INC_ERROR)
		return INC_ERROR;

	if(pList->nEmsembleLabelFlag)
	{
#ifdef INC_USER_APPLICATION_TYPE_ENABLE
		if((pList->stDAB.nPrimaryCnt+pList->stDMB.nPrimaryCnt+pList->stDATA.nPrimaryCnt)
			== nLabelCnt && pList->nUserAppCnt >= pList->nPacketCnt)
			return INC_SUCCESS;
#else
		if((pList->stDAB.nPrimaryCnt+pList->stDMB.nPrimaryCnt+pList->stDATA.nPrimaryCnt)
			== nLabelCnt)
			return INC_SUCCESS;
#endif
	}

	return INC_ERROR;
}

INC_UINT8 INC_UPDATA_FIC_LIST(void)
{
	INC_INT16 nIndex, nRcvData = 0;
	ST_FICDB_LIST*	pList;
	ST_STREAM_INFO* pStreamInfo;
	ST_FICDB_SERVICE_COMPONENT* pP;
	ST_FICDB_SERVICE_COMPONENT* pS = INC_NULL;
	static ST_STREAM_INFO	stTempChInfo;

	pList	= INC_GET_FICDB_LIST();
	pStreamInfo	= &pList->stDMB;
	memset(&stTempChInfo, 0, sizeof(stTempChInfo));
	for(nIndex = 0 ; nIndex < pStreamInfo->nPrimaryCnt; nIndex++){
		pP = &pStreamInfo->astPrimary[nIndex];
		if(INC_CHECK_ENSEMBLE_DB(1,1,pP,pS) == INC_SUCCESS){
			memcpy(&stTempChInfo.astPrimary[stTempChInfo.nPrimaryCnt], pP, sizeof(ST_FICDB_SERVICE_COMPONENT));
			stTempChInfo.nPrimaryCnt++;
		}
	}
	memcpy(pStreamInfo, &stTempChInfo, sizeof(ST_STREAM_INFO));
	
	pStreamInfo	= &pList->stDAB;
	memset(&stTempChInfo, 0, sizeof(stTempChInfo));
	for(nIndex = 0 ; nIndex < pStreamInfo->nPrimaryCnt; nIndex++){
		pP = &pStreamInfo->astPrimary[nIndex];
		if(INC_CHECK_ENSEMBLE_DB(1,1,pP,pS) == INC_SUCCESS){
			memcpy(&stTempChInfo.astPrimary[stTempChInfo.nPrimaryCnt], pP, sizeof(ST_FICDB_SERVICE_COMPONENT));
			stTempChInfo.nPrimaryCnt++;
		}
	}
	memcpy(pStreamInfo, &stTempChInfo, sizeof(ST_STREAM_INFO));

	pStreamInfo	= &pList->stDATA;
	memset(&stTempChInfo, 0, sizeof(stTempChInfo));
	for(nIndex = 0 ; nIndex < pStreamInfo->nPrimaryCnt; nIndex++){
		pP = &pStreamInfo->astPrimary[nIndex];
		if(INC_CHECK_ENSEMBLE_DB(1,1,pP,pS) == INC_SUCCESS){
			memcpy(&stTempChInfo.astPrimary[stTempChInfo.nPrimaryCnt], pP, sizeof(ST_FICDB_SERVICE_COMPONENT));
			stTempChInfo.nPrimaryCnt++;
		}
	}
	memcpy(pStreamInfo, &stTempChInfo, sizeof(ST_STREAM_INFO));

	pStreamInfo	= &pList->stDMB;
	nRcvData += pStreamInfo->nPrimaryCnt;
	pStreamInfo	= &pList->stDAB;
	nRcvData += pStreamInfo->nPrimaryCnt;
	pStreamInfo	= &pList->stDATA;
	nRcvData += pStreamInfo->nPrimaryCnt;

	if(nRcvData == 0) return INC_ERROR;
	return INC_SUCCESS;
}

INC_UINT8 INC_FICPARSING(INC_UINT8 ucI2CID, INC_UINT8* pucFicBuff, INC_INT32 uFicLength, ST_SIMPLE_FIC bSimpleFIC)
{
	ST_FIC 			stFIC;
	ST_FIB_INFO* 	pstFib;
	ST_FIG_HEAD* 	pHeader;
	ST_FICDB_LIST*	pList;
	INC_UINT8		ucLoop, ucHeader;

	pList = INC_GET_FICDB_LIST();
	stFIC.ucBlockNum = uFicLength/FIB_SIZE;
	pstFib = &stFIC.stBlock;

	pList->ucIsSetSimple = bSimpleFIC;

	for(ucLoop = 0; ucLoop < stFIC.ucBlockNum; ucLoop++)
	{
		INC_SET_UPDATEFIC(pstFib, &pucFicBuff[ucLoop*FIB_SIZE]);

		if(!pstFib->uiIsCRC) continue;

		while(pstFib->ucDataPos < FIB_SIZE-2)
		{
			ucHeader = INC_GETAT_HEADER(pstFib);
			pHeader = (ST_FIG_HEAD*)&ucHeader;

			if(!INC_GET_FIND_TYPE(pHeader) || 
				!INC_GET_NULL_BLOCK(pHeader) || 
				!INC_GET_FIND_LENGTH(pHeader)) break;

			switch(pHeader->ITEM.bitType) {
				case FIG_MCI_SI				: INC_SET_FICTYPE_0(pstFib);break;
				case FIG_LABEL				: INC_SET_FICTYPE_1(pstFib);break;
				case FIG_RESERVED_0			:
				case FIG_RESERVED_1			:
				case FIG_RESERVED_2			:
				case FIG_FICDATA_CHANNEL	:
				case FIG_CONDITION_ACCESS	:
				case FIG_IN_HOUSE			:
				default	: pstFib->ucDataPos += pHeader->ITEM.bitLength + 1;break;
			}

			if(pstFib->ucDataPos == (FIB_SIZE-1)) break;
		}
	}

	if(bSimpleFIC == SIMPLE_FIC_ENABLE && pList->ucIsSimpleFIC){
		return INC_SUCCESS;
	}

	return INC_CHECK_ENSEMBLE();

}
