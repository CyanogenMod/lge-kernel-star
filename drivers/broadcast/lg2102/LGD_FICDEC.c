#include <linux/broadcast/broadcast_lg2102_includes.h>


LGD_UINT8				g_bFIC_OK;
ST_FIC_DB				g_astFicDb;
ST_UTC_INFO				g_stUTC_Info;

LGD_UINT8 LGD_CHECK_SERVICE_DB16(LGD_UINT16* ptr, LGD_UINT16 wVal, LGD_UINT16 wNum)
{
  LGD_INT32 nLoop;
  LGD_UINT8 ucDbNum = 0;

  for(nLoop = 0; nLoop < wNum ; nLoop++) {
    if(ptr[nLoop] != wVal) ucDbNum++;
  }
  return ucDbNum;
}

LGD_UINT8 LGD_CHECK_SERVICE_DB8(LGD_UINT8* ptr, LGD_UINT8 cVal, LGD_UINT8 cNum) 
{
	LGD_UINT8 ucLoop, ucDbNum = 0;
	for(ucLoop = 0; ucLoop < cNum ; ucLoop++) {
	  if(ptr[ucLoop] != cVal)
      ucDbNum++;
	}
	return ucDbNum;
}

LGD_INT16 LGD_CHECK_SERVICE_CNT16(LGD_UINT16* ptr, LGD_UINT16 wVal, LGD_UINT8 cNum, LGD_UINT16 wMask) 
{
	LGD_INT16 cLoop;
	for(cLoop = 0; cLoop < cNum ; cLoop++) {
	  if((ptr[cLoop] & wMask) == (wVal & wMask))
      return cLoop;
	}
	return -1;
}

LGD_INT16 LGD_CHECK_SERVICE_CNT32(LGD_UINT32* ptr, LGD_UINT32 wVal, LGD_UINT8 cNum) 
{
	LGD_INT16 cLoop;
	for(cLoop = 0; cLoop < cNum ; cLoop++) {
		if(ptr[cLoop] == wVal)
			return cLoop;
	}
	return -1;
}


LGD_INT16 LGD_CHECK_SERVICE_CNT8(LGD_UINT8* ptr, LGD_UINT8 cVal, LGD_UINT8 cNum, LGD_UINT8 cMask) 
{
  LGD_INT16 cLoop;
  for(cLoop = 0; cLoop < cNum ; cLoop++) {
    if((*(ptr+cLoop) & cMask) == (cVal & cMask))
      return cLoop;
  }
  return -1;
}

LGD_UINT8 LGD_GET_BYTEDATA(ST_FIB_INFO* pFibInfo)
{
	return pFibInfo->aucBuff[pFibInfo->ucDataPos++] & 0xff;
}

LGD_UINT8 LGD_GETAT_BYTEDATA(ST_FIB_INFO* pFibInfo)
{
	return pFibInfo->aucBuff[pFibInfo->ucDataPos] & 0xff;
}

LGD_UINT16 LGD_GET_WORDDATA(ST_FIB_INFO* pFibInfo)
{
	LGD_UINT16 uiData;
	uiData = (((LGD_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos++] << 8) & 0xff00);
	uiData |= ((LGD_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos++] & 0x00ff);
	return (uiData & 0xffff);
}

LGD_UINT16 LGD_GETAT_WORDDATA(ST_FIB_INFO* pFibInfo)
{
	LGD_UINT16 uiData;
	uiData = (((LGD_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos] << 8) & 0xff00) |
		((LGD_UINT16)pFibInfo->aucBuff[pFibInfo->ucDataPos+1] & 0x00ff);
	return (uiData & 0xffff);
}

LGD_UINT32 LGD_GET_LONGDATA(ST_FIB_INFO* pFibInfo)
{
	LGD_UINT32 ulMsb, ulLsb;

	ulMsb = (LGD_UINT32)LGD_GET_WORDDATA(pFibInfo);
	ulLsb = (LGD_UINT32)LGD_GET_WORDDATA(pFibInfo);
	return (ulMsb << 16 | ulLsb);
}

LGD_UINT32 LGD_GETAT_LONGDATA(ST_FIB_INFO* pFibInfo)
{
	LGD_UINT32 ulMsb, ulLsb;
	ulMsb = (LGD_UINT32)LGD_GETAT_WORDDATA(pFibInfo);
	pFibInfo->ucDataPos += 2;
	ulLsb = (LGD_UINT32)LGD_GETAT_WORDDATA(pFibInfo);
	pFibInfo->ucDataPos -= 2;
	return (ulMsb << 16 | ulLsb);
}

LGD_UINT8 LGD_GET_HEADER(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos++];
}

LGD_UINT8 LGD_GETAT_HEADER(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos];
}

LGD_UINT8 LGD_GET_TYPE(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos++];
}

LGD_UINT8 LGD_GETAT_TYPE(ST_FIB_INFO* pInfo)
{
	return pInfo->aucBuff[pInfo->ucDataPos+1];
}

LGD_UINT8 LGD_GET_NULLBLOCK(ST_FIG_HEAD* pInfo)
{
	if(pInfo->ucInfo == END_MARKER) return LGD_ERROR;
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_GET_FINDLENGTH(ST_FIG_HEAD* pInfo)
{
 	if(!pInfo->ITEM.bitLength || pInfo->ITEM.bitLength > FIB_SIZE-3) 
		return LGD_ERROR;
	return LGD_SUCCESS;
}

LGD_UINT16 LGD_CRC_CHECK(LGD_UINT8 *pBuf, LGD_UINT8 ucSize) 
{
  LGD_UINT8	ucLoop;
  LGD_UINT16	nCrc = 0xFFFF;

  for(ucLoop = 0; ucLoop < ucSize; ucLoop++){
    nCrc  = 0xFFFF & (((nCrc<<8) | (nCrc>>8)) ^ (0xFF & pBuf[ucLoop]));                        
    nCrc  = nCrc ^ ((0xFF & nCrc) >> 4);
    nCrc  = 0xFFFF & (nCrc ^ ( (((((0xFF & nCrc))<<8)|(((0xFF & nCrc))>>8))) << 4) ^ ((0xFF & nCrc) << 5));
  }

  return((LGD_UINT16)0xFFFF & (nCrc ^ 0xFFFF));
}

LGD_UINT16 LGD_GET_BITRATE(SCH_DB_STRT * pstSchDb)
{
  LGD_UINT16 uiBitRate = 0;

	if(!pstSchDb->ucShortLong){
		if(pstSchDb->ucTableIndex <= 4)  uiBitRate = 32;
		else if(pstSchDb->ucTableIndex >= 5  && pstSchDb->ucTableIndex <= 9)  uiBitRate = 48;
		else if(pstSchDb->ucTableIndex >= 10 && pstSchDb->ucTableIndex <= 13) uiBitRate = 56;
		else if(pstSchDb->ucTableIndex >= 14 && pstSchDb->ucTableIndex <= 18) uiBitRate = 64;
		else if(pstSchDb->ucTableIndex >= 19 && pstSchDb->ucTableIndex <= 23) uiBitRate = 80;
		else if(pstSchDb->ucTableIndex >= 24 && pstSchDb->ucTableIndex <= 28) uiBitRate = 96;
		else if(pstSchDb->ucTableIndex >= 29 && pstSchDb->ucTableIndex <= 32) uiBitRate = 112;
		else if(pstSchDb->ucTableIndex >= 33 && pstSchDb->ucTableIndex <= 37) uiBitRate = 128;
		else if(pstSchDb->ucTableIndex >= 38 && pstSchDb->ucTableIndex <= 42) uiBitRate = 160;
		else if(pstSchDb->ucTableIndex >= 43 && pstSchDb->ucTableIndex <= 47) uiBitRate = 192;
		else if(pstSchDb->ucTableIndex >= 48 && pstSchDb->ucTableIndex <= 52) uiBitRate = 224;
		else if(pstSchDb->ucTableIndex >= 53 && pstSchDb->ucTableIndex <= 57) uiBitRate = 256;
		else if(pstSchDb->ucTableIndex >= 58 && pstSchDb->ucTableIndex <= 60) uiBitRate = 320;
		else if(pstSchDb->ucTableIndex >= 61 && pstSchDb->ucTableIndex <= 63) uiBitRate = 384;
		else uiBitRate = 0;
  }
  else
	{
    if(pstSchDb->ucOption == OPTION_INDICATE0) 
		{
      switch(pstSchDb->ucProtectionLevel){
      case PROTECTION_LEVEL0: uiBitRate = (pstSchDb->uiSubChSize/12)*8;break;
      case PROTECTION_LEVEL1: uiBitRate = (pstSchDb->uiSubChSize/8)*8;break;
      case PROTECTION_LEVEL2: uiBitRate = (pstSchDb->uiSubChSize/6)*8;break;
      case PROTECTION_LEVEL3: uiBitRate = (pstSchDb->uiSubChSize/4)*8;break;
      }
    }
    else if(pstSchDb->ucOption == OPTION_INDICATE1)
		{
      switch(pstSchDb->ucProtectionLevel){
      case PROTECTION_LEVEL0: uiBitRate = (pstSchDb->uiSubChSize/27)*32;break;
      case PROTECTION_LEVEL1: uiBitRate = (pstSchDb->uiSubChSize/21)*32;break;
      case PROTECTION_LEVEL2: uiBitRate = (pstSchDb->uiSubChSize/18)*32;break;
      case PROTECTION_LEVEL3: uiBitRate = (pstSchDb->uiSubChSize/15)*32;break;
     	}
    }
  }
  return uiBitRate;
}

LGD_UINT16 LGD_FIND_SUBCH_SIZE(LGD_UINT8 ucTableIndex) 
{
    LGD_UINT16 wSubCHSize = 0;

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

void LGD_SET_SHORTFORM(ST_FIC_DB* pstFicDB, LGD_INT32 nCh, ST_TYPE0of1Short_INFO* pShort)
{
	SCH_DB_STRT* pSch_db;

	pSch_db 										= &pstFicDB->astSchDb[nCh];
	pSch_db->ucShortLong 				= pShort->ITEM.bitShortLong;
	if(pSch_db->ucShortLong)		return;

 	pSch_db->ucTableSW 					= pShort->ITEM.bitTableSw;
	pSch_db->ucTableIndex 			= pShort->ITEM.bitTableIndex;
	pSch_db->ucOption 					= 0;
	pSch_db->ucProtectionLevel 	= 0;
	pSch_db->uiSubChSize        = LGD_FIND_SUBCH_SIZE(pSch_db->ucTableIndex);
	pSch_db->uiDifferentRate   	= 0;
	pSch_db->uiBitRate          = LGD_GET_BITRATE(pSch_db);
}

void LGD_SET_LONGFORM(ST_FIC_DB* pstFicDB, LGD_INT32 nCh, ST_TYPE0of1Long_INFO* pLong)
{
    SCH_DB_STRT* pSch_db;
	
	pSch_db = &pstFicDB->astSchDb[nCh];
	pSch_db->ucShortLong = pLong->ITEM.bitShortLong;

	if(!pSch_db->ucShortLong)	return;

	pSch_db->ucTableSW = 0;
	pSch_db->ucTableIndex = 0;
	pSch_db->ucOption = pLong->ITEM.bitOption;
	pSch_db->ucProtectionLevel = pLong->ITEM.bitProtecLevel;
	pSch_db->uiSubChSize = pLong->ITEM.bitSubChanSize;

	if(pSch_db->ucOption == 0 ){
		switch(pSch_db->ucProtectionLevel) {
		case  0 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/12); break;
		case  1 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/8); break;
		case  2 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/6); break;
		case  3 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/4); break;
		default : pSch_db->uiDifferentRate = 0; break; 
		}
	}
	else {
		switch(pSch_db->ucProtectionLevel) {
		case  0 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/27); break;
		case  1 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/21); break;
		case  2 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/18); break;
		case  3 : pSch_db->uiDifferentRate = (pSch_db->uiSubChSize/15); break;
		default : pSch_db->uiDifferentRate = 0; break; 
		}
	}
	pSch_db->uiBitRate = LGD_GET_BITRATE(pSch_db);
}

void LGD_EXTENSION_000(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_TYPE0of0_INFO*		pBitStarm;
	LGD_UINT32 ulBitStram = 0;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulBitStram = LGD_GET_LONGDATA(pFibInfo);
	pBitStarm = (ST_TYPE0of0_INFO*)&ulBitStram;
	pstFicDB->uiEnsembleID = pBitStarm->ITEM.bitEld;

	if(pBitStarm->ITEM.bitChangFlag) pFibInfo->ucDataPos += 1;
}

void LGD_EXTENSION_001(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
  ST_TYPE0of1Short_INFO*	pShortInfo = LGD_NULL;
  ST_TYPE0of1Long_INFO*	pLongInfo = LGD_NULL;

	LGD_UINT32 ulTypeInfo;
	LGD_UINT16 uiStartAddr;
	LGD_UINT8 ucSubChId, ucLoop, ucIndex, ucShortLongFlag, ucIsSubCh;

	LGD_UINT16 uSubCHTick = 0;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	pstFicDB->ucSubChCnt = LGD_CHECK_SERVICE_DB8(pstFicDB->aucSubChID, INIT_FIC_DB08, MAX_SUBCHANNEL);

	for(ucIndex = 0; ucIndex < pstHeader->ITEM.bitLength-1;)
	{
		uSubCHTick++;
		ucShortLongFlag = (pFibInfo->aucBuff[pFibInfo->ucDataPos+2] >> 7) & 0x01;

		if(ucShortLongFlag == 0)
		{
			ulTypeInfo = LGD_GET_LONGDATA(pFibInfo);
			pShortInfo = (ST_TYPE0of1Short_INFO*)&ulTypeInfo;
			pFibInfo->ucDataPos -= 1; 

			ucSubChId = pShortInfo->ITEM.bitSubChId;
			uiStartAddr = pShortInfo->ITEM.bitStartAddr;
		}
		else
		{	
			ulTypeInfo = LGD_GET_LONGDATA(pFibInfo);
			pLongInfo = (ST_TYPE0of1Long_INFO*)&ulTypeInfo;

			ucSubChId = pLongInfo->ITEM.bitSubChId;
			uiStartAddr = pLongInfo->ITEM.bitStartAddr;
		}

		ucIsSubCh = 0;
		for(ucLoop = 0 ; ucLoop < pstFicDB->ucSubChCnt; ucLoop++)
		{
			if(((pstFicDB->aucSubChID[ucLoop] & LGD_BIT_MASK) == (ucSubChId & LGD_BIT_MASK)) 		//inb612 for android compile
				|| (pstFicDB->auiStartAddr[ucLoop] == uiStartAddr)) 
			{
				if(ucShortLongFlag == 0) LGD_SET_SHORTFORM(pstFicDB, ucLoop, pShortInfo);
				else LGD_SET_LONGFORM(pstFicDB, ucLoop, pLongInfo);
			
				ucIsSubCh = 1;
				break;
			}
		}

		if(!ucIsSubCh) 
		{
			pstFicDB->auiStartAddr[pstFicDB->ucSubChCnt] = uiStartAddr;
			pstFicDB->aucSubChID[pstFicDB->ucSubChCnt] = ucSubChId;

			if(ucShortLongFlag == 0) LGD_SET_SHORTFORM(pstFicDB, pstFicDB->ucSubChCnt, pShortInfo);
			else LGD_SET_LONGFORM(pstFicDB, pstFicDB->ucSubChCnt, pLongInfo);
	
			pstFicDB->ucSubChCnt++;
		}

		ucIndex += (!ucShortLongFlag) ? 3 : 4;
	}

	if(pstFicDB->ucSubChCnt)
		g_bFIC_OK = 1;
}

void LGD_EXTENSION_002(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_SERVICE_COMPONENT*	pService;
	ST_TMId_MSCnFIDC*		pMscnFidc;
	ST_MSC_PACKET_INFO*		pMscPacket;

	LGD_UINT32 ulServiceId;
	LGD_UINT16 uiData;
	LGD_INT16 nChanPos;
	LGD_UINT8 ucService, ucLoop, ucIndex, ucFrame, ucTMID, ucSubCnt;

	LGD_UINT32 uServiceTick = 0;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucFrame = 0 ; ucFrame < pstHeader->ITEM.bitLength-1; )
	{
		uServiceTick++;

		if(pstType->ITEM.bitPD == 0) {
			ulServiceId = (LGD_UINT32)LGD_GET_WORDDATA(pFibInfo);
			ucFrame += 2;
		}
		else {
			ulServiceId = LGD_GET_LONGDATA(pFibInfo);
			ucFrame += 4;
		}

		ucService = LGD_GET_BYTEDATA(pFibInfo);
		ucFrame += 1;
		pService = (ST_SERVICE_COMPONENT*)&ucService;

		for(ucLoop = 0; ucLoop < pService->ITEM.bitNumComponent; ucLoop++, ucFrame+=2)
		{
			uiData = LGD_GET_WORDDATA(pFibInfo);
			ucTMID = (LGD_UINT8)(uiData >> 14) & 0x0003;

			if(TMID_2 == ucTMID) continue;
		
			switch(ucTMID) 
			{
			case TMID_0 :
			case TMID_1 :
				pMscnFidc = (ST_TMId_MSCnFIDC*)&uiData;
				//nChanPos = LGD_CHECK_SERVICE_CNT8(pstFicDB->aucSubChID, (LGD_UINT8)pMscnFidc->ITEM.bitSubChld, pstFicDB->ucSubChCnt, BIT_MASK);
				nChanPos = LGD_CHECK_SERVICE_CNT32(pstFicDB->aulServiceID, ulServiceId, pstFicDB->ucSubChCnt);
				if(nChanPos != -1)
				{
					if(pMscnFidc->ITEM.bitTMId == TMID_0) 
						pstFicDB->aucSubChID[nChanPos] &= 0x7f;

					pstFicDB->aulServiceID[nChanPos] = ulServiceId;
					pstFicDB->aucDSCType[nChanPos] = (LGD_UINT8)pMscnFidc->ITEM.bitAscDscTy;
					pstFicDB->uiSubChOk |= 1 << nChanPos;
					pstFicDB->aucTmID[nChanPos] = (LGD_UINT8)pMscnFidc->ITEM.bitTMId;
				}
				break;

			default :
				pMscPacket = (ST_MSC_PACKET_INFO*)&uiData;

				for(ucIndex = 0; ucIndex < (LGD_INT32)pstFicDB->ucServiceComponentCnt; ucIndex++){
					if((pstFicDB->auiServicePackMode[ucIndex] & 0xfff) != pMscPacket->ITEM.bitSCId) continue;
					for(ucSubCnt = 0 ; ucSubCnt < pstFicDB->ucSubChCnt ; ucSubCnt++)
					{
						if((pstFicDB->aucSubChID[ucSubCnt] & LGD_BIT_MASK) != pstFicDB->aucSubChPackMode[ucIndex])  	//inb612 for android compile
							continue;

						pstFicDB->aulServiceID[ucSubCnt] = ulServiceId;
						pstFicDB->aucDSCType[ucSubCnt] = pstFicDB->aucServiceTypePackMode[ucIndex];
						pstFicDB->aucSetPackAddr[ucSubCnt] = pstFicDB->auiPacketAddr[ucIndex];
						
						pstFicDB->uiSubChOk |= 1 << ucSubCnt;
						pstFicDB->aucTmID[ucSubCnt] = TMID_3;
					}
				}
				break;
			}
		}
	}
}

void LGD_EXTENSION_003(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_TYPE0of3_INFO*		pTypeInfo;
	ST_TYPE0of3Id_INFO*		pIdInfo;

	LGD_UINT32 uiId, uiTypeInfo;
	LGD_UINT8 ucLoop, ucIsSubCh, ucIndex;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucIndex = 0 ; ucIndex < pstHeader->ITEM.bitLength-1; )
	{
		uiTypeInfo = LGD_GET_WORDDATA(pFibInfo);
		pTypeInfo = (ST_TYPE0of3_INFO*)&uiTypeInfo;

		uiId = LGD_GET_WORDDATA(pFibInfo);
		uiId = (uiId << 16) | (LGD_GET_BYTEDATA(pFibInfo)<<8);
		pIdInfo = (ST_TYPE0of3Id_INFO*)&uiId;

		ucIndex += 5;

		pstFicDB->ucServiceComponentCnt = LGD_CHECK_SERVICE_DB16(pstFicDB->auiServicePackMode, 0xffff, MAX_SUBCHANNEL); 
		ucIsSubCh = LGD_ERROR;

		for(ucLoop = 0 ; ucLoop < pstFicDB->ucServiceComponentCnt ; ucLoop++)
		{
			if(((pstFicDB->auiServicePackMode[ucLoop] & LGD_BIT_MASK) == (pTypeInfo->ITEM.bitScid & LGD_BIT_MASK)) && 
         				((pstFicDB->aucSubChPackMode[ucLoop] & LGD_BIT_MASK) == ((LGD_UINT8)pIdInfo->ITEM.bitSubChId & LGD_BIT_MASK))) 	//inb612 for android compile
			{
				ucIsSubCh = LGD_SUCCESS;
				break;
			}
		}

		if(!ucIsSubCh)
		{
			pstFicDB->auiServicePackMode[pstFicDB->ucServiceComponentCnt] = pTypeInfo->ITEM.bitScid;
			pstFicDB->aucServiceTypePackMode[pstFicDB->ucServiceComponentCnt] = (LGD_UINT8)pIdInfo->ITEM.bitDScType;
			pstFicDB->aucSubChPackMode[pstFicDB->ucServiceComponentCnt] = (LGD_UINT8)pIdInfo->ITEM.bitSubChId;
			pstFicDB->auiPacketAddr[pstFicDB->ucServiceComponentCnt] = pIdInfo->ITEM.bitPacketAddr;
			pstFicDB->ucServiceComponentCnt++; 
		}

		if(pTypeInfo->ITEM.bitSccaFlag == 1){
			ucIndex += 2;
			pFibInfo->ucDataPos += 2;
		}
	}
}

void LGD_EXTENSION_008(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*			pstHeader;
	ST_TYPE_0*				pstType;
	ST_MSC_BIT*				pstMsc;
	ST_MSC_SHORT*			pstMscShort;
	ST_MSC_LONG*			pstMscLong;

	LGD_UINT16 uiMsgBit;
	LGD_UINT32 ulSerId = 0;
	LGD_UINT8 ucMscInfo, ucIndex, ucLSFlag;
	LGD_INT16 cResult;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	for(ucIndex = 0 ; ucIndex < pstHeader->ITEM.bitLength-1; )
	{
		if(!pstType->ITEM.bitPD){
			ulSerId = (LGD_UINT32)LGD_GET_WORDDATA(pFibInfo);
			ucIndex += 2;
		}
		else {
			ulSerId = LGD_GET_LONGDATA(pFibInfo);
			ucIndex += 4;
		}

		ucMscInfo = LGD_GET_BYTEDATA(pFibInfo); 
		pstMsc = (ST_MSC_BIT*)&ucMscInfo;
		ucIndex += 1;
		ucLSFlag = (LGD_GETAT_BYTEDATA(pFibInfo) >> 7) & 0x01;

		if(ucLSFlag) {
			uiMsgBit = LGD_GET_WORDDATA(pFibInfo);
			ucIndex += 2;
			pstMscLong = (ST_MSC_LONG*)&uiMsgBit;

			cResult = LGD_CHECK_SERVICE_CNT16(pstFicDB->auiServicePackMode, 
				pstMscLong->ITEM.bitScId, pstFicDB->ucServiceComponentCnt, 0xfff);
			if(cResult != -1){
				cResult = LGD_CHECK_SERVICE_CNT8(pstFicDB->aucSubChID, 
					pstFicDB->aucSubChPackMode[cResult], pstFicDB->ucSubChCnt, LGD_BIT_MASK);		//inb612 for android compile
				if(cResult != -1) {
					pstFicDB->aucServiceComponID[cResult] = pstMsc->ITEM.bitScIds;
					pstFicDB->aulServiceID[cResult] = ulSerId;
				}
			}
		}
		else{
			uiMsgBit = (LGD_UINT16)LGD_GET_BYTEDATA(pFibInfo);
			ucIndex += 1;
			pstMscShort = (ST_MSC_SHORT*)&uiMsgBit;
			if(!pstMscShort->ITEM.bitMscFicFlag){
				cResult = LGD_CHECK_SERVICE_CNT8(pstFicDB->aucSubChID, 
					pstMscShort->ITEM.bitSUBnFIDCId, pstFicDB->ucSubChCnt, LGD_BIT_MASK);		//inb612 for android compile
				if(cResult >= 0) {
					pstFicDB->aucServiceComponID[cResult] = pstMsc->ITEM.bitScIds;
					pstFicDB->aulServiceID[cResult] = ulSerId;
				}
			}
		}

		if(pstMsc->ITEM.bitExtFlag) {
			ucIndex += 1;
			pFibInfo->ucDataPos += 1;
		}
	}
}

void LGD_EXTENSION_010(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*		pstHeader;
	ST_TYPE_0*			pstType;
	ST_UTC_SHORT_INFO*	pstUTC_Short_Info;
	ST_UTC_LONG_INFO*	pstUTC_Long_Info;
	ST_UTC_INFO*		pstUTC_Info;
	
	LGD_UINT32		ulUtcInfo;
	LGD_UINT16		uiUtcLongForm;

	pstHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pstType = (ST_TYPE_0*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulUtcInfo = LGD_GET_LONGDATA(pFibInfo);
	pstUTC_Short_Info = (ST_UTC_SHORT_INFO*)&ulUtcInfo;

	pstUTC_Info = (ST_UTC_INFO*)&g_stUTC_Info;

	pstUTC_Info->uiHours = pstUTC_Short_Info->ITEM.bitHours;
	pstUTC_Info->uiMinutes = pstUTC_Short_Info->ITEM.bitMinutes;
	pstUTC_Info->ucUTC_Flag = pstUTC_Short_Info->ITEM.bitUTC_Flag;

	if(pstUTC_Info->ucUTC_Flag){
		uiUtcLongForm = LGD_GET_WORDDATA(pFibInfo);
		pstUTC_Long_Info = (ST_UTC_LONG_INFO*)&uiUtcLongForm;

		pstUTC_Info->uiSeconds = pstUTC_Long_Info->ITEM.bitSeconds;
		pstUTC_Info->uiMilliseconds = pstUTC_Long_Info->ITEM.bitMilliseconds;
	}
	pstUTC_Info->ucGet_Time = 1;
}

#ifdef USER_APPLICATION_TYPE
void LGD_EXTENSION_013(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*		pstHeader;
	ST_TYPE_0*			pstType;
	ST_USER_APP_IDnNUM*	pUserAppIdNum;
	ST_USER_APPTYPE*	pUserAppType;
	LGD_UINT32	ulSid;
	LGD_UINT16	uiUserAppType;
	LGD_UINT8	ucHeader, ucType, ucUserAppIdNum;
	LGD_UINT8	ucFrame, ucIndex, ucLoop, ucDataCnt;

	ucHeader = LGD_GET_HEADER(pFibInfo);
	pstHeader = (ST_FIG_HEAD*)&ucHeader;
	
	ucType = LGD_GET_TYPE(pFibInfo);
	pstType = (ST_TYPE_0*)&ucType;


	for(ucFrame = 0 ; ucFrame < pstHeader->ITEM.bitLength-1; ){

		if(pstType->ITEM.bitPD){
			ulSid = LGD_GET_LONGDATA(pFibInfo);
			ucFrame += 4;
		}
		else {
			ulSid = LGD_GET_WORDDATA(pFibInfo);
			ucFrame += 2;
		}
		
		ucUserAppIdNum = LGD_GET_BYTEDATA(pFibInfo);
		pUserAppIdNum = (ST_USER_APP_IDnNUM*)&ucUserAppIdNum;
		ucFrame += 1;
		pstFicDB->astUserAppInfo.ucSCIdS = pUserAppIdNum->ITEM.bitSCIdS;
		pstFicDB->astUserAppInfo.ucNomOfUserApp = pUserAppIdNum->ITEM.bitNomUserApp;

		for(ucIndex = 0; ucIndex < pstFicDB->astUserAppInfo.ucNomOfUserApp; ucIndex++){
			uiUserAppType = LGD_GET_WORDDATA(pFibInfo);
			pUserAppType = (ST_USER_APPTYPE*)&uiUserAppType;
			ucFrame += 2;

			for(ucLoop = 0; ucLoop < pstFicDB->ucSubChCnt; ucLoop++)
			{
				if(ulSid != pstFicDB->aulServiceID[ucLoop]) continue;

				pstFicDB->astUserAppInfo.uiUserAppType[ucLoop] = pUserAppType->ITEM.bitUserAppType;
				pstFicDB->astUserAppInfo.ucUserAppDataLength[ucLoop] = pUserAppType->ITEM.bitUserDataLength;
				
				for(ucDataCnt = 0; ucDataCnt < pstFicDB->astUserAppInfo.ucUserAppDataLength[ucLoop]; ucDataCnt++){
					pstFicDB->astUserAppInfo.aucUserAppData[ucLoop][ucDataCnt] = LGD_GET_BYTEDATA(pFibInfo);
					ucFrame += 1;
				}
				
 				if(pstFicDB->aucTmID[ucLoop] == TMID_3)
					pstFicDB->uiUserAppTypeOk |= 1 << ucLoop;
				break;
			}
		}
	}
}
#endif


LGD_UINT16 LGD_SET_FICTYPE_0(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD* pHeader;
	ST_TYPE_0*   pExtern;
	LGD_UINT8 ucHeader, ucType;
	
	ucHeader = LGD_GETAT_HEADER(pFibInfo);
	pHeader = (ST_FIG_HEAD*)&ucHeader;

	ucType = LGD_GETAT_TYPE(pFibInfo);
	pExtern = (ST_TYPE_0*)&ucType;

	switch(pExtern->ITEM.bitExtension)
	{
	case EXTENSION_0: LGD_EXTENSION_000(pFibInfo, pstFicDB); break;
	case EXTENSION_1: LGD_EXTENSION_001(pFibInfo, pstFicDB); break;
	case EXTENSION_2: LGD_EXTENSION_002(pFibInfo, pstFicDB); break;
	case EXTENSION_3: LGD_EXTENSION_003(pFibInfo, pstFicDB); break;
	case EXTENSION_8: LGD_EXTENSION_008(pFibInfo, pstFicDB); break;
	case EXTENSION_10: LGD_EXTENSION_010(pFibInfo, pstFicDB); break;
#ifdef USER_APPLICATION_TYPE
	case EXTENSION_13: LGD_EXTENSION_013(pFibInfo, pstFicDB); break;
#endif
	default: pFibInfo->ucDataPos += pHeader->ITEM.bitLength + 1; break;
	}
	return LGD_SUCCESS;
}

void LGD_EXTENSION_110(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_1*		pType;
	LGD_UINT8		ucLoop;
	LGD_UINT16		uiEId;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType	= (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	uiEId = LGD_GET_WORDDATA(pFibInfo);

	for(ucLoop = 0; ucLoop < MAX_LABEL_CHAR ; ucLoop++){
		pstFicDB->aucEnsembleLabel[ucLoop] = LGD_GET_BYTEDATA(pFibInfo);
	}
	pFibInfo->ucDataPos += 2;
	if(uiEId == pstFicDB->uiEnsembleID)	
		pstFicDB->uiSubChInfoOk |= 1 << 15;
}

void LGD_EXTENSION_111(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*	pHeader;
	ST_TYPE_1*		pType;

	LGD_UINT16 uiSId;
	LGD_UINT8 ucLoop, ucIndex, ucIsLable = 0;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	uiSId = LGD_GET_WORDDATA(pFibInfo);
	for(ucLoop = 0 ; ucLoop < (LGD_UINT8)pstFicDB->ucSubChCnt ; ucLoop++) 
	{
		if(uiSId == (LGD_UINT16)pstFicDB->aulServiceID[ucLoop] && pstFicDB->aucServiceExtension[ucLoop] == 0xff) 
		{
			for(ucIndex = 0 ; ucIndex < MAX_LABEL_CHAR ; ucIndex++) {
				pstFicDB->aucServiceLabel[ucLoop][ucIndex] = LGD_GET_BYTEDATA(pFibInfo);
			}

			pstFicDB->uiSubChInfoOk |= 1 << ucLoop;
			pFibInfo->ucDataPos += 2;
			ucIsLable = LGD_SUCCESS;
		}
	}
	if(!ucIsLable) pFibInfo->ucDataPos += 18;
}

void LGD_EXTENSION_112(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pHeader;
	ST_TYPE_1*				pType;
	ST_EXTENSION_TYPE12*	pType12;
	LGD_UINT8 ucData;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ucData = LGD_GET_BYTEDATA(pFibInfo);
	pType12 = (ST_EXTENSION_TYPE12*)&ucData;

	if(pType12->ITEM.bitCF_flag) pFibInfo->ucDataPos += 1;

	pFibInfo->ucDataPos += 19;
	if(pType12->ITEM.bitCountry == 1) 
		pFibInfo->ucDataPos += 2;
}

void LGD_EXTENSION_113(ST_FIB_INFO* pFibInfo)
{
	ST_FIG_HEAD*			pHeader;
	ST_TYPE_1*				pType;
	
	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pFibInfo->ucDataPos += 19;
}

void LGD_EXTENSION_114(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*		pHeader;
	ST_TYPE_1*			pType;
	ST_EXTENSION_TYPE14* pExtenType;
	LGD_UINT32 ulSId;
	LGD_UINT8 ucData, ucLoop, ucIndex, ucIsLable = 0;

	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ucData = LGD_GET_BYTEDATA(pFibInfo);
	pExtenType = (ST_EXTENSION_TYPE14*)&ucData;

	if(!pExtenType->ITEM.bitPD) 
		ulSId = (LGD_UINT32)LGD_GET_WORDDATA(pFibInfo); 
	else 
		ulSId = LGD_GET_LONGDATA(pFibInfo);

  for(ucLoop = 0 ; ucLoop < (LGD_UINT8)pstFicDB->ucSubChCnt ; ucLoop++)
	{
    if(ulSId == pstFicDB->aulServiceID[ucLoop] && pExtenType->ITEM.bitSCidS == pstFicDB->aucServiceComponID[ucLoop])
		{
      for(ucIndex = 0 ; ucIndex < MAX_LABEL_CHAR ; ucIndex++) {
          pstFicDB->aucServiceLabel[ucLoop][ucIndex] = LGD_GET_BYTEDATA(pFibInfo);
			}
			pstFicDB->uiSubChInfoOk |= 1 << ucLoop;
			pFibInfo->ucDataPos += 2;
			ucIsLable = LGD_SUCCESS;
    }
  }
	if(!ucIsLable) pFibInfo->ucDataPos += 18;
}

void LGD_EXTENSION_115(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_FIG_HEAD*		pHeader;
	ST_TYPE_1*			pType;
	LGD_UINT32 ulSId;
	LGD_UINT8 ucLoop, ucIndex, ucIsLable = 0;
	
	pHeader = (ST_FIG_HEAD*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];
	pType = (ST_TYPE_1*)&pFibInfo->aucBuff[pFibInfo->ucDataPos++];

	ulSId = LGD_GET_LONGDATA(pFibInfo);
  for(ucLoop = 0 ; ucLoop < (LGD_UINT8)pstFicDB->ucSubChCnt ; ucLoop++)
	{ 
    if(ulSId == pstFicDB->aulServiceID[ucLoop] && pstFicDB->aucServiceExtension[ucLoop] == 0xff){

      for(ucIndex = 0; ucIndex < MAX_LABEL_CHAR; ucIndex++) {
				pstFicDB->aucServiceLabel[ucLoop][ucIndex] = LGD_GET_BYTEDATA(pFibInfo);
			}

			pstFicDB->uiSubChInfoOk |= 1 << ucLoop;
			pFibInfo->ucDataPos += 2;
			ucIsLable = LGD_SUCCESS;
		}
	}
	if(!ucIsLable) pFibInfo->ucDataPos += 18;
}

void LGD_SET_FICTYPE_1(ST_FIB_INFO* pFibInfo, ST_FIC_DB* pstFicDB)
{
	ST_TYPE_1* pExtern;
	ST_FIG_HEAD* pHeader;
	LGD_UINT8 ucType, ucHeader, ucLoop, ucIndex;
	
	ucHeader = LGD_GETAT_HEADER(pFibInfo);
	ucType = LGD_GETAT_TYPE(pFibInfo);

	pHeader = (ST_FIG_HEAD*)&ucHeader;
	pExtern = (ST_TYPE_1*)&ucType;

	for(ucLoop = 0 ; ucLoop < (LGD_UINT8)pstFicDB->ucSubChCnt ; ucLoop++) {
		for(ucIndex = (ucLoop + 1) ; ucIndex < (LGD_UINT8)pstFicDB->ucSubChCnt ; ucIndex++) {
			if((pstFicDB->aulServiceID[ucLoop] == pstFicDB->aulServiceID[ucIndex]) && 
				(pstFicDB->aulServiceID[ucLoop] != 0xffffffff )) {
				pstFicDB->aucServiceExtension[ucLoop] = 1;
				pstFicDB->aucServiceExtension[ucIndex] = 1;
			}
		}
	}

	switch(pExtern->ITEM.bitExtension){
	case EXTENSION_0: LGD_EXTENSION_110(pFibInfo, pstFicDB); break;
	case EXTENSION_1: LGD_EXTENSION_111(pFibInfo, pstFicDB); break;
	case EXTENSION_2: LGD_EXTENSION_112(pFibInfo); break;
	case EXTENSION_3: LGD_EXTENSION_113(pFibInfo); break;
	case EXTENSION_4: LGD_EXTENSION_114(pFibInfo, pstFicDB); break;
	case EXTENSION_5: LGD_EXTENSION_115(pFibInfo, pstFicDB); break;
	default: pFibInfo->ucDataPos += (pHeader->ITEM.bitLength + 1); break;
	}
}

void LGD_SET_UPDATEFIC(ST_FIB_INFO* pstDestData, LGD_UINT8* pSourData)
{
	LGD_UINT8 cuIndex;
	LGD_UINT16 wCRC, wCRCData;
	for(cuIndex = 0; cuIndex < FIB_SIZE; cuIndex++) 
		pstDestData->aucBuff[cuIndex] = pSourData[cuIndex];

	wCRC = LGD_CRC_CHECK((LGD_UINT8*)pstDestData->aucBuff, FIB_SIZE-2);
	wCRCData = ((LGD_UINT16)pstDestData->aucBuff[30] << 8) | pstDestData->aucBuff[31];

	pstDestData->ucDataPos = 0;
	pstDestData->uiIsCRC = wCRC == wCRCData;
}

LGD_UINT8 LGD_GET_FINDTYPE(ST_FIG_HEAD* pInfo)
{
	if(pInfo->ITEM.bitType <= FIG_IN_HOUSE) return LGD_SUCCESS;
	return LGD_ERROR;
}

LGD_UINT8 LGD_FICPARSING(LGD_UINT8 ucI2CID, LGD_UINT8* pucFicBuff, LGD_INT32 uFicLength, ST_SIMPLE_FIC bSimpleFIC)
{
	ST_FIC 				stFIC;
	ST_FIB_INFO* 	pstFib;
	ST_FIG_HEAD* 	pHeader;
	ST_FIC_DB* 		pFicDb;
	
  LGD_UINT8 ucLoop, ucHeader;
  LGD_UINT16 uiTempIndex = 0;

	pFicDb = LGD_GETFIC_DB(ucI2CID);
	stFIC.ucBlockNum = uFicLength/FIB_SIZE;
	pstFib = &stFIC.stBlock;

	for(ucLoop = 0; ucLoop < stFIC.ucBlockNum; ucLoop++)
	{
		LGD_SET_UPDATEFIC(pstFib, &pucFicBuff[ucLoop*FIB_SIZE]);
		if(!pstFib->uiIsCRC) continue;

		while(pstFib->ucDataPos < FIB_SIZE-2)
		{
			ucHeader = LGD_GETAT_HEADER(pstFib);
			pHeader = (ST_FIG_HEAD*)&ucHeader;

			if(!LGD_GET_FINDTYPE(pHeader) || !LGD_GET_NULLBLOCK(pHeader) || !LGD_GET_FINDLENGTH(pHeader)) break;

			switch(pHeader->ITEM.bitType) {
				case FIG_MCI_SI			: LGD_SET_FICTYPE_0(pstFib, pFicDb);break;
				case FIG_LABLE			: LGD_SET_FICTYPE_1(pstFib, pFicDb);break;
				case FIG_RESERVED_0		: 
				case FIG_RESERVED_1		: 
				case FIG_RESERVED_2		: 
				case FIG_FICDATA_CHANNEL:
				case FIG_CONDITION_ACCESS:
				case FIG_IN_HOUSE		: 
				default					: pstFib->ucDataPos += pHeader->ITEM.bitLength + 1;break;
			}

			if(pstFib->ucDataPos == FIB_SIZE-1) break;
		}
	}

	if(stFIC.ucBlockNum && stFIC.ucBlockNum == ucLoop){
		uiTempIndex = (1 << pFicDb->ucSubChCnt) - 1;
		uiTempIndex |= 0x8000;

		if(bSimpleFIC == SIMPLE_FIC_ENABLE && g_bFIC_OK){
			return LGD_SUCCESS;
		}

		if((uiTempIndex == pFicDb->uiSubChInfoOk) && ((uiTempIndex & 0x7FFF) == pFicDb->uiSubChOk) && pFicDb->ucSubChCnt )
			return LGD_SUCCESS;

	}

 	return LGD_ERROR;
}

void LGD_INITDB(LGD_UINT8 ucI2CID) 
{
	LGD_UINT8 ucLoop;
	SCH_DB_STRT* astSchDb;
	ST_FIC_DB* pFicDb;
	ST_UTC_INFO*	pstUTC_Info;

	pFicDb = LGD_GETFIC_DB(ucI2CID);
	pstUTC_Info = (ST_UTC_INFO*)&g_stUTC_Info;

	g_bFIC_OK = 0;

	memset(pFicDb->aucEnsembleLabel, 0, sizeof(LGD_UINT8)*MAX_LABEL_CHAR);
	memset(pFicDb->aucServiceLabel, 0, sizeof(LGD_UINT8)*MAX_LABEL_CHAR*MAX_SUBCHANNEL);
	
	pstUTC_Info->ucGet_Time = 0;
	
	pFicDb->ucSubChCnt = 0;
	pFicDb->uiEnsembleID = 0;
	pFicDb->uiSubChOk = 0;
	pFicDb->uiSubChInfoOk = 0;
	pFicDb->ucServiceComponentCnt = 0; 

	pFicDb->uiUserAppTypeOk = 0;
#ifdef USER_APPLICATION_TYPE	
	pFicDb->astUserAppInfo.ucNomOfUserApp = 0;
#endif


  for(ucLoop = 0 ; ucLoop < MAX_SUBCHANNEL ; ucLoop++){              
    astSchDb = &pFicDb->astSchDb[ucLoop];
		memset(astSchDb, -1, sizeof(SCH_DB_STRT));

    pFicDb->aucDSCType[ucLoop] 							= 0xff;
    pFicDb->aucSubChID[ucLoop] 							= 0xff;
    pFicDb->aucTmID[ucLoop] 								= 0xff;
    pFicDb->auiStartAddr[ucLoop] 						= 0xffff;
    pFicDb->aulServiceID[ucLoop] 						= 0xffffffff;
    pFicDb->auiServicePackMode[ucLoop] 			= 0xffff;
    pFicDb->aucSubChPackMode[ucLoop] 				= 0xff;
    pFicDb->aucServiceTypePackMode[ucLoop] 	= 0xff;

    pFicDb->aucServiceComponID[ucLoop] 			= 0xff;
    pFicDb->aucServiceExtension[ucLoop] 		= 0xff;
    pFicDb->auiUserAppType[ucLoop] 					= 0xffff;
#ifdef USER_APPLICATION_TYPE	
    pFicDb->astUserAppInfo.uiUserAppType[ucLoop] = 0xffff;
		pFicDb->astUserAppInfo.ucUserAppDataLength[ucLoop] = 0;
#endif

	}
}

ST_FIC_DB* LGD_GETFIC_DB(LGD_UINT8 ucI2CID)
{
	return &g_astFicDb;
}

// by jin : LGD_CHANNEL_INFO ->ST_SUBCH_INFO
//LGD_UINT8 LGD_FIC_UPDATE(LGD_UINT8 ucI2CID, LGD_CHANNEL_INFO* pChInfo)
LGD_UINT8 LGD_FIC_UPDATE(LGD_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
	LGD_INT16	nLoop = 0, nIndex;
	ST_FIC_DB*	pstFicDb;
	ST_BBPINFO* pInfo;
	
	pstFicDb = LGD_GETFIC_DB(ucI2CID);
	pInfo = LGD_GET_STRINFO(ucI2CID);
	
	pInfo->IsChangeEnsemble = FALSE;

	for(nIndex = 0; nIndex < pChInfo->nSetCnt; nIndex++)
	{
		for(nLoop = 0; nLoop < pstFicDb->ucSubChCnt; nLoop++)
		{
				if(pChInfo->astSubChInfo[nIndex].ucSubChID != (pstFicDb->aucSubChID[nLoop]&0x7F)) 
				continue;
			
				if(pChInfo->astSubChInfo[nIndex].uiStarAddr != pstFicDb->auiStartAddr[nLoop]){
					pChInfo->astSubChInfo[nIndex].uiStarAddr = pstFicDb->auiStartAddr[nLoop];
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].uiBitRate != pstFicDb->astSchDb[nLoop].uiBitRate){
					pChInfo->astSubChInfo[nIndex].uiBitRate	= pstFicDb->astSchDb[nLoop].uiBitRate;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].ucSlFlag != pstFicDb->astSchDb[nLoop].ucShortLong){
					pChInfo->astSubChInfo[nIndex].ucSlFlag = pstFicDb->astSchDb[nLoop].ucShortLong;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].ucTableIndex != pstFicDb->astSchDb[nLoop].ucTableIndex){
					pChInfo->astSubChInfo[nIndex].ucTableIndex = pstFicDb->astSchDb[nLoop].ucTableIndex;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].ucOption != pstFicDb->astSchDb[nLoop].ucOption){
					pChInfo->astSubChInfo[nIndex].ucOption = pstFicDb->astSchDb[nLoop].ucOption;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].ucProtectionLevel != pstFicDb->astSchDb[nLoop].ucProtectionLevel){
					pChInfo->astSubChInfo[nIndex].ucProtectionLevel	= pstFicDb->astSchDb[nLoop].ucProtectionLevel;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].uiDifferentRate != pstFicDb->astSchDb[nLoop].uiDifferentRate){
					pChInfo->astSubChInfo[nIndex].uiDifferentRate = pstFicDb->astSchDb[nLoop].uiDifferentRate;
					pInfo->IsChangeEnsemble = TRUE;
				}

				if(pChInfo->astSubChInfo[nIndex].uiSchSize != pstFicDb->astSchDb[nLoop].uiSubChSize){
					pChInfo->astSubChInfo[nIndex].uiSchSize	= pstFicDb->astSchDb[nLoop].uiSubChSize;
					pInfo->IsChangeEnsemble = TRUE;
				}
				break;
		}
	}
	
	if(nLoop == pstFicDb->ucSubChCnt){
		return LGD_ERROR;
	}
	return LGD_SUCCESS;
}


