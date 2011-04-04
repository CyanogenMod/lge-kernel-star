#include <linux/broadcast/broadcast_lg2102_includes.h>

#ifdef LGD_FIFO_SOURCE_ENABLE

#define LGD_GET_SUBCHANNEL_SIZE(X, Y) (((((X)<<8) | (Y)) & 0x3FF) * 4)
#define LGD_GET_SUBCHANNEL_ID(X) (((X) >> 2) & 0x3F)

ST_FIFO		g_astChFifo[MAX_CHANNEL_FIFO];
LGD_UINT8	g_FicFlag = 0;

LGD_UINT8 LGD_QFIFO_INIT(PST_FIFO pFF, LGD_UINT32 ulDepth)
{
	if(pFF == LGD_NULL) return LGD_ERROR;
	pFF->ulFront = pFF->ulRear = 0;

	if(ulDepth == 0 || ulDepth >= LGD_FIFO_DEPTH) pFF->ulDepth = LGD_FIFO_DEPTH + 1; 
	else pFF->ulDepth = ulDepth + 1;
	return LGD_SUCCESS;
}

LGD_UINT32 LGD_QFIFO_FREE_SIZE(PST_FIFO pFF)
{
	if(pFF == LGD_NULL) return LGD_ERROR;

	return (pFF->ulFront >= pFF->ulRear) ?
	  ((pFF->ulRear + pFF->ulDepth) - pFF->ulFront) - 1 : (pFF->ulRear - pFF->ulFront) - 1;
}

LGD_UINT32 LGD_QFIFO_GET_SIZE(PST_FIFO pFF)
{
	if(pFF == LGD_NULL) return LGD_ERROR;
	
	return (pFF->ulFront >= pFF->ulRear) ?
		(pFF->ulFront - pFF->ulRear) : (pFF->ulFront + pFF->ulDepth - pFF->ulRear);
}

LGD_UINT8 LGD_QFIFO_AT(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize)
{
	LGD_UINT32 ulLoop, ulOldRear;
	
	if(pFF == LGD_NULL || pData == LGD_NULL || ulSize > LGD_QFIFO_GET_SIZE(pFF)) 
		return LGD_ERROR;

	ulOldRear = pFF->ulRear;
	/*
	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[ulOldRear++];
		ulOldRear %= pFF->ulDepth;
	}*/

	ulLoop = pFF->ulDepth - pFF->ulRear;
	if(ulLoop >= ulSize)
	{
		memcpy(pData, &pFF->acBuff[ulOldRear], ulSize);
	}
	else
	{
		memcpy(pData, &pFF->acBuff[ulOldRear], ulLoop);
		memcpy(pData + ulLoop, &pFF->acBuff[0], ulSize - ulLoop);
	}
	return LGD_SUCCESS;
}

LGD_UINT8* LGD_QFIFO_AT_PTR(PST_FIFO pFF)
{
	if(pFF == LGD_NULL) 
		return LGD_ERROR;

	return &pFF->acBuff[pFF->ulRear];
}

LGD_UINT8 LGD_QFIFO_ADD(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize)
{
	LGD_UINT32 ulLoop;

	if(pFF == LGD_NULL || pData == LGD_NULL || ulSize > LGD_QFIFO_FREE_SIZE(pFF)) 
		return LGD_ERROR;

	/*
	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pFF->acBuff[pFF->ulFront++] = pData[ulLoop];
		pFF->ulFront %= pFF->ulDepth;
	}
	*/

	ulLoop = pFF->ulDepth - pFF->ulFront;
	if(ulLoop >= ulSize)
	{
		memcpy(&pFF->acBuff[pFF->ulFront], pData, ulSize);
		pFF->ulFront += ulSize;
	}
	else
	{
		memcpy(&pFF->acBuff[pFF->ulFront], pData, ulLoop);
		memcpy(&pFF->acBuff[0], pData + ulLoop, ulSize - ulLoop);
		pFF->ulFront = ulSize - ulLoop;
	}
	return LGD_SUCCESS;
}

LGD_UINT8 LGD_QFIFO_BRING(PST_FIFO pFF, LGD_UINT8* pData, LGD_UINT32 ulSize)
{
	LGD_UINT32 ulLoop;

	if(pFF == LGD_NULL || pData == LGD_NULL || ulSize > LGD_QFIFO_GET_SIZE(pFF)) 
		return LGD_ERROR;

	/*
	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[pFF->ulRear++];
		pFF->ulRear %= pFF->ulDepth;
	}
	*/

	ulLoop = pFF->ulDepth - pFF->ulRear;
	if(ulLoop >= ulSize)
	{
		memcpy(pData, &pFF->acBuff[pFF->ulRear], ulSize);
		pFF->ulRear += ulSize;
	}
	else
	{
		memcpy(pData, &pFF->acBuff[pFF->ulRear], ulLoop);
		memcpy(pData + ulLoop, &pFF->acBuff[0], ulSize - ulLoop);
		pFF->ulRear = ulSize - ulLoop;
	}
	return LGD_SUCCESS;
}

void LGD_MULTI_SORT_INIT(void)
{
	LGD_UINT32 	ulLoop;
	ST_FIFO*	pFifo;

	g_FicFlag = 0;

	for(ulLoop = 0 ; ulLoop < MAX_CHANNEL_FIFO; ulLoop++){
		pFifo = LGD_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)ulLoop);
		LGD_QFIFO_INIT(pFifo, 0);
		pFifo->unSubChID = LGD_SUB_CHANNEL_ID_MASK;
	}
}

ST_FIFO* LGD_GET_CHANNEL_FIFO(MULTI_CHANNEL_INFO ucIndex)
{
	if(ucIndex >= MAX_CHANNEL_FIFO) return LGD_NULL;
	return &g_astChFifo[ucIndex];
}

ST_HEADER_INFO LGD_HEADER_CHECK(ST_FIFO* pMainFifo)
{
	LGD_UINT32 ulLoop,  ulFrame, ulIndex, ulTotalLength, ulSubChTSize;
	LGD_UINT16 aunChSize[MAX_CHANNEL_FIFO-1];
	LGD_UINT8* ptr;

	ulFrame = LGD_QFIFO_GET_SIZE(pMainFifo) / MAX_HEADER_SIZE;

	for(ulIndex = 0; ulIndex < (ulFrame-1); ulIndex++){

		ptr = LGD_QFIFO_AT_PTR(pMainFifo);
		for(ulLoop = 0 ; ulLoop < (MAX_HEADER_SIZE-1); ulLoop++)
		{
			if(ptr[ulLoop] == HEADER_ID_0x33 && ptr[ulLoop+1] == HEADER_ID_0x00)
			{
				if(ulLoop) pMainFifo->ulRear += ulLoop;

				ptr = LGD_QFIFO_AT_PTR(pMainFifo);

				ulTotalLength = ((LGD_UINT16)(ptr[4] << 8) | ptr[5])&0x7FFF;
				
				aunChSize[0] = LGD_GET_SUBCHANNEL_SIZE(ptr[0x8], ptr[0x9]);
				aunChSize[1] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xA], ptr[0xB]);
				aunChSize[2] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xC], ptr[0xD]);
				aunChSize[3] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xE], ptr[0xF]);
				
				ulSubChTSize = aunChSize[0] + aunChSize[1] + aunChSize[2] + aunChSize[3] + MAX_HEADER_SIZE;

				if(ulSubChTSize != ulTotalLength) {
					LGD_QFIFO_INIT(pMainFifo, 0);
					return LGD_HEADER_NOT_SEARACH;
				}

				if(LGD_QFIFO_GET_SIZE(pMainFifo) < (ulTotalLength + MAX_HEADER_SIZE))
					return LGD_HEADER_SIZE_ERROR;

				ptr = LGD_QFIFO_AT_PTR(pMainFifo);
				if(ptr[ulTotalLength] == HEADER_ID_0x33 && ptr[ulTotalLength+1] == HEADER_ID_0x00)
				{
					return LGD_HEADER_GOOD;
				}
				else{
					LGD_QFIFO_INIT(pMainFifo, 0);
					return LGD_HEADER_NOT_SEARACH;
				}
			}
		}

		if(ptr[ulLoop] == HEADER_ID_0x33) pMainFifo->ulRear += MAX_HEADER_SIZE-1;
		else pMainFifo->ulRear += MAX_HEADER_SIZE;
	}

	return LGD_HEADER_NOT_SEARACH;
}

LGD_UINT8 LGD_MULTI_FIFO_PROCESS(LGD_UINT8* pData, LGD_UINT32 ulSize)
{
	LGD_UINT8 cIndex, bIsData = LGD_ERROR;
	LGD_UINT16 aunChSize[MAX_CHANNEL_FIFO-1], unTotalSize=0;
	LGD_UINT16 aunSubChID[MAX_CHANNEL_FIFO-1];

	LGD_UINT8*	ptr;

	ST_FIFO*	pFifo;
	ST_FIFO*	pMainFifo;

	pMainFifo = LGD_GET_CHANNEL_FIFO(MAIN_INPUT_DATA);

	if(pMainFifo->ulDepth<(ulSize+pMainFifo->ulFront))
	{
		unTotalSize = LGD_QFIFO_GET_SIZE(pMainFifo);
		ptr = LGD_QFIFO_AT_PTR(pMainFifo);
		memcpy(pMainFifo->acBuff, ptr, unTotalSize);
		pMainFifo->ulFront = unTotalSize;
		pMainFifo->ulRear = 0;
	}

	LGD_QFIFO_ADD(pMainFifo, pData, ulSize);

	while(1){

		if(LGD_QFIFO_GET_SIZE(pMainFifo) < MAX_HEADER_SIZE) break;
		if(LGD_HEADER_CHECK(pMainFifo) != LGD_HEADER_GOOD) break;

		ptr = LGD_QFIFO_AT_PTR(pMainFifo);
		//헤더의 전체크기를 계산하고...
		unTotalSize = ((LGD_UINT16)(ptr[4] << 8) | ptr[5])&0x7FFF;

		if(unTotalSize > LGD_QFIFO_GET_SIZE(pMainFifo)) break;

		//각각의 채널별 크기를 구하고...
		memset(aunChSize, 0 , sizeof(aunChSize));

		aunChSize[0] = LGD_GET_SUBCHANNEL_SIZE(ptr[0x8], ptr[0x9]);
		aunChSize[1] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xA], ptr[0xB]);
		aunChSize[2] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xC], ptr[0xD]);
		aunChSize[3] = LGD_GET_SUBCHANNEL_SIZE(ptr[0xE], ptr[0xF]);

		aunSubChID[0] = LGD_GET_SUBCHANNEL_ID(ptr[0x8]);
		aunSubChID[1] = LGD_GET_SUBCHANNEL_ID(ptr[0xA]);
		aunSubChID[2] = LGD_GET_SUBCHANNEL_ID(ptr[0xC]);
		aunSubChID[3] = LGD_GET_SUBCHANNEL_ID(ptr[0xE]);

		pMainFifo->ulRear += MAX_HEADER_SIZE;

		if(g_FicFlag==0)
		{
			if(aunChSize[0]==384)
			{
				g_FicFlag = 1;
			}
			printk("[LG2102] g_FicFlag:%d, FicSize:%d, unTotalSize:%d \n",g_FicFlag,aunChSize[0],unTotalSize);
		}
		
		for(cIndex = 0; cIndex < (MAX_CHANNEL_FIFO-1); cIndex++){
			if(!aunChSize[cIndex]) continue;

			pFifo = LGD_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)(cIndex+1));

			if(pFifo->unSubChID == LGD_SUB_CHANNEL_ID_MASK)
					pFifo->unSubChID = aunSubChID[cIndex];
			
			while(aunChSize[cIndex] > LGD_CIF_MAX_SIZE)
			{
				ptr = LGD_QFIFO_AT_PTR(pMainFifo);
				if(g_FicFlag) LGD_QFIFO_ADD(pFifo, ptr, (LGD_UINT32)LGD_CIF_MAX_SIZE);
				pMainFifo->ulRear += LGD_CIF_MAX_SIZE;
				aunChSize[cIndex] -= LGD_CIF_MAX_SIZE;
			}

			ptr = LGD_QFIFO_AT_PTR(pMainFifo);
			if(g_FicFlag) LGD_QFIFO_ADD(pFifo, ptr, (LGD_UINT32)aunChSize[cIndex]);
			pMainFifo->ulRear += aunChSize[cIndex];
		}

		bIsData = LGD_SUCCESS;
	}

	return bIsData;
}


#endif
