
//#include "StdAfx.h"
#include "../inc/INC_INCLUDES.h"

#ifdef INC_FIFO_SOURCE_ENABLE

#define INC_GET_SUBCHANNEL_SIZE(X, Y) ((((X)<<8) | (Y)) & 0x3FF) * 2
#define INC_GET_SUBCHANNEL_ID(X) (((X) >> 2) & 0x3F)
ST_FIFO g_astChFifo[MAX_CHANNEL_FIFO];
INC_UINT8 g_acCheckBuff[INC_HEADER_CHECK_BUFF];



INC_UINT8 INC_QFIFO_INIT(PST_FIFO pFF, INC_UINT32 ulDepth)
{
	if(pFF == INC_NULL) return INC_ERROR;

	pFF->ulFront = pFF->ulRear = 0;

	if(ulDepth == 0 || ulDepth >= INC_FIFO_DEPTH) pFF->ulDepth = INC_FIFO_DEPTH + 1;
	else pFF->ulDepth = ulDepth + 1;
	return INC_SUCCESS;
}

INC_UINT32 INC_QFIFO_FREE_SIZE(PST_FIFO pFF)
{
	if(pFF == INC_NULL) return INC_ERROR;

	return (pFF->ulFront >= pFF->ulRear) ?
	  ((pFF->ulRear + pFF->ulDepth) - pFF->ulFront) - 1 : (pFF->ulRear - pFF->ulFront) - 1;
}

INC_UINT32 INC_QFIFO_GET_SIZE(PST_FIFO pFF)
{
	if(pFF == INC_NULL) return INC_ERROR;
	
	return (pFF->ulFront >= pFF->ulRear) ?
		(pFF->ulFront - pFF->ulRear) : (pFF->ulFront + pFF->ulDepth - pFF->ulRear);
}

INC_UINT8 INC_QFIFO_AT(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop, ulOldRear;
	
	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_GET_SIZE(pFF)) 
		return INC_ERROR;

	ulOldRear = pFF->ulRear;
	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[ulOldRear++];
		ulOldRear %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

INC_UINT8 INC_QFIFO_ADD(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop;

	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_FREE_SIZE(pFF)) 
		return INC_ERROR;

	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pFF->acBuff[pFF->ulFront++] = pData[ulLoop];
		pFF->ulFront %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

INC_UINT8 INC_QFIFO_BRING(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop;

	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_GET_SIZE(pFF)) 
		return INC_ERROR;

	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[pFF->ulRear++];
		pFF->ulRear %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

void INC_MULTI_SORT_INIT(void)
{
	INC_UINT32 ulLoop;
	ST_FIFO* pFifo;

	for(ulLoop = 0 ; ulLoop < MAX_CHANNEL_FIFO; ulLoop++){
		pFifo = INC_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)ulLoop);
		INC_QFIFO_INIT(pFifo, 0);
		pFifo->unSubChID = INC_SUB_CHANNEL_ID_MASK;
	}
}

ST_FIFO* INC_GET_CHANNEL_FIFO(MULTI_CHANNEL_INFO ucIndex)
{
	if(ucIndex >= MAX_CHANNEL_FIFO) return INC_NULL;
	return &g_astChFifo[ucIndex];
}

ST_HEADER_INFO INC_HEADER_CHECK(ST_FIFO* pMainFifo)
{
	INC_UINT32 ulSize, ulTotalLength, ulSubChTSize;
	INC_UINT16 aunChSize[MAX_CHANNEL_FIFO-1];
	INC_UINT16 aunSubChID[MAX_CHANNEL_FIFO-1];
	INC_UINT8 cIndex;
	ST_HEADER_INFO isData = INC_HEADER_NOT_SEARACH;
	ST_FIFO* pFifo;

	while(1){

		ulSize = INC_QFIFO_GET_SIZE(pMainFifo);
		if(ulSize < MAX_HEADER_SIZE) break;

		INC_QFIFO_AT(pMainFifo, g_acCheckBuff, 2);
		if(g_acCheckBuff[0] == HEADER_ID_0x33 && g_acCheckBuff[1] == HEADER_ID_0x00)
		{
			INC_QFIFO_AT(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE);

			ulTotalLength = (INC_UINT16)(g_acCheckBuff[4] << 8) | g_acCheckBuff[5];
			ulTotalLength = (ulTotalLength & 0x8000) ? (ulTotalLength << 1) : ulTotalLength;

			aunChSize[0] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0x8], g_acCheckBuff[0x9]);
			aunChSize[1] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xA], g_acCheckBuff[0xB]) | (aunChSize[0] & 0xC00);
			aunChSize[2] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xC], g_acCheckBuff[0xD]) | ((aunChSize[0] >> 2) & 0xC00);
			aunChSize[3] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xE], g_acCheckBuff[0xF]);

			aunSubChID[0] = INC_GET_SUBCHANNEL_ID(g_acCheckBuff[0x8]);
			aunSubChID[1] = INC_GET_SUBCHANNEL_ID(g_acCheckBuff[0xA]);
			aunSubChID[2] = INC_GET_SUBCHANNEL_ID(g_acCheckBuff[0xC]);
			aunSubChID[3] = INC_GET_SUBCHANNEL_ID(g_acCheckBuff[0xE]);

			ulSubChTSize = aunChSize[0] + aunChSize[1] + aunChSize[2] + aunChSize[3] + MAX_HEADER_SIZE;

			if(ulSubChTSize != ulTotalLength) {
				INC_QFIFO_INIT(pMainFifo, 0);
				break;
			}

			if(INC_QFIFO_GET_SIZE(pMainFifo) < (ulTotalLength + MAX_HEADER_SIZE))
				break;

			memset(g_acCheckBuff,0, sizeof(g_acCheckBuff));
			INC_QFIFO_AT(pMainFifo, g_acCheckBuff, ulTotalLength + MAX_HEADER_SIZE);

			if(g_acCheckBuff[ulTotalLength] == HEADER_ID_0x33 && g_acCheckBuff[ulTotalLength+1] == HEADER_ID_0x00)
			{
				INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE);

				for(cIndex = 0; cIndex < (MAX_CHANNEL_FIFO-1); cIndex++){
					if(!aunChSize[cIndex]) continue;

					pFifo = INC_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)(cIndex+1));

					if(pFifo->unSubChID == INC_SUB_CHANNEL_ID_MASK)
						pFifo->unSubChID = aunSubChID[cIndex];

					while(aunChSize[cIndex] > INC_CIF_MAX_SIZE)
					{
						INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, (INC_UINT32)INC_CIF_MAX_SIZE);
						INC_QFIFO_ADD(pFifo, g_acCheckBuff, (INC_UINT32)INC_CIF_MAX_SIZE);
						aunChSize[cIndex] -= INC_CIF_MAX_SIZE;
					}

					INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, (INC_UINT32)aunChSize[cIndex]);
					INC_QFIFO_ADD(pFifo, g_acCheckBuff, (INC_UINT32)aunChSize[cIndex]);
				}
				isData = INC_HEADER_GOOD;
			}
			else{	
				INC_QFIFO_INIT(pMainFifo, 0);
			}
		}
		else{
			if(g_acCheckBuff[1] == HEADER_ID_0x33) INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, 1);
			else INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, 2);
		}
	}

	return isData;
}

INC_UINT8 INC_MULTI_FIFO_PROCESS(INC_UINT8* pData, INC_UINT32 ulSize)
{
	ST_FIFO* pMainFifo;

	pMainFifo = INC_GET_CHANNEL_FIFO(MAIN_INPUT_DATA);

	if(INC_QFIFO_ADD(pMainFifo, pData, ulSize) != INC_SUCCESS){

		INC_QFIFO_INIT(pMainFifo, 0);
		return INC_ERROR;
	}

	if(INC_HEADER_CHECK(pMainFifo) == INC_HEADER_GOOD){
		return INC_SUCCESS;
	}

	return INC_ERROR;
}



#endif
