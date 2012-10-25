/******************************************************************************** 
* (c) COPYRIGHT 2010 RAONTECH, Inc. ALL RIGHTS RESERVED.
* 
* This software is the property of RAONTECH and is furnished under license by RAONTECH.                
* This software may be used only in accordance with the terms of said license.                         
* This copyright noitce may not be remoced, modified or obliterated without the prior                  
* written permission of RAONTECH, Inc.                                                                 
*                                                                                                      
* This software may not be copied, transmitted, provided to or otherwise made available                
* to any other person, company, corporation or other entity except as specified in the                 
* terms of said license.                                                                               
*                                                                                                      
* No right, title, ownership or other interest in the software is hereby granted or transferred.       
*                                                                                                      
* The information contained herein is subject to change without notice and should 
* not be construed as a commitment by RAONTECH, Inc.                                                                    
* 
* TITLE 	  : RAONTECH TV device driver API header file. 
*
* FILENAME    : raontv.c
*
* DESCRIPTION : 
*		Configuration for RAONTECH TV Services.
*
********************************************************************************/

/******************************************************************************** 
* REVISION HISTORY
*
*    DATE	  	  NAME				REMARKS
* ----------  -------------    --------------------------------------------------
* 09/27/2010  Ko, Kevin        Creat for CS Realease
*             /Yang, Maverick  1.Reformating for CS API
*                              2.pll table, ADC clock switching, SCAN function, 
*								 FM function added..
********************************************************************************/

#include "raontv_rf.h"

volatile BOOL g_afRtvChannelChange[NUM_ATTECHED_RTV_CHIP];


volatile E_RTV_ADC_CLK_FREQ_TYPE g_aeRtvAdcClkFreqType[NUM_ATTECHED_RTV_CHIP];
BOOL g_afRtvStreamEnabled[NUM_ATTECHED_RTV_CHIP];

#if defined(RTV_TDMB_ENABLE) || defined(RTV_ISDBT_ENABLE)
	E_RTV_COUNTRY_BAND_TYPE g_eRtvCountryBandType;
#endif

#ifdef RTV_IF_EBI2
	VU8 g_bRtvEbiMapSelData = 0x7;
#endif

#if defined(RTV_IF_SPI) || defined(RTV_IF_EBI2)
	UINT g_aRtvMscThresholdSize[NUM_ATTECHED_RTV_CHIP]; // For ISDBT and FM.
	U8 g_abRtvIntrMaskRegL[NUM_ATTECHED_RTV_CHIP];

#else	
   #if defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE) /* Single sub channel mode */
	U8 g_abRtvIntrMaskRegL[NUM_ATTECHED_RTV_CHIP];
   #endif
#endif	


#ifdef RTV_DUAL_CHIP_USED /* Diversity feature */
	BOOL g_fRtvDiversityEnabled;
	UINT RaonTvChipIdx = 0xFF; /* To error check, the default value is a invalid value. */
#endif	


void rtv_ConfigureHostIF(void)
{
#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF) || defined(RTV_IF_SPI_SLAVE)
	RTV_REG_MAP_SEL(HOST_PAGE);
    RTV_REG_SET(0x77, 0x15);   // TSIF Enable
    RTV_REG_SET(0x22, 0x48);   

  #if defined(RTV_IF_MPEG2_PARALLEL_TSIF)
	RTV_REG_SET(0x04, 0x01);   // I2C + TSIF Mode Enable
  #else
	RTV_REG_SET(0x04, 0x29);   // I2C + TSIF Mode Enable
  #endif
  
	RTV_REG_SET(0x0C, 0xF4);   // TSIF Enable

#elif defined(RTV_IF_SPI) || defined(RTV_IF_EBI2)
	RTV_REG_MAP_SEL(HOST_PAGE);
	RTV_REG_SET(0x77, 0x14);   //SPI Mode Enable
    RTV_REG_SET(0x04, 0x28);   // SPI Mode Enable
	RTV_REG_SET(0x0C, 0xF5);
 
#else
	#error "Code not present"
#endif
}

INT rtv_InitSystem(E_RTV_TV_MODE_TYPE eTvMode, E_RTV_ADC_CLK_FREQ_TYPE eAdcClkFreqType)
{
	INT nRet;
	int i;
	
	g_afRtvChannelChange[RaonTvChipIdx] = FALSE;
	
	g_afRtvStreamEnabled[RaonTvChipIdx] = FALSE;

#if defined(RTV_IF_SPI) || defined(RTV_IF_EBI2)
	g_abRtvIntrMaskRegL[RaonTvChipIdx] = 0xFF;
#else	
   #if defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE) /* Single sub channel mode */
	g_abRtvIntrMaskRegL[RaonTvChipIdx] = 0xFF;
   #endif
#endif	

	/* In case of SPI interface or FIC interrupt mode for T-DMB, we should lock the register page. */
	RTV_GUARD_INIT;

	for(i=0; i<100; i++)
	{
//RTV_DBGMSG0("[rtv_InitSystem] 1\n");

		RTV_REG_MAP_SEL(HOST_PAGE);

//RTV_DBGMSG0("[rtv_InitSystem] 2\n");
		

		RTV_REG_SET(0x7D, 0x06);
		if(RTV_REG_GET(0x7D) == 0x06)
		{
			goto RTV_POWER_ON_SUCCESS;
		}

		RTV_DBGMSG1("[rtv_InitSystem] Power On wait: %d\n", i);

		RTV_DELAY_MS(5);
	}

	RTV_DBGMSG1("rtv_InitSystem: Power On Check error: %d\n", i);
	return RTV_POWER_ON_CHECK_ERROR;

RTV_POWER_ON_SUCCESS:
	
	rtvRF_ConfigurePowerType(eTvMode);

	if((nRet=rtvRF_ConfigureAdcClock(eTvMode, eAdcClkFreqType)) != RTV_SUCCESS)
		return nRet;
		
	return RTV_SUCCESS;
}





