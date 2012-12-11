/*
 * The header file for X2 NV
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_TEGRA_X2_NV_H
#define __MACH_TEGRA_X2_NV_H

#define LGE_NVDATA_PARTITION			"/dev/block/mmcblk0p3"


// modify this file and /android/vendor/lge/tegra/core/system/fastboot/lge_boot/inc/lge_boot_utils.h
typedef enum{
	LGE_NVDATA_REMOVE_FAT_OFFSET = 510,

	LGE_NVDATA_RESET_CAUSE_OFFSET 	= 512,  // used size 1 byte
	LGE_NVDATA_CRASH_DUMP_OFFSET 	= 514,  // used size 1 byte
	LGE_NVDATA_AP_CRASH_DUMP_OFFSET = 515,  // used size 1 byte
	LGE_NVDATA_FORCE_CRASH_OFFSET 	= 516,  // used size 1 byte

	LGE_NVDATA_FACTORY_RESET_STATUS_OFFSET 	= 518,  // used size 1 byte, for at%frst & at%frstatus
	LGE_NVDATA_FBOOT_OFFSET 	= 520,  // used size 1 byte, for at%fboot
	//                                                                                 

// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT
	LGE_NVDATA_CP_CRASH_COUNT_OFFSET = 523,  // used size 1 byte
// CHEOLGWAK  2011-2-26 	CP_CRASH_COUNT

	//                                                    
	LGE_NVDATA_CIQ_NVDATA_RESET_OFFSET = 524, // used size 2 byte //RESET SIDE & CAUSE
	//                                                  

	//                                   
	LGE_NVDATA_HARD_RESET_OFFSET = 526, 
	//     

	LGE_NVDATA_RTC_INIT_OFFSET = 528,
	LGE_NVDATA_FRSTSTATUS3_OFFSET = 530,//11.07.23 if set, must jump to recovery mode.

	LGE_NVDATA_910K_DETECT_OFFSET	= 1024,  // used size 1 byte

	LGE_NVDATA_QEM_OFFSET			= 2048, //length 4    ==> move to static nvdata
	LGE_NVDATA_DEVICETEST_OFFSET	= 2052, //length 8     ==> move to static nvdata
	LGE_NVDATA_DEVICDTEST_DATE_OFFSET = 2060,//length 8  ==> move to static nvdata	

	LGE_NVDATA_MUIC_RETENTION_OFFSET = 2560, // used size 1 byte

	LGE_NVDATA_SMPL_EN_OFFSET	= 2570,	// used size 1 byte
	LGE_NVDATA_SMPL_COUNT_OFFSET	= 2572,	// used size 4 byte

	LGE_NVDATA_WEB_DOWNLOAD_OFFSET1 = 2580, // used size 1 byte	
	LGE_NVDATA_WEB_DOWNLOAD_OFFSET2 = 2582,	 // used size 1 byte	

	LGE_NVDATA_CHARGING_TEMP_OFFSET	= 2600,	// used size 1 byte

//MOBII_CHNANGE_S 20120619 mg.chang@mobii.co.kr : for AAT
	DEVICETEST_NVDATA_OFFSET = 2650,
	DEVICETEST_NVDATA_DATE_OFFSET = 2800,
	DEVICETEST_NVDATA_OFFSET_PARTIAL = 3680,
	DEVICETEST_NVDATA_DATE_OFFSE_PARTIALT = 3000,
//MOBII_CHNANGE_E 20120619 mg.chang@mobii.co.kr : for AAT

	LGE_NVDATA_WARMBOOT_CHECK_OFFSET = 3500,
	
	LGE_NVDATA_MAX_FASTBOOT_OFFSET = 4096,	// MAX OFFSET for FASTBOOT write NV
	/* MOBII CHANGE [jg.noh@mobii.co.kr] MISC is located behind NV Data. Don't add below here. */
}lge_nvdata_offset;

// Value define for nv data
#define LGE_NVDATA_RESET_CAUSE_VAL_USER_RESET	0x94
#define LGE_NVDATA_RESET_CAUSE_VAL_AP_CRASH		0xDE
#define LGE_NVDATA_RESET_CAUSE_VAL_CP_CRASH		0xAD
#define LGE_NVDATA_RESET_CAUSE_FACTORY_RESET	0x46

// WEB DOWNLOAD [START]
#define LGE_NVDATA_RESET_CAUSE_WEB_DOWNLOAD_RESET1	0x11
#define LGE_NVDATA_RESET_CAUSE_WEB_DOWNLOAD_RESET2	0x22
// WEB DOWNLOAD [END]

#define LGE_NDATA_CRASH_DUMP_INITIAL_VALUE	0x00
#define LGE_NDATA_CRASH_DUMP_ENABLE_VALUE	0xA5
#define LGE_NDATA_CRASH_DUMP_DISABLE_VALUE	0xB3


// err return code
#define LGE_NVDATA_EMMC_ERR_SIZE_TOO_SMALL	-1;  
#define LGE_NVDATA_EMMC_ERR_SIZE_TOO_LARGE	-2; 
// modify this file and /android/vendor/lge/tegra/core/system/fastboot/lge_boot/inc/lge_boot_utils.h

extern int lge_nvdata_read(lge_nvdata_offset offset, char* buf, int size);
extern int lge_nvdata_write(lge_nvdata_offset offset, char* buf, int size);

#endif /* __MACH_TEGRA_X2_NV_H */
