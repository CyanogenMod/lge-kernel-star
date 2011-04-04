
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
// Temperature Monitor (TMON)
{
    NV_ODM_GUID('a','d','t','7','4','6','1',' '),
    s_lge_Tmon0Addresses,
    NV_ARRAY_SIZE(s_lge_Tmon0Addresses),
    NvOdmPeripheralClass_Other
},
#elif defined(CONFIG_MACH_STAR_REV_F)	// modified in LGP990 revF
// Temperature Monitor (TMON)
{
    NV_ODM_GUID('a','d','t','7','4','6','1',' '),
    s_lge_Tmon0Addresses,
    NV_ARRAY_SIZE(s_lge_Tmon0Addresses),
    NvOdmPeripheralClass_Other
},
#endif

#if defined(CONFIG_MACH_STAR)
//20100413, cs77.ha@lge.com, power off [START]
//POWER OFF
{
    NV_VDD_SoC_ODM_ID,
    s_lge_SocOdmAddresses,
    NV_ARRAY_SIZE(s_lge_SocOdmAddresses),
    NvOdmPeripheralClass_Other
},
//20100413, cs77.ha@lge.com, power off [END]

//20100703, cs77.ha@lge.com, PMIC reset [START]
{
    NV_ODM_GUID('p','m','_','r','e','s','e','t'),
    s_lge_PowerResetAddresses,
    NV_ARRAY_SIZE(s_lge_PowerResetAddresses),
    NvOdmPeripheralClass_Other
},
//20100703, cs77.ha@lge.com, PMIC reset [END]
#endif

//20100413, cs77.ha@lge.com, powerkey [START]
#if defined(CONFIG_MACH_STAR)
{
    NV_ODM_GUID('p','o','w','e','r','k','e','y'),
    s_lge_PowerKeyAddresses,
    NV_ARRAY_SIZE(s_lge_PowerKeyAddresses),
    NvOdmPeripheralClass_Other
},
#endif
//20100413, cs77.ha@lge.com, powerkey [END]

//20101129, hyeongwon.oh@lge.com, SU660 homekey [START]
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F) 
{
    NV_ODM_GUID('h','o','m','e','-','k','e','y'),
    s_lge_HomeKeyAddresses,
    NV_ARRAY_SIZE(s_lge_HomeKeyAddresses),
    NvOdmPeripheralClass_Other
},
#endif
#endif
//20101129, hyeongwon.oh@lge.com, SU660 homekey [END]

// LCD module
{
#if defined(CONFIG_MACH_STAR)
    NV_ODM_GUID('h','i','t','a','c','c','p','u'),  //4.0" Hitachi wvga.  
#else
    NV_ODM_GUID('h','i','t','a','c','d','s','i'),  //4.0" Hitachi wvga.  
#endif
    s_lge_MainDisplayAddresses,
    NV_ARRAY_SIZE(s_lge_MainDisplayAddresses),
    NvOdmPeripheralClass_Display
},

// HDMI
{
    NV_ODM_GUID('l','g','_','_','h','d','m','i'),
    s_lge_HdmiAddresses,
    NV_ARRAY_SIZE(s_lge_HdmiAddresses),
    NvOdmPeripheralClass_Display
},

// TV Out Video Dac
{
    NV_ODM_GUID('l','g','_','t','v','o','u','t'),
    s_lge_VideoDacAddresses,
    NV_ARRAY_SIZE(s_lge_VideoDacAddresses),
    NvOdmPeripheralClass_Display
},

// RTC (NV reserved)
{
    NV_VDD_RTC_ODM_ID,
    s_lge_RtcAddresses,
    NV_ARRAY_SIZE(s_lge_RtcAddresses),
    NvOdmPeripheralClass_Other
},

// CORE (NV reserved)
{
    NV_VDD_CORE_ODM_ID,
    s_lge_CoreAddresses,
    NV_ARRAY_SIZE(s_lge_CoreAddresses),
    NvOdmPeripheralClass_Other
},

// CPU (NV reserved)
{
    NV_VDD_CPU_ODM_ID,
    s_lge_CpuAddresses,
    NV_ARRAY_SIZE(s_lge_CpuAddresses),
    NvOdmPeripheralClass_Other
},

// PLLA (NV reserved)
{
    NV_VDD_PLLA_ODM_ID,
    s_lge_PllAAddresses,
    NV_ARRAY_SIZE(s_lge_PllAAddresses),
    NvOdmPeripheralClass_Other
},

// PLLM (NV reserved)
{
    NV_VDD_PLLM_ODM_ID,
    s_lge_PllMAddresses,
    NV_ARRAY_SIZE(s_lge_PllMAddresses),
    NvOdmPeripheralClass_Other
},

// PLLP (NV reserved)
{
    NV_VDD_PLLP_ODM_ID,
    s_lge_PllPAddresses,
    NV_ARRAY_SIZE(s_lge_PllPAddresses),
    NvOdmPeripheralClass_Other
},


{
    NV_VDD_PLLD_ODM_ID,
    s_lge_PllDAddresses,
    NV_ARRAY_SIZE(s_lge_PllDAddresses),
    NvOdmPeripheralClass_Other
},


// PLLC (NV reserved)
{
    NV_VDD_PLLC_ODM_ID,
    s_lge_PllCAddresses,
    NV_ARRAY_SIZE(s_lge_PllCAddresses),
    NvOdmPeripheralClass_Other
},

// PLLE (NV reserved)
{
    NV_VDD_PLLE_ODM_ID,
    s_lge_PllEAddresses,
    NV_ARRAY_SIZE(s_lge_PllEAddresses),
    NvOdmPeripheralClass_Other
},

// PLLU (NV reserved)
{
    NV_VDD_PLLU_ODM_ID,
    s_lge_PllUsbAddresses,
    NV_ARRAY_SIZE(s_lge_PllUsbAddresses),
    NvOdmPeripheralClass_Other
},

// PLLU1 (NV reserved)
{
    NV_VDD_PLLU1_ODM_ID,
    s_lge_PllU1Addresses,
    NV_ARRAY_SIZE(s_lge_PllU1Addresses),
    NvOdmPeripheralClass_Other
},

// PLLS (NV reserved)
{
    NV_VDD_PLLS_ODM_ID,
    s_lge_PllSAddresses,
    NV_ARRAY_SIZE(s_lge_PllSAddresses),
    NvOdmPeripheralClass_Other
},

// HDMI PLL (NV reserved)
{
    NV_VDD_PLLHDMI_ODM_ID,
    s_lge_PllHdmiAddresses,
    NV_ARRAY_SIZE(s_lge_PllHdmiAddresses),
    NvOdmPeripheralClass_Other
},

// PLLX (NV reserved)
{
    NV_VDD_PLLX_ODM_ID,
    s_lge_PllXAddresses,
    NV_ARRAY_SIZE(s_lge_PllXAddresses),
    NvOdmPeripheralClass_Other
},

// OSC VDD (NV reserved)
{
    NV_VDD_OSC_ODM_ID,
    s_lge_VddOscAddresses,
    NV_ARRAY_SIZE(s_lge_VddOscAddresses),
    NvOdmPeripheralClass_Other
},

// PLL_USB (NV reserved)
{
    NV_VDD_PLL_USB_ODM_ID,
    s_lge_PllUsbAddresses,
    NV_ARRAY_SIZE(s_lge_PllUsbAddresses),
    NvOdmPeripheralClass_Other
},

// (TBD) PLL_PEX (NV reserved)

// System IO VDD (NV reserved)
{
    NV_VDD_SYS_ODM_ID,
    s_lge_VddSysAddresses,
    NV_ARRAY_SIZE(s_lge_VddSysAddresses),
    NvOdmPeripheralClass_Other
},

// USB VDD (NV reserved)
{
    NV_VDD_USB_ODM_ID,
    s_lge_VddUsbAddresses,
    NV_ARRAY_SIZE(s_lge_VddUsbAddresses),
    NvOdmPeripheralClass_Other
},

//  Fuse
{
    NV_VDD_FUSE_ODM_ID,
    s_lge_VddFuseAddresses,
    NV_ARRAY_SIZE(s_lge_VddFuseAddresses),
    NvOdmPeripheralClass_Other
},

// HDMI VDD (NV reserved)
{
    NV_VDD_HDMI_ODM_ID,
    s_lge_VddHdmiAddresses,
    NV_ARRAY_SIZE(s_lge_VddHdmiAddresses),
    NvOdmPeripheralClass_Other
},

// MIPI VDD (NV reserved)
{
    NV_VDD_MIPI_ODM_ID,
    s_lge_VddMipiAddresses,
    NV_ARRAY_SIZE(s_lge_VddMipiAddresses),
    NvOdmPeripheralClass_Other
},

// LCD VDD (NV reserved)
{
    NV_VDD_LCD_ODM_ID,
    s_lge_VddLcdAddresses,
    NV_ARRAY_SIZE(s_lge_VddLcdAddresses),
    NvOdmPeripheralClass_Other
},

// AUDIO VDD (NV reserved)
{
    NV_VDD_AUD_ODM_ID,
    s_lge_VddAudAddresses,
    NV_ARRAY_SIZE(s_lge_VddAudAddresses),
    NvOdmPeripheralClass_Other
},

// DDR VDD (NV reserved)
{
    NV_VDD_DDR_ODM_ID,
    s_lge_VddDdrAddresses,
    NV_ARRAY_SIZE(s_lge_VddDdrAddresses),
    NvOdmPeripheralClass_Other
},

// DDR_RX (NV reserved)
{
    NV_VDD_DDR_RX_ODM_ID,
    s_lge_VddDdrRxAddresses,
    NV_ARRAY_SIZE(s_lge_VddDdrRxAddresses),
    NvOdmPeripheralClass_Other
},

// NAND VDD (NV reserved)
{
    NV_VDD_NAND_ODM_ID,
    s_lge_VddNandAddresses,
    NV_ARRAY_SIZE(s_lge_VddNandAddresses),
    NvOdmPeripheralClass_Other
},

// UART VDD (NV reserved)
{
    NV_VDD_UART_ODM_ID,
    s_lge_VddUartAddresses,
    NV_ARRAY_SIZE(s_lge_VddUartAddresses),
    NvOdmPeripheralClass_Other
},

// SDIO VDD (NV reserved)
{
    NV_VDD_SDIO_ODM_ID,
    s_lge_VddSdioAddresses,
    NV_ARRAY_SIZE(s_lge_VddSdioAddresses),
    NvOdmPeripheralClass_Other
},

// VDAC VDD (NV reserved)
{
    NV_VDD_VDAC_ODM_ID,
    s_lge_VddVdacAddresses,
    NV_ARRAY_SIZE(s_lge_VddVdacAddresses),
    NvOdmPeripheralClass_Other
},

// VI VDD (NV reserved)
{
    NV_VDD_VI_ODM_ID,
    s_lge_VddViAddresses,
    NV_ARRAY_SIZE(s_lge_VddViAddresses),
    NvOdmPeripheralClass_Other
},

// BB VDD (NV reserved)
{
    NV_VDD_BB_ODM_ID,
    s_lge_VddBbAddresses,
    NV_ARRAY_SIZE(s_lge_VddBbAddresses),
    NvOdmPeripheralClass_Other
},

// HSIC (NV reserved)
{
    NV_VDD_HSIC_ODM_ID,
    s_lge_VddHsicAddresses,
    NV_ARRAY_SIZE(s_lge_VddHsicAddresses),
    NvOdmPeripheralClass_Other
},

// USB_IC (NV reserved)
{
    NV_VDD_USB_IC_ODM_ID,
    s_lge_VddUsbIcAddresses,
    NV_ARRAY_SIZE(s_lge_VddUsbIcAddresses),
    NvOdmPeripheralClass_Other
},

// (TBD) PEX (NV reserved)

//  PMU0
{
    NV_ODM_GUID('m','a','x','8','9','0','7','_'),
    s_lge_Pmu0Addresses,
    NV_ARRAY_SIZE(s_lge_Pmu0Addresses),
    NvOdmPeripheralClass_Other
},

//  VBUS for USB1
{
    NV_VDD_VBUS_ODM_ID,
    s_lge_VddUsb1VBusAddresses,
    NV_ARRAY_SIZE(s_lge_VddUsb1VBusAddresses),
    NvOdmPeripheralClass_Other
},

//  VBUS for USB3
{
    NV_VDD_USB3_VBUS_ODM_ID,
    s_lge_VddUsb3VBusAddresses,
    NV_ARRAY_SIZE(s_lge_VddUsb3VBusAddresses),
    NvOdmPeripheralClass_Other
},

//  Sdio module
{
    NV_ODM_GUID('s','d','i','o','_','m','e','m'),
    s_lge_SdioAddresses,
    NV_ARRAY_SIZE(s_lge_SdioAddresses),
    NvOdmPeripheralClass_Other,
},

//..Vibrate Module
{
    NV_ODM_GUID('v','i','b','r','a','t','o','r'),
    s_lge_VibAddresses,
    NV_ARRAY_SIZE(s_lge_VibAddresses),
    NvOdmPeripheralClass_Other,
},

// Accelerometer sk.hwang@lge.com
{
    NV_ODM_GUID('a','c','c','e','l','e','r','o'),
    s_lge_AccelerometerAddresses,
    NV_ARRAY_SIZE(s_lge_AccelerometerAddresses),
    NvOdmPeripheralClass_Other
},

// Compass sk.hwang@lge.com 
{
	NV_ODM_GUID('c','o','m','p','a','s','s','-'),
    s_lge_CompassAddresses,
    NV_ARRAY_SIZE(s_lge_CompassAddresses),
    NvOdmPeripheralClass_Other
},

// Gyroscope
{
    NV_ODM_GUID('g','y','r','o','s','c','o','p'),
	s_lge_GyroAddresses,
	NV_ARRAY_SIZE(s_lge_GyroAddresses),
    NvOdmPeripheralClass_Other
},

// Proximity sk.hwang@lge.com 
{
    NV_ODM_GUID('p','r','o','x','i','m','i','t'),
    s_lge_ProximityAddresses,
    NV_ARRAY_SIZE(s_lge_ProximityAddresses),
    NvOdmPeripheralClass_Other
},

//  audio codec
{
    NV_ODM_GUID('w','o','l','f','8','9','9','4'),
    s_lge_AudioCodecAddresses,
    NV_ARRAY_SIZE(s_lge_AudioCodecAddresses),
    NvOdmPeripheralClass_Other
},

// 20100927 hyeongwon.oh@lge.com Synaptics OneTouch support [START]
//	Touch Panel
{
// Synaptics touch is used, GUID needs to be changed accordingly
	NV_ODM_GUID('o','n','e','t','o','u','c','h'),
	s_lge_SynapticsOneTouchAddresses,
	NV_ARRAY_SIZE(s_lge_SynapticsOneTouchAddresses),
	NvOdmPeripheralClass_HCI
},
// 20100927 hyeongwon.oh@lge.com Synaptics OneTouch support [END]

// 20100527 joseph.jung@lge.com Synaptics/Cypress Touch support [START]
//	Touch Panel
{
// Synaptics touch is used, GUID needs to be changed accordingly
	NV_ODM_GUID('s','y','n','t','o','u','c','h'),
	s_lge_SynapticsTouchAddresses,
	NV_ARRAY_SIZE(s_lge_SynapticsTouchAddresses),
	NvOdmPeripheralClass_HCI
},

{
// Cypress touch is used, GUID needs to be changed accordingly
	NV_ODM_GUID('c','y','p','t','o','u','c','h'),
	s_lge_CypressTouchAddresses,
	NV_ARRAY_SIZE(s_lge_CypressTouchAddresses),
	NvOdmPeripheralClass_HCI
},
// 20100527 joseph.jung@lge.com Synaptics/Cypress Touch support [END]

//20100609, jh.ahn@lge.com, Write the description here in detail [START]
#if defined(CONFIG_MACH_STAR)
{
    NV_ODM_GUID('c','h','a','r','g','i','n','g'),
    s_lge_ChargerAddresses,
    NV_ARRAY_SIZE(s_lge_ChargerAddresses),
    NvOdmPeripheralClass_Other
},
#endif
//20100609, jh.ahn@lge.com, Write the description here in detail [END]

// 20100401 taewan.kim@lge.com MUIC driver [START]
#if defined(CONFIG_MACH_STAR)
{
    NV_ODM_GUID('s','t','a','r','m','u','i','c'),
    s_lge_MuicAddresses,
    NV_ARRAY_SIZE(s_lge_MuicAddresses),
    NvOdmPeripheralClass_Other
},
#endif
// 20100401 taewan.kim@lge.com MUIC driver [END]

//20100424, bojung.ko@lge.com, RIL code from firenze [START]
#if defined(CONFIG_MACH_STAR)
{
    NV_ODM_GUID('s','t','a','r','-','s','p','i'),
    s_lge_SpiAddresses,
    NV_ARRAY_SIZE(s_lge_SpiAddresses),
    NvOdmPeripheralClass_Other
},
#endif

//  AAT2870 backlight IC, GUID needs to be changed accordingly
{
    NV_ODM_GUID('b','l','b','d','6','0','8','4'),
    s_lge_BackLightAddresses,
    NV_ARRAY_SIZE(s_lge_BackLightAddresses),
    NvOdmPeripheralClass_Other
},

// Sdio wlan  on COMMs Module
{
    NV_ODM_GUID('s','d','i','o','w','l','a','n'),
    s_lge_WlanAddresses,
    NV_ARRAY_SIZE(s_lge_WlanAddresses),
    NvOdmPeripheralClass_Other
},

// Bluetooth on COMMs Module
{
     NV_ODM_GUID('b','l','u','t','o','o','t','h'),
     s_lge_BluetoothAddresses,
     NV_ARRAY_SIZE(s_lge_BluetoothAddresses),
     NvOdmPeripheralClass_Other
},

// Key Pad
{
      NV_ODM_GUID('k','e','y','b','o','a','r','d'),
      s_lge_KeyPadAddresses,
      NV_ARRAY_SIZE(s_lge_KeyPadAddresses),
      NvOdmIoModule_Kbd
},

// Headset Driver Module
//20100421 bergkamp.cho@lge.com for headset driver [LGE_START]
#if defined(CONFIG_MACH_STAR)
{
    NV_ODM_GUID('h','e','a','d', 's', 'e', 't', '_'),
    s_lge_HeadsetAddresses,
    NV_ARRAY_SIZE(s_lge_HeadsetAddresses),
    NvOdmPeripheralClass_Other
},
#endif /* CONFIG_MACH_STAR */
//20100421 bergkamp.cho@lge.com for headset driver [LGE_END]

//LGE_UPDATE_S neo.shin@lge.com 2010-05-024 GPS UART & GPIO Setting
//BCM4751-GPS
{
	NV_ODM_GUID('N','V','O','D','M','G','P','S'),
	s_lge_GPSAddresses,
	NV_ARRAY_SIZE(s_lge_GPSAddresses),
	NvOdmPeripheralClass_Other
},
//LGE_UPDATE_E neo.shin@lge.com 2010-05-024 GPS UART & GPIO Setting

//20100611, cs77.ha@lge.com, Touch LED [START]
{
    NV_ODM_GUID('t','o','u','c','h','L','E','D'), 
    s_lge_TouchLEDAddresses,
    NV_ARRAY_SIZE(s_lge_TouchLEDAddresses),
    NvOdmPeripheralClass_Other
},
//20100611, cs77.ha@lge.com, Touch LED [END]

//20100603, cs77.ha@lge.com, star pmic [START]
#ifdef CONFIG_STAR_PMIC
{
    NV_ODM_GUID('a','l','l','p','o','w','e','r'), 
    s_lge_AllRailAddresses,
    NV_ARRAY_SIZE(s_lge_AllRailAddresses),
    NvOdmPeripheralClass_Other
},
#endif
//20100603, cs77.ha@lge.com, star pmic [END]

//20100730, kyungsik.lee@lge.com, star cpwatcher [START]
#if defined(CONFIG_MACH_STAR)
{
	NV_ODM_GUID('c','p','d','e','v','i','c','e'),
	s_lge_CPdeviceAddresses,
	NV_ARRAY_SIZE(s_lge_CPdeviceAddresses),
	NvOdmPeripheralClass_Other
},
#endif /* CONFIG_MACH_STAR */
//20100730, kyungsik.lee@lge.com, star cpwatcher [END]

#ifdef CONFIG_SPI_TDMB	
//20100918 suyong.han@lge.com TDMB Base [START_LGE_LAB1]
//20100912, suyong.han@lge.com [START]	
{
	NV_ODM_GUID('s','p','i','_','t','d','m','b'),
	s_tdmbSIC2102Addresses,
	NV_ARRAY_SIZE(s_tdmbSIC2102Addresses),
	NvOdmPeripheralClass_Other
},
//20100912, syblue.lee@lge.com [END]
//20100918 suyong.han@lge.com TDMB Base [END_LGE_LAB1]
#endif


