GCOV_PROFILE := y
#
# Makefile for the video capture/playback device drivers.
#
obj-y				+= avp/
ifeq ($(CONFIG_MACH_BSSQ),y)
obj-$(CONFIG_LU6500)	        += imx072_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
obj-$(CONFIG_LU6500)    	+= dw9712_lu6500.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
obj-$(CONFIG_SU880)		+= imx072_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
obj-$(CONFIG_SU880)		+= dw9712_lu6500.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
obj-$(CONFIG_KU8800)		+= imx072_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
obj-$(CONFIG_KU8800)		+= dw9712_lu6500.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
##obj-$(CONFIG_LU8800)		+= imx072_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
##obj-$(CONFIG_LU8800)		+= dw9712.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
##obj-$(CONFIG_KS1001)		+= ar0832_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
obj-$(CONFIG_VIDEO_AR8032)		+= ar0832_main.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
##obj-$(CONFIG_KS1001)		+= bu9889gulw.o #E2PROM driver  #20111119 calvin.hwang@lge.com EEPROM driver
obj-$(CONFIG_VTCAM_SD_IMAGE_TUNNING)		+= bu9889gulw.o #E2PROM driver  #20111119 calvin.hwang@lge.com EEPROM driver
#obj-y				+= ar0832_focuser.o #20110525 calvin.hwang@lge.com Camsensor ks1001 merge
obj-$(CONFIG_VIDEO_HI702)       += hi702_vga.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
obj-$(CONFIG_VIDEO_HI702)       += hi702_setting_value.o #20110518 jinkwan.kim@lge.com Camsensor 1st merge
obj-$(CONFIG_VIDEO_HI702)       += register_common_init.o 
endif

obj-$(CONFIG_TEGRA_MEDIASERVER)	+= mediaserver/
obj-$(CONFIG_TEGRA_NVAVP)	+= nvavp/
obj-$(CONFIG_TEGRA_DTV)		+= tegra_dtv.o
obj-$(CONFIG_TEGRA_CAMERA)	+= tegra_camera.o
#obj-$(CONFIG_VIDEO_AR0832)	+= ar0832_main.o
obj-$(CONFIG_VIDEO_OV5650)	+= ov5650.o
obj-$(CONFIG_VIDEO_OV14810)	+= ov14810.o
obj-$(CONFIG_VIDEO_IMX073)      += imx073.o
obj-$(CONFIG_VIDEO_OV9726)	+= ov9726.o
obj-$(CONFIG_VIDEO_OV2710)	+= ov2710.o
obj-$(CONFIG_VIDEO_SOC380)	+= soc380.o
obj-$(CONFIG_VIDEO_MT9M113)     += mt9m113.o
obj-$(CONFIG_TORCH_SSL3250A)	+= ssl3250a.o
obj-$(CONFIG_TORCH_TPS61050)	+= tps61050.o
obj-$(CONFIG_VIDEO_SH532U)	+= sh532u.o
obj-$(CONFIG_VIDEO_AD5820)	+= ad5820.o
obj-$(CONFIG_VIDEO_DW9714)      += dw9714.o
obj-$(CONFIG_VIDEO_DW9712)      += dw9712.o
obj-$(CONFIG_TORCH_AAT1270)     += aat1270.o
