#
# USB peripheral controller drivers
#
ifeq ($(CONFIG_USB_GADGET_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-$(CONFIG_USB_DUMMY_HCD)	+= dummy_hcd.o
obj-$(CONFIG_USB_NET2280)	+= net2280.o
obj-$(CONFIG_USB_AMD5536UDC)	+= amd5536udc.o
obj-$(CONFIG_USB_PXA25X)	+= pxa25x_udc.o
obj-$(CONFIG_USB_PXA27X)	+= pxa27x_udc.o
obj-$(CONFIG_USB_IMX)		+= imx_udc.o
obj-$(CONFIG_USB_GOKU)		+= goku_udc.o
obj-$(CONFIG_USB_OMAP)		+= omap_udc.o
obj-$(CONFIG_USB_LH7A40X)	+= lh7a40x_udc.o
obj-$(CONFIG_USB_S3C2410)	+= s3c2410_udc.o
obj-$(CONFIG_USB_AT91)		+= at91_udc.o
obj-$(CONFIG_USB_ATMEL_USBA)	+= atmel_usba_udc.o
obj-$(CONFIG_USB_FSL_USB2)	+= fsl_usb2_udc.o
obj-$(CONFIG_USB_TEGRA)		+= fsl_udc_core.o
obj-$(CONFIG_USB_TEGRA)		+= tegra_udc.o
fsl_usb2_udc-objs		:= fsl_udc_core.o
ifeq ($(CONFIG_ARCH_MXC),y)
fsl_usb2_udc-objs		+= fsl_mx3_udc.o
endif
obj-$(CONFIG_USB_M66592)	+= m66592-udc.o
obj-$(CONFIG_USB_R8A66597)	+= r8a66597-udc.o
obj-$(CONFIG_USB_FSL_QE)	+= fsl_qe_udc.o
obj-$(CONFIG_USB_CI13XXX)	+= ci13xxx_udc.o
obj-$(CONFIG_USB_S3C_HSOTG)	+= s3c-hsotg.o
obj-$(CONFIG_USB_LANGWELL)	+= langwell_udc.o

#
# USB gadget drivers
#
# 20100825, jm1.lee@lge.com, for USB mode switching [START]
ifeq ($(CONFIG_USB_ANDROID_CDC_ECM),y)
F_ECM = f_ecm.o
endif
ifeq ($(CONFIG_USB_F_SERIAL),y)
F_SERIAL = f_serial.o
endif

ifeq ($(CONFIG_USB_ANDROID_RMNET),y)
F_RMNET = f_rmnet.o
endif

ifeq ($(CONFIG_USB_ANDROID_RNDIS),y)
F_RNDIS = f_rndis.o
endif

ifeq ($(CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET),y)
ANDROID = android_lg.o
endif
# 20100825, jm1.lee@lge.com, for USB mode switching [END]
g_zero-objs			:= zero.o
g_audio-objs			:= audio.o
g_ether-objs			:= ether.o
g_serial-objs			:= serial.o
g_midi-objs			:= gmidi.o
gadgetfs-objs			:= inode.o
g_file_storage-objs		:= file_storage.o
g_printer-objs			:= printer.o
g_cdc-objs			:= cdc2.o
# 20100825, jm1.lee@lge.com, for USB mode switching [START]
g_android-objs			:= $(ANDROID) f_adb.o f_mass_storage.o \
					u_serial.o f_acm.o u_ether.o \
					$(F_ECM) $(F_SERIAL) $(F_RMNET) $(F_RNDIS)
# 20100825, jm1.lee@lge.com, for USB mode switching [END]

obj-$(CONFIG_USB_ZERO)		+= g_zero.o
obj-$(CONFIG_USB_AUDIO)		+= g_audio.o
obj-$(CONFIG_USB_ETH)		+= g_ether.o
obj-$(CONFIG_USB_GADGETFS)	+= gadgetfs.o
obj-$(CONFIG_USB_FILE_STORAGE)	+= g_file_storage.o
obj-$(CONFIG_USB_G_SERIAL)	+= g_serial.o
obj-$(CONFIG_USB_G_PRINTER)	+= g_printer.o
obj-$(CONFIG_USB_MIDI_GADGET)	+= g_midi.o
obj-$(CONFIG_USB_CDC_COMPOSITE) += g_cdc.o
# 20100825, jm1.lee@lge.com, for USB mode switching [START]
obj-$(CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET)	+= g_android.o
# 20100825, jm1.lee@lge.com, for USB mode switching [END]
obj-$(CONFIG_USB_ANDROID)	+= android.o
obj-$(CONFIG_USB_ANDROID_ACM)	+= f_acm.o u_serial.o f_serial.o f_serial_gps.o
obj-$(CONFIG_USB_ANDROID_ADB)	+= f_adb.o
# 20100825, jm1.lee@lge.com, for USB mode switching [START]
ifeq ($(CONFIG_USB_ANDROID),y)
obj-$(CONFIG_USB_ANDROID_MASS_STORAGE)	+= f_mass_storage.o
obj-$(CONFIG_USB_ANDROID_RNDIS)	+= f_rndis.o u_ether.o
endif
# 20100825, jm1.lee@lge.com, for USB mode switching [END]

