ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

obj-$(CONFIG_KEYBOARD_TEGRA)	+= nvodm_kbc.o
obj-$(CONFIG_KEYBOARD_TEGRA)	+= nvodm_kbc_keymapping.o
obj-y				+= nvodm_sdio.o
obj-y				+= nvodm_uart.o
obj-y				+= nvodm_usbulpi.o
