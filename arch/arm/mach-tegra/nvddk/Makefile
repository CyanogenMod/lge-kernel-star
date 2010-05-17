ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ccflags-y += -DNV_DEBUG=0

obj-$(CONFIG_MTD_NAND_TEGRA)	+= nvddk_nand.o

ifneq ($(filter y,$(CONFIG_USB_TEGRA) $(CONFIG_USB_TEGRA_HCD)),)
obj-y	+= nvddk_usbphy.o
obj-y	+= nvddk_usbphy_ap20.o
endif

obj-$(CONFIG_TEGRA_FUSE)	+= nvddk_fuse_ap20.o