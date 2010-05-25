ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/nvrm/core/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/io/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/io
ccflags-y += -Iarch/arm/mach-tegra/nvrm/core

obj-y += nvrm_i2c.o
obj-y += nvrm_gpioi2c.o
obj-y += nvrm_owr.o
obj-y += nvrm_dma.o
obj-y += nvrm_gpio.o
