ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations

obj-y += nvodm_query.o
obj-y += nvodm_query_discovery.o
obj-y += nvodm_query_nand.o
obj-y += nvodm_query_gpio.o
obj-y += nvodm_query_pinmux.o
obj-y += nvodm_query_kbc.o

