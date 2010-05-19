ccflags-y += -DNV_IS_AVP=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

obj-y += nvec.o
obj-y += NvECPackage_Dispatch.o
obj-y += nvec_dispatch.o
obj-y += smbus/
