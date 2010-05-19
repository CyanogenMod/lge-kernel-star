ccflags-y += -DNV_IS_AVP=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/nvec
obj-y += nvec_i2c_transport.o
obj-y += nvec_smbus_packet.o
