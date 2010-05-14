ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

obj-$(CONFIG_TEGRA_ODM_HARMONY)		+= nvodm_keyboard.o
obj-$(CONFIG_TEGRA_ODM_WHISTLER)	+= nvodm_keyboard_stub.o
obj-$(CONFIG_TEGRA_ODM_CONCORDE)	+= nvodm_keyboard_stub.o
