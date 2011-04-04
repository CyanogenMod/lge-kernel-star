ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

obj-y += nvodm_touch.o
# 20100414 joseph.jung@lge.com for Star Touch Compile
obj-y += nvodm_touch_synaptics.o
obj-y += nvodm_touch_cypress.o
# 20100919 hyeongwon.oh@lge.com for Star OneTouch
obj-y += nvodm_onetouch.o
obj-y += nvodm_onetouch_synaptics.o
