ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ifeq ($(CONFIG_TEGRA_ODM_CONCORDE),y)
  is_tpk_touch := y
endif

ifeq ($(CONFIG_TEGRA_ODM_WHISTLER),y)
  is_tpk_touch := y
endif

ifeq ($(CONFIG_TEGRA_ODM_HARMONY),y)
  is_panjit_touch := y
endif

ifeq ($(CONFIG_TEGRA_ODM_VENTANA),y)
  is_panjit_touch := y
endif

ccflags-$(is_tpk_touch) += -DNV_TOUCH_TPK
ccflags-$(is_panjit_touch) += -DNV_TOUCH_PANJIT

obj-y			+= nvodm_touch.o
obj-$(is_tpk_touch)	+= nvodm_touch_tpk.o
obj-$(is_panjit_touch)	+= nvodm_touch_panjit.o
