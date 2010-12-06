# FIXME: Forcing these options to test AVP moduleloader.
# In the formal Kconfig layout, those options depend on CONFIG_MEDIA_SUPPORT.
# AVP moduleloader has to be tested before enabling CONFIG_MEDIA_SUPPORT.
CONFIG_TEGRA_RPC=y
CONFIG_TEGRA_AVP=y

obj-$(CONFIG_TEGRA_RPC)		+= tegra_rpc.o
obj-$(CONFIG_TEGRA_RPC)		+= trpc_local.o
obj-$(CONFIG_TEGRA_RPC)		+= trpc_sema.o
obj-$(CONFIG_TEGRA_AVP)		+= avp.o
obj-$(CONFIG_TEGRA_AVP)		+= avp_svc.o
obj-$(CONFIG_TEGRA_AVP)		+= headavp.o
