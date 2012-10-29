/*
 * arch/arm/mach-tegra/lge/star/star_bootloader_parse.c
 *
 * Based on 
 * arch/arm/mach-tegra/board-shuttle.c
 *
 * Copyright (C) 2011 Eduardo José Tagle <ejtagle@tutopia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/setup.h>
#include <mach-tegra/board.h>

/* NVidia bootloader tags and parsing routines */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM				0x1
#define ATAG_NVIDIA_DISPLAY			0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_CARVEOUT		0x6
#define ATAG_NVIDIA_WARMBOOT		0x7

#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	3
#define ATAG_NVIDIA_FORCE_32		0x7fffffff


struct tag_tegra {
	__u32 bootarg_key;
	__u32 bootarg_len;
	char bootarg[1];
};

/**
 * Resource Manager boot args.
 *
 * Nothing here yet.
 */
struct NVBOOTARGS_Rm
{
	u32 	reserved;
};

/**
 * Carveout boot args, which define the physical memory location of the GPU
 * carved-out memory region(s).
 */
struct NVBOOTARGS_Carveout
{
	void* 	base;
	u32 	size;
};

/**
 * Warmbootloader boot args. This structure only contains
 * a mem handle key to preserve the warm bootloader
 * across the bootloader->os transition
 */
struct NVBOOTARGS_Warmboot
{
	/* The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
};

/**
 * PreservedMemHandle boot args, indexed by ATAG_NVIDIA_PRESERVED_MEM_0 + n.
 * This allows physical memory allocations (e.g., for framebuffers) to persist
 * between the bootloader and operating system.  Only carveout and IRAM
 * allocations may be preserved with this interface.
 */
struct NVBOOTARGS_PreservedMemHandle
{
	u32 	Address;
	u32   	Size;
};

/**
 * Display boot args.
 *
 * The bootloader may have a splash screen. This will flag which controller
 * and device was used for the splash screen so the device will not be
 * reinitialized (which causes visual artifacts).
 */
struct NVBOOTARGS_Display
{
	/* which controller is initialized */
	u32 	Controller;

	/* index into the ODM device list of the boot display device */
	u32 	DisplayDeviceIndex;

	/* set to != 0 if the display has been initialized */
	u8 		bEnabled;
};

/**
 * Framebuffer boot args
 *
 * A framebuffer may be shared between the bootloader and the
 * operating system display driver.  When this key is present,
 * a preserved memory handle for the framebuffer must also
 * be present, to ensure that no display corruption occurs
 * during the transition.
 */
struct NVBOOTARGS_Framebuffer
{
	/*  The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
	/*  Total memory size of the framebuffer */
	u32 	Size;
	/*  Color format of the framebuffer, cast to a U32  */
	u32 	ColorFormat;
	/*  Width of the framebuffer, in pixels  */
	u16 	Width;
	/*  Height of each surface in the framebuffer, in pixels  */
	u16 	Height;
	/*  Pitch of a framebuffer scanline, in bytes  */
	u16 	Pitch;
	/*  Surface layout of the framebuffer, cast to a U8 */
	u8  	SurfaceLayout;
	/*  Number of contiguous surfaces of the same height in the
	    framebuffer, if multi-buffering.  Each surface is
	    assumed to begin at Pitch * Height bytes from the
	    previous surface.  */
	u8  	NumSurfaces;
	/* Flags for future expandability.
	   Current allowable flags are:
	   zero - default
	   NV_BOOT_ARGS_FB_FLAG_TEARING_EFFECT - use a tearing effect signal in
	   combination with a trigger from the display software to generate
	   a frame of pixels for the display device. */
	u32 	Flags;
#define NVBOOTARG_FB_FLAG_TEARING_EFFECT (0x1)

};

/**
 * Chip characterization shmoo data
 */
struct NVBOOTARGS_ChipShmoo
{
	/* The key used for accessing the preserved memory handle of packed
	   characterization tables  */
	u32 	MemHandleKey;

	/* Offset and size of each unit in the packed buffer */
	u32 	CoreShmooVoltagesListOffset;
	u32 	CoreShmooVoltagesListSize;

	u32 	CoreScaledLimitsListOffset;
	u32 	CoreScaledLimitsListSize;

	u32 	OscDoublerListOffset;
	u32 	OscDoublerListSize;

	u32 	SKUedLimitsOffset;
	u32 	SKUedLimitsSize;

	u32 	CpuShmooVoltagesListOffset;
	u32 	CpuShmooVoltagesListSize;

	u32 	CpuScaledLimitsOffset;
	u32 	CpuScaledLimitsSize;

	/* Misc characterization settings */
	u16 	CoreCorner;
	u16 	CpuCorner;
	u32 	Dqsib;
	u32 	SvopLowVoltage;
	u32 	SvopLowSetting;
	u32 	SvopHighSetting;
};

/**
 * Chip characterization shmoo data indexed by NvBootArgKey_ChipShmooPhys
 */
struct NVBOOTARGS_ChipShmooPhys
{
	u32 	PhysShmooPtr;
	u32 	Size;
};


/**
 * OS-agnostic bootarg structure.
 */
struct NVBOOTARGS
{
	struct NVBOOTARGS_Rm 					RmArgs;
	struct NVBOOTARGS_Display 				DisplayArgs;
	struct NVBOOTARGS_Framebuffer 			FramebufferArgs;
	struct NVBOOTARGS_ChipShmoo 			ChipShmooArgs;
	struct NVBOOTARGS_ChipShmooPhys			ChipShmooPhysArgs;
	struct NVBOOTARGS_Warmboot 				WarmbootArgs;
	struct NVBOOTARGS_PreservedMemHandle 	MemHandleArgs[ATAG_NVIDIA_PRESERVED_MEM_N];
};

static struct NVBOOTARGS NvBootArgs = { {0}, {0}, {0}, {0}, {0}, {0}, {{0}} }; 

static int __init get_cfg_from_tags(void)
{
	/* If the bootloader framebuffer is found, use it */
	if (tegra_bootloader_fb_start == 0 && tegra_bootloader_fb_size == 0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.FramebufferArgs.Size != 0 &&
			NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the bootloader framebuffer address and size. Store it */
		tegra_bootloader_fb_start = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_bootloader_fb_size  = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_debug("Nvidia TAG: framebuffer: %lu @ 0x%08lx\n",tegra_bootloader_fb_size,tegra_bootloader_fb_start);

		/* disable bootloader screen copying ... */
		/*tegra_bootloader_fb_size = tegra_bootloader_fb_start = 0;*/
	}

	/* If the LP0 vector is found, use it */
	if (tegra_lp0_vec_start == 0 && tegra_lp0_vec_size == 0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the Warmboot block address and size. Store it */
		tegra_lp0_vec_start = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_lp0_vec_size  = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_debug("Nvidia TAG: LP0: %lu @ 0x%08lx\n",tegra_lp0_vec_size,tegra_lp0_vec_start);		

		/* Until we find out if the bootloader supports the workaround required to implement
		   LP0, disable it */
		/*tegra_lp0_vec_start = tegra_lp0_vec_size = 0;*/

	}

	return 0;
}

static int __init parse_tag_nvidia(const struct tag *tag)
{
	const char *addr = (const char *)&tag->hdr + sizeof(struct tag_header);
	const struct tag_tegra *nvtag = (const struct tag_tegra*)addr;

	if (nvtag->bootarg_key >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			nvtag->bootarg_key <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) )
	{
		int Index = nvtag->bootarg_key - ATAG_NVIDIA_PRESERVED_MEM_0;

		struct NVBOOTARGS_PreservedMemHandle *dst = 
			&NvBootArgs.MemHandleArgs[Index];
		const struct NVBOOTARGS_PreservedMemHandle *src = 
			(const struct NVBOOTARGS_PreservedMemHandle *) nvtag->bootarg;

		if (nvtag->bootarg_len != sizeof(*dst)) {
			pr_err("Unexpected preserved memory handle tag length (expected: %d, got: %d!\n",
					sizeof(*dst), nvtag->bootarg_len);
		} else {

			pr_debug("Preserved memhandle: 0x%08x, address: 0x%08x, size: %d\n",
					nvtag->bootarg_key, src->Address, src->Size);

			memcpy(dst,src,sizeof(*dst));
		}
		return get_cfg_from_tags();
	}

	switch (nvtag->bootarg_key) {
		case ATAG_NVIDIA_CHIPSHMOO:
			{
				struct NVBOOTARGS_ChipShmoo *dst = 
					&NvBootArgs.ChipShmooArgs;
				const struct NVBOOTARGS_ChipShmoo *src = 
					(const struct NVBOOTARGS_ChipShmoo *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected preserved memory handle tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Shmoo tag with 0x%08x handle\n", src->MemHandleKey);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_DISPLAY:
			{
				struct NVBOOTARGS_Display *dst = 
					&NvBootArgs.DisplayArgs;
				const struct NVBOOTARGS_Display *src = 
					(const struct NVBOOTARGS_Display *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected display tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_FRAMEBUFFER:
			{
				struct NVBOOTARGS_Framebuffer *dst = 
					&NvBootArgs.FramebufferArgs;
				const struct NVBOOTARGS_Framebuffer *src = 
					(const struct NVBOOTARGS_Framebuffer *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected framebuffer tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Framebuffer tag with 0x%08x handle, size: %d\n",
							src->MemHandleKey,src->Size);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_RM:
			{
				struct NVBOOTARGS_Rm *dst = 
					&NvBootArgs.RmArgs;
				const struct NVBOOTARGS_Rm *src = 
					(const struct NVBOOTARGS_Rm *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected RM tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					memcpy(dst,src,sizeof(*dst));
				}

				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_CHIPSHMOOPHYS:
			{
				struct NVBOOTARGS_ChipShmooPhys *dst = 
					&NvBootArgs.ChipShmooPhysArgs;
				const struct NVBOOTARGS_ChipShmooPhys *src =
					(const struct NVBOOTARGS_ChipShmooPhys *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected phys shmoo tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Phys shmoo tag with pointer 0x%x and length %u\n",
							src->PhysShmooPtr, src->Size);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_WARMBOOT:
			{
				struct NVBOOTARGS_Warmboot *dst = 
					&NvBootArgs.WarmbootArgs;
				const struct NVBOOTARGS_Warmboot *src =
					(const struct NVBOOTARGS_Warmboot *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected Warnboot tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Found a warmboot tag with handle 0x%08x!\n", src->MemHandleKey);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}

		default:
			return get_cfg_from_tags();
	} 
	return get_cfg_from_tags();
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);
