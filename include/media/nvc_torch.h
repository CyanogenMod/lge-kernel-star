/* Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __NVC_TORCH_H__
#define __NVC_TORCH_H__

struct nvc_torch_level_info {
	__s32 guidenum;
	__u32 sustaintime;
	__s32 rechargefactor;
} __packed;

struct nvc_torch_pin_state {
	__u16 mask;
	__u16 values;
} __packed;

struct nvc_torch_flash_capabilities {
	__u32 numberoflevels;
	struct nvc_torch_level_info levels[];
} __packed;

struct nvc_torch_torch_capabilities {
	__u32 numberoflevels;
	__s32 guidenum[];
} __packed;

#endif /* __NVC_TORCH_H__ */

