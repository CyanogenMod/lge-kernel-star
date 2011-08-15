/*
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SSL3250A_H__
#define __SSL3250A_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define SSL3250A_IOCTL_MODE_SHUTDOWN	_IOW('o', 1, __u8)
#define SSL3250A_IOCTL_MODE_STANDBY	_IOW('o', 2, __u8)
#define SSL3250A_IOTCL_MODE_TORCH	_IOW('o', 3, __u8)
#define SSL3250A_IOCTL_MODE_FLASH	_IOW('o', 4, __u8)
#define SSL3250A_IOCTL_MODE_LED		_IOW('o', 5, __u8)
#define SSL3250A_IOCTL_STRB		_IOW('o', 6, __u8)
#define SSL3250A_IOCTL_TIMER		_IOW('o', 7, __u8)

#ifdef __KERNEL__
struct ssl3250a_platform_data {
	int config;
	int max_amp_indic;
	int max_amp_torch;
	int max_amp_flash;
	int (*init)(void);
	void (*exit)(void);
	int (*gpio_act)(int);
	int (*gpio_en1)(int);
	int (*gpio_en2)(int);
	int (*gpio_strb)(int);
};
#endif /* __KERNEL__ */

#endif /* __SSL3250A_H__ */

