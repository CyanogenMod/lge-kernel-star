/*
 * aeskeywrap.h
 * Perform RFC3394 AES-based key wrap and unwrap functions.
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: aeskeywrap.h,v 1.13 2007/05/15 21:22:41 Exp $
 */

#ifndef _AESWRAP_H_
#define _AESWRAP_H_

#include <typedefs.h>

#ifdef BCMDRIVER
#include <osl.h>
#else
#include <stddef.h>  /* For size_t */
#endif

#include <bcmcrypto/aes.h>

#define AKW_BLOCK_LEN	8
/* Max size of wrapped data, not including overhead
 *  The key wrap algorithm doesn't impose any upper bound the wrap length,
 *  but we need something big enought to handle all users.  802.11i and
 *  probably most other users will be limited by MTU size, so using a 2K
 *  buffer limit seems relatively safe.
 */
#define AKW_MAX_WRAP_LEN	384

/* aes_wrap: perform AES-based keywrap function defined in RFC3394
 *	return 0 on success, 1 on error
 *	input is il bytes
 *	output is (il+8) bytes
 */
int aes_wrap(size_t kl, uint8 *key, size_t il, uint8 *input, uint8 *output);

/* aes_unwrap: perform AES-based key unwrap function defined in RFC3394,
 *	return 0 on success, 1 on error
 *	input is il bytes
 *	output is (il-8) bytes
 */
int aes_unwrap(size_t kl, uint8 *key, size_t il, uint8 *input, uint8 *output);

#endif /* _AESWRAP_H_ */
