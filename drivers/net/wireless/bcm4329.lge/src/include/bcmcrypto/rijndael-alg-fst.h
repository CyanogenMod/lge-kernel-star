/* $Id: rijndael-alg-fst.h,v 1.6.472.1 2010/02/22 21:56:35 Exp $ */

/**
 * rijndael-alg-fst.h
 *
 * @version 3.0 (December 2000)
 *
 * Optimised ANSI C code for the Rijndael cipher (now AES)
 *
 * @author Vincent Rijmen <vincent.rijmen@esat.kuleuven.ac.be>
 * @author Antoon Bosselaers <antoon.bosselaers@esat.kuleuven.ac.be>
 * @author Paulo Barreto <paulo.barreto@terra.com.br>
 *
 * This code is hereby placed in the public domain.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
 * $Id: rijndael-alg-fst.h,v 1.6.472.1 2010/02/22 21:56:35 Exp $
 */

#ifndef __RIJNDAEL_ALG_FST_H

#define __RIJNDAEL_ALG_FST_H


#ifdef LINKED_WITH_OPENSSL
#include "bcmcrypto/bn_bcmcrypto.h"
#endif

#define MAXKC	(256/32)

#define MAXKB	(256/8)

#define MAXNR	14


#include <typedefs.h>


int rijndaelKeySetupEnc(uint32 rk[], const uint8 cipherKey[], int keyBits);

int rijndaelKeySetupDec(uint32 rk[], const uint8 cipherKey[], int keyBits);

void rijndaelEncrypt(const uint32 rk[], int Nr, const uint8 pt[16], uint8 ct[16]);

void rijndaelDecrypt(const uint32 rk[], int Nr, const uint8 ct[16], uint8 pt[16]);


#ifdef INTERMEDIATE_VALUE_KAT

void rijndaelEncryptRound(const uint32 rk[], int Nr, uint8 block[16], int rounds);

void rijndaelDecryptRound(const uint32 rk[], int Nr, uint8 block[16], int rounds);

#endif /* INTERMEDIATE_VALUE_KAT */

#endif /* __RIJNDAEL_ALG_FST_H */
