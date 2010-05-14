/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

 
#ifndef INCLUDED_TCA6416_EXPANDER_REG_HEADER
#define INCLUDED_TCA6416_EXPANDER_REG_HEADER


// Ports evailable on TCA6416. it is having 2 ports
#define TCA6416_PORT_0  0
#define TCA6416_PORT_1  1


// Each port is having 8 pins 
#define TCA6416_PIN_0  0
#define TCA6416_PIN_1  1
#define TCA6416_PIN_2  2
#define TCA6416_PIN_3  3
#define TCA6416_PIN_4  4
#define TCA6416_PIN_5  5
#define TCA6416_PIN_6  6
#define TCA6416_PIN_7  7


// Registers
#define TCA6416_INPUT_PORT_0            0x00    // For ports 00 to 07
#define TCA6416_INPUT_PORT_1            0x01    // For ports 10 to 17
#define TCA6416_OUTPUT_PORT_0           0x02    // For ports 00 to 07
#define TCA6416_OUTPUT_PORT_1           0x03    // For ports 10 to 17
#define TCA6416_POLARITY_INV_PORT_0     0x04    // For ports 00 to 07
#define TCA6416_POLARITY_INV_PORT_1     0x05    // For ports 10 to 17
#define TCA6416_CONFIG_PORT_0           0x06    // For ports 00 to 07
#define TCA6416_CONFIG_PORT_1           0x07    // For ports 10 to 17



#define TCA6416_INVALID_PORT    0xFF

#endif //INCLUDED_TCA6416_EXPANDER_REG_HEADER

