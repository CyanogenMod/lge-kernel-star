/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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
 
#ifndef TPK_REG_HEADER
#define TPK_REG_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif


// SMBus Aliased Address
/* To make most efficient use of the SMBus paged addressing scheme, Synaptics
 * RMI-on-SMBus devices define that for all commonly used RMI device registers, 
 * there will be a duplicate aliased register located at a new RMI address. 
 * The entire set of aliased register addresses are grouped into a single page
 * of the RMI address space. This will enable user software to access all commonly
 * used RMI registers without ever having to rewrite the Page Select register.
 *
 * The aliased addresses occupy page $04xx in the general RMI address map. At reset,
 * all RMI-on-SMBus devices initialize their Page Select register to the value $04. 
 * This means that by default, all SMBus register accesses will access the RMI
 * Aliased Address space.
 *
 * ----------------------------------------------------------------------------
 * | Aliased Address | General RMI Address | Register Group                  |
 * ----------------------------------------------------------------------------
 * | $0400-$041F     | $0400-$041F          | RMI Data Register and           |
 * |                 |                      | Device Status Reg               |
 * ----------------------------------------------------------------------------
 * | $04E0-$04E7     | $0200-$0207          | RMI Product ID queries          |
 * ----------------------------------------------------------------------------
 * | $04F0-$04F4     | $0000-$0004          | RMI control, command, and       |
 * |                 |                      | general status registers        |
 * ----------------------------------------------------------------------------
 * | $04FF           | $xxFF                | Page Select Register            |
 * ----------------------------------------------------------------------------
 
*/
#define TPK_DATA_0                                 0x00 //Data Reg 0 - Finger #0
#define TPK_DATA_1                                 0x01 //Data Reg 1 - Finger #0
#define TPK_DATA_2                                 0x02 //Data Reg 2 - Finger #0
#define TPK_DATA_3                                 0x03 //Data Reg 3 - Finger #0
#define TPK_DATA_4                                 0x04 //Data Reg 4 - Finger #0
#define TPK_DATA_5                                 0x05 //Data Reg 5 - Finger #0
#define TPK_DATA_6                                 0x06 //Data Reg 6 - Finger #1
#define TPK_DATA_7                                 0x07 //Data Reg 7 - Finger #1
#define TPK_DATA_8                                 0x08 //Data Reg 8 - Finger #1
#define TPK_DATA_9                                 0x09 //Data Reg 9 - Finger #1
#define TPK_DATA_A                                 0x0A //Data Reg 10 - Finger #1
#define TPK_DATA_B                                 0x0B //Data Reg 11 - Finger #1
#define TPK_RELATIVE_DATA_X                        0x0C //Relative Horizontal Motion - Finger #0
#define TPK_RELATIVE_DATA_Y                        0x0D //Relative Vertical Motion - Finger #0
#define TPK_DEVICE_STATUS                          0x0E //Device Status Register
#define TPK_2D_CONTROL                             0x21 //Func10: General 2D Control Reg
#define TPK_2D_RELATIVE_SPEED                      0x22 //Func10: General 2D Relative Speed Reg
#define TPK_2D_ACCLELERATION                       0x23 //Func10: General 2D Relative Acceleration Reg
#define TPK_SENSOR_SENSITIVITY                     0x24 //Func10: Sensor Sensitivity
#define TPK_SENSOR_MAXPOSITION_0                   0x26 //Func10: Sensor Max Position (bit 12:8)
#define TPK_SENSOR_MAXPOSITION_1                   0x27 //Func10: Sensor Max Position (bit 7:0)
#define TPK_RMI_PROTOCOL_VERSION                   0xE0 //RMI Protocol Version
#define TPK_MANUFACTURER_ID                        0xE1 //Manufacturer ID
#define TPK_PHYSICAL_INTERFACE_VERSION             0xE2 //Physical Interface Version
#define TPK_PRODUCT_QUERY                          0xE3 //Product Property
#define TPK_PRODUCT_INFO_QUERY_0                   0xE4 //Product Info 0
#define TPK_PRODUCT_INFO_QUERY_1                   0xE5 //Product Info 1 (REVISION_ID)
#define TPK_PRODUCT_INFO_QUERY_2                   0xE6 //Product Info 2
#define TPK_PRODUCT_INFO_QUERY_3                   0xE7 //Product Info 3
#define TPK_DEVICE_CONTROL                         0xF0 //Device Control Register
#define TPK_INTR_ENABLE                            0xF1 //Interrupt Enable Register
#define TPK_ERROR_STATUE                           0xF2 //Error Status Register
#define TPK_INTR_STATUS                            0xF3 //Interrupt Request Status Register
#define TPK_DEVICE_COMMAND                         0xF4 //Device Command Register
#define TPK_PAGE_SELECT                            0xFF //Page Select Register

// RMI Address Space
// Data registers
#define TPK_RMI_DATA_0                             0x0400 //Data Reg 0 - Finger #0
#define TPK_RMI_DATA_1                             0x0401 //Data Reg 1 - Finger #0
#define TPK_RMI_DATA_2                             0x0402 //Data Reg 2 - Finger #0
#define TPK_RMI_DATA_3                             0x0403 //Data Reg 3 - Finger #0
#define TPK_RMI_DATA_4                             0x0404 //Data Reg 4 - Finger #0
#define TPK_RMI_DATA_5                             0x0405 //Data Reg 5 - Finger #0
#define TPK_RMI_DATA_6                             0x0406 //Data Reg 6 - Finger #1
#define TPK_RMI_DATA_7                             0x0407 //Data Reg 7 - Finger #1
#define TPK_RMI_DATA_8                             0x0408 //Data Reg 8 - Finger #1
#define TPK_RMI_DATA_9                             0x0409 //Data Reg 9 - Finger #1
#define TPK_RMI_DATA_A                             0x040A //Data Reg 10 - Finger #1
#define TPK_RMI_DATA_B                             0x040B //Data Reg 11 - Finger #1
#define TPK_RMI_RELATIVE_DATA_X                    0x040C //Relative Horizontal Motion - Finger #0
#define TPK_RMI_RELATIVE_DATA_Y                    0x040D //Relative Vertical Motion - Finger #0
#define TPK_RMI_DEVICE_STATUS                      0x040E //Device Status Register

// Function $10 register pages
#define TPK_RMI_FUNCTION_VERSION                   0x1000 //Func10: Function Version query
#define TPK_RMI_2D_PROPERTIES                      0x1001 //Func10: General 2D Properties query
#define TPK_RMI_SENSOR_PROPERTIES_0                0x1002 //Func10: Sensor Properties
#define TPK_RMI_SENSOR_PROPERTIES_1                0x1003 //Func10: Sensor Properties
#define TPK_RMI_SENSOR_X_MAX_POSITION_0            0x1004 //Func10: Sensor X Max Position (bits 12:8)
#define TPK_RMI_SENSOR_X_MAX_POSITION_1            0x1005 //Func10: Sensor X Max Position (bits 7:0)
#define TPK_RMI_SENSOR_Y_MAX_POSITION_0            0x1006 //Func10: Sensor Y Max Position (bits 12:8)
#define TPK_RMI_SENSOR_Y_MAX_POSITION_1            0x1007 //Func10: Sensor Y Max Position (bits 7:0)
#define TPK_RMI_SENSOR_RESOLUTION                  0x1008 //Func10: Sensor Resolution
#define TPK_RMI_2D_CONTROL                         0x1041 //Func10: General 2D Control Reg
#define TPK_RMI_2D_RELATIVE_SPEED                  0x1042 //Func10: General 2D Relative Speed Reg
#define TPK_RMI_2D_ACCLELERATION                   0x1043 //Func10: General 2D Relative Acceleration Reg
#define TPK_RMI_SENSOR_SENSITIVITY                 0x1044 //Func10: Sensor Sensitivity
#define TPK_RMI_SENSOR_MAXPOSITION_0               0x1046 //Func10: Sensor Max Position (bit 12:8)
#define TPK_RMI_SENSOR_MAXPOSITION_1               0x1047 //Func10: Sensor Max Position (bit 7:0)

// General product information and version queries
#define TPK_RMI_RMI_PROTOCOL_VERSION               0x2000 //RMI Protocol Version
#define TPK_RMI_MANUFACTURER_ID                    0x2001 //Manufacturer ID
#define TPK_RMI_PHYSICAL_INTERFACE_VERSION         0x2002 //Physical Interface Version
#define TPK_RMI_PRODUCT_QUERY                      0x2003 //Product Property
#define TPK_RMI_PRODUCT_INFO_QUERY_0               0x2004 //Product Info 0
#define TPK_RMI_PRODUCT_INFO_QUERY_1               0x2005 //Product Info 1 (REVISION_ID)
#define TPK_RMI_PRODUCT_INFO_QUERY_2               0x2006 //Product Info 2
#define TPK_RMI_PRODUCT_INFO_QUERY_3               0x2007 //Product Info 3

// Standard RMI control, command, and status registers
#define TPK_RMI_DEVICE_CONTROL                     0x0000 //Device Control Register
#define TPK_RMI_INTR_ENABLE                        0x0001 //Interrupt Enable Register
#define TPK_RMI_ERROR_STATUE                       0x0002 //Error Status Register
#define TPK_RMI_INTR_STATUS                        0x0003 //Interrupt Request Status Register
#define TPK_RMI_DEVICE_COMMAND                     0x0004 //Device Command Register

#if defined(__cplusplus)
}
#endif


#endif //TPK_REG_HEADER


