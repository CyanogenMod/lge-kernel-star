#ifndef	REGISTER_COMMON_INIT_H
#define	REGISTER_COMMON_INIT_H

/*============================================================================
  
                 Camera Interface Device Driver Source File
  
   DESCRIPTION
     This file contains the definitions needed for the camera interface
  
   Copyright (c) 2009 by LG Electronics, Inc.  All Rights Reserved.
============================================================================*/

/*============================================================================

                      EDIT HISTORY FOR FILE

 This section contains comments describing changes made to this file.
 Notice that changes are listed in reverse chronological order.

 when         who    what, where, why   
 --------  -----  ----------------------------------------------------------
 09/06/10  jheim  ported to android.
 12/28/09  jheim  Initial create.
============================================================================*/
#undef word
typedef uint16_t word;
#undef byte
typedef uint8_t byte;
#undef uint32
typedef uint32_t uint32;
/*============================================================================
                          EXTERNAL CONSTANT DEFINITIONS
============================================================================*/
typedef enum {
  COMMON_REG_REG,        // addr 8bit, data 8bit                   : common_reg_reg_type
  COMMON_REG_REG_VAR4,   // addr 8bit, data 8bit * 4, len 8bit   : common_reg_reg_var4_type
  COMMON_REG_MEM,        // addr 16bit, data 16 bit               : common_reg_mem_type
  COMMON_REG_MEM_VAR4,   // addr 16bit, data 8bit * 4, len 8bit : common_reg_mem_var4_type

  COMMON_REG_MAX
} common_reg_enum_type;

typedef union{
  uint32 val32;
  word val16;
  byte val8[4];
} common_reg_data_type;

typedef struct
{
  byte addr;
  byte val;
} common_reg_reg_type;

typedef struct
{
  byte addr;
  common_reg_data_type vals;
  byte len;
} common_reg_reg_var4_type;

typedef struct
{
  word addr;
  word val;
} common_reg_mem_type;

typedef struct
{
  word addr;
  common_reg_data_type vals;
  byte len;
} common_reg_mem_var4_type;

typedef union
{
  common_reg_reg_type reg;
  common_reg_reg_var4_type reg_var4;
  common_reg_mem_type mem;
  common_reg_mem_var4_type mem_var4;
} common_reg_type;

typedef struct
{
  int num_regs;
  common_reg_type list_regs[1]; 
} common_reg_list_type;

extern void common_register_init(common_reg_enum_type , common_reg_list_type** );
#endif //REGISTER_COMMON_INIT_H