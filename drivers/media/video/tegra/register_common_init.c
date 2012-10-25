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



/*============================================================================
                        INCLUDE FILES  
============================================================================*/
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <media/register_common_init.h>

/*============================================================================
                        CONSTANT DEFINITIONS  
============================================================================*/

/*============================================================================
                         INTERNAL CONSTANT DEFINITIONS
============================================================================*/

/*============================================================================
                          MACRO DEFINITIONS 
============================================================================*/
#ifdef LOCAL
#undef LOCAL
#endif
#define LOCAL static

#undef TRUE
#undef FALSE
#define TRUE 1
#define FALSE 0

#define VERSTR "//VERSION"
#define MAX_TOKEN 10
#define SLASH 0x2F
#define SPACE 0x20
#define CR 0x0D
#define LF 0x0A
#define TAB 0x09
#define COLON 0x3A
#define G_CHAR 0x47
#define g_CHAR 0x67
#define COMMA 0x2C
#define HEXADECIMAL 16
#define DECIMAL 10
#define BINARY 2
#define BUFFER_DUMMY 4

#define PARAM_ROOT                "/mnt/sdcard/"
#define CAMSENSOR_MAIN_PARAM_FILE PARAM_ROOT"sensorparamtable.txt"
#define CAMSENSOR_SUB_PARAM_FILE  PARAM_ROOT"VGA_tunning.txt"

/*============================================================================
                          VARIABLES
============================================================================*/

/*============================================================================
                          Internal Function Prototypes
============================================================================*/

/*============================================================================
                        External Function 
============================================================================*/

LOCAL unsigned char *find_next_token(unsigned char *fpage, unsigned char *token)
{
	unsigned char *cur_ptr;
	unsigned char temp;
	int idx=0;

	cur_ptr = fpage;
	temp = *cur_ptr;
	memset(token, 0x00, MAX_TOKEN);

	while(temp)
  {
		if( temp==SLASH)
    {
			if(*(cur_ptr+1)==SLASH)
      {
				cur_ptr = (unsigned char *)strchr((char *)cur_ptr, (int)LF);
				if (cur_ptr == NULL && idx == 0)
					return NULL;
				if(idx>0)
					break;
			}
      else
      {
				token[idx++]=*cur_ptr;
			}
		}
    else
    {
      if(temp==SPACE || temp==TAB ||temp==LF||temp==CR||temp==COMMA)
      {
  			if(idx > 0 )
  				break;
  		}
      else
      {
  			token[idx++]=*cur_ptr;
  		}
    }

		temp = *(++cur_ptr);	
	}

	if(idx)
		return cur_ptr;
	else
		return NULL;
}

LOCAL unsigned long convert_str_to_number(unsigned char *token)
{
	unsigned char scale;
	int ret_val=0;
	int diff;
	unsigned char temp;
	
	if(!token || *token== '\0')
		return 0;

	if(*token=='0' && (*(token+1)=='x' || *(token+1)=='X')){
		scale = HEXADECIMAL;
		token+=2;
	}else if(*token=='0' && (*(token+1)=='b' || *(token+1)=='B')){
		scale = BINARY;
		token+=2;
	}else{
		scale = DECIMAL;
	}
	
	while((temp = *token) != '\0'){
		if(temp < COLON)
			diff = SLASH+1;
		else if(temp < G_CHAR)
			diff = 'A' - 10;
		else if(temp < g_CHAR)
			diff = 'a' -10;
		else
			return 0;

		ret_val = ret_val*scale + (*(token++) -diff);
	}
	return ret_val;
}

#if 0  // obsolete routine..
LOCAL unsigned char *find_param_version(unsigned char *fpage, unsigned short *version)
{
	unsigned char *cur_ptr;
	unsigned char token[MAX_TOKEN];
	if((cur_ptr = (unsigned char *)strstr((char *)fpage,VERSTR)) != NULL)
	{
		cur_ptr+= (sizeof(VERSTR)-1);
		cur_ptr = find_next_token(cur_ptr,token);
		if(cur_ptr)
			*version = (unsigned short)convert_str_to_number(token);
	}
	
	return cur_ptr;
}
#endif 

LOCAL unsigned char *find_next_val(unsigned char *fpage, unsigned int *value)
{
	unsigned char *cur_ptr;
	unsigned char token[MAX_TOKEN];
	cur_ptr = find_next_token(fpage, token);
	if(cur_ptr)
		*value = (unsigned short)convert_str_to_number(token);

	return cur_ptr;
}
#if 0
LOCAL int check_param_file(void)
{
#if 0
	fs_rsp_msg_type rsp_msg;

	if (camsensor_id == (camsensor_sensor_model_type)0)  // '0' means main camera.
		fs_nametest(CAMSENSOR_MAIN_PARAM_FILE, FS_TEST_FILE, NULL, &rsp_msg);
	else
		fs_nametest(CAMSENSOR_SUB_PARAM_FILE, FS_TEST_FILE, NULL, &rsp_msg);
#endif
	return TRUE;
}
#endif
LOCAL byte* open_param_file(void)
{
	struct file *flip;
  int file_size, read_size;
  byte* param_buffer = NULL;

  // Kernel momory access setting
  mm_segment_t old_fs = get_fs();
  set_fs(KERNEL_DS);
	
	flip = filp_open(CAMSENSOR_SUB_PARAM_FILE, O_RDONLY |O_LARGEFILE, S_IRUSR);
	if (IS_ERR(flip)) {
		printk("sensor init file does not exist.\n");
		goto err_open;
	}

  file_size = flip->f_op->llseek(flip, (loff_t)0, SEEK_END) + 0x02; // add dummy
  printk("file size : %d\n", file_size);

	param_buffer = kmalloc(file_size, GFP_KERNEL);	
	if(!param_buffer) {
		printk("mem alloc error!\n");
		goto err_mem;
	}

  flip->f_pos = 0;
	read_size = flip->f_op->read(flip, param_buffer, file_size - 2, &flip->f_pos);
  param_buffer[file_size - 1] = param_buffer[file_size - 2] = '\x00';
  printk("read size : %d\n", read_size);

err_mem: 
	filp_close(flip, NULL);
err_open:
  // restore kernel memory setting
  set_fs(old_fs);

	return param_buffer;
}

LOCAL byte* fn_default_parse_reg(byte* curr_ptr, common_reg_type* reg_item)
{
	unsigned int val1, val2;

	curr_ptr = find_next_val(curr_ptr, &val1);
	if (curr_ptr == NULL)
		return NULL;
	curr_ptr = find_next_val(curr_ptr, &val2);

	reg_item->reg.addr = (byte)val1;
	reg_item->reg.val = (byte)val2;

	return curr_ptr;
}

LOCAL byte* fn_default_parse_reg_var4(byte* curr_ptr, common_reg_type* reg_item)
{
	unsigned int val1, val2, val3;

	curr_ptr = find_next_val(curr_ptr, &val1);
	if (curr_ptr == NULL)
		return NULL;
	curr_ptr = find_next_val(curr_ptr, &val2);
	curr_ptr = find_next_val(curr_ptr, &val3);

	reg_item->reg_var4.addr = (byte)val1;
	reg_item->reg_var4.len = (byte)val3;
	if (val3 == 1)
		reg_item->reg_var4.vals.val8[0] = (byte)val2;
	else
		reg_item->reg_var4.vals.val32 = val2;

	return curr_ptr;
}

LOCAL byte* fn_default_parse_mem(byte* curr_ptr, common_reg_type* reg_item)
{
	unsigned int val1, val2;

	curr_ptr = find_next_val(curr_ptr, &val1);
	if (curr_ptr == NULL)
		return NULL;	
	curr_ptr = find_next_val(curr_ptr, &val2);

	reg_item->mem.addr = val1;
	reg_item->mem.val = val2;

	return curr_ptr;
}

LOCAL byte* fn_default_parse_mem_var4(byte* curr_ptr, common_reg_type* reg_item)
{
	unsigned int val1, val2, val3;

	curr_ptr = find_next_val(curr_ptr, &val1);
	if (curr_ptr == NULL)
		return NULL;
	curr_ptr = find_next_val(curr_ptr, &val2);
	curr_ptr = find_next_val(curr_ptr, &val3);

	reg_item->mem_var4.addr = val1;
	reg_item->mem_var4.len = (byte)val3;
	if (val3 == 1)
		reg_item->mem_var4.vals.val8[0] = (byte)val2;
	else
		reg_item->mem_var4.vals.val32 = val2;

	return curr_ptr;
}

LOCAL int parse_param_file(byte *param_buffer, common_reg_enum_type en_type)
{
	byte* (*fn_ptr)(byte*, common_reg_type*);
	common_reg_list_type* param_list;
	int param_list_idx;
	byte* curr_ptr;

	switch(en_type)
	{
		case COMMON_REG_REG :
			fn_ptr = fn_default_parse_reg;
			break;
		case COMMON_REG_REG_VAR4 :
			fn_ptr = fn_default_parse_reg_var4;
			break;
		case COMMON_REG_MEM :
			fn_ptr = fn_default_parse_mem;
			break;
		case COMMON_REG_MEM_VAR4 :
			fn_ptr = fn_default_parse_mem_var4;
			break;
		default:
			printk("PARM: Unknown type...\n");
	}
	
	curr_ptr = param_buffer;
	param_list = (common_reg_list_type *)param_buffer;
	//param_list->list_regs = (common_reg_type*)((dword)param_list + FPOS(common_reg_list_type, list_regs));
	param_list_idx = 0;
	while(curr_ptr)
		curr_ptr = fn_ptr(curr_ptr, &param_list->list_regs[param_list_idx++]);
	param_list->num_regs = param_list_idx - 1;
	
	return TRUE;
}

void common_register_init(common_reg_enum_type en_type, common_reg_list_type** ppstRegisterList)
{
	byte *param_buff;
	
	//if (unlikely(check_param_file()==FALSE) || ((param_buff = open_param_file()) == NULL))
	if ((param_buff = open_param_file()) == NULL)
	{
		*ppstRegisterList = NULL;
		return;
	}

	*ppstRegisterList = (common_reg_list_type*)param_buff;
	parse_param_file(param_buff, en_type);
	
}

