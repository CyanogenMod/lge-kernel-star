/*****************************************************************************

	Copyright(c) 2008 LG Electronics Inc. All Rights Reserved

	File name : tdmb_type.h

	Description : type definitions

    Hoistory
	----------------------------------------------------------------------
	Mar. 18, 2009:		reallee		create

*******************************************************************************/

#ifndef	__TDMB_TYPE_H__
#define __TDMB_TYPE_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef OK
#undef OK
#endif
#define OK 0
#ifdef ERROR
#undef ERROR
#endif
#define ERROR -1
#ifndef LOCAL
#define LOCAL static
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE  1
#endif
#ifndef	NULL
#define	NULL 0
#endif

#define DMB_SUCCESS		(0)
#define DMB_FAILURE		(-1)
#define DMB_WAIT 		0xFFFFFFFFUL
#define DMB_NO_WAIT		0
#define DMB_UNUSED_PARAMETER(x) ((void)(x))

//typedef int boolean;
#ifndef boolean
typedef unsigned char      boolean;
#endif

#ifndef BOOL
typedef int BOOL;
#endif

#ifndef STATUS
typedef int STATUS;
#endif

#ifndef UINT64
typedef unsigned long long 	UINT64;
#endif

#ifndef uint64
typedef unsigned long long 	uint64;
#endif

#ifndef INT64
typedef long long 			INT64;
#endif

#ifndef int64
typedef long long 			int64;
#endif

#ifndef UINT32
typedef unsigned int		UINT32;
#endif

#ifndef	uint32
typedef unsigned int		uint32;
#endif

#ifndef INT32
typedef signed int			INT32;
#endif

#ifndef int32
typedef signed int			int32;
#endif

#ifndef UINT16
typedef unsigned short		UINT16;
#endif

#ifndef uint16
typedef unsigned short		uint16;
#endif

#ifndef INT16
typedef signed short		INT16;
#endif

#ifndef int16
typedef signed short		int16;
#endif

#ifndef INT8
typedef signed char			INT8;
#endif

#ifndef int8
typedef signed char			int8;
#endif

#ifndef UINT8
typedef unsigned char		UINT8;
#endif

#ifndef uint8
typedef unsigned char		uint8;
#endif

#ifndef PUINT32
typedef unsigned int		*PUINT32;
#endif

#ifndef CHAR
typedef char				CHAR;
#endif

#ifndef UCHAR
typedef unsigned char		UCHAR;
#endif

#ifndef USHORT
typedef unsigned short		USHORT;
#endif

#ifndef SHORT
typedef short				SHORT;
#endif

#ifndef ULONG
typedef unsigned long		ULONG;
#endif

#ifndef dword
typedef unsigned long		dword;
#endif

#ifndef Byte
typedef unsigned char		Byte;
#endif

#ifndef byte
typedef unsigned char		byte;
#endif

#ifndef Word16
typedef unsigned short		Word16;
#endif

#ifndef Word32
typedef unsigned int		Word32;
#endif

#ifndef Word
typedef Word32				Word;
#endif

#ifndef word
typedef Word32				word;
#endif

#ifndef SINT8
typedef signed char			SINT8;
#endif

#ifndef SINT16
typedef signed short		SINT16;
#endif

#ifndef SINT32
typedef signed long			SINT32;
#endif

#ifndef BOOLEAN
typedef unsigned char		BOOLEAN;
#endif

#ifndef Bool
typedef unsigned int		Bool; 
#endif
typedef int(*dmb_function_type)(void *);
typedef int(*dmb_callback_type)(int);

#ifdef __cplusplus
}
#endif

#endif	/* !__TDMB_TYPE_H__ */

