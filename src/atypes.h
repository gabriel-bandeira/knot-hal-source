/*
 * Copyright (c) 2015, CESAR.
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license. See the LICENSE file for details.
 *
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef ARDUINO
#include <avr/pgmspace.h>
#else
#define	PROGMEM
#define	memcpy_P	memcpy
#endif

#ifndef	__TYPES_H__
#define	__TYPES_H__

/* Macros */
#define _MIN(a,b)						((a)<(b)?(a):(b))
#define _MAX(a,b)						((a)>(b)?(a):(b))
#define _ABS(x)							((x)>0?(x):-(x))
#define _CONSTRAIN(x,l,h)		((x)<(l)?(l):((x)>(h)?(h):(x)))

// invalid socket fd
#define SOCKET_INVALID		-1

// operation status codes
#define SUCCESS		0
#define ERROR			-1

#ifdef __cplusplus
extern "C"{
#endif

/* Fast types definition for the small platforms */
typedef uint_fast8_t		byte_t;
typedef int_fast8_t			char_t;
typedef uint_fast16_t		uint_t;
typedef int_fast16_t		int_t;
typedef char_t					result_t;
typedef void *					pdata_t;
typedef uint_t					len_t;
#define	LEN_T_MAX		((uint_fast16_t)-1)

typedef unsigned long	ulong_t;
#define ULONG_T_MAX	((ulong_t)-1)

#ifdef __GNUC__
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#else
#define likely(x)       (x)
#define unlikely(x)     (x)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif  // __TYPES_H__
