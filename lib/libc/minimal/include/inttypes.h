/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2026 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_INTTYPES_H_
#define ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_INTTYPES_H_

#include <limits.h>
#include <stdint.h>

#if INT32_MAX > INT_MAX
#define _PRI32_PREFIX "l"
#else
#define _PRI32_PREFIX ""
#endif

#if INT_FAST32_MAX > INT_MAX
#define _PRIFAST32_PREFIX "l"
#else
#define _PRIFAST32_PREFIX ""
#endif

#if INTPTR_MAX > INT_MAX
#define _PRIPTR_PREFIX "l"
#else
#define _PRIPTR_PREFIX ""
#endif

#define	PRId8			"d"		/* int8_t */
#define	PRId16			"d"		/* int16_t */
#define	PRId32			_PRI32_PREFIX "d"		/* int32_t */
#define	PRId64			"lld"		/* int64_t */
#define	PRIdFAST8		"d"		/* int_fast8_t */
#define	PRIdFAST16		"d"		/* int_fast16_t */
#define	PRIdFAST32		_PRIFAST32_PREFIX "d"		/* int_fast32_t */
#define	PRIdFAST64		"lld"		/* int_fast64_t */
#define	PRIdLEAST8		"d"		/* int_least8_t */
#define	PRIdLEAST16		"d"		/* int_least16_t */
#define	PRIdLEAST32		_PRI32_PREFIX "d"		/* int_least32_t */
#define	PRIdLEAST64		"lld"		/* int_least64_t */
#define	PRIdMAX			"lld"		/* intmax_t */
#define	PRIdPTR			_PRIPTR_PREFIX "d"		/* intptr_t */

#define	PRIi8			"i"		/* int8_t */
#define	PRIi16			"i"		/* int16_t */
#define	PRIi32			_PRI32_PREFIX "i"		/* int32_t */
#define	PRIi64			"lli"		/* int64_t */
#define	PRIiFAST8		"i"		/* int_fast8_t */
#define	PRIiFAST16		"i"		/* int_fast16_t */
#define	PRIiFAST32		_PRIFAST32_PREFIX "i"		/* int_fast32_t */
#define	PRIiFAST64		"lli"		/* int_fast64_t */
#define	PRIiLEAST8		"i"		/* int_least8_t */
#define	PRIiLEAST16		"i"		/* int_least16_t */
#define	PRIiLEAST32		_PRI32_PREFIX "i"		/* int_least32_t */
#define	PRIiLEAST64		"lli"		/* int_least64_t */
#define	PRIiMAX			"lli"		/* intmax_t */
#define	PRIiPTR			_PRIPTR_PREFIX "i"		/* intptr_t */

#define	PRIo8			"o"		/* int8_t */
#define	PRIo16			"o"		/* int16_t */
#define	PRIo32			_PRI32_PREFIX "o"		/* int32_t */
#define	PRIo64			"llo"		/* int64_t */
#define	PRIoFAST8		"o"		/* int_fast8_t */
#define	PRIoFAST16		"o"		/* int_fast16_t */
#define	PRIoFAST32		_PRIFAST32_PREFIX "o"		/* int_fast32_t */
#define	PRIoFAST64		"llo"		/* int_fast64_t */
#define	PRIoLEAST8		"o"		/* int_least8_t */
#define	PRIoLEAST16		"o"		/* int_least16_t */
#define	PRIoLEAST32		_PRI32_PREFIX "o"		/* int_least32_t */
#define	PRIoLEAST64		"llo"		/* int_least64_t */
#define	PRIoMAX			"llo"		/* intmax_t */
#define	PRIoPTR			_PRIPTR_PREFIX "o"		/* intptr_t */

#define	PRIu8			"u"		/* uint8_t */
#define	PRIu16			"u"		/* uint16_t */
#define	PRIu32			_PRI32_PREFIX "u"		/* uint32_t */
#define	PRIu64			"llu"		/* uint64_t */
#define	PRIuFAST8		"u"		/* uint_fast8_t */
#define	PRIuFAST16		"u"		/* uint_fast16_t */
#define	PRIuFAST32		_PRIFAST32_PREFIX "u"		/* uint_fast32_t */
#define	PRIuFAST64		"llu"		/* uint_fast64_t */
#define	PRIuLEAST8		"u"		/* uint_least8_t */
#define	PRIuLEAST16		"u"		/* uint_least16_t */
#define	PRIuLEAST32		_PRI32_PREFIX "u"		/* uint_least32_t */
#define	PRIuLEAST64		"llu"		/* uint_least64_t */
#define	PRIuMAX			"llu"		/* uintmax_t */
#define	PRIuPTR			_PRIPTR_PREFIX "u"		/* uintptr_t */

#define	PRIx8			"x"		/* uint8_t */
#define	PRIx16			"x"		/* uint16_t */
#define	PRIx32			_PRI32_PREFIX "x"		/* uint32_t */
#define	PRIx64			"llx"		/* uint64_t */
#define	PRIxFAST8		"x"		/* uint_fast8_t */
#define	PRIxFAST16		"x"		/* uint_fast16_t */
#define	PRIxFAST32		_PRIFAST32_PREFIX "x"		/* uint_fast32_t */
#define	PRIxFAST64		"llx"		/* uint_fast64_t */
#define	PRIxLEAST8		"x"		/* uint_least8_t */
#define	PRIxLEAST16		"x"		/* uint_least16_t */
#define	PRIxLEAST32		_PRI32_PREFIX "x"		/* uint_least32_t */
#define	PRIxLEAST64		"llx"		/* uint_least64_t */
#define	PRIxMAX			"llx"		/* uintmax_t */
#define	PRIxPTR			_PRIPTR_PREFIX "x"		/* uintptr_t */

#define	PRIX8			"X"		/* uint8_t */
#define	PRIX16			"X"		/* uint16_t */
#define	PRIX32			_PRI32_PREFIX "X"		/* uint32_t */
#define	PRIX64			"llX"		/* uint64_t */
#define	PRIXFAST8		"X"		/* uint_fast8_t */
#define	PRIXFAST16		"X"		/* uint_fast16_t */
#define	PRIXFAST32		_PRIFAST32_PREFIX "X"		/* uint_fast32_t */
#define	PRIXFAST64		"llX"		/* uint_fast64_t */
#define	PRIXLEAST8		"X"		/* uint_least8_t */
#define	PRIXLEAST16		"X"		/* uint_least16_t */
#define	PRIXLEAST32		_PRI32_PREFIX "X"		/* uint_least32_t */
#define	PRIXLEAST64		"llX"		/* uint_least64_t */
#define	PRIXMAX			"llX"		/* uintmax_t */
#define	PRIXPTR			_PRIPTR_PREFIX "X"		/* uintptr_t */

#endif
