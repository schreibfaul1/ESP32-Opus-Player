/* Copyright (c) 2003-2008 Jean-Marc Valin
   Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/**
   @file arch.h
   @brief Various architecture definitions for CELT
*/
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ARCH_H
#define ARCH_H

#include "../opus_defines.h"
#include "Arduino.h"



#define opus_likely(x)       (__builtin_expect(!!(x), 1))
#define opus_unlikely(x)     (__builtin_expect(!!(x), 0))


#define CELT_SIG_SCALE 32768.f

#define CELT_FATAL(str) celt_fatal(str, __FILE__, __LINE__);


#define celt_assert(cond)
#define celt_assert2(cond, message)
#define MUST_SUCCEED(call) do {if((call) != OPUS_OK) {RESTORE_STACK; return OPUS_INTERNAL_ERROR;} } while (0)


#define celt_sig_assert(cond)


#define IMUL32(a,b) ((a)*(b))

#define MIN16(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum 16-bit value.   */
#define MAX16(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum 16-bit value.   */
#define MIN32(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum 32-bit value.   */
#define MAX32(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum 32-bit value.   */
#define IMIN(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum int value.   */
#define IMAX(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum int value.   */
#define UADD32(a,b) ((a)+(b))
#define USUB32(a,b) ((a)-(b))

/* Set this if int64_t is a native type of the CPU. */
/* Assume that all LP64 architectures have fast 64-bit types; also x86_64
   (which can be ILP32 for x32) and Win64 (which is LLP64). */

#define OPUS_FAST_INT64 0

#define PRINT_MIPS(file)

typedef int16_t opus_val16;
typedef int32_t opus_val32;
typedef int64_t opus_val64;

typedef opus_val32 celt_sig;
typedef opus_val16 celt_norm;
typedef opus_val32 celt_ener;

#define celt_isnan(x) 0

#define Q15ONE 32767

#define SIG_SHIFT 12
/* Safe saturation value for 32-bit signals. Should be less than
   2^31*(1-0.85) to avoid blowing up on DC at deemphasis.*/
#define SIG_SAT (300000000)

#define NORM_SCALING 16384

#define DB_SHIFT 10

#define EPSILON 1
#define VERY_SMALL 0
#define VERY_LARGE16 ((opus_val16)32767)
#define Q15_ONE ((opus_val16)32767)

#define SCALEIN(a)      (a)
#define SCALEOUT(a)     (a)

#define ABS16(x) ((x) < 0 ? (-(x)) : (x))
#define ABS32(x) ((x) < 0 ? (-(x)) : (x))

static OPUS_INLINE int16_t SAT16(int32_t x) {
   return x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
}

#include "fixed_generic.h"
#define GLOBAL_STACK_SIZE 120000


#endif /* ARCH_H */
