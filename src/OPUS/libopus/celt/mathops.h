/* Copyright (c) 2002-2008 Jean-Marc Valin
   Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/**
   @file mathops.h
   @brief Various math functions
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

#ifndef MATHOPS_H
#define MATHOPS_H

#include "arch.h"
#include "entcode.h"
#include "os_support.h"

//#define PI 3.141592653f

/* Multiplies two 16-bit fractional values. Bit-exactness of this macro is important */
#define FRAC_MUL16(a,b) ((16384+((int32_t)(int16_t)(a)*(int16_t)(b)))>>15)

unsigned isqrt32(uint32_t _val);

static OPUS_INLINE opus_val32 celt_maxabs16(const opus_val16 *x, int len)
{
   int i;
   opus_val16 maxval = 0;
   opus_val16 minval = 0;
   for (i=0;i<len;i++)
   {
      maxval = MAX16(maxval, x[i]);
      minval = MIN16(minval, x[i]);
   }
   return MAX32(EXTEND32(maxval),-EXTEND32(minval));
}

static OPUS_INLINE opus_val32 celt_maxabs32(const opus_val32 *x, int len)
{
   int i;
   opus_val32 maxval = 0;
   opus_val32 minval = 0;
   for (i=0;i<len;i++)
   {
      maxval = MAX32(maxval, x[i]);
      minval = MIN32(minval, x[i]);
   }
   return MAX32(maxval, -minval);
}


#include "os_support.h"

#ifndef OVERRIDE_CELT_ILOG2
/** Integer log in base2. Undefined for zero and negative numbers */
static OPUS_INLINE int16_t celt_ilog2(int32_t x)
{
   celt_sig_assert(x>0);
   return EC_ILOG(x)-1;
}
#endif


/** Integer log in base2. Defined for zero, but not for negative numbers */
static OPUS_INLINE int16_t celt_zlog2(opus_val32 x)
{
   return x <= 0 ? 0 : celt_ilog2(x);
}

opus_val16 celt_rsqrt_norm(opus_val32 x);

opus_val32 celt_sqrt(opus_val32 x);

opus_val16 celt_cos_norm(opus_val32 x);

/** Base-2 logarithm approximation (log2(x)). (Q14 input, Q10 output) */
static OPUS_INLINE opus_val16 celt_log2(opus_val32 x)
{
   int i;
   opus_val16 n, frac;
   /* -0.41509302963303146, 0.9609890551383969, -0.31836011537636605,
       0.15530808010959576, -0.08556153059057618 */
   static const opus_val16 C[5] = {-6801+(1<<(13-DB_SHIFT)), 15746, -5217, 2545, -1401};
   if (x==0)
      return -32767;
   i = celt_ilog2(x);
   n = VSHR32(x,i-15)-32768-16384;
   frac = ADD16(C[0], MULT16_16_Q15(n, ADD16(C[1], MULT16_16_Q15(n, ADD16(C[2], MULT16_16_Q15(n, ADD16(C[3], MULT16_16_Q15(n, C[4]))))))));
   return SHL16(i-13,DB_SHIFT)+SHR16(frac,14-DB_SHIFT);
}

/*
 K0 = 1
 K1 = log(2)
 K2 = 3-4*log(2)
 K3 = 3*log(2) - 2
*/
#define D0 16383
#define D1 22804
#define D2 14819
#define D3 10204

static OPUS_INLINE opus_val32 celt_exp2_frac(opus_val16 x)
{
   opus_val16 frac;
   frac = SHL16(x, 4);
   return ADD16(D0, MULT16_16_Q15(frac, ADD16(D1, MULT16_16_Q15(frac, ADD16(D2 , MULT16_16_Q15(D3,frac))))));
}
/** Base-2 exponential approximation (2^x). (Q10 input, Q16 output) */
static OPUS_INLINE opus_val32 celt_exp2(opus_val16 x)
{
   int integer;
   opus_val16 frac;
   integer = SHR16(x,10);
   if (integer>14)
      return 0x7f000000;
   else if (integer < -15)
      return 0;
   frac = celt_exp2_frac(x-SHL16(integer,10));
   return VSHR32(EXTEND32(frac), -integer-2);
}

opus_val32 celt_rcp(opus_val32 x);

#define celt_div(a,b) MULT32_32_Q31((opus_val32)(a),celt_rcp(b))

opus_val32 frac_div32(opus_val32 a, opus_val32 b);

#define M1 32767
#define M2 -21
#define M3 -11943
#define M4 4936

/* Atan approximation using a 4th order polynomial. Input is in Q15 format
   and normalized by pi/4. Output is in Q15 format */
static OPUS_INLINE opus_val16 celt_atan01(opus_val16 x)
{
   return MULT16_16_P15(x, ADD32(M1, MULT16_16_P15(x, ADD32(M2, MULT16_16_P15(x, ADD32(M3, MULT16_16_P15(M4, x)))))));
}

#undef M1
#undef M2
#undef M3
#undef M4

/* atan2() approximation valid for positive input values */
static OPUS_INLINE opus_val16 celt_atan2p(opus_val16 y, opus_val16 x)
{
   if (y < x)
   {
      opus_val32 arg;
      arg = celt_div(SHL32(EXTEND32(y),15),x);
      if (arg >= 32767)
         arg = 32767;
      return SHR16(celt_atan01(EXTRACT16(arg)),1);
   } else {
      opus_val32 arg;
      arg = celt_div(SHL32(EXTEND32(x),15),y);
      if (arg >= 32767)
         arg = 32767;
      return 25736-SHR16(celt_atan01(EXTRACT16(arg)),1);
   }
}

#endif /* MATHOPS_H */
