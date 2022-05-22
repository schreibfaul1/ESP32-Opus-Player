/* Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/**
   @file pitch.h
   @brief Pitch analysis
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

#ifndef PITCH_H
#define PITCH_H

#include "modes.h"
#include "cpu_support.h"

void pitch_downsample(celt_sig * __restrict__ x[], int16_t * __restrict__ x_lp,
      int len, int C, int arch);

void pitch_search(const int16_t * __restrict__ x_lp, int16_t * __restrict__ y,
                  int len, int max_pitch, int *pitch, int arch);

int16_t remove_doubling(int16_t *x, int maxperiod, int minperiod,
      int N, int *T0, int prev_period, int16_t prev_gain, int arch);


/* OPT: This is the kernel you really want to optimize. It gets used a lot
   by the prefilter and by the PLC. */
static OPUS_INLINE void xcorr_kernel_c(const int16_t * x, const int16_t * y, int32_t sum[4], int len)
{
   int j;
   int16_t y_0, y_1, y_2, y_3;
   celt_assert(len>=3);
   y_3=0; /* gcc doesn't realize that y_3 can't be used uninitialized */
   y_0=*y++;
   y_1=*y++;
   y_2=*y++;
   for (j=0;j<len-3;j+=4)
   {
      int16_t tmp;
      tmp = *x++;
      y_3=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_0);
      sum[1] = MAC16_16(sum[1],tmp,y_1);
      sum[2] = MAC16_16(sum[2],tmp,y_2);
      sum[3] = MAC16_16(sum[3],tmp,y_3);
      tmp=*x++;
      y_0=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_1);
      sum[1] = MAC16_16(sum[1],tmp,y_2);
      sum[2] = MAC16_16(sum[2],tmp,y_3);
      sum[3] = MAC16_16(sum[3],tmp,y_0);
      tmp=*x++;
      y_1=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_2);
      sum[1] = MAC16_16(sum[1],tmp,y_3);
      sum[2] = MAC16_16(sum[2],tmp,y_0);
      sum[3] = MAC16_16(sum[3],tmp,y_1);
      tmp=*x++;
      y_2=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_3);
      sum[1] = MAC16_16(sum[1],tmp,y_0);
      sum[2] = MAC16_16(sum[2],tmp,y_1);
      sum[3] = MAC16_16(sum[3],tmp,y_2);
   }
   if (j++<len)
   {
      int16_t tmp = *x++;
      y_3=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_0);
      sum[1] = MAC16_16(sum[1],tmp,y_1);
      sum[2] = MAC16_16(sum[2],tmp,y_2);
      sum[3] = MAC16_16(sum[3],tmp,y_3);
   }
   if (j++<len)
   {
      int16_t tmp=*x++;
      y_0=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_1);
      sum[1] = MAC16_16(sum[1],tmp,y_2);
      sum[2] = MAC16_16(sum[2],tmp,y_3);
      sum[3] = MAC16_16(sum[3],tmp,y_0);
   }
   if (j<len)
   {
      int16_t tmp=*x++;
      y_1=*y++;
      sum[0] = MAC16_16(sum[0],tmp,y_2);
      sum[1] = MAC16_16(sum[1],tmp,y_3);
      sum[2] = MAC16_16(sum[2],tmp,y_0);
      sum[3] = MAC16_16(sum[3],tmp,y_1);
   }
}

#ifndef OVERRIDE_XCORR_KERNEL
#define xcorr_kernel(x, y, sum, len, arch) \
    ((void)(arch),xcorr_kernel_c(x, y, sum, len))
#endif /* OVERRIDE_XCORR_KERNEL */


static OPUS_INLINE void dual_inner_prod_c(const int16_t *x, const int16_t *y01, const int16_t *y02,
      int N, int32_t *xy1, int32_t *xy2)
{
   int i;
   int32_t xy01=0;
   int32_t xy02=0;
   for (i=0;i<N;i++)
   {
      xy01 = MAC16_16(xy01, x[i], y01[i]);
      xy02 = MAC16_16(xy02, x[i], y02[i]);
   }
   *xy1 = xy01;
   *xy2 = xy02;
}

#ifndef OVERRIDE_DUAL_INNER_PROD
# define dual_inner_prod(x, y01, y02, N, xy1, xy2, arch) \
    ((void)(arch),dual_inner_prod_c(x, y01, y02, N, xy1, xy2))
#endif

/*We make sure a C version is always available for cases where the overhead of
  vectorization and passing around an arch flag aren't worth it.*/
static OPUS_INLINE int32_t celt_inner_prod_c(const int16_t *x,
      const int16_t *y, int N)
{
   int i;
   int32_t xy=0;
   for (i=0;i<N;i++)
      xy = MAC16_16(xy, x[i], y[i]);
   return xy;
}


# define celt_inner_prod(x, y, N, arch) \
    ((void)(arch),celt_inner_prod_c(x, y, N))





int32_t celt_pitch_xcorr_c(const int16_t *_x, const int16_t *_y,
      int32_t *xcorr, int len, int max_pitch, int arch);

# define celt_pitch_xcorr celt_pitch_xcorr_c


#endif
