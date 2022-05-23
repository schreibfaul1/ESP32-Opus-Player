/* Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/**
   @file pitch.c
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

#include "pitch.h"
#include "os_support.h"
#include "modes.h"
#include "stack_alloc.h"
#include "mathops.h"
#include "celt_lpc.h"

static void find_best_pitch(int32_t *xcorr, int16_t *y, int len,
                            int max_pitch, int *best_pitch
                            , int yshift, int32_t maxcorr
                            )
{
   int i, j;
   int32_t Syy=1;
   int16_t best_num[2];
   int32_t best_den[2];
   int xshift;

   xshift = celt_ilog2(maxcorr)-14;
   best_num[0] = -1;
   best_num[1] = -1;
   best_den[0] = 0;
   best_den[1] = 0;
   best_pitch[0] = 0;
   best_pitch[1] = 1;
   for (j=0;j<len;j++)
      Syy = ADD32(Syy, SHR32(MULT16_16(y[j],y[j]), yshift));
   for (i=0;i<max_pitch;i++)
   {
      if (xcorr[i]>0)
      {
         int16_t num;
         int32_t xcorr16;
         xcorr16 = EXTRACT16(VSHR32(xcorr[i], xshift));
         num = MULT16_16_Q15(xcorr16,xcorr16);
         if (MULT16_32_Q15(num,best_den[1]) > MULT16_32_Q15(best_num[1],Syy))
         {
            if (MULT16_32_Q15(num,best_den[0]) > MULT16_32_Q15(best_num[0],Syy))
            {
               best_num[1] = best_num[0];
               best_den[1] = best_den[0];
               best_pitch[1] = best_pitch[0];
               best_num[0] = num;
               best_den[0] = Syy;
               best_pitch[0] = i;
            } else {
               best_num[1] = num;
               best_den[1] = Syy;
               best_pitch[1] = i;
            }
         }
      }
      Syy += SHR32(MULT16_16(y[i+len],y[i+len]),yshift) - SHR32(MULT16_16(y[i],y[i]),yshift);
      Syy = MAX32(1, Syy);
   }
}

static void celt_fir5(int16_t *x,
         const int16_t *num,
         int N)
{
   int i;
   int16_t num0, num1, num2, num3, num4;
   int32_t mem0, mem1, mem2, mem3, mem4;
   num0=num[0];
   num1=num[1];
   num2=num[2];
   num3=num[3];
   num4=num[4];
   mem0=0;
   mem1=0;
   mem2=0;
   mem3=0;
   mem4=0;
   for (i=0;i<N;i++)
   {
      int32_t sum = SHL32(EXTEND32(x[i]), 12);
      sum = MAC16_16(sum,num0,mem0);
      sum = MAC16_16(sum,num1,mem1);
      sum = MAC16_16(sum,num2,mem2);
      sum = MAC16_16(sum,num3,mem3);
      sum = MAC16_16(sum,num4,mem4);
      mem4 = mem3;
      mem3 = mem2;
      mem2 = mem1;
      mem1 = mem0;
      mem0 = x[i];
      x[i] = ROUND16(sum, 12);
   }
}


void pitch_downsample(int32_t * __restrict__ x[], int16_t * __restrict__ x_lp,
      int len, int C, int arch)
{
   int i;
   int32_t ac[5];
   int16_t tmp=32767;
   int16_t lpc[4];
   int16_t lpc2[5];
   int16_t c1 = QCONST16(.8f,15);
   int shift;
   int32_t maxabs = celt_maxabs32(x[0], len);
   if (C==2)
   {
      int32_t maxabs_1 = celt_maxabs32(x[1], len);
      maxabs = MAX32(maxabs, maxabs_1);
   }
   if (maxabs<1)
      maxabs=1;
   shift = celt_ilog2(maxabs)-10;
   if (shift<0)
      shift=0;
   if (C==2)
      shift++;
   for (i=1;i<len>>1;i++)
      x_lp[i] = SHR32(HALF32(HALF32(x[0][(2*i-1)]+x[0][(2*i+1)])+x[0][2*i]), shift);
   x_lp[0] = SHR32(HALF32(HALF32(x[0][1])+x[0][0]), shift);
   if (C==2)
   {
      for (i=1;i<len>>1;i++)
         x_lp[i] += SHR32(HALF32(HALF32(x[1][(2*i-1)]+x[1][(2*i+1)])+x[1][2*i]), shift);
      x_lp[0] += SHR32(HALF32(HALF32(x[1][1])+x[1][0]), shift);
   }

   _celt_autocorr(x_lp, ac, NULL, 0,
                  4, len>>1, arch);

   /* Noise floor -40 dB */
   ac[0] += SHR32(ac[0],13);
   /* Lag windowing */
   for (i=1;i<=4;i++)
   {
      /*ac[i] *= exp(-.5*(2*M_PI*.002*i)*(2*M_PI*.002*i));*/
      ac[i] -= MULT16_32_Q15(2*i*i, ac[i]);
   }

   _celt_lpc(lpc, ac, 4);
   for (i=0;i<4;i++)
   {
      tmp = MULT16_16_Q15(QCONST16(.9f,15), tmp);
      lpc[i] = MULT16_16_Q15(lpc[i], tmp);
   }
   /* Add a zero */
   lpc2[0] = lpc[0] + QCONST16(.8f,12);
   lpc2[1] = lpc[1] + MULT16_16_Q15(c1,lpc[0]);
   lpc2[2] = lpc[2] + MULT16_16_Q15(c1,lpc[1]);
   lpc2[3] = lpc[3] + MULT16_16_Q15(c1,lpc[2]);
   lpc2[4] = MULT16_16_Q15(c1,lpc[3]);
   celt_fir5(x_lp, lpc2, len>>1);
}

/* Pure C implementation. */
int32_t
celt_pitch_xcorr_c(const int16_t *_x, const int16_t *_y,
      int32_t *xcorr, int len, int max_pitch, int arch)
{

   int i;
   /*The EDSP version requires that max_pitch is at least 1, and that _x is
      32-bit aligned.
     Since it's hard to put asserts in assembly, put them here.*/
   int32_t maxcorr=1;
   assert(max_pitch>0);
   assert((((unsigned char *)_x-(unsigned char *)NULL)&3)==0);
   for (i=0;i<max_pitch-3;i+=4)
   {
      int32_t sum[4]={0,0,0,0};
      xcorr_kernel(_x, _y+i, sum, len, arch);
      xcorr[i]=sum[0];
      xcorr[i+1]=sum[1];
      xcorr[i+2]=sum[2];
      xcorr[i+3]=sum[3];
      sum[0] = MAX32(sum[0], sum[1]);
      sum[2] = MAX32(sum[2], sum[3]);
      sum[0] = MAX32(sum[0], sum[2]);
      maxcorr = MAX32(maxcorr, sum[0]);
   }
   /* In case max_pitch isn't a multiple of 4, do non-unrolled version. */
   for (;i<max_pitch;i++)
   {
      int32_t sum;
      sum = celt_inner_prod(_x, _y+i, len, arch);
      xcorr[i] = sum;
      maxcorr = MAX32(maxcorr, sum);
   }
   return maxcorr;
}

void pitch_search(const int16_t * __restrict__ x_lp, int16_t * __restrict__ y,
                  int len, int max_pitch, int *pitch, int arch)
{
   int i, j;
   int lag;
   int best_pitch[2]={0,0};
   VARDECL(int16_t, x_lp4);
   VARDECL(int16_t, y_lp4);
   VARDECL(int32_t, xcorr);
   int32_t maxcorr;
   int32_t xmax, ymax;
   int shift=0;
   int offset;

   SAVE_STACK;

   assert(len>0);
   assert(max_pitch>0);
   lag = len+max_pitch;

   ALLOC(x_lp4, len>>2, int16_t);
   ALLOC(y_lp4, lag>>2, int16_t);
   ALLOC(xcorr, max_pitch>>1, int32_t);

   /* Downsample by 2 again */
   for (j=0;j<len>>2;j++)
      x_lp4[j] = x_lp[2*j];
   for (j=0;j<lag>>2;j++)
      y_lp4[j] = y[2*j];

   xmax = celt_maxabs16(x_lp4, len>>2);
   ymax = celt_maxabs16(y_lp4, lag>>2);
   shift = celt_ilog2(MAX32(1, MAX32(xmax, ymax)))-11;
   if (shift>0)
   {
      for (j=0;j<len>>2;j++)
         x_lp4[j] = SHR16(x_lp4[j], shift);
      for (j=0;j<lag>>2;j++)
         y_lp4[j] = SHR16(y_lp4[j], shift);
      /* Use double the shift for a MAC */
      shift *= 2;
   } else {
      shift = 0;
   }

   /* Coarse search with 4x decimation */
   maxcorr =

   celt_pitch_xcorr(x_lp4, y_lp4, xcorr, len>>2, max_pitch>>2, arch);

   find_best_pitch(xcorr, y_lp4, len>>2, max_pitch>>2, best_pitch
                   , 0, maxcorr
                   );

   /* Finer search with 2x decimation */
   maxcorr=1;
   for (i=0;i<max_pitch>>1;i++)
   {
      int32_t sum;
      xcorr[i] = 0;
      if (abs(i-2*best_pitch[0])>2 && abs(i-2*best_pitch[1])>2)
         continue;
      sum = 0;
      for (j=0;j<len>>1;j++)
         sum += SHR32(MULT16_16(x_lp[j],y[i+j]), shift);

      xcorr[i] = MAX32(-1, sum);

      maxcorr = MAX32(maxcorr, sum);

   }
   find_best_pitch(xcorr, y, len>>1, max_pitch>>1, best_pitch
                   , shift+1, maxcorr
                   );

   /* Refine by pseudo-interpolation */
   if (best_pitch[0]>0 && best_pitch[0]<(max_pitch>>1)-1)
   {
      int32_t a, b, c;
      a = xcorr[best_pitch[0]-1];
      b = xcorr[best_pitch[0]];
      c = xcorr[best_pitch[0]+1];
      if ((c-a) > MULT16_32_Q15(QCONST16(.7f,15),b-a))
         offset = 1;
      else if ((a-c) > MULT16_32_Q15(QCONST16(.7f,15),b-c))
         offset = -1;
      else
         offset = 0;
   } else {
      offset = 0;
   }
   *pitch = 2*best_pitch[0]-offset;

   
}

static int16_t compute_pitch_gain(int32_t xy, int32_t xx, int32_t yy)
{
   int32_t x2y2;
   int sx, sy, shift;
   int32_t g;
   int16_t den;
   if (xy == 0 || xx == 0 || yy == 0)
      return 0;
   sx = celt_ilog2(xx)-14;
   sy = celt_ilog2(yy)-14;
   shift = sx + sy;
   x2y2 = SHR32(MULT16_16(VSHR32(xx, sx), VSHR32(yy, sy)), 14);
   if (shift & 1) {
      if (x2y2 < 32768)
      {
         x2y2 <<= 1;
         shift--;
      } else {
         x2y2 >>= 1;
         shift++;
      }
   }
   den = celt_rsqrt_norm(x2y2);
   g = MULT16_32_Q15(den, xy);
   g = VSHR32(g, (shift>>1)-1);
   return EXTRACT16(MIN32(g, 32767));
}

static const int second_check[16] = {0, 0, 3, 2, 3, 2, 5, 2, 3, 2, 3, 2, 5, 2, 3, 2};
int16_t remove_doubling(int16_t *x, int maxperiod, int minperiod,
      int N, int *T0_, int prev_period, int16_t prev_gain, int arch)
{
   int k, i, T, T0;
   int16_t g, g0;
   int16_t pg;
   int32_t xy,xx,yy,xy2;
   int32_t xcorr[3];
   int32_t best_xy, best_yy;
   int offset;
   int minperiod0;
   VARDECL(int32_t, yy_lookup);
   SAVE_STACK;

   minperiod0 = minperiod;
   maxperiod /= 2;
   minperiod /= 2;
   *T0_ /= 2;
   prev_period /= 2;
   N /= 2;
   x += maxperiod;
   if (*T0_>=maxperiod)
      *T0_=maxperiod-1;

   T = T0 = *T0_;
   ALLOC(yy_lookup, maxperiod+1, int32_t);
   dual_inner_prod(x, x, x-T0, N, &xx, &xy, arch);
   yy_lookup[0] = xx;
   yy=xx;
   for (i=1;i<=maxperiod;i++)
   {
      yy = yy+MULT16_16(x[-i],x[-i])-MULT16_16(x[N-i],x[N-i]);
      yy_lookup[i] = MAX32(0, yy);
   }
   yy = yy_lookup[T0];
   best_xy = xy;
   best_yy = yy;
   g = g0 = compute_pitch_gain(xy, xx, yy);
   /* Look for any pitch at T/k */
   for (k=2;k<=15;k++)
   {
      int T1, T1b;
      int16_t g1;
      int16_t cont=0;
      int16_t thresh;
      T1 = celt_udiv(2*T0+k, 2*k);
      if (T1 < minperiod)
         break;
      /* Look for another strong correlation at T1b */
      if (k==2)
      {
         if (T1+T0>maxperiod)
            T1b = T0;
         else
            T1b = T0+T1;
      } else
      {
         T1b = celt_udiv(2*second_check[k]*T0+k, 2*k);
      }
      dual_inner_prod(x, &x[-T1], &x[-T1b], N, &xy, &xy2, arch);
      xy = HALF32(xy + xy2);
      yy = HALF32(yy_lookup[T1] + yy_lookup[T1b]);
      g1 = compute_pitch_gain(xy, xx, yy);
      if (abs(T1-prev_period)<=1)
         cont = prev_gain;
      else if (abs(T1-prev_period)<=2 && 5*k*k < T0)
         cont = HALF16(prev_gain);
      else
         cont = 0;
      thresh = MAX16(QCONST16(.3f,15), MULT16_16_Q15(QCONST16(.7f,15),g0)-cont);
      /* Bias against very high pitch (very short period) to avoid false-positives
         due to short-term correlation */
      if (T1<3*minperiod)
         thresh = MAX16(QCONST16(.4f,15), MULT16_16_Q15(QCONST16(.85f,15),g0)-cont);
      else if (T1<2*minperiod)
         thresh = MAX16(QCONST16(.5f,15), MULT16_16_Q15(QCONST16(.9f,15),g0)-cont);
      if (g1 > thresh)
      {
         best_xy = xy;
         best_yy = yy;
         T = T1;
         g = g1;
      }
   }
   best_xy = MAX32(0, best_xy);
   if (best_yy <= best_xy)
      pg = 32767;
   else
      pg = SHR32(frac_div32(best_xy,best_yy+1),16);

   for (k=0;k<3;k++)
      xcorr[k] = celt_inner_prod(x, x-(T+k-1), N, arch);
   if ((xcorr[2]-xcorr[0]) > MULT16_32_Q15(QCONST16(.7f,15),xcorr[1]-xcorr[0]))
      offset = 1;
   else if ((xcorr[0]-xcorr[2]) > MULT16_32_Q15(QCONST16(.7f,15),xcorr[1]-xcorr[2]))
      offset = -1;
   else
      offset = 0;
   if (pg > g)
      pg = g;
   *T0_ = 2*T+offset;

   if (*T0_<minperiod0)
      *T0_=minperiod0;
   
   return pg;
}
