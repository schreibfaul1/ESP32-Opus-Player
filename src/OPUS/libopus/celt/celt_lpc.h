/* Copyright (c) 2009-2010 Xiph.Org Foundation
   Written by Jean-Marc Valin */
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

#ifndef PLC_H
#define PLC_H

#include "arch.h"
#include "cpu_support.h"

#if defined(OPUS_X86_MAY_HAVE_SSE4_1)
#include "x86/celt_lpc_sse.h"
#endif

#define LPC_ORDER 24

void _celt_lpc(int16_t *_lpc, const int32_t *ac, int p);

void celt_fir_c(
         const int16_t *x,
         const int16_t *num,
         int16_t *y,
         int N,
         int ord,
         int arch);

#if !defined(OVERRIDE_CELT_FIR)
#define celt_fir(x, num, y, N, ord, arch) \
    (celt_fir_c(x, num, y, N, ord, arch))
#endif

void celt_iir(const int32_t *x,
         const int16_t *den,
         int32_t *y,
         int N,
         int ord,
         int16_t *mem,
         int arch);

int _celt_autocorr(const int16_t *x, int32_t *ac,
         const int16_t *window, int overlap, int lag, int n, int arch);

#endif /* PLC_H */
