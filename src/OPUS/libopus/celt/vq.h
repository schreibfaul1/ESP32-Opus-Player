/* Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/**
   @file vq.h
   @brief Vector quantisation of the residual
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

#ifndef VQ_H
#define VQ_H



void exp_rotation(int16_t *X, int len, int dir, int stride, int K, int spread);

int16_t op_pvq_search_c(int16_t *X, int *iy, int K, int N, int arch);

#if !defined(OVERRIDE_OP_PVQ_SEARCH)
#define op_pvq_search(x, iy, K, N, arch) \
    (op_pvq_search_c(x, iy, K, N, arch))
#endif

/** Algebraic pulse-vector quantiser. The signal x is replaced by the sum of
  * the pitch and a combination of pulses such that its norm is still equal
  * to 1. This is the function that will typically require the most CPU.
 * @param X Residual signal to quantise/encode (returns quantised version)
 * @param N Number of samples to encode
 * @param K Number of pulses to use
 * @param enc Entropy encoder state
 * @ret A mask indicating which blocks in the band received pulses
*/
unsigned alg_quant(int16_t *X, int N, int K, int spread, int B, ec_enc *enc,
      int16_t gain, int resynth, int arch);

/** Algebraic pulse decoder
 * @param X Decoded normalised spectrum (returned)
 * @param N Number of samples to decode
 * @param K Number of pulses to use
 * @param dec Entropy decoder state
 * @ret A mask indicating which blocks in the band received pulses
 */
unsigned alg_unquant(int16_t *X, int N, int K, int spread, int B,
      ec_dec *dec, int16_t gain);

void renormalise_vector(int16_t *X, int N, int16_t gain, int arch);

int stereo_itheta(const int16_t *X, const int16_t *Y, int stereo, int N, int arch);

#endif /* VQ_H */
