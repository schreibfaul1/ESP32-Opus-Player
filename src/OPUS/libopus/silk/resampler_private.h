/***********************************************************************
Copyright (c) 2006-2011, Skype Limited. All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
- Neither the name of Internet Society, IETF or IETF Trust, nor the
names of specific contributors, may be used to endorse or promote
products derived from this software without specific prior written
permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef SILK_RESAMPLER_PRIVATE_H
#define SILK_RESAMPLER_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SigProc_FIX.h"
#include "resampler_structs.h"
#include "resampler_rom.h"



/* Description: Hybrid IIR/FIR polyphase implementation of resampling */
void silk_resampler_private_IIR_FIR(
    void                            *SS,            /* I/O  Resampler state             */
    int16_t                      out[],          /* O    Output signal               */
    const int16_t                in[],           /* I    Input signal                */
    int32_t                      inLen           /* I    Number of input samples     */
);

/* Description: Hybrid IIR/FIR polyphase implementation of resampling */
void silk_resampler_private_down_FIR(
    void                            *SS,            /* I/O  Resampler state             */
    int16_t                      out[],          /* O    Output signal               */
    const int16_t                in[],           /* I    Input signal                */
    int32_t                      inLen           /* I    Number of input samples     */
);

/* Upsample by a factor 2, high quality */
void silk_resampler_private_up2_HQ_wrapper(
    void                            *SS,            /* I/O  Resampler state (unused)    */
    int16_t                      *out,           /* O    Output signal [ 2 * len ]   */
    const int16_t                *in,            /* I    Input signal [ len ]        */
    int32_t                      len             /* I    Number of input samples     */
);

/* Upsample by a factor 2, high quality */
void silk_resampler_private_up2_HQ(
    int32_t                      *S,             /* I/O  Resampler state [ 6 ]       */
    int16_t                      *out,           /* O    Output signal [ 2 * len ]   */
    const int16_t                *in,            /* I    Input signal [ len ]        */
    int32_t                      len             /* I    Number of input samples     */
);

/* Second order AR filter */
void silk_resampler_private_AR2(
    int32_t                      S[],            /* I/O  State vector [ 2 ]          */
    int32_t                      out_Q8[],       /* O    Output signal               */
    const int16_t                in[],           /* I    Input signal                */
    const int16_t                A_Q14[],        /* I    AR coefficients, Q14        */
    int32_t                      len             /* I    Signal length               */
);

#ifdef __cplusplus
}
#endif
#endif /* SILK_RESAMPLER_PRIVATE_H */
