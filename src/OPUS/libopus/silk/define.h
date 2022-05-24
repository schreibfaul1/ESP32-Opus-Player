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

#ifndef SILK_DEFINE_H
#define SILK_DEFINE_H

#include "errors.h"
#include "typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif







/***************************/
/* Voice activity detector */
/***************************/


/******************/
/* NLSF quantizer */
/******************/
#define NLSF_W_Q                                2
#define NLSF_VQ_MAX_VECTORS                     32
#define NLSF_QUANT_MAX_AMPLITUDE                4
#define NLSF_QUANT_MAX_AMPLITUDE_EXT            10
#define NLSF_QUANT_LEVEL_ADJ                    0.1
#define NLSF_QUANT_DEL_DEC_STATES_LOG2          2
#define NLSF_QUANT_DEL_DEC_STATES               ( 1 << NLSF_QUANT_DEL_DEC_STATES_LOG2 )

/* Transition filtering for mode switching */
#define TRANSITION_TIME_MS                      5120    /* 5120 = 64 * FRAME_LENGTH_MS * ( TRANSITION_INT_NUM - 1 ) = 64*(20*4)*/
#define TRANSITION_NB                           3       /* Hardcoded in tables */
#define TRANSITION_NA                           2       /* Hardcoded in tables */
#define TRANSITION_INT_NUM                      5       /* Hardcoded in tables */
#define TRANSITION_FRAMES                       ( TRANSITION_TIME_MS / MAX_FRAME_LENGTH_MS )
#define TRANSITION_INT_STEPS                    ( TRANSITION_FRAMES  / ( TRANSITION_INT_NUM - 1 ) )

/* BWE factors to apply after packet loss */
#define BWE_AFTER_LOSS_Q16                      63570

/* Defines for CN generation */
#define CNG_BUF_MASK_MAX                        255     /* 2^floor(log2(MAX_FRAME_LENGTH))-1    */
#define CNG_GAIN_SMTH_Q16                       4634    /* 0.25^(1/4)                           */
#define CNG_NLSF_SMTH_Q16                       16348   /* 0.25                                 */

#ifdef __cplusplus
}
#endif

#endif
