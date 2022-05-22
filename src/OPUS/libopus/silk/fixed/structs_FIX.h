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

#ifndef SILK_STRUCTS_FIX_H
#define SILK_STRUCTS_FIX_H

#include "../typedef.h"
#include "../main.h"
#include "../structs.h"

#ifdef __cplusplus
extern "C"
{
#endif

/********************************/
/* Noise shaping analysis state */
/********************************/
typedef struct {
    int8_t                   LastGainIndex;
    int32_t                  HarmBoost_smth_Q16;
    int32_t                  HarmShapeGain_smth_Q16;
    int32_t                  Tilt_smth_Q16;
} silk_shape_state_FIX;

/********************************/
/* Encoder state FIX            */
/********************************/
typedef struct {
    silk_encoder_state          sCmn;                                   /* Common struct, shared with floating-point code       */
    silk_shape_state_FIX        sShape;                                 /* Shape state                                          */

    /* Buffer for find pitch and noise shape analysis */
    silk_DWORD_ALIGN int16_t x_buf[ 2 * MAX_FRAME_LENGTH + LA_SHAPE_MAX ];/* Buffer for find pitch and noise shape analysis  */
    int32_t                    LTPCorr_Q15;                            /* Normalized correlation from pitch lag estimator      */
    int32_t                    resNrgSmth;
} silk_encoder_state_FIX;

/************************/
/* Encoder control FIX  */
/************************/
typedef struct {
    /* Prediction and coding parameters */
    int32_t                  Gains_Q16[ MAX_NB_SUBFR ];
    silk_DWORD_ALIGN int16_t PredCoef_Q12[ 2 ][ MAX_LPC_ORDER ];
    int16_t                  LTPCoef_Q14[ LTP_ORDER * MAX_NB_SUBFR ];
    int32_t                    LTP_scale_Q14;
    int32_t                    pitchL[ MAX_NB_SUBFR ];

    /* Noise shaping parameters */
    /* Testing */
    silk_DWORD_ALIGN int16_t AR_Q13[ MAX_NB_SUBFR * MAX_SHAPE_LPC_ORDER ];
    int32_t                  LF_shp_Q14[        MAX_NB_SUBFR ];      /* Packs two int16 coefficients per int32 value         */
    int32_t                    Tilt_Q14[          MAX_NB_SUBFR ];
    int32_t                    HarmShapeGain_Q14[ MAX_NB_SUBFR ];
    int32_t                    Lambda_Q10;
    int32_t                    input_quality_Q14;
    int32_t                    coding_quality_Q14;

    /* measures */
    int32_t                 predGain_Q16;
    int32_t                    LTPredCodGain_Q7;
    int32_t                  ResNrg[ MAX_NB_SUBFR ];                 /* Residual energy per subframe                         */
    int32_t                    ResNrgQ[ MAX_NB_SUBFR ];                /* Q domain for the residual energy > 0                 */

    /* Parameters for CBR mode */
    int32_t                  GainsUnq_Q16[ MAX_NB_SUBFR ];
    int8_t                   lastGainIndexPrev;
} silk_encoder_control_FIX;

/************************/
/* Encoder Super Struct */
/************************/
typedef struct {
    silk_encoder_state_FIX      state_Fxx[ ENCODER_NUM_CHANNELS ];
    stereo_enc_state            sStereo;
    int32_t                  nBitsUsedLBRR;
    int32_t                  nBitsExceeded;
    int32_t                    nChannelsAPI;
    int32_t                    nChannelsInternal;
    int32_t                    nPrevChannelsInternal;
    int32_t                    timeSinceSwitchAllowed_ms;
    int32_t                    allowBandwidthSwitch;
    int32_t                    prev_decode_only_middle;
} silk_encoder;


#ifdef __cplusplus
}
#endif

#endif
