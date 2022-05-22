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

#ifndef SILK_MAIN_H
#define SILK_MAIN_H

#include "SigProc_FIX.h"
#include "define.h"
#include "structs.h"
#include "tables.h"
#include "PLC.h"
#include "control.h"
#include "debug.h"
#include "../celt/entenc.h"
#include "../celt/entdec.h"

#if defined(OPUS_X86_MAY_HAVE_SSE4_1)
#include "x86/main_sse.h"
#endif

#if (defined(OPUS_ARM_ASM) || defined(OPUS_ARM_MAY_HAVE_NEON_INTR))
#include "arm/NSQ_del_dec_arm.h"
#endif

/* Convert Left/Right stereo signal to adaptive Mid/Side representation */
void silk_stereo_LR_to_MS(
    stereo_enc_state            *state,                         /* I/O  State                                       */
    int16_t                  x1[],                           /* I/O  Left input signal, becomes mid signal       */
    int16_t                  x2[],                           /* I/O  Right input signal, becomes side signal     */
    int8_t                   ix[ 2 ][ 3 ],                   /* O    Quantization indices                        */
    int8_t                   *mid_only_flag,                 /* O    Flag: only mid signal coded                 */
    int32_t                  mid_side_rates_bps[],           /* O    Bitrates for mid and side signals           */
    int32_t                  total_rate_bps,                 /* I    Total bitrate                               */
    int32_t                    prev_speech_act_Q8,             /* I    Speech activity level in previous frame     */
    int32_t                    toMono,                         /* I    Last frame before a stereo->mono transition */
    int32_t                    fs_kHz,                         /* I    Sample rate (kHz)                           */
    int32_t                    frame_length                    /* I    Number of samples                           */
);

/* Convert adaptive Mid/Side representation to Left/Right stereo signal */
void silk_stereo_MS_to_LR(
    stereo_dec_state            *state,                         /* I/O  State                                       */
    int16_t                  x1[],                           /* I/O  Left input signal, becomes mid signal       */
    int16_t                  x2[],                           /* I/O  Right input signal, becomes side signal     */
    const int32_t            pred_Q13[],                     /* I    Predictors                                  */
    int32_t                    fs_kHz,                         /* I    Samples rate (kHz)                          */
    int32_t                    frame_length                    /* I    Number of samples                           */
);

/* Find least-squares prediction gain for one signal based on another and quantize it */
int32_t silk_stereo_find_predictor(                          /* O    Returns predictor in Q13                    */
    int32_t                  *ratio_Q14,                     /* O    Ratio of residual and mid energies          */
    const int16_t            x[],                            /* I    Basis signal                                */
    const int16_t            y[],                            /* I    Target signal                               */
    int32_t                  mid_res_amp_Q0[],               /* I/O  Smoothed mid, residual norms                */
    int32_t                    length,                         /* I    Number of samples                           */
    int32_t                    smooth_coef_Q16                 /* I    Smoothing coefficient                       */
);

/* Quantize mid/side predictors */
void silk_stereo_quant_pred(
    int32_t                  pred_Q13[],                     /* I/O  Predictors (out: quantized)                 */
    int8_t                   ix[ 2 ][ 3 ]                    /* O    Quantization indices                        */
);

/* Entropy code the mid/side quantization indices */
void silk_stereo_encode_pred(
    ec_enc                      *psRangeEnc,                    /* I/O  Compressor data structure                   */
    int8_t                   ix[ 2 ][ 3 ]                    /* I    Quantization indices                        */
);

/* Entropy code the mid-only flag */
void silk_stereo_encode_mid_only(
    ec_enc                      *psRangeEnc,                    /* I/O  Compressor data structure                   */
    int8_t                   mid_only_flag
);

/* Decode mid/side predictors */
void silk_stereo_decode_pred(
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    int32_t                  pred_Q13[]                      /* O    Predictors                                  */
);

/* Decode mid-only flag */
void silk_stereo_decode_mid_only(
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    int32_t                    *decode_only_mid                /* O    Flag that only mid channel has been coded   */
);

/* Encodes signs of excitation */
void silk_encode_signs(
    ec_enc                      *psRangeEnc,                        /* I/O  Compressor data structure               */
    const int8_t             pulses[],                           /* I    pulse signal                            */
    int32_t                    length,                             /* I    length of input                         */
    const int32_t              signalType,                         /* I    Signal type                             */
    const int32_t              quantOffsetType,                    /* I    Quantization offset type                */
    const int32_t              sum_pulses[ MAX_NB_SHELL_BLOCKS ]   /* I    Sum of absolute pulses per block        */
);

/* Decodes signs of excitation */
void silk_decode_signs(
    ec_dec                      *psRangeDec,                        /* I/O  Compressor data structure               */
    int16_t                  pulses[],                           /* I/O  pulse signal                            */
    int32_t                    length,                             /* I    length of input                         */
    const int32_t              signalType,                         /* I    Signal type                             */
    const int32_t              quantOffsetType,                    /* I    Quantization offset type                */
    const int32_t              sum_pulses[ MAX_NB_SHELL_BLOCKS ]   /* I    Sum of absolute pulses per block        */
);

/* Check encoder control struct */
int32_t check_control_input(
    silk_EncControlStruct        *encControl                    /* I    Control structure                           */
);

/* Control internal sampling rate */
int32_t silk_control_audio_bandwidth(
    silk_encoder_state          *psEncC,                        /* I/O  Pointer to Silk encoder state               */
    silk_EncControlStruct       *encControl                     /* I    Control structure                           */
);

/* Control SNR of redidual quantizer */
int32_t silk_control_SNR(
    silk_encoder_state          *psEncC,                        /* I/O  Pointer to Silk encoder state               */
    int32_t                  TargetRate_bps                  /* I    Target max bitrate (bps)                    */
);

/***************/
/* Shell coder */
/***************/

/* Encode quantization indices of excitation */
void silk_encode_pulses(
    ec_enc                      *psRangeEnc,                    /* I/O  compressor data structure                   */
    const int32_t              signalType,                     /* I    Signal type                                 */
    const int32_t              quantOffsetType,                /* I    quantOffsetType                             */
    int8_t                   pulses[],                       /* I    quantization indices                        */
    const int32_t              frame_length                    /* I    Frame length                                */
);

/* Shell encoder, operates on one shell code frame of 16 pulses */
void silk_shell_encoder(
    ec_enc                      *psRangeEnc,                    /* I/O  compressor data structure                   */
    const int32_t              *pulses0                        /* I    data: nonnegative pulse amplitudes          */
);

/* Shell decoder, operates on one shell code frame of 16 pulses */
void silk_shell_decoder(
    int16_t                  *pulses0,                       /* O    data: nonnegative pulse amplitudes          */
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    const int32_t              pulses4                         /* I    number of pulses per pulse-subframe         */
);

/* Gain scalar quantization with hysteresis, uniform on log scale */
void silk_gains_quant(
    int8_t                   ind[ MAX_NB_SUBFR ],            /* O    gain indices                                */
    int32_t                  gain_Q16[ MAX_NB_SUBFR ],       /* I/O  gains (quantized out)                       */
    int8_t                   *prev_ind,                      /* I/O  last index in previous frame                */
    const int32_t              conditional,                    /* I    first gain is delta coded if 1              */
    const int32_t              nb_subfr                        /* I    number of subframes                         */
);

/* Gains scalar dequantization, uniform on log scale */
void silk_gains_dequant(
    int32_t                  gain_Q16[ MAX_NB_SUBFR ],       /* O    quantized gains                             */
    const int8_t             ind[ MAX_NB_SUBFR ],            /* I    gain indices                                */
    int8_t                   *prev_ind,                      /* I/O  last index in previous frame                */
    const int32_t              conditional,                    /* I    first gain is delta coded if 1              */
    const int32_t              nb_subfr                        /* I    number of subframes                          */
);

/* Compute unique identifier of gain indices vector */
int32_t silk_gains_ID(                                       /* O    returns unique identifier of gains          */
    const int8_t             ind[ MAX_NB_SUBFR ],            /* I    gain indices                                */
    const int32_t              nb_subfr                        /* I    number of subframes                         */
);

/* Interpolate two vectors */
void silk_interpolate(
    int16_t                  xi[ MAX_LPC_ORDER ],            /* O    interpolated vector                         */
    const int16_t            x0[ MAX_LPC_ORDER ],            /* I    first vector                                */
    const int16_t            x1[ MAX_LPC_ORDER ],            /* I    second vector                               */
    const int32_t              ifact_Q2,                       /* I    interp. factor, weight on 2nd vector        */
    const int32_t              d                               /* I    number of parameters                        */
);

/* LTP tap quantizer */
void silk_quant_LTP_gains(
    int16_t                  B_Q14[ MAX_NB_SUBFR * LTP_ORDER ],          /* O    Quantized LTP gains             */
    int8_t                   cbk_index[ MAX_NB_SUBFR ],                  /* O    Codebook Index                  */
    int8_t                   *periodicity_index,                         /* O    Periodicity Index               */
    int32_t                  *sum_gain_dB_Q7,                            /* I/O  Cumulative max prediction gain  */
    int32_t                    *pred_gain_dB_Q7,                           /* O    LTP prediction gain             */
    const int32_t            XX_Q17[ MAX_NB_SUBFR*LTP_ORDER*LTP_ORDER ], /* I    Correlation matrix in Q18       */
    const int32_t            xX_Q17[ MAX_NB_SUBFR*LTP_ORDER ],           /* I    Correlation vector in Q18       */
    const int32_t              subfr_len,                                  /* I    Number of samples per subframe  */
    const int32_t              nb_subfr,                                   /* I    Number of subframes             */
    int                         arch                                        /* I    Run-time architecture           */
);

/* Entropy constrained matrix-weighted VQ, for a single input data vector */
void silk_VQ_WMat_EC_c(
    int8_t                   *ind,                           /* O    index of best codebook vector               */
    int32_t                  *res_nrg_Q15,                   /* O    best residual energy                        */
    int32_t                  *rate_dist_Q8,                  /* O    best total bitrate                          */
    int32_t                    *gain_Q7,                       /* O    sum of absolute LTP coefficients            */
    const int32_t            *XX_Q17,                        /* I    correlation matrix                          */
    const int32_t            *xX_Q17,                        /* I    correlation vector                          */
    const int8_t             *cb_Q7,                         /* I    codebook                                    */
    const uint8_t            *cb_gain_Q7,                    /* I    codebook effective gain                     */
    const uint8_t            *cl_Q5,                         /* I    code length for each codebook vector        */
    const int32_t              subfr_len,                      /* I    number of samples per subframe              */
    const int32_t            max_gain_Q7,                    /* I    maximum sum of absolute LTP coefficients    */
    const int32_t              L                               /* I    number of vectors in codebook               */
);

#if !defined(OVERRIDE_silk_VQ_WMat_EC)
#define silk_VQ_WMat_EC(ind, res_nrg_Q15, rate_dist_Q8, gain_Q7, XX_Q17, xX_Q17, cb_Q7, cb_gain_Q7, cl_Q5, subfr_len, max_gain_Q7, L, arch) \
    ((void)(arch),silk_VQ_WMat_EC_c(ind, res_nrg_Q15, rate_dist_Q8, gain_Q7, XX_Q17, xX_Q17, cb_Q7, cb_gain_Q7, cl_Q5, subfr_len, max_gain_Q7, L))
#endif

/************************************/
/* Noise shaping quantization (NSQ) */
/************************************/

void silk_NSQ_c(
    const silk_encoder_state    *psEncC,                                    /* I    Encoder State                   */
    silk_nsq_state              *NSQ,                                       /* I/O  NSQ state                       */
    SideInfoIndices             *psIndices,                                 /* I/O  Quantization Indices            */
    const int16_t            x16[],                                      /* I    Input                           */
    int8_t                   pulses[],                                   /* O    Quantized pulse signal          */
    const int16_t            PredCoef_Q12[ 2 * MAX_LPC_ORDER ],          /* I    Short term prediction coefs     */
    const int16_t            LTPCoef_Q14[ LTP_ORDER * MAX_NB_SUBFR ],    /* I    Long term prediction coefs      */
    const int16_t            AR_Q13[ MAX_NB_SUBFR * MAX_SHAPE_LPC_ORDER ], /* I  Noise shaping coefs             */
    const int32_t              HarmShapeGain_Q14[ MAX_NB_SUBFR ],          /* I    Long term shaping coefs         */
    const int32_t              Tilt_Q14[ MAX_NB_SUBFR ],                   /* I    Spectral tilt                   */
    const int32_t            LF_shp_Q14[ MAX_NB_SUBFR ],                 /* I    Low frequency shaping coefs     */
    const int32_t            Gains_Q16[ MAX_NB_SUBFR ],                  /* I    Quantization step sizes         */
    const int32_t              pitchL[ MAX_NB_SUBFR ],                     /* I    Pitch lags                      */
    const int32_t              Lambda_Q10,                                 /* I    Rate/distortion tradeoff        */
    const int32_t              LTP_scale_Q14                               /* I    LTP state scaling               */
);

#if !defined(OVERRIDE_silk_NSQ)
#define silk_NSQ(psEncC, NSQ, psIndices, x16, pulses, PredCoef_Q12, LTPCoef_Q14, AR_Q13, \
                   HarmShapeGain_Q14, Tilt_Q14, LF_shp_Q14, Gains_Q16, pitchL, Lambda_Q10, LTP_scale_Q14, arch) \
    ((void)(arch),silk_NSQ_c(psEncC, NSQ, psIndices, x16, pulses, PredCoef_Q12, LTPCoef_Q14, AR_Q13, \
                   HarmShapeGain_Q14, Tilt_Q14, LF_shp_Q14, Gains_Q16, pitchL, Lambda_Q10, LTP_scale_Q14))
#endif

/* Noise shaping using delayed decision */
void silk_NSQ_del_dec_c(
    const silk_encoder_state    *psEncC,                                    /* I    Encoder State                   */
    silk_nsq_state              *NSQ,                                       /* I/O  NSQ state                       */
    SideInfoIndices             *psIndices,                                 /* I/O  Quantization Indices            */
    const int16_t            x16[],                                      /* I    Input                           */
    int8_t                   pulses[],                                   /* O    Quantized pulse signal          */
    const int16_t            PredCoef_Q12[ 2 * MAX_LPC_ORDER ],          /* I    Short term prediction coefs     */
    const int16_t            LTPCoef_Q14[ LTP_ORDER * MAX_NB_SUBFR ],    /* I    Long term prediction coefs      */
    const int16_t            AR_Q13[ MAX_NB_SUBFR * MAX_SHAPE_LPC_ORDER ], /* I  Noise shaping coefs             */
    const int32_t              HarmShapeGain_Q14[ MAX_NB_SUBFR ],          /* I    Long term shaping coefs         */
    const int32_t              Tilt_Q14[ MAX_NB_SUBFR ],                   /* I    Spectral tilt                   */
    const int32_t            LF_shp_Q14[ MAX_NB_SUBFR ],                 /* I    Low frequency shaping coefs     */
    const int32_t            Gains_Q16[ MAX_NB_SUBFR ],                  /* I    Quantization step sizes         */
    const int32_t              pitchL[ MAX_NB_SUBFR ],                     /* I    Pitch lags                      */
    const int32_t              Lambda_Q10,                                 /* I    Rate/distortion tradeoff        */
    const int32_t              LTP_scale_Q14                               /* I    LTP state scaling               */
);

#if !defined(OVERRIDE_silk_NSQ_del_dec)
#define silk_NSQ_del_dec(psEncC, NSQ, psIndices, x16, pulses, PredCoef_Q12, LTPCoef_Q14, AR_Q13, \
                           HarmShapeGain_Q14, Tilt_Q14, LF_shp_Q14, Gains_Q16, pitchL, Lambda_Q10, LTP_scale_Q14, arch) \
    ((void)(arch),silk_NSQ_del_dec_c(psEncC, NSQ, psIndices, x16, pulses, PredCoef_Q12, LTPCoef_Q14, AR_Q13, \
                           HarmShapeGain_Q14, Tilt_Q14, LF_shp_Q14, Gains_Q16, pitchL, Lambda_Q10, LTP_scale_Q14))
#endif

/************/
/* Silk VAD */
/************/
/* Initialize the Silk VAD */
int32_t silk_VAD_Init(                                         /* O    Return value, 0 if success                  */
    silk_VAD_state              *psSilk_VAD                     /* I/O  Pointer to Silk VAD state                   */
);

/* Get speech activity level in Q8 */
int32_t silk_VAD_GetSA_Q8_c(                                   /* O    Return value, 0 if success                  */
    silk_encoder_state          *psEncC,                        /* I/O  Encoder state                               */
    const int16_t            pIn[]                           /* I    PCM input                                   */
);

#if !defined(OVERRIDE_silk_VAD_GetSA_Q8)
#define silk_VAD_GetSA_Q8(psEnC, pIn, arch) ((void)(arch),silk_VAD_GetSA_Q8_c(psEnC, pIn))
#endif

/* Low-pass filter with variable cutoff frequency based on  */
/* piece-wise linear interpolation between elliptic filters */
/* Start by setting transition_frame_no = 1;                */
void silk_LP_variable_cutoff(
    silk_LP_state               *psLP,                          /* I/O  LP filter state                             */
    int16_t                  *frame,                         /* I/O  Low-pass filtered output signal             */
    const int32_t              frame_length                    /* I    Frame length                                */
);

/******************/
/* NLSF Quantizer */
/******************/
/* Limit, stabilize, convert and quantize NLSFs */
void silk_process_NLSFs(
    silk_encoder_state          *psEncC,                            /* I/O  Encoder state                               */
    int16_t                  PredCoef_Q12[ 2 ][ MAX_LPC_ORDER ], /* O    Prediction coefficients                     */
    int16_t                  pNLSF_Q15[         MAX_LPC_ORDER ], /* I/O  Normalized LSFs (quant out) (0 - (2^15-1))  */
    const int16_t            prev_NLSFq_Q15[    MAX_LPC_ORDER ]  /* I    Previous Normalized LSFs (0 - (2^15-1))     */
);

int32_t silk_NLSF_encode(                                    /* O    Returns RD value in Q25                     */
          int8_t             *NLSFIndices,                   /* I    Codebook path vector [ LPC_ORDER + 1 ]      */
          int16_t            *pNLSF_Q15,                     /* I/O  Quantized NLSF vector [ LPC_ORDER ]         */
    const silk_NLSF_CB_struct   *psNLSF_CB,                     /* I    Codebook object                             */
    const int16_t            *pW_QW,                         /* I    NLSF weight vector [ LPC_ORDER ]            */
    const int32_t              NLSF_mu_Q20,                    /* I    Rate weight for the RD optimization         */
    const int32_t              nSurvivors,                     /* I    Max survivors after first stage             */
    const int32_t              signalType                      /* I    Signal type: 0/1/2                          */
);

/* Compute quantization errors for an LPC_order element input vector for a VQ codebook */
void silk_NLSF_VQ(
    int32_t                  err_Q26[],                      /* O    Quantization errors [K]                     */
    const int16_t            in_Q15[],                       /* I    Input vectors to be quantized [LPC_order]   */
    const uint8_t            pCB_Q8[],                       /* I    Codebook vectors [K*LPC_order]              */
    const int16_t            pWght_Q9[],                     /* I    Codebook weights [K*LPC_order]              */
    const int32_t              K,                              /* I    Number of codebook vectors                  */
    const int32_t              LPC_order                       /* I    Number of LPCs                              */
);

/* Delayed-decision quantizer for NLSF residuals */
int32_t silk_NLSF_del_dec_quant(                             /* O    Returns RD value in Q25                     */
    int8_t                   indices[],                      /* O    Quantization indices [ order ]              */
    const int16_t            x_Q10[],                        /* I    Input [ order ]                             */
    const int16_t            w_Q5[],                         /* I    Weights [ order ]                           */
    const uint8_t            pred_coef_Q8[],                 /* I    Backward predictor coefs [ order ]          */
    const int16_t            ec_ix[],                        /* I    Indices to entropy coding tables [ order ]  */
    const uint8_t           ec_rates_Q5[],                  /* I    Rates []                                    */
    const int32_t              quant_step_size_Q16,            /* I    Quantization step size                      */
    const int16_t            inv_quant_step_size_Q6,         /* I    Inverse quantization step size              */
    const int32_t            mu_Q20,                         /* I    R/D tradeoff                                */
    const int16_t            order                           /* I    Number of input values                      */
);

/* Unpack predictor values and indices for entropy coding tables */
void silk_NLSF_unpack(
          int16_t            ec_ix[],                        /* O    Indices to entropy tables [ LPC_ORDER ]     */
          uint8_t            pred_Q8[],                      /* O    LSF predictor [ LPC_ORDER ]                 */
    const silk_NLSF_CB_struct   *psNLSF_CB,                     /* I    Codebook object                             */
    const int32_t              CB1_index                       /* I    Index of vector in first LSF codebook       */
);

/***********************/
/* NLSF vector decoder */
/***********************/
void silk_NLSF_decode(
          int16_t            *pNLSF_Q15,                     /* O    Quantized NLSF vector [ LPC_ORDER ]         */
          int8_t             *NLSFIndices,                   /* I    Codebook path vector [ LPC_ORDER + 1 ]      */
    const silk_NLSF_CB_struct   *psNLSF_CB                      /* I    Codebook object                             */
);

/****************************************************/
/* Decoder Functions                                */
/****************************************************/
int32_t silk_init_decoder(
    silk_decoder_state          *psDec                          /* I/O  Decoder state pointer                       */
);

/* Set decoder sampling rate */
int32_t silk_decoder_set_fs(
    silk_decoder_state          *psDec,                         /* I/O  Decoder state pointer                       */
    int32_t                    fs_kHz,                         /* I    Sampling frequency (kHz)                    */
    int32_t                  fs_API_Hz                       /* I    API Sampling frequency (Hz)                 */
);

/****************/
/* Decode frame */
/****************/
int32_t silk_decode_frame(
    silk_decoder_state          *psDec,                         /* I/O  Pointer to Silk decoder state               */
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    int16_t                  pOut[],                         /* O    Pointer to output speech frame              */
    int32_t                  *pN,                            /* O    Pointer to size of output frame             */
    int32_t                    lostFlag,                       /* I    0: no loss, 1 loss, 2 decode fec            */
    int32_t                    condCoding,                     /* I    The type of conditional coding to use       */
    int                         arch                            /* I    Run-time architecture                       */
);

/* Decode indices from bitstream */
void silk_decode_indices(
    silk_decoder_state          *psDec,                         /* I/O  State                                       */
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    int32_t                    FrameIndex,                     /* I    Frame number                                */
    int32_t                    decode_LBRR,                    /* I    Flag indicating LBRR data is being decoded  */
    int32_t                    condCoding                      /* I    The type of conditional coding to use       */
);

/* Decode parameters from payload */
void silk_decode_parameters(
    silk_decoder_state          *psDec,                         /* I/O  State                                       */
    silk_decoder_control        *psDecCtrl,                     /* I/O  Decoder control                             */
    int32_t                    condCoding                      /* I    The type of conditional coding to use       */
);

/* Core decoder. Performs inverse NSQ operation LTP + LPC */
void silk_decode_core(
    silk_decoder_state          *psDec,                         /* I/O  Decoder state                               */
    silk_decoder_control        *psDecCtrl,                     /* I    Decoder control                             */
    int16_t                  xq[],                           /* O    Decoded speech                              */
    const int16_t            pulses[ MAX_FRAME_LENGTH ],     /* I    Pulse signal                                */
    int                         arch                            /* I    Run-time architecture                       */
);

/* Decode quantization indices of excitation (Shell coding) */
void silk_decode_pulses(
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    int16_t                  pulses[],                       /* O    Excitation signal                           */
    const int32_t              signalType,                     /* I    Sigtype                                     */
    const int32_t              quantOffsetType,                /* I    quantOffsetType                             */
    const int32_t              frame_length                    /* I    Frame length                                */
);

/******************/
/* CNG */
/******************/

/* Reset CNG */
void silk_CNG_Reset(
    silk_decoder_state          *psDec                          /* I/O  Decoder state                               */
);

/* Updates CNG estimate, and applies the CNG when packet was lost */
void silk_CNG(
    silk_decoder_state          *psDec,                         /* I/O  Decoder state                               */
    silk_decoder_control        *psDecCtrl,                     /* I/O  Decoder control                             */
    int16_t                  frame[],                        /* I/O  Signal                                      */
    int32_t                    length                          /* I    Length of residual                          */
);

/* Encoding of various parameters */
void silk_encode_indices(
    silk_encoder_state          *psEncC,                        /* I/O  Encoder state                               */
    ec_enc                      *psRangeEnc,                    /* I/O  Compressor data structure                   */
    int32_t                    FrameIndex,                     /* I    Frame number                                */
    int32_t                    encode_LBRR,                    /* I    Flag indicating LBRR data is being encoded  */
    int32_t                    condCoding                      /* I    The type of conditional coding to use       */
);

#endif
