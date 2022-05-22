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

#ifndef SILK_MAIN_FIX_H
#define SILK_MAIN_FIX_H

#include "../SigProc_FIX.h"
#include "structs_FIX.h"
#include "../control.h"
#include "../main.h"
#include "../PLC.h"
#include "../debug.h"
#include "../../celt/entenc.h"



#define silk_encoder_state_Fxx      silk_encoder_state_FIX
#define silk_encode_do_VAD_Fxx      silk_encode_do_VAD_FIX
#define silk_encode_frame_Fxx       silk_encode_frame_FIX

#define QC  10
#define QS  13

/*********************/
/* Encoder Functions */
/*********************/

/* High-pass filter with cutoff frequency adaptation based on pitch lag statistics */
void silk_HP_variable_cutoff(
    silk_encoder_state_Fxx          state_Fxx[]                             /* I/O  Encoder states                                                              */
);

/* Encoder main function */
void silk_encode_do_VAD_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    int32_t                        activity                                /* I    Decision of Opus voice activity detector                                    */
);

/* Encoder main function */
int32_t silk_encode_frame_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    int32_t                      *pnBytesOut,                            /* O    Pointer to number of payload bytes;                                         */
    ec_enc                          *psRangeEnc,                            /* I/O  compressor data structure                                                   */
    int32_t                        condCoding,                             /* I    The type of conditional coding to use                                       */
    int32_t                        maxBits,                                /* I    If > 0: maximum number of output bits                                       */
    int32_t                        useCBR                                  /* I    Flag to force constant-bitrate operation                                    */
);

/* Initializes the Silk encoder state */
int32_t silk_init_encoder(
    silk_encoder_state_Fxx          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    int                              arch                                   /* I    Run-time architecture                                                       */
);

/* Control the Silk encoder */
int32_t silk_control_encoder(
    silk_encoder_state_Fxx          *psEnc,                                 /* I/O  Pointer to Silk encoder state                                               */
    silk_EncControlStruct           *encControl,                            /* I    Control structure                                                           */
    const int32_t                  allow_bw_switch,                        /* I    Flag to allow switching audio bandwidth                                     */
    const int32_t                  channelNb,                              /* I    Channel number                                                              */
    const int32_t                  force_fs_kHz
);

/**************************/
/* Noise shaping analysis */
/**************************/
/* Compute noise shaping coefficients and initial gain values */
void silk_noise_shape_analysis_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Encoder state FIX                                                           */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  Encoder control FIX                                                         */
    const int16_t                *pitch_res,                             /* I    LPC residual from pitch analysis                                            */
    const int16_t                *x,                                     /* I    Input signal [ frame_length + la_shape ]                                    */
    int                              arch                                   /* I    Run-time architecture                                                       */
);

/* Autocorrelations for a warped frequency axis */
void silk_warped_autocorrelation_FIX_c(
          int32_t                *corr,                                  /* O    Result [order + 1]                                                          */
          int32_t                  *scale,                                 /* O    Scaling of the correlation vector                                           */
    const int16_t                *input,                                 /* I    Input data to correlate                                                     */
    const int32_t                  warping_Q16,                            /* I    Warping coefficient                                                         */
    const int32_t                  length,                                 /* I    Length of input                                                             */
    const int32_t                  order                                   /* I    Correlation order (even)                                                    */
);

#if !defined(OVERRIDE_silk_warped_autocorrelation_FIX)
#define silk_warped_autocorrelation_FIX(corr, scale, input, warping_Q16, length, order, arch) \
        ((void)(arch), silk_warped_autocorrelation_FIX_c(corr, scale, input, warping_Q16, length, order))
#endif

/* Calculation of LTP state scaling */
void silk_LTP_scale_ctrl_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  encoder state                                                               */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  encoder control                                                             */
    int32_t                        condCoding                              /* I    The type of conditional coding to use                                       */
);

/**********************************************/
/* Prediction Analysis                        */
/**********************************************/
/* Find pitch lags */
void silk_find_pitch_lags_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  encoder state                                                               */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  encoder control                                                             */
    int16_t                      res[],                                  /* O    residual                                                                    */
    const int16_t                x[],                                    /* I    Speech signal                                                               */
    int                             arch                                    /* I    Run-time architecture                                                       */
);

/* Find LPC and LTP coefficients */
void silk_find_pred_coefs_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  encoder state                                                               */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  encoder control                                                             */
    const int16_t                res_pitch[],                            /* I    Residual from pitch analysis                                                */
    const int16_t                x[],                                    /* I    Speech signal                                                               */
    int32_t                        condCoding                              /* I    The type of conditional coding to use                                       */
);

/* LPC analysis */
void silk_find_LPC_FIX(
    silk_encoder_state              *psEncC,                                /* I/O  Encoder state                                                               */
    int16_t                      NLSF_Q15[],                             /* O    NLSFs                                                                       */
    const int16_t                x[],                                    /* I    Input signal                                                                */
    const int32_t                minInvGain_Q30                          /* I    Inverse of max prediction gain                                              */
);

/* LTP analysis */
void silk_find_LTP_FIX(
    int32_t                      XXLTP_Q17[ MAX_NB_SUBFR * LTP_ORDER * LTP_ORDER ], /* O    Correlation matrix                                               */
    int32_t                      xXLTP_Q17[ MAX_NB_SUBFR * LTP_ORDER ],  /* O    Correlation vector                                                          */
    const int16_t                r_lpc[],                                /* I    Residual signal after LPC                                                   */
    const int32_t                  lag[ MAX_NB_SUBFR ],                    /* I    LTP lags                                                                    */
    const int32_t                  subfr_length,                           /* I    Subframe length                                                             */
    const int32_t                  nb_subfr,                               /* I    Number of subframes                                                         */
    int                             arch                                    /* I    Run-time architecture                                                       */
);

void silk_LTP_analysis_filter_FIX(
    int16_t                      *LTP_res,                               /* O    LTP residual signal of length MAX_NB_SUBFR * ( pre_length + subfr_length )  */
    const int16_t                *x,                                     /* I    Pointer to input signal with at least max( pitchL ) preceding samples       */
    const int16_t                LTPCoef_Q14[ LTP_ORDER * MAX_NB_SUBFR ],/* I    LTP_ORDER LTP coefficients for each MAX_NB_SUBFR subframe                   */
    const int32_t                  pitchL[ MAX_NB_SUBFR ],                 /* I    Pitch lag, one for each subframe                                            */
    const int32_t                invGains_Q16[ MAX_NB_SUBFR ],           /* I    Inverse quantization gains, one for each subframe                           */
    const int32_t                  subfr_length,                           /* I    Length of each subframe                                                     */
    const int32_t                  nb_subfr,                               /* I    Number of subframes                                                         */
    const int32_t                  pre_length                              /* I    Length of the preceding samples starting at &x[0] for each subframe         */
);

/* Calculates residual energies of input subframes where all subframes have LPC_order   */
/* of preceding samples                                                                 */
void silk_residual_energy_FIX(
          int32_t                nrgs[ MAX_NB_SUBFR ],                   /* O    Residual energy per subframe                                                */
          int32_t                  nrgsQ[ MAX_NB_SUBFR ],                  /* O    Q value per subframe                                                        */
    const int16_t                x[],                                    /* I    Input signal                                                                */
          int16_t                a_Q12[ 2 ][ MAX_LPC_ORDER ],            /* I    AR coefs for each frame half                                                */
    const int32_t                gains[ MAX_NB_SUBFR ],                  /* I    Quantization gains                                                          */
    const int32_t                  subfr_length,                           /* I    Subframe length                                                             */
    const int32_t                  nb_subfr,                               /* I    Number of subframes                                                         */
    const int32_t                  LPC_order,                              /* I    LPC order                                                                   */
    int                             arch                                    /* I    Run-time architecture                                                       */
);

/* Residual energy: nrg = wxx - 2 * wXx * c + c' * wXX * c */
int32_t silk_residual_energy16_covar_FIX(
    const int16_t                *c,                                     /* I    Prediction vector                                                           */
    const int32_t                *wXX,                                   /* I    Correlation matrix                                                          */
    const int32_t                *wXx,                                   /* I    Correlation vector                                                          */
    int32_t                      wxx,                                    /* I    Signal energy                                                               */
    int32_t                        D,                                      /* I    Dimension                                                                   */
    int32_t                        cQ                                      /* I    Q value for c vector 0 - 15                                                 */
);

/* Processing of gains */
void silk_process_gains_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Encoder state                                                               */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  Encoder control                                                             */
    int32_t                        condCoding                              /* I    The type of conditional coding to use                                       */
);

/******************/
/* Linear Algebra */
/******************/
/* Calculates correlation matrix X'*X */
void silk_corrMatrix_FIX(
    const int16_t                *x,                                     /* I    x vector [L + order - 1] used to form data matrix X                         */
    const int32_t                  L,                                      /* I    Length of vectors                                                           */
    const int32_t                  order,                                  /* I    Max lag for correlation                                                     */
    int32_t                      *XX,                                    /* O    Pointer to X'*X correlation matrix [ order x order ]                        */
    int32_t                      *nrg,                                   /* O    Energy of x vector                                                          */
    int32_t                        *rshifts,                               /* O    Right shifts of correlations                                                */
    int                              arch                                   /* I    Run-time architecture                                                       */
);

/* Calculates correlation vector X'*t */
void silk_corrVector_FIX(
    const int16_t                *x,                                     /* I    x vector [L + order - 1] used to form data matrix X                         */
    const int16_t                *t,                                     /* I    Target vector [L]                                                           */
    const int32_t                  L,                                      /* I    Length of vectors                                                           */
    const int32_t                  order,                                  /* I    Max lag for correlation                                                     */
    int32_t                      *Xt,                                    /* O    Pointer to X'*t correlation vector [order]                                  */
    const int32_t                  rshifts,                                /* I    Right shifts of correlations                                                */
    int                             arch                                    /* I    Run-time architecture                                                       */
);

#endif /* SILK_MAIN_FIX_H */
