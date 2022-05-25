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

#ifndef SILK_SIGPROC_FIX_H
#define SILK_SIGPROC_FIX_H

#ifdef  __cplusplus
extern "C"
{
#endif

/*#define silk_MACRO_COUNT */          /* Used to enable WMOPS counting */

#define SILK_MAX_ORDER_LPC            24            /* max order of the LPC analysis in schur() and k2a() */

#include <string.h>                                 /* for memset(), memcpy(), memmove() */
#include "typedef.h"
#include "../celt/celt.h"
#include <stdint.h>
#include "silk.h"


/********************************************************************/
/*                    SIGNAL PROCESSING FUNCTIONS                   */
/********************************************************************/

// /*!
//  * Initialize/reset the resampler state for a given pair of input/output sampling rates
// */
// int32_t silk_resampler_init(
//     silk_resampler_state_struct *S,                 /* I/O  Resampler state                                             */
//     int32_t                  Fs_Hz_in,           /* I    Input sampling rate (Hz)                                    */
//     int32_t                  Fs_Hz_out,          /* I    Output sampling rate (Hz)                                   */
//     int32_t                    forEnc              /* I    If 1: encoder; if 0: decoder                                */
// );

// /*!
//  * Resampler: convert from one sampling rate to another
//  */
// int32_t silk_resampler(
//     silk_resampler_state_struct *S,                 /* I/O  Resampler state                                             */
//     int16_t                  out[],              /* O    Output signal                                               */
//     const int16_t            in[],               /* I    Input signal                                                */
//     int32_t                  inLen               /* I    Number of input samples                                     */
// );

// /*!
// * Downsample 2x, mediocre quality
// */
// void silk_resampler_down2(
//     int32_t                  *S,                 /* I/O  State vector [ 2 ]                                          */
//     int16_t                  *out,               /* O    Output signal [ len ]                                       */
//     const int16_t            *in,                /* I    Input signal [ floor(len/2) ]                               */
//     int32_t                  inLen               /* I    Number of input samples                                     */
// );

// /*!
//  * Downsample by a factor 2/3, low quality
// */
// void silk_resampler_down2_3(
//     int32_t                  *S,                 /* I/O  State vector [ 6 ]                                          */
//     int16_t                  *out,               /* O    Output signal [ floor(2*inLen/3) ]                          */
//     const int16_t            *in,                /* I    Input signal [ inLen ]                                      */
//     int32_t                  inLen               /* I    Number of input samples                                     */
// );

// /*!
//  * second order ARMA filter;
//  * slower than biquad() but uses more precise coefficients
//  * can handle (slowly) varying coefficients
//  */
// void silk_biquad_alt_stride1(
//     const int16_t            *in,                /* I     input signal                                               */
//     const int32_t            *B_Q28,             /* I     MA coefficients [3]                                        */
//     const int32_t            *A_Q28,             /* I     AR coefficients [2]                                        */
//     int32_t                  *S,                 /* I/O   State vector [2]                                           */
//     int16_t                  *out,               /* O     output signal                                              */
//     const int32_t            len                 /* I     signal length (must be even)                               */
// );

// void silk_biquad_alt_stride2_c(
//     const int16_t            *in,                /* I     input signal                                               */
//     const int32_t            *B_Q28,             /* I     MA coefficients [3]                                        */
//     const int32_t            *A_Q28,             /* I     AR coefficients [2]                                        */
//     int32_t                  *S,                 /* I/O   State vector [4]                                           */
//     int16_t                  *out,               /* O     output signal                                              */
//     const int32_t            len                 /* I     signal length (must be even)                               */
// );

// /* Variable order MA prediction error filter. */
// void silk_LPC_analysis_filter(
//     int16_t                  *out,               /* O    Output signal                                               */
//     const int16_t            *in,                /* I    Input signal                                                */
//     const int16_t            *B,                 /* I    MA prediction coefficients, Q12 [order]                     */
//     const int32_t            len,                /* I    Signal length                                               */
//     const int32_t            d,                  /* I    Filter order                                                */
//     int                         arch                /* I    Run-time architecture                                       */
// );

// /* Chirp (bandwidth expand) LP AR filter */
// void silk_bwexpander(
//     int16_t                  *ar,                /* I/O  AR filter to be expanded (without leading 1)                */
//     const int32_t              d,                  /* I    Length of ar                                                */
//     int32_t                  chirp_Q16           /* I    Chirp factor (typically in the range 0 to 1)                */
// );

// /* Chirp (bandwidth expand) LP AR filter */
// void silk_bwexpander_32(
//     int32_t                  *ar,                /* I/O  AR filter to be expanded (without leading 1)                */
//     const int32_t              d,                  /* I    Length of ar                                                */
//     int32_t                  chirp_Q16           /* I    Chirp factor in Q16                                         */
// );

// /* Compute inverse of LPC prediction gain, and                           */
// /* test if LPC coefficients are stable (all poles within unit circle)    */
// int32_t silk_LPC_inverse_pred_gain_c(            /* O   Returns inverse prediction gain in energy domain, Q30        */
//     const int16_t            *A_Q12,             /* I   Prediction coefficients, Q12 [order]                         */
//     const int32_t              order               /* I   Prediction order                                             */
// );

// /* Split signal in two decimated bands using first-order allpass filters */
// void silk_ana_filt_bank_1(
//     const int16_t            *in,                /* I    Input signal [N]                                            */
//     int32_t                  *S,                 /* I/O  State vector [2]                                            */
//     int16_t                  *outL,              /* O    Low band [N/2]                                              */
//     int16_t                  *outH,              /* O    High band [N/2]                                             */
//     const int32_t            N                   /* I    Number of input samples                                     */
// );

#if !defined(OVERRIDE_silk_biquad_alt_stride2)
#define silk_biquad_alt_stride2(in, B_Q28, A_Q28, S, out, len, arch) ((void)(arch), silk_biquad_alt_stride2_c(in, B_Q28, A_Q28, S, out, len))
#endif

#if !defined(OVERRIDE_silk_LPC_inverse_pred_gain)
#define silk_LPC_inverse_pred_gain(A_Q12, order, arch)     ((void)(arch), silk_LPC_inverse_pred_gain_c(A_Q12, order))
#endif

/********************************************************************/
/*                        SCALAR FUNCTIONS                          */
/********************************************************************/

/* Approximation of 128 * log2() (exact inverse of approx 2^() below) */
/* Convert input to a log scale    */
int32_t silk_lin2log(
    const int32_t            inLin               /* I  input in linear scale                                         */
);

/* Approximation of a sigmoid function */
int32_t silk_sigm_Q15(
    int32_t                    in_Q5               /* I                                                                */
);

/* Approximation of 2^() (exact inverse of approx log2() above) */
/* Convert input to a linear scale */
int32_t silk_log2lin(
    const int32_t            inLog_Q7            /* I  input on log scale                                            */
);

/* Compute number of bits to right shift the sum of squares of a vector    */
/* of int16s to make it fit in an int32                                    */
void silk_sum_sqr_shift(
    int32_t                  *energy,            /* O   Energy of x, after shifting to the right                     */
    int32_t                    *shift,             /* O   Number of bits right shift applied to energy                 */
    const int16_t            *x,                 /* I   Input vector                                                 */
    int32_t                    len                 /* I   Length of input vector                                       */
);

/* Calculates the reflection coefficients from the correlation sequence    */
/* Faster than schur64(), but much less accurate.                          */
/* uses SMLAWB(), requiring armv5E and higher.                             */
int32_t silk_schur(                              /* O    Returns residual energy                                     */
    int16_t                  *rc_Q15,            /* O    reflection coefficients [order] Q15                         */
    const int32_t            *c,                 /* I    correlations [order+1]                                      */
    const int32_t            order               /* I    prediction order                                            */
);

/* Calculates the reflection coefficients from the correlation sequence    */
/* Slower than schur(), but more accurate.                                 */
/* Uses SMULL(), available on armv4                                        */
int32_t silk_schur64(                            /* O    returns residual energy                                     */
    int32_t                  rc_Q16[],           /* O    Reflection coefficients [order] Q16                         */
    const int32_t            c[],                /* I    Correlations [order+1]                                      */
    int32_t                  order               /* I    Prediction order                                            */
);

/* Step up function, converts reflection coefficients to prediction coefficients */
void silk_k2a(
    int32_t                  *A_Q24,             /* O    Prediction coefficients [order] Q24                         */
    const int16_t            *rc_Q15,            /* I    Reflection coefficients [order] Q15                         */
    const int32_t            order               /* I    Prediction order                                            */
);

/* Step up function, converts reflection coefficients to prediction coefficients */
void silk_k2a_Q16(
    int32_t                  *A_Q24,             /* O    Prediction coefficients [order] Q24                         */
    const int32_t            *rc_Q16,            /* I    Reflection coefficients [order] Q16                         */
    const int32_t            order               /* I    Prediction order                                            */
);

/* Apply sine window to signal vector.                              */
/* Window types:                                                    */
/*    1 -> sine window from 0 to pi/2                               */
/*    2 -> sine window from pi/2 to pi                              */
/* every other sample of window is linearly interpolated, for speed */
void silk_apply_sine_window(
    int16_t                  px_win[],           /* O    Pointer to windowed signal                                  */
    const int16_t            px[],               /* I    Pointer to input signal                                     */
    const int32_t              win_type,           /* I    Selects a window type                                       */
    const int32_t              length              /* I    Window length, multiple of 4                                */
);

/* Compute autocorrelation */
void silk_autocorr(
    int32_t                  *results,           /* O    Result (length correlationCount)                            */
    int32_t                    *scale,             /* O    Scaling of the correlation vector                           */
    const int16_t            *inputData,         /* I    Input data to correlate                                     */
    const int32_t              inputDataSize,      /* I    Length of input                                             */
    const int32_t              correlationCount,   /* I    Number of correlation taps to compute                       */
    int                         arch                /* I    Run-time architecture                                       */
);

void silk_decode_pitch(
    int16_t                  lagIndex,           /* I                                                                */
    int8_t                   contourIndex,       /* O                                                                */
    int32_t                    pitch_lags[],       /* O    4 pitch values                                              */
    const int32_t              Fs_kHz,             /* I    sampling frequency (kHz)                                    */
    const int32_t              nb_subfr            /* I    number of sub frames                                        */
);

int32_t silk_pitch_analysis_core(                  /* O    Voicing estimate: 0 voiced, 1 unvoiced                      */
    const int16_t            *frame,             /* I    Signal of length PE_FRAME_LENGTH_MS*Fs_kHz                  */
    int32_t                    *pitch_out,         /* O    4 pitch lag values                                          */
    int16_t                  *lagIndex,          /* O    Lag Index                                                   */
    int8_t                   *contourIndex,      /* O    Pitch contour Index                                         */
    int32_t                    *LTPCorr_Q15,       /* I/O  Normalized correlation; input: value from previous frame    */
    int32_t                    prevLag,            /* I    Last lag of previous frame; set to zero is unvoiced         */
    const int32_t            search_thres1_Q16,  /* I    First stage threshold for lag candidates 0 - 1              */
    const int32_t              search_thres2_Q13,  /* I    Final threshold for lag candidates 0 - 1                    */
    const int32_t              Fs_kHz,             /* I    Sample frequency (kHz)                                      */
    const int32_t              complexity,         /* I    Complexity setting, 0-2, where 2 is highest                 */
    const int32_t              nb_subfr,           /* I    number of 5 ms subframes                                    */
    int                         arch                /* I    Run-time architecture                                       */
);

/* Compute Normalized Line Spectral Frequencies (NLSFs) from whitening filter coefficients      */
/* If not all roots are found, the a_Q16 coefficients are bandwidth expanded until convergence. */
void silk_A2NLSF(
    int16_t                  *NLSF,              /* O    Normalized Line Spectral Frequencies in Q15 (0..2^15-1) [d] */
    int32_t                  *a_Q16,             /* I/O  Monic whitening filter coefficients in Q16 [d]              */
    const int32_t              d                   /* I    Filter order (must be even)                                 */
);

/* compute whitening filter coefficients from normalized line spectral frequencies */
void silk_NLSF2A(
    int16_t                  *a_Q12,             /* O    monic whitening filter coefficients in Q12,  [ d ]          */
    const int16_t            *NLSF,              /* I    normalized line spectral frequencies in Q15, [ d ]          */
    const int32_t              d,                  /* I    filter order (should be even)                               */
    int                         arch                /* I    Run-time architecture                                       */
);

/* Convert int32 coefficients to int16 coefs and make sure there's no wrap-around */
void silk_LPC_fit(
    int16_t                  *a_QOUT,            /* O    Output signal                                               */
    int32_t                  *a_QIN,             /* I/O  Input signal                                                */
    const int32_t              QOUT,               /* I    Input Q domain                                              */
    const int32_t              QIN,                /* I    Input Q domain                                              */
    const int32_t              d                   /* I    Filter order                                                */
);

void silk_insertion_sort_increasing(
    int32_t                  *a,                 /* I/O   Unsorted / Sorted vector                                   */
    int32_t                    *idx,               /* O     Index vector for the sorted elements                       */
    const int32_t              L,                  /* I     Vector length                                              */
    const int32_t              K                   /* I     Number of correctly sorted positions                       */
);

void silk_insertion_sort_decreasing_int16(
    int16_t                  *a,                 /* I/O   Unsorted / Sorted vector                                   */
    int32_t                    *idx,               /* O     Index vector for the sorted elements                       */
    const int32_t              L,                  /* I     Vector length                                              */
    const int32_t              K                   /* I     Number of correctly sorted positions                       */
);

void silk_insertion_sort_increasing_all_values_int16(
     int16_t                 *a,                 /* I/O   Unsorted / Sorted vector                                   */
     const int32_t             L                   /* I     Vector length                                              */
);

/* NLSF stabilizer, for a single input data vector */
void silk_NLSF_stabilize(
          int16_t            *NLSF_Q15,          /* I/O   Unstable/stabilized normalized LSF vector in Q15 [L]       */
    const int16_t            *NDeltaMin_Q15,     /* I     Min distance vector, NDeltaMin_Q15[L] must be >= 1 [L+1]   */
    const int32_t              L                   /* I     Number of NLSF parameters in the input vector              */
);

/* Laroia low complexity NLSF weights */
void silk_NLSF_VQ_weights_laroia(
    int16_t                  *pNLSFW_Q_OUT,      /* O     Pointer to input vector weights [D]                        */
    const int16_t            *pNLSF_Q15,         /* I     Pointer to input vector         [D]                        */
    const int32_t              D                   /* I     Input vector dimension (even)                              */
);

/* Compute reflection coefficients from input signal */
void silk_burg_modified_c(
    int32_t                  *res_nrg,           /* O    Residual energy                                             */
    int32_t                  *res_nrg_Q,         /* O    Residual energy Q value                                     */
    int32_t                  A_Q16[],            /* O    Prediction coefficients (length order)                      */
    const int16_t            x[],                /* I    Input signal, length: nb_subfr * ( D + subfr_length )       */
    const int32_t            minInvGain_Q30,     /* I    Inverse of max prediction gain                              */
    const int32_t            subfr_length,       /* I    Input signal subframe length (incl. D preceding samples)    */
    const int32_t            nb_subfr,           /* I    Number of subframes stacked in x                            */
    const int32_t            D,                  /* I    Order                                                       */
    int                      arch                /* I    Run-time architecture                                       */
);

/* Copy and multiply a vector by a constant */
void silk_scale_copy_vector16(
    int16_t                  *data_out,
    const int16_t            *data_in,
    int32_t                  gain_Q16,           /* I    Gain in Q16                                                 */
    const int32_t              dataSize            /* I    Length                                                      */
);

/* Some for the LTP related function requires Q26 to work.*/
void silk_scale_vector32_Q26_lshift_18(
    int32_t                  *data1,             /* I/O  Q0/Q18                                                      */
    int32_t                  gain_Q26,           /* I    Q26                                                         */
    int32_t                    dataSize            /* I    length                                                      */
);

/********************************************************************/
/*                        INLINE ARM MATH                           */
/********************************************************************/

/*    return sum( inVec1[i] * inVec2[i] ) */

int32_t silk_inner_prod_aligned(
    const int16_t *const     inVec1,             /*    I input vector 1                                              */
    const int16_t *const     inVec2,             /*    I input vector 2                                              */
    const int32_t              len,                /*    I vector lengths                                              */
    int                         arch                /*    I Run-time architecture                                       */
);


int32_t silk_inner_prod_aligned_scale(
    const int16_t *const     inVec1,             /*    I input vector 1                                              */
    const int16_t *const     inVec2,             /*    I input vector 2                                              */
    const int32_t              scale,              /*    I number of bits to shift                                     */
    const int32_t              len                 /*    I vector lengths                                              */
);

int64_t silk_inner_prod16_aligned_64_c(
    const int16_t            *inVec1,            /*    I input vector 1                                              */
    const int16_t            *inVec2,            /*    I input vector 2                                              */
    const int32_t              len                 /*    I vector lengths                                              */
);

/********************************************************************/
/*                                MACROS                            */
/********************************************************************/




#define silk_sign(a)                        ((a) > 0 ? 1 : ( (a) < 0 ? -1 : 0 ))

/* PSEUDO-RANDOM GENERATOR                                                          */
/* Make sure to store the result as the seed for the next call (also in between     */
/* frames), otherwise result won't be random at all. When only using some of the    */
/* bits, take the most significant bits by right-shifting.                          */
#define RAND_MULTIPLIER                     196314165
#define RAND_INCREMENT                      907633515
#define silk_RAND(seed)                     (silk_MLA_ovflw((RAND_INCREMENT), (seed), (RAND_MULTIPLIER)))

/*  Add some multiplication functions that can be easily mapped to ARM. */

/*    silk_SMMUL: Signed top word multiply.
          ARMv6        2 instruction cycles.
          ARMv3M+      3 instruction cycles. use SMULL and ignore LSB registers.(except xM)*/
/*#define silk_SMMUL(a32, b32)                (int32_t)silk_RSHIFT(silk_SMLAL(silk_SMULWB((a32), (b32)), (a32), silk_RSHIFT_ROUND((b32), 16)), 16)*/
/* the following seems faster on x86 */

//f





#ifdef  __cplusplus
}
#endif

#endif /* SILK_SIGPROC_FIX_H */
