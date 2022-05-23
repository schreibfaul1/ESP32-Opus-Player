/* Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2009 Xiph.Org Foundation
   Copyright (c) 2008 Gregory Maxwell
   Written by Jean-Marc Valin and Gregory Maxwell */
/**
  @file celt.h
  @brief Contains all the functions for encoding and decoding audio
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

#ifndef CELT_H
#define CELT_H

#include "../opus_defines.h"
#include "../opus_custom.h"
#include "entenc.h"
#include "entdec.h"
#include "arch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEAK_BANDS 19

typedef struct {
   int valid;
   float tonality;
   float tonality_slope;
   float noisiness;
   float activity;
   float music_prob;
   float music_prob_min;
   float music_prob_max;
   int   bandwidth;
   float activity_probability;
   float max_pitch_ratio;
   /* Store as Q6 char to save space. */
   unsigned char leak_boost[LEAK_BANDS];
} AnalysisInfo;

typedef struct {
   int signalType;
   int offset;
} SILKInfo;

struct band_ctx{
    int encode;
    int resynth;
    const CELTMode *m;
    int i;
    int intensity;
    int spread;
    int tf_change;
    ec_ctx *ec;
    int32_t remaining_bits;
    const int32_t *bandE;
    uint32_t seed;
    int arch;
    int theta_round;
    int disable_inv;
    int avoid_split_noise;
};

struct split_ctx{
    int inv;
    int imid;
    int iside;
    int delta;
    int itheta;
    int qalloc;
};

struct CELTDecoder {
    const CELTMode *mode;
    int overlap;
    int channels;
    int stream_channels;

    int downsample;
    int start, end;
    int signalling;
    int disable_inv;
    int arch;

    /* Everything beyond this point gets cleared on a reset */
#define DECODER_RESET_START rng

    uint32_t rng;
    int error;
    int last_pitch_index;
    int loss_count;
    int skip_plc;
    int postfilter_period;
    int postfilter_period_old;
    int16_t postfilter_gain;
    int16_t postfilter_gain_old;
    int postfilter_tapset;
    int postfilter_tapset_old;

    int32_t preemph_memD[2];

    int32_t _decode_mem[1]; /* Size = channels*(DECODE_BUFFER_SIZE+mode->overlap) */
                            /* int16_t lpc[],  Size = channels*LPC_ORDER */
                            /* int16_t oldEBands[], Size = 2*mode->nbEBands */
                            /* int16_t oldLogE[], Size = 2*mode->nbEBands */
                            /* int16_t oldLogE2[], Size = 2*mode->nbEBands */
                            /* int16_t backgroundLogE[], Size = 2*mode->nbEBands */
};

struct CELTEncoder{
    const CELTMode *mode; /**< Mode used by the encoder */
    int channels;
    int stream_channels;

    int force_intra;
    int clip;
    int disable_pf;
    int complexity;
    int upsample;
    int start, end;

    int32_t bitrate;
    int vbr;
    int signalling;
    int constrained_vbr; /* If zero, VBR can do whatever it likes with the rate */
    int loss_rate;
    int lsb_depth;
    int lfe;
    int disable_inv;
    int arch;

    /* Everything beyond this point gets cleared on a reset */
#define ENCODER_RESET_START rng

    uint32_t rng;
    int spread_decision;
    int32_t delayedIntra;
    int tonal_average;
    int lastCodedBands;
    int hf_average;
    int tapset_decision;

    int prefilter_period;
    int16_t prefilter_gain;
    int prefilter_tapset;

    int consec_transient;
    AnalysisInfo analysis;
    SILKInfo silk_info;

    int32_t preemph_memE[2];
    int32_t preemph_memD[2];

    /* VBR-related parameters */
    int32_t vbr_reservoir;
    int32_t vbr_drift;
    int32_t vbr_offset;
    int32_t vbr_count;
    int32_t overlap_max;
    int16_t stereo_saving;
    int intensity;
    int16_t *energy_mask;
    int16_t spec_avg;

    int32_t in_mem[1]; /* Size = channels*mode->overlap */
                       /* int32_t prefilter_mem[],  Size = channels*COMBFILTER_MAXPERIOD */
                       /* int16_t oldBandE[],     Size = channels*mode->nbEBands */
                       /* int16_t oldLogE[],      Size = channels*mode->nbEBands */
                       /* int16_t oldLogE2[],     Size = channels*mode->nbEBands */
                       /* int16_t energyError[],  Size = channels*mode->nbEBands */
};

#define ABS16(x) ((x) < 0 ? (-(x)) : (x))
#define ABS32(x) ((x) < 0 ? (-(x)) : (x))

#define IMUL32(a,b) ((a)*(b))

#define MIN16(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum 16-bit value.   */
#define MAX16(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum 16-bit value.   */
#define MIN32(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum 32-bit value.   */
#define MAX32(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum 32-bit value.   */
#define IMIN(a,b) ((a) < (b) ? (a) : (b))   /**< Minimum int value.   */
#define IMAX(a,b) ((a) > (b) ? (a) : (b))   /**< Maximum int value.   */
#define UADD32(a,b) ((a)+(b))
#define USUB32(a,b) ((a)-(b))

#define __celt_check_mode_ptr_ptr(ptr) ((ptr) + ((ptr) - (const CELTMode**)(ptr)))
#define __celt_check_analysis_ptr(ptr) ((ptr) + ((ptr) - (const AnalysisInfo*)(ptr)))
#define __celt_check_silkinfo_ptr(ptr) ((ptr) + ((ptr) - (const SILKInfo*)(ptr)))

/* Encoder/decoder Requests */
#define CELT_SET_PREDICTION_REQUEST    10002

/** Controls the use of interframe prediction.
    0=Independent frames
    1=Short term interframe prediction allowed
    2=Long term prediction allowed
 */
#define CELT_SET_PREDICTION(x) CELT_SET_PREDICTION_REQUEST, __opus_check_int(x)

#define CELT_SET_INPUT_CLIPPING_REQUEST    10004
#define CELT_SET_INPUT_CLIPPING(x) CELT_SET_INPUT_CLIPPING_REQUEST, __opus_check_int(x)

#define CELT_GET_AND_CLEAR_ERROR_REQUEST   10007
#define CELT_GET_AND_CLEAR_ERROR(x) CELT_GET_AND_CLEAR_ERROR_REQUEST, __opus_check_int_ptr(x)

#define CELT_SET_CHANNELS_REQUEST    10008
#define CELT_SET_CHANNELS(x) CELT_SET_CHANNELS_REQUEST, __opus_check_int(x)


/* Internal */
#define CELT_SET_START_BAND_REQUEST    10010
#define CELT_SET_START_BAND(x) CELT_SET_START_BAND_REQUEST, __opus_check_int(x)

#define CELT_SET_END_BAND_REQUEST    10012
#define CELT_SET_END_BAND(x) CELT_SET_END_BAND_REQUEST, __opus_check_int(x)

#define CELT_GET_MODE_REQUEST    10015
/** Get the CELTMode used by an encoder or decoder */
#define CELT_GET_MODE(x) CELT_GET_MODE_REQUEST, __celt_check_mode_ptr_ptr(x)

#define CELT_SET_SIGNALLING_REQUEST    10016
#define CELT_SET_SIGNALLING(x) CELT_SET_SIGNALLING_REQUEST, __opus_check_int(x)

#define CELT_SET_TONALITY_REQUEST    10018
#define CELT_SET_TONALITY(x) CELT_SET_TONALITY_REQUEST, __opus_check_int(x)
#define CELT_SET_TONALITY_SLOPE_REQUEST    10020
#define CELT_SET_TONALITY_SLOPE(x) CELT_SET_TONALITY_SLOPE_REQUEST, __opus_check_int(x)

#define CELT_SET_ANALYSIS_REQUEST    10022
#define CELT_SET_ANALYSIS(x) CELT_SET_ANALYSIS_REQUEST, __celt_check_analysis_ptr(x)

#define OPUS_SET_LFE_REQUEST    10024
#define OPUS_SET_LFE(x) OPUS_SET_LFE_REQUEST, __opus_check_int(x)

#define OPUS_SET_ENERGY_MASK_REQUEST    10026
#define OPUS_SET_ENERGY_MASK(x) OPUS_SET_ENERGY_MASK_REQUEST, __opus_check_val16_ptr(x)

#define CELT_SET_SILK_INFO_REQUEST    10028
#define CELT_SET_SILK_INFO(x) CELT_SET_SILK_INFO_REQUEST, __celt_check_silkinfo_ptr(x)

/* The maximum pitch lag to allow in the pitch-based PLC. It's possible to save CPU time in the PLC pitch search by
   making this smaller than MAX_PERIOD. The current value corresponds to a pitch of 66.67 Hz. */
#define PLC_PITCH_LAG_MAX (720)

/* The minimum pitch lag to allow in the pitch-based PLC. This corresponds to a pitch of 480 Hz. */
#define PLC_PITCH_LAG_MIN (100)

/*The number of bits to output at a time.*/
# define EC_SYM_BITS   (8)

/*The total number of bits in each of the state registers.*/
# define EC_CODE_BITS  (32)

/*The maximum symbol value.*/
# define EC_SYM_MAX    ((1U<<EC_SYM_BITS)-1)

/*Bits to shift by to move a symbol into the high-order position.*/
# define EC_CODE_SHIFT (EC_CODE_BITS-EC_SYM_BITS-1)

/*Carry bit of the high-order range symbol.*/
# define EC_CODE_TOP   (((uint32_t)1U)<<(EC_CODE_BITS-1))

/*Low-order bit of the high-order range symbol.*/
# define EC_CODE_BOT   (EC_CODE_TOP>>EC_SYM_BITS)

/*The number of bits available for the last, partial symbol in the code field.*/
# define EC_CODE_EXTRA ((EC_CODE_BITS-2)%EC_SYM_BITS+1)

#define DECODE_BUFFER_SIZE 2048

/*U(N,K) = U(K,N) := N>0?K>0?U(N-1,K)+U(N,K-1)+U(N-1,K-1):0:K>0?1:0*/
#define CELT_PVQ_U(_n, _k) (CELT_PVQ_U_ROW[IMIN(_n, _k)][IMAX(_n, _k)])
/*V(N,K) := U(N,K)+U(N,K+1) = the number of PVQ codewords for a band of size N
   with K pulses allocated to it.*/
#define CELT_PVQ_V(_n, _k) (CELT_PVQ_U(_n, _k) + CELT_PVQ_U(_n, (_k) + 1))

/* Prototypes */






int celt_encoder_get_size(int channels);
int celt_encode_with_ec(CELTEncoder * __restrict__ st, const int16_t * pcm, int frame_size, unsigned char *compressed, int nbCompressedBytes, ec_enc *enc);
int celt_encoder_init(CELTEncoder *st, int32_t sampling_rate, int channels, int arch);



/* Decoder stuff */

int celt_decoder_get_size(int channels);


int celt_decoder_init(CELTDecoder *st, int32_t sampling_rate, int channels);

int celt_decode_with_ec(CELTDecoder * __restrict__ st, const unsigned char *data,
      int len, int16_t * __restrict__ pcm, int frame_size, ec_dec *dec, int accum);


#define OPUS_CUSTOM_NOSTATIC static OPUS_INLINE


static const unsigned char trim_icdf[11] = {126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0};
/* Probs: NONE: 21.875%, LIGHT: 6.25%, NORMAL: 65.625%, AGGRESSIVE: 6.25% */
static const unsigned char spread_icdf[4] = {25, 23, 2, 0};

static const unsigned char tapset_icdf[3]={2,1,0};

#define COMBFILTER_MAXPERIOD 1024
#define COMBFILTER_MINPERIOD 15

extern const signed char tf_select_table[4][8];

#define VALIDATE_CELT_DECODER(st)

int resampling_factor(int32_t rate);

void celt_preemphasis(const int16_t * __restrict__ pcmp, int32_t * __restrict__ inp,
                        int N, int CC, int upsample, const int16_t *coef, int32_t *mem, int clip);

void comb_filter(int32_t *y, int32_t *x, int T0, int T1, int N,
      int16_t g0, int16_t g1, int tapset0, int tapset1,
      const int16_t *window, int overlap, int arch);

#ifdef NON_STATIC_COMB_FILTER_CONST_C
void comb_filter_const_c(int32_t *y, int32_t *x, int T, int N,
                         int16_t g10, int16_t g11, int16_t g12);
#endif

#ifndef OVERRIDE_COMB_FILTER_CONST
# define comb_filter_const(y, x, T, N, g10, g11, g12, arch) \
    ((void)(arch),comb_filter_const_c(y, x, T, N, g10, g11, g12))
#endif

void init_caps(const CELTMode *m,int *cap,int LM,int C);


#ifdef __cplusplus
}
#endif

#endif /* CELT_H */
