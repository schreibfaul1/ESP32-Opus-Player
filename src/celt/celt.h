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

#pragma once

#include "Arduino.h"
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <assert.h>
#include <string.h>
#include "../opus_decoder.h"


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

/*OPT: ec_window must be at least 32 bits, but if you have fast arithmetic on a
   larger type, you can speed up the decoder by using it here.*/
typedef uint32_t           ec_window;
typedef struct ec_ctx         ec_ctx;
typedef struct ec_ctx         ec_enc;
typedef struct ec_ctx         ec_dec;

struct ec_ctx {
    unsigned char *buf; /*Buffered input/output.*/
    uint32_t storage; /*The size of the buffer.*/
    uint32_t end_offs; /*The offset at which the last byte containing raw bits was read/written.*/
    ec_window end_window; /*Bits that will be read from/written at the end.*/
    int nend_bits; /*Number of valid bits in end_window.*/
    int nbits_total;
    uint32_t offs; /*The offset at which the next range coder byte will be read/written.*/
    uint32_t rng; /*The number of values in the current range.*/
    uint32_t val;
    uint32_t ext;
    int rem; /*A buffered input/output symbol, awaiting carry propagation.*/
    int error; /*Nonzero if an error occurred.*/
};

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

typedef struct {
    int32_t r;
    int32_t i;
}kiss_fft_cpx;

typedef struct {
   int16_t r;
   int16_t i;
}kiss_twiddle_cpx;

typedef struct arch_fft_state{
   int is_supported;
   void *priv;
} arch_fft_state;

#define MAXFACTORS 8

typedef struct kiss_fft_state{
    int nfft;
    int16_t scale;
    int scale_shift;
    int shift;
    int16_t factors[2*MAXFACTORS];
    const int16_t *bitrev;
    const kiss_twiddle_cpx *twiddles;
    arch_fft_state *arch_fft;
} kiss_fft_state;

typedef struct {
   int n;
   int maxshift;
   const kiss_fft_state *kfft[4];
   const int16_t * __restrict__ trig;
} mdct_lookup;

typedef struct {
    int size;
    const int16_t *index;
    const unsigned char *bits;
    const unsigned char *caps;
} PulseCache;

/** Mode definition (opaque)
 @brief Mode definition
 */
struct CELTMode {
    int32_t Fs;
    int overlap;

    int nbEBands;
    int effEBands;
    int16_t preemph[4];
    const int16_t *eBands; /**< Definition for each "pseudo-critical band" */

    int maxLM;
    int nbShortMdcts;
    int shortMdctSize;

    int nbAllocVectors;                /**< Number of lines in the matrix below */
    const unsigned char *allocVectors; /**< Number of bits in each band for several rates */
    const int16_t *logN;

    const int16_t *window;
    mdct_lookup mdct;
    PulseCache cache;
};

/* List of all the available modes */
#define TOTAL_MODES 1

#define SPREAD_NONE       (0)
#define SPREAD_LIGHT      (1)
#define SPREAD_NORMAL     (2)
#define SPREAD_AGGRESSIVE (3)

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define opus_likely(x)       (__builtin_expect(!!(x), 1))
#define opus_unlikely(x)     (__builtin_expect(!!(x), 0))

#define assert2(cond, message)
#define TWID_MAX 32767
#define TRIG_UPSCALE 1
#define LPC_ORDER 24
#define celt_fir(x, num, y, N, ord, arch) (celt_fir_c(x, num, y, N, ord, arch))
#define S_MUL(a,b) MULT16_32_Q15(b, a)
#define C_MUL(m,a,b)  do{ (m).r = SUB32_ovflw(S_MUL((a).r,(b).r) , S_MUL((a).i,(b).i)); \
                          (m).i = ADD32_ovflw(S_MUL((a).r,(b).i) , S_MUL((a).i,(b).r)); }while(0)

#define C_MULC(m,a,b) do{ (m).r = ADD32_ovflw(S_MUL((a).r,(b).r) , S_MUL((a).i,(b).i)); \
                          (m).i = SUB32_ovflw(S_MUL((a).i,(b).r) , S_MUL((a).r,(b).i)); }while(0)

#define C_MULBYSCALAR( c, s ) do{ (c).r =  S_MUL( (c).r , s ) ;  (c).i =  S_MUL( (c).i , s ) ; }while(0)

#define DIVSCALAR(x,k) (x) = S_MUL(  x, (TWID_MAX-((k)>>1))/(k)+1 )

#define C_FIXDIV(c,div) do {    DIVSCALAR( (c).r , div); DIVSCALAR( (c).i  , div); }while (0)

#define  C_ADD( res, a,b) do {(res).r=ADD32_ovflw((a).r,(b).r);  (res).i=ADD32_ovflw((a).i,(b).i); }while(0)

#define  C_SUB( res, a,b) do {(res).r=SUB32_ovflw((a).r,(b).r);  (res).i=SUB32_ovflw((a).i,(b).i); }while(0)
#define C_ADDTO( res , a) do {(res).r = ADD32_ovflw((res).r, (a).r);  (res).i = ADD32_ovflw((res).i,(a).i); }while(0)

#define C_SUBFROM( res , a) do {(res).r = ADD32_ovflw((res).r,(a).r);  (res).i = SUB32_ovflw((res).i,(a).i); }while(0)

#define CHECK_OVERFLOW_OP(a,op,b) /* noop */

#  define KISS_FFT_COS(phase)  floor(.5+TWID_MAX*cos (phase))
#  define KISS_FFT_SIN(phase)  floor(.5+TWID_MAX*sin (phase))
#  define HALF_OF(x) ((x)>>1)

#define  kf_cexp(x,phase) do{ (x)->r = KISS_FFT_COS(phase); (x)->i = KISS_FFT_SIN(phase); }while(0)

#define  kf_cexp2(x,phase) do{ (x)->r = TRIG_UPSCALE*celt_cos_norm((phase));\
                               (x)->i = TRIG_UPSCALE*celt_cos_norm((phase)-32768); }while(0)

#define LAPLACE_LOG_MINP (0)
#define LAPLACE_MINP (1<<LAPLACE_LOG_MINP)
#define LAPLACE_NMIN (16)
#define COMBFILTER_MAXPERIOD 1024
#define COMBFILTER_MINPERIOD 15

#define VALIDATE_CELT_DECODER(st)

#define opus_fft_alloc_arch(_st, arch) ((void)(arch), opus_fft_alloc_arch_c(_st))
#define opus_fft_free_arch(_st, arch)  ((void)(arch), opus_fft_free_arch_c(_st))
#define opus_fft(_cfg, _fin, _fout, arch) ((void)(arch), opus_fft_c(_cfg, _fin, _fout))
#define opus_ifft(_cfg, _fin, _fout, arch) ((void)(arch), opus_ifft_c(_cfg, _fin, _fout))

# define comb_filter_const(y, x, T, N, g10, g11, g12, arch) \
    ((void)(arch),comb_filter_const_c(y, x, T, N, g10, g11, g12))


#define clt_mdct_forward(_l, _in, _out, _window, _overlap, _shift, _stride, _arch) \
   clt_mdct_forward_c(_l, _in, _out, _window, _overlap, _shift, _stride, _arch)

#define clt_mdct_backward(_l, _in, _out, _window, _overlap, _shift, _stride, _arch) \
   clt_mdct_backward_c(_l, _in, _out, _window, _overlap, _shift, _stride, _arch)

#define SIG_SAT (300000000)
#define NORM_SCALING 16384
#define DB_SHIFT 10
#define EPSILON 1
#define VERY_SMALL 0
#define VERY_LARGE16 ((int16_t)32767)
#define Q15_ONE ((int16_t)32767)
#define SCALEIN(a)      (a)
#define SCALEOUT(a)     (a)

# define EC_WINDOW_SIZE ((int)sizeof(ec_window)*CHAR_BIT)
# define EC_UINT_BITS   (8)
# define BITRES 3
#define EC_MINI(_a,_b)      ((_a)+(((_b)-(_a))&-((_b)<(_a))))
#define EC_CLZ0    ((int)sizeof(unsigned)*CHAR_BIT)
#define EC_CLZ(_x) (__builtin_clz(_x))
#define EC_ILOG(_x) (EC_CLZ0-EC_CLZ(_x))

/** Multiply a 16-bit signed value by a 16-bit unsigned value. The result is a 32-bit signed value */
#define MULT16_16SU(a,b) ((int32_t)(int16_t)(a)*(int32_t)(uint16_t)(b))

/** 16x32 multiplication, followed by a 16-bit shift right. Results fits in 32 bits */
inline int32_t MULT16_32_Q16(int64_t a, int64_t b){return (int32_t) (a * b) >> 16;}

/** 16x32 multiplication, followed by a 16-bit shift right (round-to-nearest). Results fits in 32 bits */
#define MULT16_32_P16(a,b) ((int32_t)PSHR((int64_t)((int16_t)(a))*(b),16))


/** 16x32 multiplication, followed by a 15-bit shift right. Results fits in 32 bits */
#define MULT16_32_Q15(a,b) ((int32_t)SHR((int64_t)((int16_t)(a))*(b),15))

/** 32x32 multiplication, followed by a 31-bit shift right. Results fits in 32 bits */
#define MULT32_32_Q31(a,b) ((int32_t)SHR((int64_t)(a)*(int64_t)(b),31))


/** Compile-time conversion of float constant to 16-bit value */
#define QCONST16(x,bits) ((int16_t)(0.5L+(x)*(((int32_t)1)<<(bits))))

/** Compile-time conversion of float constant to 32-bit value */
#define QCONST32(x,bits) ((int32_t)(0.5L+(x)*(((int32_t)1)<<(bits))))

/** Negate a 16-bit value */
#define NEG16(x) (-(x))
/** Negate a 32-bit value */
#define NEG32(x) (-(x))

/** Change a 32-bit value into a 16-bit value. The value is assumed to fit in 16-bit, otherwise the result is undefined */
#define EXTRACT16(x) ((int16_t)(x))
/** Change a 16-bit value into a 32-bit value */
#define EXTEND32(x) ((int32_t)(x))

/** Arithmetic shift-right of a 16-bit value */
#define SHR16(a,shift) ((a) >> (shift))
/** Arithmetic shift-left of a 16-bit value */
#define SHL16(a,shift) ((int16_t)((uint16_t)(a)<<(shift)))
/** Arithmetic shift-right of a 32-bit value */
#define SHR32(a,shift) ((a) >> (shift))
/** Arithmetic shift-left of a 32-bit value */
#define SHL32(a,shift) ((int32_t)((uint32_t)(a)<<(shift)))

/** 32-bit arithmetic shift right with rounding-to-nearest instead of rounding down */
#define PSHR32(a,shift) (SHR32((a)+((EXTEND32(1)<<((shift))>>1)),shift))
/** 32-bit arithmetic shift right where the argument can be negative */
#define VSHR32(a, shift) (((shift)>0) ? SHR32(a, shift) : SHL32(a, -(shift)))

/** "RAW" macros, should not be used outside of this header file */
#define SHR(a,shift) ((a) >> (shift))
#define SHL(a,shift) SHL32(a,shift)
#define PSHR(a,shift) (SHR((a)+((EXTEND32(1)<<((shift))>>1)),shift))
#define SATURATE(x,a) (((x)>(a) ? (a) : (x)<-(a) ? -(a) : (x)))

#define SATURATE16(x) (EXTRACT16((x)>32767 ? 32767 : (x)<-32768 ? -32768 : (x)))

/** Shift by a and round-to-neareast 32-bit value. Result is a 16-bit value */
#define ROUND16(x,a) (EXTRACT16(PSHR32((x),(a))))
/** Shift by a and round-to-neareast 32-bit value. Result is a saturated 16-bit value */
#define SROUND16(x,a) EXTRACT16(SATURATE(PSHR32(x,a), 32767));

/** Divide by two */
#define HALF16(x)  (SHR16(x,1))
#define HALF32(x)  (SHR32(x,1))

/** Add two 16-bit values */
#define ADD16(a,b) ((int16_t)((int16_t)(a)+(int16_t)(b)))
/** Subtract two 16-bit values */
#define SUB16(a,b) ((int16_t)(a)-(int16_t)(b))
/** Add two 32-bit values */
#define ADD32(a,b) ((int32_t)(a)+(int32_t)(b))
/** Subtract two 32-bit values */
#define SUB32(a,b) ((int32_t)(a)-(int32_t)(b))

/** Add two 32-bit values, ignore any overflows */
#define ADD32_ovflw(a,b) ((int32_t)((uint32_t)(a)+(uint32_t)(b)))
/** Subtract two 32-bit values, ignore any overflows */
#define SUB32_ovflw(a,b) ((int32_t)((uint32_t)(a)-(uint32_t)(b)))
/* Avoid MSVC warning C4146: unary minus operator applied to unsigned type */
/** Negate 32-bit value, ignore any overflows */
#define NEG32_ovflw(a) ((int32_t)(0-(uint32_t)(a)))

/** 16x16 multiplication where the result fits in 16 bits */
#define MULT16_16_16(a,b)     ((((int16_t)(a))*((int16_t)(b))))

/* (int32_t)(int16_t) gives TI compiler a hint that it's 16x16->32 multiply */
/** 16x16 multiplication where the result fits in 32 bits */
#define MULT16_16(a,b)     (((int32_t)(int16_t)(a))*((int32_t)(int16_t)(b)))

/** 16x16 multiply-add where the result fits in 32 bits */
#define MAC16_16(c,a,b) (ADD32((c),MULT16_16((a),(b))))
/** 16x32 multiply, followed by a 15-bit shift right and 32-bit add.
    b must fit in 31 bits.
    Result fits in 32 bits. */
#define MAC16_32_Q15(c,a,b) ADD32((c),ADD32(MULT16_16((a),SHR((b),15)), SHR(MULT16_16((a),((b)&0x00007fff)),15)))

/** 16x32 multiplication, followed by a 16-bit shift right and 32-bit add.
    Results fits in 32 bits */
#define MAC16_32_Q16(c,a,b) ADD32((c),ADD32(MULT16_16((a),SHR((b),16)), SHR(MULT16_16SU((a),((b)&0x0000ffff)),16)))

#define MULT16_16_Q11_32(a,b) (SHR(MULT16_16((a),(b)),11))
#define MULT16_16_Q11(a,b) (SHR(MULT16_16((a),(b)),11))
#define MULT16_16_Q13(a,b) (SHR(MULT16_16((a),(b)),13))
#define MULT16_16_Q14(a,b) (SHR(MULT16_16((a),(b)),14))
#define MULT16_16_Q15(a,b) (SHR(MULT16_16((a),(b)),15))

#define MULT16_16_P13(a,b) (SHR(ADD32(4096,MULT16_16((a),(b))),13))
#define MULT16_16_P14(a,b) (SHR(ADD32(8192,MULT16_16((a),(b))),14))
#define MULT16_16_P15(a,b) (SHR(ADD32(16384,MULT16_16((a),(b))),15))

/** Divide a 32-bit value by a 16-bit value. Result fits in 16 bits */
#define DIV32_16(a,b) ((int16_t)(((int32_t)(a))/((int16_t)(b))))

/** Divide a 32-bit value by a 32-bit value. Result fits in 32 bits */
#define DIV32(a,b) (((int32_t)(a))/((int32_t)(b)))
int32_t celt_rcp(int32_t x);
#define celt_div(a,b) MULT32_32_Q31((int32_t)(a),celt_rcp(b))
#define MAX_PERIOD 1024
#define OPUS_MOVE(dst, src, n) (memmove((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src)) ))
#define OPUS_CLEAR(dst, n) (memset((dst), 0, (n)*sizeof(*(dst))))
#define ALLOC_STEPS 6

#define celt_inner_prod(x, y, N, arch) ((void)(arch),celt_inner_prod_c(x, y, N))
#define celt_pitch_xcorr celt_pitch_xcorr_c
#define dual_inner_prod(x, y01, y02, N, xy1, xy2, arch) ((void)(arch),dual_inner_prod_c(x, y01, y02, N, xy1, xy2))
#define xcorr_kernel(x, y, sum, len, arch) ((void)(arch),xcorr_kernel_c(x, y, sum, len))


/* Multiplies two 16-bit fractional values. Bit-exactness of this macro is important */
#define FRAC_MUL16(a,b) ((16384+((int32_t)(int16_t)(a)*(int16_t)(b)))>>15)

#define VARDECL(type, var)
#define ALLOC(var, size, type) type var[size]
#define FINE_OFFSET 21
#define QTHETA_OFFSET 4
#define QTHETA_OFFSET_TWOPHASE 16
#define MAX_FINE_BITS 8
#define MAX_PSEUDO 40
#define LOG_MAX_PSEUDO 6
#define ALLOC_NONE 1
//static inline int _opus_false(void) {return 0;}
//#define OPUS_CHECK_ARRAY(ptr, len) _opus_false()
//#define OPUS_PRINT_INT(value) do{}while(0)
#define OPUS_FPRINTF (void)

#define op_pvq_search(x, iy, K, N, arch) (op_pvq_search_c(x, iy, K, N, arch))

extern const signed char tf_select_table[4][8];
extern const uint32_t SMALL_DIV_TABLE[129];
extern const unsigned char LOG2_FRAC_TABLE[24];

/* Prototypes and inlines*/

static inline int16_t SAT16(int32_t x) {
   return x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
}

static inline int opus_select_arch(void)
{
  return 0;
}

static inline uint32_t ec_range_bytes(ec_ctx *_this){
  return _this->offs;
}

static inline unsigned char *ec_get_buffer(ec_ctx *_this){
  return _this->buf;
}

static inline int ec_get_error(ec_ctx *_this){
  return _this->error;
}

static inline uint32_t celt_udiv(uint32_t n, uint32_t d) {
   assert(d>0); return n/d;

}

static inline int32_t celt_sudiv(int32_t n, int32_t d) {
   assert(d>0); return n/d;

}

static inline int16_t sig2word16(int32_t x){
   x = PSHR32(x, 12);
   x = max(x, -32768);
   x = min(x, 32767);
   return EXTRACT16(x);
}

static inline int ec_tell(ec_ctx *_this){
  return _this->nbits_total-EC_ILOG(_this->rng);
}

/* Atan approximation using a 4th order polynomial. Input is in Q15 format and normalized by pi/4. Output is in
   Q15 format */
static inline int16_t celt_atan01(int16_t x) {
    return MULT16_16_P15(
        x, ADD32(32767, MULT16_16_P15(x, ADD32(-21, MULT16_16_P15(x, ADD32(-11943, MULT16_16_P15(4936, x)))))));
}

/* atan2() approximation valid for positive input values */
static inline int16_t celt_atan2p(int16_t y, int16_t x) {
    if (y < x) {
        int32_t arg;
        arg = celt_div(SHL32(EXTEND32(y), 15), x);
        if (arg >= 32767) arg = 32767;
        return SHR16(celt_atan01(EXTRACT16(arg)), 1);
    } else {
        int32_t arg;
        arg = celt_div(SHL32(EXTEND32(x), 15), y);
        if (arg >= 32767) arg = 32767;
        return 25736 - SHR16(celt_atan01(EXTRACT16(arg)), 1);
    }
}

static inline int32_t celt_maxabs16(const int16_t *x, int len) {
    int i;
    int16_t maxval = 0;
    int16_t minval = 0;
    for (i = 0; i < len; i++) {
        maxval = max(maxval, x[i]);
        minval = min(minval, x[i]);
    }
    return max(EXTEND32(maxval), -EXTEND32(minval));
}

static inline int32_t celt_maxabs32(const int32_t *x, int len) {
    int i;
    int32_t maxval = 0;
    int32_t minval = 0;
    for (i = 0; i < len; i++) {
        maxval = max(maxval, x[i]);
        minval = min(minval, x[i]);
    }
    return max(maxval, -minval);
}

/** Integer log in base2. Undefined for zero and negative numbers */
static inline int16_t celt_ilog2(int32_t x) {
    assert(x > 0);
    return EC_ILOG(x) - 1;
}

/** Integer log in base2. Defined for zero, but not for negative numbers */
static inline int16_t celt_zlog2(int32_t x) { return x <= 0 ? 0 : celt_ilog2(x); }

/** Base-2 logarithm approximation (log2(x)). (Q14 input, Q10 output) */
static inline int16_t celt_log2(int32_t x) {
    int i;
    int16_t n, frac;
    /* -0.41509302963303146, 0.9609890551383969, -0.31836011537636605,
        0.15530808010959576, -0.08556153059057618 */
    static const int16_t C[5] = {-6801 + (1 << (13 - DB_SHIFT)), 15746, -5217, 2545, -1401};
    if (x == 0) return -32767;
    i = celt_ilog2(x);
    n = VSHR32(x, i - 15) - 32768 - 16384;
    frac = ADD16(
        C[0],
        MULT16_16_Q15(
            n, ADD16(C[1], MULT16_16_Q15(n, ADD16(C[2], MULT16_16_Q15(n, ADD16(C[3], MULT16_16_Q15(n, C[4]))))))));
    return SHL16(i - 13, DB_SHIFT) + SHR16(frac, 14 - DB_SHIFT);
}

static inline int32_t celt_exp2_frac(int16_t x) {
    int16_t frac;
    frac = SHL16(x, 4);
    return ADD16(16383,
                 MULT16_16_Q15(frac, ADD16(22804, MULT16_16_Q15(frac, ADD16(14819, MULT16_16_Q15(10204, frac))))));
}
/** Base-2 exponential approximation (2^x). (Q10 input, Q16 output) */
static inline int32_t celt_exp2(int16_t x) {
    int integer;
    int16_t frac;
    integer = SHR16(x, 10);
    if (integer > 14)
        return 0x7f000000;
    else if (integer < -15)
        return 0;
    frac = celt_exp2_frac(x - SHL16(integer, 10));
    return VSHR32(EXTEND32(frac), -integer - 2);
}

static inline void dual_inner_prod_c(const int16_t *x, const int16_t *y01, const int16_t *y02, int N, int32_t *xy1,
                                     int32_t *xy2) {
    int i;
    int32_t xy01 = 0;
    int32_t xy02 = 0;
    for (i = 0; i < N; i++) {
        xy01 = MAC16_16(xy01, x[i], y01[i]);
        xy02 = MAC16_16(xy02, x[i], y02[i]);
    }
    *xy1 = xy01;
    *xy2 = xy02;
}

/*We make sure a C version is always available for cases where the overhead of vectorization and passing around an
  arch flag aren't worth it.*/
static inline int32_t celt_inner_prod_c(const int16_t *x, const int16_t *y, int N) {
    int i;
    int32_t xy = 0;
    for (i = 0; i < N; i++) xy = MAC16_16(xy, x[i], y[i]);
    return xy;
}

static inline int get_pulses(int i){
   return i<8 ? i : (8 + (i&7)) << ((i>>3)-1);
}

static inline int bits2pulses(const CELTMode *m, int band, int LM, int bits){
   int i;
   int lo, hi;
   const unsigned char *cache;

   LM++;
   cache = m->cache.bits + m->cache.index[LM*m->nbEBands+band];

   lo = 0;
   hi = cache[0];
   bits--;
   for (i=0;i<LOG_MAX_PSEUDO;i++)
   {
      int mid = (lo+hi+1)>>1;
      /* OPT: Make sure this is implemented with a conditional move */
      if ((int)cache[mid] >= bits)
         hi = mid;
      else
         lo = mid;
   }
   if (bits- (lo == 0 ? -1 : (int)cache[lo]) <= (int)cache[hi]-bits)
      return lo;
   else
      return hi;
}

static inline int pulses2bits(const CELTMode *m, int band, int LM, int pulses){
   const unsigned char *cache;

   LM++;
   cache = m->cache.bits + m->cache.index[LM*m->nbEBands+band];
   return pulses == 0 ? 0 : cache[pulses]+1;
}

static inline  void* celt_malloc(size_t count, size_t size) {
    void *ptr = ps_malloc(count * size);
    if (ptr == NULL) {
        log_e("Memory allocation failed %i bytes\n", (int)(count * size));
    }
    return ptr;
}

static inline void celt_free(void *ptr) {
    if (ptr != NULL) {
        free(ptr);
        ptr = NULL;
    }
}

static void exp_rotation1(int16_t *X, int len, int stride, int16_t c, int16_t s);
void exp_rotation(int16_t *X, int len, int dir, int stride, int K, int spread);
static void normalise_residual(int *__restrict__ iy, int16_t *__restrict__ X, int N, int32_t Ryy, int16_t gain);
static unsigned extract_collapse_mask(int *iy, int N, int B);
int16_t op_pvq_search_c(int16_t *X, int *iy, int K, int N, int arch);
unsigned alg_quant(int16_t *X, int N, int K, int spread, int B, ec_enc *enc, int16_t gain, int resynth, int arch);
unsigned alg_unquant(int16_t *X, int N, int K, int spread, int B, ec_dec *dec, int16_t gain);
void renormalise_vector(int16_t *X, int N, int16_t gain, int arch);
int stereo_itheta(const int16_t *X, const int16_t *Y, int stereo, int N, int arch);
int resampling_factor(int32_t rate);
void comb_filter_const_c(int32_t *y, int32_t *x, int T, int N, int16_t g10, int16_t g11, int16_t g12);
void comb_filter(int32_t *y, int32_t *x, int T0, int T1, int N, int16_t g0, int16_t g1, int tapset0, int tapset1,
                 const int16_t *window, int overlap, int arch);
void init_caps(const CELTMode *m, int *cap, int LM, int C);
const char *opus_strerror(int error);
int hysteresis_decision(int16_t val, const int16_t *thresholds, const int16_t *hysteresis, int N, int prev);
uint32_t celt_lcg_rand(uint32_t seed);
int16_t bitexact_cos(int16_t x);
int bitexact_log2tan(int isin, int icos);
void compute_band_energies(const CELTMode *m, const int32_t *X, int32_t *bandE, int end, int C, int LM, int arch);
void normalise_bands(const CELTMode *m, const int32_t *__restrict__ freq, int16_t *__restrict__ X, const int32_t *bandE,
                     int end, int C, int M);
void denormalise_bands(const CELTMode *m, const int16_t *__restrict__ X, int32_t *__restrict__ freq,
                       const int16_t *bandLogE, int start, int end, int M, int downsample, int silence);
void anti_collapse(const CELTMode *m, int16_t *X_, unsigned char *collapse_masks, int LM, int C, int size, int start,
                   int end, const int16_t *logE, const int16_t *prev1logE, const int16_t *prev2logE, const int *pulses,
                   uint32_t seed, int arch);
static void compute_channel_weights(int32_t Ex, int32_t Ey, int16_t w[2]);
static void intensity_stereo(const CELTMode *m, int16_t *__restrict__ X, const int16_t *__restrict__ Y,
                             const int32_t *bandE, int bandID, int N);
static void stereo_split(int16_t *__restrict__ X, int16_t *__restrict__ Y, int N);
static void stereo_merge(int16_t *__restrict__ X, int16_t *__restrict__ Y, int16_t mid, int N, int arch);
int spreading_decision(const CELTMode *m, const int16_t *X, int *average, int last_decision, int *hf_average,
                       int *tapset_decision, int update_hf, int end, int C, int M, const int *spread_weight);
static void deinterleave_hadamard(int16_t *X, int N0, int stride, int hadamard);
static void interleave_hadamard(int16_t *X, int N0, int stride, int hadamard);
void haar1(int16_t *X, int N0, int stride);
static int compute_qn(int N, int b, int offset, int pulse_cap, int stereo);
static void compute_theta(struct band_ctx *ctx, struct split_ctx *sctx, int16_t *X, int16_t *Y, int N, int *b, int B,
                          int __B0, int LM, int stereo, int *fill);
static unsigned quant_band_n1(struct band_ctx *ctx, int16_t *X, int16_t *Y, int b,  int16_t *lowband_out);
static unsigned quant_partition(struct band_ctx *ctx, int16_t *X, int N, int b, int B, int16_t *lowband, int LM,
                                int16_t gain, int fill);
static unsigned quant_band(struct band_ctx *ctx, int16_t *X, int N, int b, int B, int16_t *lowband, int LM,
                           int16_t *lowband_out, int16_t gain, int16_t *lowband_scratch, int fill);
static unsigned quant_band_stereo(struct band_ctx *ctx, int16_t *X, int16_t *Y, int N, int b, int B, int16_t *lowband,
                                  int LM, int16_t *lowband_out, int16_t *lowband_scratch, int fill);
static void special_hybrid_folding(const CELTMode *m, int16_t *norm, int16_t *norm2, int start, int M, int dual_stereo);
void quant_all_bands(int encode, const CELTMode *m, int start, int end, int16_t *X_, int16_t *Y_,
                     unsigned char *collapse_masks, const int32_t *bandE, int *pulses, int shortBlocks, int spread,
                     int dual_stereo, int intensity, int *tf_res, int32_t total_bits, int32_t balance, ec_ctx *ec,
                     int LM, int codedBands, uint32_t *seed, int complexity, int arch, int disable_inv);
int opus_custom_decoder_get_size(const CELTMode *mode, int channels);
int celt_decoder_get_size(int channels);
int opus_custom_decoder_init(CELTDecoder *st, const CELTMode *mode, int channels);
int celt_decoder_init(CELTDecoder *st, int32_t sampling_rate, int channels);
static void deemphasis_stereo_simple(int32_t *in[], int16_t *pcm, int N, const int16_t coef0, int32_t *mem);
static void deemphasis(int32_t *in[], int16_t *pcm, int N, int C, int downsample, const int16_t *coef,
               int32_t *mem, int accum);
static void celt_synthesis(const CELTMode *mode, int16_t *X, int32_t *out_syn[], int16_t *oldBandE, int start,
                           int effEnd, int C, int CC, int isTransient, int LM, int downsample, int silence, int arch);
static void tf_decode(int start, int end, int isTransient, int *tf_res, int LM, ec_dec *dec);
static int celt_plc_pitch_search(int32_t *decode_mem[2], int C, int arch);
static void celt_decode_lost(CELTDecoder *__restrict__ st, int N, int LM);
int celt_decode_with_ec(CELTDecoder *__restrict__ st, const unsigned char *data, int len, int16_t *__restrict__ pcm,
                        int frame_size, ec_dec *dec, int accum);
int celt_decoder_ctl(CELTDecoder *__restrict__ st, int request, ...);
void _celt_lpc(int16_t *_lpc, const int32_t *ac, int p);
void celt_fir_c(const int16_t *x, const int16_t *num, int16_t *y, int N, int ord, int arch);
void celt_iir(const int32_t *_x, const int16_t *den, int32_t *_y, int N, int ord, int16_t *mem, int arch);
int _celt_autocorr(const int16_t *x, int32_t *ac, const int16_t *window, int overlap, int lag, int n, int arch);
static uint32_t icwrs(int _n, const int *_y);
void encode_pulses(const int *_y, int _n, int _k, ec_enc *_enc);
static int32_t cwrsi(int _n, int _k, uint32_t _i, int *_y);
int32_t decode_pulses(int *_y, int _n, int _k, ec_dec *_dec);
uint32_t ec_tell_frac(ec_ctx *_this);
static int ec_read_byte(ec_dec *_this);
static int ec_read_byte_from_end(ec_dec *_this);
static void ec_dec_normalize(ec_dec *_this);
void ec_dec_init(ec_dec *_this, unsigned char *_buf, uint32_t _storage);
unsigned ec_decode(ec_dec *_this, unsigned _ft);
unsigned ec_decode_bin(ec_dec *_this, unsigned _bits);
void ec_dec_update(ec_dec *_this, unsigned _fl, unsigned _fh, unsigned _ft);
int ec_dec_bit_logp(unsigned _logp);
int ec_dec_icdf(const unsigned char *_icdf, unsigned _ftb);
uint32_t ec_dec_uint(ec_dec *_this, uint32_t _ft);
uint32_t ec_dec_bits(ec_dec *_this, unsigned _bits);
static int ec_write_byte(ec_enc *_this, unsigned _value);
static int ec_write_byte_at_end(ec_enc *_this, unsigned _value);
static void ec_enc_carry_out(ec_enc *_this, int _c);
static inline void ec_enc_normalize(ec_enc *_this);
void ec_enc_init(ec_enc *_this, unsigned char *_buf, uint32_t _size);
void ec_encode(ec_enc *_this, unsigned _fl, unsigned _fh, unsigned _ft);
void ec_encode_bin(ec_enc *_this, unsigned _fl, unsigned _fh, unsigned _bits);
void ec_enc_bit_logp(ec_enc *_this, int _val, unsigned _logp);
void ec_enc_icdf(ec_enc *_this, int _s, const unsigned char *_icdf, unsigned _ftb);
void ec_enc_uint(ec_enc *_this, uint32_t _fl, uint32_t _ft);
void ec_enc_bits(ec_enc *_this, uint32_t _fl, unsigned _bits);
static void kf_bfly2(kiss_fft_cpx *Fout, int m, int N);
static void kf_bfly4(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm);
static void kf_bfly3(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm);
static void kf_bfly5(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm);
void opus_fft_impl(const kiss_fft_state *st, kiss_fft_cpx *fout);
void opus_fft_c(const kiss_fft_state *st, const kiss_fft_cpx *fin, kiss_fft_cpx *fout);
void opus_ifft_c(const kiss_fft_state *st, const kiss_fft_cpx *fin, kiss_fft_cpx *fout);
static unsigned ec_laplace_get_freq1(unsigned fs0, int decay);
void ec_laplace_encode(ec_enc *enc, int *value, unsigned fs, int decay);
int ec_laplace_decode(ec_dec *dec, unsigned fs, int decay);
unsigned isqrt32(uint32_t _val);
int32_t frac_div32(int32_t a, int32_t b);
int16_t celt_rsqrt_norm(int32_t x);
int32_t celt_sqrt(int32_t x);
int16_t celt_cos_norm(int32_t x);
int32_t celt_rcp(int32_t x);
void clt_mdct_forward_c(const mdct_lookup *l, int32_t *in, int32_t *__restrict__ out,
                        const int16_t *window, int overlap, int shift, int stride, int arch);
void clt_mdct_backward_c(const mdct_lookup *l, int32_t *in, int32_t *__restrict__ out,
                         const int16_t *__restrict__ window, int overlap, int shift, int stride, int arch);
CELTMode *opus_custom_mode_create(int32_t Fs, int frame_size, int *error);
static void find_best_pitch(int32_t *xcorr, int16_t *y, int len, int max_pitch, int *best_pitch, int yshift,
                            int32_t maxcorr);
static void celt_fir5(int16_t *x, const int16_t *num, int N);
void pitch_downsample(int32_t *__restrict__ x[], int16_t *__restrict__ x_lp, int len, int C, int arch);
static int16_t compute_pitch_gain(int32_t xy, int32_t xx, int32_t yy);
static void exp_rotation1(int16_t *X, int len, int stride, int16_t c, int16_t s);
void exp_rotation(int16_t *X, int len, int dir, int stride, int K, int spread);
static void normalise_residual(int *__restrict__ iy, int16_t *__restrict__ X, int N, int32_t Ryy, int16_t gain);
static unsigned extract_collapse_mask(int *iy, int N, int B);
int16_t op_pvq_search_c(int16_t *X, int *iy, int K, int N, int arch);
unsigned alg_quant(int16_t *X, int N, int K, int spread, int B, ec_enc *enc, int16_t gain, int resynth, int arch);
unsigned alg_unquant(int16_t *X, int N, int K, int spread, int B, ec_dec *dec, int16_t gain);
void renormalise_vector(int16_t *X, int N, int16_t gain, int arch);
int stereo_itheta(const int16_t *X, const int16_t *Y, int stereo, int N, int arch);
static void find_best_pitch(int32_t *xcorr, int16_t *y, int len, int max_pitch, int *best_pitch, int yshift,
                            int32_t maxcorr);
static void celt_fir5(int16_t *x, const int16_t *num, int N);
void pitch_downsample(int32_t *__restrict__ x[], int16_t *__restrict__ x_lp, int len, int C, int arch);
int32_t celt_pitch_xcorr_c(const int16_t *_x, const int16_t *_y, int32_t *xcorr, int len, int max_pitch, int arch);
void pitch_search(const int16_t *__restrict__ x_lp, int16_t *__restrict__ y, int len, int max_pitch, int *pitch,
                  int arch);
static int16_t compute_pitch_gain(int32_t xy, int32_t xx, int32_t yy);
int16_t remove_doubling(int16_t *x, int maxperiod, int minperiod, int N, int *T0_, int prev_period, int16_t prev_gain,
                        int arch);
static int interp_bits2pulses(const CELTMode *m, int start, int end, int skip_start, const int *bits1, const int *bits2,
                              const int *thresh, const int *cap, int32_t total, int32_t *_balance, int skip_rsv,
                              int *intensity, int intensity_rsv, int *dual_stereo, int dual_stereo_rsv, int *bits,
                              int *ebits, int *fine_priority, int C, int LM, ec_ctx *ec, int encode, int prev,
                              int signalBandwidth);
int clt_compute_allocation(const CELTMode *m, int start, int end, const int *offsets, const int *cap, int alloc_trim,
                           int *intensity, int *dual_stereo, int32_t total, int32_t *balance, int *pulses, int *ebits,
                           int *fine_priority, int C, int LM, ec_ctx *ec, int encode, int prev, int signalBandwidth);
void unquant_coarse_energy(const CELTMode *m, int start, int end, int16_t *oldEBands, int intra, ec_dec *dec, int C,
                           int LM);
void unquant_fine_energy(const CELTMode *m, int start, int end, int16_t *oldEBands, int *fine_quant, ec_dec *dec,
                         int C);
void unquant_energy_finalise(const CELTMode *m, int start, int end, int16_t *oldEBands, int *fine_quant,
                             int *fine_priority, int bits_left, ec_dec *dec, int C);
static void xcorr_kernel_c(const int16_t *x, const int16_t *y, int32_t sum[4], int len);


#ifdef __cplusplus
}
#endif

