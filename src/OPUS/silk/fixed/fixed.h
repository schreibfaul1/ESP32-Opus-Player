
#pragma once

#include <stdlib.h>
#include "../silk.h"

#define SCRATCH_SIZE    22
#define SF_LENGTH_4KHZ  ( PE_SUBFR_LENGTH_MS * 4 )
#define SF_LENGTH_8KHZ  ( PE_SUBFR_LENGTH_MS * 8 )
#define MIN_LAG_4KHZ    ( PE_MIN_LAG_MS * 4 )
#define MIN_LAG_8KHZ    ( PE_MIN_LAG_MS * 8 )
#define MAX_LAG_4KHZ    ( PE_MAX_LAG_MS * 4 )
#define MAX_LAG_8KHZ    ( PE_MAX_LAG_MS * 8 - 1 )
#define CSTRIDE_4KHZ    ( MAX_LAG_4KHZ + 1 - MIN_LAG_4KHZ )
#define CSTRIDE_8KHZ    ( MAX_LAG_8KHZ + 3 - ( MIN_LAG_8KHZ - 2 ) )
#define D_COMP_MIN      ( MIN_LAG_8KHZ - 3 )
#define D_COMP_MAX      ( MAX_LAG_8KHZ + 4 )
#define D_COMP_STRIDE   ( D_COMP_MAX - D_COMP_MIN )

typedef int32_t silk_pe_stage3_vals[ PE_NB_STAGE3_LAGS ];


#define MAX_FRAME_SIZE    384 /* subfr_length * nb_subfr = ( 0.005 * 16000 + 16 ) * 4 = 384 */
#define N_BITS_HEAD_ROOM  3
#define MIN_RSHIFTS      -16
#define MAX_RSHIFTS       (32 - 25) // QA_ = 25
#define QC                10
#define QS                13

#ifdef __cplusplus
extern "C"
{
#endif

extern const int16_t silk_LTPScales_table_Q14[3];

void silk_LTP_analysis_filter_FIX(
    int16_t *LTP_res, /* O    LTP residual signal of length MAX_NB_SUBFR * ( pre_length + subfr_length )            */
    const int16_t *x, /* I    Pointer to input signal with at least max( pitchL ) preceding samples                 */
    const int16_t
        LTPCoef_Q14[LTP_ORDER * MAX_NB_SUBFR], /* I    LTP_ORDER LTP coefficients for each MAX_NB_SUBFR subframe    */
    const int32_t pitchL[MAX_NB_SUBFR],        /* I    Pitch lag, one for each subframe                             */
    const int32_t invGains_Q16[MAX_NB_SUBFR],  /* I    Inverse quantization gains, one for each subframe            */
    const int32_t subfr_length, /* I    Length of each subframe                                                     */
    const int32_t nb_subfr,     /* I    Number of subframes                                                         */
    const int32_t pre_length    /* I    Length of the preceding samples starting at &x[0] for each subframe         */
);

/* Calculates residual energies of input subframes where all subframes have LPC_order                               */
/* of preceding samples                                                                                             */
void silk_residual_energy_FIX(int32_t nrgs[MAX_NB_SUBFR],        /* O    Residual energy per subframe               */
                              int32_t nrgsQ[MAX_NB_SUBFR],       /* O    Q value per subframe                       */
                              const int16_t x[],                 /* I    Input signal                               */
                              int16_t a_Q12[2][MAX_LPC_ORDER],   /* I    AR coefs for each frame half               */
                              const int32_t gains[MAX_NB_SUBFR], /* I    Quantization gains                         */
                              const int32_t subfr_length,        /* I    Subframe length                            */
                              const int32_t nb_subfr,            /* I    Number of subframes                        */
                              const int32_t LPC_order,           /* I    LPC order                                  */
                              int arch                           /* I    Run-time architecture                      */
);

/* Residual energy: nrg = wxx - 2 * wXx * c + c' * wXX * c */
int32_t silk_residual_energy16_covar_FIX(const int16_t *c,       /* I    Prediction vector                          */
                                         const int32_t *wXX,     /* I    Correlation matrix                         */
                                         const int32_t *wXx,     /* I    Correlation vector                         */
                                         int32_t wxx,            /* I    Signal energy                              */
                                         int32_t D,              /* I    Dimension                                  */
                                         int32_t cQ              /* I    Q value for c vector 0 - 15                */
);

/******************/
/* Linear Algebra */
/******************/
/* Calculates correlation matrix X'*X */
void silk_corrMatrix_FIX(const int16_t *x,    /* I    x vector [L + order - 1] used to form data matrix X           */
                         const int32_t L,     /* I    Length of vectors                                             */
                         const int32_t order, /* I    Max lag for correlation                                       */
                         int32_t *XX,         /* O    Pointer to X'*X correlation matrix [ order x order ]          */
                         int32_t *nrg,        /* O    Energy of x vector                                            */
                         int32_t *rshifts,    /* O    Right shifts of correlations                                  */
                         int arch             /* I    Run-time architecture                                         */
);

/* Calculates correlation vector X'*t */
void silk_corrVector_FIX(const int16_t *x,      /* I    x vector [L + order - 1] used to form data matrix X         */
                         const int16_t *t,      /* I    Target vector [L]                                           */
                         const int32_t L,       /* I    Length of vectors                                           */
                         const int32_t order,   /* I    Max lag for correlation                                     */
                         int32_t *Xt,           /* O    Pointer to X'*t correlation vector [order]                  */
                         const int32_t rshifts, /* I    Right shifts of correlations                                */
                         int arch               /* I    Run-time architecture                                       */
);


/* Autocorrelations for a warped frequency axis */
void silk_warped_autocorrelation_FIX_c(int32_t *corr,             /* O    Result [order + 1]  */
                                       int32_t *scale,            /* O    Scaling of the correlation vector */
                                       const int16_t *input,      /* I    Input data to correlate  */
                                       const int32_t warping_Q16, /* I    Warping coefficient */
                                       const int32_t length,      /* I    Length of input      */
                                       const int32_t order        /* I    Correlation order (even) */
);


#define silk_warped_autocorrelation_FIX(corr, scale, input, warping_Q16, length, order, arch) \
    ((void)(arch), silk_warped_autocorrelation_FIX_c(corr, scale, input, warping_Q16, length, order))




/* LPC analysis */
void silk_find_LPC_FIX(silk_encoder_state *psEncC,  /* I/O  Encoder state */
                       int16_t NLSF_Q15[],          /* O    NLSFs */
                       const int16_t x[],           /* I    Input signal */
                       const int32_t minInvGain_Q30 /* I    Inverse of max prediction gain */
);

/* LTP analysis */
void silk_find_LTP_FIX(int32_t XXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER * LTP_ORDER], /* O    Correlation matrix */
                       int32_t xXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER],             /* O    Correlation vector */
                       const int16_t r_lpc[],                                   /* I    Residual signal after LPC */
                       const int32_t lag[MAX_NB_SUBFR],                         /* I    LTP lags */
                       const int32_t subfr_length,                              /* I    Subframe length */
                       const int32_t nb_subfr,                                  /* I    Number of subframes */
                       int arch                                                 /* I    Run-time architecture */
);


extern const int8_t silk_CB_lags_stage2[PE_MAX_NB_SUBFR][PE_NB_CBKS_STAGE2_EXT];
extern const int8_t silk_CB_lags_stage2_10_ms[PE_MAX_NB_SUBFR >> 1][PE_NB_CBKS_STAGE2_10MS];
extern const int8_t silk_CB_lags_stage3_10_ms[PE_MAX_NB_SUBFR >> 1][PE_NB_CBKS_STAGE3_10MS];
extern const int8_t silk_Lag_range_stage3_10_ms[PE_MAX_NB_SUBFR >> 1][2];
extern const int8_t silk_CB_lags_stage3[PE_MAX_NB_SUBFR][PE_NB_CBKS_STAGE3_MAX];
extern const int8_t silk_CB_lags_stage3[PE_MAX_NB_SUBFR][PE_NB_CBKS_STAGE3_MAX];
extern const int8_t silk_Lag_range_stage3[SILK_PE_MAX_COMPLEX + 1][PE_MAX_NB_SUBFR][2];
extern const int8_t silk_nb_cbk_searchs_stage3[SILK_PE_MAX_COMPLEX + 1];



void silk_apply_sine_window(int16_t px_win[], const int16_t px[], const int32_t win_type, const int32_t length);
void silk_autocorr(int32_t *results, int32_t *scale, const int16_t *inputData, const int32_t inputDataSize,
                   const int32_t correlationCount, int arch);
void silk_burg_modified_c(int32_t *res_nrg, int32_t *res_nrg_Q, int32_t A_Q16[], const int16_t x[],
                          const int32_t minInvGain_Q30, const int32_t subfr_length, const int32_t nb_subfr,
                          const int32_t D, int arch);
void silk_corrVector_FIX(const int16_t *x, const int16_t *t, const int32_t L, const int32_t order, int32_t *Xt,
                         const int32_t rshifts, int arch);
void silk_corrMatrix_FIX(const int16_t *x, const int32_t L, const int32_t order, int32_t *XX, int32_t *nrg,
                         int32_t *rshifts, int arch);
void silk_encode_do_VAD_FIX(silk_encoder_state_FIX *psEnc, int32_t activity);
int32_t silk_encode_frame_FIX(silk_encoder_state_FIX *psEnc, int32_t *pnBytesOut, ec_enc *psRangeEnc,
                              int32_t condCoding, int32_t maxBits, int32_t useCBR);

void silk_find_LPC_FIX(silk_encoder_state *psEncC, int16_t NLSF_Q15[], const int16_t x[], const int32_t minInvGain_Q30);
void silk_find_LTP_FIX(int32_t XXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER * LTP_ORDER],
                       int32_t xXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER],
                       const int16_t r_ptr[], const int32_t lag[MAX_NB_SUBFR],
                       const int32_t subfr_length, const int32_t nb_subfr, int arch);

void silk_k2a_Q16(int32_t *A_Q24, const int32_t *rc_Q16, const int32_t order);
void silk_LTP_analysis_filter_FIX(int16_t *LTP_res, const int16_t *x,
                                  const int16_t LTPCoef_Q14[LTP_ORDER * MAX_NB_SUBFR],
                                  const int32_t pitchL[MAX_NB_SUBFR], const int32_t invGains_Q16[MAX_NB_SUBFR],
                                  const int32_t subfr_length, const int32_t nb_subfr, const int32_t pre_length);

static inline int32_t warped_gain(const int32_t *coefs_Q24, int32_t lambda_Q16, int32_t order);
static inline void limit_warped_coefs(int32_t *coefs_Q24, int32_t lambda_Q16, int32_t limit_Q24, int32_t order);

int32_t silk_pitch_analysis_core(const int16_t *frame_unscaled, int32_t *pitch_out, int16_t *lagIndex,
                                 int8_t *contourIndex, int32_t *LTPCorr_Q15, int32_t  prevLag,
                                 const int32_t search_thres1_Q16, const int32_t search_thres2_Q13,
                                 const int32_t Fs_kHz, const int32_t complexity, const int32_t nb_subfr, int arch);
static void silk_P_Ana_calc_corr_st3(silk_pe_stage3_vals cross_corr_st3[], const int16_t  frame[], int32_t start_lag,
                                     int32_t sf_length, int32_t nb_subfr, int32_t complexity, int arch);
static void silk_P_Ana_calc_energy_st3(silk_pe_stage3_vals energies_st3[],const int16_t  frame[], int32_t start_lag,
                                       int32_t sf_length, int32_t nb_subfr, int32_t complexity, int arch);

void silk_regularize_correlations_FIX(int32_t *XX, int32_t *xx, int32_t noise, int32_t D);
void silk_residual_energy_FIX(int32_t nrgs[MAX_NB_SUBFR], int32_t nrgsQ[MAX_NB_SUBFR], const int16_t x[],
                              int16_t a_Q12[2][MAX_LPC_ORDER], const int32_t gains[MAX_NB_SUBFR],
                              const int32_t subfr_length, const int32_t nb_subfr, const int32_t LPC_order, int arch);
int32_t silk_residual_energy16_covar_FIX(const int16_t *c, const int32_t *wXX, const int32_t *wXx,
                                         int32_t wxx, int32_t D, int32_t cQ);
int32_t silk_schur(int16_t *rc_Q15, const int32_t *c, const int32_t order);
int32_t silk_schur64(int32_t rc_Q16[], const int32_t c[], int32_t order);
void silk_scale_copy_vector16(int16_t *data_out, const int16_t *data_in, int32_t gain_Q16, const int32_t dataSize);
void silk_scale_vector32_Q26_lshift_18(int32_t *data1, int32_t gain_Q26, int32_t dataSize);
int32_t silk_inner_prod_aligned(const int16_t *const inVec1, const int16_t *const inVec2, const int32_t len, int arch);
int64_t silk_inner_prod16_aligned_64_c(const int16_t *inVec1, const int16_t *inVec2, const int32_t len);
void silk_warped_autocorrelation_FIX_c(int32_t *corr, int32_t *scale, const int16_t *input, const int32_t warping_Q16,
                                       const int32_t length, const int32_t order);


#ifdef __cplusplus
}
#endif


